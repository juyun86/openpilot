import os
import time
from dataclasses import dataclass
from dataclasses import replace
from enum import StrEnum

from opendbc.car.structs import car
from opendbc.sunnypilot.car.tesla.ars408.shadow import ARS408PerceptionState, ARS408ShadowPipeline, SHADOW_TRANSIENT_PRODUCER_FAILED
from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP, TeslaSafetyFlagsSP
from openpilot.cereal import custom, log, messaging
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.car.ars408_state import ARS408StatePublisher


MAX_BATCH_EVENTS = 256
MAX_BATCH_FRAMES = 4096
# 4096 classic-CAN frames serialize well below 256 KiB, even with Cap'n Proto
# framing overhead. The 2 MiB aggregate cap permits eight maximum-size events
# while bounding allocations before any Event decode.
MAX_RAW_EVENT_BYTES = 256 * 1024
MAX_BATCH_RAW_BYTES = 2 * 1024 * 1024
MAX_BATCH_SOURCE_SPAN_NS = 250_000_000
MAX_BATCH_PROCESS_NS = 25_000_000
CORE_BARRIER_MIN_INTERVAL_NS = 1_000_000_000 // 14 + 1
DIAGNOSTICS_MIN_INTERVAL_NS = 250_000_000
FAULT_BACKOFF_S = 0.1
IDLE_RECHECK_S = 0.25
REPLAY = "REPLAY" in os.environ


def boottime_ns() -> int:
  """Use the CAN Event.logMonoTime clock domain on target Linux.

  Windows lacks CLOCK_BOOTTIME and uses monotonic_ns only for tests/development.
  A target Linux clock mismatch is fatal and must fail closed.
  """
  clock_id = getattr(time, "CLOCK_BOOTTIME", None)
  if clock_id is None:
    if os.name == "nt":
      return time.monotonic_ns()
    raise RuntimeError("CLOCK_BOOTTIME is required for live ARS408 CAN timestamps")
  return time.clock_gettime_ns(clock_id)


class PublishResult(StrEnum):
  SENT = "sent"
  INTENTIONALLY_SUPPRESSED = "intentionally_suppressed"
  FAILED = "failed"


class BatchResult(StrEnum):
  HEALTHY = "healthy"
  FAULTED = "faulted"
  NO_PROGRESS = "no_progress"


@dataclass(frozen=True, slots=True)
class LiveARS408Config:
  car_params: car.CarParams
  car_params_sp: custom.CarParamsSP


def _ars408_flags_valid(cp: car.CarParams, cp_sp: custom.CarParamsSP) -> bool:
  return bool(
    cp.brand == "tesla" and not cp.notCar and not cp.radarUnavailable and
    int(cp_sp.flags) & int(TeslaFlagsSP.ARS408_RADAR) and
    int(cp_sp.safetyParam) & int(TeslaSafetyFlagsSP.ARS408_RADAR)
  )


def read_live_ars408_config(params: Params) -> LiveARS408Config | None:
  """Read only current onroad params. Cache/Persistent values are intentionally forbidden."""
  try:
    # Params registers IsOffroad (not IsOnroad). Only the exact live onroad
    # encoding is accepted; missing, offroad, or malformed values fail closed.
    if params.get("IsOffroad") != b"0":
      return None
    cp_raw = params.get("CarParams")
    cp_sp_raw = params.get("CarParamsSP")
  except Exception:
    return None
  if not cp_raw or not cp_sp_raw:
    return None
  try:
    cp = messaging.log_from_bytes(cp_raw, car.CarParams)
    cp_sp = messaging.log_from_bytes(cp_sp_raw, custom.CarParamsSP)
    if not _ars408_flags_valid(cp, cp_sp):
      return None
    return LiveARS408Config(cp, cp_sp)
  except Exception:
    return None


def _get_or_create_can_socket(can_sock, socket_factory=messaging.sub_sock):
  if can_sock is None:
    return socket_factory("can", timeout=100, conflate=False)
  return can_sock


def can_capnp_to_list(raw_events: list[bytes]) -> list[tuple[int, list[tuple[int, bytes, int]]]]:
  """Bounded single-pass decoder that preserves Event validity and union checks."""
  packets = []
  for raw in raw_events:
    with log.Event.from_bytes(raw) as event:
      if not event.valid or event.which() != "can" or int(event.logMonoTime) <= 0:
        raise ValueError("invalid raw CAN Event envelope")
      frames = [(int(frame.address), bytes(frame.dat), int(frame.src)) for frame in event.can]
      packets.append((int(event.logMonoTime), frames))
  return packets


def validate_batch(can_packets: list[tuple[int, list[tuple[int, bytes, int]]]], event_count: int) -> bool:
  if event_count <= 0 or event_count > MAX_BATCH_EVENTS:
    return False
  frame_count = sum(len(frames) for _, frames in can_packets)
  if frame_count <= 0 or frame_count > MAX_BATCH_FRAMES:
    return False
  timestamps = [int(timestamp) for timestamp, _ in can_packets]
  if any(timestamp <= 0 for timestamp in timestamps):
    return False
  return max(timestamps) - min(timestamps) <= MAX_BATCH_SOURCE_SPAN_NS


def validate_raw_event_sizes(raw_events: list[bytes]) -> bool:
  total_bytes = 0
  for raw in raw_events:
    raw_size = len(raw)
    if raw_size <= 0 or raw_size > MAX_RAW_EVENT_BYTES:
      return False
    total_bytes += raw_size
    if total_bytes > MAX_BATCH_RAW_BYTES:
      return False
  return True


class ARS408ShadowWorker:
  def __init__(self, pm: messaging.PubMaster, *, clock=boottime_ns) -> None:
    self.pm = pm
    self.clock = clock
    self.pipeline = ARS408ShadowPipeline(clock=clock, source_clock_mode="live")
    self.publisher = ARS408StatePublisher()
    self.barrier_pending = True
    self.drain_required = False
    self.last_core_result = PublishResult.INTENTIONALLY_SUPPRESSED
    self.last_diagnostics_result = PublishResult.INTENTIONALLY_SUPPRESSED
    self._last_core_send_host_ns: int | None = None
    self._last_diagnostics_host_ns: int | None = None
    self._pending_failure_bits = 0
    self._source_fence_ns = 0

  def _publish_core(self, perception_state: ARS408PerceptionState, *, require_barrier: bool = False) -> PublishResult:
    host_now_ns = self.clock()
    if (self._last_core_send_host_ns is not None and
        host_now_ns - self._last_core_send_host_ns < CORE_BARRIER_MIN_INTERVAL_NS):
      return PublishResult.INTENTIONALLY_SUPPRESSED
    now_ns = self._publication_now_ns(perception_state)
    try:
      built = self.publisher.build(perception_state, now_ns)
    except Exception:
      return PublishResult.FAILED
    if built is None:
      return PublishResult.INTENTIONALLY_SUPPRESSED
    state, valid = built
    if require_barrier and not (
      perception_state.producer_epoch != 0 and state.producerEpoch == perception_state.producer_epoch and
      not valid and state.cycleStatus == "invalid" and state.health == "unavailable" and
      state.forwardPresenceState == "unknown" and state.targetCount == 0 and len(state.targets) == 0
    ):
      return PublishResult.FAILED
    try:
      event = messaging.new_message("ars408StateSP")
      event.valid = valid
      event.ars408StateSP = state
      self.pm.send("ars408StateSP", event)
    except Exception:
      return PublishResult.FAILED
    self._last_core_send_host_ns = self.clock()
    return PublishResult.SENT

  def _publish_diagnostics(self, perception_state: ARS408PerceptionState) -> PublishResult:
    host_now_ns = self.clock()
    now_ns = self._publication_now_ns(perception_state)
    try:
      prepared = self.publisher.prepare_diagnostics(perception_state, now_ns)
    except Exception:
      return PublishResult.FAILED
    if prepared.result is None:
      self.publisher.commit_diagnostics(prepared, sent=False)
      return PublishResult.INTENTIONALLY_SUPPRESSED
    if (self._last_diagnostics_host_ns is not None and
        host_now_ns - self._last_diagnostics_host_ns < DIAGNOSTICS_MIN_INTERVAL_NS):
      self.publisher.commit_diagnostics(prepared, sent=False)
      return PublishResult.INTENTIONALLY_SUPPRESSED
    state, valid = prepared.result
    try:
      event = messaging.new_message("ars408DiagnosticsSP")
      event.valid = valid
      event.ars408DiagnosticsSP = state
      self.pm.send("ars408DiagnosticsSP", event)
    except Exception:
      self.publisher.commit_diagnostics(prepared, sent=False)
      return PublishResult.FAILED
    self.publisher.commit_diagnostics(prepared, sent=True)
    self._last_diagnostics_host_ns = self.clock()
    return PublishResult.SENT

  def _publication_now_ns(self, perception_state: ARS408PerceptionState) -> int:
    return self.clock()

  def _with_pending_failure(self, state: ARS408PerceptionState) -> ARS408PerceptionState:
    bits = int(state.transient_failure_bits) | self._pending_failure_bits
    return replace(state, transient_failure_bits=bits, producer_fault=bool(state.producer_fault or bits))

  def fault(self, failure_bits: int = SHADOW_TRANSIENT_PRODUCER_FAILED) -> ARS408PerceptionState:
    self.barrier_pending = True
    self.last_core_result = PublishResult.FAILED
    self._pending_failure_bits |= int(failure_bits) | SHADOW_TRANSIENT_PRODUCER_FAILED
    return self._with_pending_failure(
      self.pipeline.hard_reset(self._pending_failure_bits, host_now_ns=self.clock()),
    )

  def try_barrier(self, state: ARS408PerceptionState | None = None) -> bool:
    if state is None:
      state = self.pipeline.state(self.clock())
    if state.snapshot is not None:
      state = self.fault()
    state = self._with_pending_failure(state)
    barrier_state = replace(state, producer_fault=False)
    result = self._publish_core(barrier_state, require_barrier=True)
    self.last_core_result = result
    if result == PublishResult.FAILED:
      self.fault()
      return False
    if result == PublishResult.INTENTIONALLY_SUPPRESSED:
      return False
    barrier_complete_host_ns = self._last_core_send_host_ns
    if barrier_complete_host_ns is None:
      self.fault()
      return False
    diagnostics_result = self._publish_diagnostics(state)
    self.last_diagnostics_result = diagnostics_result
    if diagnostics_result == PublishResult.FAILED:
      self.fault()
      return False
    if diagnostics_result == PublishResult.SENT:
      self._pending_failure_bits = 0
    self._source_fence_ns = max(
      self._source_fence_ns, barrier_complete_host_ns, int(state.shadow_now_source_ns),
    )
    self.barrier_pending = False
    self.drain_required = True
    return True

  def handle_batch(self, raw_events: list[bytes]) -> BatchResult:
    if self.drain_required:
      return BatchResult.FAULTED
    if self.barrier_pending:
      self.try_barrier()
      return BatchResult.FAULTED
    started_ns = self.clock()
    epoch_before = self.pipeline.producer_epoch
    fault_count_before = self.pipeline.producer_fault_count
    try:
      if not raw_events or len(raw_events) > MAX_BATCH_EVENTS:
        state = self.fault()
        self.try_barrier(state)
        return BatchResult.FAULTED
      if not validate_raw_event_sizes(raw_events):
        state = self.fault()
        self.try_barrier(state)
        return BatchResult.FAULTED
      can_packets = can_capnp_to_list(raw_events)
      if not validate_batch(can_packets, len(raw_events)):
        state = self.fault()
      else:
        source_times = [int(timestamp) for timestamp, _frames in can_packets]
        if min(source_times) <= self._source_fence_ns:
          state = self.fault(SHADOW_TRANSIENT_PRODUCER_FAILED)
        else:
          state = self.pipeline.process_batch(can_packets, host_now_ns=self.clock())
          if self.clock() - started_ns > MAX_BATCH_PROCESS_NS:
            state = self.fault()
      new_fault = self.pipeline.producer_fault_count > fault_count_before
      epoch_changed = state.producer_epoch != epoch_before
      if new_fault or epoch_changed:
        self._pending_failure_bits |= int(state.transient_failure_bits)
        self.barrier_pending = True
        barrier_sent = self.try_barrier(state)
        if not barrier_sent:
          return BatchResult.FAULTED
        return BatchResult.FAULTED if new_fault else BatchResult.NO_PROGRESS
      publish_state = self._with_pending_failure(state)
      diagnostics_result = self._publish_diagnostics(publish_state)
      self.last_diagnostics_result = diagnostics_result
      if diagnostics_result == PublishResult.FAILED:
        self.fault()
        return BatchResult.FAULTED
      if diagnostics_result == PublishResult.SENT:
        self._pending_failure_bits = 0
      core_result = self._publish_core(self._with_pending_failure(state))
      self.last_core_result = core_result
      if core_result == PublishResult.FAILED:
        self.fault()
        return BatchResult.FAULTED
      return BatchResult.NO_PROGRESS if state.producer_fault else BatchResult.HEALTHY
    except Exception:
      # Do not retry from this drain. The main loop observes False, backs off,
      # and will require a successful empty core barrier before reading CAN.
      self.fault()
      return BatchResult.FAULTED

  def try_teardown_barrier(self) -> bool:
    """Best effort only; future consumers must still enforce local TTL."""
    return self.try_barrier(self.fault())

  def mark_drain_complete(self) -> None:
    if self.barrier_pending:
      raise RuntimeError("cannot complete ARS408 drain before its barrier")
    self.drain_required = False


def _safe_log_exception(message: str) -> None:
  try:
    cloudlog.exception(message)
  except Exception:
    pass


def _teardown_worker(worker: ARS408ShadowWorker | None) -> None:
  """Best effort only; process loss still requires consumer-side TTL."""
  if worker is None:
    return
  try:
    worker.try_teardown_barrier()
  except Exception:
    pass


def _next_fault_count(current: int, result: BatchResult) -> int:
  if result == BatchResult.HEALTHY:
    return 0
  if result == BatchResult.FAULTED:
    return current + 1
  return current


def main() -> None:
  params = Params()
  pm = None
  worker: ARS408ShadowWorker | None = None
  can_sock = None
  fault_count = 0

  while True:
    if pm is None:
      try:
        pm = messaging.PubMaster(["ars408StateSP", "ars408DiagnosticsSP"])
      except Exception:
        _safe_log_exception("ars408shadowd PubMaster construction failed")
        time.sleep(min(1.0, FAULT_BACKOFF_S * max(fault_count, 1)))
        fault_count += 1
        continue
    if read_live_ars408_config(params) is None:
      _teardown_worker(worker)
      worker = None
      if can_sock is not None:
        try:
          messaging.drain_sock_raw(can_sock)
        except Exception:
          pass
      time.sleep(IDLE_RECHECK_S)
      continue
    if REPLAY:
      # Source/replay transport has no proven cross-socket watermark. Do not
      # subscribe or publish until a non-disruptive transport fence is proven.
      _teardown_worker(worker)
      worker = None
      time.sleep(IDLE_RECHECK_S)
      continue
    if worker is None:
      try:
        if can_sock is None:
          # msgq reader slots are process-lifetime resources. Keep exactly one
          # non-conflated CAN subscriber instead of rotating it on faults.
          can_sock = _get_or_create_can_socket(can_sock)
        worker = ARS408ShadowWorker(pm)
      except Exception:
        worker = None
        _safe_log_exception("ars408shadowd worker/socket construction failed")
        fault_count += 1
        time.sleep(min(1.0, FAULT_BACKOFF_S * fault_count))
        continue
      if not worker.try_barrier():
        fault_count += 1
        time.sleep(min(1.0, FAULT_BACKOFF_S * fault_count))
        continue
      try:
        messaging.drain_sock_raw(can_sock)
        worker.mark_drain_complete()
      except Exception:
        worker.fault()
        fault_count += 1
        time.sleep(min(1.0, FAULT_BACKOFF_S * fault_count))
        continue

    try:
      if worker.barrier_pending or worker.drain_required:
        if worker.barrier_pending and not worker.try_barrier():
          fault_count += 1
          time.sleep(min(1.0, FAULT_BACKOFF_S * fault_count))
          continue
        try:
          messaging.drain_sock_raw(can_sock)
          worker.mark_drain_complete()
        except Exception:
          worker.fault()
          fault_count += 1
          time.sleep(min(1.0, FAULT_BACKOFF_S * fault_count))
          continue
        continue
      raw_events = messaging.drain_sock_raw(can_sock, wait_for_one=True)
      if read_live_ars408_config(params) is None:
        _teardown_worker(worker)
        worker = None
        continue
      batch_result = worker.handle_batch(raw_events)
      fault_count = _next_fault_count(fault_count, batch_result)
      if batch_result == BatchResult.FAULTED:
        time.sleep(min(1.0, FAULT_BACKOFF_S * fault_count))
    except Exception:
      fault_count += 1
      if fault_count == 1 or fault_count % 10 == 0:
        _safe_log_exception("ars408shadowd loop failure")
      if worker is not None:
        worker.fault()
      time.sleep(min(1.0, FAULT_BACKOFF_S * fault_count))


if __name__ == "__main__":
  main()
