import ast
import importlib.util
import sys
import types
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import pytest


MSGQ_AVAILABLE = importlib.util.find_spec("msgq") is not None
if not MSGQ_AVAILABLE:
  fake_msgq = types.ModuleType("msgq")
  for name in (
    "Context", "Poller", "SubSocket", "PubSocket", "SocketEventHandle",
    "MultiplePublishersError", "IpcError",
  ):
    setattr(fake_msgq, name, type(name, (), {}))
  for name in (
    "fake_event_handle", "drain_sock_raw", "toggle_fake_events", "set_fake_prefix",
    "get_fake_prefix", "delete_fake_prefix", "wait_for_one_event", "pub_sock", "sub_sock",
  ):
    setattr(fake_msgq, name, lambda *_args, **_kwargs: None)
  fake_msgq.context = fake_msgq.Context()
  sys.modules["msgq"] = fake_msgq

from opendbc.car.structs import car
from opendbc.can import CANPacker
from opendbc.sunnypilot.car.tesla.ars408.constants import ARS408_BUS
from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP, TeslaSafetyFlagsSP
from openpilot.cereal import custom, log

if not MSGQ_AVAILABLE:
  fake_messaging = types.ModuleType("openpilot.cereal.messaging")

  def new_message(service, size=None, **kwargs):
    event = log.Event.new_message(valid=False, logMonoTime=0, **kwargs)
    if service is not None:
      event.init(service) if size is None else event.init(service, size)
    return event

  fake_messaging.new_message = new_message
  fake_messaging.log_from_bytes = lambda data, schema=log.Event: schema.from_bytes(data).__enter__()
  fake_messaging.PubMaster = object
  fake_messaging.sub_sock = lambda *_args, **_kwargs: None
  fake_messaging.drain_sock_raw = lambda *_args, **_kwargs: []
  sys.modules["openpilot.cereal.messaging"] = fake_messaging

  fake_params = types.ModuleType("openpilot.common.params")
  fake_params.Params = type("Params", (), {})
  sys.modules["openpilot.common.params"] = fake_params

  fake_swaglog = types.ModuleType("openpilot.common.swaglog")
  fake_swaglog.cloudlog = SimpleNamespace(exception=lambda *_args, **_kwargs: None)
  sys.modules["openpilot.common.swaglog"] = fake_swaglog

  fake_pandad = types.ModuleType("openpilot.selfdrive.pandad")
  fake_pandad.can_capnp_to_list = lambda _raw: []
  sys.modules["openpilot.selfdrive.pandad"] = fake_pandad

from openpilot.selfdrive.car import ars408_shadowd as daemon
messaging = daemon.messaging


class StubPubMaster:
  def __init__(self, fail_service: str | None = None, on_send=None) -> None:
    self.fail_service = fail_service
    self.on_send = on_send
    self.sent = []

  def send(self, service, event) -> None:
    if service == self.fail_service:
      raise RuntimeError(f"{service} send failed")
    if self.on_send is not None:
      self.on_send(service)
    self.sent.append((service, event))


class StubParams:
  def __init__(self, values=None) -> None:
    self.values = values or {}

  def get(self, key):
    return self.values.get(key)

  def get_bool(self, key):
    return self.values.get(key) in (True, b"1", "1")


class MutableClock:
  def __init__(self, value: int = 10_000) -> None:
    self.value = value

  def __call__(self) -> int:
    return self.value

  def advance(self, delta_ns: int) -> None:
    self.value += delta_ns


def worker(pm=None, *, times=None, clock=None) -> daemon.ARS408ShadowWorker:
  if clock is None:
    values = iter(times or [10_000] * 100)

    def clock():
      return next(values)
  return daemon.ARS408ShadowWorker(pm or StubPubMaster(), clock=clock)


def test_barrier_is_a_real_sent_empty_invalid_core_with_matching_epoch() -> None:
  pm = StubPubMaster()
  current = worker(pm)
  assert current.try_barrier()
  assert not current.barrier_pending and current.drain_required
  assert current.last_core_result == daemon.PublishResult.SENT
  core = next(event for service, event in pm.sent if service == "ars408StateSP")
  assert not core.valid
  assert core.ars408StateSP.producerEpoch == current.pipeline.producer_epoch != 0
  assert core.ars408StateSP.cycleStatus == "invalid"
  assert core.ars408StateSP.health == "unavailable"
  assert core.ars408StateSP.forwardPresenceState == "unknown"
  assert core.ars408StateSP.targetCount == 0 and len(core.ars408StateSP.targets) == 0
  diagnostics = [event for service, event in pm.sent if service == "ars408DiagnosticsSP"]
  assert current.last_diagnostics_result == daemon.PublishResult.SENT
  assert diagnostics[-1].ars408DiagnosticsSP.producerEpoch == current.pipeline.producer_epoch


@pytest.mark.parametrize("stage", ("build", "send"))
def test_failed_core_barrier_stays_pending_and_next_success_is_still_barrier(monkeypatch, stage: str) -> None:
  pm = StubPubMaster(fail_service="ars408StateSP" if stage == "send" else None)
  clock = MutableClock()
  current = worker(pm, clock=clock)
  first_epoch = current.pipeline.producer_epoch
  if stage == "build":
    monkeypatch.setattr(current.publisher, "build", lambda *_args: (_ for _ in ()).throw(RuntimeError("build")))
  assert not current.try_barrier()
  assert current.barrier_pending and current.pipeline.producer_epoch != first_epoch
  current.publisher = daemon.ARS408StatePublisher()
  pm.fail_service = None
  clock.advance(daemon.CORE_BARRIER_MIN_INTERVAL_NS)
  assert current.try_barrier()
  core = [event for service, event in pm.sent if service == "ars408StateSP"][-1]
  assert not core.valid and core.ars408StateSP.targetCount == 0
  assert core.ars408StateSP.producerEpoch == current.pipeline.producer_epoch


def test_normal_duplicate_state_is_intentionally_suppressed_not_sent() -> None:
  current = worker()
  state = current.pipeline.state(10_000)
  assert current._publish_core(state) == daemon.PublishResult.SENT
  assert current._publish_core(state) == daemon.PublishResult.INTENTIONALLY_SUPPRESSED


def test_all_core_sends_share_physical_14hz_limit_before_build(monkeypatch) -> None:
  pm = StubPubMaster()
  clock = MutableClock(1_000_000_000)
  current = worker(pm, clock=clock)
  state = current.pipeline.state(clock())
  assert current._publish_core(state) == daemon.PublishResult.SENT
  monkeypatch.setattr(current.publisher, "build", lambda *_args: (_ for _ in ()).throw(AssertionError("built too early")))
  assert current._publish_core(state) == daemon.PublishResult.INTENTIONALLY_SUPPRESSED
  faulted = current.fault()
  assert not current.try_barrier(faulted)
  assert sum(service == "ars408StateSP" for service, _event in pm.sent) == 1
  clock.advance(daemon.CORE_BARRIER_MIN_INTERVAL_NS)
  current.publisher = daemon.ARS408StatePublisher()
  assert current.try_barrier(faulted)


def test_barrier_then_normal_core_is_limited_by_same_physical_clock(monkeypatch) -> None:
  clock = MutableClock(1_000_000_000)
  current = worker(clock=clock)
  assert current.try_barrier()
  current.mark_drain_complete()
  calls = []
  monkeypatch.setattr(current.publisher, "build", lambda *_args: calls.append("build") or (_ for _ in ()).throw(AssertionError))
  assert current._publish_core(current.pipeline.state(clock())) == daemon.PublishResult.INTENTIONALLY_SUPPRESSED
  assert calls == []


def test_rate_limited_recovery_epoch_barrier_preserves_warm_pipeline_until_retry(monkeypatch) -> None:
  clock = MutableClock(1_000_000_000)
  current = worker(clock=clock)
  assert current.try_barrier()
  current.mark_drain_complete()
  old_epoch = current.pipeline.producer_epoch
  old_fault_count = current.pipeline.producer_fault_count
  warm_epoch = old_epoch + 1

  def warm_pipeline(*_args, **_kwargs):
    current.pipeline.producer_epoch = warm_epoch
    return replace(
      current.pipeline.state(clock()), producer_epoch=warm_epoch, producer_fault=True,
      transient_failure_bits=daemon.SHADOW_TRANSIENT_PRODUCER_FAILED,
    )

  clock.advance(10_000_000)
  monkeypatch.setattr(daemon, "can_capnp_to_list", lambda _raw: [(clock() + 1, [(0, b"", 1)])])
  monkeypatch.setattr(current.pipeline, "process_batch", warm_pipeline)
  assert current.handle_batch([b"warm"]) == daemon.BatchResult.FAULTED
  assert current.barrier_pending and current.pipeline.producer_epoch == warm_epoch
  assert current.pipeline.producer_fault_count == old_fault_count

  clock.advance(daemon.CORE_BARRIER_MIN_INTERVAL_NS - 10_000_000)
  assert current.try_barrier()
  assert current.pipeline.producer_epoch == warm_epoch
  assert current.pipeline.producer_fault_count == old_fault_count


def test_diagnostics_send_failure_after_normal_core_forces_new_epoch_barrier(monkeypatch) -> None:
  pm = StubPubMaster()
  current = worker(pm)
  assert current.try_barrier()
  current.mark_drain_complete()
  old_epoch = current.pipeline.producer_epoch
  state = current.pipeline.state(10_000)
  monkeypatch.setattr(daemon, "can_capnp_to_list", lambda _raw: [(10_001, [(0, b"", 1)])])
  monkeypatch.setattr(current.pipeline, "process_batch", lambda *_args, **_kwargs: state)
  monkeypatch.setattr(current, "_publish_diagnostics", lambda _state: daemon.PublishResult.FAILED)
  assert current.handle_batch([b"raw"]) == daemon.BatchResult.FAULTED
  assert current.pipeline.producer_epoch != old_epoch
  assert current.barrier_pending and not current.drain_required


@pytest.mark.parametrize("stage", ("build", "send"))
def test_normal_core_failure_latches_fault_without_processing_another_batch(monkeypatch, stage: str) -> None:
  pm = StubPubMaster()
  clock = MutableClock()
  current = worker(pm, clock=clock)
  assert current.try_barrier()
  current.mark_drain_complete()
  clock.advance(daemon.CORE_BARRIER_MIN_INTERVAL_NS)
  state = current.pipeline.state(10_000)
  monkeypatch.setattr(daemon, "can_capnp_to_list", lambda _raw: [(10_001, [(0, b"", 1)])])
  monkeypatch.setattr(current.pipeline, "process_batch", lambda *_args, **_kwargs: state)
  if stage == "build":
    monkeypatch.setattr(current.publisher, "build", lambda *_args: (_ for _ in ()).throw(RuntimeError("build")))
  else:
    current.publisher = daemon.ARS408StatePublisher()
    pm.fail_service = "ars408StateSP"
  assert current.handle_batch([b"raw"]) == daemon.BatchResult.FAULTED
  assert current.barrier_pending and current.last_core_result == daemon.PublishResult.FAILED


@pytest.mark.parametrize("stage", ("build", "send"))
def test_diagnostics_failure_forces_new_epoch_barrier(monkeypatch, stage: str) -> None:
  pm = StubPubMaster()
  clock = MutableClock()
  current = worker(pm, clock=clock)
  assert current.try_barrier()
  current.mark_drain_complete()
  clock.advance(daemon.DIAGNOSTICS_MIN_INTERVAL_NS)
  old_epoch = current.pipeline.producer_epoch
  state = current.pipeline.state(10_000)
  monkeypatch.setattr(daemon, "can_capnp_to_list", lambda _raw: [(10_001, [(0, b"", 1)])])
  monkeypatch.setattr(current.pipeline, "process_batch", lambda *_args, **_kwargs: state)
  if stage == "build":
    monkeypatch.setattr(current.publisher, "prepare_diagnostics",
                        lambda *_args: (_ for _ in ()).throw(RuntimeError("build")))
  else:
    current.publisher = daemon.ARS408StatePublisher()
    pm.fail_service = "ars408DiagnosticsSP"
  valid_core_before = sum(service == "ars408StateSP" and event.valid for service, event in pm.sent)
  assert current.handle_batch([b"raw"]) == daemon.BatchResult.FAULTED
  assert current.pipeline.producer_epoch != old_epoch and current.barrier_pending
  assert sum(service == "ars408StateSP" and event.valid for service, event in pm.sent) == valid_core_before


def test_event_limit_is_checked_before_decoder(monkeypatch) -> None:
  current = worker()
  assert current.try_barrier()
  current.mark_drain_complete()
  calls = []
  monkeypatch.setattr(daemon, "can_capnp_to_list", lambda raw: calls.append(raw) or (_ for _ in ()).throw(AssertionError))
  assert current.handle_batch([b"x"] * (daemon.MAX_BATCH_EVENTS + 1)) == daemon.BatchResult.FAULTED
  assert calls == [] and current.barrier_pending


@pytest.mark.parametrize("raw_events", [
  [b""],
  [b"x" * (daemon.MAX_RAW_EVENT_BYTES + 1)],
  [b"x" * daemon.MAX_RAW_EVENT_BYTES] * (daemon.MAX_BATCH_RAW_BYTES // daemon.MAX_RAW_EVENT_BYTES + 1),
])
def test_raw_byte_limits_are_checked_before_decoder(monkeypatch, raw_events: list[bytes]) -> None:
  current = worker()
  assert current.try_barrier()
  current.mark_drain_complete()
  calls = []
  monkeypatch.setattr(daemon, "can_capnp_to_list", lambda raw: calls.append(raw) or [])
  assert current.handle_batch(raw_events) == daemon.BatchResult.FAULTED
  assert calls == [] and current.barrier_pending


def test_raw_byte_limits_accept_exact_single_and_aggregate_boundaries_before_decode(monkeypatch) -> None:
  current = worker()
  assert current.try_barrier()
  current.mark_drain_complete()
  raw_events = [b"x" * daemon.MAX_RAW_EVENT_BYTES] * (daemon.MAX_BATCH_RAW_BYTES // daemon.MAX_RAW_EVENT_BYTES)
  calls = []
  monkeypatch.setattr(
    daemon, "can_capnp_to_list",
    lambda raw: calls.append(raw) or [(10_001 + index, [(0, b"", 1)]) for index in range(len(raw))],
  )
  state = replace(current.pipeline.state(10_100), producer_fault=False, transient_failure_bits=0)
  monkeypatch.setattr(current.pipeline, "process_batch", lambda *_args, **_kwargs: state)
  assert current.handle_batch(raw_events) == daemon.BatchResult.HEALTHY
  assert len(calls) == 1


def test_worker_rejects_unfenced_source_clock_api() -> None:
  with pytest.raises(TypeError):
    daemon.ARS408ShadowWorker(StubPubMaster(), source_clock_mode="source")


def serialized_event(service: str, *, valid: bool, source_ns: int = 10_001) -> bytes:
  event = messaging.new_message(service, 1 if service == "can" else None)
  event.valid = valid
  event.logMonoTime = source_ns
  if service == "can":
    event.can[0].address = 0x60A
    event.can[0].dat = b"\x01\x02\x03\x04\x05\x06\x07\x08"
    event.can[0].src = ARS408_BUS
  return event.to_bytes()


def test_local_raw_event_decoder_preserves_valid_union_and_timestamp_envelope() -> None:
  raw = serialized_event("can", valid=True)
  assert daemon.can_capnp_to_list([raw]) == [(
    10_001, [(0x60A, b"\x01\x02\x03\x04\x05\x06\x07\x08", ARS408_BUS)],
  )]


@pytest.mark.parametrize("raw", [
  serialized_event("can", valid=False),
  serialized_event("deviceState", valid=True),
  serialized_event("can", valid=True, source_ns=0),
  b"not-capnp",
])
def test_invalid_raw_event_envelope_faults_whole_batch_without_pipeline_call(monkeypatch, raw: bytes) -> None:
  current = worker()
  assert current.try_barrier()
  current.mark_drain_complete()
  calls = []
  monkeypatch.setattr(current.pipeline, "process_batch", lambda *_args, **_kwargs: calls.append("processed"))
  assert current.handle_batch([raw]) == daemon.BatchResult.FAULTED
  assert calls == [] and current.barrier_pending


def test_pre_barrier_socket_data_cannot_enter_new_epoch(monkeypatch) -> None:
  clock = MutableClock()
  current = worker(clock=clock)
  assert current.try_barrier()
  current.mark_drain_complete()
  calls = []
  monkeypatch.setattr(daemon, "can_capnp_to_list", lambda raw: calls.append(raw) or [(clock() + 1, [(0, b"", 1)])])
  monkeypatch.setattr(
    current.pipeline, "process_batch",
    lambda *_args, **_kwargs: current.pipeline.hard_reset(
      daemon.SHADOW_TRANSIENT_PRODUCER_FAILED, host_now_ns=clock(),
    ),
  )
  assert current.handle_batch([b"fault"]) == daemon.BatchResult.FAULTED
  assert current.barrier_pending
  assert current.handle_batch([b"arrived-before-barrier"]) == daemon.BatchResult.FAULTED
  assert calls == [[b"fault"]]
  clock.advance(daemon.CORE_BARRIER_MIN_INTERVAL_NS)
  assert current.try_barrier()
  current.mark_drain_complete()
  state = replace(current.pipeline.state(clock()), producer_fault=False, transient_failure_bits=0)
  monkeypatch.setattr(current.pipeline, "process_batch", lambda *_args, **_kwargs: state)
  assert current.handle_batch([b"arrived-after-drain"]) == daemon.BatchResult.HEALTHY
  assert calls[-1] == [b"arrived-after-drain"]


def test_live_fence_rejects_delayed_pre_barrier_source_time(monkeypatch) -> None:
  clock = MutableClock(1_000_000_000)
  current = worker(clock=clock)
  assert current.try_barrier()
  current.mark_drain_complete()
  calls = []
  monkeypatch.setattr(daemon, "can_capnp_to_list", lambda _raw: [(clock() - 1, [(0, b"", 1)])])
  monkeypatch.setattr(current.pipeline, "process_batch", lambda *_args, **_kwargs: calls.append("processed"))
  assert current.handle_batch([b"delayed"]) == daemon.BatchResult.FAULTED
  assert calls == [] and current.barrier_pending


def test_live_fence_uses_slow_core_send_completion_time(monkeypatch) -> None:
  clock = MutableClock(1_000_000_000)
  pm = StubPubMaster(on_send=lambda service: clock.advance(100_000_000) if service == "ars408StateSP" else None)
  current = worker(pm, clock=clock)
  assert current.try_barrier()
  assert current._source_fence_ns == 1_100_000_000
  current.mark_drain_complete()
  calls = []
  monkeypatch.setattr(daemon, "can_capnp_to_list", lambda _raw: [(1_050_000_000, [(0, b"", 1)])])
  monkeypatch.setattr(current.pipeline, "process_batch", lambda *_args, **_kwargs: calls.append("processed"))
  assert current.handle_batch([b"delayed-during-send"]) == daemon.BatchResult.FAULTED
  assert calls == []


def test_live_fence_uses_can_boottime_domain_not_offset_monotonic_domain(monkeypatch) -> None:
  # Model a host where CLOCK_BOOTTIME is 500 ms ahead of CLOCK_MONOTONIC.
  # A pre-barrier CAN timestamp would pass a 1.0 s monotonic fence, but must be
  # rejected against the 1.5 s boottime completion fence.
  clock = MutableClock(1_500_000_000)
  current = worker(clock=clock)
  assert current.try_barrier()
  assert current._source_fence_ns == 1_500_000_000
  current.mark_drain_complete()
  calls = []
  monkeypatch.setattr(daemon, "can_capnp_to_list", lambda _raw: [(1_250_000_000, [(0, b"", 1)])])
  monkeypatch.setattr(current.pipeline, "process_batch", lambda *_args, **_kwargs: calls.append("processed"))
  assert current.handle_batch([b"pre-barrier"]) == daemon.BatchResult.FAULTED
  assert calls == []


def test_target_linux_boottime_clock_is_used(monkeypatch) -> None:
  monkeypatch.setattr(daemon.os, "name", "posix")
  monkeypatch.setattr(daemon.time, "CLOCK_BOOTTIME", 7, raising=False)
  monkeypatch.setattr(daemon.time, "clock_gettime_ns", lambda clock_id: 123 if clock_id == 7 else 0, raising=False)
  assert daemon.boottime_ns() == 123


def test_windows_missing_boottime_uses_explicit_development_fallback(monkeypatch) -> None:
  monkeypatch.setattr(daemon.os, "name", "nt")
  monkeypatch.delattr(daemon.time, "CLOCK_BOOTTIME", raising=False)
  monkeypatch.setattr(daemon.time, "monotonic_ns", lambda: 456)
  assert daemon.boottime_ns() == 456


def test_linux_missing_boottime_fails_closed(monkeypatch) -> None:
  monkeypatch.setattr(daemon.os, "name", "posix")
  monkeypatch.delattr(daemon.time, "CLOCK_BOOTTIME", raising=False)
  with pytest.raises(RuntimeError, match="CLOCK_BOOTTIME"):
    daemon.boottime_ns()


def test_linux_boottime_clock_error_propagates(monkeypatch) -> None:
  monkeypatch.setattr(daemon.os, "name", "posix")
  monkeypatch.setattr(daemon.time, "CLOCK_BOOTTIME", 7, raising=False)
  monkeypatch.setattr(
    daemon.time, "clock_gettime_ns",
    lambda _clock_id: (_ for _ in ()).throw(OSError("unsupported clock")), raising=False,
  )
  with pytest.raises(OSError, match="unsupported clock"):
    daemon.boottime_ns()


def test_slow_send_completion_times_gate_next_core_and_diagnostics_send() -> None:
  clock = MutableClock(1_000_000_000)
  pm = StubPubMaster(on_send=lambda service: clock.advance(
    100_000_000 if service == "ars408StateSP" else 300_000_000,
  ))
  current = worker(pm, clock=clock)
  state = current.pipeline.state(clock())
  assert current._publish_core(state) == daemon.PublishResult.SENT
  assert current._last_core_send_host_ns == 1_100_000_000
  assert current._publish_core(state) == daemon.PublishResult.INTENTIONALLY_SUPPRESSED

  assert current._publish_diagnostics(state) == daemon.PublishResult.SENT
  diagnostics_complete_ns = current._last_diagnostics_host_ns
  assert diagnostics_complete_ns == 1_400_000_000
  faulted = current.fault()
  assert current._publish_diagnostics(faulted) == daemon.PublishResult.INTENTIONALLY_SUPPRESSED
  assert current._last_diagnostics_host_ns == diagnostics_complete_ns


def test_daemon_keeps_one_long_lived_can_subscriber_and_disables_replay_transport() -> None:
  source = Path(daemon.__file__).read_text(encoding="utf-8")
  tree = ast.parse(source)
  sub_refs = [node for node in ast.walk(tree) if isinstance(node, ast.Attribute) and node.attr == "sub_sock"]
  assert len(sub_refs) == 1
  assert "if REPLAY:" in source
  assert 'source_clock_mode="live"' in source
  assert "_new_can_socket" not in source and "_activate_post_barrier_socket" not in source


def test_fault_backoff_count_survives_barrier_drain_and_source_fence_no_progress() -> None:
  count = 0
  count = daemon._next_fault_count(count, daemon.BatchResult.FAULTED)
  assert count == 1
  # Barrier send and socket drain do not represent a healthy processed batch.
  assert daemon._next_fault_count(count, daemon.BatchResult.NO_PROGRESS) == 1
  count = daemon._next_fault_count(count, daemon.BatchResult.FAULTED)
  assert count == 2
  assert daemon._next_fault_count(count, daemon.BatchResult.NO_PROGRESS) == 2
  assert daemon._next_fault_count(count, daemon.BatchResult.HEALTHY) == 0


def test_latched_fault_can_recover_through_multidrain_real_parser_baseline(monkeypatch) -> None:
  packer = CANPacker("ARS408")
  radar = packer.make_can_msg("RadarState", ARS408_BUS, {
    "RadarState_MaxDistanceCfg": 250, "RadarState_SensorID": 0, "RadarState_OutputTypeCfg": 1,
    "RadarState_SendQualityCfg": 1, "RadarState_SendExtInfoCfg": 1, "RadarState_MotionRxState": 0,
  })

  def status(counter):
    return packer.make_can_msg("Obj_0_Status", ARS408_BUS, {
      "Obj_NofObjects": 1, "Obj_MeasCounter": counter, "Obj_InterfaceVersion": 1,
    })

  general = packer.make_can_msg("Obj_1_General", ARS408_BUS, {
    "Obj_ID": 7, "Obj_DistLong": 30, "Obj_DistLat": 0.2, "Obj_VrelLong": -1,
    "Obj_VrelLat": 0, "Obj_DynProp": 0, "Obj_RCS": 4,
  })
  quality = packer.make_can_msg("Obj_2_Quality", ARS408_BUS, {
    "Obj_ID": 7, "Obj_ProbOfExist": 6, "Obj_MeasState": 2,
  })
  extended = packer.make_can_msg("Obj_3_Extended", ARS408_BUS, {
    "Obj_ID": 7, "Obj_ArelLong": 0, "Obj_ArelLat": 0, "Obj_Class": 1,
    "Obj_OrientationAngle": 0, "Obj_Length": 4, "Obj_Width": 2,
  })

  clock = MutableClock(1_000_000_000)
  current = worker(clock=clock)
  current.pipeline = daemon.ARS408ShadowPipeline(
    clock=clock, source_clock_mode="live", expected_counter_step=1, cadence_verified=True,
  )
  assert current.try_barrier()
  current.mark_drain_complete()
  current.fault()
  clock.advance(daemon.CORE_BARRIER_MIN_INTERVAL_NS)
  assert current.try_barrier()
  current.mark_drain_complete()
  monkeypatch.setattr(daemon, "can_capnp_to_list", lambda raw: raw)
  def process(frame):
    clock.advance(100_000_000)
    return current.handle_batch([(clock(), [frame])])

  fault_epoch = current.pipeline.producer_epoch
  assert process(radar) == daemon.BatchResult.NO_PROGRESS
  assert current.pipeline.producer_epoch == fault_epoch and current.pipeline.state(clock()).producer_fault
  assert process(status(1)) == daemon.BatchResult.NO_PROGRESS
  warm_epoch = current.pipeline.producer_epoch
  assert warm_epoch != fault_epoch and current.drain_required
  current.mark_drain_complete()

  for frame in (status(2), general, quality, extended):
    assert process(frame) == daemon.BatchResult.NO_PROGRESS
    assert current.pipeline.producer_epoch == warm_epoch and not current.barrier_pending
  assert process(status(3)) == daemon.BatchResult.HEALTHY
  baseline = current.pipeline.state(clock())
  assert not baseline.producer_fault and baseline.snapshot is not None
  assert baseline.snapshot.cycle_status.value == "invalid"

  for frame in (general, quality, extended):
    assert process(frame) == daemon.BatchResult.HEALTHY
  assert process(status(4)) == daemon.BatchResult.HEALTHY
  recovered = current.pipeline.state(clock())
  assert recovered.snapshot is not None and recovered.snapshot.uncertain_targets[0].hit_count == 1


def test_fault_evidence_survives_rate_limited_barrier_diagnostics_until_logged(monkeypatch) -> None:
  pm = StubPubMaster()
  clock = MutableClock(1_000_000_000)
  current = worker(pm, clock=clock)
  assert current.try_barrier()
  current.mark_drain_complete()

  clock.advance(100_000_000)
  fault_state = current.fault()
  assert current.try_barrier(fault_state)
  assert current.last_diagnostics_result == daemon.PublishResult.INTENTIONALLY_SUPPRESSED
  assert current._pending_failure_bits & daemon.SHADOW_TRANSIENT_PRODUCER_FAILED
  current.mark_drain_complete()

  clock.advance(400_000_000)
  recovered = replace(current.pipeline.state(clock()), producer_fault=False, transient_failure_bits=0)
  monkeypatch.setattr(daemon, "can_capnp_to_list", lambda _raw: [(clock(), [(0, b"", 1)])])
  monkeypatch.setattr(current.pipeline, "process_batch", lambda *_args, **_kwargs: recovered)
  assert current.handle_batch([b"recovered"]) == daemon.BatchResult.HEALTHY
  logged = [event for service, event in pm.sent if service == "ars408DiagnosticsSP"][-1]
  assert logged.ars408DiagnosticsSP.transientFailureBits & daemon.SHADOW_TRANSIENT_PRODUCER_FAILED
  assert logged.ars408DiagnosticsSP.producerFailed
  assert current._pending_failure_bits == 0


def test_epoch_churn_cannot_exceed_physical_core_or_diagnostics_ceilings() -> None:
  pm = StubPubMaster()
  clock = MutableClock(1_000_000_000)
  current = worker(pm, clock=clock)
  assert current.try_barrier()
  current.mark_drain_complete()
  results = []
  for _ in range(13):
    clock.advance(daemon.CORE_BARRIER_MIN_INTERVAL_NS)
    if current.barrier_pending:
      assert current.try_barrier()
      current.mark_drain_complete()
    results.append(current.handle_batch([]))
  assert set(results) == {daemon.BatchResult.FAULTED}
  core_count = sum(service == "ars408StateSP" for service, _ in pm.sent)
  diagnostic_count = sum(service == "ars408DiagnosticsSP" for service, _ in pm.sent)
  assert core_count <= 14
  assert diagnostic_count <= 4


@pytest.mark.parametrize("stage", ("build", "send"))
def test_barrier_diagnostics_failure_stays_pending_and_is_not_called_success(monkeypatch, stage: str) -> None:
  pm = StubPubMaster(fail_service="ars408DiagnosticsSP" if stage == "send" else None)
  clock = MutableClock(1_000_000_000)
  current = worker(pm, clock=clock)
  if stage == "build":
    monkeypatch.setattr(current.publisher, "prepare_diagnostics",
                        lambda *_args: (_ for _ in ()).throw(RuntimeError("build")))
  assert not current.try_barrier()
  assert current.barrier_pending and current.last_diagnostics_result == daemon.PublishResult.FAILED
  assert current._pending_failure_bits & daemon.SHADOW_TRANSIENT_PRODUCER_FAILED


def test_batch_limits_reject_empty_overflow_and_excessive_source_span() -> None:
  assert not daemon.validate_batch([], 0)
  assert not daemon.validate_batch([(1, [(0, b"", 1)])], daemon.MAX_BATCH_EVENTS + 1)
  assert not daemon.validate_batch([(1, [(0, b"", 1)] * (daemon.MAX_BATCH_FRAMES + 1))], 1)
  assert not daemon.validate_batch([(1, [(0, b"", 1)]),
                                    (daemon.MAX_BATCH_SOURCE_SPAN_NS + 2, [(0, b"", 1)])], 2)
  assert daemon.validate_batch([(1, [(0, b"", 1)]), (2, [(0, b"", 1)])], 2)


def cp_and_sp_bytes(*, flag=True, safety=True):
  cp = car.CarParams.new_message(brand="tesla", notCar=False, radarUnavailable=False)
  cp_sp = custom.CarParamsSP.new_message(
    flags=int(TeslaFlagsSP.ARS408_RADAR) if flag else 0,
    safetyParam=int(TeslaSafetyFlagsSP.ARS408_RADAR) if safety else 0,
  )
  return cp, {"CarParams": cp.to_bytes(), "CarParamsSP": cp_sp.to_bytes()}


@pytest.mark.parametrize("started,brand,not_car,unavailable,flag,safety,expected", [
  (True, "tesla", False, False, True, True, True),
  (False, "tesla", False, False, True, True, False),
  (True, "toyota", False, False, True, True, False),
  (True, "tesla", True, False, True, True, False),
  (True, "tesla", False, True, True, True, False),
  (True, "tesla", False, False, False, True, False),
  (True, "tesla", False, False, True, False, False),
])
def test_internal_live_gate_is_exact_nonblocking_and_fail_closed(started, brand, not_car, unavailable, flag, safety, expected) -> None:
  cp = car.CarParams.new_message(brand=brand, notCar=not_car, radarUnavailable=unavailable)
  cp_sp = custom.CarParamsSP.new_message(
    flags=int(TeslaFlagsSP.ARS408_RADAR) if flag else 0,
    safetyParam=int(TeslaSafetyFlagsSP.ARS408_RADAR) if safety else 0,
  )
  values = {"IsOffroad": b"0" if started else b"1", "CarParams": cp.to_bytes(), "CarParamsSP": cp_sp.to_bytes()}
  assert (daemon.read_live_ars408_config(StubParams(values)) is not None) is expected
  assert daemon.read_live_ars408_config(StubParams()) is None
  assert daemon.read_live_ars408_config(StubParams({"IsOffroad": b"0", "CarParams": values["CarParams"], "CarParamsSP": b"bad"})) is None


@pytest.mark.parametrize("offroad", [None, b"1", b"bad", b"", True, False])
def test_internal_gate_accepts_only_exact_isoffroad_zero(offroad) -> None:
  _cp, values = cp_and_sp_bytes()
  if offroad is not None:
    values["IsOffroad"] = offroad
  assert daemon.read_live_ars408_config(StubParams(values)) is None


@pytest.mark.parametrize("failing_key", ["IsOffroad", "CarParams", "CarParamsSP"])
def test_internal_gate_params_read_failures_return_none_without_escaping(failing_key: str) -> None:
  _cp, values = cp_and_sp_bytes()
  values["IsOffroad"] = b"0"

  class RaisingParams(StubParams):
    def get(self, key):
      if key == failing_key:
        raise OSError(f"failed reading {key}")
      return super().get(key)

  assert daemon.read_live_ars408_config(RaisingParams(values)) is None


def test_manager_config_does_not_import_shadow_and_keeps_daemon_always_run() -> None:
  root = Path(__file__).resolve().parents[4]
  source = (root / "openpilot/system/manager/process_config.py").read_text(encoding="utf-8")
  tree = ast.parse(source)
  assert all(not (isinstance(node, (ast.Import, ast.ImportFrom)) and "ars408_shadowd" in ast.unparse(node))
             for node in tree.body)
  process = next(node for node in ast.walk(tree) if isinstance(node, ast.Call) and
                 isinstance(node.func, ast.Name) and node.func.id == "PythonProcess" and
                 node.args and isinstance(node.args[0], ast.Constant) and node.args[0].value == "ars408shadowd")
  assert isinstance(process.args[2], ast.Name) and process.args[2].id == "always_run"


def test_observational_daemon_does_not_auto_restart_and_consume_msgq_reader_slots() -> None:
  root = Path(__file__).resolve().parents[4]
  source = (root / "openpilot/system/manager/process_config.py").read_text(encoding="utf-8")
  tree = ast.parse(source)
  processes = [node for node in ast.walk(tree) if isinstance(node, ast.Call) and
               isinstance(node.func, ast.Name) and node.func.id == "PythonProcess" and
               node.args and isinstance(node.args[0], ast.Constant) and node.args[0].value == "ars408shadowd"]
  assert len(processes) == 1
  restart = [keyword for keyword in processes[0].keywords if keyword.arg == "restart_if_crash"]
  assert restart == []

  process_source = (root / "openpilot/system/manager/process.py").read_text(encoding="utf-8")
  process_tree = ast.parse(process_source)
  python_class = next(node for node in process_tree.body if isinstance(node, ast.ClassDef) and node.name == "PythonProcess")
  python_init = next(node for node in python_class.body if isinstance(node, ast.FunctionDef) and node.name == "__init__")
  restart_args = [index for index, arg in enumerate(python_init.args.args) if arg.arg == "restart_if_crash"]
  if restart_args:
    default_offset = len(python_init.args.args) - len(python_init.args.defaults)
    restart_default = python_init.args.defaults[restart_args[0] - default_offset]
    assert isinstance(restart_default, ast.Constant) and restart_default.value is False
  else:
    start = next(node for node in python_class.body if isinstance(node, ast.FunctionDef) and node.name == "start")
    assert any(isinstance(node, ast.If) and isinstance(node.test, ast.Compare) and
               isinstance(node.test.left, ast.Attribute) and node.test.left.attr == "proc" and
               any(isinstance(op, ast.IsNot) for op in node.test.ops) and
               any(isinstance(item, ast.Return) for item in node.body) for node in ast.walk(start))


def test_twenty_five_gate_toggles_reuse_one_process_lifetime_can_socket() -> None:
  calls = []
  socket = None

  def factory(*args, **kwargs):
    calls.append((args, kwargs))
    return object()

  for _ in range(25):
    socket = daemon._get_or_create_can_socket(socket, factory)
  assert len(calls) == 1
  assert calls[0] == (("can",), {"timeout": 100, "conflate": False})


def test_main_reuses_one_socket_across_twenty_five_real_gate_cycles(monkeypatch) -> None:
  class StopMain(Exception):
    pass

  gate_values = iter([value for _ in range(25) for value in (True, False, False)])
  counts = {"pubmaster": 0, "socket": 0, "worker": 0, "teardown": 0, "drain": 0}

  def gate(_params):
    try:
      return object() if next(gate_values) else None
    except StopIteration as exc:
      raise StopMain from exc

  class MainWorker:
    def __init__(self, _pm):
      counts["worker"] += 1
      self.barrier_pending = True
      self.drain_required = False

    def try_barrier(self):
      self.barrier_pending = False
      self.drain_required = True
      return True

    def mark_drain_complete(self):
      self.drain_required = False

    def try_teardown_barrier(self):
      counts["teardown"] += 1
      return True

  def pubmaster(_services):
    counts["pubmaster"] += 1
    return object()

  socket = object()

  def socket_factory(current):
    assert current is None
    counts["socket"] += 1
    return socket

  def drain(current, **_kwargs):
    assert current is socket
    counts["drain"] += 1
    return [b"raw"]

  monkeypatch.setattr(daemon, "Params", lambda: object())
  monkeypatch.setattr(daemon.messaging, "PubMaster", pubmaster)
  monkeypatch.setattr(daemon, "read_live_ars408_config", gate)
  monkeypatch.setattr(daemon, "_get_or_create_can_socket", socket_factory)
  monkeypatch.setattr(daemon, "ARS408ShadowWorker", MainWorker)
  monkeypatch.setattr(daemon.messaging, "drain_sock_raw", drain)
  monkeypatch.setattr(daemon.time, "sleep", lambda _seconds: None)
  monkeypatch.setattr(daemon, "REPLAY", False)

  with pytest.raises(StopMain):
    daemon.main()
  assert counts == {"pubmaster": 1, "socket": 1, "worker": 25, "teardown": 25, "drain": 75}

  source = Path(daemon.__file__).read_text(encoding="utf-8")
  main_tree = ast.parse(source)
  main = next(node for node in main_tree.body if isinstance(node, ast.FunctionDef) and node.name == "main")
  can_sock_none = [node for node in ast.walk(main) if isinstance(node, ast.Assign) and
                   any(isinstance(target, ast.Name) and target.id == "can_sock" for target in node.targets) and
                   isinstance(node.value, ast.Constant) and node.value.value is None]
  assert len(can_sock_none) == 1


def test_daemon_can_socket_is_explicitly_non_conflated() -> None:
  source = Path(daemon.__file__).read_text(encoding="utf-8")
  tree = ast.parse(source)
  calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call) and
           isinstance(node.func, ast.Name) and node.func.id == "socket_factory"]
  can_call = next(node for node in calls if node.args and isinstance(node.args[0], ast.Constant) and node.args[0].value == "can")
  conflate = next(keyword for keyword in can_call.keywords if keyword.arg == "conflate")
  assert isinstance(conflate.value, ast.Constant) and conflate.value.value is False


def test_both_gate_loss_paths_use_the_same_best_effort_teardown() -> None:
  source = Path(daemon.__file__).read_text(encoding="utf-8")
  tree = ast.parse(source)
  main = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "main")
  teardown_calls = [node for node in ast.walk(main) if isinstance(node, ast.Call) and
                    isinstance(node.func, ast.Name) and node.func.id == "_teardown_worker"]
  # Top-of-loop gate loss, disabled REPLAY transport, and post-read gate loss.
  assert len(teardown_calls) == 3

  calls = []
  stub = SimpleNamespace(try_teardown_barrier=lambda: calls.append("barrier") or True)
  daemon._teardown_worker(stub)
  assert calls == ["barrier"]
  daemon._teardown_worker(SimpleNamespace(
    try_teardown_barrier=lambda: (_ for _ in ()).throw(RuntimeError("send failed")),
  ))


@pytest.mark.skipif(not MSGQ_AVAILABLE, reason="Windows checkout has no built msgq extension")
def test_two_nonconflated_can_subscribers_receive_each_event() -> None:
  from openpilot.cereal import messaging

  prefix = f"ars408shadowd_test_{id(object())}"
  messaging.set_fake_prefix(prefix)
  messaging.toggle_fake_events(True)
  try:
    first = messaging.sub_sock("can", conflate=False, timeout=100)
    second = messaging.sub_sock("can", conflate=False, timeout=100)
    publisher = messaging.pub_sock("can")
    payloads = [b"one", b"two", b"three"]
    for payload in payloads:
      publisher.send(payload)
    assert [messaging.recv_one_retry(first) for _ in payloads] == payloads
    assert [messaging.recv_one_retry(second) for _ in payloads] == payloads
  finally:
    messaging.toggle_fake_events(False)
    messaging.delete_fake_prefix(prefix)
