import json
import os
import threading
import time

from opendbc.can.dbc import DBC
from opendbc.can.parser import get_raw_value
from opendbc.car.can_definitions import CanData
from openpilot.cereal import log


DAS_BODY_CONTROLS_ADDRESS = 0x3E9
UI_WARNING_ADDRESS = 0x311
FRONT_LIGHTING_ADDRESS = 0x3F5
VEHICLE_BUS = 1
PARTY_BUS = 0
TURN_REQUESTS = {"left": 1, "right": 2, "cancel": 3}
ACTIVE_TURN_REASON = 8
CANCEL_TURN_REASON = 4
TEMPLATE_MAX_AGE_NS = 1_500_000_000
ECHO_TIMEOUT_NS = 1_200_000_000
SESSION_TIMEOUT_NS = 12_000_000_000
MAX_SESSION_TIMEOUT_NS = 60_000_000_000
CONTEXT_GRACE_NS = 1_000_000_000
VEHICLE_FEEDBACK_TIMEOUT_NS = 2_500_000_000
CANCEL_FEEDBACK_TIMEOUT_NS = 1_500_000_000
CANCEL_SEND_TIMEOUT_NS = 1_500_000_000
CANCEL_TOTAL_TIMEOUT_NS = 5_000_000_000
MAX_CANCEL_ATTEMPTS = 3
REQUEST_PARAM = "TeslaTurnSignalTestRequest"
RESULT_PARAM = "TeslaTurnSignalTestResult"
STATUS_PARAM = "TeslaTurnSignalTestStatus"
CANCEL_PARAM = "TeslaTurnSignalTestCancel"
VALIDATION_LOG_PATH = "/data/tesla_turn_signal_validation.log"
VALIDATION_LOG_PREFIX = "[TESLA-TURN-SIGNAL-VALIDATION-v3]"
MAX_LOG_BYTES = 2 * 1024 * 1024
# Validation results still return through Params, while raw evidence logging is
# disabled on the normal dev branch except for Panda-rejected sessions.
TURN_SIGNAL_VALIDATION_LOGGING_ENABLED = False

_UI_WARNING_MESSAGE = DBC("tesla_model3_party").name_to_msg["UI_warning"]
_FRONT_LIGHTING_MESSAGE = DBC("tesla_model3_vehicle").name_to_msg["ID3F5VCFRONT_lighting"]


def source_details(source: int) -> tuple[int, str]:
  if source >= 0xC0:
    return source - 0xC0, "rejected"
  if source >= 0x80:
    return source - 0x80, "txEcho"
  return source, "rx"


def tesla_body_controls_checksum(data: bytes | bytearray) -> int:
  if len(data) != 8:
    raise ValueError("0x3E9 DAS_bodyControls must contain 8 bytes")
  return (0xE9 + 0x03 + sum(data[:7])) & 0xFF


def decode_body_controls(data: bytes) -> dict[str, int | bool]:
  if len(data) != 8:
    raise ValueError("0x3E9 DAS_bodyControls must contain 8 bytes")
  return {
    "turn_request": data[1] & 0x3,
    "turn_request_reason": (data[2] >> 1) & 0xF,
    "autopilot_active": bool(data[3] & 0x1),
    "acc_active": bool((data[3] >> 5) & 0x1),
    "counter": (data[6] >> 4) & 0xF,
    "checksum": data[7],
  }


def is_original_body_controls_frame(address: int, source: int, data: bytes) -> bool:
  return (address == DAS_BODY_CONTROLS_ADDRESS and source == VEHICLE_BUS and len(data) == 8 and
          tesla_body_controls_checksum(data) == data[7] and decode_body_controls(data)["turn_request"] == 0)


def create_body_control_frame(original_frame: bytes, direction: str, counter: int) -> bytes:
  if direction not in TURN_REQUESTS:
    raise ValueError(f"unsupported turn request: {direction}")
  if not 0 <= counter <= 15:
    raise ValueError(f"invalid DAS_bodyControls counter: {counter}")
  if not is_original_body_controls_frame(DAS_BODY_CONTROLS_ADDRESS, VEHICLE_BUS, original_frame):
    raise ValueError("original 0x3E9 RX template has invalid length or checksum")

  data = bytearray(original_frame)
  data[1] = (data[1] & 0xFC) | TURN_REQUESTS[direction]
  reason = CANCEL_TURN_REASON if direction == "cancel" else ACTIVE_TURN_REASON
  data[2] = (data[2] & 0xE1) | ((reason & 0xF) << 1)
  data[6] = (data[6] & 0x0F) | ((counter & 0xF) << 4)
  data[7] = tesla_body_controls_checksum(data)
  return bytes(data)


def decode_ui_warning(data: bytes) -> dict[str, int | bool]:
  left_state = int(get_raw_value(data, _UI_WARNING_MESSAGE.sigs["leftBlinkerBlinking"]))
  right_state = int(get_raw_value(data, _UI_WARNING_MESSAGE.sigs["rightBlinkerBlinking"]))
  return {
    "left_blinker": left_state in (1, 2),
    "right_blinker": right_state in (1, 2),
    "left_blinker_state": left_state,
    "right_blinker_state": right_state,
  }


def decode_front_lighting(data: bytes) -> dict[str, int | bool]:
  left_state = int(get_raw_value(data, _FRONT_LIGHTING_MESSAGE.sigs["VCFRONT_turnSignalLeftStatus"]))
  right_state = int(get_raw_value(data, _FRONT_LIGHTING_MESSAGE.sigs["VCFRONT_turnSignalRightStatus"]))
  return {
    "left_blinker": left_state == 1,
    "right_blinker": right_state == 1,
    "left_blinker_state": left_state,
    "right_blinker_state": right_state,
  }


def persist_validation_records(records: list[dict], log_path: str = VALIDATION_LOG_PATH) -> None:
  if not records or (not TURN_SIGNAL_VALIDATION_LOGGING_ENABLED and
                     not any(record.get("can_direction") == "rejected" for record in records)):
    return
  try:
    if os.path.exists(log_path) and os.path.getsize(log_path) > MAX_LOG_BYTES:
      os.replace(log_path, f"{log_path}.1")
    lines = "".join(json.dumps(record, sort_keys=True, default=str) + "\n" for record in records)
    with open(log_path, "a", encoding="utf-8") as log_file:
      log_file.write(lines)
  except OSError:
    pass


class TeslaTurnSignalRealtimeController:
  def __init__(self, configured: bool):
    self.configured = configured
    self._lock = threading.Lock()
    self._active = None
    self._cancel_failure = None
    self._lamp_feedback = {}
    self._completed: list[tuple[dict, list[dict]]] = []
    self._template = None
    self._template_nanos = 0
    self._template_generation = 0
    # Empty sentinel ensures a stale status from a previous card process is
    # removed on the first service cycle.
    self._last_published_status = {}

  def _record_locked(self, event: str, now_nanos: int, **values) -> None:
    if self._active is None:
      return
    self._active["records"].append({
      "prefix": VALIDATION_LOG_PREFIX,
      "test_id": self._active["test_id"],
      "wall_time_ns": time.time_ns(),
      "monotonic_ns": int(now_nanos),
      "direction": self._active["direction"],
      "event": event,
      **values,
    })

  def _finish_locked(self, result: str, now_nanos: int, **extra) -> None:
    if self._active is None:
      return
    session = self._active
    if session["cancel_requested"] and result not in ("PASS", "CANCELLED_BEFORE_SEND"):
      # Keep the unresolved hardware cancellation visible. A new request/session
      # is not proof of cancellation; later acknowledged, physical OFF feedback
      # can resolve this same attempt without restarting card.
      session["phase"] = "cancel_failed"
      session["result"] = result
      self._cancel_failure = session
    payload = {
      "test_id": session["test_id"],
      "direction": session["direction"],
      "result": result,
      "feedback": session["feedback"],
      "tx_echo": session["tx_echo"],
      "rejected": session["rejected"],
      "action_frames_sent": session["action_frames_sent"],
      "cancel_sent": session["cancel_sent"],
      "cancel_attempts": session["cancel_attempts"],
      "cancel_reason": session["cancel_reason"],
      "lane_change_started": session["lane_change_started"],
      **extra,
    }
    self._record_locked("test_finished", now_nanos, **{key: value for key, value in payload.items() if key != "test_id"})
    self._completed.append((payload, session["records"]))
    self._active = None

  def submit_request(self, test_id: str, direction: str, now_nanos: int,
                     session_timeout_ns: int = SESSION_TIMEOUT_NS, *, hold_until_cancel: bool = False) -> bool:
    if direction not in ("left", "right"):
      raise ValueError(f"unsupported turn request: {direction}")
    if not SESSION_TIMEOUT_NS <= int(session_timeout_ns) <= MAX_SESSION_TIMEOUT_NS:
      raise ValueError("turn-signal session timeout is outside the bounded range")
    with self._lock:
      if self._cancel_failure is not None:
        return False
      if not self.configured:
        records = [{
          "prefix": VALIDATION_LOG_PREFIX,
          "test_id": test_id,
          "wall_time_ns": time.time_ns(),
          "monotonic_ns": int(now_nanos),
          "direction": direction,
          "event": "test_finished",
          "result": "BLOCKED",
          "error": "TeslaTurnSignalValidation was not enabled when card initialized",
        }]
        result = {
          "test_id": test_id, "direction": direction, "result": "BLOCKED", "feedback": False,
          "tx_echo": False, "rejected": False, "action_frames_sent": 0, "cancel_sent": False,
        }
        self._completed.append((result, records))
        return False
      if self._active is not None:
        result = {
          "test_id": test_id, "direction": direction, "result": "BUSY", "feedback": False,
          "tx_echo": False, "rejected": False, "action_frames_sent": 0, "cancel_sent": False,
        }
        self._completed.append((result, []))
        return False

      self._active = {
        "test_id": test_id,
        "direction": direction,
        "started_nanos": int(now_nanos),
        "session_timeout_ns": int(session_timeout_ns),
        "hold_until_cancel": bool(hold_until_cancel),
        "used_template_generation": -1,
        "awaiting_data": None,
        "awaiting_phase": None,
        "awaiting_since_nanos": 0,
        "action_frames_sent": 0,
        "action_frames_echoed": 0,
        "cancel_sent": False,
        "cancel_frames": [],
        "cancel_attempts": 0,
        "cancel_rejections": 0,
        "cancel_echoed": False,
        "cancel_ack_nanos": 0,
        "cancel_requested": False,
        "cancel_requested_nanos": 0,
        "cancel_reason": None,
        "lane_change_state": int(log.LaneChangeState.off),
        "lane_change_started": False,
        "lane_change_direction": int(log.LaneChangeDirection.none),
        "last_context_nanos": 0,
        "phase": "waiting_vehicle_feedback",
        "feedback": False,
        "tx_echo": False,
        "rejected": False,
        "finalize_nanos": 0,
        "records": [],
      }
      self._record_locked("test_started", now_nanos, address=hex(DAS_BODY_CONTROLS_ADDRESS), execution="card_realtime")
      return True

  def _request_cancel_locked(self, reason: str, now_nanos: int) -> None:
    if self._active is None or self._active["cancel_requested"]:
      return
    if self._active["action_frames_sent"] == 0:
      self._finish_locked("CANCELLED_BEFORE_SEND", now_nanos, requested_cancel_reason=reason)
      return
    self._active["cancel_requested"] = True
    self._active["cancel_requested_nanos"] = int(now_nanos)
    self._active["cancel_reason"] = reason
    self._active["phase"] = "cancelling"
    self._record_locked("cancel_requested", now_nanos, reason=reason)

  def request_cancel(self, test_id: str | None, now_nanos: int) -> bool:
    with self._lock:
      if self._active is None or (test_id is not None and self._active["test_id"] != test_id):
        return False
      self._request_cancel_locked("web_cancel", now_nanos)
      return True

  def update_lane_change_context(self, now_nanos: int, *, valid: bool, state: int, direction: int,
                                 lateral_active: bool, brake_pressed: bool) -> None:
    with self._lock:
      if self._active is None or self._active["cancel_requested"]:
        return

      now_nanos = int(now_nanos)
      if brake_pressed:
        self._request_cancel_locked("brake_pressed", now_nanos)
        return
      if not lateral_active:
        self._request_cancel_locked("lateral_inactive", now_nanos)
        return
      if not valid:
        last_context = self._active["last_context_nanos"] or self._active["started_nanos"]
        if now_nanos - last_context >= CONTEXT_GRACE_NS:
          self._request_cancel_locked("lane_change_context_stale", now_nanos)
        return

      state = int(state)
      direction = int(direction)
      previous_state = self._active["lane_change_state"]
      self._active["last_context_nanos"] = now_nanos
      self._active["lane_change_state"] = state
      self._active["lane_change_direction"] = direction
      if state != previous_state:
        self._record_locked("lane_change_state_changed", now_nanos, previous_state=previous_state,
                            state=state, lane_change_direction=direction)

      requested_direction = (int(log.LaneChangeDirection.left) if self._active["direction"] == "left"
                             else int(log.LaneChangeDirection.right))
      if state in (int(log.LaneChangeState.preLaneChange), int(log.LaneChangeState.laneChangeStarting)) and \
         direction != requested_direction:
        self._request_cancel_locked("lane_change_direction_mismatch", now_nanos)
      elif state == int(log.LaneChangeState.preLaneChange):
        if self._active["lane_change_started"]:
          if not self._active["hold_until_cancel"]:
            self._request_cancel_locked("lane_change_cycle_complete", now_nanos)
        else:
          self._active["phase"] = "waiting_sp_start"
      elif state == int(log.LaneChangeState.laneChangeStarting):
        self._active["lane_change_started"] = True
        self._active["phase"] = "lane_changing"
      elif (state == int(log.LaneChangeState.laneChangeFinishing) and self._active["lane_change_started"]
            and not self._active["hold_until_cancel"]):
        self._request_cancel_locked("lane_change_finishing", now_nanos)
      elif (state == int(log.LaneChangeState.off) and self._active["lane_change_started"]
            and not self._active["hold_until_cancel"]):
        self._request_cancel_locked("lane_change_state_off", now_nanos)

  def _lamps_off_confirmed(self, session: dict, now_nanos: int) -> bool:
    if not session["cancel_echoed"]:
      return False
    # UI_warning distinguishes truly OFF (0) from the dark phase of blinking
    # (1). Front-lighting OFF must also be explicit: FAULT/SNA are not OFF.
    # Reuse the existing vehicle-feedback window, with observations after the
    # cancellation acknowledgement. TX echo alone never proves lamp state.
    for address in (UI_WARNING_ADDRESS, FRONT_LIGHTING_ADDRESS):
      stamp, left, right = self._lamp_feedback.get(address, (0, -1, -1))
      if not (stamp > session["cancel_ack_nanos"] and 0 <= now_nanos - stamp <= VEHICLE_FEEDBACK_TIMEOUT_NS
              and left == 0 and right == 0):
        return False
    return True

  def observe_frame(self, monotonic_nanos: int, address: int, data: bytes, source: int) -> None:
    if address not in (DAS_BODY_CONTROLS_ADDRESS, UI_WARNING_ADDRESS, FRONT_LIGHTING_ADDRESS):
      return
    data = bytes(data)
    with self._lock:
      if is_original_body_controls_frame(address, source, data):
        self._template = data
        self._template_nanos = int(monotonic_nanos)
        self._template_generation += 1
        if self._active is not None and self._active["awaiting_phase"] == "cancel":
          self._record_locked("cancel_pending_template", monotonic_nanos,
                              source=source, bus=VEHICLE_BUS, data=data.hex(),
                              decoded=decode_body_controls(data), template_generation=self._template_generation)

      bus, can_direction = source_details(source)
      decoded = None
      if address == UI_WARNING_ADDRESS and bus == PARTY_BUS and can_direction == "rx" and len(data) == 7:
        decoded = decode_ui_warning(data)
      elif address == FRONT_LIGHTING_ADDRESS and bus == VEHICLE_BUS and can_direction == "rx" and len(data) == 8:
        decoded = decode_front_lighting(data)
      if decoded is not None:
        if monotonic_nanos >= self._lamp_feedback.get(address, (0,))[0]:
          self._lamp_feedback[address] = (int(monotonic_nanos), decoded["left_blinker_state"], decoded["right_blinker_state"])

      if self._cancel_failure is not None:
        failed = self._cancel_failure
        if (address == DAS_BODY_CONTROLS_ADDRESS and bus == VEHICLE_BUS and can_direction == "txEcho"
            and data in failed["cancel_frames"]):
          failed["cancel_echoed"] = True
          failed["cancel_ack_nanos"] = int(monotonic_nanos)
          failed["awaiting_data"] = None
          failed["awaiting_phase"] = None
        if decoded is not None and self._lamps_off_confirmed(failed, monotonic_nanos):
          self._completed.append(({
            "test_id": failed["test_id"], "direction": failed["direction"], "result": "CANCEL_RECOVERED",
            "previous_result": failed["result"], "feedback": failed["feedback"],
            "tx_echo": failed["tx_echo"], "cancel_sent": failed["cancel_sent"],
          }, []))
          self._cancel_failure = None
        elif not failed["cancel_sent"] and is_original_body_controls_frame(address, source, data):
          # The bus/template recovered before any cancellation could be sent.
          # Resume only that pending cancel, under the same CAN/attempt limits;
          # never treat the new template as permission for another action.
          failed["phase"] = "cancelling"
          failed["cancel_requested_nanos"] = int(monotonic_nanos)
          failed.pop("result", None)
          self._active, self._cancel_failure = failed, None
        return
      if self._active is None:
        return

      if (address == DAS_BODY_CONTROLS_ADDRESS and bus == VEHICLE_BUS
          and (data == self._active["awaiting_data"]
               or (can_direction == "txEcho" and data in self._active["cancel_frames"]))):
        if can_direction == "rejected":
          phase = self._active["awaiting_phase"]
          self._active["rejected"] = True
          self._record_locked("body_controls_observation", monotonic_nanos, source=source, bus=bus,
                              can_direction=can_direction, phase=phase, data=data.hex(), decoded=decode_body_controls(data))
          self._active["awaiting_data"] = None
          self._active["awaiting_phase"] = None
          if phase == "cancel":
            self._active["cancel_rejections"] += 1
          if phase == "action" and self._active["action_frames_echoed"] > 0:
            self._request_cancel_locked("action_panda_rejected", monotonic_nanos)
          elif phase == "cancel" and self._active["cancel_rejections"] == self._active["cancel_attempts"]:
            # Explicit rejection is not a lost acknowledgement. Retry only the
            # pending cancel on a new OEM template, within the existing budget.
            if (self._active["cancel_attempts"] < MAX_CANCEL_ATTEMPTS and
                monotonic_nanos - self._active["cancel_requested_nanos"] < CANCEL_TOTAL_TIMEOUT_NS):
              self._active["phase"] = "cancelling"
            else:
              self._finish_locked("PANDA_REJECTED", monotonic_nanos)
          elif phase == "cancel" and self._active["cancel_attempts"] > 1:
            # A retry is expected to be rejected when the previous cancel was
            # accepted but its host-side TX echo was lost. Confirm via the
            # physical front-lighting feedback instead of sending more frames.
            self._active["cancel_echoed"] = True
            self._active["cancel_ack_nanos"] = int(monotonic_nanos)
            self._active["phase"] = "confirming_cancel"
            self._active["finalize_nanos"] = int(monotonic_nanos) + CANCEL_FEEDBACK_TIMEOUT_NS
          else:
            self._finish_locked("PANDA_REJECTED", monotonic_nanos)
          return
        if can_direction == "txEcho":
          phase = "cancel" if data in self._active["cancel_frames"] else self._active["awaiting_phase"]
          self._active["tx_echo"] = True
          self._active["awaiting_data"] = None
          self._active["awaiting_phase"] = None
          self._record_locked("body_controls_observation", monotonic_nanos, source=source, bus=bus,
                              can_direction=can_direction, phase=phase, data=data.hex(), decoded=decode_body_controls(data))
          if phase == "action":
            self._active["action_frames_echoed"] += 1
          else:
            self._active["cancel_echoed"] = True
            self._active["cancel_ack_nanos"] = int(monotonic_nanos)
            self._active["phase"] = "confirming_cancel"
            self._active["finalize_nanos"] = int(monotonic_nanos) + CANCEL_FEEDBACK_TIMEOUT_NS
          return

      if decoded is not None:
        blinker_on = bool(decoded[f"{self._active['direction']}_blinker"])
        if blinker_on and not self._active["feedback"]:
          self._active["feedback"] = True
          if not self._active["cancel_requested"]:
            self._active["phase"] = "waiting_sp_start"
          self._record_locked("vehicle_feedback", monotonic_nanos, source=source, bus=bus, data=data.hex(), decoded=decoded)
        if self._lamps_off_confirmed(self._active, monotonic_nanos):
          self._record_locked("vehicle_cancel_feedback", monotonic_nanos, source=source, bus=bus,
                              data=data.hex(), decoded=decoded)
          self._finish_locked("PASS", monotonic_nanos)

  def take_can_sends(self, now_nanos: int, *, cancel_only: bool = False) -> list[CanData]:
    with self._lock:
      if self._active is None or self._active["awaiting_data"] is not None:
        return []
      if cancel_only and not self._active["cancel_requested"]:
        return []
      if self._active["finalize_nanos"] or self._template is None:
        return []
      if self._template_generation == self._active["used_template_generation"]:
        return []
      if int(now_nanos) - self._template_nanos > TEMPLATE_MAX_AGE_NS:
        return []

      phase = "cancel" if self._active["cancel_requested"] else "action"
      direction = self._active["direction"] if phase == "action" else "cancel"
      counter = (decode_body_controls(self._template)["counter"] + 1) % 16
      data = create_body_control_frame(self._template, direction, counter)
      self._record_locked("baseline_frame", now_nanos, source=VEHICLE_BUS, bus=VEHICLE_BUS,
                          data=self._template.hex(), decoded=decode_body_controls(self._template),
                          template_generation=self._template_generation)
      self._active["used_template_generation"] = self._template_generation
      self._active["awaiting_data"] = data
      self._active["awaiting_phase"] = phase
      self._active["awaiting_since_nanos"] = int(now_nanos)
      if phase == "action":
        self._active["action_frames_sent"] += 1
      else:
        self._active["cancel_sent"] = True
        self._active["cancel_attempts"] += 1
        self._active["cancel_frames"].append(data)
      self._record_locked("frame_submitted", now_nanos, phase=phase, request=TURN_REQUESTS[direction],
                          reason=CANCEL_TURN_REASON if phase == "cancel" else ACTIVE_TURN_REASON,
                          counter=counter, bus=VEHICLE_BUS, data=data.hex(),
                          frame_index=self._active["action_frames_sent"] if phase == "action" else None,
                          frame_count=None)
      return [CanData(DAS_BODY_CONTROLS_ADDRESS, data, VEHICLE_BUS)]

  def advance_time(self, now_nanos: int) -> None:
    with self._lock:
      if self._active is None:
        return
      now_nanos = int(now_nanos)
      if self._active["finalize_nanos"] and now_nanos >= self._active["finalize_nanos"]:
        self._finish_locked("CANCEL_NOT_CONFIRMED", now_nanos)
      elif (self._active["awaiting_data"] is not None and
            now_nanos - self._active["awaiting_since_nanos"] >= ECHO_TIMEOUT_NS):
        if self._active["awaiting_phase"] == "action":
          self._active["awaiting_data"] = None
          self._active["awaiting_phase"] = None
          self._request_cancel_locked("action_tx_echo_timeout", now_nanos)
        elif self._active["cancel_attempts"] < MAX_CANCEL_ATTEMPTS:
          self._record_locked("cancel_tx_echo_retry", now_nanos,
                              cancel_attempt=self._active["cancel_attempts"])
          self._active["awaiting_data"] = None
          self._active["awaiting_phase"] = None
          self._active["phase"] = "cancelling"
        else:
          self._finish_locked("NO_TX_ECHO", now_nanos)
      elif (self._active["cancel_requested"] and not self._active["cancel_sent"] and
            now_nanos - self._active["cancel_requested_nanos"] >= CANCEL_SEND_TIMEOUT_NS):
        self._finish_locked("CANCEL_NOT_SENT", now_nanos)
      elif (not self._active["cancel_requested"] and not self._active["feedback"] and
            now_nanos - self._active["started_nanos"] >= VEHICLE_FEEDBACK_TIMEOUT_NS):
        self._request_cancel_locked("vehicle_feedback_timeout", now_nanos)
      elif (self._active["cancel_requested"] and
            now_nanos - self._active["cancel_requested_nanos"] >= CANCEL_TOTAL_TIMEOUT_NS):
        self._finish_locked("CANCEL_TIMEOUT", now_nanos)
      elif now_nanos - self._active["started_nanos"] >= self._active["session_timeout_ns"]:
        if not self._active["cancel_requested"]:
          self._request_cancel_locked("session_timeout", now_nanos)

  def drain_completed(self) -> list[tuple[dict, list[dict]]]:
    with self._lock:
      completed = self._completed
      self._completed = []
      return completed

  def status(self) -> dict | None:
    with self._lock:
      session = self._active if self._active is not None else self._cancel_failure
      if session is None:
        return None
      return {
        "test_id": session["test_id"],
        "direction": session["direction"],
        "phase": session["phase"],
        "feedback": session["feedback"],
        "action_frames_sent": session["action_frames_sent"],
        "lane_change_started": session["lane_change_started"],
        "cancel_requested": session["cancel_requested"],
        "cancel_attempts": session["cancel_attempts"],
        "cancel_reason": session["cancel_reason"],
        "result": session.get("result"),
      }

  def service_params(self, params, now_nanos: int | None = None, log_path: str = VALIDATION_LOG_PATH) -> None:
    now_nanos = time.monotonic_ns() if now_nanos is None else int(now_nanos)
    request = params.get(REQUEST_PARAM)
    if request is not None:
      params.remove(REQUEST_PARAM)
      try:
        self.submit_request(str(request["test_id"]), str(request["direction"]), now_nanos)
      except (KeyError, TypeError, ValueError):
        pass

    cancel = params.get(CANCEL_PARAM)
    if cancel is not None:
      params.remove(CANCEL_PARAM)
      self.request_cancel(str(cancel.get("test_id")) if isinstance(cancel, dict) and cancel.get("test_id") else None,
                          now_nanos)

    status = self.status()
    if status is not None and status != self._last_published_status:
      params.put(STATUS_PARAM, status)
      self._last_published_status = status
    elif status is None and self._last_published_status is not None:
      params.remove(STATUS_PARAM)
      self._last_published_status = None

    for result, records in self.drain_completed():
      persist_validation_records(records, log_path)
      params.put(RESULT_PARAM, result)
