#!/usr/bin/env python3
import os
import queue
import subprocess
import threading
import time
from opendbc.car.structs import car
import openpilot.cereal.messaging as messaging
from openpilot.common.realtime import Ratekeeper
from openpilot.sunnypilot.hardware.profile import HardwareProfile, get_hardware_profile
from openpilot.sunnypilot.navassist.settings import SettingsCache

AudibleAlert = car.CarControl.HUDControl.AudibleAlert
BEEP_PULSE_SECONDS = 0.010
BEEP_GAP_SECONDS = 0.02
GPIO_PIN = 42
GPIO_VALUE_PATH = f"/sys/class/gpio/gpio{GPIO_PIN}/value"

class Beepd:
  def __init__(self):
    self.current_alert = AudibleAlert.none
    self.mads_enabled = None
    self.lane_change_initialized = False
    self.lane_change_beep_request_id = 0
    self.settings_cache = SettingsCache()
    # timestamp until which promptRepeat should be suppressed
    self.prompt_suppress_until = 0
    self.beep_lock = threading.Lock()
    self.gpio_fd = None
    self.beep_queue: queue.Queue = queue.Queue(maxsize=8)
    self.worker = threading.Thread(target=self._worker, daemon=True)
    self.worker.start()
    # Never probe an unknown device's GPIO. The manager parameter is only an
    # enable switch; the hardware profile is the compatibility authority.
    if get_hardware_profile() == HardwareProfile.C3XL:
      self.enable_gpio()

  def enable_gpio(self):
    # GPIO setup may require root, but pulse edges must not spawn processes:
    # their startup latency would dominate a sub-millisecond beep.
    try:
      subprocess.run(["sudo", "tee", "/sys/class/gpio/export"], input=str(GPIO_PIN),
                     stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL, text=True, check=False)
      subprocess.run(["sudo", "tee", f"/sys/class/gpio/gpio{GPIO_PIN}/direction"], input="out",
                     stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL, text=True, check=False)
      subprocess.run(["sudo", "chmod", "0666", GPIO_VALUE_PATH],
                     stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL, check=False)
      self.gpio_fd = os.open(GPIO_VALUE_PATH, os.O_WRONLY | os.O_CLOEXEC)
      self._beep(False)
    except OSError:
      self.gpio_fd = None

  def _beep(self, on):
    if self.gpio_fd is None:
      return
    os.lseek(self.gpio_fd, 0, os.SEEK_SET)
    os.write(self.gpio_fd, b"1" if on else b"0")

  def _worker(self):
    while True:
      func = self.beep_queue.get()
      try:
        func()
      finally:
        self.beep_queue.task_done()

  def _pulse_sequence(self, count):
    def run_sequence():
      for pulse in range(count):
        self._beep(True)
        time.sleep(BEEP_PULSE_SECONDS)
        self._beep(False)
        if pulse < count - 1:
          time.sleep(BEEP_GAP_SECONDS)

    lock = getattr(self, "beep_lock", None)
    if lock is None:
      run_sequence()
    else:
      with lock:
        run_sequence()

  def engage(self):
    self._pulse_sequence(1)

  def disengage(self):
    self._pulse_sequence(2)

  def warning(self):
    self._pulse_sequence(3)

  #def startup_beep(self):
    #self._beep(True)
    #time.sleep(0.1)
    #self._beep(False)

  def dispatch_beep(self, func):
    try:
      self.beep_queue.put_nowait(func)
    except queue.Full:
      # Alerts are edge notifications. Do not build an unbounded delayed queue.
      pass

  def close(self):
    if self.gpio_fd is not None:
      try:
        self._beep(False)
      finally:
        os.close(self.gpio_fd)
        self.gpio_fd = None

  def update_alert(self, new_alert):
    now = time.monotonic()
    if new_alert != self.current_alert:
      self.current_alert = new_alert
      print(f"[BEEP] New alert: {new_alert}")
      #if new_alert == AudibleAlert.engage:
        #self.dispatch_beep(self.engage)
      #elif new_alert == AudibleAlert.disengage:
        #self.dispatch_beep(self.disengage)
      if new_alert in [AudibleAlert.warningSoft, AudibleAlert.warningImmediate]:
        self.dispatch_beep(self.warning)
      elif new_alert == AudibleAlert.promptRepeat:
        # 如果在抑制期内则忽略后续的 promptRepeat
        if now >= getattr(self, 'prompt_suppress_until', 0):
          # 设置抑制期（秒），在这段时间内忽略重复的 promptRepeat
          self.prompt_suppress_until = now + 10
          self.dispatch_beep(self.engage)
        else:
          print(f"[BEEP] promptRepeat suppressed until {self.prompt_suppress_until}")

  def get_audible_alert(self, sm):
    if sm.updated['selfdriveState']:
      new_alert = sm['selfdriveState'].alertSound.raw
      self.update_alert(new_alert)

    if sm.updated['selfdriveStateSP']:
      self.update_mads(bool(sm['selfdriveStateSP'].mads.enabled))

    if sm.updated['navLaneIntentSP']:
      self.update_lane_change(
        sm['navLaneIntentSP'], enabled=self.settings_cache.read().lane_change_buzzer_enabled,
      )

  def update_mads(self, enabled):
    if self.mads_enabled is None:
      self.mads_enabled = enabled
      return

    if enabled != self.mads_enabled:
      self.mads_enabled = enabled
      self.dispatch_beep(self.engage if enabled else self.disengage)

  def update_lane_change(self, intent, *, enabled):
    ready = bool(intent.valid and intent.spLaneChangeReady)
    request_id = int(intent.requestId)
    if not self.lane_change_initialized:
      self.lane_change_initialized = True
      if ready:
        self.lane_change_beep_request_id = request_id
      return
    if ready and request_id > 0 and request_id != self.lane_change_beep_request_id:
      self.lane_change_beep_request_id = request_id
      if enabled:
        self.dispatch_beep(self.engage)

  def test_beepd_thread(self):
    frame = 0
    rk = Ratekeeper(20)
    pm = messaging.PubMaster(['selfdriveState'])
    while True:
      cs = messaging.new_message('selfdriveState')
      if frame == 20:
        cs.selfdriveState.alertSound = AudibleAlert.engage
      if frame == 40:
        cs.selfdriveState.alertSound = AudibleAlert.disengage
      if frame == 60:
        cs.selfdriveState.alertSound = AudibleAlert.prompt
      if frame == 80:
        cs.selfdriveState.alertSound = AudibleAlert.disengage
      if frame == 85:
        cs.selfdriveState.alertSound = AudibleAlert.prompt

      pm.send("selfdriveState", cs)
      frame += 1
      rk.keep_time()

  def beepd_thread(self, test=False):
    if test:
      threading.Thread(target=self.test_beepd_thread, daemon=True).start()

    sm = messaging.SubMaster(['selfdriveState', 'selfdriveStateSP', 'navLaneIntentSP'])
    rk = Ratekeeper(20)

    while True:
      sm.update(0)
      self.get_audible_alert(sm)
      rk.keep_time()

def main():
  s = Beepd()
  try:
    s.beepd_thread(test=False)
  finally:
    s.close()

if __name__ == "__main__":
  main()
