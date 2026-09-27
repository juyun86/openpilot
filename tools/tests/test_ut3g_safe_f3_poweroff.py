from tools.ut3g_safe_f3_poweroff import ITGZ_PRODUCT, PCIE_L0, PRODUCT, SafePowerOffError, safe_power_off
import pytest


class FakeUSB:
  def __init__(self, before, after=0):
    self.product, self.values, self.writes = PRODUCT, [before, after], []
  def control_read(self, request, length, **kwargs):
    assert (request, length) == (0xE4, 1)
    return bytes((self.values.pop(0) if len(self.values) > 1 else self.values[0],))
  def control_write(self, request, **kwargs):
    self.writes.append((request, kwargs))


def test_f3_leaves_l0_and_never_uses_persistent_opcode():
  usb = FakeUSB(PCIE_L0, 0)
  report = safe_power_off(usb, sleeper=lambda _: None)
  assert usb.writes == [(0xF3, {"value": 0, "timeout": 10_000})]
  assert report["ltssm_after"] == 0 and report["ltssm_samples"] == [0, 0, 0, 0]
  assert report["persistent_writes"] == 0
  assert report["safe_to_cut_external_power"] is True


def test_already_off_is_idempotent_and_sends_no_f3():
  usb = FakeUSB(0)
  report = safe_power_off(usb, sleeper=lambda _: None)
  assert usb.writes == [] and report["f3_writes"] == 0


def test_f3_must_actually_leave_l0():
  with pytest.raises(SafePowerOffError, match="returned to L0"):
    safe_power_off(FakeUSB(PCIE_L0, PCIE_L0), sleeper=lambda _: None)


def test_dual_product_uses_same_volatile_f3_only_path():
  usb = FakeUSB(PCIE_L0, 0)
  usb.product = "custom d1377a01-UT3G-DUAL"
  report = safe_power_off(usb, sleeper=lambda _: None)
  assert usb.writes == [(0xF3, {"value": 0, "timeout": 10_000})]
  assert report["persistent_writes"] == 0


def test_dirty_or_malformed_dual_product_is_rejected():
  usb = FakeUSB(PCIE_L0, 0)
  usb.product = "custom d1377a01-UT3G-DUAL-DIRTY"
  with pytest.raises(SafePowerOffError, match="unexpected product"):
    safe_power_off(usb, sleeper=lambda _: None)


def test_verified_itgz_product_uses_same_volatile_poweroff():
  from openpilot.common.hardware.usb import ITGZ_PRODUCT as RUNTIME_PRODUCT
  assert ITGZ_PRODUCT == RUNTIME_PRODUCT
  usb = FakeUSB(PCIE_L0, 0)
  usb.product = ITGZ_PRODUCT
  report = safe_power_off(usb, sleeper=lambda _: None)
  assert usb.writes == [(0xF3, {"value": 0, "timeout": 10_000})]
  assert report["safe_to_cut_external_power"] is True


def test_unverified_itgz_product_is_rejected_without_usb_requests():
  usb = FakeUSB(PCIE_L0, 0)
  usb.product = ITGZ_PRODUCT + "-DIRTY"
  with pytest.raises(SafePowerOffError, match="unexpected product"):
    safe_power_off(usb, sleeper=lambda _: None)
  assert usb.writes == [] and usb.values == [PCIE_L0, 0]
