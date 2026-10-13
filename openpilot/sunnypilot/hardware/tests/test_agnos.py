import json
import os
from pathlib import Path
import subprocess

import pytest

from openpilot.sunnypilot.hardware.agnos import UnsafeBootChainManifest, validate_agnos_manifest
from openpilot.sunnypilot.hardware.profile import HardwareProfile


REPO_ROOT = Path(__file__).parents[4]
STANDARD_MANIFEST = Path(__file__).parents[3] / "common/hardware/comma/agnos.json"
C3XL_MANIFEST = Path(__file__).parents[3] / "common/hardware/comma/agnos-c3xl.json"


def read_manifest(path: Path = C3XL_MANIFEST) -> list[dict]:
  with path.open() as manifest_file:
    return json.load(manifest_file)


def test_c3xl_manifest_matches_allowlist() -> None:
  validate_agnos_manifest(read_manifest(), HardwareProfile.C3XL)


@pytest.mark.parametrize("name", ["xbl", "xbl_config", "abl", "aop", "devcfg", "boot"])
def test_c3xl_rejects_changed_boot_chain_image(name: str) -> None:
  manifest = read_manifest()
  partition = next(partition for partition in manifest if partition["name"] == name)
  partition["hash"] = "0" * 64

  with pytest.raises(UnsafeBootChainManifest, match=name):
    validate_agnos_manifest(manifest, HardwareProfile.C3XL)


def test_standard_profile_does_not_apply_c3xl_allowlist() -> None:
  manifest = read_manifest(STANDARD_MANIFEST)
  next(partition for partition in manifest if partition["name"] == "boot")["hash"] = "0" * 64
  validate_agnos_manifest(manifest, HardwareProfile.STANDARD)


def test_standard_manifest_is_official_agnos_19_7() -> None:
  partitions = {partition["name"]: partition for partition in read_manifest(STANDARD_MANIFEST)}
  assert partitions["abl"]["hash"] == "29fd7ed1c012e599420764840f9f11286d34dbff4adaf102a447f06d8c5e0b35"
  assert partitions["boot"]["hash"] == "6ecf6f987cd11968104abcccabbe268485d329cdb73012dfd3c381a6b8deb27d"
  assert partitions["system"]["hash_raw"] == "3c271e2b3d20d2f0a8bf6555a1319f3efb12845490967d6151195174a01e912f"


def test_c3xl_manifest_uses_official_19_7_system_with_validated_boot_chain() -> None:
  partitions = {partition["name"]: partition for partition in read_manifest(C3XL_MANIFEST)}
  assert partitions["boot"]["hash"] == "0191529aa97d90d1fa04b472d80230b777606459e1e1e9e2323c9519839827b4"
  assert partitions["system"]["hash_raw"] == "3c271e2b3d20d2f0a8bf6555a1319f3efb12845490967d6151195174a01e912f"


@pytest.mark.parametrize("profile,version,manifest", [
  ("standard", "19.7", "openpilot/common/hardware/comma/agnos.json"),
  ("c3xl", "19.7", "openpilot/common/hardware/comma/agnos-c3xl.json"),
])
def test_launch_environment_selects_profile_specific_agnos(profile: str, version: str, manifest: str) -> None:
  env = os.environ.copy()
  env["SUNNYPILOT_HARDWARE_PROFILE"] = profile
  output = subprocess.check_output(
    ["bash", "-c", "unset AGNOS_VERSION AGNOS_MANIFEST_FILE; source launch_env.sh; printf '%s\\n%s\\n' \"$AGNOS_VERSION\" \"$AGNOS_MANIFEST_FILE\""],
    cwd=REPO_ROOT, env=env, text=True,
  ).splitlines()
  assert output == [version, manifest]
  selected = REPO_ROOT / output[1]
  assert selected.is_file(), f"Selected boot manifest is missing: {selected}"
  validate_agnos_manifest(read_manifest(selected), HardwareProfile(profile))


def test_updater_reads_real_shell_configuration_and_skips_installed_os(monkeypatch):
  from openpilot.system.updated import updated
  from openpilot.common.hardware.comma import agnos
  monkeypatch.setenv("SUNNYPILOT_HARDWARE_PROFILE", "c3xl")
  monkeypatch.setattr(updated, "OVERLAY_MERGED", str(REPO_ROOT))
  monkeypatch.setattr(updated.HARDWARE, "get_os_version", lambda: "19.7")
  monkeypatch.setattr(agnos, "flash_agnos_update", lambda *a: pytest.fail("Must not flash an already installed OS"))
  updated.handle_agnos_update()


def test_fresh_install_upgrade_passes_existing_validated_manifest(monkeypatch):
  from openpilot.system.updated import updated
  from openpilot.common.hardware.comma import agnos
  monkeypatch.setenv("SUNNYPILOT_HARDWARE_PROFILE", "c3xl")
  monkeypatch.setattr(updated, "OVERLAY_MERGED", str(REPO_ROOT))
  monkeypatch.setattr(updated.HARDWARE, "get_os_version", lambda: "18.4")
  monkeypatch.setattr(updated, "set_consistent_flag", lambda value: None)
  monkeypatch.setattr(agnos, "get_target_slot_number", lambda: 1)
  flashed = []

  def validate_only(path, slot, log):
    validate_agnos_manifest(read_manifest(Path(path)), HardwareProfile.C3XL)
    flashed.append((Path(path), slot))

  monkeypatch.setattr(agnos, "flash_agnos_update", validate_only)
  updated.handle_agnos_update()
  assert flashed == [(C3XL_MANIFEST, 1)]


@pytest.mark.parametrize("model_name,manifest", [
  ("comma tici", C3XL_MANIFEST),
  ("comma tizi", STANDARD_MANIFEST),
])
def test_factory_reset_without_profile_file_uses_device_model(tmp_path, model_name, manifest):
  model = tmp_path / "model"
  model.write_bytes(model_name.encode() + b"\x00")
  env = os.environ.copy()
  env.pop("SUNNYPILOT_HARDWARE_PROFILE", None)
  env["SUNNYPILOT_HARDWARE_PROFILE_FILE"] = str(tmp_path / "absent-profile")
  env["SUNNYPILOT_HARDWARE_MODEL_FILE"] = str(model)
  selected = subprocess.check_output([
    "bash", "-c", 'unset AGNOS_VERSION AGNOS_MANIFEST_FILE; source launch_env.sh; printf "%s" "$AGNOS_MANIFEST_FILE"',
  ], cwd=REPO_ROOT, env=env, text=True)
  assert REPO_ROOT / selected == manifest
  assert (REPO_ROOT / selected).is_file()
