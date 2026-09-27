#!/usr/bin/env python3
"""Offroad Chestnut model-load performance and first-frame gate."""

from __future__ import annotations

import argparse
import json
import subprocess
import time
from pathlib import Path

import numpy as np


def within_limit(seconds: float, limit_seconds: float) -> bool:
  return seconds <= limit_seconds


def write_report(path: Path, report: dict) -> None:
  path.parent.mkdir(parents=True, exist_ok=True)
  with path.open("x") as output:
    json.dump(report, output, indent=2, sort_keys=True)
    output.write("\n")


def git_head(path: Path) -> str | None:
  try:
    return subprocess.check_output(["git", "-C", str(path), "rev-parse", "HEAD"], text=True).strip()
  except (OSError, subprocess.CalledProcessError):
    return None


def main(argv=None) -> int:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--limit-seconds", type=float, default=45.0)
  parser.add_argument("--report", type=Path, required=True)
  args = parser.parse_args(argv)
  if args.limit_seconds <= 0:
    raise ValueError("limit-seconds must be positive")

  from openpilot.common.hardware.hw import Paths
  from openpilot.common.params import Params
  from openpilot.common.file_chunker import get_chunked_file_size
  from openpilot.system.hardware.chestnut.status import PCIE_L0, read_pcie_ltssm
  from openpilot.sunnypilot.models.helpers import get_active_bundle
  from tinygrad.runtime.support.usb import USB3

  params = Params()
  report = {
    "schema": "chestnut-model-load-perf-gate-v1",
    "threshold_seconds": args.limit_seconds,
    "functional_status": "FAIL",
    "performance_status": "FAIL",
    "overall_status": "FAIL",
    "offroad": params.get_bool("IsOffroad"),
    "sp_commit": git_head(Path.cwd()),
    "tinygrad_commit": git_head(Path.cwd() / "tinygrad_repo"),
    "progress": [],
  }
  if not report["offroad"]:
    report["error"] = "benchmark requires IsOffroad=1"
    write_report(args.report, report)
    return 1

  try:
    ltssm = read_pcie_ltssm()
    report["pcie_ltssm"] = f"0x{ltssm:02x}"
    if ltssm != PCIE_L0:
      raise RuntimeError(f"PCIe is not L0: 0x{ltssm:02x}")

    bundle = get_active_bundle(chestnut=True)
    if bundle is None or not bundle.models:
      raise RuntimeError("no active Chestnut model bundle")
    artifact = bundle.models[0].artifact
    model_path = Path(Paths.model_root()) / artifact.fileName
    report["bundle"] = {
      "internal_name": bundle.internalName,
      "display_name": bundle.displayName,
      "generation": bundle.generation,
      "is_20hz": bundle.is20hz,
      "ref": bundle.ref,
      "artifact": artifact.fileName,
      "artifact_bytes": get_chunked_file_size(model_path),
      "chunks": len(artifact.chunks),
    }

    original_control_write = USB3.control_write
    original_bulk_read = USB3.bulk_read
    trace = {"f2_in": 0, "bulk_ok": 0, "bulk_fail": 0, "pending": False}

    def control_write(self, request, value=0, index=0, data=b"", timeout=1000):
      result = original_control_write(self, request, value, index, data, timeout)
      if request == 0xF2 and value & 0x8000:
        trace["f2_in"] += 1
        trace["pending"] = True
      return result

    def bulk_read(self, length, timeout=1000):
      try:
        result = original_bulk_read(self, length, timeout)
        if trace["pending"]:
          if len(result) != length:
            raise RuntimeError(f"EP81 short read {len(result)}/{length}")
          trace["bulk_ok"] += 1
          trace["pending"] = False
        return result
      except Exception:
        trace["bulk_fail"] += 1
        raise

    USB3.control_write = control_write
    USB3.bulk_read = bulk_read

    from openpilot.sunnypilot.modeld_v2.modeld import ModelState
    started = time.monotonic()

    def progress(value: int) -> None:
      if not report["progress"] or report["progress"][-1]["value"] != value:
        report["progress"].append({"value": value, "seconds": time.monotonic() - started})

    state = ModelState(1928, 1208, chestnut=True, loading_progress_callback=progress)
    report["constructor_warmup_seconds"] = time.monotonic() - started

    dummy_frames = {key: np.zeros(state.frame_buf_params[key][3], dtype=np.uint8) for key in state.vision_input_names}
    transforms = {key: np.eye(3, dtype=np.float32) for key in (state._road_key, state._wide_key) if key}
    dummy_inputs = {
      key: np.zeros(value.shape, dtype=value.dtype)
      for key, value in state.numpy_inputs.items()
      if key not in ("tfm", "big_tfm", "prev_feat")
    }
    outputs = state.run(dummy_frames, transforms, dummy_inputs)
    report["ready_seconds"] = time.monotonic() - started
    arrays = {key: value for key, value in outputs.items() if isinstance(value, np.ndarray)}
    report["output_arrays"] = len(arrays)
    report["finite"] = bool(arrays) and all(np.all(np.isfinite(value)) for value in arrays.values())
    report["plan_present"] = "plan" in outputs
    report["trace"] = trace
    report["functional_status"] = "PASS" if (
      report["finite"] and report["plan_present"] and trace["bulk_fail"] == 0 and
      not trace["pending"] and trace["f2_in"] == trace["bulk_ok"]
    ) else "FAIL"
    report["performance_status"] = "PASS" if within_limit(report["ready_seconds"], args.limit_seconds) else "FAIL"
    report["overall_status"] = "PASS" if report["functional_status"] == report["performance_status"] == "PASS" else "FAIL"
  except BaseException as error:
    report["error"] = f"{type(error).__name__}: {error}"

  write_report(args.report, report)
  print(json.dumps(report, indent=2, sort_keys=True), flush=True)
  return 0 if report["overall_status"] == "PASS" else 1


if __name__ == "__main__":
  raise SystemExit(main())
