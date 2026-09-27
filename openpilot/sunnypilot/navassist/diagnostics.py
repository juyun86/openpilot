"""Independent bounded JSONL navigation recordings and streaming downloads."""
from dataclasses import asdict, dataclass
import gzip
import json
import math
import os
from pathlib import Path
import re
import shutil
import time
import uuid
import zipfile

from openpilot.common.hardware.hw import Paths
from openpilot.sunnypilot.navassist.settings import NavAssistSettings


NAVIGATION_ONLY_SERVICES = frozenset(('navAssistStateSP', 'navLaneIntentSP', 'laneTopologyStateSP'))
LOG_PATTERN = re.compile(r'^navdiag-(\d{13})-(\d{13})-([0-9a-f]{8})\.jsonl\.gz$')
PARTIAL_PATTERN = re.compile(r'^navdiag-\d{13}-[0-9a-f]{8}\.partial$')
MAX_LOG_BYTES = 2 * 1024 * 1024 * 1024
MAX_FILES = 1200
MAX_FILE_SECONDS = 60


def log_root() -> Path:
  return Path(Paths.log_root()).resolve().parent / 'navdiagnostics'


def ingress_status_path() -> Path:
  return Path(Paths.shm_path()) / 'navassist-ingress.json'


@dataclass(frozen=True)
class NavigationLogFile:
  path: Path
  start_ms: int
  end_ms: int
  size: int
  modified_ns: int


def scan_logs(root: Path | None = None) -> tuple[NavigationLogFile, ...]:
  root = root if root is not None else log_root()
  if not root.is_dir() or root.is_symlink():
    return ()
  result = []
  for path in root.iterdir():
    match = LOG_PATTERN.fullmatch(path.name)
    if not match or path.is_symlink() or not path.is_file():
      continue
    try:
      stat = path.stat()
      result.append(NavigationLogFile(path, int(match[1]), int(match[2]), stat.st_size, stat.st_mtime_ns))
    except OSError:
      continue
  return tuple(sorted(result, key=lambda item: (item.start_ms, item.path.name)))


def finite_values(value):
  if isinstance(value, float):
    return value if math.isfinite(value) else None
  if isinstance(value, dict):
    return {k: finite_values(v) for k, v in value.items()}
  if isinstance(value, (tuple, list)):
    return [finite_values(v) for v in value]
  return value


class NavigationLogWriter:
  def __init__(self, root: Path | None = None, *, max_bytes: int = MAX_LOG_BYTES, max_files: int = MAX_FILES,
               max_seconds: int = MAX_FILE_SECONDS):
    self.root = root if root is not None else log_root()
    self.max_bytes, self.max_files, self.max_seconds = max_bytes, max_files, max_seconds
    self.stream = None
    self.partial: Path | None = None
    self.start_ms = 0
    self.token = ''
    self.raw_bytes = 0
    self.metadata: dict | None = None
    self._prune()

  def write(self, record: dict, *, wall_ms: int | None = None) -> None:
    now = time.time_ns() // 1_000_000 if wall_ms is None else wall_ms
    if self.stream is not None and (now - self.start_ms >= self.max_seconds * 1000 or self.raw_bytes >= 8 * 1024 * 1024):
      self.close(wall_ms=now)
    if self.stream is None:
      self.root.mkdir(parents=True, exist_ok=True)
      self.start_ms, self.token = now, uuid.uuid4().hex[:8]
      self.partial = self.root / f'navdiag-{now:013d}-{self.token}.partial'
      self.stream = gzip.open(self.partial, 'xb', compresslevel=1)
      self.raw_bytes = 0
      if self.metadata is not None:
        header = (json.dumps(finite_values(self.metadata), ensure_ascii=False, allow_nan=False) + '\n').encode()
        self.stream.write(header)
        self.raw_bytes += len(header)
    data = (json.dumps(finite_values(record), ensure_ascii=False, separators=(',', ':'), allow_nan=False) + '\n').encode()
    if len(data) > 64 * 1024:
      raise ValueError('导航样本超过 64 KiB')
    self.stream.write(data)
    self.raw_bytes += len(data)

  def close(self, *, wall_ms: int | None = None) -> None:
    if self.stream is None or self.partial is None:
      return
    now = time.time_ns() // 1_000_000 if wall_ms is None else wall_ms
    self.stream.close()
    self.stream = None
    final = self.root / f'navdiag-{self.start_ms:013d}-{max(now, self.start_ms):013d}-{self.token}.jsonl.gz'
    os.replace(self.partial, final)
    self.partial = None
    self._prune()

  def _prune(self) -> None:
    files = list(scan_logs(self.root))
    if self.root.is_dir() and not self.root.is_symlink():
      for path in self.root.iterdir():
        if PARTIAL_PATTERN.fullmatch(path.name) and not path.is_symlink() and path.is_file() and path != self.partial:
          stat = path.stat()
          files.append(NavigationLogFile(path, int(path.name.split('-')[1]), 0, stat.st_size, stat.st_mtime_ns))
    files.sort(key=lambda item: (item.start_ms, item.path.name))
    size = sum(item.size for item in files)
    # Reserve space for the current <=8 MiB raw block plus one bounded record.
    budget = self.max_bytes - min(9 * 1024 * 1024, self.max_bytes // 8)
    while files and (len(files) > self.max_files or size > budget):
      old = files.pop(0)
      try:
        old.path.unlink()
        size -= old.size
      except OSError:
        pass


def select_logs(start_ms: int, end_ms: int, root: Path | None = None) -> tuple[NavigationLogFile, ...]:
  if (isinstance(start_ms, bool) or isinstance(end_ms, bool) or not isinstance(start_ms, int) or not isinstance(end_ms, int)
      or start_ms < 0 or end_ms <= start_ms or end_ms - start_ms > 7 * 24 * 3600 * 1000):
    raise ValueError('请选择不超过七天的有效时间范围')
  files = tuple(item for item in scan_logs(root) if item.end_ms >= start_ms and item.start_ms <= end_ms)
  if sum(item.size for item in files) > MAX_LOG_BYTES or len(files) > MAX_FILES:
    raise ValueError('所选导航日志过大，请缩小范围')
  return files


def log_status(root: Path | None = None) -> dict:
  root = root if root is not None else log_root()
  files = scan_logs(root)
  status = {}
  try:
    if (root / 'status.json').stat().st_size <= 4096:
      candidate = json.loads((root / 'status.json').read_text())
      if isinstance(candidate, dict) and isinstance(candidate.get('updated_ms', 0), (int, float)):
        status = candidate
  except (OSError, ValueError):
    pass
  now_ms = time.time_ns() // 1_000_000
  return {
    'available': bool(files), 'file_count': len(files), 'total_bytes': sum(f.size for f in files),
    'start_ms': min((f.start_ms for f in files), default=None),
    'end_ms': max((f.end_ms for f in files), default=None),
    'recorder_running': 0 <= now_ms - status.get('updated_ms', 0) < 15000,
    'recording_enabled': status.get('recording_enabled', True), 'last_sample_ms': status.get('last_sample_ms'),
    'error': status.get('error'), 'max_bytes': MAX_LOG_BYTES,
  }


def flush_recording(root: Path | None = None) -> bool:
  root = root if root is not None else log_root()
  if not root.is_dir():
    return False
  marker = root / 'flush.request'
  marker.touch(exist_ok=True)
  deadline = time.monotonic() + 2.0
  while marker.exists() and time.monotonic() < deadline:
    time.sleep(0.05)
  return not marker.exists()


def stream_logs(files: tuple[NavigationLogFile, ...], output, *, start_ms: int, end_ms: int) -> None:
  with zipfile.ZipFile(output, 'w', compression=zipfile.ZIP_STORED, allowZip64=True) as archive:
    archive.writestr('manifest.json', json.dumps({
      'format': 'tesnav-navigation-jsonl-v1', 'start_ms': start_ms, 'end_ms': end_ms,
      'files': [{'name': f.path.name, 'start_ms': f.start_ms, 'end_ms': f.end_ms, 'bytes': f.size} for f in files],
      'contains_qlog': False, 'contains_other_diagnostics': False, 'contains_video': False,
    }, ensure_ascii=False, indent=2))
    for item in files:
      fd = os.open(item.path, os.O_RDONLY | os.O_NOFOLLOW)
      with os.fdopen(fd, 'rb') as source:
        stat = os.fstat(source.fileno())
        if stat.st_size != item.size or stat.st_mtime_ns != item.modified_ns:
          raise ValueError('导航日志已变化，请刷新后下载')
        with archive.open(f'navigation/{item.path.name}', 'w') as target:
          shutil.copyfileobj(source, target, length=64 * 1024)


def recording_metadata(settings: NavAssistSettings, params) -> dict:
  keys = ('GitCommit', 'GitBranch', 'LongitudinalPlannerMode', 'LaneTurnDesire', 'LaneTurnValue',
          'AutoLaneChangeTimer', 'TeslaTurnSignalValidation', 'RoadEdgeLaneChangeEnabled')
  values = {}
  for key in keys:
    value = params.get(key)
    values[key] = value.decode(errors='replace') if isinstance(value, bytes) else value
  return {'kind': 'metadata', 'version': 1, 'settings': asdict(settings), 'device_settings': values}
