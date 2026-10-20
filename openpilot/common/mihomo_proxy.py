"""Opt-in transport for openpilot update/model requests, without global proxy settings."""
from pathlib import Path
PROXY = 'http://127.0.0.1:17890'
def enabled() -> bool:
  root = Path('/data/mihomo')
  try:
    if not (root / 'enabled').exists(): return False
    pid = int((root / 'pid').read_text())
    return b'/data/mihomo/mihomo' in Path(f'/proc/{pid}/cmdline').read_bytes()
  except (OSError, ValueError): return False
def download_proxies() -> dict[str, str]:
  return {'http': PROXY, 'https': PROXY} if enabled() else {}
