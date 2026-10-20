"""Parked E2E: repeated starts, lost PID, missing TUN, and core crash recovery."""
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import time

assert subprocess.check_output(['hostname'], text=True).strip() == 'comma-4a408d77'
spec = importlib.util.spec_from_file_location('manager', '/data/openpilot/openpilot/selfdrive/ui/mihomo_manager.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)
m.parked()
(m.ROOT/'evidence/start_repair').mkdir(parents=True, exist_ok=True)
original_config = hashlib.sha256(m.CONFIG.read_bytes()).hexdigest()
selected = m.api('/proxies/C3-PROXY').get('now') if m.pid() else None
report = []

def check(label, same_pid=None):
  state = m.status()
  assert state['running'] and state['gateway'], state
  assert Path('/sys/class/net/c3mh0/carrier').read_text().strip() == '1'
  assert m.api('/proxies/GLOBAL')['now'] == 'C3-PROXY'
  rules = json.loads(subprocess.check_output(['ip', '-j', 'rule'], text=True))
  assert sum(r.get('priority') == 17880 for r in rules) == 1
  processes = subprocess.check_output(['ps', '-eo', 'uid,args'], text=True).splitlines()
  assert sum(line.strip().startswith('61400 /data/mihomo/mihomo ') for line in processes) == 1
  if same_pid is not None:
    assert m.pid() == same_pid
  if selected:
    m.select_node(selected)
  result = subprocess.run(['curl', '--noproxy', '*', '-4', '-fsS', '--connect-timeout', '5', '--max-time', '15',
                           '-o', '/dev/null', '-w', '%{http_code}', 'https://github.com/'], capture_output=True, text=True)
  assert result.returncode == 0 and result.stdout == '200', (label, result.returncode, result.stdout, result.stderr)
  item = {'case': label, 'pid': m.pid(), 'tun': True, 'single_core_and_policy': True, 'github_http': 200}
  report.append(item)
  print(json.dumps(item), flush=True)

try:
  for i in range(6):
    m.main('stop')
    m.main('start')
    check(f'restart_{i+1}')
  before = m.pid()
  m.main('start')
  m.main('start')
  check('repeated_start_is_idempotent', same_pid=before)
  (m.ROOT/'pid').unlink()
  m.main('start')
  check('lost_pid_file_recovery')
  m.gateway('disable')
  m.main('start')
  check('missing_tun_recovery')
  m.parked()
  subprocess.run(['sudo', '-n', '/bin/kill', '-KILL', str(m.pid())], check=True)
  time.sleep(.2)
  m.main('start')
  check('core_crash_recovery')
  assert hashlib.sha256(m.CONFIG.read_bytes()).hexdigest() == original_config
finally:
  try:
    m.main('start')
    if selected:
      m.select_node(selected)
  finally:
    (m.ROOT/'evidence/start_repair/recovery_report.json').write_text(json.dumps(report, indent=2))
print('ALL_STARTUP_AND_RECOVERY_CHECKS_PASSED', flush=True)
