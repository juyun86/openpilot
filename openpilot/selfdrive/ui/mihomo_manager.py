"""Explicit-proxy management for the C3 touchscreen. No global routes or DNS changes."""
import base64
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import urllib.parse
import urllib.request
import yaml

ROOT = Path('/data/mihomo')
CORE = ROOT / 'mihomo'
CONFIG = ROOT / 'config.yaml'
GATEWAY = '/data/openpilot/openpilot/selfdrive/ui/mihomo_gateway.py'

def gateway(action):
  result = subprocess.run(['sudo', '-n', '/usr/bin/python3', GATEWAY, action], capture_output=True, text=True, timeout=15)
  if result.returncode: raise RuntimeError('热点代理设置失败，请确认 C3 热点已开启')

def parked():
  assert Path('/data/params/d/IsOffroad').read_text().strip() == '1', '请停车后操作'
  assert not Path('/data/params/d/IsEngaged').exists() or Path('/data/params/d/IsEngaged').read_text().strip() != '1', '请先退出辅助驾驶'

def decode64(s):
  return base64.urlsafe_b64decode(s + '=' * (-len(s) % 4)).decode()

def node_uri(value):
  if value.startswith('vmess://'):
    d = json.loads(decode64(value[8:])); n = {'name': d.get('ps') or 'VMess', 'type': 'vmess', 'server': d['add'], 'port': int(d['port']), 'uuid': d['id'], 'alterId': int(d.get('aid') or 0), 'cipher': d.get('scy') or 'auto', 'tls': d.get('tls') == 'tls'}
    if d.get('net') not in (None, '', 'tcp', 'ws'): raise ValueError('此 VMess 传输方式请用 YAML 导入')
    if d.get('net') == 'ws': n.update(network='ws', **{'ws-opts': {'path': d.get('path') or '/', 'headers': {'Host': d.get('host') or d['add']}}})
    if d.get('sni'): n['servername'] = d['sni']
    return n
  if value.startswith('ss://'):
    part = value[5:].split('#', 1)[0]
    if '@' not in part: part = decode64(part)
    u = urllib.parse.urlsplit('ss://' + part)
    credentials = urllib.parse.unquote(u.username or '')
    if u.password is not None: method, password = credentials, urllib.parse.unquote(u.password)
    else: method, password = decode64(credentials).split(':', 1)
    if u.query: raise ValueError('带插件的 SS 节点请用 YAML 导入')
    return {'name': urllib.parse.unquote(value.split('#', 1)[1]) if '#' in value else 'Shadowsocks', 'type': 'ss', 'server': u.hostname, 'port': u.port, 'cipher': method, 'password': password}
  u = urllib.parse.urlsplit(value); q = dict(urllib.parse.parse_qsl(u.query))
  if u.scheme not in ('vless', 'trojan'): raise ValueError('支持订阅链接、Clash YAML、SS/VMess/VLESS/Trojan 二维码')
  n = {'name': urllib.parse.unquote(u.fragment) or u.scheme.upper(), 'type': u.scheme, 'server': u.hostname, 'port': u.port or 443}
  if u.scheme == 'vless': n.update(uuid=urllib.parse.unquote(u.username or ''), tls=q.get('security', 'tls') != 'none')
  else: n.update(password=urllib.parse.unquote(u.username or ''), sni=q.get('sni') or u.hostname)
  if q.get('sni'): n['servername' if u.scheme == 'vless' else 'sni'] = q['sni']
  if q.get('fp'): n['client-fingerprint'] = q['fp']
  if q.get('flow'): n['flow'] = q['flow']
  if q.get('security') == 'reality': n['reality-opts'] = {'public-key': q['pbk'], 'short-id': q.get('sid', '')}
  network = q.get('type', 'tcp')
  if network not in ('tcp', 'ws'): raise ValueError('此节点传输方式请用 YAML 导入')
  if network == 'ws': n.update(network='ws', **{'ws-opts': {'path': q.get('path', '/'), 'headers': {'Host': q.get('host') or u.hostname}}})
  return n

def normalize(raw):
  d = yaml.safe_load(raw)
  if isinstance(d, str):
    text = d.strip()
    if '://' not in text:
      try: text = decode64(text)
      except Exception: raise ValueError('内容不是可识别的 Clash 配置') from None
    d = {'proxies': [node_uri(line.strip()) for line in text.splitlines() if line.strip()]}
  if not isinstance(d, dict) or not (d.get('proxies') or d.get('proxy-providers')): raise ValueError('配置中没有节点或订阅集合')
  for k in ['listeners', 'external-controller-unix', 'external-controller-pipe', 'external-controller-tls', 'external-ui-url', 'external-ui-name', 'authentication', 'iptables', 'routing-mark']:
    d.pop(k, None)
  secret = (ROOT / 'secret').read_text().strip()
  d.update({'mixed-port': 17890, 'port': 0, 'socks-port': 0, 'redir-port': 0, 'tproxy-port': 0, 'allow-lan': False, 'bind-address': '127.0.0.1', 'external-controller': '127.0.0.1:17891', 'external-ui': '', 'secret': secret, 'tun': {'enable': True, 'device': 'c3mh0', 'stack': 'gvisor', 'auto-route': False, 'auto-detect-interface': False, 'mtu': 1500}, 'dns': {'enable': True, 'listen': '0.0.0.0:17893', 'enhanced-mode': 'redir-host', 'default-nameserver': ['223.5.5.5', '119.29.29.29'], 'proxy-server-nameserver': ['223.5.5.5', '119.29.29.29'], 'nameserver': ['https://1.1.1.1/dns-query#C3-PROXY']}, 'ipv6': False, 'log-level': 'warning', 'find-process-mode': 'off', 'mode': 'global'})
  for kind in ('proxy-providers', 'rule-providers'):
    for i, provider in enumerate(d.get(kind, {}).values()):
      if isinstance(provider, dict): provider['path'] = f'./providers/{kind}-{i}.yaml'
  group = {'name': 'C3-PROXY', 'type': 'select', 'proxies': [n['name'] for n in d.get('proxies', [])]}
  if d.get('proxy-providers'): group['use'] = list(d['proxy-providers'])
  d['proxy-groups'] = [group]
  d['rules'] = ['IP-CIDR,127.0.0.0/8,DIRECT,no-resolve', 'IP-CIDR,192.168.0.0/16,DIRECT,no-resolve', 'IP-CIDR,10.0.0.0/8,DIRECT,no-resolve', 'IP-CIDR,172.16.0.0/12,DIRECT,no-resolve', 'MATCH,C3-PROXY']
  d.pop('rule-providers', None)
  return d

def pid():
  try:
    p = int((ROOT / 'pid').read_text())
    if str(CORE).encode() in Path(f'/proc/{p}/cmdline').read_bytes(): return p
  except (OSError, ValueError): pass
  return None

def api(path, payload=None):
  headers = {'Authorization': 'Bearer ' + (ROOT / 'secret').read_text().strip()}
  data = None if payload is None else json.dumps(payload).encode()
  if data: headers['Content-Type'] = 'application/json'
  req = urllib.request.Request('http://127.0.0.1:17891' + path, data=data, headers=headers, method='GET' if payload is None else 'PUT')
  with urllib.request.build_opener(urllib.request.ProxyHandler({})).open(req, timeout=4) as response:
    content = response.read(); return json.loads(content) if content else {}

def stop():
  (ROOT / 'enabled').unlink(missing_ok=True)
  if (ROOT / 'gateway_state.json').exists(): gateway('disable')
  if pid(): gateway('stopcore')

def start():
  if pid(): return
  assert CONFIG.exists(), '请先添加订阅或节点'
  use_tun = yaml.safe_load(CONFIG.read_text()).get('tun', {}).get('enable', False)
  if use_tun: gateway('prepare')
  gateway('startcore')
  for _ in range(30):
    if not pid():
      stop(); raise RuntimeError('内核启动失败，请检查配置或网络')
    try: api('/version')
    except Exception:
      time.sleep(.1); continue
    try:
      api('/proxies/GLOBAL', {'name': 'C3-PROXY'})
      if use_tun:
        for _ in range(20):
          try:
            if Path('/sys/class/net/c3mh0/carrier').read_text().strip() == '1': break
          except OSError: pass
          time.sleep(.1)
        else: raise RuntimeError('TUN 未能建立，热点转发未开启')
        gateway('enable')
      (ROOT / 'enabled').write_text('1')
    except Exception:
      stop(); raise
    return
  stop(); raise RuntimeError('内核启动超时')

def status():
  d = yaml.safe_load(CONFIG.read_text()) if CONFIG.exists() else {}
  result = {'running': bool(pid()), 'configured': bool(d), 'gateway': (ROOT / 'gateway_enabled').exists() and bool(pid()), 'nodes': len(d.get('proxies', [])), 'groups': [], 'message': '尚未添加订阅或节点' if not d else '配置已保存'}
  if result['running']:
    result['version'] = api('/version').get('version')
    result['groups'] = [{'name': name, 'now': p.get('now', ''), 'all': p.get('all', [])} for name, p in api('/proxies').get('proxies', {}).items() if p.get('type') == 'Selector' and name != 'GLOBAL']
  return result

def main(command):
  if command == 'status': return status()
  parked()
  with (ROOT / 'operation.lock').open('a') as lock:
    fcntl.flock(lock, fcntl.LOCK_EX)
    if command == 'import':
      value = sys.stdin.read(2_000_001).strip(); assert len(value) <= 2_000_000, '配置过大'
      if value.startswith(('clash://', 'clash-meta://')):
        value = dict(urllib.parse.parse_qsl(urllib.parse.urlsplit(value).query)).get('url', '')
      if value.startswith(('https://', 'http://')):
        try:
          req = urllib.request.Request(value, headers={'User-Agent': 'Clash.Meta'})
          with urllib.request.build_opener(urllib.request.ProxyHandler({})).open(req, timeout=25) as response: raw = response.read(2_000_001).decode('utf-8-sig')
          assert len(raw) <= 2_000_000
        except Exception: raise ValueError('订阅下载失败，请检查链接和 C3 网络') from None
      else: raw = value
      d = normalize(raw); candidate = ROOT / 'candidate.yaml'
      candidate.write_text(yaml.safe_dump(d, allow_unicode=True, sort_keys=False)); candidate.chmod(0o600)
      check = subprocess.run([str(CORE), '-t', '-d', str(ROOT), '-f', str(candidate)], capture_output=True, timeout=35)
      if check.returncode: raise ValueError('配置校验失败；请导入兼容的 Clash YAML')
      old = CONFIG.read_bytes() if CONFIG.exists() else None; running = bool(pid())
      if running: stop()
      os.replace(candidate, CONFIG)
      try:
        if running: start()
      except Exception:
        if old is not None: CONFIG.write_bytes(old)
        else: CONFIG.unlink(missing_ok=True)
        if running and old is not None: start()
        raise
    elif command == 'start': start()
    elif command == 'stop': stop()
    elif command == 'next':
      groups = status()['groups']; assert groups, '请先启动代理并加载节点'
      g = groups[0]; choices = g['all']; assert choices, '没有可选节点'
      index = choices.index(g['now']) if g['now'] in choices else -1
      api('/proxies/' + urllib.parse.quote(g['name'], safe=''), {'name': choices[(index + 1) % len(choices)]})
    elif command == 'test':
      assert pid(), '请先启动代理'
      result = subprocess.run(['curl', '-fsS', '--proxy', 'http://127.0.0.1:17890', '--noproxy', '', '--connect-timeout', '8', '--max-time', '20', '-o', '/dev/null', '-w', '%{http_code}', 'https://api.github.com/zen'], capture_output=True, text=True)
      return {'ok': result.returncode == 0, 'message': 'GitHub 连接成功' if result.returncode == 0 else '连接失败，请检查节点和网络', 'http_code': result.stdout}
    else: raise ValueError('未知操作')
  return status()

if __name__ == '__main__':
  try: print(json.dumps(main(sys.argv[1]), ensure_ascii=False))
  except Exception as error:
    message = str(error) if isinstance(error, (AssertionError, ValueError, RuntimeError)) else '操作失败，请检查网络和配置'
    print(json.dumps({'error': message}, ensure_ascii=False)); sys.exit(1)
