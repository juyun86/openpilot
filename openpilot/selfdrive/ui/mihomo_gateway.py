"""Root helper for the dedicated C3 proxy core and global/hotspot TUN routing."""
import ipaddress,json,os,pathlib,socket,subprocess,sys,shutil,signal,time,pwd
ROOT=pathlib.Path('/data/mihomo');DEV='c3mh0';TABLE='17890';PRIORITY='17890';CHAIN='C3MH_DNS';IPT='iptables-legacy';CORE_UID=61400
def run(*args,check=True):
 return subprocess.run(list(args),capture_output=True,text=True,check=check)
def hotspot():
 data=json.loads(run('ip','-j','-4','addr','show','dev','wlan0').stdout)
 addresses=[a for dev in data for a in dev['addr_info'] if a['scope']=='global']
 if not addresses:raise RuntimeError('请先在设置 > 网络中开启 C3 热点')
 a=addresses[0];assert a['local']=='192.168.43.1','请使用 C3 热点模式'
 return a['local'],str(ipaddress.ip_network(f"{a['local']}/{a['prefixlen']}",strict=False))
def prepare():
 # Wi-Fi client mode is valid too: only add hotspot rules when its address exists.
 state=ROOT/'gateway_state.json'
 if not state.exists():state.write_text(json.dumps({'previous_forward':pathlib.Path('/proc/sys/net/ipv4/ip_forward').read_text().strip()}))
 if run('ip','link','show',DEV,check=False).returncode:
  run('ip','tuntap','add','dev',DEV,'mode','tun','user',str(CORE_UID))
 run('ip','addr','replace','198.18.0.1/30','dev',DEV);run('ip','link','set',DEV,'up')
 pathlib.Path('/proc/sys/net/ipv4/ip_forward').write_text('1')
def core_pids():
 result=[]
 for entry in pathlib.Path('/proc').iterdir():
  if not entry.name.isdigit():continue
  try:
   if entry.stat().st_uid==CORE_UID and (entry/'cmdline').read_bytes().split(b'\0')[0]==str(ROOT/'mihomo').encode():result.append(int(entry.name))
  except (OSError,ValueError):pass
 return result
def startcore():
 try:
  pwd.getpwuid(CORE_UID);raise RuntimeError('Dedicated runtime UID is already assigned')
 except KeyError:pass
 assert not core_pids(),'代理内核仍在运行，请先停止后重试'
 runtime=ROOT/'runtime';runtime.mkdir(exist_ok=True);runtime.chmod(0o700);os.chown(runtime,CORE_UID,CORE_UID)
 ROOT.chmod(0o711)
 target=runtime/'config.yaml';shutil.copyfile(ROOT/'config.yaml',target);target.chmod(0o600);os.chown(target,CORE_UID,CORE_UID)
 def demote():os.setgroups([]);os.setgid(CORE_UID);os.setuid(CORE_UID)
 env={'PATH':'/usr/bin:/bin','HOME':str(runtime),'USER':'mihomo-c3'}
 with (ROOT/'core.log').open('ab') as log:
  process=subprocess.Popen([str(ROOT/'mihomo'),'-d',str(runtime),'-f',str(target)],stdout=log,stderr=log,env=env,start_new_session=True,preexec_fn=demote)
 path=ROOT/'pid';path.write_text(str(process.pid));path.chmod(0o600);os.chown(path,1000,1000)
 # Popen returns before exec: do not report a launched PID until it is the core.
 deadline=time.monotonic()+5
 while time.monotonic()<deadline:
  if process.poll() is not None:raise RuntimeError('代理内核退出，请检查内核日志')
  if process.pid in core_pids():return
  time.sleep(.05)
 raise RuntimeError('代理内核启动超时')
def stopcore():
 # A failed early launch can leave a core whose PID is absent or stale.
 targets=core_pids()
 for pid in targets:
  try:os.kill(pid,signal.SIGTERM)
  except ProcessLookupError:pass
 deadline=time.monotonic()+4
 while time.monotonic()<deadline and set(targets)&set(core_pids()):time.sleep(.1)
 for pid in set(targets)&set(core_pids()):
  try:os.kill(pid,signal.SIGKILL)
  except ProcessLookupError:pass
 deadline=time.monotonic()+1
 while time.monotonic()<deadline and set(targets)&set(core_pids()):time.sleep(.05)
 assert not set(targets)&set(core_pids()),'代理内核未能停止'
 (ROOT/'pid').unlink(missing_ok=True)
def enable():
 try: _,network=hotspot()
 except (RuntimeError,AssertionError):network=None
 if network:run('ip','route','replace',network,'dev','wlan0','table',TABLE)
 run('ip','route','replace','default','dev',DEV,'table',TABLE)
 if network and 'iif wlan0' not in run('ip','rule','show').stdout:
  run('ip','rule','add','priority',PRIORITY,'iif','wlan0','lookup',TABLE)
 run(IPT,'-t','nat','-N',CHAIN,check=False);run(IPT,'-t','nat','-F',CHAIN)
 for protocol in ('udp','tcp'):run(IPT,'-t','nat','-A',CHAIN,'-p',protocol,'--dport','53','-j','REDIRECT','--to-ports','17893')
 rules=[['-t','nat','PREROUTING','-i','wlan0','-j',CHAIN],['-t','filter','FORWARD','-i','wlan0','-o',DEV,'-j','ACCEPT'],['-t','filter','FORWARD','-i',DEV,'-o','wlan0','-j','ACCEPT']] if network else []
 for rule in rules:
  prefix=rule[:2];rest=rule[2:]
  if run(IPT,*prefix,'-C',*rest,check=False).returncode:run(IPT,*prefix,'-I',*rest)
 for table,chain in [('mangle','C3MH_MARK'),('nat','C3MH_OUTDNS'),('filter','C3MH_INPUT')]:
  run(IPT,'-t',table,'-N',chain,check=False);run(IPT,'-t',table,'-F',chain)
 run(IPT,'-t','mangle','-A','C3MH_MARK','-m','owner','--uid-owner',str(CORE_UID),'-j','RETURN')
 for network in ['127.0.0.0/8','192.168.0.0/16','10.0.0.0/8','172.16.0.0/12','169.254.0.0/16','198.18.0.0/15','224.0.0.0/4','255.255.255.255/32']:
  run(IPT,'-t','mangle','-A','C3MH_MARK','-d',network,'-j','RETURN')
 run(IPT,'-t','mangle','-A','C3MH_MARK','-j','MARK','--set-mark','17890')
 run(IPT,'-t','nat','-A','C3MH_OUTDNS','-m','owner','--uid-owner',str(CORE_UID),'-j','RETURN')
 for protocol in ('udp','tcp'):run(IPT,'-t','nat','-A','C3MH_OUTDNS','-p',protocol,'--dport','53','-j','REDIRECT','--to-ports','17893')
 for interface in ('lo','wlan0'):run(IPT,'-t','filter','-A','C3MH_INPUT','-i',interface,'-j','RETURN')
 for protocol in ('udp','tcp'):run(IPT,'-t','filter','-A','C3MH_INPUT','-p',protocol,'--dport','17893','-j','REJECT')
 for table,chain,parent in [('mangle','C3MH_MARK','OUTPUT'),('nat','C3MH_OUTDNS','OUTPUT'),('filter','C3MH_INPUT','INPUT')]:
  if run(IPT,'-t',table,'-C',parent,'-j',chain,check=False).returncode:run(IPT,'-t',table,'-I',parent,'-j',chain)
 rules=json.loads(run('ip','-j','rule').stdout)
 if not any(r.get('priority')==17880 and str(r.get('table'))==TABLE for r in rules):run('ip','rule','add','priority','17880','fwmark','17890/0xffff','lookup',TABLE)
 (ROOT/'gateway_enabled').write_text('1')
def disable():
 (ROOT/'gateway_enabled').unlink(missing_ok=True)
 run('ip','rule','del','priority',PRIORITY,'iif','wlan0','lookup',TABLE,check=False)
 run('ip','rule','del','priority','17880','fwmark','17890/0xffff','lookup',TABLE,check=False)
 for table,chain,parent in [('mangle','C3MH_MARK','OUTPUT'),('nat','C3MH_OUTDNS','OUTPUT'),('filter','C3MH_INPUT','INPUT')]:
  run(IPT,'-t',table,'-D',parent,'-j',chain,check=False);run(IPT,'-t',table,'-F',chain,check=False);run(IPT,'-t',table,'-X',chain,check=False)
 run('ip','route','flush','table',TABLE,check=False)
 for rule in [['-t','nat','PREROUTING','-i','wlan0','-j',CHAIN],['-t','filter','FORWARD','-i','wlan0','-o',DEV,'-j','ACCEPT'],['-t','filter','FORWARD','-i',DEV,'-o','wlan0','-j','ACCEPT']]:run(IPT,*rule[:2],'-D',*rule[2:],check=False)
 run(IPT,'-t','nat','-F',CHAIN,check=False);run(IPT,'-t','nat','-X',CHAIN,check=False)
 run('ip','link','del',DEV,check=False)
 state=ROOT/'gateway_state.json'
 if state.exists():
  pathlib.Path('/proc/sys/net/ipv4/ip_forward').write_text(json.loads(state.read_text())['previous_forward']);state.unlink()
if __name__=='__main__':
 assert os.geteuid()==0 and socket.gethostname()=='comma-4a408d77'
 assert pathlib.Path('/data/params/d/IsOffroad').read_text().strip()=='1'
 assert not pathlib.Path('/data/params/d/IsEngaged').exists() or pathlib.Path('/data/params/d/IsEngaged').read_text().strip()!='1'
 try:
  {'prepare':prepare,'enable':enable,'disable':disable,'startcore':startcore,'stopcore':stopcore}[sys.argv[1]]();print(json.dumps({'ok':True}))
 except Exception as error:
  detail=(error.stderr or '').strip() if isinstance(error,subprocess.CalledProcessError) else str(error)
  # Only local interface/process commands run here; no subscription data is logged.
  labels={'prepare':'创建 TUN','startcore':'启动内核','stopcore':'停止内核','enable':'启用路由','disable':'清理路由'}
  print(json.dumps({'error':labels.get(sys.argv[1],'全局代理')+'失败：'+(detail[:240] or type(error).__name__)},ensure_ascii=False));sys.exit(1)
