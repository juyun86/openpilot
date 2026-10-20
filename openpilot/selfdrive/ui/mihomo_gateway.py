"""Root helper: route only the C3 hotspot through an existing user-owned TUN."""
import ipaddress,json,os,pathlib,socket,subprocess,sys
ROOT=pathlib.Path('/data/mihomo');DEV='c3mh0';TABLE='17890';PRIORITY='17890';CHAIN='C3MH_DNS';IPT='iptables-legacy'
def run(*args,check=True):
 return subprocess.run(list(args),capture_output=True,text=True,check=check)
def hotspot():
 data=json.loads(run('ip','-j','-4','addr','show','dev','wlan0').stdout)
 addresses=[a for dev in data for a in dev['addr_info'] if a['scope']=='global']
 if not addresses:raise RuntimeError('请先在设置 > 网络中开启 C3 热点')
 a=addresses[0];assert a['local']=='192.168.43.1','请使用 C3 热点模式'
 return a['local'],str(ipaddress.ip_network(f"{a['local']}/{a['prefixlen']}",strict=False))
def prepare():
 hotspot()
 state=ROOT/'gateway_state.json'
 if not state.exists():state.write_text(json.dumps({'previous_forward':pathlib.Path('/proc/sys/net/ipv4/ip_forward').read_text().strip()}))
 if run('ip','link','show',DEV,check=False).returncode:
  run('ip','tuntap','add','dev',DEV,'mode','tun','user','comma')
 run('ip','addr','replace','198.18.0.1/30','dev',DEV);run('ip','link','set',DEV,'up')
 pathlib.Path('/proc/sys/net/ipv4/ip_forward').write_text('1')
def enable():
 _,network=hotspot()
 run('ip','route','replace',network,'dev','wlan0','table',TABLE)
 run('ip','route','replace','default','dev',DEV,'table',TABLE)
 if 'iif wlan0' not in run('ip','rule','show').stdout:
  run('ip','rule','add','priority',PRIORITY,'iif','wlan0','lookup',TABLE)
 run(IPT,'-t','nat','-N',CHAIN,check=False);run(IPT,'-t','nat','-F',CHAIN)
 for protocol in ('udp','tcp'):run(IPT,'-t','nat','-A',CHAIN,'-p',protocol,'--dport','53','-j','REDIRECT','--to-ports','17893')
 rules=[['-t','nat','PREROUTING','-i','wlan0','-j',CHAIN],['-t','filter','FORWARD','-i','wlan0','-o',DEV,'-j','ACCEPT'],['-t','filter','FORWARD','-i',DEV,'-o','wlan0','-j','ACCEPT']]
 for rule in rules:
  prefix=rule[:2];rest=rule[2:]
  if run(IPT,*prefix,'-C',*rest,check=False).returncode:run(IPT,*prefix,'-I',*rest)
 (ROOT/'gateway_enabled').write_text('1')
def disable():
 (ROOT/'gateway_enabled').unlink(missing_ok=True)
 run('ip','rule','del','priority',PRIORITY,'iif','wlan0','lookup',TABLE,check=False)
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
  {'prepare':prepare,'enable':enable,'disable':disable}[sys.argv[1]]();print(json.dumps({'ok':True}))
 except Exception:print(json.dumps({'error':'热点转发设置失败，请确认热点已开启'}));sys.exit(1)
