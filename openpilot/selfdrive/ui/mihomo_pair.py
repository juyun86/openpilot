"""Ten-minute phone pairing form on the hotspot interface only."""
import hashlib,hmac,http.server,json,os,pathlib,secrets,subprocess,sys,time,urllib.parse
ROOT=pathlib.Path('/data/mihomo');MANAGER='/data/openpilot/openpilot/selfdrive/ui/mihomo_manager.py'
PAGE='''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>C3 添加订阅</title><style>body{font:17px system-ui;background:#111827;color:#f3f4f6;max-width:600px;margin:32px auto;padding:20px}textarea{box-sizing:border-box;width:100%;height:160px;background:#1f2937;color:white;border:1px solid #64748b;border-radius:12px;padding:12px;font:16px system-ui}button{width:100%;padding:16px;background:#2563eb;border:0;border-radius:12px;color:white;font-size:18px;margin:18px 0}p{line-height:1.7;color:#cbd5e1}#msg{white-space:pre-wrap}</style><h1>C3 · Clash 订阅</h1><p>粘贴订阅链接、节点链接，或选择 Clash YAML 文件。保存后回到 C3 屏幕，点击“启动代理热点”。</p><textarea id="value" placeholder="https://… 或节点链接 / YAML" autocomplete="off" autocapitalize="off" spellcheck="false"></textarea><p><input id="file" type="file" accept=".yaml,.yml,.txt"></p><button id="save">保存到 C3</button><p id="msg">仅用于当前热点，配对页面十分钟后失效。</p><script>const file=document.getElementById("file"),input=document.getElementById("value"),save=document.getElementById("save"),msg=document.getElementById("msg");file.onchange=async()=>{if(file.files[0])input.value=await file.files[0].text()};save.onclick=async()=>{save.disabled=true;msg.textContent='正在校验并保存，请稍候…';try{let r=await fetch(location.pathname,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({value:input.value})});let d=await r.json();msg.textContent=d.error||'已保存到 C3，请回到 C3 点击启动。';if(!d.error)input.value=''}catch(e){msg.textContent='连接失败，请确认手机仍连接 C3 热点'}finally{save.disabled=false}};</script></html>'''
def serve(bind,token,expires):
 class Handler(http.server.BaseHTTPRequestHandler):
  def log_message(self,*args):pass
  def allowed(self):
   return time.time()<expires and hmac.compare_digest(self.path,'/p/'+token)
  def send(self,status,body,kind='application/json; charset=utf-8'):
   data=body.encode();self.send_response(status);self.send_header('Content-Type',kind);self.send_header('Content-Length',str(len(data)));self.send_header('Cache-Control','no-store');self.send_header('Referrer-Policy','no-referrer');self.send_header('X-Content-Type-Options','nosniff');self.end_headers();self.wfile.write(data)
  def do_GET(self):
   if not self.allowed():return self.send(403,'{"error":"配对已失效，请在 C3 重新生成二维码"}')
   self.send(200,PAGE,'text/html; charset=utf-8')
  def do_POST(self):
   if not self.allowed():return self.send(403,'{"error":"配对已失效"}')
   if self.headers.get('Origin') not in (None,'http://'+self.headers.get('Host','')):return self.send(403,'{}')
   try:
    size=int(self.headers.get('Content-Length','0'));assert 0<size<=2000000
    value=json.loads(self.rfile.read(size))['value'];assert isinstance(value,str)
    result=subprocess.run(['/usr/bin/python3',MANAGER,'import'],input=value,text=True,capture_output=True,timeout=100)
    data=json.loads(result.stdout.splitlines()[-1]);self.send(200,json.dumps({'error':data['error']} if 'error' in data else {'ok':True},ensure_ascii=False))
   except Exception:self.send(400,'{"error":"导入失败，请检查内容后重试"}')
 server=http.server.ThreadingHTTPServer((bind,17894),Handler);server.timeout=1
 while time.time()<expires:
  try:
   if pathlib.Path('/data/params/d/IsOffroad').read_text().strip()!='1':break
  except OSError:break
  server.handle_request()
 server.server_close()

def create():
 import numpy as np,struct,zlib
 sys.path.insert(0,str(ROOT/'vendor'));import zxingcpp
 addresses=json.loads(subprocess.check_output(['ip','-j','-4','addr','show','wlan0'],text=True))
 candidates=[a['local'] for d in addresses for a in d['addr_info'] if a['scope']=='global']
 assert '192.168.43.1' in candidates,'请先在设置 > 网络中开启 C3 热点，并将手机连接该热点'
 state_path=ROOT/'pair.json'
 if state_path.exists():
  old=json.loads(state_path.read_text())
  try:
   if b'mihomo_pair.py' in pathlib.Path(f"/proc/{old['pid']}/cmdline").read_bytes():os.kill(old['pid'],15)
  except (OSError,KeyError):pass
 token=secrets.token_urlsafe(24);expires=time.time()+600;url='http://192.168.43.1:17894/p/'+token
 barcode=zxingcpp.create_barcode(url,zxingcpp.BarcodeFormat.QRCode)
 image=np.asarray(zxingcpp.write_barcode_to_image(barcode,scale=8,add_hrt=False))
 if image.ndim==3:image=image[:,:,0]
 data=b''.join(b'\0'+row.astype(np.uint8).tobytes() for row in image)
 def chunk(kind,payload):return struct.pack('!I',len(payload))+kind+payload+struct.pack('!I',zlib.crc32(kind+payload)&0xffffffff)
 png=b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('!IIBBBBB',image.shape[1],image.shape[0],8,0,0,0,0))+chunk(b'IDAT',zlib.compress(data))+chunk(b'IEND',b'')
 (ROOT/'pair.png').write_bytes(png)
 state={'bind':'192.168.43.1','token':token,'expires':expires}
 state_path.write_text(json.dumps(state));state_path.chmod(0o600)
 with (ROOT/'pair.log').open('ab') as log:
  process=subprocess.Popen(['/usr/bin/python3',__file__,'serve'],stdout=log,stderr=log,start_new_session=True)
 state['pid']=process.pid;state_path.write_text(json.dumps(state))
 return {'pair':True,'expires':expires}
if __name__=='__main__':
 try:
  if sys.argv[1]=='serve':
   state=json.loads((ROOT/'pair.json').read_text());serve(state['bind'],state['token'],state['expires'])
  else:print(json.dumps(create(),ensure_ascii=False))
 except Exception as error:print(json.dumps({'error':str(error) if isinstance(error,AssertionError) else '配对启动失败，请确认手机与 C3 同一热点'},ensure_ascii=False));sys.exit(1)
