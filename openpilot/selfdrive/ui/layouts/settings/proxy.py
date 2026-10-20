import json
import queue
import subprocess
import threading
import time
from pathlib import Path
import pyray as rl
from openpilot.system.ui.lib import application as ui_application
from openpilot.common.params import Params
from openpilot.system.ui.lib.application import gui_app, FontWeight, font_fallback
from openpilot.system.ui.widgets import Widget, DialogResult
from openpilot.system.ui.widgets.button import Button
from openpilot.system.ui.widgets.keyboard import Keyboard
from openpilot.system.ui.lib.font_characters import fallback_font_characters
from openpilot.system.ui.lib.multilang import multilang

MANAGER = '/data/openpilot/openpilot/selfdrive/ui/mihomo_manager.py'
PAIR = '/data/openpilot/openpilot/selfdrive/ui/mihomo_pair.py'

class ProxyLayout(Widget):
  def __init__(self):
    super().__init__()
    ui_application.EXTRA_FONT_CHARS += '败板保备本别并不操测插查超车成持出传此存打大代带当导到的点订动读对二发返方分辅该公功过合和核后候换回或击机集加驾兼剪检件建键将接节仅局开可空理立连链量流络码没面模幕内能盘配屏启前切请取全确热认仍容入软扫尚稍设生失十时识使驶式试是手输所态添贴停同退完网维为未文下先效校新行选验页一已用有于与阅运在载择粘正支知直止置中钟重助转状走组作'
    self._previous_fallback_fonts = list(gui_app._fallback_fonts.values())
    gui_app._fallback_fonts.clear()
    full_font = Path('/data/mihomo/NotoSansCJKsc-Regular.otf')
    if multilang.language == 'zh-CHS' and full_font.exists():
      chars = fallback_font_characters('zh-CHS', ui_application.EXTRA_FONT_CHARS)
      codepoints = sorted(map(ord, chars))
      data = rl.ffi.new('int[]', codepoints)
      font = rl.load_font_ex(str(full_font), 48, rl.ffi.cast('int *', data), len(codepoints))
      rl.gen_texture_mipmaps(font.texture)
      rl.set_texture_filter(font.texture, rl.TextureFilter.TEXTURE_FILTER_TRILINEAR)
      gui_app._fallback_fonts['zh-CHS'] = font
    self._params = Params(); self._queue = queue.Queue(); self._busy = False
    self._status = {}; self._message = '手机连接 C3 热点，点击手机扫码添加订阅'
    self._last = 0.; self._qr = None; self._expires = 0.
    self._back = Button('返回软件', lambda: gui_app.pop_widget(), font_size=42)
    self._keyboard = Keyboard(max_text_size=8192, min_text_size=1)
    self._buttons = [
      Button('输入 / 粘贴订阅', self._edit, font_size=45),
      Button('手机扫码添加', lambda: self._run('pair'), font_size=45),
      Button('读取剪贴板', self._paste, font_size=45),
      Button('启动全局 TUN', lambda: self._run('start'), font_size=45),
      Button('停止代理', lambda: self._run('stop'), font_size=45),
      Button('切换节点', lambda: self._run('next'), font_size=45),
      Button('测试 GitHub', lambda: self._run('test'), font_size=45),
    ]

  def _allowed(self):
    return self._params.get_bool('IsOffroad') and not self._params.get_bool('IsEngaged')

  def _run(self, operation, value=None):
    if self._busy: return
    if operation != 'status' and not self._allowed():
      self._message = '请停车并退出辅助驾驶后操作'; return
    self._busy = True
    if operation != 'status': self._message = '操作中，请稍候…'
    def worker():
      try:
        cmd = ['/usr/local/venv/bin/python3', PAIR, 'create'] if operation == 'pair' else ['/usr/bin/python3', MANAGER, operation]
        result = subprocess.run(cmd, input=value, text=True, capture_output=True, timeout=100)
        lines = [line for line in result.stdout.splitlines() if line.strip().startswith('{')]
        data = json.loads(lines[-1]) if lines else {'error': '组件运行失败，请重新打开本页'}
      except Exception: data = {'error': '操作未完成，请检查网络后重试'}
      self._queue.put((operation, data))
    threading.Thread(target=worker, daemon=True).start()

  def _edit(self, text=''):
    if self._busy: return
    self._keyboard.reset(); self._keyboard.set_title('订阅链接 / 节点链接')
    self._keyboard.set_text(text); self._keyboard.set_callback(self._submit)
    gui_app.push_widget(self._keyboard)

  def _paste(self):
    text = rl.get_clipboard_text()
    if text: self._edit(text)
    else: self._message = 'C3 剪贴板为空；可直接扫码或使用屏幕键盘输入'

  def _submit(self, result):
    if result == DialogResult.CONFIRM: self._run('import', self._keyboard.text.strip())

  def show_event(self):
    super().show_event(); self._last = 0

  def _render(self, rect):
    while not self._queue.empty():
      operation, data = self._queue.get(); self._busy = False
      if data.get('error'): self._message = data['error']
      elif operation == 'pair':
        if self._qr: rl.unload_texture(self._qr)
        self._qr = rl.load_texture('/data/mihomo/pair.png'); self._expires = data['expires']
        self._message = '手机连接 C3 热点后扫码，在手机中粘贴订阅并保存'
      elif operation == 'test': self._message = data['message']
      else:
        self._status = data
        if operation != 'status': self._message = '配置已保存' if operation == 'import' else '操作完成'
    if not self._busy and time.monotonic() - self._last > 5:
      self._last = time.monotonic(); self._run('status')
    rl.draw_rectangle_rec(rect, rl.Color(25, 29, 38, 255))
    font = font_fallback(gui_app.font(FontWeight.NORMAL))
    def text(value, y, size=40, color=rl.WHITE):
      rl.draw_text_ex(font, value, rl.Vector2(rect.x+20, rect.y+y), size, 0, color)
    text('Clash / mihomo', 15, 65)
    self._back.set_enabled(not self._busy)
    self._back.render(rl.Rectangle(rect.x+rect.width-300, rect.y+25, 260, 85))
    state = '全局 TUN 已启动' if self._status.get('gateway') else ('本机代理已启动' if self._status.get('running') else '未启动')
    text(f"状态：{state}     已保存节点：{self._status.get('nodes', 0)}", 105, 43)
    groups = self._status.get('groups') or []
    selected = groups[0]['now'] if groups else '尚未选择'
    text('当前节点：' + selected[:42], 170, 39)
    width = (rect.width-740)/2
    for i, button in enumerate(self._buttons):
      button.set_enabled(not self._busy and self._allowed())
      button.render(rl.Rectangle(rect.x+20+(i%2)*(width+30), rect.y+245+(i//2)*125, width, 100))
    message = self._message
    for i in range(0, min(len(message), 100), 30): text(message[i:i+30], 775+(i//30)*48, 37, rl.Color(130, 210, 255, 255))
    if self._qr and time.time() < self._expires:
      size = min(560., rect.height-360.)
      rl.draw_texture_pro(self._qr, rl.Rectangle(0, 0, self._qr.width, self._qr.height), rl.Rectangle(rect.x+rect.width-640, rect.y+245, size, size), rl.Vector2(0, 0), 0., rl.WHITE)
      text('二维码有效十分钟；请用手机扫码', 930, 34, rl.GRAY)
    text('全局 TUN：C3 和热点设备公网流量走所选节点', 990, 32, rl.GRAY)
