"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.selfdrive.debug.device_network import device_ip_address
from openpilot.sunnypilot.navassist.settings import SETTING_SPECS, SettingsCache, SettingsStore
from openpilot.system.ui.sunnypilot.widgets.list_view import ListItemSP, toggle_item_sp
from openpilot.system.ui.sunnypilot.widgets.option_control import OptionControlSP
from openpilot.system.ui.widgets.list_view import TextAction
from openpilot.system.ui.widgets.scroller_tici import Scroller
from openpilot.system.ui.widgets import Widget


class NavigationLayout(Widget):
  def __init__(self):
    super().__init__()

    self._store = SettingsStore()
    self._cache = SettingsCache(self._store)
    self._error = ''
    self._controls = {}
    self._address = ''
    items = self._initialize_items()
    self._scroller = Scroller(items, line_separator=True, spacing=0)

  def _initialize_items(self):
    self._info = ListItemSP('导航辅助', action_item=TextAction(lambda: self._address),
                            description=lambda: (self._error or ('已保存至本机。' if self._store.path.exists() else '使用默认设置。')) +
                            '<br>停车时可调整。浏览器打开上方地址，选择“导航日志”独立下载。')
    items = [self._info]
    settings = self._cache.read()
    for key, spec in SETTING_SPECS.items():
      current = getattr(settings, key)
      if isinstance(current, bool):
        item = toggle_item_sp(spec.title, spec.description, initial_state=current,
                              callback=lambda value, name=key: self.put(name, value), enabled=ui_state.is_offroad)
      else:
        action = OptionControlSP(key, spec.minimum, spec.maximum, spec.step, enabled=ui_state.is_offroad,
                                 label_width=230, label_callback=lambda value, spec=spec: (spec.choices[value - spec.minimum] if spec.choices else
                                   f'{value / 10:.1f} s' if spec.unit == '0.1 s' else f'{value} {spec.unit}'), params=self)
        item = ListItemSP(spec.title, description=spec.description, action_item=action, inline=False)
      self._controls[key] = item.action_item
      items.append(item)
    return items

  def get(self, key, return_default=False):
    return getattr(self._cache.read(), key)

  def put(self, key, value, block=False):
    if not ui_state.is_offroad():
      return
    try:
      self._cache.value = self._store.update({key: value})
      self._error = ''
    except (OSError, ValueError) as error:
      self._error = f'保存失败：{error}'

  def _update_state(self):
    super()._update_state()
    settings = self._cache.read()
    if self._cache.error:
      self._error = f'读取失败：{self._cache.error}'
    for key, control in self._controls.items():
      value = getattr(settings, key)
      if isinstance(value, bool):
        control.set_state(value)
      else:
        control.current_value = value

  def _render(self, rect):
    self._scroller.render(rect)

  def show_event(self):
    self._address = f'http://{device_ip_address() or "设备 IP"}:8088'
    self._cache.next_read = 0.0
    self._scroller.show_event()
    self._info.show_description(True)
