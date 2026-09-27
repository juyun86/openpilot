"""Small persistent navigation policy shared by the device UI and adapters."""
from dataclasses import asdict, dataclass
import json
import os
from pathlib import Path
import tempfile
import time



@dataclass(frozen=True)
class NavAssistSettings:
  enabled: bool = True
  lane_change_enabled: bool = True
  lane_change_buzzer_enabled: bool = True
  efficiency_lane_change_enabled: bool = True
  turn_signal_enabled: bool = True
  turn_slowdown_enabled: bool = True
  turn_lane_lookahead_m: int = 1000
  exit_lane_lookahead_m: int = 2000
  signal_lead_time_s: int = 8
  turn_speed_kph: int = 18
  max_lane_changes: int = 5
  logging_enabled: bool = True
  overtake_mode: int = 2
  overtake_min_distance_m: int = 12
  overtake_max_distance_m: int = 80
  overtake_closing_kph: int = 3
  overtake_time_gap_tenths: int = 30
  overtake_cruise_percent: int = 90
  overtake_lead_min_kph: int = 35
  overtake_stable_ms: int = 800
  overtake_success_cooldown_s: int = 10
  overtake_failed_cooldown_s: int = 4
  overtake_allow_right: bool = True
  overtake_prefer_left: bool = True


@dataclass(frozen=True)
class SettingSpec:
  title: str
  description: str
  minimum: int = 0
  maximum: int = 0
  step: int = 1
  unit: str = ''
  choices: tuple[str, ...] = ()


SETTING_SPECS = {
  'efficiency_lane_change_enabled': SettingSpec('高速通行收益自动变道', 'ARS408 检到本车道慢车且邻道前方无目标或有足够间隙时，请求原有 SP 变道；跨线许可由视觉或 0x399 提供，盲区仍会阻止。'),
  'enabled': SettingSpec('导航联动', '关闭后仍接收和显示手机导航，不发起导航变道、转弯灯或转弯减速。'),
  'lane_change_enabled': SettingSpec('导航自动靠边变道', '按导航目标请求 SP 原有变道功能；变道灯随此开关，仍执行现有标线、路缘和盲区判断。'),
  'lane_change_buzzer_enabled': SettingSpec('自动变道蜂鸣提示', '导航靠边或效率超车实际开始变道时，C3 蜂鸣器短鸣一次；打灯等待阶段不鸣响。'),
  'turn_signal_enabled': SettingSpec('转弯提前打灯', '控制路口转弯的灯光请求；不影响自动变道需要的转向灯。'),
  'turn_slowdown_enabled': SettingSpec('导航转弯减速', '通过公共速度目标支持三套纵向；普通靠边变道本身不触发减速。'),
  'turn_lane_lookahead_m': SettingSpec('普通路口靠边提前距离', '进入此距离后才考虑为左转或右转靠边。', 100, 1500, 50, 'm'),
  'exit_lane_lookahead_m': SettingSpec('出口与匝道靠边提前距离', '进入此距离后才考虑为出口或匝道靠边。', 300, 3000, 100, 'm'),
  'signal_lead_time_s': SettingSpec('转弯打灯提前时间', '按当前车速换算距离，保留现有 40–250 米范围和距离余量。', 3, 12, 1, 's'),
  'turn_speed_kph': SettingSpec('普通转弯目标速度', '适用于普通左转/右转；急转、掉头与匝道保留各自目标。具体减速由所选纵向执行。', 10, 30, 1, 'km/h'),
  'max_lane_changes': SettingSpec('单路口连续变道次数', '同一导航事件最多请求的已完成变道次数。每次仍需重新确认车道条件。', 1, 5, 1, '次'),
  'logging_enabled': SettingSpec('独立导航日志', '单独记录导航、车道、灯光和控制上下文；网页独立下载，自动轮转且最多占用 128 MiB。'),
}

# Local amapnavi/MainActivityLaneDecision.kt economic presets. C3 safety gates stay fixed.
OVERTAKE_PRESETS = {
  1: dict(overtake_min_distance_m=8, overtake_max_distance_m=100, overtake_closing_kph=1,
          overtake_time_gap_tenths=40, overtake_cruise_percent=95, overtake_lead_min_kph=25,
          overtake_stable_ms=400, overtake_success_cooldown_s=6,
          overtake_failed_cooldown_s=3, overtake_allow_right=True, overtake_prefer_left=False),
  2: dict(overtake_min_distance_m=12, overtake_max_distance_m=80, overtake_closing_kph=3,
          overtake_time_gap_tenths=30, overtake_cruise_percent=90, overtake_lead_min_kph=35,
          overtake_stable_ms=800, overtake_success_cooldown_s=10,
          overtake_failed_cooldown_s=4, overtake_allow_right=True, overtake_prefer_left=True),
  3: dict(overtake_min_distance_m=20, overtake_max_distance_m=60, overtake_closing_kph=8,
          overtake_time_gap_tenths=20, overtake_cruise_percent=80, overtake_lead_min_kph=45,
          overtake_stable_ms=1200, overtake_success_cooldown_s=15,
          overtake_failed_cooldown_s=6, overtake_allow_right=False, overtake_prefer_left=True),
}
_overtake_specs = {
  'overtake_mode': SettingSpec('超车模式', '选择预设会同步下方参数；单独调整后显示自定义。仅高速与高架快速路，本车至少 60 km/h；距离、盲区和跨线条件仍生效。', 1, 4, choices=('激进', '常规', '保守', '自定义')),
  'overtake_min_distance_m': SettingSpec('超车触发最小前车距离', '前车过近时不发起超车。', 8, 40, 1, 'm'),
  'overtake_max_distance_m': SettingSpec('超车触发最大前车距离', '只评估此范围内的本车道前车。', 40, 100, 5, 'm'),
  'overtake_closing_kph': SettingSpec('前车压速差', '前车比本车慢超过此值，或满足跟车时距、巡航速度比例之一时，评估收益。', 1, 15, 1, 'km/h'),
  'overtake_time_gap_tenths': SettingSpec('超车触发跟车时距', '本车与前车的距离除以本车速度；不改变目标车道安全距离。', 10, 40, 5, '0.1 s'),
  'overtake_cruise_percent': SettingSpec('巡航速度触发比例', '本车速度低于设定巡航速度的此比例时，评估超车收益。', 70, 95, 5, '%'),
  'overtake_lead_min_kph': SettingSpec('超车前车最低速度', '前车低于此速度时按拥堵处理，不发起效率超车。', 25, 60, 5, 'km/h'),
  'overtake_stable_ms': SettingSpec('超车收益确认时间', '满足收益条件后持续确认的时间。', 400, 2000, 100, 'ms'),
  'overtake_success_cooldown_s': SettingSpec('超车完成后间隔', '确认完成一次变道后，再次评估的最短间隔。', 6, 60, 1, 's'),
  'overtake_failed_cooldown_s': SettingSpec('超车退出后间隔', '失败或取消后再次评估的最短间隔；仍须模型退出、实体灯关闭及当前变道条件有效。', 3, 30, 1, 's'),
  'overtake_allow_right': SettingSpec('允许右侧超车', '开启后可评估右侧车道，仍需满足全部变道条件。'),
  'overtake_prefer_left': SettingSpec('超车优先左侧', '两侧均满足条件时优先左侧；关闭则选预计速度收益较大的一侧。'),
}
# Keep the mode and its values adjacent to the existing enable switch in the C3 UI.
SETTING_SPECS = dict(list(SETTING_SPECS.items())[:1] + list(_overtake_specs.items()) + list(SETTING_SPECS.items())[1:])



def settings_path() -> Path:
  from openpilot.common.hardware import PC
  from openpilot.common.hardware.hw import Paths

  root = Path(Paths.comma_home()) if PC else Path('/data')
  return root / 'navassist/settings.json'


def atomic_json(path: Path, value: dict) -> None:
  path.parent.mkdir(parents=True, exist_ok=True)
  fd, temporary = tempfile.mkstemp(prefix=f'.{path.name}.', dir=path.parent)
  try:
    with os.fdopen(fd, 'w') as output:
      json.dump(value, output, ensure_ascii=False, allow_nan=False)
      output.flush()
      os.fsync(output.fileno())
    os.replace(temporary, path)
  finally:
    if os.path.exists(temporary):
      os.unlink(temporary)


def parse_settings(values: dict) -> NavAssistSettings:
  if not isinstance(values, dict):
    raise ValueError('导航设置包含未知字段')
  values = {key: value for key, value in values.items() if key != 'overtake_gain_kph'}  # Legacy persisted setting.
  if set(values) - SETTING_SPECS.keys():
    raise ValueError('导航设置包含未知字段')
  defaults = asdict(NavAssistSettings())
  for key, value in values.items():
    spec = SETTING_SPECS[key]
    if isinstance(defaults[key], bool):
      if not isinstance(value, bool):
        raise ValueError(f'{spec.title}必须为开关值')
    elif (isinstance(value, bool) or not isinstance(value, int)
          or not spec.minimum <= value <= spec.maximum or (value - spec.minimum) % spec.step):
      raise ValueError(f'{spec.title}应为 {spec.minimum}–{spec.maximum}，步长 {spec.step}')
  settings = NavAssistSettings(**(defaults | values))
  if settings.overtake_min_distance_m >= settings.overtake_max_distance_m:
    raise ValueError('超车最小前车距离必须小于最大距离')
  return settings


class SettingsStore:
  def __init__(self, path: Path | None = None):
    self.path = path if path is not None else settings_path()

  def read(self) -> NavAssistSettings:
    try:
      if self.path.stat().st_size > 8192:
        raise ValueError('导航设置文件过大')
      return parse_settings(json.loads(self.path.read_text()))
    except FileNotFoundError:
      return NavAssistSettings()

  def update(self, changes: dict) -> NavAssistSettings:
    parse_settings(changes)
    changes = dict(changes)
    mode = changes.get('overtake_mode')
    if mode in OVERTAKE_PRESETS:
      changes = changes | OVERTAKE_PRESETS[mode]
    elif set(changes) & OVERTAKE_PRESETS[2].keys():
      changes['overtake_mode'] = 4
    settings = parse_settings(asdict(self.read()) | changes)
    atomic_json(self.path, asdict(settings))
    return settings


class SettingsCache:
  def __init__(self, store: SettingsStore | None = None):
    self.store = store if store is not None else SettingsStore()
    self.value = NavAssistSettings()
    self.next_read = 0.0
    self.error: str | None = None

  def read(self) -> NavAssistSettings:
    now = time.monotonic()
    if now >= self.next_read:
      self.next_read = now + 1.0
      try:
        self.value = self.store.read()
        self.error = None
      except (OSError, ValueError) as error:
        self.error = str(error)
    return self.value
