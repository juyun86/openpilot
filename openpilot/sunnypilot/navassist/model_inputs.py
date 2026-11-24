"""Optional model inputs, constrained by the existing navigation/control gates."""
import numpy as np

from openpilot.sunnypilot.navassist.map_input import MAP_MAX_AGE_NS


def model_navigation_inputs(state, *, now_ns, healthy, control_active, turn_active, driver_override):
  if (not healthy or not control_active or driver_override or not state.valid or state.stale
      or not state.routeActive or not state.routeMatched or state.gpsWeak or not state.modelInputValid
      or not 0 <= now_ns - state.publishMonoTime <= 200_000_000
      or not 0 <= now_ns - state.modelMapMonoTime <= MAP_MAX_AGE_NS
      or len(state.modelMap) != 256 * 256 or len(state.modelNavigation) != 47):
    return {}
  navigation = np.array(state.modelNavigation, dtype=np.float32)
  if not np.isfinite(navigation).all() or navigation[-1] < 0:
    return {}
  commands = np.flatnonzero(navigation[:-1])
  if len(commands) != 1 or navigation[commands[0]] != 1:
    return {}
  # Route context can alter the model's path; the map itself must also be
  # withheld while an intended turn has not passed the existing entry gate.
  if commands[0] != 8 and not turn_active:
    return {}
  return {'nav_map': np.frombuffer(state.modelMap, dtype=np.uint8).reshape(256, 256), 'navigation': navigation}


def fill_navigation_inputs(buffers, inputs):
  # Clear every inference. Losing freshness must never retain the previous map.
  for name, size in (('nav_map', 256 * 256), ('navigation', 47)):
    if name not in buffers:
      continue
    target = buffers[name]
    target.fill(0)
    value = inputs.get(name)
    if target.size == size and value is not None and value.size == size:
      target[:] = value.reshape(target.shape)
