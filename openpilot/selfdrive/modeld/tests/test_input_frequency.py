"""Check the real modeld subscription cadence without loading a GPU model."""
import ast
from pathlib import Path
from types import SimpleNamespace as NS
from collections import deque

import pytest


ROOT = Path(__file__).resolve().parents[3]


@pytest.mark.parametrize('adapter', ['selfdrive/modeld/modeld.py', 'sunnypilot/modeld_v2/modeld.py'])
@pytest.mark.parametrize('receive_hz,healthy', [(20., True), (5., False)])
def test_modeld_accepts_conflated_inputs_but_rejects_slow_stream(adapter, receive_hz, healthy):
  env = {'deque': deque}
  # Load the actual frequency checker, avoiding native socket initialization.
  for path, name in [('common/utils.py', 'MovingAverage'), ('cereal/messaging/__init__.py', 'FrequencyTracker')]:
    tree = ast.parse((ROOT / path).read_text(encoding='utf-8'))
    node = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == name)
    exec(compile(ast.Module(body=[node], type_ignores=[]), path, 'exec'), env)
  tree = ast.parse((ROOT / adapter).read_text(encoding='utf-8'))
  call = next(n for n in ast.walk(tree) if isinstance(n, ast.Call) and isinstance(n.func, ast.Name) and n.func.id == 'SubMaster')
  constants = NS(MODEL_FREQ=20., MODEL_RUN_FREQ=20.)
  env.update(model=NS(constants=constants), ModelConstants=constants,
             SubMaster=lambda services, frequency=None: env['FrequencyTracker'](100., frequency or 100., False))
  tracker = eval(compile(ast.Expression(body=call), adapter, 'eval'), env)
  for i in range(200):
    tracker.record_recv_time(10. + i / receive_hz)
  assert tracker.valid is healthy
