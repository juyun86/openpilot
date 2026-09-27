"""Exercise the actual loop's bookkeeping statements without hardware/IPC."""
import ast
from pathlib import Path
from types import SimpleNamespace


def bookkeeping_code():
  tree = ast.parse((Path(__file__).parents[1] / "hardwared.py").read_text())
  function = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "hardware_thread")
  loop = next(n for n in function.body if isinstance(n, ast.While))
  keys = {"GithubRunnerSufficientVoltage", "NetworkMetered", "UptimeOffroad", "UptimeOnroad"}
  nodes = []
  for node in loop.body:
    if (any(isinstance(n, ast.Constant) and isinstance(n.value, str) and n.value in keys for n in ast.walk(node)) or
        isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == "runner_voltage" for t in node.targets)):
      nodes.append(node)
  assert len(nodes) == 4
  return compile(ast.Module(body=nodes, type_ignores=[]), "hardwared bookkeeping", "exec")


class BlockedStorage:
  def __init__(self):
    self.pending = []

  def put(self, key, value, block=False):
    # No persistence completes during this test. A synchronous caller would
    # hang behind the captured Params lock instead of returning to publish.
    assert not block, "bookkeeping attempted to wait for blocked storage"
    self.pending.append((key, value))

  put_bool = put


def test_repeated_status_does_not_queue_duplicate_disk_writes():
  params = BlockedStorage()
  state = {"params": params, "voltage": 12000, "last_runner_voltage": None, "last_network_metered": None,
               "msg": SimpleNamespace(deviceState=SimpleNamespace(networkMetered=False)),
               "count": 1, "DT_HW": 0.5, "uptime_offroad": 1.0, "uptime_onroad": 2.0}
  code = bookkeeping_code()
  for _ in range(100):
    exec(code, state)
  assert params.pending == [("GithubRunnerSufficientVoltage", True), ("NetworkMetered", False)]
  state["voltage"] = None
  state["msg"].deviceState.networkMetered = True
  exec(code, state)
  assert params.pending[-2:] == [("GithubRunnerSufficientVoltage", False), ("NetworkMetered", True)]
  exec(code, state)
  assert len(params.pending) == 4


def test_periodic_accounting_returns_even_when_persistence_is_blocked():
  params = BlockedStorage()
  state = {"params": params, "voltage": 12000, "last_runner_voltage": True, "last_network_metered": False,
               "msg": SimpleNamespace(deviceState=SimpleNamespace(networkMetered=False)),
               "count": 120, "DT_HW": 0.5, "uptime_offroad": 12.0, "uptime_onroad": 345.0}
  exec(bookkeeping_code(), state)
  assert params.pending == [("UptimeOffroad", 12.0), ("UptimeOnroad", 345.0)]
