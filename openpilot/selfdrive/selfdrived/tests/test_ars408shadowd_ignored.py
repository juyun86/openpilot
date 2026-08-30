import ast
from pathlib import Path


def test_ars408shadowd_is_the_only_new_observational_process_ignore() -> None:
  path = Path(__file__).resolve().parents[1] / "selfdrived.py"
  tree = ast.parse(path.read_text(encoding="utf-8"))
  selfdrive = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "SelfdriveD")
  init = next(node for node in selfdrive.body if isinstance(node, ast.FunctionDef) and node.name == "__init__")
  assignment = next(
    node for node in ast.walk(init) if isinstance(node, ast.Assign) and
    any(isinstance(target, ast.Attribute) and target.attr == "ignored_processes" for target in node.targets)
  )
  ignored = {item.value for item in assignment.value.elts if isinstance(item, ast.Constant)}
  assert ignored == {"mapd", "ars408shadowd"}

  update_events = next(node for node in selfdrive.body if isinstance(node, ast.FunctionDef) and node.name == "update_events")
  assert any(
    isinstance(node, ast.BinOp) and isinstance(node.op, ast.Sub) and
    isinstance(node.right, ast.Attribute) and node.right.attr == "ignored_processes"
    for node in ast.walk(update_events)
  )
