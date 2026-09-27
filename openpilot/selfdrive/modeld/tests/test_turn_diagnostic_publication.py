"""Check the model-frame contract of both modelDataV2SP publishers."""
import ast
import unittest
from pathlib import Path
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[3]
PUBLISHERS = ('selfdrive/modeld/modeld.py', 'sunnypilot/modeld_v2/modeld.py')


def publication_statements(path):
  tree = ast.parse((ROOT / path).read_text(encoding='utf-8'))
  for node in ast.walk(tree):
    body = getattr(node, 'body', ())
    if not isinstance(body, list):
      continue
    for index, statement in enumerate(body):
      if (isinstance(statement, ast.Assign) and len(statement.targets) == 1
          and ast.unparse(statement.targets[0]) == 'mdv2sp_send.modelDataV2SP.laneTurnDirection'):
        return compile(ast.Module(body=body[index:index + 3], type_ignores=[]), path, 'exec')
  raise AssertionError(f'{path}: turn diagnostics publisher not found')


class TurnDiagnosticPublicationTest(unittest.TestCase):
  def test_frame_validity_and_rejection_reasons(self):
    for path in PUBLISHERS:
      publish = publication_statements(path)
      for model_valid in (False, True):
        for oem_present in (False, True):
          with self.subTest(path=path, model_valid=model_valid, oem_present=oem_present):
            output = SimpleNamespace(laneTurnDirection='none', turnEntryModelMonoTime=0,
                                     turnEntryInputReason='', turnEntryLeftReason='',
                                     turnEntryRightReason='', turnDecisionReason='')
            sp_message = SimpleNamespace(valid=False, modelDataV2SP=output)
            context = {
              'mdv2sp_send': sp_message,
              'modelv2_send': SimpleNamespace(valid=model_valid),
              'meta_main': SimpleNamespace(timestamp_eof=123456789),
              'turn_entry': SimpleNamespace(input_reason='cpModelObserved',
                                            detail_reasons=('boundaryHeld/cpNotTurn', 'openingUnconfirmed/cpNotTurn')),
              'DH': SimpleNamespace(lane_turn_direction='turnRight', turn_decision_reason='turnEntryPending'),
              'oem_gate': object() if oem_present else None,
            }
            exec(publish, context)
            self.assertEqual(sp_message.valid, model_valid)
            self.assertEqual(output.laneTurnDirection, 'turnRight')
            self.assertEqual(output.turnEntryModelMonoTime, 123456789 if oem_present else 0)
            self.assertEqual(output.turnEntryRightReason,
                             'openingUnconfirmed/cpNotTurn' if oem_present else '')
            self.assertEqual(output.turnDecisionReason, 'turnEntryPending' if oem_present else '')


if __name__ == '__main__':
  unittest.main()
