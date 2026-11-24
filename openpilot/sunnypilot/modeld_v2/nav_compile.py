"""Compile the navigation ONNX ABI with the repository's installed tinygrad."""
import base64
import pickle

import numpy as np
from tinygrad import Device, Tensor, TinyJit
from tinygrad.dtype import _to_np_dtype


def compile_navigation_policy(runner, metadata, *, device=None):
  device = device or Device.DEFAULT
  required = {'new_img', 'desire', 'traffic_convention', 'action_t', 'nav_map', 'navigation'}
  if not required <= runner.graph_inputs.keys():
    raise ValueError('Incomplete navigation model inputs')
  if any(not spec.shape or any(type(n) is not int or n <= 0 for n in spec.shape) for spec in runner.graph_inputs.values()):
    raise ValueError('Navigation compilation needs concrete positive shapes')
  inputs = {name: Tensor.zeros(*spec.shape, dtype=spec.dtype, device=device).realize()
            for name, spec in runner.graph_inputs.items()}
  predictions = runner(inputs)
  if 'outputs' not in predictions:
    raise ValueError('Navigation model needs the outputs tensor')
  outputs = {name: Tensor.zeros(*value.shape, dtype=value.dtype, device=device).realize()
             for name, value in predictions.items()}

  @TinyJit(prune=True)
  def run(output_buffers, **values):
    result = runner(values)
    # Materialize all predictions before writing outputs that will alias state
    # at runtime. A next-state write must not change another output's input.
    Tensor.realize(*result.values())
    Tensor.realize(*[output_buffers[name].assign(value) for name, value in result.items()])

  expected = {name: value.numpy().copy() for name, value in predictions.items()}
  for _ in range(3):
    run(output_buffers=outputs, **inputs)
    for name, value in outputs.items():
      np.testing.assert_allclose(value.numpy(), expected[name], rtol=1e-3, atol=1e-4)

  def specs(values):
    if any(_to_np_dtype(value.dtype) is None for value in values.values()):
      raise ValueError('Navigation buffer dtype has no numpy representation')
    return {name: (value.shape, np.dtype(_to_np_dtype(value.dtype)).name, value.device) for name, value in values.items()}

  return {'input_specs': specs(inputs), 'output_specs': specs(outputs), 'run': run,
          'metadata': {'output_shapes': {name: value.shape for name, value in outputs.items()},
                       'metadata': {'output_slices': base64.b64encode(pickle.dumps(metadata['output_slices'])).decode()}}}
