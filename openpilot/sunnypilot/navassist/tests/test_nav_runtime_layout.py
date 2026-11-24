import numpy as np
import pytest
import base64
import pickle
from tinygrad import Context, Tensor, TinyJit, dtypes

from openpilot.sunnypilot.modeld_v2.nav_runtime import NavigationRuntime, input_view, packed_layout
from openpilot.sunnypilot.modeld_v2.nav_compile import compile_navigation_policy


def test_actual_buffer_views_preserve_dtype_byte_offset_and_aliasing():
  storage = np.arange(16, dtype=np.uint8)
  tensor = Tensor(storage, device='NPY')
  view = input_view(tensor._buffer(), (2, 2), dtypes.uint8, 4)
  np.testing.assert_equal(view.numpy(), [[4, 5], [6, 7]])
  storage[4] = 91
  assert view.numpy()[0, 0] == 91
  # Four float32 values fit in sixteen bytes, not sixty-four: Buffer.view
  # here takes elements, unlike the newer upstream signature taking bytes.
  floats = np.array([1, 2, 3, 4], dtype=np.float32)
  np.testing.assert_equal(input_view(Tensor(floats, device='NPY')._buffer(), (2,), dtypes.float32, 4).numpy(), [2, 3])


def test_new_abi_layout_packs_uint8_map_and_float32_instruction_without_state_copy():
  specs = {'new_img': ((2, 6, 128, 256), 'uint8', 'CPU'), 'nav_map': ((1, 256, 256), 'uint8', 'CPU'),
           'navigation': ((1, 47), 'float32', 'CPU'), 'feature_state': ((1, 20, 32), 'float16', 'CPU')}
  layout, size = packed_layout(specs, {'feature_state': 'next_feature_state'})
  assert set(layout) == {'tfm', 'nav_map', 'navigation'}
  assert size == 65920
  assert all(offset % 128 == 0 for _, _, offset in layout.values())
  packed = np.zeros(size, np.uint8)
  shape, dtype, offset = layout['nav_map']
  image = np.ndarray(shape, dtype=dtype, buffer=packed, offset=offset)
  image.fill(128)
  shape, dtype, offset = layout['navigation']
  navigation = np.ndarray(shape, dtype=dtype, buffer=packed, offset=offset)
  navigation[0, 10], navigation[0, -1] = 1, .8
  assert np.all(image == 128) and navigation[0, 10] == 1
  assert dtype == np.float32


def test_nonconcrete_model_shape_is_rejected_before_buffer_allocation():
  with pytest.raises(ValueError):
    packed_layout({'navigation': ((1, 0), 'float32', 'CPU')}, {})


def test_actual_tinyjit_new_format_recurrent_outputs_and_navigation_clearing():
  device = 'PYTHON'  # Real tinygrad backend; no platform hardware import or GPU required.

  @TinyJit
  def warp(input_frame, M_inv):
    return (input_frame[:, :1].cast(dtypes.float32) + M_inv[:, 0, 0].reshape(2, 1)).realize()

  @TinyJit
  def policy(output_buffers, **inputs):
    result = (inputs['state'] + inputs['new_img'][0, 0] + inputs['navigation'][0, 46]
              + inputs['nav_map'][0, 128, 128].cast(dtypes.float32) / 128)
    output_buffers['outputs'].assign(result).realize()
    output_buffers['next_state'].assign(inputs['state'] + 1).realize()

  specs = {'new_img': ((2, 1), 'float32', device), 'state': ((1, 1), 'float32', device),
           'desire': ((1, 8), 'float32', device), 'traffic_convention': ((1, 2), 'float32', device),
           'action_t': ((1, 2), 'float32', device), 'navigation': ((1, 47), 'float32', device),
           'nav_map': ((1, 256, 256), 'uint8', device)}
  tensors = {name: Tensor(np.zeros(shape, dtype=dtype), device=device).realize() for name, (shape, dtype, _) in specs.items()}
  output_specs = {'outputs': ((1, 1), 'float32', device), 'next_state': ((1, 1), 'float32', device)}
  outputs = {name: Tensor(np.zeros(shape, dtype=dtype), device=device).realize() for name, (shape, dtype, _) in output_specs.items()}
  frame = Tensor(np.zeros((2, 16), np.uint8), device=device).realize()
  tfm = Tensor(np.zeros((2, 3, 3), np.float32), device=device).realize()
  for _ in range(3):
    warp(input_frame=frame, M_inv=tfm)
    policy(output_buffers=outputs, **tensors)
  jits = {'input_specs': specs, 'output_specs': output_specs, 'run': policy,
          'metadata': {'output_shapes': {'outputs': (1, 1), 'next_state': (1, 1)},
                       'metadata': {'output_slices': base64.b64encode(pickle.dumps({})).decode()}}}
  # The policy uses the bundle's input_specs ABI. The separate official warp
  # artifact is a normal pickle, not the policy's out-of-band file format.
  runtime = NavigationRuntime(pickle.loads(pickle.dumps(jits)), pickle.loads(pickle.dumps({'run': warp}))['run'], 16)
  navigation = np.zeros(47, np.float32)
  navigation[10], navigation[-1] = 1, .8
  inputs = {'desire': np.zeros(8, np.float32), 'traffic_convention': np.zeros(2, np.float32),
            'action_t': np.zeros(2, np.float32), 'nav_map': np.full((256, 256), 128, np.uint8), 'navigation': navigation}
  frames = {'img': np.zeros(16, np.uint8), 'big_img': np.zeros(16, np.uint8)}
  transforms = {'img': np.eye(3, dtype=np.float32), 'big_img': np.eye(3, dtype=np.float32)}
  np.testing.assert_allclose(runtime.run(frames, transforms, inputs), [2.8])
  np.testing.assert_allclose(runtime.run(frames, transforms, inputs), [3.8])
  inputs.pop('nav_map')
  inputs.pop('navigation')
  np.testing.assert_allclose(runtime.run(frames, transforms, inputs), [3.])
  runtime.reset()
  np.testing.assert_allclose(runtime.run(frames, transforms, inputs), [1.])


def test_real_onnx_compiler_pickle_and_runtime_with_recurrent_aliases(tmp_path):
  onnx = pytest.importorskip('onnx')
  from onnx import TensorProto, helper, numpy_helper
  from tinygrad.nn.onnx import OnnxRunner
  shapes = {'new_img': (2, 6, 1, 1), 'state': (1, 1), 'desire': (1, 8), 'traffic_convention': (1, 2),
            'action_t': (1, 2), 'navigation': (1, 47), 'nav_map': (1, 256, 256)}
  declarations = [helper.make_tensor_value_info(name, TensorProto.UINT8 if name == 'nav_map' else TensorProto.FLOAT, shape)
                  for name, shape in shapes.items()]
  nodes = [helper.make_node('ReduceSum', ['new_img'], ['image_sum'], keepdims=0),
           helper.make_node('ReduceSum', ['navigation'], ['navigation_sum'], keepdims=0),
           helper.make_node('Reshape', ['nav_map', 'flat_shape'], ['flat_map']),
           helper.make_node('Gather', ['flat_map', 'index'], ['pixel']),
           helper.make_node('Cast', ['pixel'], ['float_pixel'], to=TensorProto.FLOAT),
           helper.make_node('Div', ['float_pixel', 'scale'], ['map_sum']),
           helper.make_node('Add', ['state', 'image_sum'], ['image_state']),
           helper.make_node('Add', ['image_state', 'navigation_sum'], ['navigation_state']),
           helper.make_node('Add', ['navigation_state', 'map_sum'], ['outputs']),
           helper.make_node('Add', ['state', 'one'], ['next_state'])]
  constants = [numpy_helper.from_array(array, name) for name, array in
               [('flat_shape', np.array([-1], np.int64)), ('index', np.array([0], np.int64)),
                ('scale', np.array([128], np.float32)), ('one', np.array([[1]], np.float32))]]
  graph = helper.make_graph(nodes, 'nav-abi-test', declarations,
    [helper.make_tensor_value_info(name, TensorProto.FLOAT, [1, 1]) for name in ('outputs', 'next_state')], constants)
  path = tmp_path / 'navigation.onnx'
  onnx.save(helper.make_model(graph, opset_imports=[helper.make_opsetid('', 13)]), path)
  with Context(DEV='PYTHON'):
    compiled = compile_navigation_policy(OnnxRunner(path), {'output_slices': {}}, device='PYTHON')

  @TinyJit
  def warp(input_frame, M_inv):
    return (input_frame[:, :6].cast(dtypes.float32).reshape(2, 6, 1, 1)
            + M_inv[:, 0, 0].reshape(2, 1, 1, 1)).realize()

  frame = Tensor(np.zeros((2, 16), np.uint8), device='PYTHON').realize()
  tfm = Tensor(np.zeros((2, 3, 3), np.float32), device='PYTHON').realize()
  for _ in range(3):
    warp(input_frame=frame, M_inv=tfm)
  runtime = NavigationRuntime(pickle.loads(pickle.dumps(compiled)), pickle.loads(pickle.dumps(warp)), 16)
  navigation = np.zeros(47, np.float32)
  navigation[10], navigation[-1] = 1, .8
  inputs = {'desire': np.zeros(8, np.float32), 'traffic_convention': np.zeros(2, np.float32),
            'action_t': np.zeros(2, np.float32), 'nav_map': np.full((256, 256), 128, np.uint8), 'navigation': navigation}
  frames = {'img': np.zeros(16, np.uint8), 'big_img': np.zeros(16, np.uint8)}
  transforms = {name: np.eye(3, dtype=np.float32) for name in frames}
  np.testing.assert_allclose(runtime.run(frames, transforms, inputs), [14.8])
  np.testing.assert_allclose(runtime.run(frames, transforms, inputs), [15.8])
  inputs.pop('nav_map')
  inputs.pop('navigation')
  np.testing.assert_allclose(runtime.run(frames, transforms, inputs), [14.])
  runtime.reset()
  np.testing.assert_allclose(runtime.run(frames, transforms, inputs), [12.])
