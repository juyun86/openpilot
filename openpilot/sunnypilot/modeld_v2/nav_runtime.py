"""comma navigation-model input_specs ABI, using this fork's existing tinygrad.

Adapted from commaai/openpilot a03bb1d4 ModelState. The legacy loader remains
unchanged; the caller selects this runtime only for an input_specs bundle.
"""
import base64
import math
import pickle

import numpy as np

from tinygrad.device import Buffer
from tinygrad.dtype import _from_np_dtype
from tinygrad.engine.realize import lower_and_compile
from tinygrad.tensor import Tensor
from tinygrad.uop.ops import UOp

from openpilot.sunnypilot.navassist.model_inputs import fill_navigation_inputs


def input_view(buffer: Buffer, shape, dtype, offset):
  # This fork's Buffer.view takes element count, dtype, and a byte offset.
  view = buffer.view(math.prod(shape), dtype, offset).ensure_allocated()
  return Tensor(UOp.from_buffer(view)).reshape(shape)


def packed_layout(input_specs, state_pairs):
  shapes = {'tfm': ((2, 3, 3), np.dtype(np.float32))}
  shapes.update({name: (tuple(shape), np.dtype(dtype)) for name, (shape, dtype, _) in input_specs.items()
                 if name not in state_pairs and name != 'new_img'})
  layout, offset = {}, 0
  for name, (shape, dtype) in shapes.items():
    if not shape or any(type(n) is not int or n <= 0 for n in shape):
      raise ValueError('Navigation model needs concrete positive input shapes')
    layout[name] = shape, dtype, offset
    size = math.prod(shape) * dtype.itemsize
    offset += (size + 127) // 128 * 128
  return layout, offset


class NavigationRuntime:
  def __init__(self, jits, warp, frame_copy_size):
    self.model_device = jits['input_specs']['new_img'][2]
    self.input_shapes = {name: (tuple(shape), np.dtype(dtype)) for name, (shape, dtype, _) in jits['input_specs'].items()}
    self.state_pairs = {name: f'next_{name}' for name in self.input_shapes if f'next_{name}' in jits['metadata']['output_shapes']}
    self.output_slices = pickle.loads(base64.b64decode(jits['metadata']['metadata']['output_slices']))
    self.frame_copy_size = frame_copy_size
    self.run_warp, self.run_model = warp, jits['run']
    for run in (self.run_warp, self.run_model):
      run.captured._linear = lower_and_compile(run.captured._linear)
    self.input_queues = {name: Tensor(np.zeros(shape, dtype=dtype), device=self.model_device).realize()
                         for name, (shape, dtype) in self.input_shapes.items() if name in self.state_pairs}
    layout, npy_size = packed_layout(jits['input_specs'], self.state_pairs)
    self.packed_input = np.zeros(npy_size + 2 * frame_copy_size, dtype=np.uint8)
    self.input_host = Tensor(self.packed_input, device='NPY')._buffer()
    self.input_device = Tensor(self.packed_input, device=self.model_device)._buffer()
    self.npy = {}
    for name, (shape, dtype, offset) in layout.items():
      self.npy[name] = np.ndarray(shape, dtype=dtype, buffer=self.packed_input, offset=offset)
      self.input_queues[name] = input_view(self.input_device, shape, _from_np_dtype(dtype), offset)
    self.frames = self.packed_input[npy_size:].reshape(2, frame_copy_size)
    self.warp_inputs = {'input_frame': input_view(self.input_device, self.frames.shape, _from_np_dtype(self.frames.dtype), npy_size),
                        'M_inv': self.input_queues.pop('tfm')}
    self.outputs = {name: Tensor(np.zeros(shape, dtype=dtype), device=device).realize()
                    for name, (shape, dtype, device) in jits['output_specs'].items()}
    for name, next_name in self.state_pairs.items():
      state = self.input_queues[name]
      shape, dtype, device = jits['output_specs'][next_name]
      if tuple(shape) != state.shape or np.dtype(dtype) != self.input_shapes[name][1] or device != self.model_device:
        raise ValueError('Navigation recurrent state output does not match its input')
      self.outputs[next_name] = input_view(state._buffer(), state.shape, state.dtype, 0)
    self.prev_desire = np.zeros(self.npy['desire'].shape, dtype=np.float32)

  def run(self, bufs, transforms, inputs, after_enqueue=None):
    for i, name in enumerate(('img', 'big_img')):
      data = bufs[name].data if hasattr(bufs[name], 'data') else bufs[name]
      np.copyto(self.frames[i], np.frombuffer(data, dtype=np.uint8, count=self.frame_copy_size))
      self.npy['tfm'][i] = transforms[name]
    desire = np.array(inputs.get('desire', inputs.get('desire_pulse')), dtype=np.float32, copy=True)
    desire[..., 0] = 0
    self.npy['desire'][:] = np.where(desire - self.prev_desire > .99, desire, 0)
    self.prev_desire[:] = desire
    for name in ('traffic_convention', 'action_t'):
      self.npy[name][:] = inputs[name]
    fill_navigation_inputs(self.npy, inputs)
    self.input_device.copy_from(self.input_host)
    self.input_queues['new_img'] = self.run_warp(**self.warp_inputs)
    self.run_model(output_buffers=self.outputs, **self.input_queues)
    if after_enqueue is not None:
      after_enqueue()
    return self.outputs['outputs'].numpy()[0]

  def reset(self):
    self.packed_input.fill(0)
    for name in self.state_pairs:
      self.input_queues[name].assign(0).realize()
    self.prev_desire.fill(0)
