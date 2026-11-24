from dataclasses import replace
import gzip
import struct
import threading

import numpy as np
import pytest

from openpilot.cereal import custom
from openpilot.selfdrive.navigation.pmtiles import PMTiles
from openpilot.selfdrive.navigation.renderer import MapRenderer, mercator, project
from openpilot.sunnypilot.navassist.map_coordinates import gcj_offset, to_wgs84
from openpilot.sunnypilot.navassist.map_input import LocalTiles, MapFrame, ModelMapRenderer, ModelMapWorker, route_key
from openpilot.sunnypilot.navassist.model_inputs import fill_navigation_inputs, model_navigation_inputs
from openpilot.sunnypilot.navassist.tests.test_protocol import APP_KEY_ID, encode, payload, store
from openpilot.sunnypilot.navassist.tests.test_nav_model_input import route


def value():
  receiver = store(clock_ns=lambda: 0)
  receiver.accept(encode(payload()), APP_KEY_ID)
  receiver.accept_model_route(encode(route()), APP_KEY_ID)
  return receiver.model_input()


def varint(value):
  result = bytearray()
  while value > 127:
    result.append((value & 127) | 128)
    value >>= 7
  return bytes(result + bytes([value]))


def field(number, value):
  return (varint(number << 3 | 2) + varint(len(value)) + value) if isinstance(value, bytes) else varint(number << 3) + varint(value)


def archive_bytes(*, roads=True):
  # Real MVT/PMTiles bytes: a north/south road on the prime meridian at zoom 0.
  geometry = b''.join(varint(v) for v in [9, 4096, 0, 10, 0, 8192])
  feature = field(2, b'\x00\x00') + field(3, 2) + field(4, geometry)
  layer = field(15, 2) + field(1, b'roads') + field(2, feature) + field(3, b'kind') + field(4, field(1, b'minor_road')) + field(5, 4096)
  tile = gzip.compress(field(3, layer) if roads else field(3, field(1, b'buildings')))
  directory = gzip.compress(varint(1) + varint(0) + varint(1) + varint(len(tile)) + varint(1))
  header = bytearray(127)
  header[:8] = b'PMTiles\x03'
  end = 127 + len(directory)
  struct.pack_into('<8Q', header, 8, 127, len(directory), end, 0, end, 0, end, len(tile))
  header[97:102] = bytes([2, 2, 1, 0, 0])
  return bytes(header) + directory + tile


def test_actual_pmtiles_mvt_renderer_and_invalid_basemap(tmp_path):
  path = tmp_path / 'map.pmtiles'
  path.write_bytes(archive_bytes())
  archive = PMTiles(LocalTiles(path))
  try:
    image, valid = MapRenderer(archive).render(0, 0, 0, np.array([[0, -.01], [0, .01]]))
    assert valid and image.dtype == np.uint8 and image.shape == (256, 256)
    assert image[128, 128] == 128  # Official grey route, white surrounding road.
    assert np.any(image == 255)
  finally:
    archive.close()
  path.write_bytes(archive_bytes(roads=False))
  renderer = ModelMapRenderer(lambda: PMTiles(LocalTiles(path)))
  input_value = value()
  input_value = replace(input_value, route=replace(input_value.route, coordinate_system='wgs84', points=((-.01, 0), (.01, 0))),
                        latitude=0, longitude=0, speed_mps=0)
  assert renderer.render(input_value, 100) is None  # A route-only image is not valid map input.
  renderer.close()


def test_map_projection_orientation_and_scale():
  metre = 1 / 111319.49079327358
  points = mercator(np.array([[0, 0], [0, 64 * metre], [64 * metre, 0]]))
  np.testing.assert_allclose(project(points, 0, 0, 0), [[128, 192], [128, 128], [192, 192]], atol=1e-5)
  np.testing.assert_allclose(project(points, 0, 0, 90), [[128, 192], [64, 192], [128, 128]], atol=1e-5)


@pytest.mark.parametrize('lon,lat', [(116.397128, 39.916527), (121.473701, 31.230416), (113.264385, 23.129112), (0, 0)])
def test_gcj_inverse_numerical_roundtrip(lon, lat):
  dx, dy = gcj_offset(lon, lat)
  np.testing.assert_allclose(to_wgs84(lon + dx, lat + dy, 'gcj02'), [lon, lat], atol=1e-8, rtol=0)
  assert to_wgs84(lon, lat, 'wgs84') == (lon, lat)


def test_gcj_known_published_example_and_unknown_coordinates():
  # coordtransform README's published WGS84 -> GCJ02 example.
  np.testing.assert_allclose(to_wgs84(116.41024449916938, 39.91640428150164, 'gcj02'), [116.404, 39.915], atol=1e-8, rtol=0)
  with pytest.raises(ValueError):
    to_wgs84(121, 31, 'unknown')
  with pytest.raises(ValueError):
    to_wgs84(float('nan'), 31, 'gcj02')


def state():
  navigation = [0.] * 47
  navigation[10], navigation[-1] = 1., .8
  return custom.NavAssistStateSP.new_message(valid=True, routeActive=True, routeMatched=True,
    modelInputValid=True, publishMonoTime=100, modelMapMonoTime=100, modelMap=bytes([128]) * 65536, modelNavigation=navigation)


def inputs(s, **changes):
  kwargs = {'now_ns': 100, 'healthy': True, 'control_active': True, 'turn_active': True, 'driver_override': False}
  return model_navigation_inputs(s, **{**kwargs, **changes})


def test_real_capnp_payload_roundtrip_and_optional_defaults():
  s = state()
  with custom.NavAssistStateSP.from_bytes(s.to_bytes()) as restored:
    result = inputs(restored)
    assert result['nav_map'].dtype == np.uint8 and result['nav_map'].shape == (256, 256)
    assert result['navigation'].dtype == np.float32
    assert result['navigation'][10] == 1
  assert not custom.NavAssistStateSP.new_message().modelInputValid
  assert not inputs(custom.NavAssistStateSP.new_message())


@pytest.mark.parametrize('changes', [{'healthy': False}, {'control_active': False}, {'driver_override': True},
                                     {'turn_active': False}, {'now_ns': 200_000_101}, {'now_ns': 99}])
def test_existing_control_and_driver_gates_withhold_both_map_and_instruction(changes):
  assert inputs(state(), **changes) == {}


@pytest.mark.parametrize('changes', [{'stale': True}, {'routeActive': False}, {'routeMatched': False}, {'gpsWeak': True},
  {'modelInputValid': False}, {'modelMap': b''}, {'modelNavigation': []}, {'modelMapMonoTime': 600_000_000}])
def test_invalid_ipc_does_not_supply_navigation(changes):
  s = state()
  for key, value in changes.items():
    setattr(s, key, value)
  assert inputs(s) == {}


def test_turn_gate_does_not_reject_straight_context_and_expired_input_is_cleared():
  s = state()
  s.modelNavigation[10], s.modelNavigation[8] = 0, 1
  assert inputs(s, turn_active=False)
  buffers = {'nav_map': np.zeros((1, 256, 256), np.float32), 'navigation': np.zeros((1, 47), np.float32)}
  fill_navigation_inputs(buffers, inputs(s))
  assert buffers['nav_map'].max() == 128
  fill_navigation_inputs(buffers, {})
  assert not np.any(buffers['nav_map']) and not np.any(buffers['navigation'])


def test_worker_does_not_block_or_queue_a_second_render_and_discards_exit_result():
  started, release = threading.Event(), threading.Event()

  class SlowRenderer:
    calls = 0

    def render(self, v, now):
      self.calls += 1
      started.set()
      assert release.wait(2)
      return MapFrame(route_key(v), now, b'x' * 65536)

    def close(self):
      pass

  renderer = SlowRenderer()
  worker = ModelMapWorker(renderer, enabled=True)
  v = value()
  try:
    assert worker.update(v, 100) is None
    assert started.wait(1)
    for now in range(101, 110):
      assert worker.update(v, now) is None
    assert renderer.calls == 1
    assert worker.update(None, 111) is None
    release.set()
    worker.pending.result(timeout=1)
    assert worker.update(v, 112) is None  # Same identity after exit cannot resurrect the old frame.
  finally:
    release.set()
    worker.close()
