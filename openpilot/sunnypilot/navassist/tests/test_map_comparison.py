import json

import numpy as np
from PIL import Image
import pytest

from openpilot.sunnypilot.navassist.tests.test_map_input import archive_bytes
from tools.compare_navigation_map import compare, load_capture, pixel_metrics


def capture(tmp_path, **changes):
  directory = tmp_path / 'capture'
  directory.mkdir()
  metadata = {'provider': 'AMAP_SDK', 'coordinateSystem': 'gcj02', 'size': 256,
              'scaleMetersPerPixel': 1, 'pose': {'point': {'latitude': 0, 'longitude': 0}, 'bearing': 0},
              'routePoints': [{'latitude': -.01, 'longitude': 0}, {'latitude': .01, 'longitude': 0}]}
  metadata.update(changes)
  (directory / 'metadata.json').write_text(json.dumps(metadata), encoding='utf-8')
  for name in ('basemap-grey.png', 'comparison.png'):
    Image.fromarray(np.zeros((256, 256), dtype=np.uint8)).save(directory / name)
  return directory


def test_real_archive_comparison_and_missing_roads(tmp_path):
  directory = capture(tmp_path)
  archive = tmp_path / 'map.pmtiles'
  archive.write_bytes(archive_bytes())
  output = tmp_path / 'comparison'
  report = compare(directory, archive, output)
  assert report['officialTilesCompleteAndRoadNearEgo'] and not report['modelCompatible']
  assert report['roadStyleExcludingRouteAndEgo']['referenceWhiteRoadPixels'] > 0
  assert report['roadStyleExcludingRouteAndEgo']['whiteRoadIoU'] == 0
  assert report['referencePoseWgs84'] == {'longitude': 0, 'latitude': 0, 'bearing': 0}
  assert Image.open(output / 'comparison-sheet.png').size == (768, 286)
  with pytest.raises(FileExistsError):
    compare(directory, archive, output)  # Never overwrite a previous comparison.
  archive.write_bytes(archive_bytes(roads=False))
  report = compare(directory, archive, tmp_path / 'missing')
  assert not report['officialTilesCompleteAndRoadNearEgo']
  assert report['roadStyleExcludingRouteAndEgo'] is None and report['completeImage'] is None


@pytest.mark.parametrize('changes', [{'coordinateSystem': 'wgs84'}, {'size': 512}, {'scaleMetersPerPixel': 2},
                                     {'scaleMetersPerPixel': float('nan')}, {'routePoints': [{'latitude': 90, 'longitude': 0}]}])
def test_reject_invalid_capture_before_creating_output(tmp_path, changes):
  with pytest.raises(ValueError):
    load_capture(capture(tmp_path, **changes))


def test_road_metric_mask_does_not_count_route_or_marker():
  reference = np.array([[255, 128], [0, 255]], dtype=np.uint8)
  candidate = np.array([[255, 0], [32, 255]], dtype=np.uint8)
  report = pixel_metrics(candidate, reference, np.array([[True, False], [True, True]]))
  assert report['whiteRoadIoU'] == 1
  assert report['pixels'] == 3
  assert report['meanAbsoluteError'] == pytest.approx(32 / 3)
  assert report['nonBlackPixelsOverOfficialBackground'] == 1
