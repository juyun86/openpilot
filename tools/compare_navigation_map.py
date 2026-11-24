"""Offline diagnostic comparison of a frozen Android capture with the official renderer.

No network, model input or vehicle connection. Requires a local PMTiles archive.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

from openpilot.selfdrive.navigation.pmtiles import PMTiles
from openpilot.selfdrive.navigation.renderer import MapRenderer, draw_segments, mercator, project
from openpilot.sunnypilot.navassist.map_coordinates import to_wgs84
from openpilot.sunnypilot.navassist.map_input import LocalTiles


def load_capture(directory):
  metadata_path = directory / 'metadata.json'
  if metadata_path.stat().st_size > 2_097_152:
    raise ValueError('Capture metadata exceeds 2 MiB')
  metadata = json.loads(metadata_path.read_text(encoding='utf-8'))
  if metadata.get('provider') != 'AMAP_SDK' or metadata.get('coordinateSystem') != 'gcj02' or metadata.get('size') != 256:
    raise ValueError('Expected a 256 pixel AMAP_SDK GCJ-02 capture')
  scale = float(metadata['scaleMetersPerPixel'])
  if not math.isfinite(scale) or abs(scale - 1) > .05:
    raise ValueError('Capture scale is not 1 meter per pixel')
  point = metadata['pose']['point']
  longitude, latitude = to_wgs84(float(point['longitude']), float(point['latitude']), 'gcj02')
  bearing = float(metadata['pose']['bearing'])
  if abs(latitude) >= 85 or not math.isfinite(bearing):
    raise ValueError('Invalid capture pose')
  points = metadata['routePoints']
  if not isinstance(points, list) or len(points) > 20_000:
    raise ValueError('Invalid capture route')
  route = np.asarray([to_wgs84(float(p['longitude']), float(p['latitude']), 'gcj02') for p in points], dtype=np.float64).reshape(-1, 2)
  if np.any(np.abs(route[:, 1]) >= 85):
    raise ValueError('Invalid route latitude')
  images = []
  for name in ('basemap-grey.png', 'comparison.png'):
    with Image.open(directory / name) as image:
      if image.size != (256, 256):
        raise ValueError(f'{name} must be 256 by 256')
      images.append(np.asarray(image.convert('L')).copy())
  return metadata, (longitude, latitude, bearing), route, images


def pixel_metrics(candidate, reference, included):
  if not included.any():
    return {'pixels': 0, 'meanAbsoluteError': None, 'whiteRoadIoU': None}
  error = np.abs(candidate.astype(np.float64) - reference)
  candidate_roads, reference_roads = candidate >= 224, reference >= 224
  union = (candidate_roads | reference_roads) & included
  intersection = (candidate_roads & reference_roads) & included
  return {
    'pixels': int(included.sum()),
    'meanAbsoluteError': float(error[included].mean()),
    'whiteRoadIoU': float(intersection.sum() / union.sum()) if union.any() else None,
    'candidateWhiteRoadPixels': int((candidate_roads & included).sum()),
    'referenceWhiteRoadPixels': int((reference_roads & included).sum()),
    'nonBlackPixelsOverOfficialBackground': int(((candidate > 16) & (reference == 0) & included).sum()),
  }


def compare(directory, archive_path, output, *, reference_source='Local PMTiles archive; provenance unspecified'):
  metadata, pose, route, (basemap, candidate) = load_capture(directory)
  archive = PMTiles(LocalTiles(archive_path))
  try:
    renderer = MapRenderer(archive)
    reference, valid = renderer.render(*pose, route)
    reference_basemap, basemap_valid = renderer.render(*pose, np.empty((0, 2)))
  finally:
    archive.close()
  # Exclude the overlaid route and ego from road-style metrics. Keep the original
  # fixed pose and bearing, rather than snapping again onto different road data.
  mask = np.zeros((256, 256), dtype=np.uint8)
  pixels = project(mercator(route), *pose)
  draw_segments(mask, pixels[:-1], pixels[1:], 4, 255)
  ego_area = np.zeros_like(mask, dtype=bool)
  ego_area[180:202, 118:138] = True  # Includes the black ego outline.
  included = (mask == 0) & ~ego_area
  report = {
    'modelCompatible': False,
    'referenceDataSource': reference_source,
    'officialTilesCompleteAndRoadNearEgo': bool(valid and basemap_valid),
    'referencePoseWgs84': {'longitude': pose[0], 'latitude': pose[1], 'bearing': pose[2]},
    'routeRevision': metadata.get('routeRevision'), 'routePointCount': len(route),
    'styleRequested': metadata.get('styleRequested'), 'styleFailure': metadata.get('styleFailure'),
    'sdkMapType': metadata.get('sdkMapType'),
    'roadStyleExcludingRouteAndEgo': pixel_metrics(basemap, reference_basemap, included) if valid and basemap_valid else None,
    'completeImage': pixel_metrics(candidate, reference, np.ones_like(included)) if valid else None,
    'interpretation': 'Diagnostic only. Different source road geometry and rasterization affect these metrics; no compatibility threshold is asserted.',
  }
  output.mkdir(parents=True, exist_ok=False)
  Image.fromarray(reference).save(output / 'official.png')
  Image.fromarray(reference_basemap).save(output / 'reference-basemap.png')
  difference = np.abs(candidate.astype(np.int16) - reference.astype(np.int16)).astype(np.uint8)
  Image.fromarray(difference).save(output / 'absolute-difference.png')
  sheet = Image.new('RGB', (768, 286), '#202020')
  draw = ImageDraw.Draw(sheet)
  for i, (name, pixels) in enumerate((('AMAP capture', candidate), ('OP renderer / local tiles', reference), ('Absolute difference', difference))):
    draw.text((i * 256 + 8, 8), name, fill='white')
    sheet.paste(Image.fromarray(pixels).convert('RGB'), (i * 256, 30))
  sheet.save(output / 'comparison-sheet.png')
  (output / 'report.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n', encoding='utf-8')
  return report


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('capture', type=Path, help='Directory containing Android metadata.json and PNGs')
  parser.add_argument('--archive', type=Path, required=True, help='Local Protomaps PMTiles archive')
  parser.add_argument('--output', type=Path, required=True, help='New directory for images and report')
  parser.add_argument('--reference-source', default='Local PMTiles archive; provenance unspecified',
                      help='Describe archive provenance for the diagnostic report')
  args = parser.parse_args()
  report = compare(args.capture, args.archive, args.output, reference_source=args.reference_source)
  print(json.dumps(report, indent=2, allow_nan=False))
  return 0 if report['officialTilesCompleteAndRoadNearEgo'] else 2


if __name__ == '__main__':
  raise SystemExit(main())
