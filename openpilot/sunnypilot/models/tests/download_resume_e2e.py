"""Local HTTP integration checks. No real model selection or proxy changes."""
import asyncio
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import re
import socket
import tempfile
import threading

from openpilot.cereal import custom
from openpilot.sunnypilot.models import manager as module

DATA = bytes(range(256)) * 4096
DIGEST = hashlib.sha256(DATA).hexdigest()
requests_seen = []
mode = 'normal'

class Handler(BaseHTTPRequestHandler):
  def log_message(self, *_): pass

  def do_GET(self):
    value = self.headers.get('Range', '')
    requests_seen.append(value)
    if mode == 'outage':
      self.send_response(503); self.send_header('Content-Length', '0'); self.end_headers(); return
    match = re.fullmatch(r'bytes=(\d+)-(\d*)', value)
    start = int(match[1]) if match else 0
    end = min(int(match[2]) if match and match[2] else len(DATA)-1, len(DATA)-1)
    if start >= len(DATA):
      self.send_response(416); self.send_header('Content-Range', f'bytes */{len(DATA)}')
      self.send_header('Content-Length', '0'); self.end_headers(); return
    if mode == 'ignore':
      start, end = 0, len(DATA)-1
      self.send_response(200)
    else:
      self.send_response(206)
      shown_start = start+1 if mode == 'bad_range' else start
      self.send_header('Content-Range', f'bytes {shown_start}-{end}/{len(DATA)}')
    body = DATA[start:end+1]
    self.send_header('Content-Length', str(len(body))); self.end_headers()
    try:
      if mode == 'disconnect' and len(requests_seen) == 1:
        self.wfile.write(body[:len(DATA)//4]); self.wfile.flush()
        self.connection.shutdown(socket.SHUT_RDWR); self.connection.close()
      else:
        self.wfile.write(body)
    except (BrokenPipeError, ConnectionResetError): pass

def exercise(name, behavior, prefix=b'', expected_success=True, retain=False):
  global mode
  mode = behavior; requests_seen.clear()
  with tempfile.TemporaryDirectory(prefix='model-resume-') as directory:
    root = Path(directory); partial = root/'sample.bin.download'
    if prefix: partial.write_bytes(prefix)
    downloader = module.ModelManagerSP.__new__(module.ModelManagerSP)
    downloader._download_start_times = {}; downloader._chunk_size = 65536
    downloader.selected_bundle = None
    downloader._download_interrupted = lambda: False
    downloader._sync_artifact_progress = lambda _: None
    downloader._report_status = lambda: None
    artifact = custom.ModelManagerSP.Artifact()
    artifact.fileName = 'sample.bin'
    artifact.downloadUri.uri = f'http://127.0.0.1:{server.server_port}/model'
    artifact.downloadUri.sha256 = DIGEST
    error = None
    try: asyncio.run(downloader._process_artifact(artifact, directory))
    except Exception as exc: error = type(exc).__name__
    if expected_success:
      assert error is None, (name, error)
      assert hashlib.sha256((root/'sample.bin').read_bytes()).hexdigest() == DIGEST
      assert not partial.exists()
    else:
      assert error is not None and not (root/'sample.bin').exists(), name
      if retain: assert partial.read_bytes() == prefix
    if behavior == 'disconnect':
      assert len(requests_seen) > 1 and any(int(re.match(r'bytes=(\d+)', r)[1]) > 0 for r in requests_seen[1:]), requests_seen
    print(json.dumps({'case': name, 'passed': True, 'ranges': list(requests_seen), 'expected_error': error}), flush=True)

if __name__ == '__main__':
  server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
  threading.Thread(target=server.serve_forever, daemon=True).start()
  module.download_proxies = lambda: {}
  module.DOWNLOAD_MAX_RETRIES = 2
  try:
    exercise('disconnect_resumes_and_hash_matches', 'disconnect')
    exercise('server_ignores_range_restarts_safely', 'ignore', DATA[:len(DATA)//4])
    exercise('wrong_content_range_rejected', 'bad_range', DATA[:len(DATA)//4], False, True)
    exercise('outage_preserves_partial', 'outage', DATA[:len(DATA)//4], False, True)
    exercise('complete_partial_verified_after_416', 'normal', DATA)
    exercise('oversized_partial_restarted_after_416', 'normal', DATA+b'extra')
  finally:
    server.shutdown(); server.server_close()
  print('ALL_HTTP_RESUME_CHECKS_PASSED', flush=True)
