"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import asyncio
import os
from pathlib import Path
import shutil
import time
import re

import requests
from openpilot.common.mihomo_proxy import download_proxies
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper
from openpilot.common.swaglog import cloudlog
from openpilot.common.hardware.hw import Paths

from openpilot.cereal import messaging, custom
from openpilot.sunnypilot.models.fetcher import ModelFetcher
from openpilot.sunnypilot.models.helpers import (ACTIVE_BUNDLE_KEYS, get_active_bundle, get_selected_bundle,
                                                  resolve_bundle_by_ref, validate_active_bundles, verify_file)

# (connect, read) seconds. read is per-request inactivity, not a total cap
DOWNLOAD_TIMEOUT = (30, 30)
DOWNLOAD_MAX_RETRIES = 5
DOWNLOAD_RANGE_SIZE = 16 * 1024 * 1024


class DownloadCancelled(Exception):
  pass


class ModelManagerSP:
  """Manages model downloads and status reporting"""

  def __init__(self):
    self.params = Params()
    self.model_fetcher = ModelFetcher(self.params)
    self.pm = messaging.PubMaster(["modelManagerSP"])
    self.sm = messaging.SubMaster(["deviceState"])
    self.chestnut_present = False
    self.available_models: list[custom.ModelManagerSP.ModelBundle] = []
    self.source_models: dict[str, list[custom.ModelManagerSP.ModelBundle]] = {}
    self.selected_bundle: custom.ModelManagerSP.ModelBundle = None
    self.active_bundle: custom.ModelManagerSP.ModelBundle = get_active_bundle(self.params, chestnut=self.chestnut_present)
    self._chunk_size = 128 * 1000  # 128 KB chunks
    self._download_start_times: dict[str, float] = {}  # Track start time per model
    self._download_ref: bytes | str | None = None

  def _download_interrupted(self) -> bool:
    # only removal cancels: a different ref is a queued selection that
    # _release_download_ref leaves in place for the next tick
    return self.params.get("ModelManager_DownloadRef") is None

  def _release_download_ref(self) -> None:
    if self.params.get("ModelManager_DownloadRef") == self._download_ref:
      self.params.remove("ModelManager_DownloadRef")
    self._download_ref = None

  def _sync_artifact_progress(self, source_artifact) -> None:
    """Mirror download progress to all artifacts sharing the same filename in the selected bundle."""
    if not self.selected_bundle:
      return
    for model in self.selected_bundle.models:
      artifact = model.artifact
      if artifact is not source_artifact and artifact.fileName == source_artifact.fileName:
        artifact.downloadProgress.status = source_artifact.downloadProgress.status
        artifact.downloadProgress.progress = source_artifact.downloadProgress.progress
        artifact.downloadProgress.eta = source_artifact.downloadProgress.eta

  def _calculate_eta(self, filename: str, progress: float) -> int:
    """Calculate ETA based on elapsed time and current progress"""
    if filename not in self._download_start_times or progress <= 0:
      return 60  # Default ETA for new downloads

    elapsed_time = time.monotonic() - self._download_start_times[filename]
    if elapsed_time <= 0:
      return 60

    # If we're at X% after Y seconds, we can estimate total time as (Y / X) * 100
    total_estimated_time = (elapsed_time / progress) * 100
    eta = total_estimated_time - elapsed_time

    return max(1, int(eta))  # Return at least 1 second if download is ongoing

  async def _download_file(self, url: str, path: str, model) -> None:
    """Resume interrupted whole-file artifacts; only verified files are activated."""
    self._download_start_times[model.fileName] = time.monotonic()
    initial_size = os.path.getsize(path) if os.path.exists(path) else 0
    known_total = None
    retries = 0
    try:
      while True:
        if self._download_interrupted():
          raise DownloadCancelled("Download cancelled")
        offset = os.path.getsize(path) if os.path.exists(path) else 0
        headers = {"Range": f"bytes={offset}-{offset + DOWNLOAD_RANGE_SIZE - 1}", "Accept-Encoding": "identity"}
        try:
          with requests.get(url, headers=headers, stream=True, timeout=DOWNLOAD_TIMEOUT, proxies=download_proxies()) as response:  # noqa: ASYNC210
            if response.status_code == 416:
              match = re.fullmatch(r"bytes \*/(\d+)", response.headers.get("Content-Range", ""))
              if match and offset == int(match[1]) and os.path.isfile(path):
                return  # caller still verifies the complete SHA256
              if match and offset > int(match[1]):
                with open(path, 'wb'):  # noqa: ASYNC230
                  pass
                initial_size = 0
                continue
            response.raise_for_status()
            if response.headers.get("Content-Encoding", "identity").lower() not in ("", "identity"):
              raise ValueError("Compressed model ranges cannot be resumed safely")
            if response.status_code == 206:
              match = re.fullmatch(r"bytes (\d+)-(\d+)/(\d+)", response.headers.get("Content-Range", ""))
              if not match:
                raise ValueError("Missing or invalid model Content-Range")
              begin, end, total_size = map(int, match.groups())
              if begin != offset or not begin <= end < total_size or end >= offset + DOWNLOAD_RANGE_SIZE:
                raise ValueError("Server returned a different model byte range")
              if "Content-Length" in response.headers and int(response.headers["Content-Length"]) != end-begin+1:
                raise ValueError("Model range length mismatch")
              mode = 'ab'
            elif response.status_code == 200:
              # Range is optional on a server: overwrite instead of appending a full response.
              offset = initial_size = 0
              total_size = int(response.headers.get("Content-Length", 0))
              end = total_size-1 if total_size else None
              mode = 'wb'
            else:
              raise ValueError(f"Unexpected model response status {response.status_code}")
            if known_total is not None and total_size and known_total != total_size:
              raise ValueError("Model size changed during download")
            known_total = total_size or known_total
            with open(path, mode) as f:  # noqa: ASYNC230
              for chunk in response.iter_content(chunk_size=self._chunk_size):
                if not chunk:
                  continue
                if end is not None and offset + len(chunk) > end+1:
                  raise ValueError("Model response exceeded requested byte range")
                f.write(chunk)
                offset += len(chunk)
                if self._download_interrupted():
                  raise DownloadCancelled("Download cancelled")
                if total_size:
                  elapsed = time.monotonic()-self._download_start_times[model.fileName]
                  received = offset-initial_size
                  model.downloadProgress.status = custom.ModelManagerSP.DownloadStatus.downloading
                  model.downloadProgress.progress = offset / total_size * 100
                  model.downloadProgress.eta = max(1, int((total_size-offset)*elapsed/received)) if received > 0 else 60
                  self._sync_artifact_progress(model)
                  self._report_status()
            if end is not None and offset != end+1:
              raise requests.exceptions.ChunkedEncodingError("Incomplete model range")
            retries = 0
            if not total_size or offset == total_size:
              return
        except requests.exceptions.RequestException as error:
          if isinstance(error, requests.exceptions.HTTPError) and error.response is not None:
            if error.response.status_code not in (408, 429) and error.response.status_code < 500:
              raise
          if self._download_interrupted():
            raise DownloadCancelled("Download cancelled") from error
          if retries >= DOWNLOAD_MAX_RETRIES:
            raise
          retries += 1
          saved = os.path.getsize(path) if os.path.exists(path) else 0
          cloudlog.warning(f"Model download retry {retries}: resuming {model.fileName} at {saved} bytes ({type(error).__name__})")
          await asyncio.sleep(min(8, 2 ** (retries-1)))
    finally:
      self._download_start_times.pop(model.fileName, None)

  async def _download_chunked(self, base_url: str, base_path: str, artifact, skip: frozenset[int] | set[int] = frozenset()) -> None:
    from openpilot.common.file_chunker import get_chunk_name, get_manifest_path

    num_chunks = len(artifact.chunks)
    if num_chunks == 0:
      raise ValueError("No chunks defined in artifact")

    manifest_path = get_manifest_path(base_path)
    self._download_start_times[artifact.fileName] = time.monotonic()

    # Shared connection saves a TCP+TLS handshake per chunk.
    # Keep sequential: the link saturates on one stream and Session is not thread-safe.
    completed = len(skip)
    with requests.Session() as session:
      for i, _ in enumerate(artifact.chunks):
        if i in skip:
          continue
        chunk_url = get_chunk_name(base_url, i, num_chunks)
        chunk_path = get_chunk_name(base_path, i, num_chunks)
        chunk_downloaded = 0
        with session.get(chunk_url, stream=True, timeout=DOWNLOAD_TIMEOUT, proxies=download_proxies()) as response:
          response.raise_for_status()
          chunk_size = int(response.headers.get("content-length", 0))
          with open(chunk_path, 'wb') as f:  # noqa: ASYNC230
            for data in response.iter_content(chunk_size=self._chunk_size):
              f.write(data)
              chunk_downloaded += len(data)
              if self._download_interrupted():
                raise DownloadCancelled("Download cancelled")
              intra = chunk_downloaded / max(chunk_size, 1)
              progress = min(99.0, ((completed + intra) / num_chunks) * 100)
              artifact.downloadProgress.status = custom.ModelManagerSP.DownloadStatus.downloading
              artifact.downloadProgress.progress = progress
              artifact.downloadProgress.eta = self._calculate_eta(artifact.fileName, progress)
              self._sync_artifact_progress(artifact)
              self._report_status()
        completed += 1

    with open(manifest_path, 'w') as f:  # noqa: ASYNC230
      f.write(str(num_chunks))
    if os.path.isfile(base_path):  # noqa: ASYNC240
      os.remove(base_path)
    del self._download_start_times[artifact.fileName]

  async def _process_artifact(self, artifact, destination_path: str) -> None:
    if not artifact.downloadUri.uri:
      return None
    if self._download_interrupted():
      raise DownloadCancelled("Download cancelled")

    url = artifact.downloadUri.uri
    expected_hash = artifact.downloadUri.sha256
    filename = artifact.fileName
    full_path = os.path.join(destination_path, filename)
    if not Path(full_path).resolve().is_relative_to(Path(destination_path).resolve()):  # noqa: ASYNC240
      raise ValueError("Model artifact path escapes destination")
    Path(full_path).parent.mkdir(parents=True, exist_ok=True)
    temporary = f"{full_path}.download"

    try:
      # progress counts only valid chunks so a resumed download continues the
      # bar from where verification left it, instead of falling back to zero
      is_cached = False
      valid_chunks: set[int] = set()
      if len(artifact.chunks) > 0:
        from openpilot.common.file_chunker import get_chunk_name
        num_chunks = len(artifact.chunks)
        for i, chunk in enumerate(artifact.chunks):
          if self._download_interrupted():
            raise DownloadCancelled("Download cancelled")
          if await verify_file(get_chunk_name(full_path, i, num_chunks), chunk.sha256):
            valid_chunks.add(i)
          artifact.downloadProgress.status = custom.ModelManagerSP.DownloadStatus.verifying
          artifact.downloadProgress.progress = (len(valid_chunks) / num_chunks) * 100
          self._sync_artifact_progress(artifact)
          self._report_status()
        is_cached = len(valid_chunks) == num_chunks
      else:
        if await verify_file(full_path, expected_hash):
          is_cached = True

      if is_cached:
        artifact.downloadProgress.status = custom.ModelManagerSP.DownloadStatus.cached
        artifact.downloadProgress.progress = 100
        artifact.downloadProgress.eta = 0
        self._sync_artifact_progress(artifact)
        self._report_status()
        return

      if len(artifact.chunks) > 0:
        await self._download_chunked(url, full_path, artifact, skip=valid_chunks)
        from openpilot.common.file_chunker import get_chunk_name
        for i, chunk in enumerate(artifact.chunks):
          chunk_path = get_chunk_name(full_path, i, len(artifact.chunks))
          if not await verify_file(chunk_path, chunk.sha256):
            raise ValueError(f"Hash validation failed for chunk {i+1} of {filename}")
      else:
        await self._download_file(url, temporary, artifact)
        if not await verify_file(temporary, expected_hash):
          os.remove(temporary)  # a complete but corrupt file must not be resumed
          raise ValueError(f"Hash validation failed for {filename}")
        os.replace(temporary, full_path)

      artifact.downloadProgress.status = custom.ModelManagerSP.DownloadStatus.downloaded
      artifact.downloadProgress.progress = 100
      artifact.downloadProgress.eta = 0
      self._sync_artifact_progress(artifact)
      self._report_status()

    except DownloadCancelled:
      # Keep whole-file prefixes and completed chunks for a later attempt.
      self._download_start_times.pop(artifact.fileName, None)
      artifact.downloadProgress.status = custom.ModelManagerSP.DownloadStatus.failed
      artifact.downloadProgress.eta = 0
      self._sync_artifact_progress(artifact)
      if self.selected_bundle:
        self.selected_bundle.status = custom.ModelManagerSP.DownloadStatus.failed
      self._report_status()
      raise

    except Exception as e:
      cloudlog.error(f"Error downloading {filename}: {str(e)}")
      # A transport error must not erase hundreds of MB already downloaded.
      artifact.downloadProgress.status = custom.ModelManagerSP.DownloadStatus.failed
      artifact.downloadProgress.eta = 0
      self._sync_artifact_progress(artifact)
      if self.selected_bundle:
        self.selected_bundle.status = custom.ModelManagerSP.DownloadStatus.failed
      self._report_status()
      self._download_start_times.pop(artifact.fileName, None)
      raise

  async def _process_model(self, model, destination_path: str) -> None:
    """Processes a single model download including verification"""
    await self._process_artifact(model.artifact, destination_path)

  def _report_status(self) -> None:
    """Reports current status through messaging system"""
    msg = messaging.new_message('modelManagerSP', valid=True)
    model_manager_state = msg.modelManagerSP
    if self.selected_bundle:
      model_manager_state.selectedBundle = self.selected_bundle

    if self.active_bundle:
      model_manager_state.activeBundle = self.active_bundle

    model_manager_state.availableBundles = self.available_models
    self.pm.send('modelManagerSP', msg)

  async def _download_bundle(self, model_bundle: custom.ModelManagerSP.ModelBundle, destination_path: str, source: str) -> None:
    self.selected_bundle = model_bundle
    self.selected_bundle.status = custom.ModelManagerSP.DownloadStatus.downloading
    for model in self.selected_bundle.models:
      model.artifact.downloadProgress.status = custom.ModelManagerSP.DownloadStatus.downloading
    self._report_status()
    os.makedirs(destination_path, exist_ok=True)

    try:
      seen_artifacts: set[str] = set()
      for model in self.selected_bundle.models:
        artifact = model.artifact
        if not artifact.fileName:
          continue
        if artifact.fileName in seen_artifacts:
          artifact.downloadProgress.status = custom.ModelManagerSP.DownloadStatus.cached
          artifact.downloadProgress.progress = 100
          artifact.downloadProgress.eta = 0
        else:
          seen_artifacts.add(artifact.fileName)
          await self._process_artifact(artifact, destination_path)

      if self._download_interrupted():
        raise DownloadCancelled("Download cancelled")
      self.selected_bundle.status = custom.ModelManagerSP.DownloadStatus.downloaded
      self.params.put(ACTIVE_BUNDLE_KEYS[source], model_bundle.to_dict(), block=True)
      self.active_bundle = get_active_bundle(self.params, chestnut=self.chestnut_present)

    except Exception:
      if self.selected_bundle is not None:
        self.selected_bundle.status = custom.ModelManagerSP.DownloadStatus.failed
      raise

    finally:
      self._report_status()

  def download(self, model_bundle: custom.ModelManagerSP.ModelBundle, destination_path: str, source: str) -> None:
    """Main entry point for downloading a model bundle"""
    asyncio.run(self._download_bundle(model_bundle, destination_path, source))

  def _process_download_requests(self) -> None:
    # loops so a ref queued during a download starts in the same tick, without
    # the bar dropping to idle for a tick between the two transfers
    last_ref = None
    while (ref_to_download := self.params.get("ModelManager_DownloadRef")) is not None:
      if ref_to_download == last_ref:  # a repeating ref falls back to the next tick instead of spinning
        return
      last_ref = ref_to_download
      resolved = resolve_bundle_by_ref(ref_to_download, self.source_models)
      if not resolved:
        return
      model_to_download, source = resolved
      self._download_ref = ref_to_download
      try:
        self.download(model_to_download, Paths.model_root(), source)
      except Exception as e:
        cloudlog.exception(e)
      finally:
        self._release_download_ref()
        self.selected_bundle = None

  def main_thread(self) -> None:
    """Main thread for model management"""
    rk = Ratekeeper(1, print_delay_threshold=None)

    while True:
      try:
        self.sm.update(0)
        self.chestnut_present = self.sm['deviceState'].chestnutPresent
        self.source_models = {source: self.model_fetcher.get_bundles_for_source(source) for source in ModelFetcher.MODEL_SOURCES}
        self.available_models = self.source_models[ModelFetcher.active_source(self.chestnut_present)]
        validate_active_bundles(self.params, self.source_models)
        self.active_bundle = get_active_bundle(self.params, chestnut=self.chestnut_present)

        if get_selected_bundle(self.params, "chestnut") is not None and self.params.get(ACTIVE_BUNDLE_KEYS["qcom"]) is None:
          if self.params.get("ModelManager_DownloadRef") is None:
            from openpilot.sunnypilot.models.model_name import DEFAULT_MODEL_REF
            self.params.put("ModelManager_DownloadRef", DEFAULT_MODEL_REF)

        self._process_download_requests()

        if self.params.get("ModelManager_ClearCache"):
          self.clear_model_cache()
          self.params.remove("ModelManager_ClearCache")

        self._report_status()
        rk.keep_time()

      except Exception as e:
        cloudlog.exception(f"Error in main thread: {str(e)}")
        rk.keep_time()

  def clear_model_cache(self) -> None:
    """
    Clears the model cache directory of all files except those in the active model bundle.
    """

    # Get list of files used by both slots' selected bundles (either may become
    # the truly active bundle depending on hardware availability)
    active_files = []
    for source in ACTIVE_BUNDLE_KEYS:
      if selected_bundle := get_selected_bundle(self.params, source):
        for model in selected_bundle.models:
          if model.artifact.fileName:
            active_files.append(model.artifact.fileName)

    # Legacy root files belong to the previous runtime and remain available for rollback.
    model_dir = Paths.model_root()
    active_directories = {Path(name).parts[0] for name in active_files}
    try:
      for filename in os.listdir(model_dir):
        path = Path(model_dir) / filename
        if (len(filename) == 64 and all(c in "0123456789abcdef" for c in filename) and
            filename not in active_directories and path.is_dir() and not path.is_symlink()):
          shutil.rmtree(path)
      cloudlog.info("Model cache cleared, keeping active model files")
    except Exception as e:
      cloudlog.exception(f"Error clearing model cache: {str(e)}")

def main():
  ModelManagerSP().main_thread()


if __name__ == "__main__":
  main()
