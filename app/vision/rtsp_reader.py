import os

# Suppress verbose C-level OpenCV / FFmpeg stderr logs before importing cv2
os.environ.setdefault("OPENCV_LOG_LEVEL", "OFF")
os.environ.setdefault("OPENCV_FFMPEG_LOGLEVEL", "-8")

import cv2
import time
import socket
import logging
import threading
import numpy as np
from urllib.parse import urlparse
from typing import Tuple, Optional

try:
    if hasattr(cv2, "utils") and hasattr(cv2.utils, "logging"):
        cv2.utils.logging.setLogLevel(cv2.utils.logging.LOG_LEVEL_SILENT)
except Exception:
    pass

logger = logging.getLogger("rtsp_reader")
logger.setLevel(logging.INFO)


class RTSPStreamReader:
    """
    Ultra low-latency RTSP / HTTP video stream reader matching VLC real-time playback.
    Thread-safe runtime stream switching: VideoCapture is opened, read, and released
    EXCLUSIVELY inside the grabber worker thread to eliminate C-level Segmentation Faults (SIGSEGV).
    """
    def __init__(self, rtsp_url: str, transport: str = "tcp"):
        self.rtsp_url = rtsp_url.strip() if rtsp_url else ""
        self.transport = transport.lower()
        self.is_connected = False
        self.status_msg = "CHƯA KẾT NỐI RTSP"
        self.last_frame_time = 0.0
        
        self.frame_width = 640
        self.frame_height = 480
        
        # Latest frame buffer & sequence counter
        self._latest_frame: Optional[np.ndarray] = None
        self._frame_id = 0
        self._lock = threading.Lock()
        self._new_frame_event = threading.Event()
        
        # Thread-safe runtime switch request flags
        self._switch_requested = False
        self._pending_url = self.rtsp_url
        self._pending_transport = self.transport
        
        # Grabber worker thread state
        self._is_running = False
        self._thread: Optional[threading.Thread] = None
        self._cap: Optional[cv2.VideoCapture] = None
        self._last_connect_attempt = 0.0
        self._logged_waiting_url: Optional[str] = None

    def start(self):
        """Start the continuous low-latency grabber thread."""
        if self._is_running:
            return
        self._is_running = True
        self._thread = threading.Thread(target=self._grabber_worker, daemon=True)
        self._thread.start()
        logger.info(f"Zero-latency RTSP grabber thread started for: {self.rtsp_url}")

    def stop(self):
        """Stop grabber thread and release resources cleanly."""
        self._is_running = False
        if self._thread and self._thread.is_alive() and self._thread != threading.current_thread():
            self._thread.join(timeout=1.0)
        self._thread = None
        self.release()
        self.status_msg = "ĐÃ DỪNG RTSP"

    def release(self):
        """Release underlying OpenCV VideoCapture."""
        with self._lock:
            if self._cap:
                try:
                    self._cap.release()
                except Exception:
                    pass
                self._cap = None
            self.is_connected = False
            self._latest_frame = None
        self._new_frame_event.set()

    def set_rtsp_url(self, new_url: str, transport: Optional[str] = None):
        """
        Safely request switching RTSP stream URL at runtime.
        Does NOT release the capture directly in the caller's thread, preventing
        fatal SIGSEGV race conditions with cv2.VideoCapture.read() in the grabber thread.
        """
        if not new_url:
            return
        cleaned_url = new_url.strip()
        logger.info(f"Requesting RTSP URL switch: {self.rtsp_url} -> {cleaned_url}")
        
        with self._lock:
            self._pending_url = cleaned_url
            if transport:
                self._pending_transport = transport.lower()
            self._switch_requested = True
            self.is_connected = False
            self.status_msg = f"ĐANG KẾT NỐI RTSP: {cleaned_url}..."
        self._new_frame_event.set()

    def read_latest(self, timeout: float = 0.04) -> Tuple[bool, Optional[np.ndarray], bool]:
        """
        Wait up to timeout for a brand-new frame from the grabber.
        Returns:
            (is_connected, frame, is_new_frame)
        """
        is_new = self._new_frame_event.wait(timeout=timeout)
        if is_new:
            self._new_frame_event.clear()

        with self._lock:
            if self.is_connected and self._latest_frame is not None:
                if time.time() - self.last_frame_time < 2.0:
                    return True, self._latest_frame, is_new
            return False, None, False

    def read(self) -> Tuple[bool, Optional[np.ndarray]]:
        """
        Instantly retrieve the single newest frame captured by the grabber.
        Zero memory copy, zero latency accumulation.
        """
        ok, frame, _ = self.read_latest(timeout=0.0)
        return ok, frame

    def _grabber_worker(self):
        """
        Dedicated socket drainer loop:
        Exclusively handles VideoCapture creation, frame reading, and clean disposal.
        """
        consecutive_failures = 0

        while self._is_running:
            # 1. Handle runtime stream URL switch request safely on this thread
            switch_now = False
            with self._lock:
                if self._switch_requested:
                    self._switch_requested = False
                    self.rtsp_url = self._pending_url
                    if self._pending_transport:
                        self.transport = self._pending_transport
                    self._logged_waiting_url = None
                    self._last_connect_attempt = 0.0
                    switch_now = True

            if switch_now:
                if self._cap is not None:
                    try:
                        self._cap.release()
                    except Exception:
                        pass
                    self._cap = None
                with self._lock:
                    self.is_connected = False
                    self._latest_frame = None

            target_url = self.rtsp_url

            # 2. Connect / Reconnect if capture is not open
            if self._cap is None or not self._cap.isOpened():
                now = time.time()
                if now - self._last_connect_attempt < 1.5:
                    time.sleep(0.1)
                    continue
                self._last_connect_attempt = now

                if not self._is_endpoint_reachable(target_url, timeout=1.2):
                    with self._lock:
                        self.is_connected = False
                        self.status_msg = f"CHỜ TÍN HIỆU CAMERA: {target_url}"
                    if self._logged_waiting_url != target_url:
                        logger.info(f"Waiting for RTSP endpoint at {target_url}...")
                        self._logged_waiting_url = target_url
                    time.sleep(0.5)
                    continue

                with self._lock:
                    self.status_msg = f"ĐANG KẾT NỐI RTSP ({self.transport.upper()}): {target_url}..."

                cap = self._open_capture(target_url)
                if cap is None or not cap.isOpened():
                    with self._lock:
                        self.is_connected = False
                        self.status_msg = f"KHÔNG THỂ MỞ RTSP: {target_url}"
                    time.sleep(0.5)
                    continue

                self._cap = cap
                with self._lock:
                    self.is_connected = True
                    self._logged_waiting_url = None
                    self.status_msg = f"ĐÃ KẾT NỐI RTSP: {target_url}"
                consecutive_failures = 0
                logger.info(f"RTSP stream connected with zero-latency buffer: {target_url}")

            # 3. Continuous frame grabbing loop (Drain queue immediately)
            try:
                ret, frame = self._cap.read()
                if not ret or frame is None:
                    consecutive_failures += 1
                    if consecutive_failures > 15:
                        logger.warning("RTSP read failed repeatedly. Reconnecting...")
                        if self._cap:
                            try:
                                self._cap.release()
                            except Exception:
                                pass
                            self._cap = None
                        with self._lock:
                            self.is_connected = False
                            self.status_msg = "MẤT TÍN HIỆU RTSP -> ĐANG KẾT NỐI LẠI..."
                        time.sleep(0.5)
                    else:
                        time.sleep(0.01)
                    continue

                # Successfully captured newest frame
                consecutive_failures = 0
                now = time.time()
                h, w = frame.shape[:2]

                with self._lock:
                    self._latest_frame = frame
                    self.last_frame_time = now
                    self.frame_width = w
                    self.frame_height = h
                    self.is_connected = True
                    self._frame_id += 1
                self._new_frame_event.set()

            except Exception as e:
                logger.debug(f"Grabber read exception: {e}")
                time.sleep(0.02)

        # Cleanup on thread exit
        if self._cap is not None:
            try:
                self._cap.release()
            except Exception:
                pass
            self._cap = None

    def _is_endpoint_reachable(self, url: str, timeout: float = 1.2) -> bool:
        """Pre-check reachability to prevent FFmpeg connection refused stderr spam."""
        if not url:
            return False
        if url.isdigit():
            return True
        if url.startswith("/dev/video"):
            return os.path.exists(url)

        try:
            parsed = urlparse(url)
            scheme = (parsed.scheme or "").lower()
            if scheme in ("rtsp", "rtsps", "http", "https", "tcp", "rtmp"):
                if scheme == "rtsp" and self.transport == "udp":
                    return True
                host = parsed.hostname
                if not host:
                    return False
                default_ports = {
                    "rtsp": 554, "rtsps": 322, "http": 80, "https": 443, "rtmp": 1935, "tcp": 8554
                }
                port = parsed.port or default_ports.get(scheme, 554)
                with socket.create_connection((host, port), timeout=timeout):
                    return True
        except (OSError, Exception):
            return False
        return True

    def _open_capture(self, url: str) -> Optional[cv2.VideoCapture]:
        """Open VideoCapture with aggressive low-latency FFmpeg parameters."""
        if url.isdigit():
            cap = cv2.VideoCapture(int(url))
            if cap.isOpened():
                return cap
            cap.release()
            return None

        # Aggressive real-time low-latency FFmpeg options matching VLC parameters:
        # - probesize & analyzeduration set to minimum to eliminate stream analysis delay
        # - fflags;nobuffer + flags;low_delay + max_delay;0 eliminate packet queues
        # - sync;ext drops audio-video sync stalls
        # - reorder_queue_size;0 prevents packet jitter buffering
        if url.lower().startswith("rtsp://"):
            options = [
                f"rtsp_transport;{self.transport}",
                "stimeout;2000000",
                "fflags;nobuffer",
                "flags;low_delay",
                "max_delay;0",
                "probesize;32",
                "analyzeduration;0",
                "sync;ext",
                "reorder_queue_size;0"
            ]
            os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "|".join(options)
        else:
            os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "fflags;nobuffer|flags;low_delay|max_delay;0"

        cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            return cap
        cap.release()

        # Fallback to HTTP stream (e.g. tests/mock_video_streamer.py)
        if url.lower().startswith("rtsp://"):
            http_fallback = "http://" + url[7:]
            cap_http = cv2.VideoCapture(http_fallback, cv2.CAP_FFMPEG)
            if cap_http.isOpened():
                cap_http.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                logger.info(f"Connected via HTTP stream fallback: {http_fallback}")
                return cap_http
            cap_http.release()

        return None
