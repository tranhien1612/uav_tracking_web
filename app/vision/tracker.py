import cv2
import numpy as np
import threading
from typing import Tuple, Optional, Dict, Any

class ManualTracker:
    """
    OpenCV CSRT (Channel and Spatial Reliability Tracker) for UAV / Gimbal visual servoing.
    Provides state-of-the-art accuracy, robust scale handling (prevents bounding box shrinking),
    and reliable tracking even during rapid motion or partial occlusion.
    Thread-safe with internal locking.
    """
    def __init__(self):
        self._lock = threading.Lock()
        self.tracker = None
        self.is_tracking = False
        self.bbox = None  # [x, y, w, h]
        self.target_center = None  # (cx, cy)
        self.status = "IDLE"  # IDLE, TRACKING, LOST
        self.confidence = 1.0

    def _create_tracker(self):
        """Create OpenCV CSRT tracker with fallback to KCF/MIL."""
        # 1. Primary: CSRT (highest accuracy, handles scale and deformation stably)
        if hasattr(cv2, 'TrackerCSRT_create'):
            return cv2.TrackerCSRT_create()
        elif hasattr(cv2, 'legacy') and hasattr(cv2.legacy, 'TrackerCSRT_create'):
            return cv2.legacy.TrackerCSRT_create()
        # 2. Fallbacks
        elif hasattr(cv2, 'TrackerKCF_create'):
            return cv2.TrackerKCF_create()
        elif hasattr(cv2, 'legacy') and hasattr(cv2.legacy, 'TrackerKCF_create'):
            return cv2.legacy.TrackerKCF_create()
        elif hasattr(cv2, 'TrackerMIL_create'):
            return cv2.TrackerMIL_create()
        else:
            raise RuntimeError("No suitable OpenCV Tracker found in cv2 installation.")

    def start_tracking(self, frame: np.ndarray, bbox: Tuple[int, int, int, int]) -> bool:
        """
        Initialize tracking with given bounding box [x, y, w, h].
        """
        x, y, w, h = [int(v) for v in bbox]
        frame_h, frame_w = frame.shape[:2]

        # Constrain bbox within frame limits
        x = max(0, min(x, frame_w - 10))
        y = max(0, min(y, frame_h - 10))
        w = max(10, min(w, frame_w - x))
        h = max(10, min(h, frame_h - y))

        if w < 10 or h < 10:
            return False

        try:
            new_tracker = self._create_tracker()
            new_tracker.init(frame, (x, y, w, h))

            with self._lock:
                self.tracker = new_tracker
                self.bbox = [x, y, w, h]
                self.target_center = (int(x + w / 2), int(y + h / 2))
                self.is_tracking = True
                self.status = "TRACKING"
                self.confidence = 1.0
            return True
        except Exception as e:
            with self._lock:
                self.status = f"ERROR: {e}"
                self.is_tracking = False
                self.tracker = None
            return False

    def update(self, frame: np.ndarray, frame_w: int, frame_h: int, deadband_x: float, deadband_y: float):
        """
        Update tracker with current frame.
        Returns:
            is_tracking (bool),
            bbox [x, y, w, h] or None,
            error_x (float [-1, 1]),
            error_y (float [-1, 1]),
            is_centered (bool),
            status (str)
        """
        with self._lock:
            if not self.is_tracking or self.tracker is None:
                return False, None, 0.0, 0.0, True, "CHƯA KHOANH VÙNG"
            tracker = self.tracker

        try:
            success, box = tracker.update(frame)
        except Exception:
            success = False
            box = None

        with self._lock:
            if not self.is_tracking or self.tracker is not tracker:
                return False, None, 0.0, 0.0, True, "CHƯA KHOANH VÙNG"

            if success and box is not None and box[2] > 5 and box[3] > 5:
                x, y, w, h = [int(round(v)) for v in box]
                # Clamp within frame limits
                x = max(0, min(x, frame_w - 5))
                y = max(0, min(y, frame_h - 5))
                w = max(5, min(w, frame_w - x))
                h = max(5, min(h, frame_h - y))

                self.bbox = [x, y, w, h]
                cx = int(round(x + w / 2.0))
                cy = int(round(y + h / 2.0))
                self.target_center = (cx, cy)
                self.status = "TRACKING"

                center_x = frame_w / 2.0
                center_y = frame_h / 2.0
                err_x = (cx - center_x) / center_x
                err_y = (cy - center_y) / center_y
                is_centered = (abs(err_x) <= deadband_x) and (abs(err_y) <= deadband_y)

                return True, self.bbox, err_x, err_y, is_centered, "ĐANG THEO DÕI"
            else:
                self.status = "LOST"
                return False, self.bbox, 0.0, 0.0, False, "MẤT DẤU VẬT THỂ"

    def stop(self):
        """Cancel and reset tracking."""
        with self._lock:
            self.is_tracking = False
            self.tracker = None
            self.bbox = None
            self.target_center = None
            self.status = "IDLE"
            self.confidence = 1.0