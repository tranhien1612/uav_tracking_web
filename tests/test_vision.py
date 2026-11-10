import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cv2
import numpy as np
from app.vision.tracker import ManualTracker

def create_synthetic_frame(circle_pos=(320, 240), radius=25, bg_color=(20, 24, 32)):
    """Generate synthetic test frame."""
    frame = np.full((480, 640, 3), bg_color, dtype=np.uint8)
    cv2.circle(frame, circle_pos, radius, (0, 0, 235), -1)
    return frame

def test_manual_tracker_initialization():
    tracker = ManualTracker()
    frame = create_synthetic_frame()
    
    # Initialize tracker with target ROI
    success = tracker.start_tracking(frame, (300, 220, 40, 40))
    assert success is True
    assert tracker.is_tracking is True
    assert tracker.status == "TRACKING"

def test_manual_tracker_update():
    tracker = ManualTracker()
    frame = create_synthetic_frame(circle_pos=(320, 240))
    tracker.start_tracking(frame, (300, 220, 40, 40))

    # Move target in next frame
    frame2 = create_synthetic_frame(circle_pos=(340, 260))
    success, bbox, err_x, err_y, is_centered, status = tracker.update(
        frame2, 640, 480, deadband_x=0.08, deadband_y=0.08
    )

    assert success is True
    assert bbox[0] == 320 and bbox[1] == 240
    assert abs(bbox[2] - 40) <= 2 and abs(bbox[3] - 40) <= 2
    assert status == "ĐANG THEO DÕI"

def test_manual_tracker_stop():
    tracker = ManualTracker()
    frame = create_synthetic_frame()
    tracker.start_tracking(frame, (300, 220, 40, 40))
    tracker.stop()

    assert tracker.is_tracking is False
    assert tracker.bbox is None
    assert tracker.status == "IDLE"

if __name__ == "__main__":
    print("Running vision unit tests...")
    test_manual_tracker_initialization()
    test_manual_tracker_update()
    test_manual_tracker_stop()
    print("ALL VISION TESTS PASSED!")