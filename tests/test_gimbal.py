import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import time
from app.core.gimbal_controller import gimbal_ctrl

def test_gimbal_track_target_responsiveness():
    # Simulate connection flag for test
    gimbal_ctrl.is_connected = True
    gimbal_ctrl.telemetry["packets_received"] = 1
    gimbal_ctrl.telemetry["last_updated"] = time.time()
    gimbal_ctrl.telemetry["pitch"] = 0.0
    gimbal_ctrl.telemetry["yaw"] = 0.0

    # Initial state
    gimbal_ctrl.stop_tracking()
    assert gimbal_ctrl._is_active_tracking is False
    assert gimbal_ctrl._control_valid is False

    # 1. Target detected with error: err_x = 0.35, err_y = -0.20
    gimbal_ctrl.track_target(err_x=0.35, err_y=-0.20)
    assert gimbal_ctrl._is_active_tracking is True
    assert gimbal_ctrl._control_valid is True
    # Yaw should increase (turn right), Pitch should increase (pitch up because err_y < 0)
    assert gimbal_ctrl._target_yaw > 0.0
    assert gimbal_ctrl._target_pitch > 0.0

    # 2. Test fine convergence: when target enters deadband (|err| <= 0.02)
    # Give frames for EWMA filter to smoothly decelerate to zero
    for _ in range(10):
        time.sleep(0.02)
        gimbal_ctrl.track_target(err_x=0.005, err_y=-0.008)
    
    # After settling inside deadband, angles should hold completely steady
    steady_yaw = gimbal_ctrl._target_yaw
    steady_pitch = gimbal_ctrl._target_pitch
    time.sleep(0.02)
    gimbal_ctrl.track_target(err_x=0.005, err_y=-0.008)
    assert gimbal_ctrl._control_valid is True
    assert abs(gimbal_ctrl._target_yaw - steady_yaw) < 0.001
    assert abs(gimbal_ctrl._target_pitch - steady_pitch) < 0.001

    gimbal_ctrl.stop_tracking()
    gimbal_ctrl.is_connected = False
    print("GIMBAL TRACKING TESTS PASSED!")

if __name__ == "__main__":
    test_gimbal_track_target_responsiveness()

