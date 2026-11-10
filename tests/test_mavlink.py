import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import time
from app.core.mavlink_controller import MAVLinkController

def test_controller_initialization():
    ctrl = MAVLinkController(connection_str="udp:127.0.0.1:14550")
    telem = ctrl.get_telemetry()
    assert telem["connected"] is False
    assert telem["mode"] == "---"

def test_velocity_command_formatting():
    ctrl = MAVLinkController()
    
    # Climb + Sway right + Yaw right
    ctrl.send_velocity(vx=0.0, vy=0.5, vz=-0.4, yaw_rate=0.2)
    telem = ctrl.get_telemetry()
    assert "LÊN" in telem["last_command"]
    assert "SANG PHẢI" in telem["last_command"]
    assert "QUAY PHẢI" in telem["last_command"]
    assert telem["active_velocities"]["vy"] == 0.5
    assert telem["active_velocities"]["vz"] == -0.4

def test_send_stop():
    ctrl = MAVLinkController()
    ctrl.send_velocity(vx=0.0, vy=0.5, vz=-0.4, yaw_rate=0.2)
    ctrl.send_stop()
    telem = ctrl.get_telemetry()
    assert "ĐỨNG IM" in telem["last_command"]
    assert telem["active_velocities"]["vy"] == 0.0
    assert telem["active_velocities"]["vz"] == 0.0
    assert telem["active_velocities"]["yaw_rate"] == 0.0

def test_controller_disconnect():
    ctrl = MAVLinkController()
    ctrl.is_connected = True
    ctrl.telemetry["connected"] = True
    ctrl.disconnect()
    assert ctrl.is_connected is False
    assert ctrl.telemetry["connected"] is False
    assert ctrl.telemetry["mode"] == "---"

def test_ensure_guided_mode():
    ctrl = MAVLinkController()
    # When not connected
    assert ctrl.ensure_guided_mode() is False
    # When connected and mode is already GUIDED
    ctrl.is_connected = True
    ctrl.master = object() # mock object so it is not None
    ctrl.current_mode = "GUIDED"
    assert ctrl.ensure_guided_mode() is True

def test_statustext_storage():
    ctrl = MAVLinkController()
    class MockStatusText:
        text = b"GPS fix acquired\x00"
        severity = 6
        def get_type(self):
            return "STATUSTEXT"

    # Simulate receiving STATUSTEXT packet
    msg = MockStatusText()
    raw_text = getattr(msg, "text", "")
    if isinstance(raw_text, bytes):
        raw_text = raw_text.decode("utf-8", errors="ignore")
    clean_text = str(raw_text).strip("\x00 \t\r\n")
    ctrl.telemetry["statustexts"].append({
        "text": clean_text,
        "severity": msg.severity,
        "severity_name": "INFO",
        "type": "info",
        "time": time.time()
    })
    telem = ctrl.get_telemetry()
    assert len(telem["statustexts"]) == 1
    assert telem["statustexts"][0]["text"] == "GPS fix acquired"
    assert telem["statustexts"][0]["type"] == "info"

if __name__ == "__main__":
    print("Running MAVLink controller tests...")
    test_controller_initialization()
    test_velocity_command_formatting()
    test_send_stop()
    test_controller_disconnect()
    test_ensure_guided_mode()
    test_statustext_storage()
    print("ALL MAVLINK TESTS PASSED!")
