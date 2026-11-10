import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from fastapi import HTTPException
from app.core.mavlink_controller import mavlink_ctrl
from app.api.endpoints import (
    get_system_state,
    set_mode,
    ModeRequest,
    cancel_manual_tracking,
    update_rtsp_config,
    RTSPConfigRequest,
    manual_nudge,
    ManualNudgeRequest,
    get_serial_ports,
    update_deadband,
    DeadbandRequest,
    connect_mavlink,
    MAVLinkConnectRequest,
    disconnect_mavlink,
    takeoff_drone,
    TakeoffRequest,
    land_drone,
    get_gimbal_serial_ports,
    connect_gimbal,
    GimbalConnectRequest,
    disconnect_gimbal,
    center_gimbal,
    get_gimbal_telemetry,
    set_tracking_target,
    TrackingTargetRequest,
    set_tracking_model,
    TrackingModelRequest,
    update_gains,
    GainsRequest
)
from app.core.gimbal_controller import gimbal_ctrl

def test_api_state():
    data = get_system_state()
    assert "vision" in data
    assert "telemetry" in data
    assert data["vision"]["mode"] == "STANDBY"
    assert "deadband_x" in data["vision"]
    assert "deadband_y" in data["vision"]

def test_api_deadband():
    res = update_deadband(DeadbandRequest(deadband_x=0.12, deadband_y=0.15))
    assert res["status"] == "ok"
    assert abs(res["deadband_x"] - 0.12) < 1e-4
    assert abs(res["deadband_y"] - 0.15) < 1e-4

def test_api_serial_ports():
    res = get_serial_ports()
    assert "ports" in res
    assert isinstance(res["ports"], list)

def test_api_mavlink_connect_models():
    # Test UDP model construction
    req_udp = MAVLinkConnectRequest(connection_type="udp", ip="127.0.0.1", port=14550)
    assert req_udp.ip == "127.0.0.1"
    assert req_udp.port == 14550

    # Test Serial model construction
    req_serial = MAVLinkConnectRequest(connection_type="serial", serial_port="COM3", baud=57600)
    assert req_serial.serial_port == "COM3"
    assert req_serial.baud == 57600

def test_api_mode_switching():
    res = set_mode(ModeRequest(mode="TRACKING"))
    assert res["status"] == "ok"
    assert res["mode"] == "TRACKING"

    res3 = set_mode(ModeRequest(mode="STANDBY"))
    assert res3["status"] == "ok"
    assert res3["mode"] == "STANDBY"

def test_api_tracking_cancel():
    res = cancel_manual_tracking()
    assert res["status"] == "ok"

def test_api_rtsp_config():
    res = update_rtsp_config(RTSPConfigRequest(rtsp_url="rtsp://192.168.1.88:554/live", transport="udp"))
    assert res["status"] == "ok"
    assert res["rtsp_url"] == "rtsp://192.168.1.88:554/live"
    assert res["transport"] == "udp"

def test_api_manual_nudge_enforcement():
    # 1. When UAV is NOT connected -> Must reject with 400
    mavlink_ctrl.is_connected = False
    try:
        manual_nudge(ManualNudgeRequest(direction="up", speed=0.5))
        assert False, "Should have raised HTTPException 400 when UAV is not connected"
    except HTTPException as e:
        assert e.status_code == 400

    # 2. When UAV IS connected -> Must accept
    mavlink_ctrl.is_connected = True
    res = manual_nudge(ManualNudgeRequest(direction="up", speed=0.5))
    assert res["status"] == "ok"
    assert res["command"] == "up"
    mavlink_ctrl.is_connected = False

def test_api_takeoff_land():
    mavlink_ctrl.is_connected = False
    try:
        takeoff_drone(TakeoffRequest(altitude=3.0))
        assert False, "Should raise 400 when not connected"
    except HTTPException as e:
        assert e.status_code == 400

    try:
        land_drone()
        assert False, "Should raise 400 when not connected"
    except HTTPException as e:
        assert e.status_code == 400

    mavlink_ctrl.is_connected = True
    mavlink_ctrl.master = None  # mock
    res_tk = takeoff_drone(TakeoffRequest(altitude=3.0))
    assert res_tk["status"] == "failed" or res_tk["status"] == "ok"
    assert res_tk["altitude"] == 3.0

    res_land = land_drone()
    assert "status" in res_land
    mavlink_ctrl.is_connected = False

def test_api_mavlink_disconnect():
    mavlink_ctrl.is_connected = True
    mavlink_ctrl.telemetry["connected"] = True
    res = disconnect_mavlink()
    assert res["status"] == "disconnected"
    assert mavlink_ctrl.is_connected is False
    assert mavlink_ctrl.telemetry["connected"] is False

def test_api_gimbal_endpoints():
    ports_res = get_gimbal_serial_ports()
    assert "ports" in ports_res
    assert isinstance(ports_res["ports"], list)

    telem = get_gimbal_telemetry()
    assert "connected" in telem
    assert "pitch" in telem
    assert "yaw" in telem

    # Center gimbal endpoint returns ok
    res_center = center_gimbal()
    assert res_center["status"] == "ok"

    # Disconnect
    res_dis = disconnect_gimbal()
    assert res_dis["status"] in ("ok", "disconnected")

def test_api_tracking_target():
    # 1. Switch to UAV tracking
    res_uav = set_tracking_target(TrackingTargetRequest(control_target="uav"))
    assert res_uav["status"] == "ok"
    assert res_uav["control_target"] == "uav"

    # 2. Switch back to Gimbal tracking (default)
    res_gimbal = set_tracking_target(TrackingTargetRequest(control_target="gimbal"))
    assert res_gimbal["status"] == "ok"
    assert res_gimbal["control_target"] == "gimbal"

    # 3. Via gains endpoint
    res_gains = update_gains(GainsRequest(control_target="uav"))
    assert res_gains["status"] == "ok"
    assert res_gains["control_target"] == "uav"

    # Reset back to default gimbal
    set_tracking_target(TrackingTargetRequest(control_target="gimbal"))

def test_api_tracking_model():
    # 1. Switch to nanotrack
    res_nano = set_tracking_model(TrackingModelRequest(tracker_model="nanotrack"))
    assert res_nano["status"] == "ok"
    assert res_nano["tracker_model"] == "nanotrack"

    # 2. Switch back to csrt
    res_csrt = set_tracking_model(TrackingModelRequest(tracker_model="csrt"))
    assert res_csrt["status"] == "ok"
    assert res_csrt["tracker_model"] == "csrt"

    # 3. Via gains endpoint
    res_gains = update_gains(GainsRequest(tracker_model="nanotrack"))
    assert res_gains["status"] == "ok"
    assert res_gains["tracker_model"] == "nanotrack"

    # Reset back to csrt
    set_tracking_model(TrackingModelRequest(tracker_model="csrt"))

if __name__ == "__main__":
    print("Running API endpoint tests...")
    test_api_state()
    test_api_deadband()
    test_api_serial_ports()
    test_api_mavlink_connect_models()
    test_api_mode_switching()
    test_api_tracking_cancel()
    test_api_rtsp_config()
    test_api_manual_nudge_enforcement()
    test_api_takeoff_land()
    test_api_mavlink_disconnect()
    test_api_gimbal_endpoints()
    test_api_tracking_target()
    test_api_tracking_model()
    print("ALL API ENDPOINT TESTS PASSED!")
