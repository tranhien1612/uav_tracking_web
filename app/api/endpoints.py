import asyncio
import json
import logging
from fastapi import APIRouter, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.responses import StreamingResponse, JSONResponse
from pydantic import BaseModel
from typing import List, Optional
from app.vision.video_manager import video_manager, OperationMode
from app.core.mavlink_controller import mavlink_ctrl
from app.core.gimbal_controller import gimbal_ctrl

logger = logging.getLogger("api_endpoints")
router = APIRouter()

class ModeRequest(BaseModel):
    mode: str

class ROIRequest(BaseModel):
    bbox: List[int]  # [x, y, w, h]

class MAVLinkConnectRequest(BaseModel):
    connection_type: Optional[str] = "udp"  # "udp" or "serial"
    ip: Optional[str] = "127.0.0.1"
    port: Optional[int] = 14550
    serial_port: Optional[str] = "COM3"
    baud: Optional[int] = 57600
    connection_str: Optional[str] = None

class FlightModeRequest(BaseModel):
    mode: str

class GimbalConnectRequest(BaseModel):
    port: Optional[str] = "/dev/ttyUSB0"
    baud: Optional[int] = 115200

class GimbalAnglesRequest(BaseModel):
    pitch: float
    yaw: float
    roll: Optional[float] = 0.0

class ArmRequest(BaseModel):
    arm: bool

class TakeoffRequest(BaseModel):
    altitude: Optional[float] = 1.0

class ManualNudgeRequest(BaseModel):
    direction: str  # up, down, left, right, yaw_left, yaw_right, stop
    speed: float = 0.5

class RTSPConfigRequest(BaseModel):
    rtsp_url: str = "rtsp://192.168.144.108" #"rtsp://127.0.0.1:8554/live"
    transport: Optional[str] = "tcp"

class DeadbandRequest(BaseModel):
    deadband_x: float
    deadband_y: float

class TrackingTargetRequest(BaseModel):
    control_target: str  # "gimbal" or "uav"

class TrackingModelRequest(BaseModel):
    tracker_model: str  # "csrt" or "nanotrack"

class GainsRequest(BaseModel):
    kp_sway: Optional[float] = None
    kp_climb: Optional[float] = None
    kp_yaw: Optional[float] = None
    deadband_x: Optional[float] = None
    deadband_y: Optional[float] = None
    control_target: Optional[str] = None
    tracker_model: Optional[str] = None

@router.get("/video_feed")
async def video_feed():
    """
    Zero-latency MJPEG stream endpoint for real-time web video rendering.
    Only transmits new frames (frame-skipping enabled) to completely prevent
    browser network socket accumulation and lag drift.
    """
    async def frame_generator():
        last_frame_id = -1
        while video_manager.is_running:
            frame_id, frame_bytes = video_manager.get_jpeg_with_id()
            if frame_bytes and frame_id != last_frame_id:
                last_frame_id = frame_id
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
            await asyncio.sleep(0.02)  # 50Hz poll is optimal for 30FPS stream without CPU waste

    headers = {
        "Cache-Control": "no-cache, no-store, must-revalidate, pre-check=0, post-check=0, max-age=0",
        "Pragma": "no-cache",
        "Expires": "0",
        "X-Accel-Buffering": "no"
    }
    return StreamingResponse(frame_generator(), media_type="multipart/x-mixed-replace; boundary=frame", headers=headers)

@router.get("/state")
def get_system_state():
    """Retrieve combined vision, MAVLink, and Gimbal telemetry state."""
    vision = video_manager.latest_state
    telem = mavlink_ctrl.get_telemetry()
    gimbal = gimbal_ctrl.get_telemetry()
    return {
        "vision": vision,
        "telemetry": telem,
        "gimbal": gimbal
    }

@router.post("/mode")
def set_mode(req: ModeRequest):
    """Switch active mission mode: STANDBY, TRACKING."""
    video_manager.set_mode(req.mode)
    return {"status": "ok", "mode": req.mode.upper()}

@router.post("/tracking/roi")
def set_manual_roi(req: ROIRequest):
    """Start manual ROI tracking with user-drawn box."""
    if len(req.bbox) != 4:
        raise HTTPException(status_code=400, detail="bbox must be [x, y, w, h]")
    success = video_manager.start_manual_roi(tuple(req.bbox))
    return {"status": "ok" if success else "failed", "bbox": req.bbox}

@router.post("/tracking/cancel")
def cancel_manual_tracking():
    """Stop manual tracking and hold position and gimbal orientation."""
    video_manager.cancel_manual_tracking()
    return {"status": "ok", "message": "Tracking cancelled. UAV holding position, Gimbal holding orientation."}

@router.get("/mavlink/serial_ports")
def get_serial_ports():
    """Detect available serial COM/TTY ports across Windows and Linux."""
    import serial.tools.list_ports
    ports = []
    try:
        for p in serial.tools.list_ports.comports():
            ports.append({
                "device": p.device,
                "description": p.description or p.device
            })
    except Exception as e:
        logger.error(f"Error enumerating serial ports: {e}")
    return {"ports": ports}

@router.post("/mavlink/connect")
def connect_mavlink(req: MAVLinkConnectRequest):
    """Connect or switch MAVLink ArduPilot link (UDP or Serial)."""
    conn_str = req.connection_str
    if not conn_str:
        if req.connection_type == "serial":
            conn_str = req.serial_port or "COM3"
        else:
            conn_str = f"udp:{req.ip}:{req.port}"

    baud = req.baud or 57600
    connected = mavlink_ctrl.connect(conn_str, baud)
    return {
        "status": "connected" if connected else "failed",
        "connected": connected,
        "connection_str": conn_str,
        "baud": baud
    }

@router.post("/mavlink/disconnect")
def disconnect_mavlink():
    """Disconnect ArduPilot link."""
    mavlink_ctrl.disconnect()
    return {"status": "disconnected"}

@router.post("/mavlink/mode")
def set_flight_mode(req: FlightModeRequest):
    """Set ArduPilot mode (GUIDED, LOITER, RTL, LAND)."""
    success = mavlink_ctrl.set_mode(req.mode)
    return {"status": "ok" if success else "failed", "mode": req.mode.upper()}

@router.post("/mavlink/arm")
def arm_drone(req: ArmRequest):
    """Arm or Disarm ArduPilot UAV."""
    if not mavlink_ctrl.is_connected:
        raise HTTPException(status_code=400, detail="Chưa kết nối UAV! Vui lòng kết nối với máy bay trước.")
    success = mavlink_ctrl.arm(req.arm)
    return {"status": "ok" if success else "failed", "armed": req.arm}

@router.post("/mavlink/takeoff")
def takeoff_drone(req: TakeoffRequest):
    """Command UAV takeoff to specified altitude (auto-switches to GUIDED and arms)."""
    if not mavlink_ctrl.is_connected:
        raise HTTPException(status_code=400, detail="Chưa kết nối UAV! Vui lòng kết nối với máy bay trước.")
    alt = req.altitude if req.altitude is not None else 2.5
    if alt <= 0.2:
        raise HTTPException(status_code=400, detail="Độ cao cất cánh phải lớn hơn 0.2m")
    success = mavlink_ctrl.takeoff(alt)
    return {"status": "ok" if success else "failed", "altitude": alt}

@router.post("/mavlink/land")
def land_drone():
    """Command UAV to Land immediately."""
    if not mavlink_ctrl.is_connected:
        raise HTTPException(status_code=400, detail="Chưa kết nối UAV! Vui lòng kết nối với máy bay trước.")
    video_manager.cancel_manual_tracking()
    video_manager.set_mode("STANDBY")
    success = mavlink_ctrl.land()
    return {"status": "ok" if success else "failed", "message": "Lệnh hạ cánh đã được gửi."}

@router.post("/mavlink/stop")
def emergency_stop():
    """Command immediate hover/stop and center gimbal."""
    video_manager.cancel_manual_tracking()
    video_manager.set_mode("STANDBY")
    mavlink_ctrl.send_stop()
    gimbal_ctrl.center()
    return {"status": "ok", "message": "Đã dừng khẩn cấp: UAV đứng im, Gimbal về giữa."}

@router.post("/mavlink/nudge")
def manual_nudge(req: ManualNudgeRequest):
    """Manual UAV nudge for operator override."""
    if not mavlink_ctrl.is_connected:
        raise HTTPException(status_code=400, detail="Chưa kết nối UAV! Không thể điều khiển thủ công.")
    d = req.direction.lower()
    spd = req.speed
    if d == "up":
        mavlink_ctrl.send_velocity(0.0, 0.0, -spd, 0.0, "LÊN (THỦ CÔNG)", is_manual=True)
    elif d == "down":
        mavlink_ctrl.send_velocity(0.0, 0.0, spd, 0.0, "XUỐNG (THỦ CÔNG)", is_manual=True)
    elif d == "left":
        mavlink_ctrl.send_velocity(0.0, -spd, 0.0, 0.0, "SANG TRÁI (THỦ CÔNG)", is_manual=True)
    elif d == "right":
        mavlink_ctrl.send_velocity(0.0, spd, 0.0, 0.0, "SANG PHẢI (THỦ CÔNG)", is_manual=True)
    elif d == "yaw_left":
        mavlink_ctrl.send_velocity(0.0, 0.0, 0.0, -spd, "QUAY TRÁI (THỦ CÔNG)", is_manual=True)
    elif d == "yaw_right":
        mavlink_ctrl.send_velocity(0.0, 0.0, 0.0, spd, "QUAY PHẢI (THỦ CÔNG)", is_manual=True)
    else:
        mavlink_ctrl.send_stop()
    return {"status": "ok", "command": d}

@router.post("/settings/rtsp")
def update_rtsp_config(req: RTSPConfigRequest):
    """Update RTSP stream destination (default TCP)."""
    transport = req.transport or "tcp"
    video_manager.set_rtsp_config(req.rtsp_url, transport)
    return {
        "status": "ok",
        "rtsp_url": req.rtsp_url,
        "transport": transport
    }

@router.post("/settings/deadband")
def update_deadband(req: DeadbandRequest):
    """Update deadband box size at center of video."""
    video_manager.set_deadband(req.deadband_x, req.deadband_y)
    return {
        "status": "ok",
        "deadband_x": video_manager.deadband_x,
        "deadband_y": video_manager.deadband_y
    }

@router.post("/settings/tracking_target")
def set_tracking_target(req: TrackingTargetRequest):
    """Switch active tracking actuator: 'gimbal' (camera gimbal) or 'uav' (drone body motion)."""
    video_manager.set_control_target(req.control_target)
    return {"status": "ok", "control_target": video_manager.control_target}

@router.post("/settings/tracking_model")
def set_tracking_model(req: TrackingModelRequest):
    """Switch active tracking model: 'csrt' (OpenCV CSRT) or 'nanotrack' (Deep Learning NanoTrack)."""
    video_manager.set_tracker_model(req.tracker_model)
    return {"status": "ok", "tracker_model": video_manager.tracker_model}

@router.post("/settings/gains")
def update_gains(req: GainsRequest):
    """Tune servoing proportional gains and deadband."""
    if req.kp_sway is not None:
        video_manager.kp_sway = req.kp_sway
    if req.kp_climb is not None:
        video_manager.kp_climb = req.kp_climb
        gimbal_ctrl.kp_pitch = req.kp_climb * 35.0
    if req.kp_yaw is not None:
        video_manager.kp_yaw = req.kp_yaw
        gimbal_ctrl.kp_yaw = req.kp_yaw * 45.0
    if req.deadband_x is not None:
        video_manager.deadband_x = req.deadband_x
    if req.deadband_y is not None:
        video_manager.deadband_y = req.deadband_y
    if req.control_target is not None:
        video_manager.set_control_target(req.control_target)
    if req.tracker_model is not None:
        video_manager.set_tracker_model(req.tracker_model)
    return {
        "status": "ok",
        "message": "Gains updated.",
        "control_target": video_manager.control_target,
        "tracker_model": video_manager.tracker_model
    }

# =============================================================================
# GIMBAL CONTROL ENDPOINTS (CAMERA Z-1MINI GCU PRIVATE PROTOCOL)
# =============================================================================
@router.get("/gimbal/serial_ports")
def get_gimbal_serial_ports():
    """Detect available serial ports for gimbal connection."""
    return {"ports": gimbal_ctrl.get_available_ports()}

@router.post("/gimbal/connect")
def connect_gimbal(req: GimbalConnectRequest):
    """Connect to Camera Gimbal Z-1Mini via Serial port."""
    connected = gimbal_ctrl.connect(req.port, req.baud)
    if connected:
        return {
            "status": "ok",
            "connected": True,
            "port": gimbal_ctrl.port,
            "baud": gimbal_ctrl.baudrate,
            "message": f"Đã kết nối Gimbal tại {gimbal_ctrl.port} ({gimbal_ctrl.baudrate} bps)"
        }
    else:
        err_msg = getattr(gimbal_ctrl, "last_error", "") or "Không thể mở cổng Serial hoặc thiết bị không phản hồi"
        return {
            "status": "failed",
            "connected": False,
            "detail": err_msg,
            "port": req.port,
            "baud": req.baud
        }

@router.post("/gimbal/disconnect")
def disconnect_gimbal():
    """Disconnect Camera Gimbal serial connection."""
    gimbal_ctrl.disconnect()
    return {"status": "ok", "connected": False}

@router.post("/gimbal/center")
def center_gimbal():
    """Command Camera Gimbal to return to center (0, 0, 0)."""
    gimbal_ctrl.center()
    return {"status": "ok", "message": "Lệnh đưa Gimbal về giữa đã gửi."}

@router.post("/gimbal/angles")
def set_gimbal_angles(req: GimbalAnglesRequest):
    """Set manual gimbal angles in degrees."""
    gimbal_ctrl.set_angles(req.pitch, req.yaw, req.roll or 0.0)
    return {"status": "ok", "pitch": req.pitch, "yaw": req.yaw}

@router.get("/gimbal/telemetry")
def get_gimbal_telemetry():
    """Get latest gimbal telemetry state."""
    return gimbal_ctrl.get_telemetry()

@router.websocket("/ws/telemetry")
async def websocket_telemetry(websocket: WebSocket):
    """High-frequency real-time WebSocket for HUD metrics and controls."""
    await websocket.accept()
    try:
        while True:
            vision = video_manager.latest_state
            telem = mavlink_ctrl.get_telemetry()
            gimbal = gimbal_ctrl.get_telemetry()
            payload = {
                "vision": vision,
                "telemetry": telem,
                "gimbal": gimbal,
                "timestamp": asyncio.get_event_loop().time()
            }
            await websocket.send_text(json.dumps(payload))
            await asyncio.sleep(0.05)  # 20 Hz telemetry push
    except WebSocketDisconnect:
        pass
    except Exception:
        pass

