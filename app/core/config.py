import os
from dotenv import load_dotenv
from pydantic import BaseModel

# Automatically load environment variables from .env file if available
load_dotenv()

class Settings(BaseModel):
    # Web server
    HOST: str = os.getenv("HOST", "0.0.0.0")
    PORT: int = int(os.getenv("PORT", "8000"))
    DEBUG: bool = os.getenv("DEBUG", "False").lower() == "true"
    
    # RTSP Camera settings (IP camera / Drone video transmitter)
    RTSP_URL: str = os.getenv("RTSP_URL", "rtsp://192.168.144.108") #rtsp://127.0.0.1:8554/live")
    RTSP_TRANSPORT: str = os.getenv("RTSP_TRANSPORT", "tcp")  # tcp or udp
    FRAME_WIDTH: int = int(os.getenv("FRAME_WIDTH", "640"))
    FRAME_HEIGHT: int = int(os.getenv("FRAME_HEIGHT", "480"))
    CAMERA_FPS: int = int(os.getenv("CAMERA_FPS", "30"))
    
    # Control Target: "gimbal" (Điều khiển Camera Gimbal) hoặc "uav" (Điều khiển Máy bay MAVLink)
    CONTROL_TARGET: str = os.getenv("CONTROL_TARGET", "gimbal")

    # Tracking Model: "csrt" (OpenCV CSRT) hoặc "nanotrack" (Deep Learning ONNX NanoTrack)
    TRACKER_TYPE: str = os.getenv("TRACKER_TYPE", "nanotrack") #csrt nanotrack

    # Camera Gimbal Z-1Mini Serial settings
    GIMBAL_PORT: str = os.getenv("GIMBAL_PORT", "/dev/ttyUSB0")
    GIMBAL_BAUD: int = int(os.getenv("GIMBAL_BAUD", "115200"))

    # ArduPilot MAVLink settings
    MAVLINK_CONNECTION: str = os.getenv("MAVLINK_CONNECTION", "udp:127.0.0.1:14550")
    MAVLINK_BAUD: int = int(os.getenv("MAVLINK_BAUD", "57600"))
    
    # Visual Servoing / PID Gains
    DEADBAND_X: float = 0.10      # Normalized deadband (8% of half-screen)
    DEADBAND_Y: float = 0.15
    KP_YAW: float = 0.6           # Yaw rate gain (rad/s per error unit)
    KP_SWAY: float = 0.8          # Horizontal velocity gain (m/s per error unit)
    KP_CLIMB: float = 0.7         # Vertical velocity gain (m/s per error unit)
    MAX_VELOCITY_XY: float = 1.2  # Max sway speed m/s
    MAX_VELOCITY_Z: float = 0.8   # Max climb/descend speed m/s
    MAX_YAW_RATE: float = 0.6     # Max yaw rate rad/s (~34 deg/s)

    # Takeoff settings
    DEFAULT_TAKEOFF_ALT: float = float(os.getenv("DEFAULT_TAKEOFF_ALT", "1.0"))  # Default takeoff altitude in meters

settings = Settings()
