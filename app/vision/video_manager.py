import cv2
import numpy as np
import time
import threading
from typing import Optional, Dict, Any, Tuple
from app.core.config import settings
from app.core.mavlink_controller import mavlink_ctrl
from app.core.gimbal_controller import gimbal_ctrl
from app.vision.tracker import ManualTracker
from app.vision.rtsp_reader import RTSPStreamReader

class OperationMode:
    STANDBY = "STANDBY"
    TRACKING = "TRACKING"

class VideoManager:
    """
    Dedicated Video Processing Pipeline:
    - Reads from low-latency socket-draining RTSP reader (Zero buffer build-up).
    - Runs vision algorithms (Manual ROI Tracker).
    - Fast lightweight JPEG encoding (quality 72, preview-scale if resolution is 1080p+).
    - Dispatches velocity targets to UAV controller non-blockingly.
    """
    def __init__(self):
        self.mode = OperationMode.STANDBY
        
        # Stream reader helper (continuous socket drainer)
        self.rtsp_reader = RTSPStreamReader(settings.RTSP_URL, settings.RTSP_TRANSPORT)
        
        # Tracking Model Algorithm: "csrt" (default) or "nanotrack"
        self.tracker_model: str = getattr(settings, "TRACKER_TYPE", "csrt").lower()
        if self.tracker_model not in ["csrt", "nanotrack"]:
            self.tracker_model = "csrt"

        # Vision engines
        self.manual_tracker = ManualTracker(tracker_type=self.tracker_model)
        
        # Frame dimensions
        self.width = settings.FRAME_WIDTH
        self.height = settings.FRAME_HEIGHT
        
        # Servoing PID / Proportional gains
        self.kp_sway = settings.KP_SWAY
        self.kp_climb = settings.KP_CLIMB
        self.kp_yaw = settings.KP_YAW
        self.deadband_x = settings.DEADBAND_X
        self.deadband_y = settings.DEADBAND_Y
        self.max_v_xy = settings.MAX_VELOCITY_XY
        self.max_v_z = settings.MAX_VELOCITY_Z
        self.max_yaw_rate = settings.MAX_YAW_RATE
        
        # Tracking Actuator Mode: "gimbal" (default) or "uav"
        self.control_target: str = getattr(settings, "CONTROL_TARGET", "gimbal").lower()
        if self.control_target not in ["gimbal", "uav"]:
            self.control_target = "gimbal"

        # Live state & clean JPEG buffer
        self.current_frame = None
        self._latest_jpeg: Optional[bytes] = None
        self._jpeg_id = 0
        self.is_running = False
        self._lock = threading.Lock()
        self._cached_raw_dim: Optional[Tuple[int, int]] = None
        self._cached_target_wh: Optional[Tuple[int, int]] = None

        # Pre-encoded placeholder JPEG for when camera stream is waiting / offline
        self._placeholder_frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        _, placeholder_enc = cv2.imencode('.jpg', self._placeholder_frame, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
        self._placeholder_jpeg = placeholder_enc.tobytes() if placeholder_enc is not None else b''
        
        self.latest_state: Dict[str, Any] = {
            "mode": self.mode,
            "status_text": "HỆ THỐNG SẴN SÀNG",
            "active_command": "ĐỨNG IM",
            "commands": {"up": False, "down": False, "left": False, "right": False, "yaw_left": False, "yaw_right": False, "stop": True},
            "error_x": 0.0,
            "error_y": 0.0,
            "is_centered": False,
            "manual_tracking_active": False,
            "tracking_bbox": None,
            "deadband_x": self.deadband_x,
            "deadband_y": self.deadband_y,
            "fps": 30.0,
            "rtsp_url": settings.RTSP_URL,
            "rtsp_connected": False,
            "rtsp_status": "CHƯA KẾT NỐI",
            "control_target": self.control_target,
            "tracker_model": self.tracker_model
        }
        
        self._thread: Optional[threading.Thread] = None

    def start(self):
        """Start the video stream grabber and processing pipeline."""
        if self.is_running:
            return
        self.is_running = True
        self.rtsp_reader.start()
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()

    def stop(self):
        """Stop processing and release resources cleanly."""
        self.is_running = False
        if self._thread and self._thread.is_alive() and self._thread != threading.current_thread():
            self._thread.join(timeout=1.0)
        self._thread = None
        self.rtsp_reader.stop()
        mavlink_ctrl.send_stop()
        gimbal_ctrl.stop_tracking()

    def set_rtsp_config(self, rtsp_url: str, transport: str = "tcp"):
        """Update RTSP stream destination and transport."""
        with self._lock:
            if rtsp_url:
                self.rtsp_reader.set_rtsp_url(rtsp_url, transport)

    def set_deadband(self, deadband_x: float, deadband_y: float):
        """Update deadband limits (0.01 - 0.50)."""
        with self._lock:
            self.deadband_x = float(np.clip(deadband_x, 0.01, 0.50))
            self.deadband_y = float(np.clip(deadband_y, 0.01, 0.50))

    def set_control_target(self, target: str):
        """Switch active tracking actuator: 'gimbal' or 'uav'."""
        target = target.lower()
        if target not in ["gimbal", "uav"]:
            return
        with self._lock:
            self.control_target = target
            if target == "uav":
                gimbal_ctrl.stop_tracking()
            elif target == "gimbal":
                mavlink_ctrl.send_stop()
            self.latest_state["control_target"] = self.control_target

    def set_tracker_model(self, model_name: str):
        """Switch active tracking model algorithm: 'csrt' or 'nanotrack'."""
        model_name = model_name.lower().strip()
        if model_name not in ["csrt", "nanotrack"]:
            return
        with self._lock:
            self.tracker_model = model_name
            self.manual_tracker.set_tracker_type(model_name)
            self.latest_state["tracker_model"] = self.tracker_model

    def set_mode(self, mode: str):
        """Switch mode: STANDBY, TRACKING."""
        mode = mode.upper()
        if mode not in [OperationMode.STANDBY, OperationMode.TRACKING]:
            return
        
        with self._lock:
            self.mode = mode
            if mode == OperationMode.STANDBY:
                self.manual_tracker.stop()
                mavlink_ctrl.send_stop()
                gimbal_ctrl.stop_tracking()
            elif mode == OperationMode.TRACKING:
                mavlink_ctrl.send_stop()
                gimbal_ctrl.stop_tracking()

    def start_manual_roi(self, bbox: Tuple[int, int, int, int]) -> bool:
        """Start tracking a user-selected ROI box [x, y, w, h]."""
        with self._lock:
            if self.current_frame is None or not self.rtsp_reader.is_connected:
                return False
            self.mode = OperationMode.TRACKING
            return self.manual_tracker.start_tracking(self.current_frame, bbox)

    def cancel_manual_tracking(self):
        """Cancel manual tracking and command hover / stop gimbal."""
        with self._lock:
            self.manual_tracker.stop()
            mavlink_ctrl.send_stop()
            gimbal_ctrl.stop_tracking()

    def _run_loop(self):
        """
        Continuous video processing & visual servoing pipeline.
        Consumes the freshest frame directly from the socket drainer.
        """
        prev_time = time.time()

        while self.is_running:
            # 1. Grab latest frame synchronized to camera frame rate (Waits up to 33ms)
            ret, raw_frame, is_new = self.rtsp_reader.read_latest(timeout=0.033)
            if not ret or raw_frame is None:
                # Camera offline / waiting: reuse pre-encoded placeholder to save 100% CPU
                with self._lock:
                    self.current_frame = self._placeholder_frame
                    self._latest_jpeg = self._placeholder_jpeg
                    self._jpeg_id += 1
                    self.latest_state.update({
                        "mode": self.mode,
                        "status_text": self.rtsp_reader.status_msg,
                        "fps": 0.0,
                        "rtsp_url": self.rtsp_reader.rtsp_url,
                        "rtsp_connected": False,
                        "rtsp_status": self.rtsp_reader.status_msg
                    })
                time.sleep(0.08)
                continue

            if not is_new:
                # Frame unchanged since last loop: sleep briefly to avoid redundant processing
                time.sleep(0.005)
                continue

            # Standardize frame resolution: if camera feed is Full-HD (>960 width),
            # downscale to 960px width for both web streaming and vision processing.
            raw_h, raw_w = raw_frame.shape[:2]
            if raw_w > 960:
                if self._cached_raw_dim != (raw_h, raw_w):
                    self._cached_raw_dim = (raw_h, raw_w)
                    self._cached_target_wh = (960, int(round(raw_h * (960.0 / raw_w))))
                frame = cv2.resize(raw_frame, self._cached_target_wh, interpolation=cv2.INTER_LINEAR)
            else:
                frame = raw_frame

            if frame.shape[1] != self.width or frame.shape[0] != self.height:
                self.width = frame.shape[1]
                self.height = frame.shape[0]

            # 2. Fast JPEG Encoding for Web Stream (Directly from standardized frame)
            ret_enc, jpeg_enc = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70, int(cv2.IMWRITE_JPEG_OPTIMIZE), 0])

            with self._lock:
                self.current_frame = frame
                if ret_enc:
                    self._latest_jpeg = jpeg_enc.tobytes()
                    self._jpeg_id += 1

            now = time.time()
            dt = max(0.001, now - prev_time)
            fps = 1.0 / dt
            prev_time = now

            # 3. Vision Algorithm Execution based on Mode
            status_text = "ĐỨNG IM"
            err_x, err_y = 0.0, 0.0
            is_centered = True
            
            target_vy = 0.0
            target_vz = 0.0
            target_yaw = 0.0

            cmd_up = False
            cmd_down = False
            cmd_left = False
            cmd_right = False
            cmd_yaw_left = False
            cmd_yaw_right = False
            cmd_stop = True

            if not self.rtsp_reader.is_connected:
                status_text = f"ĐANG CHỜ TÍN HIỆU CAMERA: {self.rtsp_reader.rtsp_url}"

            elif self.mode == OperationMode.STANDBY:
                status_text = "CHẾ ĐỘ NGHỈ (STANDBY) - UAV ĐỨNG IM"

            elif self.mode == OperationMode.TRACKING:
                if self.manual_tracker.is_tracking:
                    success, bbox, err_x, err_y, is_centered, trk_status = self.manual_tracker.update(
                        frame, self.width, self.height, self.deadband_x, self.deadband_y
                    )
                    status_text = trk_status

                    if success:
                        if self.control_target == "gimbal":
                            # 1. Điều khiển Gimbal camera bám sát tâm tuyệt đối (0, 0) ngay lập tức
                            gimbal_ctrl.track_target(err_x, err_y)
                            cmd_stop = True
                            if not gimbal_ctrl.is_connected:
                                status_text = f"{trk_status} (CHƯA KẾT NỐI GIMBAL)"
                            else:
                                status_text = f"{trk_status} (GIMBAL)"
                        else:
                            # 2. Không điều khiển gimbal, điều khiển UAV bay (lên/xuống/trái/phải) để bám tâm
                            gimbal_ctrl.stop_tracking()
                            if not is_centered:
                                target_vy, target_vz, target_yaw, cmd_up, cmd_down, cmd_left, cmd_right, cmd_yaw_left, cmd_yaw_right = self._compute_visual_servo_commands(err_x, err_y)
                                cmd_stop = False
                            else:
                                cmd_stop = True
                            status_text = f"{trk_status} (UAV)"
                    else:
                        cmd_stop = True
                        gimbal_ctrl.stop_tracking()
                else:
                    status_text = "CHỜ KHOANH VÙNG VẬT THỂ"
                    cmd_stop = True
                    gimbal_ctrl.stop_tracking()

            # 4. Non-blocking command dispatch to UAV controller
            if self.mode != OperationMode.STANDBY and self.rtsp_reader.is_connected and not cmd_stop:
                mavlink_ctrl.send_velocity(0.0, target_vy, target_vz, target_yaw, description=status_text, is_manual=False)
            elif self.mode != OperationMode.STANDBY:
                mavlink_ctrl.send_velocity(0.0, 0.0, 0.0, 0.0, description=status_text, is_manual=False)

            # 5. State updates for telemetry pushes
            with self._lock:
                self.latest_state.update({
                    "mode": self.mode,
                    "status_text": status_text,
                    "active_command": mavlink_ctrl.telemetry.get("last_command", "ĐỨNG IM"),
                    "commands": {
                        "up": cmd_up, "down": cmd_down,
                        "left": cmd_left, "right": cmd_right,
                        "yaw_left": cmd_yaw_left, "yaw_right": cmd_yaw_right,
                        "stop": cmd_stop
                    },
                    "error_x": round(err_x, 3),
                    "error_y": round(err_y, 3),
                    "is_centered": is_centered,
                    "manual_tracking_active": self.manual_tracker.is_tracking,
                    "tracking_bbox": [int(v) for v in self.manual_tracker.bbox] if (self.mode == OperationMode.TRACKING and self.manual_tracker.is_tracking and self.manual_tracker.bbox) else None,
                    "deadband_x": round(self.deadband_x, 3),
                    "deadband_y": round(self.deadband_y, 3),
                    "fps": round(fps, 1),
                    "rtsp_url": self.rtsp_reader.rtsp_url,
                    "rtsp_connected": self.rtsp_reader.is_connected,
                    "rtsp_status": self.rtsp_reader.status_msg,
                    "control_target": self.control_target,
                    "tracker_model": self.tracker_model
                })

    def _compute_visual_servo_commands(self, err_x: float, err_y: float):
        vy = 0.0
        vz = 0.0
        yaw_rate = 0.0

        cmd_up = False
        cmd_down = False
        cmd_left = False
        cmd_right = False
        cmd_yaw_left = False
        cmd_yaw_right = False

        if abs(err_x) > self.deadband_x:
            vy = float(np.clip(self.kp_sway * err_x, -self.max_v_xy, self.max_v_xy))
            yaw_rate = float(np.clip(self.kp_yaw * err_x, -self.max_yaw_rate, self.max_yaw_rate))
            if err_x > 0:
                cmd_right = True
                cmd_yaw_right = True
            else:
                cmd_left = True
                cmd_yaw_left = True

        if abs(err_y) > self.deadband_y:
            vz = float(np.clip(self.kp_climb * err_y, -self.max_v_z, self.max_v_z))
            if err_y > 0:
                cmd_down = True
            else:
                cmd_up = True

        return vy, vz, yaw_rate, cmd_up, cmd_down, cmd_left, cmd_right, cmd_yaw_left, cmd_yaw_right

    def get_jpeg_with_id(self) -> Tuple[int, Optional[bytes]]:
        """Return (frame_sequence_id, jpeg_bytes) to eliminate redundant frame streaming."""
        with self._lock:
            return self._jpeg_id, self._latest_jpeg

video_manager = VideoManager()