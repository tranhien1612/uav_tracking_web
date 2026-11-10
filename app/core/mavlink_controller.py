import time
import math
import threading
import logging
from typing import Dict, Any, Optional, Tuple

try:
    from pymavlink import mavutil
    PYMAVLINK_AVAILABLE = True
except ImportError:
    PYMAVLINK_AVAILABLE = False

logger = logging.getLogger("mavlink_controller")
logger.setLevel(logging.INFO)

class MAVLinkController:
    """
    ArduPilot MAVLink controller for UAV telemetry & flight command actuation.
    Decoupled architecture: operates on a dedicated background thread for 20Hz steady flight
    actuation and telemetry reception, completely independent of the camera/vision processing pipeline.
    """
    def __init__(self, connection_str: str = "udp:127.0.0.1:14550", baud: int = 57600):
        self.connection_str = connection_str
        self.baud = baud
        self.master = None
        
        self.is_connected = False
        self.is_armed = False
        self.current_mode = "---"
        self.last_heartbeat = 0.0
        
        # Live Telemetry State
        self.telemetry: Dict[str, Any] = {
            "connected": False,
            "connection_str": self.connection_str,
            "baud": self.baud,
            "mode": "---",
            "armed": False,
            "battery_pct": 0,
            "battery_volt": 0.0,
            "altitude": 0.0,
            "relative_alt": 0.0,
            "heading": 0.0,
            "groundspeed": 0.0,
            "climb_rate": 0.0,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
            "last_command": "ĐỨNG IM",
            "active_velocities": {"vx": 0.0, "vy": 0.0, "vz": 0.0, "yaw_rate": 0.0},
            "last_command_time": 0.0,
            "statustexts": []
        }
        
        # Flight Command Actuation Target: (vx, vy, vz, yaw_rate, timestamp, is_manual)
        self._target_cmd: Tuple[float, float, float, float, float, bool] = (0.0, 0.0, 0.0, 0.0, 0.0, False)
        
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()       # Protects telemetry dict
        self._send_lock = threading.Lock()  # Protects MAVLink socket transmission
        self._cmd_lock = threading.Lock()   # Protects target velocity command buffer

    def connect(self, connection_str: Optional[str] = None, baud: Optional[int] = None) -> bool:
        """Connect to ArduPilot via MAVLink (UDP / Serial)."""
        if connection_str:
            self.connection_str = connection_str
        if baud:
            self.baud = baud

        if not PYMAVLINK_AVAILABLE:
            logger.error("pymavlink is not installed!")
            return False

        try:
            self.disconnect()
            with self._lock:
                self.telemetry["connection_str"] = self.connection_str
                self.telemetry["baud"] = self.baud

            logger.info(f"Connecting to ArduPilot: {self.connection_str} (baud: {self.baud})...")
            self.master = mavutil.mavlink_connection(self.connection_str, baud=self.baud)

            # Quick wait for immediate heartbeat feedback (up to 1.5s) before starting worker
            msg = self.master.wait_heartbeat(timeout=1.5)
            if msg:
                self._on_heartbeat(msg)
                self._request_data_streams()
                self._start_worker()
                logger.info(f"ArduPilot connected. Mode: {self.current_mode}")
                return True
            elif not self.connection_str.startswith("udp:") and not self.connection_str.startswith("tcp:"):
                # Probe alternative baud rates on serial
                alt_bauds = [115200, 57600, 921600]
                if self.baud in alt_bauds:
                    alt_bauds.remove(self.baud)
                for alt_b in alt_bauds:
                    logger.info(f"Probing alternative baud {alt_b} on {self.connection_str}...")
                    self.disconnect()
                    try:
                        test_master = mavutil.mavlink_connection(self.connection_str, baud=alt_b)
                        alt_msg = test_master.wait_heartbeat(timeout=1.5)
                        if alt_msg:
                            self.baud = alt_b
                            self.master = test_master
                            self._on_heartbeat(alt_msg)
                            self._request_data_streams()
                            self._start_worker()
                            logger.info(f"ArduPilot auto-detected baud {alt_b}. Mode: {self.current_mode}")
                            return True
                        test_master.close()
                    except Exception:
                        pass
                return False
            else:
                # UDP link established, listen for incoming packets in worker loop
                self._start_worker()
                logger.info(f"Connection opened to {self.connection_str}. Listening for heartbeat in background...")
                return True
        except Exception as e:
            logger.error(f"MAVLink connection error: {e}")
            self.disconnect()
            return False

    def _start_worker(self):
        """Start the single background thread for telemetry & UAV control."""
        self._running = True
        self._thread = threading.Thread(target=self._uav_worker_loop, daemon=True)
        self._thread.start()

    def disconnect(self):
        """Disconnect MAVLink connection and stop background worker thread."""
        self._running = False
        if self._thread and self._thread.is_alive() and self._thread != threading.current_thread():
            self._thread.join(timeout=1.0)
        self._thread = None

        with self._send_lock:
            if self.master:
                try:
                    self.master.close()
                except Exception:
                    pass
                self.master = None

        self.is_connected = False
        self.is_armed = False
        self.current_mode = "---"
        with self._lock:
            self.telemetry["connected"] = False
            self.telemetry["mode"] = "---"
            self.telemetry["armed"] = False
            self.telemetry["connection_str"] = self.connection_str
            self.telemetry["baud"] = self.baud
        with self._cmd_lock:
            self._target_cmd = (0.0, 0.0, 0.0, 0.0, 0.0, False)
        logger.info("MAVLink link closed.")

    def _request_data_streams(self):
        """Request periodic telemetry streams from ArduPilot."""
        if not self.master:
            return
        try:
            with self._send_lock:
                self.master.mav.request_data_stream_send(
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_DATA_STREAM_ALL,
                    4, 1
                )
        except Exception as e:
            logger.warning(f"Failed to request data streams: {e}")

    def set_mode(self, mode_name: str) -> bool:
        """Change ArduPilot flight mode (e.g. GUIDED, LOITER, RTL, LAND)."""
        mode_name = mode_name.upper()
        if not self.master or not self.is_connected:
            logger.warning(f"Cannot change mode to {mode_name}: UAV not connected.")
            return False

        try:
            mode_id = self.master.mode_mapping().get(mode_name)
            if mode_id is None:
                logger.error(f"Unknown flight mode: {mode_name}")
                return False

            with self._send_lock:
                self.master.mav.set_mode_send(
                    self.master.target_system,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    mode_id
                )
            self.current_mode = mode_name
            with self._lock:
                self.telemetry["mode"] = mode_name
            logger.info(f"Mode change request sent: {mode_name}")
            return True
        except Exception as e:
            logger.error(f"Error setting mode {mode_name}: {e}")
            return False

    def arm(self, arm_drone: bool = True) -> bool:
        """Arm or disarm the UAV."""
        if not self.master or not self.is_connected:
            logger.warning("Cannot ARM/DISARM: UAV not connected.")
            return False

        try:
            with self._send_lock:
                self.master.mav.command_long_send(
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0,
                    1 if arm_drone else 0,
                    0, 0, 0, 0, 0, 0
                )
            self.is_armed = arm_drone
            with self._lock:
                self.telemetry["armed"] = arm_drone
            logger.info(f"Command ARM/DISARM sent: {'ARM' if arm_drone else 'DISARM'}")
            return True
        except Exception as e:
            logger.error(f"Error arming/disarming: {e}")
            return False

    def ensure_guided_mode(self) -> bool:
        """
        Check if the UAV is currently in GUIDED mode.
        If not, automatically send a set_mode command to switch to GUIDED before proceeding.
        """
        if not self.master or not self.is_connected:
            return False
        
        mode = (self.current_mode or "").upper()
        if "GUID" in mode:
            return True
        
        logger.info(f"UAV current mode is '{mode}', auto-switching to GUIDED mode...")
        return self.set_mode("GUIDED")

    def takeoff(self, altitude: float = 1.0) -> bool:
        """
        Command UAV to take off to the specified altitude in meters.
        Ensures GUIDED mode, arms the UAV if disarmed, then sends MAV_CMD_NAV_TAKEOFF.
        """
        if not self.master or not self.is_connected:
            logger.warning("Cannot Takeoff: UAV not connected.")
            return False

        try:
            # 1. Ensure GUIDED mode
            self.ensure_guided_mode()
            time.sleep(0.1)

            # 2. Arm if disarmed
            if not self.is_armed:
                logger.info("Arming motors before takeoff...")
                self.arm(True)
                time.sleep(0.5)

            # 3. Send takeoff command
            logger.info(f"Sending TAKEOFF command to altitude: {altitude}m")
            with self._send_lock:
                self.master.mav.command_long_send(
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                    0,
                    0, 0, 0, 0, 0, 0,
                    float(altitude)
                )
            with self._lock:
                self.telemetry["last_command"] = f"CẤT CÁNH ({altitude}m)"
                self.telemetry["last_command_time"] = time.time()
            return True
        except Exception as e:
            logger.error(f"Error executing takeoff: {e}")
            return False

    def land(self) -> bool:
        """
        Command UAV to land immediately (switch to LAND mode or send MAV_CMD_NAV_LAND).
        """
        if not self.master or not self.is_connected:
            logger.warning("Cannot Land: UAV not connected.")
            return False

        try:
            logger.info("Sending LAND command...")
            # Try switching to LAND mode first
            success = self.set_mode("LAND")
            if not success:
                with self._send_lock:
                    self.master.mav.command_long_send(
                        self.master.target_system,
                        self.master.target_component,
                        mavutil.mavlink.MAV_CMD_NAV_LAND,
                        0,
                        0, 0, 0, 0, 0, 0, 0
                    )
            with self._cmd_lock:
                self._target_cmd = (0.0, 0.0, 0.0, 0.0, time.time(), True)
            with self._lock:
                self.telemetry["last_command"] = "HẠ CÁNH (LAND)"
                self.telemetry["active_velocities"] = {"vx": 0.0, "vy": 0.0, "vz": 0.0, "yaw_rate": 0.0}
                self.telemetry["last_command_time"] = time.time()
            return True
        except Exception as e:
            logger.error(f"Error executing land: {e}")
            return False

    def send_velocity(self, vx: float, vy: float, vz: float, yaw_rate: float = 0.0, description: str = "", is_manual: bool = False):
        """
        Set target velocity in BODY frame (MAV_FRAME_BODY_NED):
        - vx: Forward (+) / Backward (-) in m/s
        - vy: Right (+) / Left (-) in m/s
        - vz: Down (+) / Up (-) in m/s
        - yaw_rate: Clockwise (+) / Counter-Clockwise (-) in rad/s
        
        Non-blocking: immediately updates the command buffer; the dedicated background UAV thread
        dispatches velocity packets at a steady 20Hz rate, completely isolating the camera stream.
        """
        if not description:
            actions = []
            if vz < -0.05: actions.append("LÊN")
            elif vz > 0.05: actions.append("XUỐNG")
            if vy > 0.05: actions.append("SANG PHẢI")
            elif vy < -0.05: actions.append("SANG TRÁI")
            if yaw_rate > 0.05: actions.append("QUAY PHẢI")
            elif yaw_rate < -0.05: actions.append("QUAY TRÁI")
            description = " + ".join(actions) if actions else "ĐỨNG IM"

        now = time.time()
        with self._cmd_lock:
            self._target_cmd = (float(vx), float(vy), float(vz), float(yaw_rate), now, is_manual)

        with self._lock:
            self.telemetry["last_command"] = description
            self.telemetry["active_velocities"] = {
                "vx": round(vx, 3),
                "vy": round(vy, 3),
                "vz": round(vz, 3),
                "yaw_rate": round(yaw_rate, 3)
            }
            self.telemetry["last_command_time"] = now

        # Immediate dispatch for manual nudges or stop commands for zero-latency response
        if self.master and self.is_connected:
            self._send_velocity_packet(vx, vy, vz, yaw_rate)

    def send_stop(self):
        """Command UAV to hover / hold position immediately (Zero velocity)."""
        self.send_velocity(0.0, 0.0, 0.0, 0.0, description="ĐỨNG IM (HOVER)", is_manual=True)

    def _send_velocity_packet(self, vx: float, vy: float, vz: float, yaw_rate: float):
        """Low-level MAVLink SET_POSITION_TARGET_LOCAL_NED packet transmission."""
        if not self.master or not self.is_connected:
            return

        try:
            self.ensure_guided_mode()
            type_mask = 0b010111000111  # Use velocity (vx, vy, vz) and yaw_rate

            with self._send_lock:
                self.master.mav.set_position_target_local_ned_send(
                    0,
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_FRAME_BODY_NED,
                    type_mask,
                    0, 0, 0,
                    vx, vy, vz,
                    0, 0, 0,
                    0, yaw_rate
                )
        except Exception as e:
            logger.debug(f"Error sending velocity packet: {e}")

    def _on_heartbeat(self, msg):
        """Handle incoming MAVLink HEARTBEAT."""
        was_connected = self.is_connected
        self.last_heartbeat = time.time()
        self.is_connected = True
        self.is_armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        self.current_mode = mavutil.mode_string_v10(msg)
        if not was_connected:
            self._request_data_streams()
        with self._lock:
            self.telemetry["connected"] = True
            self.telemetry["connection_str"] = self.connection_str
            self.telemetry["baud"] = self.baud
            self.telemetry["armed"] = self.is_armed
            self.telemetry["mode"] = self.current_mode

    def _uav_worker_loop(self):
        """
        Single dedicated UAV background loop:
        1. Ingests all telemetry packets from ArduPilot.
        2. Sends 20Hz continuous velocity commands (with watchdog safety hover if vision drops).
        """
        last_actuation_time = 0.0

        while self._running and self.master:
            try:
                now = time.time()

                # 1. Enforce 3.0s heartbeat timeout
                if self.is_connected and (now - self.last_heartbeat > 3.0):
                    self.is_connected = False
                    self.is_armed = False
                    self.current_mode = "MẤT KẾT NỐI"
                    with self._lock:
                        self.telemetry["connected"] = False
                        self.telemetry["armed"] = False
                        self.telemetry["mode"] = "MẤT KẾT NỐI"
                    logger.warning("Heartbeat timeout (> 3.0s). Link disconnected.")

                # 2. Ingest all pending incoming MAVLink messages (non-blocking drain)
                while True:
                    msg = self.master.recv_match(blocking=False)
                    if not msg:
                        break
                    self._handle_telemetry_msg(msg)

                # 3. Steady 20Hz Flight Actuation Loop (every 50ms)
                if self.is_connected and (now - last_actuation_time >= 0.05):
                    last_actuation_time = now
                    self._actuate_flight_control(now)

                time.sleep(0.01)  # Yield CPU and maintain high resolution
            except Exception as e:
                logger.debug(f"UAV worker loop exception: {e}")
                time.sleep(0.05)

    def _actuate_flight_control(self, now: float):
        """Steady 20Hz flight command actuator with Safety Watchdog."""
        with self._cmd_lock:
            vx, vy, vz, yaw_rate, cmd_time, is_manual = self._target_cmd

        # Safety Watchdog: if command is from vision and stale (> 0.5s without update), auto-hover
        if not is_manual and (now - cmd_time > 0.5):
            vx, vy, vz, yaw_rate = 0.0, 0.0, 0.0, 0.0

        # Only send velocity stream if armed or currently commanded
        is_moving = abs(vx) > 0.01 or abs(vy) > 0.01 or abs(vz) > 0.01 or abs(yaw_rate) > 0.01
        if self.is_armed or is_moving:
            self._send_velocity_packet(vx, vy, vz, yaw_rate)

    def _handle_telemetry_msg(self, msg):
        """Parse individual MAVLink messages into telemetry state."""
        msg_type = msg.get_type()
        if msg_type == "HEARTBEAT":
            self._on_heartbeat(msg)
        elif msg_type == "ATTITUDE":
            with self._lock:
                self.telemetry["roll"] = round(math.degrees(msg.roll), 1)
                self.telemetry["pitch"] = round(math.degrees(msg.pitch), 1)
                self.telemetry["yaw"] = round(math.degrees(msg.yaw), 1)
        elif msg_type == "VFR_HUD":
            with self._lock:
                self.telemetry["heading"] = round(msg.heading, 1)
                self.telemetry["altitude"] = round(msg.alt, 2)
                self.telemetry["groundspeed"] = round(msg.groundspeed, 2)
                self.telemetry["climb_rate"] = round(msg.climb, 2)
        elif msg_type == "SYS_STATUS":
            with self._lock:
                if msg.battery_remaining != -1:
                    self.telemetry["battery_pct"] = msg.battery_remaining
                self.telemetry["battery_volt"] = round(msg.voltage_battery / 1000.0, 2)
        elif msg_type == "GLOBAL_POSITION_INT":
            with self._lock:
                self.telemetry["relative_alt"] = round(msg.relative_alt / 1000.0, 2)
        elif msg_type == "STATUSTEXT":
            raw_text = getattr(msg, "text", "")
            if isinstance(raw_text, bytes):
                raw_text = raw_text.decode("utf-8", errors="ignore")
            clean_text = str(raw_text).strip("\x00 \t\r\n")
            severity = getattr(msg, "severity", 6)

            sev_names = {0: "EMERGENCY", 1: "ALERT", 2: "CRITICAL", 3: "ERROR", 4: "WARNING", 5: "NOTICE", 6: "INFO", 7: "DEBUG"}
            sev_name = sev_names.get(severity, "INFO")
            log_type = "error" if severity in (0, 1, 2, 3) else ("warn" if severity == 4 else ("notice" if severity == 5 else "info"))

            st_item = {
                "text": clean_text,
                "severity": severity,
                "severity_name": sev_name,
                "type": log_type,
                "time": time.time()
            }
            logger.info(f"UAV STATUSTEXT [{sev_name}]: {clean_text}")
            with self._lock:
                st_list = self.telemetry.setdefault("statustexts", [])
                st_list.append(st_item)
                if len(st_list) > 100:
                    self.telemetry["statustexts"] = st_list[-100:]

    def get_telemetry(self) -> Dict[str, Any]:
        """Return current telemetry state."""
        with self._lock:
            if self.is_connected and (time.time() - self.last_heartbeat > 3.0):
                self.is_connected = False
                self.is_armed = False
                self.telemetry["connected"] = False
                self.telemetry["armed"] = False
                self.telemetry["mode"] = "MẤT KẾT NỐI"
            telem = dict(self.telemetry)
            if "statustexts" in telem and len(telem["statustexts"]) > 15:
                telem["statustexts"] = telem["statustexts"][-15:]
            return telem

mavlink_ctrl = MAVLinkController()
