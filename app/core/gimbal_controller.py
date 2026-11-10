#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
=============================================================================
AERO-TRACK GCS | Z-1Mini Camera Gimbal Controller (GCU Private Protocol V2.0)
=============================================================================
Thư viện điều khiển Gimbal camera Z-1Mini tích hợp trong hệ thống AeroTrack:
- Chạy 1 luồng nền duy nhất (Dedicated Background Thread) giao tiếp Serial Q&A
  chu kỳ 35Hz chuẩn tài liệu, đảm bảo không nghẽn/lag và điều khiển cực kỳ mượt mà.
- Hỗ trợ Visual Servoing Tracking: Tự động điều khiển Pitch & Yaw để giữ vật thể
  được khoanh vùng (ROI) luôn nằm chính giữa khung hình camera.
- Đệm gói tin Null Command (0x00) trước & sau mọi lệnh giúp GCU nhận lệnh 100%.
- Tự động quét và kết nối cổng Serial (/dev/ttyUSB*, /dev/ttyACM*, COM*).
=============================================================================
"""

import sys
import time
import math
import struct
import threading
import glob
import queue
import logging
from typing import Optional, Dict, Any, Tuple

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None

logger = logging.getLogger("gimbal_controller")

# =============================================================================
# BẢNG TRA CRC-16 (Theo mục CRC Function trong tài liệu GCU Private Protocol)
# =============================================================================
CRC16_TABLE = [
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF
]

def calculate_crc16(data: bytes) -> int:
    """Tính CRC-16 theo chuẩn Xianfei GCU Private Protocol."""
    crc = 0
    for byte in data:
        da = (crc >> 12) & 0x0F
        crc = (crc << 4) & 0xFFFF
        crc ^= CRC16_TABLE[da ^ (byte >> 4)]

        da = (crc >> 12) & 0x0F
        crc = (crc << 4) & 0xFFFF
        crc ^= CRC16_TABLE[da ^ (byte & 0x0F)]
    return crc


# =============================================================================
# MÃ LỆNH ĐIỀU KHIỂN (GCU Order Codes)
# =============================================================================
CMD_NULL            = 0x00  # Lệnh rỗng duy trì chu kỳ Q&A
CMD_CALIBRATION     = 0x01  # Hiệu chuẩn gimbal
CMD_NEUTRAL         = 0x03  # Đưa về vị trí cân bằng
CMD_ANGLE_CONTROL   = 0x10  # Điều khiển góc Euler Pitch/Roll, Yaw tương đối
CMD_HEAD_LOCK       = 0x11  # Khóa hướng (Điều khiển vận tốc góc Pitch, Yaw)
CMD_HEAD_FOLLOW     = 0x12  # Bám theo thân máy bay
CMD_ORTHOVIEW       = 0x13  # Chúc thẳng đứng 90 độ xuống mặt đất
CMD_EULER_CONTROL   = 0x14  # Điều khiển góc Euler tuyệt đối
CMD_SHUTTER         = 0x20  # Chụp ảnh (0x01)
CMD_RECORD          = 0x21  # Bắt đầu / Dừng quay video (0x01)
CMD_ZOOM_IN         = 0x22  # Zoom in liên tục
CMD_ZOOM_OUT        = 0x23  # Zoom out liên tục
CMD_ZOOM_STOP       = 0x24  # Dừng Zoom
CMD_ZOOM_SPECIFIED  = 0x25  # Zoom tới độ phóng đại cụ thể
CMD_OSD             = 0x73  # Bật/Tắt OSD
CMD_TARGET_DETECT   = 0x75  # Bật/Tắt AI Target Detection


class GimbalController:
    """
    Quản lý kết nối Serial và điều khiển Gimbal Camera Z-1Mini.
    Sử dụng 1 luồng nền duy nhất giao tiếp 35Hz mượt mà.
    """
    def __init__(self, port: Optional[str] = None, baudrate: int = 115200, freq_hz: float = 35.0):
        self.port = port
        self.baudrate = baudrate
        self.freq_hz = max(30.0, min(50.0, freq_hz))
        self.interval = 1.0 / self.freq_hz

        self.serial_conn: Optional[serial.Serial] = None
        self.is_connected = False
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self.last_error = ""

        # Hàng đợi lệnh điều khiển với đệm Null (0x00)
        self._cmd_queue = queue.Queue()

        # Trạng thái điều khiển vận tốc / góc liên tục
        self._roll_ctrl_cdeg = 0
        self._pitch_ctrl_cdeg = 0
        self._yaw_ctrl_cdeg = 0
        self._control_valid = False
        self._is_active_tracking = False
        self._tracking_start_time = 0.0
        self._target_pitch = 0.0
        self._target_yaw = 0.0
        self._last_track_time = 0.0
        self._filtered_spd_yaw = 0.0
        self._filtered_spd_pitch = 0.0

        # Hệ số điều khiển bám tâm mục tiêu mượt mà (Smooth Damped Visual Servoing)
        self.kp_pitch = 22.0       # Tốc độ góc deg/s trên mỗi đơn vị sai số normalized
        self.kp_yaw = 28.0         # Tốc độ góc deg/s trên mỗi đơn vị sai số normalized
        self.max_pitch_speed = 25.0  # Giới hạn tốc độ Pitch deg/s (tránh giật vọt lố)
        self.max_yaw_speed = 35.0    # Giới hạn tốc độ Yaw deg/s (tránh giật vọt lố)

        # Telemetry phản hồi từ gimbal
        self.telemetry: Dict[str, Any] = {
            "connected": False,
            "port": self.port or "None",
            "baudrate": self.baudrate,
            "work_mode": 0,
            "work_mode_str": "Chưa kết nối",
            "pitch": 0.0,
            "yaw": 0.0,
            "roll": 0.0,
            "rel_pitch": 0.0,
            "rel_yaw": 0.0,
            "rel_roll": 0.0,
            "abs_pitch": 0.0,
            "abs_yaw": 0.0,
            "abs_roll": 0.0,
            "ang_vel_x": 0.0,
            "ang_vel_y": 0.0,
            "ang_vel_z": 0.0,
            "zoom_rate": 1.0,
            "is_recording": False,
            "is_tracking": False,
            "target_distance": 0.0,
            "last_cmd": 0,
            "last_cmd_status": -1,
            "packets_sent": 0,
            "packets_received": 0,
            "last_updated": 0.0
        }

    @staticmethod
    def get_available_ports() -> list:
        """Quét và trả về danh sách cổng Serial khả dụng."""
        ports = []
        if serial and hasattr(serial.tools, 'list_ports'):
            try:
                for p in serial.tools.list_ports.comports():
                    ports.append({
                        "device": p.device,
                        "description": p.description or p.device
                    })
            except Exception as e:
                logger.error(f"Lỗi khi quét cổng serial: {e}")
        return ports

    def auto_detect_port(self) -> str:
        """Tự động tìm kiếm cổng Serial phù hợp với gimbal."""
        patterns = ['/dev/ttyUSB*', '/dev/ttyACM*', '/dev/serial/by-id/*']
        candidates = []
        for pat in patterns:
            candidates.extend(glob.glob(pat))

        if candidates:
            candidates.sort()
            return candidates[0]
        return '/dev/ttyUSB0'

    def connect(self, port: Optional[str] = None, baudrate: Optional[int] = None) -> bool:
        """Mở kết nối Serial tới Camera Gimbal và khởi động luồng nền."""
        if serial is None:
            logger.error("Thư viện pyserial chưa được cài đặt!")
            return False

        if port:
            self.port = port
        if not self.port:
            self.port = self.auto_detect_port()

        if baudrate:
            self.baudrate = baudrate

        self.last_error = ""
        self.disconnect()
        time.sleep(0.08)

        try:
            logger.info(f"[Gimbal] Đang kết nối tới {self.port} ở baudrate {self.baudrate}...")
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.05
            )
            self.is_connected = True
            self._running = True

            # Khởi động 1 luồng nền duy nhất giao tiếp 35Hz
            self._thread = threading.Thread(target=self._communication_worker, daemon=True)
            self._thread.start()

            with self._lock:
                self.telemetry["connected"] = True
                self.telemetry["port"] = self.port
                self.telemetry["baudrate"] = self.baudrate

            # Chờ 0.5s rồi gửi lệnh Center đưa gimbal về giữa
            threading.Thread(target=self._post_connect_setup, daemon=True).start()

            logger.info(f"[Gimbal] Đã kết nối thành công tới {self.port}.")
            return True
        except Exception as e:
            self.last_error = str(e)
            logger.error(f"[Gimbal] Không thể kết nối cổng {self.port}: {e}")
            self.is_connected = False
            with self._lock:
                self.telemetry["connected"] = False
            return False

    def _post_connect_setup(self):
        # Đảm bảo không tự ý điều khiển gimbal khi mới kết nối; chỉ bám khi người dùng khoanh box tracking
        self.stop_tracking()

    def disconnect(self):
        """Dừng luồng nền và đóng kết nối Serial."""
        self._running = False
        self.is_connected = False

        if self._thread and self._thread.is_alive() and self._thread != threading.current_thread():
            self._thread.join(timeout=1.0)
        self._thread = None

        if self.serial_conn and self.serial_conn.is_open:
            try:
                self.serial_conn.close()
            except Exception:
                pass
            self.serial_conn = None

        with self._lock:
            self.telemetry["connected"] = False
            self.telemetry["work_mode_str"] = "Đã ngắt kết nối"

        logger.info("[Gimbal] Đã ngắt kết nối cổng Serial.")

    # =========================================================================
    # ĐÓNG GÓI & GIẢI MÃ KHUNG TRUYỀN GCU
    # =========================================================================
    def _build_packet(self, cmd: int, param: bytes, roll: int, pitch: int, yaw: int, ctrl_valid: bool) -> bytes:
        pkt_len = 72 + len(param)
        ba = bytearray(pkt_len)

        # Header 0xA8 0xE5 & Length & Version 0x02
        ba[0] = 0xA8
        ba[1] = 0xE5
        struct.pack_into('<H', ba, 2, pkt_len)
        ba[4] = 0x02

        # Control values (int16 little endian)
        struct.pack_into('<h', ba, 5, int(roll))
        struct.pack_into('<h', ba, 7, int(pitch))
        struct.pack_into('<h', ba, 9, int(yaw))

        # Status: B2=1 khi control values có hiệu lực
        ba[11] = 0x04 if ctrl_valid else 0x00

        # Sub frame request = 0x01
        ba[30] = 0x01

        # Command & Params
        ba[69] = cmd & 0xFF
        if param:
            ba[70:70 + len(param)] = param

        # CRC16 Checksum
        crc = calculate_crc16(ba[:pkt_len - 2])
        ba[pkt_len - 2] = (crc >> 8) & 0xFF
        ba[pkt_len - 1] = crc & 0xFF

        return bytes(ba)

    def _parse_response(self, raw: bytes):
        if len(raw) < 72 or raw[0] != 0x8A or raw[1] != 0x5E:
            return

        pkt_len = struct.unpack_from('<H', raw, 2)[0]
        if len(raw) < pkt_len or pkt_len < 72:
            return

        # Kiểm tra CRC
        received_crc = (raw[pkt_len - 2] << 8) | raw[pkt_len - 1]
        calc_crc = calculate_crc16(raw[:pkt_len - 2])
        if received_crc != calc_crc:
            return

        work_mode = raw[5]
        mode_names = {
            0x10: "Angle Control",
            0x11: "Head Lock",
            0x12: "Head Follow",
            0x13: "Orthoview",
            0x14: "Euler Angle Control",
            0x16: "Gaze",
            0x17: "Track",
            0x1C: "FPV"
        }

        # Theo Appendix 4 tài liệu GCU Private Protocol:
        # Byte 12~13: angle_x -> CamThe (Pitch) = -AngleX
        # Byte 14~15: angle_y -> CamPhi (Roll)  = +AngleY
        # Byte 16~17: angle_z -> CamPsi (Yaw)   = +AngleZ
        angle_x = struct.unpack_from('<h', raw, 12)[0] / 100.0
        angle_y = struct.unpack_from('<h', raw, 14)[0] / 100.0
        angle_z = struct.unpack_from('<h', raw, 16)[0] / 100.0

        rel_pitch = -angle_x
        rel_roll = angle_y
        rel_yaw = angle_z

        abs_roll = struct.unpack_from('<h', raw, 18)[0] / 100.0
        abs_pitch = struct.unpack_from('<h', raw, 20)[0] / 100.0
        abs_yaw = struct.unpack_from('<H', raw, 22)[0] / 100.0

        ang_vel_x = struct.unpack_from('<h', raw, 24)[0] / 100.0
        ang_vel_y = struct.unpack_from('<h', raw, 26)[0] / 100.0
        ang_vel_z = struct.unpack_from('<h', raw, 28)[0] / 100.0

        zoom_cam1 = 1.0
        target_dist = 0.0
        is_recording = False
        is_tracking = False

        if pkt_len >= 68:
            target_dist = struct.unpack_from('<i', raw, 43)[0] / 10.0
            zoom_cam1 = struct.unpack_from('<H', raw, 59)[0] / 10.0
            camera_status = struct.unpack_from('<H', raw, 64)[0]
            is_recording = bool(camera_status & (1 << 4))
            pod_status = struct.unpack_from('<H', raw, 6)[0]
            is_tracking = bool(pod_status & 1)

        echoed_cmd = raw[69]
        cmd_status = raw[70] if pkt_len > 72 else -1

        with self._lock:
            self.telemetry.update({
                "work_mode": work_mode,
                "work_mode_str": mode_names.get(work_mode, f"0x{work_mode:02X}"),
                "pitch": round(rel_pitch, 2),
                "yaw": round(rel_yaw, 2),
                "roll": round(rel_roll, 2),
                "rel_roll": round(rel_roll, 2),
                "rel_pitch": round(rel_pitch, 2),
                "rel_yaw": round(rel_yaw, 2),
                "abs_roll": round(abs_roll, 2),
                "abs_pitch": round(abs_pitch, 2),
                "abs_yaw": round(abs_yaw, 2),
                "ang_vel_x": round(ang_vel_x, 2),
                "ang_vel_y": round(ang_vel_y, 2),
                "ang_vel_z": round(ang_vel_z, 2),
                "zoom_rate": round(zoom_cam1, 1),
                "is_recording": is_recording,
                "is_tracking": is_tracking,
                "target_distance": round(target_dist, 1),
                "last_cmd": echoed_cmd,
                "last_cmd_status": cmd_status,
                "packets_received": self.telemetry["packets_received"] + 1,
                "last_updated": time.time()
            })

    # =========================================================================
    # LUỒNG GIAO TIẾP DUY NHẤT (Single Thread Worker - 35Hz)
    # =========================================================================
    def _communication_worker(self):
        """Vòng lặp Q&A thời gian thực tốc độ 35Hz không nghẽn."""
        rx_buffer = bytearray()

        while self._running and self.serial_conn and self.serial_conn.is_open:
            loop_start = time.time()

            # Lấy lệnh từ queue hoặc duy trì Angle Control setpoint liên tục
            with self._lock:
                if not self._cmd_queue.empty():
                    cmd_to_send, param_to_send, ctrl_valid, roll_ctrl, pitch_ctrl, yaw_ctrl = self._cmd_queue.get_nowait()
                else:
                    cmd_to_send = CMD_NULL
                    param_to_send = bytes()
                    ctrl_valid = self._control_valid
                    roll_ctrl = self._roll_ctrl_cdeg
                    pitch_ctrl = self._pitch_ctrl_cdeg
                    yaw_ctrl = self._yaw_ctrl_cdeg

            # Gửi gói tin
            packet = self._build_packet(cmd_to_send, param_to_send, roll_ctrl, pitch_ctrl, yaw_ctrl, ctrl_valid)
            try:
                self.serial_conn.write(packet)
                self.serial_conn.flush()
                self.telemetry["packets_sent"] += 1
            except Exception as e:
                logger.error(f"[Gimbal] Lỗi ghi Serial: {e}")
                time.sleep(0.05)
                continue

            # Đọc phản hồi
            try:
                if self.serial_conn.in_waiting > 0:
                    incoming = self.serial_conn.read(self.serial_conn.in_waiting)
                    rx_buffer.extend(incoming)

                while len(rx_buffer) >= 4:
                    idx = rx_buffer.find(b'\x8A\x5E')
                    if idx == -1:
                        rx_buffer = rx_buffer[-1:]
                        break
                    if idx > 0:
                        del rx_buffer[:idx]

                    if len(rx_buffer) < 4:
                        break

                    expected_len = struct.unpack_from('<H', rx_buffer, 2)[0]
                    if expected_len < 72 or expected_len > 128:
                        del rx_buffer[:2]
                        continue

                    if len(rx_buffer) < expected_len:
                        break

                    complete_packet = bytes(rx_buffer[:expected_len])
                    del rx_buffer[:expected_len]
                    self._parse_response(complete_packet)

            except Exception as e:
                logger.error(f"[Gimbal] Lỗi đọc Serial: {e}")

            elapsed = time.time() - loop_start
            sleep_time = self.interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    # =========================================================================
    # HÀNG ĐỢI LỆNH & ĐỆM NULL COMMAND
    # =========================================================================
    def _queue_command(self, cmd: int, param: bytes = bytes(), ctrl_valid: Optional[bool] = None,
                       roll: Optional[int] = None, pitch: Optional[int] = None, yaw: Optional[int] = None,
                       repeats: int = 4, null_padding: int = 2):
        with self._lock:
            c_valid = self._control_valid if ctrl_valid is None else ctrl_valid
            r = self._roll_ctrl_cdeg if roll is None else roll
            p = self._pitch_ctrl_cdeg if pitch is None else pitch
            y = self._yaw_ctrl_cdeg if yaw is None else yaw

            # 1. Đệm gói Null phía trước
            for _ in range(null_padding):
                self._cmd_queue.put((CMD_NULL, bytes(), c_valid, r, p, y))

            # 2. Gói lệnh chính
            for _ in range(repeats):
                self._cmd_queue.put((cmd, param, c_valid, r, p, y))

            # 3. Đệm gói Null phía sau
            for _ in range(null_padding):
                self._cmd_queue.put((CMD_NULL, bytes(), c_valid, r, p, y))

    # =========================================================================
    # VISUAL SERVOING OBJECT TRACKING (BÁM TÂM VẬT THỂ)
    # =========================================================================
    def track_target(self, err_x: float, err_y: float, deadband: float = 0.02):
        """
        Tự động điều khiển Gimbal xoay Pitch & Yaw để đưa mục tiêu về chính giữa màn hình (0, 0).
        Sử dụng bộ điều khiển tỉ lệ mượt mà (Smooth Damped Proportional Control) với bộ lọc thông thấp (EWMA)
        và continuous deadband để triệt tiêu hoàn toàn hiện tượng vọt lố và lắc lư qua lại:
        1. Khi trong vùng deadband (<= 0.02): vận tốc về 0, gimbal đứng im không dao động.
        2. Khi ngoài deadband: vận tốc tăng liên tục từ 0 (không bị giật bước nhảy).
        3. Bộ lọc EWMA triệt tiêu rung giật do nhiễu pixel của bounding box.
        """
        if not self.is_connected:
            return

        now = time.time()
        with self._lock:
            # Khi vừa kích hoạt tracking hoặc chuyển từ trạng thái dừng:
            if not self._is_active_tracking:
                self._is_active_tracking = True
                self._last_track_time = now
                self._filtered_spd_yaw = 0.0
                self._filtered_spd_pitch = 0.0
                if self.telemetry.get("packets_received", 0) > 0 and self.telemetry.get("last_updated", 0) > 0:
                    self._target_pitch = self.telemetry.get("pitch", 0.0)
                    self._target_yaw = self.telemetry.get("yaw", 0.0)
                # Dọn sạch hàng đợi lệnh rời rạc cũ để không gây delay
                while not self._cmd_queue.empty():
                    try:
                        self._cmd_queue.get_nowait()
                    except queue.Empty:
                        break

            dt = max(0.01, min(0.06, now - self._last_track_time))
            self._last_track_time = now

            # --- TRỤC YAW (Ngang) ---
            # Lệch phải (err_x > 0) -> Gimbal quay sang phải (tăng Yaw)
            # Lệch trái (err_x < 0) -> Gimbal quay sang trái (giảm Yaw)
            if abs(err_x) <= deadband:
                spd_yaw_target = 0.0
                if abs(self._filtered_spd_yaw) < 0.2:
                    self._filtered_spd_yaw = 0.0
            else:
                # Active error tính từ mép deadband -> liên tục về 0 khi tới tâm, triệt tiêu vọt lố
                e_act_x = err_x - (deadband if err_x > 0 else -deadband)
                spd_yaw_target = self.kp_yaw * e_act_x * (1.0 + 0.8 * min(1.0, abs(e_act_x)))
                spd_yaw_target = max(-self.max_yaw_speed, min(self.max_yaw_speed, spd_yaw_target))

            # Bộ lọc thông thấp EWMA: làm mượt gia tốc, loại bỏ chấn động pixel
            alpha = 0.40
            if spd_yaw_target == 0.0 and self._filtered_spd_yaw == 0.0:
                pass
            else:
                self._filtered_spd_yaw = alpha * spd_yaw_target + (1.0 - alpha) * self._filtered_spd_yaw

            self._target_yaw = max(-160.0, min(160.0, self._target_yaw + self._filtered_spd_yaw * dt))

            # --- TRỤC PITCH (Dọc) ---
            # err_y > 0 (vật thể ở dưới tâm màn hình) -> Gimbal chúc xuống (giảm Pitch)
            # err_y < 0 (vật thể ở trên tâm màn hình) -> Gimbal ngẩng lên (tăng Pitch)
            if abs(err_y) <= deadband:
                spd_pitch_target = 0.0
                if abs(self._filtered_spd_pitch) < 0.2:
                    self._filtered_spd_pitch = 0.0
            else:
                e_act_y = err_y - (deadband if err_y > 0 else -deadband)
                spd_pitch_target = -(self.kp_pitch * e_act_y * (1.0 + 0.8 * min(1.0, abs(e_act_y))))
                spd_pitch_target = max(-self.max_pitch_speed, min(self.max_pitch_speed, spd_pitch_target))

            if spd_pitch_target == 0.0 and self._filtered_spd_pitch == 0.0:
                pass
            else:
                self._filtered_spd_pitch = alpha * spd_pitch_target + (1.0 - alpha) * self._filtered_spd_pitch

            self._target_pitch = max(-90.0, min(90.0, self._target_pitch + self._filtered_spd_pitch * dt))

            # Cập nhật giá trị góc điều khiển (0.01 deg -> nhân 100)
            self._roll_ctrl_cdeg = 0
            self._pitch_ctrl_cdeg = int(round(self._target_pitch * 100))
            self._yaw_ctrl_cdeg = int(round(self._target_yaw * 100))
            self._control_valid = True

            # Cập nhật telemetry để hiển thị mượt mà trên giao diện Web Dashboard
            if self.telemetry.get("packets_received", 0) == 0:
                self.telemetry["pitch"] = round(self._target_pitch, 1)
                self.telemetry["yaw"] = round(self._target_yaw, 1)
                self.telemetry["rel_pitch"] = round(self._target_pitch, 1)
                self.telemetry["rel_yaw"] = round(self._target_yaw, 1)
                self.telemetry["is_tracking"] = True

    def stop_tracking(self):
        """Dừng bám mục tiêu và giữ gimbal đứng im ở góc hiện tại."""
        with self._lock:
            self._is_active_tracking = False
            self._filtered_spd_yaw = 0.0
            self._filtered_spd_pitch = 0.0
            if self.telemetry.get("packets_received", 0) > 0 and self.telemetry.get("last_updated", 0) > 0:
                cur_p = self.telemetry.get("pitch", 0.0)
                cur_y = self.telemetry.get("yaw", 0.0)
                self._target_pitch = cur_p
                self._target_yaw = cur_y
            self._pitch_ctrl_cdeg = int(round(self._target_pitch * 100))
            self._yaw_ctrl_cdeg = int(round(self._target_yaw * 100))
            self._control_valid = False  # Vô hiệu hóa điều khiển góc khi chưa/không tracking
            if self.telemetry.get("packets_received", 0) == 0:
                self.telemetry["pitch"] = round(self._target_pitch, 1)
                self.telemetry["yaw"] = round(self._target_yaw, 1)
                self.telemetry["is_tracking"] = False

    def stop_motion(self):
        """Dừng quay lập tức."""
        self.stop_tracking()

    # =========================================================================
    # CÁC LỆNH ĐIỀU KHIỂN CƠ BẢN
    # =========================================================================
    def center(self):
        """Đưa Gimbal về vị trí chính giữa (Pitch=0°, Yaw=0°, Roll=0°)."""
        with self._lock:
            while not self._cmd_queue.empty():
                try:
                    self._cmd_queue.get_nowait()
                except queue.Empty:
                    break
            self._roll_ctrl_cdeg = 0
            self._pitch_ctrl_cdeg = 0
            self._yaw_ctrl_cdeg = 0
            self._target_pitch = 0.0
            self._target_yaw = 0.0
            self._control_valid = True
            self._is_active_tracking = False
            if self.telemetry.get("packets_received", 0) == 0:
                self.telemetry["pitch"] = 0.0
                self.telemetry["yaw"] = 0.0
                self.telemetry["is_tracking"] = False

        self._queue_command(CMD_ANGLE_CONTROL, ctrl_valid=True, roll=0, pitch=0, yaw=0, repeats=4, null_padding=1)

    def set_angles(self, pitch_deg: float, yaw_deg: float, roll_deg: float = 0.0):
        """Đặt góc chỉ định theo độ."""
        pitch_clamped = max(min(pitch_deg, 90.0), -90.0)
        roll_clamped = max(min(roll_deg, 60.0), -60.0)
        yaw_clamped = max(min(yaw_deg, 160.0), -160.0)

        r_cdeg = int(roll_clamped * 100)
        p_cdeg = int(pitch_clamped * 100)
        y_cdeg = int(yaw_clamped * 100)

        with self._lock:
            self._roll_ctrl_cdeg = r_cdeg
            self._pitch_ctrl_cdeg = p_cdeg
            self._yaw_ctrl_cdeg = y_cdeg
            self._target_pitch = pitch_clamped
            self._target_yaw = yaw_clamped
            self._control_valid = True
            if self.telemetry.get("packets_received", 0) == 0:
                self.telemetry["pitch"] = round(pitch_clamped, 1)
                self.telemetry["yaw"] = round(yaw_clamped, 1)
                self.telemetry["roll"] = round(roll_clamped, 1)

        self._queue_command(CMD_ANGLE_CONTROL, ctrl_valid=True, roll=r_cdeg, pitch=p_cdeg, yaw=y_cdeg, repeats=4, null_padding=1)

    def set_speeds(self, pitch_speed_dps: float, yaw_speed_dps: float):
        """Đặt vận tốc quay chỉ định theo độ/giây."""
        p_val = int(max(min(pitch_speed_dps, 200.0), -200.0) * 10)
        y_val = int(max(min(yaw_speed_dps, 200.0), -200.0) * 10)

        with self._lock:
            self._roll_ctrl_cdeg = 0
            self._pitch_ctrl_cdeg = p_val
            self._yaw_ctrl_cdeg = y_val
            self._control_valid = True

        self._queue_command(CMD_HEAD_LOCK, ctrl_valid=True, roll=0, pitch=p_val, yaw=y_val, repeats=3, null_padding=1)

    def look_down(self):
        """Chúc camera 90 độ xuống đất."""
        with self._lock:
            self._roll_ctrl_cdeg = 0
            self._pitch_ctrl_cdeg = -9000
            self._yaw_ctrl_cdeg = 0
            self._control_valid = False
        self._queue_command(CMD_ORTHOVIEW, ctrl_valid=False, repeats=4, null_padding=2)

    def take_photo(self):
        """Chụp 1 ảnh."""
        self._queue_command(CMD_SHUTTER, param=bytes([0x01]), repeats=4, null_padding=2)

    def toggle_record(self):
        """Bật/tắt quay video."""
        self._queue_command(CMD_RECORD, param=bytes([0x01]), repeats=4, null_padding=2)

    def set_zoom_rate(self, rate: float):
        """Đặt độ phóng đại zoom (1.0x - 8.0x)."""
        rate = max(1.0, min(8.0, rate))
        val = int(-round(rate * 10))
        param = bytearray(3)
        param[0] = 0x01
        struct.pack_into('<h', param, 1, val)
        self._queue_command(CMD_ZOOM_SPECIFIED, param=bytes(param), repeats=4, null_padding=2)

    def get_telemetry(self) -> Dict[str, Any]:
        """Lấy telemetry cập nhật mới nhất của gimbal."""
        with self._lock:
            return dict(self.telemetry)


# Khởi tạo singleton instance cho toàn bộ ứng dụng
gimbal_ctrl = GimbalController()

