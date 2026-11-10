"""
MOCK ARDUPILOT SITL DRONE SIMULATOR
----------------------------------
Run this standalone simulator in a separate terminal:
    python tests/mock_sitl_drone.py

This script emulates an ArduPilot UAV sending real MAVLink telemetry
to GCS at UDP 127.0.0.1:14550 and receiving commands.
"""

import time
import math
import socket
from pymavlink import mavutil

class MockSITLDrone:
    def __init__(self, target_host="127.0.0.1", target_port=14550, listen_port=14555):
        self.target_addr = (target_host, target_port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setblocking(False)
        try:
            self.sock.bind(("0.0.0.0", listen_port))
        except Exception:
            pass

        self.mav = mavutil.mavlink.MAVLink(self.sock, srcSystem=1, srcComponent=1)
        self.altitude = 12.0
        self.heading = 90.0
        self.battery = 98.0
        self.armed = True
        self.mode = "GUIDED"
        self.is_running = True

    def send_packet(self, msg):
        try:
            raw = msg.pack(self.mav)
            self.sock.sendto(raw, self.target_addr)
        except Exception:
            pass

    def run(self):
        print("=" * 60)
        print(f" [*] STARTING MOCK ARDUPILOT SITL DRONE -> Sending to {self.target_addr}")
        print(" [*] Listening for MAVLink commands from AERO-TRACK GCS...")
        print("=" * 60)

        t0 = time.time()
        last_hb = 0.0

        while self.is_running:
            now = time.time()
            t = now - t0

            if now - last_hb >= 0.8:
                last_hb = now
                # 1. HEARTBEAT
                hb = self.mav.heartbeat_encode(
                    mavutil.mavlink.MAV_TYPE_QUADROTOR,
                    mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | (mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED if self.armed else 0),
                    4, # GUIDED
                    mavutil.mavlink.MAV_STATE_ACTIVE
                )
                self.send_packet(hb)

                # 2. ATTITUDE
                roll_rad = math.radians(1.5 * math.sin(t))
                pitch_rad = math.radians(1.0 * math.cos(t))
                yaw_rad = math.radians(self.heading)
                att = self.mav.attitude_encode(int(t * 1000), roll_rad, pitch_rad, yaw_rad, 0.0, 0.0, 0.0)
                self.send_packet(att)

                # 3. VFR_HUD
                hud = self.mav.vfr_hud_encode(0.0, 1.2, int(self.heading), int(self.battery), self.altitude, 0.0)
                self.send_packet(hud)

                # 4. SYS_STATUS
                sys_st = self.mav.sys_status_encode(
                    0, 0, 0, 500, 16400, 1500, int(self.battery), 0, 0, 0, 0, 0, 0
                )
                self.send_packet(sys_st)

            # Receive commands
            try:
                data, addr = self.sock.recvfrom(2048)
                msgs = self.mav.parse_buffer(data)
                if msgs:
                    for msg in msgs:
                        mtype = msg.get_type()
                        if mtype == "SET_POSITION_TARGET_LOCAL_NED":
                            print(f"[CMD] VELOCITY -> vx={msg.vx:.2f}, vy={msg.vy:.2f}, vz={msg.vz:.2f}, yaw={msg.yaw_rate:.2f}")
                            self.altitude = max(0.5, self.altitude - msg.vz * 0.1)
                            self.heading = (self.heading + math.degrees(msg.yaw_rate) * 0.1) % 360
                        elif mtype == "COMMAND_LONG":
                            if msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                                self.armed = (msg.param1 == 1.0)
                                print(f"[CMD] ARM/DISARM -> {'ARMED' if self.armed else 'DISARMED'}")
                                st_msg = self.mav.statustext_encode(
                                    mavutil.mavlink.MAV_SEVERITY_INFO,
                                    ("Motors ARMED" if self.armed else "Motors DISARMED").encode("utf-8")
                                )
                                self.send_packet(st_msg)
                            elif msg.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
                                target_alt = msg.param7
                                self.altitude = max(1.0, float(target_alt))
                                print(f"[CMD] TAKEOFF -> Target Alt: {target_alt}m")
                                st_msg = self.mav.statustext_encode(
                                    mavutil.mavlink.MAV_SEVERITY_NOTICE,
                                    f"Takeoff to {target_alt:.1f}m initiated".encode("utf-8")
                                )
                                self.send_packet(st_msg)
                            elif msg.command == mavutil.mavlink.MAV_CMD_NAV_LAND:
                                self.altitude = 0.0
                                self.armed = False
                                print("[CMD] LAND -> Touchdown complete")
                                st_msg = self.mav.statustext_encode(
                                    mavutil.mavlink.MAV_SEVERITY_WARNING,
                                    b"Landing initiated. Touchdown"
                                )
                                self.send_packet(st_msg)
                        elif mtype == "SET_MODE":
                            print(f"[CMD] SET_MODE -> Mode: {msg.custom_mode}")
                            st_msg = self.mav.statustext_encode(
                                mavutil.mavlink.MAV_SEVERITY_INFO,
                                f"Mode changed to {msg.custom_mode}".encode("utf-8")
                            )
                            self.send_packet(st_msg)
            except BlockingIOError:
                pass
            except Exception:
                pass

            time.sleep(0.05)

def run_mock_sitl(host="127.0.0.1", port=14550):
    drone = MockSITLDrone(target_host=host, target_port=port)
    drone.run()

if __name__ == "__main__":
    try:
        run_mock_sitl()
    except KeyboardInterrupt:
        print("\nMock SITL stopped.")
