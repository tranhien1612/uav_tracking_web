"""
MOCK VIDEO STREAMER (STANDALONE TEST STREAM)
--------------------------------------------
Run this standalone simulator in a separate terminal:
    python tests/mock_video_streamer.py

This script serves a high-fps network video stream with moving red circles
at http://127.0.0.1:8554/live so you can test visual servoing and object tracking
without needing an actual IP camera or RTSP server.
"""

import cv2
import time
import math
import numpy as np
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler

targets = [
    {"x": 160.0, "y": 140.0, "r": 26, "vx": 1.2, "vy": 0.8},
    {"x": 480.0, "y": 320.0, "r": 32, "vx": -0.9, "vy": 0.6},
    {"x": 320.0, "y": 120.0, "r": 22, "vx": 0.7, "vy": -1.0}
]

def generate_test_frame(w=640, h=480):
    frame = np.full((h, w, 3), (20, 24, 32), dtype=np.uint8)
    
    # Ground grid
    for x in range(0, w, 40):
        cv2.line(frame, (x, 0), (x, h), (35, 45, 60), 1)
    for y in range(0, h, 40):
        cv2.line(frame, (0, y), (w, y), (35, 45, 60), 1)

    # Helipad
    cv2.circle(frame, (int(w/2), int(h/2)), 70, (40, 55, 75), 2)
    cv2.putText(frame, "H", (int(w/2) - 15, int(h/2) + 15), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (45, 65, 90), 2)

    # Animate targets
    for tgt in targets:
        tgt["x"] += tgt["vx"]
        tgt["y"] += tgt["vy"]
        if tgt["x"] < tgt["r"] + 20 or tgt["x"] > w - tgt["r"] - 20:
            tgt["vx"] *= -1
        if tgt["y"] < tgt["r"] + 20 or tgt["y"] > h - tgt["r"] - 20:
            tgt["vy"] *= -1

        tx, ty, r = int(tgt["x"]), int(tgt["y"]), tgt["r"]
        cv2.circle(frame, (tx + 4, ty + 6), r, (10, 12, 16), -1)
        cv2.circle(frame, (tx, ty), r, (30, 30, 225), -1)
        cv2.circle(frame, (tx - int(r*0.3), ty - int(r*0.3)), max(2, int(r*0.3)), (100, 100, 255), -1)

    return frame

class StreamHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        # Suppress noisy per-request / RTSP probe HTTP logs
        pass

    def do_GET(self):
        if self.path == '/live':
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=--jpgboundary')
            self.end_headers()
            try:
                while True:
                    frame = generate_test_frame()
                    ret, jpeg = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                    if not ret:
                        continue
                    self.wfile.write(b'--jpgboundary\r\n')
                    self.send_header('Content-type', 'image/jpeg')
                    self.send_header('Content-length', str(len(jpeg)))
                    self.end_headers()
                    self.wfile.write(jpeg.tobytes())
                    self.wfile.write(b'\r\n')
                    time.sleep(0.033)
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
        else:
            self.send_response(404)
            self.end_headers()

class DaemonThreadingHTTPServer(ThreadingHTTPServer):
    daemon_threads = True

def run_streamer(port=8554):
    print("============================================================")
    print(f" [*] STARTING MOCK VIDEO STREAMER ON http://127.0.0.1:{port}/live")
    print(f" [*] You can enter this URL in AERO-TRACK GCS to test video tracking.")
    print("============================================================")
    server = DaemonThreadingHTTPServer(('0.0.0.0', port), StreamHandler)
    server.serve_forever()

if __name__ == '__main__':
    try:
        run_streamer()
    except KeyboardInterrupt:
        print("\nMock streamer stopped.")

