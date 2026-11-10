# THƯ MỤC TESTS & MÔ PHỎNG (STANDALONE TEST SUITE)

Thư mục này chứa toàn bộ các script kiểm thử (Unit tests, Integration tests) và các công cụ giả lập độc lập (**Mock SITL Drone**, **Mock Video Streamer**). 

Mã nguồn trong thư mục `app/` hoàn toàn là mã sản phẩm (Production Code), không chứa mã giả lập hay mô phỏng.

---

## 📁 Danh Sách Công Cụ Kiểm Thử & Mô Phỏng

| File | Chức Năng | Cách Chạy |
| :--- | :--- | :--- |
| `mock_sitl_drone.py` | Giả lập Drone ArduPilot chạy SITL qua UDP `127.0.0.1:14550` (phát Heartbeat, Telemetry và nhận lệnh vận tốc) | `python tests/mock_sitl_drone.py` |
| `mock_video_streamer.py` | Giả lập camera mạng phát luồng video với các quả cầu đỏ chuyển động tại `http://127.0.0.1:8554/live` | `python tests/mock_video_streamer.py` |
| `test_vision.py` | Kiểm thử thuật toán bám điểm OpenCV Tracker và phát hiện vật đỏ tròn | `python tests/test_vision.py` |
| `test_mavlink.py` | Kiểm thử bộ điều khiển ArduPilot MAVLink, định dạng vận tốc 6-DOF | `python tests/test_mavlink.py` |
| `test_api.py` | Kiểm thử tích hợp các endpoint REST API của FastAPI | `python tests/test_api.py` |

---

## 🚀 Hướng Dẫn Sử Dụng Bộ Giả Lập Độc Lập

Khi bạn chưa có Drone thật hoặc chưa cắm Camera RTSP, bạn có thể mở 2 terminal phụ để chạy các bộ mô phỏng:

### Terminal 1: Chạy Giả Lập Drone ArduPilot
```bash
python tests/mock_sitl_drone.py
```
*Script sẽ mở cổng UDP 14550 và phản hồi lại tất cả các lệnh Lên, Xuống, Sang Trái, Sang Phải, Quay Trái, Quay Phải gửi từ GCS.*

### Terminal 2: Chạy Giả Lập Luồng Video Camera
```bash
python tests/mock_video_streamer.py
```
*Script sẽ phát luồng video tại `http://127.0.0.1:8554/live` chứa các quả cầu đỏ di chuyển.*

### Terminal 3: Chạy GCS Web Chính
```bash
python run.py
```
Truy cập `http://localhost:8000`, nhập `http://127.0.0.1:8554/live` vào ô URL camera và nhấn **KẾT NỐI RTSP**. Bạn có thể test toàn bộ tính năng bám điểm và căn tâm tuần tự một cách độc lập!

