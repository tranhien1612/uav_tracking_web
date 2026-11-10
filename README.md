# AERO-TRACK GCS | Hệ Thống Bám Điểm & Căn Tâm Mục Tiêu Cho UAV (ArduPilot MAVLink)
### Giao Diện Đơn Giản, Hiện Đại, Trực Quan | Luồng Camera RTSP

Hệ thống web điều khiển UAV theo thời gian thực (Visual Servoing Ground Control Station) sử dụng Computer Vision (OpenCV) xử lý luồng **Camera RTSP độ trễ thấp** và giao thức **MAVLink (ArduPilot Firmware)**, giao diện được tối giản hóa tối đa, hiện đại, dễ thao tác với font **Inter**.

---

## 🖥️ Bố Cục Giao Diện Mới

- **Xóa bỏ cột bên trái:** Toàn bộ không gian bên trái và trung tâm (~75% diện tích) dành trọn cho màn hình hiển thị video RTSP góc rộng sắc nét.
- **Cột thông tin & điều khiển bên phải:**
  - **Thông tin máy bay (UAV Telemetry):** Trạng thái kết nối, Chế độ bay (Mode), ARM/DISARM, Phần trăm pin kèm thanh trực quan & điện áp, Độ cao (Altitude).
  - **Chế độ hoạt động & Dừng khẩn cấp:**
    - 🛑 **Nút Dừng Khẩn Cấp / Đứng Im (ESC/SPACE):** Gộp nút dừng khẩn cấp và chế độ nghỉ vào làm 1. Nhấn nút này lập tức hủy bám điểm và lệnh UAV đứng im tại chỗ.
    - 🎯 **Theo dõi thủ công (Tracking):** Cho phép kéo chuột khoanh vùng vật thể trên video.
    - 🔴 **Nhận diện vật đỏ tròn (Object Detect):** Tự động phát hiện và lần lượt căn tâm từng vật.
  - **Điều khiển thủ công (Manual Flight Control):**
    - 🛫 / 🛬 **Nút bay động (Takeoff / Land)**: Tự động chuyển đổi giữa **Cất Cánh** (khi ở mặt đất, chưa bay) và **Hạ Cánh** (khi UAV đang bay trên không). Đảm bảo tự động chuyển mode GUIDED và ARM an toàn.
    - 🕹️ **Bàn phím điều hướng 6 hướng** (Lên, Xuống, Sang Trái, Sang Phải, Quay Trái, Quay Phải). Tự động đảm bảo mode GUIDED trước khi gửi vector vận tốc. Khóa an toàn khi mất kết nối UAV.
  - **Nhật ký hoạt động (Flight Log):** Khung log cuộn tự động, **lắng nghe bản tin STATUSTEXT trực tiếp từ ArduPilot** và tô màu theo mức độ (INFO, NOTICE, WARNING, ERROR, CRITICAL).
- **Nút Cài Đặt (⚙️) ở góc trên bên phải:**
  - Nhấp vào icon bánh răng để mở cửa sổ cấu hình Popup.
  - Cấu hình URL luồng RTSP (TCP / UDP).
  - Cấu hình cổng kết nối MAVLink ArduPilot (SITL `udp:127.0.0.1:14550` hoặc Serial `COMx` / `/dev/ttyUSB0`).
  - Cấu hình **Độ cao cất cánh mặc định (Takeoff Altitude)** theo mét.
  - Cấu hình vùng chết căn tâm (Deadband box).

---

## 🛠️ Hướng Dẫn Cài Đặt & Khởi Chạy

### A. Trên WINDOWS
1. Chạy file `setup_env.bat` để tự động tạo môi trường `.venv` và cài đặt thư viện.
2. Chạy `run.bat` (hoặc `python run.py` trong `.venv`) để mở server.

### B. Trên LINUX (Ubuntu / Debian / Raspberry Pi)
```bash
sudo apt update && sudo apt install -y python3 python3-dev python3-venv python3-pip libgl1-mesa-glx libglib2.0-0 libxml2-dev libxslt1-dev zlib1g-dev
sudo usermod -a -G dialout $USER
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
pip install -r requirements.txt
```

---

## 🌐 Sử Dụng

1. Mở trình duyệt: **`http://localhost:8000`**
2. Nhấn vào biểu tượng bánh răng **⚙️ (Cài đặt)** ở góc trên bên phải để nhập URL RTSP và kết nối ArduPilot MAVLink.
3. Khi UAV đã kết nối:
   - Nhấn **1. Theo dõi thủ công (T)** và kéo chuột trên video để khoanh vùng mục tiêu.
   - Nhấn **2. Nhận diện vật đỏ tròn (D)** để tự động bám tâm các quả cầu đỏ.
   - Nhấn **DỪNG KHẨN CẤP / ĐỨNG IM (Space / Esc)** bất kỳ lúc nào để UAV đứng im.

---

## 🧪 Chạy Bộ Mô Phỏng & Kiểm Thử Độc Lập (`tests/`)

- Giả lập Drone ArduPilot qua MAVLink UDP 14550:
  ```bash
  python tests/mock_sitl_drone.py
  ```
- Giả lập camera mạng phát luồng video quả cầu đỏ tại `http://127.0.0.1:8554/live`:
  ```bash
  python tests/mock_video_streamer.py
  ```
- Chạy unit tests:
  ```bash
  python tests/test_vision.py
  python tests/test_mavlink.py
  python tests/test_api.py
  ```

## Config network
```
sudo nano /etc/netplan/01-netcfg.yaml

network:
  version: 2
  renderer: networkd
  ethernets:
    eth0:
      optional: true
      # dhcp4: true
      dhcp4: no
      addresses:
        - 192.168.144.10/24
  wifis:
    wlan0:
      dhcp4: true
      access-points:
        "G-Group T11":
          password: "GGroup@hanoi"

sudo netplan apply
```

---

## Swap RAM
```
# Kiểm tra swap hiện tại
sudo swapon --show
free -h

# Tạo file swap
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
echo 'vm.swappiness=10' | sudo tee -a /etc/sysctl.conf
sudo sysctl -p
```


## Vins-mono
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update

sudo apt install ros-noetic-desktop
sudo apt-get install ros-noetic-cv-bridge ros-noetic-tf ros-noetic-message-filters ros-noetic-image-transport

source /opt/ros/noetic/setup.bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo rosdep init
rosdep update


# Install Ceres 1.14.0
git clone https://github.com/zhangxin0518/ceres-1.14.0

mkdir ceres-bin
cd ceres-bin
cmake ../ceres-1.14.0
make -j4
sudo make install

cd ~/catkin_ws/src
git clone https://github.com/HKUST-Aerial-Robotics/VINS-Mono.git
cd ../
catkin_make
source ~/catkin_ws/devel/setup.bash


```


## 🚀 Tự Động Khởi Chạy Cùng Hệ Thống (Autostart on Boot - Raspberry Pi)

Để web tự động chạy ngầm mỗi khi bật/khởi động lại Raspberry Pi:

### 1. Cài đặt dịch vụ systemd:
```bash
./install_autostart.sh
```

Hoặc thực hiện thủ công:
```bash
sudo cp aerotrack.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable aerotrack.service
```

### 2. Các lệnh quản lý dịch vụ:
- **Khởi động dịch vụ:** `sudo systemctl start aerotrack.service`
- **Kiểm tra trạng thái:** `sudo systemctl status aerotrack.service`
- **Khởi động lại dịch vụ:** `sudo systemctl restart aerotrack.service`
- **Dừng dịch vụ:** `sudo systemctl stop aerotrack.service`
- **Xem log realtime:** `sudo journalctl -u aerotrack.service -f`
- **Tắt tự khởi động:** `sudo systemctl disable aerotrack.service`


---

## 🚀 Tự Động Khởi Chạy Cùng Hệ Thống (Autostart on Boot - Raspberry Pi)

Để web tự động chạy ngầm mỗi khi bật/khởi động lại Raspberry Pi:

### 1. Cài đặt dịch vụ systemd:
```bash
./install_autostart.sh
```

Hoặc thực hiện thủ công:
```bash
sudo cp aerotrack.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable aerotrack.service
```

### 2. Các lệnh quản lý dịch vụ:
- **Khởi động dịch vụ:** `sudo systemctl start aerotrack.service`
- **Kiểm tra trạng thái:** `sudo systemctl status aerotrack.service`
- **Khởi động lại dịch vụ:** `sudo systemctl restart aerotrack.service`
- **Dừng dịch vụ:** `sudo systemctl stop aerotrack.service`
- **Xem log realtime:** `sudo journalctl -u aerotrack.service -f`
- **Tắt tự khởi động:** `sudo systemctl disable aerotrack.service`

