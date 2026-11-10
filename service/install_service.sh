#!/usr/bin/env bash
set -e

SERVICE_FILE="/home/ubuntu/tracking_uav_web/service/aerotrack.service"
TARGET="/etc/systemd/system/aerotrack.service"

echo "=== Đang cài đặt AERO-TRACK tự khởi động cùng hệ thống ==="

if [ ! -f "$SERVICE_FILE" ]; then
    echo "Lỗi: Không tìm thấy $SERVICE_FILE"
    exit 1
fi

sudo cp "$SERVICE_FILE" "$TARGET"
sudo systemctl daemon-reload
sudo systemctl enable aerotrack.service

echo ""
echo " ĐÃ CÀI ĐẶT THÀNH CÔNG!"
echo "- Khởi chạy ngay:         sudo systemctl start aerotrack.service"
echo "- Kiểm tra trạng thái:    sudo systemctl status aerotrack.service"
echo "- Xem log trực tiếp:      sudo journalctl -u aerotrack.service -f"
echo "- Dừng dịch vụ:           sudo systemctl stop aerotrack.service"
echo "- Hủy tự khởi động:       sudo systemctl disable aerotrack.service"

