# Trạng thái cài đặt

**Cập nhật sửa lỗi 11/09/2026:** đã kiểm chứng lại 3 lượt khởi động mới độc lập,
đi–về qua đủ 4 khe zigzag và hạ cánh/disarm. Đã tách thông số phiên, chờ sensors
trước khởi tạo EKF và thêm giám sát cất cánh/heading. Chi tiết, log và giới hạn:
`output/startup-fix-verification.md`. Nội dung ngày 09/09 bên dưới là lịch sử cài đặt,
không phải kết luận kiểm thử hiện tại.

Tốc độ hành trình mặc định hiện là **2,5 m/s**; đã chạy trọn bài đi–về riêng tại
`logs/speed-2_5ms-validation02.csv/json`. Đã thêm phanh theo khoảng cách trước
mốc quay đầu và Home; bộ điều khiển vẫn tự giảm tốc gần vật cản.

Đã tăng tốc vòng PID lách ngang (`Kp=0,8`, `Ki=0,03`, `Kd=0,4`, ngang tối đa
1,8 m/s, thay đổi vector tối đa 1,0 m/s²). Lượt `logs/pid-fast-validation01`
hoàn thành trong 91,769 giây, về cách Home thật 1,090 m.

Cập nhật: 09/09/2026. Yêu cầu người dùng: dựng mô phỏng PX4/Gazebo tích hợp **YOLO26** trên máy này.

## Hoàn tất

- Đã cài Ubuntu 22.04 qua WSL 2 với tài khoản Linux `drone`.
- Đã cài PX4 `v1.16.0`, Gazebo Harmonic, ROS 2 Humble, MAVSDK và Ultralytics `8.4.144`.
- Đã tải `models/yolo26n.pt`; kiểm tra nạp model thành công.
- Đã xác nhận CUDA: PyTorch `2.14.0+cu130` nhận NVIDIA RTX 4060 Laptop GPU.
- Đã tạo world `baylands_yolo`, giữ nguyên `baylands.sdf` của PX4, và tải Baylands, Coast Water, actor, Ambulance từ Gazebo Fuel.
- Đã chạy PX4 SITL `gz_x500_mono_cam`; camera xuất ảnh RGB 1280 x 960 tại `/world/baylands_yolo/model/x500_mono_cam_0/link/camera_link/sensor/imager/image`.
- Node `yolo_gz_node.py` đã chạy bằng GPU và xuất `/yolo/image_annotated` kiểu `sensor_msgs/msg/Image`.
- Đã lưu `output/annotated_frame.png`, có kết quả nhận diện người 0,79 và xe 0,25.
- Đã chạy bài bay 15 giây: kết nối PX4, cất cánh, bay vòng tròn, hạ cánh và disarm thành công.
- Đã chạy bài tránh vật cản 40 giây của **bản cũ (còn xoay)**: `CRUISE → SLOW → BRAKE → TURN_LEFT → CRUISE`, hạ cánh thành công; khoảng cách LiDAR gần nhất 2,41 m. Log là `logs/avoid-flight-check.csv`.
- Bản hiện tại giữ nguyên hướng mũi, chỉ lệch ngang bằng PID (`SLIDE_LEFT/RIGHT` → `CLEARING` → `CRUISE`). Đã qua toàn bộ unit test và bài kiểm tra cảm biến `--dry-run` (`logs/avoid-smooth-sensor-check.csv`), nhưng **chưa bay lại trọn bài trong Gazebo**.
- Đã chạy 56 unit tests, kiểm tra SDF, cú pháp shell/Python và dọn tiến trình mô phỏng thành công.

## Cách dùng

Nâng cấp tránh vật cản: đã thêm model `x500_mono_lidar` và world `obstacle_yolo`.
Chạy `.\Drone.ps1 sim-avoid` rồi ở cửa sổ khác `.\Drone.ps1 avoid --duration 60`.
YOLO của sân mới dùng `.\Drone.ps1 detect-avoid --fps 10 --device 0`.
Chế độ này dùng LiDAR 360 độ ở 15 Hz; vừa bay tiến vừa nhích ngang bằng PID để vượt vật cản, không xoay.
Chi tiết và giới hạn xem mục tránh vật cản trong README.

Trong PowerShell, mở lần lượt các cửa sổ:

```powershell
.\Drone.ps1 sim
.\Drone.ps1 detect --fps 10 --device 0
.\Drone.ps1 view
.\Drone.ps1 fly --duration 60
```

Sau khi kết thúc, dừng `detect` bằng Ctrl+C trước, sau đó dừng `sim`. `fly` tự hạ cánh khi hết thời lượng; Ctrl+C trong tiến trình bay cũng yêu cầu hạ cánh.

## Lưu ý đã biết

- Gazebo báo thiếu texture `ambulance.png` ở một số lần tải, nhưng mô hình xe, camera và kết quả YOLO đã hiển thị được.
- Topic camera chính xác của PX4 hiện tại là `sensor/imager/image`, khác ví dụ cũ trong PDF.
- `circle_fly.py` tắt kiểm tra chất lượng GPS thực (`EKF2_GPS_CHECK=0`) chỉ cho PX4 SITL cục bộ, vì cảnh Baylands đôi khi báo GPS vertical-speed drift.
