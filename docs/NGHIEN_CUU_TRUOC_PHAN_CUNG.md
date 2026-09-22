# Cổng nghiên cứu trước phần cứng

> Tài liệu này **không** là hướng dẫn bay thật. Project hiện chỉ chạy PX4 SITL/Gazebo. Không kết nối, arm, takeoff, Offboard, land hoặc gửi bất kỳ lệnh MAVLink nào đến drone thật.

## Mục đích

Thiết lập bằng chứng cần có trước khi một nhóm chuyên môn độc lập xem xét một kế hoạch nghiên cứu HIL hoặc phần cứng. Hoàn thành các mục dưới đây không đồng nghĩa hệ thống an toàn cho drone thật.

## Cổng 1 — Hợp đồng cảm biến trong SITL

Mỗi world `obstacle_yolo`, `slalom_yolo`, `challenge_yolo` phải có log dry-run mới và riêng biệt:

- LiDAR 360° nhận scan đủ, timestamp tăng, `range_max` hữu hạn.
- Adapter Gazebo chỉ chuẩn hóa `+Inf` no-hit thành `range_max`; `NaN`, `-Inf`, timestamp sai/lùi, geometry sai và scan cũ vẫn fail-closed.
- Không tạo MAVSDK, arm, takeoff hay Offboard trong dry-run.
- Log kết thúc `OBSERVATION_COMPLETE`; nếu fail phải giữ CSV/JSON và lý do.

Lệnh mô phỏng duy nhất:

```powershell
.\Drone.ps1 sim-slalom
.\Drone.ps1 avoid --dry-run --duration 30 --world slalom_yolo --log logs/slalom-sensor-gate.csv
```

## Cổng 2 — Fault injection chỉ trong mô phỏng

Mỗi tình huống phải được tái hiện bằng unit test hoặc world/model mô phỏng. Mục tiêu là kiểm tra hệ thống dừng/hạ cánh an toàn, không phải ép hoàn thành nhiệm vụ.

| Sự cố mô phỏng                                             | Kết quả bắt buộc                                         |
| ---------------------------------------------------------- | -------------------------------------------------------- |
| LiDAR `NaN`, `-Inf`, scan thiếu, timestamp lùi hoặc quá cũ | Lệnh ngang bằng 0; fail/land theo timeout                |
| Mất telemetry position/attitude                            | Dừng, hủy nhiệm vụ và land; xác nhận disarm              |
| Yaw EKF lệch                                               | Giữ trục world; đứng yên căn hướng; timeout thì land     |
| Nghiêng, sai cao độ, vượt bán kính                         | Hủy nhiệm vụ và land                                     |
| Vật cản mới khi về Home                                    | LiDAR/PID phát hiện và né, không dùng YOLO               |
| Không có hành lang hoặc đổi bên liên tục                   | Hysteresis/hold hoặc `BLOCKED`, không oscillation vô hạn |
| Mất Offboard                                               | Hủy nhiệm vụ, không tiếp tục gửi route                   |

## Cổng 3 — Evidence replay và review

Mỗi run mô phỏng cần lưu:

- CSV và JSON sidecar không bị sửa/xóa;
- world, tham số, run ID, timestamp, tuổi LiDAR, phase/state, lệnh/telemetry;
- lý do fail nếu có;
- xác nhận `landing_confirmed` và `disarmed_confirmed` trước `COMPLETE`;
- replay chỉ đọc log bằng [`scripts/plot_slalom_replay.py`](../scripts/plot_slalom_replay.py).

Không dùng một lượt thành công để che lỗi hoặc suy diễn về mọi seed, mọi sensor noise, hay phần cứng thật.

## Cổng 4 — HIL chỉ sau review độc lập

HIL không thuộc phạm vi code hiện tại. Trước khi bắt đầu bất kỳ hoạt động HIL nào, cần review độc lập bằng văn bản tối thiểu các hạng mục:

1. Version PX4, firmware, model dynamics và bridge interface cố định/reproducible.
2. Failsafe, geofence, battery, RC loss, estimator and sensor fault behavior được kiểm tra trong môi trường cách ly.
3. Không có parameter override làm giảm preflight hoặc sensor quality checks.
4. Quy trình emergency stop, quyền dừng, người quan sát và giới hạn không gian do tổ chức có thẩm quyền phê duyệt.
5. Pháp lý, bảo hiểm, tần số vô tuyến, khu vực vận hành và đánh giá rủi ro được xác nhận.

## Điều bị cấm trong project này

- Thêm endpoint, serial URI, UDP bridge hoặc command kết nối drone thật.
- Hạ preflight/PX4 check để arm.
- Dùng YOLO để tạo hoặc sửa velocity, yaw, arm, takeoff, land hoặc Offboard.
- Khai báo một bài test SITL/HIL là bằng chứng an toàn bay thật.
