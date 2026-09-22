# Drone PX4 SITL/Gazebo: LiDAR 360° + PID tránh vật cản

> **Phạm vi an toàn:** dự án này chỉ dành cho **PX4 SITL + Gazebo Harmonic trong WSL Ubuntu**. Không kết nối, không arm, không thử nghiệm trên drone thật. Kết quả mô phỏng, kể cả khi tất cả kiểm thử đều đạt, **không đủ để kết luận an toàn bay phần cứng thật**.

## Mục tiêu và kiến trúc

Dự án nghiên cứu bộ điều khiển tránh vật cản phản ứng theo cảm biến cho X500 trong mô phỏng:

- LiDAR 360° là nguồn duy nhất tạo quyết định tránh vật cản.
- Planner dùng khoảng trống LiDAR hiện tại, PID ngang, giới hạn vận tốc/gia tốc và quãng phanh.
- Không có đường bay theo thời gian, chuỗi trái–phải có sẵn, waypoint cổng hoặc tọa độ hình học world trong planner.
- Trục world khai báo là chuẩn. Với `slalom_yolo` và `challenge_yolo`, trục hành trình là PX4 NED yaw `90°`; controller **không** re-zero theo yaw EKF.
- Drone phải đứng yên, đúng độ cao và căn hướng liên tục trước khi có lệnh tiến. Không căn được đúng thời hạn thì hủy bài và hạ cánh.
- Khi quay về Home, controller đảo **trục điều hướng** 180° sau khi phanh/ổn định, nhưng tiếp tục dùng LiDAR 360° + PID để quét và né. Yaw setpoint mũi drone không tự đổi theo planner.
- `COMPLETE` chỉ được ghi sau khi MAVSDK quan sát drone đã disarm.

```text
LiDAR 360° hợp lệ + telemetry PX4 hợp lệ
                │
                ▼
      Planner khoảng trống + PID ngang
      • anti-windup / integral limit
      • D theo vận tốc đo được
      • braking envelope
      • hysteresis đổi bên
      • giới hạn vector tốc độ/gia tốc
                │
                ▼
        MAVSDK Offboard (PX4 SITL)

YOLO camera ──► ảnh annotate + JSON + CSV
                  (không có bất kỳ đường lệnh bay nào)
```

## YOLO chỉ quan sát

[`yolo_gz_node.py`](yolo_gz_node.py) có thể publish `/yolo/image_annotated`, `/yolo/detections` và ghi CSV. Nhánh YOLO không được import bởi [`avoid_fly.py`](avoid_fly.py), không được đưa vào [`Planner`](obstacle_avoidance.py), và không thể tạo/thay đổi lệnh vận tốc, yaw, arm, takeoff, land hoặc Offboard.

Đã loại bỏ khỏi nhánh bay:

- subcommand `track`;
- option `--track`;
- option `--dynamic-avoid`;
- subscriber `/yolo/detections` trong controller;
- API tracking/dynamic avoidance;
- APF không được runtime sử dụng.

Tùy chọn `--track` của [`yolo_gz_node.py`](yolo_gz_node.py) chỉ giữ định danh đối tượng khi **quan sát ảnh/video**; nó không có kết nối MAVSDK hoặc quyền điều khiển bay.

## Hệ tọa độ và điều kiện fail-safe

- PX4 telemetry/lệnh vận tốc dùng **NED**: North, East, Down; `down` dương.
- Gazebo truth guard đổi ENU sang NED chỉ để kiểm tra read-only; nó có thể hủy bài nhưng không cấp hình học hoặc lệnh né cho planner.
- LiDAR phải là planar 360°, đúng hình học, timestamp tăng đơn điệu và fresh. Adapter Gazebo chỉ đổi `+Inf` no-hit đã xác minh từ GPU LiDAR thành `range_max` hữu hạn trước planner; `NaN`, `-Inf`, range âm, scan thiếu, timestamp sai/lùi hoặc scan quá cũ vẫn dẫn tới lệnh dừng an toàn; mất kéo dài sẽ hủy/hạ cánh.
- Mất telemetry, rời bán kính, sai dải cao độ, nghiêng quá ngưỡng, không còn Offboard hoặc không xác nhận được landing/disarm đều được ghi `FAILED` với lý do.
- Không hạ/tắt PX4 preflight để ép arm. Controller kiểm tra `SIM_GZ_EN=1`, health và `is_armable` trước arm.
- Controller bay tròn legacy đã bị vô hiệu hóa vì từng hạ `EKF2_GPS_CHECK`; `avoid` là pilot duy nhất được hỗ trợ.
- Khóa điều khiển nằm trong [`avoid_fly.py`](avoid_fly.py) cho live run trên WSL/POSIX, nên `avoid`, Python trực tiếp và `unified avoid` không thể cùng sở hữu Offboard.

## Lệnh PowerShell

Mở PowerShell tại đúng thư mục:

```powershell
Set-Location 'C:\Users\Administrator\Documents\code\AI PROJECT\AI DRONE'
```

### 1. Mở world

```powershell
# World slalom bốn cổng, PX4 SITL + Gazebo
.\Drone.ps1 sim-slalom

# World challenge dài
.\Drone.ps1 sim-challenge

# World obstacle cơ bản
.\Drone.ps1 sim-avoid
```

Chờ Gazebo hiện drone và PX4 hoàn tất preflight. Không khởi động nhiều simulator cùng lúc.

### 2. Dry-run LiDAR, không arm hoặc gửi MAVLink

```powershell
.\Drone.ps1 avoid --dry-run --duration 10 --world slalom_yolo --log logs/slalom-dry-run.csv
```

### 3. Bay tránh vật cản một chiều trong SITL

```powershell
.\Drone.ps1 avoid --duration 120 --world slalom_yolo --max-radius 48 --log logs/slalom-avoid.csv
```

### 4. Bay đi–về, vẫn quét LiDAR/PID trên đường về

```powershell
.\Drone.ps1 avoid --duration 240 --world slalom_yolo --max-radius 48 `
  --return-home --return-at 42 --log logs/slalom-round-trip.csv
```

### 5. Chạy YOLO quan sát độc lập

Mở thêm cửa sổ sau khi world đã chạy:

```powershell
.\Drone.ps1 detect-slalom --fps 10 --device 0
.\Drone.ps1 view
```

YOLO có thể được chạy song song với `avoid`, nhưng không tác động lệnh bay.

### 6. Unified CLI và dashboard

```powershell
.\Drone.ps1 unified avoid --duration 120 --world slalom_yolo --max-radius 48
.\Drone.ps1 monitor --log logs/slalom-round-trip.csv
```

### 7. Replay log

```powershell
.\Drone.ps1 replay --log logs/slalom-round-trip.csv `
  --output output/slalom-round-trip-replay.png
```

Replay chỉ đọc CSV đã ghi. Log dry-run không có telemetry vị trí PX4 nên không thể dựng quỹ đạo bay.

## Profile tốc độ và benchmark SITL

Các profile nằm trong [`cau_hinh_toc_do.py`](cau_hinh_toc_do.py). Chúng chỉ đặt **giới hạn trên**; planner LiDAR/PID vẫn giảm tốc ở mỗi chu kỳ theo tuổi scan, khoảng cách phản ứng, quãng phanh và thời gian lách ngang. Không profile nào vượt `speed=3,0 m/s`, `max_accel=2,0 m/s²`, `max_lateral_speed=2,0 m/s` hoặc `brake_accel=2,0 m/s²`.

| Profile        | Tốc độ hành trình | Gia tốc lệnh | Tốc độ ngang | Phạm vi                                          |
| -------------- | ----------------: | -----------: | -----------: | ------------------------------------------------ |
| `conservative` |           2,0 m/s |     0,8 m/s² |      1,4 m/s | Fault injection và kiểm tra sensor               |
| `baseline`     |           2,5 m/s |     1,0 m/s² |      1,8 m/s | So sánh chuẩn SITL                               |
| `fast_sitl`    |           3,0 m/s |     1,5 m/s² |      1,8 m/s | Chỉ benchmark PX4 SITL/Gazebo, không là mặc định |

`fast_sitl` là opt-in. Nếu một lượt `fast_sitl` failed, mất sensor/telemetry/Offboard hoặc không land/disarm, không được thử lại trên cùng session; dừng simulator, lưu log, xem replay và quay về `baseline` cho lượt sau. Không có tự động tăng tốc ở world khác.

Với `fast_sitl`, scan vẫn hợp lệ nhưng già hơn 100 ms sẽ tự hạ cap tốc độ về `baseline` 2,5 m/s và ghi `profile_fallback_active=true`. Scan mất/quá cũ vẫn không được “cứu” bằng profile: planner gửi lệnh zero rồi mission fail/land theo timeout hiện có. Trong benchmark đã đo, median tuổi scan khoảng 31,7–31,9 ms; control cycle trung bình khoảng 105,5–105,6 ms, cực đại 115,4 ms ở profile fast có metric mới.

### Benchmark đã đo — `slalom_yolo`, cùng world và cùng return marker

Hai lượt `baseline` và hai lượt `fast_sitl` đã hoàn tất `COMPLETE`, land và disarm trong PX4 SITL/Gazebo. Tổng hợp được lưu tại [`logs/benchmark-baseline-vs-fast-sitl.json`](logs/benchmark-baseline-vs-fast-sitl.json):

| Metric trung bình                    |  Baseline | Fast SITL |             Thay đổi |
| ------------------------------------ | --------: | --------: | -------------------: |
| Thời gian vòng điều khiển nhiệm vụ   |  85,436 s |  78,068 s |           **-8,62%** |
| Tốc độ đo cực đại                    | 2,541 m/s | 3,043 m/s |              +19,79% |
| Khoảng hở trước tối thiểu            |   2,269 m |   2,212 m |             -0,057 m |
| Sai số PID ngang cực đại             |   4,313 m |   3,856 m |              -10,58% |
| Số mẫu `WAIT_SCAN`/`BLOCKED`/`BRAKE` |         0 |         0 |           không tăng |
| Đổi bên né                           |         6 |         6 |           không tăng |
| Sai số Home cuối                     |   1,094 m |   1,165 m | trong bán kính 1,5 m |

`fast_sitl` chỉ đạt tiêu chí benchmark nội bộ vì mọi lượt đều `COMPLETE`, đã land/disarm, không tăng stop-state và khoảng hở giảm dưới ngưỡng cho phép 0,10 m. Đây **không** là chứng nhận tốc độ an toàn ngoài đời, không chứng minh tính lặp lại theo seed/nhiễu/vật động, và không được dùng làm mặc định.

Lệnh so sánh chỉ đọc log:

```powershell
.\Drone.ps1 benchmark `
  --baseline logs/baseline-conservative-round-trip.csv logs/baseline-round-trip-02.csv `
  --fast-sitl logs/fast-sitl-round-trip-01.csv logs/fast-sitl-round-trip-02.csv `
  --output logs/benchmark-baseline-vs-fast-sitl.json
```

Lệnh chạy profile mô phỏng:

```powershell
# Chỉ dùng sau dry-run thành công và chỉ trong PX4 SITL/Gazebo
.\Drone.ps1 avoid --profile fast_sitl --duration 240 --world slalom_yolo `
  --max-radius 48 --return-home --return-at 42 --log logs/fast-sitl-run.csv
```

## Kiểm chứng lặp lại, evidence và safety audit

Mỗi run mới ghi manifest môi trường, Git revision/dirty state, phiên bản dependency, tham số đã resolve và SHA-256 của CSV vào JSON sidecar qua [`reproducibility.py`](reproducibility.py) và [`flight_log.py`](flight_log.py). Hash chỉ được tạo sau khi CSV đóng; replay/benchmark phải xem CSV và JSON là cặp evidence bất biến.

[`safety_supervisor.py`](safety_supervisor.py) chuyển trạng thái sensor/telemetry/Offboard/planner thành severity và reason code có thể audit. [`lidar_risk.py`](lidar_risk.py) dùng return LiDAR thực trong corridor để ước lượng closing speed, TTC và uncertainty margin. Hai module này chỉ được phép **hold hoặc giảm cap** lệnh đã sinh từ planner LiDAR; không có MAVSDK, Gazebo, ROS, YOLO, route, yaw hay actuator authority. Scan `range_max` no-hit hợp lệ vẫn là free-space tới sensor horizon, không tạo false hold.

### Scenario matrix offline

Lệnh dưới dùng mô hình động học đã test trong [`offline_slalom.py`](offline_slalom.py): velocity lag, segment collision, LiDAR noise, isolated/burst dropout và control jitter. Nó không khởi động PX4/Gazebo và không tạo MAVLink.

```powershell
# Sáu họ scenario x 10 seed cố định, ghi JSON evidence
.\Drone.ps1 scenario-matrix --seeds 10 --output logs/offline-scenario-matrix.json
```

Một matrix chỉ PASS nếu mọi scenario/seed đạt goal mà không có segment collision. Đây là evidence offline, không thay thế SITL hoặc chứng nhận an toàn hardware.

### Benchmark SITL nhiều lượt

[`scripts/bao_cao_benchmark_sitl.py`](scripts/bao_cao_benchmark_sitl.py) hiện báo mean, median, p95 và worst-case. `fast_sitl` chỉ accepted khi mỗi profile có tối thiểu `--min-runs` run, mọi run fast `COMPLETE`, landing/disarm đã xác nhận, không tăng stop-state và worst-case front clearance không giảm quá 0,10 m.

```powershell
.\Drone.ps1 benchmark --min-runs 3 `
  --baseline logs\baseline-01.csv logs\baseline-02.csv logs\baseline-03.csv `
  --fast-sitl logs\fast-01.csv logs\fast-02.csv logs\fast-03.csv `
  --output logs\benchmark-multirun.json
```

Replay bằng [`scripts/plot_slalom_replay.py`](scripts/plot_slalom_replay.py) hiển thị thêm safety severity, LiDAR age, temporal corridor/TTC và risk speed cap khi log có schema mới; log cũ vẫn replay được.

## Kiểm thử

Chạy tại root project:

```powershell
# Install Python dependencies in the WSL venv with reviewed major-version bounds.
pip install -r requirements.txt -c requirements-constraints.txt

python -m py_compile avoid_fly.py obstacle_avoidance.py mission_control.py simulation_guard.py sensor_fusion.py yolo_gz_node.py reproducibility.py safety_supervisor.py lidar_risk.py scripts\unified_drone.py scripts\sim_avoid.py scripts\run_scenario_matrix.py
python -m unittest discover -s tests -v
```

GitHub Actions at [`.github/workflows/python-quality.yml`](.github/workflows/python-quality.yml) compiles the pure-Python stack and runs regression with a 12-minute job ceiling. It deliberately does not run PX4/Gazebo in hosted CI.

Regression bao phủ:

- giữ trục world khi yaw EKF lệch và hạ cánh khi không căn hướng được;
- NED ↔ trục hành trình, đảo khung về Home nhưng không đổi yaw mũi;
- LiDAR thiếu/cũ/future, timestamp lùi, `NaN`/`Inf`, telemetry lỗi;
- PID anti-windup, giới hạn vận tốc ngang/gia tốc, braking envelope và hysteresis đổi bên;
- safety reason codes, temporal LiDAR closing-speed/TTC cap, uncertainty margin và clear-horizon non-regression;
- sealed CSV/JSON evidence, reproducibility manifest và deterministic offline fault-injection matrix;
- reset planner/PID sau turnaround, quét/né trên đường về, giảm tốc/ổn định Home;
- xác nhận landing và disarm trước `COMPLETE`;
- YOLO không có API hay đường dữ liệu để điều khiển bay;
- CLI unified truyền tham số world/trục hành trình đúng;
- alias tiếng Anh tương thích với tên tiếng Việt mới khi test/caller hiện có cần chúng.

## Bảng đổi tên nội bộ

Tên log, topic, API MAVSDK/ROS/Gazebo/PX4, option PowerShell đang công bố, SDF/world/model và cột log cũ không bị đổi. Các helper nội bộ sau dùng tên tiếng Việt không dấu; alias tiếng Anh chỉ được giữ tạm cho tương thích test/caller cũ.

| Tên tiếng Anh cũ              | Tên tiếng Việt mới            | Ý nghĩa                                                       | File                                                   |
| ----------------------------- | ----------------------------- | ------------------------------------------------------------- | ------------------------------------------------------ |
| `course_motion`               | `tinh_chuyen_dong_hanh_trinh` | Đổi telemetry NED sang chuyển động theo trục hành trình world | [`avoid_fly.py`](avoid_fly.py)                         |
| `ned_velocity`                | `doi_van_toc_sang_ned`        | Đổi lệnh tiến/phải thành vận tốc PX4 NED                      | [`avoid_fly.py`](avoid_fly.py)                         |
| `reverse_course_yaw`          | `dao_huong_hanh_trinh`        | Đảo trục điều hướng 180° khi về Home                          | [`avoid_fly.py`](avoid_fly.py)                         |
| `heading_error_deg`           | `tinh_sai_so_huong_do`        | Sai số yaw nhỏ nhất theo độ                                   | [`avoid_fly.py`](avoid_fly.py)                         |
| `limit_horizontal`            | `gioi_han_van_toc_ngang`      | Giới hạn độ lớn vector vận tốc ngang                          | [`avoid_fly.py`](avoid_fly.py)                         |
| `slew_horizontal`             | `gioi_han_gia_toc_ngang`      | Giới hạn biến thiên vector vận tốc ngang                      | [`avoid_fly.py`](avoid_fly.py)                         |
| `altitude_hold_down_velocity` | `tinh_van_toc_giu_do_cao`     | Vòng giữ độ cao, output NED down                              | [`avoid_fly.py`](avoid_fly.py)                         |
| `wait_for_health`             | `cho_trang_thai_san_sang`     | Chờ health/preflight PX4 bắt buộc                             | [`avoid_fly.py`](avoid_fly.py)                         |
| `safe_speed`                  | `tinh_van_toc_an_toan`        | Tính vận tốc theo quãng phản ứng và phanh                     | [`obstacle_avoidance.py`](obstacle_avoidance.py)       |
| `reaction_distance`           | `tinh_khoang_cach_phan_ung`   | Tính khoảng nhìn tối thiểu theo vận tốc                       | [`obstacle_avoidance.py`](obstacle_avoidance.py)       |
| `_add_avoid_args`             | `_them_tham_so_tranh_vat_can` | Thêm tham số CLI LiDAR/PID                                    | [`scripts/unified_drone.py`](scripts/unified_drone.py) |
| `_build_avoid_args`           | `_tao_tham_so_tranh_vat_can`  | Tạo namespace cho runner; alias cũ vẫn có                     | [`scripts/unified_drone.py`](scripts/unified_drone.py) |
| `signal_group`                | `gui_tin_hieu_nhom`           | Gửi tín hiệu chỉ đến group của phiên SITL sở hữu              | [`scripts/sim_avoid.py`](scripts/sim_avoid.py)         |
| `wait_for_group`              | `cho_nhom_tien_trinh`         | Chờ process group của phiên SITL kết thúc                     | [`scripts/sim_avoid.py`](scripts/sim_avoid.py)         |
| `cleanup`                     | `don_dep_phien`               | Dọn process group phiên mô phỏng đã sở hữu                    | [`scripts/sim_avoid.py`](scripts/sim_avoid.py)         |
| `prepare_session`             | `chuan_bi_phien`              | Tạo phiên SITL mới, không thay calibration/preflight stock    | [`scripts/sim_avoid.py`](scripts/sim_avoid.py)         |

## World và giới hạn nghiên cứu

[`worlds/obstacle_yolo.sdf`](worlds/obstacle_yolo.sdf), [`worlds/slalom_yolo.sdf`](worlds/slalom_yolo.sdf) và [`worlds/challenge_yolo.sdf`](worlds/challenge_yolo.sdf) chỉ chứa hình học mô phỏng. Planner không đọc SDF/world hoặc tọa độ cổng.

Các giới hạn còn lại trước khi có thể nghiên cứu chuyển sang drone thật:

1. Dry-run SITL ngày 20/09/2026 đã hoàn tất `OBSERVATION_COMPLETE` sau khi adapter Gazebo chuẩn hóa riêng `+Inf` no-hit thành `range_max`; đây chỉ là xác nhận đường dữ liệu sensor-only, không phải xác nhận bay SITL hay phần cứng.
2. LiDAR 2D ở một cao độ không bao phủ dây, vật thấp/cao, địa hình, động lực cánh quạt hoặc vật cản 3D phức tạp.
3. Không có SLAM, bản đồ bền vững, lập kế hoạch toàn cục, dự đoán vật động hay redundancy sensor/estimator.
4. SITL không đại diện đầy đủ cho độ trễ, nhiễu, rung, gió, GPS/magnetometer lỗi, battery failsafe, giao thức radio, geofence và quy trình vận hành thật.
5. Trước bất kỳ nghiên cứu phần cứng nào cần phân tích rủi ro độc lập, HIL có giám sát, giới hạn lực đẩy/không gian, kill switch, geofence, review PX4 parameter, tuân thủ pháp lý và kế hoạch thử nghiệm do chuyên gia an toàn phê duyệt. Xem checklist không thực thi tại [`docs/NGHIEN_CUU_TRUOC_PHAN_CUNG.md`](docs/NGHIEN_CUU_TRUOC_PHAN_CUNG.md).
