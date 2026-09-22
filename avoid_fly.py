#!/usr/bin/env python3
"""PX4/Gazebo SITL: continuous lateral bypass with fixed heading; Ctrl+C lands."""
import argparse
import asyncio
import math
import os
os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")
from pathlib import Path
import signal
import threading
import time
import uuid

from cau_hinh_toc_do import ap_dung_ho_so_toc_do, lay_ho_so_toc_do, tinh_gioi_han_toc_do_ho_so
from circle_fly import wait_for
from flight_log import FlightRecorder, utc_now
from lidar_risk import RiskConfig, TemporalLidarRisk
from mission_control import MissionConfig, MissionDirective, RoundTripMission
from obstacle_avoidance import AvoidanceConfig, Motion, Planner, Scan
from reproducibility import environment_manifest
from safety_supervisor import SafetySupervisor
from simulation_guard import SimulationGuard


def tinh_chuyen_dong_hanh_trinh(vi_tri, van_toc, yaw_do, goc_toa_do, goc_huong_hanh_trinh):
    """Đổi telemetry PX4 NED sang trục world hành trình cố định tiến/phải."""
    goc_rad = math.radians(goc_huong_hanh_trinh)
    do_lech_bac, do_lech_dong = vi_tri.north_m-goc_toa_do.north_m, vi_tri.east_m-goc_toa_do.east_m
    do_lech_yaw = math.radians((yaw_do-goc_huong_hanh_trinh+180) % 360-180)
    return Motion(van_toc.north_m_s*math.cos(goc_rad)+van_toc.east_m_s*math.sin(goc_rad),
                  -van_toc.north_m_s*math.sin(goc_rad)+van_toc.east_m_s*math.cos(goc_rad),
                  -do_lech_bac*math.sin(goc_rad)+do_lech_dong*math.cos(goc_rad), do_lech_yaw,
                  do_lech_bac*math.cos(goc_rad)+do_lech_dong*math.sin(goc_rad))


def doi_van_toc_sang_ned(van_toc_tien, van_toc_phai, goc_huong_hanh_trinh):
    """Đổi lệnh tiến/phải ở trục hành trình sang PX4 NED."""
    goc_rad = math.radians(goc_huong_hanh_trinh)
    return (van_toc_tien*math.cos(goc_rad)-van_toc_phai*math.sin(goc_rad),
            van_toc_tien*math.sin(goc_rad)+van_toc_phai*math.cos(goc_rad))


def dao_huong_hanh_trinh(goc_huong_hanh_trinh):
    """Đảo trục điều hướng khi về Home, không thay đổi yaw setpoint của mũi drone."""
    return (goc_huong_hanh_trinh+180.0) % 360.0


def tinh_sai_so_huong_do(goc_do_duoc, goc_mong_muon):
    """Sai số yaw nhỏ nhất theo độ, xử lý đúng qua mốc Bắc 0°/360°."""
    return abs((goc_do_duoc-goc_mong_muon+180.0) % 360.0-180.0)


def validate_flight_state(nav, origin, now, *, startup=False):
    """Fail closed throughout takeoff/alignment, not only after navigation starts."""
    if (now-nav["position_at"] > 0.75 or now-nav["attitude_at"] > 0.75):
        raise RuntimeError("Mat telemetry; ha canh")
    p, v = nav["position"], nav["velocity"]
    values = (p.north_m, p.east_m, p.down_m, v.north_m_s, v.east_m_s,
              v.down_m_s, nav["yaw"], nav["roll"], nav["pitch"])
    if not all(math.isfinite(value) for value in values):
        raise RuntimeError("Telemetry contains non-finite flight state")
    if max(abs(nav["roll"]), abs(nav["pitch"])) > 35.0:
        raise RuntimeError("Drone tilted beyond 35 degrees; aborting")
    if startup and math.hypot(p.north_m-origin.north_m, p.east_m-origin.east_m) > 2.0:
        raise RuntimeError("Takeoff/alignment drift exceeded 2 m; aborting before navigation")


def gioi_han_van_toc_ngang(van_toc_tien, van_toc_phai, gioi_han):
    """Giữ hướng vector lệnh khi áp giới hạn tốc độ ngang của nhiệm vụ."""
    do_lon = math.hypot(van_toc_tien, van_toc_phai)
    if gioi_han is None or do_lon <= gioi_han or do_lon <= 1e-9:
        return van_toc_tien, van_toc_phai
    ty_le = max(0.0, gioi_han)/do_lon
    return van_toc_tien*ty_le, van_toc_phai*ty_le


def gioi_han_gia_toc_ngang(van_toc_tien_truoc, van_toc_phai_truoc, van_toc_tien_dich,
                            van_toc_phai_dich, gia_toc_m_s2, dt):
    """Giới hạn thay đổi vector lệnh thường quy mà không đổi hướng đích."""
    chenh_tien = van_toc_tien_dich-van_toc_tien_truoc
    chenh_phai = van_toc_phai_dich-van_toc_phai_truoc
    do_lon_chenh = math.hypot(chenh_tien, chenh_phai)
    duoc_phep = max(0.0, gia_toc_m_s2)*max(0.0, dt)
    if do_lon_chenh <= duoc_phep or do_lon_chenh <= 1e-9:
        return van_toc_tien_dich, van_toc_phai_dich
    ty_le = duoc_phep/do_lon_chenh
    return van_toc_tien_truoc+chenh_tien*ty_le, van_toc_phai_truoc+chenh_phai*ty_le


def tinh_van_toc_giu_do_cao(do_cao_m, do_cao_muc_tieu_m, he_so_p, gioi_han_m_s):
    """Vòng ngoài giữ độ cao; kết quả theo PX4 NED, down dương."""
    if not all(math.isfinite(gia_tri) for gia_tri in
               (do_cao_m, do_cao_muc_tieu_m, he_so_p, gioi_han_m_s)):
        return 0.0
    return max(-gioi_han_m_s, min(gioi_han_m_s, he_so_p*(do_cao_m-do_cao_muc_tieu_m)))


async def cho_trang_thai_san_sang(drone, truong_bat_buoc, thoi_gian_cho):
    """Chờ PX4 báo tất cả điều kiện preflight cần thiết, không hạ ngưỡng kiểm tra."""
    lan_cuoi = None
    def san_sang(suc_khoe):
        nonlocal lan_cuoi
        lan_cuoi = suc_khoe
        return all(getattr(suc_khoe, truong) for truong in truong_bat_buoc)
    try:
        return await wait_for(drone.telemetry.health(), san_sang, thoi_gian_cho)
    except asyncio.TimeoutError as loi:
        chua_dat = [truong for truong in truong_bat_buoc
                    if lan_cuoi is None or not getattr(lan_cuoi, truong)]
        raise RuntimeError(
            f"PX4 preflight quá thời gian {thoi_gian_cho}s: {', '.join(chua_dat)}") from loi


# Alias tiếng Anh chỉ để tương thích các caller/test hiện có; mã mới dùng tên tiếng Việt.
course_motion = tinh_chuyen_dong_hanh_trinh
ned_velocity = doi_van_toc_sang_ned
reverse_course_yaw = dao_huong_hanh_trinh
heading_error_deg = tinh_sai_so_huong_do
limit_horizontal = gioi_han_van_toc_ngang
slew_horizontal = gioi_han_gia_toc_ngang
altitude_hold_down_velocity = tinh_van_toc_giu_do_cao
wait_for_health = cho_trang_thai_san_sang


class KhoaDieuKhienBay:
    """Khóa liên tiến trình, bảo đảm chỉ một controller có thể sở hữu Offboard."""
    def __init__(self):
        self.duong_dan = Path.home() / ".cache" / "ai-drone" / "flight.lock"
        self.duong_dan.parent.mkdir(parents=True, exist_ok=True)
        self.luong = None

    def acquire(self):
        try:
            import fcntl
        except ImportError as loi:
            raise RuntimeError("Bộ điều khiển bay chỉ hỗ trợ Ubuntu/WSL PX4 SITL") from loi
        self.luong = self.duong_dan.open("a+", encoding="utf-8")
        try:
            fcntl.flock(self.luong.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as loi:
            self.luong.close()
            self.luong = None
            raise RuntimeError("Đã có tiến trình điều khiển bay; chờ nó hạ cánh và thoát") from loi
        self.luong.seek(0)
        self.luong.truncate()
        self.luong.write(f"pid={os.getpid()}\n")
        self.luong.flush()

    def close(self):
        if self.luong is None:
            return
        try:
            import fcntl
            fcntl.flock(self.luong.fileno(), fcntl.LOCK_UN)
        finally:
            self.luong.close()
            self.luong = None


class Lidar:
    def __init__(self, topic):
        import gz.transport13 as transport
        from gz.msgs10.laserscan_pb2 import LaserScan
        self.topic = topic
        self.lock = threading.Lock()
        self.scan = None
        self.error = "Waiting for first scan"
        self.stamp = None
        self.sim_time_s = None
        self.node = transport.Node()
        if not self.node.subscribe(LaserScan, topic, self.receive):
            raise RuntimeError(f"Cannot subscribe to {topic}")

    def receive(self, msg):
        """Nhận một scan mới; lỗi timestamp/range luôn làm dữ liệu trở nên không hợp lệ."""
        with self.lock:
            try:
                giay = int(msg.header.stamp.sec)
                nano_giay = int(msg.header.stamp.nsec)
            except (AttributeError, TypeError, ValueError, OverflowError):
                self.scan = None
                self.error = "Timestamp LiDAR không hợp lệ"
                return
            if giay < 0 or not 0 <= nano_giay < 1_000_000_000:
                self.scan = None
                self.error = "Timestamp LiDAR nằm ngoài miền hợp lệ"
                return
            tem = (giay, nano_giay)
            if self.stamp is not None and tem <= self.stamp:
                # Timestamp lặp/lùi phải làm scan cũ tự hết hạn theo timeout,
                # tuyệt đối không được làm mới freshness của dữ liệu đóng băng.
                return
            self.stamp = tem
            try:
                # Chỉ adapter Gazebo được phép chuẩn hóa +Inf no-hit thành
                # range_max hữu hạn. Planner không nhận hoặc tự chấp nhận Inf.
                self.scan = Scan.from_message(
                    msg, time.monotonic(), cho_phep_inf_khong_va_cham=True)
                self.sim_time_s = float(giay)+float(nano_giay)/1_000_000_000.0
                self.error = ""
            except (TypeError, ValueError, OverflowError) as loi:
                self.scan = None
                self.error = str(loi)

    def latest(self):
        with self.lock:
            return self.scan

    def snapshot(self):
        with self.lock:
            return self.scan, self.sim_time_s

    def close(self):
        self.node.unsubscribe(self.topic)


async def run(args):
    """Chạy duy nhất bộ điều khiển LiDAR + PID trên PX4 SITL/Gazebo."""
    ten_ho_so = getattr(args, "profile", None)
    ho_so = ap_dung_ho_so_toc_do(args, ten_ho_so) if ten_ho_so else None
    config = AvoidanceConfig(
        speed=args.speed,
        detect_distance=getattr(args, "detect_distance", 10.0),
        kp=getattr(args, "kp", 0.8),
        ki=getattr(args, "ki", 0.03),
        kd=getattr(args, "kd", 0.4),
        approach_margin=getattr(args, "approach_margin", 1.6),
        tracking_fraction=getattr(args, "tracking_fraction", 0.65),
        clear_fov_deg=getattr(args, "clear_fov_deg", 170.0),
        corridor_half_width=getattr(args, "corridor_half_width", 10.0),
        side_switch_hysteresis_m=getattr(args, "side_switch_hysteresis", 0.5),
        side_switch_min_interval=getattr(args, "side_switch_min_interval", 1.2),
        speed_reduction_start_frac=getattr(args, "speed_reduction_start_frac", 0.9),
        min_forward_speed=getattr(args, "min_forward_speed", 1.2),
        max_lateral_speed=getattr(args, "max_lateral_speed", 1.8),
        max_accel=getattr(args, "max_accel", 1.0),
        brake_accel=getattr(args, "brake_accel", 1.5),
    )
    planner = Planner(config)
    temporal_risk = TemporalLidarRisk(RiskConfig(
        corridor_half_width_m=config.clearance,
        clearance_m=config.clearance,
        brake_accel_m_s2=config.brake_accel,
        reaction_time_s=config.reaction_time,
    ))
    safety_supervisor = SafetySupervisor()
    lidar = Lidar(args.topic)
    khoa_dieu_khien = None
    drone = None
    simulator_guard = None
    armed = False
    offboard = False
    background = []
    nav = {"position": None, "velocity": None, "position_at": 0.0, "mode": None,
           "yaw": None, "roll": None, "pitch": None, "attitude_at": 0.0}
    start_position = None
    course_yaw = 0.0
    navigation_yaw = 0.0
    
    return_home = bool(getattr(args, "return_home", False))
    return_at = float(getattr(args, "return_at", 42.0))
    return_arrival_radius = float(getattr(args, "return_arrival_radius", 1.5))
    mission_config = MissionConfig(
        return_enabled=return_home,
        return_at_m=return_at,
        arrival_radius_m=return_arrival_radius,
        approach_radius_m=float(getattr(args, "home_approach_radius", 5.0)),
        turnaround_speed_m_s=float(getattr(args, "turnaround_speed", 0.15)),
        turnaround_settle_s=float(getattr(args, "turnaround_settle_seconds", 0.5)),
        turnaround_timeout_s=float(getattr(args, "turnaround_timeout", 12.0)),
        home_settle_speed_m_s=float(getattr(args, "home_settle_speed", 0.15)),
        home_settle_s=float(getattr(args, "home_settle_seconds", 1.0)),
        home_settle_timeout_s=float(getattr(args, "home_settle_timeout", 10.0)),
        return_timeout_s=float(getattr(args, "return_timeout", 180.0)),
        max_radius_m=float(args.max_radius),
        min_altitude_m=float(getattr(args, "min_flight_altitude", 1.5)),
        max_altitude_m=float(getattr(args, "max_flight_altitude", 4.5)),
        approach_speed_m_s=float(getattr(args, "home_approach_speed", 0.75)),
    )
    mission = RoundTripMission(mission_config)
    stopping = False
    task = asyncio.current_task()
    loop = asyncio.get_running_loop()
    def stop():
        nonlocal stopping
        if not stopping:
            stopping = True
            task.cancel()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stop)
    run_id = uuid.uuid4().hex[:12]
    session_started = time.monotonic()
    log_path = Path(args.log) if getattr(args, "log", None) else Path("logs") / time.strftime("avoid-%Y%m%d-%H%M%S.csv")
    recorder = FlightRecorder(log_path, {
        "schema_version": 3,
        "run_id": run_id,
        "started_utc": utc_now(),
        "world": getattr(args, "world", "unknown"),
        "reproducibility": environment_manifest(Path(__file__).resolve().parent),
        "mode": "sensor-only" if args.dry_run else "px4-gazebo-sitl",
        "parameters": {
            "altitude_m": args.altitude, "speed_m_s": args.speed,
            "speed_profile": ho_so.ten if ho_so else "custom",
            "profile_simulation_only": ho_so.chi_duoc_mo_phong if ho_so else False,
            "max_lateral_speed_m_s": config.max_lateral_speed,
            "max_command_acceleration_m_s2": config.max_accel,
            "detect_distance_m": getattr(args, "detect_distance", 10.0),
            "kp": getattr(args, "kp", 0.8), "ki": getattr(args, "ki", 0.03),
            "kd": getattr(args, "kd", 0.4), "duration_s": args.duration,
            "return_home": return_home, "return_at_m": return_at,
            "arrival_radius_m": return_arrival_radius,
            "approach_radius_m": mission_config.approach_radius_m,
            "settle_speed_m_s": mission_config.home_settle_speed_m_s,
            "turnaround_approach_deceleration_m_s2": mission_config.turnaround_approach_deceleration_m_s2,
            "settle_seconds": mission_config.home_settle_s,
            "altitude_hold_kp": getattr(args, "altitude_kp", 0.6),
            "max_vertical_speed_m_s": getattr(args, "max_vertical_speed", 0.5),
            "expected_course_yaw_deg": getattr(args, "expected_course_yaw", None),
            "course_yaw_tolerance_deg": getattr(args, "course_yaw_tolerance", 20.0),
            "course_yaw_alignment_tolerance_deg": min(
                5.0, getattr(args, "course_yaw_tolerance", 20.0)),
            "course_yaw_alignment_settle_s": 0.8,
            "course_yaw_alignment_timeout_s": 15.0,
            "launch_pad_radius_m": getattr(args, "launch_pad_radius", 2.0),
            "max_radius_m": mission_config.max_radius_m,
            "altitude_band_m": [mission_config.min_altitude_m, mission_config.max_altitude_m],
            "brake_accel": config.brake_accel,
            "approach_margin": config.approach_margin,
            "tracking_fraction": config.tracking_fraction,
            "clear_fov_deg": config.clear_fov_deg,
            "corridor_half_width": config.corridor_half_width,
            "side_switch_hysteresis_m": config.side_switch_hysteresis_m,
            "side_switch_min_interval": config.side_switch_min_interval,
            "speed_reduction_start_frac": config.speed_reduction_start_frac,
            "min_forward_speed": config.min_forward_speed,
        },
    })
    result_status = "FAILED"
    result_reason = "Runner ended before mission completion"
    landing_confirmed = False
    disarmed_confirmed = False
    started = session_started
    try:
        if not args.dry_run and os.name == "posix":
            # PX4 SITL/Gazebo chỉ được hỗ trợ trên Ubuntu/WSL. Đặt khóa ở đây để
            # cả run.sh lẫn unified CLI đều không thể tranh quyền Offboard.
            khoa_dieu_khien = KhoaDieuKhienBay()
            khoa_dieu_khien.acquire()
        print(f"Dang cho LiDAR {args.topic}...", flush=True)
        deadline = time.monotonic() + 20
        while lidar.latest() is None:
            if time.monotonic() > deadline:
                raise RuntimeError(f"Khong co LiDAR hop le: {lidar.error}. Chay Drone.ps1 sim-avoid truoc.")
            await asyncio.sleep(0.1)
        if not args.dry_run:
            simulator_guard = SimulationGuard(getattr(args, "world", "unknown"))
            from mavsdk import System
            from mavsdk.offboard import VelocityBodyYawspeed, VelocityNedYaw
            drone = System()
            await asyncio.wait_for(drone.connect(system_address="udpin://127.0.0.1:14540"), 20)
            await wait_for(drone.core.connection_state(), lambda s: s.is_connected, 30)
            # Loopback alone is NOT proof of SITL (a hardware link may be forwarded).
            if await asyncio.wait_for(drone.param.get_param_int("SIM_GZ_EN"), 5) != 1:
                raise RuntimeError("Chi ho tro PX4 SITL voi Gazebo (SIM_GZ_EN=1)")
            print("PX4 Gazebo SITL da ket noi. Dang cho dinh vi...", flush=True)
            await cho_trang_thai_san_sang(drone, ("is_local_position_ok", "is_global_position_ok",
                                         "is_home_position_ok"), 90)
            # A completed landing leaves PX4 in LAND, which forbids re-arming.
            # Hold also avoids requiring a simulated RC stick in manual modes.
            if await wait_for(drone.telemetry.armed(), lambda _: True, 5):
                raise RuntimeError("Drone dang armed; hay ha canh truoc khi bat dau bai moi")
            await asyncio.wait_for(drone.action.hold(), 10)
            await cho_trang_thai_san_sang(drone, ("is_armable",), 60)
            await asyncio.wait_for(drone.telemetry.set_rate_position_velocity_ned(20), 5)
            await asyncio.wait_for(drone.telemetry.set_rate_attitude_euler(20), 5)
            async def positions():
                async for p in drone.telemetry.position_velocity_ned():
                    nav["position"], nav["velocity"], nav["position_at"] = p.position, p.velocity, time.monotonic()
            async def attitudes():
                async for a in drone.telemetry.attitude_euler():
                    nav["yaw"], nav["attitude_at"] = a.yaw_deg, time.monotonic()
                    nav["roll"], nav["pitch"] = a.roll_deg, a.pitch_deg
            async def modes():
                async for mode in drone.telemetry.flight_mode():
                    nav["mode"] = mode
            background = [asyncio.create_task(positions()), asyncio.create_task(modes()), asyncio.create_task(attitudes())]
            deadline = time.monotonic() + 10
            while nav["position"] is None or nav["yaw"] is None:
                if time.monotonic() > deadline:
                    raise RuntimeError("No local position/attitude telemetry")
                await asyncio.sleep(0.1)
            start_position = nav["position"]
            # EKF local origin is NOT a reliable map origin. Check the actual
            # Gazebo pad and heading without feeding truth to the controller.
            ready_since = None
            preflight_deadline = time.monotonic()+30.0
            while True:
                now = time.monotonic()
                try:
                    validate_flight_state(nav, start_position, now, startup=True)
                    simulator_guard.check(nav["yaw"], now,
                                          on_pad=simulator_guard.world in {"slalom_yolo", "challenge_yolo"},
                                          pad_radius=getattr(args, "launch_pad_radius", 2.0))
                    if (max(abs(nav["roll"]), abs(nav["pitch"])) > 3.0
                            or math.hypot(nav["velocity"].north_m_s, nav["velocity"].east_m_s) > 0.2):
                        raise RuntimeError("Ground attitude/velocity has not settled")
                    ready_since = ready_since or now
                    if now-ready_since >= 2.0:
                        break
                except RuntimeError as exc:
                    ready_since = None
                    if now >= preflight_deadline:
                        raise RuntimeError(f"Preflight integrity failed: {exc}") from exc
                await asyncio.sleep(0.1)
            recorder.metadata["preflight_evidence"] = {
                **simulator_guard.snapshot(), "yaw_deg": nav["yaw"],
                "roll_deg": nav["roll"], "pitch_deg": nav["pitch"],
                "north_m": start_position.north_m, "east_m": start_position.east_m,
            }
            if Planner(config).update(lidar.latest(), time.monotonic()).state in ("WAIT_SCAN", "BLOCKED"):
                raise RuntimeError("Cam bien/vung cat canh chua san sang")
            await asyncio.wait_for(drone.action.set_takeoff_altitude(args.altitude), 5)
            armed = True
            # Retry arming up to 5 times (1.0s delay) if PX4 EKF is still converging
            for attempt in range(5):
                try:
                    await cho_trang_thai_san_sang(drone, ("is_global_position_ok", "is_home_position_ok", "is_armable"), 15)
                    await asyncio.wait_for(drone.action.arm(), 10)
                    break
                except Exception as exc:
                    if attempt == 4:
                        raise
                    print(f"PX4 chua san sang arm (lan {attempt + 1}/5): {exc}. Doi EKF hoi tu...", flush=True)
                    await asyncio.sleep(1.0)
            await asyncio.wait_for(drone.action.takeoff(), 10)
            # Navigation events use a later zero; keep startup on an explicitly
            # named session clock instead of mixing two elapsed-time origins.
            recorder.metadata["takeoff_started_session_elapsed_s"] = time.monotonic()-session_started
            recorder.metadata["takeoff_started_utc"] = utc_now()
            takeoff_deadline = time.monotonic()+45.0
            while True:
                now = time.monotonic()
                validate_flight_state(nav, start_position, now, startup=True)
                simulator_guard.check(nav["yaw"], now)
                if start_position.down_m-nav["position"].down_m >= args.altitude*0.9:
                    break
                if now >= takeoff_deadline:
                    raise RuntimeError("Takeoff did not reach target altitude within 45 seconds")
                await asyncio.sleep(0.1)
            await asyncio.wait_for(drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 0)), 3)
            await asyncio.sleep(1.1)
            measured_takeoff_yaw = nav["yaw"]
            if not math.isfinite(measured_takeoff_yaw):
                raise RuntimeError("Invalid yaw telemetry")
            expected_yaw = getattr(args, "expected_course_yaw", None)
            yaw_tolerance = getattr(args, "course_yaw_tolerance", 20.0)
            # A multicopter can yaw while climbing before Offboard owns its yaw
            # setpoint.  For a world with a declared course, acquire that heading
            # at zero horizontal velocity instead of accepting a diagonal course
            # or rejecting an otherwise healthy simulator startup.
            course_yaw = (expected_yaw % 360.0 if expected_yaw is not None
                          else measured_takeoff_yaw)
            navigation_yaw = course_yaw
            await asyncio.wait_for(drone.offboard.set_velocity_ned(VelocityNedYaw(0, 0, 0, course_yaw)), 3)
            await asyncio.wait_for(drone.offboard.start(), 10)
            offboard = True
            if drone is not None:
                print(f"Dang can huong mui theo truc san {course_yaw:.1f} do...", flush=True)
                # The declared world axis is authoritative.  Never "fix" an
                # alignment error by redefining the course from the estimator:
                # that turns a small yaw error into metres of cross-track drift
                # and makes the LiDAR planner reason about the wrong geometry.
                align_deadline = time.monotonic() + 15.0
                aligned_since = None
                align_tolerance = min(5.0, yaw_tolerance)
                while True:
                    now = time.monotonic()
                    validate_flight_state(nav, start_position, now, startup=True)
                    simulator_guard.check(nav["yaw"], now)
                    if (now-nav["position_at"] > 0.75 or now-nav["attitude_at"] > 0.75
                            or any(t.done() for t in background)):
                        raise RuntimeError("Mat telemetry khi can huong san")
                    error = tinh_sai_so_huong_do(nav["yaw"], course_yaw)
                    speed = math.hypot(nav["velocity"].north_m_s, nav["velocity"].east_m_s)
                    altitude = start_position.down_m-nav["position"].down_m
                    if (error <= align_tolerance and speed <= 0.2
                            and abs(altitude-args.altitude) <= 0.3
                            and abs(nav["velocity"].down_m_s) <= 0.2):
                        aligned_since = aligned_since or now
                        if now-aligned_since >= 0.8:
                            break
                    else:
                        aligned_since = None
                    if now >= align_deadline:
                        raise RuntimeError(
                            f"Could not align course yaw: measured {nav['yaw']:.1f} deg, "
                            f"target {course_yaw:.1f}+/-{align_tolerance:.1f} deg")
                    await asyncio.wait_for(drone.offboard.set_velocity_ned(
                        VelocityNedYaw(0, 0, 0, course_yaw)), 0.5)
                    await asyncio.sleep(0.1)
        return_text = (f" Sau {return_at:g} m se quet LiDAR + PID tren duong ve Home."
                       if return_home else "")
        profile_text = f" profile {ho_so.ten}" if ho_so else " profile custom"
        print(f"Bay tien + dich ngang PID,{profile_text}, giu huong {course_yaw:.1f} do. Bat dau ne tu {config.detect_distance:g} m (tang theo toc do). Ctrl+C de ha canh.{return_text}", flush=True)
        print(f"Log: {log_path.resolve()}", flush=True)
        started = time.monotonic()
        recorder.event(0.0, "OBSERVE_ONLY" if args.dry_run else mission.phase,
                       "Sensor-only observation started" if args.dry_run else "Offboard mission started")
        last_state = None
        last_print = 0
        blocked_since = None
        missing_since = None
        previous_command = (0.0, 0.0)
        previous_command_at = started
        chu_ky_truoc_at = started
        trang_thai_ne_truoc = None
        so_lan_doi_ben = 0
        while time.monotonic() - started < args.duration:
            now = time.monotonic()
            elapsed = now-started
            chu_ky_dieu_khien_ms = max(0.0, (now-chu_ky_truoc_at)*1000.0)
            p = nav["position"]
            velocity = nav["velocity"]
            motion = Motion()
            outbound_forward_m = home_distance_m = altitude_m = math.nan
            directive = MissionDirective("OBSERVE_ONLY")
            if drone:
                validate_flight_state(nav, start_position, now)
                simulator_guard.check(nav["yaw"], now)
                if now-nav["position_at"] > 0.75 or now-nav["attitude_at"] > 0.75 or any(t.done() for t in background):
                    raise RuntimeError("Mat telemetry; ha canh")
                outbound_motion = tinh_chuyen_dong_hanh_trinh(
                    p, velocity, nav["yaw"], start_position, course_yaw)
                outbound_forward_m = outbound_motion.forward_m
                home_distance_m = math.hypot(p.north_m-start_position.north_m,
                                             p.east_m-start_position.east_m)
                # Local NED origin is not guaranteed to have down=0. Home is
                # sampled on the ground, so use the relative displacement.
                altitude_m = start_position.down_m-p.down_m
                directive = mission.update(now, outbound_forward_m, home_distance_m,
                                           math.hypot(velocity.north_m_s, velocity.east_m_s),
                                           altitude_m)
                if directive.transition_reason:
                    recorder.event(elapsed, directive.phase, directive.transition_reason)
                    print(f"{directive.phase}: {directive.transition_reason}", flush=True)
                if directive.switch_to_return_frame:
                    navigation_yaw = dao_huong_hanh_trinh(course_yaw)
                    # The vehicle is stationary before this reset, so the new
                    # reverse-frame PID cannot introduce a command discontinuity.
                    planner = Planner(config)
                    previous_command = (0.0, 0.0)
                return_frame = directive.phase in (
                    "RETURN_HOME", "HOME_APPROACH", "HOME_SETTLE", "LANDING", "FAILED")
                motion = tinh_chuyen_dong_hanh_trinh(p, velocity, nav["yaw"], start_position,
                                       navigation_yaw if return_frame else course_yaw)
                if elapsed > 3 and (nav["mode"] is None or nav["mode"].name != "OFFBOARD"):
                    raise RuntimeError("PX4 da thoat Offboard; ket thuc bai bay")

            scan, lidar_sim_time_s = lidar.snapshot()

            # Chỉ LiDAR 360° và PID được phép tạo lệnh bay. YOLO chạy ở tiến
            # trình quan sát riêng, không được import hoặc đọc tại đây.
            d = planner.update(scan, now, motion)
            risk = temporal_risk.update(scan, now, motion)
            lidar_age_s = now-scan.received_at if scan is not None else math.nan
            safety = safety_supervisor.evaluate(
                lidar_age_s=lidar_age_s, planner_state=d.state,
                telemetry_valid=(not drone or (nav["position"] is not None and nav["yaw"] is not None)),
                offboard_active=(not drone or (nav["mode"] is not None and nav["mode"].name == "OFFBOARD")),
                roll_deg=nav["roll"] if drone else 0.0,
                pitch_deg=nav["pitch"] if drone else 0.0,
            )

            command_forward, command_right = d.forward_m_s, d.right_m_s
            profile_fallback_active = False
            if directive.hold_position or safety.hold_position or risk.hold_recommended:
                command_forward = command_right = 0.0
            else:
                command_forward, command_right = gioi_han_van_toc_ngang(
                    command_forward, command_right, directive.speed_limit_m_s)
                # The temporal tracker can only cap the already-safe planner
                # vector; it has no route, yaw, or actuator authority.
                command_forward, command_right = gioi_han_van_toc_ngang(
                    command_forward, command_right, risk.speed_cap_m_s)
                tuoi_scan_ms = ((now-scan.received_at)*1000.0 if scan is not None else math.nan)
                if ho_so is not None:
                    gioi_han_ho_so, profile_fallback_active = tinh_gioi_han_toc_do_ho_so(
                        ho_so, tuoi_scan_ms)
                    command_forward, command_right = gioi_han_van_toc_ngang(
                        command_forward, command_right, gioi_han_ho_so)
            # Routine phase changes decelerate through the same vector acceleration
            # limit as avoidance. Sensor/emergency states stop immediately.
            if not directive.failed and d.state not in ("WAIT_SCAN", "BRAKE", "BLOCKED"):
                command_forward, command_right = gioi_han_gia_toc_ngang(
                    *previous_command, command_forward, command_right,
                    config.max_accel, min(0.2, max(0.001, now-previous_command_at)))
            previous_command = command_forward, command_right
            previous_command_at = now
            vn = ve = vd = 0.0
            if drone:
                vn, ve = doi_van_toc_sang_ned(command_forward, command_right, navigation_yaw)
                vd = tinh_van_toc_giu_do_cao(
                    altitude_m, args.altitude, getattr(args, "altitude_kp", 0.6),
                    getattr(args, "max_vertical_speed", 0.5),
                )
                await asyncio.wait_for(drone.offboard.set_velocity_ned(
                    VelocityNedYaw(vn, ve, vd, course_yaw)), 0.5)

            lidar_age_ms = ((now-scan.received_at)*1000.0 if scan is not None else math.nan)
            home = start_position
            recorder.write({
                "elapsed_s": round(elapsed, 3), "monotonic_elapsed_s": round(elapsed, 3),
                "wall_time_utc": utc_now(), "run_id": run_id,
                "world": getattr(args, "world", "unknown"),
                "event_type": "TRANSITION" if directive.transition_reason else "SAMPLE",
                "phase": directive.phase, "transition_reason": directive.transition_reason,
                "state": d.state, "front_m": d.front_m, "nearest_m": d.nearest_m,
                "forward_m_s": command_forward, "right_m_s": command_right,
                "yaw_deg_s": d.yaw_deg_s, "north_m": p.north_m if p else "",
                "east_m": p.east_m if p else "", "down_m": p.down_m if p else "",
                "reason": d.reason, "trigger_m": d.trigger_m, "stop_m": d.stop_m,
                "target_lateral_m": d.target_lateral_m,
                "command_lateral_m": d.command_lateral_m, "lateral_m": motion.lateral_m,
                "actual_forward_m_s": motion.forward_m_s,
                "actual_right_m_s": motion.right_m_s,
                "yaw_deg": nav["yaw"] if drone else "", "course_yaw_deg": course_yaw,
                "navigation_yaw_deg": navigation_yaw,
                "outbound_forward_m": outbound_forward_m, "home_distance_m": home_distance_m,
                "home_north_m": home.north_m if home else "",
                "home_east_m": home.east_m if home else "",
                "home_down_m": home.down_m if home else "", "altitude_m": altitude_m,
                "vertical_speed_m_s": velocity.down_m_s if velocity else "",
                "requested_north_m_s": vn, "requested_east_m_s": ve,
                "requested_down_m_s": vd,
                "measured_north_m_s": velocity.north_m_s if velocity else "",
                "measured_east_m_s": velocity.east_m_s if velocity else "",
                "pid_error_m": d.pid_error_m, "pid_p_m_s": d.pid_p_m_s,
                "pid_i_m_s": d.pid_i_m_s, "pid_d_m_s": d.pid_d_m_s,
                **(simulator_guard.snapshot() if simulator_guard else {}),
                "roll_deg": nav["roll"] if drone else "",
                "pitch_deg": nav["pitch"] if drone else "",
                "lidar_sim_time_s": lidar_sim_time_s if lidar_sim_time_s is not None else "",
                "lidar_age_ms": lidar_age_ms,
                "speed_profile": ho_so.ten if ho_so else "custom",
                "profile_simulation_only": ho_so.chi_duoc_mo_phong if ho_so else False,
                "profile_fallback_active": profile_fallback_active,
                "control_cycle_ms": chu_ky_dieu_khien_ms,
                "side_switch_count": so_lan_doi_ben,
                "brake_or_stop_state": d.state in ("WAIT_SCAN", "BLOCKED", "BRAKE"),
                "safety_severity": safety.severity.value, "safety_code": safety.code,
                "safety_reason": safety.reason,
                "risk_front_m": risk.front_distance_m if risk.front_distance_m is not None else "",
                "risk_closing_speed_m_s": risk.closing_speed_m_s if risk.closing_speed_m_s is not None else "",
                "risk_ttc_s": risk.time_to_collision_s if risk.time_to_collision_s is not None else "",
                "risk_uncertainty_margin_m": risk.uncertainty_margin_m,
                "risk_speed_cap_m_s": risk.speed_cap_m_s if risk.speed_cap_m_s is not None else "",
                "risk_hold_recommended": risk.hold_recommended,
                "risk_track_confidence": risk.track_confidence,
            })
            if (trang_thai_ne_truoc is not None
                    and d.state in ("SLIDE_LEFT", "SLIDE_RIGHT")
                    and trang_thai_ne_truoc in ("SLIDE_LEFT", "SLIDE_RIGHT")
                    and d.state != trang_thai_ne_truoc):
                so_lan_doi_ben += 1
            trang_thai_ne_truoc = d.state
            chu_ky_truoc_at = now
            if d.state != last_state or now-last_print >= 2 or directive.transition_reason:
                print(f"{directive.phase:16} {d.state:11} | truoc {d.front_m:5.2f} m | "
                      f"Home {home_distance_m:5.2f} m | tien {command_forward:.2f}, "
                      f"ngang {command_right:+.2f} m/s | {d.reason}", flush=True)
                last_print, last_state = now, d.state
            if directive.request_land:
                result_status = "FAILED" if directive.failed else "READY_TO_LAND"
                result_reason = directive.transition_reason or mission.failure_reason
                break
            missing_since = (missing_since or now) if d.state == "WAIT_SCAN" else None
            blocked_since = (blocked_since or now) if d.state in ("BLOCKED", "BRAKE") else None
            if missing_since and now-missing_since >= 3:
                directive = mission.fail(now, "LiDAR unavailable for 3 seconds")
                recorder.event(elapsed, directive.phase, directive.transition_reason)
                result_status, result_reason = "FAILED", directive.transition_reason
                break
            if blocked_since and now-blocked_since >= 8:
                directive = mission.fail(now, "No safe corridor for 8 seconds")
                recorder.event(elapsed, directive.phase, directive.transition_reason)
                result_status, result_reason = "FAILED", directive.transition_reason
                break
            await asyncio.sleep(0.1)
        else:
            if args.dry_run:
                result_status, result_reason = "OBSERVATION_COMPLETE", "Requested observation duration elapsed"
            elif return_home:
                directive = mission.fail(time.monotonic(), "Mission duration elapsed before stable Home arrival")
                recorder.event(time.monotonic()-started, directive.phase, directive.transition_reason)
                result_status, result_reason = "FAILED", directive.transition_reason
            else:
                directive = mission.begin_landing(time.monotonic(), "Requested flight duration elapsed")
                recorder.event(time.monotonic()-started, directive.phase, directive.transition_reason)
                result_status, result_reason = "READY_TO_LAND", directive.transition_reason
    except asyncio.CancelledError:
        result_status, result_reason = "CANCELLED", "Operator requested stop"
        if drone:
            directive = mission.begin_landing(time.monotonic(), result_reason)
            recorder.event(time.monotonic()-started, directive.phase, directive.transition_reason)
        print("Da nhan lenh dung.", flush=True)
    except Exception as exc:
        result_status, result_reason = "FAILED", str(exc) or type(exc).__name__
        directive = mission.fail(time.monotonic(), result_reason)
        recorder.event(time.monotonic()-started, directive.phase, directive.transition_reason)
        raise
    finally:
        stopping = True
        try:
            if drone and armed:
                if offboard:
                    try:
                        await asyncio.wait_for(drone.offboard.set_velocity_ned(VelocityNedYaw(0, 0, 0, course_yaw)), 3)
                        await asyncio.sleep(1)
                        await asyncio.wait_for(drone.offboard.stop(), 3)
                    except Exception as exc:
                        print(f"Offboard stop: {exc}; van gui land.", flush=True)
                print("Dang ha canh...", flush=True)
                await asyncio.wait_for(drone.action.land(), 10)
                await wait_for(drone.telemetry.armed(), lambda a: not a, 90)
                landing_confirmed = True
                disarmed_confirmed = True
                if result_status == "READY_TO_LAND":
                    directive = mission.complete(time.monotonic())
                    recorder.event(time.monotonic()-started, directive.phase,
                                   directive.transition_reason)
                    result_status, result_reason = "COMPLETE", directive.transition_reason
                print("Da ha canh va tat dong co.", flush=True)
        except Exception as exc:
            result_status = "FAILED"
            result_reason += f"; Landing/disarm not confirmed: {str(exc) or type(exc).__name__}"
            mission.fail(time.monotonic(), result_reason)
            raise
        finally:
            for t in background:
                t.cancel()
            await asyncio.gather(*background, return_exceptions=True)
            # MAVSDK 3.17's non-daemon logging thread otherwise holds Python
            # open on errors. Stop only the server owned by this System.
            try:
                if drone is not None:
                    process = getattr(drone, "_server_process", None)
                    drone._stop_mavsdk_server()
                    if process is not None:
                        await asyncio.to_thread(process.wait, timeout=5)
            finally:
                try:
                    lidar.close()
                    if khoa_dieu_khien is not None:
                        khoa_dieu_khien.close()
                    if simulator_guard is not None:
                        simulator_guard.close()
                    for sig in (signal.SIGINT, signal.SIGTERM):
                        loop.remove_signal_handler(sig)
                finally:
                    p = nav["position"]
                    velocity = nav["velocity"]
                    home = start_position
                    final_home_distance = (math.hypot(p.north_m-home.north_m,
                                                      p.east_m-home.east_m)
                                           if p is not None and home is not None else "")
                    final_phase = ("OBSERVE_ONLY" if args.dry_run else mission.phase)
                    recorder.write({
                        "elapsed_s": round(time.monotonic()-started, 3),
                        "monotonic_elapsed_s": round(time.monotonic()-started, 3),
                        "wall_time_utc": utc_now(), "run_id": run_id,
                        "world": getattr(args, "world", "unknown"), "event_type": "FINAL",
                        "phase": final_phase, "state": "FINAL", "reason": result_reason,
                        "result": result_status, "result_reason": result_reason,
                        "landing_confirmed": landing_confirmed,
                        "disarmed_confirmed": disarmed_confirmed,
                        "north_m": p.north_m if p else "", "east_m": p.east_m if p else "",
                        "down_m": p.down_m if p else "", "yaw_deg": nav["yaw"] if drone else "",
                        "course_yaw_deg": course_yaw, "navigation_yaw_deg": navigation_yaw,
                        "home_distance_m": final_home_distance,
                        "home_north_m": home.north_m if home else "",
                        "home_east_m": home.east_m if home else "",
                        "home_down_m": home.down_m if home else "",
                        "altitude_m": home.down_m-p.down_m if p and home else "",
                        "measured_north_m_s": velocity.north_m_s if velocity else "",
                        "measured_east_m_s": velocity.east_m_s if velocity else "",
                        "vertical_speed_m_s": velocity.down_m_s if velocity else "",
                    })
                    recorder.finalize(
                        result_status, result_reason,
                        ({"north_m": home.north_m, "east_m": home.east_m,
                          "down_m": home.down_m, "course_yaw_deg": course_yaw}
                         if home else None),
                        landing_confirmed, disarmed_confirmed,
                    )
                    print(f"Tom tat: {recorder.summary_path.resolve()}", flush=True)


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--topic", default="/drone/lidar/scan")
    p.add_argument("--altitude", type=float, default=3)
    p.add_argument("--speed", type=float, default=2.5, help="horizontal cruise limit (m/s), up to 3 in SITL")
    p.add_argument("--detect-distance", type=float, default=10, help="minimum obstacle lookahead (m)")
    p.add_argument("--kp", type=float, default=0.8, help="lateral position P gain (1/s)")
    p.add_argument("--ki", type=float, default=0.03, help="lateral integral gain (1/s^2)")
    p.add_argument("--kd", type=float, default=0.4, help="damping on measured lateral velocity")
    p.add_argument("--approach-margin", type=float, default=1.6,
                   help="how much faster than strictly necessary the offset is eased in (1..4)")
    p.add_argument("--tracking-fraction", type=float, default=0.65,
                   help="assumed tracking rate when budgeting forward speed (0.2..1)")
    p.add_argument("--clear-fov-deg", type=float, default=170,
                   help="forward view cone that must be empty before resuming course (40..180)")
    p.add_argument("--duration", type=float, default=60)
    p.add_argument("--max-radius", type=float, default=25)
    p.add_argument("--return-home", action="store_true", help="Return to the takeoff position with LiDAR/PID still active")
    p.add_argument("--return-at", type=float, default=42, help="outbound course distance that starts RETURN_HOME (m)")
    p.add_argument("--return-arrival-radius", type=float, default=1.5,
                   help="begin the final stationary Home check inside this radius (m)")
    p.add_argument("--home-approach-radius", type=float, default=5.0,
                   help="distance from Home where the return command starts slowing (m)")
    p.add_argument("--home-approach-speed", type=float, default=0.75,
                   help="maximum horizontal speed during final Home approach (m/s)")
    p.add_argument("--home-settle-speed", type=float, default=0.15,
                   help="maximum measured speed accepted as stationary at Home (m/s)")
    p.add_argument("--home-settle-seconds", type=float, default=1.0,
                   help="continuous position-and-speed confirmation before landing (s)")
    p.add_argument("--home-settle-timeout", type=float, default=10.0)
    p.add_argument("--turnaround-speed", type=float, default=0.15,
                   help="measured speed required before activating the reverse frame (m/s)")
    p.add_argument("--turnaround-settle-seconds", type=float, default=0.5)
    p.add_argument("--turnaround-timeout", type=float, default=12.0)
    p.add_argument("--return-timeout", type=float, default=180.0)
    p.add_argument("--min-flight-altitude", type=float, default=1.5)
    p.add_argument("--max-flight-altitude", type=float, default=4.5)
    p.add_argument("--altitude-kp", type=float, default=0.6,
                   help="outer altitude-hold proportional gain (1/s)")
    p.add_argument("--max-vertical-speed", type=float, default=0.5,
                   help="altitude-hold vertical speed limit (m/s)")
    p.add_argument("--world", choices=("unknown", "obstacle_yolo", "slalom_yolo", "challenge_yolo"), default="unknown",
                   help="world name recorded as run evidence")
    p.add_argument("--launch-pad-radius", type=float, default=2.0,
                   help="course worlds only: refuse to arm farther than this from the takeoff pad")
    p.add_argument("--expected-course-yaw", type=float,
                   help="required PX4 NED heading; slalom_yolo defaults to 90 deg")
    p.add_argument("--course-yaw-tolerance", type=float, default=20.0)
    p.add_argument("--dry-run", action="store_true", help="Read LiDAR and decisions only; no MAVLink commands")
    p.add_argument("--log", help="CSV output path")
    p.add_argument("--hud", action="store_true", help="Show real-time HUD display")
    # Corridor and lane-locking parameters
    p.add_argument("--corridor-half-width", type=float, default=10.0,
                   help="hard lateral bound for safe corridor (m), arena limit (0.5..25)")
    p.add_argument("--side-switch-hysteresis", type=float, default=0.5,
                   help="minimum lateral improvement (m) to justify a side switch (0.2..2)")
    p.add_argument("--side-switch-min-interval", type=float, default=1.2,
                   help="minimum time between side switches (s) (0.5..3)")
    p.add_argument("--speed-reduction-start-frac", type=float, default=0.9,
                   help="fraction of detect_distance where forward speed reduction begins (0.3..1)")
    p.add_argument("--min-forward-speed", type=float, default=1.2,
                   help="minimum forward speed when obstacle is close (m/s) (0.5..speed)")
    p.add_argument("--max-lateral-speed", type=float, default=1.8,
                   help="maximum lateral speed for avoidance (m/s) (0.5..2)")
    p.add_argument("--brake-accel", type=float, default=1.5,
                   help="braking deceleration for speed envelope (m/s^2) (0.5..2)")
    p.add_argument("--max-accel", type=float, default=1.0,
                   help="giới hạn gia tốc vector lệnh (m/s²) (0.5..2)")
    p.add_argument("--profile", choices=("conservative", "baseline", "fast_sitl"),
                   help="profile tốc độ tùy chọn; fast_sitl chỉ để so sánh trong PX4 SITL/Gazebo")
    a = p.parse_args()
    if a.profile:
        ap_dung_ho_so_toc_do(a, a.profile)
    if not all(math.isfinite(v) for v in (a.altitude, a.speed, a.duration, a.max_radius, a.detect_distance,
                                          a.kp, a.ki, a.kd, a.approach_margin, a.tracking_fraction,
                                          a.clear_fov_deg, a.return_at, a.return_arrival_radius,
                                          a.home_approach_radius, a.home_approach_speed,
                                          a.home_settle_speed, a.home_settle_seconds,
                                          a.home_settle_timeout, a.turnaround_speed,
                                          a.turnaround_settle_seconds, a.turnaround_timeout,
                                          a.return_timeout, a.min_flight_altitude,
                                          a.max_flight_altitude, a.altitude_kp,
                                          a.max_vertical_speed, a.course_yaw_tolerance,
                                          a.launch_pad_radius,
                                          a.corridor_half_width, a.side_switch_hysteresis,
                                          a.side_switch_min_interval, a.speed_reduction_start_frac,
                                          a.min_forward_speed, a.max_lateral_speed, a.max_accel, a.brake_accel)
                + (() if a.expected_course_yaw is None else (a.expected_course_yaw,))):
        p.error("Values must be finite")
    if not (2 <= a.altitude <= 4 and 0 < a.speed <= 3 and 0 < a.duration <= 600 and 5 <= a.max_radius <= 50):
        p.error("altitude 2..4, speed (0,3], duration (0,600], max-radius 5..50")
    if not (0 < a.altitude_kp <= 2 and 0 < a.max_vertical_speed <= 1):
        p.error("altitude-kp must be in (0,2], max-vertical-speed in (0,1]")
    if not 0 < a.course_yaw_tolerance <= 90:
        p.error("course-yaw-tolerance must be in (0,90]")
    if not 0.5 <= a.launch_pad_radius <= 5:
        p.error("launch-pad-radius must be in [0.5,5]")
    if not 0.5 <= a.corridor_half_width <= 25:
        p.error("corridor-half-width must be in [0.5,25]")
    if not 0.2 <= a.side_switch_hysteresis <= 2:
        p.error("side-switch-hysteresis must be in [0.2,2]")
    if not 0.5 <= a.side_switch_min_interval <= 3:
        p.error("side-switch-min-interval must be in [0.5,3]")
    if not 0.3 <= a.speed_reduction_start_frac <= 1:
        p.error("speed-reduction-start-frac must be in [0.3,1]")
    if not 0.5 <= a.min_forward_speed <= a.speed:
        p.error("min-forward-speed must be in [0.5, speed]")
    if not 0.5 <= a.max_lateral_speed <= 2:
        p.error("max-lateral-speed must be in [0.5,2]")
    if not 0.5 <= a.max_accel <= 2:
        p.error("max-accel must be in [0.5,2]")
    if not 0.5 <= a.brake_accel <= 2:
        p.error("brake-accel must be in [0.5,2]")
    if a.world in {"slalom_yolo", "challenge_yolo"}:
        if a.expected_course_yaw is None:
            a.expected_course_yaw = 90.0
        if a.corridor_half_width == 10.0:
            # Default safe corridor for slalom arena gates is ±4.5m from center line
            a.corridor_half_width = 4.5
    try:
        AvoidanceConfig(speed=a.speed, detect_distance=a.detect_distance, kp=a.kp, ki=a.ki, kd=a.kd,
                        approach_margin=a.approach_margin, tracking_fraction=a.tracking_fraction,
                        clear_fov_deg=a.clear_fov_deg,
                        corridor_half_width=a.corridor_half_width,
                        side_switch_hysteresis_m=a.side_switch_hysteresis,
                        side_switch_min_interval=a.side_switch_min_interval,
                        speed_reduction_start_frac=a.speed_reduction_start_frac,
                        min_forward_speed=a.min_forward_speed,
                        max_lateral_speed=a.max_lateral_speed,
                        max_accel=a.max_accel,
                        brake_accel=a.brake_accel)
        MissionConfig(
            return_enabled=a.return_home, return_at_m=a.return_at,
            arrival_radius_m=a.return_arrival_radius,
            approach_radius_m=a.home_approach_radius,
            turnaround_speed_m_s=a.turnaround_speed,
            turnaround_settle_s=a.turnaround_settle_seconds,
            turnaround_timeout_s=a.turnaround_timeout,
            home_settle_speed_m_s=a.home_settle_speed,
            home_settle_s=a.home_settle_seconds,
            home_settle_timeout_s=a.home_settle_timeout,
            return_timeout_s=a.return_timeout, max_radius_m=a.max_radius,
            min_altitude_m=a.min_flight_altitude,
            max_altitude_m=a.max_flight_altitude,
            approach_speed_m_s=a.home_approach_speed,
        )
    except ValueError as exc:
        p.error(str(exc))
    if not a.min_flight_altitude < a.altitude < a.max_flight_altitude:
        p.error("altitude must be strictly inside min-flight-altitude..max-flight-altitude")
    return a


if __name__ == "__main__":
    args = parse_args()
    # Shared by run.sh fly/avoid to prevent two pilots fighting for MAVLink.
    try:
        asyncio.run(run(args))
    except Exception as exc:
        raise SystemExit(f"Dung: {exc}")
