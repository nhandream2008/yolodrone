"""Chạy mô phỏng tránh vật cản và chỉ dọn process group PX4/Gazebo của phiên đó."""

import argparse
import os
from pathlib import Path
import signal
import shutil
import subprocess
import sys
import tempfile
import time


def gui_tin_hieu_nhom(ma_nhom, tin_hieu):
    """Gửi tín hiệu chỉ đến process group của phiên mô phỏng được sở hữu."""
    try:
        os.killpg(ma_nhom, tin_hieu)
        return True
    except ProcessLookupError:
        return False


def cho_nhom_tien_trinh(tien_trinh, thoi_gian_cho):
    """Chờ process group của phiên kết thúc, không dò process toàn hệ thống."""
    han_cuoi = time.monotonic()+thoi_gian_cho
    while True:
        tien_trinh.poll()
        if not gui_tin_hieu_nhom(tien_trinh.pid, 0):
            return True
        if time.monotonic() >= han_cuoi:
            return False
        time.sleep(0.1)


def don_dep_phien(tien_trinh, tin_hieu_dau):
    """Tiến trình con kế thừa group PX4; không dùng tìm kiếm tên process toàn cục."""
    for tin_hieu, thoi_gian_cho in ((tin_hieu_dau, 8.0), (signal.SIGTERM, 4.0),
                                    (signal.SIGKILL, 2.0)):
        if not gui_tin_hieu_nhom(tien_trinh.pid, tin_hieu):
            tien_trinh.poll()
            return True
        if cho_nhom_tien_trinh(tien_trinh, thoi_gian_cho):
            return True
    return False


def chuan_bi_phien(goc_px4, world):
    """Tạo phiên SITL mới với tham số stock, giữ nguyên toàn bộ run cũ."""
    session_root = Path.home() / ".local/state/ai-drone/sitl"
    session_root.mkdir(parents=True, exist_ok=True)
    session = Path(tempfile.mkdtemp(prefix=f"{world}-", dir=session_root))
    # PX4 discovers its generated Gazebo resource/plugin paths relative to -w.
    shutil.copy2(goc_px4 / "build/px4_sitl_default/rootfs/gz_env.sh", session / "gz_env.sh")
    # The bridge's first timestamp / IMU samples occur while the spawned model
    # settles. Do not initialize EKF on that transient. Modify only this session's
    # startup copy, never PX4's source, calibration or arming thresholds.
    startup = (goc_px4 / "build/px4_sitl_default/etc/init.d-posix/rcS").read_text()
    marker = "\tekf2 start &"
    if startup.count(marker) != 1:
        raise RuntimeError("Unsupported PX4 startup: expected exactly one EKF2 start")
    startup_path = session / "rcS"
    startup_path.write_text(startup.replace(marker, "\tsleep 3\n"+marker))
    environment = os.environ.copy()
    # Inherited calibration / parameter overrides defeat a reproducible SITL run.
    for key in list(environment):
        if key.startswith("PX4_PARAM_") or key in (
                "PX4_GZ_MODEL_NAME", "PX4_GZ_STANDALONE", "PX4_GZ_MODEL_POSE"):
            environment.pop(key)
    environment.update(PX4_SYS_AUTOSTART="4001", PX4_SIM_MODEL="gz_x500_mono_lidar",
                       PX4_GZ_WORLD=world, PX4_GZ_MODEL_POSE="0,0,0")
    command = [str(goc_px4 / "build/px4_sitl_default/bin/px4"), "-d",
               "-w", str(session), "-s", str(startup_path),
               str(goc_px4 / "build/px4_sitl_default/etc")]
    return session, command, environment


# Wrapper tương thích cho test/caller cũ; mã mới dùng tên tiếng Việt.
def signal_group(group_id, sig):
    return gui_tin_hieu_nhom(group_id, sig)


def wait_for_group(process, timeout):
    return cho_nhom_tien_trinh(process, timeout)


def cleanup(process, first_signal):
    return don_dep_phien(process, first_signal)


def prepare_session(px4_root, world):
    return chuan_bi_phien(px4_root, world)


def run(px4_dir, world="obstacle_yolo"):
    if os.name != "posix":
        print("Hay chay lenh nay trong Ubuntu/WSL qua Drone.ps1 sim-avoid.", file=sys.stderr)
        return 2
    px4_root = Path(px4_dir).expanduser().resolve()
    executable = px4_root / "build/px4_sitl_default/bin/px4"
    if not executable.is_file() or not os.access(executable, os.X_OK):
        print(f"Khong tim thay PX4 SITL da build: {executable}", file=sys.stderr)
        return 2

    session, command, environment = chuan_bi_phien(px4_root, world)
    print(f"PX4 session moi (tham so mac dinh, log duoc giu): {session}", flush=True)
    stop_signal = None
    def request_stop(sig, _frame):
        nonlocal stop_signal
        # Repeated Ctrl+C does not interrupt cleanup or raise a second exception.
        if stop_signal is None:
            stop_signal = sig

    previous_handlers = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)}
    process = None
    cleaned = True
    try:
        for sig in previous_handlers:
            signal.signal(sig, request_stop)
        process = subprocess.Popen(
            command,
            cwd=px4_root,
            env=environment,
            start_new_session=True,
        )
        print("Mo phong tranh vat can dang chay voi camera Gazebo tu do. Ctrl+C de dung PX4 va Gazebo.", flush=True)
        while process.poll() is None and stop_signal is None:
            time.sleep(0.1)
    except OSError as error:
        print(f"Khong khoi dong duoc PX4: {error}", file=sys.stderr)
        return 2
    finally:
        if process is not None:
            print("Dang dong PX4 va Gazebo cua phien nay...", flush=True)
            cleaned = don_dep_phien(process, stop_signal or signal.SIGINT)
        for sig, handler in previous_handlers.items():
            signal.signal(sig, handler)

    if not cleaned:
        print("Da gui SIGKILL; nhom tien trinh chua ket thuc trong thoi gian cho.", file=sys.stderr)
        return 1
    print("Da dong phien mo phong.", flush=True)
    if stop_signal is not None:
        return 128 + stop_signal
    code = process.returncode
    if code:
        print(f"PX4 da thoat voi ma {code}.", file=sys.stderr)
    return 128 - code if code is not None and code < 0 else (code or 0)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--px4-dir", default="~/PX4-Autopilot")
    parser.add_argument("--world", choices=("obstacle_yolo", "slalom_yolo", "challenge_yolo"), default="obstacle_yolo")
    arguments = parser.parse_args()
    sys.exit(run(arguments.px4_dir, arguments.world))
