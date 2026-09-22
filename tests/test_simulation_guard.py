import math
from pathlib import Path
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from simulation_guard import SimulationGuard
from scripts.sim_avoid import prepare_session
from avoid_fly import validate_flight_state


class SimulationGuardTests(unittest.TestCase):
    def setUp(self):
        self.guard = SimulationGuard.__new__(SimulationGuard)
        self.guard.lock = threading.Lock()
        self.guard.stamp = None
        self.guard.pose = (100.0, 0.0, 0.0, 90.0)

    def test_heading_disagreement_and_stale_data_fail_closed(self):
        self.guard.check(96.0, 100.1, on_pad=True)
        with self.assertRaisesRegex(RuntimeError, "heading disagreement"):
            self.guard.check(118.0, 100.1)
        with self.assertRaisesRegex(RuntimeError, "stale"):
            self.guard.check(90.0, 101.0)
        with self.assertRaisesRegex(RuntimeError, "invalid"):
            self.guard.check(math.nan, 100.1)

    def test_pad_uses_true_world_position_not_estimator_origin(self):
        self.guard.pose = (100.0, 18.0, 0.0, 90.0)
        with self.assertRaisesRegex(RuntimeError, "launch pad"):
            self.guard.check(90.0, 100.1, on_pad=True)
        self.guard.check(90.0, 100.1)

    def test_enu_orientation_converts_to_ned_heading(self):
        pose = SimpleNamespace(name="x500_mono_lidar_0",
            position=SimpleNamespace(x=1, y=2),
            orientation=SimpleNamespace(w=math.sqrt(.5), z=math.sqrt(.5), x=0, y=0))
        with patch("simulation_guard.time.monotonic", return_value=100.0):
            message = SimpleNamespace(pose=[pose], header=SimpleNamespace(
                stamp=SimpleNamespace(sec=1, nsec=0)))
            self.guard.receive(message)
        self.guard.check(0.0, 100.1)
        with patch("simulation_guard.time.monotonic", return_value=102.0):
            self.guard.receive(message)
        with self.assertRaisesRegex(RuntimeError, "stale"):
            self.guard.check(0.0, 102.0)

    def test_takeoff_drift_and_tilt_abort(self):
        origin = SimpleNamespace(north_m=0, east_m=0)
        nav = dict(position=SimpleNamespace(north_m=2.1, east_m=0, down_m=-1),
                   velocity=SimpleNamespace(north_m_s=0, east_m_s=0, down_m_s=0),
                   yaw=90, roll=0, pitch=0, position_at=100, attitude_at=100)
        with self.assertRaisesRegex(RuntimeError, "drift"):
            validate_flight_state(nav, origin, 100.1, startup=True)
        nav["position"].north_m = 0
        nav["roll"] = 36
        with self.assertRaisesRegex(RuntimeError, "tilted"):
            validate_flight_state(nav, origin, 100.1)

    def test_fresh_sessions_preserve_stock_and_isolate_parameters(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            build = root / "px4/build/px4_sitl_default"
            (build / "rootfs").mkdir(parents=True)
            (build / "etc/init.d-posix").mkdir(parents=True)
            (build / "rootfs/gz_env.sh").write_text("resources")
            stock = build / "etc/init.d-posix/rcS"
            stock.write_text("sensors start\n\tekf2 start &\n")
            (build / "rootfs/parameters.bson").write_bytes(b"old calibration")
            with patch("scripts.sim_avoid.Path.home", return_value=root), patch.dict(
                    "os.environ", {"PX4_PARAM_EKF2_GPS_CHECK": "0"}):
                a, command, environment = prepare_session(root / "px4", "slalom_yolo")
                b, _, _ = prepare_session(root / "px4", "slalom_yolo")
            self.assertNotEqual(a, b)
            self.assertNotIn("PX4_PARAM_EKF2_GPS_CHECK", environment)
            self.assertIn("-s", command)
            self.assertIn("sleep 3\n\tekf2 start", (a / "rcS").read_text())
            self.assertNotIn("sleep", stock.read_text())
            self.assertFalse((a / "parameters.bson").exists())
            self.assertEqual((build / "rootfs/parameters.bson").read_bytes(), b"old calibration")
