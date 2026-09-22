"""Runner safety regressions with no Gazebo, MAVSDK server or aircraft."""

import asyncio
from contextlib import ExitStack, redirect_stdout
import csv
import io
import math
from pathlib import Path
import sys
import tempfile
import threading
import time
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import avoid_fly
from obstacle_avoidance import Scan


class Clock:
    def __init__(self):
        self.now = 100.0

    def monotonic(self):
        return self.now

    async def sleep(self, seconds):
        self.now += seconds
        await asyncio.sleep(0)


class CommandSlewTests(unittest.TestCase):
    def test_vector_acceleration_is_limited_without_changing_target_direction(self):
        forward, right = avoid_fly.slew_horizontal(1.0, 0.0, 0.0, 1.0, 0.5, 0.2)
        self.assertAlmostEqual(math.hypot(forward-1.0, right), 0.1)
        self.assertGreater(forward, 0.0)
        self.assertGreater(right, 0.0)

    def test_small_command_change_passes_through(self):
        self.assertEqual(avoid_fly.slew_horizontal(0.0, 0.0, 0.02, -0.01, 1.0, 0.1),
                         (0.02, -0.01))

    def test_altitude_hold_uses_ned_down_sign_and_limit(self):
        self.assertAlmostEqual(avoid_fly.tinh_van_toc_giu_do_cao(3.5, 3.0, 0.6, 0.5), 0.3)
        self.assertAlmostEqual(avoid_fly.tinh_van_toc_giu_do_cao(2.5, 3.0, 0.6, 0.5), -0.3)
        self.assertEqual(avoid_fly.tinh_van_toc_giu_do_cao(8.0, 3.0, 0.6, 0.5), 0.5)
        self.assertIs(avoid_fly.altitude_hold_down_velocity, avoid_fly.tinh_van_toc_giu_do_cao)


class LidarTimestampTests(unittest.TestCase):
    def tao_lidar(self):
        lidar = avoid_fly.Lidar.__new__(avoid_fly.Lidar)
        lidar.lock = threading.Lock()
        lidar.scan = None
        lidar.error = ""
        lidar.stamp = None
        lidar.sim_time_s = None
        return lidar

    def tao_thong_diep(self, sec=1, nsec=0):
        return SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nsec=nsec)),
            count=361, vertical_count=1, angle_min=-math.pi, angle_max=math.pi,
            angle_step=math.pi/180, range_min=0.1, range_max=20.0,
            ranges=[20.0] * 361,
        )

    def test_malformed_timestamp_invalidates_lidar(self):
        lidar = self.tao_lidar()
        lidar.receive(self.tao_thong_diep(sec=-1))
        self.assertIsNone(lidar.latest())
        self.assertIn("Timestamp LiDAR", lidar.error)

        lidar.receive(self.tao_thong_diep(nsec=1_000_000_000))
        self.assertIsNone(lidar.latest())
        self.assertIn("Timestamp LiDAR", lidar.error)

    def test_repeated_timestamp_does_not_refresh_scan(self):
        lidar = self.tao_lidar()
        lidar.receive(self.tao_thong_diep(sec=3))
        scan_dau = lidar.latest()
        lidar.receive(self.tao_thong_diep(sec=3))
        self.assertIs(lidar.latest(), scan_dau)
        self.assertEqual(lidar.stamp, (3, 0))

    def test_adapter_gazebo_chuyen_inf_no_hit_thanh_range_max_huu_han(self):
        lidar = self.tao_lidar()
        thong_diep = self.tao_thong_diep(sec=4)
        thong_diep.ranges = [math.inf] * thong_diep.count
        lidar.receive(thong_diep)
        scan = lidar.latest()
        self.assertIsNotNone(scan)
        self.assertTrue(all(math.isfinite(value) for value in scan.ranges))
        self.assertTrue(all(value == 20.0 for value in scan.ranges))


class FakeLidar:
    def __init__(self, clock, events):
        self.clock = clock
        self.events = events
        self.available = True
        self.error = "No sensor measurements"

    def latest(self):
        if not self.available:
            return None
        return Scan(
            tuple(math.radians(angle) for angle in range(-180, 181)),
            (12.0,) * 361,
            self.clock.monotonic(),
        )

    def snapshot(self):
        return self.latest(), self.clock.monotonic()

    def close(self):
        self.events.append("lidar-closed")


class FakeTelemetry:
    def __init__(self, vehicle):
        self.vehicle = vehicle

    async def health(self):
        # LAND is position-ready but not armable until the runner requests HOLD.
        yield SimpleNamespace(
            is_local_position_ok=True,
            is_global_position_ok=True,
            is_home_position_ok=True,
            is_armable=self.vehicle.holding,
        )
        await asyncio.Event().wait()

    async def armed(self):
        self.vehicle.events.append("armed-observed" if self.vehicle.is_armed else "disarmed-observed")
        yield self.vehicle.is_armed
        if self.vehicle.landing:
            await asyncio.sleep(0)
            self.vehicle.is_armed = False
            self.vehicle.events.append("disarmed-observed")
            yield False
        await asyncio.Event().wait()

    async def position_velocity_ned(self):
        try:
            while True:
                if self.vehicle.frozen == "position":
                    await asyncio.Event().wait()
                airborne = "takeoff" in self.vehicle.events
                yield SimpleNamespace(
                    position=SimpleNamespace(north_m=self.vehicle.takeoff_drift if airborne else 0.0, east_m=0.0,
                                             down_m=-3.0 if airborne else 0.0),
                    velocity=SimpleNamespace(north_m_s=0.0, east_m_s=0.0, down_m_s=0.0),
                )
                await asyncio.sleep(0)
        finally:
            self.vehicle.events.append("positions-closed")

    async def attitude_euler(self):
        try:
            while True:
                if self.vehicle.frozen == "attitude":
                    await asyncio.Event().wait()
                yield SimpleNamespace(yaw_deg=self.vehicle.yaw_deg, roll_deg=0.0, pitch_deg=0.0)
                await asyncio.sleep(0)
        finally:
            self.vehicle.events.append("attitudes-closed")

    async def set_rate_position_velocity_ned(self, rate):
        self.vehicle.events.append(("position-rate", rate))

    async def set_rate_attitude_euler(self, rate):
        self.vehicle.events.append(("attitude-rate", rate))

    async def flight_mode(self):
        try:
            yield SimpleNamespace(name="OFFBOARD")
            await asyncio.Event().wait()
        finally:
            self.vehicle.events.append("modes-closed")

    async def position(self):
        yield SimpleNamespace(relative_altitude_m=3.0)
        await asyncio.Event().wait()


class FakeAction:
    def __init__(self, vehicle):
        self.vehicle = vehicle

    async def hold(self):
        self.vehicle.events.append("hold")
        self.vehicle.holding = True

    async def set_takeoff_altitude(self, altitude):
        self.vehicle.events.append("set-altitude")

    async def arm(self):
        self.vehicle.events.append("arm")
        self.vehicle.is_armed = True
        if self.vehicle.fail_at == "arm":
            # PX4 accepted arm, but the RPC response was lost.
            raise RuntimeError("Lost arm acknowledgement")

    async def takeoff(self):
        self.vehicle.events.append("takeoff")
        if self.vehicle.fail_at == "takeoff":
            raise RuntimeError("Takeoff RPC failed")

    async def land(self):
        self.vehicle.events.append("land")
        if self.vehicle.fail_at == "land":
            raise RuntimeError("Land RPC failed")
        self.vehicle.landing = True


class FakeOffboard:
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.commands = []
        self.ned_commands = []

    async def set_velocity_body(self, command):
        self.commands.append(command)

    async def set_velocity_ned(self, command):
        self.ned_commands.append(command)

    async def start(self):
        self.vehicle.events.append("offboard-start")
        self.vehicle.frozen = self.vehicle.freeze_on_start

    async def stop(self):
        self.vehicle.events.append("offboard-stop")


class FakeVehicle:
    def __init__(self, events):
        self.events = events
        self.sim_enabled = 1
        self.fail_at = None
        self.holding = False
        self.is_armed = False
        self.landing = False
        self.yaw_deg = 90.0
        self.takeoff_drift = 0.0
        self.frozen = None
        self.freeze_on_start = None
        self.action = FakeAction(self)
        self.offboard = FakeOffboard(self)
        self.telemetry = FakeTelemetry(self)
        self.core = SimpleNamespace(connection_state=self.connection_state)
        self.param = SimpleNamespace(get_param_int=self.get_param, set_param_int=self.set_param)
        self._server_process = SimpleNamespace(wait=self.reap_server)

    async def connect(self, system_address):
        self.events.append("connect")

    async def connection_state(self):
        yield SimpleNamespace(is_connected=True)
        await asyncio.Event().wait()

    async def get_param(self, name):
        self.events.append(("get-param", name))
        return self.sim_enabled

    async def set_param(self, name, value):
        self.events.append(("set-param", name, value))

    def _stop_mavsdk_server(self):
        self.events.append("server-stopped")

    def reap_server(self, timeout):
        self.events.append("server-reaped")


class RunnerSafetyTests(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.events = []
        self.clock = Clock()
        self.lidar = FakeLidar(self.clock, self.events)
        self.vehicle = FakeVehicle(self.events)
        self.args = SimpleNamespace(
            speed=1.0,
            topic="/test/lidar",
            dry_run=False,
            log=str(Path(self.folder.name) / "flight.csv"),
            altitude=3.0,
            duration=0.3,
            max_radius=25.0,
        )
        self.system_factory = Mock(return_value=self.vehicle)
        self.simulator_guard = Mock()
        self.simulator_guard.snapshot.return_value = {}

    async def run_runner(self):
        fake_mavsdk = ModuleType("mavsdk")
        fake_mavsdk.System = self.system_factory
        fake_offboard = ModuleType("mavsdk.offboard")
        fake_offboard.VelocityBodyYawspeed = lambda forward, right, down, yaw: SimpleNamespace(
            forward_m_s=forward, right_m_s=right, down_m_s=down, yaw_deg_s=yaw
        )
        fake_offboard.VelocityNedYaw = lambda north, east, down, yaw: SimpleNamespace(
            north_m_s=north, east_m_s=east, down_m_s=down, yaw_deg=yaw
        )
        # Replace only the runner's view of these modules. The unittest event
        # loop retains real time and a real deadline if the runner gets stuck.
        runner_asyncio = SimpleNamespace(**{
            name: getattr(asyncio, name) for name in dir(asyncio) if not name.startswith("__")
        })
        runner_asyncio.sleep = self.clock.sleep
        loop = asyncio.get_running_loop()
        with ExitStack() as stack:
            stack.enter_context(patch.dict(sys.modules, {
                "mavsdk": fake_mavsdk, "mavsdk.offboard": fake_offboard,
            }))
            stack.enter_context(patch.object(avoid_fly, "Lidar", return_value=self.lidar))
            stack.enter_context(patch.object(avoid_fly, "SimulationGuard", return_value=self.simulator_guard))
            stack.enter_context(patch.object(avoid_fly, "time", SimpleNamespace(
                monotonic=self.clock.monotonic, strftime=time.strftime,
            )))
            stack.enter_context(patch.object(avoid_fly, "asyncio", runner_asyncio))
            stack.enter_context(patch.object(loop, "add_signal_handler"))
            stack.enter_context(patch.object(loop, "remove_signal_handler"))
            stack.enter_context(redirect_stdout(io.StringIO()))
            await asyncio.wait_for(avoid_fly.run(self.args), timeout=2.0)

    def assert_server_and_sensor_closed(self):
        self.assertEqual(self.events.count("server-stopped"), 1)
        self.assertEqual(self.events.count("server-reaped"), 1)
        self.assertEqual(self.events.count("lidar-closed"), 1)

    async def test_dry_run_logs_decisions_without_creating_mavsdk(self):
        self.args.dry_run = True
        await self.run_runner()
        self.system_factory.assert_not_called()
        self.assertEqual(self.events, ["lidar-closed"])
        with Path(self.args.log).open(newline="") as stream:
            decisions = list(csv.DictReader(stream))
        self.assertTrue(decisions)
        samples = [row for row in decisions if row["event_type"] != "FINAL"]
        self.assertTrue(all(row["state"] == "CRUISE" for row in samples))
        self.assertEqual(decisions[-1]["result"], "OBSERVATION_COMPLETE")
        self.assertTrue(Path(self.args.log).with_suffix(".json").is_file())

    async def test_missing_sensor_fails_before_connecting_or_arming(self):
        self.lidar.available = False
        with self.assertRaisesRegex(RuntimeError, "Khong co LiDAR hop le"):
            await self.run_runner()
        self.system_factory.assert_not_called()
        self.assertEqual(self.events, ["lidar-closed"])

    async def test_non_gazebo_vehicle_rejected_before_mutating_params_or_arming(self):
        self.vehicle.sim_enabled = 0
        with self.assertRaisesRegex(RuntimeError, "SIM_GZ_EN=1"):
            await self.run_runner()
        self.assertIn(("get-param", "SIM_GZ_EN"), self.events)
        self.assertFalse(any(isinstance(event, tuple) and event[0] == "set-param" for event in self.events))
        self.assertNotIn("arm", self.events)
        self.assertNotIn("land", self.events)
        self.assert_server_and_sensor_closed()

    async def test_previous_land_mode_is_changed_to_hold_before_rearming(self):
        await self.run_runner()
        self.assertLess(self.events.index("disarmed-observed"), self.events.index("hold"))
        self.assertLess(self.events.index("hold"), self.events.index("arm"))
        self.assertIn("takeoff", self.events)
        self.assertIn("offboard-start", self.events)
        self.assertIn("offboard-stop", self.events)
        land_index = self.events.index("land")
        self.assertIn("armed-observed", self.events[land_index:])
        self.assertIn("disarmed-observed", self.events[land_index:])
        self.assertFalse(self.vehicle.is_armed)
        self.assert_server_and_sensor_closed()

    async def test_already_armed_vehicle_is_not_taken_over(self):
        self.vehicle.is_armed = True
        with self.assertRaisesRegex(RuntimeError, "dang armed"):
            await self.run_runner()
        self.assertNotIn("hold", self.events)
        self.assertNotIn("arm", self.events)
        self.assertNotIn("land", self.events)
        self.assert_server_and_sensor_closed()

    async def test_bad_simulator_heading_never_arms(self):
        self.simulator_guard.check.side_effect = RuntimeError("heading disagreement")
        with self.assertRaisesRegex(RuntimeError, "Preflight integrity failed"):
            await self.run_runner()
        self.assertNotIn("arm", self.events)
        self.assertNotIn("land", self.events)
        self.simulator_guard.close.assert_called_once()

    async def test_takeoff_drift_lands_before_offboard_navigation(self):
        self.vehicle.takeoff_drift = 3.0
        with self.assertRaisesRegex(RuntimeError, "drift exceeded"):
            await self.run_runner()
        self.assertIn("takeoff", self.events)
        self.assertNotIn("offboard-start", self.events)
        self.assertIn("land", self.events)
        self.assertIn("disarmed-observed", self.events)

    async def test_velocity_commands_use_initial_heading_in_ned_and_preserve_yaw(self):
        await self.run_runner()
        self.assertIn(("position-rate", 20), self.events)
        self.assertIn(("attitude-rate", 20), self.events)
        commands = self.vehicle.offboard.ned_commands
        self.assertTrue(any(command.east_m_s > 0 for command in commands))
        for command in commands:
            self.assertAlmostEqual(command.north_m_s, 0.0)
            self.assertEqual(command.down_m_s, 0.0)
            self.assertEqual(command.yaw_deg, 90.0)
        # Body commands are only the initial hover, never a rotating pilot loop.
        self.assertTrue(all(command.forward_m_s == command.right_m_s == command.yaw_deg_s == 0.0
                            for command in self.vehicle.offboard.commands))

    async def test_declared_course_yaw_settles_at_zero_translation_before_flight(self):
        self.args.expected_course_yaw = 90.0
        self.args.course_yaw_tolerance = 20.0
        await self.run_runner()
        commands = self.vehicle.offboard.ned_commands
        first_moving = next(index for index, command in enumerate(commands)
                            if math.hypot(command.north_m_s, command.east_m_s) > 0)
        # Alignment must settle before the first translation command.
        self.assertGreaterEqual(first_moving, 5)
        self.assertTrue(all(command.north_m_s == command.east_m_s == 0.0
                            and command.yaw_deg == 90.0
                            for command in commands[:first_moving]))

    async def test_declared_course_is_never_rezeroed_to_estimator_heading(self):
        self.args.expected_course_yaw = 90.0
        self.args.course_yaw_tolerance = 20.0
        self.vehicle.yaw_deg = 96.0
        with self.assertRaisesRegex(RuntimeError, "Could not align course yaw"):
            await self.run_runner()
        commands = self.vehicle.offboard.ned_commands
        self.assertTrue(commands)
        self.assertTrue(all(command.north_m_s == command.east_m_s == 0.0
                            and command.yaw_deg == 90.0 for command in commands))
        self.assertIn("land", self.events)

    async def test_stale_position_or_attitude_lands_and_closes_all_streams(self):
        self.args.duration = 2.0
        for stream in ("position", "attitude"):
            with self.subTest(stream=stream):
                self.events.clear()
                self.vehicle = FakeVehicle(self.events)
                self.vehicle.freeze_on_start = stream
                self.system_factory.return_value = self.vehicle
                with self.assertRaisesRegex(RuntimeError, "Mat telemetry"):
                    await self.run_runner()
                self.assertIn("land", self.events)
                self.assertIn("positions-closed", self.events)
                self.assertIn("attitudes-closed", self.events)
                self.assertIn("modes-closed", self.events)
                self.assertFalse(self.vehicle.is_armed)
                self.assert_server_and_sensor_closed()

    async def test_uncertain_arm_or_takeoff_failure_lands_and_observes_disarm(self):
        for failure in ("arm", "takeoff"):
            with self.subTest(failure=failure):
                self.events.clear()
                self.vehicle = FakeVehicle(self.events)
                self.vehicle.fail_at = failure
                self.system_factory.return_value = self.vehicle
                with self.assertRaises(RuntimeError):
                    await self.run_runner()
                land_index = self.events.index("land")
                self.assertIn("armed-observed", self.events[land_index:])
                self.assertIn("disarmed-observed", self.events[land_index:])
                self.assertFalse(self.vehicle.is_armed)
                self.assertLess(self.events.index("positions-closed"), self.events.index("server-stopped"))
                self.assertLess(self.events.index("modes-closed"), self.events.index("server-stopped"))
                self.assertLess(self.events.index("attitudes-closed"), self.events.index("server-stopped"))
                self.assert_server_and_sensor_closed()

    async def test_land_rpc_failure_still_reaps_owned_server_and_sensor(self):
        self.vehicle.fail_at = "land"
        with self.assertRaisesRegex(RuntimeError, "Land RPC failed"):
            await self.run_runner()
        self.assertIn("land", self.events)
        self.assertIn("positions-closed", self.events)
        self.assertIn("modes-closed", self.events)
        self.assertIn("attitudes-closed", self.events)
        self.assert_server_and_sensor_closed()


class CourseFrameTests(unittest.TestCase):
    def test_heading_error_wraps_at_north(self):
        self.assertAlmostEqual(avoid_fly.tinh_sai_so_huong_do(359.0, 1.0), 2.0)
        self.assertAlmostEqual(avoid_fly.tinh_sai_so_huong_do(80.0, 90.0), 10.0)
        self.assertIs(avoid_fly.heading_error_deg, avoid_fly.tinh_sai_so_huong_do)

    def test_forward_and_right_commands_rotate_from_course_into_ned(self):
        north, east = avoid_fly.doi_van_toc_sang_ned(2.0, 1.0, 90.0)
        self.assertAlmostEqual(north, -1.0)
        self.assertAlmostEqual(east, 2.0)

    def test_measured_velocity_and_position_rotate_back_into_initial_course(self):
        origin = SimpleNamespace(north_m=10.0, east_m=20.0)
        position = SimpleNamespace(north_m=7.0, east_m=25.0)
        velocity = SimpleNamespace(north_m_s=-1.0, east_m_s=2.0)
        motion = avoid_fly.tinh_chuyen_dong_hanh_trinh(position, velocity, 100.0, origin, 90.0)
        self.assertAlmostEqual(motion.forward_m_s, 2.0)
        self.assertAlmostEqual(motion.right_m_s, 1.0)
        self.assertAlmostEqual(motion.lateral_m, 3.0)
        self.assertAlmostEqual(motion.forward_m, 5.0)
        self.assertAlmostEqual(motion.yaw_delta_rad, math.radians(10.0))

    def test_return_course_commands_reverse_translation_but_not_the_yaw_setpoint(self):
        origin = SimpleNamespace(north_m=0.0, east_m=0.0)
        position = SimpleNamespace(north_m=0.0, east_m=42.0)
        velocity = SimpleNamespace(north_m_s=0.0, east_m_s=-1.0)
        return_yaw = avoid_fly.dao_huong_hanh_trinh(90.0)
        motion = avoid_fly.tinh_chuyen_dong_hanh_trinh(position, velocity, 90.0, origin, return_yaw)
        north, east = avoid_fly.doi_van_toc_sang_ned(motion.forward_m_s, motion.right_m_s, return_yaw)
        self.assertEqual(return_yaw, 270.0)
        self.assertAlmostEqual(motion.forward_m_s, 1.0)
        self.assertAlmostEqual(north, 0.0, places=6)
        self.assertAlmostEqual(east, -1.0, places=6)
        # The runner sends 90° as the MAVSDK yaw setpoint even while navigation
        # uses the reverse (270°) course frame.
        self.assertAlmostEqual((90.0-motion.yaw_delta_rad*180/math.pi) % 360, 270.0)

    def test_ten_tieng_viet_va_alias_tuong_thich_cung_hanh_vi(self):
        self.assertIs(avoid_fly.course_motion, avoid_fly.tinh_chuyen_dong_hanh_trinh)
        self.assertIs(avoid_fly.ned_velocity, avoid_fly.doi_van_toc_sang_ned)
        self.assertIs(avoid_fly.reverse_course_yaw, avoid_fly.dao_huong_hanh_trinh)
        self.assertIs(avoid_fly.limit_horizontal, avoid_fly.gioi_han_van_toc_ngang)
        self.assertIs(avoid_fly.slew_horizontal, avoid_fly.gioi_han_gia_toc_ngang)

    def test_yaw_wrap_preserves_small_scan_rotation(self):
        origin = SimpleNamespace(north_m=0.0, east_m=0.0)
        velocity = SimpleNamespace(north_m_s=0.0, east_m_s=0.0)
        motion = avoid_fly.tinh_chuyen_dong_hanh_trinh(origin, velocity, -179.0, origin, 179.0)
        self.assertAlmostEqual(motion.yaw_delta_rad, math.radians(2.0))


class ReturnHomeArgumentTests(unittest.TestCase):
    def test_return_home_requires_a_safe_outbound_marker_inside_radius(self):
        with patch.object(sys, "argv", ["avoid_fly.py", "--return-home", "--return-at", "42",
                                         "--max-radius", "48", "--duration", "180"]):
            args = avoid_fly.parse_args()
        self.assertTrue(args.return_home)
        self.assertEqual(args.return_at, 42.0)
        self.assertEqual(args.return_arrival_radius, 1.5)


if __name__ == "__main__":
    unittest.main()
