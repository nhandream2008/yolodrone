#!/usr/bin/env python3
"""CLI thống nhất cho mô phỏng, tránh vật cản LiDAR, kiểm thử và theo dõi log."""

import argparse
import asyncio
import os
os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")
import sys
import signal
import time
from pathlib import Path

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from avoid_fly import run as run_avoid


class DroneHUD:
    """Terminal-based Heads-Up Display for real-time flight monitoring."""
    
    def __init__(self, width=80):
        self.width = width
        self.data = {}
        self.enabled = True
        self._clear_screen()
    
    def _clear_screen(self):
        if os.name == 'nt':
            os.system('cls')
        else:
            os.system('clear')
    
    def update(self, key, value):
        self.data[key] = value
    
    def render(self):
        if not self.enabled:
            return
        
        # Move cursor to top-left
        print('\033[H', end='')
        
        # Title bar
        print('╔' + '═' * (self.width - 2) + '╗')
        title = " AI DRONE UNIFIED CONTROLLER - REAL-TIME HUD "
        padding = (self.width - 2 - len(title)) // 2
        print('║' + ' ' * padding + title + ' ' * (self.width - 2 - padding - len(title)) + '║')
        print('╠' + '═' * (self.width - 2) + '╣')
        
        # Flight data
        self._render_section("FLIGHT STATE", [
            ("Phase", self.data.get('phase', 'N/A')),
            ("Planner State", self.data.get('planner_state', 'N/A')),
            ("Mode", self.data.get('mode', 'AVOID')),
        ])
        
        self._render_section("POSITION & ATTITUDE", [
            ("Yaw", f"{self.data.get('yaw', 0):.1f}°"),
            ("Course Yaw", f"{self.data.get('course_yaw', 0):.1f}°"),
            ("Navigation Yaw", f"{self.data.get('nav_yaw', 0):.1f}°"),
            ("Altitude", f"{self.data.get('altitude', 0):.2f} m"),
            ("Home Distance", f"{self.data.get('home_dist', 0):.2f} m"),
            ("Outbound Dist", f"{self.data.get('outbound_dist', 0):.2f} m"),
        ])
        
        self._render_section("VELOCITY", [
            ("Ground Speed", f"{self.data.get('ground_speed', 0):.2f} m/s"),
            ("Forward Cmd", f"{self.data.get('cmd_forward', 0):.2f} m/s"),
            ("Right Cmd", f"{self.data.get('cmd_right', 0):.2f} m/s"),
            ("Vertical Speed", f"{self.data.get('vz', 0):.2f} m/s"),
        ])
        
        self._render_section("OBSTACLE AVOIDANCE", [
            ("Nearest Obstacle", f"{self.data.get('nearest', 0):.2f} m"),
            ("Front Clearance", f"{self.data.get('front', 0):.2f} m"),
            ("Trigger Distance", f"{self.data.get('trigger', 0):.2f} m"),
            ("Lateral Pos", f"{self.data.get('lateral', 0):.2f} m"),
            ("Target Lateral", f"{self.data.get('target_lat', 0):.2f} m"),
        ])
        
        self._render_section("PID CONTROL", [
            ("Error", f"{self.data.get('pid_error', 0):.3f} m"),
            ("P Term", f"{self.data.get('pid_p', 0):.3f} m/s"),
            ("I Term", f"{self.data.get('pid_i', 0):.3f} m/s"),
            ("D Term", f"{self.data.get('pid_d', 0):.3f} m/s"),
        ])
        
        self._render_section("SENSORS", [
            ("LiDAR Age", f"{self.data.get('lidar_age', 0):.0f} ms"),
            ("LiDAR Sim Time", f"{self.data.get('lidar_sim', 0):.3f} s"),
            ("Roll/Pitch", f"{self.data.get('roll', 0):.1f}° / {self.data.get('pitch', 0):.1f}°"),
        ])
        
        # Bottom bar
        elapsed = self.data.get('elapsed', 0)
        print('╠' + '═' * (self.width - 2) + '╣')
        print(f'║ Elapsed: {elapsed:.1f}s  |  Log: {self.data.get("log", "N/A"):<40} ║')
        print('╚' + '═' * (self.width - 2) + '╝')
        
        sys.stdout.flush()
    
    def _render_section(self, title, items):
        print(f'║ {title:<{self.width - 4}} ║')
        for label, value in items:
            line = f'  {label:<18}: {value}'
            print(f'║ {line:<{self.width - 4}} ║')


class UnifiedDroneCLI:
    """Main CLI entry point."""
    
    def __init__(self):
        self.hud = DroneHUD()
        self.running = False
    
    def run(self):
        parser = argparse.ArgumentParser(
            description="Bộ điều khiển mô phỏng drone PX4 SITL/Gazebo",
            formatter_class=argparse.RawDescriptionHelpFormatter,
            epilog="""
Ví dụ:
   # Chạy tránh vật cản bằng LiDAR trong SITL/Gazebo.
   python unified_drone.py avoid --duration 120 --world slalom_yolo --hud

   # Chạy kiểm thử đơn vị.
   python unified_drone.py test

   # Chỉ chạy mô phỏng, không điều khiển bay.
   python unified_drone.py sim --world slalom_yolo

YOLO là tiến trình quan sát độc lập qua yolo_gz_node.py; CLI này không đọc
phát hiện YOLO và không cho YOLO tạo lệnh bay.
"""
        )
        
        subparsers = parser.add_subparsers(dest='command', help='Lệnh cần chạy')
        
        # Avoid command
        avoid_parser = subparsers.add_parser('avoid', help='Bay tránh vật cản chỉ bằng LiDAR + PID')
        self._them_tham_so_tranh_vat_can(avoid_parser)
        
        # Sim command
        sim_parser = subparsers.add_parser('sim', help='Chỉ khởi động mô phỏng')
        sim_parser.add_argument('--world', choices=['slalom_yolo', 'challenge_yolo', 'obstacle_yolo'],
                                default='slalom_yolo')
        
        # Test command
        test_parser = subparsers.add_parser('test', help='Chạy unit test')
        test_parser.add_argument('--verbose', '-v', action='store_true')
        test_parser.add_argument('--pattern', help='Test pattern to match')
        
        # Monitor command
        monitor_parser = subparsers.add_parser('monitor', help='Theo dõi log bay')
        monitor_parser.add_argument('--log', required=True, help='Đường dẫn log cần theo dõi')
        
        args = parser.parse_args()
        
        if not args.command:
            parser.print_help()
            return 1
        
        try:
            if args.command == 'avoid':
                return self._run_avoid(args)
            elif args.command == 'sim':
                return self._run_sim(args)
            elif args.command == 'test':
                return self._run_test(args)
            elif args.command == 'monitor':
                return self._run_monitor(args)
        except KeyboardInterrupt:
            print("\n[ĐÃ DỪNG] Đang kết thúc an toàn...")
            return 130
        except Exception as e:
            print(f"\n[LỖI] {e}")
            import traceback
            traceback.print_exc()
            return 1
        
        return 0
    
    def _them_tham_so_tranh_vat_can(self, p):
        p.add_argument('--duration', type=float, default=60)
        p.add_argument('--altitude', type=float, default=3)
        p.add_argument('--speed', type=float, default=2.5)
        p.add_argument('--max-radius', type=float, default=25)
        p.add_argument('--world', choices=['unknown', 'obstacle_yolo', 'slalom_yolo', 'challenge_yolo'],
                       default='slalom_yolo')
        p.add_argument('--return-home', action='store_true')
        p.add_argument('--return-at', type=float, default=42)
        p.add_argument('--hud', action='store_true', help='Hiển thị HUD thời gian thực')
        p.add_argument('--dry-run', action='store_true')
        p.add_argument('--log', help='Log file path')
        # Tham số PID/LiDAR. Không có bất kỳ tham số YOLO nào tại đây.
        p.add_argument('--kp', type=float, default=0.8)
        p.add_argument('--ki', type=float, default=0.03)
        p.add_argument('--kd', type=float, default=0.4)
        p.add_argument('--detect-distance', type=float, default=10)
        p.add_argument('--profile', choices=['conservative', 'baseline', 'fast_sitl'],
                       help='Profile tốc độ tùy chọn; fast_sitl chỉ để so sánh trong PX4 SITL/Gazebo')
    
    def _run_avoid(self, args):
        # Convert to avoid_fly args format
        avoid_args = self._tao_tham_so_tranh_vat_can(args)
        
        # Enable HUD if requested
        if args.hud:
            self.hud.enabled = True
            # We'll need to hook into the run loop for HUD updates
            # For now, run normally
            pass
        
        try:
            asyncio.run(run_avoid(avoid_args))
        except KeyboardInterrupt:
            pass
        return 0
    
    def _run_sim(self, args):
        """Start simulation via sim_avoid.py"""
        from scripts.sim_avoid import run as run_sim
        return run_sim("~/PX4-Autopilot", args.world)
    
    def _run_test(self, args):
        import unittest
        loader = unittest.TestLoader()
        if args.pattern:
            suite = loader.discover('tests', pattern=args.pattern)
        else:
            suite = loader.discover('tests')
        runner = unittest.TextTestRunner(verbosity=2 if args.verbose else 1)
        result = runner.run(suite)
        return 0 if result.wasSuccessful() else 1
    
    def _run_monitor(self, args):
        from scripts.monitor_flight import main as monitor_main
        import sys
        sys.argv = ['monitor_flight.py', '--log', args.log]
        monitor_main()
        return 0
    
    def _tao_tham_so_tranh_vat_can(self, args):
        """Tạo namespace tương thích với parser của avoid_fly.py."""
        class Args:
            pass
        a = Args()
        for key, value in vars(args).items():
            setattr(a, key.replace('-', '_'), value)
        # Set defaults for avoid_fly specific args
        defaults = {
            'topic': '/drone/lidar/scan',
            'approach_margin': 1.6,
            'tracking_fraction': 0.65,
            'clear_fov_deg': 170,
            'return_arrival_radius': 1.5,
            'home_approach_radius': 5.0,
            'home_approach_speed': 0.75,
            'home_settle_speed': 0.15,
            'home_settle_seconds': 1.0,
            'home_settle_timeout': 10.0,
            'turnaround_speed': 0.15,
            'turnaround_settle_seconds': 0.5,
            'turnaround_timeout': 12.0,
            'return_timeout': 180.0,
            'min_flight_altitude': 1.5,
            'max_flight_altitude': 4.5,
            'altitude_kp': 0.6,
            'max_vertical_speed': 0.5,
            'launch_pad_radius': 2.0,
            'expected_course_yaw': None,
            'course_yaw_tolerance': 20.0,
            'corridor_half_width': 10.0,
            'side_switch_hysteresis': 0.5,
            'side_switch_min_interval': 1.2,
            'speed_reduction_start_frac': 0.9,
            'min_forward_speed': 1.2,
            'max_lateral_speed': 1.8,
            'brake_accel': 1.5,
            'max_accel': 1.0,
            'profile': None,
        }
        for key, value in defaults.items():
            if not hasattr(a, key):
                setattr(a, key, value)
        if a.world in {'slalom_yolo', 'challenge_yolo'}:
            if a.expected_course_yaw is None:
                a.expected_course_yaw = 90.0
            if a.corridor_half_width == 10.0:
                a.corridor_half_width = 4.5
        return a

    def _build_avoid_args(self, args):
        """Alias tương thích tạm thời; dùng `_tao_tham_so_tranh_vat_can` trong mã mới."""
        return self._tao_tham_so_tranh_vat_can(args)


def main():
    cli = UnifiedDroneCLI()
    return cli.run()


if __name__ == '__main__':
    sys.exit(main())
