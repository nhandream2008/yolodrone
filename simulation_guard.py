"""Read-only Gazebo truth check. It may abort a flight, never steer the PID."""
import math
import re
import subprocess
import threading
import time


class SimulationGuard:
    def __init__(self, world):
        import gz.transport13 as transport
        from gz.msgs10.pose_v_pb2 import Pose_V
        if world == "unknown":
            topics = subprocess.run(["gz", "topic", "-l"], capture_output=True,
                                    text=True, check=True, timeout=5).stdout
            worlds = re.findall(r"^/world/([^/]+)/dynamic_pose/info$", topics, re.MULTILINE)
            if len(worlds) != 1:
                raise RuntimeError("Cannot identify exactly one Gazebo world; specify --world")
            world = worlds[0]
        self.lock = threading.Lock()
        self.world = world
        self.stamp = None
        self.pose = None
        self.topic = f"/world/{world}/dynamic_pose/info"
        self.node = transport.Node()
        if not self.node.subscribe(Pose_V, self.topic, self.receive):
            raise RuntimeError(f"Cannot observe simulator truth: {self.topic}")

    def receive(self, message):
        stamp = (message.header.stamp.sec, message.header.stamp.nsec)
        for pose in message.pose:
            if pose.name == "x500_mono_lidar_0":
                q = pose.orientation
                yaw_enu = math.degrees(math.atan2(2*(q.w*q.z+q.x*q.y),
                                                 1-2*(q.y*q.y+q.z*q.z)))
                with self.lock:
                    if self.stamp is not None and stamp <= self.stamp:
                        return
                    self.stamp = stamp
                    self.pose = (time.monotonic(), pose.position.x, pose.position.y,
                                 (90-yaw_enu) % 360)
                return

    def check(self, measured_yaw, now, *, on_pad=False, pad_radius=2.0):
        with self.lock:
            pose = self.pose
        if pose is None or now-pose[0] > 0.75:
            raise RuntimeError("Simulator truth missing/stale; refusing flight")
        _, x, y, yaw = pose
        if not all(math.isfinite(v) for v in (x, y, yaw, measured_yaw)):
            raise RuntimeError("Simulator heading/position is invalid")
        error = abs((measured_yaw-yaw+180) % 360-180)
        if error > 12.0:
            raise RuntimeError(f"EKF/Gazebo heading disagreement {error:.1f} deg > 12 deg")
        if on_pad and math.hypot(x, y) > pad_radius:
            raise RuntimeError(f"Gazebo drone is outside the {pad_radius:g} m launch pad; restart simulation")

    def close(self):
        self.node.unsubscribe(self.topic)

    def snapshot(self):
        with self.lock:
            pose = self.pose
        if pose is None:
            return {}
        return dict(truth_world_x_m=pose[1], truth_world_y_m=pose[2], truth_yaw_deg=pose[3])
