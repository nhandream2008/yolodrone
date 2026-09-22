#!/usr/bin/env python3
"""Observation-only detector: annotate images, localise with LiDAR, and log evidence."""
import argparse
from dataclasses import asdict
import json
import math
import os
from pathlib import Path
import threading
import time

os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")
from frame_utils import image_to_bgr
from sensor_fusion import DetectionLog, LidarScan, fuse_box


# PX4's bundled Tools/simulation/gz/models/mono_cam/model.sdf declares
# <horizontal_fov>1.74</horizontal_fov>; convert that verified radians value.
DEFAULT_CAMERA_H_FOV_DEG = math.degrees(1.74)
# PX4 x500_mono_cam places camera_link at body (0.12, +0.03, 0.242) m;
# this project's LiDAR centre is body (0, 0, 0.325) m. In the horizontal
# forward/right plane the camera is therefore (0.12, -0.03) m from the LiDAR.
DEFAULT_CAMERA_FROM_LIDAR_FORWARD_M = 0.12
DEFAULT_CAMERA_FROM_LIDAR_RIGHT_M = -0.03


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/world/baylands_yolo/model/x500_mono_cam_0/link/camera_link/sensor/imager/image")
    parser.add_argument("--lidar-topic", default="/drone/lidar/scan")
    parser.add_argument("--model", default=str(Path(__file__).parent / "models/yolo26n.pt"))
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--device", default="", help="auto (default), cpu, or 0 for CUDA")
    parser.add_argument("--fps", type=float, default=10.0)
    parser.add_argument("--classes", type=int, nargs="+", default=[0, 1, 2, 3, 5, 7])
    parser.add_argument("--camera-hfov-deg", type=float, default=DEFAULT_CAMERA_H_FOV_DEG)
    parser.add_argument("--sync-max-ms", type=float, default=150.0)
    parser.add_argument("--camera-from-lidar-forward-m", type=float,
                        default=DEFAULT_CAMERA_FROM_LIDAR_FORWARD_M)
    parser.add_argument("--camera-from-lidar-right-m", type=float,
                        default=DEFAULT_CAMERA_FROM_LIDAR_RIGHT_M)
    parser.add_argument("--origin-north-m", type=float, default=0.0)
    parser.add_argument("--origin-east-m", type=float, default=0.0)
    parser.add_argument("--course-yaw-deg", type=float, default=0.0)
    parser.add_argument("--source", help="Saved image or video for offline processing")
    parser.add_argument("--no-ros", action="store_true", help="Require file-only mode; implies no middleware imports")
    parser.add_argument("--track", action="store_true", help="Keep object identities while processing a saved video")
    parser.add_argument("--output-dir", default="output")
    parser.add_argument("--logs-dir", default="logs")
    args = parser.parse_args(argv)
    numeric = (args.conf, args.fps, args.camera_hfov_deg, args.sync_max_ms,
               args.origin_north_m, args.origin_east_m, args.course_yaw_deg,
               args.camera_from_lidar_forward_m, args.camera_from_lidar_right_m)
    if not all(math.isfinite(value) for value in numeric):
        parser.error("Numeric arguments must be finite")
    if not 0 < args.conf <= 1 or not 0 < args.fps <= 120:
        parser.error("conf must be in (0, 1], fps must be in (0, 120]")
    if not 0 < args.camera_hfov_deg < 180 or args.sync_max_ms <= 0:
        parser.error("camera-hfov-deg must be in (0, 180); sync-max-ms must be positive")
    if args.no_ros and not args.source:
        parser.error("--no-ros requires --source")
    return args


def _as_list(value):
    if value is None:
        return []
    if hasattr(value, "tolist"):
        value = value.tolist()
    return list(value)


def _name(names, class_index):
    if isinstance(names, dict):
        return str(names.get(class_index, class_index))
    if isinstance(names, (tuple, list)) and 0 <= class_index < len(names):
        return str(names[class_index])
    return str(class_index)


def _message_stamp(message, fallback):
    """Use Gazebo's simulation capture time, with monotonic time only as fallback."""
    stamp = getattr(getattr(message, "header", None), "stamp", None)
    try:
        seconds, nanoseconds = float(stamp.sec), float(stamp.nsec)
    except (AttributeError, TypeError, ValueError):
        return fallback
    value = seconds + nanoseconds/1_000_000_000.0
    return value if math.isfinite(value) and seconds >= 0 and 0 <= nanoseconds < 1_000_000_000 else fallback


def _scan_pose_ned(message):
    """Gazebo ENU pose -> NED north/east and PX4-style yaw, or None if absent."""
    pose = getattr(message, "world_pose", None)
    position = getattr(pose, "position", None)
    orientation = getattr(pose, "orientation", None)
    try:
        east, north = float(position.x), float(position.y)
        x, y, z, w = (float(orientation.x), float(orientation.y),
                      float(orientation.z), float(orientation.w))
    except (AttributeError, TypeError, ValueError):
        return None
    if not all(math.isfinite(value) for value in (north, east, x, y, z, w)):
        return None
    norm = math.sqrt(x*x+y*y+z*z+w*w)
    if norm < 1e-6:
        return None
    x, y, z, w = x/norm, y/norm, z/norm, w/norm
    gazebo_yaw_deg = math.degrees(math.atan2(2.0*(w*z+x*y), 1.0-2.0*(y*y+z*z)))
    # ENU: X east, Y north; NED yaw is clockwise from north.
    return north, east, (90.0-gazebo_yaw_deg) % 360.0


def extract_boxes(result):
    """Return plain tuples and tolerate models that have no tracking IDs."""
    boxes = getattr(result, "boxes", None)
    if boxes is None:
        return []
    coordinates = _as_list(getattr(boxes, "xyxy", None))
    classes = _as_list(getattr(boxes, "cls", None))
    confidences = _as_list(getattr(boxes, "conf", None))
    identifiers = _as_list(getattr(boxes, "id", None)) if getattr(boxes, "id", None) is not None else []
    output = []
    for index, box in enumerate(coordinates):
        if len(box) != 4 or index >= len(classes) or index >= len(confidences):
            continue
        class_index = int(classes[index])
        track_id = int(identifiers[index]) if index < len(identifiers) and identifiers[index] is not None else None
        output.append((tuple(float(value) for value in box), _name(getattr(result, "names", {}), class_index),
                       float(confidences[index]), track_id))
    return output


def _iou(first, second):
    left, top = max(first[0], second[0]), max(first[1], second[1])
    right, bottom = min(first[2], second[2]), min(first[3], second[3])
    overlap = max(0.0, right-left)*max(0.0, bottom-top)
    area_first = max(0.0, first[2]-first[0])*max(0.0, first[3]-first[1])
    area_second = max(0.0, second[2]-second[0])*max(0.0, second[3]-second[1])
    return overlap/(area_first+area_second-overlap) if area_first+area_second-overlap else 0.0


class FallbackTracker:
    """Tiny dependency-free identity fallback when a detector returns id=None."""
    def __init__(self):
        self.next_id = 1
        self.previous = []

    def assign(self, box, class_name):
        candidates = [(score, track_id) for old_box, old_name, track_id in self.previous
                      if old_name == class_name for score in (_iou(box, old_box),)]
        if candidates and max(candidates)[0] >= 0.35:
            return max(candidates)[1]
        track_id, self.next_id = self.next_id, self.next_id+1
        return track_id

    def update(self, boxes):
        self.previous = [(box, class_name, track_id) for box, class_name, _, track_id in boxes]


def fuse_result(result, image_width, args, frame_at, scan, tracker, pose=None):
    raw = extract_boxes(result)
    resolved = []
    for box, class_name, confidence, track_id in raw:
        resolved.append((box, class_name, confidence,
                         track_id if track_id is not None else tracker.assign(box, class_name)))
    tracker.update(resolved)
    pose_valid = pose is not None
    north, east, course_yaw = pose if pose_valid else (args.origin_north_m, args.origin_east_m,
                                                        args.course_yaw_deg)
    return [fuse_box(box, class_name, confidence, image_width, args.camera_hfov_deg, frame_at,
                     scan=scan, track_id=track_id, north_m=north, east_m=east,
                     course_yaw_deg=course_yaw, max_sync_seconds=args.sync_max_ms/1000.0,
                     pose_valid=pose_valid,
                     camera_forward_from_lidar_m=args.camera_from_lidar_forward_m,
                     camera_right_from_lidar_m=args.camera_from_lidar_right_m)
            for box, class_name, confidence, track_id in resolved]


def _log_path(folder):
    stamp = time.strftime("%Y%m%d-%H%M%S")
    return Path(folder) / f"detections-{stamp}.csv"


def _predict(model, frame, args, persist=False):
    options = dict(conf=args.conf, classes=args.classes, device=args.device or None, verbose=False)
    return (model.track(frame, persist=persist, **options) if args.track else model.predict(frame, **options))[0]


def annotate(result, detections):
    """Add observation metadata only; this draws no control recommendation."""
    import cv2
    image = result.plot()
    for (box, _, _, _), detection in zip(extract_boxes(result), detections):
        x1, y1, _, _ = (int(value) for value in box)
        identity = f" id={detection.track_id}" if detection.track_id is not None else ""
        distance = f" {detection.distance_m:.2f}m" if detection.valid_range else " range=n/a"
        label = f"{detection.class_name} {detection.confidence:.2f}{identity}{distance}"
        cv2.putText(image, label, (max(0, x1), max(16, y1-5)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, (20, 20, 20), 2, cv2.LINE_AA)
        cv2.putText(image, label, (max(0, x1), max(16, y1-5)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, (255, 255, 255), 1, cv2.LINE_AA)
    return image


def run_offline(args):
    """File-only mode for evidence generation; it never initialises middleware."""
    import cv2
    from ultralytics import YOLO

    source = Path(args.source)
    if not source.is_file():
        raise FileNotFoundError(f"Source does not exist: {source}")
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    model, tracker = YOLO(args.model), FallbackTracker()
    logger = DetectionLog(_log_path(args.logs_dir))
    try:
        image = cv2.imread(str(source))
        if image is not None:
            result = _predict(model, image, args)
            detections = fuse_result(result, image.shape[1], args, 0.0, None, tracker)
            for detection in detections:
                logger.write(detection, 0.0)
            output = output_dir / f"annotated-{source.stem}.png"
            if not cv2.imwrite(str(output), annotate(result, detections)):
                raise RuntimeError(f"Could not write {output}")
            print(f"Saved {output}; detections: {len(detections)}; log: {logger.path}")
            return output
        capture = cv2.VideoCapture(str(source))
        if not capture.isOpened():
            raise ValueError(f"Unsupported image or video: {source}")
        output = output_dir / f"annotated-{source.stem}.mp4"
        rate = capture.get(cv2.CAP_PROP_FPS) or args.fps
        writer = None
        index = 0
        try:
            while True:
                ok, frame = capture.read()
                if not ok:
                    break
                result = _predict(model, frame, args, persist=True)
                detections = fuse_result(result, frame.shape[1], args, index/rate, None, tracker)
                for detection in detections:
                    logger.write(detection, index/rate)
                annotated = annotate(result, detections)
                if writer is None:
                    writer = cv2.VideoWriter(str(output), cv2.VideoWriter_fourcc(*"mp4v"), rate,
                                             (annotated.shape[1], annotated.shape[0]))
                writer.write(annotated)
                index += 1
        finally:
            capture.release()
            if writer is not None:
                writer.release()
        if index == 0:
            raise ValueError("Video contains no decodable frames")
        print(f"Saved {output}; frames: {index}; log: {logger.path}")
        return output
    finally:
        logger.close()


def _scan_from_message(message, received_at):
    ranges = tuple(float(value) for value in message.ranges)
    angle_min = math.degrees(float(message.angle_min))
    angle_step = math.degrees(float(message.angle_step))
    # Gazebo laser angles increase vehicle-left; image pixels increase camera-right.
    return LidarScan(tuple(-(angle_min+index*angle_step) for index in range(len(ranges))), ranges,
                     received_at, float(message.range_max))


def run_live(args):
    """Live observation mode publishes only the two documented vision topics."""
    import gz.transport13 as transport
    from gz.msgs10.image_pb2 import Image as GzImage
    from gz.msgs10.laserscan_pb2 import LaserScan
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    from ultralytics import YOLO

    class Detector(Node):
        def __init__(self):
            super().__init__("yolo_observation")
            self.model = YOLO(args.model)
            self.image_pub = self.create_publisher(Image, "/yolo/image_annotated", 1)
            self.detection_pub = self.create_publisher(String, "/yolo/detections", 1)
            self.lock = threading.Lock()
            self.latest_image = self.latest_scan = None
            self.tracker = FallbackTracker()
            self.logger = DetectionLog(_log_path(args.logs_dir))
            self.started_at = time.monotonic()
            self.create_timer(1.0/args.fps, self.process)

        def receive_image(self, message):
            with self.lock:
                # Header time is the exposure/capture time, not inference end time.
                self.latest_image = message, _message_stamp(message, time.monotonic())

        def receive_scan(self, message):
            try:
                snapshot = _scan_from_message(message, _message_stamp(message, time.monotonic()))
            except (AttributeError, TypeError, ValueError):
                return
            with self.lock:
                self.latest_scan = snapshot, _scan_pose_ned(message)

        def process(self):
            with self.lock:
                image_snapshot, self.latest_image = self.latest_image, None
                scan_snapshot = self.latest_scan
            if image_snapshot is None:
                return
            message, frame_at = image_snapshot
            scan, pose = scan_snapshot if scan_snapshot is not None else (None, None)
            try:
                frame = image_to_bgr(message)
            except ValueError as error:
                self.get_logger().error(str(error))
                return
            result = _predict(self.model, frame, args, persist=True)
            detections = fuse_result(result, frame.shape[1], args, frame_at, scan, self.tracker, pose)
            annotated = annotate(result, detections)
            image = Image()
            image.height, image.width = annotated.shape[:2]
            image.encoding, image.is_bigendian, image.step = "bgr8", 0, image.width*3
            image.data = annotated.tobytes()
            image.header.frame_id = "camera_link"
            self.image_pub.publish(image)
            self.detection_pub.publish(String(data=json.dumps([asdict(item) for item in detections])))
            elapsed = time.monotonic()-self.started_at
            if detections:
                det_names = [f"{d.class_name}({d.confidence:.2f})" for d in detections]
                print(f"[{elapsed:6.1f}s] Phat hien {len(detections)} muc tieu: {', '.join(det_names)}", flush=True)
            for detection in detections:
                self.logger.write(detection, elapsed)

        def close_log(self):
            self.logger.close()

    rclpy.init()
    node, transport_node = None, transport.Node()
    image_subscribed = scan_subscribed = False
    try:
        node = Detector()
        image_subscribed = transport_node.subscribe(GzImage, args.topic, node.receive_image)
        scan_subscribed = transport_node.subscribe(LaserScan, args.lidar_topic, node.receive_scan)
        if not image_subscribed or not scan_subscribed:
            raise RuntimeError("Could not subscribe to required observation topics")
        print(f"[YOLO Node] Dang lang nghe camera tai: {args.topic}")
        print(f"[YOLO Node] Dang chay model: {args.model} tren GPU...")
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if image_subscribed:
            transport_node.unsubscribe(args.topic)
        if scan_subscribed:
            transport_node.unsubscribe(args.lidar_topic)
        if node is not None:
            node.close_log()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv=None):
    args = parse_args(argv)
    if args.source:
        return run_offline(args)
    return run_live(args)


if __name__ == "__main__":
    main()
