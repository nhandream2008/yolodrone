"""Small, dependency-free camera/LiDAR geometry for the observation branch."""

from dataclasses import asdict, dataclass
import csv
import math
from pathlib import Path
import statistics


LOG_COLUMNS = (
    "elapsed_s", "track_id", "class_name", "confidence", "u_center_px", "v_center_px",
    "box_width_px", "bearing_deg", "lidar_bearing_deg", "distance_m", "sync_offset_ms", "north_m", "east_m",
    "valid_range",
)


@dataclass(frozen=True)
class LidarScan:
    """Planar scan in camera/body bearing degrees; positive is camera-right."""
    angles_deg: tuple
    ranges_m: tuple
    received_at: float
    range_max_m: float = 20.0


@dataclass(frozen=True)
class FusedDetection:
    track_id: int | None
    class_name: str
    confidence: float
    u_center_px: float
    v_center_px: float
    box_width_px: float
    bearing_deg: float
    lidar_bearing_deg: float | None
    distance_m: float | None
    sync_offset_ms: float | None
    north_m: float | None
    east_m: float | None
    valid_range: bool

    def row(self, elapsed_s):
        values = asdict(self)
        values["elapsed_s"] = elapsed_s
        return {column: values[column] for column in LOG_COLUMNS}


def _finite(value):
    return isinstance(value, (int, float)) and math.isfinite(value)


def _normalise_deg(angle):
    return (angle + 180.0) % 360.0 - 180.0


def focal_length_px(image_width, horizontal_fov_deg):
    if not (_finite(image_width) and _finite(horizontal_fov_deg)
            and image_width > 0 and 0 < horizontal_fov_deg < 180):
        raise ValueError("Image width and horizontal FOV must be finite and valid")
    return (image_width/2.0) / math.tan(math.radians(horizontal_fov_deg)/2.0)


def pixel_to_bearing_deg(u_center_px, image_width, horizontal_fov_deg):
    """Pinhole bearing: image centre is zero, right-hand pixels are positive."""
    if not _finite(u_center_px):
        raise ValueError("Pixel centre must be finite")
    return math.degrees(math.atan((u_center_px-image_width/2.0)
                                  / focal_length_px(image_width, horizontal_fov_deg)))


def lidar_window_deg(box_width_px, image_width, horizontal_fov_deg,
                     minimum_deg=1.0, maximum_deg=12.0):
    """Use a wider angular window for a visually wider object."""
    if not (_finite(box_width_px) and box_width_px >= 0):
        raise ValueError("Box width must be finite and non-negative")
    fraction = box_width_px/max(float(image_width), 1.0)
    return min(maximum_deg, max(minimum_deg, fraction*horizontal_fov_deg*0.75))


def associated_range(scan, bearing_deg, box_width_px, image_width, horizontal_fov_deg,
                     frame_at, max_sync_seconds=0.15,
                     camera_forward_from_lidar_m=0.12, camera_right_from_lidar_m=-0.03):
    """Associate camera ray to LiDAR points with the measured horizontal extrinsic."""
    if scan is None or not all(_finite(value) for value in (bearing_deg, frame_at, max_sync_seconds)):
        return None, None, None
    offset = frame_at-scan.received_at
    if abs(offset) > max_sync_seconds:
        return None, offset*1000.0, None
    half_window = lidar_window_deg(box_width_px, image_width, horizontal_fov_deg)/2.0
    usable = []
    for angle, distance in zip(scan.angles_deg, scan.ranges_m):
        if not (_finite(angle) and _finite(distance)
                and 0 < distance < scan.range_max_m-0.01):
            continue
        angle_rad = math.radians(angle)
        point_forward = distance*math.cos(angle_rad)
        point_right = distance*math.sin(angle_rad)
        camera_bearing = math.degrees(math.atan2(
            point_right-camera_right_from_lidar_m,
            point_forward-camera_forward_from_lidar_m,
        ))
        if abs(_normalise_deg(camera_bearing-bearing_deg)) <= half_window:
            usable.append((distance, angle))
    if not usable:
        return None, offset*1000.0, None
    median = statistics.median(distance for distance, _angle in usable)
    representative = min(usable, key=lambda item: abs(item[0]-median))
    return median, offset*1000.0, representative[1]


def median_range(scan, bearing_deg, box_width_px, image_width, horizontal_fov_deg,
                 frame_at, max_sync_seconds=0.15,
                 camera_forward_from_lidar_m=0.12, camera_right_from_lidar_m=-0.03):
    """Compatibility wrapper returning robust LiDAR range and timestamp offset."""
    distance, offset, _ = associated_range(
        scan, bearing_deg, box_width_px, image_width, horizontal_fov_deg,
        frame_at, max_sync_seconds, camera_forward_from_lidar_m,
        camera_right_from_lidar_m,
    )
    return distance, offset


def bearing_distance_to_world(distance_m, bearing_deg, north_m=0.0, east_m=0.0,
                              course_yaw_deg=0.0):
    """Project a camera-relative observation into an ENU-style north/east map."""
    if not all(_finite(value) for value in (distance_m, bearing_deg, north_m, east_m, course_yaw_deg)):
        return None, None
    heading = math.radians(course_yaw_deg+bearing_deg)
    return north_m+distance_m*math.cos(heading), east_m+distance_m*math.sin(heading)


def fuse_box(box, class_name, confidence, image_width, horizontal_fov_deg, frame_at,
             scan=None, track_id=None, north_m=0.0, east_m=0.0, course_yaw_deg=0.0,
             max_sync_seconds=0.15, pose_valid=True,
             camera_forward_from_lidar_m=0.12, camera_right_from_lidar_m=-0.03):
    """Associate one 2-D rectangle with its local LiDAR window, if fresh."""
    if len(box) != 4 or not all(_finite(value) for value in box):
        raise ValueError("Box must contain four finite coordinates")
    x1, y1, x2, y2 = box
    u_center, v_center = (x1+x2)/2.0, (y1+y2)/2.0
    width = abs(x2-x1)
    bearing = pixel_to_bearing_deg(u_center, image_width, horizontal_fov_deg)
    distance, offset_ms, lidar_bearing = associated_range(
        scan, bearing, width, image_width, horizontal_fov_deg, frame_at, max_sync_seconds,
        camera_forward_from_lidar_m, camera_right_from_lidar_m,
    )
    # A range cannot be represented as a trustworthy map observation without
    # the pose recorded at the associated scan.  Keep the raw LiDAR stream
    # separate rather than fabricating a world position from a static default.
    if not pose_valid:
        distance = None
        lidar_bearing = None
    north, east = bearing_distance_to_world(distance, lidar_bearing, north_m, east_m,
                                             course_yaw_deg)
    return FusedDetection(
        track_id=track_id, class_name=str(class_name), confidence=float(confidence),
        u_center_px=u_center, v_center_px=v_center, box_width_px=width,
        bearing_deg=bearing, lidar_bearing_deg=lidar_bearing, distance_m=distance,
        sync_offset_ms=offset_ms, north_m=north, east_m=east,
        valid_range=distance is not None,
    )


class DetectionLog:
    """Append-only evidence log. It has no connection to a flight process."""
    def __init__(self, path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.stream = self.path.open("w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.stream, fieldnames=LOG_COLUMNS)
        self.writer.writeheader()

    def write(self, detection, elapsed_s):
        self.writer.writerow(detection.row(elapsed_s))
        self.stream.flush()

    def close(self):
        self.stream.close()
