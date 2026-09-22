"""Install this project's separately named LiDAR model and obstacle world into PX4."""

import argparse
from pathlib import Path
import shutil
import time
import xml.etree.ElementTree as ET


ASSETS = (
    Path("models/x500_mono_lidar/model.config"),
    Path("models/x500_mono_lidar/model.sdf"),
    Path("worlds/obstacle_yolo.sdf"),
    Path("worlds/slalom_yolo.sdf"),
    Path("worlds/challenge_yolo.sdf"),
)


def install(px4_dir):
    source_root = Path(__file__).resolve().parents[1]
    # Keep the caller's lexical path for the return value (notably a Windows
    # 8.3 path such as ADMINI~1), while resolving the path used for validation
    # and writes.  Both names identify the same directory, but callers should
    # not see their path silently rewritten.
    reported_gz_root = Path(px4_dir).expanduser().absolute() / "Tools/simulation/gz"
    gz_root = reported_gz_root.resolve()
    for directory in (gz_root / "models", gz_root / "worlds"):
        if not directory.is_dir():
            raise FileNotFoundError(f"Missing PX4 Gazebo directory: {directory}")
    for dependency in ("x500_mono_cam", "x500", "x500_base", "mono_cam"):
        if not (gz_root / "models" / dependency / "model.sdf").is_file():
            raise FileNotFoundError(f"Missing stock PX4 model: {dependency}")

    # Validate all source files and destination paths before writing anything.
    for relative in ASSETS:
        ET.parse(source_root / relative)
        destination = gz_root / relative
        if destination.resolve() != destination:
            raise ValueError(f"Refusing linked destination for owned assets: {destination}")

    installed = []
    for relative in ASSETS:
        source = source_root / relative
        destination = gz_root / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        if destination.exists():
            if destination.read_bytes() == source.read_bytes():
                installed.append(reported_gz_root / relative)
                continue
            backup = destination.with_name(f"{destination.name}.backup-{time.time_ns()}")
            shutil.copy2(destination, backup)
            print(f"Backup: {backup}")
        shutil.copy2(source, destination)
        installed.append(reported_gz_root / relative)
    return installed


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--px4-dir", default="~/PX4-Autopilot")
    for installed_path in install(parser.parse_args().px4_dir):
        print(installed_path)
