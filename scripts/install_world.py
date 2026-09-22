"""Install a separately named world; never overwrite PX4's baylands.sdf."""
import argparse
from pathlib import Path
import shutil
import time


def install(px4_dir):
    source = Path(__file__).resolve().parents[1] / "worlds/baylands_yolo.sdf"
    destination_dir = Path(px4_dir).expanduser() / "Tools/simulation/gz/worlds"
    if not destination_dir.is_dir():
        raise FileNotFoundError(f"Missing PX4 Gazebo worlds directory: {destination_dir}")
    destination = destination_dir / source.name
    if destination.exists():
        if destination.read_bytes() == source.read_bytes():
            return destination
        backup = destination.with_suffix(f".sdf.backup-{time.time_ns()}")
        shutil.copy2(destination, backup)
        print(f"Backup: {backup}")
    shutil.copy2(source, destination)
    return destination


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--px4-dir", default="~/PX4-Autopilot")
    print(install(p.parse_args().px4_dir))
