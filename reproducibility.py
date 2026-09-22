"""Read-only reproducibility evidence for SITL logs and offline reports.

This module never imports MAVSDK, Gazebo, ROS, or any flight-control API.  It only
collects local version metadata and cryptographic file fingerprints so a recorded
SITL run can be reviewed and reproduced.
"""

from __future__ import annotations

from importlib import metadata
import hashlib
import os
from pathlib import Path
import platform
import subprocess
import sys
from typing import Iterable


SCHEMA_VERSION = 1
DEFAULT_PACKAGES = (
    "mavsdk",
    "numpy",
    "opencv-python-headless",
    "pytest",
    "pytest-asyncio",
    "ultralytics",
)


def sha256_file(path: str | Path) -> str:
    """Return a stable SHA-256 digest for an existing regular file."""
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def git_revision(root: str | Path = ".") -> str | None:
    """Return the current Git revision when Git metadata is available."""
    try:
        completed = subprocess.run(
            ["git", "rev-parse", "HEAD"], cwd=Path(root), check=True,
            capture_output=True, text=True, timeout=3,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    revision = completed.stdout.strip()
    return revision if revision else None


def git_dirty(root: str | Path = ".") -> bool | None:
    """Report whether the working tree has changes, or None if unavailable."""
    try:
        completed = subprocess.run(
            ["git", "status", "--porcelain"], cwd=Path(root), check=True,
            capture_output=True, text=True, timeout=3,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    return bool(completed.stdout.strip())


def installed_packages(names: Iterable[str] = DEFAULT_PACKAGES) -> dict[str, str | None]:
    """Return installed distribution versions without importing heavy packages."""
    versions: dict[str, str | None] = {}
    for name in sorted(set(names)):
        try:
            versions[name] = metadata.version(name)
        except metadata.PackageNotFoundError:
            versions[name] = None
    return versions


def environment_manifest(root: str | Path = ".", *, packages: Iterable[str] = DEFAULT_PACKAGES) -> dict:
    """Collect compact, JSON-safe evidence for a recorded simulation run."""
    return {
        "reproducibility_schema_version": SCHEMA_VERSION,
        "python": {
            "implementation": platform.python_implementation(),
            "version": platform.python_version(),
            "executable": sys.executable,
        },
        "platform": {
            "system": platform.system(),
            "release": platform.release(),
            "machine": platform.machine(),
        },
        "packages": installed_packages(packages),
        "git_revision": git_revision(root),
        "git_dirty": git_dirty(root),
        "process": {
            "pid": os.getpid(),
            "cwd": str(Path.cwd().resolve()),
        },
    }
