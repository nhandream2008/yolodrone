#!/usr/bin/env bash
set -eo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source /etc/os-release
if [[ "$ID" != ubuntu || "$VERSION_ID" != 22.04 ]]; then
  echo 'Can Ubuntu 22.04 de dung ROS 2 Humble.' >&2; exit 1
fi
if [[ "$EUID" == 0 ]]; then
  : "${SETUP_USER:?Set SETUP_USER to an existing non-root Linux user}"
  if [[ "$(id -u "$SETUP_USER")" == 0 ]]; then
    echo 'SETUP_USER must not be root.' >&2; exit 1
  fi
  TARGET_HOME=$(getent passwd "$SETUP_USER" | cut -d: -f6)
  as_user() { runuser -u "$SETUP_USER" -- "$@"; }
  sudo() { "$@"; }
else
  TARGET_HOME=$HOME
  as_user() { "$@"; }
fi
PX4_DIR=${PX4_DIR:-$TARGET_HOME/PX4-Autopilot}
VENV_DIR=${VENV_DIR:-$TARGET_HOME/.venvs/ai-drone}
YOLO_CONFIG_DIR=${YOLO_CONFIG_DIR:-$TARGET_HOME/.config/Ultralytics}
mkdir -p "$ROOT/logs"
exec > >(tee -a "$ROOT/logs/setup.log") 2>&1
sudo apt-get update
sudo apt-get upgrade -y
sudo apt-get install -y git curl wget ca-certificates software-properties-common locales python3-venv python3-pip
sudo locale-gen en_US.UTF-8
export LANG=en_US.UTF-8
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
sudo add-apt-repository -y universe

# Build inside Linux home for WSL filesystem performance.
if [[ ! -d "$PX4_DIR" ]]; then
  as_user git clone --branch v1.16.0 --recursive https://github.com/PX4/PX4-Autopilot.git "$PX4_DIR"
elif [[ ! -d "$PX4_DIR/.git" ]]; then
  echo "Thu muc da ton tai nhung khong phai PX4 Git: $PX4_DIR" >&2; exit 1
elif [[ "$(as_user git -C "$PX4_DIR" describe --tags --exact-match HEAD 2>/dev/null)" != v1.16.0 ]]; then
  echo 'PX4 hien co khac v1.16.0. Dat PX4_DIR thanh thu muc moi de tranh sua du an cu.' >&2; exit 1
fi
as_user git -C "$PX4_DIR" submodule update --init --recursive
as_user mkdir -p "$(dirname "$VENV_DIR")"
as_user mkdir -p "$YOLO_CONFIG_DIR"
as_user python3 -m venv --system-site-packages "$VENV_DIR"
source "$VENV_DIR/bin/activate"
python -m pip install --upgrade pip
bash "$PX4_DIR/Tools/setup/ubuntu.sh" --no-nuttx
if [[ "$EUID" == 0 ]]; then
  chown -R "$SETUP_USER:$(id -gn "$SETUP_USER")" "$VENV_DIR"
fi

if ! dpkg-query -W ros2-apt-source >/dev/null 2>&1; then
  ROS_SOURCE_VERSION=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | python3 -c 'import json,sys; print(json.load(sys.stdin)["tag_name"])')
  ROS_DEB=$(mktemp --suffix=.deb)
  curl -fL "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_SOURCE_VERSION}/ros2-apt-source_${ROS_SOURCE_VERSION}.jammy_all.deb" -o "$ROS_DEB"
  sudo dpkg -i "$ROS_DEB"
  rm -f "$ROS_DEB"
fi
sudo apt-get update
sudo apt-get install -y ros-humble-desktop ros-humble-rqt-image-view python3-gz-transport13 python3-gz-msgs10
as_user "$VENV_DIR/bin/python" -m pip install -r "$ROOT/requirements.txt"
source /opt/ros/humble/setup.bash
python -c 'import rclpy, gz.transport13, gz.msgs10.image_pb2, mavsdk, ultralytics; print("Python imports OK")'
mkdir -p "$ROOT/models"
cd "$ROOT/models"
as_user env YOLO_CONFIG_DIR="$YOLO_CONFIG_DIR" "$VENV_DIR/bin/python" -c 'from ultralytics import YOLO; YOLO("yolo26n.pt"); print("YOLO26n loaded")'
cd "$PX4_DIR"
as_user env PATH="$VENV_DIR/bin:$PATH" make -j"${BUILD_JOBS:-4}" px4_sitl
python -m pip freeze > "$ROOT/logs/requirements-installed.txt"
as_user git rev-parse HEAD > "$ROOT/logs/px4-commit.txt"
echo 'Cai dat hoan tat. Chay bash scripts/run.sh sim trong thu muc AI DRONE.'
