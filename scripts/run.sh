#!/usr/bin/env bash
set -eo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
PX4_DIR=${PX4_DIR:-$HOME/PX4-Autopilot}
VENV_DIR=${VENV_DIR:-$HOME/.venvs/ai-drone}
export YOLO_CONFIG_DIR=${YOLO_CONFIG_DIR:-$HOME/.config/Ultralytics}
COMMAND=${1:-help}
if (( $# )); then shift; fi
if [[ "$COMMAND" == help ]]; then
  echo 'bash scripts/run.sh {sim|sim-avoid|sim-slalom|sim-challenge|avoid|detect|detect-avoid|detect-slalom|detect-challenge|capture|unified|view|monitor|replay|benchmark|scenario-matrix|check} [options]'; exit 0
fi
if [[ ! -f "$VENV_DIR/bin/activate" || ! -f /opt/ros/humble/setup.bash ]]; then
  echo 'Chua cai xong. Chay bash scripts/setup_ubuntu.sh truoc.' >&2; exit 1
fi
source /opt/ros/humble/setup.bash
source "$VENV_DIR/bin/activate"
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
mkdir -p "$YOLO_CONFIG_DIR"
cd "$ROOT"
case "$COMMAND" in
  sim)
    python scripts/install_world.py --px4-dir "$PX4_DIR"
    cd "$PX4_DIR"
    PX4_GZ_WORLD=baylands_yolo make -j"${BUILD_JOBS:-4}" px4_sitl gz_x500_mono_cam
    ;;
  sim-avoid)
    if pgrep -x px4 >/dev/null || pgrep -f 'gz[ -]sim' >/dev/null; then
      echo 'Hay dung PX4/Gazebo cu truoc khi chay sim-avoid.' >&2; exit 1
    fi
    python scripts/install_avoidance.py --px4-dir "$PX4_DIR"
    if [[ ! -x "$PX4_DIR/build/px4_sitl_default/bin/px4" ]]; then
      echo 'Chua build PX4 SITL. Chay setup truoc.' >&2; exit 1
    fi
    python scripts/sim_avoid.py --px4-dir "$PX4_DIR"
    ;;
  sim-slalom)
    if pgrep -x px4 >/dev/null || pgrep -f 'gz[ -]sim' >/dev/null; then
      echo 'Hay dung PX4/Gazebo cu truoc khi chay sim-slalom.' >&2; exit 1
    fi
    python scripts/install_avoidance.py --px4-dir "$PX4_DIR"
    if [[ ! -x "$PX4_DIR/build/px4_sitl_default/bin/px4" ]]; then
      echo 'Chua build PX4 SITL. Chay setup truoc.' >&2; exit 1
    fi
    python scripts/sim_avoid.py --px4-dir "$PX4_DIR" --world slalom_yolo
    ;;
  sim-challenge)
    if pgrep -x px4 >/dev/null || pgrep -f 'gz[ -]sim' >/dev/null; then
      echo 'Hay dung PX4/Gazebo cu truoc khi chay sim-challenge.' >&2; exit 1
    fi
    python scripts/install_avoidance.py --px4-dir "$PX4_DIR"
    if [[ ! -x "$PX4_DIR/build/px4_sitl_default/bin/px4" ]]; then
      echo 'Chua build PX4 SITL. Chay setup truoc.' >&2; exit 1
    fi
    python scripts/sim_avoid.py --px4-dir "$PX4_DIR" --world challenge_yolo
    ;;
  detect) python yolo_gz_node.py "$@" ;;
  detect-avoid) python yolo_gz_node.py --topic /world/obstacle_yolo/model/x500_mono_lidar_0/link/camera_link/sensor/imager/image "$@" ;;
  detect-slalom) python yolo_gz_node.py --topic /world/slalom_yolo/model/x500_mono_lidar_0/link/camera_link/sensor/imager/image "$@" ;;
  detect-challenge) python yolo_gz_node.py --topic /world/challenge_yolo/model/x500_mono_lidar_0/link/camera_link/sensor/imager/image "$@" ;;
  capture) python scripts/capture_ros_image.py "$@" ;;
  avoid)
    # avoid_fly.py tự giữ khóa liên tiến trình; nhờ đó cả unified CLI và lệnh
    # trực tiếp đều không thể cùng tranh quyền Offboard/MAVSDK.
    python avoid_fly.py "$@"
    ;;
  unified) python scripts/unified_drone.py "$@" ;;
  view) ros2 run rqt_image_view rqt_image_view ;;
  monitor) python scripts/monitor_flight.py "$@" ;;
  replay) python scripts/plot_slalom_replay.py "$@" ;;
  benchmark) python scripts/bao_cao_benchmark_sitl.py "$@" ;;
  scenario-matrix) python scripts/run_scenario_matrix.py "$@" ;;
  check)
    python -c 'import rclpy,gz.transport13,gz.msgs10.image_pb2,mavsdk,ultralytics,torch; print("Imports OK; Ultralytics",ultralytics.__version__); print("CUDA:",torch.cuda.is_available())'
    gz topic -l
    ;;
  *) echo "Lenh khong hop le: $COMMAND" >&2; exit 2 ;;
esac
