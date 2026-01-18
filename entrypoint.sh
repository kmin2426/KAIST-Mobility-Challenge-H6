#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/foxy/setup.bash
source /ws/install/setup.bash
set -u

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_LOCALHOST_ONLY=0

: "${RUN_MODE:=algorithm}"  # algorithm|sim

run_simulator() {
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-100}"
  exec ros2 launch simulator_launch simulator_launch.py
}

run_algorithm() {
  : "${PROBLEM_ID:=1}"

  # 대회 제출은 task별로 run 하므로, domain은 통일(보통 100)로 두는 게 편함.
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-100}"

  case "${PROBLEM_ID}" in
    1) exec ros2 launch pkg_task_1_1 competition.launch.py ;;
    2) exec ros2 launch pkg_task_1_2 competition.launch.py ;;
    3) exec ros2 launch pkg_task_2   competition.launch.py ;;
    4) exec ros2 launch pkg_task_3   competition.launch.py ;;
    *) echo "Invalid PROBLEM_ID=${PROBLEM_ID} (expected 1|2|3|4)" >&2; exit 2 ;;
  esac
}

case "${RUN_MODE}" in
  sim|simulator) run_simulator ;;
  algorithm) run_algorithm ;;
  *) echo "Invalid RUN_MODE=${RUN_MODE} (expected algorithm|sim)" >&2; exit 2 ;;
esac
