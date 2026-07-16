#!/usr/bin/env bash
# 一键拉起小车全部 launch:雷达 -> 建图 -> 任务控制
# 用法: ./start_all.sh    (Ctrl+C 一键停止所有节点)
set -u

# ---- source ROS 环境(已 source 过则跳过) ----
if [ -z "${ROS_DISTRO:-}" ]; then
  for setup in /opt/ros/*/setup.bash; do
    # shellcheck disable=SC1090
    source "$setup"
    break
  done
fi

# ---- source 本工作空间(脚本位于 src/,工作空间根为其上一级) ----
WS_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
if [ -f "$WS_ROOT/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$WS_ROOT/install/setup.bash"
else
  echo "警告: 未找到 $WS_ROOT/install/setup.bash,请先 colcon build" >&2
fi

PIDS=()
cleanup() {
  trap - INT TERM
  echo ""
  echo "正在停止所有节点..."
  for pid in "${PIDS[@]}"; do
    kill -INT "$pid" 2>/dev/null || true
  done
  wait 2>/dev/null
  echo "已全部退出"
  exit 0
}
trap cleanup INT TERM

echo "[1/3] 启动雷达: bluesea2 uart_lidar.launch"
ros2 launch bluesea2 uart_lidar.launch &
PIDS+=($!)
sleep 3

echo "[2/3] 启动建图: cartographer_ros cartographer_live.launch.py"
ros2 launch cartographer_ros cartographer_live.launch.py &
PIDS+=($!)
sleep 3

echo "[3/3] 启动任务控制: car_bringup car_mission.launch.py"
ros2 launch car_bringup car_mission.launch.py &
PIDS+=($!)

echo "----------------------------------------"
echo "全部已启动,Ctrl+C 停止所有节点"
echo "启动 A 路线: ros2 topic pub --once /car_mission_start std_msgs/msg/String \"data: 'true'\""
echo "----------------------------------------"
wait
