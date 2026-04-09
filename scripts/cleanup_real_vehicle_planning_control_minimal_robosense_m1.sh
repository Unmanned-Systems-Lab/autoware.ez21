#!/usr/bin/env bash
set -euo pipefail

workspace_root="/mnt/data/xmt200/autoware.ez21"
launch_file="real_vehicle_planning_control_minimal_robosense_m1.launch.xml"

count_matches() {
  local pattern="$1"
  local matches
  matches="$(pgrep -af "${pattern}" || true)"
  if [ -z "${matches}" ]; then
    echo 0
  else
    printf '%s\n' "${matches}" | wc -l
  fi
}

set +u
source ~/.profile >/dev/null 2>&1 || true
source /opt/ros/humble/setup.bash >/dev/null 2>&1 || true
source "${workspace_root}/install/setup.bash" >/dev/null 2>&1 || true
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-66}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
if [ -f "$HOME/.config/cyclonedds/ros2-lan.xml" ] && [ -z "${CYCLONEDDS_URI:-}" ]; then
  export CYCLONEDDS_URI="file://$HOME/.config/cyclonedds/ros2-lan.xml"
fi

stack_node_regex='^/(adapi|control|default_adapi|ez21_vehicle_interface|ins_driver_ez21|ins_localization_bridge|logging_diag_graph|map|perception|planning|robot_state_publisher|rslidar_points_destination_0|system|trajectory_relay)($|/)'
node_count="$(timeout 8s ros2 node list 2>/dev/null | grep -Ec "${stack_node_regex}" || true)"
launch_count="$(count_matches "${launch_file}")"
workspace_proc_count="$(count_matches "${workspace_root}/install/")"
container_count="$(count_matches '/opt/ros/humble/lib/rclcpp_components/component_container')"
relay_count="$(count_matches 'topic_tools.*relay')"

echo "Detected existing stack state:"
echo "  nodes=${node_count}"
echo "  launch_processes=${launch_count}"
echo "  workspace_processes=${workspace_proc_count}"
echo "  component_containers=${container_count}"
echo "  relay_processes=${relay_count}"

if (( node_count == 0 && launch_count == 0 && workspace_proc_count == 0 && container_count == 0 && relay_count == 0 )); then
  echo "No existing Autoware M1 stack detected."
  exit 0
fi

patterns=(
  "${launch_file}"
  "${workspace_root}/install/"
  "/opt/ros/humble/lib/rclcpp_components/component_container"
  "topic_tools.*relay"
  "goal_pose_visualizer"
  "robot_state_publisher"
  "ros2cli.daemon"
)

echo "Stopping existing Autoware M1 processes..."
for pattern in "${patterns[@]}"; do
  pkill -TERM -f "${pattern}" >/dev/null 2>&1 || true
done

sleep 4

echo "Force-killing any remaining Autoware M1 processes..."
for pattern in "${patterns[@]}"; do
  pkill -KILL -f "${pattern}" >/dev/null 2>&1 || true
done

timeout 5s ros2 daemon stop >/dev/null 2>&1 || true
pkill -KILL -f '_ros2cli_daemon_' >/dev/null 2>&1 || true

sleep 2

remaining_launches="$(count_matches "${launch_file}")"
remaining_workspace="$(count_matches "${workspace_root}/install/")"
remaining_containers="$(count_matches '/opt/ros/humble/lib/rclcpp_components/component_container')"
remaining_relays="$(count_matches 'topic_tools.*relay')"

echo "Remaining after cleanup:"
echo "  launch_processes=${remaining_launches}"
echo "  workspace_processes=${remaining_workspace}"
echo "  component_containers=${remaining_containers}"
echo "  relay_processes=${remaining_relays}"
