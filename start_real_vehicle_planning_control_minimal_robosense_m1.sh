#!/usr/bin/env bash
set -euo pipefail

workspace_root="/mnt/data/xmt200/autoware.ez21"
cleanup_script="${workspace_root}/scripts/cleanup_real_vehicle_planning_control_minimal_robosense_m1.sh"
monitor_script="${workspace_root}/scripts/monitor_autoware_freespace_cycle.py"
launch_file="real_vehicle_planning_control_minimal_robosense_m1.launch.xml"
launch_args=("$@")
ui_launcher_path="${workspace_root}/src/launcher/autoware_launch/ui/start_ui_server.sh"
ui_server_script="${workspace_root}/src/launcher/autoware_launch/ui/generate_autoware_map.py"
launch_log_dir="${AUTOWARE_LAUNCH_LOG_DIR:-}"
launch_log_max_bytes="${AUTOWARE_LAUNCH_LOG_MAX_BYTES:-1073741824}"
monitor_log_dir="${AUTOWARE_MONITOR_LOG_DIR:-/tmp/autoware_real_vehicle_planning_control_minimal_robosense_m1_monitor_logs}"
launch_timestamp="$(date +%Y%m%d_%H%M%S)"
monitor_pid=""

if [[ -z "${launch_log_dir}" ]]; then
  if [[ -n "${AUTOWARE_LAUNCH_LOG_PATH:-}" ]]; then
    launch_log_dir="$(dirname "${AUTOWARE_LAUNCH_LOG_PATH}")"
  else
    launch_log_dir="/tmp/autoware_real_vehicle_planning_control_minimal_robosense_m1_logs"
  fi
fi

mkdir -p "${launch_log_dir}"
mkdir -p "${monitor_log_dir}"
launch_log_path="${launch_log_dir}/autoware_real_vehicle_planning_control_minimal_robosense_m1_${launch_timestamp}.log"
monitor_output_prefix="${monitor_log_dir}/autoware_freespace_cycle_${launch_timestamp}"
removed_logs=()

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

has_launch_arg() {
  local key="$1"
  local arg
  for arg in "${launch_args[@]}"; do
    if [[ "${arg}" == "${key}:="* ]]; then
      return 0
    fi
  done
  return 1
}

get_launch_arg() {
  local key="$1"
  local arg
  for arg in "${launch_args[@]}"; do
    if [[ "${arg}" == "${key}:="* ]]; then
      printf '%s\n' "${arg#${key}:=}"
      return 0
    fi
  done
  return 1
}

listening_pids_on_port() {
  local port="$1"
  ss -ltnp "( sport = :${port} )" 2>/dev/null \
    | grep -o 'pid=[0-9]\+' \
    | cut -d= -f2 \
    | sort -u
}

is_ui_server_pid() {
  local pid="$1"
  local cmdline=""
  [[ -r "/proc/${pid}/cmdline" ]] || return 1
  cmdline="$(tr '\0' ' ' < "/proc/${pid}/cmdline" 2>/dev/null || true)"
  [[ "${cmdline}" == *"${ui_server_script}"* ]] || [[ "${cmdline}" == *"${ui_launcher_path}"* ]]
}

healthcheck_host() {
  local host="$1"
  case "${host}" in
    0.0.0.0|::|[::])
      echo "127.0.0.1"
      ;;
    *)
      echo "${host}"
      ;;
  esac
}

advertised_ui_host() {
  local host="$1"
  local resolved=""
  case "${host}" in
    0.0.0.0|::|[::])
      resolved="$(hostname -I 2>/dev/null | awk '{for (i=1; i<=NF; ++i) if ($i !~ /^127\./) {print $i; exit}}')"
      if [[ -n "${resolved}" ]]; then
        echo "${resolved}"
      else
        hostname
      fi
      ;;
    *)
      echo "${host}"
      ;;
  esac
}

cleanup_old_logs_if_needed() {
  local total_size=0
  local oldest_log=""

  while true; do
    total_size="$(find "${launch_log_dir}" -maxdepth 1 -type f -name '*.log' -printf '%s\n' 2>/dev/null | awk '{sum += $1} END {print sum + 0}')"
    if (( total_size <= launch_log_max_bytes )); then
      return 0
    fi

    oldest_log="$(find "${launch_log_dir}" -maxdepth 1 -type f -name '*.log' -printf '%T@ %p\n' 2>/dev/null | sort -n | head -n 1 | cut -d' ' -f2-)"
    if [[ -z "${oldest_log}" ]]; then
      return 0
    fi

    rm -f "${oldest_log}"
    removed_logs+=("${oldest_log}")
  done
}

stop_existing_ui_server() {
  local port="$1"
  local pid=""
  local existing_pid=""
  local any_alive=false
  local pids=()
  local _=0

  append_unique_pid() {
    local candidate="$1"
    for existing_pid in "${pids[@]}"; do
      if [[ "${existing_pid}" == "${candidate}" ]]; then
        return 0
      fi
    done
    pids+=("${candidate}")
  }

  while read -r pid; do
    [[ -n "${pid}" ]] || continue
    if is_ui_server_pid "${pid}"; then
      append_unique_pid "${pid}"
    fi
  done < <(listening_pids_on_port "${port}")

  while read -r pid; do
    [[ -n "${pid}" ]] || continue
    if is_ui_server_pid "${pid}"; then
      append_unique_pid "${pid}"
    fi
  done < <(pgrep -f "${workspace_root}/src/launcher/autoware_launch/ui/" || true)

  if (( ${#pids[@]} == 0 )); then
    return 0
  fi

  echo "[START] Stopping existing UI server process(es): ${pids[*]}"
  for pid in "${pids[@]}"; do
    kill "${pid}" >/dev/null 2>&1 || true
  done

  for _ in $(seq 1 20); do
    any_alive=false
    for pid in "${pids[@]}"; do
      if kill -0 "${pid}" >/dev/null 2>&1; then
        any_alive=true
        break
      fi
    done
    if [[ "${any_alive}" != "true" ]]; then
      return 0
    fi
    sleep 0.25
  done

  echo "[START] Force killing lingering UI server process(es): ${pids[*]}"
  for pid in "${pids[@]}"; do
    kill -KILL "${pid}" >/dev/null 2>&1 || true
  done
}

stop_existing_monitor() {
  local pid=""
  local any_alive=false
  local pids=()
  local _=0

  while read -r pid; do
    [[ -n "${pid}" ]] || continue
    pids+=("${pid}")
  done < <(pgrep -f "${monitor_script}" || true)

  if (( ${#pids[@]} == 0 )); then
    return 0
  fi

  echo "[START] Stopping existing freespace monitor process(es): ${pids[*]}"
  for pid in "${pids[@]}"; do
    kill "${pid}" >/dev/null 2>&1 || true
  done

  for _ in $(seq 1 20); do
    any_alive=false
    for pid in "${pids[@]}"; do
      if kill -0 "${pid}" >/dev/null 2>&1; then
        any_alive=true
        break
      fi
    done
    if [[ "${any_alive}" != "true" ]]; then
      return 0
    fi
    sleep 0.25
  done

  echo "[START] Force killing lingering freespace monitor process(es): ${pids[*]}"
  for pid in "${pids[@]}"; do
    kill -KILL "${pid}" >/dev/null 2>&1 || true
  done
}

cleanup_monitor() {
  if [[ -n "${monitor_pid}" ]] && kill -0 "${monitor_pid}" >/dev/null 2>&1; then
    kill "${monitor_pid}" >/dev/null 2>&1 || true
    wait "${monitor_pid}" 2>/dev/null || true
  fi
}

wait_for_ui_ready_notice() {
  local host="$1"
  local port="$2"
  local health_url="http://$(healthcheck_host "${host}"):${port}/health"
  local public_url="http://$(advertised_ui_host "${host}"):${port}/index.html"
  local wait_seconds="${AUTOWARE_UI_HEALTH_WAIT_SECONDS:-150}"
  local attempts=0
  (
    attempts=$((wait_seconds * 2))
    if (( attempts < 1 )); then
      attempts=1
    fi
    local _=0
    for _ in $(seq 1 "${attempts}"); do
      if curl -fsS --max-time 1 "${health_url}" >/dev/null 2>&1; then
        echo "[START] UI ready: ${public_url}"
        exit 0
      fi
      sleep 0.5
    done
    echo "[START] UI health check timed out after ${wait_seconds}s: ${health_url}"
  ) &
}

cleanup_old_logs_if_needed

exec > >(
  stdbuf -oL -eL tee -a "${launch_log_path}" |
  stdbuf -oL -eL awk '
    /^\[START\]/ { print; fflush(); next }
    /\[ERROR\]/ { print; fflush(); next }
    /process has died/ { print; fflush(); next }
    /Traceback/ { print; fflush(); next }
    /Exception:/ { print; fflush(); next }
    /Could not / { print; fflush(); next }
    /failed to / { print; fflush(); next }
    /Failed to / { print; fflush(); next }
  '
) 2>&1

mkdir -p "$HOME/.config/cyclonedds"
if [ -f "$HOME/remote_ros2-lan.xml" ]; then
  cp -f "$HOME/remote_ros2-lan.xml" "$HOME/.config/cyclonedds/ros2-lan.xml"
fi

set +u
source ~/.profile >/dev/null 2>&1 || true
source /opt/ros/humble/setup.bash
source "${workspace_root}/install/setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-66}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
if [ -f "$HOME/.config/cyclonedds/ros2-lan.xml" ]; then
  export CYCLONEDDS_URI="file://$HOME/.config/cyclonedds/ros2-lan.xml"
fi

stack_node_regex='^/(adapi|control|default_adapi|ez21_vehicle_interface|ins_driver_ez21|ins_localization_bridge|logging_diag_graph|map|perception|planning|robot_state_publisher|rslidar_points_destination_0|system|trajectory_relay)($|/)'
existing_nodes="$(timeout 8s ros2 node list 2>/dev/null | grep -Ec "${stack_node_regex}" || true)"
existing_launches="$(count_matches "${launch_file}")"
existing_workspace="$(count_matches "${workspace_root}/install/")"
existing_containers="$(count_matches '/opt/ros/humble/lib/rclcpp_components/component_container')"
existing_relays="$(count_matches 'topic_tools.*relay')"

if ! has_launch_arg rviz; then
  launch_args+=("rviz:=false")
fi
if ! has_launch_arg launch_ui; then
  launch_args+=("launch_ui:=true")
fi
if ! has_launch_arg ui_launcher; then
  launch_args+=("ui_launcher:=${ui_launcher_path}")
fi
if ! has_launch_arg ui_host; then
  launch_args+=("ui_host:=0.0.0.0")
fi
if ! has_launch_arg ui_port; then
  launch_args+=("ui_port:=8090")
fi
if ! has_launch_arg open_ui_index; then
  launch_args+=("open_ui_index:=false")
fi

ui_host="$(get_launch_arg ui_host || true)"
ui_host="${ui_host:-0.0.0.0}"
ui_port="$(get_launch_arg ui_port || true)"
ui_port="${ui_port:-8090}"

if (( existing_nodes > 0 || existing_launches > 0 || existing_workspace > 0 || existing_containers > 0 || existing_relays > 0 )); then
  echo "[START] Existing Autoware M1 stack detected. Cleaning before launch."
  "${cleanup_script}"
else
  echo "[START] No existing Autoware M1 stack detected."
fi

stop_existing_ui_server "${ui_port}"
stop_existing_monitor

for removed_log in "${removed_logs[@]}"; do
  echo "[START] Removed old launch log: ${removed_log}"
done

echo "[START] Launch log: ${launch_log_path}"
echo "[START] Freespace monitor events: ${monitor_output_prefix}.events.jsonl"
echo "[START] Freespace monitor summary: ${monitor_output_prefix}.summary.txt"
echo "[START] Freespace monitor stdout: ${monitor_output_prefix}.stdout.log"
echo "[START] Remote RViz: disabled by default"
echo "[START] UI target: http://$(advertised_ui_host "${ui_host}"):${ui_port}/index.html"
echo "[START] Launching ${launch_file} ${launch_args[*]}"

trap cleanup_monitor EXIT INT TERM

if [[ -f "${monitor_script}" ]]; then
  python3 "${monitor_script}" \
    --output-prefix "${monitor_output_prefix}" \
    >"${monitor_output_prefix}.stdout.log" 2>&1 &
  monitor_pid="$!"
else
  echo "[START] Freespace monitor script is missing: ${monitor_script}"
fi

wait_for_ui_ready_notice "${ui_host}" "${ui_port}"
ros2 launch autoware_launch "${launch_file}" "${launch_args[@]}"
