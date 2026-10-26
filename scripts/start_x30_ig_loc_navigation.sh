#!/usr/bin/env bash
set -eo pipefail

# Run on the X30 workstation desktop with the lidar/IMU drivers already running.
# One Terminator window contains localization and navigation; only navigation
# starts RViz, using exactly the same launch/configuration as the bag test.
# --prepare-only checks dependencies and writes the terminal commands without
# stopping/starting localization or navigation, or changing /use_sim_time.

prepare_only=false
case "${1:-}" in
  "") ;;
  --prepare-only) prepare_only=true ;;
  -h|--help)
    cat <<'EOF'
Usage: bash scripts/start_x30_ig_loc_navigation.sh [--prepare-only]

Start ig_loc localization and DDDMR navigation using live driver data.
No drivers, rosbag playback, or automatic goals are started.
Default velocity output: /cmd_vel
To observe velocities without sending them to /cmd_vel:
  CMD_VEL_TOPIC=/dddmr_cmd_vel_test bash scripts/start_x30_ig_loc_navigation.sh

Environment overrides: ROS1_WS, ROS2_WS, NAV_CONTAINER, MAP_PCD, GROUND_PCD,
ROS_MASTER_URI, CMD_VEL_TOPIC, PUBLISH_BASE_TO_LIDAR (default false).
EOF
    exit 0 ;;
  *) echo "ERROR: unknown argument: $1" >&2; exit 2 ;;
esac
[[ $# -le 1 ]] || { echo 'ERROR: too many arguments' >&2; exit 2; }

ROS1_WS="${ROS1_WS:-/home/ysc/x30_ws/uav_localization}"
ROS2_WS="${ROS2_WS:-/root/dddmr_navigation}"
NAV_CONTAINER="${NAV_CONTAINER:-dddmr_humble_dev}"
MAP_PCD="${MAP_PCD:-${ROS2_WS}/maps/x30_dlio/map.pcd}"
GROUND_PCD="${GROUND_PCD:-${ROS2_WS}/maps/x30_dlio/ground.pcd}"
CMD_VEL_TOPIC="${CMD_VEL_TOPIC:-/cmd_vel}"
PUBLISH_BASE_TO_LIDAR="${PUBLISH_BASE_TO_LIDAR:-false}"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"

[[ -n "${DISPLAY:-}" ]] || { echo 'ERROR: DISPLAY is not set; run from the workstation desktop.' >&2; exit 1; }
command -v terminator >/dev/null || { echo 'ERROR: terminator is not installed.' >&2; exit 1; }
command -v docker >/dev/null || { echo 'ERROR: docker is not installed.' >&2; exit 1; }
[[ -r "$ROS1_WS/devel/setup.bash" ]] || { echo "ERROR: ROS 1 workspace is not built: $ROS1_WS" >&2; exit 1; }
systemctl is-active --quiet ros1_bridge.service || { echo 'ERROR: ros1_bridge.service must already be active.' >&2; exit 1; }
docker inspect "$NAV_CONTAINER" >/dev/null 2>&1 || { echo "ERROR: container not found: $NAV_CONTAINER" >&2; exit 1; }
if [[ "$(docker inspect --format '{{.State.Running}}' "$NAV_CONTAINER")" != true ]]; then
  docker start "$NAV_CONTAINER" >/dev/null
fi
docker exec -e DDDMR_LIVE_WS="$ROS2_WS" -e DDDMR_LIVE_MAP="$MAP_PCD" \
  -e DDDMR_LIVE_GROUND="$GROUND_PCD" "$NAV_CONTAINER" bash -c '
    for path in "$DDDMR_LIVE_WS/install/setup.bash" "$DDDMR_LIVE_MAP" "$DDDMR_LIVE_GROUND"; do
      [[ -r "$path" ]] || { echo "ERROR: missing container file: $path" >&2; exit 1; }
    done
  '

source /opt/ros/noetic/setup.bash
rosparam list >/dev/null || { echo "ERROR: ROS master is unavailable: $ROS_MASTER_URI" >&2; exit 1; }
if ! "$prepare_only" && pgrep -f '[/]rosbag/[p]lay|[r]osbag play' >/dev/null; then
  echo 'ERROR: rosbag playback is running. Stop playback before using live driver data.' >&2
  exit 1
fi

RUNTIME_DIR="/tmp/dddmr_x30_live_${UID}"
if "$prepare_only"; then
  RUNTIME_DIR="$(mktemp -d /tmp/dddmr_x30_live_prepare.XXXXXX)"
else
  # Stop only the old localization/navigation launches, preserving drivers.
  pkill -INT -f '[r]oslaunch ig_loc x30.launch' 2>/dev/null || true
  docker exec -e DDDMR_LIVE_WS="$ROS2_WS" "$NAV_CONTAINER" bash -c '
    pkill -INT -f "[r]os2 launch p2p_move_base x30_ig_loc_navigation.launch" 2>/dev/null || true
    for attempt in {1..40}; do
      if ! pgrep -f "$DDDMR_LIVE_WS/install/(p2p_move_base|global_planner)/lib/" >/dev/null; then exit 0; fi
      sleep 0.5
    done
    echo "ERROR: previous navigation processes did not stop." >&2
    exit 1
  '
  for attempt in {1..40}; do
    if ! pgrep -f '[/]ig_loc/ig_loc_node( |$)' >/dev/null; then break; fi
    sleep 0.5
  done
  if pgrep -f '[/]ig_loc/ig_loc_node( |$)' >/dev/null; then
    echo 'ERROR: previous localization did not stop.' >&2
    exit 1
  fi
  if [[ -f "$RUNTIME_DIR/terminator.pid" ]]; then
    old_pid="$(cat "$RUNTIME_DIR/terminator.pid")"
    old_args="$(ps -p "$old_pid" -o args= 2>/dev/null || true)"
    if [[ "$old_args" == *terminator*"--config=$RUNTIME_DIR/terminator.conf"* ]]; then
      kill -TERM "$old_pid" 2>/dev/null || true
      sleep 1
    fi
  fi
  rosparam set /use_sim_time false
fi
mkdir -p "$RUNTIME_DIR"

printf -v ros1_setup_q '%q' "$ROS1_WS/devel/setup.bash"
printf -v ros_master_uri_q '%q' "$ROS_MASTER_URI"
printf -v nav_container_q '%q' "$NAV_CONTAINER"
printf -v ros2_setup_q '%q' "$ROS2_WS/install/setup.bash"
printf -v map_pcd_q '%q' "$MAP_PCD"
printf -v ground_pcd_q '%q' "$GROUND_PCD"
printf -v cmd_vel_topic_q '%q' "$CMD_VEL_TOPIC"
printf -v publish_tf_q '%q' "$PUBLISH_BASE_TO_LIDAR"

cat > "$RUNTIME_DIR/localization.sh" <<EOF
#!/usr/bin/env bash
set -eo pipefail
printf '\033]0;X30 live localization\007'
source /opt/ros/noetic/setup.bash
source $ros1_setup_q
export ROS_MASTER_URI=$ros_master_uri_q
echo '[localization] live /lidar_points and /imu/data; use_sim_time=false'
exec roslaunch ig_loc x30.launch use_rviz:=false
EOF

navigation_inner="source /opt/ros/humble/setup.bash && source $ros2_setup_q && export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTDDS_BUILTIN_TRANSPORTS=UDPv4 && exec ros2 launch p2p_move_base x30_ig_loc_navigation.launch map_pcd:=$map_pcd_q ground_pcd:=$ground_pcd_q use_sim_time:=false cmd_vel_topic:=$cmd_vel_topic_q publish_base_to_lidar:=$publish_tf_q"
printf -v navigation_inner_q '%q' "$navigation_inner"
cat > "$RUNTIME_DIR/navigation.sh" <<EOF
#!/usr/bin/env bash
set -eo pipefail
printf '\033]0;X30 live navigation\007'
echo '[navigation] native /leg_odom; velocity output: $CMD_VEL_TOPIC'
exec docker exec -it -e DISPLAY -e XAUTHORITY=/tmp/dddmr_x30.Xauthority $nav_container_q bash -lc $navigation_inner_q
EOF
chmod +x "$RUNTIME_DIR/localization.sh" "$RUNTIME_DIR/navigation.sh"

cat > "$RUNTIME_DIR/terminator.conf" <<EOF
[global_config]
[keybindings]
[profiles]
  [[x30_live]]
    exit_action = hold
    scrollback_infinite = True
[layouts]
  [[x30_live]]
    [[[window0]]]
      type = Window
      parent = ""
      size = 1500, 900
    [[[main_split]]]
      type = HPaned
      parent = window0
      order = 0
      position = 750
      ratio = 0.5
    [[[localization]]]
      type = Terminal
      parent = main_split
      order = 0
      profile = x30_live
      command = $RUNTIME_DIR/localization.sh
    [[[navigation]]]
      type = Terminal
      parent = main_split
      order = 1
      profile = x30_live
      command = $RUNTIME_DIR/navigation.sh
[plugins]
EOF

if "$prepare_only"; then
  echo "Prepared live startup commands in: $RUNTIME_DIR"
  echo 'Localization/navigation were not started and /use_sim_time was not changed.'
  echo "Velocity output on normal startup: $CMD_VEL_TOPIC"
  exit 0
fi

desktop_xauthority="${XAUTHORITY:-$HOME/.Xauthority}"
if [[ -r "$desktop_xauthority" ]]; then
  docker cp "$desktop_xauthority" "$NAV_CONTAINER:/tmp/dddmr_x30.Xauthority"
fi
terminator --no-dbus --config="$RUNTIME_DIR/terminator.conf" --layout=x30_live \
  --title='X30 live localization and navigation' >"$RUNTIME_DIR/terminator.log" 2>&1 &
terminator_pid=$!
echo "$terminator_pid" > "$RUNTIME_DIR/terminator.pid"
sleep 1
if ! kill -0 "$terminator_pid" 2>/dev/null; then
  cat "$RUNTIME_DIR/terminator.log" >&2
  exit 1
fi
echo 'Opened one Terminator window with two panes: localization and navigation.'
echo "Velocity output: $CMD_VEL_TOPIC"
echo 'Using live driver data and real time; send goals manually in the existing navigation RViz.'
