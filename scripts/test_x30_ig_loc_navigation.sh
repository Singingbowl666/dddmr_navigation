#!/usr/bin/env bash
set -eo pipefail

# Run this script on the X30 workstation desktop. It opens one Terminator
# window with three panes for localization, navigation, and rosbag playback.

BAG_PATH="${1:-/home/ysc/bag_record/2026-10-20-16-14-13.bag}"
BAG_RATE="${BAG_RATE:-1.0}"
ROS1_WS="${ROS1_WS:-/home/ysc/x30_ws/uav_localization}"
ROS2_WS="${ROS2_WS:-/root/dddmr_navigation}"
NAV_CONTAINER="${NAV_CONTAINER:-dddmr_humble_dev}"
MAP_PCD="${MAP_PCD:-${ROS2_WS}/maps/x30_dlio/map.pcd}"
GROUND_PCD="${GROUND_PCD:-${ROS2_WS}/maps/x30_dlio/ground.pcd}"

if [[ -z "${DISPLAY:-}" ]]; then
  echo "ERROR: DISPLAY is not set. Run this script from the workstation desktop." >&2
  exit 1
fi
command -v terminator >/dev/null || { echo "ERROR: terminator is not installed." >&2; exit 1; }
command -v docker >/dev/null || { echo "ERROR: docker is not installed." >&2; exit 1; }
[[ -f "$BAG_PATH" ]] || { echo "ERROR: bag not found: $BAG_PATH" >&2; exit 1; }
[[ -f "$ROS1_WS/devel/setup.bash" ]] || { echo "ERROR: ROS 1 workspace is not built: $ROS1_WS" >&2; exit 1; }
docker inspect "$NAV_CONTAINER" >/dev/null 2>&1 || { echo "ERROR: container not found: $NAV_CONTAINER" >&2; exit 1; }
if [[ "$(docker inspect --format '{{.State.Running}}' "$NAV_CONTAINER")" != true ]]; then
  docker start "$NAV_CONTAINER" >/dev/null
fi

# Refresh the desktop cookie after a workstation reboot/login. The container
# otherwise keeps an old cookie and RViz exits with an X11 authorization error.
desktop_xauthority="${XAUTHORITY:-$HOME/.Xauthority}"
if [[ -f "$desktop_xauthority" ]]; then
  docker cp "$desktop_xauthority" "$NAV_CONTAINER:/tmp/dddmr_x30.Xauthority"
fi

if ! systemctl is-active --quiet ros1_bridge.service; then
  echo "ERROR: ros1_bridge.service is not active." >&2
  echo "Start it first with: sudo systemctl start ros1_bridge.service" >&2
  exit 1
fi

source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"

# Remove previous test processes so a restart cannot leave an old action goal,
# publisher, or /clock source in the ROS graph.
pkill -INT -f "rosbag play.*${BAG_PATH}" 2>/dev/null || true
pkill -INT -f "roslaunch ig_loc x30.launch" 2>/dev/null || true
docker exec -e DDDMR_NAV_WS="$ROS2_WS" "$NAV_CONTAINER" bash -c '
  pkill -INT -f "[r]os2 launch p2p_move_base x30_ig_loc_navigation.launch" 2>/dev/null || true
  for attempt in {1..40}; do
    if ! pgrep -f "$DDDMR_NAV_WS/install/(p2p_move_base|global_planner)/lib/" >/dev/null; then
      exit 0
    fi
    sleep 0.5
  done
  echo "ERROR: previous navigation processes did not stop." >&2
  exit 1
'
sleep 2

# The bridge was deliberately started on wall time. Only nodes launched after
# this assignment (ig_loc) consume bag time on ROS 1.
rosparam set /use_sim_time true

RUNTIME_DIR="/tmp/dddmr_x30_terminator_${UID}"
if [[ -f "$RUNTIME_DIR/terminator.pid" ]]; then
  old_terminator_pid="$(cat "$RUNTIME_DIR/terminator.pid")"
  if kill -0 "$old_terminator_pid" 2>/dev/null; then
    kill -TERM "$old_terminator_pid" 2>/dev/null || true
    sleep 1
  fi
fi
rm -rf "$RUNTIME_DIR"
mkdir -p "$RUNTIME_DIR"

printf -v ros1_setup_q '%q' "$ROS1_WS/devel/setup.bash"
printf -v ros_master_uri_q '%q' "$ROS_MASTER_URI"
printf -v nav_container_q '%q' "$NAV_CONTAINER"
printf -v bag_path_q '%q' "$BAG_PATH"
printf -v bag_rate_q '%q' "$BAG_RATE"
printf -v p2p_process_q '%q' "^$ROS2_WS/install/p2p_move_base/lib/p2p_move_base/p2p_move_base_node "
printf -v ros2_setup_q '%q' "$ROS2_WS/install/setup.bash"
printf -v map_pcd_q '%q' "$MAP_PCD"
printf -v ground_pcd_q '%q' "$GROUND_PCD"

cat > "$RUNTIME_DIR/localization.sh" <<EOF
#!/usr/bin/env bash
set -o pipefail
printf '\033]0;X30 localization\007'
source /opt/ros/noetic/setup.bash
source $ros1_setup_q
export ROS_MASTER_URI=$ros_master_uri_q
echo '[localization] roslaunch ig_loc x30.launch use_rviz:=false'
# The ROS 2 navigation launch already starts RViz. Running the localization
# RViz at the same time consumes enough CPU/GPU to make the bridged lidar drop
# frames, so keep only the navigation RViz during this integrated test.
roslaunch ig_loc x30.launch use_rviz:=false
status=\$?
echo '[localization] exited with status' \$status
exit \$status
EOF

navigation_inner="source /opt/ros/humble/setup.bash && source $ros2_setup_q && export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTDDS_BUILTIN_TRANSPORTS=UDPv4 && exec ros2 launch p2p_move_base x30_ig_loc_navigation.launch map_pcd:=$map_pcd_q ground_pcd:=$ground_pcd_q use_sim_time:=true cmd_vel_topic:=/dddmr_cmd_vel_test"
printf -v navigation_inner_q '%q' "$navigation_inner"
cat > "$RUNTIME_DIR/navigation.sh" <<EOF
#!/usr/bin/env bash
set -o pipefail
printf '\033]0;DDDMR navigation\007'
echo '[navigation] launching DDDMR in $NAV_CONTAINER'
docker exec -it -e DISPLAY -e XAUTHORITY=/tmp/dddmr_x30.Xauthority $nav_container_q bash -lc $navigation_inner_q
status=\$?
echo '[navigation] exited with status' \$status
exit \$status
EOF

cat > "$RUNTIME_DIR/bag.sh" <<EOF
#!/usr/bin/env bash
set -o pipefail
printf '\033]0;X30 bag playback\007'
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=$ros_master_uri_q
echo '[bag] waiting for /ig_loc_node ...'
until rosnode ping -c 1 /ig_loc_node >/dev/null 2>&1; do sleep 1; done
echo '[bag] waiting for the /p2p_move_base process ...'
# The action server is created only after perception receives its first TF, so
# waiting for the action server here would deadlock with bag startup.
until docker exec $nav_container_q pgrep -f $p2p_process_q >/dev/null; do sleep 1; done
sleep 2
echo '[bag] all consumers are ready; playing at rate $BAG_RATE'
echo '[bag] set BAG_RATE=0.5 only when diagnosing compute-load-related scan drops'
rosbag play --clock --rate $bag_rate_q $bag_path_q
status=\$?
echo '[bag] exited with status' \$status
exit \$status
EOF
chmod +x "$RUNTIME_DIR/localization.sh" "$RUNTIME_DIR/navigation.sh" "$RUNTIME_DIR/bag.sh"

cat > "$RUNTIME_DIR/terminator.conf" <<EOF
[global_config]
[keybindings]
[profiles]
  [[x30_test]]
    exit_action = hold
    scrollback_infinite = True
[layouts]
  [[x30_test]]
    [[[window0]]]
      type = Window
      parent = ""
      size = 1500, 900
    [[[main_split]]]
      type = VPaned
      parent = window0
      order = 0
      position = 450
      ratio = 0.5
    [[[localization]]]
      type = Terminal
      parent = main_split
      order = 0
      profile = x30_test
      command = $RUNTIME_DIR/localization.sh
    [[[bottom_split]]]
      type = HPaned
      parent = main_split
      order = 1
      position = 750
      ratio = 0.5
    [[[navigation]]]
      type = Terminal
      parent = bottom_split
      order = 0
      profile = x30_test
      command = $RUNTIME_DIR/navigation.sh
    [[[bag]]]
      type = Terminal
      parent = bottom_split
      order = 1
      profile = x30_test
      command = $RUNTIME_DIR/bag.sh
[plugins]
EOF

terminator --no-dbus --config="$RUNTIME_DIR/terminator.conf" --layout=x30_test \
  --title="X30 localization and navigation test" >"$RUNTIME_DIR/terminator.log" 2>&1 &
terminator_pid=$!
echo "$terminator_pid" > "$RUNTIME_DIR/terminator.pid"
sleep 1
if ! kill -0 "$terminator_pid" 2>/dev/null; then
  cat "$RUNTIME_DIR/terminator.log" >&2
  exit 1
fi

echo "Opened one Terminator window with three panes."
echo "Bag:  $BAG_PATH"
echo "Rate: $BAG_RATE"

# Opt in to the complete goal/velocity check from the same entry point.
# The launch above always remaps velocity to /dddmr_cmd_vel_test.
if [[ "${CHECK_GOAL:-0}" == "1" ]]; then
  echo '[test] sending one ground goal and checking /dddmr_cmd_vel_test for 12 seconds'
  docker exec -e DDDMR_TEST_WS="$ROS2_WS" "$NAV_CONTAINER" bash -lc '
    source /opt/ros/humble/setup.bash
    source "$DDDMR_TEST_WS/install/setup.bash"
    export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTDDS_BUILTIN_TRANSPORTS=UDPv4
    exec python3 "$DDDMR_TEST_WS/scripts/check_x30_bag_goal.py"
  '
fi
