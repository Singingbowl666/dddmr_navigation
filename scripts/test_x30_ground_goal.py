#!/usr/bin/env python3
"""Exercise real planners with ground goals and an elevated base_link.

Run in the built Humble workspace. This test uses an isolated ROS domain,
synthetic terrain/TF/leg odometry and a test velocity topic.
"""
import os

os.environ["ROS_DOMAIN_ID"] = "77"
os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
os.environ["FASTDDS_BUILTIN_TRANSPORTS"] = "UDPv4"

import copy
import argparse
import signal
import subprocess
import tempfile
import time
from pathlib import Path

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_prefix
from dddmr_sys_core.action import PToPMoveBase
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py.point_cloud2 import create_cloud, read_points
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--obstacles', action='store_true', help='Also exercise body-height lidar filtering and collision')
    args = parser.parse_args()
    rclpy.init()
    node = Node("x30_ground_goal_regression")
    robot = {"x": 0.0, "height": 0.6, "obstacle": False, "obstacle_height": 0.0}
    broadcaster = TransformBroadcaster(node)
    odom_pub = node.create_publisher(Odometry, "/leg_odom", 10)

    def publish_state():
        transform = TransformStamped()
        transform.header.frame_id = "map"
        transform.header.stamp = node.get_clock().now().to_msg()
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = robot["x"]
        transform.transform.translation.z = robot["height"]
        transform.transform.rotation.w = 1.0
        broadcaster.sendTransform(transform)
        odom = Odometry()
        odom.header = transform.header
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = robot["x"]
        odom.pose.pose.position.z = robot["height"]
        odom.pose.pose.orientation.w = 1.0
        odom_pub.publish(odom)

    node.create_timer(0.02, publish_state)
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    ground_pub = node.create_publisher(PointCloud2, "/mapground", qos)
    map_pub = node.create_publisher(PointCloud2, "/mapcloud", qos)
    fields = [PointField(name=n, offset=i * 4, datatype=PointField.FLOAT32, count=1)
              for i, n in enumerate(("x", "y", "z", "intensity"))]
    velocities = []
    observations = []
    node.create_subscription(Twist, '/x30_ground_goal_test/cmd_vel',
                             lambda m: velocities.append(m.linear.x), 10)
    node.create_subscription(PointCloud2, '/perception_3d_local/lidar/current_observation',
                             lambda m: observations.append(m), 10)
    if args.obstacles:
        lidar_pub = node.create_publisher(PointCloud2, '/lidar_points', 10)

        def publish_lidar():
            points = [(x / 10, y / 10, 0.0) for x in range(-10, 21) for y in range(-10, 11)]
            # Body returns must not block motion after the external obstacle clears.
            points += [(robot['x'] + x * .04, y * .04, robot['height'] - .2)
                       for x in range(-4, 5) for y in range(-3, 4)]
            points += [(robot['x'] + x, y, robot['height'] + z)
                       for x, y, z in ((-.466, 0, .035), (-.411, 0, -.056),
                                       (.433, 0, -.055), (.444, 0, .054))] * 10
            # Measured near-field returns and leg locations, including points
            # beyond the old 0.42/-0.35 m body filter. Keep publishing these
            # when the external obstacle is removed.
            points += [(robot['x'] + x, y, robot['height'] + z)
                       for x, y, z in ((.407, .05, .134), (.431, -.05, .14),
                                       (.50, .26, -.33), (-.50, -.26, -.33),
                                       (.64, .37, -.2), (-.64, -.37, -.2))] * 10
            if robot['obstacle']:
                # Start just 1 cm outside the expanded body filter: widening
                # the filter must not erase a low obstacle in front of the dog.
                points += [(.66 + x * .025, y * .05, robot['obstacle_height'])
                           for x in range(5) for y in range(-3, 4)]
            cloud_header = Header(frame_id='base_link', stamp=node.get_clock().now().to_msg())
            lidar_pub.publish(create_cloud(cloud_header, fields,
                [(x - robot['x'], y, z - robot['height'], 0.0) for x, y, z in points]))

        node.create_timer(0.1, publish_lidar)
    header = Header(frame_id="map")
    ground_pub.publish(create_cloud(header, fields,
        [(x / 10, y / 10, 0.0, 0.0) for x in range(-10, 31) for y in range(-10, 11)]))
    map_pub.publish(create_cloud(header, fields,
        [(5.0 + i / 10, 5.0 + j / 10, 0.6, 0.0) for i in range(4) for j in range(4)]))
    client = ActionClient(node, PToPMoveBase, "/p2p_move_base")
    processes = []
    logs = []
    log_dir = Path(tempfile.mkdtemp(prefix="x30_ground_goal_"))
    print(f"LOG_DIR {log_dir}", flush=True)

    def spin_for(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            for process in processes:
                if process.poll() is not None:
                    raise RuntimeError(f"Planner exited with {process.returncode}; see {log_dir}")

    def wait_future(future, seconds):
        deadline = time.monotonic() + seconds
        while not future.done() and time.monotonic() < deadline:
            spin_for(0.05)
        if not future.done():
            raise TimeoutError(f"Action response timed out; see {log_dir}")
        return future.result()

    def send_goal(x, return_handle=False):
        goal = PToPMoveBase.Goal()
        goal.target_pose = PoseStamped()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = node.get_clock().now().to_msg()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.orientation.w = 1.0
        handle = wait_future(client.send_goal_async(goal), 5)
        if not handle.accepted:
            raise AssertionError("Ground goal rejected")
        return handle if return_handle else handle.get_result_async()

    def expect_success(future):
        result = wait_future(future, 10)
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            raise AssertionError(f"Goal failed with status {result.status}; see {log_dir}")

    try:
        config = copy.deepcopy(yaml.safe_load((Path(__file__).resolve().parents[1] /
            "src/dddmr_p2p_move_base/config/x30_ig_loc_navigation.yaml").read_text()))
        # This fixture holds yaw fixed to isolate local height filtering and
        # arrival. Dynamic detours require integrated yaw/odometry and are
        # exercised separately by test_x30_obstacle_avoidance.py.
        config['perception_3d_global']['ros__parameters']['plugins'] = ['map']
        if args.obstacles:
            config['perception_3d_local']['ros__parameters']['lidar']['pub_current_observation_for_visualization'] = True
        else:
            # Isolate arrival behavior from lidar processing in the default test.
            config["perception_3d_local"]["ros__parameters"]["plugins"] = ["map", "path_blocked_strategy"]
        config_path = log_dir / "params.yaml"
        config_path.write_text(yaml.safe_dump(config))
        for package, executable in (("global_planner", "global_planner_node"),
                                    ("p2p_move_base", "p2p_move_base_node")):
            binary = Path(get_package_prefix(package)) / "lib" / package / executable
            log = (log_dir / f"{executable}.log").open("w")
            logs.append(log)
            processes.append(subprocess.Popen(
                [str(binary), "--ros-args", "--params-file", str(config_path),
                 "-r", "/cmd_vel:=/x30_ground_goal_test/cmd_vel"],
                stdout=log, stderr=subprocess.STDOUT))
        deadline = time.monotonic() + 20
        while not client.server_is_ready() and time.monotonic() < deadline:
            spin_for(0.1)
        if not client.server_is_ready():
            raise TimeoutError("Navigation action server did not start")
        spin_for(2)
        if args.obstacles:
            for height, goal_x in ((0.5, 1.5), (0.6, 1.6), (0.9, 1.7)):
                # The obstacle is 5 cm above the configured body-frame cutoff.
                high_obstacle = height - .3 + .05
                robot.update(height=height, obstacle=True, obstacle_height=high_obstacle)
                spin_for(.5)
                if not observations:
                    raise AssertionError('No filtered lidar observation received')
                points = list(read_points(observations[-1], field_names=('x', 'y', 'z'), skip_nans=True))
                if len(points) < 5 or any(abs(float(p[2]) - high_obstacle) > .01 for p in points):
                    raise AssertionError('Ground/self returns remained or obstacle above body cutoff was lost')
                if min(float(p[0]) for p in points) > .68:
                    raise AssertionError('Obstacle just outside the body filter was erased')
                if observations[-1].header.stamp.sec == 0:
                    raise AssertionError('Filtered observation lost the lidar timestamp')
                velocities.clear()
                handle = send_goal(goal_x, return_handle=True)
                spin_for(2)
                if any(abs(v) > 1e-4 for v in velocities):
                    raise AssertionError('Planner commanded forward motion through an obstacle above the cutoff')
                print(f'PASS base height {height:.2f}: obstacle at base Z=-0.25 retained and blocks motion', flush=True)
                # Lower the obstacle to 2 cm below the cutoff while body returns
                # and live ground continue arriving at 10 Hz.
                robot['obstacle_height'] = height - .3 - .02
                velocities.clear()
                spin_for(2)
                if observations[-1].width * observations[-1].height != 0:
                    raise AssertionError('Ground, self returns or below-cutoff obstacle were retained')
                if not any(v > .01 for v in velocities):
                    raise AssertionError('Forward motion did not resume after lowering obstacle below cutoff')
                print(f'PASS base height {height:.2f}: ground/self/base Z=-0.32 removed and motion resumes', flush=True)
                wait_future(handle.cancel_goal_async(), 3)
                wait_future(handle.get_result_async(), 3)
            robot['obstacle'] = False
        for height, goal_x in ((0.45, 0.20), (0.60, 0.21), (0.90, 0.22)):
            robot["height"] = height
            spin_for(0.3)
            expect_success(send_goal(goal_x))
            print(f"PASS ground z=0, base_link z={height:.2f}, horizontal error={goal_x:.2f}", flush=True)
        future = send_goal(1.5)
        spin_for(2)
        if future.done():
            raise AssertionError("Goal outside XY tolerance completed before the robot arrived")
        print("PASS horizontal error=1.50 does not count as arrival", flush=True)
        robot.update(x=1.5, height=0.7)
        spin_for(0.3)
        expect_success(future)
        print("PASS moving above the ground goal completes the action", flush=True)
    finally:
        for process in processes:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
        for process in processes:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        for log in logs:
            log.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
