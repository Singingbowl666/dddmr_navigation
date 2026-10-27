#!/usr/bin/env python3
"""Closed-loop X30 navigation regression: isolated DDS, synthetic sensors only.

Run after sourcing install/setup.bash. Commands never reach the robot bridge.
Logs, actual executed poses and test parameters are kept in the printed directory.
"""
import os
os.environ['ROS_DOMAIN_ID'] = '88'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
os.environ['FASTDDS_BUILTIN_TRANSPORTS'] = 'UDPv4'
import argparse
import json
import math
import signal
import subprocess
import tempfile
import time
from pathlib import Path
import yaml
import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_prefix
from dddmr_sys_core.action import PToPMoveBase
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py.point_cloud2 import create_cloud
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', choices=['left', 'right', 'wall', 'stale'], default='left')
    parser.add_argument('--timeout', type=float, default=65)
    args = parser.parse_args()
    rclpy.init()
    node = Node('x30_avoidance_regression')
    tf = TransformBroadcaster(node)
    logs_dir = Path(tempfile.mkdtemp(prefix='x30_avoidance_'))
    print(f'LOG_DIR {logs_dir}', flush=True)
    state = dict(x=0., y=0., yaw=0., v=0., w=0., active=False, sensing=True)
    commands, poses, paths = [], [], []
    latest = [0.]
    # Block one side too, so a detour must use the remaining side.
    side = -1 if args.case == 'left' else 1
    obstacle_xy = [(1.9 + i*.05, j*.05) for i in range(-3, 4) for j in range(-4, 5)]
    obstacle_xy += [(1.9+i*.05, side*(.25+j*.05)) for i in range(-3, 4) for j in range(19)]
    if args.case in ('wall', 'stale'):
        obstacle_xy = [(1.5+i*.05, j*.1) for i in range(-1, 2) for j in range(-35, 36)]
    obstacles = []
    fields = [PointField(name=n, offset=i*4, datatype=PointField.FLOAT32, count=1)
              for i,n in enumerate(('x','y','z','intensity'))]
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    ground_pub = node.create_publisher(PointCloud2, '/mapground', qos)
    map_pub = node.create_publisher(PointCloud2, '/mapcloud', qos)
    lidar_pub = node.create_publisher(PointCloud2, '/lidar_points', 10)
    odom_pub = node.create_publisher(Odometry, '/leg_odom', 10)
    def on_cmd(msg):
        assert math.isfinite(msg.linear.x) and math.isfinite(msg.angular.z), 'Non-finite velocity'
        assert abs(msg.linear.x) <= .2001 and abs(msg.angular.z) <= .5001, 'Velocity limit exceeded'
        state.update(v=msg.linear.x, w=msg.angular.z)
        latest[0] = time.monotonic()
        commands.append([latest[0], msg.linear.x, msg.angular.z])
    node.create_subscription(Twist, '/x30_avoidance_test/cmd_vel', on_cmd, 10)
    node.create_subscription(NavPath, '/awared_global_path', lambda m: paths.append(
        [(p.pose.position.x, p.pose.position.y) for p in m.poses]), 10)
    previous = [time.monotonic()]
    def publish_state():
        now = time.monotonic()
        dt = min(now - previous[0], .1)
        previous[0] = now
        if state['active']:
            if now - latest[0] > .5:
                state.update(v=0., w=0.)
            mid = state['yaw'] + state['w']*dt*.5
            state['x'] += state['v']*math.cos(mid)*dt
            state['y'] += state['v']*math.sin(mid)*dt
            state['yaw'] += state['w']*dt
            poses.append([now, state['x'], state['y'], state['yaw']])
            # Independent footprint test on the executed state, not planner scores.
            c,s = math.cos(state['yaw']), math.sin(state['yaw'])
            for ox,oy in obstacles:
                dx,dy = ox-state['x'], oy-state['y']
                if abs(c*dx+s*dy) < .70 and abs(-s*dx+c*dy) < .40:
                    raise AssertionError(f'Executed footprint hits obstacle at {ox,oy}; pose={state}')
        transform = TransformStamped()
        transform.header = Header(frame_id='map', stamp=node.get_clock().now().to_msg())
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = state['x']
        transform.transform.translation.y = state['y']
        transform.transform.translation.z = .5
        transform.transform.rotation.z = math.sin(state['yaw']/2)
        transform.transform.rotation.w = math.cos(state['yaw']/2)
        tf.sendTransform(transform)
        odom = Odometry()
        odom.header, odom.child_frame_id = transform.header, 'base_link'
        odom.pose.pose.position.x, odom.pose.pose.position.y = state['x'],state['y']
        odom.pose.pose.position.z = .5
        odom.pose.pose.orientation = transform.transform.rotation
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = state['v'],state['w']
        odom_pub.publish(odom)
    node.create_timer(.02, publish_state)
    def publish_scan():
        if not state['sensing']:
            return
        c,s = math.cos(state['yaw']), math.sin(state['yaw'])
        points = []
        # Ground returns, self returns and external obstacles share one scan.
        for x,y,z in ([(state['x']+i*.15, state['y']+j*.15, 0.)
                        for i in range(-15, 26) for j in range(-20, 21)] +
                       [(x,y,z) for x,y in obstacles for z in (.35,.45,.55,.65)]):
            dx,dy = x-state['x'], y-state['y']
            points.append((c*dx+s*dy, -s*dx+c*dy, z-.5, 0.))
        points += [(i*.1,j*.1,-.1,0.) for i in range(-5,6) for j in range(-3,4)]
        lidar_pub.publish(create_cloud(Header(frame_id='base_link',
            stamp=node.get_clock().now().to_msg()), fields, points))
    node.create_timer(.1, publish_scan)
    ground_pub.publish(create_cloud(Header(frame_id='map'), fields,
        [(i*.15,j*.15,0.,0.) for i in range(-15,50) for j in range(-24,25)]))
    map_pub.publish(create_cloud(Header(frame_id='map'), fields,
        [(12.+i*.1,12.+j*.1,.5,0.) for i in range(4) for j in range(4)]))
    config = yaml.safe_load((Path(__file__).resolve().parents[1]/
        'src/dddmr_p2p_move_base/config/x30_ig_loc_navigation.yaml').read_text())
    config_path = logs_dir/'params.yaml'
    config_path.write_text(yaml.safe_dump(config))
    processes, log_files = [], []
    client = ActionClient(node, PToPMoveBase, '/p2p_move_base')
    def spin(seconds):
        end = time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(node, timeout_sec=.02)
            if any(p.poll() is not None for p in processes):
                raise RuntimeError('Planner exited; inspect logs')
    def wait(future, seconds):
        end = time.monotonic()+seconds
        while not future.done() and time.monotonic()<end:
            spin(.05)
        if not future.done():
            raise TimeoutError('Action timed out')
        return future.result()
    def stopped():
        # Planning wait states may hold the last zero command without publishing
        # again; count wall-clock time rather than old samples from motion.
        recent = [c for c in commands if c[0] >= time.monotonic()-1.]
        return (bool(commands) and abs(commands[-1][1]) < 1e-4 and
                abs(commands[-1][2]) < 1e-4 and
                all(abs(v) < 1e-4 and abs(w) < 1e-4 for _,v,w in recent) and
                abs(state['v']) < 1e-4 and abs(state['w']) < 1e-4)
    try:
        for package,exe in [('global_planner','global_planner_node'),('p2p_move_base','p2p_move_base_node')]:
            binary = Path(get_package_prefix(package))/'lib'/package/exe
            log = (logs_dir/(exe+'.log')).open('w')
            log_files.append(log)
            processes.append(subprocess.Popen([str(binary),'--ros-args','--params-file',str(config_path),
                '-r','/cmd_vel:=/x30_avoidance_test/cmd_vel'], stdout=log,stderr=subprocess.STDOUT))
        end = time.monotonic()+20
        while not client.server_is_ready() and time.monotonic()<end:
            spin(.1)
        if not client.server_is_ready():
            raise TimeoutError('Navigation did not start')
        spin(2)
        goal = PToPMoveBase.Goal()
        goal.target_pose = PoseStamped(header=Header(frame_id='map'))
        goal.target_pose.pose.position.x = 3.8
        goal.target_pose.pose.orientation.w = 1.
        handle = wait(client.send_goal_async(goal),5)
        assert handle.accepted, 'Goal rejected'
        result = handle.get_result_async()
        state['active'] = True
        spin(.5)
        obstacles.extend(obstacle_xy)
        if args.case == 'wall':
            spin(10)
            assert state['x']<1., 'Failed to stop before blocking wall'
            assert stopped(), 'Wall stop not stable'
            print('PASS wall: no free corridor, stopped without footprint collision', flush=True)
            obstacles.clear()
        elif args.case == 'stale':
            # Fresh empty scans allow forward motion, then loss of sensor must stop it.
            obstacles.clear()
            spin(3)
            assert any(v>.01 for _,v,_ in commands[-10:]), 'No initial motion'
            state['sensing'] = False
            spin(2)
            assert stopped(), 'Stale scan did not stop motion'
            print('PASS stale sensor: motion stopped after scan loss', flush=True)
            state['sensing'] = True
        started, report = time.monotonic(), time.monotonic()
        while not result.done() and time.monotonic()-started<args.timeout:
            spin(.1)
            if time.monotonic()-report>5:
                print('STATE', json.dumps({k:round(state[k],3) for k in ('x','y','yaw','v','w')}), flush=True)
                report = time.monotonic()
        assert result.done(), f'Did not reach goal: {state}'
        assert result.result().status == GoalStatus.STATUS_SUCCEEDED, f'Action status {result.result().status}'
        if args.case in ('left','right'):
            excursion = max(side*-p[2] for p in poses)
            assert excursion>.6, f'No detour toward open side: {excursion}'
        assert state['x']>3.4, f'Premature goal completion: {state}'
        print('PASS',args.case,'goal reached; peak lateral deviation',round(max(abs(p[2]) for p in poses),3),
              'commands',len(commands), flush=True)
    finally:
        (logs_dir/'execution.json').write_text(json.dumps(dict(case=args.case,poses=poses,commands=commands,paths=paths)))
        for p in processes:
            if p.poll() is None:
                p.send_signal(signal.SIGINT)
        for p in processes:
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                p.kill()
                p.wait()
        for log in log_files:
            log.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()
