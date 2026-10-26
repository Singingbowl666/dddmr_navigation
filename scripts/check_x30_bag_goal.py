#!/usr/bin/env python3
"""Send one bag-test goal, sample planned velocities for 12 s, then cancel it.

Run in the Humble container after starting test_x30_ig_loc_navigation.sh.
Velocity output is observed on /dddmr_cmd_vel_test.
"""
import json, math, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener
from rclpy.action import ActionClient
from dddmr_sys_core.action import PToPMoveBase
from action_msgs.srv import CancelGoal

rclpy.init()
n=Node('x30_bag_goal_probe')
buf=Buffer(); listener=TransformListener(buf,n)
state={'velocities':[], 'path_sizes':[], 'goal':None, 'pose_at_goal':None}
ground=[]
ready_at=None
def on_weighted(msg):
    global ready_at
    if ready_at is None: ready_at=time.monotonic(); print('GLOBAL_GRAPH_READY',flush=True)
def on_ground(msg):
    if not ground:
        ground.extend((float(p[0]),float(p[1]),float(p[2])) for p in point_cloud2.read_points(msg,field_names=('x','y','z'),skip_nans=True))
        print('GROUND_POINTS',len(ground),flush=True)
def on_vel(msg):
    v=[msg.linear.x,msg.linear.y,msg.angular.z]
    state['velocities'].append(v)
    if len(state['velocities'])<=5 or (any(abs(a)>1e-5 for a in v) and not state.get('printed_nonzero')):
        print('CMD_VEL',v,flush=True)
    if any(abs(a)>1e-5 for a in v): state['printed_nonzero']=True
n.create_subscription(Twist,'/dddmr_cmd_vel_test',on_vel,10)
n.create_subscription(Path,'/awared_global_path',lambda m:state['path_sizes'].append(len(m.poses)),10)
qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
n.create_subscription(PointCloud2,'/mapground',on_ground,qos)
n.create_subscription(PointCloud2,'/weighted_ground',on_weighted,qos)
pub=n.create_publisher(PoseStamped,'/goal_pose_3d',10)
client=ActionClient(n,PToPMoveBase,'/p2p_move_base')
start=time.monotonic(); sent=None
while time.monotonic()-start<85:
    rclpy.spin_once(n,timeout_sec=0.1)
    if sent is None and ready_at is not None and time.monotonic()-ready_at>3 and ground and client.server_is_ready() and any(i.node_name=='clicked2goal' for i in n.get_subscriptions_info_by_topic('/goal_pose_3d')):
        try:
            tr=buf.lookup_transform('map','base_link',rclpy.time.Time())
        except Exception:
            continue
        p=tr.transform.translation; q=tr.transform.rotation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        tx,ty=p.x+1.5*math.cos(yaw),p.y+1.5*math.sin(yaw)
        target=min(ground,key=lambda g:(g[0]-tx)**2+(g[1]-ty)**2+0.1*(g[2]-(p.z-0.5))**2)
        goal=PoseStamped(); goal.header.frame_id='map'
        goal.pose.position.x,goal.pose.position.y,goal.pose.position.z=target
        goal.pose.orientation.z=math.sin(yaw/2); goal.pose.orientation.w=math.cos(yaw/2)
        state['velocities'].clear(); state['path_sizes'].clear(); state.pop('printed_nonzero',None)
        pub.publish(goal); sent=time.monotonic()
        state['goal']={'x':target[0],'y':target[1],'z':target[2],'qz':goal.pose.orientation.z,'qw':goal.pose.orientation.w}
        state['pose_at_goal']=[p.x,p.y,p.z,yaw]
        print('GOAL_SENT',json.dumps(state['goal']), 'ROBOT_POSE',state['pose_at_goal'],flush=True)
    if sent is not None and time.monotonic()-sent>12: break
cancel=n.create_client(CancelGoal,'/p2p_move_base/_action/cancel_goal')
if sent is not None and cancel.wait_for_service(timeout_sec=2):
    future=cancel.call_async(CancelGoal.Request())
    rclpy.spin_until_future_complete(n,future,timeout_sec=3)
    print('GOAL_CANCELLED',future.result(),flush=True)
    for _ in range(10): rclpy.spin_once(n,timeout_sec=0.1)
state['velocity_count']=len(state['velocities'])
state['nonzero_count']=sum(any(abs(a)>1e-5 for a in v) for v in state['velocities'])
state['max_abs_vx']=max((abs(v[0]) for v in state['velocities']),default=0)
state['max_abs_wz']=max((abs(v[2]) for v in state['velocities']),default=0)
with open('/tmp/x30_bag_goal_result.json','w') as f: json.dump(state,f,indent=2)
print('RESULT',json.dumps({k:v for k,v in state.items() if k not in ('velocities','path_sizes')}),'MAX_PATH_SIZE',max(state['path_sizes'],default=0),flush=True)
n.destroy_node(); rclpy.shutdown()

raise SystemExit(0 if state['nonzero_count'] > 0 and max(state['path_sizes'], default=0) > 0 else 1)
