import argparse

parser = argparse.ArgumentParser("Base control node config.")

parser.add_argument(
    "-cp",
    "--current_pose_topic",
    type=str,
    default="/airbot/base/current_pose",  # /airbot/pose is old
    help="topic name of current pose, the type should be geometry_msgs/Pose",
)
parser.add_argument(
    "-tp",
    "--target_pose_topic",
    type=str,
    default="/airbot/base_pose_cmd",
    help="topic name of target pose, the type should be geometry_msgs/PoseStamped",
)
parser.add_argument(
    "-tv",
    "--target_velocity_topic",
    type=str,
    default="/cmd_vel",
    help="topic name of velocity, the type should be geometry_msgs/Twist",
)
parser.add_argument(
    "-f",
    "--fly",
    action="store_true",
    help="if set, the position z and rotation about x&y will be considered",
)
parser.add_argument(
    "-nis",
    "--not_ignore_same_target",
    action="store_true",
    help="if set, the same target pose won't be ignored",
)
args, unknown = parser.parse_known_args()

current_pose_topic = args.current_pose_topic
target_pose_topic = args.target_pose_topic
target_velocity_topic = args.target_velocity_topic
fly = args.fly
not_ignore_same_target = args.not_ignore_same_target

from robot_tools import BaseControl, transformations

BaseControl._TEST_ = False
base_control = BaseControl()  # 初始化base位姿控制接口
base_control.set_move_kp(1.1, 1.1)
base_control.set_velocity_limits((0.7, 1.0), (0.9, 1.0))
base_control.set_direction_tolerance((0.01, 0.017 * 60))
base_control.set_wait_tolerance(0.05, 0.017 * 12, 60, 200)
base_control.avoid_321()

import numpy as np
from geometry_msgs.msg import Pose, PoseStamped, Twist

base_pose_control_flag = False
last_target_pose = PoseStamped()
last_tolerance = {}


def current_pose_sub(msg: Pose):
    pos = [msg.position.x, msg.position.y, msg.position.z]
    ori = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
    if not fly:
        # enforce the position z and rotation about x&y to be the ignored
        pos[2] = 0
        ori = list(transformations.euler_from_quaternion(ori))
        ori[0] = ori[1] = 0
    base_control.set_current_world_pose(
        np.array(pos, dtype=np.float64), np.array(ori, dtype=np.float64)
    )
    # print("current pose received", pos, ori)


def target_pose_sub(msg: PoseStamped):
    global base_pose_control_flag, last_target_pose
    # 重复命令排除
    if (
        not not_ignore_same_target
        and last_target_pose.pose == msg.pose
        and last_target_pose.header.frame_id == msg.header.frame_id
    ):
        return
    else:
        last_target_pose = msg
    pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
    ori = [
        msg.pose.orientation.x,
        msg.pose.orientation.y,
        msg.pose.orientation.z,
        msg.pose.orientation.w,
    ]
    if not fly:
        # enforce the position z and rotation about x&y to be the ignored
        pos[2] = 0
        ori = list(transformations.euler_from_quaternion(ori))
        ori[0] = ori[1] = 0
    base_control.set_target_pose(
        np.array(pos, dtype=np.float64),
        np.array(ori, dtype=np.float64),
        msg.header.frame_id,
    )
    base_pose_control_flag = True
    # print("target pose received", pos, ori)


import rospy

node_name = "base_control_node"
rospy.init_node(node_name)
print(f"{node_name} started:")
print(f"  -current_pose_topic: {current_pose_topic}")
print(f"  -target_pose_topic: {target_pose_topic}")
print(f"  -target_velocity_topic: {target_velocity_topic}")

print("waiting for current pose...", end="", flush=True)
rospy.wait_for_message(current_pose_topic, Pose)
print("OK!")
current_pose_suber = rospy.Subscriber(
    current_pose_topic, Pose, current_pose_sub, queue_size=1
)
target_pose_suber = rospy.Subscriber(
    target_pose_topic, PoseStamped, target_pose_sub, queue_size=1
)
puber = rospy.Publisher(target_velocity_topic, Twist, queue_size=1)
vel_msg = Twist()
rate = rospy.Rate(200)
while not rospy.is_shutdown():
    # base pose control
    if base_pose_control_flag:
        rospy.set_param("/airbot/base/control_finished", False)
        tolerance = rospy.get_param("/airbot/base/tolerance", {})
        if tolerance != last_tolerance:
            last_tolerance = tolerance
            base_control.set_wait_tolerance(
                tolerance["position"], tolerance["rotation"], 60, 200
            )
            base_control.set_direction_tolerance(tolerance["direction"])
        base_control.avoid_swing(rospy.get_param("/airbot/base/avoid_swing", False))
        cmd = base_control.get_velocity_cmd(ignore_stop=True)
        vel_msg.linear.x = cmd[0][0]
        vel_msg.linear.y = cmd[0][1]
        vel_msg.angular.z = cmd[1][2]
        puber.publish(vel_msg)
        if (cmd[0] == 0).all() and (cmd[1] == 0).all():
            base_pose_control_flag = False
            print("base pose control finished")
            rospy.set_param("/airbot/base/control_finished", True)
    rate.sleep()
