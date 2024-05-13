#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal
from actionlib_msgs.msg import GoalStatusArray

class MoveBaseSender:
    def __init__(self):
        rospy.init_node('move_base_sender', anonymous=True)
        self.move_base_goal_pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=1)
        self.goal_status_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, self.goal_status_callback)
        self.rate = rospy.Rate(1)  # Adjust the rate as needed

    def goal_status_callback(self, data):
        if not data.status_list:
            rospy.loginfo("No goal status information received.")
            return
        latest_goal_status = data.status_list[-1]  # Get the latest goal status

        goal_id = latest_goal_status.goal_id.id
        status = latest_goal_status.status

        rospy.loginfo(f"Received goal status for goal_id {goal_id}: {status}")

        # You can implement additional logic based on the goal status
        if status == GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal succeeded!")
        elif status == GoalStatus.ABORTED:
            rospy.logwarn("Goal aborted!")
        elif status == GoalStatus.PREEMPTED:
            rospy.logwarn("Goal preempted!")
    
    def send_goal(self, goal_pose):
        move_base_goal = MoveBaseActionGoal()
        move_base_goal.goal.target_pose = goal_pose
        self.move_base_goal_pub.publish(move_base_goal)

    def run(self):
        while not rospy.is_shutdown():
            # Define your goal pose
            
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = "map"  # Adjust the frame_id based on your setup
            goal_pose.pose.position.x = 1.0  # Adjust the goal position
            goal_pose.pose.position.y = 1.0
            goal_pose.pose.orientation.w = 1.0  # Facing forward

            # Send the goal
            self.send_goal(goal_pose)

            # Sleep for a while before sending the next goal
            self.rate.sleep()

if __name__ == '__main__':
    move_base_sender = MoveBaseSender()
    move_base_sender.run()
