import rospy
from geometry_msgs.msg import PoseStamped, Pose
rospy.init_node("test_arm")
msg = PoseStamped()
# Construct PoseStamped message with current time
msg.header.stamp = rospy.get_rostime()
# msg.pose.position.x = 0.7174252090257988
# msg.pose.position.y = -0.035814930010594985
msg.pose.position.x = 0.67902140 + 0.09
msg.pose.position.y = -0.1190196 + 0.05
msg.pose.position.z = 0.2
msg.pose.orientation.x = 0
msg.pose.orientation.y = 0
msg.pose.orientation.z = 0
msg.pose.orientation.w = 1
puber = rospy.Publisher("/airbot_play/pose_cmd", PoseStamped, queue_size=5)
rate = rospy.Rate(200)
while not rospy.is_shutdown():
    puber.publish(msg)
    rate.sleep()
