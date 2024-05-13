import rospy
from geometry_msgs.msg import PoseStamped, Pose
rospy.init_node("test_base")
msg = PoseStamped()
# Construct PoseStamped message with current time
msg.header.stamp = rospy.get_rostime()
msg.pose.position.x = -0.2741637241544763
msg.pose.position.y = 0.3241416117180577
msg.pose.position.z = -0.07743623649227918
msg.pose.orientation.x = -0.026456952502619244
msg.pose.orientation.y = 0.022510511678367467
msg.pose.orientation.z = 0.6642090190392874
msg.pose.orientation.w = 0.7467393692280592
puber = rospy.Publisher("/airbot/base_pose_cmd", PoseStamped, queue_size=5)
rate = rospy.Rate(200)
while not rospy.is_shutdown():
    puber.publish(msg)
    rate.sleep()
