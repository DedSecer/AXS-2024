#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
import geometry_msgs.msg

 
if __name__ == '__main__':
    rospy.init_node('tf_test')
 
    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('3dmap', 'livox_frame', rospy.Time(0))
            lidar_world_pose = listener.lookupTransform("3dmap", "livox_frame", rospy.Time(0))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        print(lidar_world_pose)
 
        rate.sleep()