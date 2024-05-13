""" 转换为ROS的数据类型相关函数 """
try:
    from geometry_msgs.msg import Twist, Pose, PoseStamped
except:
    _TO_TOS_MSG_INIT_FAILED_ = True
else:
    _TO_TOS_MSG_INIT_FAILED_ = False


def to_Twist(linear_all, angular=(0, 0, 0)):
    """将线速度和角速度转换为Twist消息"""
    msg = Twist()
    if len(linear_all) == 6:
        angular = linear_all[3:]
    msg.linear.x = linear_all[0]
    msg.linear.y = linear_all[1]
    msg.linear.z = linear_all[2]
    msg.angular.x = angular[0]
    msg.angular.y = angular[1]
    msg.angular.z = angular[2]
    return msg
