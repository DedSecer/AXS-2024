#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import sys
import select
import termios
import tty
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from IPython import embed
from actionlib_msgs.msg import GoalStatus
from hdl_localization.srv import getPlace,getPlaceRequest,getPlaceResponse
import subprocess
command1 ="bash /home/gyh/lesson7/airbot/airbot_play_gazebo_ws/src/airbot_play_control_upper/demo/run_pick_place.sh gazebo"
command2 = "rosrun airbot_play_vision_cpp ros_demo --not_show 0 --use_real 0 --negative 0"
class KeyboardMoveBaseController:
    def __init__(self):
        rospy.init_node('keyboard_move_base_controller', anonymous=True)
        self.rate = rospy.Rate(30)  # 10 Hz
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.wait_for_service('get_place_server')   # need to package the grasping process to a rosservice

        self.client.wait_for_server()
        self.goal_pose  = MoveBaseGoal()

    def getKey(self):
        # 获取键盘输入
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    def run_commands(*args):
    # 执行第一个命令
        process1 = subprocess.Popen(command1, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, start_new_session=True)
        
        # 等待第一个命令完成
        return_code1 = process1.wait()
        print(f"Command 1 completed with return code {return_code1}")

        # 执行第二个命令
        process2 = subprocess.Popen(command2, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, start_new_session=True)
        
        # 等待第二个命令完成
        return_code2 = process2.wait()
        print(f"Command 2 completed with return code {return_code2}")
    def run_command(*args):
        process = subprocess.Popen(command1, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
        output, error = process.communicate()

        # 获取命令的输出
        if process.returncode == 0:
            print("Command executed successfully")
            print("Output:", output.decode())
        else:
            print(f"Command failed with return code {process.returncode}")
            print("Error:", error.decode())


    def run(self):
        self.settings = termios.tcgetattr(sys.stdin)
        rospy.loginfo("'p: place of placing objects','g: place of getting objects'")
        
        while not rospy.is_shutdown():
            key = self.getKey()
            if key == 'g':   
                print("get the g value")
                self.goal_pose.target_pose.header.stamp = rospy.Time.now()
                self.goal_pose.target_pose.header.frame_id = "map"
                self.goal_pose.target_pose.pose.position.x = -0.5
                self.goal_pose.target_pose.pose.position.y = 0.25
                self.goal_pose.target_pose.pose.orientation.x = 0.0
                self.goal_pose.target_pose.pose.orientation.y = 0.0
                self.goal_pose.target_pose.pose.orientation.z = 0.0
                self.goal_pose.target_pose.pose.orientation.w = 1.0
                self.client.send_goal(self.goal_pose)
                wait = self.client.wait_for_result()
                if wait:
                    if self.client.get_state() ==  GoalStatus.SUCCEEDED:                       
                        print("begin grasping")                                                                  
                        getPlaceResult = rospy.ServiceProxy('get_place_server', getPlace)
                        response = getPlaceResult('get')
                        print(response.result)
                        if response.result==True:
                            print("finish grasping")                                               
                     
            elif key== 'p':
                print("get the p value")
                self.goal_pose.target_pose.header.stamp = rospy.Time.now()
                self.goal_pose.target_pose.header.frame_id = "map"
                self.goal_pose.target_pose.pose.position.x = 2.5
                self.goal_pose.target_pose.pose.position.y = 0.0
                self.goal_pose.target_pose.pose.orientation.x = 0.0
                self.goal_pose.target_pose.pose.orientation.y = 0.0
                self.goal_pose.target_pose.pose.orientation.z = 0.0
                self.goal_pose.target_pose.pose.orientation.w = 1.0
                self.client.send_goal(self.goal_pose)
                wait = self.client.wait_for_result()
                if wait:
                    if self.client.get_state() ==  GoalStatus.SUCCEEDED:
                        print("begin grasping")  
                        # subprocess.run(command2, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)                   
                        # getPlaceResult = rospy.ServiceProxy('get_place_server', getPlace)
                        # response = getPlaceResult('get')
                        # if response==True:
                        #     print("finish grasping")
            elif key== 'q':               
                break
            else:
                pass  
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        move_base_controller = KeyboardMoveBaseController()
        move_base_controller.run()
    except rospy.ROSInterruptException:
        pass
