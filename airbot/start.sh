#!/usr/bin/bash
roslaunch cartographer_ros agv_2d_localization.launch > /dev/null &
python /root/robot_tools/examples/ros_base_control.py > /dev/null &
python /root/Workspace/AXS_baseline/ICRA2024-Sim2Real-AXS/src/airbot/example/AXS_baseline.py
