#!/usr/bin/bash
roslaunch carto_loc 2d_localization.launch > /dev/null &
python /root/robot_tools/examples/ros_base_control.py &
python /root/Workspace/AXS_baseline/ICRA2024-Sim2Real-AXS/src/airbot/example/AXS_baseline_only_close.py
