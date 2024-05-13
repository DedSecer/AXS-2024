# robot_tools
Useful tools (python classes) used in robot systems.

1. transformations: 完整移植ROS的tf_conversions模块中常用的transformations包
2. coordinate: 坐标系相关工具类
3. base_control: 机器人底盘控制工具类，以及底盘控制接口类
4. interpolate: 插值相关类
5. pather: 路径、目录处理相关工具类

整个包仅依赖常用的Python包，如：numpy、scipy、matplotlib等，是一个轻量的机器人工具包。

安装方法：pip install ./robot_tools

TODO:data_classes: 移植的ROS消息类型等，在ROS环境下使用ROS原生类型，非ROS环境下使用简化类型（移植后string化不够美观，原因暂不明，故未移植）
