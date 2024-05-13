FROM jieyitsinghuawx/icra2024-sim2real-axs-baseline:v1.0.0
WORKDIR /root/Workspace
SHELL ["/bin/bash", "-c"]
ENV PATH=/root/miniconda3/envs/baseline/bin:/root/miniconda3/condabin:$PATH
ENV SHELL=/bin/bash
COPY misc/carto_package.xml /root/misc/carto_package.xml

# cartographer
RUN set -xe \
    && source /root/.bashrc \
    && source /opt/ros/noetic/setup.bash \
    && /root/miniconda3/bin/activate baseline \ 
    && apt-get update \
    && apt-get install -y python3-wstool python3-rosdep ninja-build stow \
    && pip uninstall --yes empy && pip install empy==3.3.4 \
    && mkdir cartographer_ws \
    && cd /root/Workspace/cartographer_ws \
    && wstool init src \
    && wstool merge -t src https://raw.githubusercontent.com/cartographer-project/cartographer_ros/master/cartographer_ros.rosinstall\
    && wstool update -t src \
    && rosdep update \
    && mv /root/misc/carto_package.xml /root/Workspace/cartographer_ws/src/cartographer/package.xml \
    && rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y \
    && ./src/cartographer/scripts/install_abseil.sh \
    && catkin_make_isolated --install --use-ninja



# fastlio and pointlio

RUN source /root/.bashrc \
    && source /opt/ros/noetic/setup.bash \
    && source /root/Workspace/cartographer_ws/devel_isolated/setup.bash \
    && git clone https://github.com/Livox-SDK/Livox-SDK --depth=1 \
    && cd Livox-SDK/build && cmake .. && make install  \
    && cd /root/Workspace/hdl_loc \
    && apt-get install -y ros-noetic-pointcloud-to-laserscan\
    && git clone https://github.com/Livox-SDK/livox_ros_driver --depth=1 ./src/livox_ros_driver/ \
    && git clone https://github.com/Livox-SDK/livox_ros_driver2 --depth=1 ./src/livox_ros_driver2 \
    && git clone https://github.com/hku-mars/FAST_LIO --depth=1 ./src/FAST_LIO \
    && cd src/FAST_LIO && git submodule update --init && cd ../.. \
    && git clone https://github.com/hku-mars/Point-LIO --depth=1 ./src/Point-LIO \
    && cd src/Point-LIO && git submodule update --init && cd ../.. \
    && catkin_make

# ultralytics for yolo
RUN pip install ultralytics


# own code
COPY airbot /root/Workspace/AXS_baseline/ICRA2024-Sim2Real-AXS/src/airbot
COPY ckpt /root/Workspace/AXS_baseline/ckpt
COPY robot_tools /root/robot_tools
COPY ws/src/hdl_global_localization /root/Workspace/hdl_loc/src/hdl_global_localization
COPY ws/src/hdl_localization /root/Workspace/hdl_loc/src/hdl_localization
COPY ws/src/carto_loc /root/Workspace/hdl_loc/src/carto_loc

COPY misc/start.sh /root/
COPY misc/start2.sh /root/

RUN source /root/.bashrc \
    && source /opt/ros/noetic/setup.bash \
    && source /root/Workspace/hdl_loc/devel/setup.bash \
    && source /root/Workspace/cartographer_ws/devel_isolated/setup.bash \
    && /root/miniconda3/bin/activate baseline \ 
    && cd /root/Workspace/hdl_loc \
    && catkin_make