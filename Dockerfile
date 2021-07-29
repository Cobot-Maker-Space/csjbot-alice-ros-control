FROM ros:noetic

RUN apt-get update && apt-get install -y ros-noetic-desktop-full \
    ros-noetic-catkin ros-noetic-teleop-twist-keyboard\
    python3-catkin-tools python3-osrf-pycommon \
    python3-wstool python3-pip bash-completion && \
    rm -rf /var/lib/apt/lists/* 

# ADD . /root/catkin_ws/

WORKDIR /root/catkin_ws
RUN pip install websockets

RUN echo "source /opt/ros/$ROS_DISTRO/setup.sh" >> /root/.bashrc
RUN echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

ENV ROS_LOG_DIR=/root/ros_logs/ 

CMD ["/ros_entrypoint.sh", "roscore"]


# TODO 
# - Map ports across from outside to inside container (or run --network=host to auto assign ports)
# - Add device support to allow access to PS3 controller
