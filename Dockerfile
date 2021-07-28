FROM ros:noetic

RUN apt-get update && apt-get install -y ros-noetic-desktop-full && \
    apt-get install -y ros-noetic-catkin python3-catkin-tools python3-osrf-pycommon && \
    apt-get install -y python3-wstool && \
    apt-get install -y bash-completion && \
    rm -rf /var/lib/apt/lists/* 

# ADD . /root/catkin_ws/

CMD ["/ros_entrypoint.sh", "roscore"]


# TODO 
# - Map ports across from outside to inside container (or run --network=host to auto assign ports)
# - Add device support to allow access to PS3 controller
