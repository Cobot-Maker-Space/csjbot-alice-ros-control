#!/bin/bash

source ~/.bashrc

cd ~/catkin_ws/
catkin build
source ~/catkin_ws/devel/setup.bash
roslaunch lily_interface robin.launch