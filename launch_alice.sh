#!/bin/bash

source ~/.bashrc

cd ~/catkin_ws/
# wstool update -t src
catkin clean -y && catkin build
source ~/catkin_ws/devel/setup.bash
pip install -r  ~/catkin_ws/src/yolo_parts/requirements.txt
roslaunch csjbot_alice alice.launch
