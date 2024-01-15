#!/bin/bash
sleep 30s
source ~/.bashrc

cd ~/catkin_ws/
# wstool update -t src
[ ! -d "~/catkin_ws/build" ] && catkin_make
catkin clean -y && catkin build --verbose
source ~/catkin_ws/devel/setup.bash
#pip install -r  ~/catkin_ws/src/yolo_parts/requirements.txt
roslaunch csjbot_alice alice.launch
