# ROBIN - Lily Service Bot Interface for ROS

## Build Docker container:

`docker build -t ros:robin .`

## Run Docker container: 

`docker run --privileged -v ~/Documents/UoN/PHD/Projects/robin-ros:/root/catkin_ws/ -it ros:robin`

Runs interactively and launches roscore as default. extend this as needed to setup everything we need to make porting easier to new machines, etc. Will need to adjust local directory for mapping source code until git repo is included in dockerfile.. (one step at a time eh).

To access docker container once running (e.g. to run catkin build, etc)... 

`docker ps` to obtain container id

`docker exec -it aab481f0f19f bash` to launch (replacing container id with relevant one)

**NOTE**: Docker container is based on ros-noetic-desktop-full but we can strip down at later point if size is issue or just needs optimising.

## TODO: 
- Add device support
- Move volume mapping into composer file
- Execute launch file on startup (to allow docker container to run lily stack autonomously)
- Look at changing velocity based on length of button press (get controller working first)

## Notes:
- Volume used to store and persist logs between container sessions. Stored in /root/ros_logs/. `docker volume create ros_logs` to create but should do it automatically on start.

## Useful commands

Start Pseudo Server:
`python src/lily_interface/src/pseudo_lily_server.py`

Movement:
`rosrun lily_interface subscriber_movement.py`
`rosrun teleop_twist_keyboard teleop_twist_keyboard.py`

Speech:
`rosrun lily_interface subscriber_speech.py`
`rostopic pub /speech std_msgs/String "Hello"`

Video:
`rosrun lily_interface subscriber_video.py`
`rostopic pub /video_enable std_msgs/bool "true"` 
`rostopic pub /video_enable std_msgs/bool "false"`
<!-- https://stackoverflow.com/questions/59587166/send-webcam-stream-from-server-in-python-using-sockets -->
<!-- https://www.youtube.com/watch?v=7-O7yeO3hNQ -->

Each frame of the picture header is: 0xff , 0xfe , 0xfd , 0xfc , 0xfb , 0xfa , 0xd8 
Tail of each frame image data is: 0xff , 0xfe , 0xfd , 0xfc , 0xfb , 0xfa , 0xd9






