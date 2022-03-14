# ROBIN - Lily Service Bot Interface for ROS

## Build Docker container:

`docker build -t ros:robin .`

## Run Docker container: 

`docker run -v ~/Documents/UoN/PHD/Projects/robin-ros:/root/catkin_ws/ -it ros:robin`

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

Movement:
`rosrun lily_interface subscriber_movement.py`
`rosrun teleop_twist_keyboard teleop_twist_keyboard.py`


Speech:
`rosrun lily_interface subscriber_speech.py`
`rostopic pub /speech std_msgs/String "Hello"`


## GUI

To launch the GUI, currently (until integrated into a launch file):

`cd ~/your-project-folder/src/lily_interface/gui && python -m SimpleHTTPServer 8000;`

In a web browser on the laptop:

http://localhost:8000/




