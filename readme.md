ROBIN - Lily Service Bot Interface for ROS

* To Build Docker:

`docker build -t ros:robin .`

* To run Docker container: 

`docker run -v /Users/sgreen/Documents/UoN/PHD/Projects/robin-ros:/root/catkin_ws/ -it ros:robin`

Runs interactively and launches roscore as default. extend this as needed to setup everything we need to make porting easier to new machines, etc. Will need to adjust local directory for mapping source code until git repo is included in dockerfile.. (one step at a time eh).

To access docker container once running (e.g. to run catkin build, etc)... 

`docker ps` to obtain container id

`docker exec -it aab481f0f19f bash` to launch (replacing container id with relevant one)


NOTE: Docker container is based on ros-noetic-desktop-full but we can strip down at later point if size is issue or just needs optimising.
