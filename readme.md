# ROBIN - Lily Service Bot Interface for ROS

## Build Docker container:

`docker build -t ros:robin .`

## Run Docker container: 

Run the docker start script from your command line. This will start the docker container and automatically fire Robins launch file to start all the relevant nodes. 

`start_docker.sh`

Runs interactively and launches roscore as default. extend this as needed to setup everything we need to make porting easier to new machines, etc. Will need to adjust local directory for mapping source code until git repo is included in dockerfile.. (one step at a time eh).

To access docker container once running (e.g. to run catkin build, etc)... 

`docker exec -it robin bash` to launch (replacing container id with relevant one)

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
`python src/csjbot_alice/src/pseudo_lily_server.py`

Movement:
`rosrun csjbot_alice subscriber_movement.py`
`rosrun teleop_twist_keyboard teleop_twist_keyboard.py`

TODO: 
- Fix acceleration on movement - maybe look at incremental changes from 0 upwards depending on keypress timings.
- Look at MPO's speech research - https://github.com/MixedRealityLab/nottreal

Speech:
`rosrun csjbot_alice subscriber_speech.py`
`rostopic pub /speech std_msgs/String "Hello"`

Video:
`rosrun csjbot_alice subscriber_video.py`
`rostopic pub /video_enable std_msgs/Bool "true"` 
`rostopic pub /video_enable std_msgs/Bool "false"`
<!-- https://stackoverflow.com/questions/59587166/send-webcam-stream-from-server-in-python-using-sockets -->
<!-- https://www.youtube.com/watch?v=7-O7yeO3hNQ -->

Each frame of the picture header is: 0xff , 0xfe , 0xfd , 0xfc , 0xfb , 0xfa , 0xd8 
Tail of each frame image data is: 0xff , 0xfe , 0xfd , 0xfc , 0xfb , 0xfa , 0xd9


Currently:

First enable the video service. Launch:

`rosrun csjbot_alice subscriber_video.py`

Send a boolean message to active/enable the video socket:

`rostopic pub /video_enable std_msgs/Bool "true"`

Then run the publisher (which retrieves the image from the web socket and republishes it over ROS):

`rosrun csjbot_alice publisher_video.py`

publishes the images to the /camera/image topic. However, can only seem to pick it up using: 

`rosrun csjbot_alice subscriber_testimage.py`

Which writes the received image to an image (test2.jpg) which is viewable. Unable to get image viewer on an alternative machine from being able to receive the image. Not sure why, possibly a multi-node ros configuration issue. 


## GUI

To launch the GUI, currently (until integrated into a launch file):

`cd ~/your-project-folder/src/csjbot_alice/gui && python -m http.server`

In a web browser on the laptop:

http://localhost:8000/

## Acknowledgements
This work was developed as part of the the [Robots Mediating Interaction project][1],
 supported by [UK Engineering and Physical Sciences Research Council][2]
(EPSRC) through [Horizon: Trusted Data-Driven Products][3] ([EP/T022493/1][4]).

[1]: https://www.horizon.ac.uk/project/robots-mediating-interaction/
[2]: https://www.ukri.org/councils/epsrc/
[3]: https://www.horizon.ac.uk/
[4]: https://gtr.ukri.org/projects?ref=EP%2FT022493%2F1
