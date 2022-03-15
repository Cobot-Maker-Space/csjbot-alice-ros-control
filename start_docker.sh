#!/bin/bash

docker rm robin
docker run --name="robin" -p 80:8000 -p 9090:9090 --privileged -v ~/Documents/Projects/csjbot-lily-ros-control:/root/catkin_ws/ -it ros:robin
