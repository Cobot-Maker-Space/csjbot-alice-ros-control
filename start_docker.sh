#!/bin/bash

docker rm robin
docker run --name="robin" -p 80:8000 -p 9090:9090 -p 11311:11311 --privileged -v ~/Documents/Projects/csjbot-lily-ros-control:/root/catkin_ws/ -it ros:robin
