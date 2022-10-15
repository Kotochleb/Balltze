#!/bin/bash
set -e

cd /home/balltze/ros2_ws

vcs import --recursive < src/balltze/balltze.repos src
vcs import --recursive < src/balltze/simulation.repos src

rosdep update --rosdistro $ROS_DISTRO
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
