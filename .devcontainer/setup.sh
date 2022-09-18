#!/bin/bash
set -e

cd /ros2_ws

vcs import < src/balltze.repos src
vcs import < src/simulation.repos src
apt update
rosdep update --rosdistro $ROS_DISTRO
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
