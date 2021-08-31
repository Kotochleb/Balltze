FROM osrf/ros:foxy-desktop

SHELL ["/bin/bash", "-c"]

RUN apt update && \
    apt install -y git \
        ros-$ROS_DISTRO-geometry2 \
        ros-$ROS_DISTRO-gazebo-ros-pkgs

# COPY ./balltze_simulation/balltze_gazebo/balltze_gazebo /ros2_ws/src/balltze_gazebo
COPY ./balltze_description/balltze_description /ros2_ws/src/balltze_description

RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    ls && \
    colcon build

# clear ubuntu packages
RUN sudo apt clean && \
    rm -rf /var/lib/apt/lists/*

# setup entrypoint
COPY ./balltze_simulation/balltze_gazebo/ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]