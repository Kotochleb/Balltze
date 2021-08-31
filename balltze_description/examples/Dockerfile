FROM osrf/ros:foxy-desktop
ENV ROS2_WS ros2_ws

SHELL ["/bin/bash", "-c"]

RUN apt update && \
    apt install -y \
        ros-$ROS_DISTRO-joint-state-publisher-gui \
        ros-$ROS_DISTRO-xacro

# build package
RUN mkdir -p /$ROS2_WS/src
COPY ./balltze_description /$ROS2_WS/src/balltze_description
WORKDIR /$ROS2_WS
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build

# clear ubuntu packages
RUN sudo apt clean && \
    rm -rf /var/lib/apt/lists/*

# setup entrypoint
COPY ./examples/ros_entrypoint.sh /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]