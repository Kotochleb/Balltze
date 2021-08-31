FROM ros:foxy-ros-core

SHELL ["/bin/bash", "-c"]

RUN apt update && \
    apt install -y git \
        python3 \
        python3-pip \
        python3-colcon-common-extensions 

RUN pip3 install RPi.GPIO
RUN pip3 install pyserial

COPY ./balltze_motors/balltze_motors /ros2_ws/src/balltze_motors
COPY ./balltze_msgs /ros2_ws/src/balltze_msgs

RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws
# RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
#     colcon build

# setup entrypoint
# COPY ./ros_entrypoint.sh /
# ENTRYPOINT ["/ros_entrypoint.sh"]
# CMD ["bash"]