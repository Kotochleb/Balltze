FROM ros:foxy-ros-core

SHELL ["/bin/bash", "-c"]

# udpate ubuntu and install dependencies
RUN apt update && \
    apt install -y git \
        python3 \
        python3-pip \
        python3-colcon-common-extensions 

# install python libraries
RUN pip3 install pyserial
RUN git clone https://github.com/NVIDIA/jetson-gpio.git --branch=master --single-brach \
    && cd jetson-gpio \
    && python3 setup.py install

RUN git clone https://github.com/Kotochleb/rpi-dynamixel-ax12a.git --branch=main --single-branch \
    && cd rpi-dynamixel-ax12a \
    && python3 setup.py install

# copy source code
RUN mkdir -p /ros2_ws/src
COPY ./balltze_motors/balltze_motors /ros2_ws/src/balltze_motors
COPY ./balltze_msgs /ros2_ws/src/balltze_msgs

# build ROS node
WORKDIR /ros2_ws
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build

# clean ubuntu packages
RUN apt clean && \
    rm -rf /var/lib/apt/lists/*

# setup entrypoint
COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]