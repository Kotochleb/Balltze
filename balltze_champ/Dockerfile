FROM osrf/ros:melodic-desktop-full

# Use bash instead of sh
SHELL ["/bin/bash", "-c"]

RUN apt update && \
    apt install -y git \
        python \
        python-pip

RUN pip install \
        rosdep \
        rospkg 

WORKDIR /ros_ws

RUN git clone --recursive --branch=master --single-branch https://github.com/chvmp/champ src/champ && \
    git clone --branch=master --single-branch https://github.com/chvmp/champ_teleop src/champ_teleop && \
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y && \
    catkin_make

RUN apt clean && \
    rm -rf /var/lib/apt/lists/*

COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]