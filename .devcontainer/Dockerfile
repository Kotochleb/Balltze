FROM osrf/ros:galactic-desktop

SHELL ["/bin/bash", "-c"]

ENV RCUTILS_COLORIZED_OUTPUT=1

RUN apt update && \
    apt install -y \
        build-essential \
        cmake \
        git \
        libbullet-dev \
        python3-colcon-common-extensions \
        python3-flake8 \
        python3-pip \
        python3-pytest-cov \
        python3-rosdep \
        python3-setuptools \
        python3-vcstool \
        wget \
        clang-format && \
    python3 -m pip install -U \
        argcomplete \
        flake8-blind-except \
        flake8-builtins \
        flake8-class-newline \
        flake8-comprehensions \
        flake8-deprecated \
        flake8-docstrings \
        flake8-import-order \
        flake8-quotes \
        pytest-repeat \
        pytest-rerunfailures \
        pytest

    
RUN useradd -rm -d /home/balltze -s /bin/bash -g root -G sudo -u 1001 balltze && \
    echo 'balltze:balltze' | chpasswd
USER balltze

COPY . /home/balltze/ros2_ws
WORKDIR /home/balltze/ros2_ws

RUN echo ". /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc \
    && echo ". /home/balltze/ros2_ws/install/setup.bash" >> ~/.bashrc

CMD [ "sleep", "infinity" ]