FROM osrf/ros:humble-desktop-full-jammy

RUN apt-get update 

RUN DEBIAN_FRONTEND=noninteractive apt-get install -y \
    keyboard-configuration --assume-yes

RUN apt-get install -y \
    libglpk-dev \
    libmpfr-dev \
    libglfw3 \
    libglfw3-dev \
    libosmesa6 \
    freeglut3-dev \
    mesa-common-dev \
    python3-pip \
    python3-wstool \
    wget

RUN apt-get install -y \
    ros-humble-pinocchio \
    ros-humble-hpp-fcl 

RUN apt-get clean \
    && rm -rf /var/lib/apt/lists/*

CMD ["bash"]
