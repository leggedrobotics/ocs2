#=============================================================================
# Copyright (C) 2021, Robotic Systems Lab, ETH Zurich
# All rights reserved.
# http://www.rsl.ethz.ch
#
# This software is distributed WITHOUT ANY WARRANTY; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the License for more information.
#=============================================================================
# Authors: Vassilios Tsounis, tsounisv@ethz.ch
#=============================================================================

#==
# Foundation
#==

ARG UBUNTU_VERSION=20.04
ARG CUDA=11.0
ARG DRIVER=460
ARG ARCH=
FROM nvidia/cuda${ARCH:+-$ARCH}:${CUDA}-base-ubuntu${UBUNTU_VERSION} as base
# ARCH and CUDA are specified again because the FROM directive resets ARGs
# (but their default value is retained if set previously)
ARG ARCH
ARG CUDA

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# Suppresses interactive calls to APT
ENV DEBIAN_FRONTEND="noninteractive"

# Install graphics drivers
RUN apt update && apt install -y \
  libnvidia-gl-${DRIVER} \
  && \
  rm -rf /var/lib/apt/lists/*

#==
# System APT dependencies and utilities
#==

# Needed for string substitution
SHELL ["/bin/bash", "-c"]
ENV TERM=xterm-256color

# Base dependencies
RUN apt update && apt install -y \
  sudo \
  lsb-release \
  ca-certificates \
  apt-utils \
  gnupg2 \
  locate \
  curl \
  wget \
  git \
  vim \
  gedit \
  unzip \
  iputils-ping \
  net-tools \
  htop \
  iotop \
  iftop \
  nmap \
  software-properties-common \
  build-essential \
  gdb \
  pkg-config \
  cmake \
  zsh \
  tzdata \
  clang-format \
  clang-tidy \
  xterm \
  gnome-terminal \
  tmux \
  terminator \
  tmuxinator \
  dialog \
  tasksel \
  meld \
  && \
  rm -rf /var/lib/apt/lists/*

#==
# ANYmal-Research
#==

# Credentials
ARG USERNAME
ARG PASSWORD

# Version
ARG RELEASE=21.03
ARG ROS=noetic

# PPA
RUN sh -c 'echo "machine packages.anymal.com login ${USERNAME} password ${PASSWORD}" > /etc/apt/auth.conf.d/packages.anymal.com.conf' && \
    chmod 600 /etc/apt/auth.conf.d/packages.anymal.com.conf && \
    curl -fsSL https://packages.anymal.com/gpg | apt-key add -

# Sources
RUN sh -c 'echo "deb [arch=amd64] https://packages.anymal.com/ros/release-${RELEASE}/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-release-${RELEASE}.list' && \
    sh -c 'echo "deb [arch=amd64] https://packages.anymal.com/rsl/release-${RELEASE}/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/rsl-release-${RELEASE}.list' && \
    curl -fsSL https://packages.anymal.com/gpg | apt-key add -

# ROS
RUN apt update && apt install -y \
  python3-pip \
  python3-rosdep \
  python3-rosclean \
  python3-rosparam \
  python3-progressbar \
  python3-catkin-tools \
  python3-osrf-pycommon \
  python3-virtualenvwrapper \
  ros-${ROS}-desktop-full \
  && \
  rm -rf /var/lib/apt/lists/*

RUN rosdep init && rosdep update

# ANYmal
RUN apt update && apt install -y \
  ros-${ROS}-rosbash \
  ros-${ROS}-ros-comm \
  ros-${ROS}-any-gazebo-msgs-dev \
  ros-${ROS}-any-hector-gazebo-plugins-dev \
  ros-${ROS}-anydrive-ethercat-ros-dev \
  ros-${ROS}-anydrive-monitor-dev \
  ros-${ROS}-anymal-c-sim-dev \
  ros-${ROS}-anymal-lowlevel-controller-common-dev \
  ros-${ROS}-anymal-motion-control-manager-dev \
  ros-${ROS}-anymal-velodyne-dev \
  ros-${ROS}-basic-contact-estimation-dev \
  ros-${ROS}-conditional-state-machine-dev \
  ros-${ROS}-cosmo-node-dev \
  ros-${ROS}-elevation-map-processing-dev \
  ros-${ROS}-gazebo-worlds-dev \
  ros-${ROS}-grid-map-visualization \
  ros-${ROS}-joy-interface-dev \
  ros-${ROS}-joy-manager-dev \
  ros-${ROS}-loco-anymal-dev \
  ros-${ROS}-loco-ros-anymal-dev \
  ros-${ROS}-pdb-msgs-dev \
  ros-${ROS}-rqt-multiplot \
  ros-${ROS}-series-elastic-actuator-anydrive-dev \
  ros-${ROS}-series-elastic-actuator-sim-dev \
  ros-${ROS}-velodyne-pointcloud \
  && \
  rm -rf /var/lib/apt/lists/*

#==
# OCS2
#==

# Base dependencies
RUN apt update && apt install -y \
  libboost-all-dev \
  libeigen3-dev \
  libglpk-dev \
  liburdfdom-dev \
  liboctomap-dev \
  libassimp-dev \
  python3-catkin-tools \
  ros-${ROS}-pybind11-catkin \
  doxygen-latex \
  && \
  rm -rf /var/lib/apt/lists/*

#==
# Cleanup
#==

# Updates
RUN apt update && apt upgrade -y

#==
# Environment
#==

COPY bashrc /etc/bash.bashrc
RUN chmod a+rwx /etc/bash.bashrc

#==
# Execution
#==

COPY entrypoint.sh /
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD []

# EOF
