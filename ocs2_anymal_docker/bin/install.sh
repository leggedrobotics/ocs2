#!/bin/bash

#========================================================================================
# Copyright (C) 2021, Robotic Systems Lab, ETH Zurich
# All rights reserved.
# http://www.rsl.ethz.ch
#
# This software is distributed WITHOUT ANY WARRANTY; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the License for more information.
#========================================================================================
# Authors: Vassilios Tsounis, tsounsiv@ethz.ch
#========================================================================================

#==
# Configurations
#==

# Exits if error occurs
set -e

# Default argument values
NVIDIA=false

# Iterate over arguments list to configure the installation.
for i in "$@"
do
case $i in
  --nvidia)
    NVIDIA=true
    shift # past argument with no value
    ;;
  *)
    echo "[docker install]: Error: Unknown arguments: ${i#*=}"
    exit 1
    ;;
esac
done

#==
# APT dependencies
#==

# Remove existing installations
sudo apt remove docker docker-engine docker.io containerd runc

# Install base APT dependencies
sudo apt update && sudo apt install -y \
  apt-transport-https \
  ca-certificates \
  curl \
  gnupg \
  lsb-release

# Add the official docker APT repository
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# Install docker APT packages
sudo apt update && sudo apt install -y \
  docker-ce \
  docker-ce-cli \
  containerd.io

# (Optionally) Install NVIDIA docker support
if [[ ${NVIDIA} == true ]]
then
  distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
  curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
  curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
  sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
fi

#==
# System configurations
#==

# Create the `docker` group and add current user to that group
sudo groupadd docker
sudo usermod -aG docker "$USER"
newgrp docker

# Restart the docker service
sudo systemctl restart docker

# Check installation
docker run hello-world

# EOF
