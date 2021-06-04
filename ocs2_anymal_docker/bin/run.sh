#!/bin/bash

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

# Set package root path
PKGROOT="$( realpath "$( cd "$( dirname "${BASH_SOURCE[0]}" )" > /dev/null 2>&1 && pwd )"/../ )"

#==
# Configurations and arguments
#==

# Exits if error occurs
set -e

# Define image name
PLATFORM=cpu
RELEASE="21.03"
IMAGE=ocs2/ocs2-anymal:$RELEASE-$PLATFORM
ARGS="--device=/dev/dri"

# Parse arguments
for i in "$@"
do
case $i in
  -i=*|--image=*)
    IMAGE=${i#*=}
    shift # past argument=value
    ;;
  -r=*|--release=*)
    RELEASE=${i#*=}
    IMAGE=ocs2/ocs2-anymal:$RELEASE-$PLATFORM
    shift # past argument=value
    ;;
  -p=*|--platform=*)
    PLATFORM=${i#*=}
    if [[ $PLATFORM != cpu &&  $PLATFORM != gpu  ]]
    then
      echo "[build.sh]: Error: Unsupported platform type '$PLATFORM'. Supported types are: cpu, gpu"
      exit 1
    fi
    if [[ $PLATFORM == cpu ]] ;then ARGS="--device=/dev/dri"; fi
    if [[ $PLATFORM == gpu ]]; then ARGS="--gpus all"; fi
    IMAGE=ocs2/ocs2-anymal:$RELEASE-$PLATFORM
    shift # past argument=value
    ;;
  *)
    echo "[run.sh]: Error: Unknown argument(s)"
    exit 1
    ;;
esac
done

#==
# Environment configuration
#==

# Define environment variables for enabling graphical output for the container.
XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]
then
    touch $XAUTH
    xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
    xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
    chmod a+r $XAUTH
fi

# Create symlinks to user configs within the build context.
mkdir -p .etc && cd .etc
ln -sf /etc/passwd .
ln -sf /etc/shadow .
ln -sf /etc/group .
cd ..

#==
# Launch container
#==

# Launch a container from the prebuilt image.
docker run \
  $ARGS \
  --volume=$XSOCK:$XSOCK:rw \
  --volume=$XAUTH:$XAUTH:rw \
  --env="QT_X11_NO_MITSHM=1" \
  --env="XAUTHORITY=$XAUTH" \
  --env="DISPLAY=$DISPLAY" \
  --ulimit rtprio=99 \
  --cap-add=sys_nice \
  --net=host \
  --privileged \
  -eHOST_USERNAME=$(whoami) \
  -v$HOME:$HOME \
  -v$(pwd)/.etc/shadow:/etc/shadow \
  -v$(pwd)/.etc/passwd:/etc/passwd \
  -v$(pwd)/.etc/group:/etc/group \
  --rm \
  -it $IMAGE

# EOF
