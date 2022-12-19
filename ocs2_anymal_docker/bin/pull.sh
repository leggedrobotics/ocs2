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
    IMAGE=ocs2/ocs2-anymal:$RELEASE-$PLATFORM
    shift # past argument=value
    ;;
  *)
    echo "[push.sh]: Error: Unknown argument(s)"
    exit 1
    ;;
esac
done

#==
# Update hosted image
#==

# Launch a container from the prebuilt image.
docker pull registry.leggedrobotics.com/"$IMAGE"

# EOF
