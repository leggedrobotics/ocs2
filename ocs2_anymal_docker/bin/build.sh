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
PASSWORD=
USERNAME=

# Parse arguments
for i in "$@"
do
case $i in
  --password=*)
    PASSWORD=${i#*=}
    shift # past argument=value
    ;;
  --user=*)
    USERNAME=${i#*=}
    shift # past argument=value
    ;;
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
    echo "[build.sh]: Error: Unknown argument(s)"
    exit 1
    ;;
esac
done

# Check if credentials have been specified.
if [[ -z "$USERNAME" ]]
then
    echo "[build.sh]: Error: ANYmal-Research PPA user has not been set. Please specify using the --user=USERNAME argument."
    exit 1
fi
if [[ -z "$PASSWORD" ]]
then
    echo "[build.sh]: Error: ANYmal-Research PPA password has not been set. Please specify using the --password=PASSWORD argument."
    exit 1
fi

#==
# Build image
#==

# Credentials from https://wiki.leggedrobotics.com/doku.php?id=platforms:anymal:main
docker build \
  -t "$IMAGE" \
  -f "$PKGROOT/src/$PLATFORM.Dockerfile" \
  --build-arg RELEASE="$RELEASE" \
  --build-arg USERNAME="$USERNAME" \
  --build-arg PASSWORD="$PASSWORD" \
  "$PKGROOT/src"

# EOF
