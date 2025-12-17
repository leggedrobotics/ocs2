#!/bin/bash
# please login docker hub before run this script

IMAGE_NAME="ros2_ocs2"
IMAGE_TAG="latest"
IMAGE_REGISTRY="ecgfiscbuildserver1.sh.intel.com:5000"

docker build -t ${IMAGE_NAME}:${IMAGE_TAG} .
docker tag ${IMAGE_NAME}:${IMAGE_TAG} ${IMAGE_REGISTRY}/${IMAGE_NAME}:${IMAGE_TAG}
docker push ${IMAGE_REGISTRY}/${IMAGE_NAME}:${IMAGE_TAG}
