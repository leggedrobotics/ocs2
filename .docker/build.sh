#!/bin/bash
# please login docker hub before run this script

IMAGE_NAME="ros2_ocs2"
IMAGE_TAG="latest"
read -p "Enter your Docker Hub username: " DOCKER_USERNAME
echo 

docker build -t ${IMAGE_NAME}:${IMAGE_TAG} .
docker tag ${IMAGE_NAME}:${IMAGE_TAG} ${DOCKER_USERNAME}/${IMAGE_NAME}:${IMAGE_TAG}
docker push ${DOCKER_USERNAME}/${IMAGE_NAME}:${IMAGE_TAG}
