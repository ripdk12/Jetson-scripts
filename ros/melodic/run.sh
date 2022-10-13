#!/bin/bash
NAME=ros_test # replace by the name of your image
TAG=melodic-desktop-full # the tag of your built image

mkdir -p source

mkdir -p lmao
 
# create a shared volume to store the ros_ws
docker volume create --driver local \
    --opt type="none" \
    --opt device="${PWD}/source/" \
    --opt o="bind" \
    "${NAME}_src_vol"

docker volume create --driver local \
    --opt type="none" \
    --opt device="${PWD}/lmao/" \
    --opt o="bind" \
    "${NAME}_lmao_vol"


xhost +
docker run \
    --net=host \
    -it \
    --rm \
    --privileged \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/dev:/dev" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="${NAME}_src_vol:/home/ros/catkin_ws/src/:rw" \
    --volume="${NAME}_lmao_vol:/home/ros/src/lmao:rw" \
    "${NAME}:${TAG}"
