#!/bin/bash

xhost +
docker run -it --rm \
    --gpus all \
    --network=host \
    --privileged \
    -v "/dev:/dev" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix" \
    -v "/home/`whoami`/Data/glider:/home/`whoami`/data" \
    -v "`pwd`/../clipper/cpp:/home/`whoami`/clipper" \
    -v "`pwd`/../glider/glider:/home/`whoami`/glider" \
    -v "`pwd`/../scenic:/home/`whoami`/scenic" \
    -v "`pwd`/../../scenic-ros:/home/`whoami`/ws/src/scenic-ros" \
    -v "`pwd`/../../scenic-msgs:/home/`whoami`/ws/src/scenic-msgs" \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY=$XAUTH \
    --name scenic-dev\
    scenic:dev \
    bash
xhost -
