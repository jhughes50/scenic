#!/bin/bash

xhost +
docker run -it --rm \
    --gpus all \
    --network=host \
    --privileged \
    -v "/dev:/dev" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix" \
    -v "./scripts:/home/`whoami`/scripts" \
    -v "./clipper:/home/`whoami`/clipper" \
    -v "./scenic:/home/`whoami`/scenic" \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY=$XAUTH \
    --name lang-embeddings-dev\
    lang-embedding:dev \
    bash
xhost -
