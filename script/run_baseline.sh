#!/usr/bin/sh
docker rm -f basel

docker run -it --name basel --network=host \
    --gpus all \
    --privileged \
    --device=/dev/ttyUSB0 \
    --device=/dev/input/js0 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v ~/vscode-server:/root/.vscode-server \
    -e DISPLAY=$DISPLAY \
    -v $HOME/Desktop/shared:/shared \
    dedsecer/baseline:v1 \
    /bin/bash
