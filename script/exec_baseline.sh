#!/usr/bin/sh
xhost +
docker start basel
docker exec -it basel /bin/bash
