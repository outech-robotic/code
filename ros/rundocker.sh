#!/bin/bash

set -e
docker build -t outech-ros "$(git rev-parse --show-toplevel)"/ros
docker run \
  --name outech \
  --publish-all \
  -it \
  -v "$(git rev-parse --show-toplevel)"/:/outech/ \
  --rm \
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  outech-ros
