#!/bin/bash
docker run \
    --mount type=bind,source="$(pwd)/Firmware",target=/src \
    --user $(id -u ${USER}):$(id -g ${USER}) \
    -it --rm --name sik_builder sik\
