#!/bin/bash

cd ${RACE_COMMON_DIR} && source ${RACE_COMMON_DIR}/src/tools/art_startup_services/launch/base_source.bash

PORT=$1

if [ -z ${PORT+x} ]; then
    echo "PORT variable is not set. Defaulting to 9091"
    PORT=9091
fi

lgsvl_bridge --port ${PORT}