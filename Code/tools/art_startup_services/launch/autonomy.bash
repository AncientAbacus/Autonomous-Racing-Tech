#!/bin/bash

cd ${RACE_COMMON_DIR} && source ${RACE_COMMON_DIR}/src/tools/art_startup_services/launch/base_source.bash
ros2 launch autonomy_launch autonomy_all.launch.py