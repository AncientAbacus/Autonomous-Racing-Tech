#!/bin/bash

cd ${RACE_COMMON_DIR} && source ${RACE_COMMON_DIR}/src/art_startup_services/launch/base_source.bash
ros2 launch drivers_bringup novatel_interface.launch.py