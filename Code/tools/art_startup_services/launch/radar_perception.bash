#!/bin/bash

cd ${RACE_COMMON_DIR} && source ${RACE_COMMON_DIR}/src/tools/art_startup_services/launch/base_source.bash
ros2 launch radar2cam dist2wall_radar_detection_launch.py
