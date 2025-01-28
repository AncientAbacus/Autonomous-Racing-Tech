#!/bin/bash

cd ${RACE_COMMON_DIR} && source ${RACE_COMMON_DIR}/src/tools/art_startup_services/launch/base_source.bash
# cd ${ROSBAG_DIR}/11a27bf6-f198-4f2a-9e36-b0f7fba39b23
cd ${ROSBAG_DIR}/${CAR_NAME}
$ ros2 bag record -s mcap -a