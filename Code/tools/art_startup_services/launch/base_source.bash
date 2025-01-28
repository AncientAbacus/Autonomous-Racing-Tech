#!/bin/bash
source /opt/ros/${ROS_DISTRO}/setup.bash
source ${RACE_COMMON_DIR}/install/setup.bash
source ${RACE_COMMON_DIR}/race.env
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
if [ -f "/etc/cyclone/cyclonedds.xml" ]; then
    export CYCLONEDDS_URI=file:///etc/cyclone/cyclonedds.xml
else
    echo "Could not find file:///etc/cyclone/cyclonedds.xml"
fi
export CYCLONE_INCLUDE=/opt/ros/${ROS_DISTRO}/include
export CYCLONE_LIB=/opt/ros/${ROS_DISTRO}/lib/x86_64-linux-gnu
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_STATIC_PEERS=""