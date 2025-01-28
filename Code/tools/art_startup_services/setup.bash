#!/bin/bash

# check if ROS_DISTRO is set
if [ -z ${ROS_DISTRO+x} ]; then
    echo "ROS_DISTRO variable is not set. Please set it to the ROS distro you are using (e.g. iron)"
    exit 1
fi

# add some env variables to /etc/default/race_common
echo "RACE_COMMON_DIR=${PWD}" | sudo tee /etc/default/race_common
echo "ROS_DISTRO=${ROS_DISTRO}" | sudo tee -a /etc/default/race_common
echo "ROSBAG_DIR=${ROSBAG_DIR}" | sudo tee -a /etc/default/race_common


# check if user is root
if [ "$EUID" -eq 0 ]; then
    echo "installing services for root user"

    # create symbolic links for all the services
    for file in ${PWD}/src/tools/art_startup_services/services/*.service; do
        sudo ln -s -f $file /etc/systemd/system/$(basename $file)
    done

    # update the systemd daemon
    systemctl daemon-reload

else
    echo "installing services for user"

    # create a folder for the user if it doesn't exist already
    if [ ! -d "~/.config/systemd/user/" ]; then
        sudo mkdir -p ~/.config/systemd/user
    fi

    # create symbolic links for all the services
    for file in ${PWD}/src/tools/art_startup_services/services/*.service; do
        sudo ln -s -f $file ~/.config/systemd/user/$(basename $file)
    done

    # update the systemd daemon
    systemctl --user daemon-reload

fi