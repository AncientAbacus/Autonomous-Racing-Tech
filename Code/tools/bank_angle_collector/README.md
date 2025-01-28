# Bank Angle Collector

## Purpose of Package

The bank angle collector node fills the banking angle of a TTL based on orientation in vehicle kinematic state message.

## Install

The package depends on `race_msgs` and `tf_transformations` packages. The former needs to be built, and the latter can be installed with

```bash
sudo apt install ros-$ROS_DISTRO-tf-transformations
sudo pip3 install transforms3d
```

Install [`trajectory_tools`](https://github.com/HaoruXue/offline-trajectory-tools).

Change the TTL parameter in the launch file `bank_angle_collector.launch.py` to match with the input and output TTL file name.

Now build the package.

```bash
colcon build --packages-select bank_angle_collector
```

## How to Run

Have the car running stably on a TTL at a constant speed. The speed should be low enough for this Python node to process, say 10 waypoints per second. For example, if the TTL waypoint interval is 0.8 meter, then the recommanded speed is 8 meter per second.

You can also replay a ROS bag and specify the replay rate to match this speed.

Now launch the node.

```bash
ros2 launch bank_angle_collector bank_angle_collector.launch.py
```

After all trajectory points' banking angle fields are filled, the node saves the new TTL and exits.

## Future improvements

Sometimes a few trajectory points will never be visited. We could find a way to early stop the node and interpolate the last few missing banking angles.