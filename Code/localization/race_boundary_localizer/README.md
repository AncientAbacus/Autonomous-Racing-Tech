# Race Boundary Localizer

##### Compile
```
cd <ros2_ws>
make localization
```

##### Source
```
source install/setup.bash
```

##### Visualize
```
rviz2 -d src/localization/race_boundary_localizer/rviz/race_boundary_localizer.rviz
ros2 bag play rosbag2_2023_01_04-21_41_38/ -l -s mcap -r 3.0 --clock
ros2 launch autoware_perception_launch lidar_concatenation.launch.py 
```

##### Deploy perception and localization modules
```
ros2 launch wall_boundary_segmentation wall_boundary_segmentation.launch.py
ros2 launch race_boundary_localizer race_boundary_localizer.launch.py
```