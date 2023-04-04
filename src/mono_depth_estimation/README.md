# Build
To build the ROS2 node, run 
```
colcon build --packages-select mono_depth_estimation deepracer_interfaces_pkg
```

Launch using the launch file (from the root of the repository):
```
ros2 launch mono_depth_estimation mono_depth_estimation_launch.py
```