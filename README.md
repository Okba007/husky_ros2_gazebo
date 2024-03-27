# ROS2_husky_ros2_gazebo documentation



## Summary

This repository is part of RAMP project, a mobile base robot equiped with an universal robot arm. Therefore, three ros packages are required to simulate the robot in a ros2 environment, each one is represented by a folder in the root path repository:

- urdf :  robot description husky the robot mobile base (which point to ur_macro.xacro file using empty.urdf)
- config: configuration file for controllers, moveit, gazebo, robot parameters
- launch: gazebo, moveit

## Dependencies
Dependencies should be installed before building this repository otherwise nothing will work. 
The following dependencies are sorted from the lowest software layer to the highest software layer, therefore they have to be installed in the same order as each software is depending on its underlying softwares:

- Ubuntu (recommanded version 22.04 LTS)
- Ros2 (recommanded version humble LTS)
- Moveit package
- Gazebo classic version 11.0

### Dependencies installation:
- Ubuntu: 
Ubuntu is installed by IT.
- Ros2: 
Follow this steps for Ros2 installation: [link](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
- Moveit: 
Execute this command:
```
sudo apt install ros-humble-moveit*
```
- Gazebo: 
Execute this command:
```
sudo apt install Gazebo
```

## Building the project:
- source /opt/ros/humble/setup.bash
- colcon build --mixin release --packages-select husky_ros2_gazebo

## Test moveit planner
- source install/setup.bash
- ros2 launch husky_ros2_gazebo gazebo_moveit.py
## Create Occupancy Map using slam_toolbox
ros2 launch slam_toolbox online_sync_launch.py use_sim_time:=True
## Test NAV2 ###
ros2 launch husky_ros2_gazebo gazebo_moveit.launch.py
ros2 launch husky_ros2_gazebo navigator.launch.py use_sim_time:=True
