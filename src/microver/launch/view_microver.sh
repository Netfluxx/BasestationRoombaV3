#!/bin/bash
echo Sourcing ROS2
cd ../../../
source /opt/ros/humble/setup.bash
source install/setup.bash
echo Running Launch File in  a new window
gnome-terminal -- ros2 launch microver rsp.launch.py
sleep 1s
cd src/microver
echo Launching RVIZ2
rviz2
