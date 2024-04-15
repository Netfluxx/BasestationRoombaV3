#!/bin/bash
echo Sourcing ROS2
source /opt/ros/humble/setup.bash
source install/setup.bash
echo Launching Rviz2
ros2 launch microver rsp.launch.py
rvi2