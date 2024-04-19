#!/bin/bash
echo Sourcing ROS2
cd ../../../
source /opt/ros/humble/setup.bash
source install/setup.bash
echo Running Launch File in  a new window
gnome-terminal -- ros2 launch microver rsp.launch.py use_sim_time:=true
sleep 1s
cd src/microver
echo Launching RVIZ2, Gazebo and Spawning Microver
gnome-terminal -- rviz2
sleep 1s
gnome-terminal -- ros2 launch gazebo_ros gazebo.launch.py
sleep 1s
gnome-terminal -- ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity microver
