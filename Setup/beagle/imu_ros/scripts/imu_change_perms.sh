#!/bin/sh
# 2017-11-29 LLW shell script for changing ownership and sticky bit for 
# imu_ros usage: ~/bin/imu_change_perms.sh
#
echo ls -l ~/catkin_ws/devel/lib/imu_ros/imu_ros 
ls -l ~/catkin_ws/devel/lib/imu_ros/imu_ros 

echo sudo chown root:root ~/catkin_ws/devel/lib/imu_ros/imu_ros 
sudo chown root:root ~/catkin_ws/devel/lib/imu_ros/imu_ros 

echo sudo chmod u+s ~/catkin_ws/devel/lib/imu_ros/imu_ros 
sudo chmod u+s ~/catkin_ws/devel/lib/imu_ros/imu_ros 

echo ls -l ~/catkin_ws/devel/lib/imu_ros/imu_ros 
ls -l ~/catkin_ws/devel/lib/imu_ros/imu_ros
