#!/bin/bash

source /opt/ros/melodic/setup.bash
source ~/ros_catkin_ws/devel/setup.bash

ROS_PACKAGE_PATH=~/ros_catkin_ws/src:$ROS_PACKAGE_PATH
ROS_WORKSPACE=~/ros_catkin_ws/src

# dumbpi_controller
echo "dumbpi_controller"
{
sudo su -c ~/ros_catkin_ws/src/dumbpi/run_controller.sh 
}&
sleep 1s

# dumbpi_keyboard
echo "dumbpi_keyboard"
rosrun dumbpi keyboard
