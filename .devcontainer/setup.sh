#!/bin/bash

git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git ~/enre467_ws/src/Universal_Robots_ROS_Driver
git clone -b noetic-devel https://github.com/ros-industrial/universal_robot.git ~/enre467_ws/src/universal_robot

cd ~/enre467_ws

sudo apt-get update && rosdep update --include-eol-distros
rosdep install --from-paths src --ignore-src -y
catkin build
source devel/setup.bash

echo "if [ -f ~/enre467_ws/devel/setup.bash ]; then source ~/enre467_ws/devel/setup.bash; fi" >> ~/.bashrc