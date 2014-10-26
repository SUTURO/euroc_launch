#!/bin/bash
echo
echo Starting EuRoC stack
echo 
ROOT=~/
source /opt/ros/hydro/setup.bash
source /opt/euroc_c2s1/ros/install/setup.bash
source $ROOT/euroc_ws/devel/setup.bash
cd $ROOT/euroc_ws
wstool update
catkin_make
if [ "$1" == "--with-server" ] then
	rosrun euroc_launch install_integrationtest
fi