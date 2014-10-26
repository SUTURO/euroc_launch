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
rosrun suturo_planning_startup start_complete_demo.py --tasks=task1_v1
LOGNAME=logs`date +%Y-%m-%d-%H-%M-%S`
tar cvfz $LOGNAME /tmp/euroc*
curl -i -F data=@$LOGNAME https://www.mho-service.net/uni/euroc/imp.php?secret=79f9fc3df583a0ae06def20b000e8130
rm $LOGNAME
dbus-send --system --print-reply --dest=org.freedesktop.ConsoleKit /org/freedesktop/ConsoleKit/Manager org.freedesktop.ConsoleKit.Manager.Stop