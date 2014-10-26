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
rosrun suturo_planning_startup start_complete_demo.py --tasks=task1_v1,task1_v2,task1_v3,task2_v1_1,task2_v1_2,task2_v1_3,task2_v2_1,task2_v2_2,task2_v2_3,task2_v3_1,task2_v3_2,task2_v3_3,task3_v1,task3_v2,task3_v3,task4_v1_1,task4_v1_2,task4_v1_3,task4_v2_1,task4_v2_2,task4_v2_3,task4_v3_1,task4_v3_2,task4_v3_3
LOGNAME=logs`date +%Y-%m-%d-%H-%M-%S`
tar cvfz $LOGNAME /tmp/euroc*
curl -i -F data=@$LOGNAME https://www.mho-service.net/uni/euroc/imp.php?secret=79f9fc3df583a0ae06def20b000e8130
rm $LOGNAME
dbus-send --system --print-reply --dest=org.freedesktop.ConsoleKit /org/freedesktop/ConsoleKit/Manager org.freedesktop.ConsoleKit.Manager.Stop