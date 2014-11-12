#!/bin/bash

NODENAME=TAGNAME

echo
echo Starting EuRoC stack
echo 
ROOT=~/
source /opt/ros/hydro/setup.bash
source /opt/euroc_c2s1/ros/install/setup.bash
source $ROOT/euroc_ws/devel/setup.bash
mkdir -p /tmp/euroc_c2

echo -n Executing tasks...
rosrun suturo_planning_startup start_complete_demo.py 2>1 >/tmp/euroc_c2/starttasks.log
echo " done"
LOGNAME=logs`date +%Y-%m-%d-%H-%M-%S`
tar cvfz $LOGNAME /tmp/euroc* 2>1 >/dev/null
curl -i -F data=@$LOGNAME https://www.mho-service.net/uni/euroc/imp.php?secret=79f9fc3df583a0ae06def20b000e8130\&name=$NODENAME 2>1 >/dev/null
rm $LOGNAME
echo " done"
echo Terminating in 10 seconds!
sleep 10
dbus-send --system --print-reply --dest=org.freedesktop.ConsoleKit /org/freedesktop/ConsoleKit/Manager org.freedesktop.ConsoleKit.Manager.Stop