#!/bin/bash
echo
echo Starting EuRoC stack
echo 
ROOT=~/
source /opt/ros/hydro/setup.bash
source /opt/euroc_c2s1/ros/install/setup.bash
source $ROOT/euroc_ws/devel/setup.bash
mkdir -p /tmp/euroc_c2
cd $ROOT/euroc_ws/src/euroc_msgs
MSGSTATE=`git log | head -n1`
cd $ROOT/euroc_ws
echo -n Updating...
wstool update 2>1 >/tmp/euroc_c2/update.log
echo " done"
cd $ROOT/euroc_ws/src/euroc_msgs
if [ "$MSGSTATE" != "`git log | head -n1`" ]
	then rm -rf $ROOT/euroc_ws/devel $ROOT/euroc_ws/build
fi
cd $ROOT/euroc_ws
echo -n Building...
catkin_make 2>1 >/tmp/euroc_c2/build.log
echo " done"
echo -n Executing tasks...
rosrun euroc_launch starttasks.sh 2>1 >/tmp/euroc_c2/starttasks.log
echo " done"
echo -n Gathering and uploading logs...
echo "###Perception GIT" >>/tmp/euroc_c2/git.log
cd $ROOT/euroc_ws/src/euroc_perception
git log -n1 >>/tmp/euroc_c2/git.log
echo "###Manipulation GIT" >>/tmp/euroc_c2/git.log
cd $ROOT/euroc_ws/src/euroc_manipulation
git log -n1 >>/tmp/euroc_c2/git.log
echo "###Planning GIT" >>/tmp/euroc_c2/git.log
cd $ROOT/euroc_ws/src/euroc_planning
git log -n1 >>/tmp/euroc_c2/git.log
echo "###Msgs GIT" >>/tmp/euroc_c2/git.log
cd $ROOT/euroc_ws/src/euroc_msgs
git log -n1 >>/tmp/euroc_c2/git.log
echo "###Launch GIT" >>/tmp/euroc_c2/git.log
cd $ROOT/euroc_ws/src/euroc_launch
git log -n1 >>/tmp/euroc_c2/git.log
cd $ROOT
LOGNAME=logs`date +%Y-%m-%d-%H-%M-%S`
tar cvfz $LOGNAME /tmp/euroc*
curl -i -F data=@$LOGNAME https://www.mho-service.net/uni/euroc/imp.php?secret=79f9fc3df583a0ae06def20b000e8130
rm $LOGNAME
echo " done"
echo Terminating in 10 seconds!
sleep 10
dbus-send --system --print-reply --dest=org.freedesktop.ConsoleKit /org/freedesktop/ConsoleKit/Manager org.freedesktop.ConsoleKit.Manager.Stop