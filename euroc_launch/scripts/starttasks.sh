#!/bin/bash

if [ "$TASKS" == "" ]
	then TASKS=task1_v1,task2_v1_1,task2_v1_2,task2_v1_3,task3_v1,task4_v1_1,task4_v1_2,task4_v1_3,task6_v1,task1_v2,task2_v2_1,task2_v2_2,task2_v2_3,task3_v2,task4_v2_1,task4_v2_2,task4_v2_3,task6_v2
fi
if [ "$TIMEOUT" == "" ]
	then TIMEOUT=6h
fi

timeout --foreground -s SIGKILL $TIMEOUT rosrun suturo_planning_startup start_complete_demo.py --tasks=$TASKS
if [ "$?" == "124" ]
	then echo \#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#
	echo \#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#
	echo \#\#\#\# Execution of start_complete_demo.py aborted by timeout. \#\#\#\#
	echo \#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#
	echo \#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#\#
fi

# Use the following for final evaluation
# rosrun suturo_planning_startup start_complete_demo.py