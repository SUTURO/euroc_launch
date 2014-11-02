#!/bin/bash

# Dont use v3 tasks, these are just slow versions of v2 tasks

# Use the following for test evaluation:
rosrun suturo_planning_startup start_complete_demo.py --tasks=task1_v1,task2_v1_1,task2_v1_2,task2_v1_3,task3_v1,task4_v1_1,task4_v1_2,task4_v1_3,task6_v1

# Use the following for final evaluation
# rosrun suturo_planning_startup start_complete_demo.py