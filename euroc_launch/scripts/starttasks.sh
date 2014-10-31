#!/bin/bash
#rosrun suturo_planning_startup start_complete_demo.py --tasks=task1_v1,task1_v2,task1_v3,task2_v1_1,task2_v1_2,task2_v1_3,task2_v2_1,task2_v2_2,task2_v2_3,task2_v3_1,task2_v3_2,task2_v3_3,task3_v1,task3_v2,task3_v3,task4_v1_1,task4_v1_2,task4_v1_3,task4_v2_1,task4_v2_2,task4_v2_3,task4_v3_1,task4_v3_2,task4_v3_3
#rosrun suturo_planning_startup start_complete_demo.py --tasks=task1_v1,task2_v1_1,task3_v1,task4_v1_1

# Dont use v3 tasks, these are just slow versions of v2 tasks
# DO NOT change, final test version:
rosrun suturo_planning_startup start_complete_demo.py --tasks=task1_v1,task2_v1_1,task2_v1_2,task2_v1_3,task3_v1,task4_v1_1,task4_v1_2,task4_v1_3,task6_v1