#!/bin/sh
roslaunch rosbridge_server rosbridge_websocket.launch &
roslaunch reemc_pose_imitation reemc_pose_imitation_test.launch &