#!/bin/bash
# Team: BabyTigers, Color: CYAN, Robot Number: 1
sleep 2
pushd ~/catkin_ws/src/ros-rcll_refbox_peer/launch
roslaunch rcll_refbox_peer.launch num_robots:=1 team_name:=BabyTigers robot_name:=kotora robot_number:=1 crypto_key:=randomkey
popd
