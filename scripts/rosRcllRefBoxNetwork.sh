#!/bin/bash
# Team: BabyTigers, Color: CYAN, Robot Number: 1
sleep 2
pushd ~/catkin_ws/src/ros-rcll_refbox_peer/launch
roslaunch rcll_refbox_peer.launch num_robots:=1 team_name:=BabyTigers robot_name:=kotora robot_number:=1 crypto_key:=randomkey peer_address:=192.168.255.255 peer_public_send_port:=4444 peer_public_recv_port:=4444 peer_cyan_send_port:=4441 peer_cyan_recv_port:=4441 peer_magenta_send_port:=4442 peer_magenta_recv_port:=4442
popd
