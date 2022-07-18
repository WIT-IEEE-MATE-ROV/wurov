#!/bin/bash
# Only work with the Lenovo surface station atm

# Must install 'expect' first (Ubuntu 20.04): sudo apt-get install expect



gnome-terminal -- bash -c roscore
gnome-terminal -- bash -c "
    export ROS_MASTER_URI=http://10.0.10.101:113111/; 
    export ROS_IP=10.0.10.101; 
    source ~/catkin_ws/devel/setup.bash; 
    exec bash"

gnome-terminal -- bash -c "
    ./ssh_rov.sh; 
    export ROS_MASTER_URI=http://10.0.10.101:113111/; 
    export ROS_IP=10.0.10.100; 
    source ~catkin_ws/devel/setup.bash; 
    exec bash"