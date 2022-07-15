#!/bin/bash
# Only work with the Lenovo surface station atm

# Must install 'expect' first (Ubuntu 20.04): sudo apt-get install expect



gnome-terminal --window-with-profile=roscore -- roscore
gnome-terminal --window-with-profile=surface_station -e "export ROS_MASTER_URI=http://10.0.10.101:113111/; export ROS_IP=10.0.10.101; source ~catkin_ws/devel/setup.bash; exec bash"
gnome-terminal --window-with-profile=rov -e " 
export ROS_MASTER_URI=http://10.0.10.101:113111/; export ROS_IP=10.0.10.101; source ~catkin_ws/devel/setup.bash; exec bash"