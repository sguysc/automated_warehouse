#!/bin/bash

#source /opt/ros/kinetic/setup.bash
source mocap/devel/setup.bash

export ROS_MASTER_URI=http://jackal1:11311/
export ROS_IP=199.168.1.108

roslaunch mocap_optitrack mocap.launch
