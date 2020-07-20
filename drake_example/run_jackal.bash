#!/bin/bash

#source /opt/ros/kinetic/setup.bash
#source mocap/devel/setup.bash

#export ROS_MASTER_URI=http://jackal1:11311
#export ROS_HOSTNAME=localhost
#export ROS_IP=199.168.1.108

if [ $1 -eq 1 ]
then
	./Jackal.py --i $1 --n 2 --robots jackal3,jackal1 #--obs hat1_1
else
	./Jackal.py --i $1 --n 2 --robots jackal3,jackal1 #--obs hat1_1
fi

