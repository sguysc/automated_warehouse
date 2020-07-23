#!/bin/bash

#source /opt/ros/kinetic/setup.bash
#source mocap/devel/setup.bash

#export ROS_MASTER_URI=http://jackal1:11311
#export ROS_HOSTNAME=localhost
#export ROS_IP=199.168.1.108

#if [ $1 -eq 1 ]
#then
#	./Jackal.py --i $1 --n 2 --robots jackal1,jackal3 --obs Helmet
#	#./Jackal.py --i 1 --n 1 --robots jackal1 --obs Helmet
#else
#	./Jackal.py --i $1 --n 2 --robots jackal1,jackal3 --obs Helmet
#	#./Jackal.py --i 1 --n 2 --robots jackal3,jackal2 --obs hat1_1
#fi

#./Jackal.py --i $1 --n 2 --robots jackal1,jackal3 #--obs Helmet
./Jackal.py --i $1 --n 2 --robots jackal2,jackal1 --obs Helmet