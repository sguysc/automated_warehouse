#!/usr/bin/env bash

source /opt/ros/kinetic/setup.bash
title gazebo
roslaunch turtlebot_gazebo turtlebot_world.launch gui:=true
