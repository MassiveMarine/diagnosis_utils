#!/bin/bash

the_path=$(echo $ROS_PACKAGE_PATH | awk -F: '{print $1}')
source $the_path/../devel/setup.sh
cd $the_path/../
catkin_make
