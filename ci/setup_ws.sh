#!/bin/bash
echo "will source '/opt/ros/indigo/setup.bash' and set ROS_MASTER/ROS_IP"
source /opt/ros/indigo/setup.sh
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1

REPO_PATH=`pwd` 
TEMP_PATH=`mktemp -d`
echo "will work in '"$TEMP_PATH"'"

mkdir $TEMP_PATH/src
cd $TEMP_PATH/src
catkin_init_workspace
cd ..
catkin_make

source ./devel/setup.sh

ln -s $REPO_PATH ./src/

rm build/ devel/ -r
mkdir build
cd build
cmake ../src -DCMAKE_INSTALL_PREFIX=../install -DCATKIN_DEVEL_PREFIX=../devel
cd ..
source ./devel/setup.sh
rm build/ devel/ -r

echo "ROS_PACKAGE_PATH:"
echo $ROS_PACKAGE_PATH
