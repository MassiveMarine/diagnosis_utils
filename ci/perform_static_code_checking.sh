#!/bin/bash
source ./devel/setup.bash
rm build/ -r
rm devel/ -r
cd src/utils/tug_static_code_checking_helper/scripts
export GCONV_PATH="/usr/lib/x86_64-linux-gnu/gconv"
export SOURCEMETERCPP_PATH="/home/robtest/Downloads/SourceMeter-8.1.0-x64-linux/CPP"
./analyze.sh
