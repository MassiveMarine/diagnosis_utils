#!/bin/bash

if [ -z "$SOURCEMETERCPP_PATH" ]; then
  echo "can't perform check if source meter as no path is set";
else
    $SOURCEMETERCPP_PATH/SourceMeterCPP -projectName=catkin_project -buildScript=build.sh -resultsDir=Results -externalSoftFilter=softfilter -resultsDir=/tmp;
fi
