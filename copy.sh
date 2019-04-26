#!/usr/bin/env bash

#
# This is a script to copy source code to our robot
# Example usage: ./copy.sh wanderer.cpp
#

# Check args
if [ "$#" -ne 1 ]; then
  echo "usage: ./copy.sh FILE"
  return 1
fi

scp $0 gold@192.168.0.100:~/catkin_ws/src/robfun_team_gold/src/