#!/bin/bash

# Utility script to send a goal position to the drone easily
# Usage: ./send_goal.sh X Y Z

if [ "$#" -ne 3 ]; then
    echo "Usage: $0 X Y Z"
    echo "Example: $0 1.0 -1.0 1.5"
    exit 1
fi

X=$1
Y=$2
Z=$3

echo "Sending drone to X: $X, Y: $Y, Z: $Z"

ros2 topic pub --once /goal geometry_msgs/PoseStamped "{pose: {position: {x: $X, y: $Y, z: $Z}}}"
