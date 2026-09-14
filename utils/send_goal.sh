#!/bin/bash

# Publishes a target coordinate to the /goal topic via ROS 2.
# Usage: ./send_goal.sh X Y Z [YAW_DEGREES]


if [ "$#" -lt 3 ]; then
    echo "Usage: $0 X Y Z [YAW_DEGREES]"
    echo "Example: $0 1.0 -1.0 1.5 90"
    exit 1
fi

X=$1
Y=$2
Z=$3
YAW_DEG=${4:-0}

# Convert Yaw degrees to Quaternion (Z and W) using python
QUAT=$(python3 -c "
import math
yaw = math.radians($YAW_DEG)
z = math.sin(yaw / 2.0)
w = math.cos(yaw / 2.0)
print(f'x: 0.0, y: 0.0, z: {z:.6f}, w: {w:.6f}')
")

echo "Sending drone to X: $X, Y: $Y, Z: $Z, Yaw: ${YAW_DEG}°"

ros2 topic pub --once /goal geometry_msgs/msg/PoseStamped "{pose: {position: {x: $X, y: $Y, z: $Z}, orientation: {$QUAT}}}"
