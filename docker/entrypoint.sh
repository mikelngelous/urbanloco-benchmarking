#!/bin/bash
set -e

# Source ROS setup
source /opt/ros/noetic/setup.bash

# Source workspace if it exists
if [ -f $CATKIN_WS/devel/setup.bash ]; then
    source $CATKIN_WS/devel/setup.bash
fi

# Execute the command
exec "$@"