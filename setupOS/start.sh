#!/bin/bash

source /opt/ros/noetic/setup.bash
source /home/pi/robohead_ws/devel/setup.bash

export PYTHONPATH=$PYTHONPATH:$ROS_ROOT/core/roslib/src
export PYTHONPATH=$PYTHONPATH:/usr/bin/python3
export PYTHONPATH=$PYTHONPATH:/home/pi/.local/lib/python3.8/site-packages

# for work with another robot
# ip of another robot:
# export ROS_MASTER_URI=http://10.42.0.1:11311 
# ip of robohead:
# export ROS_HOSTNAME=10.42.0.154

# for local work (without a robot)
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_HOSTNAME=127.0.0.1

roslaunch robohead_controller robohead_controller_py.launch

