#!/bin/bash

source /opt/ros/noetic/setup.bash
source /home/user/robohead_ws/devel/setup.bash

export PYTHONPATH=$PYTHONPATH:$ROS_ROOT/core/roslib/src
export PYTHONPATH=$PYTHONPATH:/usr/bin/python3
export PYTHONPATH=$PYTHONPATH:/home/user/.local/lib/python3.8/site-packages

pactl set-default-sink 'alsa_output.usb-SEEED_ReSpeaker_4_Mic_Array__UAC1.0_-00.analog-stereo'
pulseaudio -k
pulseaudio --start -D

# for work with MORS
#export ROS_MASTER_URI=http://10.42.0.1:11311
#export ROS_HOSTNAME=10.42.0.154

# for local work (without MORS)
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_HOSTNAME=127.0.0.1

roslaunch head_controller head_controller.launch useMors:=0

