#!/usr/bin/env bash

source /opt/ros/humble/setup.bash
source "$HOME/8vo/reto_voz/install/setup.bash"

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1
unset FASTRTPS_DEFAULT_PROFILES_FILE
unset RMW_IMPLEMENTATION
