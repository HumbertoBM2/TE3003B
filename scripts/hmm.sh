#!/bin/bash
cd ~/Documents/reto_voz
unset FASTRTPS_DEFAULT_PROFILES_FILE
sudo iptables -I INPUT -s 10.39.113.172 -j ACCEPT
colcon build
source ~/Documents/reto_voz/install/setup.bash
ros2 launch voice_hmm launch_voice_hmm.py


