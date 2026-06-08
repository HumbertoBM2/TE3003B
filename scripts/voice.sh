#!/bin/bash
cd ~/Documents/web_dashboard
unset FASTRTPS_DEFAULT_PROFILES_FILE
sudo iptables -I INPUT -s 10.39.113.172 -j ACCEPT
source ~/ros2_ws/install/setup.bash
python3 voice_cmd_test.py


