#!/bin/bash
cd ~/Documents/web_dashboard
sudo iptables -I INPUT -s 10.39.113.172 -j ACCEPT
unset FASTRTPS_DEFAULT_PROFILES_FILE
source ~/ros2_ws/install/setup.bash
python3 yolo_master.py --model ~/Downloads/best.pt --hz 5 --imgsz 640


