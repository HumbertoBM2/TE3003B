Voice HMM

Jeston

T1
```
./set_ros_network.sh <IP_MASTER>
unset FASTRTPS_DEFAULT_PROFILES_FILE
source ~/ros2_ws/install/setup.bash
sudo systemctl restart nvargus-daemon
ros2 launch puzzlebot_ros navigation.launch.py
```


Master

T1
```
sudo iptables -I INPUT -s IP_Jetson -j ACCEPT
unset FASTRTPS_DEFAULT_PROFILES_FILE
source ~/ros2_ws/install/setup.bash
cd ~/Documents/web_dashboard/
python3 dashboard.py
```

T2
```
sudo iptables -I INPUT -s IP_Jetson -j ACCEPT
unset FASTRTPS_DEFAULT_PROFILES_FILE
source ~/ros2_ws/install/setup.bash
cd ~/Documents/web_dashboard/
python3 yolo_master.py --model ~/Downloads/best.pt --hz 5 --imgsz 640
```


T3
```
sudo iptables -I INPUT -s IP_Jetson -j ACCEPT
unset FASTRTPS_DEFAULT_PROFILES_FILE
cd ~/Documents/reto_voz
colcon build
source ~/ros2_ws/install/setup.bash
ros2 launch voice_hmm launch_voice_hmm.py
```

T4
```
sudo iptables -I INPUT -s IP_Jetson -j ACCEPT
unset FASTRTPS_DEFAULT_PROFILES_FILE
source ~/ros2_ws/install/setup.bash
cd ~/Documents/web_dashboard
python3 voice_cmd_test.py
```