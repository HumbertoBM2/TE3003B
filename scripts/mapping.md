Mapping

Jeston

T1
```
./set_ros_network.sh <IP_MASTER>
unset FASTRTPS_DEFAULT_PROFILES_FILE
source ~/ros2_ws/install/setup.bash
sudo systemctl restart nvargus-daemon
ros2 launch puzzlebot_ros mapping.launch.py
```


T2
```
unset FASTRTPS_DEFAULT_PROFILES_FILE
source ~/ros2_ws/install/setup.bash
./teleop.sh
```


T3
```
unset FASTRTPS_DEFAULT_PROFILES_FILE
source ~/ros2_ws/install/setup.bash
ros2 service call /map/clear std_srvs/srv/Trigger {}

ros2 service call /map/save std_srvs/srv/Trigger {}
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