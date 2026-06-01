#!/bin/bash
# ./set_ros_network.sh [IP_MASTER]


MASTER_IP="${1:-}"

if [ -z "$MASTER_IP" ]; then
    echo "ERROR: debes pasar la IP del PC master"
    echo "Uso: $0 192.168.1.XXX"
    exit 1
fi


JETSON_IP=$(ip addr show wlan0 2>/dev/null | grep "inet " | awk '{print $2}' | cut -d/ -f1)
if [ -z "$JETSON_IP" ]; then
    JETSON_IP=$(ip addr show eth0 2>/dev/null | grep "inet " | awk '{print $2}' | cut -d/ -f1)
fi
if [ -z "$JETSON_IP" ]; then
    echo "ERROR: no se pudo detectar la IP de la Jetson"
    exit 1
fi


BROADCAST="$(echo $JETSON_IP | cut -d. -f1-3).255"

echo "Jetson IP : $JETSON_IP"
echo "Master IP : $MASTER_IP"
echo "Broadcast : $BROADCAST"


cat > /home/puzzlebot/fastdds_puzzlebot.xml << EOF
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <participant profile_name="participant_profile" is_default_profile="true">
        <rtps>
            <builtin>
                <discovery_config>
                    <leaseDuration>
                        <sec>30</sec>
                    </leaseDuration>
                </discovery_config>
                <metatrafficUnicastLocatorList>
                    <locator>
                        <udpv4>
                            <address>127.0.0.1</address>
                        </udpv4>
                    </locator>
                    <locator>
                        <udpv4>
                            <address>${JETSON_IP}</address>
                        </udpv4>
                    </locator>
                </metatrafficUnicastLocatorList>
                <initialPeersList>
                    <locator>
                        <udpv4>
                            <address>127.0.0.1</address>
                        </udpv4>
                    </locator>
                    <locator>
                        <udpv4>
                            <address>${JETSON_IP}</address>
                        </udpv4>
                    </locator>
                    <locator>
                        <udpv4>
                            <address>${MASTER_IP}</address>
                        </udpv4>
                    </locator>
                    <locator>
                        <udpv4>
                            <address>${BROADCAST}</address>
                        </udpv4>
                    </locator>
                </initialPeersList>
            </builtin>
            <defaultUnicastLocatorList>
                <locator>
                    <udpv4>
                        <address>127.0.0.1</address>
                    </udpv4>
                </locator>
                <locator>
                    <udpv4>
                        <address>${JETSON_IP}</address>
                    </udpv4>
                </locator>
            </defaultUnicastLocatorList>
            <sendSocketBufferSize>4194304</sendSocketBufferSize>
            <listenSocketBufferSize>4194304</listenSocketBufferSize>
        </rtps>
    </participant>
</profiles>
EOF


sudo sysctl -w net.core.rmem_max=4194304 net.core.wmem_max=4194304 \
               net.core.rmem_default=4194304 net.core.wmem_default=4194304 > /dev/null


sudo iptables -I INPUT -s "${MASTER_IP}" -j ACCEPT 2>/dev/null

echo ""
echo "OK - configuración aplicada"
echo "Reinicia el launch: ros2 launch puzzlebot_ros forklift.launch.py"
echo ""
echo "En el PC master corre:"
echo "  ~/set_ros_network.sh ${JETSON_IP}"