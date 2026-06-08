#!/bin/bash
# Uso: ./set_ros_network.sh [IP_JETSON]
JETSON_IP="${1:-}"
if [ -z "$JETSON_IP" ]; then echo "Uso: $0 192.168.1.XXX"; exit 1; fi
MASTER_IP=$(ip route get 8.8.8.8 2>/dev/null | grep -oP 'src \K\S+')
echo "Master: $MASTER_IP  |  Jetson: $JETSON_IP"
cat > ~/fastdds_master.xml << EOF
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <participant profile_name="participant_profile" is_default_profile="true">
        <rtps>
            <builtin>
                <discovery_config>
                    <leaseDuration><sec>DURATION_INFINITY</sec></leaseDuration>
                </discovery_config>
                <initialPeersList>
                    <locator><udpv4><address>${JETSON_IP}</address></udpv4></locator>
                    <locator><udpv4><address>192.168.$(echo $JETSON_IP | cut -d. -f3).255</address></udpv4></locator>
                </initialPeersList>
            </builtin>
            <sendSocketBufferSize>4194304</sendSocketBufferSize>
            <listenSocketBufferSize>4194304</listenSocketBufferSize>
        </rtps>
    </participant>
</profiles>
EOF
sudo sysctl -w net.core.rmem_max=4194304 net.core.wmem_max=4194304 \
               net.core.rmem_default=4194304 net.core.wmem_default=4194304 > /dev/null
sudo iptables -I INPUT -s "${JETSON_IP}" -j ACCEPT
source ~/.bashrc
echo "OK - run: ros2 topic list"
