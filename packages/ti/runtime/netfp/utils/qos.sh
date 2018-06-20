echo "***************************"
echo "L3 QoS Statistics [eth0]"
echo "***************************"
for QUEUE in l3-be-cos0 l3-hp-cos7 l3-wrr-cos5 l3-wrr-cos1 l3-wrr-cos2 l3-wrr-cos3 l3-wrr-cos4 l3-wrr-cos6
do
    PACKET_FORWARDED=`cat /sys/devices/soc.1/hwqueue.8/qos-inputs-1/statistics/$QUEUE/packets_forwarded`
    PACKET_DISCARDED=`cat /sys/devices/soc.1/hwqueue.8/qos-inputs-1/statistics/$QUEUE/packets_discarded`
    echo "L3 eth0 $QUEUE Forwarded $PACKET_FORWARDED Discard $PACKET_DISCARDED"
done

echo "***************************"
echo "L2 QoS Statistics [eth0]"
echo "***************************"

for QUEUE in be-cos0-eth0 hp-cos7-eth0 wrr-cos1-eth0 wrr-cos2-eth0 wrr-cos3-eth0 wrr-cos4-eth0 wrr-cos5-eth0 wrr-cos6-eth0
do
    PACKET_FORWARDED=`cat /sys/devices/soc.1/hwqueue.8/qos-inputs-2/statistics/$QUEUE/packets_forwarded`
    PACKET_DISCARDED=`cat /sys/devices/soc.1/hwqueue.8/qos-inputs-2/statistics/$QUEUE/packets_discarded`
    echo "L2 eth0 $QUEUE Forwarded $PACKET_FORWARDED Discard $PACKET_DISCARDED"
done

echo "***************************"
echo "L2 QoS Statistics [eth1]"
echo "***************************"

for QUEUE in be-cos0-eth1 hp-cos7-eth1 wrr-cos1-eth1 wrr-cos2-eth1 wrr-cos3-eth1 wrr-cos4-eth1 wrr-cos5-eth1 wrr-cos6-eth1
do
    PACKET_FORWARDED=`cat /sys/devices/soc.1/hwqueue.8/qos-inputs-2/statistics/$QUEUE/packets_forwarded`
    PACKET_DISCARDED=`cat /sys/devices/soc.1/hwqueue.8/qos-inputs-2/statistics/$QUEUE/packets_discarded`
    echo "L2 eth1 $QUEUE Forwarded $PACKET_FORWARDED Discard $PACKET_DISCARDED"
done

