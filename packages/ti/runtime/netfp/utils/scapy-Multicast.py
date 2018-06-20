#########################################################################################
# Multicast Packets
#
# The script is used to generate multicast packets.
#########################################################################################

a=Ether(dst = "01:00:5e:0:64:02", src="00:21:9b:6d:b3:f5")/IP(dst="227.0.100.2", src="192.168.1.1", ttl=1)/UDP(dport=5000, sport=5000)/"Hello world"
sendp(a, iface="eth1")

a=Ether(dst = "01:00:5e:00:00:01", src="00:21:9b:6d:b3:f5")/IP(dst="224.0.0.1", src="192.168.1.1", ttl=1)/UDP(dport=5001, sport=5000)/"Hello world"
sendp(a, iface="eth1")

a=Ether(dst = "FF:FF:FF:FF:FF:FF", src="00:21:9b:6d:b3:f5")/IP(dst="192.168.1.255", src="192.168.1.1", ttl=1)/UDP(dport=5000, sport=5000)/"Hello world"
sendp(a, iface="eth1")

