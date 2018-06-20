#########################################################################################
# ICMPv6 Packet Too Big Error message
#
# The script is used to generate fragments in which a destination option header is
# added after the IPv6 Fragment Header
#
# The following lines need to be executed only once in the scapy console.
#########################################################################################
sip = '7000::1'
dip = '7000::2'
conf.route6
conf.route6.ifadd ('eth1', '7000::1/64')
conf.route6.add (dst = dip, dev="eth1")

#########################################################################################
# The code below generates the ICMPv6 Packet Too Big message
#########################################################################################

# Create the outer IP Payload & ICMP Header
outerIPPayload = IPv6(src=sip, dst=dip) / ICMPv6PacketTooBig(mtu=1300)

# Create the inner IP header which caused the error
innerIPPayload  = IPv6(src=dip, dst=sip) / UDP (sport = 50000, dport = 50000)

# Generate the ICMP Packet
packet = Ether(src="00:21:9b:6d:b3:f5",dst="B4:99:4C:91:2A:D8")/outerIPPayload/innerIPPayload

# Send out the packet
sendp(packet, iface="eth1")

