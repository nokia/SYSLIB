################################################################
# Overlapping fragments (1)
################################################################

dip      = "192.168.1.2"
payload1 = "AABBAABB"
payload2 = "BBAABBAA"
payload3 = "CCCCCCCC"

# Uncomment the line below to generate an IPv4 packet with the header length
# set to 0. This can be used to verify the reassembly operation with invalid
# IPv4 header
#ip=IP(ihl = 0,dst=dip,proto=1,id=12345,flags=1)

# Uncomment the line below to generate an IPv4 packet with an invalid checksum
# This can be used to verify the reassembly operation with invalid IPv4 header
#ip=IP(dst=dip,proto=1,id=12345,flags=1, chksum=0xdead)

# Uncomment the line below to generate an IPv4 packet with an invalid IP version
#ip=IP(version=7,dst=dip,proto=1,id=12345,flags=1)

ip=IP(dst=dip,proto=1,id=12345,flags=1)
icmp=ICMP(type=8,code=0,chksum=0xe3eb)
packet=ip/icmp/payload1
send(packet)

ip=IP(dst=dip,proto=1,id=12345,flags=1, frag=1)
packet=ip/payload2
send(packet)

ip=IP(dst=dip,proto=1,id=12345,flags=0, frag=2)
packet=ip/payload3
send(packet)

