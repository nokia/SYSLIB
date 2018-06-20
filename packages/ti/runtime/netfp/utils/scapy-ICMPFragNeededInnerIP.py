#########################################################################################
# ICMP Packet Fragmentation for Inner IP
#
# The script is used to generate an ICMP Fragmentation needed message
#########################################################################################

# Construct the Outer IP and ICMP Error message
sip = '10.10.10.1'
dip = '10.10.10.2'
ip = IP()
icmp = ICMP()
ip.dst = dip
ip.src = sip
ip.protocol = 1 # ICMP
icmp.type = 3 # Destination Unreachable
icmp.code = 4 # Fragmentation needed
mtu = 1300
icmp.unused = mtu

# Construct the Inner IP embedded into the ICMP error message to simulate
# the packet which caused the ICMP error
ip_orig = IP()
ip_orig.src = '192.168.1.2'
ip_orig.dst = '192.168.1.1'
udp_orig = UDP()
udp_orig.sport = 50000
udp_orig.dport = 50000

# Send the packet
send (ip/icmp/ip_orig/udp_orig)

