#!/bin/sh
# Configuration script to setup NAT-T device 

# Add alias address
sudo ifconfig eth1 20.20.20.5 up
sudo ifconfig eth2 10.10.10.5 up 

# Enable IPv4 forwarding
sudo sysctl -w net.ipv4.conf.all.forwarding=1

# Enable IPv6 forwarding
sudo sysctl -w net.ipv6.conf.all.forwarding=1

# Enable nat
sudo /sbin/iptables -t nat -A POSTROUTING -o eth2  -j MASQUERADE
sudo /sbin/iptables -A FORWARD -i eth2 -o eth1 -m state --state RELATED,ESTABLISHED -j ACCEPT
sudo /sbin/iptables -A FORWARD -i eth1 -o eth2 -j ACCEPT


