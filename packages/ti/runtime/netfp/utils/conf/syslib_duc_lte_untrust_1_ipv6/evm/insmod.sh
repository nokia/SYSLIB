cd /lib/modules/3.10.72-rt77-gdfe5ac2/kernel/net
cd netfilter/
insmod x_tables.ko
insmod nfnetlink.ko
insmod nf_conntrack.ko
insmod nf_conntrack_netlink.ko
insmod xt_conntrack.ko
insmod xt_connmark.ko
insmod nf_conntrack_proto_sctp.ko

cd ../ipv4/netfilter

insmod ip_tables.ko
insmod iptable_filter.ko
insmod iptable_raw.ko
insmod iptable_mangle.ko
insmod nf_defrag_ipv4.ko
insmod nf_conntrack_ipv4.ko

insmod /lib/modules/3.10.72-rt77-gdfe5ac2/kernel/net/ipv6/ah6.ko
insmod /lib/modules/3.10.72-rt77-gdfe5ac2/kernel/net/ipv6/tunnel6.ko
insmod /lib/modules/3.10.72-rt77-gdfe5ac2/kernel/net/ipv6/xfrm6_mode_beet.ko
insmod /lib/modules/3.10.72-rt77-gdfe5ac2/kernel/net/ipv6/xfrm6_mode_tunnel.ko
insmod /lib/modules/3.10.72-rt77-gdfe5ac2/kernel/net/ipv6/xfrm6_tunnel.ko
insmod /lib/modules/3.10.72-rt77-gdfe5ac2/kernel/net/ipv6/ip6_tunnel.ko
insmod /lib/modules/3.10.72-rt77-gdfe5ac2/kernel/net/ipv6/ipcomp6.ko
insmod /lib/modules/3.10.72-rt77-gdfe5ac2/kernel/net/ipv6/sit.ko
insmod /lib/modules/3.10.72-rt77-gdfe5ac2/kernel/net/ipv6/netfilter/ip6_tables.ko

