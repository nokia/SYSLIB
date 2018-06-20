cd /lib/modules/3.10.10-rt7/kernel/net
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
