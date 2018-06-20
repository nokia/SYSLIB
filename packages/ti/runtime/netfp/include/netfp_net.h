/**
 *   @file  netfp_net.h
 *
 *   @brief
 *      Internal header file used by the NETFP module. Please do not
 *      directly include this file. The file defines the standard
 *      networking headers.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2012 Texas Instruments, Inc.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __NETFP_NET_H__
#define __NETFP_NET_H__

/**
 * @brief   This is the minimum size of an ethernet packet.
 */
#define ETH_MIN_PKT_SIZE                   60

/**
 * @brief   This is the maximum size of an ethernet packet.
 */
#define ETH_MAX_PKT_SIZE                    1536

/**
 * @brief   This is the protocol identification field in the Ethernet
 * header which identifies the packet as an ARP packet.
 */
#define ETH_ARP                             0x806

/**
 * @brief   This is the protocol identification field in the Ethernet
 * header which identifies the packet as an IPv4 packet.
 */
#define ETH_IP                              0x800

/**
 * @brief   This is the protocol identification field in the Ethernet
 * header which identifies the packet as an IPv6 packet.
 */
#define ETH_IP6                             0x86dd

/**
 * @brief   This is the protocol identification field in the Ethernet
 * header which identifies the packet as a VLAN Packet.
 */
#define ETH_VLAN                            0x8100

/**
 * @brief   This is the size of the trailing FCS (CRC) field in the
 * Ethernet packet.
 */
#define ETHCRC_SIZE                         4

/**
 * @brief   This is the protocol identification field in the IPv4 header
 * which identifies the packet as a UDP packet.
 */
#define IPPROTO_UDP                         17

/**
 * @brief   This is the protocol identification field in the IPv4 header
 * which identifies the packet as a IP-IP Tunnel packet.
 */
#define IPPROTO_IPIP                        4

/**
 * @brief IPv6 Extension Header: Hop-by-Hop options
 */
#define IPPROTO6_HOP                        0

/**
 * @brief   This is the protocol identification field in the IPv6 header
 * which identifies the packet as an ESP packet.
 */
#define IPPROTO6_ESP                        41

/**
 * @brief IPv6 Extension Header: Routing Header
 */
#define IPPROTO6_ROUTING                    43

/**
 * @brief   This is the protocol identification field in the IPv6 header
 * which identifies the packet as a fragmentated packet.
 */
#define IPPROTO6_FRAG                       44

/**
 * @brief IPv6 Extension Header: No Header
 */
#define IPPROTO6_NONE                       59

/**
 * @brief IPv6 Extension Header: Destination options Header
 */
#define IPPROTO6_DSTOPTS                    60

/**
 * @brief   This is the protocol identification field in the IPv4 header
 * which identifies the packet as an ESP packet.
 */
#define IPPROTO_ESP                         50

/**
 * @brief   This is the protocol identification field in the IPv4 header
 * which identifies the packet as an AH packet.
 */
#define IPPROTO_AH                          51

/**
 * @brief ESP/AH header margin. Used to add ESP/AH header. This is the MAX
 * allowed header packet size
 */
#define NETFP_SA_PKT_HDR_MARGIN             32

/**
 * @brief ESP/AH tail margin. Used for tail, hash, padding, tag. This is the
 * MAX allowed trailer packet size.
 */
#define NETFP_SA_PKT_TAIL_MARGIN            32

/**
 * @brief Maximum number of IPv6 Extension headers which can be present in a fragment
 */
#define NETFP_MAX_EXTENSION_HEADER          8

/***********************************************************************
 ****************** Standard Network Header Definitons *****************
 ***********************************************************************/

#define ETHHDR_SIZE     14
typedef struct __attribute__((packed)) Netfp_EthHeader
{
    uint8_t   DstMac[6];
    uint8_t   SrcMac[6];
    uint16_t  Type;
}Netfp_EthHeader;

#define VLANHDR_SIZE     4
typedef struct __attribute__((packed)) Netfp_VLANHeader
{
    uint16_t  tci;
    uint16_t  protocol;
}Netfp_VLANHeader;

#define IPV4_HLEN_MASK          0x0F
#define IPV4_HLEN_SHIFT         0
#define IPHDR_SIZE              20
#define IPV4_FRAGO_MASK         0x1FFF
#define IPV4_FLAGS_MF_MASK      0x2000
#define IPV4_FLAGS_DF_MASK      0x4000
#define IPV4_FRAG_DET_MASK      (IPV4_FLAGS_MF_MASK | \
                                 IPV4_FRAGO_MASK)
typedef struct __attribute__((packed)) Netfp_IPHeader
{
    uint8_t    VerLen;
    uint8_t    Tos;
    uint16_t   TotalLen;
    uint16_t   Id;
    uint16_t   FlagOff;
    uint8_t    Ttl;
    uint8_t    Protocol;
    uint16_t   Checksum;
    uint8_t    IPSrc[4];
    uint8_t    IPDst[4];
    uint8_t    Options[1];
}Netfp_IPHeader;

#define IPv6HDR_SIZE        40
#define IPV6_VER_VALUE      0x60
#define IPV6_FRAGO_MASK     0xFFF8
#define IPV6_FLAGS_MF_MASK  0x0001
typedef struct Netfp_IPv6Header
{
    uint8_t        VerTC;
    uint8_t        FlowLabel[3];
    uint16_t       PayloadLength;
    uint8_t        NextHeader;
    uint8_t        HopLimit;
    Netfp_IP6N     SrcAddr;
    Netfp_IP6N     DstAddr;
}Netfp_IPv6Header;

#define IPV6_FRAGHDR_SIZE      8
typedef struct Netfp_IPv6FragHeader
{
    uint8_t  NextHeader;
    uint8_t  Rsvd;
    uint16_t FragOffset;
    uint8_t  FragId[4];
}Netfp_IPv6FragHeader;

typedef struct Netfp_PseudoV6Hdr
{
    Netfp_IP6N     SrcAddr;
    Netfp_IP6N     DstAddr;
    uint32_t       PktLen;
    uint8_t        Rsvd[3];
    uint8_t        NxtHdr;
}Netfp_PseudoV6Hdr;

#define UDPHDR_SIZE     8
typedef struct __attribute__((packed)) Netfp_UDPHeader
{
    uint16_t   SrcPort;
    uint16_t   DstPort;
    uint16_t   Length;
    uint16_t   UDPChecksum;
}Netfp_UDPHeader;

#define GTPU_PORT       2152
#define GTPHDR_SIZE     8
typedef struct Netfp_GTPHeader
{
    uint8_t    Flags;
    uint8_t    MsgType;
    uint16_t   TotalLength;
    uint8_t    TunnelId[4];
}Netfp_GTPHeader;

/**
 *  @b Description
 *  @n
 *      Internal utility function which analyzes the IP header to determine if the
 *      packet is an IPv4 packet or IPv6 packet.
 *
 *  @param[in]  ptrIPHeader
 *      Pointer to the IP header to be analyzed
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      1   - IPv4 Packet
 *  @retval
 *      0   - IPv6 Packet
 */
static inline uint8_t Netfp_isIPv4Packet (Netfp_IPHeader* ptrIPHeader)
{
    if ((ptrIPHeader->VerLen & 0xF0) == 0x40)
        return 1;
    return 0;
}

/**
 *  @b Description
 *  @n
 *       The function computes the IP checksum. The computed checksum
 *       is populated in the IP header.
 *
 *  @param[in]  ptr_iphdr
 *      This is the pointer to the IPv4 header for which the checksum
 *      is computed.
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not Applicable.
 */
static inline void Netfp_IPChecksum(Netfp_IPHeader* ptr_iphdr)
{
    int32_t   tmp1;
    uint16_t  *pw;
    uint32_t  TSum = 0;

    /* Get header size in 4 byte chunks */
    tmp1 = ptr_iphdr->VerLen & 0xF;

    /* Checksum field is NULL in checksum calculations */
    ptr_iphdr->Checksum = 0;

    /* Checksum the header */
    pw = (uint16_t *)ptr_iphdr;
    do {
        TSum += (uint32_t)*pw++;
        TSum += (uint32_t)*pw++;
    } while( --tmp1 );
    TSum = (TSum&0xFFFF) + (TSum>>16);
    TSum = (TSum&0xFFFF) + (TSum>>16);
    TSum = ~TSum;

    /* Note checksum is Net/Host byte order independent */
    ptr_iphdr->Checksum = (uint16_t)TSum;
    return;
}

#endif /* __NETFP_NET_H__ */

