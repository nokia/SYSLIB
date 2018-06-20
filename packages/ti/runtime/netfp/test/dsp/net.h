/**
 *   @file  net.h
 *
 *   @brief
 *      Standard networking headers
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

#ifndef __NET_H__
#define __NET_H__

#ifdef _BIG_ENDIAN 

/**
 * @brief   Macro which converts 16 bit data from host to network order.
 */
#define  htons(a) (a)

/**
 * @brief   Macro which converts 32 bit data from host to network order.
 */
#define  htonl(a) (a)

/**
 * @brief   Macro which converts 32 bit data from network to host order.
 */
#define  ntohl(a) (a)

/**
 * @brief   Macro which converts 16 bit data from network to host order.
 */
#define  ntohs(a) (a)

#else

/**
 * @brief   Macro which converts 16 bit data from host to network order.
 */
#define  htons(a)    ( (((a)>>8)&0xff) + (((a)<<8)&0xff00) )

/**
 * @brief   Macro which converts 32 bit data from host to network order.
 */
#define  htonl(a)    ( (((a)>>24)&0xff) + (((a)>>8)&0xff00) + \
                       (((a)<<8)&0xff0000) + (((a)<<24)&0xff000000) )

/**
 * @brief   Macro which converts 32 bit data from network to host order.
 */
#define  ntohl(a)   htonl(a)

/**
 * @brief   Macro which converts 16 bit data from network to host order.
 */
#define  ntohs(a)   htons(a)

#endif /* _BIG_ENDIAN */


/***********************************************************************
 ****************** STANDARD NETWORK HEADER DEFINITIONS ****************
 ***********************************************************************/

/* Standard Length of the various networking headers. */
#define NET_ETH_HDR_SIZE        14
#define NET_IPV4_HDR_SIZE       20
#define NET_GTP_HDR_SIZE        8
#define NET_UDP_HDR_SIZE        8

/**
 * @brief   This is the protocol identification field in the Ethernet
 * header which identifies the packet as an IPv4 packet.
 */
#define ETH_IP                  0x800

/**
 * @brief   This is the protocol identification field in the Ethernet
 * header which identifies the packet as a VLAN packet.
 */
#define ETH_VLAN                 0x8100

/**
 * @brief   This is the protocol identification field in the IPv4 header
 * which identifies the packet as a UDP packet.
 */
#define IPPROTO_UDP             17

/* GTPU Message Type Definitions: */
#define NET_GTP_PING_REQUEST          1
#define NET_GTP_PING_RESPONSE         2
#define NET_GTP_ERROR_INDICATION      26
#define NET_GTP_HEADER_NOTIFY         31
#define NET_GTP_PARSING_ERROR         22        /* Reserved for future use */
#define NET_GTP_END_MARKER            254
#define NET_GTP_DATA                  255

typedef struct Net_EthHdr
{
    uint8_t   DstMac[6];
    uint8_t   SrcMac[6];
    uint16_t  Type;
}Net_EthHdr;

typedef struct Net_VlanEthHdr
{
    uint8_t   DstMac[6];
    uint8_t   SrcMac[6];
    uint16_t  Type;
    uint16_t  tci;
    uint16_t  Protocol;
}Net_VlanEthHdr;

typedef struct Net_IPHdr
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
}Net_IPHdr;

typedef struct Net_UDPHdr
{
    uint16_t   SrcPort;
    uint16_t   DstPort;
    uint16_t   Length;
    uint16_t   UDPChecksum;
}Net_UDPHdr;

typedef struct Net_GTPUHdr
{
    uint8_t    Flags;
    uint8_t    MsgType;
    uint16_t   TotalLength;
    uint8_t    TunnelId[4];
}Net_GTPUHdr;

#endif /* __NET_H__ */

