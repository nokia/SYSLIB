/**
 *   @file  pmtu6.c
 *
 *   @brief
 *      The file implements the Path MTU discovery for IPv6.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2015 Texas Instruments, Inc.
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

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <pthread.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <netinet/ip6.h>
#include <netinet/icmp6.h>

/* NETFP Proxy Files. */
#include <ti/apps/netfp_proxy/include/netfp_proxy_pvt.h>
#include <ti/apps/netfp_proxy/netfp_proxy.h>

/**************************************************************************
 ************************* Local Structures *******************************
 **************************************************************************/

/**
 * @brief
 *  IPv6 Path MTU Management MCB
 *
 * @details
 *  The structure contains information which is used by the IPv6 Path Management
 *  module.
 */
typedef struct NetfpProxy_PMTU6MgmtMCB
{
    /**
     * @brief   Socket used to listen to all ICMPv6 messages
     */
    int32_t     pmtuSocket;
}NetfpProxy_PMTU6MgmtMCB;

/**************************************************************************
 ************************* Global Variables *******************************
 **************************************************************************/

/* Global variable which tracks information required by the IPv6 Path MTU module */
NetfpProxy_PMTU6MgmtMCB     gNetfpProxyPMTU6MCB;

/**************************************************************************
 ************************* Path MTU6 Functions ****************************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      The function is used to process an ICMPv4 message received by the
 *      application. The function will only handle the ICMPv4 Packet Too
 *      Big error message and all other ICMP error messages are dropped.
 *
 *  @retval
 *      Not applicable
 */
void NetfpProxy_pmtu6ProcessPacket (void)
{
    int32_t                 retVal;
    char                    buffer[1536];
	struct icmp6_hdr*       ptrICMPv6Header;
    struct ip6_hdr*         ptrIPv6Header;
    Netfp_ProxyServerInfo   proxyServerInfo;
    int32_t                 errCode;
    char                    srcIP[128];
    char                    dstIP[128];

    /* Read the data from the IPv6 MTU socket: */
    retVal = recv(gNetfpProxyPMTU6MCB.pmtuSocket, &buffer[0], sizeof(buffer), 0);
    if (retVal < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to receive the ICMPv6 message [Error: %s]\n", strerror(errno));
        return;
    }

    /* Get the pointer to the ICMPv6 Header: */
    ptrICMPv6Header = (struct icmp6_hdr*)&buffer[0];

    /* ICMPv6 Error message was received: We only handle the PACKET too big */
    if (ptrICMPv6Header->icmp6_type != ICMP6_PACKET_TOO_BIG)
        return;

    /* Get the pointer to the IPv6 Header which caused the error: This will follow the ICMP Header */
    ptrIPv6Header = (struct ip6_hdr*)(&ptrICMPv6Header->icmp6_dataun.icmp6_un_data32[1]);

    /* Initialize the Proxy Server Informational Block: */
    memset ((void *)&proxyServerInfo, 0, sizeof(Netfp_ProxyServerInfo));

    /* Populate the Proxy Server Informational Block: */
    proxyServerInfo.opType                   = Netfp_ProxyServerOp_UPDATE_MTU;
    proxyServerInfo.dstIP.ver                = Netfp_IPVersion_IPV6;
    proxyServerInfo.dstIP.addr.ipv6.u.a32[0] = ptrIPv6Header->ip6_dst.s6_addr32[0];
    proxyServerInfo.dstIP.addr.ipv6.u.a32[1] = ptrIPv6Header->ip6_dst.s6_addr32[1];
    proxyServerInfo.dstIP.addr.ipv6.u.a32[2] = ptrIPv6Header->ip6_dst.s6_addr32[2];
    proxyServerInfo.dstIP.addr.ipv6.u.a32[3] = ptrIPv6Header->ip6_dst.s6_addr32[3];
    proxyServerInfo.srcIP.ver                = Netfp_IPVersion_IPV6;
    proxyServerInfo.srcIP.addr.ipv6.u.a32[0] = ptrIPv6Header->ip6_src.s6_addr32[0];
    proxyServerInfo.srcIP.addr.ipv6.u.a32[1] = ptrIPv6Header->ip6_src.s6_addr32[1];
    proxyServerInfo.srcIP.addr.ipv6.u.a32[2] = ptrIPv6Header->ip6_src.s6_addr32[2];
    proxyServerInfo.srcIP.addr.ipv6.u.a32[3] = ptrIPv6Header->ip6_src.s6_addr32[3];
    proxyServerInfo.u.updateMTU.newMTU       = ntohl(ptrICMPv6Header->icmp6_mtu);

    if (proxyServerInfo.u.updateMTU.newMTU < 1280)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_INFO, "Info: Received PMTU %d less than IPv6 minimum, forcing to 1280!\n",
                           proxyServerInfo.u.updateMTU.newMTU);
        proxyServerInfo.u.updateMTU.newMTU = 1280;
    }

    /* Convert the source & destination IP into strings for display */
    Netfp_convertIP6ToStr (proxyServerInfo.srcIP.addr.ipv6, &srcIP[0]);
    Netfp_convertIP6ToStr (proxyServerInfo.dstIP.addr.ipv6, &dstIP[0]);

    /* Did the plugin register a report PMTU function? */
    if (gNetfpProxyMcb.pluginCfg.reportPMTU != NULL)
    {
        /* YES: Pass the information to the plugin registered function: */
        if (gNetfpProxyMcb.pluginCfg.reportPMTU (&proxyServerInfo.srcIP, &proxyServerInfo.dstIP, &proxyServerInfo.u.updateMTU.newMTU) < 0)
            return;
    }

    /* Send the asynchronous MTU update to the NETFP Server */
    if (Netfp_asyncUpdate (gNetfpProxyMcb.netfpClientHandle, &proxyServerInfo, &errCode) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: PMTU Update [%s] -> [%s] MTU %d bytes failed [Error %d] \n",
                           srcIP, dstIP, proxyServerInfo.u.updateMTU.newMTU, errCode);
        return;
    }

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: PMTU Update [%s] -> [%s] MTU %d bytes\n",
                       srcIP, dstIP, proxyServerInfo.u.updateMTU.newMTU);
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to initialize the IPv6 Path MTU module
 *
 *  @retval
 *      Success     -   Pointer to the ICMPv4 socket
 *  @retval
 *      Error       -   <0
 */
int32_t NetfpProxy_pmtu6Init (void)
{
    struct icmp6_filter     icmp6Filter;

    /* Initialize the MCB */
    memset ((void *)&gNetfpProxyPMTU6MCB, 0, sizeof(NetfpProxy_PMTU6MgmtMCB));

    /* Open a raw socket to receive all the ICMPv6 messages */
    gNetfpProxyPMTU6MCB.pmtuSocket = socket(PF_INET6, SOCK_RAW, IPPROTO_ICMPV6);
    if (gNetfpProxyPMTU6MCB.pmtuSocket < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to open the ICMPv6 PMTU Socket [Error: %s]\n", strerror(errno));
        return -1;
    }

    /* Set ICMPv6 filter */
    ICMP6_FILTER_SETBLOCKALL (&icmp6Filter);
	ICMP6_FILTER_SETPASS(ICMP6_PACKET_TOO_BIG, &icmp6Filter);

    /* Set the socket option to receive only the ICMPv6 Packet too big messages: */
	if (setsockopt (gNetfpProxyPMTU6MCB.pmtuSocket, IPPROTO_ICMPV6, ICMP6_FILTER, &icmp6Filter, sizeof (struct icmp6_filter)) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR,  "Error: Unable to setup the ICMPv6 Filter [Error: %s]\n", strerror(errno));
        return -1;
    }

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: IPv6 Path MTU socket [%d] has been created successfully\n", gNetfpProxyPMTU6MCB.pmtuSocket);
    return gNetfpProxyPMTU6MCB.pmtuSocket;
}

/**
 *  @b Description
 *  @n
 *      The function is used to deinitialize and shutdown the the IPv6 Path MTU module
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
int32_t NetfpProxy_pmtu6Deinit (void)
{
    close (gNetfpProxyPMTU6MCB.pmtuSocket);
    return 0;
}

