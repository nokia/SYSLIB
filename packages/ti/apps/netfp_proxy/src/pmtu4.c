/**
 *   @file  pmtu4.c
 *
 *   @brief
 *      The file implements the Path MTU discovery for IPv4.
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
#include <netinet/ip_icmp.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/ip.h>

/* NETFP Proxy Files. */
#include <ti/apps/netfp_proxy/include/netfp_proxy_pvt.h>
#include <ti/apps/netfp_proxy/netfp_proxy.h>

/**************************************************************************
 ************************* Local Structures *******************************
 **************************************************************************/

/**
 * @brief
 *  IPv4 Path MTU Management MCB
 *
 * @details
 *  The structure contains information which is used by the IPv4 Path Management
 *  module.
 */
typedef struct NetfpProxy_PMTU4MgmtMCB
{
    /**
     * @brief   Socket used to listen to all ICMPv4 messages
     */
    int32_t     pmtuSocket;
}NetfpProxy_PMTU4MgmtMCB;

/**************************************************************************
 ************************* Global Variables *******************************
 **************************************************************************/

/* Global variable which tracks information required by the IPv4 Path MTU module */
NetfpProxy_PMTU4MgmtMCB     gNetfpProxyPMTU4MCB;

/**************************************************************************
 ************************* Path MTU4 Functions ****************************
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
void NetfpProxy_pmtu4ProcessPacket (void)
{
    int32_t                 retVal;
    char                    buffer[1536];
	struct icmp*            ptrICMPHeader;
	struct iphdr*           ptrIPHeader;
    Netfp_ProxyServerInfo   proxyServerInfo;
    int32_t                 errCode;

    /* Read the data from the IPv4 MTU socket: */
    retVal = recv(gNetfpProxyPMTU4MCB.pmtuSocket, &buffer[0], sizeof(buffer), 0);
    if (retVal < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to receive the ICMPv4 message [Error: %s]\n", strerror(errno));
        return;
    }

    /* Get the pointer to the IP Header: */
    ptrIPHeader = (struct iphdr*)&buffer[0];

    /* Skip the IP Header to get to the ICMP Header */
    ptrICMPHeader = (struct icmp*)((char*)&buffer[0] + (ptrIPHeader->ihl << 2));

    /* ICMP Error message was received: We only handle the destination unreachable error message */
    if (ptrICMPHeader->icmp_type != ICMP_DEST_UNREACH)
        return;

    /* We need to handle only the fragmentation needed/DF bit set here */
    if (ptrICMPHeader->icmp_code != ICMP_FRAG_NEEDED)
        return;

    /* Initialize the Proxy Server Informational Block: */
    memset ((void *)&proxyServerInfo, 0, sizeof(Netfp_ProxyServerInfo));

    /* Populate the Proxy Server Informational Block: */
    proxyServerInfo.opType                = Netfp_ProxyServerOp_UPDATE_MTU;
    proxyServerInfo.dstIP.ver             = Netfp_IPVersion_IPV4;
    proxyServerInfo.dstIP.addr.ipv4.u.a32 = ptrICMPHeader->icmp_ip.ip_dst.s_addr;
    proxyServerInfo.srcIP.ver             = Netfp_IPVersion_IPV4;
    proxyServerInfo.srcIP.addr.ipv4.u.a32 = ptrICMPHeader->icmp_ip.ip_src.s_addr;
    proxyServerInfo.u.updateMTU.newMTU    = ntohs(ptrICMPHeader->icmp_nextmtu);

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
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: PMTU Update [%03d.%03d.%03d.%03d] -> [%03d.%03d.%03d.%03d] MTU %d bytes failed [Error %d] \n",
                           proxyServerInfo.srcIP.addr.ipv4.u.a8[0], proxyServerInfo.srcIP.addr.ipv4.u.a8[1],
                           proxyServerInfo.srcIP.addr.ipv4.u.a8[2], proxyServerInfo.srcIP.addr.ipv4.u.a8[3],
                           proxyServerInfo.dstIP.addr.ipv4.u.a8[0], proxyServerInfo.dstIP.addr.ipv4.u.a8[1],
                           proxyServerInfo.dstIP.addr.ipv4.u.a8[2], proxyServerInfo.dstIP.addr.ipv4.u.a8[3],
                           proxyServerInfo.u.updateMTU.newMTU, errCode);
        return;
    }

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: PMTU Update [%03d.%03d.%03d.%03d] -> [%03d.%03d.%03d.%03d] MTU %d bytes\n",
                       proxyServerInfo.srcIP.addr.ipv4.u.a8[0], proxyServerInfo.srcIP.addr.ipv4.u.a8[1],
                       proxyServerInfo.srcIP.addr.ipv4.u.a8[2], proxyServerInfo.srcIP.addr.ipv4.u.a8[3],
                       proxyServerInfo.dstIP.addr.ipv4.u.a8[0], proxyServerInfo.dstIP.addr.ipv4.u.a8[1],
                       proxyServerInfo.dstIP.addr.ipv4.u.a8[2], proxyServerInfo.dstIP.addr.ipv4.u.a8[3],
                       proxyServerInfo.u.updateMTU.newMTU);
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to initialize the IPv4 Path MTU module
 *
 *  @retval
 *      Success     -   Pointer to the ICMPv4 socket
 *  @retval
 *      Error       -   <0
 */
int32_t NetfpProxy_pmtu4Init (void)
{
    /* Initialize the MCB */
    memset ((void *)&gNetfpProxyPMTU4MCB, 0, sizeof(NetfpProxy_PMTU4MgmtMCB));

    /* Open a raw socket to receive all the ICMP messages */
    gNetfpProxyPMTU4MCB.pmtuSocket = socket(AF_INET, SOCK_RAW, 1);
    if (gNetfpProxyPMTU4MCB.pmtuSocket < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to open the ICMP PMTU Socket [Error: %s]\n", strerror(errno));
        return -1;
    }

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: IPv4 Path MTU socket [%d] has been created successfully\n", gNetfpProxyPMTU4MCB.pmtuSocket);
    return gNetfpProxyPMTU4MCB.pmtuSocket;
}

/**
 *  @b Description
 *  @n
 *      The function is used to deinitialize and shutdown the the IPv4 Path MTU module
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
int32_t NetfpProxy_pmtu4Deinit (void)
{
    close (gNetfpProxyPMTU4MCB.pmtuSocket);
    return 0;
}

