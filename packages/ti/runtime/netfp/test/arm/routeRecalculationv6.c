/*
 *   @file  routeRecalculationv6.c
 *
 *   @brief
 *      Unit Test code for the v6 route recalculation test
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
 *
 */

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>

/* MCSDK Include Files. */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>

/* SYSLIB Include Files.  */
#include <ti/runtime/msgcom/msgcom.h>
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/netfp/netfp.h>
#include <ti/drv/rm/rm.h>

/* Standard socket includes */
#include <sys/socket.h>
#include <sys/un.h>
#include <errno.h>

/* NetFP Proxy includes */
#include <ti/apps/netfp_proxy/netfp_proxy_ipc.h>


/**********************************************************************
 *********************** Local Definitions ****************************
 **********************************************************************/

/* Limit on the MAX number of fast paths which are supported. */
#define TEST_MAX_FAST_PATH      253

/**********************************************************************
 ************************ Global Variables ****************************
 **********************************************************************/

/* Dummy Flow and channel created */
int32_t             dummyFlowId;
MsgCom_ChHandle     dummyChannel;

/* Global Handles:  */
extern Netfp_InboundFPHandle   ingressFPHandle;
extern Netfp_OutboundFPHandle  egressFPHandle[TEST_MAX_FAST_PATH];
extern Netfp_SockHandle        sockHandle[TEST_MAX_FAST_PATH];
extern Netfp_Reason            sockReason[TEST_MAX_FAST_PATH];

/* Global Flag which logs messages */
extern uint32_t                loggingFlag;

/**********************************************************************
 ************************ Extern Declarations *************************
 **********************************************************************/

/* NETFP Client Handle: */
extern Netfp_ClientHandle   netfpClientHandle;
extern Pktlib_HeapHandle    netfpDataRxHeap;

/* Global MSGCOM instance handle. */
extern Msgcom_InstHandle   appMsgcomInstanceHandle;

extern void Test_notifySocketFunction(Netfp_Reason reason, Netfp_SockHandle sockHandle);
extern int32_t Test_executeNetfpProxyCommand (void);

/**********************************************************************
 ************************** Unit Test Functions ***********************
 **********************************************************************/

/**
 *  @b Description
 *  @n
 *      The function is used to test the socket states.
 *
 *      The following is the test setup which is constructed here:
 *
 *      |----------|                  |--------|
 *      |   EVM    |------------------| GW     |
 *      |----------|                  |--------|
 *          7000::2                     7000::1
 *          Default Gw: 7000::1
 *
 *  (a) Sockets are connected to IP address which dont exist.
 *  (b) The outbound fast path and sockets will remain not active.
 *  (c) Routing entries are added and the sockets and fast paths remain not active
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Test_v6SocketStates(void)
{
    Netfp_InboundFPCfg  inboundFPConfig;
    Netfp_OutboundFPCfg outboundFPConfig;
    int32_t             errCode;
    int32_t             status;
    Netfp_SockAddr      sockAddress;
    Netfp_OptionTLV     optInfo;
    uint8_t             socketState;
    int32_t             index = 0;
    char                srcIP[40];
    char                dstIP[40];

    /* Debug Message: */
    printf ("***************************************************************************************\n");
    printf ("Debug: Testing socket states:\n");

    /*************************************************************************************************
     * TEST SETUP:
     *************************************************************************************************/
    system("ifconfig eth0 192.168.1.2");
    system("ifconfig eth0 7000::2/64");

    /* Create the Ingress fast paths: Initialize the fast path configuration */
    memset ((void *)&inboundFPConfig, 0, sizeof (Netfp_InboundFPCfg));

    /* Populate the fast path configuration: */
    inboundFPConfig.spId                       = NETFP_INVALID_SPID;
    inboundFPConfig.dstIP.ver                  = Netfp_IPVersion_IPV6;
    inboundFPConfig.dstIP.addr.ipv6.u.a32[0]    = 0x00000070;
    inboundFPConfig.dstIP.addr.ipv6.u.a32[1]    = 0;
    inboundFPConfig.dstIP.addr.ipv6.u.a32[2]    = 0;
    inboundFPConfig.dstIP.addr.ipv6.u.a32[3]    = 0x02000000;
    inboundFPConfig.srcIP.ver                  = Netfp_IPVersion_IPV6;
    inboundFPConfig.srcIP.addr.ipv6.u.a32[0]    = 0x00000070;
    inboundFPConfig.srcIP.addr.ipv6.u.a32[1]    = 0;
    inboundFPConfig.srcIP.addr.ipv6.u.a32[2]    = 0;
    inboundFPConfig.srcIP.addr.ipv6.u.a32[3]    = 0x05000000;
    snprintf (inboundFPConfig.name, NETFP_MAX_CHAR, "Ingress_FastPath-%d", index);

    /* Add the Ingress Fast Path. */
    ingressFPHandle = Netfp_createInboundFastPath(netfpClientHandle, &inboundFPConfig, &errCode);
    if (ingressFPHandle == NULL)
    {
        printf ("Error: Unable to add the NETFP Inbound Fast path [Error code %d]\n", errCode);
        return -1;
    }

    /* Convert the Src & Dst IP address of the SA into strings using the NETFP IPv6 Utility API */
    Netfp_convertIP6ToStr (inboundFPConfig.srcIP.addr.ipv6, &srcIP[0]);
    Netfp_convertIP6ToStr (inboundFPConfig.dstIP.addr.ipv6, &dstIP[0]);

    printf ("Debug: Adding %s [%s] -> [%s]\n", inboundFPConfig.name, srcIP, dstIP);

    /* Resolve the route to each possible destination:
     * - These resolutions will end up getting resolved into the default gateway. */
    memset ((void *)&outboundFPConfig, 0, sizeof (Netfp_OutboundFPCfg));

    /* Populate the Fast Path configuration. */
    outboundFPConfig.spId                       = NETFP_INVALID_SPID;
    outboundFPConfig.dstIP.ver                  = Netfp_IPVersion_IPV6;
    outboundFPConfig.dstIP.addr.ipv6.u.a32[0]    = 0x00000070;
    outboundFPConfig.dstIP.addr.ipv6.u.a32[1]    = 0;
    outboundFPConfig.dstIP.addr.ipv6.u.a32[2]    = 0;
    outboundFPConfig.dstIP.addr.ipv6.u.a32[3]    = (0x05000000 + index);
    outboundFPConfig.srcIP.ver                  = Netfp_IPVersion_IPV6;
    outboundFPConfig.srcIP.addr.ipv6.u.a32[0]    = 0x00000070;
    outboundFPConfig.srcIP.addr.ipv6.u.a32[1]    = 0;
    outboundFPConfig.srcIP.addr.ipv6.u.a32[2]    = 0;
    outboundFPConfig.srcIP.addr.ipv6.u.a32[3]    = 0x02000000;
    snprintf (outboundFPConfig.name, NETFP_MAX_CHAR, "Egress_FastPath-%d", index);

    /* Add the Egress Fast Path. */
    egressFPHandle[index] = Netfp_createOutboundFastPath (netfpClientHandle, &outboundFPConfig, &errCode);
    if (egressFPHandle[index] == NULL)
    {
        printf ("Error: Unable to add the NETFP Egress Fast path [Error code %d]\n", errCode);
        return -1;
    }

    /* Convert the Src & Dst IP address of the SA into strings using the NETFP IPv6 Utility API */
    Netfp_convertIP6ToStr (outboundFPConfig.srcIP.addr.ipv6, &srcIP[0]);
    Netfp_convertIP6ToStr (outboundFPConfig.dstIP.addr.ipv6, &dstIP[0]);

    printf ("Debug: Adding %s [%s] -> [%s]\n", outboundFPConfig.name, srcIP, dstIP);

    /* Relinuqish time slice: This will give enough time to allow the NETFP Proxy to resolve the next
     * hop MAC address */
    sleep(10);

    /* The outbound fast should always remain inactive because the neighbor does not exist. */
    if (Netfp_isOutboundFastPathActive (netfpClientHandle, egressFPHandle[index], &status, &errCode) < 0)
    {
        printf ("Error: Getting status of the outbound fast path failed [Error code %d]\n", errCode);
        return -1;
    }

    /* Is the fast path active? */
    if (status == 1)
    {
        printf ("Error: Outbound Fast path was detected to be active\n");
        return -1;
    }

    /* Create a socket */
    sockHandle[index] = Netfp_socket (netfpClientHandle, Netfp_SockFamily_AF_INET6, &errCode);
    if (sockHandle[index] == NULL)
    {
        printf ("Error: NETFP Socket '%d' Creation Failed [Error Code %d]\n", index, errCode);
        return -1;
    }

    /* Populate the binding information: */
    memset ((void*)&sockAddress, 0, sizeof(Netfp_SockAddr));
    sockAddress.sin_family              = Netfp_SockFamily_AF_INET6;
    sockAddress.sin_port                = 4000 + index;
    sockAddress.op.bind.inboundFPHandle = ingressFPHandle;
    sockAddress.op.bind.flowId          = dummyFlowId;
    sockAddress.op.bind.queueHandle     = Msgcom_getInternalMsgQueueInfo(dummyChannel);
    sockAddress.op.bind.notifyFunction  = Test_notifySocketFunction;

    /* Bind the socket. */
    if (Netfp_bind (sockHandle[index], &sockAddress, &errCode) < 0)
    {
        printf ("Error: NETFP Bind Failed for socket '%d' on port '%d' [Error Code %d]\n",
                index, sockAddress.sin_port, errCode);
        return -1;
    }

    /* Populate the connect information. */
    sockAddress.sin_family                  = Netfp_SockFamily_AF_INET6;
    sockAddress.sin_port                    = 4000 + index;
    sockAddress.op.connect.outboundFPHandle = egressFPHandle[index];

    /* Connect the socket */
    if (Netfp_connect(sockHandle[index], &sockAddress, &errCode) < 0)
    {
        printf ("Error: NETFP Connect Failed for socket '%d' to port '%d' [Error Code %d]\n",
                index, sockAddress.sin_port, errCode);
        return -1;
    }

    /* Socket should NOT be active becuase the fast path was not active */
    optInfo.type   = Netfp_Option_STATE;
    optInfo.length = 1;
    optInfo.value  = (void*)&socketState;

    /* Get the socket option: */
    if (Netfp_getSockOpt (sockHandle[index], &optInfo, &errCode) < 0)
    {
        printf ("Error: Getting socket option failed [Error code %d]\n", errCode);
        return -1;
    }

    /* Sanity Check: Ensure that the socket is NOT active */
    if (socketState == 1)
    {
        printf ("Error: Socket handle 0x%x index %d invalid socket state %d detected\n", (uint32_t)sockHandle[index], index, socketState);
        return -1;
    }

    /*************************************************************************************************
     * EVENT GENERATION: Add a default route
     *************************************************************************************************/
    printf ("Debug: Adding a default route\n");

    /* Cycle through and reset the socket reasons: We are about to start the test */
    sockReason[index] = 0;

    system ("ip route add default via 7000::1");

    /* Send the FLUSH command to Netfp Proxy */
    if (Test_executeNetfpProxyCommand() < 0)
    {
        printf ("Error: Failed to send Flush IP routes command to NetfpProxy\n");
        return -1;
    }
    sleep(10);

    /*************************************************************************************************
     * VALIDATIONS: Nothing should happen. This does NOT impact our system at all.
     *************************************************************************************************/
    if (sockReason[index] != 0)
    {
        /* Error: Reason detected for the socket. */
        printf ("Error: Socket handle 0x%x index %d was notified\n", (uint32_t)sockHandle[index], index);
        return -1;
    }

    /* Each socket should no longer be active: */
    optInfo.type   = Netfp_Option_STATE;
    optInfo.length = 1;
    optInfo.value  = (void*)&socketState;

    /* Get the socket option: */
    if (Netfp_getSockOpt (sockHandle[index], &optInfo, &errCode) < 0)
    {
        printf ("Error: Getting socket option failed [Error code %d]\n", errCode);
        return -1;
    }

    /* Sanity Check: Ensure that the socket is NOT active */
    if (socketState == 1)
    {
        printf ("Error: Socket handle 0x%x index %d invalid socket state %d detected\n", (uint32_t)sockHandle[index], index, socketState);
        return -1;
    }

    /* The outbound fast should always remain inactive because the neighbor does not exist. */
    if (Netfp_isOutboundFastPathActive (netfpClientHandle, egressFPHandle[index], &status, &errCode) < 0)
    {
        printf ("Error: Getting status of the outbound fast path failed [Error code %d]\n", errCode);
        return -1;
    }

    /* Is the fast path active? */
    if (status == 1)
    {
        printf ("Error: Outbound Fast path was detected to be active\n");
        return -1;
    }

    /*************************************************************************************************
     * CLEANUP:
     *************************************************************************************************/

    /* Delete the socket: */
    if (Netfp_closeSocket (sockHandle[index], &errCode) < 0)
        printf ("Error: NETFP Close socket failed [Error code %d]\n", errCode);

    /* Delete the ingress fast path: */
    if (Netfp_deleteInboundFastPath (netfpClientHandle, ingressFPHandle, &errCode) < 0)
        printf ("Error: NETFP Ingress fast path deletion failed [Error code %d]\n", errCode);

    /* Delete the egress fast path: */
    if (Netfp_deleteOutboundFastPath (netfpClientHandle, egressFPHandle[index], &errCode) < 0)
        printf ("Error: NETFP Egress fast path deletion failed [Error code %d]\n", errCode);

    /* Delete the default gateway: Moving back to the original state */
    system ("ip route del default via 7000::1");
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to test the socket notification events for interface.
 *      NETFP captures events for MTU changes and Interface status modifications.
 *
 *      The following is the test setup which is constructed here:
 *
 *      |----------|                  |--------|
 *      |   EVM    |------------------| GW     |
 *      |----------|                  |--------|
 *          7000::2                      7000::1
 *          Default Gw: 7000::1
 *
 *  (a) Interface Status is bought down on the EVM. This should ensure that the socket
 *      notification function is invoked with the appropriate reason.
 *  (b) Interface Status is bought up on the EVM. This should ensure that the socket
 *      notification function is invoked with the appropriate reason.
 *  (c) Interface MTU is bought up on the EVM. This should ensure that the socket
 *      notification function is invoked with the appropriate reason.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Test_v6InterfaceEvents(void)
{
    Netfp_InboundFPCfg  inboundFPConfig;
    Netfp_OutboundFPCfg outboundFPConfig;
    int32_t             errCode;
    int32_t             status;
    Netfp_SockAddr      sockAddress;
    Netfp_OptionTLV     optInfo;
    uint8_t             socketState;
    uint32_t            socketMTU;
    int32_t             index = 0;
    char                srcIP[40];
    char                dstIP[40];

    /* Debug Message: */
    printf ("***************************************************************************************\n");
    printf ("Debug: Testing non secure fast paths with the interface events:\n");

    /*************************************************************************************************
     * TEST SETUP:
     *************************************************************************************************/
    system("ifconfig eth0 192.168.1.2");
    system("ifconfig eth0 7000::2/64");

    /* Create the Ingress fast paths: Initialize the fast path configuration */
    memset ((void *)&inboundFPConfig, 0, sizeof (Netfp_InboundFPCfg));

    /* Populate the fast path configuration: */
    inboundFPConfig.spId                       = NETFP_INVALID_SPID;
    inboundFPConfig.dstIP.ver                  = Netfp_IPVersion_IPV6;
    inboundFPConfig.dstIP.addr.ipv6.u.a32[0]    = 0x00000070;
    inboundFPConfig.dstIP.addr.ipv6.u.a32[1]    = 0;
    inboundFPConfig.dstIP.addr.ipv6.u.a32[2]    = 0;
    inboundFPConfig.dstIP.addr.ipv6.u.a32[3]    = 0x02000000;
    inboundFPConfig.srcIP.ver                  = Netfp_IPVersion_IPV6;
    inboundFPConfig.srcIP.addr.ipv6.u.a32[0]    = 0x00000070;
    inboundFPConfig.srcIP.addr.ipv6.u.a32[1]    = 0;
    inboundFPConfig.srcIP.addr.ipv6.u.a32[2]    = 0;
    inboundFPConfig.srcIP.addr.ipv6.u.a32[3]    = 0x01000000;
    snprintf (inboundFPConfig.name, NETFP_MAX_CHAR, "Ingress_FastPath-%d", index);

    /* Add the Ingress Fast Path. */
    ingressFPHandle = Netfp_createInboundFastPath(netfpClientHandle, &inboundFPConfig, &errCode);
    if (ingressFPHandle == NULL)
    {
        printf ("Error: Unable to add the NETFP Inbound Fast path [Error code %d]\n", errCode);
        return -1;
    }

    /* Convert the Src & Dst IP address of the SA into strings using the NETFP IPv6 Utility API */
    Netfp_convertIP6ToStr (inboundFPConfig.srcIP.addr.ipv6, &srcIP[0]);
    Netfp_convertIP6ToStr (inboundFPConfig.dstIP.addr.ipv6, &dstIP[0]);

    printf ("Debug: Adding %s [%s] -> [%s]\n", inboundFPConfig.name, srcIP, dstIP);

    /* Resolve the route to each possible destination:
     * - These resolutions will end up getting resolved into the default gateway. */
    memset ((void *)&outboundFPConfig, 0, sizeof (Netfp_OutboundFPCfg));

    /* Populate the Fast Path configuration. */
    outboundFPConfig.spId                       = NETFP_INVALID_SPID;
    outboundFPConfig.dstIP.ver                  = Netfp_IPVersion_IPV6;
    outboundFPConfig.dstIP.addr.ipv6.u.a32[0]    = 0x00000070;
    outboundFPConfig.dstIP.addr.ipv6.u.a32[1]    = 0;
    outboundFPConfig.dstIP.addr.ipv6.u.a32[2]    = 0;
    outboundFPConfig.dstIP.addr.ipv6.u.a32[3]    = (0x01000000 + index);
    outboundFPConfig.srcIP.ver                  = Netfp_IPVersion_IPV6;
    outboundFPConfig.srcIP.addr.ipv6.u.a32[0]    = 0x00000070;
    outboundFPConfig.srcIP.addr.ipv6.u.a32[1]    = 0;
    outboundFPConfig.srcIP.addr.ipv6.u.a32[2]    = 0;
    outboundFPConfig.srcIP.addr.ipv6.u.a32[3]    = 0x02000000;
    snprintf (outboundFPConfig.name, NETFP_MAX_CHAR, "Egress_FastPath-%d", index);

    /* Add the Egress Fast Path. */
    egressFPHandle[index] = Netfp_createOutboundFastPath (netfpClientHandle, &outboundFPConfig, &errCode);
    if (egressFPHandle[index] == NULL)
    {
        printf ("Error: Unable to add the NETFP Egress Fast path [Error code %d]\n", errCode);
        return -1;
    }

    /* Convert the Src & Dst IP address of the SA into strings using the NETFP IPv6 Utility API */
    Netfp_convertIP6ToStr (outboundFPConfig.srcIP.addr.ipv6, &srcIP[0]);
    Netfp_convertIP6ToStr (outboundFPConfig.dstIP.addr.ipv6, &dstIP[0]);

    printf ("Debug: Adding %s [%s] -> [%s]\n", outboundFPConfig.name, srcIP, dstIP);

    /* Wait for fast path to become active? */
    while (1)
    {
        /* Get the fast path status: */
        if (Netfp_isOutboundFastPathActive (netfpClientHandle, egressFPHandle[index], &status, &errCode) < 0)
        {
            printf ("Error: Getting status of the outbound fast path failed [Error code %d]\n", errCode);
            return -1;
        }

        /* Is the fast path active? */
        if (status == 1)
            break;

        /* Sleep and try again after some time. */
        usleep(1000*20);
    }
    printf ("Debug: Egress IPv6 Fast Path Handle 0x%x is active\n", (uint32_t)egressFPHandle[index]);

    /* Create a socket */
    sockHandle[index] = Netfp_socket (netfpClientHandle, Netfp_SockFamily_AF_INET6, &errCode);
    if (sockHandle[index] == NULL)
    {
        printf ("Error: NETFP Socket '%d' Creation Failed [Error Code %d]\n", index, errCode);
        return -1;
    }

    /* Populate the binding information: */
    memset ((void*)&sockAddress, 0, sizeof(Netfp_SockAddr));
    sockAddress.sin_family              = Netfp_SockFamily_AF_INET6;
    sockAddress.sin_port                = 4000 + index;
    sockAddress.op.bind.inboundFPHandle = ingressFPHandle;
    sockAddress.op.bind.flowId          = dummyFlowId;
    sockAddress.op.bind.queueHandle     = Msgcom_getInternalMsgQueueInfo(dummyChannel);
    sockAddress.op.bind.notifyFunction  = Test_notifySocketFunction;

    /* Bind the socket. */
    if (Netfp_bind (sockHandle[index], &sockAddress, &errCode) < 0)
    {
        printf ("Error: NETFP Bind Failed for socket '%d' on port '%d' [Error Code %d]\n",
                index, sockAddress.sin_port, errCode);
        return -1;
    }

    /* Populate the connect information. */
    sockAddress.sin_family                  = Netfp_SockFamily_AF_INET6;
    sockAddress.sin_port                    = 4000 + index;
    sockAddress.op.connect.outboundFPHandle = egressFPHandle[index];

    /* Connect the socket */
    if (Netfp_connect(sockHandle[index], &sockAddress, &errCode) < 0)
    {
        printf ("Error: NETFP Connect Failed for socket '%d' to port '%d' [Error Code %d]\n",
                index, sockAddress.sin_port, errCode);
        return -1;
    }
    printf ("Debug: Socket Handle 0x%x has been bound and connected successfully to %d\n", (uint32_t)sockHandle[index], sockAddress.sin_port);

#if (defined (DEVICE_K2K) || defined (DEVICE_K2H))
    /*************************************************************************************************
     * EVENT GENERATION: Bring down the interface
     *************************************************************************************************/
    printf ("Debug: Initiating the interface down test\n");

    /* Cycle through and reset the socket reasons: We are about to start the test */
    sockReason[index] = 0;

    /* We are interested in logging: */
    loggingFlag = 1;

    system ("ifconfig eth0 down");
    sleep(10);

    /*************************************************************************************************
     * VALIDATIONS: We should have detected a valid reason on each socket.
     *************************************************************************************************/
    if (sockReason[index] == 0)
    {
        /* Error: No reason detected for the socket. */
        printf ("Error: Socket handle 0x%x index %d was not notified\n", (uint32_t)sockHandle[index], index);
        return -1;
    }

    /* Each socket should have received a routing update event which indicates that the interface is down */
    if (sockReason[index] != Netfp_Reason_IF_DOWN)
    {
        printf ("Error: Socket handle 0x%x index %d reason %d was not expected\n", (uint32_t)sockHandle[index], index, sockReason[index]);
        return -1;
    }

    /* Each socket should no longer be active: */
    optInfo.type   = Netfp_Option_STATE;
    optInfo.length = 1;
    optInfo.value  = (void*)&socketState;

    /* Get the socket option: */
    if (Netfp_getSockOpt (sockHandle[index], &optInfo, &errCode) < 0)
    {
        printf ("Error: Getting socket option failed [Error code %d]\n", errCode);
        return -1;
    }

    /* Sanity Check: Ensure that the socket is NOT active */
    if (socketState == 1)
    {
        printf ("Error: Socket handle 0x%x index %d invalid socket state %d detected\n", (uint32_t)sockHandle[index], index, socketState);
        return -1;
    }

    /*************************************************************************************************
     * EVENT GENERATION: Bring the interface back up
     *************************************************************************************************/
    printf ("Debug: Bringing the interface back up\n");

    /* Cycle through and reset the socket reasons: We are about to start the test */
    sockReason[index] = 0;

    /* We are interested in logging: */
    loggingFlag = 1;

    system ("ifconfig eth0 up");
    sleep(10);

    /*************************************************************************************************
     * VALIDATIONS: We should have detected a valid reason on each socket.
     *************************************************************************************************/
    if (sockReason[index] == 0)
    {
        /* Error: No reason detected for the socket. */
        printf ("Error: Socket handle 0x%x index %d was not notified\n", (uint32_t)sockHandle[index], index);
        return -1;
    }

    /* Each socket should have received a routing update event which indicates that the interface is up */
    if (sockReason[index] != Netfp_Reason_IF_UP)
    {
        printf ("Error: Socket handle 0x%x index %d reason %d was not expected\n", (uint32_t)sockHandle[index], index, sockReason[index]);
        return -1;
    }

    /* Each socket should be active: */
    optInfo.type   = Netfp_Option_STATE;
    optInfo.length = 1;
    optInfo.value  = (void*)&socketState;

    /* Get the socket option: */
    if (Netfp_getSockOpt (sockHandle[index], &optInfo, &errCode) < 0)
    {
        printf ("Error: Getting socket option failed [Error code %d]\n", errCode);
        return -1;
    }

    /* Sanity Check: Ensure that the socket is active */
    if (socketState == 0)
    {
        printf ("Error: Socket handle 0x%x index %d invalid socket state %d detected\n", (uint32_t)sockHandle[index], index, socketState);
        return -1;
    }
#else
    printf ("WARNING: Interface UP/DOWN Test skipped on K2L\n");
#endif

    /*************************************************************************************************
     * EVENT GENERATION: Changing the MTU
     *************************************************************************************************/
    printf ("Debug: Changing the interface MTU\n");

    /* Cycle through and reset the socket reasons: We are about to start the test */
    sockReason[index] = 0;

    /* We are interested in logging: */
    loggingFlag = 1;

    system ("ifconfig eth0 mtu 1400");
    sleep(10);

    /*************************************************************************************************
     * VALIDATIONS: We should have detected a valid reason on each socket.
     *************************************************************************************************/
    if (sockReason[index] == 0)
    {
        /* Error: No reason detected for the socket. */
        printf ("Error: Socket handle 0x%x index %d was not notified\n", (uint32_t)sockHandle[index], index);
        return -1;
    }

    /* Each socket should have received a routing update event which indicates that the interface is up */
    if (sockReason[index] != Netfp_Reason_IF_MTU_CHANGE)
    {
        printf ("Error: Socket handle 0x%x index %d reason %d was not expected\n", (uint32_t)sockHandle[index], index, sockReason[index]);
        return -1;
    }

    /* Each socket should be active: */
    optInfo.type   = Netfp_Option_STATE;
    optInfo.length = 1;
    optInfo.value  = (void*)&socketState;

    /* Get the socket option: */
    if (Netfp_getSockOpt (sockHandle[index], &optInfo, &errCode) < 0)
    {
        printf ("Error: Getting socket option failed [Error code %d]\n", errCode);
        return -1;
    }

    /* Sanity Check: Ensure that the socket is active */
    if (socketState == 0)
    {
        printf ("Error: Socket handle 0x%x index %d invalid socket state %d detected\n", (uint32_t)sockHandle[index], index, socketState);
        return -1;
    }

    /* Get the socket MTU associated with the socket. This should be the same as the interface MTU */
    optInfo.type   = Netfp_Option_SOCK_MTU;
    optInfo.length = 4;
    optInfo.value  = (void*)&socketMTU;

    /* Get the socket option: */
    if (Netfp_getSockOpt (sockHandle[index], &optInfo, &errCode) < 0)
    {
        printf ("Error: Getting socket option failed [Error code %d]\n", errCode);
        return -1;
    }

    if (socketMTU != 1400)
    {
        printf ("Error: Socket handle 0x%x invalid socket MTU %d detected\n", (uint32_t)sockHandle[index], socketMTU);
        return -1;
    }
    printf ("Debug: Socket MTU %d validated successfully\n", socketMTU);

    /*************************************************************************************************
     * CLEANUP:
     *************************************************************************************************/
    /* Delete the socket: */
    if (Netfp_closeSocket (sockHandle[index], &errCode) < 0)
        printf ("Error: NETFP Close socket failed [Error code %d]\n", errCode);

    /* Delete the ingress fast path: */
    if (Netfp_deleteInboundFastPath (netfpClientHandle, ingressFPHandle, &errCode) < 0)
        printf ("Error: NETFP Ingress fast path deletion failed [Error code %d]\n", errCode);

    /* Delete the egress fast path: */
    if (Netfp_deleteOutboundFastPath (netfpClientHandle, egressFPHandle[index], &errCode) < 0)
        printf ("Error: NETFP Egress fast path deletion failed [Error code %d]\n", errCode);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to test the route recalculation for non-secure fast paths
 *
 *      The following is the test setup which is constructed here:
 *
 *      |----------|                  |--------|---------- 8000::x
 *      |   EVM    |------------------| GW     |
 *      |----------|                  |--------|
 *          7000::2                      7000::1
 *          Default Gw: 7000::1
 *
 *  (a) Default Gateway is deleted. This implies that the sockets connected to the
 *      8000::x network will no longer be accessible and the NOTIFY function should
 *      indicate that the socket is UNREACHABLE.
 *  (b) Default Gateway is added back. This implies that the sockets connected to the
 *      8000::x network will be accessible and the NOTIFY function should indicate
 *      that the socket is REACHABLE.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Test_v6NonSecureFPEvents(void)
{
    int32_t             index;
    Netfp_InboundFPCfg  inboundFPConfig;
    Netfp_OutboundFPCfg outboundFPConfig;
    int32_t             errCode;
    int32_t             status;
    Netfp_SockAddr      sockAddress;
    Netfp_OptionTLV     optInfo;
    uint8_t             socketState;
    char                srcIP[40];
    char                dstIP[40];

    /* Debug Message: */
    printf ("***************************************************************************************\n");
    printf ("Debug: Testing non secure fast paths with the deletion and addition of default gateway.\n");

    /*************************************************************************************************
     * TEST SETUP:
     *************************************************************************************************/
    system("ifconfig eth0 192.168.1.2");
    system("ifconfig eth0 7000::2/64");
    system("ip route add default via 7000::1");

    /* Initialize the fast path configuration */
    memset ((void *)&inboundFPConfig, 0, sizeof (Netfp_InboundFPCfg));

    /* Populate the fast path configuration: */
    inboundFPConfig.spId                       = NETFP_INVALID_SPID;
    inboundFPConfig.dstIP.ver                  = Netfp_IPVersion_IPV6;
    inboundFPConfig.dstIP.addr.ipv6.u.a32[0]    = 0x00000070;
    inboundFPConfig.dstIP.addr.ipv6.u.a32[1]    = 0;
    inboundFPConfig.dstIP.addr.ipv6.u.a32[2]    = 0;
    inboundFPConfig.dstIP.addr.ipv6.u.a32[3]    = 0x02000000;
    inboundFPConfig.srcIP.ver                  = Netfp_IPVersion_IPV6;
    inboundFPConfig.srcIP.addr.ipv6.u.a32[0]    = 0x00000080;
    inboundFPConfig.srcIP.addr.ipv6.u.a32[1]    = 0;
    inboundFPConfig.srcIP.addr.ipv6.u.a32[2]    = 0;
    inboundFPConfig.srcIP.addr.ipv6.u.a32[3]    = 0x01000000;
    snprintf (inboundFPConfig.name, NETFP_MAX_CHAR, "Ingress_FastPath");

    /* Add the Ingress Fast Path. */
    ingressFPHandle = Netfp_createInboundFastPath(netfpClientHandle, &inboundFPConfig, &errCode);
    if (ingressFPHandle == NULL)
    {
        printf ("Error: Unable to add the NETFP Inbound Fast path [Error code %d]\n", errCode);
        return -1;
    }

    /* Convert the Src & Dst IP address of the SA into strings using the NETFP IPv6 Utility API */
    Netfp_convertIP6ToStr (inboundFPConfig.srcIP.addr.ipv6, &srcIP[0]);
    Netfp_convertIP6ToStr (inboundFPConfig.dstIP.addr.ipv6, &dstIP[0]);

    printf ("Debug: Adding %s [%s] -> [%s]\n", inboundFPConfig.name, srcIP, dstIP);

    /* Resolve the route to each possible destination:
     * - These resolutions will end up getting resolved into the default gateway. */
    for (index = 0; index < TEST_MAX_FAST_PATH; index++)
    {
        /* Initialize the fast path configuration. */
        memset ((void *)&outboundFPConfig, 0, sizeof (Netfp_OutboundFPCfg));

        /* Populate the Fast Path configuration. */
        outboundFPConfig.spId                       = NETFP_INVALID_SPID;
        outboundFPConfig.dstIP.ver                  = Netfp_IPVersion_IPV6;
        outboundFPConfig.dstIP.addr.ipv6.u.a32[0]    = 0x00000080;
        outboundFPConfig.dstIP.addr.ipv6.u.a32[1]    = 0;
        outboundFPConfig.dstIP.addr.ipv6.u.a32[2]    = 0;
        outboundFPConfig.dstIP.addr.ipv6.u.a32[3]    = (0x01000000 + index);
        outboundFPConfig.srcIP.ver                  = Netfp_IPVersion_IPV6;
        outboundFPConfig.srcIP.addr.ipv6.u.a32[0]    = 0x00000070;
        outboundFPConfig.srcIP.addr.ipv6.u.a32[1]    = 0;
        outboundFPConfig.srcIP.addr.ipv6.u.a32[2]    = 0;
        outboundFPConfig.srcIP.addr.ipv6.u.a32[3]    = 0x02000000;
        snprintf (outboundFPConfig.name, NETFP_MAX_CHAR, "Egress_FastPath-%d", index);

        /* Add the Egress Fast Path. */
        egressFPHandle[index] = Netfp_createOutboundFastPath (netfpClientHandle, &outboundFPConfig, &errCode);
        if (egressFPHandle[index] == NULL)
        {
            printf ("Error: Unable to add the NETFP Egress Fast path [Error code %d]\n", errCode);
            return -1;
        }

        /* Convert the Src & Dst IP address of the SA into strings using the NETFP IPv6 Utility API */
        Netfp_convertIP6ToStr (outboundFPConfig.srcIP.addr.ipv6, &srcIP[0]);
        Netfp_convertIP6ToStr (outboundFPConfig.dstIP.addr.ipv6, &dstIP[0]);

        printf ("Debug: Adding %s [%s] -> [%s]\n", outboundFPConfig.name, srcIP, dstIP);

        /* Wait for fast path to become active? */
        while (1)
        {
            /* Get the fast path status: */
            if (Netfp_isOutboundFastPathActive (netfpClientHandle, egressFPHandle[index], &status, &errCode) < 0)
            {
                printf ("Error: Getting status of the outbound fast path failed [Error code %d]\n", errCode);
                return -1;
            }

            /* Is the fast path active? */
            if (status == 1)
                break;

            /* Sleep and try again after some time. */
            usleep(1000*20);
        }
        printf ("Debug: Egress IPv6 Fast Path Handle %s is active\n", outboundFPConfig.name);
    }

    /* Create all the sockets on each of the fast paths: */
    for (index = 0; index < TEST_MAX_FAST_PATH; index++)
    {
        /* Create a socket */
        sockHandle[index] = Netfp_socket (netfpClientHandle, Netfp_SockFamily_AF_INET6, &errCode);
        if (sockHandle[index] == NULL)
        {
            printf ("Error: NETFP Socket '%d' Creation Failed [Error Code %d]\n", index, errCode);
            return -1;
        }

        /* Populate the binding information: */
        memset ((void*)&sockAddress, 0, sizeof(Netfp_SockAddr));
        sockAddress.sin_family              = Netfp_SockFamily_AF_INET6;
        sockAddress.sin_port                = 4000 + index;
        sockAddress.op.bind.inboundFPHandle = ingressFPHandle;
        sockAddress.op.bind.flowId          = dummyFlowId;
        sockAddress.op.bind.queueHandle     = Msgcom_getInternalMsgQueueInfo(dummyChannel);
        sockAddress.op.bind.notifyFunction  = Test_notifySocketFunction;

        /* Bind the socket. */
        if (Netfp_bind (sockHandle[index], &sockAddress, &errCode) < 0)
        {
            printf ("Error: NETFP Bind Failed for socket '%d' on port '%d' [Error Code %d]\n",
                    index, sockAddress.sin_port, errCode);
            return -1;
        }

        /* Populate the connect information. */
        sockAddress.sin_family                  = Netfp_SockFamily_AF_INET6;
        sockAddress.sin_port                    = 4000 + index;
        sockAddress.op.connect.outboundFPHandle = egressFPHandle[index];

        /* Connect the socket */
        if (Netfp_connect(sockHandle[index], &sockAddress, &errCode) < 0)
        {
            printf ("Error: NETFP Connect Failed for socket '%d' to port '%d' [Error Code %d]\n",
                    index, sockAddress.sin_port, errCode);
            return -1;
        }
        printf ("Debug: Socket Handle 0x%x has been bound and connected successfully to %d\n", (uint32_t)sockHandle[index], sockAddress.sin_port);
    }

    /*************************************************************************************************
     * EVENT GENERATION: Delete the default gateway
     *************************************************************************************************/
    printf ("Debug: Initiating the delete default gateway\n");

    /* Cycle through and reset the socket reasons: We are about to start the test */
    for (index = 0; index < TEST_MAX_FAST_PATH; index++)
        sockReason[index] = 0;

    system ("ip route del default via 7000::1");

    /* Send the FLUSH command to Netfp Proxy */
    if (Test_executeNetfpProxyCommand() < 0)
    {
        printf ("Error: Failed to send Flush IP routes command to NetfpProxy\n");
        return -1;
    }
    sleep(3);

    /*************************************************************************************************
     * VALIDATIONS: We should have detected a valid reason on each socket.
     *************************************************************************************************/
    for (index = 0; index < TEST_MAX_FAST_PATH; index++)
    {
        /* Each socket should have had a valid reason: */
        if (sockReason[index] == 0)
        {
            /* Error: No reason detected for the socket. */
            printf ("Error: Socket handle 0x%x index %d was not notified\n", (uint32_t)sockHandle[index], index);
            return -1;
        }

        /* Each socket should have received a routing update event which indicates that the neighbor is
         * no longer reachable */
        if (sockReason[index] != Netfp_Reason_NEIGH_UNREACHABLE)
        {
            printf ("Error: Socket handle 0x%x index %d reason %d was not expected\n", (uint32_t)sockHandle[index], index, sockReason[index]);
            return -1;
        }

        /* Each socket should no longer be active: */
        optInfo.type   = Netfp_Option_STATE;
        optInfo.length = 1;
        optInfo.value  = (void*)&socketState;

        /* Get the socket option: */
        if (Netfp_getSockOpt (sockHandle[index], &optInfo, &errCode) < 0)
        {
            printf ("Error: Getting socket option failed [Error code %d]\n", errCode);
            return -1;
        }

        /* Sanity Check: Ensure that the socket is NOT active */
        if (socketState == 1)
        {
            printf ("Error: Socket handle 0x%x index %d invalid socket state %d detected\n", (uint32_t)sockHandle[index], index, socketState);
            return -1;
        }
    }

    /*************************************************************************************************
     * EVENT GENERATION: Add the default gateway
     *************************************************************************************************/
    printf ("Debug: Initiating the adding default gateway\n");

    /* Cycle through and reset the socket reasons: We are about to start the test */
    for (index = 0; index < TEST_MAX_FAST_PATH; index++)
        sockReason[index] = 0;

    system ("ip route add default via 7000::1");

    /* Send the FLUSH command to Netfp Proxy */
    if (Test_executeNetfpProxyCommand() < 0)
    {
        printf ("Error: Failed to send Flush IP routes command to NetfpProxy\n");
        return -1;
    }
    sleep(3);

    /*************************************************************************************************
     * VALIDATIONS: We should have detected a valid reason on each socket.
     *************************************************************************************************/
    for (index = 0; index < TEST_MAX_FAST_PATH; index++)
    {
        /* Each socket should have had a valid reason: */
        if (sockReason[index] == 0)
        {
            /* Error: No reason detected for the socket. */
            printf ("Error: Socket handle 0x%x index %d was not notified\n", (uint32_t)sockHandle[index], index);
            return -1;
        }

        /* Each socket should have received a routing update event which indicates that the neighbor is
         * no longer reachable */
        if (sockReason[index] != Netfp_Reason_NEIGH_REACHABLE)
        {
            printf ("Error: Socket handle 0x%x index %d reason %d was not expected\n", (uint32_t)sockHandle[index], index, sockReason[index]);
            return -1;
        }

        /* Each socket should no longer be active: */
        optInfo.type   = Netfp_Option_STATE;
        optInfo.length = 1;
        optInfo.value  = (void*)&socketState;

        /* Get the socket option: */
        if (Netfp_getSockOpt (sockHandle[index], &optInfo, &errCode) < 0)
        {
            printf ("Error: Getting socket option failed [Error code %d]\n", errCode);
            return -1;
        }

        /* Sanity Check: Ensure that the socket is active */
        if (socketState == 0)
        {
            printf ("Error: Socket handle 0x%x index %d invalid socket state %d detected\n", (uint32_t)sockHandle[index], index, socketState);
            return -1;
        }
    }

    /*************************************************************************************************
     * CLEANUP:
     *************************************************************************************************/

    /* Delete the ingress fast path: */
    if (Netfp_deleteInboundFastPath (netfpClientHandle, ingressFPHandle, &errCode) < 0)
        printf ("Error: NETFP Ingress fast path deletion failed [Error code %d]\n", errCode);

    for (index = 0; index < TEST_MAX_FAST_PATH; index++)
    {
        /* Delete the socket: */
        if (Netfp_closeSocket (sockHandle[index], &errCode) < 0)
            printf ("Error: NETFP Close socket failed [Error code %d]\n", errCode);

        /* Delete the egress fast path: */
        if (Netfp_deleteOutboundFastPath (netfpClientHandle, egressFPHandle[index], &errCode) < 0)
            printf ("Error: NETFP Egress fast path deletion failed [Error code %d]\n", errCode);
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to test the route recalculation and event management
 *      for secure fast paths
 *
 *      The following is the test setup which is constructed here:
 *
 *  100.100.100.2                                         |-----------|
 *      |----------|                  |--------|----------|Serving-GW |
 *      |   EVM    |------------------| SecGW  |          |-----------|
 *      |----------|                  |--------|            8000::1
 *          7000::2                     7000::1
 *          Default Gw: 7000::1
 *
 *  There is a tunnel between the EVM and SecGW. Inner IP address of the
 *  EVM (9000:2) is communicating with the Serving-Gateway @ 8000::1. The
 *  SA will have a next hop neighbor to the SecGW and the fast path to the serving
 *  gateway will inherit the next hop neighbor.
 *
 *  The test case will perform the following:
 *  (a) Default Gateway is deleted. This should have no impact on the socket
 *      because it inherits the neighbor information from the Outbound SA.
 *  (b) Default Gateway is added. This should have no impact on the socket
 *      because it inherits the neighbor information from the Outbound SA.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Test_v6SecureFPEvents(void)
{
    int32_t             index = 0;
    Netfp_InboundFPCfg  inboundFPConfig;
    Netfp_OutboundFPCfg outboundFPConfig;
    int32_t             errCode;
    int32_t             status;
    Netfp_SockAddr      sockAddress;
    Netfp_OptionTLV     optInfo;
    uint8_t             socketState;
    Netfp_SACfg         saConfig;
    Netfp_SAHandle      inboundSAHandle;
    Netfp_SAHandle      outboundSAHandle;
    Netfp_SPCfg         spConfig;
    char                srcIP[40];
    char                dstIP[40];
    uint8_t encKey[32] =
        {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11,
         0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11};
    uint8_t authKey[32] =
        {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11,
         0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11};

    /* Debug Message: */
    printf ("***************************************************************************************\n");
    printf ("Debug: Testing secure fast paths with the deletion and addition of default gateway.\n");

    /*************************************************************************************************
     * TEST SETUP:
     *************************************************************************************************/
    system("ifconfig eth0 192.168.1.2");
    system("ifconfig eth0 7000::2/64");
    system("ifconfig eth0:2 9000::2/64");
    system("ip route add default via 7000::1");

    /* Create the INBOUND Security association: Initialize the SA Configuration. */
    memset ((void *)&saConfig, 0, sizeof (Netfp_SACfg));

    /* Populate the SA Configuration. */
    saConfig.direction              = Netfp_Direction_INBOUND;
    saConfig.spi                    = 100;
    saConfig.ipsecCfg.protocol      = Netfp_IPSecProto_IPSEC_ESP;
    saConfig.ipsecCfg.mode          = Netfp_IPSecMode_IPSEC_TUNNEL;
    saConfig.ipsecCfg.authMode      = Netfp_IpsecAuthMode_HMAC_SHA1;
    saConfig.ipsecCfg.encMode       = Netfp_IpsecCipherMode_NULL;
    saConfig.ipsecCfg.keyMacSize    = 12;

    if (saConfig.ipsecCfg.authMode == Netfp_IpsecAuthMode_HMAC_MD5)
        saConfig.ipsecCfg.keyAuthSize   = 16;
    else if (saConfig.ipsecCfg.authMode == Netfp_IpsecAuthMode_HMAC_SHA1)
        saConfig.ipsecCfg.keyAuthSize   = 20;
    else if (saConfig.ipsecCfg.authMode == Netfp_IpsecAuthMode_AES_XCBC)
        saConfig.ipsecCfg.keyAuthSize   = 12;
    else if (saConfig.ipsecCfg.authMode == Netfp_IpsecAuthMode_HMAC_SHA2_256)
        saConfig.ipsecCfg.keyAuthSize   = 30;
    else
        saConfig.ipsecCfg.keyAuthSize   = 0;

    if (saConfig.ipsecCfg.encMode == Netfp_IpsecCipherMode_NULL)
        saConfig.ipsecCfg.keyEncSize = 0;
    else if (saConfig.ipsecCfg.encMode == Netfp_IpsecCipherMode_AES_CTR)
        saConfig.ipsecCfg.keyEncSize = 20;
    else
        saConfig.ipsecCfg.keyEncSize = 24;

    /* Copy the keys. */
    memcpy (&saConfig.ipsecCfg.keyAuth, &authKey[0], saConfig.ipsecCfg.keyAuthSize);
    memcpy (&saConfig.ipsecCfg.keyEnc,  &encKey[0],  saConfig.ipsecCfg.keyEncSize);

    /* Configure the lifetime */
    saConfig.ipsecCfg.lifetime.softByteLimit   = 0xFFFFFFFFFFFFFFFF;
    saConfig.ipsecCfg.lifetime.hardByteLimit   = 0xFFFFFFFFFFFFFFFF;
    saConfig.ipsecCfg.lifetime.softPacketLimit = 0xFFFFFFFFFFFFFFFF;
    saConfig.ipsecCfg.lifetime.hardPacketLimit = 0xFFFFFFFFFFFFFFFF;

    /* Populate the source & destination IP address for the INBOUND SA */
    saConfig.srcIP.ver = Netfp_IPVersion_IPV6;
    saConfig.srcIP.addr.ipv6.u.a32[0] = 0x00000070;
    saConfig.srcIP.addr.ipv6.u.a32[1] = 0;
    saConfig.srcIP.addr.ipv6.u.a32[2] = 0;
    saConfig.srcIP.addr.ipv6.u.a32[3] = 0x01000000;

    saConfig.dstIP.ver = Netfp_IPVersion_IPV6;
    saConfig.dstIP.addr.ipv6.u.a32[0] = 0x00000070;
    saConfig.dstIP.addr.ipv6.u.a32[1] = 0;
    saConfig.dstIP.addr.ipv6.u.a32[2] = 0;
    saConfig.dstIP.addr.ipv6.u.a32[3] = 0x02000000;

    /* Create the SA. */
    inboundSAHandle = Netfp_addSA (netfpClientHandle, &saConfig, &errCode);
    if (inboundSAHandle == NULL)
    {
        printf ("Error: Unable to the create inbound SA [Error code %d]\n", errCode);
        return -1;
    }

    /* Create the OUTBOUND Security association: Initialize the SA Configuration. */
    memset ((void *)&saConfig, 0, sizeof (Netfp_SACfg));

    /* Populate the SA Configuration. */
    saConfig.direction              = Netfp_Direction_OUTBOUND;
    saConfig.spi                    = 200;
    saConfig.ipsecCfg.protocol      = Netfp_IPSecProto_IPSEC_ESP;
    saConfig.ipsecCfg.mode          = Netfp_IPSecMode_IPSEC_TUNNEL;
    saConfig.ipsecCfg.authMode      = Netfp_IpsecAuthMode_HMAC_SHA1;
    saConfig.ipsecCfg.encMode       = Netfp_IpsecCipherMode_NULL;
    saConfig.ipsecCfg.keyMacSize    = 12;

    if (saConfig.ipsecCfg.authMode == Netfp_IpsecAuthMode_HMAC_MD5)
        saConfig.ipsecCfg.keyAuthSize   = 16;
    else if (saConfig.ipsecCfg.authMode == Netfp_IpsecAuthMode_HMAC_SHA1)
        saConfig.ipsecCfg.keyAuthSize   = 20;
    else if (saConfig.ipsecCfg.authMode == Netfp_IpsecAuthMode_AES_XCBC)
        saConfig.ipsecCfg.keyAuthSize   = 12;
    else if (saConfig.ipsecCfg.authMode == Netfp_IpsecAuthMode_HMAC_SHA2_256)
        saConfig.ipsecCfg.keyAuthSize   = 30;
    else
        saConfig.ipsecCfg.keyAuthSize   = 0;

    if (saConfig.ipsecCfg.encMode == Netfp_IpsecCipherMode_NULL)
        saConfig.ipsecCfg.keyEncSize = 0;
    else if (saConfig.ipsecCfg.encMode == Netfp_IpsecCipherMode_AES_CTR)
        saConfig.ipsecCfg.keyEncSize = 20;
    else
        saConfig.ipsecCfg.keyEncSize = 24;

    /* Copy the keys. */
    memcpy (&saConfig.ipsecCfg.keyAuth, &authKey[0], saConfig.ipsecCfg.keyAuthSize);
    memcpy (&saConfig.ipsecCfg.keyEnc,  &encKey[0],  saConfig.ipsecCfg.keyEncSize);

    /* Configure the lifetime */
    saConfig.ipsecCfg.lifetime.softByteLimit   = 0xFFFFFFFFFFFFFFFF;
    saConfig.ipsecCfg.lifetime.hardByteLimit   = 0xFFFFFFFFFFFFFFFF;
    saConfig.ipsecCfg.lifetime.softPacketLimit = 0xFFFFFFFFFFFFFFFF;
    saConfig.ipsecCfg.lifetime.hardPacketLimit = 0xFFFFFFFFFFFFFFFF;

    /* Populate the source & destination IP address for the INBOUND SA */
    saConfig.srcIP.ver = Netfp_IPVersion_IPV6;
    saConfig.srcIP.addr.ipv6.u.a32[0] = 0x00000070;
    saConfig.srcIP.addr.ipv6.u.a32[1] = 0;
    saConfig.srcIP.addr.ipv6.u.a32[2] = 0;
    saConfig.srcIP.addr.ipv6.u.a32[3] = 0x02000000;

    saConfig.dstIP.ver = Netfp_IPVersion_IPV6;
    saConfig.dstIP.addr.ipv6.u.a32[0] = 0x00000070;
    saConfig.dstIP.addr.ipv6.u.a32[1] = 0;
    saConfig.dstIP.addr.ipv6.u.a32[2] = 0;
    saConfig.dstIP.addr.ipv6.u.a32[3] = 0x01000000;

    /* Create the Outbound SA. */
    outboundSAHandle = Netfp_addSA (netfpClientHandle, &saConfig, &errCode);
    if (outboundSAHandle == NULL)
    {
        printf ("Error: Unable to create the outbound SA [Error code %d]\n", errCode);
        return -1;
    }

    /* Create the Inbound security policy: */
    memset ((void *)&spConfig, 0, sizeof (Netfp_SPCfg));

    /* Populate the SP Configuration */
    spConfig.direction               = Netfp_Direction_INBOUND;
    spConfig.saHandle                = inboundSAHandle;
    spConfig.srcIP.ver               = Netfp_IPVersion_IPV6;
    spConfig.srcIP.addr.ipv6.u.a32[0] = 0x00000080;
    spConfig.srcIP.addr.ipv6.u.a32[1] = 0;
    spConfig.srcIP.addr.ipv6.u.a32[2] = 0;
    spConfig.srcIP.addr.ipv6.u.a32[3] = 0x01000000;
    spConfig.dstIP.ver               = Netfp_IPVersion_IPV6;
    spConfig.dstIP.addr.ipv6.u.a32[0] = 0x00000090;
    spConfig.dstIP.addr.ipv6.u.a32[1] = 0;
    spConfig.dstIP.addr.ipv6.u.a32[2] = 0;
    spConfig.dstIP.addr.ipv6.u.a32[3] = 0x02000000;
    spConfig.srcIPPrefixLen          = 24;
    spConfig.dstIPPrefixLen          = 24;
    spConfig.protocol                = 0x32;
    spConfig.srcPortStart            = 0;
    spConfig.srcPortEnd              = 0;
    spConfig.gtpuIdStart             = 0x0;
    spConfig.gtpuIdEnd               = 0x0;
    spConfig.dscpFilter              = 0;
    spConfig.spId                    = 100;

    /* Add the Inbound Security Policy. */
    if (Netfp_addSP (netfpClientHandle, &spConfig, &errCode) < 0)
    {
        printf ("Error: Unable to create Inbound SP Error %d\n", errCode);
        return -1;
    }

    /* Create the Outbound security policy: */
    memset ((void *)&spConfig, 0, sizeof (Netfp_SPCfg));

    /* Populate the SP Configuration */
    spConfig.direction               = Netfp_Direction_OUTBOUND;
    spConfig.saHandle                = outboundSAHandle;
    spConfig.srcIP.ver               = Netfp_IPVersion_IPV6;
    spConfig.srcIP.addr.ipv6.u.a32[0] = 0x00000090;
    spConfig.srcIP.addr.ipv6.u.a32[1] = 0;
    spConfig.srcIP.addr.ipv6.u.a32[2] = 0;
    spConfig.srcIP.addr.ipv6.u.a32[3] = 0x02000000;
    spConfig.dstIP.ver               = Netfp_IPVersion_IPV6;
    spConfig.dstIP.addr.ipv6.u.a32[0] = 0x00000080;
    spConfig.dstIP.addr.ipv6.u.a32[1] = 0;
    spConfig.dstIP.addr.ipv6.u.a32[2] = 0;
    spConfig.dstIP.addr.ipv6.u.a32[3] = 0x01000000;
    spConfig.srcIPPrefixLen          = 24;
    spConfig.dstIPPrefixLen          = 24;
    spConfig.protocol                = 0x32;
    spConfig.srcPortStart            = 0;
    spConfig.srcPortEnd              = 0;
    spConfig.gtpuIdStart             = 0x0;
    spConfig.gtpuIdEnd               = 0x0;
    spConfig.dscpFilter              = 0;
    spConfig.spId                    = 200;

    /* Add the Outbound Security Policy. */
    if (Netfp_addSP (netfpClientHandle, &spConfig, &errCode) < 0)
    {
        printf ("Error: Unable to create outbound SP Error %d\n", errCode);
        return -1;
    }

    /* Create the Ingress fast paths: Initialize the fast path configuration */
    memset ((void *)&inboundFPConfig, 0, sizeof (Netfp_InboundFPCfg));

    /* Populate the fast path configuration: */
    inboundFPConfig.spId                       = 100;
    inboundFPConfig.dstIP.ver                  = Netfp_IPVersion_IPV6;
    inboundFPConfig.dstIP.addr.ipv6.u.a32[0]    = 0x00000090;
    inboundFPConfig.dstIP.addr.ipv6.u.a32[1]    = 0;
    inboundFPConfig.dstIP.addr.ipv6.u.a32[2]    = 0;
    inboundFPConfig.dstIP.addr.ipv6.u.a32[3]    = 0x02000000;
    inboundFPConfig.srcIP.ver                  = Netfp_IPVersion_IPV6;
    inboundFPConfig.srcIP.addr.ipv6.u.a32[0]    = 0x00000080;
    inboundFPConfig.srcIP.addr.ipv6.u.a32[1]    = 0;
    inboundFPConfig.srcIP.addr.ipv6.u.a32[2]    = 0;
    inboundFPConfig.srcIP.addr.ipv6.u.a32[3]    = (0x01000000 + index);
    snprintf (inboundFPConfig.name, NETFP_MAX_CHAR, "Ingress_FastPath-%d", index);

    /* Add the Ingress Fast Path. */
    ingressFPHandle = Netfp_createInboundFastPath(netfpClientHandle, &inboundFPConfig, &errCode);
    if (ingressFPHandle == NULL)
    {
        printf ("Error: Unable to add the NETFP Inbound Fast path [Error code %d]\n", errCode);
        return -1;
    }

    /* Convert the Src & Dst IP address of the SA into strings using the NETFP IPv6 Utility API */
    Netfp_convertIP6ToStr (inboundFPConfig.srcIP.addr.ipv6, &srcIP[0]);
    Netfp_convertIP6ToStr (inboundFPConfig.dstIP.addr.ipv6, &dstIP[0]);

    printf ("Debug: Adding %s [%s] -> [%s]\n", inboundFPConfig.name, srcIP, dstIP);

    /* Resolve the route to each possible destination:
     * - These resolutions will end up getting resolved into the default gateway. */
    memset ((void *)&outboundFPConfig, 0, sizeof (Netfp_OutboundFPCfg));

    /* Populate the Fast Path configuration. */
    outboundFPConfig.spId                       = 200;
    outboundFPConfig.dstIP.ver                  = Netfp_IPVersion_IPV6;
    outboundFPConfig.dstIP.addr.ipv6.u.a32[0]    = 0x00000080;
    outboundFPConfig.dstIP.addr.ipv6.u.a32[1]    = 0;
    outboundFPConfig.dstIP.addr.ipv6.u.a32[2]    = 0;
    outboundFPConfig.dstIP.addr.ipv6.u.a32[3]    = (0x01000000 + index);
    outboundFPConfig.srcIP.ver                  = Netfp_IPVersion_IPV6;
    outboundFPConfig.srcIP.addr.ipv6.u.a32[0]    = 0x00000090;
    outboundFPConfig.srcIP.addr.ipv6.u.a32[1]    = 0;
    outboundFPConfig.srcIP.addr.ipv6.u.a32[2]    = 0;
    outboundFPConfig.srcIP.addr.ipv6.u.a32[3]    = 0x02000000;
    snprintf (outboundFPConfig.name, NETFP_MAX_CHAR, "Egress_FastPath-%d", index);

    /* Add the Egress Fast Path. */
    egressFPHandle[index] = Netfp_createOutboundFastPath (netfpClientHandle, &outboundFPConfig, &errCode);
    if (egressFPHandle[index] == NULL)
    {
        printf ("Error: Unable to add the NETFP Egress Fast path [Error code %d]\n", errCode);
        return -1;
    }

    /* Convert the Src & Dst IP address of the SA into strings using the NETFP IPv6 Utility API */
    Netfp_convertIP6ToStr (outboundFPConfig.srcIP.addr.ipv6, &srcIP[0]);
    Netfp_convertIP6ToStr (outboundFPConfig.dstIP.addr.ipv6, &dstIP[0]);

    printf ("Debug: Adding %s [%s] -> [%s]\n", outboundFPConfig.name, srcIP, dstIP);

    /* Wait for fast path to become active? */
    while (1)
    {
        /* Get the fast path status: */
        if (Netfp_isOutboundFastPathActive (netfpClientHandle, egressFPHandle[index], &status, &errCode) < 0)
        {
            printf ("Error: Getting status of the outbound fast path failed [Error code %d]\n", errCode);
            return -1;
        }

        /* Is the fast path active? */
        if (status == 1)
            break;

        /* Sleep and try again after some time. */
        usleep(1000*20);
    }
    printf ("Debug: Egress IPv6 Fast Path Handle 0x%x is active\n", (uint32_t)egressFPHandle[index]);

    /* Create a socket */
    sockHandle[index] = Netfp_socket (netfpClientHandle, Netfp_SockFamily_AF_INET6, &errCode);
    if (sockHandle[index] == NULL)
    {
        printf ("Error: NETFP Socket '%d' Creation Failed [Error Code %d]\n", index, errCode);
        return -1;
    }

    /* Populate the binding information: */
    memset ((void*)&sockAddress, 0, sizeof(Netfp_SockAddr));
    sockAddress.sin_family              = Netfp_SockFamily_AF_INET6;
    sockAddress.sin_port                = 4000 + index;
    sockAddress.op.bind.inboundFPHandle = ingressFPHandle;
    sockAddress.op.bind.flowId          = dummyFlowId;
    sockAddress.op.bind.queueHandle     = Msgcom_getInternalMsgQueueInfo(dummyChannel);
    sockAddress.op.bind.notifyFunction  = Test_notifySocketFunction;

    /* Bind the socket. */
    if (Netfp_bind (sockHandle[index], &sockAddress, &errCode) < 0)
    {
        printf ("Error: NETFP Bind Failed for socket '%d' on port '%d' [Error Code %d]\n",
                index, sockAddress.sin_port, errCode);
        return -1;
    }

    /* Populate the connect information. */
    sockAddress.sin_family                  = Netfp_SockFamily_AF_INET6;
    sockAddress.sin_port                    = 4000 + index;
    sockAddress.op.connect.outboundFPHandle = egressFPHandle[index];

    /* Connect the socket */
    if (Netfp_connect(sockHandle[index], &sockAddress, &errCode) < 0)
    {
        printf ("Error: NETFP Connect Failed for socket '%d' to port '%d' [Error Code %d]\n",
                index, sockAddress.sin_port, errCode);
        return -1;
    }

    /* Get the socket state: */
    optInfo.type   = Netfp_Option_STATE;
    optInfo.length = 1;
    optInfo.value  = (void*)&socketState;
    if (Netfp_getSockOpt (sockHandle[index], &optInfo, &errCode) < 0)
    {
        printf ("Error: Getting socket option failed [Error code %d]\n", errCode);
        return -1;
    }
    printf ("Debug: Socket Handle 0x%x has been bound and connected successfully to %d [State %d]\n",
            (uint32_t)sockHandle[index], sockAddress.sin_port, socketState);

    /*************************************************************************************************
     * EVENT GENERATION: Delete the default gateway
     *************************************************************************************************/
    printf ("Debug: Initiating the delete default gateway\n");

    /* Cycle through and reset the socket reasons: We are about to start the test */
    sockReason[index] = 0;

    system ("ip route del default via 7000::1");

    /* Send the FLUSH command to Netfp Proxy */
    if (Test_executeNetfpProxyCommand() < 0)
    {
        printf ("Error: Failed to send Flush IP routes command to NetfpProxy\n");
        return -1;
    }
    sleep(10);

    /* Each socket should remain active: */
    optInfo.type   = Netfp_Option_STATE;
    optInfo.length = 1;
    optInfo.value  = (void*)&socketState;

    /* Get the socket option: */
    if (Netfp_getSockOpt (sockHandle[index], &optInfo, &errCode) < 0)
    {
        printf ("Error: Getting socket option failed [Error code %d]\n", errCode);
        return -1;
    }

    /* Sanity Check: Ensure that the socket is active */
    if (socketState == 0)
    {
        printf ("Error: Socket handle 0x%x index %d invalid socket state %d detected\n", (uint32_t)sockHandle[index], index, socketState);
        return -1;
    }

    /*************************************************************************************************
     * EVENT GENERATION: Add the default gateway
     *************************************************************************************************/
    printf ("Debug: Initiating the adding default gateway\n");

    /* Cycle through and reset the socket reasons: We are about to start the test */
    sockReason[index] = 0;

    system ("ip route add default via 7000::1");
    sleep(10);

    /* Each socket should remain active: */
    optInfo.type   = Netfp_Option_STATE;
    optInfo.length = 1;
    optInfo.value  = (void*)&socketState;

    /* Get the socket option: */
    if (Netfp_getSockOpt (sockHandle[index], &optInfo, &errCode) < 0)
    {
        printf ("Error: Getting socket option failed [Error code %d]\n", errCode);
        return -1;
    }

    /* Sanity Check: Ensure that the socket is active */
    if (socketState == 0)
    {
        printf ("Error: Socket handle 0x%x index %d invalid socket state %d detected\n", (uint32_t)sockHandle[index], index, socketState);
        return -1;
    }

    /*************************************************************************************************
     * CLEANUP:
     *************************************************************************************************/
    /* Delete the socket: */
    if (Netfp_closeSocket (sockHandle[index], &errCode) < 0)
        printf ("Error: NETFP Close socket failed [Error code %d]\n", errCode);

    /* Delete the ingress fast path: */
    if (Netfp_deleteInboundFastPath (netfpClientHandle, ingressFPHandle, &errCode) < 0)
        printf ("Error: NETFP Ingress fast path deletion failed [Error code %d]\n", errCode);

    /* Delete the egress fast path: */
    if (Netfp_deleteOutboundFastPath (netfpClientHandle, egressFPHandle[index], &errCode) < 0)
        printf ("Error: NETFP Egress fast path deletion failed [Error code %d]\n", errCode);

    /* Delete the outbound SP */
    if (Netfp_delSP (netfpClientHandle, 200, &errCode) < 0)
        printf ("Error: NETFP Outbound security policy deletion failed [Error code %d]\n", errCode);

    /* Delete the inbound SP */
    if (Netfp_delSP (netfpClientHandle, 100, &errCode) < 0)
        printf ("Error: NETFP Inbound security policy deletion failed [Error code %d]\n", errCode);

    /* Delete the inbound SA */
    if (Netfp_delSA (netfpClientHandle, inboundSAHandle, &errCode) < 0)
        printf ("Error: NETFP Inbound security policy deletion failed [Error code %d]\n", errCode);

    /* Delete the outbound SA */
    if (Netfp_delSA (netfpClientHandle, outboundSAHandle, &errCode) < 0)
        printf ("Error: NETFP Outbound security policy deletion failed [Error code %d]\n", errCode);

    /* Delete the default gateway. */
    system ("ip route del default via 7000::1");

    return 0;
}


