/*
 *   @file  routeRecalculation.c
 *
 *   @brief
 *      Unit Test code for the route recalculation test
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

/* We are using eth0 in the test code and from the NETFP Master
 * configuration file on the EVM we derive the switch port number. */
#define TEST_SWITCH_PORT        1

/**********************************************************************
 ************************ Global Variables ****************************
 **********************************************************************/

/* Dummy Flow and channel created */
int32_t             dummyFlowId;
MsgCom_ChHandle     dummyChannel;

/* Global Handles:  */
Netfp_InboundFPHandle   ingressFPHandle;
Netfp_OutboundFPHandle  egressFPHandle[TEST_MAX_FAST_PATH];
Netfp_SockHandle        sockHandle[TEST_MAX_FAST_PATH];
Netfp_Reason            sockReason[TEST_MAX_FAST_PATH];

/* Global Flag which logs messages */
uint32_t                loggingFlag;

struct sockaddr_un          cmdAppSunAddr;
struct sockaddr_un          proxySunAddr;
int32_t                     cmdAppIpcSockFd;

/**********************************************************************
 ************************ Extern Declarations *************************
 **********************************************************************/

/* NETFP Client Handle: */
extern Netfp_ClientHandle   netfpClientHandle;
extern Pktlib_HeapHandle    netfpDataRxHeap;

/* Global MSGCOM instance handle. */
extern Msgcom_InstHandle   appMsgcomInstanceHandle;

/* v6 test functions */
extern int32_t Test_v6SocketStates(void);
extern int32_t Test_v6InterfaceEvents(void);
extern int32_t Test_v6NonSecureFPEvents(void);
extern int32_t Test_v6SecureFPEvents(void);

/**********************************************************************
 ************************** Unit Test Functions ***********************
 **********************************************************************/

/**
 *  @b Description
 *  @n
 *      Notify function which is registered with the socket to be invoked
 *      when there is a configuration change in the NETFP subsystem
 *
 *  @param[in]  reason
 *      Reason code because of which the socket was notified.
 *  @param[in]  sockHandle
 *      Socket Handle which was affected
 *
 *  @retval
 *      Not applicable
 */
void Test_notifySocketFunction(Netfp_Reason reason, Netfp_SockHandle sockHandle)
{
    uint32_t    index;

    /* Do we need to log messages? */
    if (loggingFlag == 1)
    {
        /* YES. Log messages */
        printf ("Debug: Notify function invoked for socket %p [Reason: '%s']\n", sockHandle, Netfp_getReasonString(reason));

        /* One message is enough. */
        loggingFlag = 0;
    }

    /* Record the reason for the failure: Cycle through and get a free block to record the reason. */
    for (index = 0; index < TEST_MAX_FAST_PATH; index++)
    {
        /* Do we have an empty slot? */
        if (sockReason[index] == 0)
        {
            /* YES. Record the reason. */
            sockReason[index] = reason;
            break;
        }
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      Notify function which is registered with the socket to be invoked
 *      when there is a configuration change in the NETFP subsystem
 *
 *  @param[in]  reason
 *      Reason code because of which the socket was notified.
 *  @param[in]  sockHandle
 *      Socket Handle which was affected
 *
 *  @retval
 *      Not applicable
 */
int32_t Test_executeNetfpProxyCommand (void)
{
    NetfpProxy_msg  reqMsg;

    /* Ensure that the messages are logged: */
    loggingFlag = 1;

    memset(&reqMsg, 0, sizeof(reqMsg));

    reqMsg.hdr.msgType  =   NETFP_PROXY_IPC_MSGTYPE_IP_ROUTE_FLUSH_REQ;
    reqMsg.hdr.transId  =   rand();

    /* Send the flush request to NetFP proxy daemon */
    if (sendto (cmdAppIpcSockFd, (void *)&reqMsg, sizeof (NetfpProxy_msg), 0,
                (const struct sockaddr *)&proxySunAddr, sizeof (struct sockaddr_un)) < 0)
    {
        printf("sending flush command Failed\n");
        return -1;
    }
    return 0;
}

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
 *          192.168.1.2            192.168.1.1
 *          Default Gw: 192.168.1.1
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
static int32_t Test_v4SocketStates(void)
{
    Netfp_InboundFPCfg  inboundFPConfig;
    Netfp_OutboundFPCfg outboundFPConfig;
    int32_t             errCode;
    int32_t             status;
    Netfp_SockAddr      sockAddress;
    Netfp_OptionTLV     optInfo;
    uint8_t             socketState;
    int32_t             index = 0;

    /* Debug Message: */
    printf ("***************************************************************************************\n");
    printf ("Debug: Testing socket states:\n");

    /*************************************************************************************************
     * TEST SETUP:
     *************************************************************************************************/
    system("ifconfig eth0 192.168.1.2");

    /* Create the Ingress fast paths: Initialize the fast path configuration */
    memset ((void *)&inboundFPConfig, 0, sizeof (Netfp_InboundFPCfg));

    /* Populate the fast path configuration: */
    inboundFPConfig.spId                       = NETFP_INVALID_SPID;
    inboundFPConfig.dstIP.ver                  = Netfp_IPVersion_IPV4;
    inboundFPConfig.dstIP.addr.ipv4.u.a8[0]    = 192;
    inboundFPConfig.dstIP.addr.ipv4.u.a8[1]    = 168;
    inboundFPConfig.dstIP.addr.ipv4.u.a8[2]    = 1;
    inboundFPConfig.dstIP.addr.ipv4.u.a8[3]    = 2;
    inboundFPConfig.srcIP.ver                  = Netfp_IPVersion_IPV4;
    inboundFPConfig.srcIP.addr.ipv4.u.a8[0]    = 192;
    inboundFPConfig.srcIP.addr.ipv4.u.a8[1]    = 168;
    inboundFPConfig.srcIP.addr.ipv4.u.a8[2]    = 1;
    inboundFPConfig.srcIP.addr.ipv4.u.a8[3]    = 5;
    snprintf (inboundFPConfig.name, NETFP_MAX_CHAR, "Ingress_FastPath-%d", index);

    /* Add the Ingress Fast Path. */
    ingressFPHandle = Netfp_createInboundFastPath(netfpClientHandle, &inboundFPConfig, &errCode);
    if (ingressFPHandle == NULL)
    {
        printf ("Error: Unable to add the NETFP Inbound Fast path [Error code %d]\n", errCode);
        return -1;
    }
    printf ("Debug: Adding %s [%03d.%03d.%03d.%03d] -> [%03d.%03d.%03d.%03d]\n",
            inboundFPConfig.name, inboundFPConfig.srcIP.addr.ipv4.u.a8[0], inboundFPConfig.srcIP.addr.ipv4.u.a8[1],
            inboundFPConfig.srcIP.addr.ipv4.u.a8[2], inboundFPConfig.srcIP.addr.ipv4.u.a8[3],
            inboundFPConfig.dstIP.addr.ipv4.u.a8[0], inboundFPConfig.dstIP.addr.ipv4.u.a8[1],
            inboundFPConfig.dstIP.addr.ipv4.u.a8[2], inboundFPConfig.dstIP.addr.ipv4.u.a8[3]);

    /* Resolve the route to each possible destination:
     * - These resolutions will end up getting resolved into the default gateway. */
    memset ((void *)&outboundFPConfig, 0, sizeof (Netfp_OutboundFPCfg));

    /* Populate the Fast Path configuration. */
    outboundFPConfig.spId                       = NETFP_INVALID_SPID;
    outboundFPConfig.dstIP.ver                  = Netfp_IPVersion_IPV4;
    outboundFPConfig.dstIP.addr.ipv4.u.a8[0]    = 192;
    outboundFPConfig.dstIP.addr.ipv4.u.a8[1]    = 168;
    outboundFPConfig.dstIP.addr.ipv4.u.a8[2]    = 1;
    outboundFPConfig.dstIP.addr.ipv4.u.a8[3]    = (5 + index);
    outboundFPConfig.srcIP.ver                  = Netfp_IPVersion_IPV4;
    outboundFPConfig.srcIP.addr.ipv4.u.a8[0]    = 192;
    outboundFPConfig.srcIP.addr.ipv4.u.a8[1]    = 168;
    outboundFPConfig.srcIP.addr.ipv4.u.a8[2]    = 1;
    outboundFPConfig.srcIP.addr.ipv4.u.a8[3]    = 2;
    snprintf (outboundFPConfig.name, NETFP_MAX_CHAR, "Egress_FastPath-%d", index);

    /* Add the Egress Fast Path. */
    egressFPHandle[index] = Netfp_createOutboundFastPath (netfpClientHandle, &outboundFPConfig, &errCode);
    if (egressFPHandle[index] == NULL)
    {
        printf ("Error: Unable to add the NETFP Egress Fast path [Error code %d]\n", errCode);
        return -1;
    }
    printf ("Debug: Adding %s [%03d.%03d.%03d.%03d] -> [%03d.%03d.%03d.%03d]\n",
            outboundFPConfig.name, outboundFPConfig.srcIP.addr.ipv4.u.a8[0], outboundFPConfig.srcIP.addr.ipv4.u.a8[1],
            outboundFPConfig.srcIP.addr.ipv4.u.a8[2], outboundFPConfig.srcIP.addr.ipv4.u.a8[3],
            outboundFPConfig.dstIP.addr.ipv4.u.a8[0], outboundFPConfig.dstIP.addr.ipv4.u.a8[1],
            outboundFPConfig.dstIP.addr.ipv4.u.a8[2], outboundFPConfig.dstIP.addr.ipv4.u.a8[3]);

    /* Relinuqish time slice: This will give enough time to allow the NETFP Proxy to resolve the next
     * hop MAC address */
    sleep(5);

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
    sockHandle[index] = Netfp_socket (netfpClientHandle, Netfp_SockFamily_AF_INET, &errCode);
    if (sockHandle[index] == NULL)
    {
        printf ("Error: NETFP Socket '%d' Creation Failed [Error Code %d]\n", index, errCode);
        return -1;
    }

    /* Populate the binding information: */
    memset ((void*)&sockAddress, 0, sizeof(Netfp_SockAddr));
    sockAddress.sin_family              = Netfp_SockFamily_AF_INET;
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
    sockAddress.sin_family                  = Netfp_SockFamily_AF_INET;
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

    system ("route add default gateway 192.168.1.1");

    /* Send the FLUSH command to Netfp Proxy */
    if (Test_executeNetfpProxyCommand() < 0)
    {
        printf ("Error: Failed to send Flush IP routes command to NetfpProxy\n");
        return -1;
    }
    sleep(5);

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
    system ("route del default gw 192.168.1.1");
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
 *          192.168.1.2            192.168.1.1
 *          Default Gw: 192.168.1.1
 *
 *  (a) Interface Status is bought down on the EVM. This should ensure that the socket
 *      notification function is invoked with the appropriate reason.
 *  (b) Interface Status is bought up on the EVM. This should ensure that the socket
 *      notification function is invoked with the appropriate reason.
 *  (c) Interface MTU is bought up on the EVM. This should ensure that the socket
 *      notification function is invoked with the appropriate reason.
 *  (d) Attach and modify the L3 Shaper configuration. This should ensure that the socket
 *      notification function is invoked with the appropriate reason.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Test_v4InterfaceEvents(void)
{
    Netfp_InboundFPCfg  inboundFPConfig;
    Netfp_OutboundFPCfg outboundFPConfig;
    int32_t             errCode;
    int32_t             status;
    Netfp_SockAddr      sockAddress;
    Netfp_OptionTLV     optInfo;
    Netfp_L3QoSCfg      l3QOSCfg;
    Netfp_IfHandle      ifHandle;
    uint8_t             socketState;
    uint32_t            socketMTU;
    int32_t             index = 0;

    /* Debug Message: */
    printf ("***************************************************************************************\n");
    printf ("Debug: Testing non secure fast paths with the interface events:\n");

    /*************************************************************************************************
     * TEST SETUP:
     *************************************************************************************************/
    system("ifconfig eth0 192.168.1.2");

    /* Create the Ingress fast paths: Initialize the fast path configuration */
    memset ((void *)&inboundFPConfig, 0, sizeof (Netfp_InboundFPCfg));

    /* Populate the fast path configuration: */
    inboundFPConfig.spId                       = NETFP_INVALID_SPID;
    inboundFPConfig.dstIP.ver                  = Netfp_IPVersion_IPV4;
    inboundFPConfig.dstIP.addr.ipv4.u.a8[0]    = 192;
    inboundFPConfig.dstIP.addr.ipv4.u.a8[1]    = 168;
    inboundFPConfig.dstIP.addr.ipv4.u.a8[2]    = 1;
    inboundFPConfig.dstIP.addr.ipv4.u.a8[3]    = 2;
    inboundFPConfig.srcIP.ver                  = Netfp_IPVersion_IPV4;
    inboundFPConfig.srcIP.addr.ipv4.u.a8[0]    = 192;
    inboundFPConfig.srcIP.addr.ipv4.u.a8[1]    = 168;
    inboundFPConfig.srcIP.addr.ipv4.u.a8[2]    = 1;
    inboundFPConfig.srcIP.addr.ipv4.u.a8[3]    = 1;
    snprintf (inboundFPConfig.name, NETFP_MAX_CHAR, "Ingress_FastPath-%d", index);

    /* Add the Ingress Fast Path. */
    ingressFPHandle = Netfp_createInboundFastPath(netfpClientHandle, &inboundFPConfig, &errCode);
    if (ingressFPHandle == NULL)
    {
        printf ("Error: Unable to add the NETFP Inbound Fast path [Error code %d]\n", errCode);
        return -1;
    }
    printf ("Debug: Adding %s [%03d.%03d.%03d.%03d] -> [%03d.%03d.%03d.%03d]\n",
            inboundFPConfig.name, inboundFPConfig.srcIP.addr.ipv4.u.a8[0], inboundFPConfig.srcIP.addr.ipv4.u.a8[1],
            inboundFPConfig.srcIP.addr.ipv4.u.a8[2], inboundFPConfig.srcIP.addr.ipv4.u.a8[3],
            inboundFPConfig.dstIP.addr.ipv4.u.a8[0], inboundFPConfig.dstIP.addr.ipv4.u.a8[1],
            inboundFPConfig.dstIP.addr.ipv4.u.a8[2], inboundFPConfig.dstIP.addr.ipv4.u.a8[3]);

    /* Resolve the route to each possible destination:
     * - These resolutions will end up getting resolved into the default gateway. */
    memset ((void *)&outboundFPConfig, 0, sizeof (Netfp_OutboundFPCfg));

    /* Populate the Fast Path configuration. */
    outboundFPConfig.spId                       = NETFP_INVALID_SPID;
    outboundFPConfig.dstIP.ver                  = Netfp_IPVersion_IPV4;
    outboundFPConfig.dstIP.addr.ipv4.u.a8[0]    = 192;
    outboundFPConfig.dstIP.addr.ipv4.u.a8[1]    = 168;
    outboundFPConfig.dstIP.addr.ipv4.u.a8[2]    = 1;
    outboundFPConfig.dstIP.addr.ipv4.u.a8[3]    = (1 + index);
    outboundFPConfig.srcIP.ver                  = Netfp_IPVersion_IPV4;
    outboundFPConfig.srcIP.addr.ipv4.u.a8[0]    = 192;
    outboundFPConfig.srcIP.addr.ipv4.u.a8[1]    = 168;
    outboundFPConfig.srcIP.addr.ipv4.u.a8[2]    = 1;
    outboundFPConfig.srcIP.addr.ipv4.u.a8[3]    = 2;
    snprintf (outboundFPConfig.name, NETFP_MAX_CHAR, "Egress_FastPath-%d", index);

    /* Add the Egress Fast Path. */
    egressFPHandle[index] = Netfp_createOutboundFastPath (netfpClientHandle, &outboundFPConfig, &errCode);
    if (egressFPHandle[index] == NULL)
    {
        printf ("Error: Unable to add the NETFP Egress Fast path [Error code %d]\n", errCode);
        return -1;
    }
    printf ("Debug: Adding %s [%03d.%03d.%03d.%03d] -> [%03d.%03d.%03d.%03d]\n",
            outboundFPConfig.name, outboundFPConfig.srcIP.addr.ipv4.u.a8[0], outboundFPConfig.srcIP.addr.ipv4.u.a8[1],
            outboundFPConfig.srcIP.addr.ipv4.u.a8[2], outboundFPConfig.srcIP.addr.ipv4.u.a8[3],
            outboundFPConfig.dstIP.addr.ipv4.u.a8[0], outboundFPConfig.dstIP.addr.ipv4.u.a8[1],
            outboundFPConfig.dstIP.addr.ipv4.u.a8[2], outboundFPConfig.dstIP.addr.ipv4.u.a8[3]);

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
    printf ("Debug: Egress IPv4 Fast Path Handle 0x%x is active\n", (uint32_t)egressFPHandle[index]);

    /* Create a socket */
    sockHandle[index] = Netfp_socket (netfpClientHandle, Netfp_SockFamily_AF_INET, &errCode);
    if (sockHandle[index] == NULL)
    {
        printf ("Error: NETFP Socket '%d' Creation Failed [Error Code %d]\n", index, errCode);
        return -1;
    }

    /* Populate the binding information: */
    memset ((void*)&sockAddress, 0, sizeof(Netfp_SockAddr));
    sockAddress.sin_family              = Netfp_SockFamily_AF_INET;
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
    sockAddress.sin_family                  = Netfp_SockFamily_AF_INET;
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
    sleep(5);

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

    system ("ifconfig eth0 mtu 1300");
    sleep(5);

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

    if (socketMTU != 1300)
    {
        printf ("Error: Socket handle 0x%x invalid socket MTU %d detected\n", (uint32_t)sockHandle[index], socketMTU);
        return -1;
    }
    printf ("Debug: Socket MTU %d validated successfully\n", socketMTU);

    /*************************************************************************************************
     * EVENT GENERATION: Setup the L3 Shaper
     *************************************************************************************************/
    printf ("Debug: Setting up the L3 Shaper on eth0\n");

    /* Setup the L3 shaper for the interface */
    ifHandle = Netfp_findInterface (netfpClientHandle, "eth0", NULL, &errCode);
    if (ifHandle == NULL)
    {
        printf ("Error: Interface eth0 NOT found; modify the test code with the correct interface [Error %d]\n", errCode);
        return -1;
    }

    /* Initialize the L3 QOS configuration */
    memset ((void *)&l3QOSCfg, 0, sizeof(Netfp_L3QoSCfg));

    /* Get the L3 Shaper on the interface */
    optInfo.type   = Netfp_Option_L3_QOS;
    optInfo.length = sizeof(Netfp_L3QoSCfg);
    optInfo.value  = (void*)&l3QOSCfg;
    if (Netfp_getIfOpt (netfpClientHandle, ifHandle, &optInfo, &errCode) < 0)
    {
        printf ("Error: Unable to get the L3 Shaper configuration [Error %d]\n", errCode);
        return -1;
    }

    /* Currently there should be no L3 QOS configuration on the interface */
    if (l3QOSCfg.isEnable == 1)
    {
        printf ("Error: L3 QOS Shaper is already configured on interface\n");
        return -1;
    }

    /* Initialize the L3 QOS configuration */
    memset ((void *)&l3QOSCfg, 0, sizeof(Netfp_L3QoSCfg));

    /* Setup the L3 shaper: */
    l3QOSCfg.isEnable = 1;
    l3QOSCfg.flowId   = 1;

    /* By default all DSCP are mapped to the best effort queue i.e. 8072. */
    {
        int32_t i;

        for (i = 0; i < 64; i++)
            l3QOSCfg.qid[i] = 8072;
    }

    /* Cycle through and reset the socket reasons: We are about to start the test */
    sockReason[index] = 0;

    /* We are interested in logging: */
    loggingFlag = 1;

    /* Setup the L3 QOS Shaper: */
    optInfo.type   = Netfp_Option_L3_QOS;
    optInfo.length = sizeof(Netfp_L3QoSCfg);
    optInfo.value  = (void*)&l3QOSCfg;

    /* We can now setup the L3 QOS shaper: */
    if (Netfp_setIfOpt (netfpClientHandle, ifHandle, &optInfo, &errCode) < 0)
    {
        printf ("Error: Unable to configure the L3 shaper [Error %d]\n", errCode);
        return -1;
    }
    sleep(5);

    /*************************************************************************************************
     * VALIDATIONS: We should have detected a valid reason on each socket.
     *************************************************************************************************/
    if (sockReason[index] == 0)
    {
        /* Error: No reason detected for the socket. */
        printf ("Error: Socket handle 0x%x index %d was not notified\n", (uint32_t)sockHandle[index], index);
        return -1;
    }

    /* Each socket should have received an interface update event which indicates that the L3 QOS configuration was changed */
    if (sockReason[index] != Netfp_Reason_L3_QOS)
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

    /* Initialize the L3 QOS configuration */
    memset ((void *)&l3QOSCfg, 0, sizeof(Netfp_L3QoSCfg));

    /* Get the L3 Shaper on the interface */
    optInfo.type   = Netfp_Option_L3_QOS;
    optInfo.length = sizeof(Netfp_L3QoSCfg);
    optInfo.value  = (void*)&l3QOSCfg;
    if (Netfp_getIfOpt (netfpClientHandle, ifHandle, &optInfo, &errCode) < 0)
    {
        printf ("Error: Unable to get the L3 Shaper configuration [Error %d]\n", errCode);
        return -1;
    }

    /* L3 QOS should be configured on the interface */
    if (l3QOSCfg.isEnable == 0)
    {
        printf ("Error: L3 QOS Shaper is NOT configured on interface\n");
        return -1;
    }

    /*************************************************************************************************
     * EVENT GENERATION: Disabling the L3 Shaper
     *************************************************************************************************/
    printf ("Debug: Disabling the L3 Shaper\n");

    /* Disable the L3 Shaper: */
    l3QOSCfg.isEnable = 0;

    /* Cycle through and reset the socket reasons: We are about to start the test */
    sockReason[index] = 0;

    /* We are interested in logging: */
    loggingFlag = 1;

    /* Setup the L3 QOS Shaper: */
    optInfo.type   = Netfp_Option_L3_QOS;
    optInfo.length = sizeof(Netfp_L3QoSCfg);
    optInfo.value  = (void*)&l3QOSCfg;

    /* We can now setup the L3 QOS shaper: */
    if (Netfp_setIfOpt (netfpClientHandle, ifHandle, &optInfo, &errCode) < 0)
    {
        printf ("Error: Unable to configure the L3 shaper [Error %d]\n", errCode);
        return -1;
    }
    sleep(5);

    /*************************************************************************************************
     * VALIDATIONS: We should have detected a valid reason on each socket.
     *************************************************************************************************/
    if (sockReason[index] == 0)
    {
        /* Error: No reason detected for the socket. */
        printf ("Error: Socket handle 0x%x index %d was not notified\n", (uint32_t)sockHandle[index], index);
        return -1;
    }

    /* Each socket should have received an interface update event which indicates that the L3 QOS configuration was changed */
    if (sockReason[index] != Netfp_Reason_L3_QOS)
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

    /* Get the L3 Shaper on the interface */
    optInfo.type   = Netfp_Option_L3_QOS;
    optInfo.length = sizeof(Netfp_L3QoSCfg);
    optInfo.value  = (void*)&l3QOSCfg;
    if (Netfp_getIfOpt (netfpClientHandle, ifHandle, &optInfo, &errCode) < 0)
    {
        printf ("Error: Unable to get the L3 Shaper configuration [Error %d]\n", errCode);
        return -1;
    }

    /* There should be no L3 QOS configuration on the interface */
    if (l3QOSCfg.isEnable == 1)
    {
        printf ("Error: L3 QOS Shaper is still configured on interface\n");
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
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to test the route recalculation for non-secure fast paths
 *
 *      The following is the test setup which is constructed here:
 *
 *      |----------|                  |--------|---------- 11.11.11.x
 *      |   EVM    |------------------| GW     |
 *      |----------|                  |--------|
 *          192.168.1.2            192.168.1.1
 *          Default Gw: 192.168.1.1
 *
 *  (a) Default Gateway is deleted. This implies that the sockets connected to the
 *      11.11.11.x network will no longer be accessible and the NOTIFY function should
 *      indicate that the socket is UNREACHABLE.
 *  (b) Default Gateway is added back. This implies that the sockets connected to the
 *      11.11.11.x network will be accessible and the NOTIFY function should indicate
 *      that the socket is REACHABLE.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Test_v4NonSecureFPEvents(void)
{
    int32_t             index;
    Netfp_InboundFPCfg  inboundFPConfig;
    Netfp_OutboundFPCfg outboundFPConfig;
    int32_t             errCode;
    int32_t             status;
    Netfp_SockAddr      sockAddress;
    Netfp_OptionTLV     optInfo;
    uint8_t             socketState;
    uint16_t            vlanId;
    uint32_t            switchPort;

    /* Debug Message: */
    printf ("***************************************************************************************\n");
    printf ("Debug: Testing non secure fast paths with the deletion and addition of default gateway.\n");

    /*************************************************************************************************
     * TEST SETUP:
     *************************************************************************************************/
    system("ifconfig eth0 192.168.1.2");
    system("route add default gw 192.168.1.1");

    /* Initialize the fast path configuration */
    memset ((void *)&inboundFPConfig, 0, sizeof (Netfp_InboundFPCfg));

    /* Populate the fast path configuration: */
    inboundFPConfig.spId                       = NETFP_INVALID_SPID;
    inboundFPConfig.dstIP.ver                  = Netfp_IPVersion_IPV4;
    inboundFPConfig.dstIP.addr.ipv4.u.a8[0]    = 192;
    inboundFPConfig.dstIP.addr.ipv4.u.a8[1]    = 168;
    inboundFPConfig.dstIP.addr.ipv4.u.a8[2]    = 1;
    inboundFPConfig.dstIP.addr.ipv4.u.a8[3]    = 2;
    inboundFPConfig.srcIP.ver                  = Netfp_IPVersion_IPV4;
    inboundFPConfig.srcIP.addr.ipv4.u.a8[0]    = 11;
    inboundFPConfig.srcIP.addr.ipv4.u.a8[1]    = 11;
    inboundFPConfig.srcIP.addr.ipv4.u.a8[2]    = 11;
    inboundFPConfig.srcIP.addr.ipv4.u.a8[3]    = 1;
    snprintf (inboundFPConfig.name, NETFP_MAX_CHAR, "Ingress_FastPath");

    /* Add the Ingress Fast Path. */
    ingressFPHandle = Netfp_createInboundFastPath(netfpClientHandle, &inboundFPConfig, &errCode);
    if (ingressFPHandle == NULL)
    {
        printf ("Error: Unable to add the NETFP Inbound Fast path [Error code %d]\n", errCode);
        return -1;
    }
    printf ("Debug: Adding %s [%03d.%03d.%03d.%03d] -> [%03d.%03d.%03d.%03d]\n",
            inboundFPConfig.name, inboundFPConfig.srcIP.addr.ipv4.u.a8[0], inboundFPConfig.srcIP.addr.ipv4.u.a8[1],
            inboundFPConfig.srcIP.addr.ipv4.u.a8[2], inboundFPConfig.srcIP.addr.ipv4.u.a8[3],
            inboundFPConfig.dstIP.addr.ipv4.u.a8[0], inboundFPConfig.dstIP.addr.ipv4.u.a8[1],
            inboundFPConfig.dstIP.addr.ipv4.u.a8[2], inboundFPConfig.dstIP.addr.ipv4.u.a8[3]);

    /* Resolve the route to each possible destination:
     * - These resolutions will end up getting resolved into the default gateway. */
    for (index = 0; index < TEST_MAX_FAST_PATH; index++)
    {
        /* Initialize the fast path configuration. */
        memset ((void *)&outboundFPConfig, 0, sizeof (Netfp_OutboundFPCfg));

        /* Populate the Fast Path configuration. */
        outboundFPConfig.spId                       = NETFP_INVALID_SPID;
        outboundFPConfig.dstIP.ver                  = Netfp_IPVersion_IPV4;
        outboundFPConfig.dstIP.addr.ipv4.u.a8[0]    = 11;
        outboundFPConfig.dstIP.addr.ipv4.u.a8[1]    = 11;
        outboundFPConfig.dstIP.addr.ipv4.u.a8[2]    = 11;
        outboundFPConfig.dstIP.addr.ipv4.u.a8[3]    = (1 + index);
        outboundFPConfig.srcIP.ver                  = Netfp_IPVersion_IPV4;
        outboundFPConfig.srcIP.addr.ipv4.u.a8[0]    = 192;
        outboundFPConfig.srcIP.addr.ipv4.u.a8[1]    = 168;
        outboundFPConfig.srcIP.addr.ipv4.u.a8[2]    = 1;
        outboundFPConfig.srcIP.addr.ipv4.u.a8[3]    = 2;
        snprintf (outboundFPConfig.name, NETFP_MAX_CHAR, "Egress_FastPath-%d", index);

        /* Add the Egress Fast Path. */
        egressFPHandle[index] = Netfp_createOutboundFastPath (netfpClientHandle, &outboundFPConfig, &errCode);
        if (egressFPHandle[index] == NULL)
        {
            printf ("Error: Unable to add the NETFP Egress Fast path [Error code %d]\n", errCode);
            return -1;
        }
        printf ("Debug: Adding %s [%03d.%03d.%03d.%03d] -> [%03d.%03d.%03d.%03d]\n",
                outboundFPConfig.name, outboundFPConfig.srcIP.addr.ipv4.u.a8[0], outboundFPConfig.srcIP.addr.ipv4.u.a8[1],
                outboundFPConfig.srcIP.addr.ipv4.u.a8[2], outboundFPConfig.srcIP.addr.ipv4.u.a8[3],
                outboundFPConfig.dstIP.addr.ipv4.u.a8[0], outboundFPConfig.dstIP.addr.ipv4.u.a8[1],
                outboundFPConfig.dstIP.addr.ipv4.u.a8[2], outboundFPConfig.dstIP.addr.ipv4.u.a8[3]);

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
        printf ("Debug: Egress IPv4 Fast Path Handle %s is active\n", outboundFPConfig.name);
    }

    /* Create all the sockets on each of the fast paths: */
    for (index = 0; index < TEST_MAX_FAST_PATH; index++)
    {
        /* Create a socket */
        sockHandle[index] = Netfp_socket (netfpClientHandle, Netfp_SockFamily_AF_INET, &errCode);
        if (sockHandle[index] == NULL)
        {
            printf ("Error: NETFP Socket '%d' Creation Failed [Error Code %d]\n", index, errCode);
            return -1;
        }

        /* Populate the binding information: */
        memset ((void*)&sockAddress, 0, sizeof(Netfp_SockAddr));
        sockAddress.sin_family              = Netfp_SockFamily_AF_INET;
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
        sockAddress.sin_family                  = Netfp_SockFamily_AF_INET;
        sockAddress.sin_port                    = 4000 + index;
        sockAddress.op.connect.outboundFPHandle = egressFPHandle[index];

        /* Connect the socket */
        if (Netfp_connect(sockHandle[index], &sockAddress, &errCode) < 0)
        {
            printf ("Error: NETFP Connect Failed for socket '%d' to port '%d' [Error Code %d]\n",
                    index, sockAddress.sin_port, errCode);
            return -1;
        }

        /* Initialize the VLAN identifier */
        vlanId = 0xFFFF;

        /* Get the VLAN Id which is being used by the socket: */
        optInfo.type   = Netfp_Option_VLAN_ID;
        optInfo.length = sizeof(uint16_t);
        optInfo.value  = (void*)&vlanId;
        if (Netfp_getSockOpt (sockHandle[index], &optInfo, &errCode) < 0)
        {
            printf ("Error: Getting socket option for VLAN identifier failed [Error code %d]\n", errCode);
            return -1;
        }

        /* The test is executed on a non-VLAN enabled interface. This implies that we should always get a
         * VLAN identifier of 0 i.e. no VLAN */
        if (vlanId != 0)
        {
            printf ("Error: Incorrect VLAN Identifier %d\n", vlanId);
            return -1;
        }

        /* Initialize the switch port: */
        switchPort = 0xFFFF;

        /* Get the Switch Port which is being used by the socket: */
        optInfo.type   = Netfp_Option_SWITCH_PORT;
        optInfo.length = sizeof(uint32_t);
        optInfo.value  = (void*)&switchPort;
        if (Netfp_getSockOpt (sockHandle[index], &optInfo, &errCode) < 0)
        {
            printf ("Error: Getting socket option for VLAN identifier failed [Error code %d]\n", errCode);
            return -1;
        }
        if (switchPort != TEST_SWITCH_PORT)
        {
            printf ("Error: Incorrect Switch Port Detected Got %d Expected %d\n", switchPort, TEST_SWITCH_PORT);
            return -1;
        }
        printf ("Debug: Socket Handle 0x%x has been bound and connected to %d VLAN Id: %d Switch Port :%d\n",
                (uint32_t)sockHandle[index], sockAddress.sin_port, vlanId, switchPort);
    }

    /*************************************************************************************************
     * EVENT GENERATION: Delete the default gateway
     *************************************************************************************************/
    printf ("Debug: Initiating the delete default gateway\n");

    /* Cycle through and reset the socket reasons: We are about to start the test */
    for (index = 0; index < TEST_MAX_FAST_PATH; index++)
        sockReason[index] = 0;

    system ("route del default gw 192.168.1.1");

    /* Send the FLUSH command to Netfp Proxy */
    if (Test_executeNetfpProxyCommand() < 0)
    {
        printf ("Error: Failed to send Flush IP routes command to NetfpProxy\n");
        return -1;
    }
    sleep(1);

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

        /* Get the Switch Port which is being used by the socket: Since the socket is NOT active we
         * should get the INVALID SWITCH Port number. */
        optInfo.type   = Netfp_Option_SWITCH_PORT;
        optInfo.length = sizeof(uint32_t);
        optInfo.value  = (void*)&switchPort;
        if (Netfp_getSockOpt (sockHandle[index], &optInfo, &errCode) < 0)
        {
            printf ("Error: Getting socket option for VLAN identifier failed [Error code %d]\n", errCode);
            return -1;
        }
        if (switchPort != NETFP_INVALID_SWITCH_PORT)
        {
            printf ("Error: Incorrect Switch Port Detected Got %d Expected %d\n", switchPort, NETFP_INVALID_SWITCH_PORT);
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

    system ("route add default gw 192.168.1.1");

    /* Send the FLUSH command to Netfp Proxy */
    if (Test_executeNetfpProxyCommand() < 0)
    {
        printf ("Error: Failed to send Flush IP routes command to NetfpProxy\n");
        return -1;
    }
    sleep(1);

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

        /* Get the Switch Port which is being used by the socket: Since the socket is active we
         * should get the valid port number. */
        optInfo.type   = Netfp_Option_SWITCH_PORT;
        optInfo.length = sizeof(uint32_t);
        optInfo.value  = (void*)&switchPort;
        if (Netfp_getSockOpt (sockHandle[index], &optInfo, &errCode) < 0)
        {
            printf ("Error: Getting socket option for VLAN identifier failed [Error code %d]\n", errCode);
            return -1;
        }
        if (switchPort != TEST_SWITCH_PORT)
        {
            printf ("Error: Incorrect Switch Port Detected Got %d Expected %d\n", switchPort, TEST_SWITCH_PORT);
            return -1;
        }
    }

    /*************************************************************************************************
     * CLEANUP:
     *************************************************************************************************/
    for (index = 0; index < TEST_MAX_FAST_PATH; index++)
    {
        /* Delete the socket: */
        if (Netfp_closeSocket (sockHandle[index], &errCode) < 0)
            printf ("Error: NETFP Close socket failed [Error code %d]\n", errCode);

        /* Delete the egress fast path: */
        if (Netfp_deleteOutboundFastPath (netfpClientHandle, egressFPHandle[index], &errCode) < 0)
            printf ("Error: NETFP Egress fast path deletion failed [Error code %d]\n", errCode);
    }

    /* Delete the ingress fast path: */
    if (Netfp_deleteInboundFastPath (netfpClientHandle, ingressFPHandle, &errCode) < 0)
        printf ("Error: NETFP Ingress fast path deletion failed [Error code %d]\n", errCode);

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
 *      |----------|                  |--------|            11.11.11.1
 *          192.168.1.2            192.168.1.1
 *          Default Gw: 192.168.1.1
 *
 *  There is a tunnel between the EVM and SecGW. Inner IP address of the
 *  EVM (100.100.100.2) is communicating with the Serving-Gateway @ 11.11.11.1. The
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
static int32_t Test_v4SecureFPEventsDefaultGateway(void)
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
    system("ifconfig eth0:2 100.100.100.2 netmask 255.255.255.0");
    system("route add default gw 192.168.1.1");

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
    saConfig.srcIP.ver = Netfp_IPVersion_IPV4;
    saConfig.srcIP.addr.ipv4.u.a8[0] = 192;
    saConfig.srcIP.addr.ipv4.u.a8[1] = 168;
    saConfig.srcIP.addr.ipv4.u.a8[2] = 1;
    saConfig.srcIP.addr.ipv4.u.a8[3] = 1;

    saConfig.dstIP.ver = Netfp_IPVersion_IPV4;
    saConfig.dstIP.addr.ipv4.u.a8[0] = 192;
    saConfig.dstIP.addr.ipv4.u.a8[1] = 168;
    saConfig.dstIP.addr.ipv4.u.a8[2] = 1;
    saConfig.dstIP.addr.ipv4.u.a8[3] = 2;

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
    saConfig.srcIP.ver = Netfp_IPVersion_IPV4;
    saConfig.srcIP.addr.ipv4.u.a8[0] = 192;
    saConfig.srcIP.addr.ipv4.u.a8[1] = 168;
    saConfig.srcIP.addr.ipv4.u.a8[2] = 1;
    saConfig.srcIP.addr.ipv4.u.a8[3] = 2;

    saConfig.dstIP.ver = Netfp_IPVersion_IPV4;
    saConfig.dstIP.addr.ipv4.u.a8[0] = 192;
    saConfig.dstIP.addr.ipv4.u.a8[1] = 168;
    saConfig.dstIP.addr.ipv4.u.a8[2] = 1;
    saConfig.dstIP.addr.ipv4.u.a8[3] = 1;

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
    spConfig.srcIP.ver               = Netfp_IPVersion_IPV4;
    spConfig.srcIP.addr.ipv4.u.a8[0] = 11;
    spConfig.srcIP.addr.ipv4.u.a8[1] = 11;
    spConfig.srcIP.addr.ipv4.u.a8[2] = 11;
    spConfig.srcIP.addr.ipv4.u.a8[3] = 1;
    spConfig.dstIP.ver               = Netfp_IPVersion_IPV4;
    spConfig.dstIP.addr.ipv4.u.a8[0] = 100;
    spConfig.dstIP.addr.ipv4.u.a8[1] = 100;
    spConfig.dstIP.addr.ipv4.u.a8[2] = 100;
    spConfig.dstIP.addr.ipv4.u.a8[3] = 2;
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
    spConfig.srcIP.ver               = Netfp_IPVersion_IPV4;
    spConfig.srcIP.addr.ipv4.u.a8[0] = 100;
    spConfig.srcIP.addr.ipv4.u.a8[1] = 100;
    spConfig.srcIP.addr.ipv4.u.a8[2] = 100;
    spConfig.srcIP.addr.ipv4.u.a8[3] = 2;
    spConfig.dstIP.ver               = Netfp_IPVersion_IPV4;
    spConfig.dstIP.addr.ipv4.u.a8[0] = 11;
    spConfig.dstIP.addr.ipv4.u.a8[1] = 11;
    spConfig.dstIP.addr.ipv4.u.a8[2] = 11;
    spConfig.dstIP.addr.ipv4.u.a8[3] = 1;
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
    inboundFPConfig.dstIP.ver                  = Netfp_IPVersion_IPV4;
    inboundFPConfig.dstIP.addr.ipv4.u.a8[0]    = 100;
    inboundFPConfig.dstIP.addr.ipv4.u.a8[1]    = 100;
    inboundFPConfig.dstIP.addr.ipv4.u.a8[2]    = 100;
    inboundFPConfig.dstIP.addr.ipv4.u.a8[3]    = 2;
    inboundFPConfig.srcIP.ver                  = Netfp_IPVersion_IPV4;
    inboundFPConfig.srcIP.addr.ipv4.u.a8[0]    = 11;
    inboundFPConfig.srcIP.addr.ipv4.u.a8[1]    = 11;
    inboundFPConfig.srcIP.addr.ipv4.u.a8[2]    = 11;
    inboundFPConfig.srcIP.addr.ipv4.u.a8[3]    = (1 + index);
    snprintf (inboundFPConfig.name, NETFP_MAX_CHAR, "Ingress_FastPath-%d", index);

    /* Add the Ingress Fast Path. */
    ingressFPHandle = Netfp_createInboundFastPath(netfpClientHandle, &inboundFPConfig, &errCode);
    if (ingressFPHandle == NULL)
    {
        printf ("Error: Unable to add the NETFP Inbound Fast path [Error code %d]\n", errCode);
        return -1;
    }
    printf ("Debug: Adding %s [%03d.%03d.%03d.%03d] -> [%03d.%03d.%03d.%03d]\n",
            inboundFPConfig.name, inboundFPConfig.srcIP.addr.ipv4.u.a8[0], inboundFPConfig.srcIP.addr.ipv4.u.a8[1],
            inboundFPConfig.srcIP.addr.ipv4.u.a8[2], inboundFPConfig.srcIP.addr.ipv4.u.a8[3],
            inboundFPConfig.dstIP.addr.ipv4.u.a8[0], inboundFPConfig.dstIP.addr.ipv4.u.a8[1],
            inboundFPConfig.dstIP.addr.ipv4.u.a8[2], inboundFPConfig.dstIP.addr.ipv4.u.a8[3]);

    /* Resolve the route to each possible destination:
     * - These resolutions will end up getting resolved into the default gateway. */
    memset ((void *)&outboundFPConfig, 0, sizeof (Netfp_OutboundFPCfg));

    /* Populate the Fast Path configuration. */
    outboundFPConfig.spId                       = 200;
    outboundFPConfig.dstIP.ver                  = Netfp_IPVersion_IPV4;
    outboundFPConfig.dstIP.addr.ipv4.u.a8[0]    = 11;
    outboundFPConfig.dstIP.addr.ipv4.u.a8[1]    = 11;
    outboundFPConfig.dstIP.addr.ipv4.u.a8[2]    = 11;
    outboundFPConfig.dstIP.addr.ipv4.u.a8[3]    = (1 + index);
    outboundFPConfig.srcIP.ver                  = Netfp_IPVersion_IPV4;
    outboundFPConfig.srcIP.addr.ipv4.u.a8[0]    = 100;
    outboundFPConfig.srcIP.addr.ipv4.u.a8[1]    = 100;
    outboundFPConfig.srcIP.addr.ipv4.u.a8[2]    = 100;
    outboundFPConfig.srcIP.addr.ipv4.u.a8[3]    = 2;
    snprintf (outboundFPConfig.name, NETFP_MAX_CHAR, "Egress_FastPath-%d", index);

    /* Add the Egress Fast Path. */
    egressFPHandle[index] = Netfp_createOutboundFastPath (netfpClientHandle, &outboundFPConfig, &errCode);
    if (egressFPHandle[index] == NULL)
    {
        printf ("Error: Unable to add the NETFP Egress Fast path [Error code %d]\n", errCode);
        return -1;
    }
    printf ("Debug: Adding %s [%03d.%03d.%03d.%03d] -> [%03d.%03d.%03d.%03d]\n",
            outboundFPConfig.name, outboundFPConfig.srcIP.addr.ipv4.u.a8[0], outboundFPConfig.srcIP.addr.ipv4.u.a8[1],
            outboundFPConfig.srcIP.addr.ipv4.u.a8[2], outboundFPConfig.srcIP.addr.ipv4.u.a8[3],
            outboundFPConfig.dstIP.addr.ipv4.u.a8[0], outboundFPConfig.dstIP.addr.ipv4.u.a8[1],
            outboundFPConfig.dstIP.addr.ipv4.u.a8[2], outboundFPConfig.dstIP.addr.ipv4.u.a8[3]);

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
    printf ("Debug: Egress IPv4 Fast Path Handle 0x%x is active\n", (uint32_t)egressFPHandle[index]);

    /* Create a socket */
    sockHandle[index] = Netfp_socket (netfpClientHandle, Netfp_SockFamily_AF_INET, &errCode);
    if (sockHandle[index] == NULL)
    {
        printf ("Error: NETFP Socket '%d' Creation Failed [Error Code %d]\n", index, errCode);
        return -1;
    }

    /* Populate the binding information: */
    memset ((void*)&sockAddress, 0, sizeof(Netfp_SockAddr));
    sockAddress.sin_family              = Netfp_SockFamily_AF_INET;
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
    sockAddress.sin_family                  = Netfp_SockFamily_AF_INET;
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

    system ("route del default gw 192.168.1.1");

    /* Send the FLUSH command to Netfp Proxy */
    if (Test_executeNetfpProxyCommand() < 0)
    {
        printf ("Error: Failed to send Flush IP routes command to NetfpProxy\n");
        return -1;
    }
    sleep(5);

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

    system ("route add default gw 192.168.1.1");

    /* Send the FLUSH command to Netfp Proxy */
    if (Test_executeNetfpProxyCommand() < 0)
    {
        printf ("Error: Failed to send Flush IP routes command to NetfpProxy\n");
        return -1;
    }
    sleep(5);

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
    system ("route del default gateway 192.168.1.1");

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
 *  192.168.1.2                                  192.168.1.1
 *      |----------|                  |--------|
 *      |   EVM    |------------------| SecGW  |
 *      |----------|                  |--------|
 *          192.168.1.2            192.168.1.1
 *          Default Gw: 192.168.1.1
 *
 *  There is a tunnel between the EVM and SecGW. The test communicates with the SeGW
 *  itself over the tunnel i.e. the Fast Path & SA have the same configuration.
 *
 *  The test case will perform the following:
 *  (a) Socket & Fast Path should be active.
 *  (b) After the rekey SA the socket and the fast path should remain ACTIVE since
 *      the SA status will be inherited.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Test_v4SecureFPEvents(void)
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
    Netfp_SAHandle      newSAHandle;
    Netfp_SPCfg         spConfig;
    uint8_t encKey[32] =
        {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11,
         0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11};
    uint8_t authKey[32] =
        {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11,
         0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11};

    /* Debug Message: */
    printf ("***************************************************************************************\n");
    printf ("Debug: Testing secure fast paths\n");

    /*************************************************************************************************
     * TEST SETUP:
     *************************************************************************************************/
    system("ifconfig eth0 192.168.1.2");
    system("ifconfig eth0:2 100.100.100.2 netmask 255.255.255.0");
    system("route add default gw 192.168.1.1");

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
    saConfig.srcIP.ver = Netfp_IPVersion_IPV4;
    saConfig.srcIP.addr.ipv4.u.a8[0] = 192;
    saConfig.srcIP.addr.ipv4.u.a8[1] = 168;
    saConfig.srcIP.addr.ipv4.u.a8[2] = 1;
    saConfig.srcIP.addr.ipv4.u.a8[3] = 1;

    saConfig.dstIP.ver = Netfp_IPVersion_IPV4;
    saConfig.dstIP.addr.ipv4.u.a8[0] = 192;
    saConfig.dstIP.addr.ipv4.u.a8[1] = 168;
    saConfig.dstIP.addr.ipv4.u.a8[2] = 1;
    saConfig.dstIP.addr.ipv4.u.a8[3] = 2;

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
    saConfig.srcIP.ver = Netfp_IPVersion_IPV4;
    saConfig.srcIP.addr.ipv4.u.a8[0] = 192;
    saConfig.srcIP.addr.ipv4.u.a8[1] = 168;
    saConfig.srcIP.addr.ipv4.u.a8[2] = 1;
    saConfig.srcIP.addr.ipv4.u.a8[3] = 2;

    saConfig.dstIP.ver = Netfp_IPVersion_IPV4;
    saConfig.dstIP.addr.ipv4.u.a8[0] = 192;
    saConfig.dstIP.addr.ipv4.u.a8[1] = 168;
    saConfig.dstIP.addr.ipv4.u.a8[2] = 1;
    saConfig.dstIP.addr.ipv4.u.a8[3] = 1;

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
    spConfig.srcIP.ver               = Netfp_IPVersion_IPV4;
    spConfig.srcIP.addr.ipv4.u.a8[0] = 11;
    spConfig.srcIP.addr.ipv4.u.a8[1] = 11;
    spConfig.srcIP.addr.ipv4.u.a8[2] = 11;
    spConfig.srcIP.addr.ipv4.u.a8[3] = 1;
    spConfig.dstIP.ver               = Netfp_IPVersion_IPV4;
    spConfig.dstIP.addr.ipv4.u.a8[0] = 100;
    spConfig.dstIP.addr.ipv4.u.a8[1] = 100;
    spConfig.dstIP.addr.ipv4.u.a8[2] = 100;
    spConfig.dstIP.addr.ipv4.u.a8[3] = 2;
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
    spConfig.srcIP.ver               = Netfp_IPVersion_IPV4;
    spConfig.srcIP.addr.ipv4.u.a8[0] = 100;
    spConfig.srcIP.addr.ipv4.u.a8[1] = 100;
    spConfig.srcIP.addr.ipv4.u.a8[2] = 100;
    spConfig.srcIP.addr.ipv4.u.a8[3] = 2;
    spConfig.dstIP.ver               = Netfp_IPVersion_IPV4;
    spConfig.dstIP.addr.ipv4.u.a8[0] = 11;
    spConfig.dstIP.addr.ipv4.u.a8[1] = 11;
    spConfig.dstIP.addr.ipv4.u.a8[2] = 11;
    spConfig.dstIP.addr.ipv4.u.a8[3] = 1;
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
    inboundFPConfig.dstIP.ver                  = Netfp_IPVersion_IPV4;
    inboundFPConfig.dstIP.addr.ipv4.u.a8[0]    = 100;
    inboundFPConfig.dstIP.addr.ipv4.u.a8[1]    = 100;
    inboundFPConfig.dstIP.addr.ipv4.u.a8[2]    = 100;
    inboundFPConfig.dstIP.addr.ipv4.u.a8[3]    = 2;
    inboundFPConfig.srcIP.ver                  = Netfp_IPVersion_IPV4;
    inboundFPConfig.srcIP.addr.ipv4.u.a8[0]    = 11;
    inboundFPConfig.srcIP.addr.ipv4.u.a8[1]    = 11;
    inboundFPConfig.srcIP.addr.ipv4.u.a8[2]    = 11;
    inboundFPConfig.srcIP.addr.ipv4.u.a8[3]    = (1 + index);
    snprintf (inboundFPConfig.name, NETFP_MAX_CHAR, "Ingress_FastPath-%d", index);

    /* Add the Ingress Fast Path. */
    ingressFPHandle = Netfp_createInboundFastPath(netfpClientHandle, &inboundFPConfig, &errCode);
    if (ingressFPHandle == NULL)
    {
        printf ("Error: Unable to add the NETFP Inbound Fast path [Error code %d]\n", errCode);
        return -1;
    }
    printf ("Debug: Adding %s [%03d.%03d.%03d.%03d] -> [%03d.%03d.%03d.%03d]\n",
            inboundFPConfig.name, inboundFPConfig.srcIP.addr.ipv4.u.a8[0], inboundFPConfig.srcIP.addr.ipv4.u.a8[1],
            inboundFPConfig.srcIP.addr.ipv4.u.a8[2], inboundFPConfig.srcIP.addr.ipv4.u.a8[3],
            inboundFPConfig.dstIP.addr.ipv4.u.a8[0], inboundFPConfig.dstIP.addr.ipv4.u.a8[1],
            inboundFPConfig.dstIP.addr.ipv4.u.a8[2], inboundFPConfig.dstIP.addr.ipv4.u.a8[3]);

    /* Resolve the route to each possible destination:
     * - These resolutions will end up getting resolved into the default gateway. */
    memset ((void *)&outboundFPConfig, 0, sizeof (Netfp_OutboundFPCfg));

    /* Populate the Fast Path configuration. */
    outboundFPConfig.spId                       = 200;
    outboundFPConfig.dstIP.ver                  = Netfp_IPVersion_IPV4;
    outboundFPConfig.dstIP.addr.ipv4.u.a8[0]    = 11;
    outboundFPConfig.dstIP.addr.ipv4.u.a8[1]    = 11;
    outboundFPConfig.dstIP.addr.ipv4.u.a8[2]    = 11;
    outboundFPConfig.dstIP.addr.ipv4.u.a8[3]    = (1 + index);
    outboundFPConfig.srcIP.ver                  = Netfp_IPVersion_IPV4;
    outboundFPConfig.srcIP.addr.ipv4.u.a8[0]    = 100;
    outboundFPConfig.srcIP.addr.ipv4.u.a8[1]    = 100;
    outboundFPConfig.srcIP.addr.ipv4.u.a8[2]    = 100;
    outboundFPConfig.srcIP.addr.ipv4.u.a8[3]    = 2;
    snprintf (outboundFPConfig.name, NETFP_MAX_CHAR, "Egress_FastPath-%d", index);

    /* Add the Egress Fast Path. */
    egressFPHandle[index] = Netfp_createOutboundFastPath (netfpClientHandle, &outboundFPConfig, &errCode);
    if (egressFPHandle[index] == NULL)
    {
        printf ("Error: Unable to add the NETFP Egress Fast path [Error code %d]\n", errCode);
        return -1;
    }
    printf ("Debug: Adding %s [%03d.%03d.%03d.%03d] -> [%03d.%03d.%03d.%03d]\n",
            outboundFPConfig.name, outboundFPConfig.srcIP.addr.ipv4.u.a8[0], outboundFPConfig.srcIP.addr.ipv4.u.a8[1],
            outboundFPConfig.srcIP.addr.ipv4.u.a8[2], outboundFPConfig.srcIP.addr.ipv4.u.a8[3],
            outboundFPConfig.dstIP.addr.ipv4.u.a8[0], outboundFPConfig.dstIP.addr.ipv4.u.a8[1],
            outboundFPConfig.dstIP.addr.ipv4.u.a8[2], outboundFPConfig.dstIP.addr.ipv4.u.a8[3]);

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
    printf ("Debug: Egress IPv4 Fast Path Handle 0x%x is active\n", (uint32_t)egressFPHandle[index]);

    /* Create a socket */
    sockHandle[index] = Netfp_socket (netfpClientHandle, Netfp_SockFamily_AF_INET, &errCode);
    if (sockHandle[index] == NULL)
    {
        printf ("Error: NETFP Socket '%d' Creation Failed [Error Code %d]\n", index, errCode);
        return -1;
    }

    /* Populate the binding information: */
    memset ((void*)&sockAddress, 0, sizeof(Netfp_SockAddr));
    sockAddress.sin_family              = Netfp_SockFamily_AF_INET;
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
    sockAddress.sin_family                  = Netfp_SockFamily_AF_INET;
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
     * EVENT GENERATION: Perform the REKEY SA
     *************************************************************************************************/
    printf ("Debug: Initiating the rekey SA\n");

    /* Cycle through and reset the socket reasons: We are about to start the test */
    for (index = 0; index < TEST_MAX_FAST_PATH; index++)
        sockReason[index] = 0;
    loggingFlag = 1;
    index       = 0;

    /* Set a new SPI for the outbound SA configuration: */
    saConfig.spi = 1000;

    /* Rekey the SA: */
    newSAHandle = Netfp_rekeySA (netfpClientHandle, outboundSAHandle, &saConfig, &errCode);
    if (newSAHandle == NULL)
    {
        printf ("Error: Rekey SA failed [Error code %d]\n", errCode);
        return -1;
    }
    printf ("Debug: SA 0x%p has been successfully rekeyed\n", outboundSAHandle);

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

    /* Because of inheritance the socket will remain ACTIVE always and should never be marked as INACTIVE */
    for (index = 0; index < TEST_MAX_FAST_PATH; index++)
    {
        if ((sockReason[index] == Netfp_Reason_SP_INACTIVE) || (sockReason[index] == Netfp_Reason_NEIGH_UNREACHABLE))
        {
            printf ("Error: Invalid Socket Reason %d @ index %d detected\n", sockReason[index], index);
            return -1;
        }
    }

    /* Cycle through and reset the socket reasons: We are about to start the test */
    for (index = 0; index < TEST_MAX_FAST_PATH; index++)
        sockReason[index] = 0;
    index = 0;

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
        printf ("Error: NETFP Inbound security association deletion failed [Error code %d]\n", errCode);

    /* Delete the outbound SA */
    if (Netfp_delSA (netfpClientHandle, outboundSAHandle, &errCode) < 0)
        printf ("Error: NETFP Outbound security association deletion failed [Error code %d]\n", errCode);

    /* Delete the outbound SA */
    if (Netfp_delSA (netfpClientHandle, newSAHandle, &errCode) < 0)
        printf ("Error: NETFP New Outbound security association deletion failed [Error code %d]\n", errCode);

    /* Delete the default gateway. */
    system ("route del default gateway 192.168.1.1");

    return 0;
}


/**
 *  @b Description
 *  @n
 *      The function is used to test the VLAN Events.
 *
 *      The following is the test setup which is constructed here:
 *
 *      |----------|                  |--------|
 *      |   EVM    |------------------| GW     |
 *      |----------|                  |--------|
 *    VLAN Id: 100   192.168.1.2         192.168.1.1
 *    Default Gw: 192.168.1.1
 *
 *  (a) Sockets are connected with manual interface and routing information
 *  (b) The outbound fast path and sockets will remain active.
 *  (c) Modify the VLAN Egress Map configuration
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Test_vlanEvents(void)
{
    Netfp_InboundFPCfg      inboundFPConfig;
    Netfp_OutboundFPCfg     outboundFPConfig;
    int32_t                 errCode;
    int32_t                 status;
    Netfp_SockAddr          sockAddress;
    Netfp_OptionTLV         optInfo;
    uint8_t                 socketState;
    int32_t                 index = 0;
    Netfp_IfHandle          ifHandle;
    Netfp_InterfaceCfg      ifConfig;
    Netfp_VLANPriorityMap   priorityMap;
    Netfp_OptionTLV         optCfg;
    uint16_t                vlanId;
    uint32_t                switchPort;

    /* Debug Message: */
    printf ("***************************************************************************************\n");
    printf ("Debug: Testing VLAN \n");

    /*************************************************************************************************
     * TEST SETUP:
     *************************************************************************************************/

    /* Initialize the interface configuration */
    memset ((void *)&ifConfig, 0, sizeof(Netfp_InterfaceCfg));

    /* Populate the configuration block. */
    ifConfig.type                          = Netfp_InterfaceType_VLAN;
    ifConfig.mtu                           = 1500;
    ifConfig.vlanId                        = 100;
    ifConfig.ipAddress.ver                 = Netfp_IPVersion_IPV4;
    ifConfig.ipAddress.addr.ipv4.u.a8[0]   = 192;
    ifConfig.ipAddress.addr.ipv4.u.a8[1]   = 168;
    ifConfig.ipAddress.addr.ipv4.u.a8[2]   = 100;
    ifConfig.ipAddress.addr.ipv4.u.a8[3]   = 1;
    ifConfig.subnetMask.ver                = Netfp_IPVersion_IPV4;
    ifConfig.subnetMask.addr.ipv4.u.a8[0]  = 0xFF;
    ifConfig.subnetMask.addr.ipv4.u.a8[1]  = 0xFF;
    ifConfig.subnetMask.addr.ipv4.u.a8[2]  = 0xFF;
    ifConfig.subnetMask.addr.ipv4.u.a8[3]  = 0x0;
    strcpy (ifConfig.name, "eth0.100");

    /* Setup the VLAN priority mapping: By default all NETFP Socket priority are marked as 0 */
    {
        int32_t i;
        for (i = 0; i < NETFP_MAX_SOCK_PRIORITY; i++)
            ifConfig.vlanMap.socketToVLANPriority[i] = 0;
    }

    /* Explicity remap socket priority 2 to VLAN priority 7 */
    ifConfig.vlanMap.socketToVLANPriority[2] = 7;

    /* Explicity remap socket priority 5 to VLAN priority 1 */
    ifConfig.vlanMap.socketToVLANPriority[5] = 1;

    /* Populate a MAC Address. */
    ifConfig.macAddress[0] = 0x00;
    ifConfig.macAddress[1] = 0x01;
    ifConfig.macAddress[2] = 0x02;
    ifConfig.macAddress[3] = 0x03;
    ifConfig.macAddress[4] = 0x04;
    ifConfig.macAddress[5] = 0x05;

    /* Create & Register the Interface with the NETFP Library. */
    ifHandle = Netfp_createInterface (netfpClientHandle, &ifConfig, &errCode);
    if (ifHandle == NULL)
    {
        printf ("Error: Unable to create the NETFP interface [Error code %d]\n", errCode);
        return -1;
    }
    printf ("Debug: Interface '%s' is created with Handle %p\n", ifConfig.name, ifHandle);

    /* Find the interface and get the interface configuration. */
    if (Netfp_findInterface (netfpClientHandle, ifConfig.name, NULL, &errCode) != ifHandle)
    {
        printf ("Error: Unable to find the interface\n");
        return -1;
    }

    /* Create the Ingress fast paths: Initialize the fast path configuration */
    memset ((void *)&inboundFPConfig, 0, sizeof (Netfp_InboundFPCfg));

    /* Populate the fast path configuration: */
    inboundFPConfig.spId                       = NETFP_INVALID_SPID;
    inboundFPConfig.dstIP.ver                  = Netfp_IPVersion_IPV4;
    inboundFPConfig.dstIP.addr.ipv4.u.a8[0]    = 192;
    inboundFPConfig.dstIP.addr.ipv4.u.a8[1]    = 168;
    inboundFPConfig.dstIP.addr.ipv4.u.a8[2]    = 1;
    inboundFPConfig.dstIP.addr.ipv4.u.a8[3]    = 2;
    inboundFPConfig.srcIP.ver                  = Netfp_IPVersion_IPV4;
    inboundFPConfig.srcIP.addr.ipv4.u.a8[0]    = 192;
    inboundFPConfig.srcIP.addr.ipv4.u.a8[1]    = 168;
    inboundFPConfig.srcIP.addr.ipv4.u.a8[2]    = 1;
    inboundFPConfig.srcIP.addr.ipv4.u.a8[3]    = 5;
    snprintf (inboundFPConfig.name, NETFP_MAX_CHAR, "Ingress_FastPath-%d", index);

    /* Add the Ingress Fast Path. */
    ingressFPHandle = Netfp_createInboundFastPath(netfpClientHandle, &inboundFPConfig, &errCode);
    if (ingressFPHandle == NULL)
    {
        printf ("Error: Unable to add the NETFP Inbound Fast path [Error code %d]\n", errCode);
        return -1;
    }
    printf ("Debug: Adding %s [%03d.%03d.%03d.%03d] -> [%03d.%03d.%03d.%03d]\n",
            inboundFPConfig.name, inboundFPConfig.srcIP.addr.ipv4.u.a8[0], inboundFPConfig.srcIP.addr.ipv4.u.a8[1],
            inboundFPConfig.srcIP.addr.ipv4.u.a8[2], inboundFPConfig.srcIP.addr.ipv4.u.a8[3],
            inboundFPConfig.dstIP.addr.ipv4.u.a8[0], inboundFPConfig.dstIP.addr.ipv4.u.a8[1],
            inboundFPConfig.dstIP.addr.ipv4.u.a8[2], inboundFPConfig.dstIP.addr.ipv4.u.a8[3]);

    /* Create the outbound fast path: */
    memset ((void *)&outboundFPConfig, 0, sizeof (Netfp_OutboundFPCfg));

    /* Populate the Outbound Fast Path configuration with a manual interface and
     * next HOP MAC address. This will bypass the Route resolution in NETFP Proxy
     * This is because we dont have a VLAN Setup and are simply simulating the
     * networking setup */
    outboundFPConfig.spId                       = NETFP_INVALID_SPID;
    outboundFPConfig.dstIP.ver                  = Netfp_IPVersion_IPV4;
    outboundFPConfig.dstIP.addr.ipv4.u.a8[0]    = 192;
    outboundFPConfig.dstIP.addr.ipv4.u.a8[1]    = 168;
    outboundFPConfig.dstIP.addr.ipv4.u.a8[2]    = 1;
    outboundFPConfig.dstIP.addr.ipv4.u.a8[3]    = (5 + index);
    outboundFPConfig.srcIP.ver                  = Netfp_IPVersion_IPV4;
    outboundFPConfig.srcIP.addr.ipv4.u.a8[0]    = 192;
    outboundFPConfig.srcIP.addr.ipv4.u.a8[1]    = 168;
    outboundFPConfig.srcIP.addr.ipv4.u.a8[2]    = 1;
    outboundFPConfig.srcIP.addr.ipv4.u.a8[3]    = 2;
    outboundFPConfig.ifHandle                   = ifHandle;
    outboundFPConfig.nextHopMACAddress[0]       = 0xAA;
    outboundFPConfig.nextHopMACAddress[1]       = 0xAA;
    outboundFPConfig.nextHopMACAddress[2]       = 0xAA;
    outboundFPConfig.nextHopMACAddress[3]       = 0xAA;
    outboundFPConfig.nextHopMACAddress[4]       = 0xAA;
    outboundFPConfig.nextHopMACAddress[5]       = 0xAA;
    snprintf (outboundFPConfig.name, NETFP_MAX_CHAR, "Egress_FastPath-%d", index);

    /* Add the Egress Fast Path. */
    egressFPHandle[index] = Netfp_createOutboundFastPath (netfpClientHandle, &outboundFPConfig, &errCode);
    if (egressFPHandle[index] == NULL)
    {
        printf ("Error: Unable to add the NETFP Egress Fast path [Error code %d]\n", errCode);
        return -1;
    }
    printf ("Debug: Adding %s [%03d.%03d.%03d.%03d] -> [%03d.%03d.%03d.%03d]\n",
            outboundFPConfig.name, outboundFPConfig.srcIP.addr.ipv4.u.a8[0], outboundFPConfig.srcIP.addr.ipv4.u.a8[1],
            outboundFPConfig.srcIP.addr.ipv4.u.a8[2], outboundFPConfig.srcIP.addr.ipv4.u.a8[3],
            outboundFPConfig.dstIP.addr.ipv4.u.a8[0], outboundFPConfig.dstIP.addr.ipv4.u.a8[1],
            outboundFPConfig.dstIP.addr.ipv4.u.a8[2], outboundFPConfig.dstIP.addr.ipv4.u.a8[3]);

    /* Get the fast path status */
    if (Netfp_isOutboundFastPathActive (netfpClientHandle, egressFPHandle[index], &status, &errCode) < 0)
    {
        printf ("Error: Getting status of the outbound fast path failed [Error code %d]\n", errCode);
        return -1;
    }

    /* Is the fast path active? */
    if (status == 0)
    {
        printf ("Error: Outbound Fast path was detected to be inactive\n");
        return -1;
    }

    /* Create a socket */
    sockHandle[index] = Netfp_socket (netfpClientHandle, Netfp_SockFamily_AF_INET, &errCode);
    if (sockHandle[index] == NULL)
    {
        printf ("Error: NETFP Socket '%d' Creation Failed [Error Code %d]\n", index, errCode);
        return -1;
    }

    /* Populate the binding information: */
    memset ((void*)&sockAddress, 0, sizeof(Netfp_SockAddr));
    sockAddress.sin_family              = Netfp_SockFamily_AF_INET;
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
    sockAddress.sin_family                  = Netfp_SockFamily_AF_INET;
    sockAddress.sin_port                    = 4000 + index;
    sockAddress.op.connect.outboundFPHandle = egressFPHandle[index];

    /* Connect the socket */
    if (Netfp_connect(sockHandle[index], &sockAddress, &errCode) < 0)
    {
        printf ("Error: NETFP Connect Failed for socket '%d' to port '%d' [Error Code %d]\n",
                index, sockAddress.sin_port, errCode);
        return -1;
    }

    /* Socket should be active becuase the fast path is already active */
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

    /* Get the VLAN Id which is being used by the socket: */
    optInfo.type   = Netfp_Option_VLAN_ID;
    optInfo.length = sizeof(uint16_t);
    optInfo.value  = (void*)&vlanId;
    if (Netfp_getSockOpt (sockHandle[index], &optInfo, &errCode) < 0)
    {
        printf ("Error: Getting socket option for VLAN identifier failed [Error code %d]\n", errCode);
        return -1;
    }
    if (vlanId != ifConfig.vlanId)
    {
        printf ("Error: Incorrect VLAN Identifier Detected Got %d Expected %d\n", vlanId, ifConfig.vlanId);
        return -1;
    }
    printf ("Debug: VLAN Id verification passed\n");

    /* Get the Switch Port which is being used by the socket: */
    optInfo.type   = Netfp_Option_SWITCH_PORT;
    optInfo.length = sizeof(uint32_t);
    optInfo.value  = (void*)&switchPort;
    if (Netfp_getSockOpt (sockHandle[index], &optInfo, &errCode) < 0)
    {
        printf ("Error: Getting socket option for VLAN identifier failed [Error code %d]\n", errCode);
        return -1;
    }
    if (switchPort != TEST_SWITCH_PORT)
    {
        printf ("Error: Incorrect Switch Port Detected Got %d Expected %d\n", switchPort, TEST_SWITCH_PORT);
        return -1;
    }
    printf ("Debug: Switch Port verification passed\n");

    /*************************************************************************************************
     * EVENT GENERATION: Modify the VLAN Egress mapping
     *************************************************************************************************/
    printf ("Debug: Modifying the VLAN Egress Map\n");

    /* Cycle through and reset the socket reasons: We are about to start the test */
    sockReason[index] = 0;

    /* Modify the VLAN Mapping: */
    priorityMap.socketToVLANPriority[0] = 7;
    priorityMap.socketToVLANPriority[1] = 7;
    priorityMap.socketToVLANPriority[2] = 7;
    priorityMap.socketToVLANPriority[3] = 7;
    priorityMap.socketToVLANPriority[4] = 7;
    priorityMap.socketToVLANPriority[5] = 7;
    priorityMap.socketToVLANPriority[6] = 7;

    /* Populate the TLV: */
    optCfg.type   = Netfp_Option_VLAN_EGRESS_PRIORITY;
    optCfg.length = sizeof(Netfp_VLANPriorityMap);
    optCfg.value  = &priorityMap;

    /* Modify the mapping: */
    if (Netfp_setIfOpt (netfpClientHandle, ifHandle, &optCfg, &errCode) < 0)
    {
        printf ("Error: Modifying the VLAN Mapping failed [Error code %d]\n", errCode);
        return -1;
    }

    /*************************************************************************************************
     * VALIDATIONS: We should have detected a valid reason on each socket.
     *************************************************************************************************/
    if (sockReason[index] == 0)
    {
        /* Error: No reason detected for the socket. */
        printf ("Error: Socket handle 0x%x index %d was not notified\n", (uint32_t)sockHandle[index], index);
        return -1;
    }

    /* Each socket should have received an interface update event which indicates that the L3 QOS configuration was changed */
    if (sockReason[index] != Netfp_Reason_VLAN_EGRESS)
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

    /* Delete the VLAN Interface: */
    if (Netfp_deleteInterface (netfpClientHandle, ifHandle, &errCode) < 0)
        printf ("Error: NETFP Interface deletion failed [Error code %d]\n", errCode);

    return 0;
}

/**
 *  @b Description
 *  @n
 *      Function open a socket to communicate with Netfp proxy.
 *      Currently only sending the ip route flush command is supported
 *
 *  @param[in]  reason
 *      Reason code because of which the socket was notified.
 *  @param[in]  sockHandle
 *      Socket Handle which was affected
 *
 *  @retval
 *      Not applicable
 */
static int32_t Test_setupNetfpProxy_communication (uint32_t proxyPid)
{
    char                sunPath[64];

    printf ("Opening socket to communicate with Netfp Proxy_%d\n", proxyPid);

    /* Open a Unix socket to talk to NetFP Proxy daemon */
    if ((cmdAppIpcSockFd = socket (AF_UNIX, SOCK_DGRAM, 0)) < 0)
    {
        printf ("Socket open failed, error: %d \n", errno);
        return -1;
    }

    /* Setup the socket to listen on messages from NetFP Proxy. */
    memset ((void *)&cmdAppSunAddr, 0, sizeof (struct sockaddr_un));
    snprintf (sunPath, sizeof(sunPath), "%s/%s_%d", Syslib_getRunTimeDirectory(), NETFP_PROXY_IPC_DEFAULT_USER_PATH, proxyPid);
    strcpy (cmdAppSunAddr.sun_path, sunPath);

    /* Unlink the socket path to ensure this is the only socket instance
     * running */
    unlink (cmdAppSunAddr.sun_path);

    /* Bind the socket to our unix path name */
    cmdAppSunAddr.sun_family = AF_UNIX;
    if (bind (cmdAppIpcSockFd, (const struct sockaddr *)&cmdAppSunAddr, sizeof (struct sockaddr_un)) < 0)
    {
        printf ("Socket bind failed, error: %d \n", errno);
        close (cmdAppIpcSockFd);
        return -1;
    }

    /* Also setup socket address for NetFP Proxy daemon. Will be
     * used to send messages to the daemon */
    memset ((void *)&proxySunAddr, 0, sizeof (struct sockaddr_un));
    proxySunAddr.sun_family = AF_UNIX;
    snprintf (sunPath, sizeof(sunPath), "%s/%s_%d", Syslib_getRunTimeDirectory(), NETFP_PROXY_IPC_DEFAULT_DAEMON_PATH, proxyPid);
    strcpy (proxySunAddr.sun_path, sunPath);

    /* Successfully opened the socket */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Entry point to the test task which executes the route recalculation
 *      unit tests
 *
 *  @retval
 *      Not Applicable.
 */
void Test_routeRecalculationThread(void* arg)
{
    Msgcom_ChannelCfg   chConfig;
    Netfp_FlowCfg       flowCfg;
    int32_t             errCode;
    uint32_t*           proxyPid = (uint32_t*)arg;

    printf ("*******************************************************\n");
    printf ("************* Route Recalculation Testing *************\n");
    printf ("*******************************************************\n");

    /* Initialize the channel configuration. */
    memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* Populate the channel configuration. */
    chConfig.mode                       = Msgcom_ChannelMode_NON_BLOCKING;
    chConfig.appCallBack                = NULL;
    chConfig.msgcomInstHandle           = appMsgcomInstanceHandle;
    chConfig.u.queueCfg.interruptMode   = Msgcom_QueueInterruptMode_NO_INTERRUPT;

    /* Create the MSGCOM channel. */
    dummyChannel = Msgcom_create ("DummyChannel", Msgcom_ChannelType_QUEUE, &chConfig, &errCode);
    if (dummyChannel == NULL)
    {
        printf ("Error: Unable to open the dummy channel [Error code: %d]\n", errCode);
        return;
    }

    /* Populate the flow configuration: Use the NETFP Data Receive Heap for this purpose. */
    sprintf (flowCfg.name, "DummyFlow");
    flowCfg.numHeaps      = 1;
    flowCfg.heapHandle[0] = netfpDataRxHeap;
    flowCfg.sopOffset     = 0;

    /* Create the flow. */
    dummyFlowId = Netfp_createFlow (netfpClientHandle, &flowCfg, &errCode);
    if (dummyFlowId < 0)
    {
        printf ("Error: Unable to create the flow '%s', [Error code %d]\n", flowCfg.name, errCode);
        return;
    }

    /* Setup the communication to NetFP Proxy daemon */
    if (Test_setupNetfpProxy_communication(*proxyPid) < 0)
    {
        printf ("Error: Unable to communicate with Netfp Proxy\n");
        return;
    }

    /* Debug Message: */
    printf ("***************************************************************************************\n");
    printf ("The TEST assumes the following test environment.\n\n");
    printf ("        |----------|                  |--------|\n");
    printf ("        |   EVM    |------------------| GW     |\n");
    printf ("        |----------|                  |--------|\n");
    printf ("            192.168.1.2            192.168.1.1  \n");
    printf ("            7000::2                7000::1      \n");
    printf ("            Default Gw: 192.168.1.1\n\n");
    printf ("            Default Gw: 7000::1\n\n");
    printf ("NOTE: Failure to have the above setup will cause the route resolution to fail\n");
    printf ("***************************************************************************************\n");

    /***********************************************************************************
     * Execute the tests:
     ***********************************************************************************/
    if (Test_v4NonSecureFPEvents() < 0)
        return;
    if (Test_v4InterfaceEvents() < 0)
        return;
    if (Test_v4SecureFPEvents() < 0)
        return;
    if (Test_v4SecureFPEventsDefaultGateway() < 0)
        return;
    if (Test_v4SocketStates() < 0)
        return;
    if (Test_vlanEvents() < 0)
        return;
    if (Test_v6NonSecureFPEvents() < 0)
        return;
    if (Test_v6InterfaceEvents() < 0)
        return;
    if (Test_v6SecureFPEvents() < 0)
        return;
    if (Test_v6SocketStates() < 0)
        return;

    /* All the tests passed. */
    printf ("Debug: Route Recalculation Testing passed\n");
    printf ("----------------------------------------------------------------\n");
    return;
}

