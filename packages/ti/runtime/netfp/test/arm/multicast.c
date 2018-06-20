/*
 *   @file  multicast.c
 *
 *   @brief
 *      Test for receiving and handling Multicast Packets
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
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <getopt.h>
#include <unistd.h>
#include <limits.h>

/* PDK Include Files. */
#include <ti/csl/csl_chip.h>
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/drv/rm/rm.h>

/* SYSLIB Include Files.  */
#include <ti/runtime/msgcom/msgcom.h>
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/netfp/netfp.h>
#include <ti/apps/netfp_master/netfp_master.h>

#include "benchmark.h"

/**********************************************************************
 *********************** Unit Test Local Definitions ******************
 **********************************************************************/

/* Base UDP port number for the test */
#define TEST_SOCKET_IPV4_BASE_UDP_PORT          5000

/**********************************************************************
 ************************ Unit Test Global variables ******************
 **********************************************************************/

/* Global Counter for number of all Multicast Packets received */
uint32_t        gNumAllMulticastPktRxed     = 0;

/* Global Counter for number of Multicast UDP-IPv4 Packets received */
uint32_t        gNumMulticastPktRxed     = 0;

/**********************************************************************
 ************************ Extern Declarations *************************
 **********************************************************************/

/* NETFP Heap for data receive and transmit. */
extern Pktlib_HeapHandle   netfpDataRxHeap;
extern volatile uint32_t   testComplete;
extern Netfp_ClientHandle  netfpClientHandle;

/* Global MSGCOM instance handle. */
extern Msgcom_InstHandle   appMsgcomInstanceHandle;

/* PKTLIB Instance handle */
extern Pktlib_InstHandle   appPktlibInstanceHandle;

/**********************************************************************
 ************************** Unit Test Functions ***********************
 **********************************************************************/

/**
 *  @b Description
 *  @n
 *      The function is used to cleanup the MSGCOM data buffers if there are
 *      any pending packets while deleting the channel.
 *
 *  @param[in]  pktlibInstHandle
 *      PKTLIB Instance handle associated with the MSGCOM instance
 *  @param[in]  chHandle
 *      MSGCOM Channel Handle which is being deleted.
 *  @param[in]  msgBuffer
 *      MSGCOM Buffer to be deleted.
 *
 *  @retval
 *      Not Applicable.
 */
static void myFreeMsgBuffer(Pktlib_InstHandle pktlibInstHandle, MsgCom_ChHandle chHandle, MsgCom_Buffer* msgBuffer)
{
    printf ("Debug: Channel Handle 0x%p is being deleted MSGCOM Buffer is 0x%p\n", chHandle, msgBuffer);
    Pktlib_freePacket(pktlibInstHandle, (Ti_Pkt*)msgBuffer);
}

/**
 *  @b Description
 *  @n
 *      The function is the bridge application which is used to receive the multicast packet.
 *
 *  @param[in]  ptrPkt
 *      Pointer to the received packet
 *
 *  @retval
 *      Not Applicable.
 */
static void Test_bridgeApplication (Ti_Pkt* ptrPkt)
{
    Netfp_RawDst    rawDst;
    int32_t         errCode;
    Ti_Pkt*         ptrClonePkt1;
    Ti_Pkt*         ptrClonePkt2;

    /* Increment the global counter: */
    gNumAllMulticastPktRxed++;

    /* Allocate a zero buffer packet for cloning: */
    ptrClonePkt1 = Pktlib_allocPacket (appPktlibInstanceHandle, netfpDataRxHeap, 0);
    if (ptrClonePkt1 == NULL)
    {
        printf ("Error: Unable to allocate a packet for cloning\n");
        return;
    }

    /* Allocate a zero buffer packet for cloning: */
    ptrClonePkt2 = Pktlib_allocPacket (appPktlibInstanceHandle, netfpDataRxHeap, 0);
    if (ptrClonePkt2 == NULL)
    {
        printf ("Error: Unable to allocate a packet for cloning\n");
        return;
    }

    /* Clone the received packet: */
    Pktlib_clonePacket (appPktlibInstanceHandle, ptrPkt, ptrClonePkt1);
    Pktlib_clonePacket (appPktlibInstanceHandle, ptrPkt, ptrClonePkt2);

    /* Copy over the meta data from the original packet into the cloned packet */
    Pktlib_copyMetaData (appPktlibInstanceHandle, ptrClonePkt1, ptrPkt);
    Pktlib_copyMetaData (appPktlibInstanceHandle, ptrClonePkt2, ptrPkt);

    /* Initialize the raw destination: */
    memset ((void *)&rawDst, 0, sizeof(Netfp_RawDst));

    /* Populate the RAW Destination: We are sending the packet to "eth1" */
    rawDst.type                = Netfp_RawDstType_SWITCH_PORT;
    rawDst.dst.switchPort.port = 2;
    if (Netfp_rawSend (netfpClientHandle, ptrClonePkt1, &rawDst, &errCode) < 0)
        printf ("Error: Unable to send the raw packet to the switch port [Error code %d]\n", errCode);

    /* Populate the RAW Destination: We are sending the packet to the fast path */
    rawDst.type                  = Netfp_RawDstType_FAST_PATH;
    rawDst.dst.fastPath.version  = Netfp_IPVersion_IPV4;
    rawDst.dst.fastPath.l3Offset = 14;
    if (Netfp_rawSend (netfpClientHandle, ptrClonePkt2, &rawDst, &errCode) < 0)
        printf ("Error: Unable to send the raw packet to the fast path [Error code %d]\n", errCode);

    /* Populate the RAW Destination: Send the packet back to Linux */
    rawDst.type = Netfp_RawDstType_CONTROL_PATH;
    if (Netfp_rawSend (netfpClientHandle, ptrPkt, &rawDst, &errCode) < 0)
        printf ("Error: Unable to send the raw packet to the control path [Error code %d]\n", errCode);

    /* Execute the garbage collection; since all the packets will end up here after the CPDMA */
    Pktlib_garbageCollection (appPktlibInstanceHandle, netfpDataRxHeap);
    return;
}

/**
 *  @b Description
 *  @n
 *      This is the receive task which receives the packets and processes them.
 *
 *  @retval
 *      Not Applicable.
 */
void* Test_mbmsThread(void* arg)
{
    Netfp_InboundFPHandle   fpIngressHandle;
    Netfp_SockHandle        sockHandle;
    Netfp_SockAddr          netfpSockAddress;
    struct sockaddr_un      sockAddress;
    Msgcom_ChannelCfg       chConfig;
    MsgCom_ChHandle         multicastChannel;
    MsgCom_ChHandle         multicastSocketChannel;
    Netfp_FlowCfg           flowCfg;
    int32_t                 multicastFlowId;
    int32_t                 errCode;
    Ti_Pkt*                 ptrRxPkt;
    int32_t                 numBytes;
    NetfpMaster_Request     request;
    NetfpMaster_Response    response;
    int32_t                 sockfd;
    struct sockaddr_un      from;
    struct sockaddr_un      to;
    socklen_t               fromLen;
    char                    socketName[PATH_MAX];

    /* Populate the flow configuration: Use the NETFP Data Receive Heap for this purpose. */
    strcpy (flowCfg.name, "MulticastTestFlow");
    flowCfg.numHeaps      = 1;
    flowCfg.heapHandle[0] = netfpDataRxHeap;
    flowCfg.sopOffset     = 0;

    /* Create the flow. */
    multicastFlowId = Netfp_createFlow (netfpClientHandle, &flowCfg, &errCode);
    if (multicastFlowId < 0)
    {
        printf ("Error: Unable to create the multicast flow [Error code %d]\n", errCode);
        return NULL;
    }
    printf ("Debug: Multicast Flow %d has been created successfully\n", multicastFlowId);

    /* Initialize the channel configuration. */
    memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* Populate the channel configuration. */
    chConfig.mode                       = Msgcom_ChannelMode_NON_BLOCKING;
    chConfig.appCallBack                = NULL;
    chConfig.msgcomInstHandle           = appMsgcomInstanceHandle;
    chConfig.u.queueCfg.interruptMode   = Msgcom_QueueInterruptMode_NO_INTERRUPT;

    /* Create the Multicast MSGCOM channel. */
    multicastChannel = Msgcom_create ("Multicast-Channel", Msgcom_ChannelType_QUEUE, &chConfig, &errCode);
    if (multicastChannel == 0)
    {
        printf ("Error: Unable to open the Multicast Channel Error : %d\n", errCode);
        return NULL;
    }

    /* Initialize the channel configuration. */
    memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* Populate the channel configuration. */
    chConfig.mode                       = Msgcom_ChannelMode_NON_BLOCKING;
    chConfig.appCallBack                = NULL;
    chConfig.msgcomInstHandle           = appMsgcomInstanceHandle;
    chConfig.u.queueCfg.interruptMode   = Msgcom_QueueInterruptMode_NO_INTERRUPT;

    /* Create the Multicast Socket MSGCOM channel. */
    multicastSocketChannel = Msgcom_create ("Multicast-Socket-Channel", Msgcom_ChannelType_QUEUE, &chConfig, &errCode);
    if (multicastSocketChannel == 0)
    {
        printf ("Error: Unable to open the Multicast Socket Channel Error : %d\n", errCode);
        return NULL;
    }

    /* Create the socket name: */
    snprintf (socketName, PATH_MAX, "%s/Test-%d", Syslib_getRunTimeDirectory(), getpid());

    /* Unlink and remove any previous socket names */
    unlink (socketName);

    /* Create the test socket. */
    sockfd = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (sockfd < 0)
    {
        printf ("Error: NETFP Master Test socket failed %s\n", strerror(errno));
        return NULL;
    }

    /* Initialize the socket address */
    memset(&sockAddress, 0, sizeof(sockAddress));

    /* Populate the binding information */
    sockAddress.sun_family = AF_UNIX;
    strncpy(sockAddress.sun_path, socketName, sizeof(sockAddress.sun_path));

    /* Bind the socket: */
    if (bind(sockfd, (struct sockaddr*)&sockAddress, sizeof(struct sockaddr_un)) < 0)
    {
        printf("Error: Socket bind failed (error: %s)\n", strerror(errno));
        return NULL;
    }
    printf ("Debug: NETFP master test socket is operational\n");

    /* We always send requests to the NETFP Master */
    memset ((void *)&to, 0, sizeof(struct sockaddr_un));
    to.sun_family = AF_UNIX;
    snprintf(to.sun_path, PATH_MAX, "%s/%s", Syslib_getRunTimeDirectory(), NETFP_MASTER_SOCKET_NAME);

    /* Send a message to the NETFP Master: */
    request.msgType = NetfpMaster_MessageType_SET_PRECLASSIFICATION;
    request.id      = 100;
    strcpy (request.u.setPreclassification.ifName, "eth0");
    request.u.setPreclassification.isBroadcast              = 0;
    request.u.setPreclassification.enablePreclassfication   = 1;
    request.u.setPreclassification.preclassificationFlowId  = multicastFlowId;
    request.u.setPreclassification.preclassificationQueueId = Msgcom_getInternalMsgQueueInfo(multicastChannel);

    /* Send the request to the NETFP master */
    numBytes = sendto(sockfd, &request, sizeof(NetfpMaster_Request), 0, (struct sockaddr *)&to, sizeof(to));
    if (numBytes < 0)
        return NULL;

    /* Debug Message: */
    printf ("Debug: Multicast Preclassification configuration request [%s] sent to the NETFP Master\n", request.u.setPreclassification.ifName);

    /* Wait for the response from the NETFP Master */
    numBytes = recvfrom(sockfd, &response, sizeof(NetfpMaster_Response), 0, (struct sockaddr *)&from, &fromLen);
    if (numBytes < 0)
        return NULL;

    /* Was the response successful? */
    if (response.errCode != 0)
    {
        printf ("Error: Multicast Preclassification configuration failed [Error code %d]\n", response.errCode);
        return NULL;
    }
    printf ("Debug: Multicast Preclassification configuration successful\n");

    /* Get the multicast ingress fast path handle */
    fpIngressHandle = Netfp_findInboundFastPath (netfpClientHandle, "Multicast-IPv4-FastPath", &errCode);
    if (fpIngressHandle == NULL)
    {
        printf ("Error: Unable to find the ingress fast path [Error code %d]\n", errCode);
        return NULL;
    }

    /* Create a socket */
	sockHandle = Netfp_socket (netfpClientHandle, Netfp_SockFamily_AF_INET, &errCode);
    if (sockHandle == NULL)
	{
        printf ("Error: NETFP Socket Creation Failed [Error Code %d]\n", errCode);
	    return NULL;
    }

    /* Populate the binding information */
    memset ((void*)&netfpSockAddress, 0, sizeof(Netfp_SockAddr));
	netfpSockAddress.sin_family              = Netfp_SockFamily_AF_INET;
	netfpSockAddress.sin_port                = TEST_SOCKET_IPV4_BASE_UDP_PORT;
    netfpSockAddress.op.bind.flowId          = multicastFlowId;
	netfpSockAddress.op.bind.queueHandle     = Msgcom_getInternalMsgQueueInfo(multicastSocketChannel);
    netfpSockAddress.op.bind.inboundFPHandle = fpIngressHandle;
    netfpSockAddress.op.bind.notifyFunction  = NULL;

    /* Bind the socket. */
	if (Netfp_bind (sockHandle, &netfpSockAddress, &errCode) < 0)
    {
	    printf ("Error: NETFP Bind Failed [Error Code %d]\n", errCode);
		return NULL;
    }
	printf ("Debug: Socket has been bound successfully\n");

    /* Debug Message: */
    printf ("------------------------------------------------------------------------------\n");
    printf ("Please use a traffic generation application to send MULTICAST packets to the UDP Port %d\n", netfpSockAddress.sin_port);
    printf ("------------------------------------------------------------------------------\n");

    /**********************************************************************************
     * Stress Test:
     *  This is the main loop where the task waits for data to arrive from the
     *  sockets.
     *********************************************************************************/
    while (testComplete == 0)
    {
        /* Check if a multicast packet has been received on the the MULTICAST Channel: All Multicast packets should
         * be received here irrespective of the address */
        Msgcom_getMessage (multicastChannel, (MsgCom_Buffer**)&ptrRxPkt);

        /* Process the packet: */
        if (ptrRxPkt != NULL)
            Test_bridgeApplication (ptrRxPkt);

        /* Check if a multicast packet has been received on the the MULTICAST Socket Channel:
         * We should receive only UDP Packets to port bound to here. */
        Msgcom_getMessage (multicastSocketChannel, (MsgCom_Buffer**)&ptrRxPkt);

        /* Process the packet: */
        if (ptrRxPkt != NULL)
        {
            /* This implies that the packet received matched the and has reached the SOCKET (UDP Port Match) */
            printf ("Debug: Received MULTICAST packet for the UDP Port %d\n", netfpSockAddress.sin_port);

            /* Increment the statistics: */
            gNumMulticastPktRxed++;
        }
    }

    /* Shutdown the socket */
    printf ("Debug: Closing the NETFP Socket\n");
    if (Netfp_closeSocket (sockHandle, &errCode) < 0)
    {
	    printf ("Error: NETFP socket close failed [Error Code %d]\n", errCode);
		return NULL;
    }
    printf ("Debug: NETFP Socket closed successfully\n");

    /* Shutdown the MSGCOM channel */
    errCode = Msgcom_delete (multicastChannel, myFreeMsgBuffer);
    if (errCode < 0)
    {
	    printf ("Error: MSGCOM Multicast channel close failed [Error Code %d]\n", errCode);
		return NULL;
    }
    printf ("Debug: MSGCOM Multicast channel deleted successfully\n");

    /* Shutdown the MSGCOM channel */
    errCode = Msgcom_delete (multicastSocketChannel, myFreeMsgBuffer);
    if (errCode < 0)
    {
	    printf ("Error: MSGCOM Multicast socket channel close failed [Error Code %d]\n", errCode);
		return NULL;
    }
    printf ("Debug: MSGCOM Multicast socket channel deleted successfully\n");

    /* TODO: Shutdown the NETFP Flow */

    return NULL;
}


