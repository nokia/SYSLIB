/*
 *   @file  gtpuControlMsg.c
 *
 *   @brief   
 *      The file implements loopback testing for GTPU control messages
 *      and to ensure that these messages are received in the proper
 *      queues.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2013-2014 Texas Instruments, Inc.
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

/* BIOS/XDC Include Files. */
#include <xdc/std.h>
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Memory.h> 
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <ti/sysbios/family/c64p/Hwi.h>

/* IPC Include Files for Shared Memory Allocation. */
#include <ti/ipc/Ipc.h>
#include <ti/ipc/SharedRegion.h>

/* MCSDK Include files */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/csl/csl_chip.h>
#include <ti/csl/csl_cacheAux.h>

/* SYSLIB Include Files.  */
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/netfp/netfp.h>

/* Test Include Files*/
#include <net.h>

/**********************************************************************
 ************************** Extern Functions **************************
 **********************************************************************/

/* Heap for data receive and transmit. */
extern Pktlib_HeapHandle        mtuReceiveHeap;

/* NETFP Client Handle: */
extern Netfp_ClientHandle       netfpClientHandle;

/* eNodeB MAC Address: */
extern uint8_t                  eNBMacAddress[6];

/* PKTLIB Instance Handle. */
extern Pktlib_InstHandle        appPktlibInstanceHandle;

/* Extern functions: */
extern int32_t Test_netSendIPv4Payload
(
    uint8_t*    ptrPayload,
    uint16_t    payloadLen,
    uint32_t    gtpuId,
    uint16_t    gtpuMsgType,
    uint16_t    dstPort,
    uint16_t    srcPort,
    uint8_t*    dstIP,
    uint8_t*    srcIP,
    uint8_t     tos,
    uint8_t*    dstMAC,
    uint8_t*    srcMAC,
    uint32_t    loopback
);
extern void appInvalidateBuffer(void* ptr, uint32_t size);

/**********************************************************************
 ************************** Unit Test Globals *************************
 **********************************************************************/

/* eNodeB IP Address: This is the address which is specified in the test vectors. */
static uint8_t  testNodeBIPAddress[] = { 192, 168, 1, 1 };

/* Serving Gateway IP Address: This is the address which is specified in the test vectors. */
static uint8_t  testSeGwIPAddress[]   = { 192, 168, 1, 2 };

/* Serving Gateway MAC Address: */
static uint8_t  testSeGwMacAddress[] = { 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA };

/**********************************************************************
 ************************** Unit Test Functions ***********************
 **********************************************************************/

/**
 *  @b Description
 *  @n  
 *      The function is used to create the IPv4 test environment. In the loopback
 *      testing we are only interested in creating an ingress fast path since the
 *      test simulate different types of packets and their behavior on reception
 *      in the NETCP subsystem.
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t Test_setupGTPUControlMsgEnv(void)
{
    Netfp_InboundFPHandle       fpIngressHandle;
    Netfp_OutboundFPHandle      fpEgressHandle;
    int32_t                     errCode;
    Netfp_InboundFPCfg          inboundFPCfg;
    Netfp_OutboundFPCfg         outboundFPCfg;
    uint32_t 			        index;
    Netfp_IfHandle              ifHandle;
    Netfp_InterfaceCfg          ifConfig;

    /* Display the configuration: */
    System_printf ("-------------------------------------------------------------\n");
    System_printf ("NETFP Configuration: \n");
    System_printf ("eNodeB MAC  Address: 0x%02x:0x%02x:0x%02x:0x%02x:0x%02x:0x%02x\n",
                    eNBMacAddress[0], eNBMacAddress[1],eNBMacAddress[2], 
                    eNBMacAddress[3], eNBMacAddress[4], eNBMacAddress[5]);
    System_printf ("SeGw   MAC  Address: 0x%02x:0x%02x:0x%02x:0x%02x:0x%02x:0x%02x\n", 
                    testSeGwMacAddress[0], testSeGwMacAddress[1], testSeGwMacAddress[2], 
                    testSeGwMacAddress[3], testSeGwMacAddress[4], testSeGwMacAddress[5]);
    System_printf ("eNodeB IPv4 Address: %02d.%02d.%02d.%02d\n", 
                    testNodeBIPAddress[0], testNodeBIPAddress[1],
                    testNodeBIPAddress[2], testNodeBIPAddress[3]);
    System_printf ("SeGW   IPv4 Address: %02d.%02d.%02d.%02d\n", 
                    testSeGwIPAddress[0], testSeGwIPAddress[1],
                    testSeGwIPAddress[2], testSeGwIPAddress[3]);
    System_printf ("-------------------------------------------------------------\n");

    /***************************************************************************
     * Setup the Interface in the NETFP Library.
     *  - The NETFP library needs to know about the networking interfaces which
     *    are participating in the Fast Path.
     *
     * For the test code there is one known interface "eth0". On TN & SCBP this
     * is the NDK created interface but on Appleton this is the ARM created
     * network interface.
     ***************************************************************************/
    memset ((void *)&ifConfig, 0, sizeof(Netfp_InterfaceCfg));
    ifConfig.type                          = Netfp_InterfaceType_ETH;    
    ifConfig.mtu                           = 1500;
    ifConfig.ipAddress.ver                 = Netfp_IPVersion_IPV4;
    ifConfig.ipAddress.addr.ipv4.u.a8[0]   = testNodeBIPAddress[0];
    ifConfig.ipAddress.addr.ipv4.u.a8[1]   = testNodeBIPAddress[1];
    ifConfig.ipAddress.addr.ipv4.u.a8[2]   = testNodeBIPAddress[2];
    ifConfig.ipAddress.addr.ipv4.u.a8[3]   = testNodeBIPAddress[3];
    ifConfig.subnetMask.ver                = Netfp_IPVersion_IPV4;
    ifConfig.subnetMask.addr.ipv4.u.a8[0]  = 0xFF;
    ifConfig.subnetMask.addr.ipv4.u.a8[1]  = 0xFF;
    ifConfig.subnetMask.addr.ipv4.u.a8[2]  = 0xFF;
    ifConfig.subnetMask.addr.ipv4.u.a8[3]  = 0x0;
    strcpy (ifConfig.name, "eth0");
    memcpy ((void *)&ifConfig.macAddress, (void *)&eNBMacAddress[0], 6);

    /* Create & Register the Interface with the NETFP Library. */
    ifHandle = Netfp_createInterface (netfpClientHandle, &ifConfig, &errCode);
    if (ifHandle == NULL)
    {
        printf ("Error: Unable to create the NETFP interface\n");
        return -1;
    }
    printf ("Debug: Interface '%s' is created with Handle 0x%x\n", ifConfig.name, ifHandle);

    /************************************************************************************
     * Ingress IPv4 Fast Path:
     ************************************************************************************/

    /* Initialize the fast path configuration. */
    memset ((void *)&inboundFPCfg, 0, sizeof (Netfp_InboundFPCfg));

    /* Populate the Fast Path configuration. */
    inboundFPCfg.spId                       = NETFP_INVALID_SPID;
    inboundFPCfg.spidMode                   = Netfp_SPIDMode_INVALID;
    inboundFPCfg.srcIP.ver                  = Netfp_IPVersion_IPV4;
    inboundFPCfg.srcIP.addr.ipv4.u.a8[0]    = testSeGwIPAddress[0];
    inboundFPCfg.srcIP.addr.ipv4.u.a8[1]    = testSeGwIPAddress[1];
    inboundFPCfg.srcIP.addr.ipv4.u.a8[2]    = testSeGwIPAddress[2];
    inboundFPCfg.srcIP.addr.ipv4.u.a8[3]    = testSeGwIPAddress[3];
    inboundFPCfg.dstIP.ver                  = Netfp_IPVersion_IPV4;
    inboundFPCfg.dstIP.addr.ipv4.u.a8[0]    = testNodeBIPAddress[0];
    inboundFPCfg.dstIP.addr.ipv4.u.a8[1]    = testNodeBIPAddress[1];
    inboundFPCfg.dstIP.addr.ipv4.u.a8[2]    = testNodeBIPAddress[2];
    inboundFPCfg.dstIP.addr.ipv4.u.a8[3]    = testNodeBIPAddress[3];
    strcpy (inboundFPCfg.name, "Ingress-IPv4-FastPath");

    /* Add the Ingress Fast Path. */
    fpIngressHandle = Netfp_createInboundFastPath (netfpClientHandle, &inboundFPCfg, &errCode);
    if (fpIngressHandle == NULL)
    {
        System_printf ("Error: Unable to create Ingress fast path Error %d\n", errCode);
        return -1;
    }
    System_printf ("Debug: Ingress IPv4 Fast Path Handle 0x%p\n", fpIngressHandle);    

    /* Initialize the fast path configuration */
    memset ((void *)&outboundFPCfg, 0, sizeof (Netfp_OutboundFPCfg));

    /* Populate the Fast Path configuration. */
    outboundFPCfg.spId                       = NETFP_INVALID_SPID;
    outboundFPCfg.dstIP.ver                  = Netfp_IPVersion_IPV4;
    outboundFPCfg.dstIP.addr.ipv4.u.a8[0]    = testSeGwIPAddress[0];
    outboundFPCfg.dstIP.addr.ipv4.u.a8[1]    = testSeGwIPAddress[1];
    outboundFPCfg.dstIP.addr.ipv4.u.a8[2]    = testSeGwIPAddress[2];
    outboundFPCfg.dstIP.addr.ipv4.u.a8[3]    = testSeGwIPAddress[3];
    outboundFPCfg.srcIP.ver                  = Netfp_IPVersion_IPV4;
    outboundFPCfg.srcIP.addr.ipv4.u.a8[0]    = testNodeBIPAddress[0];
    outboundFPCfg.srcIP.addr.ipv4.u.a8[1]    = testNodeBIPAddress[1];
    outboundFPCfg.srcIP.addr.ipv4.u.a8[2]    = testNodeBIPAddress[2];
    outboundFPCfg.srcIP.addr.ipv4.u.a8[3]    = testNodeBIPAddress[3];
    strcpy (outboundFPCfg.name, "Egress-IPv4-FastPath");

    /* Fill in the mapping table */
    for (index = 0; index < 64; index++)
        outboundFPCfg.dscpMapping[index] = index;

    /* Populate the manual routing section: */
    outboundFPCfg.ifHandle = ifHandle;
    outboundFPCfg.nextHopMACAddress[0] = testSeGwMacAddress[0];
    outboundFPCfg.nextHopMACAddress[1] = testSeGwMacAddress[1];
    outboundFPCfg.nextHopMACAddress[2] = testSeGwMacAddress[2];
    outboundFPCfg.nextHopMACAddress[3] = testSeGwMacAddress[3];
    outboundFPCfg.nextHopMACAddress[4] = testSeGwMacAddress[4];
    outboundFPCfg.nextHopMACAddress[5] = testSeGwMacAddress[5];

    /* Add the Egress Fast Path. */
    fpEgressHandle = Netfp_createOutboundFastPath(netfpClientHandle, &outboundFPCfg, &errCode);
    if (fpEgressHandle == NULL)
    {
        System_printf ("Error: Unable to create Egress fast path Error %d\n", errCode);
        return -1;
    }
    System_printf ("Debug: Egress IPv4 Fast Path Handle 0x%p\n", fpEgressHandle);

    return 0;
}

/**
 *  @b Description
 *  @n  
 *      The function is used to test the GTPU Ping Request Messages
 *      and ensure that these packets end up on the correct NETFP 
 *      configured queue.
 *
 *  @param[in]  numIterations
 *      Number of iterations for which the test is to execute
 *  @param[in]  msgType
 *      GTPU Message type
 *  @param[in]  ptrGTPUControlCfg
 *      Pointer to the GTPU control configuration
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t Test_GTPUControlMsg
(
    uint8_t                 numIterations,
    uint16_t                msgType,
    Netfp_GTPUControlCfg*   ptrGTPUControlCfg
)
{
    uint32_t            payloadLength = 100;
    uint32_t            index;
    int32_t             retVal = 0;
    uint32_t            gtpuIdentifier = 0xdead00;
    uint32_t            rxPayloadLength, rxGtpuId;
    uint8_t*            ptrDataBuffer;
    Qmss_QueueHnd       gtpuControlQueue;
    Ti_Pkt*             ptrPkt;
    Net_GTPUHdr*        ptrGTPUHdr;
    Netfp_PeerSockAddr  sockAddr;

    /* Determine the Control Queue to be used. */
    switch (msgType)
    {
        case NET_GTP_PING_REQUEST:
        {
            gtpuControlQueue = ptrGTPUControlCfg->gtpuPingReqQueueHnd;
            break;
        }
        case NET_GTP_PING_RESPONSE:
        {
            gtpuControlQueue = ptrGTPUControlCfg->gtpuPingRespQueueHnd;
            break;
        }
        case NET_GTP_ERROR_INDICATION:
        {
            gtpuControlQueue = ptrGTPUControlCfg->gtpuErrorIndQueueHnd;
            break;
        }
        case NET_GTP_HEADER_NOTIFY:
        {
            gtpuControlQueue = ptrGTPUControlCfg->gtpuHdrNotifyQueueHnd;
            break;
        }
        case NET_GTP_END_MARKER:
        {
            gtpuControlQueue = ptrGTPUControlCfg->gtpuEndMarkerQueueHnd;
            break;
        }
        case NET_GTP_PARSING_ERROR:
        {
            gtpuControlQueue = ptrGTPUControlCfg->gtpuErrorQueueHnd;
            break;
        }
        case NET_GTP_DATA:
        {
            gtpuControlQueue = ptrGTPUControlCfg->gtpuIdMatchFailQueueHnd;
            break;
        }
        default:
        {
            System_printf ("Error: Invalid Message Type detected\n");
            return -1;
        }
    }

    /* Cycle through all the specified iterations. */
    for (index = 0; index < numIterations; index++)
    {
        /* Allocate memory for the payload. */
        ptrDataBuffer = Memory_alloc (NULL, payloadLength, 0, NULL);
        if (ptrDataBuffer == NULL)
        {
            System_printf ("Error: Unable to allocate memory for the GTPU Control Payload (%d bytes)\n", 
                           payloadLength);
            return -1;
        }

        /* Initialize the allocated memory block. */
        memset ((void *)ptrDataBuffer, 0, payloadLength);

        /* Populate the payload */
        *ptrDataBuffer = (index + 1);

        /* Send out the IPv4 Packet:
         *  - This is a GTPU packet destined for the SeGW 
         *  - The GTPU Identifier is random because the rule is valid for all GTPU Control packets 
         *  - Simulating packets coming from the SeGW to the eNB so the destination is the eNB */
        retVal = Test_netSendIPv4Payload(ptrDataBuffer,                         /* Payload Data      */
                                         payloadLength,                         /* Payload Length    */
                                         (gtpuIdentifier + index),              /* GTPU Identifier   */
                                         msgType,                               /* GTPU Message Type */
                                         2152,                                  /* Destination Port  */
                                         2152,                                  /* Source Port       */
                                         &testNodeBIPAddress[0],                /* Destination IP    */
                                         &testSeGwIPAddress[0],                 /* Source IP         */
                                         0,                                     /* TOS               */
                                         &eNBMacAddress[0],                     /* Destination MAC   */
                                         &testSeGwMacAddress[0],                /* Source MAC        */
                                         1);                                    /* Loopback          */
        if (retVal < 0)
        {
            System_printf ("Error: Transmission of GTPU Control Message failed\n");
            return -1;
        }

        /* Cleanup memory for the payload buffer. */
        Memory_free (NULL, ptrDataBuffer, payloadLength);

        /* Increment the payload for the next iteration. */
        payloadLength = payloadLength + 1;
    }

    /* Sleep for some time to ensure the packets all land up */
    Task_sleep(10);

    /* All the packets have been sent out. We now need to ensure that these packets have been received
     * and placed on the correct queue. */
    if (Qmss_getQueueEntryCount(gtpuControlQueue) != numIterations)
    {
        System_printf ("Error: Receiving GTPU Control Message failed (Expected %d packet but got %d)\n", 
                        numIterations, Qmss_getQueueEntryCount(gtpuControlQueue));
        return -1;
    }

    /* Validate the packets */

    /* Original payload length started from 100 and was incremented for each iteration */
    payloadLength = 100;
    for (index = 0; index < numIterations; index++)
    {
        /* Pop off the packet from the queue. */
        ptrPkt = (Ti_Pkt*)QMSS_DESC_PTR(Qmss_queuePop(gtpuControlQueue));
        if (ptrPkt == NULL)
        {
            System_printf ("FATAL Error: QMSS malfunction\n");
            return -1;
        }

        /**************************************************************************
         * NOTE: The packet is directly getting popped off the queue and so in this
         * case the application is responsible for getting ownership of the packet.
         **************************************************************************/
        Pktlib_getOwnership (appPktlibInstanceHandle, ptrPkt);

        /* Get the data buffer length */
        Pktlib_getDataBuffer(ptrPkt, &ptrDataBuffer, &rxPayloadLength);

        /* Invalidate the cache contents. */
        appInvalidateBuffer(ptrDataBuffer, rxPayloadLength);

        if (msgType != NET_GTP_DATA)
        {
            /* Get the GTPU control message for GTPU control packets */
            if (Netfp_getPayload (ptrPkt, 1, &ptrDataBuffer, &rxPayloadLength, &sockAddr) < 0)
            {
                System_printf ("Error: GTPU get Control payload failed\n"); 
                Pktlib_freePacket(appPktlibInstanceHandle, ptrPkt);
                retVal = -1;
                continue;
            }
        }
        else
        {
            /* GTPU Id mismatch packets are GTPU data packets. The GTPU header is not available in this case. 
             * Only the GTPU payload will be available.
             */
            Pktlib_freePacket(appPktlibInstanceHandle, ptrPkt);
            continue;
        }
    
        /* Validate the packet length. The control payload contains GTPU header + GTPU payload. */
        if (rxPayloadLength != NET_GTP_HDR_SIZE + payloadLength + index)
        {
            System_printf ("Error: Incorrect GTPU control packet length Expected %d Got %d\n", 
                            NET_GTP_HDR_SIZE + payloadLength + index, rxPayloadLength);
            Pktlib_freePacket(appPktlibInstanceHandle, ptrPkt);
            continue;
        }

        /* Validate the Source port */
        if (sockAddr.sin_port != 2152)
        {
            System_printf ("Error: Incorrect source port number Expected 2152 Got %d\n", 
                            sockAddr.sin_port);
            Pktlib_freePacket(appPktlibInstanceHandle, ptrPkt);
            continue;
        }

        /* Validate the Source IP and Source port */
        if ((sockAddr.sin_addr.ver != Netfp_IPVersion_IPV4) ||
            (sockAddr.sin_addr.addr.ipv4.u.a8[0] != testSeGwIPAddress[0]) ||
            (sockAddr.sin_addr.addr.ipv4.u.a8[1] != testSeGwIPAddress[1]) ||
            (sockAddr.sin_addr.addr.ipv4.u.a8[2] != testSeGwIPAddress[2]) ||
            (sockAddr.sin_addr.addr.ipv4.u.a8[3] != testSeGwIPAddress[3]))
        {
            System_printf ("Error: Incorrect source IP address Expected %02d.%02d.%02d.%02d Got %02d.%02d.%02d.%02d\n", 
                    testSeGwIPAddress[0], testSeGwIPAddress[1],
                    testSeGwIPAddress[2], testSeGwIPAddress[3],
                    sockAddr.sin_addr.addr.ipv4.u.a8[0], sockAddr.sin_addr.addr.ipv4.u.a8[1],
                    sockAddr.sin_addr.addr.ipv4.u.a8[2], sockAddr.sin_addr.addr.ipv4.u.a8[3]);
            Pktlib_freePacket(appPktlibInstanceHandle, ptrPkt);
            continue;
        }

        /* Validate the GTPU header */
        ptrGTPUHdr = (Net_GTPUHdr *)ptrDataBuffer;

        /* Validate the message type */
        if (ptrGTPUHdr->MsgType != msgType)
        {
            System_printf ("Error: Incorrect GTPU message type Expected %d Got %d\n", 
                            msgType, ptrGTPUHdr->MsgType);
            Pktlib_freePacket(appPktlibInstanceHandle, ptrPkt);
            retVal = -1;
            continue;
        }

        /* Validate the message length */
        if (ntohs(ptrGTPUHdr->TotalLength) != payloadLength + index)
        {
            System_printf ("Error: Incorrect GTPU message length Expected %d Got %d\n", 
                            payloadLength + index, ptrGTPUHdr->TotalLength);
            Pktlib_freePacket(appPktlibInstanceHandle, ptrPkt);
            retVal = -1;
            continue;
        }

        /* Validate the GTPU Id */
        rxGtpuId = _mem4 (ptrGTPUHdr->TunnelId);
        
        if (rxGtpuId != gtpuIdentifier + index)
        {
            System_printf ("Error: Incorrect GTPU Id 0x%x Got 0x%x\n", 
                            gtpuIdentifier + index, *(ptrGTPUHdr->TunnelId));
            Pktlib_freePacket(appPktlibInstanceHandle, ptrPkt);
            retVal = -1;
            continue;
        }

        /* Cleanup the memory */
        Pktlib_freePacket(appPktlibInstanceHandle, ptrPkt);
    }

    /* Test was successful. */
    return retVal;
}

/**
 *  @b Description
 *  @n  
 *      Task which executes the IPv4 Reassembly tests. Packets are generated by
 *      the test application (using the test vectors) and pushed to the NETCP
 *
 *  @retval
 *      0   - Success
 *  @retval
 *      <0  - Error
 */
void Test_gtpuControlMsgTask(UArg arg0, UArg arg1)
{
    Netfp_FlowCfg           flowCfg;
    int32_t                 myFlowHandle;
    Netfp_GTPUControlCfg    gtpuControlCfg;
    int32_t                 errCode;
    uint8_t                 isAllocated;

    /* Setup the default test environment */
    if (Test_setupGTPUControlMsgEnv() < 0)
        return;

    /* Initialize the flow configuration. */
    memset((void *)&flowCfg, 0, sizeof(Netfp_FlowCfg));

    /* Populate the flow configuration: Use the NETFP Data Receive Heap for this purpose. */
    flowCfg.numHeaps      = 1;
    flowCfg.heapHandle[0] = mtuReceiveHeap;
    flowCfg.sopOffset     = 0;
    strcpy (flowCfg.name, "GTPU-Control-Flow");

    /* Create a test flow which will be used in the unit tests. */
    myFlowHandle = Netfp_createFlow (netfpClientHandle, &flowCfg, &errCode);
    if (myFlowHandle < 0)
    {
        System_printf ("Error: Fast Path Flow Creation Failed [Error code %d]\n", errCode);
        return;
    }

    /* Initialize the GTPU control configuration: */
    memset ((void *)&gtpuControlCfg, 0, sizeof(Netfp_GTPUControlCfg));

    /* Populate the GTPU control configuration: */
    gtpuControlCfg.gtpuMsgFlowId           = myFlowHandle;
    gtpuControlCfg.gtpuPingReqQueueHnd     = Qmss_queueOpen(Qmss_QueueType_GENERAL_PURPOSE_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated);
    gtpuControlCfg.gtpuPingRespQueueHnd    = Qmss_queueOpen(Qmss_QueueType_GENERAL_PURPOSE_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated);
    gtpuControlCfg.gtpuErrorIndQueueHnd    = Qmss_queueOpen(Qmss_QueueType_GENERAL_PURPOSE_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated);
    gtpuControlCfg.gtpuHdrNotifyQueueHnd   = Qmss_queueOpen(Qmss_QueueType_GENERAL_PURPOSE_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated);
    gtpuControlCfg.gtpuEndMarkerQueueHnd   = Qmss_queueOpen(Qmss_QueueType_GENERAL_PURPOSE_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated);
    gtpuControlCfg.gtpuErrorQueueHnd       = Qmss_queueOpen(Qmss_QueueType_GENERAL_PURPOSE_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated);
    gtpuControlCfg.gtpuIdMatchFailQueueHnd = Qmss_queueOpen(Qmss_QueueType_GENERAL_PURPOSE_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated);

    /* Configure the GTPU Control message queues */    
    if (Netfp_configureGTPUControlMessage (netfpClientHandle, &gtpuControlCfg, &errCode) < 0)
    {
        System_printf ("Error: GTPU Control message configuration failed [Error code %d]\n", errCode);
        return;
    }
    System_printf ("Debug: GTPU Control message configuration successful\n");

    /* GTPU Control Message: Ping Request Test */ 
    if (Test_GTPUControlMsg(10, NET_GTP_PING_REQUEST, &gtpuControlCfg) < 0)
    {
        System_printf ("Error: GTPU Ping Request Test Failed\n");
        return;
    }
    System_printf ("Debug: GTPU Ping Request Test passed\n");

    /* GTPU Control Message: Ping Response Test */
    if (Test_GTPUControlMsg(20, NET_GTP_PING_RESPONSE, &gtpuControlCfg) < 0)
    {
        System_printf ("Error: GTPU Ping Response Test Failed\n");
        return;
    }
    System_printf ("Debug: GTPU Ping Response Test passed\n");

    /* GTPU Control Message: Error Indication Test */
    if (Test_GTPUControlMsg(5, NET_GTP_ERROR_INDICATION, &gtpuControlCfg) < 0)
    {
        System_printf ("Error: GTPU Error Indication Test Failed\n");
        return;
    }
    System_printf ("Debug: GTPU Error indication test passed\n");

    /* GTPU Control Message: Header Notify Test */
    if (Test_GTPUControlMsg(30, NET_GTP_HEADER_NOTIFY, &gtpuControlCfg) < 0)
    {
        System_printf ("Error: GTPU Header Notify Test Failed\n");
        return;
    }
    System_printf ("Debug: GTPU Header notify test passed\n");

    /* GTPU Control Message: Header Notify Test */
    if (Test_GTPUControlMsg(10, NET_GTP_END_MARKER, &gtpuControlCfg) < 0)
    {
        System_printf ("Error: GTPU End Marker Test Failed\n");
        return;
    }
    System_printf ("Debug: GTPU End marker test passed\n");

    /* GTPU Control Message: Parsing Error Test */ 
    if (Test_GTPUControlMsg(26, NET_GTP_PARSING_ERROR, &gtpuControlCfg) < 0)
    {
        System_printf ("Error: GTPU Parsing Error Test Failed\n");
        return;
    }
    System_printf ("Debug: Parsing error test passed\n");

    /* GTPU Control Message: Id Mismatch Test 
     *  - To test this we send a valid GTP Data packet but we have not
     *    opened a socket for that specific GTPU Tunnel Identifier. */ 
    if (Test_GTPUControlMsg(30, NET_GTP_DATA, &gtpuControlCfg) < 0)
    {
        System_printf ("Error: GTPU Id Mismatch Test Failed\n");
        return;
    }
    System_printf ("Debug: All GTPU Control Tests Passed\n");
}
