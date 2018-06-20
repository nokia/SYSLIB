/*
 *   @file  hook.c
 *
 *   @brief   
 *      The file tests the hook functionality
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
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <ti/sysbios/family/c64p/Hwi.h>

/* IPC Include Files for Shared Memory Allocation. */
#include <ti/ipc/Ipc.h>
#include <ti/ipc/SharedRegion.h>

/* PDK Include Files. */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/platform/platform.h>

/* CSL Include Files. */
#include <ti/csl/csl_chip.h>
#include <ti/csl/csl_cacheAux.h>

/* SYSLIB Include Files.  */
#include <ti/runtime/msgcom/msgcom.h>
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/netfp/netfp.h>
#include <ti/drv/rm/rm.h>

/* Benchmarking Include Files */
#include "benchmark.h"

/**********************************************************************
 *********************** Unit Test Local Definitions ******************
 **********************************************************************/

/* Base UDP port number for the test */
#define TEST_SOCKET_IPV4_BASE_UDP_PORT          10000

/**********************************************************************
 ************************ Unit Test Global variables ******************
 **********************************************************************/

/* Hook Operation Status: */
Netfp_HookReturn    gHookStatus;

/* Global Variables which keep track of the NETFP functions being benchmarked */ 
BENCHMARK_INIT(Netfp_getPayloadIPv4)
BENCHMARK_INIT(Netfp_sendIPv4)

/**********************************************************************
 ************************ Extern Declarations *************************
 **********************************************************************/

/* NETFP Heap for data receive and transmit. */
extern Pktlib_HeapHandle    netfp10KReceiveHeap;

/* NETFP Client Handle: */
extern Netfp_ClientHandle   netfpClientHandle;

/* Global MSGCOM Instance handle. */
extern Msgcom_InstHandle    appMsgcomInstanceHandle;

/* PKTLIB Instance handle */
extern Pktlib_InstHandle    appPktlibInstanceHandle;

/* Global Database handle: */
extern Name_DBHandle        globalNameDatabaseHandle;

/* Global application requested resource configuration */
extern Resmgr_ResourceCfg   appResourceConfig;

/* Application Cache Management API */
extern void appInvalidateBuffer(void* ptr, uint32_t size);
extern void appWritebackBuffer(void* ptr, uint32_t size);

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
static void Test_notifySocketFunction(Netfp_Reason reason, Netfp_SockHandle sockHandle)
{
    System_printf ("Debug: Notify function invoked for socket %p [Reason: '%s']\n", sockHandle, Netfp_getReasonString(reason)); 
}

/**
 *  @b Description
 *  @n  
 *      The function is called to process the received packet.
 *
 *  @param[in]  sockHandle
 *      Socket Handle on which the packet was received
 *  @param[in]  ptrRxPkt
 *      Pointer to the received packet.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Test_socketProcessPacket (Netfp_SockHandle sockHandle, Ti_Pkt* ptrRxPkt)
{
    Ti_Pkt*             ptrTxPkt;
    uint8_t*            ptrRxDataBuffer;
    uint8_t*            ptrTxDataBuffer;
    uint32_t            rxDataBufferLen;
    uint32_t            txDataBufferLen;
    int32_t             retVal;
    int32_t             errCode;
    Netfp_PeerSockAddr  peerSockAddress;

    /* Get the data buffer & length */
    Pktlib_getDataBuffer(ptrRxPkt, &ptrRxDataBuffer, &rxDataBufferLen);

    /* Invalidate the data buffer. */
    appInvalidateBuffer(ptrRxDataBuffer, rxDataBufferLen);

    /* Use the receive payload to get the data buffer: Take timestamps also. */
    BENCHMARK_START(Netfp_getPayloadIPv4);
    retVal = Netfp_getPayload (ptrRxPkt, 0, &ptrRxDataBuffer, &rxDataBufferLen, &peerSockAddress);
    BENCHMARK_END(Netfp_getPayloadIPv4);
    if (retVal < 0)
    {
        System_printf ("Error: Unable to get the payload for packet 0x%p\n", ptrRxPkt);
        return -1;
    }

    /* Update & log the information. */
    BENCHMARK_UPDATE(Netfp_getPayloadIPv4);

    /* Send out the packet. */ 
    txDataBufferLen = rxDataBufferLen;

    /* Allocate another packet from the NETFP 10K Heap */
    ptrTxPkt = Pktlib_allocPacket(appPktlibInstanceHandle, netfp10KReceiveHeap, txDataBufferLen);
    if (ptrTxPkt == NULL)
    {
        Pktlib_freePacket(appPktlibInstanceHandle, ptrRxPkt);
        return -1;
    }

    /* Get the data buffer & length */
    Pktlib_getDataBuffer(ptrTxPkt, &ptrTxDataBuffer, &txDataBufferLen);

    /* Copy the data buffer */
    memcpy (ptrTxDataBuffer, ptrRxDataBuffer, txDataBufferLen);

    /* Writeback the data buffer */
    appWritebackBuffer(ptrTxDataBuffer, txDataBufferLen);

    /* Send out the socket on the appropriate socket: Take timestamps also. */
    BENCHMARK_START(Netfp_sendIPv4);
    retVal = Netfp_send (sockHandle, ptrTxPkt, 0, &errCode);
    BENCHMARK_END(Netfp_sendIPv4);
    if (retVal < 0)
    {
        System_printf ("Error: Unable to send the packet Error Code: %d\n", errCode);
        return -1;
    }

    /* Update & log the information. */
    BENCHMARK_UPDATE(Netfp_sendIPv4);

    /* Clean up the received packet. */
    Pktlib_freePacket(appPktlibInstanceHandle, ptrRxPkt);
    return 0;
}

/**
 *  @b Description
 *  @n  
 *      Utility function to map the Hook Return status to a string
 *
 *  @param[in]  hookStatus
 *      Hook return status
 *
 *  @retval
 *      Mapped String
 */
static char* Test_displayHookStatus(Netfp_HookReturn hookStatus)
{
    switch (hookStatus)
    {
        case Netfp_HookReturn_ACCEPT:
            return "Hook-Accept";
        case Netfp_HookReturn_DROP:
            return "Hook-Drop";
        case Netfp_HookReturn_STOLEN:
            return "Hook-Stolen";
        default:
            return "Hook-UNEXPECTED";            
    }
}

/**
 *  @b Description
 *  @n  
 *      Outer IP Reassembly hook: This is registered to be invoked on the reception
 *      of a fragment on the Outer IP channel.
 *
 *  @param[in]  hook
 *      Hook identifier
 *  @param[in]  sockHandle
 *      Socket Handle. This is a reassembly hook so this will always be NULL
 *  @param[in]  ptrPkt
 *      Pointer to the IP fragment which was received
 *  @param[in]  arg
 *      Registered argument
 *
 *  @retval
 *      Hook Return
 */
static Netfp_HookReturn Test_outerIPReassemblyHook
(
    Netfp_Hook          hook,
    Netfp_SockHandle    sockHandle,
    Ti_Pkt*             ptrPkt,
    uint32_t            arg
)
{
    /* Sanity Test: Ensure that the arguments are as expected */
    if (hook != Netfp_Hook_PRE_OUTER_REASSEMBLY)
        System_printf ("Error: Unexpected hook %d detected\n", hook);

    /* Sanity Test: Reassembly does not have a socket handle */
    if (sockHandle != NULL)
        System_printf ("Error: Unexpected socket handle %p detected\n", sockHandle);

    /* Sanity Test: We should always receive a packet */
    if (ptrPkt == NULL)
        System_printf ("Error: Hook %d invoked with a NULL packet\n", hook);

    /* Sanity Test: Validate the hook argument */
    if (arg != 0xdead)
        System_printf ("Error: Hook %d invoked with invalid argument %x\n", hook, arg);

    /* Debug Message: */
    System_printf ("Debug: Packet %p PRE_OUTER_REASSEMBLY Hook will be %s\n", ptrPkt, Test_displayHookStatus(gHookStatus));
    return gHookStatus;
}

/**
 *  @b Description
 *  @n  
 *      Reassembly hook: This is registered to be invoked on the reassembly of all the
 *      fragments and before the packet is passed back to the NETCP for further 
 *      classification
 *
 *  @param[in]  hook
 *      Hook identifier
 *  @param[in]  sockHandle
 *      Socket Handle. This is a reassembly hook so this will always be NULL
 *  @param[in]  ptrPkt
 *      Pointer to the reassembled packet
 *  @param[in]  arg
 *      Registered argument
 *
 *  @retval
 *      Hook Return
 */
static Netfp_HookReturn Test_postReassemblyHook
(
    Netfp_Hook          hook,
    Netfp_SockHandle    sockHandle,
    Ti_Pkt*             ptrPkt,
    uint32_t            arg
)
{
    pasahoLongInfo_t*       pInfo;
    uint32_t                infoLen;

    /* Sanity Test: Ensure that the arguments are as expected */
    if (hook != Netfp_Hook_POST_OUTER_REASSEMBLY)
        System_printf ("Error: Unexpected hook %d detected\n", hook);

    /* Sanity Test: Reassembly does not have a socket handle */
    if (sockHandle != NULL)
        System_printf ("Error: Unexpected socket handle %p detected\n", sockHandle);

    /* Sanity Test: We should always receive a packet */
    if (ptrPkt == NULL)
        System_printf ("Error: Hook %d invoked with a NULL packet\n", hook);

    /* Sanity Test: Validate the hook argument */
    if (arg != 0x1234)
        System_printf ("Error: Hook %d invoked with invalid argument %x\n", hook, arg);

    /* Get the protocol specific data from the packet. */
    Cppi_getPSData (Cppi_DescType_HOST, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc *)ptrPkt, (uint8_t **)&pInfo, &infoLen);
    
    /* Debug Message: */
    System_printf ("Debug: Packet %p had %d fragments POST_OUTER_REASSEMBLY Hook will be %s\n", ptrPkt, 
                    PASAHO_LINFO_READ_FRANCNT(pInfo), Test_displayHookStatus(gHookStatus));
    return gHookStatus;
}

/**
 *  @b Description
 *  @n  
 *      Socket Post Routing hook: This is registered to be invoked on the transmission
 *      of a packet once all the networking headers have been attached and just before 
 *      the packet is passed to the NETCP.
 *
 *  @param[in]  hook
 *      Hook identifier
 *  @param[in]  sockHandle
 *      Socket Handle on which the packet is being sent
 *  @param[in]  ptrPkt
 *      Pointer to the packet
 *  @param[in]  arg
 *      Registered argument
 *
 *  @retval
 *      Hook Return
 */
static Netfp_HookReturn Test_socketPostRoutingHook
(
    Netfp_Hook          hook,
    Netfp_SockHandle    sockHandle,
    Ti_Pkt*             ptrPkt,
    uint32_t            arg
)
{
    /* Sanity Test: Ensure that the arguments are as expected */
    if (hook != Netfp_Hook_POST_ROUTING)
        System_printf ("Error: Unexpected hook %d detected\n", hook);

    /* Sanity Test: Post routing hook should always have a socket handle */
    if (sockHandle == NULL)
        System_printf ("Error: Unexpected socket handle %p detected\n", sockHandle);

    /* Sanity Test: We should always receive a packet */
    if (ptrPkt == NULL)
        System_printf ("Error: Hook %d invoked with a NULL packet\n", hook);

    /* Sanity Test: Validate the hook argument */
    if (arg != 0xfeed)
        System_printf ("Error: Hook %d invoked with invalid argument %x\n", hook, arg);

    if(gHookStatus == Netfp_HookReturn_STOLEN)
        Pktlib_freePacket(appPktlibInstanceHandle, ptrPkt);

    /* Debug Message: */
    System_printf ("Debug: Packet %p POST_ROUTING Hook will be %s\n", ptrPkt, Test_displayHookStatus(gHookStatus));
    return gHookStatus;
}

/**
 *  @b Description
 *  @n  
 *      Entry point to the test task which executes the unit tests.
 *
 *  @retval
 *      Not Applicable.
 */
void Test_hookTask(UArg arg0, UArg arg1)
{
    Netfp_InboundFPHandle   fpIngressHandle;
    Netfp_OutboundFPHandle  fpEgressHandle;
    Netfp_SockHandle        sockHandle;
    Netfp_SockAddr          sockAddress;
    Msgcom_ChannelCfg       chConfig;
    MsgCom_ChHandle         udpChannelHandle;
    Netfp_FlowCfg           flowCfg;
    int32_t                 rxFlowId;
    int32_t                 errCode;
    Netfp_HookCfg           hookCfg;
    Name_ResourceCfg        namedResCfg; 
    Ti_Pkt*                 ptrRxPkt;
    char                    channelName[MSG_COM_MAX_CHANNEL_LEN];

    /* Get the ingress fast path handle: The fast paths are shared across different clients. */
    while (1)
    {
        fpIngressHandle = Netfp_findInboundFastPath (netfpClientHandle, "Ingress-IPv4-FastPath", &errCode);
        if (fpIngressHandle != NULL)
            break;
        Task_sleep(10);
    }
        
    /* Get the egress fast path handle: The fast paths are shared across different clients. */
    while (1)
    {
        fpEgressHandle = Netfp_findOutboundFastPath (netfpClientHandle, "Egress-IPv4-FastPath", &errCode);
        if (fpEgressHandle != NULL)
            break;
        Task_sleep(10);
    }

    /* Populate the flow configuration: Use the NETFP Data Receive Heap for this purpose.
     * Flows are specific to a NETFP Client. */
    strcpy (flowCfg.name, "SocketTestFlow");
    flowCfg.numHeaps      = 1;
    flowCfg.heapHandle[0] = netfp10KReceiveHeap;
    flowCfg.sopOffset     = 0;

    /* Create the flow. */
    rxFlowId = Netfp_createFlow (netfpClientHandle, &flowCfg, &errCode);
    if (rxFlowId < 0)
    {
        System_printf ("Error: Unable to create the flow [Error code %d]\n", errCode);
        return;
    }
    System_printf ("Debug: Flow %d has been created successfully\n", rxFlowId);

    /* Create a unique channel name */
    sprintf (channelName, "Socket-IPv4-Channel-%d", DNUM);

    /* Initialize the channel configuration. */
    memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* Populate the channel configuration. */
    chConfig.mode                                                   = Msgcom_ChannelMode_BLOCKING;
    chConfig.appCallBack                                            = NULL;
    chConfig.msgcomInstHandle                                       = appMsgcomInstanceHandle;
    chConfig.u.queueCfg.interruptMode                               = Msgcom_QueueInterruptMode_DIRECT_INTERRUPT;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.queuePendQueue  = appResourceConfig.qPendResponse[0].queue;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.cpIntcId        = appResourceConfig.qPendResponse[0].cpIntcId;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.systemInterrupt = appResourceConfig.qPendResponse[0].systemInterrupt;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.hostInterrupt   = appResourceConfig.qPendResponse[0].hostInterrupt;

    /* Create the Message communicator channel. */
    udpChannelHandle = Msgcom_create (channelName, Msgcom_ChannelType_QUEUE, &chConfig, &errCode);
    if (udpChannelHandle == NULL)
    {
        System_printf ("Error: Unable to open the UDP Channel Error : %d\n", errCode);
        return;
    }

    /* Create a socket */
    sockHandle = Netfp_socket (netfpClientHandle, Netfp_SockFamily_AF_INET, &errCode);
    if (sockHandle == NULL)
    {
        System_printf ("Error: NETFP Socket Creation Failed [Error Code %d]\n", errCode);
        return;
    }
    
    /* Populate the binding information */
    memset ((void*)&sockAddress, 0, sizeof(Netfp_SockAddr));
	sockAddress.sin_family              = Netfp_SockFamily_AF_INET;
	sockAddress.sin_port                = (TEST_SOCKET_IPV4_BASE_UDP_PORT + DNUM);
    sockAddress.op.bind.inboundFPHandle = fpIngressHandle;
    sockAddress.op.bind.flowId          = rxFlowId;
	sockAddress.op.bind.queueHandle     = Msgcom_getInternalMsgQueueInfo(udpChannelHandle);
    sockAddress.op.bind.notifyFunction  = Test_notifySocketFunction;
    
    /* Bind the socket. */
    if (Netfp_bind (sockHandle, &sockAddress, &errCode) < 0)
    {
        System_printf ("Error: NETFP Bind Failed [Error Code %d]\n", errCode);
        return;
    }
    System_printf ("Debug: Socket has been bound successfully\n");
    
    /* Populate the connect information. */
	sockAddress.sin_family                  = Netfp_SockFamily_AF_INET;
    sockAddress.sin_port                    = (TEST_SOCKET_IPV4_BASE_UDP_PORT + DNUM);
	sockAddress.op.connect.outboundFPHandle = fpEgressHandle;
    
    /* Connect the socket */
    if (Netfp_connect(sockHandle, &sockAddress, &errCode) < 0)
    {
        System_printf ("Error: NETFP Connect Failed [Error Code %d]\n", errCode);
        return;
    }
    System_printf ("Debug: Socket has been connected successfully\n");

    /************************************************************************************
     * Reassembly Hooks can only be instantiated on the core which handles reassembly
     ************************************************************************************/
    if (DNUM == 1)
    {
        /* Core1 is responsible for reassembly: Initialize the hook configuration: */
        memset ((void*)&hookCfg, 0, sizeof(Netfp_HookCfg));

        /* Populate the hook configuration: */
        hookCfg.hook        = Netfp_Hook_PRE_OUTER_REASSEMBLY;
        hookCfg.sockHandle  = NULL;
        hookCfg.hookFxn     = Test_outerIPReassemblyHook;
        hookCfg.hookArg     = 0xdead;

        /* Register the hook: */
        if (Netfp_registerHook (netfpClientHandle, &hookCfg, &errCode) < 0)
        {
            System_printf ("Error: PRE_OUTER_REASSEMBLY hook registeration failed [Error code %d]\n", errCode);
            return;
        }
        System_printf ("Debug: PRE_OUTER_REASSEMBLY hook registered successfully\n");

        /* Populate the hook configuration: */
        hookCfg.hook        = Netfp_Hook_PRE_OUTER_REASSEMBLY;
        hookCfg.sockHandle  = NULL;
        hookCfg.hookFxn     = NULL;
        hookCfg.hookArg     = 0;

        /* Unregister the hook: */
        if (Netfp_unregisterHook (netfpClientHandle, Netfp_Hook_PRE_OUTER_REASSEMBLY, NULL, &errCode) < 0)
        {
            System_printf ("Error: PRE_OUTER_REASSEMBLY hook unregisteration failed [Error code %d]\n", errCode);
            return;
        }
        System_printf ("Debug: PRE_OUTER_REASSEMBLY hook unregistered successfully\n");

        /* Good: We can now register the hook again. We need to test our hook operations. */
        memset ((void*)&hookCfg, 0, sizeof(Netfp_HookCfg));

        /* Populate the hook configuration: */
        hookCfg.hook        = Netfp_Hook_PRE_OUTER_REASSEMBLY;
        hookCfg.sockHandle  = NULL;
        hookCfg.hookFxn     = Test_outerIPReassemblyHook;
        hookCfg.hookArg     = 0xdead;

        /* Register the hook: */
        if (Netfp_registerHook (netfpClientHandle, &hookCfg, &errCode) < 0)
        {
            System_printf ("Error: PRE_OUTER_REASSEMBLY hook registeration failed [Error code %d]\n", errCode);
            return;
        }
        System_printf ("Debug: PRE_OUTER_REASSEMBLY hook registered successfully again\n");

        memset ((void*)&hookCfg, 0, sizeof(Netfp_HookCfg));

        /* Populate the hook configuration: */
        hookCfg.hook        = Netfp_Hook_POST_OUTER_REASSEMBLY;
        hookCfg.sockHandle  = NULL;
        hookCfg.hookFxn     = Test_postReassemblyHook;
        hookCfg.hookArg     = 0x1234;

        /* Register the hook: */
        if (Netfp_registerHook (netfpClientHandle, &hookCfg, &errCode) < 0)
        {
            System_printf ("Error: POST_OUTER_REASSEMBLY hook registeration failed [Error code %d]\n", errCode);
            return;
        }
        System_printf ("Debug: POST_OUTER_REASSEMBLY hook registered successfully again\n");
    }
    else
    {
        /* Initialize the hook configuration: */
        memset ((void*)&hookCfg, 0, sizeof(Netfp_HookCfg));

        /* Populate the hook configuration: */
        hookCfg.hook        = Netfp_Hook_PRE_OUTER_REASSEMBLY;
        hookCfg.sockHandle  = NULL;
        hookCfg.hookFxn     = Test_outerIPReassemblyHook;
        hookCfg.hookArg     = 0xdead;

        /* Register the hook: This should fail because the reassembly services are not operational */
        if (Netfp_registerHook (netfpClientHandle, &hookCfg, &errCode) == 0)
        {
            System_printf ("Error: PreOuter Reassembly hook registeration was successful\n");
            return;
        }
        if (errCode != NETFP_ENOTREADY)
        {
            System_printf ("Error: PreOuter Reassembly hook registeration failed with Invalid error code %d\n", errCode);
            return;
        }
        System_printf ("Debug: PRE_OUTER_REASSEMBLY hook could not be registered as expected\n");
    }

    /* Populate the hook configuration: */
    hookCfg.hook        = Netfp_Hook_POST_ROUTING;
    hookCfg.sockHandle  = sockHandle;
    hookCfg.hookFxn     = Test_socketPostRoutingHook;
    hookCfg.hookArg     = 0xfeed;

    /* Register the socket POST_ROUTING hook: */
    if (Netfp_registerHook (netfpClientHandle, &hookCfg, &errCode) < 0)
    {
        System_printf ("Error: Post Routing hook registeration failed [Error code %d]\n", errCode);
        return;
    }
    System_printf ("Debug: POST_ROUTING hook registered successfully\n");

    /* The CLI is generated only on DSP Core0 */
    if (DNUM == 0)
    {
        System_printf ("*********************************************************\n");
        System_printf ("Hook Operation [1-Accept, 2-Drop, 3-Stolen]:");
	    scanf ("%d", (int32_t*)&gHookStatus);
        System_printf ("*********************************************************\n");

        /* Create a named resource and indicate to the other core the test which is being executed. */
        memset ((void *)&namedResCfg, 0, sizeof(Name_ResourceCfg));
        namedResCfg.handle1  = (uint32_t)gHookStatus;
        strcpy(namedResCfg.name, "HookOperation");
        if (Name_createResource(globalNameDatabaseHandle, Name_ResourceBucket_USER_DEF1, &namedResCfg, &errCode) < 0)
            System_printf ("Error: Creating Named resource failed with error code %d\n", errCode);
    }
    else
    {
        /* Loop around till we get a selection: */
        while (1)
        {
            if (Name_findResource(globalNameDatabaseHandle, Name_ResourceBucket_USER_DEF1, "HookOperation", &namedResCfg, &errCode) == 0)
                break;
            Task_sleep(100);
        }

        /* Get the Hook Status from the named resource object. */
        gHookStatus = (Netfp_HookReturn)namedResCfg.handle1;
    }

    /* Debug Message: */
    System_printf ("-------------------------------------------------------------\n");
    System_printf ("IPv4 UDP Port         = %d\n",   sockAddress.sin_port);
    System_printf ("-------------------------------------------------------------\n");

    /**********************************************************************************
     * Stress Test:
     *  This is the main loop where the task waits for data to arrive from the 
     *  sockets.
     *********************************************************************************/
    while (1)
    {
        /* Extract the packet from the MSGCOM channel. */ 
        Msgcom_getMessage (udpChannelHandle, (MsgCom_Buffer**)&ptrRxPkt);

        /* Proceed only if there is a message pending. */
        if (ptrRxPkt != NULL)
            Test_socketProcessPacket(sockHandle, ptrRxPkt);
    }
}

