/**
 *   @file  l2_dp.c
 *
 *   @brief
 *      L2 UserPlane setup and processing code.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2013-2104 Texas Instruments, Inc.
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
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <getopt.h>

/* MCSDK Include files. */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/runtime/hplib/hplib.h>

/* SYSLIB Include Files */
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/netfp/netfp.h>
#include <ti/runtime/uintc/uintc.h>
#include <ti/runtime/name/name.h>
#include <ti/runtime/name/name_db.h>
#include <ti/runtime/name/name_proxyClient.h>
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/domain/domain.h>
#include <ti/runtime/dat/dat.h>

/* Application Include Files */
#include "l2_lte.h"

/**********************************************************************
 ************************* Global Declarations ************************
 **********************************************************************/
/* eNodeB IPv4 Address. Used for GTPU Fast path creation */
uint8_t eNodeBIPAddress[4] = {192, 168, 1, 2};

/* eNodeB MAC Address. Used only when NetFP Proxy is not used */
uint8_t eNodeBMACAddress[6] = {0x08, 0x00, 0x28, 0x32, 0x99, 0xa3};

/* PDN Gw IPv4 Address. Used for GTPU Fast path creation */
uint8_t pdnGwIPAddress[4] = {192, 168, 1, 1};

/* PDN Gw MAC Address. Used only when NetFP Proxy is not used */
uint8_t pdnGwMACAddress[6] = {0x08, 0x00, 0x27, 0xb2, 0x0c, 0x65};

/* Reassembly timeout in milliseconds */
#define REASSEMBLY_TIMER_PERIOD_MSEC            500

/* NetFP FIFOs used for various events */
#define FIFO_GTPU_RX        "NETFP_FIFO_0"
#define FIFO_FRAG_RX        "NETFP_FIFO_1"
#define FIFO_REASS_TIMER    "NETFP_FIFO_2"

/* Use accumulated channels: */
#define USE_ACC_INT

/* LTE Stack Domain MCB */
extern AppLTEStackDomainMCB    myAppDomain;

/**********************************************************************
 ********************** L2 UserPlane Functions ************************
 **********************************************************************/

/**
 *  @b Description
 *  @n
 *      Heap Data Buffer Allocator which is instantiated into the heap
 *      interface table.
 *
 *  @param[in]  size
 *      Size of the memory which is to be allocated.
 *  @param[in]  arg
 *      FAPI Module specific argument
 *
 *  @retval
 *      Success     -   Pointer to the allocated block of memory.
 *  @retval
 *      Error       -   NULL
 */
static uint8_t* L2_MemoryMalloc(uint32_t size, uint32_t arg)
{
    /* Allocate memory */
    return hplib_vmMemAlloc (size, 0, 0);
}

/**
 *  @b Description
 *  @n
 *      Heap Data Buffer Cleanup which is instantiated into the heap
 *      interface table.
 *
 *  @param[in]  ptr
 *      Pointer to the memory which is to be cleaned up.
 *  @param[in]  size
 *      Size of the memory which is to be cleaned up.
 *  @param[in]  arg
 *      FAPI Module specific argument
 *
 *  @retval
 *      Not Applicable.
 */
static void L2_MemoryFree(uint8_t* ptr, uint32_t size, uint32_t arg)
{
    /* TODO: Currently HPLIB does not support free */
}

/**
 *  @b Description
 *  @n
 *      Msgcom buffer cleanup handler.
 *
 *  @param[in]  pktlibInstHandle
 *      Pktlib instance handle
 *  @param[in]  chHandle
 *      Msgcom channel handle
 *  @param[in]  msgBuffer
 *      Descriptor to be freed up
 *
 *  @retval
 *      Not Applicable.
 */
static void L2_MsgcomFree(Pktlib_InstHandle pktlibInstHandle, MsgCom_ChHandle chHandle, MsgCom_Buffer* msgBuffer)
{
    /* Free the packet */
    Pktlib_freePacket(pktlibInstHandle, (Ti_Pkt*)msgBuffer);
}

/**
 *  @b Description
 *  @n
 *      L2 UserPlane Management Task. Handles the following:
 *          - GTPU Rx (Event Id 0)
 *          - Fragment Rx (Event Id 1)
 *          - Reassembly Timeout (Event Id 2)
 *
 *  @param[in]  arg0
 *      LTE stack domain configuration
 *
 *  @retval
 *      Not Applicable.
 */
void* L2_UserPlaneMgmtTask (void* arg0)
{
    Ti_Pkt*                 ptrRxPkt;
	uint8_t*                ptrRxDataBuffer;
    uint32_t                rxDataBufferLen;
    int32_t                 errCode;
    AppLTEStackDomainMCB*   ptrLTEStackDomain;

    /* Get the domain info handle */
    ptrLTEStackDomain = (AppLTEStackDomainMCB*)arg0;

    while (1)
    {
        /* Block on the GTP-U Rx channel waiting for a packet */
        Msgcom_getMessage (ptrLTEStackDomain->gtpuChannelHandle, (MsgCom_Buffer**)&ptrRxPkt);

        /* Received a packet. Process it. */
        if (ptrRxPkt == NULL)
            break;

        /* Increment the GTPU Rx counter */
        ptrLTEStackDomain->gtpuRxPktCtr++;

        /* Get the GTPU payload from the packet received */
        Pktlib_getDataBuffer (ptrRxPkt, &ptrRxDataBuffer, &rxDataBufferLen);

        /* Echo back the packet to sender */
        if (Netfp_send (ptrLTEStackDomain->l2DRBChannel, ptrRxPkt, 0x0, &errCode) < 0)
        {
            /* Error looping back Rx packet. Increment error stats */
            printf ("Error: Failed sending GTPU packet of size %d bytes [Error code %d]\n",
                    rxDataBufferLen, errCode);
            ptrLTEStackDomain->gtpuTxErrorPktCtr++;
            return NULL;
        }

        /* Successfully echoed back Rx packet. Increment success stats */
        ptrLTEStackDomain->gtpuTxPktCtr++;
    }
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      Sets up SYSLIB environment for L2 stack's
 *      IPv4 user plane.
 *
 *  @param[in]  ptrLTEStackDomain
 *      LTE stack domain configuration.
 *  @param[in]  ptrL2ResourceCfg
 *      Pointer to the L2 resource configuration
 *
 *  @retval
 *      <0      -   Error setting up L2 IPv4 User plane
 *      0       -   Succesfully setup L2 IPv4 user plane.
 *                  L2 User plane operational.
 */
int32_t L2_UserPlane_SetupIPv4Env
(
    AppLTEStackDomainMCB*       ptrLTEStackDomain,
    Resmgr_ResourceCfg*         ptrL2ResourceCfg
)
{
    Name_DBHandle               nameDBHandle;
    Pktlib_InstHandle           pktlibHandle;
    Msgcom_InstHandle           msgComInstHandle;
    Netfp_ClientHandle          netfpClientHandle;
    int32_t                     errCode, i;
    Name_ResourceCfg            nrConfig;
    uint32_t                    ingressSPID, egressSPID;
    Netfp_InboundFPCfg          inboundFPCfg;
    Netfp_OutboundFPCfg         outboundFPCfg;
    Pktlib_HeapCfg              heapConfig;
    Netfp_FlowCfg               flowConfig;
    Msgcom_ChannelCfg           chConfig;
    Netfp_UserCfg               userConfig;
    uint8_t                     encryptionKey[16];
    uint8_t                     integrityKey[16];
    uint32_t                    index;
    Netfp_LTEChannelBindCfg     bindConfig;
    Netfp_LTEChannelConnectCfg  connectConfig;
    uint32_t                    ueId = 0x12;
    uint32_t                    rbId = 0x03;
    uint32_t                    gtpuId = 0xdead12;
    pthread_attr_t              threadAttr;
    struct sched_param          schedParam;

    /* Get various SYSLIB handles required for UserPlane operation */
    nameDBHandle = Domain_getDatabaseHandle (ptrLTEStackDomain->syslibHandle);
    if (nameDBHandle == NULL)
    {
        printf ("Error: Can't connect to Name DB for appId: %d using NR Instance Id %d\n",
                        ptrLTEStackDomain->appId, ptrLTEStackDomain->rootSyslibCfg.nrInstanceId);
        return -1;
    }
    pktlibHandle = Domain_getPktlibInstanceHandle(ptrLTEStackDomain->syslibHandle);
    if (pktlibHandle == NULL)
    {
        printf ("Error: Can't get the PKTLIB handle to use for appId: %d\n",
                        ptrLTEStackDomain->appId);
        return -1;
    }
    msgComInstHandle = Domain_getMsgcomInstanceHandle(ptrLTEStackDomain->syslibHandle);
    if (msgComInstHandle == NULL)
    {
        printf ("Error: Can't get the Msgcom handle to use for appId: %d\n",
                        ptrLTEStackDomain->appId);
        return -1;
    }
    netfpClientHandle = Domain_getNetfpClientInstanceHandle(ptrLTEStackDomain->syslibHandle);
    if (netfpClientHandle == NULL)
    {
        printf ("Error: Can't get the NetFP client handle to use for appId: %d\n",
                        ptrLTEStackDomain->appId);
        return -1;
    }

    /* Find the ingress policy to use for L2 User Plane (UP) */
    while (1)
    {
        if (Name_findResource (nameDBHandle, Name_ResourceBucket_USER_DEF1,
                               "Ingress_SPID_IPv4_UP", &nrConfig, &errCode) == 0)
        {
            /* Found the Ingress IPv4 Policy to use for UP */
            ingressSPID =    (uint32_t)nrConfig.handle1;
            break;
        }
        else
        {
            /* Error finding the policy. Check the error */
            if (errCode == NAME_ENOTFOUND)
            {
                /* Policy not found. Wait for sometime and retry */
                usleep (1);
                continue;
            }
            else
            {
                /* Error getting the policy to use for UP. Cant proceed */
                return -1;
            }
        }
    }
    printf ("Debug: Using Policy Id %d for L2 UserPlane Ingress\n", ingressSPID);

    /* Find the egress policy to use for L2 UP */
    while (1)
    {
        if (Name_findResource (nameDBHandle, Name_ResourceBucket_USER_DEF1,
                               "Egress_SPID_IPv4_UP", &nrConfig, &errCode) == 0)
        {
            /* Found the Egress IPv4 Policy to use for UP */
            egressSPID =    (uint32_t)nrConfig.handle1;
            break;
        }
        else
        {
            /* Error finding the policy. Check the error */
            if (errCode == NAME_ENOTFOUND)
            {
                /* Policy not found. Wait for sometime and retry */
                usleep (1);
                continue;
            }
            else
            {
                /* Error getting the policy to use for UP. Cant proceed */
                return -1;
            }
        }
    }
    printf ("Debug: Using Policy Id %d for L2 UserPlane Egress\n", egressSPID);

    /* Create the ingress IPv4 fast path for UP */
    memset ((void *)&inboundFPCfg, 0, sizeof (Netfp_InboundFPCfg));

    /* Initialize spidMode */
    if(ingressSPID == NETFP_INVALID_SPID)
        inboundFPCfg.spidMode = Netfp_SPIDMode_INVALID;
    else if (ingressSPID == 0)
        inboundFPCfg.spidMode = Netfp_SPIDMode_ANY_SECURE;
    else
        inboundFPCfg.spidMode = Netfp_SPIDMode_SPECIFIC;

    strncpy (inboundFPCfg.name, "Ingress_FastPath_IPv4_UP", NETFP_MAX_CHAR);
    inboundFPCfg.spId                       =   ingressSPID;
    inboundFPCfg.dstIP.ver                  =   Netfp_IPVersion_IPV4;
    inboundFPCfg.dstIP.addr.ipv4.u.a8[0]    =   eNodeBIPAddress[0];
    inboundFPCfg.dstIP.addr.ipv4.u.a8[1]    =   eNodeBIPAddress[1];
    inboundFPCfg.dstIP.addr.ipv4.u.a8[2]    =   eNodeBIPAddress[2];
    inboundFPCfg.dstIP.addr.ipv4.u.a8[3]    =   eNodeBIPAddress[3];
    inboundFPCfg.srcIP.ver                  =   Netfp_IPVersion_IPV4;
    inboundFPCfg.srcIP.addr.ipv4.u.a8[0]    =   pdnGwIPAddress[0];
    inboundFPCfg.srcIP.addr.ipv4.u.a8[1]    =   pdnGwIPAddress[1];
    inboundFPCfg.srcIP.addr.ipv4.u.a8[2]    =   pdnGwIPAddress[2];
    inboundFPCfg.srcIP.addr.ipv4.u.a8[3]    =   pdnGwIPAddress[3];
    ptrLTEStackDomain->gtpuIngressFPHandle =   Netfp_createInboundFastPath(netfpClientHandle, &inboundFPCfg, &errCode);
    if (ptrLTEStackDomain->gtpuIngressFPHandle == NULL)
    {
        /* Error creating UP Ingress Fastpath */
        printf ("Error: Failed to create UserPlane IPv4 Ingress Fastpath [Error code %d]\n", errCode);
        return -1;
    }

    /* Create the egress IPv4 fast path for UP */
    memset ((void *)&outboundFPCfg, 0, sizeof (Netfp_OutboundFPCfg));
    strncpy (outboundFPCfg.name, "Egress_FastPath_IPv4_UP", NETFP_MAX_CHAR);
    outboundFPCfg.spId                       =   egressSPID;
    outboundFPCfg.dstIP.ver                  =   Netfp_IPVersion_IPV4;
    outboundFPCfg.dstIP.addr.ipv4.u.a8[0]    =   pdnGwIPAddress[0];
    outboundFPCfg.dstIP.addr.ipv4.u.a8[1]    =   pdnGwIPAddress[1];
    outboundFPCfg.dstIP.addr.ipv4.u.a8[2]    =   pdnGwIPAddress[2];
    outboundFPCfg.dstIP.addr.ipv4.u.a8[3]    =   pdnGwIPAddress[3];
    outboundFPCfg.srcIP.ver                  =   Netfp_IPVersion_IPV4;
    outboundFPCfg.srcIP.addr.ipv4.u.a8[0]    =   eNodeBIPAddress[0];
    outboundFPCfg.srcIP.addr.ipv4.u.a8[1]    =   eNodeBIPAddress[1];
    outboundFPCfg.srcIP.addr.ipv4.u.a8[2]    =   eNodeBIPAddress[2];
    outboundFPCfg.srcIP.addr.ipv4.u.a8[3]    =   eNodeBIPAddress[3];

    /* Setup QCI-DSCP mapping for egress fast path */
	for (i = 0; i < 64; i ++)
	    outboundFPCfg.dscpMapping[i] = i;

    ptrLTEStackDomain->gtpuEgressFPHandle =   Netfp_createOutboundFastPath (netfpClientHandle, &outboundFPCfg, &errCode);
    if (ptrLTEStackDomain->gtpuEgressFPHandle == NULL)
    {
        /* Error creating UP Egress Fastpath */
        printf ("Error: Failed to create UserPlane IPv4 Egress Fastpath [Error code %d]\n", errCode);
        return -1;
    }

   /* Loop around waiting for the fast path to become active. */
   while (1)
   {
        int32_t status;

        /* Get the status of the fast path */
        if (Netfp_isOutboundFastPathActive (netfpClientHandle, ptrLTEStackDomain->gtpuEgressFPHandle, &status, &errCode) < 0)
        {
            printf ("Error: Unable to get the outbound fast path activity status [Error code %d]\n", errCode);
            return -1;
        }

        /* Is the fast path active? */
        if(status == 1)
            break;

        /* Relinquish time slice and try again */
        usleep(100);
   }

    /* Create a PKTLIB Heap for User plane Rx data path */
    memset ((void *)&heapConfig, 0, sizeof(Pktlib_HeapCfg));
    strcpy (heapConfig.name, "L2_UP_Rx_Heap");
    heapConfig.pktlibInstHandle                = pktlibHandle;
    heapConfig.memRegion                       = ptrL2ResourceCfg->memRegionResponse[0].memRegionHandle;
    heapConfig.sharedHeap                      = 0;
    heapConfig.useStarvationQueue              = 0;
    heapConfig.dataBufferSize                  = 10*1024;
    heapConfig.numPkts                         = 64;
    heapConfig.numZeroBufferPackets            = 0;
    heapConfig.dataBufferPktThreshold          = 0;
    heapConfig.zeroBufferPktThreshold          = 0;
    heapConfig.heapInterfaceTable.dataMalloc   = L2_MemoryMalloc;
    heapConfig.heapInterfaceTable.dataFree     = L2_MemoryFree;
    ptrLTEStackDomain->gtpuRxHeap = Pktlib_createHeap(&heapConfig, &errCode);
    if (ptrLTEStackDomain->gtpuRxHeap == NULL)
    {
        printf ("Error: Unable to create L2 UserPlane Rx heap [Error code %d]\n", errCode);
        return -1;
    }

    /* Setup an Ingress flow to receive UP packets from NetCP  */
    memset((void *)&flowConfig, 0, sizeof(Netfp_FlowCfg));
    flowConfig.numHeaps         =   1;
    flowConfig.heapHandle[0]    =   ptrLTEStackDomain->gtpuRxHeap;
    flowConfig.sopOffset        =   0;
    strncpy (flowConfig.name, "L2_UP_Flow", NETFP_MAX_CHAR);
    ptrLTEStackDomain->gtpuIngressFlowId = Netfp_createFlow (netfpClientHandle, &flowConfig, &errCode);
    if (ptrLTEStackDomain->gtpuIngressFlowId < 0)
    {
        printf ("Error: Failed to create Ingress Flow for L2 UserPlane [Error code %d]\n", errCode);
        return -1;
    }

    /* Setup a Msgcom channel to receive UP Ingress (Uplink) packets from NetCP */
    memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));
    chConfig.mode                                                   = Msgcom_ChannelMode_BLOCKING;
    chConfig.msgcomInstHandle                                       = msgComInstHandle;
#ifdef USE_ACC_INT
    /* Use accumulated interrupts to receive User plane data */
    chConfig.u.queueCfg.interruptMode                               = Msgcom_QueueInterruptMode_ACCUMULATED_INTERRUPT;
    chConfig.u.queueCfg.queueIntrUnion.accCfg.type                  = Msgcom_AccumulatedChannelType_HIGH;
    chConfig.u.queueCfg.queueIntrUnion.accCfg.accChannel            = ptrL2ResourceCfg->accChannelResponse[0].accChannel;
    chConfig.u.queueCfg.queueIntrUnion.accCfg.accQueue              = ptrL2ResourceCfg->accChannelResponse[0].queue;
    chConfig.u.queueCfg.queueIntrUnion.accCfg.pdspId                = ptrL2ResourceCfg->accChannelResponse[0].pdspId;
    chConfig.u.queueCfg.queueIntrUnion.accCfg.interruptId           = ptrL2ResourceCfg->accChannelResponse[0].eventId;
    chConfig.u.queueCfg.queueIntrUnion.accCfg.maxPageEntries        = 50;
	chConfig.u.queueCfg.queueIntrUnion.accCfg.pacingTimerCount      = 1; /* Unit - 25us */
#else
    /* Use direct interrupts to receive User plane data */
    chConfig.u.queueCfg.interruptMode                               = Msgcom_QueueInterruptMode_DIRECT_INTERRUPT;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.queuePendQueue  = ptrL2ResourceCfg->qPendResponse[0].queue;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.cpIntcId        = ptrL2ResourceCfg->qPendResponse[0].cpIntcId;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.systemInterrupt = ptrL2ResourceCfg->qPendResponse[0].systemInterrupt;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.hostInterrupt   = ptrL2ResourceCfg->qPendResponse[0].hostInterrupt;
#endif
    ptrLTEStackDomain->gtpuChannelHandle = Msgcom_create ("L2_UP_IPv4_UL_Channel", Msgcom_ChannelType_QUEUE,
                                                          &chConfig, &errCode);
    if (ptrLTEStackDomain->gtpuChannelHandle == NULL)
    {
        printf ("Error: Failed to create Msgcom channel for receiving GTP-U packets [Error code %d]\n",
                        errCode);
        return -1;
    }

    /* Populate the user security configuration. */
    memset ((void *)&userConfig, 0, sizeof(Netfp_UserCfg));
    for (index = 0; index < 16; index++)
    {
        encryptionKey[index] = (index & 0xFF);
        integrityKey[index]  = (index & 0xFF);
    }
    userConfig.authMode         = Netfp_3gppAuthMode_EIA2;
    userConfig.srbCipherMode    = Netfp_3gppCipherMode_EEA2;
    userConfig.drbCipherMode    = Netfp_3gppCipherMode_EEA2;
    userConfig.ueId             = ueId;
    userConfig.srbFlowId        = ptrLTEStackDomain->gtpuIngressFlowId;
    userConfig.initialCountC    = 0;
    userConfig.chSrb1Enc        = 0;    /* Set to NULL only because we are testing DRB's here */
    userConfig.chSrb1Dec        = 0;    /* Set to NULL only because we are testing DRB's here */
    userConfig.chSrb2Enc        = 0;    /* Set to NULL only because we are testing DRB's here */
    userConfig.chSrb2Dec        = 0;    /* Set to NULL only because we are testing DRB's here */
    memcpy ((void *)&userConfig.hKeyRrcInt[0],(void *)integrityKey,  sizeof(userConfig.hKeyRrcInt));
    memcpy ((void *)&userConfig.hKeyRrcEnc[0],(void *)encryptionKey, sizeof(userConfig.hKeyRrcEnc));
    memcpy ((void *)&userConfig.hKeyUpEnc[0], (void *)encryptionKey, sizeof(userConfig.hKeyUpEnc));

    /* Create the user */
    ptrLTEStackDomain->ueHandle = Netfp_createUser (netfpClientHandle, &userConfig, &errCode);
    if (ptrLTEStackDomain->ueHandle == NULL)
    {
        printf ("Error: User creation failed [Error code %d]\n", errCode);
        return -1;
    }

    /* Populate the channel bind configuration */
    memset ((void *)&bindConfig, 0, sizeof (Netfp_LTEChannelBindCfg));
    bindConfig.flowId          = ptrLTEStackDomain->gtpuIngressFlowId;
    bindConfig.notifyFunction  = NULL;
    bindConfig.chDrbRohc       = Msgcom_getInternalMsgQueueInfo (ptrLTEStackDomain->gtpuChannelHandle);
    bindConfig.fpHandle        = ptrLTEStackDomain->gtpuIngressFPHandle;
    bindConfig.sin_gtpuId      = gtpuId;
    bindConfig.countC          = 0;
    bindConfig.enableFastPath  = 0;
    bindConfig.chDrbEnc        = 0;

    /* Populate the channel connect configuration */
    memset ((void *)&connectConfig, 0, sizeof (Netfp_LTEChannelConnectCfg));
    connectConfig.fpHandle       = ptrLTEStackDomain->gtpuEgressFPHandle;
    connectConfig.sin_gtpuId     = gtpuId;
    connectConfig.qci            = 3;
    connectConfig.dscp           = 0x22;
    connectConfig.flowId         = ptrLTEStackDomain->gtpuIngressFlowId;
    connectConfig.chDrbDec       = 0;

    /* Create the DRB LTE channel */
    ptrLTEStackDomain->l2DRBChannel = Netfp_createLTEChannel (ptrLTEStackDomain->ueHandle, rbId,
                                                              Netfp_SockFamily_AF_INET, &bindConfig,
                                                              &connectConfig, &errCode);
    if (ptrLTEStackDomain->l2DRBChannel == NULL)
    {
        printf ("Error: Failed to create the LTE channel for DRB: %d [Error code %d]\n",
                        rbId, errCode);
        return -1;
    }
    printf ("Debug: Succesfully created GTP-U channel with GTPU-Id: 0x%x\n", gtpuId);

    /* Initialize the thread attributes. */
    pthread_attr_init(&threadAttr);
    schedParam.sched_priority   =   15;
    pthread_attr_setschedpolicy (&threadAttr, SCHED_FIFO);
    pthread_attr_setschedparam (&threadAttr, &schedParam);

    /* Launch the GTPU receive thread */
	errCode = pthread_create (&ptrLTEStackDomain->l2UserPlaneTask, &threadAttr, L2_UserPlaneMgmtTask, ptrLTEStackDomain);
	if (errCode < 0)
	{
    	printf ("Error: Failed creating L2 User plane management thread [Error code %d]\n", errCode);
        return -1;
	}
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function cleans up all the NetFP handles
 *      set up as part of IPv4 environment setup for
 *      L2 to be operational
 *
 *  @param[in]  ptrLTEStackDomain
 *      Pointer to the LTE Stack domain MCB
 *
 *  @retval
 *      None
 */
void L2_UserPlane_DeInitIPv4Env
(
    AppLTEStackDomainMCB*   ptrLTEStackDomain
)
{
    int32_t            errCode;

    /* Dump the GTPU stats before we clean up */
    System_printf ("************** GTP-U Statistics **************\n");
    System_printf ("No. GTPU IPv4 Packets Received:         %d\n", ptrLTEStackDomain->gtpuRxPktCtr);
    System_printf ("No. GTPU IPv4 Packets Sent:             %d\n", ptrLTEStackDomain->gtpuTxPktCtr);
    System_printf ("No. GTPU IPv4 Send Errors:              %d\n", ptrLTEStackDomain->gtpuTxErrorPktCtr);

    if (ptrLTEStackDomain->l2DRBChannel)
        Netfp_deleteLTEChannel (ptrLTEStackDomain->l2DRBChannel, &errCode);
    if (ptrLTEStackDomain->ueHandle)
        Netfp_deleteUser (ptrLTEStackDomain->ueHandle, &errCode);
    if (ptrLTEStackDomain->gtpuChannelHandle)
        Msgcom_delete (ptrLTEStackDomain->gtpuChannelHandle, &L2_MsgcomFree);
    if (ptrLTEStackDomain->gtpuEgressFPHandle)
        Netfp_deleteOutboundFastPath (Domain_getNetfpClientInstanceHandle(ptrLTEStackDomain->syslibHandle),
                                      ptrLTEStackDomain->gtpuEgressFPHandle, &errCode);
    if (ptrLTEStackDomain->gtpuIngressFPHandle)
        Netfp_deleteInboundFastPath (Domain_getNetfpClientInstanceHandle(ptrLTEStackDomain->syslibHandle),
                                     ptrLTEStackDomain->gtpuIngressFPHandle, &errCode);
    if (ptrLTEStackDomain->gtpuRxHeap)
        Pktlib_deleteHeap (Domain_getPktlibInstanceHandle(ptrLTEStackDomain->syslibHandle),
                           ptrLTEStackDomain->gtpuRxHeap,
                           &errCode);

    /* Kill the L2 Userplane management thread */
    pthread_cancel (ptrLTEStackDomain->l2UserPlaneTask);

    /* Done cleaning up the IPv4 setup */
    return;
}
