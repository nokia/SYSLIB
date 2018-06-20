/**
 *   @file  netfp_ipsec.c
 *
 *   @brief
 *      The file implements the IPSEC functionality in NETFP.
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
#include <string.h>

#include <ddal/ddal_cpu.h> //fzm

/* MCSDK Include Files. */
#include <ti/csl/csl_cache.h>
#include <ti/drv/cppi/cppi_drv.h>

/* SYSLIB Include Files. */
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/netfp/netfp.h>
#include <ti/runtime/netfp/include/netfp_net.h>
#include <ti/runtime/netfp/include/netfp_internal.h>

/* For Debugging only. */
#ifdef __ARMv7
#include <ti/apps/netfp_config/include/NetFP_System_printf.h> //fzm
#else
#include <xdc/runtime/System.h>
#endif

/**************************************************************************
 **************************** Local Definitions ***************************
 **************************************************************************/

/**************************************************************************
 **************************** Extern Definitions **************************
 **************************************************************************/

/* This is the workaround for the SA LLD ESP send data which has been added in NETFP */
extern int16_t Netfp_salld_esp_send_data
(
    Sa_IpsecConfigParams_t* pConfig,
    Sa_SWInfo_t             swInfo,
    void*                   pktInfo
);

/**************************************************************************
 **************************** IPSEC Functions *****************************
 **************************************************************************/

void Netfp_removeSAFromRecomputationList(Netfp_ServerMCB* ptrNetfpServer, const Netfp_IPSecChannel *ptrIPSecChannel)
{
    Netfp_RecomputeRoute* ptrRecomputeRoute = (Netfp_RecomputeRoute*)Netfp_listGetHead((Netfp_ListNode**)&ptrNetfpServer->ptrRouteRecomputationList);
    while(ptrRecomputeRoute != NULL)
    {
        if(ptrRecomputeRoute->ptrResolvedId == &ptrIPSecChannel->requestId)
        {
            Netfp_RecomputeRoute* tmpNode = ptrRecomputeRoute;
            ptrRecomputeRoute = (Netfp_RecomputeRoute*)Netfp_listGetNext ((Netfp_ListNode*)ptrRecomputeRoute);

            Netfp_listRemoveNode((Netfp_ListNode**)&ptrNetfpServer->ptrRouteRecomputationList, (Netfp_ListNode*)tmpNode);
            ptrNetfpServer->cfg.free(tmpNode, sizeof(Netfp_RecomputeRoute));

            break;
        }
        else
            ptrRecomputeRoute = (Netfp_RecomputeRoute*)Netfp_listGetNext ((Netfp_ListNode*)ptrRecomputeRoute);
    }
}

/**
 *  @b Description
 *  @n
 *      The function is used to encrypt the packet and then send the packet
 *      out via the NETCP.
 *
 *  @param[in]  ptrNetfpSocket
 *      Pointer to the NETFP socket
 *  @param[in]  ptrSockTxMetaInfo
 *      Pointer to the socket transmit meta information
 *  @param[in]  ptrPkt
 *      Pointer to the packet which is to be encrypted
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_encryptPkt
(
    Netfp_Socket*           ptrNetfpSocket,
    Netfp_SockTxMetaInfo*   ptrSockTxMetaInfo,
    Ti_Pkt*                 ptrPkt
)
{
    Sa_PktInfo_t                pktInfo;
    void*                       segments[5];
    uint16_t                    usedSegmentSize[5];
    uint16_t                    allocSegmentSize[5];
    Ti_Pkt*                     ptrTrailerPkt;
    uint8_t*                    ptrHdrBuffer;
    uint32_t                    hdrBufferLen;
    uint8_t*                    ptrPayloadBuffer;
    uint32_t                    payloadBufferLen;
    uint8_t*                    ptrTrailerBuffer;
    uint32_t                    trailerBufferLen;
    int32_t                     retVal;
    uint32_t                    psInfo[NETFP_PA_CMD_BUFFER_SIZE];
    paCmdInfo_t                 cmdInfo[NETFP_TX_CMD_MAX];
    uint8_t                     cmdIdx = 0;
    uint16_t                    cmdBufferSize;
    Netfp_ClientMCB*            ptrNetfpClient;
    Netfp_SockL2ConnectInfo*    ptrSockL2ConnectInfo;
    Netfp_SockConnectInfo*      ptrSockConnectInfo;
    Qmss_QueueHnd               queueHnd;
    /* The definition supports upto 8 TX CMD */
    uint8_t                     ifaceTxCmd = 0;
    uint32_t                    saPktHdrMargin;
    uint8_t                     nattOverhead = 0; //fzm

    /* Get the client handle */
    ptrNetfpClient = ptrNetfpSocket->ptrNetfpClient;

    /* Get the socket L2 connect information: */
    ptrSockL2ConnectInfo = ptrSockTxMetaInfo->ptrL2ConnectInfo;

    /* Get the socket connect information: */
    ptrSockConnectInfo = &ptrNetfpSocket->connectInfo;

    saPktHdrMargin = ptrSockConnectInfo->pktHeaderMargin;

    /* Allocate the trailer packet. */
    ptrTrailerPkt = Pktlib_allocPacketPrefetch(ptrNetfpClient->cfg.netHeaderHeapHandle,
                                       ptrSockConnectInfo->pktTrailerMargin +
                                       ptrSockL2ConnectInfo->saGeneralChannelCtrlInfo.txCtrl.params.ipsec.encryptionBlockSize);
    if (ptrTrailerPkt == NULL)
        return NETFP_ENOMEM;

    /* fzm: The SA Margin is increased by UDPHDR_SIZE if the packet is NATT*/
    if ((ptrSockConnectInfo->nattEncapCfg.srcPort != 0) &&
        (ptrSockConnectInfo->nattEncapCfg.dstPort != 0))
        nattOverhead = UDPHDR_SIZE;

    saPktHdrMargin += nattOverhead; //fzm

    /* Each packet sent out through the NETFP to the SA has 3 parts
     *  - Header Packet (Ethernet + Inner/Outer IP headers + ESP + UDP header)
     *  - Standard Data payload.
     *  - ESP Trailer */
    pktInfo.pktDesc.nSegments       = 3;
    pktInfo.pktDesc.segments        = segments;
    pktInfo.pktDesc.segUsedSizes    = usedSegmentSize;
    pktInfo.pktDesc.segAllocSizes   = allocSegmentSize;
    pktInfo.pktDesc.size            = Pktlib_getPacketLen(ptrPkt) - saPktHdrMargin;
    pktInfo.pktDesc.payloadOffset   = ptrSockL2ConnectInfo->l2HeaderSize;
    pktInfo.pktDesc.payloadLen      = pktInfo.pktDesc.size - ptrSockL2ConnectInfo->l2HeaderSize;
    pktInfo.validBitMap             = 0;

    /* Get the header buffer length & pointer. */
    Pktlib_getDataBuffer(ptrPkt, &ptrHdrBuffer, &hdrBufferLen);

    /* Get the payload buffer length & pointer. */
    Pktlib_getDataBuffer(Pktlib_getNextPacket(ptrPkt), &ptrPayloadBuffer, &payloadBufferLen);

    /* In case of chained payload packets, the payload length must be obtained from
     * the packet length of first packet and not the chained descriptors buffer length */
    payloadBufferLen = Pktlib_getPacketLen(ptrPkt) - hdrBufferLen;

    /* Get the trailer buffer length & pointer. */
    Pktlib_getDataBuffer(ptrTrailerPkt, &ptrTrailerBuffer, &trailerBufferLen);

    /* Populate Segment 0:-
     *  - Actual Ethernet Header is located at an offset of the 'NETFP_SA_PKT_HDR_MARGIN' bytes in
     *    the packet.
     *  - The header packet allocated has space allocated for the SA required headroom and
     *    so we need to account for it in the used & allocated size calculations. */
    pktInfo.pktDesc.segments[0]      = (void *)(ptrHdrBuffer + saPktHdrMargin);
    pktInfo.pktDesc.segUsedSizes[0]  = hdrBufferLen - saPktHdrMargin;
    pktInfo.pktDesc.segAllocSizes[0] = hdrBufferLen;

    /* Populate Segment 1:-
     *  - Provide payload length. Payload buffer not used by SA LLD. */
    pktInfo.pktDesc.segments[1]      = NULL;
    pktInfo.pktDesc.segUsedSizes[1]  = payloadBufferLen;
    pktInfo.pktDesc.segAllocSizes[1] = payloadBufferLen;

    /* Populate Segment 2:-
     *  - This has the trailer packet information. */
    pktInfo.pktDesc.segments[2]      = (void *)ptrTrailerBuffer;
    pktInfo.pktDesc.segUsedSizes[2]  = 0;
    pktInfo.pktDesc.segAllocSizes[2] = trailerBufferLen;

    /* Set up NAT-T parameters if enabled. */
    if (nattOverhead) //fzm
    {
        pktInfo.validBitMap      = sa_PKT_INFO_VALID_IPSEC_NAT_T_INFO;
        pktInfo.natTInfo.dstPort = ptrSockConnectInfo->nattEncapCfg.dstPort;
        pktInfo.natTInfo.srcPort = ptrSockConnectInfo->nattEncapCfg.srcPort;
    }

    /* Send the packet to SA for processing */
    retVal = Netfp_salld_esp_send_data (&ptrSockL2ConnectInfo->saGeneralChannelCtrlInfo.txCtrl.params.ipsec,
                                        ptrSockL2ConnectInfo->ipSecTxInfo, &pktInfo);
    if (retVal < 0)
    {
        Pktlib_freePacket(ptrNetfpClient->cfg.pktlibInstHandle, ptrTrailerPkt);
        return retVal;
    }

    /* Update the packet data buffer and length modified by SA LLD
     * - SA LLD modifies only the header and trailer packets, doesnt touch the payload packet. */
    Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)ptrPkt, (Ptr)pktInfo.pktDesc.segments[0], pktInfo.pktDesc.segUsedSizes[0]);
    Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)ptrTrailerPkt, (Ptr)pktInfo.pktDesc.segments[2], pktInfo.pktDesc.segUsedSizes[2]);

    /* Merge the packets together. */
    Pktlib_packetMerge(ptrNetfpClient->cfg.pktlibInstHandle, ptrPkt, ptrTrailerPkt, NULL);

    /* Set the packet length. */
    Pktlib_setPacketLen (ptrPkt, pktInfo.pktDesc.size);

    /*====================================================================================
     * Contruct PA TX command to send the secure packet.
     * The TX commmand has to follow the following sequences:
     * 1. Hardware UDP checksum if configured in ptrSockTxMetaInfo.
     * 2. SA payload
     * 3. L3 Qos if configured in L2 Connect info
     * 4. EMAC switch routing command.
     *====================================================================================*/
    if(ptrSockTxMetaInfo->hwUDPChksum == 1)
    {
        /* UDP checksum command: startOffset, lengthBytes and initialSum are
         * updated for every packet when setting up the UDP header. */
        cmdInfo[cmdIdx].cmd                        = pa_CMD_TX_CHECKSUM;
        cmdInfo[cmdIdx].params.chksum.startOffset  = (ptrSockTxMetaInfo->udpHwChkSum.startOffset +
                                                     (uint32_t)(ptrHdrBuffer) -
                                                     (uint32_t)pktInfo.pktDesc.segments[0]);
        cmdInfo[cmdIdx].params.chksum.lengthBytes  = ptrSockTxMetaInfo->udpHwChkSum.lengthBytes;
        cmdInfo[cmdIdx].params.chksum.resultOffset = 6;
        cmdInfo[cmdIdx].params.chksum.initialSum   = ptrSockTxMetaInfo->udpHwChkSum.initialSum;
        cmdInfo[cmdIdx].params.chksum.negative0    = 1;
        cmdIdx++;

        /* Routing command to send packet to SASS */
        cmdInfo[cmdIdx].cmd                        = pa_CMD_NEXT_ROUTE;
        cmdInfo[cmdIdx].params.route.ctrlBitfield  = 0;
        cmdInfo[cmdIdx].params.route.dest          = pa_DEST_SASS;
        cmdInfo[cmdIdx].params.route.pktType_emacCtrl = 0;
        cmdInfo[cmdIdx].params.route.multiRouteIndex  = (uint16_t)pa_NO_MULTI_ROUTE;
        cmdInfo[cmdIdx].params.route.swInfo0       = pktInfo.swInfo.swInfo[0];
        cmdInfo[cmdIdx].params.route.swInfo1       = pktInfo.swInfo.swInfo[1];
        cmdInfo[cmdIdx].params.route.queue         = ptrNetfpClient->netcpTxQueue[NSS_SA_QUEUE_SASS_INDEX];
        cmdInfo[cmdIdx].params.route.flowId        = ptrSockL2ConnectInfo->ipsecChanFlowID;
        cmdIdx++;

        ifaceTxCmd += 1 << Netfp_PATxCmdSequence_UDPChksum;

        /* If UDP checksum offload is enable, packet needs to be pushed to PDSP5 first */
        queueHnd = ptrNetfpClient->netcpTxQueue[NSS_PA_QUEUE_TXCMD_INDEX];
    }
    else
    {
        /* Set the software info */
        Cppi_setSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)ptrPkt, (uint8_t *)pktInfo.swInfo.swInfo);

        /* Setup the destination Queue to push the packet */
        queueHnd = ptrNetfpClient->netcpTxQueue[NSS_SA_QUEUE_SASS_INDEX];
    }

    /* Add command for SA_PAYLOAD */
    /* Place holder for SA short info, actually data will be filled in from netfp_sa.c */
    cmdInfo[cmdIdx].cmd                      = pa_CMD_SA_PAYLOAD;
    cmdInfo[cmdIdx].params.payload.offset    = pktInfo.pktDesc.payloadOffset;
    cmdInfo[cmdIdx].params.payload.len       = pktInfo.pktDesc.payloadLen;
    cmdInfo[cmdIdx].params.payload.supData   = 0;
    cmdIdx++;
    ifaceTxCmd += 1 << Netfp_PATxCmdSequence_SAPayload;

    /* Is the L3 shaping required? */
    if (ptrSockL2ConnectInfo->l3QosCfg.isEnable == 1)
    {
        /* Construct L3Qos command */
        cmdInfo[cmdIdx].cmd                            = pa_CMD_NEXT_ROUTE;
        /* L3 shaping is required: We need to setup the route information to send the packet
         * to the L3 shaping engine. L3 shaping is always done on the Inner DSCP. */
        cmdInfo[cmdIdx].params.route.ctrlBitfield      = 0;
        cmdInfo[cmdIdx].params.route.dest              = pa_DEST_HOST;
        cmdInfo[cmdIdx].params.route.pktType_emacCtrl  = 0;
        cmdInfo[cmdIdx].params.route.multiRouteIndex   = (uint16_t)pa_NO_MULTI_ROUTE;
        cmdInfo[cmdIdx].params.route.swInfo0           = 0;
        cmdInfo[cmdIdx].params.route.swInfo1           = 0;
        cmdInfo[cmdIdx].params.route.queue             = ptrSockL2ConnectInfo->l3QosCfg.qid[ptrSockTxMetaInfo->priority];
        cmdInfo[cmdIdx].params.route.flowId            = ptrSockL2ConnectInfo->l3QosCfg.flowId;
        cmdIdx++;
        ifaceTxCmd += 1 << Netfp_PATxCmdSequence_RouteL3Qos;
    }

    /* Configure the command set to send the packet to EMAC from SA post encryption
     * Ensure that the command buffer size after formatting is a multiple of 8; if not
     * use the CMD_NONE for alignment. */
    cmdInfo[cmdIdx].cmd                          = pa_CMD_NEXT_ROUTE;
    cmdInfo[cmdIdx].params.route.ctrlBitfield    = 0;
    cmdInfo[cmdIdx].params.route.dest            = pa_DEST_EMAC;
    cmdInfo[cmdIdx].params.route.pktType_emacCtrl= ptrSockL2ConnectInfo->switchPortNum;
    cmdInfo[cmdIdx].params.route.multiRouteIndex = (uint16_t)pa_NO_MULTI_ROUTE;
    cmdInfo[cmdIdx].params.route.swInfo0         = 0;
    cmdInfo[cmdIdx].params.route.swInfo1         = 0;
    cmdInfo[cmdIdx].params.route.queue           = 0;
    cmdInfo[cmdIdx].params.route.flowId          = 0;
    cmdIdx++;
    ifaceTxCmd += 1 << Netfp_PATxCmdSequence_RouteEMAC;

    /* update interface transmit statistics */
    ptrNetfpSocket->extendedStats.ifaceTxStats[ifaceTxCmd]++;

    /* Set the command buffer size. */
    cmdBufferSize = NETFP_PA_CMD_BUFFER_SIZE;

    /* Configure the command set. */
    retVal = Pa_formatTxCmd (cmdIdx, cmdInfo, 0, (Ptr)&psInfo[0], &cmdBufferSize);
    if (retVal != pa_OK)
    {
        // Go through linked descriptors to find trailer
        // We need to unlink it on this level to free what was allocated locally
        // Rest descriptors will be freed from upper level
        Ti_Pkt* ptrHeadPkt = ptrPkt;

        while (ptrHeadPkt != NULL)
        {
            Ti_Pkt* ptrNextPkt = Pktlib_getNextPacket(ptrHeadPkt);

            if (ptrTrailerPkt == ptrNextPkt)
            {
                /* 1) Kill the next link in the payload packet, it is trailer allocated locally.
                      We want to cleanup it here to not pass this responsibility to upper level */
                Cppi_linkNextBD (Cppi_DescType_HOST, (Cppi_Desc*)ptrHeadPkt, (Cppi_Desc*)NULL);

                /* 2) Free allocated trailer */
                Pktlib_freePacket(ptrNetfpClient->cfg.pktlibInstHandle, ptrTrailerPkt);

                break;
            }

            ptrHeadPkt = ptrNextPkt;
        }

        return retVal;
    }

    /* Set Protocol Specific info fields in the descriptor */
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)ptrPkt, (uint8_t *)psInfo, cmdBufferSize );

    /* The Header & Trailer data buffers have also been touched by the SA. */
    ptrNetfpClient->cfg.endMemAccess (ptrHdrBuffer, pktInfo.pktDesc.segUsedSizes[0]);
    ptrNetfpClient->cfg.endMemAccess (ptrTrailerBuffer, pktInfo.pktDesc.segUsedSizes[2]);

    /*********************************************************************************
     * Secure Socket: POST Routing Hook
     *********************************************************************************/
    if (Netfp_socketHook (Netfp_Hook_POST_ROUTING, ptrNetfpClient, ptrNetfpSocket, ptrPkt) < 0)
        return 0;

    /************************************************************************
     * Socket Statistics: Increment the SECURE socket statistics for the
     * IPv4 and IPv6 families
     ************************************************************************/
    if (ptrNetfpSocket->family == Netfp_SockFamily_AF_INET)
        Netfp_incSecureIPv4Stats(ptrNetfpSocket, ptrSockTxMetaInfo, ptrPkt);
    else
        Netfp_incSecureIPv6Stats(ptrNetfpSocket, ptrSockTxMetaInfo, ptrPkt);

    /******************************************************************************
     * NOTE: There is an ownership change here from the DSP to the CPDMA. We need
     * to ensure that the packet contents here are written back.
     ******************************************************************************/
    Pktlib_releaseOwnership(ptrNetfpClient->cfg.pktlibInstHandle, ptrPkt);

    /* Push this into the PA or SA */
    Qmss_queuePushDescSize (queueHnd, (Cppi_Desc*)ptrPkt, 128);

    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to create IPSEC channel virtual link to be shared by
 *  mulitple SAs.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  ptrIPSecChannel
 *      Pointer to the IPSec channel
 *  @param[out]  errCode
 *      Error code populated on error
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   -1
 */
static int32_t Netfp_createPAVlink
(
    Netfp_ServerMCB*        ptrNetfpServer,
    Netfp_IPSecChannel*     ptrIPSecChannel,
    int32_t*                errCode
)
{
    Netfp_PAVirtLink*       ptrNetfpPAVlink;

    /* Create new vlink, allocate memory */
    ptrNetfpPAVlink = ptrNetfpServer->cfg.malloc(sizeof(Netfp_PAVirtLink), 0);
    if (ptrNetfpPAVlink == NULL)
    {
        *errCode = NETFP_ENOMEM;
        return -1;
    }

    /* Initialize Netfp_PAVirtLink*/
    memset((void *)ptrNetfpPAVlink, 0, sizeof(Netfp_PAVirtLink));

    /* Create a virtual link */
    *errCode = Pa_addVirtualLink (ptrNetfpServer->paHandle, &ptrNetfpPAVlink->vlinkHandle,
                                  pa_VIRTUAL_LNK_TYPE_OUTER_IP);
    if (*errCode < 0)
    {
        /* Cleanup the memory */
        ptrNetfpServer->cfg.free(ptrNetfpPAVlink, sizeof(Netfp_PAVirtLink));

        return -1;
    }

    /* Increment the refCount */
    ptrNetfpPAVlink->refCount++;

    /* Created successfully, save the vlink handle in IPSec channel */
    ptrIPSecChannel->ptrNetfpPAVlink = ptrNetfpPAVlink;

    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to create IPSEC channel virtual link to be shared by
 *  mulitple SAs.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  ptrOrigIPSecChannel
 *      Pointer to the original IPSec channel
 *  @param[out]  ptrNewIPSecChannel
 *      Pointer to the new IPSec channel
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   -1
 */
static int32_t Netfp_inheritPAVlink
(
    Netfp_ServerMCB*        ptrNetfpServer,
    Netfp_IPSecChannel*     ptrOrigIPSecChannel,
    Netfp_IPSecChannel*     ptrNewIPSecChannel
)
{
    /* Inherit the PA Vlink information */
    ptrNewIPSecChannel->ptrNetfpPAVlink = ptrOrigIPSecChannel->ptrNetfpPAVlink;

    /* Increment the refCount to share the same vlink */
    ptrNewIPSecChannel->ptrNetfpPAVlink->refCount++;

    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to delete the IPSEC channel virtual link that is shared by
 *   multiple SAs. If the refCount becomes zero, virtual link will be deleted, otherwise
 *   refCount is decremented.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  ptrNetfpPAVlink
 *      Ponter to the NETFP Vlink handle.
 *  @param[out]  errCode
 *      Error code populated on error
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   -1
 */
static int32_t Netfp_deletePAVlink
(
    Netfp_ServerMCB*        ptrNetfpServer,
    Netfp_PAVirtLink*       ptrNetfpPAVlink,
    int32_t*                errCode
)
{
    /* Sanity check on NetfpPAVlink handle */
    if(ptrNetfpPAVlink == NULL)
        return -1;

    /* Sanity check on refCount */
    if (ptrNetfpPAVlink->refCount <= 0)
        return -1;

    /* Decrement the refCount */
    ptrNetfpPAVlink->refCount--;

    /* Delete pa vlink if refCount reaches zero */
    if(ptrNetfpPAVlink->refCount == 0)
    {
        *errCode = Pa_delVirtualLink (ptrNetfpServer->paHandle, &ptrNetfpPAVlink->vlinkHandle);
        if (*errCode < 0)
            return -1;

        /* Cleanup the memory */
        ptrNetfpServer->cfg.free(ptrNetfpPAVlink, sizeof(Netfp_PAVirtLink));

        //fzm:
        if(ptrNetfpPAVlink == ptrNetfpServer->ptrNetfpSW_LUT_PAVlink)
            ptrNetfpServer->ptrNetfpSW_LUT_PAVlink = NULL;
    }

    return 0;
}

//<fzm>

static Netfp_SwIPInfo* Netfp_findSwLutIPChannel
(
    Netfp_ServerMCB*        ptrNetfpServer,
    Netfp_SACfg*            ptrSACfg
)
{
    Netfp_SwIPInfo* ptrSwIPInfo = (Netfp_SwIPInfo*)Netfp_listGetHead((Netfp_ListNode**)&ptrNetfpServer->ptrSwIPChanList);

    while (ptrSwIPInfo != NULL)
    {
        if (Netfp_matchIP(&ptrSACfg->dstIP, &ptrSwIPInfo->ipCfg.dstIP) &&
            Netfp_matchIP(&ptrSACfg->srcIP, &ptrSwIPInfo->ipCfg.srcIP))
            return ptrSwIPInfo;

        ptrSwIPInfo = (Netfp_SwIPInfo*)Netfp_listGetNext((Netfp_ListNode*)ptrSwIPInfo);
    }

    return NULL;
}

static int32_t Netfp_createSwLutIPChannel
(
    Netfp_ServerMCB*        ptrNetfpServer,
    Netfp_IPSecChannel*     ptrIPSecChannel,
    Netfp_SACfg*            ptrSACfg,
    int32_t*                errCode
)
{
    Netfp_SwIPInfo* ptrSwIPInfo = ptrNetfpServer->cfg.malloc(sizeof(*ptrSwIPInfo), 0);
    if (ptrSwIPInfo == NULL)
    {
        *errCode = NETFP_ENOMEM;
        return -1;
    }

    memset((void *)ptrSwIPInfo, 0, sizeof(*ptrSwIPInfo));

    Netfp_IPLutCfg ipLutCfg;
    memset((void *)&ipLutCfg, 0, sizeof(ipLutCfg));

    ipLutCfg.ipCfg.protocol  = IPPROTO_ESP;
    ipLutCfg.ipCfg.spidMode  = Netfp_SPIDMode_ANY_SECURE;

    ipLutCfg.dstInfo.dstType = Netfp_DestType_LAST_RULE;
    ipLutCfg.dstInfo.channel = ptrNetfpServer->swLutInfo.initSwLut.queue;
    ipLutCfg.dstInfo.flowId  = ptrNetfpServer->swLutInfo.initSwLut.flowId;

    ipLutCfg.prevLinkHandle  = NULL;
    ipLutCfg.nextLinkHandle  = NULL;
    ipLutCfg.ptrIPSecChannel = NULL;
    ipLutCfg.flags           = NETFP_IPCFG_FLAGS_NONE;

    ipLutCfg.fastpathMode    = Netfp_FastpathMode_SW_EXTENSION_CHANNEL;

    ptrSwIPInfo->lutInfo = Netfp_allocIPLutEntry (ptrNetfpServer, &ipLutCfg, 1, errCode);
    if (ptrSwIPInfo->lutInfo == NULL )
    {
        ptrNetfpServer->cfg.free(ptrSwIPInfo, sizeof(*ptrSwIPInfo));
        return -1;
    }
    else
    {
        ptrSwIPInfo->refCount++;

        /* Save IP configuration */
        memcpy((void *)&ptrSwIPInfo->ipCfg.srcIP, (void*)&ptrSACfg->srcIP, sizeof(ptrSwIPInfo->ipCfg.srcIP));
        memcpy((void *)&ptrSwIPInfo->ipCfg.dstIP, (void*)&ptrSACfg->dstIP, sizeof(ptrSwIPInfo->ipCfg.dstIP));

        /* Created successfully, save the SW IP info handle in IPSec channel */
        ptrIPSecChannel->ptrSwIPInfo = ptrSwIPInfo;

        /* Add to SW IP LUT channel list on NetFP server */
        Netfp_listAdd ((Netfp_ListNode**)&ptrNetfpServer->ptrSwIPChanList, (Netfp_ListNode*)ptrSwIPInfo);
    }

    return 0;
}

static int32_t Netfp_inheritSwLutIPChannel
(
    Netfp_IPSecChannel*     ptrIPSecChannel,
    Netfp_SwIPInfo*         ptrSwIPInfo
)
{
    if( (ptrIPSecChannel == NULL) || (ptrSwIPInfo == NULL))
        return -1;

    ptrIPSecChannel->ptrSwIPInfo = ptrSwIPInfo;

    ptrSwIPInfo->refCount++;

    return 0;
}

static int32_t Netfp_deleteSwLutIPChannel
(
    Netfp_ServerMCB*        ptrNetfpServer,
    Netfp_SwIPInfo*         ptrSwIPInfo,
    int32_t*                errCode
)
{
    if(ptrSwIPInfo == NULL)
    {
       *errCode = NETFP_EINVAL;
        return -1;
    }

    if (ptrSwIPInfo->refCount <= 0)
    {
       *errCode = NETFP_EINTERNAL;
        return -1;
    }

    ptrSwIPInfo->refCount--;

    if(ptrSwIPInfo->refCount == 0)
    {
        *errCode = Netfp_freeIPLutEntry (ptrNetfpServer, ptrSwIPInfo->lutInfo);
        if ( *errCode < 0)
            return -1;

        Netfp_listRemoveNode ((Netfp_ListNode**)&ptrNetfpServer->ptrSwIPChanList, (Netfp_ListNode*)ptrSwIPInfo);

        ptrNetfpServer->cfg.free(ptrSwIPInfo, sizeof(*ptrSwIPInfo));
    }

    return 0;
}

static int32_t Netfp_configureSwLutIPChannel
(
    Netfp_ServerMCB*    ptrNetfpServer,
    Netfp_IPSecChannel* ptrIPSecChannel,
    Netfp_SACfg*        ptrSACfg
)
{
    Netfp_SwIPInfo* ptrSwIPInfo = NULL;
    int32_t errCode = 0;

    if ( (ptrSwIPInfo = Netfp_findSwLutIPChannel(ptrNetfpServer, ptrSACfg)) == NULL)
    {
        if (Netfp_createSwLutIPChannel(ptrNetfpServer, ptrIPSecChannel, ptrSACfg, &errCode) < 0)
        {
            System_printf("ERR: Could not create SwLutChannel, errCode:%d", errCode);
            return -1;
        }
    }
    else
    {
        if (Netfp_inheritSwLutIPChannel(ptrIPSecChannel, ptrSwIPInfo) < 0)
        {
            System_printf("ERR: Could not inherit SwLutChannel");
            return -1;
        }
    }

    return 0;
}

static Netfp_NattSwIPInfo* Netfp_findNattSwLutIPChannel
(
    Netfp_ServerMCB*        ptrNetfpServer,
    Netfp_SACfg*            ptrSACfg
)
{
    Netfp_NattSwIPInfo*       ptrNattSwIPInfo;

    ptrNattSwIPInfo = (Netfp_NattSwIPInfo*)Netfp_listGetHead((Netfp_ListNode**)&ptrNetfpServer->ptrNattSwIPChanList);

    while (ptrNattSwIPInfo != NULL)
    {
        if (Netfp_matchIP(&ptrSACfg->dstIP, &ptrNattSwIPInfo->ipCfg.dstIP) &&
            Netfp_matchIP(&ptrSACfg->srcIP, &ptrNattSwIPInfo->ipCfg.srcIP))
            return ptrNattSwIPInfo;

        ptrNattSwIPInfo = (Netfp_NattSwIPInfo*)Netfp_listGetNext((Netfp_ListNode*)ptrNattSwIPInfo);
    }

    return NULL;
}

static int32_t Netfp_createNattSwLutIPChannel
(
    Netfp_ServerMCB*        ptrNetfpServer,
    Netfp_IPSecChannel*     ptrIPSecChannel,
    Netfp_SACfg*            ptrSACfg,
    int32_t*                errCode
)
{
    Netfp_NattSwIPInfo* ptrNattSwIPInfo = ptrNetfpServer->cfg.malloc(sizeof(*ptrNattSwIPInfo), 0);
    if (ptrNattSwIPInfo == NULL)
    {
        *errCode = NETFP_ENOMEM;
        return -1;
    }

    memset((void *)ptrNattSwIPInfo, 0, sizeof(*ptrNattSwIPInfo));

    Netfp_IPLutCfg ipLutCfg;
    memset((void *)&ipLutCfg, 0, sizeof(ipLutCfg));

    ipLutCfg.ipCfg.protocol  = IPPROTO_ESP;
    ipLutCfg.ipCfg.spidMode  = Netfp_SPIDMode_ANY_SECURE;

    ipLutCfg.dstInfo.dstType = Netfp_DestType_LAST_RULE;
    ipLutCfg.dstInfo.channel = ptrNetfpServer->swLutInfo.initSwLut.queue;
    ipLutCfg.dstInfo.flowId  = ptrNetfpServer->swLutInfo.initSwLut.flowId;

    ipLutCfg.prevLinkHandle  = ptrIPSecChannel->ptrNattIPInfo->nattLutInfo->ipHandle;
    ipLutCfg.nextLinkHandle  = NULL;
    ipLutCfg.ptrIPSecChannel = NULL;
    ipLutCfg.flags           = NETFP_IPCFG_FLAGS_NATT;

    ipLutCfg.fastpathMode    = Netfp_FastpathMode_SW_EXTENSION_CHANNEL;

    ptrNattSwIPInfo->nattLutInfo = Netfp_allocIPLutEntry (ptrNetfpServer, &ipLutCfg, 1, errCode);
    if (ptrNattSwIPInfo->nattLutInfo == NULL )
    {
        ptrNetfpServer->cfg.free(ptrNattSwIPInfo, sizeof(*ptrNattSwIPInfo));
        return -1;
    }
    else
    {
        ptrNattSwIPInfo->refCount++;

        /* Save IP configuration */
        memcpy((void *)&ptrNattSwIPInfo->ipCfg.srcIP, (void*)&ptrSACfg->srcIP, sizeof(ptrNattSwIPInfo->ipCfg.srcIP));
        memcpy((void *)&ptrNattSwIPInfo->ipCfg.dstIP, (void*)&ptrSACfg->dstIP, sizeof(ptrNattSwIPInfo->ipCfg.dstIP));

        /* Created successfully, save the NAT-T IP info handle in IPSec channel */
        ptrIPSecChannel->ptrNattSwIPInfo = ptrNattSwIPInfo;

        /* Add to NAT-T SW IP LUT channel list on NetFP server */
        Netfp_listAdd ((Netfp_ListNode**)&ptrNetfpServer->ptrNattSwIPChanList, (Netfp_ListNode*)ptrNattSwIPInfo);
    }

    return 0;
}

static int32_t Netfp_inheritNattSwLutIPChannel
(
    Netfp_IPSecChannel*     ptrIPSecChannel,
    Netfp_NattSwIPInfo*     ptrNattSwIPInfo
)
{
    /* Validate NAT-T IP channel and IPSec channel handle */
    if( (ptrIPSecChannel == NULL) || (ptrNattSwIPInfo == NULL))
        return -1;

    /* Inherit the NAT-T IP channel information */
    ptrIPSecChannel->ptrNattSwIPInfo = ptrNattSwIPInfo;

    /* Increment the refCount to share the same NAT-T IP channel */
    ptrNattSwIPInfo->refCount++;

    return 0;
}

static int32_t Netfp_deleteNattSwIPChannel
(
    Netfp_ServerMCB*        ptrNetfpServer,
    Netfp_NattSwIPInfo*     ptrNattSwIPInfo,
    int32_t*                errCode
)
{
    if(ptrNattSwIPInfo == NULL)
    {
       *errCode = NETFP_EINVAL;
        return -1;
    }

    if (ptrNattSwIPInfo->refCount <= 0)
    {
       *errCode = NETFP_EINTERNAL;
        return -1;
    }

    ptrNattSwIPInfo->refCount--;

    if(ptrNattSwIPInfo->refCount == 0)
    {
        *errCode = Netfp_freeIPLutEntry (ptrNetfpServer, ptrNattSwIPInfo->nattLutInfo);
        if ( *errCode < 0)
            return -1;

        Netfp_listRemoveNode ((Netfp_ListNode**)&ptrNetfpServer->ptrNattSwIPChanList, (Netfp_ListNode*)ptrNattSwIPInfo);

        ptrNetfpServer->cfg.free(ptrNattSwIPInfo, sizeof(*ptrNattSwIPInfo));
    }

    return 0;
}

static int32_t Netfp_inheritNattIPChannel (Netfp_IPSecChannel*, Netfp_NattIPInfo*);
static Netfp_NattIPInfo* Netfp_findNattIPChannel (Netfp_ServerMCB*, Netfp_SACfg*);
static int32_t Netfp_createNattIPChannel (Netfp_ServerMCB*, Netfp_IPSecChannel*, Netfp_SACfg*, int32_t*);

static int32_t Netfp_configureNattSwLutIPChannel
(
    Netfp_ServerMCB*    ptrNetfpServer,
    Netfp_IPSecChannel* ptrIPSecChannel,
    Netfp_SACfg*        ptrSACfg
)
{
    Netfp_SACfg origSACfg;
    memcpy((void *)&origSACfg, (void *)ptrSACfg,  sizeof(origSACfg));

    if (ptrNetfpServer->nattCfg.wildCardedEntry == 1)
    {
        /* Set Src IP as wild carded address */
        if(ptrSACfg->srcIP.ver == Netfp_IPVersion_IPV4)
            ptrSACfg->srcIP.addr.ipv4.u.a32 = 0;
        else
        {
            ptrSACfg->srcIP.addr.ipv6.u.a32[0] = 0;
            ptrSACfg->srcIP.addr.ipv6.u.a32[1] = 0;
            ptrSACfg->srcIP.addr.ipv6.u.a32[2] = 0;
            ptrSACfg->srcIP.addr.ipv6.u.a32[3] = 0;
        }
    }

    Netfp_NattIPInfo* ptrNattIPInfo = NULL;
    int32_t errCode = 0;

    /* Find if the NATT has been created already */
    if ( (ptrNattIPInfo = Netfp_findNattIPChannel(ptrNetfpServer, ptrSACfg)) == NULL)
    {
        /* Create NAT-T UDP encap channel */
        if (Netfp_createNattIPChannel(ptrNetfpServer, ptrIPSecChannel, ptrSACfg, &errCode) < 0)
        {
            System_printf("ERR: Could not create NattIPChannel, errCode:%d", errCode);
            return -1;
        }
    }
    else
    {
        /* Inherit NAT-T UDP encap channel */
        if (Netfp_inheritNattIPChannel(ptrIPSecChannel, ptrNattIPInfo) < 0)
        {
            System_printf("ERR: Could not inherit NattIPChannel");
            return -1;
        }
    }

    /* Sanity check of created/inheritted IPHandle for NAT-T IP Channel */
    if(ptrIPSecChannel->ptrNattIPInfo->nattLutInfo->ipHandle == NULL)
    {
        System_printf("ERR: %s: IPHandle for NAT-T IP Channel does not exist", __FUNCTION__);
        return -1;
    }

    memcpy((void *)ptrSACfg, (void *)&origSACfg, sizeof(*ptrSACfg));

    Netfp_NattSwIPInfo* ptrNattSwIPInfo = NULL;
    if ( (ptrNattSwIPInfo = Netfp_findNattSwLutIPChannel(ptrNetfpServer, ptrSACfg)) == NULL)
    {
        if (Netfp_createNattSwLutIPChannel(ptrNetfpServer, ptrIPSecChannel, ptrSACfg, &errCode) < 0)
        {
            System_printf("ERR: Could not create NattSwLutChannel, errCode:%d", errCode);
            return -1;
        }
    }
    else
    {
        if (Netfp_inheritNattSwLutIPChannel(ptrIPSecChannel, ptrNattSwIPInfo) < 0)
        {
            System_printf("ERR: Could not inherit NattSwLutChannel");
            return -1;
        }
    }

    return 0;
}

static Netfp_PendingSW_SAOffloadNode* Netfp_findPendingSW_SA
(
    Netfp_ServerMCB*    ptrNetfpServer,
    uint32_t            jobID
)
{
    Netfp_PendingSW_SAOffloadNode* swSANode = (Netfp_PendingSW_SAOffloadNode*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrPendingSW_SAOffloadList);
    while (swSANode != NULL)
    {
        if (swSANode->jobID == jobID)
            break;

        swSANode = (Netfp_PendingSW_SAOffloadNode*)Netfp_listGetNext ((Netfp_ListNode*)swSANode);
    }

    return swSANode;
}

static int32_t Netfp_addPendingSW_SA
(
    Netfp_ServerMCB*    ptrNetfpServer,
    Netfp_IPSecChannel* ptrIPSecChannel,
    Netfp_EventId       eventType,
    uint32_t*           jobID,
    int32_t*            errCode
)
{
    static uint32_t JOB_ID = 0u;
    *errCode = 0;

    Netfp_PendingSW_SAOffloadNode* swSANode = ptrNetfpServer->cfg.malloc (sizeof(Netfp_PendingSW_SAOffloadNode), 0);
    if (swSANode == NULL)
    {
        System_printf("Error while adding SW_SA (type:%s) pending job - no memory",
                      (eventType == Netfp_EventId_ADD_SW_OFFLOAD_SA ? "ADD" : "DEL"));
        *errCode = NETFP_ENOMEM;
        return -1;
    }

    memset(swSANode, 0, sizeof(*swSANode));

    *jobID = ++JOB_ID;
    swSANode->jobID = *jobID;
    swSANode->eventType = eventType;
    swSANode->ptrIPSecChannel = ptrIPSecChannel;
    swSANode->startOffloadTicks = ddal_cpu_counter_read();

    System_printf("Debug: starting SW offload (jobID: %u)", *jobID);

    Netfp_listAdd ((Netfp_ListNode**)&ptrNetfpServer->ptrPendingSW_SAOffloadList, (Netfp_ListNode*)swSANode);

    return 0;
}

static int32_t Netfp_delPendingSW_SA
(
    Netfp_ServerMCB*    ptrNetfpServer,
    uint32_t            jobID
)
{
    Netfp_PendingSW_SAOffloadNode* swSANode = Netfp_findPendingSW_SA (ptrNetfpServer, jobID);
    if (swSANode == NULL)
    {
        System_printf("Error: Deleting unexisting entry of PendingSW_SA [%u]", jobID);
        return -1;
    }

    Netfp_listRemoveNode ((Netfp_ListNode**)&ptrNetfpServer->ptrPendingSW_SAOffloadList, (Netfp_ListNode*)swSANode);
    ptrNetfpServer->cfg.free (swSANode, sizeof(Netfp_PendingSW_SAOffloadNode));

    return 0;
}

//</fzm>

/**
 *  @b Description
 *  @n
 *      The function is used to find NATT UDP encapsulation LUT 1-1 entries.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  ptrSACfg
 *      Pointer to the SA configuration
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   -1
 */
static Netfp_NattIPInfo* Netfp_findNattIPChannel
(
    Netfp_ServerMCB*        ptrNetfpServer,
    Netfp_SACfg*            ptrSACfg
)
{
    Netfp_NattIPInfo*       ptrNattIPInfo;

    /* Get the first Natt IP info from the list head */
    ptrNattIPInfo = (Netfp_NattIPInfo*)Netfp_listGetHead((Netfp_ListNode**)&ptrNetfpServer->ptrNattIPChanList);

    /* Walk through the the list to find the matching IP configuration */
    while (ptrNattIPInfo != NULL)
    {
        /* Is there a match for source IP address and destination IP address */
        if (Netfp_matchIP(&ptrSACfg->dstIP, &ptrNattIPInfo->ipCfg.dstIP) &&
            Netfp_matchIP(&ptrSACfg->srcIP, &ptrNattIPInfo->ipCfg.srcIP))
            return ptrNattIPInfo;

        /* Get the next entry in the list */
        ptrNattIPInfo = (Netfp_NattIPInfo*)Netfp_listGetNext((Netfp_ListNode*)ptrNattIPInfo);
    }

    /* There is no matching entry */
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      The function is used to create NATT UDP encapsulation LUT 1-1 entries .
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  ptrIPSecChannel
 *      Pointer to the IPSec channel
 *  @param[in]  ptrSACfg
 *      Pointer to the SA configuration
 *  @param[out]  errCode
 *      Error code populated on error
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   -1
 */
static int32_t Netfp_createNattIPChannel
(
    Netfp_ServerMCB*        ptrNetfpServer,
    Netfp_IPSecChannel*     ptrIPSecChannel,
    Netfp_SACfg*            ptrSACfg,
    int32_t*                errCode
)
{
    Netfp_NattIPInfo*       ptrNattIPInfo;
    Netfp_IPLutCfg          ipLutCfg;

    /* Create new NAT-T IP Channel, allocate memory */
    ptrNattIPInfo = ptrNetfpServer->cfg.malloc(sizeof(Netfp_NattIPInfo), 0);
    if (ptrNattIPInfo == NULL)
    {
        *errCode = NETFP_ENOMEM;
        return -1;
    }

    /* Initialize Netfp_NattIPInfo */
    memset((void *)ptrNattIPInfo, 0, sizeof(Netfp_NattIPInfo));

    /* Populate the IP configuration */
    memset((void *)&ipLutCfg, 0, sizeof(Netfp_IPLutCfg));

    ipLutCfg.ipCfg.dstIP     = ptrSACfg->dstIP;
    ipLutCfg.ipCfg.srcIP     = ptrSACfg->srcIP;
    ipLutCfg.ipCfg.protocol  = IPPROTO_UDP;

    /* Populate the destination information */
    ipLutCfg.dstInfo.dstType = Netfp_DestType_LINKED_RULE;
    ipLutCfg.dstInfo.flowId  = Cppi_getFlowId(ptrNetfpServer->ipsecFlowHandle);

    /* Prepare IP Lut configuration */
    ipLutCfg.prevLinkHandle  = NULL;
    ipLutCfg.nextLinkHandle  = NULL;
    ipLutCfg.ptrIPSecChannel = NULL;
    ipLutCfg.flags           = NETFP_IPCFG_FLAGS_NONE;

    /* Allocate LUT resources for the IPSEC channel: */
    ptrNattIPInfo->nattLutInfo = Netfp_allocIPLutEntry (ptrNetfpServer, &ipLutCfg, 1, errCode);
    if (ptrNattIPInfo->nattLutInfo == NULL )
    {
        /* Cleanup the memory */
        ptrNetfpServer->cfg.free(ptrNattIPInfo, sizeof(Netfp_NattIPInfo));
        return -1;
    }
    else
    {
        /* Increment the refCount */
        ptrNattIPInfo->refCount++;

        /* Save IP configuration */
        memcpy((void *)&ptrNattIPInfo->ipCfg.srcIP, (void*)&ptrSACfg->srcIP, sizeof(Netfp_IPAddr));
        memcpy((void *)&ptrNattIPInfo->ipCfg.dstIP, (void*)&ptrSACfg->dstIP, sizeof(Netfp_IPAddr));

        /* Created successfully, save the NAT-T IP info handle in IPSec channel */
        ptrIPSecChannel->ptrNattIPInfo = ptrNattIPInfo;

        /* Add to NAT-T IP channel list on NetFP server */
        Netfp_listAdd ((Netfp_ListNode**)&ptrNetfpServer->ptrNattIPChanList, (Netfp_ListNode*)ptrNattIPInfo);
    }

    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to inherit IPSEC channel NAT-T IP info.
 *
 *  @param[in]  ptrIPSecChannel
 *      Pointer to the IPSec channel that would like to inherit NAT-T IP channel.
 *  @param[out]  ptrNattIPInfo
 *      Pointer to the NAT-T IP channel to be inheritted.
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   -1
 */
static int32_t Netfp_inheritNattIPChannel
(
    Netfp_IPSecChannel*     ptrIPSecChannel,
    Netfp_NattIPInfo*       ptrNattIPInfo
)
{
    /* Validate NAT-T IP channel and IPSec channel handle */
    if( (ptrIPSecChannel == NULL) || (ptrNattIPInfo == NULL))
        return -1;

    /* Inherit the NAT-T IP channel information */
    ptrIPSecChannel->ptrNattIPInfo = ptrNattIPInfo;

    /* Increment the refCount to share the same NAT-T IP channel */
    ptrNattIPInfo->refCount++;

    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to delete the IPSEC channel NAT-T IP channel. RefCount is decremented.
 *   If the refCount becomes zero, NAT-T IP channel will be deleted.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  ptrNattIPInfo
 *      Pointer to the NAT-T IP channel to be inheritted.
 *  @param[out]  errCode
 *      Error code populated on error
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   -1
 */
static int32_t Netfp_deleteNattIPChannel
(
    Netfp_ServerMCB*        ptrNetfpServer,
    Netfp_NattIPInfo*       ptrNattIPInfo,
    int32_t*                errCode
)
{
    /* Sanity check on ptrNattIPInfo handle */
    if(ptrNattIPInfo == NULL)
    {
       *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Sanity check on refCount */
    if (ptrNattIPInfo->refCount <= 0)
    {
       *errCode = NETFP_EINTERNAL;
        return -1;
    }

    /* Decrement the refCount */
    ptrNattIPInfo->refCount--;

    /* Delete NAT-T IP channel if refCount reaches zero */
    if(ptrNattIPInfo->refCount == 0)
    {
        /* Delete the NAT-T IP Channel LUT Entry: */
        *errCode = Netfp_freeIPLutEntry (ptrNetfpServer, ptrNattIPInfo->nattLutInfo);
        if ( *errCode < 0)
            return -1;

        /* Remove from NAT-T IP channel list on NetFP server */
        Netfp_listRemoveNode ((Netfp_ListNode**)&ptrNetfpServer->ptrNattIPChanList, (Netfp_ListNode*)ptrNattIPInfo);

        /* Cleanup the memory */
        ptrNetfpServer->cfg.free(ptrNattIPInfo, sizeof(Netfp_NattIPInfo));
    }

    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to setup the PMTU of the IPSEC channel
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  ptrIPSecChannel
 *      Pointer to the IPSEC Channel for which the MTU is to be setup.
 *  @param[in]  newPMTU
 *      New PMTU to be configured
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
void Netfp_setupIPSECChannelPMTU
(
    Netfp_ServerMCB*        ptrNetfpServer,
    Netfp_IPSecChannel*     ptrIPSecChannel,
    uint32_t                newPMTU
)
{
    uint32_t    currentPMTU;

    /* Take a snapshot of the current PMTU */
    currentPMTU = ptrIPSecChannel->ipsecPMTU;

    /* Is the IPSEC channel ACTIVE or ZOMBIE? */
    if (ptrIPSecChannel->status == Netfp_Status_ZOMBIE)
    {
        /* ZOMBIE: The IPSEC channel is not reachable. This could happen in the following cases:-
         *  (a): Creation where the SA route resolution is not complete
         *  (b): Moving from an ACTIVE to ZOMBIE state
         *
         * In both the cases we reset the IPSEC Channel MTU back to INVALID. Case (a) is valid only
         * during the IPSEC channel creation
         *
         * Case (b) is encountered on a route flush or if the next hop neighbor is no longer
         * reachable. In this case we can no longer use the older MTU because there might be
         * another route which has a different MTU. */
        ptrIPSecChannel->ipsecPMTU = NETFP_INVALID_MTU;
    }
    else
    {
        /* Active IPSEC Channel: We need to consider the following
         *  (a) Existing IPSEC PMTU configuration
         *  (b) New PMTU which is to be configured
         * Select the minimum of the above */
         ptrIPSecChannel->ipsecPMTU = Netfp_min (ptrIPSecChannel->ipsecPMTU, newPMTU);

        /* Setup the age of the PMTU: */
        if (currentPMTU != ptrIPSecChannel->ipsecPMTU)
            ptrIPSecChannel->agePMTU = NETFP_MAX_PMTU_AGE;
    }

    /* Debug Message: */
    Netfp_logMsg (ptrNetfpServer, Netfp_LogLevel_DEBUG, "Debug: IPSEC Channel [SPI: %x] PMTU:%d bytes Age:%d sec\n",
                  ptrIPSecChannel->saCfg.spi, ptrIPSecChannel->ipsecPMTU, ptrIPSecChannel->agePMTU);
    return;
}

static void Netfp_cleanupSecChanAndPAVlink
(
    Netfp_ServerMCB*        ptrNetfpServer,
    Netfp_IPSecChannel*     ptrIPSecChannel,
    int32_t                 line
)
{
    int32_t errCode = 0;

    if(_Netfp_deleteSecurityChannel(ptrNetfpServer, ptrIPSecChannel->ptrSrvSecurityChannel, &errCode) < 0)
        System_printf("Error while deleting SecChan [%d] (LINE:%d)", errCode, line);

    if(Netfp_deletePAVlink(ptrNetfpServer, ptrIPSecChannel->ptrNetfpPAVlink, &errCode) < 0)
        System_printf("Error while deleting PAVlink [%d] (LINE:%d)", errCode, line);
}

/**
 *  @b Description
 *  @n
 *      The function is used to create the IPSEC channel and register this
 *      with the NETCP subystem. For INBOUND channels the function will also
 *      program the LUT1 with the outer IP header information.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  ptrSACfg
 *      Pointer to the security association configuration
 *  @param[in]  ptrOrigIPSecChannel
 *      Handle to the Original IPSec channel
 *  @param[out]  errCode
 *      Error code populated on error
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   Handle to the security association
 *  @retval
 *      Error   -   NULL
 */
static Netfp_SAHandle _Netfp_addSA
(
    Netfp_ServerMCB*        ptrNetfpServer,
    Netfp_SACfg*            ptrSACfg,
    Netfp_IPSecChannel*     ptrOrigIPSecChannel,
    int32_t*                errCode
)
{
    Netfp_IPSecChannel*     ptrIPSecChannel;
    Netfp_SecChannelCfg     secChannelCfg;
    Netfp_SecuritySwInfo    swInfo;
    Netfp_IPLutCfg          ipLutCfg;
    Netfp_ProxyServerInfo*  ptrProxyServerInfo;
    Netfp_EventMetaInfo     eventInfo; // fzm
    Netfp_ProxyServerOneMsg proxyServerOneMsg;

//fzm-->
    /* Check if can continue for SW LUT mode. */
    if ((ptrSACfg->offloadMode == Netfp_OffloadMode_SOFTWARE) && !ptrNetfpServer->swLutInfo.isInitialized)
    {
        *errCode = NETFP_ENOPERM;
        return NULL;
    }
//<--fzm

    /* Allocate memory for the IPSEC Channel: */
    ptrIPSecChannel = ptrNetfpServer->cfg.malloc (sizeof(Netfp_IPSecChannel), 0);
    if (ptrIPSecChannel == NULL)
    {
        *errCode = NETFP_ENOMEM;
        return NULL;
    }

    /* Initialize the allocated memory block. */
    memset ((void*)ptrIPSecChannel, 0, sizeof(Netfp_IPSecChannel));

    /* Copy over the IPSec configuration block. */
    memcpy ((void *)&ptrIPSecChannel->saCfg, (void *)ptrSACfg, sizeof(Netfp_SACfg));

    /* Initialize the security channel configuration: */
    memset ((void*)&secChannelCfg, 0, sizeof(Netfp_SecChannelCfg));

    /* Populate the security channel configuration: */
    secChannelCfg.type = Netfp_SecurityChannelType_IPSEC;
    memcpy ((void *)&secChannelCfg.u.ipSecCfg, (void *)ptrSACfg, sizeof(Netfp_SACfg));

    /* Create and register the security channel in the SA LLD and NETCP susbystem. */
    ptrIPSecChannel->ptrSrvSecurityChannel = _Netfp_createSecurityChannel (ptrNetfpServer, &secChannelCfg, &swInfo, errCode);
    if (ptrIPSecChannel->ptrSrvSecurityChannel == NULL)
    {
        /* Error: Security channel creation failed. Cleanup */
        ptrNetfpServer->cfg.free (ptrIPSecChannel, sizeof(Netfp_IPSecChannel));
        return NULL;
    }

    /* Is this an INBOUND or OUTBOUND Security association?  */
    if (ptrIPSecChannel->saCfg.direction == Netfp_Direction_INBOUND)
    {
        /* INBOUND: Program LUT 1-1 for outer IP rule: Initialize the destination information. */
        memset((void *)&ipLutCfg, 0, sizeof(Netfp_IPLutCfg));

        //<fzm>
        if (ptrSACfg->offloadMode == Netfp_OffloadMode_SOFTWARE)
        {
            if (ptrNetfpServer->ptrNetfpSW_LUT_PAVlink)
            {
                ptrIPSecChannel->ptrNetfpPAVlink = ptrNetfpServer->ptrNetfpSW_LUT_PAVlink;
                ptrNetfpServer->ptrNetfpSW_LUT_PAVlink->refCount++;
            }
            else
            {
                if (Netfp_createPAVlink(ptrNetfpServer, ptrIPSecChannel, errCode) < 0)
                {
                    System_printf("Error while creating PAVlink [%d] (LINE:%d)", *errCode, __LINE__);

                    if(_Netfp_deleteSecurityChannel(ptrNetfpServer, ptrIPSecChannel->ptrSrvSecurityChannel, errCode) < 0)
                        System_printf("Error while deleting SecChan [%d] (LINE:%d)", *errCode, __LINE__);

                    ptrNetfpServer->cfg.free (ptrIPSecChannel, sizeof(Netfp_IPSecChannel));
                    return NULL;
                }
                ptrNetfpServer->ptrNetfpSW_LUT_PAVlink = ptrIPSecChannel->ptrNetfpPAVlink;
            }
        }
        //</fzm>
        /* For rekeying, inherit the vlink, otherwise create new vlink in PA */
        else if(ptrOrigIPSecChannel == NULL)
        {
            /* Create vlink for IPSec Channel. If the vlink has already been created, then refCount
               will be incremented (for rekeying scenario). If the vlink has not been created, create
               vlink in PA (for brand new SA creation ) */
            if (Netfp_createPAVlink(ptrNetfpServer, ptrIPSecChannel, errCode) < 0)
            {
                System_printf("Error while creating PAVlink [%d] (LINE:%d)", *errCode, __LINE__);

                if(_Netfp_deleteSecurityChannel(ptrNetfpServer, ptrIPSecChannel->ptrSrvSecurityChannel, errCode) < 0)
                    System_printf("Error while deleting SecChan [%d] (LINE:%d)", *errCode, __LINE__);

                ptrNetfpServer->cfg.free (ptrIPSecChannel, sizeof(Netfp_IPSecChannel));
                return NULL;
            }
        }
        else
        {
            /* Inherit vlink handle */
            if (Netfp_inheritPAVlink(ptrNetfpServer, ptrOrigIPSecChannel, ptrIPSecChannel) < 0)
            {
                if(_Netfp_deleteSecurityChannel(ptrNetfpServer, ptrIPSecChannel->ptrSrvSecurityChannel, errCode) < 0)
                    System_printf("Error while deleting SecChan [%d] (LINE:%d)", *errCode, __LINE__);

                *errCode = NETFP_EINTERNAL;
                ptrNetfpServer->cfg.free (ptrIPSecChannel, sizeof(Netfp_IPSecChannel));
                return NULL;
            }
        }

//fzm-->
        if( (ptrSACfg->nattEncapCfg.srcPort == 0) &&
            (ptrSACfg->nattEncapCfg.dstPort == 0) &&
            (ptrSACfg->offloadMode == Netfp_OffloadMode_SOFTWARE) )
        {
            if(Netfp_configureSwLutIPChannel(ptrNetfpServer, ptrIPSecChannel, ptrSACfg) < 0)
            {
                *errCode = NETFP_EINTERNAL;
                Netfp_cleanupSecChanAndPAVlink(ptrNetfpServer, ptrIPSecChannel, __LINE__);
                ptrNetfpServer->cfg.free (ptrIPSecChannel, sizeof(*ptrIPSecChannel));
                return NULL;
            }

            /* Generate the Netfp_EventId_ADD_SW_OFFLOAD_SA event and push it to dedicated NETFP client */
            eventInfo.eventId        = Netfp_EventId_ADD_SW_OFFLOAD_SA;
            eventInfo.u.saMeta.spi   = ptrSACfg->spi;
            memcpy((void*)&eventInfo.u.saMeta.swInfo,
                   (void*)&ptrIPSecChannel->ptrSrvSecurityChannel->swInfo.rxInfo.swInfo,
                   sizeof(eventInfo.u.saMeta.swInfo));
            memcpy((void*)&eventInfo.u.saMeta.dstIP,(void*)&ptrSACfg->dstIP,sizeof(eventInfo.u.saMeta.dstIP));

            if(Netfp_addPendingSW_SA(ptrNetfpServer, ptrIPSecChannel, Netfp_EventId_ADD_SW_OFFLOAD_SA, &eventInfo.u.saMeta.jobID, errCode) < 0)
            {
                Netfp_cleanupSecChanAndPAVlink(ptrNetfpServer, ptrIPSecChannel, __LINE__);
                ptrNetfpServer->cfg.free (ptrIPSecChannel, sizeof(*ptrIPSecChannel));
                return NULL;
            }

            Netfp_generateEvent(ptrNetfpServer, &eventInfo);

            /* The INBOUND security association is ACTIVE */
            ptrIPSecChannel->status = Netfp_Status_ACTIVE;

            /* Record the security association in the server. */
            Netfp_listAdd ((Netfp_ListNode**)&ptrNetfpServer->ptrIPSecInboundChannels, (Netfp_ListNode*)ptrIPSecChannel);
            return (Netfp_SAHandle)ptrIPSecChannel;
        }
        else if (ptrNetfpServer->nattCfg.udpPort != 0
                 && ptrSACfg->offloadMode == Netfp_OffloadMode_SOFTWARE)
        {
            if(Netfp_configureNattSwLutIPChannel(ptrNetfpServer, ptrIPSecChannel, ptrSACfg) < 0)
            {
                *errCode = NETFP_EINTERNAL;
                Netfp_cleanupSecChanAndPAVlink(ptrNetfpServer, ptrIPSecChannel, __LINE__);
                ptrNetfpServer->cfg.free (ptrIPSecChannel, sizeof(*ptrIPSecChannel));
                return NULL;
            }

            /* Generate the Netfp_EventId_ADD_SW_OFFLOAD_SA event and push it to dedicated NETFP client */
            eventInfo.eventId        = Netfp_EventId_ADD_SW_OFFLOAD_SA;
            eventInfo.u.saMeta.spi   = ptrSACfg->spi;
            memcpy((void*)&eventInfo.u.saMeta.swInfo,
                   (void*)&ptrIPSecChannel->ptrSrvSecurityChannel->swInfo.rxInfo.swInfo,
                   sizeof(eventInfo.u.saMeta.swInfo));
            memcpy((void*)&eventInfo.u.saMeta.dstIP,(void*)&ptrSACfg->dstIP,sizeof(eventInfo.u.saMeta.dstIP));

            if(Netfp_addPendingSW_SA(ptrNetfpServer, ptrIPSecChannel, Netfp_EventId_ADD_SW_OFFLOAD_SA, &eventInfo.u.saMeta.jobID, errCode) < 0)
            {
                Netfp_cleanupSecChanAndPAVlink(ptrNetfpServer, ptrIPSecChannel, __LINE__);
                ptrNetfpServer->cfg.free (ptrIPSecChannel, sizeof(*ptrIPSecChannel));
                return NULL;
            }

            Netfp_generateEvent(ptrNetfpServer, &eventInfo);

            /* The INBOUND security association is ACTIVE */
            ptrIPSecChannel->status = Netfp_Status_ACTIVE;

            /* Record the security association in the server. */
            Netfp_listAdd ((Netfp_ListNode**)&ptrNetfpServer->ptrIPSecInboundChannels, (Netfp_ListNode*)ptrIPSecChannel);
            return (Netfp_SAHandle)ptrIPSecChannel;
        }
//<--fzm
        /* Is UDP encapsulation needed for NAT-T? */
        else if( (ptrSACfg->nattEncapCfg.srcPort == 0) &&
                 (ptrSACfg->nattEncapCfg.dstPort == 0) &&
                 (ptrSACfg->offloadMode == Netfp_OffloadMode_HARDWARE) )
        {
            /* Populate the IP Configuration. */
            ipLutCfg.ipCfg.dstIP     = ptrSACfg->dstIP;
            ipLutCfg.ipCfg.srcIP     = ptrSACfg->srcIP;
            ipLutCfg.ipCfg.spi       = ptrSACfg->spi;
            ipLutCfg.ipCfg.protocol  = IPPROTO_ESP;

            /* Populate the destination information */
            ipLutCfg.dstInfo.dstType = Netfp_DestType_LINKED_RULE;
            ipLutCfg.dstInfo.flowId  = Cppi_getFlowId(ptrNetfpServer->ipsecFlowHandle);

            /* Prepare IP Lut configuration */
            ipLutCfg.prevLinkHandle  = NULL;
            ipLutCfg.nextLinkHandle  = ptrIPSecChannel->ptrNetfpPAVlink->vlinkHandle;
            ipLutCfg.ptrIPSecChannel = ptrIPSecChannel;
            ipLutCfg.flags           = NETFP_IPCFG_FLAGS_NONE;

            /* Allocate LUT resources for the IPSEC channel: */
            ptrIPSecChannel->ptrSaLutInfo = Netfp_allocIPLutEntry (ptrNetfpServer, &ipLutCfg, 1, errCode);
            if (ptrIPSecChannel->ptrSaLutInfo == NULL )
            {
                Netfp_cleanupSecChanAndPAVlink(ptrNetfpServer, ptrIPSecChannel, __LINE__);
                ptrNetfpServer->cfg.free (ptrIPSecChannel, sizeof(Netfp_IPSecChannel));
                return NULL;
            }

            /* The INBOUND security association is ACTIVE */
            ptrIPSecChannel->status = Netfp_Status_ACTIVE;

            /* Record the security association in the server. */
            Netfp_listAdd ((Netfp_ListNode**)&ptrNetfpServer->ptrIPSecInboundChannels, (Netfp_ListNode*)ptrIPSecChannel);
            return (Netfp_SAHandle)ptrIPSecChannel;
        }
        /* Check if NAT-T is enabled on master/server */
        else if (ptrNetfpServer->nattCfg.udpPort != 0 &&
                 ptrSACfg->offloadMode == Netfp_OffloadMode_HARDWARE)
        {
            Netfp_NattIPInfo*       ptrNattIPInfo;
            Netfp_SACfg             origSACfg;

            /* =======================================================
             * Program LUT1-1 for NAT-T outer IP rule
             * 1. Add IP address without SPI for UDP encapsulation
             * 2. Add SPI only for ESP
             * =======================================================*/

            /* Global NATT wild carded settings is specified, then set the src IP to all zero.
               This LUT entry will accept all UDP packet for any source.
               Case 1) If the packet is non-secure UDP packet, packet will be sent to LUT2 for UDP port match
               Case 2) If the packet has UDP port == 4500, it will be sent to LUT2 for NATT handling, and then it
                       will be sent back to LUT1-1 for SPI matching
            */
            memcpy((void *)&origSACfg, (void *)ptrSACfg,  sizeof(Netfp_SACfg));

            if (ptrNetfpServer->nattCfg.wildCardedEntry == 1)
            {
                /* Set Src IP as wild carded address */
                if(ptrSACfg->srcIP.ver == Netfp_IPVersion_IPV4)
                    ptrSACfg->srcIP.addr.ipv4.u.a32 = 0;
                else
                {
                    ptrSACfg->srcIP.addr.ipv6.u.a32[0] = 0;
                    ptrSACfg->srcIP.addr.ipv6.u.a32[1] = 0;
                    ptrSACfg->srcIP.addr.ipv6.u.a32[2] = 0;
                    ptrSACfg->srcIP.addr.ipv6.u.a32[3] = 0;
                }
            }

            /* Find if the NATT has been created already */
            if ( (ptrNattIPInfo = Netfp_findNattIPChannel(ptrNetfpServer, ptrSACfg)) == NULL)
            {
                /* Create NAT-T UDP encap channel */
                if (Netfp_createNattIPChannel(ptrNetfpServer, ptrIPSecChannel, ptrSACfg, errCode) < 0)
                {
                    Netfp_cleanupSecChanAndPAVlink(ptrNetfpServer, ptrIPSecChannel, __LINE__);
                    ptrNetfpServer->cfg.free (ptrIPSecChannel, sizeof(Netfp_IPSecChannel));
                    return NULL;
                }
            }
            else
            {
                /* Inherit NAT-T UDP encap channel */
                if (Netfp_inheritNattIPChannel(ptrIPSecChannel, ptrNattIPInfo) < 0)
                {
                    *errCode = NETFP_EINTERNAL;
                    Netfp_cleanupSecChanAndPAVlink(ptrNetfpServer, ptrIPSecChannel, __LINE__);
                    ptrNetfpServer->cfg.free (ptrIPSecChannel, sizeof(Netfp_IPSecChannel));
                    return NULL;
                }
            }

            /* Sanity check of created/inheritted IPHandle for NAT-T IP Channel */
            if(ptrIPSecChannel->ptrNattIPInfo->nattLutInfo->ipHandle == NULL)
            {
                *errCode = NETFP_EINTERNAL;
                Netfp_cleanupSecChanAndPAVlink(ptrNetfpServer, ptrIPSecChannel, __LINE__);
                ptrNetfpServer->cfg.free (ptrIPSecChannel, sizeof(Netfp_IPSecChannel));
                return NULL;
            }

            /* Populate the IP Configuration. */
            memcpy((void *)ptrSACfg, (void *)&origSACfg, sizeof(Netfp_SACfg));

            ipLutCfg.ipCfg.dstIP     = ptrSACfg->dstIP;
            ipLutCfg.ipCfg.srcIP     = ptrSACfg->srcIP;
            ipLutCfg.ipCfg.spi       = ptrSACfg->spi;
            ipLutCfg.ipCfg.protocol  = IPPROTO_ESP;

            /* Populate the destination information */
            ipLutCfg.dstInfo.dstType = Netfp_DestType_LINKED_RULE;
            ipLutCfg.dstInfo.flowId  = Cppi_getFlowId(ptrNetfpServer->ipsecFlowHandle);

            /* Prepare IP Lut configuration */
            ipLutCfg.prevLinkHandle  = ptrIPSecChannel->ptrNattIPInfo->nattLutInfo->ipHandle;
            ipLutCfg.nextLinkHandle  = ptrIPSecChannel->ptrNetfpPAVlink->vlinkHandle;
            ipLutCfg.ptrIPSecChannel = ptrIPSecChannel;
            ipLutCfg.flags           = NETFP_IPCFG_FLAGS_NATT;

            /* Allocate LUT resources for the IPSEC channel: */
            ptrIPSecChannel->ptrSaLutInfo = Netfp_allocIPLutEntry (ptrNetfpServer, &ipLutCfg, 1, errCode);
            if (ptrIPSecChannel->ptrSaLutInfo == NULL )
            {
                Netfp_cleanupSecChanAndPAVlink(ptrNetfpServer, ptrIPSecChannel, __LINE__);
                ptrNetfpServer->cfg.free (ptrIPSecChannel, sizeof(Netfp_IPSecChannel));
                return NULL;
            }

            /* The INBOUND security association is ACTIVE */
            ptrIPSecChannel->status = Netfp_Status_ACTIVE;

            /* Record the security association in the server. */
            Netfp_listAdd ((Netfp_ListNode**)&ptrNetfpServer->ptrIPSecInboundChannels, (Netfp_ListNode*)ptrIPSecChannel);
            return (Netfp_SAHandle)ptrIPSecChannel;
        }
        else
        {
            *errCode = NETFP_EINVAL;
            Netfp_cleanupSecChanAndPAVlink(ptrNetfpServer, ptrIPSecChannel, __LINE__);
            ptrNetfpServer->cfg.free (ptrIPSecChannel, sizeof(Netfp_IPSecChannel));
            return NULL;
        }
    }

    /* Initialize the outbound IPSEC Channel PMTU */
    ptrIPSecChannel->ipsecPMTU = NETFP_INVALID_MTU;

    /* OUTBOUND: We need to ensure that we have a valid next hop MAC address. Is this a smart SA? */
    if (ptrSACfg->ifHandle != NULL)
    {
        /* YES. Copy over the resolution parameters: */
        ptrIPSecChannel->ptrNetfpInterface = (Netfp_Interface*)ptrSACfg->ifHandle;
        memcpy ((void*)&ptrIPSecChannel->nextHopMACAddress[0], (void*)&ptrSACfg->nextHopMACAddress[0], 6);

        /* The OUTBOUND security association is ACTIVE */
        ptrIPSecChannel->status = Netfp_Status_ACTIVE;
    }
    else
    {
        /* NO. We need to resolve the route so the security association is still not active */
        ptrIPSecChannel->status = Netfp_Status_ZOMBIE;

        /* Initialize the proxy server information, only the number to not set all array when only one item will be used */
        proxyServerOneMsg.numberOfEntries = 1;

        ptrProxyServerInfo = &proxyServerOneMsg.proxyServerInfo;

        /* Initialize the proxy server information */
        memset ((void *)ptrProxyServerInfo, 0, sizeof(Netfp_ProxyServerInfo));

        /* Populate the informational block: */
        ptrProxyServerInfo->opType = Netfp_ProxyServerOp_REQUEST;
        ptrProxyServerInfo->u.req.startMonitor = 1;
        memcpy ((void *)&ptrProxyServerInfo->dstIP, (void*)&ptrSACfg->dstIP, sizeof(Netfp_IPAddr));
        memcpy ((void *)&ptrProxyServerInfo->srcIP, (void*)&ptrSACfg->srcIP, sizeof(Netfp_IPAddr));

        *errCode = Netfp_populateServerToProxyNode(ptrNetfpServer, ptrProxyServerInfo, &ptrIPSecChannel->requestId);

        if (*errCode < 0)
        {
            Netfp_cleanupRequest(ptrNetfpServer, (Netfp_ProxyServerBulkMsg*)&proxyServerOneMsg);
        }
        else
        {
            /* Send a message to the NETFP Proxy to start monitoring: */
            *errCode = Netfp_sendProxyRequest (ptrNetfpServer, (Netfp_ProxyServerBulkMsg*)&proxyServerOneMsg);

            if (*errCode < 0)
            {
                Netfp_cleanupRequest(ptrNetfpServer, (Netfp_ProxyServerBulkMsg*)&proxyServerOneMsg);

                System_printf("Error, Netfp_sendProxyRequest failed %d, %s [LINE %d]\n", *errCode, __FUNCTION__, __LINE__);
            }
        }

        if (*errCode < 0)
        {
            ptrIPSecChannel->requestId = 0;

            /* Error: Route resolution failed */
            if(_Netfp_deleteSecurityChannel(ptrNetfpServer, ptrIPSecChannel->ptrSrvSecurityChannel, errCode) < 0)
                System_printf("Error while deleting SecChan [%d] (LINE:%d)", *errCode, __LINE__);

            ptrNetfpServer->cfg.free (ptrIPSecChannel, sizeof(Netfp_IPSecChannel));
            return NULL;
        }
    }

    /* Record the security association in the server. */
    Netfp_listAdd ((Netfp_ListNode**)&ptrNetfpServer->ptrIPSecOutboundChannels, (Netfp_ListNode*)ptrIPSecChannel);
    return (Netfp_SAHandle)ptrIPSecChannel;
}

/**
 *  @b Description
 *  @n
 *      The function is used to add a security association. Security associations
 *      will create an IPSEC channel in the NETCP subsystem.
 *
 *  @param[in]  clientHandle
 *      NETFP client handle
 *  @param[in]  ptrSAConfig
 *      Pointer to the security association configuration
 *  @param[out]  errCode
 *      Error code populated on error
 *
 * \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   Handle to the security association
 *  @retval
 *      Error   -   NULL
 */
Netfp_SAHandle Netfp_addSA(Netfp_ClientHandle clientHandle, Netfp_SACfg* ptrSAConfig, int32_t* errCode)
{
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    uint32_t                result;
    int32_t                 jobId;
    Netfp_ClientMCB*        ptrNetfpClient;
    Netfp_SACfg*            ptrRemoteSACfg;

    /* Sanity Check: Ensure that the arguments are valid */
    if ((clientHandle == NULL) || (ptrSAConfig == NULL))
    {
        *errCode = NETFP_EINVAL;
        return NULL;
    }

    /* Get the NETFP Client MCB: Services are available only if the client is active. */
    ptrNetfpClient = (Netfp_ClientMCB*)clientHandle;
    if (ptrNetfpClient->status != Netfp_ClientStatus_ACTIVE)
    {
        *errCode = NETFP_ENOTREADY;
        return NULL;
    }

    /* Get the actual job which we will execute */
    jobHandle = Josh_findJobByAddress(ptrNetfpClient->joshHandle, (Josh_JobProtype)_Netfp_addSA);
    if (jobHandle == NULL)
    {
        *errCode = NETFP_EINTERNAL;
        return NULL;
    }

    /* Initialize the arguments; to avoid any junk */
    memset ((void *)&args, 0, sizeof(args));

    /***************************************************************************
     * This is the function which is to be invoked:
     *
     *  Netfp_SAHandle _Netfp_addSA
     *  (
     *  Netfp_ServerMCB*        ptrNetfpServer,
     *  Netfp_SACfg*            ptrSAConfig,
     *  Netfp_IPSecChannel*     ptrOrigIPSecChannel,
     *  int32_t*                errCode
     *  )
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Netfp_ServerHandle);

    /*  - Argument 2: */
    args[1].type   = Josh_ArgumentType_PASS_BY_REF;
    args[1].length = sizeof(Netfp_SACfg);

    /*  - Argument 3: */
    args[2].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[2].length = sizeof(uint32_t);

    /*  - Argument 4: */
    args[3].type   = Josh_ArgumentType_PASS_BY_REF;
    args[3].length = sizeof(int32_t);

    /* Add the arguments. */
    argHandle = Josh_addArguments (ptrNetfpClient->joshHandle, &args[0]);
    if (argHandle == NULL)
    {
        *errCode = NETFP_EJOSH;
        return NULL;
    }

    /* Populate the NETFP Server handle */
    *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(ptrNetfpClient->cfg.serverHandle);

    /* Get the pointer to the SA configuration. */
    ptrRemoteSACfg = (Netfp_SACfg*)args[1].argBuffer;
    memcpy ((void*)ptrRemoteSACfg, (void *)ptrSAConfig, sizeof (Netfp_SACfg));

    /* To create SA from client side, alsways set the vlink to NULL */
    *(uint32_t*)args[2].argBuffer = (uint32_t)NULL;

    /* Submit the JOB to JOSH for execution. */
    jobId = Josh_submitJob(jobHandle, argHandle, &result, errCode);
    if (jobId < 0)
        return NULL;

    /* Get the result arguments. */
    if (Josh_getArguments (ptrNetfpClient->joshHandle, jobId, &args[0]) < 0)
    {
        *errCode = NETFP_EINTERNAL;
        return NULL;
    }

    /* Copy the error code value. */
    *errCode = *((int32_t*)args[3].argBuffer);

    /* Free the JOB Instance. */
    if (Josh_freeJobInstance(ptrNetfpClient->joshHandle, jobId) < 0)
    {
        *errCode = NETFP_EJOSH;
        return NULL;
    }
    return (Netfp_SAHandle)result;
}

//<fzm>
static void Netfp_generateDelSaEvent
(
    Netfp_ServerMCB*     ptrNetfpServer,
    Netfp_IPSecChannel*  ptrIPSecChannel,
    const Netfp_IPAddr*  dstIP,
    uint32_t             spi
)
{
    Netfp_EventMetaInfo eventInfo;
    memset(&eventInfo, 0, sizeof(eventInfo));

    eventInfo.eventId        = Netfp_EventId_DEL_SW_OFFLOAD_SA;
    eventInfo.u.saMeta.spi   = spi;
    memcpy((void*)&eventInfo.u.saMeta.dstIP, dstIP, sizeof(eventInfo.u.saMeta.dstIP));

    int32_t errCode = 0;
    if(Netfp_addPendingSW_SA(ptrNetfpServer, ptrIPSecChannel, Netfp_EventId_DEL_SW_OFFLOAD_SA, &eventInfo.u.saMeta.jobID, &errCode) < 0)
    {
        System_printf("Error while adding SW_SA pending job [%d]", errCode);
        return;
    }

    Netfp_generateEvent(ptrNetfpServer, &eventInfo);
}

static void Netfp_updatePendingSW_SA
(
    Netfp_ServerMCB*        ptrNetfpServer,
    Netfp_IPSecChannel*     ptrIPSecChannel
)
{
    Netfp_PendingSW_SAOffloadNode* pendingSa = (Netfp_PendingSW_SAOffloadNode*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrPendingSW_SAOffloadList);
    while (pendingSa != NULL)
    {
        if(pendingSa->ptrIPSecChannel == ptrIPSecChannel)
            pendingSa->ptrIPSecChannel = NULL;

        pendingSa = (Netfp_PendingSW_SAOffloadNode*)Netfp_listGetNext ((Netfp_ListNode*)pendingSa);
    }
}

//</fzm>

/**
 *  @b Description
 *  @n
 *      The function is used to delete the LUT1-1 entry associated with an INBOUND
 *      IPSEC channel
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  ptrIPSecChannel
 *      Pointer to the IPSEC Channel
 *  @param[out]  errCode
 *      Error code populated on error
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Netfp_delIPSecChannelLUTEntry
(
    Netfp_ServerMCB*        ptrNetfpServer,
    Netfp_IPSecChannel*     ptrIPSecChannel,
    int32_t*                errCode
)
{
    /* LUT Entries are associated with INBOUND IPSEC Channels. */
    if (ptrIPSecChannel->saCfg.direction == Netfp_Direction_OUTBOUND)
        return 0;

    /* Delete NAT-T channel if enabled */
    if (ptrIPSecChannel->ptrNattIPInfo)
    {
        if (Netfp_deleteNattIPChannel(ptrNetfpServer, ptrIPSecChannel->ptrNattIPInfo, errCode) < 0)
            return -1;

        /* Reset NAT-T IP handle */
        ptrIPSecChannel->ptrNattIPInfo = NULL;
    }

//fzm-->
    if(ptrIPSecChannel->ptrSwIPInfo)
    {
        Netfp_generateDelSaEvent(ptrNetfpServer, ptrIPSecChannel, &ptrIPSecChannel->ptrSwIPInfo->ipCfg.dstIP, ptrIPSecChannel->saCfg.spi);

        if (Netfp_deleteSwLutIPChannel(ptrNetfpServer, ptrIPSecChannel->ptrSwIPInfo, errCode) < 0)
            return -1;

        ptrIPSecChannel->ptrSwIPInfo = NULL;
    }

    if(ptrIPSecChannel->ptrNattSwIPInfo)
    {
        Netfp_generateDelSaEvent(ptrNetfpServer, ptrIPSecChannel, &ptrIPSecChannel->ptrNattSwIPInfo->ipCfg.dstIP, ptrIPSecChannel->saCfg.spi);

        if (Netfp_deleteNattSwIPChannel(ptrNetfpServer, ptrIPSecChannel->ptrNattSwIPInfo, errCode) < 0)
            return -1;

        ptrIPSecChannel->ptrNattSwIPInfo = NULL;
    }
//<--fzm

    /* Do we have a valid LUT entry to clean up? The LUT entry could have already been deleted */
    if (ptrIPSecChannel->ptrSaLutInfo != NULL)
    {
        /* Free LUT resource */
        *errCode = Netfp_freeIPLutEntry (ptrNetfpServer, ptrIPSecChannel->ptrSaLutInfo);
        if (*errCode < 0)
            return -1;

        /* The LUT entry has been cleaned out. */
        ptrIPSecChannel->ptrSaLutInfo = NULL;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to stop the IPSEC channel. This will cause the infection
 *      to spread to the security policies, fast paths and sockets. This however does
 *      not delete the other entities.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  saHandle
 *      SA Handle to be stopped
 *  @param[out]  errCode
 *      Error code populated on error
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t _Netfp_stopSA
(
    Netfp_ServerMCB*        ptrNetfpServer,
    Netfp_SAHandle          saHandle,
    int32_t*                errCode
)
{
    Netfp_IPSecChannel*     ptrIPSecChannel;

    /* Get the IPSEC Channel: */
    ptrIPSecChannel = (Netfp_IPSecChannel*)saHandle;
    if (ptrIPSecChannel == NULL)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    ptrIPSecChannel->status = Netfp_Status_STOP; //fzm

    /* Update the security policy to point to the NULL SA because the SA has been stopped. */
    Netfp_updateSP (ptrNetfpServer, Netfp_Reason_SP_INACTIVE, (Netfp_SAHandle)ptrIPSecChannel, (Netfp_SAHandle)NULL);

    /* Delete the LUT Entry associated with the IPSEC channel */
    return Netfp_delIPSecChannelLUTEntry (ptrNetfpServer, ptrIPSecChannel, errCode);
}

/**
 *  @b Description
 *  @n
 *      The function is used to stop a security association. Stopping a security association
 *      will cause the security policy, fast paths and sockets to become INACTIVE. The security
 *      association still needs to be deleted by the application.
 *
 *  @param[in]  clientHandle
 *      NETFP client handle
 *  @param[in]  saHandle
 *      Handle to the security association to be stopped
 *  @param[out]  errCode
 *      Error code populated on error
 *
 * \ingroup NETFP_FUNCTION
 *
 * @sa
 *  Netfp_removeSA
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_stopSA
(
    Netfp_ClientHandle  clientHandle,
    Netfp_SAHandle      saHandle,
    int32_t*            errCode
)
{
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    uint32_t                result;
    int32_t                 jobId;
    Netfp_ClientMCB*        ptrNetfpClient;

    /* Sanity Check: Ensure that the arguments are valid */
    if ((clientHandle == NULL) || (saHandle == NULL))
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Get the NETFP Client MCB: Services are available only if the client is active. */
    ptrNetfpClient = (Netfp_ClientMCB*)clientHandle;
    if (ptrNetfpClient->status != Netfp_ClientStatus_ACTIVE)
    {
        *errCode = NETFP_ENOTREADY;
        return -1;
    }

    /* Get the actual job which we will execute */
    jobHandle = Josh_findJobByAddress(ptrNetfpClient->joshHandle, (Josh_JobProtype)_Netfp_stopSA);
    if (jobHandle == NULL)
    {
        *errCode = NETFP_EINTERNAL;
        return -1;
    }

    /* Initialize the arguments; to avoid any junk */
    memset ((void *)&args, 0, sizeof(args));

    /***************************************************************************
     * This is the function which is to be invoked:
     *
     * int32_t _Netfp_stopSA
     * (
     *  Netfp_ServerMCB*    ptrNetfpServer,
     *  Netfp_SAHandle      saHandle,
     *  int32_t*            errCode
     * )
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Netfp_ServerHandle);

    /*  - Argument 2: */
    args[1].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[1].length = sizeof(Netfp_SAHandle);

    /*  - Argument 3: */
    args[2].type   = Josh_ArgumentType_PASS_BY_REF;
    args[2].length = sizeof(int32_t);

    /* Add the arguments. */
    argHandle = Josh_addArguments (ptrNetfpClient->joshHandle, &args[0]);
    if (argHandle == NULL)
    {
        *errCode = NETFP_EJOSH;
        return -1;
    }

    /* Populate the arguments */
    *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(ptrNetfpClient->cfg.serverHandle);
    *(uint32_t*)args[1].argBuffer = (uint32_t)josh_toRemoteU32(saHandle);

    /* Submit the JOB to JOSH for execution. */
    jobId = Josh_submitJob(jobHandle, argHandle, &result, errCode);
    if (jobId < 0)
        return -1;

    /* Get the result arguments. */
    if (Josh_getArguments (ptrNetfpClient->joshHandle, jobId, &args[0]) < 0)
    {
        *errCode = NETFP_EINTERNAL;
        return -1;
    }

    /* Copy the error code value. */
    *errCode = *((int32_t*)args[2].argBuffer);

    /* Free the JOB Instance. */
    if (Josh_freeJobInstance(ptrNetfpClient->joshHandle, jobId) < 0)
    {
        *errCode = NETFP_EJOSH;
        return -1;
    }
    return (int32_t)result;
}

/**
 *  @b Description
 *  @n
 *      The function is used to delete the IPSEC channel. The function is used
 *      to delete both inbound and outbound IPSEC channels
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  saHandle
 *      Pointer to the IPSEC channel to be deleted
 *  @param[in]  reason
 *      Reason because of which the function was invoked
 *  @param[out]  errCode
 *      Error code populated on error
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      0    - Inbound IPSEC Channel deleted successfully
 *  @retval
 *      <0   - Inbound/Outbound IPSEC Channel deletion failed
 *  @retval
 *      1    - Outbound IPSEC Channel deletion has been initiated
 *  @retval
 *      0    - Outbound IPSEC Channel deletion was completed.
 */
static int32_t _Netfp_delSA
(
    Netfp_ServerMCB*        ptrNetfpServer,
    Netfp_SAHandle          saHandle,
    Netfp_Reason            reason,
    int32_t*                errCode
)
{
    Netfp_ProxyServerInfo*  ptrProxyServerInfo; 
    Netfp_ProxyServerOneMsg proxyServerOneMsg;
    Netfp_IPSecChannel*     ptrIPSecChannel;
    uint32_t                ipSecChannelFound = 0; //fzm

    /* Get the IPSEC channel handle */
    ptrIPSecChannel = (Netfp_IPSecChannel *)saHandle;
//fzm-->
    do
    {
        Netfp_IPSecChannel* tmpIPSecChannel = (Netfp_IPSecChannel*)Netfp_listGetHead((Netfp_ListNode**)&ptrNetfpServer->ptrIPSecInboundChannels);
        while (tmpIPSecChannel != NULL)
        {
            if (ptrIPSecChannel == tmpIPSecChannel)
            {
                ipSecChannelFound = 1;
                break;
            }
            tmpIPSecChannel = (Netfp_IPSecChannel*)Netfp_listGetNext((Netfp_ListNode*)tmpIPSecChannel);
        }

        if (ipSecChannelFound == 1)
            break;

        tmpIPSecChannel = (Netfp_IPSecChannel*)Netfp_listGetHead((Netfp_ListNode**)&ptrNetfpServer->ptrIPSecOutboundChannels);
        while (tmpIPSecChannel != NULL)
        {
            if (ptrIPSecChannel == tmpIPSecChannel)
            {
                ipSecChannelFound = 1;
                break;
            }
            tmpIPSecChannel = (Netfp_IPSecChannel*)Netfp_listGetNext((Netfp_ListNode*)tmpIPSecChannel);
        }
    } while (0);

    if(ipSecChannelFound == 0)
    {
        Netfp_logMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "SA %p does not exist on the list, exiting without error", ptrIPSecChannel);
        return 0;
    }
//fzm<--

    /* Infect the security policy since the SA has been deleted and so any security policy which uses this
     * security association is also dead and can no longer be used. We invoke this first to ensure that the
     * fast paths which use the ingress LUT1-2 handle are killed off before we kill the LUT1-1 handles */
    Netfp_updateSP (ptrNetfpServer, reason, ptrIPSecChannel, ptrIPSecChannel);

    /* Are we deleting an INBOUND IPSEC channel? */
    if (ptrIPSecChannel->saCfg.direction == Netfp_Direction_INBOUND)
    {
        /* Delete the IPSEC Channel LUT Entry: */
        if (Netfp_delIPSecChannelLUTEntry (ptrNetfpServer, ptrIPSecChannel, errCode) < 0)
            return -1;

        /* Delete the PA virtual link which has been created for IPSec channel */
        if (Netfp_deletePAVlink(ptrNetfpServer, ptrIPSecChannel->ptrNetfpPAVlink, errCode) < 0)
            return -1;

        /* Queue the server security channel from the SA subsystem after a delay. The delay is a safety
         * mechanism primarily for egress security contexts, but we add the same delay for ingress
         * contexts out of an abundance of caution
         */
        if (Netfp_deleteSecurityChannelSlow(ptrNetfpServer, ptrIPSecChannel->ptrSrvSecurityChannel, errCode) < 0)
            return -1;

        /* Remove the channel from the INBOUND database. */
        Netfp_listRemoveNode ((Netfp_ListNode**)&ptrNetfpServer->ptrIPSecInboundChannels, (Netfp_ListNode*)ptrIPSecChannel);

        /* Remove the channel from pending SW_SA offload list */
        Netfp_updatePendingSW_SA (ptrNetfpServer, ptrIPSecChannel); //fzm

        /* Cleanup the allocated memory block. */
        ptrNetfpServer->cfg.free (ptrIPSecChannel, sizeof(Netfp_IPSecChannel));
        return 0;
    }

    /* OUTBOUND SA is being deleted: Was it using the manual route override mode? */
    if (ptrIPSecChannel->saCfg.ifHandle == NULL)
    {
        /* NO: We need to inform NETFP Proxy to stop monitoring the end point */
        if(!ptrIPSecChannel->recomputationDoneOnStopped) //fzm
        {
            /* Initialize the proxy server information, only the number to not set all array when only one item will be used */
            proxyServerOneMsg.numberOfEntries = 1;

            ptrProxyServerInfo = &proxyServerOneMsg.proxyServerInfo;

            /* Initialize the proxy server information */
            memset ((void *)ptrProxyServerInfo, 0, sizeof(Netfp_ProxyServerInfo));

            /* Populate the informational block: */
            ptrProxyServerInfo->opType = Netfp_ProxyServerOp_REQUEST;
            ptrProxyServerInfo->u.req.startMonitor = 0;
            memcpy ((void *)&ptrProxyServerInfo->dstIP, (void*)&ptrIPSecChannel->saCfg.dstIP, sizeof(Netfp_IPAddr));
            memcpy ((void *)&ptrProxyServerInfo->srcIP, (void*)&ptrIPSecChannel->saCfg.srcIP, sizeof(Netfp_IPAddr));

            *errCode = Netfp_populateServerToProxyNode(ptrNetfpServer, ptrProxyServerInfo, &ptrIPSecChannel->requestId);

            if (*errCode < 0)
            {
                Netfp_cleanupRequest(ptrNetfpServer, (Netfp_ProxyServerBulkMsg*)&proxyServerOneMsg);
            }
            else
            {
                /* Send a message to the NETFP Proxy to stop monitoring: */
                *errCode = Netfp_sendProxyRequest (ptrNetfpServer, (Netfp_ProxyServerBulkMsg*)&proxyServerOneMsg);

                if (*errCode < 0)
                {
                    ptrIPSecChannel->requestId =0;

                    Netfp_cleanupRequest(ptrNetfpServer, (Netfp_ProxyServerBulkMsg*)&proxyServerOneMsg);

                    System_printf("Error, Netfp_sendProxyRequest failed %d, %s [LINE %d]\n", *errCode, __FUNCTION__, __LINE__);
                }
            }
        }
        else // fzm: server tried to recompute route to stopped SA and the action was blocked - route doesn't exist in proxy, just continue
        {
            Netfp_logMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "Channel %p was stopped and recomputed, no need to stopMonitoring", ptrIPSecChannel);
            *errCode = 0;
        }
    }
    else
    {
        /* YES: Manual override mode; no need to inform the NETFP Proxy */
        *errCode = 0;
    }

    /* Irrespective of the error code; the SA is deleted from the NETFP Server. Worst case: The proxy will
     * be monitoring a SA which does not exist. Sending a proxy request can only fail because of a JOSH error
     * which should never occur. Log the error message */
    if (*errCode < 0)
        Netfp_logMsg (ptrNetfpServer, Netfp_LogLevel_ERROR, "Error: Sending Proxy Request on Delete SA failed [Error code %d]\n", *errCode);

    /* Queue the server security channel from the SA subsystem after a delay. The delay is a safety
     * mechanism to ensure that any preceding rekey has propagated to the client long before the
     * context is removed from the SA.  If packets are still being sent to the SA with a deleted
     * security context, errors and even SA lockup can occur.
     */
    if (Netfp_deleteSecurityChannelSlow(ptrNetfpServer, ptrIPSecChannel->ptrSrvSecurityChannel, errCode) < 0)
        return -1;

    /* Cleanup the security association */
    Netfp_listRemoveNode ((Netfp_ListNode**)&ptrNetfpServer->ptrIPSecOutboundChannels, (Netfp_ListNode*)ptrIPSecChannel);
    Netfp_removeSAFromRecomputationList(ptrNetfpServer, ptrIPSecChannel);
    ptrNetfpServer->cfg.free (ptrIPSecChannel, sizeof(Netfp_IPSecChannel));
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to delete a security association. Deleting a security
 *      association will cause the security policy, fast path and associated sockets
 *      to be marked as inactive and will be deleted.
 *
 *  @param[in]  clientHandle
 *      NETFP client handle
 *  @param[in]  saHandle
 *      Handle to the security association to be deleted
 *  @param[out]  errCode
 *      Error code populated on error
 *
 * \ingroup NETFP_FUNCTION
 *
 * @sa
 *  Netfp_removeSA
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_delSA (Netfp_ClientHandle clientHandle, Netfp_SAHandle saHandle, int32_t* errCode)
{
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    uint32_t                result;
    int32_t                 jobId;
    Netfp_ClientMCB*        ptrNetfpClient;

    /* Sanity Check: Ensure that the arguments are valid */
    if ((clientHandle == NULL) || (saHandle == NULL))
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Get the NETFP Client MCB: Services are available only if the client is active. */
    ptrNetfpClient = (Netfp_ClientMCB*)clientHandle;
    if (ptrNetfpClient->status != Netfp_ClientStatus_ACTIVE)
    {
        *errCode = NETFP_ENOTREADY;
        return -1;
    }

    /* Get the actual job which we will execute */
    jobHandle = Josh_findJobByAddress(ptrNetfpClient->joshHandle, (Josh_JobProtype)_Netfp_delSA);
    if (jobHandle == NULL)
    {
        *errCode = NETFP_EINTERNAL;
        return -1;
    }

    /* Initialize the arguments; to avoid any junk */
    memset ((void *)&args, 0, sizeof(args));

    /***************************************************************************
     * This is the function which is to be invoked:
     *
     * int32_t _Netfp_delSA
     * (
     *  Netfp_ServerMCB*    ptrNetfpServer,
     *  Netfp_SAHandle      saHandle,
     *  Netfp_Reason        reason,
     *  int32_t*            errCode
     * )
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Netfp_ServerHandle);

    /*  - Argument 2: */
    args[1].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[1].length = sizeof(Netfp_SAHandle);

    /*  - Argument 3: */
    args[2].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[2].length = sizeof(Netfp_Reason);

    /*  - Argument 4: */
    args[3].type   = Josh_ArgumentType_PASS_BY_REF;
    args[3].length = sizeof(int32_t);

    /* Add the arguments. */
    argHandle = Josh_addArguments (ptrNetfpClient->joshHandle, &args[0]);
    if (argHandle == NULL)
    {
        *errCode = NETFP_EJOSH;
        return -1;
    }

    /* Populate the arguments */
    *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(ptrNetfpClient->cfg.serverHandle);
    *(uint32_t*)args[1].argBuffer = (uint32_t)josh_toRemoteU32(saHandle);
    *(uint32_t*)args[2].argBuffer = (uint32_t)josh_toRemoteU32(Netfp_Reason_SA_DELETE);

    /* Submit the JOB to JOSH for execution. */
    jobId = Josh_submitJob(jobHandle, argHandle, &result, errCode);
    if (jobId < 0)
        return -1;

    /* Get the result arguments. */
    if (Josh_getArguments (ptrNetfpClient->joshHandle, jobId, &args[0]) < 0)
    {
        *errCode = NETFP_EINTERNAL;
        return -1;
    }

    /* Copy the error code value. */
    *errCode = *((int32_t*)args[3].argBuffer);

    /* Free the JOB Instance. */
    if (Josh_freeJobInstance(ptrNetfpClient->joshHandle, jobId) < 0)
    {
        *errCode = NETFP_EJOSH;
        return -1;
    }

    /* Outbound IPSEC Channel deletion requires a notification to the NETFP Proxy; there are
     * two possible return values which imply if the IPSEC has been deleted immediately or it
     * will be deleted after the proxy stops the monitoring. These are for internal NETFP
     * list management and from the applications perspective the IPSEC has been deleted
     * successfully. */
    if ((int32_t)result < 0)
        return -1;
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to handle the rekeying of an existing IPSEC Channel. This
 *      will ensure that the Outer IP PA Link is shared between the old and new IPSEC
 *      channels.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  oldSAHandle
 *      SA Handle which is being rekeyed.
 *  @param[in]  ptrSACfg
 *      Pointer to the security association configuration
 *  @param[out]  errCode
 *      Error code populated on error
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   Handle to the security association
 *  @retval
 *      Error   -   NULL
 */
static Netfp_SAHandle _Netfp_rekeySA
(
    Netfp_ServerMCB*        ptrNetfpServer,
    Netfp_SAHandle          oldSAHandle,
    Netfp_SACfg*            ptrSACfg,
    int32_t*                errCode
)
{
    Netfp_IPSecChannel*     ptrIPSecChannel;
    Netfp_IPSecChannel*     ptrOrigIPSecChannel;

    /* Get the original IPSEC Channel which is being rekeyed. */
    ptrOrigIPSecChannel = (Netfp_IPSecChannel*)oldSAHandle;
    if (ptrOrigIPSecChannel == NULL)
    {
        *errCode = NETFP_EINVAL;
        return NULL;
    }

    /* Create a new IPSEC channel but we inherit the link from the older IPSEC Channel: */
    ptrIPSecChannel = (Netfp_IPSecChannel*)_Netfp_addSA(ptrNetfpServer, ptrSACfg, ptrOrigIPSecChannel, errCode);
    if (ptrIPSecChannel == NULL)
        return NULL;

    /* Inherit the status of the original outbound IPSEC Channel: This will ensure that after rekeying the new
     * IPSEC channel is not marked as a ZOMBIE especially if the SA is to the same security gateway. */
    if (ptrSACfg->direction == Netfp_Direction_OUTBOUND)
    {
        /* Was either the old or new SA configuration operating in the smart mode? */
        //fzm
        if ((ptrSACfg->ifHandle == NULL) && (ptrOrigIPSecChannel->saCfg.ifHandle == NULL)
                && (ptrOrigIPSecChannel->status != Netfp_Status_STOP))
        {
            /* NO: None of the channels were operating in smart mode? We can inherit the status only if the
             * old & new IPSEC channels are to the same gateway. */
            if ((Netfp_matchIP (&ptrOrigIPSecChannel->saCfg.dstIP, &ptrIPSecChannel->saCfg.dstIP) == 1) &&
                (Netfp_matchIP (&ptrOrigIPSecChannel->saCfg.srcIP, &ptrIPSecChannel->saCfg.srcIP) == 1))
            {
                /* YES: IPSEC Channels are to the same gateway. Inherit the status. */
                ptrIPSecChannel->status            = ptrOrigIPSecChannel->status;
                ptrIPSecChannel->ptrNetfpInterface = ptrOrigIPSecChannel->ptrNetfpInterface;
                memcpy ((void*)&ptrIPSecChannel->nextHopMACAddress[0], (void*)&ptrOrigIPSecChannel->nextHopMACAddress[0], 6);
            }
        }
    }

    /* Update the security policy to point to the new SA */
    Netfp_updateSP (ptrNetfpServer, Netfp_Reason_REKEY_SA, (Netfp_SAHandle)ptrOrigIPSecChannel, (Netfp_SAHandle)ptrIPSecChannel);
    return (Netfp_SAHandle)ptrIPSecChannel;
}

/**
 *  @b Description
 *  @n
 *      The function is used to handle the rekeying of an existing IPSEC
 *      channel.
 *
 *  @param[in]  clientHandle
 *      NETFP client handle
 *  @param[in]  oldSAHandle
 *      Old SA Handle which is being rekeyed.
 *  @param[in]  ptrSAConfig
 *      Pointer to the new security association configuration which
 *      is to be used
 *  @param[out]  errCode
 *      Error code populated on error
 *
 *  \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   Handle to the new security association
 *  @retval
 *      Error   -   NULL
 */
Netfp_SAHandle Netfp_rekeySA
(
    Netfp_ClientHandle      clientHandle,
    Netfp_SAHandle          oldSAHandle,
    Netfp_SACfg*            ptrSAConfig,
    int32_t*                errCode
)
{
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    uint32_t                result;
    int32_t                 jobId;
    Netfp_ClientMCB*        ptrNetfpClient;
    Netfp_SACfg*            ptrRemoteSACfg;

    /* Sanity Check: Ensure that the arguments are valid */
    if ((clientHandle == NULL) || (oldSAHandle == NULL) || (ptrSAConfig == NULL))
    {
        *errCode = NETFP_EINVAL;
        return NULL;
    }

    /* Get the NETFP Client MCB: Services are available only if the client is active. */
    ptrNetfpClient = (Netfp_ClientMCB*)clientHandle;
    if (ptrNetfpClient->status != Netfp_ClientStatus_ACTIVE)
    {
        *errCode = NETFP_ENOTREADY;
        return NULL;
    }

    /* Sanity Check: Ensure that the src & dst ip belong to the same family. */
    if (ptrSAConfig->dstIP.ver != ptrSAConfig->srcIP.ver)
    {
        *errCode = NETFP_EINVAL;
        return NULL;
    }

    /* Get the actual job which we will execute */
    jobHandle = Josh_findJobByAddress(ptrNetfpClient->joshHandle, (Josh_JobProtype)_Netfp_rekeySA);
    if (jobHandle == NULL)
    {
        *errCode = NETFP_EINTERNAL;
        return NULL;
    }

    /* Initialize the arguments; to avoid any junk */
    memset ((void *)&args, 0, sizeof(args));

    /***************************************************************************
     * This is the function which is to be invoked:
     *
     *  Netfp_SAHandle _Netfp_rekeySA
     *  (
     *  Netfp_ServerMCB*        ptrNetfpServer,
     *  Netfp_SAHandle          oldSAHandle,
     *  Netfp_SACfg*            ptrSACfg,
     *  int32_t*                errCode
     *  )
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Netfp_ServerHandle);

    /*  - Argument 2: */
    args[1].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[1].length = sizeof(Netfp_SAHandle);

    /*  - Argument 3: */
    args[2].type   = Josh_ArgumentType_PASS_BY_REF;
    args[2].length = sizeof(Netfp_SACfg);

    /*  - Argument 4: */
    args[3].type   = Josh_ArgumentType_PASS_BY_REF;
    args[3].length = sizeof(int32_t);

    /* Add the arguments. */
    argHandle = Josh_addArguments (ptrNetfpClient->joshHandle, &args[0]);
    if (argHandle == NULL)
    {
        *errCode = NETFP_EJOSH;
        return NULL;
    }

    /* Populate the NETFP Server handle */
    *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(ptrNetfpClient->cfg.serverHandle);

    /* Populate the Old SA channel handle */
    *(uint32_t*)args[1].argBuffer = (uint32_t)josh_toRemoteU32(oldSAHandle);

    /* Get the pointer to the SA configuration. */
    ptrRemoteSACfg = (Netfp_SACfg*)args[2].argBuffer;
    memcpy ((void*)ptrRemoteSACfg, (void *)ptrSAConfig, sizeof (Netfp_SACfg));

    /* Submit the JOB to JOSH for execution. */
    jobId = Josh_submitJob(jobHandle, argHandle, &result, errCode);
    if (jobId < 0)
        return NULL;

    /* Get the result arguments. */
    if (Josh_getArguments (ptrNetfpClient->joshHandle, jobId, &args[0]) < 0)
    {
        *errCode = NETFP_EINTERNAL;
        return NULL;
    }

    /* Copy the error code value. */
    *errCode = *((int32_t*)args[3].argBuffer);

    /* Free the JOB Instance. */
    if (Josh_freeJobInstance(ptrNetfpClient->joshHandle, jobId) < 0)
    {
        *errCode = NETFP_EJOSH;
        return NULL;
    }
    return (Netfp_SAHandle)result;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the SA IPSec statistics. This function is execurted from NetFP
 *   server.
 *
 *  @param[in]  ptrNetfpServer
 *      Handle to Netfp server
 *  @param[in]  ptrIPSecChannel
 *      Pointer to the IPsec server channel.
 *  @param[out]  ipsecStats
 *      Pointer to the ipsec statistics structure.
 *  @param[out]  errCode
 *      Error code populated on error
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t _Netfp_getIPsecStats
(
    Netfp_ServerMCB*        ptrNetfpServer,
    Netfp_IPSecChannel*     ptrIPSecChannel,
    Netfp_IpSecStats*       ipsecStats,
    int32_t*                errCode
)
{
    Netfp_SrvSecurityChannel*   ptrSrvSecurityChannel;
    Sa_Stats_t                  saStats;

    /* Sanity check the Security channel handle. */
    ptrSrvSecurityChannel = ptrIPSecChannel->ptrSrvSecurityChannel;
    if (ptrSrvSecurityChannel == NULL )
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }
    /* Only get the IPsec channel statistics: */
    if (ptrSrvSecurityChannel->netfpProtocol != Netfp_SaProtocol_IPSEC)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Get the SA Statistics from the SA */
    *errCode = Netfp_getSAStatistics (ptrSrvSecurityChannel->saChannelHandle, &saStats);
    if (*errCode < 0)
    {
        /* Error: Unable to get the SA Channel statistics: */
        Netfp_logMsg (ptrNetfpServer, Netfp_LogLevel_ERROR,
                      "SA Channel 0x%x getStats failed [Error code %d]\n", ptrSrvSecurityChannel->saChannelHandle, errCode);

        return NETFP_EINTERNAL;
    }

    /* Save the Ipsec statistics */
    ipsecStats->saHandle = ptrIPSecChannel;
    ipsecStats->spi      = ptrIPSecChannel->saCfg.spi;

    if (ptrIPSecChannel->saCfg.direction == Netfp_Direction_INBOUND)
    {
        ipsecStats->u.in.inIpsecBytes             = ((uint64_t)saStats.ipsec.rxByteCountHi << 32) + (uint64_t)saStats.ipsec.rxByteCountLo;
        ipsecStats->u.in.inIpsecPkts              = ((uint64_t)saStats.ipsec.pktDecHi << 32) + (uint64_t)saStats.ipsec.pktDecLo;
        ipsecStats->u.in.inIpsecDiscReplayFail    = saStats.ipsec.replayOld + saStats.ipsec.replayDup;
        ipsecStats->u.in.inIpsecDiscIntegrityFail = saStats.ipsec.authFail;
    }
    else
    {
        ipsecStats->u.out.outIpsecDiscSeqOv       = saStats.ipsec.txRollover;
        ipsecStats->u.out.outIpsecBytes           = ((uint64_t)saStats.ipsec.txByteCountHi << 32) + ((uint64_t)saStats.ipsec.txByteCountLo);
        ipsecStats->u.out.outIpsecPkts            = ((uint64_t)saStats.ipsec.pktEncHi << 32) + ((uint64_t)saStats.ipsec.pktEncLo);
    }

    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the SA IPSec statistics. This function is execurted from NetFP
 *   server.
 *
 *  @param[in]  clientHandle
 *      Netfp client handle
 *  @param[in]  saHandle
 *      Handle to security association to get IPSec stats
 *  @param[out]  ipsecStats
 *      Pointer to the IPSec statistics structure
 *  @param[out]  errCode
 *      Error code populated on error
 *
 *  \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_getIPsecStats (Netfp_ClientHandle clientHandle, Netfp_SAHandle saHandle, Netfp_IpSecStats* ipsecStats, int32_t* errCode)
{
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    uint32_t                result;
    int32_t                 jobId;
    Netfp_ClientMCB*        ptrNetfpClient;

    /* Sanity Check: Ensure that the arguments are valid */
    if ((clientHandle == NULL) || (saHandle == NULL))
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Get the NETFP Client MCB: Services are available only if the client is active. */
    ptrNetfpClient = (Netfp_ClientMCB*)clientHandle;
    if (ptrNetfpClient->status != Netfp_ClientStatus_ACTIVE)
    {
        *errCode = NETFP_ENOTREADY;
        return -1;
    }

    /* Get the actual job which we will execute */
    jobHandle = Josh_findJobByAddress(ptrNetfpClient->joshHandle, (Josh_JobProtype)_Netfp_getIPsecStats);
    if (jobHandle == NULL)
    {
        *errCode = NETFP_EINTERNAL;
        return -1;
    }

    /* Initialize the arguments; to avoid any junk */
    memset ((void *)&args, 0, sizeof(args));

    /***************************************************************************
     * This is the function which is to be invoked:
     *
     * int32_t _Netfp_getIPsecStats
     * (
     *  Netfp_ServerMCB*    ptrNetfpServer,
     *  Netfp_SAHandle      saHandle,
     *  Netfp_IpSecStats    ipsecStats,
     *  int32_t*            errCode
     * )
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Netfp_ServerHandle);

    /*  - Argument 2: */
    args[1].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[1].length = sizeof(Netfp_SAHandle);

    /*  - Argument 3: */
    args[2].type   = Josh_ArgumentType_PASS_BY_REF;
    args[2].length = sizeof(Netfp_IpSecStats);

    /*  - Argument 4: */
    args[3].type   = Josh_ArgumentType_PASS_BY_REF;
    args[3].length = sizeof(int32_t);

    /* Add the arguments. */
    argHandle = Josh_addArguments (ptrNetfpClient->joshHandle, &args[0]);
    if (argHandle == NULL)
    {
        *errCode = NETFP_EJOSH;
        return -1;
    }

    /* Populate the arguments */
    *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(ptrNetfpClient->cfg.serverHandle);
    *(uint32_t*)args[1].argBuffer = (uint32_t)josh_toRemoteU32(saHandle);

    /* Submit the JOB to JOSH for execution. */
    jobId = Josh_submitJob(jobHandle, argHandle, &result, errCode);
    if (jobId < 0)
        return -1;

    /* Get the result arguments. */
    if (Josh_getArguments (ptrNetfpClient->joshHandle, jobId, &args[0]) < 0)
    {
        *errCode = NETFP_EINTERNAL;
        return -1;
    }

    /* Copy the IPsec stats returned from server */
    memcpy((void *)ipsecStats, (void *)args[2].argBuffer,sizeof(Netfp_IpSecStats));

    /* Copy the error code value. */
    *errCode = *((int32_t*)args[3].argBuffer);

    /* Free the JOB Instance. */
    if (Josh_freeJobInstance(ptrNetfpClient->joshHandle, jobId) < 0)
    {
        *errCode = NETFP_EJOSH;
        return -1;
    }
    return (int32_t)result;
}

//fzm-->
/**
 *  @b Description
 *  @n
 *      The function is used to handle the change in the root SA associated with
 *      an SP.  Two SAs might exist for one SP.  However, if the root (or primary)
 *      is deleted, the SP is deleted too.  If an SA is to be deleted and the
 *      SP is to remain with another SA, then the other SA must be made root first.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  oldSAHandle
 *      The current root SA for the SP.
 *  @param[in]  newSAHandle
 *      The new root SA for the SP.
 *  @param[out]  errCode
 *      Error code populated on error
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   Handle to the root security association
 *  @retval
 *      Error   -   NULL
 */
static Netfp_SAHandle _Netfp_changeRootSA
(
    Netfp_ServerMCB*        ptrNetfpServer,
    Netfp_SAHandle          oldSAHandle,
    Netfp_SAHandle          newSAHandle,
    int32_t*                errCode
)
{
    /* Update the security policy use the new root SA instead of the old one */
    Netfp_updateSP(ptrNetfpServer, Netfp_Reason_REKEY_SA, oldSAHandle, newSAHandle);

    return(newSAHandle);
}

/**
 *  @b Description
 *  @n
 *      The function is used to change the root (or Primary) IP Sec
 *      channel for a Security policy.
 *
 *  @param[in]  clientHandle
 *      NETFP client handle
 *  @param[in]  oldSAHandle
 *      The current root SA for the SP.
 *  @param[in]  newSAHandle
 *      The new root SA for the SP.
 *      is to be used
 *  @param[out]  errCode
 *      Error code populated on error
 *
 *  \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   Handle to the new root security association
 *  @retval
 *      Error   -   NULL
 */
Netfp_SAHandle Netfp_changeRootSA
(
    Netfp_ClientHandle      clientHandle,
    Netfp_SAHandle          oldSAHandle,
    Netfp_SAHandle          newSAHandle,
    int32_t*                errCode
)
{
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    uint32_t                result;
    int32_t                 jobId;
    Netfp_ClientMCB*        ptrNetfpClient;

    /* Sanity Check: Ensure that the arguments are valid */
    if ((clientHandle == NULL) || (oldSAHandle == NULL) || (newSAHandle == NULL))
    {
        *errCode = NETFP_EINVAL;
        return NULL;
    }

    /* Get the NETFP Client MCB: Services are available only if the client is active. */
    ptrNetfpClient = (Netfp_ClientMCB*)clientHandle;
    if (ptrNetfpClient->status != Netfp_ClientStatus_ACTIVE)
    {
        *errCode = NETFP_ENOTREADY;
        return NULL;
    }

    /* Get the actual job which we will execute */
    jobHandle = Josh_findJobByAddress(ptrNetfpClient->joshHandle, (Josh_JobProtype)_Netfp_changeRootSA);
    if (jobHandle == NULL)
    {
        *errCode = NETFP_EINTERNAL;
        return NULL;
    }

    /* Initialize the arguments; to avoid any junk */
    memset ((void *)&args, 0, sizeof(args));

    /***************************************************************************
     * This is the function which is to be invoked:
     *
     *  Netfp_SAHandle _Netfp_changeSAForSP
     *  (
     *  Netfp_ServerMCB*        ptrNetfpServer,
     *  Netfp_SAHandle          oldSAHandle,
     *  Netfp_SAHandle          newSAHandle,
     *  int32_t*                errCode
     *  )
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Netfp_ServerHandle);

    /*  - Argument 2: */
    args[1].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[1].length = sizeof(Netfp_SAHandle);

    /*  - Argument 3: */
    args[2].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[2].length = sizeof(Netfp_SAHandle);

    /*  - Argument 4: */
    args[3].type   = Josh_ArgumentType_PASS_BY_REF;
    args[3].length = sizeof(int32_t);

    /* Add the arguments. */
    argHandle = Josh_addArguments (ptrNetfpClient->joshHandle, &args[0]);
    if (argHandle == NULL)
    {
        *errCode = NETFP_EJOSH;
        return NULL;
    }

    /* Populate the NETFP Server handle */
    *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(ptrNetfpClient->cfg.serverHandle);

    /* Populate the current root SA channel handle */
    *(uint32_t*)args[1].argBuffer = (uint32_t)josh_toRemoteU32(oldSAHandle);

    /* Populate the new root SA channel handle */
    *(uint32_t*)args[2].argBuffer = (uint32_t)josh_toRemoteU32(newSAHandle);

    /* Submit the JOB to JOSH for execution. */
    jobId = Josh_submitJob(jobHandle, argHandle, &result, errCode);
    if (jobId < 0)
        return NULL;

    /* Get the result arguments. */
    if (Josh_getArguments (ptrNetfpClient->joshHandle, jobId, &args[0]) < 0)
    {
        *errCode = NETFP_EINTERNAL;
        return NULL;
    }

    /* Copy the error code value. */
    *errCode = *((int32_t*)args[3].argBuffer);

    /* Free the JOB Instance. */
    if (Josh_freeJobInstance(ptrNetfpClient->joshHandle, jobId) < 0)
    {
        *errCode = NETFP_EJOSH;
        return NULL;
    }
    return (Netfp_SAHandle)result;
}

static int32_t _Netfp_initSwLutInfo
(
    Netfp_ServerMCB*    ptrNetfpServer,
    Netfp_initSwLut*    initSwLut,
    int32_t*            errCode
)
{
    if (!ptrNetfpServer->swLutInfo.isInitialized)
    {
        memcpy ((void*)&ptrNetfpServer->swLutInfo.initSwLut, (void *)initSwLut, sizeof(Netfp_initSwLut));

        *errCode = 0;

        ptrNetfpServer->swLutInfo.isInitialized = 1;
    }
    else
    {
        System_printf("ERR:: Netfp_initSwLutInfo already initialized !!!");

        *errCode = -1;
    }

    return *errCode;
}

/**
 *  @b Description
 *  @n
 *      This function is used to initialize the sw-lut-extension service with the specific NETFP client.
 *
 *  @param[in]  Netfp_initSwLut
 *      Handle to the Netfp_initSwLut as configuration of
 *      required attributes.
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_initSwLutInfo
(
    Netfp_initSwLut*    initSwLut,
    int32_t*            errCode
)
{
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    int32_t                 jobId;
    uint32_t                result;
    Netfp_ClientMCB*        ptrNetfpClient;
    Netfp_initSwLut*        ptrRemoteInitSwLutInfo;

    ptrNetfpClient = (Netfp_ClientMCB*)initSwLut->client;

    /* Get the actual job which we will execute */
    jobHandle = Josh_findJobByAddress(ptrNetfpClient->joshHandle, (Josh_JobProtype)_Netfp_initSwLutInfo);
    if (jobHandle == NULL)
    {
        *errCode = NETFP_EINTERNAL;
        return -1;
    }

    /* Initialize the arguments; to avoid any junk */
    memset ((void *)&args, 0, sizeof(args));

    /***************************************************************************
     * This is the function which is to be invoked:
     *
     * int32_t _Netfp_initSwLutInfo
     * (
     *  Netfp_ServerMCB*    ptrNetfpServer,
     *  Netfp_initSwLut*    initSwLut,
     *  int32_t*            errCode
     * )
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Netfp_ServerHandle);

    /*  - Argument 2: */
    args[1].type   = Josh_ArgumentType_PASS_BY_REF;
    args[1].length = sizeof(Netfp_initSwLut);

    /*  - Argument 3: */
    args[2].type   = Josh_ArgumentType_PASS_BY_REF;
    args[2].length = sizeof(int32_t);

    /* Add the arguments. */
    argHandle = Josh_addArguments (ptrNetfpClient->joshHandle, &args[0]);
    if (argHandle == NULL)
    {
        *errCode = NETFP_EJOSH;
        return -1;
    }

    /* Populate the arguments */
    *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(ptrNetfpClient->cfg.serverHandle);

    /* Get the pointer to the initSwLut configuration. */
    ptrRemoteInitSwLutInfo = (Netfp_initSwLut*)args[1].argBuffer;
    memcpy ((void*)ptrRemoteInitSwLutInfo, (void *)initSwLut, sizeof(Netfp_initSwLut));

    /* Submit the JOB to JOSH for execution. */
    jobId = Josh_submitJob(jobHandle, argHandle, &result, errCode);
    if (jobId < 0)
        return -1;

    /* Get the result arguments. */
    if (Josh_getArguments (ptrNetfpClient->joshHandle, jobId, &args[0]) < 0)
    {
        *errCode = NETFP_EINTERNAL;
        return -1;
    }

    /* Extract the output arguments */
    *errCode = *((int32_t*)args[2].argBuffer);

    /* Free the JOB Instance. */
    if (Josh_freeJobInstance(ptrNetfpClient->joshHandle, jobId) < 0)
    {
        *errCode = NETFP_EJOSH;
        return -1;
    }

    return (int32_t)result;
}

static uint64_t Netfp_ticksToNs(const uint64_t ticks)
{
    // per 10us so we don't lose ticks, ddal_cpu_counter_freq() returns 204800000
    uint32_t cpuTicksPer10us = ddal_cpu_counter_freq() / 100000;
    uint64_t ns =  100 * (100 * ticks) / cpuTicksPer10us;
    return ns;
}

static int32_t _Netfp_swSaOffloadDone
(
    Netfp_ServerMCB*    ptrNetfpServer,
    uint32_t            jobID,
    int32_t             status,
    int32_t*            errCode
)
{
    Netfp_PendingSW_SAOffloadNode* pendingSA = Netfp_findPendingSW_SA(ptrNetfpServer, jobID);
    if(pendingSA == NULL)
    {
        *errCode = NETFP_ENOTFOUND;
        return -1;
    }

    uint64_t totalTicks = ddal_cpu_counter_read() - pendingSA->startOffloadTicks;
    System_printf("Debug: done SW offload (job ID: %u, status: %s), took %llu ns",
                  pendingSA->jobID, (status == NETFP_RETVAL_SUCCESS ? "SUCCESS" : "FAILURE"),
                  Netfp_ticksToNs(totalTicks));

    Netfp_delPendingSW_SA(ptrNetfpServer, jobID);

    return 0;
}

int32_t Netfp_swSaOffloadDone
(
    Netfp_ClientHandle      clientHandle,
    uint32_t                jobID,
    int32_t                 status,
    int32_t*                errCode
)
{
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    int32_t                 jobId;
    uint32_t                result;
    Netfp_ClientMCB*        ptrNetfpClient;

    /* Sanity Check: Ensure that the arguments are valid */
    if (clientHandle == NULL)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Get the NETFP Client MCB: Services are available only if the client is active. */
    ptrNetfpClient = (Netfp_ClientMCB*)clientHandle;
    if (ptrNetfpClient->status != Netfp_ClientStatus_ACTIVE)
    {
        *errCode = NETFP_ENOTREADY;
        return -1;
    }

    /* Get the actual job which we will execute */
    jobHandle = Josh_findJobByAddress(ptrNetfpClient->joshHandle, (Josh_JobProtype)_Netfp_swSaOffloadDone);
    if (jobHandle == NULL)
    {
        *errCode = NETFP_EINTERNAL;
        return -1;
    }

    /* Initialize the arguments; to avoid any junk */
    memset ((void *)&args, 0, sizeof(args));

    /***************************************************************************
     * This is the function which is to be invoked:
     *
     * int32_t _Netfp_swSaOffloadDone
     * (
     *  Netfp_ServerMCB*    ptrNetfpServer,
     *  uint32_t            jobID,
     *  int32_t             status,
     *  int32_t*            errCode
     * )
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Netfp_ServerHandle);

    /*  - Argument 2: */
    args[1].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[1].length = sizeof(uint32_t);

    /*  - Argument 3: */
    args[2].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[2].length = sizeof(int32_t);

    /*  - Argument 4: */
    args[3].type   = Josh_ArgumentType_PASS_BY_REF;
    args[3].length = sizeof(int32_t);

    /* Add the arguments. */
    argHandle = Josh_addArguments (ptrNetfpClient->joshHandle, &args[0]);
    if (argHandle == NULL)
    {
        *errCode = NETFP_EJOSH;
        return -1;
    }

    /* Populate the arguments */
    *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(ptrNetfpClient->cfg.serverHandle);

    *(uint32_t*)args[1].argBuffer = jobID;

    *(int32_t*)args[2].argBuffer = status;

    /* Submit the JOB to JOSH for execution. */
    jobId = Josh_submitJob(jobHandle, argHandle, &result, errCode);
    if (jobId < 0)
        return -1;

    /* Get the result arguments. */
    if (Josh_getArguments (ptrNetfpClient->joshHandle, jobId, &args[0]) < 0)
    {
        *errCode = NETFP_EINTERNAL;
        return -1;
    }

    /* Extract the output arguments */
    *errCode = *((int32_t*)args[3].argBuffer);

    /* Free the JOB Instance. */
    if (Josh_freeJobInstance(ptrNetfpClient->joshHandle, jobId) < 0)
    {
        *errCode = NETFP_EJOSH;
        return -1;
    }

    return (int32_t)result;
}

//fzm<--

/**
 *  @b Description
 *  @n
 *      Utility function which is used to conver the encryption mode to
 *      a string.
 *
 *  @param[in]  mode
 *      Encyrption Mode
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Encryption string.
 */
static const char* Netfp_getEncryptionString(Netfp_IpsecCipherMode mode)
{
    switch (mode)
    {
        case Netfp_IpsecCipherMode_NULL:
        {
            return "NULL";
        }
        case Netfp_IpsecCipherMode_AES_CTR:
        {
            return "AES-CTR";
        }
        case Netfp_IpsecCipherMode_AES_CBC:
        {
            return "AES-CBC";
        }
        case Netfp_IpsecCipherMode_3DES_CBC:
        {
            return "3DES-CBC";
        }
        case Netfp_IpsecCipherMode_DES_CBC:
        {
            return "DES-CBC";
        }
        case Netfp_IpsecCipherMode_AES_GCM:
        {
            return "AES-GCM";
        }
        default:
        {
            return "Unspecified";
        }
    }
}

/**
 *  @b Description
 *  @n
 *      Utility function which is used to conver the authentication mode to
 *      a string.
 *
 *  @param[in]  mode
 *      Authentication Mode
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Encryption string.
 */
static const char* Netfp_getAuthenticationString(Netfp_IpsecAuthMode mode)
{
    switch (mode)
    {
        case Netfp_IpsecAuthMode_NULL:
        {
            return "NULL";
        }
        case Netfp_IpsecAuthMode_HMAC_SHA1:
        {
            return "HMAC-SHA1";
        }
        case Netfp_IpsecAuthMode_HMAC_MD5:
        {
            return "HMAC-MD5";
        }
        case Netfp_IpsecAuthMode_AES_XCBC:
        {
            return "AES-XCBC";
        }
        case Netfp_IpsecAuthMode_HMAC_SHA2_256:
        {
            return "HMAC-SHA2-256";
        }
        case Netfp_IpsecAuthMode_AES_GMAC:
        {
            return "AES-GMAC";
        }
        default:
        {
            return "Unspecified";
        }
    }
}

/**
 *  @b Description
 *  @n
 *      The function is used to kill all the IPSEC channels registered
 *      with the NETFP Server.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success  -   0
 *  @retval
 *      Error    -   <0
 */
int32_t Netfp_killSecurityAssociation
(
    Netfp_ServerMCB*    ptrNetfpServer,
    int32_t*            errCode
)
{
    Netfp_IPSecChannel*     ptrIPSecChannel;
    int32_t                 retVal;

    /* Close all the inbound security associations which have been configured in the server */
    ptrIPSecChannel = (Netfp_IPSecChannel*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrIPSecInboundChannels);
    while (ptrIPSecChannel != NULL)
    {
        /* Delete the security association from the server. */
        retVal = _Netfp_delSA (ptrNetfpServer, ptrIPSecChannel, Netfp_Reason_SA_DELETE, errCode);
        if (retVal < 0)
            return -1;

        /* Get the next security association */
        ptrIPSecChannel = (Netfp_IPSecChannel*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrIPSecInboundChannels);
    }

    /* Close all the outbound security associations which have been configured in the server */
    ptrIPSecChannel = (Netfp_IPSecChannel*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrIPSecOutboundChannels);
    while (ptrIPSecChannel != NULL)
    {
        /* Delete the security association from the server. */
        retVal = _Netfp_delSA (ptrNetfpServer, ptrIPSecChannel, Netfp_Reason_SA_DELETE, errCode);
        if (retVal < 0)
            return -1;

        /* Was the SA deletion completed already? */
        if (retVal == 0)
            ptrIPSecChannel = (Netfp_IPSecChannel*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrIPSecOutboundChannels);
        else
            ptrIPSecChannel = (Netfp_IPSecChannel*)Netfp_listGetNext ((Netfp_ListNode*)ptrIPSecChannel);
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is invoked to indicate that the SA has been infected and is no
 *      longer usable. The function will ensure that in turn spread the disease to all
 *      the fast paths and security policies which use the SA. It will also then generate
 *      an event to spread the disease to the NETFP client layer4 endpoints (sockets/3gpp channels)
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  reason
 *      Reason which is causing the update to the SA
 *  @param[in]  ptrNetfpInterface
 *      Pointer to the interface which has been infected
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
void Netfp_updateSA
(
    Netfp_ServerMCB*    ptrNetfpServer,
    Netfp_Reason        reason,
    Netfp_Interface*    ptrNetfpInterface
)
{
    Netfp_IPSecChannel*     ptrIPSecChannel;
    int32_t                 errCode;
    int32_t                 retVal;

    /* Processing is based on the reason: */
    switch (reason)
    {
        case Netfp_Reason_INTERFACE_DELETE:
        {
            /********************************************************************************************
             * Interface Delete: Cycle through the outbound SA using this interface and delete them
             ********************************************************************************************/
            ptrIPSecChannel = (Netfp_IPSecChannel*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrIPSecOutboundChannels);
            while (ptrIPSecChannel != NULL)
            {
                /* Does the security association reside on the interface? */
                if (ptrIPSecChannel->ptrNetfpInterface == ptrNetfpInterface)
                {
                    /* Delete the security association */
                    retVal = _Netfp_delSA (ptrNetfpServer, ptrIPSecChannel, Netfp_Reason_INTERFACE_DELETE, &errCode);

                    /* Was the SA deletion completed already? */
                    if (retVal == 0)
                        ptrIPSecChannel = (Netfp_IPSecChannel*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrIPSecOutboundChannels);
                    else
                        ptrIPSecChannel = (Netfp_IPSecChannel*)Netfp_listGetNext ((Netfp_ListNode*)ptrIPSecChannel);
                }
                else
                {
                    /* Get the next security association */
                    ptrIPSecChannel = (Netfp_IPSecChannel*)Netfp_listGetNext ((Netfp_ListNode*)ptrIPSecChannel);
                }
            }
            break;
        }
        case Netfp_Reason_IF_MTU_CHANGE:
        {
            /********************************************************************************************
             * Interface MTU change: Cycle through the outbound SA
             ********************************************************************************************/
            ptrIPSecChannel = (Netfp_IPSecChannel*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrIPSecOutboundChannels);
            while (ptrIPSecChannel != NULL)
            {
                /* Does the security association reside on the interface? Inform the SP Module */
                if (ptrIPSecChannel->ptrNetfpInterface == ptrNetfpInterface)
                    Netfp_updateSP (ptrNetfpServer, Netfp_Reason_IF_MTU_CHANGE, ptrIPSecChannel, ptrIPSecChannel);

                /* Get the next security association */
                ptrIPSecChannel = (Netfp_IPSecChannel*)Netfp_listGetNext ((Netfp_ListNode*)ptrIPSecChannel);
            }
            break;
        }
        default:
        {
            Netfp_logMsg (ptrNetfpServer, Netfp_LogLevel_ERROR, "Error: Update SA received an invalid reason [%s]\n", Netfp_getReasonString(reason));
            break;
        }
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is a utility function which is used to display the security associations
 *      in the system.
 *
 *  @param[in]  serverHandle
 *      NETFP Server Handle
 *  @param[in]  direction
 *      Direction (INBOUND or OUTBOUND)
 *
 * \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Number of security association which have been displayed
 */
int32_t Netfp_displaySA(Netfp_ServerHandle serverHandle, Netfp_Direction direction)
{
    Netfp_IPSecChannel*     ptrIPSecChannel;
    Netfp_ServerMCB*        ptrNetfpServer;
    int32_t                 errCode;
    int32_t                 count = 0;

    /* Get the server handle. */
    ptrNetfpServer = (Netfp_ServerMCB*)serverHandle;
    if (ptrNetfpServer == NULL)
        return 0;

    /* Get the fast path information */
    if (direction == Netfp_Direction_INBOUND)
        ptrIPSecChannel = (Netfp_IPSecChannel*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrIPSecInboundChannels);
    else
        ptrIPSecChannel = (Netfp_IPSecChannel*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrIPSecOutboundChannels);

//fzm - use Netfp_dumpMsg to capture this with the USR1 signal and other changes to content displayed

    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "**********************************************************\n");

    /* Display the SwLutExtension Information: */
    if (ptrNetfpServer->swLutInfo.isInitialized == 1)
    {
        if(direction == Netfp_Direction_INBOUND) //display this only once (before inbound)
        {
            Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "SwLutExtension service initialized\n");
            Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "NetCP flowId: %u queue: %d\n",
                           ptrNetfpServer->swLutInfo.initSwLut.flowId, ptrNetfpServer->swLutInfo.initSwLut.queue);

            if(ptrNetfpServer->ptrPendingSW_SAOffloadList)
            {
                Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "Pending SW LUT extension offloads:\n");
                Netfp_PendingSW_SAOffloadNode* pendingSa = (Netfp_PendingSW_SAOffloadNode*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrPendingSW_SAOffloadList);
                while (pendingSa != NULL)
                {
                    char dstIP[128];
                    if (pendingSa->ptrIPSecChannel && pendingSa->ptrIPSecChannel->saCfg.dstIP.ver == Netfp_IPVersion_IPV4)
                        snprintf(dstIP, sizeof(dstIP), "%03d.%03d.%03d.%03d",
                                 pendingSa->ptrIPSecChannel->saCfg.dstIP.addr.ipv4.u.a8[0], pendingSa->ptrIPSecChannel->saCfg.dstIP.addr.ipv4.u.a8[1],
                                pendingSa->ptrIPSecChannel->saCfg.dstIP.addr.ipv4.u.a8[2], pendingSa->ptrIPSecChannel->saCfg.dstIP.addr.ipv4.u.a8[3]);
                    else if (pendingSa->ptrIPSecChannel && pendingSa->ptrIPSecChannel->saCfg.dstIP.ver == Netfp_IPVersion_IPV6)
                        Netfp_convertIP6ToStr (pendingSa->ptrIPSecChannel->saCfg.dstIP.addr.ipv6, &dstIP[0]);
                    else
                        strcpy(dstIP, "UNKNOWN-CHANNEL-REMOVED");

                    uint64_t timeNsElapsed = Netfp_ticksToNs(ddal_cpu_counter_read() - pendingSa->startOffloadTicks);

                    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "JobID: %u, type: %s, SPI: 0x%x, dstIP: %s, timeElapsed: %llu ns\n",
                                   pendingSa->jobID, (pendingSa->eventType == Netfp_EventId_ADD_SW_OFFLOAD_SA ? "ADD" : "DEL"),
                                   (pendingSa->ptrIPSecChannel ? pendingSa->ptrIPSecChannel->saCfg.spi : 0), dstIP, timeNsElapsed);

                    pendingSa = (Netfp_PendingSW_SAOffloadNode*)Netfp_listGetNext ((Netfp_ListNode*)pendingSa);
                }
            }
        }
    }
    else
    {
        Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "SwLutExtension service NOT initialized\n");
    }

    /* Display the banner. */
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "**********************************************************\n");
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,
                  "%s Security Associations\n", (direction == Netfp_Direction_INBOUND) ? "Inbound" : "Outbound");

    /* Are we displaying INBOUND or OUTBOUND Security Associations? */
    if (direction == Netfp_Direction_INBOUND)
    {
        /* Does the server support the configuration of user counters? */
        if (ptrNetfpServer->cfg.enableIPLutEntryCount)
        {
            /* YES: Get the user statistics */
            if (Netfp_getUserStats (ptrNetfpServer, ptrNetfpServer->ptrUserStats, &errCode) < 0)
                Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_ERROR, "Error: Unable to get the user stats [Error code %d]\n", errCode);
        }

        /* INBOUND Security Association: */
        while (ptrIPSecChannel != NULL)
        {

            /* Display the security associations */
            Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,
                              "0x%x: [%s] SPI:0x%x  replayWindow:%u  %s  %s LUT:%d(%d) [Security Channel 0x%x]\n",
                              ptrIPSecChannel, (ptrIPSecChannel->status == Netfp_Status_ACTIVE) ? "ACTIVE" : (ptrIPSecChannel->status == Netfp_Status_ZOMBIE ? "ZOMBIE" : "STOPPED"),
                              ptrIPSecChannel->saCfg.spi, ptrIPSecChannel->saCfg.replayWindowSize,
                              Netfp_getEncryptionString(ptrIPSecChannel->saCfg.ipsecCfg.encMode),
                              Netfp_getAuthenticationString(ptrIPSecChannel->saCfg.ipsecCfg.authMode),
                              (ptrIPSecChannel->ptrSaLutInfo != NULL ? ptrIPSecChannel->ptrSaLutInfo->lutInfo.lut1Inst : -1),
                              (ptrIPSecChannel->ptrSaLutInfo != NULL ? ptrIPSecChannel->ptrSaLutInfo->lutInfo.lut1Index : -1),
                          ptrIPSecChannel->ptrSrvSecurityChannel);

            /* Display the IP address */
            if (ptrIPSecChannel->saCfg.srcIP.ver == Netfp_IPVersion_IPV4)
            {
                Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,
                              "            %03d.%03d.%03d.%03d -> %03d.%03d.%03d.%03d\n",
                              ptrIPSecChannel->saCfg.srcIP.addr.ipv4.u.a8[0], ptrIPSecChannel->saCfg.srcIP.addr.ipv4.u.a8[1],
                              ptrIPSecChannel->saCfg.srcIP.addr.ipv4.u.a8[2], ptrIPSecChannel->saCfg.srcIP.addr.ipv4.u.a8[3],
                              ptrIPSecChannel->saCfg.dstIP.addr.ipv4.u.a8[0], ptrIPSecChannel->saCfg.dstIP.addr.ipv4.u.a8[1],
                              ptrIPSecChannel->saCfg.dstIP.addr.ipv4.u.a8[2], ptrIPSecChannel->saCfg.dstIP.addr.ipv4.u.a8[3]);
            }
            else
            {
                char    srcIP[128];
                char    dstIP[128];

                /* Convert the Src & Dst IP address of the SA into strings using the NETFP IPv6 Utility API */
                Netfp_convertIP6ToStr (ptrIPSecChannel->saCfg.srcIP.addr.ipv6, &srcIP[0]);
                Netfp_convertIP6ToStr (ptrIPSecChannel->saCfg.dstIP.addr.ipv6, &dstIP[0]);

                Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,
                              "            %s -> %s\n",srcIP, dstIP);
            }

            /* Display the NAT-T Information? */
            if ((ptrIPSecChannel->saCfg.nattEncapCfg.srcPort != 0) &&
                 (ptrIPSecChannel->ptrNattIPInfo != NULL) )
            {
                Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,
                              "            NAT-T port [%d -> %d], LUT:%d(%d) RefCnt:%d  UserStatIndex:%d  Packets:%d\n",
                              ptrIPSecChannel->saCfg.nattEncapCfg.srcPort, ptrIPSecChannel->saCfg.nattEncapCfg.dstPort,
                                  (ptrIPSecChannel->ptrNattIPInfo != NULL ? ptrIPSecChannel->ptrNattIPInfo->nattLutInfo->lutInfo.lut1Inst : -1),
                              ptrIPSecChannel->ptrNattIPInfo->refCount,
                              ptrIPSecChannel->ptrNattIPInfo->nattLutInfo->matchPktCntIndex,
                              ptrNetfpServer->ptrUserStats->count32[ptrIPSecChannel->ptrNattIPInfo->nattLutInfo->matchPktCntIndex -
                                                                    ptrNetfpServer->userStatCfg.num64bUserStats]);
            }

            /* Does the server support the configuration of user counters? */
            if (ptrNetfpServer->cfg.enableIPLutEntryCount)
            {
                /* YES: Display the user statistics counters matching the user statistics index: */
                Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,
                              "            UserStatIndex:%d  Packets:%d\n",
                              ptrIPSecChannel->ptrSaLutInfo->matchPktCntIndex,
                              ptrNetfpServer->ptrUserStats->count32[ptrIPSecChannel->ptrSaLutInfo->matchPktCntIndex -
                              ptrNetfpServer->userStatCfg.num64bUserStats]);
            }

            /* Increment the counter */
            count = count + 1;

            /* Get the next SA */
            ptrIPSecChannel = (Netfp_IPSecChannel*)Netfp_listGetNext ((Netfp_ListNode*)ptrIPSecChannel);
        }
        return count;
    }

    /* OUTBOUND Security Association: */
    while (ptrIPSecChannel != NULL)
    {
        /* Display the security associations */
        Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,
                      "0x%x: [%s] SPI:0x%x  replayWindow:%u  %s  %s [Security Channel 0x%x]\n",
                      ptrIPSecChannel, (ptrIPSecChannel->status == Netfp_Status_ACTIVE) ? "ACTIVE" : (ptrIPSecChannel->status == Netfp_Status_ZOMBIE ? "ZOMBIE" : "STOPPED"),
                      ptrIPSecChannel->saCfg.spi, ptrIPSecChannel->saCfg.replayWindowSize,
                      Netfp_getEncryptionString(ptrIPSecChannel->saCfg.ipsecCfg.encMode),
                      Netfp_getAuthenticationString(ptrIPSecChannel->saCfg.ipsecCfg.authMode),
                      ptrIPSecChannel->ptrSrvSecurityChannel);

        /* Display the IP address */
        if (ptrIPSecChannel->saCfg.srcIP.ver == Netfp_IPVersion_IPV4)
        {
            Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,
                          "           %03d.%03d.%03d.%03d -> %03d.%03d.%03d.%03d [0x%02x:0x%02x:0x%02x:0x%02x:0x%02x:0x%02x]\n",
                          ptrIPSecChannel->saCfg.srcIP.addr.ipv4.u.a8[0], ptrIPSecChannel->saCfg.srcIP.addr.ipv4.u.a8[1],
                          ptrIPSecChannel->saCfg.srcIP.addr.ipv4.u.a8[2], ptrIPSecChannel->saCfg.srcIP.addr.ipv4.u.a8[3],
                          ptrIPSecChannel->saCfg.dstIP.addr.ipv4.u.a8[0], ptrIPSecChannel->saCfg.dstIP.addr.ipv4.u.a8[1],
                          ptrIPSecChannel->saCfg.dstIP.addr.ipv4.u.a8[2], ptrIPSecChannel->saCfg.dstIP.addr.ipv4.u.a8[3],
                          ptrIPSecChannel->nextHopMACAddress[0], ptrIPSecChannel->nextHopMACAddress[1],
                          ptrIPSecChannel->nextHopMACAddress[2], ptrIPSecChannel->nextHopMACAddress[3],
                          ptrIPSecChannel->nextHopMACAddress[4], ptrIPSecChannel->nextHopMACAddress[5]);
        }
        else
        {
            char    srcIP[128];
            char    dstIP[128];

            /* Convert the Src & Dst IP address of the SA into strings using the NETFP IPv6 Utility API */
            Netfp_convertIP6ToStr (ptrIPSecChannel->saCfg.srcIP.addr.ipv6, &srcIP[0]);
            Netfp_convertIP6ToStr (ptrIPSecChannel->saCfg.dstIP.addr.ipv6, &dstIP[0]);

            Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,
                          "           %s -> %s [0x%02x:0x%02x:0x%02x:0x%02x:0x%02x:0x%02x]\n",
                          srcIP, dstIP, ptrIPSecChannel->nextHopMACAddress[0], ptrIPSecChannel->nextHopMACAddress[1],
                          ptrIPSecChannel->nextHopMACAddress[2], ptrIPSecChannel->nextHopMACAddress[3],
                          ptrIPSecChannel->nextHopMACAddress[4], ptrIPSecChannel->nextHopMACAddress[5]);
        }

        /* Display the PMTU configuration */
        Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,
                      "           PMTU:%d bytes Age:%d sec\n", ptrIPSecChannel->ipsecPMTU, ptrIPSecChannel->agePMTU);

        /* Increment the counter */
        count = count + 1;

        /* Get the next SA */
        ptrIPSecChannel = (Netfp_IPSecChannel*)Netfp_listGetNext ((Netfp_ListNode*)ptrIPSecChannel);
    }
    return count;
}

/**
 *  @b Description
 *  @n
 *      The function is used to obtain the software context information given the security handle.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  saHandle
 *      IPSec channel handle.
 *  @param[out] swContext
 *      Pointer to the software context for the given security association.
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t _Netfp_getIPSecSwInfo
(
    Netfp_ServerMCB*    ptrNetfpServer,
    Netfp_SAHandle      saHandle,
    Netfp_SwContext*    swContext,
    int32_t*            errCode
)
{
    Netfp_L3QoSCfg*     ptrL3QosCfg;
    Netfp_IPSecChannel* ptrIPSecChannel;

    /* Sanity check on saHandle */
    if ( saHandle == NULL)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Get IPSec channel handle */
    ptrIPSecChannel = (Netfp_IPSecChannel *)saHandle;

    /* Check if virtual interface is available */
    if (ptrIPSecChannel->ptrNetfpInterface != NULL)
    {
        /* Interface is available. Is L3 QOS configured on the interface? */
        ptrL3QosCfg    = &ptrIPSecChannel->ptrNetfpInterface->l3QosCfg;

        /* Get the flowid to be used to pass packet from SA to PA  */
        if (ptrL3QosCfg->isEnable == 1)
        {
            /* If L3 QOS is enabled, use L3 QOS Flow Identifier */
            swContext->flowId = ptrL3QosCfg->flowId;
        }
        else
        {
            /* If L3 QOS is disabled, default to IPSEC Flow Identifier */
            swContext->flowId = Cppi_getFlowId(ptrNetfpServer->ipsecFlowHandle);
        }
    }
    else
    {
        /* Interface has not been created in the system till now. Defaulting to the
         * IPSEC flow identifier. No other option here. */
        swContext->flowId = Cppi_getFlowId(ptrNetfpServer->ipsecFlowHandle);
    }

    /* Get the software context information from SA */
    if (ptrIPSecChannel->saCfg.direction == Netfp_Direction_INBOUND)
        *errCode = Netfp_getChanSwInfo(ptrIPSecChannel->ptrSrvSecurityChannel->saChannelHandle,
                                       sa_PKT_DIR_FROM_NETWORK,  &swContext->swInfo) ;
    else
        *errCode = Netfp_getChanSwInfo(ptrIPSecChannel->ptrSrvSecurityChannel->saChannelHandle,
                                       sa_PKT_DIR_TO_NETWORK,   &swContext->swInfo) ;

    /* Check return error */
    if ( *errCode < 0 )
        return -1;

    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to obtain the software context information given the security handle.
 *
 *  @param[in]  clientHandle
 *      NETFP client handle
 *  @param[in]  saHandle
 *      Security Association handle.
 *  @param[out] swContext
 *      Pointer to the software context for the given security association.
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
int32_t Netfp_getIPSecSwInfo
(
    Netfp_ClientHandle  clientHandle,
    Netfp_SAHandle      saHandle,
    Netfp_SwContext*    swContext,
    int32_t*            errCode
)
{
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    uint32_t                result;
    int32_t                 jobId;
    Netfp_ClientMCB*        ptrNetfpClient;
    Netfp_SwContext*        ptrRemoteSwContext;

    /* Sanity Check: Ensure that the arguments are valid */
    if ((clientHandle == NULL) || (saHandle == NULL) || (swContext == NULL) )
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Get the NETFP Client MCB: Services are available only if the client is active. */
    ptrNetfpClient = (Netfp_ClientMCB*)clientHandle;
    if (ptrNetfpClient->status != Netfp_ClientStatus_ACTIVE)
    {
        *errCode = NETFP_ENOTREADY;
        return -1;
    }

    /* Get the actual job which we will execute */
    jobHandle = Josh_findJobByAddress(ptrNetfpClient->joshHandle, (Josh_JobProtype)_Netfp_getIPSecSwInfo);
    if (jobHandle == NULL)
    {
        *errCode = NETFP_EINTERNAL;
        return -1;
    }

    /* Initialize the arguments; to avoid any junk */
    memset ((void *)&args, 0, sizeof(args));

    /***************************************************************************
     * This is the function which is to be invoked:
     *
     * int32_t _Netfp_getIPSecSwInfo
     * (
     *  Netfp_ServerMCB*    ptrNetfpServer,
     *  Netfp_SAHandle      saHandle,
     *  Netfp_SwContext*    swContext,
     *  int32_t*            errCode
     * )

     ****************************************************************************/
    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Netfp_ServerHandle);

    /*  - Argument 2: */
    args[1].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[1].length = sizeof(Netfp_SAHandle);

    /*  - Argument 3: */
    args[2].type   = Josh_ArgumentType_PASS_BY_REF;
    args[2].length = sizeof(Netfp_SwContext);

    /*  - Argument 4: */
    args[3].type   = Josh_ArgumentType_PASS_BY_REF;
    args[3].length = sizeof(int32_t);

    /* Add the arguments. */
    argHandle = Josh_addArguments (ptrNetfpClient->joshHandle, &args[0]);
    if (argHandle == NULL)
    {
        *errCode = NETFP_EJOSH;
        return -1;
    }

    /* Populate the arguments */
    *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(ptrNetfpClient->cfg.serverHandle);
    *(uint32_t*)args[1].argBuffer = (uint32_t)josh_toRemoteU32(saHandle);

    /* Submit the JOB to JOSH for execution. */
    jobId = Josh_submitJob(jobHandle, argHandle, &result, errCode);
    if (jobId < 0)
        return -1;

    /* Get the result arguments. */
    if (Josh_getArguments (ptrNetfpClient->joshHandle, jobId, &args[0]) < 0)
    {
        *errCode = NETFP_EINTERNAL;
        return -1;
    }

    /* Copy the error code value. */
    *errCode = *((int32_t*)args[3].argBuffer);

    /* Free the JOB Instance. */
    if (Josh_freeJobInstance(ptrNetfpClient->joshHandle, jobId) < 0)
    {
        *errCode = NETFP_EJOSH;
        return -1;
    }

    /* Verify the result */
    if ((int32_t)result < 0)
        return -1;

    /* Josh job is done successfully, get the pointer to the swcontext */
    ptrRemoteSwContext    = (Netfp_SwContext *)args[2].argBuffer;
    memcpy(swContext, ptrRemoteSwContext, sizeof(Netfp_SwContext));

    return 0;
}

/** *  @b Description
 *  @n
 *      The function is used to register the NETFP IPSEC services
 *
 *  @param[in]  nodeHandle
 *      JOSH node handle for which the jobs are being registered
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_registerIPSecServices (Josh_NodeHandle nodeHandle)
{
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Netfp_addSA);
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Netfp_stopSA);
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Netfp_delSA);
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Netfp_rekeySA);
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Netfp_getIPsecStats);
//fzm-->
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Netfp_changeRootSA);
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Netfp_initSwLutInfo);
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Netfp_swSaOffloadDone);
//<--fzm
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Netfp_getIPSecSwInfo);
    return 0;
}
