/**
 *   @file  netfp_iface.c
 *
 *   @brief
 *      The NETFP Interface Management layers keeps track of the various
 *      networking interfaces over which the NETFP is operating.
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
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/* SYSLIB Include Files. */
#include <ti/runtime/netfp/netfp.h>
#include <ti/runtime/netfp/include/netfp_net.h>
#include <ti/runtime/netfp/include/netfp_ipv6.h>
#include <ti/runtime/netfp/include/netfp_internal.h>

/* For Debugging only. */
#ifdef __ARMv7
#include <ti/apps/netfp_config/include/NetFP_System_printf.h> //fzm
#else
#include <xdc/runtime/System.h>
#endif

/**************************************************************************
 ************************* Local Definitions ******************************
 **************************************************************************/

/**
 * @brief   Status Flag definition which indicates that the interface block
 * is operational and can be used to send packets.
 */
#define NETFP_INTERFACE_UP              0x1

/**************************************************************************
 ********************* NETFP Interface Functions **************************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      The function is used to validate the Interface handle to verify
 *      if the handle is still valid or if it has been deleted
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  ptrNetfpIf
 *      Interface which is to be validated
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Valid       -   1
 *  @retval
 *      Invalid     -   0
 */
int32_t Netfp_isValidInterface
(
    Netfp_ServerMCB*    ptrNetfpServer,
    Netfp_Interface*    ptrNetfpIf
)
{
    Netfp_Interface*    ptrNetfpInterface;

    /* Cycle through all the interfaces */
    ptrNetfpInterface = (Netfp_Interface*)Netfp_listGetHead((Netfp_ListNode**)&ptrNetfpServer->ptrInterfaceList);
    while (ptrNetfpInterface != NULL)
    {
        /* Is this interface of interest? */
        if (ptrNetfpInterface == ptrNetfpIf)
            return 1;

        /* Cycle through and get the next interface */
        ptrNetfpInterface = (Netfp_Interface*)Netfp_listGetNext ((Netfp_ListNode*)ptrNetfpInterface);
    }

    /* Control comes here implies that there was no match and the interface handle is invalid */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to transmit RAW packets over a specific switch port
 *
 *  @param[in]  ptrNetfpClient
 *      Pointer to the NETFP Client
 *  @param[in]  ptrPayload
 *      Pointer to the payload
 *  @param[in]  switchPort
 *      Switch port over which the packet is to be transmitted
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_rawTransmitInterface
(
    Netfp_ClientMCB*    ptrNetfpClient,
    Ti_Pkt*             ptrPayload,
    int32_t             switchPort,
    int32_t*            errCode
)
{
    paCmdInfo_t         cmdInfo[NETFP_TX_CMD_MAX];
    paCmdNextRoute_t    switchRouteInfo;
    uint8_t             numCommands = 0;
    uint32_t            cmdBuffer[NETFP_PA_CMD_BUFFER_SIZE];
    uint16_t            cmdBufferSize;
    int32_t             retVal;

    /* Populate the switch route information: This is used to push the packet directly to the
     * switch port and then out on the wire. */
    switchRouteInfo.ctrlBitfield     = 0;
    switchRouteInfo.dest             = pa_DEST_EMAC;
    switchRouteInfo.pktType_emacCtrl = switchPort;
    switchRouteInfo.multiRouteIndex  = (uint16_t)pa_NO_MULTI_ROUTE;
    switchRouteInfo.swInfo0          = 0;
    switchRouteInfo.swInfo1          = 0;
    switchRouteInfo.queue            = 0;
    switchRouteInfo.flowId           = 0;
    switchRouteInfo.statsIndex       = 0;

    /* Setting up command for Switch port */
    cmdInfo[0].cmd          = pa_CMD_NEXT_ROUTE;
    cmdInfo[0].params.route = switchRouteInfo;

    /* Save number of commands added */
    numCommands = 1;

    /* Set the command buffer size. */
    cmdBufferSize = NETFP_PA_CMD_BUFFER_SIZE;

    /* Configure the command set. */
    retVal = Pa_formatTxCmd (numCommands, cmdInfo, 0, (Ptr)cmdBuffer, &cmdBufferSize);
    if (retVal != pa_OK)
    {
        *errCode = retVal;
        return -1;
    }

    /* Attach the command in the protocol specific data of the descriptor. */
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)ptrPayload, (uint8_t *)(cmdBuffer), cmdBufferSize);

    /******************************************************************************
     * NOTE: There is an ownership change here from the DSP to the CPDMA. We need
     * to ensure that the packet contents here are written back.
     ******************************************************************************/
    Pktlib_releaseOwnership(ptrNetfpClient->cfg.pktlibInstHandle, ptrPayload);

    /* We always push the packet to PDSP5 since the commands have been populated properly in the packet it will
     * find its way out through the configured command set to the switch port and out onto the wire. */
    Qmss_queuePushDescSize (ptrNetfpClient->netcpTxQueue[NSS_PA_QUEUE_TXCMD_INDEX], (Cppi_Desc*)ptrPayload, 128);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to transmit the packet on the specific interface
 *      The packet has been populated with all the networking headers; the
 *      function is now responsible for interfacing with the PA firmware or
 *      the switch to send out the packet.
 *
 *  @param[in]  ptrNetfpSocket
 *      Pointer to the socket which is sending the packet
 *  @param[in]  ptrSockTxMetaInfo
 *      Pointer to the socket transmit meta information
 *  @param[in]  ptrPayload
 *      Pointer to the payload packet which includes the L2 header.
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_transmitInterface
(
    Netfp_Socket*           ptrNetfpSocket,
    Netfp_SockTxMetaInfo*   ptrSockTxMetaInfo,
    Ti_Pkt*                 ptrPayload
)
{
    Netfp_ClientMCB*            ptrNetfpClient;
    uint32_t                    bytes = 0;
    int32_t                     retVal;
    paCmdIpFrag_t               ipFragCmd;
    paCmdInfo_t                 cmdInfo[NETFP_TX_CMD_MAX];
	paCmdCrcOp_t                fpCrcCmd;
    paCmdNextRoute_t            switchRouteInfo;
    paCmdNextRoute_t            qosRouteInfo;
    paCmdNextRoute_t            postUDPChksumRouteInfo;
    paCmdNextRoute_t            postFpCrcRouteInfo;
    uint32_t                    cmdBuffer[NETFP_PA_CMD_BUFFER_SIZE];
    uint16_t                    cmdBufferSize;
    uint8_t                     numCommands = 0;
    Netfp_SockL2ConnectInfo*    ptrSockL2ConnectInfo;
    uint8_t                     cmdIdx = 0;
    paTxChksum_t                chksum;
    uint8_t                     hwFragmentation;
    uint8_t                     hwUDPChksum;
    uint8_t                     hwFrameProtoCrc;
    /* The definition supports upto 8 TX CMD */
    uint8_t                     ifaceTxCmd = 0;

    /* Get the client handle */
    ptrNetfpClient = ptrNetfpSocket->ptrNetfpClient;

    /* Get the socket L2 connect information: */
    ptrSockL2ConnectInfo = ptrSockTxMetaInfo->ptrL2ConnectInfo;

    /* Is the interface operational? */
    if (ptrSockL2ConnectInfo->ifStatus == 0)
    {
        /* Interface has been administrativly brought down. The packet cannot be sent out
         * We cleanup the packet memory. */
        Pktlib_freePacket(ptrNetfpClient->cfg.pktlibInstHandle, ptrPayload);
        return 0;
    }

    /***********************************************************************************************
     * Ethernet Minimum Padding:
     ***********************************************************************************************/
    if (Pktlib_getPacketLen(ptrPayload) < ETH_MIN_PKT_SIZE)
    {
        Ti_Pkt* ptrNextPayload = Pktlib_getNextPacket(ptrPayload);

        /* YES. Determine the number of bytes by which we need to extend the
         * packet size so that it fits the MINIMUM Ethernet Packet Size. */
        bytes = ETH_MIN_PKT_SIZE - Pktlib_getPacketLen(ptrPayload);

        /* Is this a chained packet? */
        if (ptrNextPayload != NULL)
        {
            /* Loop through the chained packets till the end. */
            while (Pktlib_getNextPacket(ptrNextPayload) != NULL)
                ptrNextPayload = Pktlib_getNextPacket(ptrNextPayload);

            /* For the last chained packet we increment the buffer length with the
             * number of bytes required to ensure that it fits the MIN Ethernet packet size. */
            Pktlib_setDataBufferLen(ptrNextPayload, Pktlib_getDataBufferLen(ptrNextPayload) + bytes);
        }
        else
        {
            /* NO. Simply set the new data buffer length to be MIN Ethernet packet size. */
            Pktlib_setDataBufferLen(ptrPayload, ETH_MIN_PKT_SIZE);
        }
        /* Set the packet length to be the MINIMUM Ethernet Size. */
        Pktlib_setPacketLen(ptrPayload, ETH_MIN_PKT_SIZE);
    }

    /* Is the socket secure? Send it to the NETCP for encryption and send the packet out. */
    if (ptrNetfpSocket->connectInfo.isSecure)
        return Netfp_encryptPkt(ptrNetfpSocket, ptrSockTxMetaInfo, ptrPayload);

    /* Get Frame Protocol CRC setting. This is used only for WCDMA Frame Protocol channels */
    hwFrameProtoCrc = ptrSockTxMetaInfo->frameProtoCrc;

    if (hwFrameProtoCrc)
    {
        /* Configure the frame protocol CRC command. */
        fpCrcCmd.ctrlBitfield = pa_CRC_OP_CRC_RESULT_FOLLOW_PAYLOAD;
		fpCrcCmd.startOffset  = ptrSockTxMetaInfo->headerSize + ptrSockTxMetaInfo->frameProtoPayloadOffset;
		fpCrcCmd.len          = Pktlib_getPacketLen(ptrPayload) - (fpCrcCmd.startOffset + 2) - bytes;
		fpCrcCmd.lenOffset    = 0;
		fpCrcCmd.lenMask      = 0;
		fpCrcCmd.lenAdjust    = 0;
        fpCrcCmd.crcOffset    = fpCrcCmd.startOffset + fpCrcCmd.len;
        //fpCrcCmd.crcOffset    = 0;
		fpCrcCmd.frameType    = 0;
        fpCrcCmd.crcSize      = 2;
        fpCrcCmd.initValue    = 0x0000;

        /* In non-secure mode, the packet is routed back to PA for further handling of the packet
         * We are using the Frame Protocol flow identifier to move the packet back into PDSP5 after the
         * CRC has been calculated */
		postFpCrcRouteInfo.ctrlBitfield     = 0;
		postFpCrcRouteInfo.dest             = pa_DEST_HOST;
		postFpCrcRouteInfo.pktType_emacCtrl = 0;
		postFpCrcRouteInfo.multiRouteIndex  = (uint16_t)pa_NO_MULTI_ROUTE;
		postFpCrcRouteInfo.swInfo0          = 0;
		postFpCrcRouteInfo.swInfo1          = 0;
		postFpCrcRouteInfo.queue            = ptrNetfpClient->netcpTxQueue[NSS_PA_QUEUE_TXCMD_INDEX];
		postFpCrcRouteInfo.flowId           = ptrSockL2ConnectInfo->ipsecChanFlowID;
        postFpCrcRouteInfo.statsIndex       = 0;
    }

    /* Get Fragmentation and hardware UDP checksum settings */
    hwFragmentation = ptrSockTxMetaInfo->hwFragmentation;
    hwUDPChksum     = ptrSockTxMetaInfo->hwUDPChksum;

    /* Is the packet going to be fragmented in the PA? */
    if (hwFragmentation)
    {
        /* Configure the fragmentation parameters */
        ipFragCmd.ipOffset = ptrSockL2ConnectInfo->l2HeaderSize;
        ipFragCmd.mtuSize  = ptrSockL2ConnectInfo->mtu;
    }

    /* Is UDP checksum done in NETCP hardware? */
    if (hwUDPChksum)
    {
        /* UDP checksum command: startOffset, legnthBytes and initialSum are
         * updated for every packet when setting up the UDP header. */
        chksum.startOffset      = ptrSockTxMetaInfo->udpHwChkSum.startOffset;
        chksum.lengthBytes      = ptrSockTxMetaInfo->udpHwChkSum.lengthBytes;
        chksum.resultOffset     = 6;
        chksum.initialSum       = ptrSockTxMetaInfo->udpHwChkSum.initialSum;
        chksum.negative0        = 1;

        /* In non-secure mode, the packet is routed back to PA for further handling of the packet
         * We are using the PA-SA flow identifier to move the packet back into PDSP5 after the
         * checksum has been calculated */
        postUDPChksumRouteInfo.ctrlBitfield      = 0;
        postUDPChksumRouteInfo.dest              = pa_DEST_HOST;
        postUDPChksumRouteInfo.pktType_emacCtrl  = 0;
        postUDPChksumRouteInfo.multiRouteIndex   = (uint16_t)pa_NO_MULTI_ROUTE;
        postUDPChksumRouteInfo.swInfo0           = 0;
        postUDPChksumRouteInfo.swInfo1           = 0;
        postUDPChksumRouteInfo.queue             = ptrNetfpClient->netcpTxQueue[NSS_PA_QUEUE_TXCMD_INDEX];
        postUDPChksumRouteInfo.flowId            = ptrSockL2ConnectInfo->ipsecChanFlowID;
        postUDPChksumRouteInfo.statsIndex        = 0;
    }

    /* Do we need L3 shaping? */
    if (ptrSockL2ConnectInfo->l3QosCfg.isEnable == 1)
    {
        /* L3 shaping is required: We need to setup the route information to send the packet
         * to the L3 shaping engine. L3 shaping is always done on the Inner DSCP. */
        qosRouteInfo.ctrlBitfield      = 0;
        qosRouteInfo.dest              = pa_DEST_HOST;
        qosRouteInfo.pktType_emacCtrl  = 0;
        qosRouteInfo.multiRouteIndex   = (uint16_t)pa_NO_MULTI_ROUTE;
        qosRouteInfo.swInfo0           = 0;
        qosRouteInfo.swInfo1           = 0;
        qosRouteInfo.queue             = ptrSockL2ConnectInfo->l3QosCfg.qid[ptrSockTxMetaInfo->innerDSCP];
        qosRouteInfo.flowId            = ptrSockL2ConnectInfo->l3QosCfg.flowId;
        qosRouteInfo.statsIndex        = 0;
    }

    /* Populate the switch route information: This is used to push the packet directly to the
     * switch port and then out on the wire. */
    switchRouteInfo.ctrlBitfield     = 0;
    switchRouteInfo.dest             = pa_DEST_EMAC;
    switchRouteInfo.pktType_emacCtrl = ptrSockL2ConnectInfo->switchPortNum;
    switchRouteInfo.multiRouteIndex  = (uint16_t)pa_NO_MULTI_ROUTE;
    switchRouteInfo.swInfo0          = 0;
    switchRouteInfo.swInfo1          = 0;
    switchRouteInfo.queue            = 0;
    switchRouteInfo.flowId           = 0;
    switchRouteInfo.statsIndex       = 0;

    /*====================================================================================
     * Contruct PA TX command to send the non-secure packet.
     * The TX commmand has to follow the following sequences:
     * 1. Hardware Frame Protocol CRC if configured in ptrSockTxMetaInfo for WCDMA Frame Protocol channels.
     * 2. Hardware UDP checksum if configured in ptrSockTxMetaInfo.
     * 3. Hardware Fragmentation if configured in ptrSockTxMetaInfo.
     * 4. L3 Qos if configured in L2 Connect info
     * 5. EMAC switch routing command.
     *====================================================================================*/
    /* Setting up command for Frame Protocol CRC */
    if (hwFrameProtoCrc == 1)
    {
        cmdInfo[cmdIdx].cmd              = pa_CMD_CRC_OP;
        cmdInfo[cmdIdx].params.crcOp     = fpCrcCmd;
        cmdIdx++;

        cmdInfo[cmdIdx].cmd              = pa_CMD_NEXT_ROUTE;
        cmdInfo[cmdIdx].params.route     = postFpCrcRouteInfo;
        cmdIdx++;
        ifaceTxCmd += 1 << Netfp_PATxCmdSequence_FrameProtoCrc;
    }

    /* Setting up command for UDP checksum */
    if (hwUDPChksum == 1)
    {
        cmdInfo[cmdIdx].cmd              = pa_CMD_TX_CHECKSUM;
        cmdInfo[cmdIdx].params.chksum    = chksum;
        cmdIdx++;

        cmdInfo[cmdIdx].cmd              = pa_CMD_NEXT_ROUTE;
        cmdInfo[cmdIdx].params.route     = postUDPChksumRouteInfo;
        cmdIdx++;
        ifaceTxCmd += 1 << Netfp_PATxCmdSequence_UDPChksum;
    }

    /* Setting up command for IP Fragmentation */
    if (hwFragmentation == 1)
    {
        cmdInfo[cmdIdx].cmd              = pa_CMD_IP_FRAGMENT;
        cmdInfo[cmdIdx].params.ipFrag    = ipFragCmd;
        cmdIdx++;
        ifaceTxCmd += 1 << Netfp_PATxCmdSequence_IPFrag;
    }

    /* Setting up command for L3 Qos */
    if (ptrSockL2ConnectInfo->l3QosCfg.isEnable == 1)
    {
        cmdInfo[cmdIdx].cmd              = pa_CMD_NEXT_ROUTE;
        cmdInfo[cmdIdx].params.route     = qosRouteInfo;
        cmdIdx++;
        ifaceTxCmd += 1 << Netfp_PATxCmdSequence_RouteL3Qos;
    }

    /* Setting up command for Switch port */
    cmdInfo[cmdIdx].cmd              = pa_CMD_NEXT_ROUTE;
    cmdInfo[cmdIdx].params.route     = switchRouteInfo;
    cmdIdx++;
    ifaceTxCmd += 1 << Netfp_PATxCmdSequence_RouteEMAC;

    /* update interface transmit statistics */
    ptrNetfpSocket->extendedStats.ifaceTxStats[ifaceTxCmd]++;

    /* Save number of commands added */
    numCommands = cmdIdx;

    /* Set the command buffer size. */
    cmdBufferSize = NETFP_PA_CMD_BUFFER_SIZE;

    /* Configure the command set. */
    retVal = Pa_formatTxCmd (numCommands, cmdInfo, 0, (Ptr)cmdBuffer, &cmdBufferSize);
    if (unlikely(retVal != pa_OK))
        return retVal;

    /* Attach the command in the protocol specific data of the descriptor. */
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)ptrPayload, (uint8_t *)(cmdBuffer), cmdBufferSize);

    /*********************************************************************************
     * Non Secure Socket: POST Routing Hook
     *********************************************************************************/
    if (Netfp_socketHook (Netfp_Hook_POST_ROUTING, ptrNetfpClient, ptrNetfpSocket, ptrPayload) < 0)
        return 0;

    /************************************************************************
     * Socket Statistics: Increment the NON-SECURE socket statistics for the
     * IPv4 and IPv6 families
     ************************************************************************/
    if (ptrNetfpSocket->family == Netfp_SockFamily_AF_INET)
        Netfp_incNonSecureIPv4Stats(ptrNetfpSocket, ptrSockTxMetaInfo, ptrPayload);
    else
        Netfp_incNonSecureIPv6Stats(ptrNetfpSocket, ptrSockTxMetaInfo, ptrPayload);

    /******************************************************************************
     * NOTE: There is an ownership change here from the DSP to the CPDMA. We need
     * to ensure that the packet contents here are written back.
     ******************************************************************************/
    Pktlib_releaseOwnership(ptrNetfpClient->cfg.pktlibInstHandle, ptrPayload);

    /* We always push the packet to PDSP5 since the commands have been populated properly in the packet it will
     * find its way out through the configured command set to the switch port and out onto the wire. */
    Qmss_queuePushDescSize (ptrNetfpClient->netcpTxQueue[NSS_PA_QUEUE_TXCMD_INDEX], (Cppi_Desc*)ptrPayload, 128);
    return 0;
}

int32_t Netfp_transmitInterface_FZM
(
    Netfp_Socket*           ptrNetfpSocket,
    Netfp_SockTxMetaInfo_FZM*   ptrSockTxMetaInfo,
    Ti_Pkt*                 ptrPayload
)
{
    Netfp_SockL2ConnectInfo* ptrL2ConnectInfo = &ptrNetfpSocket->connectInfo.l2Info[ptrSockTxMetaInfo->l2CfgIndex];
    /* Get the client handle */
    Netfp_ClientMCB* ptrNetfpClient = ptrNetfpSocket->ptrNetfpClient;
    if (ptrL2ConnectInfo->ifStatus == 0)
    {
        /* Interface has been administrativly brought down. The packet cannot be sent out
         * We cleanup the packet memory. */
        Pktlib_freePacket(ptrNetfpClient->cfg.pktlibInstHandle, ptrPayload);
        return 0;
    }
    uint32_t packetSize = Pktlib_getPacketLen(ptrPayload);

    /***********************************************************************************************
     * Ethernet Minimum Padding:
     ***********************************************************************************************/
    if (unlikely(packetSize < ETH_MIN_PKT_SIZE))
    {
        Ti_Pkt* ptrNextPayload = Pktlib_getNextPacket(ptrPayload);

        /* YES. Determine the number of bytes by which we need to extend the
         * packet size so that it fits the MINIMUM Ethernet Packet Size. */

        /* Is this a chained packet? */
        if (ptrNextPayload != NULL)
        {
            uint32_t bytes = ETH_MIN_PKT_SIZE - packetSize;
            /* Loop through the chained packets till the end. */
            while (Pktlib_getNextPacket(ptrNextPayload) != NULL)
                ptrNextPayload = Pktlib_getNextPacket(ptrNextPayload);

            /* For the last chained packet we increment the buffer length with the
             * number of bytes required to ensure that it fits the MIN Ethernet packet size. */
            Pktlib_setDataBufferLen(ptrNextPayload, Pktlib_getDataBufferLen(ptrNextPayload) + bytes);
        }
        else
        {
            /* NO. Simply set the new data buffer length to be MIN Ethernet packet size. */
            Pktlib_setDataBufferLen(ptrPayload, ETH_MIN_PKT_SIZE);
        }
        /* Set the packet length to be the MINIMUM Ethernet Size. */
        Pktlib_setPacketLen(ptrPayload, ETH_MIN_PKT_SIZE);
    }

    /* Attach the command in the protocol specific data of the descriptor. */
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)ptrPayload,
        (uint8_t *)(ptrNetfpSocket->paCommands[ptrSockTxMetaInfo->l2CfgIndex].cmdBuffer),
        ptrNetfpSocket->paCommands[ptrSockTxMetaInfo->l2CfgIndex].cmdBufferSize);

    /*********************************************************************************
     * Non Secure Socket: POST Routing Hook
     *********************************************************************************/
    if (Netfp_socketHook (Netfp_Hook_POST_ROUTING, ptrNetfpClient, ptrNetfpSocket, ptrPayload) < 0)
        return 0;

    /************************************************************************
     * Socket Statistics: Increment the NON-SECURE socket statistics for the
     * IPv4
     ************************************************************************/
    Netfp_incNonSecureIPv4Stats_FZM(ptrNetfpSocket, ptrSockTxMetaInfo, packetSize);

    /* We always push the packet to PDSP5 since the commands have been populated properly in the packet it will
     * find its way out through the configured command set to the switch port and out onto the wire. */
    Qmss_queuePushDescSize (ptrNetfpClient->netcpTxQueue[NSS_PA_QUEUE_TXCMD_INDEX],
        (Cppi_Desc*)ptrPayload, 128);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to add an IP address to the specific interface.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP server on which the IP address is to be added.
 *  @param[in]  ptrNetfpInterface
 *      Pointer to the interface to which IP address is being added
 *  @param[in]  ptrIPAddress
 *      IP Address to be added
 *  @param[in]  errCode
 *      Error code populated on error
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Netfp_addInterfaceIP
(
    Netfp_ServerMCB*    ptrNetfpServer,
    Netfp_Interface*    ptrNetfpInterface,
    Netfp_InterfaceIP*  ptrIPAddress,
    int32_t*            errCode
)
{
    int32_t             index;

    /* Sanity Check: Make sure that the subnet mask & ip address in the configuration belong to
     * the same family */
    if (ptrIPAddress->ipAddress.ver != ptrIPAddress->subnetMask.ver)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Check if the address being added is IPv4 or IPv6? */
    if (Netfp_isAddressIPv4(ptrIPAddress->ipAddress) == 1)
    {
        /* IPv4 Address is being added: There can be multiple IPv4 addresses being added on
         * the interface. So lets cycle through and make sure we can add this address */
        for (index = 0; index < NETFP_MAX_IP_ADDRESS; index++)
        {
            if (ptrNetfpInterface->ipv4[index].bIsValid != 0)
                continue;

            /* Entry was free; so lets initialize it. */
            ptrNetfpInterface->ipv4[index].interfaceIP.ipAddress  = ptrIPAddress->ipAddress;
            ptrNetfpInterface->ipv4[index].interfaceIP.subnetMask = ptrIPAddress->subnetMask;

            /* Mark the Interface to indicate that it has a valid IPv4 address on it */
            ptrNetfpInterface->ipv4[index].bIsValid = 1;

            /* Set the flag indicating that the interface is configured & operational now. */
            ptrNetfpInterface->status = ptrNetfpInterface->status | NETFP_INTERFACE_UP;
            break;
        }
    }
    else
    {
        /* IPv6 Address is being added: There can be multiple IPv6 addresses being added on
         * the interface. So lets cycle through and make sure we can add this address */
        for (index = 0; index < NETFP_MAX_IP_ADDRESS; index++)
        {
            /* Check if the entry is free or not? */
            if (ptrNetfpInterface->ipv6[index].bIsValid != 0)
                continue;

            /* Entry was free; so lets initialize it. */
            ptrNetfpInterface->ipv6[index].interfaceIP.ipAddress  = ptrIPAddress->ipAddress;
            ptrNetfpInterface->ipv6[index].interfaceIP.subnetMask = ptrIPAddress->subnetMask;

            /* Set the IPv6 address valid */
            ptrNetfpInterface->ipv6[index].bIsValid = 1;

            /* Set the flag indicating that the interface is configured & operational now. */
            ptrNetfpInterface->status = ptrNetfpInterface->status | NETFP_INTERFACE_UP;
            break;
        }
    }
    /* Successfully added the address to the interface */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to delete the specified IP address from the
 *      interface.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP server
 *  @param[in]  ptrNetfpInterface
 *      Pointer to the interface to which IP address is being removed
 *  @param[in]  ptrIPAddress
 *      IP Address to be deleted
 *  @param[in]  errCode
 *      Error code populated on error
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Netfp_delInterfaceIP
(
    Netfp_ServerMCB*    ptrNetfpServer,
    Netfp_Interface*    ptrNetfpInterface,
    Netfp_InterfaceIP*  ptrIPAddress,
    int32_t*            errCode
)
{
    int32_t     index;

    /* Sanity Check: Make sure that the subnet mask & ip address in the configuration belong to
     * the same family */
    if (ptrIPAddress->ipAddress.ver != ptrIPAddress->subnetMask.ver)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Check if the address being deleted is IPv4 or IPv6? */
    if (Netfp_isAddressIPv4(ptrIPAddress->ipAddress) == 1)
    {
        /* IPv4 Address is being deleted: Match the entry. */
        for (index = 0; index < NETFP_MAX_IP_ADDRESS; index++)
        {
            /* Check if the entry is free or not? */
            if (ptrNetfpInterface->ipv4[index].bIsValid == 0)
                continue;

            /* Is this the IPv4 address being deleted? */
            if ((Netfp_matchIP (&ptrNetfpInterface->ipv4[index].interfaceIP.ipAddress,  &ptrIPAddress->ipAddress)  == 0) ||
                (Netfp_matchIP (&ptrNetfpInterface->ipv4[index].interfaceIP.subnetMask, &ptrIPAddress->subnetMask) == 0))
            {
                /* No continue searching. */
                continue;
            }

            /* IPv4 address is no longer valid. */
            ptrNetfpInterface->ipv4[index].bIsValid = 0;
            break;
        }
        /* Did we find a matching address? */
        if (index == NETFP_MAX_IP_ADDRESS)
        {
            *errCode = NETFP_EINVAL;
            return -1;
        }
    }
    else
    {
        /* IPv6 Address is being deleted: Match the entry. */
        for (index = 0; index < NETFP_MAX_IP_ADDRESS; index++)
        {
            /* Check if the entry is free or not? */
            if (ptrNetfpInterface->ipv6[index].bIsValid == 0)
                continue;

            /* Is this the IPv6 address being deleted? */
            if ((Netfp_matchIP (&ptrNetfpInterface->ipv6[index].interfaceIP.ipAddress,  &ptrIPAddress->ipAddress)  == 0) ||
                (Netfp_matchIP (&ptrNetfpInterface->ipv6[index].interfaceIP.subnetMask, &ptrIPAddress->subnetMask) == 0))
            {
                /* No continue searching. */
                continue;
            }

            /* IPv6 address is no longer valid. */
            ptrNetfpInterface->ipv6[index].bIsValid = 0;
            break;
        }

        /* Did we find a matching address? */
        if (index == NETFP_MAX_IP_ADDRESS)
        {
            *errCode = NETFP_EINVAL;
            return -1;
        }
    }

    /* Is there any IP (IPv4 or IPv6) address left on the interface? */
    for (index = 0; index < NETFP_MAX_IP_ADDRESS; index++)
    {
        if (ptrNetfpInterface->ipv4[index].bIsValid != 0)
            return 1;
        if (ptrNetfpInterface->ipv6[index].bIsValid != 0)
            return 1;
    }

    /* Control comes here implies that there is no IP address left on the interface */
    ptrNetfpInterface->status = ptrNetfpInterface->status & (~NETFP_INTERFACE_UP);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Utility function which is used to find a physical interface matching the
 *      name
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP server
 *  @param[in]  ifName
 *      Interface name
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static Netfp_PhyInterface* Netfp_findPhysicalInterface
(
    Netfp_ServerMCB*    ptrNetfpServer,
    const char*         ifName
)
{
    Netfp_PhyInterface* ptrPhyInterface;

    /* Cycle through to determine if the physical interface exists */
    ptrPhyInterface = (Netfp_PhyInterface*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrPhyInterfaceList);
    while (ptrPhyInterface != NULL)
    {
        /* Do we have a match? We are NOT searching for a perfect match because it is possible to have an
         * interface eth0.1 offloaded while the real physical interface is eth0. */
        if (strstr (ifName, ptrPhyInterface->ifName) != NULL)
            return ptrPhyInterface;

        /* Get the next physical interface. */
        ptrPhyInterface = (Netfp_PhyInterface*)Netfp_listGetNext ((Netfp_ListNode*)ptrPhyInterface);
    }
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      The function is used to create the physical interface in the NETFP server.
 *      Physical interfaces are created and setup by the NETFP server through the
 *      NETFP master.
 *
 *  @param[in]  serverHandle
 *      NETFP Server handle
 *  @param[in]  ifName
 *      Interface name
 *  @param[in]  switchPortNum
 *      Switch port number on which the physical interface resides
 *  @param[in]  ptrInnerToOuterDSCPMap
 *      Pointer to the table on inner to outer DSCP marking
 *  @param[out] errCode
 *      Error code populated by the API
 *
 * \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_setupPhyInterface
(
    Netfp_ServerHandle  serverHandle,
    const char*         ifName,
    uint32_t            switchPortNum,
    uint8_t*            ptrInnerToOuterDSCPMap,
    int32_t*            errCode
)
{
    Netfp_ServerMCB*        ptrNetfpServer;
    Netfp_PhyInterface*     ptrPhyInterface;
    Netfp_EventMetaInfo     eventInfo;
    Netfp_Interface*        ptrNetfpInterface;
    uint8_t                 isMappingChanged    = 0;

    /* Get the server handle. */
    ptrNetfpServer = (Netfp_ServerMCB*)serverHandle;
    if (ptrNetfpServer == NULL)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Cycle through to determine if the physical interface exists. */
    ptrPhyInterface = Netfp_findPhysicalInterface (ptrNetfpServer, ifName);
    if (ptrPhyInterface == NULL)
    {
        /* YES. Create a new physical interface */
        ptrPhyInterface = ptrNetfpServer->cfg.malloc (sizeof(Netfp_PhyInterface), 0);
        if (ptrPhyInterface == NULL)
        {
            *errCode = NETFP_ENOMEM;
            return -1;
        }

        /* Initialize the allocated memory block: */
        memset ((void *)ptrPhyInterface, 0, sizeof(Netfp_PhyInterface));

        /* Populate the physical interface block: */
        ptrPhyInterface->switchPortNum = switchPortNum;
        strncpy (ptrPhyInterface->ifName, ifName, NETFP_MAX_CHAR);
        memcpy ((void *)&ptrPhyInterface->innerToOuterDSCPMap, (void*)ptrInnerToOuterDSCPMap, sizeof(ptrPhyInterface->innerToOuterDSCPMap));

        /* Add the physical interface to the list. */
        Netfp_listAdd ((Netfp_ListNode**)&ptrNetfpServer->ptrPhyInterfaceList, (Netfp_ListNode*)ptrPhyInterface);
        return 0;
    }

    /* Physical interface already exists. Determine which configuration changed */
    if (memcmp ((void *)&ptrPhyInterface->innerToOuterDSCPMap, (void*)ptrInnerToOuterDSCPMap, sizeof(ptrPhyInterface->innerToOuterDSCPMap) !=0))
        isMappingChanged = 1;

    /* Copy over the new configuration: */
    ptrPhyInterface->switchPortNum = switchPortNum;
    memcpy ((void *)&ptrPhyInterface->innerToOuterDSCPMap, (void*)ptrInnerToOuterDSCPMap, sizeof(ptrPhyInterface->innerToOuterDSCPMap));

    /* We need to cycle through all the interfaces and see if we need to propogate the interface marking map
     * changes to the NETFP Universe. Physical interfaces might be shared by multiple virtual interfaces */
    ptrNetfpInterface = (Netfp_Interface*)Netfp_listGetHead((Netfp_ListNode**)&ptrNetfpServer->ptrInterfaceList);
    while (ptrNetfpInterface != NULL)
    {
        /* Is this interface of interest? */
        if (ptrNetfpInterface->ptrPhyInterface == ptrPhyInterface)
        {
            /* YES. We need to generate the events. */
            if (isMappingChanged == 1)
            {
                /* DSCP Mapping has been modified: Generate the event and notify the world. */
                eventInfo.eventId           = Netfp_EventId_UPDATE_INTERFACE;
                eventInfo.u.ifMeta.ifHandle = (Netfp_IfHandle)ptrNetfpInterface;
                eventInfo.u.ifMeta.reason   = Netfp_EventIfReason_INNER_OUTER_DSCP;
                memcpy ((void *)&eventInfo.u.ifMeta.u.innerToOuterDSCPMap, (void*)ptrInnerToOuterDSCPMap, sizeof(ptrPhyInterface->innerToOuterDSCPMap));
                Netfp_generateEvent (ptrNetfpServer, &eventInfo);
            }
        }

        /* Cycle through and get the next interface */
        ptrNetfpInterface = (Netfp_Interface*)Netfp_listGetNext ((Netfp_ListNode*)ptrNetfpInterface);
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to rename a physical interface.
 *
 *  @param[in]  serverHandle
 *      NETFP Server handle
 *  @param[in]  oldIfName
 *      Old Interface name
 *  @param[in]  newIfName
 *      New Interface name
 *  @param[out] errCode
 *      Error code populated by the API
 *
 * \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_renamePhysicalInterface
(
    Netfp_ServerHandle  serverHandle,
    const char*         oldIfName,
    const char*         newIfName,
    int32_t*            errCode
)
{
    Netfp_PhyInterface*     ptrPhyInterface;
    Netfp_ServerMCB*        ptrNetfpServer;

    /* Get the NETFP Server MCB: */
    ptrNetfpServer = (Netfp_ServerMCB*)serverHandle;
    if (ptrNetfpServer == NULL)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Sanity Check: Ensure that the interfaces name are provided */
    if ((oldIfName == NULL) || (newIfName == NULL))
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Find the physical interface associated with the old interface name: */
    ptrPhyInterface = Netfp_findPhysicalInterface (ptrNetfpServer, oldIfName);
    if (ptrPhyInterface == NULL)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Simply copy over the new interface name */
    strncpy (ptrPhyInterface->ifName, newIfName, NETFP_MAX_CHAR);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to create & register a network interface with the NETFP server.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  ptrInterfaceCfg
 *      Interface Configuration
 *  @param[out]  errCode
 *      Error Code populated only on error
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   Handle to the NETFP Interface
 *  @retval
 *      Error   -   NULL
 */
static Netfp_IfHandle _Netfp_createInterface
(
    Netfp_ServerMCB*        ptrNetfpServer,
    Netfp_InterfaceCfg*     ptrInterfaceCfg,
    int32_t*                errCode
)
{
    Netfp_Interface*    ptrNetfpInterface;
    Netfp_InterfaceIP   ipAddressInfo;
    Netfp_PhyInterface* ptrPhyInterface;

    /* Sanity Check: Validate the arguments. */
    if ((ptrInterfaceCfg == NULL) || (ptrNetfpServer == NULL))
    {
        *errCode = NETFP_EINVAL;
        return NULL;
    }

    /* Sanity Check: Ensure that the IP address & subnet mask in the configuration are from the same family */
    if (ptrInterfaceCfg->ipAddress.ver != ptrInterfaceCfg->subnetMask.ver)
    {
        *errCode = NETFP_EINVAL;
        return NULL;
    }

    /* Check if we need to find the associated physical interface or not? */
    if (ptrInterfaceCfg->isLogicalInterface == 0)
    {
        /* Sanity Check: Ensure that we can find a physical interface matching the interface.  */
        ptrPhyInterface = Netfp_findPhysicalInterface (ptrNetfpServer, ptrInterfaceCfg->name);
        if (ptrPhyInterface == NULL)
        {
            *errCode = NETFP_EINVAL;
            return NULL;
        }
    }
    else
    {
        ptrPhyInterface = NULL;
    }

    /* Allocate memory for the NETFP Interface */
    ptrNetfpInterface = ptrNetfpServer->cfg.malloc (sizeof(Netfp_Interface), 0);
    if (ptrNetfpInterface == NULL)
    {
        *errCode = NETFP_ENOMEM;
        return NULL;
    }

    /* Initialize the allocated interface block. */
    memset ((void *)ptrNetfpInterface, 0, sizeof(Netfp_Interface));

    /* Copy the interface configuration. */
    memcpy ((void *)&ptrNetfpInterface->cfg, (void *)ptrInterfaceCfg, sizeof(Netfp_InterfaceCfg));

    /* Setup the header size on the basis of the interface type. */
    if (ptrInterfaceCfg->type == Netfp_InterfaceType_VLAN)
        ptrNetfpInterface->headerSize = ETHHDR_SIZE + VLANHDR_SIZE;
    else
        ptrNetfpInterface->headerSize = ETHHDR_SIZE;

    /* Populate the Interface IP Address Information. */
    ipAddressInfo.ipAddress.ver   = ptrNetfpInterface->cfg.ipAddress.ver;
    ipAddressInfo.ipAddress       = ptrNetfpInterface->cfg.ipAddress;
    ipAddressInfo.subnetMask.ver  = ptrNetfpInterface->cfg.subnetMask.ver;
    ipAddressInfo.subnetMask      = ptrNetfpInterface->cfg.subnetMask;

    /* Populate the interface block. */
    ptrNetfpInterface->status          = NETFP_INTERFACE_UP;
    ptrNetfpInterface->ptrNetfpServer  = ptrNetfpServer;
    ptrNetfpInterface->ptrPhyInterface = ptrPhyInterface;

    /* Add the IP address to the interface. */
    if (Netfp_addInterfaceIP(ptrNetfpServer, ptrNetfpInterface, &ipAddressInfo, errCode) < 0)
    {
        ptrNetfpServer->cfg.free (ptrNetfpInterface, sizeof(Netfp_Interface));
        return NULL;
    }

    /* Debug Message: */
    if (ptrPhyInterface == NULL)
        Netfp_logMsg (ptrNetfpServer, Netfp_LogLevel_DEBUG, "Debug: Created interface %s\n", ptrInterfaceCfg->name);
    else
        Netfp_logMsg (ptrNetfpServer, Netfp_LogLevel_DEBUG, "Debug: Linking interface %s with physical Interface %s\n",
                      ptrInterfaceCfg->name, ptrPhyInterface->ifName);

    /* Add the list to the global interface list. */
    Netfp_listAdd ((Netfp_ListNode**)&ptrNetfpServer->ptrInterfaceList, (Netfp_ListNode*)ptrNetfpInterface);
    return (Netfp_IfHandle)ptrNetfpInterface;
}

/**
 *  @b Description
 *  @n
 *      The function is used to delete an interface from the NETFP Server.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  ifHandle
 *      Interface handle to be deleted
 *  @param[out]  errCode
 *      Error Code populated only on error
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t _Netfp_deleteInterface
(
    Netfp_ServerMCB*        ptrNetfpServer,
    Netfp_IfHandle          ifHandle,
    int32_t*                errCode
)
{
    Netfp_Interface*    ptrNetfpInterface;
    int32_t             index;

    /* Sanity Check: Validate the arguments. */
    if ((ifHandle == NULL) || (ptrNetfpServer == NULL))
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Get the interface information. */
    ptrNetfpInterface = (Netfp_Interface*)ifHandle;

    /* Sanity Check: Is this a valid interface? */
    if (Netfp_isValidInterface(ptrNetfpServer, ptrNetfpInterface) == 0)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Cycle through and delete all the IPv4 Addresses */
    for (index = 0; index < NETFP_MAX_IP_ADDRESS; index++)
    {
        /* Check if the entry is free or not? */
        if (ptrNetfpInterface->ipv4[index].bIsValid != 0)
            Netfp_delInterfaceIP(ptrNetfpServer, ptrNetfpInterface, &ptrNetfpInterface->ipv4[index].interfaceIP, errCode);
    }

    /* Cycle through and delete all the IPv6 Addresses */
    for (index = 0; index < NETFP_MAX_IP_ADDRESS; index++)
    {
        /* Check if the entry is free or not? */
        if (ptrNetfpInterface->ipv6[index].bIsValid != 0)
            Netfp_delInterfaceIP(ptrNetfpServer, ptrNetfpInterface, &ptrNetfpInterface->ipv6[index].interfaceIP, errCode);
    }

    /* Remove the interface from the global interface list. */
    Netfp_listRemoveNode ((Netfp_ListNode**)&ptrNetfpServer->ptrInterfaceList, (Netfp_ListNode*)ptrNetfpInterface);

    /* Update the non-secure fast paths */
    Netfp_updateFP (ptrNetfpServer, Netfp_Reason_INTERFACE_DELETE, NETFP_INVALID_SPID, ptrNetfpInterface);

    /* Update the outbound security associations */
    Netfp_updateSA (ptrNetfpServer, Netfp_Reason_INTERFACE_DELETE, ptrNetfpInterface);

    /* Cleanup the allocated memory block */
    ptrNetfpServer->cfg.free (ptrNetfpInterface, sizeof(Netfp_Interface));
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to find a network interface using the specified name
 *      on the NETFP Server.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  name
 *      Network Interface Name
 *  @param[out] ptrInterfaceCfg
 *      Pointer to the interface configuration populated if the interface exists.
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   Pointer to the interface
 *  @retval
 *      Error   -   NULL
 */
Netfp_Interface* _Netfp_findInterface
(
    Netfp_ServerMCB*    ptrNetfpServer,
    const char*         name,
    Netfp_InterfaceCfg* ptrInterfaceCfg
)
{
    Netfp_Interface*    ptrNetfpInterface;

    /* Sanity Check: Validate the arguments. */
    if ((name == NULL) || (ptrNetfpServer == NULL))
        return NULL;

    /* Cycle through all the interfaces. */
    ptrNetfpInterface = (Netfp_Interface*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrInterfaceList);
    while (ptrNetfpInterface != NULL)
    {
        /* Match the name */
        if (strcmp(ptrNetfpInterface->cfg.name, name) == 0)
        {
            /* Copy the interface configuration. */
            memcpy ((void *)ptrInterfaceCfg, (void *)&ptrNetfpInterface->cfg, sizeof(Netfp_InterfaceCfg));
            return ptrNetfpInterface;
        }

        /* Get the next interface */
        ptrNetfpInterface = (Netfp_Interface*)Netfp_listGetNext((Netfp_ListNode*)ptrNetfpInterface);
    }

    /* Control comes here implies that there was no matching entry found. */
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the interface configuration of the specific interface
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  ifHandle
 *      Handle to the interface
 *  @param[out] ptrOptInfo
 *      Option information which is populated by the API
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t _Netfp_getIfOpt
(
    Netfp_ServerMCB*        ptrNetfpServer,
    Netfp_IfHandle          ifHandle,
    Netfp_OptionTLV*        ptrOptInfo,
    int32_t*                errCode
)
{
    Netfp_Interface*    ptrNetfpInterface;
    uint32_t            index;
    uint32_t            numIPv6Address;
    Netfp_InterfaceIP*  ptrInterfaceIP;

    /* Get the interface handle */
    ptrNetfpInterface = (Netfp_Interface *)ifHandle;
    if ((ptrNetfpInterface == NULL) || (ptrNetfpServer == NULL) || (ptrOptInfo == NULL))
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Sanity Check: Is this a valid interface? */
    if (Netfp_isValidInterface(ptrNetfpServer, ptrNetfpInterface) == 0)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* The TLV has the value which is actually a pointer which points to the start of the actual data
     * Pointers cannot be handled natively by the JOSH framework. We had translated the value pointer
     * to now be moved after the TLV. So here we update this pointer appropriately. */
    ptrOptInfo->value = (uint8_t*)ptrOptInfo + sizeof(Netfp_OptionTLV);

    /* Process only the valid Interface Level Options. */
    switch (ptrOptInfo->type)
    {
        case Netfp_Option_MTU:
        {
            /* Sanity Check: Validate and ensure that the arguments are sized as documented. */
            if (ptrOptInfo->length != 4)
            {
                *errCode = NETFP_EINVAL;
                return -1;
            }

            /* Get the interface MTU. */
            *(uint32_t*)ptrOptInfo->value = ptrNetfpInterface->cfg.mtu;
            ptrOptInfo->length = 4;
            break;
        }
        case Netfp_Option_IFACE_STATUS:
        {
            /* Sanity Check: Validate and ensure that the arguments are sized as documented. */
            if (ptrOptInfo->length != 4)
            {
                *errCode = NETFP_EINVAL;
                return -1;
            }

            /* Get the interface Status */
            *(uint32_t*)ptrOptInfo->value = ptrNetfpInterface->status;
            ptrOptInfo->length = 4;
            break;
        }
        case Netfp_Option_GET_IPv6:
        {
            /* Sanity Check: Ensure that the length field is correctly configured */
            if (ptrOptInfo->length != (sizeof(Netfp_InterfaceIP) * NETFP_MAX_IP_ADDRESS))
            {
                *errCode = NETFP_EINVAL;
                return -1;
            }

            /* Get the pointer to the IP address list which is to be populated */
            ptrInterfaceIP = (Netfp_InterfaceIP*)ptrOptInfo->value;

            /* No IPv6 address detected so far. */
            numIPv6Address = 0;

            /* Cycle through all the IPv6 address which can be configured on the system */
            for (index = 0; index < NETFP_MAX_IP_ADDRESS; index++)
            {
                /* Is a valid IPv6 address configured? */
                if (ptrNetfpInterface->ipv6[index].bIsValid == 0)
                    continue;

                /* Copy the IPv6 Interface configuration */
                memcpy ((void *)ptrInterfaceIP, (void *)&ptrNetfpInterface->ipv6[index], sizeof(Netfp_InterfaceIP));

                /* Account for the IPv6 address */
                ptrInterfaceIP++;
                numIPv6Address++;
            }

            /* Populate the length value appropriately with the number of IPv6 address which have been configured, */
            ptrOptInfo->length = sizeof(Netfp_InterfaceIP) * numIPv6Address;
            break;
        }
        case Netfp_Option_VLAN_EGRESS_PRIORITY:
        {
            /* Sanity Check: Ensure that the length field is correctly configured */
            if (ptrOptInfo->length != sizeof(Netfp_VLANPriorityMap))
            {
                *errCode = NETFP_EINVAL;
                return -1;
            }

            /* Sanity Check: Ensure that the interface is a VLAN enabled interface */
            if (ptrNetfpInterface->cfg.type != Netfp_InterfaceType_VLAN)
            {
                *errCode = NETFP_EINVAL;
                return -1;
            }

            /* Copy over the mapping: */
            memcpy ((void *)ptrOptInfo->value, (void*)&ptrNetfpInterface->cfg.vlanMap, sizeof(Netfp_VLANPriorityMap));
            break;
        }
        case Netfp_Option_L3_QOS:
        {
            /* Sanity Check: Ensure that the length field is correctly configured */
            if (ptrOptInfo->length != sizeof(Netfp_L3QoSCfg))
            {
                *errCode = NETFP_EINVAL;
                return -1;
            }

            /* Copy over the L3 QOS Configuration: */
            memcpy ((void *)ptrOptInfo->value, (void*)&ptrNetfpInterface->l3QosCfg, sizeof(Netfp_L3QoSCfg));
            break;
        }
        default:
        {
            /* Option is NOT supported */
            *errCode = NETFP_EINVAL;
            return -1;
        }
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to modify configuration of the specific interface
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  ifHandle
 *      Handle to the interface
 *  @param[out] ptrOptInfo
 *      Option information which is populated by the API
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t _Netfp_setIfOpt
(
    Netfp_ServerMCB*        ptrNetfpServer,
    Netfp_IfHandle          ifHandle,
    Netfp_OptionTLV*        ptrOptInfo,
    int32_t*                errCode
)
{
    Netfp_Interface*        ptrNetfpInterface;
    Netfp_EventMetaInfo     eventInfo;

    /* Get the interface handle */
    ptrNetfpInterface = (Netfp_Interface *)ifHandle;
    if ((ptrNetfpInterface == NULL) || (ptrNetfpServer == NULL) || (ptrOptInfo == NULL))
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Sanity Check: Is this a valid interface? */
    if (Netfp_isValidInterface(ptrNetfpServer, ptrNetfpInterface) == 0)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* The TLV has the value which is actually a pointer which points to the start of the actual data
     * Pointers cannot be handled natively by the JOSH framework. We had translated the value pointer
     * to now be moved after the TLV. So here we update this pointer appropriately. */
    ptrOptInfo->value = (uint8_t*)ptrOptInfo + sizeof(Netfp_OptionTLV);

    /* Process only the valid Interface Level Options. */
    switch (ptrOptInfo->type)
    {
        case Netfp_Option_MTU:
        {
            /* Sanity Check: Validate and ensure that the arguments are sized as documented. */
            if (ptrOptInfo->length != 4)
            {
                *errCode = NETFP_EINVAL;
                return -1;
            }

            /* Set the interface MTU. */
            ptrNetfpInterface->cfg.mtu = *(uint32_t*)ptrOptInfo->value;

            /* Update the non-secure fast paths with the new interface MTU */
            Netfp_updateFP (ptrNetfpServer, Netfp_Reason_IF_MTU_CHANGE, NETFP_INVALID_SPID, ptrNetfpInterface);

            /* Update the outbound security associations with the new interface MTU */
            Netfp_updateSA (ptrNetfpServer, Netfp_Reason_IF_MTU_CHANGE, ptrNetfpInterface);
            break;
        }
        case Netfp_Option_IFACE_STATUS:
        {
            /* Sanity Check: Validate and ensure that the arguments are sized as documented. */
            if (ptrOptInfo->length != 4)
            {
                *errCode = NETFP_EINVAL;
                return -1;
            }

            /* Update the interface status */
            ptrNetfpInterface->status = *(uint32_t*)ptrOptInfo->value;

            /* Populate the event information: We need to send an update to the all the clients
             * to indicate that the interface status has been changed. */
            eventInfo.eventId           = Netfp_EventId_UPDATE_INTERFACE;
            eventInfo.u.ifMeta.ifHandle = (Netfp_IfHandle)ptrNetfpInterface;
            eventInfo.u.ifMeta.reason   = Netfp_EventIfReason_STATUS;
            eventInfo.u.ifMeta.u.status = ptrNetfpInterface->status;
            Netfp_generateEvent (ptrNetfpServer, &eventInfo);
            break;
        }
        case Netfp_Option_ADD_IP:
        {
            /* Sanity Check: Ensure that the length field is correctly configured */
            if (ptrOptInfo->length != sizeof(Netfp_InterfaceIP))
            {
                *errCode = NETFP_EINVAL;
                return -1;
            }

            /* Add the IP address to the interface. */
            if (Netfp_addInterfaceIP(ptrNetfpServer, ptrNetfpInterface, ptrOptInfo->value, errCode) < 0)
                return -1;
            break;
        }
        case Netfp_Option_DEL_IP:
        {
            /* Sanity Check: Ensure that the length field is correctly configured */
            if (ptrOptInfo->length != sizeof(Netfp_InterfaceIP))
            {
                *errCode = NETFP_EINVAL;
                return -1;
            }

            /* Delete the IP address from the interface. */
            if (Netfp_delInterfaceIP(ptrNetfpServer, ptrNetfpInterface, ptrOptInfo->value, errCode) < 0)
                return -1;
            break;
        }
        case Netfp_Option_VLAN_EGRESS_PRIORITY:
        {
            /* Sanity Check: Ensure that the length field is correctly configured */
            if (ptrOptInfo->length != sizeof(Netfp_VLANPriorityMap))
            {
                *errCode = NETFP_EINVAL;
                return -1;
            }

            /* Sanity Check: Ensure that the interface is a VLAN enabled interface */
            if (ptrNetfpInterface->cfg.type != Netfp_InterfaceType_VLAN)
            {
                *errCode = NETFP_EINVAL;
                return -1;
            }

            /* Copy over the new mapping: */
            memcpy ((void *)&ptrNetfpInterface->cfg.vlanMap, (void*)ptrOptInfo->value, sizeof(Netfp_VLANPriorityMap));

            /* Populate the event information: We need to send an update to the all the clients
             * to indicate that the VLAN egress mapping has been modified. */
            eventInfo.eventId           = Netfp_EventId_UPDATE_INTERFACE;
            eventInfo.u.ifMeta.ifHandle = (Netfp_IfHandle)ptrNetfpInterface;
            eventInfo.u.ifMeta.reason   = Netfp_EventIfReason_VLAN_PBIT;
            memcpy ((void*)&eventInfo.u.ifMeta.u.vlanMap, (void *)&ptrNetfpInterface->cfg.vlanMap,  sizeof(Netfp_VLANPriorityMap));
            Netfp_generateEvent (ptrNetfpServer, &eventInfo);
            break;
        }
        case Netfp_Option_L3_QOS:
        {
            /* Sanity Check: Ensure that the length field is correctly configured */
            if (ptrOptInfo->length != sizeof(Netfp_L3QoSCfg))
            {
                *errCode = NETFP_EINVAL;
                return -1;
            }

            /* Copy over the L3 QoS map to the interface: */
            memcpy ((void *)&ptrNetfpInterface->l3QosCfg, (void*)ptrOptInfo->value, sizeof(Netfp_L3QoSCfg));

            /* Populate the Event information: */
            eventInfo.eventId           = Netfp_EventId_UPDATE_INTERFACE;
            eventInfo.u.ifMeta.ifHandle = (Netfp_IfHandle)ptrNetfpInterface;
            eventInfo.u.ifMeta.reason   = Netfp_EventIfReason_L3_QOS;
            memcpy ((void *)&eventInfo.u.ifMeta.u.l3QosCfg, (void *)ptrOptInfo->value, sizeof(Netfp_L3QoSCfg));

            /* Notify the clients and sockets. */
            Netfp_generateEvent (ptrNetfpServer, &eventInfo);
            break;
        }
        default:
        {
            /* Option is NOT supported */
            *errCode = NETFP_EINVAL;
            return -1;
        }
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is invoked by the NETFP client to find an interface.
 *
 *  @param[in]  clientHandle
 *      NETFP Client handle
 *  @param[in]  ifName
 *      Network Interface Name
 *  @param[out] ptrInterfaceCfg
 *      Optional pointer to the interface configuration block. This is populated
 *      with the interface configuration if the specified interface name exists i.e.
 *      the function returns a valid interface handle.
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   Pointer to the interface
 *  @retval
 *      Error   -   NULL
 */
Netfp_IfHandle Netfp_findInterface
(
    Netfp_ClientHandle  clientHandle,
    const char*         ifName,
    Netfp_InterfaceCfg* ptrInterfaceCfg,
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
    if ((clientHandle == NULL) || (ifName == NULL))
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
    jobHandle = Josh_findJobByAddress(ptrNetfpClient->joshHandle, (Josh_JobProtype)_Netfp_findInterface);
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
     *  Netfp_IfHandle _Netfp_findInterface
     *  (
     *  Netfp_ServerMCB*        ptrNetfpServer,
     *  char*                   name,
     *  Netfp_InterfaceCfg*     ptrInterfaceCfg
     *  )
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Netfp_ServerHandle);

    /*  - Argument 2: */
    args[1].type   = Josh_ArgumentType_PASS_BY_REF;
    args[1].length = strlen (ifName) + 1; // fzm: https://e2eprivate.ti.com/nokia_siemens_networks/k2_-_fsm4_-_fzm_-_lrc_nokia/f/191/t/7108.aspx

    /*  - Argument 3: */
    args[2].type   = Josh_ArgumentType_PASS_BY_REF;
    args[2].length = sizeof (Netfp_InterfaceCfg);

    /* Add the arguments. */
    argHandle = Josh_addArguments (ptrNetfpClient->joshHandle, &args[0]);
    if (argHandle == NULL)
    {
        *errCode = NETFP_EJOSH;
        return NULL;
    }

    /* Populate the NETFP Server handle */
    *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(ptrNetfpClient->cfg.serverHandle);

    /* Copy the interface name. */
    strncpy ((void*)args[1].argBuffer, ifName, strlen (ifName) + 1); // fzm: https://e2eprivate.ti.com/nokia_siemens_networks/k2_-_fsm4_-_fzm_-_lrc_nokia/f/191/t/7108.aspx

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

    /* Copy over the interface configuration only if requested to do so. */
    if (ptrInterfaceCfg != NULL)
        memcpy ((void *)ptrInterfaceCfg, (void *)args[2].argBuffer, sizeof(Netfp_InterfaceCfg));

    /* Free the JOB Instance. */
    if (Josh_freeJobInstance(ptrNetfpClient->joshHandle, jobId) < 0)
    {
        *errCode = NETFP_EJOSH;
        return NULL;
    }
    return (Netfp_IfHandle)result;
}

/**
 *  @b Description
 *  @n
 *      The service function which is invoked by the NETFP Client to create an interface
 *
 *  @param[in]  clientHandle
 *      NETFP Client Handle
 *  @param[in]  ptrInterfaceCfg
 *      Pointer to the interface configuration
 *  @param[out] errCode
 *      Error code populated on error.
 *
 * \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   Interface handle
 *  @retval
 *      Error   -   NULL
 */
Netfp_IfHandle Netfp_createInterface
(
    Netfp_ClientHandle  clientHandle,
    Netfp_InterfaceCfg* ptrInterfaceCfg,
    int32_t*            errCode
)
{
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    uint32_t                result;
    int32_t                 jobId;
    Netfp_ClientMCB*        ptrNetfpClient;
    Netfp_InterfaceCfg*     ptrRemoteInterfaceCfg;

    /* Sanity Check: Ensure that the arguments are valid */
    if ((clientHandle == NULL) || (ptrInterfaceCfg == NULL))
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
    jobHandle = Josh_findJobByAddress(ptrNetfpClient->joshHandle, (Josh_JobProtype)_Netfp_createInterface);
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
     *  Netfp_IfHandle _Netfp_createInterface
     *  (
     *  Netfp_ServerMCB*        ptrNetfpServer,
     *  Netfp_InterfaceCfg*     ptrInterfaceCfg,
     *  int32_t*                errCode
     *  )
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Netfp_ServerHandle);

    /*  - Argument 2: */
    args[1].type   = Josh_ArgumentType_PASS_BY_REF;
    args[1].length = sizeof(Netfp_InterfaceCfg);

    /*  - Argument 3: */
    args[2].type   = Josh_ArgumentType_PASS_BY_REF;
    args[2].length = sizeof(int32_t);

    /* Add the arguments. */
    argHandle = Josh_addArguments (ptrNetfpClient->joshHandle, &args[0]);
    if (argHandle == NULL)
    {
        *errCode = NETFP_EJOSH;
        return NULL;
    }

    /* Populate the NETFP Server handle */
    *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(ptrNetfpClient->cfg.serverHandle);

    /* Get the pointer to the interface configuration. */
    ptrRemoteInterfaceCfg = (Netfp_InterfaceCfg*)args[1].argBuffer;
    memcpy ((void*)ptrRemoteInterfaceCfg, (void *)ptrInterfaceCfg, sizeof (Netfp_InterfaceCfg));

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
    *errCode = *((int32_t*)args[2].argBuffer);

    /* Free the JOB Instance. */
    if (Josh_freeJobInstance(ptrNetfpClient->joshHandle, jobId) < 0)
    {
        *errCode = NETFP_EJOSH;
        return NULL;
    }
    return (Netfp_IfHandle)result;
}

/**
 *  @b Description
 *  @n
 *      The service function which is invoked by the NETFP Client to delete an
 *      interface
 *
 *  @param[in]  clientHandle
 *      NETFP Client Handle
 *  @param[in]  ifHandle
 *      Handle to the interface to be deleted
 *  @param[out] errCode
 *      Error code populated on error.
 *
 * \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_deleteInterface
(
    Netfp_ClientHandle  clientHandle,
    Netfp_IfHandle      ifHandle,
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
    if ((clientHandle == NULL) || (ifHandle == NULL))
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
    jobHandle = Josh_findJobByAddress(ptrNetfpClient->joshHandle, (Josh_JobProtype)_Netfp_deleteInterface);
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
     *  Netfp_IfHandle _Netfp_deleteInterface
     *  (
     *  Netfp_ServerMCB*        ptrNetfpServer,
     *  Netfp_IfHandle          ifHandle,
     *  int32_t*                errCode
     *  )
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Netfp_ServerHandle);

    /*  - Argument 2: */
    args[1].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[1].length = sizeof(Netfp_IfHandle);

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

    /* Populate the NETFP Server handle */
    *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(ptrNetfpClient->cfg.serverHandle);

    /* Populate the interface handle. */
    *(uint32_t*)args[1].argBuffer = (uint32_t)josh_toRemoteU32(ifHandle);

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
    return result;
}

/**
 *  @b Description
 *  @n
 *      The service function which is invoked by the NETFP Client to get the
 *      configuration of the specific interface.
 *
 *  @param[in]  clientHandle
 *      NETFP Client Handle
 *  @param[in]  ifHandle
 *      Handle to the interface to be deleted
 *  @param[in]  ptrOptInfo
 *      TLV Option information
 *  @param[out] errCode
 *      Error code populated on error.
 *
 * \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_getIfOpt
(
    Netfp_ClientHandle  clientHandle,
    Netfp_IfHandle      ifHandle,
    Netfp_OptionTLV*    ptrOptInfo,
    int32_t*            errCode
)
{
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    uint32_t                result;
    int32_t                 jobId;
    Netfp_ClientMCB*        ptrNetfpClient;
    Netfp_OptionTLV*        ptrRemoteOptInfo;

    /* Sanity Check: Ensure that the arguments are valid */
    if ((clientHandle == NULL) || (ifHandle == NULL) || (ptrOptInfo == NULL))
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
    jobHandle = Josh_findJobByAddress(ptrNetfpClient->joshHandle, (Josh_JobProtype)_Netfp_getIfOpt);
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
     *  int32_t _Netfp_getIfOpt
     *  (
     *  Netfp_ServerMCB*        ptrNetfpServer,
     *  Netfp_IfHandle          ifHandle,
     *  Netfp_OptionTLV*        ptrOptInfo,
     *  int32_t*                errCode
     *  )
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Netfp_ServerHandle);

    /*  - Argument 2: */
    args[1].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[1].length = sizeof(Netfp_IfHandle);

    /*  - Argument 3: We need to account for the length of the optional parameters */
    args[2].type   = Josh_ArgumentType_PASS_BY_REF;
    args[2].length = sizeof(Netfp_OptionTLV) + ptrOptInfo->length;

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

    /* Populate the NETFP Server handle */
    *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(ptrNetfpClient->cfg.serverHandle);

    /* Populate the interface handle. */
    *(uint32_t*)args[1].argBuffer = (uint32_t)josh_toRemoteU32(ifHandle);

    /* Copy the option information
     * The value in the remote option information should point immediately after the
     * TLV data.   */
    ptrRemoteOptInfo = (Netfp_OptionTLV*)args[2].argBuffer;
    memcpy ((void*)ptrRemoteOptInfo, (void *)ptrOptInfo, sizeof(Netfp_OptionTLV));
    ptrRemoteOptInfo->value = (void*)((uint32_t)args[2].argBuffer + sizeof(Netfp_OptionTLV));
    memcpy ((void*)ptrRemoteOptInfo->value, (void *)ptrOptInfo->value, ptrOptInfo->length);

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

    /* Copy the option information back from the response. The remote
     * option "value" resides immediately after the TLV data structure. */
    ptrRemoteOptInfo = (Netfp_OptionTLV*)args[2].argBuffer;
    ptrRemoteOptInfo->value = (void*)((uint32_t)args[2].argBuffer + sizeof(Netfp_OptionTLV));
    memcpy ((void*)ptrOptInfo->value, (void *)ptrRemoteOptInfo->value, ptrRemoteOptInfo->length);

    /* Free the JOB Instance. */
    if (Josh_freeJobInstance(ptrNetfpClient->joshHandle, jobId) < 0)
    {
        *errCode = NETFP_EJOSH;
        return -1;
    }
    return result;
}

/**
 *  @b Description
 *  @n
 *      The service function which is invoked by the NETFP Client to set the
 *      configuration of the specific interface.
 *
 *  @param[in]  clientHandle
 *      NETFP Client Handle
 *  @param[in]  ifHandle
 *      Handle to the interface to be deleted
 *  @param[in]  ptrOptInfo
 *      TLV Option information
 *  @param[out] errCode
 *      Error code populated on error.
 *
 * \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_setIfOpt
(
    Netfp_ClientHandle  clientHandle,
    Netfp_IfHandle      ifHandle,
    Netfp_OptionTLV*    ptrOptInfo,
    int32_t*            errCode
)
{
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    uint32_t                result;
    int32_t                 jobId;
    Netfp_ClientMCB*        ptrNetfpClient;
    Netfp_OptionTLV*        ptrRemoteOptInfo;

    /* Sanity Check: Ensure that the arguments are valid */
    if ((clientHandle == NULL) || (ifHandle == NULL) || (ptrOptInfo == NULL))
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
    jobHandle = Josh_findJobByAddress(ptrNetfpClient->joshHandle, (Josh_JobProtype)_Netfp_setIfOpt);
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
     *  int32_t _Netfp_setIfOpt
     *  (
     *  Netfp_ServerMCB*        ptrNetfpServer,
     *  Netfp_IfHandle          ifHandle,
     *  Netfp_OptionTLV*        ptrOptInfo,
     *  int32_t*                errCode
     *  )
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Netfp_ServerHandle);

    /*  - Argument 2: */
    args[1].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[1].length = sizeof(Netfp_IfHandle);

    /*  - Argument 3: We need to account for the length of the optional parameters */
    args[2].type   = Josh_ArgumentType_PASS_BY_REF;
    args[2].length = sizeof(Netfp_OptionTLV) + ptrOptInfo->length;

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

    /* Populate the NETFP Server handle */
    *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(ptrNetfpClient->cfg.serverHandle);

    /* Populate the interface handle. */
    *(uint32_t*)args[1].argBuffer = (uint32_t)josh_toRemoteU32(ifHandle);

    /* Copy the option information:
     *  This needs to be done a bit differently since in the original TLV the value
     *  is actually a pointer which points to a different memory location. However
     *  when we send this over JOSH we compact this data structure and ensure that
     *  the pointer is pointing just after the OptionTLV. */
    ptrRemoteOptInfo = (Netfp_OptionTLV*)args[2].argBuffer;
    memcpy ((void*)ptrRemoteOptInfo, (void *)ptrOptInfo, sizeof(Netfp_OptionTLV));
    ptrRemoteOptInfo->value = (void*)((uint32_t)args[2].argBuffer + sizeof(Netfp_OptionTLV));
    memcpy ((void*)ptrRemoteOptInfo->value, (void *)ptrOptInfo->value, ptrOptInfo->length);

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
    return result;
}

/**
 *  @b Description
 *  @n
 *      The function is used to kill all the interfaces registered
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
int32_t Netfp_killInterface
(
    Netfp_ServerMCB*    ptrNetfpServer,
    int32_t*            errCode
)
{
    Netfp_Interface*    ptrNetfpInterface;

    /* Close all the interfaces which have been configured in the server */
    ptrNetfpInterface = (Netfp_Interface*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrInterfaceList);
    while (ptrNetfpInterface != NULL)
    {
        /* Delete the interface from the server. */
        if (_Netfp_deleteInterface (ptrNetfpServer, ptrNetfpInterface, errCode) < 0)
            return -1;

        /* Get the next interface */
        ptrNetfpInterface = (Netfp_Interface*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrInterfaceList);
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is a debug function to display the contents of all the interfaces
 *      which are present in the system.
 *
 *  @param[in]  serverHandle
 *      Handle to the NETFP Server
 *
 *  \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Number of interfaces which are present
 */
int32_t Netfp_displayInterface(Netfp_ServerHandle serverHandle)
{
    Netfp_Interface*    ptrNetfpInterface;
    Netfp_PhyInterface* ptrPhyInterface;
    Netfp_ServerMCB*    ptrNetfpServer;
    uint32_t            index;
    int32_t             count = 0;
    char                strIPv6Address[40];
    char                strSubnetMask[40];

    /* Get the server handle. */
    ptrNetfpServer = (Netfp_ServerMCB*)serverHandle;
    if (ptrNetfpServer == NULL)
        return 0;

//fzm - use Netfp_dumpMsg to capture this with the USR1 signal
    /* Display the banner. */
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "**********************************************************\n");
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,  "Interface List: \n");

    /* Cycle through all the fast paths and display them on the console */
    ptrNetfpInterface = (Netfp_Interface*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrInterfaceList);
    while (ptrNetfpInterface != NULL)
    {
        Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,
                      "%s     Link encap:%s   %s HWaddr %02x:%02x:%02x:%02x:%02x:%02x MTU %d Physical 0x%x\n",
                      ptrNetfpInterface->cfg.name,
                      (ptrNetfpInterface->cfg.type == Netfp_InterfaceType_ETH) ? "Ethernet" : "VLAN",
                      (ptrNetfpInterface->status & NETFP_INTERFACE_UP)         ? "UP"     : "DOWN",
                      ptrNetfpInterface->cfg.macAddress[0], ptrNetfpInterface->cfg.macAddress[1],
                      ptrNetfpInterface->cfg.macAddress[2], ptrNetfpInterface->cfg.macAddress[3],
                      ptrNetfpInterface->cfg.macAddress[4], ptrNetfpInterface->cfg.macAddress[5],
                      ptrNetfpInterface->cfg.mtu,
                      (ptrNetfpInterface->ptrPhyInterface != NULL) ? ptrNetfpInterface->ptrPhyInterface : NULL);

        /* Cycle through all the IPv6 address and display them */
        for (index = 0; index < NETFP_MAX_IP_ADDRESS; index++)
        {
            /* Display the IPv4 address if configured */
            if (ptrNetfpInterface->ipv4[index].bIsValid != 0)
            {
                Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,
                              "       inet addr: %d.%d.%d.%d   Mask %d.%d.%d.%d\n",
                              ptrNetfpInterface->ipv4[index].interfaceIP.ipAddress.addr.ipv4.u.a8[0],
                              ptrNetfpInterface->ipv4[index].interfaceIP.ipAddress.addr.ipv4.u.a8[1],
                              ptrNetfpInterface->ipv4[index].interfaceIP.ipAddress.addr.ipv4.u.a8[2],
                              ptrNetfpInterface->ipv4[index].interfaceIP.ipAddress.addr.ipv4.u.a8[3],
                              ptrNetfpInterface->ipv4[index].interfaceIP.subnetMask.addr.ipv4.u.a8[0],
                              ptrNetfpInterface->ipv4[index].interfaceIP.subnetMask.addr.ipv4.u.a8[1],
                              ptrNetfpInterface->ipv4[index].interfaceIP.subnetMask.addr.ipv4.u.a8[2],
                              ptrNetfpInterface->ipv4[index].interfaceIP.subnetMask.addr.ipv4.u.a8[3]);
            }
        }

        /* Cycle through all the IPv6 address and display them */
        for (index = 0; index < NETFP_MAX_IP_ADDRESS; index++)
        {
            if (ptrNetfpInterface->ipv6[index].bIsValid != 0)
            {
                Netfp_convertIP6ToStr (ptrNetfpInterface->ipv6[index].interfaceIP.ipAddress.addr.ipv6,  &strIPv6Address[0]);
                Netfp_convertIP6ToStr (ptrNetfpInterface->ipv6[index].interfaceIP.subnetMask.addr.ipv6, &strSubnetMask[0]);

                /* Display the IPv6 Address*/
                Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,
                              "       inet6 addr: %s Mask %s Scope: %s\n", strIPv6Address, strSubnetMask,
                              (Netfp_isIPv6LinkLocal(ptrNetfpInterface->ipv6[index].interfaceIP.ipAddress.addr.ipv6) == 1) ?
                              "Link" : "Global");
            }
        }

        /* Display the VLAN specific information (if configured) */
        if (ptrNetfpInterface->cfg.type == Netfp_InterfaceType_VLAN)
        {
            Netfp_logMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "       vlan id: %d\n", ptrNetfpInterface->cfg.vlanId);
            for (index = 0; index < NETFP_MAX_SOCK_PRIORITY; index++)
                Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "       Socket Priority %d --> VLAN p bits %d\n",
                              index, ptrNetfpInterface->cfg.vlanMap.socketToVLANPriority[index]);
        }

        /* Display the L3 QOS configuration (if configured) */
        if (ptrNetfpInterface->l3QosCfg.isEnable == 1)
        {
            /* Display the configuration: */
            Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "L3 QOS Configuration Flow Id: %d\n",
                          ptrNetfpInterface->l3QosCfg.flowId);

            /* Display the Mapping */
            for (index = 0; index < NETFP_MAX_SOCK_PRIORITY; index++)
                Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "DSCP %d --> Queue %u\n",
                              index, ptrNetfpInterface->l3QosCfg.qid[index]);
        }

        /* Increment the counter */
        count = count + 1;

        /* Get the next interface */
        ptrNetfpInterface = (Netfp_Interface*)Netfp_listGetNext ((Netfp_ListNode*)ptrNetfpInterface);
    }

    /* Display the banner. */
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,  "Physical Interface List: \n");
    ptrPhyInterface = (Netfp_PhyInterface*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrPhyInterfaceList);
    while (ptrPhyInterface != NULL)
    {
        /* Display the physical interface properties */
        Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "Phy-Interface 0x%x Name: %s Switch Port Number: %d\n",
                      ptrPhyInterface, ptrPhyInterface->ifName, ptrPhyInterface->switchPortNum);

        /* Display the physical interface marking map: */
        for (index = 0; index < NETFP_MAX_SOCK_PRIORITY; index++)
            Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "  Inner DSCP %d -> Outer DSCP %d\n",
                          index, ptrPhyInterface->innerToOuterDSCPMap[index]);

        /* Get the next physical interface. */
        ptrPhyInterface = (Netfp_PhyInterface*)Netfp_listGetNext ((Netfp_ListNode*)ptrPhyInterface);
    }
    return count;
}

/**
 *  @b Description
 *  @n
 *      The function is used to register the NETFP interface services
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
int32_t Netfp_registerInterfaceServices (Josh_NodeHandle nodeHandle)
{
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Netfp_createInterface);
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Netfp_deleteInterface);
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Netfp_findInterface);
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Netfp_getIfOpt);
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Netfp_setIfOpt);
    return 0;
}
