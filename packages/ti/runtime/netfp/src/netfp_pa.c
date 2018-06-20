/**
 *   @file  netfp_pa.c
 *
 *   @brief
 *      The file implements the interface to the PA.
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
#include <errno.h>
#include <assert.h>

#ifdef __ARMv7
#include <unistd.h>
#endif

/* PDK & CSL Include Files */
#include <ti/drv/rm/rm.h>

/* SYSLIB Include Files */
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/netfp/netfp.h>
#include <ti/runtime/netfp/include/netfp_internal.h>

/* For Debugging only. */
#ifdef __ARMv7
#include <ti/apps/netfp_config/include/NetFP_System_printf.h> //fzm
#else
#include <xdc/runtime/System.h>
#endif

/***********************************************************************************************
 * MCSDK Patch: The MCSDK patch is applied because currently the PA does not specify a mapping
 * from the Virtual Link Handle to an identifier.
 ***********************************************************************************************/
#define MCSDK_PATCH
#ifdef MCSDK_PATCH
extern paReturn_t Pa_getVirtualLinkId(Pa_Handle iHandle, paLnkHandle_t vlinkHdl, int8_t* lnkId);
#endif

/**********************************************************************
 ************************* Local Definitions **************************
 **********************************************************************/

/**
 * @brief   Number of exception routes used
 */
#define NETFP_MAX_GTPU_EROUTES              12

/**
 * @brief   Internal definition: This is the number of receive channels in the
 * NETCP subsystem. This should be picked up from the CPPI Device file.
 */
#define NETCP_MAX_RX_CHANNELS               24

/**********************************************************************
 ************************* NETFP-PA Functions *************************
 **********************************************************************/

#if defined(__ARM_ARCH_7A__)
// ARM
#  define NETFP_CFG_BOUNCE_Q(_q) PA_BOUNCE_QUEUE_DEFAULT(_q);
#else
// DSP
#  define NETFP_CFG_BOUNCE_Q(_q) PA_BOUNCE_QUEUE_NONE(_q);
#endif

static inline void Netfp_fillPaCmdReply
(
    paCmdReply_t          * const cmdReplyInfo,
    Netfp_ServerMCB const * const ptrNetfpServer
)
{
    cmdReplyInfo->dest    = pa_DEST_HOST;
    cmdReplyInfo->flowId  = Cppi_getFlowId(ptrNetfpServer->paRxFlowHandle);
    cmdReplyInfo->queue   = NETFP_CFG_BOUNCE_Q(Qmss_getQIDFromHandle(ptrNetfpServer->paCfgRespQueue));
    cmdReplyInfo->replyId = Netfp_getUniqueId();
}






/**
 *  @b Description
 *  @n
 *      The function waits for the result to come back after a command was
 *      passed to the PA Subsystem. If no response is received after a specific
 *      time the function returns error.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  id
 *      Id of the request which we are waiting to complete.
 *  @param[in]  ptrPAStats
 *      Pointer to the PA stats if this was a response for a PA Stats request
 *      else NULL.
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0
 */
static int32_t Netfp_processResponse
(
    Netfp_ServerMCB*    ptrNetfpServer,
    uint32_t            id,
    paSysStats_t*       ptrPAStats
)
{
    uint32_t        j;
    Ti_Pkt*         ptrResponsePacket;
    int32_t         cmdDest;
    int32_t         handleType;
    paEntryHandle_t retHandle;
    paReturn_t      retVal;
    uint32_t        responsePktId;
    uint8_t*        ptrDataBuffer;
    uint32_t        dataBufferLen;
    paSysStats_t*   stats;

    /* Poll on the PA response queue to see if response from PA has come */
    for (j = 0; j < 100; j++)
    {
        /* Wait for some time. */
        Netfp_cycleDelay(100);

        /* Get the response packet from the PA Response queue. */
        ptrResponsePacket = (Ti_Pkt*)QMSS_DESC_PTR(Qmss_queuePop (ptrNetfpServer->paCfgRespQueue));
        if (ptrResponsePacket != NULL)
        {
            /******************************************************************************
             * NOTE: There is an ownership change here from the CPDMA to the DSP. We need
             * to ensure that the packet contents here are invalidated
             ******************************************************************************/
            Pktlib_getOwnership(ptrNetfpServer->cfg.pktlibInstHandle, ptrResponsePacket);

            /* Response packet was detected. Get the software information of this. */
            responsePktId = Cppi_getSoftwareInfo0(Cppi_DescType_HOST, (Cppi_Desc*)ptrResponsePacket);

            /* Is this our request? */
            if (responsePktId != id)
            {
                /* No. This is not our response packet. We need to place this packet back at
                 * the end of the response queue. */
                Qmss_queuePush(ptrNetfpServer->paCfgRespQueue, ptrResponsePacket, 0, 16, Qmss_Location_TAIL);
                continue;
            }

            /* Get the data buffer from the allocated packet. */
            Pktlib_getDataBuffer(ptrResponsePacket, &ptrDataBuffer, &dataBufferLen);

            /* Invalidate data buffer. */
            ptrNetfpServer->cfg.beginMemAccess(ptrDataBuffer, dataBufferLen);

            /* This was our request. Is this a response for STATISTICS or not? */
            if (ptrPAStats != NULL)
            {
                /* Statistics Request: */
                stats = (paSysStats_t *)Pa_formatStatsReply (ptrNetfpServer->paHandle, (paCmd_t)ptrDataBuffer);
                if (stats == NULL)
                {
                    /* Cleanup the response packet. */
                    Pktlib_freePacket(ptrNetfpServer->cfg.pktlibInstHandle, ptrResponsePacket);
                    return NETFP_EINTERNAL;
                }
                /* Copy the PA Statistics */
                memcpy((void *)ptrPAStats, (void *)stats, sizeof(paSysStats_t));
            }
            else
            {
                /* Configuration Request: */
                retVal = Pa_forwardResult (ptrNetfpServer->paHandle, ptrDataBuffer, &retHandle, &handleType, &cmdDest);
                if (retVal != pa_OK)
                {
                    /* Cleanup the response packet. */
                    Pktlib_freePacket(ptrNetfpServer->cfg.pktlibInstHandle, ptrResponsePacket);
                    return retVal;
                }
            }

            /* Command was successfully forwarded to the PA so cleanup the response packet. */
            Pktlib_freePacket(ptrNetfpServer->cfg.pktlibInstHandle, ptrResponsePacket);
            return 0;
        }
    }
    /* Control comes here on timeout. This implies that we never got a response from the PA. */
    return NETFP_ETIMEOUT;
}

/**
 *  @b Description
 *  @n
 *      The function waits for the result to come back after a user stats request command was
 *      passed to the PA Subsystem. If no response is received after a specific
 *      time the function returns error.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  id
 *      Id of the request which we are waiting to complete.
 *  @param[in]  ptrUsrStats
 *      Pointer to the PA user stats.
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0
 */
static int32_t Netfp_processUserStatsResponse
(
    Netfp_ServerMCB*    ptrNetfpServer,
    uint32_t            id,
    paUsrStats_t*       ptrUsrStats
)
{
    uint32_t        j;
    Ti_Pkt*         ptrResponsePacket;
    paReturn_t      retVal;
    uint32_t        responsePktId;
    uint8_t*        ptrDataBuffer;
    uint32_t        dataBufferLen;

    /* Poll on the PA response queue to see if response from PA has come */
    for (j = 0; j < 100; j++)
    {
        /* Wait for some time. */
        Netfp_cycleDelay(100);

        /* Get the response packet from the PA Response queue. */
        ptrResponsePacket = (Ti_Pkt*)QMSS_DESC_PTR(Qmss_queuePop (ptrNetfpServer->paCfgRespQueue));
        if (ptrResponsePacket != NULL)
        {
            /******************************************************************************
             * NOTE: There is an ownership change here from the CPDMA to the DSP. We need
             * to ensure that the packet contents here are invalidated
             ******************************************************************************/
            Pktlib_getOwnership(ptrNetfpServer->cfg.pktlibInstHandle, ptrResponsePacket);

            /* Response packet was detected. Get the software information of this. */
            responsePktId = Cppi_getSoftwareInfo0(Cppi_DescType_HOST, (Cppi_Desc*)ptrResponsePacket);

            /* Is this our request? */
            if (responsePktId != id)
            {
                /* No. This is not our response packet. We need to place this packet back at
                 * the end of the response queue. */
                Qmss_queuePush(ptrNetfpServer->paCfgRespQueue, ptrResponsePacket, 0, 16, Qmss_Location_TAIL);
                continue;
            }

            /* Get the data buffer from the allocated packet. */
            Pktlib_getDataBuffer(ptrResponsePacket, &ptrDataBuffer, &dataBufferLen);

            /* Invalidate data buffer. */
            ptrNetfpServer->cfg.beginMemAccess(ptrDataBuffer, dataBufferLen);

            /* This was our request. Is this a response for STATISTICS or not? */
            if (ptrUsrStats != NULL)
            {
                /* Statistics Request: */
                retVal = Pa_formatUsrStatsReply (ptrNetfpServer->paHandle, (paCmd_t)ptrDataBuffer, ptrUsrStats);
                if (retVal != pa_OK)
                {
                    /* Cleanup the response packet. */
                    Pktlib_freePacket(ptrNetfpServer->cfg.pktlibInstHandle, ptrResponsePacket);
                    return retVal;
                }
            }

            /* Command was successfully forwarded to the PA so cleanup the response packet. */
            Pktlib_freePacket(ptrNetfpServer->cfg.pktlibInstHandle, ptrResponsePacket);
            return 0;
        }
    }
    /* Control comes here on timeout. This implies that we never got a response from the PA. */
    return NETFP_ETIMEOUT;
}

/**
 *  @b Description
 *  @n
 *      Default Fail Information configuration. This specifies the default
 *      action to be taken when there is no match with the NETFP rules.
 *      The default action is to use interface based routeing.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[out]  ptrFailInfo
 *      Pointer to the fail information populated by this API.
 *
 *  @retval
 *      Not Applicable
 */
int32_t Netfp_setupDefaultFailInfo
(
    Netfp_ServerMCB*    ptrNetfpServer,
    paRouteInfo2_t*     ptrFailInfo
)
{
    /* Santity check of interface based routing configuration */
    if( (ptrNetfpServer->interfaceBaseQueue == 0) ||
        (ptrNetfpServer->interfaceBaseFlowId  == 0) )
    {
        return -1;
    }

    /* Initialize the fail information. */
    memset(ptrFailInfo, 0, sizeof(paRouteInfo2_t));

    /* Setup the fail information to pass all packets to the ARM:*/
    ptrFailInfo->dest           = pa_DEST_HOST;
    ptrFailInfo->flowId         = ptrNetfpServer->interfaceBaseFlowId;
    ptrFailInfo->queue          = NETFP_CFG_BOUNCE_Q(ptrNetfpServer->interfaceBaseQueue);
    ptrFailInfo->mRouteIndex    = pa_NO_MULTI_ROUTE;
    ptrFailInfo->swInfo0        = 0;
    ptrFailInfo->swInfo1        = 0;
    ptrFailInfo->priorityType   = pa_ROUTE_INTF_W_FLOW;
    ptrFailInfo->validBitMap    = pa_ROUTE_INFO_VALID_PRIORITY_TYPE;

    /* Fail information has been setup. */
    return 0 ;
}

/**
 *  @b Description
 *  @n
 *      The function is used to add the Ethernet Information to the NETCP subsystem
 *      to enable cascading.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  ethInfo
 *      Ethernet info for programming PA
 *  @param[in]  routeInfo
 *      Specifies action to be taken on all packets matching the LUT rule in ethInfo
 *  @param[in]  failInfo
 *      Specifies action to be taken on all packets failing LUT match rule in ethInfo
 *  @param[out]  ethHandle
 *      Handle to the ethernet entry populated into the NETCP subsystem by this API
 *  @param[in]  lutIndex
 *      LUT1-0 index
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
int32_t Netfp_paAddEth
(
    Netfp_ServerMCB*    ptrNetfpServer,
    paEthInfo2_t*       ethInfo,
    paRouteInfo2_t*     routeInfo,
    paRouteInfo2_t*     failInfo,
    Netfp_L2Handle*     ethHandle,
    uint32_t            lutIndex,
    int32_t*            errCode
)
{
    paCmdReply_t        cmdReplyInfo;
    paReturn_t          retVal;
    Ti_Pkt*             ptrCmdPacket;
    uint32_t            dataBufferLen;
    uint8_t*            ptrDataBuffer;
    int32_t             cmdDest;
    uint16_t            cmdSize;
    uint32_t            psCmd = PASAHO_PACFG_CMD;
    paParamDesc         ethParams;

    /* Sanity Check: Make sure there are valid arguments. */
    if ((ethInfo == NULL) || (routeInfo == NULL) )
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Initialize the command reply:
     *  - We want command replies to come to the Host in the PA Configuration
     *    Response queue and also use the Flow which we had configured. */
    memset ((void *)&cmdReplyInfo, 0, sizeof(paCmdReply_t));
    Netfp_fillPaCmdReply(&cmdReplyInfo, ptrNetfpServer);

    /* Setup the ethernet routing params */
    memset ((void *)&ethParams, 0, sizeof(paParamDesc));

    ethParams.routeInfo =   routeInfo;
    ethParams.nextRtFail=   failInfo;

    /* Setup LUT instance */
    ethParams.lutInst   =   pa_LUT1_INST_0;
    ethParams.validBitMap=  pa_PARAM_VALID_LUTINST;

    /* Setup LUT index */
    ethParams.index         = lutIndex;
    ethParams.validBitMap  |=  pa_PARAM_VALID_INDEX;

    /* Allocate a packet for the configuration command. */
    ptrCmdPacket = Pktlib_allocPacket(ptrNetfpServer->cfg.pktlibInstHandle, ptrNetfpServer->cfg.cmdHeapHandle,
                                      pa_ADD_MAC_MIN_CMD_BUF_SIZE_BYTES);
    if (ptrCmdPacket == NULL)
        return -1;

    /* Get the data buffer from the allocated packet. */
    Pktlib_getDataBuffer(ptrCmdPacket, &ptrDataBuffer, &dataBufferLen);

    /* Setup the command size. */
    cmdSize = (uint16_t)dataBufferLen;

    /* Add MAC info in the PA Susbystem. */
    retVal = Pa_addMac2 (ptrNetfpServer->paHandle,
                         ethInfo,
                         &ethParams,
                         ethHandle,
                         (paCmd_t)ptrDataBuffer,
                         &cmdSize,
                         &cmdReplyInfo,
                         &cmdDest);
    if (retVal != pa_OK)
    {
        System_printf("Pa_addMac2 failed with error: %d\n", retVal);
        *errCode = retVal;
        /* Error: Configuration Failed. */
        Pktlib_freePacket(ptrNetfpServer->cfg.pktlibInstHandle, ptrCmdPacket);
        return retVal;
    }

    /* Set the packet & data buffer length. */
    Pktlib_setPacketLen(ptrCmdPacket, cmdSize);
    Pktlib_setDataBufferLen(ptrCmdPacket, cmdSize);

    /* Set the software information fields. This is required only so that the
     * PS information goes to the correct location. */
    Cppi_setSoftwareInfo0(Cppi_DescType_HOST, (Cppi_Desc *)ptrCmdPacket, 0x12345678);

    /* Mark the packet as a configuration packet */
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)ptrCmdPacket, (uint8_t *)&psCmd, 4);

    /* Ensure that the command packet is also written back */
    ptrNetfpServer->cfg.endMemAccess (ptrDataBuffer, dataBufferLen);

    /******************************************************************************
     * NOTE: There is an ownership change here from the DSP to the CPDMA. We need
     * to ensure that the packet contents here are written back.
     ******************************************************************************/
    Pktlib_releaseOwnership(ptrNetfpServer->cfg.pktlibInstHandle, ptrCmdPacket);

    /* Send the command to the PA. */
    Qmss_queuePushDescSize (ptrNetfpServer->netcpTxQueue[cmdDest - pa_CMD_TX_DEST_0 + NSS_PA_QUEUE_INPUT_INDEX], (Cppi_Desc *)ptrCmdPacket, 64);

    /* Wait for the result and process it accordingly by forwarding to the PA subsystem. */
    retVal = Netfp_processResponse(ptrNetfpServer, cmdReplyInfo.replyId, NULL);
    if (retVal < 0)
        return retVal;

    /* MAC address has been successfully added. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to delete the Ethernet Information from the
 *      NETCP subsystem
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  ethHandle
 *      Handle to the L2 entry to be deleted.
 *
 *  \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   0.
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_paDelEth
(
    Netfp_ServerMCB*    ptrNetfpServer,
    Netfp_L2Handle      ethHandle
)
{
    uint16_t        refCount;
    paCmdReply_t    cmdReplyInfo;
    paReturn_t      retVal;
    Ti_Pkt*         ptrCmdPacket;
    uint32_t        dataBufferLen;
    uint8_t*        ptrDataBuffer;
    int32_t         cmdDest;
    uint16_t        cmdSize;
    uint32_t        psCmd = PASAHO_PACFG_CMD;

    /* Sanity Check: Ensure that this is a valid handle */
    if (ethHandle == NULL)
        return -1;

    /* Get the reference counter */
    retVal = Pa_getHandleRefCount (ptrNetfpServer->paHandle, ethHandle, &refCount);
    if (retVal != pa_OK)
        return retVal;

    /* Sanity check on refCount. */
    if (refCount != 0)
        return -1;

    /* Initialize the command reply:
     *  - We want command replies to come to the Host in the PA Configuration
     *    Response queue and also use the Flow which we had configured. */
    memset ((void *)&cmdReplyInfo, 0, sizeof(paCmdReply_t));
    Netfp_fillPaCmdReply(&cmdReplyInfo, ptrNetfpServer);

    /* Allocate a packet for the configuration command. */
    ptrCmdPacket = Pktlib_allocPacket(ptrNetfpServer->cfg.pktlibInstHandle, ptrNetfpServer->cfg.cmdHeapHandle,
                                      pa_DEL_L4_HANDLE_MIN_CMD_BUF_SIZE_BYTES);
    if (ptrCmdPacket == NULL)
        return -1;

    /* Get the data buffer from the allocated packet. */
    Pktlib_getDataBuffer(ptrCmdPacket, &ptrDataBuffer, &dataBufferLen);

    /* Setup the command size. */
    cmdSize = (uint16_t)dataBufferLen;

    /* Delete the GTPU Fast Path Handle from the NETCP Subsystem. */
    retVal = Pa_delHandle(ptrNetfpServer->paHandle,
                          (paHandleL2L3_t *)&ethHandle,
                          ptrDataBuffer,
                          &cmdSize,
                          &cmdReplyInfo,
                          &cmdDest);
    if (retVal != pa_OK)
    {
        /* Error: Configuration Failed. */
        Pktlib_freePacket(ptrNetfpServer->cfg.pktlibInstHandle, ptrCmdPacket);
        return retVal;
    }

    /* Set the packet & data buffer length. */
    Pktlib_setPacketLen(ptrCmdPacket, cmdSize);
    Pktlib_setDataBufferLen(ptrCmdPacket, cmdSize);

    /* Set the software information fields. This is required only so that the
     * PS information goes to the correct location. */
    Cppi_setSoftwareInfo0(Cppi_DescType_HOST, (Cppi_Desc *)ptrCmdPacket, 0x12345678);

    /* Mark the packet as a configuration packet */
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)ptrCmdPacket, (uint8_t *)&psCmd, 4);

    /* Ensure that the command packet is also written back */
    ptrNetfpServer->cfg.endMemAccess (ptrDataBuffer, dataBufferLen);

    /******************************************************************************
     * NOTE: There is an ownership change here from the DSP to the CPDMA. We need
     * to ensure that the packet contents here are written back.
     ******************************************************************************/
    Pktlib_releaseOwnership(ptrNetfpServer->cfg.pktlibInstHandle, ptrCmdPacket);

    /* Send the command to the PA. */
    Qmss_queuePushDescSize (ptrNetfpServer->netcpTxQueue[cmdDest - pa_CMD_TX_DEST_0 + NSS_PA_QUEUE_INPUT_INDEX], (Cppi_Desc *)ptrCmdPacket, 64);

    /* Wait for the result and process it accordingly by forwarding to the PA subsystem. */
    retVal = Netfp_processResponse(ptrNetfpServer, cmdReplyInfo.replyId, NULL);
    if (retVal < 0)
        return retVal;

    /* Ethernet Handle has been deleted. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to find LUT indices for LUT1-1 and LUT1-2
 *      which matches given IP configuration
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  lutInst
 *      Instance of LUT entry to find match
 *  @param[in]  ptrIPLutCfg
 *      IP Lut entry configuration
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      NULL -      No match
 *  @retval
 *      Success -   NetFP LUT Info handle
  */
static Netfp_LUTInfo* Netfp_findIPLutEntry
(
    Netfp_ServerMCB*        ptrNetfpServer,
    uint32_t                lutInst,
    Netfp_IPLutCfg*         ptrIPLutCfg,
    int32_t*                errCode
)
{
    Netfp_LUTInfo*          ptrLutInfo;

    /* Get the head of the list of LUT entries based on the LUT instance number */
    if(lutInst == pa_LUT1_INST_1)
    {
        /* LUT Info: Cycle through all the LUT entries to determine if there is a match or not? */
        ptrLutInfo  = (Netfp_LUTInfo*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrOuterIPLutInfoList);
    }
    else if(lutInst == pa_LUT1_INST_2)
    {
        /* LUT Info: Cycle through all the LUT entries to determine if there is a match or not? */
        ptrLutInfo  = (Netfp_LUTInfo*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrInnerIPLutInfoList);
    }
    else
    {
        *errCode = NETFP_EINVAL;
        return NULL;
    }

    Netfp_LUTType lutTypeToMatch = Netfp_LUTType_DEFAULT;
    if(ptrIPLutCfg->fastpathMode == Netfp_FastpathMode_SW_EXTENSION_CHANNEL)
        lutTypeToMatch = Netfp_LUTType_SW_CHANNEL;

    /* Cycle through Lut entry list to find IP Cfg match */
    while (ptrLutInfo != NULL)
    {
        /* Verify the IP configuations. */
        if ((Netfp_matchIP(&ptrLutInfo->ptrIPCfg.srcIP, &ptrIPLutCfg->ipCfg.srcIP) == 1) &&
            (Netfp_matchIP(&ptrLutInfo->ptrIPCfg.dstIP, &ptrIPLutCfg->ipCfg.dstIP) == 1) &&
            (ptrLutInfo->ptrIPCfg.protocol == ptrIPLutCfg->ipCfg.protocol)               &&
            (ptrLutInfo->ptrIPCfg.spi      == ptrIPLutCfg->ipCfg.spi )                   &&
            (ptrLutInfo->prevLinkHandle    == ptrIPLutCfg->prevLinkHandle)               &&
            (ptrLutInfo->nextLinkHandle    == ptrIPLutCfg->nextLinkHandle)               &&
            (ptrLutInfo->lutType           == lutTypeToMatch))
            return (Netfp_LUTInfo*)ptrLutInfo;

        /* Goto the next element in the list. */
        ptrLutInfo = (Netfp_LUTInfo*)Netfp_listGetNext ((Netfp_ListNode*)ptrLutInfo);
    }
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the LUT instance where the IP LUT configuration
 *      is to be added. This function is applicable only for the following cases:-
 *         (a) Traditional Fast Paths
 *         (b) Multicast Non-Shared Fast Paths
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  ptrIPLutCfg
 *      IP LUT entry configuration
 *  @param[in]  isIPSECChannel
 *      Set to 1 to indicate that an IPSEC channel was being added
 *  @param[out] lutInstance
 *      Allocated LUT Instance
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   NETFP Error code
 */
static int32_t Netfp_getLUTInstance
(
    Netfp_ServerMCB*        ptrNetfpServer,
    Netfp_IPLutCfg*         ptrIPLutCfg,
    uint8_t                 isIPSECChannel,
    uint32_t*               lutInstance
)
{
    /* Are we adding an IPSEC Channel? */
    if (isIPSECChannel == 1)
    {
        /* IPSEC Channels are always added in LUT1-1. */
        *lutInstance = pa_LUT1_INST_1;
        return 0;
    }

    /* Do we have a link to the previous IPSEC Channel? */
    if (ptrIPLutCfg->prevLinkHandle == NULL)
    {
        /* NON-Secure connection:  */
        switch (ptrIPLutCfg->ipCfg.spidMode)
        {
            case Netfp_SPIDMode_INVALID:
            {
                /* NON Secure Fast Path: No security policy identifier specified. Program the fast path in LUT1-1 */
                *lutInstance = pa_LUT1_INST_1;
                return 0;
            }
            case Netfp_SPIDMode_ANY_SECURE:
            {
                /* There is *NO LINK* between the Outer and Inner IP address. If a fast path is being programmed in
                 * this mode; we simply assume that the LUT1-1 has already been programmed; so we program the LUT1-2
                 * here. */
                *lutInstance = pa_LUT1_INST_2;
                return 0;
            }
            case Netfp_SPIDMode_SPECIFIC:
            {
                /* Security Policy identifier was specified but this simply maps to *NO* IPSEC Channel; so no OUTER
                 * IP address. Hence we simply program the LUT1-1. */
                *lutInstance = pa_LUT1_INST_1;
                return 0;
            }
            default:
            {
                /* FATAL Error: This should NOT happen. This should be caught much earlier. */
                return NETFP_EINTERNAL;
            }
        }
    }

    /* Secure connection: */
    switch (ptrIPLutCfg->ipCfg.spidMode)
    {
        case Netfp_SPIDMode_INVALID:
        {
            /* Error: This cannot happen; because we cannot get a security channel with an INVALID SPID */
            return NETFP_EINTERNAL;
        }
        case Netfp_SPIDMode_ANY_SECURE:
        {
            /* Error: This cannot happen; because we cannot get a security channel with this mode. */
            return NETFP_EINTERNAL;
        }
        case Netfp_SPIDMode_SPECIFIC:
        {
            /* Secure Fast Path: There is a link between the Outer and Inner IP address. After the Outer IP is
             * matched it is passed to the SA for decryption and then back to the PA LUT1-2 for the match. */
             *lutInstance = pa_LUT1_INST_2;
             return 0;
        }
        default:
        {
            /* FATAL Error: This should NOT happen. This should be caught much earlier. */
            return NETFP_EINTERNAL;
        }
    }
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the LUT instance for fast paths which use the MULTICAST
 *      Sharing mode
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  ptrIPLutCfg
 *      IP LUT entry configuration
 *  @param[out] lutInstance
 *      Allocated LUT Instance
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   NETFP Error code
 */
static int32_t Netfp_getMulticastSharingLUTInstance
(
    Netfp_ServerMCB*    ptrNetfpServer,
    Netfp_IPLutCfg*     ptrIPLutCfg,
    uint32_t*           lutInstance
)
{
    /* In the case of Multicast sharing we will always have a link to the previous handle which
     * will be the Virtual MAC Link */
    switch (ptrIPLutCfg->ipCfg.spidMode)
    {
        case Netfp_SPIDMode_INVALID:
        {
            /* Non-Secure Multicast Sharing Fast Path: */
            *lutInstance = pa_LUT1_INST_1;
            return 0;
        }
        case Netfp_SPIDMode_ANY_SECURE:
        {
            /* Multicast Sharing Fast Path over a wildcarded secure fast path: This is not implemented
             * or supported. */
            return NETFP_ENOTIMPL;
        }
        case Netfp_SPIDMode_SPECIFIC:
        {
            /* Multicast Sharing Fast Path over a specified secure fast path: This is not implemented
             * or supported. */
             return NETFP_ENOTIMPL;
        }
        default:
        {
            /* FATAL Error: This should NOT happen. This should be caught much earlier. */
            return NETFP_EINVAL;
        }
    }
}

//<fzm>
static int32_t Netfp_getSwExtensionChannelLUTInstance
(
    Netfp_ServerMCB*    ptrNetfpServer,
    Netfp_IPLutCfg*     ptrIPLutCfg,
    uint32_t*           lutInstance
)
{
    switch (ptrIPLutCfg->ipCfg.spidMode)
    {
        case Netfp_SPIDMode_INVALID:
            return NETFP_ENOTIMPL;

        case Netfp_SPIDMode_ANY_SECURE:
            *lutInstance = pa_LUT1_INST_1;
            return 0;

        case Netfp_SPIDMode_SPECIFIC:
             return NETFP_ENOTIMPL;

        default:
            return NETFP_EINVAL;
    }
}

//</fzm>

/**
 *  @b Description
 *  @n
 *      The function is used to allocate LUT indexes for the LUT1-1 and LUT1-2
 *      which is used to add the IP rules.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  ptrIPLutCfg
 *      IP LUT entry configuration
 *  @param[in]  isIPSECChannel
 *      Set to 1 to indicate that an IPSEC channel was being added
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   Pointer to the LUT Information
 *  @retval
 *      Error   -   NULL
 */
Netfp_LUTInfo* Netfp_allocIPLutEntry
(
    Netfp_ServerMCB*        ptrNetfpServer,
    Netfp_IPLutCfg*         ptrIPLutCfg,
    uint8_t                 isIPSECChannel,
    int32_t*                errCode
)
{
    uint32_t        wildcardingFlag;
    Netfp_LUTInfo*  ptrNetfpLutInfo = NULL;
    uint32_t        lut1Inst;
    Netfp_IPCfg*    ptrIPCfg;

    /* Sanity check */
    if (ptrIPLutCfg == NULL)
    {
        *errCode = NETFP_EINVAL;
        return NULL;
    }

    /* Get IP configuration */
    ptrIPCfg = &ptrIPLutCfg->ipCfg;

    /* Find out if the resource is for wildcarding */
    if (Netfp_checkIPForWildCarding(ptrIPCfg->srcIP) == 1)
        wildcardingFlag = 1;
    else
        wildcardingFlag = 0;

    /* Determine which LUT Instance is to be used to configure the IP configuration */
    switch (ptrIPLutCfg->fastpathMode)
    {
        case Netfp_FastpathMode_NORMAL:
        {
            /* Standard Unicast or Wildcarded Fast Paths: */
            *errCode = Netfp_getLUTInstance(ptrNetfpServer, ptrIPLutCfg, isIPSECChannel, &lut1Inst);
            if (*errCode < 0)
                return NULL;
            break;
        }
        case Netfp_FastpathMode_INFOONLY:
        {
            /* Error: For info only fastpath mode, LUT programming is not needed */
            *errCode = NETFP_EINVAL;
            return NULL;
        }
        case Netfp_FastpathMode_MULTICASTSHARED:
        {
            /* Shared Multicast Fast Path: */
            *errCode = Netfp_getMulticastSharingLUTInstance(ptrNetfpServer, ptrIPLutCfg, &lut1Inst);
            if (*errCode < 0)
                return NULL;
            break;
        }
        case Netfp_FastpathMode_SW_EXTENSION_CHANNEL:
        {
            *errCode = Netfp_getSwExtensionChannelLUTInstance(ptrNetfpServer, ptrIPLutCfg, &lut1Inst);
            if (*errCode < 0)
                return NULL;
            break;
        }
        default:
        {
            /* Error: Invalid configuration: This should have been caught earlier */
            *errCode = NETFP_EINVAL;
            return NULL;
        }
    }

    /* Find if the LUT entry alreay exist */
    ptrNetfpLutInfo = Netfp_findIPLutEntry(ptrNetfpServer, lut1Inst, ptrIPLutCfg, errCode);
    if(ptrNetfpLutInfo != NULL)
    {
        /* Update reference count */
        ptrNetfpLutInfo->refCount++;
        return ptrNetfpLutInfo;
    }

    /* LUT entry does not exist, prepare to allocate */
    ptrNetfpLutInfo = ptrNetfpServer->cfg.malloc(sizeof(Netfp_LUTInfo), 0);
    if(ptrNetfpLutInfo == NULL)
    {
        *errCode = NETFP_ENOMEM;
        return NULL;
    }

    /* Initialize Netfp LUT info */
    memset(ptrNetfpLutInfo, 0, sizeof(Netfp_LUTInfo));

    const char* lutEntry = NULL;
    const char* lutType = NULL;

    if(lut1Inst == pa_LUT1_INST_1)
        lutEntry = "lut11";
    else
        lutEntry = "lut12";

    if(ptrIPLutCfg->fastpathMode == Netfp_FastpathMode_SW_EXTENSION_CHANNEL)
    {
        lutType = "extension";
        ptrNetfpLutInfo->lutType = Netfp_LUTType_SW_CHANNEL;
    }
    else if(wildcardingFlag)
        lutType = "wildcarding";
    else
        lutType = "fastpath";

    /* Get the LUT index resource name */
    snprintf(&ptrNetfpLutInfo->rmName[0], NETFP_MAX_CHAR, "%s_%s_index", lutEntry, lutType);

    /* Allocate the custom resource: */
    if (Resmgr_allocCustomResource(ptrNetfpServer->cfg.sysRMHandle, &ptrNetfpLutInfo->rmName[0], 1,
                                   (uint32_t*)&ptrNetfpLutInfo->lutInfo.lut1Index, errCode) < 0)
    {
        /* Free allocated memory */
        ptrNetfpServer->cfg.free(ptrNetfpLutInfo, sizeof(Netfp_LUTInfo));

        /* FATAL Error: Unable to allocate the custom resource. The DTS file is incorrect or we have
         * exceeded the number of LUT entries which can be added. */
        return NULL;
    }

    /* Does the server support the configuration of user counters? */
    if (ptrNetfpServer->cfg.enableIPLutEntryCount)
    {
        Netfp_UserStatsCfg      userStatCfg;

        /* YES: Initialize the user stat configuration */
        memset ((void*)&userStatCfg, 0, sizeof(Netfp_UserStatsCfg));

        /* For the LUT1-1 and LUT1-2 rules we are adding only 1 counter for 32 byte packets */
        userStatCfg.userStatsLen  = Netfp_UserStatsLen_32b;
        userStatCfg.userStatsType = pa_USR_STATS_TYPE_PACKET;

        /* Create the user statistics: */
        if (Netfp_createUserStats (ptrNetfpServer, 1, &userStatCfg, &ptrNetfpLutInfo->matchPktCntIndex, errCode) < 0)
        {
            /* Error: Unable to create the user statistics. Cleanup */
            ptrNetfpServer->cfg.free(ptrNetfpLutInfo, sizeof(Netfp_LUTInfo));
            return NULL;
        }
    }

    /* Save LUT Configuration info */
    ptrNetfpLutInfo->lutInfo.lut1Inst = lut1Inst;
    memcpy((void *)&ptrNetfpLutInfo->ptrIPCfg, (void *)ptrIPCfg, sizeof(Netfp_IPCfg));
    ptrNetfpLutInfo->prevLinkHandle   = ptrIPLutCfg->prevLinkHandle;
    ptrNetfpLutInfo->nextLinkHandle   = ptrIPLutCfg->nextLinkHandle;

    /* Update reference count */
    ptrNetfpLutInfo->refCount++;

    /* Add IP entry in NETCP */
    *errCode = Netfp_addIP (ptrNetfpServer, ptrIPCfg, ptrIPLutCfg->prevLinkHandle, ptrIPLutCfg->nextLinkHandle,
                            ptrIPLutCfg->ptrIPSecChannel, &ptrIPLutCfg->dstInfo, ptrIPLutCfg->flags,
                            ptrNetfpLutInfo, &ptrNetfpLutInfo->ipHandle);
    if (*errCode < 0)
    {
        /* Free allocated resources */
        ptrNetfpServer->cfg.free (ptrNetfpLutInfo, sizeof(Netfp_LUTInfo));
        return NULL;
    }

    /* Track the LUT Information: */
    if(lut1Inst == pa_LUT1_INST_1)
        Netfp_listAdd ((Netfp_ListNode**)&ptrNetfpServer->ptrOuterIPLutInfoList, (Netfp_ListNode*)ptrNetfpLutInfo);
    else
        Netfp_listAdd ((Netfp_ListNode**)&ptrNetfpServer->ptrInnerIPLutInfoList, (Netfp_ListNode*)ptrNetfpLutInfo);

    /* Return the LUT Information: */
    return ptrNetfpLutInfo;
}

/**
 *  @b Description
 *  @n
 *      The function is used to free a previously allocated LUT index
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  ptrNetfpLutInfo
 *      Pointer to LUT Information to be freed.
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   Error code
 */
int32_t Netfp_freeIPLutEntry
(
    Netfp_ServerMCB*        ptrNetfpServer,
    Netfp_LUTInfo*          ptrNetfpLutInfo
)
{
    int32_t errCode;

    /* Update reference count */
    ptrNetfpLutInfo->refCount--;

    /* Can we delete the LUT Information? */
    if(ptrNetfpLutInfo->refCount != 0)
        return 0;

    /* YES: Delete LUT entry from server list */
    if (ptrNetfpLutInfo->lutInfo.lut1Inst == pa_LUT1_INST_1)
    {
        /* Remove the LUT entry from the NETFP Server */
        Netfp_listRemoveNode((Netfp_ListNode**)&ptrNetfpServer->ptrOuterIPLutInfoList, (Netfp_ListNode*)ptrNetfpLutInfo);
    }
    else if(ptrNetfpLutInfo->lutInfo.lut1Inst == pa_LUT1_INST_2)
    {
        /* Remove the LUT entry from the NETFP Server */
        Netfp_listRemoveNode((Netfp_ListNode**)&ptrNetfpServer->ptrInnerIPLutInfoList, (Netfp_ListNode*)ptrNetfpLutInfo);
    }
    else
    {
        /* FATAL Error: Internal LUT Information corruption detected */
        return NETFP_EINTERNAL;
    }

    /* Does the server support the configuration of user counters? */
    if (ptrNetfpServer->cfg.enableIPLutEntryCount)
    {
        /* Delete the user statistics: */
        if (Netfp_deleteUserStats(ptrNetfpServer, 1, &ptrNetfpLutInfo->matchPktCntIndex, &errCode) < 0)
            return errCode;
    }

    /* Free the resource allocated for LUT */
    if (Resmgr_freeCustomResource(ptrNetfpServer->cfg.sysRMHandle, ptrNetfpLutInfo->rmName, 1,
                                  ptrNetfpLutInfo->lutInfo.lut1Index, &errCode) < 0)
        return errCode;

    /* Delete the IP from the NETCP */
    errCode = Netfp_delIP (ptrNetfpServer, ptrNetfpLutInfo->ipHandle);
    if (errCode < 0)
        return errCode;

    /* Free allocated Memory */
    ptrNetfpServer->cfg.free(ptrNetfpLutInfo, sizeof(Netfp_LUTInfo));
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is a debug function to display the added LUT entries
 *
 *  @param[in]  serverHandle
 *      Handle to the NETFP Server
 *
 *  \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Number of fast paths which are present
 */
int32_t Netfp_displayLutInfoList(Netfp_ServerHandle serverHandle)
{
    Netfp_LUTInfo*          ptrNetfpLutInfo;
    Netfp_ServerMCB*        ptrNetfpServer;
    int32_t                 count = 0;

    /* Get the server handle. */
    ptrNetfpServer = (Netfp_ServerMCB*)serverHandle;
    if (ptrNetfpServer == NULL)
        return 0;

    /* Get the Outer IP LUT list */
    ptrNetfpLutInfo = (Netfp_LUTInfo*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrOuterIPLutInfoList);

//fzm - use Netfp_dumpMsg to capture this with the USR1 signal
    /* Display the banner. */
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "**********************************************************\n");
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "Outer IP LUT Table\n");

    /* Cycle through all the fast paths and display them on the console */
    while (ptrNetfpLutInfo != NULL)
    {
        /* Display the LUT info */
        if (ptrNetfpLutInfo->ptrIPCfg.srcIP.ver == Netfp_IPVersion_IPV4)
        {
            Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,
                          "0x%x: RMName: %s Src IP %d.%d.%d.%d -> Dst IP %d.%d.%d.%d LUT:%d(%d) RefCnt:%d, ipHandle=%p\n",
                          ptrNetfpLutInfo, &ptrNetfpLutInfo->rmName[0],
                          ptrNetfpLutInfo->ptrIPCfg.srcIP.addr.ipv4.u.a8[0], ptrNetfpLutInfo->ptrIPCfg.srcIP.addr.ipv4.u.a8[1],
                          ptrNetfpLutInfo->ptrIPCfg.srcIP.addr.ipv4.u.a8[2], ptrNetfpLutInfo->ptrIPCfg.srcIP.addr.ipv4.u.a8[3],
                          ptrNetfpLutInfo->ptrIPCfg.dstIP.addr.ipv4.u.a8[0], ptrNetfpLutInfo->ptrIPCfg.dstIP.addr.ipv4.u.a8[1],
                          ptrNetfpLutInfo->ptrIPCfg.dstIP.addr.ipv4.u.a8[2], ptrNetfpLutInfo->ptrIPCfg.dstIP.addr.ipv4.u.a8[3],
                          ptrNetfpLutInfo->lutInfo.lut1Inst, ptrNetfpLutInfo->lutInfo.lut1Index,
                          ptrNetfpLutInfo->refCount, ptrNetfpLutInfo->ipHandle);
        }
        else
        {
            char    srcIP[40];
            char    dstIP[40];

            /* Convert the Src & Dst IP address of the SP into strings using the NETFP IPv6 Utility API */
            Netfp_convertIP6ToStr (ptrNetfpLutInfo->ptrIPCfg.srcIP.addr.ipv6, &srcIP[0]);
            Netfp_convertIP6ToStr (ptrNetfpLutInfo->ptrIPCfg.dstIP.addr.ipv6, &dstIP[0]);

            Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,
                          "0x%x: RMName: %s Src IP %s -> Dst IP %s LUT:%d(%d) RefCnt:%d ipHandle=%p\n",
                          ptrNetfpLutInfo, &ptrNetfpLutInfo->rmName[0],
                          srcIP, dstIP,
                          ptrNetfpLutInfo->lutInfo.lut1Inst, ptrNetfpLutInfo->lutInfo.lut1Index,
                          ptrNetfpLutInfo->refCount, ptrNetfpLutInfo->ipHandle);
        }

        /* Increment the counter */
        count = count + 1;

        /* Get the next fast path */
        ptrNetfpLutInfo = (Netfp_LUTInfo*)Netfp_listGetNext ((Netfp_ListNode*)ptrNetfpLutInfo);
    }
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "**********************************************************\n");

    /* Get the Inner IP LUT list */
    ptrNetfpLutInfo = (Netfp_LUTInfo*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrInnerIPLutInfoList);

    /* Display the banner. */
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "**********************************************************\n");
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "Inner IP LUT Table\n");

    /* Cycle through all the fast paths and display them on the console */
    while (ptrNetfpLutInfo != NULL)
    {
        /* Display the LUT info */
        if (ptrNetfpLutInfo->ptrIPCfg.srcIP.ver == Netfp_IPVersion_IPV4)
        {
            Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,
                          "0x%x: RMName: %s Src IP %d.%d.%d.%d -> Dst IP %d.%d.%d.%d LUT:%d(%d) RefCnt:%d, ipHandle=%p\n",
                          ptrNetfpLutInfo, ptrNetfpLutInfo->rmName,
                          ptrNetfpLutInfo->ptrIPCfg.srcIP.addr.ipv4.u.a8[0], ptrNetfpLutInfo->ptrIPCfg.srcIP.addr.ipv4.u.a8[1],
                          ptrNetfpLutInfo->ptrIPCfg.srcIP.addr.ipv4.u.a8[2], ptrNetfpLutInfo->ptrIPCfg.srcIP.addr.ipv4.u.a8[3],
                          ptrNetfpLutInfo->ptrIPCfg.dstIP.addr.ipv4.u.a8[0], ptrNetfpLutInfo->ptrIPCfg.dstIP.addr.ipv4.u.a8[1],
                          ptrNetfpLutInfo->ptrIPCfg.dstIP.addr.ipv4.u.a8[2], ptrNetfpLutInfo->ptrIPCfg.dstIP.addr.ipv4.u.a8[3],
                          ptrNetfpLutInfo->lutInfo.lut1Inst, ptrNetfpLutInfo->lutInfo.lut1Index,
                          ptrNetfpLutInfo->refCount, ptrNetfpLutInfo->ipHandle);
        }
        else
        {
            char    srcIP[40];
            char    dstIP[40];

            /* Convert the Src & Dst IP address of the SP into strings using the NETFP IPv6 Utility API */
            Netfp_convertIP6ToStr (ptrNetfpLutInfo->ptrIPCfg.srcIP.addr.ipv6, &srcIP[0]);
            Netfp_convertIP6ToStr (ptrNetfpLutInfo->ptrIPCfg.dstIP.addr.ipv6, &dstIP[0]);

            Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,
                          "0x%x: RMName: %s Src IP %s -> Dst IP %s LUT:%d(%d) RefCnt:%d, ipHandle=%p\n",
                          ptrNetfpLutInfo, ptrNetfpLutInfo->rmName,
                          srcIP, dstIP,
                          ptrNetfpLutInfo->lutInfo.lut1Inst, ptrNetfpLutInfo->lutInfo.lut1Index,
                          ptrNetfpLutInfo->refCount, ptrNetfpLutInfo->ipHandle);
        }

        /* Increment the counter */
        count = count + 1;

        /* Get the next fast path */
        ptrNetfpLutInfo = (Netfp_LUTInfo*)Netfp_listGetNext ((Netfp_ListNode*)ptrNetfpLutInfo);
    }
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "**********************************************************\n");

    return count;
}

/**
 *  @b Description
 *  @n
 *      The function is used to configure the LUT1 entries in the NETCP subsystem
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP server
 *  @param[in]  ptrIPCfg
 *      This is the IP information to be configured in the NETCP module.
 *  @param[in]  prevLinkHandle
 *      This is the handle from the previous LUT over which the IP rule will reside.
 *      This could be physical or virtual PA handle
 *  @param[in]  nextLinkHandle
 *      This is the handle to the next LUT to which the IP rule will be linked using virtual links
 *      characteristics. This field should be set to 0xFFFFFFFF if this is not the LAST rule.
 *  @param[in]  ptrIPSecChannel
 *      Pointer to the IPSEC channel information block.
 *  @param[in]  ptrDstInfo
 *      Destination information which specifies the action to be taken by the NETCP
 *      subsystem on a match.
 *  @param[in]  flags
 *      Indicates any special handling that needs to be done during LUT1-1 configuration.
 *      Used for NAT-T, Cascading configurations.
 *  @param[in]  ptrNetfpLutInfo
 *      LUT information including the LUT instance and index to be used to add the
 *      IP rule
 *  @param[out]  ipHandle
 *      This is the IP Handle which is populated by this API.
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0 [Error code populated]
 */
int32_t Netfp_addIP
(
    Netfp_ServerMCB*    ptrNetfpServer,
    Netfp_IPCfg*        ptrIPCfg,
    paLnkHandle_t       prevLinkHandle,
    paLnkHandle_t       nextLinkHandle,
    Netfp_IPSecChannel* ptrIPSecChannel,
    Netfp_DstInfo*      ptrDstInfo,
    uint8_t             flags,
    Netfp_LUTInfo*      ptrNetfpLutInfo,
    Netfp_L3Handle*     ipHandle
)
{
    paIpInfo2_t         ipInfo;
    paRouteInfo2_t      routeInfo;
    paRouteInfo2_t      failInfo;
    paCmdReply_t        cmdReplyInfo;
    paReturn_t          retVal;
    paParamDesc         paramDesc;
    Ti_Pkt*             ptrCmdPacket;
    uint32_t            dataBufferLen;
    uint8_t*            ptrDataBuffer;
    int32_t             cmdDest;
    uint16_t            cmdSize;
    uint32_t            psCmd = PASAHO_PACFG_CMD;
    paCmdInfo_t         paCmd;

    /* Sanity Check: Make sure there are valid arguments. */
    if (ptrIPCfg == NULL || ptrDstInfo == NULL)
        return NETFP_EINVAL;

    /* Initialize the command reply:
     *  - We want command replies to come to the Host in the PA Configuration
     *    Response queue and also use the Flow which we had configured. */
    memset ((void *)&cmdReplyInfo, 0, sizeof(paCmdReply_t));
    Netfp_fillPaCmdReply(&cmdReplyInfo, ptrNetfpServer);

    /* Initialize the IP information. */
    memset ((void *)&ipInfo, 0, sizeof(paIpInfo2_t));

    /* Based on any special handling flags specified, setup the LUT1-1 configuration */
    if (ptrNetfpLutInfo->lutType == Netfp_LUTType_SW_CHANNEL && flags != NETFP_IPCFG_FLAGS_NATT)
    {
        ipInfo.proto    = ptrIPCfg->protocol;
        ipInfo.tos      = ptrIPCfg->tos;

        ipInfo.validBitMap = pa_IP_INFO_VALID_PROTO;
    }
    else if (ptrNetfpLutInfo->lutType == Netfp_LUTType_SW_CHANNEL && flags == NETFP_IPCFG_FLAGS_NATT)
    {
        // do not set anything
    }
    else if (flags == NETFP_IPCFG_FLAGS_NATT)
    {
        /* Only SPI is specified in this case */
        ipInfo.spi          = ptrIPCfg->spi;
        ipInfo.validBitMap  = pa_IP_INFO_VALID_SPI;
    }
    else
    {
        /* No special handling required. Populate all the IP information */
        if (ptrIPCfg->dstIP.ver == Netfp_IPVersion_IPV4)
        {
            /* IPv4 Address. */
            ipInfo.ipType = pa_IPV4;

            /* Populate the IP addresses. */
            memcpy ((void *)&ipInfo.dst, (void*)&ptrIPCfg->dstIP.addr.ipv4.u.a8, sizeof(paIpv4Addr_t));
            memcpy ((void *)&ipInfo.src, (void*)&ptrIPCfg->srcIP.addr.ipv4.u.a8, sizeof(paIpv4Addr_t));
        }
        else
        {
            /* IPv6 Address. */
            ipInfo.ipType = pa_IPV6;

            /* Populate the IP addresses. */
            memcpy ((void *)&ipInfo.dst, (void*)&ptrIPCfg->dstIP.addr.ipv6.u.a8, sizeof(paIpv6Addr_t));
            memcpy ((void *)&ipInfo.src, (void*)&ptrIPCfg->srcIP.addr.ipv6.u.a8, sizeof(paIpv6Addr_t));
        }

        /* Store the protocol & TOS information. */
        ipInfo.proto    = ptrIPCfg->protocol;
        ipInfo.tos      = ptrIPCfg->tos;

        /* Set the valid bitmask for IP Info */
        ipInfo.validBitMap  = pa_IP_INFO_VALID_SRC | pa_IP_INFO_VALID_DST | pa_IP_INFO_VALID_PROTO;

        /* Populate the SPI fields if we are configuring a secure channel. */
        if (ptrIPSecChannel != NULL)
        {
            ipInfo.spi          =   ptrIPCfg->spi;
            ipInfo.validBitMap  |=  pa_IP_INFO_VALID_SPI;
        }
    }

    /* Initialize the valid bitmap */
    paramDesc.validBitMap = 0;
    paramDesc.routeInfo = &routeInfo;
    paramDesc.nextRtFail = &failInfo;

    if (prevLinkHandle)
    {
        paramDesc.prevLink      =   prevLinkHandle;
        paramDesc.validBitMap   |=  pa_PARAM_VALID_PREVLINK;
    }
    if (nextLinkHandle)
    {
        paramDesc.nextLink      =   nextLinkHandle;
        paramDesc.validBitMap   |=  pa_PARAM_VALID_NEXTLINK;
    }

    /* LUT info is always passed in from caller */
    paramDesc.index         = (int)ptrNetfpLutInfo->lutInfo.lut1Index;
    paramDesc.validBitMap   |= pa_PARAM_VALID_INDEX;

    /* Setup LUT instance */
    paramDesc.lutInst       = ptrNetfpLutInfo->lutInfo.lut1Inst;
    paramDesc.validBitMap   |= pa_PARAM_VALID_LUTINST;

    /* Is src IP address a wild carding address? */
    if (Netfp_checkIPForWildCarding(ptrIPCfg->srcIP))
    {
        /* Update valid bit map based on wildcarding setting */
        ipInfo.validBitMap &= (~pa_IP_INFO_VALID_SRC);

        /* Unset the valid bit, if SPI is set to "any". */
        if (ptrIPCfg->spidMode == Netfp_SPIDMode_ANY_SECURE)
            ipInfo.validBitMap  &=  (~pa_IP_INFO_VALID_SPI);

        if (ptrIPCfg->protocol == 0)
            ipInfo.validBitMap &= (~pa_IP_INFO_VALID_PROTO);
    }

    /* Is destination IP address a wild carding address? */
    if (Netfp_checkIPForWildCarding(ptrIPCfg->dstIP))
    {
        /* Update valid bit map based on wildcarding setting */
        ipInfo.validBitMap &= (~pa_IP_INFO_VALID_DST);

        /* Unset the valid bit, if SPI is set to "any". */
        if (ptrIPCfg->spidMode == Netfp_SPIDMode_ANY_SECURE)
            ipInfo.validBitMap  &=  (~pa_IP_INFO_VALID_SPI);

        if (ptrIPCfg->protocol == 0)
            ipInfo.validBitMap &= (~pa_IP_INFO_VALID_PROTO);
    }

    /* Initialize the routing information. */
    memset ((void *)&routeInfo, 0, sizeof(paRouteInfo2_t));

    /* Is this is the last rule? */
    if (ptrDstInfo->dstType == Netfp_DestType_LAST_RULE)
    {
        /* YES. We need to be sure a valid flow was specified. */
        if (ptrDstInfo->flowId == 0xFFFFFFFF)
            return NETFP_EINVAL;

        /* Security mode is not supported if the IP rule is the LAST rule. */
        if (ptrIPSecChannel != NULL)
            return NETFP_EINVAL;

        /* No Security. Configure the routing information appropriately. */
        routeInfo.dest          = pa_DEST_HOST;
        routeInfo.mRouteIndex   = pa_NO_MULTI_ROUTE;
        routeInfo.swInfo0       = 0;
        routeInfo.swInfo1       = 0;
        routeInfo.queue         = NETFP_CFG_BOUNCE_Q(ptrDstInfo->channel);
        routeInfo.flowId        = ptrDstInfo->flowId;
        routeInfo.validBitMap   = 0;
    }
    else
    {
        /* This is a LINKED rule. */
        if (ptrIPSecChannel == NULL)
        {
            /* No Security. Setup the routing information. */
            routeInfo.dest = pa_DEST_CONTINUE_PARSE_LUT2;
        }
        else
        {
            /* Security is ENABLED. We need to pass this packet to the SA first. */
            routeInfo.dest          = pa_DEST_SASS;
            routeInfo.mRouteIndex   = pa_NO_MULTI_ROUTE;
            routeInfo.swInfo0       = ptrIPSecChannel->ptrSrvSecurityChannel->swInfo.rxInfo.swInfo[0];
            routeInfo.swInfo1       = ptrIPSecChannel->ptrSrvSecurityChannel->swInfo.rxInfo.swInfo[1];
            // PA does not use bounce for SA
            routeInfo.queue         = ptrNetfpServer->netcpTxQueue[NSS_SA_QUEUE_SASS_INDEX];
            routeInfo.flowId        = ptrDstInfo->flowId;
            routeInfo.validBitMap   = 0;
        }
    }

    /* Setup the default fail information. */
    if (Netfp_setupDefaultFailInfo(ptrNetfpServer, &failInfo) < 0)
        return NETFP_EINTERNAL;

    /* Add user stats for this LUT entry */
    if (ptrNetfpServer->cfg.enableIPLutEntryCount)
    {
        memset(&paCmd, 0, sizeof(paCmdInfo_t));

        /* Set User stats command */
        paCmd.cmd |= pa_CMD_USR_STATS;

        /* Initialize values for user stats */
        paCmd.params.usrStats.index    = ptrNetfpLutInfo->matchPktCntIndex;

        /* Add pCmd in routeInfo */
        routeInfo.pCmd          = &paCmd;
        routeInfo.validBitMap   |= pa_ROUTE_INFO_VALID_PCMD;
    }

    /* Allocate a packet for the configuration command. */
    ptrCmdPacket = Pktlib_allocPacket(ptrNetfpServer->cfg.pktlibInstHandle, ptrNetfpServer->cfg.cmdHeapHandle,
                                      pa_ADD_IP_MIN_CMD_BUF_SIZE_BYTES);
    if (ptrCmdPacket == NULL)
        return -1;

    /* Get the data buffer from the allocated packet. */
    Pktlib_getDataBuffer(ptrCmdPacket, &ptrDataBuffer, &dataBufferLen);

    /* Setup the command size. */
    cmdSize = (uint16_t)dataBufferLen;

    /* Add the IP Information to the PA subsystem. */
    retVal = Pa_addIp2 (ptrNetfpServer->paHandle,
                       &ipInfo,
                       &paramDesc,
                       ipHandle,
                       ptrDataBuffer,
                       &cmdSize,
                       &cmdReplyInfo,
                       &cmdDest);
    if (retVal != pa_OK)
    {
        /* Error: Configuration Failed. */
        Pktlib_freePacket(ptrNetfpServer->cfg.pktlibInstHandle, ptrCmdPacket);
        return retVal;
    }

    /* Set the packet & data buffer length. */
    Pktlib_setPacketLen(ptrCmdPacket, cmdSize);
    Pktlib_setDataBufferLen(ptrCmdPacket, cmdSize);

    /* Set the software information fields. This is required only so that the
     * PS information goes to the correct location. */
    Cppi_setSoftwareInfo0(Cppi_DescType_HOST, (Cppi_Desc *)ptrCmdPacket, 0x12345678);

    /* Mark the packet as a configuration packet */
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)ptrCmdPacket, (uint8_t *)&psCmd, 4);

    /* Ensure that the command packet is also written back */
    ptrNetfpServer->cfg.endMemAccess (ptrDataBuffer, dataBufferLen);

    /******************************************************************************
     * NOTE: There is an ownership change here from the DSP to the CPDMA. We need
     * to ensure that the packet contents here are written back.
     ******************************************************************************/
    Pktlib_releaseOwnership(ptrNetfpServer->cfg.pktlibInstHandle, ptrCmdPacket);

    /* Send the command to the PA. */
    Qmss_queuePushDescSize (ptrNetfpServer->netcpTxQueue[cmdDest - pa_CMD_TX_DEST_0 + NSS_PA_QUEUE_INPUT_INDEX], (Cppi_Desc *)ptrCmdPacket, 64);

    /* Wait for the result and process it accordingly by forwarding to the PA subsystem. */
    retVal = Netfp_processResponse(ptrNetfpServer, cmdReplyInfo.replyId, NULL);
    if (retVal < 0)
        return retVal;

    /* LUT1-x has been programmed successfully. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to delete the Layer3(IP) Information from the
 *      NETCP subsystem
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP server
 *  @param[in]  ipHandle
 *      Handle to the L3 entry to be deleted.
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0.
 *  @retval
 *      Error   -   <0 [Error code populated]
 */
int32_t Netfp_delIP
(
    Netfp_ServerMCB*    ptrNetfpServer,
    Netfp_L3Handle      ipHandle
)
{
    paCmdReply_t    cmdReplyInfo;
    paReturn_t      retVal;
    Ti_Pkt*         ptrCmdPacket;
    uint32_t        dataBufferLen;
    uint8_t*        ptrDataBuffer;
    int32_t         cmdDest;
    uint16_t        cmdSize;
    uint32_t        psCmd = PASAHO_PACFG_CMD;

    /* Sanity Check: Ensure that this is a valid handle */
    if (ipHandle == NULL )
        return NETFP_EINVAL;

    /* TODO: This part needs to be investigated. Why do we need to perform the reference counter
     * check here. If the fast path is being deleted then lets kill the LUT handle immediately.
     * We cannot wait for all the NETFP clients to go ahead and kill the LUT2 handles. Events
     * have been passed to the clients and the sockets/3GPP channels will eventually be killed
     * also. */
#if 0
    {
        uint16_t        refCount;

        /* Get the reference counter */
        retVal = Pa_getHandleRefCount (ptrNetfpServer->paHandle, ipHandle, &refCount);
        if (retVal != pa_OK)
            return retVal;

        /* Are there any references held to this? This is the number of GTPU Tunnels
         * created on top of the GTPU Fast Path Handle. */
        if (refCount != 0)
            return NETFP_EINUSE;
    }
#endif

    /* Initialize the command reply:
     *  - We want command replies to come to the Host in the PA Configuration
     *    Response queue and also use the Flow which we had configured. */
    memset ((void *)&cmdReplyInfo, 0, sizeof(paCmdReply_t));
    Netfp_fillPaCmdReply(&cmdReplyInfo, ptrNetfpServer);

    /* Allocate a packet for the configuration command. */
    ptrCmdPacket = Pktlib_allocPacket(ptrNetfpServer->cfg.pktlibInstHandle, ptrNetfpServer->cfg.cmdHeapHandle,
                                      pa_DEL_HANDLE_MIN_CMD_BUF_SIZE_BYTES);
    if (ptrCmdPacket == NULL)
        return NETFP_ENOMEM;

    /* Get the data buffer from the allocated packet. */
    Pktlib_getDataBuffer(ptrCmdPacket, &ptrDataBuffer, &dataBufferLen);

    /* Setup the command size. */
    cmdSize = (uint16_t)dataBufferLen;

    /* Delete the GTPU Fast Path Handle from the NETCP Subsystem. */
    retVal = Pa_delHandle(ptrNetfpServer->paHandle,
                          (paHandleL2L3_t *)&ipHandle,
                          ptrDataBuffer,
                          &cmdSize,
                          &cmdReplyInfo,
                          &cmdDest);
    if (retVal != pa_OK)
    {
        /* Error: Configuration Failed. */
        Pktlib_freePacket(ptrNetfpServer->cfg.pktlibInstHandle, ptrCmdPacket);
        return retVal;
    }

    /* Set the packet & data buffer length. */
    Pktlib_setPacketLen(ptrCmdPacket, cmdSize);
    Pktlib_setDataBufferLen(ptrCmdPacket, cmdSize);

    /* Set the software information fields. This is required only so that the
     * PS information goes to the correct location. */
    Cppi_setSoftwareInfo0(Cppi_DescType_HOST, (Cppi_Desc *)ptrCmdPacket, 0x12345678);

    /* Mark the packet as a configuration packet */
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)ptrCmdPacket, (uint8_t *)&psCmd, 4);

    /* Ensure that the command packet is also written back */
    ptrNetfpServer->cfg.endMemAccess(ptrDataBuffer, dataBufferLen);

    /******************************************************************************
     * NOTE: There is an ownership change here from the DSP to the CPDMA. We need
     * to ensure that the packet contents here are written back.
     ******************************************************************************/
    Pktlib_releaseOwnership(ptrNetfpServer->cfg.pktlibInstHandle, ptrCmdPacket);

    /* Send the command to the PA. */
    Qmss_queuePushDescSize (ptrNetfpServer->netcpTxQueue[cmdDest - pa_CMD_TX_DEST_0 + NSS_PA_QUEUE_INPUT_INDEX], (Cppi_Desc *)ptrCmdPacket, 64);

    /* Wait for the result and process it accordingly by forwarding to the PA subsystem. */
    retVal = Netfp_processResponse(ptrNetfpServer, cmdReplyInfo.replyId, NULL);
    if (retVal < 0)
        return retVal;
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to add the GTP-U Tunnel Information to the NETCP
 *      subsystem
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP server
 *  @param[in]  gtpTunnelEndpointId
 *      GTP-U Tunnel endpoint identifier.
 *  @param[in]  l3Handle
 *      This is the handle from the L3(Ethernet) over which the UDP rule will reside
 *  @param[in]  swInfo
 *      Software Information word to be added to the route information.
 *  @param[in]  ptrSrvSecurityChannel
 *      Optional Security channel if any associated with this tunnel. If configured the
 *      packet will then be routed to the SA using this security channel.
 *  @param[in]  ptrDstInfo
 *      Destination information which specifies the action to be taken by the NETCP
 *      subsystem on a match.
 *  @param[in]  replace
 *      Indicates if adding a new LUT2 entry or updating an existing one
 *      subsystem on a match.
 *  @param[in]  removeHeaders
 *      Indicates if the L2/L3 header have to be stripped.
 *  @param[out] l4Handle
 *      This is the layer 4 Handle which is populated by this API.
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0 [Error code populated]
 */
int32_t Netfp_addGTPTunnel
(
    Netfp_ServerMCB*            ptrNetfpServer,
    uint32_t                    gtpTunnelEndpointId,
    Netfp_L3Handle              l3Handle,
    uint32_t                    swInfo,
    Netfp_SrvSecurityChannel*   ptrSrvSecurityChannel,
    Netfp_DstInfo*              ptrDstInfo,
    uint8_t                     replace,
    uint8_t                     removeHeaders,              /* TODO: This field can be obsoleted. */
    paHandleL4_t*               l4Handle
)
{
    paRouteInfo_t       routeInfo;
    paCmdReply_t        cmdReplyInfo;
    paReturn_t          retVal;
    Ti_Pkt*             ptrCmdPacket;
    uint32_t            dataBufferLen;
    uint8_t*            ptrDataBuffer;
    int32_t             cmdDest;
    uint16_t            cmdSize;
    uint32_t            psCmd = PASAHO_PACFG_CMD;
    paCmdInfo_t         cmdSetInfo;

    /* Sanity Check: Make sure there are valid arguments. */
    if ((ptrDstInfo == NULL) || (l3Handle == NULL))
        return NETFP_EINVAL;

    /* Sanity Check: No more linking is allowed. */
    if (ptrDstInfo->dstType == Netfp_DestType_LINKED_RULE)
        return NETFP_EINVAL;

    /* Sanity Check: We need to be sure a valid flow was specified. */
    if (ptrDstInfo->flowId == 0xFFFFFFFF)
        return NETFP_EINVAL;

    /* Sanity Check: We need to make sure a valid destination channel has been specified */
    if (ptrDstInfo->channel == 0)
        return NETFP_EINVAL;

    /* Initialize the command reply:
     *  - We want command replies to come to the Host in the PA Configuration
     *    Response queue and also use the Flow which we had configured. */
    memset ((void *)&cmdReplyInfo, 0, sizeof(paCmdReply_t));
    Netfp_fillPaCmdReply(&cmdReplyInfo, ptrNetfpServer);

    /* Initialize the routing information. */
    memset ((void *)&routeInfo, 0, sizeof(paRouteInfo_t));

    /* Is there a security channel associated with this IP rule? */
    if (ptrSrvSecurityChannel == NULL)
    {
        /* Non Fast Path mode: Pass the packet to the software queue after a LUT2 i.e.
         * GTPU identifier match. */
        routeInfo.dest          = pa_DEST_HOST;
        routeInfo.mRouteIndex   = pa_NO_MULTI_ROUTE;
        routeInfo.swInfo0       = swInfo;
        routeInfo.swInfo1       = 0;
        routeInfo.queue         = NETFP_CFG_BOUNCE_Q(ptrDstInfo->channel);
        routeInfo.flowId        = ptrDstInfo->flowId;

        /* Configure the command set to remove header, patch data and remove tail */
        cmdSetInfo.cmd                  = pa_CMD_CMDSET;
        cmdSetInfo.params.cmdSet.index  = ptrNetfpServer->cmdSet.cmdSetHdrRemoveIndex;
        routeInfo.pCmd                  = &cmdSetInfo;
    }
    else
    {
        /* Fast Path mode: Pass the packet to the SA subsystem for air ciphering after the
         * GTPU identifier match. */
        routeInfo.dest          = pa_DEST_SASS;
        routeInfo.mRouteIndex   = pa_NO_MULTI_ROUTE;
        routeInfo.swInfo0       = ptrSrvSecurityChannel->swInfo.rxInfo.swInfo[0];
        routeInfo.swInfo1       = ptrSrvSecurityChannel->swInfo.rxInfo.swInfo[1];
        // PA does not use bounce for SA
        //routeInfo.queue         = PA_BOUNCE_QUEUE_NONE(ptrDstInfo->channel);
        routeInfo.queue         = ptrDstInfo->channel;//ptrNetfpServer->netcpTxQueue[NSS_SA_QUEUE_SASS2_INDEX];
        routeInfo.flowId        = ptrDstInfo->flowId;

        /* Configure the command set to remove header, patch data and remove tail */
        cmdSetInfo.cmd                  = pa_CMD_CMDSET;
        cmdSetInfo.params.cmdSet.index  = ptrNetfpServer->cmdSet.cmdSetHdrRemoveIndex;
        routeInfo.pCmd                  = &cmdSetInfo;
    }

    /* Allocate a packet for the configuration command. */
    ptrCmdPacket = Pktlib_allocPacket(ptrNetfpServer->cfg.pktlibInstHandle,
                                      ptrNetfpServer->cfg.cmdHeapHandle,
                                      pa_ADD_PORT_MIN_CMD_BUF_SIZE_BYTES);
    if (ptrCmdPacket == NULL)
        return NETFP_ENOMEM;

    /* Get the data buffer from the allocated packet. */
    Pktlib_getDataBuffer(ptrCmdPacket, &ptrDataBuffer, &dataBufferLen);

    /* Setup the command size. */
    cmdSize = (uint16_t)dataBufferLen;

    /* Add the Port Information to the PA subsystem. */
    retVal = Pa_addPort (ptrNetfpServer->paHandle,
                         pa_LUT2_PORT_SIZE_32,
                         gtpTunnelEndpointId,
                         l3Handle,
                         replace,
                         pa_PARAMS_NOT_SPECIFIED,
                         &routeInfo,
                         *l4Handle,
                         ptrDataBuffer,
                         &cmdSize,
                         &cmdReplyInfo,
                         &cmdDest);
    if (retVal != pa_OK)
    {
        /* Error: Configuration Failed. */
        Pktlib_freePacket(ptrNetfpServer->cfg.pktlibInstHandle, ptrCmdPacket);
        return retVal;
    }

    /* Set the packet & data buffer length. */
    Pktlib_setPacketLen(ptrCmdPacket, cmdSize);
    Pktlib_setDataBufferLen(ptrCmdPacket, cmdSize);

    /* Set the software information fields. This is required only so that the
     * PS information goes to the correct location. */
    Cppi_setSoftwareInfo0(Cppi_DescType_HOST, (Cppi_Desc *)ptrCmdPacket, 0x12345678);

    /* Mark the packet as a configuration packet */
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)ptrCmdPacket, (uint8_t *)&psCmd, 4);

    /* Ensure that the command packet is also written back */
    ptrNetfpServer->cfg.endMemAccess(ptrDataBuffer, dataBufferLen);

    /******************************************************************************
     * NOTE: There is an ownership change here from the DSP to the CPDMA. We need
     * to ensure that the packet contents here are written back.
     ******************************************************************************/
    Pktlib_releaseOwnership(ptrNetfpServer->cfg.pktlibInstHandle, ptrCmdPacket);

    /* Send the command to the PA. */
    Qmss_queuePushDescSize (ptrNetfpServer->netcpTxQueue[cmdDest - pa_CMD_TX_DEST_0 + NSS_PA_QUEUE_INPUT_INDEX], (Cppi_Desc *)ptrCmdPacket, 64);

    /* Wait for the result and process it accordingly by forwarding to the PA subsystem. */
    retVal = Netfp_processResponse(ptrNetfpServer, cmdReplyInfo.replyId, NULL);
    if (retVal < 0)
        return retVal;

    /* GTPU tunnel has been successfully added. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to add a destination port number to the NETCP subsystem.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP server
 *  @param[in]  portNumber
 *      TCP/UDP Destination Port Number
 *  @param[in]  l3Handle
 *      This is the handle from the L3(Ethernet) over which the UDP rule will reside
 *  @param[in]  flowId
 *      Flow Identifier which is used to receive packets matching the specified destination
 *      port number
 *  @param[in]  swInfo
 *      Software Information Word added to the route information.
 *  @param[in]  ptrDstInfo
 *      Destination information which specifies the action to be taken by the NETCP
 *      subsystem on a match.
 *  @param[in]  ptrCmdSetInfo
 *      Pointer to the command set information.
 *  @param[out]  l4Handle
 *      This is the layer 4 Handle which is populated by this API.
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0 [See error code]
 */
int32_t Netfp_addPort
(
    Netfp_ServerMCB*    ptrNetfpServer,
    uint16_t            portNumber,
    Netfp_L3Handle      l3Handle,
    uint32_t            flowId,
    uint32_t            swInfo,
    Netfp_DstInfo*      ptrDstInfo,
    paCmdInfo_t*        ptrCmdSetInfo,
    paHandleL4_t*       l4Handle
)
{
    paRouteInfo_t   routeInfo;
    paCmdReply_t    cmdReplyInfo;
    paReturn_t      retVal;
    Ti_Pkt*         ptrCmdPacket;
    uint32_t        dataBufferLen;
    uint8_t*        ptrDataBuffer;
    int32_t         cmdDest;
    uint16_t        cmdSize;
    uint32_t        psCmd = PASAHO_PACFG_CMD;

    /* Sanity Check: Make sure there are valid arguments. */
    if ((ptrDstInfo == NULL) || (l3Handle == NULL))
        return NETFP_EINVAL;

    /* Sanity Check: No more linking is allowed. */
    if (ptrDstInfo->dstType == Netfp_DestType_LINKED_RULE)
        return NETFP_EINVAL;

    /* Get the flow information. Make sure we have a valid flow handle. */
    if (flowId == 0xFFFFFFFF)
        return NETFP_EINVAL;

    /* Initialize the command reply:
     *  - We want command replies to come to the Host in the PA Configuration
     *    Response queue and also use the Flow which we had configured. */
    memset ((void *)&cmdReplyInfo, 0, sizeof(paCmdReply_t));
    Netfp_fillPaCmdReply(&cmdReplyInfo, ptrNetfpServer);

    /* Initialize the routing information. */
    memset ((void *)&routeInfo, 0, sizeof(paRouteInfo_t));

    /* Configure the routing information appropriately. */
    routeInfo.dest          = pa_DEST_HOST;
    routeInfo.mRouteIndex   = pa_NO_MULTI_ROUTE;
    routeInfo.swInfo0       = swInfo;
    routeInfo.swInfo1       = 0;
    routeInfo.queue         = NETFP_CFG_BOUNCE_Q(ptrDstInfo->channel);
    routeInfo.flowId        = flowId;
    routeInfo.pCmd          = ptrCmdSetInfo;

    /* Allocate a packet for the configuration command. */
    ptrCmdPacket = Pktlib_allocPacket(ptrNetfpServer->cfg.pktlibInstHandle, ptrNetfpServer->cfg.cmdHeapHandle,
                                      pa_ADD_PORT_MIN_CMD_BUF_SIZE_BYTES);
    if (ptrCmdPacket == NULL)
        return NETFP_ENOMEM;

    /* Get the data buffer from the allocated packet. */
    Pktlib_getDataBuffer(ptrCmdPacket, &ptrDataBuffer, &dataBufferLen);

    /* Setup the command size. */
    cmdSize = (uint16_t)dataBufferLen;

    /* Add the Port Information to the PA subsystem. */
    retVal = Pa_addPort (ptrNetfpServer->paHandle,
                         pa_LUT2_PORT_SIZE_16,
                         portNumber,
                         l3Handle,
                         FALSE,
                         pa_PARAMS_NOT_SPECIFIED,
                         &routeInfo,
                         *l4Handle,
                         ptrDataBuffer,
                         &cmdSize,
                         &cmdReplyInfo,
                         &cmdDest);
    if (retVal != pa_OK)
    {
        /* Error: Configuration Failed. */
        Pktlib_freePacket(ptrNetfpServer->cfg.pktlibInstHandle, ptrCmdPacket);
        return retVal;
    }

    /* Set the packet & data buffer length. */
    Pktlib_setPacketLen(ptrCmdPacket, cmdSize);
    Pktlib_setDataBufferLen(ptrCmdPacket, cmdSize);

    /* Set the software information fields. This is required only so that the
     * PS information goes to the correct location. */
    Cppi_setSoftwareInfo0(Cppi_DescType_HOST, (Cppi_Desc *)ptrCmdPacket, 0x12345678);

    /* Mark the packet as a configuration packet */
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)ptrCmdPacket, (uint8_t *)&psCmd, 4);

    /* Ensure that the command packet is also written back */
    ptrNetfpServer->cfg.endMemAccess(ptrDataBuffer, dataBufferLen);

    /******************************************************************************
     * NOTE: There is an ownership change here from the DSP to the CPDMA. We need
     * to ensure that the packet contents here are written back.
     ******************************************************************************/
    Pktlib_releaseOwnership(ptrNetfpServer->cfg.pktlibInstHandle, ptrCmdPacket);

    /* Send the command to the PA. */
    Qmss_queuePushDescSize (ptrNetfpServer->netcpTxQueue[cmdDest - pa_CMD_TX_DEST_0 + NSS_PA_QUEUE_INPUT_INDEX], (Cppi_Desc *)ptrCmdPacket, 64);

    /* Wait for the result and process it accordingly by forwarding to the PA subsystem. */
    retVal = Netfp_processResponse(ptrNetfpServer, cmdReplyInfo.replyId, NULL);
    if (retVal < 0)
        return retVal;

    /* Layer 4 has been successfully added. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to delete a layer4 handle from the NETCP subsystem.
 *      The Layer4 entries are associated with UDP ports and GTPU endpoints.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP server
 *  @param[in]  l4Handle
 *      Layer4 Handle to be deleted
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_delL4Handle
(
    Netfp_ServerMCB*    ptrNetfpServer,
    paHandleL4_t        l4Handle
    )
{
    paCmdReply_t    cmdReplyInfo;
    paReturn_t      retVal;
    Ti_Pkt*         ptrCmdPacket;
    uint32_t        dataBufferLen;
    uint8_t*        ptrDataBuffer;
    int32_t         cmdDest;
    uint16_t        cmdSize;
    uint32_t        psCmd = PASAHO_PACFG_CMD;

    /* Sanity Check: Make sure there are valid arguments. */
    if (l4Handle == NULL)
        return NETFP_EINVAL;

    /* Initialize the command reply:
     *  - We want command replies to come to the Host in the PA Configuration
     *    Response queue and also use the Flow which we had configured. */
    memset ((void *)&cmdReplyInfo, 0, sizeof(paCmdReply_t));
    Netfp_fillPaCmdReply(&cmdReplyInfo, ptrNetfpServer);

    /* Allocate a packet for the configuration command. */
    ptrCmdPacket = Pktlib_allocPacket(ptrNetfpServer->cfg.pktlibInstHandle, ptrNetfpServer->cfg.cmdHeapHandle,
                                      pa_DEL_L4_HANDLE_MIN_CMD_BUF_SIZE_BYTES);
    if (ptrCmdPacket == NULL)
        return -1;

    /* Get the data buffer from the allocated packet. */
    Pktlib_getDataBuffer(ptrCmdPacket, &ptrDataBuffer, &dataBufferLen);

    /* Setup the command size. */
    cmdSize = (uint16_t)dataBufferLen;

    /* Delete the L4 Handle from the NETCP Subsystem. */
    retVal = Pa_delL4Handle(ptrNetfpServer->paHandle,
                            (uint32_t*)l4Handle,
                            ptrDataBuffer,
                            &cmdSize,
                            &cmdReplyInfo,
                            &cmdDest);
    if (retVal != pa_OK)
    {
        /* Error: Configuration Failed. */
        Pktlib_freePacket(ptrNetfpServer->cfg.pktlibInstHandle, ptrCmdPacket);
        return retVal;
    }

    /* Set the packet & data buffer length. */
    Pktlib_setPacketLen(ptrCmdPacket, cmdSize);
    Pktlib_setDataBufferLen(ptrCmdPacket, cmdSize);

    /* Set the software information fields. This is required only so that the
     * PS information goes to the correct location. */
    Cppi_setSoftwareInfo0(Cppi_DescType_HOST, (Cppi_Desc *)ptrCmdPacket, 0x12345678);

    /* Mark the packet as a configuration packet */
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)ptrCmdPacket, (uint8_t *)&psCmd, 4);

    /* Ensure that the command packet is also written back */
    ptrNetfpServer->cfg.endMemAccess(ptrDataBuffer, dataBufferLen);

    /******************************************************************************
     * NOTE: There is an ownership change here from the DSP to the CPDMA. We need
     * to ensure that the packet contents here are written back.
     ******************************************************************************/
    Pktlib_releaseOwnership(ptrNetfpServer->cfg.pktlibInstHandle, ptrCmdPacket);

    /* Send the command to the PA. */
    Qmss_queuePushDescSize (ptrNetfpServer->netcpTxQueue[cmdDest - pa_CMD_TX_DEST_0 + NSS_PA_QUEUE_INPUT_INDEX], (Cppi_Desc *)ptrCmdPacket, 64);

    /* Wait for the result and process it accordingly by forwarding to the PA subsystem. */
    retVal = Netfp_processResponse(ptrNetfpServer, cmdReplyInfo.replyId, NULL);
    if (retVal < 0)
        return retVal;

    /* Layer4 Handle has been successfully deleted */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to initialize the reassembly module in the
 *      NETCP subsystem.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP server
 *  @param[in]  ptrInnerIPReassemblyConfig
 *      Pointer to the inner IP reassembly configuration
 *  @param[in]  ptrOuterIPReassemblyConfig
 *      Pointer to the outer IP reassembly configuration
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
int32_t _Netfp_configureReassembly
(
    Netfp_ServerMCB*        ptrNetfpServer,
    paIpReassmConfig_t*     ptrInnerIPReassemblyConfig,
    paIpReassmConfig_t*     ptrOuterIPReassemblyConfig,
    int32_t*                errCode
)
{
    paCmdReply_t    cmdReplyInfo;
    paReturn_t      retVal;
    Ti_Pkt*         ptrCmdPacket;
    uint32_t        dataBufferLen;
    uint8_t*        ptrDataBuffer;
    int32_t         cmdDest;
    uint16_t        cmdSize;
    uint32_t        psCmd = PASAHO_PACFG_CMD;
    paCtrlInfo_t    configInfo;

    /* Initialize the configuration information. */
    memset ((void *)&configInfo, 0, sizeof(paCtrlInfo_t));

    ptrOuterIPReassemblyConfig->destQueue = NETFP_CFG_BOUNCE_Q(ptrOuterIPReassemblyConfig->destQueue);
    ptrInnerIPReassemblyConfig->destQueue = NETFP_CFG_BOUNCE_Q(ptrInnerIPReassemblyConfig->destQueue);

    /* Populate the system configuration: */
    configInfo.code                             = pa_CONTROL_SYS_CONFIG;
    configInfo.params.sysCfg.pOutIpReassmConfig = ptrOuterIPReassemblyConfig;
    configInfo.params.sysCfg.pInIpReassmConfig  = ptrInnerIPReassemblyConfig;
    configInfo.params.sysCfg.pProtoLimit        = NULL;
    configInfo.params.sysCfg.pCmdSetConfig      = NULL;
    configInfo.params.sysCfg.pUsrStatsConfig    = NULL;
    configInfo.params.sysCfg.pQueueDivertConfig = NULL;

    /* Initialize the command reply:
     *  - We want command replies to come to the Host in the PA Configuration
     *    Response queue and also use the Flow which we had configured. */
    memset ((void *)&cmdReplyInfo, 0, sizeof(paCmdReply_t));
    Netfp_fillPaCmdReply(&cmdReplyInfo, ptrNetfpServer);

    /* Allocate a packet for the configuration command. */
    ptrCmdPacket = Pktlib_allocPacket(ptrNetfpServer->cfg.pktlibInstHandle,
                                      ptrNetfpServer->cfg.cmdHeapHandle,
                                      pa_GLOBAL_CONFIG_MIN_CMD_BUF_SIZE_BYTES);
    if (ptrCmdPacket == NULL)
    {
        *errCode = NETFP_ENOMEM;
        return -1;
    }

    /* Get the data buffer from the allocated packet. */
    Pktlib_getDataBuffer(ptrCmdPacket, &ptrDataBuffer, &dataBufferLen);

    /* Setup the command size. */
    cmdSize = (uint16_t)dataBufferLen;

    /* Set the global config in the PA Susbystem. */
    retVal = Pa_control (ptrNetfpServer->paHandle,
                         &configInfo,
                         (paCmd_t)ptrDataBuffer,
                         &cmdSize,
                         &cmdReplyInfo,
                         &cmdDest);
    if (retVal != pa_OK)
    {
        /* Error: Configuration Failed. */
        *errCode = retVal;
        Pktlib_freePacket(ptrNetfpServer->cfg.pktlibInstHandle, ptrCmdPacket);
        return -1;
    }

    /* Set the packet & data buffer length. */
    Pktlib_setPacketLen(ptrCmdPacket, cmdSize);
    Pktlib_setDataBufferLen(ptrCmdPacket, cmdSize);

    /* Set the software information fields. This is required only so that the
     * PS information goes to the correct location. */
    Cppi_setSoftwareInfo0(Cppi_DescType_HOST, (Cppi_Desc *)ptrCmdPacket, 0x12345678);

    /* Mark the packet as a configuration packet */
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)ptrCmdPacket, (uint8_t *)&psCmd, 4);

    /* Ensure that the command packet is also written back */
    ptrNetfpServer->cfg.endMemAccess(ptrDataBuffer, dataBufferLen);

    /******************************************************************************
     * NOTE: There is an ownership change here from the DSP to the CPDMA. We need
     * to ensure that the packet contents here are written back.
     ******************************************************************************/
    Pktlib_releaseOwnership(ptrNetfpServer->cfg.pktlibInstHandle, ptrCmdPacket);

    /* Send the command to the PA. */
    Qmss_queuePushDescSize (ptrNetfpServer->netcpTxQueue[cmdDest - pa_CMD_TX_DEST_0 + NSS_PA_QUEUE_INPUT_INDEX], (Cppi_Desc *)ptrCmdPacket, 64);

    /* Wait for the result and process it accordingly by forwarding to the PA subsystem. */
    retVal = Netfp_processResponse(ptrNetfpServer, cmdReplyInfo.replyId, NULL);
    if (retVal < 0)
    {
        *errCode = retVal;
        return -1;
    }

    /* Global configuration has been successfully added. */
    return 0;
}


/**
*  @b Description
*  @n
*      The function is used to initialize the PA queue bounce
 *      to address ARM DMA coherency issue.
*
*  @param[in]  netfpServerHandle
*      NETFP Server Handle
*  @param[in]  paQueueBounceConfig
*      PA queue bounce configuration
*  @param[out] errCode
*      Error code populated by the API
*
* \ingroup NETFP_FUNCTION
*
*  @retval
*      Success -   0
*  @retval
*      Error   -  <0
*/
int32_t Netfp_initQueueBounce
(
    Netfp_ServerHandle      netfpServerHandle,
    paQueueBounceConfig_t   paQueueBounceConfig,
    int32_t*                errCode
)
{
    paCmdReply_t             cmdReplyInfo;
    paReturn_t               retVal;
    Ti_Pkt*                  ptrCmdPacket;
    uint32_t                 dataBufferLen;
    uint8_t*                 ptrDataBuffer;
    int32_t                  cmdDest;
    uint16_t                 cmdSize;
    uint32_t                 psCmd = PASAHO_PACFG_CMD;
    paCtrlInfo_t             configInfo;
    Netfp_ServerMCB*         ptrNetfpServer;

    /* Sanity Check: Validate the arguments: */
    if (netfpServerHandle == NULL)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Get the pointer to the NETFP Server: */
    ptrNetfpServer = (Netfp_ServerMCB*)netfpServerHandle;

    Netfp_logMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "Debug: Queue bounce configuration:\n");
    Netfp_logMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "   enable: %d ddrQueueId: %d msmcQueueId; %d hwQueueBegin: %d hwQueueEnd: %d\n",
                  paQueueBounceConfig.enable,
                  paQueueBounceConfig.ddrQueueId,
                  paQueueBounceConfig.msmcQueueId,
                  paQueueBounceConfig.hwQueueBegin,
                  paQueueBounceConfig.hwQueueEnd );
    Netfp_logMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "   defOp[0]: %d defOp[1]: %d defOp[2]; %d defOp[3]: %d defOp[4]: %d\n",
                    paQueueBounceConfig.defOp[pa_QUEUE_BOUNCE_ROUTING_CLASS_CMD_RET],
                    paQueueBounceConfig.defOp[pa_QUEUE_BOUNCE_ROUTING_CLASS_QoS],
                    paQueueBounceConfig.defOp[pa_QUEUE_BOUNCE_ROUTING_CLASS_CAPTURE],
                    paQueueBounceConfig.defOp[pa_QUEUE_BOUNCE_ROUTING_CLASS_IP_REASSEMBLY],
                    paQueueBounceConfig.defOp[pa_QUEUE_BOUNCE_ROUTING_CLASS_MISC]);

    if ( paQueueBounceConfig.enable == 0 )
    {
        *errCode = 0;
        return 0;
    }
    else if (paQueueBounceConfig.ddrQueueId == (typeof(paQueueBounceConfig.ddrQueueId))QMSS_PARAM_NOT_SPECIFIED ||
             paQueueBounceConfig.msmcQueueId == (typeof(paQueueBounceConfig.msmcQueueId))QMSS_PARAM_NOT_SPECIFIED)
    {
        *errCode = 0;
        return 0;
    }

    /* Initialize the configuration information. */
    memset ((void *)&configInfo, 0, sizeof(paCtrlInfo_t));

    /* Populate the system configuration: */
    configInfo.code                             = pa_CONTROL_SYS_CONFIG;
    configInfo.params.sysCfg.pQueueBounceConfig = &paQueueBounceConfig;

    /* Initialize the command reply:
     *  - We want command replies to come to the Host in the PA Configuration
     *    Response queue and also use the Flow which we had configured. */
    memset ((void *)&cmdReplyInfo, 0, sizeof(paCmdReply_t));
    Netfp_fillPaCmdReply(&cmdReplyInfo, ptrNetfpServer);

    /* Allocate a packet for the configuration command. */
    ptrCmdPacket = Pktlib_allocPacket(ptrNetfpServer->cfg.pktlibInstHandle, ptrNetfpServer->cfg.cmdHeapHandle,
    pa_GLOBAL_CONFIG_MIN_CMD_BUF_SIZE_BYTES);
    if (ptrCmdPacket == NULL)
    {
        *errCode = NETFP_ENOMEM;
        return -1;
    }

    /* Get the data buffer from the allocated packet. */
    Pktlib_getDataBuffer(ptrCmdPacket, &ptrDataBuffer, &dataBufferLen);

    /* Setup the command size. */
    cmdSize = (uint16_t)dataBufferLen;

    /* Set the global config in the PA Susbystem. */
    retVal = Pa_control (ptrNetfpServer->paHandle,
                            &configInfo,
                            (paCmd_t)ptrDataBuffer,
                            &cmdSize,
                            &cmdReplyInfo,
                            &cmdDest);

    if (retVal != pa_OK)
    {
        /* Error: Configuration Failed. */
        *errCode = (int32_t)retVal;
        Pktlib_freePacket(ptrNetfpServer->cfg.pktlibInstHandle, ptrCmdPacket);
        return -1;
    }

    /* Set the packet & data buffer length. */
    Pktlib_setPacketLen(ptrCmdPacket, cmdSize);
    Pktlib_setDataBufferLen(ptrCmdPacket, cmdSize);

    /* Set the software information fields. This is required only so that the
    * PS information goes to the correct location. */
    Cppi_setSoftwareInfo0(Cppi_DescType_HOST, (Cppi_Desc *)ptrCmdPacket, 0x12345678);

    /* Mark the packet as a configuration packet */
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)ptrCmdPacket, (uint8_t *)&psCmd, 4);

    /* Ensure that the command packet is also written back */
    ptrNetfpServer->cfg.endMemAccess(ptrDataBuffer, dataBufferLen);

    /******************************************************************************
    * NOTE: There is an ownership change here from the DSP to the CPDMA. We need
    * to ensure that the packet contents here are written back.
    ******************************************************************************/
    Pktlib_releaseOwnership(ptrNetfpServer->cfg.pktlibInstHandle, ptrCmdPacket);

    /* Send the command to the PA. */
    Qmss_queuePushDescSize (ptrNetfpServer->netcpTxQueue[cmdDest - pa_CMD_TX_DEST_0 + NSS_PA_QUEUE_INPUT_INDEX], (Cppi_Desc *)ptrCmdPacket, 64);

    /* Wait for the result and process it accordingly by forwarding to the PA subsystem. */
    retVal = Netfp_processResponse(ptrNetfpServer, cmdReplyInfo.replyId, NULL);
    if (retVal < 0)
    {
        *errCode = retVal;
        return -1;
    }

    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to initialized the Enhanced QOS module for
 *      the entire system.
 *
 *  @param[in]  netfpServerHandle
 *      NETFP Server Handle
 *  @param[in]  egressDefaultPriority
 *      Egress default priority to be used for non-ip packets.
 *  @param[out] errCode
 *      Error code populated by the API
 *
 * \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -  <0
 */
int32_t Netfp_initEQOS
(
    Netfp_ServerHandle  netfpServerHandle,
    uint8_t             egressDefaultPriority,
    int32_t*            errCode
)
{
    paCmdReply_t             cmdReplyInfo;
    paReturn_t               retVal;
    Ti_Pkt*                  ptrCmdPacket;
    uint32_t                 dataBufferLen;
    uint8_t*                 ptrDataBuffer;
    int32_t                  cmdDest;
    uint16_t                 cmdSize;
    uint32_t                 psCmd = PASAHO_PACFG_CMD;
    paCtrlInfo_t             configInfo;
    paPacketControl2Config_t pktControl2;
    Netfp_ServerMCB*         ptrNetfpServer;

    /* Sanity Check: Validate the arguments: */
    if (netfpServerHandle == NULL)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Get the pointer to the NETFP Server: */
    ptrNetfpServer = (Netfp_ServerMCB*)netfpServerHandle;

    /* Initialize the packet control information: */
    memset ((void *)&pktControl2, 0, sizeof(paPacketControl2Config_t));

    /* Populate the packet control information to enable the enhanced QOS */
    pktControl2.validBitMap  = pa_PKT_CTRL2_VALID_EMAC_IF_EGRESS_EQoS_MODE;
    pktControl2.ctrlBitMap   = pa_PKT_CTRL_EMAC_IF_EGRESS_EQoS_MODE;
    pktControl2.egressDefPri = egressDefaultPriority;

    /* Initialize the configuration information. */
    memset ((void *)&configInfo, 0, sizeof(paCtrlInfo_t));

    /* Populate the system configuration: */
    configInfo.code                       = pa_CONTROL_SYS_CONFIG;
    configInfo.params.sysCfg.pPktControl2 = &pktControl2;

    /* Initialize the command reply:
     *  - We want command replies to come to the Host in the PA Configuration
     *    Response queue and also use the Flow which we had configured. */
    memset ((void *)&cmdReplyInfo, 0, sizeof(paCmdReply_t));
    Netfp_fillPaCmdReply(&cmdReplyInfo, ptrNetfpServer);

    /* Allocate a packet for the configuration command. */
    ptrCmdPacket = Pktlib_allocPacket(ptrNetfpServer->cfg.pktlibInstHandle,
                                      ptrNetfpServer->cfg.cmdHeapHandle,
                                      pa_GLOBAL_CONFIG_MIN_CMD_BUF_SIZE_BYTES);
    if (ptrCmdPacket == NULL)
    {
        *errCode = NETFP_ENOMEM;
        return -1;
    }

    /* Get the data buffer from the allocated packet. */
    Pktlib_getDataBuffer(ptrCmdPacket, &ptrDataBuffer, &dataBufferLen);

    /* Setup the command size. */
    cmdSize = (uint16_t)dataBufferLen;

    /* Set the global config in the PA Susbystem. */
    retVal = Pa_control (ptrNetfpServer->paHandle,
                         &configInfo,
                         (paCmd_t)ptrDataBuffer,
                         &cmdSize,
                         &cmdReplyInfo,
                         &cmdDest);
    if (retVal != pa_OK)
    {
        /* Error: Configuration Failed. */
        *errCode = (int32_t)retVal;
        Pktlib_freePacket(ptrNetfpServer->cfg.pktlibInstHandle, ptrCmdPacket);
        return -1;
    }

    /* Set the packet & data buffer length. */
    Pktlib_setPacketLen(ptrCmdPacket, cmdSize);
    Pktlib_setDataBufferLen(ptrCmdPacket, cmdSize);

    /* Set the software information fields. This is required only so that the
     * PS information goes to the correct location. */
    Cppi_setSoftwareInfo0(Cppi_DescType_HOST, (Cppi_Desc *)ptrCmdPacket, 0x12345678);

    /* Mark the packet as a configuration packet */
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)ptrCmdPacket, (uint8_t *)&psCmd, 4);

    /* Ensure that the command packet is also written back */
    ptrNetfpServer->cfg.endMemAccess(ptrDataBuffer, dataBufferLen);

    /******************************************************************************
     * NOTE: There is an ownership change here from the DSP to the CPDMA. We need
     * to ensure that the packet contents here are written back.
     ******************************************************************************/
    Pktlib_releaseOwnership(ptrNetfpServer->cfg.pktlibInstHandle, ptrCmdPacket);

    /* Send the command to the PA. */
    Qmss_queuePushDescSize (ptrNetfpServer->netcpTxQueue[cmdDest - pa_CMD_TX_DEST_0 + NSS_PA_QUEUE_INPUT_INDEX], (Cppi_Desc *)ptrCmdPacket, 64);

    /* Wait for the result and process it accordingly by forwarding to the PA subsystem. */
    retVal = Netfp_processResponse(ptrNetfpServer, cmdReplyInfo.replyId, NULL);
    if (retVal < 0)
    {
        *errCode = retVal;
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to enable the enhanced QOS L2 shaper for each interface.
 *      Interface based QOS L2 shapers can only be enabled once the Enhanced QOS has
 *      been initialized for the entire system.
 *
 *  @sa
 *      Netfp_initEQOS
 *
 *  @param[in]  netfpServerHandle
 *      NETFP Server Handle
 *  @param[in]  ptrQoSCfg
 *      Pointer to the QOS configuration
 *  @param[out] errCode
 *      Error code populated by the API
 *
 * \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -  <0
 */
int32_t Netfp_enableL2Shaper
(
    Netfp_ServerHandle  netfpServerHandle,
    paEQosModeConfig_t* ptrQoSCfg,
    int32_t*            errCode
)
{
    paCmdReply_t        cmdReplyInfo;
    paReturn_t          retVal;
    Ti_Pkt*             ptrCmdPacket;
    uint32_t            dataBufferLen;
    uint8_t*            ptrDataBuffer;
    int32_t             cmdDest;
    uint16_t            cmdSize;
    uint32_t            psCmd = PASAHO_PACFG_CMD;
    paCtrlInfo_t        configInfo;
    Netfp_ServerMCB*    ptrNetfpServer;

    /* Sanity Check: Validate the arguments: */
    if ((netfpServerHandle == NULL) || (ptrQoSCfg == NULL))
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Get the pointer to the NETFP Server: */
    ptrNetfpServer = (Netfp_ServerMCB*)netfpServerHandle;

    /* Initialize the configuration information. */
    memset ((void *)&configInfo, 0, sizeof(paCtrlInfo_t));

    /* Populate the control information */
    configInfo.code                             = pa_CONTROL_EMAC_PORT_CONFIG;
    configInfo.params.emacPortCfg.cfgType       = pa_EMAC_PORT_CFG_EQoS_MODE;
    configInfo.params.emacPortCfg.numEntries    = 1;
    //ptrQoSCfg->queueBase = NETFP_CFG_BOUNCE_Q(ptrQoSCfg->queueBase); // TODO: Is this needed?
    configInfo.params.emacPortCfg.u.eQoSModeCfg = ptrQoSCfg;

    /* Initialize the command reply:
     *  - We want command replies to come to the Host in the PA Configuration
     *    Response queue and also use the Flow which we had configured. */
    memset ((void *)&cmdReplyInfo, 0, sizeof(paCmdReply_t));
    Netfp_fillPaCmdReply(&cmdReplyInfo, ptrNetfpServer);

    /* Allocate a packet for the configuration command. */
    ptrCmdPacket = Pktlib_allocPacket(ptrNetfpServer->cfg.pktlibInstHandle, ptrNetfpServer->cfg.cmdHeapHandle,
                                      pa_EMAC_PORT_EQoS_MODE_CONFIG_MIN_CMD_BUF_SIZE_BYTES);
    if (ptrCmdPacket == NULL)
    {
        *errCode = NETFP_ENOMEM;
        return -1;
    }

    /* Get the data buffer from the allocated packet. */
    Pktlib_getDataBuffer(ptrCmdPacket, &ptrDataBuffer, &dataBufferLen);

    /* Setup the command size. */
    cmdSize = (uint16_t)dataBufferLen;

    /* Set the enhanced QOS configuration: */
    retVal = Pa_control (ptrNetfpServer->paHandle,
                         &configInfo,
                         (paCmd_t)ptrDataBuffer,
                         &cmdSize,
                         &cmdReplyInfo,
                         &cmdDest);
    if (retVal != pa_OK)
    {
        /* Error: Configuration Failed. */
        Pktlib_freePacket(ptrNetfpServer->cfg.pktlibInstHandle, ptrCmdPacket);
        *errCode = (int32_t)retVal;
        return -1;
    }

    /* Set the packet & data buffer length. */
    Pktlib_setPacketLen(ptrCmdPacket, cmdSize);
    Pktlib_setDataBufferLen(ptrCmdPacket, cmdSize);

    /* Set the software information fields. This is required only so that the
     * PS information goes to the correct location. */
    Cppi_setSoftwareInfo0(Cppi_DescType_HOST, (Cppi_Desc *)ptrCmdPacket, 0x12345678);

    /* Mark the packet as a configuration packet */
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)ptrCmdPacket, (uint8_t *)&psCmd, 4);

    /* Ensure that the command packet is also written back */
    ptrNetfpServer->cfg.endMemAccess(ptrDataBuffer, dataBufferLen);

    /******************************************************************************
     * NOTE: There is an ownership change here from the DSP to the CPDMA. We need
     * to ensure that the packet contents here are written back.
     ******************************************************************************/
    Pktlib_releaseOwnership(ptrNetfpServer->cfg.pktlibInstHandle, ptrCmdPacket);

    /* Send the command to the PA. */
    Qmss_queuePushDescSize (ptrNetfpServer->netcpTxQueue[cmdDest - pa_CMD_TX_DEST_0 + NSS_PA_QUEUE_INPUT_INDEX], (Cppi_Desc *)ptrCmdPacket, 64);

    /* Wait for the result and process it accordingly by forwarding to the PA subsystem. */
    retVal = Netfp_processResponse(ptrNetfpServer, cmdReplyInfo.replyId, NULL);
    if (retVal < 0)
    {
        *errCode = retVal;
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to initialize the NETCP preclassification across the entire
 *      system.
 *
 *  @param[in]  serverHandle
 *      NETFP Server Handle
 *  @param[out] errCode
 *      Error code populated by the API
 *
 * \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -  <0
 */
int32_t Netfp_initPreClassification
(
    Netfp_ServerHandle  serverHandle,
    int32_t*            errCode
)
{
    paCmdReply_t             cmdReplyInfo;
    paReturn_t               retVal;
    Ti_Pkt*                  ptrCmdPacket;
    uint32_t                 dataBufferLen;
    uint8_t*                 ptrDataBuffer;
    int32_t                  cmdDest;
    uint16_t                 cmdSize;
    uint32_t                 psCmd = PASAHO_PACFG_CMD;
    paCtrlInfo_t             configInfo;
    paPacketControl2Config_t pktControl2;
    Netfp_ServerMCB*         ptrNetfpServer;

    /* Sanity Check: Validate the arguments: */
    if (serverHandle == NULL)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Get the pointer to the NETFP Server: */
    ptrNetfpServer = (Netfp_ServerMCB*)serverHandle;

    /* Initialize the packet control information: */
    memset ((void *)&pktControl2, 0, sizeof(paPacketControl2Config_t));

    /* Populate the packet control information to enable the enhanced QOS */
    pktControl2.validBitMap  = pa_PKT_CTRL2_VALID_EMAC_IF_INGRESS_DEFAULT_ROUTE;
    pktControl2.ctrlBitMap   = pa_PKT_CTRL_EMAC_IF_INGRESS_DEFAULT_ROUTE;

    /* Initialize the configuration information. */
    memset ((void *)&configInfo, 0, sizeof(paCtrlInfo_t));

    /* Populate the system configuration: */
    configInfo.code                       = pa_CONTROL_SYS_CONFIG;
    configInfo.params.sysCfg.pPktControl2 = &pktControl2;

    /* Initialize the command reply:
     *  - We want command replies to come to the Host in the PA Configuration
     *    Response queue and also use the Flow which we had configured. */
    memset ((void *)&cmdReplyInfo, 0, sizeof(paCmdReply_t));
    Netfp_fillPaCmdReply(&cmdReplyInfo, ptrNetfpServer);

    /* Allocate a packet for the configuration command. */
    ptrCmdPacket = Pktlib_allocPacket(ptrNetfpServer->cfg.pktlibInstHandle, ptrNetfpServer->cfg.cmdHeapHandle,
                                      pa_GLOBAL_CONFIG_MIN_CMD_BUF_SIZE_BYTES);
    if (ptrCmdPacket == NULL)
    {
        *errCode = NETFP_ENOMEM;
        return -1;
    }

    /* Get the data buffer from the allocated packet. */
    Pktlib_getDataBuffer(ptrCmdPacket, &ptrDataBuffer, &dataBufferLen);

    /* Setup the command size. */
    cmdSize = (uint16_t)dataBufferLen;

    /* Set the global config in the PA Susbystem. */
    retVal = Pa_control (ptrNetfpServer->paHandle,
                         &configInfo,
                         (paCmd_t)ptrDataBuffer,
                         &cmdSize,
                         &cmdReplyInfo,
                         &cmdDest);
    if (retVal != pa_OK)
    {
        /* Error: Configuration Failed. */
        *errCode = (int32_t)retVal;
        Pktlib_freePacket(ptrNetfpServer->cfg.pktlibInstHandle, ptrCmdPacket);
        return -1;
    }

    /* Set the packet & data buffer length. */
    Pktlib_setPacketLen(ptrCmdPacket, cmdSize);
    Pktlib_setDataBufferLen(ptrCmdPacket, cmdSize);

    /* Set the software information fields. This is required only so that the
     * PS information goes to the correct location. */
    Cppi_setSoftwareInfo0(Cppi_DescType_HOST, (Cppi_Desc *)ptrCmdPacket, 0x12345678);

    /* Mark the packet as a configuration packet */
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)ptrCmdPacket, (uint8_t *)&psCmd, 4);

    /* Ensure that the command packet is also written back */
    ptrNetfpServer->cfg.endMemAccess(ptrDataBuffer, dataBufferLen);

    /******************************************************************************
     * NOTE: There is an ownership change here from the DSP to the CPDMA. We need
     * to ensure that the packet contents here are written back.
     ******************************************************************************/
    Pktlib_releaseOwnership(ptrNetfpServer->cfg.pktlibInstHandle, ptrCmdPacket);

    /* Send the command to the PA. */
    Qmss_queuePushDescSize (ptrNetfpServer->netcpTxQueue[cmdDest - pa_CMD_TX_DEST_0 + NSS_PA_QUEUE_INPUT_INDEX], (Cppi_Desc *)ptrCmdPacket, 64);

    /* Wait for the result and process it accordingly by forwarding to the PA subsystem. */
    retVal = Netfp_processResponse(ptrNetfpServer, cmdReplyInfo.replyId, NULL);
    if (retVal < 0)
    {
        *errCode = retVal;
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to enable the NETCP preclassification for a specific interface.
 *      Interface based preclassification is only operational once the system wide preclassification
 *      has been enabled
 *
 *  @sa
 *      Netfp_initPreclassification
 *
 *  @param[in]  serverHandle
 *      NETFP Server Handle
 *  @param[in]  ptrPreClassificationCfg
 *      Pointer to the preclassification configuration
 *  @param[out] errCode
 *      Error code populated by the API
 *
 * \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -  <0
 */
int32_t Netfp_setupPreclassification
(
    Netfp_ServerHandle          serverHandle,
    Netfp_PreClassificationCfg* ptrPreClassificationCfg,
    int32_t*                    errCode
)
{
    paCmdReply_t        cmdReplyInfo;
    paReturn_t          retVal;
    Ti_Pkt*             ptrCmdPacket;
    uint32_t            dataBufferLen;
    uint8_t*            ptrDataBuffer;
    int32_t             cmdDest;
    uint16_t            cmdSize;
    uint32_t            psCmd = PASAHO_PACFG_CMD;
    paCtrlInfo_t        configInfo;
    Netfp_ServerMCB*    ptrNetfpServer;
    paDefRouteConfig_t  defaultRouteCfg;

    /* Sanity Check: Validate the arguments: */
    if ((serverHandle == NULL) || (ptrPreClassificationCfg == NULL))
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Get the pointer to the NETFP Server: */
    ptrNetfpServer = (Netfp_ServerMCB*)serverHandle;

    /* Initialize the default route configuration: */
    memset ((void *)&defaultRouteCfg, 0, sizeof(paDefRouteConfig_t));

    /* Setup the default port: */
    defaultRouteCfg.port = (uint8_t)ptrPreClassificationCfg->switchPortNum;

    /* Is Broadcast enabled? */
    if (ptrPreClassificationCfg->enableBroadcast == 1)
    {
        /* YES: Broadcast is enabled */
        defaultRouteCfg.ctrlBitMap |= pa_EMAC_IF_DEFAULT_ROUTE_BC_ENABLE                 |
                                      pa_EMAC_IF_DEFAULT_ROUTE_BC_PRE_CLASSIFY_ENABLE;

        /* Populate the default route information for broadcast packets. */
        defaultRouteCfg.dRouteInfo[pa_DROUTE_BROADCAST].validBitMap      = pa_ROUTE_INFO_VALID_PKTTYPE_EMAC;
        defaultRouteCfg.dRouteInfo[pa_DROUTE_BROADCAST].dest             = pa_DEST_HOST;
        defaultRouteCfg.dRouteInfo[pa_DROUTE_BROADCAST].flowId           = ptrPreClassificationCfg->broadcastFlowId;
        defaultRouteCfg.dRouteInfo[pa_DROUTE_BROADCAST].queue            = NETFP_CFG_BOUNCE_Q(ptrPreClassificationCfg->broadcastQueueId);
        defaultRouteCfg.dRouteInfo[pa_DROUTE_BROADCAST].mRouteIndex      = -1;
        defaultRouteCfg.dRouteInfo[pa_DROUTE_BROADCAST].swInfo0          = 0;
        defaultRouteCfg.dRouteInfo[pa_DROUTE_BROADCAST].swInfo1          = 0;
        defaultRouteCfg.dRouteInfo[pa_DROUTE_BROADCAST].customType       = 0;
        defaultRouteCfg.dRouteInfo[pa_DROUTE_BROADCAST].customIndex      = 0;
        defaultRouteCfg.dRouteInfo[pa_DROUTE_BROADCAST].pktType_emacCtrl = ptrPreClassificationCfg->switchPortNum;
        defaultRouteCfg.dRouteInfo[pa_DROUTE_BROADCAST].pCmd             = NULL;
        defaultRouteCfg.dRouteInfo[pa_DROUTE_BROADCAST].priorityType     = 0;
        defaultRouteCfg.dRouteInfo[pa_DROUTE_BROADCAST].efOpInfo         = NULL;
    }

    /* Is Multicast Enabled? */
    if (ptrPreClassificationCfg->enableMulticast == 1)
    {
        /* YES: Multicast is enabled */
        defaultRouteCfg.ctrlBitMap |= pa_EMAC_IF_DEFAULT_ROUTE_MC_ENABLE                 |
                                      pa_EMAC_IF_DEFAULT_ROUTE_MC_PRE_CLASSIFY_ENABLE;

        /* Populate the default route information for multicast packets. */
        defaultRouteCfg.dRouteInfo[pa_DROUTE_MULTICAST].validBitMap      = pa_ROUTE_INFO_VALID_PKTTYPE_EMAC;
        defaultRouteCfg.dRouteInfo[pa_DROUTE_MULTICAST].dest             = pa_DEST_HOST;
        defaultRouteCfg.dRouteInfo[pa_DROUTE_MULTICAST].flowId           = ptrPreClassificationCfg->multicastFlowId;
        defaultRouteCfg.dRouteInfo[pa_DROUTE_MULTICAST].queue            = NETFP_CFG_BOUNCE_Q(ptrPreClassificationCfg->multicastQueueId);
        defaultRouteCfg.dRouteInfo[pa_DROUTE_MULTICAST].mRouteIndex      = -1;
        defaultRouteCfg.dRouteInfo[pa_DROUTE_MULTICAST].swInfo0          = 0;
        defaultRouteCfg.dRouteInfo[pa_DROUTE_MULTICAST].swInfo1          = 0;
        defaultRouteCfg.dRouteInfo[pa_DROUTE_MULTICAST].customType       = 0;
        defaultRouteCfg.dRouteInfo[pa_DROUTE_MULTICAST].customIndex      = 0;
        defaultRouteCfg.dRouteInfo[pa_DROUTE_MULTICAST].pktType_emacCtrl = ptrPreClassificationCfg->switchPortNum;
        defaultRouteCfg.dRouteInfo[pa_DROUTE_MULTICAST].pCmd             = NULL;
        defaultRouteCfg.dRouteInfo[pa_DROUTE_MULTICAST].priorityType     = 0;
        defaultRouteCfg.dRouteInfo[pa_DROUTE_MULTICAST].efOpInfo         = NULL;
    }

    /* Initialize the configuration information. */
    memset ((void *)&configInfo, 0, sizeof(paCtrlInfo_t));

    /* Populate the control information */
    configInfo.code                             = pa_CONTROL_EMAC_PORT_CONFIG;
    configInfo.params.emacPortCfg.cfgType       = pa_EMAC_PORT_CFG_DEFAULT_ROUTE;
    configInfo.params.emacPortCfg.numEntries    = 1;
    configInfo.params.emacPortCfg.u.defRouteCfg = &defaultRouteCfg;

    /* Initialize the command reply:
     *  - We want command replies to come to the Host in the PA Configuration
     *    Response queue and also use the Flow which we had configured. */
    memset ((void *)&cmdReplyInfo, 0, sizeof(paCmdReply_t));
    Netfp_fillPaCmdReply(&cmdReplyInfo, ptrNetfpServer);

    /* Allocate a packet for the configuration command. */
    ptrCmdPacket = Pktlib_allocPacket(ptrNetfpServer->cfg.pktlibInstHandle, ptrNetfpServer->cfg.cmdHeapHandle,
                                      pa_EMAC_PORT_EQoS_MODE_CONFIG_MIN_CMD_BUF_SIZE_BYTES);
    if (ptrCmdPacket == NULL)
    {
        *errCode = NETFP_ENOMEM;
        return -1;
    }

    /* Get the data buffer from the allocated packet. */
    Pktlib_getDataBuffer(ptrCmdPacket, &ptrDataBuffer, &dataBufferLen);

    /* Setup the command size. */
    cmdSize = (uint16_t)dataBufferLen;

    /* Set the enhanced QOS configuration: */
    retVal = Pa_control (ptrNetfpServer->paHandle,
                         &configInfo,
                         (paCmd_t)ptrDataBuffer,
                         &cmdSize,
                         &cmdReplyInfo,
                         &cmdDest);
    if (retVal != pa_OK)
    {
        /* Error: Configuration Failed. */
        Pktlib_freePacket(ptrNetfpServer->cfg.pktlibInstHandle, ptrCmdPacket);
        *errCode = (int32_t)retVal;
        return -1;
    }

    /* Set the packet & data buffer length. */
    Pktlib_setPacketLen(ptrCmdPacket, cmdSize);
    Pktlib_setDataBufferLen(ptrCmdPacket, cmdSize);

    /* Set the software information fields. This is required only so that the
     * PS information goes to the correct location. */
    Cppi_setSoftwareInfo0(Cppi_DescType_HOST, (Cppi_Desc *)ptrCmdPacket, 0x12345678);

    /* Mark the packet as a configuration packet */
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)ptrCmdPacket, (uint8_t *)&psCmd, 4);

    /* Ensure that the command packet is also written back */
    ptrNetfpServer->cfg.endMemAccess(ptrDataBuffer, dataBufferLen);

    /******************************************************************************
     * NOTE: There is an ownership change here from the DSP to the CPDMA. We need
     * to ensure that the packet contents here are written back.
     ******************************************************************************/
    Pktlib_releaseOwnership(ptrNetfpServer->cfg.pktlibInstHandle, ptrCmdPacket);

    /* Send the command to the PA. */
    Qmss_queuePushDescSize (ptrNetfpServer->netcpTxQueue[cmdDest - pa_CMD_TX_DEST_0 + NSS_PA_QUEUE_INPUT_INDEX], (Cppi_Desc *)ptrCmdPacket, 64);

    /* Wait for the result and process it accordingly by forwarding to the PA subsystem. */
    retVal = Netfp_processResponse(ptrNetfpServer, cmdReplyInfo.replyId, NULL);
    if (retVal < 0)
    {
        *errCode = retVal;
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to create a MAC Virtual Link handle.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[out]  vlinkHandle
 *      Virtual Link Handle populated
 *  @param[out]  vlinkId
 *      Virtual Link Identifier matching the handle
 *  @param[out] errCode
 *      Error code populated by the API
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -  <0
 */
int32_t Netfp_createMACVirtualLink
(
    Netfp_ServerMCB*    ptrNetfpServer,
    Netfp_L2Handle*     vlinkHandle,
    int8_t*             vlinkId,
    int32_t*            errCode
)
{
    paReturn_t          retVal;
    paLnkHandle_t       paLinkHandle;

    /* Sanity Check: Validate the arguments */
    if (ptrNetfpServer == NULL)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Create the virtual link: */
    retVal = Pa_addVirtualLink (ptrNetfpServer->paHandle, &paLinkHandle, pa_VIRTUAL_LNK_TYPE_MAC);
    if (retVal != pa_OK)
    {
        *errCode = retVal;
        return -1;
    }
    Netfp_logMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "Debug: Virtual Link created %x\n", paLinkHandle);

    /* Get the virtual link identifier */
    retVal = Pa_getVirtualLinkId (ptrNetfpServer->paHandle, paLinkHandle, vlinkId);
    if (retVal != pa_OK)
    {
        *errCode = retVal;
        return -1;
    }
    Netfp_logMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "Debug: Virtual Link Id %x\n", *vlinkId);

    /* Populate the PA Virtual Link handle. */
    *vlinkHandle = paLinkHandle;
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to delete the MAC Virtual Link handle.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  vlinkHandle
 *      Virtual Link Handle to be deleted
 *  @param[out] errCode
 *      Error code populated by the API
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -  <0
 */
int32_t Netfp_deleteMACVirtualLink
(
    Netfp_ServerMCB*    ptrNetfpServer,
    Netfp_L2Handle      vlinkHandle,
    int32_t*            errCode
)
{
    paReturn_t      retVal;
    paLnkHandle_t   paLinkHandle;

    /* Sanity Check: Validate the arguments */
    if (ptrNetfpServer == NULL)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Sanity Check: Validate the arguments. */
    paLinkHandle = (paLnkHandle_t)vlinkHandle;
    if (paLinkHandle == NULL)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Create the virtual link: */
    retVal =  Pa_delVirtualLink	(ptrNetfpServer->paHandle, paLinkHandle);
    if (retVal != pa_OK)
    {
        *errCode = retVal;
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to initialize the port mirroring and capturing across the
 *      entire system.
 *
 *  @param[in]  netfpServerHandle
 *      NETFP Server Handle
 *  @param[out] errCode
 *      Error code populated by the API
 *
 * \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -  <0
 */
int32_t Netfp_initPortMirroringCapturing
(
    Netfp_ServerHandle      netfpServerHandle,
    int32_t*                errCode
)
{
    paCmdReply_t             cmdReplyInfo;
    paReturn_t               retVal;
    Ti_Pkt*                  ptrCmdPacket;
    uint32_t                 dataBufferLen;
    uint8_t*                 ptrDataBuffer;
    int32_t                  cmdDest;
    uint16_t                 cmdSize;
    uint32_t                 psCmd = PASAHO_PACFG_CMD;
    paCtrlInfo_t             configInfo;
    paPacketControl2Config_t pktControl2;
    Netfp_ServerMCB*         ptrNetfpServer;

    /* Sanity Check: Validate the arguments: */
    if (netfpServerHandle == NULL)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Get the pointer to the NETFP Server: */
    ptrNetfpServer = (Netfp_ServerMCB*)netfpServerHandle;

    /* Initialize the packet control information: */
    memset ((void *)&pktControl2, 0, sizeof(paPacketControl2Config_t));

    /* Populate the packet control information to enable the enhanced QOS */
    pktControl2.validBitMap  = pa_PKT_CTRL2_VALID_EMAC_IF_IGRESS_CLONE | pa_PKT_CTRL2_VALID_EMAC_IF_EGRESS_CLONE;
    pktControl2.ctrlBitMap   = pa_PKT_CTRL_EMAC_IF_IGRESS_CLONE | pa_PKT_CTRL_EMAC_IF_EGRESS_CLONE;

    /* Initialize the configuration information. */
    memset ((void *)&configInfo, 0, sizeof(paCtrlInfo_t));

    /* Populate the system configuration: */
    configInfo.code                       = pa_CONTROL_SYS_CONFIG;
    configInfo.params.sysCfg.pPktControl2 = &pktControl2;

    /* Initialize the command reply:
     *  - We want command replies to come to the Host in the PA Configuration
     *    Response queue and also use the Flow which we had configured. */
    memset ((void *)&cmdReplyInfo, 0, sizeof(paCmdReply_t));
    Netfp_fillPaCmdReply(&cmdReplyInfo, ptrNetfpServer);

    /* Allocate a packet for the configuration command. */
    ptrCmdPacket = Pktlib_allocPacket(ptrNetfpServer->cfg.pktlibInstHandle, ptrNetfpServer->cfg.cmdHeapHandle,
                                      pa_GLOBAL_CONFIG_MIN_CMD_BUF_SIZE_BYTES);
    if (ptrCmdPacket == NULL)
    {
        *errCode = NETFP_ENOMEM;
        return -1;
    }

    /* Get the data buffer from the allocated packet. */
    Pktlib_getDataBuffer(ptrCmdPacket, &ptrDataBuffer, &dataBufferLen);

    /* Setup the command size. */
    cmdSize = (uint16_t)dataBufferLen;

    /* Set the global config in the PA Susbystem. */
    retVal = Pa_control (ptrNetfpServer->paHandle,
                         &configInfo,
                         (paCmd_t)ptrDataBuffer,
                         &cmdSize,
                         &cmdReplyInfo,
                         &cmdDest);
    if (retVal != pa_OK)
    {
        /* Error: Configuration Failed. */
        *errCode = (int32_t)retVal;
        Pktlib_freePacket(ptrNetfpServer->cfg.pktlibInstHandle, ptrCmdPacket);
        return -1;
    }

    /* Set the packet & data buffer length. */
    Pktlib_setPacketLen(ptrCmdPacket, cmdSize);
    Pktlib_setDataBufferLen(ptrCmdPacket, cmdSize);

    /* Set the software information fields. This is required only so that the
     * PS information goes to the correct location. */
    Cppi_setSoftwareInfo0(Cppi_DescType_HOST, (Cppi_Desc *)ptrCmdPacket, 0x12345678);

    /* Mark the packet as a configuration packet */
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)ptrCmdPacket, (uint8_t *)&psCmd, 4);

    /* Ensure that the command packet is also written back */
    ptrNetfpServer->cfg.endMemAccess(ptrDataBuffer, dataBufferLen);

    /******************************************************************************
     * NOTE: There is an ownership change here from the DSP to the CPDMA. We need
     * to ensure that the packet contents here are written back.
     ******************************************************************************/
    Pktlib_releaseOwnership(ptrNetfpServer->cfg.pktlibInstHandle, ptrCmdPacket);

    /* Send the command to the PA. */
    Qmss_queuePushDescSize (ptrNetfpServer->netcpTxQueue[cmdDest - pa_CMD_TX_DEST_0 + NSS_PA_QUEUE_INPUT_INDEX], (Cppi_Desc *)ptrCmdPacket, 64);

    /* Wait for the result and process it accordingly by forwarding to the PA subsystem. */
    retVal = Netfp_processResponse(ptrNetfpServer, cmdReplyInfo.replyId, NULL);
    if (retVal < 0)
    {
        *errCode = retVal;
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to setup the port mirroring. Port mirroring can only be used
 *      once the functionality has been initialized across the entire system.
 *
 *  @sa
 *      Netfp_initPortMirroringCapturing
 *
 *  @param[in]  serverHandle
 *      NETFP Server Handle
 *  @param[in]  ptrPortMirrorCfg
 *      Pointer to the port mirror configuration
 *  @param[out] errCode
 *      Error code populated by the API
 *
 * \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -  <0
 */
int32_t Netfp_setupPortMirror
(
    Netfp_ServerHandle      serverHandle,
    Netfp_PortMirrorCfg*    ptrPortMirrorCfg,
    int32_t*                errCode
)
{
    paCmdReply_t            cmdReplyInfo;
    paReturn_t              retVal;
    Ti_Pkt*                 ptrCmdPacket;
    uint32_t                dataBufferLen;
    uint8_t*                ptrDataBuffer;
    int32_t                 cmdDest;
    uint16_t                cmdSize;
    uint32_t                psCmd = PASAHO_PACFG_CMD;
    paCtrlInfo_t            configInfo;
    Netfp_ServerMCB*        ptrNetfpServer;
    paPortMirrorConfig_t    portMirrorCfg;

    /* Sanity Check: Validate the arguments: */
    if ((serverHandle == NULL) || (ptrPortMirrorCfg == NULL))
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Get the pointer to the NETFP Server: */
    ptrNetfpServer = (Netfp_ServerMCB*)serverHandle;

    /* Initialize the port mirror configuration: */
    memset ((void *)&portMirrorCfg, 0, sizeof(paPortMirrorConfig_t));

    /* Sanity Check: Ensure that the source & destination port are not the same */
    if (ptrPortMirrorCfg->dstPort == ptrPortMirrorCfg->srcPort)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Do we need to enable port capturing? */
    if (ptrPortMirrorCfg->isEnable == 1)
    {
        /* YES. Do we need to mirror all incoming packets? */
        if (ptrPortMirrorCfg->direction == Netfp_Direction_INBOUND)
        {
            /* YES. Setup the ingress direction */
            portMirrorCfg.ctrlBitMap = pa_PKT_CLONE_ENABLE | pa_PKT_CLONE_INGRESS;
        }
        else
        {
            /* NO. Only egress packets. */
            portMirrorCfg.ctrlBitMap = pa_PKT_CLONE_ENABLE | ~pa_PKT_CLONE_INGRESS;
        }
    }
    else
    {
        /* NO. Do we need to stop the mirror for all incoming packets? */
        if (ptrPortMirrorCfg->direction == Netfp_Direction_INBOUND)
        {
            /* YES. Setup the ingress direction */
            portMirrorCfg.ctrlBitMap = pa_PKT_CLONE_INGRESS;
        }
        else
        {
            /* NO. Only egress packets. */
            portMirrorCfg.ctrlBitMap = 0;
        }
    }

    /* Setup the port information: */
    portMirrorCfg.portToBeMirrored = ptrPortMirrorCfg->srcPort;
    portMirrorCfg.mirrorPort       = ptrPortMirrorCfg->dstPort;

    /* Initialize the configuration information. */
    memset ((void *)&configInfo, 0, sizeof(paCtrlInfo_t));

    /* Populate the control information */
    configInfo.code                             = pa_CONTROL_EMAC_PORT_CONFIG;
    configInfo.params.emacPortCfg.cfgType       = pa_EMAC_PORT_CFG_MIRROR;
    configInfo.params.emacPortCfg.numEntries    = 1;
    configInfo.params.emacPortCfg.u.mirrorCfg   = &portMirrorCfg;

    /* Initialize the command reply:
     *  - We want command replies to come to the Host in the PA Configuration
     *    Response queue and also use the Flow which we had configured. */
    memset ((void *)&cmdReplyInfo, 0, sizeof(paCmdReply_t));
    Netfp_fillPaCmdReply(&cmdReplyInfo, ptrNetfpServer);

    /* Allocate a packet for the configuration command. */
    ptrCmdPacket = Pktlib_allocPacket(ptrNetfpServer->cfg.pktlibInstHandle, ptrNetfpServer->cfg.cmdHeapHandle,
                                      pa_EMAC_PORT_EQoS_MODE_CONFIG_MIN_CMD_BUF_SIZE_BYTES);
    if (ptrCmdPacket == NULL)
    {
        *errCode = NETFP_ENOMEM;
        return -1;
    }

    /* Get the data buffer from the allocated packet. */
    Pktlib_getDataBuffer(ptrCmdPacket, &ptrDataBuffer, &dataBufferLen);

    /* Setup the command size. */
    cmdSize = (uint16_t)dataBufferLen;

    /* Set the enhanced QOS configuration: */
    retVal = Pa_control (ptrNetfpServer->paHandle,
                         &configInfo,
                         (paCmd_t)ptrDataBuffer,
                         &cmdSize,
                         &cmdReplyInfo,
                         &cmdDest);
    if (retVal != pa_OK)
    {
        /* Error: Configuration Failed. */
        Pktlib_freePacket(ptrNetfpServer->cfg.pktlibInstHandle, ptrCmdPacket);
        *errCode = (int32_t)retVal;
        return -1;
    }

    /* Set the packet & data buffer length. */
    Pktlib_setPacketLen(ptrCmdPacket, cmdSize);
    Pktlib_setDataBufferLen(ptrCmdPacket, cmdSize);

    /* Set the software information fields. This is required only so that the
     * PS information goes to the correct location. */
    Cppi_setSoftwareInfo0(Cppi_DescType_HOST, (Cppi_Desc *)ptrCmdPacket, 0x12345678);

    /* Mark the packet as a configuration packet */
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)ptrCmdPacket, (uint8_t *)&psCmd, 4);

    /* Ensure that the command packet is also written back */
    ptrNetfpServer->cfg.endMemAccess(ptrDataBuffer, dataBufferLen);

    /******************************************************************************
     * NOTE: There is an ownership change here from the DSP to the CPDMA. We need
     * to ensure that the packet contents here are written back.
     ******************************************************************************/
    Pktlib_releaseOwnership(ptrNetfpServer->cfg.pktlibInstHandle, ptrCmdPacket);

    /* Send the command to the PA. */
    Qmss_queuePushDescSize (ptrNetfpServer->netcpTxQueue[cmdDest - pa_CMD_TX_DEST_0 + NSS_PA_QUEUE_INPUT_INDEX], (Cppi_Desc *)ptrCmdPacket, 64);

    /* Wait for the result and process it accordingly by forwarding to the PA subsystem. */
    retVal = Netfp_processResponse(ptrNetfpServer, cmdReplyInfo.replyId, NULL);
    if (retVal < 0)
    {
        *errCode = retVal;
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to setup the port capturing. Port capturing can only be used
 *      once the functionality has been initialized across the entire system.
 *
 *  @sa
 *      Netfp_initPortMirroringCapturing
 *
 *  @param[in]  serverHandle
 *      NETFP Server Handle
 *  @param[in]  ptrPortCaptureCfg
 *      Pointer to the port capture configuration
 *  @param[out] errCode
 *      Error code populated by the API
 *
 * \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -  <0
 */
int32_t Netfp_setupPortCapture
(
    Netfp_ServerHandle      serverHandle,
    Netfp_PortCaptureCfg*   ptrPortCaptureCfg,
    int32_t*                errCode
)
{
    paCmdReply_t            cmdReplyInfo;
    paReturn_t              retVal;
    Ti_Pkt*                 ptrCmdPacket;
    uint32_t                dataBufferLen;
    uint8_t*                ptrDataBuffer;
    int32_t                 cmdDest;
    uint16_t                cmdSize;
    uint32_t                psCmd = PASAHO_PACFG_CMD;
    paCtrlInfo_t            configInfo;
    Netfp_ServerMCB*        ptrNetfpServer;
    paPktCaptureConfig_t    portCaptureCfg;

    /* Sanity Check: Validate the arguments: */
    if ((serverHandle == NULL) || (ptrPortCaptureCfg == NULL))
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Get the pointer to the NETFP Server: */
    ptrNetfpServer = (Netfp_ServerMCB*)serverHandle;

    /* Initialize the port capture configuration: */
    memset ((void *)&portCaptureCfg, 0, sizeof(paPktCaptureConfig_t));

    /* Do we need to enable port capturing? */
    if (ptrPortCaptureCfg->isEnable == 1)
    {
        /* YES. Do we need to capture all incoming packets? */
        if (ptrPortCaptureCfg->direction == Netfp_Direction_INBOUND)
        {
            /* YES. Setup the ingress direction */
            portCaptureCfg.ctrlBitMap = pa_PKT_CLONE_ENABLE | pa_PKT_CLONE_INGRESS;
        }
        else
        {
            /* NO. Only egress packets. */
            portCaptureCfg.ctrlBitMap = pa_PKT_CLONE_ENABLE;
        }
    }
    else
    {
        /* NO. Do we need to stop the capture for all incoming packets? */
        if (ptrPortCaptureCfg->direction == Netfp_Direction_INBOUND)
        {
            /* YES. Setup the ingress direction */
            portCaptureCfg.ctrlBitMap = pa_PKT_CLONE_INGRESS;
        }
        else
        {
            /* NO. Only egress packets. */
            portCaptureCfg.ctrlBitMap = 0;
        }
    }

    /* Setup the port information: */
    portCaptureCfg.portToBeCaptured = ptrPortCaptureCfg->portToBeCaptured;
    portCaptureCfg.flowId           = ptrPortCaptureCfg->flowId;
    portCaptureCfg.queue            = NETFP_CFG_BOUNCE_Q(ptrPortCaptureCfg->queueId);
    portCaptureCfg.swInfo0          = ptrPortCaptureCfg->swInfo;

    /* Initialize the configuration information. */
    memset ((void *)&configInfo, 0, sizeof(paCtrlInfo_t));

    /* Populate the control information */
    configInfo.code                             = pa_CONTROL_EMAC_PORT_CONFIG;
    configInfo.params.emacPortCfg.cfgType       = pa_EMAC_PORT_CFG_PKT_CAPTURE;
    configInfo.params.emacPortCfg.numEntries    = 1;
    configInfo.params.emacPortCfg.u.pktCapCfg   = &portCaptureCfg;

    /* Initialize the command reply:
     *  - We want command replies to come to the Host in the PA Configuration
     *    Response queue and also use the Flow which we had configured. */
    memset ((void *)&cmdReplyInfo, 0, sizeof(paCmdReply_t));
    Netfp_fillPaCmdReply(&cmdReplyInfo, ptrNetfpServer);

    /* Allocate a packet for the configuration command. */
    ptrCmdPacket = Pktlib_allocPacket(ptrNetfpServer->cfg.pktlibInstHandle, ptrNetfpServer->cfg.cmdHeapHandle,
                                      pa_EMAC_PORT_EQoS_MODE_CONFIG_MIN_CMD_BUF_SIZE_BYTES);
    if (ptrCmdPacket == NULL)
    {
        *errCode = NETFP_ENOMEM;
        return -1;
    }

    /* Get the data buffer from the allocated packet. */
    Pktlib_getDataBuffer(ptrCmdPacket, &ptrDataBuffer, &dataBufferLen);

    /* Setup the command size. */
    cmdSize = (uint16_t)dataBufferLen;

    /* Set the enhanced QOS configuration: */
    retVal = Pa_control (ptrNetfpServer->paHandle,
                         &configInfo,
                         (paCmd_t)ptrDataBuffer,
                         &cmdSize,
                         &cmdReplyInfo,
                         &cmdDest);
    if (retVal != pa_OK)
    {
        /* Error: Configuration Failed. */
        Pktlib_freePacket(ptrNetfpServer->cfg.pktlibInstHandle, ptrCmdPacket);
        *errCode = (int32_t)retVal;
        return -1;
    }

    /* Set the packet & data buffer length. */
    Pktlib_setPacketLen(ptrCmdPacket, cmdSize);
    Pktlib_setDataBufferLen(ptrCmdPacket, cmdSize);

    /* Set the software information fields. This is required only so that the
     * PS information goes to the correct location. */
    Cppi_setSoftwareInfo0(Cppi_DescType_HOST, (Cppi_Desc *)ptrCmdPacket, 0x12345678);

    /* Mark the packet as a configuration packet */
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)ptrCmdPacket, (uint8_t *)&psCmd, 4);

    /* Ensure that the command packet is also written back */
    ptrNetfpServer->cfg.endMemAccess(ptrDataBuffer, dataBufferLen);

    /******************************************************************************
     * NOTE: There is an ownership change here from the DSP to the CPDMA. We need
     * to ensure that the packet contents here are written back.
     ******************************************************************************/
    Pktlib_releaseOwnership(ptrNetfpServer->cfg.pktlibInstHandle, ptrCmdPacket);

    /* Send the command to the PA. */
    Qmss_queuePushDescSize (ptrNetfpServer->netcpTxQueue[cmdDest - pa_CMD_TX_DEST_0 + NSS_PA_QUEUE_INPUT_INDEX], (Cppi_Desc *)ptrCmdPacket, 64);

    /* Wait for the result and process it accordingly by forwarding to the PA subsystem. */
    retVal = Netfp_processResponse(ptrNetfpServer, cmdReplyInfo.replyId, NULL);
    if (retVal < 0)
    {
        *errCode = retVal;
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function opens the PA driver and initializes the PA subystem.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  maxL2Handles
 *      Maximum number of L2 Handles which can exist in the system.
 *  @param[in]  maxL3Handles
 *      Maximum number of L3 Handles which can exist in the system.
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success - Handle to the PA Subsystem
 *  @retval
 *      Error   - NULL
 */
Pa_Handle Netfp_paInit
(
    Netfp_ServerMCB*    ptrNetfpServer,
    uint32_t            maxL2Handles,
    uint32_t            maxL3Handles
)
{
    uint8_t         index;
    paSizeInfo_t    paSize;
    paConfig_t      paCfg;
    paStartCfg_t    startCfg;
    int32_t         sizes[pa_N_BUFS];
    int32_t         aligns[pa_N_BUFS];
    void*           bases[pa_N_BUFS];
    Pa_Handle       paHandle;

    /* Initialize the size & alignment arrays */
    memset ((void *)&sizes[0], 0,  sizeof(sizes));
    memset ((void *)&aligns[0], 0, sizeof(aligns));

#if (defined (DEVICE_K2H) || defined (DEVICE_K2K))
    /* Allocate space for the PA LLD buffers. The buffers we need to
     * allocate space are:
     *      (1) PA LLD Instance Info Handle
     *      (2) PA LLD L2 Handle database
     *      (3) PA LLD L3 Handle database */
    paSize.nMaxL2     = maxL2Handles;
    paSize.nMaxL3     = maxL3Handles;
    paSize.nUsrStats  = pa_USR_STATS_MAX_COUNTERS;
    paSize.nMaxVlnk   = 3 * pa_MAX_NUM_LUT1_ENTRIES;
    if (Pa_getBufferReq(&paSize, sizes, aligns) != pa_OK)
        return NULL;
#elif defined (DEVICE_K2L)
    /* Allocate space for the PA LLD buffers. The buffers we need to
     * allocate space are:
     *      (1) PA LLD Instance Info Handle
     *      (2) PA LLD L2 Handle database
     *      (3) PA LLD L3 Handle database */
    paSize.nMaxL2     = maxL2Handles;
    paSize.nMaxL3     = maxL3Handles;
    paSize.nUsrStats  = pa_USR_STATS_MAX_COUNTERS;
    paSize.nMaxVlnk   = 3 * pa_MAX_NUM_LUT1_ENTRIES;
    paSize.nMaxAcl    = 512; /* TODO: parameterize this */
    paSize.nMaxFc     = 256;
    paSize.nMaxEoam   = 0;   /* EOAM is not supported */
    if (Pa_getBufferReq(&paSize, sizes, aligns) != pa_OK)
        return NULL;
#else
#error "Unsupported Device"
#endif

    /* Allocate the requested memory for all the blocks. */
    for (index = 0; index < pa_N_BUFS; index++)
    {
        /* Allocate memory to meet the size and alignment requirements. */
        if (sizes[index] != 0)
        {
            bases[index] = (void *)ptrNetfpServer->cfg.malloc(sizes[index], aligns[index]);
            if (bases[index] == NULL)
                return NULL;
        }
    }

    /* Initialize the PA configuration. */
    memset ((void *)&paCfg, 0, sizeof(paConfig_t));

    /* Setup the PA configuration accordingly. */
    paCfg.initTable        = TRUE;
    paCfg.initDefaultRoute = FALSE;
    paCfg.sizeCfg          = &paSize;

    /* Setup the PA base address correctly. Use the virtual address if specified. Else simply use
     * the default physical address. */
    if (ptrNetfpServer->cfg.passCfgVirtualAddress == 0)
        paCfg.baseAddr  = CSL_NETCP_CFG_REGS;
    else
        paCfg.baseAddr  = ptrNetfpServer->cfg.passCfgVirtualAddress;

    /* Create the PA LLD driver instance. */
    if (Pa_create (&paCfg, bases, &paHandle) != pa_OK)
        return NULL;

    /* Initialize the configuration. */
    memset ((void *)&startCfg, 0, sizeof(paStartCfg_t));

    /* Register the PA LLD to use the RMv2 services. */
    startCfg.rmServiceHandle = ptrNetfpServer->cfg.rmServiceHandle;
    startCfg.baseAddr        = paCfg.baseAddr;
    Pa_startCfg (paHandle, &startCfg);

    return paHandle;
}

/**
 *  @b Description
 *  @n
 *      The function is used to initialize the PA command handler. All requests
 *      & responses to the PA subsystem are handler by this internal block.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  heapHandle
 *      The heap handle which will be used to send commands to the PA subsystem
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_paInitCmdHandler
(
    Netfp_ServerMCB*    ptrNetfpServer,
    Pktlib_HeapHandle   heapHandle
)
{
    uint8_t             isAllocated;
    Cppi_RxFlowCfg      rxFlowCfg;
    Qmss_Queue          rxFreeQInfo;
    Qmss_Queue          rxQInfo;
    uint32_t            index;
    Cppi_TxChInitCfg    txChCfg;
    uint32_t            passBaseQueue = QMSS_PASS_QUEUE_BASE;

    /* Open a PA Command Response Queue: This queue will be used to hold responses from
     * the PA PDSP for all the commands issued by NETFP. */
    ptrNetfpServer->paCfgRespQueue = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE,
                                                     QMSS_PARAM_NOT_SPECIFIED,
                                                     &isAllocated);
    if (ptrNetfpServer->paCfgRespQueue < 0)
        return -1;

    /* We need to now created the flow which will be used by the PA to send the configuration
     * responses back to the HOST. We want the flow to use the free descriptors from the internal
     * heap queue and place the received packets into the PA Configuration response queue. */
    rxFreeQInfo = Qmss_getQueueNumber (Pktlib_getInternalHeapQueue(heapHandle));
    rxQInfo     = Qmss_getQueueNumber (ptrNetfpServer->paCfgRespQueue);

    /* Create the Flow. Initialize the flow configuration. */
    memset (&rxFlowCfg, 0, sizeof(Cppi_RxFlowCfg));

    /* CPPI pick the next available flow */
    rxFlowCfg.flowIdNum             =   CPPI_PARAM_NOT_SPECIFIED;
    rxFlowCfg.rx_dest_qmgr          =   rxQInfo.qMgr;
    rxFlowCfg.rx_dest_qnum          =   Qmss_getQIDFromHandle(ptrNetfpServer->paCfgRespQueue);
    rxFlowCfg.rx_sop_offset         =   0;
    rxFlowCfg.rx_desc_type          =   Cppi_DescType_HOST;
    rxFlowCfg.rx_ps_location        =   Cppi_PSLoc_PS_IN_DESC;
    rxFlowCfg.rx_psinfo_present     =   1;    /* Enable PS info */
    rxFlowCfg.rx_error_handling     =   0;    /* Drop the packet, do not retry on starvation by default */
    rxFlowCfg.rx_einfo_present      =   1;    /* EPIB info present */
    rxFlowCfg.rx_dest_tag_lo_sel    =   0;    /* Disable tagging */
    rxFlowCfg.rx_dest_tag_hi_sel    =   0;
    rxFlowCfg.rx_src_tag_lo_sel     =   0;
    rxFlowCfg.rx_src_tag_hi_sel     =   0;
    rxFlowCfg.rx_size_thresh0_en    =   0;    /* By default, we disable Rx Thresholds */
    rxFlowCfg.rx_size_thresh1_en    =   0;    /* By default, we disable Rx Thresholds */
    rxFlowCfg.rx_size_thresh2_en    =   0;    /* By default, we disable Rx Thresholds */
    rxFlowCfg.rx_size_thresh0       =   0x0;
    rxFlowCfg.rx_size_thresh1       =   0x0;
    rxFlowCfg.rx_size_thresh2       =   0x0;
    rxFlowCfg.rx_fdq0_sz0_qmgr      =   rxFreeQInfo.qMgr; /* Setup the Receive free queue for the flow */
    rxFlowCfg.rx_fdq0_sz0_qnum      =   Qmss_getQIDFromHandle(Pktlib_getInternalHeapQueue(heapHandle));
    rxFlowCfg.rx_fdq0_sz1_qnum      =   0x0;
    rxFlowCfg.rx_fdq0_sz1_qmgr      =   0x0;
    rxFlowCfg.rx_fdq0_sz2_qnum      =   0x0;
    rxFlowCfg.rx_fdq0_sz2_qmgr      =   0x0;
    rxFlowCfg.rx_fdq0_sz3_qnum      =   0x0;
    rxFlowCfg.rx_fdq0_sz3_qmgr      =   0x0;
    rxFlowCfg.rx_fdq1_qnum          =   Qmss_getQIDFromHandle(Pktlib_getInternalHeapQueue(heapHandle));
    rxFlowCfg.rx_fdq1_qmgr          =   rxFreeQInfo.qMgr;
    rxFlowCfg.rx_fdq2_qnum          =   Qmss_getQIDFromHandle(Pktlib_getInternalHeapQueue(heapHandle));
    rxFlowCfg.rx_fdq2_qmgr          =   rxFreeQInfo.qMgr;
    rxFlowCfg.rx_fdq3_qnum          =   Qmss_getQIDFromHandle(Pktlib_getInternalHeapQueue(heapHandle));
    rxFlowCfg.rx_fdq3_qmgr          =   rxFreeQInfo.qMgr;

    /* Configure the Rx flow */
    ptrNetfpServer->paRxFlowHandle = Cppi_configureRxFlow (ptrNetfpServer->passCPDMAHandle,
                                                           &rxFlowCfg, &isAllocated);
    if (ptrNetfpServer->paRxFlowHandle == NULL)
        return -1;

    /* Initialize the transmit channel configuration: */
    memset ((void *)&txChCfg, 0, sizeof (Cppi_TxChInitCfg));

    /* Populate the channel configuration: */
    txChCfg.channelNum = NETFP_CPPI_CIPHER_TX_CHANNEL;
    txChCfg.txEnable   = Cppi_ChState_CHANNEL_ENABLE;

    /* Open the ciphering channel. We need access to the channel during LTE reestablishment/Handover. */
    ptrNetfpServer->cppiCipherTxChHnd = Cppi_txChannelOpen (ptrNetfpServer->passCPDMAHandle, &txChCfg, &isAllocated);
    if (ptrNetfpServer->cppiCipherTxChHnd == NULL)
        return -1;

#if 0
    /* Cycle through and open all the NETCP Transmit queues.
     *  - The NETFP Library requires access to all the queues. */
    for (index = 0; index < QMSS_MAX_PASS_QUEUE; index++)
    {
        ptrNetfpServer->netcpTxQueue[index] = Qmss_queueOpen (Qmss_QueueType_PASS_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated);
        if (ptrNetfpServer->netcpTxQueue[index] < 0)
        {
            printf ("Error: Unable to open the PASS Queue\n");
            return -1;
        }
    }
#else
    /* Open all the NETCP Transmit queues. QMSS RM does not allow opening multiple transmit queues */
    for (index = 0; index < QMSS_MAX_PASS_QUEUE; index++)
        ptrNetfpServer->netcpTxQueue[index] = (Qmss_QueueHnd)(passBaseQueue + index);
#endif
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to configure the exception routes in the NETCP subsystem
 *      for the GTPU Control messages.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  ptrGTPUControlCfg
 *      Pointer to the GTPU Control message configuration
 *  @param[out] errCode
 *      Error code populated on error
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t _Netfp_configureGTPUControlMessage
(
    Netfp_ServerMCB*        ptrNetfpServer,
    Netfp_GTPUControlCfg*   ptrGTPUControlCfg,
    int32_t*                errCode
)
{
    int                 eRouteTypes[NETFP_MAX_GTPU_EROUTES];
    paRouteInfo2_t      eRoutes[NETFP_MAX_GTPU_EROUTES];
    paCmdReply_t        cmdReplyInfo;
    paReturn_t          retVal;
    Ti_Pkt*             ptrCmdPacket;
    uint32_t            dataBufferLen, index;
    uint8_t*            ptrDataBuffer;
    int32_t             cmdDest;
    uint16_t            cmdSize;
    uint32_t            psCmd = PASAHO_PACFG_CMD;
    uint32_t            eRoutesNum = 0;

    /* Initialize the exception route configuration. */
    memset ((void *)eRoutes, 0, sizeof (eRoutes));

    for (index = 0; index < 7; index++)
    {
        eRoutes[index].dest         = pa_DEST_HOST;
        eRoutes[index].flowId       = ptrGTPUControlCfg->gtpuMsgFlowId;
        eRoutes[index].mRouteIndex  = -1;
    }
    eRoutes[eRoutesNum].queue = NETFP_CFG_BOUNCE_Q(ptrGTPUControlCfg->gtpuPingReqQueueHnd);
    eRouteTypes[eRoutesNum++] = pa_EROUTE_GTPU_MESSAGE_TYPE_1;
    eRoutes[eRoutesNum].queue = NETFP_CFG_BOUNCE_Q(ptrGTPUControlCfg->gtpuPingRespQueueHnd);
    eRouteTypes[eRoutesNum++] = pa_EROUTE_GTPU_MESSAGE_TYPE_2;
    eRoutes[eRoutesNum].queue = NETFP_CFG_BOUNCE_Q(ptrGTPUControlCfg->gtpuErrorIndQueueHnd);
    eRouteTypes[eRoutesNum++] = pa_EROUTE_GTPU_MESSAGE_TYPE_26;
    eRoutes[eRoutesNum].queue = NETFP_CFG_BOUNCE_Q(ptrGTPUControlCfg->gtpuHdrNotifyQueueHnd);
    eRouteTypes[eRoutesNum++] = pa_EROUTE_GTPU_MESSAGE_TYPE_31;
    eRoutes[eRoutesNum].queue = NETFP_CFG_BOUNCE_Q(ptrGTPUControlCfg->gtpuEndMarkerQueueHnd);
    eRouteTypes[eRoutesNum++] = pa_EROUTE_GTPU_MESSAGE_TYPE_254;
    eRoutes[eRoutesNum].queue = NETFP_CFG_BOUNCE_Q(ptrGTPUControlCfg->gtpuErrorQueueHnd);
    eRouteTypes[eRoutesNum++] = pa_EROUTE_GTPU_FAIL;
    eRoutes[eRoutesNum].queue = NETFP_CFG_BOUNCE_Q(ptrGTPUControlCfg->gtpuIdMatchFailQueueHnd);
    eRouteTypes[eRoutesNum++] = pa_EROUTE_GTPU_MATCH_FAIL;

    /* Initialize the command reply:
     *  - We want command replies to come to the Host in the PA Configuration
     *    Response queue and also use the Flow which we had configured. */
    memset ((void *)&cmdReplyInfo, 0, sizeof(paCmdReply_t));
    Netfp_fillPaCmdReply(&cmdReplyInfo, ptrNetfpServer);

    /* Allocate a packet for the configuration command. */
    ptrCmdPacket = Pktlib_allocPacket(ptrNetfpServer->cfg.pktlibInstHandle, ptrNetfpServer->cfg.cmdHeapHandle,
                                      pa_CONFIG_EXCEPTION_ROUTE_MIN_CMD_BUF_SIZE_BYTES);
    if (ptrCmdPacket == NULL)
        return -1;

    /* Get the data buffer from the allocated packet. */
    Pktlib_getDataBuffer(ptrCmdPacket, &ptrDataBuffer, &dataBufferLen);

    /* Setup the command size. */
    cmdSize = (uint16_t)dataBufferLen;

    /* Configure the exception routes. */
    retVal = Pa_configExceptionRoute2 (ptrNetfpServer->paHandle, eRoutesNum,
                                      &eRouteTypes[0], &eRoutes[0], ptrDataBuffer,
                                      &cmdSize, &cmdReplyInfo, &cmdDest);
    if (retVal != pa_OK)
    {
        /* Error: Configuration Failed. */
        Pktlib_freePacket(ptrNetfpServer->cfg.pktlibInstHandle, ptrCmdPacket);
        return retVal;
    }

    /* Set the packet & data buffer length. */
    Pktlib_setPacketLen(ptrCmdPacket, cmdSize);
    Pktlib_setDataBufferLen(ptrCmdPacket, cmdSize);

    /* Set the software information fields. This is required only so that the
     * PS information goes to the correct location. */
    Cppi_setSoftwareInfo0(Cppi_DescType_HOST, (Cppi_Desc *)ptrCmdPacket, 0x12345678);

    /* Mark the packet as a configuration packet */
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)ptrCmdPacket, (uint8_t *)&psCmd, 4);

    /* Ensure that the command packet is also written back */
    ptrNetfpServer->cfg.endMemAccess(ptrDataBuffer, dataBufferLen);

    /******************************************************************************
     * NOTE: There is an ownership change here from the DSP to the CPDMA. We need
     * to ensure that the packet contents here are written back.
     ******************************************************************************/
    Pktlib_releaseOwnership(ptrNetfpServer->cfg.pktlibInstHandle, ptrCmdPacket);

    /* Send the command to the PA. */
    Qmss_queuePushDescSize (ptrNetfpServer->netcpTxQueue[cmdDest - pa_CMD_TX_DEST_0 + NSS_PA_QUEUE_INPUT_INDEX], (Cppi_Desc *)ptrCmdPacket, 64);

    /* Wait for the result and process it accordingly by forwarding to the PA subsystem. */
    retVal = Netfp_processResponse(ptrNetfpServer, cmdReplyInfo.replyId, NULL);
    if (retVal < 0)
        return retVal;

    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to initialized the interface based routing info.
 *  These information will be used for Fail route creation.
 *
 *  @param[in]  serverHandle
 *      NETFP Server Handle
 *  @param[out] baseFlowId
 *      Physical Ethernet interface base Flow Identifier
 *  @param[out] baseQueue
 *      Physical Ethernet interface base Queue
 *  @param[out] errCode
 *      Error code populated by the API
 *
 * \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -  <0
 */
int32_t Netfp_initInterfaceRoutingInfo
(
    Netfp_ServerHandle  serverHandle,
    uint32_t            baseFlowId,
    uint32_t            baseQueue,
    int32_t*            errCode
)
{
    Netfp_ServerMCB*    ptrNetfpServer;

    /* Validate input:
     * - Validate the server handle
     */
    if (serverHandle  == NULL )
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Get server MCB */
    ptrNetfpServer = (Netfp_ServerMCB *)serverHandle;

    /* Save interface based Queue and Flow */
    ptrNetfpServer->interfaceBaseFlowId = baseFlowId;
    ptrNetfpServer->interfaceBaseQueue  = baseQueue;

    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to configure the exception routes in the NETCP subsystem
 *      for the NAT-T Data messages. All other NAT-T message routes are configured by
 *      Linux using interface based routing settings.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[out] errCode
 *      Error code populated on error
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Netfp_configureNATTDataMessage
(
    Netfp_ServerMCB*        ptrNetfpServer,
    int32_t*                errCode
)
{
    int                 eRouteTypes;
    paRouteInfo2_t      eRoutes;
    paCmdReply_t        cmdReplyInfo;
    paReturn_t          retVal;
    Ti_Pkt*             ptrCmdPacket;
    uint32_t            dataBufferLen;
    uint8_t*            ptrDataBuffer;
    int32_t             cmdDest;
    uint16_t            cmdSize;
    uint32_t            psCmd = PASAHO_PACFG_CMD;

    /* Initialize the exception route configuration. */
    memset ((void *)&eRoutes, 0, sizeof (eRoutes));

    /* NAT-T detection is enabled. Configure exception route such that
     * packet is routed back to PDSP1. */
    eRouteTypes         = pa_EROUTE_NAT_T_DATA;
    eRoutes.dest        = pa_DEST_HOST;
    eRoutes.flowId      = Cppi_getFlowId(ptrNetfpServer->ipsecFlowHandle);
    eRoutes.mRouteIndex = -1;
    eRoutes.queue       = NETFP_CFG_BOUNCE_Q(ptrNetfpServer->netcpTxQueue[NSS_PA_QUEUE_OUTER_IP_INDEX]);

    /* Initialize the command reply:
     *  - We want command replies to come to the Host in the PA Configuration
     *    Response queue and also use the Flow which we had configured. */
    memset ((void *)&cmdReplyInfo, 0, sizeof(paCmdReply_t));
    Netfp_fillPaCmdReply(&cmdReplyInfo, ptrNetfpServer);

    /* Allocate a packet for the configuration command. */
    ptrCmdPacket = Pktlib_allocPacket(ptrNetfpServer->cfg.pktlibInstHandle,
                                      ptrNetfpServer->cfg.cmdHeapHandle,
                                      pa_CONFIG_EXCEPTION_ROUTE_MIN_CMD_BUF_SIZE_BYTES);
    if (ptrCmdPacket == NULL)
    {
        *errCode = NETFP_ENOMEM;
        return -1;
    }

    /* Get the data buffer from the allocated packet. */
    Pktlib_getDataBuffer(ptrCmdPacket, &ptrDataBuffer, &dataBufferLen);

    /* Setup the command size. */
    cmdSize = (uint16_t)dataBufferLen;

    /* Configure the exception routes. */
    retVal = Pa_configExceptionRoute2 (ptrNetfpServer->paHandle, 1,
                                      &eRouteTypes, &eRoutes, ptrDataBuffer,
                                      &cmdSize, &cmdReplyInfo, &cmdDest);
    if (retVal != pa_OK)
    {
        /* Error: Configuration Failed. */
        Pktlib_freePacket(ptrNetfpServer->cfg.pktlibInstHandle, ptrCmdPacket);
        *errCode = retVal;
        return -1;
    }

    /* Set the packet & data buffer length. */
    Pktlib_setPacketLen(ptrCmdPacket, cmdSize);
    Pktlib_setDataBufferLen(ptrCmdPacket, cmdSize);

    /* Set the software information fields. This is required only so that the
     * PS information goes to the correct location. */
    Cppi_setSoftwareInfo0(Cppi_DescType_HOST, (Cppi_Desc *)ptrCmdPacket, 0x12345678);

    /* Mark the packet as a configuration packet */
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)ptrCmdPacket, (uint8_t *)&psCmd, 4);

    /* Ensure that the command packet is also written back */
    ptrNetfpServer->cfg.endMemAccess(ptrDataBuffer, dataBufferLen);

    /******************************************************************************
     * NOTE: There is an ownership change here from the DSP to the CPDMA. We need
     * to ensure that the packet contents here are written back.
     ******************************************************************************/
    Pktlib_releaseOwnership(ptrNetfpServer->cfg.pktlibInstHandle, ptrCmdPacket);

    /* Send the command to the PA. */
    Qmss_queuePushDescSize (ptrNetfpServer->netcpTxQueue[cmdDest - pa_CMD_TX_DEST_0 + NSS_PA_QUEUE_INPUT_INDEX], (Cppi_Desc *)ptrCmdPacket, 64);

    /* Wait for the result and process it accordingly by forwarding to the PA subsystem. */
    retVal = Netfp_processResponse(ptrNetfpServer, cmdReplyInfo.replyId, NULL);
    if (retVal < 0)
    {
        *errCode = retVal;
        return -1;
    }

    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to update NAT-T info on netfp server.
 *
 *  @param[in]  serverHandle
 *      NETFP Server Handle
 *  @param[in]  nattCfg
 *      NAT-T configuration including UDP port number and wildcarded entry setting
 *  @param[out] errCode
 *      Error code populated by the API
 *
 * \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -  <0
 */
int32_t Netfp_updateNattInfo
(
    Netfp_ServerHandle  serverHandle,
    Netfp_NattCfg*      nattCfg,
    int32_t*            errCode
)
{
    Netfp_ServerMCB*    ptrNetfpServer;

    /* Validate input:
     * - Validate the server handle
     * - Validat the NAT-T UDP port number
     */
    if ((serverHandle  == NULL) || (nattCfg == NULL))
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Get server MCB */
    ptrNetfpServer = (Netfp_ServerMCB *)serverHandle;

    /* Save NAT-T setting on NetFP server */
    memcpy((void*)&ptrNetfpServer->nattCfg, (void *)nattCfg, sizeof(Netfp_NattCfg));

    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to initialized NAT-T in NETCP system.
 *
 *  @param[in]  serverHandle
 *      NETFP Server Handle
 *  @param[in] nattCfg
 *      NAT-T configurations including UDP port and wild carded setting
 *  @param[out] errCode
 *      Error code populated by the API
 *
 * \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -  <0
 */
int32_t Netfp_initNatt
(
    Netfp_ServerHandle  serverHandle,
    Netfp_NattCfg*      nattCfg,
    int32_t*            errCode
)
{
    Netfp_ServerMCB*    ptrNetfpServer;
    paCmdReply_t        cmdReplyInfo;
    paReturn_t          retVal;
    Ti_Pkt*             ptrCmdPacket;
    uint32_t            dataBufferLen;
    uint8_t*            ptrDataBuffer;
    int32_t             cmdDest;
    uint16_t            cmdSize;
    uint32_t            psCmd = PASAHO_PACFG_CMD;
    paCtrlInfo_t        configInfo;

    /* Validate input:
     * - Validate the server handle
     * - Validat the NAT-T UDP port number
     */
    if ((serverHandle  == NULL ) || (nattCfg == NULL))
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Get server MCB */
    ptrNetfpServer = (Netfp_ServerMCB *)serverHandle;

    /* Save NAT-T setting on NetFP server */
    memcpy((void *)&ptrNetfpServer->nattCfg, (void *)nattCfg, sizeof(Netfp_NattCfg));

    /* Initialize the command reply:
     *  - We want command replies to come to the Host in the PA Configuration
     *    Response queue and also use the Flow which we had configured. */
    memset ((void *)&cmdReplyInfo, 0, sizeof(paCmdReply_t));
    Netfp_fillPaCmdReply(&cmdReplyInfo, ptrNetfpServer);

    /* Initialize the system configuration information. */
    memset ((void *)&configInfo, 0, sizeof(paCtrlInfo_t));

    /* Issue the command set command */
    configInfo.code                              = pa_CONTROL_IPSEC_NAT_T_CONFIG;
    configInfo.params.ipsecNatTDetCfg.udpPort    = nattCfg->udpPort;
    configInfo.params.ipsecNatTDetCfg.ctrlBitMap = pa_IPSEC_NAT_T_CTRL_ENABLE;

    /* Allocate a packet for the configuration command. */
    ptrCmdPacket = Pktlib_allocPacket(ptrNetfpServer->cfg.pktlibInstHandle,
                                      ptrNetfpServer->cfg.cmdHeapHandle,
                                      pa_IPSEC_NAT_T_DET_MIN_CMD_BUF_SIZE_BYTES);
    if (ptrCmdPacket == NULL)
    {
        *errCode = NETFP_ENOMEM;
        return -1;
    }

    /* Get the data buffer from the allocated packet. */
    Pktlib_getDataBuffer(ptrCmdPacket, &ptrDataBuffer, &dataBufferLen);

    /* Setup the command size. */
    cmdSize = (uint16_t)dataBufferLen;

    /* Set the global config in the PA Susbystem. */
    retVal = Pa_control (ptrNetfpServer->paHandle,
                         &configInfo,
                         (paCmd_t)ptrDataBuffer,
                         &cmdSize,
                         &cmdReplyInfo,
                         &cmdDest);
    if (retVal != pa_OK)
    {
        /* Error: Configuration Failed. */
        Pktlib_freePacket(ptrNetfpServer->cfg.pktlibInstHandle, ptrCmdPacket);
        *errCode = (int32_t)retVal;
        return -1;
    }

    /* Set the packet & data buffer length. */
    Pktlib_setPacketLen(ptrCmdPacket, cmdSize);
    Pktlib_setDataBufferLen(ptrCmdPacket, cmdSize);

    /* Set the software information fields. This is required only so that the
     * PS information goes to the correct location. */
    Cppi_setSoftwareInfo0(Cppi_DescType_HOST, (Cppi_Desc *)ptrCmdPacket, 0x12345678);

    /* Mark the packet as a configuration packet */
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)ptrCmdPacket, (uint8_t *)&psCmd, 4);

    /* Ensure that the command packet is also written back */
    ptrNetfpServer->cfg.endMemAccess(ptrDataBuffer, dataBufferLen);

    /******************************************************************************
     * NOTE: There is an ownership change here from the DSP to the CPDMA. We need
     * to ensure that the packet contents here are written back.
     ******************************************************************************/
    Pktlib_releaseOwnership(ptrNetfpServer->cfg.pktlibInstHandle, ptrCmdPacket);

    /* Send the command to the PA. */
    Qmss_queuePushDescSize (ptrNetfpServer->netcpTxQueue[cmdDest - pa_CMD_TX_DEST_0 + NSS_PA_QUEUE_INPUT_INDEX], (Cppi_Desc *)ptrCmdPacket, 64);

    /* Wait for the result and process it accordingly by forwarding to the PA subsystem. */
    retVal = Netfp_processResponse(ptrNetfpServer, cmdReplyInfo.replyId, NULL);
    if (retVal < 0)
    {
        *errCode = retVal;
        return -1;
    }

    /* Configure exception route for NAT-T date messages */
    if (Netfp_configureNATTDataMessage(ptrNetfpServer, errCode) < 0)
        return -1;

    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to program the specific commands into the
 *      PA subsystem.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  numCommands
 *      Number of commands in this command set
 *  @param[in]  ptrCommandInfo
 *      Command set which is to be programmed into the PA
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   Command set index
 *  @retval
 *      Error   -   <0
 */
static int32_t Netfp_paConfigCommandSet
(
    Netfp_ServerMCB*    ptrNetfpServer,
    uint16_t            numCommands,
    paCmdInfo_t*        ptrCommandInfo
    )
{
    paCmdReply_t    cmdReplyInfo;
    paReturn_t      retVal;
    Ti_Pkt*         ptrCmdPacket;
    uint32_t        dataBufferLen;
    uint8_t*        ptrDataBuffer;
    int32_t         cmdDest;
    uint16_t        cmdSize;
    uint32_t        psCmd = PASAHO_PACFG_CMD;

    /* Initialize the command reply:
     *  - We want command replies to come to the Host in the PA Configuration
     *    Response queue and also use the Flow which we had configured. */
    memset ((void *)&cmdReplyInfo, 0, sizeof(paCmdReply_t));
    Netfp_fillPaCmdReply(&cmdReplyInfo, ptrNetfpServer);

    /* Allocate a packet for the configuration command. */
    ptrCmdPacket = Pktlib_allocPacket(ptrNetfpServer->cfg.pktlibInstHandle, ptrNetfpServer->cfg.cmdHeapHandle,
                                      pa_CONFIG_CMD_SET_MIN_CMD_BUF_SIZE_BYTES);
    if (ptrCmdPacket == NULL)
        return -1;

    /* Get the data buffer from the allocated packet. */
    Pktlib_getDataBuffer(ptrCmdPacket, &ptrDataBuffer, &dataBufferLen);

    /* Setup the command size. */
    cmdSize = (uint16_t)dataBufferLen;

    /* Generate a unique command set. */
    ptrNetfpServer->cmdSet.cmdSetIndexCounter++;

    /* Configure the command set. */
    retVal = Pa_configCmdSet (ptrNetfpServer->paHandle,
                              ptrNetfpServer->cmdSet.cmdSetIndexCounter,
                              numCommands,
                              ptrCommandInfo,
                              ptrDataBuffer,
                              &cmdSize,
                              &cmdReplyInfo,
                              &cmdDest);
    if (retVal != pa_OK)
    {
        /* Error: Configuration Failed. */
        Pktlib_freePacket(ptrNetfpServer->cfg.pktlibInstHandle, ptrCmdPacket);
        return retVal;
    }

    /* Set the packet & data buffer length. */
    Pktlib_setPacketLen(ptrCmdPacket, cmdSize);
    Pktlib_setDataBufferLen(ptrCmdPacket, cmdSize);

    /* Set the software information fields. This is required only so that the
     * PS information goes to the correct location. */
    Cppi_setSoftwareInfo0(Cppi_DescType_HOST, (Cppi_Desc *)ptrCmdPacket, 0x12345678);

    /* Mark the packet as a configuration packet */
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)ptrCmdPacket, (uint8_t *)&psCmd, 4);

    /* Ensure that the command packet is also written back */
    ptrNetfpServer->cfg.endMemAccess(ptrDataBuffer, dataBufferLen);

    /******************************************************************************
     * NOTE: There is an ownership change here from the DSP to the CPDMA. We need
     * to ensure that the packet contents here are written back.
     ******************************************************************************/
    Pktlib_releaseOwnership(ptrNetfpServer->cfg.pktlibInstHandle, ptrCmdPacket);

    System_printf ("---------------------> %d %d %d %d\n",
                   ptrNetfpServer->netcpTxQueue[cmdDest - pa_CMD_TX_DEST_0 + NSS_PA_QUEUE_INPUT_INDEX],
                   cmdDest, pa_CMD_TX_DEST_0, NSS_PA_QUEUE_INPUT_INDEX);

    /* Send the command to the PA. */
    Qmss_queuePushDescSize (ptrNetfpServer->netcpTxQueue[cmdDest - pa_CMD_TX_DEST_0 + NSS_PA_QUEUE_INPUT_INDEX], (Cppi_Desc *)ptrCmdPacket, 64);

    /* Wait for the result and process it accordingly by forwarding to the PA subsystem. */
    retVal = Netfp_processResponse(ptrNetfpServer, cmdReplyInfo.replyId, NULL);
    if (retVal < 0)
        return retVal;

    /* Command set has been successfully created. Return the command set index allocated for this. */
    return ptrNetfpServer->cmdSet.cmdSetIndexCounter;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the NETCP statistics
 *
 *  @param[in]  serverHandle
 *      Handle to the NETFP Server
 *  @param[out] ptrNETCPStats
 *      Pointer to the NETCP statistics
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
int32_t Netfp_getNETCPStats
(
    Netfp_ServerHandle  serverHandle,
    paSysStats_t*       ptrNETCPStats,
    int32_t*            errCode
)
{
#if 1
   /* FZM stats are through the kernel */
   FILE *fp;
   char *line = NULL;
   char *linep;
   size_t buflen = 0;
   ssize_t len;
   int ret = 0;

   /* verify we can just read directly into the stats structure */
   typeof(ptrNETCPStats->classify1.nPackets) *stat = (void*)ptrNETCPStats;
   static_assert(sizeof(*ptrNETCPStats) == sizeof(*stat) * 32, "different number of stats");

   /* it doesn't matter which netdevice is used. eth1 will always exist */
   fp = fopen("/sys/class/net/eth1/device/pa_stats", "re");
   if (!fp) {
      *errCode = -errno;
      return -1;
   }

   memset(ptrNETCPStats, 0x00, sizeof(*ptrNETCPStats));

   while (-1 != (len = getline(&line, &buflen, fp))) {
      if (stat > (typeof(stat))(ptrNETCPStats+1)) {
         /* verify we parsed the proper number of lines */
         ret = -EMSGSIZE;
         break;
      }

      /* find the space before the stat value */
      linep = memrchr(line, ' ', len);
      if (!linep) {
         ret = -EBADMSG;
         break;
      }

      /* Save the stat. Note that the file is 64-bit but paSysStats_t is
       * only 32-bit, so the counters may saturate */
      *stat++ = strtoul(linep, NULL, 10);
   }

   if (line)
      free(line);

   fclose(fp);

   *errCode = ret;

   return ret ? -1 : 0;
#elif (defined (DEVICE_K2H) || defined (DEVICE_K2K))
    Netfp_ServerMCB*    ptrNetfpServer;
    paCmdReply_t        cmdReplyInfo;
    paReturn_t          retVal;
    Ti_Pkt*             ptrCmdPacket;
    uint32_t            dataBufferLen;
    uint8_t*            ptrDataBuffer;
    int32_t             cmdDest;
    uint16_t            cmdSize;
    uint32_t            psCmd = PASAHO_PACFG_CMD;

    /* Get the NETFP Server MCB */
    ptrNetfpServer = (Netfp_ServerMCB*)serverHandle;
    if (ptrNetfpServer == NULL)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Initialize the command reply:
     *  - We want command replies to come to the Host in the PA Configuration
     *    Response queue and also use the Flow which we had configured. */
    memset ((void *)&cmdReplyInfo, 0, sizeof(paCmdReply_t));
    Netfp_fillPaCmdReply(&cmdReplyInfo, ptrNetfpServer);

    /* Allocate a packet for the configuration command. */
    ptrCmdPacket = Pktlib_allocPacket(ptrNetfpServer->cfg.pktlibInstHandle, ptrNetfpServer->cfg.cmdHeapHandle,
                                      pa_ADD_MAC_MIN_CMD_BUF_SIZE_BYTES);
    if (ptrCmdPacket == NULL)
    {
        *errCode = NETFP_ENOMEM;
        return -1;
    }

    /* Get the data buffer from the allocated packet. */
    Pktlib_getDataBuffer(ptrCmdPacket, &ptrDataBuffer, &dataBufferLen);

    /* Setup the command size. */
    cmdSize = (uint16_t)dataBufferLen;

    /* Get the statistics from the PA */
    retVal = Pa_requestStats (ptrNetfpServer->paHandle, 0, (paCmd_t)ptrDataBuffer,
                              &cmdSize, &cmdReplyInfo, &cmdDest);
    if (retVal != pa_OK)
    {
        /* Error: Request user statistics failed. */
        Pktlib_freePacket(ptrNetfpServer->cfg.pktlibInstHandle, ptrCmdPacket);
        *errCode = retVal;
        return -1;
    }

    /* Set the packet & data buffer length. */
    Pktlib_setPacketLen(ptrCmdPacket, cmdSize);
    Pktlib_setDataBufferLen(ptrCmdPacket, cmdSize);

    /* Set the software information fields. This is required only so that the
     * PS information goes to the correct location. */
    Cppi_setSoftwareInfo0(Cppi_DescType_HOST, (Cppi_Desc *)ptrCmdPacket, 0x12345678);

    /* Mark the packet as a configuration packet */
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)ptrCmdPacket, (uint8_t *)&psCmd, 4);

    /* Ensure that the command packet is also written back */
    ptrNetfpServer->cfg.endMemAccess (ptrDataBuffer, dataBufferLen);

    /******************************************************************************
     * NOTE: There is an ownership change here from the DSP to the CPDMA. We need
     * to ensure that the packet contents here are written back.
     ******************************************************************************/
    Pktlib_releaseOwnership(ptrNetfpServer->cfg.pktlibInstHandle, ptrCmdPacket);

    /* Send the command to the PA. */
    Qmss_queuePushDescSize (ptrNetfpServer->netcpTxQueue[cmdDest - pa_CMD_TX_DEST_0 + NSS_PA_QUEUE_INPUT_INDEX], (Cppi_Desc *)ptrCmdPacket, 64);

    /* Wait for the result and process it accordingly by forwarding to the PA subsystem. */
    retVal = Netfp_processResponse(ptrNetfpServer, cmdReplyInfo.replyId, ptrNETCPStats);
    if (retVal < 0)
    {
        *errCode = retVal;
        return -1;
    }
    return 0;
#else
    Netfp_ServerMCB*    ptrNetfpServer;
    paReturn_t          retVal;

    /* Get the NETFP Server MCB */
    ptrNetfpServer = (Netfp_ServerMCB*)serverHandle;
    if (ptrNetfpServer == NULL)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Get the NETFP Statistics: */
    retVal = Pa_querySysStats (ptrNetfpServer->paHandle, FALSE, ptrNETCPStats);
    if (retVal != pa_OK)
        return retVal;
    return 0;
#endif
}

/**
 *  @b Description
 *  @n
 *      The function is a debug function to display the reassembly context in the
 *      NETCP subsystem.
 *
 *  @param[in]  serverHandle
 *      Handle to the NETFP Server
 *
 *  \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   Number of active traffic flows
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_displayReassemblyContext(Netfp_ServerHandle serverHandle)
{
    Netfp_ServerMCB*            ptrNetfpServer;
    paSnapShotDebugInfo_t       snapshotDebugInfo;
    pa_trafficFlow_t*           ptrTrafficFlow;
    paReturn_t                  retVal;
    int32_t                     index;
    int32_t                     activeTrafficFlows = 0;

    /* Get the NETFP Server MCB */
    ptrNetfpServer = (Netfp_ServerMCB*)serverHandle;
    if (ptrNetfpServer == NULL)
        return -1;

    /* Initialize the debug information block: */
    memset ((void *)&snapshotDebugInfo, 0, sizeof(paSnapShotDebugInfo_t));

    /* Populate the debug information block: */
    snapshotDebugInfo.debugInfoType = pa_DBG_INFO_TYPE_REASSEMBLY_ENABLE;

    /* Get the debug information from the PA LLD */
    retVal = Pa_getDbgpInfo(ptrNetfpServer->paHandle, &snapshotDebugInfo);
    if (retVal != pa_OK)
        return -1;

    /* Get the pointer to the OUTER Reassembly Context */
    ptrTrafficFlow = &snapshotDebugInfo.u.reassemContext.outer;

//fzm - use Netfp_dumpMsg to capture this with the USR1 signal
    /* Display the banner. */
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "**********************************************************\n");
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,  "Outer IP Reassembly Context\n");
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,  "Max. number of traffic flows  : %d\n", ptrTrafficFlow->numTF);
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,  "Number of active traffic flows: %d\n", ptrTrafficFlow->numActiveTF);
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,  "Outer IP reassembly queue     : %d\n", ptrTrafficFlow->queue);
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,  "Outer IP reassembly flow      : %d\n", ptrTrafficFlow->flowId);

    /* Cycle through all the reassembly flows */
    for (index = 0; index < 32; index++)
    {
        /* Is the traffic flow active or not? */
        if (ptrTrafficFlow->traffic_flow[index].index == -1)
            continue;

        /* Increment the number of active traffic flows: */
        activeTrafficFlows++;

        /* Display the traffic flow: */
        Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,  "Traffic Flow %d Protocol %d Count: %d SrcIP:0x%x DstIP:0x%x\n",
                      activeTrafficFlows, ptrTrafficFlow->traffic_flow[index].proto, ptrTrafficFlow->traffic_flow[index].count,
                      ptrTrafficFlow->traffic_flow[index].srcIp, ptrTrafficFlow->traffic_flow[index].dstIp);
    }

    /* Get the pointer to the Inner Reassembly Context */
    ptrTrafficFlow = &snapshotDebugInfo.u.reassemContext.inner;

    /* Display the banner. */
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "**********************************************************\n");
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,  "Inner IP Reassembly Context\n");
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,  "Max. number of traffic flows  : %d\n", ptrTrafficFlow->numTF);
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,  "Number of active traffic flows: %d\n", ptrTrafficFlow->numActiveTF);
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,  "Inner IP reassembly queue     : %d\n", ptrTrafficFlow->queue);
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,  "Inner IP reassembly flow      : %d\n", ptrTrafficFlow->flowId);

    /* Cycle through all the reassembly flows */
    for (index = 0; index < 32; index++)
    {
        /* Is the traffic flow active or not? */
        if (ptrTrafficFlow->traffic_flow[index].index == -1)
            continue;

        /* Increment the number of active traffic flows: */
        activeTrafficFlows++;

        /* Display the traffic flow: */
        Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,  "Traffic Flow %d Protocol %d Count: %d SrcIP:0x%x DstIP:0x%x\n",
                      activeTrafficFlows, ptrTrafficFlow->traffic_flow[index].proto, ptrTrafficFlow->traffic_flow[index].count,
                      ptrTrafficFlow->traffic_flow[index].srcIp, ptrTrafficFlow->traffic_flow[index].dstIp);
    }
    return activeTrafficFlows;
}

/**
 *  @b Description
 *  @n
 *      The function is used to program the various command sets. The function
 *      currently supports the programming of the following command sets
 *          - Remove the networking header & trailer
 *          - Verification of Frame Protocol CRC
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_paSetupCommandSets(Netfp_ServerMCB* ptrNetfpServer)
{
    paCmdInfo_t     paCmdInfo[3];
    paCmdCrcOp_t    paFpCrcCmdInfo[8];
    paCmdInfo_t     paFpCmdInfo;
    uint32_t        index;
    paReturn_t      retVal;

    /* Setup the PA command set */
    paCmdInfo[0].cmd = pa_CMD_REMOVE_HEADER;
    paCmdInfo[1].cmd = pa_CMD_REMOVE_TAIL;
    paCmdInfo[2].cmd = pa_CMD_VERIFY_PKT_ERROR;
    paCmdInfo[2].params.verifyPktErr.errorBitfield = pa_PKT_ERR_L4_CHECKSUM | pa_PKT_ERR_IP_CHECKSUM;
    paCmdInfo[2].params.verifyPktErr.dest    = pa_DEST_DISCARD;
    paCmdInfo[2].params.verifyPktErr.flowId  = 0;
    paCmdInfo[2].params.verifyPktErr.queue   = 0;
    paCmdInfo[2].params.verifyPktErr.swInfo0 = 0;

    /* Configure the command sets for the header removal. */
    ptrNetfpServer->cmdSet.cmdSetHdrRemoveIndex = Netfp_paConfigCommandSet (ptrNetfpServer, 3, &paCmdInfo[0]);
    if (ptrNetfpServer->cmdSet.cmdSetHdrRemoveIndex < 0)
        return -1;

    /* Set up CRC command info and configure command sets */
    /* CRC command for HS-DSCH Type 1 */
    paFpCrcCmdInfo[0].ctrlBitfield = pa_CRC_OP_CRC_VALIDATE |
                                        pa_CRC_OP_PAYLOAD_LENGTH_IN_HEADER |
                                        pa_CRC_OP_PAYLOAD_LENGTH_OFFSET_IS_NEGATIVE |
                                        pa_CRC_OP_CRC_RESULT_FOLLOW_PAYLOAD;
    paFpCrcCmdInfo[0].startOffset  = 7;
    paFpCrcCmdInfo[0].len          = 0;
    paFpCrcCmdInfo[0].lenOffset    = 4;
    paFpCrcCmdInfo[0].lenMask      = 0xFFFF;
    paFpCrcCmdInfo[0].lenAdjust    = 17;
    paFpCrcCmdInfo[0].crcOffset    = 0;
    paFpCrcCmdInfo[0].frameType    = 0;
    paFpCrcCmdInfo[0].crcSize      = 2;
    paFpCrcCmdInfo[0].initValue    = 0x0000;


    /* CRC command for HS-DSCH Type 2 */
    paFpCrcCmdInfo[1].ctrlBitfield = pa_CRC_OP_CRC_VALIDATE |
                                        pa_CRC_OP_PAYLOAD_LENGTH_IN_HEADER |
                                        pa_CRC_OP_PAYLOAD_LENGTH_OFFSET_IS_NEGATIVE |
                                        pa_CRC_OP_CRC_FRAME_TYPE |
                                        pa_CRC_OP_CRC_RESULT_FOLLOW_PAYLOAD;
    paFpCrcCmdInfo[1].startOffset  = 0;
    paFpCrcCmdInfo[1].len          = 0;
    paFpCrcCmdInfo[1].lenOffset    = 4;
    paFpCrcCmdInfo[1].lenMask      = 0xffff;
    paFpCrcCmdInfo[1].lenAdjust    = 10;
    paFpCrcCmdInfo[1].crcOffset    = 0;
    paFpCrcCmdInfo[1].frameType    = pa_CRC_OP_FRAME_TYPE_IUB_FP_HS_DSCH_TYPE2;
    paFpCrcCmdInfo[1].crcSize      = 2;
    paFpCrcCmdInfo[1].initValue    = 0x0000;

    /* CRC command for HS-DSCH Type 3 */
    paFpCrcCmdInfo[2].ctrlBitfield = pa_CRC_OP_CRC_VALIDATE |
                                        pa_CRC_OP_PAYLOAD_LENGTH_IN_HEADER |
                                        pa_CRC_OP_PAYLOAD_LENGTH_OFFSET_IS_NEGATIVE |
                                        pa_CRC_OP_CRC_FRAME_TYPE |
                                        pa_CRC_OP_CRC_RESULT_FOLLOW_PAYLOAD;
    paFpCrcCmdInfo[2].startOffset  = 0;
    paFpCrcCmdInfo[2].len          = 0;
    paFpCrcCmdInfo[2].lenOffset    = 4;
    paFpCrcCmdInfo[2].lenMask      = 0xffff;
    paFpCrcCmdInfo[2].lenAdjust    = 10;
    paFpCrcCmdInfo[2].crcOffset    = 0;
    paFpCrcCmdInfo[2].frameType    = pa_CRC_OP_FRAME_TYPE_IUB_FP_HS_DSCH_TYPE3;
    paFpCrcCmdInfo[2].crcSize      = 2;
    paFpCrcCmdInfo[2].initValue    = 0x0000;

    /* CRC command for DL-DCH (number of bearers = 1) */
    paFpCrcCmdInfo[3].ctrlBitfield = pa_CRC_OP_CRC_VALIDATE |
                                        pa_CRC_OP_PAYLOAD_LENGTH_IN_HEADER |
                                        pa_CRC_OP_PAYLOAD_LENGTH_OFFSET_IS_NEGATIVE |
                                        pa_CRC_OP_CRC_RESULT_FOLLOW_PAYLOAD;
    paFpCrcCmdInfo[3].startOffset  = 3;
    paFpCrcCmdInfo[3].len          = 0;
    paFpCrcCmdInfo[3].lenOffset    = 4;
    paFpCrcCmdInfo[3].lenMask      = 0xFFFF;
    paFpCrcCmdInfo[3].lenAdjust    = 13;
    paFpCrcCmdInfo[3].crcOffset    = 0;
    paFpCrcCmdInfo[3].frameType    = 0;
    paFpCrcCmdInfo[3].crcSize      = 2;
    paFpCrcCmdInfo[3].initValue    = 0x0000;

    /* CRC command for DL-DCH (number of bearers = 2) */
    paFpCrcCmdInfo[4].ctrlBitfield = pa_CRC_OP_CRC_VALIDATE |
                                        pa_CRC_OP_PAYLOAD_LENGTH_IN_HEADER |
                                        pa_CRC_OP_PAYLOAD_LENGTH_OFFSET_IS_NEGATIVE |
                                        pa_CRC_OP_CRC_RESULT_FOLLOW_PAYLOAD;
    paFpCrcCmdInfo[4].startOffset  = 4;
    paFpCrcCmdInfo[4].len          = 0;
    paFpCrcCmdInfo[4].lenOffset    = 4;
    paFpCrcCmdInfo[4].lenMask      = 0xFFFF;
    paFpCrcCmdInfo[4].lenAdjust    = 14;
    paFpCrcCmdInfo[4].crcOffset    = 0;
    paFpCrcCmdInfo[4].frameType    = 0;
    paFpCrcCmdInfo[4].crcSize      = 2;
    paFpCrcCmdInfo[4].initValue    = 0x0000;

    /* CRC command for DL-DCH (number of bearers = 3) */
    paFpCrcCmdInfo[5].ctrlBitfield = pa_CRC_OP_CRC_VALIDATE |
                                        pa_CRC_OP_PAYLOAD_LENGTH_IN_HEADER |
                                        pa_CRC_OP_PAYLOAD_LENGTH_OFFSET_IS_NEGATIVE |
                                        pa_CRC_OP_CRC_RESULT_FOLLOW_PAYLOAD;
    paFpCrcCmdInfo[5].startOffset  = 5;
    paFpCrcCmdInfo[5].len          = 0;
    paFpCrcCmdInfo[5].lenOffset    = 4;
    paFpCrcCmdInfo[5].lenMask      = 0xFFFF;
    paFpCrcCmdInfo[5].lenAdjust    = 15;
    paFpCrcCmdInfo[5].crcOffset    = 0;
    paFpCrcCmdInfo[5].frameType    = 0;
    paFpCrcCmdInfo[5].crcSize      = 2;
    paFpCrcCmdInfo[5].initValue    = 0x0000;

    /* CRC command for DL-DCH (number of bearers = 4) */
    paFpCrcCmdInfo[6].ctrlBitfield = pa_CRC_OP_CRC_VALIDATE |
                                        pa_CRC_OP_PAYLOAD_LENGTH_IN_HEADER |
                                        pa_CRC_OP_PAYLOAD_LENGTH_OFFSET_IS_NEGATIVE |
                                        pa_CRC_OP_CRC_RESULT_FOLLOW_PAYLOAD;
    paFpCrcCmdInfo[6].startOffset  = 6;
    paFpCrcCmdInfo[6].len          = 0;
    paFpCrcCmdInfo[6].lenOffset    = 4;
    paFpCrcCmdInfo[6].lenMask      = 0xFFFF;
    paFpCrcCmdInfo[6].lenAdjust    = 16;
    paFpCrcCmdInfo[6].crcOffset    = 0;
    paFpCrcCmdInfo[6].frameType    = 0;
    paFpCrcCmdInfo[6].crcSize      = 2;
    paFpCrcCmdInfo[6].initValue    = 0x0000;

    /* CRC command for DL-FACH and DL-PCH */
    paFpCrcCmdInfo[7].ctrlBitfield = pa_CRC_OP_CRC_VALIDATE |
                                        pa_CRC_OP_PAYLOAD_LENGTH_IN_HEADER |
                                        pa_CRC_OP_PAYLOAD_LENGTH_OFFSET_IS_NEGATIVE |
                                        pa_CRC_OP_CRC_RESULT_FOLLOW_PAYLOAD;
    paFpCrcCmdInfo[7].startOffset  = 4;
    paFpCrcCmdInfo[7].len          = 0;
    paFpCrcCmdInfo[7].lenOffset    = 4;
    paFpCrcCmdInfo[7].lenMask      = 0xFFFF;
    paFpCrcCmdInfo[7].lenAdjust    = 14;
    paFpCrcCmdInfo[7].crcOffset    = 0;
    paFpCrcCmdInfo[7].frameType    = 0;
    paFpCrcCmdInfo[7].crcSize      = 2;
    paFpCrcCmdInfo[7].initValue    = 0x0000;

    paFpCmdInfo.cmd = pa_CMD_CRC_OP;

    for (index = 0; index < 8; index++)
    {
        paFpCmdInfo.params.crcOp = paFpCrcCmdInfo[index];

        /* Configure each command set */
        retVal = Netfp_paConfigCommandSet (ptrNetfpServer, 1, &paFpCmdInfo);
        if (retVal < 0)
            return retVal;
        else
        {
            /* Save the first FP CRC command set configured. */
            if (index == 0)
                ptrNetfpServer->cmdSet.cmdSetFpCrcIndex = retVal;
        }
    }

    /* Command set has been programmed successfully. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to enable the Frame Protocol CRC Offload services
 *      This can only be done if the CRC firmware has been downloaded.
 *
 *  @param[in]  serverHandle
 *      NETFP Server Handle
 *  @param[out] errCode
 *      Error code populated by the API
 *
 * @sa
 *  Netfp_initFrameProtoCRC
 *
 * \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -  <0
 */
int32_t Netfp_enableFrameProtoCrcServices
(
    Netfp_ServerHandle  serverHandle,
    int32_t*            errCode
)
{
    Netfp_ServerMCB*    ptrNetfpServer;

    /* Sanity Check: Validate the arguments: */
    if (serverHandle  == NULL)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Get server MCB */
    ptrNetfpServer = (Netfp_ServerMCB *)serverHandle;

    /* Enable the frame protocol CRC Services */
    ptrNetfpServer->frameProtoCrcOffload = 1;
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to initialize and download the Frame Protocol CRC engine
 *      in the NETCP sub-system.
 *
 *  @param[in]  serverHandle
 *      NETFP Server Handle
 *  @param[out] errCode
 *      Error code populated by the API
 *
 * \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -  <0
 */
int32_t Netfp_initFrameProtoCRC
(
    Netfp_ServerHandle      serverHandle,
    int32_t*                errCode
)
{
    Ti_Pkt*                 ptrCmdPacket;
    uint32_t                dataBufferLen;
    uint8_t*                ptrDataBuffer;
    int32_t                 cmdDest;
    uint16_t                cmdSize;
    uint32_t                psCmd = PASAHO_PACFG_CMD;
    Netfp_ServerMCB*        ptrNetfpServer;
    uint32_t                index;
    paCmdReply_t            cmdReplyInfo;
    paReturn_t              retVal;
    paCrcConfig_t           crcConfig;

    /* Sanity Check: Validate the arguments: */
    if (serverHandle == NULL)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Get the pointer to the NETFP Server: */
    ptrNetfpServer = (Netfp_ServerMCB*)serverHandle;

    /* Setup the CRC Configuration for WCDMA FP */
    crcConfig.ctrlBitfield = 0x0000;
    crcConfig.size         = pa_CRC_SIZE_16;
    crcConfig.polynomial   = 0x80050000;
    crcConfig.initValue    = 0x0000;

    /* FP CRC is functionality is supported in PDSP4 and PDSP5.
     * We configure this functionality in 2 iterations.
     * First iteration configures PDSP 4 CRC Engine.
     * Second iteration configures PDSP 5 CRC Engine.
     */
    for (index = 0; index < 2; index++)
    {
        /* Initialize the command reply information.
         * We want command replies to come to the Host in the PA Configuration
         * Response queue and also use the Flow which we had configured.
         */
        memset ((void *)&cmdReplyInfo, 0, sizeof(cmdReplyInfo));
        Netfp_fillPaCmdReply(&cmdReplyInfo, ptrNetfpServer);

        /* Allocate a packet for the configuration command. */
        ptrCmdPacket = Pktlib_allocPacket(ptrNetfpServer->cfg.pktlibInstHandle, ptrNetfpServer->cfg.cmdHeapHandle,
                                      pa_CONFIG_CRC_ENGINE_MIN_CMD_BUF_SIZE_BYTES);
        if (ptrCmdPacket == NULL)
        {
            *errCode = NETFP_ENOMEM;
            return -1;
        }

        /* Get the data buffer from the allocated packet. */
        Pktlib_getDataBuffer(ptrCmdPacket, &ptrDataBuffer, &dataBufferLen);

        /* Setup the command size. */
        cmdSize = (uint16_t)dataBufferLen;

        /* Configure the PDSP 4 and PSDP 5 CRC engine. */
        retVal = Pa_configCrcEngine (ptrNetfpServer->paHandle,
                                     index + 4,
                                     &crcConfig,
                                     ptrDataBuffer,
                                     &cmdSize,
                                     &cmdReplyInfo,
                                     &cmdDest);
        if (retVal != pa_OK)
        {
            /* Error: Configuration Failed. */
            Pktlib_freePacket(ptrNetfpServer->cfg.pktlibInstHandle, ptrCmdPacket);
            *errCode = (int32_t)retVal;
            return -1;
        }

        /* Set the packet & data buffer length. */
        Pktlib_setPacketLen(ptrCmdPacket, cmdSize);
        Pktlib_setDataBufferLen(ptrCmdPacket, cmdSize);

        /* Set the software information fields. This is required only so that the
         * PS information goes to the correct location. */
        Cppi_setSoftwareInfo0(Cppi_DescType_HOST, (Cppi_Desc *)ptrCmdPacket, 0x12345678);

        /* Mark the packet as a configuration packet */
        Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)ptrCmdPacket, (uint8_t *)&psCmd, 4);

        /* Ensure that the command packet is also written back */
        ptrNetfpServer->cfg.endMemAccess(ptrDataBuffer, dataBufferLen);

        /******************************************************************************
         * NOTE: There is an ownership change here from the DSP to the CPDMA. We need
         * to ensure that the packet contents here are written back.
         ******************************************************************************/
	    Pktlib_releaseOwnership(ptrNetfpServer->cfg.pktlibInstHandle, ptrCmdPacket);

        /* Send the command to the PA. */
        Qmss_queuePushDescSize (ptrNetfpServer->netcpTxQueue[cmdDest - pa_CMD_TX_DEST_0 + NSS_PA_QUEUE_INPUT_INDEX], (Cppi_Desc *)ptrCmdPacket, 64);

        /* Wait for the result and process it accordingly by forwarding to the PA subsystem. */
        retVal = Netfp_processResponse(ptrNetfpServer, cmdReplyInfo.replyId, NULL);
        if (retVal < 0)
        {
            *errCode = (int32_t) retVal;
            return -1;
        }
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to initialize NETCP user statistics
 *
 *  @param[in]  serverHandle
 *      Handle to the NETFP Server
 *  @param[in]  ptrUserStatCfg
 *      Pointer to the user stats configuration
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
int32_t Netfp_initUserStats
(
    Netfp_ServerHandle      serverHandle,
    Netfp_SysUserStatCfg*   ptrUserStatCfg,
    int32_t*                errCode
)
{
    Netfp_ServerMCB*            ptrNetfpServer;
    paReturn_t                  paRet;
    paCmdReply_t                cmdReplyInfo;
    paCtrlInfo_t                configInfo;
    paUsrStatsConfig_t          usrStatsConfig;
    Ti_Pkt*                     ptrCmdPacket;
    uint32_t                    dataBufferLen;
    uint8_t*                    ptrDataBuffer;
    int32_t                     cmdDest;
    uint16_t                    cmdSize;
    uint32_t                    psCmd = PASAHO_PACFG_CMD;

    /* Get the NETFP Server MCB */
    ptrNetfpServer = (Netfp_ServerMCB*)serverHandle;

    /* Sanity Check: Validate the arguments */
    if ((ptrNetfpServer == NULL) || (ptrUserStatCfg == NULL))
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Sanity Check: Validate to ensure the system limits are not exceeded */
    if (ptrUserStatCfg->numTotalUserStats > pa_USR_STATS_MAX_COUNTERS)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Sanity Check: Validate the number of 64b statistics  */
    if ((ptrUserStatCfg->num64bUserStats > ptrUserStatCfg->numTotalUserStats) ||
        (ptrUserStatCfg->num64bUserStats > pa_USR_STATS_MAX_64B_COUNTERS))
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Save the configuration on Netfp Server */
    memcpy ((void *)&ptrNetfpServer->userStatCfg, (void*)ptrUserStatCfg, sizeof(Netfp_SysUserStatCfg));

    /* Allocate memory to store user stats returned from PA */
    ptrNetfpServer->ptrUserStats = (paUsrStats_t *)ptrNetfpServer->cfg.malloc(sizeof(paUsrStats_t), 0);
    if(ptrNetfpServer->ptrUserStats == (paUsrStats_t *)NULL)
    {
        *errCode = NETFP_ENOMEM;
        return -1;
    }

    /* Initialize the command reply:
     *  - We want command replies to come to the Host in the PA Configuration
     *    Response queue and also use the Flow which we had configured. */
    memset ((void *)&cmdReplyInfo, 0, sizeof(paCmdReply_t));
    Netfp_fillPaCmdReply(&cmdReplyInfo, ptrNetfpServer);

    /* Allocate a packet for the configuration command. */
    ptrCmdPacket = Pktlib_allocPacket(ptrNetfpServer->cfg.pktlibInstHandle,
                                      ptrNetfpServer->cfg.cmdHeapHandle,
                                      pa_CONFIG_USR_STATS_MIN_CMD_BUF_SIZE_BYTES);
    if (ptrCmdPacket == NULL)
    {
        /* Cleanup the user statistics buffer */
        ptrNetfpServer->cfg.free(ptrNetfpServer->ptrUserStats, sizeof(paUsrStats_t));
        *errCode = NETFP_ENOMEM;
        return -1;
    }

    /* Get the data buffer from the allocated packet. */
    Pktlib_getDataBuffer(ptrCmdPacket, &ptrDataBuffer, &dataBufferLen);

    /* Setup the command size. */
    cmdSize = (uint16_t)dataBufferLen;

    /* Initialize the user statistics & control configuration: */
    memset(&usrStatsConfig, 0, sizeof(paUsrStatsConfig_t));
    memset(&configInfo, 0, sizeof(paCtrlInfo_t));

    /* Populate the user statistics configuration: */
    usrStatsConfig.numCounters    = ptrUserStatCfg->numTotalUserStats;
    usrStatsConfig.num64bCounters = ptrUserStatCfg->num64bUserStats;

    /* Populate the control configuration */
    configInfo.code                             = pa_CONTROL_SYS_CONFIG;
    configInfo.params.sysCfg.pOutIpReassmConfig = NULL;
    configInfo.params.sysCfg.pInIpReassmConfig  = NULL;
    configInfo.params.sysCfg.pProtoLimit        = NULL;
    configInfo.params.sysCfg.pCmdSetConfig      = NULL;
    configInfo.params.sysCfg.pUsrStatsConfig    = &usrStatsConfig;
    configInfo.params.sysCfg.pQueueDivertConfig = NULL;

    /* Set the global user stats config in the PA Susbystem. */
    paRet = Pa_control (ptrNetfpServer->paHandle, &configInfo, (paCmd_t)ptrDataBuffer,
                        &cmdSize, &cmdReplyInfo, &cmdDest);
    if (paRet != pa_OK)
    {
        /* Error: Configuration Failed. */
        Pktlib_freePacket(ptrNetfpServer->cfg.pktlibInstHandle, ptrCmdPacket);

        /* Cleanup the user statistics buffer */
        ptrNetfpServer->cfg.free(ptrNetfpServer->ptrUserStats, sizeof(paUsrStats_t));
        *errCode = (int32_t)paRet;
        return -1;
    }

    /* Set the packet & data buffer length. */
    Pktlib_setPacketLen(ptrCmdPacket, cmdSize);
    Pktlib_setDataBufferLen(ptrCmdPacket, cmdSize);

    /* Set the software information fields. This is required only so that the
     * PS information goes to the correct location. */
    Cppi_setSoftwareInfo0(Cppi_DescType_HOST, (Cppi_Desc *)ptrCmdPacket, 0x12345678);

    /* Mark the packet as a configuration packet */
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)ptrCmdPacket, (uint8_t *)&psCmd, 4);

    /* Ensure that the command packet is also written back */
    ptrNetfpServer->cfg.endMemAccess(ptrDataBuffer, dataBufferLen);

    /******************************************************************************
     * NOTE: There is an ownership change here from the DSP to the CPDMA. We need
     * to ensure that the packet contents here are written back.
     ******************************************************************************/
    Pktlib_releaseOwnership(ptrNetfpServer->cfg.pktlibInstHandle, ptrCmdPacket);

    /* Send the command to the PA. */
    Qmss_queuePushDescSize (ptrNetfpServer->netcpTxQueue[cmdDest - pa_CMD_TX_DEST_0 + NSS_PA_QUEUE_INPUT_INDEX],
                            (Cppi_Desc *)ptrCmdPacket, 64);

    /* Wait for the result and process it accordingly by forwarding to the PA subsystem. */
    paRet = Netfp_processResponse(ptrNetfpServer, cmdReplyInfo.replyId, NULL);
    if (paRet < 0)
    {
        /* Cleanup the user statistics buffer */
        ptrNetfpServer->cfg.free(ptrNetfpServer->ptrUserStats, sizeof(paUsrStats_t));
        *errCode = paRet;
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to program the user statistics in the NETCP.
 *
 *  @param[in]  ptrNetfpServer
 *      Handle to the NETFP Server
 *  @param[in]  numUserStats
 *      Number of user stats to be configured
 *  @param[in]  ptrPaUsrStatsCounterTbl
 *      Pointer to the user stats configuration
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
static int32_t Netfp_configPaUserStats
(
    Netfp_ServerMCB*                  ptrNetfpServer,
    uint32_t                          numUserStats,
    paUsrStatsCounterEntryConfig_t*   ptrPaUsrStatsCounterTbl,
    int32_t*                          errCode
)
{
    paReturn_t                  paRet;
    paUsrStatsCounterConfig_t   paUsrStatsCounterCfg;
    paUsrStatsConfigInfo_t      statsCfgInfo;
    paCmdReply_t                cmdReplyInfo;
    Ti_Pkt*                     ptrCmdPacket;
    uint32_t                    dataBufferLen;
    uint8_t*                    ptrDataBuffer;
    int32_t                     cmdDest;
    uint16_t                    cmdSize;
    uint32_t                    psCmd = PASAHO_PACFG_CMD;

    /* Initialize the command reply:
     *  - We want command replies to come to the Host in the PA Configuration
     *    Response queue and also use the Flow which we had configured. */
    memset ((void *)&cmdReplyInfo, 0, sizeof(paCmdReply_t));
    Netfp_fillPaCmdReply(&cmdReplyInfo, ptrNetfpServer);

    /* Allocate a packet for the configuration command. */
    ptrCmdPacket = Pktlib_allocPacket(ptrNetfpServer->cfg.pktlibInstHandle,
                                      ptrNetfpServer->cfg.cmdHeapHandle,
                                      pa_CONFIG_USR_STATS_MIN_CMD_BUF_SIZE_BYTES);
    if (ptrCmdPacket == NULL)
    {
        *errCode = NETFP_ENOMEM;
        return -1;
    }

    /* Get the data buffer from the allocated packet. */
    Pktlib_getDataBuffer(ptrCmdPacket, &ptrDataBuffer, &dataBufferLen);

    /* Setup the command size. */
    cmdSize = (uint16_t)dataBufferLen;

    /* Prepare and format the command */
    memset(&statsCfgInfo, 0, sizeof(statsCfgInfo));
    memset(&paUsrStatsCounterCfg, 0, sizeof(paUsrStatsCounterCfg));
    paUsrStatsCounterCfg.numCnt       = numUserStats;
    paUsrStatsCounterCfg.cntInfo      = ptrPaUsrStatsCounterTbl;
    statsCfgInfo.pCntCfg = &paUsrStatsCounterCfg;

    /* Configure the user statistics: */
    paRet = Pa_configUsrStats(ptrNetfpServer->paHandle, &statsCfgInfo, (paCmd_t)ptrDataBuffer,
                              &cmdSize, &cmdReplyInfo, &cmdDest);
    if (paRet != pa_OK)
    {
        /* Error: Configuration Failed. */
        Pktlib_freePacket(ptrNetfpServer->cfg.pktlibInstHandle, ptrCmdPacket);
        *errCode = (int32_t)paRet;
        return -1;
    }

    /* Set the packet & data buffer length. */
    Pktlib_setPacketLen(ptrCmdPacket, cmdSize);
    Pktlib_setDataBufferLen(ptrCmdPacket, cmdSize);

    /* Set the software information fields. This is required only so that the
     * PS information goes to the correct location. */
    Cppi_setSoftwareInfo0(Cppi_DescType_HOST, (Cppi_Desc *)ptrCmdPacket, 0x12345678);

    /* Mark the packet as a configuration packet */
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)ptrCmdPacket, (uint8_t *)&psCmd, 4);

    /* Ensure that the command packet is also written back */
    ptrNetfpServer->cfg.endMemAccess(ptrDataBuffer, dataBufferLen);

    /******************************************************************************
     * NOTE: There is an ownership change here from the DSP to the CPDMA. We need
     * to ensure that the packet contents here are written back.
     ******************************************************************************/
    Pktlib_releaseOwnership(ptrNetfpServer->cfg.pktlibInstHandle, ptrCmdPacket);

    /* Send the command to the PA. */
    Qmss_queuePushDescSize (ptrNetfpServer->netcpTxQueue[cmdDest - pa_CMD_TX_DEST_0 + NSS_PA_QUEUE_INPUT_INDEX], (Cppi_Desc *)ptrCmdPacket, 64);

    /* Wait for the result and process it accordingly by forwarding to the PA subsystem. */
    paRet = Netfp_processResponse(ptrNetfpServer, cmdReplyInfo.replyId, NULL);
    if (paRet < 0)
    {
        *errCode = paRet;
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to alloc User Stats index
 *
 *  @param[in]  ptrNetfpServer
 *      Handle to the NETFP Server
 *  @param[in] cntType
 *      Type of user stats counter - 32bit or 64bit
 *  @param[in] ptrCntIdex
 *      Pointer to hold the return index
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
static int32_t Netfp_allocPaUserStatsIndex
(
    Netfp_ServerMCB*    ptrNetfpServer,
    Netfp_UserStatsLen  cntType,
    uint16_t*           ptrCntIdex,
    int32_t*            errCode
)
{
    paUsrStatsAlloc_t   paCntList;
    paReturn_t          paRet;
    int32_t             numCnt;

    /* Sanity check return cntIndex pointer */
    if (ptrCntIdex == NULL)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Initialize PA user stats counter */
    memset(&paCntList, 0, sizeof(paUsrStatsAlloc_t));

    if (cntType == Netfp_UserStatsLen_64b)
        paCntList.ctrlBitfield = pa_USR_STATS_ALLOC_64B_CNT;

    /* Allocate one User stats counter */
    numCnt = 1;
    paRet = Pa_allocUsrStats(ptrNetfpServer->paHandle, &numCnt, &paCntList);
    if (paRet != pa_OK)
    {
        *errCode = paRet;
        return -1;
    }
    *ptrCntIdex = paCntList.cntIndex;
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to free the NETCP User stats index
 *
 *  @param[in]  ptrNetfpServer
 *      Handle to the NETFP Server
 *  @param[in] userCntIndex
 *      User stats counter index to be freed
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
static int32_t Netfp_freePaUserStatsIndex
(
    Netfp_ServerMCB*    ptrNetfpServer,
    uint16_t            userCntIndex,
    int32_t*            errCode
)
{
    uint16_t            paCntList;
    paReturn_t          paRet;
    int                 numCnt;

    /* Prepare to free the resource */
    numCnt       = 1;
    paCntList    = userCntIndex;

    /* Free the allocated usr status counter */
    paRet = Pa_freeUsrStats(ptrNetfpServer->paHandle, numCnt, &paCntList);
    if (paRet != pa_OK)
    {
        *errCode = paRet;
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the NETCP statistics
 *
 *  @param[in]  ptrNetfpServer
 *      Handle to the NETFP Server
 *  @param[out] ptrUserStats
 *      Pointer to the NETCP statistics
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
int32_t Netfp_getUserStats
(
    Netfp_ServerMCB*    ptrNetfpServer,
    paUsrStats_t*       ptrUserStats,
    int32_t*            errCode
)
{
    paReturn_t          paRet;
    int32_t             cmdDest;
    uint16_t            cmdSize;
    uint32_t            psCmd = PASAHO_PACFG_CMD;
    paCmdReply_t        cmdReplyInfo;
    Ti_Pkt*             ptrCmdPacket;
    uint32_t            dataBufferLen;
    uint8_t*            ptrDataBuffer;

    /* Initialize the command reply:
     *  - We want command replies to come to the Host in the PA Configuration
     *    Response queue and also use the Flow which we had configured. */
    memset ((void *)&cmdReplyInfo, 0, sizeof(paCmdReply_t));
    Netfp_fillPaCmdReply(&cmdReplyInfo, ptrNetfpServer);

    /* Allocate a packet for the configuration command. */
    ptrCmdPacket = Pktlib_allocPacket(ptrNetfpServer->cfg.pktlibInstHandle,
                                      ptrNetfpServer->cfg.cmdHeapHandle,
                                      pa_CONFIG_USR_STATS_MIN_CMD_BUF_SIZE_BYTES);
    if (ptrCmdPacket == NULL)
    {
        *errCode = NETFP_ENOMEM;
        return -1;
    }

    /* Get the data buffer from the allocated packet. */
    Pktlib_getDataBuffer(ptrCmdPacket, &ptrDataBuffer, &dataBufferLen);

    /* Setup the command size. */
    cmdSize = (uint16_t)dataBufferLen;

    /* Clear all counters from */
    memset(ptrUserStats, 0, sizeof(paUsrStats_t));

    /* Request counters from PA */
    paRet = Pa_requestUsrStats (ptrNetfpServer->paHandle,
                                0,
                                (paCmd_t)ptrDataBuffer,
                                &cmdSize,
                                &cmdReplyInfo,
                                &cmdDest,
                                NULL);
    if (paRet != pa_OK)
    {
        *errCode = paRet;
        return -1;
    }

    /* Set the packet & data buffer length. */
    Pktlib_setPacketLen(ptrCmdPacket, cmdSize);
    Pktlib_setDataBufferLen(ptrCmdPacket, cmdSize);

    /* Set the software information fields. This is required only so that the
     * PS information goes to the correct location. */
    Cppi_setSoftwareInfo0(Cppi_DescType_HOST, (Cppi_Desc *)ptrCmdPacket, 0x12345678);

    /* Mark the packet as a configuration packet */
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)ptrCmdPacket, (uint8_t *)&psCmd, 4);

    /* Ensure that the command packet is also written back */
    ptrNetfpServer->cfg.endMemAccess(ptrDataBuffer, dataBufferLen);

    /******************************************************************************
     * NOTE: There is an ownership change here from the DSP to the CPDMA. We need
     * to ensure that the packet contents here are written back.
     ******************************************************************************/
    Pktlib_releaseOwnership(ptrNetfpServer->cfg.pktlibInstHandle, ptrCmdPacket);

    /* Send the command to the PA. */
    Qmss_queuePushDescSize (ptrNetfpServer->netcpTxQueue[cmdDest - pa_CMD_TX_DEST_0 + NSS_PA_QUEUE_INPUT_INDEX], (Cppi_Desc *)ptrCmdPacket, 64);

    /* Wait for the result and process it accordingly by forwarding to the PA subsystem. */
    paRet = Netfp_processUserStatsResponse(ptrNetfpServer, cmdReplyInfo.replyId, ptrUserStats);
    if (paRet < 0)
    {
        *errCode = paRet;
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to clear the NETCP statistics
 *
 *  @param[in]  ptrNetfpServer
 *      Handle to the NETFP Server
 *  @param[in]  userStatsIndex
 *      User statistics index for which the statistics will be cleared
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
static int32_t Netfp_clearUserStats
(
    Netfp_ServerMCB*    ptrNetfpServer,
    uint16_t            userStatsIndex,
    int32_t*            errCode
)
{
    paReturn_t          paRet;
    int32_t             cmdDest;
    uint16_t            cmdSize;
    uint32_t            psCmd = PASAHO_PACFG_CMD;
    paCmdReply_t        cmdReplyInfo;
    Ti_Pkt*             ptrCmdPacket;
    uint32_t            dataBufferLen;
    uint8_t*            ptrDataBuffer;

    /* Initialize the command reply:
     *  - We want command replies to come to the Host in the PA Configuration
     *    Response queue and also use the Flow which we had configured. */
    memset ((void *)&cmdReplyInfo, 0, sizeof(paCmdReply_t));
    Netfp_fillPaCmdReply(&cmdReplyInfo, ptrNetfpServer);

    /* Allocate a packet for the configuration command. */
    ptrCmdPacket = Pktlib_allocPacket(ptrNetfpServer->cfg.pktlibInstHandle,
                                      ptrNetfpServer->cfg.cmdHeapHandle,
                                      pa_CONFIG_USR_STATS_MIN_CMD_BUF_SIZE_BYTES);
    if (ptrCmdPacket == NULL)
    {
        *errCode = NETFP_ENOMEM;
        return -1;
    }

    /* Get the data buffer from the allocated packet. */
    Pktlib_getDataBuffer(ptrCmdPacket, &ptrDataBuffer, &dataBufferLen);

    /* Setup the command size. */
    cmdSize = (uint16_t)dataBufferLen;

    /* We are clearing the counters */
    paRet = Pa_requestUsrStatsList (ptrNetfpServer->paHandle,
                                    1,
                                    1,
                                    &userStatsIndex,
                                    (paCmd_t)ptrDataBuffer,
                                    &cmdSize,
                                    &cmdReplyInfo,
                                    &cmdDest,
                                    NULL);
    if (paRet != pa_OK)
    {
        *errCode = paRet;
        return -1;
    }

    /* Set the packet & data buffer length. */
    Pktlib_setPacketLen(ptrCmdPacket, cmdSize);
    Pktlib_setDataBufferLen(ptrCmdPacket, cmdSize);

    /* Set the software information fields. This is required only so that the
     * PS information goes to the correct location. */
    Cppi_setSoftwareInfo0(Cppi_DescType_HOST, (Cppi_Desc *)ptrCmdPacket, 0x12345678);

    /* Mark the packet as a configuration packet */
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)ptrCmdPacket, (uint8_t *)&psCmd, 4);

    /* Ensure that the command packet is also written back */
    ptrNetfpServer->cfg.endMemAccess(ptrDataBuffer, dataBufferLen);

    /******************************************************************************
     * NOTE: There is an ownership change here from the DSP to the CPDMA. We need
     * to ensure that the packet contents here are written back.
     ******************************************************************************/
    Pktlib_releaseOwnership(ptrNetfpServer->cfg.pktlibInstHandle, ptrCmdPacket);

    /* Send the command to the PA. */
    Qmss_queuePushDescSize (ptrNetfpServer->netcpTxQueue[cmdDest - pa_CMD_TX_DEST_0 + NSS_PA_QUEUE_INPUT_INDEX], (Cppi_Desc *)ptrCmdPacket, 64);

    /* Wait for the result and process it accordingly by forwarding to the PA subsystem. */
    paRet = Netfp_processUserStatsResponse(ptrNetfpServer, cmdReplyInfo.replyId, NULL);
    if (paRet < 0)
    {
        *errCode = paRet;
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to create a user statistics chain and initialize NETCP
 *      with the chain.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  numUserStats
 *      Number of user statistics to be configured in the chain
 *  @param[in]  ptrUserStatCfg
 *      Pointer to the user statistics configuration
 *  @param[out] ptrUserStatsIndex
 *      Pointer to the user statistics allocated index chain
 *  @param[out] errCode
 *      Error code populated by the API
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -  <0
 */
int32_t Netfp_createUserStats
(
    Netfp_ServerMCB*    ptrNetfpServer,
    uint32_t            numUserStats,
    Netfp_UserStatsCfg* ptrUserStatCfg,
    uint16_t*           ptrUserStatsIndex,
    int32_t*            errCode
)
{
    uint32_t                        index;
    uint16_t                        allocatedIndex;
    int32_t                         intErrCode;
    paUsrStatsCounterEntryConfig_t  paUsrStatsTbl[NETFP_MAX_USR_STATS_PER_RULE];

    /* Initialize the PA User Stats counter configuration table */
    memset ((void *)&paUsrStatsTbl, 0, sizeof(paUsrStatsCounterEntryConfig_t));

    /* Cycle through the User Statistics configuration blocks */
    for (index = 0; index < numUserStats; index++)
    {
        /* Allocate a unique user statistics index */
        if (Netfp_allocPaUserStatsIndex(ptrNetfpServer, ptrUserStatCfg->userStatsLen, &allocatedIndex, errCode) < 0)
        {
            /* Error: Unable to allocate the user stats index. We need to cleanup after ourselves. Cycle
             * through all the allocated user stat index and free them up. */
            while (index != 0)
            {
                index = index - 1;
                Netfp_freePaUserStatsIndex (ptrNetfpServer, *(ptrUserStatsIndex + index), &intErrCode);
            }
            return -1;
        }

        /* Populate the response */
        *(ptrUserStatsIndex + index) = allocatedIndex;

        /* Prepare the user stats counter entry configuration */
        paUsrStatsTbl[index].cntIndex = allocatedIndex;
        paUsrStatsTbl[index].cntType  = ptrUserStatCfg->userStatsType;
        paUsrStatsTbl[index].cntLnk   = pa_USR_STATS_LNK_END;

        /* Get the next configuration block: */
        ptrUserStatCfg++;
    }

    /* All the user statistics have been allocated; we now properly link up the entries */
    for (index = 1; index < numUserStats; index++)
        paUsrStatsTbl[index - 1].cntLnk = paUsrStatsTbl[index].cntIndex;

    /* Configure the NETCP */
    if (Netfp_configPaUserStats(ptrNetfpServer, numUserStats, &paUsrStatsTbl[0], errCode) < 0 )
    {
        /* Error: Unable to configure the PA with the user statistics configuration. Cycle through
         * and cleanup all the allocated user statistics. */
        while (index != 0)
        {
            index = index - 1;
            Netfp_freePaUserStatsIndex (ptrNetfpServer, *(ptrUserStatsIndex + index), &intErrCode);
        }
        return -1;
    }

    /* Debug Message: */
    for (index = 0; index < numUserStats; index++)
       Netfp_logMsg (ptrNetfpServer, Netfp_LogLevel_DEBUG, "Debug: Allocated UserStatIndex %d\n", *(ptrUserStatsIndex + index));

    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to delete a user statistics chain
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  numUserStats
 *      Number of user statistics to be configured in the chain
 *  @param[in] ptrUserStatsIndex
 *      Pointer to the user statistics allocated index to be cleaned up
 *  @param[out] errCode
 *      Error code populated by the API
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -  <0
 */
int32_t Netfp_deleteUserStats
(
    Netfp_ServerMCB*    ptrNetfpServer,
    uint32_t            numUserStats,
    uint16_t*           ptrUserStatsIndex,
    int32_t*            errCode
)
{
    uint32_t    index;

    /* Cycle through the User Statistics configuration blocks */
    for (index = 0; index < numUserStats; index++)
    {
        /* Debug Message: */
        Netfp_logMsg (ptrNetfpServer, Netfp_LogLevel_DEBUG, "Debug: Freeing UserStatIndex %d\n", *(ptrUserStatsIndex + index));

        /* Clear the user statistics */
        if (Netfp_clearUserStats (ptrNetfpServer,  *(ptrUserStatsIndex + index), errCode) < 0)
            return -1;

        /* Free the user statistics index. */
        if (Netfp_freePaUserStatsIndex (ptrNetfpServer, *(ptrUserStatsIndex + index), errCode) < 0)
            return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to register the NETFP PA Core services
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
int32_t Netfp_registerPAServices(Josh_NodeHandle nodeHandle)
{
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Netfp_configureReassembly);
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Netfp_configureGTPUControlMessage);
    return 0;
}
