/**
 *   @file  netfp_ethRuleMgmt.c
 *
 *   @brief
 *      The file implements the NETFP Ethernet Rule management module.
 *      The module is responsible for programing LUT 1-0 with given
 *      configuration.
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
#include <string.h>

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

/**********************************************************************
 ************************* Local Definitions **************************
 **********************************************************************/

/**
 *  @b Description
 *  @n
 *      The function is used to allocate LUT1-0 index with given region Name.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  rmRegionName
 *      Name of the LUT1-0 entry region
 *  @param[out]  ptrIndex
 *      Pointer to hold the allocated LUT1-0 index
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
static int32_t Netfp_allocEthLUTIndex
(
    Netfp_ServerMCB*    ptrNetfpServer,
    const char*         rmRegionName,
    uint32_t*           ptrIndex,
    int32_t*            errCode
)
{
    /* Allocate LUT1-0 index from region specified by rmRegionName. Resources are pre-allocated in dts file */
    if ( Resmgr_allocCustomResource(ptrNetfpServer->cfg.sysRMHandle, rmRegionName, 1, ptrIndex, errCode) < 0)
        return -1;
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to free LUT1-0 index with given region Name.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  rmRegionName
 *      Name of the LUT1-0 entry region
 *  @param[in]  index
 *      LUT1-0 index to be freed
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
static int32_t Netfp_freeEthLUTIndex
(
    Netfp_ServerMCB*    ptrNetfpServer,
    const char*         rmRegionName,
    int32_t             index,
    int32_t*            errCode
)
{
    /* Free LUT1-0 index to region specified by rmRegionName. */
    if (Resmgr_freeCustomResource(ptrNetfpServer->cfg.sysRMHandle, rmRegionName, 1, index, errCode) < 0)
        return -1;
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to add Ethernet rule in LUT 1-0 to enable
 *      cascading or forwarking traffic from a given host through the
 *      NETCP subsystem.
 *
 *  @param[in]  serverHandle
 *      Handle to the NETFP Server
 *  @param[in]  ptrEthRuleCfg
 *      pointer to configuration of Ethernet Rule to be added
 *  @param[out] errCode
 *      Error Code populated only on error else ignore
 *
 *  \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   Handle to the cascading configuration
 *  @retval
 *      Error   -   <0
 */
Netfp_EthRuleHandle Netfp_addEthRule
(
    Netfp_ServerHandle    serverHandle,
    Netfp_EthRuleCfg*     ptrEthRuleCfg,
    int32_t*              errCode
)
{
    Netfp_ServerMCB*                ptrNetfpServer;
    Netfp_EthRuleInfo*              ptrEthRuleInfo;
    uint8_t                         anyMacAddr[6];
    paEthInfo2_t                    ethInfo;
    paRouteInfo2_t                  routeInfo;
    paRouteInfo2_t                  failInfo;
    int32_t                         intErr;
    paCmdInfo_t                     paCmd;

    /* Sanity Check: Validate the arguments */
    if ((serverHandle == NULL) || (ptrEthRuleCfg == NULL))
    {
        *errCode = NETFP_EINVAL;
        return NULL;
    }

    /* Get server MCB */
    ptrNetfpServer = (Netfp_ServerMCB *)serverHandle;

    /* Save Interface based flow and Queue on server running within netfp master */
    ptrNetfpServer->interfaceBaseQueue   = ptrEthRuleCfg->baseQueue;
    ptrNetfpServer->interfaceBaseFlowId  = ptrEthRuleCfg->baseFlow;

    /* Sanity check for destination */
    if ((ptrEthRuleCfg->dstType == Netfp_EthRuleDst_EMAC) && (ptrEthRuleCfg->outPort == 0))
    {
        *errCode = NETFP_EINVAL;
        return NULL;
    }

    /* Sanity check of user stats configuration */
    if (ptrEthRuleCfg->numUserStats > NETFP_MAX_USR_STATS_PER_RULE)
    {
        *errCode = NETFP_EINVAL;
        return NULL;
    }

    /* Allocate memory to hold the Ethernet rule  info */
    ptrEthRuleInfo = (Netfp_EthRuleInfo *)ptrNetfpServer->cfg.malloc (sizeof (Netfp_EthRuleInfo), 0);
    if (ptrEthRuleInfo == NULL)
    {
        *errCode = NETFP_ENOMEM;
        return NULL;
    }

    /* Initialize the Ethernet Rule Info structure */
    memset ((void *)ptrEthRuleInfo, 0, sizeof (Netfp_EthRuleInfo));

    /* Save the configuration */
    memcpy ((void *)&ptrEthRuleInfo->cfg, ptrEthRuleCfg, sizeof (Netfp_EthRuleCfg));

    /********************************************************************
     ************** Programming LUT1-0 for the Ethernet Rule ************
     ********************************************************************/

    /* Prepare any MAC address */
    memset(&anyMacAddr[0], 0, sizeof(anyMacAddr));

    /* Initialize ethInfo */
    memset (&ethInfo, 0, sizeof(paEthInfo2_t));

    /* Setting up Source MAC */
    if (memcmp((void *)&anyMacAddr[0], &ptrEthRuleCfg->srcMacAddress[0],sizeof(paMacAddr_t)) != 0 )
    {
        /* Valid SRC MAC is configured */
        memcpy ((void *)&ethInfo.src, (void*)&ptrEthRuleCfg->srcMacAddress[0], sizeof(paMacAddr_t));
        ethInfo.validBitMap = pa_ETH_INFO_VALID_SRC;
    }

    /* Setting up dstination MAC */
    if (memcmp((void *)&anyMacAddr[0], &ptrEthRuleCfg->dstMacAddress[0],sizeof(paMacAddr_t)) != 0 )
    {
        /* Valid SRC MAC is configured */
        memcpy ((void *)&ethInfo.dst, (void*)&ptrEthRuleCfg->dstMacAddress[0], sizeof(paMacAddr_t));
        ethInfo.validBitMap  |= pa_ETH_INFO_VALID_DST;
    }

    /* Configure the ethernet type only if one was specified */
    if (ptrEthRuleCfg->ethType != 0)
    {
        ethInfo.ethertype = ptrEthRuleCfg->ethType;
        ethInfo.validBitMap  |= pa_ETH_INFO_VALID_ETHERTYPE;
    }

    /* Setting up VLAN id only if it is anyVlanId = 0
     * For "any" vlan id, validBitMap should be cleared */
    if (ptrEthRuleCfg->anyVlanId == 0)
    {
        ethInfo.vlan = ptrEthRuleCfg->vlanId;
        ethInfo.validBitMap  |= pa_ETH_INFO_VALID_VLAN;
    }

    /* Setting up inport */
    if (ptrEthRuleCfg->inPort != 0)
    {
        ethInfo.inport = ptrEthRuleCfg->inPort;
        ethInfo.validBitMap  |= pa_ETH_INFO_VALID_INPORT;
    }

    /* Setup fail info for all traffic FROM cascaded host. */
    memset(&failInfo, 0, sizeof(paRouteInfo2_t));

    /* Setup the fail information to pass all packets to Linux */
    if(Netfp_setupDefaultFailInfo(ptrNetfpServer,&failInfo) < 0)
    {
        *errCode = NETFP_EINTERNAL;
        ptrNetfpServer->cfg.free (ptrEthRuleInfo, sizeof (Netfp_EthRuleInfo));
        return NULL;
    }

    /* Allocating LUT Entry Index from LUT1-0 pool */
    if (Netfp_allocEthLUTIndex(ptrNetfpServer, &ptrEthRuleInfo->cfg.rmRegionName[0], &ptrEthRuleInfo->lutIndex, errCode) < 0)
    {
        /* Error: No free index available */
        ptrNetfpServer->cfg.free (ptrEthRuleInfo, sizeof (Netfp_EthRuleInfo));
        return NULL;
    }

    /* Initialize the routing information.  */
    memset ((void *)&routeInfo, 0, sizeof (paRouteInfo2_t));

    /* Do we need to add user statistics to the Ethernet rule? */
    if (ptrEthRuleCfg->numUserStats)
    {
        /* Create the user statistics: */
        if (Netfp_createUserStats (ptrNetfpServer, ptrEthRuleCfg->numUserStats, &ptrEthRuleCfg->userStatsCfg[0],
                                   &ptrEthRuleInfo->userStatsIndex[0], errCode) < 0)
        {
            /* Error: Unable to configure the user statistics. Clean up. */
            Netfp_freeEthLUTIndex(ptrNetfpServer, ptrEthRuleInfo->cfg.rmRegionName, ptrEthRuleInfo->lutIndex, &intErr);
            ptrNetfpServer->cfg.free (ptrEthRuleInfo, sizeof (Netfp_EthRuleInfo));
            return NULL;
        }

        /* Initialize paCmd */
        memset((void *)&paCmd, 0, sizeof(paCmdInfo_t));

        /* Set User stats command */
        paCmd.cmd                   = pa_CMD_USR_STATS;
        paCmd.params.usrStats.index = ptrEthRuleInfo->userStatsIndex[0];
        routeInfo.pCmd              = &paCmd;
        routeInfo.validBitMap       = pa_ROUTE_INFO_VALID_PCMD;
    }

    /* Populate the route information on the basis of the destination type: */
    switch (ptrEthRuleCfg->dstType)
    {
        case Netfp_EthRuleDst_EMAC:
        {
            /* Setup next route info */
            routeInfo.mRouteIndex = pa_NO_MULTI_ROUTE;

            /* Is QOS Enabled? */
            if(ptrEthRuleCfg->enableQos)
            {
                /* YES: Pass the packet to the QOS Shaping queues */
                routeInfo.dest          = pa_DEST_HOST;
                routeInfo.flowId        = ptrEthRuleCfg->qosBaseFlowId;
                routeInfo.queue         = ptrEthRuleCfg->qosBaseQueue;
                routeInfo.priorityType  = pa_ROUTE_EQoS_MODE;
                routeInfo.validBitMap   |= pa_ROUTE_INFO_VALID_PRIORITY_TYPE;
            }
            else
            {
                /* NO: Directly pass the packet to the egress interface */
                routeInfo.dest  = pa_DEST_EMAC;
                routeInfo.queue = 0;
            }

            /* Destination is set to Egress interface , send to the EMAC port */
            routeInfo.pktType_emacCtrl = ptrEthRuleCfg->outPort;
            routeInfo.validBitMap      |= pa_ROUTE_INFO_VALID_PKTTYPE_EMAC;

            /* For K2H and K2K, CRC generation needs to be disabled */
#if (defined (DEVICE_K2H) || defined (DEVICE_K2K))
            routeInfo.pktType_emacCtrl |= pa_EMAC_CTRL_CRC_DISABLE;
#endif
            break;
        }
        case Netfp_EthRuleDst_CONTINUE:
        {
            /* Packets will be sent to PDSP1 for continuing parsing.
             * Scenarios: 1. Linux bridging */
            routeInfo.dest = pa_DEST_CONTINUE_PARSE_LUT1;
            break;
        }
        case Netfp_EthRuleDst_HOST:
        {
            /* Destination is HOST :packets will be sent to interface based routing destination
             * Setup interface based routing to direct the packet to Linux */
            routeInfo.dest          = pa_DEST_HOST;
            routeInfo.flowId        = ptrEthRuleCfg->baseFlow;
            routeInfo.queue         = ptrEthRuleCfg->baseQueue;
            routeInfo.priorityType  = pa_ROUTE_INTF_W_FLOW;
            routeInfo.validBitMap   |= pa_ROUTE_INFO_VALID_PRIORITY_TYPE;
            routeInfo.mRouteIndex   = pa_NO_MULTI_ROUTE;
            break;
        }
//fzm-->
        case Netfp_EthRuleDst_DISCARD:
        {
            routeInfo.dest          = pa_DEST_DISCARD;
            break;
        }
//fzm<--
        default:
        {
            /* Error: Invalid destination type */
            *errCode = NETFP_EINVAL;
            return NULL;
        }
    }

    /* Add the Ethernet Rule */
    if (Netfp_paAddEth (ptrNetfpServer, &ethInfo, &routeInfo, &failInfo, &ptrEthRuleInfo->ethHandle,
                        ptrEthRuleInfo->lutIndex, errCode) < 0)
    {
        /* Error: Unable to add the Ethernet rule. Cleanup */
        Netfp_freeEthLUTIndex(ptrNetfpServer, ptrEthRuleInfo->cfg.rmRegionName, ptrEthRuleInfo->lutIndex, &intErr);
        Netfp_deleteUserStats (ptrNetfpServer, ptrEthRuleInfo->cfg.numUserStats, &ptrEthRuleInfo->userStatsIndex[0], &intErr);
        ptrNetfpServer->cfg.free (ptrEthRuleInfo, sizeof (Netfp_EthRuleInfo));
        return NULL;
    }

    /* Add the Ethernet route info into list maintained on the NetFP server */
    Netfp_listAdd ((Netfp_ListNode**)&ptrNetfpServer->ptrEtherRuleList, (Netfp_ListNode*)ptrEthRuleInfo);
    return (Netfp_EthRuleHandle)(ptrEthRuleInfo);
}

/**
 *  @b Description
 *  @n
 *      The function is invoked from Netfp Server to delete an Ethernet rule from
 *      NETCP subsystem .
 *
 *  @param[in]  serverHandle
 *      Pointer to the NETFP Server
 *  @param[in]  ethRuleHandle
 *      pointer to Ethernet rule handle to be deleted
 *  @param[out] errCode
 *      Error Code populated only on error else ignore
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0.
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_delEthRule
(
    Netfp_ServerHandle    serverHandle,
    Netfp_EthRuleHandle   ethRuleHandle,
    int32_t*              errCode
)
{
    Netfp_ServerMCB*     ptrNetfpServer;
    Netfp_EthRuleInfo*   ptrEthRuleInfo;
    int32_t              retVal;

    /* Sanity Check: Ensure that this is a valid handle */
    if ((ethRuleHandle == NULL) || (serverHandle == NULL) )
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Get Netfp Server MCB: */
    ptrNetfpServer = (Netfp_ServerMCB *)serverHandle;

    /* Get the Ethernet rule information: */
    ptrEthRuleInfo = (Netfp_EthRuleInfo *)ethRuleHandle;

    /* Delete Ethernet Rule entry from LUT1-0 */
    retVal = Netfp_paDelEth (ptrNetfpServer, ptrEthRuleInfo->ethHandle);
    if (retVal < 0)
    {
        *errCode = retVal;
        return -1;
    }

    /* Free LUT1-0 index */
    if (Netfp_freeEthLUTIndex(ptrNetfpServer, ptrEthRuleInfo->cfg.rmRegionName, ptrEthRuleInfo->lutIndex, errCode) < 0)
        return -1;

    /* Delete the user statistics */
    if (Netfp_deleteUserStats (ptrNetfpServer, ptrEthRuleInfo->cfg.numUserStats, &ptrEthRuleInfo->userStatsIndex[0], errCode) < 0)
        return -1;

    /* Remove the rule from the list  */
    Netfp_listRemoveNode ((Netfp_ListNode**)&ptrNetfpServer->ptrEtherRuleList, (Netfp_ListNode*)ptrEthRuleInfo);
    ptrNetfpServer->cfg.free (ptrEthRuleInfo, sizeof (Netfp_EthRuleInfo));
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is invoked to get the statistics associated with the
 *      Ethernet rule.
 *
 *  @param[in]  serverHandle
 *      Pointer to the NETFP Server
 *  @param[in]  ethRuleHandle
 *      Pointer to Ethernet rule information
 *  @param[out] ptrUserStats
 *      Pointer to the user statistics associated with the rule.
 *  @param[out] errCode
 *      Error Code populated only on error else ignore
 *
 *  \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   0.
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_getEthRuleStats
(
    Netfp_ServerHandle      serverHandle,
    Netfp_EthRuleHandle     ethRuleHandle,
    Netfp_UserStats*        ptrUserStats,
    int32_t*                errCode
)
{
    uint32_t                index;
    uint32_t                userStatsIdx;
    Netfp_UserStatsLen      userStatsLen;
    Netfp_ServerMCB*        ptrNetfpServer;
    Netfp_EthRuleInfo*      ptrEthRuleInfo;

    /* Get the NETFP Server MCB */
    ptrNetfpServer = (Netfp_ServerMCB*)serverHandle;
    ptrEthRuleInfo = (Netfp_EthRuleInfo*)ethRuleHandle;

    /* Sanity Check: Validate the arguments */
    if ((ptrNetfpServer == NULL) || (ptrEthRuleInfo == NULL))
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Get the global user statistics from the NETCP */
    if (Netfp_getUserStats (ptrNetfpServer, ptrNetfpServer->ptrUserStats, errCode) < 0)
        return -1;

    /* Cycle through and find what we are interested in  */
    for (index = 0; index < ptrEthRuleInfo->cfg.numUserStats; index ++)
    {
        /* Get the user stats based on given stats Id */
        userStatsIdx = ptrEthRuleInfo->userStatsIndex[index];
        userStatsLen = ptrEthRuleInfo->cfg.userStatsCfg[index].userStatsLen;

        if (userStatsLen == Netfp_UserStatsLen_64b)
            ptrUserStats->userStats[index] = ptrNetfpServer->ptrUserStats->count64[userStatsIdx];
        else
            ptrUserStats->userStats[index] = ptrNetfpServer->ptrUserStats->count32[userStatsIdx - ptrNetfpServer->userStatCfg.num64bUserStats];
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Utility function that converts the destination of an Ethernet rule to a string.
 *
 *  @param[in]  destination
 *      Destination of the Ethernet rule
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Reason
 */
static const char* Netfp_getEthRuleDstString(Netfp_EthRuleDst destination)
{
    switch (destination)
    {
        case Netfp_EthRuleDst_EMAC:
        {
            return "EMAC";
        }
        case Netfp_EthRuleDst_CONTINUE:
        {
            return "Continue Parsing";
        }
        case Netfp_EthRuleDst_HOST:
        {
            return "Host";
        }
        default:
        {
            return "Unknown";
        }
    }
}

/**
 *  @b Description
 *  @n
 *      The function is a utility function which is used to display the active ethernet
 *      rules which are present in the system.
 *
 *  @param[in]  serverHandle
 *      NETFP Server Handle
 *
 * \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Number of ethernet rules which have been displayed
 */
int32_t Netfp_displayEthRule(Netfp_ServerHandle serverHandle)
{
    Netfp_EthRuleInfo*      ptrEthRuleInfo;
    Netfp_ServerMCB*        ptrNetfpServer;
    int32_t                 count = 0;
    Netfp_UserStats         ethRuleStats;
    int32_t                 errCode;
    uint32_t                index;

    /* Get the server handle. */
    ptrNetfpServer = (Netfp_ServerMCB*)serverHandle;
    if (ptrNetfpServer == NULL)
        return 0;

    /* Get the Ethernet Rule information */
    ptrEthRuleInfo = (Netfp_EthRuleInfo*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrEtherRuleList);

//fzm - use Netfp_dumpMsg to capture this with the USR1 signal
    /* Display the banner. */
    Netfp_dumpMsg  (ptrNetfpServer, Netfp_LogLevel_INFO, "**********************************************************\n");
    Netfp_dumpMsg  (ptrNetfpServer, Netfp_LogLevel_INFO, "Ethernet Rules\n");

    /* Cycle through all the Ethernet rules and display them */
    while (ptrEthRuleInfo != NULL)
    {
        /* Display the Ethernet Rule Info */
        Netfp_dumpMsg  (ptrNetfpServer, Netfp_LogLevel_INFO,
                      "%2d:%p:%s:index=%d [%s] -> [%s], VLAN: %d, EtherType: 0x%x, Dst: %s In Port: %d\n",
                      count, ptrEthRuleInfo,
                      &ptrEthRuleInfo->cfg.rmRegionName[0], ptrEthRuleInfo->lutIndex,
                      (ptrEthRuleInfo->cfg.ingressIfName[0] ? ptrEthRuleInfo->cfg.ingressIfName : "ANY"),
                      (ptrEthRuleInfo->cfg.egressIfName[0] ? ptrEthRuleInfo->cfg.egressIfName : "ANY"),
                      ptrEthRuleInfo->cfg.vlanId, ptrEthRuleInfo->cfg.ethType,
                      Netfp_getEthRuleDstString(ptrEthRuleInfo->cfg.dstType),
                      ptrEthRuleInfo->cfg.inPort);
        Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,
                      "       SrcMAC %02x:%02x:%02x:%02x:%02x:%02x DstMAC %02x:%02x:%02x:%02x:%02x:%02x\n",
                      ptrEthRuleInfo->cfg.srcMacAddress[0], ptrEthRuleInfo->cfg.srcMacAddress[1],
                      ptrEthRuleInfo->cfg.srcMacAddress[2], ptrEthRuleInfo->cfg.srcMacAddress[3],
                      ptrEthRuleInfo->cfg.srcMacAddress[4], ptrEthRuleInfo->cfg.srcMacAddress[5],
                      ptrEthRuleInfo->cfg.dstMacAddress[0], ptrEthRuleInfo->cfg.dstMacAddress[1],
                      ptrEthRuleInfo->cfg.dstMacAddress[2], ptrEthRuleInfo->cfg.dstMacAddress[3],
                      ptrEthRuleInfo->cfg.dstMacAddress[4], ptrEthRuleInfo->cfg.dstMacAddress[5]);
        Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,
                      "       Out Port: %d Base Flow: %d Base Queue: %d\n",
                      ptrEthRuleInfo->cfg.outPort, ptrEthRuleInfo->cfg.qosBaseFlowId,
                      ptrEthRuleInfo->cfg.qosBaseQueue);

        /* Initialize the user statistics */
        memset ((void *)&ethRuleStats, 0, sizeof(Netfp_UserStats));

        /* Get the user statistics associated with the Ethernet rule */
        if (Netfp_getEthRuleStats (ptrNetfpServer, ptrEthRuleInfo, &ethRuleStats, &errCode) < 0)
        {
            /* Error: Unable to get the user statistics */
            Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_ERROR, "Error: Unable to get the user stats [Error code %d]\n", errCode);
        }
        else
        {
            /* Display the user statistics: */
            for (index = 0; index < ptrEthRuleInfo->cfg.numUserStats; index ++)
            {
                Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,
                              "       User Stats Index: %d Counter: %lld\n",
                              ptrEthRuleInfo->userStatsIndex[index], ethRuleStats.userStats[index]);
            }
        }

        /* Increment the counter */
        count = count + 1;

        /* Get the next ethernet rule */
        ptrEthRuleInfo = (Netfp_EthRuleInfo*)Netfp_listGetNext ((Netfp_ListNode*)ptrEthRuleInfo);
    }
    return count;
}

