/**
 *   @file  netfp_proxyServer.c
 *
 *   @brief
 *      The file implements the interface between the NETFP Proxy and
 *      Server
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
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>

/* NETFP Include Files. */
#include <ti/runtime/netfp/netfp.h>
#include <ti/runtime/netfp/include/netfp_internal.h>
#include <ti/runtime/netfp/include/netfp_net.h>
#include <ti/runtime/netfp/include/netfp_ipv6.h>

/* For Debugging only. */
#ifdef __ARMv7
#include <ti/apps/netfp_config/include/NetFP_System_printf.h> //fzm
#else
#include <xdc/runtime/System.h>
#endif

/**************************************************************************
 ******************** NETFP Proxy-Server Functions ************************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      This function is used to find outbound fast path matching the request identifier
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  requestId
 *      Request identifier
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not NULL -  Pointer to the outbound fast path
 *  @retval
 *      NULL     -  No matching fast path found
 */
static Netfp_OutboundFastPath* Netfp_findResolvedFP
(
    Netfp_ServerMCB*        ptrNetfpServer,
    uint32_t                requestId
)
{
    Netfp_OutboundFastPath*     ptrOutboundFastPath;

    /* Cycle through all the registered fast paths. */
    ptrOutboundFastPath = (Netfp_OutboundFastPath*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrOutboundFPList);
    while (ptrOutboundFastPath != NULL)
    {
        /* Do we have a match? */
        if (ptrOutboundFastPath->requestId == requestId)
            return ptrOutboundFastPath;

        /* Get the next outbound fast path */
        ptrOutboundFastPath = (Netfp_OutboundFastPath*)Netfp_listGetNext ((Netfp_ListNode*)ptrOutboundFastPath);
    }
    /* Control comes here there was no match */
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      This function is used to find the outbound security association matching the request identifier
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  requestId
 *      Request identifier
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not NULL -  Pointer to the outbound security association
 *  @retval
 *      NULL     -  No matching security association
 */
static Netfp_IPSecChannel* Netfp_findResolvedSA
(
    Netfp_ServerMCB*        ptrNetfpServer,
    uint32_t                requestId
)
{
    Netfp_IPSecChannel* ptrOutboundIPSecChannel;

    /* Cycle through all the registered outbound IPSEC channels */
    ptrOutboundIPSecChannel = (Netfp_IPSecChannel*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrIPSecOutboundChannels);
    while (ptrOutboundIPSecChannel != NULL)
    {
        /* Do we have a match? */
        if (ptrOutboundIPSecChannel->requestId == requestId)
            return ptrOutboundIPSecChannel;

        /* Get the next outbound security channel. */
        ptrOutboundIPSecChannel = (Netfp_IPSecChannel*)Netfp_listGetNext ((Netfp_ListNode*)ptrOutboundIPSecChannel);
    }
    /* Control comes here there was no match */
    return NULL;
}

int32_t Netfp_populateServerToProxyNode
(
   Netfp_ServerMCB*         ptrNetfpServer,
   Netfp_ProxyServerInfo*   ptrProxyServerInfo,
   uint32_t*                ptrRequestId
)
{
    Netfp_ServerToProxyNode*  ptrServerToProxyNode = ptrNetfpServer->cfg.malloc (sizeof(Netfp_ServerToProxyNode), 0);
    if (ptrServerToProxyNode == NULL)
        return NETFP_ENOMEM;

    /* Initialize the allocated memory */
    memset ((void*)ptrServerToProxyNode, 0, sizeof(Netfp_ServerToProxyNode));

    /* Populate the resolve route block for ProxyServer list*/
    memcpy ((void*)&ptrServerToProxyNode->proxyServerInfo, (void*)ptrProxyServerInfo, sizeof(Netfp_ProxyServerInfo));
    ptrServerToProxyNode->requestId = (uint32_t)ptrServerToProxyNode;

    ptrProxyServerInfo->requestId = ptrServerToProxyNode->requestId;

    *ptrRequestId = ptrServerToProxyNode->requestId;

    return 0;
}

void Netfp_cleanupRequest(Netfp_ServerMCB* ptrNetfpServer, Netfp_ProxyServerBulkMsg* ptrProxyServerBulkMsg)
{
    // Free all ProxyServerNodes
    for (int i = 0 ; i < ptrProxyServerBulkMsg->numberOfEntries ; i++ )
    {
        if (ptrProxyServerBulkMsg->proxyServerInfo[i].requestId)
            ptrNetfpServer->cfg.free((void *)ptrProxyServerBulkMsg->proxyServerInfo[i].requestId, sizeof(Netfp_ServerToProxyNode));

        ptrProxyServerBulkMsg->proxyServerInfo[i].requestId = 0;
    }
}

/**
 *  @b Description
 *  @n
 *      This is the actual function where the route recomputation is implemented.
 *      The function parses through the recomputation list and issues the route
 *      resolution requests to the NETFP Proxy in bulk messages.
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
int32_t Netfp_processRouteRecomputation
(
    Netfp_ServerMCB*    ptrNetfpServer,
    int32_t*            errCode
)
{
    Netfp_RecomputeRoute*       ptrRecomputeRoute;
    Netfp_RecomputeRoute*       ptrRecomputeRouteBulkMessageList = NULL;
    int32_t                     numRoutesProcessed = 0;
    static Netfp_ProxyServerBulkMsg proxyServerBulkMsg;
    Netfp_ProxyServerInfo*      ptrProxyServerInfo;

    /* Is the route recomputation in progress? If not then there is nothing to be done here */
    if (ptrNetfpServer->isRouteRecomputationInProgress == 0)
        return 0;

    /* Cycle through all the entries which require a recomputation */
    ptrRecomputeRoute = (Netfp_RecomputeRoute*)Netfp_listRemove ((Netfp_ListNode**)&ptrNetfpServer->ptrRouteRecomputationList);

    while (ptrRecomputeRoute)
    {
        proxyServerBulkMsg.numberOfEntries = 0;

        /* Create bulk message of max BULK_INFO_SIZE route recomputation requests */
        while (ptrRecomputeRoute && proxyServerBulkMsg.numberOfEntries < BULK_INFO_MAX_ENTRIES)
        {
            /* Remember the message, when operation passes it will be removed, otherwise all RecomputeRoute requests
               will be restored onto ptrRouteRecomputationList */
            Netfp_listAdd((Netfp_ListNode**)&ptrRecomputeRouteBulkMessageList, (Netfp_ListNode*)ptrRecomputeRoute);

            ptrProxyServerInfo = &proxyServerBulkMsg.proxyServerInfo[proxyServerBulkMsg.numberOfEntries];

            /* Initialize the proxy server information */
            memset ((void *)ptrProxyServerInfo, 0, sizeof(Netfp_ProxyServerInfo));

            /* Populate the informational block: */
            ptrProxyServerInfo->opType = Netfp_ProxyServerOp_REQUEST;
            ptrProxyServerInfo->u.req.startMonitor = 1;
            memcpy ((void *)&ptrProxyServerInfo->dstIP, (void*)&ptrRecomputeRoute->dstIPAddress, sizeof(Netfp_IPAddr));
            memcpy ((void *)&ptrProxyServerInfo->srcIP, (void*)&ptrRecomputeRoute->srcIPAddress, sizeof(Netfp_IPAddr));

            *errCode = Netfp_populateServerToProxyNode(ptrNetfpServer, ptrProxyServerInfo, ptrRecomputeRoute->ptrResolvedId);

            if (*errCode < 0)
            {
                Netfp_cleanupRequest(ptrNetfpServer, &proxyServerBulkMsg);

                return numRoutesProcessed;
            }

            proxyServerBulkMsg.numberOfEntries++;

            /* Get the next node if allowed */
            if (proxyServerBulkMsg.numberOfEntries < BULK_INFO_MAX_ENTRIES)
                ptrRecomputeRoute = (Netfp_RecomputeRoute*)Netfp_listRemove ((Netfp_ListNode**)&ptrNetfpServer->ptrRouteRecomputationList);
        }

        /* Send a message to the NETFP Proxy to start monitoring: */
        *errCode = Netfp_sendProxyRequest (ptrNetfpServer, &proxyServerBulkMsg);
        if (*errCode < 0)
        {
             /* Always free preallocated ProxyServerNodes */
              Netfp_cleanupRequest(ptrNetfpServer, &proxyServerBulkMsg);

            /* Error Route resolution failed. We need to determine the reason for failure. It could have simply ran out of
             * JOBS to send to the NETFP Proxy. This can happen if there are too many fast paths and we are trying to perfom
             * route resolution at once. */
            if (*errCode == NETFP_EJOSH || *errCode == JOSH_EBUSY)
            {
                /* Ran out of jobs. We need to enqueue back the packet and try again */
                ptrRecomputeRoute = (Netfp_RecomputeRoute*)Netfp_listRemove ((Netfp_ListNode**)&ptrRecomputeRouteBulkMessageList);

                while (ptrRecomputeRoute)
                {
                    /* Mark the request as not started */
                    *ptrRecomputeRoute->ptrResolvedId = 0;

                    /* Put back on ptrRouteRecomputationList to try later */
                    Netfp_listAdd((Netfp_ListNode**)&ptrNetfpServer->ptrRouteRecomputationList, (Netfp_ListNode*)ptrRecomputeRoute);

                    ptrRecomputeRoute = (Netfp_RecomputeRoute*)Netfp_listRemove ((Netfp_ListNode**)&ptrRecomputeRouteBulkMessageList);
                }
            }
            else
                Netfp_logMsg (ptrNetfpServer, Netfp_LogLevel_ERROR, "Netfp_sendProxyRequest failed %d\n", *errCode);

            return numRoutesProcessed;
        }

        /* Increment the counter which tracks the number of routes processed */
        numRoutesProcessed += proxyServerBulkMsg.numberOfEntries;

        /* Remove the recompute routes from bulk message */
        ptrRecomputeRoute = (Netfp_RecomputeRoute*)Netfp_listRemove ((Netfp_ListNode**)&ptrRecomputeRouteBulkMessageList);

        while (ptrRecomputeRoute)
        {
            ptrNetfpServer->cfg.free (ptrRecomputeRoute, sizeof(Netfp_RecomputeRoute));

            ptrRecomputeRoute = (Netfp_RecomputeRoute*)Netfp_listRemove ((Netfp_ListNode**)&ptrRecomputeRouteBulkMessageList);
        }

        /* Get the next node */
        ptrRecomputeRoute = (Netfp_RecomputeRoute*)Netfp_listRemove ((Netfp_ListNode**)&ptrNetfpServer->ptrRouteRecomputationList);
    }

    /* Once we have submitted all the routes for recomputation; the proceess is complete */
    ptrNetfpServer->isRouteRecomputationInProgress = 0;

    return numRoutesProcessed;
}

/**
 *  @b Description
 *  @n
 *      The function is used to perform the route recalculation on the NETFP server
 *      In the function we simply prepare a list of all the routes which need to be
 *      recalculated.  The actual route recomputation is done outside the function.
 *      This is because this is a JOSH JOB and we need to return immediately; else
 *      this will block the NETFP Proxy which we need to perform the actual route
 *      resolution.
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
static int32_t _Netfp_recomputeRoutes
(
    Netfp_ServerMCB*    ptrNetfpServer,
    int32_t*            errCode
)
{
    Netfp_IPSecChannel*         ptrOutboundIPSecChannel;
    Netfp_RecomputeRoute*       ptrRecomputeRoute;
    Netfp_OutboundFastPath*     ptrOutboundFastPath;

    /* Flushing routes is only supported; if we have the NETFP Proxy service registered to handle the
     * route recalculation. */
    if (ptrNetfpServer->proxyClientBlock == NULL)
    {
        *errCode = NETFP_ENOTREADY;
        return -1;
    }

    /* Sanity Check: There is only 1 active flush is allowed at a time. */
    if (ptrNetfpServer->isRouteRecomputationInProgress == 1)
    {
        *errCode = NETFP_EINUSE;
        return -1;
    }

    /* Can we proceed with the actual route recomputation? Are there any active jobs between the PROXY and Server? */
    if (Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrPendingProxyServerList) != NULL)
    {
        /* YES. This implies that we need to hold off the route recomputation until all the jobs between the
         * PROXY and SERVER have been completed. We do not want to overwrite the request id and leak JOSH jobs. */
        *errCode = NETFP_EINUSE;
        return -1;
    }

    /* Set the flag to indicate that a route flush is starting: */
    ptrNetfpServer->isRouteRecomputationInProgress = 1;

    /*************************************************************************************************
     * Cycle through and perform the route recalculation for all the OUTBOUND security associations.
     *************************************************************************************************/
    ptrOutboundIPSecChannel = (Netfp_IPSecChannel*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrIPSecOutboundChannels);
    while (ptrOutboundIPSecChannel != NULL)
    {
        /* Was the outbound IPSEC channel using the manual route override mode? */
        if (ptrOutboundIPSecChannel->saCfg.ifHandle != NULL)
        {
            /* YES. Skip there is no reason to request the NETFP Proxy for this IPSEC channel */
            ptrOutboundIPSecChannel = (Netfp_IPSecChannel*)Netfp_listGetNext ((Netfp_ListNode*)ptrOutboundIPSecChannel);
            continue;
        }

        // fzm: if SA is stopped, recomputing its route would change the status to ACTIVE
        // which is not desired - FP will be re-created but it shouldn't
        if (ptrOutboundIPSecChannel->status == Netfp_Status_STOP)
        {
            Netfp_logMsg (ptrNetfpServer, Netfp_LogLevel_DEBUG, "Debug: Skipping recomputing of SA %p - STOPPED", ptrOutboundIPSecChannel);
            ptrOutboundIPSecChannel->recomputationDoneOnStopped = 1;

            ptrOutboundIPSecChannel = (Netfp_IPSecChannel*)Netfp_listGetNext ((Netfp_ListNode*)ptrOutboundIPSecChannel);
            continue;
        }

        //fzm
        if (ptrOutboundIPSecChannel->requestId != 0u)
        {
            Netfp_logMsg (ptrNetfpServer, Netfp_LogLevel_DEBUG, "Debug: Recomputing SA %p not needed, already being processed\n", ptrOutboundIPSecChannel);
            ptrOutboundIPSecChannel = (Netfp_IPSecChannel*)Netfp_listGetNext ((Netfp_ListNode*)ptrOutboundIPSecChannel);
            continue;
        }

        /* Allocate memory for the route recomputation: */
        ptrRecomputeRoute = (Netfp_RecomputeRoute*)ptrNetfpServer->cfg.malloc (sizeof(Netfp_RecomputeRoute), 0);
        if (ptrRecomputeRoute == NULL)
        {
            *errCode = NETFP_ENOMEM;
            return -1;
        }

        /* Initialize the allocated memory */
        memset ((void *)ptrRecomputeRoute, 0, sizeof(Netfp_RecomputeRoute));

        /* Populate the route recomputation node: */
        ptrRecomputeRoute->dstIPAddress  = ptrOutboundIPSecChannel->saCfg.dstIP;
        ptrRecomputeRoute->srcIPAddress  = ptrOutboundIPSecChannel->saCfg.srcIP;
        ptrRecomputeRoute->ptrResolvedId = &ptrOutboundIPSecChannel->requestId;

        /* Debug Message: */
        if (ptrOutboundIPSecChannel->saCfg.srcIP.ver == Netfp_IPVersion_IPV4)
        {
            Netfp_logMsg (ptrNetfpServer, Netfp_LogLevel_DEBUG, "Debug: Recomputing SA 0x%x %03d.%03d.%03d.%03d -> %03d.%03d.%03d.%03d\n",
                          ptrOutboundIPSecChannel, ptrOutboundIPSecChannel->saCfg.srcIP.addr.ipv4.u.a8[0],
                          ptrOutboundIPSecChannel->saCfg.srcIP.addr.ipv4.u.a8[1], ptrOutboundIPSecChannel->saCfg.srcIP.addr.ipv4.u.a8[2],
                          ptrOutboundIPSecChannel->saCfg.srcIP.addr.ipv4.u.a8[3], ptrOutboundIPSecChannel->saCfg.dstIP.addr.ipv4.u.a8[0],
                          ptrOutboundIPSecChannel->saCfg.dstIP.addr.ipv4.u.a8[1], ptrOutboundIPSecChannel->saCfg.dstIP.addr.ipv4.u.a8[2],
                          ptrOutboundIPSecChannel->saCfg.dstIP.addr.ipv4.u.a8[3]);
        }
        else
        {
            char    srcIP[40];
            char    dstIP[40];

            /* Convert the Src & Dst IP address of the SA into strings using the NETFP IPv6 Utility API */
            Netfp_convertIP6ToStr (ptrOutboundIPSecChannel->saCfg.srcIP.addr.ipv6, &srcIP[0]);
            Netfp_convertIP6ToStr (ptrOutboundIPSecChannel->saCfg.dstIP.addr.ipv6, &dstIP[0]);

            Netfp_logMsg (ptrNetfpServer, Netfp_LogLevel_DEBUG, "Debug: Recomputing SA 0x%x %s -> %s\n",
                          ptrOutboundIPSecChannel, srcIP, dstIP);
        }

        /* Register the node in the NETFP Server  */
        Netfp_listAdd ((Netfp_ListNode**)&ptrNetfpServer->ptrRouteRecomputationList, (Netfp_ListNode*)ptrRecomputeRoute);

        /* Get the next outbound security channel. */
        ptrOutboundIPSecChannel = (Netfp_IPSecChannel*)Netfp_listGetNext ((Netfp_ListNode*)ptrOutboundIPSecChannel);
    }

    /*************************************************************************************************
     * Cycle through and perform the route recalculation for all the NON-Secure OUTBOUND Fast Paths
     *************************************************************************************************/
    ptrOutboundFastPath = (Netfp_OutboundFastPath*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrOutboundFPList);
    while (ptrOutboundFastPath != NULL)
    {
        /* Is the outbound fast path secure? Secure fast paths inherit the routes from the outbound security association
         * and so they are not responsible for starting the route resolution procedure. */
        if (ptrOutboundFastPath->isSecure == 0)
        {
            /* NON Secure connection: Was the outbound fast path using the manual route override mode? */
            if (ptrOutboundFastPath->cfg.ifHandle != NULL)
            {
                /* YES. Skip there is no reason to request the NETFP Proxy for this outbound fast path */
                ptrOutboundFastPath = (Netfp_OutboundFastPath*)Netfp_listGetNext ((Netfp_ListNode*)ptrOutboundFastPath);
                continue;
            }

            //fzm
            if (ptrOutboundFastPath->requestId != 0u)
            {
                Netfp_logMsg (ptrNetfpServer, Netfp_LogLevel_DEBUG, "Debug: Recomputing FP %p not needed, already being processed\n", ptrOutboundFastPath);

                ptrOutboundFastPath = (Netfp_OutboundFastPath*)Netfp_listGetNext ((Netfp_ListNode*)ptrOutboundFastPath);
                continue;
            }

            /* Allocate memory for the route recomputation */
            ptrRecomputeRoute = (Netfp_RecomputeRoute*)ptrNetfpServer->cfg.malloc (sizeof(Netfp_RecomputeRoute), 0);
            if (ptrRecomputeRoute == NULL)
            {
                *errCode = NETFP_ENOMEM;
                return -1;
            }

            /* Initialize the allocated memory */
            memset ((void *)ptrRecomputeRoute, 0, sizeof(Netfp_RecomputeRoute));

            /* Populate the route recomputation node: */
            ptrRecomputeRoute->dstIPAddress  = ptrOutboundFastPath->cfg.dstIP;
            ptrRecomputeRoute->srcIPAddress  = ptrOutboundFastPath->cfg.srcIP;
            ptrRecomputeRoute->ptrResolvedId = &ptrOutboundFastPath->requestId;

            /* Display the fast path */
            if (ptrOutboundFastPath->cfg.srcIP.ver == Netfp_IPVersion_IPV4)
            {
                Netfp_logMsg (ptrNetfpServer, Netfp_LogLevel_DEBUG, "Debug: Recomputing FP 0x%x %03d.%03d.%03d.%03d -> %03d.%03d.%03d.%03d\n",
                              ptrOutboundFastPath, ptrOutboundFastPath->cfg.srcIP.addr.ipv4.u.a8[0],
                              ptrOutboundFastPath->cfg.srcIP.addr.ipv4.u.a8[1], ptrOutboundFastPath->cfg.srcIP.addr.ipv4.u.a8[2],
                              ptrOutboundFastPath->cfg.srcIP.addr.ipv4.u.a8[3], ptrOutboundFastPath->cfg.dstIP.addr.ipv4.u.a8[0],
                              ptrOutboundFastPath->cfg.dstIP.addr.ipv4.u.a8[1], ptrOutboundFastPath->cfg.dstIP.addr.ipv4.u.a8[2],
                              ptrOutboundFastPath->cfg.dstIP.addr.ipv4.u.a8[3]);
            }
            else
            {
                char    srcIP[40];
                char    dstIP[40];

                /* Convert the Src & Dst IP address of the SP into strings using the NETFP IPv6 Utility API */
                Netfp_convertIP6ToStr (ptrOutboundFastPath->cfg.srcIP.addr.ipv6, &srcIP[0]);
                Netfp_convertIP6ToStr (ptrOutboundFastPath->cfg.dstIP.addr.ipv6, &dstIP[0]);

                Netfp_logMsg (ptrNetfpServer, Netfp_LogLevel_DEBUG, "Debug: Recomputing FP 0x%x %s-> %s [0x%x]\n",
                              ptrOutboundFastPath, srcIP, dstIP);
            }

            /* Register the node in the NETFP Server  */
            Netfp_listAdd ((Netfp_ListNode**)&ptrNetfpServer->ptrRouteRecomputationList, (Netfp_ListNode*)ptrRecomputeRoute);
        }

        /* Get the next outbound fast path */
        ptrOutboundFastPath = (Netfp_OutboundFastPath*)Netfp_listGetNext ((Netfp_ListNode*)ptrOutboundFastPath);
    }

    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to recompute the routes for all outbound fast paths.
 *
 *  @param[in]  clientHandle
 *      Handle to the NETFP client
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
int32_t Netfp_recomputeRoutes(Netfp_ClientHandle clientHandle, int32_t* errCode)
{
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    uint32_t                result;
    int32_t                 jobId;
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
    jobHandle = Josh_findJobByAddress(ptrNetfpClient->joshHandle, (Josh_JobProtype)_Netfp_recomputeRoutes);
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
     *  int32_t _Netfp_recomputeRoutes
     *  (
     *  Netfp_ServerMCB*        ptrNetfpServer,
     *  int32_t*                errCode
     *  )
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Netfp_ServerHandle);

    /*  - Argument 2: */
    args[1].type   = Josh_ArgumentType_PASS_BY_REF;
    args[1].length = sizeof(int32_t);

    /* Add the arguments. */
    argHandle = Josh_addArguments (ptrNetfpClient->joshHandle, &args[0]);
    if (argHandle == NULL)
    {
        *errCode = NETFP_EJOSH;
        return -1;
    }

    /* Populate the arguments */
    *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(ptrNetfpClient->cfg.serverHandle);

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

    /* Populate the output arguments. */
    *errCode = *((int32_t*)args[1].argBuffer);

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
 *      The function is used to indicate that there is a status update for the next
 *      hop MAC address. If the interface handle is set to NULL then it implies that the
 *      next hop MAC address is no longer reachable and does not exist. This will cause
 *      all the SA and Fast Paths which are using this neighbor to become a zombie.
 *      However if the interface handle is set to non-NULL then it implies that the
 *      next hop MAC address is reachable and the SA and Fast paths are now active.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  ptrProxyServerInfo
 *      Pointer to the Proxy Server Info
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
static int32_t Netfp_updateNeighborStatus
(
    Netfp_ServerMCB*        ptrNetfpServer,
    Netfp_ProxyServerInfo*  ptrProxyServerInfo,
    int32_t*                errCode
)
{
    Netfp_IPSecChannel*         ptrOutboundIPSecChannel;
    Netfp_OutboundFastPath*     ptrOutboundFastPath;
    Netfp_Interface*            ptrNetfpInterface;
    Netfp_Status                newStatus;
    uint8_t                     bIsSameInterface;

    /* Get the NETFP Interface from the response: */
    ptrNetfpInterface = (Netfp_Interface*) ptrProxyServerInfo->u.updateNeigh.ifHandle;

    /* Determine the new status: */
    if (ptrNetfpInterface != NULL)
        newStatus = Netfp_Status_ACTIVE;
    else
        newStatus = Netfp_Status_ZOMBIE;

    /*************************************************************************************************
     * Cycle through to determine if the outbound IPSEC channel uses this neighbor
     *************************************************************************************************/
    ptrOutboundIPSecChannel = (Netfp_IPSecChannel*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrIPSecOutboundChannels);
    while (ptrOutboundIPSecChannel != NULL)
    {
        //fzm
        if(ptrOutboundIPSecChannel->status == Netfp_Status_STOP)
        {
            ptrOutboundIPSecChannel = (Netfp_IPSecChannel*)Netfp_listGetNext ((Netfp_ListNode*)ptrOutboundIPSecChannel);
            continue;
        }

        /* Do we have a match? */
        if ((Netfp_matchIP(&ptrOutboundIPSecChannel->saCfg.srcIP, &ptrProxyServerInfo->srcIP) == 1) &&
            (Netfp_matchIP(&ptrOutboundIPSecChannel->saCfg.dstIP, &ptrProxyServerInfo->dstIP) == 1) &&
            (memcmp((void *)&ptrOutboundIPSecChannel->nextHopMACAddress[0], (void *)ptrProxyServerInfo->u.updateNeigh.oldMacAddress, 6) == 0) &&
            /*If nothing at all has changed, suppress the event*/
            !(ptrOutboundIPSecChannel->status == newStatus &&
              ptrOutboundIPSecChannel->ptrNetfpInterface == ptrNetfpInterface &&
              (memcmp((void *)&ptrOutboundIPSecChannel->nextHopMACAddress[0], (void *)ptrProxyServerInfo->u.updateNeigh.nextHopMacAddress, 6) == 0)
             ))
        {
            /* YES. We have a match. Mark the IPSEC channel appropriately. */
            ptrOutboundIPSecChannel->status            = newStatus;
            ptrOutboundIPSecChannel->ptrNetfpInterface = ptrNetfpInterface;

            /* Update the MAC address */
            memcpy ((void*)&ptrOutboundIPSecChannel->nextHopMACAddress[0], (void*)ptrProxyServerInfo->u.updateNeigh.nextHopMacAddress, 6);

            /*************************************************************************************************
             * Update the security policy to be inline with the new IPSEC channel status.
             *************************************************************************************************/
            if (ptrOutboundIPSecChannel->status == Netfp_Status_ACTIVE)
                Netfp_updateSP (ptrNetfpServer, Netfp_Reason_NEIGH_REACHABLE, ptrOutboundIPSecChannel, ptrOutboundIPSecChannel);
            else
                Netfp_updateSP (ptrNetfpServer, Netfp_Reason_NEIGH_UNREACHABLE, ptrOutboundIPSecChannel, ptrOutboundIPSecChannel);
            Netfp_logMsg (ptrNetfpServer, Netfp_LogLevel_DEBUG, "Debug: Neighbor for Security Channel 0x%x is now %s\n",
                          (uint32_t)ptrOutboundIPSecChannel, (ptrOutboundIPSecChannel->status != Netfp_Status_ACTIVE) ? "not reachable" : "reachable");
        }

        /* Get the next outbound security channel. */
        ptrOutboundIPSecChannel = (Netfp_IPSecChannel*)Netfp_listGetNext ((Netfp_ListNode*)ptrOutboundIPSecChannel);
    }

    /*************************************************************************************************
     * Cycle through to determine if the Non-Secure Fast Path uses this neighbor
     *************************************************************************************************/
    ptrOutboundFastPath = (Netfp_OutboundFastPath*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrOutboundFPList);
    while (ptrOutboundFastPath != NULL)
    {
        /* Is this a non secure fast path? */
        if (ptrOutboundFastPath->isSecure == 0)
        {
            /* YES. Do we have a match? */
            if ((Netfp_matchIP(&ptrOutboundFastPath->cfg.srcIP, &ptrProxyServerInfo->srcIP) == 1) &&
                (Netfp_matchIP(&ptrOutboundFastPath->cfg.dstIP, &ptrProxyServerInfo->dstIP) == 1) &&
                (memcmp((void *)&ptrOutboundFastPath->nextHopMACAddress[0], (void *)ptrProxyServerInfo->u.updateNeigh.oldMacAddress, 6) == 0))
            {
                /* YES. Has there been a change in the interface? */
                if (ptrOutboundFastPath->ptrNetfpInterface == ptrNetfpInterface)
                    bIsSameInterface = 1;
                else
                    bIsSameInterface = 0;

                /* Update the fields in the outbound fast path */
                ptrOutboundFastPath->ptrNetfpInterface = ptrNetfpInterface;

                /* Update the MAC address */
                memcpy ((void*)&ptrOutboundFastPath->nextHopMACAddress[0], (void*)ptrProxyServerInfo->u.updateNeigh.nextHopMacAddress, 6);

                /* Notify the NETFP universe (intelligently): */
                Netfp_notifyOutboundFPChanges (ptrNetfpServer, ptrOutboundFastPath, bIsSameInterface, ptrNetfpInterface,
                                               ptrProxyServerInfo->u.updateNeigh.oldMacAddress, ptrProxyServerInfo->u.updateNeigh.nextHopMacAddress);

                /* Debug Message: */
                Netfp_logMsg (ptrNetfpServer, Netfp_LogLevel_DEBUG, "Debug: Neighbor for FP 0x%x is now %s\n",
                              (uint32_t)ptrOutboundFastPath, (newStatus == Netfp_Status_ZOMBIE) ? "not reachable" : "reachable");
            }
        }
        /* Get the next outbound fast path */
        ptrOutboundFastPath = (Netfp_OutboundFastPath*)Netfp_listGetNext ((Netfp_ListNode*)ptrOutboundFastPath);
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      NETFP Proxy has detected an ICMP Path MTU update
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  ptrProxyServerInfo
 *      Pointer to the Proxy Server Info
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
static int32_t Netfp_updateMTU
(
    Netfp_ServerMCB*        ptrNetfpServer,
    Netfp_ProxyServerInfo*  ptrProxyServerInfo,
    int32_t*                errCode
)
{
    Netfp_IPSecChannel*         ptrOutboundIPSecChannel;
    Netfp_OutboundFastPath*     ptrOutboundFastPath;
    Netfp_EventMetaInfo         eventInfo;

    /*************************************************************************************************
     * Cycle through to determine if the outbound IPSEC channel MTU needs to be updated. This will
     * handle the MTU for the Outer IP.
     *************************************************************************************************/
    ptrOutboundIPSecChannel = (Netfp_IPSecChannel*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrIPSecOutboundChannels);
    while (ptrOutboundIPSecChannel != NULL)
    {
        /* Do we have a match? */
        if ((Netfp_matchIP(&ptrOutboundIPSecChannel->saCfg.srcIP, &ptrProxyServerInfo->srcIP) == 1) &&
            (Netfp_matchIP(&ptrOutboundIPSecChannel->saCfg.dstIP, &ptrProxyServerInfo->dstIP) == 1))
        {
            /* YES: Setup the IPSEC channel PMTU to determine if we can use the new MTU or not. */
            Netfp_setupIPSECChannelPMTU (ptrNetfpServer, ptrOutboundIPSecChannel, ptrProxyServerInfo->u.updateMTU.newMTU);

            /* Propogate this information to the SP module which will in turn pass this on to the FP Module. */
            Netfp_updateSP (ptrNetfpServer, Netfp_Reason_PMTU_CHANGE, ptrOutboundIPSecChannel, ptrOutboundIPSecChannel);
        }

        /* Get the next outbound security channel. */
        ptrOutboundIPSecChannel = (Netfp_IPSecChannel*)Netfp_listGetNext ((Netfp_ListNode*)ptrOutboundIPSecChannel);
    }

    /*************************************************************************************************
     * Cycle through *all* the fast paths to determine if we need to change the MTU. This is because
     * we could receive an ICMP MTU update
     * (a) Non-Secure Fast Paths
     * (b) Secure Fast Paths for the Inner IP
     *************************************************************************************************/
    ptrOutboundFastPath = (Netfp_OutboundFastPath*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrOutboundFPList);
    while (ptrOutboundFastPath != NULL)
    {
        /* YES. Do we have a match? */
        if ((Netfp_matchIP(&ptrOutboundFastPath->cfg.srcIP, &ptrProxyServerInfo->srcIP) == 1) &&
            (Netfp_matchIP(&ptrOutboundFastPath->cfg.dstIP, &ptrProxyServerInfo->dstIP) == 1))
        {
            /* YES: Setup the Fast Path PMTU */
            Netfp_setupFastPathPMTU (ptrNetfpServer, ptrOutboundFastPath, ptrProxyServerInfo->u.updateMTU.newMTU);

            /* Generate an event to indicate that the Path MTU for the fast path has changed */
            eventInfo.eventId               = Netfp_EventId_UPDATE_FP;
            eventInfo.u.fpMeta.reason       = Netfp_Reason_PMTU_CHANGE;
            eventInfo.u.fpMeta.fpHandle     = (void*)ptrOutboundFastPath;
            eventInfo.u.fpMeta.status       = ptrOutboundFastPath->status;
            Netfp_populateSocketL2ConnectInfo (ptrNetfpServer, ptrOutboundFastPath, &eventInfo.u.fpMeta.l2ConnectInfo);
            Netfp_generateEvent (ptrNetfpServer, &eventInfo);
        }
        /* Get the next outbound fast path */
        ptrOutboundFastPath = (Netfp_OutboundFastPath*)Netfp_listGetNext ((Netfp_ListNode*)ptrOutboundFastPath);
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get an asynchronous update from the NETFP Proxy. The Proxy
 *      will inform the NETFP Server about neighbor updates or PMTU updates dynamically as
 *      soon as it detects the condition.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  ptrProxyServerInfo
 *      Pointer to the Proxy Server Info
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
static int32_t _Netfp_asyncUpdate
(
    Netfp_ServerMCB*        ptrNetfpServer,
    Netfp_ProxyServerInfo*  ptrProxyServerInfo,
    int32_t*                errCode
)
{
    /* We have received an asynchronous update from the PROXY. Dispatch this to the appropriate handler */
    if (ptrProxyServerInfo->opType == Netfp_ProxyServerOp_UPDATE_NEIGH)
        return Netfp_updateNeighborStatus (ptrNetfpServer, ptrProxyServerInfo, errCode);
    else if (ptrProxyServerInfo->opType == Netfp_ProxyServerOp_UPDATE_MTU)
        return Netfp_updateMTU (ptrNetfpServer, ptrProxyServerInfo, errCode);

    /* Error: Received an invalid operational code. */
    *errCode = NETFP_EINVAL;
    return -1;
}

/**
 *  @b Description
 *  @n
 *      The function is invoked by the NETFP Proxy to indicate that there is a status update with
 *      respect to the specific MAC address that is being monitored.
 *
 *  @param[in]  clientHandle
 *      Handle to the NETFP client
 *  @param[in]  ptrProxyServerInfo
 *      Pointer to the Proxy Server Info
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   job ID
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_asyncUpdate
(
    Netfp_ClientHandle      clientHandle,
    Netfp_ProxyServerInfo*  ptrProxyServerInfo,
    int32_t*                errCode
)
{
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    int32_t                 jobId;
    Netfp_ClientMCB*        ptrNetfpClient;
    Netfp_ProxyServerInfo*  ptrRemoteProxyServerInfo;

    /* Sanity Check: Ensure that the arguments are valid */
    if (clientHandle == NULL)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Sanity Check: Ensure that the arguments are valid */
    if (ptrProxyServerInfo == NULL)
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
    jobHandle = Josh_findJobByAddress(ptrNetfpClient->joshHandle, (Josh_JobProtype)_Netfp_asyncUpdate);
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
     *  int32_t _Netfp_asyncUpdate
     *  (
     *  Netfp_ServerMCB*        ptrNetfpServer,
     *  Netfp_ProxyServerInfo*  ptrProxyServerInfo,
     *  int32_t*                errCode
     *  )
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Netfp_ServerHandle);

    /*  - Argument 2: */
    args[1].type   = Josh_ArgumentType_PASS_BY_REF;
    args[1].length = sizeof(Netfp_ProxyServerInfo);

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

    /* Get the pointer to the Proxy Server Info */
    ptrRemoteProxyServerInfo = (Netfp_ProxyServerInfo*)args[1].argBuffer;
    memcpy ((void *)ptrRemoteProxyServerInfo, (void*)ptrProxyServerInfo, sizeof(Netfp_ProxyServerInfo));

    /* Submit the job asynchronously to the NETFP Server */
    jobId = Josh_submitAsyncJob (jobHandle, argHandle, errCode);
    if (jobId < 0)
    {
        uint32_t jobInstanceId = Josh_getJobInstanceId(argHandle);
        Josh_freeJobInstance(ptrNetfpClient->joshHandle, jobInstanceId);

        return -1;
    }

    Josh_freeJobInstance(ptrNetfpClient->joshHandle, jobId);

    return 0;
}

/**
 *  @b Description
 *  @n
 *      This function is invoked on the NETFP Server to indicate that the proxy has sent the
 *      response to the message.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  ptrProxyServerBulkMsg
 *      Pointer to the Proxy server information block
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   NETFP Error code
 */
static int32_t _Netfp_sendProxyResponse
(
    Netfp_ServerMCB*           ptrNetfpServer,
    Netfp_ProxyServerBulkMsg*  ptrProxyServerBulkMsg
)
{
    Netfp_IPSecChannel*         ptrOutboundIPSecChannel;
    Netfp_OutboundFastPath*     ptrOutboundFastPath;
    Netfp_ServerToProxyNode*    ptrServerToProxyNode;
    int32_t                     result = 0;
    uint8_t                     bIsSameInterface;
    uint8_t                     oldNextHopMACAddress[6];
    uint32_t                    startMonitor;
    uint32_t                    requestId;
    Netfp_ProxyServerInfo*      ptrProxyServerInfo;

    for ( int i = 0 ; i < ptrProxyServerBulkMsg->numberOfEntries; i++ )
    {
        requestId = ptrProxyServerBulkMsg->proxyServerInfo[i].requestId;

        /* Cycle through all the entries which are pending */
        ptrServerToProxyNode = (Netfp_ServerToProxyNode*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrPendingProxyServerList);
        while (ptrServerToProxyNode != NULL)
        {
            /* Is this what we are looking for? */
            if (ptrServerToProxyNode->requestId == requestId)
                break;

            /* Get the next entry in the pending list */
            ptrServerToProxyNode = (Netfp_ServerToProxyNode*)Netfp_listGetNext ((Netfp_ListNode*)ptrServerToProxyNode);
        }

        /* Did we get a match? This could happen if the route resolution identifier was not valid and does NOT exist in the system.
         * NETFP Proxy could have passed an invalid route identifier. */
        if (ptrServerToProxyNode == NULL)
        {
            result = NETFP_EINVAL;
            continue;
        }

        /* Remove this entry from the pending route list */
        Netfp_listRemoveNode ((Netfp_ListNode**)&ptrNetfpServer->ptrPendingProxyServerList, (Netfp_ListNode*)ptrServerToProxyNode);

        ptrProxyServerInfo = &ptrProxyServerBulkMsg->proxyServerInfo[i];

        /* Keep track of the type of request: */
        startMonitor = ptrServerToProxyNode->proxyServerInfo.u.req.startMonitor;

        /* Debug Message: */
        if (ptrServerToProxyNode->proxyServerInfo.dstIP.ver == Netfp_IPVersion_IPV4)
        {
            Netfp_logMsg (ptrNetfpServer, Netfp_LogLevel_DEBUG, "Debug: %s Monitoring %03d.%03d.%03d.%03d -> %03d.%03d.%03d.%03d [Id: 0x%x]\n",
                          (startMonitor == 1) ? "Started" : "Stopped",
                          ptrServerToProxyNode->proxyServerInfo.srcIP.addr.ipv4.u.a8[0], ptrServerToProxyNode->proxyServerInfo.srcIP.addr.ipv4.u.a8[1],
                          ptrServerToProxyNode->proxyServerInfo.srcIP.addr.ipv4.u.a8[2], ptrServerToProxyNode->proxyServerInfo.srcIP.addr.ipv4.u.a8[3],
                          ptrServerToProxyNode->proxyServerInfo.dstIP.addr.ipv4.u.a8[0], ptrServerToProxyNode->proxyServerInfo.dstIP.addr.ipv4.u.a8[1],
                          ptrServerToProxyNode->proxyServerInfo.dstIP.addr.ipv4.u.a8[2], ptrServerToProxyNode->proxyServerInfo.dstIP.addr.ipv4.u.a8[3],
                          requestId);
        }
        else
        {
            char    srcIP[256];
            char    dstIP[256];

            /* Convert the Src & Dst IP address into strings using the NETFP IPv6 Utility API */
            Netfp_convertIP6ToStr (ptrServerToProxyNode->proxyServerInfo.srcIP.addr.ipv6, &srcIP[0]);
            Netfp_convertIP6ToStr (ptrServerToProxyNode->proxyServerInfo.dstIP.addr.ipv6, &dstIP[0]);

            Netfp_logMsg (ptrNetfpServer, Netfp_LogLevel_DEBUG, "Debug: %s Monitoring %s -> %s [Id: 0x%x]\n",
                          (startMonitor == 1) ? "Started" : "Stopped",
                          srcIP, dstIP, requestId);
        }

        /* Cleanup the proxy server node; we are done. */
        ptrNetfpServer->cfg.free (ptrServerToProxyNode, sizeof(Netfp_ServerToProxyNode));

        /* Did we send a request to stop monitoring? */
        if (startMonitor == 0)
        {
            /**************************************************************************************
             * STOP Monitoring updates can be sent by the NETFP Proxy for the following sub-modules
             * (a) Outbound Security associations
             * (b) Outbound Fast Paths
             **************************************************************************************/
            ptrOutboundIPSecChannel = Netfp_findResolvedSA (ptrNetfpServer, requestId);
            if (ptrOutboundIPSecChannel != NULL)
            {
                /* Remove the outbound IPSEC channel from the NETFP Server */
                Netfp_listRemoveNode ((Netfp_ListNode**)&ptrNetfpServer->ptrIPSecOutboundChannels, (Netfp_ListNode*)ptrOutboundIPSecChannel);

                /* Cleanup the memory associated with the IPSEC channel. */
                Netfp_removeSAFromRecomputationList(ptrNetfpServer, ptrOutboundIPSecChannel);
                ptrNetfpServer->cfg.free (ptrOutboundIPSecChannel, sizeof(Netfp_IPSecChannel));
                continue;
            }

            /* Case (b) is handled here: */
            ptrOutboundFastPath = Netfp_findResolvedFP (ptrNetfpServer, requestId);
            if (ptrOutboundFastPath != NULL)
            {
                /* Remove the fast path from the NETFP Server */
                Netfp_listRemoveNode((Netfp_ListNode**)&ptrNetfpServer->ptrOutboundFPList, (Netfp_ListNode*)ptrOutboundFastPath);

                /* Cleanup the memory associated with the fast path */
                Netfp_removeFPFromRecomputationList(ptrNetfpServer, ptrOutboundFastPath);
                ptrNetfpServer->cfg.free (ptrOutboundFastPath, sizeof(Netfp_OutboundFastPath));
                continue;
            }

            /* Control comes here implies that there was no Outbound SA/Fast Path matching the request identifier.
             * This should NOT happen; because we marked them as ZOMBIE and they can only get removed after we updated
             * NETFP Proxy. But no harm done here. */
            Netfp_logMsg (ptrNetfpServer, Netfp_LogLevel_ERROR, "No Outbound SA/FastPath matching the request identifier 0x%x, entry %d!!!", requestId, i);

            continue;
        }

        /**************************************************************************************
         * START Monitoring updates can be sent by the NETFP Proxy for the following sub-modules
         * (a) Outbound Security associations
         * (b) Outbound Fast Paths
         **************************************************************************************/
        ptrOutboundIPSecChannel = Netfp_findResolvedSA (ptrNetfpServer, requestId);
        if (ptrOutboundIPSecChannel != NULL)
        {
            /* Case (a) is handled here. Was the route resolved? */
            if (ptrProxyServerInfo->u.resp.ifHandle != NULL)
                ptrOutboundIPSecChannel->status = Netfp_Status_ACTIVE;
            else
                ptrOutboundIPSecChannel->status = Netfp_Status_ZOMBIE;

            /* Debug Message: */
            if (ptrOutboundIPSecChannel->status == Netfp_Status_ZOMBIE)
            {
                Netfp_logMsg (ptrNetfpServer, Netfp_LogLevel_DEBUG, "Debug: SA 0x%x is UNRESOLVED\n",
                              (uint32_t)ptrOutboundIPSecChannel);
            }
            else
            {
                Netfp_logMsg (ptrNetfpServer, Netfp_LogLevel_DEBUG, "Debug: SA 0x%x is via [0x%02x:0x%02x:0x%02x:0x%02x:0x%02x:0x%02x]\n",
                    (uint32_t)ptrOutboundIPSecChannel, ptrProxyServerInfo->u.resp.nextHopMacAddress[0], ptrProxyServerInfo->u.resp.nextHopMacAddress[1],
                    ptrProxyServerInfo->u.resp.nextHopMacAddress[2], ptrProxyServerInfo->u.resp.nextHopMacAddress[3],
                    ptrProxyServerInfo->u.resp.nextHopMacAddress[4], ptrProxyServerInfo->u.resp.nextHopMacAddress[5]);
            }

            /* Updated the fields in the outbound IPSEC channel */
            ptrOutboundIPSecChannel->ptrNetfpInterface = (Netfp_Interface*)ptrProxyServerInfo->u.resp.ifHandle;
            ptrOutboundIPSecChannel->requestId         = 0;
            memcpy ((void*)&ptrOutboundIPSecChannel->nextHopMACAddress[0], (void*)ptrProxyServerInfo->u.resp.nextHopMacAddress, 6);

            /* Setup the IPSEC Channel PMTU: This could be affected because of the IPSEC Channel status */
            Netfp_setupIPSECChannelPMTU (ptrNetfpServer, ptrOutboundIPSecChannel, NETFP_INVALID_MTU);

            /*************************************************************************************************
             * Update the security policy to be inline with the new IPSEC channel status.
             *************************************************************************************************/
            if (ptrOutboundIPSecChannel->status == Netfp_Status_ACTIVE)
                Netfp_updateSP (ptrNetfpServer, Netfp_Reason_NEIGH_REACHABLE, ptrOutboundIPSecChannel, ptrOutboundIPSecChannel);
            else
                Netfp_updateSP (ptrNetfpServer, Netfp_Reason_NEIGH_UNREACHABLE, ptrOutboundIPSecChannel, ptrOutboundIPSecChannel);

            continue;
        }

        /* Case (b) is handled here: Get the matching outbound fast path */
        ptrOutboundFastPath = Netfp_findResolvedFP (ptrNetfpServer, requestId);
        if (ptrOutboundFastPath != NULL)
        {
            /* YES. Has there been a change in the interface? */
            if (ptrOutboundFastPath->ptrNetfpInterface == (Netfp_Interface*)ptrProxyServerInfo->u.resp.ifHandle)
                bIsSameInterface = 1;
            else
                bIsSameInterface = 0;

            /* Keep a copy of the older resolved MAC address: */
            memcpy ((void *)&oldNextHopMACAddress[0], (void*)&ptrOutboundFastPath->nextHopMACAddress[0], 6);

            /* Update the fields in the outbound fast path */
            ptrOutboundFastPath->ptrNetfpInterface = (Netfp_Interface*)ptrProxyServerInfo->u.resp.ifHandle;
            ptrOutboundFastPath->requestId         = 0;
            memcpy ((void*)&ptrOutboundFastPath->nextHopMACAddress[0], (void*)ptrProxyServerInfo->u.resp.nextHopMacAddress, 6);

            /* Notify the NETFP universe (intelligently): */
            Netfp_notifyOutboundFPChanges (ptrNetfpServer, ptrOutboundFastPath, bIsSameInterface,
                (Netfp_Interface*)ptrProxyServerInfo->u.resp.ifHandle, &oldNextHopMACAddress[0], &ptrProxyServerInfo->u.resp.nextHopMacAddress[0]);

            /* Debug Messages:  */
            if (ptrProxyServerInfo->u.resp.ifHandle != NULL)
            {
                Netfp_logMsg (ptrNetfpServer, Netfp_LogLevel_DEBUG, "Debug: FP 0x%x is via [0x%02x:0x%02x:0x%02x:0x%02x:0x%02x:0x%02x]\n",
                    (uint32_t)ptrOutboundFastPath, ptrProxyServerInfo->u.resp.nextHopMacAddress[0], ptrProxyServerInfo->u.resp.nextHopMacAddress[1],
                    ptrProxyServerInfo->u.resp.nextHopMacAddress[2], ptrProxyServerInfo->u.resp.nextHopMacAddress[3],
                    ptrProxyServerInfo->u.resp.nextHopMacAddress[4], ptrProxyServerInfo->u.resp.nextHopMacAddress[5]);
            }
            else
            {
                Netfp_logMsg (ptrNetfpServer, Netfp_LogLevel_DEBUG, "Debug: FP 0x%x is UNRESOLVED\n", (uint32_t)ptrOutboundFastPath);
            }

            continue;
        }

        /* Control comes here implies that the SA/Outbound Fast Path which had initiated the request has been deleted
         * before the response was received. */
    }

    return result;
}

/**
 *  @b Description
 *  @n
 *      This function is available only to the NETFP Proxy client. The proxy uses this API to
 *      send a response back to the NETFP server. The request identifier is used to correlate
 *      the request and response with each other.
 *
 *      If the NETFP Interface handle used to get to the next HOP MAC Address is set
 *      to NULL indicates that the route resolution failed and the destination is unreachable.
 *
 *  @param[in]  clientHandle
 *      Handle to the NETFP Proxy client
 *  @param[in]  ptrProxyServerBulkMsg
 *      Pointer to the Proxy server information block
 *  @param[out] errCode
 *      Error code populated on error
 *
 * \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_sendProxyResponse
(
    Netfp_ClientHandle          clientHandle,
    Netfp_ProxyServerBulkMsg*   ptrProxyServerBulkMsg,
    int32_t*                    errCode
)
{
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    int32_t                 jobId;
    Netfp_ClientMCB*        ptrNetfpClient;
    Netfp_ProxyServerBulkMsg*  ptrRemoteproxyServerBulkMsg;

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

    /* Sanity Check: To prevent misuse; only NETFP Proxy clients can call this API; so here we ensure
     * that the proxy configuration is valid */
    if (ptrNetfpClient->proxyCfg.proxyServerInterfaceFunction == NULL)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Get the actual job which we will execute */
    jobHandle = Josh_findJobByAddress(ptrNetfpClient->joshHandle, (Josh_JobProtype)_Netfp_sendProxyResponse);
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
     *  int32_t _Netfp_sendProxyResponse
     *  (
     *  Netfp_ServerMCB*           ptrNetfpServer,
     *  Netfp_ProxyServerBulkMsg*  ptrProxyServerBulkMsg
     *  )
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Netfp_ServerHandle);

    /*  - Argument 2: */
    args[1].type   = Josh_ArgumentType_PASS_BY_REF;
    args[1].length = sizeof(ptrProxyServerBulkMsg->numberOfEntries) +
                     ptrProxyServerBulkMsg->numberOfEntries * sizeof(ptrProxyServerBulkMsg->proxyServerInfo[0]);

    /* Add the arguments. */
    argHandle = Josh_addArguments (ptrNetfpClient->joshHandle, &args[0]);
    if (argHandle == NULL)
    {
        *errCode = NETFP_EJOSH;
        return -1;
    }

    /* Populate the arguments */
    *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(ptrNetfpClient->cfg.serverHandle);

    /* Get the pointer to the Proxy Server Bulk Info: */
    ptrRemoteproxyServerBulkMsg = (Netfp_ProxyServerBulkMsg*)args[1].argBuffer;
    memcpy ((void*)ptrRemoteproxyServerBulkMsg, (void *)ptrProxyServerBulkMsg, args[1].length);

    /* Submit the JOB to JOSH for execution. */
    jobId = Josh_submitAsyncJob(jobHandle, argHandle, errCode);
    if (jobId < 0)
    {
        uint32_t jobInstanceId = Josh_getJobInstanceId(argHandle);
        Josh_freeJobInstance(ptrNetfpClient->joshHandle, jobInstanceId);

        return -1;
    }

    Josh_freeJobInstance(ptrNetfpClient->joshHandle, jobId);

    return 0;
}

/**
 *  @b Description
 *  @n
 *      This function is invoked only on the NETFP Proxy to indicate that the NETFP
 *      server has invoked the Proxy Server interface API.
 *
 *  @param[in]  clientHandle
 *      Handle to the NETFP Proxy client
 *  @param[in]  ptrProxyServerBulkMsg
 *      Pointer to the Proxy server information block
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t _Netfp_sendProxyRequest
(
    Netfp_ClientHandle          clientHandle,
    Netfp_ProxyServerBulkMsg*   ptrProxyServerBulkMsg
)
{
    Netfp_ClientMCB*        ptrNetfpClient;

    /* Get the NETFP Client MCB: Services are available only if the client is active. */
    ptrNetfpClient = (Netfp_ClientMCB*)clientHandle;

    /* Sanity Check: This is the NETFP Proxy so the route resolve function should be specified.
     * This has already been verified during proxy registeration. */
    if (ptrNetfpClient->proxyCfg.proxyServerInterfaceFunction == NULL)
    {
        System_printf ("FATAL Error: Proxy does NOT have a proxy server bulk interface function");
        return -1;
    }

    /* Invoke the NETFP Proxy registered "interface function" */
    return ptrNetfpClient->proxyCfg.proxyServerInterfaceFunction (clientHandle, ptrProxyServerBulkMsg);
}

static void Netfp_addToPendingServerToProxyList(Netfp_ServerMCB* ptrNetfpServer,
                                              Netfp_ProxyServerBulkMsg* ptrProxyServerBulkMsg)
{
    for (int i = 0 ; i < ptrProxyServerBulkMsg->numberOfEntries ; i++ )
    {
        Netfp_listAdd ((Netfp_ListNode**)&ptrNetfpServer->ptrPendingProxyServerList, (Netfp_ListNode*)ptrProxyServerBulkMsg->proxyServerInfo[i].requestId);

        if (ptrProxyServerBulkMsg->proxyServerInfo[i].dstIP.ver == Netfp_IPVersion_IPV4)
        {
            Netfp_logMsg (ptrNetfpServer, Netfp_LogLevel_DEBUG, "Debug: Initiating %s for %03d.%03d.%03d.%03d -> %03d.%03d.%03d.%03d [0x%x]\n",
                (ptrProxyServerBulkMsg->proxyServerInfo[i].u.req.startMonitor == 1) ? "Start Monitor" : "Stop Monitor",
                ptrProxyServerBulkMsg->proxyServerInfo[i].srcIP.addr.ipv4.u.a8[0], ptrProxyServerBulkMsg->proxyServerInfo[i].srcIP.addr.ipv4.u.a8[1],
                ptrProxyServerBulkMsg->proxyServerInfo[i].srcIP.addr.ipv4.u.a8[2], ptrProxyServerBulkMsg->proxyServerInfo[i].srcIP.addr.ipv4.u.a8[3],
                ptrProxyServerBulkMsg->proxyServerInfo[i].dstIP.addr.ipv4.u.a8[0], ptrProxyServerBulkMsg->proxyServerInfo[i].dstIP.addr.ipv4.u.a8[1],
                ptrProxyServerBulkMsg->proxyServerInfo[i].dstIP.addr.ipv4.u.a8[2], ptrProxyServerBulkMsg->proxyServerInfo[i].dstIP.addr.ipv4.u.a8[3],
                ptrProxyServerBulkMsg->proxyServerInfo[i].requestId);
        }
        else
        {
            char    srcIP[40];
            char    dstIP[40];

            /* Convert the Src & Dst IP address into strings using the NETFP IPv6 Utility API */
            Netfp_convertIP6ToStr (ptrProxyServerBulkMsg->proxyServerInfo[i].srcIP.addr.ipv6, &srcIP[0]);
            Netfp_convertIP6ToStr (ptrProxyServerBulkMsg->proxyServerInfo[i].dstIP.addr.ipv6, &dstIP[0]);

            Netfp_logMsg (ptrNetfpServer, Netfp_LogLevel_DEBUG, "Debug: Initiating %s for %s -> %s [0x%x]\n",
                          (ptrProxyServerBulkMsg->proxyServerInfo[i].u.req.startMonitor == 1) ? "Start Monitor" : "Stop Monitor",
                          srcIP, dstIP, ptrProxyServerBulkMsg->proxyServerInfo[i].requestId);
        }
    }
}

/**
 *  @b Description
 *  @n
 *      The function is used to invoked from the NETFP Server to invoke the Proxy-Server registered
 *      interface function on the NETFP Proxy. This allows the
 *      server to send a request as array of actions. Each
 *      request is associated with a unique identifier
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  ptrProxyServerBulkMsg
 *      Pointer to the Proxy Server Bulk Information
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   Error code
 */
int32_t Netfp_sendProxyRequest
(
    Netfp_ServerMCB*            ptrNetfpServer,
    Netfp_ProxyServerBulkMsg*   ptrProxyServerBulkMsg
)
{
    Josh_Argument               args[JOSH_MAX_ARGS];
    Josh_JobHandle              jobHandle;
    Josh_ArgHandle              argHandle;
    Netfp_ProxyServerBulkMsg*   ptrRemoteProxyServerBulkMsg;
    int32_t                     errCode;

    /* Route resolution can only proceed if a valid PROXY client has been configured. */
    if (ptrNetfpServer->proxyClientBlock == NULL)
        return NETFP_ENOTREADY;

    /* Get the actual job which we will execute */
    jobHandle = Josh_findJobByAddress(ptrNetfpServer->proxyClientBlock->joshHandle, (Josh_JobProtype)_Netfp_sendProxyRequest);
    if (jobHandle == NULL)
        return NETFP_EINTERNAL;

    /* Initialize the arguments; to avoid any junk */
    memset ((void *)&args, 0, sizeof(args));

    /***************************************************************************
     * This is the function which is to be invoked:
     *
     *  int32_t _Netfp_sendProxyBulkRequest
     *  (
     *  Netfp_ClientHandle          proxyNetfpClientHandle,
     *  Netfp_ProxyServerBulkMsg*  ptrProxyServerBulkMsg
     *  )
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Netfp_ClientHandle);

    /*  - Argument 2: */
    args[1].type   = Josh_ArgumentType_PASS_BY_REF;
    args[1].length = sizeof(ptrProxyServerBulkMsg->numberOfEntries) +
                     ptrProxyServerBulkMsg->numberOfEntries * sizeof(ptrProxyServerBulkMsg->proxyServerInfo[0]);

    /* Add the arguments. */
    argHandle = Josh_addArguments (ptrNetfpServer->proxyClientBlock->joshHandle, &args[0]);
    if (argHandle == NULL)
        return NETFP_EJOSH;

    /* Populate the arguments */
    *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(ptrNetfpServer->proxyClientBlock->clientHandle);

    /* Populate the destination IP address which is to be resolved. */
    ptrRemoteProxyServerBulkMsg = (Netfp_ProxyServerBulkMsg*)args[1].argBuffer;
    memcpy ((void*)ptrRemoteProxyServerBulkMsg, (void *)ptrProxyServerBulkMsg, args[1].length);

    /* Submit the job asynchronously to the NETFP Proxy for resolution. This is done because we are already
     * processing a JOB */
    int32_t jobId = Josh_submitAsyncJob (jobHandle, argHandle, &errCode);
    if (jobId < 0)
    {
        uint32_t jobInstanceId = Josh_getJobInstanceId(argHandle);
        Josh_freeJobInstance(ptrNetfpServer->proxyClientBlock->joshHandle, jobInstanceId);

        return errCode;
    }

    Josh_freeJobInstance(ptrNetfpServer->proxyClientBlock->joshHandle, jobId);

    Netfp_addToPendingServerToProxyList(ptrNetfpServer, ptrProxyServerBulkMsg);

    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is a debug function to display the contents of the interface
 *      between the NETFP Server & Proxy.
 *
 *  @param[in]  serverHandle
 *      Handle to the NETFP Server
 *
 *  \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
void Netfp_displayProxyServerInterface(Netfp_ServerHandle serverHandle)
{
    Netfp_ServerMCB*            ptrNetfpServer;
    Netfp_ServerToProxyNode*    ptrServerToProxyNode;

    /* Get the NETFP Server MCB: */
    ptrNetfpServer = (Netfp_ServerMCB*)serverHandle;
    if (ptrNetfpServer == NULL)
        return;

//fzm - use Netfp_dumpMsg to capture this with the USR1 signal
    /* Display the banner. */
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "**********************************************************\n");
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "Server To Proxy Interface [Pending List]\n");

    /* Cycle through the Server-Proxy List */
    ptrServerToProxyNode = (Netfp_ServerToProxyNode*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrPendingProxyServerList);
    while (ptrServerToProxyNode != NULL)
    {
        /* Display the Proxy Server Node: */
        if (ptrServerToProxyNode->proxyServerInfo.srcIP.ver == Netfp_IPVersion_IPV4)
        {
            Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,
                          "RequestId:0x%x %03d.%03d.%03d.%03d -> %03d.%03d.%03d.%03d\n",
                          ptrServerToProxyNode->requestId,
                          ptrServerToProxyNode->proxyServerInfo.srcIP.addr.ipv4.u.a8[0], ptrServerToProxyNode->proxyServerInfo.srcIP.addr.ipv4.u.a8[1],
                          ptrServerToProxyNode->proxyServerInfo.srcIP.addr.ipv4.u.a8[2], ptrServerToProxyNode->proxyServerInfo.srcIP.addr.ipv4.u.a8[3],
                          ptrServerToProxyNode->proxyServerInfo.dstIP.addr.ipv4.u.a8[0], ptrServerToProxyNode->proxyServerInfo.dstIP.addr.ipv4.u.a8[1],
                          ptrServerToProxyNode->proxyServerInfo.dstIP.addr.ipv4.u.a8[2], ptrServerToProxyNode->proxyServerInfo.dstIP.addr.ipv4.u.a8[3]);
        }
        else
        {
            char    srcIP[128];
            char    dstIP[128];

            /* Convert the Src & Dst IP address into strings using the NETFP IPv6 Utility API */
            Netfp_convertIP6ToStr (ptrServerToProxyNode->proxyServerInfo.srcIP.addr.ipv6, &srcIP[0]);
            Netfp_convertIP6ToStr (ptrServerToProxyNode->proxyServerInfo.dstIP.addr.ipv6, &dstIP[0]);

            Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,
                          "RequestId:0x%x %s -> %s\n",
                          ptrServerToProxyNode->requestId, srcIP, dstIP);
        }

        /* Get the next element in the list: */
        ptrServerToProxyNode = (Netfp_ServerToProxyNode*)Netfp_listGetNext ((Netfp_ListNode*)ptrServerToProxyNode);
    }

    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to register the NETFP Proxy-Server interface
 *      JOSH functions.
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
int32_t Netfp_registerProxyServerServices (Josh_NodeHandle nodeHandle)
{
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Netfp_sendProxyRequest);
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Netfp_sendProxyResponse);
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Netfp_recomputeRoutes);
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Netfp_asyncUpdate);
    return 0;
}
