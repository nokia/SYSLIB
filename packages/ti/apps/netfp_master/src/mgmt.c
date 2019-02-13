/**
 *   @file  mgmt.c
 *
 *   @brief
 *      The file implements the interface allows ARM applications to
 *      communicate with the NETFP master
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
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <getopt.h>
#include <unistd.h>

/* SYSLIB Include Files */
#include <ti/runtime/netfp/netfp.h>
#include <ti/apps/netfp_master/include/netfp_master_internal.h>

/**********************************************************************
 ************************ Master-Mgmt Functions ***********************
 **********************************************************************/

/**
 *  @b Description
 *  @n
 *      The function is used to find the port capture block given the
 *      specific port number and direction.
 *
 *  @param[in]  ptrNetfpMasterMCB
 *      Pointer to the NETFP Master
 *  @param[in]  portToBeCaptured
 *      Switch Port to be captured
 *  @param[in]  direction
 *      Direction in which the capture needs to be enabled
 *
 *  @retval
 *      Not NULL - Port capture block found
 *  @retval
 *      NULL     - No matching entry found
 */
static NetfpMaster_PortCapture* NetfpMaster_findPortCapture
(
    NetfpMaster_MCB*    ptrNetfpMasterMCB,
    uint8_t             portToBeCaptured,
    Netfp_Direction     direction
)
{
    NetfpMaster_PortCapture*    ptrPortCapture;

    /* Cycle through all the registered entities. */
    ptrPortCapture = (NetfpMaster_PortCapture*)NetfpMaster_listGetHead ((NetfpMaster_ListNode**)&ptrNetfpMasterMCB->ptrPortCaptureList);
    while (ptrPortCapture != NULL)
    {
        if ((ptrPortCapture->portCaptureRequest.portToBeCaptured == portToBeCaptured) &&
            (ptrPortCapture->portCaptureRequest.direction == direction))
            return ptrPortCapture;

        /* Get the next port capture element */
        ptrPortCapture = (NetfpMaster_PortCapture*)NetfpMaster_listGetNext((NetfpMaster_ListNode*)ptrPortCapture);
    }
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      The function is used to enable the port capture block and register it with the
 *      NETFP Master
 *
 *  @param[in]  ptrNetfpMasterMCB
 *      Pointer to the NETFP Master
 *  @param[in]  ptrPortCaptureRequest
 *      Pointer to the port capture request
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t NetfpMaster_enablePortCapture
(
    NetfpMaster_MCB*                    ptrNetfpMasterMCB,
    NetfpMaster_SetPortCaptureRequest*  ptrPortCaptureRequest,
    int32_t*                            errCode
)
{
    NetfpMaster_PortCapture*    ptrPortCapture;
    Netfp_PortCaptureCfg        portCaptureCfg;

    /* Is port capturing already enabled? */
    ptrPortCapture = NetfpMaster_findPortCapture (ptrNetfpMasterMCB, ptrPortCaptureRequest->portToBeCaptured,
                                                  ptrPortCaptureRequest->direction);
    if (ptrPortCapture != NULL)
    {
        /* YES. Port capturing is already enabled. We do not allow an edit. Applications should disable
         * port capturing and then reenable it with the new configuration. */
        *errCode = NETFP_MASTER_EINVAL;
        return -1;
    }

    /* Allocate memory for the port capture block: */
    ptrPortCapture = (NetfpMaster_PortCapture*)malloc (sizeof(NetfpMaster_PortCapture));
    if (ptrPortCapture == NULL)
    {
        *errCode = NETFP_MASTER_ENOMEM;
        return -1;
    }

    /* Initialize the allocated memory block: */
    memset ((void*)ptrPortCapture, 0, sizeof(NetfpMaster_PortCapture));

    /* Populate the port capture request */
    memcpy ((void *)&ptrPortCapture->portCaptureRequest, (void *)ptrPortCaptureRequest, sizeof(NetfpMaster_SetPortCaptureRequest));
    ptrPortCapture->flowId      = ptrPortCaptureRequest->flowId;

    /* Register this with the NETFP master */
    NetfpMaster_listAdd ((NetfpMaster_ListNode**)&ptrNetfpMasterMCB->ptrPortCaptureList, (NetfpMaster_ListNode*)ptrPortCapture);

    /* We now need to program the NETCP for port capturing. So initialize the port capture configuration */
    memset ((void*)&portCaptureCfg, 0, sizeof(Netfp_PortCaptureCfg));

    /* Populate the port configuration */
    portCaptureCfg.isEnable         = ptrPortCaptureRequest->isEnable;
    portCaptureCfg.portToBeCaptured = ptrPortCaptureRequest->portToBeCaptured;
    portCaptureCfg.flowId           = ptrPortCaptureRequest->flowId;
    portCaptureCfg.queueId          = ptrPortCaptureRequest->dstQueue;
    portCaptureCfg.swInfo           = ptrPortCaptureRequest->swInfo;
    portCaptureCfg.direction        = ptrPortCaptureRequest->direction;

    /* Enable the port capturing: */
    if (Netfp_setupPortCapture(ptrNetfpMasterMCB->netfpServerHandle, &portCaptureCfg, errCode) < 0)
    {
        /* Error: Unable to enable the port capturing. */
        NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: Unable to enable port capturing [Error code %d]\n", *errCode);

        /* Cleanup the flow and port capture block. */
        NetfpMaster_listRemoveNode ((NetfpMaster_ListNode**)&ptrNetfpMasterMCB->ptrPortCaptureList, (NetfpMaster_ListNode*)ptrPortCapture);
        free (ptrPortCapture);
        return -1;
    }

    /* Debug Message: */
    NetfpMaster_log(NetfpMaster_LogLevel_DEBUG, "Debug: Port capturing ENABLED Direction [%s] on %d Flow %d is created\n",
                    (portCaptureCfg.direction == Netfp_Direction_INBOUND) ? "Ingress" : "Egress",
                    portCaptureCfg.portToBeCaptured, ptrPortCapture->flowId);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to disable the port capture block and deregister it from
 *      the NETFP Master
 *
 *  @param[in]  ptrNetfpMasterMCB
 *      Pointer to the NETFP Master
 *  @param[in]  ptrPortCaptureRequest
 *      Pointer to the port capture request
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t NetfpMaster_disablePortCapture
(
    NetfpMaster_MCB*                    ptrNetfpMasterMCB,
    NetfpMaster_SetPortCaptureRequest*  ptrPortCaptureRequest,
    int32_t*                            errCode
)
{
    NetfpMaster_PortCapture*    ptrPortCapture;
    Netfp_PortCaptureCfg        portCaptureCfg;

    /* Is port capturing enabled? */
    ptrPortCapture = NetfpMaster_findPortCapture (ptrNetfpMasterMCB, ptrPortCaptureRequest->portToBeCaptured,
                                                  ptrPortCaptureRequest->direction);
    if (ptrPortCapture == NULL)
    {
        /* NO. Port capturing was disabled. */
        *errCode = NETFP_MASTER_EINVAL;
        return -1;
    }

    /* We now need to program the NETCP for port capturing. So initialize the port capture configuration */
    memset ((void*)&portCaptureCfg, 0, sizeof(Netfp_PortCaptureCfg));

    /* Populate the port configuration */
    portCaptureCfg.isEnable         = ptrPortCaptureRequest->isEnable;
    portCaptureCfg.portToBeCaptured = ptrPortCaptureRequest->portToBeCaptured;
    portCaptureCfg.flowId           = ptrPortCapture->flowId;
    portCaptureCfg.queueId          = ptrPortCaptureRequest->dstQueue;
    portCaptureCfg.swInfo           = ptrPortCaptureRequest->swInfo;
    portCaptureCfg.direction        = ptrPortCaptureRequest->direction;

    /* Disable the port capturing: */
    if (Netfp_setupPortCapture(ptrNetfpMasterMCB->netfpServerHandle, &portCaptureCfg, errCode) < 0)
    {
        /* Error: Unable to disable the port capturing. */
        NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: Unable to disable port capturing [Error code %d]\n", *errCode);
        return -1;
    }

    /* Debug Message: */
    NetfpMaster_log(NetfpMaster_LogLevel_DEBUG, "Debug: Port capturing DISABLED Direction [%s] on %d Flow %d is deleted\n",
                    (portCaptureCfg.direction == Netfp_Direction_INBOUND) ? "Ingress" : "Egress",
                    portCaptureCfg.portToBeCaptured, ptrPortCapture->flowId);

    /* Deregister the block from the NETFP Master */
    NetfpMaster_listRemoveNode ((NetfpMaster_ListNode**)&ptrNetfpMasterMCB->ptrPortCaptureList, (NetfpMaster_ListNode*)ptrPortCapture);

    /* Cleanup the memory */
    free (ptrPortCapture);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to process the management messages
 *
 *  @param[in]  ptrNetfpMasterMCB
 *      Pointer to the NETFP Master
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0
 */
static NetfpMaster_Entity* NetfpMaster_findEntityName
(
    NetfpMaster_MCB*    ptrNetfpMasterMCB,
    struct sockaddr_un* ptrEntityName
)
{
    NetfpMaster_Entity*     ptrEntity;

    /* Get the entity head */
    ptrEntity = (NetfpMaster_Entity*)NetfpMaster_listGetHead((NetfpMaster_ListNode**)&ptrNetfpMasterMCB->ptrEntityList);
    while (ptrEntity != NULL)
    {
        /* Do we have a match */
        if (strcmp (ptrEntityName->sun_path, ptrEntity->address.sun_path) == 0)
            return ptrEntity;

        /* Get the next entity. */
        ptrEntity = (NetfpMaster_Entity*)NetfpMaster_listGetNext((NetfpMaster_ListNode*)ptrEntity);
    }
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      The function is used to find the physical interface by Name
 *
 *  @param[in]  ptrNetfpMasterMCB
 *      Pointer to the NETFP Master
 *  @param[in]  ifName
 *      Name of the interface
 *
 *  @retval
 *      Success - Interface block handle
 *  @retval
 *      Error   - NULL
 */
static NetfpMaster_IfBlock* NetfpMaster_findPhyInterface
(
    NetfpMaster_MCB*    ptrNetfpMasterMCB,
    char*               ifName
)
{
    NetfpMaster_IfBlock*    ptrNetfpIfBlock;

    /* Cycle through all the registered interfaces to find a match */
    ptrNetfpIfBlock = (NetfpMaster_IfBlock*)NetfpMaster_listGetHead((NetfpMaster_ListNode**)&ptrNetfpMasterMCB->ptrInterfaceList);
    while (ptrNetfpIfBlock != NULL)
    {
        /* Did we find the interface? */
        if (strncmp (ptrNetfpIfBlock->name, ifName, NETFP_MASTER_MAX_CHAR) == 0)
        {
            /* YES. Populate the response. */
            return ptrNetfpIfBlock;
        }
        /* Get the next interface block */
        ptrNetfpIfBlock = (NetfpMaster_IfBlock*)NetfpMaster_listGetNext((NetfpMaster_ListNode*)ptrNetfpIfBlock);
    }

    return (NetfpMaster_IfBlock*)NULL;
}

/**
 *  @b Description
 *  @n
 *      The function is used to send an update to all the registered entities
 *      about an update to the specific interface.
 *
 *  @param[in]  ptrNetfpMasterMCB
 *      Pointer to the NETFP Master
 *  @param[in]  ifName
 *      Interface name which had been updated.
 *  @param[in]  newIfName
 *      New Interface name to be used
 *
 *  @retval
 *      Not applicable
 */
static void NetfpMaster_sendUpdate
(
    NetfpMaster_MCB*    ptrNetfpMasterMCB,
    char*               ifName,
    char*               newIfName
)
{
    NetfpMaster_Entity*     ptrEntity;
    NetfpMaster_Request     request;
    struct sockaddr_un      to;

    /* Get the entity head */
    ptrEntity = (NetfpMaster_Entity*)NetfpMaster_listGetHead((NetfpMaster_ListNode**)&ptrNetfpMasterMCB->ptrEntityList);
    while (ptrEntity != NULL)
    {
        /* Initialize the request */
        memset ((void*)&request, 0, sizeof(NetfpMaster_Request));

        /* Populate the update notice message */
        request.msgType = NetfpMaster_MessageType_UPDATE_NOTICE;
        request.id      = random();

        /* Copy the interface names */
       strcpy (request.u.updateIfNotice.ifName, ifName);
       strcpy (request.u.updateIfNotice.newIfName, newIfName);

        /* Send the update notice message to the entity.  */
        memset ((void *)&to, 0, sizeof(struct sockaddr_un));
        to.sun_family = AF_UNIX;
        snprintf(to.sun_path, sizeof(to.sun_path), "%s", ptrEntity->address.sun_path);

        /* Send the update message to the entity */
        sendto(ptrNetfpMasterMCB->masterMgmtSocket, &request, sizeof(NetfpMaster_Request), 0, (struct sockaddr *)&to, sizeof(to));

        /* Debug Message: */
        NetfpMaster_log(NetfpMaster_LogLevel_DEBUG, "Debug: Sending update for '%s' to '%s'\n", ifName, to.sun_path);

        /* Get the next entity. */
        ptrEntity = (NetfpMaster_Entity*)NetfpMaster_listGetNext((NetfpMaster_ListNode*)ptrEntity);
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to process the management messages
 *
 *  @param[in]  ptrNetfpMasterMCB
 *      Pointer to the NETFP Master
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0
 */
int32_t NetfpMaster_processMgmt(NetfpMaster_MCB* ptrNetfpMasterMCB)
{
    int32_t                         numBytes;
    int32_t                         errCode;
    struct sockaddr_un              from;
    NetfpMaster_Request             request;
    NetfpMaster_Response            response;
    NetfpMaster_Entity*             ptrEntity;
    NetfpMaster_IfBlock*            ptrNetfpIfBlock;
    Netfp_PortMirrorCfg             portMirrorCfg;
    Netfp_ReassemblyStats           reassemblyStats;
    Netfp_EthRuleCfg                ethRuleCfg;
    socklen_t                       fromLen;
    int32_t                         index;
    NetfpMaster_LogLevel            logLevel=NetfpMaster_LogLevel_DEBUG; //fzm

    /* Initialize the length of the data buffer to receive the socket address */
    fromLen = sizeof(struct sockaddr_un);

    /* Receive request from the application */
    numBytes = recvfrom(ptrNetfpMasterMCB->masterMgmtSocket, &request, sizeof(NetfpMaster_Request), 0,
                        (struct sockaddr *)&from, &fromLen);
    if (numBytes < 0)
    {
        /* Error: Unable to receive data from the server socket. */
        NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: Unable to receive NETFP master request: %s\n", strerror(errno));
        return -1;
    }

//fzm---->
    // Set VERBOSE mode to suppress specific message in normal log
    // TRSW uses NetfpMaster_MessageType_GET_NETCP_STATS as supervision command
    if (request.msgType == NetfpMaster_MessageType_GET_NETCP_STATS)
    {
       logLevel = NetfpMaster_LogLevel_VERBOSE;
    }

    NetfpMaster_log(logLevel, "Debug: NETFP master request message: %d from '%s' \n",
                     request.msgType, from.sun_path);
//<----fzm

    /* Initialize the response */
    memset ((void *)&response, 0, sizeof(NetfpMaster_Response));

    /* Populate the response: */
    response.id      = request.id;
    response.reqType = request.msgType;
    response.msgType = NetfpMaster_MessageType_RESPONSE;
    response.errCode = 0;

    /* Process the request */
    switch (request.msgType)
    {
        case NetfpMaster_MessageType_REGISTER_NOTIFICATION:
        {
            /* Register the entity for notifications: */
            ptrEntity = NetfpMaster_findEntityName (ptrNetfpMasterMCB, &from);
            if (ptrEntity != NULL)
                break;

            /* Entity does not exist; create and register it */
            ptrEntity = (NetfpMaster_Entity*)malloc (sizeof(NetfpMaster_Entity));
            if (ptrEntity == NULL)
            {
                response.errCode = NETFP_MASTER_ENOMEM;
                break;
            }

            /* Initialize the allocated memory. */
            memset ((void *)ptrEntity, 0, sizeof(NetfpMaster_Entity));

            /* Populate the entity block: */
            strcpy (ptrEntity->address.sun_path, from.sun_path);

            /* Register the entity */
            NetfpMaster_listAdd ((NetfpMaster_ListNode**)&ptrNetfpMasterMCB->ptrEntityList, (NetfpMaster_ListNode*)ptrEntity);
            break;
        }
        case NetfpMaster_MessageType_DEREGISTER_NOTIFICATION:
        {
            /* Deregister the entity from notifications: */
            ptrEntity = NetfpMaster_findEntityName (ptrNetfpMasterMCB, &from);
            if (ptrEntity == NULL)
            {
                response.errCode = NETFP_MASTER_ENOTFOUND;
                break;
            }

            /* Remove the entity from the list */
            NetfpMaster_listRemoveNode ((NetfpMaster_ListNode**)&ptrNetfpMasterMCB->ptrEntityList, (NetfpMaster_ListNode*)ptrEntity);

            /* Cleanup the memory */
            free (ptrEntity);
            break;
        }
        case NetfpMaster_MessageType_GET_INTERFACE_LIST:
        {
            /* Initialize the index */
            index = 0;

            /* Cycle through all the registered interfaces to find a match */
            ptrNetfpIfBlock = (NetfpMaster_IfBlock*)NetfpMaster_listGetHead((NetfpMaster_ListNode**)&ptrNetfpMasterMCB->ptrInterfaceList);
            while (ptrNetfpIfBlock != NULL)
            {
                /* YES. Populate the response */
                strncpy ((void *)&response.u.ifName[index++], (void*)&ptrNetfpIfBlock->name, NETFP_MASTER_MAX_CHAR);

                /* Get the next interface block */
                ptrNetfpIfBlock = (NetfpMaster_IfBlock*)NetfpMaster_listGetNext((NetfpMaster_ListNode*)ptrNetfpIfBlock);
            }
            break;
        }
        case NetfpMaster_MessageType_GET_INTERFACE_REQUEST:
        {
            /* Sanity Check: Validate the arguments */
            ptrNetfpIfBlock = NetfpMaster_findPhyInterface(ptrNetfpMasterMCB, &request.u.getIfRequest.ifName[0]);
            if (ptrNetfpIfBlock != NULL)
            {
                /* Populate the response: */
                response.u.getIfResponse.switchPort = ptrNetfpIfBlock->switchPort;
                memcpy ((void *)&response.u.getIfResponse.innerToOuterDSCPMap, (void*)&ptrNetfpIfBlock->innerToOuterDSCPMap,
                            sizeof(ptrNetfpIfBlock->innerToOuterDSCPMap));
            }
            else
            {
                /* Error: Physical interface does not exist. */
                response.errCode = NETFP_MASTER_ENOTFOUND;
            }
            break;
        }
        case NetfpMaster_MessageType_GET_NETCP_STATS:
        {
            /* Get the NETCP Statistics  */
            Netfp_getNETCPStats(ptrNetfpMasterMCB->netfpServerHandle, &response.u.netcpStats, &response.errCode);
            break;
        }
        case NetfpMaster_MessageType_RENAME_INTERFACE:
        {
            /* Sanity Check: Validate the arguments */
            ptrNetfpIfBlock = NetfpMaster_findPhyInterface(ptrNetfpMasterMCB, &request.u.renameInterface.oldIfName[0]);
            if (ptrNetfpIfBlock != NULL)
            {
                /* Store the new interface name: */
                strcpy (ptrNetfpIfBlock->name, request.u.renameInterface.newIfName);

                /* Notify all the entities in the system: Send the old & new names. */
                NetfpMaster_sendUpdate (ptrNetfpMasterMCB, &request.u.renameInterface.oldIfName[0], &request.u.renameInterface.newIfName[0]);
            }
            else
            {
                /* Error: Physical interface does not exist. */
                response.errCode = NETFP_MASTER_ENOTFOUND;
            }
            break;
        }
        case NetfpMaster_MessageType_SET_INNER_OUTER_DSCP_MAP:
        {
            /* Sanity Check: Validate the arguments */
            ptrNetfpIfBlock = NetfpMaster_findPhyInterface(ptrNetfpMasterMCB, &request.u.setInnerOuterDSCPMap.ifName[0]);
            if (ptrNetfpIfBlock == NULL)
            {
                /* Error: Physical interface does not exist */
                response.errCode = NETFP_MASTER_ENOTFOUND;
                break;
            }

            /* Sanity Check: Validate the arguments: */
            if ((request.u.setInnerOuterDSCPMap.innerDSCP >= NETFP_MASTER_MAX_DSCP_ENTRY)  ||
                (request.u.setInnerOuterDSCPMap.outerDSCP >= NETFP_MASTER_MAX_DSCP_ENTRY))
            {
                /* Error: Invalid arguments */
                response.errCode = NETFP_MASTER_EINVAL;
                break;
            }

            /* Update the marking map entry: */
            ptrNetfpIfBlock->innerToOuterDSCPMap[request.u.setInnerOuterDSCPMap.innerDSCP] = request.u.setInnerOuterDSCPMap.outerDSCP;

            /* Notify all the entities in the system */
            NetfpMaster_sendUpdate (ptrNetfpMasterMCB, ptrNetfpIfBlock->name, ptrNetfpIfBlock->name);
            break;
        }
        case NetfpMaster_MessageType_SET_DEFAULT_HOST_PRIORITY:
        {
            /* Copy over the default host priority: */
            ptrNetfpMasterMCB->defaultHostPriority = request.u.setDefaultHostPriority.defaultHostPriority;

            /* Reinitialize the EQOS Block: */
            Netfp_initEQOS (ptrNetfpMasterMCB->netfpServerHandle, ptrNetfpMasterMCB->defaultHostPriority, &response.errCode);
            break;
        }
        case NetfpMaster_MessageType_SET_DEFAULT_FWD_PRIORITY:
        {
            /* Sanity Check: Validate the arguments */
            ptrNetfpIfBlock = NetfpMaster_findPhyInterface(ptrNetfpMasterMCB, &request.u.setDefaultFwdPriority.ifName[0]);
            if (ptrNetfpIfBlock == NULL)
            {
                /* Error: Physical interface does not exist */
                response.errCode = NETFP_MASTER_ENOTFOUND;
                break;
            }

            /* Update the default forwarding priority */
            ptrNetfpIfBlock->defaultFwdPriority = request.u.setDefaultFwdPriority.defaultForwardPriority;

            /* Reconfigure the L2 shaper for the specified NETFP interface block.
             * If the function fails the error code is already populated in the response. */
            NetfpMaster_setupL2Shaper(ptrNetfpMasterMCB, ptrNetfpIfBlock, &response.errCode);
            break;
        }
        case NetfpMaster_MessageType_SET_ROUTING_MODE:
        {
            /* Sanity Check: Get the physical interface using the interface name */
            ptrNetfpIfBlock = NetfpMaster_findPhyInterface(ptrNetfpMasterMCB, &request.u.setRoutingMode.ifName[0]);
            if (ptrNetfpIfBlock == NULL)
            {
                /* Error: Physical interface does not exist in the system */
                response.errCode = NETFP_MASTER_ENOTFOUND;
                break;
            }

            /* Sanity Check: Validate the arguments */
            if (strncmp (&request.u.setRoutingMode.routingMode[0], "dscp", NETFP_MASTER_MAX_CHAR) == 0)
            {
                ptrNetfpIfBlock->routingMode = NetfpMaster_RoutingMode_DSCP;
            }
            else if (strncmp (&request.u.setRoutingMode.routingMode[0], "dp-bit", NETFP_MASTER_MAX_CHAR) == 0)
            {
                ptrNetfpIfBlock->routingMode = NetfpMaster_RoutingMode_DPBIT;
            }
            else
            {
                /* Error: Invalid routing mode specified. */
                response.errCode = NETFP_MASTER_EINVAL;
                break;
            }

            /* Reconfigure the L2 shaper for the specified NETFP interface block.
             * If the function fails the error code is already populated in the response. */
            NetfpMaster_setupL2Shaper(ptrNetfpMasterMCB, ptrNetfpIfBlock, &response.errCode);
            break;
        }
        case NetfpMaster_MessageType_SET_DSCP_MAPPING:
        {
            /* Sanity Check: Get the physical interface using the interface name */
            ptrNetfpIfBlock = NetfpMaster_findPhyInterface(ptrNetfpMasterMCB, &request.u.setDSCPMap.ifName[0]);
            if (ptrNetfpIfBlock == NULL)
            {
                /* Error: Physical interface does not exist in the system */
                response.errCode = NETFP_MASTER_ENOTFOUND;
                break;
            }

            /* Sanity Check: Validate the arguments */
            if (request.u.setDSCPMap.dscp >= NETFP_MASTER_MAX_DSCP_ENTRY)
            {
                /* Error: DSCP Range is incorrect. */
                response.errCode = NETFP_MASTER_EINVAL;
                break;
            }

            /* Update the DSCP map in the NETFP Interface block. */
            ptrNetfpIfBlock->dscpMap[request.u.setDSCPMap.dscp].queueOffset = request.u.setDSCPMap.queueOffset;

            /* Reconfigure the L2 shaper for the specified NETFP interface block.
             * If the function fails the error code is already populated in the response. */
            NetfpMaster_setupL2Shaper(ptrNetfpMasterMCB, ptrNetfpIfBlock, &response.errCode);
            break;
        }
        case NetfpMaster_MessageType_SET_PRECLASSIFICATION:
        {
            /* Sanity Check: Get the physical interface using the interface name */
            ptrNetfpIfBlock = NetfpMaster_findPhyInterface(ptrNetfpMasterMCB, &request.u.setPreclassification.ifName[0]);
            if (ptrNetfpIfBlock == NULL)
            {
                /* Error: Physical interface does not exist in the system */
                response.errCode = NETFP_MASTER_ENOTFOUND;
                break;
            }

            /* Do we need to setup the broadcast/multicast preclassification? Populate the interface block. */
            if (request.u.setPreclassification.isBroadcast == 1)
            {
                /* Broadcast Preclassification: */
                ptrNetfpIfBlock->broadcastPreclassification        = request.u.setPreclassification.enablePreclassfication;
                ptrNetfpIfBlock->broadcastPreclassificationFlowId  = request.u.setPreclassification.preclassificationFlowId;
                ptrNetfpIfBlock->broadcastPreclassificationQueueId = request.u.setPreclassification.preclassificationQueueId;
            }
            else
            {
                /* Multicast Preclassification: */
                ptrNetfpIfBlock->multicastPreclassification        = request.u.setPreclassification.enablePreclassfication;
                ptrNetfpIfBlock->multicastPreclassificationFlowId  = request.u.setPreclassification.preclassificationFlowId;
                ptrNetfpIfBlock->multicastPreclassificationQueueId = request.u.setPreclassification.preclassificationQueueId;
            }

            /* Setup the preclassification in the NETFP: */
            NetfpMaster_setupPreclassification (ptrNetfpIfBlock, &response.errCode);
            break;
        }
        case NetfpMaster_MessageType_SET_VLAN_MAPPING:
        {
            /* Sanity Check: Get the physical interface using the interface name */
            ptrNetfpIfBlock = NetfpMaster_findPhyInterface(ptrNetfpMasterMCB, &request.u.setVLANMap.ifName[0]);
            if (ptrNetfpIfBlock == NULL)
            {
                /* Error: Physical interface does not exist in the system */
                response.errCode = NETFP_MASTER_ENOTFOUND;
                break;
            }

            /* Sanity Check: Validate the arguments */
            if (request.u.setVLANMap.pbit >= NETFP_MASTER_MAX_VLAN_ENTRY)
            {
                /* Error: DSCP Range is incorrect. */
                response.errCode = NETFP_MASTER_EINVAL;
                break;
            }

            /* Update the VLAN map in the NETFP Interface block. */
            ptrNetfpIfBlock->vlanMap[request.u.setVLANMap.pbit].queueOffset = request.u.setVLANMap.queueOffset;

            /* Reconfigure the L2 shaper for the specified NETFP interface block.
             * If the function fails the error code is already populated in the response. */
            NetfpMaster_setupL2Shaper(ptrNetfpMasterMCB, ptrNetfpIfBlock, &response.errCode);
            break;
        }
        case NetfpMaster_MessageType_SET_PORT_MIRROR_REQUEST:
        {
            /* Sanity Check: Validate the arguments. */
            if (request.u.setPortMirrorRequest.srcPort == request.u.setPortMirrorRequest.dstPort)
            {
                response.errCode = NETFP_MASTER_EINVAL;
                break;
            }

            /* Initialize the port capture configuration */
            memset ((void*)&portMirrorCfg, 0, sizeof(Netfp_PortMirrorCfg));

            /* Populate the port configuration */
            portMirrorCfg.isEnable  = request.u.setPortMirrorRequest.isEnable;
            portMirrorCfg.srcPort   = request.u.setPortMirrorRequest.srcPort;
            portMirrorCfg.dstPort   = request.u.setPortMirrorRequest.dstPort;
            if (request.u.setPortMirrorRequest.direction == 1)
                portMirrorCfg.direction = Netfp_Direction_INBOUND;
            else
                portMirrorCfg.direction = Netfp_Direction_OUTBOUND;

            /* Setup the port mirroring: */
            if (Netfp_setupPortMirror (ptrNetfpMasterMCB->netfpServerHandle, &portMirrorCfg, &errCode) < 0)
            {
                NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: Unable to configure port mirroring\n");
                response.errCode = errCode;
                break;
            }

            /* Debug Message: */
            NetfpMaster_log(NetfpMaster_LogLevel_DEBUG, "Debug: Port mirroring [%s] Direction [%s] from %d -> %d\n",
                            (portMirrorCfg.isEnable  == 1) ? "ENABLED" : "DISABLED",
                            (portMirrorCfg.direction == Netfp_Direction_INBOUND) ? "Ingress" : "Egress",
                            portMirrorCfg.srcPort, portMirrorCfg.dstPort);
            break;
        }
        case NetfpMaster_MessageType_GET_REASSEMBLY_STATS_REQUEST:
        {
            /* Is the NETFP master responsible for reassembly? */
            if (ptrNetfpMasterMCB->reassemblyHandling == 1)
            {
                if (Netfp_getReassemblyStats (ptrNetfpMasterMCB->netfpClientHandle, &reassemblyStats, &errCode) < 0)
                {
                    /* Error: This should not occur because the NETFP master is responsible for the reassembly */
                    response.errCode = errCode;
                }
                else
                {
                    /* Populate the reassembly statistics in the response */
                    memcpy ((void *)&response.u.reassemblyStats, (void*)&reassemblyStats, sizeof(Netfp_ReassemblyStats));
                }
            }
            else
            {
                /* No: Reassembly statistics are not available since the NETFP master is NOT responsible for it Applications
                 * should not request for reassembly statistics if the NETFP master was provisioned not to do so. */
                response.errCode = NETFP_MASTER_EINVAL;
            }
            break;
        }
        case NetfpMaster_MessageType_SET_PORT_CAPTURE_REQUEST:
        {
            /* Sanity Check: Validate the arguments. */
            if (request.u.setPortCaptureRequest.portToBeCaptured == 0)
            {
                response.errCode = NETFP_MASTER_EINVAL;
                break;
            }

            /* Is the request to enable or disable the port capturing? */
            if (request.u.setPortCaptureRequest.isEnable == 1)
            {
                /* Enable Port capturing: */
                if (NetfpMaster_enablePortCapture (ptrNetfpMasterMCB, &request.u.setPortCaptureRequest, &errCode) < 0)
                {
                    /* Error: Enable Port capturing failed. */
                    response.errCode = errCode;
                }
                else
                {
                    /* Port capturing was successful. */
                    response.errCode = 0;
                }
            }
            else
            {
                /* Disable Port capturing: */
                if (NetfpMaster_disablePortCapture (ptrNetfpMasterMCB, &request.u.setPortCaptureRequest, &errCode) < 0)
                {
                    /* Error: Disable Port capturing failed. */
                    response.errCode = errCode;
                }
                else
                {
                    /* Port capturing was successful. */
                    response.errCode = 0;
                }
            }
            break;
        }
        case NetfpMaster_MessageType_ADD_ETHERNET_RULE:
        {
            /* Initialize the ethernet rule configuration */
            memset((void *)&ethRuleCfg, 0, sizeof (Netfp_EthRuleCfg));

            /* Setup user configuration */
            strncpy ((void *)&ethRuleCfg.rmRegionName[0], (void *)&request.u.addEthRuleRequest.rmRegionName[0], NETFP_MAX_CHAR - 1);
            ethRuleCfg.ethType      = request.u.addEthRuleRequest.ethType;
            ethRuleCfg.vlanId       = request.u.addEthRuleRequest.vlanId;
            ethRuleCfg.dstType      = request.u.addEthRuleRequest.dstType;
            ethRuleCfg.anyVlanId    = request.u.addEthRuleRequest.anyVlanId;

            /* Save MAC address */
            memcpy ((void *)&ethRuleCfg.srcMacAddress[0], (void *)&request.u.addEthRuleRequest.srcMacAddress[0], sizeof(ethRuleCfg.srcMacAddress));
            memcpy ((void *)&ethRuleCfg.dstMacAddress[0], (void *)&request.u.addEthRuleRequest.dstMacAddress[0], sizeof(ethRuleCfg.dstMacAddress));

            /* Egress setup */
            if(request.u.addEthRuleRequest.egressIfName[0] != 0)
            {
                /* Save Egress name */
                strncpy ((void *)&ethRuleCfg.egressIfName[0], (void *)&request.u.addEthRuleRequest.egressIfName[0], NETFP_MAX_CHAR - 1);

                /* Find Egress Interface on Master */
                ptrNetfpIfBlock = NetfpMaster_findPhyInterface(ptrNetfpMasterMCB, &ethRuleCfg.egressIfName[0]);
                if(ptrNetfpIfBlock == NULL)
                {
                     NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: Invalid Egress interface name %s\n",
                                     &request.u.addEthRuleRequest.egressIfName[0]);
                     response.errCode = NETFP_MASTER_EINVAL;
                     break;
                }

                /* Pass base queue and base flow configuration from master */
                ethRuleCfg.qosBaseQueue  = ptrNetfpIfBlock->baseQueue;
                ethRuleCfg.qosBaseFlowId = ptrNetfpIfBlock->baseFlowId;
                ethRuleCfg.outPort    =  ptrNetfpIfBlock->switchPort;
            }

            if(request.u.addEthRuleRequest.ingressIfName[0] != 0)
            {
                /* Save Ingress interface name */
                strncpy ((void *)&ethRuleCfg.ingressIfName[0], (void *)&request.u.addEthRuleRequest.ingressIfName[0], NETFP_MAX_CHAR - 1);

                /* Get the ingress switch port number from interface database */
                ptrNetfpIfBlock = NetfpMaster_findPhyInterface(ptrNetfpMasterMCB, &request.u.addEthRuleRequest.ingressIfName[0]);
                if(ptrNetfpIfBlock == NULL)
                {
                     NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: Invalid Ingress interface name %s\n",
                                     &request.u.addEthRuleRequest.egressIfName[0]);
                     response.errCode = NETFP_MASTER_EINVAL;
                     break;
                }

                /* Pass Ingress Ingress port */
                ethRuleCfg.inPort = ptrNetfpIfBlock->switchPort;
            }

            /* Setup system level configuration */
            ethRuleCfg.enableQos    = ptrNetfpMasterMCB->enableEQOS;
            ethRuleCfg.baseQueue    = ptrNetfpMasterMCB->interfaceBaseQueue;
            ethRuleCfg.baseFlow     = ptrNetfpMasterMCB->interfaceBaseFlow;

            /* Check User Stats for this rule if enabled */
            if (ethRuleCfg.numUserStats > NETFP_MAX_USR_STATS_PER_RULE)
            {
                 response.errCode = NETFP_MASTER_EINVAL;
                 break;
            }

            /* Setting up the user stats */
            ethRuleCfg.numUserStats      = request.u.addEthRuleRequest.numUserStats;
            memcpy(&ethRuleCfg.userStatsCfg[0], &request.u.addEthRuleRequest.userStatsCfg[0], sizeof(ethRuleCfg.userStatsCfg));

            /* Call Netfp server API to add Ethernet rule in PA LUT 1-0 */
            if ( (response.u.ethRuleHandle = Netfp_addEthRule(ptrNetfpMasterMCB->netfpServerHandle, &ethRuleCfg, &errCode)) == NULL)
            {
                NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: Unable to add the Ethernet Rule, errCode =%d\n",
                                errCode);
                response.errCode = errCode;
                break;
            }
            break;
        }
        case NetfpMaster_MessageType_DEL_ETHERNET_RULE:
        {
            /* Sanity check of the handle */
            if (request.u.delEthRuleRequest.ethRuleHandle == NULL)
            {
                response.errCode = NETFP_MASTER_EINVAL;
                break;
            }

            /* Request to delete Ethernet rule from netfp server */
            if (Netfp_delEthRule(ptrNetfpMasterMCB->netfpServerHandle, request.u.delEthRuleRequest.ethRuleHandle, &errCode) < 0)
            {
                NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: Unable to deleting the Ethernet Rule handle=%p, errCode =%d\n",
                               request.u.delEthRuleRequest.ethRuleHandle,  errCode);
                response.errCode = errCode;
                break;
            }

            /* Debug Message: */
            NetfpMaster_log(NetfpMaster_LogLevel_DEBUG, "Debug: Deleted Ethernet Rule %p \n", request.u.delEthRuleRequest.ethRuleHandle);
            break;
        }
        case NetfpMaster_MessageType_GET_ETHERNET_RULE_STATS:
        {
            /* Sanity check: Validate the received handle */
            if (request.u.getEthRuleStats.ethRuleHandle == NULL)
            {
                response.errCode = NETFP_MASTER_EINVAL;
                break;
            }

            /* Request to get the user statistics associated with the Ethernet rule. */
            if (Netfp_getEthRuleStats(ptrNetfpMasterMCB->netfpServerHandle, request.u.getEthRuleStats.ethRuleHandle,
                                      &response.u.stats, &errCode) < 0)
            {
                response.errCode = errCode;
                break;
            }
            break;
        }
        case NetfpMaster_MessageType_DISPLAY_ETHERNET_RULE:
        {
            if (Netfp_displayEthRule (ptrNetfpMasterMCB->netfpServerHandle) < 0)
            {
                NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: Unable to display the Ethernet Rule \n") ;
                break;
            }
            break;
        }
        case NetfpMaster_MessageType_GET_NATT_SETTINGS_REQUEST:
        {
             /* Send the NAT-T configuration to NETFP server */
             response.u.nattCfg.udpPort         = ptrNetfpMasterMCB->nattCfg.udpPort;
             response.u.nattCfg.wildCardedEntry = ptrNetfpMasterMCB->nattCfg.wildCardedEntry;
             break;
        }
        case NetfpMaster_MessageType_GET_USRSTATS_SETTINGS_REQUEST:
        {
            /* Send the User statistics configuration to NETFP server */
            memcpy ((void *)&response.u.sysUserStatCfg, (void*)&ptrNetfpMasterMCB->userStatCfg, sizeof(Netfp_SysUserStatCfg));
            break;
        }
        case NetfpMaster_MessageType_GET_INTERFACE_ROUTING_INFO:
        {
             /* Send the base Queue and base flow id to NetFP Server, the following information
              * come from the master config file and should be non-zero values */
             response.u.ifBasedRouteInfo.baseQueue  = ptrNetfpMasterMCB->interfaceBaseQueue;
             response.u.ifBasedRouteInfo.baseFlowId = ptrNetfpMasterMCB->interfaceBaseFlow;
             break;
        }
        case NetfpMaster_MessageType_SET_PRIORITY_OVERRIDE:
        {
            /* Sanity Check: Get the physical interface using the interface name */
            ptrNetfpIfBlock = NetfpMaster_findPhyInterface(ptrNetfpMasterMCB, &request.u.setPriorityOverride.ifName[0]);
            if (ptrNetfpIfBlock == NULL)
            {
                /* Error: Physical interface does not exist in the system */
                response.errCode = NETFP_MASTER_ENOTFOUND;
                break;
            }

            /* Sanity Check: This is valid only if the routing mode is DP-BIT */
            if (ptrNetfpIfBlock->routingMode == NetfpMaster_RoutingMode_DSCP)
            {
                /* Error: Invalid routing mode we cannot accept the message */
                response.errCode = NETFP_MASTER_EINVAL;
                break;
            }

            /* Get the new priority override */
            ptrNetfpIfBlock->priorityOverride = request.u.setPriorityOverride.priorityOverride;

            /* Reconfigure the L2 shaper for the specified NETFP interface block.
             * If the function fails the error code is already populated in the response. */
            NetfpMaster_setupL2Shaper(ptrNetfpMasterMCB, ptrNetfpIfBlock, &response.errCode);
            break;
        }
        case NetfpMaster_MessageType_GET_FRAME_PROTO_CRC_SETTINGS_REQUEST:
        {
             /* Send the Frame Protocol CRC Offload settings to NetFP server */
            response.u.frameProtoInfo.frameProtoCrcOffload = ptrNetfpMasterMCB->frameProtoCrcOffload;
            NetfpMaster_log(NetfpMaster_LogLevel_DEBUG, "Debug: NETFP Master sending response frameProtoCrcOffload: %d\n",
                                response.u.frameProtoInfo.frameProtoCrcOffload);
            break;
        }
        default:
        {
            /* Error: Received an invalid request type (Misbehaving application) */
            NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: NETFP Master received invalid request type %d from %s\n",
                            request.msgType, from.sun_path);
            response.errCode = NETFP_MASTER_EINVAL;
            break;
        }
    }

    /* Send back the response to the application: */
    numBytes = sendto(ptrNetfpMasterMCB->masterMgmtSocket, &response, sizeof(NetfpMaster_Response), 0,
                      (struct sockaddr *)&from, sizeof(from));
    if (numBytes < 0)
    {
        NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: NETFP Master unable to send back the response to %s [Error %s]\n",
                        from.sun_path, strerror(errno));
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to initialize the master management interface
 *      The interface will allows applications to communicate with the
 *      NETFP master
 *
 *  @param[in]  ptrNetfpMasterMCB
 *      Pointer to the NETFP Master
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0
 */
int32_t NetfpMaster_initMgmt(NetfpMaster_MCB* ptrNetfpMasterMCB)
{
    struct sockaddr_un  sockAddress;

    /* Unlink any previous server instances: RM Server names need to be unique in the system anyway. */
    unlink (NETFP_MASTER_SOCKET_NAME);

    /* Create the NETFP Master socket. */
    ptrNetfpMasterMCB->masterMgmtSocket = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (ptrNetfpMasterMCB->masterMgmtSocket < 0)
    {
        NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: NETFP Master socket failed %s\n", strerror(errno));
        return -1;
    }

    /* Initialize the socket address */
    memset(&sockAddress, 0, sizeof(sockAddress));

    /* Populate the binding information */
    sockAddress.sun_family = AF_UNIX;
    snprintf(sockAddress.sun_path, sizeof(sockAddress.sun_path), "%s/%s", Syslib_getRunTimeDirectory(), NETFP_MASTER_SOCKET_NAME);

    /* Bind the server socket: */
    if (bind(ptrNetfpMasterMCB->masterMgmtSocket, (struct sockaddr*)&sockAddress, sizeof(struct sockaddr_un)) < 0)
    {
        NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: Socket bind failed (error: %s)\n", strerror(errno));
        return -1;
    }
    NetfpMaster_log(NetfpMaster_LogLevel_DEBUG, "Debug: NETFP master socket %d is operational\n", ptrNetfpMasterMCB->masterMgmtSocket);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to deinitialize the master management interface
 *
 *  @param[in]  ptrNetfpMasterMCB
 *      Pointer to the NETFP Master
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0
 */
int32_t NetfpMaster_deinitMgmt(NetfpMaster_MCB* ptrNetfpMasterMCB)
{
    /* Close and shutdown the master management socket. */
    close (ptrNetfpMasterMCB->masterMgmtSocket);

    /* Unlink and remove the NETFP master socket name. */
    unlink (NETFP_MASTER_SOCKET_NAME);
    return 0;
}

