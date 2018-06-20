/**
 *   @file  netfp_masterServerIf.c
 *
 *   @brief
 *      NETFP Master and server interface which allows the entities to
 *      communicate with each other and exchange information
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
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <time.h>
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

/* SYSLIB Include files: */
#include <ti/apps/netfp_master/netfp_master.h>
#include <ti/apps/netfp_server/include/netfp_server.h>

/**********************************************************************
 ******************* NETFP Master Server Interface ********************
 **********************************************************************/

/**
 *  @b Description
 *  @n
 *      The function is used to wait for the response for a request which
 *      had been sent out.
 *
 *  @param[in]  ptrNetfpServerMCB
 *      Pointer to the NETFP Server MCB
 *  @param[in]  ptrRequest
 *      Pointer to the request which had been sent out
 *  @param[out] ptrResponse
 *      Pointer to the received response
 *
 *  @retval
 *      Success 	- 0
 *  @retval
 *      Error		- <0
 */
static int32_t NetfpServer_getResponse
(
    NetfpServer_MCB*        ptrNetfpServerMCB,
    NetfpMaster_Request*    ptrRequest,
    NetfpMaster_Response*   ptrResponse
)
{
    int32_t                 numBytes;
    struct sockaddr_un      from;
    socklen_t               fromLen;

    /* Initialize and setup the length of the destination: */
    fromLen = sizeof(struct sockaddr_un);

    /* Wait for the NETFP master to respond. */
    numBytes = recvfrom(ptrNetfpServerMCB->masterSockFd, ptrResponse, sizeof(NetfpMaster_Response), 0, (struct sockaddr *)&from, &fromLen);
    if (numBytes < 0)
    {
        NetfpServer_log(NetfpServer_LogLevel_ERROR, "Error: Unable to receive the response from the NETFP Master [Error %s]\n", strerror(errno));
        return -1;
    }

    /* Sanity Check: Ensure that the response is matching the request */
    if (ptrResponse->msgType != NetfpMaster_MessageType_RESPONSE)
    {
        NetfpServer_log(NetfpServer_LogLevel_ERROR, "Error: Received a non-response packet [%d]\n", ptrResponse->msgType);
        return -1;
    }

    /* Validate the transaction identifier */
    if (ptrRequest->id != ptrResponse->id)
    {
        NetfpServer_log(NetfpServer_LogLevel_ERROR, "Error: Mismatch in the Request [%d] & Response [%d] transaction identifiers\n",
                        ptrRequest->id, ptrResponse->id);
        return -1;
    }

    /* Ensure that the response was for the request */
    if (ptrRequest->msgType != ptrResponse->reqType)
    {
        NetfpServer_log(NetfpServer_LogLevel_ERROR, "Error: Invalid response type [%d] detected [Expected %d]\n",
                        ptrResponse->reqType, ptrRequest->msgType);
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to sent a request to the NETFP master
 *
 *  @param[in]  ptrNetfpServerMCB
 *      Pointer to the NETFP Server MCB
 *  @param[in]  ptrRequest
 *      Pointer to the request which had been sent out
 *
 *  @retval
 *      Success 	- 0
 *  @retval
 *      Error		- <0
 */
static int32_t NetfpServer_sendRequest
(
    NetfpServer_MCB*        ptrNetfpServerMCB,
    NetfpMaster_Request*    ptrRequest
)
{
    struct sockaddr_un      to;
    int32_t                 numBytes;

    /* We always send back the responses to the NETFP Master */
    memset ((void *)&to, 0, sizeof(struct sockaddr_un));
    to.sun_family = AF_UNIX;
    snprintf(to.sun_path, sizeof(to.sun_path), "%s/%s", Syslib_getRunTimeDirectory(), NETFP_MASTER_SOCKET_NAME);

    /* Send the request to the NETFP master */
    numBytes = sendto(ptrNetfpServerMCB->masterSockFd, ptrRequest, sizeof(NetfpMaster_Request), 0, (struct sockaddr *)&to, sizeof(to));
    if (numBytes < 0)
    {
        /* Error: This could imply that the NETFP master is not operational.  */
        NetfpServer_log(NetfpServer_LogLevel_ERROR, "Error: Unable to send the request to the NETFP Master [Error %s]\n", strerror(errno));
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get a list of physical interfaces which exist in the master
 *      and set these interfaces in the NETFP Server.
 *
 *  @param[in]  ptrNetfpServerMCB
 *      Pointer to the NETFP Server MCB
 *
 *  @retval
 *      Success 	- 0
 *  @retval
 *      Error		- <0
 */
static int32_t NetfpServer_setupPhysicalInterfaces(NetfpServer_MCB* ptrNetfpServerMCB)
{
    NetfpMaster_Request     request;
    NetfpMaster_Response    response;
    char                    physicalInterfaceList[NETFP_MASTER_MAX_INTERFACE][NETFP_MASTER_MAX_CHAR];
    int32_t                 phyInterfaceCount=0; //fzm
    int32_t                 index;
    int32_t                 errCode;

    /* Initialize the physical interface list. */
    memset ((void *)&physicalInterfaceList, 0, sizeof(physicalInterfaceList));

    /* Get a list of all the interfaces registered with the NETFP master */
    request.msgType = NetfpMaster_MessageType_GET_INTERFACE_LIST;
    request.id      = random();

    /* Send the request to the NETFP master. */
    if (NetfpServer_sendRequest (ptrNetfpServerMCB, &request) < 0)
        return -1;

    /* Wait for and process the response. */
    if (NetfpServer_getResponse (ptrNetfpServerMCB, &request, &response) < 0)
        return -1;

    /* Did we get a successful response? */
    if (response.errCode == 0)
    {
        /* Cycle through the interface list. */
        phyInterfaceCount = 0;
        while (1)
        {
            /* NULL terminated list: */
            if (response.u.ifName[phyInterfaceCount][0] == 0)
                break;

            /* Copy the interface name from the response */
            strcpy (physicalInterfaceList[phyInterfaceCount], response.u.ifName[phyInterfaceCount]);

            /* Debug Message: */
            NetfpServer_log(NetfpServer_LogLevel_DEBUG, "Debug: %d -> %s \n", phyInterfaceCount, physicalInterfaceList[phyInterfaceCount]);
            phyInterfaceCount++;
        }
    }

    /* Cycle through all the physical interfaces and set the interface markings in the NETFP server. */
    for (index = 0; index < phyInterfaceCount; index++)
    {
        /* Populate the request */
        request.msgType = NetfpMaster_MessageType_GET_INTERFACE_REQUEST;
        request.id      = random();
        strcpy (request.u.getIfRequest.ifName, physicalInterfaceList[index]);

        /* Debug Message: */
        NetfpServer_log(NetfpServer_LogLevel_DEBUG, "Debug: Getting the interface marking map for %s\n", physicalInterfaceList[index]);

        /* Send the request to the NETFP master. */
        if (NetfpServer_sendRequest (ptrNetfpServerMCB, &request) < 0)
            return -1;

        /* Wait for and process the response. */
        if (NetfpServer_getResponse (ptrNetfpServerMCB, &request, &response) < 0)
            return -1;

        /* Set the interface markings in the NETFP server */
        if (Netfp_setupPhyInterface (ptrNetfpServerMCB->netfpServerHandle, physicalInterfaceList[index],
                                     response.u.getIfResponse.switchPort, &response.u.getIfResponse.innerToOuterDSCPMap[0],
                                     &errCode) < 0)
        {
            /* Error: Unable to set the interface markings: */
            NetfpServer_log(NetfpServer_LogLevel_ERROR, "Error: Unable to set the interface marking for %s [Error %d]\n",
                           physicalInterfaceList[index], errCode);
            return -1;
        }
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is the thread which listens for notifications from the
 *      NETFP master
 *
 *  @param[in]  arg
 *      Pointer to the NETFP Server MCB
 *
 *  @retval
 *      Success 	- 0
 *  @retval
 *      Error		- <0
 */
void* NetfpServer_masterMgmtInterfaceThread(void *arg)
{
    int32_t                 numBytes;
    struct sockaddr_un      from;
    socklen_t               fromLen;
    NetfpMaster_Request     request;
    NetfpMaster_Response    response;
    NetfpServer_MCB*        ptrNetfpServerMCB;
    int32_t                 errCode;

    /* Get the pointer to the NETFP Server MCB */
    ptrNetfpServerMCB = (NetfpServer_MCB*)arg;

    /*****************************************************************************************
     * Get the interface based routing information & setup them up in the NETFP Server
     *****************************************************************************************/
    NetfpServer_log(NetfpServer_LogLevel_DEBUG, "Debug: Getting the interface base routing info\n");

    /* Get a list of all the interfaces registered with the NETFP master */
    request.msgType = NetfpMaster_MessageType_GET_INTERFACE_ROUTING_INFO;
    request.id      = random();

    /* Send the request to the NETFP master. */
    if (NetfpServer_sendRequest (ptrNetfpServerMCB, &request) < 0)
        return NULL;

    /* Wait for and process the response. */
    if (NetfpServer_getResponse (ptrNetfpServerMCB, &request, &response) < 0)
        return NULL;

    /* Did we get a successful response? */
    if (response.errCode == 0)
    {
        /* Save routing info on netfp server */
        if (Netfp_initInterfaceRoutingInfo (ptrNetfpServerMCB->netfpServerHandle,
                                            response.u.ifBasedRouteInfo.baseFlowId,
                                            response.u.ifBasedRouteInfo.baseQueue,
                                            &errCode) < 0)
        {
            NetfpServer_log(NetfpServer_LogLevel_DEBUG, "Debug: Initialize interface based routing info on server failed [Error=%d]\n", errCode);
            return NULL;
        }
    }

    /*****************************************************************************************
     * Get the NAT-T configuration from the NETFP Master and setup them in the NETFP Server
     *****************************************************************************************/
    NetfpServer_log(NetfpServer_LogLevel_DEBUG, "Debug: Getting the NAT-T setting\n");

    /* Get NAT-T setting from the NETFP master */
    request.msgType = NetfpMaster_MessageType_GET_NATT_SETTINGS_REQUEST;
    request.id      = random();

    /* Send the request to the NETFP master. */
    if (NetfpServer_sendRequest (ptrNetfpServerMCB, &request) < 0)
        return NULL;

    /* Wait for and process the response. */
    if (NetfpServer_getResponse (ptrNetfpServerMCB, &request, &response) < 0)
        return NULL;

    /* Did we get a successful response? */
    if (response.errCode == 0)
    {
        if (Netfp_updateNattInfo(ptrNetfpServerMCB->netfpServerHandle, &response.u.nattCfg, &errCode) < 0)
        {
            NetfpServer_log(NetfpServer_LogLevel_DEBUG, "Debug: Update NAT-T info on server failed [Error=%d]\n", errCode);
            return NULL;
        }
        NetfpServer_log(NetfpServer_LogLevel_DEBUG, "Debug: Update NAT-T port %d Wild-Carded=%d on server\n",
                        response.u.nattCfg.udpPort, response.u.nattCfg.wildCardedEntry);
    }

    /*****************************************************************************************
     * Get the User Stats configuration from the NETFP Master and setup them in the NETFP Server
     *****************************************************************************************/
    NetfpServer_log(NetfpServer_LogLevel_DEBUG, "Debug: Getting the User Stats setting\n");

    /* Get User statistics configuration from the NETFP master. The NETFP Universe needs to be in synch
     * with each other for this configuration. */
    request.msgType = NetfpMaster_MessageType_GET_USRSTATS_SETTINGS_REQUEST;
    request.id      = random();

    /* Send the request to the NETFP master. */
    if (NetfpServer_sendRequest (ptrNetfpServerMCB, &request) < 0)
        return NULL;

    /* Wait for and process the response. */
    if (NetfpServer_getResponse (ptrNetfpServerMCB, &request, &response) < 0)
        return NULL;

    /* Did we get a successful response? */
    if (response.errCode == 0)
    {
        /* Initialize the user statistics: */
        if (Netfp_initUserStats(ptrNetfpServerMCB->netfpServerHandle, &response.u.sysUserStatCfg, &errCode) < 0)
        {
            NetfpServer_log(NetfpServer_LogLevel_DEBUG, "Debug: Update User Stats info on server failed [Error=%d]\n", errCode);
            return NULL;
        }
        NetfpServer_log(NetfpServer_LogLevel_DEBUG, "Debug: User Stats initialized Total:%d 64bit:%d \n",
                        response.u.sysUserStatCfg.numTotalUserStats, response.u.sysUserStatCfg.num64bUserStats);
    }

    /******************************************************************************************************
     * Get the Frame Protocol CRC Offload Setting from the NETFP Master and setup them in the NETFP Server
     ******************************************************************************************************/
    NetfpServer_log(NetfpServer_LogLevel_DEBUG, "Debug: Getting the Frame Protocol CRC Offload Configuration\n");

    /* Get Frame Protocol CRC Offload setting from the NETFP master */
    request.msgType = NetfpMaster_MessageType_GET_FRAME_PROTO_CRC_SETTINGS_REQUEST;
    request.id      = random();

    /* Send the request to the NETFP master. */
    if (NetfpServer_sendRequest (ptrNetfpServerMCB, &request) < 0)
        return NULL;

    /* Wait for and process the response. */
    if (NetfpServer_getResponse (ptrNetfpServerMCB, &request, &response) < 0)
        return NULL;

    /* Did we get a successful response? */
    if (response.errCode == 0)
    {
        /* YES: Do we need to enable the FP CRC Services */
        if (response.u.frameProtoInfo.frameProtoCrcOffload == 1)
        {
            /* YES: Enable the FP Services */
            if (Netfp_enableFrameProtoCrcServices(ptrNetfpServerMCB->netfpServerHandle, &errCode) < 0)
            {
                NetfpServer_log(NetfpServer_LogLevel_DEBUG, "Debug: Update Frame Protocol CRC Offload setting on server failed [Error=%d]\n", errCode);
                return NULL;
            }
            NetfpServer_log(NetfpServer_LogLevel_DEBUG, "Debug: Enabling FP CRC services on the NETFP Server\n");
        }
        else
        {
            /* NO: Keep the FP Services disabled */
            NetfpServer_log(NetfpServer_LogLevel_DEBUG, "Debug: Disabling FP CRC services on the NETFP Server\n");
        }
    }

    /*****************************************************************************************
     * Get a list of all the physical interfaces from the NETFP Master & set them up
     *****************************************************************************************/
    if (NetfpServer_setupPhysicalInterfaces (ptrNetfpServerMCB) < 0)
        return NULL;

    /* Debug Message: */
    NetfpServer_log(NetfpServer_LogLevel_DEBUG, "Debug: Waiting for interface updates\n");

    /* Initialize and setup the length of the destination: */
    fromLen = sizeof(struct sockaddr_un);
    while (1)
    {
        /* Listen to notifications from the NETFP master */
        numBytes = recvfrom(ptrNetfpServerMCB->masterSockFd, &request, sizeof(NetfpMaster_Request), 0, (struct sockaddr *)&from, &fromLen);
        if (numBytes < 0)
        {
            NetfpServer_log(NetfpServer_LogLevel_ERROR, "Error: Unable to receive notifications from the NETFP Master [Error %s]\n", strerror(errno));
            break;
        }

        /* Sanity Check: Only updates are expected to the NETFP Server */
        if (request.msgType != NetfpMaster_MessageType_UPDATE_NOTICE)
        {
            NetfpServer_log(NetfpServer_LogLevel_ERROR, "Error: Received unexpected message %d from the NETFP Master\n", request.msgType);
            break;
        }

        /* Received a notification from the NETFP Master: Is this because there was a rename command? */
        if (strcmp (request.u.updateIfNotice.ifName, request.u.updateIfNotice.newIfName) != 0)
        {
            /* YES: The physical interface list needs to be recreated from the NETFP Master */
            NetfpServer_log(NetfpServer_LogLevel_DEBUG, "Debug: Interface %s has been renamed to %s\n",
                            request.u.updateIfNotice.ifName, request.u.updateIfNotice.newIfName);

            /* Rename the physical interfaces in the NETFP Server */
            if (Netfp_renamePhysicalInterface (ptrNetfpServerMCB->netfpServerHandle, request.u.updateIfNotice.ifName,
                                               request.u.updateIfNotice.newIfName, &errCode) < 0)
            {
                NetfpServer_log(NetfpServer_LogLevel_ERROR, "Error: Unable to rename physical interface %s to %s [Error code %d]\n",
                                request.u.updateIfNotice.ifName, request.u.updateIfNotice.newIfName, errCode);
            }
            continue;
        }

        /* NO: This is just an rename command. There has been a change in the markings for the interface */
        NetfpServer_log(NetfpServer_LogLevel_DEBUG, "Debug: Received an interface update for %s\n", request.u.updateIfNotice.ifName);

        /* Populate the request */
        request.msgType = NetfpMaster_MessageType_GET_INTERFACE_REQUEST;
        request.id      = random();
        strcpy (request.u.getIfRequest.ifName, request.u.updateIfNotice.ifName);

        /* Debug Message: */
        NetfpServer_log(NetfpServer_LogLevel_DEBUG, "Debug: Getting the interface marking map for %s\n", request.u.getIfRequest.ifName);

        /* Send the request to the NETFP master. */
        if (NetfpServer_sendRequest (ptrNetfpServerMCB, &request) < 0)
            break;

        /* Wait for and process the response. */
        if (NetfpServer_getResponse (ptrNetfpServerMCB, &request, &response) < 0)
            break;

        /* Set the interface markings in the NETFP server */
        if (Netfp_setupPhyInterface (ptrNetfpServerMCB->netfpServerHandle, request.u.getIfRequest.ifName,
                                     response.u.getIfResponse.switchPort, &response.u.getIfResponse.innerToOuterDSCPMap[0],
                                     &errCode) < 0)
        {
            /* Error: Unable to set the interface markings: */
            NetfpServer_log(NetfpServer_LogLevel_ERROR, "Error: Unable to set the interface marking for %s [Error %d]\n",
                            request.u.getIfRequest.ifName, errCode);
            break;
        }
        NetfpServer_log(NetfpServer_LogLevel_DEBUG, "Debug: Updated the interface marking map for %s\n", request.u.getIfRequest.ifName);
    }

    /* Control comes here implies that there was an error: We are exiting the thread and this will imply that the communication
     * between the Master & Server is dead. */
    NetfpServer_log(NetfpServer_LogLevel_ERROR, "Error: Exiting the NETFP Master/Server Interface thread on error\n");
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      The function is used to initialize the interface between the NETFP master
 *      and server. The interface is used to exchange information between the 2 entities
 *
 *  @param[in]  ptrNetfpServerMCB
 *      Pointer to the NETFP Server MCB
 *
 *  @retval
 *      Success 	- 0
 *  @retval
 *      Error		- <0
 */
int32_t NetfpServer_initMasterMgmtInterface (NetfpServer_MCB* ptrNetfpServerMCB)
{
    //https://e2eprivate.ti.com/nokia_siemens_networks/k2_-_fsm4_-_fzm_-_lrc_nokia/f/191/t/6642.aspx
    static const uint32_t    MAX_MASTER_REQUESTS_RETRIES = 5;

    struct sockaddr_un      sockAddress;
    NetfpMaster_Request     request;
    NetfpMaster_Response    response;
    char                    socketName[PATH_MAX];

    /* Create the socket name: */
    snprintf (socketName, PATH_MAX, "%s", ptrNetfpServerMCB->serverName);

    /* Unlink and remove any previous socket names */
    unlink (socketName);

    /* Create the test socket. */
    ptrNetfpServerMCB->masterSockFd = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (ptrNetfpServerMCB->masterSockFd < 0)
    {
        NetfpServer_log(NetfpServer_LogLevel_ERROR, "Error: Unable to create the master management socket [Error: %s]\n", strerror(errno));
        return -1;
    }

    /* Initialize the socket address */
    memset(&sockAddress, 0, sizeof(sockAddress));

    /* Populate the binding information */
    sockAddress.sun_family = AF_UNIX;
    snprintf(sockAddress.sun_path, sizeof(sockAddress.sun_path), "%s/%s", Syslib_getRunTimeDirectory(), socketName);

    /* Bind the socket: */
    if (bind(ptrNetfpServerMCB->masterSockFd, (struct sockaddr*)&sockAddress, sizeof(struct sockaddr_un)) < 0)
    {
        NetfpServer_log(NetfpServer_LogLevel_ERROR, "Error: Socket bind failed [Error: %s]\n", strerror(errno));
        return -1;
    }

    /* Debug Message: */
    NetfpServer_log(NetfpServer_LogLevel_DEBUG, "Debug: Registering server with the NETFP master\n");

    /* Register the NETFP Server with the master to receive notifications */
    request.msgType = NetfpMaster_MessageType_REGISTER_NOTIFICATION;
    request.id      = random();

    /* Send the request to the NETFP master. */
    //https://e2eprivate.ti.com/nokia_siemens_networks/k2_-_fsm4_-_fzm_-_lrc_nokia/f/191/t/6642.aspx
    int32_t result = -1;
    uint32_t retries = 0;
    while (((result = NetfpServer_sendRequest (ptrNetfpServerMCB, &request)) < 0) && (MAX_MASTER_REQUESTS_RETRIES > retries++))
        sleep(1);
    if (result < 0)
        return -1;

    /* Wait for and process the response. */
    if (NetfpServer_getResponse (ptrNetfpServerMCB, &request, &response) < 0)
        return -1;

    /* Was the registeration succesful? */
    if (response.errCode < 0)
    {
        NetfpServer_log(NetfpServer_LogLevel_ERROR, "Error: Notification registeration failed [Error: %d]\n", response.errCode);
        return -1;
    }
    NetfpServer_log(NetfpServer_LogLevel_DEBUG, "Debug: Registered server with the NETFP master\n");
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to initialize the interface between the NETFP master
 *      and server. The interface is used to exchange information between the 2 entities
 *
 *  @param[in]  ptrNetfpServerMCB
 *      Pointer to the NETFP Server MCB
 *
 *  @retval
 *      Success 	- 0
 *  @retval
 *      Error		- <0
 */
int32_t NetfpServer_deinitMasterMgmtInterface(NetfpServer_MCB* ptrNetfpServerMCB)
{
    NetfpMaster_Request     request;
    NetfpMaster_Response    response;

    /* Debug Message: */
    NetfpServer_log(NetfpServer_LogLevel_DEBUG, "Debug: Deregistering server from the NETFP master\n");

    /* Register the NETFP Server with the master to receive notifications */
    request.msgType = NetfpMaster_MessageType_DEREGISTER_NOTIFICATION;
    request.id      = random();

    /* Send the request to the NETFP master. */
    if (NetfpServer_sendRequest (ptrNetfpServerMCB, &request) < 0)
        return -1;

    /* Wait for and process the response. */
    if (NetfpServer_getResponse (ptrNetfpServerMCB, &request, &response) < 0)
        return -1;

    /* Was the deregisteration succesful? */
    if (response.errCode < 0)
        NetfpServer_log(NetfpServer_LogLevel_ERROR, "Error: Notification deregisteration failed [Error: %d]\n", response.errCode);
    else
        NetfpServer_log(NetfpServer_LogLevel_DEBUG, "Debug: Notification deregisteration successful\n");

    /* Close the management socket: */
    close (ptrNetfpServerMCB->masterSockFd);
    return 0;
}

