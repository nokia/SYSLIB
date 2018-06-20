/**
 *   @file  resmgr_sockClient.c
 *
 *   @brief
 *      The file implements the RMv2 client transport interface. The
 *      implementation is done using unix sockets and is available
 *      for ARM applications (which execute the RMv2 client) to
 *      communicate with the SYSLIB-RM Server
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
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <sys/socket.h>
#include <errno.h>
#include <sys/un.h>
#include <unistd.h>

/* PDK Include Files. */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/drv/rm/rm.h>
#include <ti/drv/rm/rm_transport.h>
#include <ti/drv/rm/rm_services.h>

/* Resource Manager Include Files. */
#include <ti/runtime/resmgr/resmgr.h>

/**********************************************************************
 ************************** Local Definitions *************************
 **********************************************************************/
#ifdef __ARMv7
#include <ti/apps/netfp_config/include/NetFP_System_printf.h> //fzm
#else
#include <xdc/runtime/System.h>
#endif

/* This is the maximum size of the RM Client response which can be handled. */
#define RESMGR_CLIENT_MAX_RESPONSE_SIZE         384

/**********************************************************************
 ************************** Local Structures **************************
 **********************************************************************/

/**
 * @brief
 *  Socket MCB
 *
 * @details
 *  This data structure defines the socket transport MCB which
 *  holds all the relevant information for the socket transport
 */
typedef struct ResmgrServer_SocketMCB
{
    /**
     * @brief  RM transport handle
     */
    Rm_TransportHandle  transportHandle;

    /**
     * @brief  Name of the RM Client.
     */
    char                rmClient[RM_NAME_MAX_CHARS];

    /**
     * @brief  Name of the RM Server.
     */
    char                rmServer[RM_NAME_MAX_CHARS];

    /**
     * @brief  RM Client Socket
     */
    int32_t            rmClientSocket;
}ResmgrServer_SocketMCB;

/**********************************************************************
 ***************** Resource Management Client Functions ***************
 **********************************************************************/

/**
 *  @b Description
 *  @n
 *      The function is the registered callback function which is used to allocate
 *      memory for the resource manager transaction.
 *
 *  @param[in]  appTransport
 *      Registered application transport callback function
 *  @param[in]  pktSize
 *      Size of the memory to be allocated
 *  @param[out]  pktHandle
 *      Pointer to the actual packet which is to be sent out.
 *
 *  \ingroup RES_MGR_ARM_INTERNAL_FUN
 *
 *  @retval
 *      Success   - 0
 *  @retval
 *      Error     - <0
 */
static Rm_Packet* Resmgr_transportAlloc
(
    Rm_AppTransportHandle   appTransport,
    uint32_t                pktSize,
    Rm_PacketHandle*        pktHandle
)
{
    Rm_Packet*  ptrRmPacket;

    /* Allocate memory for the packet. */
    ptrRmPacket = malloc (pktSize);
    if (ptrRmPacket == NULL)
    {
        System_printf ("Error: Memory allocation for RM Packet failed\n");
        return NULL;
    }
    ptrRmPacket->pktLenBytes = pktSize;

    /* Store and return the handle. */
    *pktHandle = ptrRmPacket;
    return ptrRmPacket;
}

/**
 *  @b Description
 *  @n
 *      The function is the registered callback function which is used to send a request
 *      from the RM client to the server.
 *
 *  @param[in]  appTransport
 *      Registered application transport callback function
 *  @param[in]  pktHandle
 *      Pointer to the start of the application's transport "packet"
 *
 * \ingroup RES_MGR_ARM_INTERNAL_FUN
 *
 *  @retval
 *      Success   - 0
 *  @retval
 *      Error     - <0
 */
static int32_t Resmgr_transportSendRcv (Rm_AppTransportHandle appTransport, Rm_PacketHandle pktHandle)
{
    ResmgrServer_SocketMCB*     ptrSocketMCB;
    struct sockaddr_un          serveraddr;
    Rm_Packet*                  ptrRmPacket;
    int32_t                     numBytes;
    char*                       ptrRmResponsePacket;
    socklen_t                   fromLen;
    struct sockaddr_un          from;
    int32_t                     rmResult;

    /* Get the pointer to the RM packet which needs to be sent out. */
    ptrRmPacket = (Rm_Packet*)pktHandle;

    /* Get the socket MCB */
    ptrSocketMCB = (ResmgrServer_SocketMCB*)appTransport;

    /* Populate the server configuration */
	memset(&serveraddr, 0, sizeof(serveraddr));
    serveraddr.sun_family = AF_UNIX;
    snprintf(serveraddr.sun_path, sizeof(serveraddr.sun_path), "%s/%s", Syslib_getRunTimeDirectory(), ptrSocketMCB->rmServer);

    /* Send out the configuration */
    numBytes = sendto(ptrSocketMCB->rmClientSocket, ptrRmPacket, ptrRmPacket->pktLenBytes, 0,
                      (struct sockaddr *)&serveraddr, sizeof(serveraddr));
    if (numBytes < 0)
    {
        System_printf("Error: Sending RM request failed with error %s\n", strerror(errno));
        return -1;
    }

    /* Cleanup the memory associated with the RM packet which has been sent out */
    free (ptrRmPacket);

    /* Allocate memory for the response packet. */
    ptrRmResponsePacket = malloc (RESMGR_CLIENT_MAX_RESPONSE_SIZE);
    if (ptrRmResponsePacket == NULL)
    {
        System_printf ("Error: Memory allocation for the response failed\n");
        return -1;
    }

    /* Initialize the length. */
    fromLen = sizeof(struct sockaddr_un);

    /* Once the client has sent the request to the server; we need to loop around
     * and wait for a response also. */
    numBytes = recvfrom(ptrSocketMCB->rmClientSocket, ptrRmResponsePacket, RESMGR_CLIENT_MAX_RESPONSE_SIZE, 0,
                        (struct sockaddr *)&from, &fromLen);
    if (numBytes < 0)
    {
        /* Error: Unable to receive data from the server socket. */
        System_printf("Error: Unable to receive RM response: %s\n", strerror(errno));
        return -1;
    }

    /* Provide packet to RM Client for processing */
    rmResult = Rm_receivePacket(ptrSocketMCB->transportHandle, (Rm_Packet*)ptrRmResponsePacket);
    if (rmResult != RM_OK)
    {
        System_printf("Error: RM failed to process received packet: %d\n", rmResult);
        return -1;
    }

    /* Cleanup the response */
    free (ptrRmResponsePacket);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to setup the RM socket client transport
 *
 *  @param[in]  ptrSystemCfg
 *      System Configuration
 *  @param[in]  rmClientHandle
 *      RMv2 Client Handle
 *  @param[out] errCode
 *      Error code populated by the API
 *
 * \ingroup RES_MGR_ARM_INTERNAL_FUN
 *
 *  @retval
 *      Success   - Opaque handle to the client transport
 *  @retval
 *      Error     - NULL
 */
void* Resmgr_setupClientTransport
(
    Resmgr_SystemCfg*   ptrSystemCfg,
    Rm_Handle           rmClientHandle,
    int32_t*            errCode
)
{
    ResmgrServer_SocketMCB*     ptrSocketMCB;
    Rm_TransportCfg             transportCfg;
    int32_t                     result;
    struct sockaddr_un          sockAddress;

    /* Allocate memory for the socket transport MCB */
    ptrSocketMCB = (ResmgrServer_SocketMCB*)malloc (sizeof(ResmgrServer_SocketMCB));
    if (ptrSocketMCB == NULL)
    {
        *errCode = RESMGR_ENOMEM;
        return NULL;
    }

    /* Initialize the allocated memory */
    memset ((void *)ptrSocketMCB, 0, sizeof(ResmgrServer_SocketMCB));

    /* Populate the client and server name. */
    strcpy (ptrSocketMCB->rmClient, ptrSystemCfg->rmClient);
    strcpy (ptrSocketMCB->rmServer, ptrSystemCfg->rmServer);

    /* Open a client socket; which is used for communication between the client and server */
    ptrSocketMCB->rmClientSocket = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (ptrSocketMCB->rmClientSocket < 0)
    {
        System_printf ("Error: Unable to open the socket for RMv2 client-server communication (error %s)\n", strerror(errno));
        *errCode = errno;
        return NULL;
    }

    /* Populate the binding information */
    memset(&sockAddress, 0, sizeof(sockAddress));
    sockAddress.sun_family = AF_UNIX;
    snprintf (sockAddress.sun_path, sizeof(sockAddress.sun_path), "%s/%s-%d", Syslib_getRunTimeDirectory(), ptrSocketMCB->rmClient, getpid());

    /* Unlink any previous connections */
    unlink(sockAddress.sun_path);

    /* Bind the client socket: */
    if (bind(ptrSocketMCB->rmClientSocket, (struct sockaddr*)&sockAddress, sizeof(struct sockaddr_un)) < 0)
    {
        System_printf ("Error: Socket bind failed (error: %s)", strerror(errno));
        *errCode = errno;
        return NULL;
    }

    /* Initialize the transport configuration */
    memset ((void *)&transportCfg, 0, sizeof(Rm_TransportCfg));

    /* Populate the transport configuration */
    transportCfg.rmHandle                     = rmClientHandle;
    transportCfg.appTransportHandle           = (Rm_AppTransportHandle)ptrSocketMCB;
    transportCfg.remoteInstType               = Rm_instType_SERVER;
    transportCfg.transportCallouts.rmAllocPkt = Resmgr_transportAlloc;
    transportCfg.transportCallouts.rmSendPkt  = Resmgr_transportSendRcv;

    /* Register the transport with the resource manager */
    ptrSocketMCB->transportHandle = Rm_transportRegister(&transportCfg, &result);
    if (result != RM_OK)
    {
        System_printf ("Error: Unable to register the transport [Error Code %d]\n", result);
        *errCode = result;
        return NULL;
    }

    /* Opaque Socket Transport Handle. */
    return (void*)ptrSocketMCB;
}

/**
 *  @b Description
 *  @n
 *      The function is used to delete the client transport
 *
 *  @param[in]  clientTransportHandle
 *      Handle to the client transport
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup RES_MGR_ARM_INTERNAL_FUN
 *
 *  @retval
 *      Success   - 0
 *  @retval
 *      Error     - <0
 */
int32_t Resmgr_deleteClientTransport (void* clientTransportHandle, int32_t* errCode)
{
    ResmgrServer_SocketMCB*     ptrSocketMCB;
    struct sockaddr_un          sockAddress;

    /* Get the client transport handle. */
    ptrSocketMCB = (ResmgrServer_SocketMCB*)clientTransportHandle;
    if (ptrSocketMCB == NULL)
    {
        *errCode = RESMGR_EINVAL;
        return -1;
    }

    /* Deregister the transport */
    *errCode = Rm_transportUnregister (ptrSocketMCB->transportHandle);
    if (*errCode !=  RM_OK)
        return -1;

    /* Close & Unlink the socket. */
    close (ptrSocketMCB->rmClientSocket);
    snprintf (sockAddress.sun_path, sizeof(sockAddress.sun_path), "%s-%d", ptrSocketMCB->rmClient, getpid());
    unlink(sockAddress.sun_path);

    /* Cleanup the memory allocated for the socket transport */
    free (ptrSocketMCB);
    return 0;
}

