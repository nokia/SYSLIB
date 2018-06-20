/**
 *   @file  name_armProxyClient.c
 *
 *   @brief
 *      The file implements the ARM execution realm functionality which
 *      is required for the name Proxy and client to operate in the ARM
 *      execution realm
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2014 Texas Instruments, Inc.
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
#include <sys/socket.h>
#include <sys/un.h>
#include <limits.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

/* SYSLIB Include files */
#include <ti/runtime/name/name.h>
#include <ti/runtime/josh/josh.h>
#include <ti/runtime/name/name_proxyClient.h>
#include <ti/runtime/name/include/name_internal.h>

/* For Debugging only. */
#ifdef __ARMv7
#define System_printf   printf
#else
#include <xdc/runtime/System.h>
#endif

/**************************************************************************
 *********************** Name ARM Realm Structures ************************
 **************************************************************************/

/**
 * @brief   Maximum size of the received JOSH packet size
 */
#define NAME_ARM_REALM_MAX_SIZE         1664

/**************************************************************************
 ********************* Name Client ARM Realm Structures *******************
 **************************************************************************/

/**
 * @brief
 *  Name Tranport
 *
 * @details
 *  The structure describes the Name Transport data structure which
 *  carries information with respect to the data payload which is being
 *  sent/received on the Custom Transport
 */
typedef struct Name_Transport
{
    /**
     * @brief   Data Buffer which carries the actual payload
     */
    uint8_t*        ptrDataBuffer;

    /**
     * @brief   Size of the data buffer.
     */
    uint32_t        size;
}Name_Transport;

/**************************************************************************
 *********************** Name ARM Client Functions ************************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      Josh transport call back function which is invoked by by the JOSH framework
 *      to allocate a message which is to be sent across
 *
 *  @param[in]   nodeId
 *      Node identifier passed during JOSH initialization
 *  @param[in]   size
 *      Size of the message to be allocated
 *  @param[out]  msgBuffer
 *      Opaque handle to the message to be sent out
 *
 *  \ingroup NAME_INTERNAL_ARM_FUNCTION
 *
 *  @retval
 *      Success -   Allocated message
 *  @retval
 *      Error   -   NULL
 */
static uint8_t* Name_armClientJoshAlloc(uint32_t nodeId, int32_t size, void** msgBuffer)
{
    Name_ClientMCB*     ptrNameClient;
    Name_Transport*     ptrNameTransport;

    /* Get the agent client block which is actually the node identifier */
    ptrNameClient = (Name_ClientMCB*)nodeId;

    /* Allocate memory for the Name Transport Block */
    ptrNameTransport = (Name_Transport*)ptrNameClient->cfg.malloc(sizeof(Name_Transport), 0);
    if (ptrNameTransport == NULL)
    {
        printf ("Error: Out of memory error while allocating memory for the client transport\n");
        return NULL;
    }

    /* Allocate memory for the data buffer */
    ptrNameTransport->ptrDataBuffer = (uint8_t*)ptrNameClient->cfg.malloc(size, 0);
    if (ptrNameTransport->ptrDataBuffer == NULL)
    {
        printf ("Error: Out of memory while allocating data buffer for Custom Transport\n");
        ptrNameClient->cfg.free (ptrNameTransport, sizeof(Name_Transport));
        return NULL;
    }

    /* Remember the size of the packet being transported. */
    ptrNameTransport->size = size;

    /* Populate the return values. */
    *msgBuffer = (void*)ptrNameTransport;
    return ptrNameTransport->ptrDataBuffer;
}

/**
 *  @b Description
 *  @n
 *      Josh transport call back function which is invoked by by the JOSH framework
 *      to free a previously allocated message
 *
 *  @param[in]   nodeId
 *      Node identifier passed during JOSH initialization
 *  @param[in]   size
 *      Size of the message to be freed up
 *  @param[in]  msgBuffer
 *      Opaque handle to the message to be cleaned up
 *
 *  \ingroup NAME_INTERNAL_ARM_FUNCTION
 *
 *  @retval
 *      Success -   Allocated message
 *  @retval
 *      Error   -   NULL
 */
static void Name_armClientJoshFree(uint32_t nodeId, int32_t size, void* msgBuffer)
{
    Name_Transport*    ptrNameTransport;
    Name_ClientMCB*    ptrNameClient;

    /* Get the agent client block which is actually the node identifier */
    ptrNameClient = (Name_ClientMCB*)nodeId;

    /* Allocate memory for the Name Transport Block */
    ptrNameTransport = (Name_Transport*)msgBuffer;
    if (ptrNameTransport == NULL)
    {
        printf ("Error: Custom Transport free invoked with invalid MSGBuffer\n");
        return;
    }

    /* Once we are done with sending out the packet. We should clean it up. */
    ptrNameClient->cfg.free (ptrNameTransport->ptrDataBuffer, size);
    ptrNameClient->cfg.free (ptrNameTransport, sizeof(Name_Transport));
    return;
}

/**
 *  @b Description
 *  @n
 *      Josh transport call back function which is invoked by by the JOSH framework
 *      to receive a message.
 *
 *  @param[in]   nodeId
 *      Node identifier passed during JOSH initialization
 *  @param[in]   readerChannel
 *      Opaque reader channel on which the message is to be received
 *  @param[in]  msgBuffer
 *      Opaque handle to the message which has been received
 *  @param[in]  ptrDataBuffer
 *      Data buffer which points to the received message
 *
 *  \ingroup NAME_INTERNAL_ARM_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
static void Name_armClientJoshGet(uint32_t nodeId, void* readerChannel, void** msgBuffer, uint8_t** ptrDataBuffer)
{
    Name_Transport* ptrNameTransport;
    int32_t         retVal;

    /* Allocate memory for the Transport block. */
    *ptrDataBuffer = Name_armClientJoshAlloc(nodeId, NAME_ARM_REALM_MAX_SIZE, (void**)&ptrNameTransport);
    if (*ptrDataBuffer == NULL)
        return;

    /* Receive the data */
    retVal = recv((int32_t)readerChannel, *ptrDataBuffer, ptrNameTransport->size, 0);
    if (retVal < 0)
    {
        /* Error: Failed to receive message from the client socket. Cleanup the allocated memory */
        Name_armClientJoshFree (nodeId, NAME_ARM_REALM_MAX_SIZE, (void *)ptrNameTransport);

        /* Use the error code to determine the severity of the error. The reader channel is
         * created in non-blocking mode so it can fail with an error message of EAGAIN indicating
         * that there was no data available. Any other error code needs to be reported. */
        if (errno == EAGAIN)
        {
            /* No data was received. */
            *msgBuffer     = NULL;
            *ptrDataBuffer = NULL;
            return;
        }
    	perror("Error: Unable to receive message from the JOSH reader channel\n");
        return;
    }

    /* Populate the return fields with the received message */
    *msgBuffer     = (void *)ptrNameTransport;
    return;
}

/**
 *  @b Description
 *  @n
 *      Josh transport call back function which is invoked by by the JOSH framework
 *      to send a message
 *
 *  @param[in]   nodeId
 *      Node identifier passed during JOSH initialization
 *  @param[in]   writerChannel
 *      Opaque writer channel on which the message is to be transmitted
 *  @param[in]  msgBuffer
 *      Opaque handle to the message which has to be sent
 *
 *  \ingroup NAME_INTERNAL_ARM_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
static int Name_armClientJoshPut(uint32_t nodeId, void* writerChannel, void* msgBuffer)
{
    Name_ClientMCB*         ptrNameClient;
    Name_Transport*         ptrNameTransport;
    struct sockaddr_un 	    serveraddr;
    int32_t                 retVal;

    /* Get the Transport Block */
    ptrNameTransport = (Name_Transport*)msgBuffer;
    if (ptrNameTransport == NULL)
    {
        printf ("Error: Custom Transport PUT invoked with invalid MSGBuffer\n");
        return -1;
    }

    /* Get the agent client block which is actually the node identifier */
    ptrNameClient = (Name_ClientMCB*)nodeId;

    /* Get the server socket information to which the packet is being sent. */
	memset(&serveraddr, 0, sizeof(serveraddr));
    serveraddr.sun_family = AF_UNIX;
    snprintf(serveraddr.sun_path, sizeof(serveraddr.sun_path), "%s/%s-proxy", Syslib_getRunTimeDirectory(), ptrNameClient->cfg.proxyName);

    /* Send out the configuration */
    retVal = sendto((int32_t)writerChannel, ptrNameTransport->ptrDataBuffer, ptrNameTransport->size, 0,
                    (struct sockaddr *)&serveraddr, sizeof(serveraddr));
    if (retVal < 0)
    {
    	perror("Error: Sending Name(Josh) data packet failed\n");
        return -1;
    }
    /* Clean up the message once it has been sent out. */
    Name_armClientJoshFree (nodeId, ptrNameTransport->size, ptrNameTransport);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This function is used to create the DSP realm specific transport which is
 *      a MSGCOM channel and connect this with the name proxy by setting up the
 *      JOSH nodes appropriately.
 *
 *  @param[in]  ptrNameClient
 *      Pointer to the name client
 *  @param[out] errCode
 *      Error Code populated by the API
 *
 *  \ingroup NAME_INTERNAL_ARM_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Name_createClientTransport
(
    Name_ClientMCB* ptrNameClient,
    int32_t*        errCode
)
{
    struct sockaddr_un 	    clientaddr;
    int32_t                 clientSocketHandle;
    int32_t                 retVal;
    Josh_NodeCfg            nodeCfg;

    /* Create the socket used by the name client. */
	clientSocketHandle = socket(AF_UNIX, SOCK_DGRAM, 0);
	if (clientSocketHandle < 0)
    {
        /* Error: Name client socket creation failed. */
    	perror("Error: Name client socket creation failed");
        return -1;
	}

    /* Initialize the socket information. */
    memset(&clientaddr, 0, sizeof(clientaddr));

    /* Map the client identifer to a unix socket address */
    clientaddr.sun_family = AF_UNIX;
    snprintf (clientaddr.sun_path, sizeof(clientaddr.sun_path), "%s/%s-%x", Syslib_getRunTimeDirectory(), ptrNameClient->cfg.proxyName, (uint32_t)ptrNameClient);

    /* Unlink any previous client instances: */
    unlink (clientaddr.sun_path);

	/* Bind the client socket. */
    retVal = bind(clientSocketHandle, (struct sockaddr *)&clientaddr, SUN_LEN(&clientaddr));
    if (retVal < 0)
    {
		/* Error: Unable to bind the socket. */
    	perror("Error: Socket BIND Failed\n");
        return -1;
	}

    /* Mark the client as a non blocking socket. */
    if (fcntl(clientSocketHandle, F_SETFL, O_NONBLOCK) < 0)
    {
        perror ("Error: Configuring client socket to non-blocking failed\n");
        return -1;
    }

    /* Remember the socket which has been created: There is only 1 socket for bidirectional
     * communication to the NAME proxy. */
    ptrNameClient->clientReaderChannel = (void*)clientSocketHandle;
    ptrNameClient->serverReaderChannel = NULL;

    /* Initialize the node information. */
    memset ((void*)&nodeCfg, 0, sizeof(Josh_NodeCfg));

    /* Populate the node configuration. */
    nodeCfg.nodeId                     = (uint32_t)ptrNameClient;
    nodeCfg.transportCfg.readerChannel = ptrNameClient->clientReaderChannel;
    nodeCfg.transportCfg.writerChannel = ptrNameClient->clientReaderChannel;

    /* Populate the transport interface */
    nodeCfg.transportCfg.transport.alloc = Name_armClientJoshAlloc;
    nodeCfg.transportCfg.transport.free  = Name_armClientJoshFree;
    nodeCfg.transportCfg.transport.get   = Name_armClientJoshGet;
    nodeCfg.transportCfg.transport.put   = Name_armClientJoshPut;

    /* Populate the OSAL table */
    nodeCfg.malloc      = ptrNameClient->cfg.malloc;
    nodeCfg.free        = ptrNameClient->cfg.free;
    nodeCfg.enterCS     = ptrNameClient->cfg.enterCS;
    nodeCfg.exitCS      = ptrNameClient->cfg.exitCS;
    nodeCfg.createSem   = ptrNameClient->cfg.createSem;
    nodeCfg.deleteSem   = ptrNameClient->cfg.deleteSem;
    nodeCfg.postSem     = ptrNameClient->cfg.postSem;
    nodeCfg.pendSem     = ptrNameClient->cfg.pendSem;

    /* Initialize the JOSH. */
    ptrNameClient->joshHandle = Josh_initNode (&nodeCfg, errCode);
    if (ptrNameClient->joshHandle == NULL)
        return -1;

    return 0;
}

/**
 *  @b Description
 *  @n
 *      This function is used to delete an agent client and deregister this with
 *      the agent server. The function can only be invoked under the following
 *      conditions:
 *          - Application should kill client execution
 *          - Application should ensure that there the client services are no
 *            longer being used.
 *
 *  @param[in]  ptrNameClient
 *      Pointer to the name client
 *  @param[out] errCode
 *      Error Code populated by the API
 *
 *  \ingroup NAME_INTERNAL_ARM_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Name_deleteClientTransport
(
    Name_ClientMCB* ptrNameClient,
    int32_t*        errCode
)
{
    struct sockaddr_un 	    clientaddr;

    /* Ensure that the JOSH services are shutdown. */
    if (Josh_deinitNode (ptrNameClient->joshHandle , errCode) < 0)
    {
        /* Error: JOSH Services failed to shutdown. Use the error code to determine
         * the reason for failure. */
        if (*errCode == JOSH_EBUSY)
        {
            /* This implies that there were still pending jobs in the JOSH node.
             * The prerequisite for the function was not met. */
            *errCode = NAME_ENOTREADY;
            return -1;
        }
        /* Any other error; simply propogate the error code. */
        return -1;
    }

    /* Get the UNIX socket name which is being used by the agent client. */
    snprintf (clientaddr.sun_path, sizeof(clientaddr.sun_path), "%s-%x", ptrNameClient->cfg.proxyName, (uint32_t)ptrNameClient);
    unlink (clientaddr.sun_path);

    /* Kill the socket: */
    close ((int32_t)ptrNameClient->clientReaderChannel);
    return 0;
}

/***************************************************************************************
 * Name Proxy-Client Interface Functions:
 ***************************************************************************************/

/**
 *  @b Description
 *  @n
 *      This function is used to initialize the Name Client-Proxy communication
 *      channels. This communication path is MSGCOM on the DSP realm
 *
 *  @param[in]   ptrNameProxy
 *      Pointer to the name proxy
 *  @param[out] errCode
 *      Error code populated on error.
 *
 *  \ingroup NAME_INTERNAL_ARM_FUNCTION
 *
 *  @retval
 *      Success -   Opaque handle to the communication path
 *  @retval
 *      Error   -   NULL
 */
void* Name_createProxyChannel
(
    Name_ProxyMCB*      ptrNameProxy,
    int32_t*            errCode
)
{
    struct sockaddr_un 	clientaddr;
    int32_t             retVal;
    int32_t             socketHandle;

    /* Create the proxy socket: */
	socketHandle = socket(AF_UNIX, SOCK_DGRAM, 0);
	if (socketHandle < 0)
    {
        /* Error: Proxy Socket creation failed. */
    	perror("Error: Name Proxy socket creation failed");
        *errCode = errno;
        return NULL;
	}

    /* Initialize the socket information. */
    memset(&clientaddr, 0, sizeof(clientaddr));

    /* Use the well known PROXY name to advertize the services. */
    snprintf(clientaddr.sun_path, sizeof(clientaddr.sun_path), "%s/%s-proxy", Syslib_getRunTimeDirectory(), ptrNameProxy->cfg.proxyName);
    clientaddr.sun_family = AF_UNIX;

    /* Unlink any previous proxy instances: */
    unlink (clientaddr.sun_path);

	/* Bind the control socket. */
    retVal = bind(socketHandle, (struct sockaddr *)&clientaddr, SUN_LEN(&clientaddr));
    if (retVal < 0)
    {
		/* Error: Unable to bind the socket. */
    	perror("Error: Name proxy server socket bind failed");
        *errCode = errno;
        return NULL;
	}

    /* Set the proxy socket to operate in non blocking mode. */
    if (fcntl(socketHandle, F_SETFL, O_NONBLOCK) < 0)
    {
        perror ("Error: Configuring name proxy socket in non-blocking failed\n");
        *errCode = errno;
        return NULL;
    }

    /* Return the created socket handle. */
    return (void*)socketHandle;
}

/**
 *  @b Description
 *  @n
 *      This function is used by the server to receive messages from the clients
 *      in the ARM realm
 *
 *  @param[in]  ptrNameProxy
 *      Pointer to the name proxy
 *  @param[out] ptrDataBuffer
 *      Pointer to the data buffer of the received message
 *  @param[out] dataLen
 *      Data length of the received message
 *  @param[out] msgBuffer
 *      Message buffer associated with the received message
 *  @param[out] fromClientId
 *      Client identifer from where the packet was received.
 *
 *  \ingroup NAME_INTERNAL_ARM_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Name_receiveFromClient
(
    Name_ProxyMCB*      ptrNameProxy,
    uint8_t**           ptrDataBuffer,
    uint32_t*           dataLen,
    uint32_t*           msgBuffer,
    uint32_t*           fromClientId
)
{
    uint8_t*                    ptrRxDataBuffer;
	socklen_t    		        fromLen;
	struct sockaddr_un 	        from;
    int32_t                     numBytes;

    /* Allocate memory for the data buffer. */
    ptrRxDataBuffer = ptrNameProxy->cfg.malloc (NAME_ARM_REALM_MAX_SIZE, 0);
    if (ptrRxDataBuffer == NULL)
        return -1;

    /* Initialize the arguments. */
	from.sun_family = AF_UNIX;
    fromLen = sizeof(from);

    /* Receive data from the server socket. */
	numBytes = recvfrom((int32_t)ptrNameProxy->localRealmServerChannel, ptrRxDataBuffer, NAME_ARM_REALM_MAX_SIZE, 0,
                        (struct sockaddr *)&from, &fromLen);
    if (numBytes < 0)
	{
        /* Error: Failed to receive message from the server socket. This could occur since the socket
         * was created in the non-blocking mode. */
        if (errno == EAGAIN)
        {
            /* No data was received. */
            ptrNameProxy->cfg.free (ptrRxDataBuffer, NAME_ARM_REALM_MAX_SIZE);
            *dataLen = 0;
            return 0;
        }
        /* Error: Unable to receive data from the server socket. */
    	perror("Error: Receive data from the name proxy socket failed");
        *dataLen = 0;
		return -1;
    }

    /* Did we get any data? */
    if (numBytes > 0)
    {
        /* Data was received on the server socket. */
        *dataLen          = numBytes;
        *msgBuffer        = (uint32_t)ptrRxDataBuffer;
        *ptrDataBuffer    = ptrRxDataBuffer;
        *fromClientId     = Josh_getNodeId(ptrRxDataBuffer);
        return 0;
    }

    /* No data was received on the server socket. */
    *dataLen          = 0;
    *msgBuffer        = 0;
    *ptrDataBuffer    = 0;
    *fromClientId     = 0xFFFFFFFF;
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This function is used by the server to send messages to the clients
 *      in the ARM realm
 *
 *  @param[in]  ptrNameProxy
 *      Pointer to the name proxy
 *  @param[in]  ptrTxDataBuffer
 *      Pointer to the data buffer of the message to be sent out
 *  @param[in]  txDataLen
 *      Data length of the message to be sent
 *  @param[in]  toClientId
 *      Client Id to which the message is being sent.
 *
 *  \ingroup NAME_INTERNAL_ARM_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Name_sendToClient
(
    Name_ProxyMCB*      ptrNameProxy,
    uint8_t*            ptrTxDataBuffer,
    uint32_t            txDataLen,
    uint32_t            toClientId
)
{
    struct sockaddr_un 	        clientaddr;

    /* Initialize the socket information. */
    memset(&clientaddr, 0, sizeof(clientaddr));

    /* Map the client identifer to the unix socket address */
    clientaddr.sun_family = AF_UNIX;
    snprintf (clientaddr.sun_path, sizeof(clientaddr.sun_path), "%s/%s-%x", Syslib_getRunTimeDirectory(), ptrNameProxy->cfg.proxyName, toClientId);

    /* Send the packet to the agent client. */
    if (sendto((int32_t)ptrNameProxy->localRealmServerChannel, ptrTxDataBuffer, txDataLen, 0,
               (struct sockaddr *)&clientaddr, sizeof(clientaddr)) < 0)
    {
        perror("Error: Unable to send back the response to the name client");
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This function is used by the server to free messages received from clients
 *      in the ARM realm
 *
 *  @param[in] ptrNameProxy
 *      Pointer to the name proxy
 *  @param[in] msgBuffer
 *      Message buffer to be cleaned up
 *
 *  \ingroup NAME_INTERNAL_ARM_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Name_freePacket(Name_ProxyMCB* ptrNameProxy, uint32_t msgBuffer)
{
    /* In the ARM realm the message buffer is 1-1 mapped to the data buffer. So
     * we simply clean the data buffer */
    ptrNameProxy->cfg.free ((uint8_t*)msgBuffer, NAME_ARM_REALM_MAX_SIZE);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This function is used to deinitialize the Name Proxy communication
 *      channels.
 *
 *  @param[in]  localCommChannel
 *      Pointer to the local communication channel
 *  @param[in]  serverName
 *      Name of the agent server
 *  @param[out] errCode
 *      Error code populated on error.
 *
 *  \ingroup NAME_INTERNAL_ARM_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Name_deleteProxyChannel (void* localCommChannel, const char* serverName, int32_t* errCode)
{
    int32_t socketHandle;
    char    serverAddress[NAME_MAX];

    /* Get the socket handle. */
	socketHandle = (int32_t)localCommChannel;

    /* Unlink the socket name */
    snprintf(serverAddress, NAME_MAX, "%s-proxy", serverName);
    unlink (serverAddress);

    /* Close the socket */
    close (socketHandle);
    return 0;
}

