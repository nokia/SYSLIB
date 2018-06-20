/**
 *   @file  test.c
 *
 *   @brief
 *      NETFP Proxy Test Mode implementation. This is a test simulation
 *      which executes various networking scenarios and then uses the
 *      Plugin IPC command interface to send messages to the NETFP Proxy
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
#include <stddef.h>
#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <pthread.h>
#include <getopt.h>
#include <time.h>
#include <signal.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/un.h>
#include <dlfcn.h>
#include <dirent.h>

/* NETFP Proxy includes */
//#include <ti/apps/netfp_proxy/include/netfp_proxy_pvt.h>
#include <ti/runtime/netfp/netfp.h>
#include <ti/apps/netfp_proxy/netfp_proxy.h>
#include <ti/apps/netfp_proxy/netfp_proxy_ipc.h>

/* MCSDK Include files. */
#include <ti/runtime/hplib/hplib.h>

/**********************************************************************
 ************************ Global Variables ****************************
 **********************************************************************/

/* Test IPC Socket created and used to communicate with the plugin */
int32_t                         testCoreIPCSocket;

/* Proxy socket address: IPC socket will send messages to this address */
struct sockaddr_un              proxySunAddr;

/* Global NETFP Client Handle which is passed to the Test task during initialization */
Netfp_ClientHandle              testNetfpClientHandle;

/* Proxy registered to perform route lookups: In a real world use case the route
 * resolution requests will be performed by the NETFP Server but for the use case
 * we are trying to be self reliant so we are doing this by ourselves */
Netfp_ProxyServerIfFunction     testProxyServerIfFxn;

/**********************************************************************
 ************************ Extern definitions **************************
 **********************************************************************/
extern int32_t NetfpProxy_routeGetNumEntries(void);
extern int32_t NetfpProxy_neighGetNumEntries(void);

/**********************************************************************
 ************************* Test Functions *****************************
 **********************************************************************/

/**
 *  @b Description
 *  @n
 *      The function is used by the test module to send an IPC message to create an
 *      interface.
 *
 *  @param[in]  ifName
 *      Interface name
 *  @param[out] errCode
 *      Optional interface name of the bridge port2
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t NetfpProxy_testCreateInterface (const char* ifName)
{
    NetfpProxy_msg      requestMsg;
    NetfpProxy_msg      responseMsg;
    int32_t             msgLen;
    struct sockaddr_un  fromAddr;
    uint32_t            destAddrLen;

    /* Initialize the request message: */
    memset ((void *)&requestMsg, 0, sizeof(NetfpProxy_msg));

    /* Populate the request message: */
    requestMsg.hdr.msgType = NETFP_PROXY_IPC_MSGTYPE_ADD_INTERFACE_REQ;
    requestMsg.hdr.transId = 100;
    strcpy (requestMsg.body.ifaceAddReq.interfaceName, ifName);

    /* Send the message through the IPC interface to the PROXY core: */
    if (sendto (testCoreIPCSocket, (void *)&requestMsg, sizeof (NetfpProxy_msg), 0,
                (const struct sockaddr *)&proxySunAddr, sizeof (struct sockaddr_un)) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to send the message [Error %s]\n", strerror(errno));
        return -1;
    }

    /* Initialize the response */
    memset ((void *)&responseMsg, 0, sizeof(NetfpProxy_msg));
    msgLen      = sizeof(NetfpProxy_msg);
    destAddrLen = sizeof(struct sockaddr_un);

    /* Wait for the response */
    if (recvfrom (testCoreIPCSocket, (void *)&responseMsg, msgLen, 0, (struct sockaddr *)&fromAddr, &destAddrLen) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to receive the message [Error %s]\n", strerror(errno));
        return -1;
    }

    /* Validate the response */
    if (responseMsg.hdr.msgType != NETFP_PROXY_IPC_MSGTYPE_ADD_INTERFACE_RESP)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Received invalid response %d\n", responseMsg.hdr.msgType);
        return -1;
    }

    /* Was the response successful? */
    if (responseMsg.body.ifaceAddResp.retVal < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to add the interface [Error %d NETFP Error %d]\n",
                           responseMsg.body.ifaceAddResp.retVal, responseMsg.body.ifaceAddResp.netfpErrCode);
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used by the test module to send an IPC message to delete an
 *      interface.
 *
 *  @param[in]  ifName
 *      Interface name
 *  @param[out] errCode
 *      Optional interface name of the bridge port2
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t NetfpProxy_testDeleteInterface (const char* ifName)
{
    NetfpProxy_msg      requestMsg;
    NetfpProxy_msg      responseMsg;
    int32_t             msgLen;
    struct sockaddr_un  fromAddr;
    uint32_t            destAddrLen;

    /* Initialize the request message: */
    memset ((void *)&requestMsg, 0, sizeof(NetfpProxy_msg));

    /* Populate the request message: */
    requestMsg.hdr.msgType = NETFP_PROXY_IPC_MSGTYPE_DEL_INTERFACE_REQ;
    requestMsg.hdr.transId = 101;
    strcpy (requestMsg.body.ifaceDelReq.interfaceName, ifName);

    /* Send the message through the IPC interface to the PROXY core: */
    if (sendto (testCoreIPCSocket, (void *)&requestMsg, sizeof (NetfpProxy_msg), 0,
                (const struct sockaddr *)&proxySunAddr, sizeof (struct sockaddr_un)) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to send the message [Error %s]\n", strerror(errno));
        return -1;
    }

    /* Initialize the response */
    memset ((void *)&responseMsg, 0, sizeof(NetfpProxy_msg));
    msgLen      = sizeof(NetfpProxy_msg);
    destAddrLen = sizeof(struct sockaddr_un);

    /* Wait for the response */
    if (recvfrom (testCoreIPCSocket, (void *)&responseMsg, msgLen, 0, (struct sockaddr *)&fromAddr, &destAddrLen) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to receive the message [Error %s]\n", strerror(errno));
        return -1;
    }

    /* Validate the response */
    if (responseMsg.hdr.msgType != NETFP_PROXY_IPC_MSGTYPE_DEL_INTERFACE_RESP)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Received invalid response %d\n", responseMsg.hdr.msgType);
        return -1;
    }

    /* Was the response successful? */
    if (responseMsg.body.ifaceDelResp.retVal < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to delete the interface [Error %d NETFP Error %d]\n",
                           responseMsg.body.ifaceAddResp.retVal, responseMsg.body.ifaceAddResp.netfpErrCode);
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to configure the L3 Shaper
 *
 *  @param[in]  ifName
 *      Interface name
 *  @param[in]  ptrL3QoSCfg
 *      Pointer to the L3 QOS configuration
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t NetfpProxy_testConfigureL3Shaper (const char* ifName, Netfp_L3QoSCfg* ptrL3QoSCfg)
{
    NetfpProxy_msg      requestMsg;
    NetfpProxy_msg      responseMsg;
    int32_t             msgLen;
    struct sockaddr_un  fromAddr;
    uint32_t            destAddrLen;

    /* Initialize the request message: */
    memset ((void *)&requestMsg, 0, sizeof(NetfpProxy_msg));

    /* Populate the request message: */
    requestMsg.hdr.msgType = NETFP_PROXY_IPC_MSGTYPE_CONFIGURE_L3_QOS_REQ;
    requestMsg.hdr.transId = 102;
    strcpy (requestMsg.body.configL3QosReq.interfaceName, ifName);
    memcpy ((void *)&requestMsg.body.configL3QosReq.l3QoSCfg, (void*)ptrL3QoSCfg, sizeof(Netfp_L3QoSCfg));

    /* Send the message through the IPC interface to the PROXY core: */
    if (sendto (testCoreIPCSocket, (void *)&requestMsg, sizeof (NetfpProxy_msg), 0,
                (const struct sockaddr *)&proxySunAddr, sizeof (struct sockaddr_un)) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to send the message [Error %s]\n", strerror(errno));
        return -1;
    }

    /* Initialize the response */
    memset ((void *)&responseMsg, 0, sizeof(NetfpProxy_msg));
    msgLen      = sizeof(NetfpProxy_msg);
    destAddrLen = sizeof(struct sockaddr_un);

    /* Wait for the response */
    if (recvfrom (testCoreIPCSocket, (void *)&responseMsg, msgLen, 0, (struct sockaddr *)&fromAddr, &destAddrLen) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to receive the message [Error %s]\n", strerror(errno));
        return -1;
    }

    /* Validate the response */
    if (responseMsg.hdr.msgType != NETFP_PROXY_IPC_MSGTYPE_CONFIGURE_L3_QOS_RESP)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Received invalid response %d\n", responseMsg.hdr.msgType);
        return -1;
    }

    /* Was the response successful? */
    if (responseMsg.body.configL3QosResp.retVal < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to configure the L3 QoS [Error %d NETFP Error %d]\n",
                           responseMsg.body.configL3QosResp.retVal, responseMsg.body.configL3QosResp.netfpErrCode);
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to flush the VLAN egress map
 *
 *  @param[in]  ifName
 *      Interface name
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t NetfpProxy_testFlushVLANEgressMap (const char* ifName)
{
    NetfpProxy_msg      requestMsg;
    NetfpProxy_msg      responseMsg;
    int32_t             msgLen;
    struct sockaddr_un  fromAddr;
    uint32_t            destAddrLen;

    /* Initialize the request message: */
    memset ((void *)&requestMsg, 0, sizeof(NetfpProxy_msg));

    /* Populate the request message: */
    requestMsg.hdr.msgType = NETFP_PROXY_IPC_MSGTYPE_FLUSH_VLAN_PRIORITY_REQ;
    requestMsg.hdr.transId = 103;
    strcpy (requestMsg.body.flushVlanPriorityReq.interfaceName, ifName);

    /* Send the message through the IPC interface to the PROXY core: */
    if (sendto (testCoreIPCSocket, (void *)&requestMsg, sizeof (NetfpProxy_msg), 0,
                (const struct sockaddr *)&proxySunAddr, sizeof (struct sockaddr_un)) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to send the message [Error %s]\n", strerror(errno));
        return -1;
    }

    /* Initialize the response */
    memset ((void *)&responseMsg, 0, sizeof(NetfpProxy_msg));
    msgLen      = sizeof(NetfpProxy_msg);
    destAddrLen = sizeof(struct sockaddr_un);

    /* Wait for the response */
    if (recvfrom (testCoreIPCSocket, (void *)&responseMsg, msgLen, 0, (struct sockaddr *)&fromAddr, &destAddrLen) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to receive the message [Error %s]\n", strerror(errno));
        return -1;
    }

    /* Validate the response */
    if (responseMsg.hdr.msgType != NETFP_PROXY_IPC_MSGTYPE_FLUSH_VLAN_PRIORITY_RESP)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Received invalid response %d\n", responseMsg.hdr.msgType);
        return -1;
    }

    /* Was the response successful? */
    if (responseMsg.body.flushVlanPriorityResp.retVal < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to flush the VLAN Egress map [Error %d NETFP Error %d]\n",
                           responseMsg.body.flushVlanPriorityResp.retVal, responseMsg.body.flushVlanPriorityResp.netfpErrCode);
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used by the test module to send an IPC message to flush the routes
 *      and for the server to perform a route recomputation
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t NetfpProxy_testFlushRoute (void)
{
    NetfpProxy_msg      requestMsg;
    NetfpProxy_msg      responseMsg;
    int32_t             msgLen;
    struct sockaddr_un  fromAddr;
    uint32_t            destAddrLen;

    /* Initialize the request message: */
    memset ((void *)&requestMsg, 0, sizeof(NetfpProxy_msg));

    /* Populate the request message: */
    requestMsg.hdr.msgType = NETFP_PROXY_IPC_MSGTYPE_IP_ROUTE_FLUSH_REQ;
    requestMsg.hdr.transId = 104;

    /* Send the message through the IPC interface to the PROXY core: */
    if (sendto (testCoreIPCSocket, (void *)&requestMsg, sizeof (NetfpProxy_msg), 0,
                (const struct sockaddr *)&proxySunAddr, sizeof (struct sockaddr_un)) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to send the message [Error %s]\n", strerror(errno));
        return -1;
    }

    /* Initialize the response */
    memset ((void *)&responseMsg, 0, sizeof(NetfpProxy_msg));
    msgLen      = sizeof(NetfpProxy_msg);
    destAddrLen = sizeof(struct sockaddr_un);

    /* Wait for the response */
    if (recvfrom (testCoreIPCSocket, (void *)&responseMsg, msgLen, 0, (struct sockaddr *)&fromAddr, &destAddrLen) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to receive the message [Error %s]\n", strerror(errno));
        return -1;
    }

    /* Validate the response */
    if (responseMsg.hdr.msgType != NETFP_PROXY_IPC_MSGTYPE_IP_ROUTE_FLUSH_RESP)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Received invalid response %d\n", responseMsg.hdr.msgType);
        return -1;
    }

    /* Was the response successful? */
    if (responseMsg.body.ipRouteFlushResp.retVal < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to flush the routes [Error %d NETFP Error %d]\n",
                           responseMsg.body.ipRouteFlushResp.retVal, responseMsg.body.ipRouteFlushResp.netfpErrCode);
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to configure and validate the L3 shaper configuration.
 *
 *  @param[in]  ifHandle
 *      Interface handle
 *  @param[in]  baseQueueId
 *      Base L3 QOS Identifier
 *  @param[in]  ifBridgePort1Name
 *      Optional interface name of the bridge port1
 *  @param[in]  ifBridgePort2Name
 *      Optional interface name of the bridge port2
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t NetfpProxy_validateL3Shaper
(
    const char* ifName,
    uint32_t    baseQueueId,
    const char* ifBridgePort1Name,
    const char* ifBridgePort2Name
)
{
    Netfp_L3QoSCfg      l3QoSCfg;
    Netfp_L3QoSCfg      l3QoSCfgRetreieved;
    Netfp_IfHandle      ifHandle;
    Netfp_OptionTLV     optCfg;
    Netfp_IfHandle      ifBridgePortHandle1 = NULL;
    Netfp_IfHandle      ifBridgePortHandle2 = NULL;
    int32_t             errCode;
    char                bridgedPortName[NETFP_MAX_CHAR];

    /* Find the interface handles for each of the names specified: */
    ifHandle = Netfp_findInterface (testNetfpClientHandle, ifName, NULL, &errCode);
    if (ifHandle == NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to find the interface %s [Error code %d]\n", ifName, errCode);
        return -1;
    }

    /* Are there any bridged port interfaces specified? */
    if (ifBridgePort1Name != NULL)
    {
        /* YES: Find the bridged port1 handle */
        snprintf (bridgedPortName, NETFP_MAX_CHAR, "%s->%s", ifName, ifBridgePort1Name);
        ifBridgePortHandle1 = Netfp_findInterface (testNetfpClientHandle, bridgedPortName, NULL, &errCode);
        if (ifBridgePortHandle1 == NULL)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to find the interface %s [Error code %d]\n", bridgedPortName, errCode);
            return -1;
        }
    }
    if (ifBridgePort2Name != NULL)
    {
        /* YES: Find the bridged port2 handle */
        snprintf (bridgedPortName, NETFP_MAX_CHAR, "%s->%s", ifName, ifBridgePort2Name);
        ifBridgePortHandle2 = Netfp_findInterface (testNetfpClientHandle, bridgedPortName, NULL, &errCode);
        if (ifBridgePortHandle2 == NULL)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to find the interface %s [Error code %d]\n", bridgedPortName, errCode);
            return -1;
        }
    }

    /* Initialize the L3 QOS Configuration: */
    memset ((void *)&l3QoSCfg, 0, sizeof(Netfp_L3QoSCfg));

    /* Populate the L3 QOS Configuration: */
    l3QoSCfg.isEnable = 1;
    l3QoSCfg.flowId   = 1;
    l3QoSCfg.qid[0]   = baseQueueId;
    l3QoSCfg.qid[1]   = baseQueueId + 1;

    /* Configure the L3 Shaper */
    if (NetfpProxy_testConfigureL3Shaper (ifName, &l3QoSCfg) < 0)
        return -1;

    /* Reset the L3 QoS Configuration: */
    memset ((void*)&l3QoSCfgRetreieved, 0, sizeof(Netfp_L3QoSCfg));

    /**************************************************************************************************************
     * Validations: Ensure that the NETFP Server is configured correctly
     **************************************************************************************************************/
    optCfg.type   = Netfp_Option_L3_QOS;
    optCfg.length = sizeof(Netfp_L3QoSCfg);
    optCfg.value  = &l3QoSCfgRetreieved;
    if (Netfp_getIfOpt (testNetfpClientHandle, ifHandle, &optCfg, &errCode) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to get the L3 Shaper configuration on %s [Error code %d]\n", ifName, errCode);
        return -1;
    }

    /* Is this what we had configured? */
    if (memcmp ((void *)&l3QoSCfgRetreieved, (void*)&l3QoSCfg, sizeof(Netfp_L3QoSCfg)) != 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Mismatch in the retreived L3 QoS configuration for %s [%d %d %d %d]\n",
                           ifName, l3QoSCfg.isEnable, l3QoSCfg.flowId, l3QoSCfg.qid[0], l3QoSCfg.qid[1]);
        return -1;
    }

    /**************************************************************************************************************
     * Validations: If there are bridged port interfaces then the PROXY API should have configured the L3 Shaper
     * on all the bridged port interfaces.
     **************************************************************************************************************/
    if (ifBridgePortHandle1 != NULL)
    {
        /* Reset the L3 QoS Configuration: */
        memset ((void*)&l3QoSCfgRetreieved, 0, sizeof(Netfp_L3QoSCfg));

        /* Get the L3 QOS configuration from the NETFP Server */
        if (Netfp_getIfOpt (testNetfpClientHandle, ifBridgePortHandle1, &optCfg, &errCode) < 0)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to get the L3 Shaper configuration on %s [Error code %d]\n",
                               ifBridgePort1Name, errCode);
            return -1;
        }

        /* Is this what we had configured? */
        if (memcmp ((void *)&l3QoSCfgRetreieved, (void*)&l3QoSCfg, sizeof(Netfp_L3QoSCfg)) != 0)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Mismatch in the retreived L3 QoS configuration for %s [%d %d %d %d]\n",
                               ifBridgePort1Name, l3QoSCfg.isEnable, l3QoSCfg.flowId, l3QoSCfg.qid[0], l3QoSCfg.qid[1]);
            return -1;
        }
    }
    if (ifBridgePortHandle2 != NULL)
    {
        /* Reset the L3 QoS Configuration: */
        memset ((void*)&l3QoSCfgRetreieved, 0, sizeof(Netfp_L3QoSCfg));

        /* Get the L3 QOS configuration from the NETFP Server */
        if (Netfp_getIfOpt (testNetfpClientHandle, ifBridgePortHandle2, &optCfg, &errCode) < 0)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to get the L3 Shaper configuration on %s [Error code %d]\n",
                               ifBridgePort2Name, errCode);
            return -1;
        }

        /* Is this what we had configured? */
        if (memcmp ((void *)&l3QoSCfgRetreieved, (void*)&l3QoSCfg, sizeof(Netfp_L3QoSCfg)) != 0)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Mismatch in the retreived L3 QoS configuration for %s [%d %d %d %d]\n",
                               ifBridgePort2Name, l3QoSCfg.isEnable, l3QoSCfg.flowId, l3QoSCfg.qid[0], l3QoSCfg.qid[1]);
            return -1;
        }
    }
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: L3 Shaper configuration [Base Queue: %d] validated for %s %s %s\n",
                       baseQueueId, ifName, ifBridgePort1Name, ifBridgePort2Name);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to validate the MTU
 *
 *  @param[in]  ifHandle
 *      Interface handle
 *  @param[in]  mtu
 *      Configured MTU
 *  @param[in]  ifBridgePort1Name
 *      Optional interface name of the bridge port1
 *  @param[in]  ifBridgePort2Name
 *      Optional interface name of the bridge port2
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t NetfpProxy_validateMTU
(
    const char* ifName,
    uint32_t    mtu,
    const char* ifBridgePort1Name,
    const char* ifBridgePort2Name
)
{
    uint32_t            mtuRetreieved;
    Netfp_IfHandle      ifHandle;
    Netfp_OptionTLV     optCfg;
    Netfp_IfHandle      ifBridgePortHandle1 = NULL;
    Netfp_IfHandle      ifBridgePortHandle2 = NULL;
    int32_t             errCode;
    char                bridgedPortName[NETFP_MAX_CHAR];

    /* Find the interface handles for each of the names specified: */
    ifHandle = Netfp_findInterface (testNetfpClientHandle, ifName, NULL, &errCode);
    if (ifHandle == NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to find the interface %s [Error code %d]\n", ifName, errCode);
        return -1;
    }

    /* Are there any bridged port interfaces specified? */
    if (ifBridgePort1Name != NULL)
    {
        /* YES: Find the bridged port1 handle */
        snprintf (bridgedPortName, NETFP_MAX_CHAR, "%s->%s", ifName, ifBridgePort1Name);
        ifBridgePortHandle1 = Netfp_findInterface (testNetfpClientHandle, bridgedPortName, NULL, &errCode);
        if (ifBridgePortHandle1 == NULL)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to find the interface %s [Error code %d]\n", bridgedPortName, errCode);
            return -1;
        }
    }
    if (ifBridgePort2Name != NULL)
    {
        /* YES: Find the bridged port2 handle */
        snprintf (bridgedPortName, NETFP_MAX_CHAR, "%s->%s", ifName, ifBridgePort2Name);
        ifBridgePortHandle2 = Netfp_findInterface (testNetfpClientHandle, bridgedPortName, NULL, &errCode);
        if (ifBridgePortHandle2 == NULL)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to find the interface %s [Error code %d]\n", bridgedPortName, errCode);
            return -1;
        }
    }

    /**************************************************************************************************************
     * Validations: Ensure that the NETFP Server is configured correctly
     **************************************************************************************************************/
    optCfg.type     = Netfp_Option_MTU;
    optCfg.length   = 4;
    optCfg.value    = &mtuRetreieved;
    if (Netfp_getIfOpt (testNetfpClientHandle, ifHandle, &optCfg, &errCode) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Getting MTU failed for %s [Error code %d]\n", ifName, errCode);
        return -1;
    }
    if (mtuRetreieved != mtu)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Expected MTU %d Got MTU %d for Interface %s\n", mtu, mtuRetreieved, ifName);
        return -1;
    }

    /**************************************************************************************************************
     * Validations: If there are bridged port interfaces then the PROXY API should have configured the L3 Shaper
     * on all the bridged port interfaces.
     **************************************************************************************************************/
    if (ifBridgePortHandle1 != NULL)
    {
        /* Reset the retreived MTU */
        mtuRetreieved = 0;

        /* Get the MTU configuration for the bridged port interface from the NETFP Server */
        if (Netfp_getIfOpt (testNetfpClientHandle, ifBridgePortHandle1, &optCfg, &errCode) < 0)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Getting MTU failed for %s [Error code %d]\n", ifBridgePort1Name, errCode);
            return -1;
        }

        /* Is this what we had configured? */
        if (mtuRetreieved != mtu)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Expected MTU %d Got MTU %d for Interface %s\n", mtu, mtuRetreieved, ifBridgePort1Name);
            return -1;
        }
    }
    if (ifBridgePortHandle2 != NULL)
    {
        /* Reset the retreived MTU */
        mtuRetreieved = 0;

        /* Get the MTU configuration for the bridged port interface from the NETFP Server */
        if (Netfp_getIfOpt (testNetfpClientHandle, ifBridgePortHandle2, &optCfg, &errCode) < 0)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Getting MTU failed for %s [Error code %d]\n", ifBridgePort2Name, errCode);
            return -1;
        }

        /* Is this what we had configured? */
        if (mtuRetreieved != mtu)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Expected MTU %d Got MTU %d for Interface %s\n", mtu, mtuRetreieved, ifBridgePort2Name);
            return -1;
        }
    }
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: MTU validated for %s %s %s\n", ifName, ifBridgePort1Name, ifBridgePort2Name);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to validate the interface status
 *
 *  @param[in]  ifHandle
 *      Interface handle
 *  @param[in]  ifStatus
 *      Interface Status
 *  @param[in]  ifBridgePort1Name
 *      Optional interface name of the bridge port1
 *  @param[in]  ifBridgePort2Name
 *      Optional interface name of the bridge port2
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t NetfpProxy_validateIfStatus
(
    const char* ifName,
    uint32_t    ifStatus,
    const char* ifBridgePort1Name,
    const char* ifBridgePort2Name
)
{
#if(defined (DEVICE_K2H) || defined (DEVICE_K2K))
    uint32_t            ifStatusRetreieved;
    Netfp_IfHandle      ifHandle;
    Netfp_OptionTLV     optCfg;
    Netfp_IfHandle      ifBridgePortHandle1 = NULL;
    Netfp_IfHandle      ifBridgePortHandle2 = NULL;
    int32_t             errCode;
    char                bridgedPortName[NETFP_MAX_CHAR];

    /* Find the interface handles for each of the names specified: */
    ifHandle = Netfp_findInterface (testNetfpClientHandle, ifName, NULL, &errCode);
    if (ifHandle == NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to find the interface %s [Error code %d]\n", ifName, errCode);
        return -1;
    }

    /* Are there any bridged port interfaces specified? */
    if (ifBridgePort1Name != NULL)
    {
        /* YES: Find the bridged port1 handle */
        snprintf (bridgedPortName, NETFP_MAX_CHAR, "%s->%s", ifName, ifBridgePort1Name);
        ifBridgePortHandle1 = Netfp_findInterface (testNetfpClientHandle, bridgedPortName, NULL, &errCode);
        if (ifBridgePortHandle1 == NULL)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to find the interface %s [Error code %d]\n", bridgedPortName, errCode);
            return -1;
        }
    }
    if (ifBridgePort2Name != NULL)
    {
        /* YES: Find the bridged port2 handle */
        snprintf (bridgedPortName, NETFP_MAX_CHAR, "%s->%s", ifName, ifBridgePort2Name);
        ifBridgePortHandle2 = Netfp_findInterface (testNetfpClientHandle, bridgedPortName, NULL, &errCode);
        if (ifBridgePortHandle2 == NULL)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to find the interface %s [Error code %d]\n", bridgedPortName, errCode);
            return -1;
        }
    }

    /**************************************************************************************************************
     * Validations: Ensure that the NETFP Server is configured correctly
     **************************************************************************************************************/
    optCfg.type     = Netfp_Option_IFACE_STATUS;
    optCfg.length   = 4;
    optCfg.value    = &ifStatusRetreieved;
    if (Netfp_getIfOpt (testNetfpClientHandle, ifHandle, &optCfg, &errCode) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Getting IfStatus failed for %s [Error code %d]\n", ifName, errCode);
        return -1;
    }
    if (ifStatusRetreieved != ifStatus)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Expected IfStatus %d Got IfStatus %d for Interface %s\n", ifStatus, ifStatusRetreieved, ifName);
        return -1;
    }

    /**************************************************************************************************************
     * Validations: If there are bridged port interfaces then the PROXY API should have configured the L3 Shaper
     * on all the bridged port interfaces.
     **************************************************************************************************************/
    if (ifBridgePortHandle1 != NULL)
    {
        /* Reset the retreived interface status */
        ifStatusRetreieved = 0;

        /* Get the MTU configuration for the bridged port interface from the NETFP Server */
        if (Netfp_getIfOpt (testNetfpClientHandle, ifBridgePortHandle1, &optCfg, &errCode) < 0)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Getting IfStatus failed for %s [Error code %d]\n", ifBridgePort1Name, errCode);
            return -1;
        }

        /* Is this what we had configured? */
        if (ifStatusRetreieved != ifStatus)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Expected IfStatus %d Got IfStatus %d for Interface %s\n",
                               ifStatus, ifStatusRetreieved, ifBridgePort1Name);
            return -1;
        }
    }
    if (ifBridgePortHandle2 != NULL)
    {
        /* Reset the retreived interface status */
        ifStatusRetreieved = 0;

        /* Get the MTU configuration for the bridged port interface from the NETFP Server */
        if (Netfp_getIfOpt (testNetfpClientHandle, ifBridgePortHandle2, &optCfg, &errCode) < 0)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Getting IfStatus failed for %s [Error code %d]\n", ifBridgePort2Name, errCode);
            return -1;
        }

        /* Is this what we had configured? */
        if (ifStatusRetreieved != ifStatus)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Expected IfStatus %d Got IfStatus %d for Interface %s\n",
                               ifStatus, ifStatusRetreieved, ifBridgePort2Name);
            return -1;
        }
    }
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: IfStatus validated for %s %s %s\n", ifName, ifBridgePort1Name, ifBridgePort2Name);
    return 0;
#else
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Warning: Interface status test bypassed on K2L\n");
    return 0;
#endif
}

/**
 *  @b Description
 *  @n
 *      The function is used to validate the egress map.
 *
 *  @param[in]  ifHandle
 *      Interface handle
 *  @param[in]  sockPriority
 *      Configured MTU
 *  @param[in]  ifBridgePort1Name
 *      Optional interface name of the bridge port1
 *  @param[in]  ifBridgePort2Name
 *      Optional interface name of the bridge port2
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t NetfpProxy_validateEgressMap
(
    const char* ifName,
    uint32_t    sockPriority,
    uint32_t    priority,
    const char* ifBridgePort1Name,
    const char* ifBridgePort2Name
)
{
    Netfp_IfHandle          ifHandle;
    Netfp_OptionTLV         optCfg;
    Netfp_IfHandle          ifBridgePortHandle1 = NULL;
    Netfp_IfHandle          ifBridgePortHandle2 = NULL;
    int32_t                 errCode;
    Netfp_VLANPriorityMap   priorityMap;
    char                    bridgedPortName[NETFP_MAX_CHAR];

    /* Find the interface handles for each of the names specified: */
    ifHandle = Netfp_findInterface (testNetfpClientHandle, ifName, NULL, &errCode);
    if (ifHandle == NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to find the interface %s [Error code %d]\n", ifName, errCode);
        return -1;
    }

    /* Are there any bridged port interfaces specified? */
    if (ifBridgePort1Name != NULL)
    {
        /* YES: Find the bridged port1 handle */
        snprintf (bridgedPortName, NETFP_MAX_CHAR, "%s->%s", ifName, ifBridgePort1Name);
        ifBridgePortHandle1 = Netfp_findInterface (testNetfpClientHandle, bridgedPortName, NULL, &errCode);
        if (ifBridgePortHandle1 == NULL)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to find the interface %s [Error code %d]\n", bridgedPortName, errCode);
            return -1;
        }
    }
    if (ifBridgePort2Name != NULL)
    {
        /* YES: Find the bridged port2 handle */
        snprintf (bridgedPortName, NETFP_MAX_CHAR, "%s->%s", ifName, ifBridgePort2Name);
        ifBridgePortHandle2 = Netfp_findInterface (testNetfpClientHandle, bridgedPortName, NULL, &errCode);
        if (ifBridgePortHandle2 == NULL)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to find the interface %s [Error code %d]\n", bridgedPortName, errCode);
            return -1;
        }
    }

    /* Initialize the priority map */
    memset ((void*)&priorityMap, 0, sizeof(Netfp_VLANPriorityMap));

    /**************************************************************************************************************
     * Validations: Ensure that the NETFP Server is configured correctly
     **************************************************************************************************************/
    optCfg.type     = Netfp_Option_VLAN_EGRESS_PRIORITY;
    optCfg.length   = sizeof(Netfp_VLANPriorityMap);
    optCfg.value    = &priorityMap;
    if (Netfp_getIfOpt (testNetfpClientHandle, ifHandle, &optCfg, &errCode) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Getting VLAN Egress Priority failed for %s [Error code %d]\n", ifName, errCode);
        return -1;
    }

    /* Sanity Check: Ensure that the VLAN Priority map is changed appropriately */
    if (priorityMap.socketToVLANPriority[sockPriority] != priority)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Expected socket priority %d mapped to %d but is %d for Interface %s\n",
                           sockPriority, priority, priorityMap.socketToVLANPriority[sockPriority], ifName);
        return -1;
    }

    /**************************************************************************************************************
     * Validations: If there are bridged port interfaces then the PROXY API should have configured the L3 Shaper
     * on all the bridged port interfaces.
     **************************************************************************************************************/
    if (ifBridgePortHandle1 != NULL)
    {
        /* Reset the retreived priority map*/
        memset ((void*)&priorityMap, 0, sizeof(Netfp_VLANPriorityMap));

        /* Get the MTU configuration for the bridged port interface from the NETFP Server */
        if (Netfp_getIfOpt (testNetfpClientHandle, ifBridgePortHandle1, &optCfg, &errCode) < 0)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Getting VLAN Egress Priority failed for %s [Error code %d]\n", ifBridgePort1Name, errCode);
            return -1;
        }

        /* Is this what we had configured? */
        if (priorityMap.socketToVLANPriority[sockPriority] != priority)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Expected socket priority %d mapped to %d but is %d for Interface %s\n",
                               sockPriority, priority, priorityMap.socketToVLANPriority[sockPriority], ifBridgePort1Name);
            return -1;
        }
    }
    if (ifBridgePortHandle2 != NULL)
    {
        /* Reset the retreived priority map*/
        memset ((void*)&priorityMap, 0, sizeof(Netfp_VLANPriorityMap));

        /* Get the MTU configuration for the bridged port interface from the NETFP Server */
        if (Netfp_getIfOpt (testNetfpClientHandle, ifBridgePortHandle2, &optCfg, &errCode) < 0)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Getting VLAN Egress Priority failed for %s [Error code %d]\n", ifBridgePort2Name, errCode);
            return -1;
        }

        /* Is this what we had configured? */
        if (priorityMap.socketToVLANPriority[sockPriority] != priority)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Expected socket priority %d mapped to %d but is %d for Interface %s\n",
                               sockPriority, priority, priorityMap.socketToVLANPriority[sockPriority], ifBridgePort2Name);
            return -1;
        }
    }
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: VLAN Egress Map validated for %s %s %s\n", ifName, ifBridgePort1Name, ifBridgePort2Name);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to test the basic interface creation and
 *      deletion.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t NetfpProxy_testBasicInterface (void)
{
    int32_t             errCode;
    Netfp_InterfaceCfg  ifCfg;
    Netfp_IfHandle      ifHandle;

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "************************************************\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Basic Interface Test\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "************************************************\n");

    /****************************************************************************************
     * Test Setup:
     ****************************************************************************************/
    system ("ifconfig eth0 192.168.1.2 mtu 1500");
    system ("ip addr add dev eth0 7000::2/64");
    system ("route add default gw 192.168.1.1");

    /* Create the interface in NETFP Proxy: */
    if (NetfpProxy_testCreateInterface ("eth0") < 0)
        return -1;

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Interface eth0 has been created in NETFP\n");

    /* Sanity Check: Interface should have been created & offloaded to the NETFP Server also. */
    ifHandle = Netfp_findInterface (testNetfpClientHandle, "eth0", &ifCfg, &errCode);
    if (ifHandle == NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Failed to find interface eth0 [Error code %d]\n", errCode);
        return -1;
    }

    /* Ensure that the interface type is valid: */
    if (ifCfg.type != Netfp_InterfaceType_ETH)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Invalid Interface type [%d] detected\n", ifCfg.type);
        return -1;
    }

    /****************************************************************************************
     * MTU:
     ****************************************************************************************/
    system ("ifconfig eth0 mtu 1300");
    sleep(5);

    /* Validate the MTU: */
    if (NetfpProxy_validateMTU ("eth0", 1300, NULL, NULL) < 0)
        return -1;

    /****************************************************************************************
     * L3 Shaper:
     ****************************************************************************************/
    if (NetfpProxy_validateL3Shaper ("eth0", 1000, NULL, NULL) < 0)
        return -1;

#if(defined (DEVICE_K2H) || defined (DEVICE_K2K))
    /****************************************************************************************
     * Link Status:
     ****************************************************************************************/
    system ("ifconfig eth0 mtu 1500 down");
    sleep(5);
#endif
    if (NetfpProxy_validateIfStatus ("eth0", 0, NULL, NULL) < 0)
        return -1;

    /****************************************************************************************
     * Delete the interface:
     ****************************************************************************************/
    if (NetfpProxy_testDeleteInterface ("eth0") < 0)
        return -1;

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Interface eth0 has been deleted in NETFP\n");

    /* Sanity Check: Interface should have been deleted from the NETFP Server also. */
    ifHandle = Netfp_findInterface (testNetfpClientHandle, "eth0", &ifCfg, &errCode);
    if (ifHandle != NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Interface eth0 still exists in the NETFP Server\n");
        return -1;
    }

    /****************************************************************************************
     * Cleanup:
     ****************************************************************************************/
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Resetting the Linux networking configuration\n");
    system ("ifconfig eth0 0.0.0.0");
    system ("ip addr del dev eth0 7000::2/64");
    system ("route del default gw 192.168.1.1");

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "************************************************\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Basic Interface Testing Passed\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "************************************************\n");
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to test the bridge interface creation and
 *      deletion.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t NetfpProxy_testBridgeInterface (void)
{
    int32_t             errCode;
    Netfp_InterfaceCfg  ifCfg;
    Netfp_IfHandle      ifBridgeHandle;
    Netfp_IfHandle      ifBridgePortHandle1;
    Netfp_IfHandle      ifBridgePortHandle2;

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "************************************************\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Bridge Interface Test\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "************************************************\n");

    /****************************************************************************************
     * Test Setup:
     ****************************************************************************************/
    system ("ifconfig eth0 0.0.0.0 mtu 1500 up");
    system ("ifconfig eth1 0.0.0.0 mtu 1500 up");
    system ("brctl addbr br0");
    system ("brctl addif br0 eth0");
    system ("brctl addif br0 eth1");
    system ("ifconfig br0 192.168.1.2");

    /* Create the interface in NETFP Proxy: */
    if (NetfpProxy_testCreateInterface ("br0") < 0)
        return -1;

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Interface br0 has been created in NETFP\n");

    /* Sanity Check: Interface should have been created & offloaded to the NETFP Server also. */
    ifBridgeHandle = Netfp_findInterface (testNetfpClientHandle, "br0", &ifCfg, &errCode);
    if (ifBridgeHandle == NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Failed to find interface br0 [Error code %d]\n", errCode);
        return -1;
    }

    /* Ensure that the interface type is valid: */
    if (ifCfg.type != Netfp_InterfaceType_ETH)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Invalid Interface type [%d] detected for br0\n", ifCfg.type);
        return -1;
    }

    /* Sanity Check: Bridged Port interfaces should also have been offloaded to the NETFP Server */
    ifBridgePortHandle1 = Netfp_findInterface (testNetfpClientHandle, "br0->eth0", &ifCfg, &errCode);
    if (ifBridgePortHandle1 == NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Failed to find interface eth0 [Error code %d]\n", errCode);
        return -1;
    }

    /* Ensure that the interface type is valid: */
    if (ifCfg.type != Netfp_InterfaceType_ETH)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Invalid Interface type [%d] detected for eth0\n", ifCfg.type);
        return -1;
    }

    /* Sanity Check: Bridged Port interfaces should also have been offloaded to the NETFP Server */
    ifBridgePortHandle2 = Netfp_findInterface (testNetfpClientHandle, "br0->eth1", &ifCfg, &errCode);
    if (ifBridgePortHandle2 == NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Failed to find interface %s [Error code %d]\n", "eth1", errCode);
        return -1;
    }

    /* Ensure that the interface type is valid: */
    if (ifCfg.type != Netfp_InterfaceType_ETH)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Invalid Interface type [%d] detected for eth1\n", ifCfg.type);
        return -1;
    }

    /****************************************************************************************
     * MTU:
     ****************************************************************************************/
    system ("ifconfig br0 mtu 1300");
    sleep(5);

    /* Validate the MTU: */
    if (NetfpProxy_validateMTU ("br0", 1300, "eth0", "eth1") < 0)
        return -1;

    /****************************************************************************************
     * L3 Shaper:
     ****************************************************************************************/
    if (NetfpProxy_validateL3Shaper ("br0", 4000, "eth0", "eth1") < 0)
        return -1;

#if(defined (DEVICE_K2H) || defined (DEVICE_K2K))
    /****************************************************************************************
     * Link Status:
     ****************************************************************************************/
    system ("ifconfig br0 down");
    sleep(5);
#endif
    if (NetfpProxy_validateIfStatus ("br0", 0, "eth0", "eth1") < 0)
        return -1;

    /****************************************************************************************
     * Delete the Bridged Interface:
     ****************************************************************************************/
    if (NetfpProxy_testDeleteInterface ("br0") < 0)
        return -1;

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Interface br0 has been deleted in NETFP\n");

    /* Sanity Check: Interface should have been deleted from the NETFP Server also. */
    if (Netfp_findInterface (testNetfpClientHandle, "br0", &ifCfg, &errCode) != NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Interface br0 still exists in the NETFP Server\n");
        return -1;
    }

    /****************************************************************************************
     * Bridged Port Deletion: Automatically done once the bridged interface is deleted
     ****************************************************************************************/
    if (Netfp_findInterface (testNetfpClientHandle, "br0->eth0", &ifCfg, &errCode) != NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Interface eth0 still exists in the NETFP Server\n");
        return -1;
    }
    if (Netfp_findInterface (testNetfpClientHandle, "br0->eth1", &ifCfg, &errCode) != NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Interface eth1 still exists in the NETFP Server\n");
        return -1;
    }

    /****************************************************************************************
     * Cleanup:
     ****************************************************************************************/
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Resetting the Linux networking configuration\n");
    system ("brctl delif br0 eth0");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: eth0 has been deleted from the bridge\n");
    system ("brctl delif br0 eth1");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: eth1 has been deleted from the bridge\n");
    system ("ifconfig br0 down");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Bridged interface has been bought down\n");
    system ("brctl delbr br0");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Deleted br0 \n");

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "************************************************\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Bridge Interface Testing Passed\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "************************************************\n");
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to test the VLAN interface creation/deletion and the flushing
 *      of the VLAN egress maps
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t NetfpProxy_testVLANInterface (void)
{
    int32_t                 errCode;
    Netfp_InterfaceCfg      ifCfg;
    Netfp_IfHandle          ifHandle;

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "************************************************\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: VLAN Interface Test\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "************************************************\n");

    /****************************************************************************************
     * Test Setup:
     ****************************************************************************************/
    system ("ifconfig eth0 0.0.0.0 mtu 1500 up");
    system ("vconfig add eth0 2");
    system ("vconfig add eth0 3");
    system ("ifconfig eth0.2 192.168.1.2");
    system ("ifconfig eth0.3 192.168.2.2");
    system ("ip addr add dev eth0.2 7000::2/64");

    /****************************************************************************************
     * Create the interface eth0.2 in the NETFP Proxy
     ****************************************************************************************/

    /* Create the interface in NETFP Proxy: */
    if (NetfpProxy_testCreateInterface ("eth0.2") < 0)
        return -1;

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Interface eth0.2 has been created in NETFP\n");

    /* Sanity Check: Interface should have been created & offloaded to the NETFP Server also. */
    ifHandle = Netfp_findInterface (testNetfpClientHandle, "eth0.2", &ifCfg, &errCode);
    if (ifHandle == NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Failed to find interface eth0.2 [Error code %d]\n", errCode);
        return -1;
    }

    /* Ensure that the interface type is valid: */
    if (ifCfg.type != Netfp_InterfaceType_VLAN)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Invalid Interface type [%d] detected\n", ifCfg.type);
        return -1;
    }

    /****************************************************************************************
     * Create the interface eth0.3 in the NETFP Proxy
     ****************************************************************************************/

    /* Create the interface in NETFP Proxy: */
    if (NetfpProxy_testCreateInterface ("eth0.3") < 0)
        return -1;

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Interface eth0.3 has been created in NETFP\n");

    /* Sanity Check: Interface should have been created & offloaded to the NETFP Server also. */
    ifHandle = Netfp_findInterface (testNetfpClientHandle, "eth0.3", &ifCfg, &errCode);
    if (ifHandle == NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Failed to find interface eth0.3 [Error code %d]\n", errCode);
        return -1;
    }

    /* Ensure that the interface type is valid: */
    if (ifCfg.type != Netfp_InterfaceType_VLAN)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Invalid Interface type [%d] detected\n", ifCfg.type);
        return -1;
    }

    /****************************************************************************************
     * MTU:
     ****************************************************************************************/
    system ("ifconfig eth0.2 mtu 1300");
    system ("ifconfig eth0.3 mtu 1400");
    sleep(5);

    /* Validate the MTU on each interface */
    if (NetfpProxy_validateMTU ("eth0.2", 1300, NULL, NULL) < 0)
        return -1;

    /* Validate the MTU on each interface */
    if (NetfpProxy_validateMTU ("eth0.3", 1400, NULL, NULL) < 0)
        return -1;

    /****************************************************************************************
     * L3 Shaper:
     ****************************************************************************************/
    if (NetfpProxy_validateL3Shaper ("eth0.2", 3000, NULL, NULL) < 0)
        return -1;
    if (NetfpProxy_validateL3Shaper ("eth0.3", 3500, NULL, NULL) < 0)
        return -1;

    /****************************************************************************************
     * Link Status: Modify the Egress Map
     ****************************************************************************************/
    system ("vconfig set_egress_map eth0.2 43 4");
    system ("vconfig set_egress_map eth0.3 15 7");
    sleep(5);

    /* Flush the egress map: */
    if (NetfpProxy_testFlushVLANEgressMap("eth0.2") < 0)
        return -1;

    /* Flush the egress map: */
    if (NetfpProxy_testFlushVLANEgressMap("eth0.3") < 0)
        return -1;

    /* Validate the Egress Map: */
    if (NetfpProxy_validateEgressMap ("eth0.2", 43, 4, NULL, NULL) < 0)
        return -1;

    /* Validate the Egress Map: */
    if (NetfpProxy_validateEgressMap ("eth0.3", 15, 7, NULL, NULL) < 0)
        return -1;

#if(defined (DEVICE_K2H) || defined (DEVICE_K2K))
    /****************************************************************************************
     * Link Status:
     ****************************************************************************************/
    system ("ifconfig eth0.2 down");
    sleep(5);
#endif
    if (NetfpProxy_validateIfStatus ("eth0.2", 0, NULL, NULL) < 0)
        return -1;

    /****************************************************************************************
     * Delete the interface:
     ****************************************************************************************/
    if (NetfpProxy_testDeleteInterface ("eth0.2") < 0)
        return -1;

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Interface eth0.2 has been deleted in NETFP\n");

    if (NetfpProxy_testDeleteInterface ("eth0.3") < 0)
        return -1;

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Interface eth0.3 has been deleted in NETFP\n");

    /* Sanity Check: Interface should have been deleted from the NETFP Server also. */
    ifHandle = Netfp_findInterface (testNetfpClientHandle, "eth0.2", &ifCfg, &errCode);
    if (ifHandle != NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Interface eth0.2 still exists in the NETFP Server\n");
        return -1;
    }
    /* Sanity Check: Interface should have been deleted from the NETFP Server also. */
    ifHandle = Netfp_findInterface (testNetfpClientHandle, "eth0.3", &ifCfg, &errCode);
    if (ifHandle != NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Interface eth0.3 still exists in the NETFP Server\n");
        return -1;
    }

    /****************************************************************************************
     * Cleanup:
     ****************************************************************************************/
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Resetting the Linux networking configuration\n");
    system ("ifconfig eth0.2 0.0.0.0 mtu 1500");
    system ("ifconfig eth0.3 0.0.0.0 mtu 1500");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: IPv4 address removed from the VLAN interface\n");
    system ("ip addr del dev eth0.2 7000::2/64");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: IPv6 address removed from the VLAN interface\n");
    system ("vconfig rem eth0.2");
    system ("vconfig rem eth0.3");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: VLAN interface has been removed\n");

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "************************************************\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: VLAN Interface Testing Passed\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "************************************************\n");
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to test the bridge which has VLAN interfaces below it.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t NetfpProxy_testBridgeWithVLANInterface (void)
{
    int32_t                 errCode;
    Netfp_InterfaceCfg      ifCfg;
    Netfp_IfHandle          ifBridgeHandle;
    Netfp_IfHandle          ifBridgePortHandle1;
    Netfp_IfHandle          ifBridgePortHandle2;

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "************************************************\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Bridge With VLAN Interfaces Test\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "************************************************\n");

    /****************************************************************************************
     * Test Setup:
     ****************************************************************************************/
    system ("ifconfig eth0 0.0.0.0 mtu 1500 up");
    system ("ifconfig eth1 0.0.0.0 mtu 1500 up");
    system ("vconfig add eth0 2");
    system ("vconfig add eth1 2");
    system ("ifconfig eth0.2 up");
    system ("ifconfig eth1.2 up");
    system ("brctl addbr br0");
    system ("brctl addif br0 eth0.2");
    system ("brctl addif br0 eth1.2");
    system ("ifconfig br0 192.168.1.2");

    /* Create the interface in NETFP Proxy: */
    if (NetfpProxy_testCreateInterface ("br0") < 0)
        return -1;

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Interface br0 has been created in NETFP\n");

    /* Sanity Check: Interface should have been created & offloaded to the NETFP Server also. */
    ifBridgeHandle = Netfp_findInterface (testNetfpClientHandle, "br0", &ifCfg, &errCode);
    if (ifBridgeHandle == NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Failed to find interface br0 [Error code %d]\n", errCode);
        return -1;
    }

    /* Ensure that the interface type is valid: */
    if (ifCfg.type != Netfp_InterfaceType_VLAN)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Invalid Interface type [%d] detected for br0\n", ifCfg.type);
        return -1;
    }

    /* Sanity Check: Bridged Port interfaces should also have been offloaded to the NETFP Server */
    ifBridgePortHandle1 = Netfp_findInterface (testNetfpClientHandle, "br0->eth0.2", &ifCfg, &errCode);
    if (ifBridgePortHandle1 == NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Failed to find interface eth0.2 [Error code %d]\n", errCode);
        return -1;
    }

    /* Ensure that the interface type is valid: */
    if (ifCfg.type != Netfp_InterfaceType_VLAN)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Invalid Interface type [%d] detected for eth0\n", ifCfg.type);
        return -1;
    }

    /* Sanity Check: Bridged Port interfaces should also have been offloaded to the NETFP Server */
    ifBridgePortHandle2 = Netfp_findInterface (testNetfpClientHandle, "br0->eth1.2", &ifCfg, &errCode);
    if (ifBridgePortHandle2 == NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Failed to find interface eth1.2 [Error code %d]\n", errCode);
        return -1;
    }

    /* Ensure that the interface type is valid: */
    if (ifCfg.type != Netfp_InterfaceType_VLAN)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Invalid Interface type [%d] detected for eth1.2\n", ifCfg.type);
        return -1;
    }

    /****************************************************************************************
     * MTU:
     ****************************************************************************************/
    system ("ifconfig br0 mtu 1300");
    sleep(5);

    /* Validate the MTU: */
    if (NetfpProxy_validateMTU ("br0", 1300, "eth0.2", "eth1.2") < 0)
        return -1;

    /****************************************************************************************
     * L3 Shaper:
     ****************************************************************************************/
    if (NetfpProxy_validateL3Shaper ("br0", 4000, "eth0.2", "eth1.2") < 0)
        return -1;

    /****************************************************************************************
     * Link Status: Modify the Egress Map
     ****************************************************************************************/
    system ("vconfig set_egress_map eth0.2 43 4");
    system ("vconfig set_egress_map eth1.2 43 4");
    sleep(5);

    /* Flush the egress map [This is done for the VLAN Interfaces] */
    if (NetfpProxy_testFlushVLANEgressMap("eth0.2") < 0)
        return -1;
    if (NetfpProxy_testFlushVLANEgressMap("eth1.2") < 0)
        return -1;

    /* Validate the Egress Map: */
    if (NetfpProxy_validateEgressMap ("br0", 43, 4, "eth0.2", "eth1.2") < 0)
        return -1;

#if(defined (DEVICE_K2H) || defined (DEVICE_K2K))
    /****************************************************************************************
     * Link Status:
     ****************************************************************************************/
    system ("ifconfig br0 down");
    sleep(5);
#endif
    if (NetfpProxy_validateIfStatus ("br0", 0, "eth0.2", "eth1.2") < 0)
        return -1;

    /****************************************************************************************
     * Delete the Bridged Interface:
     ****************************************************************************************/
    if (NetfpProxy_testDeleteInterface ("br0") < 0)
        return -1;

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Interface br0 has been deleted in NETFP\n");

    /* Sanity Check: Interface should have been deleted from the NETFP Server also. */
    if (Netfp_findInterface (testNetfpClientHandle, "br0", &ifCfg, &errCode) != NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Interface br0 still exists in the NETFP Server\n");
        return -1;
    }

    /****************************************************************************************
     * Bridged Port Deletion: Automatically done once the bridged interface is deleted
     ****************************************************************************************/
    if (Netfp_findInterface (testNetfpClientHandle, "br0->eth0.2", &ifCfg, &errCode) != NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Interface eth0.2 still exists in the NETFP Server\n");
        return -1;
    }
    if (Netfp_findInterface (testNetfpClientHandle, "br0->eth1.2", &ifCfg, &errCode) != NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Interface eth1.2 still exists in the NETFP Server\n");
        return -1;
    }

    /****************************************************************************************
     * Cleanup:
     ****************************************************************************************/
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Resetting the Linux networking configuration\n");
    system ("brctl delif br0 eth0.2");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: eth0.2 has been removed from the bridge\n");
    system ("brctl delif br0 eth1.2");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: eth1.2 has been removed from the bridge\n");
    system ("vconfig rem eth0.2");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: eth0.2 has been removed from the system\n");
    system ("vconfig rem eth1.2");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: eth1.2 has been removed from the system\n");
    system ("ifconfig br0 down");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: br0 has been bought down\n");
    system ("brctl delbr br0");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Bridge br0 has been deleted\n");

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "************************************************\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Bridge With VLAN Interfaces Testing Passed\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "************************************************\n");
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to test the bridge which is configured with a VLAN identfier
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t NetfpProxy_testVLANBridge (void)
{
    int32_t                 errCode;
    Netfp_InterfaceCfg      ifCfg;
    Netfp_IfHandle          ifBridgeHandle;
    Netfp_IfHandle          ifBridgePortHandle1;
    Netfp_IfHandle          ifBridgePortHandle2;

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "************************************************\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: VLAN Bridge Test\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "************************************************\n");

    /****************************************************************************************
     * Test Setup:
     ****************************************************************************************/
    system ("ifconfig eth0 0.0.0.0 mtu 1500 up");
    system ("ifconfig eth1 0.0.0.0 mtu 1500 up");
    system ("brctl addbr br0");
    system ("brctl addif br0 eth0");
    system ("brctl addif br0 eth1");
    system ("vconfig add br0 2");
    system ("vconfig add br0 3");
    system ("ifconfig br0 up");
    system ("ifconfig br0.2 192.168.1.2");
    system ("ifconfig br0.3 192.168.2.2");

    /****************************************************************************************
     * Offload Interface br0.2
     *  NETFP Proxy will offload the following interfaces br0.2->eth0 and br0.2->eth1
     ****************************************************************************************/

    /* Create the interface in NETFP Proxy: */
    if (NetfpProxy_testCreateInterface ("br0.2") < 0)
        return -1;

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Interface br0.2 has been created in NETFP\n");

    /* Sanity Check: Interface should have been created & offloaded to the NETFP Server also. */
    ifBridgeHandle = Netfp_findInterface (testNetfpClientHandle, "br0.2", &ifCfg, &errCode);
    if (ifBridgeHandle == NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Failed to find interface br0.2 [Error code %d]\n", errCode);
        return -1;
    }

    /* Ensure that the interface type is valid: */
    if (ifCfg.type != Netfp_InterfaceType_VLAN)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Invalid Interface type [%d] detected for br0.2\n", ifCfg.type);
        return -1;
    }

    /* Sanity Check: Bridged Port interfaces should also have been offloaded to the NETFP Server */
    ifBridgePortHandle1 = Netfp_findInterface (testNetfpClientHandle, "br0.2->eth0", &ifCfg, &errCode);
    if (ifBridgePortHandle1 == NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Failed to find interface br0.2->eth0 [Error code %d]\n", errCode);
        return -1;
    }

    /* Ensure that the interface type is valid: */
    if (ifCfg.type != Netfp_InterfaceType_VLAN)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Invalid Interface type [%d] detected for br0.2->eth0\n", ifCfg.type);
        return -1;
    }

    /* Sanity Check: Bridged Port interfaces should also have been offloaded to the NETFP Server */
    ifBridgePortHandle2 = Netfp_findInterface (testNetfpClientHandle, "br0.2->eth1", &ifCfg, &errCode);
    if (ifBridgePortHandle2 == NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Failed to find interface br0.2->eth1 [Error code %d]\n", errCode);
        return -1;
    }

    /* Ensure that the interface type is valid: */
    if (ifCfg.type != Netfp_InterfaceType_VLAN)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Invalid Interface type [%d] detected for br0.2->eth1\n", ifCfg.type);
        return -1;
    }

    /****************************************************************************************
     * Offload Interface br0.3
     *  NETFP Proxy will offload the following interfaces br0.3->eth0 and br0.3->eth1
     ****************************************************************************************/

    /* Create the interface in NETFP Proxy: */
    if (NetfpProxy_testCreateInterface ("br0.3") < 0)
        return -1;

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Interface br0.3 has been created in NETFP\n");

    /* Sanity Check: Interface should have been created & offloaded to the NETFP Server also. */
    ifBridgeHandle = Netfp_findInterface (testNetfpClientHandle, "br0.3", &ifCfg, &errCode);
    if (ifBridgeHandle == NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Failed to find interface br0.3 [Error code %d]\n", errCode);
        return -1;
    }

    /* Ensure that the interface type is valid: */
    if (ifCfg.type != Netfp_InterfaceType_VLAN)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Invalid Interface type [%d] detected for br0.3\n", ifCfg.type);
        return -1;
    }

    /* Sanity Check: Bridged Port interfaces should also have been offloaded to the NETFP Server */
    ifBridgePortHandle1 = Netfp_findInterface (testNetfpClientHandle, "br0.3->eth0", &ifCfg, &errCode);
    if (ifBridgePortHandle1 == NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Failed to find interface br0.3->eth0 [Error code %d]\n", errCode);
        return -1;
    }

    /* Ensure that the interface type is valid: */
    if (ifCfg.type != Netfp_InterfaceType_VLAN)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Invalid Interface type [%d] detected for br0.3->eth0\n", ifCfg.type);
        return -1;
    }

    /* Sanity Check: Bridged Port interfaces should also have been offloaded to the NETFP Server */
    ifBridgePortHandle2 = Netfp_findInterface (testNetfpClientHandle, "br0.3->eth1", &ifCfg, &errCode);
    if (ifBridgePortHandle2 == NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Failed to find interface br0.3->eth1 [Error code %d]\n", errCode);
        return -1;
    }

    /* Ensure that the interface type is valid: */
    if (ifCfg.type != Netfp_InterfaceType_VLAN)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Invalid Interface type [%d] detected for br0.3->eth1\n", ifCfg.type);
        return -1;
    }

    /****************************************************************************************
     * MTU: Configure the MTU for the Bridge Interface 'br0.2'
     ****************************************************************************************/
    system ("ifconfig br0.2 mtu 1300");
    sleep(5);

    /* Validate the MTU: */
    if (NetfpProxy_validateMTU ("br0.2", 1300, "eth0", "eth1") < 0)
        return -1;

    /****************************************************************************************
     * MTU: Configure the MTU for the Bridge Interface 'br0.3'
     ****************************************************************************************/
    system ("ifconfig br0.3 mtu 1400");
    sleep(5);

    /* Validate the MTU: */
    if (NetfpProxy_validateMTU ("br0.3", 1400, "eth0", "eth1") < 0)
        return -1;

    /****************************************************************************************
     * L3 Shaper: Configure the L3 Shaper for the Bridge Interface 'br0.2'
     ****************************************************************************************/
    if (NetfpProxy_validateL3Shaper ("br0.2", 1001, "eth0", "eth1") < 0)
        return -1;

    /****************************************************************************************
     * L3 Shaper: Configure the L3 Shaper for the Bridge Interface 'br0.3'
     ****************************************************************************************/
    if (NetfpProxy_validateL3Shaper ("br0.3", 2001, "eth0", "eth1") < 0)
        return -1;

    /****************************************************************************************
     * Egress Map: Modify the Egress Map for the Bridge Interface 'br0.2'
     ****************************************************************************************/
    system ("vconfig set_egress_map br0.2 43 4");
    sleep(5);

    /* Flush the egress map [This is done for the VLAN Interfaces] */
    if (NetfpProxy_testFlushVLANEgressMap("br0.2") < 0)
        return -1;

    /* Validate the Egress Map: */
    if (NetfpProxy_validateEgressMap ("br0.2", 43, 4, "eth0", "eth1") < 0)
        return -1;

    /****************************************************************************************
     * Egress Map: Modify the Egress Map for the Bridge Interface 'br0.2'
     ****************************************************************************************/
    system ("vconfig set_egress_map br0.3 15 7");
    sleep(5);

    /* Flush the egress map [This is done for the VLAN Interfaces] */
    if (NetfpProxy_testFlushVLANEgressMap("br0.3") < 0)
        return -1;

    /* Validate the Egress Map: */
    if (NetfpProxy_validateEgressMap ("br0.3", 15, 7, "eth0", "eth1") < 0)
        return -1;

#if(defined (DEVICE_K2H) || defined (DEVICE_K2K))
    /****************************************************************************************
     * Link Status:
     ****************************************************************************************/
    system ("ifconfig br0.2 down");
    sleep(5);
#endif
    if (NetfpProxy_validateIfStatus ("br0.2", 0, "eth0", "eth1") < 0)
        return -1;

    /****************************************************************************************
     * Delete the Bridged Interface:
     ****************************************************************************************/
    if (NetfpProxy_testDeleteInterface ("br0.2") < 0)
        return -1;

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Interface br0.2 has been deleted in NETFP\n");

    /* Sanity Check: Interface should have been deleted from the NETFP Server also. */
    if (Netfp_findInterface (testNetfpClientHandle, "br0.2", &ifCfg, &errCode) != NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Interface br0.2 still exists in the NETFP Server\n");
        return -1;
    }

    /****************************************************************************************
     * Bridged Port Deletion: Automatically done once the bridged interface is deleted
     ****************************************************************************************/
    if (Netfp_findInterface (testNetfpClientHandle, "br0.2->eth0", &ifCfg, &errCode) != NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Interface eth0 still exists in the NETFP Server\n");
        return -1;
    }
    if (Netfp_findInterface (testNetfpClientHandle, "br0.2->eth1", &ifCfg, &errCode) != NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Interface eth1 still exists in the NETFP Server\n");
        return -1;
    }

    /****************************************************************************************
     * Cleanup:
     ****************************************************************************************/
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Resetting the Linux networking configuration\n");
    system ("brctl delif br0 eth0");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: eth0 has been deleted from the bridge\n");
    system ("brctl delif br0 eth1");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: eth1 has been deleted from the bridge\n");
    system ("vconfig rem br0.2");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: br0.2 has been removed from the system\n");
    system ("ifconfig br0 down");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: br0 has been bought down\n");
    system ("brctl delbr br0");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: br0 has been deleted from the system\n");

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "************************************************\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: VLAN Bridge Testing Passed\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "************************************************\n");
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function tests the following scenario:
 *         (a) Create single outbound IPv4 fast paths
 *         (b) Ensures that there is 1 routing entry and 1 neighbor entry.
 *         (c) Deletes 1 outbound fast path
 *
 *  Expectation:
 *      - Ensures that there are 0 routing and neighbor entries
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t NetfpProxy_testIPv4Basic(void)
{
    int32_t                 errCode;
    Netfp_InterfaceCfg      ifCfg;
    Netfp_IfHandle          ifHandle;
    int32_t                 numRouteEntries;
    int32_t                 numNeighEntries;
    Netfp_OutboundFPCfg     outboundFPCfg;
    Netfp_OutboundFPHandle  outboundFPHandle;

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "************************************************\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Basic IPv4 Route Resolution\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "************************************************\n");

    /* Sanity Check: Get the number of entries in the routing table */
    numRouteEntries = NetfpProxy_routeGetNumEntries ();
    if (numRouteEntries != 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Detected %d entries in the route cache before test\n", numRouteEntries);
        return -1;
    }
    numNeighEntries = NetfpProxy_neighGetNumEntries();
    if (numNeighEntries != 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Detected %d entries in the neighbor cache before test\n", numNeighEntries);
        return -1;
    }

    /*****************************************************************************************
     * Create an OUTBOUND Fast Path:
     *****************************************************************************************/
    memset ((void *)&outboundFPCfg, 0, sizeof (Netfp_OutboundFPCfg));

    /* Populate the Fast Path configuration. */
    outboundFPCfg.spId                    = NETFP_INVALID_SPID;
    outboundFPCfg.dstIP.ver               = Netfp_IPVersion_IPV4;
    outboundFPCfg.dstIP.addr.ipv4.u.a8[0] = 192;
    outboundFPCfg.dstIP.addr.ipv4.u.a8[1] = 168;
    outboundFPCfg.dstIP.addr.ipv4.u.a8[2] = 1;
    outboundFPCfg.dstIP.addr.ipv4.u.a8[3] = 1;
    outboundFPCfg.srcIP.ver               = Netfp_IPVersion_IPV4;
    outboundFPCfg.srcIP.addr.ipv4.u.a8[0] = 192;
    outboundFPCfg.srcIP.addr.ipv4.u.a8[1] = 168;
    outboundFPCfg.srcIP.addr.ipv4.u.a8[2] = 1;
    outboundFPCfg.srcIP.addr.ipv4.u.a8[3] = 2;
    strcpy (outboundFPCfg.name, "TestBasicIPv4FastPath");

    /* Create the outbound fast path */
    outboundFPHandle = Netfp_createOutboundFastPath  (testNetfpClientHandle, &outboundFPCfg, &errCode);
    if (outboundFPHandle == NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to create the OUTBOUND fast path [Error %d]\n", errCode);
        return -1;
    }

    /*****************************************************************************************
     * Wait for some time in order to get the routes resolved and the interfaces offloaded
     *****************************************************************************************/
    sleep(5);

    /* Sanity Check: Ensure that the interface has been offloaded. */
    ifHandle = Netfp_findInterface (testNetfpClientHandle, "eth0", &ifCfg, &errCode);
    if (ifHandle == NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Failed to find interface eth0 [Error code %d]\n", errCode);
        return -1;
    }

    /* Ensure that the interface type is valid: */
    if (ifCfg.type != Netfp_InterfaceType_ETH)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Invalid Interface type [%d] detected for eth0\n", ifCfg.type);
        return -1;
    }

    /*****************************************************************************************
     * Get the number of entries in the routing & neighbor table.
     * - There should be 1 entry in the routing cache
     * - There should only be 1 neighbor entry
     *****************************************************************************************/
    numRouteEntries = NetfpProxy_routeGetNumEntries ();
    if (numRouteEntries != 1)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Detected %d entries in the route cache after the test\n", numRouteEntries);
        return -1;
    }
    numNeighEntries = NetfpProxy_neighGetNumEntries();
    if (numNeighEntries != 1)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Detected %d entries in the neighbor cache after the test\n", numNeighEntries);
        return -1;
    }
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Detected %d entries in the Route cache\n", numRouteEntries);
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Detected %d entries in the Neighbor cache\n", numNeighEntries);

    /***********************************************************************************************
     * Delete the outbound fast path
     ***********************************************************************************************/
    if (Netfp_deleteOutboundFastPath (testNetfpClientHandle, outboundFPHandle, &errCode) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to delete the fast path [Error code %d]\n", errCode);
        return -1;
    }

    /* Sleep allow the neighbor & routes to be deleted */
    sleep(1);

    /*****************************************************************************************
     * Get the number of entries in the routing & neighbor table.
     * - There should be no more entries in either table
     *****************************************************************************************/
    numRouteEntries = NetfpProxy_routeGetNumEntries ();
    if (numRouteEntries != 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Detected %d entries in the route cache after the test\n", numRouteEntries);
        return -1;
    }
    numNeighEntries = NetfpProxy_neighGetNumEntries();
    if (numNeighEntries != 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Detected %d entries in the neighbor cache after the test\n", numNeighEntries);
        return -1;
    }
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Detected %d entries in the Route cache\n", numRouteEntries);
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Detected %d entries in the Neighbor cache\n", numNeighEntries);

    /****************************************************************************************
     * Delete the Interface:
     ****************************************************************************************/
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Cleaning up\n");
    if (NetfpProxy_testDeleteInterface ("eth0") < 0)
        return -1;

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Interface eth0 has been deleted in NETFP\n");

    /* Sanity Check: Get the number of route & neighbor entries. */
    numRouteEntries = NetfpProxy_routeGetNumEntries ();
    if (numRouteEntries != 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Detected %d entries in the route cache after interface is deleted\n", numRouteEntries);
        return -1;
    }
    numNeighEntries = NetfpProxy_neighGetNumEntries();
    if (numNeighEntries != 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Detected %d entries in the neighbor cache after interface is deleted\n", numNeighEntries);
        return -1;
    }

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "************************************************\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Basic IPv4 Route Resolution Passed\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "************************************************\n");
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function tests the following scenario:
 *         (a) Create single outbound IPv6 fast paths
 *         (b) Ensures that there is 1 routing entry and 1 neighbor entry.
 *         (c) Deletes 1 outbound fast path
 *
 *  Expectation:
 *      - Ensures that there are 0 routing and neighbor entries
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t NetfpProxy_testIPv6Basic(void)
{
    int32_t                 errCode;
    Netfp_InterfaceCfg      ifCfg;
    Netfp_IfHandle          ifHandle;
    int32_t                 numRouteEntries;
    int32_t                 numNeighEntries;
    Netfp_OutboundFPCfg     outboundFPCfg;
    Netfp_OutboundFPHandle  outboundFPHandle;

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "************************************************\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Basic IPv6 Route Resolution\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "************************************************\n");

    /* Sanity Check: Get the number of entries in the routing table */
    numRouteEntries = NetfpProxy_routeGetNumEntries ();
    if (numRouteEntries != 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Detected %d entries in the route cache before test\n", numRouteEntries);
        return -1;
    }
    numNeighEntries = NetfpProxy_neighGetNumEntries();
    if (numNeighEntries != 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Detected %d entries in the neighbor cache before test\n", numNeighEntries);
        return -1;
    }

    /*****************************************************************************************
     * Create an OUTBOUND Fast Path:
     *****************************************************************************************/
    memset ((void *)&outboundFPCfg, 0, sizeof (Netfp_OutboundFPCfg));

    /* Populate the Fast Path configuration. */
    outboundFPCfg.spId                     = NETFP_INVALID_SPID;
    outboundFPCfg.dstIP.ver                = Netfp_IPVersion_IPV6;
    outboundFPCfg.dstIP.addr.ipv6.u.a32[0] = Netfp_htonl(0x70000000);
    outboundFPCfg.dstIP.addr.ipv6.u.a32[1] = Netfp_htonl(0);
    outboundFPCfg.dstIP.addr.ipv6.u.a32[2] = Netfp_htonl(0);
    outboundFPCfg.dstIP.addr.ipv6.u.a32[3] = Netfp_htonl(0x1);
    outboundFPCfg.srcIP.ver                = Netfp_IPVersion_IPV6;
    outboundFPCfg.srcIP.addr.ipv6.u.a32[0] = Netfp_htonl(0x70000000);
    outboundFPCfg.srcIP.addr.ipv6.u.a32[1] = Netfp_htonl(0);
    outboundFPCfg.srcIP.addr.ipv6.u.a32[2] = Netfp_htonl(0);
    outboundFPCfg.srcIP.addr.ipv6.u.a32[3] = Netfp_htonl(0x2);
    strcpy (outboundFPCfg.name, "TestBasicIPv6FastPath");

    /* Create the outbound fast path */
    outboundFPHandle = Netfp_createOutboundFastPath  (testNetfpClientHandle, &outboundFPCfg, &errCode);
    if (outboundFPHandle == NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to create the OUTBOUND fast path [Error %d]\n", errCode);
        return -1;
    }

    /*****************************************************************************************
     * Wait for some time in order to get the routes resolved and the interfaces offloaded
     *****************************************************************************************/
    sleep(5);

    /* Sanity Check: Ensure that the interface has been offloaded. */
    ifHandle = Netfp_findInterface (testNetfpClientHandle, "eth0", &ifCfg, &errCode);
    if (ifHandle == NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Failed to find interface eth0 [Error code %d]\n", errCode);
        return -1;
    }

    /* Ensure that the interface type is valid: */
    if (ifCfg.type != Netfp_InterfaceType_ETH)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Invalid Interface type [%d] detected for eth0\n", ifCfg.type);
        return -1;
    }

    /*****************************************************************************************
     * Get the number of entries in the routing & neighbor table.
     * - There should be 1 entry in the routing cache
     * - There should only be 1 neighbor entry
     *****************************************************************************************/
    numRouteEntries = NetfpProxy_routeGetNumEntries ();
    if (numRouteEntries != 1)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Detected %d entries in the route cache after the test\n", numRouteEntries);
        return -1;
    }
    numNeighEntries = NetfpProxy_neighGetNumEntries();
    if (numNeighEntries != 1)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Detected %d entries in the neighbor cache after the test\n", numNeighEntries);
        return -1;
    }
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Detected %d entries in the Route cache\n", numRouteEntries);
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Detected %d entries in the Neighbor cache\n", numNeighEntries);

    /***********************************************************************************************
     * Delete the outbound fast path
     ***********************************************************************************************/
    if (Netfp_deleteOutboundFastPath (testNetfpClientHandle, outboundFPHandle, &errCode) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to delete the fast path [Error code %d]\n", errCode);
        return -1;
    }

    /* Sleep allow the neighbor & routes to be deleted */
    sleep(1);

    /*****************************************************************************************
     * Get the number of entries in the routing & neighbor table.
     * - There should be no more entries in either table
     *****************************************************************************************/
    numRouteEntries = NetfpProxy_routeGetNumEntries ();
    if (numRouteEntries != 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Detected %d entries in the route cache after the test\n", numRouteEntries);
        return -1;
    }
    numNeighEntries = NetfpProxy_neighGetNumEntries();
    if (numNeighEntries != 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Detected %d entries in the neighbor cache after the test\n", numNeighEntries);
        return -1;
    }
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Detected %d entries in the Route cache\n", numRouteEntries);
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Detected %d entries in the Neighbor cache\n", numNeighEntries);

    /****************************************************************************************
     * Delete the Interface:
     ****************************************************************************************/
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Cleaning up\n");
    if (NetfpProxy_testDeleteInterface ("eth0") < 0)
        return -1;

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Interface eth0 has been deleted in NETFP\n");

    /* Sanity Check: Get the number of route & neighbor entries. */
    numRouteEntries = NetfpProxy_routeGetNumEntries ();
    if (numRouteEntries != 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Detected %d entries in the route cache after interface is deleted\n", numRouteEntries);
        return -1;
    }
    numNeighEntries = NetfpProxy_neighGetNumEntries();
    if (numNeighEntries != 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Detected %d entries in the neighbor cache after interface is deleted\n", numNeighEntries);
        return -1;
    }

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "************************************************\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Basic IPv6 Route Resolution Passed\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "************************************************\n");
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function tests the following scenario:
 *         (a) Create multiple outbound fast paths with different configuration but all routed
 *             via the default gateway
 *         (b) Ensures that there is N routing entry but 1 neighbor entry.
 *         (c) Deletes 1 outbound fast path
 *
 *  Expectation:
 *      - Ensures that there are *still* N-1 routing entry and 1 neighbor entry.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t NetfpProxy_testDefaultRoute(void)
{
    int32_t                 errCode;
    Netfp_InterfaceCfg      ifCfg;
    int32_t                 index;
    Netfp_IfHandle          ifHandle;
    int32_t                 numRouteEntries;
    int32_t                 numNeighEntries;
    uint32_t                numResolveRequests = 3;
    Netfp_OutboundFPCfg     outboundFPCfg;
    Netfp_OutboundFPHandle  outboundFPHandle[64];

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "************************************************\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Route Resolution [Default gateway]\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "************************************************\n");

    /* Sanity Check: Get the number of entries in the routing table */
    numRouteEntries = NetfpProxy_routeGetNumEntries ();
    if (numRouteEntries != 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Detected %d entries in the route cache before test\n", numRouteEntries);
        return -1;
    }
    numNeighEntries = NetfpProxy_neighGetNumEntries();
    if (numNeighEntries != 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Detected %d entries in the neighbor cache before test\n", numNeighEntries);
        return -1;
    }

    /*****************************************************************************************
     * Create OUTBOUND Fast Paths:
     *****************************************************************************************/
    for (index = 0; index < numResolveRequests; index++)
    {
        /* Initialize the fast path configuration */
        memset ((void *)&outboundFPCfg, 0, sizeof (Netfp_OutboundFPCfg));

        /* Populate the Fast Path configuration. */
        outboundFPCfg.spId                    = NETFP_INVALID_SPID;
        outboundFPCfg.dstIP.ver               = Netfp_IPVersion_IPV4;
        outboundFPCfg.dstIP.addr.ipv4.u.a8[0] = 192;
        outboundFPCfg.dstIP.addr.ipv4.u.a8[1] = 168;
        outboundFPCfg.dstIP.addr.ipv4.u.a8[2] = (100 + index);
        outboundFPCfg.dstIP.addr.ipv4.u.a8[3] = (1 + index);
        outboundFPCfg.srcIP.ver               = Netfp_IPVersion_IPV4;
        outboundFPCfg.srcIP.addr.ipv4.u.a8[0] = 192;
        outboundFPCfg.srcIP.addr.ipv4.u.a8[1] = 168;
        outboundFPCfg.srcIP.addr.ipv4.u.a8[2] = 1;
        outboundFPCfg.srcIP.addr.ipv4.u.a8[3] = 2;
        sprintf (outboundFPCfg.name, "Test-%d", index);

        /* Create the outbound fast path */
        outboundFPHandle[index] = Netfp_createOutboundFastPath (testNetfpClientHandle, &outboundFPCfg, &errCode);
        if (outboundFPHandle[index] == NULL)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to create the OUTBOUND fast path [Error %d]\n", errCode);
            return -1;
        }
    }

    /*****************************************************************************************
     * Wait for some time in order to get the routes resolved and the interfaces offloaded
     *****************************************************************************************/
    sleep(5);

    /* Sanity Check: Ensure that the interface has been offloaded. */
    ifHandle = Netfp_findInterface (testNetfpClientHandle, "eth0", &ifCfg, &errCode);
    if (ifHandle == NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Failed to find interface eth0 [Error code %d]\n", errCode);
        return -1;
    }

    /* Ensure that the interface type is valid: */
    if (ifCfg.type != Netfp_InterfaceType_ETH)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Invalid Interface type [%d] detected for eth0\n", ifCfg.type);
        return -1;
    }

    /*****************************************************************************************
     * Get the number of entries in the routing & neighbor table.
     * - There should be 'numResolveRequests' in the routing table. One for each routeLookup
     *   request
     * - There should only be 1 neighbor entry; since all the routes get resolved to the same
     *   default gateway.
     *****************************************************************************************/
    numRouteEntries = NetfpProxy_routeGetNumEntries ();
    if (numRouteEntries != numResolveRequests)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Detected %d entries in the route cache after the test\n", numRouteEntries);
        return -1;
    }
    numNeighEntries = NetfpProxy_neighGetNumEntries();
    if (numNeighEntries != 1)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Detected %d entries in the neighbor cache after the test\n", numNeighEntries);
        return -1;
    }
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Detected %d entries in the Route cache\n", numRouteEntries);
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Detected %d entries in the Neighbor cache\n", numNeighEntries);

    /*****************************************************************************************
     * Delete only 1 OUTBOUND Fast Path:
     *****************************************************************************************/
    if (Netfp_deleteOutboundFastPath (testNetfpClientHandle, outboundFPHandle[0], &errCode) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to delete the fast path [Error %d]\n", errCode);
        return -1;
    }
    sleep(1);

    /*****************************************************************************************
     * Get the number of entries in the routing & neighbor table.
     * - There should be 'numResolveRequests - 1' in the routing table.
     * - There should only be 1 neighbor entry; since all the routes get resolved to the same
     *   default gateway.
     *****************************************************************************************/
    numRouteEntries = NetfpProxy_routeGetNumEntries ();
    if (numRouteEntries != (numResolveRequests - 1))
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Detected %d entries in the route cache after the test\n", numRouteEntries);
        return -1;
    }
    numNeighEntries = NetfpProxy_neighGetNumEntries();
    if (numNeighEntries != 1)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Detected %d entries in the neighbor cache after the test\n", numNeighEntries);
        return -1;
    }
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Detected %d entries in the Route cache\n", numRouteEntries);
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Detected %d entries in the Neighbor cache\n", numNeighEntries);

    /****************************************************************************************
     * Delete the Interface:
     ****************************************************************************************/
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Cleaning up\n");
    if (NetfpProxy_testDeleteInterface ("eth0") < 0)
        return -1;

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Interface eth0 has been deleted in NETFP\n");

    /* Sanity Check: Get the number of route & neighbor entries. */
    numRouteEntries = NetfpProxy_routeGetNumEntries ();
    if (numRouteEntries != 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Detected %d entries in the route cache after interface is deleted\n", numRouteEntries);
        return -1;
    }
    numNeighEntries = NetfpProxy_neighGetNumEntries();
    if (numNeighEntries != 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Detected %d entries in the neighbor cache after interface is deleted\n", numNeighEntries);
        return -1;
    }

    /* Sanity Check: Ensure that none of the fast paths exist; since these have been deleted */
    for (index = 0; index < numResolveRequests; index++)
    {
        char fastPathName[NETFP_MAX_CHAR];

        /* Construct the fast path name: */
        sprintf (fastPathName, "Test-%d", index);
        if (Netfp_findOutboundFastPath (testNetfpClientHandle, &fastPathName[0], &errCode) != NULL)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Fast Path '%s' exists\n", fastPathName);
            return -1;
        }
    }

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "************************************************\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Route Resolution [Default gateway] Passed\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "************************************************\n");
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function tests the following scenario:
 *         (a) Create multiple outbound fast paths with the same configuration
 *         (b) Ensures that there is 1 routing entry and 1 neighbor entry.
 *         (c) Deletes 1 outbound fast path
 *
 *  Expectation:
 *      - Ensures that there is *still* 1 routing entry and 1 neighbor entry.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t NetfpProxy_testDuplicateFastPath(void)
{
    int32_t                 errCode;
    Netfp_InterfaceCfg      ifCfg;
    int32_t                 index;
    Netfp_IfHandle          ifHandle;
    int32_t                 numRouteEntries;
    int32_t                 numNeighEntries;
    uint32_t                numResolveRequests = 5;
    Netfp_OutboundFPCfg     outboundFPCfg;
    Netfp_OutboundFPHandle  outboundFPHandle[64];

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "************************************************\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Duplicate Fast Path\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "************************************************\n");

    /* Sanity Check: Get the number of entries in the routing table */
    numRouteEntries = NetfpProxy_routeGetNumEntries ();
    if (numRouteEntries != 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Detected %d entries in the route cache before test\n", numRouteEntries);
        return -1;
    }
    numNeighEntries = NetfpProxy_neighGetNumEntries();
    if (numNeighEntries != 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Detected %d entries in the neighbor cache before test\n", numNeighEntries);
        return -1;
    }

    /*****************************************************************************************
     * Create multiple OUTBOUND Fast Path:
     *****************************************************************************************/
    for (index = 0; index < numResolveRequests; index++)
    {
        /* Initialize the fast path configuration */
        memset ((void *)&outboundFPCfg, 0, sizeof (Netfp_OutboundFPCfg));

        /* Populate the Fast Path configuration. */
        outboundFPCfg.spId                    = NETFP_INVALID_SPID;
        outboundFPCfg.dstIP.ver               = Netfp_IPVersion_IPV4;
        outboundFPCfg.dstIP.addr.ipv4.u.a8[0] = 192;
        outboundFPCfg.dstIP.addr.ipv4.u.a8[1] = 168;
        outboundFPCfg.dstIP.addr.ipv4.u.a8[2] = 1;
        outboundFPCfg.dstIP.addr.ipv4.u.a8[3] = 1;
        outboundFPCfg.srcIP.ver               = Netfp_IPVersion_IPV4;
        outboundFPCfg.srcIP.addr.ipv4.u.a8[0] = 192;
        outboundFPCfg.srcIP.addr.ipv4.u.a8[1] = 168;
        outboundFPCfg.srcIP.addr.ipv4.u.a8[2] = 1;
        outboundFPCfg.srcIP.addr.ipv4.u.a8[3] = 2;
        sprintf (outboundFPCfg.name, "Test-%d", index);

        /* Create the outbound fast path */
        outboundFPHandle[index] = Netfp_createOutboundFastPath  (testNetfpClientHandle, &outboundFPCfg, &errCode);
        if (outboundFPHandle[index] == NULL)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to create the OUTBOUND fast path [Error %d]\n", errCode);
            return -1;
        }
    }

    /*****************************************************************************************
     * Wait for some time in order to get the routes resolved and the interfaces offloaded
     *****************************************************************************************/
    sleep(5);

    /* Sanity Check: Ensure that the interface has been offloaded. */
    ifHandle = Netfp_findInterface (testNetfpClientHandle, "eth0", &ifCfg, &errCode);
    if (ifHandle == NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Failed to find interface eth0 [Error code %d]\n", errCode);
        return -1;
    }

    /* Ensure that the interface type is valid: */
    if (ifCfg.type != Netfp_InterfaceType_ETH)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Invalid Interface type [%d] detected for eth0\n", ifCfg.type);
        return -1;
    }

    /*****************************************************************************************
     * Get the number of entries in the routing & neighbor table.
     * - There should only be 1 route entry since the test is resolving the route to the same
     *   destination multiple times.
     * - There should only be 1 neighbor entry; since all the routes get resolved to the same
     *   default gateway.
     *****************************************************************************************/
    numRouteEntries = NetfpProxy_routeGetNumEntries ();
    if (numRouteEntries != 1)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Detected %d entries in the route cache after the test\n", numRouteEntries);
        return -1;
    }
    numNeighEntries = NetfpProxy_neighGetNumEntries();
    if (numNeighEntries != 1)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Detected %d entries in the neighbor cache after the test\n", numNeighEntries);
        return -1;
    }
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Detected %d entries in the Route cache\n", numRouteEntries);
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Detected %d entries in the Neighbor cache\n", numNeighEntries);

    /*****************************************************************************************
     * Delete one OUTBOUND Fast Path:
     *****************************************************************************************/
    if (Netfp_deleteOutboundFastPath (testNetfpClientHandle, outboundFPHandle[0], &errCode) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to delete the fast path [Error %d]\n", errCode);
        return -1;
    }

    /* Sleep allow the neighbor & routes to be deleted */
    sleep(1);

    /*****************************************************************************************
     * Get the number of entries in the routing & neighbor table.
     * - The stop resolution should have had no impact on the routing & neighbor table since
     *   there are other entities still using these.
     *****************************************************************************************/
    numRouteEntries = NetfpProxy_routeGetNumEntries ();
    if (numRouteEntries != 1)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Detected %d entries in the route cache after the test\n", numRouteEntries);
        return -1;
    }
    numNeighEntries = NetfpProxy_neighGetNumEntries();
    if (numNeighEntries != 1)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Detected %d entries in the neighbor cache after the test\n", numNeighEntries);
        return -1;
    }
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Detected %d entries in the Route cache\n", numRouteEntries);
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Detected %d entries in the Neighbor cache\n", numNeighEntries);

    /****************************************************************************************
     * Delete the Interface:
     ****************************************************************************************/
    if (NetfpProxy_testDeleteInterface ("eth0") < 0)
        return -1;

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Interface eth0 has been deleted in NETFP\n");

    /* Sanity Check: Get the number of route & neighbor entries. */
    numRouteEntries = NetfpProxy_routeGetNumEntries ();
    if (numRouteEntries != 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Detected %d entries in the route cache after interface is deleted\n", numRouteEntries);
        return -1;
    }
    numNeighEntries = NetfpProxy_neighGetNumEntries();
    if (numNeighEntries != 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Detected %d entries in the neighbor cache after interface is deleted\n", numNeighEntries);
        return -1;
    }

    /* Sanity Check: Ensure that none of the fast paths exist; since these have been deleted */
    for (index = 0; index < numResolveRequests; index++)
    {
        char fastPathName[NETFP_MAX_CHAR];

        /* Construct the fast path name: */
        sprintf (fastPathName, "Test-%d", index);
        if (Netfp_findOutboundFastPath (testNetfpClientHandle, &fastPathName[0], &errCode) != NULL)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Fast Path '%s' exists\n", fastPathName);
            return -1;
        }
    }

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "************************************************\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Duplicate Fast Path Passed\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "************************************************\n");
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function tests the following scenario:
 *         (a) Create multiple outbound fast paths
 *         (b) Ensures that the neighbor and route entries match
 *         (c) Issues single route flush
 *
 *  Expectation:
 *      - Since there was no update to the routing table; the route flush has no impact
 *        on the number of neighbor & route entries in step(b)
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t NetfpProxy_testBasicFlush(void)
{
    int32_t                 index;
    int32_t                 numRouteEntries;
    int32_t                 numNeighEntries;
    uint32_t                numResolveRequests = 5;
    int32_t                 errCode;
    Netfp_OutboundFPCfg     outboundFPCfg;
    Netfp_OutboundFPHandle  outboundFPHandle[64];

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "************************************************\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Basic Flush Route Test\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "************************************************\n");

    /* Sanity Check: Get the number of entries in the routing table */
    numRouteEntries = NetfpProxy_routeGetNumEntries ();
    if (numRouteEntries != 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Detected %d entries in the route cache before test\n", numRouteEntries);
        return -1;
    }
    numNeighEntries = NetfpProxy_neighGetNumEntries();
    if (numNeighEntries != 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Detected %d entries in the neighbor cache before test\n", numNeighEntries);
        return -1;
    }

    /*****************************************************************************************
     * Create multiple OUTBOUND Fast Path:
     *****************************************************************************************/
    for (index = 0; index < numResolveRequests; index++)
    {
        /* Initialize the fast path configuration */
        memset ((void *)&outboundFPCfg, 0, sizeof (Netfp_OutboundFPCfg));

        /* Populate the Fast Path configuration. */
        outboundFPCfg.spId                    = NETFP_INVALID_SPID;
        outboundFPCfg.dstIP.ver               = Netfp_IPVersion_IPV4;
        outboundFPCfg.dstIP.addr.ipv4.u.a8[0] = 192;
        outboundFPCfg.dstIP.addr.ipv4.u.a8[1] = 168;
        outboundFPCfg.dstIP.addr.ipv4.u.a8[2] = (100 + index);
        outboundFPCfg.dstIP.addr.ipv4.u.a8[3] = 1;
        outboundFPCfg.srcIP.ver               = Netfp_IPVersion_IPV4;
        outboundFPCfg.srcIP.addr.ipv4.u.a8[0] = 192;
        outboundFPCfg.srcIP.addr.ipv4.u.a8[1] = 168;
        outboundFPCfg.srcIP.addr.ipv4.u.a8[2] = 1;
        outboundFPCfg.srcIP.addr.ipv4.u.a8[3] = 2;
        sprintf (outboundFPCfg.name, "Test-%d", index);

        /* Create the outbound fast path */
        outboundFPHandle[index] = Netfp_createOutboundFastPath  (testNetfpClientHandle, &outboundFPCfg, &errCode);
        if (outboundFPHandle[index] == NULL)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to create the OUTBOUND fast path [Error %d]\n", errCode);
            return -1;
        }
    }

    /* Sleep for some time: This will allow the caches to be populated. */
    sleep(10);

    /* Sanity Check: We have multiple routing blocks but all of them resolve to the same neighbor (i.e. the default gateway) */
    numRouteEntries = NetfpProxy_routeGetNumEntries ();
    if (numRouteEntries != numResolveRequests)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Invalid number of entries %d in the route cache\n", numRouteEntries);
        return -1;
    }
    numNeighEntries = NetfpProxy_neighGetNumEntries();
    if (numNeighEntries != 1)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Invalid number of entries %d in the neighbor cache\n", numNeighEntries);
        return -1;
    }

    /*****************************************************************************************
     * Flush Route:
     *****************************************************************************************/
    NetfpProxy_testFlushRoute();

    /* Sleep for some time */
    sleep(10);

    /* Sanity Check: After the flush the route resolution should have the same number of routing blocks with all of them being resolved to
     * the same neighbor (i.e. the default gateway) */
    numRouteEntries = NetfpProxy_routeGetNumEntries ();
    if (numRouteEntries != numResolveRequests)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Invalid number of entries in the route cache Expected %d Detected %d\n",
                           numResolveRequests, numRouteEntries);
        return -1;
    }
    numNeighEntries = NetfpProxy_neighGetNumEntries();
    if (numNeighEntries != 1)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Invalid number of entries in the neighbor cache Detected %d\n",
                           numNeighEntries);
        return -1;
    }

    /****************************************************************************************
     * Delete the Interface:
     ****************************************************************************************/
    if (NetfpProxy_testDeleteInterface ("eth0") < 0)
        return -1;

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Interface eth0 has been deleted in NETFP\n");

    /* Sanity Check: Ensure that none of the fast paths exist; since these have been deleted */
    for (index = 0; index < numResolveRequests; index++)
    {
        char fastPathName[NETFP_MAX_CHAR];

        /* Construct the fast path name: */
        sprintf (fastPathName, "Test-%d", index);
        if (Netfp_findOutboundFastPath (testNetfpClientHandle, &fastPathName[0], &errCode) != NULL)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Fast Path '%s' exists\n", fastPathName);
            return -1;
        }
    }

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "************************************************\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Basic Flush Route Test Passed\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "************************************************\n");
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function tests the following scenario:
 *         (a) Create multiple outbound fast paths
 *         (b) Ensures that the neighbor and route entries match
 *         (c) Issues multiple route flushes in a tight loop
 *
 *  Expectation:
 *      - Since there was no update to the routing table; the route flush has no impact
 *        on the number of neighbor & route entries in step(b)
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t NetfpProxy_testMultipleFlush(void)
{
    int32_t                 index;
    int32_t                 numRouteEntries;
    int32_t                 numNeighEntries;
    uint32_t                numResolveRequests = 8;
    int32_t                 errCode;
    Netfp_OutboundFPCfg     outboundFPCfg;
    Netfp_OutboundFPHandle  outboundFPHandle[64];

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "************************************************\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Multiple Flush Route Test\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "************************************************\n");

    /* Sanity Check: Get the number of entries in the routing table */
    numRouteEntries = NetfpProxy_routeGetNumEntries ();
    if (numRouteEntries != 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Detected %d entries in the route cache before test\n", numRouteEntries);
        return -1;
    }
    numNeighEntries = NetfpProxy_neighGetNumEntries();
    if (numNeighEntries != 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Detected %d entries in the neighbor cache before test\n", numNeighEntries);
        return -1;
    }

    /*****************************************************************************************
     * Create multiple outbound fast paths
     *****************************************************************************************/
    for (index = 0; index < numResolveRequests; index++)
    {
        /* Initialize the fast path configuration */
        memset ((void *)&outboundFPCfg, 0, sizeof (Netfp_OutboundFPCfg));

        /* Populate the Fast Path configuration. */
        outboundFPCfg.spId                    = NETFP_INVALID_SPID;
        outboundFPCfg.dstIP.ver               = Netfp_IPVersion_IPV4;
        outboundFPCfg.dstIP.addr.ipv4.u.a8[0] = 192;
        outboundFPCfg.dstIP.addr.ipv4.u.a8[1] = 168;
        outboundFPCfg.dstIP.addr.ipv4.u.a8[2] = (100 + index);
        outboundFPCfg.dstIP.addr.ipv4.u.a8[3] = 1;
        outboundFPCfg.srcIP.ver               = Netfp_IPVersion_IPV4;
        outboundFPCfg.srcIP.addr.ipv4.u.a8[0] = 192;
        outboundFPCfg.srcIP.addr.ipv4.u.a8[1] = 168;
        outboundFPCfg.srcIP.addr.ipv4.u.a8[2] = 1;
        outboundFPCfg.srcIP.addr.ipv4.u.a8[3] = 2;
        sprintf (outboundFPCfg.name, "Test-%d", index);

        /* Create the outbound fast path */
        outboundFPHandle[index] = Netfp_createOutboundFastPath  (testNetfpClientHandle, &outboundFPCfg, &errCode);
        if (outboundFPHandle[index] == NULL)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to create the OUTBOUND fast path [Error %d]\n", errCode);
            return -1;
        }
    }
    sleep(10);

    /* Sanity Check: We have multiple routing blocks but all of them resolve to the same neighbor (i.e. the default gateway) */
    numRouteEntries = NetfpProxy_routeGetNumEntries ();
    if (numRouteEntries != numResolveRequests)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Invalid number of entries %d in the route cache\n", numRouteEntries);
        return -1;
    }
    numNeighEntries = NetfpProxy_neighGetNumEntries();
    if (numNeighEntries != 1)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Invalid number of entries %d in the neighbor cache\n", numNeighEntries);
        return -1;
    }

    /*****************************************************************************************
     * Multiple Flushes:
     *****************************************************************************************/
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "************************************************\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Initiating Multiple Flushes\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "************************************************\n");
    for (index = 0; index < 16; index++)
        NetfpProxy_testFlushRoute();

    /* Sleep for some time */
    sleep(10);

    /* Sanity Check: Ensure that we have the same number of entries after the flush */
    numRouteEntries = NetfpProxy_routeGetNumEntries ();
    if (numRouteEntries != numResolveRequests)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Invalid number of entries in the route cache Expected %d Detected %d\n",
                           numResolveRequests, numRouteEntries);
        return -1;
    }
    numNeighEntries = NetfpProxy_neighGetNumEntries();
    if (numNeighEntries != 1)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Invalid number of entries in the neighbor cache Detected %d\n",
                           numNeighEntries);
        return -1;
    }

    /****************************************************************************************
     * Delete the Interface:
     ****************************************************************************************/
    if (NetfpProxy_testDeleteInterface ("eth0") < 0)
        return -1;

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Interface eth0 has been deleted in NETFP\n");

    /* Sanity Check: Ensure that none of the fast paths exist; since these have been deleted */
    for (index = 0; index < numResolveRequests; index++)
    {
        char fastPathName[NETFP_MAX_CHAR];

        /* Construct the fast path name: */
        sprintf (fastPathName, "Test-%d", index);
        if (Netfp_findOutboundFastPath (testNetfpClientHandle, &fastPathName[0], &errCode) != NULL)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Fast Path '%s' exists\n", fastPathName);
            return -1;
        }
    }

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "************************************************\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Multiple Flush Route Test Passed\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "************************************************\n");
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function tests the following scenario:
 *         (a) Create multiple outbound fast paths
 *         (b) Ensures that the neighbor and route entries match
 *         (c) Delete each outbound fast path and then flush the route cache
 *
 *  Expectation:
 *      - All the fast paths should be deleted
 *      - There should be 0 Neighbor & 0 route entry
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t NetfpProxy_testDeleteFastPathWithFlush(void)
{
    int32_t                 errCode;
    Netfp_InterfaceCfg      ifCfg;
    int32_t                 index;
    Netfp_IfHandle          ifHandle;
    Netfp_OutboundFPCfg     outboundFPCfg;
    Netfp_OutboundFPHandle  outboundFPHandle[64];
    int32_t                 numRouteEntries;
    int32_t                 numNeighEntries;
    uint32_t                numResolveRequests = 16;

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "************************************************\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Delete Fast Path and Flush\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "************************************************\n");

    /* Sanity Check: Get the number of entries in the routing table */
    numRouteEntries = NetfpProxy_routeGetNumEntries ();
    if (numRouteEntries != 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Detected %d entries in the route cache before test\n", numRouteEntries);
        return -1;
    }
    numNeighEntries = NetfpProxy_neighGetNumEntries();
    if (numNeighEntries != 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Detected %d entries in the neighbor cache before test\n", numNeighEntries);
        return -1;
    }

    /*****************************************************************************************
     * Create multiple OUTBOUND Fast Path:
     *****************************************************************************************/
    for (index = 0; index < numResolveRequests; index++)
    {
        /* Initialize the fast path configuration */
        memset ((void *)&outboundFPCfg, 0, sizeof (Netfp_OutboundFPCfg));

        /* Populate the Fast Path configuration. */
        outboundFPCfg.spId                    = NETFP_INVALID_SPID;
        outboundFPCfg.dstIP.ver               = Netfp_IPVersion_IPV4;
        outboundFPCfg.dstIP.addr.ipv4.u.a8[0] = 192;
        outboundFPCfg.dstIP.addr.ipv4.u.a8[1] = 168;
        outboundFPCfg.dstIP.addr.ipv4.u.a8[2] = (200 + index);
        outboundFPCfg.dstIP.addr.ipv4.u.a8[3] = 1;
        outboundFPCfg.srcIP.ver               = Netfp_IPVersion_IPV4;
        outboundFPCfg.srcIP.addr.ipv4.u.a8[0] = 192;
        outboundFPCfg.srcIP.addr.ipv4.u.a8[1] = 168;
        outboundFPCfg.srcIP.addr.ipv4.u.a8[2] = 1;
        outboundFPCfg.srcIP.addr.ipv4.u.a8[3] = 2;
        sprintf (outboundFPCfg.name, "Test-%d", index);

        /* Create the outbound fast path */
        outboundFPHandle[index] = Netfp_createOutboundFastPath  (testNetfpClientHandle, &outboundFPCfg, &errCode);
        if (outboundFPHandle[index] == NULL)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to create the OUTBOUND fast path [Error %d]\n", errCode);
            return -1;
        }
    }
    sleep(5);

    /* Sanity Check: Ensure that the interface has been offloaded. */
    ifHandle = Netfp_findInterface (testNetfpClientHandle, "eth0", &ifCfg, &errCode);
    if (ifHandle == NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Failed to find interface eth0 [Error code %d]\n", errCode);
        return -1;
    }

    /* Ensure that the interface type is valid: */
    if (ifCfg.type != Netfp_InterfaceType_ETH)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Invalid Interface type [%d] detected for eth0\n", ifCfg.type);
        return -1;
    }

    /*****************************************************************************************
     * Get the number of entries in the routing & neighbor table.
     * - There should be 'numResolveRequests' in the routing table. One for each routeLookup
     *   request
     * - There should only be 1 neighbor entry; since all the routes get resolved to the same
     *   default gateway.
     *****************************************************************************************/
    numRouteEntries = NetfpProxy_routeGetNumEntries ();
    if (numRouteEntries != numResolveRequests)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Detected %d entries in the route cache\n", numRouteEntries);
        return -1;
    }
    numNeighEntries = NetfpProxy_neighGetNumEntries();
    if (numNeighEntries != 1)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Detected %d entries in the neighbor cache\n", numNeighEntries);
        return -1;
    }
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Route Cache = %d entries Neighbor Cache=%d entries\n", numRouteEntries, numNeighEntries);

    /*****************************************************************************************
     * Delete the OUTBOUND Fast Paths & Flush the routes also
     *****************************************************************************************/
    for (index = 0; index < numResolveRequests; index++)
    {
        /* Delete the outbound fast path: */
        if (Netfp_deleteOutboundFastPath (testNetfpClientHandle, outboundFPHandle[index], &errCode) < 0)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to delete the fast path [Error code %d]\n", errCode);
            return -1;
        }
        NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Deleted Fast Path @ index %d %p\n", index, outboundFPHandle[index]);

        /*****************************************************************************************
         * Flush the route cache:
         *****************************************************************************************/
        NetfpProxy_testFlushRoute();
    }
    sleep(10);

    /*****************************************************************************************
     * Get the number of entries in the routing & neighbor table.
     * - There should be no more entries in the Neighbor & Routing entries
     *****************************************************************************************/
    numRouteEntries = NetfpProxy_routeGetNumEntries ();
    if (numRouteEntries != 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Detected %d entries in the route cache after stop monitoring\n", numRouteEntries);
        return -1;
    }
    numNeighEntries = NetfpProxy_neighGetNumEntries();
    if (numNeighEntries != 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Detected %d entries in the neighbor cache after stop monitoring\n", numNeighEntries);
        return -1;
    }

    /* Sanity Check: Ensure that none of the fast paths exist; since these have been deleted */
    for (index = 0; index < numResolveRequests; index++)
    {
        char fastPathName[NETFP_MAX_CHAR];

        /* Construct the fast path name: */
        sprintf (fastPathName, "Test-%d", index);
        if (Netfp_findOutboundFastPath (testNetfpClientHandle, &fastPathName[0], &errCode) != NULL)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Fast Path '%s' exists\n", fastPathName);
            return -1;
        }
    }

    /****************************************************************************************
     * Delete the Interface:
     ****************************************************************************************/
    if (NetfpProxy_testDeleteInterface ("eth0") < 0)
        return -1;

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Interface eth0 has been deleted in NETFP\n");

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "************************************************\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Delete Fast Path and Flush Passed\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "************************************************\n");
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function tests the following scenario:
 *         (a) Create multiple outbound security associations
 *         (b) Ensures that the neighbor and route entries match
 *         (c) Delete each outbound security association and then flush the route cache
 *
 *  Expectation:
 *      - All the fast paths should be deleted
 *      - There should be 0 Neighbor & 0 route entry
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t NetfpProxy_testDeleteSAWithFlush(void)
{
    int32_t                 errCode;
    Netfp_InterfaceCfg      ifCfg;
    int32_t                 index;
    Netfp_IfHandle          ifHandle;
    Netfp_SACfg             saConfig;
    Netfp_SAHandle          saHandle[64];
    int32_t                 numRouteEntries;
    int32_t                 numNeighEntries;
    uint32_t                numResolveRequests = 16;

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "************************************************\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Delete SA and Flush\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "************************************************\n");

    /* Sanity Check: Get the number of entries in the routing table */
    numRouteEntries = NetfpProxy_routeGetNumEntries ();
    if (numRouteEntries != 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Detected %d entries in the route cache before test\n", numRouteEntries);
        return -1;
    }
    numNeighEntries = NetfpProxy_neighGetNumEntries();
    if (numNeighEntries != 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Detected %d entries in the neighbor cache before test\n", numNeighEntries);
        return -1;
    }

    /*****************************************************************************************
     * Create multiple OUTBOUND Security associations:
     *****************************************************************************************/
    for (index = 0; index < numResolveRequests; index++)
    {
        /* Initialize the SA configuration */
        memset ((void *)&saConfig, 0, sizeof (Netfp_SACfg));

        /* Populate the SA configuration. */
        saConfig.direction              = Netfp_Direction_OUTBOUND;
        saConfig.spi                    = index;
        saConfig.ipsecCfg.protocol      = Netfp_IPSecProto_IPSEC_ESP;
        saConfig.ipsecCfg.mode          = Netfp_IPSecMode_IPSEC_TUNNEL;
        saConfig.ipsecCfg.authMode      = Netfp_IpsecAuthMode_HMAC_SHA1;
        saConfig.ipsecCfg.encMode       = Netfp_IpsecCipherMode_NULL;
        saConfig.ipsecCfg.keyMacSize    = 12;

        if (saConfig.ipsecCfg.authMode == Netfp_IpsecAuthMode_HMAC_MD5)
            saConfig.ipsecCfg.keyAuthSize   = 16;
        else if (saConfig.ipsecCfg.authMode == Netfp_IpsecAuthMode_HMAC_SHA1)
            saConfig.ipsecCfg.keyAuthSize   = 20;
        else
            saConfig.ipsecCfg.keyAuthSize   = 0;

        if (saConfig.ipsecCfg.encMode == Netfp_IpsecCipherMode_NULL)
            saConfig.ipsecCfg.keyEncSize = 0;
        else if (saConfig.ipsecCfg.encMode == Netfp_IpsecCipherMode_AES_CTR)
            saConfig.ipsecCfg.keyEncSize = 20;
        else
            saConfig.ipsecCfg.keyEncSize = 24;

        /* Copy the keys. */
        memset (&saConfig.ipsecCfg.keyAuth, 0xAA, saConfig.ipsecCfg.keyAuthSize);
        memset (&saConfig.ipsecCfg.keyEnc,  0xBB, saConfig.ipsecCfg.keyEncSize);

        /* Configure the lifetime */
        saConfig.ipsecCfg.lifetime.softByteLimit   = 0xFFFFFFFFFFFFFFFF;
        saConfig.ipsecCfg.lifetime.hardByteLimit   = 0xFFFFFFFFFFFFFFFF;
        saConfig.ipsecCfg.lifetime.softPacketLimit = 0xFFFFFFFFFFFFFFFF;
        saConfig.ipsecCfg.lifetime.hardPacketLimit = 0xFFFFFFFFFFFFFFFF;

        /* Populate the IP address: */
        saConfig.srcIP.ver = Netfp_IPVersion_IPV4;
        saConfig.srcIP.addr.ipv4.u.a8[0] = 192;
        saConfig.srcIP.addr.ipv4.u.a8[1] = 168;
        saConfig.srcIP.addr.ipv4.u.a8[2] = 1;
        saConfig.srcIP.addr.ipv4.u.a8[3] = 2;
        saConfig.dstIP.ver = Netfp_IPVersion_IPV4;
        saConfig.dstIP.addr.ipv4.u.a8[0] = 11;
        saConfig.dstIP.addr.ipv4.u.a8[1] = 11;
        saConfig.dstIP.addr.ipv4.u.a8[2] = (11 + index);
        saConfig.dstIP.addr.ipv4.u.a8[3] = 1;

        /* Create the outbound SA. */
        saHandle[index] = Netfp_addSA (testNetfpClientHandle, &saConfig, &errCode);
        if (saHandle == NULL)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to create the SA [Error %d]\n", errCode);
            return -1;
        }
    }
    sleep(5);

    /* Sanity Check: Ensure that the interface has been offloaded. */
    ifHandle = Netfp_findInterface (testNetfpClientHandle, "eth0", &ifCfg, &errCode);
    if (ifHandle == NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Failed to find interface eth0 [Error code %d]\n", errCode);
        return -1;
    }

    /* Ensure that the interface type is valid: */
    if (ifCfg.type != Netfp_InterfaceType_ETH)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Invalid Interface type [%d] detected for eth0\n", ifCfg.type);
        return -1;
    }

    /*****************************************************************************************
     * Get the number of entries in the routing & neighbor table.
     * - There should be 'numResolveRequests' in the routing table. One for each routeLookup
     *   request
     * - There should only be 1 neighbor entry; since all the routes get resolved to the same
     *   default gateway.
     *****************************************************************************************/
    numRouteEntries = NetfpProxy_routeGetNumEntries ();
    if (numRouteEntries != numResolveRequests)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Detected %d entries in the route cache\n", numRouteEntries);
        return -1;
    }
    numNeighEntries = NetfpProxy_neighGetNumEntries();
    if (numNeighEntries != 1)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Detected %d entries in the neighbor cache\n", numNeighEntries);
        return -1;
    }
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Route Cache = %d entries Neighbor Cache=%d entries\n", numRouteEntries, numNeighEntries);

    /*****************************************************************************************
     * Delete the OUTBOUND Security associations & Flush the routes also
     *****************************************************************************************/
    for (index = 0; index < numResolveRequests; index++)
    {
        /* Delete the outbound security association: */
        if (Netfp_delSA (testNetfpClientHandle, saHandle[index], &errCode) < 0)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to delete the SA [Error code %d]\n", errCode);
            return -1;
        }
        NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Deleted SA @ index %d %p\n", index, saHandle[index]);

        /*****************************************************************************************
         * Flush the route cache:
         *****************************************************************************************/
        NetfpProxy_testFlushRoute();
    }
    sleep(10);

    /*****************************************************************************************
     * Get the number of entries in the routing & neighbor table.
     * - There should be no more entries in the Neighbor & Routing entries
     *****************************************************************************************/
    numRouteEntries = NetfpProxy_routeGetNumEntries ();
    if (numRouteEntries != 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Detected %d entries in the route cache after stop monitoring\n", numRouteEntries);
        return -1;
    }
    numNeighEntries = NetfpProxy_neighGetNumEntries();
    if (numNeighEntries != 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Detected %d entries in the neighbor cache after stop monitoring\n", numNeighEntries);
        return -1;
    }

    /****************************************************************************************
     * Delete the Interface:
     ****************************************************************************************/
    if (NetfpProxy_testDeleteInterface ("eth0") < 0)
        return -1;

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Interface eth0 has been deleted in NETFP\n");

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "************************************************\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Delete SA and Flush Passed\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "************************************************\n");
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function tests the following scenario:
 *         (a) Create a fast path FP[0] which is unresolved
 *         (b) Modify the routing table to add a default gateway
 *         (c) Create a fast path FP[1] which is now resolved
 *         (d) Delete the FP[0]
 *         (e) Flush the route cache
 *
 *  Expectation:
 *      - FP[1] remains active and valid.
 *      - There is 1 Neighbor & 1 route entry which matches FP[1]
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t NetfpProxy_testMultipleFastPathWithFlush(void)
{
    int32_t                 errCode;
    int32_t                 index;
    int32_t                 status;
    Netfp_OutboundFPCfg     outboundFPCfg;
    Netfp_OutboundFPHandle  outboundFPHandle[64];
    int32_t                 numRouteEntries;
    int32_t                 numNeighEntries;
    uint32_t                numResolveRequests = 1;

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "************************************************\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Multiple Fast Path with flush\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "************************************************\n");

    /* Sanity Check: Get the number of entries in the routing table */
    numRouteEntries = NetfpProxy_routeGetNumEntries ();
    if (numRouteEntries != 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Detected %d entries in the route cache before test\n", numRouteEntries);
        return -1;
    }
    numNeighEntries = NetfpProxy_neighGetNumEntries();
    if (numNeighEntries != 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Detected %d entries in the neighbor cache before test\n", numNeighEntries);
        return -1;
    }

    /*****************************************************************************************
     * Delete the default gateway:
     *****************************************************************************************/
    system ("route del default gw 192.168.1.1");

    /*****************************************************************************************
     * Create first OUTBOUND Fast Path:
     *****************************************************************************************/
    memset ((void *)&outboundFPCfg, 0, sizeof (Netfp_OutboundFPCfg));

    /* Populate the Fast Path configuration. */
    outboundFPCfg.spId                    = NETFP_INVALID_SPID;
    outboundFPCfg.dstIP.ver               = Netfp_IPVersion_IPV4;
    outboundFPCfg.dstIP.addr.ipv4.u.a8[0] = 100;
    outboundFPCfg.dstIP.addr.ipv4.u.a8[1] = 100;
    outboundFPCfg.dstIP.addr.ipv4.u.a8[2] = 100;
    outboundFPCfg.dstIP.addr.ipv4.u.a8[3] = 1;
    outboundFPCfg.srcIP.ver               = Netfp_IPVersion_IPV4;
    outboundFPCfg.srcIP.addr.ipv4.u.a8[0] = 192;
    outboundFPCfg.srcIP.addr.ipv4.u.a8[1] = 168;
    outboundFPCfg.srcIP.addr.ipv4.u.a8[2] = 1;
    outboundFPCfg.srcIP.addr.ipv4.u.a8[3] = 2;
    sprintf (outboundFPCfg.name, "Test-0");

    /* Create the outbound fast path: This fast path will be unresolved since there is no default gateway. */
    outboundFPHandle[0] = Netfp_createOutboundFastPath  (testNetfpClientHandle, &outboundFPCfg, &errCode);
    if (outboundFPHandle[0] == NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to create the OUTBOUND fast path [Error %d]\n", errCode);
        return -1;
    }
    sleep(5);

    /* Sanity Check: Since there is no route; the outbound fast path will remain INACTIVE */
    if (Netfp_isOutboundFastPathActive(testNetfpClientHandle, outboundFPHandle[0], &status, &errCode) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Getting OUTBOUND fast path status failed [Error %d]\n", errCode);
        return -1;
    }
    if (status != 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: OUTBOUND fast path status reported to be active %d\n", status);
        return -1;
    }
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Created FP[0] Handle %p in INACTIVE state\n", outboundFPHandle[0]);

    /* Sanity Check: Get the number of route & neighbor entries. Since the route was not resolved the cache is empty. */
    numRouteEntries = NetfpProxy_routeGetNumEntries ();
    if (numRouteEntries != 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Detected %d entries in the route cache\n", numRouteEntries);
        return -1;
    }
    numNeighEntries = NetfpProxy_neighGetNumEntries();
    if (numNeighEntries != 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Detected %d entries in the neighbor cache\n", numNeighEntries);
        return -1;
    }
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Route Cache = %d entries Neighbor Cache=%d entries\n", numRouteEntries, numNeighEntries);

    /*****************************************************************************************
     * Add the default gateway:
     *****************************************************************************************/
    system ("route add default gw 192.168.1.1");

    /*****************************************************************************************
     * Create second OUTBOUND Fast Path:
     *****************************************************************************************/
    outboundFPCfg.spId                    = NETFP_INVALID_SPID;
    outboundFPCfg.dstIP.ver               = Netfp_IPVersion_IPV4;
    outboundFPCfg.dstIP.addr.ipv4.u.a8[0] = 100;
    outboundFPCfg.dstIP.addr.ipv4.u.a8[1] = 100;
    outboundFPCfg.dstIP.addr.ipv4.u.a8[2] = 100;
    outboundFPCfg.dstIP.addr.ipv4.u.a8[3] = 1;
    outboundFPCfg.srcIP.ver               = Netfp_IPVersion_IPV4;
    outboundFPCfg.srcIP.addr.ipv4.u.a8[0] = 192;
    outboundFPCfg.srcIP.addr.ipv4.u.a8[1] = 168;
    outboundFPCfg.srcIP.addr.ipv4.u.a8[2] = 1;
    outboundFPCfg.srcIP.addr.ipv4.u.a8[3] = 2;
    sprintf (outboundFPCfg.name, "Test-1");

    /* Create the outbound fast path: This fast path will be resolved since there is a default gateway. */
    outboundFPHandle[1] = Netfp_createOutboundFastPath  (testNetfpClientHandle, &outboundFPCfg, &errCode);
    if (outboundFPHandle[1] == NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to create the OUTBOUND fast path [Error %d]\n", errCode);
        return -1;
    }
    sleep(5);

    /* Sanity Check: Since there is a route; the second outbound fast path will be ACTIVE */
    if (Netfp_isOutboundFastPathActive(testNetfpClientHandle, outboundFPHandle[1], &status, &errCode) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Getting OUTBOUND fast path status failed [Error %d]\n", errCode);
        return -1;
    }
    if (status != 1)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: OUTBOUND fast path status reported to be inactive %d\n", status);
        return -1;
    }
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Created FP[1] Handle %p in ACTIVE state\n", outboundFPHandle[1]);

    /* Sanity Check: Get the number of route & neighbor entries. */
    numRouteEntries = NetfpProxy_routeGetNumEntries ();
    if (numRouteEntries != 1)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Detected %d entries in the route cache\n", numRouteEntries);
        return -1;
    }
    numNeighEntries = NetfpProxy_neighGetNumEntries();
    if (numNeighEntries != 1)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Detected %d entries in the neighbor cache\n", numNeighEntries);
        return -1;
    }
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Route Cache = %d entries Neighbor Cache=%d entries\n", numRouteEntries, numNeighEntries);

    /*****************************************************************************************
     * Delete the first OUTBOUND Fast Path:
     *****************************************************************************************/
    if (Netfp_deleteOutboundFastPath (testNetfpClientHandle, outboundFPHandle[0], &errCode) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to delete the outbound fast path [Error code %d]\n", errCode);
        return -1;
    }
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Deleted FP[0] Handle %p\n", outboundFPHandle[0]);
    sleep(10);

    /* Sanity Check: Display the number of route & neighbor entries after the deletion of the INACTIVE Fast Path */
    numRouteEntries = NetfpProxy_routeGetNumEntries ();
    numNeighEntries = NetfpProxy_neighGetNumEntries();
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: After deleting FP[0] Route=%d Neighbor=%d entries\n", numRouteEntries, numNeighEntries);

    /* Get the fast path status for the second outbound fast path */
    if (Netfp_isOutboundFastPathActive(testNetfpClientHandle, outboundFPHandle[1], &status, &errCode) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Getting OUTBOUND fast path status failed [Error %d]\n", errCode);
        return -1;
    }
    if (status != 1)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: FP[1] status is now %d\n", status);
        return -1;
    }

    /****************************************************************************************
     * Send the ROUTE Flush command:
     ****************************************************************************************/
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Flushing the route cache\n");
    NetfpProxy_testFlushRoute();
    sleep(5);

    /* Sanity Check: Get the number of route & neighbor entries. */
    numRouteEntries = NetfpProxy_routeGetNumEntries ();
    if (numRouteEntries != 1)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Detected %d entries in the route cache\n", numRouteEntries);
        return -1;
    }
    numNeighEntries = NetfpProxy_neighGetNumEntries();
    if (numNeighEntries != 1)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Detected %d entries in the neighbor cache\n", numNeighEntries);
        return -1;
    }

    /* Get the fast path status for the second outbound fast path. It should remain ACTIVE. */
    if (Netfp_isOutboundFastPathActive(testNetfpClientHandle, outboundFPHandle[1], &status, &errCode) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Getting OUTBOUND fast path status failed [Error %d]\n", errCode);
        return -1;
    }
    if (status != 1)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: FP[1] status is now %d\n", status);
        return -1;
    }

    /****************************************************************************************
     * Delete the Interface:
     ****************************************************************************************/
    if (NetfpProxy_testDeleteInterface ("eth0") < 0)
        return -1;

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Interface eth0 has been deleted in NETFP\n");

    /* Sanity Check: Ensure that none of the fast paths exist; since these have been deleted */
    for (index = 0; index < numResolveRequests; index++)
    {
        char fastPathName[NETFP_MAX_CHAR];

        /* Construct the fast path name: */
        sprintf (fastPathName, "Test-%d", index);
        if (Netfp_findOutboundFastPath (testNetfpClientHandle, &fastPathName[0], &errCode) != NULL)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Fast Path '%s' exists\n", fastPathName);
            return -1;
        }
    }

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "************************************************\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Multiple fast path with flush Passed\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "************************************************\n");
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to test the stop monitoring with unresolved fast paths. The
 *      function tests the following scenario:
 *         (a) Create a fast path FP[0] which is unresolved
 *         (b) Modify the routing table to add a default gateway
 *         (c) Create a fast path FP[1] which is now resolved
 *         (d) Delete the FP[0]
 *         (e) Delete the FP[1]
 *
 *  Expectation:
 *      - All the FP are killed in the System
 *      - There is 0 Neighbor & 0 route entry in the proxy
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t NetfpProxy_testMultipleFastPath(void)
{
    int32_t                 errCode;
    int32_t                 index;
    int32_t                 status;
    Netfp_OutboundFPCfg     outboundFPCfg;
    Netfp_OutboundFPHandle  outboundFPHandle[64];
    int32_t                 numRouteEntries;
    int32_t                 numNeighEntries;
    uint32_t                numResolveRequests = 1;

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "************************************************\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Multiple fast path with no flush\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "************************************************\n");

    /* Sanity Check: Get the number of entries in the routing table */
    numRouteEntries = NetfpProxy_routeGetNumEntries ();
    if (numRouteEntries != 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Detected %d entries in the route cache before test\n", numRouteEntries);
        return -1;
    }
    numNeighEntries = NetfpProxy_neighGetNumEntries();
    if (numNeighEntries != 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Detected %d entries in the neighbor cache before test\n", numNeighEntries);
        return -1;
    }

    /*****************************************************************************************
     * Delete the default gateway:
     *****************************************************************************************/
    system ("route del default gw 192.168.1.1");

    /*****************************************************************************************
     * Create first OUTBOUND Fast Path:
     *****************************************************************************************/
    memset ((void *)&outboundFPCfg, 0, sizeof (Netfp_OutboundFPCfg));

    /* Populate the Fast Path configuration. */
    outboundFPCfg.spId                    = NETFP_INVALID_SPID;
    outboundFPCfg.dstIP.ver               = Netfp_IPVersion_IPV4;
    outboundFPCfg.dstIP.addr.ipv4.u.a8[0] = 192;
    outboundFPCfg.dstIP.addr.ipv4.u.a8[1] = 168;
    outboundFPCfg.dstIP.addr.ipv4.u.a8[2] = 100;
    outboundFPCfg.dstIP.addr.ipv4.u.a8[3] = 1;
    outboundFPCfg.srcIP.ver               = Netfp_IPVersion_IPV4;
    outboundFPCfg.srcIP.addr.ipv4.u.a8[0] = 192;
    outboundFPCfg.srcIP.addr.ipv4.u.a8[1] = 168;
    outboundFPCfg.srcIP.addr.ipv4.u.a8[2] = 1;
    outboundFPCfg.srcIP.addr.ipv4.u.a8[3] = 2;
    sprintf (outboundFPCfg.name, "Test-0");

    /* Create the outbound fast path: This fast path will be unresolved since there is no default gateway. */
    outboundFPHandle[0] = Netfp_createOutboundFastPath  (testNetfpClientHandle, &outboundFPCfg, &errCode);
    if (outboundFPHandle[0] == NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to create the OUTBOUND fast path [Error %d]\n", errCode);
        return -1;
    }
    sleep(5);

    /* Sanity Check: Since there is no route; the outbound fast path will remain INACTIVE */
    if (Netfp_isOutboundFastPathActive(testNetfpClientHandle, outboundFPHandle[0], &status, &errCode) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Getting OUTBOUND fast path status failed [Error %d]\n", errCode);
        return -1;
    }
    if (status != 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: OUTBOUND fast path status reported to be active %d\n", status);
        return -1;
    }
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Created FP[0] Handle %p in INACTIVE state\n", outboundFPHandle[0]);

    /* Sanity Check: Get the number of route & neighbor entries. Since the route was not resolved the cache is empty. */
    numRouteEntries = NetfpProxy_routeGetNumEntries ();
    if (numRouteEntries != 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Detected %d entries in the route cache\n", numRouteEntries);
        return -1;
    }
    numNeighEntries = NetfpProxy_neighGetNumEntries();
    if (numNeighEntries != 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Detected %d entries in the neighbor cache\n", numNeighEntries);
        return -1;
    }
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Route Cache = %d entries Neighbor Cache=%d entries\n", numRouteEntries, numNeighEntries);

    /*****************************************************************************************
     * Add the default gateway:
     *****************************************************************************************/
    system ("route add default gw 192.168.1.1");

    /*****************************************************************************************
     * Create second OUTBOUND Fast Path:
     *****************************************************************************************/
    outboundFPCfg.spId                    = NETFP_INVALID_SPID;
    outboundFPCfg.dstIP.ver               = Netfp_IPVersion_IPV4;
    outboundFPCfg.dstIP.addr.ipv4.u.a8[0] = 192;
    outboundFPCfg.dstIP.addr.ipv4.u.a8[1] = 168;
    outboundFPCfg.dstIP.addr.ipv4.u.a8[2] = 100;
    outboundFPCfg.dstIP.addr.ipv4.u.a8[3] = 1;
    outboundFPCfg.srcIP.ver               = Netfp_IPVersion_IPV4;
    outboundFPCfg.srcIP.addr.ipv4.u.a8[0] = 192;
    outboundFPCfg.srcIP.addr.ipv4.u.a8[1] = 168;
    outboundFPCfg.srcIP.addr.ipv4.u.a8[2] = 1;
    outboundFPCfg.srcIP.addr.ipv4.u.a8[3] = 2;
    sprintf (outboundFPCfg.name, "Test-1");

    /* Create the outbound fast path: This fast path will be resolved since there is a default gateway. */
    outboundFPHandle[1] = Netfp_createOutboundFastPath  (testNetfpClientHandle, &outboundFPCfg, &errCode);
    if (outboundFPHandle[1] == NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to create the OUTBOUND fast path [Error %d]\n", errCode);
        return -1;
    }
    sleep(5);

    /* Sanity Check: Since there is a route; the second outbound fast path will be ACTIVE */
    if (Netfp_isOutboundFastPathActive(testNetfpClientHandle, outboundFPHandle[1], &status, &errCode) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Getting OUTBOUND fast path status failed [Error %d]\n", errCode);
        return -1;
    }
    if (status != 1)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: OUTBOUND fast path status reported to be inactive %d\n", status);
        return -1;
    }
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Created FP[1] Handle %p in ACTIVE state\n", outboundFPHandle[1]);

    /* Sanity Check: Get the number of route & neighbor entries. */
    numRouteEntries = NetfpProxy_routeGetNumEntries ();
    if (numRouteEntries != 1)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Detected %d entries in the route cache\n", numRouteEntries);
        return -1;
    }
    numNeighEntries = NetfpProxy_neighGetNumEntries();
    if (numNeighEntries != 1)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Detected %d entries in the neighbor cache\n", numNeighEntries);
        return -1;
    }
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Route Cache = %d entries Neighbor Cache=%d entries\n", numRouteEntries, numNeighEntries);

    /*****************************************************************************************
     * Delete the first OUTBOUND Fast Path:
     *****************************************************************************************/
    if (Netfp_deleteOutboundFastPath (testNetfpClientHandle, outboundFPHandle[0], &errCode) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to delete the outbound fast path [Error code %d]\n", errCode);
        return -1;
    }
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Deleted FP[0] Handle %p\n", outboundFPHandle[0]);
    sleep(10);

    /* Sanity Check: Display the number of route & neighbor entries after the deletion of the INACTIVE Fast Path */
    numRouteEntries = NetfpProxy_routeGetNumEntries ();
    numNeighEntries = NetfpProxy_neighGetNumEntries();
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: After deleting FP[0] Route=%d Neighbor=%d entries\n", numRouteEntries, numNeighEntries);

    /* Get the fast path status for the second outbound fast path */
    if (Netfp_isOutboundFastPathActive(testNetfpClientHandle, outboundFPHandle[1], &status, &errCode) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Getting OUTBOUND fast path status failed [Error %d]\n", errCode);
        return -1;
    }
    if (status != 1)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: FP[1] status is now %d\n", status);
        return -1;
    }

    /*****************************************************************************************
     * Delete the second OUTBOUND Fast Path:
     *****************************************************************************************/
    if (Netfp_deleteOutboundFastPath (testNetfpClientHandle, outboundFPHandle[1], &errCode) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to delete the outbound fast path [Error code %d]\n", errCode);
        return -1;
    }
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Deleted FP[1] Handle %p\n", outboundFPHandle[0]);
    sleep(10);

    /* Sanity Check: There should be no entries in the route/neighbor cache */
    numRouteEntries = NetfpProxy_routeGetNumEntries ();
    numNeighEntries = NetfpProxy_neighGetNumEntries();
    if (numRouteEntries != 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Detected %d entries in the route cache\n", numRouteEntries);
        return -1;
    }
    if (numNeighEntries != 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Detected %d entries in the neighbor cache\n", numNeighEntries);
        return -1;
    }

    /****************************************************************************************
     * Delete the Interface:
     ****************************************************************************************/
    if (NetfpProxy_testDeleteInterface ("eth0") < 0)
        return -1;

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Interface eth0 has been deleted in NETFP\n");

    /* Sanity Check: Ensure that none of the fast paths exist; since these have been deleted */
    for (index = 0; index < numResolveRequests; index++)
    {
        char fastPathName[NETFP_MAX_CHAR];

        /* Construct the fast path name: */
        sprintf (fastPathName, "Test-%d", index);
        if (Netfp_findOutboundFastPath (testNetfpClientHandle, &fastPathName[0], &errCode) != NULL)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Fast Path '%s' exists\n", fastPathName);
            return -1;
        }
    }

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "************************************************\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Multiple fast path with no flush Passed\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "************************************************\n");
    return 0;
}

/**
 *  @b Description
 *  @n
 *      NETFP-Proxy Test thread which executes the unit tests
 *
 *  @param[in]
 *      Not applicable
 *
 *  @retval
 *      Not applicable
 */
void* NetfpProxy_test (void* arg)
{
    struct sockaddr_un  sockAddress;
    uint32_t*           argList;

    argList = (uint32_t*)arg;

    /* Get the arguments: */
    testNetfpClientHandle = (Netfp_ProxyServerIfFunction)(*argList++);
    testProxyServerIfFxn  = (Netfp_ClientHandle)(*argList);

    /* Open the IPC socket to communicate with the plugin */
    testCoreIPCSocket = socket (AF_UNIX, SOCK_DGRAM, 0);
    if (testCoreIPCSocket < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to create the IPC socket [Error %s]\n", strerror(errno));
        return NULL;
    }

    /* Initialize the socket address: */
    memset ((void *)&sockAddress, 0, sizeof (struct sockaddr_un));

    /* Populate the binding configuration: */
    sockAddress.sun_family = AF_UNIX;
    snprintf (sockAddress.sun_path, PATH_MAX, "%s/test_%d", Syslib_getRunTimeDirectory(), getpid());

    /* Unlink any previous name: */
    unlink (sockAddress.sun_path);

    /* Bind the socket: */
    if (bind(testCoreIPCSocket, (struct sockaddr*)&sockAddress, sizeof(struct sockaddr_un)) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR,"Error: Socket bind failed [Error: %s]\n", strerror(errno));
        return NULL;
    }

    /* Construct the name of the PROXY Plugin to which the test will be communicating: */
    memset ((void *)&proxySunAddr, 0, sizeof (struct sockaddr_un));
    proxySunAddr.sun_family = AF_UNIX;
    snprintf (proxySunAddr.sun_path, PATH_MAX, "%s/%s_%d", Syslib_getRunTimeDirectory(), NETFP_PROXY_IPC_DEFAULT_DAEMON_PATH, getpid());

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Test module will be sending messages to %s\n", proxySunAddr.sun_path);

    /******************************************************************************************************
     * Launch the NETFP Proxy unit tests: Setup the test  environment
     ******************************************************************************************************/
    system ("ifconfig eth0 192.168.1.2 mtu 1500 up");
    system ("ip addr add 7000::2/64 dev eth0");
    system ("route add default gw 192.168.1.1");

    if (NetfpProxy_testIPv4Basic() < 0)
        return NULL;

    if (NetfpProxy_testIPv6Basic() < 0)
        return NULL;

    if (NetfpProxy_testDuplicateFastPath() < 0)
        return NULL;

    if (NetfpProxy_testDefaultRoute() < 0)
        return NULL;

    if (NetfpProxy_testBasicFlush() < 0)
        return NULL;

    if (NetfpProxy_testMultipleFlush() < 0)
        return NULL;

    if (NetfpProxy_testDeleteFastPathWithFlush() < 0)
        return NULL;

    if (NetfpProxy_testDeleteSAWithFlush() < 0)
        return NULL;

    if (NetfpProxy_testMultipleFastPathWithFlush() < 0)
        return NULL;

    if (NetfpProxy_testMultipleFastPath() < 0)
        return NULL;

    /* Cleanup the test environment */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Resetting the Linux networking configuration\n");
    system ("ifconfig eth0 0.0.0.0");
    system ("ip addr del 7000::2/64 dev eth0");
    system ("route del default gw 192.168.1.1");

    /******************************************************************************************************
     * Launch the NETFP Proxy Tests for the different networking setups:
     ******************************************************************************************************/
    if (NetfpProxy_testBasicInterface() < 0)
        return NULL;

    if (NetfpProxy_testBridgeInterface() < 0)
        return NULL;

    if (NetfpProxy_testVLANInterface() < 0)
        return NULL;

    if (NetfpProxy_testBridgeWithVLANInterface() < 0)
        return NULL;

    if (NetfpProxy_testVLANBridge() < 0)
        return NULL;

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "************************************************\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Netfp Proxy Testing Passed\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "************************************************\n");

    /* Testing is complete: */
    return NULL;
}

