/*
 * Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/
 *
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
 *
*/
/* Standard library includes */
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/types.h>
#include <time.h>
#include <pthread.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <errno.h>
#include <unistd.h>

/* SYSLIB Include Files */
#include <ti/runtime/netfp/netfp.h>
#include <ti/apps/netfp_proxy/netfp_proxy_plugin.h>
#include <ti/apps/netfp_proxy/netfp_proxy_ipc.h>

/* Global variable definitions */
struct sockaddr_un          proxySunAddr;
int32_t                     proxyIpcSockFd;

/**
 * @brief   Default log file for plugin
 */
#define NETFP_PROXY_LOG_FILE    "/var/log/netfp_proxy.log"

/**
 * @brief   Current log level
 */
int32_t                    currLogLevel;

/**
 * @brief   Log file descriptor
 */
FILE*                       logFd = NULL;

/**
 *  @b Description
 *  @n
 *      Plugin logger initializer.
 *
 *  @param[in]  instanceId
 *      Instance number to uniquely identify this Proxy run
 *
 *  @retval
 *      0   -   Success
 *      <0  -   Error setting up logger
 */
int32_t my_logInit (int32_t instanceId)
{
    char    fileName[128];

    /* Get the file name to log */
    snprintf (fileName, sizeof(fileName), "%s_%d", NETFP_PROXY_LOG_FILE, instanceId);

    /* Open a file to log all messages from Proxy */
    if ((logFd  = fopen(fileName, "w")) == NULL)
    {
        printf ("Error opening log file %s\n", fileName);
        return -1;
    }

    /* Set log level by default to INFO */
    currLogLevel = NETFP_PROXY_LOG_INFO;

    /* Log setup done */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Sets up the logger level
 *
 *  @retval
 *      Not Applicable.
 */
void my_logSetLogLevel (NetfpProxy_LogLevel  logLevel)
{
    /* Modify the current log level to the argument passed */
    currLogLevel    =   logLevel;
    return;
}

/**
 *  @b Description
 *  @n
 *      Plugin logger function.
 *
 *  @param[in]  level
 *      Print level
 *  @param[in]  fmt
 *      Formatting string
 *  @param[in]  ap
 *      Argument list
 *
 *  @retval
 *      Not Applicable.
 */
void my_logMsg (NetfpProxy_LogLevel level, char* fmt, va_list ap)
{
    char        buffer[1024];

    /* Filter logs by log level */
    if (level < currLogLevel)
        return;

    if (logFd)
    {
        /* Build a buffer with the message passed according to format specified. */
        vsnprintf (buffer, sizeof(buffer)-1, fmt, ap);

        /* Log the message to the log file */
        fprintf (logFd, "%s", buffer);
        fflush (logFd);
    }
    else
    {
        /* No log file open. Log to console instead */
        vprintf (fmt, ap);
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      Plugin PMTU Update registered function:
 *
 *  @param[in]  srcIP
 *      Source IP associated with the PMTU update
 *  @param[in]  dstIP
 *      Destination IP associated with the PMTU update
 *  @param[in]  mtu
 *      New MTU received
 *
 *  @retval
 *      0   -   Pass the PMTU to the NETFP Domain
 *  @retval
 *      <0  -   Drop the PMTU update
 */
int32_t my_report_PMTU (Netfp_IPAddr* srcIP, Netfp_IPAddr* dstIP, uint32_t* mtu)
{
    /* Default plugin is always accepting all the PMTU updates */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Plugin logger cleanup.
 *
 *  @retval
 *      Not Applicable.
 */
int32_t my_logDeInit (void)
{
    /* Close the log file */
    if (logFd)
    {
        fclose (logFd);
    }
    logFd   =   NULL;

    /* Logger deinit  done */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Plugin's main loop. Polls for commands from user
 *      application over Unix IPC socket and executes them
 *      in Proxy's context.
 *
 *  @retval
 *      Not applicable
 */
#define IPC_POLL_INTVL_MSEC   1000
void my_ipc_poll (void)
{
#if 0
    NetfpProxy_msg      rxMsg;
    int32_t             msgLen      =   sizeof (NetfpProxy_msg);
    uint32_t            destAddrLen =   sizeof (struct sockaddr_un);
    fd_set              readFdList;
    struct timeval      timeout;
    int32_t             numReadFds = 0, retVal;
    struct sockaddr_un  fromAddr;
    NetfpProxy_CmdType  cmdType;

    /* Setup the receive socket that needs to be monitored */
    FD_ZERO (&readFdList);
    FD_SET (proxyIpcSockFd, &readFdList);
    numReadFds = proxyIpcSockFd + 1;

    /* Set the timeout to zero: Return immediately if there is no command message
     * from user application */
    timeout.tv_sec  =   0;
    timeout.tv_usec =   0;

    if (1)
    {
        /* Check if we have any messages pending on the IPC socket. If so,
         * send them over to NetFP Proxy daemon for processing. */
        if ((retVal = select (numReadFds, &readFdList, NULL, NULL, &timeout)) > 0)
        {
            if (FD_ISSET (proxyIpcSockFd, &readFdList))
            {
                if (recvfrom (proxyIpcSockFd,
                             (void *)&rxMsg,
                             msgLen,
                             0,
                             (struct sockaddr *)&fromAddr,
                             &destAddrLen) > 0)
                {
                    /* Got a command from user application. Parse it. */
                    switch (rxMsg.hdr.msgType)
                    {
                        case NETFP_PROXY_IPC_MSGTYPE_START_OFFLOAD_SP_REQ:
                        {
                            cmdType =   NETFP_PROXY_CMDTYPE_START_OFFLOAD_SP_REQ;

                            /* Send the "Start Offload" command to Proxy for execution */
                            NetfpProxy_executeCommand (cmdType, rxMsg.hdr.transId, &rxMsg.body.spReq, &fromAddr, destAddrLen);
                            break;
                        }
                        case NETFP_PROXY_IPC_MSGTYPE_STOP_OFFLOAD_SP_REQ:
                        {
                            cmdType =   NETFP_PROXY_CMDTYPE_STOP_OFFLOAD_SP_REQ;

                            /* Send the "Stop Offload" command to Proxy for execution */
                            NetfpProxy_executeCommand (cmdType, rxMsg.hdr.transId, &rxMsg.body.spReq, &fromAddr, destAddrLen);
                            break;
                        }
                        case NETFP_PROXY_IPC_MSGTYPE_IP_ROUTE_FLUSH_REQ:
                        {
                            cmdType =   NETFP_PROXY_CMDTYPE_IP_ROUTE_FLUSH_REQ;

                            /* Send the "IP Route Flush" command to Proxy for execution */
                            NetfpProxy_executeCommand (cmdType, rxMsg.hdr.transId, NULL, &fromAddr, destAddrLen);
                            break;
                        }
                        case NETFP_PROXY_IPC_MSGTYPE_ADD_INTERFACE_REQ:
                        {
                            cmdType =   NETFP_PROXY_CMDTYPE_ADD_INTERFACE_REQ;

                            /* Send the "interface add" command to Proxy for execution */
                            NetfpProxy_executeCommand (cmdType, rxMsg.hdr.transId, &rxMsg.body.ifaceAddReq, &fromAddr, destAddrLen);
                            break;
                        }
                        case NETFP_PROXY_IPC_MSGTYPE_DEL_INTERFACE_REQ:
                        {
                            cmdType =   NETFP_PROXY_CMDTYPE_DEL_INTERFACE_REQ;

                            /* Send the "interface del" command to Proxy for execution */
                            NetfpProxy_executeCommand (cmdType, rxMsg.hdr.transId, &rxMsg.body.ifaceDelReq, &fromAddr, destAddrLen);
                            break;
                        }
                        case NETFP_PROXY_IPC_MSGTYPE_CONFIGURE_L3_QOS_REQ:
                        {
                            cmdType =   NETFP_PROXY_CMDTYPE_CONFIGURE_L3_QOS_REQ;

                            /* Send the "configure QoS" command to Proxy for execution */
                            NetfpProxy_executeCommand (cmdType, rxMsg.hdr.transId, &rxMsg.body.configL3QosReq, &fromAddr, destAddrLen);
                            break;
                        }
                        case NETFP_PROXY_IPC_MSGTYPE_FLUSH_VLAN_PRIORITY_REQ:
                        {
                            cmdType =   NETFP_PROXY_CMDTYPE_FLUSH_VLAN_PRIORITY_REQ;

                            /* Send the "Flush VLAN Egress Priority Mapping" command to Proxy for execution */
                            NetfpProxy_executeCommand (cmdType, rxMsg.hdr.transId, &rxMsg.body.flushVlanPriorityReq, &fromAddr, destAddrLen);
                            break;
                        }
                        default:
                        {
                            NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "ERROR: Invalid command %d \n", rxMsg.hdr.msgType);
                        }
                    }
                }
                else
                {
                    NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "ERROR: Rx error?? \n");
                }
            }
        }
    }
#else
    NetfpProxy_msg      rxMsg;
    uint32_t            destAddrLen =   sizeof (struct sockaddr_un);
    struct sockaddr_un  fromAddr;

    /* Read the IPC message: */
    if (recvfrom (proxyIpcSockFd, (void *)&rxMsg, sizeof (NetfpProxy_msg), 0, (struct sockaddr *)&fromAddr, &destAddrLen) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to receive data from the Plugin IPC Socket [Error: %s]\n", strerror(errno));
        return;
    }

    /* Process the received message: */
    switch (rxMsg.hdr.msgType)
    {
        case NETFP_PROXY_IPC_MSGTYPE_START_OFFLOAD_SP_REQ:
        {
            /* Send the "Start Offload" command to Proxy for execution */
            NetfpProxy_executeCommand (NETFP_PROXY_CMDTYPE_START_OFFLOAD_SP_REQ, rxMsg.hdr.transId, &rxMsg.body.spReq, &fromAddr, destAddrLen);
            break;
        }
        case NETFP_PROXY_IPC_MSGTYPE_STOP_OFFLOAD_SP_REQ:
        {
            /* Send the "Stop Offload" command to Proxy for execution */
            NetfpProxy_executeCommand (NETFP_PROXY_CMDTYPE_STOP_OFFLOAD_SP_REQ, rxMsg.hdr.transId, &rxMsg.body.spReq, &fromAddr, destAddrLen);
            break;
        }
        case NETFP_PROXY_IPC_MSGTYPE_IP_ROUTE_FLUSH_REQ:
        {
            /* Send the "IP Route Flush" command to Proxy for execution */
            NetfpProxy_executeCommand (NETFP_PROXY_CMDTYPE_IP_ROUTE_FLUSH_REQ, rxMsg.hdr.transId, NULL, &fromAddr, destAddrLen);
            break;
        }
        case NETFP_PROXY_IPC_MSGTYPE_ADD_INTERFACE_REQ:
        {
            /* Send the "interface add" command to Proxy for execution */
            NetfpProxy_executeCommand (NETFP_PROXY_CMDTYPE_ADD_INTERFACE_REQ, rxMsg.hdr.transId, &rxMsg.body.ifaceAddReq, &fromAddr, destAddrLen);
            break;
        }
        case NETFP_PROXY_IPC_MSGTYPE_DEL_INTERFACE_REQ:
        {
            /* Send the "interface del" command to Proxy for execution */
            NetfpProxy_executeCommand (NETFP_PROXY_CMDTYPE_DEL_INTERFACE_REQ, rxMsg.hdr.transId, &rxMsg.body.ifaceDelReq, &fromAddr, destAddrLen);
            break;
        }
        case NETFP_PROXY_IPC_MSGTYPE_CONFIGURE_L3_QOS_REQ:
        {
            /* Send the "configure QoS" command to Proxy for execution */
            NetfpProxy_executeCommand (NETFP_PROXY_CMDTYPE_CONFIGURE_L3_QOS_REQ, rxMsg.hdr.transId, &rxMsg.body.configL3QosReq, &fromAddr, destAddrLen);
            break;
        }
        case NETFP_PROXY_IPC_MSGTYPE_FLUSH_VLAN_PRIORITY_REQ:
        {
            /* Send the "Flush VLAN Egress Priority Mapping" command to Proxy for execution */
            NetfpProxy_executeCommand (NETFP_PROXY_CMDTYPE_FLUSH_VLAN_PRIORITY_REQ, rxMsg.hdr.transId, &rxMsg.body.flushVlanPriorityReq, &fromAddr, destAddrLen);
            break;
        }
        case NETFP_PROXY_IPC_MSGTYPE_SETUP_PMTU_REQ:
        {
            /* Send the "Setup PMTU" command to Proxy for execution */
            NetfpProxy_executeCommand (NETFP_PROXY_CMDTYPE_SETUP_PMTU_REQ, rxMsg.hdr.transId, &rxMsg.body.setupPMTUReq, &fromAddr, destAddrLen);
            break;
        }
        default:
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Error: Invalid IPC message type %d received\n", rxMsg.hdr.msgType);
            break;
        }
    }
#endif
    return;
}

/**
 *  @b Description
 *  @n
 *      Plugin Rx handler. Called by the Proxy to report
 *      responses to any commands sent earlier by the plugin
 *      to Proxy.
 *
 *  @param[in]  cmd
 *      Command for which response is being reported
 *  @param[in]  cmdId
 *      Corresponding command Id for which response is being reported
 *  @param[in]  cmdData
 *      Corresponding command response
 *  @param[in]  toAddr
 *      Socket address to which response must be sent back to.
 *  @param[in]  toAddrLen
 *      Destination socket address length.
 *
 *  @retval
 *      Not applicable
 */
void my_report_cmd_response (NetfpProxy_CmdType cmd, NetfpProxy_CmdId cmdId, void* cmdData, void* toAddr, uint32_t toAddrLen)
{
    NetfpProxy_msg      rxMsg;
    struct sockaddr_un* toSunAddr = (struct sockaddr_un *)toAddr;

    /* Translate response back to user application */
    switch (cmd)
    {
        case NETFP_PROXY_CMDTYPE_START_OFFLOAD_SP_REQ:
        {
            rxMsg.hdr.msgType   =   NETFP_PROXY_IPC_MSGTYPE_START_OFFLOAD_SP_RESP;
            memcpy ((void*)&rxMsg.body.spResp, cmdData, sizeof (NetfpProxy_OffloadSpResp));
            break;
        }
        case NETFP_PROXY_CMDTYPE_STOP_OFFLOAD_SP_REQ:
        {
            rxMsg.hdr.msgType   =   NETFP_PROXY_IPC_MSGTYPE_STOP_OFFLOAD_SP_RESP;
            memcpy ((void*)&rxMsg.body.spResp, cmdData, sizeof (NetfpProxy_OffloadSpResp));
            break;
        }
        case NETFP_PROXY_CMDTYPE_IP_ROUTE_FLUSH_REQ:
        {
            rxMsg.hdr.msgType   =   NETFP_PROXY_IPC_MSGTYPE_IP_ROUTE_FLUSH_RESP;
            memcpy ((void*)&rxMsg.body.ipRouteFlushResp, cmdData, sizeof (NetfpProxy_IpRouteFlushResp));
            break;
        }
        case NETFP_PROXY_CMDTYPE_ADD_INTERFACE_REQ:
        {
            rxMsg.hdr.msgType   =   NETFP_PROXY_IPC_MSGTYPE_ADD_INTERFACE_RESP;
            memcpy ((void*)&rxMsg.body.ifaceAddResp, cmdData, sizeof (NetfpProxy_InterfaceAddResp));
            break;
        }
        case NETFP_PROXY_CMDTYPE_DEL_INTERFACE_REQ:
        {
            rxMsg.hdr.msgType   =   NETFP_PROXY_IPC_MSGTYPE_DEL_INTERFACE_RESP;
            memcpy ((void*)&rxMsg.body.ifaceDelResp, cmdData, sizeof (NetfpProxy_InterfaceDelResp));
            break;
        }
        case NETFP_PROXY_CMDTYPE_CONFIGURE_L3_QOS_REQ:
        {
            rxMsg.hdr.msgType   =   NETFP_PROXY_IPC_MSGTYPE_CONFIGURE_L3_QOS_RESP;
            memcpy ((void*)&rxMsg.body.configL3QosResp, cmdData, sizeof (NetfpProxy_ConfigureL3QosResp));
            break;
        }
        case NETFP_PROXY_CMDTYPE_FLUSH_VLAN_PRIORITY_REQ:
        {
            rxMsg.hdr.msgType   =   NETFP_PROXY_IPC_MSGTYPE_FLUSH_VLAN_PRIORITY_RESP;
            memcpy ((void*)&rxMsg.body.flushVlanPriorityResp, cmdData, sizeof (NetfpProxy_FlushVlanPriorityResp));
            break;
        }
        case NETFP_PROXY_CMDTYPE_SETUP_PMTU_REQ:
        {
            rxMsg.hdr.msgType   =   NETFP_PROXY_IPC_MSGTYPE_SETUP_PMTU_RESP;
            memcpy ((void*)&rxMsg.body.flushVlanPriorityResp, cmdData, sizeof (NetfpProxy_FlushVlanPriorityResp));
            break;
        }
        default:
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "ERROR: Unrecognized command %s\n", NetfpProxy_cmdType2Str(cmd));
            return;
        }
    }

    /* Copy over the transaction Id for the message */
    rxMsg.hdr.transId       =   cmdId;

    /* Send the command response to User application */
    if (sendto (proxyIpcSockFd,
                (void *)&rxMsg,
                sizeof (NetfpProxy_msg),
                0,
                (const struct sockaddr *)toAddr,
                toAddrLen) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "ERROR: Failed sending response for %s\n", NetfpProxy_cmdType2Str(cmd));
    }
    else
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "DEBUG: Sent response for %s trans_id: 0x%x to IPC socket on: %s\n",
                           NetfpProxy_cmdType2Str(cmd), cmdId, toSunAddr->sun_path);
    }

    return;
}

/**
 *  @b Description
 *  @n
 *      Plugin's Network event handler. Callback called by the
 *      Proxy daemon to report network events.
 *
 *  @param[in]  action
 *      Event
 *  @param[in]  eventType
 *      Corresponding command Id for which response is being reported
 *  @param[in]  eventData
 *      Event info
 *
 *  @retval
 *      Not applicable
 */
void my_report_net_event (int32_t action, NetfpProxy_NetEventType eventType, void* eventData)
{
    return;
}

/**
 *  @b Description
 *  @n
 *      Plugin exit function.
 *
 *  @retval
 *      Not applicable
 */
void my_exit (void)
{
    /* Close the IPC socket */
    close (proxyIpcSockFd);

    /* Close the log file */
    my_logDeInit ();
    return;
}

/**
 *  @b Description
 *  @n
 *      This is the plugin initialization function which is invoked by the NETFP Proxy
 *      once the plugin is loaded and initialized.
 *
 *  @param[in]  instanceId
 *      Netfp Proxy instance Id that this plugin is loaded to run with
 *
 *  @retval
 *      Success -   Descriptor used for communication with the core module
 *  @retval
 *      Error   -   <0
 */
int32_t NetfpProxy_pluginInit (int32_t instanceId)
{
    NetfpProxy_PluginCfg        pluginCfg;
    char                        sunPath[64];

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG,  "Debug: Registering plugin for Proxy PID %d \n", instanceId);

    /* Initialize the logger */
    if (my_logInit (instanceId) < 0)
        return -1;

    /* Log all errors and debug messages */
    my_logSetLogLevel (NETFP_PROXY_LOG_DEBUG);

    /* Open a Unix socket to talk to User application */
    proxyIpcSockFd = socket (AF_UNIX, SOCK_DGRAM, 0);
    if (proxyIpcSockFd < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR,  "Error: Socket open failed [Error: %s]\n", strerror(errno));
        return -1;
    }

    /* Setup a unique socket name for this Proxy/plugin instance.
     * We are using by default Proxy's PID as the unique instance Id. */
    snprintf (sunPath, sizeof(sunPath), "%s/%s_%d", Syslib_getRunTimeDirectory(), NETFP_PROXY_IPC_DEFAULT_DAEMON_PATH, instanceId);

    /* Setup the socket to listen on messages from User application. */
    memset ((void *)&proxySunAddr, 0, sizeof (struct sockaddr_un));
    strcpy (proxySunAddr.sun_path, sunPath);

    /* Unlink the socket path to ensure this is the only socket instance running */
    unlink (proxySunAddr.sun_path);

    /* Bind the socket to our unix path name */
    proxySunAddr.sun_family = AF_UNIX;
    if (bind (proxyIpcSockFd, (const struct sockaddr *)&proxySunAddr, sizeof (struct sockaddr_un)) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "ERROR: Socket bind failed, error: %d \n", errno);
        my_exit ();
        return -1;
    }
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG,  "DEBUG: Plugin for Proxy PID listening on %s \n", sunPath);

    /* Initialize the plugin configuration */
    memset ((void*)&pluginCfg, 0, sizeof (NetfpProxy_PluginCfg));

    /* Setup plugin callback functions */
    pluginCfg.report_cmd_response = my_report_cmd_response;
    pluginCfg.report_net_event    = my_report_net_event;
    pluginCfg.reportPMTU          = my_report_PMTU;
    pluginCfg.run                 = my_ipc_poll;
    pluginCfg.logMsg              = my_logMsg;
    pluginCfg.exit                = my_exit;

    /* Register the plugin with the PROXY. */
    if (NetfpProxy_registerPlugin (&pluginCfg) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Plugin registration failed\n");
        my_exit ();
    }
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Plugin registration succeeded\n");
    return proxyIpcSockFd;
}

