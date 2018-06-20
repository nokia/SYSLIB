/**
 *   @file  netfp_server.h
 *
 *   @brief
 *      NETFP Server internal header file
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2013-2104 Texas Instruments, Inc.
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
#ifndef __NETFP_SERVER_H__
#define __NETFP_SERVER_H__

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <getopt.h>

/* MCSDK Include files. */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/runtime/hplib/hplib.h>

/* SYSLIB Include Files */
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/netfp/netfp.h>
#include <ti/runtime/uintc/uintc.h>
#include <ti/runtime/name/name_db.h>
#include <ti/runtime/name/name_proxyClient.h>
#include <ti/runtime/resmgr/resmgr.h>

/**********************************************************************
 ******************* NETFP Server Data Structures *********************
 **********************************************************************/

/**
 * @brief
 *  Enumeration for logging levels
 *
 * @details
 *  Log messages are logged at the following levels.
 */
typedef enum NetfpServer_LogLevel
{
    /**
    * @brief  Debug messages
    */
    NetfpServer_LogLevel_VERBOSE, //fzm
    /**
     * @brief  Debug messages
     */
    NetfpServer_LogLevel_DEBUG,

    /**
    * @brief   Informational message
    */
    NetfpServer_LogLevel_INFO, //fzm

    /**
     * @brief  Fatal Error Messages
     */
    NetfpServer_LogLevel_ERROR
}NetfpServer_LogLevel;

/**
 * @brief
 *  Structure which describes Resource Manager Server MCB
 *
 * @details
 *  This is the master control block which holds all the relevant information
 *  for the resource manager server instance.
 */
typedef struct NetfpServer_MCB
{
    /**
     * @brief Server Name
     */
    char                    serverName[128];

    /**
     * @brief RM Client Name
     */
    char                    rmClientName[128];

    /**
     * @brief Named resource instance identifier.
     */
    uint32_t                nrInstanceId;

    /**
     * @brief Pointer to the logging file which is to be used to log all the NETFP Server
     * generated log messages
     */
    FILE*                   loggingFile;

    /**
     * @brief   Database handle used by the NETFP Server.
     */
    Name_DBHandle           databaseHandle;

    /**
     * @brief   PKTLIB Instance handle
     */
    Pktlib_InstHandle       pktlibInstanceHandle;

    /**
     * @brief   MSGCOM Instance populated and used by the NETFP server.
     */
    Msgcom_InstHandle       appMsgcomInstanceHandle;

    /**
     * @brief NETP Server Logging level.
     */
    NetfpServer_LogLevel    logLevel;

    /**
     * @brief NETP Server number of security channels
     */
    uint32_t                numSecurityChannels;

    /**
     * @brief Base Security Context Identifier
     */
    uint32_t                baseSecurityContextId;

    /**
     * @brief Polling Timeout in milliseconds.
     */
    uint32_t                pollingTimeout;

    /**
     * @brief Configuration which allows the addition of user stat counters to IP rules.
     */
    uint32_t                enableIPLutEntryCount;

    /**
     * @brief Name of the client list file
     */
    char*                   clientListFile;

    /**
     * @brief Name of all the NETFP clients which are parsed from the client list.
     */
    char                    netfpClientList[NETFP_MAX_CLIENTS][NETFP_MAX_CHAR];

    /**
     * @brief NETFP client status
     */
    uint32_t                netfpClientStatus[NETFP_MAX_CLIENTS];

    /**
     * @brief NETFP clients which have been passed to the server.
     */
    int32_t                 netfpClientCount;

    /**
     * @brief Server State
     */
    volatile uint32_t       serverState;

    /**
     * @brief Socket interface to the NETFP master
     */
    int32_t                 masterSockFd;

    /**
     * @brief PA Command Heap Handle: The heap is used to allocate packets which are used to
     * send commands to configured the NETCP subsystem.
     */
    Pktlib_HeapHandle       paCommandHeapHandle;

    /**
     * @brief UINTC Handle used to manage interrupts which are used by the NETFP Server
     */
    UintcHandle             uintcHandle;

    /**
     * @brief IPSEC Command Heap Handle: The heap is used to allocate packets which are
     * exchanged between the PA and SA (vice-versa) as packets traverse between the subsystem
     */
    Pktlib_HeapHandle       ipSecHeapHandle;

    /**
     * @brief PKTLIB Server heap handle which is used to receive messages from DSP
     * NETFP clients
     */
    Pktlib_HeapHandle       serverHeapHandle;

    /**
     * @brief Global System configuration handle
     */
    Resmgr_SysCfgHandle     handleSysCfg;

    /**
     * @brief NETFP Server Handle
     */
    Netfp_ServerHandle      netfpServerHandle;

    paQueueBounceConfig_t    paQueueBounceConfig; //fzm
}NetfpServer_MCB;

/********************************************************************************************
 * Internal exported API:
 ********************************************************************************************/
extern void NetfpServer_log (NetfpServer_LogLevel level, const char* fmt, ...) __attribute__ ((format (printf, 2, 3)));
// <fzm
extern void NetfpServer_dump (NetfpServer_LogLevel level, const char* fmt, ...) __attribute__ ((format (printf, 2, 3)));
// </fzm>

extern int32_t NetfpServer_initMasterMgmtInterface (NetfpServer_MCB* ptrNetfpServerMCB);
extern void* NetfpServer_masterMgmtInterfaceThread(void *arg);
extern int32_t NetfpServer_deinitMasterMgmtInterface(NetfpServer_MCB* ptrNetfpServerMCB);

#endif /* __NETFP_SERVER_H__ */
