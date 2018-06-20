/**
 *   @file  dat_server.h
 *
 *   @brief
 *      DAT Server internal header file
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


#ifndef __DAT_SERVER_H__
#define __DAT_SERVER_H__

#include <ti/runtime/dat/dat.h>

#ifdef __cplusplus
extern "C" {
#endif

/**********************************************************************
 ******************** DAT Server Local Definitions ********************
 **********************************************************************/
/**
 * @brief   Maximum number of clients which can be supported concurrently by the
 * DAT Server.
 */
#define DAT_MAX_CLIENTS                   16

/**********************************************************************
 ********************* DAT Server Data Structures *********************
 **********************************************************************/

/**
 * @brief
 *  Enumeration for logging levels
 *
 * @details
 *  Log messages are logged at the following levels.
 */
typedef enum DatServer_LogLevel
{
    /**
     * @brief  Debug messages
     */
    DatServer_LogLevel_DEBUG = 0x1,

    /**
     * @brief  Fatal Error Messages
     */
    DatServer_LogLevel_ERROR = 0x2
}DatServer_LogLevel;

/**
 * @brief
 *  Structure which describes DAT Server MCB
 *
 * @details
 *  This is the master control block which holds all the relevant information
 *  for the DAT server instance.
 */
typedef struct DatServer_MCB
{
    /**
     * @brief DAT Server Name
     */
    char                    serverName[128];

    /**
     * @brief RM Client Name
     */
    char                    rmClientName[128];

    /**
     * @brief DAT Server Log Level.
     */
    DatServer_LogLevel      logLevel;

    /**
     * @brief Name of the client list file
     */
    char*                   clientListFile;

    /**
     * @brief Polling Timeout in milliseconds.
     */
    uint32_t                pollingTimeout;

    /**
     * @brief   Named resource instance identifier for the DAT Server
     */
    uint32_t                nrInstanceId;

    /**
     * @brief   Database Handle mapped to the DAT Server which is used to store all the
     * information used by the DAT server.
     */
    Name_DBHandle           databaseHandle;

    /**
     * @brief   PKTLIB Instance handle
     */
    Pktlib_InstHandle       pktlibInstanceHandle;

    /**
     * @brief   MSGCOM Instance handle
     */
    Msgcom_InstHandle       msgcomInstanceHandle;

    /**
     * @brief UINTC Handle used to manage interrupts which are used by the NETFP Server
     */
    UintcHandle             uintcHandle;

    /**
     * @brief Server State
     */
    uint32_t                serverState;

    /**
     * @brief DAT Server Handle
     */
    Dat_ServerHandle        datServerHandle;

    /**
     * @brief DAT Server Heap Handle which is used to send/receive all
     * messages with the DAT clients
     */
    Pktlib_HeapHandle       datServerPktlibHeapHandle;

    /**
     * @brief Global System configuration handle
     */
    Resmgr_SysCfgHandle     handleSysCfg;

    /**
     * @brief Name of all the DAT clients which are parsed from the client list.
     */
    char                    datClientList[DAT_MAX_CLIENTS][DAT_MAX_CHAR];

    /**
     * @brief DAT client status
     */
    uint32_t                datClientStatus[DAT_MAX_CLIENTS];

    /**
     * @brief DAT clients which have been passed to the server.
     */
    int32_t                 datClientCount;
}DatServer_MCB;

/********************************************************************************************
 * Internal exported API:
 ********************************************************************************************/
extern void DatServer_log (DatServer_LogLevel level, char* fmt, ...);

#ifdef __cplusplus
}
#endif

#endif /* __DAT_SERVER_H__ */


