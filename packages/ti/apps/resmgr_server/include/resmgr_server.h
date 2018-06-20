/**
 *   @file  resmgr_server.h
 *
 *   @brief
 *      Internal header file for the SYSLIB RM server
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2012 Texas Instruments, Inc.
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
#ifndef __RESMGR_SERVER__H__
#define __RESMGR_SERVER__H__

/* Include Files. */
#include <ti/drv/rm/rm.h>
#include <ti/drv/rm/rm_services.h>
#include <ti/drv/rm/rm_transport.h>

#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/msgcom/msgcom.h>
#include <ti/runtime/uintc/uintc.h>
#include <ti/apps/resmgr_server/include/listlib.h>

/**********************************************************************
 ***************************** Definitions ****************************
 **********************************************************************/

/* Maximum Size of the RM Client request which can be received and processed
 * by the RM Server. */
#define RM_MAX_REQUEST_SIZE                 384

/* Maximum number of mailboxes supported by the RM server */
#define RM_MAX_MAILBOX                      32

/**********************************************************************
 *************** Resource Manager Server Data Structures **************
 **********************************************************************/

/**
 * @brief
 *  Mailbox status
 *
 * @details
 *  The enumeration describes the mailbox status
 */
typedef enum ResmgrServer_MailBoxStatus
{
    /**
     * @brief   Mailbox is free and available.
     */
    ResmgrServer_MailBoxStatus_FREE         = 0x0,

    /**
     * @brief   Mailbox has been allocated but the data has still not been populated
     */
    ResmgrServer_MailBoxStatus_ALLOCATED    = 0x1,

    /**
     * @brief   Mailbox holds an RM request
     */
    ResmgrServer_MailBoxStatus_REQUEST      = 0x2,

    /**
     * @brief   Mailbox holds an RM response
     */
    ResmgrServer_MailBoxStatus_RESPONSE     = 0x3
}ResmgrServer_MailBoxStatus;

/**
 * @brief
 *  Shared Memory Mailbox
 *
 * @details
 *  This data structure defines the mailbox which is required to communicate
 *  between the DSP and ARM through shared memory.
 */
typedef struct ResmgrServer_MailBox
{
    /**
     * @brief Status of the data.
     */
    ResmgrServer_MailBoxStatus  status;

    /**
     * @brief Padding to align the RM request and response
     */
    uint8_t                     padding1[12];

    /**
     * @brief RM Request
     */
    char                        request[RM_MAX_REQUEST_SIZE];

    /**
     * @brief RM Response
     */
    char                        response[RM_MAX_REQUEST_SIZE];

    /**
     * @brief Padding to align to cache.
     */
    uint8_t                     padding2[112];
}ResmgrServer_MailBox;

/**
 * @brief
 *  Enumeration for logging levels
 *
 * @details
 *  Log messages are logged at the following levels.
 */
typedef enum ResmgrServer_LogLevel
{
    /**
    * @brief  Verbose messages
    */
    ResmgrServer_LogLevel_VERBOSE, //fzm

    /**
    * @brief  Debug messages
    */
    ResmgrServer_LogLevel_DEBUG,

    /**
    * @brief  Info messages
    */
    ResmgrServer_LogLevel_INFO, //fzm

    /**
    * @brief  Fatal Error Messages
    */
    ResmgrServer_LogLevel_ERROR
}ResmgrServer_LogLevel;
/**
 * @brief
 *  Structure which describes Resource Manager Client
 *
 * @details
 *  The structure is used to store information about the RM clients
 *  which have communicated with the Resource Manager server.
 */
typedef struct ResmgrServer_Client
{
    /**
     * @brief Links to the other clients in the system
     */
    ResmgrServer_ListNode   links;

    /**
     * @brief RM Client name.
     */
    char                    clientName[128];

    /**
     * @brief Flag which indicates if the client is executing on ARM or DSP.
     */
    uint8_t                 isARMClient;

    /**
     * @brief RM Client transport handle.
     */
    Rm_TransportHandle      transportHandle;
}ResmgrServer_Client;

/**
 * @brief
 *  Structure which describes Resource Manager Server MCB
 *
 * @details
 *  This is the master control block which holds all the relevant information
 *  for the resource manager server instance.
 */
typedef struct ResmgrServer_MCB
{
    /**
     * @brief Server Name
     */
    char                    serverName[128];

    /**
     * @brief RM Server socket
     */
    int32_t                 rmServerSocket;

    /**
     * @brief Global resource file descriptor
     */
    int32_t                 globalResourceFileDescriptor;

    /**
     * @brief Server policy file descriptor
     */
    int32_t                 serverPolicyFileDescriptor;

    /**
     * @brief User space interrupt handle block
     */
    UintcHandle             uintcHandle;

    /**
     * @brief File descriptor which is used to handle the DSP interrupts.
     */
    int32_t                 dspInterruptFd;

    /**
     * @brief DSP cores communicate with the RMv2 server through IPC source identifiers.
     * This is configured source identifier.
     */
    int32_t                 dspSrcId;

    /**
     * @brief Pipe handler to enable the handling of signals.
     */
    int32_t                 signalPipe[2];

    /**
     * @brief RM Server Handle
     */
    ResmgrServer_LogLevel   logLevel;

    /**
     * @brief Physical Shared Memory Address.
     */
    uint32_t                sharedMemoryAddress;

    /**
     * @brief List of mailboxs which are available
     */
    ResmgrServer_MailBox*   ptrMailboxList;

    /**
     * @brief Mailbox which is being serviced.
     */
    ResmgrServer_MailBox*   ptrInServiceMailbox;

    /**
     * @brief RM Server Handle
     */
    Rm_Handle               rmServerHandle;

    /**
     * @brief  RM server service handle
     */
    Rm_Handle               rmServerServiceHandle;

    /**
     * @brief List of RM Clients attached to the server.
     */
    ResmgrServer_Client     ptrClientList;
}ResmgrServer_MCB;

/**********************************************************************
 ********************** External Definitions **************************
 **********************************************************************/

/* Global Resource Manager Server Master Control block*/
extern ResmgrServer_MCB    gResmgrServerMCB;

/* Exported API: */
extern void ResmgrServer_log (ResmgrServer_LogLevel level, const char* fmt, ...) __attribute__ ((format (printf, 2, 3)));
extern void ResmgrServer_dump(ResmgrServer_LogLevel level, const char* fmt, ...) __attribute__ ((format (printf, 2, 3))); //fzm
extern int32_t ResmgrServer_systemInit (Rm_Handle rmServerHandle);

#endif /* __RESMGR_SERVER__H__ */




