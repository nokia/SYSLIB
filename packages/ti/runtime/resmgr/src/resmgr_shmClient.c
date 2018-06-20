/**
 *   @file  resmgr_shmClient.c
 *
 *   @brief
 *      The file implements the RMv2 client transport interface. The
 *      implementation is done using shared memory which allows DSP
 *      cores (executing the RMv2 client) to communicate with the
 *      SYSLIB-RM Server
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

/* PDK Include Files. */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/drv/rm/rm.h>
#include <ti/drv/rm/rm_transport.h>
#include <ti/drv/rm/rm_services.h>
#include <ti/csl/csl_chipAux.h>
#include <ti/csl/csl_bootcfgAux.h>

/* Resource Manager Include Files. */
#include <ti/runtime/resmgr/resmgr.h>

/* For Debugging only. */
#ifdef __ARMv7
#include <ti/apps/netfp_config/include/NetFP_System_printf.h> //fzm
#else
#include <xdc/runtime/System.h>
#endif

/**********************************************************************
 ************************** Local Definitions *************************
 **********************************************************************/

/* Maximum Size of the RM Client request which can be received and processed
 * by the RM Server. */
#define RM_MAX_REQUEST_SIZE                 384

/* Maximum number of mailboxes which is supported by the RM client. */
#define RM_CLIENT_MAX_MAILBOX               2

/**********************************************************************
 ************************** Local Structures **************************
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
 *  Shared Memory Transport Internal data structure
 *
 * @details
 *  This data structure defines the shared memory transport MCB which
 *  holds all the relevant information with respect to the shared
 *  memory transport instance.
 */
typedef struct ResmgrServer_SharedMemMCB
{
    /**
     * @brief Mailbox list associated with the transport
     */
    ResmgrServer_MailBox*   ptrMailboxList;

    /**
     * @brief  RM transport handle
     */
    Rm_TransportHandle      transportHandle;

    /**
     * @brief  System configuration
     */
    Resmgr_SystemCfg        cfg;
}ResmgrServer_SharedMemMCB;

uint32_t gPankajDebugCounter = 0;

/**********************************************************************
 ***************** Resource Management Client Functions ***************
 **********************************************************************/

/**
 *  @b Description
 *  @n
 *      Internal BUG workaround for the CSL Bootcfg function which is used
 *      to generate an interrupt to ARM.
 *
 *      This function should be *obsoleted* as soon as the CSL bug is fixed.
 *
 *  @param[in]  coreId
 *      ARM core identifier
 *  @param[in]  srcId
 *      Source Identifier
 *
 *  \ingroup RES_MGR_DSP_INTERNAL_FUN
 *
 *  @retval
 *      Not applicable
 */
static inline void CSL_BugFixBootCfgGenerateIPCInterrupt (uint32_t coreId, uint32_t srcId)
{
    /* Increment the counter. */
    gPankajDebugCounter++;

    /* Generate the interrupt. */
    hBootCfg->IPCGR[coreId] = hBootCfg->IPCGR[coreId] | CSL_FMKR (srcId + 4, srcId + 4, 1) |  CSL_FMK(BOOTCFG_IPCGR0_IPCGR0_REG, 1);
    return;
}

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
 *  \ingroup RES_MGR_DSP_INTERNAL_FUN
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
    Rm_Packet*                  ptrRmPacket;
    ResmgrServer_MailBox*       ptrMailbox;
    uint32_t                    mailboxIndex;
    ResmgrServer_SharedMemMCB*  ptrSharedMemMCB;

    /* Get the shared memory MCB */
    ptrSharedMemMCB = (ResmgrServer_SharedMemMCB*)appTransport;

    /* Get the mailbox information */
    ptrMailbox = ptrSharedMemMCB->ptrMailboxList;

    /* Cycle through and determine which mailbox is free */
    for (mailboxIndex = 0; mailboxIndex < RM_CLIENT_MAX_MAILBOX; mailboxIndex++)
    {
        /* Determine the status of the mailbox */
        if (ptrMailbox->status == ResmgrServer_MailBoxStatus_FREE)
        {
            /* Mailbox was free; so we mark it now as allocated. This will ensure that
             * the request is NOT allocated again. */
            ptrMailbox->status = ResmgrServer_MailBoxStatus_ALLOCATED;
            break;
        }

        /* Next mailbox */
        ptrMailbox++;
    }

    /* Did we get a mailbox? */
    if (mailboxIndex == RM_CLIENT_MAX_MAILBOX)
    {
        System_printf ("Error: Unable to allocate shared memory for the RMv2 transport\n");
        return NULL;
    }

    /* Initialize the RM packet with the request */
    ptrRmPacket = (Rm_Packet*)&ptrMailbox->request[0];
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
 *  \ingroup RES_MGR_DSP_INTERNAL_FUN
 *
 *  @retval
 *      Success   - 0
 *  @retval
 *      Error     - <0
 */
static int32_t Resmgr_transportSendRcv (Rm_AppTransportHandle appTransport, Rm_PacketHandle pktHandle)
{
    ResmgrServer_MailBox*       ptrMailbox;
    int32_t                     rmResult;
    ResmgrServer_SharedMemMCB*  ptrSharedMemMCB;
    uint32_t                    mailboxIndex;

    /* Get the shared memory MCB */
    ptrSharedMemMCB = (ResmgrServer_SharedMemMCB*)appTransport;

    /* Get the mailbox information */
    ptrMailbox = ptrSharedMemMCB->ptrMailboxList;

    /* We need to determine which mailbox is to be used. */
    for (mailboxIndex = 0; mailboxIndex < RM_CLIENT_MAX_MAILBOX; mailboxIndex++)
    {
        /* Do we have a match? */
        if (pktHandle == (Rm_PacketHandle)&ptrMailbox->request[0])
            break;

        /* Next mailbox */
        ptrMailbox++;
    }

    /* Did we get a match? */
    if (mailboxIndex == RM_CLIENT_MAX_MAILBOX)
    {
        System_printf ("Error: Unable to match RM Packet %p to a mailbox\n", pktHandle);
        return -1;
    }

    /* Send the request to the RM Server and writeback the cache contents. */
    ptrMailbox->status = ResmgrServer_MailBoxStatus_REQUEST;
    ptrSharedMemMCB->cfg.endMemAccess (ptrMailbox, sizeof(ResmgrServer_MailBox));

    /* Generating an interrupt to ARM core */
    CSL_BugFixBootCfgGenerateIPCInterrupt (ptrSharedMemMCB->cfg.dspSystemCfg.armCoreId, ptrSharedMemMCB->cfg.dspSystemCfg.sourceId);

    /* We need to loop around till the response for the mailbox is received. */
    while (1)
    {
        /* Invalidate the cache contents */
        ptrSharedMemMCB->cfg.beginMemAccess (ptrMailbox, sizeof(ResmgrServer_MailBox));

        /* Did we get the response? */
        if (ptrMailbox->status == ResmgrServer_MailBoxStatus_RESPONSE)
            break;
    }

    /* YES. Response has been received. Process it. */
    rmResult = Rm_receivePacket(ptrSharedMemMCB->transportHandle, (Rm_Packet*)&ptrMailbox->response[0]);
    if (rmResult != RM_OK)
    {
        System_printf ("Error: RM failed to process received packet: %d\n", rmResult);
        return -1;
    }

    /* Cleanup the mailbox */
    ptrMailbox->status = ResmgrServer_MailBoxStatus_FREE;
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to setup the RM shared memory client transport
 *
 *  @param[in]  ptrSystemCfg
 *      System Configuration
 *  @param[in]  rmClientHandle
 *      RMv2 Client Handle
 *  @param[out] errCode
 *      Error code populated by the API
 *
 *  \ingroup RES_MGR_DSP_INTERNAL_FUN
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
    ResmgrServer_SharedMemMCB*  ptrSharedMemMCB;
    Rm_TransportCfg             transportCfg;
    int32_t                     result;
    uint32_t                    mailboxSize;
    uint32_t                    mailboxOffset;

    /* Get the mailbox size & offset requirements for the client. */
    mailboxSize   = (RM_CLIENT_MAX_MAILBOX * sizeof(ResmgrServer_MailBox));
    mailboxOffset = DNUM * mailboxSize;

    /* Ensure that the mailbox size does not exceed the system specified size */
    if (mailboxSize > ptrSystemCfg->dspSystemCfg.sizeSharedMemory)
    {
        *errCode = RESMGR_ENOMEM;
        return NULL;
    }

    /* Allocate memory for the shared memory transport MCB */
    ptrSharedMemMCB = (ResmgrServer_SharedMemMCB*)ptrSystemCfg->malloc (Resmgr_MallocMode_LOCAL, sizeof(ResmgrServer_SharedMemMCB));
    if (ptrSharedMemMCB == NULL)
    {
        *errCode = RESMGR_ENOMEM;
        return NULL;
    }

    /* Initialize the allocated memory */
    memset ((void *)ptrSharedMemMCB, 0, sizeof(ResmgrServer_SharedMemMCB));

    /* Copy over the DSP system configuration. */
    memcpy ((void *)&ptrSharedMemMCB->cfg, (void*)ptrSystemCfg, sizeof(Resmgr_SystemCfg));

    /* Initialize the transport configuration */
    memset ((void *)&transportCfg, 0, sizeof(Rm_TransportCfg));

    /* Populate the transport configuration */
    transportCfg.rmHandle                     = rmClientHandle;
    transportCfg.appTransportHandle           = (Rm_AppTransportHandle)ptrSharedMemMCB;
    transportCfg.remoteInstType               = Rm_instType_SERVER;
    transportCfg.transportCallouts.rmAllocPkt = Resmgr_transportAlloc;
    transportCfg.transportCallouts.rmSendPkt  = Resmgr_transportSendRcv;

    /* Register the transport with the resource manager */
    ptrSharedMemMCB->transportHandle = Rm_transportRegister(&transportCfg, &result);
    if (result != RM_OK)
    {
        *errCode = result;
        return NULL;
    }

    /* Each core has its own mailbox which is used for all communication with the RMv2 server. All memory
     * allocations are done only by the RMv2 client. The request and response buffers are bunched together
     * Since there is only 1 master responisble for the allocation and cleanup we can protect the mailbox
     * with single core critical sections. */
    ptrSharedMemMCB->ptrMailboxList = (ResmgrServer_MailBox*)(ptrSystemCfg->dspSystemCfg.sharedMemAddress + mailboxOffset);

    /* Initialize the shared memory mailbox */
    memset ((void*)ptrSharedMemMCB->ptrMailboxList, 0, mailboxSize);

    /* Writeback the cache contents. */
    ptrSystemCfg->endMemAccess (ptrSharedMemMCB->ptrMailboxList, mailboxSize);

    /* Debug Message: */
    System_printf ("Debug: Mailbox list address 0x%x Size %d\n", ptrSharedMemMCB->ptrMailboxList, mailboxSize);
    return (void*)ptrSharedMemMCB;
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
 *  \ingroup RES_MGR_DSP_INTERNAL_FUN
 *
 *  @retval
 *      Success   - 0
 *  @retval
 *      Error     - <0
 */
int32_t Resmgr_deleteClientTransport (void* clientTransportHandle, int32_t* errCode)
{
    ResmgrServer_SharedMemMCB*  ptrSharedMemMCB;

    /* Get the client transport handle. */
    ptrSharedMemMCB = (ResmgrServer_SharedMemMCB*)clientTransportHandle;
    if (ptrSharedMemMCB == NULL)
    {
        *errCode = RESMGR_EINVAL;
        return -1;
    }

    /* Deregister the transport */
    *errCode = Rm_transportUnregister (ptrSharedMemMCB->transportHandle);
    if (*errCode !=  RM_OK)
        return -1;

    /* Cleanup the memory allocated for the socket transport */
    ptrSharedMemMCB->cfg.free (Resmgr_MallocMode_LOCAL, ptrSharedMemMCB, sizeof(ResmgrServer_SharedMemMCB));
    return 0;
}

