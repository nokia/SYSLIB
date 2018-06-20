/**
 *   @file  netfp_sa.c
 *
 *   @brief
 *      The file implements the interface between the NETFP & SA module
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
#include <stdlib.h>
#include <string.h>

/* PDK & CSL Include Files */
#include <ti/drv/rm/rm.h>
#include <ti/csl/csl_cache.h>
#include <ti/csl/csl_tsc.h>

#if (defined(DEVICE_K2H) || defined(DEVICE_K2K))
#include <ti/drv/sa/fw/v0/saphp1_bin.c>
#include <ti/drv/sa/fw/v0/saphp2_bin.c>
#elif defined (DEVICE_K2L)
#include <ti/drv/sa/fw/v1/saphp1_bin.c>
#include <ti/drv/sa/fw/v1/saphp2_bin.c>
#include <ti/drv/sa/fw/v1/saphp3_bin.c>
#else
#error "Error: Unsupported device in the NETFP"
#endif

/* SYSLIB Include Files */
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/josh/josh.h>
#include <ti/runtime/msgcom/msgcom.h>
#include <ti/runtime/netfp/netfp.h>
#include <ti/runtime/netfp/include/netfp_net.h>
#include <ti/runtime/netfp/include/netfp_internal.h>

/* For Debugging only. */
#ifdef __ARMv7
#include <ti/apps/netfp_config/include/NetFP_System_printf.h> //fzm
#else
#include <xdc/runtime/System.h>
#endif

#include <ddal/ddal_cpu.h>

/* Define this flag to halt the SA on any error. */
#undef SA_DEBUG

/**************************************************************************
 ************************** Local Functions *******************************
 **************************************************************************/

static void Netfp_saDebugInfo (void* mID, uint16_t msgType,uint16_t msgCode,uint16_t msgLen,uint16_t* supData);
static void Netfp_saChanKeyRequest (Sa_ChanHandle handle, Sa_KeyRequest_t* keyReq);
static void Netfp_secContextAlloc (Sa_ChanHandle handle, Sa_ScReqInfo_t* scReqInfo);
static void Netfp_secContextFree (Sa_ChanHandle handle, uint16_t scID);
static void Netfp_secChannelReg (Sa_ChanHandle handle, Sa_SWInfo_t* pSwInfo);
static void Netfp_secChannelUnreg (Sa_ChanHandle handle, Sa_SWInfo_t* pSwInfo);
static void Netfp_secChannelSendNullPkt (Sa_ChanHandle handle, Sa_PktInfo_t *pktInfo);

/**************************************************************************
 ************************** Global Variables ******************************
 **************************************************************************/

/**
 * @brief   Global SALLD call-out function table
 */
Sa_CallOutFuncs_t saMgmt_calloutFunc =
{
    Netfp_saDebugInfo,          /* Debug function pointer                       */
    Netfp_saChanKeyRequest,     /* Key Request function Pointer                 */
    Netfp_secContextAlloc,      /* Security Context Allocation function pointer */
    Netfp_secContextFree,       /* Security Context Free Function pointer       */
    Netfp_secChannelReg,        /* Channel Registration Function pointer        */
    Netfp_secChannelUnreg,      /* Channel Unregister Function pointer          */
    Netfp_secChannelSendNullPkt /* Channel Send Null Packet function pointer    */
};

/**************************************************************************
 ***************************** NETFP SA Functions *************************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      The function is used to download the SA PDSP.
 *
 *  @param[in]  saHandle
 *      Handle to the SA LLD
 *  @param[out] errCode
 *      Error code populated on error
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Netfp_downloadSAPDSP(Sa_Handle saHandle, int32_t* errCode)
{
    /* Reset the SA Subsystem. */
    Sa_resetControl (saHandle, sa_STATE_RESET);

#if (defined(DEVICE_K2H) || defined(DEVICE_K2K))
    /* Download the SA Images (PDSP0) */
    *errCode = Sa_downloadImage (saHandle, 0, (Ptr)Sa_php1, Sa_php1Size);
    if (*errCode != sa_ERR_OK)
        return -1;

    /* Download the SA Images (PDSP1) */
    *errCode = Sa_downloadImage (saHandle, 1, (Ptr)Sa_php2, Sa_php2Size);
    if (*errCode != sa_ERR_OK)
        return -1;
#elif defined (DEVICE_K2L)
    /* Download the SA Images (PDSP0) */
    *errCode = Sa_downloadImage (saHandle, 0, (Ptr)Sa2_php1, Sa2_php1Size);
    if (*errCode != sa_ERR_OK)
        return -1;

    /* Download the SA Images (PDSP1) */
    *errCode = Sa_downloadImage (saHandle, 1, (Ptr)Sa2_php2, Sa2_php2Size);
    if (*errCode != sa_ERR_OK)
        return -1;

    /* Download the SA Images (PDSP2) */
    *errCode = Sa_downloadImage (saHandle, 2, (Ptr)Sa2_php3, Sa2_php3Size);
    if (*errCode != sa_ERR_OK)
        return -1;
#else
#error "Unsupported Device"
#endif

    /* Enable the SA susbystem. */
    if(Sa_resetControl (saHandle, sa_STATE_ENABLE) != sa_STATE_ENABLE)
    {
        *errCode = NETFP_ESA;
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is the used to allocate a security context identifier
 *      from the server managed list.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   Security Context identifer allocated
 *  @retval
 *      Error   -   <0
 */
static int32_t Netfp_allocateSecurityContextId (Netfp_ServerMCB* ptrNetfpServer)
{
    static uint8_t index = 0;
    uint8_t startIndex = index;
    static uint32_t databaseIndex = 0;
    uint32_t i;
    uint32_t contextId;

    /* Cycle through all the entries in the security context database */
    for (i = 0; i < ptrNetfpServer->cfg.maxSecurityChannels/32; i++)
    {
        /* Is there a free entry in this database entry? */
        if (ptrNetfpServer->ptrSecContextIdDatabase[databaseIndex] != 0xFFFFFFFF)
        {
            /* YES. Free entry detected; now we need to determine which index can be used */
            contextId = ptrNetfpServer->ptrSecContextIdDatabase[databaseIndex];

            contextId = contextId >> startIndex;

            /* Allocate a security context */
            for (index = startIndex; index < 32; index++)
            {
                /* Is the context available? */
                if (contextId & 0x1)
                {
                    /* Nope: Context is not available; skip to the next one */
                    contextId = contextId >> 1;
                }
                else
                {
                    /* Context was available. Mark it as taken. Remember the actual security context
                     * identifier is computed by adding the base number */
                    ptrNetfpServer->ptrSecContextIdDatabase[databaseIndex] = ptrNetfpServer->ptrSecContextIdDatabase[databaseIndex] | (1 << index);
                    contextId = ptrNetfpServer->cfg.baseSecurityContextId + (databaseIndex*32) + index;
                    index++;
                    if (index == 32)
                    {
                        index = 0;
                        databaseIndex++;
                        if (databaseIndex == ptrNetfpServer->cfg.maxSecurityChannels/32)
                        {
                            databaseIndex = 0;
                        }
                    }
                    return (contextId);
                }
            }
        }
        databaseIndex++;
        if (databaseIndex == ptrNetfpServer->cfg.maxSecurityChannels/32)
        {
            databaseIndex = 0;
        }
        startIndex = 0;
    }
    /* Control comes here implies that there was no security context identifier available. */
    return -1;
}

/**
 *  @b Description
 *  @n
 *      The function is the used to allocate a security context identifier
 *      from the server managed list.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  secContextId
 *      Security Context identifier to be cleaned up
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
static void Netfp_freeSecurityContextId (Netfp_ServerMCB* ptrNetfpServer, int32_t secContextId)
{
    uint32_t    index;
    uint32_t    databaseIndex;

    /* Remove the base security context identifier offset. */
    secContextId = secContextId - ptrNetfpServer->cfg.baseSecurityContextId;

    /* Get the database index associated with the security context identifier */
    databaseIndex = secContextId / 32;
    index         = secContextId % 32;

    /* Mark the entry as free. */
    ptrNetfpServer->ptrSecContextIdDatabase[databaseIndex] = ptrNetfpServer->ptrSecContextIdDatabase[databaseIndex] & ~(1 << index);
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is the implementation of the DebugTrace callback function
 *      which is registered with the SA instance.
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not Applicable.
 */
static void Netfp_saDebugInfo
(
    void*       mID,
    uint16_t    msgType,
    uint16_t    messageCode,
    uint16_t    msgLength,
    uint16_t*   supportingData
)
{
    if (msgType == sa_DBG_INFORMATIONAL)
    {
        System_printf ("SA Debug Information: Message Code: 0x%x Message Length 0x%x Supporting Data %p\n",
                       messageCode, msgLength, supportingData);
    }
    else if (msgType == sa_DBG_FATAL_ERROR)
    {
        System_printf ("SA FATAL Error: Message Code: 0x%x Message Length 0x%x Supporting Data %p\n",
                       messageCode, msgLength, supportingData);
    }
    else
    {
        System_printf ("SA Warning: Message Code: 0x%x Message Length 0x%x Supporting Data %p\n",
                       messageCode, msgLength, supportingData);
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is called by the SA driver to request for a new security key. This is
 *      not implemented in the application.
 *
 *  @param[in]   handle
 *      SALLD channel instance identifier.
 *  @param[in]   keyReq
 *      Pointer to SALLD key Request structure.
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not Applicable.
 */
static void Netfp_saChanKeyRequest (Sa_ChanHandle handle, Sa_KeyRequest_t *keyReq)
{
    System_printf ("Error: SA Channel Key Request is NOT implemented.\n");
    return;
}

/**
 *  @b Description
 *  @n
 *      The function creates a copy of the the specific security context and adds this
 *      to the garbage list.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]   ptrSecurityContext
 *      Pointer to the security context to be added to the garbage list
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not Applicable
 */
static void Netfp_addGarbageSecContext
(
    Netfp_ServerMCB*        ptrNetfpServer,
    Netfp_SecurityContext*  ptrSecurityContext
)
{
    Netfp_SecurityContext*  ptrGarbageSecurityContext;

    /* Allocate memory for the garbage security context: */
    ptrGarbageSecurityContext = (Netfp_SecurityContext*)ptrNetfpServer->cfg.malloc (sizeof(Netfp_SecurityContext), 0);
    if (ptrGarbageSecurityContext == NULL)
    {
        Netfp_logMsg (ptrNetfpServer, Netfp_LogLevel_ERROR, "Error: Garbage Security Context allocation failed\n");
        return;
    }

    /* Copy over the security context: */
    memcpy ((void *)ptrGarbageSecurityContext, ptrSecurityContext, sizeof(Netfp_SecurityContext));

    /* Add this to the garbage list: */
    Netfp_listAdd ((Netfp_ListNode**)&ptrNetfpServer->ptrGarbageSecurityContextList, (Netfp_ListNode*)ptrGarbageSecurityContext);
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to execute the garbage collection on the Security context list.
 *      The function will limit the number of entries which are cleaned up through the 'maxEntries'
 *      parameter.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  maxEntries
 *      Maximum number of entries to be garbage collected
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Number of entries garbage collected
 */
static int32_t Netfp_executeSecContextGarbageCollection
(
    Netfp_ServerMCB*        ptrNetfpServer,
    int32_t                 maxEntries
)
{
    Netfp_SecurityContext*  ptrGarbageSecurityContext;
    Netfp_SecurityContext*  ptrGarbageSecurityContextNext;
    uint32_t                counter = 0;

    /* Cycle through all the entries in the garbage collection */
    ptrGarbageSecurityContext = (Netfp_SecurityContext*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrGarbageSecurityContextList);
    while (ptrGarbageSecurityContext != NULL)
    {
        ptrGarbageSecurityContextNext = (Netfp_SecurityContext*)Netfp_listGetNext ((Netfp_ListNode*)ptrGarbageSecurityContext);
        /* Is the security context free? */
        if (Sa_isScBufFree (ptrGarbageSecurityContext->ptrSecurityContext) == 0)
        {
            /* NO: Skip the entry. */
            ptrGarbageSecurityContext = ptrGarbageSecurityContextNext;
            continue;
        }

        /* Free the security context immediately. */
        ptrNetfpServer->cfg.freeSecurityContext (ptrGarbageSecurityContext->netfpProtocol,
                                                 ptrGarbageSecurityContext->ptrSecurityContext,
                                                 ptrGarbageSecurityContext->sizeSecurityContext);

        /* Cleanup the security context identifier. */
        Netfp_freeSecurityContextId (ptrNetfpServer, ptrGarbageSecurityContext->secContextId);

        /* Cleanup the garbage security context: */
        Netfp_listRemoveNode ((Netfp_ListNode**)&ptrNetfpServer->ptrGarbageSecurityContextList, (Netfp_ListNode*)ptrGarbageSecurityContext);
        ptrNetfpServer->cfg.free (ptrGarbageSecurityContext, sizeof(Netfp_SecurityContext));

        /* Increment the counter: */
        counter++;

        /* Have we done what we were allowed to do? */
        if (counter == maxEntries)
            break;

        /* Restart the search */
        ptrGarbageSecurityContext = ptrGarbageSecurityContextNext;
    }
    return counter;
}

/**
 *  @b Description
 *  @n
 *
 *  @param[in]   handle
 *      SALLD channel instance identifier.
 *  @param[out]   scReqInfo
 *      Pointer to SALLD security context control information which is populated
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not Applicable
 */
static void Netfp_secContextAlloc (Sa_ChanHandle handle, Sa_ScReqInfo_t* scReqInfo)
{
    Netfp_SrvSecurityChannel*   ptrSrvSecurityChannel;
    int32_t                     secContextId;

    /* Get the security channel information: */
    ptrSrvSecurityChannel = (Netfp_SrvSecurityChannel*)Sa_chanGetID(handle);

    Netfp_executeSlowSecContextCleanup(ptrSrvSecurityChannel->ptrNetfpServer);

    /* Execute the security context garbage collection: */
    Netfp_executeSecContextGarbageCollection (ptrSrvSecurityChannel->ptrNetfpServer, 2);

    /* Allocate a security context identifier from the server. */
    secContextId = Netfp_allocateSecurityContextId (ptrSrvSecurityChannel->ptrNetfpServer);
    if (secContextId < 0)
    {
        /* No. Unable to allocate a security context from the NETFP server.*/
        scReqInfo->scSize = 0;
        scReqInfo->scID   = 0;

// fzm: if secContextId is -1 then nothing was allocated, attempting to free it would cause a buffer overflow
//        Netfp_freeSecurityContextId (ptrSrvSecurityChannel->ptrNetfpServer, secContextId);
        /* Error Message: */
        Netfp_logMsg (ptrSrvSecurityChannel->ptrNetfpServer, Netfp_LogLevel_ERROR,
                      "Error: Security Context id allocation failed\n");
        return;
    }

    /* Populate the security context identifier */
    scReqInfo->scID = secContextId;

    /* Security context buffers are shared between the software & the SA PDSP and so they need
     * to be cache aligned. Here we ensure that the requested size is a multiple of the cache size */
    if ((scReqInfo->scSize % CACHE_L2_LINESIZE) != 0)
        scReqInfo->scSize = ((scReqInfo->scSize / CACHE_L2_LINESIZE) + 1) * CACHE_L2_LINESIZE;

    /* Allocate memory for the security context: This is done through another exported OSAL function
     * rather than overriding the generic OSAL malloc function since application would deem to place
     * security context into preffered memory areas. */
    scReqInfo->scBuf = ptrSrvSecurityChannel->ptrNetfpServer->cfg.mallocSecurityContext (ptrSrvSecurityChannel->netfpProtocol,
                                                                                         scReqInfo->scSize,
                                                                                         CACHE_L2_LINESIZE);
    if (scReqInfo->scBuf == NULL)
    {
        /* No. We cannot satisfy the request. */
        scReqInfo->scSize = 0;
        scReqInfo->scID   = 0;

        /* Error Message: */
        Netfp_logMsg (ptrSrvSecurityChannel->ptrNetfpServer, Netfp_LogLevel_ERROR,
                      "Error: Security Context allocation [Id: %d] failed\n",
                      secContextId);
        return;
    }

    /* Record the security context: Use the security context which is free. */
    if (ptrSrvSecurityChannel->secContext[0].ptrSecurityContext == NULL)
    {
        ptrSrvSecurityChannel->secContext[0].netfpProtocol       = ptrSrvSecurityChannel->netfpProtocol;
        ptrSrvSecurityChannel->secContext[0].secContextId        = scReqInfo->scID;
        ptrSrvSecurityChannel->secContext[0].ptrSecurityContext  = scReqInfo->scBuf;
        ptrSrvSecurityChannel->secContext[0].sizeSecurityContext = scReqInfo->scSize;
    }
    else if (ptrSrvSecurityChannel->secContext[1].ptrSecurityContext == NULL)
    {
        ptrSrvSecurityChannel->secContext[1].netfpProtocol       = ptrSrvSecurityChannel->netfpProtocol;
        ptrSrvSecurityChannel->secContext[1].secContextId        = scReqInfo->scID;
        ptrSrvSecurityChannel->secContext[1].ptrSecurityContext  = scReqInfo->scBuf;
        ptrSrvSecurityChannel->secContext[1].sizeSecurityContext = scReqInfo->scSize;
    }
    else
    {
        Netfp_logMsg (ptrSrvSecurityChannel->ptrNetfpServer, Netfp_LogLevel_ERROR,
                      "Error: Security channel %x Security context 0 [%x] and 1 [%x] are full\n",
                      ptrSrvSecurityChannel, ptrSrvSecurityChannel->secContext[0].ptrSecurityContext,
                      ptrSrvSecurityChannel->secContext[1].ptrSecurityContext);
    }

    /* Initialize the security context */
    memset ((void *)scReqInfo->scBuf, 0, scReqInfo->scSize);
    return;
}

/**
 *  @b Description
 *  @n
 *      The function releases the security context of the specified id.
 *
 *  @param[in]   handle
 *      SALLD channel instance identifier.
 *  @param[in]   scId
 *      Security context identifier to be cleaned up
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not Applicable
 */
static void Netfp_secContextFree (Sa_ChanHandle handle, uint16_t scId)
{
    Netfp_SrvSecurityChannel*   ptrSrvSecurityChannel;

    /* Get the security channel information: */
    ptrSrvSecurityChannel = (Netfp_SrvSecurityChannel*)Sa_chanGetID(handle);

    /* Determine which security context is being deleted. We also use the address of the
     * security context just to be sure that for the security context 0 there is no confusion */
    if ((ptrSrvSecurityChannel->secContext[0].secContextId == scId) &&
        (ptrSrvSecurityChannel->secContext[0].ptrSecurityContext != NULL))
    {
        /* Add the security context to the garbage collection: */
        Netfp_addGarbageSecContext (ptrSrvSecurityChannel->ptrNetfpServer, &ptrSrvSecurityChannel->secContext[0]);

        /* Reset the fields to ensure these are marked as free. They will eventually be garbage collected but from
         * the security channel perspective these are done. */
        ptrSrvSecurityChannel->secContext[0].ptrSecurityContext  = NULL;
        ptrSrvSecurityChannel->secContext[0].sizeSecurityContext = 0;
    }
    else if((ptrSrvSecurityChannel->secContext[1].secContextId == scId) &&
            (ptrSrvSecurityChannel->secContext[1].ptrSecurityContext != NULL))
    {
        /* Add the security context to the garbage collection: */
        Netfp_addGarbageSecContext (ptrSrvSecurityChannel->ptrNetfpServer, &ptrSrvSecurityChannel->secContext[1]);

        /* Reset the fields to ensure these are marked as free. They will eventually be garbage collected but from
         * the security channel perspective these are done. */
        ptrSrvSecurityChannel->secContext[1].ptrSecurityContext  = NULL;
        ptrSrvSecurityChannel->secContext[1].sizeSecurityContext = 0;
    }
    else
    {
        Netfp_logMsg (ptrSrvSecurityChannel->ptrNetfpServer, Netfp_LogLevel_ERROR,
                      "Error: Security channel %x Security context %d does not match [%d] [%d]\n",
                      ptrSrvSecurityChannel, scId, ptrSrvSecurityChannel->secContext[0].secContextId,
                      ptrSrvSecurityChannel->secContext[1].secContextId);
    }
    return;
}

/**
 *  @brief
 *      The function registers the channel with the software routing
 *      information which is to be programmed in the PASS.
 *
 *  @param[in]   handle
 *      SALLD channel instance identifier.
 *  @param[in]   pSwInfo
 *      Pointer to SALLD software routing information structure.
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not Applicable
 */
static void Netfp_secChannelReg (Sa_ChanHandle handle, Sa_SWInfo_t* pSwInfo)
{
    Netfp_SrvSecurityChannel*   ptrSrvSecurityChannel;

    /* Get the security channel information: */
    ptrSrvSecurityChannel = (Netfp_SrvSecurityChannel*)Sa_chanGetID(handle);

    /* Store the rx swinfo in security channel handle */
    memcpy ((void *)&ptrSrvSecurityChannel->swInfo.rxInfo, (void *)pSwInfo, sizeof(Sa_SWInfo_t));
    return;
}

/**
 *  @brief
 *      The function unregisters the channel with the software routing information
 *
 *  @param[in]   handle
 *      SALLD channel instance identifier.
 *  @param[in]   pSwInfo
 *      Pointer to SALLD software routing information structure.
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not Applicable
 */
static void Netfp_secChannelUnreg (Sa_ChanHandle handle, Sa_SWInfo_t *pSwInfo)
{
    return;
}

/**
 *  @brief
 *      The function is used to send a NULL packet to tear down the Security
 *      Context associated with the channel.
 *
 *  @param[in]   handle
 *      SALLD channel instance identifier.
 *  @param[in]   pktInfo
 *      Pointer to the packet info structure.
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not Applicable
 */
static void Netfp_secChannelSendNullPkt (Sa_ChanHandle handle, Sa_PktInfo_t* pktInfo)
{
    Netfp_SrvSecurityChannel*   ptrSrvSecurityChannel;
    Sa_SWInfo_t*                pSwInfo;
    Ti_Pkt*                     ptrNullPacket;

    /* Get the security channel information: */
    ptrSrvSecurityChannel = (Netfp_SrvSecurityChannel*)Sa_chanGetID(handle);

    /* Get the software information: */
    pSwInfo = &pktInfo->swInfo;

    /* Allocate a packet from the NETFP Server command heap. */
    ptrNullPacket = Pktlib_allocPacket(ptrSrvSecurityChannel->ptrNetfpServer->cfg.pktlibInstHandle,
                                       ptrSrvSecurityChannel->ptrNetfpServer->cfg.cmdHeapHandle, 64);
    if (ptrNullPacket == NULL)
    {
        Netfp_logMsg (ptrSrvSecurityChannel->ptrNetfpServer, Netfp_LogLevel_ERROR,
                      "Error: No descriptor to send NULL packet for Security Channel %p\n", ptrSrvSecurityChannel);
        return;
    }
    Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)ptrNullPacket, 0);
    Cppi_setPSLen(Cppi_DescType_HOST, (Cppi_Desc *)ptrNullPacket,  0);
  	Cppi_setSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)ptrNullPacket, (uint8_t *)pSwInfo->swInfo);

    /******************************************************************************
     * NOTE: There is an ownership change here from the DSP to the CPDMA. We need
     * to ensure that the packet contents here are written back.
     ******************************************************************************/
	Pktlib_releaseOwnership(ptrSrvSecurityChannel->ptrNetfpServer->cfg.pktlibInstHandle, ptrNullPacket);

    /* Send the NULL packet to the correct queue. */
    if (ptrSrvSecurityChannel->netfpProtocol == Netfp_SaProtocol_IPSEC)
        Qmss_queuePushDescSize (ptrSrvSecurityChannel->ptrNetfpServer->netcpTxQueue[NSS_SA_QUEUE_SASS_INDEX], ptrNullPacket, 128);
    else
        Qmss_queuePushDescSize (ptrSrvSecurityChannel->ptrNetfpServer->netcpTxQueue[NSS_SA_QUEUE_SASS2_INDEX], ptrNullPacket, 128);
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to create the flow which is used to exchange packets between
 *      the PA and SA subsystem.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Netfp_createIPSecFlow (Netfp_ServerMCB* ptrNetfpServer)
{
    uint8_t             isAllocated;
    Cppi_RxFlowCfg      rxFlowCfg;
    Qmss_Queue          rxFreeQInfo;

    /* We need to now created the flow which will be used by the PA to send the configuration
     * responses back to the HOST. We want the flow to use the free descriptors from the internal
     * heap queue and place the received packets into the PA Configuration response queue. */
    rxFreeQInfo = Qmss_getQueueNumber (Pktlib_getInternalHeapQueue(ptrNetfpServer->cfg.ipSecHeapHandle));

    /* Create the Flow. Initialize the flow configuration. */
    memset (&rxFlowCfg, 0, sizeof(Cppi_RxFlowCfg));

    /* CPPI pick the next available flow */
    rxFlowCfg.flowIdNum             =   CPPI_PARAM_NOT_SPECIFIED;
    rxFlowCfg.rx_dest_qmgr          =   0;
    rxFlowCfg.rx_dest_qnum          =   0;
    rxFlowCfg.rx_sop_offset         =   0;
    rxFlowCfg.rx_desc_type          =   Cppi_DescType_HOST;
    rxFlowCfg.rx_ps_location        =   Cppi_PSLoc_PS_IN_DESC;
    rxFlowCfg.rx_psinfo_present     =   1;    /* Enable PS info */
    rxFlowCfg.rx_error_handling     =   0;    /* Drop the packet, do not retry on starvation by default */
    rxFlowCfg.rx_einfo_present      =   1;    /* EPIB info present */
    rxFlowCfg.rx_dest_tag_lo_sel    =   0;    /* Disable tagging */
    rxFlowCfg.rx_dest_tag_hi_sel    =   0;
    rxFlowCfg.rx_src_tag_lo_sel     =   0;
    rxFlowCfg.rx_src_tag_hi_sel     =   0;
    rxFlowCfg.rx_size_thresh0_en    =   0;    /* By default, we disable Rx Thresholds */
    rxFlowCfg.rx_size_thresh1_en    =   0;    /* By default, we disable Rx Thresholds */
    rxFlowCfg.rx_size_thresh2_en    =   0;    /* By default, we disable Rx Thresholds */
    rxFlowCfg.rx_size_thresh0       =   0x0;
    rxFlowCfg.rx_size_thresh1       =   0x0;
    rxFlowCfg.rx_size_thresh2       =   0x0;
    rxFlowCfg.rx_fdq0_sz0_qmgr      =   rxFreeQInfo.qMgr; /* Setup the Receive free queue for the flow */
    rxFlowCfg.rx_fdq0_sz0_qnum      =   Qmss_getQIDFromHandle(Pktlib_getInternalHeapQueue(ptrNetfpServer->cfg.ipSecHeapHandle));
    rxFlowCfg.rx_fdq0_sz1_qnum      =   0x0;
    rxFlowCfg.rx_fdq0_sz1_qmgr      =   0x0;
    rxFlowCfg.rx_fdq0_sz2_qnum      =   0x0;
    rxFlowCfg.rx_fdq0_sz2_qmgr      =   0x0;
    rxFlowCfg.rx_fdq0_sz3_qnum      =   0x0;
    rxFlowCfg.rx_fdq0_sz3_qmgr      =   0x0;
    rxFlowCfg.rx_fdq1_qnum          =   Qmss_getQIDFromHandle(Pktlib_getInternalHeapQueue(ptrNetfpServer->cfg.ipSecHeapHandle));
    rxFlowCfg.rx_fdq1_qmgr          =   rxFreeQInfo.qMgr;
    rxFlowCfg.rx_fdq2_qnum          =   Qmss_getQIDFromHandle(Pktlib_getInternalHeapQueue(ptrNetfpServer->cfg.ipSecHeapHandle));
    rxFlowCfg.rx_fdq2_qmgr          =   rxFreeQInfo.qMgr;
    rxFlowCfg.rx_fdq3_qnum          =   Qmss_getQIDFromHandle(Pktlib_getInternalHeapQueue(ptrNetfpServer->cfg.ipSecHeapHandle));
    rxFlowCfg.rx_fdq3_qmgr          =   rxFreeQInfo.qMgr;

    /* Configure the Rx flow */
    ptrNetfpServer->ipsecFlowHandle = Cppi_configureRxFlow (ptrNetfpServer->passCPDMAHandle,
                                                           &rxFlowCfg, &isAllocated);
    if (ptrNetfpServer->ipsecFlowHandle == NULL)
        return -1;
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to initialize the SA module in the NETCP subsystem.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[out] errCode
 *      Error code populated on error
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_saInit (Netfp_ServerMCB* ptrNetfpServer, int32_t* errCode)
{
    Sa_SizeCfg_t        sizeCfg;
    Sa_Config_t         cfg;
    int32_t             aligns[sa_N_BUFS];
    void*               bases[sa_N_BUFS];
    int32_t             sizes[sa_N_BUFS];
    int32_t             index;
    uint32_t            size;
    uint32_t            pdspVersion;

    /* Sanity Check: Validate the arguments */
    if (ptrNetfpServer->cfg.maxSecurityChannels == 0)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Sanity Check: Ensure that this is a multiple of 32. This allows us to optimize the
     * security context identifier allocation and cleanup. */
    if ((ptrNetfpServer->cfg.maxSecurityChannels % 32) != 0)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Sanity Check: In the ARM the virtual address configuration should be specified. */
    if ((ptrNetfpServer->cfg.realm == Netfp_ExecutionRealm_ARM) && (ptrNetfpServer->cfg.passCfgVirtualAddress == 0))
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Sanity Check: Ensure that there is a valid IPSEC heap handle which has been passed */
    if (ptrNetfpServer->cfg.ipSecHeapHandle == NULL)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Create the IPSEC Flow: */
    if (Netfp_createIPSecFlow(ptrNetfpServer) < 0)
    {
        *errCode = NETFP_EINTERNAL;
        return -1;
    }

    /* Initialize the SA size configuration: */
    memset ((void *)&sizeCfg, 0, sizeof (Sa_SizeCfg_t));

    /* Get the SA buffer requirements */
    sizeCfg.nMaxChan        = ptrNetfpServer->cfg.maxSecurityChannels;
    sizeCfg.cacheLineSize   = CACHE_L2_LINESIZE;
    sizeCfg.ctrlBitMap      = 0;
#ifdef DEVICE_K2L
    sizeCfg.ctrlBitMap      |= sa_SIZE_CONFIG_SASS_GEN2;
#endif
    *errCode = Sa_getBufferReq (&sizeCfg, sizes, aligns);
    if (*errCode != sa_ERR_OK)
        return -1;

    /* Allocate memory for the buffers. */
    for (index = 0; index < sa_N_BUFS; index++)
    {
        /* Align to cache line instead of one sent by SA LLD */
        bases[index] = (void *)ptrNetfpServer->cfg.malloc(sizes[index], CACHE_L2_LINESIZE);
        if (bases[index] == NULL)
            return -1;
    }

    /* Create the SA LLD Instance. */
    memset ((void *)&cfg, 0, sizeof (Sa_Config_t));

    /* Populate the SA configuration: */
    cfg.ID         = 0xC0;
    cfg.callTable  = &saMgmt_calloutFunc;
    cfg.sizeConfig = &sizeCfg;

    /* Setup the base address depending upon the execution realm. */
    if (ptrNetfpServer->cfg.realm == Netfp_ExecutionRealm_ARM)
        cfg.baseAddr = (uint32_t)ptrNetfpServer->cfg.passCfgVirtualAddress +
                       ((uint32_t)CSL_NETCP_CFG_SA_CFG_REGS - (uint32_t)CSL_NETCP_CFG_REGS);
    else
        cfg.baseAddr = (uint32_t)CSL_NETCP_CFG_SA_CFG_REGS;

    /* Debug Message: */
    Netfp_logMsg (ptrNetfpServer, Netfp_LogLevel_DEBUG, "Debug: SA Configuration base address %x\n", cfg.baseAddr);

#ifdef DEVICE_K2L
    /* We need to allocate memory for the internal buffer used by SA for temporary key storage: */
    cfg.intBuf = ptrNetfpServer->cfg.malloc(sa_MAX_SC_SIZE, CACHE_L2_LINESIZE);
    if (cfg.intBuf == NULL)
    {
        *errCode = NETFP_ENOMEM;
        return -1;
    }
    Netfp_logMsg (ptrNetfpServer, Netfp_LogLevel_DEBUG, "Debug: SA Internal Address %x [Size %d bytes]\n", cfg.intBuf, sa_MAX_SC_SIZE);
#endif

#ifdef SA_DEBUG
    /* Debug Flag: This will halt the SA on error */
    cfg.ctrlBitMap = sa_CONFIG_CTRL_BITMAP_TRIGGER_SYS_ERR_HALT;
#endif

    /* Create the SA */
    *errCode = Sa_create (&cfg, bases, &ptrNetfpServer->saHandle);
    if (*errCode != sa_ERR_OK)
        return -1;

    /* Download the SA firmware only if required to do so. */
    if (ptrNetfpServer->cfg.initSecureSubsystem == 1)
    {
        /* Download the Firmware images into the SA PDSP. */
        if (Netfp_downloadSAPDSP(ptrNetfpServer->saHandle, errCode) < 0)
            return -1;
    }

#ifdef DEVICE_K2L
    /* Debug Message: */
    Netfp_logMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "Debug: SA Version      : %x\n", Sa_getVersion());
    Sa_getPDSPVersion (ptrNetfpServer->saHandle, 0, &pdspVersion);
    Netfp_logMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "Debug: SA PDSP0 Version: %x\n", pdspVersion);
    Sa_getPDSPVersion (ptrNetfpServer->saHandle, 1, &pdspVersion);
    Netfp_logMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "Debug: SA PDSP1 Version: %x\n", pdspVersion);
    Sa_getPDSPVersion (ptrNetfpServer->saHandle, 2, &pdspVersion);
    Netfp_logMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "Debug: SA PDSP2 Version: %x\n", pdspVersion);
#else
    /* Debug Message: */
    Netfp_logMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "Debug: SA Version      : %x\n", Sa_getVersion());
    Sa_getPDSPVersion (ptrNetfpServer->saHandle, 0, &pdspVersion);
    Netfp_logMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "Debug: SA PDSP0 Version: %x\n", pdspVersion);
    Sa_getPDSPVersion (ptrNetfpServer->saHandle, 1, &pdspVersion);
    Netfp_logMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "Debug: SA PDSP1 Version: %x\n", pdspVersion);
#endif

    /* Allocate memory for the security context identifier database:
     *  We use 1 bit to determine if the security context is free or available.
     *  This ensures that 1 entry can handle upto 32 security context identifiers. */
    size = (ptrNetfpServer->cfg.maxSecurityChannels / 32) * 4;
    ptrNetfpServer->ptrSecContextIdDatabase = ptrNetfpServer->cfg.malloc(size, 0);
    if (ptrNetfpServer->ptrSecContextIdDatabase == NULL)
    {
        *errCode = NETFP_ENOMEM;
        return -1;
    }

    /* Initialize the security context base and mark as all identifiers are available */
    memset ((void *)ptrNetfpServer->ptrSecContextIdDatabase, 0, size);

    /* Debug Message: */
    Netfp_logMsg (ptrNetfpServer, Netfp_LogLevel_DEBUG, "Debug: SA Security Context Id Database [Database %p Size %d]\n",
                  ptrNetfpServer->ptrSecContextIdDatabase, size);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to configure the security channel for F8 i.e.
 *      ciphering
 *
 *  @param[in]  ptrSrvSecurityChannel
 *      Pointer to the NETFP Security channel
 *  @param[in]  ptrF8Cfg
 *      Pointer to the F8 configuration
 *  @param[out]  errCode
 *      Error code populated on error
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Netfp_configureF8Channel
(
    Netfp_SrvSecurityChannel*   ptrSrvSecurityChannel,
    Netfp_F8Cfg*                ptrF8Cfg,
    int32_t*                    errCode
)
{
    Sa_ChanCtrlInfo_t       channelControlInfo;
    Sa_PktInfo_t            pktInfo;

    /*************************************************************************
     ************* General Transmit Control Configuration ********************
     *************************************************************************/

    /* Initialize the channel configuration */
    memset((void *)&channelControlInfo, 0, sizeof(channelControlInfo));

    /* Populate the general channel configuration */
    channelControlInfo.ctrlType                      = sa_CHAN_CTRL_GEN_CONFIG;
    channelControlInfo.ctrlInfo.gen.validBitfield    = sa_CONTROLINFO_VALID_RX_CTRL     |
                                                       sa_CONTROLINFO_VALID_REPLAY_WIN  |
                                                       sa_CONTROLINFO_VALID_TX_CTRL;
    channelControlInfo.ctrlInfo.gen.replayWindowSize = 64;
    channelControlInfo.ctrlInfo.gen.txCtrl.authMode  = sa_AuthMode_NULL;

    /* Translate the ciphering mode to the SA ciphering mode. */
    switch (ptrF8Cfg->cipherMode)
    {
        case Netfp_3gppCipherMode_EEA0:
        {
            channelControlInfo.ctrlInfo.gen.txCtrl.cipherMode = sa_CipherMode_NULL;
            break;
        }
        case Netfp_3gppCipherMode_EEA1:
        {
            channelControlInfo.ctrlInfo.gen.txCtrl.cipherMode = sa_CipherMode_SNOW3G_F8;
            break;
        }
        case Netfp_3gppCipherMode_EEA2:
        {
            channelControlInfo.ctrlInfo.gen.txCtrl.cipherMode = sa_CipherMode_AES_CTR;
            break;
        }
    }

    /* NOTE: Only for SRB SNOW3G ciphering is NOT supported since this is done through the software module */
    if ((ptrF8Cfg->isDataRadioBearer == 0) && (ptrF8Cfg->cipherMode == Netfp_3gppCipherMode_EEA1))
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    if (ptrF8Cfg->isDataRadioBearer == 0)
        channelControlInfo.ctrlInfo.gen.txCtrl.params.ac.pduType = sa_AcPduType_LTE_CP;
    else
        channelControlInfo.ctrlInfo.gen.txCtrl.params.ac.pduType = sa_AcPduType_LTE;

    /* The definition of the ctrlBitMap defines that the direction bit it set to 0 i.e. from the
     * UE to the RNC. [NOTE: See the SA LLD documentation of Sa_AcConfigParams_t] Since the direction
     * is uplink; the packets are decoded and placed into the decoded queue */
    channelControlInfo.ctrlInfo.gen.txCtrl.params.ac.countC             = ptrF8Cfg->countC;
    channelControlInfo.ctrlInfo.gen.txCtrl.params.ac.fresh              = 0;
    channelControlInfo.ctrlInfo.gen.txCtrl.params.ac.ivLow26            = 0;
    channelControlInfo.ctrlInfo.gen.txCtrl.params.ac.ctrlBitMap         = (sa_AC_CONFIG_COPY_COUNTC | (0 << 8) | (ptrF8Cfg->rbId - 1));
    channelControlInfo.ctrlInfo.gen.txCtrl.params.ac.sessionEncKeySize  = 16;
    channelControlInfo.ctrlInfo.gen.txCtrl.params.ac.sessionMacKeySize  = 0;
    channelControlInfo.ctrlInfo.gen.txCtrl.params.ac.macSize            = 0;
    channelControlInfo.ctrlInfo.gen.txCtrl.params.ac.ivSize             = 16;

    /* Populate the destination where the channel will place the packets after the packet has been decoded */
    channelControlInfo.ctrlInfo.gen.txCtrl.destInfo.flowID  = ptrF8Cfg->decodeFlowId;
    channelControlInfo.ctrlInfo.gen.txCtrl.destInfo.queueID = Qmss_getQIDFromHandle(ptrF8Cfg->decodeQueue);
    channelControlInfo.ctrlInfo.gen.txCtrl.destInfo.swInfo0 = ((ptrF8Cfg->ueId << 16) | (ptrF8Cfg->qci << 8) | (ptrF8Cfg->rbId << 2));

    /*************************************************************************
     ************** General Receive Control Configuration ********************
     *************************************************************************/

    channelControlInfo.ctrlInfo.gen.rxCtrl.authMode   = sa_AuthMode_NULL;

    /* Translate the ciphering mode to the SA ciphering mode. */
    switch (ptrF8Cfg->cipherMode)
    {
        case Netfp_3gppCipherMode_EEA0:
        {
            channelControlInfo.ctrlInfo.gen.rxCtrl.cipherMode = sa_CipherMode_NULL;
            break;
        }
        case Netfp_3gppCipherMode_EEA1:
        {
            channelControlInfo.ctrlInfo.gen.rxCtrl.cipherMode = sa_CipherMode_SNOW3G_F8;
            break;
        }
        case Netfp_3gppCipherMode_EEA2:
        {
            channelControlInfo.ctrlInfo.gen.rxCtrl.cipherMode = sa_CipherMode_AES_CTR;
            break;
        }
    }

    if (ptrF8Cfg->isDataRadioBearer == 0)
        channelControlInfo.ctrlInfo.gen.rxCtrl.params.ac.pduType = sa_AcPduType_LTE_CP;
    else
        channelControlInfo.ctrlInfo.gen.rxCtrl.params.ac.pduType = sa_AcPduType_LTE;

    /* DRB: The COUNTC can be specified by the application */
    if (ptrF8Cfg->isDataRadioBearer == 1)
        channelControlInfo.ctrlInfo.gen.rxCtrl.params.ac.ctrlBitMap = sa_AC_CONFIG_COUNTC_BY_APP;

    /* The definition of the ctrlBitMap defines that the direction bit it set to 1 i.e. from the
     * RNC to the UE. [NOTE: See the SA LLD documentation of Sa_AcConfigParams_t] Since the direction
     * is downlink; the packets are encoded and placed into the encoded queue */
    channelControlInfo.ctrlInfo.gen.rxCtrl.params.ac.countC             = ptrF8Cfg->countC;
    channelControlInfo.ctrlInfo.gen.rxCtrl.params.ac.fresh              = 0;
    channelControlInfo.ctrlInfo.gen.rxCtrl.params.ac.ivLow26            = 0;
    channelControlInfo.ctrlInfo.gen.rxCtrl.params.ac.ctrlBitMap         |= (sa_AC_CONFIG_DIR | (ptrF8Cfg->rbId - 1));
    channelControlInfo.ctrlInfo.gen.rxCtrl.params.ac.sessionEncKeySize  = 16;
    channelControlInfo.ctrlInfo.gen.rxCtrl.params.ac.sessionMacKeySize  = 0;
    channelControlInfo.ctrlInfo.gen.rxCtrl.params.ac.macSize            = 0;
    channelControlInfo.ctrlInfo.gen.rxCtrl.params.ac.ivSize             = 16;

    /* Populate the destination where the channel will place the packets after the packet has been encoded */
    channelControlInfo.ctrlInfo.gen.rxCtrl.destInfo.flowID  = ptrF8Cfg->encodeFlowId;
    channelControlInfo.ctrlInfo.gen.rxCtrl.destInfo.queueID = Qmss_getQIDFromHandle(ptrF8Cfg->encodeQueue);
    channelControlInfo.ctrlInfo.gen.rxCtrl.destInfo.swInfo0 =
            ((ptrF8Cfg->ueId << 16) | (ptrF8Cfg->qci << 8) | ((ptrF8Cfg->rbId) << 2) | (ptrF8Cfg->currentMark));

    /* Configure the channel with the general configuration. */
    *errCode = Sa_chanControl (ptrSrvSecurityChannel->saChannelHandle, &channelControlInfo);
    if (*errCode != sa_ERR_OK)
        return -1;

    /* Copy over the SA channel general information into the security channel. */
    memcpy ((void *)&ptrSrvSecurityChannel->saGeneralChannelCtrlInfo, (void *)&channelControlInfo.ctrlInfo.gen, sizeof(Sa_GenCtrlInfo_t));

    /*************************************************************************
     ****************** Key Transmit Control Configuration *******************
     *************************************************************************/

    /* Initialize the channel configuration. */
    memset((void *)&channelControlInfo, 0, sizeof(channelControlInfo));

    /* Populate the general channel configuration. */
    channelControlInfo.ctrlType                             = sa_CHAN_CTRL_KEY_CONFIG;
    channelControlInfo.ctrlInfo.key.ctrlBitfield            = sa_KEY_CONTROL_TX_KEY_VALID | sa_KEY_CONTROL_RX_KEY_VALID;
    channelControlInfo.ctrlInfo.key.txKey.ac.ctrlBitfield   = sa_AC_KEY_CTRL_ENC_KEY;

    if (ptrF8Cfg->isDataRadioBearer == 0)
        channelControlInfo.ctrlInfo.key.txKey.ac.sessionEncKey = ptrF8Cfg->hKeyRrcEnc;
    else
        channelControlInfo.ctrlInfo.key.txKey.ac.sessionEncKey = ptrF8Cfg->hKeyUpEnc;

    /*************************************************************************
     ****************** Key Receive Control Configuration ********************
     *************************************************************************/

    channelControlInfo.ctrlInfo.key.rxKey.ac.ctrlBitfield = sa_AC_KEY_CTRL_ENC_KEY;

    if (ptrF8Cfg->isDataRadioBearer == 0)
        channelControlInfo.ctrlInfo.key.rxKey.ac.sessionEncKey = ptrF8Cfg->hKeyRrcEnc;
    else
        channelControlInfo.ctrlInfo.key.rxKey.ac.sessionEncKey = ptrF8Cfg->hKeyUpEnc;

    /* Configure the channel with the key configuration. */
    *errCode = Sa_chanControl (ptrSrvSecurityChannel->saChannelHandle, &channelControlInfo);
    if (*errCode != sa_ERR_OK)
        return -1;

    /*************************************************************************
     ****************************** Enable Channels **************************
     *************************************************************************/

    /* Initialize the channel configuration. */
    memset((void *)&channelControlInfo, 0, sizeof(channelControlInfo));

    /* Populate the general channel configuration to enable the transmit channel. */
    channelControlInfo.ctrlType                   = sa_CHAN_CTRL_GEN_CONFIG;
    channelControlInfo.ctrlInfo.gen.ctrlBitfield  = sa_CONTROLINFO_CTRL_TX_ON | sa_CONTROLINFO_CTRL_RX_ON;
    channelControlInfo.ctrlInfo.gen.validBitfield = sa_CONTROLINFO_VALID_CTRL_BITMAP;

    /* Enable the transmit & receive channels. */
    *errCode = Sa_chanControl (ptrSrvSecurityChannel->saChannelHandle, &channelControlInfo);
    if (*errCode != sa_ERR_OK)
        return -1;

    pktInfo.validBitMap = 0;

    /* Call SA LLD API to get software information for use when uplink slow path processing */
    Sa_chanSendData (ptrSrvSecurityChannel->saChannelHandle, &pktInfo, FALSE);
    memcpy ((void *) &ptrSrvSecurityChannel->swInfo.txInfo, (void *) &pktInfo.swInfo, sizeof(Sa_SWInfo_t));

    /* Call SA LLD API to get software information for use when downlink slow path processing */
    pktInfo.validBitMap = sa_PKT_INFO_VALID_PKT_ERR_CODE;
    pktInfo.pktErrCode = sa_PKT_ERR_OK;

    Sa_chanReceiveData (ptrSrvSecurityChannel->saChannelHandle, &pktInfo);
    memcpy ((void *) &ptrSrvSecurityChannel->swInfo.rxInfo, (void *) &pktInfo.swInfo, sizeof(Sa_SWInfo_t));
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to configure the security channel for F9 i.e.
 *      integrity protection
 *
 *  @param[in]  ptrSrvSecurityChannel
 *      Pointer to the NETFP Security channel
 *  @param[in]  ptrF9Cfg
 *      Pointer to the F9 configuration
 *  @param[out]  errCode
 *      Error code populated on error
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Netfp_configureF9Channel
(
    Netfp_SrvSecurityChannel*   ptrSrvSecurityChannel,
    Netfp_F9Cfg*                ptrF9Cfg,
    int32_t*                    errCode
)
{
    Sa_ChanCtrlInfo_t       channelControlInfo;
    Sa_PktInfo_t            pktInfo;

    /*************************************************************************
     ************* General Transmit Control Configuration ********************
     *************************************************************************/

    /* Initialize the channel configuration */
    memset((void *)&channelControlInfo, 0, sizeof(channelControlInfo));

    /* Populate the general channel configuration */
    channelControlInfo.ctrlType                       = sa_CHAN_CTRL_GEN_CONFIG;
    channelControlInfo.ctrlInfo.gen.validBitfield     = sa_CONTROLINFO_VALID_RX_CTRL    |
                                                        sa_CONTROLINFO_VALID_REPLAY_WIN |
                                                        sa_CONTROLINFO_VALID_TX_CTRL;
    channelControlInfo.ctrlInfo.gen.replayWindowSize  = 64;
    channelControlInfo.ctrlInfo.gen.txCtrl.cipherMode = sa_CipherMode_NULL;

    /* Translate the authentication mode to the SA authentication mode. */
    switch (ptrF9Cfg->authMode)
    {
        case Netfp_3gppAuthMode_EIA0:
        {
            channelControlInfo.ctrlInfo.gen.txCtrl.authMode   = sa_AuthMode_NULL;
            break;
        }
        case Netfp_3gppAuthMode_EIA2:
        {
            channelControlInfo.ctrlInfo.gen.txCtrl.authMode   = sa_AuthMode_CMAC;
            break;
        }
        default:
        {
            /* SNOW3G is supported via the software libaries. */
            *errCode = NETFP_EINVAL;
            return -1;
        }
    }

    if (ptrF9Cfg->isDataRadioBearer == 0)
        channelControlInfo.ctrlInfo.gen.txCtrl.params.ac.pduType = sa_AcPduType_LTE_CP;
    else
        channelControlInfo.ctrlInfo.gen.txCtrl.params.ac.pduType = sa_AcPduType_LTE;

    /* The definition of the ctrlBitMap defines that the direction bit it set to 0 i.e. from the
     * UE to the RNC. [NOTE: See the SA LLD documentation of Sa_AcConfigParams_t] Since the direction
     * is uplink the MAC-I is removed and COUNTC is copied into the timestamp field. */
    channelControlInfo.ctrlInfo.gen.txCtrl.params.ac.countC             = ptrF9Cfg->countC;
    channelControlInfo.ctrlInfo.gen.txCtrl.params.ac.fresh              = 0;
    channelControlInfo.ctrlInfo.gen.txCtrl.params.ac.ivLow26            = 0;
    channelControlInfo.ctrlInfo.gen.txCtrl.params.ac.ctrlBitMap         = (sa_AC_CONFIG_COPY_COUNTC | (0 << 8) | (ptrF9Cfg->rbId - 1));
    channelControlInfo.ctrlInfo.gen.txCtrl.params.ac.sessionEncKeySize  = 0;
    channelControlInfo.ctrlInfo.gen.txCtrl.params.ac.sessionMacKeySize  = 16;
    channelControlInfo.ctrlInfo.gen.txCtrl.params.ac.macSize            = 4;
    channelControlInfo.ctrlInfo.gen.txCtrl.params.ac.ivSize             = 8;

    /* Populate the destination where the channel will place the packets after the MAC-I has been removed */
    channelControlInfo.ctrlInfo.gen.txCtrl.destInfo.flowID   = ptrF9Cfg->uplinkFlowId;
    channelControlInfo.ctrlInfo.gen.txCtrl.destInfo.queueID  = Qmss_getQIDFromHandle(ptrF9Cfg->uplinkQueue);
    channelControlInfo.ctrlInfo.gen.txCtrl.destInfo.swInfo0 = ((ptrF9Cfg->ueId << 16) | (ptrF9Cfg->qci << 8) | (ptrF9Cfg->rbId << 2));

    /*************************************************************************
     ************** General Receive Control Configuration ********************
     *************************************************************************/

    channelControlInfo.ctrlInfo.gen.rxCtrl.cipherMode = sa_CipherMode_NULL;

    /* Translate the authentication mode to the SA authentication mode. */
    switch (ptrF9Cfg->authMode)
    {
        case Netfp_3gppAuthMode_EIA0:
        {
            channelControlInfo.ctrlInfo.gen.rxCtrl.authMode   = sa_AuthMode_NULL;
            break;
        }
        case Netfp_3gppAuthMode_EIA2:
        {
            channelControlInfo.ctrlInfo.gen.rxCtrl.authMode   = sa_AuthMode_CMAC;
            break;
        }
        default:
        {
            /* SNOW3G is supported via the software libaries. */
            *errCode = NETFP_EINVAL;
            return -1;
        }
    }

    if (ptrF9Cfg->isDataRadioBearer == 0)
        channelControlInfo.ctrlInfo.gen.rxCtrl.params.ac.pduType = sa_AcPduType_LTE_CP;
    else
        channelControlInfo.ctrlInfo.gen.rxCtrl.params.ac.pduType = sa_AcPduType_LTE;

    /* The definition of the ctrlBitMap defines that the direction bit it set to 1 i.e. from the
     * RNC to the UE. [NOTE: See the SA LLD documentation of Sa_AcConfigParams_t] Since the direction
     * is downlink the MAC-I is added to the packet */
    channelControlInfo.ctrlInfo.gen.rxCtrl.params.ac.countC             = ptrF9Cfg->countC;
    channelControlInfo.ctrlInfo.gen.rxCtrl.params.ac.fresh              = 0;
    channelControlInfo.ctrlInfo.gen.rxCtrl.params.ac.ivLow26            = 0;
    channelControlInfo.ctrlInfo.gen.rxCtrl.params.ac.ctrlBitMap         = (sa_AC_CONFIG_DIR | (ptrF9Cfg->rbId - 1));
    channelControlInfo.ctrlInfo.gen.rxCtrl.params.ac.sessionEncKeySize  = 0;
    channelControlInfo.ctrlInfo.gen.rxCtrl.params.ac.sessionMacKeySize  = 16;
    channelControlInfo.ctrlInfo.gen.rxCtrl.params.ac.macSize            = 4;
    channelControlInfo.ctrlInfo.gen.rxCtrl.params.ac.ivSize             = 8;

    /* Populate the destination where the channel will place the packets after the MAC-I has been added */
    channelControlInfo.ctrlInfo.gen.rxCtrl.destInfo.flowID  = ptrF9Cfg->downlinkFlowId;
    channelControlInfo.ctrlInfo.gen.rxCtrl.destInfo.queueID = Qmss_getQIDFromHandle(ptrF9Cfg->downlinkQueue);
    channelControlInfo.ctrlInfo.gen.rxCtrl.destInfo.swInfo0 = ((ptrF9Cfg->ueId << 16) | (ptrF9Cfg->qci << 8) | (ptrF9Cfg->rbId << 2));

    /* Configure the channel with the general configuration. */
    *errCode = Sa_chanControl (ptrSrvSecurityChannel->saChannelHandle, &channelControlInfo);
    if (*errCode != sa_ERR_OK)
        return -1;

    /* Copy over the SA channel general information into the security channel. */
    memcpy ((void *)&ptrSrvSecurityChannel->saGeneralChannelCtrlInfo, (void *)&channelControlInfo.ctrlInfo.gen, sizeof(Sa_GenCtrlInfo_t));

    /*************************************************************************
     ********************** Key Control Configuration ************************
     *************************************************************************/

    /* Initialize the channel configuration. */
    memset((void *)&channelControlInfo, 0, sizeof(channelControlInfo));

    /* Populate the channel configuration. */
    channelControlInfo.ctrlType                             = sa_CHAN_CTRL_KEY_CONFIG;
    channelControlInfo.ctrlInfo.key.ctrlBitfield            = sa_KEY_CONTROL_TX_KEY_VALID | sa_KEY_CONTROL_RX_KEY_VALID;
    channelControlInfo.ctrlInfo.key.txKey.ac.ctrlBitfield   = sa_AC_KEY_CTRL_MAC_KEY;
    channelControlInfo.ctrlInfo.key.txKey.ac.sessionAuthKey = ptrF9Cfg->hKeyRrcInt;
    channelControlInfo.ctrlInfo.key.rxKey.ac.ctrlBitfield   = sa_AC_KEY_CTRL_MAC_KEY;
    channelControlInfo.ctrlInfo.key.rxKey.ac.sessionAuthKey = ptrF9Cfg->hKeyRrcInt;

    /* Configure the channel with the keys */
    *errCode = Sa_chanControl (ptrSrvSecurityChannel->saChannelHandle, &channelControlInfo);
    if (*errCode != sa_ERR_OK)
        return -1;

    /*************************************************************************
     ****************************** Enable Channels **************************
     *************************************************************************/

    /* Initialize the channel configuration. */
    memset((void *)&channelControlInfo, 0, sizeof(channelControlInfo));

    /* Populate the general channel configuration to enable the transmit channel. */
    channelControlInfo.ctrlType                   = sa_CHAN_CTRL_GEN_CONFIG;
    channelControlInfo.ctrlInfo.gen.ctrlBitfield  = sa_CONTROLINFO_CTRL_TX_ON | sa_CONTROLINFO_CTRL_RX_ON;
    channelControlInfo.ctrlInfo.gen.validBitfield = sa_CONTROLINFO_VALID_CTRL_BITMAP;

    /* Enable the transmit & receive channels. */
    *errCode = Sa_chanControl (ptrSrvSecurityChannel->saChannelHandle, &channelControlInfo);
    if (*errCode != sa_ERR_OK)
        return -1;

    pktInfo.validBitMap = 0;

    /* Call SA LLD API to get software information for use when uplink slow path processing */
    Sa_chanSendData (ptrSrvSecurityChannel->saChannelHandle, &pktInfo, FALSE);
    memcpy ((void *)&ptrSrvSecurityChannel->swInfo.txInfo, (void *) &pktInfo.swInfo, sizeof(Sa_SWInfo_t));

    /* Call SA LLD API to get software information for use when downlink slow path processing */
    pktInfo.validBitMap = sa_PKT_INFO_VALID_PKT_ERR_CODE;
    pktInfo.pktErrCode  = sa_PKT_ERR_OK;
    Sa_chanReceiveData (ptrSrvSecurityChannel->saChannelHandle, &pktInfo);
    memcpy ((void *)&ptrSrvSecurityChannel->swInfo.rxInfo, (void *) &pktInfo.swInfo, sizeof(Sa_SWInfo_t));

    /* Channel has been configured. */
    return 0;
}
/**
 *  @b Description
 *  @n
 *      The function is used to configure the security channel for IPSEC
 *
 *  @param[in]  ptrSrvSecurityChannel
 *      Pointer to the NETFP Security channel
 *  @param[in]  ptrSACfg
 *      Pointer to the SA configuration
 *  @param[out]  errCode
 *      Error code populated on error
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Netfp_configureIPSecChannel
(
    Netfp_SrvSecurityChannel*   ptrSrvSecurityChannel,
    Netfp_SACfg*                ptrSACfg,
    int32_t*                    errCode
)
{
    Sa_ChanCtrlInfo_t       channelControlInfo;
    uint8_t*                ptrSalt=NULL;
    uint16_t                sessionEncKeySize;
    uint16_t                sessionSaltSize;

    /* Initialize the channel configuration */
    memset((void *)&channelControlInfo, 0, sizeof(channelControlInfo));

    /* Populate the general channel configuration */
    channelControlInfo.ctrlType                      = sa_CHAN_CTRL_GEN_CONFIG;
    channelControlInfo.ctrlInfo.gen.replayWindowSize = ptrSACfg->replayWindowSize;

    /* Configure the channel depending upon the direction of the security association */
    if (ptrSACfg->direction == Netfp_Direction_INBOUND)
    {
        channelControlInfo.ctrlInfo.gen.validBitfield  = sa_CONTROLINFO_VALID_RX_CTRL       |
                                                         sa_CONTROLINFO_VALID_REPLAY_WIN;

        /* Setup the transport type: NETFP supports only tunnel. */
        channelControlInfo.ctrlInfo.gen.rxCtrl.params.ipsec.transportType = sa_IPSEC_TRANSPORT_TUNNEL;

        /* Setup the parameters depending upon the encryption mode. */
        switch (ptrSACfg->ipsecCfg.encMode)
        {
            case Netfp_IpsecCipherMode_NULL:
            {
                channelControlInfo.ctrlInfo.gen.rxCtrl.cipherMode                       = sa_CipherMode_NULL;
                channelControlInfo.ctrlInfo.gen.rxCtrl.params.ipsec.encryptionBlockSize = 1;
                channelControlInfo.ctrlInfo.gen.rxCtrl.params.ipsec.ivSize              = 0;
                channelControlInfo.ctrlInfo.gen.rxCtrl.params.ipsec.sessionSaltSize     = 0;
                break;
            }
            case Netfp_IpsecCipherMode_AES_CBC:
            {
                channelControlInfo.ctrlInfo.gen.rxCtrl.cipherMode                       = sa_CipherMode_AES_CBC;
                channelControlInfo.ctrlInfo.gen.rxCtrl.params.ipsec.encryptionBlockSize = 16;
                channelControlInfo.ctrlInfo.gen.rxCtrl.params.ipsec.ivSize              = 16;
                channelControlInfo.ctrlInfo.gen.rxCtrl.params.ipsec.sessionSaltSize     = 0;
                break;
            }
            case Netfp_IpsecCipherMode_3DES_CBC:
            {
                channelControlInfo.ctrlInfo.gen.rxCtrl.cipherMode                       = sa_CipherMode_3DES_CBC;
                channelControlInfo.ctrlInfo.gen.rxCtrl.params.ipsec.encryptionBlockSize = 8;
                channelControlInfo.ctrlInfo.gen.rxCtrl.params.ipsec.ivSize              = 8;
                channelControlInfo.ctrlInfo.gen.rxCtrl.params.ipsec.sessionSaltSize     = 0;
                break;
            }
            case Netfp_IpsecCipherMode_DES_CBC:
            {
                channelControlInfo.ctrlInfo.gen.rxCtrl.cipherMode                       = sa_CipherMode_DES_CBC;
                channelControlInfo.ctrlInfo.gen.rxCtrl.params.ipsec.encryptionBlockSize = 8;
                channelControlInfo.ctrlInfo.gen.rxCtrl.params.ipsec.ivSize              = 8;
                channelControlInfo.ctrlInfo.gen.rxCtrl.params.ipsec.sessionSaltSize     = 0;
                break;
            }
            case Netfp_IpsecCipherMode_AES_CTR:
            {
                channelControlInfo.ctrlInfo.gen.rxCtrl.cipherMode                       = sa_CipherMode_AES_CTR;
                channelControlInfo.ctrlInfo.gen.rxCtrl.params.ipsec.encryptionBlockSize = 4;
                channelControlInfo.ctrlInfo.gen.rxCtrl.params.ipsec.ivSize              = 8;
                channelControlInfo.ctrlInfo.gen.rxCtrl.params.ipsec.sessionSaltSize     = 4;
                break;
            }
            case Netfp_IpsecCipherMode_AES_GCM:
            {
                channelControlInfo.ctrlInfo.gen.rxCtrl.cipherMode                       = sa_CipherMode_GCM;
                channelControlInfo.ctrlInfo.gen.rxCtrl.params.ipsec.encryptionBlockSize = 4;
                channelControlInfo.ctrlInfo.gen.rxCtrl.params.ipsec.ivSize              = 8;
                channelControlInfo.ctrlInfo.gen.rxCtrl.params.ipsec.sessionSaltSize     = 4;
                break;
            }
            default:
            {
                *errCode = NETFP_EINVAL;
                return -1;
            }
        }

        /* Translate the NETFP authentication mode to the SA LLD language */
        switch (ptrSACfg->ipsecCfg.authMode)
        {
            case Netfp_IpsecAuthMode_NULL:
            {
                channelControlInfo.ctrlInfo.gen.rxCtrl.authMode = sa_AuthMode_NULL;
                break;
            }
            case Netfp_IpsecAuthMode_HMAC_SHA1:
            {
                channelControlInfo.ctrlInfo.gen.rxCtrl.authMode = sa_AuthMode_HMAC_SHA1;
                break;
            }
            case Netfp_IpsecAuthMode_HMAC_MD5:
            {
                channelControlInfo.ctrlInfo.gen.rxCtrl.authMode = sa_AuthMode_HMAC_MD5;
                break;
            }
            case Netfp_IpsecAuthMode_AES_XCBC:
            {
                channelControlInfo.ctrlInfo.gen.rxCtrl.authMode = sa_AuthMode_AES_XCBC;
                break;
            }
            case Netfp_IpsecAuthMode_HMAC_SHA2_256:
            {
                channelControlInfo.ctrlInfo.gen.rxCtrl.authMode = sa_AuthMode_HMAC_SHA2_256;
                break;
            }
            case Netfp_IpsecAuthMode_AES_GMAC:
            {

                channelControlInfo.ctrlInfo.gen.rxCtrl.authMode = sa_AuthMode_GMAC;
                channelControlInfo.ctrlInfo.gen.rxCtrl.params.ipsec.ivSize              = 8;
                channelControlInfo.ctrlInfo.gen.rxCtrl.params.ipsec.sessionSaltSize     = 4;

                /* Switch from encryption algorithm to authentication algorithm */
                ptrSACfg->ipsecCfg.keyAuthSize = ptrSACfg->ipsecCfg.keyEncSize -channelControlInfo.ctrlInfo.gen.rxCtrl.params.ipsec.sessionSaltSize;
                memcpy(ptrSACfg->ipsecCfg.keyAuth,ptrSACfg->ipsecCfg.keyEnc, ptrSACfg->ipsecCfg.keyEncSize);
                ptrSalt = ptrSACfg->ipsecCfg.keyAuth;
                ptrSalt = ptrSalt + ptrSACfg->ipsecCfg.keyAuthSize;

                /* Disable encryption */
                ptrSACfg->ipsecCfg.encMode = Netfp_IpsecCipherMode_NULL;
                ptrSACfg->ipsecCfg.keyEncSize=0;
                memset(ptrSACfg->ipsecCfg.keyEnc,0,sizeof(ptrSACfg->ipsecCfg.keyEnc));


                break;
            }
            default:
            {
                *errCode = NETFP_EINVAL;
                return -1;
            }
        }

        /************************************************************************************
         * For INBOUND IPSEC channels; we are indicating to the SA LLD to pass the packet
         * to the NETCP PDSP2 once it is done with the packet decryption. This is done using
         * the IPSEC Flow identifier.
         *
         * The NETFP_IPSEC_DEBUG flag can be defined to capture the packet into a temporary
         * queue post decryption.
         *************************************************************************************/
        channelControlInfo.ctrlInfo.gen.rxCtrl.destInfo.flowID  = Cppi_getFlowId(ptrSrvSecurityChannel->ptrNetfpServer->ipsecFlowHandle);

#ifdef NETFP_IPSEC_DEBUG
        /* Test queue number 2000 */
        channelControlInfo.ctrlInfo.gen.rxCtrl.destInfo.queueID = 2000;
#else
        /* PDSP 2 */
        channelControlInfo.ctrlInfo.gen.rxCtrl.destInfo.queueID = ptrSrvSecurityChannel->ptrNetfpServer->netcpTxQueue[NSS_PA_QUEUE_INNER_IP_INDEX];
#endif

        channelControlInfo.ctrlInfo.gen.rxCtrl.params.ipsec.ctrlBitMap          = ptrSACfg->ipsecCfg.esnEnabled;
        channelControlInfo.ctrlInfo.gen.rxCtrl.params.ipsec.esnLo               = ptrSACfg->ipsecCfg.esnLo;
        channelControlInfo.ctrlInfo.gen.rxCtrl.params.ipsec.esnHi               = ptrSACfg->ipsecCfg.esnHi;
        channelControlInfo.ctrlInfo.gen.rxCtrl.params.ipsec.nextHdr             = IPPROTO_IPIP;
        channelControlInfo.ctrlInfo.gen.rxCtrl.params.ipsec.spi                 = ptrSACfg->spi;
        channelControlInfo.ctrlInfo.gen.rxCtrl.params.ipsec.sessionMacKeySize   = ptrSACfg->ipsecCfg.keyAuthSize;
        channelControlInfo.ctrlInfo.gen.rxCtrl.params.ipsec.macSize             = ptrSACfg->ipsecCfg.keyMacSize;

        /* Is there an encryption key which has been specified? */
        if (ptrSACfg->ipsecCfg.keyEncSize == 0)
            channelControlInfo.ctrlInfo.gen.rxCtrl.params.ipsec.sessionEncKeySize = 0;
        else
            channelControlInfo.ctrlInfo.gen.rxCtrl.params.ipsec.sessionEncKeySize = ptrSACfg->ipsecCfg.keyEncSize -
                                                                                    channelControlInfo.ctrlInfo.gen.rxCtrl.params.ipsec.sessionSaltSize;

        /* Configure the channel with the general configuration. */
        *errCode = Sa_chanControl (ptrSrvSecurityChannel->saChannelHandle, &channelControlInfo);
        if (*errCode != sa_ERR_OK)
            return -1;

        /* Keep track of the session key and salt sizes. */
        sessionEncKeySize = channelControlInfo.ctrlInfo.gen.rxCtrl.params.ipsec.sessionEncKeySize;
        sessionSaltSize   = channelControlInfo.ctrlInfo.gen.rxCtrl.params.ipsec.sessionSaltSize;

        /* Copy over the SA channel general information into the security channel. */
        memcpy ((void *)&ptrSrvSecurityChannel->saGeneralChannelCtrlInfo, (void *)&channelControlInfo.ctrlInfo.gen, sizeof(Sa_GenCtrlInfo_t));

        /*************************************************************************
         ******************* Receive Key Control Configuration *******************
         *************************************************************************/

        /* Initialize the channel configuration. */
        memset((void *)&channelControlInfo, 0, sizeof(channelControlInfo));

        /* Populate the general channel configuration. */
        channelControlInfo.ctrlType                  = sa_CHAN_CTRL_KEY_CONFIG;
        channelControlInfo.ctrlInfo.key.ctrlBitfield = sa_KEY_CONTROL_RX_KEY_VALID;

        /* Configure the keys */
        channelControlInfo.ctrlInfo.key.rxKey.ipsec.sessionEncKey   = ptrSACfg->ipsecCfg.keyEnc;
        channelControlInfo.ctrlInfo.key.rxKey.ipsec.sessionAuthKey  = ptrSACfg->ipsecCfg.keyAuth;


        /* TODO: Do we need to perform the check for NULL encryption here? In the case of NULL ciphering
         * the sessionSaltSize is already 0. So the check is irrelevant */
        if ((sessionSaltSize != 0) && (ptrSACfg->ipsecCfg.encMode != Netfp_IpsecCipherMode_NULL))
        {
            ptrSalt = ptrSACfg->ipsecCfg.keyEnc;
            ptrSalt = ptrSalt + sessionEncKeySize;

        }
        channelControlInfo.ctrlInfo.key.rxKey.ipsec.sessionSalt = ptrSalt;

        channelControlInfo.ctrlInfo.key.rxKey.ipsec.ctrlBitfield = sa_IPSEC_KEY_CTRL_MAC_KEY;
        if (sessionEncKeySize)
            channelControlInfo.ctrlInfo.key.rxKey.ipsec.ctrlBitfield |= sa_IPSEC_KEY_CTRL_ENC_KEY;
        if (sessionSaltSize)
            channelControlInfo.ctrlInfo.key.rxKey.ipsec.ctrlBitfield |= sa_IPSEC_KEY_CTRL_SALT;

        /* Configure the channel with the key configuration. */
        *errCode = Sa_chanControl (ptrSrvSecurityChannel->saChannelHandle, &channelControlInfo);
        if (*errCode != sa_ERR_OK)
            return -1;

        /*************************************************************************
         ****************************** Enable Channels **************************
        *************************************************************************/

        /* Initialize the channel configuration. */
        memset((void *)&channelControlInfo, 0, sizeof(channelControlInfo));

        /* Populate the general channel configuration to enable the receive channel. */
        channelControlInfo.ctrlType                   = sa_CHAN_CTRL_GEN_CONFIG;
        channelControlInfo.ctrlInfo.gen.ctrlBitfield  = sa_CONTROLINFO_CTRL_RX_ON;
        channelControlInfo.ctrlInfo.gen.validBitfield = sa_CONTROLINFO_VALID_CTRL_BITMAP;

        /* Enable the receive channel. */
        *errCode = Sa_chanControl (ptrSrvSecurityChannel->saChannelHandle, &channelControlInfo);
        if (*errCode != sa_ERR_OK)
            return -1;

        /* The IPSEC channel has been created and instantiated in the SA LLD. We now need to initialize
         * the SW information which is required by the NETCP subsystem to decrypt the packet. */
        *errCode = Sa_chanGetSwInfo (ptrSrvSecurityChannel->saChannelHandle,  sa_PKT_DIR_FROM_NETWORK,
                                     &ptrSrvSecurityChannel->swInfo.rxInfo);
        if (*errCode != sa_ERR_OK)
	    	return -1;

        /* Inbound IPSEC channel has been configured successfully */
        return 0;
    }

    /* Outbound IPSEC channels are configured here. */
	channelControlInfo.ctrlInfo.gen.validBitfield  = sa_CONTROLINFO_VALID_TX_CTRL;

    /* NETFP supports only tunnel transports. */
    channelControlInfo.ctrlInfo.gen.txCtrl.params.ipsec.transportType = sa_IPSEC_TRANSPORT_TUNNEL;

    /* Setup the parameters depending upon the encryption mode. */
    switch (ptrSACfg->ipsecCfg.encMode)
    {
		case Netfp_IpsecCipherMode_NULL:
		{
			channelControlInfo.ctrlInfo.gen.txCtrl.cipherMode                       = sa_CipherMode_NULL;
			channelControlInfo.ctrlInfo.gen.txCtrl.params.ipsec.encryptionBlockSize = 1;
			channelControlInfo.ctrlInfo.gen.txCtrl.params.ipsec.ivSize              = 0;
			channelControlInfo.ctrlInfo.gen.txCtrl.params.ipsec.sessionSaltSize     = 0;
			break;
		}
		case Netfp_IpsecCipherMode_AES_CBC:
		{
			channelControlInfo.ctrlInfo.gen.txCtrl.cipherMode                       = sa_CipherMode_AES_CBC;
			channelControlInfo.ctrlInfo.gen.txCtrl.params.ipsec.encryptionBlockSize = 16;
			channelControlInfo.ctrlInfo.gen.txCtrl.params.ipsec.ivSize              = 16;
			channelControlInfo.ctrlInfo.gen.txCtrl.params.ipsec.sessionSaltSize     = 0;
			break;
		}
		case Netfp_IpsecCipherMode_3DES_CBC:
		{
			channelControlInfo.ctrlInfo.gen.txCtrl.cipherMode                       = sa_CipherMode_3DES_CBC;
			channelControlInfo.ctrlInfo.gen.txCtrl.params.ipsec.encryptionBlockSize = 8;
			channelControlInfo.ctrlInfo.gen.txCtrl.params.ipsec.ivSize              = 8;
			channelControlInfo.ctrlInfo.gen.txCtrl.params.ipsec.sessionSaltSize     = 0;
			break;
		}
		case Netfp_IpsecCipherMode_DES_CBC:
		{
			channelControlInfo.ctrlInfo.gen.txCtrl.cipherMode                       = sa_CipherMode_DES_CBC;
			channelControlInfo.ctrlInfo.gen.txCtrl.params.ipsec.encryptionBlockSize = 8;
			channelControlInfo.ctrlInfo.gen.txCtrl.params.ipsec.ivSize              = 8;
			channelControlInfo.ctrlInfo.gen.txCtrl.params.ipsec.sessionSaltSize     = 0;
			break;
		}
		case Netfp_IpsecCipherMode_AES_CTR:
		{
			channelControlInfo.ctrlInfo.gen.txCtrl.cipherMode                       = sa_CipherMode_AES_CTR;
			channelControlInfo.ctrlInfo.gen.txCtrl.params.ipsec.encryptionBlockSize = 4;
			channelControlInfo.ctrlInfo.gen.txCtrl.params.ipsec.ivSize              = 8;
			channelControlInfo.ctrlInfo.gen.txCtrl.params.ipsec.sessionSaltSize     = 4;
			break;
		}
        case Netfp_IpsecCipherMode_AES_GCM:
        {
            channelControlInfo.ctrlInfo.gen.txCtrl.cipherMode                       = sa_CipherMode_GCM;
            channelControlInfo.ctrlInfo.gen.txCtrl.params.ipsec.encryptionBlockSize = 4;
            channelControlInfo.ctrlInfo.gen.txCtrl.params.ipsec.ivSize              = 8;
            channelControlInfo.ctrlInfo.gen.txCtrl.params.ipsec.sessionSaltSize     = 4;
            break;
        }
		default:
		{
			*errCode = NETFP_EINVAL;
			return -1;
		}
    }

    /* Translate the NETFP authentication mode to the SA LLD language */
    switch (ptrSACfg->ipsecCfg.authMode)
    {
        case Netfp_IpsecAuthMode_NULL:
        {
            channelControlInfo.ctrlInfo.gen.txCtrl.authMode = sa_AuthMode_NULL;
            break;
        }
        case Netfp_IpsecAuthMode_HMAC_SHA1:
        {
            channelControlInfo.ctrlInfo.gen.txCtrl.authMode = sa_AuthMode_HMAC_SHA1;
            break;
        }
        case Netfp_IpsecAuthMode_HMAC_MD5:
        {
            channelControlInfo.ctrlInfo.gen.txCtrl.authMode = sa_AuthMode_HMAC_MD5;
            break;
        }
        case Netfp_IpsecAuthMode_AES_XCBC:
        {
            channelControlInfo.ctrlInfo.gen.txCtrl.authMode = sa_AuthMode_AES_XCBC;
            break;
        }
        case Netfp_IpsecAuthMode_HMAC_SHA2_256:
        {
            channelControlInfo.ctrlInfo.gen.txCtrl.authMode = sa_AuthMode_HMAC_SHA2_256;
            break;
        }
        case Netfp_IpsecAuthMode_AES_GMAC:
        {

            channelControlInfo.ctrlInfo.gen.txCtrl.authMode = sa_AuthMode_GMAC;
            channelControlInfo.ctrlInfo.gen.txCtrl.params.ipsec.ivSize              = 8;
            channelControlInfo.ctrlInfo.gen.txCtrl.params.ipsec.sessionSaltSize     = 4;

            /* Switch from encryption algorithm to authentication algorithm */
            ptrSACfg->ipsecCfg.keyAuthSize = ptrSACfg->ipsecCfg.keyEncSize -channelControlInfo.ctrlInfo.gen.txCtrl.params.ipsec.sessionSaltSize;
            memcpy(ptrSACfg->ipsecCfg.keyAuth,ptrSACfg->ipsecCfg.keyEnc, ptrSACfg->ipsecCfg.keyEncSize);
            ptrSalt = ptrSACfg->ipsecCfg.keyAuth;
            ptrSalt = ptrSalt + ptrSACfg->ipsecCfg.keyAuthSize;

            /* Disable encryption */
            ptrSACfg->ipsecCfg.encMode = Netfp_IpsecCipherMode_NULL;
            ptrSACfg->ipsecCfg.keyEncSize=0;
            memset(ptrSACfg->ipsecCfg.keyEnc,0,sizeof(ptrSACfg->ipsecCfg.keyEnc));
            break;
        }
        default:
        {
            *errCode = NETFP_EINVAL;
            return -1;
        }
    }

    /************************************************************************************
     * For OUTBOUND IPSEC channels; we are indicating to the SA LLD to pass the packet
     * to the NETCP PDSP5 once it is done with the packet encryption. The PDSP5 will then
     * follow the additional PA command which have been added to the packets before
     * it gets sent out.
     *
     * The NETFP_IPSEC_DEBUG flag can be defined to capture the packet into a temporary
     * queue post encryption.
     *************************************************************************************/
    channelControlInfo.ctrlInfo.gen.txCtrl.destInfo.flowID  = Cppi_getFlowId(ptrSrvSecurityChannel->ptrNetfpServer->ipsecFlowHandle);

#ifdef NETFP_IPSEC_DEBUG
    channelControlInfo.ctrlInfo.gen.txCtrl.destInfo.queueID = 3000; /* Test queue number 3000 */
#else
    channelControlInfo.ctrlInfo.gen.txCtrl.destInfo.queueID = ptrSrvSecurityChannel->ptrNetfpServer->netcpTxQueue[NSS_PA_QUEUE_TXCMD_INDEX]; /* PDSP 5 */
#endif

    channelControlInfo.ctrlInfo.gen.txCtrl.params.ipsec.ctrlBitMap          = ptrSACfg->ipsecCfg.esnEnabled;
    channelControlInfo.ctrlInfo.gen.txCtrl.params.ipsec.esnLo               = ptrSACfg->ipsecCfg.esnLo;
    channelControlInfo.ctrlInfo.gen.txCtrl.params.ipsec.esnHi               = ptrSACfg->ipsecCfg.esnHi;
    channelControlInfo.ctrlInfo.gen.txCtrl.params.ipsec.nextHdr             = IPPROTO_IPIP;
    channelControlInfo.ctrlInfo.gen.txCtrl.params.ipsec.spi                 = ptrSACfg->spi;
    channelControlInfo.ctrlInfo.gen.txCtrl.params.ipsec.sessionMacKeySize   = ptrSACfg->ipsecCfg.keyAuthSize;
    channelControlInfo.ctrlInfo.gen.txCtrl.params.ipsec.macSize             = ptrSACfg->ipsecCfg.keyMacSize;

    if (ptrSACfg->ipsecCfg.keyEncSize == 0)
        channelControlInfo.ctrlInfo.gen.txCtrl.params.ipsec.sessionEncKeySize = 0;
    else
        channelControlInfo.ctrlInfo.gen.txCtrl.params.ipsec.sessionEncKeySize = ptrSACfg->ipsecCfg.keyEncSize -
                                                                                channelControlInfo.ctrlInfo.gen.txCtrl.params.ipsec.sessionSaltSize;

    /* Configure the channel with the general configuration. */
    *errCode = Sa_chanControl (ptrSrvSecurityChannel->saChannelHandle, &channelControlInfo);
    if (*errCode != sa_ERR_OK)
        return -1;

    /* Keep track of the session key and salt sizes. */
    sessionEncKeySize = channelControlInfo.ctrlInfo.gen.txCtrl.params.ipsec.sessionEncKeySize;
    sessionSaltSize   = channelControlInfo.ctrlInfo.gen.txCtrl.params.ipsec.sessionSaltSize;

    /* Copy over the SA channel general information into the security channel. */
    memcpy ((void *)&ptrSrvSecurityChannel->saGeneralChannelCtrlInfo, (void *)&channelControlInfo.ctrlInfo.gen, sizeof(Sa_GenCtrlInfo_t));

	/*************************************************************************
	 ****************** Transmit Key Control Configuration *******************
	 *************************************************************************/

	/* Initialize the channel configuration. */
	memset((void *)&channelControlInfo, 0, sizeof(channelControlInfo));

	/* Populate the general channel configuration. */
	channelControlInfo.ctrlType                                 = sa_CHAN_CTRL_KEY_CONFIG;
	channelControlInfo.ctrlInfo.key.ctrlBitfield                = sa_KEY_CONTROL_TX_KEY_VALID;
	channelControlInfo.ctrlInfo.key.txKey.ipsec.sessionEncKey   = ptrSACfg->ipsecCfg.keyEnc;
	channelControlInfo.ctrlInfo.key.txKey.ipsec.sessionAuthKey  = ptrSACfg->ipsecCfg.keyAuth;

    /* TODO: Do we need to perform the check for NULL encryption here? In the case of NULL ciphering
     * the sessionSaltSize is already 0. So the check is irrelevant */
	if ((sessionSaltSize != 0) && (ptrSACfg->ipsecCfg.encMode != Netfp_IpsecCipherMode_NULL))
	{
		ptrSalt = ptrSACfg->ipsecCfg.keyEnc;
		ptrSalt = ptrSalt + sessionEncKeySize;
	}
    channelControlInfo.ctrlInfo.key.txKey.ipsec.sessionSalt = ptrSalt;

	channelControlInfo.ctrlInfo.key.txKey.ipsec.ctrlBitfield = sa_IPSEC_KEY_CTRL_MAC_KEY;
	if (sessionEncKeySize)
		channelControlInfo.ctrlInfo.key.txKey.ipsec.ctrlBitfield |= sa_IPSEC_KEY_CTRL_ENC_KEY;
	if (sessionSaltSize)
		channelControlInfo.ctrlInfo.key.txKey.ipsec.ctrlBitfield |= sa_IPSEC_KEY_CTRL_SALT;

	/* Configure the channel with the key configuration. */
	*errCode = Sa_chanControl (ptrSrvSecurityChannel->saChannelHandle, &channelControlInfo);
	if (*errCode != sa_ERR_OK)
		return -1;

	/*************************************************************************
	 ****************************** Enable Channels **************************
	*************************************************************************/

	/* Initialize the channel configuration. */
	memset((void *)&channelControlInfo, 0, sizeof(channelControlInfo));

	/* Populate the general channel configuration to enable the transmit channel. */
	channelControlInfo.ctrlType                   = sa_CHAN_CTRL_GEN_CONFIG;
	channelControlInfo.ctrlInfo.gen.ctrlBitfield  = sa_CONTROLINFO_CTRL_TX_ON;
	channelControlInfo.ctrlInfo.gen.validBitfield = sa_CONTROLINFO_VALID_CTRL_BITMAP;

	/* Enable the transmit & receive channels. */
	*errCode = Sa_chanControl (ptrSrvSecurityChannel->saChannelHandle, &channelControlInfo);
	if (*errCode != sa_ERR_OK)
		return -1;

    /* The IPSEC channel has been created and instantiated in the SA LLD. We now need to initialize
     * the SW information which is required by the NETCP subsystem to encrypt the packet. */
    *errCode = Sa_chanGetSwInfo (ptrSrvSecurityChannel->saChannelHandle, sa_PKT_DIR_TO_NETWORK,
                                 &ptrSrvSecurityChannel->swInfo.txInfo);
    if (*errCode != sa_ERR_OK)
		return -1;

    /* Outbound IPSEC channel has been configured */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to configure the security channel with the specified
 *      configuration
 *
 *  @param[in]  ptrSrvSecurityChannel
 *      Pointer to the NETFP Security channel
 *  @param[in]  ptrSecChannelCfg
 *      Pointer to the security channel configuration
 *  @param[out]  errCode
 *      Error code populated on error
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Netfp_configureSecurityChannel
(
    Netfp_SrvSecurityChannel*   ptrSrvSecurityChannel,
    Netfp_SecChannelCfg*        ptrSecChannelCfg,
    int32_t*                    errCode
)
{
    /* Process the configuration depending upon the */
    switch (ptrSecChannelCfg->type)
    {
        case Netfp_SecurityChannelType_AIR_F8:
        {
            return Netfp_configureF8Channel (ptrSrvSecurityChannel, &ptrSecChannelCfg->u.f8Cfg, errCode);
        }
        case Netfp_SecurityChannelType_AIR_F9:
        {
            return Netfp_configureF9Channel (ptrSrvSecurityChannel, &ptrSecChannelCfg->u.f9Cfg, errCode);
        }
        case Netfp_SecurityChannelType_IPSEC:
        {
            return Netfp_configureIPSecChannel (ptrSrvSecurityChannel, &ptrSecChannelCfg->u.ipSecCfg, errCode);
        }
    }
    *errCode = NETFP_EINVAL;
    return -1;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the SA channel statistics from the NETCP
 *
 *  @param[in]  saChannelHandle
 *      Handle to the SA channel for which the statistics are requested
 *  @param[out] ptrSAStats
 *      Pointer to the SA channel stats populated on error
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_getSAStatistics
(
    Sa_ChanHandle   saChannelHandle,
    Sa_Stats_t*     ptrSAStats
)
{
    int32_t     index;
    int16_t     errCode;

    /* Get the SA LLD Channel statistics: */
    errCode = Sa_chanGetStats (saChannelHandle, sa_STATS_QUERY_FLAG_TRIG, ptrSAStats);
    if (errCode == sa_ERR_OK)
        return 0;

    /* Wait for the stats reply */
    for (index = 0; index < 100; index++)
    {
        /* Delay for some time */
	    Netfp_cycleDelay (1000);

        /* Try and get the SA Channel statistics: */
        errCode = Sa_chanGetStats (saChannelHandle, 0, ptrSAStats);
	    if (errCode == sa_ERR_OK)
		    return 0;
    }

    /* Did we timeout? */
    if (index == 100)
    {
        /* YES. Try and get the statistics immediately: */
        errCode = Sa_chanGetStats (saChannelHandle, sa_STATS_QUERY_FLAG_NOW, ptrSAStats);
        if (errCode == sa_ERR_OK)
            return 0;
    }
    return errCode;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the SA channel statistics from the NETCP
 *
 *  @param[in]  saChannelHandle
 *      Handle to the SA channel for which the statistics are requested
 *  @param[in]  dir
 *      Direction of the SA channel
 *  @param[out] swInfo
 *      Pointer to Hold SA software info
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_getChanSwInfo
(
    Sa_ChanHandle   saChannelHandle,
    Sa_PktDir_t     dir,
    Sa_SWInfo_t*    swInfo
)
{
    /* Get security channel software info from SA */
    return ( Sa_chanGetSwInfo (saChannelHandle,  dir, swInfo) );
}

/**
 *  @b Description
 *  @n
 *      The function is used to create the channel in the SA LLD. The function
 *      supports channel creation for IPSEC and 3GPP.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  ptrSecChannelCfg
 *      Pointer to the security channel configuration
 *  @param[out] ptrSwInfo
 *      Pointer to the channel software information populated by the API
 *  @param[out]  errCode
 *      Error code populated on error
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   Server security channel handle
 *  @retval
 *      Error   -   NULL
 */
Netfp_SrvSecChannelHandle _Netfp_createSecurityChannel
(
    Netfp_ServerMCB*        ptrNetfpServer,
    Netfp_SecChannelCfg*    ptrSecChannelCfg,
    Netfp_SecuritySwInfo*   ptrSwInfo,
    int32_t*                errCode
)
{
    Sa_ChanConfig_t             channelConfig;
    int32_t                     index;
    Netfp_SrvSecurityChannel*   ptrSrvSecurityChannel;
    Sa_SecProto_e               protocol;
    Sa_Stats_t                  saDummyStats;

    /* Security Context was available and has been allocated. Allocate memory for the
     * security channel information block. */
    ptrSrvSecurityChannel = ptrNetfpServer->cfg.malloc(sizeof(Netfp_SrvSecurityChannel), 0);
    if (ptrSrvSecurityChannel == NULL)
    {
        *errCode = NETFP_ENOMEM;
        return NULL;
    }

    /* Initialize the allocated memory block. */
    memset ((void *)ptrSrvSecurityChannel, 0, sizeof(Netfp_SrvSecurityChannel));

    /* Populate the security channel information: */
    ptrSrvSecurityChannel->ptrNetfpServer = ptrNetfpServer;

    /* Initialize the channel properties if the channel being created is IPSEC or AC */
    if (ptrSecChannelCfg->type == Netfp_SecurityChannelType_IPSEC)
    {
        ptrSrvSecurityChannel->netfpProtocol = Netfp_SaProtocol_IPSEC;
        protocol                             = sa_PT_IPSEC_ESP;
    }
    else
    {
        ptrSrvSecurityChannel->netfpProtocol = Netfp_SaProtocol_3GPP;
        protocol                             = sa_PT_3GPP_AC;
    }

    /* Get the SA buffer requirements for the channel: */
    memset ((void *)&channelConfig, 0, sizeof(Sa_ChanConfig_t));

    /* Populate the channel configuration */
    channelConfig.ID                       = (uint32_t)ptrSrvSecurityChannel;
    channelConfig.sizeConfig.protocolType  = protocol;
    channelConfig.sizeConfig.cacheLineSize = CACHE_L2_LINESIZE;
    channelConfig.sizeConfig.ctrlBitMap    = 0;
#ifdef DEVICE_K2L
    channelConfig.engSelect                = 0;
#endif

    /* Get the SA Buffer Requirements */
    *errCode = Sa_chanGetBufferReq (&channelConfig.sizeConfig, ptrSrvSecurityChannel->sizes, ptrSrvSecurityChannel->aligns);
    if (*errCode != sa_ERR_OK)
        return NULL;

    /* Cycle through all the buffers */
    for (index = 0; index < sa_N_BUFS; index++)
    {
        /* Allocate memory for the SA Buffers: Align to cache line instead of one sent by SA LLD */
        ptrSrvSecurityChannel->bases[index] = ptrNetfpServer->cfg.malloc(ptrSrvSecurityChannel->sizes[index], CACHE_L2_LINESIZE);
        if (ptrSrvSecurityChannel->bases[index] == NULL)
        {
            *errCode = NETFP_ENOMEM;
            return NULL;
        }
    }

    /* Create the SA channel. */
    *errCode = Sa_chanCreate (ptrNetfpServer->saHandle, &channelConfig, (void *)ptrSrvSecurityChannel->bases,
                              &ptrSrvSecurityChannel->saChannelHandle);
    if (*errCode != sa_ERR_OK)
        return NULL;

    /* Channel has been created: We now need to configure the channel with the supplied configuration. */
    if (Netfp_configureSecurityChannel (ptrSrvSecurityChannel, ptrSecChannelCfg, errCode) < 0)
        return NULL;

    /* Add the server security channel to the list. */
    Netfp_listAdd ((Netfp_ListNode**)&ptrNetfpServer->ptrServerSecurityChannelList, (Netfp_ListNode*)ptrSrvSecurityChannel);

    /* Copy over the software information from the server security channel */
    memcpy ((void *)ptrSwInfo, (void *)&ptrSrvSecurityChannel->swInfo, sizeof(Netfp_SecuritySwInfo));

    /* SA Security Context Magic: We need to ensure that the security context status is synchronized between the SA cache
     * and DDR3 memory. Requesting for the SA channel statistics will ensure that the synchronization is done. So here we
     * trigger the dummy SA statistics retreival just for the synchronization. */
    Sa_chanGetStats (ptrSrvSecurityChannel->saChannelHandle, sa_STATS_QUERY_FLAG_TRIG, &saDummyStats);

    /* Return the server channel handle which has been created. */
    return (Netfp_SrvSecChannelHandle)ptrSrvSecurityChannel;
}

/**
 *  @b Description
 *  @n
 *      The function is used to delete the channel in the SA LLD. The function
 *      supports channel deletion for IPSEC and 3GPP.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  srvSecurityChannel
 *      Server security channel handle to be deleted
 *  @param[out]  errCode
 *      Error code populated on error
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t _Netfp_deleteSecurityChannel
(
    Netfp_ServerMCB*            ptrNetfpServer,
    Netfp_SrvSecChannelHandle   srvSecurityChannel,
    int32_t*                    errCode
)
{
    uint32_t                    index;
    Netfp_SrvSecurityChannel*   ptrSrvSecurityChannel;

    /* Get the security channel handle. */
    ptrSrvSecurityChannel = (Netfp_SrvSecurityChannel*)srvSecurityChannel;
    if (ptrSrvSecurityChannel == NULL)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* SA Security Context Magic: On channel creation we had triggered the dummy statistics retrieval
     * which would have ensured that the security context buffer status has been synchronized between
     * the DDR3 and SA. At this time the security context buffer should be owned by the SA if the
     * ownership has not been transferred to the SA we will continue to loop around. */
    if (ptrSrvSecurityChannel->secContext[0].ptrSecurityContext != NULL)
    {
        while (1)
        {
            if (Sa_isScBufFree (ptrSrvSecurityChannel->secContext[0].ptrSecurityContext) == 0)
                break;
        }
    }
    if (ptrSrvSecurityChannel->secContext[1].ptrSecurityContext != NULL)
    {
        while (1)
        {
            if (Sa_isScBufFree (ptrSrvSecurityChannel->secContext[1].ptrSecurityContext) == 0)
                break;
        }
    }

    /* Close the channel in the SA LLD */
    *errCode = Sa_chanClose (ptrSrvSecurityChannel->saChannelHandle, ptrSrvSecurityChannel->bases);
    if (*errCode != sa_ERR_OK)
        return -1;

    /* Cleanup the memory associated which was allocated to the SA channel */
    for (index = 0; index < sa_N_BUFS; index++)
        ptrNetfpServer->cfg.free (ptrSrvSecurityChannel->bases[index], ptrSrvSecurityChannel->sizes[index]);

    /* Remove the server security channel from the list. */
    Netfp_listRemoveNode ((Netfp_ListNode**)&ptrNetfpServer->ptrServerSecurityChannelList, (Netfp_ListNode*)ptrSrvSecurityChannel);

    /* Cleanup the memory associated with the security channel information: */
    ptrNetfpServer->cfg.free (ptrSrvSecurityChannel, sizeof(Netfp_SrvSecurityChannel));

    /* Execute the security context garbage collection: We try and cleanup 2 security contexts [Receive & Transmit] */
    Netfp_executeSecContextGarbageCollection (ptrNetfpServer, 2);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to delete the channel in the SA LLD. The function
 *      supports channel deletion for IPSEC and 3GPP.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  srvSecurityChannel
 *      Server security channel handle to be deleted
 *  @param[out]  errCode
 *      Error code populated on error
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_deleteSecurityChannelSlow
(
    Netfp_ServerMCB*            ptrNetfpServer,
    Netfp_SrvSecChannelHandle   srvSecurityChannel,
    int32_t*                    errCode
)
{
    Netfp_SrvSecurityChannel*   ptrSrvSecurityChannel;

    /* Get the security channel handle. */
    ptrSrvSecurityChannel = (Netfp_SrvSecurityChannel*)srvSecurityChannel;
    if (ptrSrvSecurityChannel == NULL)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }
    Netfp_executeSlowSecContextCleanup(ptrNetfpServer);

    /* Remove the server security channel from the list. */
    Netfp_listRemoveNode ((Netfp_ListNode**)&ptrNetfpServer->ptrServerSecurityChannelList, (Netfp_ListNode*)ptrSrvSecurityChannel);

    ptrSrvSecurityChannel->slowCleanupTimestamp = ddal_cpu_counter_read();
    /* Add this to the slow cleanup list: */
    Netfp_listAdd ((Netfp_ListNode**)&ptrNetfpServer->ptrSlowCleanupSecurityContextList, (Netfp_ListNode*)ptrSrvSecurityChannel);
    Netfp_logMsg (ptrNetfpServer, Netfp_LogLevel_VERBOSE, "Moved security channel [%p] to slow cleanup list at time: %llu\n", ptrSrvSecurityChannel, ptrSrvSecurityChannel->slowCleanupTimestamp);
    return 0;
}

int32_t Netfp_executeSlowSecContextCleanup
(
    Netfp_ServerMCB*        ptrNetfpServer
)
{
    int32_t errCode;
    uint32_t count = 0;
    uint32_t index;
    uint64_t currentStamp = ddal_cpu_counter_read();
    uint64_t freq = ddal_cpu_counter_freq();
    Netfp_SrvSecurityChannel*  ptrSrvSecurityChannel;
    Netfp_SrvSecurityChannel*  ptrSrvSecurityChannelNext;

    /* Cycle through all the entries in the garbage collection */
    ptrSrvSecurityChannel = (Netfp_SrvSecurityChannel*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrSlowCleanupSecurityContextList);
    while (ptrSrvSecurityChannel != NULL)
    {
        ptrSrvSecurityChannelNext = (Netfp_SrvSecurityChannel*)Netfp_listGetNext ((Netfp_ListNode*)ptrSrvSecurityChannel);
        /* Check if the entry was pushed to the slow cleanup list more than 30 seconds ago*/
        if (currentStamp - ptrSrvSecurityChannel->slowCleanupTimestamp < (freq * 30))
        {
            /* NO: Skip the entry. */
            ptrSrvSecurityChannel = ptrSrvSecurityChannelNext;
            continue;
        }

        /* SA Security Context Magic: On channel creation we had triggered the dummy statistics retrieval
         * which would have ensured that the security context buffer status has been synchronized between
         * the DDR3 and SA. At this time the security context buffer should be owned by the SA if the
         * ownership has not been transferred to the SA we will continue to loop around. */
        if (ptrSrvSecurityChannel->secContext[0].ptrSecurityContext != NULL)
        {
            while (1)
            {
                if (Sa_isScBufFree (ptrSrvSecurityChannel->secContext[0].ptrSecurityContext) == 0)
                    break;
            }
        }
        if (ptrSrvSecurityChannel->secContext[1].ptrSecurityContext != NULL)
        {
            while (1)
            {
                if (Sa_isScBufFree (ptrSrvSecurityChannel->secContext[1].ptrSecurityContext) == 0)
                    break;
            }
        }

        /* Close the channel in the SA LLD */
        errCode = Sa_chanClose (ptrSrvSecurityChannel->saChannelHandle, ptrSrvSecurityChannel->bases);
        if (errCode != sa_ERR_OK)
            return -1;

        /* Cleanup the memory associated which was allocated to the SA channel */
        for (index = 0; index < sa_N_BUFS; index++)
            ptrNetfpServer->cfg.free (ptrSrvSecurityChannel->bases[index], ptrSrvSecurityChannel->sizes[index]);

        /* Remove the server security channel from the list. */
        Netfp_listRemoveNode ((Netfp_ListNode**)&ptrNetfpServer->ptrSlowCleanupSecurityContextList, (Netfp_ListNode*)ptrSrvSecurityChannel);

        /* Cleanup the memory associated with the security channel information: */
        ptrNetfpServer->cfg.free (ptrSrvSecurityChannel, sizeof(Netfp_SrvSecurityChannel));


        Netfp_logMsg (ptrNetfpServer, Netfp_LogLevel_VERBOSE, "Deleted security channel [%p] at time: %llu\n", ptrSrvSecurityChannel, currentStamp);
        count++;
        ptrSrvSecurityChannel = ptrSrvSecurityChannelNext;
    }
    /* Execute the security context garbage collection*/
    Netfp_executeSecContextGarbageCollection (ptrNetfpServer, count);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the security channel information.
 *      Currently only CountC option is supported.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  srvSecurityChannelHandle
 *      Server security channel handle
 *  @param[out] ptrOptInfo
 *      Option information which is populated by the API
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t _Netfp_getSecurityChannelOpt
(
    Netfp_ServerMCB*            ptrNetfpServer,
    Netfp_SrvSecChannelHandle   srvSecurityChannelHandle,
    Netfp_OptionTLV*            ptrOptInfo,
    int32_t*                    errCode
)
{
    Netfp_SrvSecurityChannel*   ptrSrvSecurityChannel;
    Sa_Stats_t                  saStats;

    /* Get the Server security channel handle */
    ptrSrvSecurityChannel = (Netfp_SrvSecurityChannel *)srvSecurityChannelHandle;

    /* The TLV has the value which is actually a pointer which points to the start of the actual data
     * Pointers cannot be handled natively by the JOSH framework. We had translated the value pointer
     * to now be moved after the TLV. So here we update this pointer appropriately. */
    ptrOptInfo->value = (uint8_t*)ptrOptInfo + sizeof(Netfp_OptionTLV);

    /* Process only the valid Interface Level Options. */
    switch (ptrOptInfo->type)
    {
        case Netfp_Option_COUNTC:
        {
            /* Sanity Check: Validate and ensure that the arguments are sized as documented. */
            if (ptrOptInfo->length != 4)
            {
                *errCode = NETFP_EINVAL;
                return -1;
            }

            /* Get the CountC value from the channel statistics. */
            *errCode = Netfp_getSAStatistics (ptrSrvSecurityChannel->saChannelHandle, &saStats);
            if (*errCode < 0)
                return -1;

            *(uint32_t*)ptrOptInfo->value = saStats.ac.toAirCountC;
            ptrOptInfo->length = 4;
            break;
        }
        default:
        {
            /* Option is NOT supported */
            *errCode = NETFP_EINVAL;
            return -1;
        }
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to create the SRB Authentication and Ciphering channels in the SA LLD for 3GPP.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server.
 *  @param[in]  ptrUserCfg
 *      Pointer to the User configuration.
 *  @param[out]  ptrUserSrbInfo
 *      Pointer to the user SRB security channel and software information.
 *      [in]    Temporary queue information used between authentication and ciphering operation
 *  @param[out]  errCode
 *      Error code populated on error.
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t _Netfp_createSrb
(
    Netfp_ServerMCB*            ptrNetfpServer,
    Netfp_UserCfg*              ptrUserCfg,
    Netfp_UserSRBInfo*          ptrUserSrbInfo,
    int32_t*                    errCode
)
{
    Netfp_SecChannelCfg         secChannelCfg;
    uint32_t                    index;

    /* Populate the security channel configuration for integrity protection */
    secChannelCfg.type                       = Netfp_SecurityChannelType_AIR_F9;
    secChannelCfg.u.f9Cfg.ueId               = ptrUserCfg->ueId;
    secChannelCfg.u.f9Cfg.qci                = 0;
    secChannelCfg.u.f9Cfg.isDataRadioBearer  = 0;
    secChannelCfg.u.f9Cfg.authMode           = ptrUserCfg->authMode;
    secChannelCfg.u.f9Cfg.downlinkFlowId     = ptrUserCfg->srbFlowId;
    secChannelCfg.u.f9Cfg.uplinkFlowId       = ptrUserCfg->srbFlowId;
    secChannelCfg.u.f9Cfg.countC             = ptrUserCfg->initialCountC;
    memcpy ((void *)&secChannelCfg.u.f9Cfg.hKeyRrcInt, (void *)ptrUserCfg->hKeyRrcInt, 16);

    /* Create the SRB security channels with the specific configuration: */
    for (index = 1; index < 3; index++)
    {
        /* Override fields in the security channel configuration which are different for
         * each SRB. */
        if (index == 1)
        {
            secChannelCfg.u.f9Cfg.rbId          = 1;
            secChannelCfg.u.f9Cfg.uplinkQueue   = ptrUserCfg->chSrb1Dec;

            /* TODO: In the downlink the packets are passed through the F9 (integrity protection)
             * and then they are then pushed for F8 (ciphering). However if the ciperhing mode is set
             * to AES then we need to pass the packet after the F9 to a temporary queue and then move
             * it back to the SA for the F8. */
            if (ptrUserCfg->srbCipherMode == Netfp_3gppCipherMode_EEA2)
            {
                /* AES Ciphering: The packets once authenticated should be passed to the temporary queue */
                secChannelCfg.u.f9Cfg.downlinkQueue = ptrUserSrbInfo->SRBAuthInfo[index].destQueueHnd;
            }
            else
            {
                /* NULL/SNOW3G Ciphering: The packets once authenticated can be directly passed to the final
                 * encoded queue. */
                secChannelCfg.u.f9Cfg.downlinkQueue = ptrUserCfg->chSrb1Enc;
            }
        }
        else
        {
            secChannelCfg.u.f9Cfg.rbId          = 2;
            secChannelCfg.u.f9Cfg.uplinkQueue   = ptrUserCfg->chSrb2Dec;

            /* TODO: In the downlink the packets are passed through the F9 (integrity protection)
             * and then they are then pushed for F8 (ciphering). However if the ciperhing mode is set
             * to AES then we need to pass the packet after the F9 to a temporary queue and then move
             * it back to the SA for the F8. */
            if (ptrUserCfg->srbCipherMode == Netfp_3gppCipherMode_EEA2)
            {
                /* AES Ciphering: The packets once authenticated should be passed to the temporary queue */
                secChannelCfg.u.f9Cfg.downlinkQueue = ptrUserSrbInfo->SRBAuthInfo[index].destQueueHnd;
            }
            else
            {
                /* NULL/SNOW3G Ciphering: The packets once authenticated can be directly passed to the final
                 * encoded queue. */
                secChannelCfg.u.f9Cfg.downlinkQueue = ptrUserCfg->chSrb2Enc;
            }
        }

        /* Create the SRB server security channel. */
        ptrUserSrbInfo->SRBAuthInfo[index].srvSecurityChannelHandle = _Netfp_createSecurityChannel (ptrNetfpServer, &secChannelCfg,
                                                                         &ptrUserSrbInfo->SRBAuthInfo[index].swInfo, errCode);
        if (ptrUserSrbInfo->SRBAuthInfo[index].srvSecurityChannelHandle == NULL)
            return -1;
    }

    /* Populate the security channel configuration for ciphering */
    secChannelCfg.type                       = Netfp_SecurityChannelType_AIR_F8;
    secChannelCfg.u.f8Cfg.ueId               = ptrUserCfg->ueId;
    secChannelCfg.u.f8Cfg.qci                = 0;
    secChannelCfg.u.f8Cfg.isDataRadioBearer  = 0;
    secChannelCfg.u.f8Cfg.cipherMode         = ptrUserCfg->srbCipherMode;
    secChannelCfg.u.f8Cfg.encodeFlowId       = ptrUserCfg->srbFlowId;
    secChannelCfg.u.f8Cfg.decodeFlowId       = ptrUserCfg->srbFlowId;
    secChannelCfg.u.f8Cfg.countC             = ptrUserCfg->initialCountC;
    secChannelCfg.u.f8Cfg.currentMark        = 0;
    memcpy ((void *)&secChannelCfg.u.f8Cfg.hKeyRrcEnc, (void *)ptrUserCfg->hKeyRrcEnc, 16);

    /* Create the SRB security channels with the specific configuration: */
    for (index = 1; index < 3; index++)
    {
        /* Override fields in the security channel configuration which are different for
         * each SRB. */
        if (index == 1)
        {
            secChannelCfg.u.f8Cfg.rbId          = 1;
            secChannelCfg.u.f8Cfg.encodeQueue   = ptrUserCfg->chSrb1Enc;

            /* TODO: In the uplink the packets are passed through the F8 (ciphering) and then they are
             * then pushed for F9 (integrity protection). However if the ciperhing mode is set
             * to AES then we need to pass the packet after the F8 to a temporary queue and then move
             * it back to the SA for the F9. */
            if (ptrUserCfg->srbCipherMode == Netfp_3gppCipherMode_EEA2)
            {
                /* AES Ciphering: The packets are decoded and placed into the temporary queue */
                secChannelCfg.u.f8Cfg.decodeQueue = ptrUserSrbInfo->SRBCipherInfo[index].destQueueHnd;
            }
            else
            {
                /* NULL/SNOW3G Ciphering: The packets are decoded and placed into the final decoded queue */
                secChannelCfg.u.f8Cfg.decodeQueue = ptrUserCfg->chSrb1Dec;
            }
        }
        else
        {
            secChannelCfg.u.f8Cfg.rbId          = 2;
            secChannelCfg.u.f8Cfg.encodeQueue   = ptrUserCfg->chSrb2Enc;

            /* TODO: In the uplink the packets are passed through the F8 (ciphering) and then they are
             * then pushed for F9 (integrity protection). However if the ciperhing mode is set
             * to AES then we need to pass the packet after the F8 to a temporary queue and then move
             * it back to the SA for the F9. */
            if (ptrUserCfg->srbCipherMode == Netfp_3gppCipherMode_EEA2)
            {
                /* AES Ciphering: The packets are decoded and placed into the temporary queue */
                secChannelCfg.u.f8Cfg.decodeQueue = ptrUserSrbInfo->SRBCipherInfo[index].destQueueHnd;
            }
            else
            {
                /* NULL/SNOW3G Ciphering: The packets are decoded and placed into the final decoded queue */
                secChannelCfg.u.f8Cfg.decodeQueue = ptrUserCfg->chSrb2Dec;
            }
        }

        /* Create the SRB server security channel. */
        ptrUserSrbInfo->SRBCipherInfo[index].srvSecurityChannelHandle = _Netfp_createSecurityChannel (ptrNetfpServer, &secChannelCfg,
                                                                               &ptrUserSrbInfo->SRBCipherInfo[index].swInfo, errCode);
        if (ptrUserSrbInfo->SRBCipherInfo[index].srvSecurityChannelHandle == NULL)
            return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to create the SRB Authentication and Ciphering channels in the SA LLD for 3GPP.
 *
 *  @param[in]  ptrNetfpClient
 *      Pointer to the NETFP client.
 *  @param[in]  ptrUserCfg
 *      Pointer to the User configuration.
 *  @param[out]  ptrUserSrbInfo
 *      Pointer to the user SRB security channel and software information.
 *  @param[out]  errCode
 *      Error code populated on error.
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_createSrb
(
    Netfp_ClientMCB*            ptrNetfpClient,
    Netfp_UserCfg*              ptrUserCfg,
    Netfp_UserSRBInfo*          ptrUserSrbInfo,
    int32_t*                    errCode
)
{
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    uint32_t                result;
    int32_t                 jobId;
    Netfp_UserCfg*          ptrRemoteUserCfg;
    Netfp_UserSRBInfo*      ptrRemoteUserSrbInfo;

    /* Get the actual job which we will execute */
    jobHandle = Josh_findJobByAddress(ptrNetfpClient->joshHandle, (Josh_JobProtype)_Netfp_createSrb);
    if (jobHandle == NULL)
    {
        *errCode = NETFP_EINTERNAL;
        return -1;
    }

    /* Initialize the arguments; to avoid any junk */
    memset ((void *)&args, 0, sizeof(args));

    /***************************************************************************
     * This is the function which is to be invoked:
     *
     *  int32_t _Netfp_createSrb
     *  (
     *  Netfp_ServerMCB*        ptrNetfpServer,
     *  Netfp_UserCfg*          ptrUserCfg,
     *  Netfp_UserSRBInfo*      ptrUserSrbInfo,
     *  int32_t*                errCode
     *  )
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Netfp_ServerHandle);

    /*  - Argument 2: */
    args[1].type   = Josh_ArgumentType_PASS_BY_REF;
    args[1].length = sizeof(Netfp_UserCfg);

    /*  - Argument 3: */
    args[2].type   = Josh_ArgumentType_PASS_BY_REF;
    args[2].length = sizeof(Netfp_UserSRBInfo);

    /*  - Argument 4: */
    args[3].type   = Josh_ArgumentType_PASS_BY_REF;
    args[3].length = sizeof(int32_t);

    /* Add the arguments. */
    argHandle = Josh_addArguments (ptrNetfpClient->joshHandle, &args[0]);
    if (argHandle == NULL)
    {
        *errCode = NETFP_EJOSH;
        return -1;
    }

    /* Populate the NETFP Server handle */
    *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(ptrNetfpClient->cfg.serverHandle);

    /* Copy the User configuration: */
    ptrRemoteUserCfg = (Netfp_UserCfg*)args[1].argBuffer;
    memcpy ((void*)ptrRemoteUserCfg, (void *)ptrUserCfg, sizeof (Netfp_UserCfg));

    /* Copy the User SRB Info: */
    ptrRemoteUserSrbInfo = (Netfp_UserSRBInfo*)args[2].argBuffer;
    memcpy ((void*)ptrRemoteUserSrbInfo, (void *)ptrUserSrbInfo, sizeof (Netfp_UserSRBInfo));

    /* Submit the JOB to JOSH for execution. */
    jobId = Josh_submitJob(jobHandle, argHandle, &result, errCode);
    if (jobId < 0)
        return -1;

    /* Get the result arguments. */
    if (Josh_getArguments (ptrNetfpClient->joshHandle, jobId, &args[0]) < 0)
    {
        *errCode = NETFP_EINTERNAL;
        return -1;
    }

    /* Copy over the server security channel and software information. */
    memcpy ((void *)ptrUserSrbInfo, (void *)args[2].argBuffer, sizeof(Netfp_UserSRBInfo));

    /* Copy the error code value. */
    *errCode = *((int32_t*)args[3].argBuffer);

    /* Free the JOB Instance. */
    if (Josh_freeJobInstance(ptrNetfpClient->joshHandle, jobId) < 0)
    {
        *errCode = NETFP_EJOSH;
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to delete the SRB security channel in the SA LLD.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  ptrUserSrbInfo
 *      Pointer to the user SRB security channel information.
 *  @param[out]  errCode
 *      Error code populated on error
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t _Netfp_deleteSrb
(
    Netfp_ServerMCB*            ptrNetfpServer,
    Netfp_UserSRBInfo*          ptrUserSrbInfo,
    int32_t*                    errCode
)
{
    uint32_t                    index;

    for (index = 1; index < 3; index++)
    {
        /* Delete the SRB authentication channels */
        if (ptrUserSrbInfo->SRBAuthInfo[index].srvSecurityChannelHandle != NULL)
            if (_Netfp_deleteSecurityChannel (ptrNetfpServer, ptrUserSrbInfo->SRBAuthInfo[index].srvSecurityChannelHandle, errCode) < 0)
                return -1;

        /* Delete the SRB ciphering channels */
        if (ptrUserSrbInfo->SRBCipherInfo[index].srvSecurityChannelHandle != NULL)
            if (_Netfp_deleteSecurityChannel (ptrNetfpServer, ptrUserSrbInfo->SRBCipherInfo[index].srvSecurityChannelHandle, errCode) < 0)
                return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to delete the SRB security channel in the SA LLD.
 *
 *  @param[in]  ptrNetfpClient
 *      Pointer to the NETFP client
 *  @param[in]  ptrUserSrbInfo
 *      Pointer to the user SRB security channel information.
 *  @param[out]  errCode
 *      Error code populated on error
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_deleteSrb
(
    Netfp_ClientMCB*            ptrNetfpClient,
    Netfp_UserSRBInfo*          ptrUserSrbInfo,
    int32_t*                    errCode
)
{
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    uint32_t                result;
    int32_t                 jobId;
    Netfp_UserSRBInfo*      ptrRemoteUserSrbInfo;

    /* Get the actual job which we will execute */
    jobHandle = Josh_findJobByAddress(ptrNetfpClient->joshHandle, (Josh_JobProtype)_Netfp_deleteSrb);
    if (jobHandle == NULL)
    {
        *errCode = NETFP_EINTERNAL;
        return -1;
    }

    /* Initialize the arguments; to avoid any junk */
    memset ((void *)&args, 0, sizeof(args));

    /***************************************************************************
     * This is the function which is to be invoked:
     *
     *  int32_t _Netfp_deleteSrb
     *  (
     *  Netfp_ServerMCB*            ptrNetfpServer,
     *  Netfp_UserSRBInfo*          ptrUserSrbInfo,
     *  int32_t*                    errCode
     *  )
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Netfp_ServerHandle);

    /*  - Argument 2: */
    args[1].type   = Josh_ArgumentType_PASS_BY_REF;
    args[1].length = sizeof(Netfp_UserSRBInfo);

    /*  - Argument 3: */
    args[2].type   = Josh_ArgumentType_PASS_BY_REF;
    args[2].length = sizeof(int32_t);

    /* Add the arguments. */
    argHandle = Josh_addArguments (ptrNetfpClient->joshHandle, &args[0]);
    if (argHandle == NULL)
    {
        *errCode = NETFP_EJOSH;
        return -1;
    }

    /* Populate the arguments */
    *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(ptrNetfpClient->cfg.serverHandle);

    /* Copy the User SRB Info: */
    ptrRemoteUserSrbInfo = (Netfp_UserSRBInfo*)args[1].argBuffer;
    memcpy ((void*)ptrRemoteUserSrbInfo, (void *)ptrUserSrbInfo, sizeof (Netfp_UserSRBInfo));

    /* Submit the JOB to JOSH for execution. */
    jobId = Josh_submitJob(jobHandle, argHandle, &result, errCode);
    if (jobId < 0)
        return -1;

    /* Get the result arguments. */
    if (Josh_getArguments (ptrNetfpClient->joshHandle, jobId, &args[0]) < 0)
    {
        *errCode = NETFP_EINTERNAL;
        return -1;
    }

    /* Copy the error code value. */
    *errCode = *((int32_t*)args[2].argBuffer);

    /* Free the JOB Instance. */
    if (Josh_freeJobInstance(ptrNetfpClient->joshHandle, jobId) < 0)
    {
        *errCode = NETFP_EJOSH;
        return -1;
    }
    return (int32_t)result;
}

/**
 *  @b Description
 *  @n
 *      The function is used to create the channel in the SA LLD. The function
 *      supports channel creation for IPSEC and 3GPP.
 *
 *  @param[in]  ptrNetfpClient
 *      Pointer to the NETFP client
 *  @param[in]  ptrSecChannelCfg
 *      Pointer to the security channel configuration
 *  @param[out]  ptrSwInfo
 *      Pointer to the software information populated by the API
 *  @param[out]  errCode
 *      Error code populated on error
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   Server security channel handle
 *  @retval
 *      Error   -   NULL
 */
Netfp_SrvSecChannelHandle Netfp_createSecurityChannel
(
    Netfp_ClientMCB*        ptrNetfpClient,
    Netfp_SecChannelCfg*    ptrSecChannelCfg,
    Netfp_SecuritySwInfo*   ptrSwInfo,
    int32_t*                errCode
)
{
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    uint32_t                result;
    int32_t                 jobId;
    Netfp_SecChannelCfg*    ptrRemoteSecChannelCfg;

    /* Get the actual job which we will execute */
    jobHandle = Josh_findJobByAddress(ptrNetfpClient->joshHandle, (Josh_JobProtype)_Netfp_createSecurityChannel);
    if (jobHandle == NULL)
    {
        *errCode = NETFP_EINTERNAL;
        return NULL;
    }

    /* Initialize the arguments; to avoid any junk */
    memset ((void *)&args, 0, sizeof(args));

    /***************************************************************************
     * This is the function which is to be invoked:
     *
     *  Netfp_SrvSecChannelHandle _Netfp_createSecurityChannel
     *  (
     *  Netfp_ServerMCB*        ptrNetfpServer,
     *  Netfp_SecChannelCfg*    ptrSecChannelCfg,
     *  Netfp_SecuritySwInfo*   ptrSwInfo,
     *  int32_t*                errCode
     *  )
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Netfp_ServerHandle);

    /*  - Argument 2: */
    args[1].type   = Josh_ArgumentType_PASS_BY_REF;
    args[1].length = sizeof(Netfp_SecChannelCfg);

    /*  - Argument 3: */
    args[2].type   = Josh_ArgumentType_PASS_BY_REF;
    args[2].length = sizeof(Netfp_SecuritySwInfo);

    /*  - Argument 4: */
    args[3].type   = Josh_ArgumentType_PASS_BY_REF;
    args[3].length = sizeof(int32_t);

    /* Add the arguments. */
    argHandle = Josh_addArguments (ptrNetfpClient->joshHandle, &args[0]);
    if (argHandle == NULL)
    {
        *errCode = NETFP_EJOSH;
        return NULL;
    }

    /* Populate the NETFP Server handle */
    *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(ptrNetfpClient->cfg.serverHandle);

    /* Copy the security channel configuration: */
    ptrRemoteSecChannelCfg = (Netfp_SecChannelCfg*)args[1].argBuffer;
    memcpy ((void*)ptrRemoteSecChannelCfg, (void *)ptrSecChannelCfg, sizeof (Netfp_SecChannelCfg));

    /* Submit the JOB to JOSH for execution. */
    jobId = Josh_submitJob(jobHandle, argHandle, &result, errCode);
    if (jobId < 0)
        return NULL;

    /* Get the result arguments. */
    if (Josh_getArguments (ptrNetfpClient->joshHandle, jobId, &args[0]) < 0)
    {
        *errCode = NETFP_EINTERNAL;
        return NULL;
    }

    /* Copy over the software information. */
    memcpy ((void *)ptrSwInfo, (void *)args[2].argBuffer, sizeof(Netfp_SecuritySwInfo));

    /* Copy the error code value. */
    *errCode = *((int32_t*)args[3].argBuffer);

    /* Free the JOB Instance. */
    if (Josh_freeJobInstance(ptrNetfpClient->joshHandle, jobId) < 0)
    {
        *errCode = NETFP_EJOSH;
        return NULL;
    }
    return (Netfp_SrvSecChannelHandle)result;
}

/**
 *  @b Description
 *  @n
 *      The function is used to delete the security channel in the SA LLD.
 *
 *  @param[in]  ptrNetfpClient
 *      Pointer to the NETFP client
 *  @param[in]  srvSecurityChannel
 *      Handle to the security channel to be deleted
 *  @param[out]  errCode
 *      Error code populated on error
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_deleteSecurityChannel
(
    Netfp_ClientMCB*            ptrNetfpClient,
    Netfp_SrvSecChannelHandle   srvSecurityChannel,
    int32_t*                    errCode
)
{
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    uint32_t                result;
    int32_t                 jobId;

    /* Get the actual job which we will execute */
    jobHandle = Josh_findJobByAddress(ptrNetfpClient->joshHandle, (Josh_JobProtype)_Netfp_deleteSecurityChannel);
    if (jobHandle == NULL)
    {
        *errCode = NETFP_EINTERNAL;
        return -1;
    }

    /* Initialize the arguments; to avoid any junk */
    memset ((void *)&args, 0, sizeof(args));

    /***************************************************************************
     * This is the function which is to be invoked:
     *
     *  int32_t _Netfp_deleteSecurityChannel
     *  (
     *  Netfp_ServerMCB*            ptrNetfpServer,
     *  Netfp_SrvSecChannelHandle   srvSecurityChannel,
     *  int32_t*                    errCode
     *  )
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Netfp_ServerHandle);

    /*  - Argument 2: */
    args[1].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[1].length = sizeof(Netfp_SrvSecChannelHandle);

    /*  - Argument 3: */
    args[2].type   = Josh_ArgumentType_PASS_BY_REF;
    args[2].length = sizeof(int32_t);

    /* Add the arguments. */
    argHandle = Josh_addArguments (ptrNetfpClient->joshHandle, &args[0]);
    if (argHandle == NULL)
    {
        *errCode = NETFP_EJOSH;
        return -1;
    }

    /* Populate the arguments */
    *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(ptrNetfpClient->cfg.serverHandle);
    *(uint32_t*)args[1].argBuffer = (uint32_t)josh_toRemoteU32(srvSecurityChannel);

    /* Submit the JOB to JOSH for execution. */
    jobId = Josh_submitJob(jobHandle, argHandle, &result, errCode);
    if (jobId < 0)
        return -1;

    /* Get the result arguments. */
    if (Josh_getArguments (ptrNetfpClient->joshHandle, jobId, &args[0]) < 0)
    {
        *errCode = NETFP_EINTERNAL;
        return -1;
    }

    /* Copy the error code value. */
    *errCode = *((int32_t*)args[2].argBuffer);

    /* Free the JOB Instance. */
    if (Josh_freeJobInstance(ptrNetfpClient->joshHandle, jobId) < 0)
    {
        *errCode = NETFP_EJOSH;
        return -1;
    }
    return (int32_t)result;
}

/**
 *  @b Description
 *  @n
 *      The function is used to kill the security channels registered with the NETFP
 *      server.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success  -   0
 *  @retval
 *      Error    -   <0
 */
int32_t Netfp_killSecurityChannel
(
    Netfp_ServerMCB*    ptrNetfpServer,
    int32_t*            errCode
)
{
    Netfp_SrvSecurityChannel*   ptrSrvSecurityChannel;

    /* Close all the server security channels. */
    ptrSrvSecurityChannel = (Netfp_SrvSecurityChannel*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrServerSecurityChannelList);
    while (ptrSrvSecurityChannel != NULL)
    {
        /* Delete the security channel from the server. */
        if (_Netfp_deleteSecurityChannel  (ptrNetfpServer, ptrSrvSecurityChannel, errCode) < 0)
            return -1;

        /* Get the next server security channel */
        ptrSrvSecurityChannel = (Netfp_SrvSecurityChannel*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrServerSecurityChannelList);
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to configure the countC of a security channel
 *      to a specific value
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  ptrSrvSecurityChannel
 *      Pointer to the security channel for which the countC is to be configured
 *  @param[in]  countC
 *      New countC to be used
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success  -   0
 *  @retval
 *      Error    -   Error code populated by the API
 */
int32_t Netfp_configureCountC
(
    Netfp_ServerMCB*            ptrNetfpServer,
    Netfp_SrvSecurityChannel*   ptrSrvSecurityChannel,
    uint32_t                    countC
)
{
    Ti_Pkt*         ptrNullPacket;
    Qmss_QueueHnd   queueHandle;
    uint32_t        packetLen;
    uint32_t        psInfo[2];
    uint32_t        swInfo[3];
    uint32_t        flowId;

    /* Allocate a packet from the NETFP Server command heap. */
    ptrNullPacket = Pktlib_allocPacket(ptrSrvSecurityChannel->ptrNetfpServer->cfg.pktlibInstHandle,
                                       ptrSrvSecurityChannel->ptrNetfpServer->cfg.cmdHeapHandle, 64);
    if (ptrNullPacket == NULL)
        return NETFP_ENOMEM;

    /* We are using the PA command heap & flow for sending this packet. This should get recycled immediately. */
    queueHandle = Pktlib_getInternalHeapQueue (ptrSrvSecurityChannel->ptrNetfpServer->cfg.cmdHeapHandle);
    flowId      = Cppi_getFlowId(ptrNetfpServer->paRxFlowHandle);

    /* Get the packet length: */
    packetLen = Pktlib_getPacketLen (ptrNullPacket);

    /* Populate the short information in the packet: */
    psInfo[0] = PASAHO_SINFO_FORMAT_CMD(0, packetLen);
    psInfo[1] = countC - 1;
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)ptrNullPacket, (uint8_t  *)psInfo, 8);

    /* Populate the software information: */
    swInfo[0] = ptrSrvSecurityChannel->swInfo.rxInfo.swInfo[0];
    swInfo[1] = ptrSrvSecurityChannel->swInfo.rxInfo.swInfo[1];
    swInfo[2] = ptrSrvSecurityChannel->swInfo.rxInfo.swInfo[2];
    sa_SWINFO_UPDATE_DEST_INFO (swInfo, queueHandle, flowId);
    Cppi_setSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)ptrNullPacket, (uint8_t *)&swInfo);

    /******************************************************************************
     * NOTE: There is an ownership change here from the DSP to the CPDMA. We need
     * to ensure that the packet contents here are written back.
     ******************************************************************************/
	Pktlib_releaseOwnership(ptrSrvSecurityChannel->ptrNetfpServer->cfg.pktlibInstHandle, ptrNullPacket);

    /* Push the packet to the head of the Air Ciphering queue. */
    Qmss_queuePush (ptrNetfpServer->netcpTxQueue[NSS_SA_QUEUE_SASS2_INDEX], ptrNullPacket, packetLen, 128, Qmss_Location_HEAD);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the security channel information.
 *      Currently only CountC option is supported.
 *
 *  @param[in]  ptrNetfpClient
 *      Pointer to the NETFP client
 *  @param[in]  srvSecurityChannelHandle
 *      Handle to the security channel whose information is read
 *  @param[out] ptrOptInfo
 *      Option info in TLV format which is populated with the requested information
 *  @param[out]  errCode
 *      Error code populated on error
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_getSecurityChannelOpt
(
    Netfp_ClientMCB*            ptrNetfpClient,
    Netfp_SrvSecChannelHandle   srvSecurityChannelHandle,
    Netfp_OptionTLV*            ptrOptInfo,
    int32_t*                    errCode
)
{
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    uint32_t                result;
    int32_t                 jobId;
    Netfp_OptionTLV*        ptrRemoteOptInfo;

    /* Check the NETFP Client MCB: Services are available only if the client is active. */
    if (ptrNetfpClient->status != Netfp_ClientStatus_ACTIVE)
    {
        *errCode = NETFP_ENOTREADY;
        return -1;
    }

    /* Get the actual job which we will execute */
    jobHandle = Josh_findJobByAddress(ptrNetfpClient->joshHandle, (Josh_JobProtype)_Netfp_getSecurityChannelOpt);
    if (jobHandle == NULL)
    {
        *errCode = NETFP_EINTERNAL;
        return -1;
    }

    /* Initialize the arguments; to avoid any junk */
    memset ((void *)&args, 0, sizeof(args));

    /***************************************************************************
    * This is the function which is to be invoked:
    *
    *  int32_t _Netfp_getSecurityChannelOpt
    *  (
    *  Netfp_ServerMCB*            ptrNetfpServer,
    *  Netfp_SrvSecChannelHandle   srvSecurityChannelHandle,
    *  Netfp_OptionTLV*            ptrOptInfo,
    *  int32_t*                    errCode
    *  )
    ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
     args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
     args[0].length = sizeof(Netfp_ServerHandle);

     /*  - Argument 2: */
     args[1].type   = Josh_ArgumentType_PASS_BY_VALUE;
     args[1].length = sizeof(Netfp_SrvSecChannelHandle);

     /*  - Argument 3: We need to account for the length of the optional parameters */
     args[2].type   = Josh_ArgumentType_PASS_BY_REF;
     args[2].length = sizeof(Netfp_OptionTLV) + ptrOptInfo->length;

     /*  - Argument 4: */
    args[3].type   = Josh_ArgumentType_PASS_BY_REF;
    args[3].length = sizeof(int32_t);

    /* Add the arguments. */
    argHandle = Josh_addArguments (ptrNetfpClient->joshHandle, &args[0]);
    if (argHandle == NULL)
    {
        *errCode = NETFP_EJOSH;
        return -1;
    }

    /* Populate the NETFP Server handle */
    *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(ptrNetfpClient->cfg.serverHandle);

    /* Populate the server security channel handle. */
    *(uint32_t*)args[1].argBuffer = (uint32_t)josh_toRemoteU32(srvSecurityChannelHandle);

    /* Copy the option information
     * The value in the remote option information should point immediately after the
     * TLV data.   */
    ptrRemoteOptInfo = (Netfp_OptionTLV*)args[2].argBuffer;
    memcpy ((void*)ptrRemoteOptInfo, (void *)ptrOptInfo, sizeof(Netfp_OptionTLV));
    ptrRemoteOptInfo->value = (void*)((uint32_t)args[2].argBuffer + sizeof(Netfp_OptionTLV));
    memcpy ((void*)ptrRemoteOptInfo->value, (void *)ptrOptInfo->value, ptrOptInfo->length);

    /* Submit the JOB to JOSH for execution. */
    jobId = Josh_submitJob(jobHandle, argHandle, &result, errCode);
    if (jobId < 0)
        return -1;

    /* Get the result arguments. */
    if (Josh_getArguments (ptrNetfpClient->joshHandle, jobId, &args[0]) < 0)
    {
        *errCode = NETFP_EINTERNAL;
        return -1;
    }

    /* Copy the error code value. */
    *errCode = *((int32_t*)args[3].argBuffer);

    /* Copy the option information back from the response. The remote
     * option "value" resides immediately after the TLV data structure. */
     ptrRemoteOptInfo = (Netfp_OptionTLV*)args[2].argBuffer;
     ptrRemoteOptInfo->value = (void*)((uint32_t)args[2].argBuffer + sizeof(Netfp_OptionTLV));
     memcpy ((void*)ptrOptInfo->value, (void *)ptrRemoteOptInfo->value, ptrRemoteOptInfo->length);

    /* Free the JOB Instance. */
    if (Josh_freeJobInstance(ptrNetfpClient->joshHandle, jobId) < 0)
    {
        *errCode = NETFP_EJOSH;
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is a debug function to display the contents of the security contexts
 *      which have been allocated.
 *
 *  @param[in]  serverHandle
 *      Handle to the NETFP Server
 *
 *  \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Number of allocated security context
 */
int32_t Netfp_displaySecurityContext(Netfp_ServerHandle serverHandle)
{
    Netfp_SecurityContext*      ptrGarbageSecurityContext;
    Netfp_SrvSecurityChannel*   ptrSrvSecurityChannel;
    Netfp_ServerMCB*            ptrNetfpServer;
    Sa_SysStats_t               saSysStats;
    Sa_Stats_t                  saStats;
    int16_t                     errCode;
    int32_t                     count = 0;
    int32_t                     gcSecurityContextCount = 1;

    /* Get the server handle. */
    ptrNetfpServer = (Netfp_ServerMCB*)serverHandle;
    if (ptrNetfpServer == NULL)
        return 0;

    /* Initialize the SA System statistics: */
    memset ((void *)&saSysStats, 0, sizeof(Sa_SysStats_t));

    /* Get the SA System statistics: */
    errCode = Sa_getSysStats (ptrNetfpServer->saHandle, &saSysStats);
    if (errCode != sa_ERR_OK)
    {
        Netfp_logMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "Error: Getting SA System statistics failed\n");
        return 0;
    }
//fzm - use Netfp_dumpMsg to capture this with the USR1 signal
    /* Display the banner. */
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "**********************************************************\n");
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "SA System Statistics:\n");

    /* SA Error System statistics: */
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "No PDSP Memory               : %d\n", saSysStats.err.errNoMem);
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "Security Context Error       : %d\n", saSysStats.err.errCtx);
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "Security Context Engine Error: %d\n", saSysStats.err.errEngine);
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "Protocol Error               : %d\n", saSysStats.err.errProto);

    /* SA IPSEC ESP System statistics: */
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "Replay Failures with Old     : %d\n", saSysStats.esp.replayOld);
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "Replay Failures with Dup     : %d\n", saSysStats.esp.replayDup);
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "Authentication Failures      : %d\n", saSysStats.esp.authFail);
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "Packets encrypted [High]     : %d\n", saSysStats.esp.pktEncHi);
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "Packets encrypted [Low]      : %d\n", saSysStats.esp.pktEncLo);
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "Packets decrypted [High]     : %d\n", saSysStats.esp.pktDecHi);
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "Packets decrypted [Low]      : %d\n", saSysStats.esp.pktDecLo);

    /* SA Air Ciphering System statistics: */
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "Authentication Failures      : %d\n", saSysStats.ac.authFail);
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "To Air [High]                : %d\n", saSysStats.ac.pktToAirHi);
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "To Air [Low]                 : %d\n", saSysStats.ac.pktToAirLo);
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "From Air [High]              : %d\n", saSysStats.ac.pktFromAirHi);
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "From Air [Low]               : %d\n", saSysStats.ac.pktFromAirLo);

    /* Display the banner. */
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "**********************************************************\n");
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "Security Channels\n");

    /* Get the allocated security channel. */
    ptrSrvSecurityChannel = (Netfp_SrvSecurityChannel*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrServerSecurityChannelList);
    while (ptrSrvSecurityChannel != NULL)
    {
        Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,
                      "Security Channel 0x%x Protocol: %s\n", ptrSrvSecurityChannel,
                      (ptrSrvSecurityChannel->netfpProtocol == Netfp_SaProtocol_IPSEC) ? "IPSEC" : "Air Ciphering");
        Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,
                      "           Context 0 Id: %d Address: 0x%x Size %d bytes Ownership: %s\n",
                      ptrSrvSecurityChannel->secContext[0].secContextId , ptrSrvSecurityChannel->secContext[0].ptrSecurityContext,
                      ptrSrvSecurityChannel->secContext[0].sizeSecurityContext,
                      ptrSrvSecurityChannel->secContext[0].ptrSecurityContext==NULL?"Not Used":
                      (Sa_isScBufFree (ptrSrvSecurityChannel->secContext[0].ptrSecurityContext) == 1) ? "Host Owned" : "SA Owned");
        Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,
                      "           Context 1 Id: %d Address: 0x%x Size %d bytes Ownership: %s\n",
                      ptrSrvSecurityChannel->secContext[1].secContextId , ptrSrvSecurityChannel->secContext[1].ptrSecurityContext,
                      ptrSrvSecurityChannel->secContext[1].sizeSecurityContext,
                      ptrSrvSecurityChannel->secContext[1].ptrSecurityContext==NULL?"Not Used":
                      (Sa_isScBufFree (ptrSrvSecurityChannel->secContext[1].ptrSecurityContext) == 1) ? "Host Owned" : "SA Owned");

        /* Get the SA Statistics from the SA */
        errCode = Netfp_getSAStatistics (ptrSrvSecurityChannel->saChannelHandle, &saStats);
        if (errCode < 0)
        {
            /* Error: Unable to get the SA Channel statistics: */
            Netfp_logMsg (ptrNetfpServer, Netfp_LogLevel_ERROR,
                          "SA Channel 0x%x getStats failed [Error code %d]\n", ptrSrvSecurityChannel->saChannelHandle, errCode);

            /* Get the next security channel */
            ptrSrvSecurityChannel = (Netfp_SrvSecurityChannel*)Netfp_listGetNext ((Netfp_ListNode*)ptrSrvSecurityChannel);
            continue;
        }

        /* Display the statistics: */
        if (ptrSrvSecurityChannel->netfpProtocol == Netfp_SaProtocol_IPSEC)
        {
            /* IPSEC Channel: */
            Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,
                          "           replayOld:      %d\n", saStats.ipsec.replayOld);
            Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,
                          "           replayDup:      %d\n", saStats.ipsec.replayDup);
            Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,
                          "           authFail:       %d\n", saStats.ipsec.authFail);
            Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,
                          "           txRollover:     %d\n", saStats.ipsec.txRollover);
            Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,
                          "           txESN:          %d\n", saStats.ipsec.txESN);
            Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,
                          "           txSN:           %d\n", saStats.ipsec.txSN);
            Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,
                          "           rxESN:          %d\n", saStats.ipsec.rxESN);
            Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,
                          "           pktEncHi:       %d\n", saStats.ipsec.pktEncHi);
            Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,
                          "           pktEncLo:        %d\n", saStats.ipsec.pktEncLo);
            Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,
                          "           pktDecHi:        %d\n", saStats.ipsec.pktDecHi);
            Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,
                          "           pktDecLo:        %d\n", saStats.ipsec.pktDecLo);
            Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,
                          "           txByteCountHi:   %d\n", saStats.ipsec.txByteCountHi);
            Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,
                          "           txByteCountLo:   %d\n", saStats.ipsec.txByteCountLo);
            Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,
                          "           rxByteCountHi:   %d\n", saStats.ipsec.rxByteCountHi);
            Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,
                          "           rxByteCountLo:   %d\n", saStats.ipsec.rxByteCountLo);
        }
        else
        {
            /* AIR Ciphering Channel: */
            Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,
                          "           pktToAirHi:      %d\n", saStats.ac.pktToAirHi);
            Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,
                          "           pktToAirLo:      %d\n", saStats.ac.pktToAirLo);
            Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,
                          "           pktFromAirHi:    %d\n", saStats.ac.pktFromAirHi);
            Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,
                          "           pktFromAirLo:    %d\n", saStats.ac.pktFromAirLo);
            Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,
                          "           authFail:        %d\n", saStats.ac.authFail);
            Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,
                          "           toAirCountC:     %d\n", saStats.ac.toAirCountC);
            Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,
                          "           fromAirCountC:   %d\n", saStats.ac.fromAirCountC);
        }

        /* Increment the counter */
        count++;

        /* Get the next security channel */
        ptrSrvSecurityChannel = (Netfp_SrvSecurityChannel*)Netfp_listGetNext ((Netfp_ListNode*)ptrSrvSecurityChannel);
    }

    /* Display the banner. */
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "**********************************************************\n");
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "Garbage Security Contexts\n");

    /* Cycle through all the entries in the garbage collection */
    ptrGarbageSecurityContext = (Netfp_SecurityContext*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrGarbageSecurityContextList);
    while (ptrGarbageSecurityContext != NULL)
    {
        /* Debug Message: */
        Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "%d. Protocol %s Id: %d %p size %d bytes Ownership:%s\n",
                      gcSecurityContextCount++,
                      (ptrGarbageSecurityContext->netfpProtocol == Netfp_SaProtocol_IPSEC) ? "IPSEC" : "Air Ciphering",
                      ptrGarbageSecurityContext->secContextId, ptrGarbageSecurityContext->ptrSecurityContext,
                      ptrGarbageSecurityContext->sizeSecurityContext,
                      (Sa_isScBufFree (ptrGarbageSecurityContext->ptrSecurityContext) == 1) ? "Host Owned" : "SA Owned");

        /* Increment the counter */
        count++;

        /* Get the next entry: */
        ptrGarbageSecurityContext = (Netfp_SecurityContext*)Netfp_listGetNext ((Netfp_ListNode*)ptrGarbageSecurityContext);
    }

    /* Display the banner. */
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "**********************************************************\n");
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "Slow Cleanup Security Contexts\n");

    /* Get the allocated security channel. */
    ptrSrvSecurityChannel = (Netfp_SrvSecurityChannel*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrSlowCleanupSecurityContextList);
    while (ptrSrvSecurityChannel != NULL)
    {
        Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,
                      "Security Channel 0x%x Protocol: %s\n", ptrSrvSecurityChannel,
                      (ptrSrvSecurityChannel->netfpProtocol == Netfp_SaProtocol_IPSEC) ? "IPSEC" : "Air Ciphering");
        Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,
                      "           Context 0 Id: %d Address: 0x%x Size %d bytes Ownership: %s\n",
                      ptrSrvSecurityChannel->secContext[0].secContextId , ptrSrvSecurityChannel->secContext[0].ptrSecurityContext,
                      ptrSrvSecurityChannel->secContext[0].sizeSecurityContext,
                      ptrSrvSecurityChannel->secContext[0].ptrSecurityContext == NULL ? "Not Used" :
                      (Sa_isScBufFree (ptrSrvSecurityChannel->secContext[0].ptrSecurityContext) == 1) ? "Host Owned" : "SA Owned");
        Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,
                      "           Context 1 Id: %d Address: 0x%x Size %d bytes Ownership: %s\n",
                      ptrSrvSecurityChannel->secContext[1].secContextId , ptrSrvSecurityChannel->secContext[1].ptrSecurityContext,
                      ptrSrvSecurityChannel->secContext[1].sizeSecurityContext,
                      ptrSrvSecurityChannel->secContext[1].ptrSecurityContext == NULL ? "Not Used" :
                      (Sa_isScBufFree (ptrSrvSecurityChannel->secContext[1].ptrSecurityContext) == 1) ? "Host Owned" : "SA Owned");

        /* Increment the counter */
        count++;

        /* Get the next security channel */
        ptrSrvSecurityChannel = (Netfp_SrvSecurityChannel*)Netfp_listGetNext ((Netfp_ListNode*)ptrSrvSecurityChannel);
    }

    return count;
}

/**
 *  @b Description
 *  @n
 *      The function is used to register the NETFP SA services
 *
 *  @param[in]  nodeHandle
 *      JOSH node handle for which the jobs are being registered
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_registerSAServices (Josh_NodeHandle nodeHandle)
{
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Netfp_createSecurityChannel);
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Netfp_deleteSecurityChannel);
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Netfp_getSecurityChannelOpt);
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Netfp_createSrb);
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Netfp_deleteSrb);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get Authetication IV size needed by cipher.
 *      Return value includes all the fixed sizes. Padding is not included.
 *
 *  @param[in]  ptrIPSecChannel
 *      Pointer to Ipsec Channel
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   IV size
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_getEncIvSize(Netfp_IPSecChannel* ptrIPSecChannel)
{
   int32_t retVal = 0;

   /* Bytes allocated for ESP auth data, equals to digest size */
   switch (ptrIPSecChannel->saCfg.ipsecCfg.encMode)
    {
		case Netfp_IpsecCipherMode_NULL:
		{
			retVal = 0; /* 0 bits */
			break;
		}
		case Netfp_IpsecCipherMode_AES_CBC:
		{
			retVal = 16; /* 128 bits */
			break;
		}
		case Netfp_IpsecCipherMode_3DES_CBC:
		{
			retVal = 8; /* 64 bits */
			break;
		}
		case Netfp_IpsecCipherMode_DES_CBC:
		{
			retVal = 8; /* 64 bits */
			break;
		}
		case Netfp_IpsecCipherMode_AES_CTR:
		{
			retVal = 8; /* 64 bits */
			break;
		}
		case Netfp_IpsecCipherMode_AES_GCM:
		{
			retVal = 8; /* 64 bits */
			break;
		}
        default:
        {
            retVal = -1;
        }
    }

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get Authetication output size needed by cipher.
 *      Return value includes all the fixed sizes. Padding is not included.
 *
 *  @param[in]  ptrIPSecChannel
 *      Pointer to Ipsec Channel
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   Authentication digest size
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_getAuthDigestSize(Netfp_IPSecChannel* ptrIPSecChannel)
{
   int32_t retVal = 0;

    /* Bytes allocated for ESP auth data, equals to digest size */
   switch (ptrIPSecChannel->saCfg.ipsecCfg.authMode)
    {
        case Netfp_IpsecAuthMode_NULL:
        {
			retVal = 0; /* 0 bits */
            break;
        }
        case Netfp_IpsecAuthMode_HMAC_SHA1:
        case Netfp_IpsecAuthMode_HMAC_MD5:
        case Netfp_IpsecAuthMode_AES_XCBC:
        case Netfp_IpsecAuthMode_HMAC_SHA2_256:
        case Netfp_IpsecAuthMode_AES_GMAC:
        {
           retVal = ptrIPSecChannel->saCfg.ipsecCfg.keyMacSize;
           break;
        }
        default:
        {
            retVal = -1;
        }
    }

    return retVal;
}


