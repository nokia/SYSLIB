/**
 *   @file  msgcom.c
 *
 *   @brief
 *      The file implements the message communication library.
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
#include <stddef.h>
#include <string.h>

/* SYSLIB Include files. */
#include <ti/runtime/name/name_db.h>
#include <ti/runtime/msgcom/msgcom.h>
#include <ti/runtime/msgcom/include/msgcom_internal.h>

/* For Debugging only. */
#if defined(__ARMv7)
#include <ti/apps/netfp_config/include/NetFP_System_printf.h> //fzm
#else
#include <xdc/runtime/System.h>
#endif

/**********************************************************************
 ************************** Global Variables **************************
 **********************************************************************/

/**
 * @brief Global Channel Interface Table which has a list of all the supported
 * channel types.
 */
Msgcom_ChannelInterface gChannelInterface[] =
{
    /* Queue Channel */
    {
        Msgcom_queueCreateReader,
        Msgcom_queueCreateVirtualReader,
        Msgcom_queueCreateWriter,
        Msgcom_queuePut,
        Msgcom_queueGet,
        Msgcom_queueRxHandler,
        Msgcom_queueDelete,
        Msgcom_queueGetInternalMsgQueueInfo
    },
    /* Queue-DMA Channel */
    {
        Msgcom_queueDMACreateReader,
        Msgcom_queueDMACreateVirtualReader,
        Msgcom_queueDMACreateWriter,
        Msgcom_queueDMAPut,
        Msgcom_queueDMAGet,
        Msgcom_queueDMARxHandler,
        Msgcom_queueDMADelete,
        Msgcom_queueDMAGetInternalMsgQueueInfo
    },
};

/**********************************************************************
 ****************** MSGCOM Device Specific Functionality **************
 **********************************************************************/

#ifdef DEVICE_K2H
/**
 *  @b Description
 *  @n
 *      The function is the device interface which lists the number of QMSS
 *      CPPI instances for the device. In the case of the K2H there are 2
 *      CPDMA instances. Please refer to the K2H specification.
 *
 *  @param[out]  ptrCPDMAList
 *      Pointer to the CPDMA instances which are available for the device.
 *  @param[out]  ptrQueueType
 *      Pointer to the QMSS Queue Types which are available for the device
 *  @param[out]  ptrQmssBase
 *      Pointer to the QMSS Queue bases which are available for the device
 *
 * \ingroup MSG_COM_INTERNAL_FUNCTION
 *
 *  @retval
 *      Number of CPDMA Instances
 */
static int32_t Msgcom_getNumQMSSCPDMAInstances
(
    Cppi_CpDma*     ptrCPDMAList,
    Qmss_QueueType* ptrQueueType,
    uint32_t*       ptrQmssBase
)
{
    /* Populate the list with the CPDMA instance information: */
    *ptrCPDMAList       = Cppi_CpDma_QMSS_CPDMA;
    *(ptrCPDMAList + 1) = Cppi_CpDma_QMSS_QM2_CPDMA;

    /* Populate the list with the QMSS Queue types. */
    *ptrQueueType       = Qmss_QueueType_INFRASTRUCTURE_QUEUE;
    *(ptrQueueType + 1) = Qmss_QueueType_QM2_INFRASTRUCTURE_QUEUE;

    /* Populate the QMSS Base queues */
    *ptrQmssBase        = QMSS_QM1_INFRASTRUCTURE_DMA_QUEUE_BASE;
    *(ptrQmssBase + 1)  = QMSS_QM2_INFRASTRUCTURE_DMA_QUEUE_BASE;

    /* Return the number of CPDMA Instances. */
    return 2;
}
#elif defined (DEVICE_K2K)
/**
 *  @b Description
 *  @n
 *      The function is the device interface which lists the number of QMSS
 *      CPPI instances for the device. In the case of the K2K there are 2
 *      CPDMA instances. Please refer to the K2K specification.
 *
 *  @param[out]  ptrCPDMAList
 *      Pointer to the CPDMA instances which are available for the device.
 *  @param[out]  ptrQueueType
 *      Pointer to the QMSS Queue Types which are available for the device
 *  @param[out]  ptrQmssBase
 *      Pointer to the QMSS Queue bases which are available for the device
 *
 * \ingroup MSG_COM_INTERNAL_FUNCTION
 *
 *  @retval
 *      Number of CPDMA Instances
 */
static int32_t Msgcom_getNumQMSSCPDMAInstances
(
    Cppi_CpDma*     ptrCPDMAList,
    Qmss_QueueType* ptrQueueType,
    uint32_t*       ptrQmssBase
)
{
    /* Populate the list with the CPDMA instance information: */
    *ptrCPDMAList       = Cppi_CpDma_QMSS_CPDMA;
    *(ptrCPDMAList + 1) = Cppi_CpDma_QMSS_QM2_CPDMA;

    /* Populate the list with the QMSS Queue types. */
    *ptrQueueType       = Qmss_QueueType_INFRASTRUCTURE_QUEUE;
    *(ptrQueueType + 1) = Qmss_QueueType_QM2_INFRASTRUCTURE_QUEUE;

    /* Populate the QMSS Base queues */
    *ptrQmssBase        = QMSS_QM1_INFRASTRUCTURE_DMA_QUEUE_BASE;
    *(ptrQmssBase + 1)  = QMSS_QM2_INFRASTRUCTURE_DMA_QUEUE_BASE;

    /* Return the number of CPDMA Instances. */
    return 2;
}
#elif defined (DEVICE_K2L)
/**
 *  @b Description
 *  @n
 *      The function is the device interface which lists the number of QMSS
 *      CPPI instances for the device. In the case of the K2L there is 1
 *      CPDMA instances. Please refer to the K2L specification.
 *
 *  @param[out]  ptrCPDMAList
 *      Pointer to the CPDMA instances which are available for the device.
 *  @param[out]  ptrQueueType
 *      Pointer to the QMSS Queue Types which are available for the device
 *  @param[out]  ptrQmssBase
 *      Pointer to the QMSS Queue bases which are available for the device
 *
 * \ingroup MSG_COM_INTERNAL_FUNCTION
 *
 *  @retval
 *      Number of CPDMA Instances
 */
static int32_t Msgcom_getNumQMSSCPDMAInstances
(
    Cppi_CpDma*     ptrCPDMAList,
    Qmss_QueueType* ptrQueueType,
    uint32_t*       ptrQmssBase
)
{
    /* Populate the list with the CPDMA instance information: */
    *ptrCPDMAList       = Cppi_CpDma_QMSS_CPDMA;

    /* Populate the list with the QMSS Queue types. */
    *ptrQueueType       = Qmss_QueueType_INFRASTRUCTURE_QUEUE;

    /* Populate the QMSS Base queues */
    *ptrQmssBase        = QMSS_QMSS_PKTDMA_QUEUE_BASE;

    /* Return the number of CPDMA Instances. */
    return 1;
}
#else
#error "MSGCOM is not ported for the selected Device. "
#endif

/**********************************************************************
 *************************** MSGCOM Functions *************************
 **********************************************************************/

/**
 *  @b Description
 *  @n
 *      The function is used to create the MSGCOM channel with the
 *      specified configuration.
 *
 *  @param[in]  channelName
 *      Name of the channel.
 *  @param[in]  channelType
 *      Type of the channel.
 *  @param[in]  ptrChannelConfig
 *      Channel Type specific configuration.
 *  @param[out]  errCode
 *      Error code populated if there was an error.
 *
 * \ingroup MSG_COM_FUNCTION
 *
 *  @retval
 *      Success   - Valid Channel Handle.
 *  @retval
 *      Error     - NULL (refer to the error code for more details)
 */
MsgCom_ChHandle Msgcom_create
(
    const char*             channelName,
    Msgcom_ChannelType      channelType,
    Msgcom_ChannelCfg*      ptrChannelConfig,
    int32_t*                errCode
)
{
    Msgcom_channel*         ptrChannel;
    Msgcom_channel*         ptrMsgcomPhyChannel;
    Name_ResourceCfg        namedResourceCfg;
    Msgcom_InstanceMCB*     ptrInstanceMCB;

    /* Sanity Check: Make sure we have a valid name and configuration passed */
    if ((channelName == NULL) || (ptrChannelConfig == NULL))
    {
        *errCode = MSGCOM_INVALID_PARAM;
        return NULL;
    }

    /* Get the MSGCOM Instance information. */
    ptrInstanceMCB = (Msgcom_InstanceMCB*)ptrChannelConfig->msgcomInstHandle;
    if (ptrInstanceMCB == NULL)
    {
        *errCode = MSGCOM_INVALID_PARAM;
        return NULL;
    }

    /* Search for duplicate channel name in the resource manager: Name space needs
     * to be unique in the system. */
    if (Name_findResource (ptrInstanceMCB->cfg.databaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB,
                           channelName, &namedResourceCfg, errCode) == 0)
    {
        /* The channel name already exists in the database. */
        *errCode = MSGCOM_CHANNEL_ALREADY_EXISTS;
        return NULL;
    }

    /* Resource Manager lookup failed. If the failure is because of any other reason besides
     * the resource not existing report this to the callee. */
    if (*errCode != NAME_ENOTFOUND)
        return NULL;

    /* Failure was because the channel name did not exist. We can now go ahead and create
     * the channel: Allocate memory for the channel block */
    ptrChannel = ptrInstanceMCB->cfg.malloc(Msgcom_MemAllocMode_INTERNAL, sizeof(Msgcom_channel));
    if (ptrChannel == NULL)
    {
        *errCode = MSGCOM_MALLOC_FAILED;
        return NULL;
    }

    /* Initialize the allocated block of memory. */
    memset ((void *)ptrChannel, 0, sizeof(Msgcom_channel));

    /* MSGCOM channel knows the instance to which it belongs. */
    ptrChannel->ptrInstanceMCB = ptrInstanceMCB;

    /* Is this a VIRTUAL or PHYSICAL Channel? */
    if (channelType == Msgcom_ChannelType_VIRTUAL)
    {
        /* VIRTUAL Channel: Get the PHYSICAL channel Handle from the specified configuration */
        ptrMsgcomPhyChannel = (Msgcom_channel*)ptrChannelConfig->u.virtualChannelCfg.phyChannel;

        /* Sanity Check: Ensure that we are operating on a physical channel. */
        if (ptrMsgcomPhyChannel->isVirtualChannel == 1)
        {
            *errCode = MSGCOM_VIRTUAL_CHANNEL_NOT_SUPPORTED;
            return NULL;
        }

        /* Sanity Check: All channel types need not support virtual channels. */
        if (gChannelInterface[ptrMsgcomPhyChannel->channelType].createVirtualReader == NULL)
        {
            *errCode = MSGCOM_VIRTUAL_CHANNEL_NOT_SUPPORTED;
            return NULL;
        }

        /* Sanity Check: Make sure the mode for the PHYSICAL and VIRTUAL channels are the same
         * If not then we cannot proceed. Because the PHYSICAL channel mode always wins. */
        if (ptrMsgcomPhyChannel->cfg.mode != ptrChannelConfig->mode)
        {
            *errCode = MSGCOM_INVALID_PARAM;
            return NULL;
        }

        /* This is a VIRTUAL Channel; so set the fields appropriately. */
        ptrChannel->isVirtualChannel = 1;
        ptrChannel->isReader         = 1;
        ptrChannel->channelType      = ptrMsgcomPhyChannel->channelType;
        memcpy((void *)&ptrChannel->cfg, (void *)ptrChannelConfig, sizeof(Msgcom_ChannelCfg));
        strncpy(ptrChannel->channelName, channelName, MSG_COM_MAX_CHANNEL_LEN);

        /* Create the virtual channel using the channel interface table. */
        *errCode = gChannelInterface[ptrChannel->channelType].createVirtualReader(channelName, ptrChannel, ptrMsgcomPhyChannel);
        if (*errCode < 0)
            return NULL;
    }
    else
    {
        /* PHYSICAL Channel: Populate the channel block */
        ptrChannel->channelType      = channelType;
        ptrChannel->isVirtualChannel = 0;
        ptrChannel->isReader         = 1;
        strncpy(ptrChannel->channelName, channelName, MSG_COM_MAX_CHANNEL_LEN);

        /* Create the channel using the channel interface table */
        *errCode = gChannelInterface[ptrChannel->channelType].createReader(channelName, ptrChannel, ptrChannelConfig);
        if (*errCode < 0)
            return NULL;
    }
    /* Return the MSGCOM channel handle. */
    return (MsgCom_ChHandle)ptrChannel;
}

/**
 *  @b Description
 *  @n
 *      The function is used to find a message communicator channel with the
 *      specified name.
 *
 *  @param[in]  msgcomInstHandle
 *      MSGCOM instance handle
 *  @param[in]  channelName
 *      Name of the channel.
 *  @param[out] errCode
 *      Error code populated on error
 *
 * \ingroup MSG_COM_FUNCTION
 *
 *  @retval
 *      Success     - Valid Channel Handle.
 *  @retval
 *      Error       - NULL
 */
MsgCom_ChHandle Msgcom_find
(
    Msgcom_InstHandle   msgcomInstHandle,
    const char*         channelName,
    int32_t*            errCode
)
{
    Msgcom_channel*         ptrChannel;
    Msgcom_InstanceMCB*     ptrInstanceMCB;
    Name_ResourceCfg        namedResourceCfg;
    uint32_t                channelType;

    /* Sanity Check: Make sure we have a valid name */
    if (channelName == NULL)
    {
        *errCode = MSGCOM_INVALID_PARAM;
        return NULL;
    }

    /* Get the MSGCOM Instance information. */
    ptrInstanceMCB = (Msgcom_InstanceMCB*)msgcomInstHandle;
    if (ptrInstanceMCB == NULL)
    {
        *errCode = MSGCOM_INVALID_PARAM;
        return NULL;
    }

    /* Search for duplicate channel name in the named resource instance handle. The name
     * space for MSGCOM channels needs to be unique in the each named resource instance. */
    if (Name_findResource (ptrInstanceMCB->cfg.databaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB,
                           channelName, &namedResourceCfg, errCode) < 0)
    {
        /*  Error: The named resource lookup failed. */
        return NULL;
    }

    /* Allocate memory for the MSGCOM channel. */
    ptrChannel = ptrInstanceMCB->cfg.malloc(Msgcom_MemAllocMode_INTERNAL, sizeof(Msgcom_channel));
    if (ptrChannel == NULL)
    {
        *errCode = MSGCOM_MALLOC_FAILED;
        return NULL;
    }

    /* Initialize the allocated block of memory */
    memset ((void *)ptrChannel, 0, sizeof(Msgcom_channel));

    /* MSGCOM channel knows the instance to which it belongs. */
    ptrChannel->ptrInstanceMCB = ptrInstanceMCB;

    /* Populate the channel block. */
    strncpy (ptrChannel->channelName, channelName, MSG_COM_MAX_CHANNEL_LEN);
    ptrChannel->isReader = 0;

    /* Channel exists in the named resource domain: Get the channel type from the named resource block
     * - All channel types should store this in the first handle. */
    channelType = namedResourceCfg.handle1;

    /* Create the channel writer: */
    if (gChannelInterface[channelType].createWriter(channelName, ptrChannel, &namedResourceCfg, errCode) < 0)
        return NULL;

    /* Return the channel handle. */
    return ptrChannel;
}

/**
 *  @b Description
 *  @n
 *      The function is used to delete a message communicator channel.
 *      Please ensure that the following criteria are met:-
 *
 *      @verbatim
           a) Each core (reader or writer) is responsible for deleting the channel
              which it has created (reader) or which was found (writer)
           b) If the reader core has deleted the channel it is the duty of the
              application to ensure that the MSGCOM channel is NOT used across
              the rest of the SYSTEM. The MSGCOM library does not perform runtime
              checking to determine if the channel is UP or not since the cache
              operations are costly and this will impact performance.
        @endverbatim
 *
 *  @param[in]  msgChHandle
 *      Handle to the channel which is to be deleted
 *  @param[in]  freePkt
 *      Application registered call back API to cleanup any pending messages
 *
 * \ingroup MSG_COM_FUNCTION
 *
 *  @retval
 *      Success     - 0.
 *  @retval
 *      Error       - <0    [See error code]
 */
int32_t Msgcom_delete(MsgCom_ChHandle msgChHandle, Msgcom_freePkt freePkt)
{
    Msgcom_channel*     ptrChannel;
    int32_t             errCode;
    Msgcom_InstanceMCB* ptrInstanceMCB;

    /* Get the channel information. */
    ptrChannel = (Msgcom_channel*)msgChHandle;

    /* Sanity Check: Validate the arguments. */
    if ((ptrChannel == NULL) || (freePkt == NULL))
        return MSGCOM_INVALID_PARAM;

    /* MSGCOM channel knows the instance to which it belongs. */
    ptrInstanceMCB = ptrChannel->ptrInstanceMCB;
    if (ptrInstanceMCB == NULL)
        return MSGCOM_INVALID_PARAM;

    /* Are we trying to delete the reader or writer channel? */
    if (ptrChannel->isReader == 1)
    {
        /* Reader Channel: Invoke the channel interface registered 'delete' API */
        errCode = gChannelInterface[ptrChannel->channelType].delete(ptrChannel, freePkt);
        if (errCode < 0)
            return errCode;

        /* Reader channels are also responsible for cleaning the named resource database */
        if (Name_deleteResource (ptrInstanceMCB->cfg.databaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB,
                                 ptrChannel->channelName, &errCode) < 0)
            return errCode;
    }

    /* Cleanup the allocated channel memory (Reader and Writer channels now have seperate channel blocks */
    ptrInstanceMCB->cfg.free(Msgcom_MemAllocMode_INTERNAL, ptrChannel, sizeof(Msgcom_channel));
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to put a message to the specified channel.
 *
 *  @param[in]  msgChHandle
 *      Handle to the channel over which the message is to be sent.
 *  @param[in]  msgBuffer
 *      Message Buffer which is to be sent.
 *
 * \ingroup MSG_COM_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t Msgcom_putMessage
(
    MsgCom_ChHandle         msgChHandle,
    MsgCom_Buffer*          msgBuffer
)
{
    Msgcom_channel*     ptrChannel;

    /* Get the channel information. */
    ptrChannel = (Msgcom_channel*)msgChHandle;

    /* Debug Message: Only writer channels can be used to send a message. This warning error messages is
     * printed to ensure that applications use the correct channels. */
    if (ptrChannel->isReader == 1)
    {
        System_printf ("Error: Using a reader MSGCOM channel '%s' to put message\n", ptrChannel->channelName);
        return -1;
    }

    /* Invoke the channel interface registered 'put' API */
    return gChannelInterface[ptrChannel->channelType].put(ptrChannel, msgBuffer);
}

/**
 *  @b Description
 *  @n
 *      The function is used to get a message from the specified channel. If
 *      the channel was BLOCKING the function will block until data was received.
 *
 *  @param[in]  msgChHandle
 *      Handle to the channel over which the message is to be received.
 *  @param[out]  msgBuffer
 *      Message Buffer which was received.
 *
 * \ingroup MSG_COM_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t Msgcom_getMessage
(
    MsgCom_ChHandle         msgChHandle,
    MsgCom_Buffer**         msgBuffer
)
{
    Msgcom_channel*     ptrChannel;

    /* Get the channel information. */
    ptrChannel = (Msgcom_channel*)msgChHandle;

    /* Debug Message: Only reader channels can be used to read a message. This warning error messages is
     * printed to ensure that applications use the correct channels. */
    if (ptrChannel->isReader == 0)
    {
        System_printf ("Error: Using a writer MSGCOM channel '%s' to get message\n", ptrChannel->channelName);
        return -1;
    }

    /* Invoke the channel interface registered 'get' API */
    return gChannelInterface[ptrChannel->channelType].get(ptrChannel, msgBuffer);
}

/**
 *  @b Description
 *  @n
 *      The function allows application callback functions to be registered/deregistered
 *      with a MSGCOM channel
 *
 *  @param[in]  msgChHandle
 *      Channel Handle which is to be addressed.
 *  @param[in]  appCallback
 *      Application call back function. Passing a NULL will deregister a callback function
 *  @param[in]  arg
 *      Optional argument which needs to be passed to the registered callback function.
 *
 * \ingroup MSG_COM_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Msgcom_registerCallBack
(
    MsgCom_ChHandle     msgChHandle,
    MsgCom_AppCallBack  appCallback,
    uint32_t            arg
)
{
    Msgcom_channel*     ptrChannel;

    /* Get the channel information. */
    ptrChannel = (Msgcom_channel*)msgChHandle;
    if (ptrChannel == NULL)
        return MSGCOM_INVALID_PARAM;

    /* Overwite the callback function & argument in the channel configuration */
    ptrChannel->cfg.appCallBack = appCallback;
    ptrChannel->cfg.arg         = arg;
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the receive handler for the specified channel. The handler checks
 *      if a message was received and if so it can unblock the callee (for blocking
 *      channels). If running in polled mode with blocking channels the application
 *      needs to ensure that this function is called periodically.
 *
 *  @param[in]  msgChHandle
 *      Channel Handle which is to be addressed.
 *
 * \ingroup MSG_COM_FUNCTION
 *
 *  @retval
 *      Not Applicable.
 */
void Msgcom_channelRxHandler(MsgCom_ChHandle msgChHandle)
{
    Msgcom_channel*     ptrChannel;

    /* Get the channel information. */
    ptrChannel = (Msgcom_channel*)msgChHandle;

    /* Invoke the channel interface registered receive handler API */
    gChannelInterface[ptrChannel->channelType].rxHandler(ptrChannel);
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the internal queue associated with the
 *      MSGCOM channel. This API is meant for advanced users. This API is
 *      valid only for PHYSICAL channels.
 *
 *  @param[in]  msgChHandle
 *      Channel Handle for which the queue information is required.
 *
 * \ingroup MSG_COM_FUNCTION
 *
 *  @retval
 *      Success     - Queue Number
 *  @retval
 *      Error       - <0
 */
int32_t Msgcom_getInternalMsgQueueInfo(MsgCom_ChHandle msgChHandle)
{
    Msgcom_channel*     ptrChannel;

    /* Get the channel information. */
    ptrChannel = (Msgcom_channel*)msgChHandle;

    /* Debug Message: Only reader channels support the API */
    if (ptrChannel->isReader == 0)
    {
        System_printf ("Error: Using writer MSGCOM channel '%s' to get MSGCOM Queue Information\n", ptrChannel->channelName);
        return -1;
    }

    /* NOTE: This is an optional API and if the channel does not support it we return error. */
    if (gChannelInterface[ptrChannel->channelType].getInternalMsgQueueInfo(ptrChannel))
        return gChannelInterface[ptrChannel->channelType].getInternalMsgQueueInfo(ptrChannel);
    return MSGCOM_VIRTUAL_CHANNEL_NOT_SUPPORTED;
}

/**
 *  @b Description
 *  @n
 *      The function is used to create a MSGCOM instance. MSGCOM instances need to be
 *      created in the context of each processing entity. MSGCOM channels exist only in
 *      the context of the MSGCOM instance.
 *
 *  @param[in]  ptrInstCfg
 *      Pointer to the instance configuration.
 *  @param[out] errCode
 *      Error code populated on error
 *
 * \ingroup MSG_COM_FUNCTION
 *
 *  @retval
 *      Success     - Pointer to the MSGCOM instance
 *  @retval
 *      Error       - NULL
 */
Msgcom_InstHandle Msgcom_createInstance(Msgcom_InstCfg* ptrInstCfg, int32_t* errCode)
{
    Msgcom_InstanceMCB*     ptrInstanceMCB;
    Cppi_CpDmaInitCfg       cpdmaCfg;
    Cppi_CpDma              cpDma[MSGCOM_MAX_CPDMA_BLOCKS];
    int32_t                 index;

    /* Sanity Check: Validate the arguments. */
    if (ptrInstCfg == NULL)
    {
        *errCode = MSGCOM_INVALID_PARAM;
        return NULL;
    }

    /* Sanity Check: Ensure that the OSAL functions are valid */
    if ((ptrInstCfg->malloc        == NULL) || (ptrInstCfg->free          == NULL)  ||
        (ptrInstCfg->registerIsr   == NULL) || (ptrInstCfg->deregisterIsr == NULL)  ||
        (ptrInstCfg->disableSysInt == NULL) || (ptrInstCfg->enableSysInt  == NULL)  ||
        (ptrInstCfg->enterCS       == NULL) || (ptrInstCfg->exitCS        == NULL)  ||
        (ptrInstCfg->createSem     == NULL) || (ptrInstCfg->deleteSem     == NULL)  ||
        (ptrInstCfg->postSem       == NULL) || (ptrInstCfg->pendSem       == NULL))
    {
        *errCode = MSGCOM_INVALID_PARAM;
        return NULL;
    }

    /* Sanity Check: Ensure that valid instance handles have been passed */
    if ((ptrInstCfg->databaseHandle == NULL) || (ptrInstCfg->pktlibInstHandle == NULL))
    {
        *errCode = MSGCOM_INVALID_PARAM;
        return NULL;
    }

    /* Allocate memory for the MSGCOM instance. */
    ptrInstanceMCB = (Msgcom_InstanceMCB*)ptrInstCfg->malloc (Msgcom_MemAllocMode_INTERNAL, sizeof(Msgcom_InstanceMCB));
    if (ptrInstanceMCB == NULL)
    {
        *errCode = MSGCOM_MALLOC_FAILED;
        return NULL;
    }

    /* Initialize the allocated memory block. */
    memset ((void *)ptrInstanceMCB, 0, sizeof(Msgcom_InstanceMCB));

    /* Populate the instance configuration. */
    memcpy ((void *)&ptrInstanceMCB->cfg, (void *)ptrInstCfg, sizeof(Msgcom_InstCfg));

#if defined(__ARM_ARCH_7A__)
    if (ptrInstCfg->sysCfgHandle) {
        int32_t internalErrCode = 0;
        /* Prepare QMSS Barrier Queue */
        if (Resmgr_nameServiceGet(ptrInstCfg->sysCfgHandle,
                                  "QMSS_BarrierQ_MSMC",
                                  (uint32_t*)&ptrInstanceMCB->qmssBarrierQMsmc,
                                  &internalErrCode) < 0) {
            ptrInstanceMCB->qmssBarrierQMsmc = (Qmss_QueueHnd)QMSS_PARAM_NOT_SPECIFIED;
            System_printf("qmssBarrierQMsmc not supported by design\n"); //fzm

        } else {
            System_printf("   *** MSGCOM MSMC Barrier Q : %5u ***\n",
                          ptrInstanceMCB->qmssBarrierQMsmc);
        }

        if (Resmgr_nameServiceGet(ptrInstCfg->sysCfgHandle,
                                  "QMSS_BarrierQ_DDR",
                                  (uint32_t*)&ptrInstanceMCB->qmssBarrierQDdr,
                                  &internalErrCode) < 0) {
            ptrInstanceMCB->qmssBarrierQDdr = (Qmss_QueueHnd)QMSS_PARAM_NOT_SPECIFIED;
            System_printf("Resmger_nameServiceGet returned errcode %d for qmssBarrierQDdr\n", internalErrCode); //fzm
        } else {
            System_printf("   *** MSGCOM DDR  Barrier Q : %5u ***\n",
                          ptrInstanceMCB->qmssBarrierQDdr);
        }
    } else {
        System_printf(" ############################################################################\n ");
        System_printf("    ResMgr service handle is NULL! Skip QMSS Barrier Queues initialization!\n");
        System_printf(" ############################################################################\n ");
        ptrInstanceMCB->qmssBarrierQMsmc = (Qmss_QueueHnd)QMSS_PARAM_NOT_SPECIFIED;
        ptrInstanceMCB->qmssBarrierQDdr  = (Qmss_QueueHnd)QMSS_PARAM_NOT_SPECIFIED;
    }
#endif



    /* Get the number of CPDMA Instances which can be supported by MSGCOM */
    ptrInstanceMCB->numCPDMAInstances = Msgcom_getNumQMSSCPDMAInstances (&cpDma[0], &ptrInstanceMCB->queueType[0], &ptrInstanceMCB->queueBase[0]);
    if (ptrInstanceMCB->numCPDMAInstances > MSGCOM_MAX_CPDMA_BLOCKS)
    {
        /* FATAL Error: */
        *errCode = MSGCOM_INVALID_PARAM;
        return NULL;
    }

    /* Use the load balanced mode: */
    ptrInstanceMCB->dummyAllocationMode = 0;

    /* Cycle through all the CPDMA instances: */
    for (index = 0; index < ptrInstanceMCB->numCPDMAInstances; index++)
    {
        /* Initialize the configuration block */
        memset ((void *) &cpdmaCfg, 0, sizeof (Cppi_CpDmaInitCfg));

        /* Populate the DMA configuration: Dont override the hardware configuration. SOC Init is responsible
         * for the hardware configuration. */
        cpdmaCfg.dmaNum       = cpDma[index];
        cpdmaCfg.regWriteFlag = Cppi_RegWriteFlag_OFF;

        /* Open the CPPI handle */
        ptrInstanceMCB->cppiHnd[index] =  (Cppi_Handle)Cppi_open (&cpdmaCfg);
        if (ptrInstanceMCB->cppiHnd[index] == NULL)
        {
            /* FATAL Error: Unable to open the CPPI QMSS CPDMA Block. */
            *errCode = MSGCOM_QUEUE_OPEN_FAILED;
            ptrInstanceMCB->cfg.free (Msgcom_MemAllocMode_INTERNAL, ptrInstanceMCB , sizeof(Msgcom_InstanceMCB));
            return NULL;
        }
    }

    /* MSGCOM instance has been created successfully. */
    return (Msgcom_InstHandle)ptrInstanceMCB;
}

/**
 *  @b Description
 *  @n
 *      The function is used to delete the MSGCOM instance.
 *
 *  @param[in]  instHandle
 *      Handle to the MSGCOM instance
 *  @param[out] errCode
 *      Error code populated on error
 *
 * \ingroup MSG_COM_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t Msgcom_deleteInstance(Msgcom_InstHandle instHandle, int32_t* errCode)
{
    Msgcom_InstanceMCB* ptrInstanceMCB;
    int32_t             index;

    /* Sanity Check: Validate the arguments. */
    ptrInstanceMCB = (Msgcom_InstanceMCB*)instHandle;
    if (ptrInstanceMCB == NULL)
    {
        *errCode = MSGCOM_INVALID_PARAM;
        return -1;
    }

    /* Cycle through all the CPDMA instances: */
    for (index = 0; index < ptrInstanceMCB->numCPDMAInstances; index++)
        Cppi_close (ptrInstanceMCB->cppiHnd[index]);

    /* Cleanup the memory associated with the instance. */
    ptrInstanceMCB->cfg.free (Msgcom_MemAllocMode_INTERNAL, ptrInstanceMCB , sizeof(Msgcom_InstanceMCB));
    return 0;
}
