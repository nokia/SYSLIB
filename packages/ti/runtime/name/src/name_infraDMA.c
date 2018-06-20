/**
 *   @file  name_infraDMA.c
 *
 *   @brief
 *      The file implements the infrastructure DMA support which allows
 *      the name proxy between the DSP and ARM realm to communicate
 *      with each other.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2013 Texas Instruments, Inc.
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

/* MCSDK Include files */
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/cppi/cppi_desc.h>

/* SYSLIB Include files */
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/name/name.h>
#include <ti/runtime/name/include/name_internal.h>

/* For Debugging only. */
#ifdef __ARMv7
#define System_printf   printf
#else
#include <xdc/runtime/System.h>
#endif

/**********************************************************************
 ************* Name Infrastructure DMA Transport Functions ************
 **********************************************************************/

/**
 *  @b Description
 *  @n
 *      The function is used to create a Infrastructure channel with the
 *      specified configuration.
 *
 *  @param[in]  ptrNameProxy
 *      Pointer to the name proxy
 *  @param[out]  errCode
 *      Error code populated on error
 *
 *  @retval
 *      Success   - 0
 *  @retval
 *      Error     - <0
 */
int32_t Name_infraDMACreate(Name_ProxyMCB* ptrNameProxy, int32_t* errCode)
{
    uint8_t             isAllocated;
    Cppi_RxFlowCfg      rxFlowCfg;
    Qmss_Queue          queueInfo;
    Cppi_CpDmaInitCfg   cpdmaCfg;
    Cppi_TxChInitCfg    txChCfg;
    Cppi_RxChInitCfg    rxChCfg;

    /* Initialize the CPDMA Configuration. */
    memset ((void *) &cpdmaCfg, 0, sizeof (Cppi_CpDmaInitCfg));

    /* Populate the DMA configuration: Dont override the hardware configuration. SOC Init is responsible
     * for the hardware configuration. */
    cpdmaCfg.dmaNum       = Cppi_CpDma_QMSS_CPDMA;
    cpdmaCfg.regWriteFlag = Cppi_RegWriteFlag_OFF;

    /* Open the QMSS CPDMA Block. */
    ptrNameProxy->cppiHnd = (Cppi_Handle) Cppi_open (&cpdmaCfg);
    if (ptrNameProxy->cppiHnd == NULL)
    {
        *errCode = NAME_EINTERNAL;
        return -1;
    }

    /* Use the local flow identifier and open the appropriate channel number. */
    rxChCfg.channelNum  = ptrNameProxy->cfg.localFlowId;
    rxChCfg.rxEnable    = Cppi_ChState_CHANNEL_ENABLE;

    /* Open the receive channel. */
    ptrNameProxy->rxChannelHnd = Cppi_rxChannelOpen (ptrNameProxy->cppiHnd, &rxChCfg, &isAllocated);
    if (ptrNameProxy->rxChannelHnd == NULL)
    {
        *errCode = NAME_EINTERNAL;
        return -1;
    }
    System_printf ("Debug: Receive Channel %d Handle %p has been configured\n", rxChCfg.channelNum, ptrNameProxy->rxChannelHnd);

    /* Open a receive queue. Data which is received from the remote agent proxy peer will be placed
     * into this queue */
    ptrNameProxy->messageRxQueue = Qmss_queueOpen(Qmss_QueueType_GENERAL_PURPOSE_QUEUE,
                                                  QMSS_PARAM_NOT_SPECIFIED, &isAllocated);
    if (ptrNameProxy->messageRxQueue < 0)
    {
        *errCode = NAME_EINTERNAL;
        return -1;
    }
    System_printf ("Debug: Receive Queue -> 0x%x [%d.%d]\n", Qmss_getQIDFromHandle(ptrNameProxy->messageRxQueue),
                    Qmss_getQueueNumber(ptrNameProxy->messageRxQueue).qMgr,
                    Qmss_getQueueNumber(ptrNameProxy->messageRxQueue).qNum);

    /* Get the queue information about the proxy heap which has been configured to receive messages from the peer proxy */
    queueInfo = Qmss_getQueueNumber (Pktlib_getInternalHeapQueue(ptrNameProxy->cfg.proxyHeapHandle));

    System_printf ("Debug: Free Heap Queue -> [%d.%d]\n", queueInfo.qMgr, queueInfo.qNum);

    /* Setup the receive flow to receive data from the remote peer. Initialize the receive
     * flow configuration.*/
    memset ((void *) &rxFlowCfg, 0, sizeof (Cppi_RxFlowCfg));

    /* Configure the receive flow */
    rxFlowCfg.rx_dest_qmgr       = Qmss_getQueueNumber(ptrNameProxy->messageRxQueue).qMgr;
    rxFlowCfg.rx_dest_qnum       = Qmss_getQueueNumber(ptrNameProxy->messageRxQueue).qNum;
    rxFlowCfg.rx_sop_offset      = 0x0;
    rxFlowCfg.rx_desc_type       = Cppi_DescType_HOST;
    rxFlowCfg.rx_einfo_present   = 0x1;
    rxFlowCfg.flowIdNum          = ptrNameProxy->cfg.localFlowId;

    /* Set the Destination TAG LO to be overwritten. */
    rxFlowCfg.rx_dest_tag_lo_sel = 0x4;

    /* Setup the receive free queues. */
    rxFlowCfg.rx_fdq0_sz0_qnum  = queueInfo.qNum;
    rxFlowCfg.rx_fdq0_sz0_qmgr  = queueInfo.qMgr;
    rxFlowCfg.rx_fdq1_qnum      = queueInfo.qNum;
    rxFlowCfg.rx_fdq1_qmgr      = queueInfo.qMgr;
    rxFlowCfg.rx_fdq2_qnum      = queueInfo.qNum;
    rxFlowCfg.rx_fdq2_qmgr      = queueInfo.qMgr;
    rxFlowCfg.rx_fdq3_qnum      = queueInfo.qNum;
    rxFlowCfg.rx_fdq3_qmgr      = queueInfo.qMgr;

    /* Configure Rx flow */
    ptrNameProxy->flowChannelHnd = Cppi_configureRxFlow (ptrNameProxy->cppiHnd, &rxFlowCfg, &isAllocated);
    if (ptrNameProxy->flowChannelHnd == NULL)
    {
        *errCode = NAME_EINTERNAL;
        return -1;
    }
    System_printf ("Debug: Receive Flow %d Handle %p has been configured\n",
                    Cppi_getFlowId(ptrNameProxy->flowChannelHnd), ptrNameProxy->flowChannelHnd);

    /* Setup the transmit channel configuration: We open the transmit channel associated with the remote
     * flow identifier. */
    txChCfg.channelNum  = ptrNameProxy->cfg.remoteFlowId;
    txChCfg.priority    = 0;
    txChCfg.filterEPIB  = 0;
    txChCfg.filterPS    = 0;
    txChCfg.aifMonoMode = 0;
    txChCfg.txEnable    = Cppi_ChState_CHANNEL_ENABLE;

    /* Open the transmit channel. */
    ptrNameProxy->txChannelHnd = Cppi_txChannelOpen (ptrNameProxy->cppiHnd, &txChCfg, &isAllocated);
    if (ptrNameProxy->txChannelHnd == NULL)
    {
        *errCode = NAME_EINTERNAL;
        return -1;
    }
    System_printf ("Debug: Transmit Channel %d Handle %p has been configured\n",
                    Cppi_getChannelNumber(ptrNameProxy->txChannelHnd), ptrNameProxy->txChannelHnd);

    /* Open the corresponding infrastructure queue associated with the remote flow identifier. */
    ptrNameProxy->messageTxQueue = Qmss_queueOpen(Qmss_QueueType_INFRASTRUCTURE_QUEUE,
                                                    QMSS_INFRASTRUCTURE_QUEUE_BASE + ptrNameProxy->cfg.remoteFlowId,
                                                    &isAllocated);
    if (ptrNameProxy->messageTxQueue < 0)
    {
        *errCode = NAME_EINTERNAL;
        return -1;
    }
    System_printf ("Debug: Transmit Queue %x has been opened\n", ptrNameProxy->messageTxQueue);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get a message from the name proxy infrastructure channel
 *
 *  @param[in]  ptrNameProxy
 *      Pointer to the name proxy
 *  @param[out] ptrPkt
 *      Pointer to the received packet
 *
 *  @retval
 *      Success   - 0
 *  @retval
 *      Error     - <0
 */
int32_t Name_infraDMAGetMsg (Name_ProxyMCB* ptrNameProxy, Ti_Pkt** ptrPkt)
{
    /* Pop a packet from the message queue. */
    *ptrPkt = Qmss_queuePop (ptrNameProxy->messageRxQueue);
    if (*ptrPkt != NULL)
    {
        /* Message was available. We return the received message. */
        *ptrPkt = (Ti_Pkt*)QMSS_DESC_PTR((uint32_t)(*ptrPkt));

        /******************************************************************************
         * NOTE: There is an ownership change here from the CPDMA to the DSP. The DSP
         * takes over ownership of the packet.
         ******************************************************************************/
        Pktlib_getOwnership(ptrNameProxy->cfg.pktlibInstHandle, (Ti_Pkt *)(*ptrPkt));
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to send a message to the name proxy infrastructure channel
 *
 *  @param[in]  ptrNameProxy
 *      Pointer to the name proxy
 *  @param[out] ptrPkt
 *      Pointer to the received packet
 *
 *  @retval
 *      Success   - 0
 *  @retval
 *      Error     - <0
 */
int32_t Name_infraDMAPutMsg (Name_ProxyMCB* ptrNameProxy, Ti_Pkt* ptrPkt)
{
    Cppi_DescTag    tagInfo;

    /* Get the receive flow identifier. */
    tagInfo.destTagLo = 0;
    tagInfo.destTagHi = 0;
    tagInfo.srcTagHi  = 0;
    tagInfo.srcTagLo  = ptrNameProxy->cfg.remoteFlowId;

    /* Configure the TAG into the descriptor: */
    Pktlib_setTags(ptrPkt, &tagInfo);

    /* Since the NAME Infrastructure channels do not support transmission/reception of
     * PS Info. Reset these fields in descriptor to ensure that this is not happening.
     * This can cause the CPDMA to lock up if incorrectly configured. */
    Cppi_setPSLen (Cppi_DescType_HOST, (Cppi_Desc *)Pktlib_getDescFromPacket(ptrPkt), 0);

    /******************************************************************************
     * NOTE: There is an ownership change here from the DSP to the CPDMA. We need
     * to ensure that the packet contents here are written back.
     ******************************************************************************/
    Pktlib_releaseOwnership(ptrNameProxy->cfg.pktlibInstHandle, (Ti_Pkt *)ptrPkt);

    /* Write the packet to the specified message queue. Currently descriptor size is hardcoded.  */
    Qmss_queuePushDescSize (ptrNameProxy->messageTxQueue, (void*)ptrPkt, 128);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to delete the infrastructure channel.
 *
 *  @param[in]  ptrNameProxy
 *
 *  @retval
 *      Success   - 0
 *  @retval
 *      Error     - <0
 */
int32_t Name_infraDMADelete (Name_ProxyMCB* ptrNameProxy)
{
    /* Close the receive flow. */
    Cppi_closeRxFlow (ptrNameProxy->flowChannelHnd);

#if 0
    /* Close and disable the transmit and receive channels. */
    Cppi_channelDisable(ptrNameProxy->txChannelHnd);
    Cppi_channelClose(ptrNameProxy->txChannelHnd);
    Cppi_channelDisable(ptrNameProxy->rxChannelHnd);
    Cppi_channelClose(ptrNameProxy->rxChannelHnd);
#endif

    /* Close the CPPI Infrastructure CPDMA Handle; accounting for the additonal open. */
    Cppi_close(ptrNameProxy->cppiHnd);

    /* Close the receive & transmit queue. */
    Qmss_queueClose(ptrNameProxy->messageRxQueue);
    Qmss_queueClose(ptrNameProxy->messageTxQueue);

    /* Infrastructure DMA was successfully shutdown. */
    return 0;
}

