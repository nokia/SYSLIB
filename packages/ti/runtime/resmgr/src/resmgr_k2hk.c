/**
 *   @file  resmgr_k2hk.c
 *
 *   @brief
 *      K2H/K2K Device specific implementation
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
#if defined (DEVICE_K2H) || defined (DEVICE_K2K)

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

/* SYSLIB include files. */
#include <ti/runtime/resmgr/resmgr.h>

/**********************************************************************
 *************************** K2H/K2K Functions ************************
 **********************************************************************/

/**
 *  @b Description
 *  @n
 *      The function is used to return the system interrupt which is mapped
 *      to the direct interrupt queue.
 *
 *  @param[in]  cpIntcId
 *      CPINTC Instance identifier
 *  @param[in]  qMgr
 *      Queue Manager
 *  @param[in]  qNum
 *      Queue Number
 *
 * \ingroup RES_MGR_INTERNAL_FUN
 *
 *  @retval
 *      Success   - System Interrupt which is mapped
 *  @retval
 *      Error     - <0
 */
int32_t Resmgr_mapQueuePendToInterrupt
(
    int32_t     cpIntcId,
    int32_t     qMgr,
    int32_t     qNum
)
{
#ifdef __ARMv7
    /* On Hawking: INTC SET 2 and the GIC Queues are mapped to the ARM interrupts. */
    Qmss_Queue      queueInfo;
    uint32_t        queueId;

    /* System Interrupt Mapping for INTC_SET2 Queues */
    const int32_t intcSet2Mapping [] =
    {
        40,    /* Queue 658  */
        41,    /* Queue 659  */
        42,    /* Queue 660  */
        43,    /* Queue 661  */
        44,    /* Queue 662  */
        45,    /* Queue 663  */
        46,    /* Queue 664  */
        47,    /* Queue 665  */
    };

    /* System Interrupt Mapping for the GIC Queues */
    const int32_t gicSetMapping [] =
    {
        48,    /* Queue 8704 */
        49,    /* Queue 8705 */
        50,    /* Queue 8706 */
        51,    /* Queue 8707 */
        52,    /* Queue 8708 */
        53,    /* Queue 8709 */
        54,    /* Queue 8710 */
        55,    /* Queue 8711 */
        56,    /* Queue 8712 */
        57,    /* Queue 8713 */
        58,    /* Queue 8714 */
        59,    /* Queue 8715 */
        60,    /* Queue 8716 */
        61,    /* Queue 8717 */
        62,    /* Queue 8718 */
        63,    /* Queue 8719 */
        64,    /* Queue 8720 */
        65,    /* Queue 8721 */
        66,    /* Queue 8722 */
        67,    /* Queue 8723 */
        68,    /* Queue 8724 */
        69,    /* Queue 8725 */
        70,    /* Queue 8726 */
        71,    /* Queue 8727 */
        72,    /* Queue 8728 */
        73,    /* Queue 8729 */
        74,    /* Queue 8730 */
        75,    /* Queue 8731 */
        76,    /* Queue 8732 */
        77,    /* Queue 8733 */
        78,    /* Queue 8734 */
        79     /* Queue 8735 */
    };

    /* Get the queue information. */
    queueInfo.qMgr = qMgr;
    queueInfo.qNum = qNum;

    /* Get the queue identifier. */
    queueId = Qmss_getQIDFromHandle(Qmss_getQueueHandle(queueInfo));

    /* Sanity Check: Validate the arguments. Which queue manager? This will decide
     * which INTC Set we are trying to get the mapping for. */
    if (qMgr == 0)
    {
        /* Queue Manager 0: INTC_SET 2 are located here */
        if ((qNum < QMSS_INTC_SET2_QUEUE_BASE) || (qNum >= (QMSS_INTC_SET2_QUEUE_BASE + QMSS_MAX_INTC_SET2_QUEUE)))
            return -1;

        /* Control comes here implies that the queue number is within the range of the
         * Direct Interrupt Queues. So here we now need to map these out as per the
         * device architecture specifications */
        return intcSet2Mapping[qNum - QMSS_INTC_SET2_QUEUE_BASE];
    }

    /* Queue Manager 2: GIC Interrupt Queues are located here */
    if ((queueId < QMSS_GIC400_QUEUE_BASE) || (queueId > (QMSS_GIC400_QUEUE_BASE + QMSS_MAX_GIC400_QUEUE)))
        return -1;

    /* Return the mapping. */
    return gicSetMapping[queueId - QMSS_GIC400_QUEUE_BASE];
#else
    /* On Hawking: INTC SET 1, 2, 3 and 4 are mapped to the DSP interrupts via the CPINTC. */
    Qmss_Queue      queueInfo;
    uint32_t        queueId;

    /* CPINTC0 System Interrupt Mapping for INTC_SET 1, 2 and 3 */
    const int32_t cpIntc0Set123Mapping [] =
    {
        47,     /* Queue 652  */
        91,     /* Queue 653  */
        93,     /* Queue 654  */
        95,     /* Queue 655  */
        97,     /* Queue 656  */
        151,    /* Queue 657  */
        152,    /* Queue 658  */
        153,    /* Queue 659  */
        154,    /* Queue 660  */
        155,    /* Queue 661  */
        156,    /* Queue 662  */
        157,    /* Queue 663  */
        158,    /* Queue 664  */
        159,    /* Queue 665  */
        292,    /* Queue 666  */
        293,    /* Queue 667  */
        294,    /* Queue 668  */
        295,    /* Queue 669  */
        296,    /* Queue 670  */
        297,    /* Queue 671  */
    };

    /* CPINTC0 System Interrupt Mapping for INTC_SET 4 */
    const int32_t cpIntc0Set4Mapping [] =
    {
        298,    /* Queue 8844 */
        299,    /* Queue 8845 */
        300,    /* Queue 8846 */
        301,    /* Queue 8847 */
        302,    /* Queue 8848 */
        303,    /* Queue 8849 */
        304,    /* Queue 8850 */
        305,    /* Queue 8851 */
        306,    /* Queue 8852 */
        307,    /* Queue 8853 */
        308,    /* Queue 8854 */
        309,    /* Queue 8855 */
        310,    /* Queue 8856 */
        311,    /* Queue 8857 */
        312,    /* Queue 8858 */
        313,    /* Queue 8859 */
        314,    /* Queue 8860 */
        315,    /* Queue 8861 */
        316,    /* Queue 8862 */
        317     /* Queue 8863 */
    };

    /* CPINTC1 System Interrupt Mapping for INTC_SET 1, 2 and 3 */
    const int32_t cpIntc1Set123Mapping [] =
    {
        292,    /* Queue 652  */
        293,    /* Queue 653  */
        294,    /* Queue 654  */
        295,    /* Queue 655  */
        296,    /* Queue 656  */
        297,    /* Queue 657  */
        47,     /* Queue 658  */
        91,     /* Queue 659  */
        93,     /* Queue 660  */
        95,     /* Queue 661  */
        97,     /* Queue 662  */
        151,    /* Queue 663  */
        152,    /* Queue 664  */
        153,    /* Queue 665  */
        154,    /* Queue 666  */
        155,    /* Queue 667  */
        156,    /* Queue 668  */
        157,    /* Queue 669  */
        158,    /* Queue 670  */
        159,    /* Queue 671  */
    };

    /* CPINTC1 System Interrupt Mapping for INTC_SET 4 */
    const int32_t cpIntc1Set4Mapping [] =
    {
        298,    /* Queue 8844 */
        299,    /* Queue 8845 */
        300,    /* Queue 8846 */
        301,    /* Queue 8847 */
        302,    /* Queue 8848 */
        303,    /* Queue 8849 */
        304,    /* Queue 8850 */
        305,    /* Queue 8851 */
        306,    /* Queue 8852 */
        307,    /* Queue 8853 */
        308,    /* Queue 8854 */
        309,    /* Queue 8855 */
        310,    /* Queue 8856 */
        311,    /* Queue 8857 */
        312,    /* Queue 8858 */
        313,    /* Queue 8859 */
        314,    /* Queue 8860 */
        315,    /* Queue 8861 */
        316,    /* Queue 8862 */
        317     /* Queue 8863 */
    };

    /* Get the queue information. */
    queueInfo.qMgr = qMgr;
    queueInfo.qNum = qNum;

    /* Get the queue identifier. */
    queueId = Qmss_getQIDFromHandle(Qmss_getQueueHandle (queueInfo));

    /* Sanity Check: Validate the arguments. Which queue manager? This will decide
     * which INTC Set we are trying to get the mapping for. */
    if (qMgr == 0)
    {
        /* Queue Manager 0: INTC_SET 1, 2 and 3 are located here */
        if (qNum < QMSS_INTC_SET1_QUEUE_BASE)
            return -1;

        /* Detect if this is a valid direct interrupt queue number */
        if (qNum >= (QMSS_INTC_SET3_QUEUE_BASE + QMSS_MAX_INTC_SET3_QUEUE))
            return -1;

        /* Control comes here implies that the queue number is within the range of the
         * Direct Interrupt Queues. So here we now need to map these out as per the
         * device architecture specifications */
        if (cpIntcId == 0)
            return cpIntc0Set123Mapping[qNum - QMSS_INTC_SET1_QUEUE_BASE];
        else
            return cpIntc1Set123Mapping[qNum - QMSS_INTC_SET1_QUEUE_BASE];
    }

    /* Queue Manager 2: INTC_SET 4 is located here */
    if ((queueId < QMSS_INTC_SET4_QUEUE_BASE) || (queueId >= (QMSS_INTC_SET4_QUEUE_BASE + QMSS_MAX_INTC_SET4_QUEUE)))
        return -1;

    /* Return the mapping. */
    if (cpIntcId == 0)
        return cpIntc0Set4Mapping[queueId - QMSS_INTC_SET4_QUEUE_BASE];
    else
        return cpIntc1Set4Mapping[queueId - QMSS_INTC_SET4_QUEUE_BASE];
#endif
}

/**
 *  @b Description
 *  @n
 *      Each accumulated channel is associated with the following characteristics:
 *          - Queue Information
 *          - Event Identifier
 *          - PDSP Identifier.
 *
 *  @param[in]  dtsAccChannel
 *      Accumulated channel
 *  @param[in]  dspCoreId
 *      DSP core identifier
 *  @param[out] accChannel
 *      Translated accumulated channel from the DTS to the real API usable one.
 *  @param[out] queueInfo
 *      Pointer to the queue information
 *  @param[out] eventId
 *      Event identifier associated with the accumulated channel
 *  @param[out] qmssPdspId
 *      PDSP identifier associated with the channel
 *
 *  \ingroup RES_MGR_INTERNAL_FUN
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Resmgr_getAccumulatedChannelMapping
(
    uint8_t         dtsAccChannel,
    uint8_t         dspCoreId,
    uint8_t*        accChannel,
    Qmss_Queue*     queueInfo,
    uint32_t*       eventId,
    Qmss_PdspId*    qmssPdspId
)
{
#ifdef __ARMv7
    /* Hawking:
     *  (a) On ARM all the accumulated channels are routed to all the ARM cores
     *      via the GIC interrupt lines.
     *  (b) Accumulated Channels <32 use PDSP1 while >32 use PDSP3. */
    if (dtsAccChannel < 32)
    {
        /* Accumulated channels <32 are used by PDSP1. */
        *accChannel     = dtsAccChannel;
        queueInfo->qMgr = 0;
        queueInfo->qNum = 704 + *accChannel;
        *qmssPdspId     = Qmss_PdspId_PDSP1;
        *eventId        = 179 + *accChannel;
    }
    else
    {
        /* Accumulated channels >32 are used by PDSP3. */
        *accChannel     = dtsAccChannel - 32;
        queueInfo->qMgr = 2;
        queueInfo->qNum = 8896 + *accChannel;
        *qmssPdspId     = Qmss_PdspId_PDSP3;
        *eventId        = 229 + *accChannel;
    }
    return 0;
#else
    /* Hawking:
     *  (a) Each core has 4 accumulated channels mapped to PDSPID 1. Queues
     *      are from 704+N, 712+N, 720+N, 728+N.
     *  (b) Each core has 4 accumulated channels mapped to PDSPID 3. Queues
     *      are from 8896+N, 8904+N, 8912+N, 8920+N. */
    if (dtsAccChannel < 8)
    {
        queueInfo->qMgr = 0;
        queueInfo->qNum = 704 + dspCoreId;
        *eventId        = 48;
        *accChannel     = dtsAccChannel;
        *qmssPdspId     = Qmss_PdspId_PDSP1;
    }
    else if ((dtsAccChannel >= 8) && (dtsAccChannel < 16))
    {
        queueInfo->qMgr = 0;
        queueInfo->qNum = 712 + dspCoreId;
        *eventId        = 49;
        *accChannel     = dtsAccChannel;
        *qmssPdspId     = Qmss_PdspId_PDSP1;
    }
    else if ((dtsAccChannel >= 16) && (dtsAccChannel < 24))
    {
        queueInfo->qMgr = 0;
        queueInfo->qNum = 720 + dspCoreId;
        *eventId        = 50;
        *accChannel     = dtsAccChannel;
        *qmssPdspId     = Qmss_PdspId_PDSP1;
    }
    else if ((dtsAccChannel >= 24) && (dtsAccChannel < 32))
    {
        queueInfo->qMgr = 0;
        queueInfo->qNum = 728 + dspCoreId;
        *eventId        = 51;
        *accChannel     = dtsAccChannel;
        *qmssPdspId     = Qmss_PdspId_PDSP1;
    }
    else if ((dtsAccChannel >= 32) && (dtsAccChannel < 40))
    {
        queueInfo->qMgr = 2;
        queueInfo->qNum = 8896 + dspCoreId;
        *eventId        = 52;
        *accChannel     = dtsAccChannel - 32;
        *qmssPdspId     = Qmss_PdspId_PDSP3;
    }
    else if ((dtsAccChannel >= 40) && (dtsAccChannel < 48))
    {
        queueInfo->qMgr = 2;
        queueInfo->qNum = 8904 + dspCoreId;
        *eventId        = 53;
        *accChannel     = dtsAccChannel - 32;
        *qmssPdspId     = Qmss_PdspId_PDSP3;
    }
    else if ((dtsAccChannel >= 48) && (dtsAccChannel < 56))
    {
        queueInfo->qMgr = 2;
        queueInfo->qNum = 8912 + dspCoreId;
        *eventId        = 54;
        *accChannel     = dtsAccChannel - 32;
        *qmssPdspId     = Qmss_PdspId_PDSP3;
    }
    else if ((dtsAccChannel >= 56) && (dtsAccChannel < 63))
    {
        queueInfo->qMgr = 2;
        queueInfo->qNum = 8920 + dspCoreId;
        *eventId        = 55;
        *accChannel     = dtsAccChannel - 32;
        *qmssPdspId     = Qmss_PdspId_PDSP3;
    }
    else
    {
        /* Control should never come here and this implies that the DTS files are
         * invalid. */
        return -1;
    }

    /* Control comes here implies that the mapping was successfully completed */
    return 0;
#endif
}

#endif /* DEVICE_K2H/DEVICE_K2K */
