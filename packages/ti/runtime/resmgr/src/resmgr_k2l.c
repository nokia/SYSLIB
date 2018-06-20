/**
 *   @file  resmgr_k2l.c
 *
 *   @brief
 *      K2L Device specific implementation
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
#if defined (DEVICE_K2L)

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
 ***************************** K2L Functions **************************
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
    /*****************************************************************************
     * ARM:
     *  SOC_SET_1 and the GIC Queues are mapped to the ARM.
     *****************************************************************************/
    Qmss_Queue      queueInfo;
    uint32_t        queueId;
    const int32_t socSet1Mapping [] =
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
    const int32_t gicSetMapping [] =
    {
        48,    /* Queue 528 */
        49,    /* Queue 529 */
        50,    /* Queue 530 */
        51,    /* Queue 531 */
        52,    /* Queue 532 */
        53,    /* Queue 533 */
        54,    /* Queue 534 */
        55,    /* Queue 535 */
        56,    /* Queue 536 */
        57,    /* Queue 537 */
        58,    /* Queue 538 */
        59,    /* Queue 539 */
        60,    /* Queue 540 */
        61,    /* Queue 541 */
        62,    /* Queue 542 */
        63,    /* Queue 543 */
        64,    /* Queue 544 */
        65,    /* Queue 545 */
        66,    /* Queue 546 */
        67,    /* Queue 547 */
        68,    /* Queue 548 */
        69,    /* Queue 549 */
        70,    /* Queue 550 */
        71,    /* Queue 551 */
        72,    /* Queue 552 */
        73,    /* Queue 553 */
        74,    /* Queue 554 */
        75,    /* Queue 555 */
        76,    /* Queue 556 */
        77,    /* Queue 557 */
        78,    /* Queue 558 */
        79     /* Queue 559 */
    };

    /* Get the queue information. */
    queueInfo.qMgr = qMgr;
    queueInfo.qNum = qNum;

    /* Get the queue identifier. */
    queueId = Qmss_getQIDFromHandle(Qmss_getQueueHandle(queueInfo));

    /* Is the queue number in the ARM GIC Queue range? */
    if ((qNum >= QMSS_ARM_GIC_QUEUE_BASE) && (qNum < (QMSS_ARM_GIC_QUEUE_BASE + QMSS_MAX_ARM_GIC_QUEUE)))
        return gicSetMapping[queueId - QMSS_ARM_GIC_QUEUE_BASE];

    /* Is the queue number in the SOC Set1 Queue range? */
    if ((qNum >= QMSS_SOC_SET_1_QUEUE_BASE) && (qNum < (QMSS_SOC_SET_1_QUEUE_BASE + QMSS_MAX_SOC_SET_1_QUEUE)))
        return socSet1Mapping[qNum - QMSS_SOC_SET_1_QUEUE_BASE];

    /* Error: The range is NOT supported for ARM. */
    return -1;
#else
    /*****************************************************************************
     * DSP:
     *  CIC0_QUEUE, CIC_SET0, CIC_SET1, CIC_SET2, CIC_SET3 and SOC_SET1 are mapped
     *  to DSP
     *****************************************************************************/
    Qmss_Queue      queueInfo;
    uint32_t        queueId;
    const int32_t cic0QueueMapping [] =
    {
        389,        /* Queue 570 */
        398,        /* Queue 571 */
        399,        /* Queue 572 */
        400,        /* Queue 573 */
        401,        /* Queue 574 */
        402,        /* Queue 575 */
        403,        /* Queue 576 */
        404,        /* Queue 577 */
        405,        /* Queue 578 */
        406,        /* Queue 579 */
        407,        /* Queue 580 */
        408,        /* Queue 581 */
        409,        /* Queue 582 */
        410,        /* Queue 583 */
        411,        /* Queue 584 */
        412,        /* Queue 585 */
        413,        /* Queue 586 */
        443,        /* Queue 587 */
    };
    const int32_t cicSet0QueueMapping [] =
    {
        213,        /* Queue 605 */
        214,        /* Queue 606 */
        215,        /* Queue 607 */
        216,        /* Queue 608 */
        217,        /* Queue 609 */
        218,        /* Queue 610 */
        219,        /* Queue 611 */
        220,        /* Queue 612 */
        221,        /* Queue 613 */
        222,        /* Queue 614 */
        223,        /* Queue 615 */
        224,        /* Queue 616 */
        225,        /* Queue 617 */
        226,        /* Queue 618 */
        227,        /* Queue 619 */
        228,        /* Queue 620 */
        229,        /* Queue 621 */
        230,        /* Queue 622 */
        231,        /* Queue 623 */
        232,        /* Queue 624 */
        273,        /* Queue 625 */
        274,        /* Queue 626 */
        275,        /* Queue 627 */
        276,        /* Queue 628 */
        277,        /* Queue 629 */
        278,        /* Queue 630 */
        176,        /* Queue 631 */
        177,        /* Queue 632 */
        178,        /* Queue 633 */
        179,        /* Queue 634 */
        180,        /* Queue 635 */
        181,        /* Queue 636 */
    };
    const int32_t cicSet1QueueMapping [] =
    {
        47,         /* Queue 652 */
        91,         /* Queue 653 */
        93,         /* Queue 654 */
        95,         /* Queue 655 */
        97,         /* Queue 656 */
        151,        /* Queue 657 */
    };
    const int32_t cicSet2QueueMapping [] =
    {
        292,        /* Queue 666 */
        293,        /* Queue 667 */
        294,        /* Queue 668 */
        295,        /* Queue 669 */
        296,        /* Queue 670 */
        297,        /* Queue 671 */
        298,        /* Queue 672 */
        299,        /* Queue 673 */
        300,        /* Queue 674 */
        301,        /* Queue 675 */
        302,        /* Queue 676 */
        303,        /* Queue 677 */
        304,        /* Queue 678 */
        305,        /* Queue 679 */
        306,        /* Queue 680 */
        307,        /* Queue 681 */
        308,        /* Queue 682 */
        309,        /* Queue 683 */
        310,        /* Queue 684 */
        311,        /* Queue 685 */
        312,        /* Queue 686 */
        313,        /* Queue 687 */
    };
    const int32_t cicSet3QueueMapping [] =
    {
        383,        /* Queue 599 */
        384,        /* Queue 600 */
        385,        /* Queue 601 */
        386,        /* Queue 602 */
    };
    const int32_t socSet1Mapping [] =
    {
        152,    /* Queue 658  */
        153,    /* Queue 659  */
        154,    /* Queue 660  */
        155,    /* Queue 661  */
        156,    /* Queue 662  */
        157,    /* Queue 663  */
        158,    /* Queue 664  */
        159,    /* Queue 665  */
    };

    /* Get the queue information. */
    queueInfo.qMgr = qMgr;
    queueInfo.qNum = qNum;

    /* Get the queue identifier. */
    queueId = Qmss_getQIDFromHandle(Qmss_getQueueHandle (queueInfo));

    /* Is the queue number in the CIC0 Queue range? */
    if ((qNum >= QMSS_CIC0_QUEUE_BASE) && (qNum < (QMSS_CIC0_QUEUE_BASE + QMSS_MAX_CIC0_QUEUE)))
        return cic0QueueMapping[queueId - QMSS_CIC0_QUEUE_BASE];

    /* Is the queue number in the CIC_SET0 Queue range? */
    if ((qNum >= QMSS_CIC_SET_0_QUEUE_BASE) && (qNum < (QMSS_CIC_SET_0_QUEUE_BASE + QMSS_MAX_CIC_SET_0_QUEUE)))
        return cicSet0QueueMapping[queueId - QMSS_CIC_SET_0_QUEUE_BASE];

    /* Is the queue number in the CIC_SET1 Queue range? */
    if ((qNum >= QMSS_CIC_SET_1_QUEUE_BASE) && (qNum < (QMSS_CIC_SET_1_QUEUE_BASE + QMSS_MAX_CIC_SET_1_QUEUE)))
        return cicSet1QueueMapping[queueId - QMSS_CIC_SET_1_QUEUE_BASE];

    /* Is the queue number in the CIC_SET2 Queue range? */
    if ((qNum >= QMSS_CIC_SET_2_QUEUE_BASE) && (qNum < (QMSS_CIC_SET_2_QUEUE_BASE + QMSS_MAX_CIC_SET_2_QUEUE)))
        return cicSet2QueueMapping[queueId - QMSS_CIC_SET_2_QUEUE_BASE];

    /* Is the queue number in the CIC_SET3 Queue range? */
    if ((qNum >= QMSS_CIC_SET_3_QUEUE_BASE) && (qNum < (QMSS_CIC_SET_3_QUEUE_BASE + QMSS_MAX_CIC_SET_3_QUEUE)))
        return cicSet3QueueMapping[queueId - QMSS_CIC_SET_3_QUEUE_BASE];

    /* Is the queue number in the SOC_SET1 Queue range? */
    if ((qNum >= QMSS_SOC_SET_1_QUEUE_BASE) && (qNum < (QMSS_SOC_SET_1_QUEUE_BASE + QMSS_MAX_SOC_SET_1_QUEUE)))
        return socSet1Mapping[queueId - QMSS_SOC_SET_1_QUEUE_BASE];

    /* Invalid queue number which is not in the range of queues which can generate interrupts */
    return -1;
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
    /* Lamarr:
     *  On ARM all the accumulated channels are routed to all the ARM cores
     *  via the GIC interrupt lines. */
    if (dtsAccChannel < 4)
    {
        *accChannel     = 4 + dtsAccChannel;
        queueInfo->qMgr = 0;
        queueInfo->qNum = 704 + dtsAccChannel;
        *qmssPdspId     = Qmss_PdspId_PDSP1;
        *eventId        = 179 + dtsAccChannel;
    }
    else if ((dtsAccChannel >= 4) && (dtsAccChannel < 8))
    {
        *accChannel     = 12 + (dtsAccChannel - 4);
        queueInfo->qMgr = 0;
        queueInfo->qNum = 708 + (dtsAccChannel - 4);
        *qmssPdspId     = Qmss_PdspId_PDSP1;
        *eventId        = 179 + dtsAccChannel;
    }
    else if ((dtsAccChannel >= 8) && (dtsAccChannel < 12))
    {
        *accChannel     = 20 + (dtsAccChannel - 8);
        queueInfo->qMgr = 0;
        queueInfo->qNum = 712 + (dtsAccChannel - 8);
        *qmssPdspId     = Qmss_PdspId_PDSP1;
        *eventId        = 179 + dtsAccChannel;
    }
    else if ((dtsAccChannel >= 12) && (dtsAccChannel < 16))
    {
        *accChannel     = 28 + (dtsAccChannel - 12);
        queueInfo->qMgr = 0;
        queueInfo->qNum = 716 + (dtsAccChannel - 12);
        *qmssPdspId     = Qmss_PdspId_PDSP1;
        *eventId        = 179 + dtsAccChannel;
    }
    else if ((dtsAccChannel >= 16) && (dtsAccChannel < 20))
    {
        *accChannel     = 4 + (dtsAccChannel - 16);
        queueInfo->qMgr = 0;
        queueInfo->qNum = 720 + (dtsAccChannel - 16);
        *qmssPdspId     = Qmss_PdspId_PDSP3;
        *eventId        = 229 + dtsAccChannel;
    }
    else if ((dtsAccChannel >= 20) && (dtsAccChannel < 24))
    {
        *accChannel     = 12 + (dtsAccChannel - 20);
        queueInfo->qMgr = 0;
        queueInfo->qNum = 724 + (dtsAccChannel - 20);
        *qmssPdspId     = Qmss_PdspId_PDSP3;
        *eventId        = 229 + dtsAccChannel;
    }
    else if ((dtsAccChannel >= 24) && (dtsAccChannel < 28))
    {
        *accChannel     = 20 + (dtsAccChannel - 24);
        queueInfo->qMgr = 0;
        queueInfo->qNum = 728 + (dtsAccChannel - 24);
        *qmssPdspId     = Qmss_PdspId_PDSP3;
        *eventId        = 229 + dtsAccChannel;
    }
    else if ((dtsAccChannel >= 28) && (dtsAccChannel < 32))
    {
        *accChannel     = 28 + (dtsAccChannel - 28);
        queueInfo->qMgr = 0;
        queueInfo->qNum = 732 + (dtsAccChannel - 28);
        *qmssPdspId     = Qmss_PdspId_PDSP3;
        *eventId        = 229 + dtsAccChannel;
    }
    else
    {
        /* Control should never come here and this implies that the DTS files are
         * invalid. */
        return -1;
    }
    /* Control comes here implies that the mapping was successfully completed */
    return 0;
#else
    /* Lamarr:
     * Each core has 4 accumulated channels mapped to PDSPID1 and  PDSPID3.
     * Queues are from:-
     *      704+N, 708+N, 712+N, 716+N
     * Priority Channels are from:-
     *      N, N+8, N+16 and N+24 */
    if (dtsAccChannel < 4)
    {
        queueInfo->qMgr = 0;
        queueInfo->qNum = 704 + dspCoreId;
        *eventId        = 48;
        *accChannel     = dtsAccChannel;
        *qmssPdspId     = Qmss_PdspId_PDSP1;
    }
    else if ((dtsAccChannel >= 4) && (dtsAccChannel < 8))
    {
        queueInfo->qMgr = 0;
        queueInfo->qNum = 708 + dspCoreId;
        *eventId        = 49;
        *accChannel     = dtsAccChannel + 4;
        *qmssPdspId     = Qmss_PdspId_PDSP1;
    }
    else if ((dtsAccChannel >= 8) && (dtsAccChannel < 12))
    {
        queueInfo->qMgr = 0;
        queueInfo->qNum = 712 + dspCoreId;
        *eventId        = 50;
        *accChannel     = dtsAccChannel + 8;
        *qmssPdspId     = Qmss_PdspId_PDSP1;
    }
    else if ((dtsAccChannel >= 12) && (dtsAccChannel < 16))
    {
        queueInfo->qMgr = 0;
        queueInfo->qNum = 716 + dspCoreId;
        *eventId        = 51;
        *accChannel     = dtsAccChannel + 12;
        *qmssPdspId     = Qmss_PdspId_PDSP1;
    }
    else if ((dtsAccChannel >= 16) && (dtsAccChannel < 20))
    {
        queueInfo->qMgr = 0;
        queueInfo->qNum = 720 + dspCoreId;
        *eventId        = 52;
        *accChannel     = dtsAccChannel - 16;
        *qmssPdspId     = Qmss_PdspId_PDSP3;
    }
    else if ((dtsAccChannel >= 20) && (dtsAccChannel < 24))
    {
        queueInfo->qMgr = 0;
        queueInfo->qNum = 724 + dspCoreId;
        *eventId        = 53;
        *accChannel     = dtsAccChannel - 12;
        *qmssPdspId     = Qmss_PdspId_PDSP3;
    }
    else if ((dtsAccChannel >= 24) && (dtsAccChannel < 28))
    {
        queueInfo->qMgr = 0;
        queueInfo->qNum = 728 + dspCoreId;
        *eventId        = 54;
        *accChannel     = dtsAccChannel - 8;
        *qmssPdspId     = Qmss_PdspId_PDSP3;
    }
    else if ((dtsAccChannel >= 28) && (dtsAccChannel < 32))
    {
        queueInfo->qMgr = 0;
        queueInfo->qNum = 732 + dspCoreId;
        *eventId        = 55;
        *accChannel     = dtsAccChannel - 4;
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

#endif /* DEVICE_K2L */

