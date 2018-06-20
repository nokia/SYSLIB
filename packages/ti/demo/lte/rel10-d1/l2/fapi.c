/*
 *   @file  fapi.c
 *
 *   @brief
 *      FAPI Template Implementation. This file implements the basic
 *      functionality which showcases the communication of FAPI messages
 *      between the L2 and L1 PHY.
 *
 *      It is highly *recommended* that application developers modify
 *      this implementation as per their application requirements.
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

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <malloc.h>
#include <semaphore.h>

/* MCSDK Include files */
#include <ti/runtime/hplib/hplib.h>

/* SYSLIB Include Files.  */
#include <ti/runtime/msgcom/msgcom.h>
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/netfp/netfp.h>
#include <ti/runtime/resmgr/resmgr.h>

/* cUIA Include Files. */
#include <ti/uia/runtime/LogSnapshot.h>
#include <ti/uia/events/UIABenchmark.h>
#include <ti/uia/events/UIAEvt.h>
#include <ti/uia/events/UIAErr.h>
#include <ti/uia/events/UIAStatistic.h>
#include <xdc/runtime/Log.h>
#include <xdc/runtime/Diags.h>

/* FAPI Interface */
#include <ti/demo/lte/common/ti_fapi.h>
#include <ti/demo/lte/common/ti_fapiChanNames.h>
#include <ti/runtime/fapi_tracing/fapi_tracing.h>
#include "l2_lte.h"

/* Flag to use MEMLOGGING for FAPI logs */
#define FAPI_MEMLOGGING 0

/**********************************************************************
 ************************** Local Structures **************************
 **********************************************************************/

/**
 * @brief
 *  FAPI States on the eNodeB
 *
 * @details
 *  The enumerating which describes the FAPI states
 */
typedef enum Fapi_State
{
    /**
     * @brief   Initial state of the FAPI
     */
    Fapi_State_INITIAL                   =   0x0,

    /**
     * @brief   State after PARAMETERS have been exchanged
     */
    Fapi_State_PARAM_EXCHANGED          =   0x1,

    /**
     * @brief   State after CELL Configuration has been done.
     */
    Fapi_State_CELL_CONFIG_RESPONSE      =   0x2,

    /**
     * @brief   State after which the PHY has been started.
     */
    Fapi_State_PHY_STARTED               =   0x3
}Fapi_State;

/**
 * @brief
 *  FAPI Tracing statistics
 *
 * @details
 *  Statistics for FAPI Tracing
 */
typedef struct FapiTracing_Stats
{
    /**
     * @brief   Total number of packets sent out
     */
    uint64_t TotalPackets;

    /**
     * @brief   Total number of dropped packets
     */
    uint64_t DroppedPackets;
}FapiTracing_Stats;

/**
 * @brief
 *  FAPI States on the eNodeB
 *
 * @details
 *  The enumerating which describes the FAPI states
 */
typedef enum Fapi_CellId
{
    /**
     * @brief   Id for Cell A
     */
    Fapi_Cell_A                   =   0x1,

    /**
     * @brief   Id for Cell A
     */
    Fapi_Cell_B                   =   0x2,
}Fapi_CellId;

/**
 * @brief
 *  Master Control Block for the FAPI
 *
 * @details
 *  The structure holds all the information which is required for the FAPI
 *  interface to function.
 */
typedef struct Fapi_MCB
{
    /**
     * @brief   eNodeB FAPI State
     */
    Fapi_State              state;

    /**
     * @brief   Cell ID
     */
    Fapi_CellId             cellId;

    /**
     * @brief   Subframe numbers
     */
    uint16_t                sfnSf;

    /**
     * @brief   Subframe number
     */
    uint16_t                sfn;

    /**
     * @brief   Subframe
     */
    uint16_t                sf;

    /**
     * @brief   FAPI High Priority MSGCOM channel used to receive messages
     * from the L1
     */
    MsgCom_ChHandle         fapiL1L2HighPriorityChannel;

    /**
     * @brief   FAPI Low Priority MSGCOM channel used to receive messages
     * from the L1
     */
    MsgCom_ChHandle         fapiL1L2LowPriorityChannel;

    /**
     * @brief   FAPI High Priority MSGCOM channel used to send messages
     * to the L1
     */
    MsgCom_ChHandle         fapiL2L1HighPriorityChannel;

    /**
     * @brief   FAPI Low Priority MSGCOM channel used to send messages
     * to the L1
     */
    MsgCom_ChHandle         fapiL2L1LowPriorityChannel;

    /**
     * @brief   FAPI MAC PDU MSGCOM channel used to receive messages
     * from the L1
     */
    MsgCom_ChHandle         fapiL1L2MacPduChannel;

    /**
     * @brief   FAPI MAC PDU MSGCOM channel used to send messages
     * to the L1
     */
    MsgCom_ChHandle         fapiL2L1MacPduChannel;

    /**
     * @brief   FAPI DDR3 Heap
     */
    Pktlib_HeapHandle       fapiDDR3HeapHandle;

    /**
     * @brief  FAPI Tracing Heap
     */
    Pktlib_HeapHandle       fapiTracingHeap;

    /**
     * @brief  FAPI Memory Tracing Heap
     */
    Pktlib_HeapHandle       fapiMemTracingHeap;

    /**
     * @brief   FAPI Thread
     */
    pthread_t               fapiThread;

    /**
     * @brief   FAPI Tracing statistics
     */
    FapiTracing_Stats       stats;

    /**
     * @brief   PKTLIB Instance Handle associated with the domain
     */
    Pktlib_InstHandle       pktlibInstHandle;

    /**
     * @brief   MSGCOM Instance Handle associated with the domain
     */
    Msgcom_InstHandle       msgcomInstHandle;

    /**
     * @brief   FapiTracing instance handle
     */
    FapiTracing_InstHandle  fapiTracingHandle;

    /**
     * @brief   Pointer to the FAPI semaphore handle
     */
    sem_t*                  fapiSemaphoreHandle;

    /**
     * @brief   Status of the FAPI
     */
    int32_t                fapiStatus;

    /**
     * @brief   Pointer to the DL Config FAPI MIB in virtual HPLIB memory
     */
    uint8_t*                ptrDlConfigMIBRequest;

    /**
     * @brief   Pointer to the DL Config FAPI SIB1 in virtual HPLIB memory
     */
    uint8_t*                ptrDlConfigSIB1Request;

    /**
     * @brief   Pointer to the DL Config FAPI SIB2 in virtual HPLIB memory
     */
    uint8_t*                ptrDlConfigSIB2Request;

    /**
     * @brief   Pointer to the actual MIB request in virtual HPLIB memory
     */
    uint8_t*                ptrMIBRequest;

    /**
     * @brief   Pointer to the actual SIB1 request in virtual HPLIB memory
     */
    uint8_t*                ptrSIB1Request;

    /**
     * @brief   Pointer to the actual SIB2 request in virtual HPLIB memory
     */
    uint8_t*                ptrSIB2Request;

    /**
     * @brief   Pointer to the LTE Stack domain
     */
    AppLTEStackDomainMCB*   ptrLTEStackDomain;

    /**
     * @brief   Pointer to the memory base address allocated for FAPI memlog
     */
    uint32_t                memlogMemBase;

    /**
     * @brief   Size of the memory  allocated for FAPI memlog
     */
    uint32_t                memlogMemSize;

    /**
     * @brief   Memlog channel handle used for FAPI memlog
     */
    Memlog_ChHandle         memlogChanHandle;


    /**
     * @brief   Memlogging controller handle
     */
    Memlog_CtrlHandle       MemlogChanCtrl;

    /**
     * @brief   Thread to save FAPI tracing logs in memory
     */
    pthread_t               memlogThread;
}Fapi_MCB;

/**********************************************************************
 ************************** Global Variables **************************
 **********************************************************************/

/* Cell Configuration Request (This is just a template) */
TiFapi_configTLV_s configReqTLVs[]=
{
    {
        TI_FAPI_DUPLEXING_MODE,                //T: tag
        0x2,                                //L: length
        {0x1},                                //V: value
    },
    {
        TI_FAPI_PCFICH_POWER_OFFSET,        //T: tag
        0x2,                                //L: length
        {0x1770},                            //V: value
    },
    {
        TI_FAPI_P_B,                        //T: tag
        0x2,                                //L: length
        {0x1},                                //V: value
    },
    {
        TI_FAPI_DL_CYCLIC_PREFIX_TYPE,        //T: tag
        0x2,                                //L: length
        {0x0},                                //V: value
    },
    {
        TI_FAPI_UL_CYCLIC_PREFIX_TYPE,        //T: tag
        0x2,                                //L: length
        {0x0},                                //V: value
    },
    {
        TI_FAPI_DL_CHANNEL_BANDWIDTH,        //T: tag
        0x2,                                //L: length
        {0x64},                                //V: value
    },
    {
        TI_FAPI_UL_CHANNEL_BANDWIDTH,        //T: tag
        0x2,                                //L: length
        {0x64},                                //V: value
    },
    {
        TI_FAPI_REFERENCE_SIGNAL_POWER,        //T: tag
        0x2,                                //L: length
        {0x28},                                //V: value
    },
    {
        TI_FAPI_TX_ANTENNA_PORTS,            //T: tag
        0x2,                                //L: length
        {0x2},                                //V: value
    },
    {
        TI_FAPI_RX_ANTENNA_PORTS,            //T: tag
        0x2,                                //L: length
        {0x2},                                //V: value
    },
    {
        TI_FAPI_PHICH_RESOURCE,                //T: tag
        0x2,                                //L: length
        {0x0},                                //V: value
    },
    {
        TI_FAPI_PHICH_DURATION,                //T: tag
        0x2,                                //L: length
        {0x0},                                //V: value
    },
    {
        TI_FAPI_PHICH_POWER_OFFSET,            //T: tag
        0x2,                                //L: length
        {0x1770},                            //V: value
    },
    {
        TI_FAPI_PRIMARY_SYNC_SIGNAL,        //T: tag
        0x2,                                //L: length
        {0x1770},                            //V: value
    },
    {
        TI_FAPI_SECONDARY_SYNC_SIGNAL,        //T: tag
        0x2,                                //L: length
        {0x1770},                            //V: value
    },
    {
        TI_FAPI_PHYSICAL_CELL_ID,            //T: tag
        0x2,                                //L: length
        {0x1C0},                            //V: value
    },
    {
        TI_FAPI_CONFIGURATION_INDEX,        //T: tag
        0x2,                                //L: length
        {0x3},                                //V: value
    },
    {
        TI_FAPI_ROOT_SEQUENCE_INDEX,        //T: tag
        0x2,                                //L: length
        {0x7B},                                //V: value
    },
    {
        TI_FAPI_ZERO_CORRELATION_ZONE_CONFIGURATION,        //T: tag
        0x2,                                //L: length
        {0x9},                                //V: value
    },
    {
        TI_FAPI_HIGH_SPEED_FLAG,            //T: tag
        0x2,                                //L: length
        {0x0},                                //V: value
    },
    {
        TI_FAPI_FREQUENCY_OFFSET,            //T: tag
        0x2,                                //L: length
        {0x3},                                //V: value
    },
    {
        TI_FAPI_HOPPING_MODE,                //T: tag
        0x2,                                //L: length
        {0x0},                                //V: value
    },
    {
        TI_FAPI_HOPPIG_OFFSET,                //T: tag
        0x2,                                //L: length
        {0x0},                                //V: value
    },
    {
        TI_FAPI_NUM_OF_SUB_BANDS,            //T: tag
        0x2,                                //L: length
        {0x1},                                //V: value
    },
    {
        TI_FAPI_DELTA_PUCCH_SHIFT,            //T: tag
        0x2,                                //L: length
        {0x2},                                //V: value
    },
    {
        TI_FAPI_N_CQI_RB,                    //T: tag
        0x2,                                //L: length
        {0x0},                                //V: value
    },
    {
        TI_FAPI_N_AN_CS,                    //T: tag
        0x2,                                //L: length
        {0x0},                                //V: value
    },
    {
        TI_FAPI_N_1_PUCCH_AN,                //T: tag
        0x2,                                //L: length
        {0x5},                                //V: value
    },
    {
        TI_FAPI_BANDWIDTH_CONFIGURATION,    //T: tag
        0x2,                                //L: length
        {0x0},                                //V: value
    },
    {
        TI_FAPI_MAX_UP_PTS,                    //T: tag
        0x2,                                //L: length
        {0x0},                                //V: value
    },
    {
        TI_FAPI_SRS_SUB_FRAME_CONFIGURATION,//T: tag
        0x2,                                //L: length
        {0x0},                                //V: value
    },
    {
        TI_FAPI_SRS_ACK_NACK_SRS_SIMULTANEOUS_TRANSMISSION,        //T: tag
        0x1,                                //L: length
        {0x0},                                //V: value
    },
    {
        TI_FAPI_UPLINK_RS_HOPPING,            //T: tag
        0x2,                                //L: length
        {0x0},                                //V: value
    },
    {
        TI_FAPI_GROUP_ASSIGNMENT,            //T: tag
        0x2,                                //L: length
        {0x0},                                //V: value
    },
    {
        TI_FAPI_CYCLIC_SHIFT_1_FOR_DMRS,    //T: tag
        0x2,                                //L: length
        {0x0},                                //V: value
    },
    {
        TI_FAPI_DATA_REPORT_MODE,            //T: tag
        0x2,                                //L: length
        {0x0},                                //V: value
    },
    {
        TI_FAPI_HEADER_VERSION,                //T: tag
        0x2,                                //L: length
        {0x0107},                            //V: value
    }
};

TiFapi_configTLVExt_s configReqTLVExt_s[]=
{
    {
        TI_FAPI_AIF2_TRIGGER_TYPE,      //T: tag
        0x4,                            //L: length
        {0x0, 0x0,},                     //pad[2]
        {0},                            //V: value
    },
    {
        TI_FAPI_DL_SAMPLING_FREQ,        //T: tag
        0x4,                            //L: length
        {0x0, 0x0,},                     //pad[2]
        {30720},                        //V: value
    },
};

/* DL Config Request (This is just a template) */
TiFapi_dlCfgReq_s Fapi_dlConfigRequest[] =
{{
    0, //sfnSf
    3, //numOfdmSymbPdcch
    0, //numDlDciPdus
    0, //numBchPdus;
    0, //numDlschPdus;
    0, //numMchPdus;
    0, //numPchPdus;
    0, //numPrsPdus;
    0, //numCsiRsPdus;
    0, //numPdschRntis;
    (uint16_t)0x1770,
    0,  //numBfLayers
}};

/* DL Config Request with MIB (This is just a template) */
TiFapi_dlCfgReq_s Fapi_dlConfigMIBRequest[] =
{{
    0, //sfnSf
    3, //numOfdmSymbPdcch
    0, //numDlDciPdus
    1, //numBchPdus;
    0, //numDlschPdus;
    0, //numMchPdus;
    0, //numPchPdus;
    0, //numPrsPdus;
    0, //numCsiRsPdus;
    0, //numPdschRntis;
    (uint16_t)0x1770,
    0,  //numBfLayers

    /* The BCH PDU configuration.*/
    {
        3, //len
        0, //pdu index
        0x1770  //tx power
    },
}};

TiFapi_dlCfgReq_s Fapi_dlConfigSIB1Request =
{
    0, //sfnSf
    3, //numOfdmSymbPdcch
    1, //numDlDciPdus
    0, //numBchPdus;
    1, //numDlschPdus;
    0, //numMchPdus;
    0, //numPchPdus;
    0, //numPrsPdus;
    0, //numCsiRsPdus;
    1, //numPdschRntis;
    (uint16_t)0x1770,
    0, //numBfLayers
    /* The BCH PDU configuration.*/
    {0, 0, 0},

    /* The PRS PDU configuration.*/
    {0},

    /* DCI PDU */
    {
        {

           1,            //          dciFormat;                  /* DL DCI format*/
           0,            //          cceIndex;                   /* The CCE index used to send the DCI*/
           0x4,          //          aggregationLevel;           /* The aggregation level used*/
           0xffff,       //          rnti;                       /* The Radio Network Temporary Identifier used for identifying the UE*/
           2,            //          resourceAllocationType;     /* Resource allocation type*/
           TI_FAPI_LOCALISED, //     virtualRbType;              /* VRB assignment flag*/
           200,          //          resourceBlockCoding;        /* Resource block coding. Specifies the  Resource allocation. The value depends upon the resource allocation type*/
           4,            //          mcsTb0;                     /* The MCS for TB 0*/
           0,            //          redundancyVersionTb0;       /* The HARQ redundancy version for TB 0*/
           0,            //          newDataIndicatorTb0;        /* New data indicator for TB 0*/
           0,            //          tbSwapFlag;                 /* Indicates the mapping of transport blocks to codewords*/
           0,            //          mcsTb1;                     /* The MCS for TB 1*/
           0,            //          redundancyVersionTb1;       /* The HARQ redundancy version for TB 1*/
           0,            //          newDataIndicatorTb1;        /* New data indicator for TB 1*/
           0,            //          harqProcessNumber;          /* The HARQ process number*/
           0,            //          tpmi;                       /* The codebook index used for precoding. Valid for DCI formats 1B and 1D*/
           0,            //          pmi;                        /* Confirmation of the precoding Valid for DCI format 1B*/
           0,            //          precodingInformation;       /* Precoding information. Valid for DCI formats 2 and 2A*/
           1,            //          tpc;                        /* Tx power control command for PUCCH. Valid for DCI formats 1, 1A, 1B, 1D, 2 and 2A*/
           0,            //          downlinkAssignmentIndex;    /* Downlink assignment index. Valid for DCI formats 1, 1A, 1B, 1D, 2, 2A*/
           0,            //          nGap;                       /* N_gap; used for VRB. Valid for DCI formats 1B, 1C, 1D*/
           0xd0,         //          tbsIndex;                   /* TBS index. Valid for DCI formats 1C*/
           0,            //          dlPowerOffset;              /* DL power offset. Valid for DCI formats 1D*/
           0,            //          allocatePrachFlag;          /* Allocate PRACH flag. Valid for DCI formats 1A*/
           0x40,         //          preambleIndex;              /* Preamble index. Valid for DCI formats 1A*/
           0x1,          //          prachMaskIndex;             /* PRACH mask index. Valid for DCI formats 1A*/
           TI_FAPI_SPS_CRNTI,//      rntiType;                   /* RNTI type. Valid for DCI format 1, 1A, 2, 2A*/
           0x1770,       //          txPower;                    /* Offset to the RS power*/
        },

        /* empty DCI pdus */
        {0}, {0}, {0}, {0}, {0}, {0},

        /* the following is for 8 UEs */
        {0}, {0}, {0}, {0},

    },

    /* DLSCH PDU */
    {
        /* DLSCH pdu for SIB1 request */
        {
            0x1a,             //       length;                     /* The length in bytes of the DLSCH PDU*/
            0,                //       pduIdx;                     /* A count value incremented for each BCH, MCH, PCH or DLSCH PDU*/
            0xffff,           //       rnti;                       /* The RNTI associated with the DLSCH*/
            TI_FAPI_RES_ALLOC_TYPE_2,//          resourceAllocationType;     /* Resource allocation type*/
            TI_FAPI_LOCALISED, //      virtualRbType;              /* VRB assignment flag*/
            200,              //       resourceBlockCoding;        /* Resource block coding Specifies the  Resource allocation The value depends upon the resource allocation type*/
            TI_FAPI_QPSK,     //       modulation;                 /* The modulation type*/
            0,                //       redundancyVersion;          /* The HARQ redundancy version*/
            1,                //       transportBlocks;            /* A value of 2 indicates the second TB for MIMO Else always set to 1*/
            0,                //       tbSwapFlag;                 /* Indicates the mapping of transport blocks to codewords*/
            TI_FAPI_TX_DIVERSITY, //   txScheme;                   /* The transmission scheme used for this PDU*/
            2,                //       numLayers;                  /* The number of layers used in transmission*/
            0,                //       numSubbands;                /* Only valid when transmission scheme = 3,4,5*/
                        //codebookIdx[TI_FAPI_DLSCH_PDU_CFG_MAX_SUBBANDS];    /* Only valid when tx scheme = 3,4,5 Defined the codebook used*/
                {0,0,0,0,0,0,0,0,0,0,0,0,0},

            0x3,            //       ueCategory;                 /* The UE capabilities category*/
            0x4,            //       pa;                         /* The ratio of PDSCH EPRE to cell-specific RS EPRE among PDSCH REs in all the OFDM symbols not containing cell-specific RS*/
            0x0,            //       deltaPowerOffsetIdx;        /* Delta power offset, value 0,1*/
            0x0,            //       nGap;                       /* Used in the virtual resource block distribution*/
            0x1,            //       nPrb;                       /* Used with DCI format 1A and SI-RNTI or RA-RNTI Value should match with the TPC field in the corresponding DCI 1A PDU*/
            0x0,            //       numBfPrbPerSubband;         /* Number of PRBs that are treated as one subband*/
            0,              //       numBfVector;                /* Number of beam forming vectors*/
        },

      /* empty DLSCH pdus */
      {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0},

      /* the following is for 8 UEs*/
      {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}
    }
};

/* DL Config Request with SIB2 (This is just a template) */
TiFapi_dlCfgReq_s Fapi_dlConfigSIB2Request =
{
    0, //sfnSf
    3, //numOfdmSymbPdcch
    1, //numDlDciPdus
    0, //numBchPdus;
    1, //numDlschPdus;
    0, //numMchPdus;
    0, //numPchPdus;
    0, //numPrsPdus;
    0, //numCsiRsPdus;
    1, //numPdschRntis;
    (uint16_t)0x1770,
    0, //numBfLayers

    /* The BCH PDU configuration.*/
    {0, 0, 0},

    /* The PRS PDU configuration.*/
    {0},

    /* DCI PDU */
    {
        /* DCI for SIB2 request */
        {
           1,  //          dciFormat;                  /* DL DCI format*/
           0,  //          cceIndex;                   /* The CCE index used to send the DCI*/
           0x4,  //                      aggregationLevel;           /* The aggregation level used*/
           0xffff,       //             rnti;                       /* The Radio Network Temporary Identifier used for identifying the UE*/
           2, //          resourceAllocationType;     /* Resource allocation type*/
           TI_FAPI_LOCALISED, //    virtualRbType;              /* VRB assignment flag*/
           200,//                      resourceBlockCoding;        /* Resource block coding. Specifies the  Resource allocation. The value depends upon the resource allocation type*/
           4, //                      mcsTb0;                     /* The MCS for TB 0*/
           0,   //                     redundancyVersionTb0;       /* The HARQ redundancy version for TB 0*/
           0, //                       newDataIndicatorTb0;        /* New data indicator for TB 0*/
           0, // tbSwapFlag;                 /* Indicates the mapping of transport blocks to codewords*/
           0,  //                       mcsTb1;                     /* The MCS for TB 1*/
           0, //                       redundancyVersionTb1;       /* The HARQ redundancy version for TB 1*/
           0, //                       newDataIndicatorTb1;        /* New data indicator for TB 1*/
           0, //                       harqProcessNumber;          /* The HARQ process number*/
           0, //                       tpmi;                       /* The codebook index used for precoding. Valid for DCI formats 1B and 1D*/
           0, //      pmi;                        /* Confirmation of the precoding Valid for DCI format 1B*/
           0, //                       precodingInformation;       /* Precoding information. Valid for DCI formats 2 and 2A*/
           1, //             tpc;                        /* Tx power control command for PUCCH. Valid for DCI formats 1, 1A, 1B, 1D, 2 and 2A*/
           0, //                        downlinkAssignmentIndex;    /* Downlink assignment index. Valid for DCI formats 1, 1A, 1B, 1D, 2, 2A*/
           0, //             nGap;                       /* N_gap; used for VRB. Valid for DCI formats 1B, 1C, 1D*/
           0xd0, //                       tbsIndex;                   /* TBS index. Valid for DCI formats 1C*/
           0, //                       dlPowerOffset;              /* DL power offset. Valid for DCI formats 1D*/
           0,    //allocatePrachFlag;          /* Allocate PRACH flag. Valid for DCI formats 1A*/
           0x40,  //                       preambleIndex;              /* Preamble index. Valid for DCI formats 1A*/
           0x1, //                       prachMaskIndex;             /* PRACH mask index. Valid for DCI formats 1A*/
           TI_FAPI_SPS_CRNTI,  //             rntiType;                   /* RNTI type. Valid for DCI format 1, 1A, 2, 2A*/
           0x1770, //                      txPower;                    /* Offset to the RS power*/
        },

        /* empty DCI pdus */
        {0}, {0}, {0}, {0}, {0}, {0},

        /* The following is for 8 UEs */
        {0}, {0}, {0}, {0},
    },

    /* DLSCH PDU */
    {
        /* DLSCH pdu for SIB2 request */
        {
            0x1a, //                      length;                     /* The length in bytes of the DLSCH PDU*/
            0, //                      pduIdx;                     /* A count value incremented for each BCH, MCH, PCH or DLSCH PDU*/
            0xffff, //                      rnti;                       /* The RNTI associated with the DLSCH*/
            TI_FAPI_RES_ALLOC_TYPE_2, //          resourceAllocationType;     /* Resource allocation type*/
            TI_FAPI_LOCALISED, //    virtualRbType;              /* VRB assignment flag*/
            200, //                      resourceBlockCoding;        /* Resource block coding Specifies the  Resource allocation The value depends upon the resource allocation type*/
            TI_FAPI_QPSK, //       modulation;                 /* The modulation type*/
            0, //                       redundancyVersion;          /* The HARQ redundancy version*/
            1, //                       transportBlocks;            /* A value of 2 indicates the second TB for MIMO Else always set to 1*/
            0, // tbSwapFlag;                 /* Indicates the mapping of transport blocks to codewords*/
            TI_FAPI_TX_DIVERSITY, //          txScheme;                   /* The transmission scheme used for this PDU*/
            2, //                       numLayers;                  /* The number of layers used in transmission*/
            0, //                       numSubbands;                /* Only valid when transmission scheme = 3,4,5*/
            //codebookIdx[TI_FAPI_DLSCH_PDU_CFG_MAX_SUBBANDS];    /* Only valid when tx scheme = 3,4,5 Defined the codebook used*/
                {0,0,0,0,0,0,0,0,0,0,0,0,0},

            0x3, //                       ueCategory;                 /* The UE capabilities category*/
            0x4, //              pa;                         /* The ratio of PDSCH EPRE to cell-specific RS EPRE among PDSCH REs in all the OFDM symbols not containing cell-specific RS*/
            0x0, //                       deltaPowerOffsetIdx;        /* Delta power offset, value 0,1*/
            0x0, //             nGap;                       /* Used in the virtual resource block distribution*/
            0x1, //            nPrb;                       /* Used with DCI format 1A and SI-RNTI or RA-RNTI Value should match with the TPC field in the corresponding DCI 1A PDU*/
            0x0, //                       numBfPrbPerSubband;         /* Number of PRBs that are treated as one subband*/
            0, //                       numBfVector;                /* Number of beam forming vectors*/
        },

      /* empty DLSCH pdus */
      {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0},

      /* The following is for 8 UEs*/
      {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}
    }
};

/* UL Config Request (This is just a template) */
TiFapi_ulCfgReq_s  Fapi_ulConfigRequest[] =
{{
    0x0000,        //    sfnSf;                                          /* SFN/SF*/
    0,         //    numUlschPdus;                                   /* The number of PDUs in the ulsch array*/
    0,        //  numUciPdus;                                     /* The number of PDUs in the uci array*/
    0,        //  numSrsPdus;                                     /* The number of PDUs in the srs array*/
    0,        //  prachResource;                                  /* For FDD 0 means no RACH in this sf, 1 means rach present in this sf;*/
                                                                 /*for TDD bits 0:5 indicates the RACH resources used*/
    0,        //   srsPresent;                                     /* The SRS present flag*/
}};

/* UL Config Request (This is just a template) */
TiFapi_ulCfgReq_s  Fapi_ulConfigRequestWithULData[] =
{{
    0x0000,        //    sfnSf;                                          /* SFN/SF*/
    1,         //    numUlschPdus;                                   /* The number of PDUs in the ulsch array*/
    0,        //  numUciPdus;                                     /* The number of PDUs in the uci array*/
    0,        //  numSrsPdus;                                     /* The number of PDUs in the srs array*/
    0,        //  prachResource;                                  /* For FDD 0 means no RACH in this sf, 1 means rach present in this sf;*/
                                                                 /*for TDD bits 0:5 indicates the RACH resources used*/
    0,        //   srsPresent;                                     /* The SRS present flag*/

    /* Ulsch pdus */
    {{
        0,     //    pduType
        {{{0x7054,     //                           handle;                    /* An opaque handle that will be returned to L23*/
        0xf7b,         //                           size;                      /* The size of the PDU in bytes Can be 0 in case that only HARQ and CQI is received on ULSCH*/
        0x33,         //                         rnti;                      /* The RNTI used for identifying the UE when receiving the PDU*/
        0,            //                         resourceBlockStart;        /* Resource indication value corresponding to the starting resource block*/
        0x40,        //                          numRbs;                    /* The number of resource blocks this PDU will be mapped to*/
        0x4,        //                           modulationType;            /* Modulation Type*/
        0,             //                           n2Dmrs;                    /* The 2nd cyclic shift for DMRS assigned to the UE in the relevant UL grant*/
        0,            //                             hoppingFlag;               /* Frequency hopping flag*/
        0x0,        //                           hoppingBits;               /* Frequency hopping bits*/
        0x1,         //                           newDataIndicator;          /* New data indicator for TB 0 Specify whether this received PUSCH is a new transmission from UE*/
        0x0,        //                           redundancyVersion;         /* Redundancy Version*/
        0x0,        //                           harqProcessNumber;         /* The HARQ process number*/
        0x0,        //                             txMode;                    /* UL Tx Mode*/
        0x0,        //                           currentTxNb;               /* The current HARQ transmission count of this TX Valid if frequency hopping is enabled*/
        0x0,}}}        //                           nSrs;                      /* Indicates if the RBs allocated for this grant overlap with the SRS configuration*/
    },},
}};
/* System Information Block1 Request (This is just a template) */
uint8_t Fapi_sib1[] =
{
    0x48,0x66,0x1e,0xca,0x93,0x13,0x60,0x00,0x01,0xc0,0xc5,0x00,0x29,0x03,0x08,0x18,
    0x42,0xc2,0x26,0x11,0xb0,0x99,0x81,0x55,0x60,0x00
};

/* System Information Block (This is just a template) */
uint8_t Fapi_sib2[] =
{
    0x00,0x00,0x02,0x7c,0xfd,0x0c,0x87,0xb0,0xd2,0x11,0xe2,0x80,0x00,0x04,0x10,0x00,
    0xa3,0x3b,0xb0,0x00,0x9f,0xa0,0x83,0x41,0xc0,0x00
};

/* Master Information Block Request (This is just a template) */
uint8_t Fapi_mib[3] =
{
    0xa2,0x84,0x00
};

extern void debugMemMonitor (uint32_t id);

/* eNodeB IPv4 Address. Used for FAPI Fast path creation */
extern uint8_t eNodeBIPAddress[4];

/* Logger IPv4 Address. Used for FAPI Fast path creation */
uint8_t loggerIPAddress[4] = {192, 168, 1, 1};

/* eNodeB IPv4 Address. Used for FAPI Fast path creation */
char eNodeBIPv6Address[32] = "7000::2";

/* Logger IPv4 Address. Used for FAPI Fast path creation */
char loggerIPv6Address[32] = "7000::1";

/* Logger MAC Address. Used only when NetFP Proxy is not used */
uint8_t loggerMACAddress[6] = {0x08, 0x00, 0x27, 0xb2, 0x0c, 0x65};

/* Memlog thread termination flag */
uint32_t gMemlogThreadTerminate = 0;

/**********************************************************************
 ************************** FAPI Functions ****************************
 **********************************************************************/

/**
 *  @b Description
 *  @n
 *     The function is used by the FapiTracing module to allocate memory for its
 *     internal data structures.
 *
 *  @param[in]  mode
 *      Memory mode
 *  @param[in]  numBytes
 *      Number of bytes to be allocated
 *
 *  @retval
 *      Success -   Pointer to the allocated memory
 *  @retval
 *      Error   -   NULL
 */
static void* FapiTracing_osalMalloc(uint32_t numBytes, uint32_t alignment)
{
    return memalign (alignment, numBytes);
}

/**
 *  @b Description
 *  @n
 *     The function is used by the FapiTracing module to free memory which had been
 *     previously allocated
 *
 *  @param[in]  ptr
 *      Pointer to the memory to be freed
 *  @param[in]  numBytes
 *      Number of bytes to be freed
 *
 *  @retval
 *      Not applicable
 */
static void FapiTracing_osalFree(void* ptr, uint32_t numBytes)
{
    free (ptr);
}

/**
 *  @b Description
 *  @n
 *      The function is used to cleanup the MSGCOM data buffers if there are
 *      any pending packets while deleting the channel.
 *
 *  @param[in]  pktlibInstHandle
 *      PKTLIB instance handle
 *  @param[in]  chHandle
 *      MSGCOM Channel Handle which is being deleted.
 *  @param[in]  msgBuffer
 *      MSGCOM Buffer to be deleted.
 *
 *  @retval
 *      Not Applicable.
 */
static void myFreeMsgBuffer(Pktlib_InstHandle pktlibInstHandle, MsgCom_ChHandle chHandle, MsgCom_Buffer* msgBuffer)
{
    printf ("Debug: Channel Handle 0x%p is being deleted MSGCOM Buffer is 0x%p\n", chHandle, msgBuffer);
    Pktlib_freePacket(pktlibInstHandle, (Ti_Pkt*)msgBuffer);
}

/**
 *  @b Description
 *  @n
 *      Heap Data Buffer Allocator which is instantiated into the heap
 *      interface table.
 *
 *  @param[in]  size
 *      Size of the memory which is to be allocated.
 *  @param[in]  arg
 *      FAPI Module specific argument
 *
 *  @retval
 *      Success     -   Pointer to the allocated block of memory.
 *  @retval
 *      Error       -   NULL
 */
static uint8_t* Fapi_DDR3MemoryMalloc(uint32_t size, uint32_t arg)
{
    return hplib_vmMemAlloc (size, 0, 0);
}

/**
 *  @b Description
 *  @n
 *      Heap Data Buffer Cleanup which is instantiated into the heap
 *      interface table.
 *
 *  @param[in]  ptr
 *      Pointer to the memory which is to be cleaned up.
 *  @param[in]  size
 *      Size of the memory which is to be cleaned up.
 *  @param[in]  arg
 *      FAPI Module specific argument
 *
 *  @retval
 *      Not Applicable.
 */
static void Fapi_DDR3MemoryFree(uint8_t* ptr, uint32_t size, uint32_t arg)
{
    /* TODO: Currently HPLIB does not support free */
}

/**
 *  @b Description
 *  @n
 *      The function is used to allocate a message of the specific
 *      message type.
 *
 *  @param[in]  ptrFAPI
 *      Pointer to the FAPI MCB
 *  @param[in]  msgType
 *      Message type
 *  @param[in]  ptrPayload
 *      Pointer to the payload. This is not required for all message types
 *  @param[in]  payloadLen
 *      Length of the payload. This is not required for all message types
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Fapi_sendMessage
(
    Fapi_MCB*   ptrFAPI,
    uint8_t     msgType,
    uint8_t*    ptrPayload,
    uint32_t    payloadLen
)
{
    Ti_Pkt*             ptrFAPIPkt;
    TiFapi_msg_s*       ptrFAPIMessage;
    uint32_t            messageLength;
    MsgCom_ChHandle     fapiChannel;
    int32_t                errCode;

    /* Populate the message on the basis of the message type */
    if (msgType == TI_FAPI_PARAM_REQUEST)
    {
        /* Allocate memory from the FAPI DDR3 Heap. */
        ptrFAPIPkt = Pktlib_allocPacket(ptrFAPI->pktlibInstHandle,
                                        ptrFAPI->fapiDDR3HeapHandle, sizeof(TiFapi_msg_s));
        if (ptrFAPIPkt == NULL)
        {
            printf ("Error: Unable to allocate the FAPI Message [PARAM Request]\n");
            return -1;
        }

        /* Get the FAPI message */
        Pktlib_getDataBuffer(ptrFAPIPkt, (uint8_t**)&ptrFAPIMessage, &messageLength);

        /* Populate the FAPI message */
        ptrFAPIMessage->msgId             = msgType;
        ptrFAPIMessage->lenVendorSpecific = 0;
        ptrFAPIMessage->msgLen            = 0;

        /* Send the message on the high priority channel */
        fapiChannel = ptrFAPI->fapiL2L1HighPriorityChannel;
    }
    else if (msgType == TI_FAPI_CELL_CONFIG_REQUEST)
    {
        TiFapi_configReq_s      *ptrConfigReqMsg;
        uint32_t               numOfExtTLVs, numOfStdTLVs, numOfTLVs;
        void                  *tempPtr;
        uint32_t              tlvIdx=0;

        numOfStdTLVs = sizeof(configReqTLVs)/sizeof(TiFapi_configTLV_s);
        numOfExtTLVs = sizeof(configReqTLVExt_s)/sizeof(TiFapi_configTLVExt_s);

        /* Standard TLVs + Proprietary(extension) TLVs */
        numOfTLVs = numOfStdTLVs + numOfExtTLVs;

        /*
         * Calculate message length(standard),
         * this calculation does not include AIF TLVs
         */
        messageLength = sizeof(TiFapi_nonTlvMsgHeader_s) +
                        sizeof(TiFapi_configReq_s) +
                        (sizeof(TiFapi_configTLV_s) * (numOfStdTLVs - 1));

        /* Add message length(extention) */
        messageLength += (sizeof(configReqTLVExt_s) * (numOfExtTLVs - 1));

        /* Allocate memory from the FAPI DDR3 Heap. */
        ptrFAPIPkt = Pktlib_allocPacket(ptrFAPI->pktlibInstHandle,
                                        ptrFAPI->fapiDDR3HeapHandle, messageLength);
        if (ptrFAPIPkt == NULL)
        {
            printf ("Error: Unable to allocate the FAPI Message [Cell Config]\n");
            return -1;
        }

        /* Get the FAPI message */
        Pktlib_getDataBuffer(ptrFAPIPkt, (uint8_t**)&ptrFAPIMessage, &messageLength);

        /* Populate the FAPI message */
        ptrFAPIMessage->msgId             = msgType;
        ptrFAPIMessage->lenVendorSpecific = 0;
        /* Set FAPI message body length based on number of TLVs */
        ptrFAPIMessage->msgLen  = messageLength -
                                   sizeof(TiFapi_nonTlvMsgHeader_s);

        /* Copy the hard-coded cell configuration request to config.req structure */
        ptrConfigReqMsg                   = (TiFapi_configReq_s *)ptrFAPIMessage->msgBody;

        /* Determine number of TLVs from configReqTLV array */
        ptrConfigReqMsg->numOfTlv           = numOfTLVs;
        memset((void *)ptrConfigReqMsg->pad, 0, sizeof(ptrConfigReqMsg->pad));

        /* Populate Standard TLVs in config.req message */
        memcpy ((void *)ptrConfigReqMsg->configTLVs, (void *)&configReqTLVs[0],
                    sizeof(TiFapi_configTLV_s) * numOfStdTLVs);

        /*
         * Populate Proprietary TLVs in config.req message
         */
        tempPtr = (void *)((uint8_t *)ptrConfigReqMsg->configTLVs +
                              (sizeof(TiFapi_configTLV_s) * numOfStdTLVs));

        for (tlvIdx=0; tlvIdx < numOfExtTLVs; tlvIdx++)
        {
            memcpy (tempPtr, (void *)&configReqTLVExt_s[tlvIdx],
                                     (4+configReqTLVExt_s[tlvIdx].tagLen));

            tempPtr = (void *)((uint8_t *)tempPtr +
                                     (4+configReqTLVExt_s[tlvIdx].tagLen));
        }

        /* Send the message on the high priority channel */
        fapiChannel = ptrFAPI->fapiL2L1HighPriorityChannel;
    }
    else if (msgType == TI_FAPI_START_REQUEST)
    {
        /* Allocate memory from the FAPI DDR3 Heap. */
        ptrFAPIPkt = Pktlib_allocPacket(ptrFAPI->pktlibInstHandle,
                                        ptrFAPI->fapiDDR3HeapHandle, sizeof(TiFapi_msg_s));
        if (ptrFAPIPkt == NULL)
        {
            printf ("Error: Unable to allocate the FAPI Message [START Request]\n");
            return -1;
        }

        /* Get the FAPI message */
        Pktlib_getDataBuffer(ptrFAPIPkt, (uint8_t**)&ptrFAPIMessage, &messageLength);

        /* Populate the FAPI message */
        ptrFAPIMessage->msgId             = msgType;
        ptrFAPIMessage->lenVendorSpecific = 0;
        ptrFAPIMessage->msgLen            = 0;

        /* Send the message on the high priority channel */
        fapiChannel = ptrFAPI->fapiL2L1HighPriorityChannel;
    }
    else if (msgType == TI_FAPI_DL_CONFIG_REQUEST)
    {
        TiFapi_dlCfgReq_s*   ptrDLConfigRequest;
        uint32_t             dlConfigMsgSize;

        /* Initialize the size to the size of TiFapi_dlCfgReq_s, it will be compressed later */
        dlConfigMsgSize = sizeof(TiFapi_dlCfgReq_s);

        /* Allocate memory from the FAPI DDR3 Heap. */
        ptrFAPIPkt = Pktlib_allocPacket(ptrFAPI->pktlibInstHandle,
                                        ptrFAPI->fapiDDR3HeapHandle,
                                        sizeof(TiFapi_nonTlvMsgHeader_s) + dlConfigMsgSize);
        if (ptrFAPIPkt == NULL)
        {
            printf ("Error: Unable to allocate the FAPI Message [DL Config]\n");
            return -1;
        }

        /* Get the FAPI message */
        Pktlib_getDataBuffer(ptrFAPIPkt, (uint8_t**)&ptrFAPIMessage, &messageLength);

        /* Get the DL FAPI Config request */
        ptrDLConfigRequest = (TiFapi_dlCfgReq_s*)&ptrFAPIMessage->msgBody[0];

        /* Populate the FAPI message */
        ptrFAPIMessage->msgId             = msgType;
        ptrFAPIMessage->lenVendorSpecific = 0;
        /* The msgLen is the length of the FAPI message body, does not include hearder.
           It should be set by Stack using the real message body length
           In this demo, we just set it to the size of TiFapi_dlCfgReq_s */
        ptrFAPIMessage->msgLen            = sizeof(TiFapi_dlCfgReq_s);

        /* Populate the DL Configuration request message appropriately. */
        if ((ptrFAPI->sf == 0) && ((ptrFAPI->sfn % 4) == 0))
            memcpy ((void *)ptrFAPIMessage->msgBody, (void *)ptrFAPI->ptrDlConfigMIBRequest, sizeof(Fapi_dlConfigMIBRequest));
        else if ((ptrFAPI->sf == 5) && ((ptrFAPI->sfn % 2) == 0))
            memcpy ((void *)ptrFAPIMessage->msgBody, (void *)ptrFAPI->ptrDlConfigSIB1Request, sizeof(Fapi_dlConfigSIB1Request));
        else if ((ptrFAPI->sf == 6) && ((ptrFAPI->sfn % 16) == 0))
            memcpy ((void *)ptrFAPIMessage->msgBody, (void *)ptrFAPI->ptrDlConfigSIB2Request , sizeof(Fapi_dlConfigSIB2Request));
        else
            memcpy ((void *)ptrFAPIMessage->msgBody, (void *)&Fapi_dlConfigRequest[0] , sizeof(Fapi_dlConfigRequest));

        /* Overwrite the SFN */
        ptrDLConfigRequest->sfnSf = ptrFAPI->sfnSf;

        /* Send the message on the low priority channel */
        fapiChannel = ptrFAPI->fapiL2L1LowPriorityChannel;

        /* Compress the packet and corrected the msglen in FAPI header */
        ptrFAPIMessage->msgLen            = FapiTracing_compressDlConfig((TiFapi_nonTlvMsg_s *)ptrFAPIMessage);
        messageLength = sizeof(TiFapi_nonTlvMsgHeader_s) + ptrFAPIMessage->msgLen;
    }
    else if (msgType == TI_FAPI_UL_CONFIG_REQUEST)
    {
        TiFapi_ulCfgReq_s*   ptrULConfigRequest;

        /* Allocate memory from the FAPI DDR3 Heap. */
        ptrFAPIPkt = Pktlib_allocPacket(ptrFAPI->pktlibInstHandle,
                                        ptrFAPI->fapiDDR3HeapHandle,
                                        sizeof(TiFapi_nonTlvMsgHeader_s) + sizeof(TiFapi_ulCfgReq_s));
        if (ptrFAPIPkt == NULL)
        {
            printf ("Error: Unable to allocate the FAPI Message [UL Config]\n");
            return -1;
        }

        /* Get the FAPI message */
        Pktlib_getDataBuffer(ptrFAPIPkt, (uint8_t**)&ptrFAPIMessage, &messageLength);

        /* Populate the FAPI message */
        ptrFAPIMessage->msgId             = msgType;
        ptrFAPIMessage->lenVendorSpecific = 0;
        ptrFAPIMessage->msgLen            = sizeof(Fapi_ulConfigRequest);

        /* Get the UL FAPI Config request */
        ptrULConfigRequest = (TiFapi_ulCfgReq_s*)&ptrFAPIMessage->msgBody[0];

        /* Copy the hard-coded cell configuration request */
        memcpy ((void *)ptrFAPIMessage->msgBody, (void *)&Fapi_ulConfigRequest[0] , sizeof(Fapi_ulConfigRequest));

        /* Overwrite the SFN */
        ptrULConfigRequest->sfnSf = ptrFAPI->sfnSf;

        /* Send the message on the low priority channel */
        fapiChannel = ptrFAPI->fapiL2L1LowPriorityChannel;

        /* Compress the packet and corrected the msglen in FAPI header */
        ptrFAPIMessage->msgLen            = FapiTracing_compressUlConfig((TiFapi_nonTlvMsg_s *)ptrFAPIMessage);
        messageLength = sizeof(TiFapi_nonTlvMsgHeader_s) + ptrFAPIMessage->msgLen;

    }
    else if (msgType == TI_FAPI_DL_TX_REQUEST)
    {
        TiFapi_txReq_s*     ptrFapiTxRequest;
        uint8_t*            ptrDataBuffer;
        uint32_t            memorySize;
        uint32_t            pduMsgLen;
        Ti_Pkt*             ptrPayloadPkt;


        /* The payload packet should be allocated and constructed at MAC layer.
           The following code is used to demonstrate merging tx.req packet with MAC PDU packet.
           If MAC PDU is already passed in as a Ti_Pkt handle, then allocate packet and memcpy 
           are not needed. 
           */
        ptrPayloadPkt = Pktlib_allocPacket(ptrFAPI->pktlibInstHandle,
                                        ptrFAPI->fapiDDR3HeapHandle, payloadLen);
        if (ptrPayloadPkt == NULL)
        {
            printf ("Error: Unable to allocate the  DL Tx Request Payload packet \n");
            return -1;
        }

        /* Get the data buffer associated with the payload packet. */
        Pktlib_getDataBuffer(ptrPayloadPkt, (uint8_t**)&ptrDataBuffer, &messageLength);

        /* Copy the payload */
        memcpy(ptrDataBuffer, ptrPayload, payloadLen);

        /* Find out the pduLen for Embedded mode */
        pduMsgLen    = (offsetof(TiFapi_txReq_s, u) + sizeof(TiFapi_txReqEmbeddedPdu_s));

        /* construct a tx.request message ,
         if the application cannot provide memorySize, then use the fillowing length for allocation */
        memorySize = sizeof(TiFapi_nonTlvMsgHeader_s) + pduMsgLen;

        /* Allocate memory from the FAPI DDR3 Heap. */
        ptrFAPIPkt = Pktlib_allocPacket(ptrFAPI->pktlibInstHandle,
                                        ptrFAPI->fapiDDR3HeapHandle, memorySize);
        if (ptrFAPIPkt == NULL)
        {
            printf ("Error: Unable to allocate the FAPI Message [DL Tx Request]\n");
            return -1;
        }

        /* Get the data buffer associated with the FAPI packet. */
        Pktlib_getDataBuffer(ptrFAPIPkt, (uint8_t**)&ptrDataBuffer, &messageLength);

        /* Populate the FAPI message */
        ptrFAPIMessage = (TiFapi_msg_s*)ptrDataBuffer;
        ptrFAPIMessage->msgId             = msgType;
        ptrFAPIMessage->lenVendorSpecific = 0;
        ptrFAPIMessage->msgLen            = pduMsgLen + payloadLen;

        /* Get the FAPI DL Tx request */
        ptrFapiTxRequest = (TiFapi_txReq_s*)&ptrFAPIMessage->msgBody[0];

        /* Initialize the FAPI DL Transmit request  */
        ptrFapiTxRequest->sfnSf     = ptrFAPI->sfnSf;

        /* Embedded FAPI mode */
        ptrFapiTxRequest->numPdus   = 0;
        ptrFapiTxRequest->u.pduInfo.pduLength   = (uint32_t)payloadLen;;
        ptrFapiTxRequest->u.pduInfo.pduIndex    = 0;

        /* Merge Tx Request packet with payload packet */
        Pktlib_packetMerge(ptrFAPI->pktlibInstHandle, ptrFAPIPkt, ptrPayloadPkt, NULL);

        /* Send the message on the MAC PDU channel */
        fapiChannel = ptrFAPI->fapiL2L1MacPduChannel;

        /* Compress the packet is not needed in embedded mode */
        ptrFAPIMessage->msgLen = FapiTracing_compressTxRequest((TiFapi_nonTlvMsg_s *)ptrFAPIMessage) + payloadLen;

        /* Calculate the message length */
        messageLength = sizeof(TiFapi_nonTlvMsgHeader_s) +  ptrFAPIMessage->msgLen;
    }
    else
    {
        /* Currently no other message is supported. */
        return -1;
    }

    /* Set the packet length. */
    Pktlib_setPacketLen(ptrFAPIPkt, messageLength);

    /* Pass the message to the FAPI Tracing Module.
     *  - Trace the entire packet which is being transmitted */
    if(FapiTracing_trace(ptrFAPI->fapiTracingHandle, ptrFAPIPkt, &errCode) < 0)
        ptrFAPI->stats.DroppedPackets++;
    ptrFAPI->stats.TotalPackets++;

    /* Log Message: */
    Log_write3(UIAEvt_detailWithStr, 0x11, (IArg)"Stack Sending FAPI Message with id %x", msgType);

    /* Send the message out. */
    Msgcom_putMessage(fapiChannel, (MsgCom_Buffer*)ptrFAPIPkt);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to process all the FAPI messages received on
 *      the specific FAPI channel.
 *
 *  @param[in]  ptrFAPI
 *      Pointer to the FAPI MCB
 *  @param[in]  fapiChannelHandle
 *      FAPI MSGCOM Channel on which the messages are received
 *
 *  @retval
 *      Not Applicable
 */
static void Fapi_processMessage(Fapi_MCB* ptrFAPI, MsgCom_ChHandle fapiChannelHandle)
{
    Ti_Pkt*             ptrFAPIPkt;
    TiFapi_msg_s*       ptrFAPIMessage;
    uint32_t            fapiMessageLen;
    int32_t             errCode;

    /* Cycle through and process all the received FAPI messages */
    while (1)
    {
        /* Process all packets on the handover channel. */
        Msgcom_getMessage (fapiChannelHandle, (MsgCom_Buffer**)&ptrFAPIPkt);

        /* Did we process all the messages */
        if (ptrFAPIPkt == NULL)
            break;

        /* Get the actual FAPI Message. */
        Pktlib_getDataBuffer(ptrFAPIPkt, (uint8_t**)&ptrFAPIMessage, &fapiMessageLen);

        /* The actual length of the FAPI message is stored in the packet length */
        fapiMessageLen = Pktlib_getPacketLen(ptrFAPIPkt);

        /* Log Message: */
        Log_write3(UIAEvt_detailWithStr, 0x11, (IArg)"Stack receiving FAPI Message with id %x", ptrFAPIMessage->msgId);

        /* Populate the message on the basis of the message type */
        if (ptrFAPIMessage->msgId == TI_FAPI_PARAM_RESPONSE)
        {
            /* Parameter Response has been received. */
            ptrFAPI->state = Fapi_State_PARAM_EXCHANGED;

            /* Send the FAPI Packet to the Tracer. */
            if(FapiTracing_trace(ptrFAPI->fapiTracingHandle, ptrFAPIPkt, &errCode) < 0)
                ptrFAPI->stats.DroppedPackets++;
            ptrFAPI->stats.TotalPackets++;

            /* Send the Config message */
            Fapi_sendMessage(ptrFAPI, TI_FAPI_CELL_CONFIG_REQUEST, NULL, 0);
        }
        else if (ptrFAPIMessage->msgId == TI_FAPI_CELL_CONFIG_RESPONSE)
        {
            /* Cell Configuration Response has been received. */
            ptrFAPI->state = Fapi_State_CELL_CONFIG_RESPONSE;

            /* Send the FAPI Packet to the Tracer. */
            if(FapiTracing_trace(ptrFAPI->fapiTracingHandle, ptrFAPIPkt, &errCode) < 0)
                ptrFAPI->stats.DroppedPackets++;
            ptrFAPI->stats.TotalPackets++;

            /* Send the Start Request message */
            Fapi_sendMessage(ptrFAPI, TI_FAPI_START_REQUEST, NULL, 0);
        }
        else if (ptrFAPIMessage->msgId == TI_FAPI_UL_SUBFRAME_INDICATION)
        {
            TiFapi_sfInd_s* ptrSubframeIndication;

            /* Subframe Indication has been received. */
            ptrFAPI->state = Fapi_State_PHY_STARTED;

            /* Get the pointer to the subframe indication */
            ptrSubframeIndication = (TiFapi_sfInd_s*)&ptrFAPIMessage->msgBody[0];

            /* Get the subframe number and store this into the FAPI MCB. */
            ptrFAPI->sfnSf = ptrSubframeIndication->sfnSf;

            /* Get the subframe number */
            ptrFAPI->sf  = (ptrFAPI->sfnSf & 0x0F);
            ptrFAPI->sfn = (ptrFAPI->sfnSf & 0xFFF0) >> 4;

            /* Send the FAPI Packet to the Tracer. */
            if(FapiTracing_trace(ptrFAPI->fapiTracingHandle, ptrFAPIPkt, &errCode) < 0)
                ptrFAPI->stats.DroppedPackets++;
            ptrFAPI->stats.TotalPackets++;

            /* Send the DL Config Request */
            Fapi_sendMessage(ptrFAPI, TI_FAPI_DL_CONFIG_REQUEST, NULL, 0);

            /* Send the UL Config Request */
            Fapi_sendMessage(ptrFAPI, TI_FAPI_UL_CONFIG_REQUEST, NULL, 0);

            /* Send the Master Information block message. */
            if ((ptrFAPI->sf == 0) && ((ptrFAPI->sfn % 4) == 0))
            {
                /* Update the subframe indication inside the FAPI MIB message. */
                ptrFAPI->ptrMIBRequest[0] = 0xa0 | ((ptrFAPI->sfn & 0xF00) >> 8);
                ptrFAPI->ptrMIBRequest[1] = (ptrFAPI->sfn & 0xFF);

                /* Update the subframe indication inside the FAPI MIB message. */
                Fapi_sendMessage(ptrFAPI, TI_FAPI_DL_TX_REQUEST, (uint8_t*)ptrFAPI->ptrMIBRequest, sizeof(Fapi_mib));
            }

            /* Send the System Information block1 message. */
            if ((ptrFAPI->sf == 5) && ((ptrFAPI->sfn % 2) == 0))
                Fapi_sendMessage(ptrFAPI, TI_FAPI_DL_TX_REQUEST, (uint8_t*)ptrFAPI->ptrSIB1Request, sizeof(Fapi_sib1));

            /* Send the System Information block2 message. */
            if ((ptrFAPI->sf == 6) && ((ptrFAPI->sfn % 16) == 0))
                Fapi_sendMessage(ptrFAPI, TI_FAPI_DL_TX_REQUEST, (uint8_t*)ptrFAPI->ptrSIB2Request, sizeof(Fapi_sib2));
        }
        else
        {
            /* Send the FAPI Packet to the Tracer. */
            if(FapiTracing_trace(ptrFAPI->fapiTracingHandle, ptrFAPIPkt, &errCode) < 0)
                ptrFAPI->stats.DroppedPackets++;
            ptrFAPI->stats.TotalPackets++;
        }

        /* Cleanup the FAPI Packet */
        Pktlib_freePacket(ptrFAPI->pktlibInstHandle, ptrFAPIPkt);
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      This is the FAPI Receive Function called from ISR
 *
 *  @retval
 *      Not Applicable
 */
static void* Fapi_receive(Fapi_MCB* ptrFAPI)
{
    /* Process all the messages on the high priority channel. */
    Fapi_processMessage(ptrFAPI, ptrFAPI->fapiL1L2HighPriorityChannel);
    
    /* Process all the messages on the low priority channel - this channel is polled only. */
    Fapi_processMessage(ptrFAPI, ptrFAPI->fapiL1L2LowPriorityChannel);
    
    /* Process all the messages on MAC PDU channel - this channel is polled only. */
    Fapi_processMessage(ptrFAPI, ptrFAPI->fapiL1L2MacPduChannel);

    return NULL;
}

/**
 *  @b Description
 *  @n
 *      Utility function which computes the maximum of numbers.
 *
 *  @param[in]  a
 *      First Number
 *  @param[in]  b
 *      Second Number
 *
 *  @retval
 *      Largest number
 */
static inline uint32_t max (uint32_t a, uint32_t b)
{
    return (a > b) ? a : b;
}

/**
 *  @b Description
 *  @n
 *      This is the Application managed UINTC thread to handle FAPI channel interrupts
 *
 *  @param[in]  arg
 *      Argument passed to the thread.
 *
 *  @retval
 *      Not Applicable.
 */
static void* Fapi_uintcRcvTask(void *arg)
{
    AppLTEStackDomainMCB*   ptrLTEStackDomain;
    int32_t                 numFd;
    uint32_t                value;
    fd_set                  fds;
    int32_t                 maxFd;
    AppLTEStackUintcInfo*   ptrUintcInfo;
    Fapi_MCB*               ptrFAPI;
    struct timeval          timeout;

    /* Get the pointer to the FAPI MCB */
    ptrFAPI = (Fapi_MCB*)arg;

    /* Get the interrupt handle. */
    ptrLTEStackDomain = (AppLTEStackDomainMCB*)ptrFAPI->ptrLTEStackDomain;

    /* Sanity check: Uintc should be initialized already */
    if(ptrLTEStackDomain->fapiUintcHandle == NULL)
    {
        printf("Error: UINTC FAPI handle is not initialized\n");
        return NULL;
    }

    /* Polling Low priority Queue before reaching PHY started state */
    while (ptrFAPI->state != Fapi_State_CELL_CONFIG_RESPONSE)
    {
        /* Configure the timeout: This is used to handle the 
        Low Priority Queue during PHY startup. */
        timeout.tv_sec  = 0;
        timeout.tv_usec = 10000;

        /* Wait for the timeout to poll low priority channel. */
        if(select(0, NULL, NULL, NULL, &timeout) < 0)
        {
            /* All other errors are FATAL. */
            printf ("Error: Fapi_uintcRcvTask(CellID:%d): UINTC select failed [%s]\n", ptrFAPI->cellId, strerror(errno));
            return NULL;
        }

        /* Timeout, Process all the messages on the low priority channel. */
        Fapi_processMessage(ptrFAPI, ptrFAPI->fapiL1L2LowPriorityChannel);
    }

    /* Based on Cell id, get the associated Uintc Info */
    if(ptrFAPI->cellId == Fapi_Cell_A)
        ptrUintcInfo = &ptrLTEStackDomain->uintcInfoCellAHPChannel;
    else
        ptrUintcInfo = &ptrLTEStackDomain->uintcInfoCellBHPChannel;

    /* Interrupt will be trigged on High priority channel only */
    maxFd = ptrUintcInfo->uintcFd;

    /* Loop around: The UINTC thread is waiting for an interrupt to arrive on events 
     * which have been registered with the UINTC instance. 
     * This example demonstrates the followings:
     *   - Interrupt is only triggered on High Priority channel
     *   - Low Priority channel and MAC PDU channel are polled after High Prioriy channel
     */
    while (ptrFAPI->fapiStatus == 1)
    {
        /* Clear the file descriptor set */
        FD_ZERO(&fds);
    
        /* Set the event block file descriptor in the set. */
        FD_SET(ptrUintcInfo->uintcFd, &fds);

        /* Wait for the interrupts to get fired. */
        numFd = select(maxFd + 1, &fds, NULL, NULL, NULL);
        if (numFd == -1)
        {
            /* Error: Select failed. Determine the error on why this happened? */
            if (errno == EBADF)
            {
                /* Bad file descriptor actually implies that one of the descriptors
                 * in the file set has been closed or is no longer valid. This can happen
                 * because once an ISR is deregistered; the file descriptor is closed in
                 * that context. Ignore this error, reconstruct the file descriptor list
                 * and try again too see if there are other events. */
                continue;
            }

            /* All other errors are FATAL. */
            printf ("Error: UINTC select failed [%s]\n", strerror(errno));
            return NULL;
        }

        /* Is there a timeout? */
        if (numFd == 0)
            return 0;

        /* Is the event pending on HP FAPI channel? Dispatch the application registered ISR handler. */
        if (FD_ISSET(ptrUintcInfo->uintcFd, &fds))
        {
            /* YES. Read from the device: This is required to clean up the pending events
             * from the UIO driver. This is required else the file descriptor will always
             * be ready with data. */
            if (read(ptrUintcInfo->uintcFd, &value, sizeof(uint32_t)) != sizeof(uint32_t))
            {
                /* FATAL Error: UIO read should never fail. */
                printf("Internal Error: HP channel UINTC UIO read failed [%s]\n", strerror(errno));
                return NULL;
            }

            /* Since we are executing the UINTC in application managed mode; we are responsible for
             * invoking the MSGCOM channel receive handler. */
            Msgcom_channelRxHandler (ptrUintcInfo->msgcomChannel);

            /* Finally, Calling Fapi receive function to handle FAPI messages. */
            Fapi_receive(ptrFAPI);
        }
    }
    printf ("Debug: Shutting down the FAPI receive task\n");

    return NULL;
}

/**
 *  @b Description
 *  @n
 *      The function sets up required IPv4 NetFP Fast Path
 *      handles for FAPI to be operational.
 *
 *  @param[in]  ptrLTEStackDomain
 *      Pointer to the LTE Stack domain MCB
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
int32_t Fapi_setupIPv4Env(AppLTEStackDomainMCB* ptrLTEStackDomain)
{
    int32_t                 errCode;
    uint32_t                ingressSPID, egressSPID;
    Name_ResourceCfg        nrConfig;
    Netfp_InboundFPCfg      inboundFPCfg;
    Netfp_OutboundFPCfg     outboundFPCfg;
    Netfp_ClientHandle      netfpClientHandle;

    /* Get the NETFP Client Handle: */
    netfpClientHandle = Domain_getNetfpClientInstanceHandle(ptrLTEStackDomain->syslibHandle);

    /* Find the ingress policy to use for FAPI */
    while (1)
    {
        if (Name_findResource (Domain_getDatabaseHandle(ptrLTEStackDomain->syslibHandle),
                               Name_ResourceBucket_USER_DEF1,
                               "Ingress_SPID_IPv4_UP",
                               &nrConfig,
                               &errCode) == 0)
        {
            /* Found the Ingress IPv4 Policy to use for FAPI.
             * In this case, FAPI is using the same policy as the UserPlane */
            ingressSPID =    (uint32_t)nrConfig.handle1;
            break;
        }
        else
        {
            /* Error finding the policy. Check the error */
            if (errCode == NAME_ENOTFOUND)
            {
                /* Policy not found. Wait for sometime and retry */
                usleep (1);
                continue;
            }
            else
            {
                /* Error getting the policy to use for UP. Cant proceed */
                return -1;
            }
        }
    }

    /* Find the egress policy to use for FAPI */
    while (1)
    {
        if (Name_findResource (Domain_getDatabaseHandle(ptrLTEStackDomain->syslibHandle),
                               Name_ResourceBucket_USER_DEF1,
                               "Egress_SPID_IPv4_UP",
                               &nrConfig,
                               &errCode) == 0)
        {
            /* Found the Egress IPv4 Policy to use for FAPI.
             * In this case, FAPI is using the same policy as the UserPlane */
            egressSPID =    (uint32_t)nrConfig.handle1;
            break;
        }
        else
        {
            /* Error finding the policy. Check the error */
            if (errCode == NAME_ENOTFOUND)
            {
                /* Policy not found. Wait for sometime and retry */
                usleep (1);
                continue;
            }
            else
            {
                /* Error getting the policy to use for UP. Cant proceed */
                return -1;
            }
        }
    }

    /* Initialize the fast path configuration. */
    memset ((void *)&inboundFPCfg, 0, sizeof (Netfp_InboundFPCfg));

    /* Initialize spidMode */
    if(ingressSPID == NETFP_INVALID_SPID)
        inboundFPCfg.spidMode = Netfp_SPIDMode_INVALID;
    else
        inboundFPCfg.spidMode = Netfp_SPIDMode_SPECIFIC;

    /* Populate the Fast Path configuration. */
    strcpy (inboundFPCfg.name, "Ingress_FastPath_Debug");
    inboundFPCfg.fastpathMode             = Netfp_FastpathMode_INFOONLY;
    inboundFPCfg.spId                     = ingressSPID;
    if(ingressSPID == NETFP_INVALID_SPID)
        inboundFPCfg.spidMode = Netfp_SPIDMode_INVALID;
    else
        inboundFPCfg.spidMode = Netfp_SPIDMode_SPECIFIC;
    inboundFPCfg.srcIP.ver               = Netfp_IPVersion_IPV4;
    inboundFPCfg.srcIP.addr.ipv4.u.a8[0] = loggerIPAddress[0];
    inboundFPCfg.srcIP.addr.ipv4.u.a8[1] = loggerIPAddress[1];
    inboundFPCfg.srcIP.addr.ipv4.u.a8[2] = loggerIPAddress[2];
    inboundFPCfg.srcIP.addr.ipv4.u.a8[3] = loggerIPAddress[3];
    inboundFPCfg.dstIP.ver               = Netfp_IPVersion_IPV4;
    inboundFPCfg.dstIP.addr.ipv4.u.a8[0] = eNodeBIPAddress[0];
    inboundFPCfg.dstIP.addr.ipv4.u.a8[1] = eNodeBIPAddress[1];
    inboundFPCfg.dstIP.addr.ipv4.u.a8[2] = eNodeBIPAddress[2];
    inboundFPCfg.dstIP.addr.ipv4.u.a8[3] = eNodeBIPAddress[3];

    /* Add the Ingress Fast Path. */
    ptrLTEStackDomain->fapiTracingIngressFastPath = Netfp_createInboundFastPath (netfpClientHandle, &inboundFPCfg, &errCode);
    if (ptrLTEStackDomain->fapiTracingIngressFastPath == NULL)
    {
        printf ("Error: Unable to create FAPI Tracing Ingress fast path [Error code %d]\n", errCode);
        return -1;
    }
    printf ("Debug: FAPI Tracing Ingress Fast Path Handle %p\n", ptrLTEStackDomain->fapiTracingIngressFastPath);

    /* Initialize the fast path configuration. */
    memset ((void *)&outboundFPCfg, 0, sizeof (Netfp_OutboundFPCfg));

    /* Populate the Fast Path configuration. */
    strcpy (outboundFPCfg.name, "Egress_FastPath_Debug");
    outboundFPCfg.spId                      = egressSPID;
    outboundFPCfg.dstIP.ver               = Netfp_IPVersion_IPV4;
    outboundFPCfg.srcIP.ver               = Netfp_IPVersion_IPV4;
    outboundFPCfg.srcIP.addr.ipv4.u.a8[0] = eNodeBIPAddress[0];
    outboundFPCfg.srcIP.addr.ipv4.u.a8[1] = eNodeBIPAddress[1];
    outboundFPCfg.srcIP.addr.ipv4.u.a8[2] = eNodeBIPAddress[2];
    outboundFPCfg.srcIP.addr.ipv4.u.a8[3] = eNodeBIPAddress[3];
    outboundFPCfg.dstIP.addr.ipv4.u.a8[0] = loggerIPAddress[0];
    outboundFPCfg.dstIP.addr.ipv4.u.a8[1] = loggerIPAddress[1];
    outboundFPCfg.dstIP.addr.ipv4.u.a8[2] = loggerIPAddress[2];
    outboundFPCfg.dstIP.addr.ipv4.u.a8[3] = loggerIPAddress[3];

    /* Add the Egress Fast Path. */
    ptrLTEStackDomain->fapiTracingEgressFastPath = Netfp_createOutboundFastPath (netfpClientHandle, &outboundFPCfg, &errCode);
    if (ptrLTEStackDomain->fapiTracingEgressFastPath == NULL)
    {
        printf ("Error: Unable to create FAPI Tracing Egress fast path [Error code %d]\n", errCode);
        return -1;
    }
   printf ("Debug: FAPI Tracing Egress Fast Path Handle %p\n", ptrLTEStackDomain->fapiTracingEgressFastPath);

   /* Loop around waiting for the fast path to become active. */
   while (1)
   {
        int32_t status;

        /* Get the status of the fast path */
        if (Netfp_isOutboundFastPathActive (netfpClientHandle, ptrLTEStackDomain->fapiTracingEgressFastPath, &status, &errCode) < 0)
        {
            printf ("Error: Unable to get the outbound fast path activity status [Error code %d]\n", errCode);
            return -1;
        }

        /* Is the fast path active? */
        if(status == 1)
            break;

        /* Relinquish time slice and try again */
        usleep(100);
   }
   /* Return success */
   return 0;
}

/**
 *  @b Description
 *  @n
 *      The function sets up required IPv6 NetFP Fast Path
 *      handles for FAPI to be operational.
 *
 *  @param[in]  ptrLTEStackDomain
 *      Pointer to the LTE Stack domain MCB
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
int32_t Fapi_setupIPv6Env(AppLTEStackDomainMCB* ptrLTEStackDomain)
{
    int32_t                 errCode;
    uint32_t                ingressSPID, egressSPID;
    Name_ResourceCfg        nrConfig;
    Netfp_InboundFPCfg      inboundFPCfg;
    Netfp_OutboundFPCfg     outboundFPCfg;
    Netfp_ClientHandle      netfpClientHandle;

    /* Get the NETFP Client Handle: */
    netfpClientHandle = Domain_getNetfpClientInstanceHandle(ptrLTEStackDomain->syslibHandle);

    /* Find the ingress policy to use for FAPI */
    while (1)
    {
        if (Name_findResource (Domain_getDatabaseHandle(ptrLTEStackDomain->syslibHandle),
                               Name_ResourceBucket_USER_DEF1,
                               "Ingress_SPID_IPv6_UP",
                               &nrConfig,
                               &errCode) == 0)
        {
            /* Found the Ingress IPv4 Policy to use for FAPI.
             * In this case, FAPI is using the same policy as the UserPlane */
            ingressSPID =    (uint32_t)nrConfig.handle1;
            break;
        }
        else
        {
            /* Error finding the policy. Check the error */
            if (errCode == NAME_ENOTFOUND)
            {
                /* Policy not found. Wait for sometime and retry */
                usleep (1);
                continue;
            }
            else
            {
                /* Error getting the policy to use for UP. Cant proceed */
                return -1;
            }
        }
    }

    /* Find the egress policy to use for FAPI */
    while (1)
    {
        if (Name_findResource (Domain_getDatabaseHandle(ptrLTEStackDomain->syslibHandle),
                               Name_ResourceBucket_USER_DEF1,
                               "Egress_SPID_IPv6_UP",
                               &nrConfig,
                               &errCode) == 0)
        {
            /* Found the Egress IPv4 Policy to use for FAPI.
             * In this case, FAPI is using the same policy as the UserPlane */
            egressSPID =    (uint32_t)nrConfig.handle1;
            break;
        }
        else
        {
            /* Error finding the policy. Check the error */
            if (errCode == NAME_ENOTFOUND)
            {
                /* Policy not found. Wait for sometime and retry */
                usleep (1);
                continue;
            }
            else
            {
                /* Error getting the policy to use for UP. Cant proceed */
                return -1;
            }
        }
    }

    /* Initialize the fast path configuration. */
    memset ((void *)&inboundFPCfg, 0, sizeof (Netfp_InboundFPCfg));

    /* Initialize spidMode */
    inboundFPCfg.spidMode = Netfp_SPIDMode_INVALID;

    /* Populate the Fast Path configuration. */
    strcpy (inboundFPCfg.name, "Ingress_IPv6_FastPath_Debug");
    inboundFPCfg.fastpathMode             = Netfp_FastpathMode_INFOONLY;
    inboundFPCfg.spId                     = ingressSPID;
    if(ingressSPID == NETFP_INVALID_SPID)
        inboundFPCfg.spidMode = Netfp_SPIDMode_INVALID;
    else
        inboundFPCfg.spidMode = Netfp_SPIDMode_SPECIFIC;
    inboundFPCfg.srcIP.ver               = Netfp_IPVersion_IPV6;
    Netfp_convertStrToIP6(loggerIPv6Address, &inboundFPCfg.srcIP.addr.ipv6);
    inboundFPCfg.dstIP.ver               = Netfp_IPVersion_IPV6;
    Netfp_convertStrToIP6(eNodeBIPv6Address, &inboundFPCfg.dstIP.addr.ipv6);

    /* Add the Ingress Fast Path. */
    ptrLTEStackDomain->fapiTracingIPv6IngressFastPath = Netfp_createInboundFastPath (netfpClientHandle, &inboundFPCfg, &errCode);
    if (ptrLTEStackDomain->fapiTracingIPv6IngressFastPath == NULL)
    {
        printf ("Error: Unable to create FAPI Tracing Ingress fast path [Error code %d]\n", errCode);
        return -1;
    }
    printf ("Debug: FAPI Tracing Ingress Fast Path Handle %p\n", ptrLTEStackDomain->fapiTracingIPv6IngressFastPath);

    /* Initialize the fast path configuration. */
    memset ((void *)&outboundFPCfg, 0, sizeof (Netfp_OutboundFPCfg));

    /* Populate the Fast Path configuration. */
    strcpy (outboundFPCfg.name, "Egress_IPv6_FastPath_Debug");
    outboundFPCfg.spId                    = egressSPID;
    outboundFPCfg.dstIP.ver               = Netfp_IPVersion_IPV6;
    outboundFPCfg.srcIP.ver               = Netfp_IPVersion_IPV6;
    Netfp_convertStrToIP6(eNodeBIPv6Address, &outboundFPCfg.srcIP.addr.ipv6);
    Netfp_convertStrToIP6(loggerIPv6Address, &outboundFPCfg.dstIP.addr.ipv6);

    /* Add the Egress Fast Path. */
    ptrLTEStackDomain->fapiTracingIPv6EgressFastPath = Netfp_createOutboundFastPath (netfpClientHandle, &outboundFPCfg, &errCode);
    if (ptrLTEStackDomain->fapiTracingIPv6EgressFastPath == NULL)
    {
        printf ("Error: Unable to create FAPI Tracing Egress fast path [Error code %d]\n", errCode);
        return -1;
    }
   printf ("Debug: FAPI Tracing Egress Fast Path Handle %p\n", ptrLTEStackDomain->fapiTracingIPv6EgressFastPath);

   /* Loop around waiting for the fast path to become active. */
   while (1)
   {
        int32_t status;

        /* Get the status of the fast path */
        if (Netfp_isOutboundFastPathActive (netfpClientHandle, ptrLTEStackDomain->fapiTracingIPv6EgressFastPath, &status, &errCode) < 0)
        {
            printf ("Error: Unable to get the outbound fast path activity status [Error code %d]\n", errCode);
            return -1;
        }

        /* Is the fast path active? */
        if(status == 1)
            break;

        /* Relinquish time slice and try again */
        usleep(100);
   }
   /* Return success */
   return 0;
}

/**
 *  @b Description
 *  @n
 *      The function cleans up all the NetFP handles
 *      set up as part of IPv4 environment setup for
 *      FAPI to be operational
 *
 *  @param[in]  ptrLTEStackDomain
 *      Pointer to the LTE Stack domain MCB
 *
 *  @retval
 *      None
 */
void Fapi_deInitIPv4Env(AppLTEStackDomainMCB* ptrLTEStackDomain)
{
    int32_t                 errCode;

    /* Delete the IPv4 FAPI FastPaths */
    Netfp_deleteInboundFastPath (Domain_getNetfpClientInstanceHandle(ptrLTEStackDomain->syslibHandle),
                                 ptrLTEStackDomain->fapiTracingIngressFastPath, &errCode);
    ptrLTEStackDomain->fapiTracingIngressFastPath   =    NULL;
    Netfp_deleteOutboundFastPath (Domain_getNetfpClientInstanceHandle(ptrLTEStackDomain->syslibHandle),
                                  ptrLTEStackDomain->fapiTracingEgressFastPath, &errCode);
    ptrLTEStackDomain->fapiTracingEgressFastPath    =    NULL;

    /* Done cleaning up the IPv4 setup */
    return;
}

/**
 *  @b Description
 *  @n
 *      The function cleans up all the NetFP handles
 *      set up as part of IPv6 environment setup for
 *      FAPI to be operational
 *
 *  @param[in]  ptrLTEStackDomain
 *      Pointer to the LTE Stack domain MCB
 *
 *  @retval
 *      None
 */
void Fapi_deInitIPv6Env(AppLTEStackDomainMCB* ptrLTEStackDomain)
{
    int32_t                 errCode;

    /* Delete the IPv4 FAPI FastPaths */
    Netfp_deleteInboundFastPath (Domain_getNetfpClientInstanceHandle(ptrLTEStackDomain->syslibHandle),
                                 ptrLTEStackDomain->fapiTracingIPv6IngressFastPath, &errCode);
    ptrLTEStackDomain->fapiTracingIPv6IngressFastPath   =    NULL;
    Netfp_deleteOutboundFastPath (Domain_getNetfpClientInstanceHandle(ptrLTEStackDomain->syslibHandle),
                                  ptrLTEStackDomain->fapiTracingIPv6EgressFastPath, &errCode);
    ptrLTEStackDomain->fapiTracingIPv6EgressFastPath    =    NULL;

    /* Done cleaning up the IPv4 setup */
    return;
}

#if FAPI_MEMLOGGING
/**
 *  @b Description
 *  @n
 *     Task to handle memory logging.
 *
 *  @param[in]  arg
 *      Memlog channel name
 *
 *  @retval
 *      Not Applicable.
 */
static void* Fapi_memLoggingThread(void* arg)
{
    int32_t                 errCode;
    Memlog_CtrlHandle       ptrMemlogCtrl;
    MemLog_LoggerInfo       memlogInfo;
    char                    memlogChanName[MEMLOG_MAX_CHAR];
    uint32_t                mmap_fd;
    uint32_t                pageSize;
    void*                   mmapBaseAddress = NULL;
    char                    logFileName[64];
    FILE*                   fp = NULL;

    /* Get MEMLOG controller */
    ptrMemlogCtrl = (Memlog_CtrlHandle)arg;

    /* Get MEMLOG channel name */
    if (Memlog_getMemLogChanName(ptrMemlogCtrl, &memlogChanName[0], &errCode) < 0)
    {
        /* Get producer info failed */
        printf("Get MEMLOG channel name failed, error Code = %d \n", errCode);
        goto errExit;
    }

    /* Get MEMLOG channel info */
    if (Memlog_getMemlogChanInfo(ptrMemlogCtrl, &memlogInfo, &errCode) < 0)
    {
        /* Get producer info failed */
        printf("Get memlog info failed, error Code = %d \n", errCode);
        goto errExit;
    }

    printf("Debug: Got FAPI memlog information for %s\n", memlogChanName);
    printf("    memory address=0x%x\n", memlogInfo.memBase);
    printf("    memory size=0x%x\n", memlogInfo.memSize);
    printf("    buffer size=%d\n", memlogInfo.bufferSize);
    printf("    FAPI memlog channel realm=%d\n", memlogInfo.realm);

    /* Memory map the logging buffer */
    if (memlogInfo.realm == Memlog_ExecutionRealm_DSP)
    {
        /* Open the /dev/mem file - for non-cachable memroy */
        mmap_fd = open("/dev/mem", (O_RDWR | O_SYNC));
        if(mmap_fd == -1)
        {
            printf("Error: Unable to open device memory file\n");
            goto errExit;
        }

        /* Get the page size. */
        pageSize = sysconf(_SC_PAGE_SIZE);
        if (pageSize <= 0)
            goto errExit;

        /* Ensure block size and physical addresses are aligned to page size. */
        if ((memlogInfo.memSize % pageSize) || ((uint32_t)memlogInfo.memBase % pageSize))
        {
             printf("Error: Logging memory (base=0x%x, size=0x%x) is not page size (%d) aligned\n",
                     memlogInfo.memBase, memlogInfo.memSize, pageSize);
             goto errExit;
        }

        /* Create a mapping of the physical address. */
        mmapBaseAddress = mmap(0, memlogInfo.memSize, (PROT_READ|PROT_WRITE), MAP_SHARED, mmap_fd, (off_t)memlogInfo.memBase);
        if(mmapBaseAddress == MAP_FAILED)
        {
            printf("Error: Unable to map log memory!\n");
            goto errExit;
        }
        /* Close mmap file */
        close(mmap_fd);

        printf("Debug: Log memory(0x%x) is successfully mapped to 0x%x\n",  memlogInfo.memBase, (uint32_t)mmapBaseAddress);
    }
    else
    {
          mmapBaseAddress = (void *)memlogInfo.memBase;
    }
    printf("    memory virtual address=0x%x\n", (uint32_t)mmapBaseAddress);

    /* Open the log file */
    sprintf(logFileName, "%s%s.%s", "/tmp/", memlogChanName, "log");

    printf("log file: %s\n", logFileName);
    /* open a file, save the log in the file */
    fp = fopen(logFileName, "w");
    if(fp == NULL)
    {
        printf("ERROR: Open file %s failed.\n", logFileName);
        goto errExit;
    }

    /* Save log file periodically */
    while(!gMemlogThreadTerminate)
    {
        /* Sleep for 1s */
        usleep(1000000);

        /* Stop logging */
        Memlog_stopLogging (ptrMemlogCtrl, &errCode);

        /* Mandatory delay for 100ms to wait until producer side has done with writing memory */
        usleep(100000);

        /* Save logs in a file in memory dump format */
        if(fseek(fp, 0, SEEK_SET) < 0 )
        {
            /* Error: Unable to send out the packet. */
            printf ("Error: fseek() failed.\n");
            goto errExit;
        }

        if(fwrite((void *)mmapBaseAddress, 128, memlogInfo.memSize/128, fp) < 0 )
        {
            /* Error: Unable to send out the packet. */
            printf ("Error: fwrite() failed.\n");
            goto errExit;
        }

        /* Start logging */
        Memlog_startLogging (ptrMemlogCtrl, &errCode);
    }

    Memlog_startLogging (ptrMemlogCtrl, &errCode);
errExit:

    /* Delete producer controller */
    if( ptrMemlogCtrl )
    {
        /* Delete producer controller */
        if ( Memlog_deleteMemlogController(ptrMemlogCtrl, &errCode) < 0)
        {
            printf("Delete memlog controller for %s failed with error Code=%d\n", memlogChanName, errCode);
        }
        printf("Memlog controller has been deleted!\n");
    }

    /* Close log file */
    if(fp)
        fclose(fp);

    /* Unmap LOg memory */
    if(mmapBaseAddress && (memlogInfo.realm == Memlog_ExecutionRealm_DSP) )
        munmap(mmapBaseAddress,  memlogInfo.memSize);

    printf("Debug: Thread for memory logging for producer %s exits!\n", memlogChanName);

    return NULL;
}

/**
 *  @b Description
 *  @n  
 *      Socket Post Routing hook: This is registered to be invoked on the transmission
 *      of a tracing packet once all the networking headers have been attached and just before 
 *      the packet is passed to the NETCP. This is used to save FAPI logs in Memory.
 *
 *  @param[in]  hook
 *      Hook identifier
 *  @param[in]  sockHandle
 *      Socket Handle on which the packet is being sent
 *  @param[in]  ptrPkt
 *      Pointer to the packet
 *  @param[in]  arg
 *      Registered argument
 *
 *  @retval
 *      Hook Return
 */
static Netfp_HookReturn FapiTracing_MemoryCaptureHook
(
    Netfp_Hook          hook,
    Netfp_SockHandle    sockHandle,
    Ti_Pkt*             ptrPkt,
    uint32_t            arg
)
{
    Fapi_MCB*           ptrFAPI;
    Ti_Pkt              *pTimeStampPkt;
    uint32_t            *ptrTimeStampBuffer;	
    uint32_t            BufferLen;
    struct timeval      now;

    /* Get FAPI MCB handle */
    ptrFAPI = (Fapi_MCB*)arg;

    /* Sanity Test: Ensure that the arguments are as expected */
    if (hook != Netfp_Hook_POST_ROUTING)
        return Netfp_HookReturn_DROP;

    /* Verify the memlog channle handle */
    if( (ptrFAPI == NULL) || (sockHandle == NULL) )
        return Netfp_HookReturn_DROP;

    /* Sanity Test: We should always receive a packet */
    if (ptrPkt == NULL)
        return Netfp_HookReturn_DROP;

    /* Allocate a packet to add TimeStamp */
    pTimeStampPkt = Pktlib_allocPacket(ptrFAPI->pktlibInstHandle, ptrFAPI->fapiMemTracingHeap, 8);
    if (pTimeStampPkt == NULL)
    {
        printf("Failed to allocate time stamp packet.\n");
        return Netfp_HookReturn_DROP;
    }

    /* Get data buffer pointer and fill in time stamp data 
       This timestamp is used in the post-processing script to re-order the packet */
    Pktlib_getDataBuffer(pTimeStampPkt, (uint8_t **)&ptrTimeStampBuffer, &BufferLen);
    gettimeofday(&now, NULL);
    ptrTimeStampBuffer[0] = now.tv_sec;
    ptrTimeStampBuffer[1] = now.tv_usec;

    /* Merge time stamp packet with FAPI tracing packet */
    Pktlib_packetMerge (ptrFAPI->pktlibInstHandle, pTimeStampPkt, ptrPkt, NULL);	

    /* Send the packet to MEMLOG channel */
    Memlog_saveLog(ptrFAPI->memlogChanHandle, pTimeStampPkt);

    /* Since we use the packet for Memory Logging, hence return Netfp_HookReturn_STOLEN .
       Packet is not shipped to switch port */
    return Netfp_HookReturn_STOLEN;
}
#endif

/**
 *  @b Description
 *  @n
 *      The function initializes the FAPI Interface.
 *
 *  @param[in]  ptrLTEStackDomain
 *      Pointer to the LTE Stack domain MCB
 *  @param[in]  phyId
 *      PHY identifier for which the FAPI is being initialized.
 *  @param[in]  ptrL2ResourceCfg
 *      Pointer to the L2 resource configuration
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
void* Fapi_initComponent
(
    AppLTEStackDomainMCB*   ptrLTEStackDomain,
    char                    phyId,
    Resmgr_ResourceCfg*     ptrL2ResourceCfg,
    int32_t*                errCode
)
{
    struct sched_param    param;
    pthread_attr_t        attr;
    Msgcom_ChannelCfg     chConfig;
    Fapi_TracingCfg       fapiTracingCfg;
    Pktlib_HeapCfg        heapCfg;
    Fapi_MCB*             ptrFAPI;
    FapiTracing_InstCfg   fapiTracingInstCfg;
    uint8_t               responseSelection;
    Memlog_ChannelCfg     memlogChanConfig;

    /* Determine which is the response selection to be used.
     * - Queue Pend Responses 0, 1 and 2 are used for PHY-A
     * - Queue Pend Responses 3, 4 and 5 are used for PHY-B  */
    if (phyId == 'A')
        responseSelection = 0;
    else
        responseSelection = 3;

    /* Allocate memory for the FAPI object. */
    ptrFAPI = (Fapi_MCB*)malloc (sizeof(Fapi_MCB));
    if (ptrFAPI == NULL)
    {
        printf ("Error: Unable to allocate memory for the FAPI object\n");
        return NULL;
    }

    /* Initialize the FAPI Master Control block */
    memset((void *)ptrFAPI, 0, sizeof(Fapi_MCB));

    /* Keep a copy to the application domain configuration. */
    ptrFAPI->ptrLTEStackDomain  = ptrLTEStackDomain;
    ptrFAPI->pktlibInstHandle   = Domain_getPktlibInstanceHandle(ptrLTEStackDomain->syslibHandle);
    ptrFAPI->msgcomInstHandle   = Domain_getMsgcomInstanceHandle(ptrLTEStackDomain->syslibHandle);

    /* Setup Cell id */
    if (phyId == 'A')
        ptrFAPI->cellId   = Fapi_Cell_A;
    else
        ptrFAPI->cellId   = Fapi_Cell_B;

    /* Create the local heaps which are used to send data to the L1 */
    memset ((void *)&heapCfg, 0, sizeof(Pktlib_HeapCfg));

    /* Populate the heap configuration */
    snprintf (heapCfg.name, PKTLIB_MAX_CHAR, "DDR3_Heap_%c", phyId);
    heapCfg.pktlibInstHandle                = ptrFAPI->pktlibInstHandle;
    heapCfg.memRegion                       = ptrL2ResourceCfg->memRegionResponse[0].memRegionHandle;
    heapCfg.sharedHeap                      = 0;
    heapCfg.useStarvationQueue              = 0;
    heapCfg.dataBufferSize                  = sizeof(TiFapi_nonTlvMsg_s);
    heapCfg.numPkts                         = 32;
    heapCfg.numZeroBufferPackets            = 32;
    heapCfg.dataBufferPktThreshold          = 0;
    heapCfg.zeroBufferPktThreshold          = 0;
    heapCfg.arg                             = 0;
    heapCfg.heapInterfaceTable.dataMalloc   = Fapi_DDR3MemoryMalloc;
    heapCfg.heapInterfaceTable.dataFree     = Fapi_DDR3MemoryFree;

    /* Create the DDR3 Heap used to exchange FAPI messages with the PHY */
    ptrFAPI->fapiDDR3HeapHandle = Pktlib_createHeap(&heapCfg, errCode);
    if (ptrFAPI->fapiDDR3HeapHandle == NULL)
    {
        printf ("Error: Unable to create FAPI DDR3 heap [Error code %d]\n", *errCode);
        return NULL;
    }
    printf ("Debug: FAPI DDR3 heap created successfully [%d]\n", ptrL2ResourceCfg->memRegionResponse[0].memRegionHandle);

    /*********************************************************************************************
     * Convert the FAPI MIB and SIB messages from pure virtual memory to HPLIB Virtual memory. This
     * is required for the FAPI tracing to work.
     *********************************************************************************************/
    ptrFAPI->ptrDlConfigMIBRequest = hplib_vmMemAlloc (sizeof(Fapi_dlConfigMIBRequest), 0, 0);
    if (ptrFAPI->ptrDlConfigMIBRequest == NULL)
    {
        printf ("Error: Unable to allocate HPLIB Virtual memory for the DL config MIB\n");
        return NULL;
    }
    ptrFAPI->ptrDlConfigSIB1Request = hplib_vmMemAlloc (sizeof(Fapi_dlConfigSIB1Request), 0, 0);
    if (ptrFAPI->ptrDlConfigSIB1Request == NULL)
    {
        printf ("Error: Unable to allocate HPLIB Virtual memory for the DL config SIB1\n");
        return NULL;
    }
    ptrFAPI->ptrDlConfigSIB2Request = hplib_vmMemAlloc (sizeof(Fapi_dlConfigSIB2Request), 0, 0);
    if (ptrFAPI->ptrDlConfigSIB2Request == NULL)
    {
        printf ("Error: Unable to allocate HPLIB Virtual memory for the DL config SIB2\n");
        return NULL;
    }

    ptrFAPI->ptrMIBRequest = hplib_vmMemAlloc (sizeof(Fapi_mib), 0, 0);
    if (ptrFAPI->ptrMIBRequest == NULL)
    {
        printf ("Error: Unable to allocate HPLIB Virtual memory for the MIB\n");
        return NULL;
    }
    ptrFAPI->ptrSIB1Request = hplib_vmMemAlloc (sizeof(Fapi_sib1), 0, 0);
    if (ptrFAPI->ptrSIB1Request == NULL)
    {
        printf ("Error: Unable to allocate HPLIB Virtual memory for the SIB1\n");
        return NULL;
    }
    ptrFAPI->ptrSIB2Request = hplib_vmMemAlloc (sizeof(Fapi_sib2), 0, 0);
    if (ptrFAPI->ptrSIB2Request == NULL)
    {
        printf ("Error: Unable to allocate HPLIB Virtual memory for the SIB2\n");
        return NULL;
    }

    /* Debug Message: */
    printf ("DL Config MIB  Request Physical Address: %p MIB : %p\n", hplib_mVMVirtToPhy(ptrFAPI->ptrDlConfigMIBRequest), hplib_mVMVirtToPhy(ptrFAPI->ptrMIBRequest));
    printf ("DL Config SIB1 Request Physical Address: %p SIB1: %p\n", hplib_mVMVirtToPhy(ptrFAPI->ptrDlConfigSIB1Request), hplib_mVMVirtToPhy(ptrFAPI->ptrSIB1Request));
    printf ("DL Config SIB2 Request Physical Address: %p SIB2: %p\n", hplib_mVMVirtToPhy(ptrFAPI->ptrDlConfigSIB2Request), hplib_mVMVirtToPhy(ptrFAPI->ptrSIB2Request));

    /* Copy over the DL config MIB, SIB1 and SIB2 into the allocated memory. */
    memcpy ((void*)ptrFAPI->ptrDlConfigMIBRequest,  (void *)&Fapi_dlConfigMIBRequest, sizeof(Fapi_dlConfigMIBRequest));
    memcpy ((void*)ptrFAPI->ptrDlConfigSIB1Request, (void *)&Fapi_dlConfigSIB1Request, sizeof(Fapi_dlConfigSIB1Request));
    memcpy ((void*)ptrFAPI->ptrDlConfigSIB2Request, (void *)&Fapi_dlConfigSIB2Request, sizeof(Fapi_dlConfigSIB2Request));

    /* Copy over the DL MIB, SIB1 and SIB2 into the allocated memory. */
    memcpy ((void*)ptrFAPI->ptrMIBRequest,  (void *)&Fapi_mib, sizeof(Fapi_mib));
    memcpy ((void*)ptrFAPI->ptrSIB1Request, (void *)&Fapi_sib1, sizeof(Fapi_sib1));
    memcpy ((void*)ptrFAPI->ptrSIB2Request, (void *)&Fapi_sib2, sizeof(Fapi_sib2));

    /*************************************************************************
     * FAPI High Priority Channel
     *   This is the HIGH Priority Channel which is created by the L2 (Reader)
     *   and is used to receive messages from the L1(Writer)
     *   It is *recommended* that applications verify the channel
     *   configuration and modify it while designing the FAPI Interface.
     *************************************************************************/

    /* Initialize the channel configuration */
    memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* Populate the channel configuration. */
    chConfig.mode                                                         = Msgcom_ChannelMode_NON_BLOCKING;
    chConfig.arg                                                          = 0;
    chConfig.appCallBack                                                  = NULL;
    chConfig.msgcomInstHandle                                             = ptrFAPI->msgcomInstHandle;
    chConfig.u.queueDMACfg.interruptMode                                  = Msgcom_QueueInterruptMode_DIRECT_INTERRUPT;
    chConfig.u.queueDMACfg.rxFreeQueueNum                                 = (Qmss_getQIDFromHandle(Pktlib_getInternalHeapQueue(ptrFAPI->fapiDDR3HeapHandle)));
    chConfig.u.queueDMACfg.queueDMAIntrUnion.queuePendCfg.queuePendQueue  = ptrL2ResourceCfg->qPendResponse[responseSelection].queue;
    chConfig.u.queueDMACfg.queueDMAIntrUnion.queuePendCfg.cpIntcId        = ptrL2ResourceCfg->qPendResponse[responseSelection].cpIntcId;
    chConfig.u.queueDMACfg.queueDMAIntrUnion.queuePendCfg.systemInterrupt = ptrL2ResourceCfg->qPendResponse[responseSelection].systemInterrupt;
    chConfig.u.queueDMACfg.queueDMAIntrUnion.queuePendCfg.hostInterrupt   = ptrL2ResourceCfg->qPendResponse[responseSelection].hostInterrupt;

    /* Find the FAPI Channel: Channels are found depending upon the PHY to which the L2 is communicating */
    if (phyId == 'A')
    {
        ptrFAPI->fapiL1L2HighPriorityChannel = Msgcom_create (TI_FAPI_CELLA_L1L2_HP_CHANNEL_NAME,
                                                               Msgcom_ChannelType_QUEUE_DMA, &chConfig, errCode);
        if (ptrFAPI->fapiL1L2HighPriorityChannel == 0)
        {
            printf ("Error: Unable to create the High Priority FAPI channel [Error code : %d]\n", *errCode);
            return NULL;
        }
        printf ("Debug: FAPI High Priority FAPI channel '%s' created\n", TI_FAPI_CELLA_L1L2_HP_CHANNEL_NAME);
    }
    else
    {
        ptrFAPI->fapiL1L2HighPriorityChannel = Msgcom_create (TI_FAPI_CELLB_L1L2_HP_CHANNEL_NAME,
                                                               Msgcom_ChannelType_QUEUE_DMA, &chConfig, errCode);
        if (ptrFAPI->fapiL1L2HighPriorityChannel == 0)
        {
            printf ("Error: Unable to create the High Priority FAPI channel [Error code : %d]\n", *errCode);
            return NULL;
        }
        printf ("Debug: FAPI High Priority FAPI channel '%s' created\n", TI_FAPI_CELLB_L1L2_HP_CHANNEL_NAME);
    }

    /*************************************************************************
     * FAPI Low Priority Channel
     *   This is the Low Priority Channel which is created by the L2 (Reader)
     *   and is used to  receive messages from the L1(Writer)
     *   It is *recommended* that applications verify the channel
     *   configuration and modify it while designing the FAPI Interface.
     *************************************************************************/

    /* Populate the channel configuration. */
    chConfig.mode                                                         = Msgcom_ChannelMode_NON_BLOCKING;
    chConfig.arg                                                          = 0;
    chConfig.appCallBack                                                  = NULL;
    chConfig.msgcomInstHandle                                             = ptrFAPI->msgcomInstHandle;
    chConfig.u.queueDMACfg.interruptMode                                  = Msgcom_QueueInterruptMode_NO_INTERRUPT;
    chConfig.u.queueDMACfg.rxFreeQueueNum                                 = (Qmss_getQIDFromHandle(Pktlib_getInternalHeapQueue(ptrFAPI->fapiDDR3HeapHandle)));
    chConfig.u.queueDMACfg.queueDMAIntrUnion.queuePendCfg.queuePendQueue  = ptrL2ResourceCfg->qPendResponse[responseSelection+1].queue;
    chConfig.u.queueDMACfg.queueDMAIntrUnion.queuePendCfg.cpIntcId        = ptrL2ResourceCfg->qPendResponse[responseSelection+1].cpIntcId;
    chConfig.u.queueDMACfg.queueDMAIntrUnion.queuePendCfg.systemInterrupt = ptrL2ResourceCfg->qPendResponse[responseSelection+1].systemInterrupt;
    chConfig.u.queueDMACfg.queueDMAIntrUnion.queuePendCfg.hostInterrupt   = ptrL2ResourceCfg->qPendResponse[responseSelection+1].hostInterrupt;

    /* Create the FAPI Channel: Channels are created depending upon the PHY to which the L2 is communicating */
    if (phyId == 'A')
    {
        ptrFAPI->fapiL1L2LowPriorityChannel = Msgcom_create (TI_FAPI_CELLA_L1L2_LP_CHANNEL_NAME,
                                                             Msgcom_ChannelType_QUEUE_DMA, &chConfig, errCode);
        if (ptrFAPI->fapiL1L2LowPriorityChannel == 0)
        {
            printf ("Error: Unable to create the Low Priority FAPI channel Error : %d\n", *errCode);
            return NULL;
        }
        printf ("Debug: FAPI Low Priority FAPI channel '%s' created\n", TI_FAPI_CELLA_L1L2_LP_CHANNEL_NAME);
    }
    else
    {
        ptrFAPI->fapiL1L2LowPriorityChannel = Msgcom_create (TI_FAPI_CELLB_L1L2_LP_CHANNEL_NAME,
                                                             Msgcom_ChannelType_QUEUE_DMA, &chConfig, errCode);
        if (ptrFAPI->fapiL1L2LowPriorityChannel == 0)
        {
            printf ("Error: Unable to create the Low Priority FAPI channel Error : %d\n", *errCode);
            return NULL;
        }
        printf ("Debug: FAPI Low Priority FAPI channel '%s' created\n", TI_FAPI_CELLB_L1L2_LP_CHANNEL_NAME);
    }

    /*************************************************************************
     * FAPI MACPDU Channel
     *   This is the MAC PDU Channel which is created by the L2 (Reader)
     *   and is used to receive MAC PDUs from the L1(Writer)
     *   It is *recommended* that applications verify the channel
     *   configuration and modify it while designing the FAPI Interface.
     *************************************************************************/

    /* Initialize the channel configuration */
    memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* Populate the channel configuration. */
    chConfig.mode                                                         = Msgcom_ChannelMode_NON_BLOCKING;
    chConfig.arg                                                          = 0;
    chConfig.appCallBack                                                  = NULL;
    chConfig.msgcomInstHandle                                             = ptrFAPI->msgcomInstHandle;
    chConfig.u.queueDMACfg.interruptMode                                  = Msgcom_QueueInterruptMode_NO_INTERRUPT;
    chConfig.u.queueDMACfg.rxFreeQueueNum                                 = (Qmss_getQIDFromHandle(Pktlib_getInternalHeapQueue(ptrFAPI->fapiDDR3HeapHandle)));
    chConfig.u.queueDMACfg.queueDMAIntrUnion.queuePendCfg.queuePendQueue  = ptrL2ResourceCfg->qPendResponse[responseSelection+2].queue;
    chConfig.u.queueDMACfg.queueDMAIntrUnion.queuePendCfg.cpIntcId        = ptrL2ResourceCfg->qPendResponse[responseSelection+2].cpIntcId;
    chConfig.u.queueDMACfg.queueDMAIntrUnion.queuePendCfg.systemInterrupt = ptrL2ResourceCfg->qPendResponse[responseSelection+2].systemInterrupt;
    chConfig.u.queueDMACfg.queueDMAIntrUnion.queuePendCfg.hostInterrupt   = ptrL2ResourceCfg->qPendResponse[responseSelection+2].hostInterrupt;

    /* Create the FAPI Channel: Channels are found depending upon the PHY to which the L2 is communicating */
    if (phyId == 'A')
    {
        ptrFAPI->fapiL1L2MacPduChannel = Msgcom_create (TI_FAPI_CELLA_L1L2_MAC_PDU_CHANNEL_NAME,
                                                        Msgcom_ChannelType_QUEUE_DMA, &chConfig, errCode);
        if (ptrFAPI->fapiL1L2MacPduChannel == 0)
        {
            printf ("Error: Unable to create the High Priority FAPI channel [Error code : %d]\n", *errCode);
            return NULL;
        }
        printf ("Debug: FAPI MAC PDU FAPI channel '%s' created\n", TI_FAPI_CELLA_L1L2_MAC_PDU_CHANNEL_NAME);
    }
    else
    {
        ptrFAPI->fapiL1L2MacPduChannel = Msgcom_create (TI_FAPI_CELLB_L1L2_MAC_PDU_CHANNEL_NAME,
                                                        Msgcom_ChannelType_QUEUE_DMA, &chConfig, errCode);
        if (ptrFAPI->fapiL1L2MacPduChannel == 0)
        {
            printf ("Error: Unable to create the High Priority FAPI channel [Error code : %d]\n", *errCode);
            return NULL;
        }
        printf ("Debug: FAPI MAC PDU FAPI channel '%s' created\n", TI_FAPI_CELLB_L1L2_MAC_PDU_CHANNEL_NAME);
    }

    /* Find the FAPI Channel: Channels are found depending upon the PHY to which the L2 is communicating */
    if (phyId == 'A')
    {
        /* Find the FAPI High Priority MSGCOM Channels:
         *  - These channels are created by the L1(Reader) and are used by the L2 to send
         *    messages from the L2(Writer) to the L1.
         *  - This is also a SYNC point since we cannot proceed till the PHY has created the
         *    channel. */
        while (1)
        {
            ptrFAPI->fapiL2L1HighPriorityChannel = Msgcom_find (ptrFAPI->msgcomInstHandle,
                                                                TI_FAPI_CELLA_L2L1_HP_CHANNEL_NAME, errCode);
            if (ptrFAPI->fapiL2L1HighPriorityChannel != NULL)
                break;
        }

        /* Find the FAPI Low Priority MSGCOM Channels:
         *  - These channels are created by the L1(Reader) and are used by the L2 to send
         *    messages from the L2(Writer) to the L1.
         *  - This is also a SYNC point since we cannot proceed till the PHY has created the
         *    channel. */
        while (1)
        {
            ptrFAPI->fapiL2L1LowPriorityChannel = Msgcom_find (ptrFAPI->msgcomInstHandle,
                                                               TI_FAPI_CELLA_L2L1_LP_CHANNEL_NAME, errCode);
            if (ptrFAPI->fapiL2L1LowPriorityChannel != NULL)
                break;
        }

        /* Find the FAPI MAC PDU MSGCOM Channels:
         *  - These channels are created by the L1(Reader) and are used by the L2 to send
         *    Tx.request messages from the L2(Writer) to the L1.
         *  - This is also a SYNC point since we cannot proceed till the PHY has created the
         *    channel. */
        while (1)
        {
            ptrFAPI->fapiL2L1MacPduChannel = Msgcom_find (ptrFAPI->msgcomInstHandle,
                                                          TI_FAPI_CELLA_L2L1_MAC_PDU_CHANNEL_NAME, errCode);
            if (ptrFAPI->fapiL2L1MacPduChannel != NULL)
                break;
        }

        /* Debug Message: */
        printf ("-----------------------------------------------------------------\n");
        printf ("FAPI Informational Block: \n");
        printf ("Debug: FAPI Channel %s -> Channel Handle %p\n",
                        TI_FAPI_CELLA_L1L2_HP_CHANNEL_NAME, ptrFAPI->fapiL1L2HighPriorityChannel);
        printf ("Debug: FAPI Channel %s -> Channel Handle %p\n",
                        TI_FAPI_CELLA_L1L2_LP_CHANNEL_NAME, ptrFAPI->fapiL1L2LowPriorityChannel);
        printf ("Debug: FAPI Channel %s -> Channel Handle %p\n",
                        TI_FAPI_CELLA_L2L1_HP_CHANNEL_NAME, ptrFAPI->fapiL2L1HighPriorityChannel);
        printf ("Debug: FAPI Channel %s -> Channel Handle %p\n",
                        TI_FAPI_CELLA_L2L1_LP_CHANNEL_NAME, ptrFAPI->fapiL2L1LowPriorityChannel);
        printf ("Debug: FAPI Channel %s -> Channel Handle %p\n",
                        TI_FAPI_CELLA_L1L2_MAC_PDU_CHANNEL_NAME, ptrFAPI->fapiL1L2MacPduChannel);
        printf ("Debug: FAPI Channel %s -> Channel Handle %p\n",
                        TI_FAPI_CELLA_L2L1_MAC_PDU_CHANNEL_NAME, ptrFAPI->fapiL2L1MacPduChannel);
        printf ("Debug: FAPI DDR3 Heap %s -> Heap Handle %p (Free Queue 0x%x)\n",
                        TI_FAPI_MSG_HEAP_DDR_NAME, ptrFAPI->fapiDDR3HeapHandle,
                        Pktlib_getInternalHeapQueue(ptrFAPI->fapiDDR3HeapHandle));
        printf ("-----------------------------------------------------------------\n");
    }
    else
    {
        /* Find the FAPI High Priority MSGCOM Channels:
         *  - These channels are created by the L1(Reader) and are used by the L2 to send
         *    messages from the L2(Writer) to the L1.
         *  - This is also a SYNC point since we cannot proceed till the PHY has created the
         *    channel. */
        while (1)
        {
            ptrFAPI->fapiL2L1HighPriorityChannel = Msgcom_find (ptrFAPI->msgcomInstHandle,
                                                                TI_FAPI_CELLB_L2L1_HP_CHANNEL_NAME, errCode);
            if (ptrFAPI->fapiL2L1HighPriorityChannel != NULL)
                break;
        }

        /* Find the FAPI Low Priority MSGCOM Channels:
         *  - These channels are created by the L1(Reader) and are used by the L2 to send
         *    messages from the L2(Writer) to the L1.
         *  - This is also a SYNC point since we cannot proceed till the PHY has created the
         *    channel. */
        while (1)
        {
            ptrFAPI->fapiL2L1LowPriorityChannel = Msgcom_find (ptrFAPI->msgcomInstHandle,
                                                               TI_FAPI_CELLB_L2L1_LP_CHANNEL_NAME, errCode);
            if (ptrFAPI->fapiL2L1LowPriorityChannel != NULL)
                break;
        }

        /* Find the FAPI MAC PDU MSGCOM Channels:
         *  - These channels are created by the L1(Reader) and are used by the L2 to send
         *    Tx.request messages from the L2(Writer) to the L1.
         *  - This is also a SYNC point since we cannot proceed till the PHY has created the
         *    channel. */
        while (1)
        {
            ptrFAPI->fapiL2L1MacPduChannel = Msgcom_find (ptrFAPI->msgcomInstHandle,
                                                          TI_FAPI_CELLB_L2L1_MAC_PDU_CHANNEL_NAME, errCode);
            if (ptrFAPI->fapiL2L1MacPduChannel != NULL)
                break;
        }

        /* Debug Message: */
        printf ("-----------------------------------------------------------------\n");
        printf ("FAPI Informational Block: \n");
        printf ("Debug: FAPI Channel %s -> Channel Handle %p\n",
                        TI_FAPI_CELLB_L1L2_HP_CHANNEL_NAME, ptrFAPI->fapiL1L2HighPriorityChannel);
        printf ("Debug: FAPI Channel %s -> Channel Handle %p\n",
                        TI_FAPI_CELLB_L1L2_LP_CHANNEL_NAME, ptrFAPI->fapiL1L2LowPriorityChannel);
        printf ("Debug: FAPI Channel %s -> Channel Handle %p\n",
                        TI_FAPI_CELLB_L2L1_HP_CHANNEL_NAME, ptrFAPI->fapiL2L1HighPriorityChannel);
        printf ("Debug: FAPI Channel %s -> Channel Handle %p\n",
                        TI_FAPI_CELLB_L2L1_LP_CHANNEL_NAME, ptrFAPI->fapiL2L1LowPriorityChannel);
        printf ("Debug: FAPI Channel %s -> Channel Handle %p\n",
                        TI_FAPI_CELLB_L1L2_MAC_PDU_CHANNEL_NAME, ptrFAPI->fapiL1L2MacPduChannel);
        printf ("Debug: FAPI Channel %s -> Channel Handle %p\n",
                        TI_FAPI_CELLB_L2L1_MAC_PDU_CHANNEL_NAME, ptrFAPI->fapiL2L1MacPduChannel);
        printf ("Debug: FAPI DDR3 Heap %s -> Heap Handle %p (Free Queue 0x%x)\n",
                        TI_FAPI_MSG_HEAP_DDR_NAME, ptrFAPI->fapiDDR3HeapHandle,
                        Pktlib_getInternalHeapQueue(ptrFAPI->fapiDDR3HeapHandle));
        printf ("-----------------------------------------------------------------\n");
    }

    /* Display the memory: */
    debugMemMonitor (16);

    /* FAPI Tracing services are available only if the NETFP Client has been configured */
    if (Domain_getNetfpClientInstanceHandle(ptrLTEStackDomain->syslibHandle) != NULL)
    {
        /* Initialize the FAPI Tracing configuration */
        memset ((void *)&fapiTracingCfg, 0, sizeof(Fapi_TracingCfg));

        /* Initialize the heap configuration. */
        memset ((void *)&heapCfg, 0, sizeof(Pktlib_HeapCfg));

        /* Populate the heap configuration */
        snprintf (heapCfg.name, PKTLIB_MAX_CHAR, "FAPI_Tracing_%c", phyId);
        heapCfg.pktlibInstHandle                = ptrFAPI->pktlibInstHandle;
        heapCfg.memRegion                       = ptrL2ResourceCfg->memRegionResponse[0].memRegionHandle;
        heapCfg.sharedHeap                      = 0;
        heapCfg.useStarvationQueue              = 0;
        heapCfg.dataBufferSize                  = sizeof(TiFapi_nonTlvMsg_s);
        heapCfg.numPkts                         = 32;
        heapCfg.numZeroBufferPackets            = 32;
        heapCfg.dataBufferPktThreshold          = 0;
        heapCfg.zeroBufferPktThreshold          = 0;
        heapCfg.arg                             = 0;
        heapCfg.heapInterfaceTable.dataMalloc   = Fapi_DDR3MemoryMalloc;
        heapCfg.heapInterfaceTable.dataFree     = Fapi_DDR3MemoryFree;

        /* Create the FAPI Tracing Heap. */
        ptrFAPI->fapiTracingHeap = Pktlib_createHeap(&heapCfg, errCode);
        if (ptrFAPI->fapiTracingHeap == NULL)
        {
            printf ("Error: Unable to create FAPI Tracing heap [Error code %d]\n", *errCode);
            return NULL;
        }
        printf ("Debug: FAPI Tracing heap %p created successfully [%d Free Queue(s) 0x%x 0x%x]\n",
                        ptrFAPI->fapiTracingHeap, ptrL2ResourceCfg->memRegionResponse[0].memRegionHandle,
                        Pktlib_getInternalHeapQueue(ptrFAPI->fapiTracingHeap),
                        Pktlib_getZeroHeapQueue(ptrFAPI->fapiTracingHeap));

        /* Populate the heap configuration for memory logging heap 
           This heap will be used for the following two purpose:
           - To hold 8 bytes TimeStamp , used for post process
           - To clone packet when both Ethernet stream and memory logging is required
           */
        memset ((void *)&heapCfg, 0, sizeof(Pktlib_HeapCfg));

        /* Populate the heap configuration */
        snprintf (heapCfg.name, PKTLIB_MAX_CHAR, "FAPI_MemTracing_%c", phyId);
        heapCfg.pktlibInstHandle                = ptrFAPI->pktlibInstHandle;
        heapCfg.memRegion                       = ptrL2ResourceCfg->memRegionResponse[1].memRegionHandle;
        heapCfg.sharedHeap                      = 0;
        heapCfg.useStarvationQueue              = 0;
        heapCfg.dataBufferSize                  = 128;
        heapCfg.numPkts                         = Qmss_getQueueEntryCount(Pktlib_getInternalHeapQueue(ptrFAPI->fapiTracingHeap));
        heapCfg.numZeroBufferPackets            = Qmss_getQueueEntryCount(Pktlib_getInternalHeapQueue(ptrFAPI->fapiTracingHeap));
        heapCfg.dataBufferPktThreshold          = 0;
        heapCfg.zeroBufferPktThreshold          = 0;
        heapCfg.arg                             = 0;
        heapCfg.heapInterfaceTable.dataMalloc   = Fapi_DDR3MemoryMalloc;
        heapCfg.heapInterfaceTable.dataFree     = Fapi_DDR3MemoryFree;

        /* Create the FAPI Tracing Heap. */
        ptrFAPI->fapiMemTracingHeap = Pktlib_createHeap(&heapCfg, errCode);
        if (ptrFAPI->fapiMemTracingHeap == NULL)
        {
            printf ("Error: Unable to create FAPI Tracing heap [Error code %d]\n", *errCode);
            return NULL;
        }
        printf ("Debug: FAPI Memory Tracing heap %p created successfully [%d Free Queue(s) 0x%x 0x%x]\n",
                        ptrFAPI->fapiMemTracingHeap, ptrL2ResourceCfg->memRegionResponse[1].memRegionHandle,
                        Pktlib_getInternalHeapQueue(ptrFAPI->fapiMemTracingHeap),
                        Pktlib_getZeroHeapQueue(ptrFAPI->fapiMemTracingHeap));

        /* Display the memory: */
        debugMemMonitor (17);

        /* Create Memlog chan for FAPI */
        memset((void *)&memlogChanConfig, 0, sizeof(Memlog_ChannelCfg) );

        /* The memory should be reserved for Memory logging in DDR.
           In this demo, it uses memory allocated through HPlib*/
        ptrFAPI->memlogMemSize = 0x10000;
        ptrFAPI->memlogMemBase = (uint32_t)hplib_vmMemAlloc(ptrFAPI->memlogMemSize, 1536, 0);

        snprintf(memlogChanConfig.name, MEMLOG_MAX_CHAR, "FAPI_Memlog_%c", phyId);
        memlogChanConfig.memlogInstHandle = Domain_getMemlogInstanceHandle(ptrLTEStackDomain->syslibHandle);
        memlogChanConfig.memRegion        = ptrL2ResourceCfg->memRegionResponse[1].memRegionHandle;
        memlogChanConfig.memBaseAddr      = ptrFAPI->memlogMemBase;
        memlogChanConfig.memSize          = ptrFAPI->memlogMemSize;
        memlogChanConfig.bufferSize       = 1536;
        memlogChanConfig.numPktDescs      = Qmss_getQueueEntryCount(Pktlib_getInternalHeapQueue(ptrFAPI->fapiTracingHeap));

        if ((ptrFAPI->memlogChanHandle = Memlog_create(&memlogChanConfig, errCode)) == NULL)
        {
            System_printf ("Error: MEMLOG create channel Failed [Error Code %d]\n", *errCode);
            return NULL;
        }
        System_printf ("Debug: MEMLOG channel(%p) has been created successfully\n", ptrFAPI->memlogChanHandle );

        /* Push the channel name from the DSP to the ARM realm. */
        if (Name_push (Domain_getNameClientInstanceHandle(ptrLTEStackDomain->syslibHandle), memlogChanConfig.name, Name_ResourceBucket_INTERNAL_SYSLIB,
                       Name_ResourceOperationType_CREATE, errCode) < 0)
        {
            System_printf ("Error: Channel name '%s' PUSH to ARM realm failed [Error code %d] \n", memlogChanConfig.name, *errCode);
            return NULL;
        }

        /* Populate FAPI instance configurations */
        memset(&fapiTracingInstCfg, 0 , sizeof(fapiTracingInstCfg));
        fapiTracingInstCfg.malloc            = FapiTracing_osalMalloc;
        fapiTracingInstCfg.free              = FapiTracing_osalFree;

        /* Create FapiTracing Instance */
        ptrFAPI->fapiTracingHandle = FapiTracing_createInstance(&fapiTracingInstCfg, errCode);
        if(ptrFAPI->fapiTracingHandle == NULL)
        {
            printf ("Error: Create FAPI Tracing instance failed with error: %d\n", *errCode);
            return NULL;
        }
        printf ("Debug: FAPI Tracing Instance %p created successfully\n", ptrFAPI->fapiTracingHandle);

        /* Populate the FAPI Tracing configuration */
        fapiTracingCfg.isEnabled                          = 1;
        fapiTracingCfg.isDataTracingEnabled               = 1;
        fapiTracingCfg.transportCfg.netfpClientHandle     = Domain_getNetfpClientInstanceHandle(ptrLTEStackDomain->syslibHandle);
        fapiTracingCfg.transportCfg.pktlibInstHandle      = ptrFAPI->pktlibInstHandle;
        fapiTracingCfg.transportCfg.heapHandle            = ptrFAPI->fapiTracingHeap;
        fapiTracingCfg.transportCfg.transportType         = TRACING_TRANSPORT_NETFP;

#if FAPI_MEMLOGGING
        fapiTracingCfg.transportCfg.postRoutingHook       = FapiTracing_MemoryCaptureHook;
        fapiTracingCfg.transportCfg.postRoutingHookArg    = (uint32_t)ptrFAPI;
#else
        fapiTracingCfg.transportCfg.postRoutingHook       = NULL;
#endif

        if (phyId == 'A')
        {
            /* Cell A will send fapi over Ipv4 FP */
            fapiTracingCfg.transportCfg.sin_family            = Netfp_SockFamily_AF_INET;
            fapiTracingCfg.transportCfg.egressFastPathHandle  = ptrLTEStackDomain->fapiTracingEgressFastPath;
            fapiTracingCfg.transportCfg.ingressFastPathHandle = ptrLTEStackDomain->fapiTracingIngressFastPath;
            fapiTracingCfg.transportCfg.destUDPPort = 51000;
            fapiTracingCfg.transportCfg.srcUDPPort  = 23000;
        }
        else
        {
            /* Cell A will send fapi over Ipv6 FP */
            fapiTracingCfg.transportCfg.sin_family            = Netfp_SockFamily_AF_INET6;
            fapiTracingCfg.transportCfg.egressFastPathHandle  = ptrLTEStackDomain->fapiTracingIPv6EgressFastPath;
            fapiTracingCfg.transportCfg.ingressFastPathHandle = ptrLTEStackDomain->fapiTracingIPv6IngressFastPath;
            fapiTracingCfg.transportCfg.destUDPPort = 51003;
            fapiTracingCfg.transportCfg.srcUDPPort  = 23003;
        }

        /* Initialize the FAPI Tracing. */
        if (FapiTracing_init (ptrFAPI->fapiTracingHandle, &fapiTracingCfg, errCode) < 0)
        {
            printf ("Error: FAPI Tracing initialization failed with error: %d\n", *errCode);
            return NULL;
        }
        printf ("Debug: FAPI Tracing Instance %p initialized successfully\n", ptrFAPI->fapiTracingHandle);

        /* Create Memlog controller for PHY master FAPI tracing */
        while ( (ptrFAPI->MemlogChanCtrl = Memlog_createMemlogController(Domain_getMemlogInstanceHandle(ptrLTEStackDomain->syslibHandle),
                 memlogChanConfig.name, errCode)) == NULL)
        {
            if(*errCode == NAME_ENOTFOUND)
            {
                /* Can not find the memlog producer wait and try again */
                usleep(10000);

                continue;
            }
            /* Creation failed, print out the error */
            printf("Create memlogging controller for %s failed, error Code = %d \n", memlogChanConfig.name, *errCode);
            return NULL;
        }
#if FAPI_MEMLOGGING
        /* Create thread to handle memory logging */
        *errCode = pthread_create (&ptrFAPI->memlogThread, NULL, Fapi_memLoggingThread, ptrFAPI->memlogChanHandle);
        if (*errCode < 0)
        {
            printf ("Error: Memory logging thread for UIA_CONTRACT_PHY_SLAVE0 failed to start error code %d \n", *errCode);
            return NULL;
        }
#endif
    }

    /* FAPI is active */
    ptrFAPI->fapiStatus = 1;

    /* Initialize the thread attributes. */
    pthread_attr_init(&attr);
    param.sched_priority    = 10;
    pthread_attr_setschedpolicy (&attr, SCHED_FIFO);
    pthread_attr_setschedparam (&attr, &param);

    /* Launch the FAP receive thread */
    *errCode = pthread_create (&ptrFAPI->fapiThread, &attr, Fapi_uintcRcvTask, ptrFAPI);
    if (*errCode < 0)
    {
        printf ("Error: Unable to create the root slave thread [Error code %d]\n", *errCode);
        return NULL;
    }

    /* Return the handle to the FAPI object */
    return (void *)ptrFAPI;
}

/**
 *  @b Description
 *  @n
 *      The function deinitializes the FAPI Interface.
 *
 *  @param[in]  fapiHandle
 *      FAPI handle
 *
 *  @retval
 *      Not applicable
 */
void Fapi_deinitComponent (void* fapiHandle)
{
    Fapi_MCB*   ptrFAPI;
    int32_t     errCode;

    /* Get the FAPI MCB */
    ptrFAPI = (Fapi_MCB*)fapiHandle;
    if (ptrFAPI == NULL)
        return;

    /* Waiting FAPI memory logging thread to terminate */
    gMemlogThreadTerminate = 1;
    pthread_join(ptrFAPI->memlogThread, NULL);

    /* Disable the FAPI Tracing. */
    FapiTracing_configure (ptrFAPI->fapiTracingHandle, 0, 0, &errCode);

    /* Shutdown the writer channels:
     *  - High & Low Priority writer channels which sends messages to the L1  */
    if (ptrFAPI->fapiL1L2HighPriorityChannel != NULL)
    {
        if (Msgcom_delete (ptrFAPI->fapiL1L2HighPriorityChannel, myFreeMsgBuffer) < 0)
            printf ("Error: Unable to delete the high priority writer channel '%p'\n", ptrFAPI->fapiL1L2HighPriorityChannel);
    }
    if (ptrFAPI->fapiL1L2LowPriorityChannel != NULL)
    {
        if (Msgcom_delete (ptrFAPI->fapiL1L2LowPriorityChannel, myFreeMsgBuffer) < 0)
            printf  ("Error: Unable to delete the low priority writer channel '%p'\n", ptrFAPI->fapiL1L2LowPriorityChannel);
    }
    if (ptrFAPI->fapiL1L2MacPduChannel != NULL)
    {
        if (Msgcom_delete (ptrFAPI->fapiL1L2MacPduChannel, myFreeMsgBuffer) < 0)
            printf  ("Error: Unable to delete the low priority writer channel '%p'\n", ptrFAPI->fapiL1L2LowPriorityChannel);
    }

    /* Shutdown the reader channels:
     *  - High & Low Priority Reader channel which receives messages from L1 */
    if (ptrFAPI->fapiL2L1HighPriorityChannel != NULL)
    {
        if (Msgcom_delete (ptrFAPI->fapiL2L1HighPriorityChannel, myFreeMsgBuffer) < 0)
            printf ("Error: Unable to delete the high priority reader channel '%p'\n", ptrFAPI->fapiL1L2HighPriorityChannel);
    }
    if (ptrFAPI->fapiL2L1LowPriorityChannel != NULL)
    {
        if (Msgcom_delete (ptrFAPI->fapiL2L1LowPriorityChannel, myFreeMsgBuffer) < 0)
            printf  ("Error: Unable to delete the low priority reader channel '%p'\n", ptrFAPI->fapiL1L2LowPriorityChannel);
    }
    if (ptrFAPI->fapiL2L1MacPduChannel != NULL)
    {
        if (Msgcom_delete (ptrFAPI->fapiL2L1MacPduChannel, myFreeMsgBuffer) < 0)
            printf  ("Error: Unable to delete the low priority reader channel '%p'\n", ptrFAPI->fapiL1L2LowPriorityChannel);
    }

    /* FAPI is down! Post the semaphore to indicate so */
    ptrFAPI->fapiStatus = 0;

    /* Shutdown the FAPI Tracing Heap */
    if (ptrFAPI->fapiTracingHeap != NULL)
    {
        /* Deinitialize the FAPI Tracing module. */
        FapiTracing_deinit(ptrFAPI->fapiTracingHandle, &errCode);
  
        /* Delete memlog channel */
        memlog_delete( ptrFAPI->memlogChanHandle, &errCode);

        /* Free logging memory */
        hplib_vmMemFree((void *)ptrFAPI->memlogMemBase, ptrFAPI->memlogMemSize, 0);

        /* Shutdown the FAPI Tracing heap. */
        if (Pktlib_deleteHeap (ptrFAPI->pktlibInstHandle, ptrFAPI->fapiTracingHeap, &errCode) < 0)
            printf  ("Error: Unable to delete the FAPI Tracing Heap [Error code %d]\n", errCode);
    }

    /* Shutdown the FAPI DDR3 Heap */
    if (ptrFAPI->fapiDDR3HeapHandle != NULL)
    {
        /* Shutdown the FAPI Tracing heap. */
        if (Pktlib_deleteHeap (ptrFAPI->pktlibInstHandle, ptrFAPI->fapiDDR3HeapHandle, &errCode) < 0)
            printf  ("Error: Unable to delete the FAPI DDR3 Heap [Error code %d]\n", errCode);
    }

    /* Cleanup memory allocated to the FAPI MCB */
    free (ptrFAPI);
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is invoked after the FAPI Interface has been initialized
 *      and is used to initialize & configure the PHY and have it running.
 *
 *  @param[in]  fapiHandle
 *      FAPI handle
 *
 *  @retval
 *      Not applicable
 */
void Fapi_initPhy (void* fapiHandle)
{
    Fapi_MCB*   ptrFAPI;

    /* Get the FAPI MCB */
    ptrFAPI = (Fapi_MCB*)fapiHandle;
    if (ptrFAPI == NULL)
        return;

    /* Send the PARAM request message: This will start the PHY initialization. */
    Fapi_sendMessage(ptrFAPI, TI_FAPI_PARAM_REQUEST, NULL, 0);
    return;
}

