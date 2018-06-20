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

/* BIOS/XDC Include Files. */
#include <xdc/std.h>
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Memory.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <ti/sysbios/family/c64p/Hwi.h>

/* IPC Include Files for Shared Memory Allocation. */
#include <ti/ipc/Ipc.h>
#include <ti/ipc/SharedRegion.h>

/* PDK Include Files. */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/platform/platform.h>

/* CSL Include Files. */
#include <ti/csl/csl_chip.h>
#include <ti/csl/csl_cacheAux.h>

/* SYSLIB Include Files.  */
#include <ti/runtime/msgcom/msgcom.h>
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/netfp/netfp.h>
#include <ti/runtime/resmgr/resmgr.h>

/* Logger streamer Include Files */
#include <ti/uia/events/UIAEvt.h>
#include <ti/uia/runtime/LogUC.h>
#include <ti/uia/runtime/LogSnapshot.h>
#include <ti/uia/sysbios/LoggerStreamer2.h>

/* FAPI Interface */
#include <ti/demo/lte/common/ti_fapi.h>
#include <ti/demo/lte/common/ti_fapiChanNames.h>
#include <ti/runtime/fapi_tracing/fapi_tracing.h>
#include "l2_lte.h"

/* Logger handles. Used to demostrate usage of log_iwrite() */
#include <xdc/cfg/global.h>

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
    Fapi_State            state;

    /**
     * @brief   Subframe numbers
     */
    uint16_t              sfnSf;

    /**
     * @brief   Subframe number
     */
    uint16_t              sfn;

    /**
     * @brief   Subframe
     */
    uint16_t              sf;

    /**
     * @brief   FAPI High Priority MSGCOM channel used to receive messages
     * from the L1
     */
    MsgCom_ChHandle       fapiL1L2HighPriorityChannel;

    /**
     * @brief   FAPI Low Priority MSGCOM channel used to receive messages
     * from the L1
     */
    MsgCom_ChHandle       fapiL1L2LowPriorityChannel;

    /**
     * @brief   FAPI High Priority MSGCOM channel used to send messages
     * to the L1
     */
    MsgCom_ChHandle       fapiL2L1HighPriorityChannel;

    /**
     * @brief   FAPI Low Priority MSGCOM channel used to send messages
     * to the L1
     */
    MsgCom_ChHandle       fapiL2L1LowPriorityChannel;

    /**
     * @brief   FAPI Heap UL Shared Channel to be used.
     */
    Pktlib_HeapHandle     fapiUlSchDataHeapHandle;

    /**
     * @brief   FAPI MSMC Heap
     */
    Pktlib_HeapHandle     fapiMSMCHeapHandle;

    /**
     * @brief   FAPI DDR3 Heap
     */
    Pktlib_HeapHandle     fapiDDR3HeapHandle;

    /**
     * @brief   FAPI Event Object
     */
    Event_Handle          fapiEventHandle;

    /**
     * @brief   FAPI component task handle
     */
    Task_Handle           taskHandle;

    /**
     * @brief   FAPI Tracing statistics
     */
    FapiTracing_Stats     stats;

    /**
     * @brief   PKTLIB Instance Handle associated with the domain
     */
    Pktlib_InstHandle     pktlibInstHandle;

    /**
     * @brief   MSGCOM Instance Handle associated with the domain
     */
    Msgcom_InstHandle     msgcomInstHandle;

    /**
     * @brief   Pointer to the LTE Stack domain
     */
    AppLTEStackDomainMCB* ptrLTEStackDomain;

    /**
     * @brief   FapiTracing instance handle
     */
    FapiTracing_InstHandle fapiTracingHandle;
}Fapi_MCB;

/**********************************************************************
 ************************** Global Variables **************************
 **********************************************************************/

/* Cell Configuration Request (This is just a template) */
TiFapi_configTLV_s configReqTLVs[]=
{
	{
		TI_FAPI_DUPLEXING_MODE,				//T: tag
		0x2,								//L: length
		{0x1},								//V: value
	},
	{
		TI_FAPI_PCFICH_POWER_OFFSET,		//T: tag
		0x2,								//L: length
		{0x1770},							//V: value
	},
	{
		TI_FAPI_P_B,						//T: tag
		0x2,								//L: length
		{0x1},								//V: value
	},
	{
		TI_FAPI_DL_CYCLIC_PREFIX_TYPE,		//T: tag
		0x2,								//L: length
		{0x0},								//V: value
	},
	{
		TI_FAPI_UL_CYCLIC_PREFIX_TYPE,		//T: tag
		0x2,								//L: length
		{0x0},								//V: value
	},
	{
		TI_FAPI_DL_CHANNEL_BANDWIDTH,		//T: tag
		0x2,								//L: length
		{0x64},								//V: value
	},
	{
		TI_FAPI_UL_CHANNEL_BANDWIDTH,		//T: tag
		0x2,								//L: length
		{0x64},								//V: value
	},
	{
		TI_FAPI_REFERENCE_SIGNAL_POWER,		//T: tag
		0x2,								//L: length
		{0x28},								//V: value
	},
	{
		TI_FAPI_TX_ANTENNA_PORTS,			//T: tag
		0x2,								//L: length
		{0x2},								//V: value
	},
	{
		TI_FAPI_RX_ANTENNA_PORTS,			//T: tag
		0x2,								//L: length
		{0x2},								//V: value
	},
	{
		TI_FAPI_PHICH_RESOURCE,				//T: tag
		0x2,								//L: length
		{0x0},								//V: value
	},
	{
		TI_FAPI_PHICH_DURATION,				//T: tag
		0x2,								//L: length
		{0x0},								//V: value
	},
	{
		TI_FAPI_PHICH_POWER_OFFSET,			//T: tag
		0x2,								//L: length
		{0x1770},							//V: value
	},
	{
		TI_FAPI_PRIMARY_SYNC_SIGNAL,		//T: tag
		0x2,								//L: length
		{0x1770},							//V: value
	},
	{
		TI_FAPI_SECONDARY_SYNC_SIGNAL,		//T: tag
		0x2,								//L: length
		{0x1770},							//V: value
	},
	{
		TI_FAPI_PHYSICAL_CELL_ID,			//T: tag
		0x2,								//L: length
		{0x1C0},							//V: value
	},
	{
		TI_FAPI_CONFIGURATION_INDEX,		//T: tag
		0x2,								//L: length
		{0x3},								//V: value
	},
	{
		TI_FAPI_ROOT_SEQUENCE_INDEX,		//T: tag
		0x2,								//L: length
		{0x7B},								//V: value
	},
	{
		TI_FAPI_ZERO_CORRELATION_ZONE_CONFIGURATION,		//T: tag
		0x2,								//L: length
		{0x9},								//V: value
	},
	{
		TI_FAPI_HIGH_SPEED_FLAG,			//T: tag
		0x2,								//L: length
		{0x0},								//V: value
	},
	{
		TI_FAPI_FREQUENCY_OFFSET,			//T: tag
		0x2,								//L: length
		{0x3},								//V: value
	},
	{
		TI_FAPI_HOPPING_MODE,				//T: tag
		0x2,								//L: length
		{0x0},								//V: value
	},
	{
		TI_FAPI_HOPPIG_OFFSET,				//T: tag
		0x2,								//L: length
		{0x0},								//V: value
	},
	{
		TI_FAPI_NUM_OF_SUB_BANDS,			//T: tag
		0x2,								//L: length
		{0x1},								//V: value
	},
	{
		TI_FAPI_DELTA_PUCCH_SHIFT,			//T: tag
		0x2,								//L: length
		{0x2},								//V: value
	},
	{
		TI_FAPI_N_CQI_RB,					//T: tag
		0x2,								//L: length
		{0x0},								//V: value
	},
	{
		TI_FAPI_N_AN_CS,					//T: tag
		0x2,								//L: length
		{0x0},								//V: value
	},
	{
		TI_FAPI_N_1_PUCCH_AN,				//T: tag
		0x2,								//L: length
		{0x5},								//V: value
	},
	{
		TI_FAPI_BANDWIDTH_CONFIGURATION,	//T: tag
		0x2,								//L: length
		{0x0},								//V: value
	},
	{
		TI_FAPI_MAX_UP_PTS,					//T: tag
		0x2,								//L: length
		{0x0},								//V: value
	},
	{
		TI_FAPI_SRS_SUB_FRAME_CONFIGURATION,//T: tag
		0x2,								//L: length
		{0x0},								//V: value
	},
	{
		TI_FAPI_SRS_ACK_NACK_SRS_SIMULTANEOUS_TRANSMISSION,		//T: tag
		0x1,								//L: length
		{0x0},								//V: value
	},
	{
		TI_FAPI_UPLINK_RS_HOPPING,			//T: tag
		0x2,								//L: length
		{0x0},								//V: value
	},
	{
		TI_FAPI_GROUP_ASSIGNMENT,			//T: tag
		0x2,								//L: length
		{0x0},								//V: value
	},
	{
		TI_FAPI_CYCLIC_SHIFT_1_FOR_DMRS,	//T: tag
		0x2,								//L: length
		{0x0},								//V: value
	},
	{
		TI_FAPI_DATA_REPORT_MODE,			//T: tag
		0x2,								//L: length
		{0x0},								//V: value
	},
	{
		TI_FAPI_HEADER_VERSION,				//T: tag
		0x2,								//L: length
		{0x0107},							//V: value
	}
};

TiFapi_configTLVExt_s configReqTLVExt_s[]=
{
    {
        TI_FAPI_AIF2_TRIGGER_TYPE,      //T: tag
        0x4,							//L: length
        {0x0, 0x0,},                     //pad[2]
        {0},							//V: value
    },
	{
		TI_FAPI_DL_SAMPLING_FREQ,	    //T: tag
		0x4,							//L: length
        {0x0, 0x0,},                     //pad[2]
		{30720},						//V: value
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

        /* the following is for 8 UEs */
        {0}, {0}, {0}, {0},

    },

    /* DLSCH PDU */
    {
    	/* DLSCH pdu for SIB1 request */
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
	0x0000,		//	sfnSf;                                          /* SFN/SF*/
	0, 		//	numUlschPdus;                                   /* The number of PDUs in the ulsch array*/
	0,		//  numUciPdus;                                     /* The number of PDUs in the uci array*/
	0,		//  numSrsPdus;                                     /* The number of PDUs in the srs array*/
	0,		//  prachResource;                                  /* For FDD 0 means no RACH in this sf, 1 means rach present in this sf;*/
																 /*for TDD bits 0:5 indicates the RACH resources used*/
	0,		//   srsPresent;                                     /* The SRS present flag*/
}};

/* UL Config Request (This is just a template) */
TiFapi_ulCfgReq_s  Fapi_ulConfigRequestWithULData[] =
{{
	0x0000,		//	sfnSf;                                          /* SFN/SF*/
	1, 		//	numUlschPdus;                                   /* The number of PDUs in the ulsch array*/
	0,		//  numUciPdus;                                     /* The number of PDUs in the uci array*/
	0,		//  numSrsPdus;                                     /* The number of PDUs in the srs array*/
	0,		//  prachResource;                                  /* For FDD 0 means no RACH in this sf, 1 means rach present in this sf;*/
																 /*for TDD bits 0:5 indicates the RACH resources used*/
	0,		//   srsPresent;                                     /* The SRS present flag*/

	/* Ulsch pdus */
	{{
		0, 	//	pduType
	    {{{0x7054, 	//	                       handle;                    /* An opaque handle that will be returned to L23*/
	    0xf7b, 		//	                       size;                      /* The size of the PDU in bytes Can be 0 in case that only HARQ and CQI is received on ULSCH*/
	    0x33, 		//                         rnti;                      /* The RNTI used for identifying the UE when receiving the PDU*/
	    0,			//                         resourceBlockStart;        /* Resource indication value corresponding to the starting resource block*/
	    0x40,		//                          numRbs;                    /* The number of resource blocks this PDU will be mapped to*/
	    0x4,		//           				modulationType;            /* Modulation Type*/
	    0, 			//                           n2Dmrs;                    /* The 2nd cyclic shift for DMRS assigned to the UE in the relevant UL grant*/
	    0,			//				     		hoppingFlag;               /* Frequency hopping flag*/
	    0x0,		//                           hoppingBits;               /* Frequency hopping bits*/
	    0x1, 		//                           newDataIndicator;          /* New data indicator for TB 0 Specify whether this received PUSCH is a new transmission from UE*/
	    0x0,		//                           redundancyVersion;         /* Redundancy Version*/
	    0x0,		//                           harqProcessNumber;         /* The HARQ process number*/
	    0x0,		//                 			txMode;                    /* UL Tx Mode*/
	    0x0,		//                           currentTxNb;               /* The current HARQ transmission count of this TX Valid if frequency hopping is enabled*/
	    0x0,}}}		//           				nSrs;                      /* Indicates if the RBs allocated for this grant overlap with the SRS configuration*/
	},},
}};
/* System Information Block1 Request (This is just a template) */
#pragma DATA_ALIGN (Fapi_sib1, 128)
uint8_t Fapi_sib1[] =
{
    0x48,0x66,0x1e,0xca,0x93,0x13,0x60,0x00,0x01,0xc0,0xc5,0x00,0x29,0x03,0x08,0x18,
    0x42,0xc2,0x26,0x11,0xb0,0x99,0x81,0x55,0x60,0x00
};

/* System Information Block (This is just a template) */
#pragma DATA_ALIGN (Fapi_sib2, 128)
uint8_t Fapi_sib2[] =
{
    0x00,0x00,0x02,0x7c,0xfd,0x0c,0x87,0xb0,0xd2,0x11,0xe2,0x80,0x00,0x04,0x10,0x00,
    0xa3,0x3b,0xb0,0x00,0x9f,0xa0,0x83,0x41,0xc0,0x00
};

/* Master Information Block Request (This is just a template) */
#pragma DATA_ALIGN (Fapi_mib, 128)
uint8_t Fapi_mib[3] =
{
    0xa2,0x84,0x00
};

/**********************************************************************
 ************************* Extern Definitions *************************
 **********************************************************************/

/* Cache operations. */
extern void appInvalidateBuffer(void* ptr, uint32_t size);
extern void appWritebackBuffer(void* ptr, uint32_t size);

/* FapiTracing Osal functions */
extern void* FapiTracing_osalMalloc(uint32_t numBytes, uint32_t alignment);
extern void FapiTracing_osalFree(void* ptr, uint32_t numBytes);

//extern uint32_t get_ArmTrace_Sem(void);

/**********************************************************************
 ************************** FAPI Functions ****************************
 **********************************************************************/

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
    System_printf ("Debug: Channel Handle 0x%p is being deleted MSGCOM Buffer is 0x%p\n", chHandle, msgBuffer);
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
    Error_Block	errorBlock;

    /* Allocate memory from the shared memory heap. */
    return Memory_alloc ((xdc_runtime_IHeap_Handle)arg, size, 0, &errorBlock);
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
    /* Cleanup the memory block. */
    Memory_free ((xdc_runtime_IHeap_Handle)arg, ptr, size);
}

/**
 *  @b Description
 *  @n
 *      Heap Data Buffer Cleanup which is instantiated into the heap
 *      interface table.
 *
 *  @param[in]  nameClientHandle
 *      Handle to the name client
 *  @param[in]  nameDatabaseId
 *      Identifier associated with the name database
 *  @param[in]  name
 *      Name which is to be synchronized
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Fapi_synch
(
    Name_ClientHandle   nameClientHandle,
    uint32_t            nameDatabaseId,
    char*               name,
    int32_t*            errCode
)
{
    Name_ResourceCfg    nameResourceCfg;

    /* Perform a remote get to determine if the name has been created in the RAT database
     * It is possible that the name does not exist in the remote database in that case we
     * wait & synchronize */
    while (1)
    {
        if (Name_get (nameClientHandle, nameDatabaseId, Name_ResourceBucket_INTERNAL_SYSLIB,
                      name, &nameResourceCfg, errCode) == 0)
        {
            /* Name has been found in the RAT database. */
            break;
        }
        if (*errCode != NAME_ENOTFOUND)
        {
            System_printf ("Error: Remote get for name '%s' failed. [Error code %d]\n", name, *errCode);
            return -1;
        }
        Task_sleep(1);
    }

    /* Name has been found in the RAT database. Create the entry in the local database also. */
    if (Name_createResource (Name_getDatabaseHandle(nameDatabaseId), Name_ResourceBucket_INTERNAL_SYSLIB,
                             &nameResourceCfg, errCode) < 0)
    {
        System_printf ("Error: Unable to create the name '%s' in the local database. [Error code %d]\n", name, *errCode);
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      FAPI Message callback function which is invoked whenever a message
 *      is received on the High Priority FAPI channel.
 *
 *  @param[in]  chHandle
 *      FAPI MSGCOM Channel on which the message is received
 *  @param[in]  arg
 *      Optional application defined argument
 *
 *  @retval
 *      Not Applicable.
 */
static void Fapi_highPriorityMsgCallback(MsgCom_ChHandle chHandle, uint32_t arg)
{
    Event_post((Event_Handle)arg, Event_Id_00);
    return;
}

/**
 *  @b Description
 *  @n
 *      FAPI Message callback function which is invoked whenever a message
 *      is received on the Low Priority FAPI channel.
 *
 *  @param[in]  chHandle
 *      FAPI MSGCOM Channel on which the message is received
 *  @param[in]  arg
 *      Optional application defined argument
 *
 *  @retval
 *      Not Applicable.
 */
static void Fapi_lowPriorityMsgCallback(MsgCom_ChHandle chHandle, uint32_t arg)
{
    Event_post((Event_Handle)arg, Event_Id_01);
    return;
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
    int32_t				errCode;

    /* Populate the message on the basis of the message type */
    if (msgType == TI_FAPI_PARAM_REQUEST)
    {
        /* Allocate memory from the FAPI DDR3 Heap. */
        ptrFAPIPkt = Pktlib_allocPacket(ptrFAPI->pktlibInstHandle,
                                        ptrFAPI->fapiDDR3HeapHandle, sizeof(TiFapi_msg_s));
        if (ptrFAPIPkt == NULL)
        {
            System_printf ("Error: Unable to allocate the FAPI Message [PARAM Request]\n");
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
    	TiFapi_configReq_s	  *ptrConfigReqMsg;
    	uint32_t 			  numOfExtTLVs, numOfStdTLVs, numOfTLVs;
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
            System_printf ("Error: Unable to allocate the FAPI Message [Cell Config]\n");
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
        ptrConfigReqMsg 				  = (TiFapi_configReq_s *)ptrFAPIMessage->msgBody;

        /* Determine number of TLVs from configReqTLV array */
        ptrConfigReqMsg->numOfTlv 		  = numOfTLVs;
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
            System_printf ("Error: Unable to allocate the FAPI Message [START Request]\n");
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
                                        ptrFAPI->fapiMSMCHeapHandle,
                                        sizeof(TiFapi_nonTlvMsgHeader_s) + dlConfigMsgSize);
        if (ptrFAPIPkt == NULL)
        {
            System_printf ("Error: Unable to allocate the FAPI Message [DL Config]\n");
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
            memcpy ((void *)ptrFAPIMessage->msgBody, (void *)&Fapi_dlConfigMIBRequest[0] , sizeof(Fapi_dlConfigMIBRequest));
        else if ((ptrFAPI->sf == 5) && ((ptrFAPI->sfn % 2) == 0))
            memcpy ((void *)ptrFAPIMessage->msgBody, (void *)&Fapi_dlConfigSIB1Request , sizeof(Fapi_dlConfigSIB1Request));
        else if ((ptrFAPI->sf == 6) && ((ptrFAPI->sfn % 16) == 0))
            memcpy ((void *)ptrFAPIMessage->msgBody, (void *)&Fapi_dlConfigSIB2Request , sizeof(Fapi_dlConfigSIB2Request));
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
            System_printf ("Error: Unable to allocate the FAPI Message [UL Config]\n");
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

        /* construct a tx.request message ,
         if the application cannot provide memorySize, then use the fillowing length for allocation */
        memorySize = sizeof(TiFapi_nonTlvMsgHeader_s) + sizeof(TiFapi_txReq_s);

        /* Allocate memory from the FAPI DDR3 Heap. */
        ptrFAPIPkt = Pktlib_allocPacket(ptrFAPI->pktlibInstHandle,
                                        ptrFAPI->fapiDDR3HeapHandle, memorySize);
        if (ptrFAPIPkt == NULL)
        {
            System_printf ("Error: Unable to allocate the FAPI Message [DL Tx Request]\n");
            return -1;
        }

        /* Get the data buffer associated with the FAPI packet. */
        Pktlib_getDataBuffer(ptrFAPIPkt, (uint8_t**)&ptrDataBuffer, &messageLength);

        /* Populate the FAPI message */
        ptrFAPIMessage = (TiFapi_msg_s*)ptrDataBuffer;
        ptrFAPIMessage->msgId             = msgType;
        ptrFAPIMessage->lenVendorSpecific = 0;
        ptrFAPIMessage->msgLen            = sizeof(TiFapi_txReq_s);

        /* Get the FAPI DL Tx request */
        ptrFapiTxRequest = (TiFapi_txReq_s*)&ptrFAPIMessage->msgBody[0];

        /* Initialize the FAPI DL Transmit request  */
        ptrFapiTxRequest->sfnSf     = ptrFAPI->sfnSf;
        ptrFapiTxRequest->numPdus   = 1;

        /* Populate the PDU Information. */
        ptrFapiTxRequest->u.pdu[0].pduIndex        = 0;
        ptrFapiTxRequest->u.pdu[0].numScatBuf      = 1;

        /* Initialize the scattered base address and length. */
        ptrFapiTxRequest->u.pdu[0].data[0].bufBasePtr = l2_global_address((uint32_t)ptrPayload);
        ptrFapiTxRequest->u.pdu[0].data[0].bufLen  = (uint32_t)payloadLen;

        /* write back payload buffer */
        appWritebackBuffer((uint8_t*)ptrPayload, payloadLen);

        /* Send the message on the low priority channel */
        fapiChannel = ptrFAPI->fapiL2L1LowPriorityChannel;

        /* Compress the packet and corrected the msglen in FAPI header */
        ptrFAPIMessage->msgLen = FapiTracing_compressTxRequest((TiFapi_nonTlvMsg_s *)ptrFAPIMessage);
        messageLength = sizeof(TiFapi_nonTlvMsgHeader_s) +  ptrFAPIMessage->msgLen;

       /* Update the data buffer length for the packet for the 
        * compressed mesage length.
        */
        Pktlib_setDataBufferLen(ptrFAPIPkt, messageLength);

    }
    else
    {
        /* Currently no other message is supported. */
        return -1;
    }

    /* Set the packet length. */
    Pktlib_setPacketLen(ptrFAPIPkt, messageLength);

    /* Writeback the data buffer */
    appWritebackBuffer(ptrFAPIMessage, messageLength);

    /* Pass the message to the FAPI Tracing Module.
     *  - Trace the entire packet which is being transmitted */
    if(FapiTracing_trace(ptrFAPI->fapiTracingHandle, ptrFAPIPkt, &errCode) < 0)
    {
    	//System_printf("Error: FapiTracing for packet(msgid=%x) failed, errCode=%d\n", msgType, errCode);
    	/* Error condition, increase FAPI tracing dropped packet counts */
    	ptrFAPI->stats.DroppedPackets++;
    }
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
    int32_t				errCode;

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

        /* Invalidate the FAPI Message */
        appInvalidateBuffer(ptrFAPIMessage, fapiMessageLen);

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
            	//System_printf("Error: FapiTracing for packet(msgid=%x) failed, errCode=%d\n", ptrFAPIMessage->msgId, errCode);

            /* Send the Config message */
            Fapi_sendMessage(ptrFAPI, TI_FAPI_CELL_CONFIG_REQUEST, NULL, 0);
        }
        else if (ptrFAPIMessage->msgId == TI_FAPI_CELL_CONFIG_RESPONSE)
        {
            /* Cell Configuration Response has been received. */
            ptrFAPI->state = Fapi_State_CELL_CONFIG_RESPONSE;

            /* Send the FAPI Packet to the Tracer. */
            if(FapiTracing_trace(ptrFAPI->fapiTracingHandle, ptrFAPIPkt, &errCode) < 0)
            	//System_printf("Error: FapiTracing for packet(msgid=%x) failed, errCode=%d\n", ptrFAPIMessage->msgId, errCode);
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
            	//System_printf("Error: FapiTracing for packet(msgid=%x) failed, errCode=%d\n", ptrFAPIMessage->msgId, errCode);
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
                Fapi_mib[0] = 0xa0 | ((ptrFAPI->sfn & 0xF00) >> 8);
                Fapi_mib[1] = (ptrFAPI->sfn & 0xFF);

                /* Update the subframe indication inside the FAPI MIB message. */
                Fapi_sendMessage(ptrFAPI, TI_FAPI_DL_TX_REQUEST, (uint8_t*)&Fapi_mib, sizeof(Fapi_mib));
            }

            /* Send the System Information block1 message. */
            if ((ptrFAPI->sf == 5) && ((ptrFAPI->sfn % 2) == 0))
                Fapi_sendMessage(ptrFAPI, TI_FAPI_DL_TX_REQUEST, (uint8_t*)&Fapi_sib1, sizeof(Fapi_sib1));

            /* Send the System Information block2 message. */
            if ((ptrFAPI->sf == 6) && ((ptrFAPI->sfn % 16) == 0))
                Fapi_sendMessage(ptrFAPI, TI_FAPI_DL_TX_REQUEST, (uint8_t*)&Fapi_sib2, sizeof(Fapi_sib2));
        }
        else
        {
            /* Send the FAPI Packet to the Tracer. */
            if(FapiTracing_trace(ptrFAPI->fapiTracingHandle, ptrFAPIPkt, &errCode) < 0)
            	//System_printf("Error: FapiTracing for packet(msgid=%x) failed, errCode=%d\n", ptrFAPIMessage->msgId, errCode);
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
 *      This is the FAPI Receive Task
 *
 *  @retval
 *      Not Applicable
 */
static void Fapi_receiveTask(UArg arg0, UArg arg1)
{
    uint32_t        events;
    Fapi_MCB*       ptrFAPI;

    /* Get the pointer to the FAPI MCB */
    ptrFAPI = (Fapi_MCB*)arg0;

    /* Loop around forever. */
    while (1)
    {
        /* Wait for an event to occur. */
        events = Event_pend(ptrFAPI->fapiEventHandle, Event_Id_NONE,
                            Event_Id_00 + Event_Id_01, BIOS_WAIT_FOREVER);

        /* Did we get a message on the HIGH Priority Channel? If so process all
         * the FAPI messages */
        if (events & Event_Id_00)
            Fapi_processMessage(ptrFAPI, ptrFAPI->fapiL1L2HighPriorityChannel);

        /* Did we get a message on the LOW Priority Channel? If so process all
         * the FAPI messages */
        if (events & Event_Id_01)
            Fapi_processMessage(ptrFAPI, ptrFAPI->fapiL1L2LowPriorityChannel);
    }
}

/**
 *  @b Description
 *  @n
 *      The function initializes the FAPI Interface.
 *
 *  @param[in]  ptrLTEStackDomain
 *      Pointer to the LTE Stack domain MCB
 *  @param[in]  phyId
 *      Phy identifier
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
    int32_t*                errCode
)
{
    Task_Params           taskParams;
    Msgcom_ChannelCfg     chConfig;
    Fapi_TracingCfg       fapiTracingCfg;
    Pktlib_HeapCfg        heapCfg;
    Fapi_MCB*             ptrFAPI;
    FapiTracing_InstCfg   fapiTracingInstCfg;
    Resmgr_ResourceCfg    fapiResourceConfig =
    {
        0,    /* Number of CPINTC Output  requested                               */
        1,    /* Number of Accumulator Channels requested                         */
        0,    /* Number of Hardware Semaphores requested                          */
        1,    /* Number of QPEND Queues requested                                 */
        /* Requested Memory Region Configuration. */
        {
		    /* Name,           Type,                     Linking RAM,                          Num,    Size */
        	{ "FAPI-ULSCH",  Resmgr_MemRegionType_DDR3,  Resmgr_MemRegionLinkingRAM_DONT_CARE, 1024,     128},
        }
    };

    /* Allocate memory for the FAPI object. */
    ptrFAPI = Memory_alloc ((xdc_runtime_IHeap_Handle)ptrLTEStackDomain->ddr3PrivateHeapHandle, sizeof(Fapi_MCB), 0, NULL);
    if (ptrFAPI == NULL)
	{
		System_printf ("Error: Unable to allocate memory for the FAPI object\n", DNUM);
		return NULL;
	}

    /* Initialize the FAPI Master Control block */
    memset((void *)ptrFAPI, 0, sizeof(Fapi_MCB));

    /* Keep a copy to the application domain configuration. */
    ptrFAPI->ptrLTEStackDomain  = ptrLTEStackDomain;
    ptrFAPI->pktlibInstHandle   = Domain_getPktlibInstanceHandle(ptrLTEStackDomain->syslibHandle);
    ptrFAPI->msgcomInstHandle   = Domain_getMsgcomInstanceHandle(ptrLTEStackDomain->syslibHandle);

    /* Request the resource manager for the resources requested by the FAPI component. */
    if (Resmgr_processConfig (Domain_getSysCfgHandle(ptrLTEStackDomain->syslibHandle), &fapiResourceConfig, errCode) < 0)
	{
	    System_printf ("Error: SYSRM configuration failed with error code %d\n", *errCode);
	    return NULL;
    }

    /* Create the FAPI Event object */
    ptrFAPI->fapiEventHandle = Event_create(NULL, NULL);
    if (ptrFAPI->fapiEventHandle == NULL)
    {
        System_printf ("Error: FAPI Event Object creation failed\n");
        return NULL;
    }

    /*************************************************************************
     * PKTLIB FAPI ULSCH Heap:
     *************************************************************************/

    /* Initialize the heap configuration. */
    memset ((void *)&heapCfg, 0, sizeof(Pktlib_HeapCfg));

    /* Populate the heap configuration */
    if (phyId == 'A')
        strcpy (heapCfg.name, TI_FAPI_CELLA_ULSCH_DATA_HEAP_NAME);
    else
        strcpy (heapCfg.name, TI_FAPI_CELLB_ULSCH_DATA_HEAP_NAME);
    heapCfg.pktlibInstHandle                = ptrFAPI->pktlibInstHandle;
    heapCfg.memRegion                       = fapiResourceConfig.memRegionResponse[0].memRegionHandle;
    heapCfg.sharedHeap                      = 1;
    heapCfg.useStarvationQueue              = 0;
    heapCfg.dataBufferSize                  = sizeof(TiFapi_nonTlvMsg_s);
    heapCfg.numPkts                         = 64;
    heapCfg.numZeroBufferPackets            = 0;
    heapCfg.dataBufferPktThreshold          = 0;
    heapCfg.zeroBufferPktThreshold          = 0;
    heapCfg.arg                             = (uint32_t)SharedRegion_getHeap(1);
    heapCfg.heapInterfaceTable.dataMalloc   = Fapi_DDR3MemoryMalloc;
    heapCfg.heapInterfaceTable.dataFree     = Fapi_DDR3MemoryFree;

    /* Create the FAPI Heap. */
    ptrFAPI->fapiUlSchDataHeapHandle = Pktlib_createHeap(&heapCfg, errCode);
    if (ptrFAPI->fapiUlSchDataHeapHandle == NULL)
    {
	    System_printf ("Error: Unable to create ULSCH heap [Error code %d]\n", *errCode);
        return NULL;
    }

    /* Push the ULSCH heap to the RAT database */
    if (Name_push (Domain_getNameClientInstanceHandle(ptrLTEStackDomain->syslibHandle), heapCfg.name,
                   Name_ResourceBucket_INTERNAL_SYSLIB, Name_ResourceOperationType_CREATE, errCode) < 0)
    {
        System_printf ("Error: Unable to push the ULSCH heap name '%s' between realms [Error code %d]\n", heapCfg.name, *errCode);
        return NULL;
    }
    System_printf ("Debug: ULSCH Heap name '%s' pushed successfully\n", heapCfg.name);

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
    chConfig.mode                                                   = Msgcom_ChannelMode_NON_BLOCKING;
    chConfig.arg                                                    = (uint32_t)ptrFAPI->fapiEventHandle;
    chConfig.appCallBack                                            = Fapi_highPriorityMsgCallback;
    chConfig.msgcomInstHandle                                       = ptrFAPI->msgcomInstHandle;
    chConfig.u.queueCfg.interruptMode                               = Msgcom_QueueInterruptMode_DIRECT_INTERRUPT;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.queuePendQueue  = fapiResourceConfig.qPendResponse[0].queue;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.cpIntcId        = fapiResourceConfig.qPendResponse[0].cpIntcId;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.systemInterrupt = fapiResourceConfig.qPendResponse[0].systemInterrupt;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.hostInterrupt   = fapiResourceConfig.qPendResponse[0].hostInterrupt;

    /* Create the FAPI Channel: The channel name is selected on the basis of which PHY is being initialized. */
    if (phyId == 'A')
    {
    	ptrFAPI->fapiL1L2HighPriorityChannel = Msgcom_create (TI_FAPI_CELLA_L1L2_HP_CHANNEL_NAME,
                                                              Msgcom_ChannelType_QUEUE, &chConfig, errCode);
        if (ptrFAPI->fapiL1L2HighPriorityChannel == 0)
        {
            System_printf ("Error: Unable to create the High Priority FAPI channel [Error code : %d]\n", *errCode);
            return NULL;
        }
        System_printf ("Debug: FAPI High Priority FAPI channel '%s' created\n", TI_FAPI_CELLA_L1L2_HP_CHANNEL_NAME);

        /* Push the channel name across the execution realm. */
        if (Name_push (Domain_getNameClientInstanceHandle(ptrLTEStackDomain->syslibHandle), TI_FAPI_CELLA_L1L2_HP_CHANNEL_NAME,
                       Name_ResourceBucket_INTERNAL_SYSLIB, Name_ResourceOperationType_CREATE, errCode) < 0)
        {
            System_printf ("Error: Unable to push the FAPI channel name between realms [Error code %d]\n", *errCode);
            return NULL;
        }
        System_printf ("Debug: FAPI High Priority FAPI channel '%s' pushed\n", TI_FAPI_CELLA_L1L2_HP_CHANNEL_NAME);
    }
    else
    {
    	ptrFAPI->fapiL1L2HighPriorityChannel = Msgcom_create (TI_FAPI_CELLB_L1L2_HP_CHANNEL_NAME,
                                                              Msgcom_ChannelType_QUEUE, &chConfig, errCode);
        if (ptrFAPI->fapiL1L2HighPriorityChannel == 0)
        {
            System_printf ("Error: Unable to create the High Priority FAPI channel [Error code : %d]\n", *errCode);
            return NULL;
        }
        System_printf ("Debug: FAPI High Priority FAPI channel '%s' created\n", TI_FAPI_CELLB_L1L2_HP_CHANNEL_NAME);

        /* Push the channel name across the execution realm. */
        if (Name_push (Domain_getNameClientInstanceHandle(ptrLTEStackDomain->syslibHandle), TI_FAPI_CELLB_L1L2_HP_CHANNEL_NAME,
                       Name_ResourceBucket_INTERNAL_SYSLIB, Name_ResourceOperationType_CREATE, errCode) < 0)
        {
            System_printf ("Error: Unable to push the FAPI channel name between realms [Error code %d]\n", *errCode);
            return NULL;
        }
        System_printf ("Debug: FAPI High Priority FAPI channel '%s' pushed\n", TI_FAPI_CELLB_L1L2_HP_CHANNEL_NAME);
    }

    /*************************************************************************
     * FAPI Low Priority Channel
     *   This is the Low Priority Channel which is created by the L2 (Reader)
     *   and is used to  receive messages from the L1(Writer)
     *   It is *recommended* that applications verify the channel
     *   configuration and modify it while designing the FAPI Interface.
     *************************************************************************/

    /* Initialize the channel configuration */
    memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* Populate the channel configuration. */
    chConfig.mode                                                   = Msgcom_ChannelMode_NON_BLOCKING;
    chConfig.arg                                                    = (uint32_t)ptrFAPI->fapiEventHandle;
    chConfig.appCallBack                                            = Fapi_lowPriorityMsgCallback;
    chConfig.msgcomInstHandle                                       = ptrFAPI->msgcomInstHandle;
    chConfig.u.queueCfg.interruptMode                               = Msgcom_QueueInterruptMode_ACCUMULATED_INTERRUPT;
    chConfig.u.queueCfg.queueIntrUnion.accCfg.type                  = Msgcom_AccumulatedChannelType_HIGH;
    chConfig.u.queueCfg.queueIntrUnion.accCfg.accChannel            = fapiResourceConfig.accChannelResponse[0].accChannel;
    chConfig.u.queueCfg.queueIntrUnion.accCfg.accQueue              = fapiResourceConfig.accChannelResponse[0].queue;
    chConfig.u.queueCfg.queueIntrUnion.accCfg.pdspId                = fapiResourceConfig.accChannelResponse[0].pdspId;
    chConfig.u.queueCfg.queueIntrUnion.accCfg.interruptId           = fapiResourceConfig.accChannelResponse[0].eventId;
    chConfig.u.queueCfg.queueIntrUnion.accCfg.maxPageEntries        = 5;
	chConfig.u.queueCfg.queueIntrUnion.accCfg.pacingTimerCount      = 0;

    /* Create the FAPI Channel: The channel name is selected on the basis of which PHY is being initialized. */
    if (phyId == 'A')
    {
	    ptrFAPI->fapiL1L2LowPriorityChannel = Msgcom_create (TI_FAPI_CELLA_L1L2_LP_CHANNEL_NAME,
                                                              Msgcom_ChannelType_QUEUE, &chConfig, errCode);
        if (ptrFAPI->fapiL1L2LowPriorityChannel == 0)
        {
            System_printf ("Error: Unable to create the Low Priority FAPI channel Error : %d\n", *errCode);
            return NULL;
        }
        System_printf ("Debug: FAPI Low Priority FAPI channel '%s' created\n", TI_FAPI_CELLA_L1L2_LP_CHANNEL_NAME);

        /* Push the channel name across the execution realm. */
        if (Name_push (Domain_getNameClientInstanceHandle(ptrLTEStackDomain->syslibHandle), TI_FAPI_CELLA_L1L2_LP_CHANNEL_NAME,
                       Name_ResourceBucket_INTERNAL_SYSLIB, Name_ResourceOperationType_CREATE, errCode) < 0)
        {
            System_printf ("Error: Unable to push the FAPI channel name between realms [Error code %d]\n", *errCode);
            return NULL;
        }
        System_printf ("Debug: FAPI Low Priority FAPI channel '%s' pushed\n", TI_FAPI_CELLA_L1L2_LP_CHANNEL_NAME);
    }
    else
    {
	    ptrFAPI->fapiL1L2LowPriorityChannel = Msgcom_create (TI_FAPI_CELLB_L1L2_LP_CHANNEL_NAME,
                                                              Msgcom_ChannelType_QUEUE, &chConfig, errCode);
        if (ptrFAPI->fapiL1L2LowPriorityChannel == 0)
        {
            System_printf ("Error: Unable to create the Low Priority FAPI channel Error : %d\n", *errCode);
            return NULL;
        }
        System_printf ("Debug: FAPI Low Priority FAPI channel '%s' created\n", TI_FAPI_CELLB_L1L2_LP_CHANNEL_NAME);

        /* Push the channel name across the execution realm. */
        if (Name_push (Domain_getNameClientInstanceHandle(ptrLTEStackDomain->syslibHandle), TI_FAPI_CELLB_L1L2_LP_CHANNEL_NAME,
                       Name_ResourceBucket_INTERNAL_SYSLIB, Name_ResourceOperationType_CREATE, errCode) < 0)
        {
            System_printf ("Error: Unable to push the FAPI channel name between realms [Error code %d]\n", *errCode);
            return NULL;
        }
        System_printf ("Debug: FAPI Low Priority FAPI channel '%s' pushed\n", TI_FAPI_CELLB_L1L2_LP_CHANNEL_NAME);
    }

    /* Find the peer MSGCOM channels; this is a synchronization point which allows the L2 and L1
     * to synchronize with each other. */
    if (phyId == 'A')
    {
        Name_ClientHandle   nameClientHandle = Domain_getNameClientInstanceHandle(ptrLTEStackDomain->syslibHandle);
        uint32_t            nameDatabaseId   = Name_getDatabaseInstanceId(Domain_getDatabaseHandle(ptrLTEStackDomain->syslibHandle), errCode);

        /* Synchronize for the L1 High priority channel. */
        if (Fapi_synch (nameClientHandle, nameDatabaseId, TI_FAPI_CELLA_L2L1_HP_CHANNEL_NAME, errCode) < 0)
            return NULL;

        /* Use the MSGCOM API to find the channel. The channel should always be present in the local database. */
        ptrFAPI->fapiL2L1HighPriorityChannel = Msgcom_find (ptrFAPI->msgcomInstHandle,
                                                            TI_FAPI_CELLA_L2L1_HP_CHANNEL_NAME, errCode);
        if (ptrFAPI->fapiL2L1HighPriorityChannel == NULL)
        {
            System_printf ("Error: Unable to find the channel '%s' [Error code %d]\n", TI_FAPI_CELLA_L2L1_HP_CHANNEL_NAME);
            return NULL;
        }

        /* Synchronize for the L1 Low priority channel. */
        if (Fapi_synch (nameClientHandle, nameDatabaseId, TI_FAPI_CELLA_L2L1_LP_CHANNEL_NAME, errCode) < 0)
            return NULL;

        /* Use the MSGCOM API to find the channel. The channel should always be present in the local database. */
        ptrFAPI->fapiL2L1LowPriorityChannel = Msgcom_find (ptrFAPI->msgcomInstHandle,
                                                           TI_FAPI_CELLA_L2L1_LP_CHANNEL_NAME, errCode);
        if (ptrFAPI->fapiL2L1LowPriorityChannel == NULL)
        {
            System_printf ("Error: Unable to find the channel '%s' [Error code %d]\n", TI_FAPI_CELLA_L2L1_LP_CHANNEL_NAME);
            return NULL;
        }

        /* Synchronize for the L1 MSMC Heap */
        if (Fapi_synch (nameClientHandle, nameDatabaseId, TI_FAPI_CELLA_MSG_HEAP_MSMC_NAME, errCode) < 0)
            return NULL;

        /* Find the FAPI MSMC Heap Handle: */
        ptrFAPI->fapiMSMCHeapHandle = Pktlib_findHeapByName (ptrFAPI->pktlibInstHandle,
                                                             TI_FAPI_CELLA_MSG_HEAP_MSMC_NAME, errCode);
        if (ptrFAPI->fapiMSMCHeapHandle == NULL)
        {
            System_printf ("Error: Unable to find the MSMC heap '%s' [Error code %d]\n", TI_FAPI_CELLA_MSG_HEAP_MSMC_NAME);
            return NULL;
        }

        /* Synchronize for the L1 DDR3 Heap */
        if (Fapi_synch (nameClientHandle, nameDatabaseId, TI_FAPI_CELLA_MSG_HEAP_DDR_NAME, errCode) < 0)
            return NULL;

        /* Find the FAPI DDR3 Heap Handle: */
        ptrFAPI->fapiDDR3HeapHandle = Pktlib_findHeapByName (ptrFAPI->pktlibInstHandle,
                                                             TI_FAPI_CELLA_MSG_HEAP_DDR_NAME, errCode);
        if (ptrFAPI->fapiDDR3HeapHandle == NULL)
        {
            System_printf ("Error: Unable to find the DDR3 heap '%s' [Error code %d]\n", TI_FAPI_CELLA_MSG_HEAP_DDR_NAME);
            return NULL;
        }

        /* Debug Message: */
        System_printf ("-----------------------------------------------------------------\n");
        System_printf ("FAPI Informational Block: \n");
        System_printf ("Debug: FAPI Channel %s -> Channel Handle 0x%p\n",
                        TI_FAPI_CELLA_L1L2_HP_CHANNEL_NAME, ptrFAPI->fapiL1L2HighPriorityChannel);
        System_printf ("Debug: FAPI Channel %s -> Channel Handle 0x%p\n",
                        TI_FAPI_CELLA_L1L2_LP_CHANNEL_NAME, ptrFAPI->fapiL1L2LowPriorityChannel);
        System_printf ("Debug: FAPI Channel %s -> Channel Handle 0x%p\n",
                        TI_FAPI_CELLA_L2L1_HP_CHANNEL_NAME, ptrFAPI->fapiL2L1HighPriorityChannel);
        System_printf ("Debug: FAPI Channel %s -> Channel Handle 0x%p\n",
                        TI_FAPI_CELLA_L2L1_LP_CHANNEL_NAME, ptrFAPI->fapiL2L1LowPriorityChannel);
        System_printf ("Debug: FAPI UL Sch Heap %s -> Heap Handle 0x%p\n",
                        TI_FAPI_CELLA_ULSCH_DATA_HEAP_NAME, ptrFAPI->fapiUlSchDataHeapHandle);
        System_printf ("Debug: FAPI MSMC Heap %s -> Heap Handle 0x%p (Free Queue 0x%x)\n",
                        TI_FAPI_CELLA_MSG_HEAP_MSMC_NAME, ptrFAPI->fapiMSMCHeapHandle,
                        Pktlib_getInternalHeapQueue(ptrFAPI->fapiMSMCHeapHandle));
        System_printf ("Debug: FAPI DDR3 Heap %s -> Heap Handle 0x%p (Free Queue 0x%x)\n",
                        TI_FAPI_CELLA_MSG_HEAP_DDR_NAME, ptrFAPI->fapiDDR3HeapHandle,
                        Pktlib_getInternalHeapQueue(ptrFAPI->fapiDDR3HeapHandle));
        System_printf ("-----------------------------------------------------------------\n");
    }
    else
    {
        Name_ClientHandle   nameClientHandle = Domain_getNameClientInstanceHandle(ptrLTEStackDomain->syslibHandle);
        uint32_t            nameDatabaseId   = Name_getDatabaseInstanceId(Domain_getDatabaseHandle(ptrLTEStackDomain->syslibHandle), errCode);

        /* Synchronize for the L1 High priority channel. */
        if (Fapi_synch (nameClientHandle, nameDatabaseId, TI_FAPI_CELLB_L2L1_HP_CHANNEL_NAME, errCode) < 0)
            return NULL;

        /* Use the MSGCOM API to find the channel. The channel should always be present in the local database. */
        ptrFAPI->fapiL2L1HighPriorityChannel = Msgcom_find (ptrFAPI->msgcomInstHandle,
                                                            TI_FAPI_CELLB_L2L1_HP_CHANNEL_NAME, errCode);
        if (ptrFAPI->fapiL2L1HighPriorityChannel == NULL)
        {
            System_printf ("Error: Unable to find the channel '%s' [Error code %d]\n", TI_FAPI_CELLB_L2L1_HP_CHANNEL_NAME);
            return NULL;
        }

        /* Synchronize for the L1 Low priority channel. */
        if (Fapi_synch (nameClientHandle, nameDatabaseId, TI_FAPI_CELLB_L2L1_LP_CHANNEL_NAME, errCode) < 0)
            return NULL;

        /* Use the MSGCOM API to find the channel. The channel should always be present in the local database. */
        ptrFAPI->fapiL2L1LowPriorityChannel = Msgcom_find (ptrFAPI->msgcomInstHandle,
                                                           TI_FAPI_CELLB_L2L1_LP_CHANNEL_NAME, errCode);
        if (ptrFAPI->fapiL2L1LowPriorityChannel == NULL)
        {
            System_printf ("Error: Unable to find the channel '%s' [Error code %d]\n", TI_FAPI_CELLB_L2L1_LP_CHANNEL_NAME);
            return NULL;
        }

        /* Synchronize for the L1 MSMC Heap */
        if (Fapi_synch (nameClientHandle, nameDatabaseId, TI_FAPI_CELLB_MSG_HEAP_MSMC_NAME, errCode) < 0)
            return NULL;

        /* Find the FAPI MSMC Heap Handle: */
        ptrFAPI->fapiMSMCHeapHandle = Pktlib_findHeapByName (ptrFAPI->pktlibInstHandle,
                                                             TI_FAPI_CELLB_MSG_HEAP_MSMC_NAME, errCode);
        if (ptrFAPI->fapiMSMCHeapHandle == NULL)
        {
            System_printf ("Error: Unable to find the MSMC heap '%s' [Error code %d]\n", TI_FAPI_CELLB_MSG_HEAP_MSMC_NAME);
            return NULL;
        }

        /* Synchronize for the L1 DDR3 Heap */
        if (Fapi_synch (nameClientHandle, nameDatabaseId, TI_FAPI_CELLB_MSG_HEAP_DDR_NAME, errCode) < 0)
            return NULL;

        /* Find the FAPI DDR3 Heap Handle: */
        ptrFAPI->fapiDDR3HeapHandle = Pktlib_findHeapByName (ptrFAPI->pktlibInstHandle,
                                                             TI_FAPI_CELLB_MSG_HEAP_DDR_NAME, errCode);
        if (ptrFAPI->fapiDDR3HeapHandle == NULL)
        {
            System_printf ("Error: Unable to find the DDR3 heap '%s' [Error code %d]\n", TI_FAPI_CELLB_MSG_HEAP_DDR_NAME);
            return NULL;
        }

        /* Debug Message: */
        System_printf ("-----------------------------------------------------------------\n");
        System_printf ("FAPI Informational Block: \n");
        System_printf ("Debug: FAPI Channel %s -> Channel Handle 0x%p\n",
                        TI_FAPI_CELLB_L1L2_HP_CHANNEL_NAME, ptrFAPI->fapiL1L2HighPriorityChannel);
        System_printf ("Debug: FAPI Channel %s -> Channel Handle 0x%p\n",
                        TI_FAPI_CELLB_L1L2_LP_CHANNEL_NAME, ptrFAPI->fapiL1L2LowPriorityChannel);
        System_printf ("Debug: FAPI Channel %s -> Channel Handle 0x%p\n",
                        TI_FAPI_CELLB_L2L1_HP_CHANNEL_NAME, ptrFAPI->fapiL2L1HighPriorityChannel);
        System_printf ("Debug: FAPI Channel %s -> Channel Handle 0x%p\n",
                        TI_FAPI_CELLB_L2L1_LP_CHANNEL_NAME, ptrFAPI->fapiL2L1LowPriorityChannel);
        System_printf ("Debug: FAPI UL Sch Heap %s -> Heap Handle 0x%p\n",
                        TI_FAPI_CELLB_ULSCH_DATA_HEAP_NAME, ptrFAPI->fapiUlSchDataHeapHandle);
        System_printf ("Debug: FAPI MSMC Heap %s -> Heap Handle 0x%p (Free Queue 0x%x)\n",
                        TI_FAPI_CELLB_MSG_HEAP_MSMC_NAME, ptrFAPI->fapiMSMCHeapHandle,
                        Pktlib_getInternalHeapQueue(ptrFAPI->fapiMSMCHeapHandle));
        System_printf ("Debug: FAPI DDR3 Heap %s -> Heap Handle 0x%p (Free Queue 0x%x)\n",
                        TI_FAPI_CELLB_MSG_HEAP_DDR_NAME, ptrFAPI->fapiDDR3HeapHandle,
                        Pktlib_getInternalHeapQueue(ptrFAPI->fapiDDR3HeapHandle));
        System_printf ("-----------------------------------------------------------------\n");
    }

    /* FAPI Tracing services are available only if the NETFP Client has been configured */
    if (Domain_getNetfpClientInstanceHandle(ptrLTEStackDomain->syslibHandle) != NULL)
    {
        Netfp_InboundFPHandle  debugIngressFPHandle;
        Netfp_OutboundFPHandle debugEgressFPHandle;
        char                   fastPathName[NETFP_MAX_CHAR];

        /* Initialize the FAPI Tracing configuration */
        memset ((void *)&fapiTracingCfg, 0, sizeof(Fapi_TracingCfg));

        /* Initialize the heap configuration. */
        memset ((void *)&heapCfg, 0, sizeof(Pktlib_HeapCfg));

        /* Populate the heap configuration */
        strcpy (heapCfg.name, "FAPI Tracing");
        heapCfg.pktlibInstHandle                = ptrFAPI->pktlibInstHandle;
        heapCfg.memRegion                       = fapiResourceConfig.memRegionResponse[0].memRegionHandle;
        heapCfg.sharedHeap                      = 0;
        heapCfg.useStarvationQueue              = 0;
        heapCfg.dataBufferSize                  = sizeof(TiFapi_nonTlvMsg_s);
        heapCfg.numPkts                         = 64;
        heapCfg.numZeroBufferPackets            = 64;
        heapCfg.dataBufferPktThreshold          = 0;
        heapCfg.zeroBufferPktThreshold          = 0;
        heapCfg.arg                             = (uint32_t)SharedRegion_getHeap(1);
        heapCfg.heapInterfaceTable.dataMalloc   = Fapi_DDR3MemoryMalloc;
        heapCfg.heapInterfaceTable.dataFree     = Fapi_DDR3MemoryFree;

        /* Create the FAPI Tracing Heap. */
        ptrLTEStackDomain->fapiTracingHeap = Pktlib_createHeap(&heapCfg, errCode);
        if (ptrLTEStackDomain->fapiTracingHeap == NULL)
        {
    	    System_printf ("Error: Unable to create FAPI Tracing heap [Error code %d]\n", *errCode);
            return NULL;
        }

        /* SYNCH Point: Loop around till the ingress debug fast paths are created. */
        snprintf (fastPathName, NETFP_MAX_CHAR, "Debug-Tracing-Ingress_%c", phyId);
        while (1)
        {
            debugIngressFPHandle = Netfp_findInboundFastPath (Domain_getNetfpClientInstanceHandle(ptrLTEStackDomain->syslibHandle),
                                                              fastPathName, errCode);
            if (debugIngressFPHandle != NULL)
                break;

            /* Fast path has not been created. Wait and try again */
            Task_sleep(1);
        }

        /* SYNCH Point: Loop around till the egress debug fast paths are created. */
        snprintf (fastPathName, NETFP_MAX_CHAR, "Debug-Tracing-Egress_%c", phyId);
        while (1)
        {
            debugEgressFPHandle = Netfp_findOutboundFastPath (Domain_getNetfpClientInstanceHandle(ptrLTEStackDomain->syslibHandle),
                                                              fastPathName, errCode);
            if (debugEgressFPHandle != NULL)
                break;

            /* Fast path has not been created. Wait and try again */
            Task_sleep(1);
        }

        /* Populate FAPI instance configurations */
		memset(&fapiTracingInstCfg, 0 , sizeof(fapiTracingInstCfg));
		fapiTracingInstCfg.malloc               = FapiTracing_osalMalloc;
		fapiTracingInstCfg.free                 = FapiTracing_osalFree;

        /* Create the FAPI Tracing Instance: */
		ptrFAPI->fapiTracingHandle = FapiTracing_createInstance(&fapiTracingInstCfg, errCode);
		if(ptrFAPI->fapiTracingHandle == NULL)
        {
            System_printf ("Error: Create FAPI Tracing instance failed with error: %d\n", *errCode);
            return NULL;
        }

        /* Populate the FAPI Tracing configuration */
        fapiTracingCfg.isEnabled                          = 1;
        fapiTracingCfg.isDataTracingEnabled               = 1;
        fapiTracingCfg.transportCfg.egressFastPathHandle  = debugEgressFPHandle;
        fapiTracingCfg.transportCfg.ingressFastPathHandle = debugIngressFPHandle;
        fapiTracingCfg.transportCfg.sin_family            = Netfp_SockFamily_AF_INET;
        fapiTracingCfg.transportCfg.netfpClientHandle     = Domain_getNetfpClientInstanceHandle(ptrLTEStackDomain->syslibHandle);
        fapiTracingCfg.transportCfg.pktlibInstHandle      = ptrFAPI->pktlibInstHandle;
        fapiTracingCfg.transportCfg.heapHandle            = ptrLTEStackDomain->fapiTracingHeap;
        fapiTracingCfg.transportCfg.srcUDPPort            = (23000 + DNUM);
        fapiTracingCfg.transportCfg.destUDPPort           = (51000 + DNUM);
        fapiTracingCfg.transportCfg.transportType         = TRACING_TRANSPORT_NETFP;

        /* Initialize the FAPI Tracing. */
        if (FapiTracing_init (ptrFAPI->fapiTracingHandle, &fapiTracingCfg, errCode) < 0)
        {
            System_printf ("Error: FAPI Tracing initialization failed with error: %d\n", *errCode);
            return NULL;
        }
    }

    /* Initialize the FAPI Tasks. */
    Task_Params_init(&taskParams);
    taskParams.priority  = 8;
    taskParams.stackSize = 16*1024;
    taskParams.arg0      = (UArg)ptrFAPI;
    ptrFAPI->taskHandle = Task_create(Fapi_receiveTask, &taskParams, NULL);

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
 *  @param[in]  phyIdentifier
 *      PHY identifier.
 *
 *  @retval
 *      Not applicable
 */
void Fapi_deinitComponent (void* fapiHandle, char phyId)
{
    Fapi_MCB*   ptrFAPI;
    int32_t     errCode;
    char*       resourceName;

    /* Get the FAPI MCB */
    ptrFAPI = (Fapi_MCB*)fapiHandle;
    if (ptrFAPI == NULL)
        return;

    /* Disable the FAPI Tracing. */
    FapiTracing_configure (ptrFAPI->fapiTracingHandle, 0, 0, &errCode);

    /* Shutdown the FAPI Task */
    if (ptrFAPI->taskHandle != NULL)
        Task_delete(&ptrFAPI->taskHandle);

    /* Shutdown the Event block */
    if (ptrFAPI->fapiEventHandle != NULL)
        Event_delete(&ptrFAPI->fapiEventHandle);

    /* Shutdown the writer channels:
     *  - High & Low Priority writer channels which sends messages to the L1  */
    if (ptrFAPI->fapiL1L2HighPriorityChannel != NULL)
    {
        /* Delete the MSGCOM channel from the local database */
        if (Msgcom_delete (ptrFAPI->fapiL1L2HighPriorityChannel, myFreeMsgBuffer) < 0)
        {
            System_printf ("Error: Unable to delete the high priority writer channel '%p'\n", ptrFAPI->fapiL1L2HighPriorityChannel);
            return;
        }

        /* Get the resource name which is being deleted and also remove this from the RAT database */
        resourceName = (phyId == 'A') ? TI_FAPI_CELLA_L1L2_HP_CHANNEL_NAME : TI_FAPI_CELLB_L1L2_HP_CHANNEL_NAME;

        /* Delete the channel name from the RAT database also. */
        if (Name_push (Domain_getNameClientInstanceHandle(ptrFAPI->ptrLTEStackDomain->syslibHandle), resourceName,
                       Name_ResourceBucket_INTERNAL_SYSLIB, Name_ResourceOperationType_DELETE, &errCode) < 0)
        {
            System_printf ("Error: Unable to push the FAPI channel name between realms [Error code %d]\n", errCode);
            return;
        }
        System_printf ("Debug: FAPI High Priority FAPI channel '%s' deleted from the RAT database\n", resourceName);
    }
    if (ptrFAPI->fapiL1L2LowPriorityChannel != NULL)
    {
        /* Delete the MSGCOM channel from the local database. */
        if (Msgcom_delete (ptrFAPI->fapiL1L2LowPriorityChannel, myFreeMsgBuffer) < 0)
        {
            System_printf  ("Error: Unable to delete the low priority writer channel '%p'\n", ptrFAPI->fapiL1L2LowPriorityChannel);
            return;
        }

        /* Get the resource name which is being deleted and also remove this from the RAT database */
        resourceName = (phyId == 'A') ? TI_FAPI_CELLA_L1L2_LP_CHANNEL_NAME : TI_FAPI_CELLB_L1L2_LP_CHANNEL_NAME;

        /* Delete the channel name from the RAT database also. */
        if (Name_push (Domain_getNameClientInstanceHandle(ptrFAPI->ptrLTEStackDomain->syslibHandle), resourceName,
                       Name_ResourceBucket_INTERNAL_SYSLIB, Name_ResourceOperationType_DELETE, &errCode) < 0)
        {
            System_printf ("Error: Unable to push the FAPI channel name between realms [Error code %d]\n", errCode);
            return;
        }
        System_printf ("Debug: FAPI Low Priority FAPI channel '%s' deleted from the RAT database\n", resourceName);
    }

    /* Shutdown the reader channels:
     *  - High & Low Priority Reader channel which receives messages from L1 */
    if (ptrFAPI->fapiL2L1HighPriorityChannel != NULL)
    {
        if (Msgcom_delete (ptrFAPI->fapiL2L1HighPriorityChannel, myFreeMsgBuffer) < 0)
        {
            System_printf ("Error: Unable to delete the high priority reader channel '%p'\n", ptrFAPI->fapiL1L2HighPriorityChannel);
            return;
        }
    }
    if (ptrFAPI->fapiL2L1LowPriorityChannel != NULL)
    {
        if (Msgcom_delete (ptrFAPI->fapiL2L1LowPriorityChannel, myFreeMsgBuffer) < 0)
        {
            System_printf  ("Error: Unable to delete the low priority reader channel '%p'\n", ptrFAPI->fapiL1L2LowPriorityChannel);
            return;
        }
    }

    /* Shutdown the ULSCH Heap */
    if (ptrFAPI->fapiUlSchDataHeapHandle != NULL)
    {
        /* Debug Message: */
        System_printf ("Debug: Deleting the ULSCH heap\n");
        if (Pktlib_deleteHeap (ptrFAPI->pktlibInstHandle, ptrFAPI->fapiUlSchDataHeapHandle, &errCode) < 0)
        {
            System_printf  ("Error: Unable to delete the FAPI ULSCH Heap [Error code %d]\n", errCode);
            return;
        }

        /* Get the resource name: */
        resourceName =  (phyId == 'A') ? TI_FAPI_CELLA_ULSCH_DATA_HEAP_NAME: TI_FAPI_CELLB_ULSCH_DATA_HEAP_NAME;

        /* Delete the ULSCH heap name from the RAT database */
        if (Name_push (Domain_getNameClientInstanceHandle(ptrFAPI->ptrLTEStackDomain->syslibHandle), resourceName,
                       Name_ResourceBucket_INTERNAL_SYSLIB, Name_ResourceOperationType_DELETE, &errCode) < 0)
        {
            System_printf ("Error: Unable to push the ULSCH heap name '%s' between realms [Error code %d]\n", resourceName, errCode);
            return;
        }
        System_printf ("Debug: ULSCH Heap name '%s' deleted from the remote database\n", resourceName);
    }

    /* Shutdown the FAPI Tracing Heap */
    if (ptrFAPI->ptrLTEStackDomain->fapiTracingHeap != NULL)
    {
        /* Debug Message: */
        System_printf ("Debug: Deinitializing FAPI Tracing\n");

        /* Deinitialize the FAPI Tracing module. */
        if(FapiTracing_deinit(ptrFAPI->fapiTracingHandle, &errCode)< 0)
        {
            System_printf  ("Error: Unable to delete the FAPI Tracing [Error code %d]\n", errCode);
            return;
        }

        /* Delete FapiTracing instance */
        if (FapiTracing_deleteInstance(ptrFAPI->fapiTracingHandle, &errCode) < 0)
        {
            System_printf  ("Error: Unable to delete the FAPI Tracing Instance [Error code %d]\n", errCode);
            return;
        }

        /* Debug Message: */
        System_printf ("Debug: Deleting the FAPI Tracing Heap\n");

        /* Shutdown the FAPI Tracing heap. */
        if (Pktlib_deleteHeap (ptrFAPI->pktlibInstHandle,
                               ptrFAPI->ptrLTEStackDomain->fapiTracingHeap, &errCode) < 0)
        {
            System_printf  ("Error: Unable to delete the FAPI Tracing Heap [Error code %d]\n", errCode);
            return;
        }
    }

    /* Cleanup memory allocated to the FAPI MCB */
    Memory_free ((xdc_runtime_IHeap_Handle)ptrFAPI->ptrLTEStackDomain->ddr3PrivateHeapHandle, ptrFAPI, sizeof(Fapi_MCB));
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

#if 0
/**
 *  @b Description
 *  @n
 *      The function is used to get the status of the FAPI.
 *
 *  @param[out]  sfn
 *      Subframe number populated by the API
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
void Fapi_getStatus(uint16_t* sfn)
{
    *sfn = ptrFAPI->sfnSf;
}

#endif


