/******************************************************************************
*
*  ti_fapi.h
*
*  (C) Copyright 2015, Texas Instruments, Inc.
*
*
******************************************************************************
*
*  File Description : This file contains the definitions based
*                     on the Small Cell Forum 
*                     LTE eNB L1 API Definition document
*                     FF_WG2_(10)_xxx_v2_1  (document date issued: 07-05-2013)
*                      
******************************************************************************/

/******************************************************************************
*ti_fapi.h change log:
* 2.0 - Initial FAPI 2.0 version based on FF_WG2_(10)_xxx_v2_1 
* 2.1 - Added dciLength field in TiFapi_dlDciPduCfg_s and TiFapi_dci0PduCfg_s
*     - Changed value of TI_FAPI_MAX_NUM_SRS_PDUS 
*     - Added TiFapi_harqAckNackMode_t
*     - Added controlType field to TiFapi_pmiRiPeriodic_s
*     - Removed pucchIndex field from TiFapi_uciHarqCfg_s
*     - Changed size of cqiPayload in TiFapi_cqiPduCfg_s
* 2.2 - Updated TiFapi_pmiRiAperiodic_s dlCqiPmiSizeRank field to report a
*       value per CC.
* 2.3 - Created union structures for RX_ULSCH.indication and TX.request 
*       messages to support reference and embedded FAPI transfer modes.
*     - Added support for 256 QAM in TiFapi_modulationType_t
*
******************************************************************************/

#ifndef TI_FAPI_H_
#define TI_FAPI_H_

#include <stdint.h>

#define TI_FAPI_H_MAJOR_VERSION 2
#define TI_FAPI_H_MINOR_VERSION 3

/******************************************************************************
*
*                PHY <-> L2 MsgCom channels                                     
*
******************************************************************************/
/* Low priority MsgCom channel name for L1 -> L2 */
#define TI_FAPI_L1L2_LP_CHANNEL_NAME            "TiFapi_L1L2LP"

/* High priority MsgCom channel name for L1 -> L2 */
#define TI_FAPI_L1L2_HP_CHANNEL_NAME            "TiFapi_L1L2HP"

/* MsgCom channel name for transmitting MAC PDUs from L1 -> L2 
   Used only if L2 resides on ARM */
#define TI_FAPI_L1L2_MAC_PDU_CHANNEL_NAME       "TiFapi_L1L2MACPDU"

/* Low priority MsgCom channel name for L2 -> L1*/
#define TI_FAPI_L2L1_LP_CHANNEL_NAME            "TiFapi_L2L1LP"

/* High priority MsgCom channel name for L2 -> L1*/
#define TI_FAPI_L2L1_HP_CHANNEL_NAME            "TiFapi_L2L1HP"

/* MsgCom channel name for transmitting MAC PDUs from L2 -> L1 
   Used only if L2 resides on ARM */
#define TI_FAPI_L2L1_MAC_PDU_CHANNEL_NAME       "TiFapi_L2L1MACPDU"

/******************************************************************************
*
*                PktLib heaps                                     
*
******************************************************************************/
/* FAPI message heap names*/
/* Message heap in DDR */
#define TI_FAPI_MSG_HEAP_DDR_NAME         "TiFapi_MsgHeapDdr"
/* Message heap in MSMC */
#define TI_FAPI_MSG_HEAP_MSMC_NAME        "TiFapi_MsgHeapMsmc"

/* PktLib ULSCH payload heap name*/
#define TI_FAPI_ULSCH_DATA_HEAP_NAME      "TiFapi_UlschDataHeap"

/******************************************************************************
*
*                PHY Configuration parameters                                     
*
******************************************************************************/
/****************************************
Implementation-specific restrictions
****************************************/


#define TI_FAPI_MAX_NUM_DL_USERS                               ( 8 )

#define TI_FAPI_MAX_NUM_UL_USERS                               ( 8 )

/* The maximum number of DL UEs*/
#define TI_FAPI_MAX_NUM_DL_UES                                 ( TI_FAPI_MAX_NUM_DL_USERS )

/* The maximum number of UL PDUs received */
#define TI_FAPI_MAX_NUM_ULSCH_PDUS                             ( TI_FAPI_MAX_NUM_UL_USERS )

/* The maximum number of UL HARQ PDUs received from L23*/
#define TI_FAPI_MAX_NUM_HARQ_PDUS                              ( 4*TI_FAPI_MAX_NUM_DL_UES )

/* The maximum number of DL DCI PDUs*/
#define TI_FAPI_MAX_NUM_DL_DCI_PDUS                            ( TI_FAPI_MAX_NUM_DL_UES + 3 ) /* + 3 for SIB/PCH/RAR */

/* The maximum number of DLSCH PDUs*/
#define TI_FAPI_MAX_NUM_DLSCH_PDUS                             ( TI_FAPI_MAX_NUM_DL_UES * 2 + 3) /* times 2 for MIMO + 3 for PCH/SI/RAR */

/* The maximum number of DL TX REQUEST PDUs*/
/* Maximum number of MAC PDUs that can be transmitted in one TX.req message*/
#define TI_FAPI_MAX_NUM_TX_REQUEST_PDUS                        ( TI_FAPI_MAX_NUM_DLSCH_PDUS + 1 ) /* +1 for BCH */

/* The maximum number of DL PHICH PDUs*/
#define TI_FAPI_MAX_NUM_HI_PDUS                                ( 2 * TI_FAPI_MAX_NUM_ULSCH_PDUS ) /* 2X to account for TDD UL/DL config 0 */

/* The maximum number of UL DCI PDUs*/
#define TI_FAPI_MAX_NUM_UL_DCI_PDUS                            ( TI_FAPI_MAX_NUM_ULSCH_PDUS + 2 ) /* + 2 for DCI formats 3 or 3A*/

/* The maximum number of SRS PDUs*/
#define TI_FAPI_MAX_NUM_SRS_PDUS                               ( 32 )

/* The maximum number of CQI PDUs*/
#define TI_FAPI_MAX_NUM_CQI_PDUS                               ( 32 )

/* The maximum number of SR PDUs*/
#define TI_FAPI_MAX_NUM_SR_PDUS                                ( 32 )

/* The maximum number of UCI PDUs*/
#define TI_FAPI_MAX_NUM_UCI_PDUS                               ( TI_FAPI_MAX_NUM_CQI_PDUS + TI_FAPI_MAX_NUM_SR_PDUS + TI_FAPI_MAX_NUM_HARQ_PDUS) 

/* The maximum number of CRC PDUs*/
#define TI_FAPI_MAX_NUM_CRC_PDUS                               ( TI_FAPI_MAX_NUM_ULSCH_PDUS )

/* The maximum number of RACH preambles per sub-frame*/
#define TI_FAPI_MAX_NUM_RACH_PREAMBLES                         ( 64 )

/* The maximum number of antennas per beam-forming vector*/
#define TI_FAPI_BF_MAX_NUM_ANTENNAS                            (  2 )

/* The maximum number of layers per beam-forming vector*/
#define TI_FAPI_BF_MAX_NUM_LAYERS                              (  2 )

/* The maximum number of RBs in a beam-forming vectors*/
#define TI_FAPI_BF_MAX_NUM_RBS                                 ( 100 )

/* The maximum number of beam-forming vector per DLSCH PDU */
#define TI_FAPI_BF_MAX_NUM_VEC_PER_DLSCH_PDU                   ( 5 )

/* Maximum CQI payload size */
#define TI_FAPI_MAX_CQI_PAYLOAD                                (8)

/* Maximum number of scattered buffers that can be used to build one TX MAC PDU */
#define TI_FAPI_MAX_NUM_TX_REQ_SCATTERED_BUF                   (16)

/* Maximum number of MAC PDUs that can be received in one RX_ULSCH.ind message*/
#define TI_FAPI_MAX_NUM_RX_ULSCH_IND_PDUS                      TI_FAPI_MAX_NUM_ULSCH_PDUS


/****************************************
Maximum values defined by the LTE spec
****************************************/
/* The maximum number of SRS RBs reported for one UE*/
#define TI_FAPI_MAX_NUM_SRS_RBS_REPORTED_PER_UE                ( 96 )

/* The maximum number of sub-bands for the code-book index array*/
#define TI_FAPI_DLSCH_PDU_CFG_MAX_SUBBANDS                     ( 13 )

/* The maximum number of PCH PDUs*/
#define TI_FAPI_MAX_NUM_PCH_PDUS                               ( 1 )

/* The maximum number of MCH PDUs*/
#define TI_FAPI_MAX_NUM_MCH_PDUS                               ( 1 )

/* The maximum number of CSI RS PDUs*/
#define TI_FAPI_MAX_NUM_CSIRS_PDUS                             ( TI_FAPI_MAX_NUM_DL_UES )

/******************************************************************************
*
*                FAPI messages                                     
*
******************************************************************************/

/****************************************
Message ID type
****************************************/
typedef uint8_t TiFapi_msgIdType_t;

/****************************************
Possible message types
****************************************/
#define TI_FAPI_PARAM_REQUEST               0x00
#define TI_FAPI_PARAM_RESPONSE              0x01
#define TI_FAPI_CELL_CONFIG_REQUEST         0x02
#define TI_FAPI_CELL_CONFIG_RESPONSE        0x03
#define TI_FAPI_START_REQUEST               0x04
#define TI_FAPI_STOP_REQUEST                0x05
#define TI_FAPI_STOP_INDICATION             0x06
#define TI_FAPI_UE_CONFIG_REQUEST           0x07
#define TI_FAPI_UE_CONFIG_RESPONSE          0x08
#define TI_FAPI_ERROR_INDICATION            0x09
#define TI_FAPI_UE_RELEASE_REQUEST          0x0a
#define TI_FAPI_UE_RELEASE_RESPONSE         0x0b

#define TI_FAPI_DL_CONFIG_REQUEST           0x80
#define TI_FAPI_UL_CONFIG_REQUEST           0x81
#define TI_FAPI_UL_SUBFRAME_INDICATION      0x82
#define TI_FAPI_DL_HI_DCI0_REQUEST          0x83
#define TI_FAPI_DL_TX_REQUEST               0x84
#define TI_FAPI_UL_HARQ_INDICATION          0x85
#define TI_FAPI_UL_CRC_INDICATION           0x86
#define TI_FAPI_UL_RX_ULSCH_INDICATION      0x87
#define TI_FAPI_UL_RACH_INDICATION          0x88
#define TI_FAPI_UL_SRS_INDICATION           0x89
#define TI_FAPI_UL_RX_SR_INDICATION         0x8a
#define TI_FAPI_UL_RX_CQI_INDICATION        0x8b

/* This macro is used for declaring array of variable length */
#define TI_FAPI_VAR_SIZE(x) 1

/****************************************
L1 Error Indication 
****************************************/
/* Possible error codes */
typedef uint8_t TiFapi_l1ErrorCodes_t; 
   #define TI_FAPI_MSG_OK              0
   #define TI_FAPI_MSG_INVALID_STATE   1
   #define TI_FAPI_MSG_INVALID_CONFIG  2
   #define TI_FAPI_SFN_OUT_OF_SYNC     3
   #define TI_FAPI_MSG_SUBFRAME_ERR    4
   #define TI_FAPI_MSG_BCH_MISSING     5
   #define TI_FAPI_MSG_INVALID_SFN     6
   #define TI_FAPI_MSG_HI_ERR          7
   #define TI_FAPI_MSG_TX_ERR          8
   #define TI_FAPI_MSG_TX_MISSING_ERR  9

/****************************************
Possible TLV tags for PARAM and CONFIG 
messages (table 18) 
****************************************/
typedef uint8_t TiFapi_cellConfig_t; 
   #define TI_FAPI_DUPLEXING_MODE                              1
   #define TI_FAPI_PCFICH_POWER_OFFSET                         2
   #define TI_FAPI_P_B                                         3
   #define TI_FAPI_DL_CYCLIC_PREFIX_TYPE                       4
   #define TI_FAPI_UL_CYCLIC_PREFIX_TYPE                       5
   
/* RF CONFIG TAGS */
   #define TI_FAPI_DL_CHANNEL_BANDWIDTH                        10
   #define TI_FAPI_UL_CHANNEL_BANDWIDTH                        11
   #define TI_FAPI_REFERENCE_SIGNAL_POWER                      12
   #define TI_FAPI_TX_ANTENNA_PORTS                            13
   #define TI_FAPI_RX_ANTENNA_PORTS                            14
/* RF CONFIG TAGS ENDS */

/* PHICH CONFIG */
   #define TI_FAPI_PHICH_RESOURCE                              20
   #define TI_FAPI_PHICH_DURATION                              21
   #define TI_FAPI_PHICH_POWER_OFFSET                          22
/* PHICH CONFIG ENDS */

/* SCH CONFIG */
   #define TI_FAPI_PRIMARY_SYNC_SIGNAL                         30
   #define TI_FAPI_SECONDARY_SYNC_SIGNAL                       31
   #define TI_FAPI_PHYSICAL_CELL_ID                            32
/* SCH CONFIG  ENDS */

/* PRACH CONFIG */
   #define TI_FAPI_CONFIGURATION_INDEX                         40
   #define TI_FAPI_ROOT_SEQUENCE_INDEX                         41
   #define TI_FAPI_ZERO_CORRELATION_ZONE_CONFIGURATION         42
   #define TI_FAPI_HIGH_SPEED_FLAG                             43
   #define TI_FAPI_FREQUENCY_OFFSET                            44 
/* PRACH CONFIG ENDS */

/* PUSCH CONFIG */
   #define TI_FAPI_HOPPING_MODE                                50
   #define TI_FAPI_HOPPIG_OFFSET                               51
   #define TI_FAPI_NUM_OF_SUB_BANDS                            52
/* PUSCH CONFIG  ENDS */

/* PUCCH CONFIG */
   #define TI_FAPI_DELTA_PUCCH_SHIFT                           60
   #define TI_FAPI_N_CQI_RB                                    61
   #define TI_FAPI_N_AN_CS                                     62
   #define TI_FAPI_N_1_PUCCH_AN                                63
/* PUCCH CONFIG ENDS */

/* SRS CONFIG */
   #define TI_FAPI_BANDWIDTH_CONFIGURATION                     70
   #define TI_FAPI_MAX_UP_PTS                                  71
   #define TI_FAPI_SRS_SUB_FRAME_CONFIGURATION                 72 
   #define TI_FAPI_SRS_ACK_NACK_SRS_SIMULTANEOUS_TRANSMISSION  73
/* SRS CONFIG ENDS */

/* UPLINK REFERENCE SIGNAL CONFIG */
   #define TI_FAPI_UPLINK_RS_HOPPING                           80
   #define TI_FAPI_GROUP_ASSIGNMENT                            81
   #define TI_FAPI_CYCLIC_SHIFT_1_FOR_DMRS                     82
/* UPLINK REFERENCE SIGNAL CONFIG ENDS */

/* TDD FRAME STRUCTURE CONFIG */
   #define TI_FAPI_SUB_FRAME_ASSIGNMENT                        90
   #define TI_FAPI_SPECIAL_SUB_FRAME_PATTERNS                  91
/* TDD FRAME STRUCTURE CONFIG ENDS */

/* MBSFN CONFIG */
   #define TI_FAPI_MBSFN_AREA_ID                               100
/* MBSFN CONFIG ENDS*/

/* PRS CONFIG */
   #define TI_FAPI_PRS_BANDWIDTH                               110
   #define TI_FAPI_PRS_CYCLIC_PREFIX_TYPE                      111
/* PRS CONFIG ENDS*/

/******************* START TI PROPRIETARY TAGS **********************/
/* PHY configuration extension parameters */
   #define TI_FAPI_TEST_MODE_SUPPORT                           140
   #define TI_FAPI_DL_SAMPLING_FREQ_SUPPORT                    141 
   #define TI_FAPI_UL_SAMPLING_FREQ_SUPPORT                    142
   
   #define TI_FAPI_TEST_MODE                                   143
   #define TI_FAPI_DL_SAMPLING_FREQ                            144
   #define TI_FAPI_UL_SAMPLING_FREQ                            145
   #define TI_FAPI_AIF2_TRIGGER_TYPE                           146
   #define TI_FAPI_AIF2_PLL_MULT_FACTOR                        147
   #define TI_FAPI_AIF2_SYNC_TIMEOUT                           148 
   #define TI_FAPI_AIF2_LINK_CONFIG                            149
   #define TI_FAPI_AIF2_ANT_CONFIG                             150
   
   /*gain between the rx antenna ports and the PHY SW interface */
   #define TI_FAPI_GHW                                         151
   /*Number of PRACH preambles to be searched*/ 
   #define TI_FAPI_NUM_OF_RACH_PREAMBLES                       152
   /*Maximum number of DL users supported by the application*/
   #define TI_FAPI_SUPPORTED_NUM_DL_USERS                      153
   /*Maximum number of UL users supported by the application*/
   #define TI_FAPI_SUPPORTED_NUM_UL_USERS                      154
   
   /*Reserved parameters*/  
   #define TI_FAPI_CELL_CFG_RESERVED_PARAM_00                  160
   #define TI_FAPI_CELL_CFG_RESERVED_PARAM_01                  161
   #define TI_FAPI_CELL_CFG_RESERVED_PARAM_02                  162
   #define TI_FAPI_CELL_CFG_RESERVED_PARAM_03                  163
   #define TI_FAPI_CELL_CFG_RESERVED_PARAM_04                  164
   #define TI_FAPI_CELL_CFG_RESERVED_PARAM_05                  165
   #define TI_FAPI_CELL_CFG_RESERVED_PARAM_06                  166
   #define TI_FAPI_CELL_CFG_RESERVED_PARAM_07                  167
   #define TI_FAPI_CELL_CFG_RESERVED_PARAM_08                  168
   #define TI_FAPI_CELL_CFG_RESERVED_PARAM_09                  169
   #define TI_FAPI_CELL_CFG_RESERVED_PARAM_10                  170
   #define TI_FAPI_CELL_CFG_RESERVED_PARAM_11                  171
   #define TI_FAPI_CELL_CFG_RESERVED_PARAM_12                  172
   #define TI_FAPI_CELL_CFG_RESERVED_PARAM_13                  173
   #define TI_FAPI_CELL_CFG_RESERVED_PARAM_14                  174
   #define TI_FAPI_CELL_CFG_RESERVED_PARAM_15                  175
/******************* END OF TI PROPRIETARY TAGS **********************/

/* tags used by L1 to reports its physical capabilities to L2/L3 software */
   #define TI_FAPI_DL_BANDWIDTH_SUPPORT                        200
   #define TI_FAPI_UL_BANDWIDTH_SUPPORT                        201
   #define TI_FAPI_DL_MODULATION_SUPPORT                       202 
   #define TI_FAPI_UL_MODULATION_SUPPORT                       203
   #define TI_FAPI_PHY_ANTENNA_CAPABILITY                      204

/* Indicates which release the PHY supports */
   #define TI_FAPI_RELEASE_CAPABILITY                          205
   
/* Indicates MBSFN capability */
   #define TI_FAPI_MBSFN_CAPABILITY                            206

/* Tags used by L2/L3 software to configure the interaction between L2/L3 and L1 */
   #define TI_FAPI_DATA_REPORT_MODE                            240
   #define TI_FAPI_SFN_SF                                      241 

/* Tag used by L1 to report its current status */
   #define TI_FAPI_PHY_STATE                                   250

/* TI FAPI header file version*/
   #define TI_FAPI_HEADER_VERSION                              251


/****************************************
Possible values for the TLVs (table 18)
****************************************/
/* PHY states */
typedef uint16_t TiFapi_phyStates_t; 
   #define TI_FAPI_IDLE        0
   #define TI_FAPI_CONFIGURED  1
   #define TI_FAPI_RUNNING     2

/* Types of Duplexing Mode */
typedef uint16_t TiFapi_duplexingMode_t; 
   #define TI_FAPI_TDD     0
   #define TI_FAPI_FDD     1 
   #define TI_FAPI_HD_FDD  2

/* Types of Cylic Prefix */
typedef uint16_t TiFapi_cyclicPrefix_t; 
   #define TI_FAPI_CP_NORMAL    0
   #define TI_FAPI_CP_EXTENDED  1

/* UL/DL Channel Bandwidth */
typedef uint16_t TiFapi_uldlChannelBw_t; 
   #define TI_FAPI_CHANNEL_BW_6RB     6
   #define TI_FAPI_CHANNEL_BW_15RB    15
   #define TI_FAPI_CHANNEL_BW_25RB    25
   #define TI_FAPI_CHANNEL_BW_50RB    50
   #define TI_FAPI_CHANNEL_BW_75RB    75
   #define TI_FAPI_CHANNEL_BW_100RB   100

/* No. of Tx Antenna Ports */
typedef uint16_t TiFapi_txAntennaPort_t; 
   #define TI_FAPI_TX_ANTENNA_PORT_1  1
   #define TI_FAPI_TX_ANTENNA_PORT_2  2
   #define TI_FAPI_TX_ANTENNA_PORT_4  4

/* No. of Rx Antenna Ports */
typedef uint16_t TiFapi_rxAntennaPort_t; 
   #define TI_FAPI_RX_ANTENNA_PORT_1  1
   #define TI_FAPI_RX_ANTENNA_PORT_2  2
   #define TI_FAPI_RX_ANTENNA_PORT_4  4

/* POSSIBLE PHICH RESOURCE VALUES */
typedef uint16_t TiFapi_phichResourceValues_t; 
   #define TI_FAPI_PHICH_R_ONE_SIXTH  0
   #define TI_FAPI_PHICH_R_HALF       1
   #define TI_FAPI_PHICH_R_ONE        2
   #define TI_FAPI_PHICH_R_TWO        3

/* Phich Duration */
typedef uint16_t TiFapi_phichDuration_t; 
   #define TI_FAPI_PHICH_D_NORMAL     0
   #define TI_FAPI_PHICH_D_EXTENDED   1

/* High Speed Flag */
typedef uint16_t TiFapi_highSpeedFlag_t; 
   #define TI_FAPI_HS_UNRESTRICTED_SET 0
   #define TI_FAPI_HS_RESTRICTED_SET   1

/* Hopping Mode */
typedef uint16_t TiFapi_hoppingMode_t; 
   #define TI_FAPI_HM_INTER_SF        0
   #define TI_FAPI_HM_INTRA_INTER_SF  1

/* Types of hopping */
typedef uint16_t TiFapi_hoppingType_t; 
   #define TI_FAPI_RS_NO_HOPPING        0
   #define TI_FAPI_RS_GROUP_HOPPING     1
   #define TI_FAPI_RS_SEQUENCE_HOPPING  2

/* MaxUpPTS (TDD only) */
typedef uint16_t TiFapi_maxUpPts_t; 
   #define TI_FAPI_MAX_UP_PTS_DISABLED   0
   #define TI_FAPI_MAX_UP_PTS_ENABLED    1
   
/* Physical Antenna Capabililty */
typedef uint16_t TiFapi_phyAntennaCapability_t; 
   #define TI_FAPI_PHY_ANTENNA_CAP_1  1
   #define TI_FAPI_PHY_ANTENNA_CAP_2  2
   #define TI_FAPI_PHY_ANTENNA_CAP_4  4
   #define TI_FAPI_PHY_ANTENNA_CAP_8  8

/* PRS Transmission Bandwidth */
typedef uint16_t TiFapi_prsBw_t; 
   #define TI_FAPI_PRS_BW_6RB    6
   #define TI_FAPI_PRS_BW_15RB   15
   #define TI_FAPI_PRS_BW_25RB   25
   #define TI_FAPI_PRS_BW_50RB   50
   #define TI_FAPI_PRS_BW_75RB   75
   #define TI_FAPI_PRS_BW_100RB  100

/*The cyclic prefix used for PRS transmission.*/
typedef uint16_t TiFapi_prsCyclicPrefix_t; 
   #define TI_FAPI_PRS_CP_NORMAL    0
   #define TI_FAPI_PRS_CP_EXTENDED  1

/****************************************
Other pre-defined values
****************************************/
/* Types of Resource Allocation */
typedef uint8_t TiFapi_resAllcType_t; 
   #define TI_FAPI_RES_ALLOC_TYPE_0         0
   #define TI_FAPI_RES_ALLOC_TYPE_1         1
   #define TI_FAPI_RES_ALLOC_TYPE_2         2
   #define TI_FAPI_RES_ALLOC_TYPE_2_DCI_1C  3 

/* Resource Allocation Flag*/
typedef uint8_t TiFapi_resAllcFlag_t; 
   #define TI_FAPI_RES_ALLOC_TYPE_FIELD_NOT_VALID     0
   #define TI_FAPI_RES_ALLOC_TYPE_FIELD_VALID         1
   
/* vRB Assignment Flag */
typedef uint8_t TiFapi_vRBAssignmentFlag_t; 
    #define TI_FAPI_LOCALISED   0
    #define TI_FAPI_DISTRIBUTED 1

/* tb To CodeWordSwap Flag */
typedef uint8_t TiFapi_tbToCodeWordSwapFlag_t; 
    #define TI_FAPI_NOSWAPPING 0
    #define TI_FAPI_SWAPPED    1

/* types of TPC */
typedef uint8_t TiFapi_tpcValue_t; 
    #define TI_FAPI_TX_POWER_CONTROL_0         0
    #define TI_FAPI_TX_POWER_CONTROL_1         1
    #define TI_FAPI_TX_POWER_CONTROL_2         2
    #define TI_FAPI_TX_POWER_CONTROL_3         3

/* Types of Transmission Scheme */
typedef uint8_t TiFapi_transScheme_t; 
    #define TI_FAPI_SINGLE_ANTENNA_PORT_0             0
    #define TI_FAPI_TX_DIVERSITY                      1
    #define TI_FAPI_LARGE_DELAY_CDD                   2
    #define TI_FAPI_CLOSED_LOOP_SPATIAL_MULTIPLEXING  3
    #define TI_FAPI_MULTI_USER_MIMO                   4 
    #define TI_FAPI_CLOSED_LOOP_RANK_1_PRECODING      5
    #define TI_FAPI_SINGLE_ANTENNA_PORT_5             6
    #define TI_FAPI_SINGLE_ANTENNA_PORT_7             7
    #define TI_FAPI_SINGLE_ANTENNA_PORT_8             8
    #define TI_FAPI_DUAL_LAYER_ANTENNA_PORT_7_8       9

/* Types of Transmission Scheme for ULSCH PDU*/
typedef uint8_t TiFapi_transSchemeUlsch_t; 
    #define TI_FAPI_ULSCH_SINGLE_ANTENNA_PORT_10            0
    #define TI_FAPI_ULSCH_CLOSED_LOOP_SPATIAL_MULTIPLEXING  1

/* Types of RNTI */
typedef uint8_t TiFapi_rntiType_t; 
    #define TI_FAPI_C_RNTI                   1
    #define TI_FAPI_RA_RNTI_P_RNTI_SI_RNTI   2
    #define TI_FAPI_SPS_CRNTI                3 

/* P-A values */
typedef uint8_t TiFapi_paValue_t; 
    #define TI_FAPI_DB_MINUS6         0
    #define TI_FAPI_DB_MINUS_4DOT77   1
    #define TI_FAPI_DB_MINUS_3        2
    #define TI_FAPI_DB_MINUS_1DOT77   3
    #define TI_FAPI_DB0               4
    #define TI_FAPI_DB1               5 
    #define TI_FAPI_DB2               6
    #define TI_FAPI_DB3               7

/* Types of modulation*/
typedef uint8_t TiFapi_modulationType_t;
    #define TI_FAPI_QPSK   2
    #define TI_FAPI_16QAM  4
    #define TI_FAPI_64QAM  6
    #define TI_FAPI_256QAM 8

/* ul Tx Mode type */
typedef uint8_t  TiFapi_ulTxMode_t;
    #define TI_FAPI_ULTX_SISO_SIMO 0
    #define TI_FAPI_ULTX_MIMO      1 

/* Types of HI Values */
typedef uint8_t  TiFapi_hiValue_t;
    #define TI_FAPI_HI_NACK 0
    #define TI_FAPI_HI_ACK  1 
	
/* Flag that indicates if HI is present for a second transport block*/
typedef uint8_t  TiFapi_hiFlagTb2_t;
    #define TI_FAPI_HI_TB2_NOT_PRESENT 0
    #define TI_FAPI_HI_TB2_PRESENT     1 
	
/* Types of CQI Request */
typedef uint8_t  TiFapi_cqiRequest_t;
    #define TI_FAPI_APERIODIC_CQI_NOT_REQUESTED                         0
    #define TI_FAPI_APERIODIC_CQI_REQUESTED                             1
    #define TI_FAPI_APERIODIC_CQI_REQUESTED_FIRST_SET_SERVING_CELLS     2
    #define TI_FAPI_APERIODIC_CQI_REQUESTED_SECOND_SET_SERVING_CELLS    3

/* ue Tx Antenna Selection */
typedef uint8_t  TiFapi_ueTxAntennaSelection_t;
    #define TI_FAPI_ANT_PORT_NOT_CONFIGURED  0
    #define TI_FAPI_CONF_UE_PORT_0           1
    #define TI_FAPI_CONF_UE_PORT_1           2

/* CRC FLAGS */
typedef uint8_t  TiFapi_crcFlags_t;
    #define TI_FAPI_CRC_CORRECT  0
    #define TI_FAPI_CRC_ERROR    1 

/* DCI format */
typedef uint8_t TiFapi_ulDciFormat_t;
    #define TI_FAPI_UCI_DCI_FORMAT_0   0        /* DCI format 0. */
    #define TI_FAPI_UCI_DCI_FORMAT_3   1        /* DCI format 3.*/
    #define TI_FAPI_UCI_DCI_FORMAT_3A  2        /* DCI format 3A.*/
    #define TI_FAPI_UCI_DCI_FORMAT_4   3        /* DCI format 4.*/

/* Frequency enabled flag, indicates if hopping is being used */
typedef uint8_t  TiFapi_frequencyHoppingFlag_t;
    #define TI_FAPI_FREQUENCY_HOPPING_FLAG_NO_HOPPING        0            /* Frequency hopping disabled.*/
    #define TI_FAPI_FREQUENCY_HOPPING_FLAG_HOPPING_ENABLED   1            /* Frequency hopping enabled.*/

/* Disable Sequece Hopping  flag, indicates if any configured group hopping should be disabled for the UE */
typedef uint8_t  TiFapi_disSequenceHoppingFlag_t;
    #define TI_FAPI_DIS_SEQUENCE_HOPPING_FLAG_NOT_DISABLED   0            /* Any configured sequence hopping not disabled.*/
    #define TI_FAPI_DIS_SEQUENCE_HOPPING_FLAG_DISABLED       1            /* Any configured sequence hopping disabled.*/

/* N SRS field: Indicates if the resource blocks allocated for this grant overlap with the SRS config*/
typedef uint8_t   TiFapi_srsOverlapFlag_t;
    #define TI_FAPI_SRS_OVERLAP_FLAG_NO_OVERLAP  0        /* RBs do not overlap with the SRS.*/
    #define TI_FAPI_SRS_OVERLAP_FLAG_OVERLAP     1        /* RBs overlap with the SRS.*/

typedef uint8_t   TiFapi_lastOfdmSymbolPunctured_t;
    #define TI_FAPI_LAST_OFDM_SYMBOL_PUNCTURED_NOT_PUNCTURED  0        /* Last OFDM symbol is not punctured.*/
    #define TI_FAPI_LAST_OFDM_SYMBOL_PUNCTURED_PUNCTURED      1        /* Last OFDM symbol is punctured.*/

typedef uint8_t   TiFapi_dlDciFormat_t;
    #define TI_FAPI_DL_DCI_FORMAT_FORMAT_1   0        /* DCI format 1.*/
    #define TI_FAPI_DL_DCI_FORMAT_FORMAT_1A  1        /* DCI format 1A.*/
    #define TI_FAPI_DL_DCI_FORMAT_FORMAT_1B  2        /* DCI format 1B.*/
    #define TI_FAPI_DL_DCI_FORMAT_FORMAT_1C  3        /* DCI format 1C.*/
    #define TI_FAPI_DL_DCI_FORMAT_FORMAT_1D  4        /* DCI format 1D.*/
    #define TI_FAPI_DL_DCI_FORMAT_FORMAT_2   5        /* DCI format 2.*/
    #define TI_FAPI_DL_DCI_FORMAT_FORMAT_2A  6        /* DCI format 2A.*/
    #define TI_FAPI_DL_DCI_FORMAT_FORMAT_2B  7        /* DCI format 2B.*/
    #define TI_FAPI_DL_DCI_FORMAT_FORMAT_2C  8        /* DCI format 2C.*/

typedef uint8_t  TiFapi_pmiConfirmation_t;
    #define TI_FAPI_PMI_CONFIRMATION_USE_TPMI_FIELD         0        /* Use precoding indicated in TPMI field.*/
    #define TI_FAPI_PMI_CONFIRMATION_USE_LAST_PMI_REPORT    1        /* Use precoding indicated in last PMI report.*/
 
typedef uint8_t   TiFapi_ngapType_t;
    #define TI_FAPI_NGAPTYPE_N_GAP_ONE     0    /* N_gap1.*/
    #define TI_FAPI_NGAPTYPE_N_GAP_TWO     1     /* N_gap2.*/

typedef uint8_t   TiFapi_allocatePrachFlag_t;
    #define TI_FAPI_ALLOCATE_PRACH_FLAG_FALSE   0        /* False.*/
    #define TI_FAPI_ALLOCATE_PRACH_FLAG_TRUE    1         /* True.*/

typedef uint8_t   TiFapi_mcchFlag_t;
    #define TI_FAPI_MCCH_FIELD_NOT_VALID_FLAG   0        /* MCCH change notification field is not valid*/
    #define TI_FAPI_MCCH_FIELD_VALID_FLAG       1        /* MCCH change notification field is valid*/

typedef uint8_t   TiFapi_crossCarrierFlag_t;
    #define TI_FAPI_CARRIER_IND_NOT_VALID_FLAG   0        /* Carrier indicator field is not valid*/
    #define TI_FAPI_CARRIER_IND_VALID_FLAG       1        /* Carrier indicator field is valid*/

typedef uint8_t   TiFapi_srsValidFlag_t;
    #define TI_FAPI_SRS_NOT_VALID_FLAG          0        /* SRS request field is not valid*/
    #define TI_FAPI_SRS_VALID_FLAG              1        /* SRS request field is valid*/

typedef uint8_t   TiFapi_srsRequestFlag_t;
    #define TI_FAPI_SRS_NOT_REQUESTED_FLAG          0        /* SRS not requested*/
    #define TI_FAPI_SRS_REQUESTED_FLAG              1        /* SRS requested*/
	
typedef uint8_t   TiFapi_ulPduType_t;
    #define TI_FAPI_UL_PDU_TYPE_ULSCH                0                  /* ULSCH TB.*/
    #define TI_FAPI_UL_PDU_TYPE_ULSCH_CQI_RI         1                  /* ULSCH TB + CQI + RI.*/
    #define TI_FAPI_UL_PDU_TYPE_ULSCH_HARQ           2                  /* ULSCH TB + HARQ.*/
    #define TI_FAPI_UL_PDU_TYPE_ULSCH_CQI_HARQ_RI    3                  /* ULSCH TB + CQI + HARQ + RI.*/
    #define TI_FAPI_UL_PDU_TYPE_UCI_CQI              4                  /* UCI + CQI.*/
    #define TI_FAPI_UL_PDU_TYPE_UCI_SR               5                  /* UCI + SR.*/
    #define TI_FAPI_UL_PDU_TYPE_UCI_HARQ             6                  /* UCI + HARQ.*/
    #define TI_FAPI_UL_PDU_TYPE_UCI_SR_HARQ          7                  /* UCI + SR + HARQ.*/
    #define TI_FAPI_UL_PDU_TYPE_UCI_CQI_HARQ         8                  /* CQI + SR.*/
    #define TI_FAPI_UL_PDU_TYPE_UCI_CQI_SR           9                  /* CQI + HARQ.*/
    #define TI_FAPI_UL_PDU_TYPE_UCI_CQI_SR_HARQ      10                 /* UCI + CQI + HARQ*/
    #define TI_FAPI_UL_PDU_TYPE_SRS                  11                 /* SRS.*/
    #define TI_FAPI_UL_PDU_TYPE_HARQ_BUFFER          12               	/* NOT USED/SUPPORTED. HARQ buffer.*/
    #define TI_FAPI_UL_PDU_TYPE_ULSCH_UCI_CSI        13                 /* Data and control over PUSCH and PUCCH simultaneously.*/
    #define TI_FAPI_UL_PDU_TYPE_ULSCH_UCI_HARQ       14                 /* Data and control over PUSCH and PUCCH simultaneously.*/
    #define TI_FAPI_UL_PDU_TYPE_ULSCH_CSI_UCI_HARQ   15                 /* Data and control over PUSCH and PUCCH simultaneously.*/

typedef uint8_t   TiFapi_nPrbDci1a_t;
    #define TI_FAPI_NPRBDCI_1A_TWO      0        /* N_PRB is equal to 2.*/
    #define TI_FAPI_NPRBDCI_1A_THREE    1        /* N_PRB is equal to 3.*/

typedef uint8_t   TiFapi_csiRsFlag_t;
    #define TI_FAPI_CSIRS_NOT_VALID_FLAG          0        /* CSI-RS parameters are not valid*/
    #define TI_FAPI_CSIRS_VALID_FLAG              1        /* CSI-RS parameters are valid*/
	
/* DATA OFFSET FLAG */
typedef uint16_t  TiFapi_dataOffsetFlag_t;
    #define TI_FAPI_DATA_OFFSET_ERROR  0
    #define TI_FAPI_DATA_OFFSET_OK     1 

/* Types of CSI Report */
typedef uint8_t  TiFapi_csiReportType_t;
    #define TI_FAPI_CSI_REPORT_PERIODIC      0
    #define TI_FAPI_CSI_REPORT_APERIODIC     1

/* Types of CQI/PMI/RI Periodic Report control type */
typedef uint8_t  TiFapi_periodicRepCtrlType_t;
    #define TI_FAPI_PERIODIC_REPORT_CQI_PMI  0
    #define TI_FAPI_PERIODIC_REPORT_RI       1

/* SRS PDU antenna ports */
typedef uint8_t  TiFapi_numSrsAntPort_t;
    #define TI_FAPI_SRS_1_ANT_PORT           0
    #define TI_FAPI_SRS_2_ANT_PORT           1
    #define TI_FAPI_SRS_4_ANT_PORT           2

/* UL DCI CQI size */
typedef uint8_t   TiFapi_ulDciCqiSize_t;
    #define TI_FAPI_UL_DCI_CQI_SIZE_1_BIT   0        /* CQI/CSI size 1 bit*/
    #define TI_FAPI_UL_DCI_CQI_SIZE_2_BITS  1        /* CQI/CSI size 2 bits*/

/* UL DCI antenna ports */
typedef uint8_t  TiFapi_numUlDciAntPort_t;
    #define TI_FAPI_UL_DCI_1_ANT_PORT           0
    #define TI_FAPI_UL_DCI_2_ANT_PORT           1
    #define TI_FAPI_UL_DCI_4_ANT_PORT           2
	
/*harq feedback */
typedef uint8_t  TiFapi_harqFeedback_t;
    #define TI_FAPI_HARQ_FDBK_ACK           1
    #define TI_FAPI_HARQ_FDBK_NACK          2
    #define TI_FAPI_HARQ_FDBK_ACK_NACK      3
    #define TI_FAPI_HARQ_FDBK_DTX           4
    #define TI_FAPI_HARQ_FDBK_ACK_DTX       5
    #define TI_FAPI_HARQ_FDBK_NACK_DTX      6
    #define TI_FAPI_HARQ_FDBK_ACK_NACK_DTX  7

/*TDD harq data feedback for specialBundling mode*/
typedef uint8_t  TiFapi_harqFeedbackSpecBundMode_t;
    #define TI_FAPI_HARQ_FDBK_SPEC_BUND_NONE         0
    #define TI_FAPI_HARQ_FDBK_SPEC_BUND_1_4_7_ACKS   1
    #define TI_FAPI_HARQ_FDBK_SPEC_BUND_2_5_8_ACKS   2
    #define TI_FAPI_HARQ_FDBK_SPEC_BUND_3_6_9_ACKS   3
    #define TI_FAPI_HARQ_FDBK_SPEC_BUND_DTX          4

/* HARQ ack/nack Modes for HARQ indication only*/
typedef uint8_t  TiFapi_harqAckNackMode_t;
	/* TDD modes */
	/*both HARQ over PUSCH and HARQ over PUCCH*/
    #define TI_FAPI_HARQ_AN_MODE_TDD_BUNDLING          0 /*Indicates that harq data is for TDD bundling*/
    #define TI_FAPI_HARQ_AN_MODE_TDD_MULTIPLEXING      1 /*Indicates that harq data is for TDD multiplexing*/
    #define TI_FAPI_HARQ_AN_MODE_TDD_SPECIAL_BUNDLING  2 /*Indicates that harq data is for TDD special bundling*/
    #define TI_FAPI_HARQ_AN_MODE_TDD_CH_SELECTION      3 /*Indicates that harq data is for TDD channel selection*/
    #define TI_FAPI_HARQ_AN_MODE_TDD_FORMAT_3          4 /*Indicates that harq data is for TDD format 3*/
	/* FDD modes */
	/* FDD - HARQ over PUCCH*/
    #define TI_FAPI_HARQ_AN_MODE_FDD_FORMAT_1AB        0 /*Indicates that harq data is for FDD format 1a/1b*/
    #define TI_FAPI_HARQ_AN_MODE_FDD_CH_SELECTION      1 /*Indicates that harq data is for FDD channel selection*/
    #define TI_FAPI_HARQ_AN_MODE_FDD_FORMAT_3          2 /*Indicates that harq data is for FDD format 3*/
	/* FDD HARQ over PUSCH (mode not defined in FAPI, therefore use value 255)*/
	#define TI_FAPI_HARQ_AN_MODE_FDD_OVER_PUSCH        255 /*AckNack format not available for HARQ over PUSCH*/

/****************************************
Generic FAPI Message in TLV format. 
****************************************/
/* Message must start at a 32bit boundary*/
typedef struct TiFapi_msg_st
{
     /* Message type ID. Must start at a 32bit boundary. */
     TiFapi_msgIdType_t   msgId;
     /* Length of vendor-specific message body (bytes) */
     uint8_t              lenVendorSpecific; 
     /* Length of message body (bytes) */
     uint16_t             msgLen;
     /* Message body. Must start at a 32bit boundary.*/
     uint8_t              msgBody[TI_FAPI_VAR_SIZE(msgLen)];
     /* Vendor-specific message body. Must start at a 32bit boundary. */
     uint8_t              vendorMsgBody[TI_FAPI_VAR_SIZE(lenVendorSpecific)];
}TiFapi_msg_s;

/******************************************************************************
*
*                Configuration Messages (TLV based)                                    
*
******************************************************************************/

/****************************************
Config Params (table 18)
****************************************/
typedef union 
{
     /* Type of duplexing mode*/
     uint16_t  duplexingMode;
     /*The power per antenna of the PCFICH with respect to the reference signal.*/
     uint16_t  pcfichPowerOffset;
     /*Refers to downlink power allocation. */
     uint16_t  pb;
     /*Cyclic prefix type*/
     uint16_t  dlCyclicPrefixType;
     /*Cyclic prefix type*/
     uint16_t  ulCyclicPrefixType;

     /* RF Config */
     /*Downlink channel bandwidth in resource blocks.*/
     uint16_t  dlChannelBW;
     /*Uplink channel bandwidth in resource blocks.*/
     uint16_t  ulChannelBW;
     /* Normalized value levels (relative) to accommodate different 
     absolute Tx Power used by eNb.*/
     uint16_t  refSignalPower;
     /*No. of cell specific transmit antenna ports.*/
     uint16_t  txAntennaPort;
     /* No. of cell specific receive antenna ports.*/
     uint16_t  rxAntennaPort;
     /* RF Config ends */

     /* PHICH Config */
     /* No.of resource element groups used for PHICH*/
     uint16_t  phichResource;
     /* No. resource element groups used for PHICH*/
     uint16_t  phichDuration;
     /* The power per antenna of the PHICH with respect to the reference signal*/
     uint16_t  phichPowOffset;
     /* PHICH Config  ends */
     
     /* SCH Config */
     /* The power of synchronization signal with respect to the reference signal*/
     uint16_t  primarySyncSignal;
     /* The power of synchronization signal with respect to the reference signal*/
     uint16_t  secondarySyncSignal;
     /* The Cell ID sent with the synchronization signal*/
     uint16_t  physicalCellId;
     /* SCH Config ends */

     /* PRACH Config start*/
     /* Provides information about the location and format of the PRACH.*/ 
     uint16_t  configurationIndex;
     /* PRACH Root sequence index*/
     uint16_t  rootSeqIndex;
     /* Equivalent to Ncs*/
     uint16_t  zeroCorelationZoneConfig;
     /* Indicates if unrestricted, or restricted, set of preambles is used*/
     uint16_t  highSpeedFlag;
     /* The first physical resource block available for PRACH*/
     uint16_t  freqOffset;
     /* PRACH Config ends */

     /* PUSCH Config */
     /* If hopping is enabled indicates the type of hopping used*/
     uint16_t  hoppingMode;
     /* The offset used if hopping is enabled*/
     uint16_t  hoppingOffset;
     /* The number of sub-bands used for hopping*/
     uint16_t  numOfSubBand;
     /* PUSCH Config ends */
    
     /* PUCCH Config */
     /* The cyclic shift difference*/
     uint16_t  deltaPUCCHShift;
     /* The bandwidth, in units of resource blocks, that is available for use 
     by PUCCH formats 2/2a/2b transmission in each slot.*/
     uint16_t  nCQIRB;
     /* The number of cyclic shifts used for PUCCH formats 1/1a/1b in a 
     resource block with a mix of formats 1/a/1/ab and 2/2a/2b*/
     uint16_t  nAnCs;
     /*N(1)PUCCH*/
     uint16_t  n1PucchAn;
     /* PUCCH Config ends */

     /* SRS Config */
     /* The available SRS bandwidth of the cell
     The value is an index into the referenced table.*/
     uint16_t  bandWidthConfiguration;
     /*Used for TDD only
     Indicates how SRS operates in UpPTS subframes*/
     uint16_t  maxUpPTS;
     /* The subframe configuration. 
     Needed if semi-static configuration is held in PHY.*/
     uint16_t  SRSSubframeConfiguration;
     /* Indicates if SRS and ACK/NACK can be received in the same subframe. 
     Needed if semi-static configuration is held in PHY.*/
     uint8_t  srsAckNackSimulTx;
     /* SRS Config ends */

     /* Uplink Reference Signal Config */
     /* Indicates the type of hopping to use.*/
     uint16_t  uplinkRSHoping;
     /* The sequence shift pattern used if group hopping is enabled.*/
     uint16_t  groupAssignment;
     /* Specifies the cyclic shift for the reference signal used in the cell.
     The value is an index into the referenced table.*/
     uint16_t  cyclicShift1forDMRS;
     /* Uplink Reference Signal Config ends */

     /* Tdd frame structure config */
     /* indicates the DL/UL subframe structure.*/
     uint16_t  subFrameAssignment;
     /* Length of fields DwPTS, GP and UpPTS.*/
     uint16_t  specialSubFramePatterns;
     /* Tdd frame structure config ends */

     /* MBSFN Config starts */
	 /*Indicates MBSFN area ID. Value*/
  	 uint16_t    mbSfnAreaId;
     /* MBSFN Config ends */

     /* PRS Config starts */
 	 /*Transmission bandwidth of PRS. */
	 uint8_t     prsBandwidth;
	 /*The cyclic prefix used for PRS transmission.*/
	 uint8_t     prsCyclicPrefixType;
     /* PRS Config ends */

     /* The PHY downlink channel bandwidth capability (in resource blocks).*/
     uint16_t    dlBandWidthSupport;
     /* The PHY uplink channel bandwidth capability (in resource blocks).*/
     uint16_t    ulBandWidthSupport;
     /* The PHY downlink modulation capability.*/
     uint16_t    dlModulationSupport;
     /* The PHY uplink modulation capability.*/
     uint16_t    ulModulationSupport;
     /* Number of antennas supported.*/
     uint16_t    phyAntennaCapability;
	 /*Indicates which release the PHY supports*/
	 uint16_t    releaseCapability;
	 /*Indicates support for MBSFN features*/
	 uint16_t    mbSfnCapability;

     /* The data report mode for the uplink data.*/
     uint16_t    dataReportingMode;
     /* The future SFN/SF */
     uint16_t    sfnsf;

     /* Indicates the current operational state of the PHY.*/
     uint16_t    phyState; 
     
	 /* TI FAPI header file version:
	  * Used by L1/L2 to check if correct FAPI header file is being used
	  * The most significant byte:  major version.
	  * The least significant byte: minor version.
        Note that this parameter is not part of the FAPI spec.*/
     uint16_t    tiFapiHeaderVersion;
}TiFapi_configParam_u;

/****************************************
PARAM.request
****************************************/
/* No message body is defined in FAPI*/

/****************************************
PARAM.response
****************************************/
/* TLV used for param.response. TLV size must be a multiple of 4 bytes. 
   That is, sizeof(TiFapi_paramResponseTLV_s) must be a multiple of 4 bytes.*/ 
typedef struct TiFapi_paramResponseTLV_st
{
     /* tag values are defined in TiFapi_cellConfig_t */
     uint8_t                   tag; 
     /* length */
     uint8_t                   tagLen;
	 /* Value*/
     TiFapi_configParam_u      configParam;     
}TiFapi_paramResponseTLV_s;

typedef struct TiFapi_paramResponse_st
{
	 /* error code */
     TiFapi_l1ErrorCodes_t       errCode; 
     /* number of TLVs */
	 uint8_t                     numOfTlv;
	 /* padding to ensure that the TLVs and the proprietary parameters start at a four byte boundary*/
	 uint8_t                     pad[2];
	 /* list of TLVs. Must start at a 4 bytes boundary and 
	    each TLV in the list must have a size that is a multiple of 4 bytes.*/
     TiFapi_paramResponseTLV_s   tlvs[TI_FAPI_VAR_SIZE(numOfTlv)];
}TiFapi_paramResponse_s;

/****************************************
CONFIG.request
****************************************/
/* Config TLV. Each TLV must have a total length multiple of 4 bytes. 
   That is, sizeof(TiFapi_configTLV_s) must be a multiple of 4 bytes.*/
typedef struct TiFapi_configTLV_st 
{
     /* tag values are defined in TiFapi_cellConfig_t */
     uint8_t                   tag; 
     /* length */
     uint8_t                   tagLen;
	 /* value */
     TiFapi_configParam_u      configParam;     
}TiFapi_configTLV_s;

typedef struct TiFapi_configReq_st
{
	 /* number of TLVs */
     uint8_t               numOfTlv;
     /* padding to ensure that the TLVs and the proprietary parameters start at a four byte boundary*/
	 uint8_t               pad[3];
	 /* list of TLVs. Must start at a 4 bytes boundary and 
	    each TLV in the list must have a size that is a multiple of 4 bytes.*/
     TiFapi_configTLV_s    configTLVs[TI_FAPI_VAR_SIZE(numOfTlv)];
}TiFapi_configReq_s;

/****************************************
CONFIG.response
****************************************/
typedef struct TiFapi_configResp_st
{
    /* error code */
    TiFapi_l1ErrorCodes_t   errorCode; 
    /* Number of invalid or unsupported TLVs contained in the message body. */
    uint8_t                 numOfInvalidOrUnsupportedTLV;
    /* Number of missing TLVs contained in the message body. */
    uint8_t                 numOfMissingTLV;
	/* Pad to ensure that the TLVs and the proprietary parameters start at a 4 byte boundary*/
    uint8_t                 pad[1];
    /* A list of invalid or unsupported TLVs. 
	  Must start at a 4 bytes boundary and each TLV in the list must have a size that is a multiple of 4 bytes.*/
    TiFapi_configTLV_s      listOfTLVs[TI_FAPI_VAR_SIZE(numOfInvalidOrUnsupportedTLV)];
    /* A list of missing TLVs.
	  Must start at a 4 bytes boundary and each TLV in the list must have a size that is a multiple of 4 bytes.*/
    TiFapi_configTLV_s      listOfMissingTLVs[TI_FAPI_VAR_SIZE(numOfTlv)];
}TiFapi_configResp_s;

/******************************************************************************
*
*                PROPRIETARY PARAMETERS USED IN THE CONFIGURATION MESSAGES
*
******************************************************************************/
typedef uint32_t  TiFapi_testMode_t;
    #define TI_FAPI_TEST_MODE_DISABLED  0 /* Test mode is disabled */
    #define TI_FAPI_TEST_MODE_ETM11     1 /* Activate E-TM 1.1 */
    #define TI_FAPI_TEST_MODE_ETM12     2 /* Activate E-TM 1.2 */
    #define TI_FAPI_TEST_MODE_ETM20     3 /* Activate E-TM 2.0 */
    #define TI_FAPI_TEST_MODE_ETM31     4 /* Activate E-TM 3.1 */
    #define TI_FAPI_TEST_MODE_ETM32     5 /* Activate E-TM 3.2 */
    #define TI_FAPI_TEST_MODE_ETM33     6  /* Activate E-TM 3.3 */

typedef uint32_t  TiFapi_samplingFreq_t;
    #define TI_FAPI_SAMPLING_FREQ_VOID   0       /* Sampling frequency not specified; deduced from the configured bandwidth */
    #define TI_FAPI_SAMPLING_FREQ_7680   7680    /* 7680 */
    #define TI_FAPI_SAMPLING_FREQ_15360  15360   /* 15360 */
    #define TI_FAPI_SAMPLING_FREQ_23040  23040   /* 23040 */
    #define TI_FAPI_SAMPLING_FREQ_30720  30720   /* 30720 */

typedef uint32_t  TiFapi_samplingFreqSupport_t;
    #define TI_FAPI_SAMPLING_FREQ_SUPPORT_7680   0x01  /* 7680 */
    #define TI_FAPI_SAMPLING_FREQ_SUPPORT_15360  0x02  /* 15360 */
    #define TI_FAPI_SAMPLING_FREQ_SUPPORT_23040  0x04  /* 23040 */
    #define TI_FAPI_SAMPLING_FREQ_SUPPORT_30720  0x08  /* 30720 */

typedef uint32_t  TiFapi_testModeSupport_t;
    #define TI_FAPI_TEST_MODE_SUPPORT_DISABLED  0     /* Deactivate E-TM */
    #define TI_FAPI_TEST_MODE_SUPPORT_ETM11     0x1   /* Activate E-TM 1.1 */
    #define TI_FAPI_TEST_MODE_SUPPORT_ETM12     0x2   /* Activate E-TM 1.2 */
    #define TI_FAPI_TEST_MODE_SUPPORT_ETM20     0x4   /* Activate E-TM 2.0 */
    #define TI_FAPI_TEST_MODE_SUPPORT_ETM31     0x8   /* Activate E-TM 3.1 */
    #define TI_FAPI_TEST_MODE_SUPPORT_ETM32     0x10  /* Activate E-TM 3.2 */
    #define TI_FAPI_TEST_MODE_SUPPORT_ETM33     0x20  /* Activate E-TM 3.3 */

typedef uint32_t  TiFapi_linkRate_t;
    #define TI_FAPI_LINK_RATE_2X         0 /* CPRI link rate 2x */
    #define TI_FAPI_LINK_RATE_4X         1  /* CPRI link rate 4x */

typedef uint32_t  TiFapi_linkType_t;
    #define TI_FAPI_LINK_TYPE_NORMAL     0 /* CPRI link type normal */
    #define TI_FAPI_LINK_TYPE_RESERVED   1  /* CPRI link type reserved */

typedef uint32_t  TiFapi_aif2PllMultFact_t;
    #define TI_FAPI_AIF2_MULT_FACT_4X        0
    #define TI_FAPI_AIF2_MULT_FACT_5X        1
    #define TI_FAPI_AIF2_MULT_FACT_6X        2
    #define TI_FAPI_AIF2_MULT_FACT_8X        3
    #define TI_FAPI_AIF2_MULT_FACT_8_25X     4
    #define TI_FAPI_AIF2_MULT_FACT_10X       5
    #define TI_FAPI_AIF2_MULT_FACT_12X       6
    #define TI_FAPI_AIF2_MULT_FACT_12_5X     7
    #define TI_FAPI_AIF2_MULT_FACT_15X       8
    #define TI_FAPI_AIF2_MULT_FACT_16X       9
    #define TI_FAPI_AIF2_MULT_FACT_16_5X     10
    #define TI_FAPI_AIF2_MULT_FACT_20X       11
    #define TI_FAPI_AIF2_MULT_FACT_22X       12
    #define TI_FAPI_AIF2_MULT_FACT_25X       13

typedef uint32_t  TiFapi_aif2TriggerType_t;
    #define TI_FAPI_AIF2_TRIGGER_TYPE_SW        0
    #define TI_FAPI_AIF2_TRIGGER_TYPE_HW        1
    #define TI_FAPI_AIF2_TRIGGER_TYPE_HW_RELAY  2

typedef uint32_t  TiFapi_antConfigDirection_t;
    #define TI_FAPI_ANT_CONFIG_DIRECTION_RX  0
    #define TI_FAPI_ANT_CONFIG_DIRECTION_TX  1

/***************************************************************
Proprietary config parameters are passed to the PHY 
as proprietary (extension) TLVs in the configuration messages
****************************************************************/
/* Link configuration parameters. Must have a size multiple of 4 bytes */	
typedef struct TiFapi_linkConfig_st
{
    uint32_t           linkIndex;             /* Link index */
    TiFapi_linkRate_t  linkRate;              /* Link rate */
    TiFapi_linkType_t  linkType;              /* Link type */
    uint32_t           piOffset;              /* PI Offset */
    uint32_t           deltaOffset;           /* Delta Offset */
    uint32_t           rxAxcOffset;           /* RX AxC Offset */
    uint32_t           ulCpriFrameDelay;      /* Delay in UL CPRI Frame for antenna radio frame alignment */
    uint32_t           dlCpriFrameAdvance;    /* Advance in DL CPRI Frame for antenna radio frame alignment */
} TiFapi_linkConfig_s;

/* Antenna configuration parameters. Must have a size multiple of 4 bytes */	
typedef struct TiFapi_antConfig_st
{
    uint32_t                    cellIndex;
    uint32_t                    antNumber;
    uint32_t                    linkIndex;
    uint32_t                    axcIndex;
    TiFapi_antConfigDirection_t antConfigDirection;
} TiFapi_antConfig_s;

/* Union of all proprietary parameters. All parameters must have a size multiple of 4 bytes.*/
typedef union 
{
   TiFapi_testMode_t            testMode;
   TiFapi_samplingFreq_t        dlSamplFreq;
   TiFapi_samplingFreq_t        ulSamplFreq;
   TiFapi_aif2TriggerType_t     aif2TriggerType;
   TiFapi_aif2PllMultFact_t     aif2PllMultFact;
   uint32_t                     aif2SyncTimeout;
   TiFapi_linkConfig_s          aif2LinkConfig;
   TiFapi_antConfig_s           antConfig;
   uint32_t					    cellCfgReserved;
   uint32_t                     ghw;
   uint32_t                     numOfRaPreambles;
   uint32_t                     supportedDlUsers;
   uint32_t                     supportedUlUsers;
   TiFapi_testModeSupport_t     testModeSupport;
   TiFapi_samplingFreqSupport_t dlSamplFreqSupport;
   TiFapi_samplingFreqSupport_t ulSamplFreqSupport;
}TiFapi_configParamExt_u;

/* Extension/proprietary TLV definition. This TLV must have a size that is multiple of 4 bytes.
   That is, sizeof(TiFapi_configTLVExt_s) is a multiple of 4 bytes.  */
typedef struct TiFapi_configTLVExt_st
{
    /* tag values are defined in TiFapi_cellConfig_t */
    uint8_t                 tag;
	/* size in bytes of TLV element, not sizeof union (TiFapi_configParamExt_u)*/
    uint8_t                 tagLen;
	/* padding to ensure that the TLV size is a multiple of 4 bytes.*/
	uint8_t				    pad[2];
	/* Proprietary configuration parameter. 
	   Must be aligned at a 4 bytes boundary and must have a size that is multiple of 4 bytes */
    TiFapi_configParamExt_u configParamExt;     
} TiFapi_configTLVExt_s;

/* Extension/proprietary TLV definition. This TLV must have a size that is multiple of 4 bytes. */
typedef struct TiFapi_paramResponseTLVExt_st
{
    /* tag values are defined in TiFapi_cellConfig_t */
    uint8_t                      tag;
	/* size in bytes of TLV element, not sizeof union*/
    uint8_t                      tagLen;
	/* padding to ensure that the TLV size is a multiple of 4 bytes.*/
	uint8_t						 pad[2];
	/* Proprietary configuration parameter. 
	   Must be aligned at a 4 bytes boundary and must have a size that is multiple of 4 bytes */
    TiFapi_configParamExt_u      configParamExt;     
} TiFapi_paramResponseTLVExt_s;

/****************************************
START.request
****************************************/
/* No message body is defined in FAPI*/

/****************************************
STOP.request
****************************************/
/* No message body is defined in FAPI*/

/****************************************
STOP.indication
****************************************/
/* No message body is defined in FAPI*/

/****************************************
ERROR.indication
****************************************/
/* For error codes TI_FAPI_SFN_OUT_OF_SYNC and TI_FAPI_MSG_INVALID_SFN */
typedef struct TiFapi_errMsgBody1_st
{
    /* The SFN/SF value received in the message */
    uint16_t  recvdSfnSf;
    /* The SFN/SF value the PHY was expecting to receive in the message */
    uint16_t  expectedSfnSf;
}TiFapi_errMsgBody1_s;

/* if error codes is TI_FAPI_MSG_PDU_ERR */
typedef struct TiFapi_errMsgBody2_st
{
    uint8_t    subErrCode;
    /* Indicates if this error was in a DL subframe configuration or an 
    UL subframe configuration.
    0 = DL, 1 = UL */
    uint8_t    direction;  
    /* The RNTI in the received PDU. If the error occurred in a MCH, or BCH, 
    PDU this value is set to 0  */
    uint16_t   rnti;
    /* The PDU Type  parameter specified in both DL subframe configuration and 
    UL subframe configuration   */
    uint8_t   pduType;
}TiFapi_errMsgBody2_s;

/* if error codes is TI_FAPI_MSG_HI_ERR */
typedef struct TiFapi_errMsgBody3_st
{
      uint8_t   subErrCode;
      /* The PHICH RB Index parameters specified in each HI PDU */
      uint8_t   phichLowestulRbIndex;
}TiFapi_errMsgBody3_s;

/* if error codes is TI_FAPI_MSG_TX_ERR */
typedef struct TiFapi_errMsgBody4_st
{
      uint8_t   subErrCode;
      /* The PDU index parameter specified for each PDU */
      uint8_t   pduIndex;
}TiFapi_errMsgBody4_s;


typedef struct TiFapi_rntiInfo_st
{
     uint16_t rnti;
     uint8_t pduIndex;
     uint8_t codeWord;
}TiFapi_rntiInfo_s; 

/* if error codes is TI_FAPI_MSG_TX_MISSING_ERR */
typedef struct TiFapi_errMsgBody5_st
{
      uint16_t          sfnSf;
      uint8_t           subErrCode;
      uint8_t           numMissedRecords;
      TiFapi_rntiInfo_s missedRecords[TI_FAPI_MAX_NUM_DLSCH_PDUS];
}TiFapi_errMsgBody5_s;

/* Union of all message body types */
typedef union 
{
    TiFapi_errMsgBody1_s msgBody1;
    TiFapi_errMsgBody2_s msgBody2;
    TiFapi_errMsgBody3_s msgBody3;
    TiFapi_errMsgBody4_s msgBody4;
    TiFapi_errMsgBody5_s msgBody5;
} TiFapi_errMsgBody_u;

typedef struct TiFapi_errorInd_st
{
    /* Indicate which message received by the PHY has an error */
    TiFapi_msgIdType_t      msgId;
    TiFapi_l1ErrorCodes_t   errorCode;
    TiFapi_errMsgBody_u     errMsgBody;
}TiFapi_errorInd_s;


/******************************************************************************
*
*                Subframe Messages - no data (non-TLV based)
*
******************************************************************************/

/****************************************
DL_CONFIG.request
****************************************/
typedef struct TiFapi_dlDciPduCfg_st  
{
    TiFapi_dlDciFormat_t          dciFormat;                  /* DL DCI format*/
    uint8_t                       cceIndex;                   /* The CCE index used to send the DCI*/
    uint8_t                       aggregationLevel;           /* The aggregation level used*/
    uint16_t                      rnti;                       /* The Radio Network Temporary Identifier used for identifying the UE*/
    TiFapi_resAllcType_t          resourceAllocationType;     /* Resource allocation type*/
    TiFapi_vRBAssignmentFlag_t    virtualRbType;              /* VRB assignment flag*/
    uint32_t                      resourceBlockCoding;        /* Resource block coding. Specifies the  Resource allocation. The value depends upon the resource allocation type*/
    uint8_t                       mcsTb0;                     /* The MCS for TB 0*/
    uint8_t                       redundancyVersionTb0;       /* The HARQ redundancy version for TB 0*/
    uint8_t                       newDataIndicatorTb0;        /* New data indicator for TB 0*/
    TiFapi_tbToCodeWordSwapFlag_t tbSwapFlag;                 /* Indicates the mapping of transport blocks to codewords*/
    uint8_t                       mcsTb1;                     /* The MCS for TB 1*/
    uint8_t                       redundancyVersionTb1;       /* The HARQ redundancy version for TB 1*/
    uint8_t                       newDataIndicatorTb1;        /* New data indicator for TB 1*/
    uint8_t                       harqProcessNumber;          /* The HARQ process number*/
    uint8_t                       tpmi;                       /* The codebook index used for precoding. Valid for DCI formats 1B and 1D*/
    TiFapi_pmiConfirmation_t      pmi;                        /* Confirmation of the precoding Valid for DCI format 1B*/
    uint8_t                       precodingInformation;       /* Precoding information. Valid for DCI formats 2 and 2A*/
    TiFapi_tpcValue_t             tpc;                        /* Tx power control command for PUCCH. Valid for DCI formats 1, 1A, 1B, 1D, 2 and 2A*/
    uint8_t                       downlinkAssignmentIndex;    /* Downlink assignment index. Valid for DCI formats 1, 1A, 1B, 1D, 2, 2A*/
    TiFapi_ngapType_t             nGap;                       /* N_gap; used for VRB. Valid for DCI formats 1B, 1C, 1D*/
    uint8_t                       tbsIndex;                   /* TBS index. Valid for DCI formats 1C*/
    uint8_t                       dlPowerOffset;              /* DL power offset. Valid for DCI formats 1D*/
    TiFapi_allocatePrachFlag_t    allocatePrachFlag;          /* Allocate PRACH flag. Valid for DCI formats 1A*/
    uint8_t                       preambleIndex;              /* Preamble index. Valid for DCI formats 1A*/
    uint8_t                       prachMaskIndex;             /* PRACH mask index. Valid for DCI formats 1A*/
    TiFapi_rntiType_t             rntiType;                   /* RNTI type. Valid for DCI format 1, 1A, 2, 2A*/
    uint16_t                      txPower;                    /* Offset to the RS power*/
    TiFapi_mcchFlag_t             mcchFlag;                   /* Indicates if format 1C is being used to signal a MCCH change notification. Valid for DCI formats 1C*/
    uint8_t                       mcchChangeNotification;     /* MCCH Change Notification. Valid for DCI format 1C.*/
    uint8_t                       scramblingId;               /* Scrambling Identity. Indicates the scrambling identity value NSCID. Valid for DCI format 2B*/
    TiFapi_crossCarrierFlag_t     crossCarrierFlag;           /* Indicates if cross carrier scheduling has been enabled for UE*/
    uint8_t                       carrierIndicator;           /* Serving cell index*/
    TiFapi_srsValidFlag_t         srsFlag;                    /* SRS valid flag*/
    TiFapi_srsRequestFlag_t       srsRequestFlag;             /* SRS request flag*/
    uint8_t                       portScrambLayers;           /* Indicates the antenna port, scrambling identity value and number of layers*/
    uint8_t                       dciLength;                  /* The total DCI length including padding bits.*/
} TiFapi_dlDciPduCfg_s;

typedef struct TiFapi_bchPduCfg_st 
{
    uint16_t  length;     /* The length in bytes of the BCH PDU*/
    uint16_t  pduIdx;     /* A count value incremented for each BCH, MCH, PCH or DLSCH PDU*/
    uint16_t  txPower;    /* Offset to the reference signal power*/
} TiFapi_bchPduCfg_s;

typedef struct TiFapi_mchPduCfg_st  
{
    uint16_t                     length;                     /* The length in bytes of the MCH PDU*/
    uint16_t                     pduIdx;                     /* A count value incremented for each BCH, MCH, PCH or DLSCH PDU*/
    uint16_t                     rnti;                       /* The RNTI associated with the MCH*/
    TiFapi_resAllcType_t         resourceAllocationType;     /* Resource allocation type*/
    uint32_t                     resourceBlockCoding;        /* Resource block coding Specifies the  Resource allocation The value depends upon the resource allocation type*/
    TiFapi_modulationType_t      modulation;                 /* The modulation type*/
    uint16_t                     txPower;                    /* Offset to the reference signal power*/
} TiFapi_mchPduCfg_s;

typedef struct TiFapi_dlschPduCfg_st  
{
    uint16_t                      length;                     /* The length in bytes of the DLSCH PDU*/
    uint16_t                      pduIdx;                     /* A count value incremented for each BCH, MCH, PCH or DLSCH PDU*/
    uint16_t                      rnti;                       /* The RNTI associated with the DLSCH*/
    TiFapi_resAllcType_t          resourceAllocationType;     /* Resource allocation type*/
    TiFapi_vRBAssignmentFlag_t    virtualRbType;              /* VRB assignment flag*/
    uint32_t                      resourceBlockCoding;        /* Resource block coding Specifies the  Resource allocation The value depends upon the resource allocation type*/
    TiFapi_modulationType_t       modulation;                 /* The modulation type*/
    uint8_t                       redundancyVersion;          /* The HARQ redundancy version*/
    uint8_t                       transportBlocks;            /* A value of 2 indicates the second TB for MIMO Else always set to 1*/
    TiFapi_tbToCodeWordSwapFlag_t tbSwapFlag;                 /* Indicates the mapping of transport blocks to codewords*/
    TiFapi_transScheme_t          txScheme;                   /* The transmission scheme used for this PDU*/
    uint8_t                       numLayers;                  /* The number of layers used in transmission*/
    uint8_t                       numSubbands;                /* Only valid when transmission scheme = 3,4,5*/
    uint8_t                       codebookIdx[TI_FAPI_DLSCH_PDU_CFG_MAX_SUBBANDS];    /* Only valid when tx scheme = 3,4,5 Defined the codebook used*/
    uint8_t                       ueCategory;                 /* The UE capabilities category*/
    TiFapi_paValue_t              pa;                         /* The ratio of PDSCH EPRE to cell-specific RS EPRE among PDSCH REs in all the OFDM symbols not containing cell-specific RS*/
    uint8_t                       deltaPowerOffsetIdx;        /* Delta power offset, value 0,1*/
    TiFapi_ngapType_t             nGap;                       /* Used in the virtual resource block distribution*/
    TiFapi_nPrbDci1a_t            nPrb;                       /* Used with DCI format 1A and SI-RNTI or RA-RNTI Value should match with the TPC field in the corresponding DCI 1A PDU*/
    uint8_t                       numBfPrbPerSubband;         /* Number of PRBs that are treated as one subband*/
	uint8_t                       txMode;                     /* The transmission mode associated with the UE*/
    uint8_t                       numBfVector;                /* Number of beam forming vectors*/
    uint8_t                       nScId;                      /* 0: NSCID=0 1: NSCID=1*/
	TiFapi_csiRsFlag_t            csiRsFlag;                  /* Indicates if parameters related to CSI-RS are valid or not*/
    uint8_t                       csiRsResCfg;                /* Referece signal configuration for CSI-RS*/
    uint16_t                      csiRsTxPwCfg;               /* Bitmap of 16 bits for CSI-RS power config*/
} TiFapi_dlschPduCfg_s;

typedef struct TiFapi_pchPduCfg_st  
{
    uint16_t                      length;                     /* The length in bytes of the PCH PDU*/
    uint16_t                      pduIdx;                     /* A count value incremented for each BCH, MCH, PCH or DLSCH PDU*/
    uint16_t                      rnti;                       /* The RNTI associated with the DLSCH*/
    TiFapi_resAllcType_t          resourceAllocationType;     /* Resource allocation type*/
    TiFapi_vRBAssignmentFlag_t    virtualRbType;              /* VRB assignment flag*/
    uint32_t                      resourceBlockCoding;        /* Resource block coding Specifies the  Resource allocation The value depends upon the resource allocation type*/
    uint8_t                       mcs;                        /* The modulation type*/
    uint8_t                       redundancyVersion;          /* The HARQ redundancy version*/
    uint8_t                       transportBlocks;            /* num of transport blocks*/
    TiFapi_tbToCodeWordSwapFlag_t tbSwapFlag;                 /* Indicates the mapping of transport blocks to codewords*/
    TiFapi_transScheme_t          txScheme;                   /* The transmission scheme used for this PDU*/
    uint8_t                       numLayers;                  /* The number of layers used in transmission*/
    uint8_t                       codebookIdx;                /* Codebook idx*/
    uint8_t                       ueCategory;                 /* The UE capabilities category*/
    TiFapi_paValue_t              pa;                         /* The ratio of PDSCH EPRE to cell-specific RS EPRE among PDSCH REs in all the OFDM symbols not containing cell-specific RS*/
    uint16_t                      txPower;                    /* Transmission power */
    TiFapi_nPrbDci1a_t            nPrb;                       /* Used with DCI format 1A and SI-RNTI or RA-RNTI Value should match with the TPC field in the corresponding DCI 1A PDU*/
	TiFapi_ngapType_t             nGap;                       /* Ngap */
} TiFapi_pchPduCfg_s;

typedef struct TiFapi_prsPduCfg_st  
{
    uint16_t  txPower;    /* Offset to the reference signal power. */ 
} TiFapi_prsPduCfg_s;

typedef struct TiFapi_csiRsPduCfg_st  
{
    uint8_t    antPortCnt;    /* Number of antennas used for transmission of CSI ref signal. */
    uint8_t    resCfg;        /* Referece signal configuration for CSI-RS*/
    uint16_t   txPower;       /* Offset to the reference signal power */
    uint16_t   txPwCfg;       /* Bitmap of 16 bits for CSI-RS power config*/
} TiFapi_csiRsPduCfg_s;

typedef struct TiFapi_dlCfgReq_st
{
    uint16_t  sfnSf;                               /* SFN/SF.*/
    uint8_t   numOfdmSymbPdcch;                    /* The number of OFDM symbols for the PDCCH.*/
    uint8_t   numDlDciPdus;                        /* The number of DL DCI PDUs for this TTI.*/
    uint8_t   numBchPdus;                          /* Either 0 or 1 for not present respectively present BCH PDU.*/
    uint8_t   numDlschPdus;                        /* The number of DLSCH PDUs for this TTI.*/
    uint8_t   numMchPdus;                          /* The number of MCH PDUs for this TTI.*/
    uint8_t   numPchPdus;                          /* The number of PCH PDUs for this TTI.*/
    uint8_t   numPrsPdus;                          /* The number of PRS PDUs for this TTI.*/
    uint8_t   numCsiRsPdus;                        /* The number of CSI-RS PDUs for this TTI.*/
    uint8_t   numPdschRntis;                       /* The number of unique RNTIs sent on the PDSCH.*/
    uint16_t  txPowerPcfich;                       /* Offset to the reference signal power.*/
	uint8_t   numBfLayers;                         /* Number of layers for the beam-forming vectors */
    TiFapi_bchPduCfg_s        bch;                                         /* The BCH PDU configuration.*/	
    TiFapi_prsPduCfg_s        prs;                                         /* The PRS PDU configuration.*/
    TiFapi_dlDciPduCfg_s      dldci[TI_FAPI_MAX_NUM_DL_DCI_PDUS];          /* The DL DCI PDU configuration.*/
    TiFapi_dlschPduCfg_s      dlsch[TI_FAPI_MAX_NUM_DLSCH_PDUS];           /* The DLSCH PDU configuration.*/
    TiFapi_mchPduCfg_s        mch[TI_FAPI_MAX_NUM_MCH_PDUS];               /* The MCH PDU configuration.*/
    TiFapi_pchPduCfg_s        pch[TI_FAPI_MAX_NUM_PCH_PDUS];               /* The PCH PDU configuration.*/
    TiFapi_csiRsPduCfg_s      csiRs[TI_FAPI_MAX_NUM_CSIRS_PDUS];           /* The CSI-RS PDU configuration.*/
	uint32_t                  addrOfBfBuffers[TI_FAPI_BF_MAX_NUM_LAYERS];  /* Address to buffers that contain the beam-forming weights. Each BfBuffer contains 
	                                                                          the BF weights for all RBs and antennas in one layer. The size of each 
																			  BfBuffer is (2 * TI_FAPI_BF_MAX_NUM_RBS * TI_FAPI_BF_MAX_NUM_ANTENNAS) */
} TiFapi_dlCfgReq_s;

/* Structures of the beam-forming buffers: bfBuffers */
typedef struct TiFapi_bfValue_st {
    uint8_t           real;     /* Beam forming value real part*/
    uint8_t           imag;     /* Beam forming value imag part*/
}TiFapi_bfValue_s;

typedef struct TiFapi_bfRb_st {
    TiFapi_bfValue_s  bfValAnt[TI_FAPI_BF_MAX_NUM_ANTENNAS];
}TiFapi_bfRb_s;

/*bfBuffer for a given MIMO layer*/
typedef struct TiFapi_bfBuffer_st
{
    TiFapi_bfRb_s     rb[TI_FAPI_BF_MAX_NUM_RBS];
}TiFapi_bfBuffer_s;


/****************************************
UL_CONFIG.request
****************************************/

typedef struct TiFapi_ulschCfg_st
{
    uint32_t                          handle;                    /* An opaque handle that will be returned to L23*/
    uint16_t                          size;                      /* The size of the PDU in bytes Can be 0 in case that only HARQ and CQI is received on ULSCH*/
    uint16_t                          rnti;                      /* The RNTI used for identifying the UE when receiving the PDU*/
    uint8_t                           resourceBlockStart;        /* Resource indication value corresponding to the starting resource block*/
    uint8_t                           numRbs;                    /* The number of resource blocks this PDU will be mapped to*/
    TiFapi_modulationType_t           modulationType;            /* Modulation Type*/
    uint8_t                           n2Dmrs;                    /* The 2nd cyclic shift for DMRS assigned to the UE in the relevant UL grant*/
    TiFapi_frequencyHoppingFlag_t     hoppingFlag;               /* Frequency hopping flag*/
    uint8_t                           hoppingBits;               /* Frequency hopping bits*/
    uint8_t                           newDataIndicator;          /* New data indicator for TB 0 Specify whether this received PUSCH is a new transmission from UE*/
    uint8_t                           redundancyVersion;         /* Redundancy Version*/
    uint8_t                           harqProcessNumber;         /* The HARQ process number*/
    TiFapi_ulTxMode_t                 txMode;                    /* UL Tx Mode*/
    uint8_t                           currentTxNb;               /* The current HARQ transmission count of this TX Valid if frequency hopping is enabled*/
    TiFapi_srsOverlapFlag_t           nSrs;                      /* Indicates if the RBs allocated for this grant overlap with the SRS configuration*/
    TiFapi_resAllcType_t              resourceAllocationType;    /* Resource allocation type*/
    uint32_t                          resourceBlockCoding;       /* Used for resource allocation type 1*/
    uint8_t                           transportBlocks;           /* Transport block transmitted from this RNTI*/
    TiFapi_transSchemeUlsch_t         txScheme;                  /* The transmission scheme used for this PDU*/
    uint8_t                           numLayers;                 /* The number of layers used in transmission*/
    uint8_t                           codebookIdx;               /* Codebook used for precoding*/
	TiFapi_disSequenceHoppingFlag_t   disSequenceHopFlag;        /* Indicates if any configured group hopping should be disabled for the UE */
} TiFapi_ulschCfg_s;

/* ULSCH PDU */
typedef struct TiFapi_ulschPduCfg_st
{
    TiFapi_ulschCfg_s    ulschCfg;   /* The ULSCH configuration*/
} TiFapi_ulschPduCfg_s;

/* CQI_RI information */
typedef struct TiFapi_pmiRiPeriodic_st
{
    uint8_t                      dlCqiPmiSize;              /* The size of the DL CQI/PMI in bits in case of rank1 report*/
	TiFapi_periodicRepCtrlType_t controlType;               /* 0 = CQI/PMI  1 = RI */
} TiFapi_pmiRiPeriodic_s;

#define TI_FAPI_CQI_RI_SIZE_ARRAY_LENGTH    8
#define TI_FAPI_CQI_RI_MAX_NUM_CC           5

typedef struct TiFapi_pmiRiAperiodic_st
{
    uint8_t                  dlCqiPmiSizeRank[TI_FAPI_CQI_RI_MAX_NUM_CC][TI_FAPI_CQI_RI_SIZE_ARRAY_LENGTH]; /* The size of the DL CQI/PMI in bits in case of rank=x report*/
    uint8_t                  numOfCc;                                            /* The number of CC in the aperiodic report */   
	uint8_t                  riSize[TI_FAPI_CQI_RI_MAX_NUM_CC];                  /* The size of RI in bits*/
} TiFapi_pmiRiAperiodic_s;

typedef union 
{
    TiFapi_pmiRiPeriodic_s    periodicReport;
    TiFapi_pmiRiAperiodic_s   aperiodicReport;
} TiFapi_pmiRi_u;

typedef struct TiFapi_cqiRiCfg_st
{
	TiFapi_csiReportType_t   reportType;                 /* Type of CSI report */
    uint8_t                  deltaOffsetCqi;             /* Delta Offset Cqi*/
    uint8_t                  deltaOffsetRi;              /* Delta Offset RI*/
	TiFapi_pmiRi_u           pmiRi;
} TiFapi_cqiRiCfg_s;

typedef struct TiFapi_initTxCfg_st
{
    TiFapi_lastOfdmSymbolPunctured_t    nSrsInitial;        /* The OFDM symbol puncturing*/
    uint8_t                             initialNumRbs;      /* Initial number of resource blocks*/
} TiFapi_initTxCfg_s;

/* ULSCH_CQI_RI PDU */
typedef struct TiFapi_ulschCqiRiPduCfg_st
{
    TiFapi_ulschCfg_s    ulschCfg;   /* The ULSCH configuration*/
    TiFapi_cqiRiCfg_s    cqiRiCfg;   /* The CQI + RI configuration*/
    TiFapi_initTxCfg_s   initTxCfg;  /* The initial TX configuration*/
} TiFapi_ulschCqiRiPduCfg_s;

typedef struct TiFapi_harqCfg_st
{
    uint8_t harqSize;                   /* The size of the ACK/NACK in bits*/
    uint8_t deltaOffsetHarq;            /* Delta Offset Cqi*/
    uint8_t ackNackMode;                /* TDD only*/
} TiFapi_harqCfg_s;

/* ULSCH_HARQ PDU */
typedef struct TiFapi_ulschHarqPduCfg_st
{
    TiFapi_ulschCfg_s    ulschCfg;   /* The ULSCH configuration*/
    TiFapi_harqCfg_s     harqCfg;    /* The HARQ configuration*/
    TiFapi_initTxCfg_s   initTxCfg;  /* The initial TX configuration*/
} TiFapi_ulschHarqPduCfg_s;

/* ULSCH_CQI_HARQ_RI PDU */
typedef struct TiFapi_ulschCqiHarqRiPduCfg_st
{
    TiFapi_ulschCfg_s    ulschCfg;   /* The ULSCH configuration*/
    TiFapi_cqiRiCfg_s    cqiRiCfg;   /* The CQI + RI configuration*/
    TiFapi_harqCfg_s     harqCfg;    /* The HARQ configuration*/
    TiFapi_initTxCfg_s   initTxCfg;  /* The initial TX configuration*/
} TiFapi_ulschCqiHarqRiPduCfg_s;


/*****UCI*******/

typedef struct TiFapi_uciCqiCfg_st
{
    uint16_t  pucchIndex2;    /* PUCCH index 2*/
    uint8_t   dlCqiSize;      /* The size of the DL CQI in bits*/
	uint8_t   numPucchRes;    /* A value of 2 indicates that UE is configured to transmit on two antenna ports*/
	uint16_t  pucchIndexP1;   /* PUCCH index 2 for antenna port P1.*/
} TiFapi_uciCqiCfg_s;

/* UCI_CQI PDU */
typedef struct TiFapi_uciCqiPduCfg_st
{
    uint32_t              handle;     /* An opaque handle that will be returned to L23*/
    uint16_t              rnti;       /* The RNTI used for identifying the UE when receiving the PDU*/
    TiFapi_uciCqiCfg_s    cqiCfg;     /* The CQI configuration*/
} TiFapi_uciCqiPduCfg_s;

typedef struct TiFapi_uciSrCfg_st
{
    uint16_t  pucchIndex1;    /* PUCCH index 1*/
	uint8_t   numPucchRes;    /* A value of 2 indicates that UE is configured to transmit on two antenna ports*/
	uint16_t  pucchIndexP1;   /* PUCCH index 2 for antenna port P1.*/
} TiFapi_uciSrCfg_s;

/* UCI_SR PDU */
typedef struct TiFapi_uciSrPduCfg_st
{
    uint32_t              handle;     /* An opaque handle that will be returned to L23*/
    uint16_t              rnti;       /* The RNTI used for identifying the UE when receiving the PDU*/
    TiFapi_uciSrCfg_s     srCfg;      /* The CQI configuration*/
} TiFapi_uciSrPduCfg_s;

/* HARQ config structure contains a superset of all parameters valid for TDD and FDD.
   The parameters that are invalid for the current duplexing mode should be set to zero.*/
typedef struct TiFapi_uciHarqCfg_st
{
    uint8_t                    harqSize;       /* The size of the ACK/NACK in bits - Valid for both FDD and TDD*/
	                                           /* The HARQ size should be set assuming that the SR is not present (negative). 
											      For TDD HARQ bundling possible values for this field are 1 or 2 bits depending on the number of DLSCH PDUs sent. 
												  For TDD multiplexing the only possible value is 2. */
    uint8_t                    ackNackMode;    /* Format of the ack/nack response*/
    uint8_t                    numPucchRes;    /* The number of ack/nack responses*/
    uint16_t                   n_PUCCH_1_0;    /* HARQ resource 0 */
    uint16_t                   n_PUCCH_1_1;    /* HARQ resource 1*/
    uint16_t                   n_PUCCH_1_2;    /* HARQ resource 2*/
    uint16_t                   n_PUCCH_1_3;    /* HARQ resource 3*/
} TiFapi_uciHarqCfg_s;


/* UCI_HARQ PDU */
typedef struct TiFapi_uciHarqPduCfg_st
{
    uint32_t              handle;     /* An opaque handle that will be returned to L23*/
    uint16_t              rnti;       /* The RNTI used for identifying the UE when receiving the PDU*/
    TiFapi_uciHarqCfg_s   harqCfg;    /* The HARQ configuration*/
} TiFapi_uciHarqPduCfg_s;

/*UCI_SR_HARQ PDU */
typedef struct TiFapi_uciSrHarqPduCfg_st
{
    uint32_t              handle;     /* An opaque handle that will be returned to L23*/
    uint16_t              rnti;       /* The RNTI used for identifying the UE when receiving the PDU*/
    TiFapi_uciSrCfg_s     srCfg;      /* The SR configuration*/
    TiFapi_uciHarqCfg_s   harqCfg;    /* The HARQ configuration*/
} TiFapi_uciSrHarqPduCfg_s;

/*UCI_CQI_HARQ PDU */
typedef struct TiFapi_uciCqiHarqPduCfg_st
{
    uint32_t              handle;     /* An opaque handle that will be returned to L23*/
    uint16_t              rnti;       /* The RNTI used for identifying the UE when receiving the PDU*/
    TiFapi_uciCqiCfg_s    cqiCfg;     /* The CQI configuration*/
    TiFapi_uciHarqCfg_s   harqCfg;    /* The HARQ configuration*/
} TiFapi_uciCqiHarqPduCfg_s;

/*UCI_CQI_SR PDU */
typedef struct TiFapi_uciCqiSrPduCfg_st
{
    uint32_t              handle;     /* An opaque handle that will be returned to L23*/
    uint16_t              rnti;       /* The RNTI used for identifying the UE when receiving the PDU*/
    TiFapi_uciCqiCfg_s    cqiCfg;     /* The CQI configuration*/
    TiFapi_uciSrCfg_s     srCfg;      /* The SR configuration*/
} TiFapi_uciCqiSrPduCfg_s;

/*UCI_CQI_SR_HARQ PDU */
typedef struct TiFapi_uciCqiSrHarqPduCfg_st
{
    uint32_t              handle;     /* An opaque handle that will be returned to L23*/
    uint16_t              rnti;       /* The RNTI used for identifying the UE when receiving the PDU*/
    TiFapi_uciCqiCfg_s    cqiCfg;     /* The CQI configuration*/
    TiFapi_uciSrCfg_s     srCfg;      /* The SR configuration*/
    TiFapi_uciHarqCfg_s   harqCfg;    /* The HARQ configuration*/
} TiFapi_uciCqiSrHarqPduCfg_s;

/* SRS PDU */
typedef struct TiFapi_srsPduCfg_st
{
    uint32_t                 handle;                     /* An opaque handle that will be returned to L23*/
    uint16_t                 size;                       /* The size of the PDU in bytes. Can be 0 in case that only HARQ and CQI is received on ULSCH*/
    uint16_t                 rnti;                       /* The RNTI used for identifying the UE when receiving the PDU*/
    uint8_t                  srsBandwidth;               /* SRS Bandwidth*/
    uint8_t                  freqDomainPosition;         /* Frequency domain position ("b")*/
    uint8_t                  srsHoppingBw;               /* SRS hopping bandwidth*/
    uint8_t                  txComb;                     /* Value: 0,1*/
    uint16_t                 srsPeriodicity;             /* Periodicity of the SRS transmission in order to allow correct SRS hopping in PHY*/
    uint8_t                  soundingRefCyclicShift;     /* SRS cyclic shift*/
	TiFapi_numSrsAntPort_t   antennaPort;                /* Number of antenna ports used by UE for the SRS*/
} TiFapi_srsPduCfg_s;

/* ULSCH_UCI_CSI PDU */
typedef struct TiFapi_ulschUciCsiPduCfg_st
{
    TiFapi_ulschCfg_s    ulschCfg;   /* The ULSCH configuration*/
    TiFapi_uciCqiCfg_s   cqiCfg;     /* The UCI CQI configuration*/
} TiFapi_ulschUciCsiPduCfg_s;

/* ULSCH_UCI_HARQ PDU */
typedef struct TiFapi_ulschUciHarqPduCfg_st
{
    TiFapi_ulschCfg_s    ulschCfg;   /* The ULSCH configuration*/
    TiFapi_uciHarqCfg_s  harqCfg;    /* The HARQ configuration*/
} TiFapi_ulschUciHarqPduCfg_s;

/* ULSCH_CSI_UCI_HARQ PDU */
typedef struct TiFapi_ulschUciCsiHarqPduCfg_st
{
    TiFapi_ulschCfg_s    ulschCfg;   /* The ULSCH configuration*/
    TiFapi_cqiRiCfg_s    csiCfg;     /* The CSI configuration*/
    TiFapi_uciHarqCfg_s  harqCfg;    /* The HARQ configuration*/
} TiFapi_ulschUciCsiHarqPduCfg_s;

/* ULSCH PDU types */
typedef union 
{
	TiFapi_ulschPduCfg_s             ulsch;          /* The ULSCH PDU configuration*/
	TiFapi_ulschCqiRiPduCfg_s        ulschCqiRi;     /* The ULSCH + CQI + RI PDU configuration*/
	TiFapi_ulschHarqPduCfg_s         ulschHarq;      /* The ULSCH + HARQ PDU configuration*/
	TiFapi_ulschCqiHarqRiPduCfg_s    ulschCqiHarqRi; /* The ULSCH + CQI + RI + HARQ PDU configuration uration*/
	TiFapi_ulschUciCsiPduCfg_s       ulschUciCSi;    /* The ULSCH + UCI CSI*/
	TiFapi_ulschUciHarqPduCfg_s      ulschUciHarq;   /* The ULSCH + UCI HARQ*/
	TiFapi_ulschUciCsiHarqPduCfg_s   ulschUciCsiHarq;/* The ULSCH + UCI CSI + UCI HARQ*/
}TiFapi_ulschPdu_u;

/* POSSIBLE ULSCH PDU TYPES*/
typedef struct TiFapi_ulschPdu_st
{
    TiFapi_ulPduType_t pduType; /* The PDU type*/
	TiFapi_ulschPdu_u  u;       /* Union for the different ULSCH PDU combinations*/
}TiFapi_ulschPdu_s;

/* UCI PDU types */
typedef union 
{
	TiFapi_uciCqiPduCfg_s            cqi;            /* The CQI PDU configuration*/
	TiFapi_uciSrPduCfg_s             sr;             /* The SR PDU configuration*/
	TiFapi_uciHarqPduCfg_s           harq;           /* The HARQ PDU configuration*/
	TiFapi_uciSrHarqPduCfg_s         srHarq;         /* The SR + HARQ PDU configuration*/
	TiFapi_uciCqiHarqPduCfg_s        cqiHarq;        /* The CQI + HARQ PDU configuration*/
	TiFapi_uciCqiSrPduCfg_s          cqiSr;          /* The CQI + SR PDU configuration*/
	TiFapi_uciCqiSrHarqPduCfg_s      cqiSrHarq;      /* The CQI + SR + HARQ PDU configuration*/
}TiFapi_uciPdu_u;

/* POSSIBLE UCI PDU TYPES*/
typedef struct TiFapi_uciPdu_st
{
    TiFapi_ulPduType_t pduType; /* The PDU type*/
	TiFapi_uciPdu_u    u;       /* Union for the different UCI PDU combinations*/
}TiFapi_uciPdu_s;

/* SRS PDU types */
typedef union 
{
    TiFapi_srsPduCfg_s   srs;    /* The SRS PDU configuration*/
}TiFapi_srsPdu_u;

/* POSSIBLE SRS PDU TYPES*/
typedef struct TiFapi_srsPdu_st
{
    TiFapi_ulPduType_t pduType; /* The PDU type*/
	TiFapi_srsPdu_u    u;       /* Union for the different SRS PDU combinations*/
}TiFapi_srsPdu_s;

/* UL CONFIG REQUEST */
typedef struct TiFapi_ulCfgReq_st
{
    uint16_t          sfnSf;                                          /* SFN/SF*/
    uint8_t           numUlschPdus;                                   /* The number of PDUs in the ulsch array*/
    uint8_t           numUciPdus;                                     /* The number of PDUs in the uci array*/
    uint8_t           numSrsPdus;                                     /* The number of PDUs in the srs array*/
    uint8_t           prachResource;                                  /* For FDD 0 means no RACH in this sf, 1 means rach present in this sf;
                                                                         for TDD bits 0:5 indicates the RACH resources used*/
    uint8_t           srsPresent;                                     /* The SRS present flag*/
    TiFapi_ulschPdu_s ulsch[TI_FAPI_MAX_NUM_ULSCH_PDUS];              /* Array of ULSCH PDU configurations*/
	TiFapi_uciPdu_s	  uci[TI_FAPI_MAX_NUM_UCI_PDUS];                  /* Array of UCI PDU configurations*/
	TiFapi_srsPdu_s   srs[TI_FAPI_MAX_NUM_SRS_PDUS];                  /* Array of SRS PDU configurations*/
}TiFapi_ulCfgReq_s;

/****************************************
SUBFRAME.indication
****************************************/
typedef struct TiFapi_sfInd_st 
{
    uint16_t sfnSf;       /* SFN/SF*/
}TiFapi_sfInd_s;

/****************************************
HI_DCI0.request
****************************************/

/* HI PDU*/
typedef struct TiFapi_hiPduCfg_st 
{
    /* The starting resource block assigned to the ULSCH grant associated with this HI response.*/
    uint8_t                 resourceBlockStart;
    /* Second cyclic shift for DRMS assigned to the ULSCH grant associated with this HI response.*/
    uint8_t                 cyclicShift2Dmrs;
    /* The PHICH value which is sent on the resource.*/
    TiFapi_hiValue_t        hiValue;
    /* Is used in the calculation of the PHICH location. For TDD only.*/
    uint8_t                 iPhich;
    /* Offset to the reference signal power.*/
    uint16_t                txPower;
	/*Indicates if HI is present for a second transport block*/
    TiFapi_hiFlagTb2_t      flagTb2;
	/*The PHICH value for a second transport block*/
    TiFapi_hiValue_t        hiValue2;
}TiFapi_hiPduCfg_s;

/* DCI0 PDU */
typedef struct TiFapi_dci0PduCfg_st 
{
    TiFapi_ulDciFormat_t             dciFormat;              /* Format of the UL DCI.*/
    uint8_t                          cceIndex;               /* The CCE index used to send the DCI.*/
    uint8_t                          aggregationLevel;       /* The aggregation level.*/
    uint16_t                         rnti;                   /* The RNTI used for identifying the UE when receiving the PDU.*/
    uint8_t                          rbStart;                /* The starting resource block for this ULSCH allocation.*/
    uint8_t                          numRbs;                 /* The number of resource blocks allocated to this ULSCH grant.*/
    uint8_t                          mcs;                    /* The modulation and redundancy version.*/
    uint8_t                          cyclicShift2Dmrs;       /* Second cyclic shift for DRMS assigned to the UE in the ULSCH grant.*/
    TiFapi_frequencyHoppingFlag_t    frequencyEnabledFlag;   /* Indicates if hopping is being used.*/
    uint8_t                          frequencyHopping;       /* The frequency hopping bits.*/
    uint8_t                          ndi;                    /* The new data indicator for the transport block.*/
    TiFapi_ueTxAntennaSelection_t    ueTxAntennaSelection;   /* Indicates how the CRC is calculated on the PDCCH.*/
    TiFapi_tpcValue_t                tpc;                    /* Tx power control command for PUSCH.*/
    TiFapi_cqiRequest_t              cqiRequest;             /* Aperiodic CQI request flag.*/
    uint8_t                          ulIdx;                  /* UL index. Valid for TDD only.*/
    uint8_t                          dlAssignmentIdx;        /* DL assignment index. Valid for TDD only.*/
    uint32_t                         tpcBitmap;              /* TPC commands for PUCCH and PUSCH.*/
    TiFapi_crossCarrierFlag_t        crossCarrierFlag;       /* Indicates if cross carrier scheduling has been enabled for UE*/
    uint8_t                          carrierIndicator;       /* Serving cell index*/
    TiFapi_ulDciCqiSize_t            cqiSize;                /* Indicates the size of the CQI/CSI request field*/
    TiFapi_srsValidFlag_t            srsFlag;                /* SRS valid flag*/
    uint8_t                          srsRequest;             /* SRS request */
    TiFapi_resAllcFlag_t             resourceAllocationFlag; /* Resource allocation flag*/	
    TiFapi_resAllcType_t             resourceAllocationType; /* Resource allocation type*/	
    uint32_t                         resourceBlockCoding;    /* Resource block coding. */
    uint8_t                          mcs2;                   /* The modulation and redundancy version for second transport block.*/
    uint8_t                          ndi2;                   /* The new data indicator for the second transport block.*/
	TiFapi_numUlDciAntPort_t         numAntPort;             /* Defines number of antenna ports for this ULSCH allocation*/
    uint8_t                          tpmi;                   /* The codebook index to be used for precoding*/
    uint16_t                         txPower;                /* Offset to the RS power*/
    uint8_t                          dciLength;              /* The total DCI length including padding bits.*/	
} TiFapi_dci0PduCfg_s;

/* HI_DCI0 REQUEST*/
typedef struct TiFapi_hiDci0Req_st
{
    uint16_t sfnSf;                    /* SFN/SF*/
    uint8_t  numUlDciPdus;             /* The number of DCI 0 PDUs for this TTI*/
    uint8_t  numHiPdus;                /* The number of HI PDUs for this TTI*/
    TiFapi_hiPduCfg_s    hi[TI_FAPI_MAX_NUM_HI_PDUS];           /* Array of PHICH configurations*/
    TiFapi_dci0PduCfg_s  uldci[TI_FAPI_MAX_NUM_UL_DCI_PDUS];    /* The DCI0 PDU configuration*/
}TiFapi_hiDci0Req_s;

/****************************************
HARQ.indication
****************************************/
typedef struct TiFapi_harqBundling_st
{
    TiFapi_harqFeedback_t value0; /* HARQ feedback */
}TiFapi_harqBundling_s;

typedef struct TiFapi_harqMultiplexing_st
{
    TiFapi_harqFeedback_t value0; /* HARQ feedback */
}TiFapi_harqMultiplexing_s;

typedef struct TiFapi_harqSplBundling_st
{
    TiFapi_harqFeedbackSpecBundMode_t value0; /* HARQ feedback */
}TiFapi_harqSplBundling_s;

typedef struct TiFapi_harqChSelection_st
{
    TiFapi_harqFeedback_t value0; /* HARQ feedback */
}TiFapi_harqChSelection_s;

typedef struct TiFapi_harqFormat3_st
{
    TiFapi_harqFeedback_t value0; /* HARQ feedback */
}TiFapi_harqFormat3_s;

typedef struct TiFapi_harqFddData_st
{
    TiFapi_harqFeedback_t harqTbN; /* HARQ feedback of Nth TB - FDD only */
}TiFapi_harqFddData_s;

typedef union 
{
    TiFapi_harqMultiplexing_s harqMultiplexing; /*TDD only*/
    TiFapi_harqBundling_s     harqBundling;     /*TDD only*/
    TiFapi_harqSplBundling_s  harqSplBundling;  /*TDD only*/
	TiFapi_harqChSelection_s  harqChSelection;  /*TDD only*/
	TiFapi_harqFormat3_s      harqFormat3;      /*TDD only*/
    TiFapi_harqFddData_s      harqFdd;          /*FDD only*/
} TiFapi_harqData_u;

#define TI_FAPI_HARQ_IND_MAX_NUM_ACK_NACK    20
typedef struct TiFapi_harqPduInd_st
{
    uint32_t                  handle;                                      /* The handle received in the HARQ PDU.*/
    uint16_t                  rnti;                                        /* The RA-RNTI value.*/
	uint8_t                   pucchF1Sinr;                                 /* PUCCH Format_1 SINR, values from 0 to 255, representing -64dB to 63.5dB, with 0.5dBstep.*/	
    TiFapi_harqAckNackMode_t  mode;                                        /* The format of the ack/nack response expected */
    uint8_t                   numAckNack;                                  /* The number of ack/nack reported for this UE */
    TiFapi_harqData_u         harqData[TI_FAPI_HARQ_IND_MAX_NUM_ACK_NACK]; /* HARQ data*/
} TiFapi_harqPduInd_s;

typedef struct TiFapi_harqInd_st
{
    /* SFN/SF */
    uint16_t sfnSf;
    /* The number of HARQ PDUs for this TTI */
    uint16_t numHarqPdus;
    /* Array of HARQ responses, one for each UE */
    TiFapi_harqPduInd_s harq[TI_FAPI_MAX_NUM_HARQ_PDUS];
}TiFapi_harqInd_s;

/****************************************
RACH.indication
****************************************/
typedef struct TiFapi_rachPduCfg_st 
{
    uint16_t  rnti;                       /*The RA-RNTI value.*/
    uint8_t   preamble;                   /*The detected preamble.*/
	uint8_t   sinr;                       /*PRACH SINR, values from 0 to 255, representing -64dB to 63.5dB, with 0.5dB step.*/
    uint16_t  timingAdvance;              /*The timing advance measured for this preamble.*/	
    uint16_t  timingAdvancePositioning;   /*The timing advance used for positioning.*/
    uint32_t  power;                      /*Linear power*/	
}TiFapi_rachPduCfg_s;

typedef struct TiFapi_rachInd_st
{
    /* SFN/SF*/
    uint16_t sfnSf;
    /* The number of RACH preambles*/
    uint16_t numOfPreamble;
    /* Array of RACH outputs, one for each UE*/
    TiFapi_rachPduCfg_s rach[TI_FAPI_MAX_NUM_RACH_PREAMBLES];
}TiFapi_rachInd_s;


/****************************************
SRS.indication
****************************************/
typedef struct TiFapi_srsIndPduCfg_st
{
    uint32_t  handle;                                 /* The handle received in the SRS PDU.*/
    uint16_t  rnti;                                   /* The RNTI passed to the PHY in an uplink subframe configuration PDU.*/
    uint16_t  dopplerEstimation;                      /* Values 0 to 255.*/
    uint16_t  timingAdvance;                          /* The timing advance measured for this UE.*/	
    uint16_t  timingAdvancePositioning;               /* The timing advance used for positioning.*/
    uint8_t   numRbs;                                 /* Number of RBs to be reported for this UE.*/
    uint8_t   rbStart;                                /* The starting point of the RBs to be reported.*/
    uint8_t   snr[TI_FAPI_MAX_NUM_SRS_RBS_REPORTED_PER_UE];   /* SNR for RBs.*/
    uint16_t  srsPower[TI_FAPI_MAX_NUM_SRS_RBS_REPORTED_PER_UE];   /* SRS power per RB (dBm)*/
} TiFapi_srsIndPduCfg_s;

typedef struct TiFapi_srsInd_st
{
    /* SFN/SF */
    uint16_t sfnSf;
    /* The number of SRS PDUs for this TTI*/
    uint16_t numSrsPdus;
    /* Array of SRS outputs, one for each UE*/
    TiFapi_srsIndPduCfg_s srs[TI_FAPI_MAX_NUM_SRS_PDUS];
}TiFapi_srsInd_s;

/****************************************
CRC.indication
****************************************/
typedef struct TiFapi_crcPduCfg_st
{
    uint32_t  handle;                     /* The handle */
    uint16_t  rnti;                       /* The RNTI identifying the UE*/
    uint8_t   crcFlag;                    /* CRC flag*/
}TiFapi_crcPduCfg_s;

typedef struct TiFapi_crcInd_st
{
    /* SFN/SF*/
    uint16_t sfnSf;
    /* The number of CRCs*/
    uint16_t numCrcs;
    /* Array of CRCs */
    TiFapi_crcPduCfg_s crc[TI_FAPI_MAX_NUM_CRC_PDUS];
}TiFapi_crcInd_s;

/****************************************
RX_SR.indication
****************************************/
typedef struct TiFapi_rxSrPduInd_st
{
    uint32_t  handle;                     
    uint16_t  rnti;                       
	uint8_t   pucchF1Sinr;/*PUCCH Format_1 SINR, values from 0 to 255, representing -64dB to 63.5dB, with 0.5dBstep.*/	
}TiFapi_rxSrPduInd_s;

typedef struct TiFapi_rxSrInd_st
{
    /* SFN/SF */
    uint16_t sfnSf;
    /* The number of Scheduling Request PDUs for this TTI */
    uint16_t numOfSr;
    /* Array of Scheduling Request outputs, one for each UE*/
    TiFapi_rxSrPduInd_s sr[TI_FAPI_MAX_NUM_SR_PDUS];
}TiFapi_rxSrInd_s;


/****************************************
RX_CQI.indication
****************************************/
#define TI_FAPI_CQI_IND_MAX_NUM_CC           5

typedef struct TiFapi_cqiPduCfg_st
{
    uint32_t  handle;                     /* The handle received in the ULSCH or UCI PDU*/
    uint16_t  rnti;                       /* The RNTI identifying the UE*/
    uint16_t  length;                     /* The length of the CQI payload in bytes*/
    uint16_t  dataOffset;                 /* Data offset*/
    uint8_t   ulCqi;                      /* The SNR, values from 0 to 255*/
    uint8_t   numOfCc;                             /* Number of CC reported */
	uint8_t   riArray[TI_FAPI_CQI_IND_MAX_NUM_CC]; /* RI reported by UE*/
    uint16_t  timingAdvance;              /* The timing advance measured for this PDU and UE*/
    uint16_t  timingAdvancePositioning;   /* The timing advance used for positioning*/
    uint8_t   cqiPayload[TI_FAPI_CQI_IND_MAX_NUM_CC*TI_FAPI_MAX_CQI_PAYLOAD]; /* The decoded CQI payload for all CCs*/
}TiFapi_cqiPduCfg_s;

typedef struct TiFapi_cqiInd_st
{
    /* SFN/SF*/
    uint16_t sfnSf;
    /* The number of CQI PDUs for this TTI*/
    uint16_t numCqiPdus;
    /* Array of CQI outputs, one for each UE*/
    TiFapi_cqiPduCfg_s cqi[TI_FAPI_MAX_NUM_CQI_PDUS];
}TiFapi_cqiInd_s;


/******************************************************************************
*
*                Subframe Messages - with data (non-TLV based)                                    
*
******************************************************************************/

/****************************************
TX.request
****************************************/

typedef struct TiFapi_txDataBuf_st
{
    /* Pointer to the data buffer. This is a pointer stored as a 32bit value.*/
    uint32_t  bufBasePtr;
	/* Length of the data buffer in bytes */
    uint32_t  bufLen;
}TiFapi_txDataBuf_s;

typedef struct TiFapi_txReqPdu_st
{
    /* PDU index: This is a count value which starts from 0. 
	   It is incremented for each BCH, MCH, PCH or DLSCH PDU. */
    uint16_t pduIndex;
    /* Number of scattered data buffers that make one PDU*/
    uint8_t numScatBuf;
    /* Scattered data buffers that make one PDU */
    TiFapi_txDataBuf_s data[TI_FAPI_MAX_NUM_TX_REQ_SCATTERED_BUF];
} TiFapi_txReqPdu_s;

typedef struct TiFapi_txReqEmbeddedPdu_st
{
    uint16_t pduLength; /*Length in bytes of the MAC PDU*/
    uint16_t pduIndex;  /*PDU index as specified in the DL_CONFIG.request*/
} TiFapi_txReqEmbeddedPdu_s;

typedef union 
{
    TiFapi_txReqPdu_s         pdu[TI_FAPI_MAX_NUM_TX_REQUEST_PDUS]; /*MAC PDU passed by reference*/
    TiFapi_txReqEmbeddedPdu_s pduInfo;                              /*MAC PDU embedded in TX.request message*/
}TiFapi_txReqPdu_u;

typedef struct TiFapi_txReq_st 
{
    /* SFN/SF */
    uint16_t sfnSf;
    /* Number of PDUs*/
    uint16_t numPdus;
    /* PDUs */
    TiFapi_txReqPdu_u u;
} TiFapi_txReq_s;


/****************************************
RX_ULSCH.indication
****************************************/

typedef struct TiFapi_rxDataBuf_st
{
    /* Pointer to buffer descriptor for the received MAC PDU. This is a pointer stored as a 32bit value.*/
    uint32_t  bufDescriptorPtr;
}TiFapi_rxDataBuf_s;

typedef struct TiFapi_rxUlschPdu_st
{

    uint32_t                 handle;           /*The handle received with the configuration of the ULSCH PDU.*/
    uint16_t                 rnti;             /*The RNTI received with the configuration of the ULSCH PDU.*/
    uint16_t                 length;           /*Length of PDU */
    TiFapi_dataOffsetFlag_t  dataOffset;       /*Data offset. Set to ERROR in case of CRC or decoding error. Set to OK otherwise. Do not use real offset as the buffers are scatered. */
    uint8_t                  ulCqi;            /*The SNR*/
    uint16_t                 timingAdvance;    /*The timing advance measured for this PDU and UE.*/	
    uint16_t                 timingAdvancePositioning; /*The timing advance used for positioning.*/	
    TiFapi_rxDataBuf_s       data;             /*Descriptor for this MAC PDU. */
                                               /*If L2 resides on ARM, the 'data' field is not used.
                                                 In this case, the MAC PDU is transmitted over the channel TI_FAPI_L1L2_MAC_PDU_CHANNEL_NAME*/
} TiFapi_rxUlschPdu_s;

typedef struct TiFapi_rxUlschEmbeddedPdu_st
{

    uint32_t                 handle;           /*The handle received with the configuration of the ULSCH PDU.*/
    uint16_t                 rnti;             /*The RNTI received with the configuration of the ULSCH PDU.*/
    uint16_t                 length;           /*Length of PDU */
    TiFapi_dataOffsetFlag_t  dataOffset;       /*Data offset. Set to ERROR in case of CRC or decoding error. Set to OK otherwise. Do not use real offset as the buffers are scatered. */
    uint8_t                  ulCqi;            /*The SNR*/
    uint16_t                 timingAdvance;    /*The timing advance measured for this PDU and UE.*/	
    uint16_t                 timingAdvancePositioning; /*The timing advance used for positioning.*/	
} TiFapi_rxUlschEmbeddedPdu_s;

typedef union 
{
	TiFapi_rxUlschPdu_s         pdu[TI_FAPI_MAX_NUM_RX_ULSCH_IND_PDUS]; /*MAC PDU passed by reference*/
	TiFapi_rxUlschEmbeddedPdu_s pduInfo;                                /*MAC PDU embedded in RX_ULSCH.indication message*/
}TiFapi_rxUlschPdu_u;

typedef struct TiFapi_rxUlschInd_st
{
    /* SFN/SF */
    uint16_t sfnSf;                  
    /* Number of PDUs*/
    uint16_t numPdus;
    /* PDUs*/
	TiFapi_rxUlschPdu_u u;    
}TiFapi_rxUlschInd_s;


/******************************************************************************
*
*    Subframe Messages - header and generic msg structure (non-TLV based)                                    
*
******************************************************************************/

/* Message header */
typedef struct TiFapi_nonTlvMsgHeader_st
{
    /* Message type ID */
    TiFapi_msgIdType_t   msgId;
    /* Length of vendor-specific message body (bytes) */
    uint8_t              lenVendorSpecific; 
    /* Length of message body (bytes) */
    uint16_t             msgLen;
}TiFapi_nonTlvMsgHeader_s;

/* Message body */
typedef union 
{
    TiFapi_dlCfgReq_s   dlCfgReq;
    TiFapi_ulCfgReq_s   ulCfgReq;
	TiFapi_sfInd_s      sfInd;
	TiFapi_hiDci0Req_s  hiDci0Req;
	TiFapi_harqInd_s    harqInd;
    TiFapi_rachInd_s    rachInd;
    TiFapi_srsInd_s     srsInd;
	TiFapi_crcInd_s		crcInd;
    TiFapi_rxSrInd_s    rxSrInd;
    TiFapi_cqiInd_s     cqiInd;
    TiFapi_txReq_s      txReq;
    TiFapi_rxUlschInd_s rxUlschInd;
}TiFapi_nonTlvMsgPayload_u;

/* Message must start at a 32bit boundary*/
typedef struct TiFapi_nonTlvMsg_st
{
    /* Message Header */
    TiFapi_nonTlvMsgHeader_s  hdr;
	/* Message Body */
	TiFapi_nonTlvMsgPayload_u msg;
}TiFapi_nonTlvMsg_s;


#endif /* TI_FAPI_H_ */

