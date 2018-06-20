/******************************************************************************
*
*  ti_fapiChanNames.h
*
*  (C) Copyright 2014, Texas Instruments, Inc.
*
*
******************************************************************************/

#ifndef TI_FAPI_CHANNAME_H
#define TI_FAPI_CHANNAME_H

/******************************************************************************
* Release 10:
*                PHY <-> L2 MsgCom channels
*
******************************************************************************/

/* Low priority MsgCom channel name for cellA L1 -> L2 */
#define TI_FAPI_CELLA_L1L2_LP_CHANNEL_NAME            "TiFapi_L1L2LP_A"

/*  High priority MsgCom channel name for cellA L1 -> L2 */
#define TI_FAPI_CELLA_L1L2_HP_CHANNEL_NAME            "TiFapi_L1L2HP_A"

/*  Low priority MsgCom channel name for cellA L2 -> L1*/
#define TI_FAPI_CELLA_L2L1_LP_CHANNEL_NAME            "TiFapi_L2L1LP_A"

/*  High priority MsgCom channel name for cellA L2 -> L1*/
#define TI_FAPI_CELLA_L2L1_HP_CHANNEL_NAME            "TiFapi_L2L1HP_A"

/* MsgCom channel name for transmitting MAC PDUs from L1 -> L2 
   Used only if L2 resides on ARM */
#define TI_FAPI_CELLA_L1L2_MAC_PDU_CHANNEL_NAME       "TiFapi_L1L2MACPDU_A"

/* MsgCom channel name for transmitting MAC PDUs from L2 -> L1 
   Used only if L2 resides on ARM */
#define TI_FAPI_CELLA_L2L1_MAC_PDU_CHANNEL_NAME       "TiFapi_L2L1MACPDU_A"

/* Low priority MsgCom channel name for cellB  L1 -> L2 */
#define TI_FAPI_CELLB_L1L2_LP_CHANNEL_NAME            "TiFapi_L1L2LP_B"

/*  High priority MsgCom channel name for cellB L1 -> L2 */
#define TI_FAPI_CELLB_L1L2_HP_CHANNEL_NAME            "TiFapi_L1L2HP_B"

/*  Low priority MsgCom channel name for cellB L2 -> L1*/
#define TI_FAPI_CELLB_L2L1_LP_CHANNEL_NAME            "TiFapi_L2L1LP_B"

/*  High priority MsgCom channel name for cellB L2 -> L1*/
#define TI_FAPI_CELLB_L2L1_HP_CHANNEL_NAME            "TiFapi_L2L1HP_B"

/* MsgCom channel name for transmitting MAC PDUs from L1 -> L2 
   Used only if L2 resides on ARM */
#define TI_FAPI_CELLB_L1L2_MAC_PDU_CHANNEL_NAME       "TiFapi_L1L2MACPDU_B"

/* MsgCom channel name for transmitting MAC PDUs from L2 -> L1 
   Used only if L2 resides on ARM */
#define TI_FAPI_CELLB_L2L1_MAC_PDU_CHANNEL_NAME       "TiFapi_L2L1MACPDU_B"


/* Heap Names for multiple cells. */
#define TI_FAPI_CELLA_ULSCH_DATA_HEAP_NAME            "TiFapi_UlschDataHeapA"
#define TI_FAPI_CELLB_ULSCH_DATA_HEAP_NAME            "TiFapi_UlschDataHeapB"
#define TI_FAPI_CELLA_MSG_HEAP_DDR_NAME               "TiFapi_MsgHeapDdrA"
#define TI_FAPI_CELLB_MSG_HEAP_DDR_NAME               "TiFapi_MsgHeapDdrB"
#define TI_FAPI_CELLA_MSG_HEAP_MSMC_NAME              "TiFapi_MsgHeapMsmcA"
#define TI_FAPI_CELLB_MSG_HEAP_MSMC_NAME              "TiFapi_MsgHeapMsmcB"

#endif /* TI_FAPI_CHANNAME_H */
