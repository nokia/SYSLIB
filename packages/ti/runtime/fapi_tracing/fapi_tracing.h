/**
 *   @file  fapi_tracing.h
 *
 *   @brief
 *      Header file for the FAPI tracing.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2013 Texas Instruments, Inc.
 *  \par
 */

/** @defgroup FAPI_TRACING_API FAPI Tracing API
 */

#ifndef __FAPI_TRACING_H__
#define __FAPI_TRACING_H__

/* SYSLIB Include Files. */
#include <ti/runtime/common/syslib.h>
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/netfp/netfp.h>
#include <ti/runtime/name/name_db.h>
#include <ti/runtime/fapi_tracing/include/ti_fapi.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
@defgroup FAPI_TRACING_FUNCTION  FAPI Tracing API Functions
@ingroup FAPI_TRACING_API
*/
/**
@defgroup FAPI_TRACING_INTERNAL_FUNCTION  FAPI Tracing Internal Functions
@ingroup FAPI_TRACING_API
*/
/**
@defgroup FAPI_TRACING_ERR_CODE  FAPI Tracing Error Codes
@ingroup FAPI_TRACING_API
*/
/**
@defgroup FAPI_TRACING_DATASTRUCT  FAPI Tracing Data Structures
@ingroup FAPI_TRACING_API
*/
/**
@defgroup FAPI_TRACING_OSAL_API FAPI Tracing OSAL Functions
@ingroup FAPI_TRACING_API
*/

/** @addtogroup FAPI_TRACING_ERR_CODE
 *
 * @brief 
 *  Base error code for the FAPI Tracing module is defined in the 
 *  \include ti/runtime/common/syslib.h 
 * 
 @{ */

/**
 * @brief   The error code implies that the arguments being passed to the trace function 
 * is invalid
 */
#define FAPI_TRACING_EINVAL                 (SYSLIB_ERRNO_FAPI_TRACING_BASE - 1)

/**
 * @brief   The error code implies that the tracing function failed because the heaps were
 * empty.
 */
#define FAPI_TRACING_ENOMEM                 (SYSLIB_ERRNO_FAPI_TRACING_BASE - 2)

/**
 * @brief   The error code implies that the tracing function failed because the heaps were
 * empty.
 */
#define FAPI_TRACING_ENOTRANSPOT            (SYSLIB_ERRNO_FAPI_TRACING_BASE - 3)

/**
 * @brief   The error code implies that the tracing function failed because of socket error
 */
#define FAPI_TRACING_ENOSOCKET              (SYSLIB_ERRNO_FAPI_TRACING_BASE - 4)

/**
 * @brief   The error code implies the feature is not implemented 
 */
#define FAPI_TRACING_ENOTIMPLEMENT          (SYSLIB_ERRNO_FAPI_TRACING_BASE - 9)

/**
@}
*/

/** @addtogroup FAPI_TRACING_DATASTRUCT
 @{ */

/**
 * @brief 
 *  Opaque FapiTracing Instance handle
 */
typedef void*   FapiTracing_InstHandle;


/** 
 * @brief 
 *  FAPI tracing transport type
 *
 * @details
 *  Enumeration used to indicate whether logs have to shipped over ethernet using NetFP 
 *  or if the logs are shipped through infrastructure DMA to ARM.
 */
typedef enum Fapi_TracingTransportType
{
    /* Transport type is NetFP. Tracing packet will be sent to Ethernet */
    TRACING_TRANSPORT_NETFP = 0,

    /* Transport tye is capture. Tracing packet will be saved in memory(such as DDR). */
    TRACING_TRANSPORT_CAPTURE
}Fapi_TracingTransportType;

/** 
 * @brief 
 *  FAPI tracing transport configuration
 *
 * @details
 *  Transport Configuration specified to the FAPI tracing library.
 */
typedef struct Fapi_TracingTransportCfg
{
    /**
     * @brief   Transport type.
     */
    Fapi_TracingTransportType transportType;
    
    /**
     * @brief   Netfp client handle for socket creation.
     */
     Netfp_ClientHandle      netfpClientHandle;
    
    /**
     * @brief   Ingress Fast Path Handle.
     */
    Netfp_InboundFPHandle   ingressFastPathHandle;

    /**
     * @brief   Egress Fast Path Handle over which the logs will be shipped.
     */
    Netfp_OutboundFPHandle  egressFastPathHandle;

    /**
     * @brief  This is the socket address family: AF_INET or AF_INET6
     */
    Netfp_SockFamily        sin_family;

    /**
     * @brief   PKTLIB instance handle associated with Fapi Tracing. 
     */  
    Pktlib_InstHandle       pktlibInstHandle;
    /**
     * @brief   Pktlib heap containing bufferless packets for transport. 
     * The logs are linked to these and transmitted using NetFP.
     */
    Pktlib_HeapHandle       heapHandle;

    /**
     * @brief   UDP source port number for the NetFP socket.
     */
    uint16_t                srcUDPPort;

    /**
     * @brief   UDP destination port number for the NetFP socket.
     */
    uint16_t                destUDPPort;

    /**
     * @brief   DSCP value added to the logs' IP headers by NetFP.
     */
    uint8_t                 packetDSCP;

    /**
     * @brief   Application registered POST_ROUTING Hook for memory logging
     */
    Netfp_HookFunction      postRoutingHook;

    /**
     * @brief   Application provided argument used for netfp POST_ROUTING Hook
     */
    uint32_t                postRoutingHookArg;
}Fapi_TracingTransportCfg;

/**
 * @brief
 *  FAPI tracing Configuration
 *
 * @details
 *  The structure holds all the Configurations passed to library at FAPI 
 *  tracing initialization time.
 */
typedef struct Fapi_TracingCfg
{
    /**
     * @brief   FAPI Tracing flag which indicates if the tracing support
     * is enabled or disabled. This can be modified at run time
     */
    uint32_t                  isEnabled;

    /**
     * @brief   FAPI Data Tracing flag which indicates if the tracing support
     * is enabled or disabled. This can be modified at run time
     */
    uint32_t                  isDataTracingEnabled;

    /**
     * @brief   FAPI Data Tracing Transport configuration
     */
    Fapi_TracingTransportCfg  transportCfg;

}Fapi_TracingCfg;


/**
@}
*/

/** @addtogroup FAPI_TRACING_OSAL_API
 @{ */

/**
 *  @b Description
 *  @n
 *      OSAL Allocation API which is used by the FAPI Tracing library to 
 *      allocate memory block to save captured tracing buffers
 *
 *  @param[in]  numBytes 
 *      Number of bytes to be allocated
 *  @param[in]  alignment 
 *      Alignment requirements
 *
 *  @retval
 *      Success -   Pointer to the allocated memory block
 *  @retval
 *      Error   -   NULL
 */
typedef void* (*Osal_FapiTracingMalloc) (uint32_t numBytes, uint32_t alignment);
      
/**
 *  @b Description
 *  @n  
 *      The OSAL API is used to free a memory block of the specified size.
 *
 *  @param[in]  ptr 
 *      Pointer to the memory block that is to be freed.
 *  @param[in]  numBytes
 *      Number of bytes to be freed.
 *
 *  @retval
 *      Not Applicable
 */
typedef void  (*Osal_FapiTracingFree) (void* ptr, uint32_t numBytes);

/**
@}
*/

/** @addtogroup FAPI_TRACING_DATASTRUCT
 @{ */
 
/**
 * @brief 
 *  Instance configuration
 *
 * @details
 *  FAPI Tracing instance configuration.
 */
typedef struct FapiTracing_InstCfg
{
    /**
     * @brief   OSAL Malloc 
     */
    Osal_FapiTracingMalloc                    malloc;

    /**
     * @brief   OSAL Free
     */
    Osal_FapiTracingFree                      free;

}FapiTracing_InstCfg;

/**
@}
*/

/**
 * @brief   This is the FAPI Tracing Version. Versions numbers are encoded in the following 
 * format:
 *  0xAABBCCDD -> Arch (AA); API Changes (BB); Major (CC); Minor (DD)
 */
#define FAPI_TRACING_VERSION_ID                   (0x02030402)

/**
 * @brief   This is the version string which describes the FAPI Tracing 
            library along with the date and build information.
 */
#define FAPI_TRACING_VERSION_STR                  "FAPI Tracing Revision: 02.03.04.02"

/* Instance API: */
FapiTracing_InstHandle FapiTracing_createInstance (FapiTracing_InstCfg const * const ptrInstCfg, int32_t* const errCode);
int32_t FapiTracing_deleteInstance (FapiTracing_InstHandle fapiTracingInstHandle, int32_t* const errCode);

/* Initialization/Deinitialization API: */
int32_t FapiTracing_init(FapiTracing_InstHandle fapiTracingInstHandle, Fapi_TracingCfg const * const ptrFapiTracingCfg, int32_t* const errCode);
int32_t FapiTracing_deinit(FapiTracing_InstHandle fapiTracingInstHandle, int32_t* const errCode);

/* Version API: */
/**
 *  @b Description
 *  @n  
 *      The function is used to return the version information for the FAPI Tracing
 *
 *  \ingroup FAPI_TRACING_FUNCTION
 *
 *  @retval
 *      Version information which identifies the FAPI Tracing library
 */
#ifdef _TMS320C6X
#ifdef __cplusplus
#pragma FUNC_ALWAYS_INLINE;
#else
#pragma FUNC_ALWAYS_INLINE(FapiTracing_getVersion);
#endif
#endif
static inline uint32_t FapiTracing_getVersion(void) {
    return FAPI_TRACING_VERSION_ID;
}

/* Configuration API: */
int32_t FapiTracing_configure(FapiTracing_InstHandle fapiTracingInstHandle, uint8_t const tracing, uint8_t const dataTracing, int32_t* const errCode);

/* Tracing API: */
int32_t FapiTracing_trace(FapiTracing_InstHandle fapiTracingInstHandle, Ti_Pkt const * const ptrPkt, int32_t* const errCode);

/* Compression API: */
/**
 *  @b Description
 *  @n  
 *      The function is used to compress the DL.config FAPI message to remove the 
 *      unnecessary zeroes at the end of the message.
 *
 *  \ingroup FAPI_TRACING_FUNCTION
 *
 *  @param[in]  p_fapiNonTlvMsg
 *      This is a pointer to the DL config packet buffer for compression.
 *
 *  @retval
 *      length of the message body (without fapi message header) after 
 *      compression 
 */
#ifdef _TMS320C6X
#ifdef __cplusplus
#pragma FUNC_ALWAYS_INLINE;
#else
#pragma FUNC_ALWAYS_INLINE(FapiTracing_compressDlConfig);
#endif
#endif
static inline uint32_t FapiTracing_compressDlConfig(TiFapi_nonTlvMsg_s const * const p_fapiNonTlvMsg) {
    uint32_t        bufLen = 0;

    if(NULL != p_fapiNonTlvMsg)
    {
        /* Get the pointer for DL Config */
        TiFapi_dlCfgReq_s const * const p_dlCfgReq = &p_fapiNonTlvMsg->msg.dlCfgReq;

        /* Initialize bufLen to the size of TiFapi_dlCfgReq_s structure */
        bufLen = sizeof(TiFapi_dlCfgReq_s);

        /* Based on num of pch/mch/dlsch/ pdus in the TiFapi_dlCfgReq_s, strip 
         * off unnecessary white space at the end. This process starts from 
         * the end of the structure 
         */
        if(p_dlCfgReq->numBfLayers == 0)
        {
            bufLen -= sizeof(uint32_t) * TI_FAPI_BF_MAX_NUM_LAYERS;
            
            if (p_dlCfgReq->numCsiRsPdus == 0)
            {
                bufLen -= sizeof(TiFapi_csiRsPduCfg_s) * TI_FAPI_MAX_NUM_CSIRS_PDUS;
                
                if(p_dlCfgReq->numPchPdus == 0)
                {
                    /* strip off PCH pdus */
                    bufLen -= sizeof(TiFapi_pchPduCfg_s) * TI_FAPI_MAX_NUM_PCH_PDUS;

                    /* MCH-pdus */
                    if(p_dlCfgReq->numMchPdus == 0)
                    {
                        /* strip off the MCH pdus */
                        bufLen -= sizeof(TiFapi_mchPduCfg_s) * TI_FAPI_MAX_NUM_MCH_PDUS;

                        /* DLSCH-pdus */
                        if(p_dlCfgReq->numDlschPdus == 0)
                        {
                            /* strip off DLSCH pdus */
                            bufLen -= sizeof(TiFapi_dlschPduCfg_s) * TI_FAPI_MAX_NUM_DLSCH_PDUS;

                            /* DLdciCH-pdus */
                            if(p_dlCfgReq->numDlDciPdus == 0)
                            {
                                /* strip off dlDci pdus */
                                bufLen -= sizeof(TiFapi_dlDciPduCfg_s) * TI_FAPI_MAX_NUM_DL_DCI_PDUS;

                                if(p_dlCfgReq->numPrsPdus == 0)
                                {
                                    /* strip off prs pdus */
                                    bufLen -= sizeof(TiFapi_prsPduCfg_s);

                                    /* BCH-pdus */
                                    if(p_dlCfgReq->numBchPdus == 0)
                                        bufLen -= sizeof(TiFapi_bchPduCfg_s);
                                }
                            }
                            else
                            {
                                bufLen -= sizeof(TiFapi_dlDciPduCfg_s) * (TI_FAPI_MAX_NUM_DL_DCI_PDUS - p_dlCfgReq->numDlDciPdus);
                            }
                        }
                        else
                        {
                            bufLen -= sizeof(TiFapi_dlschPduCfg_s) * (TI_FAPI_MAX_NUM_DLSCH_PDUS - p_dlCfgReq->numDlschPdus);
                        }
                    }
                    else
                    {
                        bufLen -= sizeof(TiFapi_mchPduCfg_s) * (TI_FAPI_MAX_NUM_MCH_PDUS - p_dlCfgReq->numMchPdus);
                    }
                }
                else
                {
                    bufLen -= sizeof(TiFapi_pchPduCfg_s) * (TI_FAPI_MAX_NUM_PCH_PDUS - p_dlCfgReq->numPchPdus);
                }
            }
            else
            {
                bufLen -= sizeof(TiFapi_csiRsPduCfg_s) * (TI_FAPI_MAX_NUM_CSIRS_PDUS - p_dlCfgReq->numCsiRsPdus);
            }
        }/* If there are BF layers, nothing can be trimmed off */
    }
    
    return bufLen;
}

/**
 *  @b Description
 *  @n  
 *      The function is used to compress the UL.config FAPI message to remove 
 *      the unnecessary zeroes at the end of the message.
 *
 *  \ingroup FAPI_TRACING_FUNCTION
 *
 *  @param[in]  p_fapiNonTlvMsg
 *      This is a pointer to the UL config packet buffer for compression.
 *
 *  @retval
 *      length of the message body (without fapi message header) after 
 *      compression
 */
#ifdef _TMS320C6X
#ifdef __cplusplus
#pragma FUNC_ALWAYS_INLINE;
#else
#pragma FUNC_ALWAYS_INLINE(FapiTracing_compressUlConfig);
#endif
#endif
static inline uint32_t FapiTracing_compressUlConfig(TiFapi_nonTlvMsg_s const * const p_fapiNonTlvMsg) {
    uint32_t        bufLen = 0;

    if(p_fapiNonTlvMsg != NULL)
    {
        /* Get UL config message pointer and initialize bufLen to sizeof of TiFapi_ulCfgReq_s */
        TiFapi_ulCfgReq_s const * const p_ulCfgReq = &p_fapiNonTlvMsg->msg.ulCfgReq;
        bufLen = sizeof(TiFapi_ulCfgReq_s);

        /* Based on number of pdus, strip off white space at the end of the packet */
        if(p_ulCfgReq->numSrsPdus == 0)
        {
            /* strip off srs pdus */
            bufLen -= sizeof(TiFapi_srsPdu_s) * TI_FAPI_MAX_NUM_SRS_PDUS;

            /* uci-pdus */
            if(p_ulCfgReq->numUciPdus == 0)
            {
                /* strip off uci */
                bufLen -= sizeof(TiFapi_uciPdu_s) * TI_FAPI_MAX_NUM_UCI_PDUS;

                /* Ulsch-pdus */
                if(p_ulCfgReq->numUlschPdus == 0)
                {
                    /* strip off ulsch */
                    bufLen -= sizeof(TiFapi_ulschPdu_s) * TI_FAPI_MAX_NUM_ULSCH_PDUS;
                }
                else
                {
                    bufLen -= sizeof(TiFapi_ulschPdu_s) * (TI_FAPI_MAX_NUM_ULSCH_PDUS - p_ulCfgReq->numUlschPdus);
                }
            }
            else
            {
                bufLen -= sizeof(TiFapi_uciPdu_s) * (TI_FAPI_MAX_NUM_UCI_PDUS - p_ulCfgReq->numUciPdus);
            }
        }
        else
        {
            bufLen -= sizeof(TiFapi_srsPdu_s) * (TI_FAPI_MAX_NUM_SRS_PDUS - p_ulCfgReq->numSrsPdus);
        }
    }

    return bufLen;
}

/**
 *  @b Description
 *  @n  
 *      The function is used to compress the Tx.request FAPI message to 
 *      remove the unnecessary zeroes at the end of the message. When embedded 
 *      mode is being used, this function returns the size of the FAPI message,
 *      without the payload size. The payload size will need to be added to the
 *      returned value from this function to yield the value written to the 
 *      FAPI message header length field. 
 *
 *  \ingroup FAPI_TRACING_FUNCTION
 *
 *  @param[in]  p_fapiNonTlvMsg
 *      This is a pointer to the TX request packet buffer for compression.
 *
 *  @retval
 *      length of the message body (without fapi message header) after 
 *      compression
 */
#ifdef _TMS320C6X
#ifdef __cplusplus
#pragma FUNC_ALWAYS_INLINE;
#else
#pragma FUNC_ALWAYS_INLINE(FapiTracing_compressTxRequest);
#endif
#endif
static inline uint32_t FapiTracing_compressTxRequest(TiFapi_nonTlvMsg_s const * const p_fapiNonTlvMsg) {
    uint32_t     bufLen = 0;
    if(p_fapiNonTlvMsg != NULL)
    {
        TiFapi_txReq_s const * const p_txReq = &p_fapiNonTlvMsg->msg.txReq;

        if (p_txReq != NULL)
        {
            bufLen = offsetof(TiFapi_txReq_s, u);
            
            /* Reference FAPI mode? */
            if (p_txReq->numPdus != 0)
            {
                bufLen += sizeof(TiFapi_txReqPdu_s) * p_txReq->numPdus;
            }
            else /* Embedded FAPI mode */ 
            {
                bufLen += sizeof(TiFapi_txReqEmbeddedPdu_s);
            }
        }
    }
    return bufLen;
}

/**
 *  @b Description
 *  @n  
 *      The function is used to compress the HI_DCI FAPI message to remove the 
 *      unnecessary zeroes at the end of the message.
 *
 *  \ingroup FAPI_TRACING_FUNCTION
 *
 *  @param[in]  p_fapiNonTlvMsg
 *      This is a pointer to the HI_DCI0 packet buffer for compression.
 *  @retval
 *      length of the message body (without fapi message header) after 
 *      compression 
 */
#ifdef _TMS320C6X
#ifdef __cplusplus
#pragma FUNC_ALWAYS_INLINE;
#else
#pragma FUNC_ALWAYS_INLINE(FapiTracing_compressHiDCI0Config);
#endif
#endif
static inline uint32_t FapiTracing_compressHiDCI0Config(TiFapi_nonTlvMsg_s const * const p_fapiNonTlvMsg) {
    uint32_t            bufLen = 0;

    /* calculate effective buffer length based on number of Pdus. */
    if(NULL != p_fapiNonTlvMsg)
    {
        TiFapi_hiDci0Req_s const * const p_HiDci0Req = &p_fapiNonTlvMsg->msg.hiDci0Req;
        bufLen = sizeof(TiFapi_hiDci0Req_s);

        /* strip off dci0 packets based on numUlDciPdus */
        bufLen -= sizeof(TiFapi_dci0PduCfg_s) * (TI_FAPI_MAX_NUM_UL_DCI_PDUS - p_HiDci0Req->numUlDciPdus);

        /* if there is no Dci pdu, continue to strip off Hi pdus */
        if(p_HiDci0Req->numUlDciPdus == 0)
        {           
            bufLen  -= sizeof(TiFapi_hiPduCfg_s) * (TI_FAPI_MAX_NUM_HI_PDUS - p_HiDci0Req->numHiPdus);
        }
    }

    return bufLen;
}

/**
 *  @b Description
 *  @n  
 *      The function is used to compress the RX ULSCH request FAPI message to 
 *      remove the unnecessary zeroes at the end of the message. When embedded 
 *      mode is being used, this function returns the size of the FAPI message,
 *      without the payload size. The payload size will need to be added to the
 *      returned value from this function to yield the value written to the 
 *      FAPI message header length field. 
 *
 *  \ingroup FAPI_TRACING_FUNCTION
 *
 *  @param[in]  p_fapiNonTlvMsg
 *      This is a pointer to the RX ULSCH Indication packet buffer for 
 *      compression.
 *
 *  @retval
 *      length of the message body (without fapi message header) after 
 *      compression
 */
#ifdef _TMS320C6X
#ifdef __cplusplus
#pragma FUNC_ALWAYS_INLINE;
#else
#pragma FUNC_ALWAYS_INLINE(FapiTracing_compressRxULSCHInd);
#endif
#endif
static inline uint32_t FapiTracing_compressRxULSCHInd(TiFapi_nonTlvMsg_s const * const p_fapiNonTlvMsg) {
    uint32_t     bufLen = 0;
    if(p_fapiNonTlvMsg != NULL)
    {
        TiFapi_rxUlschInd_s const * const p_rxUlsch = &p_fapiNonTlvMsg->msg.rxUlschInd;

        if (p_rxUlsch != NULL)
        {
            bufLen = offsetof(TiFapi_rxUlschInd_s, u);
            
            /* Reference FAPI mode? */
            if (p_rxUlsch->numPdus != 0)
            {
                bufLen += sizeof(TiFapi_rxUlschPdu_s) * p_rxUlsch->numPdus;
            }
            else /* Embedded FAPI mode */ 
            {
                bufLen += sizeof(TiFapi_rxUlschEmbeddedPdu_s);
            }
        }
    }
    return bufLen;
}

/**
 *  @b Description
 *  @n  
 *      The function is used to compress the HARQ.indication FAPI message to 
 *      remove the unnecessary zeroes at the end of the message.
 *
 *  \ingroup FAPI_TRACING_FUNCTION
 *
 *  @param[in]  p_fapiNonTlvMsg
 *      This is a pointer to the HARQ indication packet buffer for compression.
 *
 *  @retval
 *      length of the message body (without fapi message header) after 
 *      compression
 */
#ifdef _TMS320C6X
#ifdef __cplusplus
#pragma FUNC_ALWAYS_INLINE;
#else
#pragma FUNC_ALWAYS_INLINE(FapiTracing_compressHarqInd);
#endif
#endif
static inline uint32_t FapiTracing_compressHarqInd(TiFapi_nonTlvMsg_s const * const p_fapiNonTlvMsg) {
    uint32_t        bufLen = 0;

    if(p_fapiNonTlvMsg != NULL)
    {
        /* Get HARQ indication message pointer and initialize bufLen to sizeof 
         * of TiFapi_harqInd_s 
         */
        TiFapi_harqInd_s const * const p_harqInd = &p_fapiNonTlvMsg->msg.harqInd;
        bufLen = sizeof(TiFapi_harqInd_s);

        /* Based on number of pdus, strip off white space at the end of 
         * the packet 
         */
        if(p_harqInd->numHarqPdus == 0)
        {
            /* strip off harq pdus */
            bufLen -= sizeof(TiFapi_harqPduInd_s) * TI_FAPI_MAX_NUM_HARQ_PDUS;
        }
        else
        {
            uint32_t numAckNack;
            bufLen -= sizeof(TiFapi_harqPduInd_s) * (TI_FAPI_MAX_NUM_HARQ_PDUS 
                                                     - p_harqInd->numHarqPdus);

            numAckNack = p_harqInd->harq[p_harqInd->numHarqPdus-1].numAckNack;
            if (numAckNack == 0)
            {
                /* Trim the harqData array */
                bufLen -= sizeof(TiFapi_harqData_u) * 
			  TI_FAPI_HARQ_IND_MAX_NUM_ACK_NACK;
            }
            else 
            {
                /* Trim the harqData array */
                bufLen -= sizeof(TiFapi_harqData_u) * 
                          (TI_FAPI_HARQ_IND_MAX_NUM_ACK_NACK - numAckNack);
            }
        }
    }

    return bufLen;
}

/**
 *  @b Description
 *  @n  
 *      The function is used to compress the RACH.indication FAPI message to 
 *      remove the unnecessary zeroes at the end of the message.
 *
 *  \ingroup FAPI_TRACING_FUNCTION
 *
 *  @param[in]  p_fapiNonTlvMsg
 *      This is a pointer to the RACH indication packet buffer for compression.
 *
 *  @retval
 *      length of the message body (without fapi message header) after 
 *      compression
 */
#ifdef _TMS320C6X
#ifdef __cplusplus
#pragma FUNC_ALWAYS_INLINE;
#else
#pragma FUNC_ALWAYS_INLINE(FapiTracing_compressRachInd);
#endif
#endif
static inline uint32_t FapiTracing_compressRachInd(TiFapi_nonTlvMsg_s const * const p_fapiNonTlvMsg) {
    uint32_t        bufLen = 0;

    if(p_fapiNonTlvMsg != NULL)
    {
        /* Get RACH indication message pointer and initialize bufLen to sizeof 
         * of TiFapi_rachInd_s 
         */
        TiFapi_rachInd_s const * const p_rachInd = 
                                                &p_fapiNonTlvMsg->msg.rachInd;
        bufLen = sizeof(TiFapi_rachInd_s);

        /* Based on number of preambles, strip off white space at the end of 
         * the packet 
         */
        if(p_rachInd->numOfPreamble == 0)
        {
            /* strip off preambles */
            bufLen -= sizeof(TiFapi_rachPduCfg_s) * 
                      TI_FAPI_MAX_NUM_RACH_PREAMBLES;  
        }
        else
        {
            bufLen -= sizeof(TiFapi_rachPduCfg_s) * 
                   (TI_FAPI_MAX_NUM_RACH_PREAMBLES - p_rachInd->numOfPreamble);
        }
    }

    return bufLen;
}

/**
 *  @b Description
 *  @n  
 *      The function is used to compress the SRS.indication FAPI message to 
 *      remove the unnecessary zeroes at the end of the message.
 *
 *  \ingroup FAPI_TRACING_FUNCTION
 *
 *  @param[in]  p_fapiNonTlvMsg
 *      This is a pointer to the SRS indication packet buffer for compression.
 *
 *  @retval
 *      length of the message body (without fapi message header) after 
 *      compression
 */
#ifdef _TMS320C6X
#ifdef __cplusplus
#pragma FUNC_ALWAYS_INLINE;
#else
#pragma FUNC_ALWAYS_INLINE(FapiTracing_compressSrsInd);
#endif
#endif
static inline uint32_t FapiTracing_compressSrsInd(TiFapi_nonTlvMsg_s const * const p_fapiNonTlvMsg) {
    uint32_t        bufLen = 0;

    if(p_fapiNonTlvMsg != NULL)
    {
        /* Get SRS indication message pointer and initialize bufLen to sizeof 
         * of TiFapi_srsInd_s 
         */
        TiFapi_srsInd_s const * const p_srsInd = &p_fapiNonTlvMsg->msg.srsInd;
        bufLen = sizeof(TiFapi_srsInd_s);

        /* Based on number of pdus, strip off white space at the end of 
         * the packet 
         */
        if(p_srsInd->numSrsPdus == 0)
        {
            /* strip off srs pdus */
            bufLen -= sizeof(TiFapi_srsIndPduCfg_s) * TI_FAPI_MAX_NUM_SRS_PDUS;
        }
        else 
        {
            uint32_t numRbs;
            bufLen -= sizeof(TiFapi_srsIndPduCfg_s) * (TI_FAPI_MAX_NUM_SRS_PDUS
                                                       - p_srsInd->numSrsPdus);
            
            numRbs = p_srsInd->srs[p_srsInd->numSrsPdus-1].numRbs;
            if (numRbs == 0)
            {
                /* Trim the srsPower and snr arrays */
                bufLen -= sizeof(uint16_t) * TI_FAPI_MAX_NUM_SRS_RBS_REPORTED_PER_UE;
                bufLen -= sizeof(uint8_t) * TI_FAPI_MAX_NUM_SRS_RBS_REPORTED_PER_UE;
            }
            else 
            {
                /* Trim the srsPower array */
                bufLen -= sizeof(uint16_t) * 
                          (TI_FAPI_MAX_NUM_SRS_RBS_REPORTED_PER_UE - numRbs);
            }
        }
    }

    return bufLen;
}

/**
 *  @b Description
 *  @n  
 *      The function is used to compress the CRC.indication FAPI message to 
 *      remove the unnecessary zeroes at the end of the message.
 *
 *  \ingroup FAPI_TRACING_FUNCTION
 *
 *  @param[in]  p_fapiNonTlvMsg
 *      This is a pointer to the CRC indication packet buffer for compression.
 *
 *  @retval
 *      length of the message body (without fapi message header) after 
 *      compression
 */
#ifdef _TMS320C6X
#ifdef __cplusplus
#pragma FUNC_ALWAYS_INLINE;
#else
#pragma FUNC_ALWAYS_INLINE(FapiTracing_compressCrcInd);
#endif
#endif
static inline uint32_t FapiTracing_compressCrcInd(TiFapi_nonTlvMsg_s const * const p_fapiNonTlvMsg) {
    uint32_t        bufLen = 0;

    if(p_fapiNonTlvMsg != NULL)
    {
        /* Get CRC indication message pointer and initialize bufLen to sizeof
         * of TiFapi_crcInd_s 
         */
        TiFapi_crcInd_s const * const p_crcInd = &p_fapiNonTlvMsg->msg.crcInd;
        bufLen = sizeof(TiFapi_crcInd_s);

        /* Based on number of CRCs, strip off white space at the end of the 
         * packet 
         */
        if(p_crcInd->numCrcs == 0)
        {
            /* strip off CRC pdus */
            bufLen -= sizeof(TiFapi_crcPduCfg_s) * TI_FAPI_MAX_NUM_CRC_PDUS; 
        }
        else
        {
            bufLen -= sizeof(TiFapi_crcPduCfg_s) * (TI_FAPI_MAX_NUM_CRC_PDUS - 
                                                    p_crcInd->numCrcs);
        }
    }

    return bufLen;
}

/**
 *  @b Description
 *  @n  
 *      The function is used to compress the RX_SR.indication FAPI message to 
 *      remove the unnecessary zeroes at the end of the message.
 *
 *  \ingroup FAPI_TRACING_FUNCTION
 *
 *  @param[in]  p_fapiNonTlvMsg
 *      This is a pointer to the RX_SR indication packet buffer for compression.
 *
 *  @retval
 *      length of the message body (without fapi message header) after 
 *      compression
 */
#ifdef _TMS320C6X
#ifdef __cplusplus
#pragma FUNC_ALWAYS_INLINE;
#else
#pragma FUNC_ALWAYS_INLINE(FapiTracing_compressRxSrInd);
#endif
#endif
static inline uint32_t FapiTracing_compressRxSrInd(TiFapi_nonTlvMsg_s const * const p_fapiNonTlvMsg) {
    uint32_t        bufLen = 0;

    if(p_fapiNonTlvMsg != NULL)
    {
        /* Get RX_SR indication message pointer and initialize bufLen to sizeof 
         * of TiFapi_rxSrInd_s 
         */
        TiFapi_rxSrInd_s const * const p_rxSrInd = &p_fapiNonTlvMsg->msg.rxSrInd;
        bufLen = sizeof(TiFapi_rxSrInd_s);

        /* Based on number of SR, strip off white space at the end of the 
         * packet 
         */
        if(p_rxSrInd->numOfSr == 0)
        {
            /* strip off SR pdus */
            bufLen -= sizeof(TiFapi_rxSrPduInd_s) * TI_FAPI_MAX_NUM_SR_PDUS;
        }
        else
        {
            bufLen -= sizeof(TiFapi_rxSrPduInd_s) * (TI_FAPI_MAX_NUM_SR_PDUS -
                                                     p_rxSrInd->numOfSr);
        }          
    }

    return bufLen;
}

/**
 *  @b Description
 *  @n  
 *      The function is used to compress the RX_CQI.indication FAPI message to 
 *      remove the unnecessary zeroes at the end of the message.
 *
 *  \ingroup FAPI_TRACING_FUNCTION
 *
 *  @param[in]  p_fapiNonTlvMsg
 *      This is a pointer to the RX_CQI indication packet buffer for compression.
 *
 *  @retval
 *      length of the message body (without fapi message header) after 
 *      compression
 */
#ifdef _TMS320C6X
#ifdef __cplusplus
#pragma FUNC_ALWAYS_INLINE;
#else
#pragma FUNC_ALWAYS_INLINE(FapiTracing_compressRxCqiInd);
#endif
#endif
static inline uint32_t FapiTracing_compressRxCqiInd(TiFapi_nonTlvMsg_s const * const p_fapiNonTlvMsg) {
    uint32_t        bufLen = 0;

    if(p_fapiNonTlvMsg != NULL)
    {
        /* Get CQI indication message pointer and initialize bufLen to sizeof 
         * of TiFapi_cqiInd_s 
         */
        TiFapi_cqiInd_s const * const p_cqiInd = &p_fapiNonTlvMsg->msg.cqiInd;
        bufLen = sizeof(TiFapi_cqiInd_s);

        /* Based on number of pdus, strip off white space at the end of the packet */
        if(p_cqiInd->numCqiPdus == 0)
        {
            /* strip off CQI pdus */
            bufLen -= sizeof(TiFapi_cqiPduCfg_s) * TI_FAPI_MAX_NUM_CQI_PDUS;
        }
        else
        {
            uint32_t numOfCc;
            
            bufLen -= sizeof(TiFapi_cqiPduCfg_s) * (TI_FAPI_MAX_NUM_CQI_PDUS - 
                                                    p_cqiInd->numCqiPdus);
            numOfCc = p_cqiInd->cqi[p_cqiInd->numCqiPdus-1].numOfCc;
         
            if (numOfCc == 0)
            {
                /* strip off CQI payloads for CCs */
                bufLen -= sizeof(uint8_t)*TI_FAPI_CQI_IND_MAX_NUM_CC*\
                          TI_FAPI_MAX_CQI_PAYLOAD;         
            }
            else
            {
                bufLen -= sizeof(uint8_t)*\
                          (TI_FAPI_CQI_IND_MAX_NUM_CC - numOfCc)*\
                          TI_FAPI_MAX_CQI_PAYLOAD;
            }           
        }
    }

    return bufLen;
}


#ifdef __cplusplus
}
#endif

#endif /* __FAPI_TRACING_H__ */

