/*
 *   @file  fapi_tracing.c
 *
 *   @brief   
 *      FAPI Tracing Template Implementation which can be invoked by the 
 *      L2 stack to send out all FAPI messages exchanged between the PHY
 *      and stack. These packets can then be captured and inspected using
 *      the Wireshark dissectors
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2014 Texas Instruments, Inc.
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

/* PDK Include Files. */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>

/* CSL Include Files. */
#include <ti/csl/csl_chip.h>
#include <ti/csl/csl_semAux.h>

/* Syslib Include Files. */
#include <ti/runtime/msgcom/msgcom.h>
#include <ti/runtime/resmgr/resmgr.h>

/* FAPI Header Files */
#include <ti/runtime/fapi_tracing/fapi_tracing.h>

/* For Debugging only. */
#ifdef __ARMv7
#define System_printf   printf
#else
#include <xdc/runtime/System.h>
#endif

/**********************************************************************
 ************************** Local Definitions *************************
 **********************************************************************/
#define FAPI_TRACING_TIMESTAMP_LENGTH           8
#define FAPI_TRACING_LOGBUFFER_ALIGNMENT        4096
#define FAPI_TRACING_MCB_ALIGNMENT              128

/**********************************************************************
 ************************** Local Structures **************************
 **********************************************************************/
/**
 * @brief 
 *  FapiTracing NetFP-based transport configuration
 *
 * @details
 *  NetFP based Transport Configuration for the FAPI tracing library 
 */
typedef struct FapiTracing_TransportInfo
{
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
     * @brief   UDP destination port number for the NetFP socket.
     */
    uint16_t                destPort;

    /**
     * @brief   DSCP value added to the logs' IP headers by NetFP.
     */
    uint8_t                 packetDSCP;

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
     * @brief   Socket Handle associated for sending out the tracing 
     * information
     */
    Netfp_SockHandle        socketHandle;   
    
    /**
     * @brief   Application registered POST_ROUTING Hook for memory logging
     */
    Netfp_HookFunction      postRoutingHook;

    /**
     * @brief   Application provided argument used for netfp POST_ROUTING Hook 
     */
    uint32_t                postRoutingHookArg;
}FapiTracing_transportInfo;

/** 
 * @brief 
 *  The structure encapsulates the stats of FAPI tracing Library.
 */
typedef struct FapiTracing_Stats
{
    /**
     * @brief   counter for total traced messages.
     */
    uint32_t            totalTraceMsg;

    /**
     * @brief   counter for total netfp send failed messages.
     */
    uint32_t            totalFailedMsg;

}FapiTracing_Stats;

/** 
 * @brief
 *  Master Control Block for the FAPI
 *
 * @details
 *  The structure holds all the information which is required for the FAPI
 *  interface to function.
 */
typedef struct FapiTracing_InstanceMCB
{
    /**
     * @brief   FAPI Tracing flag which indicates if the tracing support
     * is enabled or disabled. This can be modified at run time
     */
    uint32_t            isEnabled;

    /**
     * @brief   FAPI Data Tracing flag which indicates if the tracing support
     * is enabled or disabled. This can be modified at run time
     */
    uint32_t            isDataTracingEnabled;

    /**
     * @brief   FAPI Tracing transport control block
     */
    FapiTracing_transportInfo  transport;

    /**
     * @brief   counter for FAPI tracing messages.
     */
    FapiTracing_Stats          stats;
    
    /**
     * @brief   Instance configuration which was used to create the FAPI 
     * Tracing instance.
     */
    FapiTracing_InstCfg        instCfg;

}FapiTracing_InstanceMCB;


/**********************************************************************
 *********************** FAPI Tracing Functions ***********************
 **********************************************************************/

/**
 *  @b Description
 *  @n  
 *      The function is used to trace the DL TX request packets
 *
 *  \ingroup FAPI_TRACING_INTERNAL_FUNCTION
 *
 *  @param[in]  ptrInstanceMCB
 *      Pointer to the FAPI Tracing master control block for needed 
 *      configuration parameters
 *  @param[in]  ptrFAPIDLTxRequest
 *      Pointer to the DL config FAPI packet which is to be traced.
 *  @param[in]  ptrPkt
 *      Pointer to FAPI message packet descriptor
 *  @param[in]  dataBufferLen
 *      Length of data buffer for FAPI message packet
 *  @param[out]  errCode
 *      Error code populated by the API
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t FapiTracing_traceDLTransmitRequest (FapiTracing_InstanceMCB *ptrInstanceMCB, uint8_t* ptrFAPIDLTxRequest, Ti_Pkt const * const ptrPkt, uint32_t dataBufferLen, int32_t *errCode)
{
    Ti_Pkt*             ptrLastTracePkt = NULL;
    TiFapi_nonTlvMsg_s* nTlv;
    TiFapi_txReq_s*     inTxReq;
    uint8_t*            msgBuffer;
    Pktlib_InstHandle   pktlibInstHandle; 
    Pktlib_HeapHandle   pktlibHeapHandle;
    uint32_t            pduIndex;
    uint32_t            txReqMsgLen, bufLen;
    uint32_t            scatteredBufferIndex;
    uint32_t            pduPayloadLen;
    uint32_t            scatteredDataBuffer;
    Ti_Pkt*             ptrDataTracingPkt, ptrDataBufferPkt;
    Ti_Pkt*             ptrNextPkt;

    /* Obtain the Pktlib handles for use in this function. */
    pktlibInstHandle  = ptrInstanceMCB->transport.pktlibInstHandle;
    pktlibHeapHandle  = ptrInstanceMCB->transport.heapHandle;
    
    /* Decode the FAPI DL Transmit request Packet. */
    nTlv = (TiFapi_nonTlvMsg_s*)ptrFAPIDLTxRequest;

    /* Get the DL Transmit request */
    inTxReq = (TiFapi_txReq_s*) &nTlv->msg;
    
    /* Calculate tx Request message length based on number of pdus
     * if the return value is 0, then it is a invalid tx request message 
     */
    txReqMsgLen = FapiTracing_compressTxRequest((TiFapi_nonTlvMsg_s*)ptrFAPIDLTxRequest);
    if(txReqMsgLen == 0)
    {
        *errCode = FAPI_TRACING_EINVAL;
        return -1;
    }

    /* Add Fapi Header length */
    txReqMsgLen += sizeof(TiFapi_nonTlvMsgHeader_s);

    /* Allocate a zero buffer packet from the FAPI Tracing Heap */
    ptrDataTracingPkt = Pktlib_allocPacket(pktlibInstHandle, pktlibHeapHandle, 0);
    if (ptrDataTracingPkt == NULL)
    {
        *errCode = FAPI_TRACING_ENOMEM;
        return -1;
    }

    /* Check to see if the Data Trace is required. 
     * In embedded FAPI mode:
     *      Data Trace enabled: packet length should be comprised of 
     *                          all chained packets 
     *      Data Trace not enabled: packet length should only be the header 
     *                              + FAPI message (minus payload).
     * In reference FAPI mode: 
     *      Data Trace enabled: packet length should be changed to include 
     *                          the addition packets needed to trace the 
     *                          PDUs pointed to in the FAPI message.
     *      Data Trace not enabled: packet length should be unchanged.
     */
    
    /* If embedded FAPI mode message, then buffer size determination is 
     * different since the data follows the header and message content.
     */  
    if (inTxReq->numPdus == 0) /* Embedded FAPI mode */ 
    {
        if (ptrInstanceMCB->isDataTracingEnabled)
        {
            /* Determine the size of the first packet. 
             * The data buffer and message length for embedded mode is 
             * the same as the dataBufferLen from the packet, unless 
             * the message header length + pduLength in the message is less 
             * than the dataBufferLen, meaning there are multiple packets.
             */
            if (dataBufferLen > (txReqMsgLen + inTxReq->u.pduInfo.pduLength))
            {
                /* Data is all in this packet */ 
                txReqMsgLen += inTxReq->u.pduInfo.pduLength;
            }
            else
            {
                /* Data is across multiple packets */
                txReqMsgLen = dataBufferLen;
            }
        }
        /* If data tracing mode is not enabled, then txReqMsgLen already 
         * contains the side of the FAPI message + header, without payload.
         */
    }
    
    /* Set the FAPI Packet with the new data buffer & length */
    Pktlib_setPacketLen(ptrDataTracingPkt, txReqMsgLen);

    /* Link the data buffer together */
    Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc*)
                  Pktlib_getDescFromPacket(ptrDataTracingPkt),
                  (uint8_t*)ptrFAPIDLTxRequest, txReqMsgLen);
                 
    if (ptrInstanceMCB->isDataTracingEnabled)
    {
        if (inTxReq->numPdus == 0)
        {
            ptrNextPkt = Pktlib_getNextPacket((Ti_Pkt*)ptrPkt);
            
            /* Need to link any remaining buffers to the buffer being traced. */
            while (ptrNextPkt != NULL)
            {
                /* Get data buffer from DL TX packet data descriptor */
                Pktlib_getDataBuffer (ptrNextPkt, (uint8_t**)&msgBuffer, &bufLen);
                
                /* Allocate a zero buffer packet from the FAPI Tracing Heap */
                ptrDataBufferPkt = Pktlib_allocPacket(pktlibInstHandle, pktlibHeapHandle, 0);
                if (ptrDataBufferPkt == NULL)
                {
                    /* In case of allocation error, free all the packets allocated */
                    Pktlib_freePacket(pktlibInstHandle, ptrDataTracingPkt);
                    *errCode = FAPI_TRACING_ENOMEM;
                    return -1;
                }
            
                /* Set data packet to DL TX packet chained data buffers, use
                 * buffer length from the original data buffer 
                 */
                Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc*)
                              Pktlib_getDescFromPacket(ptrDataBufferPkt),
                              (uint8_t*)msgBuffer, bufLen);
            
                /* Set the FAPI Packet with the new length */
                Pktlib_setPacketLen(ptrDataBufferPkt, bufLen);
                
                /* Link packet to previous one */
                Pktlib_packetMerge (pktlibInstHandle, ptrDataTracingPkt, ptrDataBufferPkt, ptrLastTracePkt);
                
                /* Update the last tracing pkt descriptor in the chain */
                ptrLastTracePkt = ptrDataBufferPkt;
            
                /* Get next packet from chained packet, if it is not chained 
                 * packet, ptrNextPkt is NULL 
                 */
                ptrNextPkt = Pktlib_getNextPacket(ptrNextPkt);
            }
        }
        else
        {
            TiFapi_txReqPdu_s*  pInPdu;

            /* Cycle through all pdu and all the buffers */
            for(pduIndex = 0; pduIndex < inTxReq->numPdus; pduIndex++)
            {
                /* Get the PDU */
                pInPdu = &inTxReq->u.pdu[pduIndex];

                /* Each PDU can have multiple scattered buffers.
                 *  - Cycle through all such buffers and copy them into the packet being traced. 
                 */
                for (scatteredBufferIndex = 0; scatteredBufferIndex < pInPdu->numScatBuf;scatteredBufferIndex++)
                {
                    /* Get the scattered payload length & address */
                    pduPayloadLen       = pInPdu->data[scatteredBufferIndex].bufLen;
                    scatteredDataBuffer = (uint32_t)pInPdu->data[scatteredBufferIndex].bufBasePtr;

                    if((scatteredDataBuffer != (uint32_t)NULL) && (pduPayloadLen != 0))
                    {
                        /* Allocate a zero buffer packet from the FAPI Tracing Heap */
                        ptrDataBufferPkt = Pktlib_allocPacket(pktlibInstHandle, pktlibHeapHandle, 0);
                        if (ptrDataBufferPkt == NULL)
                        {
                            /* Allocation failure, free all allocated packets and return an error */
                            Pktlib_freePacket(pktlibInstHandle, ptrDataTracingPkt);
                            *errCode = FAPI_TRACING_ENOMEM;
                            return -1;
                        }

                        /* Attach buffer to the tracing packet */
                        Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc*)Pktlib_getDescFromPacket(ptrDataBufferPkt),
                                     (uint8_t*)scatteredDataBuffer, pduPayloadLen);

                        /* Set the FAPI Packet with the new data buffer & length */
                        Pktlib_setPacketLen(ptrDataBufferPkt, pduPayloadLen);

                        /* Link packet to previous one */
                        Pktlib_packetMerge (pktlibInstHandle, ptrDataTracingPkt, ptrDataBufferPkt, ptrLastTracePkt);

                        /* Update the last tracing pkt descriptor in the 
                         * chain 
                         */
                        ptrLastTracePkt = ptrDataBufferPkt;

                    }
                }
            }
        }
    } /* Data Tracing enabled */

    /* The DATA Tracing packet is actually a Zero buffer packet so it will end up 
     * moving to the garbage queue and we will need to execute the garbage collector. 
     */
    if(Netfp_send (ptrInstanceMCB->transport.socketHandle, ptrDataTracingPkt, 0, errCode) < 0)
    {
        Pktlib_freePacket(pktlibInstHandle, ptrDataTracingPkt);
        ptrInstanceMCB->stats.totalFailedMsg++;
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n  
 *      The function is used to trace the DL config request, with or without 
 *      beam-forming vectors. 
 *
 *  \ingroup FAPI_TRACING_INTERNAL_FUNCTION
 *
 *  @param[in]  ptrInstanceMCB
 *      Pointer to the FAPI Tracing master control block for needed 
 *      configuration parameters
 *  @param[in]  ptrFAPIDLCfgRequest
 *      Pointer to the DL config FAPI packet which is to be traced.
 *  @param[out]  errCode
 *      Error code populated by the API
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t FapiTracing_traceDLCfgRequest (FapiTracing_InstanceMCB *ptrInstanceMCB, uint8_t* ptrFAPIDLCfgRequest, int32_t *errCode)
{
    Ti_Pkt*             ptrPrevPkt = NULL;
    Ti_Pkt*             ptrFAPITracePkt;
    Ti_Pkt*             ptrDataBufferPkt;
    TiFapi_nonTlvMsg_s* nTlv;
    TiFapi_dlCfgReq_s*  inCfgReq;
    uint8_t*            pLayerBuffer;
    Pktlib_InstHandle   pktlibInstHandle; 
    Pktlib_HeapHandle   pktlibHeapHandle;
    uint32_t            vectorBuffLen;
    uint32_t            msgLen;
    uint32_t            layerIndex;
    
    /* Obtain the Pktlib handles for use in this function. */
    pktlibInstHandle  = ptrInstanceMCB->transport.pktlibInstHandle;
    pktlibHeapHandle  = ptrInstanceMCB->transport.heapHandle;
        
    /* Decode the FAPI DL Transmit request Packet. */
    nTlv = (TiFapi_nonTlvMsg_s*)ptrFAPIDLCfgRequest;

    /* Get the DL Config request */
    inCfgReq = (TiFapi_dlCfgReq_s*) &nTlv->msg;

    /* Get the payload length + FAPI header length*/
    msgLen = (uint32_t)(nTlv->hdr.msgLen) + sizeof(TiFapi_nonTlvMsgHeader_s);

    /* Allocate a zero buffer packet from the FAPI Tracing Heap */
    ptrFAPITracePkt = Pktlib_allocPacket(pktlibInstHandle, pktlibHeapHandle, 0);
    if (ptrFAPITracePkt == NULL)
    {
        *errCode = FAPI_TRACING_ENOMEM;
        return -1;
    }
    
    /* Set the FAPI Packet with the new data buffer & length */
    Pktlib_setPacketLen(ptrFAPITracePkt, msgLen);

    /* Link the data buffer together */
    Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc*)Pktlib_getDescFromPacket(ptrFAPITracePkt), 
                 (uint8_t*)ptrFAPIDLCfgRequest, msgLen);

    /* Determine if there are beam forming vectors present in the message. 
     * If not, the packet can be sent like other packets. If there are 
     * beam forming vectors, the present buffers need to be merged to the end 
     * of the zero copy buffer so that they are sent with the config message.
     */
    if (inCfgReq->numBfLayers != 0)
    {              
        /* Vectors are defined as uint8_t values, 1 for each of real and 
         * imaginary components, for all RBs and Antennae.
         */
        vectorBuffLen = sizeof(TiFapi_bfBuffer_s);
                            
        /* set the previous packet to be the trace packet */
        ptrPrevPkt = ptrFAPITracePkt;

        /* Cycle through all BF layers and skip non-specified vectors 
         * with NULL-valued pointers.
         */
        for(layerIndex = 0; layerIndex < inCfgReq->numBfLayers; layerIndex++)
        {
            /* Get the PDU */
            pLayerBuffer = (uint8_t*)inCfgReq->addrOfBfBuffers[layerIndex];
            
            if (pLayerBuffer != NULL)
            {
                /* Allocate a zero buffer packet from the FAPI Tracing Heap */
                ptrDataBufferPkt = Pktlib_allocPacket(pktlibInstHandle, pktlibHeapHandle, 0);
                if (ptrDataBufferPkt == NULL)
                {
                    /* Allocation failure, free all allocated packets and return an error */
                    Pktlib_freePacket(pktlibInstHandle, ptrFAPITracePkt);
                    *errCode = FAPI_TRACING_ENOMEM;
                    return -1;
                }

                /* Attach buffer to the tracing packet */
                Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc*)Pktlib_getDescFromPacket(ptrDataBufferPkt),
                             (uint8_t*)pLayerBuffer, vectorBuffLen);

                /* Set the FAPI Packet with the new data buffer & length */
                Pktlib_setPacketLen(ptrDataBufferPkt, vectorBuffLen);

                /* Link packet to previous one */
                Pktlib_packetMerge (pktlibInstHandle, ptrPrevPkt, ptrDataBufferPkt, NULL);
            }
        }
    }
   
    /* The Tracing packet is actually a Zero buffer packet so it will end up
     * moving to the garbage heap and we will need to execute the garbage 
     * collector. 
     */
    if(Netfp_send (ptrInstanceMCB->transport.socketHandle, ptrFAPITracePkt, 0, errCode) < 0)
    {
        /* Send failure, NetFp error code will be returned */
        Pktlib_freePacket(pktlibInstHandle, ptrFAPITracePkt);
        ptrInstanceMCB->stats.totalFailedMsg++;
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n  
 *      The function is used to trace the ULSCH indication packets
 *
 *  \ingroup FAPI_TRACING_INTERNAL_FUNCTION      

 *  @param[in]  ptrInstanceMCB
 *      Pointer to the FAPI Tracing master control block for needed 
 *      configuration parameters
 *  @param[in]  ptrULRxULSHInd
 *      Pointer to the Ulsch Indication FAPI packet which is to be traced.
 *  @param[in]  ptrPkt
 *      Pointer to FAPI message packet descriptor
 *  @param[in]  dataBufferLen
 *      Length of data buffer for FAPI message packet
 *  @param[out]  errCode
 *      Error code populated by the API
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t FapiTracing_traceULRxULSCHIndication (FapiTracing_InstanceMCB *ptrInstanceMCB, uint8_t* ptrULRxULSHInd, Ti_Pkt const * const ptrPkt, uint32_t dataBufferLen, int32_t* errCode)
{
    Ti_Pkt*                 ptrLastTracePkt = NULL;
    TiFapi_nonTlvMsg_s*     nTlv;
    TiFapi_rxUlschInd_s*    inULSchInd;
    uint32_t                ulschIndMsgLen, bufLen;
    Ti_Pkt*                 ptrDataTracingPkt;
    Ti_Pkt*                 ptrDataBufferPkt, ptrNextPkt;
    Pktlib_InstHandle       pktlibInstHandle; 
    Pktlib_HeapHandle       pktlibHeapHandle;
    uint32_t                pduIndex;
    uint8_t*                msgBuffer;

    /* Obtain the Pktlib handles for use in this function. */
    pktlibInstHandle  = ptrInstanceMCB->transport.pktlibInstHandle;
    pktlibHeapHandle  = ptrInstanceMCB->transport.heapHandle;
    
    /* Decode the FAPI DL Transmit request Packet. */
    nTlv = (TiFapi_nonTlvMsg_s*)ptrULRxULSHInd;

    /* Get the UL Receive Indication request */
    inULSchInd = (TiFapi_rxUlschInd_s*) &nTlv->msg;
    
    /* Calculate Ulsch.ind message size based on number of Pdus, 
     * this will be the data buffer offset 
     */
    ulschIndMsgLen = FapiTracing_compressRxULSCHInd((TiFapi_nonTlvMsg_s*)ptrULRxULSHInd);
    if(ulschIndMsgLen == 0)
    {
        *errCode = FAPI_TRACING_EINVAL;
        return -1;
    }
    
    /* Add the message header length */
    ulschIndMsgLen += sizeof(TiFapi_nonTlvMsgHeader_s);
    
    /* Allocate memory for the data tracing packet. */
    ptrDataTracingPkt = Pktlib_allocPacket(pktlibInstHandle, pktlibHeapHandle, 0);
    if (ptrDataTracingPkt == NULL)
    {
        *errCode = FAPI_TRACING_ENOMEM;
        return -1;
    }

    /* Check to see if the Data Trace is required. 
     * In embedded FAPI mode:
     *      Data Trace enabled: packet length should be comprised of 
     *                          all chained packets 
     *      Data Trace not enabled: packet length should only be the header 
     *                              + FAPI message (minus payload).
     * In reference FAPI mode: 
     *      Data Trace enabled: packet length should be changed to include 
     *                          the addition packets needed to trace the 
     *                          PDUs pointed to in the FAPI message.
     *      Data Trace not enabled: packet length should be unchanged.
     */
        
    /* If embedded FAPI mode message, then buffer size determination is 
     * different since the data follows the header and message content.
     */  
    if (inULSchInd->numPdus == 0) /* Embedded FAPI mode */ 
    {
        if (ptrInstanceMCB->isDataTracingEnabled)
        {
            /* Determine the size of the first packet.
             * The data buffer and message length for embedded mode is 
             * the same as the dataBufferLen from the packet, unless 
             * the message header length + pduLength in the message is less 
             * than the dataBufferLen.
             */
            if (dataBufferLen > (ulschIndMsgLen + inULSchInd->u.pduInfo.length))
            {
                /* Data is all in this packet */
                ulschIndMsgLen += inULSchInd->u.pduInfo.length;
            }
            else
            {
                /* Data is across multiple packets */
                ulschIndMsgLen = dataBufferLen;
            }
        }
        /* If data tracing is not enabled, the ulschIndMsgLen already 
         * accounts for the size of the FAPI message + header, without payload.
         */
    }
    
    /* Set the FAPI Packet with the new data buffer & length */
    Pktlib_setPacketLen(ptrDataTracingPkt, ulschIndMsgLen);

    /* Link the data buffer together */
    Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc*)
                 Pktlib_getDescFromPacket(ptrDataTracingPkt),
                 (uint8_t*)ptrULRxULSHInd, ulschIndMsgLen);

    /* Handling of data buffers for embedded and reference FAPI modes is 
     * different, so split the processing here. 
     */ 
    if (ptrInstanceMCB->isDataTracingEnabled)
    {
        if (inULSchInd->numPdus == 0) /* embedded FAPI mode handling */ 
        {
            ptrNextPkt = Pktlib_getNextPacket((Ti_Pkt*)ptrPkt);
            
            /* Need to link any remaining buffers to the buffer being traced. */
            while (ptrNextPkt != NULL)
            {
                /* Get data buffer from Ulsch.ind packet data descriptor */
                Pktlib_getDataBuffer (ptrNextPkt, (uint8_t**)&msgBuffer, &bufLen);
                
                /* Allocate a zero buffer packet from the FAPI Tracing Heap */
                ptrDataBufferPkt = Pktlib_allocPacket(pktlibInstHandle, pktlibHeapHandle, 0);
                if (ptrDataBufferPkt == NULL)
                {
                    /* In case of allocation error, free all the packets allocated */
                    Pktlib_freePacket(pktlibInstHandle, ptrDataTracingPkt);
                    *errCode = FAPI_TRACING_ENOMEM;
                    return -1;
                }
            
                /* Set data packet to Ulsch.ind packet chained data buffers, use
                 * buffer length from the original data buffer 
                 */
                Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)ptrDataBufferPkt,
                              (uint8_t*)msgBuffer, bufLen);
            
                /* Set the FAPI Packet with the new length */
                Pktlib_setPacketLen(ptrDataBufferPkt, bufLen);
                
                /* Link packet to previous one */
                Pktlib_packetMerge (pktlibInstHandle, ptrDataTracingPkt, ptrDataBufferPkt, ptrLastTracePkt);
            
                /* Update the last tracing pkt descriptor in the chain */
                ptrLastTracePkt = ptrDataBufferPkt;

                /* Get next packet from chained packet, if it is not chained 
                 * packet, ptrNextPkt is NULL 
                 */
                ptrNextPkt = Pktlib_getNextPacket(ptrNextPkt);
            }
        }
        else /* reference FAPI mode handling */
        {
            TiFapi_rxUlschPdu_s*    pInUlSchPdu;
            
            /* Cycle through and copy data for each PDU */
            for(pduIndex = 0; pduIndex < inULSchInd->numPdus; pduIndex++)
            {
                /* Get the PDU */
                pInUlSchPdu = &inULSchInd->u.pdu[pduIndex];
                
                /* Validate and ensure that the ULSCH packet is valid */
                if(pInUlSchPdu->data.bufDescriptorPtr == 0)
                  continue;

                /* Get the packet handle from the first packet in the chain, and go through chained buffers */
                ptrNextPkt = (Ti_Pkt*)pInUlSchPdu->data.bufDescriptorPtr; 
                while (ptrNextPkt != NULL)
                {
                    /* Get data buffer from Ulsch.ind packet data descriptor */
                    Pktlib_getDataBuffer (ptrNextPkt, (uint8_t**)&msgBuffer, &bufLen);

                    /* Allocate a zero buffer packet from the FAPI Tracing Heap */
                    ptrDataBufferPkt = Pktlib_allocPacket(pktlibInstHandle, pktlibHeapHandle, 0);
                    if (ptrDataBufferPkt == NULL)
                    {
                           /* In case of allocation error, free all the packets allocated */
                           Pktlib_freePacket(pktlibInstHandle, ptrDataTracingPkt);
                           *errCode = FAPI_TRACING_ENOMEM;
                           return -1;
                    }

                    /* Set data packet to Ulsch.ind packet chained data buffers, use
                     * buffer length from the original data buffer 
                     */
                    Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)ptrDataBufferPkt,
                                 (uint8_t*)msgBuffer, bufLen);

                    /* Set the FAPI Packet with the new data buffer & length */
                    Pktlib_setPacketLen(ptrDataBufferPkt, Pktlib_getPacketLen(ptrNextPkt));

                    /* Link packet to previous one */
                    Pktlib_packetMerge (pktlibInstHandle, ptrDataTracingPkt, ptrDataBufferPkt, ptrLastTracePkt);

                    /* Update the last tracing pkt descriptor in the chain */
                    ptrLastTracePkt = ptrDataBufferPkt;

                    /* Get next packet from chained packet, if it is not chained packet, ptrNextPkt is NULL */
                    ptrNextPkt = Pktlib_getNextPacket(ptrNextPkt);
                
               }
            }
        }
    } /* Data Tracing enabled */

    /* The DATA Tracing packet is actually a Zero buffer packet so it will end up 
     * moving to the garbage heap and we will need to execute the garbage collector. 
     */
    if (Netfp_send (ptrInstanceMCB->transport.socketHandle, ptrDataTracingPkt, 0, errCode) < 0)
    {
        Pktlib_freePacket(pktlibInstHandle, ptrDataTracingPkt);
        ptrInstanceMCB->stats.totalFailedMsg++;
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to configure the FAPI tracing options. The API can be used
 *      to enable/disable the tracing capabilities. If tracing is disabled all data 
 *      tracing is disabled also.
 *
 *  \ingroup FAPI_TRACING_FUNCTION
 *
 *  @param[in]  fapiTracingInstHandle
 *      FAPI Tracing instance handle for needed configuration parameters
 *  @param[in]  tracing
 *      Tracing flag. Set to 0 to disable all tracing including data tracing
 *  @param[in]  dataTracing
 *      Data Tracing flag. Set to 0 to disable data tracing
 *  @param[out] errCode
 *      Error code populated by the API
 *
 *  @retval
 *      Not applicable
 */
int32_t FapiTracing_configure(FapiTracing_InstHandle fapiTracingInstHandle, uint8_t const tracing, uint8_t const dataTracing, int32_t* const errCode)
{
    FapiTracing_InstanceMCB * const ptrInstanceMCB = (FapiTracing_InstanceMCB*) fapiTracingInstHandle;
    /* Validate the FAPI Tracing Instance handle before proceeding. */
    if (ptrInstanceMCB == NULL)
    {
        *errCode = FAPI_TRACING_EINVAL;
        return -1;
    }  

    /* Configure the data tracing flags. */
    ptrInstanceMCB->isEnabled              = tracing;
    ptrInstanceMCB->isDataTracingEnabled   = dataTracing;
    
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to trace the FAPI packet.
 *
 *  \ingroup FAPI_TRACING_FUNCTION      
 *
 *  @param[in]  fapiTracingInstHandle
 *      FAPI Tracing instance handle for needed configuration parameters
 *  @param[in]  ptrPkt
 *      Packet to be traced
 *  @param[out]  errCode
 *      Error Code populated by the API
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
int32_t FapiTracing_trace(FapiTracing_InstHandle fapiTracingInstHandle, Ti_Pkt const * const ptrPkt, int32_t* const errCode)
{
    Ti_Pkt*             ptrFAPITracePkt;
    uint8_t*            ptrDataBuffer;
    Pktlib_InstHandle   pktlibInstHandle; 
    Pktlib_HeapHandle   pktlibHeapHandle;
    uint32_t            dataBufferLen;
    TiFapi_msgIdType_t  msgId;
    uint32_t            traceLength;
    int32_t             retVal = 0;

    FapiTracing_InstanceMCB * const ptrInstanceMCB = (FapiTracing_InstanceMCB*) fapiTracingInstHandle;
    
    /* Validate the FAPI Tracing Instance handle before proceeding. */
    if (ptrInstanceMCB == NULL)
    {
        *errCode = FAPI_TRACING_EINVAL;
        return -1;
    }  
    
    /* Is FAPI Tracing enabled? */
    if (ptrInstanceMCB->isEnabled == 0)
        return 0;
    ptrInstanceMCB->stats.totalTraceMsg++;
    
    /* Get the data buffer associated with the packet */
    Pktlib_getDataBuffer ((Ti_Pkt*)ptrPkt, &ptrDataBuffer, &dataBufferLen);

    /* Get the message identifier from the FAPI message. */
    msgId = (TiFapi_msgIdType_t)(*ptrDataBuffer);

    /* Obtain the Pktlib handles for use in this function. */
    pktlibInstHandle  = ptrInstanceMCB->transport.pktlibInstHandle;
    pktlibHeapHandle  = ptrInstanceMCB->transport.heapHandle; 

    /* Is this a DL Transmit request packet? */
    if (msgId == TI_FAPI_DL_TX_REQUEST)
    {
        retVal = FapiTracing_traceDLTransmitRequest (ptrInstanceMCB, ptrDataBuffer, ptrPkt, dataBufferLen, errCode);
    }
    else if (msgId == TI_FAPI_UL_RX_ULSCH_INDICATION)
    {
        retVal = FapiTracing_traceULRxULSCHIndication (ptrInstanceMCB, ptrDataBuffer, ptrPkt, dataBufferLen, errCode);
    }
    else if (msgId == TI_FAPI_DL_CONFIG_REQUEST)
    {
        retVal = FapiTracing_traceDLCfgRequest (ptrInstanceMCB, ptrDataBuffer, errCode);
    }
    else
    {
        /* Use packet Len as trace length. To achieve better performance, 
	 * application should compress the FAPI message, set proper packet 
	 * len, and then pass the packet for tracing. 
	 */
        traceLength = Pktlib_getPacketLen((Ti_Pkt*)ptrPkt);

        /* Allocate a zero buffer packet from the FAPI Tracing Heap */
        ptrFAPITracePkt = Pktlib_allocPacket(pktlibInstHandle, pktlibHeapHandle, 0);
        if (ptrFAPITracePkt == NULL)
        {
            *errCode = FAPI_TRACING_ENOMEM;
            return -1;
        }

        /* Set the FAPI Packet with the new data buffer & length */
        Pktlib_setPacketLen(ptrFAPITracePkt, traceLength);

        /* Link the data buffer together */
        Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc*)Pktlib_getDescFromPacket(ptrFAPITracePkt), 
                     (uint8_t*)ptrDataBuffer, traceLength);
   
        /* The Tracing packet is actually a Zero buffer packet so it will end up
         * moving to the garbage heap and we will need to execute the garbage collector 
	 */
        if(Netfp_send (ptrInstanceMCB->transport.socketHandle, ptrFAPITracePkt, 0, errCode) < 0)
        {
            /* Send failure, NetFp error code will be returned */
            
            Pktlib_freePacket(pktlibInstHandle, ptrFAPITracePkt);
            ptrInstanceMCB->stats.totalFailedMsg++;
            return -1;
        }
    }

    if (retVal == 0)
    {
        /* Execute the garbage collector on the FAPI Tracing Heap 
         *  - Zero buffer packets which get recycled after the CPDMA end up in the
         *    Garbage collection queue. 
         *  - The Garbage collector will move these packets back into the free 
         *    queues. 
         */
        Pktlib_garbageCollection(pktlibInstHandle, pktlibHeapHandle);
    }
    return retVal;
}

/**
 *  @b Description
 *  @n  
 *      The function initializes the FAPI tracing framework. 
 *
 *  \ingroup FAPI_TRACING_FUNCTION
 *
 *  @param[in]  fapiTracingInstHandle
 *      FAPI Tracing instance handle for needed configuration parameters
 *  @param[in]  ptrFapiTracingCfg
 *      Configuration passed to the library at initialization.
 *  @param[out]  errCode
 *      Error Code populated by the API
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
int32_t FapiTracing_init(FapiTracing_InstHandle fapiTracingInstHandle, Fapi_TracingCfg const * const ptrFapiTracingCfg, int32_t* const errCode)
{
    Netfp_SockAddr                 sockAddress;
    Netfp_OptionTLV                OptCfg;
    int32_t                        internalErr;
    Netfp_HookCfg                  hookCfg;
    
    /* Validate the FAPI Tracing Instance handle before proceeding. */
    if (fapiTracingInstHandle == NULL)
    {
        *errCode = FAPI_TRACING_EINVAL;
        return -1;
    }

    Fapi_TracingTransportCfg const * const transportCfg = &ptrFapiTracingCfg->transportCfg;
    
    /* Validate the arguments */
    if ((transportCfg->ingressFastPathHandle == NULL) || 
        (transportCfg->egressFastPathHandle == NULL) ||
        (transportCfg->heapHandle == NULL) ||
        (transportCfg->srcUDPPort == 0) ||
        (transportCfg->destUDPPort == 0) ||
        (transportCfg->netfpClientHandle == NULL) ||
        (transportCfg->pktlibInstHandle == NULL))
    {
        *errCode = FAPI_TRACING_EINVAL;
        return -1;
    }
    
    /* Sanity check the sin_family */
    if( (transportCfg->sin_family != Netfp_SockFamily_AF_INET ) &&
        (transportCfg->sin_family != Netfp_SockFamily_AF_INET6))
    {
        *errCode = FAPI_TRACING_EINVAL;
        return -1;
    }
    FapiTracing_InstanceMCB* const ptrInstanceMCB = (FapiTracing_InstanceMCB*) fapiTracingInstHandle;
    FapiTracing_transportInfo* const pTransport   = &ptrInstanceMCB->transport;

    /* Initialize the FAPI Tracing MCB */
    pTransport->pktlibInstHandle      = transportCfg->pktlibInstHandle;
    pTransport->heapHandle            = transportCfg->heapHandle;
    pTransport->ingressFastPathHandle = transportCfg->ingressFastPathHandle;
    pTransport->egressFastPathHandle  = transportCfg->egressFastPathHandle;
    pTransport->destPort              = transportCfg->destUDPPort;
    pTransport->packetDSCP            = transportCfg->packetDSCP;
    pTransport->netfpClientHandle     = transportCfg->netfpClientHandle;
    pTransport->postRoutingHook       = transportCfg->postRoutingHook;
    pTransport->postRoutingHookArg    = transportCfg->postRoutingHookArg;
       
    /************************************************************************************
    * FAPI Tracing Socket:
    ************************************************************************************/
    /* Create a socket */
    pTransport->socketHandle = Netfp_socket (pTransport->netfpClientHandle, transportCfg->sin_family, errCode);
    if (pTransport->socketHandle == NULL)
    {
        return -1;
    }

    /* Populate the binding information */
    memset ((void*)&sockAddress, 0, sizeof(Netfp_SockAddr));
    sockAddress.sin_family              = transportCfg->sin_family;
    sockAddress.sin_port                = transportCfg->srcUDPPort;
    sockAddress.op.bind.flowId          = 0xFFFFFFFF;
    sockAddress.op.bind.queueHandle     = (Qmss_QueueHnd)NULL;
    sockAddress.op.bind.notifyFunction  = NULL;
    sockAddress.op.bind.inboundFPHandle = pTransport->ingressFastPathHandle;

    /* Bind the socket. */
    if (Netfp_bind (pTransport->socketHandle, &sockAddress, errCode) < 0)
    {
        Netfp_closeSocket (pTransport->socketHandle, &internalErr);
        return -1;
    }
    
    /* Initialize the socket address */
    memset((void *)&sockAddress, 0, sizeof(Netfp_SockAddr));
    
    /* Populate the connect information. */
    sockAddress.sin_family                  = transportCfg->sin_family;
    sockAddress.sin_port                    = transportCfg->destUDPPort;
    sockAddress.op.connect.outboundFPHandle = pTransport->egressFastPathHandle;
    
    /* Connect the socket */
    if (Netfp_connect(pTransport->socketHandle, &sockAddress, errCode) < 0)
    {
        Netfp_closeSocket (pTransport->socketHandle, &internalErr);
        return -1;
    }
    
    /* Set socket priority */
    memset((void *)&OptCfg, 0, sizeof(Netfp_OptionTLV));
    OptCfg.length = sizeof(uint8_t);
    OptCfg.type   = Netfp_Option_PRIORITY;
    OptCfg.value  = (void *)&(pTransport->packetDSCP);
    
    if (Netfp_setSockOpt(pTransport->socketHandle, &OptCfg, errCode) < 0)
    {
        Netfp_closeSocket (pTransport->socketHandle, &internalErr);
        *errCode = FAPI_TRACING_ENOTRANSPOT;
        return -1;
    }

    if(pTransport->postRoutingHook)
    {
        /* Populate the hook configuration: */
        hookCfg.hook        = Netfp_Hook_POST_ROUTING;
        hookCfg.sockHandle  = pTransport->socketHandle;
        hookCfg.hookFxn     = pTransport->postRoutingHook;
        hookCfg.hookArg     = pTransport->postRoutingHookArg;

        /* Register the socket POST_ROUTING hook: */
        if (Netfp_registerHook (pTransport->netfpClientHandle, &hookCfg, errCode) < 0)
        {
            return -1;
        }
    }

    /* Enable FAPI Tracing. */
    ptrInstanceMCB->isEnabled            = ptrFapiTracingCfg->isEnabled;
    ptrInstanceMCB->isDataTracingEnabled = ptrFapiTracingCfg->isDataTracingEnabled;

    /* Tracing Initialization is done */
    return 0;
}

/**
 *  @b Description
 *  @n  
 *      The function is used to de-initialize the FAPI tracing framework
 *
 *  \ingroup FAPI_TRACING_FUNCTION
 *
 *  @param[in]  fapiTracingInstHandle
 *      FAPI Tracing instance handle for needed configuration parameters
 *  @param[out]  errCode
 *      Error Code populated by the API
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
int32_t FapiTracing_deinit(FapiTracing_InstHandle fapiTracingInstHandle, int32_t* const errCode)
{
    FapiTracing_InstanceMCB* const ptrInstanceMCB = (FapiTracing_InstanceMCB*) fapiTracingInstHandle;
    if (ptrInstanceMCB == NULL)
    {
        *errCode = FAPI_TRACING_EINVAL;
        return -1;
    }    
    
    /* Disable the tracing flags */
    if (FapiTracing_configure (fapiTracingInstHandle, 0, 0, errCode) < 0) {
        return -1;
    }

    /* Execute the garbage collector on the FAPI Tracing Heap */
    Pktlib_garbageCollection(ptrInstanceMCB->transport.pktlibInstHandle, ptrInstanceMCB->transport.heapHandle);

    /* Close the FAPI tracing socket. */
    if (Netfp_closeSocket(ptrInstanceMCB->transport.socketHandle, errCode) < 0)
        return -1;

    ptrInstanceMCB->transport.socketHandle = NULL;

    /* Tracing module has been shut down. */
    return 0;
}

/**
 *  @b Description
 *  @n  
 *      The function is used to create the FAPI Tracing instance. 
 *
 *  @param[in]  ptrInstCfg
 *      Pointer to the FAPI Tracing instance configuration
 *  @param[out]  errCode
 *      Error Code populated if there is an error.
 *
 *  \ingroup FAPI_TRACING_FUNCTION      
 *
 *  @retval
 *      0   -   Success
 *  @retval
 *      <0  -   Error
 */
FapiTracing_InstHandle FapiTracing_createInstance (FapiTracing_InstCfg const * const ptrInstCfg, int32_t* const errCode)
{
    FapiTracing_InstanceMCB* ptrInstanceMCB;

    /* Sanity Check: Validate the arguments. */
    if (ptrInstCfg == NULL)
    {
        *errCode = FAPI_TRACING_EINVAL;
        return NULL;
    } 

    /* Sanity Check: Ensure that the OSAL callout has been specified. */
    if ((ptrInstCfg->malloc         == NULL)    ||  (ptrInstCfg->free           == NULL)) 
    {
        *errCode = FAPI_TRACING_EINVAL;
        return NULL;
    }

    /* Allocate memory for the FapiTracing instance. */
    ptrInstanceMCB = ptrInstCfg->malloc (sizeof(FapiTracing_InstanceMCB), FAPI_TRACING_MCB_ALIGNMENT);
    if (ptrInstanceMCB == NULL)
    {
        *errCode = FAPI_TRACING_ENOMEM;
        return NULL;
    }

    /* Initialize the allocated memory block. */
    memset ((void *)ptrInstanceMCB, 0 , sizeof(FapiTracing_InstanceMCB));

    /* Populate the configuration */
    memcpy ((void *)&ptrInstanceMCB->instCfg, (void *)ptrInstCfg, sizeof(FapiTracing_InstCfg));

    /* Return the heap instance handle. */
    return (FapiTracing_InstHandle)ptrInstanceMCB;
}

/**
 *  @b Description
 *  @n  
 *      The function is used to delete the FAPI Tracing instance. It is the 
 *      responsibility of the application to ensure that all of the PKTLIB 
 *      instances, associated heaps, and NETFP handles are deleted.
 *
 *  @param[in]  fapiTracingInstHandle
 *      FAPI Tracing instance handle which is being deleted
 *  @param[out]  errCode
 *      Error Code populated if there is an error.
 *
 *  \ingroup FAPI_TRACING_FUNCTION      
 *
 *  @retval
 *      0   -   Success
 *  @retval
 *      <0  -   Error
 */
int32_t FapiTracing_deleteInstance (FapiTracing_InstHandle fapiTracingInstHandle, int32_t* const errCode)
{
    FapiTracing_InstanceMCB* ptrInstanceMCB;

    /* Sanity Check: Validate the arguments. */
    ptrInstanceMCB = (FapiTracing_InstanceMCB*)fapiTracingInstHandle;
    if (ptrInstanceMCB == NULL)
    {
        *errCode = FAPI_TRACING_EINVAL;
        return -1;
    }

    /* Clean up memory for the FAPI Tracing library instance. */
    ptrInstanceMCB->instCfg.free (ptrInstanceMCB, sizeof(FapiTracing_InstanceMCB));
    return 0;
}


