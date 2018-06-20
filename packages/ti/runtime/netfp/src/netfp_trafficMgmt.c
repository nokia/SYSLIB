/**
 *   @file  netfp_trafficMgmt.c
 *
 *   @brief
 *      The file contains the default implementation of the reassembly
 *      traffic management routine.
 *
 *      In this implementation, when number of used buffers in the reassembly
 *      system exceeds the upper threshold (application provided parameter
 *      at init time), the oldest packets under construction are dropped
 *      till the lower threshold limit is met.
 *
 *      Users can create a custom implementation of the traffic management
 *      routine and pass the function pointer to Netfp_init() to override
 *      this implementation.
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
#include <string.h>

/* Packet Library Include Files. */
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/netfp/netfp.h>
#include <ti/runtime/netfp/include/netfp_internal.h>

/* For Debugging only. */
#ifdef __ARMv7
#include <ti/apps/netfp_config/include/NetFP_System_printf.h> //fzm
#else
#include <xdc/runtime/System.h>
#endif

/**************************************************************************
 ************************** Extern Functions ******************************
 **************************************************************************/

/* Exported API which cleans up the reassembly context. This is an internal
 * NETFP function but is exposed to the traffic management module. */
extern int32_t Netfp_freeReassemblyContext
(
    Netfp_ClientMCB*            ptrClientMCB,
    Netfp_ReassemblyMCB*        ptrReassemblyMCB,
    Netfp_IPReassemblyContext*  ptrReassemblyContext,
    uint32_t                    cleanTrafficFlow
);

/**************************************************************************
 *********************** Default Reassembly Management ********************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      This is the default NETFP supplied traffic management implementation.
 *
 *  @param[in]  clientHandle
 *      NETFP Client handle
 *  @param[in]  ptrReassemblyMgmtCfg
 *      Pointer to the reassembly management configuration
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not Applicable.
 */
void Netfp_defaultReassemblyTrafficMgmt (Netfp_ClientHandle clientHandle, void* ptrReassemblyMgmtCfg)
{
    Netfp_ClientMCB*                 ptrClientMCB;
    Netfp_ReassemblyMCB*             ptrReassemblyMCB;
    Netfp_IPReassemblyContext*       ptrReassemblyContext;
    Netfp_IPReassemblyContext*       ptrPrevReassemblyContext;
    uint32_t                         reqdDropFragCount;
    uint32_t                         currentDropFragCount;
    Netfp_DefaultReassemblyMgmtCfg*  ptrMgmtConfig;
    Ti_Pkt*                          ptrPkt; //fzm

    /* Get the pointer to the NETFP Client */
    ptrClientMCB = (Netfp_ClientMCB*)clientHandle;

    /* Get the pointer to the Reassembly MCB */
    ptrReassemblyMCB = ptrClientMCB->ptrReassemblyMCB;

    /* Get the default reassembly management configuration. */
    ptrMgmtConfig = (Netfp_DefaultReassemblyMgmtCfg*)ptrReassemblyMgmtCfg;

    /* Do we need to kick start the number of active fragments? */
    if (ptrReassemblyMCB->stats.numActiveFragments <= ptrMgmtConfig->bufferThresholdUpper)
        return;

    /* Increment the number of invocations. */
    ptrReassemblyMCB->stats.numDefaultReassemblyMgmtInvocations++;

    /* Threshold based dropping has been activated: Determine the number of fragments to be dropped. */
    reqdDropFragCount = ptrReassemblyMCB->stats.numActiveFragments - ptrMgmtConfig->bufferThresholdLower;

    /* Get the current head of the reassembly context list. */
    ptrReassemblyContext = (Netfp_IPReassemblyContext*)Netfp_listGetHead ((Netfp_ListNode**)&ptrReassemblyMCB->ptrUsedContextList);
    if (ptrReassemblyContext == NULL)
    {
        System_printf ("Error: Empty reassembly context but %d active fragments\n", ptrReassemblyMCB->stats.numActiveFragments);
        return;
    }

    /* Cycle through all the reassembly contexts and get the oldest reassembly context */
    while (Netfp_listGetNext((Netfp_ListNode*)ptrReassemblyContext) != NULL)
        ptrReassemblyContext = (Netfp_IPReassemblyContext*)Netfp_listGetNext((Netfp_ListNode*)ptrReassemblyContext);

    /* Start discarding packets */
    currentDropFragCount = 0;
    while (currentDropFragCount < reqdDropFragCount)
    {
        /* Drop all the fragments which are pending in the reassembly context. */
        currentDropFragCount = currentDropFragCount + ptrReassemblyContext->fragCnt;

        /* Increment the number of fragments which are being dropped by the default reassembly management module. */
        ptrReassemblyMCB->stats.numDefaultReassemblyMgmtDroppedFragments += ptrReassemblyContext->fragCnt;

        /* Get the next oldest reassembly block before we kill the reassembly context */
        ptrPrevReassemblyContext = (Netfp_IPReassemblyContext*)Netfp_listGetPrev((Netfp_ListNode*)ptrReassemblyContext);

        ptrPkt = ptrReassemblyContext->ptrPkt;

        /* Free the reassembled context and also cleanup the traffic flow in the NETCP subsystem. */
        Netfp_freeReassemblyContext (ptrClientMCB, ptrReassemblyMCB, ptrReassemblyContext, 1);

        /* Cleanup the *partial* reassembled packet. */
        Pktlib_freePacket (ptrClientMCB->cfg.pktlibInstHandle, ptrPkt);

        /* Ensure that we have not reached the start of the list. */
        ptrReassemblyContext = ptrPrevReassemblyContext;
        if (ptrReassemblyContext == NULL)
            break;
    }

    /* Did we drop all the required fragments? */
    if (currentDropFragCount < reqdDropFragCount)
        System_printf ("Error: No more reassembly contexts left [Dropped %d Required %d]\n", currentDropFragCount, reqdDropFragCount);
    return;
}


