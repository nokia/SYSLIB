/**
 *   @file  sim_phy.h
 *
 *   @brief
 *      Main header file for the Simulated PHY application
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
#ifndef __SIM_PHY_DOMAIN_H__
#define __SIM_PHY_DOMAIN_H__

/* SYSLIB Include Files */
#include <ti/runtime/msgcom/msgcom.h>
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/netfp/netfp.h>
#include <ti/runtime/root/root.h>
#include <ti/runtime/domain/domain.h>

/**
 * @brief
 *  Simulated PHY Domain MCB
 *
 * @details
 *  The structure is used to hold all the necessary information for
 *  creating the LTE Stack Domain. Application developers can map this
 *  to their native data structures.
 */
typedef struct AppSimPHYDomainMCB
{
    /**
     * @brief  Application identifier identifier
     */
    uint32_t                appId;

    /**
     * @brief  SYSLIB configuration passed by the root master
     */
    Root_SyslibConfig       rootSyslibCfg;

    /**
     * @brief  SYSLIB domain handle.
     */
    Domain_SyslibHandle     syslibHandle;

    /**
     * @brief  Opaque handle to the FAPI module.
     */
    void*                   fapiHandle;

    /**
     * @brief  FAPI MSMC Heap Handle
     */
    Pktlib_HeapHandle       fapiMSMCHeapHandle;

    /**
     * @brief  FAPI MSMC Heap Handle
     */
    Pktlib_HeapHandle       fapiDDR3HeapHandle;

    /**
     * @brief  DAT Producer heap which is used to exchange UIA messages between the producer
     * and the L2 consumer
     */
    Pktlib_HeapHandle       datProducerHeap;

    /**
     * @brief  DAT producer handle which generates the log messages.
     */
    Dat_ProdHandle          producerHandle;

    /**
     * @brief  Heap Handle: This is the local L2 heap handle available for allocating memory
     * for the local L2 memory
     */
    IHeap_Handle            localHeapHandle;

    /**
     * @brief  Heap Handle: This is the shared MSMC heap handle available for allocating memory
     * for the MSMC memory. This heap is shared across all the other cores in the RAT. Please
     * restrict usage of this heap
     */
    IHeap_Handle            msmcSharedHeapHandle;

    /**
     * @brief  Heap Handle: This is the shared DDR3 heap handle available for allocating memory
     * for the DDR3 memory. This heap is shared across all the other cores in the RAT. Please
     * restrict usage of this heap
     */
    IHeap_Handle            ddr3SharedHeapHandle;

    /**
     * @brief  Heap Handle: This is the private DDR3 heap handle available for allocating memory
     * for the DDR3 memory and is private to the domain.
     */
    IHeap_Handle            ddr3PrivateHeapHandle;

    /**
     * @brief  Task Handle: This is the task handle for all the application tasks
     */
    Task_Handle             simPhyInitTaskHandle;
}AppSimPHYDomainMCB;

#endif /* __SIM_PHY_DOMAIN_H__ */

