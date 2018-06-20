/**
 *   @file  root_dsp.c
 *
 *   @brief
 *      The file implements the root domain functionality which allows
 *      the DSP to start application specific domains on the DSP.
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

/* MCSDK include files */
#include <ti/csl/csl_chipAux.h>
#include <ti/csl/csl_bootcfgAux.h>

/* ROOT Include Files. */
#include <ti/runtime/root/root.h>
#include <ti/runtime/root/include/root_internal.h>

/* For Debugging only. */
#include <xdc/runtime/System.h>

/**************************************************************************
 ************************* Root DSP Functions *****************************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      The function is used to map the physical address to a virtual address
 *      This is a wrapper function in the DSP realm since the virtual and physical
 *      address are one and the same.
 *
 *  @param[in]   phyAddr
 *      Physical address
 *  @param[in]   size
 *      Size of the physical address which is to be mapped
 *
 *  \ingroup ROOT_DSP_FUNCTIONS
 *
 *  @retval
 *      Success -   Mapped virtual address
 *  @retval
 *      Error   -   0
 */
uint32_t Root_mapPhyAddrToVirtual (uint32_t phyAddr, uint32_t size)
{
    /* For DSP: The physical and virtual address are the same. */
    return phyAddr;
}

/**
 *  @b Description
 *  @n
 *      The function is used to generate an interrupt to the peer domain
 *      to indicate that a JOSH message has been populated and should be
 *      processed.
 *
 *  @param[in]  ptrMasterSlaveMCB
 *      Pointer to the master/slave MCB
 *
 *  \ingroup ROOT_DSP_FUNCTIONS
 *
 *  @retval
 *      Not applicable
 */
static void Root_generateIPCInterrupt (Root_MasterSlaveMCB* ptrMasterSlaveMCB)
{
    /* Generate the interrupt. */
    ptrMasterSlaveMCB->ptrBootCfgRegs->IPCGR[ptrMasterSlaveMCB->peerCoreId] =
            CSL_FMKR(ptrMasterSlaveMCB->ipcSrcId + 4, ptrMasterSlaveMCB->ipcSrcId + 4, 1) |
            CSL_FMK (BOOTCFG_IPCGR0_IPCGR0_REG, 1);
}

/**
 *  @b Description
 *  @n
 *      The function is used to initialize the socket services which are
 *      available to the root slave to communicate with the root master. In the
 *      DSP realm there are no socket services used to communicate with the
 *      master; since this is done through IPC interrupts.
 *
 *  @param[in]  ptrMasterSlaveMCB
 *      Pointer to the master/slave MCB
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup ROOT_DSP_FUNCTIONS
 *
 *  @retval
 *      Always returns success (i.e. 0)
 */
int32_t Root_slaveInitializeSocket (Root_MasterSlaveMCB* ptrMasterSlaveMCB, int32_t* errCode)
{
    /* No sockets on DSP. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to determine if the slave buffer has valid data
 *      or not. On the DSP there is always valid data because the blocking
 *      implementation is done on the IPC interrupt; which basically implies
 *      that by the time the JOSH receive is done the data was available.
 *
 *  @param[in]  ptrMasterSlaveMCB
 *      Pointer to the master/slave MCB
 *
 *  \ingroup ROOT_DSP_FUNCTIONS
 *
 *  @retval
 *      Always returns valid data
 */
int32_t Root_isSlaveBufferReady (Root_MasterSlaveMCB* ptrMasterSlaveMCB)
{
    return 1;
}

/**
 *  @b Description
 *  @n
 *      The function is used to notify the root slave from the root master to
 *      indicate that the JOSH buffer has been populated and is available.
 *
 *  @param[in]  ptrMasterSlaveMCB
 *      Pointer to the master/slave MCB
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup ROOT_DSP_FUNCTIONS
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0
 */
int32_t Root_notifySlave (Root_MasterSlaveMCB* ptrMasterSlaveMCB, int32_t* errCode)
{
    /* FATAL Error: The master never executes on the DSP so this should never be called */
    System_printf ("FATAL Error: Root notify slave invoked in the DSP\n");
    *errCode = ROOT_EINTERNAL;
    return -1;
}

/**
 *  @b Description
 *  @n
 *      The function is used to notify the root master from the root slave to
 *      indicate that the JOSH buffer has been populated and is available.
 *
 *  @param[in]  ptrMasterSlaveMCB
 *      Pointer to the master/slave MCB
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup ROOT_DSP_FUNCTIONS
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0
 */
int32_t Root_notifyMaster (Root_MasterSlaveMCB* ptrMasterSlaveMCB, int32_t* errCode)
{
    /* Master execute on the ARM and are always notified by the IPC interrupt. */
    Root_generateIPCInterrupt (ptrMasterSlaveMCB);
    return 0;
}

