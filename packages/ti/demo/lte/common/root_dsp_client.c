/**
 *   @file  root_dsp_client.c
 *
 *   @brief
 *      Root slave implementation for all DSP cores.
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
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

/* BIOS/XDC Include Files. */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Memory.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <ti/sysbios/family/c64p/Hwi.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/family/c64p/EventCombiner.h>

/* IPC Include Files for Shared Memory Allocation. */
#include <ti/ipc/Ipc.h>
#include <ti/ipc/SharedRegion.h>
#include <ti/sdo/ipc/notifyDrivers/IInterrupt.h>

/* MCSDK Include Files */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/drv/rm/rm.h>
#include <ti/csl/csl_chip.h>
#include <ti/csl/csl_cacheAux.h>
#include <ti/csl/csl_device_interrupt.h>

/* SYSLIB Include Files */
#include <ti/runtime/msgcom/msgcom.h>
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/root/root.h>

/**********************************************************************
 ************************** Global Variables **************************
 **********************************************************************/

/* ROOT Slave Handle */
Root_SlaveHandle        rootSlaveHandle;

/* ROOT Slave Semaphore Handle */
Semaphore_Handle        rootSlaveSemHandle;

/**********************************************************************
 ************************** Extern Definitions ************************
 **********************************************************************/

/* CACHE Management API: */
extern void appInvalidateBuffer(void* ptr, uint32_t size);
extern void appWritebackBuffer(void* ptr, uint32_t size);

/**********************************************************************
 *********************** Unit Test  Functions *************************
 **********************************************************************/

/**
 *  @b Description
 *  @n
 *      The function is the ISR handler which handles the interrupt from the
 *      master root domain
 *
 *  @retval
 *      Not Applicable.
 */
static void Root_slaveISRHandler(UArg argument)
{
    /* Post the semaphore to wake up the root task. */
    Semaphore_post(rootSlaveSemHandle);
}

/**
 *  @b Description
 *  @n
 *      Root Task which executes and is listening for commands from the root
 *      master
 *
 *  @retval
 *      Not Applicable.
 */
void Root_slaveTask(UArg arg0, UArg arg1)
{
    int32_t errCode;

    /* Root Task executes forever until the DSP lives. */
    while (1)
    {
        /* Wait for a notification from the root master */
        Semaphore_pend(rootSlaveSemHandle, BIOS_WAIT_FOREVER);

        /* Request received. Execute the root slave. */
        if (Root_executeSlave (rootSlaveHandle, &errCode) < 0)
            System_printf ("Error: Root Slave execution failed [Error code %d]\n", errCode);
    }
}

/**
 *  @b Description
 *  @n
 *     The function is used by the root module to allocate memory
 *
 *  @param[in]  numBytes
 *      Number of bytes of memory to be allocated
 *  @param[in]  alignment
 *      Alignment requirements
 *
 *  @retval
 *      Success -   Pointer to the allocated memory
 *  @retval
 *      Error   -   NULL
 */
static void* Root_osalMalloc(uint32_t numBytes, uint32_t alignment)
{
    return Memory_alloc (NULL, numBytes, alignment, NULL);
}

/**
 *  @b Description
 *  @n
 *     The function is used by the root module to free memory
 *
 *  @param[in]  ptr
 *      Pointer to the memory to be cleaned up
 *  @param[in]  numBytes
 *      Number of bytes of memory to be cleaned up
 *
 *  @retval
 *      Not applicable
 */
static void Root_osalFree(void* ptr, uint32_t numBytes)
{
    Memory_free (NULL, ptr, numBytes);
}

/**
 *  @b Description
 *  @n
 *     The function is used by the root module to create a semaphore
 *
 *  @retval
 *      Opaque critical section handle
 */
static void* Root_osalCreateSem(void)
{
    return (void*)Semaphore_create(0, NULL, NULL);
}

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to pend on the semaphore associated with the job
 *      This is done once a SYNC job is submitted.
 *
 *  @param[in]  semHandle
 *      Opaque Semaphore Handle
 *
 *  @retval
 *      Not Applicable.
 */
static void Root_osalPendSem(void* semHandle)
{
    Semaphore_pend(semHandle, BIOS_WAIT_FOREVER);
}

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to post the semaphore associated with the job
 *      This is done once a SYNC job is submitted.
 *
 *  @param[in]  semHandle
 *      Opaque Semaphore Handle
 *
 *  @retval
 *      Not Applicable.
 */
static void Root_osalPostSem(void* semHandle)
{
    Semaphore_post(semHandle);
}

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to delete a semaphore associated with a JOB.
 *
 *  @param[in]  semHandle
 *      Opaque Semaphore Handle
 *
 *  @retval
 *      Not Applicable.
 */
static void Root_osalDeleteSem(void* semHandle)
{
    /* Delete the semaphore. */
    Semaphore_delete ((Semaphore_Handle*)semHandle);
}

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to enter the critical section. This is required to
 *      protect the ROOT services from multiple threads.
 *
 *  @retval
 *      Opaque critical section handle
 */
static void* Root_osalEnterCS(void)
{
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to exit the critical section. This is required to
 *      protect the ROOT services from multiple threads.
 *
 *  @param[in]  csHandle
 *      Opaque critical section handle
 *
 *  @retval
 *      Not Applicable.
 */
static void Root_osalExitCS(void* csHandle)
{
    return;
}

/**
 *  @b Description
 *  @n
 *     OSAL API is used by the root module to invalidate the contents
 *     of the cache.
 *
 *  @param[in]  ptr
 *      Pointer to the buffer to be invalidated
 *  @param[in]  size
 *      Size of the buffer to be invalidated
 *
 *  @retval
 *      Not applicable
 */
static void Root_osalBeginMemoryAccess(void* ptr, uint32_t size)
{
    appInvalidateBuffer(ptr, size);
}

/**
 *  @b Description
 *  @n
 *     OSAL API is used by the root module to writeback the contents
 *     of the cache.
 *
 *  @param[in]  ptr
 *      Pointer to the buffer to be written back
 *  @param[in]  size
 *      Size of the buffer to be written back
 *
 *  @retval
 *      Not applicable
 */
static void Root_osalEndMemoryAccess(void* ptr, uint32_t size)
{
    appWritebackBuffer(ptr, size);
}

/**
 *  @b Description
 *  @n
 *      C Entry point
 *
 *  @retval
 *      Not Applicable.
 */
void main (void)
{
    Task_Params         taskParams;
    Root_SlaveConfig    rootSlaveCfg;
    int32_t             errCode;

    /* Initialize TSCL register */
    TSCL = 0;

    /* Enable the caches. */
	CACHE_setL1DSize (CACHE_L1_32KCACHE);
	CACHE_setL1PSize (CACHE_L1_32KCACHE);

	/* Initialize the heap in shared memory. Using IPC module to do that */
    Ipc_start();

    /* Initialize the slave root configuration. */
    memset ((void *)&rootSlaveCfg, 0, sizeof(Root_SlaveConfig));

    /* Populate the slave root configuration.
     * - This configuration is device specific. The master core identifier is
     *   set to 8 which is the ARM Core0 identifier as per the device specifications
     * - Address is from the ROOT_RSVD_MEM in the platform memory file. */
    strcpy(rootSlaveCfg.rootMasterName, "SYSLIB-TestRootMaster");
    rootSlaveCfg.ptrSharedMemory        = (uint8_t*)DDR3_SYSLIB_ROOT_RSVD_MEM;
    rootSlaveCfg.sizeSharedMemory       = 64*1024;
    rootSlaveCfg.bootCfgAddress         = (uint32_t)CSL_BOOT_CFG_REGS;
    rootSlaveCfg.slaveCoreId            = ROOT_SLAVE_COREID(1, DNUM);
    rootSlaveCfg.masterCoreId           = 8;
    rootSlaveCfg.ipcSrcId               = 16;
    rootSlaveCfg.osalFxn.malloc         = Root_osalMalloc;
    rootSlaveCfg.osalFxn.free           = Root_osalFree;
    rootSlaveCfg.osalFxn.beginMemAccess = Root_osalBeginMemoryAccess;
    rootSlaveCfg.osalFxn.endMemAccess   = Root_osalEndMemoryAccess;
    rootSlaveCfg.osalFxn.enterCS        = Root_osalEnterCS;
    rootSlaveCfg.osalFxn.exitCS         = Root_osalExitCS;
    rootSlaveCfg.osalFxn.createSem      = Root_osalCreateSem;
    rootSlaveCfg.osalFxn.deleteSem      = Root_osalDeleteSem;
    rootSlaveCfg.osalFxn.postSem        = Root_osalPostSem;
    rootSlaveCfg.osalFxn.pendSem        = Root_osalPendSem;

    /* Create the root slave. */
    rootSlaveHandle = Root_slaveCreate (&rootSlaveCfg, &errCode);
    if (rootSlaveHandle == NULL)
    {
        System_printf ("Error: Unable to create the slave root domain [Error code %d]\n", errCode);
        return;
    }
    System_printf ("Debug: Slave root domain [%p] created successfully\n", rootSlaveHandle);

    /* Create the root slave semaphore handle */
    rootSlaveSemHandle = Semaphore_create(0, NULL, NULL);
    if (rootSlaveSemHandle == NULL)
        return;

    /* Register the interrupt handler for IPC interrupts. */
    EventCombiner_dispatchPlug (CSL_C66X_COREPAC_IPC_GRN, (EventCombiner_FuncPtr)Root_slaveISRHandler, (xdc_UArg)rootSlaveHandle, TRUE);
    EventCombiner_enableEvent (CSL_C66X_COREPAC_IPC_GRN);

    /* Launching the root task */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 16*1024;
    taskParams.priority  = 8;
    Task_create((ti_sysbios_knl_Task_FuncPtr)Root_slaveTask, &taskParams, NULL);

    /* Start BIOS */
    BIOS_start();
}

