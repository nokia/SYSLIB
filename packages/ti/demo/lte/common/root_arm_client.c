/**
 *   @file  root_arm_client.c
 *
 *   @brief
 *      Root slave implementation for an ARM processes. There is 1
 *      root slave instantiated per ARM process.
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
#include <pthread.h>
#include <semaphore.h>

/* MCSDK Include files. */
#include <ti/csl/cslr_device.h>

/* SYSLIB Include Files */
#include <ti/runtime/root/root.h>

/**********************************************************************
 ************************** Global Variables **************************
 **********************************************************************/

/* ROOT Slave Handle */
Root_SlaveHandle        rootSlaveHandle;

/**********************************************************************
 *********************** Unit Test  Functions *************************
 **********************************************************************/

/**
 *  @b Description
 *  @n
 *      Root Task which executes and is listening for commands from the root
 *      master
 *
 *  @retval
 *      Not Applicable.
 */
static void* Root_slaveTask(void *arg)
{
    int32_t             errCode;
    struct sched_param  param;

    /* Set the configured policy and priority: Root threads should have the HIGHEST priority in the system */
    param.sched_priority = 10;
    errCode = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
    if (errCode != 0)
    {
        printf ("Error: Unable to set the UINTC thread priority & policy [Error Code %d]\n", errCode);
        return NULL;
    }

    /* Debug Message: */
    printf ("Debug: Root Slave thread is operational\n");

    /* Root Task executes forever in the background. Listening to requests issued by the root master */
    while (1)
    {
        /* Interrupt has been raised by the root master. Execute the root slave. */
        if (Root_executeSlave (rootSlaveHandle, &errCode) < 0)
            printf ("Error: Root Slave execution failed [Error code %d]\n", errCode);
        else
            printf ("Debug: Root Slave executed successfully\n");
    }
    return NULL;
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
    return malloc(numBytes);
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
    free(ptr);
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
    int32_t     retVal;
    sem_t*      ptrSemaphore;

    /* Allocate memory for a semaphore. */
    ptrSemaphore = malloc (sizeof(sem_t));
    if (ptrSemaphore == NULL)
    {
        printf ("Error: Out of memory while allocating agent semaphore\n");
        return NULL;
    }

    /* Create a semaphore. */
    retVal = sem_init (ptrSemaphore, 0, 0);
    if (retVal < 0)
    {
        free (ptrSemaphore);
        return NULL;
    }
    return (void*)ptrSemaphore;
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
    sem_wait ((sem_t*)semHandle);
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
    sem_post ((sem_t*)semHandle);
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
    sem_close ((sem_t*)semHandle);
    free (semHandle);
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
    /* There is no need for cache operations on ARM */
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
    /* There is no need for cache operations on ARM */
}

/**
 *  @b Description
 *  @n
 *      C Entry point
 *
 *  @retval
 *      Not Applicable.
 */
int32_t main (void)
{
    Root_SlaveConfig    rootSlaveCfg;
    pthread_t           rootSlaveThread;
    int32_t             errCode;

    /* Initialize the slave root configuration. */
    memset ((void *)&rootSlaveCfg, 0, sizeof(Root_SlaveConfig));

    /* Populate the slave root configuration.
     * - This configuration is device specific. The master core identifier is
     *   set to 8 which is the ARM Core0 identifier as per the device specifications
     * - Address is from the ROOT_RSVD_MEM in the DSP platform memory file.
     * - The slave core identifier is set to 8. */
    strcpy(rootSlaveCfg.rootMasterName, "SYSLIB-TestRootMaster");
    rootSlaveCfg.ptrSharedMemory        = (uint8_t*)0xA0000000;
    rootSlaveCfg.sizeSharedMemory       = 64*1024;
    rootSlaveCfg.bootCfgAddress         = (uint32_t)CSL_BOOT_CFG_REGS;
#if defined(DEVICE_K2H)
    rootSlaveCfg.slaveCoreId            = ROOT_SLAVE_COREID(0, 8);
#elif defined (DEVICE_K2L)
    rootSlaveCfg.slaveCoreId            = ROOT_SLAVE_COREID(0, 4);
#else
#error "Error: Unsupported Device"
#endif
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
        printf ("Error: Unable to create the slave root domain [Error code %d]\n", errCode);
        return -1;
    }
    printf ("Debug: Slave root domain [%p] created successfully\n", rootSlaveHandle);

    /* Launch the ROOT thread */
	errCode = pthread_create (&rootSlaveThread, NULL, Root_slaveTask, NULL);
	if (errCode < 0)
	{
    	printf ("Error: Unable to create the root slave thread [Error code %d]\n", errCode);
        return -1;
	}

    /* Wait for the root thread to terminate. */
    pthread_join (rootSlaveThread, NULL);

    /* Control should never come here. The root thread should always be executing */
    return 0;
}


