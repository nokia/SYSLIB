/**
 *   @file  main_core1.c
 *
 *   @brief
 *      Unit Test code which executes on Core1
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
#include <xdc/cfg/global.h>
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Memory.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <ti/sysbios/family/c64p/Hwi.h>

/* IPC Include Files for Shared Memory Allocation. */
#include <ti/ipc/Ipc.h>
#include <ti/ipc/SharedRegion.h>

/* MCSDK Include Files. */
#include <ti/csl/csl_chip.h>
#include <ti/csl/csl_cacheAux.h>
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/drv/rm/rm.h>

/* SYSLIB Include Files.  */
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/name/name.h>
#include <ti/runtime/name/name_db.h>

/**********************************************************************
 *********************** Unit Test Local Definitions ******************
 **********************************************************************/

/* Maximum number of test resources which are created by the stress test */
#define TEST_MAX_RESOURCES      4096

/**********************************************************************
 ************************ Unit Test Global Variables ******************
 **********************************************************************/

/* Resource Manager Handle: */
Resmgr_SysCfgHandle     handleSysCfg;

/* Name Database Handle: */
Name_DBHandle           globalNameDatabaseHandle;

/* Name client task handle */
Task_Handle             nameClientTask;

/*********************************************************************
 * OSAL Callout Functions:
 *********************************************************************/

/* RESMGR: */
extern void* Resmgr_osalMalloc(Resmgr_MallocMode , uint32_t );
extern void  Resmgr_osalFree(Resmgr_MallocMode , void* , uint32_t );
extern void* Resmgr_osalMallocMemoryRegion(char*, Resmgr_MemRegionType , uint32_t );
extern void  Resmgr_osalFreeMemoryRegion(char*, Resmgr_MemRegionType , void* , uint32_t );
extern void* Resmgr_osalCreateSem (void);
extern void  Resmgr_osalDeleteSem (void*);
extern void  Resmgr_osalPostSem (void*);
extern void  Resmgr_osalPendSem (void*);
extern void  Resmgr_osalBeginMemAccess (void*, uint32_t);
extern void  Resmgr_osalEndMemAccess (void*, uint32_t);

/* Name Database: */
extern void* Name_osalDBMalloc (Name_MallocMode, uint32_t, uint32_t);
extern void  Name_osalDBFree (Name_MallocMode, void* , uint32_t);
extern void* Name_osalEnterMultipleCoreCS (void);
extern void  Name_osalExitMultipleCoreCS (void*);
extern void  Name_osalBeginMemAccess (void*, uint32_t);
extern void  Name_osalEndMemAccess (void*, uint32_t);

/**********************************************************************
 ************************** Unit Test Functions ***********************
 **********************************************************************/

/**
 *  @b Description
 *  @n
 *      Name Client Test Task
 *
 *  @retval
 *      Not Applicable.
 */
static void Test_nameClientTask(UArg arg0, UArg arg1)
{
    Name_ResourceCfg    nameResourceCfg;
    int32_t             errCode;
    int32_t             index;

    /* Loop around till Core0 has created N resources */
    while (1)
    {
        /* Find the name resource */
        if (Name_findResource (globalNameDatabaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB, "Notify_Core1",
                               &nameResourceCfg, &errCode) == 0)
            break;

        /* Relinquish time */
        Task_sleep(1);
    }
    System_printf ("Debug: Core1 has been notified; modifying all the resources. \n");

    /*******************************************************************************
     * Stress Testing: Core1 modifies N number of resources.
     *******************************************************************************/
    for (index = 0; index < TEST_MAX_RESOURCES; index++)
    {
        /* Initialize the named resource configuration. */
        memset ((void *)&nameResourceCfg, 0, sizeof(Name_ResourceCfg));

        /* Populate the configuration: */
        sprintf (nameResourceCfg.name, "Stress-Test%d", index);
        nameResourceCfg.handle1 = 6000 + index;
        nameResourceCfg.handle2 = 8000 + index;

        /* Modify the named resource. */
        if (Name_modifyResource (globalNameDatabaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB, &nameResourceCfg, &errCode) < 0)
        {
            System_printf ("Error: Unable to modified the name resource '%s' [Error code %d]\n", nameResourceCfg.name, errCode);
            return;
        }
    }
    System_printf ("Debug: Successfully modified %d resources\n", index);

    /*******************************************************************************
     * Stress Testing: Indicate to Core0 that N resources have been modified
     *******************************************************************************/
    memset ((void *)&nameResourceCfg, 0, sizeof(Name_ResourceCfg));

    /* Populate the configuration: */
    strcpy (nameResourceCfg.name, "Notify_Core0");
    nameResourceCfg.handle1 = 0xdead;
    nameResourceCfg.handle2 = 0xdead;
    if (Name_createResource (globalNameDatabaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB, &nameResourceCfg, &errCode) < 0)
    {
        System_printf ("Error: Unable to create the name resource [Error code %d]\n", errCode);
        return;
    }
    System_printf ("Debug: Notified Core0\n");

    /*******************************************************************************
     * Stress Testing: Wait for a notification from Core0
     *******************************************************************************/
    while (1)
    {
        /* Find the name resource */
        if (Name_findResource (globalNameDatabaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB, "Notify_Core1",
                               &nameResourceCfg, &errCode) < 0)
        {
            System_printf ("Error: Unable to find the Core1 SYNC name [Error code %d]\n", errCode);
            return;
        }

        /* Have we been asked to proceed? */
        if ((nameResourceCfg.handle1 == 0xdead) && (nameResourceCfg.handle2 == 0xdead))
            break;

        /* Relinquish time */
        Task_sleep(1);
    }
    System_printf ("Debug: Core1 has been notified; validating all the names have been deleted \n");

    /*******************************************************************************
     * Stress Testing: Validate all the resources have been deleted
     *******************************************************************************/
    for (index = 0; index < TEST_MAX_RESOURCES; index++)
    {
        /* Construct the name */
        sprintf (nameResourceCfg.name, "Stress-Test%d", index);

        /* Find the name resource */
        if (Name_findResource (globalNameDatabaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB, nameResourceCfg.name,
                               &nameResourceCfg, &errCode) == 0)
        {
            System_printf ("Error: Found resource '%s' after deletion\n", nameResourceCfg.name);
            return;
        }
    }

    /*******************************************************************************
     * Stress Testing: Purge
     *******************************************************************************/
    for (index = 0; index < TEST_MAX_RESOURCES; index++)
    {
        /* Initialize the named resource configuration. */
        memset ((void *)&nameResourceCfg, 0, sizeof(Name_ResourceCfg));

        /* Populate the configuration: */
        sprintf (nameResourceCfg.name, "Purge-Test%d", index);
        nameResourceCfg.handle1 = 1 + index;
        nameResourceCfg.handle2 = 2 + index;

        /* Create the named resource. */
        if (Name_createResource (globalNameDatabaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB, &nameResourceCfg, &errCode) < 0)
        {
            System_printf ("Error: Unable to create the name resource '%s' [Error code %d]\n", nameResourceCfg.name, errCode);
            return;
        }
    }
    System_printf ("Debug: Successfully created %d resources for purging\n", index);

    /* Purge the global database: */
    if (Name_purgeDatabase (globalNameDatabaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB, NULL, &errCode) < 0)
    {
        System_printf ("Error: Unable to purge the global database [Error code %d]\n", errCode);
        return;
    }
    System_printf ("Debug: Global database successfully purged\n");

    /* Delete the global database: */
    if (Name_deleteDatabase (globalNameDatabaseHandle, &errCode) < 0)
    {
        System_printf ("Error: Unable to delete the global database [Error code %d]\n", errCode);
        return;
    }
    System_printf ("Debug: Global database successfully deleted\n");

    /* All the tests completed successfully. */
    System_printf ("Debug: Unit Testing passed\n");
    return;
}

/**
 *  @b Description
 *  @n
 *      System Initialization Task which initializes the various
 *      components in the system.
 *
 *  @retval
 *      Not Applicable.
 */
static void Test_sysInitTask(UArg arg0, UArg arg1)
{
    int32_t                 errCode;
    Resmgr_SystemCfg        sysConfig;
    Task_Params             taskParams;
    Name_DatabaseCfg        databaseCfg;

    /* Initialize the system configuration. */
    memset ((void *)&sysConfig, 0, sizeof(Resmgr_SystemCfg));

    /* Populate the configuration.*/
    sysConfig.realm                             = Resmgr_ExecutionRealm_DSP;
    strcpy (sysConfig.rmClient, "Rm_LTE9A_L1");
    strcpy (sysConfig.rmServer, "Rm_Server");
    sysConfig.coreId                            = DNUM;
    sysConfig.malloc                            = Resmgr_osalMalloc;
    sysConfig.free                              = Resmgr_osalFree;
    sysConfig.mallocMemoryRegion                = Resmgr_osalMallocMemoryRegion;
    sysConfig.freeMemoryRegion                  = Resmgr_osalFreeMemoryRegion;
    sysConfig.createSem                         = Resmgr_osalCreateSem;
    sysConfig.pendSem                           = Resmgr_osalPendSem;
    sysConfig.postSem                           = Resmgr_osalPostSem;
    sysConfig.deleteSem                         = Resmgr_osalDeleteSem;
    sysConfig.beginMemAccess                    = Resmgr_osalBeginMemAccess;
    sysConfig.endMemAccess                      = Resmgr_osalEndMemAccess;
    sysConfig.dspSystemCfg.armCoreId        	= SYSLIB_ARM_CORE_ID;
    sysConfig.dspSystemCfg.sourceId         	= 17;
    sysConfig.dspSystemCfg.sharedMemAddress 	= DDR3_SYSLIB_RESMGR_RSVD;
    sysConfig.dspSystemCfg.sizeSharedMemory 	= DDR3_SYSLIB_RESMGR_RSVD_LEN;

    /* Initialize the system configuration. */
    handleSysCfg = Resmgr_init(&sysConfig, &errCode);
    if (handleSysCfg == NULL)
	{
	    System_printf ("Error: SYSRM initialization failed with error code %d\n", errCode);
	    return;
    }
    System_printf ("Debug: SYSRM initialized successfully [Handle %p]\n", handleSysCfg);

    /* Initialize the database configuration */
    memset ((void *)&databaseCfg, 0, sizeof(Name_DatabaseCfg));

    /* Populate the configuration: */
    databaseCfg.instanceId                        = 32;
    databaseCfg.realm                             = Name_ExecutionRealm_DSP;
    strcpy (databaseCfg.owner, "LTE9A_L1");
    databaseCfg.dspCfg.baseNamedResourceAddress   = LTE1_DDR3_NAMED_RESOURCE;
    databaseCfg.dspCfg.sizeNamedResourceMemory    = LTE1_DDR3_NAMED_RESOURCE_LEN;
    databaseCfg.dspCfg.initNamedResourceDatabase  = 0;
    databaseCfg.dspCfg.malloc                     = Name_osalDBMalloc;
    databaseCfg.dspCfg.free                       = Name_osalDBFree;
    databaseCfg.dspCfg.enterCS                    = Name_osalEnterMultipleCoreCS;
    databaseCfg.dspCfg.exitCS                     = Name_osalExitMultipleCoreCS;
    databaseCfg.dspCfg.beginMemAccess             = Name_osalBeginMemAccess;
    databaseCfg.dspCfg.endMemAccess               = Name_osalEndMemAccess;

    /* Create the global database */
    globalNameDatabaseHandle = Name_createDatabase (&databaseCfg, &errCode);
    if (globalNameDatabaseHandle == NULL)
	{
	    System_printf ("Error: Name database creation failed [Error code %d]\n", errCode);
	    return;
    }
    System_printf ("Debug: Name database created successfully [Handle %p]\n", globalNameDatabaseHandle);

    /* Launch the name client task: */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 16*1024;
    taskParams.priority  = 2;
    Task_create(Test_nameClientTask, &taskParams, NULL);

    return;
}

/**
 *  @b Description
 *  @n
 *      Entry point into the test code.
 *
 *  @retval
 *      Not Applicable.
 */
void main (void)
{
    Task_Params taskParams;

    /* Enable the caches. */
	CACHE_setL1DSize (CACHE_L1_32KCACHE);
	CACHE_setL1PSize (CACHE_L1_32KCACHE);

    /* Enable the timestamp */
    TSCL = 0;

    /* Initialize the heap in shared memory. Using IPC module to do that */
    Ipc_start();

    /* System Initialization Task: */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 16*1024;
    Task_create(Test_sysInitTask, &taskParams, NULL);

    /* Start BIOS */
    BIOS_start();
}

