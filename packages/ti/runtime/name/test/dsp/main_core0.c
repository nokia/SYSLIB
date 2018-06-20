/*
 *   @file  main_core0.c
 *
 *   @brief
 *      Unit Test code which executes on Core0
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
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <ti/sysbios/family/c64p/Hwi.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/family/c64p/EventCombiner.h>

/* IPC Include Files  */
#include <ti/ipc/Ipc.h>
#include <ti/ipc/SharedRegion.h>

/* PDK Include Files. */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/csl/csl_chip.h>
#include <ti/csl/csl_cacheAux.h>
#include <ti/csl/csl_device_interrupt.h>

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
 *      The function is used to test the basic API
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0
 */
static int32_t Test_basic (void)
{
    Name_ResourceCfg    nameResourceCfg;
    int32_t             errCode;

    /*******************************************************************************
     * Name creation:
     *******************************************************************************/

    /* Initialize the named resource configuration. */
    memset ((void *)&nameResourceCfg, 0, sizeof(Name_ResourceCfg));

    /* Populate the configuration: */
    strcpy (nameResourceCfg.name, "Testing");
    nameResourceCfg.handle1 = 100;
    nameResourceCfg.handle2 = 200;
    if (Name_createResource (globalNameDatabaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB, &nameResourceCfg, &errCode) < 0)
    {
        System_printf ("Error: Unable to create the name resource [Error code %d]\n", errCode);
        return -1;
    }
    System_printf ("Debug: Name resource '%s' created successfully \n", nameResourceCfg.name);

    /*******************************************************************************
     * Name find:
     *******************************************************************************/

    /* Reset the name resource configuration */
    memset ((void *)&nameResourceCfg, 0, sizeof(Name_ResourceCfg));

    /* Find the name resource */
    if (Name_findResource (globalNameDatabaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB, "Testing", &nameResourceCfg, &errCode) < 0)
    {
        System_printf ("Error: Unable to find the name resource [Error code %d]\n", errCode);
        return -1;
    }
    System_printf ("Debug: Name resource '%s' found successfully \n", nameResourceCfg.name);

    /* Sanity Check: Ensure that the configuration is valid */
    if ((nameResourceCfg.handle1 != 100) || (nameResourceCfg.handle2 != 200))
    {
        System_printf ("Error: Invalid value detected [%d %d]\n", nameResourceCfg.handle1, nameResourceCfg.handle2);
        return -1;
    }

    /*******************************************************************************
     * Name modification:
     *******************************************************************************/

    /* Modify the named resource */
    nameResourceCfg.handle1 = 0xdead;
    nameResourceCfg.handle2 = 0xbeef;

    if (Name_modifyResource (globalNameDatabaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB, &nameResourceCfg, &errCode) < 0)
    {
        System_printf ("Error: Unable to modify the name resource [Error code %d]\n", errCode);
        return -1;
    }
    System_printf ("Debug: Name resource '%s' modified successfully \n", nameResourceCfg.name);

    /* Reset the name resource configuration */
    memset ((void *)&nameResourceCfg, 0, sizeof(Name_ResourceCfg));

    /* Find the name resource */
    if (Name_findResource (globalNameDatabaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB, "Testing", &nameResourceCfg, &errCode) < 0)
    {
        System_printf ("Error: Unable to find the name resource [Error code %d]\n", errCode);
        return -1;
    }
    System_printf ("Debug: Name resource '%s' found successfully \n", nameResourceCfg.name);

    /* Sanity Check: Ensure that the configuration is valid */
    if ((nameResourceCfg.handle1 != 0xdead) || (nameResourceCfg.handle2 != 0xbeef))
    {
        System_printf ("Error: Invalid value detected [%x %x]\n", nameResourceCfg.handle1, nameResourceCfg.handle2);
        return -1;
    }

    /*******************************************************************************
     * Name deletion:
     *******************************************************************************/

    /* Delete the name */
    if (Name_deleteResource (globalNameDatabaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB, "Testing", &errCode) < 0)
    {
        System_printf ("Error: Unable to delete the name resource [Error code %d]\n", errCode);
        return -1;
    }
    System_printf ("Debug: Name resource '%s' deleted successfully \n", nameResourceCfg.name);

    /* Find the name resource */
    if (Name_findResource (globalNameDatabaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB, "Testing", &nameResourceCfg, &errCode) == 0)
    {
        System_printf ("Error: Name resource '%s' found successfully after deletion\n", nameResourceCfg.name);
        return -1;
    }
    if (errCode != NAME_ENOTFOUND)
    {
        System_printf ("Error: Unable to find the name resource but invalid error code %d\n", errCode);
        return -1;
    }
    System_printf ("Debug: Basic name resource testing passed\n");
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to test the operation of multiple database
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0
 */
static int32_t Test_multipleDatabase (void)
{
    Name_DatabaseCfg        databaseCfg;
    Name_DBHandle           privateNameDatabaseHandle1;
    Name_DBHandle           privateNameDatabaseHandle2;
    Name_ResourceCfg        nameResourceCfg;
    int32_t                 errCode;

    /* Initialize the database configuration */
    memset ((void *)&databaseCfg, 0, sizeof(Name_DatabaseCfg));

    /* Populate the configuration: */
    databaseCfg.instanceId                        = 45;
    databaseCfg.realm                             = Name_ExecutionRealm_DSP;
    strcpy (databaseCfg.owner, "LTE9A_L2_TEST1");
    databaseCfg.dspCfg.baseNamedResourceAddress   = LTE2_DDR3_NAMED_RESOURCE;
    databaseCfg.dspCfg.sizeNamedResourceMemory    = LTE2_DDR3_NAMED_RESOURCE_LEN;
    databaseCfg.dspCfg.initNamedResourceDatabase  = 1;
    databaseCfg.dspCfg.malloc                     = Name_osalDBMalloc;
    databaseCfg.dspCfg.free                       = Name_osalDBFree;
    databaseCfg.dspCfg.enterCS                    = Name_osalEnterMultipleCoreCS;
    databaseCfg.dspCfg.exitCS                     = Name_osalExitMultipleCoreCS;
    databaseCfg.dspCfg.beginMemAccess             = Name_osalBeginMemAccess;
    databaseCfg.dspCfg.endMemAccess               = Name_osalEndMemAccess;

    /* Create the private database */
    privateNameDatabaseHandle1 = Name_createDatabase (&databaseCfg, &errCode);
    if (privateNameDatabaseHandle1 == NULL)
	{
	    System_printf ("Error: Name database creation failed [Error code %d]\n", errCode);
	    return -1;
    }
    System_printf ("Debug: Name database created successfully [Handle %p]\n", privateNameDatabaseHandle1);

    /* Initialize the database configuration */
    memset ((void *)&databaseCfg, 0, sizeof(Name_DatabaseCfg));

    /* Populate the configuration: */
    databaseCfg.instanceId                        = 21;
    databaseCfg.realm                             = Name_ExecutionRealm_DSP;
    strcpy (databaseCfg.owner, "LTE9A_L2_TEST2");
    databaseCfg.dspCfg.baseNamedResourceAddress   = LTE2_DDR3_NAMED_RESOURCE;
    databaseCfg.dspCfg.sizeNamedResourceMemory    = LTE2_DDR3_NAMED_RESOURCE_LEN;
    databaseCfg.dspCfg.initNamedResourceDatabase  = 1;
    databaseCfg.dspCfg.malloc                     = Name_osalDBMalloc;
    databaseCfg.dspCfg.free                       = Name_osalDBFree;
    databaseCfg.dspCfg.enterCS                    = Name_osalEnterMultipleCoreCS;
    databaseCfg.dspCfg.exitCS                     = Name_osalExitMultipleCoreCS;
    databaseCfg.dspCfg.beginMemAccess             = Name_osalBeginMemAccess;
    databaseCfg.dspCfg.endMemAccess               = Name_osalEndMemAccess;

    /* Create the private database */
    privateNameDatabaseHandle2 = Name_createDatabase (&databaseCfg, &errCode);
    if (privateNameDatabaseHandle2 == NULL)
	{
	    System_printf ("Error: Name database creation failed [Error code %d]\n", errCode);
	    return -1;
    }
    System_printf ("Debug: Name database created successfully [Handle %p]\n", privateNameDatabaseHandle2);

    /* Initialize the name resource configuration.  */
    memset ((void *)&nameResourceCfg, 0, sizeof(Name_ResourceCfg));

    /* Populate the configuration: */
    strcpy (nameResourceCfg.name, "Private-Test");
    nameResourceCfg.handle1 = 0xdead;
    nameResourceCfg.handle2 = 0xdead;
    if (Name_createResource (privateNameDatabaseHandle1, Name_ResourceBucket_INTERNAL_SYSLIB, &nameResourceCfg, &errCode) < 0)
    {
        System_printf ("Error: Unable to create the name resource [Error code %d]\n", errCode);
        return -1;
    }
    System_printf ("Debug: Name resource '%s' created successfully in private database\n", nameResourceCfg.name);

    /* Sanity Check: Ensure that the name is not visible in the global name database */
    if (Name_findResource (globalNameDatabaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB, "Private-Test",
                           &nameResourceCfg, &errCode) == 0)
    {
        System_printf ("Error: Private name '%s' leaks into the global database\n", nameResourceCfg.name);
        return -1;
    }

    /* Sanity Check: Ensure that the name is visible in the private name database */
    if (Name_findResource (privateNameDatabaseHandle1, Name_ResourceBucket_INTERNAL_SYSLIB, "Private-Test",
                           &nameResourceCfg, &errCode) < 0)
    {
        System_printf ("Error: Private name is not found in the private database\n", nameResourceCfg.name);
        return -1;
    }

    /* Purge the private database: */
    if (Name_purgeDatabase (privateNameDatabaseHandle1, Name_ResourceBucket_INTERNAL_SYSLIB, NULL, &errCode) < 0)
    {
        System_printf ("Error: Unable to purge the private database [Error code %d]\n", errCode);
        return -1;
    }
    System_printf ("Debug: Private database successfully purged\n");

    /* Sanity Check: Ensure that the entry has been deleted */
    if (Name_findResource (privateNameDatabaseHandle1, Name_ResourceBucket_INTERNAL_SYSLIB, "Private-Test",
                           &nameResourceCfg, &errCode) == 0)
    {
        System_printf ("Error: Private database was not purged\n", nameResourceCfg.name);
        return -1;
    }
    if (errCode != NAME_ENOTFOUND)
    {
        System_printf ("Error: Private database purged but invalid error code %d\n", errCode);
        return -1;
    }
    System_printf ("Debug: Basic purging test passed.\n");

    /*******************************************************************************************
     * Purging with different owners:
     *******************************************************************************************/

    /* Initialize the name resource configuration.  */
    memset ((void *)&nameResourceCfg, 0, sizeof(Name_ResourceCfg));

    /* Populate the configuration: */
    strcpy (nameResourceCfg.name, "Private-Test");
    nameResourceCfg.handle1 = 0xdead;
    nameResourceCfg.handle2 = 0xdead;
    if (Name_createResource (privateNameDatabaseHandle1, Name_ResourceBucket_INTERNAL_SYSLIB, &nameResourceCfg, &errCode) < 0)
    {
        System_printf ("Error: Unable to create the name resource [Error code %d]\n", errCode);
        return -1;
    }
    System_printf ("Debug: Name resource '%s' created successfully in private database\n", nameResourceCfg.name);

    /* Sanity Check: Ensure that the name is visible in both the databases */
    if (Name_findResource (privateNameDatabaseHandle2, Name_ResourceBucket_INTERNAL_SYSLIB, "Private-Test",
                           &nameResourceCfg, &errCode) < 0)
    {
        System_printf ("Error: Private name is not found in the private database\n", nameResourceCfg.name);
        return -1;
    }
    System_printf ("Debug: Name resource '%s' found in both database\n", nameResourceCfg.name);

    /* Purge the private database: Using a different owner name */
    if (Name_purgeDatabase (privateNameDatabaseHandle2, Name_ResourceBucket_INTERNAL_SYSLIB, "LTE9A_L2_TEST2", &errCode) < 0)
    {
        System_printf ("Error: Unable to purge the private database [Error code %d]\n", errCode);
        return -1;
    }

    /* Sanity Check: Ensure that the name is still visible. Purging with a different owner name will not go through  */
    if (Name_findResource (privateNameDatabaseHandle2, Name_ResourceBucket_INTERNAL_SYSLIB, "Private-Test",
                           &nameResourceCfg, &errCode) < 0)
    {
        System_printf ("Error: Private name '%s' is not found after purge with a different owner name\n", nameResourceCfg.name);
        return -1;
    }
    System_printf ("Debug: Purge with different owner name passed\n");

    /* Purge the private database: Using the same owner name */
    if (Name_purgeDatabase (privateNameDatabaseHandle2, Name_ResourceBucket_INTERNAL_SYSLIB, "LTE9A_L2_TEST1", &errCode) < 0)
    {
        System_printf ("Error: Unable to purge the private database [Error code %d]\n", errCode);
        return -1;
    }

    /* Sanity Check: Ensure that the name is no longer valid.  */
    if (Name_findResource (privateNameDatabaseHandle2, Name_ResourceBucket_INTERNAL_SYSLIB, "Private-Test",
                           &nameResourceCfg, &errCode) == 0)
    {
        System_printf ("Error: Private name '%s' is found after purge with the same owner name\n", nameResourceCfg.name);
        return -1;
    }
    System_printf ("Debug: Purge with same owner name passed\n");

    /* Delete the private database: */
    if (Name_deleteDatabase (privateNameDatabaseHandle1, &errCode) < 0)
    {
        System_printf ("Error: Unable to delete the private database [Error code %d]\n", errCode);
        return -1;
    }
    if (Name_deleteDatabase (privateNameDatabaseHandle2, &errCode) < 0)
    {
        System_printf ("Error: Unable to delete the private database [Error code %d]\n", errCode);
        return -1;
    }
    System_printf ("Debug: Private database successfully deleted\n");
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to stress test the resource API
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0
 */
static int32_t Test_stressResource (void)
{
    Name_ResourceCfg    nameResourceCfg;
    int32_t             errCode;
    int32_t             index;

    /*******************************************************************************
     * Stress Testing: Core0 creates N number of resources.
     *******************************************************************************/
    for (index = 0; index < TEST_MAX_RESOURCES; index++)
    {
        /* Initialize the named resource configuration. */
        memset ((void *)&nameResourceCfg, 0, sizeof(Name_ResourceCfg));

        /* Populate the configuration: */
        sprintf (nameResourceCfg.name, "Stress-Test%d", index);
        nameResourceCfg.handle1 = 1000 + index;
        nameResourceCfg.handle2 = 2000 + index;

        /* Create the named resource. */
        if (Name_createResource (globalNameDatabaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB, &nameResourceCfg, &errCode) < 0)
        {
            System_printf ("Error: Unable to create the name resource '%s' [Error code %d]\n", nameResourceCfg.name, errCode);
            return -1;
        }
    }
    System_printf ("Debug: Successfully created %d resources\n", index);

    /*******************************************************************************
     * Stress Testing: Indicate to Core1 that N resources have been created
     *******************************************************************************/
    memset ((void *)&nameResourceCfg, 0, sizeof(Name_ResourceCfg));

    /* Populate the configuration: */
    strcpy (nameResourceCfg.name, "Notify_Core1");
    nameResourceCfg.handle1 = 0xbabe;
    nameResourceCfg.handle2 = 0xbabe;
    if (Name_createResource (globalNameDatabaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB, &nameResourceCfg, &errCode) < 0)
    {
        System_printf ("Error: Unable to create the name resource [Error code %d]\n", errCode);
        return -1;
    }

    /* Loop around till Core1 has modified N resources */
    while (1)
    {
        /* Find the name resource */
        if (Name_findResource (globalNameDatabaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB, "Notify_Core0",
                               &nameResourceCfg, &errCode) == 0)
            break;

        /* Relinquish time */
        Task_sleep(1);
    }
    System_printf ("Debug: Core0 has been notified; validating resource modifications \n");

    /*******************************************************************************
     * Stress Testing: Core0 validates N number of resources modifications & deletes
     * them.
     *******************************************************************************/
    for (index = 0; index < TEST_MAX_RESOURCES; index++)
    {
        /* Initialize the named resource configuration. */
        memset ((void *)&nameResourceCfg, 0, sizeof(Name_ResourceCfg));

        /* Construct the name. */
        sprintf (nameResourceCfg.name, "Stress-Test%d", index);
        if (Name_findResource (globalNameDatabaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB, nameResourceCfg.name,
                               &nameResourceCfg, &errCode) < 0)
        {
            System_printf ("Error: Unable to find name '%s' [Error code %d]\n", nameResourceCfg.name, errCode);
            return -1;
        }

        /* Validate the configuration: */
        if ((nameResourceCfg.handle1 != (6000 + index)) || (nameResourceCfg.handle2 != (8000 + index)))
        {
            System_printf ("Error: Resource Validation for '%s' failed [%d %d]\n",
                            nameResourceCfg.name, nameResourceCfg.handle1, nameResourceCfg.handle2);
            return -1;
        }

        /* Delete the name */
        if (Name_deleteResource (globalNameDatabaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB, nameResourceCfg.name, &errCode) < 0)
        {
            System_printf ("Error: Unable to delete name '%s' [Error code %d]\n", nameResourceCfg.name, errCode);
            return -1;
        }
    }
    System_printf ("Debug: Successfully validated and deleted %d resources\n", index);

    /*******************************************************************************
     * Stress Testing: Indicate to Core1 that N resources have been deleted
     *******************************************************************************/
    memset ((void *)&nameResourceCfg, 0, sizeof(Name_ResourceCfg));

    /* Populate the configuration: */
    strcpy (nameResourceCfg.name, "Notify_Core1");
    nameResourceCfg.handle1 = 0xdead;
    nameResourceCfg.handle2 = 0xdead;
    if (Name_modifyResource (globalNameDatabaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB, &nameResourceCfg, &errCode) < 0)
    {
        System_printf ("Error: Unable to modify the name resource [Error code %d]\n", errCode);
        return -1;
    }

    /* Delete the global database: Core1 is now responsible for purging all the resources. */
    if (Name_deleteDatabase (globalNameDatabaseHandle, &errCode) < 0)
    {
        System_printf ("Error: Unable to delete the global database [Error code %d]\n", errCode);
        return -1;
    }
    System_printf ("Debug: Global database successfully deleted\n");
    return 0;
}

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
    if (Test_basic() < 0)
        return;

    if (Test_multipleDatabase() < 0)
        return;

    if (Test_stressResource() < 0)
        return;

    System_printf ("Debug: Unit Testing passed\n");
    return;
}

/**
 *  @b Description
 *  @n
 *      System Initialization Code.
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t Test_systemInit (void)
{
    int32_t                 errCode;
    Resmgr_SystemCfg        sysConfig;
    Task_Params             taskParams;
    Name_DatabaseCfg        databaseCfg;

    /* Initialize the system configuration. */
    memset ((void *)&sysConfig, 0, sizeof(Resmgr_SystemCfg));

    /* Populate the configuration.*/
    sysConfig.realm                             = Resmgr_ExecutionRealm_DSP;
    strcpy (sysConfig.rmClient, "Rm_LTE9A_L2");
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
	    return -1;
    }
    System_printf ("Debug: SYSRM initialized successfully [Handle %p]\n", handleSysCfg);

    /* Initialize the database configuration */
    memset ((void *)&databaseCfg, 0, sizeof(Name_DatabaseCfg));

    /* Populate the configuration: */
    databaseCfg.instanceId                        = 1;
    databaseCfg.realm                             = Name_ExecutionRealm_DSP;
    strcpy (databaseCfg.owner, "LTE9A_L2");
    databaseCfg.dspCfg.baseNamedResourceAddress   = LTE1_DDR3_NAMED_RESOURCE;
    databaseCfg.dspCfg.sizeNamedResourceMemory    = LTE1_DDR3_NAMED_RESOURCE_LEN;
    databaseCfg.dspCfg.initNamedResourceDatabase  = 1;
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
	    return -1;
    }
    System_printf ("Debug: Name database created successfully [Handle %p]\n", globalNameDatabaseHandle);

    /* Launch the name client task: */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 16*1024;
    taskParams.priority  = 2;
    Task_create(Test_nameClientTask, &taskParams, NULL);

    return 0;
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
    /* Initialize the system modules */
    if (Test_systemInit() < 0)
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

    /* Initialize TSCL register */
    TSCL = 0;

    /* Enable the caches. */
	CACHE_setL1DSize (CACHE_L1_32KCACHE);
	CACHE_setL1PSize (CACHE_L1_32KCACHE);

    /* Initialize the heap in shared memory. Using IPC module to do that */
    Ipc_start();

    /* System Initialization Task: */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 16*1024;
    Task_create(Test_sysInitTask, &taskParams, NULL);

    /* Start BIOS */
    BIOS_start();
}

