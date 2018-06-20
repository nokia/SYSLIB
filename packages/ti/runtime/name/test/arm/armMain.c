/*
 *   @file  armMain.c
 *
 *   @brief
 *      Unit Test code which executes on ARM
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
#include <pthread.h>
#include <signal.h>
#include <getopt.h>

/* MCSDK Include files. */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/runtime/hplib/hplib.h>

/* Device specific dependencies. */
#ifdef DEVICE_K2H
#include <ti/drv/qmss/device/k2h/src/qmss_device.c>
#include <ti/drv/cppi/device/k2h/src/cppi_device.c>
#elif defined(DEVICE_K2K)
#include <ti/drv/qmss/device/k2k/src/qmss_device.c>
#include <ti/drv/cppi/device/k2k/src/cppi_device.c>
#elif defined(DEVICE_K2L)
#include <ti/drv/qmss/device/k2l/src/qmss_device.c>
#include <ti/drv/cppi/device/k2l/src/cppi_device.c>
#else
#error "Error: Unsupported Device"
#endif

/* SYSLIB Include Files */
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/name/name.h>
#include <ti/runtime/name/name_db.h>

/**********************************************************************
 *********************** Unit Test Local Definitions ******************
 **********************************************************************/

/* Maximum number of test resources which are created by the stress test */
#define TEST_MAX_RESOURCES      1000

/**********************************************************************
 ************************ Unit Test Global Variables ******************
 **********************************************************************/

/* Resource Manager Handle: */
Resmgr_SysCfgHandle     handleSysCfg;

/* Name Database Handle: */
Name_DBHandle           globalNameDatabaseHandle;

/*********************************************************************
 * OSAL Callout Functions:
 *********************************************************************/

/* RESMGR: */
extern void* Resmgr_osalMalloc(Resmgr_MallocMode , uint32_t );
extern void  Resmgr_osalFree(Resmgr_MallocMode , void* , uint32_t );
extern void* Resmgr_osalMallocMemoryRegion(char*, Resmgr_MemRegionType , uint32_t );
extern void  Resmgr_osalFreeMemoryRegion(char*, Resmgr_MemRegionType , void* , uint32_t );
extern void  Resmgr_osalBeginMemAccess (void*, uint32_t);
extern void  Resmgr_osalEndMemAccess (void*, uint32_t);
extern void* Resmgr_osalCreateSem (void);
extern void  Resmgr_osalDeleteSem (void*);
extern void  Resmgr_osalPostSem (void*);
extern void  Resmgr_osalPendSem (void*);

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
        printf ("Error: Unable to create the name resource [Error code %d]\n", errCode);
        return -1;
    }
    printf ("Debug: Name resource '%s' created successfully \n", nameResourceCfg.name);

    /*******************************************************************************
     * Name find:
     *******************************************************************************/

    /* Reset the name resource configuration */
    memset ((void *)&nameResourceCfg, 0, sizeof(Name_ResourceCfg));

    /* Find the name resource */
    if (Name_findResource (globalNameDatabaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB, "Testing", &nameResourceCfg, &errCode) < 0)
    {
        printf ("Error: Unable to find the name resource [Error code %d]\n", errCode);
        return -1;
    }
    printf ("Debug: Name resource '%s' found successfully \n", nameResourceCfg.name);

    /* Sanity Check: Ensure that the configuration is valid */
    if ((nameResourceCfg.handle1 != 100) || (nameResourceCfg.handle2 != 200))
    {
        printf ("Error: Invalid value detected [%d %d]\n", nameResourceCfg.handle1, nameResourceCfg.handle2);
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
        printf ("Error: Unable to modify the name resource [Error code %d]\n", errCode);
        return -1;
    }
    printf ("Debug: Name resource '%s' modified successfully \n", nameResourceCfg.name);

    /* Reset the name resource configuration */
    memset ((void *)&nameResourceCfg, 0, sizeof(Name_ResourceCfg));

    /* Find the name resource */
    if (Name_findResource (globalNameDatabaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB, "Testing", &nameResourceCfg, &errCode) < 0)
    {
        printf ("Error: Unable to find the name resource [Error code %d]\n", errCode);
        return -1;
    }
    printf ("Debug: Name resource '%s' found successfully \n", nameResourceCfg.name);

    /* Sanity Check: Ensure that the configuration is valid */
    if ((nameResourceCfg.handle1 != 0xdead) || (nameResourceCfg.handle2 != 0xbeef))
    {
        printf ("Error: Invalid value detected [%x %x]\n", nameResourceCfg.handle1, nameResourceCfg.handle2);
        return -1;
    }

    /*******************************************************************************
     * Name deletion:
     *******************************************************************************/

    /* Delete the name */
    if (Name_deleteResource (globalNameDatabaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB, "Testing", &errCode) < 0)
    {
        printf ("Error: Unable to delete the name resource [Error code %d]\n", errCode);
        return -1;
    }
    printf ("Debug: Name resource '%s' deleted successfully \n", nameResourceCfg.name);

    /* Find the name resource */
    if (Name_findResource (globalNameDatabaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB, "Testing", &nameResourceCfg, &errCode) == 0)
    {
        printf ("Error: Name resource '%s' found successfully after deletion\n", nameResourceCfg.name);
        return -1;
    }
    if (errCode != NAME_ENOTFOUND)
    {
        printf ("Error: Unable to find the name resource but invalid error code %d\n", errCode);
        return -1;
    }
    printf ("Debug: Basic name resource testing passed\n");
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
    databaseCfg.instanceId   = 45;
    databaseCfg.realm        = Name_ExecutionRealm_ARM;
    strcpy (databaseCfg.owner, "LTE9A_L2_TEST1");

    /* Create the private database */
    privateNameDatabaseHandle1 = Name_createDatabase (&databaseCfg, &errCode);
    if (privateNameDatabaseHandle1 == NULL)
	{
	    printf ("Error: Name database creation failed [Error code %d]\n", errCode);
	    return -1;
    }
    printf ("Debug: Name database created successfully [Handle %p]\n", privateNameDatabaseHandle1);

    /* Initialize the database configuration */
    memset ((void *)&databaseCfg, 0, sizeof(Name_DatabaseCfg));

    /* Populate the configuration: */
    databaseCfg.instanceId   = 21;
    databaseCfg.realm        = Name_ExecutionRealm_ARM;
    strcpy (databaseCfg.owner, "LTE9A_L2_TEST2");

    /* Create the private database */
    privateNameDatabaseHandle2 = Name_createDatabase (&databaseCfg, &errCode);
    if (privateNameDatabaseHandle2 == NULL)
	{
	    printf ("Error: Name database creation failed [Error code %d]\n", errCode);
	    return -1;
    }
    printf ("Debug: Name database created successfully [Handle %p]\n", privateNameDatabaseHandle2);

    /* Initialize the name resource configuration.  */
    memset ((void *)&nameResourceCfg, 0, sizeof(Name_ResourceCfg));

    /* Populate the configuration: */
    strcpy (nameResourceCfg.name, "Private-Test");
    nameResourceCfg.handle1 = 0xdead;
    nameResourceCfg.handle2 = 0xdead;
    if (Name_createResource (privateNameDatabaseHandle1, Name_ResourceBucket_INTERNAL_SYSLIB, &nameResourceCfg, &errCode) < 0)
    {
        printf ("Error: Unable to create the name resource [Error code %d]\n", errCode);
        return -1;
    }
    printf ("Debug: Name resource '%s' created successfully in private database\n", nameResourceCfg.name);

    /* Sanity Check: Ensure that the name is not visible in the global name database */
    if (Name_findResource (globalNameDatabaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB, "Private-Test",
                           &nameResourceCfg, &errCode) == 0)
    {
        printf ("Error: Private name '%s' leaks into the global database\n", nameResourceCfg.name);
        return -1;
    }
    printf ("Debug: Name resource did not leak into the global database\n");

    /* Sanity Check: Ensure that the name is visible in the private name database */
    if (Name_findResource (privateNameDatabaseHandle1, Name_ResourceBucket_INTERNAL_SYSLIB, "Private-Test",
                           &nameResourceCfg, &errCode) < 0)
    {
        printf ("Error: Private name is not found in the private database\n");
        return -1;
    }
    printf ("Debug: Name resource is present in the private database\n");

    /* Purge the private database: */
    if (Name_purgeDatabase (privateNameDatabaseHandle1, Name_ResourceBucket_INTERNAL_SYSLIB, NULL, &errCode) < 0)
    {
        printf ("Error: Unable to purge the private database [Error code %d]\n", errCode);
        return -1;
    }
    printf ("Debug: Private database successfully purged\n");

    /* Sanity Check: Ensure that the entry has been deleted */
    if (Name_findResource (privateNameDatabaseHandle1, Name_ResourceBucket_INTERNAL_SYSLIB, "Private-Test",
                           &nameResourceCfg, &errCode) == 0)
    {
        printf ("Error: Private database was not purged\n");
        return -1;
    }
    if (errCode != NAME_ENOTFOUND)
    {
        printf ("Error: Private database purged but invalid error code %d\n", errCode);
        return -1;
    }
    printf ("Debug: Basic purging test passed.\n");
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to stress test resource creation & deletion
 *      between different processes. This is the test code for the parent
 *      process.
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0
 */
static int32_t Test_parentStressResource(void)
{
    Name_ResourceCfg    nameResourceCfg;
    int32_t             errCode;
    int32_t             index;

    /*******************************************************************************
     * Stress Testing: Parent Process creates N number of resources.
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
            printf ("Error: Unable to create the name resource '%s' [Error code %d]\n", nameResourceCfg.name, errCode);
            return -1;
        }
    }
    printf ("Debug: Parent process successfully created %d resources\n", index);

    /*******************************************************************************
     * Stress Testing: Indicate to child process that N resources have been created
     *******************************************************************************/
    memset ((void *)&nameResourceCfg, 0, sizeof(Name_ResourceCfg));

    /* Populate the configuration: */
    strcpy (nameResourceCfg.name, "Notify_Core1");
    nameResourceCfg.handle1 = 0xbabe;
    nameResourceCfg.handle2 = 0xbabe;
    if (Name_createResource (globalNameDatabaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB, &nameResourceCfg, &errCode) < 0)
    {
        printf ("Error: Unable to create the name resource [Error code %d]\n", errCode);
        return -1;
    }

    /* Loop around till child process has modified N resources */
    while (1)
    {
        /* Find the name resource */
        if (Name_findResource (globalNameDatabaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB, "Notify_Core0",
                               &nameResourceCfg, &errCode) == 0)
            break;

        /* Relinquish time */
        usleep(100);
    }
    printf ("Debug: Parent process has been notified; validating resource modifications \n");

    /*******************************************************************************
     * Stress Testing: Parent process validates N number of resources modifications
     * & deletes them.
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
            printf ("Error: Unable to find name '%s' [Error code %d]\n", nameResourceCfg.name, errCode);
            return -1;
        }

        /* Validate the configuration: */
        if ((nameResourceCfg.handle1 != (6000 + index)) || (nameResourceCfg.handle2 != (8000 + index)))
        {
            printf ("Error: Resource Validation for '%s' failed [%d %d]\n",
                            nameResourceCfg.name, nameResourceCfg.handle1, nameResourceCfg.handle2);
            return -1;
        }

        /* Delete the name */
        if (Name_deleteResource (globalNameDatabaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB, nameResourceCfg.name, &errCode) < 0)
        {
            printf ("Error: Unable to delete name '%s' [Error code %d]\n", nameResourceCfg.name, errCode);
            return -1;
        }
    }
    printf ("Debug: Successfully validated and deleted %d resources\n", index);

    /*******************************************************************************
     * Stress Testing: Indicate to child process that N resources have been deleted
     *******************************************************************************/
    memset ((void *)&nameResourceCfg, 0, sizeof(Name_ResourceCfg));

    /* Populate the configuration: */
    strcpy (nameResourceCfg.name, "Notify_Core1");
    nameResourceCfg.handle1 = 0xdead;
    nameResourceCfg.handle2 = 0xdead;
    if (Name_modifyResource (globalNameDatabaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB, &nameResourceCfg, &errCode) < 0)
    {
        printf ("Error: Unable to modify the name resource [Error code %d]\n", errCode);
        return -1;
    }

    /* Delete the global database: Child process is now responsible for purging all the resources. */
    if (Name_deleteDatabase (globalNameDatabaseHandle, &errCode) < 0)
    {
        printf ("Error: Unable to delete the global database [Error code %d]\n", errCode);
        return -1;
    }
    printf ("Debug: Global database successfully deleted\n");
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to stress test resource creation & deletion
 *      between different processes. This is the test code for the child
 *      process.
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0
 */
static int32_t Test_childStressResource(void)
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
        usleep(100);
    }
    printf ("Debug: Child process has been notified; modifying all the resources. \n");

    /*******************************************************************************
     * Stress Testing: Child process modifies N number of resources.
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
            printf ("Error: Unable to modified the name resource '%s' [Error code %d]\n", nameResourceCfg.name, errCode);
            return -1;
        }
    }
    printf ("Debug: Child process successfully modified %d resources\n", index);

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
        printf ("Error: Unable to create the name resource [Error code %d]\n", errCode);
        return -1;
    }
    printf ("Debug: Notified Parent process\n");

    /*******************************************************************************
     * Stress Testing: Wait for a notification from Core0
     *******************************************************************************/
    while (1)
    {
        /* Find the name resource */
        if (Name_findResource (globalNameDatabaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB, "Notify_Core1",
                               &nameResourceCfg, &errCode) < 0)
        {
            printf ("Error: Unable to find the Child Process SYNC name [Error code %d]\n", errCode);
            return -1;
        }

        /* Have we been asked to proceed? */
        if ((nameResourceCfg.handle1 == 0xdead) && (nameResourceCfg.handle2 == 0xdead))
            break;

        /* Relinquish time */
        usleep(10);
    }
    printf ("Debug: Child process has been notified; validating all the names have been deleted \n");

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
            printf ("Error: Found resource '%s' after deletion\n", nameResourceCfg.name);
            return -1;
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
            printf ("Error: Unable to create the name resource '%s' [Error code %d]\n", nameResourceCfg.name, errCode);
            return -1;
        }
    }
    printf ("Debug: Child process successfully created %d resources for purging\n", index);

    /* Purge the global database: */
    if (Name_purgeDatabase (globalNameDatabaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB, NULL, &errCode) < 0)
    {
        printf ("Error: Unable to purge the global database [Error code %d]\n", errCode);
        return -1;
    }
    printf ("Debug: Global database successfully purged\n");

    /* Delete the global database: */
    if (Name_deleteDatabase (globalNameDatabaseHandle, &errCode) < 0)
    {
        printf ("Error: Unable to delete the global database [Error code %d]\n", errCode);
        return -1;
    }
    printf ("Debug: Global database successfully deleted\n");

    /* All the tests completed successfully. */
    printf ("Debug: Unit Testing passed\n");
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
static void* TestTask(void* arg)
{
    pid_t childPID;

    if (Test_basic() < 0)
        return NULL;

    if (Test_multipleDatabase() < 0)
        return NULL;

    /* Stress Resource Testing: This is initiated between the child and parent process. */
    printf ("Debug: Stress Resource Testing (Forking process)\n");
    childPID = fork ();
    if (childPID >= 0)
    {
        if (childPID == 0)
        {
            if (Test_childStressResource() < 0)
                return NULL;
        }
        else
        {
            if (Test_parentStressResource() < 0)
                return NULL;
        }
    }
    else
    {
        printf ("FATAL Error: fork() failed");
        exit (-1);
    }

    printf ("Debug: Unit Testing passed\n");
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      Entry point into the test code.
 *
 *  @retval
 *      Not Applicable.
 */
int32_t main (void)
{
    int32_t                 errCode;
    Resmgr_SystemCfg        sysConfig;
    Name_DatabaseCfg        databaseCfg;
    pthread_t               testThread;

    /* Initialize the system configuration. */
    memset ((void *)&sysConfig, 0, sizeof(Resmgr_SystemCfg));

    /* Populate the configuration.*/
    sysConfig.realm                 = Resmgr_ExecutionRealm_ARM;
    strcpy (sysConfig.rmClient, "Rm_LTE9A_L2");
    strcpy (sysConfig.rmServer, "Rm_Server");
    sysConfig.coreId                = 8;
    sysConfig.malloc                = Resmgr_osalMalloc;
    sysConfig.free                  = Resmgr_osalFree;
    sysConfig.mallocMemoryRegion    = Resmgr_osalMallocMemoryRegion;
    sysConfig.freeMemoryRegion      = Resmgr_osalFreeMemoryRegion;
    sysConfig.createSem             = Resmgr_osalCreateSem;
    sysConfig.pendSem               = Resmgr_osalPendSem;
    sysConfig.postSem               = Resmgr_osalPostSem;
    sysConfig.deleteSem             = Resmgr_osalDeleteSem;
    sysConfig.beginMemAccess        = Resmgr_osalBeginMemAccess;
    sysConfig.endMemAccess          = Resmgr_osalEndMemAccess;

    /* Initialize the system configuration. */
    handleSysCfg = Resmgr_init(&sysConfig, &errCode);
    if (handleSysCfg == NULL)
	{
	    printf ("Error: SYSRM initialization failed with error code %d\n", errCode);
	    return -1;
    }
    printf ("Debug: SYSRM initialized successfully [Handle %p]\n", handleSysCfg);

    /* Initialize the database configuration */
    memset ((void *)&databaseCfg, 0, sizeof(Name_DatabaseCfg));

    /* Populate the configuration: */
    databaseCfg.instanceId  = 32;
    databaseCfg.realm       = Name_ExecutionRealm_ARM;
    strcpy (databaseCfg.owner, "LTE9A_L2");

    /* Create the global database */
    globalNameDatabaseHandle = Name_createDatabase (&databaseCfg, &errCode);
    if (globalNameDatabaseHandle == NULL)
	{
	    printf ("Error: Name database creation failed [Error code %d]\n", errCode);
	    return -1;
    }
    printf ("Debug: Name database created successfully [Handle %p]\n", globalNameDatabaseHandle);

    /* Create the test task */
    errCode = pthread_create (&testThread, NULL, TestTask, NULL);
    if (errCode < 0)
    {
        printf ("Error: Unable to create the test thread %d\n", errCode);
        return -1;
    }

    /* Blocked till the tests are done. */
    pthread_join (testThread, NULL);

    /* Cleanup the resource manager configuration */
    if (Resmgr_deinit (handleSysCfg, &errCode) < 0)
        printf ("Error: Shutting down the system configuration failed\n");
    else
        printf ("Debug: Shutting down the system configuration passed\n");
    return 0;
}

