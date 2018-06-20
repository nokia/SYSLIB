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

/* Device specific dependencies: */
#ifdef DEVICE_K2H
#include <ti/drv/qmss/device/k2h/src/qmss_device.c>
#include <ti/drv/cppi/device/k2h/src/cppi_device.c>
#elif defined (DEVICE_K2K)
#include <ti/drv/qmss/device/k2k/src/qmss_device.c>
#include <ti/drv/cppi/device/k2k/src/cppi_device.c>
#elif defined (DEVICE_K2L)
#include <ti/drv/qmss/device/k2l/src/qmss_device.c>
#include <ti/drv/cppi/device/k2l/src/cppi_device.c>
#endif

/* SYSLIB Include Files */
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/name/name.h>
#include <ti/runtime/name/name_db.h>
#include <ti/runtime/name/name_proxyClient.h>

/**********************************************************************
 *********************** Unit Test Local Definitions ******************
 **********************************************************************/

/* Maximum number of test resources which are created by the stress test */
#define TEST_MAX_ITERATIONS      100

/**********************************************************************
 ************************ Unit Test Global Variables ******************
 **********************************************************************/

/* Global SYSLIB Handle(s): */
Resmgr_SysCfgHandle     handleSysCfg;
Name_DBHandle           globalNameDatabaseHandle;

/* Global Test Status: */
volatile uint32_t       testComplete = 0;

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

/* Name Proxy/Client: */
extern void* Name_osalMalloc (uint32_t, uint32_t);
extern void  Name_osalFree (void* , uint32_t);
extern void  Name_osalBeginMemAccess (void*, uint32_t);
extern void  Name_osalEndMemAccess (void*, uint32_t);
extern void* Name_osalEnterCS (void);
extern void  Name_osalExitCS (void*);
extern void* Name_osalCreateSem (void);
extern void  Name_osalDeleteSem(void*);
extern void  Name_osalPendSem(void*);
extern void  Name_osalPostSem(void*);

/**********************************************************************
 ************************** Unit Test Functions ***********************
 **********************************************************************/

/**
 *  @b Description
 *  @n
 *      The function is used to synhronize for the specific resource name.
 *
 *  @param[in]  syncName
 *      Resource name to be synchronized
 *
 *  @retval
 *      Not Applicable.
 */
static void NameTest_sync (char* syncName, uint32_t handle1)
{
    int32_t             errCode;
    Name_ResourceCfg    namedResourceCfg;

    /* Loop around till the synchronize name is found. */
    while (1)
    {
        /* Find the named resource. */
        if (Name_findResource (globalNameDatabaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB,
                               syncName, &namedResourceCfg, &errCode) < 0)
        {
            if (errCode != NAME_ENOTFOUND)
            {
                printf ("Error: Find named resource %s failed [Error code %d]\n", syncName, errCode);
                return;
            }

            /* Synhronize name is not found. Try again after some time */
            usleep (100);
            continue;
        }
        /* Perform the handle validation. */
        if (namedResourceCfg.handle1 != handle1)
            printf ("Error: Handle verification %s failed [Received %x Expected %x]\n", syncName, namedResourceCfg.handle1, handle1);
        break;
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to synhronize for the specific resource name.
 *
 *  @param[in]  nameClientHandle
 *      Handle to the name client
 *  @param[in]  resName
 *      Name of the resource to be created and pushed
 *  @param[in]  handle1
 *      Handle1 value to be assigned
 *
 *  @retval
 *      Not Applicable.
 */
static void NameTest_createPushNamedResource
(
    Name_ClientHandle   nameClientHandle,
    char*               resName,
    uint32_t            handle1
)
{
    Name_ResourceCfg    namedResourceCfg;
    int32_t             errCode;

    /* Create a named resource and indicate to the DSP cores that the ARM is now operational */
    memset ((void *)&namedResourceCfg, 0, sizeof(Name_ResourceCfg));

    /* Populate the named resource configuration. */
    strcpy(namedResourceCfg.name, resName);
    namedResourceCfg.handle1  = handle1;

    /* Create the named resource in the ARM realm. */
    if (Name_createResource(globalNameDatabaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB,
                            &namedResourceCfg, &errCode) < 0)
    {
        printf ("Error: Named resource creation '%s' failed (Error %d) \n", resName, errCode);
        return;
    }
    printf ("Debug: Created named resource '%s'.\n", namedResourceCfg.name);

    /* Push the named resource to the DSP realm */
    if (Name_push (nameClientHandle, &namedResourceCfg.name[0],
                   Name_ResourceBucket_INTERNAL_SYSLIB,
                   Name_ResourceOperationType_CREATE, &errCode) < 0)
    {
        printf ("Error: Named resource push '%s' failed (Error %d) \n", resName, errCode);
        return;
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to delete the specific named resource and push it across
 *      to the DSP realm.
 *
 *  @param[in]  nameClientHandle
 *      Handle to the name client
 *  @param[in]  resName
 *      Name of the resource to be created and pushed
 *
 *  @retval
 *      Not Applicable.
 */
static void NameTest_deletePushNamedResource
(
    Name_ClientHandle   nameClientHandle,
    char*               resName
)
{
    int32_t errCode;

    /* Debug Message: */
    printf ("Debug: Deleting named resource '%s'\n", resName);

    /* Delete the named resource in the ARM realm. */
    if (Name_deleteResource(globalNameDatabaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB,
                            resName, &errCode) < 0)
    {
        printf ("Error: Named resource deletion '%s' failed (Error %d) \n", resName, errCode);
        return;
    }

    /* Push the named resource to the DSP realm */
    if (Name_push (nameClientHandle, resName, Name_ResourceBucket_INTERNAL_SYSLIB,
                   Name_ResourceOperationType_DELETE, &errCode) < 0)
    {
        printf ("Error: Delete named resource push '%s' failed (Error %d) \n", resName, errCode);
        return;
    }
    return;
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
    Name_ResourceCfg        namedResourceCfg;
    int32_t                 errCode;
    Name_ClientHandle       nameClientHandle;
    uint32_t                index;
    char                    resourceName[NAME_MAX_CHAR];

    /* Get the name client handle: */
    nameClientHandle = (Name_ClientHandle)arg;

    /* Populate the named resource configuration. */
    memset ((void *)&namedResourceCfg, 0, sizeof(Name_ResourceCfg));
    strcpy(namedResourceCfg.name, "SecretARMResource");
    namedResourceCfg.handle1  = 0xbabe;
    namedResourceCfg.handle2  = 0xface;

    /* Create the PRIVATE named resource in the ARM realm. */
    if (Name_createResource(globalNameDatabaseHandle, Name_ResourceBucket_USER_DEF1,
                            &namedResourceCfg, &errCode) < 0)
    {
        printf ("Error: Named resource creation '%s' failed [Error code %d]\n", namedResourceCfg.name, errCode);
        return NULL;
    }
    printf ("Debug: Created private named resource '%s'.\n", namedResourceCfg.name);

    /* Debug Message: */
    printf ("Debug: Waiting for the DSP cores to be operational [Default Client]\n");
    NameTest_sync ("DSP-Core-Status", 0xDEAD);
    printf ("Debug: DSP Cores are operational\n");

    /* Get the private DSP resource */
    if (Name_get (nameClientHandle, 1, Name_ResourceBucket_USER_DEF1,
                  "SecretDSPResource", &namedResourceCfg, &errCode) < 0)
    {
        printf ("Error: Named resource get of ARM Private name failed [Error code %d]\n", errCode);
        return NULL;
    }
    if ((namedResourceCfg.handle1 != 0x1111) || (namedResourceCfg.handle2 != 0x2222))
    {
        printf ("Debug: Invalid handles detected 0x%x 0x%x\n", namedResourceCfg.handle1, namedResourceCfg.handle2);
        return NULL;
    }
    printf ("Debug: Get DSP Private named resource '%s' passed.\n", namedResourceCfg.name);

    /* Create the named resource to indicate that ARM is operational. */
    NameTest_createPushNamedResource (nameClientHandle, "ARM-Status", 0xBEEF);

    /*********************************************************************************
     * TEST: Create named resource and push from the ARM to the DSP realm.
     *********************************************************************************/
    for (index = 0; index < TEST_MAX_ITERATIONS; index++)
    {
        /* Populate the resource name to be created and pushed. */
        sprintf(resourceName, "ARM-Test-%d", index);

        /* Create and push the named resource to the peer realm. */
        NameTest_createPushNamedResource (nameClientHandle, resourceName, index);
    }

    /* Synchronize till the DSP cores have acknowledged that the modification test is done.*/
    NameTest_sync ("ModificationTest", 1);

    /*********************************************************************************
     * TEST: Test the modification of the named resource from the DSP to the ARM realm
     *********************************************************************************/
    printf ("Debug: Launching the named resource modification verification test\n");
    for (index = 0; index < TEST_MAX_ITERATIONS; index++)
    {
        /* Populate the resource name which has been modified. */
        sprintf(resourceName, "ARM-Test-%d", index);

        /* Ensure that the resource name has been modified */
        NameTest_sync (resourceName, index+5000);
    }

    /*********************************************************************************
     * TEST: Test the deletion of the named resource from the DSP to the ARM realm
     *********************************************************************************/
    printf ("Debug: Launching the named resource deletion test\n");
    for (index = 0; index < TEST_MAX_ITERATIONS; index++)
    {
        /* Populate the resource name which has been modified. */
        sprintf(resourceName, "ARM-Test-%d", index);

        /* Delete the named resource */
        NameTest_deletePushNamedResource (nameClientHandle, resourceName);
    }

    /* Indicate to the DSP that the ARM has successfully deleted all the named resources */
    NameTest_createPushNamedResource (nameClientHandle, "DeletionTest", 0xABCD1234);

    /* Test passed */
    printf ("Debug: All tests passed\n");

    /* Set the flag to indicate that the tests are complete */
    testComplete = 1;
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      Name Client Execution Task: The task provides an execution context for
 *      the NAME client
 *
 *  @retval
 *      Not Applicable.
 */
static void* NameClientTask(void* arg)
{
    Name_ClientHandle       nameClientHandle;

    /* Get the name client handle: */
    nameClientHandle = (Name_ClientHandle)arg;

    /* Loop around till the test completes */
    while (testComplete == 0)
    {
        /* Execute the name client */
        Name_executeClient (nameClientHandle);
        usleep(10);
    }
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
    Name_ClientCfg          clientCfg;
    pthread_t               testThread;
    pthread_t               nameClientThread;
    Name_ClientHandle       nameClientHandle;

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
    databaseCfg.instanceId  = 1;
    databaseCfg.realm       = Name_ExecutionRealm_ARM;
    strcpy (databaseCfg.owner, "LTE9A_L3");

    /* Create the global database */
    globalNameDatabaseHandle = Name_createDatabase (&databaseCfg, &errCode);
    if (globalNameDatabaseHandle == NULL)
	{
	    printf ("Error: Name database creation failed [Error code %d]\n", errCode);
	    return -1;
    }
    printf ("Debug: Name database created successfully [Handle %p]\n", globalNameDatabaseHandle);

    /* Initialize the name client configuration */
    memset ((void *)&clientCfg, 0, sizeof(Name_ClientCfg));

    /* Populate the client configuration: */
    strcpy (clientCfg.clientName, "UnitTestARM");
    strcpy (clientCfg.proxyName, "NameServer_LTE9A");
    clientCfg.databaseHandle            = globalNameDatabaseHandle;
    clientCfg.realm                     = Name_ExecutionRealm_ARM;
    clientCfg.malloc                    = Name_osalMalloc;
    clientCfg.free                      = Name_osalFree;
    clientCfg.beginMemAccess            = Name_osalBeginMemAccess;
    clientCfg.endMemAccess              = Name_osalEndMemAccess;
    clientCfg.enterCS                   = Name_osalEnterCS;
    clientCfg.exitCS                    = Name_osalExitCS;
    clientCfg.createSem                 = Name_osalCreateSem;
    clientCfg.deleteSem                 = Name_osalDeleteSem;
    clientCfg.postSem                   = Name_osalPostSem;
    clientCfg.pendSem                   = Name_osalPendSem;

    /* Create the name clients: Name clients can only be created once the PROXY is operational */
    while (1)
    {
        /* Create the name client */
        nameClientHandle = Name_initClient (&clientCfg, &errCode);
        if (nameClientHandle != NULL)
            break;

        if (errCode != NAME_ENOTREADY)
            printf ("Error: Name client creation failed [Error code %d]\n", errCode);
    }
    printf ("Debug: Name client %p has been created successfully\n", nameClientHandle);

    /* Create the name client task which provides an application context for the NAME Client */
    errCode = pthread_create (&nameClientThread, NULL, NameClientTask, nameClientHandle);
    if (errCode < 0)
    {
        printf ("Error: Unable to create the test thread %d\n", errCode);
        return -1;
    }

    /* Create the test task */
    errCode = pthread_create (&testThread, NULL, TestTask, nameClientHandle);
    if (errCode < 0)
    {
        printf ("Error: Unable to create the test thread %d\n", errCode);
        return -1;
    }

    /* Blocked till the tests are done. */
    pthread_join (testThread, NULL);
    pthread_join (nameClientThread, NULL);

    /* Shutdown the name client */
    if (Name_deleteClient (nameClientHandle, &errCode) < 0)
        printf ("Error: Name client deletion failed [Error code %d]\n", errCode);
    else
        printf ("Debug: Name client deleted successfully\n");

    /* Cleanup the resource manager configuration */
    if (Resmgr_deinit (handleSysCfg, &errCode) < 0)
        printf ("Error: Shutting down the system configuration failed\n");
    else
        printf ("Debug: Shutting down the system configuration passed\n");
    return 0;
}

