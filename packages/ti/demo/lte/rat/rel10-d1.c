/**
 *   @file  rel10-d1.c
 *
 *   @brief
 *      The file implements the Release 10 Deployment 1.
 *
 *      The deployment is defined as follows:
 *      - L1A is executing on DSP Core0 and Core1
 *      - L1B is executing on DSP Core2 and Core3
 *      - L2 is executing in the ARM
 *
 *      The NETFP & DAT Server execute on ARM. The L1A and L1B execute
 *      the name proxy to cross into the ARM realm. There is no need
 *      to execute the name proxy for L2 since it is already executing in
 *      the ARM.
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
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <pthread.h>
#include <malloc.h>
#include <semaphore.h>
#include <signal.h>
#include <unistd.h>
#include <getopt.h>
#include <limits.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <errno.h>

/* MCSDK include files. */
#include <ti/runtime/root/root.h>
#include <ti/csl/cslr_device.h>

/* SYSLIB include files */
#include <ti/runtime/name/name.h>
#include <ti/runtime/root/root.h>
#include <ti/runtime/netfp/netfp.h>
#include <ti/runtime/dat/dat.h>
#include <ti/apps/netfp_proxy/netfp_proxy_ipc.h>

/* RAT Master file */
#include "master.h"

/**********************************************************************
 *********************** Extern Declarations **************************
 **********************************************************************/

/* Root Master Handle: */
extern Root_MasterHandle   rootMasterHandle;

/* Application Domain Handles for the DSP Core(s) & ARM processes */
extern void*   appDomainHandle[16];

/**********************************************************************
 ******************** Rel 10 Deployment1 Functions ********************
 **********************************************************************/

/**
 *  @b Description
 *  @n
 *      The function initialize the RAT Release 10 D1 LTE cell
 *
 *  @param[in]  l1a
 *      PHY identifier [Set to 'A' will enable L1A on Core0, Core1]
 *  @param[in]  l1b
 *      PHY identifier [Set to 'B' will enable L1B on Core2, Core3]
 *  @param[out]  nameDBHandle
 *      RAT Name Database handle
 *  @param[out]  nameProxyPidL1A
 *      Name Proxy process identifier associated with the L1A PHY
 *  @param[out]  nameProxyPidL1B
 *      Name Proxy process identifier associated with the L1B PHY
 *  @param[out]  netfpServerPid
 *      NETFP Server process identifier associated with the Rel10 domain
 *  @param[out]  datServerPid
 *      DAT Server process identifier associated with the Rel10 domain
 *  @param[out]  netfpProxyPid
 *      NetFP Proxy process identifier associated with Rel10 domain
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t Rat_Rel10D1InitializeCell
(
    char            l1a,
    char            l1b,
    Name_DBHandle*  nameDBHandle,
    pid_t*          nameProxyPidL1A,
    pid_t*          nameProxyPidL1B,
    pid_t*          netfpServerPid,
    pid_t*          datServerPid,
    pid_t*          netfpProxyPid
)
{
    char*               argv[20];
    int32_t             errCode;
    uint32_t            appConfig;
    Name_DatabaseCfg    dbConfig;
    Root_SyslibConfig   rootSyslibConfig;
    char*               ptrNamedResourceInstanceId;
    char                lteCellRMClientName[RM_NAME_MAX_CHARS];
    uint32_t            rmBaseAddress;
    char                lteCellNameProxyL1A[NAME_MAX_CHAR];
    char                lteCellNameProxyL1ABaseAddress[PATH_MAX];
    char                lteCellNameProxyL1B[NAME_MAX_CHAR];
    char                lteCellNameProxyL1BBaseAddress[PATH_MAX];
    uint32_t            namedResL1ABaseAddress;
    uint32_t            nameProxyL1ABaseAddress;
    char*               ptrLocalL1AFlowId;
    char*               ptrRemoteL1AFlowId;
    uint32_t            namedResL1BBaseAddress;
    uint32_t            nameProxyL1BBaseAddress;
    char*               ptrLocalL1BFlowId;
    char*               ptrRemoteL1BFlowId;
    char                lteCellNetfpConfigFileName[PATH_MAX];
    char                lteCellNetfpServerName[NETFP_MAX_CHAR];
    char                lteCellDatConfigFileName[PATH_MAX];
    char                lteCellDatServerName[DAT_MAX_CHAR];
    char                netfpProxyPluginName[PATH_MAX];
    char*               ptrNetfpProxyPollingDelay;
    char                lteCellNetfpProxyClientName[NETFP_MAX_CHAR];
    uint32_t            namedResourceL1AHwSem;
    uint32_t            namedResourceL1BHwSem;
    uint32_t            qmssAccHwSemaphore;
#if defined(DEVICE_K2H)
    char*               gNetfpServerExecutable = "./netfp_server_k2h.out";
    char*               gDatServerExecutable = "./dat_server_k2h.out";
    char*               gNameProxyExecutable = "./name_proxy_k2h.out";
    char*               gNetfpProxy = "./netfpproxy_k2h.out";
    char*               gNetfpPlugin = "./netfpproxy_plugin_k2h.so";
#elif defined (DEVICE_K2L)
    char*               gNetfpServerExecutable = "./netfp_server_k2l.out";
    char*               gDatServerExecutable = "./dat_server_k2l.out";
    char*               gNameProxyExecutable = "./name_proxy_k2l.out";
    char*               gNetfpProxy = "./netfpproxy_k2l.out";
    char*               gNetfpPlugin = "./netfpproxy_plugin_k2l.so";
#else
#error "Error: Unsupported Device"
#endif

    /* Debug Message: */
    LOG_DEBUG ("Debug: Launching Release10 Deployment 1 LTE Cell\n");

    /* Setup the base address for the various SYSLIB services. These values should be derived from
     * the DSP memory map. NOTE: RM Base address remains the same across all the domains */
    rmBaseAddress = 0xA0010000;

    /* The RM Client name which is passed to the configuration for all the SYSLIB servers. */
    snprintf (lteCellRMClientName, RM_NAME_MAX_CHARS,    "Rm_LTE10");

    /* There is a single named resource database identifier which throughout the entire RAT cell. */
    ptrNamedResourceInstanceId = "1";

    /* Create the RAT named database which executes on the ARM. */
    memset ((void *)&dbConfig, 0, sizeof (Name_DatabaseCfg));
    dbConfig.instanceId = atoi(ptrNamedResourceInstanceId);
    dbConfig.realm      = Name_ExecutionRealm_ARM;
    strncpy (dbConfig.owner, "OAM_LTE_Rel10D2", NAME_MAX_CHAR);
    *nameDBHandle = Name_createDatabase (&dbConfig, &errCode);
    if (*nameDBHandle == NULL)
    {
        LOG_ERROR ("Error: RAT Name database creation failed [Error code %d]\n", errCode);
        return -1;
    }

    /* QMSS Accumulator Hardware Semaphore Protection: */
    qmssAccHwSemaphore = 0;

    /* There are 2 Name Servers: One for the L1A and one for the L1B.
     * The L1A Name Server is connected from the DSP realm to the ARM realm via Name Proxy#1
     * Hardware Semaphore 1 is used for protecting the database on the DSP. */
    namedResL1ABaseAddress  = 0xA0020000;
    nameProxyL1ABaseAddress = 0xA001F000;
    namedResourceL1AHwSem   = 1;
    ptrLocalL1AFlowId       = "0";
    ptrRemoteL1AFlowId      = "1";
    snprintf (lteCellNameProxyL1ABaseAddress, PATH_MAX, "0x%x", nameProxyL1ABaseAddress);
    snprintf (lteCellNameProxyL1A, NAME_MAX_CHAR, "NameProxy_LTE10_L1A");

    /* The L1B Name Server is connected from the DSP realm to the ARM realm via Name Proxy#2
     * Hardware Semaphore 2 is used for protecting the database on the DSP. */
    namedResL1BBaseAddress  = 0xB0001000;
    nameProxyL1BBaseAddress = 0xB0000000;
    namedResourceL1BHwSem   = 2;
    ptrLocalL1BFlowId       = "2";
    ptrRemoteL1BFlowId      = "3";
    snprintf (lteCellNameProxyL1BBaseAddress, PATH_MAX, "0x%x", nameProxyL1BBaseAddress);
    snprintf (lteCellNameProxyL1B, NAME_MAX_CHAR, "NameProxy_LTE10_L1B");

    /* There are only 1 DAT & NETFP Server which are across the entire domain. */
    snprintf (lteCellDatServerName,       DAT_MAX_CHAR,   "DatServer_LTE10");
    snprintf (lteCellDatConfigFileName,   PATH_MAX,       "dat_LTE10.conf");
    snprintf (lteCellNetfpServerName,     NETFP_MAX_CHAR, "NetfpServer_LTE10");
    snprintf (lteCellNetfpConfigFileName, PATH_MAX,       "netfp_LTE10.conf");

    /******************************************************************************************
     * Debug: Display the LTE cell specific parameters
     ******************************************************************************************/
    LOG_DEBUG ("Debug: RM Client Name                    : %s\n",   lteCellRMClientName);
    LOG_DEBUG ("Debug: Named Resource Instance Id        : %s\n",   ptrNamedResourceInstanceId);
    LOG_DEBUG ("Debug: Name Proxy[L1A] Local Flow Id     : %s\n",   ptrLocalL1AFlowId);
    LOG_DEBUG ("Debug: Name Proxy[L1A] Remote Flow Id    : %s\n",   ptrRemoteL1AFlowId);
    LOG_DEBUG ("Debug: Name Proxy[L1A] Base Address      : %s\n",   lteCellNameProxyL1ABaseAddress);
    LOG_DEBUG ("Debug: Name Proxy[L1A]                   : %s\n",   lteCellNameProxyL1A);
    LOG_DEBUG ("Debug: Name Proxy[L1B] Local Flow Id     : %s\n",   ptrLocalL1BFlowId);
    LOG_DEBUG ("Debug: Name Proxy[L1B] Remote Flow Id    : %s\n",   ptrRemoteL1BFlowId);
    LOG_DEBUG ("Debug: Name Proxy[L1B] Base Address      : %s\n",   lteCellNameProxyL1BBaseAddress);
    LOG_DEBUG ("Debug: Name Proxy[L1B]                   : %s\n",   lteCellNameProxyL1B);
    LOG_DEBUG ("Debug: DAT Server Name                   : %s\n",   lteCellDatServerName);
    LOG_DEBUG ("Debug: DAT Server Config                 : %s\n",   lteCellDatConfigFileName);
    LOG_DEBUG ("Debug: NETFP Server Name                 : %s\n",   lteCellNetfpServerName);
    LOG_DEBUG ("Debug: NETFP Server Config               : %s\n",   lteCellNetfpConfigFileName);

    /*********************************************************************************
     * Launch the SYSLIB Servers depending upon the configuration:
     *********************************************************************************/
    if (l1a == 'A')
    {
        /*******************************************************************************
         * Launching the SYSLIB NAME Proxy #1 for L1A:
         *******************************************************************************/
        argv[0]  = gNameProxyExecutable;
        argv[1]  = "-n";
        argv[2]  = &lteCellNameProxyL1A[0];
        argv[3]  = "-c";
        argv[4]  = &lteCellRMClientName[0];
        argv[5]  = "-l";
        argv[6]  = ptrLocalL1AFlowId;
        argv[7]  = "-r";
        argv[8]  = ptrRemoteL1AFlowId;
        argv[9]  = "-i";
        argv[10] = ptrNamedResourceInstanceId;
        argv[11] = "-a";
        argv[12] = &lteCellNameProxyL1ABaseAddress[0];
        argv[13] = NULL;
        *nameProxyPidL1A = Rat_SpawnSyslibServer (gNameProxyExecutable, argv);
    }

    if (l1b == 'B')
    {
        /*******************************************************************************
         * Launching the SYSLIB NAME Proxy #1 for the L1B:
         *******************************************************************************/
        argv[0]  = gNameProxyExecutable;
        argv[1]  = "-n";
        argv[2]  = &lteCellNameProxyL1B[0];
        argv[3]  = "-c";
        argv[4]  = &lteCellRMClientName[0];
        argv[5]  = "-l";
        argv[6]  = ptrLocalL1BFlowId;
        argv[7]  = "-r";
        argv[8]  = ptrRemoteL1BFlowId;
        argv[9]  = "-i";
        argv[10] = ptrNamedResourceInstanceId;
        argv[11] = "-a";
        argv[12] = &lteCellNameProxyL1BBaseAddress[0];
        argv[13] = NULL;
        *nameProxyPidL1B = Rat_SpawnSyslibServer (gNameProxyExecutable, argv);
    }

    /*******************************************************************************
     * Launching the SYSLIB NETFP Server for L2:
     *******************************************************************************/
    argv[0]  = gNetfpServerExecutable;
    argv[1]  = "-n";
    argv[2]  = &lteCellNetfpServerName[0];
    argv[3]  = "-r";
    argv[4]  = &lteCellRMClientName[0];
    argv[5]  = "-c";
    argv[6]  = &lteCellNetfpConfigFileName[0];
    argv[7]  = "-i";
    argv[8]  = ptrNamedResourceInstanceId;
    argv[9]  = NULL;
    *netfpServerPid = Rat_SpawnSyslibServer (gNetfpServerExecutable, argv);

    /*******************************************************************************
     * Launching the SYSLIB DAT Server for L1B:
     *******************************************************************************/
    argv[0]  = gDatServerExecutable;
    argv[1]  = "-n";
    argv[2]  = &lteCellDatServerName[0];
    argv[3]  = "-r";
    argv[4]  = lteCellRMClientName;
    argv[5]  = "-i";
    argv[6]  = ptrNamedResourceInstanceId;
    argv[7]  = "-c";
    argv[8]  = &lteCellDatConfigFileName[0];
    argv[9]  = NULL;
    *datServerPid = Rat_SpawnSyslibServer (gDatServerExecutable, argv);

    /*******************************************************************************
     * Launching the SYSLIB NetFP Proxy for L2:
     *******************************************************************************/
    snprintf (netfpProxyPluginName, PATH_MAX, gNetfpPlugin);
    ptrNetfpProxyPollingDelay   =   "1";
    strncpy (lteCellNetfpProxyClientName, "Netfp_LTE10_PROXY", NETFP_MAX_CHAR);

    /* Launching the NETFP Proxy for the domain. */
    argv[0]  = gNetfpProxy;
    argv[1]  = "-p";
    argv[2]  = &netfpProxyPluginName[0];
    argv[3]  = "-d";
    argv[4]  = ptrNetfpProxyPollingDelay;
    argv[5]  = "-s";
    argv[6]  = &lteCellNetfpServerName[0];
    argv[7]  = "-r";
    argv[8]  = &lteCellRMClientName[0];
    argv[9]  = "-c";
    argv[10] = &lteCellNetfpProxyClientName[0];
    argv[11] = "-i";
    argv[12] = ptrNamedResourceInstanceId;
    argv[13] = NULL;
    *netfpProxyPid = Rat_SpawnSyslibServer (gNetfpProxy, argv);

    /*******************************************************************************
     * ARM Process Provisioning: LTE L2 Stack
     *******************************************************************************/

    /* Initialize the application: Populate the application domain configuration */
    appConfig = 0;
    if (l1a == 'A')
        appConfig = appConfig | (1 << 0);
    if (l1b == 'B')
        appConfig = appConfig | (1 << 1);

    /* Initialize the root SYSLIB configuration. */
    memset ((void *)&rootSyslibConfig, 0, sizeof(Root_SyslibConfig));

    /* Provision and configure the SYSLIB modules: */
    snprintf (rootSyslibConfig.name, ROOT_MAX_CHAR, "LTE10_L2");
    snprintf (rootSyslibConfig.rmClient, RM_NAME_MAX_CHARS, "Rm_LTE10_L2");
    strcpy (rootSyslibConfig.rmServer, "Rm_Server");
    strcpy (rootSyslibConfig.proxyName, lteCellNameProxyL1B);
#if defined (DEVICE_K2H)
    rootSyslibConfig.coreId                                             = 0x8;
#elif defined (DEVICE_K2L)
    rootSyslibConfig.coreId                                             = 0x4;
#else
#error "Error: Unsupported Device"
#endif
    rootSyslibConfig.nrInstanceId                                       = atoi(ptrNamedResourceInstanceId);
    rootSyslibConfig.qmssAccHwSemaphore                                 = qmssAccHwSemaphore;

    /* Instantiate the name client on ARM process */
    rootSyslibConfig.nameClientConfig.instantiateNameClient             = 1;

    /* Instantiate the NETFP client */
    rootSyslibConfig.netfpClientConfig.instantiateNetfpClient           = 1;
    strcpy (rootSyslibConfig.netfpClientConfig.serverName, lteCellNetfpServerName);
    snprintf (rootSyslibConfig.netfpClientConfig.clientName, NETFP_MAX_CHAR, "Netfp_LTE10_L2");

    /* Instantiate the DAT client */
    rootSyslibConfig.datClientConfig.instantiateDatClient               = 1;
    rootSyslibConfig.datClientConfig.datClientId                        = 0x8;
    strcpy (rootSyslibConfig.datClientConfig.serverName, lteCellDatServerName);
    snprintf (rootSyslibConfig.datClientConfig.clientName, DAT_MAX_CHAR, "Dat_LTE10_L2");

    /* Instantiate the MEMLOG instance */
    rootSyslibConfig.memlogConfig.instantiateMemlog                     = 1;

#if defined(DEVICE_K2H)
    /* Initialize the L2 domain on the ARM process */
    if (Root_appInit (rootMasterHandle, ROOT_SLAVE_COREID(0, 8), LTE_REL10D1_L2_DOMAIN_ID, &appConfig,
                      sizeof(appConfig), &rootSyslibConfig, &errCode) < 0)
    {
        LOG_ERROR ("Error: Root application initialization failed [Error code %d]\n", errCode);
        return -1;
    }
    LOG_DEBUG ("Debug: L2 ARM process initialization [Done]\n");
#elif defined (DEVICE_K2L)
    /* Initialize the L2 domain on the ARM process */
    if (Root_appInit (rootMasterHandle, ROOT_SLAVE_COREID(0, 4), LTE_REL10D1_L2_DOMAIN_ID, &appConfig,
                      sizeof(appConfig), &rootSyslibConfig, &errCode) < 0)
    {
        LOG_ERROR ("Error: Root application initialization failed [Error code %d]\n", errCode);
        return -1;
    }
    LOG_DEBUG ("Debug: L2 ARM process initialization [Done]\n");
#else
#error "Error: Unsupported Device"
#endif

    /*******************************************************************************
     * DSP Core Provisioning: L1-A
     *******************************************************************************/
    if (l1a == 'A')
    {
        /*******************************************************************************
         * DSP Core Provisioning: Core0 [PHY SLAVE]
         *******************************************************************************/

        /* Initialize the application: Populate the application domain configuration */
        appConfig = (1 << 0);

        /* Initialize the root SYSLIB configuration. */
        memset ((void *)&rootSyslibConfig, 0, sizeof(Root_SyslibConfig));

        /* Provision and configure the SYSLIB modules: */
        snprintf (rootSyslibConfig.name, ROOT_MAX_CHAR, "LTE10_L1A");
        snprintf (rootSyslibConfig.rmClient, RM_NAME_MAX_CHARS, "Rm_LTE10_L1A");
        strcpy (rootSyslibConfig.rmServer, "Rm_Server");
        strcpy (rootSyslibConfig.proxyName, lteCellNameProxyL1A);
        rootSyslibConfig.coreId                                             = 0;
        rootSyslibConfig.nrInstanceId                                       = atoi(ptrNamedResourceInstanceId);
        rootSyslibConfig.qmssAccHwSemaphore                                 = qmssAccHwSemaphore;
        rootSyslibConfig.dspSyslibConfig.sharedMemAddress                   = rmBaseAddress;
        rootSyslibConfig.dspSyslibConfig.sizeSharedMemory                   = 8192;
        rootSyslibConfig.dspSyslibConfig.armCoreId                          = SYSLIB_ARM_CORE_ID;
        rootSyslibConfig.dspSyslibConfig.sysRMIPCSourceId                   = 17;
        rootSyslibConfig.dspSyslibConfig.instantiateNameProxy               = 1;
        rootSyslibConfig.dspSyslibConfig.localFlowId                        = atoi(ptrRemoteL1AFlowId);
        rootSyslibConfig.dspSyslibConfig.remoteFlowId                       = atoi(ptrLocalL1AFlowId);
        rootSyslibConfig.dspSyslibConfig.namedResourceSharedMemAddress      = namedResL1ABaseAddress;
        rootSyslibConfig.dspSyslibConfig.sizeNamedResourceSharedMemAddress  = 4*1024;
        rootSyslibConfig.dspSyslibConfig.nameHwSemaphore                    = namedResourceL1AHwSem;
        rootSyslibConfig.dspSyslibConfig.initNamedResourceDatabase          = 1;
        rootSyslibConfig.dspSyslibConfig.nameProxySharedMemAddress          = nameProxyL1ABaseAddress;

        /* Instantiate the name client. */
        rootSyslibConfig.nameClientConfig.instantiateNameClient             = 1;

        /* Instantiate the NETFP client */
        rootSyslibConfig.netfpClientConfig.instantiateNetfpClient           = 1;
        strcpy (rootSyslibConfig.netfpClientConfig.serverName, lteCellNetfpServerName);
        snprintf (rootSyslibConfig.netfpClientConfig.clientName, NETFP_MAX_CHAR, "Netfp_LTE10_L1A_0");

        /* Instantiate the MEMLOG  */
        rootSyslibConfig.memlogConfig.instantiateMemlog                     = 1;

        /* Instantiate the DAT client */
        rootSyslibConfig.datClientConfig.instantiateDatClient               = 1;
        rootSyslibConfig.datClientConfig.datClientId                        = 0;
        strcpy (rootSyslibConfig.datClientConfig.serverName, lteCellDatServerName);
        snprintf (rootSyslibConfig.datClientConfig.clientName, DAT_MAX_CHAR, "Dat_LTE10_L1A_0");

        /* Initialize the L1 PHY application */
        if (Root_appInit (rootMasterHandle, ROOT_SLAVE_COREID(1, 0), LTE_REL10D1_L1A_DOMAIN_ID, &appConfig,
                          sizeof(appConfig), &rootSyslibConfig, &errCode) < 0)
        {
            LOG_ERROR ("Error: Root application initialization failed [Error code %d]\n", errCode);
            return -1;
        }
        LOG_DEBUG ("Debug: DSP Core0 [Done]\n");

        /*******************************************************************************
         * DSP Core Provisioning: Core1 [PHY MASTER]
         *******************************************************************************/

        /* Initialize the root SYSLIB configuration. */
        memset ((void *)&rootSyslibConfig, 0, sizeof(Root_SyslibConfig));

        /* Provision and configure the SYSLIB modules  */
        snprintf (rootSyslibConfig.name, ROOT_MAX_CHAR, "LTE10_L1A");
        snprintf (rootSyslibConfig.rmClient, RM_NAME_MAX_CHARS, "Rm_LTE10_L1A");
        strcpy (rootSyslibConfig.rmServer, "Rm_Server");
        strcpy (rootSyslibConfig.proxyName, lteCellNameProxyL1A);
        rootSyslibConfig.coreId                                             = 1;
        rootSyslibConfig.nrInstanceId                                       = atoi(ptrNamedResourceInstanceId);
        rootSyslibConfig.qmssAccHwSemaphore                                 = qmssAccHwSemaphore;
        rootSyslibConfig.dspSyslibConfig.sharedMemAddress                   = rmBaseAddress;
        rootSyslibConfig.dspSyslibConfig.sizeSharedMemory                   = 8192;
        rootSyslibConfig.dspSyslibConfig.armCoreId                          = SYSLIB_ARM_CORE_ID;
        rootSyslibConfig.dspSyslibConfig.sysRMIPCSourceId                   = 17;
        rootSyslibConfig.dspSyslibConfig.instantiateNameProxy               = 0;
        rootSyslibConfig.dspSyslibConfig.namedResourceSharedMemAddress      = namedResL1ABaseAddress;
        rootSyslibConfig.dspSyslibConfig.sizeNamedResourceSharedMemAddress  = 4*1024;
        rootSyslibConfig.dspSyslibConfig.nameHwSemaphore                    = namedResourceL1AHwSem;
        rootSyslibConfig.dspSyslibConfig.initNamedResourceDatabase          = 0;
        rootSyslibConfig.dspSyslibConfig.nameProxySharedMemAddress          = nameProxyL1ABaseAddress;

        /* Instantiate the name client. */
        rootSyslibConfig.nameClientConfig.instantiateNameClient             = 1;

        /* No need to instantiate the NETFP client */
        rootSyslibConfig.netfpClientConfig.instantiateNetfpClient           = 0;
        strcpy (rootSyslibConfig.netfpClientConfig.serverName, lteCellNetfpServerName);
        snprintf (rootSyslibConfig.netfpClientConfig.clientName, NETFP_MAX_CHAR, "Netfp_LTE10_L1A_1");

        /* Instantiate the MEMLOG */
        rootSyslibConfig.memlogConfig.instantiateMemlog                     = 1;

        /* Instantiate the DAT client */
        rootSyslibConfig.datClientConfig.instantiateDatClient               = 1;
        rootSyslibConfig.datClientConfig.datClientId                        = 1;
        strcpy (rootSyslibConfig.datClientConfig.serverName, lteCellDatServerName);
        snprintf (rootSyslibConfig.datClientConfig.clientName, DAT_MAX_CHAR, "Dat_LTE10_L1A_1");

        /* Initialize the L1 PHY application */
        if (Root_appInit (rootMasterHandle, ROOT_SLAVE_COREID(1, 1), (LTE_REL10D1_L1A_DOMAIN_ID + 1), &appConfig,
                          sizeof(appConfig), &rootSyslibConfig, &errCode) < 0)
        {
            LOG_ERROR ("Error: Root application initialization failed [Error code %d]\n", errCode);
            return -1;
        }
        LOG_DEBUG ("Debug: DSP Core1 [Done]\n");
    }

    /*******************************************************************************
     * DSP Core Provisioning: L1-B
     *******************************************************************************/
    if (l1b == 'B')
    {
        /*******************************************************************************
         * DSP Core Provisioning: Core2 [PHY SLAVE]
         *******************************************************************************/

        /* Initialize the application: Populate the application domain configuration */
        appConfig = (1 << 1);

        /* Initialize the root SYSLIB configuration. */
        memset ((void *)&rootSyslibConfig, 0, sizeof(Root_SyslibConfig));

        /* Provision and configure the SYSLIB modules  */
        snprintf (rootSyslibConfig.name, ROOT_MAX_CHAR, "LTE10_L1B");
        snprintf (rootSyslibConfig.rmClient, RM_NAME_MAX_CHARS, "Rm_LTE10_L1B");
        strcpy (rootSyslibConfig.rmServer, "Rm_Server");
        strcpy (rootSyslibConfig.proxyName, lteCellNameProxyL1B);
        rootSyslibConfig.coreId                                             = 2;
        rootSyslibConfig.nrInstanceId                                       = atoi(ptrNamedResourceInstanceId);
        rootSyslibConfig.qmssAccHwSemaphore                                 = qmssAccHwSemaphore;
        rootSyslibConfig.dspSyslibConfig.sharedMemAddress                   = rmBaseAddress;
        rootSyslibConfig.dspSyslibConfig.sizeSharedMemory                   = 8192;
        rootSyslibConfig.dspSyslibConfig.armCoreId                          = SYSLIB_ARM_CORE_ID;
        rootSyslibConfig.dspSyslibConfig.sysRMIPCSourceId                   = 17;
        rootSyslibConfig.dspSyslibConfig.instantiateNameProxy               = 1;
        rootSyslibConfig.dspSyslibConfig.localFlowId                        = atoi(ptrRemoteL1BFlowId);
        rootSyslibConfig.dspSyslibConfig.remoteFlowId                       = atoi(ptrLocalL1BFlowId);
        rootSyslibConfig.dspSyslibConfig.namedResourceSharedMemAddress      = namedResL1BBaseAddress;
        rootSyslibConfig.dspSyslibConfig.sizeNamedResourceSharedMemAddress  = 4*1024;
        rootSyslibConfig.dspSyslibConfig.nameHwSemaphore                    = namedResourceL1BHwSem;
        rootSyslibConfig.dspSyslibConfig.initNamedResourceDatabase          = 1;
        rootSyslibConfig.dspSyslibConfig.nameProxySharedMemAddress          = nameProxyL1BBaseAddress;

        /* Instantiate the name client. */
        rootSyslibConfig.nameClientConfig.instantiateNameClient             = 1;

        /* Instantiate the NETFP client */
        rootSyslibConfig.netfpClientConfig.instantiateNetfpClient           = 1;
        strcpy (rootSyslibConfig.netfpClientConfig.serverName, lteCellNetfpServerName);
        snprintf (rootSyslibConfig.netfpClientConfig.clientName, NETFP_MAX_CHAR, "Netfp_LTE10_L1B_0");

        /* Instantiate the MEMLOG */
        rootSyslibConfig.memlogConfig.instantiateMemlog                     = 1;

        /* Instantiate the DAT client */
        rootSyslibConfig.datClientConfig.instantiateDatClient               = 1;
        rootSyslibConfig.datClientConfig.datClientId                        = 2;
        strcpy (rootSyslibConfig.datClientConfig.serverName, lteCellDatServerName);
        snprintf (rootSyslibConfig.datClientConfig.clientName, DAT_MAX_CHAR, "Dat_LTE10_L1B_0");

        /* Initialize the L1 PHY application */
        if (Root_appInit (rootMasterHandle, ROOT_SLAVE_COREID(1, 2), LTE_REL10D1_L1B_DOMAIN_ID, &appConfig,
                          sizeof(appConfig), &rootSyslibConfig, &errCode) < 0)
        {
            LOG_ERROR ("Error: Root application initialization failed [Error code %d]\n", errCode);
            return -1;
        }
        LOG_DEBUG ("Debug: DSP Core2 [Done]\n");

        /*******************************************************************************
         * DSP Core Provisioning: Core3 [PHY MASTER]
         *******************************************************************************/

        /* Initialize the root SYSLIB configuration. */
        memset ((void *)&rootSyslibConfig, 0, sizeof(Root_SyslibConfig));

        /* Provision and configure the SYSLIB modules  */
        snprintf (rootSyslibConfig.name, ROOT_MAX_CHAR, "LTE10_L1B");
        snprintf (rootSyslibConfig.rmClient, RM_NAME_MAX_CHARS, "Rm_LTE10_L1B");
        strcpy (rootSyslibConfig.rmServer, "Rm_Server");
        strcpy (rootSyslibConfig.proxyName, lteCellNameProxyL1B);
        rootSyslibConfig.coreId                                             = 3;
        rootSyslibConfig.nrInstanceId                                       = atoi(ptrNamedResourceInstanceId);
        rootSyslibConfig.qmssAccHwSemaphore                                 = qmssAccHwSemaphore;
        rootSyslibConfig.dspSyslibConfig.sharedMemAddress                   = rmBaseAddress;
        rootSyslibConfig.dspSyslibConfig.sizeSharedMemory                   = 8192;
        rootSyslibConfig.dspSyslibConfig.armCoreId                          = SYSLIB_ARM_CORE_ID;
        rootSyslibConfig.dspSyslibConfig.sysRMIPCSourceId                   = 17;
        rootSyslibConfig.dspSyslibConfig.instantiateNameProxy               = 0;
        rootSyslibConfig.dspSyslibConfig.namedResourceSharedMemAddress      = namedResL1BBaseAddress;
        rootSyslibConfig.dspSyslibConfig.sizeNamedResourceSharedMemAddress  = 4*1024;
        rootSyslibConfig.dspSyslibConfig.nameHwSemaphore                    = namedResourceL1BHwSem;
        rootSyslibConfig.dspSyslibConfig.initNamedResourceDatabase          = 0;
        rootSyslibConfig.dspSyslibConfig.nameProxySharedMemAddress          = nameProxyL1BBaseAddress;

        /* Instantiate the name client. */
        rootSyslibConfig.nameClientConfig.instantiateNameClient             = 1;

        /* No need to instantiate the NETFP client */
        rootSyslibConfig.netfpClientConfig.instantiateNetfpClient           = 0;
        strcpy (rootSyslibConfig.netfpClientConfig.serverName, lteCellNetfpServerName);
        snprintf (rootSyslibConfig.netfpClientConfig.clientName, NETFP_MAX_CHAR, "Netfp_LTE10_L1B_1");

        /* Instantiate the MEMLOG */
        rootSyslibConfig.memlogConfig.instantiateMemlog                     = 1;

        /* Instantiate the DAT client */
        rootSyslibConfig.datClientConfig.instantiateDatClient               = 1;
        rootSyslibConfig.datClientConfig.datClientId                        = 3;
        strcpy (rootSyslibConfig.datClientConfig.serverName, lteCellDatServerName);
        snprintf (rootSyslibConfig.datClientConfig.clientName, DAT_MAX_CHAR, "Dat_LTE10_L1B_1");

        /* Initialize the L1 PHY application */
        if (Root_appInit (rootMasterHandle, ROOT_SLAVE_COREID(1, 3), (LTE_REL10D1_L1B_DOMAIN_ID + 1), &appConfig,
                          sizeof(appConfig), &rootSyslibConfig, &errCode) < 0)
        {
            LOG_ERROR ("Error: Root application initialization failed [Error code %d]\n", errCode);
            return -1;
        }
        LOG_DEBUG ("Debug: DSP Core3 [Done]\n");
    }

    /* Offload required policies for the domain to be operational */
#if 1
    Rat_DoPolicyOffloads (*nameDBHandle, *netfpProxyPid);
#else
    Rat_DoPolicyOffloads_FromFile (*nameDBHandle, "10", *netfpProxyPid);
#endif

    /* Loop around till the RAT is operational */
    printf ("Debug: Waiting for all the domains to be operational\n");
    while (1)
    {
        /* Only L1A deployment. */
        if ((l1a == 'A') && (l1b == 0))
        {
            if ((appDomainHandle[0] != NULL) && (appDomainHandle[1] != NULL) && (appDomainHandle[2] != NULL))
                break;
        }
        else if ((l1a == 0) && (l1b == 'B'))
        {
            if ((appDomainHandle[0] != NULL) && (appDomainHandle[3] != NULL) && (appDomainHandle[4] != NULL))
                break;
        }
        else if ((l1a == 'A') && (l1b == 'B'))
        {
            if ((appDomainHandle[0] != NULL) && (appDomainHandle[1] != NULL) && (appDomainHandle[2] != NULL) &&
                (appDomainHandle[3] != NULL) && (appDomainHandle[4] != NULL))
                break;
        }
        else
        {
            printf ("Error: Invalid configuration\n");
            return -1;
        }
        /* Wait & sleep for some time */
        sleep(1);
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function deinitializes the RAT Release 10 LTE cell
 *
 *  @param[in]  l1a
 *      PHY identifier [Set to 'A' will enable L1A on Core0, Core1]
 *  @param[in]  l1b
 *      PHY identifier [Set to 'B' will enable L1B on Core2, Core3]
 *  @param[in]  nameDBHandle
 *      RAT Cell name database handle
 *  @param[in]  nameProxyPidL1A
 *      Name Proxy process identifier associated with the L1A PHY
 *  @param[in]  nameProxyPidL1B
 *      Name Proxy process identifier associated with the L1B PHY
 *  @param[in]  netfpServerPid
 *      NETFP Server process identifier associated with the Rel10 domain
 *  @param[in]  datServerPid
 *      DAT Server process identifier associated with the Rel10 domain
 *  @param[out]  netfpProxyPid
 *      NetFP Proxy process identifier associated with Rel10 domain
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t Rat_Rel10D1DeinitializeCell
(
    char            l1a,
    char            l1b,
    Name_DBHandle   nameDBHandle,
    pid_t           nameProxyPidL1A,
    pid_t           nameProxyPidL1B,
    pid_t           netfpServerPid,
    pid_t           datServerPid,
    pid_t           netfpProxyPid
)
{
    int32_t errCode;

    /**********************************************************************************
     * Shutdown the Release 10-D2 cell
     *
     * We start by shutting down all the DSP cores for L1A and L1B which dont execute
     * the NAME Proxy. Once they have been shutdown successfully; we then shutdown
     * the NAME Proxy cores. This is because the Name Proxy is required by the other
     * SYSLIB services to shut themselves down.
     **********************************************************************************/

    /* Deinitialize the L1A PHY application */
    if (Root_appDeinit (rootMasterHandle, ROOT_SLAVE_COREID(1, 1), appDomainHandle[2], &errCode) < 0)
    {
        LOG_ERROR ("Error: Root application deinitialization on Core 1 failed [Error code %d]\n", errCode);
        return -1;
    }
    LOG_DEBUG ("Debug: DSP Core1 deinitialization initiated\n");

    /* Deinitialize the L1B PHY application */
    if (Root_appDeinit (rootMasterHandle, ROOT_SLAVE_COREID(1, 3), appDomainHandle[4], &errCode) < 0)
    {
        LOG_ERROR ("Error: Root application deinitialization on Core 1 failed [Error code %d]\n", errCode);
        return -1;
    }
    LOG_DEBUG ("Debug: DSP Core3 deinitialization initiated\n");

    /* Deinitialize the L2 application */
    if (Root_appDeinit (rootMasterHandle, ROOT_SLAVE_COREID(0, 8), appDomainHandle[0], &errCode) < 0)
    {
        LOG_ERROR ("Error: Root application deinitialization for the ARM process failed [Error code %d]\n", errCode);
        return -1;
    }
    LOG_DEBUG ("Debug: L2 ARM Process deinitialization initiated\n");

    /* Make sure all the domain are down */
    while (1)
    {
        if ((appDomainHandle[0] == NULL) && (appDomainHandle[2] == NULL) && (appDomainHandle[4] == NULL))
            break;

        /* Wait & sleep for some time */
        sleep(1);
    }

    /**********************************************************************************
     * Control comes here implies that we can now shutdown the remaining DSP cores.
     **********************************************************************************/

    /* Deinitialize the L1A PHY application */
    if (Root_appDeinit (rootMasterHandle, ROOT_SLAVE_COREID(1, 0), appDomainHandle[1], &errCode) < 0)
    {
        LOG_ERROR ("Error: Root application deinitialization on Core 0 failed [Error code %d]\n", errCode);
        return -1;
    }
    LOG_DEBUG ("Debug: DSP Core0 deinitialization initiated\n");

    /* Deinitialize the L1B PHY application */
    if (Root_appDeinit (rootMasterHandle, ROOT_SLAVE_COREID(1, 2), appDomainHandle[3], &errCode) < 0)
    {
        LOG_ERROR ("Error: Root application deinitialization on Core 0 failed [Error code %d]\n", errCode);
        return -1;
    }
    LOG_DEBUG ("Debug: DSP Core2 deinitialization initiated\n");

    /* Make sure all the domain are down */
    while (1)
    {
        if ((appDomainHandle[1] == NULL) && (appDomainHandle[3] == NULL))
            break;

        /* Wait & sleep for some time */
        sleep(1);
    }

    /* Stop NETFP Proxy*/
    kill (netfpProxyPid, SIGTERM);
    LOG_DEBUG ("Debug: NETFP Proxy is down!\n");

    /* NETFP Server can now be deleted: All the NETFP clients have been deinitialized. */
    kill (netfpServerPid, SIGTERM);
    LOG_DEBUG ("Debug: NETFP Server is down!\n");

    /* Shutdown the DAT Server: All the DAT clients have been deinitialized */
    kill (datServerPid, SIGTERM);
    LOG_DEBUG ("Debug: DAT Server is down!\n");

    LOG_INFO ("Debug: RAT domains on the DSP have been shutdown successfully\n");

    /* Purge the security policies which have been offloaded: */
    if (Name_deleteResource (nameDBHandle, Name_ResourceBucket_USER_DEF1,
                             "Egress_SPID_IPv4_UP",  &errCode) < 0)
        LOG_ERROR ("Debug: Egress security policy name deletion failed [Error code %d]\n", errCode);
    if (Name_deleteResource (nameDBHandle, Name_ResourceBucket_USER_DEF1,
                             "Ingress_SPID_IPv4_UP",  &errCode) < 0)
        LOG_ERROR ("Debug: Ingress security policy name deletion failed [Error code %d]\n", errCode);

    /* Delete the RAT name database */
    Name_deleteDatabase (nameDBHandle, &errCode);

    /* Shutdown the name proxies */
    if (l1a == 'A')
        kill (nameProxyPidL1A, SIGTERM);
    if (l1b == 'B')
        kill (nameProxyPidL1B, SIGTERM);
    LOG_INFO ("Debug: RAT has been shutdown!\n");
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function handles the Rel10 Deployment1 CLI
 *
 *  @retval
 *      Not Applicable.
 */
void Rat_Rel10D1_CLI (void)
{
    int32_t         testSelection;
    uint32_t        ratCellStatus = 0;
    pid_t           nameProxyL1APid;
    pid_t           nameProxyL1BPid;
    pid_t           netfpServerPid;
    pid_t           datServerPid;
    pid_t           netfpProxyPid;
    Name_DBHandle   nameDBHandle;

    /* Release 10: */
    while (1)
    {
        LOG_DEBUG ("******************************************\n");
        LOG_DEBUG ("1. Launch   Release 10 D1 Cell with L1A, L1B\n");
        LOG_DEBUG ("2. Shutdown Release 10 D1 Cell\n");
        LOG_DEBUG ("3. Release 10 Status\n");
        LOG_DEBUG ("**************************************\n");

        /* Get the user input. */
        LOG_DEBUG ("Enter the selection:");
        scanf("%d", &testSelection);

        /* Process the Release 10 configuration: */
        switch (testSelection)
        {
            case 1:
            {
                /* Release 10: With both L1A and L1B */
                Rat_Rel10D1InitializeCell('A', 'B', &nameDBHandle, &nameProxyL1APid, &nameProxyL1BPid, &netfpServerPid,
                                          &datServerPid, &netfpProxyPid);

                /* Update the RAT Cell Status (Both A & B) are operational. */
                ratCellStatus = (1 << 0) | (1 << 1);
                break;
            }
            case 111:
            {
                /* Release 10: With only L1A [Hidden option for debugging] */
                Rat_Rel10D1InitializeCell('A', 0, &nameDBHandle, &nameProxyL1APid, &nameProxyL1BPid, &netfpServerPid,
                                          &datServerPid, &netfpProxyPid);

                /* Update the RAT Cell Status (Only A) is operational. */
                ratCellStatus = (1 << 0);
                break;
            }
            case 112:
            {
                /* Release 10: With only L1B [Hidden option for debugging] */
                Rat_Rel10D1InitializeCell(0, 'B', &nameDBHandle, &nameProxyL1APid, &nameProxyL1BPid, &netfpServerPid,
                                          &datServerPid, &netfpProxyPid);

                /* Update the RAT Cell Status (Only B) is operational. */
                ratCellStatus = (1 << 1);
                break;
            }
            case 2:
            {
                /* Release 10: Shutdown the cell. */
                if (ratCellStatus == ((1 << 0) | (1 << 1)))
                    Rat_Rel10D1DeinitializeCell ('A', 'B', nameDBHandle, nameProxyL1APid, nameProxyL1BPid, netfpServerPid,
                                                 datServerPid, netfpProxyPid);
                else if (ratCellStatus == (1 << 0))
                    Rat_Rel10D1DeinitializeCell ('A', 0, nameDBHandle, nameProxyL1APid, nameProxyL1BPid, netfpServerPid,
                                                 datServerPid, netfpProxyPid);
                else
                    Rat_Rel10D1DeinitializeCell (0, 'B', nameDBHandle, nameProxyL1APid, nameProxyL1BPid, netfpServerPid,
                                                 datServerPid, netfpProxyPid);

                /* The release10 cell is down! */
                ratCellStatus = 0;
                break;
            }
            case 3:
            {
                if (ratCellStatus == 0)
                    LOG_ERROR ("RAT Cell is down!\n");
                else if (ratCellStatus == (1 << 0))
                    LOG_INFO ("RAT CellA is up!\n");
                else if (ratCellStatus == (1 << 1))
                    LOG_INFO ("RAT CellB is up!\n");
                else if (ratCellStatus == ((1 << 1) | (1 << 0)))
                    LOG_INFO ("RAT CellA and CellB is up!\n");
                break;
            }
            default:
            {
                break;
            }
        }
    }
}
