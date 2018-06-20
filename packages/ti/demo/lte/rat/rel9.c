/**
 *   @file  rel10-d2.c
 *
 *   @brief
 *      The file implements the Release 9 Deployment
 *
 *      The deployment is defined as follows:
 *      - L1A is executing on DSP Core1 and Core2
 *      - L2A is executing on DSP Core0 and Core3
 *      - L1B is executing on DSP Core5 and Core6
 *      - L2B is executing on DSP Core4 and Core7
 *
 *    The A and B domains are completely different and there is
 *    no sharing of resouces between the domains
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
 ****************** Release 9 Deployment Functions ********************
 **********************************************************************/

/**
 *  @b Description
 *  @n
 *      The function initialize the RAT Release 9 LTE cell
 *
 *  @param[in]  cellId
 *      Cell identifier [Only 'A' or 'B']
 *  @param[out]  nameProxyPid
 *      Name Proxy process identifier
 *  @param[out]  netfpServerPid
 *      NETFP Server process identifier
 *  @param[out]  datServerPid
 *      DAT Server process identifier
 *  @param[out]  netfpProxyPid
 *      NetFP Proxys process identifier
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t Rat_Rel9InitializeCell
(
    char    cellId,
    pid_t*  nameProxyPid,
    pid_t*  netfpServerPid,
    pid_t*  datServerPid,
    pid_t*  netfpProxyPid
)
{
    char*               argv[20];
    int32_t             errCode;
    uint32_t            appConfig;
    Root_SyslibConfig   rootSyslibConfig;
    uint32_t            baseDspCoreId;
    uint32_t            baseDomainId;
    char*               ptrNamedResourceInstanceId;
    char*               ptrLocalAgentServerFlowId;
    char*               ptrRemoteAgentServerFlowId;
    char                lteCellRMClientName[RM_NAME_MAX_CHARS];
    char                lteCellNameProxy[NAME_MAX_CHAR];
    char                lteCellNetfpServerName[NETFP_MAX_CHAR];
    char                lteCellDatServerName[DAT_MAX_CHAR];
    char                lteCellNetfpConfigFileName[PATH_MAX];
    char                lteCellDatConfigFileName[PATH_MAX];
    char                lteCellNameProxyBaseAddress[PATH_MAX];
    uint32_t            rmBaseAddress;
    uint32_t            namedResBaseAddress;
    uint32_t            nameProxyBaseAddress;
    uint32_t            nameHwSem;
    uint32_t            qmssAccHwSemaphore;
#ifdef USE_NETFP_PROXY
    char                cellName[8];
    char                netfpProxyPluginName[PATH_MAX];
    char*               ptrNetfpProxyPollingDelay;
    char                lteCellNetfpProxyClientName[NETFP_MAX_CHAR];
#endif

    /* Debug Message: */
    LOG_DEBUG ("Debug: Launching Release9 LTE Cell'%c'\n", cellId);

    /* QMSS Accumulator Hardware Semaphore Protection: */
    qmssAccHwSemaphore = 0;

    /* Determine the base DSP core identifer depending upon the cell deployment. */
    if (cellId == 'A')
    {
        /* LTE CellA executes on DSP core 0, 1, 2 and 3 */
        baseDspCoreId = 0;

        /* Cell A uses the local flow id 0 & remote flow id 1 for name proxy */
        ptrLocalAgentServerFlowId  = "0";
        ptrRemoteAgentServerFlowId = "1";

        /* Cell A is using named resource instance id 1 */
        ptrNamedResourceInstanceId = "1";

        /* Setup the base address for the various SYSLIB services. These values should be derived from the DSP memory map.
         * NOTE: RM Base address remains the same across all the domains */
        rmBaseAddress           = 0xA0010000;
        namedResBaseAddress     = 0xA0020000;
        nameProxyBaseAddress    = 0xA001F000;
        nameHwSem               = 1;

        /* Base domain identifier which identifies the cell */
        baseDomainId = LTE_REL9A_DOMAIN_ID;
    }
    else
    {
        /* LTE CellB executes on DSP core 4, 5, 6 and 7 */
        baseDspCoreId = 4;

        /* Cell B uses the local flow id 2 & remote flow id 3 for name proxy */
        ptrLocalAgentServerFlowId  = "2";
        ptrRemoteAgentServerFlowId = "3";

        /* Cell B is using named resource instance id 2 */
        ptrNamedResourceInstanceId = "2";

        /* Setup the base address for the various SYSLIB services. These values should be derived from the DSP memory map.
         * NOTE: RM Base address remains the same across all the domains */
        rmBaseAddress           = 0xA0010000;
        namedResBaseAddress     = 0xB0001000;
        nameProxyBaseAddress    = 0xB0000000;
        nameHwSem               = 2;

        /* Base domain identifier which identifies the cell */
        baseDomainId = LTE_REL9B_DOMAIN_ID;
    }

    /* Reset the application domain handles depending upon which cell is being initialized */
    appDomainHandle[baseDspCoreId]   = 0;
    appDomainHandle[baseDspCoreId+1] = 0;
    appDomainHandle[baseDspCoreId+2] = 0;
    appDomainHandle[baseDspCoreId+3] = 0;

    /* Construct the names used for the LTE cell */
    snprintf (lteCellRMClientName,          RM_NAME_MAX_CHARS,    "Rm_LTE9%c"         , cellId);
    snprintf (lteCellNameProxy,             NAME_MAX_CHAR,        "NameServer_LTE9%c",  cellId);
    snprintf (lteCellDatServerName,         DAT_MAX_CHAR,         "DatServer_LTE9%c",   cellId);
    snprintf (lteCellDatConfigFileName,     PATH_MAX,             "dat_LTE9%c.conf" ,   cellId);
    snprintf (lteCellNetfpServerName,       NETFP_MAX_CHAR,       "NetfpServer_LTE9%c", cellId);
    snprintf (lteCellNetfpConfigFileName,   PATH_MAX,             "netfp_LTE9%c.conf" , cellId);
    snprintf (lteCellNameProxyBaseAddress,  PATH_MAX,             "0x%x",               nameProxyBaseAddress);

    /* Debug: Display the LTE cell specific parameters */
    LOG_DEBUG ("Debug: DSP Core Id                  : %d\n",   baseDspCoreId);
    LOG_DEBUG ("Debug: Name Proxy Local Flow Id     : %s\n",   ptrLocalAgentServerFlowId);
    LOG_DEBUG ("Debug: Name Proxy Remote Flow Id    : %s\n",   ptrRemoteAgentServerFlowId);
    LOG_DEBUG ("Debug: Named Resource Instance Id   : %s\n",   ptrNamedResourceInstanceId);
    LOG_DEBUG ("Debug: Base Domain Id               : 0x%x\n", baseDomainId);
    LOG_DEBUG ("Debug: Name Proxy Base Address      : %s\n",   lteCellNameProxyBaseAddress);
    LOG_DEBUG ("Debug: RM Client Name               : %s\n",   lteCellRMClientName);
    LOG_DEBUG ("Debug: Name Proxy                   : %s\n",   lteCellNameProxy);
    LOG_DEBUG ("Debug: DAT Server Name              : %s\n",   lteCellDatServerName);
    LOG_DEBUG ("Debug: DAT Server Config            : %s\n",   lteCellDatConfigFileName);
    LOG_DEBUG ("Debug: NETFP Server Name            : %s\n",   lteCellNetfpServerName);
    LOG_DEBUG ("Debug: NETFP Server Config          : %s\n",   lteCellNetfpConfigFileName);

    /* Launching the SYSLIB NAME Proxy for the domain: */
    argv[0]  = "./name_proxy_k2h.out";
    argv[1]  = "-n";
    argv[2]  = &lteCellNameProxy[0];
    argv[3]  = "-c";
    argv[4]  = &lteCellRMClientName[0];
    argv[5]  = "-l";
    argv[6]  = ptrLocalAgentServerFlowId;
    argv[7]  = "-r";
    argv[8]  = ptrRemoteAgentServerFlowId;
    argv[9]  = "-i";
    argv[10] = ptrNamedResourceInstanceId;
    argv[11] = "-a";
    argv[12] = &lteCellNameProxyBaseAddress[0];
    argv[13] = NULL;
    *nameProxyPid = Rat_SpawnSyslibServer ("./name_proxy_k2h.out", argv);

    /* Launching the SYSLIB DAT server for the domain: */
    argv[0]  = "./dat_server_k2h.out";
    argv[1]  = "-n";
    argv[2]  = &lteCellDatServerName[0];
    argv[3]  = "-r";
    argv[4]  = lteCellRMClientName;
    argv[5]  = "-i";
    argv[6]  = ptrNamedResourceInstanceId;
    argv[7]  = "-c";
    argv[8] = &lteCellDatConfigFileName[0];
    argv[9] = "-v";
    argv[10] = NULL;
    *datServerPid = Rat_SpawnSyslibServer ("./dat_server_k2h.out", argv);

    /*******************************************************************************
     * DSP Core Provisioning: LTE L2 Stack
     *******************************************************************************/

    /* Initialize the application: On DSP Core0 populate the application domain configuration */
    appConfig = 0xdeaddead;

    /* Initialize the root SYSLIB configuration. */
    memset ((void *)&rootSyslibConfig, 0, sizeof(Root_SyslibConfig));

    /* Provision and configure the SYSLIB modules  */
    snprintf (rootSyslibConfig.name, ROOT_MAX_CHAR, "LTE9%c_L2", cellId);
    snprintf (rootSyslibConfig.rmClient, RM_NAME_MAX_CHARS, "Rm_LTE9%c_L2", cellId);
    strcpy (rootSyslibConfig.rmServer, "Rm_Server");
    strcpy (rootSyslibConfig.proxyName, lteCellNameProxy);
    rootSyslibConfig.coreId                                             = baseDspCoreId;
    rootSyslibConfig.nrInstanceId                                       = atoi(ptrNamedResourceInstanceId);
    rootSyslibConfig.qmssAccHwSemaphore                                 = qmssAccHwSemaphore;
    rootSyslibConfig.dspSyslibConfig.armCoreId                          = 8;
    rootSyslibConfig.dspSyslibConfig.sysRMIPCSourceId                   = 17;
    rootSyslibConfig.dspSyslibConfig.sharedMemAddress                   = rmBaseAddress;
    rootSyslibConfig.dspSyslibConfig.sizeSharedMemory                   = 8192;
    rootSyslibConfig.dspSyslibConfig.instantiateNameProxy               = 1;
    rootSyslibConfig.dspSyslibConfig.localFlowId                        = atoi(ptrRemoteAgentServerFlowId);
    rootSyslibConfig.dspSyslibConfig.remoteFlowId                       = atoi(ptrLocalAgentServerFlowId);
    rootSyslibConfig.dspSyslibConfig.namedResourceSharedMemAddress      = namedResBaseAddress;
    rootSyslibConfig.dspSyslibConfig.sizeNamedResourceSharedMemAddress  = 4*1024;
    rootSyslibConfig.dspSyslibConfig.nameHwSemaphore                    = nameHwSem;
    rootSyslibConfig.dspSyslibConfig.initNamedResourceDatabase          = 1;
    rootSyslibConfig.dspSyslibConfig.nameProxySharedMemAddress          = nameProxyBaseAddress;

    /* Instantiate the name client */
    rootSyslibConfig.nameClientConfig.instantiateNameClient             = 1;

    /* Instantiate the NETFP client */
    rootSyslibConfig.netfpClientConfig.instantiateNetfpClient           = 1;
    strcpy (rootSyslibConfig.netfpClientConfig.serverName, lteCellNetfpServerName);
    snprintf (rootSyslibConfig.netfpClientConfig.clientName, NETFP_MAX_CHAR, "Netfp_Client_LTE9%c_L2DP", cellId);

    /* Instantiate the DAT client */
    rootSyslibConfig.datClientConfig.instantiateDatClient               = 1;
    rootSyslibConfig.datClientConfig.datClientId                        = baseDspCoreId;
    strcpy (rootSyslibConfig.datClientConfig.serverName, lteCellDatServerName);
    snprintf (rootSyslibConfig.datClientConfig.clientName, DAT_MAX_CHAR, "Dat_Client_LTE9%c_L2DP", cellId);

    /* Initialize the L2 domain on the base DSP core. */
    if (Root_appInit (rootMasterHandle, ROOT_SLAVE_COREID(1, baseDspCoreId), baseDomainId, &appConfig,
                      sizeof(appConfig), &rootSyslibConfig, &errCode) < 0)
    {
        LOG_ERROR ("Error: Root application initialization failed [Error code %d]\n", errCode);
        return -1;
    }
    LOG_DEBUG ("Debug: DSP Core%d [Done]\n", baseDspCoreId);

    /*******************************************************************************
     * DSP Core Provisioning: PHY
     *******************************************************************************/

    /* Initialize the root SYSLIB configuration. */
    memset ((void *)&rootSyslibConfig, 0, sizeof(Root_SyslibConfig));

    /* Provision and configure the SYSLIB modules  */
    snprintf (rootSyslibConfig.name, ROOT_MAX_CHAR, "LTE9%c_L1", cellId);
    snprintf (rootSyslibConfig.rmClient, RM_NAME_MAX_CHARS, "Rm_LTE9%c_L1", cellId);
    strcpy (rootSyslibConfig.rmServer, "Rm_Server");
    strcpy (rootSyslibConfig.proxyName, lteCellNameProxy);
    rootSyslibConfig.coreId                                             = (baseDspCoreId + 1);
    rootSyslibConfig.nrInstanceId                                       = atoi(ptrNamedResourceInstanceId);
    rootSyslibConfig.qmssAccHwSemaphore                                 = qmssAccHwSemaphore;
    rootSyslibConfig.dspSyslibConfig.sharedMemAddress                   = rmBaseAddress;
    rootSyslibConfig.dspSyslibConfig.sizeSharedMemory                   = 8192;
    rootSyslibConfig.dspSyslibConfig.armCoreId                          = 8;
    rootSyslibConfig.dspSyslibConfig.sysRMIPCSourceId                   = 17;
    rootSyslibConfig.dspSyslibConfig.instantiateNameProxy               = 0;
    rootSyslibConfig.dspSyslibConfig.namedResourceSharedMemAddress      = namedResBaseAddress;
    rootSyslibConfig.dspSyslibConfig.sizeNamedResourceSharedMemAddress  = 4*1024;
    rootSyslibConfig.dspSyslibConfig.nameHwSemaphore                    = nameHwSem;
    rootSyslibConfig.dspSyslibConfig.initNamedResourceDatabase          = 0;
    rootSyslibConfig.dspSyslibConfig.nameProxySharedMemAddress          = nameProxyBaseAddress;

    /* Instantiate the name client. */
    rootSyslibConfig.nameClientConfig.instantiateNameClient             = 1;

    /* No need to instantiate the NETFP client */
    rootSyslibConfig.netfpClientConfig.instantiateNetfpClient           = 0;
    strcpy (rootSyslibConfig.netfpClientConfig.serverName, lteCellNetfpServerName);

    /* Instantiate the DAT client */
    rootSyslibConfig.datClientConfig.instantiateDatClient               = 1;
    rootSyslibConfig.datClientConfig.datClientId                        = (baseDspCoreId + 1);
    strcpy (rootSyslibConfig.datClientConfig.serverName, lteCellDatServerName);
    snprintf (rootSyslibConfig.datClientConfig.clientName, DAT_MAX_CHAR, "Dat_Client_LTE9%c_L1_SLAVE", cellId);

    /* Initialize the L1 PHY application */
    if (Root_appInit (rootMasterHandle, ROOT_SLAVE_COREID(1, (baseDspCoreId + 1)), (baseDomainId + 1), &appConfig,
                      sizeof(appConfig), &rootSyslibConfig, &errCode) < 0)
    {
        LOG_ERROR ("Error: Root application initialization failed [Error code %d]\n", errCode);
        return -1;
    }
    LOG_DEBUG ("Debug: DSP Core%d [Done]\n", (baseDspCoreId + 1));

    /*******************************************************************************
     * DSP Core2 Provisioning: PHY
     *******************************************************************************/

    /* Initialize the root SYSLIB configuration. */
    memset ((void *)&rootSyslibConfig, 0, sizeof(Root_SyslibConfig));

    /* Provision and configure the SYSLIB modules  */
    snprintf (rootSyslibConfig.name, ROOT_MAX_CHAR, "LTE9%c_L1", cellId);
    snprintf (rootSyslibConfig.rmClient, RM_NAME_MAX_CHARS, "Rm_LTE9%c_L1", cellId);
    strcpy (rootSyslibConfig.rmServer, "Rm_Server");
    strcpy (rootSyslibConfig.proxyName, lteCellNameProxy);
    rootSyslibConfig.coreId                                             = (baseDspCoreId + 2);
    rootSyslibConfig.nrInstanceId                                       = atoi(ptrNamedResourceInstanceId);
    rootSyslibConfig.qmssAccHwSemaphore                                 = qmssAccHwSemaphore;
    rootSyslibConfig.dspSyslibConfig.sharedMemAddress                   = rmBaseAddress;
    rootSyslibConfig.dspSyslibConfig.sizeSharedMemory                   = 8192;
    rootSyslibConfig.dspSyslibConfig.armCoreId                          = 8;
    rootSyslibConfig.dspSyslibConfig.sysRMIPCSourceId                   = 17;
    rootSyslibConfig.dspSyslibConfig.instantiateNameProxy               = 0;
    rootSyslibConfig.dspSyslibConfig.namedResourceSharedMemAddress      = namedResBaseAddress;
    rootSyslibConfig.dspSyslibConfig.sizeNamedResourceSharedMemAddress  = 4*1024;
    rootSyslibConfig.dspSyslibConfig.nameHwSemaphore                    = nameHwSem;
    rootSyslibConfig.dspSyslibConfig.initNamedResourceDatabase          = 0;
    rootSyslibConfig.dspSyslibConfig.nameProxySharedMemAddress          = nameProxyBaseAddress;

    /* Instantiate the name client. */
    rootSyslibConfig.nameClientConfig.instantiateNameClient             = 1;

    /* Instantiate the NETFP client */
    rootSyslibConfig.netfpClientConfig.instantiateNetfpClient           = 1;
    strcpy (rootSyslibConfig.netfpClientConfig.serverName, lteCellNetfpServerName);
    snprintf (rootSyslibConfig.netfpClientConfig.clientName, NETFP_MAX_CHAR, "Netfp_Client_LTE9%c_L1", cellId);

    /* Instantiate the DAT client */
    rootSyslibConfig.datClientConfig.instantiateDatClient               = 1;
    rootSyslibConfig.datClientConfig.datClientId                        = (baseDspCoreId + 2);
    strcpy (rootSyslibConfig.datClientConfig.serverName, lteCellDatServerName);
    snprintf (rootSyslibConfig.datClientConfig.clientName, DAT_MAX_CHAR, "Dat_Client_LTE9%c_L1_MASTER", cellId);

    /* Initialize the L1 PHY application */
    if (Root_appInit (rootMasterHandle, ROOT_SLAVE_COREID(1, (baseDspCoreId + 2)), (baseDomainId + 2), &appConfig,
                      sizeof(appConfig), &rootSyslibConfig, &errCode) < 0)
    {
        LOG_ERROR ("Error: Root application initialization failed [Error code %d]\n", errCode);
        return -1;
    }
    LOG_DEBUG ("Debug: DSP Core%d [Done]\n", (baseDspCoreId + 2));

    /*******************************************************************************
     * DSP Core3 Provisioning: LTE L2
     *******************************************************************************/

    /* Initialize the root SYSLIB configuration. */
    memset ((void *)&rootSyslibConfig, 0, sizeof(Root_SyslibConfig));

    /* Provision and configure the SYSLIB modules  */
    snprintf (rootSyslibConfig.name, ROOT_MAX_CHAR, "LTE9%c_L2", cellId);
    snprintf (rootSyslibConfig.rmClient, RM_NAME_MAX_CHARS, "Rm_LTE9%c_L2", cellId);
    strcpy (rootSyslibConfig.rmServer, "Rm_Server");
    strcpy (rootSyslibConfig.proxyName, lteCellNameProxy);
    rootSyslibConfig.coreId                                             = (baseDspCoreId + 3);
    rootSyslibConfig.nrInstanceId                                       = atoi(ptrNamedResourceInstanceId);
    rootSyslibConfig.qmssAccHwSemaphore                                 = qmssAccHwSemaphore;
    rootSyslibConfig.dspSyslibConfig.sharedMemAddress                   = rmBaseAddress;
    rootSyslibConfig.dspSyslibConfig.sizeSharedMemory                   = 8192;
    rootSyslibConfig.dspSyslibConfig.armCoreId                          = 8;
    rootSyslibConfig.dspSyslibConfig.sysRMIPCSourceId                   = 17;
    rootSyslibConfig.dspSyslibConfig.instantiateNameProxy               = 0;
    rootSyslibConfig.dspSyslibConfig.namedResourceSharedMemAddress      = namedResBaseAddress;
    rootSyslibConfig.dspSyslibConfig.sizeNamedResourceSharedMemAddress  = 4*1024;
    rootSyslibConfig.dspSyslibConfig.nameHwSemaphore                    = nameHwSem;
    rootSyslibConfig.dspSyslibConfig.initNamedResourceDatabase          = 0;
    rootSyslibConfig.dspSyslibConfig.nameProxySharedMemAddress          = nameProxyBaseAddress;

    /* Instantiate the name client. */
    rootSyslibConfig.nameClientConfig.instantiateNameClient             = 1;

    /* Instantiate the NETFP client */
    rootSyslibConfig.netfpClientConfig.instantiateNetfpClient           = 1;
    strcpy (rootSyslibConfig.netfpClientConfig.serverName, lteCellNetfpServerName);
    snprintf (rootSyslibConfig.netfpClientConfig.clientName, NETFP_MAX_CHAR, "Netfp_Client_LTE9%c_L2SCHED", cellId);

    /* Instantiate the DAT client */
    rootSyslibConfig.datClientConfig.instantiateDatClient               = 1;
    rootSyslibConfig.datClientConfig.datClientId                        = (baseDspCoreId + 3);
    strcpy (rootSyslibConfig.datClientConfig.serverName, lteCellDatServerName);
    snprintf (rootSyslibConfig.datClientConfig.clientName, DAT_MAX_CHAR, "Dat_Client_LTE9%c_L2SCHED", cellId);

    /* Initialize the L2 application */
    if (Root_appInit (rootMasterHandle, ROOT_SLAVE_COREID(1, (baseDspCoreId + 3)), (baseDomainId + 3), &appConfig,
                      sizeof(appConfig), &rootSyslibConfig, &errCode) < 0)
    {
        LOG_ERROR ("Error: Root application initialization failed [Error code %d]\n", errCode);
        return -1;
    }
    LOG_DEBUG ("Debug: DSP Core%d [Done]\n", (baseDspCoreId + 3));

    /* Delay the startup of the NETFP server on the ARM realm so that the agent services on the DSP are operational. */
    sleep(10);

    /* Launching the SYSLIB NETFP server for the domain. */
    LOG_DEBUG ("Debug: Launching the NETFP server for the domain.\n");
    argv[0] = "./netfp_server_k2h.out";
    argv[1] = "-n";
    argv[2] = &lteCellNetfpServerName[0];
    argv[3] = "-r";
    argv[4] = &lteCellRMClientName[0];
    argv[5] = "-c";
    argv[6] = &lteCellNetfpConfigFileName[0];
    argv[7] = "-i";
    argv[8] = ptrNamedResourceInstanceId;
    argv[9] = NULL;
    *netfpServerPid = Rat_SpawnSyslibServer ("./netfp_server_k2h.out", argv);

#ifdef USE_NETFP_PROXY
    /* Setup the NETFP Proxy startup configuration */
    snprintf (netfpProxyPluginName,          PATH_MAX,    "./netfpproxy_plugin_k2h.so");
    ptrNetfpProxyPollingDelay   =   "1";
    snprintf (lteCellNetfpProxyClientName,   NETFP_MAX_CHAR,    "Netfp_Client_LTE9%c_NET_PROXY",    cellId);

    /* Launching the NETFP Proxy for the domain. */
    LOG_DEBUG ("Debug: Launching the NETFP Proxy for the domain.\n");
    argv[0]  = "./netfpproxy_k2h.out";
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
    *netfpProxyPid = Rat_SpawnSyslibServer ("./netfpproxy_k2h.out", argv);

    /* Offload required policies for domain to be operational */
    snprintf (cellName, 8, "9%c", cellId);
    Rat_DoPolicyOffloads (cellName, atoi(ptrNamedResourceInstanceId), *netfpProxyPid);
#endif

    /* Loop around till all the domains in the RAT are operational. */
    printf ("Debug: Waiting for all the domains to be operational\n");
    while (1)
    {
        /* Release 9 RAT cell is across 4 domains. */
        if ((appDomainHandle[baseDspCoreId] != NULL)   && (appDomainHandle[baseDspCoreId+1] != NULL) &&
            (appDomainHandle[baseDspCoreId+2] != NULL) && (appDomainHandle[baseDspCoreId+3] != NULL))
            break;

        /* Wait & sleep for some time */
        sleep(1);
    }
    LOG_INFO ("Debug: Release9 RAT cell is operational\n");
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to deinitialize the RAT Release 9 LTE cell
 *
 *  @param[in]  cellId
 *      Cell identifier [Only 'A' or 'B']
 *  @param[in]  nameProxyPid
 *      Name Proxy process identifier
 *  @param[in]  netfpServerPid
 *      NETFP Server process identifier
 *  @param[in]  datServerPid
 *      DAT Server process identifier
 *  @param[in]  netfpProxyPid
 *      NETFP Proxy process identifier
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t Rat_Rel9DeinitializeCell
(
    char    cellId,
    pid_t   nameProxyPid,
    pid_t   netfpServerPid,
    pid_t   datServerPid,
    pid_t   netfpProxyPid
)
{
    int32_t     errCode;
    uint32_t    baseDspCoreId;

    /* Debug Message: */
    LOG_DEBUG ("Debug: Shutting down Release9 LTE Cell'%c'\n", cellId);

    /* Determine the base DSP core identifer depending upon the cell deployment. */
    if (cellId == 'A')
    {
        /* LTE CellA executes on DSP core 0, 1, 2 and 3 */
        baseDspCoreId = 0;
    }
    else
    {
        /* LTE CellB executes on DSP core 4, 5, 6 and 7 */
        baseDspCoreId = 4;
    }

    /* Deinitialize the L2 application */
    if (Root_appDeinit (rootMasterHandle, ROOT_SLAVE_COREID(1, (baseDspCoreId+3)), appDomainHandle[baseDspCoreId+3], &errCode) < 0)
    {
        LOG_ERROR ("Error: Root application deinitialization on Core 3 failed [Error code %d]\n", errCode);
        return -1;
    }
    LOG_DEBUG ("Debug: DSP Core%d deinitialization initiated\n", (baseDspCoreId+3));

    /* Deinitialize the L1 PHY application */
    if (Root_appDeinit (rootMasterHandle, ROOT_SLAVE_COREID(1, (baseDspCoreId+2)), appDomainHandle[baseDspCoreId+2], &errCode) < 0)
    {
        LOG_ERROR ("Error: Root application deinitialization on Core 3 failed [Error code %d]\n", errCode);
        return -1;
    }
    LOG_DEBUG ("Debug: DSP Core%d deinitialization initiated\n", (baseDspCoreId+2));

    /* Deinitialize the L1 PHY application */
    if (Root_appDeinit (rootMasterHandle, ROOT_SLAVE_COREID(1, (baseDspCoreId+1)), appDomainHandle[baseDspCoreId+1], &errCode) < 0)
    {
        LOG_ERROR ("Error: Root application deinitialization on Core 1 failed [Error code %d]\n", errCode);
        return -1;
    }
    LOG_DEBUG ("Debug: DSP Core%d deinitialization initiated\n", (baseDspCoreId+1));

    /* Make sure all the other domain are down! This is required before we shutdown the agent server. */
    while (1)
    {
        if ((appDomainHandle[baseDspCoreId+1] == NULL) && (appDomainHandle[baseDspCoreId+2] == NULL) &&
            (appDomainHandle[baseDspCoreId+3] == NULL))
            break;

        /* Wait & sleep for some time */
        sleep(1);
    }

    /* Deinitialize the root slave and domain on Core X
     * NOTE: The DSP Core which executes the name proxy needs to be shutdown at the very end
     * since name services are required by the other modules to deinitialize themselves. */
    if (Root_appDeinit (rootMasterHandle, ROOT_SLAVE_COREID(1, baseDspCoreId), appDomainHandle[baseDspCoreId], &errCode) < 0)
    {
        LOG_ERROR ("Error: Root application deinitialization on Core 0 failed [Error code %d]\n", errCode);
        return -1;
    }
    LOG_DEBUG ("Debug: DSP Core%d deinitialization initiated\n", baseDspCoreId);

#ifdef USE_NETFP_PROXY
    kill (netfpProxyPid, SIGTERM);
    LOG_DEBUG ("Debug: NETFP Proxy is down!\n");
#endif

    /* NETFP Server can now be deleted: All the NETFP clients have been deinitialized. */
    kill (netfpServerPid, SIGTERM);
    LOG_DEBUG ("Debug: NETFP Server is down!\n");

    /* Shutdown the DAT Server: All the DAT clients have been deinitialized */
    kill (datServerPid, SIGTERM);
    LOG_DEBUG ("Debug: DAT Server is down!\n");

    /* Wait for some time to allow the NETFP Server deletion to complete. */
    sleep(10);

    /* Sanity Check: Ensure that the agent server domain has been shutdown! */
    while (1)
    {
        if (appDomainHandle[baseDspCoreId] == NULL)
            break;

        /* Wait & sleep for some time */
        sleep(1);
    }
    LOG_INFO ("Debug: RAT domains on the DSP have been shutdown successfully\n");

    /* Shutdown the name proxy */
    kill (nameProxyPid, SIGTERM);
    LOG_INFO ("Debug: RAT has been shutdown!\n");
    return 0;
}


/**
 *  @b Description
 *  @n
 *      The function handles the Rel9 CLI
 *
 *  @retval
 *      Not Applicable.
 */
void Rat_Rel9CLI (void)
{
    int32_t     testSelection;
    uint32_t    rat_CellAStatus = 0;
    uint32_t    rat_CellBStatus = 0;
    pid_t       rel9A_NameServerPid;
    pid_t       rel9A_NetfpServerPid;
    pid_t       rel9A_DatServerPid;
    pid_t       rel9B_NameServerPid;
    pid_t       rel9B_NetfpServerPid;
    pid_t       rel9B_DatServerPid;
    pid_t       rel9A_NetfpProxyPid;
    pid_t       rel9B_NetfpProxyPid;

    /* Release 9 Dual Cell */
    while (1)
    {
        LOG_DEBUG ("**************************************\n");
        LOG_DEBUG ("1. Launch   CellA\n");
        LOG_DEBUG ("2. Shutdown CellA\n");
        LOG_DEBUG ("3. Launch   CellB\n");
        LOG_DEBUG ("4. Shutdown CellB\n");
        LOG_DEBUG ("5. Cell Status\n");
        LOG_DEBUG ("**************************************\n");

        /* Get the user input. */
        LOG_DEBUG ("Enter the selection:");
        scanf("%d", &testSelection);

        switch (testSelection)
        {
            case 1:
            {
                /* Initialize the RAT Cell A only if it is not operational */
                if (rat_CellAStatus == 0)
                {
                    Rat_Rel9InitializeCell('A', &rel9A_NameServerPid, &rel9A_NetfpServerPid, &rel9A_DatServerPid, &rel9A_NetfpProxyPid);
                    rat_CellAStatus = 1;
                }
                else
                {
                    LOG_ERROR ("Error: RAT CellA is already operational\n");
                }
                break;
            }
            case 2:
            {
                /* Shutdown the RAT Cell A only if it is operational. */
                if (rat_CellAStatus == 1)
                {
                    Rat_Rel9DeinitializeCell('A', rel9A_NameServerPid, rel9A_NetfpServerPid, rel9A_DatServerPid, rel9A_NetfpProxyPid);
                    rat_CellAStatus = 0;
                }
                else
                {
                    LOG_ERROR ("Error: RAT CellA is NOT operational\n");
                }
                break;
            }
            case 3:
            {
                /* Initialize the RAT Cell B only if it is not operational */
                if (rat_CellBStatus == 0)
                {
                    Rat_Rel9InitializeCell('B', &rel9B_NameServerPid, &rel9B_NetfpServerPid, &rel9B_DatServerPid, &rel9B_NetfpProxyPid);
                    rat_CellBStatus = 1;
                }
                else
                {
                    LOG_ERROR ("Error: RAT CellB is already operational\n");
                }
                break;
            }
            case 4:
            {
                /* Shutdown the RAT Cell B only if it is operational. */
                if (rat_CellBStatus == 1)
                {
                    Rat_Rel9DeinitializeCell('B', rel9B_NameServerPid, rel9B_NetfpServerPid, rel9B_DatServerPid, rel9B_NetfpProxyPid);
                    rat_CellBStatus = 0;
                }
                else
                {
                    LOG_ERROR ("Error: RAT CellB is NOT operational\n");
                }
                break;
            }
            case 5:
            {
                if (rat_CellAStatus == 1)
                    LOG_INFO ("RAT CellA is up!\n");
                else
                    LOG_ERROR ("RAT CellA is down!\n");

                if (rat_CellBStatus == 1)
                    LOG_INFO ("RAT CellB is up!\n");
                else
                    LOG_ERROR ("RAT CellB is down!\n");
                break;
            }
            default:
            {
                LOG_ERROR ("Error: Invalid selection\n");
                break;
            }
        }
    }
}

