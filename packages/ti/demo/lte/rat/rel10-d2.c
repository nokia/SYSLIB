/**
 *   @file  rel10-d2.c
 *
 *   @brief
 *      The file implements the Release 10 Deployment 2.
 *
 *      The deployment is defined as follows:
 *      - L1A is executing on DSP Core1 and Core2
 *      - L1B is executing on DSP Core5 and Core6
 *      - L2A is executing on DSP Core0 and Core3
 *      - L2B is executing on DSP Core4 and Core7
 *
 *      The NETFP & DAT Server execute on ARM. Each entity (L1A, L1B, L2A and L2B)
 *      communicate with the ARM using its own NAME Proxy.
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
 ******************** Rel 10 Deployment2 Functions ********************
 **********************************************************************/

/**
 *  @b Description
 *  @n
 *      The function initialize the RAT Release 10 D2 LTE cell
 *
 *  @param[out]  nameDBHandle
 *      RAT Name Database handle
 *  @param[out]  nameProxyPidL1A
 *      Name Proxy process identifier associated with the L1A PHY
 *  @param[out]  nameProxyPidL1B
 *      Name Proxy process identifier associated with the L1B PHY
 *  @param[out]  nameProxyPidL2A
 *      Name Proxy process identifier associated with the L2A
 *  @param[out]  nameProxyPidL2B
 *      Name Proxy process identifier associated with the L2B
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
static int32_t Rat_Rel10D2InitializeCell
(
    Name_DBHandle*  nameDBHandle,
    pid_t*          nameProxyPidL1A,
    pid_t*          nameProxyPidL1B,
    pid_t*          nameProxyPidL2A,
    pid_t*          nameProxyPidL2B,
    pid_t*          netfpServerPid,
    pid_t*          datServerPid,
    pid_t*          netfpProxyPid
)
{
    char*               argv[20];
    int32_t             errCode;
    uint32_t            appConfig = 0;
    Root_SyslibConfig   rootSyslibConfig;
    char*               ptrNamedResourceInstanceId;
    char                lteCellRMClientName[RM_NAME_MAX_CHARS];
    uint32_t            rmBaseAddress;
    char                lteCellNameProxyL2A[NAME_MAX_CHAR];
    char                lteCellNameProxyL2ABaseAddress[PATH_MAX];
    uint32_t            namedResL2ABaseAddress;
    uint32_t            namedResourceL2AHwSem;
    uint32_t            nameProxyL2ABaseAddress;
    char*               ptrLocalL2AFlowId;
    char*               ptrRemoteL2AFlowId;
    char                lteCellNameProxyL2B[NAME_MAX_CHAR];
    char                lteCellNameProxyL2BBaseAddress[PATH_MAX];
    uint32_t            namedResL2BBaseAddress;
    uint32_t            namedResourceL2BHwSem;
    uint32_t            namedResourceL1BHwSem;
    uint32_t            nameProxyL2BBaseAddress;
    char*               ptrLocalL2BFlowId;
    char*               ptrRemoteL2BFlowId;
    char                lteCellNameProxyL1A[NAME_MAX_CHAR];
    char                lteCellNameProxyL1ABaseAddress[PATH_MAX];
    char                lteCellNameProxyL1B[NAME_MAX_CHAR];
    char                lteCellNameProxyL1BBaseAddress[PATH_MAX];
    uint32_t            namedResL1ABaseAddress;
    uint32_t            namedResourceL1AHwSem;
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
    Name_DatabaseCfg    dbConfig;
    uint32_t            qmssAccHwSemaphore;

    /* Debug Message: */
    LOG_DEBUG ("Debug: Launching Release10 Deployment 2 LTE Cell\n");

    /* Setup the base address for the various SYSLIB services. These values should be derived from
     * the DSP memory map. NOTE: RM Base address remains the same across all the domains */
    rmBaseAddress = 0xA0010000;

    /* The RM Client name which is passed to the configuration for all the SYSLIB servers. */
    snprintf (lteCellRMClientName, RM_NAME_MAX_CHARS,  "Rm_LTE10");

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

    /* There are 4 Name Servers: L1A, L1B, L2A and L2B
     * The L1A Name Server is connected from the DSP realm to the ARM realm via Name Proxy#1 */
    namedResL1ABaseAddress  = 0xA0020000;
    nameProxyL1ABaseAddress = 0xA001F000;
    namedResourceL1AHwSem   = 1;
    ptrLocalL1AFlowId       = "0";
    ptrRemoteL1AFlowId      = "1";
    snprintf (lteCellNameProxyL1ABaseAddress, PATH_MAX, "0x%x", nameProxyL1ABaseAddress);
    snprintf (lteCellNameProxyL1A, NAME_MAX_CHAR, "NameProxy_LTE10_L1A");

    /* The L1B Name Server is connected from the DSP realm to the ARM realm via Name Proxy#2 */
    namedResL1BBaseAddress  = 0xB0001000;
    nameProxyL1BBaseAddress = 0xB0000000;
    namedResourceL1BHwSem   = 2;
    ptrLocalL1BFlowId       = "2";
    ptrRemoteL1BFlowId      = "3";
    snprintf (lteCellNameProxyL1BBaseAddress, PATH_MAX, "0x%x", nameProxyL1BBaseAddress);
    snprintf (lteCellNameProxyL1B, NAME_MAX_CHAR, "NameProxy_LTE10_L1B");

    /* The L2A Name Server is connected from the DSP realm to the ARM realm via Name Proxy#3 */
    namedResL2ABaseAddress  = 0xB5003000;
    nameProxyL2ABaseAddress = 0xB5002000;
    namedResourceL2AHwSem   = 3;
    ptrLocalL2AFlowId       = "4";
    ptrRemoteL2AFlowId      = "5";
    snprintf (lteCellNameProxyL2ABaseAddress, PATH_MAX, "0x%x", nameProxyL2ABaseAddress);
    snprintf (lteCellNameProxyL2A, NAME_MAX_CHAR, "NameProxy_LTE10_L2A");

    /* The L2B Name Server is connected from the DSP realm to the ARM realm via Name Proxy#4 */
    namedResL2BBaseAddress  = 0xB5005000;
    nameProxyL2BBaseAddress = 0xB5004000;
    namedResourceL2BHwSem   = 4;
    ptrLocalL2BFlowId       = "6";
    ptrRemoteL2BFlowId      = "7";
    snprintf (lteCellNameProxyL2BBaseAddress, PATH_MAX, "0x%x", nameProxyL2BBaseAddress);
    snprintf (lteCellNameProxyL2B, NAME_MAX_CHAR, "NameProxy_LTE10_L2B");

    /* Single DAT & NETFP Server executes in the system */
    snprintf (lteCellDatServerName,       DAT_MAX_CHAR,   "DatServer_LTE10");
    snprintf (lteCellDatConfigFileName,   PATH_MAX,       "dat_LTE10_D2.conf");
    snprintf (lteCellNetfpServerName,     NETFP_MAX_CHAR, "NetfpServer_LTE10");
    snprintf (lteCellNetfpConfigFileName, PATH_MAX,       "netfp_LTE10_D2.conf");

    /* Single NETFP Proxy which interfaces with the NETFP Server. */
    snprintf (netfpProxyPluginName, PATH_MAX,    "./netfpproxy_plugin_k2h.so");
    ptrNetfpProxyPollingDelay   =   "1";
    strncpy (lteCellNetfpProxyClientName, "Netfp_LTE10_PROXY", NETFP_MAX_CHAR);

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
    LOG_DEBUG ("Debug: Name Proxy[L2A] Local Flow Id     : %s\n",   ptrLocalL2AFlowId);
    LOG_DEBUG ("Debug: Name Proxy[L2A] Remote Flow Id    : %s\n",   ptrRemoteL2AFlowId);
    LOG_DEBUG ("Debug: Name Proxy[L2A] Base Address      : %s\n",   lteCellNameProxyL2ABaseAddress);
    LOG_DEBUG ("Debug: Name Proxy[L2A]                   : %s\n",   lteCellNameProxyL2A);
    LOG_DEBUG ("Debug: Name Proxy[L2B] Local Flow Id     : %s\n",   ptrLocalL2BFlowId);
    LOG_DEBUG ("Debug: Name Proxy[L2B] Remote Flow Id    : %s\n",   ptrRemoteL2BFlowId);
    LOG_DEBUG ("Debug: Name Proxy[L2B] Base Address      : %s\n",   lteCellNameProxyL2BBaseAddress);
    LOG_DEBUG ("Debug: Name Proxy[L2B]                   : %s\n",   lteCellNameProxyL2B);
    LOG_DEBUG ("Debug: DAT Server Name                   : %s\n",   lteCellDatServerName);
    LOG_DEBUG ("Debug: DAT Server Config                 : %s\n",   lteCellDatConfigFileName);
    LOG_DEBUG ("Debug: NETFP Server Name                 : %s\n",   lteCellNetfpServerName);
    LOG_DEBUG ("Debug: NETFP Server Config               : %s\n",   lteCellNetfpConfigFileName);

    /*******************************************************************************
     * Launching the SYSLIB NAME Proxy for L1A
     *******************************************************************************/
    argv[0]  = "./name_proxy_k2h.out";
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
    *nameProxyPidL1A = Rat_SpawnSyslibServer ("./name_proxy_k2h.out", argv);
    sleep(1);

    /*******************************************************************************
     * Launching the SYSLIB NAME Proxy for L1B
     *******************************************************************************/
    argv[0]  = "./name_proxy_k2h.out";
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
    *nameProxyPidL1B = Rat_SpawnSyslibServer ("./name_proxy_k2h.out", argv);
    sleep(1);

    /*******************************************************************************
     * Launching the SYSLIB NAME Proxy for L2A
     *******************************************************************************/
    argv[0]  = "./name_proxy_k2h.out";
    argv[1]  = "-n";
    argv[2]  = &lteCellNameProxyL2A[0];
    argv[3]  = "-c";
    argv[4]  = &lteCellRMClientName[0];
    argv[5]  = "-l";
    argv[6]  = ptrLocalL2AFlowId;
    argv[7]  = "-r";
    argv[8]  = ptrRemoteL2AFlowId;
    argv[9]  = "-i";
    argv[10] = ptrNamedResourceInstanceId;
    argv[11] = "-a";
    argv[12] = &lteCellNameProxyL2ABaseAddress[0];
    argv[13] = NULL;
    *nameProxyPidL2A = Rat_SpawnSyslibServer ("./name_proxy_k2h.out", argv);
    sleep(1);

    /*******************************************************************************
     * Launching the SYSLIB NAME Proxy for L2B
     *******************************************************************************/
    argv[0]  = "./name_proxy_k2h.out";
    argv[1]  = "-n";
    argv[2]  = &lteCellNameProxyL2B[0];
    argv[3]  = "-c";
    argv[4]  = &lteCellRMClientName[0];
    argv[5]  = "-l";
    argv[6]  = ptrLocalL2BFlowId;
    argv[7]  = "-r";
    argv[8]  = ptrRemoteL2BFlowId;
    argv[9]  = "-i";
    argv[10] = ptrNamedResourceInstanceId;
    argv[11] = "-a";
    argv[12] = &lteCellNameProxyL2BBaseAddress[0];
    argv[13] = NULL;
    *nameProxyPidL2B = Rat_SpawnSyslibServer ("./name_proxy_k2h.out", argv);
    sleep(1);

    /*******************************************************************************
     * Launching the SYSLIB NETFP Server
     *******************************************************************************/
    argv[0]  = "./netfp_server_k2h.out";
    argv[1]  = "-n";
    argv[2]  = &lteCellNetfpServerName[0];
    argv[3]  = "-r";
    argv[4]  = &lteCellRMClientName[0];
    argv[5]  = "-c";
    argv[6]  = &lteCellNetfpConfigFileName[0];
    argv[7]  = "-i";
    argv[8]  = ptrNamedResourceInstanceId;
    argv[9]  = NULL;
    *netfpServerPid = Rat_SpawnSyslibServer ("./netfp_server_k2h.out", argv);

    /*******************************************************************************
     * Launching the SYSLIB DAT Server
     *******************************************************************************/
    argv[0]  = "./dat_server_k2h.out";
    argv[1]  = "-n";
    argv[2]  = &lteCellDatServerName[0];
    argv[3]  = "-r";
    argv[4]  = lteCellRMClientName;
    argv[5]  = "-i";
    argv[6]  = ptrNamedResourceInstanceId;
    argv[7]  = "-c";
    argv[8]  = &lteCellDatConfigFileName[0];
    argv[9]  = NULL;
    *datServerPid = Rat_SpawnSyslibServer ("./dat_server_k2h.out", argv);

    /*******************************************************************************
     * Launching the SYSLIB NetFP Proxy for L2:
     *******************************************************************************/
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

#if 1
    /********************************************************************************
     * DSP Core Provisioning:
     *  - L2A is loaded on Core0 and Core3
     *  - L1A is loaded on Core1 and Core2
     *
     * DSP Core Provisioning: L2A_0 on Core0
     *******************************************************************************/

    /* Initialize the root SYSLIB configuration. */
    memset ((void *)&rootSyslibConfig, 0, sizeof(Root_SyslibConfig));

    /* Provision and configure the SYSLIB modules: */
    snprintf (rootSyslibConfig.name, ROOT_MAX_CHAR, "LTE10_L2");
    snprintf (rootSyslibConfig.rmClient, RM_NAME_MAX_CHARS, "Rm_LTE10_L2A");
    strcpy (rootSyslibConfig.rmServer, "Rm_Server");
    strcpy (rootSyslibConfig.proxyName, lteCellNameProxyL2A);
    rootSyslibConfig.coreId                                             = 0;
    rootSyslibConfig.nrInstanceId                                       = atoi(ptrNamedResourceInstanceId);
    rootSyslibConfig.qmssAccHwSemaphore                                 = qmssAccHwSemaphore;
    rootSyslibConfig.dspSyslibConfig.sharedMemAddress                   = rmBaseAddress;
    rootSyslibConfig.dspSyslibConfig.sizeSharedMemory                   = 8192;
    rootSyslibConfig.dspSyslibConfig.armCoreId                          = 8;
    rootSyslibConfig.dspSyslibConfig.sysRMIPCSourceId                   = 17;
    rootSyslibConfig.dspSyslibConfig.instantiateNameProxy               = 1;
    rootSyslibConfig.dspSyslibConfig.localFlowId                        = atoi(ptrRemoteL2AFlowId);
    rootSyslibConfig.dspSyslibConfig.remoteFlowId                       = atoi(ptrLocalL2AFlowId);
    rootSyslibConfig.dspSyslibConfig.namedResourceSharedMemAddress      = namedResL2ABaseAddress;
    rootSyslibConfig.dspSyslibConfig.sizeNamedResourceSharedMemAddress  = 4*1024;
    rootSyslibConfig.dspSyslibConfig.nameHwSemaphore                    = namedResourceL2AHwSem;
    rootSyslibConfig.dspSyslibConfig.initNamedResourceDatabase          = 1;
    rootSyslibConfig.dspSyslibConfig.nameProxySharedMemAddress          = nameProxyL2ABaseAddress;

    /* Instantiate the name client for L2A */
    rootSyslibConfig.nameClientConfig.instantiateNameClient             = 1;

    /* Instantiate the NETFP client */
    rootSyslibConfig.netfpClientConfig.instantiateNetfpClient           = 1;
    strcpy (rootSyslibConfig.netfpClientConfig.serverName, lteCellNetfpServerName);
    snprintf (rootSyslibConfig.netfpClientConfig.clientName, NETFP_MAX_CHAR, "Netfp_LTE10_L2A_0");

    /* Instantiate the DAT client */
    rootSyslibConfig.datClientConfig.instantiateDatClient               = 1;
    rootSyslibConfig.datClientConfig.datClientId                        = 0;
    strcpy (rootSyslibConfig.datClientConfig.serverName, lteCellDatServerName);
    snprintf (rootSyslibConfig.datClientConfig.clientName, DAT_MAX_CHAR, "Dat_LTE10_L2A_0");

    /* Initialize the L2 domain on the Core0 */
    if (Root_appInit (rootMasterHandle, ROOT_SLAVE_COREID(1, 0), LTE_REL10D2_L2A_DOMAIN_ID, &appConfig,
                      sizeof(appConfig), &rootSyslibConfig, &errCode) < 0)
    {
        LOG_ERROR ("Error: Root application initialization failed [Error code %d]\n", errCode);
        return -1;
    }
    LOG_DEBUG ("Debug: DSP Core0 initialization [Done]\n");

    /*******************************************************************************
     * DSP Core Provisioning: L2A_1 on Core3
     *******************************************************************************/

    /* Initialize the root SYSLIB configuration. */
    memset ((void *)&rootSyslibConfig, 0, sizeof(Root_SyslibConfig));

    /* Provision and configure the SYSLIB modules: */
    snprintf (rootSyslibConfig.name, ROOT_MAX_CHAR, "LTE10_L2");
    snprintf (rootSyslibConfig.rmClient, RM_NAME_MAX_CHARS, "Rm_LTE10_L2A");
    strcpy (rootSyslibConfig.rmServer, "Rm_Server");
    strcpy (rootSyslibConfig.proxyName, lteCellNameProxyL2A);
    rootSyslibConfig.coreId                                             = 3;
    rootSyslibConfig.nrInstanceId                                       = atoi(ptrNamedResourceInstanceId);
    rootSyslibConfig.qmssAccHwSemaphore                                 = qmssAccHwSemaphore;
    rootSyslibConfig.dspSyslibConfig.sharedMemAddress                   = rmBaseAddress;
    rootSyslibConfig.dspSyslibConfig.sizeSharedMemory                   = 8192;
    rootSyslibConfig.dspSyslibConfig.armCoreId                          = 8;
    rootSyslibConfig.dspSyslibConfig.sysRMIPCSourceId                   = 17;
    rootSyslibConfig.dspSyslibConfig.instantiateNameProxy               = 0;
    rootSyslibConfig.dspSyslibConfig.localFlowId                        = atoi(ptrRemoteL2AFlowId);
    rootSyslibConfig.dspSyslibConfig.remoteFlowId                       = atoi(ptrLocalL2AFlowId);
    rootSyslibConfig.dspSyslibConfig.namedResourceSharedMemAddress      = namedResL2ABaseAddress;
    rootSyslibConfig.dspSyslibConfig.sizeNamedResourceSharedMemAddress  = 4*1024;
    rootSyslibConfig.dspSyslibConfig.nameHwSemaphore                    = namedResourceL2AHwSem;
    rootSyslibConfig.dspSyslibConfig.initNamedResourceDatabase          = 0;
    rootSyslibConfig.dspSyslibConfig.nameProxySharedMemAddress          = nameProxyL2ABaseAddress;

    /* Instantiate the name client for L2A */
    rootSyslibConfig.nameClientConfig.instantiateNameClient             = 1;

    /* Instantiate the NETFP client */
    rootSyslibConfig.netfpClientConfig.instantiateNetfpClient           = 1;
    strcpy (rootSyslibConfig.netfpClientConfig.serverName, lteCellNetfpServerName);
    snprintf (rootSyslibConfig.netfpClientConfig.clientName, NETFP_MAX_CHAR, "Netfp_LTE10_L2A_1");

    /* Instantiate the DAT client */
    rootSyslibConfig.datClientConfig.instantiateDatClient               = 1;
    rootSyslibConfig.datClientConfig.datClientId                        = 3;
    strcpy (rootSyslibConfig.datClientConfig.serverName, lteCellDatServerName);
    snprintf (rootSyslibConfig.datClientConfig.clientName, DAT_MAX_CHAR, "Dat_LTE10_L2A_1");

    /* Initialize the L2 domain on the Core3 */
    if (Root_appInit (rootMasterHandle, ROOT_SLAVE_COREID(1, 3), (LTE_REL10D2_L2A_DOMAIN_ID+1), &appConfig,
                      sizeof(appConfig), &rootSyslibConfig, &errCode) < 0)
    {
        LOG_ERROR ("Error: Root application initialization failed [Error code %d]\n", errCode);
        return -1;
    }
    LOG_DEBUG ("Debug: DSP Core3 initialization [Done]\n");

    /*******************************************************************************
    * DSP Core Provisioning: L1A_0 on Core1
	 *******************************************************************************/

	/* Initialize the root SYSLIB configuration. */
	memset ((void *)&rootSyslibConfig, 0, sizeof(Root_SyslibConfig));

	/* Provision and configure the SYSLIB modules: */
	snprintf (rootSyslibConfig.name, ROOT_MAX_CHAR, "LTE10_L1A");
	snprintf (rootSyslibConfig.rmClient, RM_NAME_MAX_CHARS, "Rm_LTE10_L1A");
	strcpy (rootSyslibConfig.rmServer, "Rm_Server");
	strcpy (rootSyslibConfig.proxyName, lteCellNameProxyL1A);
	rootSyslibConfig.coreId                                             = 1;
	rootSyslibConfig.nrInstanceId                                       = atoi(ptrNamedResourceInstanceId);
    rootSyslibConfig.qmssAccHwSemaphore                                 = qmssAccHwSemaphore;
	rootSyslibConfig.dspSyslibConfig.sharedMemAddress                   = rmBaseAddress;
	rootSyslibConfig.dspSyslibConfig.sizeSharedMemory                   = 8192;
	rootSyslibConfig.dspSyslibConfig.armCoreId                          = 8;
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

	/* Instantiate the DAT client */
	rootSyslibConfig.datClientConfig.instantiateDatClient               = 1;
	rootSyslibConfig.datClientConfig.datClientId                        = 1;
	strcpy (rootSyslibConfig.datClientConfig.serverName, lteCellDatServerName);
	snprintf (rootSyslibConfig.datClientConfig.clientName, DAT_MAX_CHAR, "Dat_LTE10_L1A_0");

	/* Initialize the L1 PHY application */
	if (Root_appInit (rootMasterHandle, ROOT_SLAVE_COREID(1, 1), LTE_REL10D2_L1A_DOMAIN_ID, &appConfig,
					  sizeof(appConfig), &rootSyslibConfig, &errCode) < 0)
	{
		LOG_ERROR ("Error: Root application initialization failed [Error code %d]\n", errCode);
		return -1;
	}
	LOG_DEBUG ("Debug: DSP Core1 [Done]\n");

	/*******************************************************************************
	 * DSP Core Provisioning: L1A_1 on Core2
	 *******************************************************************************/

	/* Initialize the root SYSLIB configuration. */
	memset ((void *)&rootSyslibConfig, 0, sizeof(Root_SyslibConfig));

	/* Provision and configure the SYSLIB modules  */
	snprintf (rootSyslibConfig.name, ROOT_MAX_CHAR, "LTE10_L1A");
	snprintf (rootSyslibConfig.rmClient, RM_NAME_MAX_CHARS, "Rm_LTE10_L1A");
	strcpy (rootSyslibConfig.rmServer, "Rm_Server");
	strcpy (rootSyslibConfig.proxyName, lteCellNameProxyL1A);
	rootSyslibConfig.coreId                                             = 2;
	rootSyslibConfig.nrInstanceId                                       = atoi(ptrNamedResourceInstanceId);
    rootSyslibConfig.qmssAccHwSemaphore                                 = qmssAccHwSemaphore;
	rootSyslibConfig.dspSyslibConfig.sharedMemAddress                   = rmBaseAddress;
	rootSyslibConfig.dspSyslibConfig.sizeSharedMemory                   = 8192;
	rootSyslibConfig.dspSyslibConfig.armCoreId                          = 8;
	rootSyslibConfig.dspSyslibConfig.sysRMIPCSourceId                   = 17;
	rootSyslibConfig.dspSyslibConfig.instantiateNameProxy               = 0;
	rootSyslibConfig.dspSyslibConfig.namedResourceSharedMemAddress      = namedResL1ABaseAddress;
	rootSyslibConfig.dspSyslibConfig.sizeNamedResourceSharedMemAddress  = 4*1024;
    rootSyslibConfig.dspSyslibConfig.nameHwSemaphore                    = namedResourceL1AHwSem;
	rootSyslibConfig.dspSyslibConfig.initNamedResourceDatabase          = 0;
	rootSyslibConfig.dspSyslibConfig.nameProxySharedMemAddress          = nameProxyL1ABaseAddress;

	/* Instantiate the name client. */
	rootSyslibConfig.nameClientConfig.instantiateNameClient             = 1;

	/* Instantiate the NETFP client */
	rootSyslibConfig.netfpClientConfig.instantiateNetfpClient           = 1;
	strcpy (rootSyslibConfig.netfpClientConfig.serverName, lteCellNetfpServerName);
	snprintf (rootSyslibConfig.netfpClientConfig.clientName, NETFP_MAX_CHAR, "Netfp_LTE10_L1A_1");

	/* Instantiate the DAT client */
	rootSyslibConfig.datClientConfig.instantiateDatClient               = 1;
	rootSyslibConfig.datClientConfig.datClientId                        = 2;
	strcpy (rootSyslibConfig.datClientConfig.serverName, lteCellDatServerName);
	snprintf (rootSyslibConfig.datClientConfig.clientName, DAT_MAX_CHAR, "Dat_LTE10_L1A_1");

	/* Initialize the L1 PHY application */
	if (Root_appInit (rootMasterHandle, ROOT_SLAVE_COREID(1, 2), (LTE_REL10D2_L1A_DOMAIN_ID + 1), &appConfig,
					  sizeof(appConfig), &rootSyslibConfig, &errCode) < 0)
	{
		LOG_ERROR ("Error: Root application initialization failed [Error code %d]\n", errCode);
		return -1;
	}
	LOG_DEBUG ("Debug: DSP Core2 [Done]\n");
#endif

#if 1
    /********************************************************************************
     * DSP Core Provisioning:
     *  - L2B is loaded on Core4 and Core7
     *  - L1B is loaded on Core5 and Core6
     *
     * DSP Core Provisioning: L2B_0 on Core4
     *******************************************************************************/
    /* Initialize the root SYSLIB configuration. */
    memset ((void *)&rootSyslibConfig, 0, sizeof(Root_SyslibConfig));

    /* Provision and configure the SYSLIB modules: */
    snprintf (rootSyslibConfig.name, ROOT_MAX_CHAR, "LTE10_L2");
    snprintf (rootSyslibConfig.rmClient, RM_NAME_MAX_CHARS, "Rm_LTE10_L2B");
    strcpy (rootSyslibConfig.rmServer, "Rm_Server");
    strcpy (rootSyslibConfig.proxyName, lteCellNameProxyL2B);
    rootSyslibConfig.coreId                                             = 4;
    rootSyslibConfig.nrInstanceId                                       = atoi(ptrNamedResourceInstanceId);
    rootSyslibConfig.qmssAccHwSemaphore                                 = qmssAccHwSemaphore;
    rootSyslibConfig.dspSyslibConfig.sharedMemAddress                   = rmBaseAddress;
    rootSyslibConfig.dspSyslibConfig.sizeSharedMemory                   = 8192;
    rootSyslibConfig.dspSyslibConfig.armCoreId                          = 8;
    rootSyslibConfig.dspSyslibConfig.sysRMIPCSourceId                   = 17;
    rootSyslibConfig.dspSyslibConfig.instantiateNameProxy               = 1;
    rootSyslibConfig.dspSyslibConfig.localFlowId                        = atoi(ptrRemoteL2BFlowId);
    rootSyslibConfig.dspSyslibConfig.remoteFlowId                       = atoi(ptrLocalL2BFlowId);
    rootSyslibConfig.dspSyslibConfig.namedResourceSharedMemAddress      = namedResL2BBaseAddress;
    rootSyslibConfig.dspSyslibConfig.sizeNamedResourceSharedMemAddress  = 4*1024;
    rootSyslibConfig.dspSyslibConfig.nameHwSemaphore                    = namedResourceL2BHwSem;
    rootSyslibConfig.dspSyslibConfig.initNamedResourceDatabase          = 1;
    rootSyslibConfig.dspSyslibConfig.nameProxySharedMemAddress          = nameProxyL2BBaseAddress;

    /* Instantiate the name client for L2B */
    rootSyslibConfig.nameClientConfig.instantiateNameClient             = 1;

    /* Instantiate the NETFP client */
    rootSyslibConfig.netfpClientConfig.instantiateNetfpClient           = 1;
    strcpy (rootSyslibConfig.netfpClientConfig.serverName, lteCellNetfpServerName);
    snprintf (rootSyslibConfig.netfpClientConfig.clientName, NETFP_MAX_CHAR, "Netfp_LTE10_L2B_0");

    /* Instantiate the DAT client */
    rootSyslibConfig.datClientConfig.instantiateDatClient               = 1;
    rootSyslibConfig.datClientConfig.datClientId                        = 4;
    strcpy (rootSyslibConfig.datClientConfig.serverName, lteCellDatServerName);
    snprintf (rootSyslibConfig.datClientConfig.clientName, DAT_MAX_CHAR, "Dat_LTE10_L2B_0");

    /* Initialize the L2 domain on the Core4 */
    if (Root_appInit (rootMasterHandle, ROOT_SLAVE_COREID(1, 4), LTE_REL10D2_L2B_DOMAIN_ID, &appConfig,
                      sizeof(appConfig), &rootSyslibConfig, &errCode) < 0)
    {
        LOG_ERROR ("Error: Root application initialization failed [Error code %d]\n", errCode);
        return -1;
    }
    LOG_DEBUG ("Debug: DSP Core4 initialization [Done]\n");

    /*******************************************************************************
     * DSP Core Provisioning: L2B_1 on Core7
     *******************************************************************************/

    /* Initialize the root SYSLIB configuration. */
    memset ((void *)&rootSyslibConfig, 0, sizeof(Root_SyslibConfig));

    /* Provision and configure the SYSLIB modules: */
    snprintf (rootSyslibConfig.name, ROOT_MAX_CHAR, "LTE10_L2");
    snprintf (rootSyslibConfig.rmClient, RM_NAME_MAX_CHARS, "Rm_LTE10_L2B");
    strcpy (rootSyslibConfig.rmServer, "Rm_Server");
    strcpy (rootSyslibConfig.proxyName, lteCellNameProxyL2B);
    rootSyslibConfig.coreId                                             = 7;
    rootSyslibConfig.nrInstanceId                                       = atoi(ptrNamedResourceInstanceId);
    rootSyslibConfig.qmssAccHwSemaphore                                 = qmssAccHwSemaphore;
    rootSyslibConfig.dspSyslibConfig.sharedMemAddress                   = rmBaseAddress;
    rootSyslibConfig.dspSyslibConfig.sizeSharedMemory                   = 8192;
    rootSyslibConfig.dspSyslibConfig.armCoreId                          = 8;
    rootSyslibConfig.dspSyslibConfig.sysRMIPCSourceId                   = 17;
    rootSyslibConfig.dspSyslibConfig.instantiateNameProxy               = 0;
    rootSyslibConfig.dspSyslibConfig.localFlowId                        = atoi(ptrRemoteL2BFlowId);
    rootSyslibConfig.dspSyslibConfig.remoteFlowId                       = atoi(ptrLocalL2BFlowId);
    rootSyslibConfig.dspSyslibConfig.namedResourceSharedMemAddress      = namedResL2BBaseAddress;
    rootSyslibConfig.dspSyslibConfig.sizeNamedResourceSharedMemAddress  = 4*1024;
    rootSyslibConfig.dspSyslibConfig.nameHwSemaphore                    = namedResourceL2BHwSem;
    rootSyslibConfig.dspSyslibConfig.initNamedResourceDatabase          = 0;
    rootSyslibConfig.dspSyslibConfig.nameProxySharedMemAddress          = nameProxyL2BBaseAddress;

    /* Instantiate the name client for L2B */
    rootSyslibConfig.nameClientConfig.instantiateNameClient             = 1;

    /* Instantiate the NETFP client */
    rootSyslibConfig.netfpClientConfig.instantiateNetfpClient           = 1;
    strcpy (rootSyslibConfig.netfpClientConfig.serverName, lteCellNetfpServerName);
    snprintf (rootSyslibConfig.netfpClientConfig.clientName, NETFP_MAX_CHAR, "Netfp_LTE10_L2B_1");

    /* Instantiate the DAT client */
    rootSyslibConfig.datClientConfig.instantiateDatClient               = 1;
    rootSyslibConfig.datClientConfig.datClientId                        = 7;
    strcpy (rootSyslibConfig.datClientConfig.serverName, lteCellDatServerName);
    snprintf (rootSyslibConfig.datClientConfig.clientName, DAT_MAX_CHAR, "Dat_LTE10_L2B_1");

    /* Initialize the L2 domain on the Core7 */
    if (Root_appInit (rootMasterHandle, ROOT_SLAVE_COREID(1, 7), (LTE_REL10D2_L2B_DOMAIN_ID+1), &appConfig,
                      sizeof(appConfig), &rootSyslibConfig, &errCode) < 0)
    {
        LOG_ERROR ("Error: Root application initialization failed [Error code %d]\n", errCode);
        return -1;
    }
    LOG_DEBUG ("Debug: DSP Core7 initialization [Done]\n");

	/*******************************************************************************
	 * DSP Core Provisioning: L1B_0 on Core5
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
	rootSyslibConfig.coreId                                             = 5;
	rootSyslibConfig.nrInstanceId                                       = atoi(ptrNamedResourceInstanceId);
    rootSyslibConfig.qmssAccHwSemaphore                                 = qmssAccHwSemaphore;
	rootSyslibConfig.dspSyslibConfig.sharedMemAddress                   = rmBaseAddress;
	rootSyslibConfig.dspSyslibConfig.sizeSharedMemory                   = 8192;
	rootSyslibConfig.dspSyslibConfig.armCoreId                          = 8;
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

	/* Instantiate the DAT client */
	rootSyslibConfig.datClientConfig.instantiateDatClient               = 1;
	rootSyslibConfig.datClientConfig.datClientId                        = 5;
	strcpy (rootSyslibConfig.datClientConfig.serverName, lteCellDatServerName);
	snprintf (rootSyslibConfig.datClientConfig.clientName, DAT_MAX_CHAR, "Dat_LTE10_L1B_0");

	/* Initialize the L1 PHY application */
	if (Root_appInit (rootMasterHandle, ROOT_SLAVE_COREID(1, 5), LTE_REL10D2_L1B_DOMAIN_ID, &appConfig,
					  sizeof(appConfig), &rootSyslibConfig, &errCode) < 0)
	{
		LOG_ERROR ("Error: Root application initialization failed [Error code %d]\n", errCode);
		return -1;
	}
	LOG_DEBUG ("Debug: DSP Core5 [Done]\n");

	/*******************************************************************************
	 * DSP Core Provisioning: L1B_1 on Core6
	 *******************************************************************************/

	/* Initialize the root SYSLIB configuration. */
	memset ((void *)&rootSyslibConfig, 0, sizeof(Root_SyslibConfig));

	/* Provision and configure the SYSLIB modules  */
	snprintf (rootSyslibConfig.name, ROOT_MAX_CHAR, "LTE10_L1B");
	snprintf (rootSyslibConfig.rmClient, RM_NAME_MAX_CHARS, "Rm_LTE10_L1B");
	strcpy (rootSyslibConfig.rmServer, "Rm_Server");
	strcpy (rootSyslibConfig.proxyName, lteCellNameProxyL1B);
	rootSyslibConfig.coreId                                             = 6;
	rootSyslibConfig.nrInstanceId                                       = atoi(ptrNamedResourceInstanceId);
    rootSyslibConfig.qmssAccHwSemaphore                                 = qmssAccHwSemaphore;
	rootSyslibConfig.dspSyslibConfig.sharedMemAddress                   = rmBaseAddress;
	rootSyslibConfig.dspSyslibConfig.sizeSharedMemory                   = 8192;
	rootSyslibConfig.dspSyslibConfig.armCoreId                          = 8;
	rootSyslibConfig.dspSyslibConfig.sysRMIPCSourceId                   = 17;
	rootSyslibConfig.dspSyslibConfig.instantiateNameProxy               = 0;
	rootSyslibConfig.dspSyslibConfig.namedResourceSharedMemAddress      = namedResL1BBaseAddress;
	rootSyslibConfig.dspSyslibConfig.sizeNamedResourceSharedMemAddress  = 4*1024;
    rootSyslibConfig.dspSyslibConfig.nameHwSemaphore                    = namedResourceL1BHwSem;
	rootSyslibConfig.dspSyslibConfig.initNamedResourceDatabase          = 0;
	rootSyslibConfig.dspSyslibConfig.nameProxySharedMemAddress          = nameProxyL1BBaseAddress;

	/* Instantiate the name client. */
	rootSyslibConfig.nameClientConfig.instantiateNameClient             = 1;

	/* Instantiate the NETFP client */
	rootSyslibConfig.netfpClientConfig.instantiateNetfpClient           = 1;
	strcpy (rootSyslibConfig.netfpClientConfig.serverName, lteCellNetfpServerName);
	snprintf (rootSyslibConfig.netfpClientConfig.clientName, NETFP_MAX_CHAR, "Netfp_LTE10_L1B_1");

	/* Instantiate the DAT client */
	rootSyslibConfig.datClientConfig.instantiateDatClient               = 1;
	rootSyslibConfig.datClientConfig.datClientId                        = 6;
	strcpy (rootSyslibConfig.datClientConfig.serverName, lteCellDatServerName);
	snprintf (rootSyslibConfig.datClientConfig.clientName, DAT_MAX_CHAR, "Dat_LTE10_L1B_1");

	/* Initialize the L1 PHY application */
	if (Root_appInit (rootMasterHandle, ROOT_SLAVE_COREID(1, 6), (LTE_REL10D2_L1B_DOMAIN_ID + 1), &appConfig,
					  sizeof(appConfig), &rootSyslibConfig, &errCode) < 0)
	{
		LOG_ERROR ("Error: Root application initialization failed [Error code %d]\n", errCode);
		return -1;
	}
	LOG_DEBUG ("Debug: DSP Core6 [Done]\n");
#endif

#if 1
    /* Do the policy offloads: */
    Rat_DoPolicyOffloads (*nameDBHandle, *netfpProxyPid);

    /* Loop around till the RAT is operational */
    printf ("Debug: Waiting for all the domains to be operational\n");
    while (1)
    {
        if ((appDomainHandle[0] != NULL) && (appDomainHandle[1] != NULL) && (appDomainHandle[2] != NULL) &&
            (appDomainHandle[3] != NULL) && (appDomainHandle[4] != NULL) && (appDomainHandle[5] != NULL) &&
            (appDomainHandle[6] != NULL) && (appDomainHandle[7] != NULL))
                break;

        /* Wait & sleep for some time */
        sleep(1);
    }
#else
    /* Do the policy offloads: */
    Rat_DoPolicyOffloads ("10", atoi(ptrNamedResourceInstanceId), *netfpProxyPid);

    /* Loop around till the RAT is operational */
    printf ("Debug: Waiting for all the domains to be operational\n");
    while (1)
    {
        if ((appDomainHandle[4] != NULL) && (appDomainHandle[5] != NULL) &&
            (appDomainHandle[6] != NULL) && (appDomainHandle[7] != NULL))
                break;

        /* Wait & sleep for some time */
        sleep(1);
    }
#endif
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function deinitializes the RAT Release 10 Deployment2 LTE cell
 *
 *  @param[in]  ratNameDBHandle
 *      RAT Name database handle
 *  @param[in]  nameProxyPidL1A
 *      Name Proxy process identifier associated with the L1A PHY
 *  @param[in]  nameProxyPidL1B
 *      Name Proxy process identifier associated with the L1B PHY
 *  @param[in]  nameProxyPidL2A
 *      Name Proxy process identifier associated with the L2A PHY
 *  @param[in]  nameProxyPidL2B
 *      Name Proxy process identifier associated with the L2B PHY
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
static int32_t Rat_Rel10D2DeinitializeCell
(
    Name_DBHandle   ratNameDBHandle,
    pid_t           nameProxyPidL1A,
    pid_t           nameProxyPidL1B,
    pid_t           nameProxyPidL2A,
    pid_t           nameProxyPidL2B,
    pid_t           netfpServerPid,
    pid_t           datServerPid,
    pid_t           netfpProxyPid
)
{
    int32_t errCode;

    /**********************************************************************************
     * Shutdown the Release 10-D2 cell
     *
     * We start by shutting down all the DSP cores for carrier A and carrier B which
     * dont execute the NAME Proxy. Once they have been shutdown successfully; we then
     * shutdown the NAME Proxy cores. This is because the Name Proxy is required by
     * the other SYSLIB services to shut themselves down.
     **********************************************************************************/

    /* Deinitialize the L1A PHY application */
    if (Root_appDeinit (rootMasterHandle, ROOT_SLAVE_COREID(1, 2), appDomainHandle[2], &errCode) < 0)
    {
        LOG_ERROR ("Error: Root application deinitialization on Core2 failed [Error code %d]\n", errCode);
        return -1;
    }
    LOG_DEBUG ("Debug: DSP Core2 deinitialization initiated\n");

    /* Deinitialize the L2 application */
    if (Root_appDeinit (rootMasterHandle, ROOT_SLAVE_COREID(1, 3), appDomainHandle[3], &errCode) < 0)
    {
        LOG_ERROR ("Error: Root application deinitialization on Core3 failed [Error code %d]\n", errCode);
        return -1;
    }
    LOG_DEBUG ("Debug: DSP Core3 deinitialization initiated\n");

    /* Deinitialize the L1B PHY application */
    if (Root_appDeinit (rootMasterHandle, ROOT_SLAVE_COREID(1, 6), appDomainHandle[6], &errCode) < 0)
    {
        LOG_ERROR ("Error: Root application deinitialization on Core6 failed [Error code %d]\n", errCode);
        return -1;
    }
    LOG_DEBUG ("Debug: DSP Core6 deinitialization initiated\n");

    /* Deinitialize the L2 application */
    if (Root_appDeinit (rootMasterHandle, ROOT_SLAVE_COREID(1, 7), appDomainHandle[7], &errCode) < 0)
    {
        LOG_ERROR ("Error: Root application deinitialization on Core7 failed [Error code %d]\n", errCode);
        return -1;
    }
    LOG_DEBUG ("Debug: DSP Core7 deinitialization initiated\n");

    /* Make sure all the DSP cores are down */
    while (1)
    {
        if ((appDomainHandle[2] == NULL) && (appDomainHandle[3] == NULL) &&
            (appDomainHandle[6] == NULL) && (appDomainHandle[7] == NULL))
            break;

        /* Wait & sleep for some time */
        sleep(1);
    }

    /**********************************************************************************
     * Control comes here implies that we can now shutdown the remaining DSP cores.
     **********************************************************************************/

    /* Deinitialize the L2 application */
    if (Root_appDeinit (rootMasterHandle, ROOT_SLAVE_COREID(1, 0), appDomainHandle[0], &errCode) < 0)
    {
        LOG_ERROR ("Error: Root application deinitialization on Core0 failed [Error code %d]\n", errCode);
        return -1;
    }
    LOG_DEBUG ("Debug: DSP Core0 deinitialization initiated\n");

    /* Deinitialize the L1A PHY application */
    if (Root_appDeinit (rootMasterHandle, ROOT_SLAVE_COREID(1, 1), appDomainHandle[1], &errCode) < 0)
    {
        LOG_ERROR ("Error: Root application deinitialization on Core1 failed [Error code %d]\n", errCode);
        return -1;
    }
    LOG_DEBUG ("Debug: DSP Core1 deinitialization initiated\n");

    /* Deinitialize the L2 application */
    if (Root_appDeinit (rootMasterHandle, ROOT_SLAVE_COREID(1, 4), appDomainHandle[4], &errCode) < 0)
    {
        LOG_ERROR ("Error: Root application deinitialization on Core4 failed [Error code %d]\n", errCode);
        return -1;
    }
    LOG_DEBUG ("Debug: DSP Core4 deinitialization initiated\n");

    /* Deinitialize the L1B PHY application */
    if (Root_appDeinit (rootMasterHandle, ROOT_SLAVE_COREID(1, 5), appDomainHandle[5], &errCode) < 0)
    {
        LOG_ERROR ("Error: Root application deinitialization on Core5 failed [Error code %d]\n", errCode);
        return -1;
    }
    LOG_DEBUG ("Debug: DSP Core5 deinitialization initiated\n");

    /* Make sure all the domain are down */
    while (1)
    {
        if ((appDomainHandle[0] == NULL) && (appDomainHandle[1] == NULL) &&
            (appDomainHandle[4] == NULL) && (appDomainHandle[5] == NULL))
            break;

        /* Wait & sleep for some time */
        sleep(1);
    }

    /* Stop NETFP Proxy */
    kill (netfpProxyPid, SIGTERM);
    LOG_DEBUG ("Debug: NETFP Proxy is down!\n");

    /* NETFP Server can now be deleted: All the NETFP clients have been deinitialized. */
    kill (netfpServerPid, SIGTERM);
    LOG_DEBUG ("Debug: NETFP Server is down!\n");

    /* Shutdown the DAT Server: All the DAT clients have been deinitialized */
    kill (datServerPid, SIGTERM);
    LOG_DEBUG ("Debug: DAT Server is down!\n");

    /* Purge the security policies which have been offloaded: */
    if (Name_deleteResource (ratNameDBHandle, Name_ResourceBucket_USER_DEF1,
                             "Egress_SPID_IPv4_UP",  &errCode) < 0)
        LOG_ERROR ("Debug: Egress security policy name deletion failed [Error code %d]\n", errCode);
    if (Name_deleteResource (ratNameDBHandle, Name_ResourceBucket_USER_DEF1,
                             "Ingress_SPID_IPv4_UP",  &errCode) < 0)
        LOG_ERROR ("Debug: Ingress security policy name deletion failed [Error code %d]\n", errCode);

    LOG_INFO ("Debug: RAT domains on the DSP have been shutdown successfully\n");

    /* Shutdown the name proxies */
    kill (nameProxyPidL1A, SIGTERM);
    kill (nameProxyPidL1B, SIGTERM);
    kill (nameProxyPidL2A, SIGTERM);
    kill (nameProxyPidL2B, SIGTERM);

    /* Delete the RAT name database */
    Name_deleteDatabase (ratNameDBHandle, &errCode);

    LOG_INFO ("Debug: RAT has been shutdown!\n");
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function handles the Rel10 Deployment2 CLI
 *
 *  @retval
 *      Not Applicable.
 */
void Rat_Rel10D2_CLI (void)
{
    int32_t         testSelection;
    uint32_t        ratCellStatus = 0;
    pid_t           nameProxyL1APid;
    pid_t           nameProxyL1BPid;
    pid_t           nameProxyL2APid;
    pid_t           nameProxyL2BPid;
    pid_t           netfpServerPid;
    pid_t           datServerPid;
    pid_t           netfpProxyPid;
    Name_DBHandle   ratNameDBHandle;

    /* Release 10: */
    while (1)
    {
        LOG_DEBUG ("******************************************\n");
        LOG_DEBUG ("1. Launch   Release 10 D2 Cell with L1A, L1B\n");
        LOG_DEBUG ("2. Shutdown Release 10 D2 Cell\n");
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
                /* Bring up the cell only if it is down! */
                if (ratCellStatus != 0)
                    continue;

                /* Release 10 D2 With both L1A and L1B */
                Rat_Rel10D2InitializeCell(&ratNameDBHandle, &nameProxyL1APid, &nameProxyL1BPid, &nameProxyL2APid,
                                          &nameProxyL2BPid, &netfpServerPid, &datServerPid, &netfpProxyPid);

                /* Update the RAT Cell Status (Both A & B) are operational. */
                ratCellStatus = 1;
                break;
            }
            case 2:
            {
                /* Shutdown the cell only if it is up! */
                if (ratCellStatus == 0)
                    continue;

                /* Release 10: Shutdown the cell. */
                Rat_Rel10D2DeinitializeCell (ratNameDBHandle, nameProxyL1APid, nameProxyL1BPid, nameProxyL2APid,
                                             nameProxyL2BPid, netfpServerPid, datServerPid, netfpProxyPid);

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


