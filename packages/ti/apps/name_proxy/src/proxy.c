/**
 *   @file  proxy.c
 *
 *   @brief
 *      Name proxy daemon which executes on the ARM and ensures that the
 *      name client on the DSP and ARM can exchange information with each
 *      other.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2013-2104 Texas Instruments, Inc.
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
#include <stdarg.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
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
#include <ti/runtime/name/name.h>
#include <ti/runtime/name/name_proxyClient.h>
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/apps/name_proxy/include/name_proxy.h>

/**********************************************************************
 ******************* Name Proxy Data Structures ***********************
 **********************************************************************/

/**
 * @brief
 *  Proxy MCB
 *
 * @details
 *  This is the master control block which holds all the relevant information
 *  for the NAME Proxy instance
 */
typedef struct NameProxy_MCB
{
    /**
     * @brief Proxy Name
     */
    char                    proxyName[128];

    /**
     * @brief RM Client Name
     */
    char                    rmClientName[128];

    /**
     * @brief Logging Level for the NAME Proxy application
     */
    NameProxy_LogLevel      logLevel;

    /**
     * @brief Polling Timeout in milliseconds.
     */
    uint32_t                pollingTimeout;

    /**
     * @brief Physical Shared Memory Address.
     */
    uint32_t                sharedMemoryAddress;

    /**
     * @brief   Local flow identifier: This is the flow identifier which is used
     * to receive messages from the peer proxy
     */
    uint32_t                localFlowId;

    /**
     * @brief   Remote flow identifier: This is the flow identifier which is used
     * to send messages to the peer proxy
     */
    uint32_t                remoteFlowId;

    /**
     * @brief   Named resource instance identifier to be used for the name proxy
     */
    uint32_t                databaseInstanceId;

    /**
     * @brief   Database handle associated with the database instance identifier
     */
    Name_DBHandle           databaseHandle;

    /**
     * @brief   PKTLIB Instance handle
     */
    Pktlib_InstHandle       pktlibInstanceHandle;

    /**
     * @brief Proxy State
     */
    uint32_t                proxyState;

    /**
     * @brief Name Proxy Handle
     */
    Name_ProxyHandle        nameProxyHandle;

    /**
     * @brief Name Proxy Heap handle which is used to exchange messages with the peer
     * name proxy executing on the DSP.
     */
    Pktlib_HeapHandle       nameProxyHeapHandle;

    /**
     * @brief Global System configuration handle
     */
    Resmgr_SysCfgHandle     handleSysCfg;
}NameProxy_MCB;

/**********************************************************************
 * OSAL Callout Functions:
 **********************************************************************/

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

/* PKTLIB: */
extern void* Pktlib_osalMalloc(Pktlib_MallocMode, uint32_t);
extern void  Pktlib_osalFree(Pktlib_MallocMode, void*, uint32_t);
extern void  Pktlib_osalBeginMemAccess(void*, uint32_t);
extern void  Pktlib_osalEndMemAccess(void*, uint32_t);
extern void  Pktlib_osalBeginPktAccess(Pktlib_HeapHandle, Ti_Pkt*, uint32_t);
extern void  Pktlib_osalEndPktAccess(Pktlib_HeapHandle, Ti_Pkt*, uint32_t);
extern void* Pktlib_osalEnterCS(Pktlib_HeapHandle);
extern void  Pktlib_osalExitCS(Pktlib_HeapHandle, void*);
extern void* Pktlib_osalPhyToVirt(void* );

/* Name Proxy/Client: */
extern void* Name_osalMalloc (uint32_t, uint32_t);
extern void  Name_osalFree (void* , uint32_t);
extern void  Name_osalBeginMemAccess (void*, uint32_t);
extern void  Name_osalEndMemAccess (void*, uint32_t);
extern void* Name_osalEnterCS (void);
extern void  Name_osalExitCS (void*);

/**********************************************************************
 *********************** Global Declarations **************************
 **********************************************************************/

/* Name Proxy MCB */
NameProxy_MCB           gNameProxyMCB;

/* Application requested resources */
Resmgr_ResourceCfg      appResourceConfig =
{
	0,    /* Number of CPINTC Output  requested                               */
	0,    /* Number of Accumulator Channels requested                         */
	0,    /* Number of Hardware Semaphores requested                          */
	0,    /* Number of QPEND Queues requested                                 */
    /* Requested Memory Region Configuration. */
	{
        /* Name,           Type,                    Linking RAM,                           Num,     Size */
		{ "ARM-DDR3-0", Resmgr_MemRegionType_DDR3,  Resmgr_MemRegionLinkingRAM_DONT_CARE,  32,      128},
    }
};

/**********************************************************************
 ************************* Unit Test Functions ************************
 **********************************************************************/

/**
 *  @b Description
 *  @n
 *      Heap Data Buffer Allocator which is instantiated into the heap
 *      interface table.
 *
 *  @param[in]  size
 *      Size of the memory to be allocated
 *  @param[in]  arg
 *      Application specific argument
 *
 *  @retval
 *      Success     -   Pointer to the allocated block of memory.
 *  @retval
 *      Error       -   NULL
 */
static uint8_t* myMalloc(uint32_t size, uint32_t arg)
{
    uint8_t*    ptr;

    /* Allocate memory from the HPLIB pools. */
    ptr = (uint8_t *)hplib_vmMemAlloc (size, 0, 0);
    if (ptr == NULL)
        return NULL;
    return ptr;
}

/**
 *  @b Description
 *  @n
 *      Heap Data Buffer Cleanup which is instantiated into the heap
 *      interface table.
 *
 *  @param[in]  ptr
 *      Pointer to the memory which is to be cleaned up.
 *  @param[in]  size
 *      Size of the memory which is to be cleaned up.
 *  @param[in]  arg
 *      Application specific argument
 *
 *  @retval
 *      Not Applicable.
 */
static void myFree(uint8_t* ptr, uint32_t size, uint32_t arg)
{
    hplib_vmMemFree (ptr, size, 0);
}

/**
 *  @b Description
 *  @n
 *     Display the usage for the resource manager server
 *
 *  @retval
 *      Not Applicable.
 */
static void NameProxy_displayUsage (void)
{
    printf ("Name Proxy:\n");
    printf ("name_proxy <arguments>\n");
    printf ("Mandatory Arguments: \n");
    printf ("-n <name>         - Name of the Proxy server; unique in the system\n");
    printf ("-c <name>         - Name of the RM Client to be used; this should match the client names in the RMv2 Policy file\n");
    printf ("-l <flowId>       - Local flow identifier used to receive packets from the remote peer\n");
    printf ("-r <flowId>       - Remote flow identifier used to send packets to the remote peer\n");
    printf ("-i <InstantId>    - Named resource instant identifier\n");
    printf ("Optional Arguments: \n");
    printf ("-a <address>      - Shared memory address for DSP clients [Default is 0xA001F000]\n");
    printf ("-v                - Verbosity\n");
    return;
}

/**
 *  @b Description
 *  @n
 *      The function process the command line arguments passed to the applicaton
 *
 *  @param[in]  argc
 *      Number of arguments.
 *  @param[in]  argv
 *      Command Line arguments.
 *
 *  @retval
 *      Success 	- 0
 *  @retval
 *      Error		- <0
 */
static int32_t NameProxy_processCmdLineArgs(int32_t argc, char* argv[])
{
    while (1)
    {
        int option_index = 0;
        int c;

        static struct option long_options[] = {
            {"name",            required_argument, 0,  0 },
            {"clientName",      required_argument, 0,  0 },
            {"timeout",         required_argument, 0,  0 },
            {"instantId",       required_argument, 0,  0 },
            {"address",         required_argument, 0,  0 },
            {"local",           required_argument, 0,  0 },
            {"remote",          required_argument, 0,  0 },
            {0,         0,                 0,  0 }
        };

        c = getopt_long(argc, argv, "vn:c:r:l:i:a:", long_options, &option_index);
        if (c == -1)
            break;

       switch (c)
       {
            case 'n':
            {
                /* Proxy Name: */
                strncpy (gNameProxyMCB.proxyName, optarg, sizeof(gNameProxyMCB.proxyName));
                break;
            }
            case 'c':
            {
                /* RM Client Name: */
                strncpy (gNameProxyMCB.rmClientName, optarg, sizeof(gNameProxyMCB.rmClientName));
                break;
            }
            case 'i':
            {
                /* Database instance identifier */
                gNameProxyMCB.databaseInstanceId = atoi (optarg);
                break;
            }
            case 't':
            {
                /* Polling Timeout: This is specified in micro-seconds */
                gNameProxyMCB.pollingTimeout = atoi (optarg);
                gNameProxyMCB.pollingTimeout = gNameProxyMCB.pollingTimeout * 1000;
                break;
            }
            case 'a':
            {
                /* Shared Memory Address: */
                sscanf (optarg, "0x%x", &gNameProxyMCB.sharedMemoryAddress);
                break;
            }
            case 'l':
            {
                /* Local flow identifier */
                gNameProxyMCB.localFlowId = atoi (optarg);
                break;
            }
            case 'r':
            {
                /* Remote flow identifier */
                gNameProxyMCB.remoteFlowId = atoi (optarg);
                break;
            }
            case 'v':
            {
                gNameProxyMCB.logLevel = NameProxy_LogLevel_DEBUG;
                break;
            }
            case '?':
            {
                return -1;
            }
        }
    }

    /* We should have a valid proxy name */
    if (gNameProxyMCB.proxyName[0] == 0)
        return -1;

    /* We should have a valid RM client name */
    if (gNameProxyMCB.rmClientName[0] == 0)
        return -1;

    /* We should have a valid local & remote flow identifier specified in the argument list. */
    if ((gNameProxyMCB.localFlowId == 0xFFFFFFFF) || (gNameProxyMCB.remoteFlowId == 0xFFFFFFFF))
        return -1;

    /* Command Line Arguments processed successfully. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Logging Function used to log messages generated by the NAME Proxy
 *
 *  @param[in]  level
 *      Log level
 *  @param[in]  fmt
 *      Formatting string
 *
 *  @retval
 *      Not Applicable.
 */
void NameProxy_log (NameProxy_LogLevel level, char* fmt, ...)
{
	va_list arg;

    /* Log the message as per the configured level. */
    if (level >= gNameProxyMCB.logLevel)
    {
    	/* Log the message on the console. */
    	va_start (arg, fmt);
	    vprintf (fmt, arg);
    	va_end (arg);
    }
	return;
}

/**
 *  @b Description
 *  @n
 *      Logging function which is registered with the proxy library
 *
 *  @param[in]  logLevel
 *      Name Log Level
 *  @param[in]  fmt
 *      Formatted string
 *  @param[in]  arg
 *      Variable length argument list
 *
 *  @retval
 *      Not applicable
 */
static void NameProxy_LogFxn (Name_LogLevel logLevel, char* fmt, va_list arg)
{
    /* If the name proxy library has reported an error message; we always log this */
    if (logLevel == Name_LogLevel_ERROR)
    {
        vprintf (fmt, arg);
        return;
    }

    /* If the name proxy library has reported a debug message and the server is executing
     * with the verbosity on. Log the message. */
    if ((logLevel == Name_LogLevel_DEBUG) && (gNameProxyMCB.logLevel == NameProxy_LogLevel_DEBUG))
        vprintf (fmt, arg);
}

/**
 *  @b Description
 *  @n
 *      Signal Handler installed to catch the SIGINT
 *
 *  @param[in]  signo
 *      Signal Number
 *  @param[in]  siginfo
 *      Signal Information
 *  @param[in]  context
 *      Context information.
 *
 *  @retval
 *      Not Applicable.
 */
static void NameProxy_terminated (int sig, siginfo_t *siginfo, void *context)
{
    /* Name Proxy is going down. */
    NameProxy_log(NameProxy_LogLevel_DEBUG, "Debug: Signal received\n");
    gNameProxyMCB.proxyState = 0;
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to initialize and setup the shared memory
 *      synchronization between the DSP and ARM. This is used to ensure
 *      that the control channels are setup properly on the DSP & ARM.
 *
 *  @retval
 *      Success 	- 0
 *  @retval
 *      Error		- <0
 */
static int32_t NameProxy_initSharedMemSetup(void)
{
    void*           mmapBaseAddress;
    uint32_t        pageSize;
    int32_t         retVal;
    uint32_t        mask;
    uint32_t        size;
    uint32_t        mmapFD;

    /* Minimum page size for the mmapped region. */
    size = 4096;
    mask = size - 1;

    /* Open the file that will be mapped. */
    if((mmapFD = open("/dev/mem", (O_RDWR | O_SYNC))) == -1)
        return -1;

    /* Get the page size. */
    retVal = sysconf(_SC_PAGE_SIZE);
    if (retVal < 0)
        return -1;

    /* Ensure block size and physical addresses are aligned to page size. */
    pageSize = (uint32_t)retVal;
    if ((size % pageSize) || ((uint32_t)gNameProxyMCB.sharedMemoryAddress % pageSize))
        return -1;

    /* Create a mapping of the physical address. */
    mmapBaseAddress = mmap(0, size, (PROT_READ|PROT_WRITE), MAP_SHARED, mmapFD, (off_t)gNameProxyMCB.sharedMemoryAddress & ~mask);
    if(mmapBaseAddress == MAP_FAILED)
        return -1;
    gNameProxyMCB.sharedMemoryAddress = (uint32_t)(mmapBaseAddress + ((off_t)gNameProxyMCB.sharedMemoryAddress & mask));

    /* Initialize the mapped region. */
    memset ((void*)gNameProxyMCB.sharedMemoryAddress, 0, size);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the NAME Proxy thread which provides an application context
 *      for the proxy to execute
 *
 *  @param[in]  arg
 *      Argument passed to the thread.
 *
 *  @retval
 *      Not Applicable.
 */
static void* NameProxy_thread(void *arg)
{
    int32_t     errCode;
    int32_t     synchStatus;

    /* Debug Message: */
    NameProxy_log(NameProxy_LogLevel_DEBUG, "Debug: Name Proxy execution thread started\n");

    /* Waiting for the peer servers to be synchronized. */
    while (gNameProxyMCB.proxyState == 1)
    {
        /* Get the proxy synchronization status. */
        synchStatus = Name_isProxySynched (gNameProxyMCB.nameProxyHandle, &errCode);
        if (synchStatus < 0)
        {
            NameProxy_log(NameProxy_LogLevel_ERROR, "Error: PROXY Synchronization failed [Error code %d]\n", errCode);
            return NULL;
        }
        if (synchStatus == 1)
            break;

        /* Polling Delay: */
        usleep (gNameProxyMCB.pollingTimeout);
        continue;
    }

    /* We could come out of the loop because:
     *  1. Synchronization with the remote peer has been completed.
     *  2. Process has been terminated. */
    if (gNameProxyMCB.proxyState == 1)
        NameProxy_log(NameProxy_LogLevel_DEBUG, "Debug: Synchronization acheived with peer proxy\n");
    else
        NameProxy_log(NameProxy_LogLevel_DEBUG, "Debug: Name proxy is going down!\n");

    /* Execute the NAME Proxy until terminated */
    while (1)
    {
        /* Execute the proxy: */
        if (Name_executeProxy (gNameProxyMCB.nameProxyHandle, &errCode) < 0)
            NameProxy_log(NameProxy_LogLevel_ERROR, "Error: Name proxy execution failed [Error code %d]\n", errCode);

        /* Have we been asked to shutdown? */
        if (gNameProxyMCB.proxyState == 0)
        {
            /* Name Proxy has been shutdown. */
            NameProxy_log(NameProxy_LogLevel_DEBUG, "Debug: Name Proxy shutting down\n");

            /* YES. Try and shutdown the proxy which cannot be done if there are pending
             * requests which are still being handled. */
            if (Name_deleteProxy (gNameProxyMCB.nameProxyHandle, &errCode) == 0)
                break;

            /* Error: Name Proxy deletion failed. Determine the error */
            if (errCode != NAME_ENOTREADY)
            {
                /* FATAL error: Proxy deletion failed.  */
                NameProxy_log(NameProxy_LogLevel_ERROR, "Error: Unable to delete the name proxy [Error code %d]\n", errCode);
                return NULL;
            }
        }
        /* Polling Delay: */
        usleep (gNameProxyMCB.pollingTimeout);
    }
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      Entry point into the name proxy.
 *
 *  @param[in]  argc
 *      Number of arguments.
 *  @param[in]  argv
 *      Command Line arguments.
 *
 *  @retval
 *      Success 	- 0
 *  @retval
 *      Error		- <0
 */
int32_t main(int argc, char* argv[])
{
    Resmgr_ResourceCfg*         ptrResCfg;
    int32_t                     errCode;
    Pktlib_HeapCfg              heapCfg;
    Resmgr_SystemCfg            sysConfig;
    Name_DatabaseCfg            databaseCfg;
    Pktlib_InstCfg              pktlibInstCfg;
    struct sigaction            act;
    int32_t                     retVal;
    pthread_t                   proxyThread;
    Name_ProxyCfg               proxyCfg;

    /* Initialize the Name Proxy MCB */
    memset ((void *)&gNameProxyMCB, 0, sizeof(NameProxy_MCB));

    /* Setup the defaults.
     *  - Polling Timeout is 100msec.
     *  - Address 0xA001F000 is used for the shared memory transport
     *  - Invalid local and remote flow identifiers. */
    gNameProxyMCB.logLevel            = NameProxy_LogLevel_ERROR;
    gNameProxyMCB.pollingTimeout      = 100*1000;
    gNameProxyMCB.sharedMemoryAddress = 0xA001F000;
    gNameProxyMCB.localFlowId         = 0xFFFFFFFF;
    gNameProxyMCB.remoteFlowId        = 0xFFFFFFFF;

	/* Use the sa_sigaction field because the handles has two additional parameters */
	act.sa_sigaction = &NameProxy_terminated;
	act.sa_flags     = SA_SIGINFO;

    /* Setup the signals. */
	sigaction(SIGTERM, &act, NULL);
    sigaction(SIGKILL, &act, NULL);
    sigaction(SIGINT,  &act, NULL);

    /* Process the command line arguments: */
    if (NameProxy_processCmdLineArgs(argc, argv) < 0)
    {
        NameProxy_displayUsage();
        return -1;
    }

    /* Initialize the database configuration */
    memset ((void *)&databaseCfg, 0, sizeof(Name_DatabaseCfg));

    /* Populate the configuration: */
    databaseCfg.instanceId  = gNameProxyMCB.databaseInstanceId;
    databaseCfg.realm       = Name_ExecutionRealm_ARM;
    strcpy (databaseCfg.owner, gNameProxyMCB.proxyName);

    /* Create the global database */
    gNameProxyMCB.databaseHandle = Name_createDatabase (&databaseCfg, &errCode);
    if (gNameProxyMCB.databaseHandle == NULL)
	{
	    NameProxy_log(NameProxy_LogLevel_ERROR, "Error: Name database creation failed [Error code %d]\n", errCode);
	    return -1;
    }
    NameProxy_log(NameProxy_LogLevel_DEBUG, "Debug: Name database created successfully [Handle %p]\n", gNameProxyMCB.databaseHandle);

    /* Initialize the system configuration. */
    memset ((void *)&sysConfig, 0, sizeof(Resmgr_SystemCfg));

    /* Populate the configuration.*/
    sysConfig.realm                 = Resmgr_ExecutionRealm_ARM;
    strcpy (sysConfig.rmClient, gNameProxyMCB.rmClientName);
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
    gNameProxyMCB.handleSysCfg = Resmgr_init(&sysConfig, &errCode);
    if (gNameProxyMCB.handleSysCfg == NULL)
	{
	    NameProxy_log(NameProxy_LogLevel_ERROR, "Error: SYSRM initialization failed with error code %d\n", errCode);
	    return -1;
    }
    NameProxy_log(NameProxy_LogLevel_DEBUG, "Debug: SYSRM initialized successfully [Handle %p]\n", gNameProxyMCB.handleSysCfg);

    /* Process the configuration: */
    if (Resmgr_processConfig (gNameProxyMCB.handleSysCfg, &appResourceConfig, &errCode) < 0)
	{
	    NameProxy_log(NameProxy_LogLevel_ERROR, "Error: SYSRM configuration failed with error code %d\n", errCode);
	    return -1;
    }

    /* Get the pointer to the application resource configuration */
    ptrResCfg = &appResourceConfig;

    /* Initialize the PKTLIB instance configuration. */
    memset ((void *)&pktlibInstCfg, 0, sizeof(Pktlib_InstCfg));

    /* Populate the PKTLIB Instance configuration: */
    pktlibInstCfg.instanceId        = 1;
    pktlibInstCfg.databaseHandle    = gNameProxyMCB.databaseHandle;
    pktlibInstCfg.sysCfgHandle      = gNameProxyMCB.handleSysCfg;
    pktlibInstCfg.malloc            = Pktlib_osalMalloc;
    pktlibInstCfg.free              = Pktlib_osalFree;
    pktlibInstCfg.beginMemAccess    = Pktlib_osalBeginMemAccess;
    pktlibInstCfg.endMemAccess      = Pktlib_osalEndMemAccess;
    pktlibInstCfg.beginPktAccess    = Pktlib_osalBeginPktAccess;
    pktlibInstCfg.endPktAccess      = Pktlib_osalEndPktAccess;
    pktlibInstCfg.enterCS           = Pktlib_osalEnterCS;
    pktlibInstCfg.exitCS            = Pktlib_osalExitCS;
    pktlibInstCfg.phyToVirt         = Pktlib_osalPhyToVirt;

    /* Create the PKTLIB instance */
    gNameProxyMCB.pktlibInstanceHandle = Pktlib_createInstance (&pktlibInstCfg, &errCode);
    if (gNameProxyMCB.pktlibInstanceHandle == NULL)
    {
        NameProxy_log(NameProxy_LogLevel_ERROR, "Error: Creating the PKTLIB instance failed [Error code %d]\n", errCode);
        return -1;
    }

    /* Initialize the heap configuration */
    memset((void *)&heapCfg, 0 , sizeof(Pktlib_HeapCfg));

    /* Populate the heap configuration */
    snprintf(heapCfg.name, PKTLIB_MAX_CHAR, "%s-NameProxyHeap", gNameProxyMCB.proxyName);
    heapCfg.memRegion                       = ptrResCfg->memRegionResponse[0].memRegionHandle;
    heapCfg.pktlibInstHandle                = gNameProxyMCB.pktlibInstanceHandle;
    heapCfg.sharedHeap                      = 0;
    heapCfg.useStarvationQueue              = 0;
    heapCfg.dataBufferSize                  = 1664;
    heapCfg.numPkts                         = 16;
    heapCfg.numZeroBufferPackets            = 0;
    heapCfg.dataBufferPktThreshold          = 0;
    heapCfg.zeroBufferPktThreshold          = 0;
    heapCfg.heapInterfaceTable.dataMalloc   = myMalloc;
    heapCfg.heapInterfaceTable.dataFree     = myFree;

    /* Create the Name Proxy heap */
    gNameProxyMCB.nameProxyHeapHandle = Pktlib_createHeap(&heapCfg, &errCode);
    if (gNameProxyMCB.nameProxyHeapHandle == NULL)
    {
        NameProxy_log(NameProxy_LogLevel_ERROR, "Error: Unable to create the Name Proxy heap [Error code %d]\n", errCode);
        return -1;
    }
    NameProxy_log(NameProxy_LogLevel_DEBUG, "Debug: Heap %s has been created successfully [Queue 0x%x]\n",
                  heapCfg.name, Pktlib_getInternalHeapQueue(gNameProxyMCB.nameProxyHeapHandle));

    /* Initialize the shared memory */
    if (NameProxy_initSharedMemSetup() < 0)
    {
        NameProxy_log(NameProxy_LogLevel_ERROR, "Error: Shared memory setup failed[Error code %d]\n", errCode);
        return -1;
    }

    /* Initialize the name proxy configuration */
    memset ((void *)&proxyCfg, 0, sizeof(Name_ProxyCfg));

    /* Populate the configuration: */
    strcpy (proxyCfg.proxyName, gNameProxyMCB.proxyName);
    proxyCfg.realm                          = Name_ExecutionRealm_ARM;
    proxyCfg.sharedMemoryAddress            = gNameProxyMCB.sharedMemoryAddress;
    proxyCfg.databaseHandle                 = gNameProxyMCB.databaseHandle;
    proxyCfg.localFlowId      	            = gNameProxyMCB.localFlowId;
    proxyCfg.remoteFlowId     	            = gNameProxyMCB.remoteFlowId;
    proxyCfg.pktlibInstHandle		        = gNameProxyMCB.pktlibInstanceHandle;
    proxyCfg.proxyHeapHandle 	            = gNameProxyMCB.nameProxyHeapHandle;
    proxyCfg.logFxn                         = NameProxy_LogFxn;
    proxyCfg.malloc                         = Name_osalMalloc;
    proxyCfg.free                           = Name_osalFree;
    proxyCfg.beginMemAccess                 = Name_osalBeginMemAccess;
    proxyCfg.endMemAccess                   = Name_osalEndMemAccess;

    /* Create the Name Proxy */
    gNameProxyMCB.nameProxyHandle = Name_initProxy (&proxyCfg, &errCode);
    if (gNameProxyMCB.nameProxyHandle == NULL)
    {
        NameProxy_log(NameProxy_LogLevel_ERROR, "Error: Unable to create the name proxy [Error code %d]\n", errCode);
        return -1;
    }
    NameProxy_log(NameProxy_LogLevel_DEBUG, "Debug: Name proxy %p has been created successfully\n", gNameProxyMCB.nameProxyHandle);

    /* NAME Proxy can now be started */
    gNameProxyMCB.proxyState = 1;

    /* Launch the proxy thread */
	retVal = pthread_create (&proxyThread, NULL, NameProxy_thread, NULL);
	if (retVal < 0)
	{
    	NameProxy_log(NameProxy_LogLevel_ERROR, "Error: Server thread create failed error code %d\n", retVal);
        return -1;
	}

    /* Wait for the thread to terminate. */
    pthread_join (proxyThread, NULL);

    /* Shutdown the server heap */
    if (Pktlib_deleteHeap (gNameProxyMCB.pktlibInstanceHandle, gNameProxyMCB.nameProxyHeapHandle, &errCode) < 0)
    {
        NameProxy_log(NameProxy_LogLevel_ERROR, "Error: Unable to delete the name proxy heap [Error code %d]\n", errCode);
        return -1;
    }

    /* Shutdown the PKTLIB instance */
    if (Pktlib_deleteInstance (gNameProxyMCB.pktlibInstanceHandle, &errCode) < 0)
    {
        NameProxy_log(NameProxy_LogLevel_ERROR, "Error: Unable to delete the PKTLIB instance [Error code %d]\n", errCode);
        return -1;
    }

    /* Cleanup the resource manager configuration */
    if (Resmgr_deinit (gNameProxyMCB.handleSysCfg, &errCode) < 0)
        NameProxy_log(NameProxy_LogLevel_ERROR, "Error: Shutting down the system configuration failed\n");
    else
        NameProxy_log(NameProxy_LogLevel_DEBUG, "Debug: Shutting down the system configuration passed\n");
    return 0;
}

