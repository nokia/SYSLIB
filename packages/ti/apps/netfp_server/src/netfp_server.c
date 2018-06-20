/**
 *   @file  netfp_server.c
 *
 *   @brief
 *      NETFP Server implementation which executes on the ARM as a
 *      daemon and is responsible for interfacing with the NETFP
 *      clients which could be present on the ARM or DSP. The NETFP
 *      server is responsible for the maintaining information which
 *      is shared across all the clients.
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
//fzm---->
#include <ddal/ddal_common.h>
#include <ddal/ddal_cma.h>
#include <syslog.h>
//<----fzm
#include <stdint.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <limits.h>
#include <pthread.h>
#include <signal.h>
#include <getopt.h>

/* MCSDK Include files. */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/runtime/hplib/hplib.h>
#include <ti/drv/sa/sa3gppEnabler/sa3gpp.h>

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
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/netfp/netfp.h>
#include <ti/runtime/uintc/uintc.h>
#include <ti/runtime/name/name.h>
#include <ti/runtime/name/name_db.h>
#include <ti/runtime/name/name_proxyClient.h>
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/apps/netfp_server/include/netfp_server.h>

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

/* MSGCOM: */
extern void*   Msgcom_osalMalloc(Msgcom_MemAllocMode , uint32_t );
extern void    Msgcom_osalFree(Msgcom_MemAllocMode , void* , uint32_t );
extern int32_t Msgcom_osalRegisterIsr(const char* , Qmss_Queue , MsgCom_Isr , MsgCom_ChHandle, MsgCom_Interrupt*);
extern int32_t Msgcom_osalDeregisterIsr(const char* , Qmss_Queue ,MsgCom_Interrupt*);
extern void    Msgcom_osalDisableSysInt(int32_t , int32_t );
extern void    Msgcom_osalEnableSysInt(int32_t , int32_t );
extern void*   Msgcom_osalEnterSingleCoreCS(void);
extern void    Msgcom_osalExitSingleCoreCS(void* );
extern void*   Msgcom_osalCreateSem(void);
extern void    Msgcom_osalDeleteSem(void* );
extern void    Msgcom_osalPendSem(void* );
extern void    Msgcom_osalPostSem(void* );

/* NETFP: */
extern void* Netfp_osalMalloc (uint32_t , uint32_t );
extern void  Netfp_osalFree (void* , uint32_t );
extern void* Netfp_osalMallocSecurityContext (Netfp_SaProtocol , uint32_t , uint32_t );
extern void  Netfp_osalFreeSecurityContext (Netfp_SaProtocol , void* , uint32_t );
extern void* Netfp_osalEnterSingleCoreCriticalSection (void);
extern void  Netfp_osalExitSingleCoreCriticalSection (void* csHandle);
extern void  Netfp_osalBeginMemoryAccess (void* ptr, uint32_t size);
extern void  Netfp_osalEndMemoryAccess (void* ptr, uint32_t size);
extern void* Netfp_osalCreateSem(void);
extern void  Netfp_osalDeleteSem(void*);
extern void  Netfp_osalPostSem(void*);
extern void  Netfp_osalPendSem(void*);
// <fzm>
extern void  Netfp_LoggingFxn (Netfp_LogLevel logLevel, const char* fmt, va_list arg);
extern int32_t Server_logInit (void);
extern int32_t Server_logDeInit (void);
extern void Netfp_DumpingFxn (Netfp_LogLevel logLevel, const char* fmt, va_list arg);
extern int NetfpServer_dumpInit(void);
extern void NetfpServer_dumpDeInit(void);
// </fzm>

/**********************************************************************
 *********************** Local Declarations ***************************
 **********************************************************************/

/* NETFP Server Timeout in seconds */
#define NETFP_SERVER_TIMEOUT        5

/**********************************************************************
 *********************** Global Declarations **************************
 **********************************************************************/
//fzm-->
/* Overriding NetfpServer_log () with new callback nfunction */
# define NetfpServer_LogLevel_DEBUG Netfp_LogLevel_DEBUG
# define NetfpServer_LogLevel_INFO Netfp_LogLevel_INFO
# define NetfpServer_LogLevel_ERROR Netfp_LogLevel_ERROR

/* Netfp Server Logging FD */
extern FILE*                       ServerlogFd;
//fzm<--

/* NETFP Server MCB */
NetfpServer_MCB         gNetfpServerMCB;

/* Application requested resources */
Resmgr_ResourceCfg      appResourceConfig =
{
    /* Requested Memory Region Configuration. */
    .memRegionCfg =
    {
        /* Name,                Type,                       Linking RAM,                           Num,        Size */
        { "NETFP-ServerRegion", Resmgr_MemRegionType_DDR3,  Resmgr_MemRegionLinkingRAM_INTERNAL,  512,        128}, //fzm
    },
};

/**********************************************************************
 *********************** NETFP Server Functions ***********************
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
//fzm---->
    (void)arg;

    uint8_t* ptr = NULL;
    if(ddal_cma_alloc(DDAL_CMA_MEM_TYPE_DDR, size, (void *)&ptr) != DDAL_OK)
    {
        NetfpServer_log(NetfpServer_LogLevel_ERROR, "Error: Unable to allocate CMA memory\n");
        return NULL;
    }
    return ptr;
//<----fzm
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
    return;
}

/**
 *  @b Description
 *  @n
 *     Display the usage for the resource manager server
 *
 *  @retval
 *      Not Applicable.
 */
static void NetfpServer_displayUsage (void)
{
    printf ("NETFP Server:\n");
    printf ("netfp_server <arguments>\n");
    printf ("Mandatory Arguments: \n");
    printf ("-n <name>           - Name of the NETFP server; unique in the system\n");
    printf ("-r <name>           - Name of the RM Client to be used; this should match the client names in the RMv2 Policy file\n");
    printf ("-c <netfp.conf>     - NETFP Server configuration file.\n");
    printf ("-i <InstantId>      - Named resource instant identifier\n");
    printf ("Optional Arguments: \n");
    printf ("-t <timeout>      - Polling Timeout specified in msec [Default is 100msec]\n");
    printf ("-l                - Log the output to a file [File Name: /var/log/netfp_server_<pid>.log]\n");
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
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
static int32_t NetfpServer_processCmdLineArgs(int32_t argc, char* argv[])
{
    while (1)
    {
        int option_index = 0;
        int c;

        static struct option long_options[] = {
            {"name",            required_argument, 0,  0 },
            {"rmClientName",    required_argument, 0,  0 },
            {"timeout",         required_argument, 0,  0 },
            {"clientList",      required_argument, 0,  0 },
            {"instantId",       required_argument, 0,  0 },
            {0,                 0,                 0,  0 }
        };

        c = getopt_long(argc, argv, "lv:n:r:t:c:p:i:", long_options, &option_index); //fzm
        if (c == -1)
            break;

       switch (c)
       {
            case 'n':
            {
                /* Server Name: */
                strncpy (gNetfpServerMCB.serverName, optarg, sizeof(gNetfpServerMCB.serverName));
                break;
            }
            case 'l':
            {
                char    netfpServerLogFile[PATH_MAX];

                /* Generate the NETFP Server log filename */
                snprintf (netfpServerLogFile, PATH_MAX, "/var/log/netfp_server_%d.log", getpid());

                /* Log the server console output to a file: */
                gNetfpServerMCB.loggingFile = fopen (netfpServerLogFile, "w");
                if (gNetfpServerMCB.loggingFile == NULL)
                    return -1;

                break;
            }
            case 'r':
            {
                /* RM Client Name: */
                strncpy (gNetfpServerMCB.rmClientName, optarg, sizeof(gNetfpServerMCB.rmClientName));
                break;
            }
            case 'c':
            {
                /* NETFP Client List File name. */
                gNetfpServerMCB.clientListFile = optarg;
                break;
            }
            case 'i':
            {
                /* Named resource instance identifier. */
                gNetfpServerMCB.nrInstanceId = atoi (optarg);
                break;
            }
            case 't':
            {
                /* Polling Timeout: This is specified in micro-seconds */
                gNetfpServerMCB.pollingTimeout = atoi (optarg);
                gNetfpServerMCB.pollingTimeout = gNetfpServerMCB.pollingTimeout * 1000;
                break;
            }
            case 'v':
            {
//fzm-->
                if (!strcmp("VRB", optarg))
                {
                   gNetfpServerMCB.logLevel = NetfpServer_LogLevel_VERBOSE;
                }
                else if (!strcmp("DBG", optarg))
                {
                   gNetfpServerMCB.logLevel = NetfpServer_LogLevel_DEBUG;
                }
                else if (!strcmp("INF", optarg))
                {
                   gNetfpServerMCB.logLevel = NetfpServer_LogLevel_INFO;
                }
                else if (!strcmp("ERR", optarg))
                {
                   gNetfpServerMCB.logLevel = NetfpServer_LogLevel_ERROR;
                }
                else
                {
                   NetfpServer_log(NetfpServer_LogLevel_ERROR, "Error: wrong logging level %s, supported VRB|DBG|INF|ERR\n", optarg);
                }
//fzm<--
                break;
            }
            case '?':
            {
                return -1;
            }
        }
    }

    /* We should have a valid agent server name */
    if (gNetfpServerMCB.serverName[0] == 0)
        return -1;

    /* We should have a valid RM client name */
    if (gNetfpServerMCB.rmClientName[0] == 0)
        return -1;

    /* We should have a valid client list. */
    if (gNetfpServerMCB.clientListFile == NULL)
        return -1;

    /* Was the logging file specified; if not redirect all the output to the standard console */
    if (gNetfpServerMCB.loggingFile == NULL)
        gNetfpServerMCB.loggingFile = stdout;

    /* Command Line Arguments processed successfully. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Logging Function used by the MSGRouter Functions.
 *
 *  @param[in]  level
 *      Log level
 *  @param[in]  fmt
 *      Formatting string
 *
 *  @retval
 *      Not Applicable.
 */
void NetfpServer_log (NetfpServer_LogLevel level, const char* fmt, ...)
{
    va_list arg;

    /* Log the message as per the configured level. */
    if (level >= gNetfpServerMCB.logLevel)
    {
        va_start (arg, fmt);
        /* fzm - Overriding call to this function by calling registered callback directly from here */
        Netfp_LoggingFxn (level, fmt, arg); //fzm
        va_end (arg);
    }
    return;
}

// <fzm>
void NetfpServer_dump (NetfpServer_LogLevel level, const char* fmt, ...)
{
    va_list arg;

    va_start(arg, fmt);
    /* Overriding call to this function by calling registered callback directly from here */
    Netfp_DumpingFxn(level, fmt, arg);
    va_end(arg);

    return;
}
// </fzm>

#if 0 //fzm
/**
 *  @b Description
 *  @n
 *      Logging function which is registered with the NETFP Server library
 *
 *  @param[in]  logLevel
 *      Agent Log Level
 *  @param[in]  fmt
 *      Formatted string
 *  @param[in]  arg
 *      Variable length argument list
 *
 *  @retval
 *      Not applicable
 */
static void NetfpServer_LogFxn (Netfp_LogLevel logLevel, const char* fmt, va_list arg)
{
    /* If the NETFP library has reported an error or informational message; we always log this */
    if ((logLevel == Netfp_LogLevel_ERROR) || (logLevel == Netfp_LogLevel_INFO))
        vfprintf (gNetfpServerMCB.loggingFile, fmt, arg);

    /* If the NETFP library has reported a debug message and the server is executing
     * with the verbosity on. Log the message. */
    if ((logLevel == Netfp_LogLevel_DEBUG) && (gNetfpServerMCB.logLevel == NetfpServer_LogLevel_DEBUG))
        vfprintf (gNetfpServerMCB.loggingFile, fmt, arg);

    /* Flush the output stream */
    if (fflush(gNetfpServerMCB.loggingFile) < 0)
        printf ("Error: Flushing the stream failed: %s\n", strerror(errno));
}
#endif //fzm

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
static void NetfpServer_terminated (int sig, siginfo_t *siginfo, void *context)
{
    /* Server is going down. */
    NetfpServer_log(NetfpServer_LogLevel_DEBUG, "Debug: Signal received\n");
    gNetfpServerMCB.serverState = 0;
    return;
}

/**
 *  @b Description
 *  @n
 *      Registered UINTC message identifier to display the NETFP Server debug
 *      information
 *
 *  @param[in]  arg
 *      Registered argument
 *
 *  @retval
 *      Not Applicable.
 */
static void NetfpServer_displayServerInfo (void* arg)
{
    //fzm
    if (NetfpServer_dumpInit() == -1)
        return;

    /* Display server general information */
    Netfp_displayServerGenInfo(gNetfpServerMCB.netfpServerHandle);

    /* Display all the security contexts configured on the server: */
    Netfp_displaySecurityContext(gNetfpServerMCB.netfpServerHandle);

    /* Display all the security associations configured on the server: */
    Netfp_displaySA(gNetfpServerMCB.netfpServerHandle,       Netfp_Direction_INBOUND);
    Netfp_displaySA(gNetfpServerMCB.netfpServerHandle,       Netfp_Direction_OUTBOUND);

    /* Display all the security policies offloaded on the server: */
    Netfp_displaySP(gNetfpServerMCB.netfpServerHandle,       Netfp_Direction_INBOUND);
    Netfp_displaySP(gNetfpServerMCB.netfpServerHandle,       Netfp_Direction_OUTBOUND);

    /* Display all the fast paths created on the server: */
    Netfp_displayInboundFastPath(gNetfpServerMCB.netfpServerHandle);
    Netfp_displayOutboundFastPath(gNetfpServerMCB.netfpServerHandle);

    /* Display the L4 Bindings: */
    Netfp_displayL4Binding(gNetfpServerMCB.netfpServerHandle);

    /* Display the Proxy-Server Interface */
    Netfp_displayProxyServerInterface(gNetfpServerMCB.netfpServerHandle);

    /* Display all the LUT entries added on the server: */
    Netfp_displayLutInfoList(gNetfpServerMCB.netfpServerHandle);

    /* Display the interface list on the server: */
    Netfp_displayInterface(gNetfpServerMCB.netfpServerHandle);

    /* Display the NETFP client status: */
    Netfp_displayClient(gNetfpServerMCB.netfpServerHandle);

    /* fzm This string is appended at the end of the log to enable collection with FPControl app */
    NetfpServer_dump (NetfpServer_LogLevel_INFO, "**************************************</netfp_server_dump>\n");
    NetfpServer_dumpDeInit();
}

/**
 *  @b Description
 *  @n
 *      Signal Handler installed to catch the SIGUSR1
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
static void NetfpServer_sigHandler (int sig, siginfo_t *siginfo, void *context)
{
    int32_t errCode;

    /* Send off the message to the UINTC module: */
    if (Uintc_sendMessageId (gNetfpServerMCB.uintcHandle, 0xabcd, &errCode) < 0)
    {
        NetfpServer_log(NetfpServer_LogLevel_ERROR, "Error: Unable to send message to the UINTC module [Error code %d]\n", errCode); //fzm
        return;
    }
}

/**
 *  @b Description
 *  @n
 *      The function is used to parse the client list file and generate a list of
 *      all the NETFP client names which could attach to the NETFP Server. The
 *      names of all the NETFP clients are placed into the client database
 *
 *  @retval
 *      Success -   Number of NETFP clients
 *  @retval
 *      Error   -   <0
 */
//fzm changed as part of BTSFZM-1666: Switch from hplibmod to cmamod for heap allocations
static int32_t NetfpServer_parseClientList (void)
{
    FILE*       fp;
    int32_t     netfpClientCount = 0;
    int32_t     size;
    const char* delimitters = "=,;\r\n";
    char*       line = NULL;
    size_t     len = 0;

    /* Open the NETFP configuration file for the tests */
    fp = fopen (gNetfpServerMCB.clientListFile, "r");
    if (fp == NULL)
    {
        NetfpServer_log(NetfpServer_LogLevel_ERROR, "Error: Unable to open client list '%s'\n", gNetfpServerMCB.clientListFile);
        return -1;
    }

    while (getline(&line, &len, fp) != -1)
    {
        char *tokenName = strtok(line, delimitters);
        char *tokenValue = strtok(NULL, delimitters);

        if(tokenName == NULL || tokenValue == NULL)
            continue;

        if ((*tokenName == '#') || (*tokenName == '\n') || (*tokenName == '\r'))
            continue;

        if (strcmp (tokenName, "CLIENT") == 0)
        {
            /* Copy the client name into the NETFP Client database. */
            strcpy (gNetfpServerMCB.netfpClientList[netfpClientCount], tokenValue);
            netfpClientCount++;
        }
        else if (strcmp (tokenName, "NUM_SECURITY_CHANNELS") == 0)
        {
            /* Convert the number of security channels to integer format. */
            gNetfpServerMCB.numSecurityChannels = atoi (tokenValue);

            /* Sanity Check: Number of security channels is limited to 16 bits */
            if (gNetfpServerMCB.numSecurityChannels > 64*1024)
            {
                NetfpServer_log(NetfpServer_LogLevel_ERROR, "Error: Invalid number of security channels specified %d\n",
                                gNetfpServerMCB.numSecurityChannels);
                free(line);
                fclose(fp);
                return -1;
            }
        }
        else if (strcmp (tokenName, "BASE_SECURITY_CONTEXT") == 0)
        {
            /* Convert the base security context identifier to integer format. */
            gNetfpServerMCB.baseSecurityContextId = atoi (tokenValue);
        }
        else if (strcmp (tokenName, "IP_USER_STAT_COUNTER_SUPPORT") == 0)
        {
            /* Convert the IP user statistics counter support */
            gNetfpServerMCB.enableIPLutEntryCount = atoi (tokenValue);
        }
        else
        {
            /* Error: Invalid Token */
            NetfpServer_log(NetfpServer_LogLevel_ERROR, "Error: Invalid token detected '%s'\n", tokenName);
            free(line);
            fclose(fp);
            return -1;
        }
    }
    fclose(fp);

    //free the buffer allocated in getline
    free(line);

    /* Display the parsed information: */
    for (size = 0; size < netfpClientCount; size++)
        NetfpServer_log(NetfpServer_LogLevel_DEBUG, "Debug: Client Name %d = '%s'\n", size, gNetfpServerMCB.netfpClientList[size]);
    NetfpServer_log(NetfpServer_LogLevel_DEBUG, "Debug: Base Security Context Id: %d Security Channels %d\n",
                    gNetfpServerMCB.baseSecurityContextId, gNetfpServerMCB.numSecurityChannels);

    return netfpClientCount;
}

/**
 *  @b Description
 *  @n
 *      This is the NETFP Server thread which handles all the NETFP API which are
 *      executed from the registered NETFP client(s)
 *
 *  @param[in]  arg
 *      Argument passed to the thread.
 *
 *  @retval
 *      Not Applicable.
 */
static void* NetfpServer_thread(void *arg)
{
    int32_t             errCode;
    int32_t             retVal;
    int32_t             numClientsServiced;
    struct sched_param  param;
    struct timeval      timeout;

    /* Set the configured policy and priority */
    param.sched_priority = 0; //fzm
    errCode = pthread_setschedparam(pthread_self(), SCHED_OTHER, &param);
    if (errCode != 0)
    {
        NetfpServer_log(NetfpServer_LogLevel_ERROR, "Error: Unable to set the thread priority & policy [Error Code %d]\n", errCode);
        return NULL;
    }

    if(nice(NETFP_SERVER_NICENESS) == -1)
    {
        NetfpServer_log(NetfpServer_LogLevel_ERROR, "Error: Unable to set the thread nicess [Error Code %d]\n", errno);
        return NULL;
    }

    /* Debug Message: */
    NetfpServer_log(NetfpServer_LogLevel_DEBUG, "Debug: NETFP Server UINTC execution thread started\n");

    /* Initialize the timeout for the NETFP Server */
    timeout.tv_sec  = NETFP_SERVER_TIMEOUT;
    timeout.tv_usec = 0;

    /* Execute the NETFP Server thread. The thread is terminated by the UINTC module; else it executes forever. */
    while (1)
    {
        /* Wait for an interrupt to be received. */
        retVal = Uintc_select (gNetfpServerMCB.uintcHandle, &timeout, &errCode);
        if (retVal < 0)
        {
            /* Error: UINTC select failed. Has the UINTC module been deinitialized? */
            if (errCode == UINTC_EDEINIT)
                break;

            /* Report the UINTC Module error: */
            NetfpServer_log(NetfpServer_LogLevel_ERROR, "Error: UINTC select failed [Error code %d]\n", errCode);
            return NULL;
        }

        /* Was there a timeout? */
        if (retVal == 0)
        {
            /* YES: Execute the timeout handler. */
            Netfp_executeServerTimeout (gNetfpServerMCB.netfpServerHandle, NETFP_SERVER_TIMEOUT, &errCode);

            /* Initialize the timeout for the NETFP Server */
            timeout.tv_sec  = NETFP_SERVER_TIMEOUT;
            timeout.tv_usec = 0;
            continue;
        }

        /* Control comes here implies that the UINTC thread was woken up because one of the interrupts
         * was fired. This implies that the server has received a message from one of the NETFP clients
         * So here we execute the NETFP server until all the NETFP clients have been processed. */
        while (1)
        {
            numClientsServiced = Netfp_executeServer (gNetfpServerMCB.netfpServerHandle);
            if (numClientsServiced == 0)
                break;
        }
    }
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      This is the NETFP management thread which registers NETFP clients. The thread
 *      is executed periodically every polling tick.
 *
 *  @param[in]  arg
 *      Argument passed to the thread.
 *
 *  @retval
 *      Not Applicable.
 */
static void* NetfpServer_mgmtThread(void *arg)
{
    pthread_t                   netfpThread;
    Msgcom_DirectInterruptCfg   directInterruptCfg;
    int32_t                     errCode;
    int32_t                     count;
    int32_t                     retVal;
//fzm --->
    uint32_t                    deregisterCount = 0;
//<----fzm

    /* Debug Message: */
    NetfpServer_log(NetfpServer_LogLevel_DEBUG, "Debug: NETFP server client management thread started\n");

    /* Create the NETFP Server thread. */
    errCode = pthread_create (&netfpThread, NULL, NetfpServer_thread, NULL);
    if (errCode < 0)
    {
        NetfpServer_log(NetfpServer_LogLevel_ERROR, "Error: NETFP Server thread create failed error code %d\n", errCode);
        return NULL;
    }

    /* Execute the NETFP Server: Until a termination signal is detected. */
    while (gNetfpServerMCB.serverState == 1)
    {
        /* Cycle through all the clients and register them if possible */
        for (count = 0; count < gNetfpServerMCB.netfpClientCount; count++)
        {
            /* Has the client already been deregistered? */
            if (gNetfpServerMCB.netfpClientStatus[count] == 2)
                continue;

            /* Has the client already been registered? */
            if (gNetfpServerMCB.netfpClientStatus[count] == 1)
            {
                /* YES. Client has already been registered. So here we check and make sure that the client is still active? */
                retVal = Netfp_isClientActive(gNetfpServerMCB.netfpServerHandle, gNetfpServerMCB.netfpClientList[count], &errCode);
                if (retVal < 0)
                {
                    /* FATAL Error: */
                    NetfpServer_log(NetfpServer_LogLevel_ERROR, "Error: NETFP Server Client %s Active Status failed %d\n",
                                    gNetfpServerMCB.netfpClientList[count], errCode);
                    return NULL;
                }
                if (retVal == 0)
                {
                    /* Client is no longer active and needs to be deregistered from the NETFP Server MCB also */
                    if (Netfp_deregisterClient (gNetfpServerMCB.netfpServerHandle, gNetfpServerMCB.netfpClientList[count], &errCode) == 0)
                    {
                        /* NETFP client has been deregistered successfully. Once the client is deregistered we are
                         * not allowing registeration again */
                        gNetfpServerMCB.netfpClientStatus[count] = 2;
                        NetfpServer_log(NetfpServer_LogLevel_DEBUG, "Debug: NETFP Client %s has been deregistered\n",
                                        gNetfpServerMCB.netfpClientList[count]);
                        continue;
                    }
                    else
                    {
                        NetfpServer_log(NetfpServer_LogLevel_ERROR, "Error: NETFP deregister client failed [Error code %d]\n", errCode);
                    }
                }
                else
                {
                    /* NETFP client is still active; so there is nothing else which needs to be done. */
                }
                continue;
            }

            /* NO. NETFP Client has not been registered. So populate the direct interrupt configuration and try and register
             * the NETFP client. */
            directInterruptCfg.queuePendQueue  = appResourceConfig.qPendResponse[count].queue;
            directInterruptCfg.cpIntcId        = appResourceConfig.qPendResponse[count].cpIntcId;
            directInterruptCfg.systemInterrupt = appResourceConfig.qPendResponse[count].systemInterrupt;
            directInterruptCfg.hostInterrupt   = appResourceConfig.qPendResponse[count].hostInterrupt;

            /* Try and register the client.  */
            int sockFd = -1;
            if ((sockFd = Netfp_registerClient (gNetfpServerMCB.netfpServerHandle, gNetfpServerMCB.netfpClientList[count],
                                      &directInterruptCfg, &errCode)) >= 0)
            {
                /* NETFP client registeration was successful. */
                if(Uintc_registerAdditionalFd(gNetfpServerMCB.uintcHandle, sockFd, &errCode) != sockFd)
                {
                    NetfpServer_log(NetfpServer_LogLevel_ERROR, "Error: Failed to register Client '%s' sockFd: %d in UINTC [%d]\n", gNetfpServerMCB.netfpClientList[count], sockFd, errCode);
                    return NULL;
                }
                gNetfpServerMCB.netfpClientStatus[count] = 1;
                NetfpServer_log(NetfpServer_LogLevel_DEBUG, "Debug: NETFP Client '%s' registered, its sockFd: %d\n", gNetfpServerMCB.netfpClientList[count], sockFd);
            }
            else
            {
                /* Registration failed; use the error code to determine the reason. */
                if (errCode != NETFP_ENOTREADY)
                {
                    NetfpServer_log(NetfpServer_LogLevel_ERROR, "Error: NETFP Register client failed [Error code %d]\n", errCode);
                }
            }
        }
        usleep (gNetfpServerMCB.pollingTimeout);
    }

    /* Debug Mesage: */
    NetfpServer_log(NetfpServer_LogLevel_DEBUG, "Debug: NETFP server client management thread shutting down\n");

    /* Delete the NETFP server: The server has been sent a signal to shutdown its services. */
    while (1)
    {
        /* Delete the NETFP server. */
        if (Netfp_deleteServer (gNetfpServerMCB.netfpServerHandle, &errCode) == 0)
            break;

        /* The NETFP Server deletion can fail if the NETFP clients have not been deleted */
        if (errCode == NETFP_ENOTREADY)
        {
//fzm --->
            deregisterCount++;
            if (deregisterCount >= 50000)
            {
                 NetfpServer_log(NetfpServer_LogLevel_INFO, "NETFP clients not deregistered after 5s, exiting");
                 syslog(LOG_WARNING, "NETFP clients not deregistered after 5s, exiting");
                 break;
            }
//<----fzm
            /* The NETFP clients are still active and have not been deleted; sleep and try again. */
            usleep(100);
            continue;
        }
        NetfpServer_log(NetfpServer_LogLevel_ERROR, "Error: Unable to delete the NETFP server [Error code %d]\n", errCode);
        return NULL;
    }

    /* Shutdown the UINTC module: This will initiate the shutdown of the NETFP Server thread. */
    if (Uintc_deinit (gNetfpServerMCB.uintcHandle, &errCode) < 0)
        NetfpServer_log(NetfpServer_LogLevel_ERROR, "Error: UINTC module deinitialization failed [Error code %d]\n", errCode);

    /* Wait for the NETFP Server thread to exit */
    pthread_join (netfpThread, NULL);
    Server_logDeInit (); //fzm

    NetfpServer_log(NetfpServer_LogLevel_DEBUG, "Debug: NETFP server has been deleted successfully. \n");
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      Entry point into the NETFP server.
 *
 *  @param[in]  argc
 *      Number of arguments.
 *  @param[in]  argv
 *      Command Line arguments.
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t main(int argc, char* argv[])
{
    Resmgr_ResourceCfg*         ptrResCfg;
    int32_t                     errCode;
    Pktlib_HeapCfg              heapCfg;
    Netfp_ServerConfig          serverConfig;
    Resmgr_SystemCfg            sysConfig;
    struct sigaction            act;
    int32_t                     retVal;
    UintcConfig                 uintcConfig;
    pthread_t                   serverMgmtThread;
    pthread_t                   masterMgmtInterfaceThread;
    Name_DatabaseCfg            databaseCfg;
    Pktlib_InstCfg              pktlibInstCfg;
    Msgcom_InstCfg              msgcomInstCfg;

    /* Initialize the NETFP server MCB */
    memset ((void *)&gNetfpServerMCB, 0, sizeof(NetfpServer_MCB));

    /* Initialize the logger */
    if (Server_logInit() < 0) //fzm
        return -1;

    /* Setup the defaults.
     *  - Polling Timeout is 100msec. */
    gNetfpServerMCB.logLevel            = NetfpServer_LogLevel_ERROR;
    gNetfpServerMCB.pollingTimeout      = 100*1000;

    /* Use the sa_sigaction field because the handles has two additional parameters */
    act.sa_sigaction = &NetfpServer_terminated;
    act.sa_flags     = SA_SIGINFO;

    /* Setup the signals. */
    sigaction(SIGTERM, &act, NULL);
    sigaction(SIGKILL, &act, NULL);
    sigaction(SIGINT,  &act, NULL);

    /* Setup the user defined signal to display the resource usage. */
    act.sa_sigaction = &NetfpServer_sigHandler;
    act.sa_flags     = SA_SIGINFO;
    sigaction(SIGUSR1, &act, NULL);

    /* Process the command line arguments: */
    if (NetfpServer_processCmdLineArgs(argc, argv) < 0)
    {
        NetfpServer_displayUsage();
        return -1;
    }

    //fzm --->
    //initialize the ddal cma subsystem (mmaps /dev/cma into our address space)
    int ddalCmaInitStatus = ddal_cma_process_init();
    if(ddalCmaInitStatus != DDAL_OK)
    {
        NetfpServer_log(NetfpServer_LogLevel_ERROR, "Failed to initialize DDAL CMA, errcode: %d", ddalCmaInitStatus);
        return -1;
    }
    //<---- fzm

    /* Initialize the database configuration */
    memset ((void *)&databaseCfg, 0, sizeof(Name_DatabaseCfg));

    /* Populate the configuration: */
    databaseCfg.instanceId  = gNetfpServerMCB.nrInstanceId;
    databaseCfg.realm       = Name_ExecutionRealm_ARM;
    strcpy (databaseCfg.owner, gNetfpServerMCB.serverName);

    /* Create the NETFP Server database handle */
    gNetfpServerMCB.databaseHandle = Name_createDatabase (&databaseCfg, &errCode);
    if (gNetfpServerMCB.databaseHandle == NULL)
    {
        NetfpServer_log(NetfpServer_LogLevel_ERROR, "Error: Name database creation failed [Error code %d]\n", errCode);
        return -1;
    }
    NetfpServer_log(NetfpServer_LogLevel_DEBUG, "Debug: Name database created successfully [Handle %p]\n", gNetfpServerMCB.databaseHandle);

    /* Parse the NETFP Client List */
    gNetfpServerMCB.netfpClientCount = NetfpServer_parseClientList();
    if (gNetfpServerMCB.netfpClientCount <= 0)
    {
        NetfpServer_log(NetfpServer_LogLevel_ERROR, "Error: Parsing client list yielded %d clients; aborting\n",
                        gNetfpServerMCB.netfpClientCount);
        return -1;
    }

    /* NETFP Server & clients communicate using MSGCOM queue pend channels. Once we have determined the NETFP clients
     * which are allowed as per the NETFP client configuration file we request the queue pend resources. */
    appResourceConfig.numQpendQueues = 0; //fzm

    /* Initialize the system configuration. */
    memset ((void *)&sysConfig, 0, sizeof(Resmgr_SystemCfg));

    /* Populate the configuration.*/
    sysConfig.realm                 = Resmgr_ExecutionRealm_ARM;
    strcpy (sysConfig.rmClient, gNetfpServerMCB.rmClientName);
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
    gNetfpServerMCB.handleSysCfg = Resmgr_init(&sysConfig, &errCode);
    if (gNetfpServerMCB.handleSysCfg == NULL)
    {
        NetfpServer_log(NetfpServer_LogLevel_ERROR, "Error: System configuration initialization failed [Error code %d]\n", errCode);
        return -1;
    }
    NetfpServer_log(NetfpServer_LogLevel_DEBUG, "Debug: NETFP Server system configuration has been processed.\n");

    /* Process the configuration */
    if (Resmgr_processConfig (gNetfpServerMCB.handleSysCfg, &appResourceConfig, &errCode) < 0)
    {
        NetfpServer_log(NetfpServer_LogLevel_ERROR, "Error: SYSRM configuration failed with error code %d\n", errCode);
        return -1;
    }

    /* Get the pointer to the application resource configuration */
    ptrResCfg = &appResourceConfig;

    /* Initialize the PKTLIB instance configuration. */
    memset ((void *)&pktlibInstCfg, 0, sizeof(Pktlib_InstCfg));

    /* Populate the PKTLIB Instance configuration: */
    pktlibInstCfg.instanceId        = 1;
    pktlibInstCfg.databaseHandle    = gNetfpServerMCB.databaseHandle;
    pktlibInstCfg.sysCfgHandle      = gNetfpServerMCB.handleSysCfg;
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
    gNetfpServerMCB.pktlibInstanceHandle = Pktlib_createInstance (&pktlibInstCfg, &errCode);
    if (gNetfpServerMCB.pktlibInstanceHandle == NULL)
    {
        NetfpServer_log(NetfpServer_LogLevel_ERROR, "Error: Creating the PKTLIB instance failed [Error code %d]\n", errCode);
        return -1;
    }

    /* Initialize the heap configuration */
    memset((void *)&heapCfg, 0 , sizeof(Pktlib_HeapCfg));

    /* Populate the heap configuration */
    snprintf(heapCfg.name, PKTLIB_MAX_CHAR, "%s-PACmdHeap", gNetfpServerMCB.serverName);
    heapCfg.memRegion                       = ptrResCfg->memRegionResponse[0].memRegionHandle;
    heapCfg.pktlibInstHandle                = gNetfpServerMCB.pktlibInstanceHandle;
    heapCfg.sharedHeap                      = 0;
    heapCfg.useStarvationQueue              = 0;
    heapCfg.dataBufferSize                  = pa_MAX_CMD_BUF_SIZE_BYTES;
    heapCfg.numPkts                         = 32;
    heapCfg.numZeroBufferPackets            = 0;
    heapCfg.dataBufferPktThreshold          = 0;
    heapCfg.zeroBufferPktThreshold          = 0;
    heapCfg.heapInterfaceTable.dataMalloc   = myMalloc;
    heapCfg.heapInterfaceTable.dataFree     = myFree;
    heapCfg.linearAlloc = 1;

    /* Create the PA command heap */
    gNetfpServerMCB.paCommandHeapHandle = Pktlib_createHeap(&heapCfg, &errCode);
    if (gNetfpServerMCB.paCommandHeapHandle == NULL)
    {
        NetfpServer_log(NetfpServer_LogLevel_ERROR, "Error: Unable to create the PA command heap [Error code %d]\n", errCode);
        return -1;
    }
    NetfpServer_log(NetfpServer_LogLevel_DEBUG, "Debug: Heap %s has been created successfully\n", heapCfg.name);

    /* Initialize the heap configuration. */
    memset ((void *)&heapCfg, 0, sizeof(Pktlib_HeapCfg));

    /* Populate the heap configuration */
    snprintf(heapCfg.name, PKTLIB_MAX_CHAR, "%s-PASAHeap", gNetfpServerMCB.serverName);
    heapCfg.memRegion                       = ptrResCfg->memRegionResponse[0].memRegionHandle;
    heapCfg.pktlibInstHandle                = gNetfpServerMCB.pktlibInstanceHandle;
    heapCfg.sharedHeap                      = 0;
    heapCfg.useStarvationQueue              = 1;
    heapCfg.dataBufferSize                  = 1664;
    heapCfg.numPkts                         = 128;
    heapCfg.numZeroBufferPackets            = 0;
    heapCfg.dataBufferPktThreshold          = 0;
    heapCfg.zeroBufferPktThreshold          = 0;
    heapCfg.heapInterfaceTable.dataMalloc   = myMalloc;
    heapCfg.heapInterfaceTable.dataFree     = myFree;
    heapCfg.linearAlloc = 1;

    /* Create the heap for the IPSEC Traffic */
    gNetfpServerMCB.ipSecHeapHandle = Pktlib_createHeap(&heapCfg, &errCode);
    if (gNetfpServerMCB.ipSecHeapHandle == NULL)
    {
        NetfpServer_log(NetfpServer_LogLevel_ERROR, "Error: Unable to create the PASA heap [Error code %d]\n", errCode);
        return -1;
    }
    NetfpServer_log(NetfpServer_LogLevel_DEBUG, "Debug: Heap %s has been created successfully\n", heapCfg.name);

    /* Initialize the heap configuration. */
    memset ((void *)&heapCfg, 0, sizeof(Pktlib_HeapCfg));

    /* Populate the heap configuration */
    snprintf(heapCfg.name, PKTLIB_MAX_CHAR, "%s-SrvHeap", gNetfpServerMCB.serverName);
    heapCfg.memRegion                       = ptrResCfg->memRegionResponse[0].memRegionHandle;
    heapCfg.pktlibInstHandle                = gNetfpServerMCB.pktlibInstanceHandle;
    heapCfg.sharedHeap                      = 0;
    heapCfg.useStarvationQueue              = 0;
    heapCfg.dataBufferSize                  = 4096;
    heapCfg.numPkts                         = 256;
    heapCfg.numZeroBufferPackets            = 0;
    heapCfg.dataBufferPktThreshold          = 0;
    heapCfg.zeroBufferPktThreshold          = 0;
    heapCfg.heapInterfaceTable.dataMalloc   = myMalloc;
    heapCfg.heapInterfaceTable.dataFree     = myFree;
    heapCfg.linearAlloc = 1;

    /* Create the PASA heap */
    gNetfpServerMCB.serverHeapHandle = Pktlib_createHeap(&heapCfg, &errCode);
    if (gNetfpServerMCB.serverHeapHandle == NULL)
    {
        NetfpServer_log(NetfpServer_LogLevel_ERROR, "Error: Unable to create the PASA heap [Error code %d]\n", errCode);
        return -1;
    }
    NetfpServer_log(NetfpServer_LogLevel_DEBUG, "Debug: Heap %s has been created successfully\n", heapCfg.name);

    /* Initialize the MSGCOM instance. */
    memset ((void *)&msgcomInstCfg, 0, sizeof(Msgcom_InstCfg));

    /* Populate the MSGCOM Instance configuration */
    msgcomInstCfg.databaseHandle    = gNetfpServerMCB.databaseHandle;
    msgcomInstCfg.sysCfgHandle      = gNetfpServerMCB.handleSysCfg;
    msgcomInstCfg.pktlibInstHandle  = gNetfpServerMCB.pktlibInstanceHandle;
    msgcomInstCfg.malloc            = Msgcom_osalMalloc;
    msgcomInstCfg.free              = Msgcom_osalFree;
    msgcomInstCfg.registerIsr       = Msgcom_osalRegisterIsr;
    msgcomInstCfg.deregisterIsr     = Msgcom_osalDeregisterIsr;
    msgcomInstCfg.disableSysInt     = Msgcom_osalDisableSysInt;
    msgcomInstCfg.enableSysInt      = Msgcom_osalEnableSysInt;
    msgcomInstCfg.enterCS           = Msgcom_osalEnterSingleCoreCS;
    msgcomInstCfg.exitCS            = Msgcom_osalExitSingleCoreCS;
    msgcomInstCfg.createSem         = Msgcom_osalCreateSem;
    msgcomInstCfg.deleteSem         = Msgcom_osalDeleteSem;
    msgcomInstCfg.postSem           = Msgcom_osalPostSem;
    msgcomInstCfg.pendSem           = Msgcom_osalPendSem;

    /* Create the MSGCOM instance */
    gNetfpServerMCB.appMsgcomInstanceHandle = Msgcom_createInstance (&msgcomInstCfg, &errCode);
    if (gNetfpServerMCB.appMsgcomInstanceHandle == NULL)
    {
        NetfpServer_log(NetfpServer_LogLevel_ERROR, "Error: MSGCOM Instance creation failed [Error code %d]\n", errCode);
        return -1;
    }

    /* Initialize and populate the user space interrupt configuration */
    memset ((void *)&uintcConfig, 0, sizeof(UintcConfig));

    /* Populate the user space interrupt configuration:
     * - We are operating the UINTC module in the UINTC managed mode. */
    snprintf(uintcConfig.name, 32, "%s_UINTC", gNetfpServerMCB.serverName);
    uintcConfig.mode            = Uintc_Mode_UINTC_MANAGED;

    /* Initialize the UINTC module. */
    gNetfpServerMCB.uintcHandle = Uintc_init (&uintcConfig, &errCode);
    if (gNetfpServerMCB.uintcHandle == NULL)
    {
        NetfpServer_log(NetfpServer_LogLevel_ERROR, "Error: Unable to open the UINTC module [Error code %d]\n", errCode);
        return -1;
    }
    NetfpServer_log(NetfpServer_LogLevel_DEBUG, "Debug: UINTC module has been opened successfully.\n");

    /* Register the message for the debug information display: */
    if (Uintc_registerMessageId (gNetfpServerMCB.uintcHandle, 0xabcd, NetfpServer_displayServerInfo, NULL, &errCode) < 0)
    {
        NetfpServer_log(NetfpServer_LogLevel_ERROR, "Error: Unable to register the message [Error code %d]\n", errCode);
        return -1;
    }
    NetfpServer_log(NetfpServer_LogLevel_DEBUG, "Debug: UINTC Message Id has been registered successfully.\n");

    /* Create the management interface to the NETFP master: We cannot proceed if the master is NOT operational */
    if (NetfpServer_initMasterMgmtInterface(&gNetfpServerMCB) < 0)
        return -1;

    /* NETFP Server has been started */
    gNetfpServerMCB.serverState = 1;

    /* Initialize the server configuration. */
    memset ((void *)&serverConfig, 0, sizeof(Netfp_ServerConfig));

    /* Populate the server configuration:
     * - Secure Subystem is initialized by the NETFP Master. */
    strcpy (serverConfig.serverName, gNetfpServerMCB.serverName);
    serverConfig.pktlibInstHandle       = gNetfpServerMCB.pktlibInstanceHandle;
    serverConfig.msgcomInstHandle       = gNetfpServerMCB.appMsgcomInstanceHandle;
    serverConfig.realm                  = Netfp_ExecutionRealm_ARM;
    serverConfig.serverHeapHandle       = gNetfpServerMCB.serverHeapHandle;
    serverConfig.initSecureSubsystem    = 0;
    serverConfig.enableIPLutEntryCount  = gNetfpServerMCB.enableIPLutEntryCount;
    serverConfig.nrInstanceId           = gNetfpServerMCB.nrInstanceId;
    serverConfig.cmdHeapHandle          = gNetfpServerMCB.paCommandHeapHandle;
    serverConfig.ipSecHeapHandle        = gNetfpServerMCB.ipSecHeapHandle;
    //fzm-->
    serverConfig.logFxn                 = Netfp_LoggingFxn;
    serverConfig.dumpFxn                = Netfp_DumpingFxn;
    //fzm<--
    serverConfig.maxSecurityChannels    = gNetfpServerMCB.numSecurityChannels;
    serverConfig.baseSecurityContextId  = gNetfpServerMCB.baseSecurityContextId;
    serverConfig.passCfgVirtualAddress  = Resmgr_getPASSVirtualAddress(gNetfpServerMCB.handleSysCfg);
    serverConfig.rmServiceHandle        = Resmgr_getRMServiceHandle (gNetfpServerMCB.handleSysCfg);
    serverConfig.sysRMHandle            = gNetfpServerMCB.handleSysCfg;
    serverConfig.malloc                 = Netfp_osalMalloc;
    serverConfig.free                   = Netfp_osalFree;
    serverConfig.mallocSecurityContext  = Netfp_osalMallocSecurityContext;
    serverConfig.freeSecurityContext    = Netfp_osalFreeSecurityContext;
    serverConfig.beginMemAccess         = Netfp_osalBeginMemoryAccess;
    serverConfig.endMemAccess           = Netfp_osalEndMemoryAccess;
    serverConfig.enterCS                = Netfp_osalEnterSingleCoreCriticalSection;
    serverConfig.exitCS                 = Netfp_osalExitSingleCoreCriticalSection;
    serverConfig.createSem              = Netfp_osalCreateSem;
    serverConfig.deleteSem              = Netfp_osalDeleteSem;
    serverConfig.postSem                = Netfp_osalPostSem;
    serverConfig.pendSem                = Netfp_osalPendSem;

    /* Initialize and create the NETFP Server: */
    gNetfpServerMCB.netfpServerHandle = Netfp_initServer (&serverConfig, &errCode);
    if (gNetfpServerMCB.netfpServerHandle == NULL)
    {
        NetfpServer_log(NetfpServer_LogLevel_ERROR, "Error: Failed to initialize the NETFP Server [Error code %d]\n", errCode);
        Server_logDeInit (); //fzm
        return -1;
    }
    NetfpServer_log(NetfpServer_LogLevel_DEBUG, "Debug: NETFP Server '%s' is operational %p\n", serverConfig.serverName, gNetfpServerMCB.netfpServerHandle);

//fzm-->
    /* Enable the SA 3GPP Enabler: */
    Sa_3gppEnabler();

    NetfpServer_log(NetfpServer_LogLevel_DEBUG, "Debug: ******************************************\n");
    NetfpServer_log(NetfpServer_LogLevel_DEBUG, "Debug: SA3GPP Services Enabled\n");
    NetfpServer_log(NetfpServer_LogLevel_DEBUG, "Debug: ******************************************\n");

    /* Get Queues from RM */
    int32_t  internalErrCode = 0;
    uint32_t tempValue       = 0;

    gNetfpServerMCB.paQueueBounceConfig.enable = 1;
    gNetfpServerMCB.paQueueBounceConfig.hwQueueBegin = 640;
    gNetfpServerMCB.paQueueBounceConfig.hwQueueEnd = 648;
    gNetfpServerMCB.paQueueBounceConfig.defOp[pa_QUEUE_BOUNCE_ROUTING_CLASS_CMD_RET] = 1;
    gNetfpServerMCB.paQueueBounceConfig.defOp[pa_QUEUE_BOUNCE_ROUTING_CLASS_QoS] = 0;
    gNetfpServerMCB.paQueueBounceConfig.defOp[pa_QUEUE_BOUNCE_ROUTING_CLASS_CAPTURE] = 0;
    gNetfpServerMCB.paQueueBounceConfig.defOp[pa_QUEUE_BOUNCE_ROUTING_CLASS_IP_REASSEMBLY] = 1;
    gNetfpServerMCB.paQueueBounceConfig.defOp[pa_QUEUE_BOUNCE_ROUTING_CLASS_MISC] = 1;
    gNetfpServerMCB.paQueueBounceConfig.msmcQueueId = (Qmss_QueueHnd)QMSS_PARAM_NOT_SPECIFIED;
    gNetfpServerMCB.paQueueBounceConfig.ddrQueueId = (Qmss_QueueHnd)QMSS_PARAM_NOT_SPECIFIED;

    /* Prepare QMSS Barrier Queue */
    if (Resmgr_nameServiceGet(gNetfpServerMCB.handleSysCfg,
                "QMSS_BarrierQ_MSMC_NetCP",
                (uint32_t*)&tempValue,
                &internalErrCode) < 0) {
        gNetfpServerMCB.paQueueBounceConfig.msmcQueueId = (Qmss_QueueHnd)QMSS_PARAM_NOT_SPECIFIED;
        NetfpServer_log(NetfpServer_LogLevel_INFO, "Resmger_nameServiceGet returned errcode %d for msmcQueueId\n", internalErrCode);
    } else {
        gNetfpServerMCB.paQueueBounceConfig.msmcQueueId = tempValue;
    }

    tempValue = 0;
    if (Resmgr_nameServiceGet(gNetfpServerMCB.handleSysCfg,
                "QMSS_BarrierQ_DDR_NetCP",
                (uint32_t*)&tempValue,
                &internalErrCode) < 0) {
        gNetfpServerMCB.paQueueBounceConfig.ddrQueueId = (Qmss_QueueHnd)QMSS_PARAM_NOT_SPECIFIED;
        NetfpServer_log(NetfpServer_LogLevel_INFO, "Resmger_nameServiceGet returned errcode %d for ddrQueueId\n", internalErrCode);
    } else {
        gNetfpServerMCB.paQueueBounceConfig.ddrQueueId = tempValue;
    }

    /* Initalize Barrier Q */
    if(Netfp_initQueueBounce(gNetfpServerMCB.netfpServerHandle, gNetfpServerMCB.paQueueBounceConfig, &retVal) < 0 )
    {
        NetfpServer_log(NetfpServer_LogLevel_ERROR, "Error:PA queue bounce initialization failed [Error code %d]\n", retVal);
        return -1;
    }
//fzm<--

    /* Launch the NETFP Master management interface thread once the server is operational. */
    retVal = pthread_create (&masterMgmtInterfaceThread, NULL, NetfpServer_masterMgmtInterfaceThread, &gNetfpServerMCB);
    if (retVal < 0)
    {
        NetfpServer_log(NetfpServer_LogLevel_ERROR, "Error: Master Management Interface thread create failed error code %d\n", retVal);
        return -1;
    }

    /* Launch the NETFP Server management thread */
    retVal = pthread_create (&serverMgmtThread, NULL, NetfpServer_mgmtThread, NULL);
    if (retVal < 0)
    {
        NetfpServer_log(NetfpServer_LogLevel_ERROR, "Error: Server thread create failed error code %d\n", retVal);
        return -1;
    }

    /* Wait for the management thread to terminate. */
    pthread_join (serverMgmtThread, NULL);

    /* Shutdown and cancel the master interface thread: */
    pthread_cancel (masterMgmtInterfaceThread);

    /* Wait and ensure that the interface thread has been cancelled before proceeding. */
    pthread_join (masterMgmtInterfaceThread, NULL);

    /* Delete the management interface to the NETFP master: */
    NetfpServer_deinitMasterMgmtInterface(&gNetfpServerMCB);

    return 0; //fzm: https://e2eprivate.ti.com/nokia_siemens_networks/k2_-_fsm4_-_fzm_-_lrc_nokia/f/191/t/5791.aspx

    /* Shutdown the PKTLIB heaps which had been created by the NETFP Server */
    if (Pktlib_deleteHeap (gNetfpServerMCB.pktlibInstanceHandle, gNetfpServerMCB.paCommandHeapHandle, &errCode) < 0)
        NetfpServer_log(NetfpServer_LogLevel_ERROR, "Error: NETFP PA command heap deletion failed [Error code %d]\n", errCode);
    if (Pktlib_deleteHeap (gNetfpServerMCB.pktlibInstanceHandle, gNetfpServerMCB.ipSecHeapHandle, &errCode) < 0)
        NetfpServer_log(NetfpServer_LogLevel_ERROR, "Error: NETFP IPSEC heap deletion failed [Error code %d]\n", errCode);
    if (Pktlib_deleteHeap (gNetfpServerMCB.pktlibInstanceHandle, gNetfpServerMCB.serverHeapHandle, &errCode) < 0)
        NetfpServer_log(NetfpServer_LogLevel_ERROR, "Error: NETFP Server heap deletion failed [Error code %d]\n", errCode);

    /* Shutdown the PKTLIB instance */
    if (Pktlib_deleteInstance (gNetfpServerMCB.pktlibInstanceHandle, &errCode) < 0)
    {
        NetfpServer_log(NetfpServer_LogLevel_ERROR, "Error: Unable to delete the PKTLIB instance [Error code %d]\n", errCode);
        return -1;
    }

    /* Cleanup the resource manager configuration */
    if (Resmgr_deinit (gNetfpServerMCB.handleSysCfg, &errCode) < 0)
        NetfpServer_log(NetfpServer_LogLevel_ERROR, "Error: Shutting down the system configuration failed\n");
    else
        NetfpServer_log(NetfpServer_LogLevel_DEBUG, "Debug: Shutting down the system configuration passed\n");
    return 0;
}

