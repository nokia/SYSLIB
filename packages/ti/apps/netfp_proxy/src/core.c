/**
 *   @file  core.c
 *
 *   @brief
 *      NETFP Proxy Core Implementation
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2015 Texas Instruments, Inc.
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
//<----fzm
#include <semaphore.h>
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <pthread.h>
#include <getopt.h>
#include <time.h>
#include <signal.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/un.h>
#include <dlfcn.h>
#include <dirent.h>

/* JOSH Include Files. */
#include <ti/runtime/josh/josh.h>
#include <ti/runtime/josh/include/josh_internal.h>
#include <ti/runtime/josh/include/listlib.h>

/* NETFP Include Files. */
#include <ti/runtime/netfp/netfp.h>
#include <ti/runtime/netfp/include/netfp_internal.h>

/* NETFP Proxy includes */
#include <ti/apps/netfp_proxy/include/netfp_proxy_pvt.h>
#include <ti/apps/netfp_proxy/netfp_proxy.h>

/* MCSDK Include files. */
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

/**********************************************************************
 ************************** Local Definitions *************************
 **********************************************************************/

/* This flag is used to define the operation of the NETFP Proxy to be
 * in daemon mode or not. Undefine this to log all the console outputs
 * on the terminal. Useful in certain cases to debug certain issues. */
#define DAEMON_MODE

/**********************************************************************
 ************************** Global Variables **************************
 **********************************************************************/

/**
 * @brief   Global NETFP Proxy Instance
 */
NetfpProxy_MCB                  gNetfpProxyMcb;

//fzm-->
uint8_t criticalErrorOccurred = 0;
uint32_t gL3QosPassFlowId;
sem_t shutdownSemaphore;
//fzm<--

/**
 * @brief   WA: Mutex used for libnl not being threadsafe.
 */
pthread_mutex_t libnlGuard = PTHREAD_MUTEX_INITIALIZER;

/**********************************************************************
 ************************** OSAL Functions ****************************
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
extern void* Netfp_osalEnterSingleCoreCriticalSection (void);
extern void  Netfp_osalExitSingleCoreCriticalSection (void* csHandle);
extern void  Netfp_osalBeginMemoryAccess (void* ptr, uint32_t size);
extern void  Netfp_osalEndMemoryAccess (void* ptr, uint32_t size);
extern void* Netfp_osalCreateSem(void);
extern void  Netfp_osalDeleteSem(void*);
extern void  Netfp_osalPostSem(void*);
extern void  Netfp_osalPendSem(void*);

/**********************************************************************
 ************************* Local Functions ****************************
 **********************************************************************/

/* Shutdown the NETFP Proxy application: */
static void NetfpProxy_coreShutDown(void);

//static void Netfp_displayProxyToServerInterface(void);
//static void Netfp_proxyToServerNodeRecycling(void);

/**********************************************************************
 ************************* Core Functions *****************************
 **********************************************************************/

#if 0 // fzm: function definition was moved to netfp_proxy_logging.c to provide support for shared memory logger
/**
 *  @b Description
 *  @n
 *      NETFP Proxy logging function which is exported and used by all the internal
 *      modules.
 *
 *  @param[in]  level
 *      Print level
 *
 *  @param[in]  fmt
 *      Formatting string
 *
 *  @retval
 *      Not Applicable.
 */
void NetfpProxy_logMsg (NetfpProxy_LogLevel level, char* fmt, ...)
{
    va_list arg;

    /* Get the formatted argument list */
    va_start (arg, fmt);

#ifdef DAEMON_MODE
    /* Is the plugin registration complete? */
    if (gNetfpProxyMcb.pluginCfg.logMsg)
    {
        /* Plugin registered. Log the message using the plugins logger API */
        gNetfpProxyMcb.pluginCfg.logMsg (level, fmt, arg);
    }
    else
    {
        /* No, plugin not registered yet. Print to console instead */
        vprintf (fmt, arg);
    }
#else
    /* In the normal mode; bypass the plugin logging and dump all the information on the console. */
    vprintf (fmt, arg);
#endif

    /* Done using the argument list. Clean it up */
    va_end (arg);
    return;
}
#endif // </fzm>

//fzm-->
void NetfpProxy_assertCriticalError(const int32_t errorCode, const char* functionName, uint32_t lineNumber)
{
    if(criticalErrorOccurred)
        return;
    switch (errorCode)
    {
        case NETFP_EJOSH:
        case NETFP_ETIMEOUT:
        case NETFP_ENOMEM:
        case NETFP_EINTERNAL:
        case NETFP_PROXY_RETVAL_E_NO_MEM:
            criticalErrorOccurred = 1;
            NetfpProxy_logMsg(NETFP_PROXY_LOG_ERROR, "Critical error occurred in function %s, line %u. Error number: %d", functionName, lineNumber, errorCode);
        default:
            return;
    }
}

/*
    Reading from pipe may be interupted by any signal (USER1 per syslib design) and it should not return error.
    Thus readFromPipe covers such scenario as possible use case.
*/
int readFromPipe(int fd, void *buf, size_t count, int* error)
{
    size_t   leftToRead = count;
    uint32_t limit = 0;

    do
    {
        uint32_t offset = 0;
        ssize_t readBytes = read(fd, (char*)buf + offset, leftToRead);

        if (readBytes < 0)
        {
            if (errno != EINTR)
                break;
            else
                NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "SIGNAL received, error %s\n", strerror(errno));
        }
        else
        {
            leftToRead -= readBytes;
            offset += readBytes;
        }

    } while (leftToRead && limit++ < 10);

    if (leftToRead)
    {
        *error = errno;

        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Unable to retrieve message from the pipe, read only %u of %u, error %s\n",
                           count - leftToRead, count, strerror(errno));

        return -1;
    }

    return 0;
}

/*
    Writing to the pipe may be interupted by any signal (USER1 per syslib design) and it should not return error.
    Thus writeToPipe covers such scenario as possible use case.
*/
int writeToPipe(int fd, const void *buf, size_t count, int* error)
{
    uint32_t limit = 0;
    ssize_t written;

    do
    {
        written = write(fd, buf, count);

        if (written < 0)
        {
            if (errno != EINTR)
                break;
        }
    } while (written < 0 && limit++ < 10);

    if (written <=0)
    {
        *error = errno;
        return -1;
    }

    return 0;
}

//fzm<--

/**
 *  @b Description
 *  @n
 *      The function is used to display the NETFP Proxy usage
 *
 *  @retval
 *      Not applicable
 */
static void NetfpProxy_usage(void)
{
    printf ("NETFP Proxy usage:");
    printf ("netfp_proxy <arguments>\n");
    printf ("Mandatory Arguments: \n");
    printf ("-p <pluginName>     - Plugin name\n");
    printf ("-i <DatabaseId>     - Name Database Id\n");
    printf ("-s <NETFP Server>   - Name of the NETFP Server\n");
    printf ("-c <Proxy Name>     - Name of the NETFP Proxy Client\n");
    printf ("-r <RM Client Name> - Name of the RM client\n");
    printf ("-u <Update Time>    - Timeout to poll the SA statistics\n");
    return;
}

#if 0
static void Netfp_displayProxyToServerInterface(void)
{
    /* Display the banner. */
    NetfpProxy_dumpMsg (NETFP_PROXY_LOG_INFO, "**********************************************************\n");
    NetfpProxy_dumpMsg (NETFP_PROXY_LOG_INFO, "Proxy To Server Interface [Pending Jobs List]\n");

    /* Cycle through the Proxy-Server Job List */
    Netfp_ProxyToServerNode* ptrProxyToServerNode = (Netfp_ProxyToServerNode*)Netfp_listGetHead ((Netfp_ListNode**)&gNetfpProxyMcb.ptrListProxyToServer);
    while (ptrProxyToServerNode != NULL)
    {
        NetfpProxy_dumpMsg (NETFP_PROXY_LOG_INFO, "JobId: %d waiting for cleanup\n", ptrProxyToServerNode->jobId);

        /* Get the next element in the list: */
        ptrProxyToServerNode = (Netfp_ProxyToServerNode*)Netfp_listGetNext ((Netfp_ListNode*)ptrProxyToServerNode);
    }
}
#endif

/**
 *  @b Description
 *  @n
 *      The function is used to display the NETFP Proxy configuration
 *
 *  @retval
 *      Not applicable
 */
static void NetfpProxy_coreDump (void)
{
    if ( NetfpProxy_dumpInit() == -1) //fzm
        return;

    /* Dump Proxy state info snapshot */
//fzm - use NetfpProxy_dumpMsg to capture this with the USR1 signal
    NetfpProxy_dumpMsg (NETFP_PROXY_LOG_INFO, "*******************************************\n");
    NetfpProxy_dumpMsg (NETFP_PROXY_LOG_INFO, "NETFP Proxy Dump:\n");
    NetfpProxy_dumpMsg (NETFP_PROXY_LOG_INFO, "*******************************************\n");
    NetfpProxy_dumpMsg (NETFP_PROXY_LOG_INFO, "Debug: Plugin Name      : %s\n", gNetfpProxyMcb.pluginName);
    NetfpProxy_dumpMsg (NETFP_PROXY_LOG_INFO, "Debug: Polling Delay(ms): %d\n", gNetfpProxyMcb.pollDelay);
    NetfpProxy_dumpMsg (NETFP_PROXY_LOG_INFO, "Debug: Stats Update(sec): %d\n", gNetfpProxyMcb.statsUpdateInterval);
    NetfpProxy_dumpMsg (NETFP_PROXY_LOG_INFO, "Debug: Named Resource Id: %d\n", gNetfpProxyMcb.nrInstanceId);
    NetfpProxy_dumpMsg (NETFP_PROXY_LOG_INFO, "Debug: RM Client Name   : %s\n", gNetfpProxyMcb.rmClientName);
    NetfpProxy_dumpMsg (NETFP_PROXY_LOG_INFO, "Debug: NETFP Server Name: %s\n", gNetfpProxyMcb.netfpServerName);
    NetfpProxy_dumpMsg (NETFP_PROXY_LOG_INFO, "Debug: NETFP Client Name: %s\n", gNetfpProxyMcb.netfpClientName);
    NetfpProxy_dumpMsg (NETFP_PROXY_LOG_INFO, "Debug: Is Proxy in Reset: %s \n",(gNetfpProxyMcb.bIsInReset == 1) ? "Yes" : "No");

    //Netfp_displayProxyToServerInterface();

    /* Dump the contents of each individual module: */
//fzm    NetfpProxy_ipsecDump();
    NetfpProxy_routeDump();
    NetfpProxy_ifaceDump();
    /* This function has to be invoked last because it executes from different thread and
       appends terminator pattern for FPControl app at the end of the log */
    NetfpProxy_neighDump();
    return;
}

/**
 *  @b Description
 *  @n
 *      NETFP Proxy signal handler.
 *
 *  @param[in]  sigNum
 *      Signal that triggered this function
 */
static void NetfpProxy_coreSignalHandler (int32_t sigNum)
{
    switch (sigNum)
    {
        case SIGTERM:
        case SIGINT:
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Signal %d received", sigNum);

            sem_post(&shutdownSemaphore);
        }
        case SIGUSR1:
        {
            /* Log Proxy state */
            NetfpProxy_coreDump ();
            break;
        }
        default:
        {
            /* Ignore all else and continue processing */
            break;
        }
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to parse the command line arguments.
 *
 *  @param[in]  argc
 *      Number of arguments passed to the application
 *  @param[in]  argv
 *      Arguments passed to the application.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t NetfpProxy_coreParseCommandLineArgs (int argc, char** argv)
{
//fzm-->
    while (1)
    {
        int c = getopt(argc, argv, "s:c:r:i:p:d:u:tv:");

        if (c == -1)
            break;

        switch (c)
        {
            case 's':
            {
                strncpy (gNetfpProxyMcb.netfpServerName, optarg, NETFP_MAX_CHAR-1);
                break;
            }
            case 'c':
            {
                strncpy (gNetfpProxyMcb.netfpClientName, optarg, NETFP_MAX_CHAR-1);
                break;
            }
            case 'r':
            {
                strncpy (gNetfpProxyMcb.rmClientName, optarg, RM_NAME_MAX_CHARS-1);
                break;
            }
            case 'i':
            {
                gNetfpProxyMcb.nrInstanceId = atoi(optarg);
                break;
            }
            case 'p':
            {
                strncpy (gNetfpProxyMcb.pluginName, optarg, sizeof(gNetfpProxyMcb.pluginName)-1);
                break;
            }
            case 'd':
            {
                gNetfpProxyMcb.pollDelay = atoi(optarg);
                break;
            }
            case 'u':
            {
                gNetfpProxyMcb.statsUpdateInterval = atoi(optarg);
                break;
            }
            case 't':
            {
                gNetfpProxyMcb.testMode = 1;
                break;
            }
            case 'v':
            {
                if (!strcmp("VRB", optarg))
                {
                    NetfpProxy_setLogLevel(NETFP_PROXY_LOG_VRB);
                }
                else if (!strcmp("DBG", optarg))
                {
                    NetfpProxy_setLogLevel(NETFP_PROXY_LOG_DEBUG);
                }
                else if (!strcmp("INF", optarg))
                {
                    NetfpProxy_setLogLevel(NETFP_PROXY_LOG_INFO);
                }
                else if (!strcmp("ERR", optarg))
                {
                    NetfpProxy_setLogLevel(NETFP_PROXY_LOG_ERROR);
                }
                else
                {
                    NetfpProxy_logMsg(NETFP_PROXY_LOG_ERROR, "Error: wrong logging level %s, supported VRB|DBG|INF|ERR\n", optarg);
                }
                break;
            }
            case '?':
            {
                return -1;
            }
        }
    }
//fzm<--

    /* Validate configuration. All mandatory parameters configured? */
    if ((gNetfpProxyMcb.netfpServerName[0] == '\0') || (gNetfpProxyMcb.netfpClientName[0] == '\0') ||
        (gNetfpProxyMcb.rmClientName[0] == '\0')    || (gNetfpProxyMcb.pluginName[0] == '\0'))
    {
        return -1;
    }

    return 0;
}

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
static uint8_t* NetfpProxy_coreDataMalloc(uint32_t size, uint32_t arg)
{
//fzm---->
    (void)arg;

    uint8_t* ptr = NULL;
    if(ddal_cma_alloc(DDAL_CMA_MEM_TYPE_DDR, size, (void *)&ptr) != DDAL_OK)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to allocate memory");
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
static void NetfpProxy_coreDataFree(uint8_t* ptr, uint32_t size, uint32_t arg)
{
    return;
}

/**
 *  @b Description
 *  @n
 *      NETFP Proxy UINTC thread which is woken up on the reception of
 *      an interrupt. ISR are executed in this thread context.
 *
 *  @param[in]  arg
 *      Argument passed to the thread.
 *
 *  @retval
 *      Not Applicable.
 */
static void* NetfpProxy_coreExecuteUINTC(void *arg)
{
    int32_t             errCode;
    int32_t             retVal;
    struct sched_param  param;

    /* Set the configured policy and priority */
    param.sched_priority = NETFP_PROXY_UINTC_SCHED_PRIORITY; //fzm
    errCode = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
    if (errCode != 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to set the UINTC thread priority & policy [Error Code %d]\n", errCode);
        return NULL;
    }

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Launched the UINTC Thread\n");

    /* Loop around: The UINTC thread is waiting for an interrupt to arrive on events which have been registered
     * with the UINTC instance. */
    while (1)
    {
        /* Dispatch received events to the appropriate handler. */
        retVal = Uintc_select (gNetfpProxyMcb.uintcHandle, NULL, &errCode);
        if (retVal < 0)
        {
            /* Error: UINTC select failed. Has the UINTC module been deinitialized? */
            if (errCode == UINTC_EDEINIT)
                break;

            /* Report the UINTC Module error: */
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: UINTC select failed [Error code %d]\n", errCode);
            NetfpProxy_assertCriticalError(errCode, __func__, __LINE__); //fzm

            return NULL;
        }
    }
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      This is the NETFP Client Thread which needs to execute in the background
 *      and is used to communicate and exchange messages with the NETFP Server
 *
 *  @param[in]  arg
 *      Argument passed to the thread.
 *
 *  @retval
 *      Not Applicable.
 */
static void* NetfpProxy_coreExecuteNETFPClient (void* arg)
{
    Netfp_ClientHandle  netfpClientHandle = (Netfp_ClientHandle)arg;
    struct sched_param  param;
    int32_t             errCode;

    /* Set the configured policy and priority */
    param.sched_priority = 0; //fzm
    errCode = pthread_setschedparam(pthread_self(), SCHED_OTHER, &param);
    if (errCode != 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to set the NETFP Client thread priority & policy [Error Code %d]\n", errCode);
        return NULL;
    }

    if(nice(NETFP_PROXY_CLIENT_NICENESS) == -1)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error while setting nice value, errno: %s", strerror(errno));
        return NULL;
    }

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Launched the NETFP Client Thread [Priority %d]\n", param.sched_priority);

    /* Execute the NETFP Client. */
    while (1)
        Netfp_executeClient (netfpClientHandle);

    /* Control never comes here */
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      The function is used to execute and process the NETFP-Proxy Core events.
 *
 *  @param[in]
 *      Not applicable
 *
 *  @retval
 *      Not applicable
 */
static void NetfpProxy_coreExecute (void)
{
    static  Netfp_ProxyServerBulkMsg proxyServerBulkMsg; //this local variable is so big so keep it as static
    int     error;

    /* Most of cases we expect only one 'proxyServerInfo' item so always read 1st one
       If 'numberOfEntries > 1' next read opeartion will pull the rest */
    size_t expected = sizeof(proxyServerBulkMsg.numberOfEntries) + sizeof(proxyServerBulkMsg.proxyServerInfo[0]);

    if (readFromPipe(gNetfpProxyMcb.corePipe[0], (void*)&proxyServerBulkMsg, expected, &error) < 0)
    {
        /* Error: Unable to receive data from the core socket */
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to receive message size from the core pipe, Error: %s\n", strerror(error));
        return;
    }

    if (proxyServerBulkMsg.numberOfEntries > 1)
    {
        expected = sizeof(proxyServerBulkMsg.proxyServerInfo[0]) * (proxyServerBulkMsg.numberOfEntries - 1);

        if (readFromPipe(gNetfpProxyMcb.corePipe[0], (void*)&proxyServerBulkMsg.proxyServerInfo[1], expected, &error) < 0)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to receive whole message from the core pipe, Error: %s\n", strerror(error));
            return;
        }
    }

    /* Lookup the route: */
    NetfpProxy_routeLookup (&proxyServerBulkMsg);

    return;
}

#if 0
void Netfp_addToPendingProxyToServerList(int32_t jobId)
{
    /* Allocate memory for the new jobId */
    Netfp_ProxyToServerNode* ptrProxyToServerNode = malloc(sizeof(Netfp_ProxyToServerNode));

    if (ptrProxyToServerNode == NULL)
    {

        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Lack of free memory to allocate new proxy to server node\n");
        return;
    }

    ptrProxyToServerNode->jobId = jobId;

    Netfp_listAddTail ((Netfp_ListNode**)&gNetfpProxyMcb.ptrListProxyToServer, (Netfp_ListNode*)ptrProxyToServerNode);
}

static void Netfp_proxyToServerNodeRecycling(void)
{
    Netfp_ClientMCB* clientHandle= (Netfp_ClientMCB*)gNetfpProxyMcb.netfpClientHandle;

    Netfp_ProxyToServerNode* ptrProxyToServerNode = (Netfp_ProxyToServerNode*)Netfp_listGetHead((Netfp_ListNode**)&gNetfpProxyMcb.ptrListProxyToServer);
    while (ptrProxyToServerNode != NULL)
    {
        uint32_t jobResult = 0;
        int32_t errCode;

        Netfp_ProxyToServerNode* ptrProxyToServerNodeNext = (Netfp_ProxyToServerNode*)Netfp_listGetNext((Netfp_ListNode*)ptrProxyToServerNode);

        /* YES. Is the job associated with the event complete? */
        int32_t retVal = Josh_isJobCompleted (clientHandle->joshHandle, ptrProxyToServerNode->jobId, &jobResult, &errCode);
        if (retVal < 0)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "JOSH Job %d failure [Error code %d]\n",
                          ptrProxyToServerNode->jobId, errCode);
        }
        else
        {
            /* Is the job completed? */
            if (retVal == 1)
            {
                if (jobResult)
                {
                    NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "JOSH Job %d operation failed [Error code %d]\n",
                                  ptrProxyToServerNode->jobId, jobResult);
                }

                /* JOB has been completed: Cleanup the job in the JOSH. */
                errCode = Josh_freeJobInstance(clientHandle->joshHandle, ptrProxyToServerNode->jobId);

                if (errCode < 0)
                    NetfpProxy_assertCriticalError(errCode, __func__, __LINE__);

                /* Remove this from the list. */
                Netfp_listRemoveNode((Netfp_ListNode**)&gNetfpProxyMcb.ptrListProxyToServer, (Netfp_ListNode*)ptrProxyToServerNode);

                free(ptrProxyToServerNode);
            }
        }

        ptrProxyToServerNode = ptrProxyToServerNodeNext;

    }
}
#endif

/**
 *  @b Description
 *  @n
 *      NETFP-Proxy main processing task.
 *
 *  @param[in]
 *      Not applicable
 *
 *  @retval
 *      Not applicable
 */
static void* NetfpProxy_coreRun (void* arg)
{
    fd_set          fds;
    int32_t         maxFd;
    int32_t         errCode;
    struct timeval  timeout;

    /* Get the maximum file descriptor */
    maxFd = max (gNetfpProxyMcb.routeSocket, gNetfpProxyMcb.ifSocket);
    maxFd = max (maxFd, gNetfpProxyMcb.corePipe[0]);
    maxFd = max (maxFd, gNetfpProxyMcb.neighSocket);
    maxFd = max (maxFd, gNetfpProxyMcb.pluginSocket);

    if(nice(NETFP_PROXY_CORE_THREAD_NICENESS) == -1)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error while setting nice value on core thread, errno: %s", strerror(errno));
        return NULL;
    }

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Launched the NETFP Proxy Thread [Max: %d]\n", maxFd);

    /* Initialize the timeout: */
    timeout.tv_sec  = 0;
    timeout.tv_usec = gNetfpProxyMcb.pollDelay * 1000;

    while (1)
    {
        /* Setup the event FIFO to wait on using select () */
        FD_ZERO (&fds);
        FD_SET (gNetfpProxyMcb.ifSocket, &fds);
        FD_SET (gNetfpProxyMcb.routeSocket, &fds);
        FD_SET (gNetfpProxyMcb.corePipe[0], &fds);
        FD_SET (gNetfpProxyMcb.neighSocket, &fds);
        FD_SET (gNetfpProxyMcb.pluginSocket, &fds);

        /* Wait for the events to arrive */
        errCode = select ((maxFd + 1), &fds, NULL, NULL, &timeout);
        if (errCode < 0)
        {
            /* If the error code is because of a signal retry again. */
            if (errno == EINTR)
                continue;

            /* Error: */
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to select in the NETFP Proxy Thread[Error: %s]\n", strerror(errno));
            break;
        }

        /* Is there a timeout? */
        if (errCode == 0)
        {
            /* YES: Execute any module which executes in a periodic polled mode. */
//fzm            NetfpProxy_ipsecExecute();
            NetfpProxy_routePoll ();

            /* Reset the timeout: */
            timeout.tv_sec  = 0;
            timeout.tv_usec = gNetfpProxyMcb.pollDelay * 1000;
        }

        // Netfp_proxyToServerNodeRecycling();

        /* Did we get a core event? */
        if (FD_ISSET(gNetfpProxyMcb.corePipe[0], &fds) != 0)
            NetfpProxy_coreExecute();

        /* Did we get a neighbor event? */
        if (FD_ISSET(gNetfpProxyMcb.neighSocket, &fds) != 0)
            NetfpProxy_neighExecute();

        /* Did we get an interface event? */
        if (FD_ISSET(gNetfpProxyMcb.ifSocket, &fds) != 0)
            NetfpProxy_ifaceExecute();

        /* Did we get a route event? */
        if (FD_ISSET(gNetfpProxyMcb.routeSocket, &fds) != 0)
            NetfpProxy_routeExecute();

        /* Did we get a plugin event? */
        if (FD_ISSET(gNetfpProxyMcb.pluginSocket, &fds) != 0)
            gNetfpProxyMcb.pluginCfg.run();
    }

    return NULL;
}

/**
 *  @b Description
 *  @n
 *      This is the NETFP Proxy registered call back function register the CORE module with
 *      the NETFP Server. The NETFP Server will invoke the API in order to resolve a route
 *      request; this is called in the context of the NETFP Client thread.
 *
 *  @param[in]  clientHandle
 *      NETFP client handle
 *  @param[in]  ptrProxyServerBulkMsg
 *      Pointer to the Netfp proxy server interface information block.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t NetfpProxy_coreProcessServerMsg
(
    Netfp_ClientHandle          clientHandle,
    Netfp_ProxyServerBulkMsg*   ptrProxyServerBulkMsg
)
{
    int error;

    /* Sanity Check: Validate the arguments */
    if ((clientHandle == NULL) || (ptrProxyServerBulkMsg == NULL))
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Invalid NULL arguments passed for route resolution\n");
        return NETFP_PROXY_RETVAL_E_OP_FAILED;
    }

    /* Sanity Check: Validate the range of items */
    if ((!ptrProxyServerBulkMsg->numberOfEntries) || (ptrProxyServerBulkMsg->numberOfEntries > BULK_INFO_MAX_ENTRIES))
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Invalid Operation sent to NETFP Proxy, number of entries %u\n", ptrProxyServerBulkMsg->numberOfEntries);
        return NETFP_PROXY_RETVAL_E_OP_FAILED;
    }

    for (int i = 0 ; i < ptrProxyServerBulkMsg->numberOfEntries ; i++)
    {
        /* Sanity Check: Make sure we got a NETFP Proxy request */
        if (ptrProxyServerBulkMsg->proxyServerInfo[i].opType != Netfp_ProxyServerOp_REQUEST)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Invalid Operation Type sent to NETFP Proxy %u, %d of %u\n",
                               ptrProxyServerBulkMsg->proxyServerInfo[i].opType,
                               i, ptrProxyServerBulkMsg->numberOfEntries);
            return NETFP_PROXY_RETVAL_E_OP_FAILED;
        }

        /* Sanity Check: Validate the source and destination version match */
        if (ptrProxyServerBulkMsg->proxyServerInfo[i].srcIP.ver != ptrProxyServerBulkMsg->proxyServerInfo[i].dstIP.ver)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Invalid Source and Destination IP version in route resolution\n");
            return NETFP_PROXY_RETVAL_E_OP_FAILED;
        }
    }

    uint32_t sizeOfMsg = sizeof(ptrProxyServerBulkMsg->numberOfEntries) +
                       ptrProxyServerBulkMsg->numberOfEntries * sizeof(ptrProxyServerBulkMsg->proxyServerInfo[0]);

    /* Send the message to the PROXY core module for route resolution: */
    if (writeToPipe(gNetfpProxyMcb.corePipe[1], (void*)ptrProxyServerBulkMsg, sizeOfMsg, &error) < 0 )
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "CoreProcessServerMsg request failed: error %s\n", strerror(error));
        return NETFP_PROXY_RETVAL_E_OP_FAILED;
    }
    else
        return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to initialize and setup the SYSLIB modules and initialize
 *      the NETFP clients and connect this to the NETFP Server.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t NetfpProxy_coreInitNetFP (void)
{
    Name_DatabaseCfg            databaseCfg;
    int32_t                     errCode;
    Resmgr_SystemCfg            sysConfig;
    Resmgr_ResourceCfg*         ptrResCfg;
    Netfp_ClientConfig          clientConfig;
    UintcConfig                 uintcConfig;
    int32_t                     clientStatus;
    Pktlib_InstCfg              pktlibInstCfg;
    Msgcom_InstCfg              msgcomInstCfg;
    Pktlib_HeapCfg              heapCfg;

    //fzm-->
    Pktlib_HeapCfg              l3QosHeapCfg;
    Pktlib_HeapHandle           l3QosHeapHandle;
    Netfp_FlowCfg flowCfg;
    //fzm<--

    //fzm --->
    //initialize the ddal cma subsystem (mmaps /dev/cma into our address space)
    int ddalCmaInitStatus = ddal_cma_process_init();
    if(ddalCmaInitStatus != DDAL_OK)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Failed to initialize DDAL CMA, errcode: %d", ddalCmaInitStatus);
        return -1;
    }
    //<---- fzm

    /* Initialize the database configuration */
    memset ((void *)&databaseCfg, 0, sizeof(Name_DatabaseCfg));

    /* Populate the configuration: */
    databaseCfg.instanceId  = gNetfpProxyMcb.nrInstanceId;
    databaseCfg.realm       = Name_ExecutionRealm_ARM;
    strncpy (databaseCfg.owner, gNetfpProxyMcb.netfpClientName, NETFP_MAX_CHAR-1);

    /* Create the global database */
    gNetfpProxyMcb.databaseHandle = Name_createDatabase (&databaseCfg, &errCode);
    if (gNetfpProxyMcb.databaseHandle == NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Name database creation failed [Error code %d]\n", errCode);
        return -1;
    }

    /* Initialize the system configuration. */
    memset ((void *)&sysConfig, 0, sizeof(Resmgr_SystemCfg));

    /* Populate the configuration.*/
    sysConfig.realm                 = Resmgr_ExecutionRealm_ARM;
    strncpy (sysConfig.rmClient, gNetfpProxyMcb.rmClientName, RM_NAME_MAX_CHARS-1);
    strncpy (sysConfig.rmServer, "Rm_Server", RM_NAME_MAX_CHARS-1);       //Assuming RMv2 server name.
    sysConfig.coreId                = SYSLIB_ARM_CORE_ID;
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
    gNetfpProxyMcb.sysCfgHandle = Resmgr_init(&sysConfig, &errCode);
    if (gNetfpProxyMcb.sysCfgHandle == NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: SYSRM initialization failed with error code %d\n", errCode);
        goto cleanup_return_error;
    }
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: SYSRM initialized successfully [Handle %p]\n", gNetfpProxyMcb.sysCfgHandle);

    /* Get the handle to the Proxy resource configuration */
    ptrResCfg = &gNetfpProxyMcb.appResourceConfig;

    /* Setup a resource request. */
    memset ((void*)ptrResCfg, 0, sizeof(Resmgr_ResourceCfg));
    snprintf (ptrResCfg->memRegionCfg[0].name, sizeof(ptrResCfg->memRegionCfg[0].name)-1,
              "%s_DDR3_MemRegion", gNetfpProxyMcb.netfpClientName);
    ptrResCfg->numQpendQueues               = 0; //fzm
    ptrResCfg->memRegionCfg[0].type         = Resmgr_MemRegionType_DDR3;
    ptrResCfg->memRegionCfg[0].linkingRAM   = Resmgr_MemRegionLinkingRAM_INTERNAL; //fzm
    ptrResCfg->memRegionCfg[0].numDesc      = 4096;   //fzm<--
    ptrResCfg->memRegionCfg[0].sizeDesc     = 128;

    /* Get HW resources required for Proxy to communicate with NetFP server */
    if (Resmgr_processConfig (gNetfpProxyMcb.sysCfgHandle, ptrResCfg, &errCode) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: SYSRM configuration failed with error code %d\n", errCode);
        goto cleanup_return_error;
    }

    /* Initialize the PKTLIB instance configuration. */
    memset ((void *)&pktlibInstCfg, 0, sizeof(Pktlib_InstCfg));

    /* Populate the PKTLIB Instance configuration: */
    pktlibInstCfg.instanceId        = 1; //TODO: Will be obsoleted. Clean this up then.
    pktlibInstCfg.databaseHandle    = gNetfpProxyMcb.databaseHandle;
    pktlibInstCfg.sysCfgHandle      = gNetfpProxyMcb.sysCfgHandle;
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
    gNetfpProxyMcb.pktlibInstHandle = Pktlib_createInstance (&pktlibInstCfg, &errCode);
    if (gNetfpProxyMcb.pktlibInstHandle == NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Creating the PKTLIB instance failed [Error code %d]\n", errCode);
        goto cleanup_return_error;
    }

    /* Initialize the MSGCOM instance. */
    memset ((void *)&msgcomInstCfg, 0, sizeof(Msgcom_InstCfg));

    /* Populate the MSGCOM Instance configuration */
    msgcomInstCfg.databaseHandle    = gNetfpProxyMcb.databaseHandle;
    msgcomInstCfg.sysCfgHandle      = gNetfpProxyMcb.sysCfgHandle;
    msgcomInstCfg.pktlibInstHandle  = gNetfpProxyMcb.pktlibInstHandle;
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
    gNetfpProxyMcb.msgcomInstHandle = Msgcom_createInstance (&msgcomInstCfg, &errCode);
    if (gNetfpProxyMcb.msgcomInstHandle == NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: MSGCOM Instance creation failed [Error code %d]\n", errCode);
        goto cleanup_return_error;
    }

    /* Initialize and populate the user space interrupt configuration */
    memset ((void *)&uintcConfig, 0, sizeof(UintcConfig));

    /* Populate the user space interrupt configuration: */
    strncpy(uintcConfig.name, gNetfpProxyMcb.netfpClientName, sizeof(uintcConfig.name)-1);
    uintcConfig.mode           = Uintc_Mode_UINTC_MANAGED;
    gNetfpProxyMcb.uintcHandle = Uintc_init (&uintcConfig, &errCode);
    if (gNetfpProxyMcb.uintcHandle == NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to open the UINTC module\n");
        goto cleanup_return_error;
    }
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: UINTC module has been opened successfully.\n");

    /* Initialize the heap configuration. */
    memset ((void *)&heapCfg, 0, sizeof(Pktlib_HeapCfg));

    /* Populate the heap configuration
     *  - This heap is used for communication between NetFP server and Proxy (NetFP client) */
    snprintf (heapCfg.name, sizeof(heapCfg.name)-1, "%s_%s", gNetfpProxyMcb.netfpClientName, "Heap");
    heapCfg.memRegion                       = ptrResCfg->memRegionResponse[0].memRegionHandle;
    heapCfg.pktlibInstHandle                = gNetfpProxyMcb.pktlibInstHandle;
    heapCfg.sharedHeap                      = 0;
    heapCfg.useStarvationQueue              = 0;
    heapCfg.dataBufferSize                  = 1500;
    heapCfg.numPkts                         = 1; //fzm: after switching to AF_UNIX transport for josh, only 1 desc is needed just to satisfy validations
    heapCfg.numZeroBufferPackets            = 0;
    heapCfg.dataBufferPktThreshold          = 8;
    heapCfg.zeroBufferPktThreshold          = 0;
    heapCfg.heapInterfaceTable.dataMalloc   = NetfpProxy_coreDataMalloc;
    heapCfg.heapInterfaceTable.dataFree     = NetfpProxy_coreDataFree;
    heapCfg.linearAlloc = 1;
    gNetfpProxyMcb.netfpProxyHeap = Pktlib_createHeap(&heapCfg, &errCode);
    if (gNetfpProxyMcb.netfpProxyHeap == NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to create NETFP Proxy Heap [Error code %d]\n", errCode);
        goto cleanup_return_error;
    }

    //fzm-->
    /* Populate the heap configuration
     *  - This heap is used for L3 Qos */
    snprintf (l3QosHeapCfg.name, sizeof(l3QosHeapCfg.name)-1, "%s_%s", gNetfpProxyMcb.netfpClientName, "L3QosHeap");
    l3QosHeapCfg.memRegion                       = ptrResCfg->memRegionResponse[0].memRegionHandle;
    l3QosHeapCfg.pktlibInstHandle                = gNetfpProxyMcb.pktlibInstHandle;
    l3QosHeapCfg.sharedHeap                      = 0;
    l3QosHeapCfg.useStarvationQueue              = 0;
    l3QosHeapCfg.dataBufferSize                  = 2048;
    l3QosHeapCfg.numPkts                         = 4060;
    l3QosHeapCfg.numZeroBufferPackets            = 0;
    l3QosHeapCfg.dataBufferPktThreshold          = 8;
    l3QosHeapCfg.zeroBufferPktThreshold          = 0;
    l3QosHeapCfg.heapInterfaceTable.dataMalloc   = NetfpProxy_coreDataMalloc;
    l3QosHeapCfg.heapInterfaceTable.dataFree     = NetfpProxy_coreDataFree;
    l3QosHeapCfg.linearAlloc                     = 1;
    l3QosHeapHandle = Pktlib_createHeap(&l3QosHeapCfg, &errCode);
    if (l3QosHeapHandle == NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "ERROR: Unable to create shared l3 qos heap, error code %d\n", errCode);
        goto cleanup_return_error;
    }
    //fzm<--

    /* SYNC Point: Ensure that the NETFP Server has been started. This is required before
     * we create and register the NETFP client. */
    while (1)
    {
        /* Try to start the server: The server might not be ready. Since the NETFP Proxy and Server
         * execute in the same realm. The name client handle is specified as NULL. */
        gNetfpProxyMcb.netfpServerHandle = Netfp_startServer (gNetfpProxyMcb.databaseHandle,
                                                              NULL, gNetfpProxyMcb.netfpServerName,
                                                              &errCode);
        if (gNetfpProxyMcb.netfpServerHandle != NULL)
            break;

        /* Check the error code. */
        if (errCode != NETFP_ENOTREADY)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: NETFP Starting server failed [Error code %d]\n", errCode);
            goto cleanup_return_error;
        }
        usleep (1000);
    }

    /* Initialize the client configuration */
    memset ((void *)&clientConfig, 0, sizeof(Netfp_ClientConfig));

    /* Populate the client configuration.
     * - Agent Client Handle is set to NULL since the NETFP Server and clients are
     *   both executing in the same realm. */
    strncpy (clientConfig.serverName, gNetfpProxyMcb.netfpServerName, NETFP_MAX_CHAR-1);
    strncpy (clientConfig.clientName, gNetfpProxyMcb.netfpClientName, NETFP_MAX_CHAR-1);
    clientConfig.serverHandle                       = gNetfpProxyMcb.netfpServerHandle;
    clientConfig.nrInstanceId                       = gNetfpProxyMcb.nrInstanceId;
    clientConfig.msgcomInstHandle                   = gNetfpProxyMcb.msgcomInstHandle;
    clientConfig.pktlibInstHandle                   = gNetfpProxyMcb.pktlibInstHandle;
    clientConfig.nameClientHandle                   = NULL;
    clientConfig.netHeaderHeapHandle                = gNetfpProxyMcb.netfpProxyHeap;
    clientConfig.fragmentHeap                       = gNetfpProxyMcb.netfpProxyHeap;
    clientConfig.realm                              = Netfp_ExecutionRealm_ARM;
    clientConfig.clientHeapHandle                   = gNetfpProxyMcb.netfpProxyHeap;
    clientConfig.malloc                             = Netfp_osalMalloc;
    clientConfig.free                               = Netfp_osalFree;
    clientConfig.beginMemAccess                     = Netfp_osalBeginMemoryAccess;
    clientConfig.endMemAccess                       = Netfp_osalEndMemoryAccess;
    clientConfig.enterCS                            = Netfp_osalEnterSingleCoreCriticalSection;
    clientConfig.exitCS                             = Netfp_osalExitSingleCoreCriticalSection;
    clientConfig.createSem                          = Netfp_osalCreateSem;
    clientConfig.deleteSem                          = Netfp_osalDeleteSem;
    clientConfig.postSem                            = Netfp_osalPostSem;
    clientConfig.pendSem                            = Netfp_osalPendSem;

    /* Create the NETFP Client. */
    gNetfpProxyMcb.netfpClientHandle = Netfp_initClient (&clientConfig, &errCode);
    if (gNetfpProxyMcb.netfpClientHandle == NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: NETFP Client Initialization failed [Error code %d]\n", errCode);
        goto cleanup_return_error;
    }
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: NETFP Client '%s' Initialized %p\n",
                       clientConfig.clientName, gNetfpProxyMcb.netfpClientHandle);

    /* Loop around and synchronize the client registeration with the server. */
    while (1)
    {
        /* Get the client status. */
        clientStatus = Netfp_startClient (gNetfpProxyMcb.netfpClientHandle, &errCode);
        if (clientStatus < 0)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Client registeration status failed [Error code %d]\n", errCode);
            goto cleanup_return_error;
        }

        /* If the client has been registered; we can proceed */
        if (clientStatus == 1)
            break;

        /* Client has not been registered; wait for some time and try again */
        usleep (1000);
    }
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: NETFP Client has been registered successfully with the server\n");

    //fzm-->
    strcpy(flowCfg.name, "L3QosPassFlow");
    flowCfg.numHeaps = 1;
    flowCfg.heapHandle[0] = l3QosHeapHandle;
    flowCfg.sopOffset = 0;
    gL3QosPassFlowId = Netfp_createFlow(gNetfpProxyMcb.netfpClientHandle, &flowCfg, &errCode);
    if(gL3QosPassFlowId == 0) {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "ERROR: Unable to create l3 qos flow, error code %d\n", errCode);
        goto cleanup_return_error;
    }
    //fzm<--

    /* Done setting up NetFP Client */
    return 0;

cleanup_return_error:
    /* Delete the NetFP client */
    if (gNetfpProxyMcb.netfpClientHandle)
        Netfp_deleteClient (gNetfpProxyMcb.netfpClientHandle, &errCode);

    /* Deinit the UINTC instance */
    if (gNetfpProxyMcb.uintcHandle)
        Uintc_deinit (gNetfpProxyMcb.uintcHandle, &errCode);

    /* Deinit Msgcom instance */
    if (gNetfpProxyMcb.msgcomInstHandle)
        Msgcom_deleteInstance (gNetfpProxyMcb.msgcomInstHandle, &errCode);

    /* Delete the Pktlib heap */
    if (gNetfpProxyMcb.netfpProxyHeap)
        Pktlib_deleteHeap (gNetfpProxyMcb.pktlibInstHandle, gNetfpProxyMcb.netfpProxyHeap, &errCode);

    /* Delete the Pktlib instance */
    if (gNetfpProxyMcb.pktlibInstHandle)
        Pktlib_deleteInstance (gNetfpProxyMcb.pktlibInstHandle, &errCode);

    /* Clean up the Resource Manager context */
    if (gNetfpProxyMcb.sysCfgHandle)
        Resmgr_deinit (gNetfpProxyMcb.sysCfgHandle, &errCode);

    /* Clean up the Named Resource instance */
    if (gNetfpProxyMcb.databaseHandle)
        Name_deleteDatabase (gNetfpProxyMcb.databaseHandle, &errCode);

    /* Return error */
    return -1;
}

/**
 *  @b Description
 *  @n
 *      NetFP Proxy's exit function
 */
static void NetfpProxy_coreShutDown (void)
{
    int32_t errCode;

    /* If Proxy is already in reset? If so, do nothing. */
    if (gNetfpProxyMcb.bIsInReset)
        return;

    /* Modify Proxy state to indicate that Proxy is being shutdown.
     * This stops all further offload request/Network event processing by Proxy threads. */
    gNetfpProxyMcb.bIsInReset = 1;

    /* Stop and delete the NetFP client */
    Netfp_stopClient (gNetfpProxyMcb.netfpClientHandle, &errCode);

    /* Kill the NetFP, UINTC threads */
    pthread_cancel (gNetfpProxyMcb.netfpProxyThread);
    pthread_cancel (gNetfpProxyMcb.netfpClientThread);
    pthread_cancel (gNetfpProxyMcb.uintcThread);

    Netfp_deleteClient (gNetfpProxyMcb.netfpClientHandle, &errCode);

    pthread_join (gNetfpProxyMcb.netfpProxyThread, NULL);
    pthread_join (gNetfpProxyMcb.netfpClientThread, NULL);
    pthread_join (gNetfpProxyMcb.uintcThread, NULL);

    /* Deinit the UINTC instance */
    Uintc_deinit (gNetfpProxyMcb.uintcHandle, &errCode);

//fzm ---> https://e2eprivate.ti.com/nokia_siemens_networks/k2_-_fsm4_-_fzm_-_lrc_nokia/f/191/t/5791.aspx
    /* Deinitialize and Route module */
    NetfpProxy_routeDeinit();

    /* Cleanup the plugin context if registered */
    if (gNetfpProxyMcb.pluginCfg.exit)
        gNetfpProxyMcb.pluginCfg.exit ();

    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Proxy cleanup completed - shutting down");

    return; //do an early exit, because the cleanup sometimes generates coredump
    // and as we are exiting, the board is reseting anyway, so there's no need
    // to free descriptors, memory region or msgcom instance.
//<--- fzm
}

/**
 *  @b Description
 *  @n
 *      Entry point for the NetFP Proxy application.
 *
 *  @param[in]  argc
 *      Number of arguments passed to the application
 *  @param[in]  argv
 *      Arguments passed to the application.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t main (int argc, char* argv[])
{
    int32_t                 retVal;
    Netfp_ProxyCfg          proxyCfg;
    void*                   pluginPtr;
    int32_t                 (*plugin_init_fxn_ptr) (int32_t);

    /* Initialize the NetFP Proxy state info. */
    memset ((void *)&gNetfpProxyMcb, 0, sizeof (gNetfpProxyMcb));

    /* Setup the defaults:
     * - SA statistics collection update interval */
    gNetfpProxyMcb.statsUpdateInterval = 10;

    /* Read and parse command line options */
    if (NetfpProxy_coreParseCommandLineArgs (argc, argv) < 0)
    {
        /* Error: Unable to parse the command line arguments */
        NetfpProxy_usage();
        return -1;
    }

#if 0 // Below IPSecMgr implementation is not used by Nokia
    /* Initialize and start the IPSecMgr Snoop functionality. */
    if (NetfpProxy_ipsecInit() < 0)
        return -1;
#endif

    NetfpProxy_logInit(); //fzm

    if(sem_init(&shutdownSemaphore, 0, 0) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Failed to initialize shutdown semaphore: %s", strerror(errno));
        return -1;
    }

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: IPSEC Initialized\n");

    /* Initialize NETFP and all required system resources */
    if (NetfpProxy_coreInitNetFP () < 0)
        return -1;

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: NETFP Initialized\n");

    /* Initialize the core module: */
    if (pipe(gNetfpProxyMcb.corePipe) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to create the core pipe\n");
        return -1;
    }

    if(fcntl(gNetfpProxyMcb.corePipe[0], F_SETPIPE_SZ, 1048576) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable set pipe[0] size: %s", strerror(errno));
        return -1;
    }
    if(fcntl(gNetfpProxyMcb.corePipe[1], F_SETPIPE_SZ, 1048576) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable set pipe[1] size: %s", strerror(errno));
        return -1;
    }

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: NETFP Core Pipe %d %d\n", gNetfpProxyMcb.corePipe[0], gNetfpProxyMcb.corePipe[1]);

    /* Initialize the interface module */
    gNetfpProxyMcb.ifSocket = NetfpProxy_ifaceInit();
    if (gNetfpProxyMcb.ifSocket < 0)
        return -1;

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: NETFP Interface module assigned descriptor %d\n", gNetfpProxyMcb.ifSocket);

    /* Initialize the route module */
    gNetfpProxyMcb.routeSocket = NetfpProxy_routeInit();
    if (gNetfpProxyMcb.routeSocket < 0)
        return -1;

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: NETFP Route module assigned descriptor %d\n", gNetfpProxyMcb.routeSocket);

    /* Initialize the Neighbor management module: */
    gNetfpProxyMcb.neighSocket = NetfpProxy_neighInit();
    if (gNetfpProxyMcb.neighSocket < 0)
        return -1;

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: NETFP Neighbor module assigned descriptor %d\n", gNetfpProxyMcb.neighSocket);

    /* Enable signal handling */
    signal (SIGTERM, NetfpProxy_coreSignalHandler);
    signal (SIGINT,  NetfpProxy_coreSignalHandler);
    signal (SIGUSR1, NetfpProxy_coreSignalHandler);

    /*********************************************************************************************
     * Launch the threads:
     *  - NETFP Client & UINTC Thread should be operational before the PLUGIN is loaded because
     *    the plugin could invoke NETFP API.
     *********************************************************************************************/
    retVal = pthread_create (&gNetfpProxyMcb.uintcThread, NULL, NetfpProxy_coreExecuteUINTC, NULL);
    if (retVal < 0)
    {
        /* Clean up and exit */
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: UINTC thread create failed error code %d\n", retVal);
        NetfpProxy_coreShutDown ();
        return -1;
    }
    retVal = pthread_create (&gNetfpProxyMcb.netfpClientThread, NULL, NetfpProxy_coreExecuteNETFPClient, gNetfpProxyMcb.netfpClientHandle);
    if (retVal < 0)
    {
        /* Clean up and exit */
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: NETFP Client thread create failed error code %d\n", retVal);
        NetfpProxy_coreShutDown ();
        return -1;
    }

    /********************************************************************************************
     * Register as NETFP Proxy:
     ********************************************************************************************/
    memset ((void*)&proxyCfg, 0, sizeof(Netfp_ProxyCfg));
    proxyCfg.proxyServerInterfaceFunction =  NetfpProxy_coreProcessServerMsg;
    if (Netfp_registerProxyService(gNetfpProxyMcb.netfpClientHandle, &proxyCfg, &retVal) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Proxy registeration failed [Error code %d]\n", retVal);
        return -1;
    }
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: NETFP proxy has been registered successfully with the server\n");

    /********************************************************************************************
     * Initialize the PLUGIN:
     ********************************************************************************************/
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Loading plugin... \n");

    /* Load the Application plugin library. */
    pluginPtr = dlopen(gNetfpProxyMcb.pluginName, RTLD_GLOBAL | RTLD_NOW);
    if (pluginPtr == NULL)
    {
        NetfpProxy_logMsg(NETFP_PROXY_LOG_ERROR, "Unable to load plugin %s Error: %s.\n", gNetfpProxyMcb.pluginName, dlerror());
        NetfpProxy_coreShutDown ();
        return -1;
    }
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Loaded plugin... \n");

    /* Get the Initialization function handle */
    if ((plugin_init_fxn_ptr = dlsym (pluginPtr, "NetfpProxy_pluginInit")) == NULL)
    {
        NetfpProxy_logMsg(NETFP_PROXY_LOG_ERROR, "Error: Bad plugin! Can't find initialization function NetfpProxy_pluginInit()\n");
        NetfpProxy_coreShutDown ();
        return -1;
    }
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Invoking plugin initialization for %d Proxy\n", getpid());

    /* Initialize the Plugin. */
    gNetfpProxyMcb.pluginSocket = plugin_init_fxn_ptr (getpid());
    if (gNetfpProxyMcb.pluginSocket < 0)
    {
        NetfpProxy_logMsg(NETFP_PROXY_LOG_ERROR, "Plugin %s init failed. Error: %s\n", gNetfpProxyMcb.pluginName, dlerror());
        NetfpProxy_coreShutDown ();
        return -1;
    }
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Plugin Initialized\n");

    /*********************************************************************************************
     * Launch the NETFP Proxy Thread:
     *********************************************************************************************/
    retVal = pthread_create (&gNetfpProxyMcb.netfpProxyThread, NULL, NetfpProxy_coreRun, NULL);
    if (retVal < 0)
    {
        /* Clean up and exit */
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: NETFP Proxy thread create failed error code %d\n", retVal);
        NetfpProxy_coreShutDown ();
        return -1;
    }

    /***********************************************************************************************
     * Are we executing the *test* mode? This is a test mode which is used only for auto validating
     * the NETFP Proxy. This mode should NOT be executed by application developers and is used only
     * by the internal development teams.
     ***********************************************************************************************/
    if (gNetfpProxyMcb.testMode == 1)
    {
        pthread_t   testThread;
        extern void* NetfpProxy_test (void* arg);
        uint32_t     testArgument[2];

        /* Pass the test arguments: */
        testArgument[0] = (uint32_t)gNetfpProxyMcb.netfpClientHandle;
        testArgument[1] = (uint32_t)NetfpProxy_coreProcessServerMsg;

        /*********************************************************************************************
         * Launch the Test Thread:
         *********************************************************************************************/
        retVal = pthread_create (&testThread, NULL, NetfpProxy_test, &testArgument[0]);
        if (retVal < 0)
        {
            /* Clean up and exit */
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: NETFP Proxy thread create failed error code %d\n", retVal);
            return -1;
        }
    }

    sem_wait(&shutdownSemaphore);
    sem_destroy(&shutdownSemaphore);

    /* Clean up and exit */
    NetfpProxy_coreShutDown ();
    NetfpProxy_logDeInit(); //fzm

    return 0;
}


