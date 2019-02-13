/**
 *   @file  master.c
 *
 *   @brief
 *      The file implements the NETFP master. The NETCP is fundamentally
 *      a single IP block which could be shared across multiple NETFP
 *      domains (i.e. NETFP servers and NETFP clients) The NETFP master is
 *      a system application which is executed during  initialization time.
 *      The NETFP master is responsible for performing the following:-
 *          - Initialization of NETCP
 *          - eQoS configuration
 *          - Reassembly of all packets across multiple NETFP domains
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2014 Texas Instruments, Inc.
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
#include <stdint.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <getopt.h>

/* MCSDK Include files. */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/drv/qmss/qmss_firmware.h>
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
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/netfp/netfp.h>
#include <ti/runtime/name/name.h>
#include <ti/runtime/name/name_db.h>
#include <ti/runtime/name/name_proxyClient.h>
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/apps/netfp_master/include/netfp_master_internal.h>

/* Flag which defines which channel type is used for handling the reassembly. */
#define USE_ACCUMULATED_CHANNEL

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
extern void  Pktlib_osalSharedEnterCS(void);
extern void  Pktlib_osalSharedExitCS(void);

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

//fzm-->
extern int32_t NetfpMaster_logInit(void);
extern int32_t NetfpMaster_logDeInit(void);
extern void NetfpMaster_smlogMsg(Netfp_LogLevel logLevel, const char* fmt, va_list arg) __attribute__ ((format (printf, 2, 0)));
extern int  NetfpMaster_dumpInit(void);
extern void NetfpMaster_dumpDeInit(void);
extern void NetfpMaster_dumpMsg(Netfp_LogLevel logLevel, const char* fmt, va_list arg) __attribute__ ((format (printf, 2, 0)));
//fzm<--

/**************************************************************************
 ************************* Global Variables *******************************
 **************************************************************************/

/* Global MCB which keeps track of the NETFPD information: */
NetfpMaster_MCB      gNetfpMasterMCB;

/**************************************************************************
 ************************ NETFP Master Functions **************************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      Utility function which computes the maximum of numbers.
 *
 *  @param[in]  a
 *      First Number
 *  @param[in]  b
 *      Second Number
 *
 *  @retval
 *      Largest number
 */
static inline uint32_t max (uint32_t a, uint32_t b)
{
    return (a > b) ? a : b;
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
static uint8_t* NetfpMaster_heapDataMalloc(uint32_t size, uint32_t arg)
{
//fzm---->
    (void)arg;

    uint8_t* ptr = NULL;
    if(ddal_cma_alloc(DDAL_CMA_MEM_TYPE_DDR, size, (void *)&ptr) != DDAL_OK)
    {
        NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: Unable to allocate memory");
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
static void NetfpMaster_heapDataFree(uint8_t* ptr, uint32_t size, uint32_t arg)
{
    (void)ptr, (void)size; (void)arg; //fzm
}

/**
 *  @b Description
 *  @n
 *      The function is used to cleanup the MSGCOM data buffers if there are
 *      any pending packets while deleting the channel.
 *
 *  @param[in]  pktlibInstHandle
 *      PKTLIB Instance handle associated with the MSGCOM instance
 *  @param[in]  chHandle
 *      MSGCOM Channel Handle which is being deleted.
 *  @param[in]  msgBuffer
 *      MSGCOM Buffer to be deleted.
 *
 *  @retval
 *      Not Applicable.
 */
static void NetfpMaster_flushFragment
(
    Pktlib_InstHandle   pktlibInstHandle,
    MsgCom_ChHandle     chHandle,
    MsgCom_Buffer*      msgBuffer
)
{
    Pktlib_freePacket(gNetfpMasterMCB.pktlibInstanceHandle, (Ti_Pkt*)msgBuffer);
}

/**
 *  @b Description
 *  @n
 *     Display the usage for the resource manager server
 *
 *  @retval
 *      Not Applicable.
 */
static void NetfpMaster_displayUsage (void)
{
    printf ("NETFP Master:\n");
    printf ("netfp_master <arguments>\n");
    printf ("Mandatory Arguments: \n");
    printf ("-r <name>           - Name of the RM Client to be used; this should match the client names in the RMv2 Policy file\n");
    printf ("-c <netfp.conf>     - NETFP Master configuration file.\n");
    printf ("Optional Arguments: \n");
    printf ("-t <timeout>      - Polling Timeout specified in msec [Default is 100msec]\n");
    printf ("-v                - Verbosity\n");
    return;
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
void NetfpMaster_log (NetfpMaster_LogLevel level, const char* fmt, ...)
{
    va_list arg;

    /* Log the message as per the configured level. */
    if (level >= gNetfpMasterMCB.logLevel)
    {
        va_start (arg, fmt);
        NetfpMaster_smlogMsg(level, fmt, arg); //fzm
        va_end (arg);
    }
    return;
}
 // <fzm>
void NetfpMaster_dump (NetfpMaster_LogLevel level, const char* fmt, ...)
{
    va_list arg;

    va_start(arg, fmt);
    NetfpMaster_dumpMsg(level, fmt, arg);
    va_end(arg);

    return;
}
// </fzm>

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
static void NetfpMaster_terminated (int sig, siginfo_t *siginfo, void *context)
{
    uint32_t    terminate = 0xdead;

    /* Master is going down. */
    NetfpMaster_log(NetfpMaster_LogLevel_DEBUG, "Debug: Signal received\n");
    gNetfpMasterMCB.executionStatus = 0;

    /* Send a message via the pipe to indicate the same. */
    write (gNetfpMasterMCB.signalPipe[1], &terminate, sizeof(terminate));
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to setup the signal pipes which are used
 *      to communicate from the signal handlers.
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
static int32_t NetfpMaster_setupSignalPipe(void)
{
    int32_t     flags;

    /* Open the signal pipes */
    if (pipe (gNetfpMasterMCB.signalPipe) < 0)
    {
        NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: Unable to open signal pipe[%s]\n", strerror(errno));
        return -1;
    }

    /* Setup the read pipe to be non blocking */
    flags = fcntl(gNetfpMasterMCB.signalPipe[0], F_GETFL);
    if (flags == -1)
    {
        NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: Unable to get read signal pipe flags [%s]\n", strerror(errno));
        return -1;
    }
    flags |= O_NONBLOCK;
    if (fcntl(gNetfpMasterMCB.signalPipe[0], F_SETFL, flags) == -1)
    {
        NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: Unable to set read signal pipe flags [%s]\n", strerror(errno));
        return -1;
    }

    /* Setup the write pipe to be non blocking */
    flags = fcntl(gNetfpMasterMCB.signalPipe[1], F_GETFL);
    if (flags == -1)
    {
        NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: Unable to get write signal pipe flags [%s]\n", strerror(errno));
        return -1;
    }
    flags |= O_NONBLOCK;
    if (fcntl(gNetfpMasterMCB.signalPipe[1], F_SETFL, flags) == -1)
    {
        NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: Unable to set write signal pipe flags [%s]\n", strerror(errno));
        return -1;
    }
    NetfpMaster_log(NetfpMaster_LogLevel_DEBUG, "Debug: NETFP Master Signal Pipe configured\n");
    return 0;
}


/**
 *  @b Description
 *  @n
 *      Utility function which is used to get the number of flows configured
 *      for the interface.
 *
 *  @param[in]  ptrNetfpIfBlock
 *      Pointer to the NETFP master interface block
 *
 *  @retval
 *      Not applicable
 */
static uint32_t NetfpMaster_getNumFlows (NetfpMaster_IfBlock* ptrNetfpIfBlock)
{
    uint8_t index;
    uint8_t numFlows = 0;

    /* Cycle through all the possible flows */
    for (index = 0; index < NETFP_MASTER_MAX_FLOW; index++)
    {
        /* Flow block with 0 descriptors implies that this is an unspecified flow. */
        if (ptrNetfpIfBlock->flowBlock[index].numDescriptors != 0)
            numFlows++;
    }
    return numFlows;
}

/**
 *  @b Description
 *  @n
 *      Utility function which is used to display the NETFP Master interface block
 *
 *  @param[in]  ptrNetfpIfBlock
 *      Pointer to the NETFP master interface block
 *
 *  @retval
 *      Not applicable
 */
static void NetfpMaster_displayInterfaceBlock (NetfpMaster_IfBlock* ptrNetfpIfBlock)
{
    uint8_t             index;
    Pktlib_HeapStats    heapStats;

    /* Is QOS configured? */
    if (gNetfpMasterMCB.enableEQOS == 1)
    {
        /* YES: Enhanced QOS is enabled */
//fzm - use NetfpMaster_dump to capture this with the USR1 signal
        NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "***************************************************************\n");
        NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "Debug: Displaying Interface Block %p\n", ptrNetfpIfBlock);
        NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "Debug: Name              %s\n", ptrNetfpIfBlock->name);
        NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "Debug: Switch Port       %d\n", ptrNetfpIfBlock->switchPort);
        NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "Debug: Routing Mode      %s\n",
                        ptrNetfpIfBlock->routingMode == NetfpMaster_RoutingMode_DSCP ? "dscp" : "dp-bit");
        NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "Debug: Default Priority  %d\n", ptrNetfpIfBlock->defaultFwdPriority);
        NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "Debug: Priority Override %d\n", ptrNetfpIfBlock->priorityOverride);
        NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "Debug: Max Flow Offset   %d\n", ptrNetfpIfBlock->maxFlowOffset);
        NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "Debug: Max Queue Offset  %d\n", ptrNetfpIfBlock->maxQueueOffset);
        NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "Debug: Base Queue        %d\n", ptrNetfpIfBlock->baseQueue);

        /* Display the Flow configuration for each interface */
        NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "Debug: Flow configuration\n");
        for (index = 0; index < NETFP_MASTER_MAX_FLOW; index++)
        {
            if (ptrNetfpIfBlock->flowBlock[index].numDescriptors != 0)
            {
                NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "  Flow Offset %d -> Num of descriptors %d\n",
                                ptrNetfpIfBlock->flowBlock[index].flowOffset, ptrNetfpIfBlock->flowBlock[index].numDescriptors);

                /* Get the heap statistics */
                Pktlib_getHeapStats (ptrNetfpIfBlock->flowBlock[index].heapHandle, &heapStats);

                /* Display the heap statistics */
                NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "Debug: Number of free data packets    : %d\n", heapStats.numFreeDataPackets);
                NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "Debug: Data buffer starvation counter : %d\n", heapStats.dataBufferStarvationCounter);
            }
        }

        /* Display the DSCP mapping: */
        NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "Debug: DSCP QoS configuration\n");
        for (index = 0; index < NETFP_MASTER_MAX_DSCP_ENTRY; index++)
            NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "  DSCP %d -> Flow Offset %d Queue Offset %d Queue %d\n", index,
                            ptrNetfpIfBlock->dscpMap[index].flowOffset, ptrNetfpIfBlock->dscpMap[index].queueOffset,
                            (ptrNetfpIfBlock->dscpMap[index].queueOffset + ptrNetfpIfBlock->baseQueue));

        /* Display the VLAN Priority bit mapping: */
        NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "Debug: VLAN QoS configuration\n");
        for (index = 0; index < NETFP_MASTER_MAX_VLAN_ENTRY; index++)
            NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "  VLAN %d -> Flow Offset %d Queue Offset %d Queue %d\n", index,
                            ptrNetfpIfBlock->vlanMap[index].flowOffset, ptrNetfpIfBlock->vlanMap[index].queueOffset,
                            (ptrNetfpIfBlock->vlanMap[index].queueOffset + ptrNetfpIfBlock->baseQueue));
    }
    else
    {
        /* NO: Enhanced QOS is disabled */
        NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "***************************************************************\n");
        NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "Debug: Displaying Interface Block %p\n", ptrNetfpIfBlock);
        NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "Debug: Name             %s\n", ptrNetfpIfBlock->name);
    }

    /* Display the inner to outer DSCP marking: This is QOS independent. */
    NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "Debug: Inner to Outer DSCP configuration\n");
    for (index = 0; index < NETFP_MAX_SOCK_PRIORITY; index++)
        NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "  Inner DSCP %d -> Outer DSCP %d\n", index, ptrNetfpIfBlock->innerToOuterDSCPMap[index]);
    NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "***************************************************************\n");
    return;
}

/**
 *  @b Description
 *  @n
 *      Utility function which trims the spaces from the buffer
 *      is indicated by the file pointer.
 *
 *  @param[in]  ptrBuffer
 *      Pointer to the data buffer.
 *
 *  @retval
 *      Not applicable
 */
static void NetfpMaster_trimSpaces (char* ptrBuffer)
{
    int32_t     len;
    int32_t     index;

    /* Get the length of the string. */
    len = strlen (ptrBuffer);

    /* Initialize the index */
    index = 0;

    /* Cycle through the entire buffer */
    while (1)
    {
        /* Is this the end of the buffer? */
        if (*(ptrBuffer + index) == 0)
            return;

        /* Skip the spaces if present. */
        if ((*(ptrBuffer + index) != ' ') && (*(ptrBuffer + index) != '\t'))
        {
            index++;
            continue;
        }

        /* Move the contents of the entire buffer back by 1 character */
        for (; index <= (len - 1); index++)
            *(ptrBuffer + index) = *(ptrBuffer + index + 1);
        *(ptrBuffer + index) = 0;

        /* Reset the index and restart again: */
        index = 0;
    }
}

/**
 *  @b Description
 *  @n
 *      Utility function which extracts a token by parsing the buffer.
 *
 *  @param[in]  ptrInitialBuffer
 *      Pointer to the data buffer to be parsed for tokens
 *
 *  @retval
 *      Token
 */
static char* NetfpMaster_getToken (char* ptrInitialBuffer)
{
    const char*  delimitters = "<>=\n\r";
    char*  token;

    while (1)
    {
        /* Parse the token */
        token = strtok(ptrInitialBuffer, delimitters);
        if (token == NULL)
            return NULL;

        /* Trim out spaces and tabs from the token */
        NetfpMaster_trimSpaces(token);

        /* Did we trim everything out? */
        if (*token != 0)
            break;
    }
    return token;
}

/**
 *  @b Description
 *  @n
 *      Utility function which is used to get the tokens specified in the map sections
 *
 *  @param[out]  numTokens
 *      Number of tokens to be retrieved
 *  @param[out]  value1
 *      value1 specified in the configuration file
 *  @param[out]  value2
 *      value2 specified in the configuration file
 *  @param[out]  value3
 *      value3 specified in the configuration file
 *
 *  @retval
 *      Token
 */
static int32_t NetfpMaster_getMapTokens
(
    uint8_t     numTokens,
    uint32_t*   value1,
    uint32_t*   value2,
    uint32_t*   value3
)
{
    const char*  delimitters = "<>,\n\r ";
    char*  token;

    /* Sanity Check: Validate the number of arguments */
    if ((numTokens == 0) || (numTokens > 3))
        return -1;

    /* Parse the string to get the value1 */
    token = strtok(NULL, delimitters);
    if (token == NULL)
        return -1;

    /* Convert the token to the output value1: */
    *value1 = (uint32_t)atoi (token);

    /* Are we done? */
    if (numTokens == 1)
        return 0;

    /* Parse the string to get the value2 */
    token = strtok(NULL, delimitters);
    if (token == NULL)
        return -1;

    /* Convert the token to the output value2: */
    *value2 = (uint32_t)atoi (token);

    /* Are we done? */
    if (numTokens == 2)
        return 0;

    /* Parse the string to get the value3 */
    token = strtok(NULL, delimitters);
    if (token == NULL)
        return -1;

    /* Convert the token to the output value3: */
    *value3 = (uint32_t)atoi (token);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to parse the NETFP configuration file which
 *      is indicated by the file pointer.
 *
 *  @param[in]  configBuffer
 *      Data Buffer which has the NETFP configuration to be parsed
 *  @param[in]  configBufferSize
 *      Size of NETFP configuration file.
 *  @param[out] ptrNetfpConfig
 *      NETFP Configuration populated by this API.
 *
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t NetfpMaster_parseConfigFile(NetfpMaster_MCB* ptrNetfpMasterMCB)
{
    struct stat             buf;
    FILE*                   fpCfgFile;
    char*                   fileBuffer;
    char*                   ptrFileBuffer;
    char*                   tokenName;
    char*                   tokenValue;
    int32_t                 size;
    uint32_t                value1;
    uint32_t                value2;
    uint32_t                value3;
    NetfpMaster_IfBlock*    ptrNetfpIfBlock = NULL;

    /* Get the file statistics */
    if (stat(ptrNetfpMasterMCB->cfgFile, &buf) != 0)
    {
        NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: Unable to get the file statistics from %s\n", ptrNetfpMasterMCB->cfgFile);
        return -1;
    }

    /* Allocate memory for the file buffer: */
    fileBuffer = (char*)malloc (buf.st_size + 1);
    if (fileBuffer == NULL)
    {
        NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: Unable to allocate memory for the file buffer\n");
        return -1;
    }

    /* Initialize the file buffer. */
    memset ((void *)fileBuffer, 0, buf.st_size + 1);

    /* Open the configuration file: */
    fpCfgFile = fopen (ptrNetfpMasterMCB->cfgFile, "r");
    if (fpCfgFile == NULL)
    {
        NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: Unable to open the configuration file '%s'\n", ptrNetfpMasterMCB->cfgFile);
        return -1;
    }

    /* Initialize the variables: */
    ptrFileBuffer = fileBuffer;
    size          = 0;

    /* Read data from the file and place it into the buffer */
    while (1)
    {
        int ch = fgetc(fpCfgFile);
        if (ch == EOF)
            break;

        /* Place the data into the file buffer: */
        *fileBuffer++ = ch;
        size++;

        /* Did we reach the end of file? */
        if (size == buf.st_size)
            break;
    }
    *fileBuffer = 0;

    /* Once all the data has been placed into the buffer; reset the buffer pointer */
    fileBuffer = ptrFileBuffer;

    /* Close the file. */
    fclose (fpCfgFile);

    /* Initialize the variables */
    size = 0;

    /* Run through the entire file. */
    while (size < buf.st_size)
    {
        /* Get the token name: */
        tokenName = NetfpMaster_getToken (ptrFileBuffer);
        if (tokenName == NULL)
            break;

        /* Subsequent calls to the strtok API requires NULL parameters. */
        ptrFileBuffer = NULL;
        size = size + strlen(tokenName);

        /* Process and parse the buffer */
        if (*tokenName == '#' || *tokenName == '\n' || *tokenName == '\r')
            continue;

        /* Process the tokens */
        if (strcmp (tokenName, "reassembly_descriptors") == 0)
        {
            /* Sanity Check: Validate and ensure that the 'reassembly_descriptors' is NOT under a valid 'interface' block.
             * These parameters are NOT interface specific but are across the entire system. */
            if (ptrNetfpIfBlock != NULL)
            {
                /* FATAL Error: Parsing failure detected. Aborting */
                NetfpMaster_log(NetfpMaster_LogLevel_ERROR,
                                "Error: Parsing failure:: Keyword 'reassembly_descriptors' was specified inside the 'interface' block\n");
                return -1;
            }

            /* Get the token value */
            tokenValue = NetfpMaster_getToken (ptrFileBuffer);
            size = size + strlen(tokenValue);

            /* Get the number of reassembly descriptors. */
            ptrNetfpMasterMCB->numReassemblyDescriptors = atoi(tokenValue);
        }
        else if (strcmp (tokenName, "enable_eQOS") == 0)
        {
            /* Sanity Check: Validate and ensure that the 'enable_eQOS' is NOT under a valid 'interface' block.
             * These parameters are NOT interface specific but are across the entire system. */
            if (ptrNetfpIfBlock != NULL)
            {
                /* FATAL Error: Parsing failure detected. Aborting */
                NetfpMaster_log(NetfpMaster_LogLevel_ERROR,
                                "Error: Parsing failure:: Keyword 'enable_eQOS' was specified inside the 'interface' block\n");
                return -1;
            }

            /* Get the token value */
            tokenValue = NetfpMaster_getToken (ptrFileBuffer);
            size = size + strlen(tokenValue);

            /* Get the eQOS status. */
            ptrNetfpMasterMCB->enableEQOS = atoi(tokenValue);
//fzm-->
            if(ptrNetfpMasterMCB->forceDisableEQOS == 1u)
            {
                NetfpMaster_log(NetfpMaster_LogLevel_INFO, "Forced disabled eQoS - L2 SCT environment");
                ptrNetfpMasterMCB->enableEQOS = 0u;
            }
//fzm<--
        }
        else if (strcmp (tokenName, "frame_protocol_crc_offload") == 0)
        {
            /* Sanity Check: Validate and ensure that the 'frame_protocol_crc_offload' is NOT under a valid 'interface' block.
             * These parameters are NOT interface specific but are across the entire system. */
            if (ptrNetfpIfBlock != NULL)
            {
                /* FATAL Error: Parsing failure detected. Aborting */
                NetfpMaster_log(NetfpMaster_LogLevel_ERROR,
                                "Error: Parsing failure:: Keyword 'frame_protocol_crc_offload' was specified inside the 'interface' block\n");
                return -1;
            }

            /* Get the token value */
            tokenValue = NetfpMaster_getToken (ptrFileBuffer);
            size = size + strlen(tokenValue);

            /* Get the Frame Protocol CRC Offload status. */
            ptrNetfpMasterMCB->frameProtoCrcOffload = atoi(tokenValue);
        }
        else if (strcmp (tokenName, "reassembly_handling") == 0)
        {
            /* Sanity Check: Validate and ensure that the 'reassembly_handling' is NOT under a valid 'interface' block.
             * These parameters are NOT interface specific but are across the entire system. */
            if (ptrNetfpIfBlock != NULL)
            {
                /* FATAL Error: Parsing failure detected. Aborting */
                NetfpMaster_log(NetfpMaster_LogLevel_ERROR,
                                "Error: Parsing failure:: Keyword 'reassembly_handling' was specified inside the 'interface' block\n");
                return -1;
            }

            /* Get the token value */
            tokenValue = NetfpMaster_getToken (ptrFileBuffer);
            size = size + strlen(tokenValue);

            /* Get the reassembly handling status. */
            ptrNetfpMasterMCB->reassemblyHandling = atoi(tokenValue);
        }
        else if (strcmp (tokenName, "reassembly_timeout") == 0)
        {
            /* Sanity Check: Validate and ensure that the 'reassembly_timeout' is NOT under a valid 'interface' block.
             * These parameters are NOT interface specific but are across the entire system. */
            if (ptrNetfpIfBlock != NULL)
            {
                /* FATAL Error: Parsing failure detected. Aborting */
                NetfpMaster_log(NetfpMaster_LogLevel_ERROR,
                                "Error: Parsing failure:: Keyword 'reassembly_timeout' was specified inside the 'interface' block\n");
                return -1;
            }

            /* Get the token value */
            tokenValue = NetfpMaster_getToken (ptrFileBuffer);
            size = size + strlen(tokenValue);

            /* Get the reassembly timeout */
            ptrNetfpMasterMCB->reassemblyTimeout = atoi(tokenValue);
        }
        else if (strcmp (tokenName, "reassembly_context") == 0)
        {
            /* Sanity Check: Validate and ensure that the 'reassembly_context' is NOT under a valid 'interface' block.
             * These parameters are NOT interface specific but are across the entire system. */
            if (ptrNetfpIfBlock != NULL)
            {
                /* FATAL Error: Parsing failure detected. Aborting */
                NetfpMaster_log(NetfpMaster_LogLevel_ERROR,
                                "Error: Parsing failure:: Keyword 'reassembly_context' was specified inside the 'interface' block\n");
                return -1;
            }

            /* Get the token value */
            tokenValue = NetfpMaster_getToken (ptrFileBuffer);
            size = size + strlen(tokenValue);

            /* Get the number of reassembly descriptors. */
            ptrNetfpMasterMCB->numReassemblyContext = atoi(tokenValue);
        }
        else if (strcmp (tokenName, "default_host_priority") == 0)
        {
            /* Sanity Check: Validate and ensure that the 'default_host_priority' is NOT under a valid 'interface' block.
             * These parameters are NOT interface specific but are across the entire system. */
            if (ptrNetfpIfBlock != NULL)
            {
                /* FATAL Error: Parsing failure detected. Aborting */
                NetfpMaster_log(NetfpMaster_LogLevel_ERROR,
                                "Error: Parsing failure:: Keyword 'default_host_priority' was specified inside the 'interface' block\n");
                return -1;
            }

            /* Get the token value */
            tokenValue = NetfpMaster_getToken (ptrFileBuffer);
            size = size + strlen(tokenValue);

            /* Get the default host priority */
            ptrNetfpMasterMCB->defaultHostPriority = atoi(tokenValue);
        }
        else if (strcmp (tokenName, "interface_base_queue") == 0)
        {
            /* Sanity Check: Validate and ensure that the 'interface_base_Queue' is NOT under a valid 'interface' block.
             * These parameters are NOT interface specific but are across the entire system. */
            if (ptrNetfpIfBlock != NULL)
            {
                /* FATAL Error: Parsing failure detected. Aborting */
                NetfpMaster_log(NetfpMaster_LogLevel_ERROR,
                                "Error: Parsing failure:: Keyword 'interface_base_Queue' was specified inside the 'interface' block\n");
                return -1;
            }

            /* Get the token value */
            tokenValue = NetfpMaster_getToken (ptrFileBuffer);
            size = size + strlen(tokenValue);

            /* Get the eQOS status. */
            ptrNetfpMasterMCB->interfaceBaseQueue = atoi(tokenValue);
        }
        else if (strcmp (tokenName, "interface_base_flow") == 0)
        {
            /* Sanity Check: Validate and ensure that the 'interface_base_Flow' is NOT under a valid 'interface' block.
             * These parameters are NOT interface specific but are across the entire system. */
            if (ptrNetfpIfBlock != NULL)
            {
                /* FATAL Error: Parsing failure detected. Aborting */
                NetfpMaster_log(NetfpMaster_LogLevel_ERROR,
                                "Error: Parsing failure:: Keyword 'interface_base_Flow' was specified inside the 'interface' block\n");
                return -1;
            }

            /* Get the token value */
            tokenValue = NetfpMaster_getToken (ptrFileBuffer);
            size = size + strlen(tokenValue);

            /* Get the eQOS status. */
            ptrNetfpMasterMCB->interfaceBaseFlow = atoi(tokenValue);
        }
        else if (strcmp (tokenName, "interface") == 0)
        {
            /* Allocate the interface classfier block: */
            ptrNetfpIfBlock = (NetfpMaster_IfBlock*)malloc (sizeof(NetfpMaster_IfBlock));
            if (ptrNetfpIfBlock == NULL)
            {
                /* FATAL Error: */
                NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: Unable to allocate memory for the NETFP Interface block\n");
                return -1;
            }

            /* Initialize the interface classifier block: */
            memset ((void *)ptrNetfpIfBlock, 0, sizeof(NetfpMaster_IfBlock));

            /* The broadcast/multicast preclassification flow identifer is set to a non-zero value; since 0 is a valid flow identifier */
            ptrNetfpIfBlock->broadcastPreclassificationFlowId = 0xFFFFFFFF;
            ptrNetfpIfBlock->multicastPreclassificationFlowId = 0xFFFFFFFF;

            /* Priority override is set to a non-zero value since 0(Disabled) is a valid value. */
            ptrNetfpIfBlock->priorityOverride = 0xFFFFFFFF;
        }
        else if (strcmp (tokenName, "name") == 0)
        {
            /* Sanity Check: Validate and ensure that the 'name' was under a valid 'interface' block. */
            if (ptrNetfpIfBlock == NULL)
            {
                /* FATAL Error: Parsing failure detected. Aborting */
                NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: Parsing failure:: Keyword 'name' was specified outside the 'interface' block\n");
                return -1;
            }

            /* Get the token value */
            tokenValue = NetfpMaster_getToken (ptrFileBuffer);
            size = size + strlen(tokenValue);

            /* Populate the interface name into the NETFP Interface block */
            strcpy (ptrNetfpIfBlock->name, tokenValue);
        }
        else if (strcmp (tokenName, "base_queue") == 0)
        {
            /* Sanity Check: Validate and ensure that the 'base_queue' was under a valid 'interface' block. */
            if (ptrNetfpIfBlock == NULL)
            {
                /* FATAL Error: Parsing failure detected. Aborting */
                NetfpMaster_log(NetfpMaster_LogLevel_ERROR,
                                "Error: Parsing failure:: Keyword 'base_queue' was specified outside the 'interface' block\n");
                return -1;
            }

            /* Get the token value */
            tokenValue = NetfpMaster_getToken (ptrFileBuffer);
            size = size + strlen(tokenValue);

            /* Populate the base quueue into the NETFP Interface block */
            ptrNetfpIfBlock->baseQueue = atoi(tokenValue);
        }
        else if (strcmp (tokenName, "routing_mode") == 0)
        {
            /* Sanity Check: Validate and ensure that the 'routing_mode' was under a valid 'interface' block. */
            if (ptrNetfpIfBlock == NULL)
            {
                /* FATAL Error: Parsing failure detected. Aborting */
                NetfpMaster_log(NetfpMaster_LogLevel_ERROR,
                                "Error: Parsing failure:: Keyword 'routing_mode' was specified outside the 'interface' block\n");
                return -1;
            }

            /* Get the token value */
            tokenValue = NetfpMaster_getToken (ptrFileBuffer);
            size = size + strlen(tokenValue);

            /* Populate the routing mode into the NETFP Interface block */
            if (strcasecmp (tokenValue, "dscp") == 0)
            {
                /* Routing mode is DSCP */
                ptrNetfpIfBlock->routingMode = NetfpMaster_RoutingMode_DSCP;
            }
            else if (strcasecmp (tokenValue, "dp-bit") == 0)
            {
                /* Routing mode is DP-Bit */
                ptrNetfpIfBlock->routingMode = NetfpMaster_RoutingMode_DPBIT;
            }
            else
            {
                /* Error: Invalid routing algorithm specified */
                NetfpMaster_log(NetfpMaster_LogLevel_ERROR,
                                "Error: Parsing failure:: Invalid value %s detected in routing_mode for interface %s\n",
                                tokenValue, ptrNetfpIfBlock->name);
                return -1;
            }
        }
        else if (strcmp (tokenName, "priority_override") == 0)
        {
            /* Sanity Check: Validate and ensure that the 'priority_override' was under a valid 'interface' block. */
            if (ptrNetfpIfBlock == NULL)
            {
                /* FATAL Error: Parsing failure detected. Aborting */
                NetfpMaster_log(NetfpMaster_LogLevel_ERROR,
                                "Error: Parsing failure:: Keyword 'priority_override' was specified outside the 'interface' block\n");
                return -1;
            }

            /* Get the token value */
            tokenValue = NetfpMaster_getToken (ptrFileBuffer);
            size = size + strlen(tokenValue);

            /* Get the default priority. */
            ptrNetfpIfBlock->priorityOverride = atoi(tokenValue);
        }
        else if (strcmp (tokenName, "default_forwarding_priority") == 0)
        {
            /* Sanity Check: Validate and ensure that the 'default_forwarding_priority' was under a valid 'interface' block. */
            if (ptrNetfpIfBlock == NULL)
            {
                /* FATAL Error: Parsing failure detected. Aborting */
                NetfpMaster_log(NetfpMaster_LogLevel_ERROR,
                                "Error: Parsing failure:: Keyword 'default_forwarding_priority' was specified outside the 'interface' block\n");
                return -1;
            }

            /* Get the token value */
            tokenValue = NetfpMaster_getToken (ptrFileBuffer);
            size = size + strlen(tokenValue);

            /* Get the default priority. */
            ptrNetfpIfBlock->defaultFwdPriority = atoi(tokenValue);
        }
        else if (strcmp (tokenName, "broadcast_preclassification") == 0)
        {
            /* Sanity Check: Validate and ensure that the 'broadcast_preclassification' was under a valid 'interface' block. */
            if (ptrNetfpIfBlock == NULL)
            {
                /* FATAL Error: Parsing failure detected. Aborting */
                NetfpMaster_log(NetfpMaster_LogLevel_ERROR,
                                "Error: Parsing failure:: Keyword 'broadcast_preclassification' was specified outside the 'interface' block\n");
                return -1;
            }

            /* Get the token value */
            tokenValue = NetfpMaster_getToken (ptrFileBuffer);
            size = size + strlen(tokenValue);

            /* Get the broadcast preclassification status */
            ptrNetfpIfBlock->broadcastPreclassification = atoi(tokenValue);
        }
        else if (strcmp (tokenName, "broadcast_preclassification_flowId") == 0)
        {
            /* Sanity Check: Validate and ensure that the 'broadcast_preclassification_flowId' was under a valid 'interface' block. */
            if (ptrNetfpIfBlock == NULL)
            {
                /* FATAL Error: Parsing failure detected. Aborting */
                NetfpMaster_log(NetfpMaster_LogLevel_ERROR,
                                "Error: Parsing failure:: Keyword 'broadcast_preclassification_flowId' was specified outside the 'interface' block\n");
                return -1;
            }

            /* Get the token value */
            tokenValue = NetfpMaster_getToken (ptrFileBuffer);
            size = size + strlen(tokenValue);

            /* Get the broadcast preclassification flow identifier */
            ptrNetfpIfBlock->broadcastPreclassificationFlowId = atoi(tokenValue);
        }
        else if (strcmp (tokenName, "broadcast_preclassification_queueId") == 0)
        {
            /* Sanity Check: Validate and ensure that the 'broadcast_preclassification_queueId' was under a valid 'interface' block. */
            if (ptrNetfpIfBlock == NULL)
            {
                /* FATAL Error: Parsing failure detected. Aborting */
                NetfpMaster_log(NetfpMaster_LogLevel_ERROR,
                                "Error: Parsing failure:: Keyword 'broadcast_preclassification_queueId' was specified outside the 'interface' block\n");
                return -1;
            }

            /* Get the token value */
            tokenValue = NetfpMaster_getToken (ptrFileBuffer);
            size = size + strlen(tokenValue);

            /* Get the broadcast preclassification queue identifier */
            ptrNetfpIfBlock->broadcastPreclassificationQueueId = atoi(tokenValue);
        }
        else if (strcmp (tokenName, "multicast_preclassification") == 0)
        {
            /* Sanity Check: Validate and ensure that the 'multicast_preclassification' was under a valid 'interface' block. */
            if (ptrNetfpIfBlock == NULL)
            {
                /* FATAL Error: Parsing failure detected. Aborting */
                NetfpMaster_log(NetfpMaster_LogLevel_ERROR,
                                "Error: Parsing failure:: Keyword 'multicast_preclassification' was specified outside the 'interface' block\n");
                return -1;
            }

            /* Get the token value */
            tokenValue = NetfpMaster_getToken (ptrFileBuffer);
            size = size + strlen(tokenValue);

            /* Get the multicast preclassification status */
            ptrNetfpIfBlock->multicastPreclassification = atoi(tokenValue);
        }
        else if (strcmp (tokenName, "multicast_preclassification_flowId") == 0)
        {
            /* Sanity Check: Validate and ensure that the 'multicast_preclassification_flowId' was under a valid 'interface' block. */
            if (ptrNetfpIfBlock == NULL)
            {
                /* FATAL Error: Parsing failure detected. Aborting */
                NetfpMaster_log(NetfpMaster_LogLevel_ERROR,
                                "Error: Parsing failure:: Keyword 'multicast_preclassification_flowId' was specified outside the 'interface' block\n");
                return -1;
            }

            /* Get the token value */
            tokenValue = NetfpMaster_getToken (ptrFileBuffer);
            size = size + strlen(tokenValue);

            /* Get the multicast preclassification flow identifier */
            ptrNetfpIfBlock->multicastPreclassificationFlowId = atoi(tokenValue);
        }
        else if (strcmp (tokenName, "multicast_preclassification_queueId") == 0)
        {
            /* Sanity Check: Validate and ensure that the 'multicast_preclassification_queueId' was under a valid 'interface' block. */
            if (ptrNetfpIfBlock == NULL)
            {
                /* FATAL Error: Parsing failure detected. Aborting */
                NetfpMaster_log(NetfpMaster_LogLevel_ERROR,
                                "Error: Parsing failure:: Keyword 'multicast_preclassification_queueId' was specified outside the 'interface' block\n");
                return -1;
            }

            /* Get the token value */
            tokenValue = NetfpMaster_getToken (ptrFileBuffer);
            size = size + strlen(tokenValue);

            /* Get the multicast preclassification queue identifier */
            ptrNetfpIfBlock->multicastPreclassificationQueueId = atoi(tokenValue);
        }
        else if (strcmp (tokenName, "switch_port") == 0)
        {
            /* Sanity Check: Validate and ensure that the 'routing_mode' was under a valid 'interface' block. */
            if (ptrNetfpIfBlock == NULL)
            {
                /* FATAL Error: Parsing failure detected. Aborting */
                NetfpMaster_log(NetfpMaster_LogLevel_ERROR,
                                "Error: Parsing failure:: Keyword 'switch_port' was specified outside the 'interface' block\n");
                return -1;
            }

            /* Get the token value */
            tokenValue = NetfpMaster_getToken (ptrFileBuffer);
            size = size + strlen(tokenValue);

            /* Populate the switch port information: */
            ptrNetfpIfBlock->switchPort = atoi(tokenValue);
        }
        else if (strcmp (tokenName, "natt_dstPort") == 0)
        {
            /* Sanity Check: Validate and ensure that the 'natt_dstPort' is NOT under a valid 'interface' block.
             * These parameters are NOT interface specific but are across the entire system. */
            if (ptrNetfpIfBlock != NULL)
            {
                /* FATAL Error: Parsing failure detected. Aborting */
                NetfpMaster_log(NetfpMaster_LogLevel_ERROR,
                                "Error: Parsing failure:: Keyword 'natt_dstPort' was specified inside the 'interface' block\n");
                return -1;
            }

            /* Get the token value */
            tokenValue = NetfpMaster_getToken (ptrFileBuffer);
            size = size + strlen(tokenValue);

            /* Get the natt status. */
            ptrNetfpMasterMCB->nattCfg.udpPort = atoi(tokenValue);
        }
        else if (strcmp (tokenName, "natt_wildCardedEntry") == 0)
        {
            /* Sanity Check: Validate and ensure that the 'natt_dstPort' is NOT under a valid 'interface' block.
             * These parameters are NOT interface specific but are across the entire system. */
            if (ptrNetfpIfBlock != NULL)
            {
                /* FATAL Error: Parsing failure detected. Aborting */
                NetfpMaster_log(NetfpMaster_LogLevel_ERROR,
                                "Error: Parsing failure:: Keyword 'natt_wildCardedEntry' was specified inside the 'interface' block\n");
                return -1;
            }

            /* Get the token value */
            tokenValue = NetfpMaster_getToken (ptrFileBuffer);
            size = size + strlen(tokenValue);

            /* Get the natt status. */
            ptrNetfpMasterMCB->nattCfg.wildCardedEntry = atoi(tokenValue);
        }
        else if (strcmp (tokenName, "dscp_map") == 0)
        {
            /* Sanity Check: Validate and ensure that the 'dscp_map' was under a valid 'interface' block. */
            if (ptrNetfpIfBlock == NULL)
            {
                /* FATAL Error: Parsing failure detected. Aborting */
                NetfpMaster_log(NetfpMaster_LogLevel_ERROR,
                                "Error: Parsing failure:: Keyword 'dscp_map' was specified outside the 'interface' block\n");
                return -1;
            }

            /* Get the token value */
            if (NetfpMaster_getMapTokens (3, &value1, &value2, &value3) < 0)
            {
                NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: Get MAP Tokens parsing failed\n");
                return -1;
            }

            /* Sanity Check: Validate the arguments: */
            if (value1 > NETFP_MASTER_MAX_DSCP_ENTRY)
            {
                NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: Invalid DSCP value %d detected\n", value1);
                return -1;
            }

            /* Keep track of the flow and queue offset */
            ptrNetfpIfBlock->dscpMap[value1].flowOffset  = value2;
            ptrNetfpIfBlock->dscpMap[value1].queueOffset = value3;

            /* Keep track of the maximum flow and queue offset requested */
            if (ptrNetfpIfBlock->dscpMap[value1].flowOffset > ptrNetfpIfBlock->maxFlowOffset)
                ptrNetfpIfBlock->maxFlowOffset = ptrNetfpIfBlock->dscpMap[value1].flowOffset;
            if (ptrNetfpIfBlock->dscpMap[value1].queueOffset > ptrNetfpIfBlock->maxQueueOffset)
                ptrNetfpIfBlock->maxQueueOffset = ptrNetfpIfBlock->dscpMap[value1].queueOffset;

            continue;
        }
        else if (strcmp (tokenName, "vlan_map") == 0)
        {
            /* Sanity Check: Validate and ensure that the 'vlan_map' was under a valid 'interface' block. */
            if (ptrNetfpIfBlock == NULL)
            {
                /* FATAL Error: Parsing failure detected. Aborting */
                NetfpMaster_log(NetfpMaster_LogLevel_ERROR,
                                "Error: Parsing failure:: Keyword 'vlan_map' was specified outside the 'interface' block\n");
                return -1;
            }

            /* Get the token value */
            if (NetfpMaster_getMapTokens (3, &value1, &value2, &value3) < 0)
            {
                NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: Get MAP Tokens parsing failed\n");
                return -1;
            }

            /* Sanity Check: Validate the arguments: */
            if (value1 > NETFP_MASTER_MAX_VLAN_ENTRY)
            {
                NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: Invalid VLAN value %d detected\n", value1);
                return -1;
            }

            /* Keep track of the flow and queue offset */
            ptrNetfpIfBlock->vlanMap[value1].flowOffset  = value2;
            ptrNetfpIfBlock->vlanMap[value1].queueOffset = value3;

            /* Keep track of the maximum flow and queue offset requested */
            if (ptrNetfpIfBlock->vlanMap[value1].flowOffset > ptrNetfpIfBlock->maxFlowOffset)
                ptrNetfpIfBlock->maxFlowOffset = ptrNetfpIfBlock->vlanMap[value1].flowOffset;
            if (ptrNetfpIfBlock->vlanMap[value1].queueOffset > ptrNetfpIfBlock->maxQueueOffset)
                ptrNetfpIfBlock->maxQueueOffset = ptrNetfpIfBlock->vlanMap[value1].queueOffset;

            continue;
        }
        else if (strcmp (tokenName, "innerToOuterDSCP_map") == 0)
        {
            /* Sanity Check: Validate and ensure that the 'innerToOuterDSCP_map' was under a valid 'interface' block. */
            if (ptrNetfpIfBlock == NULL)
            {
                /* FATAL Error: Parsing failure detected. Aborting */
                NetfpMaster_log(NetfpMaster_LogLevel_ERROR,
                                "Error: Parsing failure:: Keyword 'innerToOuterDSCP_map' was specified outside the 'interface' block\n");
                return -1;
            }

            /* Get the token value */
            if (NetfpMaster_getMapTokens (2, &value1, &value2, &value3) < 0)
            {
                NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: Get MAP Tokens parsing failed\n");
                return -1;
            }

            /* Sanity Check: Validate the arguments: */
            if (value1 > NETFP_MAX_SOCK_PRIORITY)
            {
                NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: Invalid NETFP socket priority value %d detected\n", value1);
                return -1;
            }

            /* Setup the inner to outer DSCP map */
            ptrNetfpIfBlock->innerToOuterDSCPMap[value1] = value2;
            continue;
        }
        else if (strcmp (tokenName, "flow_configuration") == 0)
        {
            /* Sanity Check: Validate and ensure that the 'flow_configuration' was under a valid 'interface' block. */
            if (ptrNetfpIfBlock == NULL)
            {
                /* FATAL Error: Parsing failure detected. Aborting */
                NetfpMaster_log(NetfpMaster_LogLevel_ERROR,
                                "Error: Parsing failure:: Keyword 'flow_configuration' was specified outside the 'interface' block\n");
                return -1;
            }

            /* Get the token value */
            if (NetfpMaster_getMapTokens (2, &value1, &value2, &value3) < 0)
            {
                NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: Get MAP Tokens parsing failed\n");
                return -1;
            }

            /* Sanity Check: Validate the flow offset and ensure that it does not exceed the MAX limits */
            if (value1 > NETFP_MASTER_MAX_FLOW)
            {
                NetfpMaster_log(NetfpMaster_LogLevel_ERROR,
                                "Error: Flow offset %d exceeds max allowed for interface %s\n", value1, ptrNetfpIfBlock->name);
                return -1;
            }
            if (value2 == 0)
            {
                NetfpMaster_log(NetfpMaster_LogLevel_ERROR,
                                "Error: Number of descriptors is 0 for flow offset %d interface %s\n", value1, ptrNetfpIfBlock->name);
                return -1;
            }

            /* Use the flow offset to determine which flow block is to be used: */
            ptrNetfpIfBlock->flowBlock[value1].flowOffset     = value1;
            ptrNetfpIfBlock->flowBlock[value1].numDescriptors = value2;
        }
        else if (strcmp (tokenName, "enableQueueBounce") == 0)
        {
            /* Get the token value */
            tokenValue = NetfpMaster_getToken (ptrFileBuffer);
            size = size + strlen(tokenValue);

            ptrNetfpMasterMCB->paQueueBounceConfig.enable = atoi(tokenValue);

        }
        else if (strcmp (tokenName, "hwQueueBegin") == 0)
        {
            /* Get the token value */
            tokenValue = NetfpMaster_getToken (ptrFileBuffer);
            size = size + strlen(tokenValue);

            ptrNetfpMasterMCB->paQueueBounceConfig.hwQueueBegin = atoi(tokenValue);

        }
        else if (strcmp (tokenName, "hwQueueEnd") == 0)
        {
            /* Get the token value */
            tokenValue = NetfpMaster_getToken (ptrFileBuffer);
            size = size + strlen(tokenValue);

            ptrNetfpMasterMCB->paQueueBounceConfig.hwQueueEnd = atoi(tokenValue);

        }
        else if (strcmp (tokenName, "routeCmdRet") == 0)
        {
            /* Get the token value */
            tokenValue = NetfpMaster_getToken (ptrFileBuffer);
            size = size + strlen(tokenValue);

            ptrNetfpMasterMCB->paQueueBounceConfig.defOp[pa_QUEUE_BOUNCE_ROUTING_CLASS_CMD_RET] = atoi(tokenValue);

        }
        else if (strcmp (tokenName, "routeQoS") == 0)
        {
            /* Get the token value */
            tokenValue = NetfpMaster_getToken (ptrFileBuffer);
            size = size + strlen(tokenValue);

            ptrNetfpMasterMCB->paQueueBounceConfig.defOp[pa_QUEUE_BOUNCE_ROUTING_CLASS_QoS] = atoi(tokenValue);

        }
        else if (strcmp (tokenName, "routeCapture") == 0)
        {
            /* Get the token value */
            tokenValue = NetfpMaster_getToken (ptrFileBuffer);
            size = size + strlen(tokenValue);

            ptrNetfpMasterMCB->paQueueBounceConfig.defOp[pa_QUEUE_BOUNCE_ROUTING_CLASS_CAPTURE] = atoi(tokenValue);

        }
        else if (strcmp (tokenName, "routeIPReassembly") == 0)
        {
            /* Get the token value */
            tokenValue = NetfpMaster_getToken (ptrFileBuffer);
            size = size + strlen(tokenValue);

            ptrNetfpMasterMCB->paQueueBounceConfig.defOp[pa_QUEUE_BOUNCE_ROUTING_CLASS_IP_REASSEMBLY] = atoi(tokenValue);

        }
        else if (strcmp (tokenName, "routeMisc") == 0)
        {
            /* Get the token value */
            tokenValue = NetfpMaster_getToken (ptrFileBuffer);
            size = size + strlen(tokenValue);

            ptrNetfpMasterMCB->paQueueBounceConfig.defOp[pa_QUEUE_BOUNCE_ROUTING_CLASS_MISC] = atoi(tokenValue);

        }
        else if (strcmp (tokenName, "{") == 0)
        {
            continue;
        }
        else if (strcmp (tokenName, "}") == 0)
        {
            uint8_t numFlows;

            /* Sanity Check: Interface configuration block ends: Ensure that the interface configuration was started */
            if (ptrNetfpIfBlock == NULL)
            {
                /* FATAL Error: Parsing failure detected. Aborting */
                NetfpMaster_log(NetfpMaster_LogLevel_ERROR,
                                "Error: Parsing failure:: Interface block end detected without a block start\n");
                return -1;
            }

            /* Sanity Check: Validate all the arguments required by the interface block.
             * We should have a valid interface name. */
            if (ptrNetfpIfBlock->name[0] == 0)
            {
                NetfpMaster_log(NetfpMaster_LogLevel_ERROR,
                                "Error: Interface block detected without a valid name\n");
                return -1;
            }

            /* Sanity Check: Interface switch port should always be non-zero. */
            if (ptrNetfpIfBlock->switchPort == 0)
            {
                NetfpMaster_log(NetfpMaster_LogLevel_ERROR,
                                "Error: Interface %s has an invalid switch port configuration\n", ptrNetfpIfBlock->name);
                return -1;
            }

            /* Sanity Check: Ensure that a valid routing mode was specified in the configuration file. */
            if (ptrNetfpIfBlock->routingMode == 0)
            {
                NetfpMaster_log(NetfpMaster_LogLevel_ERROR,
                                "Error: Interface %s has an invalid routing mode configuration\n",
                                ptrNetfpIfBlock->name);
                return -1;
            }

            /* Sanity Check: Ensure that if the routing mode is set to dp-bit then the priority override is configured */
            if (ptrNetfpIfBlock->routingMode == NetfpMaster_RoutingMode_DPBIT)
            {
                if (ptrNetfpIfBlock->priorityOverride == 0xFFFFFFFF)
                {
                    NetfpMaster_log(NetfpMaster_LogLevel_ERROR,
                                    "Error: Interface %s operating in DP-BIT routing mode does not specify priority override\n",
                                    ptrNetfpIfBlock->name);
                    return -1;
                }
            }

            /* Sanity Check: Ensure that a valid base queue was specified in the configuration file. */
            if (ptrNetfpIfBlock->baseQueue == 0)
            {
                NetfpMaster_log(NetfpMaster_LogLevel_ERROR,
                                "Error: Interface %s does not specify the base queue\n",
                                ptrNetfpIfBlock->name);
                return -1;
            }

            /* Sanity Check: Was broadcast preclassification enabled for the interface? */
            if (ptrNetfpIfBlock->broadcastPreclassification == 1)
            {
                /* YES. Ensure that a valid flow and queue identifier was specified */
                if (ptrNetfpIfBlock->broadcastPreclassificationFlowId == 0xFFFFFFFF)
                {
                    NetfpMaster_log(NetfpMaster_LogLevel_ERROR,
                                    "Error: Interface %s does not specify a broadcast preclassification flow identifier\n",
                                    ptrNetfpIfBlock->name);
                    return -1;
                }
                if (ptrNetfpIfBlock->broadcastPreclassificationQueueId == 0)
                {
                    NetfpMaster_log(NetfpMaster_LogLevel_ERROR,
                                    "Error: Interface %s does not specify a broadcast preclassification queue identifier\n",
                                    ptrNetfpIfBlock->name);
                    return -1;
                }

                /* Enable the global system wide preclassification */
                gNetfpMasterMCB.enableGlobalPreclassification = 1;
            }

            /* Sanity Check: Was multicast preclassification enabled for the interface? */
            if (ptrNetfpIfBlock->multicastPreclassification == 1)
            {
                /* YES. Ensure that a valid flow and queue identifier was specified */
                if (ptrNetfpIfBlock->multicastPreclassificationFlowId == 0xFFFFFFFF)
                {
                    NetfpMaster_log(NetfpMaster_LogLevel_ERROR,
                                    "Error: Interface %s does not specify a multicast preclassification flow identifier\n",
                                    ptrNetfpIfBlock->name);
                    return -1;
                }
                if (ptrNetfpIfBlock->multicastPreclassificationQueueId == 0)
                {
                    NetfpMaster_log(NetfpMaster_LogLevel_ERROR,
                                    "Error: Interface %s does not specify a multicast preclassification queue identifier\n",
                                    ptrNetfpIfBlock->name);
                    return -1;
                }

                /* Enable the global system wide preclassification */
                gNetfpMasterMCB.enableGlobalPreclassification = 1;
            }

            /* Sanity Check: We should have at least 1 flow configuration specified */
            numFlows = NetfpMaster_getNumFlows(ptrNetfpIfBlock);
            if (numFlows == 0)
            {
                NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: Interface %s has no flows specified\n", ptrNetfpIfBlock->name);
                return -1;
            }

            /* Sanity Check: Ensure that the number of flows configured in the interface matches the flow offset
             * The maxFlowOffset is the max. flow offset value detected in the dscp_map/vlan_map and this should
             * always be +1 greater than the number of flows specified in the */
            if (ptrNetfpIfBlock->maxFlowOffset > (numFlows - 1))
            {
                NetfpMaster_log(NetfpMaster_LogLevel_ERROR,
                                "Error: Interface %s using invalid flow offset in the dscp/vlan map [Valid Range is 0 to %d] Detected %d\n",
                                ptrNetfpIfBlock->name, (numFlows - 1), ptrNetfpIfBlock->maxFlowOffset);
                return -1;
            }

            /* Keep track of the NETFP interface blocks: */
            NetfpMaster_listAdd ((NetfpMaster_ListNode**)&gNetfpMasterMCB.ptrInterfaceList, (NetfpMaster_ListNode*)ptrNetfpIfBlock);

            /* Interface block work is over: Reset the pointer */
            ptrNetfpIfBlock = NULL;
        }
        else
        {
            NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: Parsing error; invalid token '%s' detected\n", tokenName);
            return -1;
        }
    }

    /* Sanity Check: Validate interfaceBaseQueue settings. */
    if (ptrNetfpMasterMCB->interfaceBaseQueue == 0)
    {
        NetfpMaster_log(NetfpMaster_LogLevel_ERROR,
                       "Error: Interface Base Queue is not valid\n");
        return -1;
    }
    /* Sanity Check: Validate interfaceBaseFlow settings. */
    if (ptrNetfpMasterMCB->interfaceBaseFlow == 0)
    {
         NetfpMaster_log(NetfpMaster_LogLevel_ERROR,
                        "Error: Interface Base Flow is not valid\n");
         return -1;
    }

    /* Was reassembly handling offloaded to the NETFP master? */
    if (ptrNetfpMasterMCB->reassemblyHandling == 1)
    {
        /* Sanity Check: Ensure that all the other reassembly parameters have also been specified */
        if (ptrNetfpMasterMCB->numReassemblyDescriptors == 0)
        {
            NetfpMaster_log(NetfpMaster_LogLevel_ERROR,
                            "Error: Missing configuration for 'reassembly_descriptors' in the configuration file\n");
            return -1;
        }
        if (ptrNetfpMasterMCB->reassemblyTimeout == 0)
        {
            NetfpMaster_log(NetfpMaster_LogLevel_ERROR,
                            "Error: Missing configuration for 'reassembly_timeout' in the configuration file\n");
            return -1;
        }
        if (ptrNetfpMasterMCB->numReassemblyContext == 0)
        {
            NetfpMaster_log(NetfpMaster_LogLevel_ERROR,
                            "Error: Missing configuration for 'reassembly_context' in the configuration file\n");
            return -1;
        }
    }
    return 0;
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
static void NetfpMaster_displayInfo (int sig, siginfo_t *siginfo, void *context)
{
    NetfpMaster_IfBlock*    ptrNetfpIfBlock;
    int32_t                 errCode;
    paSysStats_t            paStats;
    Netfp_ReassemblyStats   reassemblyStats;
    NetfpMaster_Entity*     ptrEntity;

//fzm-->
    if(NetfpMaster_dumpInit() == -1)
        return;
//fzm<--

//fzm - use NetfpMaster_dump to capture this with the USR1 signal
    NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "***************************************************************\n");
    NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "Debug: Default Host Priority for all non-ip packets :%d\n", gNetfpMasterMCB.defaultHostPriority);
    NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "Debug: Enable Frame Protocol CRC Offload :%s\n",
                    (gNetfpMasterMCB.frameProtoCrcOffload) ? "Enabled" : "Disabled");

    /* Get the head of all the detected interfaces */
    ptrNetfpIfBlock = (NetfpMaster_IfBlock*)NetfpMaster_listGetHead((NetfpMaster_ListNode**)&gNetfpMasterMCB.ptrInterfaceList);
    while (ptrNetfpIfBlock != NULL)
    {
        /* Display the interface block: */
        NetfpMaster_displayInterfaceBlock(ptrNetfpIfBlock);

        /* Get the next interface block */
        ptrNetfpIfBlock = (NetfpMaster_IfBlock*)NetfpMaster_listGetNext((NetfpMaster_ListNode*)ptrNetfpIfBlock);
    }

    /* Is the master handling the reassembly services? */
    if (gNetfpMasterMCB.reassemblyHandling == 1)
    {
        /* YES. Get the reassembly statistics */
        if (Netfp_getReassemblyStats (gNetfpMasterMCB.netfpClientHandle, &reassemblyStats, &errCode) < 0)
            NetfpMaster_dump(NetfpMaster_LogLevel_ERROR, "Error: Getting reassembly stats failed [Error code %d]\n", errCode);
        else
        {
            NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "***************************************************************\n");
            NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "Debug: Reassembly Stats\n");
            NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "Debug: Active Reassembly Contexts              : %d\n", reassemblyStats.activeReassemblyContexts);
            NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "Debug: Outer IP fragments                      : %d\n", reassemblyStats.numOuterIPPktReceived);
            NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "Debug: Inner IP fragments                      : %d\n", reassemblyStats.numInnerIPPktReceived);
            NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "Debug: Active fragments                        : %d\n", reassemblyStats.numActiveFragments);
            NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "Debug: Non accelerated packets                 : %d\n", reassemblyStats.nonAccleratedTrafficFlow);
            NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "Debug: IPv4 fragments                          : %d\n", reassemblyStats.numIPv4Fragments);
            NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "Debug: IPv6 fragments                          : %d\n", reassemblyStats.numIPv6Fragments);
            NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "Debug: Reassembled packets                     : %d\n", reassemblyStats.numReassembledPackets);
            NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "Debug: Reassembly timeouts                     : %d\n", reassemblyStats.numReassemblyTimeout);
            NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "Debug: Duplicated fragments                    : %d\n", reassemblyStats.numDuplicatedFragment);
            NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "Debug: Overlapping fragments                   : %d\n", reassemblyStats.numOverlappingFragment);
            NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "Debug: Corrupted packets                       : %d\n", reassemblyStats.numCorruptedPackets);
            NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "Debug: IPv4 Header Error                       : %d\n", reassemblyStats.numIPv4HeaderError);
            NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "Debug: IPv6 Header Error                       : %d\n", reassemblyStats.numIPv6HeaderError);
            NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "Debug: Pre-outer fragments rejected            : %d\n", reassemblyStats.numPreOuterFragmentsRejected);
            NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "Debug: Pre-inner fragments rejected            : %d\n", reassemblyStats.numPreInnerFragmentsRejected);
            NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "Debug: Post-outer reassembled packets rejected : %d\n", reassemblyStats.numPostOuterReassembledPktsRejected);
            NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "Debug: Post-inner reassembled packets rejected : %d\n", reassemblyStats.numPostInnerReassembledPktsRejected);
            NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "Debug: Large packets                           : %d\n", reassemblyStats.numLargePackets);
            NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "Debug: Reassembly Heap Starvation Counter      : %d\n", reassemblyStats.reassemblyHeapStarvationCounter);
            NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "Debug: Number of TF not deleted                : %d\n", reassemblyStats.numFreeTrafficFlowFailure);
            NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "Debug: Fragments dropped- No Reassembly Context: %d\n", reassemblyStats.noReassemblyContext);
            NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "Debug: Traffic management Invocations          : %d\n", reassemblyStats.numDefaultReassemblyMgmtInvocations);
            NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "Debug: Fragments dropped - L2 Header Error     : %d\n", reassemblyStats.numL2HeaderError);
            NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "Debug: Fragments dropped - In Port Error       : %d\n", reassemblyStats.numInPortError);
            NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "***************************************************************\n");
        }
    }

    /* Get the list of Ethernet Rules added in netfp master */
    if (Netfp_displayEthRule (gNetfpMasterMCB.netfpServerHandle) < 0)
        NetfpMaster_dump(NetfpMaster_LogLevel_ERROR, "Error: Unable to display the Ethernet Rule \n") ;

    /* Display the Reassembly context */
    Netfp_displayReassemblyContext(gNetfpMasterMCB.netfpServerHandle);

    /* Get a list of all the registered entities and display them */
    NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "***************************************************************\n");
    NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "Registered Entity List:\n");
    ptrEntity = (NetfpMaster_Entity*)NetfpMaster_listGetHead((NetfpMaster_ListNode**)&gNetfpMasterMCB.ptrEntityList);
    while (ptrEntity != NULL)
    {
        /* Do we have a match */
        NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "-> %s\n", ptrEntity->address.sun_path);

        /* Get the next entity. */
        ptrEntity = (NetfpMaster_Entity*)NetfpMaster_listGetNext((NetfpMaster_ListNode*)ptrEntity);
    }
    NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "***************************************************************\n");

    /* Display the NETCP Statistics: */
    if (Netfp_getNETCPStats(gNetfpMasterMCB.netfpServerHandle, &paStats, &errCode) < 0)
        NetfpMaster_dump(NetfpMaster_LogLevel_ERROR, "Error: Unable to get the NETCP Statistics [Error code %d]\n", errCode); //fzm
    else
    {
        /* Display the NETCP Statistics: */
        NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "**********************************************************\n");
        NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "C1 number of packets:             %"PRIu32"\n", paStats.classify1.nPackets);
        NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "C1 number IPv4 packets:           %"PRIu32"\n", paStats.classify1.nIpv4Packets);
        NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "C1 number Inner IPv4 packets:     %"PRIu32"\n", paStats.classify1.nIpv4PacketsInner);
        NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "C1 number IPv6 packets:           %"PRIu32"\n", paStats.classify1.nIpv6Packets);
        NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "C1 number Inner IPv6 packets:     %"PRIu32"\n", paStats.classify1.nIpv6PacketsInner);
        NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "C1 number custom packets:         %"PRIu32"\n", paStats.classify1.nCustomPackets);
        NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "C1 number SRIO packets :          %"PRIu32"\n", paStats.classify1.nSrioPackets);
        NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "C1 number llc/snap fail:          %"PRIu32"\n", paStats.classify1.nLlcSnapFail);
        NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "C1 number table matched:          %"PRIu32"\n", paStats.classify1.nTableMatch);
        NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "C1 number failed table matched:   %"PRIu32"\n", paStats.classify1.nNoTableMatch);
        NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "C1 number IP frags:               %"PRIu32"\n", paStats.classify1.nIpFrag);
        NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "C1 number IP depth overflow:      %"PRIu32"\n", paStats.classify1.nIpDepthOverflow);
        NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "C1 number vlan depth overflow:    %"PRIu32"\n", paStats.classify1.nVlanDepthOverflow);
        NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "C1 number gre depth overflow:     %"PRIu32"\n", paStats.classify1.nGreDepthOverflow);
        NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "C1 number mpls packets:           %"PRIu32"\n", paStats.classify1.nMplsPackets);
        NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "C1 number of parse fail:          %"PRIu32"\n", paStats.classify1.nParseFail);
        NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "C1 number invalid IPv6 opts:      %"PRIu32"\n", paStats.classify1.nInvalidIPv6Opt);
        NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "C1 number of silent discard:      %"PRIu32"\n", paStats.classify1.nSilentDiscard);
        NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "C1 number of invalid control:     %"PRIu32"\n", paStats.classify1.nInvalidControl);
        NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "C1 number of invalid states:      %"PRIu32"\n", paStats.classify1.nInvalidState);
        NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "C1 number of system fails:        %"PRIu32"\n", paStats.classify1.nSystemFail);
        NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "C2 number of packets:             %"PRIu32"\n", paStats.classify2.nPackets);
        NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "C2 number of UDP packets:         %"PRIu32"\n", paStats.classify2.nUdp);
        NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "C2 number of TCP packets:         %"PRIu32"\n", paStats.classify2.nTcp);
        NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "C2 number of custom packets:      %"PRIu32"\n", paStats.classify2.nCustom);
        NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "C2 number of silent discard:      %"PRIu32"\n", paStats.classify2.nSilentDiscard);
        NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "C2 number of invalid control:     %"PRIu32"\n", paStats.classify2.nInvalidControl);
        NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "Modify number of command file:    %"PRIu32"\n", paStats.modify.nCommandFail);
    }
    /* fzm - This string is appended at the end of the log to enable collection with FPControl app */
    NetfpMaster_dump(NetfpMaster_LogLevel_INFO, "*******************************************</netfp_master_dump>\n");
    NetfpMaster_dumpDeInit();
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
static int32_t NetfpMaster_processCmdLineArgs(int32_t argc, char* argv[])
{
    while (1)
    {
        int option_index = 0;
        int c;

        static struct option long_options[] = {
            {"rmClientName",    required_argument, 0,  'r' }, //fzm
            {"cfgFile",         required_argument, 0,  'c' }, //fzm
            {"instantId",       required_argument, 0,  'i' }, //fzm
            {"l2sctenv",        no_argument,       0,  'l' }, //fzm
            {0,                 0,                 0,  0 }
        };

        c = getopt_long(argc, argv, "v:r:c:i:l", long_options, &option_index); //fzm
        if (c == -1)
            break;

       switch (c)
       {
            case 'r':
            {
                /* RM Client Name: */
                strncpy (gNetfpMasterMCB.rmClientName, optarg, sizeof(gNetfpMasterMCB.rmClientName));
                break;
            }
            case 'c':
            {
                /* Configuration file name */
                strcpy (gNetfpMasterMCB.cfgFile, optarg);
                break;
            }
            case 'i':
            {
                /* Named resource instance identifier. */
                gNetfpMasterMCB.nrInstanceId = atoi (optarg);
                break;
            }
//fzm-->
            case 'l':
            {
               /* Force eQoS disable (in L2 SCT env). */
               gNetfpMasterMCB.forceDisableEQOS = 1u;
               break;
            }
//fzm<--
            case 'v':
            {
//fzm-->
               if (!strcmp("VRB", optarg))
               {
                  gNetfpMasterMCB.logLevel = NetfpMaster_LogLevel_VERBOSE;
               }
               else if (!strcmp("DBG", optarg))
               {
                  gNetfpMasterMCB.logLevel = NetfpMaster_LogLevel_DEBUG;
               }
               else if (!strcmp("INF", optarg))
               {
                  gNetfpMasterMCB.logLevel = NetfpMaster_LogLevel_INFO;
               }
               else if (!strcmp("ERR", optarg))
               {
                  gNetfpMasterMCB.logLevel = NetfpMaster_LogLevel_ERROR;
               }
               else
               {
                  NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: wrong logging level %s, supported VRB|DBG|INF|ERR\n", optarg);
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

    /* We should have a valid RM client name */
    if (gNetfpMasterMCB.rmClientName[0] == 0)
        return -1;

    /* We should have a valid client list. */
    if (gNetfpMasterMCB.cfgFile[0] == 0)
        return -1;

    /* Command Line Arguments processed successfully. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to setup the preclassification for a specific interface
 *
 *  @param[in]  ptrNetfpIfBlock
 *      Pointer to the interface for which preclassification is to be configured
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t NetfpMaster_setupPreclassification (NetfpMaster_IfBlock* ptrNetfpIfBlock, int32_t* errCode)
{
    Netfp_PreClassificationCfg  preClassificationCfg;

    /* Initialize the preclassification configuration */
    memset ((void *)&preClassificationCfg, 0, sizeof(Netfp_PreClassificationCfg));

    /* Populate the configuration: */
    preClassificationCfg.switchPortNum = ptrNetfpIfBlock->switchPort;

    /* Do we need to enable broadcast preclassification? */
    if (ptrNetfpIfBlock->broadcastPreclassification == 1)
    {
        preClassificationCfg.enableBroadcast  = 1;
        preClassificationCfg.broadcastFlowId  = ptrNetfpIfBlock->broadcastPreclassificationFlowId;
        preClassificationCfg.broadcastQueueId = ptrNetfpIfBlock->broadcastPreclassificationQueueId;
    }

    /* Do we need to enable multicast preclassification? */
    if (ptrNetfpIfBlock->multicastPreclassification == 1)
    {
        preClassificationCfg.enableMulticast  = 1;
        preClassificationCfg.multicastFlowId  = ptrNetfpIfBlock->multicastPreclassificationFlowId;
        preClassificationCfg.multicastQueueId = ptrNetfpIfBlock->multicastPreclassificationQueueId;
    }

    /* Do we need to enable the preclassification on the interface? */
    if ((ptrNetfpIfBlock->broadcastPreclassification == 1) || (ptrNetfpIfBlock->multicastPreclassification == 1))
    {
        /* YES: Setup the preclassification on the interface: */
        if (Netfp_setupPreclassification (gNetfpMasterMCB.netfpServerHandle, &preClassificationCfg, errCode) < 0)
        {
            /* Error: Failed to initialize the NETFP broadcast preclassification */
            NetfpMaster_log(NetfpMaster_LogLevel_ERROR,
                            "Error: NETFP preclassification for '%s' failed [Error code %d]\n",
                             ptrNetfpIfBlock->name, *errCode);
            return -1;
        }
        NetfpMaster_log(NetfpMaster_LogLevel_DEBUG, "Debug: Preclassification for '%s' successful [Broadcast Flow %d Queue %d] [Multicast Flow %d Queue %d]\n",
                        ptrNetfpIfBlock->name,
                        ptrNetfpIfBlock->broadcastPreclassificationFlowId,
                        ptrNetfpIfBlock->broadcastPreclassificationQueueId,
                        ptrNetfpIfBlock->multicastPreclassificationFlowId,
                        ptrNetfpIfBlock->multicastPreclassificationQueueId);
    }
    else
    {
        /* NO. Skipping preclassification for the interface */
        NetfpMaster_log(NetfpMaster_LogLevel_DEBUG, "Debug: NETFP preclassification for '%s' bypassed\n",
                        ptrNetfpIfBlock->name);
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the NETFP master thread which provides an execution context for
 *      the NETFP master to execute
 *
 *  @param[in]  arg
 *      Argument passed to the thread.
 *
 *  @retval
 *      Not Applicable.
 */
static void* NetfpMaster_thread(void *arg)
{
    fd_set                          fds;
    int32_t                         maxFd;
    int32_t                         errCode;
    Netfp_ReassemblyConfig          reassemblyCfg;
    Netfp_PreClassificationCfg      preClassificationCfg;
    Netfp_DefaultReassemblyMgmtCfg  defaultReassemblyCfg;
    NetfpMaster_IfBlock*            ptrNetfpIfBlock;

    /********************************************************************************************
     * Initialize and setup the management interface:
     ********************************************************************************************/

    /* Initialize the PA LLD and initialize the NETCP subsystem. */
    if (NetfpMaster_initMgmt(&gNetfpMasterMCB) < 0)
        return NULL;

    /* The NETFP master is operational. */
    gNetfpMasterMCB.executionStatus = 1;

    /* Is the NETFP master responsible for reassembly? */
    if (gNetfpMasterMCB.reassemblyHandling == 1)
    {
        /* YES. Determine the maximum descriptor from the managment socket, inner & outer channels. */
        maxFd = max (maxFd, gNetfpMasterMCB.masterMgmtSocket);
        maxFd = max (maxFd, gNetfpMasterMCB.signalPipe[0]);
    }
    else
    {
        /* NO. We can receive data only from the management socket. */
        maxFd = max (gNetfpMasterMCB.signalPipe[0], gNetfpMasterMCB.masterMgmtSocket);
    }


    /* Get Queues from RM */
    if (gNetfpMasterMCB.handleSysCfg) {
        int32_t  internalErrCode = 0;
        uint32_t tempValue       = 0;
        /* Prepare QMSS Barrier Queue */
        if (Resmgr_nameServiceGet(gNetfpMasterMCB.handleSysCfg,
                                  "QMSS_BarrierQ_MSMC_NetCP",
                                  (uint32_t*)&tempValue,
                                  &internalErrCode) < 0) {
            gNetfpMasterMCB.paQueueBounceConfig.msmcQueueId = (Qmss_QueueHnd)QMSS_PARAM_NOT_SPECIFIED;
            NetfpMaster_log(NetfpMaster_LogLevel_INFO, "Resmger_nameServiceGet returned errcode %d for msmcQueueId\n", internalErrCode); //fzm
        } else {
            gNetfpMasterMCB.paQueueBounceConfig.msmcQueueId = tempValue;
        }

        tempValue = 0;
        if (Resmgr_nameServiceGet(gNetfpMasterMCB.handleSysCfg,
                                  "QMSS_BarrierQ_DDR_NetCP",
                                  (uint32_t*)&tempValue,
                                  &internalErrCode) < 0) {
            gNetfpMasterMCB.paQueueBounceConfig.ddrQueueId = (Qmss_QueueHnd)QMSS_PARAM_NOT_SPECIFIED;
            NetfpMaster_log(NetfpMaster_LogLevel_INFO, "Resmger_nameServiceGet returned errcode %d for ddrQueueId\n", internalErrCode); //fzm
        } else {
            gNetfpMasterMCB.paQueueBounceConfig.ddrQueueId = tempValue;
        }
    } else {
        NetfpMaster_log(NetfpMaster_LogLevel_ERROR, " ############################################################################\n ");
        NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "    ResMgr service handle is NULL! Skip QMSS Barrier Queues initialization!\n");
        NetfpMaster_log(NetfpMaster_LogLevel_ERROR, " ############################################################################\n ");
        gNetfpMasterMCB.paQueueBounceConfig.enable = 0;
    }

    if(Netfp_initQueueBounce(gNetfpMasterMCB.netfpServerHandle, gNetfpMasterMCB.paQueueBounceConfig, &errCode) < 0 )
    {
        NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error:PA queue bounce initialization failed [Error code %d]\n", errCode);
        return NULL;
    }

    /**********************************************************************************************
     * EQOS configuration:
     **********************************************************************************************/
    if (gNetfpMasterMCB.enableEQOS == 1)
    {
        if (NetfpMaster_initQoS(&gNetfpMasterMCB) < 0)
            return NULL;
    }
    else
    {
        NetfpMaster_log(NetfpMaster_LogLevel_DEBUG, "Debug: NETFP master bypassing the eQOS configuration\n");
    }

    /**********************************************************************************************
     * Frame Protocol CRC configuration:
     **********************************************************************************************/
    if (gNetfpMasterMCB.frameProtoCrcOffload == 1)
    {
        if (Netfp_initFrameProtoCRC(gNetfpMasterMCB.netfpServerHandle, &errCode) < 0)
            return NULL;
        else
            NetfpMaster_log(NetfpMaster_LogLevel_DEBUG, "Debug: NETFP Frame Protocol CRC Services initialized\n");
    }
    else
    {
        NetfpMaster_log(NetfpMaster_LogLevel_DEBUG, "Debug: NETFP master bypassing the Frame Protocol CRC Services\n");
    }

    /**********************************************************************************************
     * Reassembly configuration:
     **********************************************************************************************/
    if (gNetfpMasterMCB.reassemblyHandling == 1)
    {
        /* Initialize the reassembly configuration */
        memset ((void*)&reassemblyCfg, 0, sizeof(Netfp_ReassemblyConfig));

        /* Initialize the default reassembly configuration. */
        memset ((void*)&defaultReassemblyCfg, 0, sizeof(Netfp_DefaultReassemblyMgmtCfg));

        /* Populate the default reassembly configuration: */
        defaultReassemblyCfg.bufferThresholdUpper = 20000;
        defaultReassemblyCfg.bufferThresholdLower = 18000;

        /* Populate the reassembly configuration */
        reassemblyCfg.reassemblyMemRegion    = gNetfpMasterMCB.memoryRegionHandle;
        reassemblyCfg.numFragmentedPkt       = gNetfpMasterMCB.numReassemblyDescriptors;
        reassemblyCfg.numReassemblyContexts  = gNetfpMasterMCB.numReassemblyContext;
        reassemblyCfg.reassemblyTimeout      = gNetfpMasterMCB.reassemblyTimeout;               /* Unit - Seconds */
        reassemblyCfg.outerIpReassemChannel  = gNetfpMasterMCB.outerIPChannel;
        reassemblyCfg.innerIpReassemChannel  = gNetfpMasterMCB.innerIPChannel;
        reassemblyCfg.largePacketChannel     = NULL;
        reassemblyCfg.reassemblyHeapAlloc    = NetfpMaster_heapDataMalloc;
        reassemblyCfg.reassemblyHeapFree     = NetfpMaster_heapDataFree;
        reassemblyCfg.reassemblyMgmt         = NULL;
        reassemblyCfg.ptrReassemblyMgmtCfg   = &defaultReassemblyCfg;
        reassemblyCfg.sizeReassemblyCfg      = sizeof(defaultReassemblyCfg);

        /* Register the client to handle the reassembly for all fragmented packets */
        if (Netfp_registerReassemblyService (gNetfpMasterMCB.netfpClientHandle, &reassemblyCfg, &errCode) < 0)
        {
            /* Error: Failed to initialize the NETFP reassembly */
            NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: NETFP Reassembly initialization failed [Error code %d]\n", errCode);
            return NULL;
        }
        NetfpMaster_log(NetfpMaster_LogLevel_DEBUG, "Debug: NETFP Reassembly services registered\n");
    }

    /**********************************************************************************************
     * Preclassification configuration:
     **********************************************************************************************/
    if (gNetfpMasterMCB.enableGlobalPreclassification == 1)
    {
        /* Enable the system wide preclassification: */
        if (Netfp_initPreClassification (gNetfpMasterMCB.netfpServerHandle, &errCode) < 0)
        {
            /* Error: Failed to initialize the NETFP preclassification */
            NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: NETFP system preclassification initialization failed [Error code %d]\n", errCode);
            return NULL;
        }
        NetfpMaster_log(NetfpMaster_LogLevel_DEBUG, "Debug: NETFP system preclassification initialized\n");

        /* Cycle through all the interfaces and enable preclassification is required to do so */
        ptrNetfpIfBlock = (NetfpMaster_IfBlock*)NetfpMaster_listGetHead((NetfpMaster_ListNode**)&gNetfpMasterMCB.ptrInterfaceList);
        while (ptrNetfpIfBlock != NULL)
        {
            /* Initialize the preclassification configuration */
            memset ((void *)&preClassificationCfg, 0, sizeof(Netfp_PreClassificationCfg));

            /* Populate the configuration: */
            preClassificationCfg.switchPortNum = ptrNetfpIfBlock->switchPort;

            /* Do we need to enable broadcast preclassification? */
            if (ptrNetfpIfBlock->broadcastPreclassification == 1)
            {
                preClassificationCfg.enableBroadcast  = 1;
                preClassificationCfg.broadcastFlowId  = ptrNetfpIfBlock->broadcastPreclassificationFlowId;
                preClassificationCfg.broadcastQueueId = ptrNetfpIfBlock->broadcastPreclassificationQueueId;
            }

            /* Do we need to enable multicast preclassification? */
            if (ptrNetfpIfBlock->multicastPreclassification == 1)
            {
                preClassificationCfg.enableMulticast  = 1;
                preClassificationCfg.multicastFlowId  = ptrNetfpIfBlock->multicastPreclassificationFlowId;
                preClassificationCfg.multicastQueueId = ptrNetfpIfBlock->multicastPreclassificationQueueId;
            }

            /* Setup the preclassification for the specific interface: */
            NetfpMaster_setupPreclassification (ptrNetfpIfBlock, &errCode);

            /* Get the next interface block */
            ptrNetfpIfBlock = (NetfpMaster_IfBlock*)NetfpMaster_listGetNext((NetfpMaster_ListNode*)ptrNetfpIfBlock);
        }
    }

    /**********************************************************************************************
     * Port Mirroring/Capture configuration:
     **********************************************************************************************/
    if (Netfp_initPortMirroringCapturing(gNetfpMasterMCB.netfpServerHandle, &errCode) < 0)
    {
        /* Error: Failed to initialize the NETFP port capture */
        NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: NETFP system port mirror/capture initialization failed [Error code %d]\n", errCode);
        return NULL;
    }
    NetfpMaster_log(NetfpMaster_LogLevel_DEBUG, "Debug: NETFP system port mirror/capture initialized successfully\n");

    /**********************************************************************************************
     * NAT-T Initialization: LUT2 programming and exception route programming.
     **********************************************************************************************/
    if (gNetfpMasterMCB.nattCfg.udpPort != 0)
    {
        /* NAT-T init */
        if (Netfp_initNatt (gNetfpMasterMCB.netfpServerHandle, &gNetfpMasterMCB.nattCfg, &errCode) < 0)
        {
            NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: Initialization of NAT-T failed [Error code %d]\n", errCode);
            return NULL;
        }
        NetfpMaster_log(NetfpMaster_LogLevel_DEBUG, "Debug: Initialize NAT-T successfully.\n");
    }

    /**********************************************************************************************
     * Netfp User statistics configuration:
     **********************************************************************************************/
    if (gNetfpMasterMCB.userStatCfg.numTotalUserStats != 0)
    {
        /* Initialize the user stats: */
        if (Netfp_initUserStats(gNetfpMasterMCB.netfpServerHandle, &gNetfpMasterMCB.userStatCfg, &errCode) < 0)
        {
            NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: Initialization of User stats [Error code %d]\n", errCode);
            return NULL;
        }
        NetfpMaster_log(NetfpMaster_LogLevel_DEBUG, "Debug: User statistics initialized successfully [%d %d]\n",
                        gNetfpMasterMCB.userStatCfg.numTotalUserStats, gNetfpMasterMCB.userStatCfg.num64bUserStats);
    }

    /**********************************************************************************************
     * Netfp Master thread:
     **********************************************************************************************/
    while (gNetfpMasterMCB.executionStatus == 1)
    {
        /* Setup the event FIFO to wait on using select () */
        FD_ZERO (&fds);

        /* Wait for data to arrive on the management socket. */
        FD_SET (gNetfpMasterMCB.masterMgmtSocket, &fds);
        FD_SET (gNetfpMasterMCB.signalPipe[0],    &fds);

        /* Wait for the data to arrive */
        errCode = select (maxFd + 1, &fds, NULL, NULL, NULL);
        if (errCode < 0)
        {
            NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: NETFP master select failed [Error: %s]\n", strerror(errno));
            break;
        }

        /* Is there a timeout? */
        if (errCode == 0)
        {
            /* YES. Provide the reassembly timer services only if the reassembly services had been offloaded */
            if (gNetfpMasterMCB.reassemblyHandling == 1)
            {
                if (Netfp_reassemblyTimerTick (gNetfpMasterMCB.netfpClientHandle, gNetfpMasterMCB.reassemblyTimeout, &errCode) < 0)
                    NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: NETFP Reassembly Timer Tick failed [Error code %d]\n", errCode);
            }
        }

        /* Did we get a message from the signal pipe? */
        if (FD_ISSET(gNetfpMasterMCB.signalPipe[0], &fds) != 0)
            break;

        /* Did we get a management message? */
        if (FD_ISSET(gNetfpMasterMCB.masterMgmtSocket, &fds) != 0)
            NetfpMaster_processMgmt(&gNetfpMasterMCB);
    }

    /* NETFP Master thread has been terminated; shutting down... */
    NetfpMaster_log(NetfpMaster_LogLevel_DEBUG, "Debug: NETFP master thread terminated\n");
    return NULL;
}

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
static void NetfpMaster_LogFxn (Netfp_LogLevel logLevel, const char* fmt, va_list arg)
{
    /* If the NETFP library has reported an error or informational message; we always log this */
    if ((logLevel == Netfp_LogLevel_ERROR) || (logLevel == Netfp_LogLevel_INFO))
    {
        vprintf (fmt, arg);
        return;
    }

    /* If the NETFP library has reported a debug message and the server is executing
     * with the verbosity on. Log the message. */
    if ((logLevel == Netfp_LogLevel_DEBUG) && (gNetfpMasterMCB.logLevel == NetfpMaster_LogLevel_DEBUG))
        vprintf (fmt, arg);
}

/**
 *  @b Description
 *  @n
 *      Utility function which is used to round off the number of descriptors as per
 *      the Keystone2 guidelines. This is required else the insertion of memory region
 *      will fail.
 *
 *  @param[in]  numDesc
 *      Number of descriptors
 *
 *  @retval
 *      Success -   Rounded off descriptors
 *  @retval
 *      Error   -   0
 */
uint32_t NetfpMaster_roundNumberDescriptors (uint32_t numDesc)
{
    int32_t index;
    uint32_t count;

    /* Round off the number of descriptors requested as per the Keystone2 guidelines:
     *  - Min. number of descriptors: 2^5
     *  - Max. number of descriptors: 2^20 */
    for (index = 5; index <= 20; index++)
    {
        count = 1 << index;
        if (numDesc <= count)
        {
            NetfpMaster_log(NetfpMaster_LogLevel_DEBUG, "Debug: Rounding off descriptors from %u to %u\n",
                            numDesc, count);
            return count;
        }
    }

    /* Debug Message: */
    NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: Descriptor usage %u exceeds max allowed %u\n", numDesc, 1<<20);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The thread initializes the NETFP Master
 *
 *  @param[in]  arg
 *      Argument passed to the thread.
 *
 *  @retval
 *      Not Applicable.
 */
static void* NetfpMaster_initThread(void *arg)
{
    Resmgr_SystemCfg            sysConfig;
    Name_DatabaseCfg            databaseCfg;
    Pktlib_InstCfg              pktlibInstCfg;
    Msgcom_InstCfg              msgcomInstCfg;
    pthread_t                   masterThread;
#if 0 //https://e2eprivate.ti.com/nokia_siemens_networks/k2_-_fsm4_-_fzm_-_lrc_nokia/f/191/t/5915.aspx
    pthread_t                   netfpServerThread;
    pthread_t                   netfpClientThread;
#endif
    Pktlib_HeapCfg              heapCfg;
    Netfp_ServerConfig          serverConfig;
    Msgcom_ChannelCfg           chConfig;
#if 0 //https://e2eprivate.ti.com/nokia_siemens_networks/k2_-_fsm4_-_fzm_-_lrc_nokia/f/191/t/5915.aspx
    Msgcom_DirectInterruptCfg   directInterruptCfg;
#endif
    Netfp_ClientConfig          clientCfg;
#if 0 //https://e2eprivate.ti.com/nokia_siemens_networks/k2_-_fsm4_-_fzm_-_lrc_nokia/f/191/t/5915.aspx
    int32_t                     clientStatus;
#endif
    int32_t                     errCode;
    Cppi_CpDmaInitCfg           cpdmaCfg;
    Resmgr_ResourceCfg          netfpMasterResourceConfig =
    {
        /* Requested Memory Region Configuration. */
        .memRegionCfg =
        {
            /* Name,                Type,                       Linking RAM,                           Num,        Size */
            { "NETFPMaster_Region", Resmgr_MemRegionType_DDR3,  Resmgr_MemRegionLinkingRAM_DONT_CARE,  128,       128},
        },
    };
    //fzm --->
    //initialize the ddal cma subsystem (mmaps /dev/cma into our address space)
    NetfpMaster_IfBlock*    ptrNetfpIfBlock;
    int32_t                 index;
    uint32_t                totalEQoSDescriptors = 0;
    //<---- fzm

    /* Parse the configuration file: */
    if (NetfpMaster_parseConfigFile (&gNetfpMasterMCB) < 0)
        return NULL;

    //fzm --->
    //initialize the ddal cma subsystem (mmaps /dev/cma into our address space)
    int ddalCmaInitStatus = ddal_cma_process_init();
    if(ddalCmaInitStatus != DDAL_OK)
    {
        NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Failed to initialize DDAL CMA, errcode: %d", ddalCmaInitStatus);
        return NULL;
    }
    //<---- fzm

    /********************************************************************************************
     * Initialize the SYSLIB modules:
     * - Initialize the name database module
     ********************************************************************************************/

    /* Initialize the database configuration */
    memset ((void *)&databaseCfg, 0, sizeof(Name_DatabaseCfg));

    /* Populate the configuration: */
    databaseCfg.instanceId  = gNetfpMasterMCB.nrInstanceId;
    databaseCfg.realm       = Name_ExecutionRealm_ARM;
    strcpy (databaseCfg.owner, "NETFP_Master");

    /* Create the NETFP Server database handle */
    gNetfpMasterMCB.databaseHandle = Name_createDatabase (&databaseCfg, &errCode);
    if (gNetfpMasterMCB.databaseHandle == NULL)
    {
        NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: Name database creation failed [Error code %d]\n", errCode);
        return NULL;
    }
    NetfpMaster_log(NetfpMaster_LogLevel_DEBUG, "Debug: Name database created successfully [Handle %p]\n", gNetfpMasterMCB.databaseHandle);

    /********************************************************************************************
     * Initialize the resource manager module
     ********************************************************************************************/

    /* Initialize the system configuration. */
    memset ((void *)&sysConfig, 0, sizeof(Resmgr_SystemCfg));

    /* Populate the configuration.*/
    sysConfig.realm                 = Resmgr_ExecutionRealm_ARM;
    strcpy (sysConfig.rmClient, gNetfpMasterMCB.rmClientName);
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
    gNetfpMasterMCB.handleSysCfg = Resmgr_init(&sysConfig, &errCode);
    if (gNetfpMasterMCB.handleSysCfg == NULL)
    {
        NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: System configuration initialization failed [Error code %d]\n", errCode);
        return NULL;
    }

    /********************************************************************************************
     * Initialize the PKTLIB module
     ********************************************************************************************/

    /* Initialize the PKTLIB instance configuration. */
    memset ((void *)&pktlibInstCfg, 0, sizeof(Pktlib_InstCfg));

    /* Populate the PKTLIB Instance configuration: */
    pktlibInstCfg.instanceId        = 1;
    pktlibInstCfg.databaseHandle    = gNetfpMasterMCB.databaseHandle;
    pktlibInstCfg.sysCfgHandle      = gNetfpMasterMCB.handleSysCfg;
    pktlibInstCfg.malloc            = Pktlib_osalMalloc;
    pktlibInstCfg.free              = Pktlib_osalFree;
    pktlibInstCfg.beginMemAccess    = Pktlib_osalBeginMemAccess;
    pktlibInstCfg.endMemAccess      = Pktlib_osalEndMemAccess;
    pktlibInstCfg.beginPktAccess    = Pktlib_osalBeginPktAccess;
    pktlibInstCfg.endPktAccess      = Pktlib_osalEndPktAccess;
    pktlibInstCfg.enterCS           = Pktlib_osalEnterCS;
    pktlibInstCfg.exitCS            = Pktlib_osalExitCS;
    pktlibInstCfg.phyToVirt         = Pktlib_osalPhyToVirt;
    pktlibInstCfg.sharedEnterCS     = Pktlib_osalSharedEnterCS;
    pktlibInstCfg.sharedExitCS      = Pktlib_osalSharedExitCS;
    pktlibInstCfg.useSharedStarvationQueue = 1;

    /* Create the PKTLIB instance */
    gNetfpMasterMCB.pktlibInstanceHandle = Pktlib_createInstance (&pktlibInstCfg, &errCode);
    if (gNetfpMasterMCB.pktlibInstanceHandle == NULL)
    {
        NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: Creating the PKTLIB instance failed [Error code %d]\n", errCode);
        return NULL;
    }

    /********************************************************************************************
     * Initialize the MSGCOM module
     ********************************************************************************************/

    /* Initialize the MSGCOM instance. */
    memset ((void *)&msgcomInstCfg, 0, sizeof(Msgcom_InstCfg));

    /* Populate the MSGCOM Instance configuration */
    msgcomInstCfg.databaseHandle    = gNetfpMasterMCB.databaseHandle;
    msgcomInstCfg.sysCfgHandle      = gNetfpMasterMCB.handleSysCfg;
    msgcomInstCfg.pktlibInstHandle  = gNetfpMasterMCB.pktlibInstanceHandle;
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
    gNetfpMasterMCB.msgcomInstanceHandle = Msgcom_createInstance (&msgcomInstCfg, &errCode);
    if (gNetfpMasterMCB.msgcomInstanceHandle == NULL)
    {
        NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: MSGCOM Instance creation failed [Error code %d]\n", errCode);
        return NULL;
    }

    /********************************************************************************************
     * Request resources:
     ********************************************************************************************/

    /* We need additional descriptors for the reassembly if required to do so */
    netfpMasterResourceConfig.memRegionCfg[0].numDesc = netfpMasterResourceConfig.memRegionCfg[0].numDesc +
                                                        gNetfpMasterMCB.numReassemblyDescriptors;
//fzm-->
    if (gNetfpMasterMCB.enableEQOS == 1)
    {
        /*Adding descriptors needed for eQoS*/
        ptrNetfpIfBlock = (NetfpMaster_IfBlock*)NetfpMaster_listGetHead((NetfpMaster_ListNode**)&gNetfpMasterMCB.ptrInterfaceList);
        while (ptrNetfpIfBlock != NULL)
        {
            /* Cycle through all the flows configurations for the interface and determine the number of descriptors. */
            for (index = 0; index < NETFP_MASTER_MAX_FLOW; index++)
                totalEQoSDescriptors += ptrNetfpIfBlock->flowBlock[index].numDescriptors;
            /* Get the next interface block */
            ptrNetfpIfBlock = (NetfpMaster_IfBlock*)NetfpMaster_listGetNext((NetfpMaster_ListNode*)ptrNetfpIfBlock);
        }

        /* Debug Message: */
        NetfpMaster_log(NetfpMaster_LogLevel_DEBUG, "Debug: eQoS needs %d descriptors for all interfaces\n", totalEQoSDescriptors);
//fzm<--

        /* Round off the total number of descriptors required by all the interface flows: */
        if (totalEQoSDescriptors == 0)
            return NULL;
        netfpMasterResourceConfig.memRegionCfg[0].numDesc += totalEQoSDescriptors;
    }

    /* Round off the number of descriptors: */
    netfpMasterResourceConfig.memRegionCfg[0].numDesc = NetfpMaster_roundNumberDescriptors (netfpMasterResourceConfig.memRegionCfg[0].numDesc);
    if (netfpMasterResourceConfig.memRegionCfg[0].numDesc == 0)
        return NULL;

    NetfpMaster_log(NetfpMaster_LogLevel_DEBUG, "After round up, netfp master total numDesc: %d\n", netfpMasterResourceConfig.memRegionCfg[0].numDesc); //fzm

    /* Is the NETFP master responsible for the reassembly? */
    if (gNetfpMasterMCB.reassemblyHandling == 1)
    {
#ifdef USE_ACCUMULATED_CHANNEL
        netfpMasterResourceConfig.numAccumalatorChannels = 2;   /* Inner + Outer Fragment channels */
#else
        netfpMasterResourceConfig.numQpendQueues         = netfpMasterResourceConfig.numQpendQueues + 2;   /* Inner + Outer Fragment channels */
#endif
    }

    /* Process the resource configuration: */
    if (Resmgr_processConfig (gNetfpMasterMCB.handleSysCfg, &netfpMasterResourceConfig, &errCode) < 0)
    {
        NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: SYSRM configuration failed [Error code %d]\n", errCode);
        return NULL;
    }
    NetfpMaster_log(NetfpMaster_LogLevel_DEBUG, "Debug: NETFP Master configuration processed succesfully\n");

    /* Track the memory region which had been allocated */
    gNetfpMasterMCB.memoryRegionHandle = netfpMasterResourceConfig.memRegionResponse[0].memRegionHandle;

    /********************************************************************************************
     * Read User stats configuration from DTS file
     ********************************************************************************************/
    if (Resmgr_allocCustomResource(gNetfpMasterMCB.handleSysCfg, "pa-num64bUserStats", 1,
                                   (uint32_t*)&gNetfpMasterMCB.userStatCfg.num64bUserStats, &errCode) < 0)
    {
        NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: Unable to allocate resource 'pa-num64bUserStats' [Error code %d]\n", errCode);
        return NULL;
    }

    /* Cleanup the resource: */
    if (Resmgr_freeCustomResource(gNetfpMasterMCB.handleSysCfg, "pa-num64bUserStats", 1,
                                  gNetfpMasterMCB.userStatCfg.num64bUserStats, &errCode) < 0)
    {
        NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: Cleaning resource 'pa-num64bUserStats' failed [Error code %d]\n", errCode);
        return NULL;
    }

    /* Find out total number of counters */
    gNetfpMasterMCB.userStatCfg.numTotalUserStats = pa_USR_STATS_MAX_COUNTERS - gNetfpMasterMCB.userStatCfg.num64bUserStats;

    /* Debug Message: */
    NetfpMaster_log(NetfpMaster_LogLevel_DEBUG, "Debug: NETFP Master User stats Total counter: %d, 64bit counter: %d\n",
                    gNetfpMasterMCB.userStatCfg.numTotalUserStats, gNetfpMasterMCB.userStatCfg.num64bUserStats);

    /********************************************************************************************
     * Initialize the PKTLIB heaps:
     ********************************************************************************************/

    /* Initialize the heap configuration */
    memset((void *)&heapCfg, 0 , sizeof(Pktlib_HeapCfg));

    /* Populate the heap configuration */
    strcpy(heapCfg.name, "NETFPMaster-NETCPHeap");
    heapCfg.memRegion                       = gNetfpMasterMCB.memoryRegionHandle;
    heapCfg.pktlibInstHandle                = gNetfpMasterMCB.pktlibInstanceHandle;
    heapCfg.sharedHeap                      = 0;
    heapCfg.useStarvationQueue              = 0;
    heapCfg.dataBufferSize                  = pa_MAX_CMD_BUF_SIZE_BYTES;   /* size of the user stats PA cmd */
    heapCfg.numPkts                         = 128;
    heapCfg.numZeroBufferPackets            = 0;
    heapCfg.dataBufferPktThreshold          = 0;
    heapCfg.zeroBufferPktThreshold          = 0;
    heapCfg.heapInterfaceTable.dataMalloc   = NetfpMaster_heapDataMalloc;
    heapCfg.heapInterfaceTable.dataFree     = NetfpMaster_heapDataFree;
    heapCfg.linearAlloc = 1;

    /* Create the NETFP Master heap */
    gNetfpMasterMCB.masterHeapHandle = Pktlib_createHeap(&heapCfg, &errCode);
    if (gNetfpMasterMCB.masterHeapHandle == NULL)
    {
        NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: Unable to create the master heap [Error code %d]\n", errCode);
        return NULL;
    }
    NetfpMaster_log(NetfpMaster_LogLevel_DEBUG, "Debug: Heap %s has been created successfully\n", heapCfg.name);

    /********************************************************************************************
     * Initialize the NETFP Server
     ********************************************************************************************/

    /* Initialize the server configuration. */
    memset ((void *)&serverConfig, 0, sizeof(Netfp_ServerConfig));

    /* Populate the server configuration:
     *  a) Initialize the secure subystem.
     *  b) The maximum security channels are hardcoded to 512
     *  c) Base security context identifier is starting from 16
     * Items (b) and (c) might need to be parameterized later on if we move on to address
     * shared security channels. */
    strcpy (serverConfig.serverName, "__NETFPMaster_Server");
    serverConfig.pktlibInstHandle       = gNetfpMasterMCB.pktlibInstanceHandle;
    serverConfig.msgcomInstHandle       = gNetfpMasterMCB.msgcomInstanceHandle;
    serverConfig.realm                  = Netfp_ExecutionRealm_ARM;
    serverConfig.serverHeapHandle       = gNetfpMasterMCB.masterHeapHandle;
    serverConfig.initSecureSubsystem    = 1;
    serverConfig.nrInstanceId           = gNetfpMasterMCB.nrInstanceId;
    serverConfig.cmdHeapHandle          = gNetfpMasterMCB.masterHeapHandle;
    serverConfig.ipSecHeapHandle        = gNetfpMasterMCB.masterHeapHandle;
    serverConfig.dumpFxn                = NetfpMaster_dumpMsg; //fzm
    serverConfig.logFxn                 = NetfpMaster_LogFxn;
    serverConfig.maxSecurityChannels    = 512;
    serverConfig.baseSecurityContextId  = 16;
    serverConfig.enableIPLutEntryCount  = 0;
    serverConfig.passCfgVirtualAddress  = Resmgr_getPASSVirtualAddress(gNetfpMasterMCB.handleSysCfg);
    serverConfig.rmServiceHandle        = Resmgr_getRMServiceHandle (gNetfpMasterMCB.handleSysCfg);
    serverConfig.sysRMHandle            = gNetfpMasterMCB.handleSysCfg;
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
    gNetfpMasterMCB.netfpServerHandle = Netfp_initServer (&serverConfig, &errCode);
    if (gNetfpMasterMCB.netfpServerHandle == NULL)
    {
        NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: Failed to initialize the NETFP Server [Error code %d]\n", errCode);
        return NULL;
    }
    NetfpMaster_log(NetfpMaster_LogLevel_DEBUG, "Debug: NETFP Server '%s' is operational %p\n", serverConfig.serverName, gNetfpMasterMCB.netfpServerHandle);

    /* Initialize the CPDMA configuration: */
    memset (&cpdmaCfg, 0, sizeof (Cppi_CpDmaInitCfg));

    /* Populate the DMA configuration: Dont override the hardware configuration. SOC Init is responsible for the
     * hardware configuration. */
    cpdmaCfg.dmaNum       = Cppi_CpDma_PASS_CPDMA;
    cpdmaCfg.regWriteFlag = Cppi_RegWriteFlag_OFF;

    /* Open the NETCP DMA: */
    gNetfpMasterMCB.passCPDMAHandle = Cppi_open (&cpdmaCfg);
    if (gNetfpMasterMCB.passCPDMAHandle == NULL)
    {
        NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: Failed to open the NETCP CPDMA\n");
        return NULL;
    }

    /* Launch the NETFP client: */
    memset ((void *)&clientCfg, 0, sizeof(Netfp_ClientConfig));

    /* Populate the NETFP Client configuration:  */
    strcpy (clientCfg.serverName, serverConfig.serverName);
    sprintf (clientCfg.clientName, "__NETFPMaster_Client");
    clientCfg.nrInstanceId                       = gNetfpMasterMCB.nrInstanceId;
    clientCfg.serverHandle                       = gNetfpMasterMCB.netfpServerHandle;
    clientCfg.pktlibInstHandle                   = gNetfpMasterMCB.pktlibInstanceHandle;
    clientCfg.msgcomInstHandle                   = gNetfpMasterMCB.msgcomInstanceHandle;
#if 0 //https://e2eprivate.ti.com/nokia_siemens_networks/k2_-_fsm4_-_fzm_-_lrc_nokia/f/191/t/5915.aspx
    clientCfg.directInterruptCfg.queuePendQueue  = netfpMasterResourceConfig.qPendResponse[1].queue;
    clientCfg.directInterruptCfg.cpIntcId        = netfpMasterResourceConfig.qPendResponse[1].cpIntcId;
    clientCfg.directInterruptCfg.systemInterrupt = netfpMasterResourceConfig.qPendResponse[1].systemInterrupt;
    clientCfg.directInterruptCfg.hostInterrupt   = netfpMasterResourceConfig.qPendResponse[1].hostInterrupt;
#endif
    clientCfg.nameClientHandle                   = NULL;
    clientCfg.netHeaderHeapHandle                = gNetfpMasterMCB.masterHeapHandle;
    clientCfg.fragmentHeap                       = gNetfpMasterMCB.masterHeapHandle;
    clientCfg.realm                              = Netfp_ExecutionRealm_ARM;
    clientCfg.clientHeapHandle                   = gNetfpMasterMCB.masterHeapHandle;
    clientCfg.malloc                             = Netfp_osalMalloc;
    clientCfg.free                               = Netfp_osalFree;
    clientCfg.beginMemAccess                     = Netfp_osalBeginMemoryAccess;
    clientCfg.endMemAccess                       = Netfp_osalEndMemoryAccess;
    clientCfg.enterCS                            = Netfp_osalEnterSingleCoreCriticalSection;
    clientCfg.exitCS                             = Netfp_osalExitSingleCoreCriticalSection;
    clientCfg.createSem                          = Netfp_osalCreateSem;
    clientCfg.deleteSem                          = Netfp_osalDeleteSem;
    clientCfg.postSem                            = Netfp_osalPostSem;
    clientCfg.pendSem                            = Netfp_osalPendSem;

    /* Create the NETFP Client. */
    gNetfpMasterMCB.netfpClientHandle = Netfp_initClient (&clientCfg, &errCode);
    if (gNetfpMasterMCB.netfpClientHandle == NULL)
    {
        NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: NETFP Client Initialization failed [Error code %d]\n", errCode);
        return NULL;
    }
    NetfpMaster_log(NetfpMaster_LogLevel_DEBUG, "Debug: NETFP Client [%s] Initialized %p\n", clientCfg.clientName, gNetfpMasterMCB.netfpClientHandle);

#if 0 //https://e2eprivate.ti.com/nokia_siemens_networks/k2_-_fsm4_-_fzm_-_lrc_nokia/f/191/t/5915.aspx
    /* Register the NETFP Client with the NETFP server: */
    directInterruptCfg.queuePendQueue  = netfpMasterResourceConfig.qPendResponse[0].queue;
    directInterruptCfg.cpIntcId        = netfpMasterResourceConfig.qPendResponse[0].cpIntcId;
    directInterruptCfg.systemInterrupt = netfpMasterResourceConfig.qPendResponse[0].systemInterrupt;
    directInterruptCfg.hostInterrupt   = netfpMasterResourceConfig.qPendResponse[0].hostInterrupt;

    /* Try and register the client.  */
    if (Netfp_registerClient (gNetfpMasterMCB.netfpServerHandle, clientCfg.clientName,
                              &directInterruptCfg, &errCode) < 0)
    {
        /* NETFP client registeration was successful. */
        NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: NETFP Client registeration failed [Error code %d]\n", errCode);
        return NULL;
    }
    NetfpMaster_log(NetfpMaster_LogLevel_DEBUG, "Debug: NETFP Client '%s' registered\n", clientCfg.clientName);

    /* Loop around and synchronize the client registration with the server. */
    while (1)
    {
        /* Start the NETFP client */
        clientStatus = Netfp_startClient (gNetfpMasterMCB.netfpClientHandle, &errCode);
        if (clientStatus < 0)
        {
            NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: Client registration status failed [Error code %d]\n", errCode);
            return NULL;
        }

        /* If the client has been registered; we can proceed */
        if (clientStatus == 1)
            break;

        /* Client has not been registered; wait for some time and try again */
        usleep(100);
    }
    NetfpMaster_log(NetfpMaster_LogLevel_DEBUG, "Debug: NETFP Client '%s' has been started\n", clientCfg.clientName);
#endif

    /* Do we need to launch the NETFP client? NETFP client services are required only if we need to support the reassembly */
    if (gNetfpMasterMCB.reassemblyHandling == 1)
    {
        /********************************************************************************************
         * Initialize the Outer IP & Inner IP MSGCOM channels:
         ********************************************************************************************/

        /* Initialize the channel configuration. */
        memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

#ifdef USE_ACCUMULATED_CHANNEL
        /* Populate the channel configuration. */
        chConfig.mode                                               = Msgcom_ChannelMode_NON_BLOCKING;
        chConfig.appCallBack                                        = NULL;
        chConfig.msgcomInstHandle                                   = gNetfpMasterMCB.msgcomInstanceHandle;
        chConfig.u.queueCfg.interruptMode                           = Msgcom_QueueInterruptMode_ACCUMULATED_INTERRUPT;
        chConfig.u.queueCfg.queueIntrUnion.accCfg.type              = Msgcom_AccumulatedChannelType_HIGH;
        chConfig.u.queueCfg.queueIntrUnion.accCfg.accChannel        = netfpMasterResourceConfig.accChannelResponse[0].accChannel;
        chConfig.u.queueCfg.queueIntrUnion.accCfg.accQueue          = netfpMasterResourceConfig.accChannelResponse[0].queue;
        chConfig.u.queueCfg.queueIntrUnion.accCfg.pdspId            = netfpMasterResourceConfig.accChannelResponse[0].pdspId;
        chConfig.u.queueCfg.queueIntrUnion.accCfg.interruptId       = netfpMasterResourceConfig.accChannelResponse[0].eventId;
        chConfig.u.queueCfg.queueIntrUnion.accCfg.maxPageEntries    = 50;
        chConfig.u.queueCfg.queueIntrUnion.accCfg.pacingTimerCount  = 40;
#else
        /* Populate the channel configuration. */
        chConfig.mode                                                   = Msgcom_ChannelMode_NON_BLOCKING;
        chConfig.appCallBack                                            = NULL;
        chConfig.msgcomInstHandle                                       = gNetfpMasterMCB.msgcomInstanceHandle;
        chConfig.u.queueCfg.interruptMode                               = Msgcom_QueueInterruptMode_DIRECT_INTERRUPT;
        chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.queuePendQueue  = netfpMasterResourceConfig.qPendResponse[2].queue;
        chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.cpIntcId        = netfpMasterResourceConfig.qPendResponse[2].cpIntcId;
        chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.systemInterrupt = netfpMasterResourceConfig.qPendResponse[2].systemInterrupt;
        chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.hostInterrupt   = netfpMasterResourceConfig.qPendResponse[2].hostInterrupt;
#endif

        /* Outer-IP reassembly channel: */
        gNetfpMasterMCB.outerIPChannel = Msgcom_create ("NETFPMaster-OuterIP", Msgcom_ChannelType_QUEUE, &chConfig, &errCode);
        if (gNetfpMasterMCB.outerIPChannel == NULL)
        {
            NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: Unable to open the OuterIP reassembly channel [Error code: %d]\n", errCode);
            return NULL;
        }

        /* Initialize the channel configuration. */
        memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

#ifdef USE_ACCUMULATED_CHANNEL
        /* Populate the channel configuration. */
        chConfig.mode                                               = Msgcom_ChannelMode_NON_BLOCKING;
        chConfig.appCallBack                                        = NULL;
        chConfig.msgcomInstHandle                                   = gNetfpMasterMCB.msgcomInstanceHandle;
        chConfig.u.queueCfg.interruptMode                           = Msgcom_QueueInterruptMode_ACCUMULATED_INTERRUPT;
        chConfig.u.queueCfg.queueIntrUnion.accCfg.type              = Msgcom_AccumulatedChannelType_HIGH;
        chConfig.u.queueCfg.queueIntrUnion.accCfg.accChannel        = netfpMasterResourceConfig.accChannelResponse[1].accChannel;
        chConfig.u.queueCfg.queueIntrUnion.accCfg.accQueue          = netfpMasterResourceConfig.accChannelResponse[1].queue;
        chConfig.u.queueCfg.queueIntrUnion.accCfg.pdspId            = netfpMasterResourceConfig.accChannelResponse[1].pdspId;
        chConfig.u.queueCfg.queueIntrUnion.accCfg.interruptId       = netfpMasterResourceConfig.accChannelResponse[1].eventId;
        chConfig.u.queueCfg.queueIntrUnion.accCfg.maxPageEntries    = 50;
        chConfig.u.queueCfg.queueIntrUnion.accCfg.pacingTimerCount  = 40;
#else
        /* Populate the channel configuration. */
        chConfig.mode                                                   = Msgcom_ChannelMode_NON_BLOCKING;
        chConfig.appCallBack                                            = NULL;
        chConfig.msgcomInstHandle                                       = gNetfpMasterMCB.msgcomInstanceHandle;
        chConfig.u.queueCfg.interruptMode                               = Msgcom_QueueInterruptMode_DIRECT_INTERRUPT;
        chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.queuePendQueue  = netfpMasterResourceConfig.qPendResponse[3].queue;
        chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.cpIntcId        = netfpMasterResourceConfig.qPendResponse[3].cpIntcId;
        chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.systemInterrupt = netfpMasterResourceConfig.qPendResponse[3].systemInterrupt;
        chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.hostInterrupt   = netfpMasterResourceConfig.qPendResponse[3].hostInterrupt;
#endif
        /* Inner IP reassembly channel: */
        gNetfpMasterMCB.innerIPChannel = Msgcom_create ("NETFPMaster-InnerIP", Msgcom_ChannelType_QUEUE, &chConfig, &errCode);
        if (gNetfpMasterMCB.innerIPChannel == NULL)
        {
            NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: Unable to open the InnerIP reassembly channel [Error code : %d]\n", errCode);
            return NULL;
        }
    }
    else
    {
        /* Reassembly has not been offloaded to the master; so there is no need to create a NETFP client. */
        NetfpMaster_log(NetfpMaster_LogLevel_DEBUG, "Debug: Bypassing NETFP client creation [No reassembly offload]\n");
    }
// fzm-->
    /********************************************************************************************
     * Configure QMSS BarrierQ DDR
     ********************************************************************************************/
    uint32_t ddrQnum = -1;
    uint32_t ddrQnumNetcp = -1;
    Qmss_PdspId   pdspIdDdr = -1;

    if (Resmgr_nameServiceGet(gNetfpMasterMCB.handleSysCfg, "QMSS_BarrierQ_DDR",
                                   &ddrQnum, &errCode) < 0)
    {
        NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: Unable to allocate resource 'QMSS_BarrierQ_DDR' [Error code %d]\n", errCode);
        return NULL;
    }
    if (Resmgr_nameServiceGet(gNetfpMasterMCB.handleSysCfg, "QMSS_BarrierQ_DDR_NetCP",
                                   &ddrQnumNetcp, &errCode) < 0)
    {
        NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: Unable to allocate resource 'QMSS_BarrierQ_DDR_NetCP' [Error code %d]\n", errCode);
        return NULL;
    }
    if (Resmgr_nameServiceGet(gNetfpMasterMCB.handleSysCfg, "QMSS_BarrierQ_DDR_PDSPID",
                                   (uint32_t*)&pdspIdDdr, &errCode) < 0)
    {
        NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: Unable to allocate resource 'QMSS_BarrierQ_DDR_PDSPID' [Error code %d]\n", errCode);
        return NULL;
    }
    // configure DDR barrier Q
    NetfpMaster_log(NetfpMaster_LogLevel_INFO,
            "  ========== DDR Barrier queue is %5u , NetCP queue %5u , PDSP ID %2u ============\n",
            ddrQnum, ddrQnumNetcp, pdspIdDdr);

    errCode = Qmss_programDDRBarrierQueue(pdspIdDdr,
            (Qmss_QueueHnd)ddrQnum,
            (Qmss_QueueHnd)ddrQnumNetcp);
    if (errCode < QMSS_SOK)
    {
        NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "    ==> DDR Barrier Q init failed! [errCode: %d]\n", errCode);
        return NULL;
    }
    else
    {
        NetfpMaster_log(NetfpMaster_LogLevel_INFO, "    * DDR Barrier Q initialized!\n");
    }
// <--fzm

    /**********************************************************************************
     * SYSLIB modules are operational: Spawn all the threads
     **********************************************************************************/
    errCode = pthread_create (&masterThread, NULL, NetfpMaster_thread, NULL);
    if (errCode < 0)
    {
        NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: Master thread create failed error code %d\n", errCode);
        return NULL;
    }
#if 0 //https://e2eprivate.ti.com/nokia_siemens_networks/k2_-_fsm4_-_fzm_-_lrc_nokia/f/191/t/5915.aspx
    errCode = pthread_create (&netfpServerThread, NULL, NetfpMaster_netfpServerThread, NULL);
    if (errCode < 0)
    {
        NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: Server thread create failed error code %d\n", errCode);
        return NULL;
    }
    errCode = pthread_create (&netfpClientThread, NULL, NetfpMaster_netfpClientThread, NULL);
    if (errCode < 0)
    {
        NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: Server thread create failed error code %d\n", errCode);
        return NULL;
    }
#endif

    /* Wait for the master thread to terminate. */
    pthread_join (masterThread, NULL);

    /* Shutdown the MSGCOM Inner & Outer reassembly channels: */
    if (gNetfpMasterMCB.outerIPChannel != NULL)
    {
        errCode = Msgcom_delete (gNetfpMasterMCB.outerIPChannel, NetfpMaster_flushFragment);
        if (errCode < 0)
            NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: Outer IP reassembly channel deletion failed [Error code %d]\n", errCode);
        else
            NetfpMaster_log(NetfpMaster_LogLevel_DEBUG, "Debug: Outer IP reassembly channel deleted successfully\n");
    }
    if (gNetfpMasterMCB.innerIPChannel != NULL)
    {
        errCode = Msgcom_delete (gNetfpMasterMCB.innerIPChannel, NetfpMaster_flushFragment);
        if (errCode < 0)
            NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: Inner IP reassembly channel deletion failed [Error code %d]\n", errCode);
        else
            NetfpMaster_log(NetfpMaster_LogLevel_DEBUG, "Debug: Inner IP reassembly channel deleted successfully\n");
    }

    /* Do we need to shutdown the reassembly services? */
    if (gNetfpMasterMCB.reassemblyHandling == 1)
    {
        if (Netfp_deregisterReassemblyService (gNetfpMasterMCB.netfpClientHandle, &errCode) < 0)
            NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: Reassembly service deregisteration failed [Error code %d]\n", errCode);
        else
            NetfpMaster_log(NetfpMaster_LogLevel_DEBUG, "Debug: Reassembly service deregistered successfully\n");
    }

#if 0 //https://e2eprivate.ti.com/nokia_siemens_networks/k2_-_fsm4_-_fzm_-_lrc_nokia/f/191/t/5915.aspx
    /* Stop the NETFP client */
    if (Netfp_stopClient (gNetfpMasterMCB.netfpClientHandle, &errCode) < 0)
        NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: NETFP client stopping failed [Error code %d]\n", errCode);
    else
        NetfpMaster_log(NetfpMaster_LogLevel_DEBUG, "Debug: NETFP client stopped successfully\n");
#endif

#if 0 // fails due to client's state = INITIALIZED
    /* Delete the NETFP Client: */
    if (Netfp_deleteClient (gNetfpMasterMCB.netfpClientHandle, &errCode) < 0)
        NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: NETFP Client deletion failed [Error code %d]\n", errCode);
    else
        NetfpMaster_log(NetfpMaster_LogLevel_DEBUG, "Debug: NETFP Client deleted successfully\n");
#endif

#if 0 //https://e2eprivate.ti.com/nokia_siemens_networks/k2_-_fsm4_-_fzm_-_lrc_nokia/f/191/t/5915.aspx
    /* Shutdown the NETFP Server and NETFP client threads */
    pthread_cancel (netfpServerThread);
    pthread_cancel (netfpClientThread);
#endif

    /* Delete the NETFP server: */
    while (1)
    {
        /* Delete the NETFP server. */
        if (Netfp_deleteServer (gNetfpMasterMCB.netfpServerHandle, &errCode) == 0)
            break;

        /* The NETFP Server deletion can fail if the NETFP clients have not been deleted */
        if (errCode == NETFP_ENOTREADY)
        {
            /* The NETFP clients are still active and have not been deleted; sleep and try again. */
            usleep(100);
            continue;
        }
        NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: Unable to delete the NETFP server [Error code %d]\n", errCode);
        return NULL;
    }

#if 0 //fzm: https://e2eprivate.ti.com/nokia_siemens_networks/k2_-_fsm4_-_fzm_-_lrc_nokia/f/191/t/5791.aspx
    /* Shutdown the PKTLIB heaps which had been created by the NETFP master */
    if (gNetfpMasterMCB.masterHeapHandle != NULL)
    {
        if (Pktlib_deleteHeap (gNetfpMasterMCB.pktlibInstanceHandle, gNetfpMasterMCB.masterHeapHandle, &errCode) < 0)
            NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: NETFP Master heap deletion failed [Error code %d]\n", errCode);
        else
            NetfpMaster_log(NetfpMaster_LogLevel_DEBUG, "Debug: NETFP Master heap deleted successfully\n");
    }
#endif //fzm

#if 0 //fzm: https://e2eprivate.ti.com/nokia_siemens_networks/k2_-_fsm4_-_fzm_-_lrc_nokia/f/191/t/5791.aspx
    /* Shutdown the eQoS module */
    if (NetfpMaster_deInitQoS(&gNetfpMasterMCB) < 0)
        NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: Unable to deinitialize the eQoS module\n");
    else
        NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Debug: Successfully deinitialized the eQoS module\n");

    /* Shutdown the PKTLIB instance */
    if (Pktlib_deleteInstance (gNetfpMasterMCB.pktlibInstanceHandle, &errCode) < 0)
    {
        NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: Unable to delete the PKTLIB instance [Error code %d]\n", errCode);
        return NULL;
    }

    /* Shutdown the MSGCOM instance */
    if (Msgcom_deleteInstance (gNetfpMasterMCB.msgcomInstanceHandle, &errCode) < 0)
    {
        NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: Unable to delete the MSGCOM instance [Error code %d]\n", errCode);
        return NULL;
    }

    /* Cleanup the resource manager configuration */
    if (Resmgr_deinit (gNetfpMasterMCB.handleSysCfg, &errCode) < 0)
        NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: Shutting down the system configuration failed\n");
    else
        NetfpMaster_log(NetfpMaster_LogLevel_DEBUG, "Debug: Shutting down the system configuration passed\n");
#endif //fzm

    return NULL;
}

/**
 *  @b Description
 *  @n
 *      Entry point into the NETFP master.
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
    struct sigaction    act;
    pthread_t           initThread;
    int32_t             errCode;

    /* Initialize the allocated memory block */
    memset ((void *)&gNetfpMasterMCB, 0, sizeof(NetfpMaster_MCB));

    /* By default the verbosity is turned off */
    gNetfpMasterMCB.logLevel = NetfpMaster_LogLevel_INFO;

//fzm-->
    /* Initialize the logger */
    if (NetfpMaster_logInit() < 0)
        return -1;
//fzm<--

    /* Use the sa_sigaction field because the handles has two additional parameters */
    act.sa_sigaction = &NetfpMaster_terminated;
    act.sa_flags     = SA_SIGINFO;

    /* Setup the signals. */
    sigaction(SIGTERM, &act, NULL);
    sigaction(SIGKILL, &act, NULL);
    sigaction(SIGINT,  &act, NULL);

    /* Setup the user defined signal to display the resource usage. */
    act.sa_sigaction = &NetfpMaster_displayInfo;
    act.sa_flags     = SA_SIGINFO;
    sigaction(SIGUSR1, &act, NULL);

    /* Cycle through the arguments and parse the command line arguments. */
    if (NetfpMaster_processCmdLineArgs(argc, argv) < 0)
    {
        NetfpMaster_displayUsage();
        NetfpMaster_logDeInit(); //fzm
        return -1;
    }

    /* Setup the signal pipes: */
    if (NetfpMaster_setupSignalPipe() < 0)
    {
        NetfpMaster_logDeInit(); //fzm
        return -1;
    }

    /* Launch the initialization thread: */
    errCode = pthread_create (&initThread, NULL, NetfpMaster_initThread, NULL);
    if (errCode < 0)
    {
        NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: Initialization thread create failed error code %d\n", errCode);
        NetfpMaster_logDeInit(); //fzm
        return -1;
    }

    /* Wait for the initialization thread to terminate. */
    pthread_join (initThread, NULL);
    NetfpMaster_logDeInit(); //fzm
    return 0;
}
