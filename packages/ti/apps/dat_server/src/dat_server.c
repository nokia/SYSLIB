/**
 *   @file  dat_server.c
 *
 *   @brief
 *      DAT Server implementation which executes on the ARM as a
 *      daemon and is responsible for interfacing with all the DAT
 *      clients executing on the DSP/ARM. The central database allows
 *      the DAT Services to now coexist and be reused across different
 *      execution realms.
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
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/uintc/uintc.h>
#include <ti/runtime/dat/dat.h>
#include <ti/runtime/name/name_db.h>
#include <ti/runtime/name/name_proxyClient.h>
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/apps/dat_server/include/dat_server.h>

/**********************************************************************
 ******************** DAT Server Local Definitions ********************
 **********************************************************************/

/**
 * @brief   Maximum number of clients which can be supported concurrently by the
 * DAT Server.
 */
#define DAT_MAX_CLIENTS                   16

/**********************************************************************
 ********************* DAT Server Data Structures *********************
 **********************************************************************/

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

/* DAT: */
extern void* Dat_osalMalloc (uint32_t , uint32_t );
extern void  Dat_osalFree (void* , uint32_t );
extern void* Dat_osalEnterSingleCoreCriticalSection (void);
extern void  Dat_osalExitSingleCoreCriticalSection (void* csHandle);
extern void  Dat_osalBeginMemoryAccess (void* ptr, uint32_t size);
extern void  Dat_osalEndMemoryAccess (void* ptr, uint32_t size);
extern void* Dat_osalCreateSem(void);
extern void  Dat_osalDeleteSem(void*);
extern void  Dat_osalPostSem(void*);
extern void  Dat_osalPendSem(void*);

/**********************************************************************
 *********************** Global Declarations **************************
 **********************************************************************/

/* DAT Server MCB */
DatServer_MCB         gDatServerMCB;

/* Application requested resources */
Resmgr_ResourceCfg      appResourceConfig =
{
    0,    /* Number of CPINTC Output  requested                               */
    0,    /* Number of Accumulator Channels requested                         */
    0,    /* Number of Hardware Semaphores requested                          */
    0,    /* Number of QPEND Queues requested                                 */
    /* Requested Memory Region Configuration. */
    {
        /* Name,             Type,                       Linking RAM,                           Num,        Size */
        { "DAT-ServerHeap",  Resmgr_MemRegionType_DDR3,  Resmgr_MemRegionLinkingRAM_DONT_CARE,  128,         128},
    }
};

/**********************************************************************
 *********************** DAT Server Functions ***********************
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
static void DatServer_displayUsage (void)
{
    printf ("DAT Server:\n");
    printf ("dat_server <arguments>\n");
    printf ("Mandatory Arguments: \n");
    printf ("-n <name>           - Name of the DAT server; unique in the system\n");
    printf ("-r <name>           - Name of the RM Client to be used; this should match the client names in the RMv2 Policy file\n");
    printf ("-i <InstantId>      - Named resource instant identifier\n");
    printf ("-c <dat.conf>       - DAT Server configuration file.\n");
    printf ("Optional Arguments: \n");
    printf ("-t <timeout>      - Polling Timeout specified in msec [Default is 100msec]\n");
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
static int32_t DatServer_processCmdLineArgs(int32_t argc, char* argv[])
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
            {"nameProxyName",   required_argument, 0,  0 },
            {0,                 0,                 0,  0 }
        };

        c = getopt_long(argc, argv, "vn:r:t:c:p:i:", long_options, &option_index);
        if (c == -1)
            break;

       switch (c)
       {
            case 'n':
            {
                /* Server Name: */
                strncpy (gDatServerMCB.serverName, optarg, sizeof(gDatServerMCB.serverName));
                break;
            }
            case 'r':
            {
                /* RM Client Name: */
                strncpy (gDatServerMCB.rmClientName, optarg, sizeof(gDatServerMCB.rmClientName));
                break;
            }
            case 'c':
            {
                /* DAT Client List File name. */
                gDatServerMCB.clientListFile = optarg;
                break;
            }
            case 'i':
            {
                /* Named resource instance identifier. */
                gDatServerMCB.nrInstanceId = atoi (optarg);
                break;
            }
            case 't':
            {
                /* Polling Timeout: This is specified in micro-seconds */
                gDatServerMCB.pollingTimeout = atoi (optarg);
                gDatServerMCB.pollingTimeout = gDatServerMCB.pollingTimeout * 1000;
                break;
            }
            case 'v':
            {
                gDatServerMCB.logLevel = DatServer_LogLevel_DEBUG;
                break;
            }
            case '?':
            {
                return -1;
            }
        }
    }

    /* We should have a valid DAT server name */
    if (gDatServerMCB.serverName[0] == 0)
        return -1;

    /* We should have a valid RM client name */
    if (gDatServerMCB.rmClientName[0] == 0)
        return -1;

    /* We should have a valid client list. */
    if (gDatServerMCB.clientListFile == NULL)
        return -1;

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
void DatServer_log (DatServer_LogLevel level, char* fmt, ...)
{
    va_list arg;

    /* Log the message as per the configured level. */
    if (level >= gDatServerMCB.logLevel)
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
 *      Logging function which is registered with the DAT Server library
 *
 *  @param[in]  logLevel
 *      DAT Log Level
 *  @param[in]  fmt
 *      Formatted string
 *  @param[in]  arg
 *      Variable length argument list
 *
 *  @retval
 *      Not applicable
 */
static void DatServer_LogFxn (Dat_LogLevel logLevel, char* fmt, va_list arg)
{
    /* If the DAT library has reported an error or informational message; we always log this */
    if ((logLevel == Dat_LogLevel_ERROR) || (logLevel == Dat_LogLevel_INFO))
    {
        vprintf (fmt, arg);
        return;
    }

    /* If the DAT library has reported a debug message and the server is executing
     * with the verbosity on. Log the message. */
    if ((logLevel == Dat_LogLevel_DEBUG) && (gDatServerMCB.logLevel == DatServer_LogLevel_DEBUG))
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
static void DatServer_terminated (int sig, siginfo_t *siginfo, void *context)
{
    /* Server is going down. */
    DatServer_log(DatServer_LogLevel_DEBUG, "Debug: Signal received\n");
    gDatServerMCB.serverState = 0;
    return;
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
static void DatServer_displayInfo (int sig, siginfo_t *siginfo, void *context)
{
    uint32_t index;

    /* Display the client status of each NETFP client. */
    for (index = 0; index < gDatServerMCB.datClientCount; index++)
        DatServer_log(Dat_LogLevel_INFO, "Debug: Client Status [%s] is %d\n", gDatServerMCB.datClientList[index], gDatServerMCB.datClientStatus[index]);

    Dat_displayServer(gDatServerMCB.datServerHandle);
}

/**
 *  @b Description
 *  @n
 *      The function is used to parse the client list file and generate a list of
 *      all the DAT client names which could attach to the DAT Server. The
 *      names of all the DAT clients are placed into the client database
 *
 *  @retval
 *      Success -   Number of DAT clients
 *  @retval
 *      Error   -   <0
 */
static int32_t DatServer_parseClientList (void)
{
    FILE*       fp;
    int32_t     datClientCount;
    char*       fileBuffer;
    struct stat fileStat;
    int32_t     size;
    char*       tokenName;
    char*       tokenValue;
    const char* delimitters = "=,;\n\r";
    char*       ptrFileBuffer;

    /* Get the file statistics for the file. */
    if (stat(gDatServerMCB.clientListFile, &fileStat) < 0)
    {
        perror ("Error: Unable to get file stat for the client list file\n");
        return -1;
    }
    DatServer_log(DatServer_LogLevel_DEBUG, "Debug: Client List file is %d bytes\n", fileStat.st_size);

    /* Allocate memory for the buffer */
    fileBuffer = malloc (fileStat.st_size + 1);
    if (fileBuffer == NULL)
    {
        DatServer_log(DatServer_LogLevel_ERROR, "Error: Out of memory while parsing the client list file [%d bytes]\n", fileStat.st_size);
        return -1;
    }

    /* Open the DAT configuration file for the tests */
    fp = fopen (gDatServerMCB.clientListFile, "r");
    if (fp == NULL)
    {
        DatServer_log(DatServer_LogLevel_ERROR, "Error: Unable to open client list '%s'\n", gDatServerMCB.clientListFile);
        return -1;
    }

    /* Initialize the variables: */
    ptrFileBuffer = fileBuffer;
    size          = 0;

    /* Read data from the file and place it into the buffer */
    while (1)
    {
        *fileBuffer++ = fgetc(fp);
        size++;
        if (size == fileStat.st_size)
            break;
    }

    /* Once all the data has been placed into the buffer; reset the buffer pointer */
    fileBuffer = ptrFileBuffer;

    /* Close the file. */
    fclose (fp);

    /* Parse the DAT client list from the file: Initialize the variables */
    size            = 0;
    datClientCount  = 0;

    /* Run through the entire file. */
    while (size < fileStat.st_size)
    {
        /* Get the DAT Client Name. */
        tokenName = strtok(ptrFileBuffer, delimitters);
        if (tokenName == NULL)
            break;

        /* Subsequent calls to the strtok API requires NULL parameters. */
        ptrFileBuffer = NULL;
        size = size + strlen(tokenName);

        /* Process and parse the buffer */
        if (*tokenName == '#' || *tokenName == '\n' || *tokenName == '\r')
            continue;

        /* Get the Token Value */
        tokenValue = strtok(NULL, delimitters);
        size = size + strlen(tokenValue);

        /* Process the tokens */
        if (strcmp (tokenName, "CLIENT") == 0)
        {
            /* Copy the client name into the DAT Client database. */
            strcpy (gDatServerMCB.datClientList[datClientCount], tokenValue);
            datClientCount++;
        }
        else
        {
            /* Error: Invalid Token */
            DatServer_log(DatServer_LogLevel_ERROR, "Error: Invalid token detected '%s'\n", tokenName);
            return -1;
        }
    }

    /* Display the parsed information: */
    for (size = 0; size < datClientCount; size++)
        DatServer_log(DatServer_LogLevel_DEBUG, "Debug: DAT Client Name %d = '%s'\n", size, gDatServerMCB.datClientList[size]);

    /* Cleanup the allocated memory */
    free (ptrFileBuffer);
    return datClientCount;
}

/**
 *  @b Description
 *  @n
 *      This is the DAT Server thread which executes the DAT Server
 *
 *  @param[in]  arg
 *      Argument passed to the thread.
 *
 *  @retval
 *      Not Applicable.
 */
static void* DatServer_thread(void *arg)
{
    int32_t errCode;

    /* Debug Message: */
    DatServer_log(DatServer_LogLevel_DEBUG, "Debug: DAT Server execution thread started\n");

    /* DAT Server executes in a polled mode: Polling Timeout is specified by the application */
    while (1)
    {
        /* Execute the server. */
        Dat_executeServer (gDatServerMCB.datServerHandle);

        /* Has there been a request to stop the DAT Server. */
        if (gDatServerMCB.serverState == 0)
        {
            /* Delete the DAT Server: The deletion will not go through if DAT clients are still
             * attached to the DAT Server */
            if (Dat_deleteServer (gDatServerMCB.datServerHandle, &errCode) == 0)
            {
                /* DAT Server deleted successful. We are done with the thread execution. */
                DatServer_log(DatServer_LogLevel_DEBUG, "Debug: DAT server has been deleted successfully. \n");
                break;
            }

            /* The DAT Server deletion can fail if the DAT clients have not been deleted */
            if (errCode != DAT_ENOTREADY)
                DatServer_log(DatServer_LogLevel_ERROR, "Error: Unable to delete the DAT server [Error code %d]\n", errCode);

        }
        /* Relinquish time slice as per the polling timeout. */
        usleep(gDatServerMCB.pollingTimeout);
    }
    DatServer_log(Dat_LogLevel_DEBUG, "Exiting from DatServer_thread!\n");
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      This is the DAT management thread which registers DAT clients. The thread
 *      is executed periodically every polling tick.
 *
 *  @param[in]  arg
 *      Argument passed to the thread.
 *
 *  @retval
 *      Not Applicable.
 */
static void* DatServer_mgmtThread(void *arg)
{
    pthread_t                   datThread;
    int32_t                     errCode;
    int32_t                     count;
    int32_t                     retVal;

    /* Debug Message: */
    DatServer_log(DatServer_LogLevel_DEBUG, "Debug: DAT server client management thread started\n");

    /* Create the DAT Server thread. */
    errCode = pthread_create (&datThread, NULL, DatServer_thread, NULL);
    if (errCode < 0)
    {
        DatServer_log(DatServer_LogLevel_ERROR, "Error: DAT Server thread create failed error code %d\n", errCode);
        return NULL;
    }

    /* Execute the DAT Server: Until a termination signal is detected. */
    while (gDatServerMCB.serverState == 1)
    {
        /* Cycle through all the clients and register them if possible */
        for (count = 0; count < gDatServerMCB.datClientCount; count++)
        {

            /* Has the client already been deregistered? */
            if (gDatServerMCB.datClientStatus[count] == 2)
                continue;

            /* Has the client already been registered? */
            if (gDatServerMCB.datClientStatus[count] == 1)
            {
                /* YES. Client has already been registered. So here we check and make sure that the client is still active? */
                retVal = Dat_isClientActive(gDatServerMCB.datServerHandle, gDatServerMCB.datClientList[count], &errCode);
                if (retVal < 0)
                {
                    /* FATAL Error: */
                    DatServer_log(DatServer_LogLevel_ERROR, "Error: DAT Server Client %s Active Status failed %d\n",
                                  gDatServerMCB.datClientList[count], errCode);
                    return NULL;
                }
                if (retVal == 0)
                {
                    /* Client is no longer active and needs to be deregistered from the DAT Server MCB also */
                    if (Dat_deregisterClient (gDatServerMCB.datServerHandle, gDatServerMCB.datClientList[count], &errCode) == 0)
                    {
                        /* DAT client has been deregistered successfully. Once the client is deregistered we are not
                           allowing to register again */
                        gDatServerMCB.datClientStatus[count] = 2;
                        DatServer_log(DatServer_LogLevel_DEBUG, "Debug: DAT Client %s has been deregistered\n",
                                      gDatServerMCB.datClientList[count]);
                        continue;
                    }
                    else
                    {
                        DatServer_log(DatServer_LogLevel_ERROR, "Error: DAT deregister client failed [Error code %d]\n", errCode);
                    }
                }
                else
                {
                    /* DAT client is still active; so there is nothing else which needs to be done. */
                }
                continue;
            }

            /* Try and register the DAT client.  */
            if (Dat_registerClient (gDatServerMCB.datServerHandle, gDatServerMCB.datClientList[count],  &errCode) == 0)
            {
                /* DAT client registeration was successful. */
                gDatServerMCB.datClientStatus[count] = 1;
                DatServer_log(DatServer_LogLevel_DEBUG, "Debug: DAT Client '%s' registered\n", gDatServerMCB.datClientList[count]);
            }
            else
            {
                /* Registration failed; use the error code to determine the reason. */
                if (errCode != DAT_ENOTREADY)
                {
                    DatServer_log(DatServer_LogLevel_ERROR, "Error: DAT Register client '%s' failed [Error code %d]\n",
                                  gDatServerMCB.datClientList[count], errCode);
                }
            }
        }
        usleep (gDatServerMCB.pollingTimeout);
    }

    /* Wait for the DAT server thread to terminate. */
    pthread_join (datThread, NULL);

    /* Debug Mesage: */
    DatServer_log(DatServer_LogLevel_DEBUG, "Debug: DAT server client management thread shutting down\n");
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      Entry point into the DAT server.
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
    Resmgr_SystemCfg            sysConfig;
    Name_DatabaseCfg            databaseCfg;
    struct sigaction            act;
    int32_t                     retVal;
    pthread_t                   serverMgmtThread;
    Dat_ServerCfg               datServerCfg;
    Pktlib_InstCfg              pktlibInstCfg;
    Msgcom_InstCfg              msgcomInstCfg;

    DatServer_log(Dat_LogLevel_DEBUG, "Debug: Starting up DAT server\n");

    /* Initialize the DAT server MCB */
    memset ((void *)&gDatServerMCB, 0, sizeof(DatServer_MCB));

    /* Setup the defaults.
     *  - Polling Timeout is 100msec. */
    gDatServerMCB.logLevel            = DatServer_LogLevel_ERROR;
    gDatServerMCB.pollingTimeout      = 10*1000;

    /* Use the sa_sigaction field because the handles has two additional parameters */
    act.sa_sigaction = &DatServer_terminated;
    act.sa_flags     = SA_SIGINFO;

    /* Setup the signals. */
    sigaction(SIGTERM, &act, NULL);
    sigaction(SIGKILL, &act, NULL);
    sigaction(SIGINT,  &act, NULL);

    /* Setup the user defined signal to display the resource usage. */
    act.sa_sigaction = &DatServer_displayInfo;
    act.sa_flags     = SA_SIGINFO;
    sigaction(SIGUSR1, &act, NULL);

    /* Process the command line arguments: */
    if (DatServer_processCmdLineArgs(argc, argv) < 0)
    {
        DatServer_displayUsage();
        return -1;
    }

    /* Initialize the database configuration */
    memset ((void *)&databaseCfg, 0, sizeof(Name_DatabaseCfg));

    /* Populate the configuration: */
    databaseCfg.instanceId  = gDatServerMCB.nrInstanceId;
    databaseCfg.realm       = Name_ExecutionRealm_ARM;
    strcpy (databaseCfg.owner, gDatServerMCB.serverName);

    /* Create the database */
    gDatServerMCB.databaseHandle = Name_createDatabase (&databaseCfg, &errCode);
    if (gDatServerMCB.databaseHandle == NULL)
    {
        DatServer_log(Dat_LogLevel_ERROR, "Error: Name database creation failed [Error code %d]\n", errCode);
        return -1;
    }
    DatServer_log(Dat_LogLevel_DEBUG, "Debug: Name database created successfully [Handle %p]\n", gDatServerMCB.databaseHandle);

    /* Parse the DAT Client List */
    gDatServerMCB.datClientCount = DatServer_parseClientList();
    if (gDatServerMCB.datClientCount <= 0)
    {
        DatServer_log(DatServer_LogLevel_ERROR, "Error: Parsing client list yielded %d clients; aborting\n",
                        gDatServerMCB.datClientCount);
        return -1;
    }

    /* Initialize the system configuration. */
    memset ((void *)&sysConfig, 0, sizeof(Resmgr_SystemCfg));

    /* Populate the configuration.*/
    sysConfig.realm                 = Resmgr_ExecutionRealm_ARM;
    strcpy (sysConfig.rmClient, gDatServerMCB.rmClientName);
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
    gDatServerMCB.handleSysCfg = Resmgr_init(&sysConfig, &errCode);
    if (gDatServerMCB.handleSysCfg == NULL)
    {
        DatServer_log(DatServer_LogLevel_ERROR, "Error: System configuration initialization failed [Error code %d]\n", errCode);
        return -1;
    }
    DatServer_log(DatServer_LogLevel_DEBUG, "Debug: DAT Server system configuration has been processed.\n");

    /* Process the configuration */
    if (Resmgr_processConfig (gDatServerMCB.handleSysCfg, &appResourceConfig, &errCode) < 0)
    {
        DatServer_log(DatServer_LogLevel_ERROR, "Error: SYSRM configuration failed with error code %d\n", errCode);
        return -1;
    }

    /* Get the pointer to the application resource configuration */
    ptrResCfg = &appResourceConfig;

    /* Initialize the PKTLIB instance configuration. */
    memset ((void *)&pktlibInstCfg, 0, sizeof(Pktlib_InstCfg));

    /* Populate the PKTLIB Instance configuration: */
    pktlibInstCfg.instanceId        = 1;
    pktlibInstCfg.databaseHandle    = gDatServerMCB.databaseHandle;
    pktlibInstCfg.sysCfgHandle      = gDatServerMCB.handleSysCfg;
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
    gDatServerMCB.pktlibInstanceHandle = Pktlib_createInstance (&pktlibInstCfg, &errCode);
    if (gDatServerMCB.pktlibInstanceHandle == NULL)
    {
        DatServer_log(DatServer_LogLevel_ERROR, "Error: Creating the PKTLIB instance failed [Error code %d]\n", errCode);
        return -1;
    }

    /* Initialize the heap configuration */
    memset((void *)&heapCfg, 0 , sizeof(Pktlib_HeapCfg));

    /* Populate the heap configuration */
    snprintf(heapCfg.name, PKTLIB_MAX_CHAR, "%s-SrvHeap", gDatServerMCB.serverName);
    heapCfg.memRegion                       = ptrResCfg->memRegionResponse[0].memRegionHandle;
    heapCfg.pktlibInstHandle                = gDatServerMCB.pktlibInstanceHandle;
    heapCfg.sharedHeap                      = 0;
    heapCfg.useStarvationQueue              = 0;
    heapCfg.dataBufferSize                  = 1536;
    heapCfg.numPkts                         = ptrResCfg->memRegionResponse[0].memRegionCfg.numDesc;
    heapCfg.numZeroBufferPackets            = 0;
    heapCfg.dataBufferPktThreshold          = 0;
    heapCfg.zeroBufferPktThreshold          = 0;
    heapCfg.heapInterfaceTable.dataMalloc   = myMalloc;
    heapCfg.heapInterfaceTable.dataFree     = myFree;

    /* Create the DAT Server-Client Heap: This heap is used to exchage control messages between the DAT Server & Clients. */
    gDatServerMCB.datServerPktlibHeapHandle = Pktlib_createHeap(&heapCfg, &errCode);
    if (gDatServerMCB.datServerPktlibHeapHandle == NULL)
    {
        DatServer_log(DatServer_LogLevel_ERROR, "Error: Unable to create the DAT Server/Client heap [Error code %d]\n", errCode);
        return -1;
    }
    DatServer_log(DatServer_LogLevel_DEBUG, "Debug: DAT Server/Client Heap %s has been created successfully\n", heapCfg.name);

    /* Initialize the MSGCOM instance. */
    memset ((void *)&msgcomInstCfg, 0, sizeof(Msgcom_InstCfg));

    /* Populate the MSGCOM Instance configuration */
    msgcomInstCfg.databaseHandle    = gDatServerMCB.databaseHandle;
    msgcomInstCfg.sysCfgHandle      = gDatServerMCB.handleSysCfg;
    msgcomInstCfg.pktlibInstHandle  = gDatServerMCB.pktlibInstanceHandle;
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
    gDatServerMCB.msgcomInstanceHandle = Msgcom_createInstance (&msgcomInstCfg, &errCode);
    if (gDatServerMCB.msgcomInstanceHandle == NULL)
    {
        DatServer_log(Dat_LogLevel_ERROR, "Error: MSGCOM Instance creation failed [Error code %d]\n", errCode);
        return -1;
    }

    /* DAT Server has been started */
    gDatServerMCB.serverState = 1;

    /* Initialize the DAT server configuration */
    memset ((void *)&datServerCfg, 0, sizeof(Dat_ServerCfg));

    /* Populate the DAT Server configuration
     * - Named resource instance identifier is set to 1; the same as named resource domain initialized above */
    strcpy (datServerCfg.serverName, gDatServerMCB.serverName);
    datServerCfg.nrInstanceId           = gDatServerMCB.nrInstanceId;
    datServerCfg.pktlibInstHandle       = gDatServerMCB.pktlibInstanceHandle;
    datServerCfg.msgcomInstHandle       = gDatServerMCB.msgcomInstanceHandle;
    datServerCfg.realm                  = Dat_ExecutionRealm_ARM;
    datServerCfg.logFxn                 = DatServer_LogFxn;
    datServerCfg.serverHeapHandle       = gDatServerMCB.datServerPktlibHeapHandle;
    datServerCfg.malloc                 = Dat_osalMalloc;
    datServerCfg.free                   = Dat_osalFree;
    datServerCfg.beginMemAccess         = Dat_osalBeginMemoryAccess;
    datServerCfg.endMemAccess           = Dat_osalEndMemoryAccess;
    datServerCfg.createSem              = Dat_osalCreateSem;
    datServerCfg.deleteSem              = Dat_osalDeleteSem;
    datServerCfg.postSem                = Dat_osalPostSem;
    datServerCfg.pendSem                = Dat_osalPendSem;
    datServerCfg.enterCS                = Dat_osalEnterSingleCoreCriticalSection;
    datServerCfg.exitCS                 = Dat_osalExitSingleCoreCriticalSection;

    /* Initialize the DAT Server */
    gDatServerMCB.datServerHandle = Dat_initServer (&datServerCfg, &errCode);
    if (gDatServerMCB.datServerHandle == NULL)
    {
        DatServer_log(DatServer_LogLevel_ERROR, "Error: Unable to create the DAT server [Error code %d]\n", errCode);
        return -1;
    }
    DatServer_log(DatServer_LogLevel_DEBUG, "Debug: DAT Server '%s' is operational\n", gDatServerMCB.datServerHandle);

    /* Launch the DAT Server management thread */
    retVal = pthread_create (&serverMgmtThread, NULL, DatServer_mgmtThread, NULL);
    if (retVal < 0)
    {
        DatServer_log(DatServer_LogLevel_ERROR, "Error: Server thread create failed error code %d\n", retVal);
        return -1;
    }

    /* Wait for the management thread to terminate. */
    pthread_join (serverMgmtThread, NULL);

    /* Shutdown the PKTLIB heaps which had been created by the DAT Server */
    if (Pktlib_deleteHeap (gDatServerMCB.pktlibInstanceHandle, gDatServerMCB.datServerPktlibHeapHandle, &errCode) < 0)
        DatServer_log(DatServer_LogLevel_ERROR, "Error: DAT Server/Client heap deletion failed [Error code %d]\n", errCode);

    /* Shutdown the MSGCOM instance */
    if (Msgcom_deleteInstance (gDatServerMCB.msgcomInstanceHandle, &errCode) < 0)
    {
        DatServer_log(DatServer_LogLevel_ERROR, "Error: Unable to delete the MSGCOM instance [Error code %d]\n", errCode);
        return -1;
    }

    /* Shutdown the PKTLIB instance */
    if (Pktlib_deleteInstance (gDatServerMCB.pktlibInstanceHandle, &errCode) < 0)
    {
        DatServer_log(DatServer_LogLevel_ERROR, "Error: Unable to delete the PKTLIB instance [Error code %d]\n", errCode);
        return -1;
    }

    /* Cleanup the resource manager configuration */
    if (Resmgr_deinit (gDatServerMCB.handleSysCfg, &errCode) < 0)
        DatServer_log(DatServer_LogLevel_ERROR, "Error: Shutting down the system configuration failed\n");
    else
        DatServer_log(DatServer_LogLevel_DEBUG, "Debug: Shutting down the system configuration passed\n");
    return 0;
}


