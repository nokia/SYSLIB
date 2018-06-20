/**
 *   @file  resmgr_server.c
 *
 *   @brief
 *      This is the implementation of the SYSLIB-RM server which executes
 *      on the ARM. The SYSLIB RM server should be executed immediately
 *      since it will manage the resource requests from clients across
 *      different realms.
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

/**********************************************************************
 *************************** Include Files ****************************
 **********************************************************************/

/* Standard Include Files. */
//fzm---->
#include <ddal/ddal_common.h>
#include <ddal/ddal_cma.h>
#include <systemd/sd-daemon.h>
//<----fzm
#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <getopt.h>

/* MCSDK Include Files */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>

/* RM Server Include Files */
#include <ti/apps/resmgr_server/include/listlib.h>
#include <ti/apps/resmgr_server/include/resmgr_server.h>

// <fzm>
#define RESMGR_SERVER_CONF_FILE "/rom/private/resmgr_server.conf"

extern int32_t ResmgrServer_logInit(void);
extern int32_t ResmgrServer_logDeInit(void);
extern void ResmgrServer_smlogMsg(ResmgrServer_LogLevel logLevel, const char* fmt, va_list arg) __attribute__ ((format (printf, 2, 0)));
extern int ResmgrServer_dumpInit(void);
extern void ResmgrServer_dumpDeInit(void);
extern void ResmgrServer_dumpMsg(const char* fmt, va_list arg) __attribute__ ((format (printf, 1, 0)));
extern void ResmgrServer_setLogLevel(ResmgrServer_LogLevel level);
extern ResmgrServer_LogLevel ResmgrServer_getLogLevel(void);
// </fzm>

/**********************************************************************
 ************************** Global Variables **************************
 **********************************************************************/

/* Global Resource Manager Server Master Control block*/
ResmgrServer_MCB    gResmgrServerMCB;

int32_t             gPankajDebugCounter = 0;

/**********************************************************************
 ********************** Resource Manager Functions ********************
 **********************************************************************/

/** @addtogroup RESMGR_FUNCTION
 @{ */

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

// <fzm>
/**
 *  @b Description
 *  @n
 *      The function is used to parse the configuration file
 */
void ResmgrServer_parseConfigFile(void)
{
    FILE*      fp;
    const char* delimitters = "=,;\r\n";
    char*      line = NULL;
    size_t     len = 0;
    int        newLogLevel;

    /* Try to open the configuration file */
    fp = fopen (RESMGR_SERVER_CONF_FILE, "r");
    if (fp == NULL)
    {
        return;
    }

    ResmgrServer_log(ResmgrServer_LogLevel_INFO, "Info: Parsing %s file", RESMGR_SERVER_CONF_FILE);

    while (getline(&line, &len, fp) != -1)
    {
        char *tokenName = strtok(line, delimitters);
        char *tokenValue = strtok(NULL, delimitters);

        if(tokenName == NULL || tokenValue == NULL)
            continue;

        if ((*tokenName == '#') || (*tokenName == '\n') || (*tokenName == '\r'))
            continue;

        if (strcmp (tokenName, "logLevel") == 0)
        {
            newLogLevel = atoi(tokenValue);

            if ( (ResmgrServer_LogLevel_VERBOSE <= newLogLevel) && (newLogLevel <= ResmgrServer_LogLevel_ERROR) )
            {
                ResmgrServer_log (ResmgrServer_LogLevel_INFO, "Info: Changing logging level %d->%d", ResmgrServer_getLogLevel(), newLogLevel);
                ResmgrServer_setLogLevel(newLogLevel);
            }
            else
            {
                ResmgrServer_log (ResmgrServer_LogLevel_ERROR, "Error: Not supported logging level %d", newLogLevel);
            }
        }
        else
        {
            /* Error: Invalid Token */
            ResmgrServer_log (ResmgrServer_LogLevel_ERROR, "Error: Invalid token detected '%s'", tokenName);
        }
    }
    fclose(fp);

    //free the buffer allocated in getline
    free(line);
}
// </fzm>

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
void ResmgrServer_log (ResmgrServer_LogLevel level, const char* fmt, ...)
{
	va_list arg;

    /* Log the message as per the configured level. */
    if (level >= gResmgrServerMCB.logLevel)
    {
    	/* Log the message on the console. */
    	va_start (arg, fmt);
        ResmgrServer_smlogMsg(level, fmt, arg); //fzm
    	va_end (arg);
    }
	return;
}

//fzm-->
void ResmgrServer_dump(ResmgrServer_LogLevel level, const char* fmt, ...)
{
    va_list arg;

    va_start(arg, fmt);
    ResmgrServer_dumpMsg(fmt, arg);
    va_end(arg);

    return;
}
//fzm<--

/**
 *  @b Description
 *  @n
 *     Display the usage for the resource manager server
 *
 *  @retval
 *      Not Applicable.
 */
static void ResmgrServer_displayUsage (void)
{
    printf ("SYSLIB Resource Server:\n");
    printf ("resmgr_server <arguments>\n");
    printf ("Mandatory Arguments: \n");
    printf ("-n <name>          - Name of the SYSLIB Resource server; unique in the system\n");
    printf ("-g <resource.dtb>  - Global Resource DTB file\n");
    printf ("-p <policy.dtb>    - Server Policy DTB file\n");
    printf ("-s <DSP Source Id> - DSP source identifier which is used to generated interrupts from the DSP\n");
    printf ("Optional Arguments: \n");
    printf ("-a <address>      - Shared memory address for DSP clients [Default is 0xA0001000]\n");
    printf ("-v                - Verbosity\n");
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to find the specific RM Client using the name
 *
 *  @param[in]  clientName
 *      RM Client name
 *
 *  @retval
 *      Success -   Pointer to the client
 *  @retval
 *      Error   -   NULL
 */
static ResmgrServer_Client* ResmgrServer_findClient(const char* clientName)
{
    ResmgrServer_Client*    ptrClient;

    /* Get the list of clients. */
    ptrClient = (ResmgrServer_Client*)ResmgrServer_listGetHead((ResmgrServer_ListNode**)&gResmgrServerMCB.ptrClientList);
    while (ptrClient != NULL)
    {
        /* Does the name match */
        if (strcmp (clientName, ptrClient->clientName) == 0)
            return ptrClient;

        /* Get the next client. */
        ptrClient = (ResmgrServer_Client*)ResmgrServer_listGetNext((ResmgrServer_ListNode*)ptrClient);
    }

    /* Control comes here implies that there was no matching client name. */
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      Signal Handler installed to catch the SIGINT signal
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
static void ResmgrServer_terminated (int sig, siginfo_t *siginfo, void *context)
{
    uint32_t    terminate = 0xdead;

    /* Server is going down. */
    ResmgrServer_log(ResmgrServer_LogLevel_INFO, "Debug: Termination signal received\n"); //fzm

    /* Send a message via the pipe to the main resource manager thread. */
    write (gResmgrServerMCB.signalPipe[1], &terminate, sizeof(terminate));
    return;
}

/**
 *  @b Description
 *  @n
 *      Signal Handler installed to catch the SIGUSR1 signal
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
static void ResmgrServer_displayResourceUsage (int sig, siginfo_t *siginfo, void *context)
{
    uint32_t    display = 0xbeef;

    ResmgrServer_log(ResmgrServer_LogLevel_DEBUG, "Debug: User defined signal received\n");

    /* Send a message via the pipe to the main resource manager thread. */
    write (gResmgrServerMCB.signalPipe[1], &display, sizeof(display));
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
static int32_t ResmgrServer_processCmdLineArgs(int32_t argc, char* argv[])
{
    while (1)
    {
        int option_index = 0;
        int c;

        static struct option long_options[] = {
            {"name",    required_argument, 0,  0 },
            {"global",  required_argument, 0,  0 },
            {"address", required_argument, 0,  0 },
            {"policy",  required_argument, 0,  0 },
            {"srcId",   required_argument, 0,  0 },
            {0,         0,                 0,  0 }
        };

        c = getopt_long(argc, argv, "vn:g:s:p:d:a:", long_options, &option_index);
        if (c == -1)
            break;

       switch (c)
       {
            case 'n':
            {
                /* Server Name: */
                strncpy (gResmgrServerMCB.serverName, optarg, sizeof(gResmgrServerMCB.serverName));
                break;
            }
            case 'a':
            {
                /* Shared Memory Address: */
                sscanf (optarg, "0x%x", &gResmgrServerMCB.sharedMemoryAddress);
                break;
            }
            case 'g':
            {
                /* Validate the argument. */
                gResmgrServerMCB.globalResourceFileDescriptor = open(optarg, O_RDONLY);
                if (gResmgrServerMCB.globalResourceFileDescriptor == -1)
                {
                    ResmgrServer_log(ResmgrServer_LogLevel_ERROR, "Error: Global Resource List file '%s' could not be opened\n", optarg);
                    return -1;
                }
                break;
            }
            case 's':
            {
                /* Validate the argument. */
                sscanf (optarg, "%d", &gResmgrServerMCB.dspSrcId);
                if (gResmgrServerMCB.dspSrcId > 27)
                {
                    ResmgrServer_log(ResmgrServer_LogLevel_ERROR, "Error: Invalid source id (%s) range [0-27]\n", optarg);
                    return -1;
                }
                break;
            }
            case 'p':
            {
                /* Open the server policy file and ensure that the file exists. */
                gResmgrServerMCB.serverPolicyFileDescriptor = open(optarg, O_RDONLY);
                if (gResmgrServerMCB.serverPolicyFileDescriptor == -1)
                {
                    ResmgrServer_log(ResmgrServer_LogLevel_ERROR, "Error: Server Policy file '%s' could not be opened\n", optarg);
                    return -1;
                }
                break;
            }
            case 'v':
            {
                gResmgrServerMCB.logLevel = ResmgrServer_LogLevel_DEBUG;
                break;
            }
            case '?':
            {
                return -1;
            }
        }
    }

    /* Sanity Check: Ensure that all the required parameters were specified. */
    if ((gResmgrServerMCB.globalResourceFileDescriptor == 0)     ||
        (gResmgrServerMCB.serverPolicyFileDescriptor   == 0))
    {
        return -1;
    }

#ifdef RM_DSP_CLIENT_SPT
    /* Sanity Check: DSP source identifier should be specified */
    if (gResmgrServerMCB.dspSrcId == 0xFF)
        return -1;
#endif

    /* Command Line Arguments processed successfully. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to initialize and setup the resource manager server
 *
 *  @retval
 *      Success 	- 0
 *  @retval
 *      Error		- <0
 */
static int32_t ResmgrServer_initRM (void)
{
    struct stat         file_stat;
    Rm_InitCfg          rmInitCfg;
    int32_t             rmResult;
    void*               globalResourceAddress;
    void*               serverPolicyAddress;

    //fzm --->
    //initialize the ddal cma subsystem (mmaps /dev/cma into our address space)
    int ddalCmaInitStatus = ddal_cma_process_init();
    if(ddalCmaInitStatus != DDAL_OK)
    {
        ResmgrServer_log(ResmgrServer_LogLevel_ERROR, "Failed to initialize DDAL CMA, errcode: %d", ddalCmaInitStatus);
        return -1;
    }
    //<---- fzm

    /***********************************************************************************************
     * RMv2 Server Initializations:
     ***********************************************************************************************/

    /* Obtain file size for the global resource list */
    if (fstat(gResmgrServerMCB.globalResourceFileDescriptor, &file_stat) == -1)
    {
        ResmgrServer_log(ResmgrServer_LogLevel_ERROR,
                         "Error: fstat [Global resource list] returned failure with error code %s\n",
                         strerror(errno));
        return -1;
    }

    /* Map to the virtual address */
    globalResourceAddress = mmap(NULL, file_stat.st_size, PROT_READ, MAP_PRIVATE, gResmgrServerMCB.globalResourceFileDescriptor, 0);
    if (globalResourceAddress == MAP_FAILED)
    {
        ResmgrServer_log(ResmgrServer_LogLevel_ERROR,
                         "Error: mmap [Global resource list] failed with error code %s\n",
                         strerror(errno));
        return -1;
    }

    /* Obtain file size for the global resource list */
    if (fstat(gResmgrServerMCB.serverPolicyFileDescriptor, &file_stat) == -1)
    {
        ResmgrServer_log(ResmgrServer_LogLevel_ERROR,
                         "Error: fstat [Server Policy] returned failure with error code %s\n",
                         strerror(errno));
        return -1;
    }

    /* Map to the virtual address */
    serverPolicyAddress = mmap(NULL, file_stat.st_size, PROT_READ, MAP_PRIVATE, gResmgrServerMCB.serverPolicyFileDescriptor, 0);
    if (serverPolicyAddress == MAP_FAILED)
    {
        ResmgrServer_log(ResmgrServer_LogLevel_ERROR,
                         "Error: mmap [Server Policy] failed with error code %s\n",
                         strerror(errno));
        return -1;
    }

    /* Initialize the RM Server initialization configuration. */
    memset(&rmInitCfg, 0, sizeof(Rm_InitCfg));

    /* Populate the RM Server initialization configuration. */
    rmInitCfg.instName                             = gResmgrServerMCB.serverName;
    rmInitCfg.instType                             = Rm_instType_SERVER;
    rmInitCfg.instCfg.serverCfg.globalResourceList = globalResourceAddress;
    rmInitCfg.instCfg.serverCfg.linuxDtb           = 0;
    rmInitCfg.instCfg.serverCfg.globalPolicy       = serverPolicyAddress;

    /* Initialize the RM Server. */
    gResmgrServerMCB.rmServerHandle = Rm_init(&rmInitCfg, &rmResult);
    if (rmResult != RM_OK)
    {
        ResmgrServer_log(ResmgrServer_LogLevel_ERROR, "Error: RM Server Initialization failed [Error code %d]\n", rmResult);
        return -1;
    }

    /* Initialize the RM Server Service */
    gResmgrServerMCB.rmServerServiceHandle = Rm_serviceOpenHandle(gResmgrServerMCB.rmServerHandle, &rmResult);
    if (rmResult != RM_OK)
    {
        ResmgrServer_log(ResmgrServer_LogLevel_ERROR, "Error: RM Server Service Initialization failed\n");
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to initialize and setup the communication channels
 *      which are required for the clients to communicate with the RM server.
 *
 *  @retval
 *      Success 	- 0
 *  @retval
 *      Error		- <0
 */
static int32_t ResmgrServer_initClientCommunicationChannels(void)
{
    struct sockaddr_un  sockAddress;
#ifdef RM_DSP_CLIENT_SPT //fzm
    void*               mmapBaseAddress;
    uint32_t            pageSize;
    int32_t             retVal;
    uint32_t            mask;
    uint32_t            size;
    uint32_t            mmapFD;
#endif

    /***********************************************************************************************
     * RMv2 Server Socket: This is used for ARM clients to communicate with the SYSLIB RM Server
     ***********************************************************************************************/

    /* Unlink any previous server instances: RM Server names need to be unique in the system anyway. */
    unlink (gResmgrServerMCB.serverName);

    /* Create the RM server socket. */
    gResmgrServerMCB.rmServerSocket = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (gResmgrServerMCB.rmServerSocket < 0)
    {
        ResmgrServer_log(ResmgrServer_LogLevel_ERROR, "Error: RM Server Socket failed %s\n", strerror(errno));
        return -1;
    }

    /* Initialize the socket address */
    memset(&sockAddress, 0, sizeof(sockAddress));

    /* Populate the binding information */
    sockAddress.sun_family = AF_UNIX;
    snprintf(sockAddress.sun_path, sizeof(sockAddress.sun_path), "%s/%s", Syslib_getRunTimeDirectory(), gResmgrServerMCB.serverName);

    /* Bind the server socket: */
    if (bind(gResmgrServerMCB.rmServerSocket, (struct sockaddr*)&sockAddress, sizeof(struct sockaddr_un)) < 0)
    {
        ResmgrServer_log(ResmgrServer_LogLevel_ERROR, "Error: Socket bind failed (error: %s)\n", strerror(errno));
        return -1;
    }
    ResmgrServer_log(ResmgrServer_LogLevel_INFO, "Debug: RMv2 ARM sockets are operational\n"); //fzm
#ifdef RM_DSP_CLIENT_SPT //fzm
    /***********************************************************************************************
     * RMv2 Server-Client Mailbox for ARM-DSP communication
     ***********************************************************************************************/

    /* Sanity Check: Allocate memory for all the mailboxes; align the size to the page boundary */
    size = (RM_MAX_MAILBOX * sizeof(ResmgrServer_MailBox));
    if ((size % 4096) != 0)
        size = ((size / 4096) + 1)*4096;

    /* Minimum page size for the mmapped region. */
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
    if ((size % pageSize) || ((uint32_t)gResmgrServerMCB.sharedMemoryAddress % pageSize))
    {
        ResmgrServer_log(ResmgrServer_LogLevel_ERROR, "Error: Shared memory alignment checks failed [size %d]\n", size);
        return -1;
    }

    /* Create a mapping of the physical address. */
    mmapBaseAddress = mmap(0, size, (PROT_READ|PROT_WRITE), MAP_SHARED, mmapFD, (off_t)gResmgrServerMCB.sharedMemoryAddress & ~mask);
    if(mmapBaseAddress == MAP_FAILED)
        return -1;

    /* Map the mailbox to the virtual address: */
    gResmgrServerMCB.ptrMailboxList = (ResmgrServer_MailBox*)(mmapBaseAddress + ((off_t)gResmgrServerMCB.sharedMemoryAddress & mask));

    /* Initialize the mailbox. */
    memset ((void*)gResmgrServerMCB.ptrMailboxList, 0, size);

    /* Debug Message: */
    ResmgrServer_log(ResmgrServer_LogLevel_DEBUG, "Debug: Shared Memory Physical Base Address %p Size %d\n",
                     gResmgrServerMCB.sharedMemoryAddress, size);

    /* Display all the mailbox which are being used for DSP-ARM communications. */
    {
        ResmgrServer_MailBox*   ptrMailbox;
        uint32_t                mailboxIndex;

        /* Get the mailbox starting address */
        ptrMailbox = gResmgrServerMCB.ptrMailboxList;

        /* Cycle through all the mailboxes */
        for (mailboxIndex = 0; mailboxIndex < RM_MAX_MAILBOX; mailboxIndex++)
            ResmgrServer_log(ResmgrServer_LogLevel_DEBUG, "Debug: Mailbox %d Address %p\n", mailboxIndex, ptrMailbox++);
    }

#else
    ResmgrServer_log(ResmgrServer_LogLevel_INFO, "Debug: RM Server running without Mailbox.\n"); //fzm
#endif
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is the registered callback function for ARM clients which is used
 *      to allocate memory for the resource manager transaction.
 *
 *  @param[in]  appTransport
 *      Registered application transport callback function
 *  @param[in]  pktSize
 *      Size of the memory to be allocated
 *  @param[out]  pktHandle
 *      Pointer to the actual packet which is to be sent out.
 *
 *  @retval
 *      Success   - 0
 *  @retval
 *      Error     - <0
 */
static Rm_Packet* ResMgr_armClientTransportAlloc
(
    Rm_AppTransportHandle   appTransport,
    uint32_t                pktSize,
    Rm_PacketHandle*        pktHandle
)
{
    Rm_Packet*  ptrRmPacket;

    /* Allocate memory for the packet. */
    ptrRmPacket = malloc (pktSize);
    if (ptrRmPacket == NULL)
        return NULL;
    ptrRmPacket->pktLenBytes = pktSize;

    /* Store and return the handle. */
    *pktHandle = ptrRmPacket;
    return ptrRmPacket;
}

/**
 *  @b Description
 *  @n
 *      The function is the registered callback function for ARM clients which is
 *      used to send a request from the RM client to the server.
 *
 *  @param[in]  appTransport
 *      Registered application transport callback function
 *  @param[in]  pktHandle
 *      Pointer to the start of the application's transport "packet"
 *
 *  @retval
 *      Success   - 0
 *  @retval
 *      Error     - <0
 */
static int32_t ResMgr_armClientTransportSend
(
    Rm_AppTransportHandle   appTransport,
    Rm_PacketHandle         pktHandle
)
{
    Rm_Packet*              ptrRmPacket;
    ResmgrServer_Client*    ptrClient;
    int32_t                 numBytes;
    struct sockaddr_un      to;

    /* Get the pointer to the RM packet which needs to be sent out. */
    ptrRmPacket = (Rm_Packet*)pktHandle;

    /* Get the pointer to the RM Client */
    ptrClient = (ResmgrServer_Client*)appTransport;

    /* Initialize the destination */
	memset(&to, 0, sizeof(struct sockaddr_un));

    /* Populate the destination configuration */
    to.sun_family = AF_UNIX;
    snprintf(to.sun_path, sizeof(to.sun_path), "%s", ptrClient->clientName);

    /* Send the packet out to the RM client. This is an ARM Client */
    numBytes = sendto(gResmgrServerMCB.rmServerSocket, ptrRmPacket, ptrRmPacket->pktLenBytes, 0,
                      (struct sockaddr *)&to, sizeof(to));
    if (numBytes < 0)
    {
        ResmgrServer_log(ResmgrServer_LogLevel_ERROR, "Error: Unable to send the packet to RM Client %s\n", strerror(errno));
        return -1;
    }

    /* Cleanup the allocated memory */
    free (ptrRmPacket);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is the registered callback function for the DSP Clients which is
 *      used to allocate memory for the resource manager transaction.
 *
 *  @param[in]  appTransport
 *      Registered application transport callback function
 *  @param[in]  pktSize
 *      Size of the memory to be allocated
 *  @param[out]  pktHandle
 *      Pointer to the actual packet which is to be sent out.
 *
 *  @retval
 *      Success   - 0
 *  @retval
 *      Error     - <0
 */
static Rm_Packet* ResMgr_dspClientTransportAlloc
(
    Rm_AppTransportHandle   appTransport,
    uint32_t                pktSize,
    Rm_PacketHandle*        pktHandle
)
{
    Rm_Packet*              ptrRmPacket;
    ResmgrServer_MailBox*   ptrMailbox;

    /* Get the mailbox which is currently being serviced. This approach allows us to
     * have only the DSP perform the mailbox allocations. */
    ptrMailbox = gResmgrServerMCB.ptrInServiceMailbox;

    /* Initialize the RM packet with the response. */
    ptrRmPacket = (Rm_Packet*)&ptrMailbox->response[0];
    ptrRmPacket->pktLenBytes = pktSize;

    /* Store and return the handle. */
    *pktHandle = ptrRmPacket;
    return ptrRmPacket;
}

/**
 *  @b Description
 *  @n
 *      The function is the registered callback function for DSP clients which is used to
 *      send a request from the RM client to the server.
 *
 *  @param[in]  appTransport
 *      Registered application transport callback function
 *  @param[in]  pktHandle
 *      Pointer to the start of the application's transport "packet"
 *
 *  @retval
 *      Success   - 0
 *  @retval
 *      Error     - <0
 */
static int32_t ResMgr_dspClientTransportSend
(
    Rm_AppTransportHandle   appTransport,
    Rm_PacketHandle         pktHandle
)
{
    ResmgrServer_MailBox*   ptrMailbox;

    /* Get the mailbox which is currently being serviced. This approach allows us to
     * have only the DSP perform the mailbox allocations. */
    ptrMailbox = gResmgrServerMCB.ptrInServiceMailbox;

    /* Mark the response as available */
    ptrMailbox->status = ResmgrServer_MailBoxStatus_RESPONSE;
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to add the RMv2 client into the internal database.
 *      RMv2 clients could be a DSP or ARM clients. The clients are registered
 *      with the RMv2 library with different transports depending upon this.
 *
 *  @param[in]  clientName
 *      RM client name which is to be added
 *  @param[in]  isARMClient
 *      Flag to indicate if the client is an ARM or DSP Client.
 *
 *  @retval
 *      Success -   Pointer to the added RM Client block
 *  @retval
 *      Error   -   NULL
 */
static ResmgrServer_Client* ResmgrServer_addClient
(
    const char* clientName,
    uint8_t     isARMClient
)
{
    ResmgrServer_Client*    ptrClient;
    Rm_TransportCfg         transportCfg;
    int32_t                 result;

    /* Allocate memory for the client. */
    ptrClient = (ResmgrServer_Client*) malloc(sizeof(ResmgrServer_Client));
    if (ptrClient == NULL)
        return NULL;

    /* Initialize the client memory */
    memset ((void *)ptrClient, 0, sizeof(ResmgrServer_Client));

    /* Populate the client block. */
    strcpy (ptrClient->clientName, clientName);
    ptrClient->isARMClient = isARMClient;

    /* Initialize the transport configuration */
    memset ((void *)&transportCfg, 0, sizeof(Rm_TransportCfg));

    /* Create and register the transports accordingly. */
    if (isARMClient == 1)
    {
        /* Populate the transport configuration for ARM Clients */
        transportCfg.rmHandle                     = gResmgrServerMCB.rmServerHandle;
        transportCfg.appTransportHandle           = (Rm_AppTransportHandle)ptrClient;
        transportCfg.remoteInstType               = Rm_instType_CLIENT;
        transportCfg.transportCallouts.rmAllocPkt = ResMgr_armClientTransportAlloc;
        transportCfg.transportCallouts.rmSendPkt  = ResMgr_armClientTransportSend;
    }
    else
    {
        /* Populate the transport configuration for DSP Clients. */
        transportCfg.rmHandle                     = gResmgrServerMCB.rmServerHandle;
        transportCfg.appTransportHandle           = (Rm_AppTransportHandle)ptrClient;
        transportCfg.remoteInstType               = Rm_instType_CLIENT;
        transportCfg.transportCallouts.rmAllocPkt = ResMgr_dspClientTransportAlloc;
        transportCfg.transportCallouts.rmSendPkt  = ResMgr_dspClientTransportSend;
    }

    /* Register the transport */
    ptrClient->transportHandle = Rm_transportRegister(&transportCfg, &result);
    if (result != RM_OK)
    {
        ResmgrServer_log(ResmgrServer_LogLevel_ERROR, "Error: Unable to register the transport [Error Code %d]\n", result);
        return NULL;
    }

    /* Debug Message: */
    ResmgrServer_log(ResmgrServer_LogLevel_INFO, "Debug: Registering %s Client '%s' Transport %p\n",
                     (isARMClient == 1) ? "ARM" : "DSP", clientName, ptrClient->transportHandle); //fzm

    /* Add the client to the list */
    ResmgrServer_listAdd((ResmgrServer_ListNode**)&gResmgrServerMCB.ptrClientList, (ResmgrServer_ListNode*)ptrClient);
    return ptrClient;
}

/**
 *  @b Description
 *  @n
 *      The function is used to process an RM request received from any of
 *      the RM clients.
 *
 *  @param[in]  isARMClient
 *      Flag which indicates if the client is an ARM or DSP client.
 *  @param[in]  ptrRMClientReqPacket
 *      Pointer to the RM Request packet received
 *  @param[in]  clientName
 *      Client Name
 *
 *  @retval
 *      Not applicable
 */
static void ResmgrServer_processClientRequest
(
    uint8_t     isARMClient,
    const char* ptrRMClientReqPacket,
    const char* clientName
)
{
    Rm_Packet*              ptrRmPacket;
    ResmgrServer_Client*    ptrClient;
    int32_t                 rmResult;

    /* Get the RM Client information */
    ptrClient = ResmgrServer_findClient (clientName);
    if (ptrClient == NULL)
    {
        /* Add the client in the server database. */
        ptrClient = ResmgrServer_addClient (clientName, isARMClient);
        if (ptrClient == NULL)
        {
            /* Error: Unable to register the client with the RM Server. Dropping request */
            ResmgrServer_log(ResmgrServer_LogLevel_ERROR, "Error: Unable to add the client %s dropping RM request\n", clientName);
            return;
        }
    }

    /* Get the RM request. */
    ptrRmPacket = (Rm_Packet*)ptrRMClientReqPacket;

    /* Debug Message: */
    ResmgrServer_log(ResmgrServer_LogLevel_DEBUG, "Debug: Received Request from %s Request %p Transport %p for %d bytes\n",
                     clientName, ptrRmPacket, ptrClient->transportHandle, ptrRmPacket->pktLenBytes);

    /* Provide packet to RM Server for processing */
    rmResult = Rm_receivePacket(ptrClient->transportHandle, (Rm_Packet*)ptrRmPacket);
    if (rmResult != RM_OK)
    {
        ResmgrServer_log(ResmgrServer_LogLevel_ERROR, "Error: RM Server failed to process received packet: %d\n", rmResult);
        return;
    }
    return;
}

#ifdef RM_DSP_CLIENT_SPT //fzm
/**
 *  @b Description
 *  @n
 *      The function is used to poll the RMv2 requests received from the DSP clients.
 *
 *  @retval
 *      Not applicable.
 */
static void ResmgrServer_pollDSPClients (void)
{
    ResmgrServer_MailBox*   ptrMailbox;
    uint32_t                mailboxIndex;
    char                    clientName[RM_NAME_MAX_CHARS];
    uint32_t                numMailboxProcessed = 0;

    /* Get the pointer to the mailbox */
    ptrMailbox = (ResmgrServer_MailBox*)gResmgrServerMCB.ptrMailboxList;

    /* Cycle through all the mailboxes */
    for (mailboxIndex = 0; mailboxIndex < RM_MAX_MAILBOX; mailboxIndex++)
    {
        /* Is there an outstanding RMv2 DSP request message? */
        if (ptrMailbox->status == ResmgrServer_MailBoxStatus_REQUEST)
        {
            /* Get the RM Client name from the request packet. */
            Rm_receiveGetPktServiceSrcName((const Rm_Packet *)&ptrMailbox->request, clientName, RM_NAME_MAX_CHARS);

            /* Count the number of mailboxes which have been processed. */
            numMailboxProcessed++;

            /* Keep track of the mailbox being serviced. This is stored in a global pointer. We use this
             * to ensure that all the mailbox allocations are done only in the DSP and none in the ARM so
             * there is only 1 master for all allocations. The ARM needs to allocate memory for the
             * response however the mailbox stores memory for both request and response. But with the RM
             * transport it is not possible to pass some meta-information. But since we are running a
             * single thread we should be able to store the information in a global variable and access it
             * in the transport allocation function. */
            gResmgrServerMCB.ptrInServiceMailbox = ptrMailbox;

            /* Process the client request. */
            ResmgrServer_processClientRequest(0, &ptrMailbox->request[0], clientName);

            /* Mailbox has been serviced. */
            gResmgrServerMCB.ptrInServiceMailbox = NULL;
        }

        /* Next mailbox */
        ptrMailbox++;
    }

    /* We should have processed at least 1 mailbox */
    if (numMailboxProcessed == 0)
        ResmgrServer_log(ResmgrServer_LogLevel_ERROR, "RM Server: No DSP client processed\n");

    return;
}
#endif

/**
 *  @b Description
 *  @n
 *      This is the resource manager server thread
 *
 *  @param[in]  arg
 *      Argument passed to the thread.
 *
 *  @retval
 *      Not Applicable.
 */
static void* ResmgrServer_thread(void *arg)
{
    fd_set              fds;
    int32_t             retVal;
    char*               ptrRMClientReqPacket;
    char                clientName[128];
    struct sockaddr_un  from;
    socklen_t           fromLen;
    int32_t             numBytes;
    int32_t             maxFd;
    uint32_t            serverState;
    struct sigaction    act;

    /* Initialize the maximum file descriptor:
     *  - We account for the ARM RM Clients, DSP RM Clients & signal pipes */
#ifdef RM_DSP_CLIENT_SPT //fzm
    maxFd = max (gResmgrServerMCB.signalPipe[0], gResmgrServerMCB.dspInterruptFd);
    maxFd = max (maxFd, gResmgrServerMCB.rmServerSocket);
#else
    maxFd = max (gResmgrServerMCB.signalPipe[0], gResmgrServerMCB.rmServerSocket);
#endif
	/* Use the sa_sigaction field because the handles has two additional parameters */
	act.sa_sigaction = &ResmgrServer_terminated;
    act.sa_flags     = 0;

    /* Setup the signals. */
	sigaction(SIGTERM, &act, NULL);
    sigaction(SIGKILL, &act, NULL);
    sigaction(SIGINT,  &act, NULL);

    /* Setup the user defined signal to display the resource usage. */
	act.sa_sigaction = &ResmgrServer_displayResourceUsage;
    act.sa_flags     = 0;
    sigaction(SIGUSR1, &act, NULL);

    /* Server is operational */
    serverState = 1;

    /* Execute the server */
    while (serverState == 1)
    {
        /* Set the read socket */
        FD_ZERO(&fds);
#ifdef RM_DSP_CLIENT_SPT //fzm
        FD_SET(gResmgrServerMCB.dspInterruptFd, &fds);
#endif
        FD_SET(gResmgrServerMCB.rmServerSocket, &fds);
        FD_SET(gResmgrServerMCB.signalPipe[0], &fds);

        /* Wait for the data to arrive */
        retVal = select (maxFd + 1, &fds, NULL, NULL, NULL);
        if (retVal < 0)
        {
            ResmgrServer_log(ResmgrServer_LogLevel_ERROR, "RM Server: select returned %s\n", strerror(errno));
            break;
        }

        /* Did we get a message from the signal pipe? */
        if (FD_ISSET(gResmgrServerMCB.signalPipe[0], &fds) != 0)
        {
            /* YES. Read all the messages from the signal pipes and process them */
            while (1)
            {
                /* Read the message. */
                if (read(gResmgrServerMCB.signalPipe[0], &retVal, sizeof(uint32_t)) < 0)
                {
                    /* No more data. */
                    if (errno == EAGAIN)
                        break;

                    /* This is a FATAL error */
                    ResmgrServer_log(ResmgrServer_LogLevel_ERROR, "Error: Signal pipe read failed [%s]\n", strerror(errno));
                    break;
                }

                /* Process the received message. */
                if (retVal == 0xdead)
                {
                    /* Server is dead and has been terminated. Shutdown the thread and clean up resources. */
                    serverState = 0;
                }
                else if (retVal == 0xbeef && ResmgrServer_dumpInit() == 0) //fzm
                {
                    /* Invoke the RM API to display the resource usage */
                    Rm_resourceStatus(gResmgrServerMCB.rmServerHandle, 1);
                    Rm_instanceStatus(gResmgrServerMCB.rmServerHandle);
                    ResmgrServer_dump(ResmgrServer_LogLevel_INFO, "*************************************</resmgr_server_dump>\n"); //fzm
                    ResmgrServer_dumpDeInit();
                }
                else
                {
                    ResmgrServer_log(ResmgrServer_LogLevel_ERROR, "Error: Invalid message received on signal pipe [%x]\n", retVal);
                }
            }
        }

        /* Did we get a message from any of the DSP clients? */
#ifdef RM_DSP_CLIENT_SPT //fzm
        if (FD_ISSET(gResmgrServerMCB.dspInterruptFd, &fds) != 0)
        {
            /* Read from the DSP interrupt file descriptor.
             *  - This is required else the select will not block. */
            if (read(gResmgrServerMCB.dspInterruptFd, &retVal, sizeof(uint32_t)) != sizeof(uint32_t))
            {
                ResmgrServer_log(ResmgrServer_LogLevel_ERROR, "Internal Error: UINTC UIO read failed [%s]\n", strerror(errno));
                break;
            }

            /* The UIO module disable the event which needs to be reenabled from the application else
             * there will be no more interrupts detected. */
            if (Uintc_enableEvent (gResmgrServerMCB.uintcHandle, gResmgrServerMCB.dspSrcId, &retVal) < 0)
            {
                ResmgrServer_log(ResmgrServer_LogLevel_ERROR, "Error: Enable events failed [%d]\n", retVal);
                break;
            }

            /* Increment the number of interrupts detected */
            gPankajDebugCounter++;

            /* Debug message: */
            ResmgrServer_log(ResmgrServer_LogLevel_DEBUG, "Debug: DSP Client Interrupt request %d %d detected\n",
                             gPankajDebugCounter, retVal);

            /* Poll DSP Clients. */
            ResmgrServer_pollDSPClients();
        }
#endif

        /* Did we get a message from any of the ARM clients? */
        if (FD_ISSET(gResmgrServerMCB.rmServerSocket, &fds) != 0)
        {
            /* ARM Client: */
            fromLen = sizeof(struct sockaddr_un);

            /* Allocate memory for the RM Client request */
            ptrRMClientReqPacket = malloc (RM_MAX_REQUEST_SIZE);
            if (ptrRMClientReqPacket == NULL)
            {
                ResmgrServer_log(ResmgrServer_LogLevel_ERROR, "Out of memory\n");
                break;
            }

            /* Receive the RM Client request from the server socket. */
            numBytes = recvfrom(gResmgrServerMCB.rmServerSocket, ptrRMClientReqPacket, RM_MAX_REQUEST_SIZE, 0,
                                (struct sockaddr *)&from, &fromLen);
            if (numBytes < 0)
            {
                /* Error: Unable to receive data from the server socket. */
                ResmgrServer_log(ResmgrServer_LogLevel_ERROR, "Error: Unable to receive RM request: %s\n", strerror(errno));
                break;
            }

            /* On ARM Clients; we use the 'from' field to determine the client name */
            strcpy (clientName, from.sun_path);

            /* Process the client request. */
            ResmgrServer_processClientRequest(1, ptrRMClientReqPacket, clientName);

            /* Cleanup the client request memory. */
            free (ptrRMClientReqPacket);
        }
    }

    /* Shutdown the server socket. */
    ResmgrServer_log(ResmgrServer_LogLevel_INFO, "RM Server: Shutting down.\n"); //fzm
    close (gResmgrServerMCB.rmServerSocket);
    unlink (gResmgrServerMCB.serverName);

#ifdef RM_DSP_CLIENT_SPT //fzm
    /* Shutdown and close the UINTC module */
    if (Uintc_deinit (gResmgrServerMCB.uintcHandle, &retVal) < 0)
        ResmgrServer_log(ResmgrServer_LogLevel_ERROR, "Error: Shutting down the UINTC module failed [Error code %d]\n", retVal);
    else
        ResmgrServer_log(ResmgrServer_LogLevel_DEBUG, "Debug: Shutting down the UINTC module successful\n");
#endif
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      The function is used to setup the signal pipes which are used
 *      to communicate from the signal handlers.
 *
 *  @retval
 *      Success 	- 0
 *  @retval
 *      Error		- <0
 */
static int32_t ResmgrServer_setupSignalPipe(void)
{
    int32_t     flags;

    /* Open the signal pipes */
    if (pipe (gResmgrServerMCB.signalPipe) < 0)
    {
        ResmgrServer_log(ResmgrServer_LogLevel_ERROR, "Error: Unable to open signal pipe[%s]\n", strerror(errno));
        return -1;
    }

    /* Setup the read pipe to be non blocking */
    flags = fcntl(gResmgrServerMCB.signalPipe[0], F_GETFL);
    if (flags == -1)
    {
        ResmgrServer_log(ResmgrServer_LogLevel_ERROR, "Error: Unable to get read signal pipe flags [%s]\n", strerror(errno));
        return -1;
    }
    flags |= O_NONBLOCK;
    if (fcntl(gResmgrServerMCB.signalPipe[0], F_SETFL, flags) == -1)
    {
        ResmgrServer_log(ResmgrServer_LogLevel_ERROR, "Error: Unable to set read signal pipe flags [%s]\n", strerror(errno));
        return -1;
    }

    /* Setup the write pipe to be non blocking */
    flags = fcntl(gResmgrServerMCB.signalPipe[1], F_GETFL);
    if (flags == -1)
    {
        ResmgrServer_log(ResmgrServer_LogLevel_ERROR, "Error: Unable to get write signal pipe flags [%s]\n", strerror(errno));
        return -1;
    }
    flags |= O_NONBLOCK;
    if (fcntl(gResmgrServerMCB.signalPipe[1], F_SETFL, flags) == -1)
    {
        ResmgrServer_log(ResmgrServer_LogLevel_ERROR, "Error: Unable to set write signal pipe flags [%s]\n", strerror(errno));
        return -1;
    }

    /* Pipes have been created & configured. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Entry point for the Resource Manager Server Application.
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
    int                 retVal;
    pthread_t           serverThread;
#ifdef RM_DSP_CLIENT_SPT //fzm
    UintcConfig         uintcConfig;
#endif

    /* Initialize the resource manager server block. */
    memset((void *)&gResmgrServerMCB, 0, sizeof(ResmgrServer_MCB));

    /* Setup the defaults.
     *  - Address 0xA0010000 is used for the shared memory transport */
    gResmgrServerMCB.logLevel = ResmgrServer_LogLevel_INFO; //fzm
    gResmgrServerMCB.sharedMemoryAddress = 0xA0010000;
    gResmgrServerMCB.dspSrcId            = 0xFF;

    /* Initialize the logger */
    if (ResmgrServer_logInit() < 0) //fzm
        return -1;

    /* Process the command line arguments: */
    if (ResmgrServer_processCmdLineArgs(argc, argv) < 0)
    {
        ResmgrServer_displayUsage();
        ResmgrServer_logDeInit(); //fzm
        return -1;
    }

    ResmgrServer_parseConfigFile(); //fzm

    /* Initialize and start the resource manager server */
    if (ResmgrServer_initRM() < 0)
    {
        ResmgrServer_logDeInit(); //fzm
        return -1;
    }

    /* Initialize the system */
    if (ResmgrServer_systemInit (gResmgrServerMCB.rmServerServiceHandle) < 0)
    {
        ResmgrServer_logDeInit(); //fzm
        return -1;
    }

    /* Initialize and start the communication channels required by the RM clients. */
    if (ResmgrServer_initClientCommunicationChannels() < 0)
    {
        ResmgrServer_logDeInit(); //fzm
        return -1;
    }

#ifdef RM_DSP_CLIENT_SPT //fzm
    /* Initialize and populate the user space interrupt configuration */
    memset ((void *)&uintcConfig, 0, sizeof(UintcConfig));

    /* Populate the user space interrupt configuration: */
    snprintf(uintcConfig.name, sizeof(uintcConfig.name), "UINTC-ResmgrSrv-%d", getpid());
    uintcConfig.mode = Uintc_Mode_APPLICATION_MANAGED;

    /* Initialize the user space interrupt module */
    gResmgrServerMCB.uintcHandle = Uintc_init (&uintcConfig, &retVal);
    if (gResmgrServerMCB.uintcHandle == NULL)
    {
        ResmgrServer_log(ResmgrServer_LogLevel_ERROR, "Error: Unable to open the UINTC module [Error code %d]\n", retVal);
        ResmgrServer_logDeInit(); //fzm
        return -1;
    }

    /* Register interrupts using the UINTC module.
     * - The ISR handler is passed as NULL i.e. dont care since we will handle the interrupts without
     *   uintc_select. */
    gResmgrServerMCB.dspInterruptFd = Uintc_registerIsr (gResmgrServerMCB.uintcHandle, gResmgrServerMCB.dspSrcId, NULL,
                                                         (void*)gResmgrServerMCB.dspSrcId, &retVal);
    if (gResmgrServerMCB.dspInterruptFd < 0)
    {
        ResmgrServer_log(ResmgrServer_LogLevel_ERROR, "Error: Unable to register the IPC interrupt [Error code %d]\n", retVal);
        ResmgrServer_logDeInit(); //fzm
        return -1;
    }
#endif

    /* Create the signal pipe. */
    if (ResmgrServer_setupSignalPipe() < 0)
    {
        ResmgrServer_logDeInit(); //fzm
        return -1;
    }

#ifdef RM_DSP_CLIENT_SPT //fzm
    /* Display the PID information */
    ResmgrServer_log(ResmgrServer_LogLevel_DEBUG, "Debug: SYSRM Server PID %d [DSP Src Id %d]\n", getpid(), gResmgrServerMCB.dspSrcId);
#else
    ResmgrServer_log(ResmgrServer_LogLevel_INFO, "Debug: SYSRM Server running without DSP client support.\n"); //fzm
#endif

    /* Launch the server thread */
	retVal = pthread_create (&serverThread, NULL, ResmgrServer_thread, NULL);
	if (retVal < 0)
	{
    	ResmgrServer_log(ResmgrServer_LogLevel_ERROR, "Error: Server thread create failed error code %d\n", retVal);
        ResmgrServer_logDeInit(); //fzm
        return -1;
	}

    /* <fzm>                                              */
    /* Systemd info - ready to start netfpStarter service */
    sd_notify(0, "READY=1");
    /* </fzm>                                             */

	/* Blocked till the server thread is terminated. */
    pthread_join (serverThread, NULL);
    ResmgrServer_logDeInit(); //fzm
	return 0;
}

/**
@}
*/

