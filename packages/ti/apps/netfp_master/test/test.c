/**
 *   @file  test.c
 *
 *   @brief
 *      Testing utility which test the NETFP master management interface
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
#include <unistd.h>
#include <limits.h>

/* MCSDK Include files. */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/runtime/hplib/hplib.h>

/* SYSLIB Include Files */
#include <ti/runtime/netfp/netfp.h>
#include <ti/apps/netfp_master/netfp_master.h>
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/uintc/uintc.h>
#include <ti/runtime/name/name.h>
#include <ti/runtime/name/name_db.h>

#define NUM_ETHERNET_RULES    20

/**********************************************************************
 *********************** Global Declarations **************************
 **********************************************************************/

/* Application requested resources */
Resmgr_ResourceCfg      appResourceConfig =
{
    /* Requested Memory Region Configuration. */
   .memRegionCfg =
	{
        /* Name,           Type,                       Linking RAM,                           Num,     Size */
		{ "ARM-Capture-0", Resmgr_MemRegionType_DDR3,  Resmgr_MemRegionLinkingRAM_DONT_CARE,  2048,    128 },
    },
};

/* Number of Ethernet Rule entries added in the test */
uint32_t gEthRuleEntries = 0;

/* Array with added rules from user config file */
Netfp_EthRuleHandle    ethRuleHandleArray[NUM_ETHERNET_RULES];

/* Array of Ethernet Rule Handles added in PA LUT 1-0 */
NetfpMaster_addEthRuleRequest ethRules[NUM_ETHERNET_RULES];

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

/**********************************************************************
 ************************** Test Functions ****************************
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
 *      Utility function which trims the spaces from the buffer
 *      is indicated by the file pointer.
 *
 *  @param[in]  ptrBuffer
 *      Pointer to the data buffer.
 *
 *  @retval
 *      Not applicable
 */
static void trimSpaces (char* ptrBuffer)
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
static char* getToken (char* ptrInitialBuffer)
{
    const char*  delimitters = "=\n\r";
    char*  token;

    while (1)
    {
        /* Parse the token */
        token = strtok(ptrInitialBuffer, delimitters);
        if (token == NULL)
            return NULL;

        /* Trim out spaces and tabs from the token */
        trimSpaces(token);

        /* Did we trim everything out? */
        if (*token != 0)
            break;
    }
    return token;
}

/**
 *  @b Description
 *  @n
 *      The function is used to parse the Ethernet Rule test file.
 *  The test file should be named as netfp_ethrule.dat.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */

static int32_t Test_parseEthRuleFile(void)
{
    char                    testFileName[PATH_MAX];
    struct stat             buf;
    FILE*                   testFile;
    char*                   fileBuffer;
    char*                   ptrFileBuffer;
    int32_t                 size;
    NetfpMaster_addEthRuleRequest*    request;
    char*                   tokenName;
    char*                   tokenValue;
    /* set the test file name */
    strcpy(testFileName, "netfp_ethrule.dat");

    /* Reset number of entries */
    gEthRuleEntries = 0;

    /* Get the file statistics */
    if (stat(testFileName, &buf) != 0)
    {
        printf("Error: Unable to get test file statistics from %s\n", testFileName);
        return -1;
    }

    /* Allocate memory for the file buffer: */
    fileBuffer = (char*)malloc (buf.st_size + 1);
    if (fileBuffer == NULL)
    {
        printf("Error: Unable to allocate memory for the file buffer\n");
        return -1;
    }

    /* Initialize the file buffer. */
    memset ((void *)fileBuffer, 0, buf.st_size + 1);

    /* Open the configuration file: */
    testFile = fopen (testFileName, "r");
    if (testFile == NULL)
    {
        printf("Error: Unable to open the test file '%s'\n", testFileName);
        return -1;
    }

    /* Initialize the variables: */
    ptrFileBuffer = fileBuffer;
    size          = 0;


    /* Read data from the file and place it into the buffer */
    while (1)
    {
        int ch = fgetc(testFile);
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
    fclose (testFile);

    /* Initialize the variables */
    size = 0;

    /* Run through the entire file. */
    while (size < buf.st_size)
    {
        /* Get the token */
        tokenName = getToken(ptrFileBuffer);
        if (tokenName == NULL)
            break;

        /* Subsequent calls to the strtok API requires NULL parameters. */
        ptrFileBuffer = NULL;
        size = size + strlen(tokenName);

        /* Process and parse the buffer */
        if (*tokenName == '#' || *tokenName == '\n' || *tokenName == '\r')
            continue;

        /* Process the tokens */
        if (strcmp (tokenName, "rule") == 0)
        {
            if (gEthRuleEntries >= NUM_ETHERNET_RULES)
                break;

            /* Get the handle to the current request */
            request = &ethRules[gEthRuleEntries];
            if (request == NULL)
                break;

            memset ((void *)request , 0, sizeof (NetfpMaster_addEthRuleRequest));
        }
        else if (strcmp (tokenName, "{") == 0)
        {
            continue;
        }
        else if (strcmp (tokenName, "}") == 0)
        {
            gEthRuleEntries++;

            request = (NetfpMaster_addEthRuleRequest *)NULL;

        }
        else if (strcmp (tokenName, "rmRegionName") == 0)
        {
            if(request == NULL)
            {
                printf("Error: Parsing Error: Keyword %s was specified outside the 'rule' block\n", tokenName);
                return -1;
            }
            tokenValue = getToken(ptrFileBuffer);
            strncpy (request->rmRegionName, tokenValue, NETFP_MAX_CHAR - 1);
            size = size + strlen(tokenValue);
        }
        else if (strcmp (tokenName, "ethType") == 0)
        {
            if(request == NULL)
            {
                printf("Error: Parsing Error: Keyword %s was specified outside the 'rule' block\n", tokenName);
                return -1;
            }
            tokenValue = getToken(ptrFileBuffer);
            size = size + strlen(tokenValue);

            request->ethType = atoi(tokenValue);
        }
        else if (strcmp (tokenName, "vlanId") == 0)
        {
            if(request == NULL)
            {
                printf("Error: Parsing Error: Keyword %s was specified outside the 'rule' block\n", tokenName);
                return -1;
            }
            tokenValue = getToken(ptrFileBuffer);
            size = size + strlen(tokenValue);

            request->vlanId = atoi(tokenValue);
        }
        else if (strcmp (tokenName, "anyVlanId") == 0)
        {
            if(request == NULL)
            {
                printf("Error: Parsing Error: Keyword %s was specified outside the 'rule' block\n", tokenName);
                return -1;
            }
            tokenValue = getToken(ptrFileBuffer);
            size = size + strlen(tokenValue);

            request->anyVlanId = atoi(tokenValue);
        }
        else if (strcmp (tokenName, "dstType") == 0)
        {
            if(request == NULL)
            {
                printf("Error: Parsing Error: Keyword %s was specified outside the 'rule' block\n", tokenName);
                return -1;
            }
            tokenValue = getToken(ptrFileBuffer);
            size = size + strlen(tokenValue);

            if(strcmp (tokenValue, "EMAC") == 0)
                request->dstType = Netfp_EthRuleDst_EMAC;
            else if (strcmp (tokenValue, "PDSP1") == 0)
                request->dstType = Netfp_EthRuleDst_CONTINUE;
            else if (strcmp (tokenValue, "HOST") == 0)
                request->dstType = Netfp_EthRuleDst_HOST;
            else
            {
                printf("Error: Parsing Error: dstType %s was not supported \n", tokenValue);
                return -1;
            }
        }
        else if (strcmp (tokenName, "ingressIfName") == 0)
        {

            if(request == NULL)
            {
                printf("Error: Parsing Error: Keyword %s was specified outside the 'rule' block\n", tokenName);
                return -1;
            }
            tokenValue = getToken(ptrFileBuffer);
            size = size + strlen(tokenValue);

            strncpy (request->ingressIfName, tokenValue, NETFP_MAX_CHAR - 1);

        }
        else if (strcmp (tokenName, "egressIfName") == 0)
        {
            if(request == NULL)
            {
                printf("Error: Parsing Error: Keyword %s was specified outside the 'rule' block\n", tokenName);
                return -1;
            }
            tokenValue = getToken(ptrFileBuffer);
            size = size + strlen(tokenValue);

            strncpy (request->egressIfName, tokenValue, NETFP_MAX_CHAR - 1);

        }
        else if (strcmp (tokenName, "srcMac") == 0)
        {
            if(request == NULL)
            {
                printf("Error: Parsing Error: Keyword %s was specified outside the 'rule' block\n", tokenName);
                return -1;
            }
            tokenValue = getToken(ptrFileBuffer);
            size = size + strlen(tokenValue);

            {
                uint32_t   mac[6];
                uint32_t   i;

                sscanf(tokenValue, "%x.%x.%x.%x.%x.%x",
                       &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]);

                for (i=0; i<6; i++)
                    request->srcMacAddress[i] = (uint8_t)(mac[i] & 0xff);
            }
        }
        else if (strcmp (tokenName, "dstMac") == 0)
        {
            if(request == NULL)
            {
                printf("Error: Parsing Error: Keyword %s was specified outside the 'rule' block\n", tokenName);
                return -1;
            }
            tokenValue = getToken(ptrFileBuffer);
            size = size + strlen(tokenValue);

            {
                uint32_t     idx;
                uint32_t     mac[6];

                /* Get MAC address from string */
                sscanf(tokenValue, "%x.%x.%x.%x.%x.%x",
                       &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]);

                for (idx=0; idx<6; idx++)
                    request->dstMacAddress[idx] = (uint8_t)mac[idx];

            }
        }
        else if (strcmp (tokenName, "byteCounter") == 0)
        {
            if(request == NULL)
            {
                printf("Error: Parsing Error: Keyword %s was specified outside the 'rule' block\n", tokenName);
                return -1;
            }
            tokenValue = getToken(ptrFileBuffer);
            size = size + strlen(tokenValue);

            /* Enable byte counter */
            if (atoi(tokenValue) == 1)
            {
                /* Add byte based user statistics */
                request->userStatsCfg[request->numUserStats].userStatsLen  = Netfp_UserStatsLen_64b;
                request->userStatsCfg[request->numUserStats].userStatsType = pa_USR_STATS_TYPE_BYTE;

                request->numUserStats++;
            }
        }
        else if (strcmp (tokenName, "packetCounter") == 0)
        {
            if(request == NULL)
            {
                printf("Error: Parsing Error: Keyword %s was specified outside the 'rule' block\n", tokenName);
                return -1;
            }
            tokenValue = getToken(ptrFileBuffer);
            size = size + strlen(tokenValue);

            /* Enable byte counter */
            if (atoi(tokenValue) == 1)
            {
                /* Add byte based user statistics */
                request->userStatsCfg[request->numUserStats].userStatsLen  = Netfp_UserStatsLen_32b;
                request->userStatsCfg[request->numUserStats].userStatsType = pa_USR_STATS_TYPE_PACKET;

                request->numUserStats++;
            }
        }
        else
        {
            printf("Error: Parsing error; invalid token '%s' detected\n", tokenName);
            return -1;
        }
    }

    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to validate the request and response and
 *      ensure that it matches the NETFP Master documented messaging
 *      behavior.
 *
 *  @param[in]  ptrRequest
 *      Pointer to the NETFP Server
 *  @param[in]  heapHandle
 *      The heap handle which will be used to send commands to the PA subsystem
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t validateResponse
(
    NetfpMaster_Request*    ptrRequest,
    NetfpMaster_Response*   ptrResponse
)
{
    /* Ensure that we received a response: */
    if (ptrResponse->msgType != NetfpMaster_MessageType_RESPONSE)
    {
        printf ("Error: Received a non-response packet [%d]\n", ptrResponse->msgType);
        return -1;
    }

    /* Validate the transaction identifier */
    if (ptrRequest->id != ptrResponse->id)
    {
        printf ("Error: Request [%d] & Response [%d] transaction identifiers are not same\n", ptrRequest->id, ptrResponse->id);
        return -1;
    }

    /* Ensure that the response was for the request */
    if (ptrRequest->msgType != ptrResponse->reqType)
    {
        printf ("Error: Invalid response type [%d] detected [Expected %d]\n", ptrResponse->reqType, ptrRequest->msgType);
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Entry point to the test code
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t main(void)
{
    Resmgr_SystemCfg            sysConfig;
    int32_t                     sockfd;
    int32_t                     selection;
    struct sockaddr_un          sockAddress;
    NetfpMaster_Request         request;
    NetfpMaster_Response        response;
    int32_t                     numBytes;
    struct sockaddr_un          from;
    struct sockaddr_un          to;
    socklen_t                   fromLen;
    int32_t                     index;
    char                        socketName[PATH_MAX];
    Pktlib_InstCfg              pktlibInstCfg;
    Name_DatabaseCfg            databaseCfg;
    Resmgr_SysCfgHandle         handleSysCfg;
    Name_DBHandle               nameDatabaseHandle;
    Pktlib_InstHandle           pktlibInstanceHandle;
    Pktlib_HeapCfg              heapCfg;
    int32_t                     errCode;
    Pktlib_HeapHandle           captureHeapHandle;

    /* Initialize the database configuration */
    memset ((void *)&databaseCfg, 0, sizeof(Name_DatabaseCfg));

    /* Populate the configuration: */
    databaseCfg.instanceId  = 1;
    databaseCfg.realm       = Name_ExecutionRealm_ARM;
    strcpy (databaseCfg.owner, "UnitTest");

    /* Create the global database */
    nameDatabaseHandle = Name_createDatabase (&databaseCfg, &errCode);
    if (nameDatabaseHandle == NULL)
	{
	    printf ("Error: Name database creation failed [Error code %d]\n", errCode);
	    return -1;
    }
    printf ("Debug: Name database created successfully [Handle %p]\n", nameDatabaseHandle);

    if (Name_purgeDatabase (nameDatabaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB, databaseCfg.owner, &errCode) < 0)
	{
	    printf ("Error: Name database purge failed [Error code %d]\n", errCode);
	    return -1;
    }
    printf ("Debug: Name database purged successfully [Handle %p]\n", nameDatabaseHandle);

    /* Initialize the system configuration. */
    memset ((void *)&sysConfig, 0, sizeof(Resmgr_SystemCfg));

    /* Populate the configuration.*/
    sysConfig.realm                 = Resmgr_ExecutionRealm_ARM;
    strcpy (sysConfig.rmClient, "Rm_LTE9A");
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

    /* Process the configuration: */
    if (Resmgr_processConfig (handleSysCfg, &appResourceConfig, &errCode) < 0)
	{
	    printf ("Error: SYSRM configuration failed with error code %d\n", errCode);
	    return -1;
    }

    /* Initialize the PKTLIB instance configuration. */
    memset ((void *)&pktlibInstCfg, 0, sizeof(Pktlib_InstCfg));

    /* Populate the PKTLIB Instance configuration: */
    pktlibInstCfg.instanceId        = 1;
    pktlibInstCfg.databaseHandle    = nameDatabaseHandle;
    pktlibInstCfg.sysCfgHandle      = handleSysCfg;
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
    pktlibInstanceHandle = Pktlib_createInstance (&pktlibInstCfg, &errCode);
    if (pktlibInstanceHandle == NULL)
    {
        printf ("Error: Creating the PKTLIB instance failed [Error code %d]\n", errCode);
        return -1;
    }

    /* Initialize the heap configuration. */
    memset ((void *)&heapCfg, 0, sizeof(Pktlib_HeapCfg));

    /* Populate the heap configuration
     *  - The heap is used for the reception and transmission of all packets */
    strcpy(heapCfg.name, "Test_Heap");
    heapCfg.memRegion                       = appResourceConfig.memRegionResponse[0].memRegionHandle;
    heapCfg.pktlibInstHandle                = pktlibInstanceHandle;
    heapCfg.sharedHeap                      = 0;
    heapCfg.useStarvationQueue              = 0;
    heapCfg.dataBufferSize                  = 1536;
    heapCfg.numPkts                         = 64;
    heapCfg.numZeroBufferPackets            = 0;
    heapCfg.dataBufferPktThreshold          = 8;
    heapCfg.zeroBufferPktThreshold          = 0;
    heapCfg.heapInterfaceTable.dataMalloc   = myMalloc;
    heapCfg.heapInterfaceTable.dataFree     = myFree;
    captureHeapHandle = Pktlib_createHeap(&heapCfg, &errCode);
    if (captureHeapHandle == NULL)
    {
	    printf ("Error: Unable to create packet capture heap [Error code %d]\n", errCode);
        return -1;
    }
    printf ("Debug: Packet Capture Heap created sucessfully.\n");

    /* Create the socket name: */
    snprintf (socketName, PATH_MAX, "%s/Test-%d", Syslib_getRunTimeDirectory(), getpid());

    /* Unlink and remove any previous socket names */
    unlink (socketName);

    /* Create the test socket. */
    sockfd = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (sockfd < 0)
    {
        printf ("Error: NETFP Master Test socket failed %s\n", strerror(errno));
        return -1;
    }

    /* Initialize the socket address */
    memset(&sockAddress, 0, sizeof(sockAddress));

    /* Populate the binding information */
    sockAddress.sun_family = AF_UNIX;
    strncpy(sockAddress.sun_path, socketName, sizeof(sockAddress.sun_path));

    /* Bind the socket: */
    if (bind(sockfd, (struct sockaddr*)&sockAddress, sizeof(struct sockaddr_un)) < 0)
    {
        printf("Error: Socket bind failed (error: %s)\n", strerror(errno));
        return -1;
    }
    printf ("Debug: NETFP master test socket is operational\n");

    /* We always send back the responses to the NETFP Master */
    memset ((void *)&to, 0, sizeof(struct sockaddr_un));
    to.sun_family = AF_UNIX;
    snprintf(to.sun_path, PATH_MAX, "%s/%s", Syslib_getRunTimeDirectory(), NETFP_MASTER_SOCKET_NAME);

    while (1)
    {
        printf ("*******************************************\n");
        printf ("1. Get Interface Map\n");
        printf ("2. Set Interface Map\n");
        printf ("3. Get Reassembly statistics\n");
        printf ("4. Register for notifications\n");
        printf ("5. Deregister from notifications\n");
        printf ("6. Get interface list\n");
        printf ("7. Wait for updates\n");
        printf ("8. Port mirror request\n");
        printf ("9. Port capture request\n");
        printf ("10. Error\n");
        printf ("11. Add Ethernet Rule\n");
        printf ("12. Delete Ethernet Rule\n");
        printf ("13. Display Ethernet Rule\n");
        printf ("14. Change the routing mode\n");
        printf ("15. Change the DSCP Map\n");
        printf ("16. Change the PBit Map\n");
        printf ("17. Change the Default Host Priority\n");
        printf ("18. Change the Default Forwarding Priority\n");
        printf ("19. Rename the interface\n");
        printf ("20. Enable/Disable Priority Override\n");
        printf ("21. Get the NETCP Statistics\n");
        printf ("22. Preclassification configuration\n");
        printf ("23. Ethernet rule statistics\n");
        printf ("*******************************************\n");
        printf ("Enter your selection:");
        scanf ("%d", &selection);

        /* Initialize the request and response */
        memset ((void *)&request,  0, sizeof(NetfpMaster_Request));
        memset ((void *)&response, 0, sizeof(NetfpMaster_Response));

        /* Set the from Length */
        fromLen = sizeof (struct sockaddr_un);

        switch (selection)
        {
            case 1:
            {
                /***************************************************************
                 * Get Interface MAP Request
                 ***************************************************************/
                request.msgType = NetfpMaster_MessageType_GET_INTERFACE_REQUEST;
                request.id      = random();
                strcpy (request.u.getIfRequest.ifName, "eth0");

                /* Send the request to the NETFP master */
                numBytes = sendto(sockfd, &request, sizeof(NetfpMaster_Request), 0, (struct sockaddr *)&to, sizeof(to));
                if (numBytes < 0)
                {
                    printf ("Error: Unable to send the request to the NETFP Master [Error %s]\n", strerror(errno));
                    break;
                }

                /* Wait for the response to arrive: */
                numBytes = recvfrom(sockfd, &response, sizeof(NetfpMaster_Response), 0, (struct sockaddr *)&from, &fromLen);
                if (numBytes < 0)
                {
                    printf ("Error: Unable to receive the response from the NETFP Master [Error %s]\n", strerror(errno));
                    break;
                }

                /* Validate the response as per the NETFP master messaging specification: */
                if (validateResponse (&request, &response) < 0)
                    break;

                /* Was the response successful? */
                if (response.errCode == 0)
                {
                    /* YES. Excellent dump the response on the console */
                    printf ("Switch Port --> %d\n", response.u.getIfResponse.switchPort);
                    for (index = 0; index < 64; index++)
                        printf ("Inner DSCP %d --> Outer DSCP %d\n", index, response.u.getIfResponse.innerToOuterDSCPMap[index]);
                }
                else
                {
                    /* NO. This could be because an invalid interface name was specified; simply report this */
                    printf ("Debug: GET interface request for interface %s failed [Error code %d]\n",
                            request.u.getIfRequest.ifName, response.errCode);
                }
                break;
            }
            case 2:
            {
                /***************************************************************
                 * Set Interface map request
                 ***************************************************************/
                request.msgType = NetfpMaster_MessageType_SET_INNER_OUTER_DSCP_MAP;
                request.id      = random();
                strcpy (request.u.setInnerOuterDSCPMap.ifName, "eth0");

                printf ("Inner DSCP:");
                scanf ("%d", (int32_t*)&request.u.setInnerOuterDSCPMap.innerDSCP);

                printf ("New Outer DSCP Marking:");
                scanf ("%d", (int32_t*)&request.u.setInnerOuterDSCPMap.outerDSCP);

                /* Send the request to the NETFP master */
                numBytes = sendto(sockfd, &request, sizeof(NetfpMaster_Request), 0, (struct sockaddr *)&to, sizeof(to));
                if (numBytes < 0)
                {
                    printf ("Error: Unable to send the request to the NETFP Master [Error %s]\n", strerror(errno));
                    break;
                }

                /* Wait for the response to arrive: */
                numBytes = recvfrom(sockfd, &response, sizeof(NetfpMaster_Response), 0, (struct sockaddr *)&from, &fromLen);
                if (numBytes < 0)
                {
                    printf ("Error: Unable to receive the response from the NETFP Master [Error %s]\n", strerror(errno));
                    break;
                }

                /* Validate the response as per the NETFP master messaging specification: */
                if (validateResponse (&request, &response) < 0)
                    break;

                /* Was the response successful? */
                if (response.errCode != 0)
                {
                    /* NO. This could be because an invalid interface name was specified; simply report this */
                    printf ("Debug: SET interface map request for interface %s failed [Error code %d]\n",
                             request.u.getIfRequest.ifName, response.errCode);
                }
                else
                {
                    printf ("Debug: Validate the markings using the GET Interface map request\n");
                }
                break;
            }
            case 3:
            {
                /***************************************************************
                 * Get Reassembly Statistics request
                 ***************************************************************/
                request.msgType = NetfpMaster_MessageType_GET_REASSEMBLY_STATS_REQUEST;
                request.id      = random();

                /* Send the request to the NETFP master */
                numBytes = sendto(sockfd, &request, sizeof(NetfpMaster_Request), 0, (struct sockaddr *)&to, sizeof(to));
                if (numBytes < 0)
                {
                    printf ("Error: Unable to send the request to the NETFP Master [Error %s]\n", strerror(errno));
                    break;
                }

                /* Wait for the response to arrive: */
                numBytes = recvfrom(sockfd, &response, sizeof(NetfpMaster_Response), 0, (struct sockaddr *)&from, &fromLen);
                if (numBytes < 0)
                {
                    printf ("Error: Unable to receive the response from the NETFP Master [Error %s]\n", strerror(errno));
                    break;
                }

                /* Validate the response as per the NETFP master messaging specification: */
                if (validateResponse (&request, &response) < 0)
                    break;

                /* Was the response successful? */
                if (response.errCode != 0)
                {
                    /* NO. This could be because an invalid interface name was specified; simply report this */
                    printf ("Debug: Unable to get the reassembly statistics [Error code %d]\n", response.errCode);
                    break;
                }

                printf ("***************************************************************\n");
                printf ("Debug: Reassembly Stats\n");
                printf ("Debug: Active Reassembly Contexts              : %d\n", response.u.reassemblyStats.activeReassemblyContexts);
                printf ("Debug: Outer IP fragments                      : %d\n", response.u.reassemblyStats.numOuterIPPktReceived);
                printf ("Debug: Inner IP fragments                      : %d\n", response.u.reassemblyStats.numInnerIPPktReceived);
                printf ("Debug: Active fragments                        : %d\n", response.u.reassemblyStats.numActiveFragments);
                printf ("Debug: Non accelerated packets                 : %d\n", response.u.reassemblyStats.nonAccleratedTrafficFlow);
                printf ("Debug: IPv4 fragments                          : %d\n", response.u.reassemblyStats.numIPv4Fragments);
                printf ("Debug: IPv6 fragments                          : %d\n", response.u.reassemblyStats.numIPv6Fragments);
                printf ("Debug: Reassembled packets                     : %d\n", response.u.reassemblyStats.numReassembledPackets);
                printf ("Debug: Reassembly timeouts                     : %d\n", response.u.reassemblyStats.numReassemblyTimeout);
                printf ("Debug: Duplicated fragments                    : %d\n", response.u.reassemblyStats.numDuplicatedFragment);
                printf ("Debug: Large packets                           : %d\n", response.u.reassemblyStats.numLargePackets);
                printf ("Debug: Number of TF not deleted                : %d\n", response.u.reassemblyStats.numFreeTrafficFlowFailure);
                printf ("Debug: Traffic management Invocations          : %d\n", response.u.reassemblyStats.numDefaultReassemblyMgmtInvocations);
                printf ("Debug: Dropped fragments by traffic management : %d\n", response.u.reassemblyStats.numDefaultReassemblyMgmtDroppedFragments);
                printf ("***************************************************************\n");
                break;
            }
            case 4:
            {
                /***************************************************************
                 * Register Notification request
                 ***************************************************************/
                request.msgType = NetfpMaster_MessageType_REGISTER_NOTIFICATION;
                request.id      = random();

                /* Send the request to the NETFP master */
                numBytes = sendto(sockfd, &request, sizeof(NetfpMaster_Request), 0, (struct sockaddr *)&to, sizeof(to));
                if (numBytes < 0)
                {
                    printf ("Error: Unable to send the request to the NETFP Master [Error %s]\n", strerror(errno));
                    break;
                }

                /* Wait for the response to arrive: */
                numBytes = recvfrom(sockfd, &response, sizeof(NetfpMaster_Response), 0, (struct sockaddr *)&from, &fromLen);
                if (numBytes < 0)
                {
                    printf ("Error: Unable to receive the response from the NETFP Master [Error %s]\n", strerror(errno));
                    break;
                }

                /* Validate the response as per the NETFP master messaging specification: */
                if (validateResponse (&request, &response) < 0)
                    break;

                /* Was registeration successful? */
                if (response.errCode == 0)
                    printf ("Debug: Registered for notifications successfully\n");
                else
                    printf ("Error: Unable to register for notifications [Error code %d]\n", response.errCode);
                break;
            }
            case 5:
            {
                /***************************************************************
                 * Deregister for notification request
                 ***************************************************************/
                request.msgType = NetfpMaster_MessageType_DEREGISTER_NOTIFICATION;
                request.id      = random();

                /* Send the request to the NETFP master */
                numBytes = sendto(sockfd, &request, sizeof(NetfpMaster_Request), 0, (struct sockaddr *)&to, sizeof(to));
                if (numBytes < 0)
                {
                    printf ("Error: Unable to send the request to the NETFP Master [Error %s]\n", strerror(errno));
                    break;
                }

                /* Wait for the response to arrive: */
                numBytes = recvfrom(sockfd, &response, sizeof(NetfpMaster_Response), 0, (struct sockaddr *)&from, &fromLen);
                if (numBytes < 0)
                {
                    printf ("Error: Unable to receive the response from the NETFP Master [Error %s]\n", strerror(errno));
                    break;
                }

                /* Validate the response as per the NETFP master messaging specification: */
                if (validateResponse (&request, &response) < 0)
                    break;

                /* Was registeration successful? */
                if (response.errCode == 0)
                    printf ("Debug: Deregistered from notifications successfully\n");
                else
                    printf ("Error: Unable to deregister from notifications [Error code %d]\n", response.errCode);
                break;
            }
            case 6:
            {
                /***************************************************************
                 * Interface List
                 ***************************************************************/
                request.msgType = NetfpMaster_MessageType_GET_INTERFACE_LIST;
                request.id      = random();

                /* Send the request to the NETFP master */
                numBytes = sendto(sockfd, &request, sizeof(NetfpMaster_Request), 0, (struct sockaddr *)&to, sizeof(to));
                if (numBytes < 0)
                {
                    printf ("Error: Unable to send the request to the NETFP Master [Error %s]\n", strerror(errno));
                    break;
                }

                /* Wait for the response to arrive: */
                numBytes = recvfrom(sockfd, &response, sizeof(NetfpMaster_Response), 0, (struct sockaddr *)&from, &fromLen);
                if (numBytes < 0)
                {
                    printf ("Error: Unable to receive the response from the NETFP Master [Error %s]\n", strerror(errno));
                    break;
                }

                /* Validate the response as per the NETFP master messaging specification: */
                if (validateResponse (&request, &response) < 0)
                    break;

                /* Was registeration successful? */
                if (response.errCode != 0)
                {
                    printf ("Error: Unable to deregister from notifications [Error code %d]\n", response.errCode);
                    break;
                }

                /* Initialize the index */
                index = 0;

                /* Display the received interface list: */
                while (1)
                {
                    /* NULL terminated list: */
                    if (response.u.ifName[index][0] == 0)
                        break;

                    /* Display the interface name */
                    printf ("Interface %d -> %s\n", index, response.u.ifName[index]);
                    index++;
                }
                break;
            }
            case 7:
            {
                /***************************************************************
                 * Waiting for updates
                 ***************************************************************/
                numBytes = recvfrom(sockfd, &request, sizeof(NetfpMaster_Request), 0, (struct sockaddr *)&from, &fromLen);
                if (numBytes < 0)
                {
                    printf ("Error: Unable to receive the update notice from the NETFP Master [Error %s]\n", strerror(errno));
                    break;
                }

                /* Sanity Check: Validate and ensure that we received the UPDATE notice request */
                if (request.msgType != NetfpMaster_MessageType_UPDATE_NOTICE)
                {
                    printf ("Error: Received an invalid message type [Got %d]\n", request.msgType);
                    break;
                }

                /* Sanity Check: Only the NETFP master is capable of sending the UPDATE notice request */
                if (strcmp (from.sun_path, NETFP_MASTER_SOCKET_NAME) != 0)
                {
                    printf ("Error: Received update notice request from %s \n", from.sun_path);
                    break;
                }

                /* Extract the interface name for which the request was received */
                printf ("Debug: Received update notice request for interface %s\n", request.u.updateIfNotice.ifName);
                break;
            }
            case 8:
            {
                /***************************************************************
                 * Port mirror request
                 ***************************************************************/
                request.msgType = NetfpMaster_MessageType_SET_PORT_MIRROR_REQUEST;
                request.id      = random();

                /* Get the user inputs: */
                printf ("Enter the Port mirror status [1 to enable, 0 to disable]:");
                scanf ("%d", &request.u.setPortMirrorRequest.isEnable);
                printf ("Enter the Port mirror direction [1 for Ingress, 0 for egress]:");
                scanf ("%d", (int32_t*)&request.u.setPortMirrorRequest.direction);
                printf ("Enter the source port:");
                scanf ("%d", (int32_t*)&request.u.setPortMirrorRequest.srcPort);
                printf ("Enter the destination port:");
                scanf ("%d", (int32_t*)&request.u.setPortMirrorRequest.dstPort);

                /* Send the request to the NETFP master */
                numBytes = sendto(sockfd, &request, sizeof(NetfpMaster_Request), 0, (struct sockaddr *)&to, sizeof(to));
                if (numBytes < 0)
                {
                    printf ("Error: Unable to send the request to the NETFP Master [Error %s]\n", strerror(errno));
                    break;
                }

                /* Wait for the response to arrive: */
                numBytes = recvfrom(sockfd, &response, sizeof(NetfpMaster_Response), 0, (struct sockaddr *)&from, &fromLen);
                if (numBytes < 0)
                {
                    printf ("Error: Unable to receive the response from the NETFP Master [Error %s]\n", strerror(errno));
                    break;
                }

                /* Validate the response as per the NETFP master messaging specification: */
                if (validateResponse (&request, &response) < 0)
                    break;

                /* Was port mirror successful? */
                if (response.errCode != 0)
                {
                    printf ("Error: Unable to configure the port mirror [Error code %d]\n", response.errCode);
                    break;
                }
                break;
            }
            case 9:
            {
                /***************************************************************
                 * Port capture request
                 ***************************************************************/
                request.msgType = NetfpMaster_MessageType_SET_PORT_CAPTURE_REQUEST;
                request.id      = random();

                /* Get the user inputs: */
                printf ("Enter the Port capture status [1 to enable, 0 to disable]:");
                scanf ("%d", &request.u.setPortCaptureRequest.isEnable);
                printf ("Enter the Port mirror direction [1 for Ingress, 0 for egress]:");
                scanf ("%d", (int32_t*)&request.u.setPortCaptureRequest.direction);
                printf ("Enter the port to be captured:");
                scanf ("%d", (int32_t*)&request.u.setPortCaptureRequest.portToBeCaptured);
                printf ("Enter the destination queue:");
                scanf ("%d", (int32_t*)&request.u.setPortCaptureRequest.dstQueue);
                printf ("Enter the swInfo:");
                scanf ("%d", (int32_t*)&request.u.setPortCaptureRequest.swInfo);

                /* Populate the free identifier; we use the capture heap for this purpose */
                request.u.setPortCaptureRequest.freeQueueId = Pktlib_getInternalHeapQueue (captureHeapHandle);

                /* Send the request to the NETFP master */
                numBytes = sendto(sockfd, &request, sizeof(NetfpMaster_Request), 0, (struct sockaddr *)&to, sizeof(to));
                if (numBytes < 0)
                {
                    printf ("Error: Unable to send the request to the NETFP Master [Error %s]\n", strerror(errno));
                    break;
                }

                /* Wait for the response to arrive: */
                numBytes = recvfrom(sockfd, &response, sizeof(NetfpMaster_Response), 0, (struct sockaddr *)&from, &fromLen);
                if (numBytes < 0)
                {
                    printf ("Error: Unable to receive the response from the NETFP Master [Error %s]\n", strerror(errno));
                    break;
                }

                /* Validate the response as per the NETFP master messaging specification: */
                if (validateResponse (&request, &response) < 0)
                    break;

                /* Was port capture successful? */
                if (response.errCode != 0)
                {
                    printf ("Error: Unable to configure the port capture [Error code %d]\n", response.errCode);
                    break;
                }
                break;
            }
            case 10:
            {
                /***************************************************************
                 * Invalid request
                 ***************************************************************/
                request.msgType = (NetfpMaster_MessageType)0xdead;
                request.id      = random();

                /* Send the request to the NETFP master */
                numBytes = sendto(sockfd, &request, sizeof(NetfpMaster_Request), 0, (struct sockaddr *)&to, sizeof(to));
                if (numBytes < 0)
                {
                    printf ("Error: Unable to send the request to the NETFP Master [Error %s]\n", strerror(errno));
                    break;
                }

                /* Wait for the response to arrive: */
                numBytes = recvfrom(sockfd, &response, sizeof(NetfpMaster_Response), 0, (struct sockaddr *)&from, &fromLen);
                if (numBytes < 0)
                {
                    printf ("Error: Unable to receive the response from the NETFP Master [Error %s]\n", strerror(errno));
                    break;
                }

                /* Validate the response as per the NETFP master messaging specification: */
                if (validateResponse (&request, &response) < 0)
                    break;

                /* Was the response successful? Response should always fail. */
                if (response.errCode != NETFP_MASTER_EINVAL)
                {
                    printf ("Error: Invalid error code detected %d\n", response.errCode);
                    break;
                }
                break;
            }
            case 11:
            {
                /* Parse the ethernet rule test file */
                if (Test_parseEthRuleFile() < 0 )
                    break;

                printf("Trying to adding %d rules\n", gEthRuleEntries);

                /***************************************************************
                 * Add Ethernet Rule Request
                 ***************************************************************/
                 for(index=0; index < gEthRuleEntries; index++ )
                 {

                     request.msgType = NetfpMaster_MessageType_ADD_ETHERNET_RULE;
                     request.id      = random();

                     printf("***************************************************\n");
                     printf("Adding ethernet rule %d\n", index);

                     memcpy(&request.u.addEthRuleRequest, &ethRules[index], sizeof(NetfpMaster_addEthRuleRequest));

                     /* Send the request to the NETFP master */
                     numBytes = sendto(sockfd, &request, sizeof(NetfpMaster_Request), 0, (struct sockaddr *)&to, sizeof(to));
                     if (numBytes < 0)
                     {
                         printf ("Error: Unable to send the request to the NETFP Master [Error %s]\n", strerror(errno));
                        break;
                     }

                     /* Wait for the response to arrive: */
                     numBytes = recvfrom(sockfd, &response, sizeof(NetfpMaster_Response), 0, (struct sockaddr *)&from, &fromLen);
                     if (numBytes < 0)
                     {
                         printf ("Error: Unable to receive the response from the NETFP Master [Error %s]\n", strerror(errno));
                         break;
                     }

                     /* Validate the response as per the NETFP master messaging specification: */
                     if (validateResponse (&request, &response) < 0)
                         break;

                     /* Save the ethernet rule handle to be used for deletion */
                     ethRuleHandleArray[index] = response.u.ethRuleHandle;

                      /* Was adding Ethernet Rule successful? */
                     if (response.u.ethRuleHandle == NULL )
                     {
                         printf ("Error: Unable to add Ethernet Rule %d [Error code %d]\n", index, response.errCode);
                     }
                }
                break;
            }
            case 12:
            {
                /***************************************************************
                 * Delete Ethernet Rule Request
                 ***************************************************************/
                 for(index=0; index < gEthRuleEntries; index++ )
                 {
                     request.msgType = NetfpMaster_MessageType_DEL_ETHERNET_RULE;
                     request.id      = random();

                     printf("Deleting ethernet rule %d: %p\n", index, ethRuleHandleArray[index]);
                     request.u.delEthRuleRequest.ethRuleHandle   = ethRuleHandleArray[index];

                     /* Send the request to the NETFP master */
                     numBytes = sendto(sockfd, &request, sizeof(NetfpMaster_Request), 0, (struct sockaddr *)&to, sizeof(to));
                     if (numBytes < 0)
                     {
                         printf ("Error: Unable to send the request to the NETFP Master [Error %s]\n", strerror(errno));
                         break;
                     }

                     /* Wait for the response to arrive: */
                     numBytes = recvfrom(sockfd, &response, sizeof(NetfpMaster_Response), 0, (struct sockaddr *)&from, &fromLen);
                     if (numBytes < 0)
                     {
                         printf ("Error: Unable to receive the response from the NETFP Master [Error %s]\n", strerror(errno));
                         break;
                     }

                     /* Validate the response as per the NETFP master messaging specification: */
                     if (validateResponse (&request, &response) < 0)
                         break;

                     /* Was deleting Ethernet Rule successful? */
                     if (response.errCode != 0)
                     {
                         printf ("Error: Unable to delete Ethernet Rule %d : %p [Error code %d]\n", index,
                                 ethRuleHandleArray[index], response.errCode);
                     }
                }
                break;
            }
            case 13:
            {
                /***************************************************************
                 * Display Ethernet Rule Request
                 ***************************************************************/
                request.msgType = NetfpMaster_MessageType_DISPLAY_ETHERNET_RULE;
                request.id      = random();

                printf("Display ethernet rules\n");

                /* Send the request to the NETFP master */
                numBytes = sendto(sockfd, &request, sizeof(NetfpMaster_Request), 0, (struct sockaddr *)&to, sizeof(to));
                if (numBytes < 0)
                {
                     printf ("Error: Unable to send the request to the NETFP Master [Error %s]\n", strerror(errno));
                     break;
                }

                /* Wait for the response to arrive: */
                numBytes = recvfrom(sockfd, &response, sizeof(NetfpMaster_Response), 0, (struct sockaddr *)&from, &fromLen);
                if (numBytes < 0)
                {
                    printf ("Error: Unable to receive the response from the NETFP Master [Error %s]\n", strerror(errno));
                    break;
                }

                /* Validate the response as per the NETFP master messaging specification: */
                if (validateResponse (&request, &response) < 0)
                    break;

                /* Was adding Ethernet Rule successful? */
                if ( response.errCode != 0 )
                {
                     printf ("Error: Unable to add Ethernet Rule %d [Error code %d]\n", index, response.errCode);
                }
                break;
            }
            case 14:
            {
                /***************************************************************
                 * Set Port Routing mode
                 ***************************************************************/
                request.msgType = NetfpMaster_MessageType_SET_ROUTING_MODE;
                request.id      = random();

                /* Get the user inputs: */
                printf ("Enter the interface name:");
                scanf ("%s", &request.u.setRoutingMode.ifName[0]);
                printf ("Enter the new routing mode [dscp, dp-bit]:");
                scanf ("%s", &request.u.setRoutingMode.routingMode[0]);

                /* Send the request to the NETFP master */
                numBytes = sendto(sockfd, &request, sizeof(NetfpMaster_Request), 0, (struct sockaddr *)&to, sizeof(to));
                if (numBytes < 0)
                {
                    printf ("Error: Unable to send the request to the NETFP Master [Error %s]\n", strerror(errno));
                    break;
                }

                /* Wait for the response to arrive: */
                numBytes = recvfrom(sockfd, &response, sizeof(NetfpMaster_Response), 0, (struct sockaddr *)&from, &fromLen);
                if (numBytes < 0)
                {
                    printf ("Error: Unable to receive the response from the NETFP Master [Error %s]\n", strerror(errno));
                    break;
                }

                /* Validate the response as per the NETFP master messaging specification: */
                if (validateResponse (&request, &response) < 0)
                    break;

                /* Was the reconfiguration for the routing mode successful? */
                if (response.errCode != 0)
                {
                    printf ("Error: Unable to modify the QOS routing mode [Error code %d]\n", response.errCode);
                    break;
                }
                break;
            }
            case 15:
            {
                /***************************************************************
                 * Set DSCP Map
                 ***************************************************************/
                request.msgType = NetfpMaster_MessageType_SET_DSCP_MAPPING;
                request.id      = random();

                /* Get the user inputs: */
                printf ("Enter the interface name:");
                scanf ("%s", &request.u.setDSCPMap.ifName[0]);
                printf ("Enter the DSCP:");
                scanf ("%hhu", &request.u.setDSCPMap.dscp);
                printf ("Enter the new Queue Offset:");
                scanf ("%hhu", &request.u.setDSCPMap.queueOffset);

                /* Send the request to the NETFP master */
                numBytes = sendto(sockfd, &request, sizeof(NetfpMaster_Request), 0, (struct sockaddr *)&to, sizeof(to));
                if (numBytes < 0)
                {
                    printf ("Error: Unable to send the request to the NETFP Master [Error %s]\n", strerror(errno));
                    break;
                }

                /* Wait for the response to arrive: */
                numBytes = recvfrom(sockfd, &response, sizeof(NetfpMaster_Response), 0, (struct sockaddr *)&from, &fromLen);
                if (numBytes < 0)
                {
                    printf ("Error: Unable to receive the response from the NETFP Master [Error %s]\n", strerror(errno));
                    break;
                }

                /* Validate the response as per the NETFP master messaging specification: */
                if (validateResponse (&request, &response) < 0)
                    break;

                /* Was the reconfiguration for the routing mode successful? */
                if (response.errCode != 0)
                {
                    printf ("Error: Unable to modify the QOS routing mode [Error code %d]\n", response.errCode);
                    break;
                }
                break;
            }
            case 16:
            {
                /***************************************************************
                 * Set VLAN Map
                 ***************************************************************/
                request.msgType = NetfpMaster_MessageType_SET_VLAN_MAPPING;
                request.id      = random();

                /* Get the user inputs: */
                printf ("Enter the interface name:");
                scanf ("%s", &request.u.setVLANMap.ifName[0]);
                printf ("Enter the VLAN Priority:");
                scanf ("%hhu", &request.u.setVLANMap.pbit);
                printf ("Enter the new Queue Offset:");
                scanf ("%hhu", &request.u.setVLANMap.queueOffset);

                /* Send the request to the NETFP master */
                numBytes = sendto(sockfd, &request, sizeof(NetfpMaster_Request), 0, (struct sockaddr *)&to, sizeof(to));
                if (numBytes < 0)
                {
                    printf ("Error: Unable to send the request to the NETFP Master [Error %s]\n", strerror(errno));
                    break;
                }

                /* Wait for the response to arrive: */
                numBytes = recvfrom(sockfd, &response, sizeof(NetfpMaster_Response), 0, (struct sockaddr *)&from, &fromLen);
                if (numBytes < 0)
                {
                    printf ("Error: Unable to receive the response from the NETFP Master [Error %s]\n", strerror(errno));
                    break;
                }

                /* Validate the response as per the NETFP master messaging specification: */
                if (validateResponse (&request, &response) < 0)
                    break;

                /* Was the reconfiguration for the routing mode successful? */
                if (response.errCode != 0)
                {
                    printf ("Error: Unable to modify the QOS routing mode [Error code %d]\n", response.errCode);
                    break;
                }
                break;
            }
            case 17:
            {
                /***************************************************************
                 * Set Default Host Priority
                 ***************************************************************/
                request.msgType = NetfpMaster_MessageType_SET_DEFAULT_HOST_PRIORITY;
                request.id      = random();

                /* Get the user inputs: */
                printf ("Enter the default host priority:");
                scanf ("%hhu", &request.u.setDefaultHostPriority.defaultHostPriority);

                /* Send the request to the NETFP master */
                numBytes = sendto(sockfd, &request, sizeof(NetfpMaster_Request), 0, (struct sockaddr *)&to, sizeof(to));
                if (numBytes < 0)
                {
                    printf ("Error: Unable to send the request to the NETFP Master [Error %s]\n", strerror(errno));
                    break;
                }

                /* Wait for the response to arrive: */
                numBytes = recvfrom(sockfd, &response, sizeof(NetfpMaster_Response), 0, (struct sockaddr *)&from, &fromLen);
                if (numBytes < 0)
                {
                    printf ("Error: Unable to receive the response from the NETFP Master [Error %s]\n", strerror(errno));
                    break;
                }

                /* Validate the response as per the NETFP master messaging specification: */
                if (validateResponse (&request, &response) < 0)
                    break;

                /* Was the reconfiguration for the routing mode successful? */
                if (response.errCode != 0)
                {
                    printf ("Error: Unable to modify the QOS routing mode [Error code %d]\n", response.errCode);
                    break;
                }
                break;
            }
            case 18:
            {
                /***************************************************************
                 * Set Default Forwarding Priority
                 ***************************************************************/
                request.msgType = NetfpMaster_MessageType_SET_DEFAULT_FWD_PRIORITY;
                request.id      = random();

                /* Get the user inputs: */
                printf ("Enter the interface name:");
                scanf ("%s", &request.u.setDefaultFwdPriority.ifName[0]);
                printf ("Enter the default forwarding priority:");
                scanf ("%hhu", &request.u.setDefaultFwdPriority.defaultForwardPriority);

                /* Send the request to the NETFP master */
                numBytes = sendto(sockfd, &request, sizeof(NetfpMaster_Request), 0, (struct sockaddr *)&to, sizeof(to));
                if (numBytes < 0)
                {
                    printf ("Error: Unable to send the request to the NETFP Master [Error %s]\n", strerror(errno));
                    break;
                }

                /* Wait for the response to arrive: */
                numBytes = recvfrom(sockfd, &response, sizeof(NetfpMaster_Response), 0, (struct sockaddr *)&from, &fromLen);
                if (numBytes < 0)
                {
                    printf ("Error: Unable to receive the response from the NETFP Master [Error %s]\n", strerror(errno));
                    break;
                }

                /* Validate the response as per the NETFP master messaging specification: */
                if (validateResponse (&request, &response) < 0)
                    break;

                /* Was the reconfiguration for the routing mode successful? */
                if (response.errCode != 0)
                {
                    printf ("Error: Unable to modify the QOS routing mode [Error code %d]\n", response.errCode);
                    break;
                }
                break;
            }
            case 19:
            {
                /***************************************************************
                 * Rename the interface
                 ***************************************************************/
                request.msgType = NetfpMaster_MessageType_RENAME_INTERFACE;
                request.id      = random();

                /* Get the user inputs: */
                printf ("Enter the interface name:");
                scanf ("%s", &request.u.renameInterface.oldIfName[0]);
                printf ("Enter the new interface name:");
                scanf ("%s", &request.u.renameInterface.newIfName[0]);

                /* Send the request to the NETFP master */
                numBytes = sendto(sockfd, &request, sizeof(NetfpMaster_Request), 0, (struct sockaddr *)&to, sizeof(to));
                if (numBytes < 0)
                {
                    printf ("Error: Unable to send the request to the NETFP Master [Error %s]\n", strerror(errno));
                    break;
                }

                /* Wait for the response to arrive: */
                numBytes = recvfrom(sockfd, &response, sizeof(NetfpMaster_Response), 0, (struct sockaddr *)&from, &fromLen);
                if (numBytes < 0)
                {
                    printf ("Error: Unable to receive the response from the NETFP Master [Error %s]\n", strerror(errno));
                    break;
                }

                /* Validate the response as per the NETFP master messaging specification: */
                if (validateResponse (&request, &response) < 0)
                    break;

                /* Was the reconfiguration for the routing mode successful? */
                if (response.errCode != 0)
                {
                    printf ("Error: Unable to modify the QOS routing mode [Error code %d]\n", response.errCode);
                    break;
                }
                break;
            }
            case 20:
            {
                /***************************************************************
                 * Enable/Disable Priority Override
                 ***************************************************************/
                request.msgType = NetfpMaster_MessageType_SET_PRIORITY_OVERRIDE;
                request.id      = random();

                /* Get the user inputs: */
                printf ("Enter the interface name:");
                scanf ("%s", &request.u.setPriorityOverride.ifName[0]);
                printf ("Enter the priority override:");
                scanf ("%d", &request.u.setPriorityOverride.priorityOverride);

                /* Send the request to the NETFP master */
                numBytes = sendto(sockfd, &request, sizeof(NetfpMaster_Request), 0, (struct sockaddr *)&to, sizeof(to));
                if (numBytes < 0)
                {
                    printf ("Error: Unable to send the request to the NETFP Master [Error %s]\n", strerror(errno));
                    break;
                }

                /* Wait for the response to arrive: */
                numBytes = recvfrom(sockfd, &response, sizeof(NetfpMaster_Response), 0, (struct sockaddr *)&from, &fromLen);
                if (numBytes < 0)
                {
                    printf ("Error: Unable to receive the response from the NETFP Master [Error %s]\n", strerror(errno));
                    break;
                }

                /* Validate the response as per the NETFP master messaging specification: */
                if (validateResponse (&request, &response) < 0)
                    break;

                /* Was the reconfiguration for the routing mode successful? */
                if (response.errCode != 0)
                {
                    printf ("Error: Unable to modify the QOS Priority Override [Error code %d]\n", response.errCode);
                    break;
                }
                break;
            }
            case 21:
            {
                /***************************************************************
                 * Get the NETCP statistics
                 ***************************************************************/
                request.msgType = NetfpMaster_MessageType_GET_NETCP_STATS;
                request.id      = random();

                /* Send the request to the NETFP master */
                numBytes = sendto(sockfd, &request, sizeof(NetfpMaster_Request), 0, (struct sockaddr *)&to, sizeof(to));
                if (numBytes < 0)
                {
                    printf ("Error: Unable to send the request to the NETFP Master [Error %s]\n", strerror(errno));
                    break;
                }

                /* Wait for the response to arrive: */
                numBytes = recvfrom(sockfd, &response, sizeof(NetfpMaster_Response), 0, (struct sockaddr *)&from, &fromLen);
                if (numBytes < 0)
                {
                    printf ("Error: Unable to receive the response from the NETFP Master [Error %s]\n", strerror(errno));
                    break;
                }

                /* Validate the response as per the NETFP master messaging specification: */
                if (validateResponse (&request, &response) < 0)
                    break;

                /* Was the reconfiguration for the routing mode successful? */
                if (response.errCode != 0)
                {
                    printf ("Error: Unable to modify the QOS routing mode [Error code %d]\n", response.errCode);
                    break;
                }

                /* Display the NETCP Statistics: */
                printf("**********************************************************\n");
                printf("C1 number of packets:             %d\n", response.u.netcpStats.classify1.nPackets);
                printf("C1 number IPv4 packets:           %d\n", response.u.netcpStats.classify1.nIpv4Packets);
                printf("C1 number Inner IPv4 packets:     %d\n", response.u.netcpStats.classify1.nIpv4PacketsInner);
                printf("C1 number IPv6 packets:           %d\n", response.u.netcpStats.classify1.nIpv6Packets);
                printf("C1 number Inner IPv6 packets:     %d\n", response.u.netcpStats.classify1.nIpv6PacketsInner);
                printf("C1 number custom packets:         %d\n", response.u.netcpStats.classify1.nCustomPackets);
                printf("C1 number SRIO packets :          %d\n", response.u.netcpStats.classify1.nSrioPackets);
                printf("C1 number llc/snap fail:          %d\n", response.u.netcpStats.classify1.nLlcSnapFail);
                printf("C1 number table matched:          %d\n", response.u.netcpStats.classify1.nTableMatch);
                printf("C1 number failed table matched:   %d\n", response.u.netcpStats.classify1.nNoTableMatch);
                printf("C1 number IP frags:               %d\n", response.u.netcpStats.classify1.nIpFrag);
                printf("C1 number IP depth overflow:      %d\n", response.u.netcpStats.classify1.nIpDepthOverflow);
                printf("C1 number vlan depth overflow:    %d\n", response.u.netcpStats.classify1.nVlanDepthOverflow);
                printf("C1 number gre depth overflow:     %d\n", response.u.netcpStats.classify1.nGreDepthOverflow);
                printf("C1 number mpls packets:           %d\n", response.u.netcpStats.classify1.nMplsPackets);
                printf("C1 number of parse fail:          %d\n", response.u.netcpStats.classify1.nParseFail);
                printf("C1 number invalid IPv6 opts:      %d\n", response.u.netcpStats.classify1.nInvalidIPv6Opt);
                printf("C1 number of silent discard:      %d\n", response.u.netcpStats.classify1.nSilentDiscard);
                printf("C1 number of invalid control:     %d\n", response.u.netcpStats.classify1.nInvalidControl);
                printf("C1 number of invalid states:      %d\n", response.u.netcpStats.classify1.nInvalidState);
                printf("C1 number of system fails:        %d\n", response.u.netcpStats.classify1.nSystemFail);
                printf("C2 number of packets:             %d\n", response.u.netcpStats.classify2.nPackets);
                printf("C2 number of UDP packets:         %d\n", response.u.netcpStats.classify2.nUdp);
                printf("C2 number of TCP packets:         %d\n", response.u.netcpStats.classify2.nTcp);
                printf("C2 number of custom packets:      %d\n", response.u.netcpStats.classify2.nCustom);
                printf("C2 number of silent discard:      %d\n", response.u.netcpStats.classify2.nSilentDiscard);
                printf("C2 number of invalid control:     %d\n", response.u.netcpStats.classify2.nInvalidControl);
                printf("Modify number of command file:    %d\n", response.u.netcpStats.modify.nCommandFail);
                break;
            }
            case 22:
            {
                /***************************************************************
                 * Preclassification configuration:
                 ***************************************************************/
                request.msgType = NetfpMaster_MessageType_SET_PRECLASSIFICATION;
                request.id      = random();

                /* Get the user inputs: */
                printf ("Enter the interface name:");
                scanf ("%s", &request.u.setPreclassification.ifName[0]);
                printf ("Is this for Broadcast[1]/Multicast[0]:");
                scanf ("%d", &request.u.setPreclassification.isBroadcast);
                printf ("Enable/Disable Preclassification:");
                scanf ("%d", &request.u.setPreclassification.enablePreclassfication);

                /* If the preclassification configuration is enabled then get the flow & queue identifier */
                if (request.u.setPreclassification.enablePreclassfication == 1)
                {
                    printf ("Preclassification Flow Id:");
                    scanf ("%d", &request.u.setPreclassification.preclassificationFlowId);
                    printf ("Preclassification Queue Id:");
                    scanf ("%d", &request.u.setPreclassification.preclassificationQueueId);
                }

                /* Send the request to the NETFP master */
                numBytes = sendto(sockfd, &request, sizeof(NetfpMaster_Request), 0, (struct sockaddr *)&to, sizeof(to));
                if (numBytes < 0)
                {
                    printf ("Error: Unable to send the request to the NETFP Master [Error %s]\n", strerror(errno));
                    break;
                }

                /* Wait for the response to arrive: */
                numBytes = recvfrom(sockfd, &response, sizeof(NetfpMaster_Response), 0, (struct sockaddr *)&from, &fromLen);
                if (numBytes < 0)
                {
                    printf ("Error: Unable to receive the response from the NETFP Master [Error %s]\n", strerror(errno));
                    break;
                }

                /* Validate the response as per the NETFP master messaging specification: */
                if (validateResponse (&request, &response) < 0)
                    break;

                /* Was the reconfiguration for the routing mode successful? */
                if (response.errCode != 0)
                {
                    printf ("Error: Unable to modify the QOS Priority Override [Error code %d]\n", response.errCode);
                    break;
                }
                break;
            }
            case 23:
            {
                /***************************************************************
                 * Request Ethernet Rule Stats
                 ***************************************************************/
                 for (index = 0; index < gEthRuleEntries; index++)
                 {
                     request.msgType = NetfpMaster_MessageType_GET_ETHERNET_RULE_STATS;
                     request.id      = random();

                     printf("Request ethernet rule stats %d: %p\n", index, ethRuleHandleArray[index]);
                     request.u.getEthRuleStats.ethRuleHandle  = ethRuleHandleArray[index];

                     /* Send the request to the NETFP master */
                     numBytes = sendto(sockfd, &request, sizeof(NetfpMaster_Request), 0, (struct sockaddr *)&to, sizeof(to));
                     if (numBytes < 0)
                     {
                         printf ("Error: Unable to send the request to the NETFP Master [Error %s]\n", strerror(errno));
                         break;
                     }

                     /* Wait for the response to arrive: */
                     numBytes = recvfrom(sockfd, &response, sizeof(NetfpMaster_Response), 0, (struct sockaddr *)&from, &fromLen);
                     if (numBytes < 0)
                     {
                         printf ("Error: Unable to receive the response from the NETFP Master [Error %s]\n", strerror(errno));
                         break;
                     }

                     /* Validate the response as per the NETFP master messaging specification: */
                     if (validateResponse (&request, &response) < 0)
                         break;

                     /* Was getting the Ethernet stats successful? */
                     if (response.errCode != 0)
                     {
                        /* Error: Get Ethernet User statistics failed */
                        printf ("Error: Unable to get the Ethernet Rule statistics %d : %p [Error code %d]\n", index,
                                 ethRuleHandleArray[index], response.errCode);
                     }
                     else
                     {
                         /* Display the statistics: */
                         printf("Debug: User Stats[1]=%lld User Stats[2]=%lld\n", response.u.stats.userStats[0], response.u.stats.userStats[1]);
                     }
                }
                break;
            }
            default:
            {
                break;
            }
        }
    }

    /* Delete the packet capture heap */
    if (Pktlib_deleteHeap(pktlibInstanceHandle, captureHeapHandle, &errCode) < 0)
        printf ("Error: PKTLIB Delete Packet capture heap failed [Error code %d]\n", errCode);
    else
        printf ("Debug: PKTLIB Delete Packet capture heap deleted successfully\n");

    return 0;
}

