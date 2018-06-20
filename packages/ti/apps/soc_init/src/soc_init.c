/**
 *   @file  soc_init.c
 *
 *   @brief
 *      This is the SOC initialization service which initializes
 *      and sets up the system.
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

/**********************************************************************
 *************************** Include Files ****************************
 **********************************************************************/

/* Standard Include Files. */
#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <limits.h>
#include <signal.h>
#include <getopt.h>
#include <getopt.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sched.h>
#include <sys/syscall.h>

/* MCSDK Include Files */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/runtime/hplib/hplib.h>
#include <ti/drv/qmss/qmss_firmware.h>

/* SYSLIB Include Files */
#include <ti/apps/soc_init/include/soc_init.h>
#include <ti/runtime/resmgr/resmgr.h>

/* Device specific dependencies. */
extern Cppi_GlobalCPDMAConfigParams cppiGblCPDMACfgParams[];
extern Cppi_GlobalConfigParams cppiGblCfgParams;

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

extern hplib_RetValue hplib_utilInitProc(void);
extern pid_t gettid(void);

/**********************************************************************
 ************************** Global Variables **************************
 **********************************************************************/

/* Global MCB which keeps track of the SOC Initialization information: */
SOCInit_MCB      gSocInitMCB;

/**********************************************************************
 ************************** Global Variables **************************
 **********************************************************************/

/* Extern device specific power up table */
extern SOCInit_powerUpFxn  gDevicePowerUpTable[];

/* Extern device specific power up table */
extern const int32_t gNumARMCores;

/**********************************************************************
 ********************* SOC Initialization Functions *******************
 **********************************************************************/

/**
 *  @b Description
 *  @n
 *      Logging Function used by the SOC Initialization module
 *
 *  @param[in]  level
 *      Log level
 *  @param[in]  fmt
 *      Formatting string
 *
 *  @retval
 *      Not Applicable.
 */
void SOCInit_log (SOCInit_LogLevel level, char* fmt, ...)
{
    va_list arg;

    /* Log the message as per the configured level. */
    if (level >= gSocInitMCB.logLevel)
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
 *      Utility function which trims the spaces from the buffer
 *      is indicated by the file pointer.
 *
 *  @param[in]  ptrBuffer
 *      Pointer to the data buffer.
 *
 *  @retval
 *      Not applicable
 */
static void SOCInit_trimSpaces (char* ptrBuffer)
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
static char* SOCInit_getToken (char* ptrInitialBuffer)
{
    const char*  delimitters = "<>=\n\r;";
    char*  token;

    while (1)
    {
        /* Parse the token */
        token = strtok(ptrInitialBuffer, delimitters);
        if (token == NULL)
            return NULL;

        /* Trim out spaces and tabs from the token */
        SOCInit_trimSpaces(token);

        /* Did we trim everything out? */
        if (*token != 0)
            break;
    }
    return token;
}

/**
 *  @b Description
 *  @n
 *      Utility function which is used to find a duplicate SOC block. Each CPDMA identifier
 *      should be unique
 *
 *  @param[in]  dmaId
 *      DMA Identifier
 *
 *  @retval
 *      Non NULL -   Matching DMA Block
 *  @retval
 *      NULL     -   No Matching DMA Block
 *
 */
static SOCInit_DMABlock* SOCInit_findSOCBlock(uint32_t dmaId)
{
    SOCInit_DMABlock*   ptrSOCInitDMABlock;

    /* Cycle through all the DMA blocks. */
    ptrSOCInitDMABlock = (SOCInit_DMABlock*)SOCInit_listGetHead ((SOCInit_ListNode**)&gSocInitMCB.ptrDMABlockList);
    while (ptrSOCInitDMABlock != NULL)
    {
        /* Do we have a match? */
        if (ptrSOCInitDMABlock->cpdmaId == dmaId)
            return ptrSOCInitDMABlock;

        /* Get the next DMA Block */
        ptrSOCInitDMABlock = (SOCInit_DMABlock*)SOCInit_listGetNext((SOCInit_ListNode*)ptrSOCInitDMABlock);
    }
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      The function is used to parse the SYSLIB SOC Initialization configuration
 *      file
 *
 *  @param[in]  ptrSOCInitMCB
 *      Pointer to the SOC Initialization MCB
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t SOCInit_parseConfigFile(SOCInit_MCB* ptrSOCInitMCB)
{
    struct stat                 buf;
    FILE*                       fpCfgFile;
    char*                       fileBuffer;
    char*                       ptrFileBuffer;
    char*                       tokenName;
    char*                       tokenValue;
    int32_t                     size;
    SOCInit_DMABlock*           ptrSOCInitDMABlock = NULL;
    SOCInit_PeripheralBlock*    ptrSOCInitPeripheralBlock = NULL;
    SOCInit_BCPBlock*           ptrSOCInitBCPBlock = NULL;
    SOCInit_DDRBlock*           ptrSOCInitDDRBlock = NULL;

    /* Get the file statistics */
    if (stat(ptrSOCInitMCB->cfgFile, &buf) != 0)
    {
        SOCInit_log (SOCInit_LogLevel_ERROR, "Error: Unable to get the file statistics from %s\n", ptrSOCInitMCB->cfgFile);
        return -1;
    }

    /* Allocate memory for the file buffer: */
    fileBuffer = (char*)malloc (buf.st_size + 1);
    if (fileBuffer == NULL)
    {
        SOCInit_log (SOCInit_LogLevel_ERROR, "Error: Unable to allocate memory for the file buffer\n");
        return -1;
    }

    /* Initialize the file buffer. */
    memset ((void *)fileBuffer, 0, buf.st_size + 1);

    /* Open the configuration file: */
    fpCfgFile = fopen (ptrSOCInitMCB->cfgFile, "r");
    if (fpCfgFile == NULL)
    {
        SOCInit_log (SOCInit_LogLevel_ERROR, "Error: Unable to open the configuration file '%s'\n", ptrSOCInitMCB->cfgFile);
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
        tokenName = SOCInit_getToken (ptrFileBuffer);
        if (tokenName == NULL)
            break;

        /* Subsequent calls to the strtok API requires NULL parameters. */
        ptrFileBuffer = NULL;
        size = size + strlen(tokenName);

        /* Process and parse the buffer */
        if (*tokenName == '#' || *tokenName == '\n' || *tokenName == '\r')
            continue;

        /* Process the tokens */
        if (strcmp (tokenName, "dma") == 0)
        {
            /* Allocate the dma classfier block: */
            ptrSOCInitDMABlock = (SOCInit_DMABlock*)malloc (sizeof(SOCInit_DMABlock));
            if (ptrSOCInitDMABlock == NULL)
            {
                /* FATAL Error: */
                SOCInit_log (SOCInit_LogLevel_ERROR, "Error: Unable to allocate memory for the SOC Init DMA Block\n");
                return -1;
            }

            /* Initialize the interface classifier block: */
            memset ((void *)ptrSOCInitDMABlock, 0, sizeof(SOCInit_DMABlock));
        }
        else if (strcmp (tokenName, "peripheral") == 0)
        {
            /* Allocate the peripheral classfier block: */
            ptrSOCInitPeripheralBlock = (SOCInit_PeripheralBlock*)malloc (sizeof(SOCInit_PeripheralBlock));
            if (ptrSOCInitPeripheralBlock == NULL)
            {
                /* FATAL Error: */
                SOCInit_log (SOCInit_LogLevel_ERROR, "Error: Unable to allocate memory for the SOC Init peripheral Block\n");
                return -1;
            }

            /* Initialize the interface classifier block: */
            memset ((void *)ptrSOCInitPeripheralBlock, 0, sizeof(SOCInit_PeripheralBlock));
        }
        else if (strcmp (tokenName, "bcp") == 0)
        {
            /* Allocate the bcp classfier block: */
            ptrSOCInitBCPBlock = (SOCInit_BCPBlock*)malloc (sizeof(SOCInit_BCPBlock));
            if (ptrSOCInitBCPBlock == NULL)
            {
                /* FATAL Error: */
                SOCInit_log (SOCInit_LogLevel_ERROR, "Error: Unable to allocate memory for the SOC Init BCP Block\n");
                return -1;
            }

            /* Initialize the interface classifier block: */
            memset ((void *)ptrSOCInitBCPBlock, 0, sizeof(SOCInit_BCPBlock));
        }
        else if (strcmp (tokenName, "ddr") == 0)
        {
            /* Allocate the ddr classfier block: */
            ptrSOCInitDDRBlock = (SOCInit_DDRBlock*)malloc (sizeof(SOCInit_DDRBlock));
            if (ptrSOCInitDDRBlock == NULL)
            {
                /* FATAL Error: */
                SOCInit_log (SOCInit_LogLevel_ERROR, "Error: Unable to allocate memory for the SOC Init DDR Block\n");
                return -1;
            }

            /* Initialize the interface classifier block: */
            memset ((void *)ptrSOCInitDDRBlock, 0, sizeof(SOCInit_DDRBlock));
        }
        else if (strcmp (tokenName, "name") == 0)
        {
            /* Sanity Check: Validate and ensure that the 'name' was under a valid 'dma' block. */
            if ((ptrSOCInitDMABlock == NULL) && (ptrSOCInitPeripheralBlock == NULL))
            {
                /* FATAL Error: Parsing failure detected. Aborting */
                SOCInit_log (SOCInit_LogLevel_ERROR, "Error: Parsing failure:: Keyword 'name' was specified outside the 'dma' or 'peripheral' block\n");
                return -1;
            }

            /* Get the token value */
            tokenValue = SOCInit_getToken (ptrFileBuffer);
            size = size + strlen(tokenValue);

            /* Populate the interface name into the DMA block */
            if (ptrSOCInitDMABlock != NULL)
                strcpy (ptrSOCInitDMABlock->name, tokenValue);
            else
                strcpy (ptrSOCInitPeripheralBlock->name, tokenValue);
        }
        else if (strcmp (tokenName, "cpdma_id") == 0)
        {
            /* Sanity Check: Validate and ensure that the 'cpdma_id' was under a valid 'dma' block. */
            if (ptrSOCInitDMABlock == NULL)
            {
                /* FATAL Error: Parsing failure detected. Aborting */
                SOCInit_log (SOCInit_LogLevel_ERROR, "Error: Parsing failure:: Keyword 'cpdma_id' was specified outside the 'dma' block\n");
                return -1;
            }

            /* Get the token value */
            tokenValue = SOCInit_getToken (ptrFileBuffer);
            size = size + strlen(tokenValue);

            /* Populate the CPDMA Identifier into the DMA block */
            ptrSOCInitDMABlock->cpdmaId = atoi(tokenValue);
        }
        else if (strcmp (tokenName, "timeout_count") == 0)
        {
            /* Sanity Check: Validate and ensure that the 'timeout_count' was under a valid 'dma' block. */
            if (ptrSOCInitDMABlock == NULL)
            {
                /* FATAL Error: Parsing failure detected. Aborting */
                SOCInit_log (SOCInit_LogLevel_ERROR, "Error: Parsing failure:: Keyword 'timeout_count' was specified outside the 'dma' block\n");
                return -1;
            }

            /* Get the token value */
            tokenValue = SOCInit_getToken (ptrFileBuffer);
            size = size + strlen(tokenValue);

            /* Populate the Timeout count into the DMA block */
            ptrSOCInitDMABlock->timeoutCount = atoi(tokenValue);
        }
        else if (strcmp (tokenName, "write_fifo_depth") == 0)
        {
            /* Sanity Check: Validate and ensure that the 'write_fifo_depth' was under a valid 'dma' block. */
            if (ptrSOCInitDMABlock == NULL)
            {
                /* FATAL Error: Parsing failure detected. Aborting */
                SOCInit_log (SOCInit_LogLevel_ERROR, "Error: Parsing failure:: Keyword 'write_fifo_depth' was specified outside the 'dma' block\n");
                return -1;
            }

            /* Get the token value */
            tokenValue = SOCInit_getToken (ptrFileBuffer);
            size = size + strlen(tokenValue);

            /* Populate the Write FIFO Depth into the DMA block */
            ptrSOCInitDMABlock->writeFifoDepth = atoi(tokenValue);
        }
        else if (strcmp (tokenName, "rx_priority") == 0)
        {
            /* Sanity Check: Validate and ensure that the 'rx_priority' was under a valid 'dma' block. */
            if (ptrSOCInitDMABlock == NULL)
            {
                /* FATAL Error: Parsing failure detected. Aborting */
                SOCInit_log (SOCInit_LogLevel_ERROR, "Error: Parsing failure:: Keyword 'rx_priority' was specified outside the 'dma' block\n");
                return -1;
            }

            /* Get the token value */
            tokenValue = SOCInit_getToken (ptrFileBuffer);
            size = size + strlen(tokenValue);

            /* Populate the Receive Priority into the DMA block */
            ptrSOCInitDMABlock->rxPriority = atoi(tokenValue);
        }
        else if (strcmp (tokenName, "tx_priority") == 0)
        {
            /* Sanity Check: Validate and ensure that the 'tx_priority' was under a valid 'dma' block. */
            if (ptrSOCInitDMABlock == NULL)
            {
                /* FATAL Error: Parsing failure detected. Aborting */
                SOCInit_log (SOCInit_LogLevel_ERROR, "Error: Parsing failure:: Keyword 'tx_priority' was specified outside the 'dma' block\n");
                return -1;
            }

            /* Get the token value */
            tokenValue = SOCInit_getToken (ptrFileBuffer);
            size = size + strlen(tokenValue);

            /* Populate the Transmit Priority into the DMA block */
            ptrSOCInitDMABlock->txPriority = atoi(tokenValue);
        }
        else if (strcmp (tokenName, "instance_id") == 0)
        {
            /* Sanity Check: Validate and ensure that the 'instance_id' was under a valid 'peripheral' or 'bcp' block. */
            if ((ptrSOCInitPeripheralBlock == NULL) && (ptrSOCInitBCPBlock == NULL) && (ptrSOCInitDDRBlock == NULL))
            {
                /* FATAL Error: Parsing failure detected. Aborting */
                SOCInit_log (SOCInit_LogLevel_ERROR, "Error: Parsing failure:: Keyword 'instance_id' was specified outside the 'peripheral' or 'bcp' or 'ddr' block\n");
                return -1;
            }

            /* Get the token value */
            tokenValue = SOCInit_getToken (ptrFileBuffer);
            size = size + strlen(tokenValue);

            /* Populate the instance_id into the peripheral or bcp block */
            if (ptrSOCInitPeripheralBlock != NULL)
                ptrSOCInitPeripheralBlock->instanceId = atoi(tokenValue);
            else if (ptrSOCInitBCPBlock != NULL)
                ptrSOCInitBCPBlock->instanceId = atoi(tokenValue);
            else
                ptrSOCInitDDRBlock->instanceId = atoi(tokenValue);
        }
        else if (strcmp (tokenName, "power_up") == 0)
        {
            /* Sanity Check: Validate and ensure that the 'power_up' was under a valid 'dma' block. */
            if ((ptrSOCInitDMABlock == NULL) && (ptrSOCInitPeripheralBlock == NULL))
            {
                /* FATAL Error: Parsing failure detected. Aborting */
                SOCInit_log (SOCInit_LogLevel_ERROR, "Error: Parsing failure:: Keyword 'power_up' was specified outside the 'dma' or 'peripheral' block\n");
                return -1;
            }

            /* Get the token value */
            tokenValue = SOCInit_getToken (ptrFileBuffer);
            size = size + strlen(tokenValue);

            /* Populate the Power up status into the DMA block */
            if (ptrSOCInitDMABlock != NULL)
                ptrSOCInitDMABlock->powerUp = atoi(tokenValue);
            else
                ptrSOCInitPeripheralBlock->powerUp = atoi(tokenValue);
        }
        else if (strcmp (tokenName, "Q_PPBMap") == 0)
        {
            uint32_t txq = 0;
            /* Sanity Check: Validate and ensure that the 'Q_PPBMap' was under a valid 'bcp' block. */
            if (ptrSOCInitBCPBlock == NULL)
            {
                /* FATAL Error: Parsing failure detected. Aborting */
                SOCInit_log (SOCInit_LogLevel_ERROR, "Error: Parsing failure:: Keyword 'Q_PPBMap' was specified outside the 'bcp' block\n");
                return -1;
            }
            for (txq=0;txq<BCP_MAX_NUM_TXQUEUES;txq++) {
                /* Get the token value */
                tokenValue = SOCInit_getToken (ptrFileBuffer);
                size = size + strlen(tokenValue);

                /* Populate the Q_PPBMap into the BCP block */
                ptrSOCInitBCPBlock->Q_to_PPB_map[txq]= atoi(tokenValue);
            }
        }
        else if (strcmp (tokenName, "Q_PRI") == 0)
        {
            uint32_t txq = 0;
            /* Sanity Check: Validate and ensure that the 'Q_PRI' was under a valid 'bcp' block. */
            if (ptrSOCInitBCPBlock == NULL)
            {
                /* FATAL Error: Parsing failure detected. Aborting */
                SOCInit_log (SOCInit_LogLevel_ERROR, "Error: Parsing failure:: Keyword 'Q_PRI' was specified outside the 'bcp' block\n");
                return -1;
            }
            for (txq=0;txq<BCP_MAX_NUM_TXQUEUES;txq++) {
                /* Get the token value */
                tokenValue = SOCInit_getToken (ptrFileBuffer);
                size = size + strlen(tokenValue);

                /* Populate the Q_PRI into the BCP block */
                ptrSOCInitBCPBlock->Qpri[txq]= atoi(tokenValue);
            }
        }
        else if (strcmp (tokenName, "VBUSM_CONFIG") == 0)
        {
            /* Sanity Check: Validate and ensure that the 'VBUSM_CONFIG' was under a valid 'ddr' block. */
            if (ptrSOCInitDDRBlock == NULL)
            {
                /* FATAL Error: Parsing failure detected. Aborting */
                SOCInit_log (SOCInit_LogLevel_ERROR, "Error: Parsing failure:: Keyword 'VBUSM_CONFIG' was specified outside the 'ddr' block\n");
                return -1;
            }

            /* Get the token value */
            tokenValue = SOCInit_getToken (ptrFileBuffer);
            size = size + strlen(tokenValue);

            /* Populate the vbusm_config into the DDR block */
            ptrSOCInitDDRBlock->vbusm_config= (uint32_t)strtod(tokenValue, NULL);
            SOCInit_log (SOCInit_LogLevel_DEBUG, "Debug: vbusm_config: 0x%x %s\n",ptrSOCInitDDRBlock->vbusm_config,tokenValue);
        }
        else if (strcmp (tokenName, "MSTID_COS_1_MAP") == 0)
        {
            /* Sanity Check: Validate and ensure that the 'MSTID_COS_1_MAP' was under a valid 'ddr' block. */
            if (ptrSOCInitDDRBlock == NULL)
            {
                /* FATAL Error: Parsing failure detected. Aborting */
                SOCInit_log (SOCInit_LogLevel_ERROR, "Error: Parsing failure:: Keyword 'MSTID_COS_1_MAP' was specified outside the 'ddr' block\n");
                return -1;
            }

            /* Get the token value */
            tokenValue = SOCInit_getToken (ptrFileBuffer);
            size = size + strlen(tokenValue);

            /* Populate the mstid_cos_1_map into the DDR block */
            ptrSOCInitDDRBlock->mstid_cos_1_map= (uint32_t)strtod(tokenValue, NULL);
            SOCInit_log (SOCInit_LogLevel_DEBUG, "Debug: mstid_cos_1_map: 0x%x %s\n",ptrSOCInitDDRBlock->mstid_cos_1_map,tokenValue);
        }
        else if (strcmp (tokenName, "MSTID_COS_2_MAP") == 0)
        {
            /* Sanity Check: Validate and ensure that the 'MSTID_COS_2_MAP' was under a valid 'ddr' block. */
            if (ptrSOCInitDDRBlock == NULL)
            {
                /* FATAL Error: Parsing failure detected. Aborting */
                SOCInit_log (SOCInit_LogLevel_ERROR, "Error: Parsing failure:: Keyword 'MSTID_COS_2_MAP' was specified outside the 'ddr' block\n");
                return -1;
            }

            /* Get the token value */
            tokenValue = SOCInit_getToken (ptrFileBuffer);
            size = size + strlen(tokenValue);

            /* Populate the mstid_cos_2_map into the DDR block */
            ptrSOCInitDDRBlock->mstid_cos_2_map= (uint32_t)strtod(tokenValue, NULL);
            SOCInit_log (SOCInit_LogLevel_DEBUG, "Debug: mstid_cos_2_map: 0x%x %s\n",ptrSOCInitDDRBlock->mstid_cos_2_map,tokenValue);
        }
        else if (strcmp (tokenName, "{") == 0)
        {
            continue;
        }
        else if (strcmp (tokenName, "}") == 0)
        {
            /* Sanity Check: DMA block ends: Ensure that the interface configuration was started */
            if ((ptrSOCInitDMABlock == NULL) && (ptrSOCInitPeripheralBlock == NULL) &&
                (ptrSOCInitBCPBlock == NULL) && (ptrSOCInitDDRBlock == NULL))
            {
                /* FATAL Error: Parsing failure detected. Aborting */
                SOCInit_log (SOCInit_LogLevel_ERROR, "Error: Parsing failure:: DMA/BCP/DDR/Peripheral block end detected without a block start\n");
                return -1;
            }
            if (ptrSOCInitDMABlock != NULL)
            {
                SOCInit_DMABlock* ptrDuplicationDMABlock;

                /* Sanity Check: Validate all the arguments required by the DMA block.
                 * We should have a valid interface name. */
                if (ptrSOCInitDMABlock->name[0] == 0)
                {
                    SOCInit_log (SOCInit_LogLevel_ERROR, "Error: DMA block detected without a valid name\n");
                    return -1;
                }

                /* Sanity Check: We cannot have a CPDMA identifier which exceeds the MAX */
                if (ptrSOCInitDMABlock->cpdmaId > CPPI_MAX_CPDMA)
                {
                    SOCInit_log (SOCInit_LogLevel_ERROR, "Error: CDPMA Id for DMA '%s': %d exceeds MAX allowed %d\n",
                                 ptrSOCInitDMABlock->name, ptrSOCInitDMABlock->cpdmaId, CPPI_MAX_CPDMA);
                    return -1;
                }

                /* Ensure that there is no duplication */
                ptrDuplicationDMABlock = SOCInit_findSOCBlock (ptrSOCInitDMABlock->cpdmaId);
                if (ptrDuplicationDMABlock != NULL)
                {
                    SOCInit_log (SOCInit_LogLevel_ERROR, "Error: DMA identifier %d has already been specified for '%s'\n",
                                 ptrSOCInitDMABlock->cpdmaId, ptrDuplicationDMABlock->name);
                    return -1;
                }

                /* Add this to the list of DMA blocks which have been detected */
                SOCInit_listAdd ((SOCInit_ListNode**)&gSocInitMCB.ptrDMABlockList, (SOCInit_ListNode*)ptrSOCInitDMABlock);

                /* Interface block work is over: */
                ptrSOCInitDMABlock = NULL;
            }
            if (ptrSOCInitPeripheralBlock != NULL)
            {
                /* Sanity Check: Validate all the arguments required by the peripheral block.
                 * We should have a valid interface name. */
                if (ptrSOCInitPeripheralBlock->name[0] == 0)
                {
                    SOCInit_log (SOCInit_LogLevel_ERROR, "Error: peripheral block detected without a valid name\n");
                    return -1;
                }
                /* Add this to the list of DMA blocks which have been detected */
                SOCInit_listAdd ((SOCInit_ListNode**)&gSocInitMCB.ptrPeripheralBlockList, (SOCInit_ListNode*)ptrSOCInitPeripheralBlock);

                /* Interface block work is over: */
                ptrSOCInitPeripheralBlock = NULL;
            }
            if (ptrSOCInitBCPBlock != NULL)
            {
                /* Add this to the list of DMA blocks which have been detected */
                SOCInit_listAdd ((SOCInit_ListNode**)&gSocInitMCB.ptrBCPBlockList, (SOCInit_ListNode*)ptrSOCInitBCPBlock);

                /* Interface block work is over: */
                ptrSOCInitBCPBlock = NULL;
            }
            if (ptrSOCInitDDRBlock != NULL)
            {
                /* Add this to the list of DMA blocks which have been detected */
                SOCInit_listAdd ((SOCInit_ListNode**)&gSocInitMCB.ptrDDRBlockList, (SOCInit_ListNode*)ptrSOCInitDDRBlock);

                /* Interface block work is over: */
                ptrSOCInitDDRBlock = NULL;
            }
        }
        else
        {
            SOCInit_log (SOCInit_LogLevel_ERROR, "Error: Parsing error; invalid token '%s' detected\n", tokenName);
            return -1;
        }
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *     Display the usage for the SOC Init application
 *
 *  @retval
 *      Not Applicable.
 */
static void SOCInit_displayUsage (void)
{
    printf ("SYSLIB SOC Init Application:\n");
    printf ("Mandatory Arguments: \n");
    printf ("-r <name>           - Name of the RM Client to be used; this should match the client names in the RMv2 Policy file\n");
    printf ("-c <soc_init.conf>  - SYSLIB SOC Initialization configuration file.\n");
    printf ("Optional Arguments: \n");
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
 *      Error        - <0
 */
static int32_t SOCInit_processCmdLineArgs(int32_t argc, char* argv[])
{
    while (1)
    {
        int option_index = 0;
        int c;

        static struct option long_options[] = {
            {"rmClientName",    required_argument, 0,  0 },
            {"cfgFile",         required_argument, 0,  0 },
            {0,                 0,                 0,  0 }
        };

        c = getopt_long(argc, argv, "vr:c:", long_options, &option_index);
        if (c == -1)
            break;

       switch (c)
       {
            case 'r':
            {
                /* RM Client Name: */
                strncpy (gSocInitMCB.rmClientName, optarg, sizeof(gSocInitMCB.rmClientName));
                break;
            }
            case 'c':
            {
                /* Configuration file name */
                strncpy (gSocInitMCB.cfgFile, optarg, PATH_MAX);
                break;
            }
            case 'v':
            {
                gSocInitMCB.logLevel = SOCInit_LogLevel_DEBUG;
                break;
            }
            case '?':
            {
                return -1;
            }
        }
    }

    /* We should have a valid RM client name */
    if (gSocInitMCB.rmClientName[0] == 0)
        return -1;

    /* We should have a valid client list. */
    if (gSocInitMCB.cfgFile[0] == 0)
        return -1;

    /* Command Line Arguments processed successfully. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The thread is used to initialize the HPLIB on a specific core.
 *
 *  @param[in]  arg
 *      This is the ARM core number.
 *
 * \ingroup DAT_INTERNAL_FUNCTION
 *
 *  @retval
 *      Always return NULL
 */
static void* SOCInit_hplibProcThread(void *arg)
{
    uint32_t     core;
    cpu_set_t    set;
    unsigned long long timestamp= 0;

    /* Get core number */
    core = *(uint32_t*)arg;

    /* Set the thread affinity: */
    CPU_ZERO(&set);
    CPU_SET(core, &set);
    if (sched_setaffinity(gettid(), sizeof(cpu_set_t), &set))
    {
        SOCInit_log (SOCInit_LogLevel_ERROR,"Error: Unable to set the scheduler affinity for core %d [Error %s]\n", core, strerror(errno));
        return NULL;
    }

    /* Call hplib util function */
    if (hplib_utilInitProc() != hplib_OK)
    {
        SOCInit_log (SOCInit_LogLevel_ERROR, "Error: HPLIB Initialization failed\n");
        return NULL;
    }

    /* Get the current timestamp */
    timestamp = hplib_mUtilGetTimestamp();

    /* Informational Message: */
    SOCInit_log (SOCInit_LogLevel_INFO, "Debug: HPLIB Initialized on core %d Current Time Stamp[0x%x:0x%x]\n",
                 core, (uint32_t)((timestamp >> 32 ) & 0xFFFFFFFF), (uint32_t)(timestamp & 0xFFFFFFFF));
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      The function is used to initialize HPLIB across all the cores. ARM applications
 *      will now have access to the timestamp & PMU counters.
 *
 *  @param[in]  ptrSOCInitMCB
 *      Pointer to the application MCB
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t SOCInit_initHPLIB(SOCInit_MCB* ptrSOCInitMCB)
{
    uint32_t            core;
    pthread_t           thread;
    int32_t             hplibModuleFd;
    pthread_attr_t      attr;
    struct sched_param  param;

    /* Open Hplib , it will used to get hardware timestamp */
    hplibModuleFd = hplib_utilModOpen();
    if(hplibModuleFd == -1)
    {
        SOCInit_log (SOCInit_LogLevel_ERROR, "Error: Open HPlib module failed with error: '%s'\n",  strerror(errno));
        return -1;
    }

    /* Initialize the HPLIB across all the ARM cores. */
    for(core = 0; core < gNumARMCores; core++)
    {
        /* Set up thread attributes */
        pthread_attr_init(&attr);

        /* Setting up the scheduling priority: */
        param.sched_priority = 60;
        pthread_attr_setschedparam(&attr, &param);
        pthread_create(&thread, &attr, SOCInit_hplibProcThread, (void *)&core);

        /* Wait for the thread to finish */
        pthread_join(thread, NULL);
    }
    return 0;
}


#define SOC_INIT_BARRIER_INVALID 0xffffffffu

static uint32_t SOC_InitGetFromDts
(
    char *resourceName
)
{
    uint32_t      value         = SOC_INIT_BARRIER_INVALID;
    int32_t       errCode       = 0;

    if (Resmgr_allocCustomResource(gSocInitMCB.handleSysCfg, resourceName, 1, &value, &errCode) != 0) {
        SOCInit_log(SOCInit_LogLevel_ERROR,
                    "Custom resource '%s' alloc failed! [errCode: %d]\n",
                    resourceName,
                    errCode);
    }
    return value;
}


static int32_t SOC_InitQmssBarrier(void) {
    int32_t       retVal        = 0;
    int32_t       errCode       = 0;
    // Get MSMC barrier infos
    uint32_t      msmcQnum      = SOC_InitGetFromDts("msmc_barrier_Q");
    uint32_t      msmcQnumNetcp = SOC_InitGetFromDts("msmc_barrier_Q_NetCP");
    Qmss_PdspId   pdspIdMsmc    = (Qmss_PdspId)SOC_InitGetFromDts("msmc_barrier_PDSPID");
    // Get DDR barrier infos
    uint32_t      ddrQnum       = SOC_InitGetFromDts("ddr_barrier_Q");
    uint32_t      ddrQnumNetcp  = SOC_InitGetFromDts("ddr_barrier_Q_NetCP");
    Qmss_PdspId   pdspIdDdr     = (Qmss_PdspId)SOC_InitGetFromDts("ddr_barrier_PDSPID");


    if (msmcQnum      != SOC_INIT_BARRIER_INVALID &&
        msmcQnumNetcp != SOC_INIT_BARRIER_INVALID &&
        pdspIdMsmc    != (Qmss_PdspId)SOC_INIT_BARRIER_INVALID) {
        // configure MSMC barrier Q
        SOCInit_log(SOCInit_LogLevel_INFO,
                    "  ========== MSMC Barrier queue is %5u , NetCP queue %5u , PDSP ID %2u ============\n",
                    msmcQnum, msmcQnumNetcp, pdspIdMsmc);

        errCode = Qmss_programMSMCBarrierQueue(pdspIdMsmc,
                                               (Qmss_QueueHnd)msmcQnum,
                                               (Qmss_QueueHnd)msmcQnumNetcp);
        if (errCode < QMSS_SOK) {
            SOCInit_log(SOCInit_LogLevel_ERROR,
                        "    ==> MSMC Barrier Q init failed! [errCode: %d]\n",
                        errCode);
            retVal = -1;
        } else {
            SOCInit_log(SOCInit_LogLevel_INFO,
                        "    * MSMC Barrier Q initialized!\n");

            // Store Q num to RM for MsgCom
            if (Resmgr_nameServiceSet(gSocInitMCB.handleSysCfg,
                                      "QMSS_BarrierQ_MSMC",
                                      msmcQnum,
                                      &errCode) < 0) {
                SOCInit_log(SOCInit_LogLevel_ERROR,
                            "    ==> Cannot store queue id for later usage...\n");
                retVal = -1;
            }
            // Store NetCP Q num to RM for PA
            if (Resmgr_nameServiceSet(gSocInitMCB.handleSysCfg,
                                      "QMSS_BarrierQ_MSMC_NetCP",
                                      msmcQnumNetcp,
                                      &errCode) < 0) {
                SOCInit_log(SOCInit_LogLevel_ERROR,
                            "    ==> Cannot store netcp queue id for later usage...\n");
                retVal = -1;
            }

        }
    }


    if (ddrQnum      != SOC_INIT_BARRIER_INVALID &&
        ddrQnumNetcp != SOC_INIT_BARRIER_INVALID &&
        pdspIdDdr    != (Qmss_PdspId)SOC_INIT_BARRIER_INVALID) {
        // configure DDR barrier Q
        SOCInit_log(SOCInit_LogLevel_INFO,
                    "  ========== DDR Barrier queue is %5u , NetCP queue %5u , PDSP ID %2u ============\n",
                    ddrQnum, ddrQnumNetcp, pdspIdDdr);

        errCode = Qmss_programDDRBarrierQueue(pdspIdDdr,
                                              (Qmss_QueueHnd)ddrQnum,
                                              (Qmss_QueueHnd)ddrQnumNetcp);
        if (errCode < QMSS_SOK) {
            SOCInit_log(SOCInit_LogLevel_ERROR,
                        "    ==> DDR Barrier Q init failed! [errCode: %d]\n",
                        errCode);
            retVal = -1;
        } else {
            SOCInit_log(SOCInit_LogLevel_INFO,
                        "    * DDR Barrier Q initialized!\n");

            // Store Q num to RM for MsgCom
            if (Resmgr_nameServiceSet(gSocInitMCB.handleSysCfg,
                                      "QMSS_BarrierQ_DDR",
                                      ddrQnum,
                                      &errCode) < 0) {
                SOCInit_log(SOCInit_LogLevel_ERROR,
                            "    ==> Cannot store queue id for later usage...\n");
                retVal = -1;
            }
            // Store NetCP Q num to RM for PA
            if (Resmgr_nameServiceSet(gSocInitMCB.handleSysCfg,
                                      "QMSS_BarrierQ_DDR_NetCP",
                                      ddrQnumNetcp,
                                      &errCode) < 0) {
                SOCInit_log(SOCInit_LogLevel_ERROR,
                            "    ==> Cannot store netcp queue id for later usage...\n");
                retVal = -1;
            }

        }
    }

    return retVal;
}


/**
 *  @b Description
 *  @n
 *      SOC Initialization Entry point
 *
 *  @param[in]  argc
 *      Number of arguments.
 *  @param[in]  argv
 *      Command Line arguments.
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error        - <0
 */
int32_t main(int argc, char* argv[])
{
    Resmgr_SystemCfg                sysConfig;
    Cppi_CpDmaInitCfg               cpdmaCfg;
    int32_t                         errCode;
    SOCInit_DMABlock*               ptrSOCInitDMABlock;
    SOCInit_PeripheralBlock*        ptrSOCInitPeripheralBlock;
    SOCInit_BCPBlock*               ptrSOCInitBCPBlock;
    SOCInit_DDRBlock*               ptrSOCInitDDRBlock;
    Cppi_GlobalCPDMAConfigParams*   ptrCPDMAGlobalConfig;
    Cppi_Handle                     cppiHandle;
    Qmss_Result                     result;

    /* Initialize the memory block */
    memset ((void *)&gSocInitMCB, 0, sizeof(SOCInit_MCB));

    /* By default the verbosity is turned off */
    gSocInitMCB.logLevel = SOCInit_LogLevel_ERROR;

    /* Cycle through the arguments and parse the command line arguments. */
    if (SOCInit_processCmdLineArgs(argc, argv) < 0)
    {
        SOCInit_displayUsage();
        return -1;
    }

    /* Parse the configuration file: */
    if (SOCInit_parseConfigFile (&gSocInitMCB) < 0)
        return -1;

    /********************************************************************************************
     * Device Specific Validations & Initializations:
     ********************************************************************************************/
    if (SOCInit_validateDMABlocks(&gSocInitMCB) < 0)
        return -1;
    if (SOCInit_initHPLIB(&gSocInitMCB) < 0)
        return -1;

    /********************************************************************************************
     * Update the global CPPI Device with the configuration parameters from the configuration
     * This is done before we initialize the CPPI in the RESMGR. This will ensure that the
     * global structure is properly populated with the CPDMA Rx and Tx priorities.
     ********************************************************************************************/
    ptrSOCInitDMABlock = (SOCInit_DMABlock*)SOCInit_listGetHead ((SOCInit_ListNode**)&gSocInitMCB.ptrDMABlockList);
    while (ptrSOCInitDMABlock != NULL)
    {
        /* Get the CPDMA Global configuration parameters: */
        ptrCPDMAGlobalConfig = &cppiGblCfgParams.cpDmaCfgs[ptrSOCInitDMABlock->cpdmaId];

        /* Update the receive and transmit priority: */
        ptrCPDMAGlobalConfig->rxPriority = ptrSOCInitDMABlock->rxPriority;
        ptrCPDMAGlobalConfig->txPriority = ptrSOCInitDMABlock->txPriority;

        /* Debug Message: */
        SOCInit_log (SOCInit_LogLevel_DEBUG, "Debug: DMA '%s' Rx Priority: %d Tx Priority: %d \n",
                     ptrSOCInitDMABlock->name, ptrSOCInitDMABlock->rxPriority, ptrSOCInitDMABlock->txPriority);

        /* Get the next DMA Block */
        ptrSOCInitDMABlock = (SOCInit_DMABlock*)SOCInit_listGetNext((SOCInit_ListNode*)ptrSOCInitDMABlock);
    }

    /********************************************************************************************
     * Initialize the resource manager module
     ********************************************************************************************/

    /* Initialize the system configuration. */
    memset ((void *)&sysConfig, 0, sizeof(Resmgr_SystemCfg));

    /* Populate the configuration.*/
    sysConfig.realm                 = Resmgr_ExecutionRealm_ARM;
    strcpy (sysConfig.rmClient, gSocInitMCB.rmClientName);
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
    gSocInitMCB.handleSysCfg = Resmgr_init(&sysConfig, &errCode);
    if (gSocInitMCB.handleSysCfg == NULL)
    {
        SOCInit_log(SOCInit_LogLevel_ERROR, "Error: System configuration initialization failed [Error code %d]\n", errCode);
        return -1;
    }

    /********************************************************************************************
     * QMSS PDSP3 Download: This needs to be done for the entire system. QMSS PDSP1 is downloaded
     * already by the Linux kernel.
     ********************************************************************************************/
    result = Qmss_downloadFirmware (Qmss_PdspId_PDSP3, &acc48_le, sizeof (acc48_le));
    if (result != QMSS_SOK)
    {
        SOCInit_log(SOCInit_LogLevel_ERROR, "Error: Downloading PDSP3 failed [Result %d]\n", result);
        return -1;
    }
    SOCInit_log(SOCInit_LogLevel_DEBUG, "Debug: QMSS PDSP3 Download successful\n");


    if (SOC_InitQmssBarrier() < 0) {
        SOCInit_log(SOCInit_LogLevel_ERROR, "FAILED!\n");
    }


    /********************************************************************************************
     * Power up all the CPDMA blocks which were configured to be done:
     ********************************************************************************************/
    gSocInitMCB.pscReg = (CSL_PscRegs* )hplib_VM_MemMap ((void*)CSL_PSC_REGS, 0x10000);
    if (gSocInitMCB.pscReg == NULL)
    {
        SOCInit_log(SOCInit_LogLevel_ERROR, "Error: Memory mapping PSC register space failed\n");
        return -1;
    }
    SOCInit_log(SOCInit_LogLevel_DEBUG, "Debug: PSC mapping successful\n");

    /* Cycle through all the registered DMA blocks: */
    ptrSOCInitDMABlock = (SOCInit_DMABlock*)SOCInit_listGetHead ((SOCInit_ListNode**)&gSocInitMCB.ptrDMABlockList);
    while (ptrSOCInitDMABlock != NULL)
    {
        /* Do we need to power up the DMA? */
        if (ptrSOCInitDMABlock->powerUp == 0)
        {
            /* No. The configuration file skips this. */
            SOCInit_log (SOCInit_LogLevel_DEBUG, "Debug: DMA '%s' skipping power up\n", ptrSOCInitDMABlock->name);
            ptrSOCInitDMABlock = (SOCInit_DMABlock*)SOCInit_listGetNext((SOCInit_ListNode*)ptrSOCInitDMABlock);
            continue;
        }

        /* Is there a power up function registered? */
        if (gDevicePowerUpTable[ptrSOCInitDMABlock->cpdmaId] == NULL)
        {
            /* No. The DMA is either already powered on[Linux kernel powered on QMSS, NETCP etc] *OR* we currently dont support it
             * Report this information back to the user. This is *not* a fatal error so we continue with the rest of the DMA blocks. */
            SOCInit_log (SOCInit_LogLevel_INFO, "Debug: Skipping DMA '%s' no power up [Is the DMA powered on?]\n", ptrSOCInitDMABlock->name);
            ptrSOCInitDMABlock = (SOCInit_DMABlock*)SOCInit_listGetNext((SOCInit_ListNode*)ptrSOCInitDMABlock);
            continue;
        }

        /* Invoke the power up function: */
        if (gDevicePowerUpTable[ptrSOCInitDMABlock->cpdmaId] (&gSocInitMCB, ptrSOCInitDMABlock) < 0)
        {
            /* FATAL Error: Unable to power up the DMA block. */
            SOCInit_log (SOCInit_LogLevel_ERROR, "Error: Powering up the DMA '%s' failed\n", ptrSOCInitDMABlock->name);
            return -1;
        }

        /* Debug Message: */
        SOCInit_log (SOCInit_LogLevel_DEBUG, "Debug: DMA '%s' powered up successfully\n", ptrSOCInitDMABlock->name);

        /* Get the next DMA Block */
        ptrSOCInitDMABlock = (SOCInit_DMABlock*)SOCInit_listGetNext((SOCInit_ListNode*)ptrSOCInitDMABlock);
    }

    /********************************************************************************************
     * Peripheral configuration: Cycle through all the registered Peripheral blocks
     ********************************************************************************************/
    ptrSOCInitPeripheralBlock = (SOCInit_PeripheralBlock*)SOCInit_listGetHead ((SOCInit_ListNode**)&gSocInitMCB.ptrPeripheralBlockList);
    while (ptrSOCInitPeripheralBlock != NULL)
    {
        /* Do we need to power up the DMA? */
        if (ptrSOCInitPeripheralBlock->powerUp == 0)
        {
            /* No. The configuration file skips this. */
            SOCInit_log (SOCInit_LogLevel_DEBUG, "Debug: Peripheral '%s' instance %d skipping power up\n",
                         ptrSOCInitPeripheralBlock->name, ptrSOCInitPeripheralBlock->instanceId);
            ptrSOCInitPeripheralBlock = (SOCInit_PeripheralBlock*)SOCInit_listGetNext((SOCInit_ListNode*)ptrSOCInitPeripheralBlock);
            continue;
        }

        /* YES: Power up the peripheral */
        if (SOCInit_powerPeripheral(&gSocInitMCB, ptrSOCInitPeripheralBlock))
        {
            /* FATAL Error: Unable to power up the Peripheral block. */
            SOCInit_log (SOCInit_LogLevel_ERROR, "Error: Powering up the Peripheral '%s' instance %d failed\n",
                         ptrSOCInitPeripheralBlock->name, ptrSOCInitPeripheralBlock->instanceId);
            return -1;
        }

        /* Debug Message: */
        SOCInit_log (SOCInit_LogLevel_DEBUG, "Debug: Peripheral '%s' instance %d powered up successfully\n",
                     ptrSOCInitPeripheralBlock->name, ptrSOCInitPeripheralBlock->instanceId);

        /* Get the next Peripheral Block */
        ptrSOCInitPeripheralBlock = (SOCInit_PeripheralBlock*)SOCInit_listGetNext((SOCInit_ListNode*)ptrSOCInitPeripheralBlock);
    }

    /********************************************************************************************
     * BCP configuration: Cycle through all the BCP devices
     ********************************************************************************************/
    gSocInitMCB.BCPCfgRegs = (uint32_t)hplib_VM_MemMap ((void*)CSL_BCP_CFG_REGS, 0xD000);
    if (gSocInitMCB.BCPCfgRegs == 0)
    {
        SOCInit_log(SOCInit_LogLevel_ERROR, "Error: Memory mapping BCP CFG register space failed\n");
        return -1;
    }
    SOCInit_log(SOCInit_LogLevel_DEBUG, "Debug: BCP CFG mapping successful\n");
    ptrSOCInitBCPBlock = (SOCInit_BCPBlock*)SOCInit_listGetHead ((SOCInit_ListNode**)&gSocInitMCB.ptrBCPBlockList);
    while (ptrSOCInitBCPBlock != NULL)
    {
        /* Initialize the BCP Device: */
        if (SOCInit_initBCPDevice(&gSocInitMCB, ptrSOCInitBCPBlock))
        {
            /* FATAL Error: Unable to power up the Peripheral block. */
            SOCInit_log (SOCInit_LogLevel_ERROR, "Error: SOCInit_initBCPDevice %d failed\n", ptrSOCInitBCPBlock->instanceId);
            return -1;
        }

        /* Debug Message: */
        SOCInit_log (SOCInit_LogLevel_DEBUG, "Debug: SOCInit_initBCPDevice %d done\n", ptrSOCInitBCPBlock->instanceId);

        /* Get the next Peripheral Block */
        ptrSOCInitBCPBlock = (SOCInit_BCPBlock*)SOCInit_listGetNext((SOCInit_ListNode*)ptrSOCInitBCPBlock);
    }

    /********************************************************************************************
     * DDR3 configuration: Cycle through all the registered DDR blocks
     ********************************************************************************************/
    ptrSOCInitDDRBlock = (SOCInit_DDRBlock*)SOCInit_listGetHead ((SOCInit_ListNode**)&gSocInitMCB.ptrDDRBlockList);
    while (ptrSOCInitDDRBlock != NULL)
    {
        if (SOCInit_configureDDR(&gSocInitMCB, ptrSOCInitDDRBlock))
        {
            /* FATAL Error: Unable to power up the Peripheral block. */
            SOCInit_log (SOCInit_LogLevel_ERROR, "Error: SOCInit_configureDDR %d failed\n", ptrSOCInitDDRBlock->instanceId);
            return -1;
        }

        /* Debug Message: */
        SOCInit_log (SOCInit_LogLevel_DEBUG, "Debug: SOCInit_configureDDR %d done\n", ptrSOCInitDDRBlock->instanceId);

        /* Get the next Peripheral Block */
        ptrSOCInitDDRBlock = (SOCInit_DDRBlock*)SOCInit_listGetNext((SOCInit_ListNode*)ptrSOCInitDDRBlock);
    }

    /********************************************************************************************
     * CPDMA configuration: Cycle through the DMA list and configure all the registered DMA blocks
     ********************************************************************************************/
    ptrSOCInitDMABlock = (SOCInit_DMABlock*)SOCInit_listRemove ((SOCInit_ListNode**)&gSocInitMCB.ptrDMABlockList);
    while (ptrSOCInitDMABlock != NULL)
    {
        /* Sanity Check: Ensure that the CPDMA identifier is not outside the range specified in the device file */
        if (ptrSOCInitDMABlock->cpdmaId > CPPI_MAX_CPDMA)
        {
            SOCInit_log (SOCInit_LogLevel_ERROR, "Error: CDPMA Id for DMA '%s': %d exceeds MAX allowed %d\n",
                         ptrSOCInitDMABlock->name, ptrSOCInitDMABlock->cpdmaId, CPPI_MAX_CPDMA);
            return -1;
        }

        /* Get the CPDMA Global configuration parameters: */
        ptrCPDMAGlobalConfig = &cppiGblCfgParams.cpDmaCfgs[ptrSOCInitDMABlock->cpdmaId];

        /* Sanity Check: This is done to ensure that the configuration file matches the CPPI device file. */
        if (ptrSOCInitDMABlock->cpdmaId != (uint32_t)ptrCPDMAGlobalConfig->dmaNum)
        {
            SOCInit_log (SOCInit_LogLevel_ERROR, "Error: Mismatch in the DMA '%s' [Config File %d CPPI Device %d]\n",
                         ptrSOCInitDMABlock->name, ptrSOCInitDMABlock->cpdmaId, ptrCPDMAGlobalConfig->dmaNum);
            return -1;
        }

        /* Do we have a valid configuration space? */
        if (ptrCPDMAGlobalConfig->gblCfgRegs != NULL)
        {
            /* YES. Initialize the configuration block */
            memset ((void *)&cpdmaCfg, 0, sizeof (Cppi_CpDmaInitCfg));

            /* Populate the CPDMA configuration block: */
            cpdmaCfg.dmaNum         = ptrSOCInitDMABlock->cpdmaId;
            cpdmaCfg.timeoutCount   = ptrSOCInitDMABlock->timeoutCount;
            cpdmaCfg.writeFifoDepth = ptrSOCInitDMABlock->writeFifoDepth;
            cpdmaCfg.regWriteFlag   = Cppi_RegWriteFlag_ON;

            /* Open the CPPI Handle: */
            cppiHandle = Cppi_open (&cpdmaCfg);
            if (cppiHandle == NULL)
            {
                SOCInit_log(SOCInit_LogLevel_ERROR, "Error: Unable to open the CPPI for DMA '%s'\n", ptrSOCInitDMABlock->name);
                return -1;
            }

            /* Debug Message: */
            SOCInit_log (SOCInit_LogLevel_DEBUG, "Debug: DMA '%s' Virtual: %p Timeout Count: 0x%x Write FIFO Depth: %d\n",
                         ptrSOCInitDMABlock->name, ptrCPDMAGlobalConfig->gblCfgRegs, ptrSOCInitDMABlock->timeoutCount,
                         ptrSOCInitDMABlock->writeFifoDepth);
        }
        else
        {
            /* No. There is no configuration space specified for the CPDMA. Fall through! */
        }

        /* Cleanup the allocated memory */
        free (ptrSOCInitDMABlock);

        /* Get the next SOC DMA Block: */
        ptrSOCInitDMABlock = (SOCInit_DMABlock*)SOCInit_listRemove ((SOCInit_ListNode**)&gSocInitMCB.ptrDMABlockList);
    }

    /* Cycle through all the registered Peripheral blocks: cleanup*/
    ptrSOCInitPeripheralBlock = (SOCInit_PeripheralBlock*)SOCInit_listRemove ((SOCInit_ListNode**)&gSocInitMCB.ptrPeripheralBlockList);
    while (ptrSOCInitPeripheralBlock != NULL)
    {
        /* Cleanup the allocated memory */
        free (ptrSOCInitPeripheralBlock);

        /* Get the next SOC Peripheral Block: */
        ptrSOCInitPeripheralBlock = (SOCInit_PeripheralBlock*)SOCInit_listRemove ((SOCInit_ListNode**)&gSocInitMCB.ptrPeripheralBlockList);
    }

    /* Cycle through all the registered BCP blocks: cleanup*/
    ptrSOCInitBCPBlock = (SOCInit_BCPBlock*)SOCInit_listRemove ((SOCInit_ListNode**)&gSocInitMCB.ptrBCPBlockList);
    while (ptrSOCInitBCPBlock != NULL)
    {
        /* Cleanup the allocated memory */
        free (ptrSOCInitBCPBlock);

        /* Get the next SOC Peripheral Block: */
        ptrSOCInitBCPBlock = (SOCInit_BCPBlock*)SOCInit_listRemove ((SOCInit_ListNode**)&gSocInitMCB.ptrBCPBlockList);
    }

    /* Cycle through all the registered DDR blocks: cleanup*/
    ptrSOCInitDDRBlock = (SOCInit_DDRBlock*)SOCInit_listRemove ((SOCInit_ListNode**)&gSocInitMCB.ptrDDRBlockList);
    while (ptrSOCInitDDRBlock != NULL)
    {
        /* Cleanup the allocated memory */
        free (ptrSOCInitDDRBlock);

        /* Get the next SOC Peripheral Block: */
        ptrSOCInitDDRBlock = (SOCInit_DDRBlock*)SOCInit_listRemove ((SOCInit_ListNode**)&gSocInitMCB.ptrDDRBlockList);
    }

    /* Debug Message: */
    SOCInit_log (SOCInit_LogLevel_DEBUG, "Debug: SOC Initialized successfully\n");
    return 0;
}
