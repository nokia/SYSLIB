/**
 *   @file  queueDump.c
 *
 *   @brief
 *      This is a a generic utility which can be used to dump the contents of
 *      a queue. The descriptors from the queue are just popped off and are 
 *      discarded. This is used as a purely debug utility. 
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
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>

/* BIOS/XDC Include Files. */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Memory.h> 
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <ti/sysbios/family/c64p/Hwi.h>

/* MCSDK Include Files */
#include <ti/csl/csl_chip.h>
#include <ti/csl/csl_cacheAux.h>
#include <ti/csl/csl_cacheAux.h>
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/drv/rm/rm.h>

/* SYSLIB Include files */
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/resmgr/resmgr.h>

/* For Debugging only. */
#ifdef __ARMv7
#define System_printf   printf
#else
#include <xdc/runtime/System.h>
#endif

/*********************************************************************
 * OSAL Callout Functions:
 *********************************************************************/

/* RESMGR: */
extern void* Resmgr_osalMalloc(Resmgr_MallocMode , uint32_t );
extern void  Resmgr_osalFree(Resmgr_MallocMode , void* , uint32_t );
extern void* Resmgr_osalMallocMemoryRegion(char*, Resmgr_MemRegionType , uint32_t );
extern void  Resmgr_osalFreeMemoryRegion(char*, Resmgr_MemRegionType , void* , uint32_t );
extern void* Resmgr_osalCreateSem (void);
extern void  Resmgr_osalDeleteSem (void*);
extern void  Resmgr_osalPostSem (void*);
extern void  Resmgr_osalPendSem (void*);
extern void  Resmgr_osalBeginMemAccess (void*, uint32_t);
extern void  Resmgr_osalEndMemAccess (void*, uint32_t);

/**************************************************************************
 *********************** Queue Dump Functions *****************************
 **************************************************************************/

/**
 *  @b Description
 *  @n  
 *      Utility Function to dump memory on the console.
 *
 *  @param[in]  cp
 *      Pointer to the memory to be dumped
 *  @param[in]  length
 *      Length of the memory to be dumped
 *  @param[in]  prefix
 *      Prefix information to be printed.
 *
 *  @retval
 *      Not Applicable.
 */
void xdump(uint8_t*  cp, int  length, char*  prefix )
{
    int32_t col, count;
    char prntBuf[120];
    char*  pBuf = prntBuf;
    count = 0;
    while(count < length){
        pBuf += sprintf( pBuf, "%s", prefix );
        for(col = 0;count + col < length && col < 16; col++){
            if (col != 0 && (col % 4) == 0)
                pBuf += sprintf( pBuf, " " );
            pBuf += sprintf( pBuf, "%02X ", cp[count + col] );
        }
        while(col++ < 16){      /* pad end of buffer with blanks */
            if ((col % 4) == 0)
                sprintf( pBuf, " " );
            pBuf += sprintf( pBuf, "   " );
        }
        pBuf += sprintf( pBuf, "  " );
        for(col = 0;count + col < length && col < 16; col++){
            if (isprint((int)cp[count + col]))
                pBuf += sprintf( pBuf, "%c", cp[count + col] );
            else
                pBuf += sprintf( pBuf, "." );
                }
        sprintf( pBuf, "\n" );
        // SPrint(prntBuf);
        System_printf(prntBuf);
        count += col;
        pBuf = prntBuf;
    }
}

/**
 *  @b Description
 *  @n  
 *      Entry point into the queue dump utility
 *
 *  @retval
 *      Not applicable
 */
void main (void)
{
    Resmgr_SysCfgHandle handleSysCfg;
    Qmss_QueueHnd       queueHandle;
    Ti_Pkt*             ptrPkt;
    uint8_t*            ptrDataBuffer;
    uint32_t            dataBufferLen;
    uint32_t            packetLen;
    uint32_t            numPackets;
    uint32_t            index;
    int32_t             errCode;
    Resmgr_SystemCfg    sysConfig;
    char                prefix[256];

    /* Display the warning message: Nobody likes to read code anymore */
    System_printf ("*********************************************************************************************\n");
    System_printf ("Warning: The utility will pop off descriptors from the queue and these packets are discarded.\n");
    System_printf ("This should only be used as a debug tool. \n");
    System_printf ("*********************************************************************************************\n");

    /* Initialize the system configuration. */
    memset ((void *)&sysConfig, 0, sizeof(Resmgr_SystemCfg));

    /* Populate the configuration.*/
    sysConfig.realm                             = Resmgr_ExecutionRealm_DSP;
    sysConfig.coreId                            = DNUM;
    strcpy (sysConfig.rmClient, "Rm_LTE9A_L2");
    strcpy (sysConfig.rmServer, "Rm_Server");
    sysConfig.malloc                            = Resmgr_osalMalloc;
    sysConfig.free                              = Resmgr_osalFree;
    sysConfig.mallocMemoryRegion                = Resmgr_osalMallocMemoryRegion;
    sysConfig.freeMemoryRegion                  = Resmgr_osalFreeMemoryRegion;
    sysConfig.createSem                         = Resmgr_osalCreateSem;
    sysConfig.pendSem                           = Resmgr_osalPendSem;
    sysConfig.postSem                           = Resmgr_osalPostSem;
    sysConfig.deleteSem                         = Resmgr_osalDeleteSem;
    sysConfig.beginMemAccess                    = Resmgr_osalBeginMemAccess;
    sysConfig.endMemAccess                      = Resmgr_osalEndMemAccess;
    sysConfig.dspSystemCfg.armCoreId        	= SYSLIB_ARM_CORE_ID;
    sysConfig.dspSystemCfg.sourceId         	= 17;
    sysConfig.dspSystemCfg.sharedMemAddress 	= DDR3_SYSLIB_RESMGR_RSVD;
    sysConfig.dspSystemCfg.sizeSharedMemory 	= DDR3_SYSLIB_RESMGR_RSVD_LEN;

    /* Initialize the system configuration. */ 
    handleSysCfg = Resmgr_init(&sysConfig, &errCode);
    if (handleSysCfg == NULL)
	{
	    System_printf ("Error: SYSRM initialization failed with error code %d\n", errCode);
	    return;
    }
    System_printf ("Debug: SYSRM initialized successfully [Handle %p]\n", handleSysCfg);

    /* Request the user for a queue identifier. */
    System_printf ("Enter the queue Id: ");
    scanf ("%d", &queueHandle);

    /* Pop off the packets from the queue */
    ptrPkt = (Ti_Pkt*)QMSS_DESC_PTR((uint32_t)Qmss_queuePop(queueHandle));
    while (ptrPkt != NULL)
    {
        packetLen  = Pktlib_getPacketLen(ptrPkt);
        numPackets = Pktlib_packetBufferCount(ptrPkt);

        /* Display the header: */
        System_printf ("*********************************************************************************************\n");
        System_printf ("Debug: Descriptor 0x%x Packet Length %d bytes %d chained packets SwInfo0: 0x%x\n", 
                        ptrPkt, packetLen, numPackets, Cppi_getSoftwareInfo0(Cppi_DescType_HOST, (Cppi_Desc*)ptrPkt));

        /* Display each data buffer: */
        for (index = 0; index < numPackets; index++)
        {
            /* Get the data buffer & length */
            Pktlib_getDataBuffer (ptrPkt, &ptrDataBuffer, &dataBufferLen);
            snprintf (prefix, 256, "0x%x: ", ptrDataBuffer);

            /* Display the data buffer: Just the first 64 bytes should be good */
            xdump(ptrDataBuffer, (dataBufferLen < 64) ? dataBufferLen: 64, &prefix[0]);
            System_flush();

            /* Get the next packet in the chain */
            ptrPkt = Pktlib_getNextPacket(ptrPkt);
        }

        /* Pop off the next packet from the queue */
        ptrPkt = (Ti_Pkt*)QMSS_DESC_PTR((uint32_t)Qmss_queuePop(queueHandle));
    }
    System_printf ("Debug: Queue has been drained out\n");
}

