/**
 *   @file  root_arm.c
 *
 *   @brief
 *      The file implements the root domain functionality which allows
 *      the ARM to start application specific domains on the ARM.
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
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <limits.h>
#include <errno.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <getopt.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

/* ROOT Include Files. */
#include <ti/runtime/root/root.h>
#include <ti/runtime/root/include/root_internal.h>

/**************************************************************************
 ************************* Root ARM Functions *****************************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      The function is used to map the physical address to a virtual address
 *
 *  @param[in]   phyAddr
 *      Physical address
 *  @param[in]   size
 *      Size of the physical address which is to be mapped
 *
 *  \ingroup ROOT_ARM_FUNCTIONS
 *
 *  @retval
 *      Success -   Mapped virtual address
 *  @retval
 *      Error   -   0
 */
uint32_t Root_mapPhyAddrToVirtual (uint32_t phyAddr, uint32_t size)
{
    void*           mmapBaseAddress;
    int32_t         retVal;
    uint32_t        mask;
    uint32_t        mmapFD;
    uint32_t        virtualAddress;

    /* Ovewriting size to be 4K */
    size = 4*1024;

    /* Minimum page size for the mmapped region. */
    mask = size - 1;

    /* Open the file that will be mapped. */
    mmapFD = open("/dev/mem", (O_RDWR | O_SYNC));
    if (mmapFD== -1)
    {
        perror ("Error: Unable to open the /dev/mem file for address mapping\n");
        return 0;
    }

    /* Get the page size. */
    retVal = sysconf(_SC_PAGE_SIZE);
    if (retVal < 0)
        return 0;

    /* Create a mapping of the physical address. */
    mmapBaseAddress = mmap(0, size, (PROT_READ|PROT_WRITE), MAP_SHARED, mmapFD, (off_t)phyAddr & ~mask);
    if(mmapBaseAddress == MAP_FAILED)
    {
        perror ("Error: Memory map failed\n");
        return 0;
    }
    virtualAddress = (uint32_t)(mmapBaseAddress + ((off_t)phyAddr & mask));

    /* Close the file */
    close (mmapFD);

    /* Return the mapped virtual address */
    return virtualAddress;
}

/**
 *  @b Description
 *  @n
 *      The function is used to generate an interrupt to the peer domain
 *      to indicate that a JOSH message has been populated and should be
 *      processed.
 *
 *  @param[in]  ptrMasterSlaveMCB
 *      Pointer to the master/slave MCB
 *
 *  \ingroup ROOT_ARM_FUNCTIONS
 *
 *  @retval
 *      Not applicable
 */
static void Root_generateIPCInterrupt (Root_MasterSlaveMCB* ptrMasterSlaveMCB)
{
    /* Generate the interrupt. */
    ptrMasterSlaveMCB->ptrBootCfgRegs->IPCGR[ptrMasterSlaveMCB->peerCoreId] =
            CSL_FMKR(ptrMasterSlaveMCB->ipcSrcId + 4, ptrMasterSlaveMCB->ipcSrcId + 4, 1) |
            CSL_FMK (BOOTCFG_IPCGR0_IPCGR0_REG, 1);
}

/**
 *  @b Description
 *  @n
 *      The function is used to initialize the socket services which are available to the
 *      root slave to communicate with the root master.
 *
 *  @param[in]  ptrMasterSlaveMCB
 *      Pointer to the master/slave MCB
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup ROOT_ARM_FUNCTIONS
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0
 */
int32_t Root_slaveInitializeSocket (Root_MasterSlaveMCB* ptrMasterSlaveMCB, int32_t* errCode)
{
    struct sockaddr_un 	    socketAddress;

    /* Create the root slave socket. */
    ptrMasterSlaveMCB->masterSlaveSocket = socket(AF_UNIX, SOCK_DGRAM, 0);
	if (ptrMasterSlaveMCB->masterSlaveSocket < 0)
    {
        /* Error: Root Master socket creation failed. */
    	perror("Error: Root Master socket creation Failed");
        *errCode = errno;
        return -1;
	}

    /* Initialize the socket information. */
    memset(&socketAddress, 0, sizeof(socketAddress));

    /* Map the client identifer to a unix socket address */
    socketAddress.sun_family = AF_UNIX;
    snprintf (socketAddress.sun_path, sizeof(socketAddress.sun_path), "%s/%s-%x",
              Syslib_getRunTimeDirectory(), ptrMasterSlaveMCB->rootMasterName, ptrMasterSlaveMCB->slaveCoreId);

    printf ("Debug: Creating the root slave socket for %s [%d]\n", socketAddress.sun_path, ptrMasterSlaveMCB->masterSlaveSocket);

    /* Unlink any previous client instances: */
    unlink (socketAddress.sun_path);

	/* Bind the client socket. */
    *errCode = bind(ptrMasterSlaveMCB->masterSlaveSocket, (struct sockaddr *)&socketAddress, SUN_LEN(&socketAddress));
    if (*errCode < 0)
    {
		/* Error: Unable to bind the socket. */
    	perror("Error: Root master socket BIND failed\n");
        *errCode = errno;
        return -1;
	}
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to determine if the slave buffer has valid data
 *      or not.
 *
 *  @param[in]  ptrMasterSlaveMCB
 *      Pointer to the master/slave MCB
 *
 *  \ingroup ROOT_ARM_FUNCTIONS
 *
 *  @retval
 *      1   -   Root slave has valid data
 *  @retval
 *      0   -   Root slave does not have data
 */
int32_t Root_isSlaveBufferReady (Root_MasterSlaveMCB* ptrMasterSlaveMCB)
{
    uint8_t buffer;

    /* On the ARM we check if the socket created has received a notification. The "read" call
     * will block the root slave till a valid request is received. */
    if (read(ptrMasterSlaveMCB->masterSlaveSocket, &buffer, sizeof(uint8_t)) != sizeof(uint8_t))
    {
        printf ("Error: Invalid read on slave buffer status\n");
        return 0;
    }

    /* Valid data was received. */
    return 1;
}

/**
 *  @b Description
 *  @n
 *      The function is used to notify the root slave from the root master to
 *      indicate that the JOSH buffer has been populated and is available.
 *
 *  @param[in]  ptrMasterSlaveMCB
 *      Pointer to the master/slave MCB
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup ROOT_ARM_FUNCTIONS
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0
 */
int32_t Root_notifySlave (Root_MasterSlaveMCB* ptrMasterSlaveMCB, int32_t* errCode)
{
    struct sockaddr_un  clientaddr;
    char                buffer;

    /* Is the slave executing on the DSP? */
    if (ptrMasterSlaveMCB->realm == Root_ExecutionRealm_DSP)
    {
        /* YES. Master on ARM will always send an IPC interrupt to the DSP slaves */
        Root_generateIPCInterrupt (ptrMasterSlaveMCB);
        return 0;
    }

    /* Slave is executing on the DSP so we send a message to the UNIX socket. */
    memset(&clientaddr, 0, sizeof(clientaddr));

    /* Map the client identifer to the unix socket address */
    clientaddr.sun_family = AF_UNIX;
    snprintf (clientaddr.sun_path, sizeof(clientaddr.sun_path), "%s/%s-%x",
              Syslib_getRunTimeDirectory(), ptrMasterSlaveMCB->rootMasterName, ptrMasterSlaveMCB->peerCoreId);

    /* Populate the notification message: */
    buffer = 0xAA;

    /* Send the packet to the remote slave */
    if (sendto((int32_t)ptrMasterSlaveMCB->masterSlaveSocket, &buffer, 1, 0,
               (struct sockaddr *)&clientaddr, sizeof(clientaddr)) < 0)
    {
        perror("Error: Unable to send back the response to the root slave\n");
        *errCode = errno;
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to notify the root master from the root slave to
 *      indicate that the JOSH buffer has been populated and is available.
 *
 *  @param[in]  ptrMasterSlaveMCB
 *      Pointer to the master/slave MCB
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup ROOT_ARM_FUNCTIONS
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0
 */
int32_t Root_notifyMaster (Root_MasterSlaveMCB* ptrMasterSlaveMCB, int32_t* errCode)
{
    /* Master execute on the ARM and are always notified by the IPC interrupt. */
    Root_generateIPCInterrupt (ptrMasterSlaveMCB);
    return 0;
}

