/**
 *   @file  dat_uio.c
 *
 *   @brief
 *      The file implements the DAT uio functions to access memorys
 *   through uio.
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

#ifdef __ARMv7
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
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#define System_printf   printf

/* SYSLIB Include files */
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/msgcom/msgcom.h>
#include <ti/runtime/dat/dat.h>
#include <ti/runtime/dat/include/dat_internal.h>

/**********************************************************************
 *********************** DAT UIO Internal Structure *******************
 **********************************************************************/

Dat_uioMap*  gUioMapInfo;

/**********************************************************************
 *********************** DAT UIO Functions *************************
 **********************************************************************/
static int endian_check() {
    int check = 1;
    char ret = *(char *) &check;
    return ret;
}

static int byte_swap(int x)
{
    return ( ((x>>0) & 0xff) << 24)
         | ( ((x>>8) & 0xff) << 16)
         | ( ((x>>16) & 0xff) << 8)
         | ( ((x>>24) & 0xff) << 0);
}


static int uioutil_get_ints(char *file_name, int *buf, int size)
{
    FILE *fp;
    int i;

    printf("Debug: open file %s to get integer number \n", file_name);

    fp = fopen(file_name, "rb");
    if (!fp)
    {
        //printf("Error opening file %s (%s)", file_name, strerror(errno));
        return -1;
    }
    printf("Open file is successful\n");

    if (!fread(buf, sizeof(int), size, fp))
    {
        //printf("Error reading file %s (%s)", file_name, strerror(errno));
        fclose(fp);
        return -1;
    }
    printf("Reading an interger is successful\n");

    if (endian_check())
    {
        for (i = 0; i < size; i++)
            buf[i] = byte_swap(buf[i]);

        printf("Done with Data swamp for endian difference\n");
    }
    fclose(fp);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to initialize the UIO memory map
 *
 *  @param[in]  clientHandle
 *      Handle to the DAT client which is associate with the request
 *  @param[in]  name
 *      Name of the uio device
 *  @param[out] errCode
 *      Error code populated by the API
 *
 * \ingroup DAT_FUNCTION
 *
 * @sa Dat_uio_initMemMap
 *
 *  @retval
 *      >0   -  Handle to  Dat_uioMap structure
 *  @retval
 *      NULL  - Error
 */
Dat_uioMap* Dat_uioInitMemMap(Dat_ClientHandle clientHandle, char* name, int32_t *errCode)
{
    int               fd;
    char              device_name[64];
    char              file_name[128];
    int32_t           memBase;
    uint32_t          freq[2];
    Dat_uioMap*       ptrUioMapInst;
    Dat_ClientMCB*    ptrDatClient;

    /* Open the UIO emucnt device */
    snprintf(device_name, 64, "/dev/%s", name);
    fd = open (device_name, (O_RDWR | O_SYNC));
    if (fd < 0)
    {
        printf("\n Error opening device %s", device_name);
        *errCode = DAT_EINVAL;
        return NULL;
    }

    snprintf(file_name, 128, "/proc/device-tree/soc/%s/mem", "emucnt");
    if (uioutil_get_ints(file_name, &memBase, 1) < 0)
    {
        /* Error getting the base address for the memory region */
        *errCode = DAT_EINVAL;
        close (fd);
        return NULL;
    }

    /* Allocate memory to store the uio map info */
    ptrDatClient = (Dat_ClientMCB*)clientHandle;
    ptrUioMapInst = ptrDatClient->cfg.malloc(sizeof(Dat_uioMap), 0);
    ptrUioMapInst->fd = fd;
    ptrUioMapInst->memMapBase = memBase;

    /* Save EMU counter clock frequency */
    ptrUioMapInst->freqHi    = freq[0];
    ptrUioMapInst->freqLo    = freq[1];

    printf("DAT: emuCnt base address: 0x%x, freq=0x%x:0x%x\n", memBase, ptrUioMapInst->freqHi, ptrUioMapInst->freqLo);

    /* Temporay change to save the info in a global structure */
    gUioMapInfo = ptrUioMapInst;

    return ptrUioMapInst;
}

/**
 *  @b Description
 *  @n
 *      The function is used to initialize the UIO memory map
 *
 *  @param[in]  clientHandle
 *      Handle to the DAT client which is associate with the request
 *  @param[in]  ptrUioMapInst,
 *      UIO map handle for emu counter
 *  @param[out] errCode
 *      Error code populated by the API
 *
 * \ingroup DAT_FUNCTION
 *
 * @sa Dat_uio_initMemMap
 *
 *  @retval
 *      Successful       - 0
 *  @retval
 *      Failed           < 0
 */
int32_t Dat_UioDeinitMemMap(Dat_ClientHandle clientHandle, Dat_uioMap* ptrUioMapInst, int32_t *errCode)
{
    Dat_ClientMCB*    ptrDatClient;

    /* Get DAT Client MCB */
    ptrDatClient    = (Dat_ClientMCB*)clientHandle;

    if(ptrUioMapInst == NULL )
    {
        *errCode = DAT_EINVAL;
        return -1;
    }

    /* Close devive file */
    if(ptrUioMapInst->fd)
        close (ptrUioMapInst->fd);

    /* Free memory */
    ptrDatClient->cfg.free(ptrUioMapInst, sizeof(Dat_uioMap));

    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to read registers from UIO mapped region
 *
 *  @param[in]  ptrDatUioMap
 *      Pointer to DAT UIO map structure
 *  @param[in]  offset
 *      Offset of the register from the UIO memory base address
 *
 * \ingroup DAT_FUNCTION
 *
 * @sa Dat_uio_read32
 *
 *  @retval
 *      32bit data read from the UIO mapped memory
 */
uint32_t Dat_uio_read32(Dat_uioMap* ptrDatUioMap, uint32_t offset)
{
    uint32_t data;

    pread(ptrDatUioMap->fd, &data, sizeof(uint32_t), (off_t)(offset + ptrDatUioMap->memMapBase) );
    return data;
}

/**
 *  @b Description
 *  @n
 *      The function is used to write data to registers mapped by UIO
 *
 *  @param[in]  ptrDatUioMap
 *      Pointer to DAT UIO map structure
 *  @param[in]  offset
 *      Offset of the register from the UIO memory base address
 *  @param[in]  data
 *      32bit data to be written
 *
 * \ingroup DAT_FUNCTION
 *
 * @sa Dat_uio_write32
 *
 *  @retval
 *      32bit data read from the UIO mapped memory
 */
void Dat_uio_write32(Dat_uioMap* ptrDatUioMap, uint32_t offset, uint32_t *data)
{

    pwrite(ptrDatUioMap->fd, data, sizeof(uint32_t), (off_t)(offset + ptrDatUioMap->memMapBase) );
}

#endif

