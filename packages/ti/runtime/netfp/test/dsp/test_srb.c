/*
 *   @file  test_srb.c
 *
 *   @brief
 *      Unit Test code for testing SRB ciphering & integrity protection
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
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>

/* BIOS/XDC Include Files. */
#include <xdc/std.h>
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

/* IPC Include Files for Shared Memory Allocation. */
#include <ti/ipc/Ipc.h>
#include <ti/ipc/SharedRegion.h>

/* MCSDK Include Files. */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/csl/csl_chip.h>
#include <ti/csl/csl_cacheAux.h>

/* SYSLIB Include Files.  */
#include <ti/runtime/msgcom/msgcom.h>
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/netfp/netfp.h>

/* SNOW3G Files */
#include <ti/snow3g/f8.h>
#include <ti/snow3g/f9.h>

/* Test Vectors */
#include "testVectors.h"

/**********************************************************************
 ************************ Unit Test Definitions ***********************
 **********************************************************************/

/* Test User Identifier. */
#define TEST_UE_ID                  10

/* Definition which controls the generation of the output of the test vectors. */
#undef GENERATE_TEST_VECTOR

#ifndef _LITTLE_ENDIAN

/**
 * @brief   Macro which converts 16 bit data from host to network order.
 */
#define  htons(a) (a)

/**
 * @brief   Macro which converts 32 bit data from host to network order.
 */
#define  htonl(a) (a)

/**
 * @brief   Macro which converts 32 bit data from network to host order.
 */
#define  ntohl(a) (a)

/**
 * @brief   Macro which converts 16 bit data from network to host order.
 */
#define  ntohs(a) (a)

#else

/**
 * @brief   Macro which converts 16 bit data from host to network order.
 */
#define  htons(a)    ( (((a)>>8)&0xff) + (((a)<<8)&0xff00) )

/**
 * @brief   Macro which converts 32 bit data from host to network order.
 */
#define  htonl(a)    ( (((a)>>24)&0xff) + (((a)>>8)&0xff00) + \
                       (((a)<<8)&0xff0000) + (((a)<<24)&0xff000000) )

/**
 * @brief   Macro which converts 32 bit data from network to host order.
 */
#define  ntohl(a)   htonl(a)

/**
 * @brief   Macro which converts 16 bit data from network to host order.
 */
#define  ntohs(a)   htons(a)

#endif /* _LITTLE_ENDIAN */

/**********************************************************************
 ************************** Unit Test Extern  *************************
 **********************************************************************/

/* NETFP Client Handle: */
extern Netfp_ClientHandle       netfpClientHandle;

/* Global MSGCOM Instance handle.*/
extern Msgcom_InstHandle        appMsgcomInstanceHandle;

/* PKTLIB Instance Handle. */
extern Pktlib_InstHandle        appPktlibInstanceHandle;

/* Global application requested resource configuration */
extern Resmgr_ResourceCfg       appResourceConfig;

/* Cache operations: */
extern void appWritebackBuffer(void* ptr, uint32_t size);
extern void appInvalidateBuffer(void* ptr, uint32_t size);

/**********************************************************************
 ************************** Unit Test Functions ***********************
 **********************************************************************/

/**
 *  @b Description
 *  @n
 *      The function is used to get the authentication string
 *
 *  @param[in]  authMode
 *      Authentication mode
 *
 *  @retval
 *      String name associated with the authentication mode
 */
static char* Test_getAuthenticationString (Netfp_3gppAuthMode authMode)
{
    if (authMode == Netfp_3gppAuthMode_EIA0)
        return "NULL";
    if (authMode == Netfp_3gppAuthMode_EIA1)
        return "SNOW3G";
    if (authMode == Netfp_3gppAuthMode_EIA2)
        return "AES";
    return "Unsupported";
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the ciphering string
 *
 *  @param[in]  cipherMode
 *      Ciphering mode
 *
 *  @retval
 *      String name associated with the ciphering mode
 */
static char* Test_getCipheringString (Netfp_3gppCipherMode cipherMode)
{
    if (cipherMode == Netfp_3gppCipherMode_EEA0)
        return "NULL";
    if (cipherMode == Netfp_3gppCipherMode_EEA1)
        return "SNOW3G";
    if (cipherMode == Netfp_3gppCipherMode_EEA2)
        return "AES";
    return "Unsupported";
}

/**
 *  @b Description
 *  @n
 *      The function is used to populate the user security configuration.
 *
 *  @param[in]  encodeQueueHandle
 *      SRB Encoded Queue handle where encoded SRB packet will arrive
 *  @param[in]  decodeQueueHandle
 *      SRB Decoded Queue handle where decoded SRB packet will arrive
 *  @param[in]  srbFlowHandle
 *      SRB Flow Handle to be used
 *  @param[in]  srbAuthMode
 *      SRB Authentication mode.
 *  @param[in]  srbCipherMode
 *      SRB Ciphering mode
 *  @param[in]  integrityKey
 *      Integrity Protection Key
 *  @param[in]  encyrptionKey
 *      Encryption Key.
 *  @param[out] ptrUserCfg
 *      Pointer to the populated user security configuration.
 *
 *  @retval
 *      Not Applicable
 */
static void Test_populateUserSecurityCfg
(
    Qmss_QueueHnd           encodeQueueHandle,
    Qmss_QueueHnd           decodeQueueHandle,
    int32_t                 srbFlowId,
    Netfp_3gppAuthMode      srbAuthMode,
    Netfp_3gppCipherMode    srbCipherMode,
    uint8_t                 integrityKey[],
    uint8_t                 encyrptionKey[],
    Netfp_UserCfg*          ptrUserCfg
)
{
    /* Initialize the user security configuration. */
    memset ((void *)ptrUserCfg, 0, sizeof(Netfp_UserCfg));

    /* Populate the user security configuration. */
    ptrUserCfg->authMode         = srbAuthMode;
    ptrUserCfg->srbCipherMode    = srbCipherMode;
    ptrUserCfg->drbCipherMode    = Netfp_3gppCipherMode_EEA0;
    ptrUserCfg->ueId             = TEST_UE_ID;
    ptrUserCfg->srbFlowId        = srbFlowId;
    ptrUserCfg->initialCountC    = 0;
    ptrUserCfg->chSrb1Enc        = encodeQueueHandle;
    ptrUserCfg->chSrb1Dec        = decodeQueueHandle;
    ptrUserCfg->chSrb2Enc        = encodeQueueHandle;
    ptrUserCfg->chSrb2Dec        = decodeQueueHandle;
    memcpy ((void *)&ptrUserCfg->hKeyRrcInt[0], (void *)integrityKey, sizeof(ptrUserCfg->hKeyRrcInt));
    memcpy ((void *)&ptrUserCfg->hKeyRrcEnc[0], (void *)encyrptionKey, sizeof(ptrUserCfg->hKeyRrcEnc));
    memcpy ((void *)&ptrUserCfg->hKeyUpEnc[0], (void *)encyrptionKey, sizeof(ptrUserCfg->hKeyUpEnc));
    return;
}

#ifdef GENERATE_TEST_VECTOR

/**
 *  @b Description
 *  @n
 *      The function is used to dump the hex on the console
 *
 *  @param[in]  cp
 *      Pointer to the memory to be dumped
 *  @param[in]  length
 *      Length of the memory to be dumped
 *  @param[in]  prefix
 *      Prefix information to be printed.
 *
 *  @retval
 *      Not Applicable
 */
static void Test_dumpHex (uint8_t*  cp, int32_t length, char* prefix)
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
            pBuf += sprintf( pBuf, "0x%02X,", cp[count + col] );
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
        System_printf (prntBuf);
        count += col;
        pBuf = prntBuf;
    }
    System_flush();
}

/**
 *  @b Description
 *  @n
 *      The function is used to generate DL test vectors.
 *
 *  @param[in]  encodeChannel
 *      SRB Encoded channel handle where encoded SRB packet will arrive
 *  @param[in]  decodeChannel
 *      SRB Decoded channel handle where decoded SRB packet will arrive
 *  @param[in]  srbFlowHandle
 *      SRB Flow Handle to be used.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Test_generateDLTestVector
(
    MsgCom_ChHandle         encodeChannel,
    MsgCom_ChHandle         decodeChannel,
    int32_t                 srbFlowId
)
{
    int32_t                 index;
    int32_t                 errCode;
    Netfp_UserCfg           userCfg;
    Netfp_UserHandle        ueHandle;
    Pktlib_HeapHandle       heapHandle;
    Ti_Pkt*                 ptrTestPacket;
    uint32_t                messageLength;
    uint8_t*                ptrTestPacketBuffer;

    /* Get the NETFP Data Receive Heap */
    heapHandle = Pktlib_findHeapByName ("Core1_MTU_ReceiveHeap");
    if (heapHandle == NULL)
    {
        System_printf ("Error: Unable to find the NETFP Packet Receive Heap\n");
        return -1;
    }
    System_flush();

    /* Cycle through and execute all the tests. */
    for (index = 0; index < DOWNLINK_TEST_SETS; index++)
    {
        /* Debug Message: */
        System_printf ("-------------------------------------------------------------------------------\n");
        System_printf ("Debug: Generating DL Test Vectors %d for Authentication: %s Ciphering: %s\n",
                       (index + 1), Test_getAuthenticationString(downLinkTestVectors[index].authMode),
                       Test_getCipheringString(downLinkTestVectors[index].cipherMode));

        /* Populate the user security configuration. */
        Test_populateUserSecurityCfg (Msgcom_getInternalMsgQueueInfo(encodeChannel),
                                      Msgcom_getInternalMsgQueueInfo(decodeChannel),
                                      srbFlowId,
                                      downLinkTestVectors[index].authMode,
                                      downLinkTestVectors[index].cipherMode,
                                      downLinkTestVectors[index].integrityKey,
                                      downLinkTestVectors[index].encryptionKey,
                                      &userCfg);

        /* Create a User Context */
        ueHandle = Netfp_createUser (netfpClientHandle, &userCfg, &errCode);
        if (ueHandle == NULL)
        {
            /* User Security Context creation failed. */
            System_printf ("Error: LTE Creation User Security Context failed [Error code %d]\n", errCode);
            return -1;
        }

        /* Allocate memory for the packet:  */
        ptrTestPacket = Pktlib_allocPacket (appPktlibInstanceHandle, heapHandle, downLinkTestVectors[index].length);
        if (ptrTestPacket == NULL)
        {
            /* Error: Out of memory */
            System_printf ("Error: Packet allocation failed\n");
            return -1;
        }

        /* Get the data buffer associated with the allocated packet. */
        Pktlib_getDataBuffer (ptrTestPacket, &ptrTestPacketBuffer, &messageLength);

        /* Copy the data from the downlink test vector. */
        memcpy ((void *)ptrTestPacketBuffer, (void *)downLinkTestVectors[index].plainMessage, messageLength);

        /* CACHE operations: */
        appWritebackBuffer(ptrTestPacketBuffer, messageLength);

        /* We now pass the packet for encoding. */
        if (Netfp_encodeSrb (ueHandle, ptrTestPacket, downLinkTestVectors[index].count, downLinkTestVectors[index].bearerId) < 0)
        {
            System_printf ("Error: Encoding of SRB failed\n");
            return -1;
        }

        /* Once encoded the packet; will be pushed into the encoded channel */
        Msgcom_getMessage (encodeChannel, (MsgCom_Buffer**)&ptrTestPacket);

        /* Get the data buffer */
        Pktlib_getDataBuffer (ptrTestPacket, &ptrTestPacketBuffer, &messageLength);

        /* CACHE Operation: */
        appInvalidateBuffer(ptrTestPacketBuffer, messageLength);

        /* Generate the HEX Dump. */
        Test_dumpHex (ptrTestPacketBuffer, messageLength, "Encoded Message: ");

        /* Cleanup the packet */
        Pktlib_freePacket(appPktlibInstanceHandle, ptrTestPacket);

        /* Delete the user security context once we are done with the test */
        if (Netfp_deleteUserSecContext(ueHandle) < 0)
        {
            System_printf ("Error: LTE Deletion User Security Context failed\n");
            return -1;
        }
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to generate UL test vectors.
 *
 *  @param[in]  encodeChannel
 *      SRB Encoded channel handle where encoded SRB packet will arrive
 *  @param[in]  decodeChannel
 *      SRB Decoded channel handle where decoded SRB packet will arrive
 *  @param[in]  srbFlowHandle
 *      SRB Flow Handle to be used.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Test_generateULTestVector
(
    MsgCom_ChHandle         encodeChannel,
    MsgCom_ChHandle         decodeChannel,
    int32_t                 srbFlowId
)
{
    int32_t                 index;
    int32_t                 errCode;
    Netfp_UserCfg           userCfg;
    Netfp_UserHandle        ueHandle;
    Pktlib_HeapHandle       heapHandle;
    Ti_Pkt*                 ptrTestPacket;
    uint32_t                messageLength;
    uint8_t*                ptrTestPacketBuffer;
    uint32_t                packetError;

    /* Get the NETFP Data Receive Heap */
    heapHandle = Pktlib_findHeapByName ("Core1_MTU_ReceiveHeap");
    if (heapHandle == NULL)
    {
        System_printf ("Error: Unable to find the NETFP Packet Receive Heap\n");
        return -1;
    }

    /* Cycle through and execute all the tests. */
    for (index = 0; index < UPLINK_TEST_SETS; index++)
    {
        /* Debug Message: */
        System_printf ("-------------------------------------------------------------------------------\n");
        System_printf ("Debug: Generating UL Test Vectors %d for Authentication: %s Ciphering: %s\n",
                       (index + 1), Test_getAuthenticationString(upLinkTestVectors[index].authMode),
                       Test_getCipheringString(upLinkTestVectors[index].cipherMode));

        /* Populate the user security configuration. */
        Test_populateUserSecurityCfg (Msgcom_getInternalMsgQueueInfo(encodeChannel),
                                      Msgcom_getInternalMsgQueueInfo(decodeChannel),
                                      srbFlowId,
                                      upLinkTestVectors[index].authMode,
                                      upLinkTestVectors[index].cipherMode,
                                      upLinkTestVectors[index].integrityKey,
                                      upLinkTestVectors[index].encryptionKey,
                                      &userCfg);

        /* Create a User Context */
        ueHandle = Netfp_createUser (netfpClientHandle, &userCfg, &errCode);
        if (ueHandle == NULL)
        {
            /* User Security Context creation failed. */
            System_printf ("Error: LTE Creation User Security Context failed [Error code %d]\n", errCode);
            return -1;
        }

        /* Allocate memory for the packet:  */
        ptrTestPacket = Pktlib_allocPacket (appPktlibInstanceHandle, heapHandle, upLinkTestVectors[index].length);
        if (ptrTestPacket == NULL)
        {
            /* Error: Out of memory */
            System_printf ("Error: Packet allocation failed\n");
            return -1;
        }

        /* Get the data buffer associated with the allocated packet. */
        Pktlib_getDataBuffer (ptrTestPacket, &ptrTestPacketBuffer, &messageLength);

        /* Copy the data from the uplink test vector. */
        memcpy ((void *)ptrTestPacketBuffer, (void *)upLinkTestVectors[index].cipheredMessage, messageLength);

        /* CACHE operations: */
        appWritebackBuffer(ptrTestPacketBuffer, messageLength);

        /* We now pass the packet for decoding */
        if (Netfp_decodeSrb (ueHandle, ptrTestPacket, upLinkTestVectors[index].count, upLinkTestVectors[index].bearerId) < 0)
        {
            System_printf ("Error: Decoding of SRB failed\n");
            return -1;
        }

        /* Once decoded the packet; will be pushed into the decoded channel */
        Msgcom_getMessage (decodeChannel, (MsgCom_Buffer**)&ptrTestPacket);

        /* Get the data buffer */
        Pktlib_getDataBuffer (ptrTestPacket, &ptrTestPacketBuffer, &messageLength);

        /* CACHE Operation: */
        appInvalidateBuffer(ptrTestPacketBuffer, messageLength);

        /* Ensure that the packet was correctly decoded: */
        packetError = Netfp_getPacketError(ptrTestPacket);
        if (packetError != 0)
        {
            System_printf ("Error: Packet decoding failed %d\n", packetError);
        }
        else
        {
            /* Generate the HEX Dump. */
            Test_dumpHex (ptrTestPacketBuffer, messageLength, "Decoded Message: ");
        }

        /* Cleanup the packet */
        Pktlib_freePacket(appPktlibInstanceHandle, ptrTestPacket);

        /* Delete the user security context once we are done with the test */
        if (Netfp_deleteUserSecContext(ueHandle) < 0)
        {
            System_printf ("Error: LTE Deletion User Security Context failed\n");
            return -1;
        }
    }
    return 0;
}

#endif /* GENERATE_TEST_VECTOR */

/**
 *  @b Description
 *  @n
 *      The function is used to test the downlink test vectors
 *
 *  @param[in]  encodeChannel
 *      SRB Encoded channel handle where encoded SRB packet will arrive
 *  @param[in]  decodeChannel
 *      SRB Decoded channel handle where decoded SRB packet will arrive
 *  @param[in]  srbFlowHandle
 *      SRB Flow Handle to be used.
 *  @param[in]  isDataBufferAligned
 *      Set to 1 to indicate that the data buffer being passed to the encode/decode
 *      functions is aligned on the word boundary or not
 *  @param[in]  useGenerateMacI
 *      Set to 1 to indicate the generateMacI function is used instead of regular encodeSrb function
 *
 *  @retval
 *      Not Applicable
 */
static int32_t Test_downlinkVectors
(
    MsgCom_ChHandle         encodeChannel,
    MsgCom_ChHandle         decodeChannel,
    int32_t                 srbFlowId,
    uint8_t                 isDataBufferAligned,
    uint8_t                 useGenerateMacI
)
{
    int32_t                 index;
    int32_t                 errCode;
    Netfp_UserCfg           userCfg;
    Netfp_UserHandle        ueHandle;
    Pktlib_HeapHandle       heapHandle;
    Ti_Pkt*                 ptrTestPacket;
    uint32_t                messageLength;
    uint8_t*                ptrTestPacketBuffer;
    uint32_t                orgMessageLength;
    uint8_t*                ptrOrgTestPacketBuffer;
    uint32_t                validateIndex;
    uint16_t                ueId;
    uint8_t                 qci;
    uint8_t                 bearerId;
    uint8_t                 flag;

    /* Debug Message: */
    System_printf ("Debug: Downlink Test for the Encode API using %s with %s data buffers\n",
                    (useGenerateMacI == 1) ? "generateMacI": "encodeSrb", (isDataBufferAligned == 1) ? "aligned": "unaligned");

    /* Get the NETFP Data Receive Heap */
    heapHandle = Pktlib_findHeapByName (appPktlibInstanceHandle, "Core1_MTU_ReceiveHeap", &errCode);
    if (heapHandle == NULL)
    {
        System_printf ("Error: Unable to find the NETFP Packet Receive Heap [Error code %d]\n", errCode);
        return -1;
    }

    /* Cycle through all the test vectors:  */
    for (index = 0; index < DOWNLINK_TEST_SETS; index++)
    {
        if ((useGenerateMacI) && downLinkTestVectors[index].cipherMode != Netfp_3gppCipherMode_EEA0)
            continue;

        System_printf ("Debug: Downlink Test Vectors %d for Authentication: %s Ciphering: %s\n",
                       (index + 1), Test_getAuthenticationString(downLinkTestVectors[index].authMode),
                       Test_getCipheringString(downLinkTestVectors[index].cipherMode));

        /* Populate the user security configuration. */
        Test_populateUserSecurityCfg (Msgcom_getInternalMsgQueueInfo(encodeChannel),
                                      Msgcom_getInternalMsgQueueInfo(decodeChannel),
                                      srbFlowId,
                                      downLinkTestVectors[index].authMode,
                                      downLinkTestVectors[index].cipherMode,
                                      downLinkTestVectors[index].integrityKey,
                                      downLinkTestVectors[index].encryptionKey,
                                      &userCfg);

        /* Create a User Context */
        ueHandle = Netfp_createUser (netfpClientHandle, &userCfg, &errCode);
        if (ueHandle == NULL)
        {
            /* User Security Context creation failed. */
            System_printf ("Error: LTE Creation User Security Context failed [Error code %d]\n", errCode);
            return -1;
        }

        /* Perform the alignment checking here */
        if (isDataBufferAligned == 1)
        {
            /* Allocate memory for the packet: */
            ptrTestPacket = Pktlib_allocPacket (appPktlibInstanceHandle, heapHandle, downLinkTestVectors[index].length);
            if (ptrTestPacket == NULL)
            {
                /* Error: Out of memory */
                System_printf ("Error: Packet allocation failed\n");
                return -1;
            }

            /* Get the data buffer. */
            Pktlib_getDataBuffer (ptrTestPacket, &ptrOrgTestPacketBuffer, &orgMessageLength);
        }
        else
        {
            /* Allocate memory for the packet: We allocate an additional byte to misalign the packet. */
            ptrTestPacket = Pktlib_allocPacket (appPktlibInstanceHandle, heapHandle, downLinkTestVectors[index].length + 1);
            if (ptrTestPacket == NULL)
            {
                /* Error: Out of memory */
                System_printf ("Error: Packet allocation failed\n");
                return -1;
            }

            /* Get the data buffer. */
            Pktlib_getDataBuffer (ptrTestPacket, &ptrOrgTestPacketBuffer, &orgMessageLength);

            /* Offset the data packet by 1 byte. */
            Pktlib_setDataOffset (ptrTestPacket, 1);

            /* Discount the additional 1 byte of padding from the packet length also */
            Pktlib_setPacketLen(ptrTestPacket, downLinkTestVectors[index].length);
        }

        /* Get the data buffer associated with the allocated packet. */
        Pktlib_getDataBuffer (ptrTestPacket, &ptrTestPacketBuffer, &messageLength);

        /* Copy the actual payload from the downlink test vector. */
        memcpy ((void *)ptrTestPacketBuffer, (void *)downLinkTestVectors[index].plainMessage, messageLength);

        /***************************************************************************************************
         * Perform cache operations only if either the ciphering *OR* authentication is AES
         * There is no need to perform these actions for SNOW3G or NULL
         ***************************************************************************************************/
        if ((Netfp_getSrbAuthMode(ueHandle)   == Netfp_3gppAuthMode_EIA2)  ||
            (Netfp_getSrbCipherMode(ueHandle) == Netfp_3gppCipherMode_EEA2))
        {
            /* CACHE operations: */
            appWritebackBuffer(ptrOrgTestPacketBuffer, orgMessageLength);
        }
        if (!useGenerateMacI)
        {
            /* Encode the packet: */
            if (Netfp_encodeSrb (ueHandle, ptrTestPacket, downLinkTestVectors[index].count, downLinkTestVectors[index].bearerId) < 0)
            {
                System_printf ("Error: Encoding of SRB using Netfp_encodeSrb() failed\n");
                return -1;
            }
        }
        else
        {
            /* Testing F9 only with NULL F8 */
            if (Netfp_generateMacI (ueHandle, ptrTestPacket, downLinkTestVectors[index].count,
                                    downLinkTestVectors[index].bearerId, 1, Msgcom_getInternalMsgQueueInfo(encodeChannel)) < 0)
            {
                System_printf ("Error: Encoding of SRB using Netfp_generateMacI() failed\n");
                return -1;
            }
        }

        /* Once encoded the packet; will be pushed into the encoded channel */
        Msgcom_getMessage (encodeChannel, (MsgCom_Buffer**)&ptrTestPacket);

        if (1)
        {
            /***************************************************************************************************
             * Perform cache operations only if either the ciphering *OR* authentication is AES
             * There is no need to perform these actions for SNOW3G or NULL
             ***************************************************************************************************/
            if ((Netfp_getSrbAuthMode(ueHandle)   == Netfp_3gppAuthMode_EIA2)  ||
                (Netfp_getSrbCipherMode(ueHandle) == Netfp_3gppCipherMode_EEA2))
            {
                /* Get the data buffer */
                Pktlib_getDataBuffer (ptrTestPacket, &ptrTestPacketBuffer, &messageLength);

                /* CACHE Operation: */
                appInvalidateBuffer(ptrTestPacketBuffer, messageLength);
            }
            else
            {
                /* Get the data buffer */
                Pktlib_getDataBuffer (ptrTestPacket, &ptrTestPacketBuffer, &messageLength);
            }
        }
        else
        {
            /***************************************************************************************************
             * Perform cache operations only if either the ciphering *OR* authentication is AES or NULL
             * There is no need to perform these actions for SNOW3G. This is applicable while doing a
             * Netfp_generateMacI
             ***************************************************************************************************/
            if ((Netfp_getSrbAuthMode(ueHandle)   == Netfp_3gppAuthMode_EIA0)   ||
                (Netfp_getSrbAuthMode(ueHandle)   == Netfp_3gppAuthMode_EIA2)   ||
                (Netfp_getSrbCipherMode(ueHandle) == Netfp_3gppCipherMode_EEA0) ||
                (Netfp_getSrbCipherMode(ueHandle) == Netfp_3gppCipherMode_EEA2))
            {
                /* Get the data buffer */
                Pktlib_getDataBuffer (ptrTestPacket, &ptrTestPacketBuffer, &messageLength);

                /* CACHE Operation: */
                appInvalidateBuffer(ptrTestPacketBuffer, messageLength);
            }
            else
            {
                /* Get the data buffer */
                Pktlib_getDataBuffer (ptrTestPacket, &ptrTestPacketBuffer, &messageLength);
            }
        }

        /* Get the packet identifier */
        Netfp_getPacketId(ptrTestPacket, &ueId, &qci, &bearerId, &flag);

        /* Validate the packet identifier. */
        if ((ueId != TEST_UE_ID) || (bearerId != downLinkTestVectors[index].bearerId))
            System_printf ("Error: Packet Identifier validation failed [Got User Id: %d:%d Expected %d:%d]\n",
                            ueId, bearerId, TEST_UE_ID, downLinkTestVectors[index].bearerId);

        /* Validation: Ensure that the results are correct */
        for (validateIndex = 0; validateIndex < messageLength; validateIndex++)
        {
            if (ptrTestPacketBuffer[validateIndex] != downLinkTestVectors[index].outputMessage[validateIndex])
            {
                System_printf ("Error: Encode validation failed @ index %d Expected 0x%x Got 0x%x\n",
                                validateIndex, downLinkTestVectors[index].outputMessage[validateIndex],
                                ptrTestPacketBuffer[validateIndex]);
                return -1;
            }
        }

        /* Cleanup the packet */
        Pktlib_freePacket(appPktlibInstanceHandle, ptrTestPacket);

        /* Delete the user security context once we are done with the test */
        if (Netfp_deleteUser(ueHandle, &errCode) < 0)
        {
            System_printf ("Error: LTE Deletion User Security Context failed [Error code %d]\n", errCode);
            return -1;
        }
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to test the uplink vectors
 *
 *  @param[in]  encodeChannel
 *      SRB Encoded channel handle where encoded SRB packet will arrive
 *  @param[in]  decodeChannel
 *      SRB Decoded channel handle where decoded SRB packet will arrive
 *  @param[in]  srbFlowHandle
 *      SRB Flow Handle to be used.
 *  @param[in]  isDataBufferAligned
 *      Set to 1 to indicate that the data buffer being passed to the encode/decode
 *      functions is aligned on the word boundary or not
 *
 *  @retval
 *      Not Applicable
 */
static int32_t Test_uplinkVectors
(
    MsgCom_ChHandle         encodeChannel,
    MsgCom_ChHandle         decodeChannel,
    int32_t                 srbFlowId,
    uint8_t                 isDataBufferAligned
)
{
    int32_t                 index;
    int32_t                 errCode;
    Netfp_UserCfg           userCfg;
    Netfp_UserHandle        ueHandle;
    Pktlib_HeapHandle       heapHandle;
    Ti_Pkt*                 ptrTestPacket;
    uint32_t                messageLength;
    uint8_t*                ptrTestPacketBuffer;
    uint32_t                orgMessageLength;
    uint8_t*                ptrOrgTestPacketBuffer;
    uint32_t                validateIndex;
    uint32_t                packetError;
    uint16_t                ueId;
    uint8_t                 qci;
    uint8_t                 bearerId;
    uint8_t                 flag;

    /* Debug Message: */
    System_printf ("Debug: Uplink Test with the Decode API with %s data buffers\n", (isDataBufferAligned == 1) ? "aligned": "unaligned");

    /* Get the NETFP Data Receive Heap */
    heapHandle = Pktlib_findHeapByName (appPktlibInstanceHandle, "Core1_MTU_ReceiveHeap", &errCode);
    if (heapHandle == NULL)
    {
        System_printf ("Error: Unable to find the NETFP Packet Receive Heap [Error code %d]\n", errCode);
        return -1;
    }

    /* Cycle through all the test vectors: */
    for (index = 0; index < UPLINK_TEST_SETS; index++)
    {
        System_printf ("Debug: Uplink Test Vectors %d for Authentication: %s Ciphering: %s\n",
                       (index + 1), Test_getAuthenticationString(upLinkTestVectors[index].authMode),
                       Test_getCipheringString(upLinkTestVectors[index].cipherMode));

        /* Populate the user security configuration. */
        Test_populateUserSecurityCfg (Msgcom_getInternalMsgQueueInfo(encodeChannel),
                                      Msgcom_getInternalMsgQueueInfo(decodeChannel),
                                      srbFlowId,
                                      upLinkTestVectors[index].authMode,
                                      upLinkTestVectors[index].cipherMode,
                                      upLinkTestVectors[index].integrityKey,
                                      upLinkTestVectors[index].encryptionKey,
                                      &userCfg);

        /* Create a User Context */
        ueHandle = Netfp_createUser (netfpClientHandle, &userCfg, &errCode);
        if (ueHandle == NULL)
        {
            /* User Security Context creation failed. */
            System_printf ("Error: LTE Creation User Security Context failed [Error code %d]\n", errCode);
            return -1;
        }

        /* Perform the alignment checking here */
        if (isDataBufferAligned == 1)
        {
            /* Allocate memory for the packet: */
            ptrTestPacket = Pktlib_allocPacket (appPktlibInstanceHandle, heapHandle, upLinkTestVectors[index].length);
            if (ptrTestPacket == NULL)
            {
                /* Error: Out of memory */
                System_printf ("Error: Packet allocation failed\n");
                return -1;
            }

            /* Get the data buffer. */
            Pktlib_getDataBuffer (ptrTestPacket, &ptrOrgTestPacketBuffer, &orgMessageLength);
        }
        else
        {
            /* Allocate memory for the packet: Allocate an additional byte to misalign the packet */
            ptrTestPacket = Pktlib_allocPacket (appPktlibInstanceHandle, heapHandle, upLinkTestVectors[index].length + 1);
            if (ptrTestPacket == NULL)
            {
                /* Error: Out of memory */
                System_printf ("Error: Packet allocation failed\n");
                return -1;
            }

            /* Get the data buffer. */
            Pktlib_getDataBuffer (ptrTestPacket, &ptrOrgTestPacketBuffer, &orgMessageLength);

            /* Offset the data packet by 1 byte. */
            Pktlib_setDataOffset (ptrTestPacket, 1);

            /* Discount the additional 1 byte of padding from the packet length also */
            Pktlib_setPacketLen(ptrTestPacket, upLinkTestVectors[index].length);
        }

        /* Get the data buffer associated with the allocated packet. */
        Pktlib_getDataBuffer (ptrTestPacket, &ptrTestPacketBuffer, &messageLength);

        /* Now copy the actual payload from the test vector. */
        memcpy ((void *)ptrTestPacketBuffer, (void *)upLinkTestVectors[index].cipheredMessage, messageLength);

        /***************************************************************************************************
         * Perform cache operations only if either the ciphering *OR* authentication is AES
         * There is no need to perform these actions for SNOW3G or NULL
         ***************************************************************************************************/
        if ((Netfp_getSrbAuthMode(ueHandle)   == Netfp_3gppAuthMode_EIA2)  ||
            (Netfp_getSrbCipherMode(ueHandle) == Netfp_3gppCipherMode_EEA2))
        {
            /* CACHE operations: */
            appWritebackBuffer(ptrOrgTestPacketBuffer, orgMessageLength);
        }

        /* We now pass the packet for decoding */
        if (Netfp_decodeSrb (ueHandle, ptrTestPacket, upLinkTestVectors[index].count, upLinkTestVectors[index].bearerId) < 0)
        {
            System_printf ("Error: Decoding of SRB using Netfp_decodeSrb() failed\n");
            return -1;
        }

        /* Once decoded the packet; will be pushed into the decoded channel */
        Msgcom_getMessage (decodeChannel, (MsgCom_Buffer**)&ptrTestPacket);

        /***************************************************************************************************
         * Perform cache operations only if either the ciphering *OR* authentication is AES
         * There is no need to perform these actions for SNOW3G or NULL
         ***************************************************************************************************/
        if ((Netfp_getSrbAuthMode(ueHandle)   == Netfp_3gppAuthMode_EIA2)  ||
            (Netfp_getSrbCipherMode(ueHandle) == Netfp_3gppCipherMode_EEA2))
        {
            /* Get the data buffer */
            Pktlib_getDataBuffer (ptrTestPacket, &ptrTestPacketBuffer, &messageLength);

            /* CACHE Operation: */
            appInvalidateBuffer(ptrTestPacketBuffer, messageLength);
        }
        else
        {
            /* Get the data buffer */
            Pktlib_getDataBuffer (ptrTestPacket, &ptrTestPacketBuffer, &messageLength);
        }

        /* Get the packet identifer */
        Netfp_getPacketId(ptrTestPacket, &ueId, &qci, &bearerId, &flag);

        /* Validate the packet identifier. */
        if ((ueId != TEST_UE_ID) || (bearerId != upLinkTestVectors[index].bearerId))
            System_printf ("Error: Packet Identifer validation failed\n");

        /* Ensure that there is NO decoding error */
        packetError = Netfp_getPacketError(ptrTestPacket);
        if (packetError != 0)
        {
            /* Error: Packet Decoding failed */
    	    System_printf ("Error: Packet Decoding failed error code detected %d\n", packetError);
        }
        else
        {
            /* Validation: Ensure that the results are correct */
            for (validateIndex = 0; validateIndex < messageLength; validateIndex++)
            {
                if (ptrTestPacketBuffer[validateIndex] != upLinkTestVectors[index].decodedMessage[validateIndex])
                {
                    System_printf ("Error: Decode validation failed @ index %d Expected 0x%x Got 0x%x\n",
                                    validateIndex, upLinkTestVectors[index].decodedMessage[validateIndex],
                                    ptrTestPacketBuffer[validateIndex]);
                    return -1;
                }
            }
        }

        /* Cleanup the packet */
        Pktlib_freePacket(appPktlibInstanceHandle, ptrTestPacket);

        /* Delete the user security context once we are done with the test */
        if (Netfp_deleteUser(ueHandle, &errCode) < 0)
        {
            System_printf ("Error: LTE Deletion User Security Context failed\n");
            return -1;
        }
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Entry point to the test task which executes the SRB Unit Tests
 *
 *  @retval
 *      Not Applicable.
 */
void Test_srbTask(UArg arg0, UArg arg1)
{
    MsgCom_ChHandle         encodeChannel;
    MsgCom_ChHandle         decodeChannel;
    Msgcom_ChannelCfg       chConfig;
    Netfp_FlowCfg           flowConfig;
    int32_t                 myFlowId;
    int32_t                 errCode;
    Pktlib_HeapHandle       heapHandle;

    System_printf ("***********************************************************\n");
    System_printf ("***************** SRB-SNOW3G Testing **********************\n");
    System_printf ("***********************************************************\n");

    /************************************************************************************
     * Encode Channel: This is the channel where the received packet which is actually
     * meant to be sent over the TEST_SRB_ID will be encoded and placed.
     ************************************************************************************/

    /* Initialize the channel configuration. */
    memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* Populate the channel configuration. */
    chConfig.mode                                                   = Msgcom_ChannelMode_BLOCKING;
    chConfig.appCallBack                                            = NULL;
    chConfig.msgcomInstHandle                                       = appMsgcomInstanceHandle;
    chConfig.u.queueCfg.interruptMode                               = Msgcom_QueueInterruptMode_DIRECT_INTERRUPT;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.queuePendQueue  = appResourceConfig.qPendResponse[0].queue;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.cpIntcId        = appResourceConfig.qPendResponse[0].cpIntcId;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.systemInterrupt = appResourceConfig.qPendResponse[0].systemInterrupt;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.hostInterrupt   = appResourceConfig.qPendResponse[0].hostInterrupt;

    /* Create the Message communicator channel. */
    encodeChannel = Msgcom_create ("Encode", Msgcom_ChannelType_QUEUE, &chConfig, &errCode);
    if (encodeChannel == 0)
    {
        System_printf ("Error: Unable to open the encode channel Error : %d\n", errCode);
        return;
    }

    /************************************************************************************
     * Decode Channel: This is the channel where the encoded packet will be placed once
     * it has been decoded for the specific TEST_SRB_ID
     ************************************************************************************/

    /* Initialize the channel configuration. */
    memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* Populate the channel configuration. */
    chConfig.mode                                                   = Msgcom_ChannelMode_BLOCKING;
    chConfig.appCallBack                                            = NULL;
    chConfig.msgcomInstHandle                                       = appMsgcomInstanceHandle;
    chConfig.u.queueCfg.interruptMode                               = Msgcom_QueueInterruptMode_DIRECT_INTERRUPT;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.queuePendQueue  = appResourceConfig.qPendResponse[1].queue;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.cpIntcId        = appResourceConfig.qPendResponse[1].cpIntcId;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.systemInterrupt = appResourceConfig.qPendResponse[1].systemInterrupt;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.hostInterrupt   = appResourceConfig.qPendResponse[1].hostInterrupt;

    /* Create the Message communicator channel. */
    decodeChannel = Msgcom_create ("Decode", Msgcom_ChannelType_QUEUE, &chConfig, &errCode);
    if (decodeChannel == 0)
    {
        System_printf ("Error: Unable to open the decode channel Error : %d\n", errCode);
        return;
    }

    /* Get the NETFP Data Receive Heap */
    heapHandle = Pktlib_findHeapByName (appPktlibInstanceHandle, "Core1_MTU_ReceiveHeap", &errCode);
    if (heapHandle == NULL)
    {
        System_printf ("Error: Unable to find the Core1 MTU Receive Heap [Error code %d]\n", errCode);
        return;
    }

    /* Initialize the flow configuration. */
    memset((void *)&flowConfig, 0, sizeof(Netfp_FlowCfg));

    /* Populate the flow configuration: Use the NETFP Data Receive Heap for this purpose. */
    flowConfig.numHeaps      = 1;
    flowConfig.heapHandle[0] = heapHandle;
    flowConfig.sopOffset     = 0;
    strcpy (flowConfig.name, "Data-Rx-Flow Handle");

    /* Create the NETFP Flow. */
    myFlowId = Netfp_createFlow (netfpClientHandle, &flowConfig, &errCode);
    if (myFlowId < 0)
    {
        System_printf ("Error: Fast Path Flow Creation Failed [Error code %d]\n", errCode);
        return;
    }

#ifdef GENERATE_TEST_VECTOR
    Test_generateDLTestVector(encodeChannel, decodeChannel, myFlowId);
    Test_generateULTestVector(encodeChannel, decodeChannel, myFlowId);
#else
    System_printf ("Debug: Bypassing generation of test vectors; using predefined test vectors\n");
#endif

    /* Debug Message: */
    System_printf ("------------------------------------------------------------------\n");
    System_printf ("Testing SRB Ciphering & Integrity Protection\n");

    errCode = 0;

    /* Downlink Tests: Aligned using Netfp_encodeSrb */
    if (Test_downlinkVectors(encodeChannel, decodeChannel, myFlowId, 1, 0) < 0)
    {
        System_printf ("Error: Downlink Ciphering Test using encodeSrb with aligned buffers failed\n");
        errCode = 1;
    }
    else
    {
        System_printf ("Debug: Downlink Ciphering Test using encodeSrb with aligned buffers passed\n");
    }

    /* Downlink Tests: Unaligned using Netfp_encodeSrb */
    if (Test_downlinkVectors(encodeChannel, decodeChannel, myFlowId, 0, 0) < 0)
    {
        System_printf ("Error: Downlink Ciphering Test using encodeSrb with unaligned buffers failed\n");
        errCode = 1;
    }
    else
    {
        System_printf ("Debug: Downlink Ciphering Test using encodeSrb with unaligned buffers passed\n");
    }

    /* Downlink Tests: Aligned using Netfp_generateMacI */
    if (Test_downlinkVectors(encodeChannel, decodeChannel, myFlowId, 1, 1) < 0)
    {
        System_printf ("Error: Downlink Ciphering Test using generateMacI with aligned buffers failed\n");
        errCode = 1;
    }
    else
    {
        System_printf ("Debug: Downlink Ciphering Test using generateMacI with aligned buffers passed\n");
    }

    /* Downlink Tests: Unaligned using Netfp_generateMacI */
    if (Test_downlinkVectors(encodeChannel, decodeChannel, myFlowId, 0, 1) < 0)
    {
        System_printf ("Error: Downlink Ciphering Test using generateMacI with unaligned buffers failed\n");
        errCode = 1;
    }
    else
    {
        System_printf ("Debug: Downlink Ciphering Test using generateMacI with unaligned buffers passed\n");
    }

    /* Uplink Tests: Aligned using Netfp_decodeSrb */
    if (Test_uplinkVectors(encodeChannel, decodeChannel, myFlowId, 1) < 0)
    {
        System_printf ("Error: Uplink Ciphering Test using decodeSrb with aligned buffers failed\n");
        errCode = 1;
    }
    else
    {
        System_printf ("Debug: Uplink Ciphering Test using decodeSrb with aligned buffers passed\n");
    }

    /* Uplink Tests: Unaligned using Netfp_decodeSrb */
    if (Test_uplinkVectors(encodeChannel, decodeChannel, myFlowId, 0) < 0)
    {
        System_printf ("Error: Uplink Ciphering Test using decodeSrb with unaligned buffers failed\n");
        errCode = 1;
    }
    else
    {
        System_printf ("Debug: Uplink Ciphering Test using decodeSrb with unaligned buffers passed\n");
    }

#if 0
    /* Uplink Tests: Aligned using Netfp_generateMacI */
    if (Test_uplinkVectors(encodeChannel, decodeChannel, myFlowId, 1, 1) < 0)
    {
        System_printf ("Error: Uplink Ciphering Test using generateMacI with aligned buffers failed\n");
        errCode = 1;
    }
    else
    {
        System_printf ("Debug: Uplink Ciphering Test using generateMacI with aligned buffers passed\n");
    }

    /* Uplink Tests: Unaligned using Netfp_generateMacI */
    if (Test_uplinkVectors(encodeChannel, decodeChannel, myFlowId, 0, 1) < 0)
    {
        System_printf ("Error: Uplink Ciphering Test using generateMacI with unaligned buffers failed\n");
        errCode = 1;
    }
    else
    {
        System_printf ("Debug: Uplink Ciphering Test using generateMacI with unaligned buffers passed\n");
    }
#endif

    /* Display the test status */
    if (errCode == 0)
        System_printf ("Debug: Downlink & Uplink tests passed\n");
    else
        System_printf ("Error: Downlink & Uplink tests failed\n");
    return;
}

