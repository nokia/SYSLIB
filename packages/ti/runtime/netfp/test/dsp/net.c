/*
 *   @file  net.c
 *
 *   @brief   
 *      The file implements standard packet generation utility functions
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
 *
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
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/netfp/netfp.h>

/* Standard Networking Header File. */
#include <net.h>

/**********************************************************************
 ************************** Extern Functions **************************
 **********************************************************************/

/* Heap for sending and receiving data */
extern Pktlib_HeapHandle   mtuReceiveHeap;

/* PKTLIB Instance Handle. */
extern Pktlib_InstHandle    appPktlibInstanceHandle;

/* Cache Writeback Functions. */
extern void appWritebackBuffer(void* ptr, uint32_t size);

/**********************************************************************
 ************************** Unit Test Functions ***********************
 **********************************************************************/

/**
 *  @b Description
 *  @n  
 *      The function is used to populate the GTPU header
 *
 *  @param[in]  msgType
 *      Message type 
 *  @param[in]  GTPU Identifier
 *      GTPU Identifier
 *  @param[in/out]  payloadLen
 *      Length of the data payload which is then modified
 *      to account for the GTPU header 
 *  @param[in]  ptrPayload
 *      Data pointer which points to the actual GTPU Payload.
 *
 *  @retval
 *      Success     -   Pointer to the GTPU header
 *  @retval
 *      Error       -   NULL
 */
static uint8_t* Test_netSetupGTPUHeader
(
    uint8_t     msgType,
    uint8_t*    gtpuId,
    uint16_t*   payloadLen,
    uint8_t*    ptrPayload
)
{
    Net_GTPUHdr*    ptrGTPUHdr;

    /* Get the GTPU Header from the application payload. */
    ptrGTPUHdr = (Net_GTPUHdr*)(ptrPayload - NET_GTP_HDR_SIZE);

    /* Populate the GTPU Header. */
    ptrGTPUHdr->Flags       = 0x30;
    ptrGTPUHdr->MsgType     = msgType;
    ptrGTPUHdr->TotalLength = htons(*payloadLen);
    ptrGTPUHdr->TunnelId[0] = *gtpuId;
    ptrGTPUHdr->TunnelId[1] = *(gtpuId + 1);
    ptrGTPUHdr->TunnelId[2] = *(gtpuId + 2);
    ptrGTPUHdr->TunnelId[3] = *(gtpuId + 3);

    /* The payload length now accounts for the GTP Header */
    *payloadLen = *payloadLen + NET_GTP_HDR_SIZE;

    /* GTPU header has been successfully added */
    return (uint8_t*)ptrGTPUHdr;
}

/**
 *  @b Description
 *  @n  
 *      The function is used to populate the UDP header
 *
 *  @param[in]  srcPort
 *      Source Port
 *  @param[in]  dstPort
 *      Destination Port
 *  @param[in/out]  payloadLen
 *      Length of the payload used and then modified to account
 *      for the additional UDP header
 *  @param[in]  ptrPayload
 *      Pointer to the payload.
 *
 *  @retval
 *      Success     -   Pointer to the UDP header
 *  @retval
 *      Error       -   NULL
 */
static uint8_t* Test_netSetupUDPHeader
(
    uint16_t    srcPort,
    uint16_t    dstPort,
    uint16_t*   payloadLen,
    uint8_t*    ptrPayload
)
{
    Net_UDPHdr* ptrUDPHeader;

    /* Get the UDP Header. */
    ptrUDPHeader = (Net_UDPHdr*)(ptrPayload - NET_UDP_HDR_SIZE);

    /* Populate the UDP Header. 
     *  Ensure that the UDP Header length */
    ptrUDPHeader->DstPort     = htons(dstPort);
    ptrUDPHeader->SrcPort     = htons(srcPort);
    ptrUDPHeader->Length      = htons(NET_UDP_HDR_SIZE + *payloadLen);
    ptrUDPHeader->UDPChecksum = 0;

    /* The new payload length now accounts for the UDP header. */
    *payloadLen = *payloadLen + NET_UDP_HDR_SIZE;

    /* Return the UDP Header. */
    return (uint8_t*)ptrUDPHeader;
}

/**
 *  @b Description
 *  @n  
 *       The function computes the IP checksum. The computed checksum
 *       is populated in the IP header.
 *
 *  @param[in]  ptrIPHeader
 *      This is the pointer to the IPv4 header for which the checksum
 *      is computed.
 *
 *  @retval
 *      Not Applicable.
 */
static void Test_netIPChecksum(Net_IPHdr* ptrIPHeader)
{
    int32_t   tmp1;
    uint16_t  *pw;
    uint32_t  TSum = 0;

    /* Get header size in 4 byte chunks */
    tmp1 = ptrIPHeader->VerLen & 0xF;

    /* Checksum field is NULL in checksum calculations */
    ptrIPHeader->Checksum = 0;

    /* Checksum the header */
    pw = (uint16_t *)ptrIPHeader;
    do {
        TSum += (uint32_t)*pw++;
        TSum += (uint32_t)*pw++;
    } while( --tmp1 );
    TSum = (TSum&0xFFFF) + (TSum>>16);
    TSum = (TSum&0xFFFF) + (TSum>>16);
    TSum = ~TSum;

    /* Note checksum is Net/Host byte order independent */
    ptrIPHeader->Checksum = (uint16_t)TSum;
    return;
}

/**
 *  @b Description
 *  @n  
 *      The function is used to populate the IPv4 header
 *
 *  @param[in]  srcIP
 *      Source IP address
 *  @param[in]  dstIP
 *      Destination IP address
 *  @param[in/out]  payloadLen
 *      Length of the payload which is then modified to account for
 *      the additional IPv4 header
 *  @param[in]  ptrPayload
 *      Pointer to the payload.
 *
 *  @retval
 *      Success     -   Pointer to the IPv4 header
 *  @retval
 *      Error       -   NULL
 */
static uint8_t* Test_netSetupIPv4Header
(
    uint8_t*    srcIP,
    uint8_t*    dstIP,
    uint8_t     tos,
    uint16_t    id,
    uint16_t*   payloadLen,
    uint8_t*    ptrPayload
)
{
    Net_IPHdr* ptrIPHeader;

    /* Get the IPv4 Header. */
    ptrIPHeader = (Net_IPHdr*)(ptrPayload - NET_IPV4_HDR_SIZE);

    /* Populate the IPv4 Header. */
    ptrIPHeader->VerLen     = 0x45;
    ptrIPHeader->Tos        = tos;
    ptrIPHeader->Id         = id;
    ptrIPHeader->FlagOff    = 0x0;
    ptrIPHeader->Ttl        = 128;
    ptrIPHeader->Protocol   = IPPROTO_UDP;
    ptrIPHeader->IPSrc[0]   = *srcIP;
    ptrIPHeader->IPSrc[1]   = *(srcIP + 1);
    ptrIPHeader->IPSrc[2]   = *(srcIP + 2);
    ptrIPHeader->IPSrc[3]   = *(srcIP + 3);
    ptrIPHeader->IPDst[0]   = *dstIP;
    ptrIPHeader->IPDst[1]   = *(dstIP + 1);
    ptrIPHeader->IPDst[2]   = *(dstIP + 2);
    ptrIPHeader->IPDst[3]   = *(dstIP + 3);
    ptrIPHeader->TotalLen   = htons(*payloadLen + NET_IPV4_HDR_SIZE);

    /* Setup the new payload length */
    *payloadLen = *payloadLen + NET_IPV4_HDR_SIZE;

    /* Setup the IP Header checksum. */
    Test_netIPChecksum (ptrIPHeader);

    /* Return the IPv4 Header. */
    return (uint8_t*)ptrIPHeader;
}

/**
 *  @b Description
 *  @n  
 *      The function is used to populate the Ethernet header
 *
 *  @param[in]  srcMAC
 *      Source MAC Address
 *  @param[in]  dstMAC
 *      Destination MAC Address
 *  @param[in]  protocol
 *      Protocol
 *
 *  @retval
 *      Success     -   Pointer to the Eth header
 *  @retval
 *      Error       -   NULL
 */
static uint8_t* Test_netSetupEthHeader
(
    uint8_t*    srcMAC,
    uint8_t*    dstMAC,
    uint16_t    protocol,
    uint8_t*    ptrPayload
)
{
#if 1
    Net_EthHdr* ptrEthHeader;
    uint8_t     index;

    /* Get the Ethernet Header. */
    ptrEthHeader = (Net_EthHdr*)(ptrPayload - NET_ETH_HDR_SIZE);

    /* Setup the destination MAC Address */
    for (index = 0; index < 6; index++)
        ptrEthHeader->DstMac[index] = dstMAC[index];

    /* Setup the source MAC Address */
    for (index = 0; index < 6; index++)
        ptrEthHeader->SrcMac[index] = srcMAC[index];

    /* IP Packet Type */
    ptrEthHeader->Type = htons(protocol);

    /* Return the Ethernet header. */
    return (uint8_t*)ptrEthHeader;
#else
    Net_VlanEthHdr* ptrEthHeader;
    uint8_t         index;

    /* Get the Ethernet Header. */
    ptrEthHeader = (Net_VlanEthHdr*)(ptrPayload - NET_ETH_HDR_SIZE);

    /* Setup the destination MAC Address */
    for (index = 0; index < 6; index++)
        ptrEthHeader->DstMac[index] = dstMAC[index];

    /* Setup the source MAC Address */
    for (index = 0; index < 6; index++)
        ptrEthHeader->SrcMac[index] = srcMAC[index];

    /* IP Packet Type */
    ptrEthHeader->Type = htons(ETH_VLAN);

    /* VLAN Identifier of 0. */
    ptrEthHeader->tci      = htons (0);
    ptrEthHeader->Protocol = htons(protocol);

    /* Return the Ethernet header. */
    return (uint8_t*)ptrEthHeader;
#endif
}

/**
 *  @b Description
 *  @n  
 *      The function is used to send an application payload.
 *
 *  @param[in]  ptrPayload
 *      Pointer to the payload data buffer
 *  @param[in]  payloadLen
 *      Length of the payload being sent
 *  @param[in]  gtpuId
 *      GTPU Identifier; applicable only if the destination
 *      port is 2152.
 *  @param[in]  gtpuMsgType
 *      GTPU Message type which is applicable only if the destination
 *      port is 2152.
 *  @param[in]  dstPort
 *      UDP Destination Port to which the packet is being sent.
 *  @param[in]  srcPort
 *      UDP Source Port from which the packet will be sent
 *  @param[in]  dstIP
 *      IPv4 Destination IP address to which the packet is being sent.
 *  @param[in]  srcIP
 *      IPv4 Source IP address from which the packet will be sent
 *  @param[in]  tos
 *      TOS Byte to be marked in the packet.
 *  @param[in]  dstMAC
 *      Destination MAC address to which the packet is being sent.
 *  @param[in]  srcMAC
 *      Source MAC address from which the packet will be sent
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
int32_t Test_netSendIPv4Payload
(
    uint8_t*    ptrPayload,
    uint16_t    payloadLen,
    uint32_t    gtpuId,
    uint16_t    gtpuMsgType,
    uint16_t    dstPort,
    uint16_t    srcPort,
    uint8_t*    dstIP,
    uint8_t*    srcIP,
    uint8_t     tos,
    uint8_t*    dstMAC,
    uint8_t*    srcMAC,
    uint32_t    loopback
)
{
    Ti_Pkt*     ptrPkt;
    uint32_t    netHeaderSize;
    uint8_t*    ptrDataBuffer;
    uint32_t    dataBufferLen;
    Qmss_QueueHnd   queue;
#if (defined (DEVICE_K2H) || defined (K2K))
    queue    = 640;
#else
    queue    = 904;
#endif

    /* Compute the total size of all the networking headers which are to be added. */
    netHeaderSize = NET_ETH_HDR_SIZE + NET_IPV4_HDR_SIZE + NET_UDP_HDR_SIZE;

    /* If the destination port is the GTPU Port then we need to add the GTPU Header also. */
    if (dstPort == 2152)
        netHeaderSize = netHeaderSize + NET_GTP_HDR_SIZE;

    /* Allocate memory from the heap for a packet which is big enough for all the networking header 
     * and the actual data payload. */
    ptrPkt = Pktlib_allocPacket(appPktlibInstanceHandle, mtuReceiveHeap, netHeaderSize + payloadLen); // + 4);
    if (ptrPkt == NULL)
        return -1;

    /* Get the data buffer. */
    Pktlib_getDataBuffer(ptrPkt, &ptrDataBuffer, &dataBufferLen);

    /* Initialize the data buffer */
    memset ((void *)ptrDataBuffer, 0, dataBufferLen);

    /* Set the data buffer to point to the actual payload */
    ptrDataBuffer = ptrDataBuffer + netHeaderSize;

    /* Copy the payload immediately after leaving the headroom for the networking headers */
    memcpy ((void *)ptrDataBuffer, (void *)ptrPayload, payloadLen);

    /* Is this a GTPU packet? If so populate the GTPU header */
    if (dstPort == 2152)
        ptrDataBuffer = Test_netSetupGTPUHeader (gtpuMsgType, (uint8_t*)&gtpuId, &payloadLen, ptrDataBuffer);

    /* Populate the UDP header */
    ptrDataBuffer = Test_netSetupUDPHeader(srcPort, dstPort, &payloadLen, ptrDataBuffer);

    /* Populate the IPv4 header */
    ptrDataBuffer = Test_netSetupIPv4Header(srcIP, dstIP, tos, TSCL, &payloadLen, ptrDataBuffer);

    /* Populate the Ethernet header */
    ptrDataBuffer = Test_netSetupEthHeader (srcMAC, dstMAC, ETH_IP, ptrDataBuffer);

    if (loopback == 1)
    {
        Cppi_DescTag	tag;
        
        tag.destTagLo = 0x0;
        tag.destTagHi = 0x0;
        tag.srcTagLo  = 0x0;
        tag.srcTagHi  = 0x1;  /* pa_EMAC_PORT_0 */
        Cppi_setTag(Cppi_DescType_HOST, (Cppi_Desc *)ptrPkt, &tag);
    }

    /* PA firmware uses PS data fields for its internal operations.
     * Clear this field so that there is no misbehavior. */
    Cppi_setPSLen (Cppi_DescType_HOST, (Cppi_Desc*)ptrPkt, 0);
    Cppi_setPSFlags (Cppi_DescType_HOST, (Cppi_Desc *)ptrPkt, (1 << 0));

    /* Writeback the packet and the data buffer */
    appWritebackBuffer(ptrDataBuffer, (netHeaderSize + payloadLen));

    /******************************************************************************
     * NOTE: There is an ownership change here from the DSP to the CPDMA. We need
     * to ensure that the packet contents here are written back. This is an example
     * where the MSGCOM channels are NOT being used and an application is directly
     * pushing to the hardware queue. In this case an application is responsible
     * for releasing ownership by itself.
     ******************************************************************************/
    Pktlib_releaseOwnership(appPktlibInstanceHandle, ptrPkt);

    /* The packet can now be pushed to PA Input Queue for classification */
    Qmss_queuePushDescSize((Qmss_QueueHnd)(queue), (Cppi_Desc*)ptrPkt, 128);

    return 0;
}

/**
 *  @b Description
 *  @n  
 *      The function is used to send a packet 
 *      (inclusive of standard networking headers).
 *
 *  @param[in]  ptrBuffer
 *      Pointer to the packet data buffer
 *  @param[in]  packetLen
 *      Length of the packet being sent
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
int32_t Test_netSendPacket
(
    uint8_t*    ptrBuffer,
    uint16_t    packetLen,
    uint32_t    loopback
)
{
    Ti_Pkt*     ptrPkt;
    uint8_t*    ptrDataBuffer;
    uint32_t    dataBufferLen;
    Qmss_QueueHnd   queue;

#if (defined (DEVICE_K2H) || defined (K2K))
    queue    = 640;
#else
    queue    = 904;
#endif

    /* Allocate memory from the heap for a packet which is big enough for the packet. */
    ptrPkt = Pktlib_allocPacket(appPktlibInstanceHandle, mtuReceiveHeap, packetLen);
    if (ptrPkt == NULL)
        return -1;

    /* Get the data buffer. */
    Pktlib_getDataBuffer(ptrPkt, &ptrDataBuffer, &dataBufferLen);

    if (loopback == 1)
    {
        Cppi_DescTag	tag;
        
        tag.destTagLo = 0x0;
        tag.destTagHi = 0x0;
        tag.srcTagLo  = 0x0;
        tag.srcTagHi  = 0x1;  /* pa_EMAC_PORT_0 */
        Cppi_setTag(Cppi_DescType_HOST, (Cppi_Desc *)ptrPkt, &tag);
    }

    /* PA firmware uses PS data fields for its internal operations. 
     * Clear this field so that there is no misbehavior. */
    Cppi_setPSLen (Cppi_DescType_HOST, (Cppi_Desc*)ptrPkt, 0);
    Cppi_setPSFlags (Cppi_DescType_HOST, (Cppi_Desc *)ptrPkt, (1 << 0));

    /* Copy the payload */
    memcpy ((void *)ptrDataBuffer, (void *)ptrBuffer, packetLen);

    /* Writeback the packet and the data buffer */
    appWritebackBuffer(ptrDataBuffer, packetLen);
    
    /******************************************************************************
     * NOTE: There is an ownership change here from the DSP to the CPDMA. We need
     * to ensure that the packet contents here are written back. This is an example
     * where the MSGCOM channels are NOT being used and an application is directly
     * pushing to the hardware queue. In this case an application is responsible
     * for releasing ownership by itself.
     ******************************************************************************/
    Pktlib_releaseOwnership(appPktlibInstanceHandle, ptrPkt);

    /* The packet can now be pushed to Queue 640 for classification */
    Qmss_queuePushDescSize((Qmss_QueueHnd)(queue), (Cppi_Desc*)ptrPkt, 128);
    return 0;
}
