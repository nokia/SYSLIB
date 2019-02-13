/**
 *   @file  netfp_socket.c
 *
 *   @brief
 *      The NETFP Socket provides an interface similar to BSD sockets
 *      which will allows applications to use the NETCP to send &
 *      receive packets.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2012 Texas Instruments, Inc.
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
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <ddal/ddal_cpu.h> //fzm

/* PDK Include Files. */
#include <ti/csl/csl_chip.h>
#include <ti/csl/csl_cache.h>

/* SYSLIB Include Files. */
#include <ti/runtime/netfp/netfp.h>
#include <ti/runtime/netfp/include/netfp_internal.h>
#include <ti/runtime/netfp/include/netfp_net.h>

/* For Debugging only. */
#ifdef __ARMv7
#include <ti/apps/netfp_config/include/NetFP_System_printf.h> //fzm
#else
#include <xdc/runtime/System.h>
#endif

/* MCSDK Patches: These macros are *not* currently supported in the MCSDK. Furture releases
 * of MCSDK will have these macros. We will remove these once those changes are integrated
 * These macros allow the ability to send packets using the raw interface */
#define PA_MCSDK_PATCHES
#ifdef  PA_MCSDK_PATCHES
#ifdef NSS_GEN2
#define NETFP_PASAHO_LINFO_SET_NXT_HDR_TYPE(x, v)       PASAHO_SET_BITFIELD((x)->word1,(v),0,6)
#define NETFP_PASAHO_LINFO_CLR_PMATCH(x)                PASAHO_SET_BITFIELD ((x)->word0,0, 23,1)
#define NETFP_PASAHO_LINFO_SET_VLINK_ENABLE(x)          PASAHO_SET_BITFIELD((x)->word0,1,10,1)
#define NETFP_PASAHO_LINFO_SET_VLINK_ID(x, v)           PASAHO_SET_BITFIELD((x)->word4,(v),16,13)
#else
#define NETFP_PASAHO_LINFO_SET_NXT_HDR_TYPE(x, v)       PASAHO_SET_BITFIELD((x)->word3,(v),16,5)
#define NETFP_PASAHO_LINFO_CLR_PMATCH(x)                PASAHO_SET_BITFIELD ((x)->word1,0, 10,1)
#define NETFP_PASAHO_LINFO_SET_VLINK_ENABLE(x)          PASAHO_SET_BITFIELD((x)->word1,1,9,1)
#define NETFP_PASAHO_LINFO_SET_VLINK_ID(x, v)           PASAHO_SET_BITFIELD((x)->word4,(v),8,8)
#endif /* NSS_GEN2 */
#endif /* MCSDK_PATCHES */

/**************************************************************************
 ************************** Local Definitions *****************************
 **************************************************************************/

/**
 * @brief   Flag which indicates that the socket is bound
 */
#define NETFP_SOCKET_BOUND                  0x1

/**
 * @brief   Flag which indicates that the socket is connected
 */
#define NETFP_SOCKET_CONNECTED              0x2


/**
 * @brief ESP tail containing padding size and next header information
 */
#define IPSEC_ESP_TAIL_SIZE_BYTES 2

/**************************************************************************
 *********************** NETFP Socket Functions ***************************
 **************************************************************************/

static void updateNetHeaderPktIPv4(Netfp_Socket*, Netfp_SockTxMetaInfo_FZM*, uint16_t, int, uint16_t);
static void updateNetHeaderPktIPv6(Netfp_Socket*, Netfp_SockTxMetaInfo_FZM*, int);

#ifndef __ARMv7
/**
 *  @b Description
 *  @n
 *      The function computes a CRC (Cyclic Redundancy Check) value for a given
 *      input bit stream using byte-wise implementation.
 *      This function uses C64+ specific GMPY instruction.
 *
 *  @param[in]  inputBytes[]
 *              Input byte buffer (packed 8-bits-per-byte). The bit order in each
 *              byte is MSB first. When NumInputBits is not multiple of 8, the
 *              last (NumInputBits mod 8) bits are in the MSB bits of
 *              iputBytes[numInputBits/8].
 *  @param[in]  numInputBits
 *              Number of input bits.
 *  @param[in]  crcPolynomial
 *              CRC generation polynomial.
 *              The parameter crcPolynomial shall contain in its L MSB bits the
 *              binary representation of the generator polynomial.
 *  @param[in]  crcInitCond
 *              The initial condition of the CRC shift register.
 *              crcInitCond shall contain in its L MSB bits the initial value of
 *              CRC. The order of these L bits is MSB bit first.
 *  @param[in]  numCrcBits
 *              Number of CRC bits - L = 24 or 16 or 8 bits.
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   CRC value for the input bit stream.
 *                  The return value contains CRC in its L LSB bits.
 *                  For example, for 24-bit CRC:
 *                  32-bit return value = 0000 0000 p0p1p2p3 ... p22p23.
 *                  These L bits shall be appended to the input bits by the
 *                  system in the same way as the input bits are packed, i.e.
 *                  MSB bit first and MSB byte first.
 *  @retval
 *      Error   -   <0
 */
static uint32_t Netfp_crcC64pByte
(
    uint8_t     inputBytes[],
    uint32_t    numInputBits,
    uint32_t    crcPolynomial,
    uint32_t    crcInitCond,
    uint8_t     numCrcBits
)
{
    uint32_t    numBytes;
    uint8_t     numBitsLeft;
    uint8_t     inputByte;
    uint32_t    tmpByte1;
    uint32_t    tmpByte2;
    uint32_t    crcValue;
    uint32_t    byteCnt;

    numBytes     = numInputBits >> 3;
    numBitsLeft  = numInputBits & 7;

    GPLYA = crcPolynomial;
    GPLYB = crcPolynomial;

    /* CRC initialization
     * bit 0 of CrcInitCond is delay 1
     * bit 1 of CrcInitCond is delay 2
     * etc. */
    crcValue = crcInitCond;

    for (byteCnt = 0; byteCnt < numBytes; byteCnt++)
    {
        inputByte = inputBytes[byteCnt];

        tmpByte1 = _gmpy (crcPolynomial, inputByte);
        tmpByte2 = _gmpy (crcValue, 1 << 8);

        crcValue = tmpByte1 ^ tmpByte2;
    }

    /* Process the last numBitsLeft bits. */
    if ( numBitsLeft > 0 )
    {
        inputByte = inputBytes[byteCnt] >> (8 - numBitsLeft);
        tmpByte1  = _gmpy (crcPolynomial, inputByte);
        tmpByte2  = _gmpy (crcValue, 1 << numBitsLeft);
        crcValue  = tmpByte1 ^ tmpByte2;
    }

    crcValue >>= (32 - numCrcBits);
    /* Return CRC. */
    return crcValue;
}
#endif /* __ARMv7 */

/**
 *  @b Description
 *  @n
 *      The function computes Frame Protocol payload CRC.
 *
 *  @param[in]  ptrNetfpSocket
 *      NETFP Socket.
 *  @param[in]  ptrSockTxMetaInfo
 *      Socket Transmit Meta Information
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not Applicable.
 */
static int32_t Netfp_computeFrameProtoCRC
(
    Netfp_Socket*           ptrNetfpSocket,
    Netfp_SockTxMetaInfo*   ptrSockTxMetaInfo
)
{
#ifdef __ARMv7
    /* Error: Frame Protocol CRC calculations is not supported on ARM */
    return -1;
#else
    uint32_t    fpCrc;
    uint32_t    crcLength;
    uint32_t    initVal = 0;
    Ti_Pkt*     ptrNextPayload = ptrSockTxMetaInfo->ptrPayload;
    uint8_t*    ptrPayloadCRC;
    uint8_t*    ptrPayloadBuffer;
    uint32_t 	payloadBufferLen;
    uint32_t 	alignOffset;

     /* Application Layer: Frame Protocol CRC Calculation needs to be added before the
     * packets are encrypted and sent out. This needs to be done in the Software for
     * secure packets. */

    /* CRC Offload: Get the payload data buffer & length. */

    while (ptrNextPayload != NULL)
    {
        /* Get the data payload buffer & length. */
        Pktlib_getDataBuffer(ptrNextPayload, &ptrPayloadBuffer, &payloadBufferLen);

        /* If it is a chain, get the next packet. */
        ptrNextPayload = Pktlib_getNextPacket(ptrNextPayload);

        if (ptrNextPayload == NULL)
        {
            /* This is the last part of the packet. CRC has to be inserted
             * here. Compute CRC length accordingly. */
             crcLength = payloadBufferLen - ptrSockTxMetaInfo->frameProtoPayloadOffset - 2;
        }
        else
        {
            /* Compute CRC of the Frame Protocol payload. */
            crcLength = payloadBufferLen - ptrSockTxMetaInfo->frameProtoPayloadOffset;
        }

        /* Compute the CRC. */
        fpCrc = Netfp_crcC64pByte (ptrPayloadBuffer + ptrSockTxMetaInfo->frameProtoPayloadOffset,
                                       crcLength * 8, 0x80050000, initVal, 16);

        /* Set the init value for the next section of the packet. */
        initVal = fpCrc << 16;
        ptrSockTxMetaInfo->frameProtoPayloadOffset = 0;
    }

    /* Add the CRC to the last two bytes of the packet. */
    *(ptrPayloadBuffer + payloadBufferLen - 2) = fpCrc >> 8;
    *(ptrPayloadBuffer + payloadBufferLen - 1) = (uint8_t) fpCrc;

    /* Cache Hooks: Writeback the payload data buffer. */
    ptrPayloadCRC = ptrPayloadBuffer + payloadBufferLen - 2;
    if ((alignOffset = ((uint32_t)ptrPayloadCRC % 0x80)) != 0)
    {
        /* Not cache line aligned. */
        ptrPayloadCRC = ptrPayloadCRC - alignOffset;
    }

    /* Payload CRC bytes are split across multiple cache lines. */
    if((CACHE_L2_LINESIZE-1) == alignOffset)
	{
        ptrNetfpSocket->ptrNetfpClient->cfg.endMemAccess(ptrPayloadCRC, 2*CACHE_L2_LINESIZE);
    }
	else
	{
        ptrNetfpSocket->ptrNetfpClient->cfg.endMemAccess(ptrPayloadCRC, CACHE_L2_LINESIZE);
    }
#endif /* __ARMv7 */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is determine the PA command set based on the Frame Protocol channel configuration.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP server
 *  @param[in]  ptrFrameProtoCfg
 *      Pointer to the Frame Protocol configuration set using Netfp_Option_SOCK_3GPP_CFG socket option
 *  @param[out]  ptrCmdSetInfo
 *      Pointer to the PA command set information configured based on the Frame Protocol configuration
 *  @param[out]  enableCmdSet
 *      Indicates if the command set has to be configured in PA
 *  @param[out]  errCode
 *      Error Code populated only on error
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   Pointer to command set
 *  @retval
 *      Error   -   NULL
 */
static int32_t Netfp_getFrameProtoCmdSet
(
    Netfp_ServerMCB*                ptrNetfpServer,
    Netfp_FrameProtoCfg*            ptrFrameProtoCfg,
    paCmdInfo_t*                    ptrCmdSetInfo,
    uint32_t*                       enableCmdSet,
    int32_t*                        errCode
)
{
    if ((ptrFrameProtoCfg->fpType == Netfp_FpType_NONE) ||
        (ptrFrameProtoCfg->fpType == Netfp_FpType_EDCH))
    {
        /* E-DCH or NONE: Do not compute CRC */
        *enableCmdSet = 0;
        return 0;
    }
    else if ((ptrFrameProtoCfg->enableRxFpCrc == 0) &&
        (ptrFrameProtoCfg->fpType == Netfp_FpType_DL_DCH))
    {
        /* DL-DCH with CRC disabled: Do not compute CRC */
        *enableCmdSet = 0;
        return 0;
    }
    else
    {
        /* Enable CRC and set command set index */
        *enableCmdSet = 1;
        memset (ptrCmdSetInfo, 0 , sizeof (paCmdInfo_t));
        ptrCmdSetInfo->cmd = pa_CMD_CMDSET;

        /* Determine the command set index based on the Frame Type.
         * Note - the index order is determined by the command set configuration in Netfp_paSetupCommandSets()
         */
        if (ptrFrameProtoCfg->fpType == Netfp_FpType_HS_DSCH_TYPE1)
            ptrCmdSetInfo->params.cmdSet.index = ptrNetfpServer->cmdSet.cmdSetFpCrcIndex;
        else if (ptrFrameProtoCfg->fpType == Netfp_FpType_HS_DSCH_TYPE2)
            ptrCmdSetInfo->params.cmdSet.index = ptrNetfpServer->cmdSet.cmdSetFpCrcIndex+1;
        else if (ptrFrameProtoCfg->fpType == Netfp_FpType_HS_DSCH_TYPE3)
            ptrCmdSetInfo->params.cmdSet.index = ptrNetfpServer->cmdSet.cmdSetFpCrcIndex+2;
        else if (ptrFrameProtoCfg->fpType == Netfp_FpType_DL_DCH)
        {
            if (ptrFrameProtoCfg->numDchBearers == 1)
                ptrCmdSetInfo->params.cmdSet.index = ptrNetfpServer->cmdSet.cmdSetFpCrcIndex+3;
            else if (ptrFrameProtoCfg->numDchBearers == 2)
                ptrCmdSetInfo->params.cmdSet.index = ptrNetfpServer->cmdSet.cmdSetFpCrcIndex+4;
            else if (ptrFrameProtoCfg->numDchBearers == 3)
                ptrCmdSetInfo->params.cmdSet.index = ptrNetfpServer->cmdSet.cmdSetFpCrcIndex+5;
            else if (ptrFrameProtoCfg->numDchBearers == 4)
                ptrCmdSetInfo->params.cmdSet.index = ptrNetfpServer->cmdSet.cmdSetFpCrcIndex+6;
            else
            {
                *errCode = NETFP_EINVAL;
                return -1;
            }
        }
        else if ((ptrFrameProtoCfg->fpType == Netfp_FpType_DL_FACH) ||
                (ptrFrameProtoCfg->fpType == Netfp_FpType_DL_PCH))
                    ptrCmdSetInfo->params.cmdSet.index = ptrNetfpServer->cmdSet.cmdSetFpCrcIndex+7;
        else
        {
            *errCode = NETFP_EINVAL;
            return -1;
        }
    }
    /* Configured command set succesfully */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is called by the application to get the data payload from
 *      the received packet. The standard networking headers (Ethernet, VLAN,
 *      IPv4, IPv6 and UDP) are all skipped.
 *
 *      The function uses the parameter 'gtpuPayload' to determine if the GTPU
 *      header needs to be skipped. If the flag is set to 0 then the function will
 *      never skip the GTPU header. If the flag is set to 1 the function will skip
 *      the GTPU Header. However in the following cases:-
 *          - UDP Destination Port != 2152
 *          - GTPU Message Type != 0xFF
 *      The function will not skip the GTPU header. Please refer to the return code
 *      which will reflect the result of the API. In these case the application is
 *      responsible for parsing the GTPU header by themselves.
 *
 *      The function returns the pointer to the received data payload and actual
 *      payload length. The  function also modifies the data buffer and packet length
 *      in the received packet.
 *
 *      NOTE: The functions reads the networking headers in the packet. The
 *      application should ensure that at least the networking headers are
 *      cache coherent before calling the API.
 *
 *  @param[in]  ptrRxPkt
 *      Pointer to the received packet.
 *  @param[in]  gtpuPayload
 *      Hint flag which if set will try to get the GTPU payload i.e. skipping the GTPU header
 *      else the flag will simply skip the UDP Header and get the UDP payload.
 *  @param[out]  ptrPayload
 *      Pointer to the application payload buffer.
 *  @param[out]  payloadLen
 *      Length of the application payload
 *  @param[out]  ptrPeerAddress
 *      Optional argument which has the peer address information. If the peer
 *      information is not required this parameter can be set to NULL.
 *
 *  \ingroup NETFP_FUNCTION
 *
 *  @retval
 *       0      -   UDP Header was skipped
 *  @retval
 *       1      -   GTPU Header was skipped
 *  @retval
 *      <0      -   Error code
 */
int32_t Netfp_getPayload
(
    Ti_Pkt*             ptrRxPkt,
    uint8_t             gtpuPayload,
    uint8_t**           ptrPayload,
    uint32_t*           payloadLen,
    Netfp_PeerSockAddr* ptrPeerAddress
)
{
    pasahoLongInfo_t*   pasaInfo;
    uint32_t            pasaInfoLen;
    uint8_t*            ptrDataBuffer;
    uint32_t            dataBufferLen;
    uint32_t            endOffset;
    uint16_t            trailerBytes;
    uint16_t            l3Offset;
    uint16_t            l4Offset;
    uint16_t            l5Offset;
    uint16_t            srcPort;
    uint16_t            dstPort;
    uint8_t             index;
    int32_t             retVal;
    Ti_Pkt*             ptrLastPkt;
    Ti_Pkt*             ptrBeforeLastPkt;

    /* Sanity Check: Ensure we are operating on a valid packet. */
    if (ptrRxPkt == NULL)
        return NETFP_EINVAL;

    /* Get the PS information from the packet. */
    if (Cppi_getPSData (Cppi_DescType_HOST, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc *)ptrRxPkt,
                        (uint8_t **)&pasaInfo, &pasaInfoLen) != CPPI_SOK)
    {
        /* Error: Received packet did not have any PS information. This could be because of
         * any of the following reasons:
         *  - Packet was copied (software or infra-dma) and the PS information was lost.
         *  - Large packet which did not pass through the NETCP so the PS information was
         *    not set.
         * In either case we cannot proceed with parsing the packet. Report this as
         * an invalid packet issue. */
        return NETFP_EINVAL;
    }

    /* Get the end offset as reported by the NETCP in the protocol specific information */
    endOffset = PASAHO_LINFO_READ_END_OFFSET(pasaInfo);

    /* Determine the trailer bytes which need to be removed from the packet. The Trailer bytes include the
     * Ethernet FCS and IPSEC Trailers if present */
    trailerBytes = Pktlib_getPacketLen(ptrRxPkt) - endOffset;

    /* Are there any trailer bytes which we need to discount out from the packet. */
    if (trailerBytes != 0)
    {
        /* YES. Set the packet length of the received packet to discount all the trailers. */
        Pktlib_setPacketLen (ptrRxPkt, endOffset);

        /*****************************************************************************
         * We need to now cycle through the chain of packets.
         *  P1 -> P2 -> P3 -> P4
         * Trailer could be in the following cases:
         *  CASE (a) P4 has Data & Trailer
         *  CASE (b) P3 has Data & Trailer and P4 has only the trailer.
         * It is assumed that the data buffers are big enough such that the trailers
         * dont spill over into P2. This is NOT implemented.
         ******************************************************************************/
        ptrBeforeLastPkt = NULL;
        ptrLastPkt       = ptrRxPkt;
        while (Pktlib_getNextPacket (ptrLastPkt) != NULL)
        {
            ptrBeforeLastPkt = ptrLastPkt;
            ptrLastPkt       = Pktlib_getNextPacket (ptrLastPkt);
        }

        /* Get the data buffer length of the last packet in the chain. */
        Pktlib_getDataBuffer(ptrLastPkt, &ptrDataBuffer, &dataBufferLen);

        /* Is the trailer completely in the last buffer? */
        if (dataBufferLen >= trailerBytes)
        {
            /* YES. This is CASE(a) explained above: Remove the trailer from the last buffer. */
            Pktlib_setDataBufferLen (ptrLastPkt, (dataBufferLen - trailerBytes));
        }
        else
        {
            /* NO. Unfortunately the trailer is spread over multiple chained packets.
             * We should have a valid last before packet. */
            if (ptrBeforeLastPkt == NULL)
                return NETFP_EINTERNAL;

            /* There is no data in the last packet (It was all trailer) */
            Pktlib_setDataBufferLen (ptrLastPkt, 0);

            /* Recompute the new trailer which is left now */
            trailerBytes = trailerBytes - dataBufferLen;

            /* Get the data buffer in the before last packet. */
            Pktlib_getDataBuffer(ptrBeforeLastPkt, &ptrDataBuffer, &dataBufferLen);

            /* Sanity Check: The trailer should end in this packet */
            if (dataBufferLen < trailerBytes)
                return NETFP_ENOTIMPL;

            /* The before last packet is */
            Pktlib_setDataBufferLen (ptrBeforeLastPkt, (dataBufferLen - trailerBytes));
        }
    }

    /* Get the offset from the descriptor. */
    l3Offset = PASAHO_LINFO_READ_L3_OFFSET(pasaInfo);
    l4Offset = PASAHO_LINFO_READ_L4_OFFSET(pasaInfo);
    l5Offset = PASAHO_LINFO_READ_L5_OFFSET(pasaInfo);

    /* Get the data buffer. */
    Pktlib_getDataBuffer(ptrRxPkt, &ptrDataBuffer, &dataBufferLen);

    /* Ensure that all the networking headers are in the first packet. */
    if (l5Offset > dataBufferLen)
        return NETFP_ENOTIMPL;

    /* Do we need to populate the peer socket address? */
    if (ptrPeerAddress != NULL)
    {
        /* Determine if the packet is IPv4 or not? */
        if (Netfp_isIPv4Packet ((Netfp_IPHeader*)(ptrDataBuffer + l3Offset)) == 1)
        {
            /* IPv4 Packet: Populate the peer ip address information. */
            Netfp_IPHeader* ipv4hdr = (Netfp_IPHeader*)(ptrDataBuffer + l3Offset);
            ptrPeerAddress->sin_addr.ver = Netfp_IPVersion_IPV4;
            for (index = 0; index < 4; index++)
                ptrPeerAddress->sin_addr.addr.ipv4.u.a8[index] = ipv4hdr->IPSrc[index];
        }
        else
        {
            /* IPv6 Packet: Populate the peer ip address information. */
            ptrPeerAddress->sin_addr.ver = Netfp_IPVersion_IPV6;
            Netfp_IPv6Header* ipv6hdr = (Netfp_IPv6Header*)(ptrDataBuffer + l3Offset);
            for (index = 0; index < 16; index++)
                ptrPeerAddress->sin_addr.addr.ipv6.u.a8[index] = ipv6hdr->SrcAddr.u.a8[index];
        }

        /* Populate the peer UDP port information: We cannot access the UDP header directly
         * because on ARM this could lead to misalignment exceptions. */
        srcPort = (*(ptrDataBuffer + l4Offset + offsetof (Netfp_UDPHeader, SrcPort)) << 8) |
                  (*(ptrDataBuffer + l4Offset + offsetof (Netfp_UDPHeader, SrcPort) + 1));
        ptrPeerAddress->sin_port = srcPort;
    }

    /* The payload data buffer points to after the UDP header. */
    *ptrPayload = (ptrDataBuffer + l5Offset);

    /* Get the UDP Payload length and discount the UDP Header to get the actual payload length. */
    *payloadLen = (*(ptrDataBuffer + l4Offset + offsetof (Netfp_UDPHeader, Length)) << 8) |
                   *(ptrDataBuffer + l4Offset + offsetof (Netfp_UDPHeader, Length) + 1);
    *payloadLen = *payloadLen - UDPHDR_SIZE;

    /* Do we need to get the GTPU Payload? */
    if (gtpuPayload == 1)
    {
        /* YES. Get the UDP destination port: */
        dstPort = (*(ptrDataBuffer + l4Offset + offsetof (Netfp_UDPHeader, DstPort)) << 8) |
                   *(ptrDataBuffer + l4Offset + offsetof (Netfp_UDPHeader, DstPort) + 1);

        /* Was the packet received on the GTPU port? */
        if (dstPort == GTPU_PORT)
        {
            uint8_t gtpMsgType;

            /* YES. Get the GTPU Message type: */
            gtpMsgType = *(ptrDataBuffer + l5Offset + offsetof(Netfp_GTPHeader, MsgType));

            /* Skip the GTPU header if the message type only for data packets. */
            if (gtpMsgType == 0xFF)
            {
                /* Data packet so skip the GTPU Header. */
                l5Offset    = l5Offset    + GTPHDR_SIZE;
                *payloadLen = *payloadLen - GTPHDR_SIZE;
                *ptrPayload = *ptrPayload + GTPHDR_SIZE;

                /* GTPU Header was skipped */
                retVal = 1;
            }
            else
            {
                /* GTPU packet but the message type is not data. This is treated as a UDP packet. */
                retVal = 0;
            }
        }
        else
        {
            /* NO: Destination port was not the GTPU Port. This is treated as a UDP packet. */
            retVal = 0;
        }
    }
    else
    {
        /* NO. We dont need to skip the GTPU Payload. */
        retVal = 0;
    }

    /* Set the data offset in the received packet to skip all the networking headers */
    Pktlib_setDataOffset (ptrRxPkt, l5Offset);
    Pktlib_setPacketLen (ptrRxPkt, *payloadLen);
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to create a NETFP socket. NETFP sockets are
 *      endpoints which can be used to send & receive data directed to
 *      well defined UDP ports.
 *
 *  @param[in]  clientHandle
 *      NETFP Client on which the socket is being created
 *  @param[in]  family
 *      Family for which the socket is being created
 *  @param[out]  errCode
 *      Error Code populated only on error else ignore
 *
 *  \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   Handle to the socket
 *  @retval
 *      Error   -   NULL
 */
Netfp_SockHandle Netfp_socket
(
    Netfp_ClientHandle  clientHandle,
    Netfp_SockFamily    family,
    int32_t*            errCode
)
{
    Netfp_Socket*       ptrNetfpSocket;
    Netfp_ClientMCB*    ptrNetfpClient;
    void*               context;

    /* Sanity Check: Validate the arguments. */
    if (clientHandle == NULL)
    {
        *errCode = NETFP_EINVAL;
        return NULL;
    }

    /* Get the client MCB */
    ptrNetfpClient = (Netfp_ClientMCB*)clientHandle;

    /* Allocate memory for the socket. */
    ptrNetfpSocket = (Netfp_Socket*)ptrNetfpClient->cfg.malloc(sizeof(Netfp_Socket), 0);
    if (ptrNetfpSocket == NULL)
    {
        /* Error: Unable to allocate memory */
        *errCode = NETFP_ENOMEM;
        return NULL;
    }

    /* Initialize the allocated memory block */
    memset ((void *)ptrNetfpSocket, 0, sizeof(Netfp_Socket));

    /* Populate the socket configuration. */
    ptrNetfpSocket->family            = family;
    ptrNetfpSocket->ptrNetfpClient    = ptrNetfpClient;

    /* By default: Enable UDP checksum offload */
    ptrNetfpSocket->udpChksumOffload  = 1;

    /* By default: Set priority to lowest (background) and NOT enabled by default */
    ptrNetfpSocket->priorityTag       = NETFP_VLAN_PRIORITYTAG_DEFAULT;

    /* Single Core Critical Section Enter: */
    context = ptrNetfpClient->cfg.enterCS();

    /* Add the socket to the used list. */
    Netfp_listAdd ((Netfp_ListNode**)&ptrNetfpClient->ptrSocketList, (Netfp_ListNode*)ptrNetfpSocket);

    /* Single Core Critical Section Exit: */
    ptrNetfpClient->cfg.exitCS(context);

    /* Return the socket handle. */
    return (Netfp_SockHandle)ptrNetfpSocket;
}

static Netfp_Socket* Netfp_findInboundSocketByAddress
(
    Netfp_ClientMCB*    ptrNetfpClient,
    Netfp_IPAddr*       ptrNetfpAddr,
    uint16_t            sin_port,
    uint16_t            secure
)
{
    void* context = ptrNetfpClient->cfg.enterCS();

    Netfp_Socket* ptrNetfpSocket = (Netfp_Socket*)Netfp_listGetHead((Netfp_ListNode**)&ptrNetfpClient->ptrSocketList);
    while (ptrNetfpSocket)
    {
        if (((ptrNetfpSocket->sockState & (NETFP_SOCKET_BOUND | NETFP_SOCKET_CONNECTED)) == NETFP_SOCKET_BOUND) &&
            ptrNetfpSocket->localSockAddr.sin_port == sin_port &&
            (ptrNetfpSocket->localSockAddr.op.bind.appInfo >> 31) == secure &&
            Netfp_matchIP (ptrNetfpAddr, &ptrNetfpSocket->bindInfo.innerIPSrc) == 1)
            break;

        ptrNetfpSocket = (Netfp_Socket*)Netfp_listGetNext((Netfp_ListNode*)ptrNetfpSocket);
    }

    ptrNetfpClient->cfg.exitCS(context);

    return ptrNetfpSocket;
}

Qmss_QueueHnd Netfp_getSocketQueueByAddress
(
    Netfp_ClientHandle  clientHandle,
    Netfp_IPAddr*       ptrNetfpAddr,
    uint16_t            sin_port,
    uint16_t            secure,
    uint32_t*           appInfo
)
{
    Netfp_ClientMCB* ptrNetfpClient = (Netfp_ClientMCB*)clientHandle;
    Netfp_Socket* ptrNetfpSocket = Netfp_findInboundSocketByAddress (ptrNetfpClient, ptrNetfpAddr, sin_port, secure);
    if (ptrNetfpSocket)
    {
        *appInfo = ptrNetfpSocket->localSockAddr.op.bind.appInfo;
        return (ptrNetfpSocket->localSockAddr.op.bind.queueHandle & pa_QUEUE_BOUNCE_QUEUE_MASK);
    }

    return -1;
}

/**
 *  @b Description
 *  @n
 *      The function is used to find a matching Layer4 binding.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP server
 *  @param[in]  ptrSockAddr
 *      Socket address to which socket is being bound
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Non-NULL    - Matching Entry
 *  @retval
 *      NULL        - No Matching Entry
 */
static Netfp_Layer4Node* Netfp_findL4Binding
(
    Netfp_ServerMCB*    ptrNetfpServer,
    Netfp_SockAddr*     ptrSockAddr
)
{
    Netfp_Layer4Node*           ptrL4Node;
    Netfp_InboundFastPath*      ptrL4NodeInboundFP;
    Netfp_InboundFastPath*      ptrInboundFastPath;
    uint8_t                     gtpuPort;
    uint8_t                     match;

    /* Are we trying to bind to a GTPU *or* UDP? */
    if (ptrSockAddr->sin_gtpuId != 0)
        gtpuPort = 1;
    else
        gtpuPort = 0;

    /* Get the inbound fast path: */
    ptrInboundFastPath = (Netfp_InboundFastPath*)ptrSockAddr->op.bind.inboundFPHandle;

    /* Cycle through all the Layer4 Nodes: */
    ptrL4Node = (Netfp_Layer4Node*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrLayer4List);
    while (ptrL4Node != NULL)
    {
        /* Reset the match status */
        match = 0;

        /* Is this a GTPU or UDP Port? */
        if (gtpuPort == 1)
        {
            /* GTPU Port: Do the GTPU Identifiers match? */
            if (ptrSockAddr->sin_gtpuId == ptrL4Node->sockAddress.sin_gtpuId)
                match = 1;
        }
        else
        {
            /* UDP Port: Do the ports match? */
            if (ptrSockAddr->sin_port == ptrL4Node->sockAddress.sin_port)
                match = 1;
        }

        /* Do we have a match? */
        if (match == 0)
        {
            /* Get the next layer4 node: */
            ptrL4Node = (Netfp_Layer4Node*)Netfp_listGetNext ((Netfp_ListNode*)ptrL4Node);
            continue;
        }

        /* Get the inbound fast path handle from the L4 Node: */
        ptrL4NodeInboundFP = (Netfp_InboundFastPath*)ptrL4Node->sockAddress.op.bind.inboundFPHandle;

        /* It is possible that a fast path has been deleted and we have sent an event to the NETFP Client
         * but the client has still *not* deleted the socket which implies that the L4 Node is still valid
         * So we cannot trust the INBOUND Fast Path which is in the L4 node */
        if (Netfp_isValidInboundFastPath (ptrNetfpServer, ptrL4NodeInboundFP) == 0)
        {
            /* YES: This L4 node resides on a fast path which has already been deleted but the socket
             * has still not been deleted. This L4 node is basically a ZOMBIE and is not to be considered. */
            ptrL4Node = (Netfp_Layer4Node*)Netfp_listGetNext ((Netfp_ListNode*)ptrL4Node);
            continue;
        }

        /* Ok: This implies that the fast path in the L4 Node is valid. We have already matched the port.
         * Match the LUT Information also to determine if both use the same LUT Information because this is
         * the link to the previous LUT1-x match. */
        if (ptrL4NodeInboundFP->ptrFpLutInfo == ptrInboundFastPath->ptrFpLutInfo)
        {
            /* YES: Perfect Match */
            return ptrL4Node;
        }

        /* Get the next layer4 node: */
        ptrL4Node = (Netfp_Layer4Node*)Netfp_listGetNext ((Netfp_ListNode*)ptrL4Node);
    }
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      The function is used to create and register the Layer4 Binding
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP server
 *  @param[in]  ptrSockAddr
 *      Socket address to which socket is being bound
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   Pointer to the Layer4 Node
 *  @retval
 *      Error   -   NULL
 */
static Netfp_Layer4Node* Netfp_registerL4Binding
(
    Netfp_ServerMCB*    ptrNetfpServer,
    Netfp_SockAddr*     ptrSockAddr
)
{
    Netfp_Layer4Node*   ptrL4Node;

    /* Allocate memory for the L4 Node: */
    ptrL4Node = (Netfp_Layer4Node*)ptrNetfpServer->cfg.malloc (sizeof(Netfp_Layer4Node), 0);
    if (ptrL4Node == NULL)
        return NULL;

    /* Initialize the allocated memory */
    memset ((void *)ptrL4Node, 0, sizeof(Netfp_Layer4Node));

    /* Populate the L4 Node: */
    memcpy ((void*)&ptrL4Node->sockAddress, (void*)ptrSockAddr, sizeof(Netfp_SockAddr));

    /* Add the L4 Node to the server */
    Netfp_listAdd ((Netfp_ListNode**)&ptrNetfpServer->ptrLayer4List, (Netfp_ListNode*)ptrL4Node);
    return ptrL4Node;
}

/**
 *  @b Description
 *  @n
 *      The function is used to unregister and delete the Layer4 Binding
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP server
 *  @param[in]  ptrL4Node
 *      Pointer to the L4 Node to be unregistered
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Netfp_unregisterL4Binding
(
    Netfp_ServerMCB*    ptrNetfpServer,
    Netfp_Layer4Node*   ptrL4Node
)
{
    /* Remove the L4 Node from the server */
    Netfp_listRemoveNode ((Netfp_ListNode**)&ptrNetfpServer->ptrLayer4List, (Netfp_ListNode*)ptrL4Node);

    /* Cleanup the memory */
    ptrNetfpServer->cfg.free (ptrL4Node, sizeof(Netfp_Layer4Node));
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to bind the NETFP Socket with the specific
 *      properties. The socket can receive data only once this API has
 *      been invoked
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP server
 *  @param[in]  ptrSockAddr
 *      Socket address to which socket is being bound
 *  @param[out] ptrSockBindInfo
 *      Pointer to the socket binding information populated
 *  @param[out]  errCode
 *      Error Code populated only on error
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t _Netfp_bind
(
    Netfp_ServerMCB*        ptrNetfpServer,
    Netfp_SockAddr*         ptrSockAddr,
    Netfp_SockBindInfo*     ptrSockBindInfo,
    int32_t*                errCode
)
{
    Netfp_InboundFastPath*      ptrInboundFastPath;
    int32_t                     retVal;
    Netfp_DstInfo               dstInfo;
    paCmdInfo_t                 cmdSetInfo;
    uint32_t                    enableCmdSet;
    int32_t                     l4BindingCheck;

    /* Sanity Check: Get the Fast Path Information. */
    ptrInboundFastPath = (Netfp_InboundFastPath*)ptrSockAddr->op.bind.inboundFPHandle;
    if (unlikely(ptrInboundFastPath == NULL))
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Sanity Check: Ensure that the fast path is valid & still registered with the NETFP Server */
    if (unlikely(Netfp_isValidInboundFastPath (ptrNetfpServer, ptrInboundFastPath) == 0))
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Sanity Check: Fast paths should be inbound since we are trying to bind the socket */
    if (unlikely(ptrInboundFastPath->direction != Netfp_Direction_INBOUND))
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Sanity Check: Make sure that the family in the fast path & socket are one and the same.
     * While adding the fast path we have already verified that the source ip and destination
     * ip belong to the same family. */
    if (ptrSockAddr->sin_family == Netfp_SockFamily_AF_INET)
    {
        /* Socket Family is AF_INET i.e. we support only IPv4 */
        if (unlikely(ptrInboundFastPath->cfg.dstIP.ver != Netfp_IPVersion_IPV4))
        {
            *errCode = NETFP_EINVAL;
            return -1;
        }
    }
    else
    {
        /* Socket Family is AF_INET6 i.e. we support only IPv6 */
        if (unlikely(ptrInboundFastPath->cfg.dstIP.ver != Netfp_IPVersion_IPV6))
        {
            *errCode = NETFP_EINVAL;
            return -1;
        }
    }

    /* Check if the Fast Path is associated with a Security Policy? */
    if (ptrInboundFastPath->ptrSPInfo != NULL)
    {
        /* Ingress SP was specified. Check if the policy check allows the socket bind to operate */
        if (unlikely(Netfp_policyCheckPort(Netfp_Direction_INBOUND, ptrInboundFastPath->ptrSPInfo, ptrSockAddr->sin_port) == 0))
        {
            /* Policy Check failed: */
            *errCode = NETFP_ENOPERM;
            return -1;
        }
    }

    /* We need to decide if we need to perform the L4 Binding check: These are the rules which
     * are used.
     *  (a) LUT Entry is being programmed always perform the L4 Binding check
     *  (b) LUT Entry is NOT being programmed i.e. EGRESS only socket */
    if (ptrSockAddr->op.bind.flowId != 0xFFFFFFFF)
    {
        if (unlikely(ptrInboundFastPath->cfg.fastpathMode == Netfp_FastpathMode_INFOONLY))
        {
            /* Error: Can not do L4 binding with info only fast path. */
            *errCode = NETFP_EINVAL;
            return -1;
        }

        /* Case (a): Always perform the L4 Binding check. */
        l4BindingCheck = 1;
    }
    else
    {
        /* Case (b): Dont perform the L4 Binding check. */
        l4BindingCheck = 0;
    }

    /* Do we need to perform a duplicate L4 Binding check? */
    if (l4BindingCheck == 1)
    {
        /* YES: We need to ensure that the L4 Binding is unique */
        if (unlikely(Netfp_findL4Binding (ptrNetfpServer, ptrSockAddr) != NULL))
        {
            *errCode = NETFP_EINUSE;
            return -1;
        }

        /* Register the L4 binding handle: */
        ptrSockBindInfo->l4BindingHandle = Netfp_registerL4Binding (ptrNetfpServer, ptrSockAddr);
        if (unlikely(ptrSockBindInfo->l4BindingHandle == NULL))
        {
            *errCode = NETFP_ENOMEM;
            return -1;
        }
    }

    /* Populate the socket binding information:*/
    ptrSockBindInfo->ingressFastPathHandle = ptrInboundFastPath;
    ptrSockBindInfo->spId                  = ptrInboundFastPath->cfg.spId;
    ptrSockBindInfo->isSecure              = ptrInboundFastPath->isSecure;

    /* Inherit the "inner" source ip address from the fast path configuration. */
    ptrSockBindInfo->innerIPSrc = ptrInboundFastPath->cfg.dstIP;

    /* Do we need to program the LUT2? */
    if (ptrSockAddr->op.bind.flowId != 0xFFFFFFFF)
    {
        /* YES. Configure the LUT2 and we want all matching packets to be sent to the host. */
        dstInfo.dstType = Netfp_DestType_LAST_RULE;
        dstInfo.channel = ptrSockAddr->op.bind.queueHandle;
        dstInfo.flowId  = ptrSockAddr->op.bind.flowId;

        /* Check the type of channel we are trying to configure */
        if (ptrSockBindInfo->l3GPPcfg.protocol == Netfp_3GPPProto_FRAME_PROTOCOL)
        {
            /* Check if Frame Protocol CRC Offload is enabled in the Server */
            if (unlikely((ptrNetfpServer->frameProtoCrcOffload == 0) && (ptrSockBindInfo->l3GPPcfg.cfg.frameProtoCfg.enableRxFpCrc == 1)))
            {
                /* Error: Frame Protocol CRC Offload is disabled. */
                *errCode = NETFP_EINVAL;
                Netfp_unregisterL4Binding (ptrNetfpServer, ptrSockBindInfo->l4BindingHandle);
                return -1;
            }

            /* Check if this is a Frame Protocol channel and the command set needs to be configured? */
            if (unlikely(Netfp_getFrameProtoCmdSet(ptrNetfpServer, &ptrSockBindInfo->l3GPPcfg.cfg.frameProtoCfg,
                    &cmdSetInfo, &enableCmdSet, errCode) < 0))
            {
                /* Invalid configuration. Failed to create the command set */
                Netfp_unregisterL4Binding (ptrNetfpServer, ptrSockBindInfo->l4BindingHandle);
                return -1;
            }
            if (enableCmdSet)
            {
                retVal = Netfp_addPort (ptrNetfpServer, ptrSockAddr->sin_port, ptrInboundFastPath->ptrFpLutInfo->ipHandle,
                                        ptrSockAddr->op.bind.flowId, ptrSockAddr->op.bind.appInfo,
                                        &dstInfo, &cmdSetInfo, &ptrSockBindInfo->l4Handle);
            }
            else
            {
                retVal = Netfp_addPort (ptrNetfpServer, ptrSockAddr->sin_port, ptrInboundFastPath->ptrFpLutInfo->ipHandle,
                                        ptrSockAddr->op.bind.flowId, ptrSockAddr->op.bind.appInfo,
                                        &dstInfo, NULL, &ptrSockBindInfo->l4Handle);
            }
        }
        else
        {
            /* Is this a GTPU or UDP Port? Instead of the using the port number; we use the GTPU Identifier
             * to help determine how we should program the LUT2. */
            if (ptrSockAddr->sin_gtpuId == 0)
            {
                /* UDP Port  */
                retVal = Netfp_addPort (ptrNetfpServer, ptrSockAddr->sin_port, ptrInboundFastPath->ptrFpLutInfo->ipHandle,
                                        ptrSockAddr->op.bind.flowId, ptrSockAddr->op.bind.appInfo,
                                        &dstInfo, NULL, &ptrSockBindInfo->l4Handle);
            }
            else
            {
                /* GTPU Identifier */
                retVal = Netfp_addGTPTunnel (ptrNetfpServer, ptrSockAddr->sin_gtpuId, ptrInboundFastPath->ptrFpLutInfo->ipHandle,
                                             ptrSockAddr->op.bind.appInfo, NULL, &dstInfo, FALSE, FALSE, &ptrSockBindInfo->l4Handle);
            }
        }
        /* Determine if the operation was succesful or not? */
        if (unlikely(retVal < 0))
        {
            *errCode = retVal;
            Netfp_unregisterL4Binding (ptrNetfpServer, ptrSockBindInfo->l4BindingHandle);
            return -1;
        }

        /* Set the flag to indicate that the LUT handle is valid */
        ptrSockBindInfo->lutEntryValid = 1;
    }
    else
    {
        /* LUT2 does not need to be programmed. Reset the L4 handle */
        ptrSockBindInfo->lutEntryValid = 0;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to bind the socket such that after the LUT2 GTPU identifier
 *      match the packet is passed to the SA for air ciphering. This is applicable only for
 *      LTE channels operating in fast path mode.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP server
 *  @param[in]  ptrSockAddr
 *      Socket address to which socket is being bound
 *  @param[in] ptrSrvSecurityChannel
 *      Pointer to the server security channel.
 *  @param[out] ptrSockBindInfo
 *      Pointer to the socket binding information populated
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0 [See error code]
 */
static int32_t _Netfp_secureBind
(
    Netfp_ServerMCB*            ptrNetfpServer,
    Netfp_SockAddr*             ptrSockAddr,
    Netfp_SrvSecurityChannel*   ptrSrvSecurityChannel,
    Netfp_SockBindInfo*         ptrSockBindInfo
)
{
    Netfp_InboundFastPath*          ptrInboundFastPath;
    int32_t                         retVal;
    Netfp_DstInfo                   dstInfo;

    /* Sanity Check: Get the Fast Path Information. */
    ptrInboundFastPath = (Netfp_InboundFastPath*)ptrSockAddr->op.bind.inboundFPHandle;
    if (ptrInboundFastPath == NULL)
        return NETFP_EINVAL;

    /* Sanity Check: Ensure that the fast path is valid & still registered with the NETFP Server */
    if (Netfp_isValidInboundFastPath (ptrNetfpServer, ptrInboundFastPath) == 0)
        return NETFP_EINVAL;

    /* Sanity Check: Fast paths should be inbound since we are trying to bind the socket */
    if (ptrInboundFastPath->direction != Netfp_Direction_INBOUND)
        return NETFP_EINVAL;

    /* Sanity Check: Make sure that the family in the fast path & socket are one and the same.
     * While adding the fast path we have already verified that the source ip and destination
     * ip belong to the same family. */
    if (ptrSockAddr->sin_family == Netfp_SockFamily_AF_INET)
    {
        /* Socket Family is AF_INET i.e. we support only IPv4 */
        if (ptrInboundFastPath->cfg.dstIP.ver != Netfp_IPVersion_IPV4)
            return NETFP_EINVAL;
    }
    else
    {
        /* Socket Family is AF_INET6 i.e. we support only IPv6 */
        if (ptrInboundFastPath->cfg.dstIP.ver != Netfp_IPVersion_IPV6)
            return NETFP_EINVAL;
    }

    /* Sanity Check: This is valid only for GTPU ports. */
    if (ptrSockAddr->sin_port != GTPU_PORT)
        return NETFP_EINVAL;

    /* Sanity Check: In secure binding the flow identifer needs to be specified. */
    if (ptrSockAddr->op.bind.flowId == 0xFFFFFFFF)
        return NETFP_EINVAL;

    /* Sanity Check: Ensure that a valid queue handle was specified? */
    if (ptrSockAddr->op.bind.queueHandle == 0)
        return NETFP_EINVAL;

    /* Sanity Check: In secure binding the fast path can not be information only. */
    if (ptrInboundFastPath->cfg.fastpathMode == Netfp_FastpathMode_INFOONLY)
        return NETFP_EINVAL;

    /* Check if the Fast Path is associated with a Security Policy? */
    if (ptrInboundFastPath->ptrSPInfo != NULL)
    {
        /* Ingress SP was specified. Check if the policy check allows the socket bind to operate */
        if (Netfp_policyCheckPort(Netfp_Direction_INBOUND, ptrInboundFastPath->ptrSPInfo, ptrSockAddr->sin_port) == 0)
            return NETFP_ENOPERM;
    }

    /* Sanity Check: We do not allow duplicate Layer4 bindings of the UDP Port */
    if (Netfp_findL4Binding (ptrNetfpServer, ptrSockAddr) != NULL)
        return NETFP_EINUSE;

    /* Populate the socket binding information:*/
    ptrSockBindInfo->ingressFastPathHandle = ptrInboundFastPath;
    ptrSockBindInfo->spId                  = ptrInboundFastPath->cfg.spId;
    ptrSockBindInfo->isSecure              = ptrInboundFastPath->isSecure;

    /* Register the L4 binding handle: */
    ptrSockBindInfo->l4BindingHandle = Netfp_registerL4Binding (ptrNetfpServer, ptrSockAddr);
    if (ptrSockBindInfo->l4BindingHandle == NULL)
        return NETFP_ENOMEM;

    /* Inherit the "inner" source ip address from the fast path configuration. */
    ptrSockBindInfo->innerIPSrc = ptrInboundFastPath->cfg.dstIP;

    /* YES. Configure the LUT2 and we want all matching packets to be sent to the host. */
    dstInfo.dstType = Netfp_DestType_LAST_RULE;
    dstInfo.channel = ptrSockAddr->op.bind.queueHandle;
    dstInfo.flowId  = ptrSockAddr->op.bind.flowId;

    /* Program the LUT2 with the security channel */
    retVal = Netfp_addGTPTunnel (ptrNetfpServer, ptrSockAddr->sin_gtpuId, ptrInboundFastPath->ptrFpLutInfo->ipHandle,
                                 ptrSockAddr->op.bind.appInfo, ptrSrvSecurityChannel,
                                 &dstInfo, FALSE, FALSE, &ptrSockBindInfo->l4Handle);
    if (retVal < 0)
    {
        /* Error: Unable to add the GTPU Identifier. Cleanup the L4 binding */
        Netfp_unregisterL4Binding (ptrNetfpServer, ptrSockBindInfo->l4BindingHandle);
        return retVal;
    }

    /* Set the flag to indicate that the LUT handle is valid */
    ptrSockBindInfo->lutEntryValid = 1;
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is invoked on the NETFP Server to unbind a previously
 *      bound socket.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  ptrSockBindInfo
 *      Socket Bind information.
 *  @param[out]  errCode
 *      Error Code populated only on error else ignore
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t _Netfp_unbind
(
    Netfp_ServerMCB*        ptrNetfpServer,
    Netfp_SockBindInfo*     ptrSockBindInfo,
    int32_t*                errCode
)
{
    /* Sockets could be associated with an L4 handle; if the LUT has been programmed. */
    if (ptrSockBindInfo->lutEntryValid == 1)
    {
        /* LUT was programmed; delete the LUT entry. */
        *errCode = Netfp_delL4Handle (ptrNetfpServer, ptrSockBindInfo->l4Handle);
        if (*errCode < 0)
            return -1;

        /* Reset the flag since the LUT entry is not valid anymore */
        ptrSockBindInfo->lutEntryValid = 0;
    }

    /* Unregister the L4 Binding: */
    if(ptrSockBindInfo->l4BindingHandle)
        Netfp_unregisterL4Binding (ptrNetfpServer, ptrSockBindInfo->l4BindingHandle);

    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to populate the Layer2 information required to send
 *      out the packets. L2 connect information is valid only if the next hop
 *      MAC address is known.
 *
 *      NOTE: This function populated all the fields in the Netfp_SockL2ConnectInfo
 *      structure. Ensure that if the structure is modified the function is
 *      coded appropriately.
 *
 *  @sa Netfp_SockL2ConnectInfo
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP server
 *  @param[in]  ptrOutboundFastPath
 *      Pointer to the Outbound fast path
 *  @param[out]  ptrSockL2ConnectInfo
 *      Pointer to the L2 socket connect information populated by the API
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
void Netfp_populateSocketL2ConnectInfo
(
    Netfp_ServerMCB*            ptrNetfpServer,
    Netfp_OutboundFastPath*     ptrOutboundFastPath,
    Netfp_SockL2ConnectInfo*    ptrSockL2ConnectInfo
)
{
    Netfp_Interface*        ptrNetfpInterface;
    Netfp_PhyInterface*     ptrPhyInterface;
    Netfp_IPSecChannel*     ptrIPSecChannel;

    /* Initialize the socket L2 connect information: */
    memset ((void *)ptrSockL2ConnectInfo, 0, sizeof(Netfp_SockL2ConnectInfo));

    /* Is the outbound fast path ACTIVE or ZOMBIE? */
    if (ptrOutboundFastPath->status == Netfp_Status_ZOMBIE)
        return;

    /* Get the NETFP interface & PHYSICAL interface */
    ptrNetfpInterface = ptrOutboundFastPath->ptrNetfpInterface;
    ptrPhyInterface   = ptrNetfpInterface->ptrPhyInterface;

    /* Is the NETFP Interface VLAN? */
    if (ptrNetfpInterface->cfg.type == Netfp_InterfaceType_VLAN)
    {
        /* YES. Populate the VLAN configuration. */
        ptrSockL2ConnectInfo->vlanId = ptrNetfpInterface->cfg.vlanId;
        memcpy ((void *)&ptrSockL2ConnectInfo->vlanMap, (void *)&ptrNetfpInterface->cfg.vlanMap,
                sizeof(Netfp_VLANPriorityMap));
    }
    else
    {
        /* No VLAN identifier. */
        ptrSockL2ConnectInfo->vlanId = 0;
    }
    ptrSockL2ConnectInfo->ifHandle        = (Netfp_IfHandle)ptrNetfpInterface;
    ptrSockL2ConnectInfo->switchPortNum   = ptrPhyInterface->switchPortNum;
    ptrSockL2ConnectInfo->l2HeaderSize    = ptrNetfpInterface->headerSize;
    ptrSockL2ConnectInfo->mtu             = Netfp_min (ptrOutboundFastPath->fastPathPMTU, ptrNetfpInterface->cfg.mtu);
    ptrSockL2ConnectInfo->ifStatus        = ptrNetfpInterface->status;

    /* Inherit the L3 QOS shaper configuration from the interface */
    memcpy ((void *)&ptrSockL2ConnectInfo->l3QosCfg, (void *)&ptrNetfpInterface->l3QosCfg, sizeof(Netfp_L3QoSCfg));

    /* Inherit the DSCP marking map from the physical interface. */
    memcpy ((void *)&ptrSockL2ConnectInfo->innerToOuterDSCPMap[0], (void *)&ptrPhyInterface->innerToOuterDSCPMap[0],
            sizeof(ptrPhyInterface->innerToOuterDSCPMap));

    /* Get the source MAC address to be used from the interface properties */
    memcpy ((void *)&ptrSockL2ConnectInfo->srcMacAddress[0], (void *)ptrNetfpInterface->cfg.macAddress, 6);

    /* Populate the destination MAC address to which the packet is to be sent. */
    memcpy ((void *)&ptrSockL2ConnectInfo->dstMacAddress[0], (void *)ptrOutboundFastPath->nextHopMACAddress, 6);

    /* Populate the IPSec flow id used to pass packet between PA and SA */
    ptrSockL2ConnectInfo->ipsecChanFlowID = Cppi_getFlowId(ptrNetfpServer->ipsecFlowHandle);

    /* Do we have a valid security policy */
    if (ptrOutboundFastPath->ptrSPInfo != NULL)
    {
        /* YES: Is the security policy ACTIVE or a ZOMBIE? */
        if (ptrOutboundFastPath->ptrSPInfo->status == Netfp_Status_ZOMBIE)
        {
            /* Security Policy is a ZOMBIE. This implies that the Security association is not of any
             * concern and we dont need to worry about it now. */
            return;
        }

        /* Is the fast path secure? */
        if (ptrOutboundFastPath->isSecure == 1)
        {
            /* Secure Socket: Get the IPSEC Channel information. */
            ptrIPSecChannel = (Netfp_IPSecChannel*)ptrOutboundFastPath->ptrSPInfo->spCfg.saHandle;

            /* Copy over the necessary information which is required to send packets over the IPSEC tunnel.
             * - SA General channel information
             *      This has the IV, Encryption size etc which is required to calculate the ESP header &
             *      Trailer.
             * - Transmit software information
             *      This is populated in the descriptor and used by the NETCP subystem to encrypt the packet */
            memcpy ((void *)&ptrSockL2ConnectInfo->saGeneralChannelCtrlInfo,
                    (void *)&ptrIPSecChannel->ptrSrvSecurityChannel->saGeneralChannelCtrlInfo,
                    sizeof(Sa_GenCtrlInfo_t));
            memcpy ((void *)&ptrSockL2ConnectInfo->ipSecTxInfo,
                    (void *)&ptrIPSecChannel->ptrSrvSecurityChannel->swInfo.txInfo,
                    sizeof(Sa_SWInfo_t));
        }
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to connect the NETFP Socket with the specific
 *      properties. The socket can send data only once it has been connected
 *      to the destination.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP server
 *  @param[in]  ptrSockAddr
 *      Socket address to which socket is being connected
 *  @param[out] ptrSockConnectInfo
 *      Pointer to the socket connection information populated
 *  @param[out]  errCode
 *      Error Code populated only on error
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t _Netfp_connect
(
    Netfp_ServerMCB*        ptrNetfpServer,
    Netfp_SockAddr*         ptrSockAddr,
    Netfp_SockConnectInfo*  ptrSockConnectInfo,
    int32_t*                errCode
)
{
    Netfp_OutboundFastPath* ptrOutboundFastPath;
    Netfp_IPSecChannel*     ptrIPSecChannel = NULL;

    /* Sanity Check: Get the Fast Path Information. */
    ptrOutboundFastPath = (Netfp_OutboundFastPath*)ptrSockAddr->op.connect.outboundFPHandle;
    if (unlikely(ptrOutboundFastPath == NULL))
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Sanity Check: Ensure that the fast path is valid & still registered with the NETFP Server */
    if (unlikely(Netfp_isValidOutboundFastPath (ptrNetfpServer, ptrOutboundFastPath) == 0))
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Sanity Check: Fast paths should be outbound since we are trying to connect to the remote socket */
    if (unlikely(ptrOutboundFastPath->direction != Netfp_Direction_OUTBOUND))
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Sanity Check: Make sure that the family in the fast path & socket are one and the same.
     * While adding the fast path we have already verified that the source ip and destination
     * ip belong to the same family. */
    if (ptrSockAddr->sin_family == Netfp_SockFamily_AF_INET)
    {
        /* Socket Family is AF_INET i.e. we support only IPv4 */
        if (ptrOutboundFastPath->cfg.dstIP.ver != Netfp_IPVersion_IPV4)
        {
            *errCode = NETFP_EINVAL;
            return -1;
        }
    }
    else
    {
        /* Socket Family is AF_INET6 i.e. we support only IPv6 */
        if (ptrOutboundFastPath->cfg.dstIP.ver != Netfp_IPVersion_IPV6)
        {
            *errCode = NETFP_EINVAL;
            return -1;
        }
    }

    /* Initialize the socket connect information: */
    memset ((void *)ptrSockConnectInfo, 0, sizeof(Netfp_SockConnectInfo));

    /*************************************************************************************************
     * Socket Layer3 Properties
     *************************************************************************************************/

    /* Populate the socket connect information: */
    ptrSockConnectInfo->egressFastPathHandle = ptrOutboundFastPath;
    ptrSockConnectInfo->spId                 = ptrOutboundFastPath->cfg.spId;
    ptrSockConnectInfo->isSecure             = ptrOutboundFastPath->isSecure;

    /* Inherit the fast path marking map from the egress fast path. */
    memcpy ((void *)&ptrSockConnectInfo->fpDSCPMapping[0], (void *)&ptrOutboundFastPath->cfg.dscpMapping[0],
            sizeof(uint8_t)*NETFP_MAX_SOCK_PRIORITY);

    /* Is the fast path associated with a security policy? */
    if (ptrOutboundFastPath->ptrSPInfo != NULL)
    {
        /* Egress SP was specified. Check if the policy check allows the socket connect to operate */
        if (Netfp_policyCheckPort(Netfp_Direction_OUTBOUND, ptrOutboundFastPath->ptrSPInfo, ptrSockAddr->sin_port) == 0)
        {
            /* Policy Check failed: */
            *errCode = NETFP_ENOPERM;
            return -1;
        }

        /* Determine the number of IP addresses which need to be accounted for */
        if (ptrOutboundFastPath->isSecure == 0)
        {
            /* Non secure socket. There is only a single IP address */
            ptrSockConnectInfo->numIP = 1;

            /* Inherit the inner IP destination address from the fast path. */
            ptrSockConnectInfo->innerIPDst = ptrOutboundFastPath->cfg.dstIP;
        }
        else
        {
            /* Secure socket. There will be multiple IP addresses. */
            ptrSockConnectInfo->numIP = 2;

            /* Get the IPSEC Channel information. */
            ptrIPSecChannel = (Netfp_IPSecChannel*)ptrOutboundFastPath->ptrSPInfo->spCfg.saHandle;

            /* Inherit the inner IP destination address from the fast path */
            ptrSockConnectInfo->innerIPDst = ptrOutboundFastPath->cfg.dstIP;

            /* Inherit the outer IP addresses from the egress SA */
            //fzm outerIPSrc/Dst have been changed to an array
            ptrSockConnectInfo->outerIPDst[0] = ptrIPSecChannel->saCfg.dstIP;
            ptrSockConnectInfo->outerIPSrc[0] = ptrIPSecChannel->saCfg.srcIP;

            /* Populate the NAT-T configuration */
            ptrSockConnectInfo->nattEncapCfg.srcPort = ptrIPSecChannel->saCfg.nattEncapCfg.srcPort;
            ptrSockConnectInfo->nattEncapCfg.dstPort = ptrIPSecChannel->saCfg.nattEncapCfg.dstPort;

            /* Setup max allocations for ESP overhead. Varies per algorithms. */
            ptrSockConnectInfo->pktHeaderMargin  = ptrOutboundFastPath->ptrSPInfo->ipsecHdrLen;
            ptrSockConnectInfo->pktTrailerMargin = ptrOutboundFastPath->ptrSPInfo->ipsecTrlLen;

#if 0
            /* ICV alignment may be set to 4 bytes for IPv4, 8 for IPv6 */
	        if ((Netfp_isAddressIPv4(ptrIPSecChannel->saCfg.srcIP) == 1) &&
	            (ptrSockConnectInfo->pktTrailerMargin > 2) )
	        {
	            if( (ptrSockConnectInfo->pktTrailerMargin - 2) % 4 )
	            {
	                ptrSockConnectInfo->pktTrailerMargin += 4-((ptrSockConnectInfo->pktTrailerMargin-2) % 4);
	            }
	        }
	        else
	        {
	            if( (ptrSockConnectInfo->pktTrailerMargin - 2) % 8 )
	            {
	                ptrSockConnectInfo->pktTrailerMargin += 8-((ptrSockConnectInfo->pktTrailerMargin-2) % 8);
	            }
	        }
#endif
	    }
    }
    else
    {
        /* No security policy associated with the fast path. This implies that there is only a single
         * IP address which needs to be accounted for. There is also no need to perform a policy check */
        ptrSockConnectInfo->numIP = 1;

        /* Inherit the inner IP destination address from the fast path. */
        ptrSockConnectInfo->innerIPDst = ptrOutboundFastPath->cfg.dstIP;
    }

    /*************************************************************************************************
     * Socket Layer2 Properties:
     *************************************************************************************************/
    if (ptrOutboundFastPath->status == Netfp_Status_ACTIVE)
    {
        /* Outbound Fast Path was ACTIVE: Populate the L2 connect information in configuration block 0 */
        Netfp_populateSocketL2ConnectInfo (ptrNetfpServer, ptrOutboundFastPath, &ptrSockConnectInfo->l2Info[0]);
    }
    else
    {
        /* OUTBOUND Fast Path was a ZOMBIE: We dont have information to populate the layer2 connect information
         * since we dont have all the information to route the packet. The socket is marked as a ZOMBIE. If it
         * ever becomes active because of some event; the layer2 properties will be setup correctly. */
    }

    /* Configuration Index 0 is active initially: */
    ptrSockConnectInfo->cfgIndex = 0;

    /* Initial Socket status is the same as the status of the fast path:
     * Non Secure Fast Path: If the fast path is active then the socket is operational.
     * Secure Fast Path    : The Fast path status is derived from the outbound SA */
    ptrSockConnectInfo->initalSocketStatus = ptrOutboundFastPath->status;
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is invoked on the NETFP Server to unconnect a previously
 *      connected socket.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  ptrSockConnectInfo
 *      Socket Connect information.
 *  @param[out]  errCode
 *      Error Code populated only on error else ignore
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t _Netfp_unconnect
(
    Netfp_ServerMCB*        ptrNetfpServer,
    Netfp_SockConnectInfo*  ptrSockConnectInfo,
    int32_t*                errCode
)
{
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to suspend the NETFP Socket.
 *      The socket cannot receive data when suspended. Instead the data will be buffered and can be
 *      retrieved when the socket is resumed.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP server
 *  @param[in]  ptrSockAddr
 *      Socket Address Properties which are used to suspend the socket.
 *  @param[in] ptrSockBindInfo
 *      Pointer to the socket bind information.
 *  @param[out]  errCode
 *      Error Code populated only on error
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t _Netfp_suspendSocket
(
    Netfp_ServerMCB*        ptrNetfpServer,
    Netfp_SockAddr*         ptrSockAddr,
    Netfp_SockBindInfo*     ptrSockBindInfo,
    int32_t*                errCode
)
{
    Netfp_InboundFastPath*      ptrInboundFastPath;
    int32_t                     retVal;
    Netfp_DstInfo               dstInfo;

    /* Sanity Check: Get the Fast Path Information. */
    ptrInboundFastPath = (Netfp_InboundFastPath*)ptrSockAddr->op.bind.inboundFPHandle;
    if (ptrInboundFastPath == NULL)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Configure the LUT2 and we want all matching packets to be sent to the buffering queue. */
    dstInfo.dstType = Netfp_DestType_LAST_RULE;
    dstInfo.channel = ptrSockAddr->op.bind.queueHandle;
    dstInfo.flowId  = ptrSockAddr->op.bind.flowId;
    retVal = Netfp_addGTPTunnel (ptrNetfpServer, ptrSockAddr->sin_gtpuId, ptrInboundFastPath->ptrFpLutInfo->ipHandle,
                                 ptrSockAddr->op.bind.appInfo, NULL, &dstInfo, TRUE, FALSE, &ptrSockBindInfo->l4Handle);
    if (retVal < 0)
    {
        *errCode = retVal;
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to resume a suspended the NETFP Socket.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP server
 *  @param[in]  ptrSockAddr
 *      Socket Address Properties which are used to resume the socket.
 *  @param[in] ptrSockBindInfo
 *      Pointer to the socket bind information.
 *  @param[out]  errCode
 *      Error Code populated only on error
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t _Netfp_resumeSocket
(
    Netfp_ServerMCB*        ptrNetfpServer,
    Netfp_SockAddr*         ptrSockAddr,
    Netfp_SockBindInfo*     ptrSockBindInfo,
    int32_t*                errCode
)
{
    Netfp_InboundFastPath*      ptrInboundFastPath;
    Netfp_DstInfo               dstInfo;

    /* Sanity Check: Get the Fast Path Information. */
    ptrInboundFastPath = (Netfp_InboundFastPath*)ptrSockAddr->op.bind.inboundFPHandle;
    if (ptrInboundFastPath == NULL)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Configure the LUT2 to forward all packets to the destination queue. */
    dstInfo.dstType = Netfp_DestType_LAST_RULE;
    dstInfo.channel = ptrSockAddr->op.bind.queueHandle;
    dstInfo.flowId  = ptrSockAddr->op.bind.flowId;
    *errCode = Netfp_addGTPTunnel (ptrNetfpServer, ptrSockAddr->sin_gtpuId, ptrInboundFastPath->ptrFpLutInfo->ipHandle,
                                   ptrSockAddr->op.bind.appInfo, NULL, &dstInfo, TRUE, FALSE, &ptrSockBindInfo->l4Handle);
    if (*errCode < 0)
        return -1;
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to resume a suspended the NETFP Socket. After the LUT2 GTPU identifier
 *      match the packet is passed to the SA for air ciphering. This is applicable only for
 *      LTE channels operating in fast path mode.
 *
 *      This is used internally and is not exported to the application developers. Please be aware
 *      that the following parameters are overloaded for the API:-
 *      - Bind App Information (appInfo):
 *        This should be configured to be the new COUNTC to be used for the security channel.
 *      - Source Port (sin_port):
 *        The function is only applicable for GTPU Identifiers and so the port is always 2152 and is
 *        not really used since the GTPU Identifier is all that is required to be specified. If the
 *        sin_port is set to 0 we dont program the COUNTC else we configure it.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP server
 *  @param[in]  ptrSockAddr
 *      Socket Address Properties which are used to resume the socket.
 *  @param[in] ptrSrvSecurityChannel
 *      Pointer to the server security channel.
 *  @param[in] ptrSockBindInfo
 *      Pointer to the socket bind information.
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t _Netfp_resumeSecureSocket
(
    Netfp_ServerMCB*            ptrNetfpServer,
    Netfp_SockAddr*             ptrSockAddr,
    Netfp_SrvSecurityChannel*   ptrSrvSecurityChannel,
    Netfp_SockBindInfo*         ptrSockBindInfo
)
{
    Netfp_InboundFastPath*      ptrInboundFastPath;
    int32_t                     retVal;
    Netfp_DstInfo               dstInfo;

    /* Sanity Check: Get the Fast Path Information. */
    ptrInboundFastPath = (Netfp_InboundFastPath*)ptrSockAddr->op.bind.inboundFPHandle;
    if (ptrInboundFastPath == NULL)
        return NETFP_EINVAL;

    /* Sanity Check: We need to have a valid queue handle */
    if (ptrSockAddr->op.bind.queueHandle == 0)
        return NETFP_EINVAL;

    /* Stop SA: This is done to preserve the ordering of packets. */
    Cppi_channelPause (ptrNetfpServer->cppiCipherTxChHnd);

    /* Populate the destination information: We want all packet matching the GTPU Identifier to be
     * sent to the SA for Ciphering. */
    dstInfo.dstType = Netfp_DestType_LAST_RULE;
    dstInfo.channel = ptrNetfpServer->netcpTxQueue[NSS_SA_QUEUE_SASS2_INDEX];
    dstInfo.flowId  = ptrSockAddr->op.bind.flowId;

    /* Reprogram the LUT2: */
    retVal = Netfp_addGTPTunnel (ptrNetfpServer, ptrSockAddr->sin_gtpuId, ptrInboundFastPath->ptrFpLutInfo->ipHandle,
                                 ptrSockAddr->op.bind.appInfo, ptrSrvSecurityChannel,
                                 &dstInfo, TRUE, FALSE, &ptrSockBindInfo->l4Handle);
    if (retVal == 0)
        Qmss_queueDivert (ptrSockAddr->op.bind.queueHandle, ptrNetfpServer->netcpTxQueue[NSS_SA_QUEUE_SASS2_INDEX], Qmss_Location_HEAD);

    /* NOTE: Update and setup the countC for the security channel. The order of this is very important and can NOT
     * be exchanged with the above divert because we would want the SA Air Ciphering PDSP to first process the COUNTC
     * command and then process all the diverted packets. */
    if (ptrSockAddr->sin_port == 1)
        retVal = Netfp_configureCountC(ptrNetfpServer, ptrSrvSecurityChannel, ptrSockAddr->op.bind.appInfo);

    /* Enable the SA: */
    Cppi_channelEnable (ptrNetfpServer->cppiCipherTxChHnd);
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to populate the GTPU Header.
 *
 *  @param[in]  ptrNetfpSocket
 *      NETFP Socket.
 *  @param[in]  ptrDataBuffer
 *      Pointer to the GTPU Header.
 *  @param[in]  payloadLen
 *      Length of the actual data payload which is being sent.
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not Applicable.
 */
static void Netfp_setupGTPUHeader
(
    Netfp_Socket*   ptrNetfpSocket,
    uint8_t*        ptrDataBuffer,
    uint16_t        payloadLen
)
{
    Netfp_GTPHeader*    ptrGTPUHdr;

    /* Get the GTPU Header. */
    ptrGTPUHdr = (Netfp_GTPHeader*)ptrDataBuffer;

    /* Populate the GTPU Header.
     *  - The flags is configured for GTPU Version 1 (Version & Protocol Type)
     *  - Set the length to include the length of the payload.
     *  - Message Type is set to 0xFF */
    ptrGTPUHdr->Flags       = 0x30;
    ptrGTPUHdr->MsgType     = 0xFF;
    ptrGTPUHdr->TotalLength = Netfp_htons(payloadLen);
    ptrGTPUHdr->TunnelId[0] = (ptrNetfpSocket->peerSockAddr.sin_gtpuId & 0xFF000000) >> 24;
    ptrGTPUHdr->TunnelId[1] = (ptrNetfpSocket->peerSockAddr.sin_gtpuId & 0x00FF0000) >> 16;
    ptrGTPUHdr->TunnelId[2] = (ptrNetfpSocket->peerSockAddr.sin_gtpuId & 0x0000FF00) >> 8;
    ptrGTPUHdr->TunnelId[3] = (ptrNetfpSocket->peerSockAddr.sin_gtpuId & 0x000000FF);
    return;
}

/**
 *  @b Description
 *  @n
 *      The function does the Layer4 Checksum computation for UDP
 *
 *  @param[in]  ptrPayloadPkt
 *      Pointer to the data payload packet.
 *  @param[in]  gtpuHeaderSize
 *      GTPU Header Size if one is present.
 *  @param[in]  ptr_l4Hdr
 *      Pointer to the Layer4 UDP Header.
 *  @param[in]  ptr_pseudoHdr
 *      Pointer to the Pseudo Header.
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Returns the computed checksum.
 *
 *  @pre
 *      Ensure that the Checksum field in the layer4 header is set to 0
 *      before calling this function.
 */
static uint16_t Netfp_udp6Checksum
(
    Ti_Pkt*             ptrPayloadPkt,
    uint8_t             gtpuHeaderSize,
    uint8_t*            ptr_l4Hdr,
    Netfp_PseudoV6Hdr*  ptr_pseudoHdr
)
{
    int         tmp1;
    uint16_t*   pw;
    uint32_t    TSum;
    uint8_t*    ptrPayloadBuffer;
    Ti_Pkt*     ptrChainedPkt;
    volatile    uint16_t    checksum;

    /*************************************************************
     * UDP Checksum is calculated in the following parts:-
     *  - UDP Header
     *  - Optional GTPU Header which if present starts immediately
     *    after the UDP Header.
     *  - Payload which is carried in a different packet
     *************************************************************/

    /* Get the header size (GTPU Header is actually a payload to the
     * UDP but it immediately follows the UDP Header) */
    tmp1 = UDPHDR_SIZE + gtpuHeaderSize;

    /* Initialize the checksum field. */
    TSum = 0;

    /* Checksum the header */
    pw = (uint16_t *)ptr_l4Hdr;
    for( ; tmp1 > 1; tmp1 -= 2 )
    {
        TSum = TSum + *pw;
        pw++;
    }

#ifdef _BIG_ENDIAN
    if( tmp1 )
        TSum += (uint32_t)(*pw & 0xFF00);
#else
    if( tmp1 )
        TSum += (uint32_t)(*pw & 0x00FF);
#endif

    /* Point to the first packet in the Chained packet */
    ptrChainedPkt = ptrPayloadPkt;

    /* Walk through all chained packet to calculate the UDP checksum */
    while(ptrChainedPkt)
    {
        /* Get the payload data buffer & length. */
        Pktlib_getDataBuffer(ptrChainedPkt, &ptrPayloadBuffer, (uint32_t*)&tmp1);

        /* Get the payload buffer. */
        pw = (uint16_t *)ptrPayloadBuffer;

        /* Checksum the payload */
        for( ; tmp1 > 1; tmp1 -= 2 )
        {
            TSum = TSum + *pw;
            pw++;
        }

#ifdef _BIG_ENDIAN
        if( tmp1 )
            TSum += (uint32_t)(*pw & 0xFF00);
#else
        if( tmp1 )
            TSum += (uint32_t)(*pw & 0x00FF);
#endif

        /* Get next chained packet */
        ptrChainedPkt = Pktlib_getNextPacket(ptrChainedPkt);
    }

    /* Checksum the pseudo header */
    pw = (uint16_t *)ptr_pseudoHdr;
    for( tmp1=0; tmp1 < sizeof(Netfp_PseudoV6Hdr)/2; tmp1++ )
    {
        TSum = TSum + *pw;
        pw++;
    }

    TSum = (TSum & 0xFFFF) + (TSum >> 16);
    checksum = (TSum & 0xFFFF) + (TSum >> 16);

    /* Special case the 0xFFFF checksum - don't use a checksum
     * value of 0x0000 */
    if( checksum != 0xFFFF )
        checksum = ~checksum;

    /* Note checksum is Net/Host byte order independent */
    return checksum;
}

/**
 *  @b Description
 *  @n
 *      The function is used to populate the UDP header.
 *
 *  @param[in]  ptrNetfpSocket
 *      NETFP Socket.
 *  @param[in]  ptrSockTxMetaInfo
 *      Pointer to the NETFP socket transmit meta information
 *  @param[in]  ptrHdrDataBuffer
 *      Pointer to the UDP Header
 *  @param[in]  payloadLen
 *      Length of the actual data payload which is being sent.
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not Applicable.
 */
static void Netfp_setupUDPHeader
(
    Netfp_Socket*           ptrNetfpSocket,
    Netfp_SockTxMetaInfo*   ptrSockTxMetaInfo,
    uint8_t*                ptrHdrDataBuffer,
    uint16_t                payloadLen
)
{
    Netfp_UDPHeader*    ptrUDPHeader;
    Ti_Pkt*             ptrPayloadPkt;
    uint8_t             gtpuHeaderSize;
    uint16_t            udpHdrOffset;

    /* Get the payload packet to calculate the UDP checksum */
    ptrPayloadPkt  = ptrSockTxMetaInfo->ptrPayload;
    /* Get the GTPU header size */
    gtpuHeaderSize = ptrSockTxMetaInfo->gtpuHeaderSize;
    /* Get the UDP header offset */
    udpHdrOffset   = ptrSockTxMetaInfo->headerSize - ptrSockTxMetaInfo->gtpuHeaderSize - UDPHDR_SIZE;

    /* Get the UDP Header. */
    ptrUDPHeader = (Netfp_UDPHeader*)ptrHdrDataBuffer;

    /* Populate the UDP Header. */
    ptrUDPHeader->DstPort     = Netfp_htons(ptrNetfpSocket->peerSockAddr.sin_port);
    ptrUDPHeader->SrcPort     = Netfp_htons(ptrNetfpSocket->localSockAddr.sin_port);
    ptrUDPHeader->Length      = Netfp_htons(UDPHDR_SIZE + payloadLen);
    ptrUDPHeader->UDPChecksum = 0x0000;

    /* Prepare to calculate UDP checksum through PA hardware for IPv6. */
    if (ptrNetfpSocket->family == Netfp_SockFamily_AF_INET6)
    {
        Netfp_PseudoV6Hdr pseudoHdr;

        if (ptrSockTxMetaInfo->hwUDPChksum)
        {
            uint32_t     initVal = 0;
            int          tmp1;
            uint16_t*    pw;
            volatile     uint16_t checksum;

            /* Initialize the Pseudo header which is used for  UDP checksum calculations. */
            pseudoHdr.DstAddr = ptrNetfpSocket->connectInfo.innerIPDst.addr.ipv6;
            pseudoHdr.SrcAddr = ptrNetfpSocket->bindInfo.innerIPSrc.addr.ipv6;
            pseudoHdr.PktLen  = ptrUDPHeader->Length;
            pseudoHdr.Rsvd[0] = 0;
            pseudoHdr.Rsvd[1] = 0;
            pseudoHdr.Rsvd[2] = 0;
            pseudoHdr.NxtHdr  = IPPROTO_UDP;

            /* Calculate pseudo Header checksum value */
            pw = (uint16_t *)&pseudoHdr;
            for( tmp1=0; tmp1 < sizeof(Netfp_PseudoV6Hdr)/2; tmp1++ )
            {
                initVal = initVal + *pw;
                pw++;
            }

            /* Convert to 16bits checksum */
            initVal = (initVal & 0xFFFF) + (initVal >> 16);
            if(initVal >> 16)
                checksum = (initVal & 0xFFFF) + (initVal >> 16);
            else
                checksum = initVal & 0xFFFF;

            /* Prepare UDP checksum fields required by PA, these values will be
               used when construct UDP checksum Tx command */
            ptrSockTxMetaInfo->udpHwChkSum.startOffset   = udpHdrOffset;
            ptrSockTxMetaInfo->udpHwChkSum.lengthBytes   = payloadLen + UDPHDR_SIZE;
            ptrSockTxMetaInfo->udpHwChkSum.initialSum    = Netfp_htons(checksum);
        }
        else
        {
            /* Initialize the Pseudo header which is used for Layer4 UDP checksum calculations. */
            pseudoHdr.SrcAddr = ptrNetfpSocket->bindInfo.innerIPSrc.addr.ipv6;
            pseudoHdr.DstAddr = ptrNetfpSocket->connectInfo.innerIPDst.addr.ipv6;
            pseudoHdr.PktLen  = ptrUDPHeader->Length;
            pseudoHdr.Rsvd[0] = 0;
            pseudoHdr.Rsvd[1] = 0;
            pseudoHdr.Rsvd[2] = 0;
            pseudoHdr.NxtHdr  = IPPROTO_UDP;

            /* Compute the UDP Checksum in software. */
            ptrUDPHeader->UDPChecksum = Netfp_udp6Checksum(ptrPayloadPkt, gtpuHeaderSize, (uint8_t*)ptrUDPHeader, &pseudoHdr);
        }
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to populate the IPv4 header.
 *
 *  @param[in]  ptrNetfpSocket
 *      Pointer to the NETFP Socket
 *  @param[in]  ptrDataBuffer
 *      Pointer to the IP Header
 *  @param[in]  srcIP
 *      Source IPv4 Address to be added to the packet.
 *  @param[in]  dstIP
 *      Destination IPv4 Address to be added to the packet.
 *  @param[in]  protocol
 *      Protocol to be added to the packet.
 *  @param[in]  packetId
 *      Unique Packet Identifier
 *  @param[in]  priority
 *      TOS Byte to be configured in the IP header.
 *  @param[in]  flagOffset
 *      Fragmentation Offset & Flag definition
 *  @param[in]  payloadLen
 *      Length of the actual data payload which is being sent.
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not Applicable.
 */
static void Netfp_setupIPHeader
(
    Netfp_Socket*   ptrNetfpSocket,
    uint8_t*        ptrDataBuffer,
    uint8_t*        srcIP,
    uint8_t*        dstIP,
    uint8_t         protocol,
    uint16_t        packetId,
    uint8_t         priority,
    uint16_t        flagOffset,
    uint16_t        payloadLen
)
{
    Netfp_IPHeader* ptrIPHeader;

    /* Get the IP Header. */
    ptrIPHeader = (Netfp_IPHeader*)ptrDataBuffer;

    /* Does the socket support dont fragmentation? */
    if (ptrNetfpSocket->dontFrag)
        flagOffset = flagOffset | IPV4_FLAGS_DF_MASK;

    /* Populate the IP Header: Source & Destination IP address are retreived from
     * the local & peer socket address.
     *
     * Build the TOS byte using DSCP passed. Consider lower order 2 bits - ECN to be always zero. */
    ptrIPHeader->VerLen     = 0x45;
    ptrIPHeader->Tos        = (priority << 2);
    ptrIPHeader->Id         = packetId;
    ptrIPHeader->FlagOff    = Netfp_htons(flagOffset);
    ptrIPHeader->Ttl        = 255; //fzm
    ptrIPHeader->Protocol   = protocol;
    ptrIPHeader->IPSrc[0]   = srcIP[0];
    ptrIPHeader->IPSrc[1]   = srcIP[1];
    ptrIPHeader->IPSrc[2]   = srcIP[2];
    ptrIPHeader->IPSrc[3]   = srcIP[3];
    ptrIPHeader->IPDst[0]   = dstIP[0];
    ptrIPHeader->IPDst[1]   = dstIP[1];
    ptrIPHeader->IPDst[2]   = dstIP[2];
    ptrIPHeader->IPDst[3]   = dstIP[3];
    ptrIPHeader->TotalLen   = Netfp_htons(payloadLen + IPHDR_SIZE);

    /* Setup the IP Header checksum. */
    Netfp_IPChecksum (ptrIPHeader);
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to populate the IPv6 header.
 *
 *  @param[in]  ptrDataBuffer
 *      Pointer to the IPv6 Header
 *  @param[in]  srcIP
 *      Source IPv6 Address to be added to the packet.
 *  @param[in]  dstIP
 *      Destination IPv6 Address to be added to the packet.
 *  @param[in]  protocol
 *      Protocol to be added to the packet.
 *  @param[in]  priority
 *      TOS Byte to be configured in the IP header.
 *  @param[in]  payloadLen
 *      Length of the actual data payload which is being sent.
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not Applicable.
 */
static void Netfp_setupIPv6Header
(
    uint8_t*        ptrDataBuffer,
    uint8_t*        srcIP,
    uint8_t*        dstIP,
    uint8_t         protocol,
    uint8_t         priority,
    uint16_t        payloadLen
)
{
    Netfp_IPv6Header* ptrIPv6Header;
    uint8_t           index;

    /* Get the IPv6 Header. */
    ptrIPv6Header = (Netfp_IPv6Header*)ptrDataBuffer;

    /* Populate the IPv6 header. */
    ptrIPv6Header->VerTC         = 0x60;
    ptrIPv6Header->FlowLabel[0]  = 0;
    ptrIPv6Header->FlowLabel[1]  = 0;
    ptrIPv6Header->FlowLabel[2]  = 0;
    ptrIPv6Header->VerTC         = (ptrIPv6Header->VerTC & 0xF0) | ((priority >> 2) & 0x0F);
    ptrIPv6Header->FlowLabel[0]  = (ptrIPv6Header->FlowLabel[0] & 0x3F) | (((priority & 0x03) << 6) & 0xC0);
    ptrIPv6Header->PayloadLength = Netfp_htons(payloadLen);
    ptrIPv6Header->NextHeader    = protocol;
    ptrIPv6Header->HopLimit      = 255; //fzm

    /* Setup the Source ip address */
    for (index = 0; index < 16; index++)
        ptrIPv6Header->SrcAddr.u.a8[index] = srcIP[index];

    /* Setup the Destination ip address */
    for (index = 0; index < 16; index++)
        ptrIPv6Header->DstAddr.u.a8[index] = dstIP[index];
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to populate the IPv6 Fragmentation header.
 *
 *  @param[in]  ptrDataBuffer
 *      Pointer to the IPv6 Fragmentation Header
 *  @param[in]  nextHeader
 *      Next Header Field
 *  @param[in]  fragOffset
 *      Fragmentation Offset
 *  @param[in]  fragId
 *      Fragmentation Identifier
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not Applicable.
 */
static void Netfp_setupIPv6FragHeader
(
    uint8_t*        ptrDataBuffer,
    uint8_t         nextHeader,
    uint16_t        fragOffset,
    uint8_t*        fragId
)
{
    Netfp_IPv6FragHeader* ptrFragIPv6Header;

    /* Get the IPv6 Header. */
    ptrFragIPv6Header = (Netfp_IPv6FragHeader*)ptrDataBuffer;

    /* Populate the Fragmentation Header. */
    ptrFragIPv6Header->NextHeader = nextHeader;
    ptrFragIPv6Header->Rsvd       = 0;
    ptrFragIPv6Header->FragOffset = Netfp_htons(fragOffset);
    ptrFragIPv6Header->FragId[0]  = fragId[0];
    ptrFragIPv6Header->FragId[1]  = fragId[1];
    ptrFragIPv6Header->FragId[2]  = fragId[2];
    ptrFragIPv6Header->FragId[3]  = fragId[3];
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to setup the L2 header.
 *
 *  @param[in]  ptrSockTxMetaInfo
 *      Pointer to the NETFP socket transmit meta information
 *  @param[in]  layer3Protocol
 *      Layer3 Protocol to be used
 *  @param[in]  priority
 *      Priority of the packet being sent out.
 *  @param[in]  ptrDataBuffer
 *      Pointer to the L2 Header
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not Applicable.
 */
static void Netfp_setupL2Header
(
    Netfp_SockTxMetaInfo*   ptrSockTxMetaInfo,
    uint16_t                layer3Protocol,
    uint8_t                 priority,
    uint8_t*                ptrDataBuffer
)
{
    Netfp_EthHeader*        ptrEthHeader;
    Netfp_VLANHeader*       ptrVLANHeader;
    int32_t                 index;

    /* Add the L2 Header depending upon the interface properties. */
    if ((ptrSockTxMetaInfo->ptrL2ConnectInfo->vlanId != 0) ||
       ((ptrSockTxMetaInfo->priorityTag & NETFP_VLAN_PRIORITYTAG_ENABLED) == NETFP_VLAN_PRIORITYTAG_ENABLED))
    {
        uint16_t    tci;

        /* Get the pointer to the Ethernet & VLAN Header. */
        ptrEthHeader = (Netfp_EthHeader*)ptrDataBuffer;
        ptrVLANHeader= (Netfp_VLANHeader*)((uint8_t*)ptrEthHeader + ETHHDR_SIZE);

        if( ptrSockTxMetaInfo->ptrL2ConnectInfo->vlanId != 0 )
        {
            /* Compute the TCI value i.e. setup the priority & VLAN identifier. */
            tci = (ptrSockTxMetaInfo->ptrL2ConnectInfo->vlanMap.socketToVLANPriority[ptrSockTxMetaInfo->priority] << 13)
                | (ptrSockTxMetaInfo->ptrL2ConnectInfo->vlanId);
        }
        else
        {
            /* Set only the PCP, keep VLAN ID as zero */
            tci = (ptrSockTxMetaInfo->priorityTag & NETFP_VLAN_PRIORITYTAG_MASK) << 13;
        }

        /* Populate the VLAN header. */
        ptrVLANHeader->tci      = Netfp_htons(tci);
        ptrVLANHeader->protocol = Netfp_htons(layer3Protocol);

        /* Setup the ETH header type to be VLAN */
        ptrEthHeader->Type = Netfp_htons(ETH_VLAN);
    }
    else
    {
        /* Get the pointer to the Ethernet header. */
        ptrEthHeader = (Netfp_EthHeader*)ptrDataBuffer;
        ptrEthHeader->Type = Netfp_htons(layer3Protocol);
    }

    /* Populate the rest of the Ethernet Header: DST & SRC MAC address. */
    for (index = 0; index < 6; index++)
    {
        ptrEthHeader->DstMac[index] = ptrSockTxMetaInfo->ptrL2ConnectInfo->dstMacAddress[index];
        ptrEthHeader->SrcMac[index] = ptrSockTxMetaInfo->ptrL2ConnectInfo->srcMacAddress[index];
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to perform software fragmentation for
 *      IPv4 packets.
 *
 *  @param[in]  ptrNetfpSocket
 *      Socket over which the packet is being sent.
 *  @param[in]  ptrSockTxMetaInfo
 *      Socket Transmit Meta Information
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Netfp_fragment
(
    Netfp_Socket*           ptrNetfpSocket,
    Netfp_SockTxMetaInfo*   ptrSockTxMetaInfo,
    int32_t*                errCode
)
{
    uint32_t                    packetLen;
    uint32_t                    fragmentHeaderSize;
    uint32_t                    fragmentSize;
    uint32_t                    fragmentOffset;
    Ti_Pkt*                     ptrFragment;
    Ti_Pkt*                     ptrFragHeaderPkt;
    Ti_Pkt*                     ptrPkt1;
    Ti_Pkt*                     ptrPkt2;
    int32_t                     retVal;
    uint8_t*                    ptrHeader;
    uint16_t                    ipv4FlagOffset;
    uint16_t                    ipPayloadLength;
    uint16_t                    packetId;
    Netfp_ClientMCB*            ptrClientMCB;
    Pktlib_InstHandle           pktlibInstHandle;
    Netfp_SockL2ConnectInfo*    ptrSockL2ConnectInfo;
    uint32_t                    saPktHdrMargin;
//fzm-->
    uint8_t                     paddingLen;
    uint32_t                    blockSize;
    uint32_t                    encryptedPayloadSize;
    Pktlib_HeapHandle           payloadHeap;
    int32_t                     splitRetVal;
//fzm<--

    *errCode = 0;

    /* All fragments will have the same unique identifier in the IP packet */
    packetId = (uint16_t)Netfp_getUniqueId();

    /* Get the NETFP Client MCB */
    ptrClientMCB = ptrNetfpSocket->ptrNetfpClient;

    /* Get the NETFP Client PKTLIB Instance handle. */
    pktlibInstHandle = ptrClientMCB->cfg.pktlibInstHandle;

    /* Get the NETFP Socket L2 connect information: */
    ptrSockL2ConnectInfo = ptrSockTxMetaInfo->ptrL2ConnectInfo;

    /* Get the total payload length. */
    packetLen = Pktlib_getPacketLen(ptrSockTxMetaInfo->ptrPayload);

    /* Get heap handler to not touch the descriptor after it is pushed to HW */
    payloadHeap = Pktlib_getPktHeap(ptrSockTxMetaInfo->ptrPayload);

    /* The header size passed to the function includes the Layer2 header also;
     * which needs to be discounted here. */
    ptrSockTxMetaInfo->headerSize = ptrSockTxMetaInfo->headerSize - ptrSockL2ConnectInfo->l2HeaderSize;

    /* Fragmentation will be done is software. */
    ptrSockTxMetaInfo->hwFragmentation        = 0;

    /* Hardware UDP checksum can not be supported since fragmentation is done in software */
    ptrSockTxMetaInfo->hwUDPChksum            = 0;

    /* Pass the VLAN priority tag */
    ptrSockTxMetaInfo->priorityTag            = ptrNetfpSocket->priorityTag;

    saPktHdrMargin = ptrNetfpSocket->connectInfo.pktHeaderMargin;

    /* fzm: sa Header Margin is increased by UDPHDR_SIZE if NATT is configured */
    if ((ptrNetfpSocket->connectInfo.nattEncapCfg.srcPort != 0) &&
        (ptrNetfpSocket->connectInfo.nattEncapCfg.dstPort != 0))
           saPktHdrMargin += UDPHDR_SIZE;

    /* Initialize the fragmentation offset */
    fragmentOffset = 0;

    blockSize = ptrSockTxMetaInfo->ptrL2ConnectInfo->saGeneralChannelCtrlInfo.txCtrl.params.ipsec.encryptionBlockSize; //fzm
    /* Loop around till the entire payload packet is fragmented & sent out. */
    while (1)
    {
        /* fzm--> */
        /* Determine the MAX size of each IP fragment. Fragment Sizes
         * are a multiple of 8 octets. Take this into account while computing the
         * size of each fragment. */
        if (ptrNetfpSocket->connectInfo.isSecure == 1)
        {
            //First, determine the largest possible IP payload size without rounding or padding
            fragmentSize = ptrSockL2ConnectInfo->mtu - ptrSockTxMetaInfo->headerSize - ptrNetfpSocket->connectInfo.pktTrailerMargin;

            //Round the IP payload down to a multiple of 8 bytes
            fragmentSize = (fragmentSize & ~0x7);

            //Determine the encrypted payload length without padding for that calculated size
            encryptedPayloadSize = fragmentSize + ptrSockTxMetaInfo->innerHeaderSize + IPSEC_ESP_TAIL_SIZE_BYTES;

            //Determine how much padding would be required for that payload
            paddingLen = blockSize - (encryptedPayloadSize % blockSize);

            //if, with the padding, the packet now exceeds MTU, Reduce the fragment by 1 block size
            if (ptrSockL2ConnectInfo->mtu < (fragmentSize + ptrSockTxMetaInfo->headerSize + ptrNetfpSocket->connectInfo.pktTrailerMargin + paddingLen))
            {
                 fragmentSize -= blockSize;
                 //round down to multiple of 8 again if necessary
                 fragmentSize &= ~0x7;
            }
            if (fragmentSize > packetLen)
                fragmentSize = packetLen;
        }
        else
        {
            fragmentSize = ptrSockL2ConnectInfo->mtu - ptrSockTxMetaInfo->headerSize;
            if (fragmentSize > packetLen)
                fragmentSize = packetLen;
            else
                fragmentSize &= ~0x7;
        }
        /* fzm<-- */

        /* Allocate a buffer-less packet so that we can split the packet. */
        ptrFragment = Pktlib_allocPacketPrefetch(ptrClientMCB->cfg.fragmentHeap, 0); //fzm

        if (unlikely(ptrFragment == NULL))
        {
            *errCode = NETFP_ENOMEM;
            break;
        }

        /* Allocate a header packet which carries all the headers including the Layer2 Header. */
        ptrFragHeaderPkt = Pktlib_allocPacketPrefetch(ptrClientMCB->cfg.netHeaderHeapHandle,
                                              (ptrSockTxMetaInfo->headerSize + ptrSockL2ConnectInfo->l2HeaderSize)); //fzm

        if (unlikely(ptrFragHeaderPkt == NULL))
        {
            Pktlib_freePacket(pktlibInstHandle, ptrFragment);
            *errCode = NETFP_ENOMEM;
            break;
        }

        /* Each fragment which is created will have additional headers (layer3 & layer2) added to it. */
        Pktlib_getDataBuffer(ptrFragHeaderPkt, &ptrHeader, &fragmentHeaderSize);

        /* Packet was split successfully. Is this the first fragment? */
        if (fragmentOffset == 0)
        {
            /* Special Case: First Fragment.
             *  - This will carry the GTPU Header if the socket is configured to do so.
             *  - Always carry the UDP Header. The UDP Header carries the total payload length.
             *    This is NOT the length of each fragment but the actual total length of the full data
             *    payload. */
            if (unlikely(ptrSockTxMetaInfo->gtpuHeaderSize != 0))
            {
                /* Populate the GTPU Header. */
                Netfp_setupGTPUHeader(ptrNetfpSocket,
                                      (uint8_t*)ptrHeader + ptrSockL2ConnectInfo->l2HeaderSize +
                                      ptrSockTxMetaInfo->headerSize - ptrSockTxMetaInfo->gtpuHeaderSize,
                                      packetLen);

                /* Once the GTPU Header has been added increment the packet to account for the UDP Header */
                ptrSockTxMetaInfo->packetSize = ptrSockTxMetaInfo->packetSize + GTPHDR_SIZE;

                /* Increment the statistics */
                ptrNetfpSocket->extendedStats.numGTPUTxPkts++;
            }

            /* Setup the UDP Header. */
            Netfp_setupUDPHeader(ptrNetfpSocket, ptrSockTxMetaInfo,
                                 (uint8_t*)ptrHeader + ptrSockL2ConnectInfo->l2HeaderSize +
                                 ptrSockTxMetaInfo->headerSize - ptrSockTxMetaInfo->gtpuHeaderSize - UDPHDR_SIZE,
                                 packetLen + ptrSockTxMetaInfo->gtpuHeaderSize);

            /* Once the UDP Header has been added increment the packet to account for the UDP Header */
            ptrSockTxMetaInfo->packetSize = ptrSockTxMetaInfo->packetSize + UDPHDR_SIZE;

            /* Increment the statistics */
            ptrNetfpSocket->extendedStats.numUDPTxPkts++;

            /* The IP Payload Length accounts for the UDP Header also in this case. */
            ipPayloadLength = fragmentSize + UDPHDR_SIZE + ptrSockTxMetaInfo->gtpuHeaderSize;
        }
        else
        {
            /* All other fragments dont have any additional Layer4 headers */
            ipPayloadLength = fragmentSize;
        }

        /* Split the payload packet into the fragment size */
        splitRetVal = Pktlib_splitPacket (pktlibInstHandle, ptrSockTxMetaInfo->ptrPayload, ptrFragment, fragmentSize, &ptrPkt1, &ptrPkt2);

        if (splitRetVal == 1)
        {
            Pktlib_freePacket(pktlibInstHandle, ptrFragment);
        }

        if (unlikely(splitRetVal < 0))
        {
            // Locally new allocated packets not merged yet are not used at all, free them
            Pktlib_freePacket(pktlibInstHandle, ptrFragment);
            Pktlib_freePacket(pktlibInstHandle, ptrFragHeaderPkt);

            *errCode = NETFP_EFRAGFAIL;
            break;
        }

        /* Are there more fragments or not? */
        packetLen = packetLen - fragmentSize;
        if (packetLen > 0)
            ipv4FlagOffset = 0x2000;
        else
            ipv4FlagOffset = 0;

        /* Setup the IP Fragmentation. */
        ipv4FlagOffset = ipv4FlagOffset | (fragmentOffset >> 3);

        /* Is this a secure socket? If so we need to add the inner and outer IP header on the packet. */
        if (ptrNetfpSocket->connectInfo.isSecure == 1)
        {
            /* We now need to add the OUTER IP header: This could be IPv4 or IPv6. */
            if (Netfp_isAddressIPv4(ptrNetfpSocket->connectInfo.outerIPSrc[ptrNetfpSocket->connectInfo.cfgIndex]) == 1)
            {
                /* Outer IPv4 Header + Inner IPv4 Header: Add the inner IPv4 header */
                Netfp_setupIPHeader(ptrNetfpSocket, (uint8_t*)ptrHeader + ptrSockL2ConnectInfo->l2HeaderSize + IPHDR_SIZE + saPktHdrMargin,
                                    &ptrNetfpSocket->bindInfo.innerIPSrc.addr.ipv4.u.a8[0],
                                    &ptrNetfpSocket->connectInfo.innerIPDst.addr.ipv4.u.a8[0],
                                    IPPROTO_UDP, packetId, ptrSockTxMetaInfo->innerDSCP,
                                    ipv4FlagOffset, ipPayloadLength);

                /* Populate the Outer IPv4 Header. */
                Netfp_setupIPHeader(ptrNetfpSocket, (uint8_t*)ptrHeader + ptrSockL2ConnectInfo->l2HeaderSize + saPktHdrMargin,
                                    &ptrNetfpSocket->connectInfo.outerIPSrc[ptrNetfpSocket->connectInfo.cfgIndex].addr.ipv4.u.a8[0],
                                    &ptrNetfpSocket->connectInfo.outerIPDst[ptrNetfpSocket->connectInfo.cfgIndex].addr.ipv4.u.a8[0],
                                    IPPROTO_IPIP, (uint16_t)Netfp_getUniqueId(),
                                    ptrSockTxMetaInfo->outerDSCP, 0, ipPayloadLength + IPHDR_SIZE);

                /* Setup the L2 Header. */
                Netfp_setupL2Header(ptrSockTxMetaInfo, ETH_IP, ptrSockTxMetaInfo->priority,
                                    (uint8_t*)ptrHeader + saPktHdrMargin);

                /* Increment the 4in4 statistics */
                ptrNetfpSocket->extendedStats.numTxPkts4in4Tunnel++;
            }
            else
            {
                /* Outer IPv6 Header + Inner IPv4 Header: Add the inner IPv4 header */
                Netfp_setupIPHeader(ptrNetfpSocket, (uint8_t*)ptrHeader + ptrSockL2ConnectInfo->l2HeaderSize + IPv6HDR_SIZE + saPktHdrMargin,
                                    &ptrNetfpSocket->bindInfo.innerIPSrc.addr.ipv4.u.a8[0],
                                    &ptrNetfpSocket->connectInfo.innerIPDst.addr.ipv4.u.a8[0],
                                    IPPROTO_UDP, packetId, ptrSockTxMetaInfo->innerDSCP, ipv4FlagOffset, ipPayloadLength);

                /* Populate the Outer IPv6 Header */
                Netfp_setupIPv6Header((uint8_t*)ptrHeader + saPktHdrMargin +
                                      ptrSockL2ConnectInfo->l2HeaderSize,
                                      &ptrNetfpSocket->connectInfo.outerIPSrc[ptrNetfpSocket->connectInfo.cfgIndex].addr.ipv6.u.a8[0],
                                      &ptrNetfpSocket->connectInfo.outerIPDst[ptrNetfpSocket->connectInfo.cfgIndex].addr.ipv6.u.a8[0],
                                      IPPROTO_IPIP, ptrSockTxMetaInfo->outerDSCP, (ipPayloadLength + IPHDR_SIZE));

                /* Setup the L2 Header. */
                Netfp_setupL2Header(ptrSockTxMetaInfo, ETH_IP6, ptrSockTxMetaInfo->priority, (uint8_t*)ptrHeader + saPktHdrMargin);

                /* Increment the 4in6 statistics */
                ptrNetfpSocket->extendedStats.numTxPkts4in6Tunnel++;
            }
        }
        else
        {
            /* Non Secure Socket: Setup the Outer IP Header. */
            Netfp_setupIPHeader(ptrNetfpSocket, (uint8_t*)ptrHeader + ptrSockL2ConnectInfo->l2HeaderSize,
                                &ptrNetfpSocket->bindInfo.innerIPSrc.addr.ipv4.u.a8[0],
                                &ptrNetfpSocket->connectInfo.innerIPDst.addr.ipv4.u.a8[0],
                                IPPROTO_UDP, packetId, ptrSockTxMetaInfo->outerDSCP,
                                ipv4FlagOffset, ipPayloadLength);

            /* Setup the L2 Header. */
            Netfp_setupL2Header(ptrSockTxMetaInfo, ETH_IP, ptrSockTxMetaInfo->priority, (uint8_t*)ptrHeader);
        }

        /* Cache Hooks: The networking headers (L2, L3 and L4) have all been populated and the header
         * needs to be written back. No more changes are done to the networking headers after this point
         * Descriptors ownership will be released just before the CPDMA push. */
        ptrNetfpSocket->ptrNetfpClient->cfg.endMemAccess(ptrHeader, fragmentHeaderSize);

        /* Link the header packet to the 'split' packet. */
        Pktlib_packetMerge(pktlibInstHandle, ptrFragHeaderPkt, ptrPkt1, NULL);

        /* Increment the statistics for the number of IPv4 fragments which have been created. */
        ptrNetfpSocket->extendedStats.numIPv4Frags++;

        /* Send the packet on the specified interface.
         *  - Software fragmentation has already been done. Bypass this in the NETCP */
        retVal = Netfp_transmitInterface(ptrNetfpSocket, ptrSockTxMetaInfo, ptrFragHeaderPkt);

        if (unlikely(retVal < 0))
        {
            /* 1) Kill the next link in the fragment header packet to not free main descriptor passed to Netfp_fragment() */
            Cppi_linkNextBD (Cppi_DescType_HOST, (Cppi_Desc*)ptrFragHeaderPkt, (Cppi_Desc*)NULL);

            /* 2) Free allocated fragment header */
            Pktlib_freePacket(pktlibInstHandle, ptrFragHeaderPkt);

            /* 3) Free the packet if was not freed yet as split()'s step */
            if (splitRetVal != 1)
                Pktlib_freePacket(pktlibInstHandle, ptrFragment);

            /* Because errCode may contain failures from different level of transmitting,
               set EINTERNAL as recognizable critical error when not ENOMEM type */
            if (retVal == NETFP_ENOMEM)
                *errCode = NETFP_ENOMEM;
            else
                *errCode = NETFP_EINTERNAL;

            break;
        }

        /* Are we done sending all the packets. */
        if ((int32_t)packetLen <= 0)
            break;

        /* Is this a secure socket? */
        if (ptrNetfpSocket->connectInfo.isSecure == 1)
        {
            /* Secure Socket: Check if the Outer IP is IPv4 or IPv6? */
            if (Netfp_isAddressIPv4(ptrNetfpSocket->connectInfo.outerIPSrc[ptrNetfpSocket->connectInfo.cfgIndex]) == 1)
            {
                /* Secure Socket: Inner IPv4 + Outer IPv4 + ESP Header */
                ptrSockTxMetaInfo->headerSize = IPHDR_SIZE + IPHDR_SIZE + saPktHdrMargin;
            }
            else
            {
                /* Secure Socket: Inner IPv4 + Outer IPv6 + ESP Header */
                ptrSockTxMetaInfo->headerSize = IPHDR_SIZE + IPv6HDR_SIZE + saPktHdrMargin;
            }
            /* fzm: UDP and GTP headers are just once per packet, not per fragment.  As a result
             * After the first fragment, the inner header is only the IP header for the remaining
             * fragments
             */
            ptrSockTxMetaInfo->innerHeaderSize = IPHDR_SIZE;
        }
        else
        {
            /* NO. All fragments will only have the IP Header added to it. */
            ptrSockTxMetaInfo->headerSize = IPHDR_SIZE;
        }

        /* Increment the fragmentation offset to account for the number of bytes sent */
        fragmentOffset = fragmentOffset + ipPayloadLength;

        /* The new payload packet is now what is left */
        ptrSockTxMetaInfo->ptrPayload = ptrPkt2;
    }

    /* Run Garbage Collection on the following heaps.
     *  - The fragment heap has zero buffer descriptors which after the CPDMA transmission
     *    will always get recycled into the garbage queue.
     *  - The payload packet has been split and referenced; so this packet will also end
     *    up in the garbage queue. */
    Pktlib_garbageCollection (pktlibInstHandle, ptrNetfpSocket->ptrNetfpClient->cfg.fragmentHeap);
    Pktlib_garbageCollection (pktlibInstHandle, payloadHeap);

    return (*errCode) ? -1 : 0;
}

static int32_t Netfp_fragment_FZM
(
    Netfp_Socket*           ptrNetfpSocket,
    Netfp_SockTxMetaInfo_FZM*   sockTxMetaInfo,
    int32_t*                errCode
)
{
    uint32_t                    packetLen;
    uint32_t                    fragmentHeaderSize;
    uint32_t                    fragmentSize;
    uint32_t                    fragmentOffset;
    Ti_Pkt*                     ptrFragment;
    Ti_Pkt*                     ptrFragHeaderPkt;
    Ti_Pkt*                     ptrPkt1;
    Ti_Pkt*                     ptrPkt2;
    int32_t                     retVal;
    uint16_t                    ipv4FlagOffset;
    uint16_t                    ipPayloadLength;
    uint16_t                    packetId;
    Netfp_ClientMCB*            ptrClientMCB;
    Pktlib_InstHandle           pktlibInstHandle;
    Pktlib_HeapHandle           payloadHeap;
    int32_t                     splitRetVal;

    *errCode = 0;

    ptrClientMCB = ptrNetfpSocket->ptrNetfpClient;
    pktlibInstHandle = ptrClientMCB->cfg.pktlibInstHandle;

    /* All fragments will have the same unique identifier in the IP packet */
    packetId = (uint16_t)Netfp_getUniqueId();

    /* Get the total payload length. */
    packetLen = Pktlib_getPacketLen(sockTxMetaInfo->ptrPayload);

    /* Get heap handler to not touch the descriptor after it is pushed to HW */
    payloadHeap = Pktlib_getPktHeap(sockTxMetaInfo->ptrPayload);

    /* The header size passed to the function includes the Layer2 header also;
     * which needs to be discounted here. */
    uint32_t headerSize = sockTxMetaInfo->headerSize - sockTxMetaInfo->l2ConnectInfo->l2HeaderSize;

    /* Initialize the fragmentation offset */
    fragmentOffset = 0;

    /* Loop around till the entire payload packet is fragmented & sent out. */
    while (1)
    {
        /* fzm--> */
        /* Determine the MAX size of each IP fragment. Fragment Sizes
         * are a multiple of 8 octets. Take this into account while computing the
         * size of each fragment. */
        fragmentSize = sockTxMetaInfo->l2ConnectInfo->mtu - headerSize;
        if (fragmentSize > packetLen)
            fragmentSize = packetLen;
        else
            fragmentSize &= ~0x7;
        /* fzm<-- */

        /* Allocate a header packet which carries all the headers including the Layer2 Header. */
        ptrFragHeaderPkt = Pktlib_allocPacketPrefetch(ptrClientMCB->cfg.netHeaderHeapHandle, headerSize + sockTxMetaInfo->l2ConnectInfo->l2HeaderSize);
        if (unlikely(ptrFragHeaderPkt == NULL))
        {
            Pktlib_freePacket(pktlibInstHandle, ptrFragment);
            *errCode = NETFP_ENOMEM;
            break;
        }

        /* Are there more fragments or not? */
        packetLen = packetLen - fragmentSize;
        if (packetLen > 0)
            ipv4FlagOffset = 0x2000;
        else
            ipv4FlagOffset = 0;

        /* Each fragment which is created will have additional headers (layer3 & layer2) added to it. */
        Pktlib_getDataBuffer(ptrFragHeaderPkt, &sockTxMetaInfo->ptrHeaderBuffer, &fragmentHeaderSize);

        /* Is this the first fragment? */
        if (fragmentOffset == 0)
        {
            /* Special Case: First Fragment. */
            NetfpNetHeaders* netHeadersPtr = &ptrNetfpSocket->netHeaders[sockTxMetaInfo->l2CfgIndex];
            memcpy(sockTxMetaInfo->ptrHeaderBuffer, netHeadersPtr->headerBuff, netHeadersPtr->headerSize);
            updateNetHeaderPktIPv4(ptrNetfpSocket, sockTxMetaInfo, packetId, 1, ipv4FlagOffset);

            /* Increment the statistics */
            ptrNetfpSocket->extendedStats.numUDPTxPkts++;

            /* The IP Payload Length accounts for the UDP Header also in this case. */
            ipPayloadLength = fragmentSize + UDPHDR_SIZE;
        }
        else if (fragmentOffset > 0)
        {
            /* Setup the IP Fragmentation. */
            ipv4FlagOffset = ipv4FlagOffset | (fragmentOffset >> 3);

            NetfpNetHeaders* netHeadersPtr = &ptrNetfpSocket->netHeaders[sockTxMetaInfo->l2CfgIndex];
            memcpy(sockTxMetaInfo->ptrHeaderBuffer, netHeadersPtr->headerBuff, netHeadersPtr->headerSize - UDPHDR_SIZE);
            updateNetHeaderPktIPv4(ptrNetfpSocket, sockTxMetaInfo, packetId, 0, ipv4FlagOffset);

            /* All other fragments dont have any additional Layer4 headers */
            ipPayloadLength = fragmentSize;
        }

        /* Allocate a buffer-less packet so that we can split the packet. */
        ptrFragment = Pktlib_allocPacketPrefetch(ptrClientMCB->cfg.fragmentHeap, 0); //fzm
        if (unlikely(ptrFragment == NULL))
        {
            *errCode = NETFP_ENOMEM;
            break;
        }

        /* Split the payload packet into the fragment size */
        splitRetVal = Pktlib_splitPacket (pktlibInstHandle, sockTxMetaInfo->ptrPayload, ptrFragment, fragmentSize, &ptrPkt1, &ptrPkt2);

        if (splitRetVal == 1)
        {
            Pktlib_freePacket(pktlibInstHandle, ptrFragment);
        }

        if (unlikely(splitRetVal < 0))
        {
            Pktlib_freePacket(pktlibInstHandle, ptrFragment);
            Pktlib_freePacket(pktlibInstHandle, ptrFragHeaderPkt);

            *errCode = NETFP_EFRAGFAIL;
            break;
        }

        /* Cache Hooks: The networking headers (L2, L3 and L4) have all been populated and the header
         * needs to be written back. No more changes are done to the networking headers after this point
         * Descriptors ownership will be released just before the CPDMA push. */
        ptrNetfpSocket->ptrNetfpClient->cfg.endMemAccess(sockTxMetaInfo->ptrHeaderBuffer, fragmentHeaderSize);

        /* Link the header packet to the 'split' packet. */
        Pktlib_packetMerge(pktlibInstHandle, ptrFragHeaderPkt, ptrPkt1, NULL);

        /* Increment the statistics for the number of IPv4 fragments which have been created. */
        ptrNetfpSocket->extendedStats.numIPv4Frags++;

        /* Send the packet on the specified interface.
         *  - Software fragmentation has already been done. Bypass this in the NETCP */
        retVal = Netfp_transmitInterface_FZM(ptrNetfpSocket, sockTxMetaInfo, ptrFragHeaderPkt);

        if (unlikely(retVal < 0))
        {
            /* 1) Kill the next link in the fragment header packet to not free main descriptor passed to Netfp_fragment() */
            Cppi_linkNextBD (Cppi_DescType_HOST, (Cppi_Desc*)ptrFragHeaderPkt, (Cppi_Desc*)NULL);

            /* 2) Free allocated fragment header */
            Pktlib_freePacket(pktlibInstHandle, ptrFragHeaderPkt);

            /* 3) Free the packet if was not freed yet as split()'s step */
            if (splitRetVal != 1)
                Pktlib_freePacket(pktlibInstHandle, ptrFragment);

            /* Because errCode may contain failures from different level of transmitting,
               set EINTERNAL as recognizable critical error when not ENOMEM type */
            if (retVal == NETFP_ENOMEM)
                *errCode = NETFP_ENOMEM;
            else
                *errCode = NETFP_EINTERNAL;

            break;
        }

        /* Are we done sending all the packets. */
        if ((int32_t)packetLen <= 0)
            break;

        /* Is this a secure socket? */
        /* NO. All fragments will only have the IP Header added to it. */
        headerSize = IPHDR_SIZE;

        /* Increment the fragmentation offset to account for the number of bytes sent */
        fragmentOffset = fragmentOffset + ipPayloadLength;

        /* The new payload packet is now what is left */
        sockTxMetaInfo->ptrPayload = ptrPkt2;
    }

    /* Run Garbage Collection on the following heaps.
     *  - The fragment heap has zero buffer descriptors which after the CPDMA transmission
     *    will always get recycled into the garbage queue.
     *  - The payload packet has been split and referenced; so this packet will also end
     *    up in the garbage queue. */
    Pktlib_garbageCollection (pktlibInstHandle, ptrNetfpSocket->ptrNetfpClient->cfg.fragmentHeap);
    Pktlib_garbageCollection (pktlibInstHandle, payloadHeap);

    return (*errCode) ? -1 : 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to fragment and send out IPv6 packets.
 *
 *  @param[in]  ptrNetfpSocket
 *      Socket over which the packet is being sent out.
 *  @param[in]  ptrSockTxMetaInfo
 *      Pointer to the socket transmit meta Information.
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Netfp_fragment6
(
    Netfp_Socket*           ptrNetfpSocket,
    Netfp_SockTxMetaInfo*   ptrSockTxMetaInfo,
    int32_t*                errCode
)
{
    uint32_t                    packetLen;
    uint32_t                    fragmentHeaderSize;
    uint32_t                    fragmentSize;
    uint32_t                    fragmentOffset;
    Ti_Pkt*                     ptrFragment;
    Ti_Pkt*                     ptrFragHeaderPkt;
    Ti_Pkt*                     ptrPkt1;
    Ti_Pkt*                     ptrPkt2;
    int32_t                     retVal;
    uint8_t*                    ptrHeader;
    uint16_t                    ipv6FlagOffset;
    uint16_t                    ipPayloadLength;
    uint32_t                    packetId;
    Netfp_ClientMCB*            ptrClientMCB;
    Pktlib_InstHandle           pktlibInstHandle;
    Netfp_SockL2ConnectInfo*    ptrSockL2ConnectInfo;
    uint32_t                    saPktHdrMargin;
//fzm-->
    uint8_t                     paddingLen;
    uint32_t                    blockSize;
    uint32_t                    encryptedPayloadSize;
    int32_t                     splitRetVal;
//fzm<--

    *errCode = 0;

    /* All fragments will have the same unique identifier in the IP packet */
    packetId = Netfp_getUniqueId();

    /* Get the NETFP Client MCB */
    ptrClientMCB = ptrNetfpSocket->ptrNetfpClient;

    /* Get the NETFP Socket L2 connect information: */
    ptrSockL2ConnectInfo = ptrSockTxMetaInfo->ptrL2ConnectInfo;

    /* Get the NETFP Client PKTLIB Instance handle. */
    pktlibInstHandle = ptrClientMCB->cfg.pktlibInstHandle;

    /* We start with the first fragment */
    fragmentOffset = 0;

    /* Get the total payload length. */
    packetLen = Pktlib_getPacketLen(ptrSockTxMetaInfo->ptrPayload);

    /* The header size passed to the function includes the Layer2 header also;
     * which needs to be discounted here. */
    ptrSockTxMetaInfo->headerSize = ptrSockTxMetaInfo->headerSize - ptrSockL2ConnectInfo->l2HeaderSize;

    /* Fragmentation will be done is software. */
    ptrSockTxMetaInfo->hwFragmentation        = 0;

    /* Hardware UDP checksum can not be supported since fragmentation is done in software */
    ptrSockTxMetaInfo->hwUDPChksum            = 0;

    /* Pass the VLAN priority tag */
    ptrSockTxMetaInfo->priorityTag            = ptrNetfpSocket->priorityTag;

    saPktHdrMargin = ptrNetfpSocket->connectInfo.pktHeaderMargin;

    /* fzm: sa Header Margin is increased by UDPHDR_SIZE if NATT is configured.
     * NATT is not currently supported for IPv6, but we put this here to prevent possible
     * future bugs
     */
    if ((ptrNetfpSocket->connectInfo.nattEncapCfg.srcPort != 0) &&
        (ptrNetfpSocket->connectInfo.nattEncapCfg.dstPort != 0))
           saPktHdrMargin += UDPHDR_SIZE;

    blockSize = ptrSockTxMetaInfo->ptrL2ConnectInfo->saGeneralChannelCtrlInfo.txCtrl.params.ipsec.encryptionBlockSize; //fzm

    /* Loop around till the entire payload packet is fragmented & sent out. */
    while (1)
    {
        /* fzm--> */
        /* Determine the MAX size of each IP fragment. Fragment Sizes
         * are a multiple of 8 octets. Take this into account while computing the
         * size of each fragment. */
        if (ptrNetfpSocket->connectInfo.isSecure == 1)
        {
            //First, determine the largest possible IP payload size without rounding or padding
            fragmentSize = ptrSockL2ConnectInfo->mtu - ptrSockTxMetaInfo->headerSize - ptrNetfpSocket->connectInfo.pktTrailerMargin - IPV6_FRAGHDR_SIZE;

            //Round the IP payload down to a multiple of 8 bytes
            fragmentSize = (fragmentSize & ~0x7);

            //Determine the encrypted payload length without padding for that calculated size
            encryptedPayloadSize = fragmentSize + ptrSockTxMetaInfo->innerHeaderSize + IPSEC_ESP_TAIL_SIZE_BYTES + IPV6_FRAGHDR_SIZE;

            //Determine how much padding would be required for that payload
            paddingLen = blockSize - (encryptedPayloadSize % blockSize);

            //if, with the padding, the packet now exceeds MTU, Reduce the fragment by 1 block size
            if (ptrSockL2ConnectInfo->mtu < (fragmentSize + ptrSockTxMetaInfo->headerSize + ptrNetfpSocket->connectInfo.pktTrailerMargin + paddingLen + IPV6_FRAGHDR_SIZE))
            {
                 fragmentSize -= blockSize;
                 //round down to multiple of 8 again if necessary
                 fragmentSize &= ~0x7;
            }
            if (fragmentSize > packetLen)
                fragmentSize = packetLen;
        }
        else
        {
            fragmentSize = ptrSockL2ConnectInfo->mtu - ptrSockTxMetaInfo->headerSize - IPV6_FRAGHDR_SIZE;
            if (fragmentSize > packetLen)
                fragmentSize = packetLen;
            else
                fragmentSize &= ~0x7;
        }
        /* fzm<-- */

        /* Allocate a buffer-less packet so that we can split the packet. */
        ptrFragment = Pktlib_allocPacketPrefetch(ptrClientMCB->cfg.fragmentHeap, 0);
        if (unlikely(ptrFragment == NULL))
        {
            *errCode = NETFP_ENOMEM;
            break;
        }

        /* Allocate a header packet which carries all the standard networking headers (including the IPv6
         * fragment header) */
        ptrFragHeaderPkt = Pktlib_allocPacketPrefetch(ptrClientMCB->cfg.netHeaderHeapHandle,
                                              (ptrSockL2ConnectInfo->l2HeaderSize + ptrSockTxMetaInfo->headerSize +
                                               IPV6_FRAGHDR_SIZE)); //fzm
        if (unlikely(ptrFragHeaderPkt == NULL))
        {
            Pktlib_freePacket(pktlibInstHandle, ptrFragment);
            *errCode = NETFP_ENOMEM;
            break;
        }

        /* Each Fragment which is created will have additional headers (layer3 & layer2) added to it. */
        Pktlib_getDataBuffer(ptrFragHeaderPkt, &ptrHeader, &fragmentHeaderSize);

        /* Packet was split successfully. Is this the first fragment? */
        if (fragmentOffset == 0)
        {
            /* Special Case: First Fragment.
             *  - This will carry the GTPU Header if the socket is configured to do so.
             *  - Always carry the UDP Header. The UDP Header carries the total payload length.
             *    This is NOT the length of each fragment but the actual total length of the full data
             *    payload. */
            if (unlikely(ptrSockTxMetaInfo->gtpuHeaderSize != 0))
            {
                /* Populate the GTPU Header. */
                Netfp_setupGTPUHeader(ptrNetfpSocket,
                                      (uint8_t*)ptrHeader + ptrSockL2ConnectInfo->l2HeaderSize +
                                      IPV6_FRAGHDR_SIZE + ptrSockTxMetaInfo->headerSize -
                                      ptrSockTxMetaInfo->gtpuHeaderSize,
                                      packetLen);

                /* Once the GTPU Header has been added increment the packet to account for the UDP Header */
                ptrSockTxMetaInfo->packetSize = ptrSockTxMetaInfo->packetSize + GTPHDR_SIZE;

                /* Increment the statistics */
                ptrNetfpSocket->extendedStats.numGTPUTxPkts++;
            }

            /* Setup the UDP Header.
             *  - UDP Checksum is being reused for software fragmentation. */
            Netfp_setupUDPHeader(ptrNetfpSocket, ptrSockTxMetaInfo,
                                 (uint8_t*)ptrHeader + ptrSockL2ConnectInfo->l2HeaderSize +
                                 ptrSockTxMetaInfo->headerSize + IPV6_FRAGHDR_SIZE -
                                 ptrSockTxMetaInfo->gtpuHeaderSize -UDPHDR_SIZE,
                                 packetLen + ptrSockTxMetaInfo->gtpuHeaderSize);

            /* Once the UDP Header has been added increment the packet to account for the UDP Header */
            ptrSockTxMetaInfo->packetSize = ptrSockTxMetaInfo->packetSize + UDPHDR_SIZE;

            /* Increment the statistics */
            ptrNetfpSocket->extendedStats.numUDPTxPkts++;

            /* Setup the IP Payload length to account for the UDP & GTPU Header and the IPv6 Fragment header. */
            ipPayloadLength = fragmentSize + UDPHDR_SIZE + ptrSockTxMetaInfo->gtpuHeaderSize + IPV6_FRAGHDR_SIZE;
        }
        else
        {
            /* Setup the IP Payload length to account for only the fragment + IPv6 Fragment Header */
            ipPayloadLength = fragmentSize + IPV6_FRAGHDR_SIZE;
        }

        /* Split the payload packet into the fragment size */
        splitRetVal = Pktlib_splitPacket (pktlibInstHandle, ptrSockTxMetaInfo->ptrPayload, ptrFragment, fragmentSize, &ptrPkt1, &ptrPkt2);
        if (splitRetVal == 1)
            Pktlib_freePacket(pktlibInstHandle, ptrFragment);

        if (unlikely(splitRetVal < 0))
        {
            Pktlib_freePacket(pktlibInstHandle, ptrFragment);
            Pktlib_freePacket(pktlibInstHandle, ptrFragHeaderPkt);

            *errCode = NETFP_EFRAGFAIL;
            break;
        }

        /* Are there more fragments or not? */
        packetLen = packetLen - fragmentSize;
        if ((int32_t)packetLen <= 0)
            ipv6FlagOffset = 0x0;
        else
            ipv6FlagOffset = 0x1;

        /* Setup the IP Fragmentation. */
        ipv6FlagOffset = ipv6FlagOffset | fragmentOffset;

        /* Is this a secure socket?  */
        if (ptrNetfpSocket->connectInfo.isSecure == 1)
        {
            /* Secure Socket: Check if the Outer IP is IPv4 or IPv6? */
            if (Netfp_isAddressIPv4(ptrNetfpSocket->connectInfo.outerIPSrc[ptrNetfpSocket->connectInfo.cfgIndex]) == 1)
            {
                /* Outer IPv4 Header + Inner IPv6 Header: */
                Netfp_setupIPv6FragHeader((uint8_t*)ptrHeader + saPktHdrMargin + IPv6HDR_SIZE + IPHDR_SIZE +
                                          ptrSockL2ConnectInfo->l2HeaderSize,
                                          IPPROTO_UDP, ipv6FlagOffset, (uint8_t*)&packetId);

                /* Populate the Inner IPv6 Header */
                Netfp_setupIPv6Header((uint8_t*)ptrHeader + saPktHdrMargin + IPHDR_SIZE +
                                      ptrSockL2ConnectInfo->l2HeaderSize,
                                      &ptrNetfpSocket->bindInfo.innerIPSrc.addr.ipv6.u.a8[0],
                                      &ptrNetfpSocket->connectInfo.innerIPDst.addr.ipv6.u.a8[0],
                                      IPPROTO6_FRAG, ptrSockTxMetaInfo->innerDSCP, ipPayloadLength);

                /* Populate the Outer IPv4 Header */
                Netfp_setupIPHeader(ptrNetfpSocket, (uint8_t*)ptrHeader + saPktHdrMargin + ptrSockL2ConnectInfo->l2HeaderSize,
                                    &ptrNetfpSocket->connectInfo.outerIPSrc[ptrNetfpSocket->connectInfo.cfgIndex].addr.ipv4.u.a8[0],
                                    &ptrNetfpSocket->connectInfo.outerIPDst[ptrNetfpSocket->connectInfo.cfgIndex].addr.ipv4.u.a8[0],
                                    IPPROTO6_ESP, (uint16_t)Netfp_getUniqueId(), ptrSockTxMetaInfo->outerDSCP,
                                    0, (ipPayloadLength + IPv6HDR_SIZE));

                /* Setup the L2 Header. */
                Netfp_setupL2Header(ptrSockTxMetaInfo, ETH_IP, ptrSockTxMetaInfo->priority, (uint8_t*)ptrHeader + saPktHdrMargin);

                /* Increment the 6in4 statistics */
                ptrNetfpSocket->extendedStats.numTxPkts6in4Tunnel++;
            }
            else
            {
                /* Outer IPv6 Header + Inner IPv6 Header: */
                Netfp_setupIPv6FragHeader((uint8_t*)ptrHeader + saPktHdrMargin + IPv6HDR_SIZE + IPv6HDR_SIZE +
                                          ptrSockL2ConnectInfo->l2HeaderSize,
                                          IPPROTO_UDP, ipv6FlagOffset, (uint8_t*)&packetId);

                /* Populate the Inner IPv6 Header */
                Netfp_setupIPv6Header((uint8_t*)ptrHeader + saPktHdrMargin + IPv6HDR_SIZE +
                                      ptrSockL2ConnectInfo->l2HeaderSize,
                                      &ptrNetfpSocket->bindInfo.innerIPSrc.addr.ipv6.u.a8[0],
                                      &ptrNetfpSocket->connectInfo.innerIPDst.addr.ipv6.u.a8[0],
                                      IPPROTO6_FRAG, ptrSockTxMetaInfo->innerDSCP, ipPayloadLength);

                /* Populate the Outer IPv6 Header */
                Netfp_setupIPv6Header((uint8_t*)ptrHeader + saPktHdrMargin +
                                      ptrSockL2ConnectInfo->l2HeaderSize,
                                      &ptrNetfpSocket->connectInfo.outerIPSrc[ptrNetfpSocket->connectInfo.cfgIndex].addr.ipv6.u.a8[0],
                                      &ptrNetfpSocket->connectInfo.outerIPDst[ptrNetfpSocket->connectInfo.cfgIndex].addr.ipv6.u.a8[0],
                                      IPPROTO6_ESP, ptrSockTxMetaInfo->outerDSCP, (ipPayloadLength + IPv6HDR_SIZE));

                /* Setup the L2 Header. */
                Netfp_setupL2Header(ptrSockTxMetaInfo, ETH_IP6, ptrSockTxMetaInfo->priority, (uint8_t*)ptrHeader + saPktHdrMargin);

                /* Increment the 6in6 statistics */
                ptrNetfpSocket->extendedStats.numTxPkts6in6Tunnel++;
            }
        }
        else
        {
            /* Non Secure Socket: Setup the IPv6 Fragmentation Header. */
            Netfp_setupIPv6FragHeader((uint8_t*)ptrHeader + ptrSockL2ConnectInfo->l2HeaderSize + IPv6HDR_SIZE,
                                      IPPROTO_UDP, ipv6FlagOffset, (uint8_t*)&packetId);

            /* Setup the IPv6 Header. */
            Netfp_setupIPv6Header((uint8_t*)ptrHeader + ptrSockL2ConnectInfo->l2HeaderSize,
                                  &ptrNetfpSocket->bindInfo.innerIPSrc.addr.ipv6.u.a8[0],
                                  &ptrNetfpSocket->connectInfo.innerIPDst.addr.ipv4.u.a8[0],
                                  IPPROTO6_FRAG, ptrSockTxMetaInfo->outerDSCP, ipPayloadLength);

            /* Setup the L2 Header. */
            Netfp_setupL2Header(ptrSockTxMetaInfo, ETH_IP6, ptrSockTxMetaInfo->priority, (uint8_t*)ptrHeader);
        }

        /* Cache Hooks: The networking headers (L2, L3 and L4) have all been populated and the header
         * needs to be written back. No more changes are done to the networking headers after this point
         * Descriptors ownership will be released just before the CPDMA push. */
        ptrNetfpSocket->ptrNetfpClient->cfg.endMemAccess(ptrHeader, fragmentHeaderSize);

        /* Link the header packet to the 'split' packet. */
        Pktlib_packetMerge(pktlibInstHandle, ptrFragHeaderPkt, ptrPkt1, NULL);

        /* Increment the statistics for the number of IPv6 fragments which have been created. */
        ptrNetfpSocket->extendedStats.numIPv6Frags++;

        /* Send the packet on the specified interface.
         *  - Software fragmentation has already been done. Bypass this in the NETCP */
        retVal = Netfp_transmitInterface(ptrNetfpSocket, ptrSockTxMetaInfo, ptrFragHeaderPkt);
        if (unlikely(retVal < 0))
        {
            /* 1) Kill the next link in the fragment header packet to not free main descriptor passed to Netfp_fragment() */
            Cppi_linkNextBD (Cppi_DescType_HOST, (Cppi_Desc*)ptrFragHeaderPkt, (Cppi_Desc*)NULL);

            /* 2) Free allocated fragment header */
            Pktlib_freePacket(pktlibInstHandle, ptrFragHeaderPkt);

            /* 3) Free the packet if was not freed yet as split()'s step */
            if (splitRetVal != 1)
                Pktlib_freePacket(pktlibInstHandle, ptrFragment);

            /* Because errCode may contain failures from different level of transmitting,
               set EINTERNAL as recognizable critical error when not ENOMEM type */
            if (retVal == NETFP_ENOMEM)
                *errCode = NETFP_ENOMEM;
            else
                *errCode = NETFP_EINTERNAL;

            break;
        }

        /* Are we done sending all the packets. */
        if ((int32_t)packetLen <= 0)
            break;

        /* Is this a secure socket?  */
        if (ptrNetfpSocket->connectInfo.isSecure == 1)
        {
            /* Secure Socket: Check if the Outer IP is IPv4 or IPv6? */
            if (Netfp_isAddressIPv4(ptrNetfpSocket->connectInfo.outerIPSrc[ptrNetfpSocket->connectInfo.cfgIndex]) == 1)
            {
                /* Secure Socket: Outer IPv4 header + Inner IPv6 Header + ESP Headroom */
                ptrSockTxMetaInfo->headerSize = IPHDR_SIZE + IPv6HDR_SIZE + saPktHdrMargin;
            }
            else
            {
                /* Secure Socket: Outer IPv6 header + Inner IPv6 Header + ESP Headroom */
                ptrSockTxMetaInfo->headerSize = IPv6HDR_SIZE + IPv6HDR_SIZE +
                                                saPktHdrMargin;
            }
            /* fzm: UDP and GTP headers are just once per packet, not per fragment.  As a result
             * After the first fragment, the inner header is only the IP header for the remaining
             * fragments
             */
            ptrSockTxMetaInfo->innerHeaderSize = IPv6HDR_SIZE;
        }
        else
        {
            /* Not a secure socket; account for a single IPv6 Header */
            ptrSockTxMetaInfo->headerSize = IPv6HDR_SIZE;
        }

        /* Compute the number of bytes of the fragment which have been sent out but account for the
         * Fragmentation header while doing so. */
        fragmentOffset = fragmentOffset + ipPayloadLength - IPV6_FRAGHDR_SIZE;

        /* The new payload packet is now what is left */
        ptrSockTxMetaInfo->ptrPayload = ptrPkt2;
    }

    /* Run Garbage Collection on the following heaps.
     *  - The fragment heap has zero buffer descriptors which after the CPDMA transmission
     *    will always get recycled into the garbage queue.
     *  - The payload packet has been split and referenced; so this packet will also end
     *    up in the garbage queue. */
    Pktlib_garbageCollection (pktlibInstHandle, ptrNetfpSocket->ptrNetfpClient->cfg.fragmentHeap);
    Pktlib_garbageCollection (pktlibInstHandle, Pktlib_getPktHeap (ptrSockTxMetaInfo->ptrPayload));

    return (*errCode) ? -1 : 0;
}

static void Netfp_setupUdpHeader_FZM
(
    Netfp_Socket*               ptrNetfpSocket,
    Netfp_SockTxMetaInfo_FZM*   ptrSockTxMetaInfo,
    uint8_t*                    ptrHdrDataBuffer,
    uint16_t                    payloadLen
)
{
    /* Get the payload packet to calculate the UDP checksum */
    Ti_Pkt* ptrPayloadPkt  = ptrSockTxMetaInfo->ptrPayload;

    /* Get the UDP Header. */
    Netfp_UDPHeader* ptrUDPHeader = (Netfp_UDPHeader*)ptrHdrDataBuffer;

    NetfpNetHeaders* netHeadersPtr = &ptrNetfpSocket->netHeaders[ptrSockTxMetaInfo->l2CfgIndex];
    /* Populate the UDP Header. */
    memcpy(ptrUDPHeader, netHeadersPtr->headerBuff + netHeadersPtr->headerSize - UDPHDR_SIZE, UDPHDR_SIZE);
    ptrUDPHeader->Length = Netfp_htons(UDPHDR_SIZE + payloadLen);

    Netfp_PseudoV6Hdr pseudoHdr;
    /* Initialize the Pseudo header which is used for Layer4 UDP checksum calculations. */
    pseudoHdr.SrcAddr = ptrNetfpSocket->bindInfo.innerIPSrc.addr.ipv6;
    pseudoHdr.DstAddr = ptrNetfpSocket->connectInfo.innerIPDst.addr.ipv6;
    pseudoHdr.PktLen  = ptrUDPHeader->Length;
    pseudoHdr.Rsvd[0] = 0;
    pseudoHdr.Rsvd[1] = 0;
    pseudoHdr.Rsvd[2] = 0;
    pseudoHdr.NxtHdr  = IPPROTO_UDP;

    /* Compute the UDP Checksum in software. */
    ptrUDPHeader->UDPChecksum = Netfp_udp6Checksum(ptrPayloadPkt, 0 ,(uint8_t*)ptrUDPHeader, &pseudoHdr);

    return;
}

static int32_t Netfp_fragment6_FZM
(
    Netfp_Socket*           ptrNetfpSocket,
    Netfp_SockTxMetaInfo_FZM*   sockTxMetaInfo,
    int32_t*                errCode
)
{
    *errCode = 0;

    /* Get the NETFP Client MCB */
    Netfp_ClientMCB* ptrClientMCB = ptrNetfpSocket->ptrNetfpClient;

    /* Get the NETFP Client PKTLIB Instance handle. */
    Pktlib_InstHandle pktlibInstHandle = ptrClientMCB->cfg.pktlibInstHandle;

    /* Get the NETFP Socket L2 connect information: */
    Netfp_SockL2ConnectInfo* ptrSockL2ConnectInfo = sockTxMetaInfo->l2ConnectInfo;

    sockTxMetaInfo->hwUDPChksum = 0;

    /* All fragments will have the same unique identifier in the IP packet */
    uint32_t packetId = Netfp_getUniqueId();

    /* Get the total payload length. */
    uint32_t packetLen = Pktlib_getPacketLen(sockTxMetaInfo->ptrPayload);

    /* Get heap handler to not touch the descriptor after it is pushed to HW */
    Pktlib_HeapHandle payloadHeap = Pktlib_getPktHeap(sockTxMetaInfo->ptrPayload);

    /* The header size passed to the function includes the Layer2 header also;
     * which needs to be discounted here. */
    uint32_t headerSize = sockTxMetaInfo->headerSize - sockTxMetaInfo->l2ConnectInfo->l2HeaderSize;

    /* Initialize the fragmentation offset */
    uint32_t fragmentOffset = 0;

    /* Non Secure Socket: Prepare the IPv6 Fragmentation Header. */
    Netfp_IPv6FragHeader fragIPv6Header;
    uint16_t ipv6FlagOffset = 0;
    Netfp_setupIPv6FragHeader((uint8_t*)&fragIPv6Header, IPPROTO_UDP, ipv6FlagOffset, (uint8_t*)&packetId);

    /* Loop around till the entire payload packet is fragmented & sent out. */
    while (1)
    {
        /* fzm--> */
        /* Determine the MAX size of each IP fragment. Fragment Sizes
         * are a multiple of 8 octets. Take this into account while computing the
         * size of each fragment. */
        uint32_t fragmentSize = sockTxMetaInfo->l2ConnectInfo->mtu - headerSize - IPV6_FRAGHDR_SIZE;
        if (fragmentSize > packetLen)
            fragmentSize = packetLen;
        else
            fragmentSize &= ~0x7;
        /* fzm<-- */

        /* Allocate a buffer-less packet so that we can split the packet. */
        Ti_Pkt* ptrFragment = Pktlib_allocPacketPrefetch(ptrClientMCB->cfg.fragmentHeap, 0); //fzm
        if (unlikely(ptrFragment == NULL))
        {
            *errCode = NETFP_ENOMEM;
            break;
        }

        /* Allocate a header packet which carries all standard networking headers (including the IPv6
        * fragment header*/
        Ti_Pkt* ptrFragHeaderPkt = Pktlib_allocPacketPrefetch(ptrClientMCB->cfg.netHeaderHeapHandle,
                                                      headerSize + sockTxMetaInfo->l2ConnectInfo->l2HeaderSize + IPV6_FRAGHDR_SIZE);
        if (unlikely(ptrFragHeaderPkt == NULL))
        {
            Pktlib_freePacket(pktlibInstHandle, ptrFragment);
            *errCode = NETFP_ENOMEM;
            break;
        }

        /* Are there more fragments or not? */
        packetLen = packetLen - fragmentSize;
        if ((int32_t)packetLen <= 0)
            ipv6FlagOffset = 0x0;
        else
            ipv6FlagOffset = 0x1;

        /* Setup the IP Fragmentation. */
        ipv6FlagOffset = ipv6FlagOffset | fragmentOffset;

        /* Each fragment which is created will have additional headers (layer3 & layer2) added to it. */
        uint32_t fragmentHeaderSize;
        Pktlib_getDataBuffer(ptrFragHeaderPkt, &sockTxMetaInfo->ptrHeaderBuffer, &fragmentHeaderSize);

        uint16_t ipPayloadLength = 0;
        /* Is this the first fragment? */
        if (fragmentOffset == 0)
        {
            /* Special Case: First Fragment. */
            NetfpNetHeaders* netHeadersPtr = &ptrNetfpSocket->netHeaders[sockTxMetaInfo->l2CfgIndex];
            memcpy(sockTxMetaInfo->ptrHeaderBuffer, netHeadersPtr->headerBuff, netHeadersPtr->headerSize);
            updateNetHeaderPktIPv6(ptrNetfpSocket, sockTxMetaInfo, 1);

            Netfp_setupUdpHeader_FZM(ptrNetfpSocket, sockTxMetaInfo,
                                     (uint8_t*)sockTxMetaInfo->ptrHeaderBuffer + netHeadersPtr->l2HeaderSize + IPv6HDR_SIZE + IPV6_FRAGHDR_SIZE,
                                     Pktlib_getPacketLen(sockTxMetaInfo->ptrPayload));

            /* Increment the statistics */
            ptrNetfpSocket->extendedStats.numUDPTxPkts++;

            /* The IP Payload Length accounts for the UDP Header also in this case. */
            ipPayloadLength = fragmentSize + UDPHDR_SIZE + IPV6_FRAGHDR_SIZE;
        }
        else if (fragmentOffset > 0)
        {
            NetfpNetHeaders* netHeadersPtr = &ptrNetfpSocket->netHeaders[sockTxMetaInfo->l2CfgIndex];
            memcpy(sockTxMetaInfo->ptrHeaderBuffer, netHeadersPtr->headerBuff, netHeadersPtr->headerSize - UDPHDR_SIZE);
            updateNetHeaderPktIPv6(ptrNetfpSocket, sockTxMetaInfo, 0);

            /* All other fragments dont have any additional Layer4 headers */
            ipPayloadLength = fragmentSize + IPV6_FRAGHDR_SIZE;
        }

        /* Get the IPv6 Header. */
        Netfp_IPv6Header* ptrIPv6Header = (Netfp_IPv6Header*)(sockTxMetaInfo->ptrHeaderBuffer + ptrSockL2ConnectInfo->l2HeaderSize);

        /* Update packet length and next header in IPv6 Header*/
        ptrIPv6Header->PayloadLength = Netfp_htons(ipPayloadLength);
        ptrIPv6Header->NextHeader = IPPROTO6_FRAG;

        /*Update IPv6 FragHdr */
        fragIPv6Header.FragOffset = Netfp_htons(ipv6FlagOffset);
        Netfp_IPv6FragHeader* ptrFragIPv6Header = (Netfp_IPv6FragHeader*)(sockTxMetaInfo->ptrHeaderBuffer + ptrSockL2ConnectInfo->l2HeaderSize + IPv6HDR_SIZE);
        memcpy(ptrFragIPv6Header, &fragIPv6Header, IPV6_FRAGHDR_SIZE);

        /* Split the payload packet into the fragment size */
        Ti_Pkt* ptrPkt1, *ptrPkt2;
        int32_t splitRetVal = Pktlib_splitPacket (pktlibInstHandle, sockTxMetaInfo->ptrPayload, ptrFragment, fragmentSize, &ptrPkt1, &ptrPkt2);

        if (splitRetVal == 1)
        {
            Pktlib_freePacket(pktlibInstHandle, ptrFragment);
        }

        if (unlikely(splitRetVal < 0))
        {
            Pktlib_freePacket(pktlibInstHandle, ptrFragment);
            Pktlib_freePacket(pktlibInstHandle, ptrFragHeaderPkt);

            *errCode = NETFP_EFRAGFAIL;
            break;
        }

        /* Link the header packet to the 'split' packet. */
        Pktlib_packetMerge(pktlibInstHandle, ptrFragHeaderPkt, ptrPkt1, NULL);

        /* Increment the statistics for the number of IPv6 fragments which have been created. */
        ptrNetfpSocket->extendedStats.numIPv6Frags++;

        /* Send the packet on the specified interface.
         *  - Software fragmentation has already been done. Bypass this in the NETCP */
        int32_t retVal = Netfp_transmitInterface_FZM(ptrNetfpSocket, sockTxMetaInfo, ptrFragHeaderPkt);

        if (unlikely(retVal < 0))
        {
            /* 1) Kill the next link in the fragment header packet to not free main descriptor passed to Netfp_fragment() */
            Cppi_linkNextBD (Cppi_DescType_HOST, (Cppi_Desc*)ptrFragHeaderPkt, (Cppi_Desc*)NULL);

            /* 2) Free allocated fragment header */
            Pktlib_freePacket(pktlibInstHandle, ptrFragHeaderPkt);

            /* 3) Free the packet if was not freed yet as split()'s step */
            if (splitRetVal != 1)
                Pktlib_freePacket(pktlibInstHandle, ptrFragment);

            /* Because errCode may contain failures from different level of transmitting,
               set EINTERNAL as recognizable critical error when not ENOMEM type */
            if (retVal == NETFP_ENOMEM)
                *errCode = NETFP_ENOMEM;
            else
                *errCode = NETFP_EINTERNAL;

            break;
        }

        /* Are we done sending all the packets. */
        if ((int32_t)packetLen <= 0)
            break;

        /* Is this a secure socket? */
        /* NO. All fragments will only have the IP Header added to it. */
        headerSize = IPv6HDR_SIZE;
        /* Increment the fragmentation offset to account for the number of bytes sent */
        fragmentOffset = fragmentOffset + ipPayloadLength - IPV6_FRAGHDR_SIZE;

        /* The new payload packet is now what is left */
        sockTxMetaInfo->ptrPayload = ptrPkt2;
    }

    /* Run Garbage Collection on the following heaps.
     *  - The fragment heap has zero buffer descriptors which after the CPDMA transmission
     *    will always get recycled into the garbage queue.
     *  - The payload packet has been split and referenced; so this packet will also end
     *    up in the garbage queue. */
    Pktlib_garbageCollection (pktlibInstHandle, ptrNetfpSocket->ptrNetfpClient->cfg.fragmentHeap);
    Pktlib_garbageCollection (pktlibInstHandle, payloadHeap);

    return (*errCode) ? -1 : 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to create and send an IPv4 packet with
 *      no IPSEC.
 *
 *  @param[in]  ptrNetfpSocket
 *      Socket over which the packet is being sent out.
 *  @param[in]  ptrSockTxMetaInfo
 *      Pointer to the socket transmit meta Information.
 *  @param[out]  errCode
 *      Error Code populated only on error else ignore
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Netfp_sendNonSecureIPv4Pkt
(
    Netfp_Socket*           ptrNetfpSocket,
    Netfp_SockTxMetaInfo*   ptrSockTxMetaInfo,
    int32_t*                errCode
)
{
    uint32_t                    packetSize;
    uint32_t                    headerBufferLen;
    Netfp_SockL2ConnectInfo*    ptrSockL2ConnectInfo;

    /* Get the socket L2 connect information: */
    ptrSockL2ConnectInfo = ptrSockTxMetaInfo->ptrL2ConnectInfo;

    /* Get the total size of the packet including all the headers & trailers. */
    packetSize = Pktlib_getPacketLen(ptrSockTxMetaInfo->ptrPayload) + ptrSockTxMetaInfo->headerSize +
                 ptrSockTxMetaInfo->trailerSize;

    /* Check if we need software fragmentation or not? */
    if (unlikely(packetSize > NETFP_PASS_MAX_BUFFER_SIZE))
    {
        /* WCDMA Frame Protocol CRC Calculation needs to be done here. */
        if (unlikely(ptrSockTxMetaInfo->frameProtoCrc == 1))
        {
            /* Compute software CRC */
            if (Netfp_computeFrameProtoCRC (ptrNetfpSocket, ptrSockTxMetaInfo) < 0)
            {
                *errCode = NETFP_ENOTIMPL;
                return -1;
            }

            /* Turn OFF the flag to indicate CRC computation is done. */
            ptrSockTxMetaInfo->frameProtoCrc = 0;

            /* Increment the statistics */
            ptrNetfpSocket->extendedStats.numFrameProtoCrc++;
        }

        /* Fragment & send them out */
        if (unlikely(Netfp_fragment(ptrNetfpSocket, ptrSockTxMetaInfo, errCode) < 0))
        {
            /* Error: Unable to fragment the packet. */
            ptrNetfpSocket->extendedStats.numIPv4FragsFail++;
            return -1;
        }

        /* Packets have been successfully transmitted. */
        return 0;
    }

    ptrSockTxMetaInfo->ptrHeaderPkt = Pktlib_allocPacketPrefetch(ptrNetfpSocket->ptrNetfpClient->cfg.netHeaderHeapHandle,
                                                    ptrSockTxMetaInfo->headerSize);

    if (unlikely(ptrSockTxMetaInfo->ptrHeaderPkt == NULL))
    {
        *errCode = NETFP_ENOMEM;
        return -1;
    }

    /* Get the allocated data buffer & length. */
    Pktlib_getDataBuffer(ptrSockTxMetaInfo->ptrHeaderPkt, &ptrSockTxMetaInfo->ptrHeaderBuffer, &headerBufferLen);

    /* Fragmentation will be done is hardware. */
    ptrSockTxMetaInfo->hwFragmentation        = 1;

    /* Hardware UDP checksum is disabled for IPv4 */
    ptrSockTxMetaInfo->hwUDPChksum            = 0;

    /* Pass the VLAN priority tag */
    ptrSockTxMetaInfo->priorityTag            = ptrNetfpSocket->priorityTag;

    /************************************************************************
     * Layer4 Headers:-
     *  - Add the GTPU Header to the data packet if the socket is sending
     *    GTPU Packets
     *  - Add the UDP Header.
     ************************************************************************/
    if (unlikely(ptrSockTxMetaInfo->gtpuHeaderSize != 0))
    {
        /* Populate the GTPU Header. */
        Netfp_setupGTPUHeader(ptrNetfpSocket, (uint8_t*)ptrSockTxMetaInfo->ptrHeaderBuffer + ptrSockTxMetaInfo->headerSize -
                              ptrSockTxMetaInfo->gtpuHeaderSize, ptrSockTxMetaInfo->packetSize);

        /* Once the GTPU Header has been added increment the packet to account for the UDP Header */
        ptrSockTxMetaInfo->packetSize = ptrSockTxMetaInfo->packetSize + GTPHDR_SIZE;

        /* Increment the statistics */
        ptrNetfpSocket->extendedStats.numGTPUTxPkts++;
    }

    /* Hardware UDP checksum is only supported for outer-IP fragmentation
     * - Non-Secure
     */
    Netfp_setupUDPHeader(ptrNetfpSocket, ptrSockTxMetaInfo,
                         (uint8_t*)ptrSockTxMetaInfo->ptrHeaderBuffer + ptrSockTxMetaInfo->headerSize -
                          ptrSockTxMetaInfo->gtpuHeaderSize - UDPHDR_SIZE,
                          ptrSockTxMetaInfo->packetSize);

    /* Once the UDP Header has been added increment the packet to account for the UDP Header */
    ptrSockTxMetaInfo->packetSize = ptrSockTxMetaInfo->packetSize + UDPHDR_SIZE;

    /* Increment the statistics */
    ptrNetfpSocket->extendedStats.numUDPTxPkts++;

    /* Software fragmentation is not required. */
    Netfp_setupIPHeader(ptrNetfpSocket, (uint8_t*)ptrSockTxMetaInfo->ptrHeaderBuffer + ptrSockL2ConnectInfo->l2HeaderSize,
                        &ptrNetfpSocket->bindInfo.innerIPSrc.addr.ipv4.u.a8[0],
                        &ptrNetfpSocket->connectInfo.innerIPDst.addr.ipv4.u.a8[0],
                        IPPROTO_UDP, (uint16_t)Netfp_getUniqueId(), ptrSockTxMetaInfo->outerDSCP,
                        0, ptrSockTxMetaInfo->packetSize);

    /* Once the IP Header has been added; increment the packet to account for it. */
    ptrSockTxMetaInfo->packetSize = ptrSockTxMetaInfo->packetSize + IPHDR_SIZE;

    /* Layer2 header: */
    Netfp_setupL2Header(ptrSockTxMetaInfo, ETH_IP, ptrSockTxMetaInfo->priority,
                        (uint8_t*)ptrSockTxMetaInfo->ptrHeaderBuffer);

    /* Once the L2 Header has been added; increment the packet to account for it. */
    ptrSockTxMetaInfo->packetSize = ptrSockTxMetaInfo->packetSize + ptrSockL2ConnectInfo->l2HeaderSize;

    /* Link the header packet to the payload packet. */
    Pktlib_packetMerge(ptrNetfpSocket->ptrNetfpClient->cfg.pktlibInstHandle, ptrSockTxMetaInfo->ptrHeaderPkt, ptrSockTxMetaInfo->ptrPayload, NULL);

    /* Cache Hooks: Writeback the header data buffer */
    ptrNetfpSocket->ptrNetfpClient->cfg.endMemAccess(ptrSockTxMetaInfo->ptrHeaderBuffer, ptrSockTxMetaInfo->headerSize);

    /* Send the packet on the specified interface.
     *  - Fragmentation support is needed from the NETCP */
    *errCode = Netfp_transmitInterface(ptrNetfpSocket, ptrSockTxMetaInfo, ptrSockTxMetaInfo->ptrHeaderPkt);
    if (unlikely(*errCode < 0))
    {
        /* 1) Kill the next link in the header packet to not free main descriptor passed to Netfp_sendNonSecureIPv4Pkt() */
        Cppi_linkNextBD (Cppi_DescType_HOST, (Cppi_Desc*)ptrSockTxMetaInfo->ptrHeaderPkt, (Cppi_Desc*)NULL);

        /* 2) Free allocated header */
        Pktlib_freePacket(ptrNetfpSocket->ptrNetfpClient->cfg.pktlibInstHandle, ptrSockTxMetaInfo->ptrHeaderPkt);

        /* Because errCode may contain failures from different level of transmitting,
           set EINTERNAL as recognizable critical error when not ENOMEM type*/
        if (*errCode != NETFP_ENOMEM)
            *errCode = NETFP_EINTERNAL;

        return -1;
    }

    return 0;
}

static void preparePaCommands (Netfp_Socket* ptrNetfpSocket, uint32_t l2CfgIdx)
{
    Netfp_SockL2ConnectInfo* ptrSockL2ConnectInfo = &ptrNetfpSocket->connectInfo.l2Info[l2CfgIdx];

    paCmdIpFrag_t ipFragCmd;
    memset(&ipFragCmd, 0, sizeof(ipFragCmd));
    ipFragCmd.ipOffset      = ptrSockL2ConnectInfo->l2HeaderSize;
    ipFragCmd.mtuSize       = ptrSockL2ConnectInfo->mtu;

    paCmdNextRoute_t qosRouteInfo;
    if (ptrSockL2ConnectInfo->l3QosCfg.isEnable == 1)
    {
        memset(&qosRouteInfo, 0, sizeof(qosRouteInfo));
        qosRouteInfo.ctrlBitfield      = 0;
        qosRouteInfo.dest              = pa_DEST_HOST;
        qosRouteInfo.pktType_emacCtrl  = 0;
        qosRouteInfo.multiRouteIndex   = (uint16_t)pa_NO_MULTI_ROUTE;
        qosRouteInfo.swInfo0           = 0;
        qosRouteInfo.swInfo1           = 0;
        qosRouteInfo.queue             = ptrSockL2ConnectInfo->l3QosCfg.qid[
            ptrNetfpSocket->connectInfo.fpDSCPMapping[ptrNetfpSocket->priority]
            ];
        qosRouteInfo.flowId            = ptrSockL2ConnectInfo->l3QosCfg.flowId;
        qosRouteInfo.statsIndex        = 0;
    }

    paCmdNextRoute_t switchRouteInfo;
    memset(&switchRouteInfo, 0, sizeof(switchRouteInfo));
    switchRouteInfo.ctrlBitfield     = 0;
    switchRouteInfo.dest             = pa_DEST_EMAC;
    switchRouteInfo.pktType_emacCtrl = ptrSockL2ConnectInfo->switchPortNum;
    switchRouteInfo.multiRouteIndex  = (uint16_t)pa_NO_MULTI_ROUTE;
    switchRouteInfo.swInfo0          = 0;
    switchRouteInfo.swInfo1          = 0;
    switchRouteInfo.queue            = 0;
    switchRouteInfo.flowId           = 0;
    switchRouteInfo.statsIndex       = 0;

    uint8_t cmdIdx = 0;
    paCmdInfo_t cmdInfo[NETFP_TX_CMD_MAX];
    uint32_t* cmdBuffer = ptrNetfpSocket->paCommands[l2CfgIdx].cmdBuffer;
    memset(cmdBuffer, 0 , sizeof(uint32_t)*NETFP_PA_CMD_BUFFER_SIZE);
    memset(cmdInfo, 0 , sizeof(cmdInfo));

    /*====================================================================================
     * Contruct PA TX command to send the non-secure packet.
     * The TX commmand has to follow the following sequences:
     * 1. Hardware Fragmentation
     * 2. L3 Qos if configured in L2 Connect info
     * 3. EMAC switch routing command
     *====================================================================================*/

    /* Setting up command for IP Fragmentation */
    cmdInfo[cmdIdx].cmd              = pa_CMD_IP_FRAGMENT;
    cmdInfo[cmdIdx].params.ipFrag    = ipFragCmd;
    cmdIdx++;

    /* Setting up command for L3 Qos */
    if (ptrSockL2ConnectInfo->l3QosCfg.isEnable == 1)
    {
        cmdInfo[cmdIdx].cmd              = pa_CMD_NEXT_ROUTE;
        cmdInfo[cmdIdx].params.route     = qosRouteInfo;
        cmdIdx++;
    }

    /* Setting up command for Switch port */
    cmdInfo[cmdIdx].cmd              = pa_CMD_NEXT_ROUTE;
    cmdInfo[cmdIdx].params.route     = switchRouteInfo;
    cmdIdx++;

    /* Configure the command set. */
    ptrNetfpSocket->paCommands[l2CfgIdx].cmdBufferSize = NETFP_PA_CMD_BUFFER_SIZE;
    Pa_formatTxCmd (cmdIdx, cmdInfo, 0, (Ptr)cmdBuffer, &ptrNetfpSocket->paCommands[l2CfgIdx].cmdBufferSize);
}

static void prepareL2Header(Netfp_Socket* ptrNetfpSocket, uint8_t cfgIndex)
{
    NetfpNetHeaders* netHeadersPtr = &ptrNetfpSocket->netHeaders[cfgIndex];
    uint32_t  l2HeaderSize = 0;
    Netfp_EthHeader* ptrEthHeader = (Netfp_EthHeader*)(netHeadersPtr->headerBuff);
    Netfp_VLANHeader* ptrVLANHeader = (Netfp_VLANHeader*)((uint8_t *)ptrEthHeader + sizeof(*ptrEthHeader));
    Netfp_SockL2ConnectInfo* ptrL2ConnectInfo = &ptrNetfpSocket->connectInfo.l2Info[cfgIndex];
    memcpy(ptrEthHeader->DstMac, ptrL2ConnectInfo->dstMacAddress, sizeof(ptrEthHeader->DstMac));
    memcpy(ptrEthHeader->SrcMac, ptrL2ConnectInfo->srcMacAddress, sizeof(ptrEthHeader->SrcMac));

    if ((ptrL2ConnectInfo->vlanId != 0) || ((ptrNetfpSocket->priorityTag & NETFP_VLAN_PRIORITYTAG_ENABLED) == NETFP_VLAN_PRIORITYTAG_ENABLED))
    {
        uint16_t    tci;
        ptrEthHeader->Type = Netfp_htons(ETH_VLAN);
        if( ptrL2ConnectInfo->vlanId != 0 )
        {
            /* Compute the TCI value i.e. setup the priority & VLAN identifier. */
            tci = (ptrL2ConnectInfo->vlanMap.socketToVLANPriority[ptrNetfpSocket->priority] << 13)
                | (ptrL2ConnectInfo->vlanId);
        }
        else
        {
            /* Set only the PCP, keep VLAN ID as zero */
            tci = (ptrNetfpSocket->priorityTag & NETFP_VLAN_PRIORITYTAG_MASK) << 13;
        }
        ptrVLANHeader->tci = Netfp_htons(tci);
        ptrVLANHeader->protocol = (ptrNetfpSocket->family == Netfp_SockFamily_AF_INET) ? Netfp_htons(ETH_IP) : Netfp_htons(ETH_IP6);
        l2HeaderSize = ETHHDR_SIZE + VLANHDR_SIZE;
    }
    else
    {
        ptrEthHeader->Type = (ptrNetfpSocket->family == Netfp_SockFamily_AF_INET) ? Netfp_htons(ETH_IP) : Netfp_htons(ETH_IP6);
        l2HeaderSize = ETHHDR_SIZE;
    }

    netHeadersPtr->l2HeaderSize = l2HeaderSize;
    netHeadersPtr->headerSize = l2HeaderSize + UDPHDR_SIZE;
    netHeadersPtr->headerSize += (ptrNetfpSocket->family == Netfp_SockFamily_AF_INET) ? IPHDR_SIZE : IPv6HDR_SIZE;
}

static void prepareIPv4Header(Netfp_Socket* ptrNetfpSocket, uint8_t cfgIndex)
{
    NetfpNetHeaders* netHeadersPtr = &ptrNetfpSocket->netHeaders[cfgIndex];
    Netfp_IPHeader* ipHeaderPtr = (Netfp_IPHeader*)(netHeadersPtr->headerBuff + netHeadersPtr->l2HeaderSize);
    ipHeaderPtr->VerLen = 0x45;
    uint8_t dscp = ptrNetfpSocket->connectInfo.l2Info[cfgIndex].innerToOuterDSCPMap[ptrNetfpSocket->connectInfo.fpDSCPMapping[ptrNetfpSocket->priority]];
    ipHeaderPtr->Tos = (dscp << 2);
    ipHeaderPtr->TotalLen = 0;
    ipHeaderPtr->Id = 0;
    ipHeaderPtr->FlagOff = Netfp_htons((ptrNetfpSocket->dontFrag) ? IPV4_FLAGS_DF_MASK : 0);
    ipHeaderPtr->Ttl = 255;
    ipHeaderPtr->Protocol = IPPROTO_UDP;
    ipHeaderPtr->Checksum = 0;
    memcpy(ipHeaderPtr->IPSrc, &ptrNetfpSocket->bindInfo.innerIPSrc.addr.ipv4.u.a8[0], 4);
    memcpy(ipHeaderPtr->IPDst, &ptrNetfpSocket->connectInfo.innerIPDst.addr.ipv4.u.a8[0], 4);
}

static void prepareIPv6Header(Netfp_Socket* ptrNetfpSocket, uint8_t cfgIndex)
{
    NetfpNetHeaders* netHeadersPtr = &ptrNetfpSocket->netHeaders[cfgIndex];
    Netfp_IPv6Header* ipv6HeaderPtr = (Netfp_IPv6Header*)(netHeadersPtr->headerBuff + netHeadersPtr->l2HeaderSize);
    ipv6HeaderPtr->VerTC = IPV6_VER_VALUE;
    uint8_t dscp = ptrNetfpSocket->connectInfo.l2Info[cfgIndex].innerToOuterDSCPMap[ptrNetfpSocket->connectInfo.fpDSCPMapping[ptrNetfpSocket->priority]];
    ipv6HeaderPtr->FlowLabel[0]  = 0;
    ipv6HeaderPtr->FlowLabel[1]  = 0;
    ipv6HeaderPtr->FlowLabel[2]  = 0;
    ipv6HeaderPtr->VerTC         = (ipv6HeaderPtr->VerTC & 0xF0) | ((dscp >> 2) & 0x0F);
    ipv6HeaderPtr->FlowLabel[0]  = (ipv6HeaderPtr->FlowLabel[0] & 0x3F) | (((dscp & 0x03) << 6) & 0xC0);
    ipv6HeaderPtr->NextHeader    = IPPROTO_UDP;
    ipv6HeaderPtr->HopLimit      = 255; //fzm

    memcpy(ipv6HeaderPtr->SrcAddr.u.a8, &ptrNetfpSocket->bindInfo.innerIPSrc.addr.ipv6.u.a8[0], sizeof(Netfp_IP6N));
    memcpy(ipv6HeaderPtr->DstAddr.u.a8, &ptrNetfpSocket->connectInfo.innerIPDst.addr.ipv6.u.a8[0], sizeof(Netfp_IP6N));

    /* Initialize the Pseudo header which is used for UDP checksum calculations. */
    Netfp_PseudoV6Hdr pseudoHdr;
    pseudoHdr.DstAddr = ptrNetfpSocket->connectInfo.innerIPDst.addr.ipv6;
    pseudoHdr.SrcAddr = ptrNetfpSocket->bindInfo.innerIPSrc.addr.ipv6;
    pseudoHdr.PktLen  = 0;
    pseudoHdr.Rsvd[0] = 0;
    pseudoHdr.Rsvd[1] = 0;
    pseudoHdr.Rsvd[2] = 0;
    pseudoHdr.NxtHdr  = IPPROTO_UDP;

    /* Calculate pseudo Header checksum value */
    uint32_t initVal = 0;
    int tmp1;
    uint16_t* pw = (uint16_t *)&pseudoHdr;
    for( tmp1=0; tmp1 < sizeof(Netfp_PseudoV6Hdr)/2; tmp1++ )
    {
        initVal = initVal + *pw;
        pw++;
    }

    netHeadersPtr->partialChkSum = initVal;

    __asm__ __volatile__ ("dsb ishst" : : : "memory");
}

static void prepareUdpHeader(Netfp_Socket* ptrNetfpSocket, uint8_t cfgIndex)
{
    uint32_t ipHdrSize;
    if(ptrNetfpSocket->family == Netfp_SockFamily_AF_INET)
        ipHdrSize = IPHDR_SIZE;
    else
        ipHdrSize = IPv6HDR_SIZE;

    NetfpNetHeaders* netHeadersPtr = &ptrNetfpSocket->netHeaders[cfgIndex];
    Netfp_UDPHeader* udpHeaderPtr = (Netfp_UDPHeader*)(netHeadersPtr->headerBuff + netHeadersPtr->l2HeaderSize + ipHdrSize);
    udpHeaderPtr->SrcPort = Netfp_htons(ptrNetfpSocket->localSockAddr.sin_port);
    udpHeaderPtr->DstPort = Netfp_htons(ptrNetfpSocket->peerSockAddr.sin_port);
    udpHeaderPtr->Length = 0;
    udpHeaderPtr->UDPChecksum = 0;
}

static void updateIpHeaderDontFrag(Netfp_Socket* ptrNetfpSocket)
{
    NetfpNetHeaders* netHeadersPtr = &ptrNetfpSocket->netHeaders[ptrNetfpSocket->connectInfo.cfgIndex];
    Netfp_IPHeader* ipHeaderPtr = (Netfp_IPHeader*)(netHeadersPtr->headerBuff + netHeadersPtr->l2HeaderSize);
    ipHeaderPtr->FlagOff = Netfp_htons((ptrNetfpSocket->dontFrag) ? IPV4_FLAGS_DF_MASK : 0);
}

static void updateNetHeaderPktIPv4(Netfp_Socket* ptrNetfpSocket, Netfp_SockTxMetaInfo_FZM* ptrSockTxMetaInfo, uint16_t uniqueId, int incUDPHdr, uint16_t flagOff)
{
    NetfpNetHeaders* netHeadersPtr = &ptrNetfpSocket->netHeaders[ptrSockTxMetaInfo->l2CfgIndex];
    Netfp_IPHeader* ipHeaderPtr = (Netfp_IPHeader*)(ptrSockTxMetaInfo->ptrHeaderBuffer + netHeadersPtr->l2HeaderSize);

    uint32_t packetLen = Pktlib_getPacketLen(ptrSockTxMetaInfo->ptrPayload);
    if(incUDPHdr)
    {
        Netfp_UDPHeader* udpHeaderPtr = (Netfp_UDPHeader*)(ptrSockTxMetaInfo->ptrHeaderBuffer + netHeadersPtr->l2HeaderSize + IPHDR_SIZE);
        udpHeaderPtr->Length = Netfp_htons(packetLen + UDPHDR_SIZE);
    }

    ipHeaderPtr->TotalLen = Netfp_htons(packetLen + (incUDPHdr ? UDPHDR_SIZE : 0)+ IPHDR_SIZE);
    ipHeaderPtr->FlagOff = Netfp_htons((ptrNetfpSocket->dontFrag) ? IPV4_FLAGS_DF_MASK : flagOff);
    ipHeaderPtr->Id = uniqueId;

    Netfp_IPChecksum(ipHeaderPtr);
}

static void updateNetHeaderPktIPv6(Netfp_Socket* ptrNetfpSocket, Netfp_SockTxMetaInfo_FZM* ptrSockTxMetaInfo, int incUDPHdr)
{
    NetfpNetHeaders* netHeadersPtr = &ptrNetfpSocket->netHeaders[ptrSockTxMetaInfo->l2CfgIndex];
    Netfp_IPv6Header* ipHeaderPtr = (Netfp_IPv6Header*)(ptrSockTxMetaInfo->ptrHeaderBuffer + netHeadersPtr->l2HeaderSize);

    uint32_t packetLen = Pktlib_getPacketLen(ptrSockTxMetaInfo->ptrPayload);

    if(incUDPHdr)
    {
        Netfp_UDPHeader* udpHeaderPtr = (Netfp_UDPHeader*)(ptrSockTxMetaInfo->ptrHeaderBuffer + netHeadersPtr->l2HeaderSize + IPv6HDR_SIZE);
        udpHeaderPtr->Length = Netfp_htons(UDPHDR_SIZE + packetLen);

        if(ptrSockTxMetaInfo->hwUDPChksum)
        {
            uint32_t initVal = netHeadersPtr->partialChkSum + udpHeaderPtr->Length;
            /* Convert to 16bits checksum */
            initVal = (initVal & 0xFFFF) + (initVal >> 16);
            volatile uint16_t checksum;
            if(initVal >> 16)
                checksum = (initVal & 0xFFFF) + (initVal >> 16);
            else
                checksum = initVal & 0xFFFF;

            /* Prepare UDP checksum fields required by PA, these values will be
               used when construct UDP checksum Tx command */
            ptrSockTxMetaInfo->udpHwChkSum.startOffset   = netHeadersPtr->l2HeaderSize + IPv6HDR_SIZE;
            ptrSockTxMetaInfo->udpHwChkSum.lengthBytes   = packetLen + UDPHDR_SIZE;
            ptrSockTxMetaInfo->udpHwChkSum.initialSum    = Netfp_htons(checksum);
        }
    }

    ipHeaderPtr->PayloadLength = Netfp_htons(packetLen + (incUDPHdr ? UDPHDR_SIZE : 0));
}

static void populateNewConfig(Netfp_Socket* ptrNetfpSocket, uint8_t cfgIndex)
{
    prepareL2Header(ptrNetfpSocket, cfgIndex);

    if(ptrNetfpSocket->family == Netfp_SockFamily_AF_INET)
        prepareIPv4Header(ptrNetfpSocket, cfgIndex);
    else
        prepareIPv6Header(ptrNetfpSocket, cfgIndex);

    prepareUdpHeader(ptrNetfpSocket, cfgIndex);
    preparePaCommands(ptrNetfpSocket, cfgIndex);
}

void Netfp_incNonSecureStats_FZM
(
    Netfp_Socket*           ptrNetfpSocket,
    Netfp_SockTxMetaInfo_FZM*   ptrSockTxMetaInfo,
    uint32_t                 packetLen
)
{
    Netfp_SockL2ConnectInfo* ptrSockL2ConnectInfo = &ptrNetfpSocket->connectInfo.l2Info[ptrSockTxMetaInfo->l2CfgIndex];
    NetfpNetHeaders* netHeaders = &ptrNetfpSocket->netHeaders[ptrSockTxMetaInfo->l2CfgIndex];

    /* Non-Secure Socket: There are multiple cases which we need to address here:
     *  Case (A): Packet is not a fragment
     *  Case (B): Packet is already fragmented by software
     *  Case (C): Packet fragmentation is handled by the NETCP
     * Is the packet <= MTU? */
    if (packetLen <= (ptrSockL2ConnectInfo->mtu + netHeaders->l2HeaderSize))
    {
        /* YES: This handles case (A) and case (B) because there is only 1 IP packet
         * which is being sent out. */
        ptrNetfpSocket->extendedStats.totalStats.outIPPackets += 1;
        ptrNetfpSocket->extendedStats.priorityStats[ptrNetfpSocket->priority].outIPPackets += 1;

        /* The packet could be padded to meet the minimum Ethernet size. So use the real packet size and not the padded size. */
        ptrNetfpSocket->extendedStats.totalStats.outIPOctets += (packetLen - netHeaders->l2HeaderSize);
        ptrNetfpSocket->extendedStats.priorityStats[ptrNetfpSocket->priority].outIPOctets += (packetLen - netHeaders->l2HeaderSize);

        /* Increment the number of ethernet octets seen on the wire. */
        ptrNetfpSocket->extendedStats.totalStats.outEthOctets += packetLen;
        ptrNetfpSocket->extendedStats.priorityStats[ptrNetfpSocket->priority].outEthOctets += packetLen;
        return;
    }

    /* We need to add the IP Header to each fragment. */
    uint32_t headerSize;
    if (ptrNetfpSocket->family == Netfp_SockFamily_AF_INET)
        headerSize = IPHDR_SIZE;
    else
        headerSize = IPv6HDR_SIZE + IPV6_FRAGHDR_SIZE;

    /* NO: This handles case (C) since the fragmentation is done by NETCP we need to do some estimations.
     * Determine the number of IP fragments which NETCP would generate. In order to acheive this we need
     * to start back from top; so get the payload which was passed to the IP layer. */

    uint32_t payloadLen = packetLen - (netHeaders->l2HeaderSize + headerSize);

    /* Compute the number of fragments which are generated: */
    while (1)
    {
        /* Determine the MAX size of each IP fragment. Fragment Sizes are a multiple of 8 octets.
         * Take this into account while computing the size of each fragment. This is the size of
         * only the data part of the fragment. */
        uint32_t paddingLen = 0;
        uint32_t fragmentSize = ptrSockL2ConnectInfo->mtu - headerSize;
        if (fragmentSize > payloadLen)
            fragmentSize = payloadLen;
        else
            fragmentSize &= ~0x7;

        /* Discount the fragment size from the payload length. */
        payloadLen = payloadLen - fragmentSize;

        /* Each fragment carries the networking headers also (So we add those headers here) */
        fragmentSize = fragmentSize + headerSize;

        /* Each fragment is an IP packet: */
        ptrNetfpSocket->extendedStats.totalStats.outIPPackets += 1;
        ptrNetfpSocket->extendedStats.priorityStats[ptrNetfpSocket->priority].outIPPackets += 1;

        /* Increment for the IP Fragment size */
        ptrNetfpSocket->extendedStats.totalStats.outIPOctets  += fragmentSize;
        ptrNetfpSocket->extendedStats.priorityStats[ptrNetfpSocket->priority].outIPOctets += fragmentSize;

        /* Do we need to worry about Ethernet padding? */
        if ((fragmentSize + netHeaders->l2HeaderSize) < ETH_MIN_PKT_SIZE)
            paddingLen = ETH_MIN_PKT_SIZE - (fragmentSize + netHeaders->l2HeaderSize);

        /* Increment the Ethernet statistics; by adding the L2 header to the fragment size */
        ptrNetfpSocket->extendedStats.totalStats.outEthOctets += (fragmentSize + netHeaders->l2HeaderSize + paddingLen);
        ptrNetfpSocket->extendedStats.priorityStats[ptrNetfpSocket->priority].outEthOctets += (fragmentSize + netHeaders->l2HeaderSize + paddingLen);

        /* Are we done? */
        if ((int32_t)payloadLen <= 0)
            break;
    }
    return;
}

static int32_t Netfp_sendNonSecureIPv4Pkt_FZM
(
    Netfp_Socket*           ptrNetfpSocket,
    Netfp_SockTxMetaInfo_FZM*   ptrSockTxMetaInfo,
    int32_t*                errCode
)
{
    NetfpNetHeaders* netHeadersPtr = &ptrNetfpSocket->netHeaders[ptrSockTxMetaInfo->l2CfgIndex];
    memcpy(ptrSockTxMetaInfo->ptrHeaderBuffer, netHeadersPtr->headerBuff, netHeadersPtr->headerSize);
    updateNetHeaderPktIPv4(ptrNetfpSocket, ptrSockTxMetaInfo, Netfp_getUniqueId(), 1, 0);
    /* Link the header packet to the payload packet. */
    Pktlib_packetMerge(ptrNetfpSocket->ptrNetfpClient->cfg.pktlibInstHandle,
        ptrSockTxMetaInfo->ptrHeaderPkt, ptrSockTxMetaInfo->ptrPayload,
        ptrSockTxMetaInfo->ptrHeaderPkt);

    /* Send the packet on the specified interface.
     *  - Fragmentation support is needed from the NETCP */
    *errCode = Netfp_transmitInterface_FZM(ptrNetfpSocket, ptrSockTxMetaInfo,
        ptrSockTxMetaInfo->ptrHeaderPkt);
    if (*errCode < 0)
        return -1;
    return 0;
}

static int32_t Netfp_sendNonSecureIPv6Pkt_FZM
(
    Netfp_Socket*           ptrNetfpSocket,
    Netfp_SockTxMetaInfo_FZM*   ptrSockTxMetaInfo,
    int32_t*                errCode
)
{
    NetfpNetHeaders* netHeadersPtr = &ptrNetfpSocket->netHeaders[ptrSockTxMetaInfo->l2CfgIndex];
    memcpy(ptrSockTxMetaInfo->ptrHeaderBuffer, netHeadersPtr->headerBuff, netHeadersPtr->headerSize);
    updateNetHeaderPktIPv6(ptrNetfpSocket, ptrSockTxMetaInfo, 1);
    /* Link the header packet to the payload packet. */
    Pktlib_packetMerge(ptrNetfpSocket->ptrNetfpClient->cfg.pktlibInstHandle,
        ptrSockTxMetaInfo->ptrHeaderPkt, ptrSockTxMetaInfo->ptrPayload,
        ptrSockTxMetaInfo->ptrHeaderPkt);

    /* Send the packet on the specified interface.
     *  - Fragmentation support is needed from the NETCP */
    *errCode = Netfp_transmitInterface_FZM(ptrNetfpSocket, ptrSockTxMetaInfo,
        ptrSockTxMetaInfo->ptrHeaderPkt);
    if (*errCode < 0)
        return -1;
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to create and send an IPv4 packet with IPSEC.
 *
 *  @param[in]  ptrNetfpSocket
 *      Socket over which the packet is being sent out.
 *  @param[in]  ptrSockTxMetaInfo
 *      Pointer to the socket transmit meta Information.
 *  @param[out]  errCode
 *      Error Code populated only on error else ignore
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Netfp_sendSecureIPv4Pkt
(
    Netfp_Socket*           ptrNetfpSocket,
    Netfp_SockTxMetaInfo*   ptrSockTxMetaInfo,
    int32_t*                errCode
)
{
    uint32_t                    packetSize;
    uint32_t                    headerBufferLen;
    Netfp_SockL2ConnectInfo*    ptrSockL2ConnectInfo;

    /* Get the socket L2 connect information: */
    ptrSockL2ConnectInfo = ptrSockTxMetaInfo->ptrL2ConnectInfo;

    /* Get the total size of the packet including all the headers & trailers. */
    packetSize = Pktlib_getPacketLen(ptrSockTxMetaInfo->ptrPayload) + ptrSockTxMetaInfo->headerSize +
                 ptrSockTxMetaInfo->trailerSize;

    /* WCDMA Frame Protocol CRC Calculation needs to be done here. */
    if (unlikely(ptrSockTxMetaInfo->frameProtoCrc == 1))
    {
        /* Compute software CRC */
        Netfp_computeFrameProtoCRC (ptrNetfpSocket, ptrSockTxMetaInfo);

        /* Turn OFF the flag to indicate CRC computation is done. */
        ptrSockTxMetaInfo->frameProtoCrc = 0;

        /* Increment the statistics */
        ptrNetfpSocket->extendedStats.numFrameProtoCrc++;
    }

    /* Do we need to fragment the packet before it is sent out?
     * for inner IP fragmentation. */
    if (packetSize > (ptrSockL2ConnectInfo->mtu + ptrSockTxMetaInfo->ptrL2ConnectInfo->l2HeaderSize))
    {
        /* Fragment & send them out */
        if (Netfp_fragment(ptrNetfpSocket, ptrSockTxMetaInfo, errCode) < 0)
        {
            /* Error: Unable to fragment the packet. */
            ptrNetfpSocket->extendedStats.numIPv4FragsFail++;
            return -1;
        }
        return 0;
    }

    ptrSockTxMetaInfo->ptrHeaderPkt = Pktlib_allocPacketPrefetch(ptrNetfpSocket->ptrNetfpClient->cfg.netHeaderHeapHandle,
                                                    ptrSockTxMetaInfo->headerSize);

    if (unlikely(ptrSockTxMetaInfo->ptrHeaderPkt == NULL))
    {
        *errCode = NETFP_ENOMEM;
        return -1;
    }

    /* Get the allocated data buffer & length. */
    Pktlib_getDataBuffer(ptrSockTxMetaInfo->ptrHeaderPkt, &ptrSockTxMetaInfo->ptrHeaderBuffer, &headerBufferLen);

    /* Fragmentation will be done is hardware. */
    ptrSockTxMetaInfo->hwFragmentation        = 1;

    /* Hardware UDP checksum is disabled for IPv4 */
    ptrSockTxMetaInfo->hwUDPChksum            = 0;

    /* Pass the VLAN priority tag */
    ptrSockTxMetaInfo->priorityTag            = ptrNetfpSocket->priorityTag;

    /************************************************************************
     * Layer4 Headers:-
     *  - Add the GTPU Header to the data packet if the socket is sending
     *    GTPU Packets
     *  - Add the UDP Header.
     ************************************************************************/
    if (unlikely(ptrSockTxMetaInfo->gtpuHeaderSize != 0))
    {
        /* Populate the GTPU Header. */
        Netfp_setupGTPUHeader(ptrNetfpSocket, (uint8_t*)ptrSockTxMetaInfo->ptrHeaderBuffer + ptrSockTxMetaInfo->headerSize -
                              ptrSockTxMetaInfo->gtpuHeaderSize, ptrSockTxMetaInfo->packetSize);

        /* Once the GTPU Header has been added increment the packet to account for the UDP Header */
        ptrSockTxMetaInfo->packetSize = ptrSockTxMetaInfo->packetSize + GTPHDR_SIZE;

        /* Increment the statistics */
        ptrNetfpSocket->extendedStats.numGTPUTxPkts++;
    }

    /* Hardware UDP checksum is supported for IPSec channel with outer IP frag */
    Netfp_setupUDPHeader(ptrNetfpSocket, ptrSockTxMetaInfo,
                         (uint8_t*)ptrSockTxMetaInfo->ptrHeaderBuffer + ptrSockTxMetaInfo->headerSize -
                          ptrSockTxMetaInfo->gtpuHeaderSize - UDPHDR_SIZE,
                          ptrSockTxMetaInfo->packetSize);

    /* Once the UDP Header has been added increment the packet to account for the UDP Header */
    ptrSockTxMetaInfo->packetSize = ptrSockTxMetaInfo->packetSize + UDPHDR_SIZE;

    /* Increment the statistics */
    ptrNetfpSocket->extendedStats.numUDPTxPkts++;

    /* Software fragmentation is not required: Add the inner IPv4 header */
    Netfp_setupIPHeader(ptrNetfpSocket, (uint8_t*)ptrSockTxMetaInfo->ptrHeaderBuffer + ptrSockTxMetaInfo->headerSize -
                        ptrSockTxMetaInfo->gtpuHeaderSize - UDPHDR_SIZE - IPHDR_SIZE,
                        &ptrNetfpSocket->bindInfo.innerIPSrc.addr.ipv4.u.a8[0],
                        &ptrNetfpSocket->connectInfo.innerIPDst.addr.ipv4.u.a8[0],
                        IPPROTO_UDP, (uint16_t)Netfp_getUniqueId(),
                        ptrSockTxMetaInfo->innerDSCP, 0, ptrSockTxMetaInfo->packetSize);

    /* Once the IP Header has been added; increment the packet to account for it. */
    ptrSockTxMetaInfo->packetSize = ptrSockTxMetaInfo->packetSize + IPHDR_SIZE;

    /* We now need to add the OUTER IP header: This could be IPv4 or IPv6. */
    if (Netfp_isAddressIPv4(ptrNetfpSocket->connectInfo.outerIPSrc[ptrNetfpSocket->connectInfo.cfgIndex]) == 1)
    {
        /* Populate the Outer IPv4 Header: */
        Netfp_setupIPHeader(ptrNetfpSocket, (uint8_t*)ptrSockTxMetaInfo->ptrHeaderBuffer + ptrSockTxMetaInfo->headerSize -
                            ptrSockTxMetaInfo->gtpuHeaderSize - UDPHDR_SIZE - IPHDR_SIZE - IPHDR_SIZE,
                            &ptrNetfpSocket->connectInfo.outerIPSrc[ptrNetfpSocket->connectInfo.cfgIndex].addr.ipv4.u.a8[0],
                            &ptrNetfpSocket->connectInfo.outerIPDst[ptrNetfpSocket->connectInfo.cfgIndex].addr.ipv4.u.a8[0],
                            IPPROTO_IPIP, (uint16_t)Netfp_getUniqueId(),
                            ptrSockTxMetaInfo->outerDSCP, 0, ptrSockTxMetaInfo->packetSize);

        /* Once the IP Header has been added; increment the packet to account for it. */
        ptrSockTxMetaInfo->packetSize = ptrSockTxMetaInfo->packetSize + IPHDR_SIZE;

        /* Layer2 header: */
        Netfp_setupL2Header(ptrSockTxMetaInfo, ETH_IP, ptrSockTxMetaInfo->priority,
                            (uint8_t*)ptrSockTxMetaInfo->ptrHeaderBuffer + ptrSockTxMetaInfo->headerSize -
                            ptrSockTxMetaInfo->gtpuHeaderSize - UDPHDR_SIZE - IPHDR_SIZE - IPHDR_SIZE -
                            ptrSockL2ConnectInfo->l2HeaderSize);

        /* Increment the 4in4 statistics */
        ptrNetfpSocket->extendedStats.numTxPkts4in4Tunnel++;
    }
    else
    {
        /* Outer IPv6 header: */
        Netfp_setupIPv6Header((uint8_t*)ptrSockTxMetaInfo->ptrHeaderBuffer + ptrSockTxMetaInfo->headerSize -
                              ptrSockTxMetaInfo->gtpuHeaderSize - UDPHDR_SIZE - IPHDR_SIZE - IPv6HDR_SIZE,
                              &ptrNetfpSocket->connectInfo.outerIPSrc[ptrNetfpSocket->connectInfo.cfgIndex].addr.ipv6.u.a8[0],
                              &ptrNetfpSocket->connectInfo.outerIPDst[ptrNetfpSocket->connectInfo.cfgIndex].addr.ipv6.u.a8[0],
                              IPPROTO_IPIP, ptrSockTxMetaInfo->outerDSCP, ptrSockTxMetaInfo->packetSize);

        /* Once the IP Header has been added; increment the packet to account for it. */
        ptrSockTxMetaInfo->packetSize = ptrSockTxMetaInfo->packetSize + IPv6HDR_SIZE;

        /* Layer2 header: */
        Netfp_setupL2Header(ptrSockTxMetaInfo, ETH_IP6, ptrSockTxMetaInfo->priority,
                            (uint8_t*)ptrSockTxMetaInfo->ptrHeaderBuffer + ptrSockTxMetaInfo->headerSize -
                            ptrSockTxMetaInfo->gtpuHeaderSize - UDPHDR_SIZE - IPHDR_SIZE - IPv6HDR_SIZE -
                            ptrSockL2ConnectInfo->l2HeaderSize);

        /* Increment the 4in6 statistics */
        ptrNetfpSocket->extendedStats.numTxPkts4in6Tunnel++;
    }

    /* Once the L2 Header has been added; increment the packet to account for it. */
    ptrSockTxMetaInfo->packetSize = ptrSockTxMetaInfo->packetSize + ptrSockL2ConnectInfo->l2HeaderSize;

    /* Link the header packet to the payload packet. */
    Pktlib_packetMerge(ptrNetfpSocket->ptrNetfpClient->cfg.pktlibInstHandle, ptrSockTxMetaInfo->ptrHeaderPkt, ptrSockTxMetaInfo->ptrPayload, NULL);

    /* Cache Hooks: Writeback the header data buffer */
    ptrNetfpSocket->ptrNetfpClient->cfg.endMemAccess(ptrSockTxMetaInfo->ptrHeaderBuffer, ptrSockTxMetaInfo->headerSize);

    /* Send the packet on the specified interface.
     *  - Fragmentation support is needed from the NETCP */
    *errCode = Netfp_transmitInterface(ptrNetfpSocket, ptrSockTxMetaInfo, ptrSockTxMetaInfo->ptrHeaderPkt);
    if (unlikely(*errCode < 0))
    {
        /* 1) Kill the next link in the header packet to not free main descriptor passed to Netfp_sendSecureIPv4Pkt() */
        Cppi_linkNextBD (Cppi_DescType_HOST, (Cppi_Desc*)ptrSockTxMetaInfo->ptrHeaderPkt, (Cppi_Desc*)NULL);

        /* 2) Free allocated header */
        Pktlib_freePacket(ptrNetfpSocket->ptrNetfpClient->cfg.pktlibInstHandle, ptrSockTxMetaInfo->ptrHeaderPkt);

        /* Because errCode may contain failures from different level of transmitting,
           set EINTERNAL as recognizable critical error when not ENOMEM type */
        if (*errCode != NETFP_ENOMEM)
            *errCode = NETFP_EINTERNAL;

        return -1;
    }

    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to create and send an IPv6 packet with no IPSEC.
 *
 *  @param[in]  ptrNetfpSocket
 *      Socket over which the packet is being sent out.
 *  @param[in]  ptrSockTxMetaInfo
 *      Pointer to the socket transmit meta Information.
 *  @param[out]  errCode
 *      Error Code populated only on error else ignore
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0

 */
static int32_t Netfp_sendNonSecureIPv6Pkt
(
    Netfp_Socket*           ptrNetfpSocket,
    Netfp_SockTxMetaInfo*   ptrSockTxMetaInfo,
    int32_t*                errCode
)
{
    uint32_t                    packetSize;
    uint32_t                    headerBufferLen;
    Netfp_SockL2ConnectInfo*    ptrSockL2ConnectInfo;

    /* Get the socket L2 connect information: */
    ptrSockL2ConnectInfo = ptrSockTxMetaInfo->ptrL2ConnectInfo;

    /* Get the total size of the packet including all the headers & trailers. */
    packetSize = Pktlib_getPacketLen(ptrSockTxMetaInfo->ptrPayload) + ptrSockTxMetaInfo->headerSize +
                 ptrSockTxMetaInfo->trailerSize;

    /* Check if we need software fragmentation or not? */
    if (packetSize > NETFP_PASS_MAX_BUFFER_SIZE)
    {
        /* WCDMA Frame Protocol CRC Calculation needs to be done here. */
        if (unlikely(ptrSockTxMetaInfo->frameProtoCrc == 1))
        {
            /* Compute software CRC */
            Netfp_computeFrameProtoCRC (ptrNetfpSocket, ptrSockTxMetaInfo);

            /* Turn OFF the flag to indicate CRC computation is done. */
            ptrSockTxMetaInfo->frameProtoCrc = 0;

            /* Increment the statistics */
            ptrNetfpSocket->extendedStats.numFrameProtoCrc++;
        }

        /* Fragment & send them out */
        if (Netfp_fragment6(ptrNetfpSocket, ptrSockTxMetaInfo, errCode) < 0)
        {
            /* Error: Unable to fragment the packet. */
            ptrNetfpSocket->extendedStats.numIPv6FragsFail++;
            return -1;
        }

        /* Packets have been successfully transmitted. */
        return 0;
    }

    ptrSockTxMetaInfo->ptrHeaderPkt = Pktlib_allocPacketPrefetch(ptrNetfpSocket->ptrNetfpClient->cfg.netHeaderHeapHandle,
                                                    ptrSockTxMetaInfo->headerSize);

    if (unlikely(ptrSockTxMetaInfo->ptrHeaderPkt == NULL))
    {
        *errCode = NETFP_ENOMEM;
        return -1;
    }

    /* Get the allocated data buffer & length. */
    Pktlib_getDataBuffer(ptrSockTxMetaInfo->ptrHeaderPkt, &ptrSockTxMetaInfo->ptrHeaderBuffer, &headerBufferLen);

    /* Fragmentation will be done is hardware. */
    ptrSockTxMetaInfo->hwFragmentation        = 1;

    /* Hardware UDP checksum setting depends on the setting from socket */
    ptrSockTxMetaInfo->hwUDPChksum            = ptrNetfpSocket->udpChksumOffload;

    /* Pass the VLAN priority tag */
    ptrSockTxMetaInfo->priorityTag            = ptrNetfpSocket->priorityTag;

    /************************************************************************
     * Layer4 Headers:-
     *  - Add the GTPU Header to the data packet if the socket is sending
     *    GTPU Packets
     *  - Add the UDP Header.
     ************************************************************************/
    if (unlikely(ptrSockTxMetaInfo->gtpuHeaderSize != 0))
    {
        /* Populate the GTPU Header. */
        Netfp_setupGTPUHeader(ptrNetfpSocket, (uint8_t*)ptrSockTxMetaInfo->ptrHeaderBuffer + ptrSockTxMetaInfo->headerSize -
                              ptrSockTxMetaInfo->gtpuHeaderSize, ptrSockTxMetaInfo->packetSize);

        /* Once the GTPU Header has been added increment the packet to account for the UDP Header */
        ptrSockTxMetaInfo->packetSize = ptrSockTxMetaInfo->packetSize + GTPHDR_SIZE;

        /* Increment the statistics */
        ptrNetfpSocket->extendedStats.numGTPUTxPkts++;
    }

    /* Hardware UDP checksum is only supported for outer-IP fragmentation
     * - Non-Secure
     */
    Netfp_setupUDPHeader(ptrNetfpSocket, ptrSockTxMetaInfo,
                         (uint8_t*)ptrSockTxMetaInfo->ptrHeaderBuffer + ptrSockTxMetaInfo->headerSize -
                          ptrSockTxMetaInfo->gtpuHeaderSize - UDPHDR_SIZE,
                          ptrSockTxMetaInfo->packetSize);

    /* Once the UDP Header has been added increment the packet to account for the UDP Header */
    ptrSockTxMetaInfo->packetSize = ptrSockTxMetaInfo->packetSize + UDPHDR_SIZE;

    /* Increment the statistics */
    ptrNetfpSocket->extendedStats.numUDPTxPkts++;

    /* Software fragmentation is not required. */
    Netfp_setupIPv6Header((uint8_t*)ptrSockTxMetaInfo->ptrHeaderBuffer + ptrSockL2ConnectInfo->l2HeaderSize,
                          &ptrNetfpSocket->bindInfo.innerIPSrc.addr.ipv6.u.a8[0],
                          &ptrNetfpSocket->connectInfo.innerIPDst.addr.ipv6.u.a8[0],
                          IPPROTO_UDP, ptrSockTxMetaInfo->outerDSCP, ptrSockTxMetaInfo->packetSize);

    /* Once the IP Header has been added; increment the packet to account for it. */
    ptrSockTxMetaInfo->packetSize = ptrSockTxMetaInfo->packetSize + IPv6HDR_SIZE;

    /* Layer2 header: */
    Netfp_setupL2Header(ptrSockTxMetaInfo, ETH_IP6, ptrSockTxMetaInfo->priority,
                        (uint8_t*)ptrSockTxMetaInfo->ptrHeaderBuffer);

    /* Once the L2 Header has been added; increment the packet to account for it. */
    ptrSockTxMetaInfo->packetSize = ptrSockTxMetaInfo->packetSize + ptrSockL2ConnectInfo->l2HeaderSize;

    /* Link the header packet to the payload packet. */
    Pktlib_packetMerge(ptrNetfpSocket->ptrNetfpClient->cfg.pktlibInstHandle, ptrSockTxMetaInfo->ptrHeaderPkt, ptrSockTxMetaInfo->ptrPayload, NULL);

    /* Cache Hooks: Writeback the header data buffer */
    ptrNetfpSocket->ptrNetfpClient->cfg.endMemAccess(ptrSockTxMetaInfo->ptrHeaderBuffer, ptrSockTxMetaInfo->headerSize);

    /* Send the packet on the specified interface.
     *  - Fragmentation support is needed from the NETCP */
    *errCode = Netfp_transmitInterface(ptrNetfpSocket, ptrSockTxMetaInfo, ptrSockTxMetaInfo->ptrHeaderPkt);
    if (unlikely(*errCode < 0))
    {
        /* 1) Kill the next link in the header packet to not free main descriptor passed to Netfp_sendNonSecureIPv6Pkt() */
        Cppi_linkNextBD (Cppi_DescType_HOST, (Cppi_Desc*)ptrSockTxMetaInfo->ptrHeaderPkt, (Cppi_Desc*)NULL);

        /* 2) Free allocated header */
        Pktlib_freePacket(ptrNetfpSocket->ptrNetfpClient->cfg.pktlibInstHandle, ptrSockTxMetaInfo->ptrHeaderPkt);

        /* Because errCode may contain failures from different level of transmitting,
           set EINTERNAL as recognizable critical error when not ENOMEM type */
        if (*errCode != NETFP_ENOMEM)
            *errCode = NETFP_EINTERNAL;

        return -1;
    }

    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to create and send an IPv6 packet with IPSEC.
 *
 *  @param[in]  ptrNetfpSocket
 *      Socket over which the packet is being sent out.
 *  @param[in]  ptrSockTxMetaInfo
 *      Pointer to the socket transmit meta Information.
 *  @param[out]  errCode
 *      Error Code populated only on error else ignore
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Netfp_sendSecureIPv6Pkt
(
    Netfp_Socket*           ptrNetfpSocket,
    Netfp_SockTxMetaInfo*   ptrSockTxMetaInfo,
    int32_t*                errCode
)
{
    uint32_t                    packetSize;
    uint32_t                    headerBufferLen;
    Netfp_SockL2ConnectInfo*    ptrSockL2ConnectInfo;

    /* Get the socket L2 connect information: */
    ptrSockL2ConnectInfo = ptrSockTxMetaInfo->ptrL2ConnectInfo;

    /* Get the total size of the packet including all the headers & trailers. */
    packetSize = Pktlib_getPacketLen(ptrSockTxMetaInfo->ptrPayload) + ptrSockTxMetaInfo->headerSize +
                 ptrSockTxMetaInfo->trailerSize;

    /* WCDMA Frame Protocol CRC Calculation needs to be done here. */
    if (unlikely(ptrSockTxMetaInfo->frameProtoCrc == 1))
    {
        /* Compute software CRC */
        Netfp_computeFrameProtoCRC (ptrNetfpSocket, ptrSockTxMetaInfo);

        /* Turn OFF the flag to indicate CRC computation is done. */
        ptrSockTxMetaInfo->frameProtoCrc = 0;

        /* Increment the statistics */
        ptrNetfpSocket->extendedStats.numFrameProtoCrc++;
    }

    /* Do we need to fragment the packet before it is sent out? TODO: We need to port the functionality
     * for inner IP fragmentation. */
    if (packetSize > (ptrSockL2ConnectInfo->mtu + ptrSockTxMetaInfo->ptrL2ConnectInfo->l2HeaderSize))
    {
        /* Fragment & send them out */
        if (unlikely(Netfp_fragment6(ptrNetfpSocket, ptrSockTxMetaInfo, errCode) < 0))
        {
            /* Error: Unable to fragment the packet. */
            ptrNetfpSocket->extendedStats.numIPv6FragsFail++;
            return -1;
        }
        return 0;
    }

    ptrSockTxMetaInfo->ptrHeaderPkt = Pktlib_allocPacketPrefetch(ptrNetfpSocket->ptrNetfpClient->cfg.netHeaderHeapHandle,
                                                    ptrSockTxMetaInfo->headerSize);

    if (unlikely(ptrSockTxMetaInfo->ptrHeaderPkt == NULL))
    {
        *errCode = NETFP_ENOMEM;
        return -1;
    }

    /* Get the allocated data buffer & length. */
    Pktlib_getDataBuffer(ptrSockTxMetaInfo->ptrHeaderPkt, &ptrSockTxMetaInfo->ptrHeaderBuffer, &headerBufferLen);

    /* Fragmentation will be done is hardware. */
    ptrSockTxMetaInfo->hwFragmentation        = 1;

    /* Hardware UDP checksum setting depends on the setting from socket */
    ptrSockTxMetaInfo->hwUDPChksum            = ptrNetfpSocket->udpChksumOffload;

    /* Pass the VLAN priority tag */
    ptrSockTxMetaInfo->priorityTag            = ptrNetfpSocket->priorityTag;

    /************************************************************************
     * Layer4 Headers:-
     *  - Add the GTPU Header to the data packet if the socket is sending
     *    GTPU Packets
     *  - Add the UDP Header.
     ************************************************************************/
    if (unlikely(ptrSockTxMetaInfo->gtpuHeaderSize != 0))
    {
        /* Populate the GTPU Header. */
        Netfp_setupGTPUHeader(ptrNetfpSocket, (uint8_t*)ptrSockTxMetaInfo->ptrHeaderBuffer + ptrSockTxMetaInfo->headerSize -
                              ptrSockTxMetaInfo->gtpuHeaderSize, ptrSockTxMetaInfo->packetSize);

        /* Once the GTPU Header has been added increment the packet to account for the UDP Header */
        ptrSockTxMetaInfo->packetSize = ptrSockTxMetaInfo->packetSize + GTPHDR_SIZE;

        /* Increment the statistics */
        ptrNetfpSocket->extendedStats.numGTPUTxPkts++;
    }


    /* Hardware UDP checksum is supported for IPSec channel with outer IP frag */
    Netfp_setupUDPHeader(ptrNetfpSocket, ptrSockTxMetaInfo,
                         (uint8_t*)ptrSockTxMetaInfo->ptrHeaderBuffer + ptrSockTxMetaInfo->headerSize -
                          ptrSockTxMetaInfo->gtpuHeaderSize - UDPHDR_SIZE,
                          ptrSockTxMetaInfo->packetSize);

    /* Once the UDP Header has been added increment the packet to account for the UDP Header */
    ptrSockTxMetaInfo->packetSize = ptrSockTxMetaInfo->packetSize + UDPHDR_SIZE;

    /* Increment the statistics */
    ptrNetfpSocket->extendedStats.numUDPTxPkts++;

    /* Add the Inner IPv6 header: */
    Netfp_setupIPv6Header((uint8_t*)ptrSockTxMetaInfo->ptrHeaderBuffer + ptrSockTxMetaInfo->headerSize -
                          ptrSockTxMetaInfo->gtpuHeaderSize - UDPHDR_SIZE - IPv6HDR_SIZE,
                          &ptrNetfpSocket->bindInfo.innerIPSrc.addr.ipv6.u.a8[0],
                          &ptrNetfpSocket->connectInfo.innerIPDst.addr.ipv6.u.a8[0],
                          IPPROTO_UDP, ptrSockTxMetaInfo->innerDSCP, ptrSockTxMetaInfo->packetSize);

    /* Once the IP Header has been added; increment the packet to account for it. */
    ptrSockTxMetaInfo->packetSize = ptrSockTxMetaInfo->packetSize + IPv6HDR_SIZE;

    /* We now need to add the OUTER IP header: This could be IPv4 or IPv6. */
    if (Netfp_isAddressIPv4(ptrNetfpSocket->connectInfo.outerIPSrc[ptrNetfpSocket->connectInfo.cfgIndex]) == 1)
    {
        /* Outer IPv4 Header: */
        Netfp_setupIPHeader(ptrNetfpSocket, (uint8_t*)ptrSockTxMetaInfo->ptrHeaderBuffer + ptrSockTxMetaInfo->headerSize -
                            ptrSockTxMetaInfo->gtpuHeaderSize - UDPHDR_SIZE - IPv6HDR_SIZE - IPHDR_SIZE,
                            &ptrNetfpSocket->connectInfo.outerIPSrc[ptrNetfpSocket->connectInfo.cfgIndex].addr.ipv4.u.a8[0],
                            &ptrNetfpSocket->connectInfo.outerIPDst[ptrNetfpSocket->connectInfo.cfgIndex].addr.ipv4.u.a8[0],
                            IPPROTO6_ESP, (uint16_t)Netfp_getUniqueId(),
                            ptrSockTxMetaInfo->outerDSCP, 0, ptrSockTxMetaInfo->packetSize);

        /* Once the IP Header has been added; increment the packet to account for it. */
        ptrSockTxMetaInfo->packetSize = ptrSockTxMetaInfo->packetSize + IPHDR_SIZE;

        /* Layer2 header: */
        Netfp_setupL2Header(ptrSockTxMetaInfo, ETH_IP, ptrSockTxMetaInfo->priority,
                            (uint8_t*)ptrSockTxMetaInfo->ptrHeaderBuffer + ptrSockTxMetaInfo->headerSize -
                            ptrSockTxMetaInfo->gtpuHeaderSize - UDPHDR_SIZE - IPv6HDR_SIZE - IPHDR_SIZE -
                            ptrSockL2ConnectInfo->l2HeaderSize);

        /* Increment the 6in4 statistics */
        ptrNetfpSocket->extendedStats.numTxPkts6in4Tunnel++;
    }
    else
    {
        /* Outer IPv6 header: */
        Netfp_setupIPv6Header((uint8_t*)ptrSockTxMetaInfo->ptrHeaderBuffer + ptrSockTxMetaInfo->headerSize -
                              ptrSockTxMetaInfo->gtpuHeaderSize - UDPHDR_SIZE - IPv6HDR_SIZE - IPv6HDR_SIZE,
                              &ptrNetfpSocket->connectInfo.outerIPSrc[ptrNetfpSocket->connectInfo.cfgIndex].addr.ipv6.u.a8[0],
                              &ptrNetfpSocket->connectInfo.outerIPDst[ptrNetfpSocket->connectInfo.cfgIndex].addr.ipv6.u.a8[0],
                              IPPROTO6_ESP, ptrSockTxMetaInfo->outerDSCP, ptrSockTxMetaInfo->packetSize);

        /* Once the IP Header has been added; increment the packet to account for it. */
        ptrSockTxMetaInfo->packetSize = ptrSockTxMetaInfo->packetSize + IPv6HDR_SIZE;

        /* Layer2 header: */
        Netfp_setupL2Header(ptrSockTxMetaInfo, ETH_IP6, ptrSockTxMetaInfo->priority,
                            (uint8_t*)ptrSockTxMetaInfo->ptrHeaderBuffer + ptrSockTxMetaInfo->headerSize -
                            ptrSockTxMetaInfo->gtpuHeaderSize - UDPHDR_SIZE - IPv6HDR_SIZE - IPv6HDR_SIZE -
                            ptrSockL2ConnectInfo->l2HeaderSize);

        /* Increment the 6in6 statistics */
        ptrNetfpSocket->extendedStats.numTxPkts6in6Tunnel++;
    }

    /* Once the L2 Header has been added; increment the packet to account for it. */
    ptrSockTxMetaInfo->packetSize = ptrSockTxMetaInfo->packetSize + ptrSockL2ConnectInfo->l2HeaderSize;

    /* Link the header packet to the payload packet. */
    Pktlib_packetMerge(ptrNetfpSocket->ptrNetfpClient->cfg.pktlibInstHandle, ptrSockTxMetaInfo->ptrHeaderPkt, ptrSockTxMetaInfo->ptrPayload, NULL);

    /* Cache Hooks: Writeback the header data buffer */
    ptrNetfpSocket->ptrNetfpClient->cfg.endMemAccess(ptrSockTxMetaInfo->ptrHeaderBuffer, ptrSockTxMetaInfo->headerSize);

    /* Send the packet on the specified interface.
     *  - Fragmentation support is needed from the NETCP */
    *errCode = Netfp_transmitInterface(ptrNetfpSocket, ptrSockTxMetaInfo, ptrSockTxMetaInfo->ptrHeaderPkt);
    if (unlikely(*errCode < 0))
    {
        /* 1) Kill the next link in the header packet to not free main descriptor passed to Netfp_sendSecureIPv6Pkt() */
        Cppi_linkNextBD (Cppi_DescType_HOST, (Cppi_Desc*)ptrSockTxMetaInfo->ptrHeaderPkt, (Cppi_Desc*)NULL);

        /* 2) Free allocated header */
        Pktlib_freePacket(ptrNetfpSocket->ptrNetfpClient->cfg.pktlibInstHandle, ptrSockTxMetaInfo->ptrHeaderPkt);

        /* Because errCode may contain failures from different level of transmitting,
           set EINTERNAL as recognizable critical error when not ENOMEM type */
        if (*errCode != NETFP_ENOMEM)
            *errCode = NETFP_EINTERNAL;

        return -1;
    }

    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to send a fully specified Ethernet packet
 *      to the specific destination. The function does *not* add IP or
 *      Ethernet headers as this is the responsibility of the application.
 *
 *  @param[in]  clientHandle
 *      NETFP Client Handle
 *  @param[in]  ptrPayload
 *      Pointer to the actual payload. The packet length should be
 *      correctly configured and all associated data should be
 *      coherent with the cache before this API is invoked.
 *  @param[in]  ptrDst
 *      Pointer to the destination where the packet is to be sent out.
 *  @param[out]  errCode
 *      Error Code populated. On error the payload packet is required to be
 *      cleaned up by the callee.
 *
 *  \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_rawSend
(
    Netfp_ClientHandle  clientHandle,
    Ti_Pkt*             ptrPayload,
    Netfp_RawDst*       ptrDst,
    int32_t*            errCode
)
{
    Netfp_ClientMCB*    ptrNetfpClient;
    uint8_t*            ptrPSInfo;
    uint32_t            psInfoLen;
    Cppi_Result         result;

    /* Get the NETFP Client MCB: */
    ptrNetfpClient = (Netfp_ClientMCB*)clientHandle;

    /* Sanity Check: Validate the arguments. */
    if ((ptrPayload == NULL) || (ptrDst == NULL) || (ptrNetfpClient == NULL))
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Use the destination to determine how the packet should be sent out */
    switch (ptrDst->type)
    {
        case Netfp_RawDstType_SWITCH_PORT:
        {
            /* Send the packet to the specific switch port: */
            if (Netfp_rawTransmitInterface (ptrNetfpClient, ptrPayload, ptrDst->dst.switchPort.port, errCode) < 0)
                return -1;
            break;
        }
        case Netfp_RawDstType_CONTROL_PATH:
        {
            /* Get the PS Information from the packet: */
            result = Cppi_getPSData (Cppi_DescType_HOST, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc *)ptrPayload, (uint8_t **)&ptrPSInfo, &psInfoLen);
            if (result != CPPI_SOK)
            {
                /* Error: There was no PS Information in the packet. This should not be the case. */
                *errCode = result;
                return -1;
            }

            /* Set the next header type to UNKNOWN */
            NETFP_PASAHO_LINFO_SET_NXT_HDR_TYPE((pasahoLongInfo_t*)ptrPSInfo, PASAHO_HDR_UNKNOWN);

            /* Clear the previous match */
            NETFP_PASAHO_LINFO_CLR_PMATCH ((pasahoLongInfo_t*)ptrPSInfo);

            /******************************************************************************
             * NOTE: There is an ownership change here from the DSP to the CPDMA. We need
             * to ensure that the packet contents here are written back.
             ******************************************************************************/
            Pktlib_releaseOwnership(ptrNetfpClient->cfg.pktlibInstHandle, ptrPayload);

            /* Push the packet back to LUT1-1 for further classification */
            Qmss_queuePushDescSize (ptrNetfpClient->netcpTxQueue[NSS_PA_QUEUE_OUTER_IP_INDEX], (Cppi_Desc*)ptrPayload, 128);
            break;
        }
        case Netfp_RawDstType_FAST_PATH:
        {
            /* Sanity Check: Ensure that the multicast services have been enabled */
            if (ptrNetfpClient->multicastInfo.isInitialized == 0)
            {
                /* Error: NETFP client has not initialized the multicast services. */
                *errCode = NETFP_ENOTREADY;
                return -1;
            }

            /* Get the PS Information from the packet: */
            result = Cppi_getPSData (Cppi_DescType_HOST, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc *)ptrPayload, (uint8_t **)&ptrPSInfo, &psInfoLen);
            if (result != CPPI_SOK)
            {
                /* Error: There was no PS Information in the packet. This should not be the case. */
                *errCode = result;
                return -1;
            }

            /* Add the meta data into the packet depending upon the IP version?  */
            if (ptrDst->dst.fastPath.version == Netfp_IPVersion_IPV4)
            {
                /* IPv4: Set the next header and use the correct virtual link identifier */
                NETFP_PASAHO_LINFO_SET_NXT_HDR_TYPE((pasahoLongInfo_t*)ptrPSInfo, PASAHO_HDR_IPv4);
                NETFP_PASAHO_LINFO_SET_VLINK_ID((pasahoLongInfo_t*)ptrPSInfo, ptrNetfpClient->multicastInfo.vlinkIPv4Id);
            }
            else
            {
                /* IPv6: Set the next header and use the correct virtual link identifier */
                NETFP_PASAHO_LINFO_SET_NXT_HDR_TYPE((pasahoLongInfo_t*)ptrPSInfo, PASAHO_HDR_IPv6);
                NETFP_PASAHO_LINFO_SET_VLINK_ID((pasahoLongInfo_t*)ptrPSInfo, ptrNetfpClient->multicastInfo.vlinkIPv6Id);
            }

            /* Set the L3 Offset */
            PASAHO_LINFO_SET_START_OFFSET((pasahoLongInfo_t*)ptrPSInfo, ptrDst->dst.fastPath.l3Offset);

            /* Enable the Virtual Link. */
            NETFP_PASAHO_LINFO_SET_VLINK_ENABLE((pasahoLongInfo_t*)ptrPSInfo);

            /******************************************************************************
             * NOTE: There is an ownership change here from the DSP to the CPDMA. We need
             * to ensure that the packet contents here are written back.
             ******************************************************************************/
            Pktlib_releaseOwnership(ptrNetfpClient->cfg.pktlibInstHandle, ptrPayload);

            /* Push the packet back to LUT1-1 for further classification */
            Qmss_queuePushDescSize (ptrNetfpClient->netcpTxQueue[NSS_PA_QUEUE_OUTER_IP_INDEX], (Cppi_Desc*)ptrPayload, 128);
            break;
        }
        default:
        {
            *errCode = NETFP_EINVAL;
            break;
        }
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to send a packet over the specified socket.
 *
 *  @param[in]  sockHandle
 *      Handle to the socket over which the packet will be sent.
 *  @param[in]  ptrPayload
 *      Pointer to the actual payload. The packet length should be
 *      correctly configured and all associated data should be
 *      coherent with the cache before this API is invoked.
 *  @param[in]  frameProtoPayloadOffset
 *      Offset to the Frame Protocol payload. The NETCP provides the ability
 *      to perform the frame protocol CRC so the parameter here allows
 *      applications to use this feature. Sockets should be marked with the
 *      Netfp_SockOption_CRC_OFFLOAD to utilize this feature; else this
 *      parameter is ignored.
 *  @param[out]  errCode
 *      Error Code populated. On error the payload packet is required to be
 *      cleaned up by the callee.
 *
 *  \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_send
(
    Netfp_SockHandle    sockHandle,
    Ti_Pkt*             ptrPayload,
    uint8_t             frameProtoPayloadOffset,
    int32_t*            errCode
)
{
    Netfp_Socket*           ptrNetfpSocket;
    Netfp_SockTxMetaInfo    sockTxMetaInfo;
    uint8_t*                ptrPayloadBuffer;
    uint32_t                payloadBufferLen;
    int32_t                 retVal;
    uint8_t                 frameType;
//fzm-->
    uint8_t                 paddingLen;
    uint32_t                innerHeaderSize;
//<--fzm

    /* Get the socket information */
    ptrNetfpSocket = (Netfp_Socket*)sockHandle;

    /* Sanity Check: Validate the arguments. */
    if (unlikely((ptrNetfpSocket == NULL) || (ptrPayload == NULL)))
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Sockets should be connected before the packets can be sent out */
    if (unlikely((ptrNetfpSocket->sockState & NETFP_SOCKET_CONNECTED) == 0))
    {
        *errCode = NETFP_ENOTCONNECTED;
        return -1;
    }

    /* Zombie sockets are not operational. */
    if (unlikely(ptrNetfpSocket->status == Netfp_Status_ZOMBIE))
    {
        *errCode = NETFP_ECFG;
        return -1;
    }

    /* Populate the Socket Transmit Meta Information:
     *  Use the configuration index to get the L2 active configuration */
    sockTxMetaInfo.ptrPayload             = ptrPayload;
    sockTxMetaInfo.priority               = ptrNetfpSocket->priority;
    sockTxMetaInfo.frameProtoPayloadOffset = frameProtoPayloadOffset;
    sockTxMetaInfo.headerSize             = UDPHDR_SIZE;
    sockTxMetaInfo.ptrL2ConnectInfo       = &ptrNetfpSocket->connectInfo.l2Info[ptrNetfpSocket->connectInfo.cfgIndex];

    /* Reserve space for the VLAN tag */
    if ((sockTxMetaInfo.ptrL2ConnectInfo->vlanId != 0) ||
       ((ptrNetfpSocket->priorityTag & NETFP_VLAN_PRIORITYTAG_ENABLED) == NETFP_VLAN_PRIORITYTAG_ENABLED))
    {
        sockTxMetaInfo.ptrL2ConnectInfo->l2HeaderSize = ETHHDR_SIZE + VLANHDR_SIZE;
    }
    else
    {
        sockTxMetaInfo.ptrL2ConnectInfo->l2HeaderSize = ETHHDR_SIZE;
    }

    /* Sanity Check: Ensure that the MTU in the L2 connect information is setup correctly. This should always be
     * a valid value for ACTIVE sockets. If this is not correct the event propagation has not been handled
     * correctly. */
    if (unlikely(sockTxMetaInfo.ptrL2ConnectInfo->mtu == NETFP_INVALID_MTU))
    {
        *errCode = NETFP_EINTERNAL;
        return -1;
    }

    /* Get the payload data buffer & length. */
    Pktlib_getDataBuffer(ptrPayload, &ptrPayloadBuffer, &payloadBufferLen);

    /************************************************************************
     * Application Layer:
     *  - For Frame Protocol Channels we check if it is a control or a data
     *    packet
     *  - This needs to be done only if CRC offload is enabled in the socket.
     ************************************************************************/
    if ((ptrNetfpSocket->bindInfo.l3GPPcfg.protocol == Netfp_3GPPProto_FRAME_PROTOCOL) &&
            (ptrNetfpSocket->frameProtoCrcOffload == 1))
    {
        /* CRC Offload: Extract frame type bit to establish control/data packet. */
        frameType = *ptrPayloadBuffer & 0x01;

        if (frameType == 1)
        {
            /* Control frame: No CRC Offload. */
            sockTxMetaInfo.frameProtoCrc = 0;
        }
        else
        {
            /* If this is a data packet, check if any payload is present in the data packet.
             * If no payload present, no CRC is required on this packet.
             */
            if (payloadBufferLen - sockTxMetaInfo.frameProtoPayloadOffset < 2)
            {
                /* No room for CRC in the packet. Do not do CRC Offload. */
                sockTxMetaInfo.frameProtoCrc = 0;
            }
            else
            {
                /* Data frame: Offload CRC. */
                sockTxMetaInfo.frameProtoCrc = 1;
            }
        }
    }
    else
    {
        /* No CRC Offload. */
        sockTxMetaInfo.frameProtoCrc = 0;
    }

    /* Inner IP address have always got to be added. */
    if (Netfp_isAddressIPv4(ptrNetfpSocket->connectInfo.innerIPDst) == 1)
        sockTxMetaInfo.headerSize = sockTxMetaInfo.headerSize + IPHDR_SIZE;
    else
        sockTxMetaInfo.headerSize = sockTxMetaInfo.headerSize + IPv6HDR_SIZE;

    /* Are we sending a GTPU packet? */
    if (unlikely(ptrNetfpSocket->peerSockAddr.sin_gtpuId != 0))
        sockTxMetaInfo.gtpuHeaderSize = GTPHDR_SIZE;
    else
        sockTxMetaInfo.gtpuHeaderSize = 0;

    sockTxMetaInfo.headerSize = sockTxMetaInfo.headerSize + sockTxMetaInfo.gtpuHeaderSize;

    innerHeaderSize = sockTxMetaInfo.headerSize;

    /* Packet Length is now set to the application data payload. */
    sockTxMetaInfo.packetSize = Pktlib_getPacketLen(ptrPayload);

    /* Additional headers need to be account for secure sockets? */
    if (ptrNetfpSocket->connectInfo.isSecure)
    {
        /* Secure Socket: Account for the outer IP address also. */
        if (Netfp_isAddressIPv4(ptrNetfpSocket->connectInfo.outerIPSrc[ptrNetfpSocket->connectInfo.cfgIndex]) == 1)
            sockTxMetaInfo.headerSize = sockTxMetaInfo.headerSize + IPHDR_SIZE;
        else
            sockTxMetaInfo.headerSize = sockTxMetaInfo.headerSize + IPv6HDR_SIZE;

        /* ESP Header consists of IV, SPI and sequence number */
        sockTxMetaInfo.headerSize = sockTxMetaInfo.headerSize +
            ptrNetfpSocket->connectInfo.pktHeaderMargin;

        /* When NAT-T is used, add one UDP header */
        if ((ptrNetfpSocket->connectInfo.nattEncapCfg.srcPort != 0) &&
            (ptrNetfpSocket->connectInfo.nattEncapCfg.dstPort != 0))
            sockTxMetaInfo.headerSize += UDPHDR_SIZE;

        //fzm-->

        /* Here we calculate the padding.  The padding is based the encrypted payload + 2
         * rounded up to the nearest encryption block size. According to requirements padded data are 'initial-vector + payload'
         * but we may simplify it to not use initial vector size because it does not change the result when
         * encryption-block-size=initial-vector-size
         */
        paddingLen = (innerHeaderSize + sockTxMetaInfo.packetSize + IPSEC_ESP_TAIL_SIZE_BYTES) %
                       sockTxMetaInfo.ptrL2ConnectInfo->saGeneralChannelCtrlInfo.txCtrl.params.ipsec.encryptionBlockSize;

        if (paddingLen)
            paddingLen = sockTxMetaInfo.ptrL2ConnectInfo->saGeneralChannelCtrlInfo.txCtrl.params.ipsec.encryptionBlockSize - paddingLen;

        sockTxMetaInfo.trailerSize = ptrNetfpSocket->connectInfo.pktTrailerMargin + paddingLen;
        sockTxMetaInfo.innerHeaderSize = innerHeaderSize;

        //<--fzm
    }
    else
    {
        /* Non Secure socket: No need to add a trailer. */
        sockTxMetaInfo.trailerSize = 0;
    }

    /* Compute the total header size
     * - This includes the layer3 headers + Layer4 (GTPU Header) + L2 headers. */
    sockTxMetaInfo.headerSize = sockTxMetaInfo.headerSize + sockTxMetaInfo.ptrL2ConnectInfo->l2HeaderSize; //fzm

    /* Get the inner and outer DSCP:
     * - Inner DSCP is always derived from the mapping of the NETFP Socket priority to the
     *   DSCP specified in the fast path.
     * - Outer DSCP is always derived from the Inner to Outer DSCP mapping which is specified
     *   in the physical interface. */
    sockTxMetaInfo.innerDSCP = ptrNetfpSocket->connectInfo.fpDSCPMapping[sockTxMetaInfo.priority];
    sockTxMetaInfo.outerDSCP = sockTxMetaInfo.ptrL2ConnectInfo->innerToOuterDSCPMap[sockTxMetaInfo.innerDSCP];

    /************************************************************************
     * IP Layer:
     *  - Routing the Packet
     *      This is already done during CONNECT
     *  - Fragmentation required or not
     *      Routing has already indicated the the destination interface
     *      where the packet has to be sent. Ideally NETCP will handle the
     *      fragmentation of the packet but there is an upper limit. If the
     *      packet exceeds the upper limit we need to perform software
     *      fragmentation before we can send the packet across.
     ************************************************************************/
    if (ptrNetfpSocket->family == Netfp_SockFamily_AF_INET)
    {
        /* Is the socket secure? */
        if (ptrNetfpSocket->connectInfo.isSecure == 0)
            retVal = Netfp_sendNonSecureIPv4Pkt(ptrNetfpSocket, &sockTxMetaInfo, errCode);
        else
            retVal = Netfp_sendSecureIPv4Pkt(ptrNetfpSocket, &sockTxMetaInfo, errCode);
    }
    else
    {
        /* Is the socket secure? */
        if (ptrNetfpSocket->connectInfo.isSecure == 0)
            retVal = Netfp_sendNonSecureIPv6Pkt(ptrNetfpSocket, &sockTxMetaInfo, errCode);
        else
            retVal = Netfp_sendSecureIPv6Pkt(ptrNetfpSocket, &sockTxMetaInfo, errCode);
    }
    return retVal;
}

int32_t Netfp_send_FZM
(
    Netfp_SockHandle    sockHandle,
    Ti_Pkt*             ptrPayload,
    int32_t*            errCode
)
{
    Netfp_SockTxMetaInfo_FZM    sockTxMetaInfo;
    int32_t                 retVal = -1;
    uint32_t                headerBufferLen;

    /* Get the socket information */
    Netfp_Socket* ptrNetfpSocket = (Netfp_Socket*)sockHandle;

    /* Sanity Check: Validate the arguments. */
    if (unlikely((ptrNetfpSocket == NULL) || (ptrPayload == NULL)))
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Populate the Socket Transmit Meta Information:
     *  Use the configuration index to get the L2 active configuration */
    sockTxMetaInfo.ptrPayload = ptrPayload;
    sockTxMetaInfo.l2CfgIndex = ptrNetfpSocket->connectInfo.cfgIndex;
    sockTxMetaInfo.hwUDPChksum = ptrNetfpSocket->udpChksumOffload;
    /* Fragmentation will be done is hardware. */
    sockTxMetaInfo.hwFragmentation = 1;

    Netfp_SockL2ConnectInfo* ptrL2ConnectInfo = &ptrNetfpSocket->connectInfo.l2Info[sockTxMetaInfo.l2CfgIndex];
    __builtin_prefetch(ptrL2ConnectInfo);
    __builtin_prefetch(&ptrNetfpSocket->netHeaders[sockTxMetaInfo.l2CfgIndex]);

    /* Sockets should be connected before the packets can be sent out */
    if (unlikely((ptrNetfpSocket->sockState & NETFP_SOCKET_CONNECTED) == 0))
    {
        *errCode = NETFP_ENOTCONNECTED;
        return -1;
    }

    /* Zombie sockets are not operational. */
    if (unlikely(ptrNetfpSocket->status == Netfp_Status_ZOMBIE))
    {
        *errCode = NETFP_ECFG;
        return -1;
    }

    /* Get the NETFP client. */
    Netfp_ClientMCB* ptrNetfpClient = ptrNetfpSocket->ptrNetfpClient;

    /* Sanity Check: Ensure that the MTU in the L2 connect information is setup correctly. This should always be
     * a valid value for ACTIVE sockets. If this is not correct the event propagation has not been handled
     * correctly. */
    if (unlikely(ptrL2ConnectInfo->mtu == NETFP_INVALID_MTU))
    {
        *errCode = NETFP_EINTERNAL;
        return -1;
    }

    sockTxMetaInfo.headerSize = ptrNetfpSocket->netHeaders[sockTxMetaInfo.l2CfgIndex].headerSize;
    uint32_t packetSize = Pktlib_getPacketLen(ptrPayload) + ptrNetfpSocket->netHeaders[sockTxMetaInfo.l2CfgIndex].headerSize;

    if (packetSize > NETFP_PASS_MAX_BUFFER_SIZE)
    {
        /* Fragmentation will be done is software. */
        sockTxMetaInfo.hwFragmentation = 0;
        sockTxMetaInfo.l2ConnectInfo = ptrL2ConnectInfo;
        uint8_t innerDSCP = ptrNetfpSocket->connectInfo.fpDSCPMapping[ptrNetfpSocket->priority];
        sockTxMetaInfo.outerDSCP = ptrL2ConnectInfo->innerToOuterDSCPMap[innerDSCP];

        /* Fragment & send them out */
        if (ptrNetfpSocket->family == Netfp_SockFamily_AF_INET){
            if (unlikely(Netfp_fragment_FZM(ptrNetfpSocket, &sockTxMetaInfo, errCode) < 0))
            {
                /* Error: Unable to fragment the packet. */
                ptrNetfpSocket->extendedStats.numIPv4FragsFail++;
                return -1;
            }
        }
        else
        {
            if (unlikely(Netfp_fragment6_FZM(ptrNetfpSocket, &sockTxMetaInfo, errCode) < 0))
            {
                /* Error: Unable to fragment the packet. */
                ptrNetfpSocket->extendedStats.numIPv6FragsFail++;
                return -1;
            }
        }

        /* Packets have been successfully transmitted. */
        return 0;
    }

    /* Allocate memory for the header packet. */
    sockTxMetaInfo.ptrHeaderPkt = Pktlib_allocPacketPrefetch(ptrNetfpClient->cfg.netHeaderHeapHandle,
                                                     sockTxMetaInfo.headerSize);
    if (unlikely(sockTxMetaInfo.ptrHeaderPkt == NULL))
    {
        *errCode = NETFP_ENOMEM;
        return -1;
    }

    /* Get the allocated data buffer & length. */
    Pktlib_getDataBuffer(sockTxMetaInfo.ptrHeaderPkt, &sockTxMetaInfo.ptrHeaderBuffer, &headerBufferLen);

    if (ptrNetfpSocket->family == Netfp_SockFamily_AF_INET)
    {
        if (likely(ptrNetfpSocket->connectInfo.isSecure == 0))
            retVal = Netfp_sendNonSecureIPv4Pkt_FZM(ptrNetfpSocket, &sockTxMetaInfo, errCode);
        else
            Pktlib_freePacket(ptrNetfpClient->cfg.pktlibInstHandle, sockTxMetaInfo.ptrHeaderPkt);
    }
    else
    {
        if (likely(ptrNetfpSocket->connectInfo.isSecure == 0))
            retVal = Netfp_sendNonSecureIPv6Pkt_FZM(ptrNetfpSocket, &sockTxMetaInfo, errCode);
        else
            Pktlib_freePacket(ptrNetfpClient->cfg.pktlibInstHandle, sockTxMetaInfo.ptrHeaderPkt);
    }

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to increment statistics for an IPv4 sockets
 *      configured to operate in a non-secure mode.
 *
 *  @param[in]  ptrNetfpSocket
 *      Pointer to the socket which is sending the packet
 *  @param[in]  ptrSockTxMetaInfo
 *      Pointer to the socket transmit meta information
 *  @param[in]  ptrPayload
 *      Pointer to the payload packet which includes the L2 header.
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
void Netfp_incNonSecureIPv4Stats
(
    Netfp_Socket*           ptrNetfpSocket,
    Netfp_SockTxMetaInfo*   ptrSockTxMetaInfo,
    Ti_Pkt*                 ptrPayload
)
{
    Netfp_SockL2ConnectInfo*    ptrSockL2ConnectInfo;
    uint32_t                    packetLen;
    uint32_t                    payloadLen;
    uint32_t                    fragmentSize;
    uint32_t                    headerSize;
    uint32_t                    paddingLen;

    /* Get the socket L2 connect information: */
    ptrSockL2ConnectInfo = ptrSockTxMetaInfo->ptrL2ConnectInfo;

    /* Get the packet length: This is the total length of the packet including the L2 Header
     * but this includes the padding: */
    packetLen = Pktlib_getPacketLen (ptrPayload);

    /* Non-Secure Socket: There are multiple cases which we need to address here:
     *  Case (A): Packet is not a fragment
     *  Case (B): Packet is already fragmented by software
     *  Case (C): Packet fragmentation is handled by the NETCP
     * Is the packet <= MTU? */
    if (packetLen <= (ptrSockL2ConnectInfo->mtu + ptrSockL2ConnectInfo->l2HeaderSize))
    {
        /* YES: This handles case (A) and case (B) because there is only 1 IP packet
         * which is being sent out. */
        ptrNetfpSocket->extendedStats.totalStats.outIPPackets += 1;
        ptrNetfpSocket->extendedStats.priorityStats[ptrNetfpSocket->priority].outIPPackets += 1;

        /* The packet could be padded to meet the minimum Ethernet size. So use the real packet size and not the padded size. */
        ptrNetfpSocket->extendedStats.totalStats.outIPOctets += (ptrSockTxMetaInfo->packetSize - ptrSockL2ConnectInfo->l2HeaderSize);
        ptrNetfpSocket->extendedStats.priorityStats[ptrNetfpSocket->priority].outIPOctets += (ptrSockTxMetaInfo->packetSize - ptrSockL2ConnectInfo->l2HeaderSize);

        /* Increment the number of ethernet octets seen on the wire. */
        ptrNetfpSocket->extendedStats.totalStats.outEthOctets += packetLen;
        ptrNetfpSocket->extendedStats.priorityStats[ptrNetfpSocket->priority].outEthOctets += packetLen;
        return;
    }
    /* NO: This handles case (C) since the fragmentation is done by NETCP we need to do some estimations.
     * Determine the number of IP fragments which NETCP would generate. In order to acheive this we need
     * to start back from top; so get the payload which was passed to the IP layer. */
    payloadLen = packetLen - (ptrSockL2ConnectInfo->l2HeaderSize + IPHDR_SIZE);

    /* We need to add the IP Header to each fragment. */
    headerSize = IPHDR_SIZE;

    /* Compute the number of fragments which are generated: */
    while (1)
    {
        /* Determine the MAX size of each IP fragment. Fragment Sizes are a multiple of 8 octets.
         * Take this into account while computing the size of each fragment. This is the size of
         * only the data part of the fragment. */
        fragmentSize = ptrSockL2ConnectInfo->mtu - headerSize - ptrSockTxMetaInfo->trailerSize;
        if (fragmentSize > payloadLen)
            fragmentSize = payloadLen;
        else
            fragmentSize &= ~0x7;

        /* Discount the fragment size from the payload length. */
        payloadLen = payloadLen - fragmentSize;

        /* Each fragment carries the networking headers also (So we add those headers here) */
        fragmentSize = fragmentSize + headerSize;

        /* Each fragment is an IP packet: */
        ptrNetfpSocket->extendedStats.totalStats.outIPPackets += 1;
        ptrNetfpSocket->extendedStats.priorityStats[ptrNetfpSocket->priority].outIPPackets += 1;

        /* Increment for the IP Fragment size */
        ptrNetfpSocket->extendedStats.totalStats.outIPOctets  += fragmentSize;
        ptrNetfpSocket->extendedStats.priorityStats[ptrNetfpSocket->priority].outIPOctets += fragmentSize;

        /* Do we need to worry about Ethernet padding? */
        if ((fragmentSize + ptrSockL2ConnectInfo->l2HeaderSize) < ETH_MIN_PKT_SIZE)
            paddingLen = ETH_MIN_PKT_SIZE - (fragmentSize + ptrSockL2ConnectInfo->l2HeaderSize);
        else
            paddingLen = 0;

        /* Increment the Ethernet statistics; by adding the L2 header to the fragment size */
        ptrNetfpSocket->extendedStats.totalStats.outEthOctets += (fragmentSize + ptrSockL2ConnectInfo->l2HeaderSize + paddingLen);
        ptrNetfpSocket->extendedStats.priorityStats[ptrNetfpSocket->priority].outEthOctets += (fragmentSize + ptrSockL2ConnectInfo->l2HeaderSize + paddingLen);

        /* Are we done? */
        if ((int32_t)payloadLen <= 0)
            break;
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to increment statistics for an IPv4 sockets
 *      configured to operate in a secure mode.
 *
 *  @param[in]  ptrNetfpSocket
 *      Pointer to the socket which is sending the packet
 *  @param[in]  ptrSockTxMetaInfo
 *      Pointer to the socket transmit meta information
 *  @param[in]  ptrPayload
 *      Pointer to the payload packet which includes the L2 header.
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
void Netfp_incSecureIPv4Stats
(
    Netfp_Socket*           ptrNetfpSocket,
    Netfp_SockTxMetaInfo*   ptrSockTxMetaInfo,
    Ti_Pkt*                 ptrPayload
)
{
    Netfp_SockL2ConnectInfo*    ptrSockL2ConnectInfo;
    uint32_t                    packetLen;

    /* Get the socket L2 connect information: */
    ptrSockL2ConnectInfo = ptrSockTxMetaInfo->ptrL2ConnectInfo;

    /* Get the packet length: This is the total length of the packet including the L2 Header */
    packetLen = Pktlib_getPacketLen (ptrPayload);

    /* Secure Socket: In the case of secure sockets; the NETFP library always performs software fragmentation.
     * So in this case we need to address the following cases:
     * Case (A): Packet is not a fragment
     * Case (B): Packet is already fragmented by software
     * Case (C): Packet fragmentation is handled by NETCP [This is NOT handled in the NETFP library]*/
    if (packetLen > (ptrSockL2ConnectInfo->mtu + ptrSockL2ConnectInfo->l2HeaderSize))
    {
        /* Case(C) This is a just a sanity check and control should never come here because for secure sockets
         * fragmentation is always done in the software. */
        System_printf ("Error: IPv4 Secure Socket Fragmentation offloaded to NETCP\n");
        return;
    }

    /* Case (A) and case (B) are handled here. There is only 1 IP packet which is being sent out. */
    ptrNetfpSocket->extendedStats.totalStats.outIPPackets += 1;
    ptrNetfpSocket->extendedStats.priorityStats[ptrNetfpSocket->priority].outIPPackets += 1;

    /* Update UDP encapsulation stats */
    if ((ptrNetfpSocket->connectInfo.nattEncapCfg.srcPort != 0) &&
        (ptrNetfpSocket->connectInfo.nattEncapCfg.dstPort != 0))
    {
        ptrNetfpSocket->extendedStats.numPktsUdpEncap += 1;
    }

    /* The packet length includes the L2 Header. So we need to discount it off while
     * tracking the IP Octets. */
    ptrNetfpSocket->extendedStats.totalStats.outIPOctets += (packetLen - ptrSockL2ConnectInfo->l2HeaderSize);
    ptrNetfpSocket->extendedStats.priorityStats[ptrNetfpSocket->priority].outIPOctets += (packetLen - ptrSockL2ConnectInfo->l2HeaderSize);

    /* Increment the number of ethernet octets seen on the wire. */
    ptrNetfpSocket->extendedStats.totalStats.outEthOctets += packetLen;
    ptrNetfpSocket->extendedStats.priorityStats[ptrNetfpSocket->priority].outEthOctets += packetLen;
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to increment statistics for an IPv6 sockets
 *      configured to operate in a non-secure mode.
 *
 *  @param[in]  ptrNetfpSocket
 *      Pointer to the socket which is sending the packet
 *  @param[in]  ptrSockTxMetaInfo
 *      Pointer to the socket transmit meta information
 *  @param[in]  ptrPayload
 *      Pointer to the payload packet which includes the L2 header.
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
void Netfp_incNonSecureIPv6Stats
(
    Netfp_Socket*           ptrNetfpSocket,
    Netfp_SockTxMetaInfo*   ptrSockTxMetaInfo,
    Ti_Pkt*                 ptrPayload
)
{
    Netfp_SockL2ConnectInfo*    ptrSockL2ConnectInfo;
    uint32_t                    packetLen;
    uint32_t                    payloadLen;
    uint32_t                    fragmentSize;
    uint32_t                    headerSize;
    uint32_t                    paddingLen;

    /* Get the socket L2 connect information: */
    ptrSockL2ConnectInfo = ptrSockTxMetaInfo->ptrL2ConnectInfo;

    /* Get the packet length: This is the total length of the packet including the L2 Header */
    packetLen = Pktlib_getPacketLen (ptrPayload);

    /* Non-Secure Socket: There are multiple cases which we need to address here:
     *  Case (A): Packet is not a fragment
     *  Case (B): Packet is already fragmented by software
     *  Case (C): Packet fragmentation is handled by the NETCP
     * Is the packet <= MTU? */
    if (packetLen <= (ptrSockL2ConnectInfo->mtu + ptrSockL2ConnectInfo->l2HeaderSize))
    {
        /* YES: This handles case (A) and case (B) because there is only 1 IP packet
         * which is being sent out. */
        ptrNetfpSocket->extendedStats.totalStats.outIPPackets += 1;
        ptrNetfpSocket->extendedStats.priorityStats[ptrNetfpSocket->priority].outIPPackets += 1;

        /* The packet could be padded to meet the minimum Ethernet size. So use the real packet size and not the padded size. */
        ptrNetfpSocket->extendedStats.totalStats.outIPOctets += (ptrSockTxMetaInfo->packetSize - ptrSockL2ConnectInfo->l2HeaderSize);
        ptrNetfpSocket->extendedStats.priorityStats[ptrNetfpSocket->priority].outIPOctets += (ptrSockTxMetaInfo->packetSize - ptrSockL2ConnectInfo->l2HeaderSize);

        /* Increment the number of ethernet octets seen on the wire. */
        ptrNetfpSocket->extendedStats.totalStats.outEthOctets += packetLen;
        ptrNetfpSocket->extendedStats.priorityStats[ptrNetfpSocket->priority].outEthOctets += packetLen;
        return;
    }

    /* NO: This handles case (C) since the fragmentation is done by NETCP we need to do some estimations.
     * Determine the number of IP fragments which NETCP would generate. In order to acheive this we need
     * to start back from top; so get the payload which was passed to the IP layer. */
    payloadLen = packetLen - (ptrSockL2ConnectInfo->l2HeaderSize + IPv6HDR_SIZE);

    /* We need to add the IP Header to each fragment. */
    headerSize = IPv6HDR_SIZE;

    /* Cycle through to determine the number of fragments which will be generated. */
    while (1)
    {
        /* Determine the MAX size of each IP fragment. Fragment Sizes are a multiple of 8 octets.
         * Take this into account while computing the size of each fragment. This is the size of
         * only the data part of the fragment. */
        fragmentSize = ptrSockL2ConnectInfo->mtu - headerSize - ptrSockTxMetaInfo->trailerSize - IPV6_FRAGHDR_SIZE;
        if (fragmentSize > payloadLen)
            fragmentSize = payloadLen;
        else
            fragmentSize &= ~0x7;

        /* Discount the fragment size from the payload length. */
        payloadLen = payloadLen - fragmentSize;

        /* Each IPv6 fragment carries the networking headers also (So we add those headers here) */
        fragmentSize = fragmentSize + headerSize + IPV6_FRAGHDR_SIZE;

        /* Each fragment is an IP packet: */
        ptrNetfpSocket->extendedStats.totalStats.outIPPackets += 1;
        ptrNetfpSocket->extendedStats.priorityStats[ptrNetfpSocket->priority].outIPPackets += 1;

        /* Increment for the IP Fragment size */
        ptrNetfpSocket->extendedStats.totalStats.outIPOctets  += fragmentSize;
        ptrNetfpSocket->extendedStats.priorityStats[ptrNetfpSocket->priority].outIPOctets += fragmentSize;

        /* Do we need to worry about Ethernet padding? */
        if ((fragmentSize + ptrSockL2ConnectInfo->l2HeaderSize) < ETH_MIN_PKT_SIZE)
            paddingLen = ETH_MIN_PKT_SIZE - (fragmentSize + ptrSockL2ConnectInfo->l2HeaderSize);
        else
            paddingLen = 0;

        /* Increment the Ethernet statistics; by adding the L2 header to the fragment size */
        ptrNetfpSocket->extendedStats.totalStats.outEthOctets += (fragmentSize + ptrSockL2ConnectInfo->l2HeaderSize + paddingLen);
        ptrNetfpSocket->extendedStats.priorityStats[ptrNetfpSocket->priority].outEthOctets += (fragmentSize + ptrSockL2ConnectInfo->l2HeaderSize + paddingLen);

        /* Are we done? */
        if ((int32_t)payloadLen <= 0)
            break;
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to increment statistics for an IPv6 sockets
 *      configured to operate in a secure mode.
 *
 *  @param[in]  ptrNetfpSocket
 *      Pointer to the socket which is sending the packet
 *  @param[in]  ptrSockTxMetaInfo
 *      Pointer to the socket transmit meta information
 *  @param[in]  ptrPayload
 *      Pointer to the payload packet which includes the L2 header.
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
void Netfp_incSecureIPv6Stats
(
    Netfp_Socket*           ptrNetfpSocket,
    Netfp_SockTxMetaInfo*   ptrSockTxMetaInfo,
    Ti_Pkt*                 ptrPayload
)
{
    Netfp_SockL2ConnectInfo*    ptrSockL2ConnectInfo;
    uint32_t                    packetLen;

    /* Get the socket L2 connect information: */
    ptrSockL2ConnectInfo = ptrSockTxMetaInfo->ptrL2ConnectInfo;

    /* Get the packet length: This is the total length of the packet including the L2 Header */
    packetLen = Pktlib_getPacketLen (ptrPayload);

    /* Secure Socket: In the case of secure sockets; the NETFP library always performs software fragmentation.
     * So in this case we need to address the following cases:
     * Case (A): Packet is not a fragment
     * Case (B): Packet is already fragmented by software
     * Case (C): Packet fragmentation is handled by NETCP */
    if (packetLen > (ptrSockL2ConnectInfo->mtu + ptrSockL2ConnectInfo->l2HeaderSize))
    {
        /* Case (C): This is NOT supported by NETCP */
        System_printf ("Error: IPv6 Secure Socket Fragmentation offloaded to NETCP\n");
        return;
    }

    /* Case (A) and (B) are handled here because there is only 1 IP packet which is being sent out. */
    ptrNetfpSocket->extendedStats.totalStats.outIPPackets += 1;
    ptrNetfpSocket->extendedStats.priorityStats[ptrNetfpSocket->priority].outIPPackets += 1;

    /* Update UDP encapsulation stats */
    if ((ptrNetfpSocket->connectInfo.nattEncapCfg.srcPort != 0) &&
        (ptrNetfpSocket->connectInfo.nattEncapCfg.dstPort != 0))
    {
        ptrNetfpSocket->extendedStats.numPktsUdpEncap += 1;
    }

    /* The packet length includes the L2 Header. So we need to discount it off while
     * tracking the IP Octets. */
    ptrNetfpSocket->extendedStats.totalStats.outIPOctets += (packetLen - ptrSockL2ConnectInfo->l2HeaderSize);
    ptrNetfpSocket->extendedStats.priorityStats[ptrNetfpSocket->priority].outIPOctets += (packetLen - ptrSockL2ConnectInfo->l2HeaderSize);

    /* Increment the number of ethernet octets seen on the wire. */
    ptrNetfpSocket->extendedStats.totalStats.outEthOctets += packetLen;
    ptrNetfpSocket->extendedStats.priorityStats[ptrNetfpSocket->priority].outEthOctets += packetLen;
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is an internal function which is used to handle the socket
 *      hook.
 *
 *  @param[in]  hook
 *      Hook to be invoked
 *  @param[in]  ptrNetfpClient
 *      Pointer to the NETFP Client
 *  @param[in]  ptrNetfpSocket
 *      Pointer to the NETFP Socket
 *  @param[in]  ptrPayload
 *      Pointer to the payload packet.
 *
 *  \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Proceed - 0
 *  @retval
 *      Abort   - <0
 */
int32_t Netfp_socketHook
(
    Netfp_Hook          hook,
    Netfp_ClientMCB*    ptrNetfpClient,
    Netfp_Socket*       ptrNetfpSocket,
    Ti_Pkt*             ptrPayload
)
{
    Netfp_HookReturn    hookRetVal;

    /* Is there a hook registered? */
    if (ptrNetfpSocket->postRoutingHook == NULL)
        return 0;

    /* YES. Pass the packet to the application registered hook? */
    hookRetVal = ptrNetfpSocket->postRoutingHook (hook, ptrNetfpSocket, ptrPayload, ptrNetfpSocket->postRoutingHookArg);
    switch (hookRetVal)
    {
        case Netfp_HookReturn_DROP:
        {
            /* Packet was dropped by the hook. Cleanup the memory of the packet */
            Pktlib_freePacket(ptrNetfpClient->cfg.pktlibInstHandle, ptrPayload);

            /* Increment the statistics: */
            ptrNetfpSocket->extendedStats.numPktsRejected++;

            /* Packet was dropped we cannot proceed. */
            return -1;
        }
        case Netfp_HookReturn_STOLEN:
        {
            /* Increment the statistics: */
            ptrNetfpSocket->extendedStats.numPktsRejected++;

            /* Packet was dropped we cannot proceed. */
            return -1;
        }
        case Netfp_HookReturn_ACCEPT:
        {
            /* Continue; packet was accepted */
            return 0;
        }
    }

    /* Error: Hook misbehavior */
    System_printf ("Error: Reassembly Hook POST_ROUTING for Socket %p Bad return %d\n",
                   ptrNetfpSocket, hookRetVal);
    return -1;
}

/**
 *  @b Description
 *  @n
 *      The function is used to bind the NETFP Socket with the specific
 *      properties. The socket can receive data only once this API has
 *      been invoked.
 *
 *  @param[in]  sockHandle
 *      Socket Handle which is to be bound
 *  @param[in]  ptrSockAddr
 *      Socket Address Properties which are used to bind the socket.
 *  @param[out]  errCode
 *      Error Code populated only on error else ignore
 *
 *  \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_bind
(
    Netfp_SockHandle    sockHandle,
    Netfp_SockAddr*     ptrSockAddr,
    int32_t*            errCode
)
{
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    uint32_t                result;
    int32_t                 jobId;
    Netfp_Socket*           ptrNetfpSocket;
    Netfp_SockAddr*         ptrRemoteSockAddr;
    Netfp_ClientMCB*        ptrNetfpClient;
    Netfp_SockBindInfo*     ptrRemoteBindInfo;

    /* Initialize the error code */
    *errCode = 0;

    /* Sanity Check: Ensure that the arguments are valid */
    if (unlikely((sockHandle == NULL) || (ptrSockAddr == NULL)))
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Get the socket. */
    ptrNetfpSocket = (Netfp_Socket*)sockHandle;

    /* Get the NETFP Client MCB: Services are available only if the client is active. */
    ptrNetfpClient = (Netfp_ClientMCB*)ptrNetfpSocket->ptrNetfpClient;
    if (unlikely(ptrNetfpClient->status != Netfp_ClientStatus_ACTIVE))
    {
        *errCode = NETFP_ENOTREADY;
        return -1;
    }

    /* Sanity Check: Ensure that the socket and the bind information belong to the same family */
    if (unlikely(ptrNetfpSocket->family != ptrSockAddr->sin_family))
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Get the actual job which we will execute */
    jobHandle = Josh_findJobByAddress(ptrNetfpClient->joshHandle, (Josh_JobProtype)_Netfp_bind);
    if (unlikely(jobHandle == NULL))
    {
        *errCode = NETFP_EINTERNAL;
        return -1;
    }

    /* Initialize the arguments; to avoid any junk */
    memset ((void *)&args, 0, sizeof(args));

    /***************************************************************************
     * This is the function which is to be invoked:
     *
     *  int32_t _Netfp_bind
     *  (
     *  Netfp_ServerMCB*        ptrNetfpServer,
     *  Netfp_SockAddr*         ptrSockAddr,
     *  Netfp_SockBindInfo*     ptrSockBindInfo,
     *  int32_t*                errCode
     *  )
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Netfp_ServerHandle);

    /*  - Argument 2:  */
    args[1].type   = Josh_ArgumentType_PASS_BY_REF;
    args[1].length = sizeof(Netfp_SockAddr);

    /*  - Argument 3: */
    args[2].type   = Josh_ArgumentType_PASS_BY_REF;
    args[2].length = sizeof(Netfp_SockBindInfo);

    /*  - Argument 4: */
    args[3].type   = Josh_ArgumentType_PASS_BY_REF;
    args[3].length = sizeof(int32_t);

    /* Add the arguments. */
    argHandle = Josh_addArguments (ptrNetfpClient->joshHandle, &args[0]);
    if (argHandle == NULL)
    {
        *errCode = NETFP_EJOSH;
        return -1;
    }

    /* Populate the arguments */
    *(uint32_t*)args[0].argBuffer = (uint32_t)ptrNetfpClient->cfg.serverHandle;

    /* Populate the socket address. */
    ptrRemoteSockAddr = (Netfp_SockAddr*)args[1].argBuffer;
    memcpy (ptrRemoteSockAddr, ptrSockAddr, sizeof(Netfp_SockAddr));

    /* Populate the bind info. */
    ptrRemoteBindInfo = (Netfp_SockBindInfo*)args[2].argBuffer;
    memset ((void *)ptrRemoteBindInfo, 0, sizeof(Netfp_SockBindInfo));
    memcpy (&ptrRemoteBindInfo->l3GPPcfg, &ptrNetfpSocket->bindInfo.l3GPPcfg, sizeof(ptrNetfpSocket->bindInfo.l3GPPcfg)); // fzm: https://e2eprivate.ti.com/nokia_siemens_networks/k2_-_fsm4_-_fzm_-_lrc_nokia/f/191/t/7108.aspx

    /* Populate the errCode. */
    memset ((void *)args[3].argBuffer, 0, sizeof(int32_t));

    /* Submit the JOB to JOSH for execution. */
    jobId = Josh_submitJob(jobHandle, argHandle, &result, errCode);
    if (jobId < 0)
        return -1;

    /* Get the result arguments. */
    if (Josh_getArguments (ptrNetfpClient->joshHandle, jobId, &args[0]) < 0)
    {
        *errCode = NETFP_EINTERNAL;
        return -1;
    }

    /* Copy the error code value. */
    *errCode = *((int32_t*)args[3].argBuffer);

    /* Was the socket bind successful? */
    if (result == 0)
    {
        /* YES. Socket has been successfully bound. */
        ptrNetfpSocket->sockState = ptrNetfpSocket->sockState | NETFP_SOCKET_BOUND;

        /* Remember the binding information inside the socket. */
        memcpy((void *)&ptrNetfpSocket->localSockAddr, (void *)ptrSockAddr, sizeof(Netfp_SockAddr));

        /* Copy the binding information over. */
        memcpy (&ptrNetfpSocket->bindInfo, (void *)args[2].argBuffer, sizeof(Netfp_SockBindInfo));
    }

    /* Free the JOB Instance. */
    if (Josh_freeJobInstance(ptrNetfpClient->joshHandle, jobId) < 0)
    {
        *errCode = NETFP_EJOSH;
        return -1;
    }
    return (int32_t)result;
}

/**
 *  @b Description
 *  @n
 *      The function is used to bind the NETFP socket and to ensure that after the LUT2 GTPU
 *      identifier match the packets are passed to the SA for air ciphering. This is an internal
 *      function and is not exposed to the applications and is only called from the LTE channels
 *      in fast path mode.
 *
 *  @param[in]  sockHandle
 *      Socket Handle which is to be bound
 *  @param[in]  ptrSockAddr
 *      Socket Address Properties which are used to bind the socket.
 *  @param[in]  srvSecurityChannelHandle
 *      Server security channel handle
 *  @param[out]  errCode
 *      Error Code populated only on error else ignore
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_secureBind
(
    Netfp_SockHandle            sockHandle,
    Netfp_SockAddr*             ptrSockAddr,
    Netfp_SrvSecChannelHandle   srvSecurityChannelHandle,
    int32_t*                    errCode
)
{
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    uint32_t                result;
    int32_t                 jobId;
    Netfp_Socket*           ptrNetfpSocket;
    Netfp_SockAddr*         ptrRemoteSockAddr;
    Netfp_ClientMCB*        ptrNetfpClient;

    /* Initialize the error code */
    *errCode = 0;

    /* Sanity Check: Ensure that the arguments are valid */
    if ((sockHandle == NULL) || (ptrSockAddr == NULL) || (srvSecurityChannelHandle == NULL))
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Get the socket. */
    ptrNetfpSocket = (Netfp_Socket*)sockHandle;

    /* Get the NETFP Client MCB: Services are available only if the client is active. */
    ptrNetfpClient = (Netfp_ClientMCB*)ptrNetfpSocket->ptrNetfpClient;
    if (ptrNetfpClient->status != Netfp_ClientStatus_ACTIVE)
    {
        *errCode = NETFP_ENOTREADY;
        return -1;
    }

    /* Sanity Check: Ensure that the socket and the bind information belong to the same family */
    if (ptrNetfpSocket->family != ptrSockAddr->sin_family)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Get the actual job which we will execute */
    jobHandle = Josh_findJobByAddress(ptrNetfpClient->joshHandle, (Josh_JobProtype)_Netfp_secureBind);
    if (jobHandle == NULL)
    {
        *errCode = NETFP_EINTERNAL;
        return -1;
    }

    /* Initialize the arguments; to avoid any junk */
    memset ((void *)&args, 0, sizeof(args));

    /***************************************************************************
     * This is the function which is to be invoked:
     *
     *  int32_t _Netfp_secureBind
     *  (
     *  Netfp_ServerMCB*            ptrNetfpServer,
     *  Netfp_SockAddr*             ptrSockAddr,
     *  Netfp_SrvSecChannelHandle   srvSecurityChannelHandle,
     *  Netfp_SockBindInfo*         ptrSockBindInfo,
     *  )
     *
     *  Error code is populated in the return value.
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Netfp_ServerHandle);

    /*  - Argument 2:  */
    args[1].type   = Josh_ArgumentType_PASS_BY_REF;
    args[1].length = sizeof(Netfp_SockAddr);

    /*  - Argument 3: */
    args[2].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[2].length = sizeof(Netfp_SrvSecChannelHandle);

    /*  - Argument 4: */
    args[3].type   = Josh_ArgumentType_PASS_BY_REF;
    args[3].length = sizeof(Netfp_SockBindInfo);

    /* Add the arguments. */
    argHandle = Josh_addArguments (ptrNetfpClient->joshHandle, &args[0]);
    if (argHandle == NULL)
    {
        *errCode = NETFP_EJOSH;
        return -1;
    }

    /* Populate the arguments */
    *(uint32_t*)args[0].argBuffer = (uint32_t)ptrNetfpClient->cfg.serverHandle;

    /* Populate the socket address. */
    ptrRemoteSockAddr = (Netfp_SockAddr*)args[1].argBuffer;
    memcpy (ptrRemoteSockAddr, ptrSockAddr, sizeof(Netfp_SockAddr));

    /* Populate the server security channel handle. */
    *(uint32_t*)args[2].argBuffer = (uint32_t)srvSecurityChannelHandle;

    /* Submit the JOB to JOSH for execution. */
    jobId = Josh_submitJob(jobHandle, argHandle, &result, errCode);
    if (jobId < 0)
        return -1;

    /* Get the result arguments. */
    if (Josh_getArguments (ptrNetfpClient->joshHandle, jobId, &args[0]) < 0)
    {
        *errCode = NETFP_EINTERNAL;
        return -1;
    }

    /* Was the socket bind successful? */
    if (result == 0)
    {
        /* YES. Socket has been successfully bound. */
        ptrNetfpSocket->sockState = ptrNetfpSocket->sockState | NETFP_SOCKET_BOUND;

        /* Remember the binding information inside the socket. */
        memcpy((void *)&ptrNetfpSocket->localSockAddr, (void *)ptrSockAddr, sizeof(Netfp_SockAddr));

        /* Copy the binding information over. */
        memcpy (&ptrNetfpSocket->bindInfo, (void *)args[3].argBuffer, sizeof(Netfp_SockBindInfo));
    }
    else
    {
        /* Copy the error code value. */
        *errCode = result;
    }

    /* Free the JOB Instance. */
    if (Josh_freeJobInstance(ptrNetfpClient->joshHandle, jobId) < 0)
    {
        *errCode = NETFP_EJOSH;
        return -1;
    }
    return (int32_t)result;
}

/**
 *  @b Description
 *  @n
 *      The function is used to bind the NETFP Socket with the specific
 *      properties. The socket can receive data only once this API has
 *      been invoked.
 *
 *  @param[in]  sockHandle
 *      Socket Handle which is to be bound
 *  @param[in]  ptrSockAddr
 *      Socket Address Properties which are used to connect the socket.
 *  @param[out]  errCode
 *      Error Code populated only on error else ignore
 *
 *  \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_connect
(
    Netfp_SockHandle    sockHandle,
    Netfp_SockAddr*     ptrSockAddr,
    int32_t*            errCode
)
{
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    uint32_t                result;
    int32_t                 jobId;
    Netfp_Socket*           ptrNetfpSocket;
    Netfp_SockAddr*         ptrRemoteSockAddr;
    Netfp_ClientMCB*        ptrNetfpClient;

    /* Initialize the error code */
    *errCode = 0;

    /* Sanity Check: Ensure that the arguments are valid */
    if (unlikely((sockHandle == NULL) || (ptrSockAddr == NULL)))
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Get the socket. */
    ptrNetfpSocket = (Netfp_Socket*)sockHandle;

    /* Get the NETFP Client MCB: Services are available only if the client is active. */
    ptrNetfpClient = (Netfp_ClientMCB*)ptrNetfpSocket->ptrNetfpClient;
    if (unlikely(ptrNetfpClient->status != Netfp_ClientStatus_ACTIVE))
    {
        *errCode = NETFP_ENOTREADY;
        return -1;
    }

    /* Sanity Check: Ensure that the socket and the bind information belong to the same family */
    if (unlikely(ptrNetfpSocket->family != ptrSockAddr->sin_family))
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Get the actual job which we will execute */
    jobHandle = Josh_findJobByAddress(ptrNetfpClient->joshHandle, (Josh_JobProtype)_Netfp_connect);
    if (unlikely(jobHandle == NULL))
    {
        *errCode = NETFP_EINTERNAL;
        return -1;
    }

    /* Initialize the arguments; to avoid any junk */
    memset ((void *)&args, 0, sizeof(args));

    /***************************************************************************
     * This is the function which is to be invoked:
     *
     *  int32_t _Netfp_connect
     *  (
     *  Netfp_ServerMCB*        ptrNetfpServer,
     *  Netfp_SockAddr*         ptrSockAddr,
     *  Netfp_SockConnectInfo*  ptrSockConnectInfo,
     *  int32_t*                errCode
     *  )
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Netfp_ServerHandle);

    /*  - Argument 2:  */
    args[1].type   = Josh_ArgumentType_PASS_BY_REF;
    args[1].length = sizeof(Netfp_SockAddr);

    /*  - Argument 3: */
    args[2].type   = Josh_ArgumentType_PASS_BY_REF;
    args[2].length = sizeof(Netfp_SockConnectInfo);

    /*  - Argument 4: */
    args[3].type   = Josh_ArgumentType_PASS_BY_REF;
    args[3].length = sizeof(int32_t);

    /* Add the arguments. */
    argHandle = Josh_addArguments (ptrNetfpClient->joshHandle, &args[0]);
    if (argHandle == NULL)
    {
        *errCode = NETFP_EJOSH;
        return -1;
    }

    /* Populate the arguments */
    *(uint32_t*)args[0].argBuffer = (uint32_t)ptrNetfpClient->cfg.serverHandle;

    /* Populate the socket address. */
    ptrRemoteSockAddr = (Netfp_SockAddr*)args[1].argBuffer;
    memcpy (ptrRemoteSockAddr, ptrSockAddr, sizeof(Netfp_SockAddr));

    /* Populate the errCode. */
    memset ((void *)args[3].argBuffer, 0, sizeof(int32_t));

    /* Submit the JOB to JOSH for execution. */
    jobId = Josh_submitJob(jobHandle, argHandle, &result, errCode);
    if (jobId < 0)
        return -1;

    /* Get the result arguments. */
    if (Josh_getArguments (ptrNetfpClient->joshHandle, jobId, &args[0]) < 0)
    {
        *errCode = NETFP_EINTERNAL;
        return -1;
    }

    /* Copy the error code value. */
    *errCode = *((int32_t*)args[3].argBuffer);

    /* Was the socket bind successful? */
    if (result == 0)
    {
        /* YES. Socket has been successfully connected */
        /* Remember the connect information inside the socket. */
        memcpy((void *)&ptrNetfpSocket->peerSockAddr, (void *)ptrSockAddr, sizeof(Netfp_SockAddr));

        /* Copy the connect information over. */
        memcpy (&ptrNetfpSocket->connectInfo, (void *)args[2].argBuffer, sizeof(Netfp_SockConnectInfo));

        /* Setup the status from the initial socket status. Further updates are done later. */
        ptrNetfpSocket->status = ptrNetfpSocket->connectInfo.initalSocketStatus;

        /* Setup the structures for optimized version of NetfpSend*/
        memset ((void *)ptrNetfpSocket->netHeaders, 0, sizeof(ptrNetfpSocket->netHeaders));
        populateNewConfig(ptrNetfpSocket, ptrNetfpSocket->connectInfo.cfgIndex);

        ptrNetfpSocket->sockState = ptrNetfpSocket->sockState | NETFP_SOCKET_CONNECTED;
    }

    /* Free the JOB Instance. */
    if (Josh_freeJobInstance(ptrNetfpClient->joshHandle, jobId) < 0)
    {
        *errCode = NETFP_EJOSH;
        return -1;
    }
    return result;
}

/**
 *  @b Description
 *  @n
 *      The function is used to suspend the NETFP Socket.
 *      The socket cannot receive data when suspended. Instead the data will be buffered and can be
 *      retrieved when the socket is resumed.
 *      This is an internal function and is not exposed to the applications.
 *
 *  @param[in]  sockHandle
 *      Socket Handle which is to be suspended.
 *  @param[in]  ptrSockAddr
 *      Socket Address Properties which are used to suspend the socket.
 *  @param[out]  errCode
 *      Error Code populated only on error else ignore
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_suspendSocket
(
    Netfp_SockHandle    sockHandle,
    Netfp_SockAddr*     ptrSockAddr,
    int32_t*            errCode
)
{
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    uint32_t                result;
    int32_t                 jobId;
    Netfp_Socket*           ptrNetfpSocket;
    Netfp_SockAddr*         ptrRemoteSockAddr;
    Netfp_ClientMCB*        ptrNetfpClient;
    Netfp_SockBindInfo*     ptrRemoteSockBindInfo;

    /* Initialize the error code */
    *errCode = 0;

    /* Sanity Check: Ensure that the arguments are valid */
    if (sockHandle == NULL)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Get the socket. */
    ptrNetfpSocket = (Netfp_Socket*)sockHandle;

    /* Get the NETFP Client MCB: Services are available only if the client is active. */
    ptrNetfpClient = (Netfp_ClientMCB*)ptrNetfpSocket->ptrNetfpClient;
    if (ptrNetfpClient->status != Netfp_ClientStatus_ACTIVE)
    {
        *errCode = NETFP_ENOTREADY;
        return -1;
    }

    /* Get the actual job which we will execute */
    jobHandle = Josh_findJobByAddress(ptrNetfpClient->joshHandle, (Josh_JobProtype)_Netfp_suspendSocket);
    if (jobHandle == NULL)
    {
        *errCode = NETFP_EINTERNAL;
        return -1;
    }

    /* Initialize the arguments; to avoid any junk */
    memset ((void *)&args, 0, sizeof(args));

    /***************************************************************************
     * This is the function which is to be invoked:
     *
     *  int32_t _Netfp_suspendSocket
     *  (
     *  Netfp_ServerMCB*        ptrNetfpServer,
     *  Netfp_SockAddr*         ptrSockAddr,
     *  Netfp_SockBindInfo*     ptrSockBindInfo,
     *  int32_t*                errCode
     *  )
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Netfp_ServerHandle);

    /*  - Argument 2:  */
    args[1].type   = Josh_ArgumentType_PASS_BY_REF;
    args[1].length = sizeof(Netfp_SockAddr);

    /*  - Argument 3: */
    args[2].type   = Josh_ArgumentType_PASS_BY_REF;
    args[2].length = sizeof(Netfp_SockBindInfo);

    /*  - Argument 4: */
    args[3].type   = Josh_ArgumentType_PASS_BY_REF;
    args[3].length = sizeof(int32_t);

    /* Add the arguments. */
    argHandle = Josh_addArguments (ptrNetfpClient->joshHandle, &args[0]);
    if (argHandle == NULL)
    {
        *errCode = NETFP_EJOSH;
        return -1;
    }

    /* Populate the arguments */
    *(uint32_t*)args[0].argBuffer = (uint32_t)ptrNetfpClient->cfg.serverHandle;

    /* Populate the socket address. */
    ptrRemoteSockAddr = (Netfp_SockAddr*)args[1].argBuffer;
    memcpy (ptrRemoteSockAddr, ptrSockAddr, sizeof(Netfp_SockAddr));

    /* Populate the socket binding information */
    ptrRemoteSockBindInfo = (Netfp_SockBindInfo*)args[2].argBuffer;
    memcpy ((void *)ptrRemoteSockBindInfo, (void *)&ptrNetfpSocket->bindInfo, sizeof(Netfp_SockBindInfo));

    /* Submit the JOB to JOSH for execution. */
    jobId = Josh_submitJob(jobHandle, argHandle, &result, errCode);
    if (jobId < 0)
        return -1;

    /* Get the result arguments. */
    if (Josh_getArguments (ptrNetfpClient->joshHandle, jobId, &args[0]) < 0)
    {
        *errCode = NETFP_EINTERNAL;
        return -1;
    }

    /* Copy the error code value. */
    *errCode = *((int32_t*)args[3].argBuffer);

    /* Free the JOB Instance. */
    if (Josh_freeJobInstance(ptrNetfpClient->joshHandle, jobId) < 0)
    {
        *errCode = NETFP_EJOSH;
        return -1;
    }

    return result;
}

/**
 *  @b Description
 *  @n
 *      The function is used to resume a suspended the NETFP Socket.
 *      This is an internal function and is not exposed to the applications.
 *
 *  @param[in]  sockHandle
 *      Socket Handle which is to be resumed.
 *  @param[in]  ptrSockAddr
 *      Socket Address Properties which are used to resume the socket.
 *  @param[out]  errCode
 *      Error Code populated only on error else ignore
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_resumeSocket
(
    Netfp_SockHandle    sockHandle,
    Netfp_SockAddr*     ptrSockAddr,
    int32_t*            errCode
)
{
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    uint32_t                result;
    int32_t                 jobId;
    Netfp_Socket*           ptrNetfpSocket;
    Netfp_SockAddr*         ptrRemoteSockAddr;
    Netfp_ClientMCB*        ptrNetfpClient;
    Netfp_SockBindInfo*     ptrRemoteSockBindInfo;

    /* Initialize the error code */
    *errCode = 0;

    /* Sanity Check: Ensure that the arguments are valid */
    if ((sockHandle == NULL) || (ptrSockAddr == NULL))
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Get the socket. */
    ptrNetfpSocket = (Netfp_Socket*)sockHandle;

    /* Get the NETFP Client MCB: Services are available only if the client is active. */
    ptrNetfpClient = (Netfp_ClientMCB*)ptrNetfpSocket->ptrNetfpClient;
    if (ptrNetfpClient->status != Netfp_ClientStatus_ACTIVE)
    {
        *errCode = NETFP_ENOTREADY;
        return -1;
    }

    /* Get the actual job which we will execute */
    jobHandle = Josh_findJobByAddress(ptrNetfpClient->joshHandle, (Josh_JobProtype)_Netfp_resumeSocket);
    if (jobHandle == NULL)
    {
        *errCode = NETFP_EINTERNAL;
        return -1;
    }

    /* Initialize the arguments; to avoid any junk */
    memset ((void *)&args, 0, sizeof(args));

    /***************************************************************************
     * This is the function which is to be invoked:
     *
     *  int32_t _Netfp_resumeSocket
     *  (
     *  Netfp_ServerMCB*        ptrNetfpServer,
     *  Netfp_SockAddr*         ptrSockAddr,
     *  Netfp_SockBindInfo*     ptrSockBindInfo,
     *  int32_t*                errCode
     *  )
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Netfp_ServerHandle);

    /*  - Argument 2:  */
    args[1].type   = Josh_ArgumentType_PASS_BY_REF;
    args[1].length = sizeof(Netfp_SockAddr);

    /*  - Argument 3: */
    args[2].type   = Josh_ArgumentType_PASS_BY_REF;
    args[2].length = sizeof(Netfp_SockBindInfo);

    /*  - Argument 4: */
    args[3].type   = Josh_ArgumentType_PASS_BY_REF;
    args[3].length = sizeof(int32_t);

    /* Add the arguments. */
    argHandle = Josh_addArguments (ptrNetfpClient->joshHandle, &args[0]);
    if (argHandle == NULL)
    {
        *errCode = NETFP_EJOSH;
        return -1;
    }

    /* Populate the arguments */
    *(uint32_t*)args[0].argBuffer = (uint32_t)ptrNetfpClient->cfg.serverHandle;

    /* Populate the socket address. */
    ptrRemoteSockAddr = (Netfp_SockAddr*)args[1].argBuffer;
    memcpy (ptrRemoteSockAddr, ptrSockAddr, sizeof(Netfp_SockAddr));

    /* Populate the socket binding information */
    ptrRemoteSockBindInfo = (Netfp_SockBindInfo*)args[2].argBuffer;
    memcpy ((void *)ptrRemoteSockBindInfo, (void *)&ptrNetfpSocket->bindInfo, sizeof(Netfp_SockBindInfo));

    /* Submit the JOB to JOSH for execution. */
    jobId = Josh_submitJob(jobHandle, argHandle, &result, errCode);
    if (jobId < 0)
        return -1;

    /* Get the result arguments. */
    if (Josh_getArguments (ptrNetfpClient->joshHandle, jobId, &args[0]) < 0)
    {
        *errCode = NETFP_EINTERNAL;
        return -1;
    }

    /* Copy the error code value. */
    *errCode = *((int32_t*)args[3].argBuffer);

    /* Free the JOB Instance. */
    if (Josh_freeJobInstance(ptrNetfpClient->joshHandle, jobId) < 0)
    {
        *errCode = NETFP_EJOSH;
        return -1;
    }

    return result;
}

/**
 *  @b Description
 *  @n
 *      The function is used to resume a suspended the NETFP Socket.
 *      After the LUT2 GTPU identifier match the packets are passed to the SA for air ciphering.
 *      This is an internal function and is not exposed to the applications and is only called from the LTE channels
 *      in fast path mode.
 *
 *  @param[in]  sockHandle
 *      Socket Handle which is to be bound
 *  @param[in]  ptrSockAddr
 *      Socket Address Properties which are used to bind the socket.
 *  @param[in]  srvSecurityChannelHandle
 *      Server security channel handle
 *  @param[out]  errCode
 *      Error Code populated only on error else ignore
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_resumeSecureSocket
(
    Netfp_SockHandle            sockHandle,
    Netfp_SockAddr*             ptrSockAddr,
    Netfp_SrvSecChannelHandle   srvSecurityChannelHandle,
    int32_t*                    errCode
)
{
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    uint32_t                result;
    int32_t                 jobId;
    Netfp_Socket*           ptrNetfpSocket;
    Netfp_SockAddr*         ptrRemoteSockAddr;
    Netfp_ClientMCB*        ptrNetfpClient;
    Netfp_SockBindInfo*     ptrRemoteSockBindInfo;

    /* Initialize the error code */
    *errCode = 0;

    /* Sanity Check: Ensure that the arguments are valid */
    if ((sockHandle == NULL) || (ptrSockAddr == NULL) || (srvSecurityChannelHandle == NULL))
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Get the socket. */
    ptrNetfpSocket = (Netfp_Socket*)sockHandle;

    /* Get the NETFP Client MCB: Services are available only if the client is active. */
    ptrNetfpClient = (Netfp_ClientMCB*)ptrNetfpSocket->ptrNetfpClient;
    if (ptrNetfpClient->status != Netfp_ClientStatus_ACTIVE)
    {
        *errCode = NETFP_ENOTREADY;
        return -1;
    }

    /* Get the actual job which we will execute */
    jobHandle = Josh_findJobByAddress(ptrNetfpClient->joshHandle, (Josh_JobProtype)_Netfp_resumeSecureSocket);
    if (jobHandle == NULL)
    {
        *errCode = NETFP_EINTERNAL;
        return -1;
    }

    /* Initialize the arguments; to avoid any junk */
    memset ((void *)&args, 0, sizeof(args));

    /***************************************************************************
     * This is the function which is to be invoked:
     *
     *  int32_t _Netfp_resumeSecureSocket
     *  (
     *  Netfp_ServerMCB*            ptrNetfpServer,
     *  Netfp_SockAddr*             ptrSockAddr,
     *  Netfp_SrvSecChannelHandle   srvSecurityChannelHandle,
     *  Netfp_SockBindInfo*         ptrSockBindInfo,
     *  )
     *
     *  Error code is populated in the return value.
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Netfp_ServerHandle);

    /*  - Argument 2:  */
    args[1].type   = Josh_ArgumentType_PASS_BY_REF;
    args[1].length = sizeof(Netfp_SockAddr);

    /*  - Argument 3: */
    args[2].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[2].length = sizeof(Netfp_SrvSecChannelHandle);

    /*  - Argument 4: */
    args[3].type   = Josh_ArgumentType_PASS_BY_REF;
    args[3].length = sizeof(Netfp_SockBindInfo);

    /* Add the arguments. */
    argHandle = Josh_addArguments (ptrNetfpClient->joshHandle, &args[0]);
    if (argHandle == NULL)
    {
        *errCode = NETFP_EJOSH;
        return -1;
    }

    /* Populate the arguments */
    *(uint32_t*)args[0].argBuffer = (uint32_t)ptrNetfpClient->cfg.serverHandle;

    /* Populate the socket address. */
    ptrRemoteSockAddr = (Netfp_SockAddr*)args[1].argBuffer;
    memcpy (ptrRemoteSockAddr, ptrSockAddr, sizeof(Netfp_SockAddr));

    /* Populate the server security channel handle. */
    *(uint32_t*)args[2].argBuffer = (uint32_t)srvSecurityChannelHandle;

    /* Populate the socket binding information */
    ptrRemoteSockBindInfo = (Netfp_SockBindInfo*)args[3].argBuffer;
    memcpy ((void *)ptrRemoteSockBindInfo, (void *)&ptrNetfpSocket->bindInfo, sizeof(Netfp_SockBindInfo));

    /* Submit the JOB to JOSH for execution. */
    jobId = Josh_submitJob(jobHandle, argHandle, &result, errCode);
    if (jobId < 0)
        return -1;

    /* Get the result arguments. */
    if (Josh_getArguments (ptrNetfpClient->joshHandle, jobId, &args[0]) < 0)
    {
        *errCode = NETFP_EINTERNAL;
        return -1;
    }

    /* Was the socket resume successful? */
    if (result != 0)
    {
        /* Copy the error code value. */
        *errCode = result;

        /* Initialize the return value since the function has failed. */
        result = (uint32_t)-1;
    }

    /* Free the JOB Instance. */
    if (Josh_freeJobInstance(ptrNetfpClient->joshHandle, jobId) < 0)
    {
        *errCode = NETFP_EJOSH;
        return -1;
    }
    return result;
}

/**
 *  @b Description
 *  @n
 *      The function is used to close the NETFP socket
 *
 *  @param[in]  sockHandle
 *      Socket Handle which is to be closed
 *  @param[out]  errCode
 *      Error Code populated only on error else ignore
 *
 *  \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_closeSocket
(
    Netfp_SockHandle    sockHandle,
    int32_t*            errCode
)
{
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    uint32_t                result;
    int32_t                 jobId;
    Netfp_Socket*           ptrNetfpSocket;
    Netfp_ClientMCB*        ptrNetfpClient;
    Netfp_SockBindInfo*     ptrRemoteSockBindInfo;
    Netfp_SockConnectInfo*  ptrRemoteSockConnectInfo;
    void*                   context;

    /* Get the socket information: */
    ptrNetfpSocket = (Netfp_Socket*)sockHandle;
    if (unlikely(ptrNetfpSocket == NULL))
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Sanity Check: 3GPP LTE channels should not directly call the API */
    if (unlikely(ptrNetfpSocket->ptr3GPPSecurityChannel != NULL))
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Get the NETFP Client MCB: Services are available only if the client is active. */
    ptrNetfpClient = (Netfp_ClientMCB*)ptrNetfpSocket->ptrNetfpClient;
    if (unlikely(ptrNetfpClient->status != Netfp_ClientStatus_ACTIVE))
    {
        *errCode = NETFP_ENOTREADY;
        return -1;
    }

    /* Get the actual job which we will execute */
    jobHandle = Josh_findJobByAddress(ptrNetfpClient->joshHandle, (Josh_JobProtype)_Netfp_unbind);
    if (unlikely(jobHandle == NULL))
    {
        *errCode = NETFP_EINTERNAL;
        return -1;
    }

    /* Initialize the arguments; to avoid any junk */
    memset ((void *)&args, 0, sizeof(args));

    /***************************************************************************
     * This is the function which is to be invoked:
     *
     * int32_t _Netfp_unbind
     * (
     *  Netfp_ServerMCB*        ptrNetfpServer,
     *  Netfp_SockBindInfo*     ptrSockBindInfo,
     *  int32_t*                errCode
     * )
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Netfp_ServerHandle);

    /*  - Argument 2:  */
    args[1].type   = Josh_ArgumentType_PASS_BY_REF;
    args[1].length = sizeof(Netfp_SockBindInfo);

    /*  - Argument 3: */
    args[2].type   = Josh_ArgumentType_PASS_BY_REF;
    args[2].length = sizeof(int32_t);

    /* Add the arguments. */
    argHandle = Josh_addArguments (ptrNetfpClient->joshHandle, &args[0]);
    if (argHandle == NULL)
    {
        *errCode = NETFP_EJOSH;
        return -1;
    }

    /* Populate the arguments */
    *(uint32_t*)args[0].argBuffer = (uint32_t)ptrNetfpClient->cfg.serverHandle;

    /* Populate the socket binding information */
    ptrRemoteSockBindInfo = (Netfp_SockBindInfo*)args[1].argBuffer;
    memcpy ((void *)ptrRemoteSockBindInfo, (void *)&ptrNetfpSocket->bindInfo, sizeof(Netfp_SockBindInfo));

    /* Submit the JOB to JOSH for execution. */
    jobId = Josh_submitJob(jobHandle, argHandle, &result, errCode);
    if (jobId < 0)
        return -1;

    /* Get the result arguments. */
    if (Josh_getArguments (ptrNetfpClient->joshHandle, jobId, &args[0]) < 0)
    {
        *errCode = NETFP_EINTERNAL;
        return -1;
    }

    /* Copy the error code value. */
    *errCode = *((int32_t*)args[2].argBuffer);

    /* Free the JOB Instance. */
    if (Josh_freeJobInstance(ptrNetfpClient->joshHandle, jobId) < 0)
    {
        *errCode = NETFP_EJOSH;
        return -1;
    }

    /* Ensure that the unbind was successful? Error code has already been populated */
    if (result != 0)
        return -1;

    /* Was the socket connected? */
    if (ptrNetfpSocket->sockState & NETFP_SOCKET_CONNECTED)
    {
        /* Socket was connected: We need to UNCONNECT the socket at the NETFP Server. */
        jobHandle = Josh_findJobByAddress(ptrNetfpClient->joshHandle, (Josh_JobProtype)_Netfp_unconnect);
        if (jobHandle == NULL)
        {
            *errCode = NETFP_EINTERNAL;
            return -1;
        }

        /* Initialize the arguments; to avoid any junk */
        memset ((void *)&args, 0, sizeof(args));

        /***************************************************************************
         * This is the function which is to be invoked:
         *
         * int32_t _Netfp_unconnect
         * (
         *  Netfp_ServerMCB*        ptrNetfpServer,
         *  Netfp_SockConnectInfo*  ptrSockConnectInfo,
         *  int32_t*                errCode
         * )
         ****************************************************************************/

        /* Populate the arguments.
         * - Argument 1: */
        args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
        args[0].length = sizeof(Netfp_ServerHandle);

        /*  - Argument 2:  */
        args[1].type   = Josh_ArgumentType_PASS_BY_REF;
        args[1].length = sizeof(Netfp_SockConnectInfo);

        /*  - Argument 3: */
        args[2].type   = Josh_ArgumentType_PASS_BY_REF;
        args[2].length = sizeof(int32_t);

        /* Add the arguments. */
        argHandle = Josh_addArguments (ptrNetfpClient->joshHandle, &args[0]);
        if (argHandle == NULL)
        {
            *errCode = NETFP_EJOSH;
            return -1;
        }

        /* Populate the arguments */
        *(uint32_t*)args[0].argBuffer = (uint32_t)ptrNetfpClient->cfg.serverHandle;

        /* Populate the socket binding information */
        ptrRemoteSockConnectInfo = (Netfp_SockConnectInfo*)args[1].argBuffer;
        memcpy ((void *)ptrRemoteSockConnectInfo, (void *)&ptrNetfpSocket->connectInfo, sizeof(Netfp_SockConnectInfo));

        /* Submit the JOB to JOSH for execution. */
        jobId = Josh_submitJob(jobHandle, argHandle, &result, errCode);
        if (jobId < 0)
            return -1;

        /* Get the result arguments. */
        if (Josh_getArguments (ptrNetfpClient->joshHandle, jobId, &args[0]) < 0)
        {
            *errCode = NETFP_EINTERNAL;
            return -1;
        }

        /* Copy the error code value. */
        *errCode = *((int32_t*)args[2].argBuffer);

        /* Free the JOB Instance. */
        if (Josh_freeJobInstance(ptrNetfpClient->joshHandle, jobId) < 0)
        {
            *errCode = NETFP_EJOSH;
            return -1;
        }

        /* Ensure that the unconnect was successful? Error code has already been populated */
        if (result != 0)
            return -1;
    }

    /* Single Core Critical Section Enter: */
    context = ptrNetfpClient->cfg.enterCS();

    /* Remove the socket from the used list. */
    Netfp_listRemoveNode ((Netfp_ListNode**)&ptrNetfpClient->ptrSocketList, (Netfp_ListNode*)ptrNetfpSocket);

    /* Single Core Critical Section Exit: */
    ptrNetfpClient->cfg.exitCS(context);

    /* Cleanup the memory associated with the socket */
    ptrNetfpClient->cfg.free(ptrNetfpSocket, sizeof(Netfp_Socket));
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get configuration information from sockets
 *
 *  @param[in]  sockHandle
 *      Socket handle
 *  @param[out] ptrOptInfo
 *      Option info in TLV format which is populated with the requested information
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_getSockOpt
(
    Netfp_SockHandle    sockHandle,
    Netfp_OptionTLV*    ptrOptInfo,
    int32_t*            errCode
)
{
    Netfp_Socket*            ptrNetfpSocket;
    Netfp_SockL2ConnectInfo* ptrActiveL2ConnectInfo;

    /* Sanity Check: Validate the arguments: */
    if ((sockHandle == NULL) || (ptrOptInfo == NULL))
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Get the pointer to the socket.*/
    ptrNetfpSocket = (Netfp_Socket*)sockHandle;

    /* Process the supported options */
    switch (ptrOptInfo->type)
    {
        case Netfp_Option_PRIORITY:
        {
            /* Get the socket priority. */
            *(uint8_t*)ptrOptInfo->value = ptrNetfpSocket->priority;
            ptrOptInfo->length = 1;
            break;
        }
        case Netfp_Option_PRIORITYTAG:
        {
            /* Get the active L2 connect information: */
            ptrActiveL2ConnectInfo = &ptrNetfpSocket->connectInfo.l2Info[ptrNetfpSocket->connectInfo.cfgIndex];

            if( ptrActiveL2ConnectInfo->vlanId != 0 )
            {
                *errCode = NETFP_EINVAL;
                return -1;
            }

            ptrOptInfo->length = 1;

            /* Tagging not enabled, return current (default) value and error */
            if ((ptrNetfpSocket->priorityTag & NETFP_VLAN_PRIORITYTAG_ENABLED) == ~NETFP_VLAN_PRIORITYTAG_ENABLED )
            {
                *(uint8_t*)ptrOptInfo->value = ptrNetfpSocket->priorityTag & NETFP_VLAN_PRIORITYTAG_MASK;
                *errCode = NETFP_ENOTREADY;
                return -1;
            }

            /* Get the socket priority value bits w/o translation */
            *(uint8_t*)ptrOptInfo->value = ptrNetfpSocket->priorityTag & NETFP_VLAN_PRIORITYTAG_MASK;

            break;
        }
        case Netfp_Option_DONT_FRAG:
        {
            /* Get the socket dont fragment option */
            *(uint8_t*)ptrOptInfo->value = ptrNetfpSocket->dontFrag;
            ptrOptInfo->length = 1;
            break;
        }
        case Netfp_Option_SOCK_MTU:
        {
            /* Sanity Check: Validate the arguments */
            if (ptrOptInfo->length != sizeof(uint32_t))
            {
                *errCode = NETFP_EINVAL;
                return -1;
            }

            /* Get the active L2 connect information and get the MTU associated with the socket. */
            ptrActiveL2ConnectInfo = &ptrNetfpSocket->connectInfo.l2Info[ptrNetfpSocket->connectInfo.cfgIndex];
            *(uint32_t*)ptrOptInfo->value = ptrActiveL2ConnectInfo->mtu;
            break;
        }
        case Netfp_Option_STATE:
        {
            /* Get the socket state. */
            if (ptrNetfpSocket->status == Netfp_Status_ACTIVE)
                *(uint8_t*)ptrOptInfo->value = 1;
            else
                *(uint8_t*)ptrOptInfo->value = 0;
            ptrOptInfo->length = 1;
            break;
        }
        case Netfp_Option_SWITCH_PORT:
        {
            /* Sanity Check: Validate the arguments */
            if (ptrOptInfo->length != sizeof(uint32_t))
            {
                *errCode = NETFP_EINVAL;
                return -1;
            }

            /* Get the active L2 connect information: */
            ptrActiveL2ConnectInfo = &ptrNetfpSocket->connectInfo.l2Info[ptrNetfpSocket->connectInfo.cfgIndex];

            /* Get the socket state: INACTIVE sockets are not associated with an interface */
            if (ptrNetfpSocket->status == Netfp_Status_ACTIVE)
                *(uint32_t*)ptrOptInfo->value = ptrActiveL2ConnectInfo->switchPortNum;
            else
                *(uint32_t*)ptrOptInfo->value = NETFP_INVALID_SWITCH_PORT;
            break;
        }
        case Netfp_Option_VLAN_ID:
        {
            /* Sanity Check: Validate the arguments */
            if (ptrOptInfo->length != sizeof(uint16_t))
            {
                *errCode = NETFP_EINVAL;
                return -1;
            }

            /* Get the active L2 connect information: */
            ptrActiveL2ConnectInfo = &ptrNetfpSocket->connectInfo.l2Info[ptrNetfpSocket->connectInfo.cfgIndex];

            /* Populate the VLAN identifier. */
            if (ptrNetfpSocket->status == Netfp_Status_ACTIVE)
                *(uint16_t*)ptrOptInfo->value = ptrActiveL2ConnectInfo->vlanId;
            else
                *(uint16_t*)ptrOptInfo->value = 0x0;
            break;
        }
        case Netfp_Option_PRIORITY_STATISTICS:
        {
            /* Sanity Check: Validate the arguments. */
            if (ptrOptInfo->length != sizeof(Netfp_SocketStats))
            {
                *errCode = NETFP_EINVAL;
                return -1;
            }

            /* Copy over the socket priority statistics and pass it back to the application. */
            memcpy ((void*)ptrOptInfo->value, (void *)&ptrNetfpSocket->extendedStats.priorityStats[ptrNetfpSocket->priority], sizeof(Netfp_SocketStats));
            break;
        }
        case Netfp_Option_TOTAL_STATISTICS:
        {
            /* Sanity Check: Validate the arguments. */
            if (ptrOptInfo->length != sizeof(Netfp_SocketStats))
            {
                *errCode = NETFP_EINVAL;
                return -1;
            }

            /* Copy over the total socket statistics and pass it back to the application. */
            memcpy ((void*)ptrOptInfo->value, (void *)&ptrNetfpSocket->extendedStats.totalStats, sizeof(Netfp_SocketStats));
            break;
        }
        case Netfp_Option_EXTENDED_STATISTICS:
        {
            /* Sanity Check: Validate the arguments. */
            if (ptrOptInfo->length != sizeof(Netfp_ExtendedSocketStats))
            {
                *errCode = NETFP_EINVAL;
                return -1;
            }
            /* Copy over the extended statistics and pass it back to the application. */
            memcpy ((void*)ptrOptInfo->value, (void *)&ptrNetfpSocket->extendedStats, sizeof(Netfp_ExtendedSocketStats));
            break;
        }
        case Netfp_Option_TX_CHECKSUM_OFFLOAD:
        {
            /* Sanity Check: Validate the argument */
            if (ptrOptInfo->length != sizeof(uint8_t))
            {
                *errCode = NETFP_EINVAL;
                return -1;
            }

            /* Get the udp checksum offload setting from socket */
            *(uint8_t*)ptrOptInfo->value = ptrNetfpSocket->udpChksumOffload;
            break;
        }
        default:
        {
            /* Option is NOT supported */
            *errCode = NETFP_EINVAL;
            return -1;
        }
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to configure the sockets.
 *
 *  @param[in]  sockHandle
 *      Socket handle
 *  @param[in] ptrOptInfo
 *      Option info in TLV format which is to be configured
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_setSockOpt
(
    Netfp_SockHandle    sockHandle,
    Netfp_OptionTLV*    ptrOptInfo,
    int32_t*            errCode
)
{
    Netfp_Socket*            ptrNetfpSocket;
    uint8_t                  socketPriority;
    int8_t                   priorityTag;
    Netfp_SockL2ConnectInfo* ptrActiveL2ConnectInfo;

    /* Sanity Check: Validate the arguments: */
    if ((sockHandle == NULL) || (ptrOptInfo == NULL))
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Get the pointer to the socket.*/
    ptrNetfpSocket = (Netfp_Socket*)sockHandle;

    /* Process the supported options */
    switch (ptrOptInfo->type)
    {
        case Netfp_Option_PRIORITYTAG:
        {
            /* Sanity Check: Validate the argument */
            if (ptrOptInfo->length != sizeof(int8_t))
            {
                *errCode = NETFP_EINVAL;
                return -1;
            }

            /* Sanity Check: Validate the argument */
            priorityTag = *(int8_t*)ptrOptInfo->value;
            if (priorityTag > 7 || priorityTag < -1 )
            {
                *errCode = NETFP_EINVAL;
                return -1;
            }

            ptrActiveL2ConnectInfo = &ptrNetfpSocket->connectInfo.l2Info[ptrNetfpSocket->connectInfo.cfgIndex];

            /* Sanity Check: Not supported when VLAN id != 0 */
            if( ptrActiveL2ConnectInfo->vlanId != 0)
            {
                *errCode = NETFP_EINVAL;
                return -1;
            }

            if (priorityTag == -1)
            {
                /* Disable tagging */
                ptrNetfpSocket->priorityTag = NETFP_VLAN_PRIORITYTAG_DEFAULT & ~NETFP_VLAN_PRIORITYTAG_ENABLED;
            }
            else
            {
                /* No translation */
                ptrNetfpSocket->priorityTag = priorityTag | NETFP_VLAN_PRIORITYTAG_ENABLED;
            }
            break;
        }
        case Netfp_Option_PRIORITY:
        {
            /* Sanity Check: Validate the argument */
            if (ptrOptInfo->length != sizeof(uint8_t))
            {
                *errCode = NETFP_EINVAL;
                return -1;
            }

            /* Sanity Check: Validate the argument */
            socketPriority = *(uint8_t*)ptrOptInfo->value;
            if (socketPriority > 63)
            {
                *errCode = NETFP_EINVAL;
                return -1;
            }

            /* Set the socket priority. */
            ptrNetfpSocket->priority = socketPriority;

            /* L2 & L3 headers are dependent on socket priority so update them.
               The rest is recreated to be safe for potential update of connectInfo.cfgIndex */
            populateNewConfig(ptrNetfpSocket, ptrNetfpSocket->connectInfo.cfgIndex);

            break;
        }
        case Netfp_Option_DONT_FRAG:
        {
            /* Sanity Check: Validate the argument */
            if (ptrOptInfo->length != sizeof(uint8_t))
            {
                *errCode = NETFP_EINVAL;
                return -1;
            }

            /* Set the socket DF option */
            ptrNetfpSocket->dontFrag = *(uint8_t*)ptrOptInfo->value;
            updateIpHeaderDontFrag(ptrNetfpSocket);
            break;
        }
        case Netfp_Option_PRIORITY_STATISTICS:
        {
            /* Sanity Check: Validate the argument */
            if (ptrOptInfo->length != sizeof(uint8_t))
            {
                *errCode = NETFP_EINVAL;
                return -1;
            }

            /* Clear out the statistics for the specific socket priority */
            ptrNetfpSocket->extendedStats.priorityStats[ptrNetfpSocket->priority].outIPPackets = 0;
            ptrNetfpSocket->extendedStats.priorityStats[ptrNetfpSocket->priority].outIPOctets  = 0;
            ptrNetfpSocket->extendedStats.priorityStats[ptrNetfpSocket->priority].outEthOctets = 0;
            break;
        }
        case Netfp_Option_TX_CHECKSUM_OFFLOAD:
        {
            /* Sanity Check: Validate the argument */
            if (ptrOptInfo->length != sizeof(uint8_t))
            {
                *errCode = NETFP_EINVAL;
                return -1;
            }

            /* Set the udpChksumOffload flag */
            if ( (*(uint8_t*)ptrOptInfo->value == 1) ||
                 (*(uint8_t*)ptrOptInfo->value == 0) )
                ptrNetfpSocket->udpChksumOffload = *(uint8_t*)ptrOptInfo->value;
            else
            {
                *errCode = NETFP_EINVAL;
                return -1;
            }

            break;
        }
        case Netfp_Option_FRAME_PROTOCOL_CRC_OFFLOAD:
        {
            /* Check if the socket is Frame Protocol type */
            if (ptrNetfpSocket->bindInfo.l3GPPcfg.protocol != Netfp_3GPPProto_FRAME_PROTOCOL)
            {
                /* Error: Not a Frame Protocol socket. CRC cannot be enabled. */
                *errCode = NETFP_EINVAL;
                return -1;
            }


            /* Sanity Check: Validate the argument */
            if (ptrOptInfo->length != sizeof(uint8_t))
            {
                *errCode = NETFP_EINVAL;
                return -1;
            }

            /* Set the frameProtoCrcOffload flag */
            if ((*(uint8_t*)ptrOptInfo->value == 1) ||
                 (*(uint8_t*)ptrOptInfo->value == 0))
                    ptrNetfpSocket->frameProtoCrcOffload = *(uint8_t*)ptrOptInfo->value;
            else
            {
                *errCode = NETFP_EINVAL;
                return -1;
            }
            break;
        }
        case Netfp_Option_SOCK_3GPP_CFG:
        {
            Netfp_Sock3GPPCfg*      ptrCfg;

            /* Check the socket state */
            if (ptrNetfpSocket->sockState & NETFP_SOCKET_BOUND)
            {
                /* Cannot reconfigure once socket is bound */
                *errCode = NETFP_EINVAL;
                return -1;
            }

            /* Sanity Check: Validate the argument */
            if (ptrOptInfo->length != sizeof(Netfp_Sock3GPPCfg))
            {
                *errCode = NETFP_EINVAL;
                return -1;
            }

            ptrCfg = (Netfp_Sock3GPPCfg*)ptrOptInfo->value;

            if (ptrCfg->protocol == Netfp_3GPPProto_FRAME_PROTOCOL)
            {
                memcpy ((void *)&ptrNetfpSocket->bindInfo.l3GPPcfg, ptrCfg, sizeof (Netfp_Sock3GPPCfg));
            }
            else
            {
                *errCode = NETFP_ENOTIMPL;
                return -1;
            }
            break;
        }
        default:
        {
            /* Option is NOT supported */
            *errCode = NETFP_EINVAL;
            return -1;
        }
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Utility function which converts the reason to a string which describes the
 *      reason code.
 *
 *  @param[in]  reason
 *      Reason code which is converted to a string
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Reason
 */
const char* Netfp_getReasonString(Netfp_Reason reason)
{
    switch (reason)
    {
        case Netfp_Reason_FAST_PATH_DELETE:
        {
            return "Fast Path deleted";
        }
        case Netfp_Reason_SA_DELETE:
        {
            return "Security association deleted";
        }
        case Netfp_Reason_SP_DELETE:
        {
            return "Security policy deleted";
        }
        case Netfp_Reason_SP_INACTIVE:
        {
            return "Security policy inactive";
        }
        case Netfp_Reason_SP_ACTIVE:
        {
            return "Security policy active";
        }
        case Netfp_Reason_INTERFACE_DELETE:
        {
            return "Interface deleted";
        }
        case Netfp_Reason_NEIGH_REACHABLE:
        {
            return "Neighbor reachable";
        }
        case Netfp_Reason_NEIGH_UNREACHABLE:
        {
            return "Neighbor unreachable";
        }
        case Netfp_Reason_IF_MTU_CHANGE:
        {
            return "Interface MTU change";
        }
        case Netfp_Reason_PMTU_CHANGE:
        {
            return "Path MTU change";
        }
        case Netfp_Reason_IF_DOWN:
        {
            return "Interface down";
        }
        case Netfp_Reason_IF_UP:
        {
            return "Interface up";
        }
        case Netfp_Reason_DSCP_MAPPING:
        {
            return "DSCP Mapping change";
        }
        case Netfp_Reason_VLAN_EGRESS:
        {
            return "VLAN Mapping change";
        }
        case Netfp_Reason_L3_QOS:
        {
            return "L3 QOS configuration change";
        }
        case Netfp_Reason_REKEY_SA:
        {
            return "Rekey SA";
        }
        default:
        {
            return "Unknown";
        }
    }
}

/**
 *  @b Description
 *  @n
 *      The function is used to use the event information to generate a reason which
 *      can be exported to the application
 *
 *  @param[in]  ptrEventMetaInfo
 *      Pointer to the event meta information used to generate a reason
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Reason
 */
static Netfp_Reason Netfp_getReason (Netfp_EventMetaInfo* ptrEventMetaInfo)
{
    /* Was the fast path deleted? */
    if (ptrEventMetaInfo->eventId == Netfp_EventId_DELETE_FP)
    {
        /* YES. Fast paths can get deleted because of multiple reasons. Get the reason for
         * the deletion. */
        return ptrEventMetaInfo->u.fpMeta.reason;
    }

    /* Was the fast path updated? */
    if (ptrEventMetaInfo->eventId == Netfp_EventId_UPDATE_FP)
    {
        /* YES. Fast paths can get updated because of multiple reasons. Get the reason for
         * the updation. */
        return ptrEventMetaInfo->u.fpMeta.reason;
    }

    /* Was the interface updated? */
    if (ptrEventMetaInfo->eventId == Netfp_EventId_UPDATE_INTERFACE)
    {
        /* The NETFP Interface module generates events for multiple reasons */
        switch (ptrEventMetaInfo->u.ifMeta.reason)
        {
            case Netfp_EventIfReason_STATUS:
            {
                /* Interface status was updated: */
                if (ptrEventMetaInfo->u.ifMeta.u.status == 1)
                    return Netfp_Reason_IF_UP;
                else
                    return Netfp_Reason_IF_DOWN;
            }
            case Netfp_EventIfReason_INNER_OUTER_DSCP:
            {
                /* Inner-Outer DSCP mapping has been modified. */
                return Netfp_Reason_DSCP_MAPPING;
            }
            case Netfp_EventIfReason_VLAN_PBIT:
            {
                /* VLAN Pbit mapping has been modified. */
                return Netfp_Reason_VLAN_EGRESS;
            }
            case Netfp_EventIfReason_L3_QOS:
            {
                /* L3 QOS mapping has been modified. */
                return Netfp_Reason_L3_QOS;
            }
            default:
            {
                /* Catch all: Unknown reason. */
                return Netfp_Reason_UNKNOWN;
            }
        }
    }

    /* Catch all: Unknown reason */
    return Netfp_Reason_UNKNOWN;
}

/**
 *  @b Description
 *  @n
 *      Event handle for the INTERFACE Update event. This cycles through all the Layer4
 *      endpoints and updates the configuration. This event does not impact the operation
 *      of the sockets.
 *
 *  @param[in]  ptrNetfpClient
 *      Pointer to the NETFP client
 *  @param[in]  ptrEventMetaInfo
 *      Pointer to the event meta information
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
void Netfp_socketUpdateInterfaceHandler
(
    Netfp_ClientMCB*        ptrNetfpClient,
    Netfp_EventMetaInfo*    ptrEventMetaInfo
)
{
    Netfp_Socket*               ptrNetfpSocket;
    void*                       context;
    Netfp_SockL2ConnectInfo*    ptrNewSockL2ConnectInfo;
    Netfp_SockL2ConnectInfo*    ptrActiveSockL2ConnectInfo;

    /* Single Core Critical Section Enter: The socket database is a shared resource */
    context = ptrNetfpClient->cfg.enterCS();

    /* Get the head of the sockets for the NETFP client. */
    ptrNetfpSocket = (Netfp_Socket*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpClient->ptrSocketList);
    while (ptrNetfpSocket != NULL)
    {
        /* Get the socket connect information which is currently active. */
        ptrActiveSockL2ConnectInfo = &ptrNetfpSocket->connectInfo.l2Info[ptrNetfpSocket->connectInfo.cfgIndex];

        /* Interface update events are only applicable for connected sockets and sockets which map to
         * the interface handle which is being updated. Is this socket effected by the event? */
        if ((ptrNetfpSocket->sockState & NETFP_SOCKET_CONNECTED) &&
            (ptrActiveSockL2ConnectInfo->ifHandle == ptrEventMetaInfo->u.ifMeta.ifHandle))
        {
            /* YES. Socket is affected so derive the new configuration block. */
            uint32_t secondaryCfgIndex = (ptrNetfpSocket->connectInfo.cfgIndex == 1)?0:1; //fzm
            ptrNewSockL2ConnectInfo = &ptrNetfpSocket->connectInfo.l2Info[secondaryCfgIndex]; //fzm

            /* Copy over the configuration from the active block. */
            memcpy ((void*)ptrNewSockL2ConnectInfo, (void *)ptrActiveSockL2ConnectInfo, sizeof(Netfp_SockL2ConnectInfo));

            /* fzm - Copy over the outer source and destnation IP addresses */
            ptrNetfpSocket->connectInfo.outerIPSrc[secondaryCfgIndex] =
                ptrNetfpSocket->connectInfo.outerIPSrc[ptrNetfpSocket->connectInfo.cfgIndex];
            ptrNetfpSocket->connectInfo.outerIPDst[secondaryCfgIndex] =
                ptrNetfpSocket->connectInfo.outerIPDst[ptrNetfpSocket->connectInfo.cfgIndex];

            /* Now we update only the new configuration: */
            switch (ptrEventMetaInfo->u.ifMeta.reason)
            {
                case Netfp_EventIfReason_STATUS:
                {
                    /* Update to the new status */
                    ptrNewSockL2ConnectInfo->ifStatus = ptrEventMetaInfo->u.ifMeta.u.status;

                    /* Inherit the socket status from the interface */
                    if (ptrNewSockL2ConnectInfo->ifStatus == 1)
                        ptrNetfpSocket->status = Netfp_Status_ACTIVE;
                    else
                        ptrNetfpSocket->status = Netfp_Status_ZOMBIE;
                    break;
                }
                case Netfp_EventIfReason_INNER_OUTER_DSCP:
                {
                    /* Update to the new Inner to Outer DSCP mapping */
                    memcpy ((void*)&ptrNewSockL2ConnectInfo->innerToOuterDSCPMap[0],
                            (void*)&ptrEventMetaInfo->u.ifMeta.u.innerToOuterDSCPMap,
                            sizeof(ptrNewSockL2ConnectInfo->innerToOuterDSCPMap));
                    break;
                }
                case Netfp_EventIfReason_VLAN_PBIT:
                {
                    /* Update to the new VLAN Priority bit mapping */
                    memcpy ((void*)&ptrNewSockL2ConnectInfo->vlanMap,
                            (void*)&ptrEventMetaInfo->u.ifMeta.u.vlanMap,
                            sizeof(Netfp_VLANPriorityMap));
                    break;
                }
                case Netfp_EventIfReason_L3_QOS:
                {
                    /* Update to the new L3 QOS configuration */
                    memcpy ((void*)&ptrNewSockL2ConnectInfo->l3QosCfg,
                            (void*)&ptrEventMetaInfo->u.ifMeta.u.l3QosCfg,
                            sizeof(Netfp_L3QoSCfg));
                    break;
                }
            }

            /*  We switch to the new configuration so update all headers,
                even if some are not touched at all for specific event */
            populateNewConfig(ptrNetfpSocket, secondaryCfgIndex);

            /* Switch the configuration now: */
            ptrNetfpSocket->connectInfo.cfgIndex = secondaryCfgIndex; //fzm

            /* Notify the application: We need to map the event identifier to generate a reason: */
            if (ptrNetfpSocket->localSockAddr.op.bind.notifyFunction != NULL)
                ptrNetfpSocket->localSockAddr.op.bind.notifyFunction (Netfp_getReason(ptrEventMetaInfo), ptrNetfpSocket);
        }
        /* Get the next socket. */
        ptrNetfpSocket = (Netfp_Socket*)Netfp_listGetNext ((Netfp_ListNode*)ptrNetfpSocket);
    }

    /* Single Core Critical Section Exit: */
    ptrNetfpClient->cfg.exitCS(context);
    return;
}

/**
 *  @b Description
 *  @n
 *      Event handle for the INTERFACE Fast Path event. This cycles through all the Layer4
 *      endpoints and updates the configuration. This event does not impact the operation
 *      of the sockets.
 *
 *  @param[in]  ptrNetfpClient
 *      Pointer to the NETFP client
 *  @param[in]  ptrEventMetaInfo
 *      Pointer to the received event meta information
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
void Netfp_socketUpdateFastPathHandler
(
    Netfp_ClientMCB*        ptrNetfpClient,
    Netfp_EventMetaInfo*    ptrEventMetaInfo
)
{
    Netfp_Socket*               ptrNetfpSocket;
    void*                       context;
    Netfp_SockL2ConnectInfo*    ptrNewSockL2ConnectInfo;
    Netfp_SockL2ConnectInfo*    ptrActiveSockL2ConnectInfo;

    /* Single Core Critical Section Enter: The socket database is a shared resource */
    context = ptrNetfpClient->cfg.enterCS();

    /* Get the head of the sockets for the NETFP client. */
    ptrNetfpSocket = (Netfp_Socket*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpClient->ptrSocketList);
    while (ptrNetfpSocket != NULL)
    {
        /* FAST Path has been updated: Is this an INBOUND or OUTBOUND fast path? */
        if ((ptrNetfpSocket->sockState & NETFP_SOCKET_BOUND) &&
            (ptrNetfpSocket->bindInfo.ingressFastPathHandle == ptrEventMetaInfo->u.fpMeta.fpHandle))
        {
            /* INBOUND Fast Path was updated: Inherit the fast path status */
            ptrNetfpSocket->status = ptrEventMetaInfo->u.fpMeta.status;

            /* Use the notify function: */
            if (ptrNetfpSocket->localSockAddr.op.bind.notifyFunction != NULL)
                ptrNetfpSocket->localSockAddr.op.bind.notifyFunction (Netfp_getReason(ptrEventMetaInfo), ptrNetfpSocket);
        }

        /* Is this an OUTBOUND Fast Path? */
        if ((ptrNetfpSocket->sockState & NETFP_SOCKET_CONNECTED) &&
            (ptrNetfpSocket->connectInfo.egressFastPathHandle == ptrEventMetaInfo->u.fpMeta.fpHandle))
        {
            /* YES. Derive the new configuration block. */
            uint32_t secondaryCfgIndex = (ptrNetfpSocket->connectInfo.cfgIndex == 1)?0:1;
            ptrNewSockL2ConnectInfo = &ptrNetfpSocket->connectInfo.l2Info[secondaryCfgIndex];

            /* Get the socket connect information which is currently active. */
            ptrActiveSockL2ConnectInfo = &ptrNetfpSocket->connectInfo.l2Info[ptrNetfpSocket->connectInfo.cfgIndex];

            /* Copy over the configuration from the active block. */
            memcpy ((void*)ptrNewSockL2ConnectInfo, (void *)ptrActiveSockL2ConnectInfo, sizeof(Netfp_SockL2ConnectInfo));

            /* If this is a SA update, then copy the new address.  Otherwise, copy over the existing one.
               source and destnation IP addresses */
            if(ptrEventMetaInfo->u.fpMeta.reason == Netfp_Reason_SP_ACTIVE)
            {
                ptrNetfpSocket->connectInfo.outerIPSrc[secondaryCfgIndex] = ptrEventMetaInfo->u.fpMeta.srcIP;
                ptrNetfpSocket->connectInfo.outerIPDst[secondaryCfgIndex] = ptrEventMetaInfo->u.fpMeta.dstIP;
            } else {
                ptrNetfpSocket->connectInfo.outerIPSrc[secondaryCfgIndex] =
                    ptrNetfpSocket->connectInfo.outerIPSrc[ptrNetfpSocket->connectInfo.cfgIndex];
                ptrNetfpSocket->connectInfo.outerIPDst[secondaryCfgIndex] =
                    ptrNetfpSocket->connectInfo.outerIPDst[ptrNetfpSocket->connectInfo.cfgIndex];
            }

            /* Now update the status of the fast path: */
            if (ptrEventMetaInfo->u.fpMeta.status == Netfp_Status_ACTIVE)
            {
                /* Fast Path is active */
                ptrNetfpSocket->status = Netfp_Status_ACTIVE;

                /* Copy over the L2 connect information into the new configuration block: */
                memcpy ((void*)ptrNewSockL2ConnectInfo, (void *)&ptrEventMetaInfo->u.fpMeta.l2ConnectInfo,
                        sizeof(Netfp_SockL2ConnectInfo));

                populateNewConfig(ptrNetfpSocket, secondaryCfgIndex);
            }
            else
            {
                /* Fast Path is NOT active. */
                ptrNetfpSocket->status = Netfp_Status_ZOMBIE;

                /* No need to overwrite the L2 connect information. Marking of the socket as a ZOMBIE is sufficient.
                 * Once the socket is operational the L2 connect information will be populated back. */
            }

            /* Switch the configuration now: */
            ptrNetfpSocket->connectInfo.cfgIndex = secondaryCfgIndex;

            /* Notify the application: We need to map the event identifier to generate a reason: */
            if (ptrNetfpSocket->localSockAddr.op.bind.notifyFunction != NULL)
                ptrNetfpSocket->localSockAddr.op.bind.notifyFunction (Netfp_getReason(ptrEventMetaInfo), ptrNetfpSocket);
        }

        /* Get the next socket. */
        ptrNetfpSocket = (Netfp_Socket*)Netfp_listGetNext ((Netfp_ListNode*)ptrNetfpSocket);
    }

    /* Single Core Critical Section Exit: */
    ptrNetfpClient->cfg.exitCS(context);
    return;
}

/**
 *  @b Description
 *  @n
 *      Event Handler for the delete fast path event. This cycles through all the sockets
 *      and marks the sockets of as ZOMBIES. There is no recovery from this event.
 *
 *  @param[in]  ptrNetfpClient
 *      Pointer to the NETFP client
 *  @param[in]  ptrEventMetaInfo
 *      Pointer to the event meta information
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
void Netfp_socketDeleteFastPathHandler
(
    Netfp_ClientMCB*        ptrNetfpClient,
    Netfp_EventMetaInfo*    ptrEventMetaInfo
)
{
    Netfp_Socket*   ptrNetfpSocket;
    void*           context;
    Netfp_Socket*   ptrInfectedSocketList;

    /* Initialize the infected socket list */
    ptrInfectedSocketList = NULL;

    /* Single Core Critical Section Enter: The socket database is a shared resource */
    context = ptrNetfpClient->cfg.enterCS();

    /* Get the head of the sockets for the NETFP client. */
    ptrNetfpSocket = (Netfp_Socket*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpClient->ptrSocketList);
    while (ptrNetfpSocket != NULL)
    {
        /* Fast Path has been deleted. Is the socket affected? Sockets are always BOUND. */
        if (ptrNetfpSocket->bindInfo.ingressFastPathHandle == ptrEventMetaInfo->u.fpMeta.fpHandle)
        {
            /* YES. Inbound fast path handle associated with the socket has been deleted. The socket
             * is removed from the socket list and moved to the infected socket list */
            Netfp_Socket* ptrNextNetfpSocket = (Netfp_Socket*)Netfp_listGetNext ((Netfp_ListNode*)ptrNetfpSocket);

            Netfp_listRemoveNode ((Netfp_ListNode**)&ptrNetfpClient->ptrSocketList, (Netfp_ListNode*)ptrNetfpSocket);
            Netfp_listAdd ((Netfp_ListNode**)&ptrInfectedSocketList, (Netfp_ListNode*)ptrNetfpSocket);

            /* Move to the next socket on the list. */
            ptrNetfpSocket = ptrNextNetfpSocket;
        }
        else if ((ptrNetfpSocket->sockState & NETFP_SOCKET_CONNECTED) &&
                 (ptrNetfpSocket->connectInfo.egressFastPathHandle == ptrEventMetaInfo->u.fpMeta.fpHandle))
        {
            /* YES. Outbound fast path handle associated with the socket has been deleted. The socket
             * is removed from the socket list and moved to the infected socket list */
            Netfp_Socket* ptrNextNetfpSocket = (Netfp_Socket*)Netfp_listGetNext ((Netfp_ListNode*)ptrNetfpSocket);

            Netfp_listRemoveNode ((Netfp_ListNode**)&ptrNetfpClient->ptrSocketList, (Netfp_ListNode*)ptrNetfpSocket);
            Netfp_listAdd ((Netfp_ListNode**)&ptrInfectedSocketList, (Netfp_ListNode*)ptrNetfpSocket);

            /* Move to the next socket on the list. */
            ptrNetfpSocket = ptrNextNetfpSocket;
        }
        else
        {
            /* Get the next socket. */
            ptrNetfpSocket = (Netfp_Socket*)Netfp_listGetNext ((Netfp_ListNode*)ptrNetfpSocket);
        }
    }

    /* All the infected sockets are now in the infected list. Cycle through these sockets and notify the
     * application about the infection */
    ptrNetfpSocket = (Netfp_Socket*)Netfp_listRemove ((Netfp_ListNode**)&ptrInfectedSocketList);
    while (ptrNetfpSocket != NULL)
    {
        /* Move this back to the socket list
         * - This is because we use the Netfp_closeSocket to delete the socket and the function
         *   expects the socket to be in the socket list */
        Netfp_listAdd ((Netfp_ListNode**)&ptrNetfpClient->ptrSocketList, (Netfp_ListNode*)ptrNetfpSocket);

        /* Mark the socket as a ZOMBIE. */
        ptrNetfpSocket->status = Netfp_Status_ZOMBIE;

        /* Notify the application: We need to map the event identifier to generate a reason: */
        if (ptrNetfpSocket->localSockAddr.op.bind.notifyFunction != NULL)
            ptrNetfpSocket->localSockAddr.op.bind.notifyFunction (Netfp_getReason(ptrEventMetaInfo), ptrNetfpSocket);

        /* Get the new head of the socket. */
        ptrNetfpSocket = (Netfp_Socket*)Netfp_listRemove ((Netfp_ListNode**)&ptrInfectedSocketList);
    }

    /* Single Core Critical Section Exit: */
    ptrNetfpClient->cfg.exitCS(context);

    return;
}

/**
 *  @b Description
 *  @n
 *      The function is a debug function to display the contents of the L4 Bindings which
 *      exist on the NETFP Server
 *
 *  @param[in]  serverHandle
 *      Handle to the NETFP Server
 *
 *  \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Number of L4 Bindings
 */
int32_t Netfp_displayL4Binding(Netfp_ServerHandle serverHandle)
{
    Netfp_ServerMCB*            ptrNetfpServer;
    Netfp_Layer4Node*           ptrL4Node;
    Netfp_InboundFastPath*      ptrL4NodeInboundFP;
    const char*                 ptrStatus;
    int32_t                     numL4Bindings = 0;

    /* Get the NETFP Server */
    ptrNetfpServer = (Netfp_ServerMCB*)serverHandle;

//fzm - use Netfp_dumpMsg to capture this with the USR1 signal
    /* Display the banner. */
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "**********************************************************\n");
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "L4 Bindings\n");

    /* Get the head of the L4 Binding */
    ptrL4Node = (Netfp_Layer4Node*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrLayer4List);
    while (ptrL4Node != NULL)
    {
        /* Get the inbound fast path handle from the L4 Node: */
        ptrL4NodeInboundFP = (Netfp_InboundFastPath*)ptrL4Node->sockAddress.op.bind.inboundFPHandle;

        /* It is possible that a fast path has been deleted and we have sent an event to the NETFP Client
         * but the client has still *not* deleted the socket which implies that the L4 Node is still valid
         * So we cannot trust the INBOUND Fast Path which is in the L4 node */
        if (Netfp_isValidInboundFastPath (ptrNetfpServer, ptrL4NodeInboundFP) == 0)
        {
            /* This L4 node resides on a fast path which has already been deleted but the socket
             * has still not been deleted. This L4 node is basically a ZOMBIE and is not to be considered. */
            ptrStatus = "ZOMBIE";
        }
        else
        {
            /* This L4 node resides on a fast path which is active and so this binding is also active */
            ptrStatus = "ACTIVE";
        }

        /* Increment the number of L4 Binding */
        numL4Bindings++;

        /* Is this a GTPU or UDP Port? */
        if (ptrL4Node->sockAddress.sin_gtpuId != 0)
            Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "[%s] GTPU Id : 0x%x on '%s'\n",
                          ptrStatus, ptrL4Node->sockAddress.sin_gtpuId, ptrL4NodeInboundFP->cfg.name);
        else
            Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "[%s] UDP Port: %d on '%s'\n",
                          ptrStatus, ptrL4Node->sockAddress.sin_port, ptrL4NodeInboundFP->cfg.name);

        /* Get the next L4 Binding: */
        ptrL4Node = (Netfp_Layer4Node*)Netfp_listGetNext ((Netfp_ListNode*)ptrL4Node);
    }
    return numL4Bindings;
}

/**
 *  @b Description
 *  @n
 *      The function is used to register the NETFP Socket services
 *
 *  @param[in]  nodeHandle
 *      JOSH node handle for which the jobs are being registered
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_registerSocketServices (Josh_NodeHandle nodeHandle)
{
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Netfp_bind);
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Netfp_secureBind);
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Netfp_unbind);
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Netfp_connect);
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Netfp_unconnect);
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Netfp_suspendSocket);
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Netfp_resumeSocket);
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Netfp_resumeSecureSocket);
    return 0;
}
//fzm-->
/**
 *  @b Description
 *  @n
 *      The function is used to extract vlan id and physical
 *      switch port number assigned to Netfp Socket.
 *
 *  @param[in]  socketHandle
 *      Handle to the NETFP socket for which vlanid and port
 *      number are to be extracted
 *  @param[out]  vlanId
 *      vlan id to which socket socketHandle is assigned
 *  @param[out]  switchPortNum
 *      port number on physical switch to which socket
 *      socketHandle is assigned
 *
 *  \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_getVlanInfo(
    Netfp_SockHandle    socketHandle,
    uint32_t*           vlanId,
    uint32_t*           switchPortNum
)
{
    Netfp_Socket* ptrNetfpSocket = (Netfp_Socket*)socketHandle;
    if (ptrNetfpSocket == NULL)
    {
        *vlanId = 0;
        *switchPortNum = 0;
        return NETFP_EINVAL;
    }
    Netfp_SockL2ConnectInfo* ptrSockL2ConnectInfo = &ptrNetfpSocket->connectInfo.l2Info[ptrNetfpSocket->connectInfo.cfgIndex];
    *switchPortNum = ptrSockL2ConnectInfo->switchPortNum;
    *vlanId = ptrSockL2ConnectInfo->vlanId;
    return 0;
}
//fzm<--
