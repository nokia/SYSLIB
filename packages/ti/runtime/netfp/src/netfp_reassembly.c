/**
 *   @file  netfp_reassembly.c
 *
 *   @brief
 *      The file contains the IP Reassembly routines that interact with the
 *      NetCP to perform IPv4/IPv6 reassembly operation.
 *
 *      This implementation consists of an IP reassembly algorithm which
 *      supports non-overlapping segments only and performs the following tasks:
 *      @li Maintain the IP reassembly contexts consist of source IP,
 *          destination IP, IP identification, protocol, fragments count and the
 *          corresponding traffic flow id.
 *      @li Forward the non-fragmented IP packet with its flow id and
 *          count = 1 to PA PDSP queue. This avoids reordering the non-fragmented
 *          packets.
 *      @li For IPSEC inner IP fragments, call SA LLD to perform the
 *          post-decryption operation including padding check and IPSEC header
 *          and authentication tag removal.
 *      @li Forward the reassembled IP packet with its flow id and fragments
 *          count to PA PDSP queue.
 *      @li Send a null packet with its flow id and fragments count to PA PDSP
 *          queue if the fragments are discarded due to timeout or other error.
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

/* MCSDK Include Files. */
#include <ti/csl/csl_cache.h>

/* SYSLIB Include files */
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/msgcom/msgcom.h>
#include <ti/runtime/netfp/netfp.h>
#include <ti/runtime/netfp/include/netfp_net.h>
#include <ti/runtime/netfp/include/netfp_internal.h>

#ifdef __ARMv7
#include <ti/apps/netfp_config/include/NetFP_System_printf.h> //fzm
#else
#include <xdc/runtime/System.h>
#endif
/**************************************************************************
 ************************** Local Definitions *****************************
 **************************************************************************/

/* MCSDK Patches: These macros are *not* currently supported in the MCSDK. Furture releases
 * of MCSDK will have these macros. We will remove these once those changes are integrated
 * These macros allow the ability to send packets using the raw interface */
#define PA_MCSDK_PATCHES
#ifdef  PA_MCSDK_PATCHES
#ifdef NSS_GEN2

#else
#define PASAHO_LINFO_SET_L3_OFFSET(x, v)       PASAHO_SET_BITFIELD((x)->word2,(v),24,8)    /**< Set the offset to the level 3 header (PASS Gen1)*/
#define PASAHO_LINFO_SET_L4_OFFSET(x, v)       PASAHO_SET_BITFIELD((x)->word2,(v),16,8)    /**< Set the offset to the level 4 header (PASS Gen1)*/
#define PASAHO_LINFO_SET_L5_OFFSET(x, v)       PASAHO_SET_BITFIELD((x)->word2,(v),8,8)     /**< Set the offset to the level 5 header (PASS Gen1)*/
#define PASAHO_LINFO_SET_INNER_IP_OFFSET(x, v) PASAHO_SET_BITFIELD((x)->word4,(v),16,8)    /**< Set the offset to the most inner IP header (PASS Gen1)*/
#endif /* NSS_GEN2 */
#endif /* MCSDK_PATCHES */

/**
 * @brief   Internal flag which is set to indicate that the fragment is the
 * first received fragment
 */
#define FRAG_FIRST                      0x1

/**
 * @brief   Internal flag which is set to indicate that the fragment not the
 * first or last.
 */
#define FRAG_MIDDLE                     0x2

/**
 * @brief   Internal flag which is set to indicate that the fragment is the
 * last fragment
 */
#define FRAG_LAST                       0x4

/**************************************************************************
 ************************** Local Structures ******************************
 **************************************************************************/

/**
 * @brief
 *  IPv6 Extension Header
 *
 * @details
 *  This structure holds the information for each IPv6 Extension header
 *  which has been received
 */
typedef struct Netfp_IPv6ExtHeader
{
    /**
     * @brief   Type of the Extension header
     */
    uint8_t        type;

    /**
     * @brief   Pointer to the start of the extension header
     */
    uint8_t*       ptrHeader;

    /**
     * @brief   Length of the extension header
     */
    uint8_t        length;
}Netfp_IPv6ExtHeader;

/**
 * @brief
 *  Fragment information
 *
 * @details
 *  This structure holds the information for each fragment.
 */
typedef struct Netfp_IPFragmentInfo
{
    /**
     * @brief   Traffic flow index of the fragment.
     */
    uint16_t                    tfIndex;

    /**
     * @brief   Destination queue to which packet has to be forwarded to
     * post reassembly.
     */
    int32_t                     dstQueue;

    /**
     * @brief   Offset to the IP header in the fragment.
     */
    uint16_t                    ipOffset;

    /**
     * @brief   Switch port on which the fragment was received.
     */
    uint8_t                     inPort;

    /**
     * @brief   VLAN Identifier if any contained in the fragment.
     */
    uint16_t                    vlanId;

    /**
     * @brief   Fragment offset field in the IP header.
     */
    uint16_t                    fragOffset;

    /**
     * @brief   Pointer to the IP header (IP Header could be IPv4 or IPv6) depending
     * upon the IP header version.
     */
    uint8_t*                    ptrIPHeader;

    /**
     * @brief   Pointer to the IPv6 Extension Header. This is not used for IPv4.
     */
    Netfp_IPv6ExtHeader         ipv6ExtensionHeader[NETFP_MAX_EXTENSION_HEADER];

    /**
     * @brief   Number of IPv6 Extension headers which are present in the received fragment
     */
    int32_t                     numIPv6ExtHeader;

    /**
     * @brief   Size of all the IPv6 Extension headers which are present in the received fragment
     */
    uint32_t                    sizeIPv6ExtensionHeader;

    /**
     * @brief   Pointer to the IPv6 Fragmentation header:
     */
    Netfp_IPv6FragHeader*       ptrIPv6FragHeader;

    /**
     * @brief   PS information populated by the NETCP subsystem in every received
     * fragment which is pushed to the NETFP client for reassembly
     */
    pasahoLongInfo_t*           pasaInfo;

    /**
     * @brief   Length of the PS information populated by the NETCP subsystem
     * in every received fragment which is pushed to the NETFP client for reassembly
     */
    uint32_t                    pasaInfoLen;

    /**
     * @brief   IP header version.
     */
    Netfp_IPVersion             ipVer;

    /**
     * @brief   Protocol in the received fragment
     */
    uint8_t                     protocol;

    /**
     * @brief   IP identifer
     */
    uint32_t                    id;

    /**
     * @brief   Source IP address in the received fragment
     */
    Netfp_IPAddr                srcAddr;

    /**
     * @brief   Destination IP address in the received fragment
     */
    Netfp_IPAddr                dstAddr;
}Netfp_IPFragmentInfo;

/**************************************************************************
 ************************* Reassembly Functions ***************************
 **************************************************************************/

//fzm-->
typedef enum { STATE_INITIAL = 0x0, STATE_FRAGMENT, STATE_HBH, STATE_ANY, STATE_FAILED, NUM_STATES } state_t;
typedef state_t state_func_t(int next_header);

static state_t do_state_initial(int next_header)
{
    switch(next_header)
    {
        case IPPROTO6_FRAG:
            return STATE_FRAGMENT;
        case IPPROTO6_HOP:
            return STATE_HBH;
        default:
            return STATE_ANY;
    };
}

static state_t do_state_fragment(int next_header)
{
    switch(next_header)
    {
        case IPPROTO6_FRAG: //fall through
        case IPPROTO6_HOP:
            return STATE_FAILED;
        default:
            return STATE_ANY;
    };
}

static state_t do_state_hbh(int next_header)
{
    switch(next_header)
    {
        case IPPROTO6_FRAG:
            return STATE_FRAGMENT;
        case IPPROTO6_HOP:
            return STATE_HBH;
        default:
            return STATE_ANY;
    };
}

static state_t do_state_any(int next_header)
{
    switch(next_header)
    {
        case IPPROTO6_FRAG:
            return STATE_FRAGMENT;
        case IPPROTO6_HOP:
            return STATE_HBH;
        default:
            return STATE_ANY;
    };
}

static state_t do_state_failed(int next_header)
{
    return STATE_FAILED;
}

state_func_t* const state_table[NUM_STATES] = { do_state_initial, do_state_fragment, do_state_hbh, do_state_any, do_state_failed};

state_t run_state(state_t cur_state, int next_header)
{
    return state_table[cur_state](next_header);
}
//fzm<--

/**
 *  @b Description
 *  @n
 *      Internal utility function which indicates if the received packet is
 *      fragmented or not
 *
 *  @param[in]  ptrIPHeader
 *      Pointer to the IP header to be analyzed
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Frag offset - Packet is a fragment
 *  @retval
 *      0           - Packet is not a fragment
 */
static uint16_t Netfp_isIPv4PacketFragmented (Netfp_IPHeader* ptrIPHeader)
{
    /* IPv4 packets are fragmented if the fragmentation offset is non zero or
     * if the MF bit is set */
    return Netfp_ntohs(ptrIPHeader->FlagOff) & IPV4_FRAG_DET_MASK;
}

/**
 *  @b Description
 *  @n
 *      The function parses the IPv6 packet and populates all the detected IPv6 extension headers
 *
 *  @param[in]  ptrIPv6Header
 *      Pointer to the IPv6 header to be analyzed
 *  @param[out] ptrIPv6ExtensionHeader
 *      Populated with the IPv6 Extension headers
 *  @param[out] ptrIPv6FragHdr
 *      Populated with the IPv6 Fragmentation header if the packet is a fragment
 *  @param[out] sizeExtensionHeader
 *      Populated with the size of all the IPv6 extension headers in the packet
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Number of extension headers
 */
static int32_t Netfp_parseIPv6ExtensionHeader
(
    Netfp_IPv6Header*       ptrIPv6Header,
    Netfp_IPv6ExtHeader*    ptrIPv6ExtensionHeader,
    Netfp_IPv6FragHeader**  ptrIPv6FragHdr,
    uint32_t*               sizeExtensionHeader
)
{
    uint8_t     nextHeader;
    uint8_t*    ptrHeader;
    uint8_t     headerLen;
    uint16_t    fragOffset;
    int32_t     doneParsing = 0;
    int32_t     numExtensionHeader = 0;

    /* Initialize the output parameters */
    *ptrIPv6FragHdr      = NULL;
    *sizeExtensionHeader = 0;

    /* Get the next extension header: */
    nextHeader = ptrIPv6Header->NextHeader;

    /* Get the pointer to the first extension header */
    ptrHeader = (uint8_t*)((uint8_t*)ptrIPv6Header + IPv6HDR_SIZE);

    /* Set up the state machine */
    state_t parsingState = STATE_INITIAL; //fzm

    /* The IPv6 Header might carry multiple extension headers. We need to cycle through all the extension
     * headers to determine if the packet is a fragment or not. */
    while (doneParsing == 0)
    {
        /* Did we exceed the maximum allowed? */
        if (numExtensionHeader == NETFP_MAX_EXTENSION_HEADER)
            return -1;

        /* update state state machine */
        if(IPPROTO6_HOP == nextHeader ||
           IPPROTO6_ROUTING == nextHeader ||
           IPPROTO6_FRAG == nextHeader ||
           IPPROTO6_DSTOPTS == nextHeader)
            parsingState = run_state(parsingState, nextHeader); //fzm

        /* Process on the basis of the header: */
        switch (nextHeader)
        {
            case IPPROTO6_HOP:
            {
                /* Hop by Hop Option Extension Header: */
                nextHeader = *ptrHeader;
                headerLen  = *(ptrHeader + 1);

                /* Store the information: */
                ptrIPv6ExtensionHeader->type      = IPPROTO6_HOP;
                ptrIPv6ExtensionHeader->ptrHeader = ptrHeader;
                ptrIPv6ExtensionHeader->length    = ((headerLen + 1) << 3);

                /* Keep track of the total size of all the extension headers */
                *sizeExtensionHeader += ptrIPv6ExtensionHeader->length;

                /* We are done with this entry */
                numExtensionHeader++;
                ptrIPv6ExtensionHeader++;

                /* Skip to the next header */
                ptrHeader = ptrHeader + ((headerLen + 1) << 3);
                break;
            }
            case IPPROTO6_ROUTING:
            {
                /* Routing Extension Header: */
                nextHeader = *ptrHeader;
                headerLen  = *(ptrHeader + 1);

                /* Store the information: */
                ptrIPv6ExtensionHeader->type      = IPPROTO6_ROUTING;
                ptrIPv6ExtensionHeader->ptrHeader = ptrHeader;
                ptrIPv6ExtensionHeader->length    = ((headerLen + 1) << 3);

                /* Keep track of the total size of all the extension headers */
                *sizeExtensionHeader += ptrIPv6ExtensionHeader->length;

                /* We are done with this entry */
                numExtensionHeader++;
                ptrIPv6ExtensionHeader++;

                /* Skip to the next header */
                ptrHeader = ptrHeader + ((headerLen + 1) << 3);
                break;
            }
            case IPPROTO6_FRAG:
            {
                /* Fragmentation Extension Header: */
                nextHeader = *ptrHeader;
                headerLen  = IPV6_FRAGHDR_SIZE;

                /* Store the information: */
                ptrIPv6ExtensionHeader->type      = IPPROTO6_FRAG;
                ptrIPv6ExtensionHeader->ptrHeader = ptrHeader;
                ptrIPv6ExtensionHeader->length    = IPV6_FRAGHDR_SIZE;

                /* Keep track of the total size of all the extension headers */
                *sizeExtensionHeader += ptrIPv6ExtensionHeader->length;

                /* We are done with this entry */
                numExtensionHeader++;
                ptrIPv6ExtensionHeader++;

                /* Store the fragmentation header: */
                *ptrIPv6FragHdr = (Netfp_IPv6FragHeader*)ptrHeader;

                /* Get the fragmentation offset: */
                fragOffset = Netfp_ntohs((*ptrIPv6FragHdr)->FragOffset);

                /* If there are extension headers following the fragmentation header; we follow it only for
                 * the first fragment. For all other fragments this is not considered */
                if ((fragOffset & IPV6_FRAGO_MASK) != 0)
                    doneParsing = 1;

                /* Skip to the next header */
                ptrHeader = ptrHeader + IPV6_FRAGHDR_SIZE;
                break;
            }
            case IPPROTO6_NONE:
            {
                /* No Header: This is the last header; there is no upper layer header. */
                ptrIPv6ExtensionHeader->type      = IPPROTO6_NONE;
                ptrIPv6ExtensionHeader->ptrHeader = ptrHeader;
                ptrIPv6ExtensionHeader->length    = 0;

                /* Keep track of the total size of all the extension headers */
                *sizeExtensionHeader += ptrIPv6ExtensionHeader->length;

                /* We are done with this entry */
                numExtensionHeader++;
                ptrIPv6ExtensionHeader++;

                /* There are no more extension headers after this */
                doneParsing = 1;
                break;
            }
            case IPPROTO6_DSTOPTS:
            {
                /* Destination Option Header: Get the next header */
                nextHeader = *ptrHeader;
                headerLen  = *(ptrHeader + 1);

                /* Store the information: */
                ptrIPv6ExtensionHeader->type      = IPPROTO6_DSTOPTS;
                ptrIPv6ExtensionHeader->ptrHeader = ptrHeader;
                ptrIPv6ExtensionHeader->length    = ((headerLen + 1) << 3);

                /* Keep track of the total size of all the extension headers */
                *sizeExtensionHeader += ptrIPv6ExtensionHeader->length;

                /* We are done with this entry */
                numExtensionHeader++;
                ptrIPv6ExtensionHeader++;

                /* Skip to the next header */
                ptrHeader = ptrHeader + ((headerLen + 1) << 3);
                break;
            }
            default:
            {
                /* All other headers imply that we are done with the parsing of the IPv6 extension headers */
                doneParsing = 1;
                break;
            }
        }
    }

    if(parsingState == STATE_FAILED) //fzm
        return -1;

    return numExtensionHeader;
}

/**
 *  @b Description
 *  @n
 *      The function is used to free the traffic flow which has been configured
 *      in the NETCP.
 *
 *  @param[in]  ptrClientMCB
 *      Pointer to the client MCB
 *  @param[in]  ptrPkt
 *      Pointer to the packet
 *  @param[in]  tfIndex
 *      Traffic flow index associated with the fragment
 *  @param[in]  fragCount
 *      Number of fragments to be cleaned up.
 *  @param[in]  dstQueue
 *      Destination queue where the packet is to be pushed.
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
static void Netfp_freeTrafficFlow
(
    Netfp_ClientMCB*    ptrClientMCB,
    Ti_Pkt*             ptrPkt,
    uint16_t            tfIndex,
    int32_t             fragCount,
    int32_t             dstQueue
)
{
    Netfp_ReassemblyMCB*    ptrReassemblyMCB;
    Ti_Pkt*                 ptrNullPacket;
    pasahoLongInfo_t*       pInfo;
    uint32_t                infoLen;

    /* Is there a valid traffic flow index? */
    if (tfIndex == PA_INV_TF_INDEX)
        return;

    /* Get the reassembly MCB */
    ptrReassemblyMCB = ptrClientMCB->ptrReassemblyMCB;

    /* Reassembly context was acclerated and we needed to flush this from the NETCP subsystem
     * Allocate a packet to send the command. We know that the reassembly heap was allocated
     * to handle at least */
    ptrNullPacket = Pktlib_allocPacket (ptrClientMCB->cfg.pktlibInstHandle, ptrReassemblyMCB->reassemblyHeapHandle,
                                        ETH_MAX_PKT_SIZE);
    if (ptrNullPacket == NULL)
    {
        /* Error: This means that the traffic flow in the NETCP cannot be cleared up.
         * This might imply that the application should configure more packets for the
         * client reassembly configuration. */
        ptrReassemblyMCB->stats.numFreeTrafficFlowFailure++;
        return;
    }

    /* Get the protocol specific information from the packet in the reassembly context */
    Cppi_getPSData (Cppi_DescType_HOST, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc *)ptrPkt, (uint8_t **)&pInfo, &infoLen);

    /* Copy over the PS Information to the NULL packet */
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)ptrNullPacket, (uint8_t *)pInfo, infoLen);
    Cppi_getPSData (Cppi_DescType_HOST, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc *)ptrNullPacket, (uint8_t **)&pInfo, &infoLen);
    PASAHO_LINFO_SET_FRANCNT(pInfo, fragCount);
    PASAHO_LINFO_SET_NULL_PKT_IND(pInfo, 1);

    /* Release ownership of the NULL Packet and push this to the destination queue; the NULL packet will
     * automatically be recycled back into the free queue by the PA CPDMA. */
    Pktlib_releaseOwnership (ptrClientMCB->cfg.pktlibInstHandle, ptrNullPacket);
    Qmss_queuePushDescSize (dstQueue, ptrNullPacket, 128);
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to allocate a reassembly context
 *
 *  @param[in]  ptrReassemblyMCB
 *      Pointer to the client reassembly MCB which is handling the reassembly
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   Allocated reassembly context
 *  @retval
 *      Error   -   NULL
 */
static Netfp_IPReassemblyContext* Netfp_allocateReassemblyContext
(
    Netfp_ReassemblyMCB*    ptrReassemblyMCB
)
{
    Netfp_IPReassemblyContext*  ptrReassemblyContext;

    /* Allocate the reassembly context from the free list  */
    ptrReassemblyContext = (Netfp_IPReassemblyContext*)Netfp_listRemove ((Netfp_ListNode**)&ptrReassemblyMCB->ptrFreeContextList);
    if (ptrReassemblyContext != NULL)
    {
        /* Add the reassembly context to the used list */
        Netfp_listAdd ((Netfp_ListNode**)&ptrReassemblyMCB->ptrUsedContextList, (Netfp_ListNode*)ptrReassemblyContext);

        /* Increment the stats */
        ptrReassemblyMCB->stats.activeReassemblyContexts++;
    }
    return ptrReassemblyContext;
}

/**
 *  @b Description
 *  @n
 *      The function is used to free the reassembly context and to flush out
 *      the traffic flow from the NETCP if required. The packet in the reassembly
 *      context is not touched by the API.
 *
 *  @param[in]  ptrClientMCB
 *      Pointer to the client MCB
 *  @param[in]  ptrReassemblyMCB
 *      Pointer to the client reassembly MCB which is handling the reassembly
 *  @param[in]  ptrReassemblyContext
 *      Pointer to the reassembly context
 *  @param[in]  cleanTrafficFlow
 *      Flag set to 1 to indicate that the traffic flow in the NETCP should also be
 *      cleaned
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
int32_t Netfp_freeReassemblyContext
(
    Netfp_ClientMCB*            ptrClientMCB,
    Netfp_ReassemblyMCB*        ptrReassemblyMCB,
    Netfp_IPReassemblyContext*  ptrReassemblyContext,
    uint32_t                    cleanTrafficFlow
)
{
    /* Do we need to clean the traffic flow in the NETCP subsystem? NOTE: Not all reassembly
     * contexts are handled by the NETCP subsystem */
    if (cleanTrafficFlow == 1)
        Netfp_freeTrafficFlow (ptrClientMCB, ptrReassemblyContext->ptrPkt, ptrReassemblyContext->tfId,
                               ptrReassemblyContext->fragCnt, ptrReassemblyContext->dstQueue);

    /* Decrement the number of active fragments associated with the reassembly context. */
    ptrReassemblyMCB->stats.numActiveFragments -= ptrReassemblyContext->fragCnt;

    /* Remove the reassembly context from the used list and add this to the free list. */
    Netfp_listRemoveNode ((Netfp_ListNode**)&ptrReassemblyMCB->ptrUsedContextList, (Netfp_ListNode*)ptrReassemblyContext);
    Netfp_listAdd ((Netfp_ListNode**)&ptrReassemblyMCB->ptrFreeContextList, (Netfp_ListNode*)ptrReassemblyContext);
    ptrReassemblyMCB->stats.activeReassemblyContexts--;
    return 0;
}

static Netfp_IPReassemblyContext* Netfp_freeAndGetOldestReassemblyContext (Netfp_ClientMCB* ptrClientMCB)
{
    Netfp_ReassemblyMCB* ptrReassemblyMCB = ptrClientMCB->ptrReassemblyMCB;

    /* Get the last (oldest) context and move it to the beginning (simulate allocation) */
    Netfp_IPReassemblyContext* ptrReassemblyContext = (Netfp_IPReassemblyContext*)Netfp_listGetHead ((Netfp_ListNode**)&ptrReassemblyMCB->ptrUsedContextList);
    if (!ptrReassemblyContext)
        return NULL;

    while (ptrReassemblyContext != NULL)
    {
        Netfp_IPReassemblyContext* nextContext = (Netfp_IPReassemblyContext*)Netfp_listGetNext ((Netfp_ListNode*)ptrReassemblyContext);
        if (!nextContext)
            break;

        ptrReassemblyContext = nextContext;
    }

    Ti_Pkt* ptrPkt = ptrReassemblyContext->ptrPkt;

    Netfp_freeTrafficFlow (ptrClientMCB, ptrReassemblyContext->ptrPkt, ptrReassemblyContext->tfId,
                           ptrReassemblyContext->fragCnt, ptrReassemblyContext->dstQueue);

    ptrReassemblyMCB->stats.numActiveFragments -= ptrReassemblyContext->fragCnt;

    Netfp_listRemoveNode ((Netfp_ListNode**)&ptrReassemblyMCB->ptrUsedContextList, (Netfp_ListNode*)ptrReassemblyContext);
    Netfp_listAdd ((Netfp_ListNode**)&ptrReassemblyMCB->ptrUsedContextList, (Netfp_ListNode*)ptrReassemblyContext);

    Pktlib_freePacket (ptrClientMCB->cfg.pktlibInstHandle, ptrPkt);

    return ptrReassemblyContext;
}

/**
 *  @b Description
 *  @n
 *      The function is used to free a received fragment
 *
 *  @param[in]  ptrClientMCB
 *      Pointer to the client MCB
 *  @param[in]  tfId
 *      Traffic Flow identifier associated with the the fragment
 *  @param[in]  dstQueue
 *      Destination Queue where the fragment is to be pushed
 *  @param[in]  ptrFragment
 *      Pointer to the fragment to be cleaned up
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
static void Netfp_freeFragment
(
    Netfp_ClientMCB*    ptrClientMCB,
    uint16_t            tfId,
    Qmss_QueueHnd       dstQueue,
    Ti_Pkt*             ptrFragment
)
{
    pasahoLongInfo_t*   pInfo;
    uint32_t            infoLen;

    /* We dont need to report this to the NETCP if it had already indicated that it was not interested in
     * the traffic flow. */
    if (tfId != PA_INV_TF_INDEX)
    {
        /* Get and update the packet context. */
        Cppi_getPSData (Cppi_DescType_HOST, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc*)ptrFragment, (uint8_t **)&pInfo, &infoLen);
        PASAHO_LINFO_SET_FRANCNT(pInfo, 1);
        PASAHO_LINFO_SET_NULL_PKT_IND(pInfo, 1);
        Pktlib_setPacketLen (ptrFragment, 0);

        /* Release the ownership: */
        Pktlib_releaseOwnership (ptrClientMCB->cfg.pktlibInstHandle, ptrFragment);

        /* Push to the destination queue. This will also result in the fragment getting cleaned up. */
        Qmss_queuePushDescSize (dstQueue, ptrFragment, 128);
    }
    else
    {
        /* Clean the fragment. */
        Pktlib_freePacket(ptrClientMCB->cfg.pktlibInstHandle, ptrFragment);
    }
    return;
}


/**
 *  @b Description
 *  @n
 *      The function is used to find the reassembly context given the fragmentation
 *      information.
 *
 *  @param[in]  ptrReassemblyMCB
 *      Pointer to the client reassembly MCB which is handling the reassembly
 *  @param[in]  ptrFragmentInfo
 *      Fragmentation information which has meta-information regarding the received
 *      fragment.
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Pointer to the matching reassembly context or NULL
 */
static Netfp_IPReassemblyContext* Netfp_findReassemblyContext
(
    Netfp_ReassemblyMCB*    ptrReassemblyMCB,
    Netfp_IPFragmentInfo*   ptrFragmentInfo
)
{
    Netfp_IPReassemblyContext*  ptrReassemblyContext;

    /* Cycle through the active list for a match. */
    ptrReassemblyContext = (Netfp_IPReassemblyContext*)Netfp_listGetHead((Netfp_ListNode**)&ptrReassemblyMCB->ptrUsedContextList);
    while (ptrReassemblyContext != NULL)
    {
        /* Reassembly contexts are a unique tuple of the source ip, destination ip, protocol and IP identifier
         * There is a perfect match only when all the fields match up perfectly. */
        if ((Netfp_matchIP(&ptrFragmentInfo->srcAddr, &ptrReassemblyContext->srcAddr) == 1) &&
            (Netfp_matchIP(&ptrFragmentInfo->dstAddr, &ptrReassemblyContext->dstAddr) == 1) &&
            (ptrReassemblyContext->protocol == ptrFragmentInfo->protocol)                   &&
            (ptrReassemblyContext->id == ptrFragmentInfo->id)                               &&
            (ptrReassemblyContext->dstQueue == ptrFragmentInfo->dstQueue))
        {
            /* Match found. Search is complete. */
            break;
        }

        /* Get the next used reassembly context */
        ptrReassemblyContext = (Netfp_IPReassemblyContext*)Netfp_listGetNext((Netfp_ListNode*)ptrReassemblyContext);
    }
    return ptrReassemblyContext;
}

/**
 *  @b Description
 *  @n
 *      Pre Reassembly Hook: This is invoked on the reception of a fragment.
 *      Fragments could be received on the Inner or Outer channel.
 *
 *  @param[in]  ptrClientMCB
 *      Pointer to the NETFP Client
 *  @param[in]  ptrFragment
 *      Pointer to the received fragment
 *  @param[in]  dstQueue
 *      Destination queue where the reassembled packet is to be pushed
 *  @param[in]  tfIndex
 *      Traffic flow index allocated by the NETCP
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Proceed - 0
 *  @retval
 *      Abort   - <0
 */
static int32_t Netfp_preReassemblyHook
(
    Netfp_ClientMCB*    ptrClientMCB,
    Ti_Pkt*             ptrFragment,
    Qmss_QueueHnd       dstQueue,
    uint16_t            tfIndex
)
{
    Netfp_HookReturn        hookRetVal;
    Netfp_ReassemblyMCB*    ptrReassemblyMCB;
    Netfp_HookFunction      preReassemblyHookFxn;
    Netfp_Hook              hook;

    /* Is this an inner/outer IP fragment? */
    if (dstQueue == ptrClientMCB->netcpTxQueue[NSS_PA_QUEUE_OUTER_IP_INDEX])
        hook = Netfp_Hook_PRE_OUTER_REASSEMBLY;
    else
        hook = Netfp_Hook_PRE_INNER_REASSEMBLY;

    /* Get the Pre-Reassembly Hook function */
    preReassemblyHookFxn = ptrClientMCB->reassemblyHook[hook];

    /* Is there a hook registered? */
    if (preReassemblyHookFxn == NULL)
        return 0;

    /* Get the reassembly MCB */
    ptrReassemblyMCB = ptrClientMCB->ptrReassemblyMCB;

    /* Pass the packet to the application registered hook */
    hookRetVal = preReassemblyHookFxn (hook, NULL, ptrFragment, ptrClientMCB->reassemblyHookArg[hook]);
    switch (hookRetVal)
    {
        case Netfp_HookReturn_DROP:
        {
            /* Packet was dropped by the hook: Clean up the traffic flow in NETCP */
            Netfp_freeTrafficFlow (ptrClientMCB, ptrFragment, tfIndex, 1, dstQueue);

            /* Cleanup the fragment */
            Pktlib_freePacket (ptrClientMCB->cfg.pktlibInstHandle, ptrFragment);

            /* Increment the statistics: */
            if (hook == Netfp_Hook_PRE_OUTER_REASSEMBLY)
                ptrReassemblyMCB->stats.numPreOuterFragmentsRejected++;
            else
                ptrReassemblyMCB->stats.numPreInnerFragmentsRejected++;

            /* Packet was dropped we cannot proceed. */
            return -1;
        }
        case Netfp_HookReturn_STOLEN:
        {
            /* Packet was dropped by the hook: Clean up the traffic flow in NETCP */
            Netfp_freeTrafficFlow (ptrClientMCB, ptrFragment, tfIndex, 1, dstQueue);

            /* Increment the statistics: */
            if (hook == Netfp_Hook_PRE_OUTER_REASSEMBLY)
                ptrReassemblyMCB->stats.numPreOuterFragmentsRejected++;
            else
                ptrReassemblyMCB->stats.numPreInnerFragmentsRejected++;

            /* Packet was dropped we cannot proceed. */
            return -1;
        }
        case Netfp_HookReturn_ACCEPT:
        {
            /* Continue; packet was accepted */
            return 0;
        }
    }

    return -1;
}

/**
 *  @b Description
 *  @n
 *      Post Reassembly Hook: This is invoked once all the fragments have been reassembled
 *      and before the packet is passed back to the NETCP for classification.
 *
 *  @param[in]  ptrClientMCB
 *      Pointer to the NETFP Client
 *  @param[in]  ptrReassemblyContext
 *      Pointer to the reassembly context
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Proceed - 0
 *  @retval
 *      Abort   - <0
 */
static int32_t Netfp_postReassemblyHook
(
    Netfp_ClientMCB*            ptrClientMCB,
    Netfp_IPReassemblyContext*  ptrReassemblyContext
)
{
    Netfp_HookReturn        hookRetVal;
    Netfp_ReassemblyMCB*    ptrReassemblyMCB;
    Netfp_HookFunction      postReassemblyHookFxn;
    Netfp_Hook              hook;
    Ti_Pkt*                 ptrPkt;

    /* Is this an inner/outer IP reassembled packet? */
    if (ptrReassemblyContext->dstQueue == ptrClientMCB->netcpTxQueue[NSS_PA_QUEUE_OUTER_IP_INDEX])
        hook = Netfp_Hook_POST_OUTER_REASSEMBLY;
    else
        hook = Netfp_Hook_POST_INNER_REASSEMBLY;

    /* Get the Post-Reassembly Hook function */
    postReassemblyHookFxn = ptrClientMCB->reassemblyHook[hook];

    /* Is there a hook registered? */
    if (postReassemblyHookFxn == NULL)
        return 0;

    /* Get the reassembly MCB */
    ptrReassemblyMCB = ptrClientMCB->ptrReassemblyMCB;

    /* Pass the packet to the application registered hook */
    hookRetVal = postReassemblyHookFxn (hook, NULL, ptrReassemblyContext->ptrPkt, ptrClientMCB->reassemblyHookArg[hook]);
    switch (hookRetVal)
    {
        case Netfp_HookReturn_DROP:
        {
            ptrPkt = ptrReassemblyContext->ptrPkt;

            /* Packet was dropped by the hook: Clean up the reassembly context in NETCP since the packet
             * reassembly was successful. */
            Netfp_freeReassemblyContext (ptrClientMCB, ptrReassemblyMCB, ptrReassemblyContext, 1);

            /* Cleanup the reassembled packet. */
            Pktlib_freePacket (ptrClientMCB->cfg.pktlibInstHandle, ptrPkt);

            /* Increment the statistics: */
            if (hook == Netfp_Hook_POST_OUTER_REASSEMBLY)
                ptrReassemblyMCB->stats.numPostOuterReassembledPktsRejected++;
            else
                ptrReassemblyMCB->stats.numPostInnerReassembledPktsRejected++;

            /* Packet was dropped we cannot proceed. */
            return -1;
        }
        case Netfp_HookReturn_STOLEN:
        {
            /* Packet was stolen by the hook: Clean up the reassembly context in NETCP since the packet
             * reassembly was successful. */
            Netfp_freeReassemblyContext (ptrClientMCB, ptrReassemblyMCB, ptrReassemblyContext, 1);

            /* Increment the statistics: */
            if (hook == Netfp_Hook_POST_OUTER_REASSEMBLY)
                ptrReassemblyMCB->stats.numPostOuterReassembledPktsRejected++;
            else
                ptrReassemblyMCB->stats.numPostInnerReassembledPktsRejected++;

            /* Packet was dropped we cannot proceed. */
            return -1;
        }
        case Netfp_HookReturn_ACCEPT:
        {
            /* Continue; packet was accepted */
            return 0;
        }
    }
    return -1;
}

/**
 *  @b Description
 *  @n
 *      The function is used to insert the received fragment into the reassembly
 *      context at an appropriate offset. The fragment is inserted in ascending
 *      order of fragmentation offset.
 *
 *      The functions returns 0 if the fragment was successfully inserted. The
 *      function will fail if the fragment is either a duplicate/overlap.
 *
 *  @param[in]  ptrClientMCB
 *      Pointer to the client MCB
 *  @param[in]  ptrReassemblyContext
 *      Pointer to the reassembly context to which the fragment belongs.
 *  @param[in]  ptrFragment
 *      Fragmented Packet which has been received
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      0   -   Fragment successfully inserted
 *  @retval
 *      <0  -   Unable to insert the fragment
 */
static int32_t Netfp_insertPktReassemblyContext
(
    Netfp_ClientMCB*              ptrClientMCB,
    Netfp_IPReassemblyContext*    ptrReassemblyContext,
    Ti_Pkt*                       ptrFragment
)
{
    Ti_Pkt*             ptrPrevListPkt;
    Ti_Pkt*             ptrListPkt;
    uint32_t            fragPayloadLength;
    uint32_t            fragOffset;
    uint32_t            listFragOffset    = (uint32_t)-1;
    uint32_t            listPayloadLength = (uint32_t)-1;

    /* Get the IP Payload length & Offset from the fragment */
    fragOffset        = Cppi_getSoftwareInfo0 (Cppi_DescType_HOST, (Cppi_Desc*)ptrFragment);
    fragPayloadLength = Cppi_getSoftwareInfo1 (Cppi_DescType_HOST, (Cppi_Desc*)ptrFragment);

    /* Start from the head of the descriptor chain */
    ptrListPkt = ptrReassemblyContext->ptrPkt;

    /* Set the previous list descriptor to NULL. */
    ptrPrevListPkt = NULL;

    /* Cycle through the chained packets to determine where we can place the received packet.
     * We are simply sorting through the list to locate where the packet can be inserted. */
    while (ptrListPkt != NULL)
    {
        /* Get the fragmentation offset & payloaded length of the list packet: */
        listFragOffset    = Cppi_getSoftwareInfo0 (Cppi_DescType_HOST, (Cppi_Desc*)ptrListPkt);
        listPayloadLength = Cppi_getSoftwareInfo1 (Cppi_DescType_HOST, (Cppi_Desc*)ptrListPkt);

        /* Does our fragment come before this? */
        if (fragOffset > listFragOffset)
        {
            /* NO. Remember this descriptor */
            ptrPrevListPkt = ptrListPkt;

            /* Goto the next descriptor. */
            ptrListPkt = Pktlib_getNextPacket(ptrListPkt);
        }
        else
        {
            /* YES. We can now insert the descriptor. */
            break;
        }
    }

    /* Is this a duplicate fragment? */
    if ((fragOffset == listFragOffset) && (fragPayloadLength == listPayloadLength))
    {
        /* YES: Duplicate fragment */
        ptrReassemblyContext->ptrReassemblyMCB->stats.numDuplicatedFragment++;
        return -1;
    }

    /* Is this an overlapping fragment? */
    if ((fragOffset == listFragOffset) && (fragPayloadLength != listPayloadLength))
    {
        /* YES: Overlapping fragment */
        ptrReassemblyContext->ptrReassemblyMCB->stats.numOverlappingFragment++;
        ptrReassemblyContext->overlapFrag = 1;
        return -1;
    }

    /* Was last fragment received? */
    if (ptrReassemblyContext->ptrLastFrag)
    {
        /* Is this last fragment */
        if (ptrReassemblyContext->ptrLastFrag == ptrFragment)
        {
            /* YES. Is there fragment with larger offset? */
            if(ptrListPkt != NULL)
            {
                /* YES. This is a problem. We have received last fragment but other fragment with greater offset exists */
                ptrReassemblyContext->ptrReassemblyMCB->stats.numCorruptedPackets++;
                ptrReassemblyContext->corruptedPkt = 1;
                return -1;
            }
        }
        else
        {
            /* We are not inserting last fragment */
            if(ptrListPkt == NULL)
            {
                /* YES. This is a problem. We have received last fragment earlier but current fragment has got greater offset */
                ptrReassemblyContext->ptrReassemblyMCB->stats.numCorruptedPackets++;
                ptrReassemblyContext->corruptedPkt = 1;
                return -1;
            }
        }
    }

    /*********************************************************************************************
     * Determine the location where the packet is to be inserted
     *********************************************************************************************/
    if ((ptrPrevListPkt == NULL) && (ptrListPkt == NULL))
    {
        /* Empty List: This is the first fragment which we received just simply insert the fragment
         * into the list. */
        ptrReassemblyContext->ptrPkt = ptrFragment;
        return 0;
    }
    else if ((ptrPrevListPkt == NULL) && (ptrListPkt != NULL))
    {
        /* Insert the packet at the head of the list. Get the fragmentation offset & payload length */
        listFragOffset = Cppi_getSoftwareInfo0 (Cppi_DescType_HOST, (Cppi_Desc*)ptrListPkt);

        /* Overlap Fragment: Does the current fragment overlap the next fragment? */
        if ((fragOffset + fragPayloadLength) > listFragOffset)
        {
            /* YES: Overlap detected */
            ptrReassemblyContext->ptrReassemblyMCB->stats.numOverlappingFragment++;
            ptrReassemblyContext->overlapFrag = 1;
            return -1;
        }
        else
        {
            /* NO: Overlap detected */
            ptrReassemblyContext->ptrPkt = ptrFragment;

            /* Ensure that the current descriptor points to the next. */
            Cppi_linkNextBD (Cppi_DescType_HOST, (Cppi_Desc*)ptrFragment, (Cppi_Desc*)ptrListPkt);
        }
    }
    else if ((ptrPrevListPkt != NULL) && (ptrListPkt == NULL))
    {
        /* Inserting the packet at the end of the list. Get the fragmentation offset & payload length */
        listFragOffset    = Cppi_getSoftwareInfo0 (Cppi_DescType_HOST, (Cppi_Desc*)ptrPrevListPkt);
        listPayloadLength = Cppi_getSoftwareInfo1 (Cppi_DescType_HOST, (Cppi_Desc*)ptrPrevListPkt);

        /* Overlap Fragment: Does the previous fragment overlap the new fragment? */
        if ((listFragOffset + listPayloadLength) > fragOffset)
        {
            /* YES: Overlap detected */
            ptrReassemblyContext->ptrReassemblyMCB->stats.numOverlappingFragment++;
            ptrReassemblyContext->overlapFrag = 1;
            return -1;
        }
        else
        {
            /* NO: Overlap detected. The previous BD points to the current BD. */
            Cppi_linkNextBD (Cppi_DescType_HOST, (Cppi_Desc*)ptrPrevListPkt, (Cppi_Desc*)ptrFragment);

            /* The current BD points to the next BD */
            Cppi_linkNextBD (Cppi_DescType_HOST, (Cppi_Desc*)ptrFragment, (Cppi_Desc*)ptrListPkt);
        }
    }
    else if ((ptrPrevListPkt != NULL) && (ptrListPkt != NULL))
    {
        /* Inserting the packet in the middle of the list. Get the fragmentation offset & payload length */
        listFragOffset    = Cppi_getSoftwareInfo0 (Cppi_DescType_HOST, (Cppi_Desc*)ptrPrevListPkt);
        listPayloadLength = Cppi_getSoftwareInfo1 (Cppi_DescType_HOST, (Cppi_Desc*)ptrPrevListPkt);

        /* Overlap Fragment: Does the previous fragment overlap the new fragment? */
        if ((listFragOffset + listPayloadLength) > fragOffset)
        {
            /* YES: Overlap detected */
            ptrReassemblyContext->ptrReassemblyMCB->stats.numOverlappingFragment++;
            ptrReassemblyContext->overlapFrag = 1;
            return -1;
        }
        else
        {
            /* Overlap Fragment: Does the current fragment overlap the next fragment? */
            listFragOffset    = Cppi_getSoftwareInfo0 (Cppi_DescType_HOST, (Cppi_Desc*)ptrListPkt);
            if ((fragOffset + fragPayloadLength) > listFragOffset)
            {
                /* YES: Overlap detected */
                ptrReassemblyContext->ptrReassemblyMCB->stats.numOverlappingFragment++;
                ptrReassemblyContext->overlapFrag = 1;
                return -1;
            }
            else
            {
                /* NO: Overlap detected. The previous BD points to the current BD. */
                Cppi_linkNextBD (Cppi_DescType_HOST, (Cppi_Desc*)ptrPrevListPkt, (Cppi_Desc*)ptrFragment);

                /* The current BD points to the next BD */
                Cppi_linkNextBD (Cppi_DescType_HOST, (Cppi_Desc*)ptrFragment, (Cppi_Desc*)ptrListPkt);
            }
        }
    }

    /* Control comes here implies that the packet has been successfully inserted into the list */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the ESP Trailer Length.
 *
 *  @param[in]  ptrDataBuffer
 *      Pointer to the start of the fragment data buffer
 *  @param[in]  ptrFragmentInfo
 *      Pointer to the fragmentation info block
  *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      ESP Trailer length
 */
static int32_t Netfp_getESPTrailerLength (uint8_t* ptrDataBuffer, Netfp_IPFragmentInfo* ptrFragmentInfo)
{
    uint32_t    espHeaderLen;
    uint32_t    espTrailerLen;
    uint32_t    outerIPTotalLen;

    /* Determine if the packet is an Inner or Outer fragment? For outer fragments; the ESP trailer length is
     * opaque and does not need to be accounted for. */
    if (PASAHO_LINFO_IS_IPSEC(ptrFragmentInfo->pasaInfo) == 0)
        return 0;

    /***********************************************************************************************
     * Explanation:
     * ------------
     * PASAHO_LINFO_READ_START_OFFSET  -> Offset to the start of the Inner IP Header.
     * PASAHO_LINFO_READ_L3_OFFSET     -> Offset to the start of the Outer IP Header.
     * PASAHO_LINFO_READ_ESP_AH_OFFSET -> Offset to the start of the ESP Header.
     ***********************************************************************************************/

    /* Is the Outer IP Header IPv4 or IPv6? */
    if (Netfp_isIPv4Packet ((Netfp_IPHeader*)(ptrDataBuffer + PASAHO_LINFO_READ_L3_OFFSET(ptrFragmentInfo->pasaInfo))) == 1)
    {
        /* Outer IPv4 Header: Get the outer IPv4 Header. */
        Netfp_IPHeader*     ptrOuterIPv4Header;
        ptrOuterIPv4Header = (Netfp_IPHeader*)(ptrDataBuffer + PASAHO_LINFO_READ_L3_OFFSET(ptrFragmentInfo->pasaInfo));

        /* Is the Inner IP header IPv4 or IPv6? */
        if (Netfp_isIPv4Packet ((Netfp_IPHeader*)ptrFragmentInfo->ptrIPHeader) == 1)
        {
            /* Inner IPv4 Header: Get the inner IPv4 Header */
            Netfp_IPHeader*     ptrInnerIPv4Header;
            ptrInnerIPv4Header = (Netfp_IPHeader*)ptrFragmentInfo->ptrIPHeader;

            /* Outer IPv4 Header: Get the total length of the outer IPv4 packet. */
            outerIPTotalLen = Netfp_ntohs (ptrOuterIPv4Header->TotalLen);

            /* Get the length of the ESP Header: This is calculated as the difference between the starting offset
             * of the START_OFFSET (Inner IPv4 Header) and the ESP Header */
            espHeaderLen = PASAHO_LINFO_READ_START_OFFSET(ptrFragmentInfo->pasaInfo) -
                           PASAHO_LINFO_READ_ESP_AH_OFFSET(ptrFragmentInfo->pasaInfo);

            /********************************************************************************
             * Packet format which is being handled here is as follows:
             *
             *  |---------------------------------|        ^
             *  | Outer IPv4 Header               |        |
             *  |---------------------------------|        |
             *  | ESP Header                      |        |
             *  |---------------------------------|        |
             *  | Inner IPv4 Header               |        |
             *  |---------------------------------|        |  Outer IP Total Length
             *  | Data Payload                    |        |
             *  |                                 |        |
             *  |---------------------------------|        |
             *  | ESP Trailer                     |        |
             *  |---------------------------------|        v
             *
             * NOTE: IPv4 Headers total length include the size of the header
             ********************************************************************************/
            espTrailerLen = outerIPTotalLen - (((ptrOuterIPv4Header->VerLen & IPV4_HLEN_MASK) << 2) +
                                                espHeaderLen + Netfp_ntohs (ptrInnerIPv4Header->TotalLen));

            /* If UDP Encapsulation header is present, adjust the trailer length by 8 */
            if(PASAHO_LINFO_IS_IPSEC_NAT_T(ptrFragmentInfo->pasaInfo))
                espTrailerLen = espTrailerLen - UDPHDR_SIZE;
        }
        else
        {
            /* Inner IPv6 Header: Get the inner IPv6 header */
            Netfp_IPv6Header*   ptrInnerIPv6Header;
            ptrInnerIPv6Header = (Netfp_IPv6Header*)ptrFragmentInfo->ptrIPHeader;

            /* Outer IPv4 Header: Get the total length of the outer IPv4 packet. */
            outerIPTotalLen = Netfp_ntohs (ptrOuterIPv4Header->TotalLen);

            /* Get the length of the ESP Header: This is calculated as the difference between the starting offset
             * of the START_OFFSET (Inner IPv4 Header) and the ESP Header */
            espHeaderLen = PASAHO_LINFO_READ_START_OFFSET(ptrFragmentInfo->pasaInfo) -
                           PASAHO_LINFO_READ_ESP_AH_OFFSET(ptrFragmentInfo->pasaInfo);

            /********************************************************************************
             * Packet format which is being handled here is as follows:
             *
             *  |---------------------------------|        ^
             *  | Outer IPv4 Header               |        |
             *  |---------------------------------|        |
             *  | ESP Header                      |        |
             *  |---------------------------------|        |
             *  | Inner IPv6 Header               |        |
             *  |---------------------------------|        |  Outer IP Total Length
             *  | Data Payload                    |        |
             *  |                                 |        |
             *  |---------------------------------|        |
             *  | ESP Trailer                     |        |
             *  |---------------------------------|        v
             *
             * NOTE: IPv4 Headers total length include the size of the header
             ********************************************************************************/
            espTrailerLen = outerIPTotalLen - (((ptrOuterIPv4Header->VerLen & IPV4_HLEN_MASK) << 2) +
                                                espHeaderLen + IPv6HDR_SIZE + Netfp_ntohs (ptrInnerIPv6Header->PayloadLength));

            /* If UDP Encapsulation header is present, adjust the trailer length by 8 */
            if(PASAHO_LINFO_IS_IPSEC_NAT_T(ptrFragmentInfo->pasaInfo))
                espTrailerLen = espTrailerLen - UDPHDR_SIZE;
        }
    }
    else
    {
        /* Outer IPv6 Header: Get the outer IPv6 header */
        Netfp_IPv6Header*   ptrOuterIPv6Header;
        ptrOuterIPv6Header = (Netfp_IPv6Header*)(ptrDataBuffer + PASAHO_LINFO_READ_L3_OFFSET(ptrFragmentInfo->pasaInfo));

        /* Is the Inner IP header IPv4 or IPv6? */
        if (Netfp_isIPv4Packet ((Netfp_IPHeader*)ptrFragmentInfo->ptrIPHeader) == 1)
        {
            /* Inner IPv4 Header: Get the inner IPv4 Header */
            Netfp_IPHeader*     ptrInnerIPv4Header;
            ptrInnerIPv4Header = (Netfp_IPHeader*)ptrFragmentInfo->ptrIPHeader;

            /* Outer IPv6 Header: Get the total length of the outer IPv6 packet. */
            outerIPTotalLen = Netfp_ntohs (ptrOuterIPv6Header->PayloadLength);

            /* Get the length of the ESP Header: This is calculated as the difference between the starting offset
             * of the START_OFFSET (Inner IPv4 Header) and the ESP Header */
            espHeaderLen = PASAHO_LINFO_READ_START_OFFSET(ptrFragmentInfo->pasaInfo) -
                           PASAHO_LINFO_READ_ESP_AH_OFFSET(ptrFragmentInfo->pasaInfo);

            /********************************************************************************
             * Packet format which is being handled here is as follows:
             *
             *  |---------------------------------|
             *  | Outer IPv6 Header               |
             *  |---------------------------------|        ^
             *  | ESP Header                      |        |
             *  |---------------------------------|        |
             *  | Inner IPv4 Header               |        |
             *  |---------------------------------|        |  Outer IP Total Length
             *  | Data Payload                    |        |
             *  |                                 |        |
             *  |---------------------------------|        |
             *  | ESP Trailer                     |        |
             *  |---------------------------------|        v
             *
             * NOTE: IPv4 Headers total length include the size of the header
             ********************************************************************************/
            espTrailerLen = outerIPTotalLen - (espHeaderLen + Netfp_ntohs (ptrInnerIPv4Header->TotalLen));

            /* If UDP Encapsulation header is present, adjust the trailer length by 8 */
            if(PASAHO_LINFO_IS_IPSEC_NAT_T(ptrFragmentInfo->pasaInfo))
                espTrailerLen = espTrailerLen - UDPHDR_SIZE;
        }
        else
        {
            /* Inner IPv6 Header: Get the inner IPv6 Header. */
            Netfp_IPv6Header* ptrInnerIPv6Header;
            ptrInnerIPv6Header = (Netfp_IPv6Header*)ptrFragmentInfo->ptrIPHeader;

            /* Outer IPv6 Header: Get the total length of the outer IPv6 packet. */
            outerIPTotalLen = Netfp_ntohs (ptrOuterIPv6Header->PayloadLength);

            /* Get the length of the ESP Header: This is calculated as the difference between the starting offset
             * of the START_OFFSET (Inner IPv4 Header) and the ESP Header */
            espHeaderLen = PASAHO_LINFO_READ_START_OFFSET(ptrFragmentInfo->pasaInfo) -
                           PASAHO_LINFO_READ_ESP_AH_OFFSET(ptrFragmentInfo->pasaInfo);

            /********************************************************************************
             * Packet format which is being handled here is as follows:
             *
             *  |---------------------------------|
             *  | Outer IPv6 Header               |
             *  |---------------------------------|        ^
             *  | ESP Header                      |        |
             *  |---------------------------------|        |
             *  | Inner IPv6 Header               |        |
             *  |---------------------------------|        |  Outer IP Total Length
             *  | Data Payload                    |        |
             *  |                                 |        |
             *  |---------------------------------|        |
             *  | ESP Trailer                     |        |
             *  |---------------------------------|        v
             *
             * NOTE: IPv6 Headers Payload length does NOT include the size of IPv6 header.
             ********************************************************************************/
            espTrailerLen = outerIPTotalLen - (espHeaderLen + IPv6HDR_SIZE + Netfp_ntohs (ptrInnerIPv6Header->PayloadLength));

            /* If UDP Encapsulation header is present, adjust the trailer length by 8 */
            if(PASAHO_LINFO_IS_IPSEC_NAT_T(ptrFragmentInfo->pasaInfo))
                espTrailerLen = espTrailerLen - UDPHDR_SIZE;
        }
    }
    return espTrailerLen;
}

/**
 *  @b Description
 *  @n
 *      The function is used to perform the actual IPv6 reassembly operation when an
 *      IPv6 fragment is received.
 *
 *  @param[in]  ptrClientMCB
 *      Pointer to the client MCB
 *  @param[in]  ptrReassemblyContext
 *      Pointer to the client reassembly context
 *  @param[in]  ptrFragment
 *      Fragmented packet which has been received and which needs to be reassembled
 *  @param[in]  ptrFragmentInfo
 *      Fragmentation information which has meta-information regarding the received
 *      fragment.
 *  @param[out]  ptrReassembledPkt
 *      Set to NULL indicates packet is not reassembled else set to the reassembled
 *      packet.
 *  @param[out]  errCode
 *      Error code populated on error
  *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      0       - Success
 *  @retval
 *      <0      - Error
 */
static int32_t Netfp_ipv6Reassembly
(
    Netfp_ClientMCB*            ptrClientMCB,
    Netfp_IPReassemblyContext*  ptrReassemblyContext,
    Ti_Pkt*                     ptrFragment,
    Netfp_IPFragmentInfo*       ptrFragmentInfo,
    Ti_Pkt**                    ptrReassembledPkt,
    int32_t*                    errCode
)
{
    Netfp_IPv6Header*       ptrIPv6Header;
    uint32_t                espTrailerLen;
    uint32_t                fragmentType;
    uint32_t                ipHdrPayloadLen;
    uint32_t                reassembledLen;
    uint8_t*                ptrFragmentData;
    uint8_t*                ptrDataBuffer;
    uint32_t                dataBufferLen;
    uint16_t                fragOffset;
    uint8_t*                ptrIPv6Payload;
    int32_t                 fragIndex;
    int32_t                 index;

    /* Packet reassembly is not complete. */
    *ptrReassembledPkt = NULL;

    /* Get the data buffer from the fragmented packet. */
    Pktlib_getDataBuffer(ptrFragment, &ptrDataBuffer, &dataBufferLen);

    /* Reassemble the fragment: Get the IPv6 header */
    ptrIPv6Header = (Netfp_IPv6Header*)ptrFragmentInfo->ptrIPHeader;

    /* Get the pointer to the IPv6 Payload [Skipping the IPv6 + Extension headers] */
    ptrIPv6Payload = (uint8_t*)ptrIPv6Header + IPv6HDR_SIZE + ptrFragmentInfo->sizeIPv6ExtensionHeader;

    /* Extract the IP length including the IP header (discounting the fragmented header)
     * The Payload length in IPv6 does not include the size of the IPv6 header. */
    ipHdrPayloadLen  = Netfp_ntohs (ptrIPv6Header->PayloadLength);
    ipHdrPayloadLen  = ipHdrPayloadLen - ptrFragmentInfo->sizeIPv6ExtensionHeader;

    /* Determine if the packet is FIRST, MIDDLE or LAST Fragment. */
    if ((ptrFragmentInfo->fragOffset & IPV6_FLAGS_MF_MASK) == 0)
        fragmentType = FRAG_LAST;
    else if ((ptrFragmentInfo->fragOffset & IPV6_FRAGO_MASK) == 0)
        fragmentType = FRAG_FIRST;
    else
        fragmentType = FRAG_MIDDLE;

    /* Get the fragmentation offset now. We can remove the flags. */
    fragOffset = (ptrFragmentInfo->fragOffset  & IPV6_FRAGO_MASK);

    /* Reassembly depends upon which fragment has been received: */
    switch (fragmentType)
    {
        case FRAG_FIRST:
        {
            /* First Fragment: Get the ESP Trailer Length. */
            espTrailerLen = Netfp_getESPTrailerLength (ptrDataBuffer, ptrFragmentInfo);

            /* The first fragment needs to include all the networking header (including L2) - the IPv6
             * fragment header. The length of the data buffer is what we received in the descriptor */
            ptrFragmentData = (uint8_t*)ptrDataBuffer + IPV6_FRAGHDR_SIZE;
            dataBufferLen   = dataBufferLen - IPV6_FRAGHDR_SIZE;

#if (defined (DEVICE_K2H) || defined (DEVICE_K2K))
            /* Discount the trailers from the total packet: Ethernet FCS and the ESP Trailer length */
            dataBufferLen  = dataBufferLen - (espTrailerLen+4);
#else
            /* Discount the trailers from the total packet: ESP Trailer length */
            dataBufferLen  = dataBufferLen - espTrailerLen;
#endif
            /* Cycle through and get the extension header and find the fragmentation header: */
            for (fragIndex = 0; fragIndex < ptrFragmentInfo->numIPv6ExtHeader; fragIndex++)
            {
                /* Do we have the fragmentation header? */
                if (ptrFragmentInfo->ipv6ExtensionHeader[fragIndex].type == IPPROTO6_FRAG)
                    break;
            }

            /*****************************************************************************************
             * Setup the Reassembly length to account for the Fragmentable/Non-Fragmentable:
             * Do we have any extension headers following the Fragmentation Header?
             *****************************************************************************************/
            if ((ptrFragmentInfo->numIPv6ExtHeader == (fragIndex + 1)))
            {
                /* NO: This implies that the fragment header was either the only extension header or the
                 * last extension header. We dont need to discount the length of extension headers here.
                 * The IP Header payload length is correct. Simply fall through. */
            }
            else
            {
                /* YES: This implies that there are certain other headers which make the fragmentable part. These
                 * need to be accounted for in the reassembled length. */
                for (index = (fragIndex + 1); index < ptrFragmentInfo->numIPv6ExtHeader; index++)
                    ipHdrPayloadLen = ipHdrPayloadLen + ptrFragmentInfo->ipv6ExtensionHeader[index].length;
            }

            /* From the IP reassembly perspective; we have reassembled the entire first fragment. */
            reassembledLen = ipHdrPayloadLen;

            /* Is the fragmentation header the first header? */
            if (fragIndex == 0)
            {
                /* YES: Modify the IPv6 Next header to the next header */
                ptrIPv6Header->NextHeader = ptrFragmentInfo->ptrIPv6FragHeader->NextHeader;
            }
            else
            {
                /* NO: The IPv6 Fragmentation header is somewhere in the middle/last. Modify the next header
                 * of the previous extension header */
                *(ptrFragmentInfo->ipv6ExtensionHeader[fragIndex - 1].ptrHeader) = ptrFragmentInfo->ptrIPv6FragHeader->NextHeader;

                /* Cycle through all the previous extension headers and shift each of them up */
                while (fragIndex != 0)
                {
                    memmove ((void*)(ptrFragmentInfo->ipv6ExtensionHeader[fragIndex].ptrHeader),
                             (void*)(ptrFragmentInfo->ipv6ExtensionHeader[fragIndex - 1].ptrHeader),
                             ptrFragmentInfo->ipv6ExtensionHeader[fragIndex - 1].length);
                    fragIndex = fragIndex - 1;
                }
            }

            /* Shift the L2 and IPv6 headers */
            memmove ((void*)ptrFragmentData, (void*)ptrDataBuffer, ptrFragmentInfo->ipOffset + IPv6HDR_SIZE);

            /* Modify the data buffers in the descriptors to remove the Ethernet CRC/FCS. */
            Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc*)ptrFragment, ptrFragmentData, dataBufferLen);
            break;
        }
        case FRAG_MIDDLE:
        {
            /* Middle Fragments need to skip the IPv6 header & fragment header. We are interested
             * in only the IPv6 Payload; which follows the IPv6 fragmentation header. */
            ptrFragmentData = (uint8_t*)ptrIPv6Payload;
            reassembledLen  = ipHdrPayloadLen;

            /* Modify the data buffers in the descriptors. */
            Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc*)ptrFragment, ptrFragmentData, ipHdrPayloadLen);
            break;
        }
        case FRAG_LAST:
        {
            if (ptrReassemblyContext->ptrLastFrag == NULL )
            {
                ptrReassemblyContext->ptrLastFrag = ptrFragment;
                /* The LAST Fragment is used to determine the length of the original packet.
                 * The total length of the expected data is the fragmentation offset + length
                 * of the payload carried in the last packet. */
                ptrReassemblyContext->origIPLength = ipHdrPayloadLen + fragOffset;
            }
            else
            {
                /* Drop the last fragment packet as not expected */
                Netfp_freeFragment (ptrClientMCB, ptrReassemblyContext->tfId, ptrReassemblyContext->dstQueue, ptrFragment);

                ptrReassemblyContext->ptrReassemblyMCB->stats.numCorruptedPackets++;
                ptrReassemblyContext->corruptedPkt = 1;

                /* Discount off the fragment since it has been dropped */
                ptrReassemblyContext->fragCnt--;
                ptrClientMCB->ptrReassemblyMCB->stats.numActiveFragments--;
                return 0;
            }

            /* Last Fragment: Need to skip the IP headers but we need to add the ESP trailer if
             * the packet was an inner IP fragment. This is because the reassembled packets could
             * be meant for Linux */
            ptrFragmentData = (uint8_t*)ptrIPv6Payload;
            reassembledLen  = ipHdrPayloadLen;

            /* Get the ESP Trailer Length. */
            espTrailerLen = Netfp_getESPTrailerLength (ptrDataBuffer, ptrFragmentInfo);

            /* Modify the data buffers in the descriptors. */
            Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc*)ptrFragment, ptrFragmentData, (reassembledLen + espTrailerLen));
            break;
        }
    }

    /* Reset the packet length of each fragment. Once the packet has been reassembled completely
     * the packet lengths will be setup correctly only for the head of the reassembled packet. */
    Pktlib_setPacketLen(ptrFragment, 0);

    /* Set the information fields into the descriptor:
     * - Fragmentation Offset
     * - IP Payload Length */
    Cppi_setSoftwareInfo0 (Cppi_DescType_HOST, (Cppi_Desc*)ptrFragment, fragOffset);
    Cppi_setSoftwareInfo1 (Cppi_DescType_HOST, (Cppi_Desc*)ptrFragment, ipHdrPayloadLen);

    /* Cycle through and determine where the allocated reassembly context can be inserted.
     * The sorting is done in ascending order of the fragmentation offset */
    if (Netfp_insertPktReassemblyContext(ptrClientMCB, ptrReassemblyContext, ptrFragment) < 0)
    {
        /* Drop the duplicate/overlapped fragment packet: */
        Netfp_freeFragment (ptrClientMCB, ptrReassemblyContext->tfId, ptrReassemblyContext->dstQueue, ptrFragment);

        /* Discount off the fragment since it has been dropped */
        ptrReassemblyContext->fragCnt--;
        ptrClientMCB->ptrReassemblyMCB->stats.numActiveFragments--;
        return 0;
    }

    /* We have detected another fragment. Account for the length of this fragment */
    ptrReassemblyContext->detectedIPLen  = ptrReassemblyContext->detectedIPLen + ipHdrPayloadLen;
    ptrReassemblyContext->totalPacketLen = ptrReassemblyContext->totalPacketLen + Pktlib_getDataBufferLen(ptrFragment);

    /* Check if the reassembly is complete? */
    if (ptrReassemblyContext->origIPLength == ptrReassemblyContext->detectedIPLen)
        *ptrReassembledPkt = ptrReassemblyContext->ptrPkt;
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to perform the actual IPv4 reassembly operation when an
 *      IPv4 fragment is received.
 *
 *  @param[in]  ptrClientMCB
 *      Pointer to the client MCB
 *  @param[in]  ptrReassemblyContext
 *      Pointer to the client reassembly context
 *  @param[in]  ptrFragment
 *      Fragmented packet which has been received and which needs to be reassembled
 *  @param[in]  ptrFragmentInfo
 *      Fragmentation information which has meta-information regarding the received
 *      fragment.
 *  @param[out]  ptrReassembledPkt
 *      Set to NULL indicates packet is not reassembled else set to the reassembled
 *      packet.
 *  @param[out]  errCode
 *      Error code populated on error
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      0       - Success
 *  @retval
 *      <0      - Error
 */
static int32_t Netfp_ipv4Reassembly
(
    Netfp_ClientMCB*            ptrClientMCB,
    Netfp_IPReassemblyContext*  ptrReassemblyContext,
    Ti_Pkt*                     ptrFragment,
    Netfp_IPFragmentInfo*       ptrFragmentInfo,
    Ti_Pkt**                    ptrReassembledPkt,
    int32_t*                    errCode
)
{
    Netfp_IPHeader*     ptrIPv4Header;
    uint32_t            fragmentType;
    uint32_t            ipHdrTotalLen;
    uint32_t            reassembledLen;
    uint32_t            ipHdrLen;
    uint8_t*            ptrFragmentData;
    uint8_t*            ptrDataBuffer;
    uint32_t            dataBufferLen;
    uint16_t            fragOffset;
    uint32_t            espTrailerLen;

    /* Packet reassembly is not complete. */
    *ptrReassembledPkt = NULL;

    /* Packet has not been reassembled */
    reassembledLen = 0;

    /* Get the data buffer from the fragmented packet. */
    Pktlib_getDataBuffer(ptrFragment, &ptrDataBuffer, &dataBufferLen);

    /* Reassemble the fragment: Get the IPv4 header */
    ptrIPv4Header = (Netfp_IPHeader*)ptrFragmentInfo->ptrIPHeader;

    /* Extract the IP length including the IP header */
    ipHdrTotalLen  = Netfp_ntohs (ptrIPv4Header->TotalLen);
    ipHdrLen       = (ptrIPv4Header->VerLen & IPV4_HLEN_MASK) << 2;

    /* Determine if the packet is FIRST, MIDDLE or LAST Fragment. */
    if ((ptrFragmentInfo->fragOffset & IPV4_FLAGS_MF_MASK) == 0)
        fragmentType = FRAG_LAST;
    else if ((ptrFragmentInfo->fragOffset & IPV4_FRAGO_MASK) == 0)
        fragmentType = FRAG_FIRST;
    else
        fragmentType = FRAG_MIDDLE;

    /* Get the fragmentation offset now. We can remove the flags. */
    fragOffset = (ptrFragmentInfo->fragOffset  & IPV4_FRAGO_MASK) << 3;

    /* Reassembly depends upon which fragment has been received: */
    switch (fragmentType)
    {
        case FRAG_FIRST:
        {
            /* First fragment: We need to include all the L2 headers but remove any trailers
             * if present. The first fragment needs to include all the headers (including L2) */
            ptrFragmentData = ptrDataBuffer;

            /* Get the ESP Trailer Length. */
            espTrailerLen = Netfp_getESPTrailerLength (ptrDataBuffer, ptrFragmentInfo);

#if (defined (DEVICE_K2H) || defined (DEVICE_K2K))
            /* Discount the trailers from the total packet: Ethernet FCS and the ESP Trailer length */
            dataBufferLen  = dataBufferLen - (espTrailerLen+4);
#else
            /* Discount the trailers from the total packet: ESP Trailer length */
            dataBufferLen  = dataBufferLen - espTrailerLen;
#endif
            /* From the IP reassembly perspective; we have reassembled the entire first fragment. */
            reassembledLen = ipHdrTotalLen;

            /* Modify the data buffers in the descriptors to remove the Ethernet CRC/FCS. */
            Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc*)ptrFragment, ptrFragmentData, dataBufferLen);
            break;
        }
        case FRAG_MIDDLE:
        {
            /* Middle fragments need to skip the IPv4 header */
            ptrFragmentData = ((uint8_t*)ptrIPv4Header) + ipHdrLen;
            reassembledLen  = ipHdrTotalLen - ipHdrLen;

            /* Modify the data buffers in the descriptors. */
            Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc*)ptrFragment, ptrFragmentData, reassembledLen);
            break;
        }
        case FRAG_LAST:
        {
            if (ptrReassemblyContext->ptrLastFrag == NULL )
            {
                ptrReassemblyContext->ptrLastFrag = ptrFragment;
                /* The LAST Fragment is used to determine the length of the original packet.
                 * The total length of the expected data is the fragmentation offset + length
                 * of the payload carried in the last packet. */
                ptrReassemblyContext->origIPLength = ipHdrTotalLen + fragOffset;
            }
            else
            {
                /* Drop the last fragment packet as not expected */
                Netfp_freeFragment (ptrClientMCB, ptrReassemblyContext->tfId, ptrReassemblyContext->dstQueue, ptrFragment);

                ptrReassemblyContext->ptrReassemblyMCB->stats.numCorruptedPackets++;
                ptrReassemblyContext->corruptedPkt = 1;

                /* Discount off the fragment since it has been dropped */
                ptrReassemblyContext->fragCnt--;
                ptrClientMCB->ptrReassemblyMCB->stats.numActiveFragments--;
                return 0;
            }

            /* Last Fragment: Need to skip the IP header but we need to add the ESP trailer if
             * the packet was an inner IP fragment. This is because the reassembled packets could
             * be meant for Linux */
            ptrFragmentData = ((uint8_t*)ptrIPv4Header) + ipHdrLen;
            reassembledLen  = ipHdrTotalLen - ipHdrLen;

            /* Get the ESP Trailer Length. */
            espTrailerLen = Netfp_getESPTrailerLength (ptrDataBuffer, ptrFragmentInfo);

            /* Modify the data buffers in the descriptors. */
            Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc*)ptrFragment, ptrFragmentData, (reassembledLen + espTrailerLen));
            break;
        }
    }

    /* Reset the packet length of each fragment. Once the packet has been reassembled completely
     * the packet lengths will be setup correctly only for the head of the reassembled packet. */
    Pktlib_setPacketLen(ptrFragment, 0);

    /* Set the information fields into the descriptor:
     * - Fragmentation Offset
     * - IP Payload Length */
    Cppi_setSoftwareInfo0 (Cppi_DescType_HOST, (Cppi_Desc*)ptrFragment, fragOffset);
    Cppi_setSoftwareInfo1 (Cppi_DescType_HOST, (Cppi_Desc*)ptrFragment, (ipHdrTotalLen - ipHdrLen));

    /* Cycle through and determine where the allocated reassembly context can be inserted.
     * The sorting is done in ascending order of the fragmentation offset */
    if (Netfp_insertPktReassemblyContext(ptrClientMCB, ptrReassemblyContext, ptrFragment) < 0)
    {
        /* Drop the duplicate/overlapped fragment packet: */
        Netfp_freeFragment (ptrClientMCB, ptrReassemblyContext->tfId, ptrReassemblyContext->dstQueue, ptrFragment);

        /* Discount off the fragment since it has been dropped */
        ptrReassemblyContext->fragCnt--;
        ptrClientMCB->ptrReassemblyMCB->stats.numActiveFragments--;
        return 0;
    }

    /* We have detected another fragment. Account for the length of this fragment only */
    ptrReassemblyContext->detectedIPLen  = ptrReassemblyContext->detectedIPLen  + reassembledLen;
    ptrReassemblyContext->totalPacketLen = ptrReassemblyContext->totalPacketLen + Pktlib_getDataBufferLen(ptrFragment);

    /* Check if the reassembly is complete? */
    if (ptrReassemblyContext->origIPLength == ptrReassemblyContext->detectedIPLen)
        *ptrReassembledPkt = ptrReassemblyContext->ptrPkt;
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is called once the packet has been completed reassembled.
 *      The function determines where the reassembled packet needs to be pushed
 *      and reports the completion status to the NETCP.
 *
 *  @param[in]  ptrClientMCB
 *      Pointer to the NETFP client
 *  @param[in]  ptrReassemblyContext
 *      Pointer to the reassembly context which is complete
 *  @param[in]  ptrReassembledPacket
 *      Head of the reassembled packet
 *  @param[in]  ptrFragmentInfo
 *      Fragment information associated with the packet
 *  @param[out]  errCode
 *      Error code populated on error
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      0       - Success
 *  @retval
 *      <0      - Error
 */
static int32_t Netfp_completeReassembly
(
    Netfp_ClientMCB*            ptrClientMCB,
    Netfp_IPReassemblyContext*  ptrReassemblyContext,
    Ti_Pkt*                     ptrReassembledPacket,
    Netfp_IPFragmentInfo*       ptrFragmentInfo,
    int32_t*                    errCode
)
{
    pasahoLongInfo_t*       pInfo;
    uint32_t                infoLen;
    uint8_t*                ptrDataBuffer;
    uint32_t                dataBufferLen;
    Netfp_ReassemblyMCB*    ptrReassemblyMCB;

    /* Get the reassembly MCB */
    ptrReassemblyMCB = ptrClientMCB->ptrReassemblyMCB;

    /* Increment the statistics */
    ptrReassemblyMCB->stats.numReassembledPackets++;

    /* Get the PS Information associated with the reassembled packet. */
    Cppi_getPSData (Cppi_DescType_HOST, Cppi_PSLoc_PS_IN_DESC,
                    (Cppi_Desc *)ptrReassembledPacket, (uint8_t **)&pInfo, &infoLen);
    PASAHO_LINFO_SET_FRANCNT(pInfo, ptrReassemblyContext->fragCnt);

    /* Get the data buffer from the reassembled packet. */
    Pktlib_getDataBuffer(ptrReassembledPacket, &ptrDataBuffer, &dataBufferLen);

    /* Update the IP header. */
    if (ptrFragmentInfo->ipVer == Netfp_IPVersion_IPV4)
    {
        /* Get the IP Header */
         Netfp_IPHeader* const ptrIPHeader = (Netfp_IPHeader*)((uint8_t *)ptrDataBuffer + ptrFragmentInfo->ipOffset);

        /* Modify the fields in the IP header to indicate that the packet has been reassembled
         * and is no longer a fragmented */
        ptrIPHeader->TotalLen  = Netfp_ntohs (ptrReassemblyContext->origIPLength);
        ptrIPHeader->FlagOff   = 0;

        /* Recompute the IPv4 checksum. */
        Netfp_IPChecksum (ptrIPHeader);
    }
    else
    {
        /* Get the IPv6 header: Modify the fields to indicate that the packet has been reassembled. */
           Netfp_IPv6Header* const ptrIPv6Header = (Netfp_IPv6Header*)((uint8_t *)ptrDataBuffer + ptrFragmentInfo->ipOffset);

        /* We have already stripped off the IPv6 Fragmentation header but we need to account for any additional extension
         * headers */
        ptrFragmentInfo->sizeIPv6ExtensionHeader = ptrFragmentInfo->sizeIPv6ExtensionHeader - IPV6_FRAGHDR_SIZE;
        ptrIPv6Header->PayloadLength = Netfp_ntohs (ptrReassemblyContext->origIPLength + ptrFragmentInfo->sizeIPv6ExtensionHeader);
    }

    /* Do we need to update the outer IP Header? This is required only for the inner IP fragments. */
    if (PASAHO_LINFO_IS_IPSEC(ptrFragmentInfo->pasaInfo) == 1)
    {
        /* Get the L2 Header Size */
        uint32_t const l2HeaderSize = PASAHO_LINFO_READ_L3_OFFSET(ptrFragmentInfo->pasaInfo);

        /* Outer IP Header IPv4 or IPv6? */
        if (Netfp_isIPv4Packet ((Netfp_IPHeader*)(ptrDataBuffer + l2HeaderSize)) == 1)
        {
            /* IPv4 Header: Determine the total length of the outer IP Header. This is the total running
             * length of the packet excluding the L2 header. */
            Netfp_IPHeader* const ptrOuterIPv4Header = (Netfp_IPHeader*)(ptrDataBuffer + l2HeaderSize);
            uint16_t const totalLength = ptrReassemblyContext->totalPacketLen - l2HeaderSize;
            ptrOuterIPv4Header->TotalLen  = Netfp_ntohs (totalLength);

            /* Recompute the IPv4 checksum. */
            Netfp_IPChecksum (ptrOuterIPv4Header);

            // Is ESP packges?
            if (PASAHO_LINFO_IS_IPSEC(ptrFragmentInfo->pasaInfo) != 0 &&
                ptrOuterIPv4Header->Protocol == 0x11) { // ESP package (UDP Encapsulation)
                    uint16_t const hdrLen = ((ptrOuterIPv4Header->VerLen & IPV4_HLEN_MASK) * 4);
                    Netfp_UDPHeader * const udpHdrPtr = (Netfp_UDPHeader*)((uint8_t *)ptrOuterIPv4Header + hdrLen);
                    udpHdrPtr->Length = Netfp_ntohs(totalLength - hdrLen);
                    //fzm UDPChecksum for IPv4 is optional, but we explicitly have requirements
                    //to ignore the UDP checksum if present for IPv4 packets, even if incorrect.
                    if(udpHdrPtr->UDPChecksum)
                        udpHdrPtr->UDPChecksum = 0;
            }
        }
        else
        {
            /* IPv6 Header: Determine the total length of the outer IP Header. This is the total running
             * length of the packet excluding the L2 header and the IPv6 Header Size. */
            Netfp_IPv6Header* const ptrOuterIPv6Header = (Netfp_IPv6Header*)(ptrDataBuffer + l2HeaderSize);
            uint16_t const totalLength = ptrReassemblyContext->totalPacketLen - l2HeaderSize - IPv6HDR_SIZE;
            ptrOuterIPv6Header->PayloadLength = Netfp_ntohs(totalLength);

            // Is ESP packges?
            if (PASAHO_LINFO_IS_IPSEC(ptrFragmentInfo->pasaInfo) != 0 &&
                ptrOuterIPv6Header->NextHeader == 0x11) { // ESP package (UDP Encapsulation)
                Netfp_UDPHeader * const udpHdrPtr = (Netfp_UDPHeader*)((uint8_t *)ptrOuterIPv6Header + IPv6HDR_SIZE);
                udpHdrPtr->Length = Netfp_ntohs(totalLength - IPv6HDR_SIZE);
            }
        }
    }

    /* The networking headers have been modified. Writeback the networking headers we are about
     * to relinquish ownership to the CPDMA. The networking headers fit in 1 cache line. In the case
     * of IPv6 the data buffer is offsetted since we had removed the fragmentation header; this
     * would cause a cache alignment software breakpoint. So we simply use the original data buffer
     * to perform the writeback. */
    Cppi_getOriginalBufInfo (Cppi_DescType_HOST, (Cppi_Desc*)Pktlib_getDescFromPacket(ptrReassembledPacket),
                             &ptrDataBuffer, &dataBufferLen);
    ptrClientMCB->cfg.endMemAccess (ptrDataBuffer, CACHE_L2_LINESIZE);

#if (defined (DEVICE_K2H) || defined (DEVICE_K2K))
    /* FIX: The infamous FCS fix required to ensure that the Linux ethernet driver does NOT drop
     * the packet. The packet length at this time includes the IP payload length but does NOT
     * include the Ethernet FCS. Right now we blindly add the FCS at the end of the packet before
     * we push the reassembled packet back to the NETCP. This obvioulsy implies that the switch
     * has been programmed to always pass the FCS to the host.*/
    ptrReassemblyContext->totalPacketLen = ptrReassemblyContext->totalPacketLen + 4;
    Pktlib_setPacketLen(ptrReassembledPacket, ptrReassemblyContext->totalPacketLen);

    /* Cycle through and goto the last descriptor in the chained packet and increments it
     * buffer length to include the extra 4 bytes for the FCS. This is required else this is a
     * CPDMA violation */
    {
        Ti_Pkt*     ptrLastPkt;

        /* Get the last descriptor in the reassembled packet. */
        ptrLastPkt = ptrReassembledPacket;
        while (Pktlib_getNextPacket (ptrLastPkt) != NULL)
            ptrLastPkt = Pktlib_getNextPacket (ptrLastPkt);

        /* Get the data buffer length and include the extra 4 bytes of the FCS. */
        dataBufferLen = Pktlib_getDataBufferLen(ptrLastPkt);
        Pktlib_setDataBufferLen(ptrLastPkt, dataBufferLen + 4);
    }
#else
    /* Setup the packet length correctly: Without the 4 bytes CRC */
    Pktlib_setPacketLen(ptrReassembledPacket, ptrReassemblyContext->totalPacketLen);
#endif

    /* Can the reassembled packet be handled by the NETCP subsystem? */
    if (ptrReassemblyContext->totalPacketLen <= NETFP_PASS_MAX_BUFFER_SIZE)
    {
        /*********************************************************************************
         * NETFP Client: POST Reassembly Hook
         *********************************************************************************/
        if (Netfp_postReassemblyHook (ptrClientMCB, ptrReassemblyContext) < 0)
            return 0;

        /* Release ownership of the packet and push this back into the NETCP subsystem */
        Pktlib_releaseOwnership (ptrClientMCB->cfg.pktlibInstHandle, ptrReassembledPacket);
        Qmss_queuePushDescSize (ptrReassemblyContext->dstQueue, ptrReassembledPacket, 128);

        /* Cleanup the reassembly context: No need to clean the traffic flow from PA; since packet
         * has been completely reassembled and is being pushed back to the NETCP. The reassembled
         * packet will automatically get recycled by the NETCP CPDMA block. */
        Netfp_freeReassemblyContext (ptrClientMCB, ptrReassemblyMCB, ptrReassemblyContext, 0);
        return 0;
    }

    /* Increment the statistics */
    ptrReassemblyMCB->stats.numLargePackets++;

    /*********************************************************************************
     * NETFP Client: POST Reassembly Hook
     *********************************************************************************/
     if (Netfp_postReassemblyHook (ptrClientMCB, ptrReassemblyContext) < 0)
        return 0;

     /* Large packets are not pushed back to the NETCP; however the fragmented packets would have created
      * a traffic flow in the NETCP susbystem which we would need to clean up
      * This needs to be done before freeing packet because we copy some internals of original data */
     Netfp_freeReassemblyContext (ptrClientMCB, ptrReassemblyMCB, ptrReassemblyContext, 1);

    /* Large packets cannot be pushed back into the NETCP subsystem and so they need to be handled
     * within the NETFP client realm. Is the NETFP client interested in a large packet? */
    if (ptrReassemblyMCB->cfg.largePacketChannel)
    {
        /* Large Channel was configured; send the packet to the application. */
        Msgcom_putMessage (ptrReassemblyMCB->cfg.largePacketChannel, (MsgCom_Buffer*)ptrReassembledPacket);
    }
    else
    {
//fzm-->
        if(ptrReassemblyMCB->cfg.largePacketPushToSocket)
        {
            Netfp_IPAddr netfpAddr;
            uint16_t sin_port;
            int forwardToLinux = 1;
            int ethLen = PASAHO_LINFO_READ_VLAN_COUNT(pInfo) ? ETHHDR_SIZE + 4 : ETHHDR_SIZE;
            uint16_t ipHdrLen = 0;

            if (ptrFragmentInfo->ipVer == Netfp_IPVersion_IPV4)
            {
                const Netfp_IPHeader* const ptrIPHeader = (Netfp_IPHeader*)((uint8_t *)ptrDataBuffer + ptrFragmentInfo->ipOffset);
                if (ptrIPHeader->Protocol == IPPROTO_UDP)
                {
                    netfpAddr.ver = Netfp_IPVersion_IPV4;
                    memcpy(netfpAddr.addr.ipv4.u.a8, ptrIPHeader->IPDst, sizeof(netfpAddr.addr.ipv4.u.a8));

                    ipHdrLen = ((ptrIPHeader->VerLen & IPV4_HLEN_MASK) * 4);
                    const Netfp_UDPHeader* const udpHdrPtr = (Netfp_UDPHeader*)((uint8_t *)ptrIPHeader + ipHdrLen);
                    sin_port = Netfp_ntohs (udpHdrPtr->DstPort);

                    forwardToLinux = 0;
                }
            }
            else
            {
                const Netfp_IPv6Header* const ptrIPv6Header = (Netfp_IPv6Header*)((uint8_t *)ptrDataBuffer + ptrFragmentInfo->ipOffset);
                if (ptrIPv6Header->NextHeader == IPPROTO_UDP)
                {
                    netfpAddr.ver = Netfp_IPVersion_IPV6;
                    memcpy(netfpAddr.addr.ipv6.u.a8, ptrIPv6Header->DstAddr.u.a8, sizeof(netfpAddr.addr.ipv6.u.a8));

                    const Netfp_UDPHeader* const udpHdrPtr = (Netfp_UDPHeader*)((uint8_t *)ptrIPv6Header + IPv6HDR_SIZE);
                    sin_port = Netfp_ntohs (udpHdrPtr->DstPort);
                    ipHdrLen = IPv6HDR_SIZE;

                    forwardToLinux = 0;
                }
            }

            if (!forwardToLinux)
            {
                uint32_t appInfo;
                Qmss_QueueHnd socketQueue = Netfp_getSocketQueueByAddress (ptrClientMCB, &netfpAddr, sin_port, 0, &appInfo);
                if (socketQueue != -1)
                {
                    PASAHO_LINFO_SET_L3_OFFSET(pInfo, ethLen);
                    PASAHO_LINFO_SET_INNER_IP_OFFSET(pInfo, ethLen);
                    PASAHO_LINFO_SET_L4_OFFSET(pInfo, ethLen + ipHdrLen);
                    PASAHO_LINFO_SET_L5_OFFSET(pInfo, ethLen + ipHdrLen + UDPHDR_SIZE);
                    PASAHO_LINFO_SET_END_OFFSET(pInfo, ptrReassemblyContext->totalPacketLen - 4);
                    Cppi_setSoftwareInfo0(Cppi_DescType_HOST, (Cppi_Desc*)ptrReassembledPacket, appInfo);

                    ptrReassemblyMCB->stats.numLargePacketsFwdToSocket++;

                    Qmss_queuePushDesc (socketQueue, ptrReassembledPacket);
                    return 0;
                }
            }
        }

        if(ptrReassemblyMCB->cfg.largePacketHandler)
        {
            if(ptrReassemblyMCB->cfg.largePacketHandler((void *)ptrReassembledPacket) < 0)
            {
                Pktlib_freePacket (ptrClientMCB->cfg.pktlibInstHandle, ptrReassembledPacket);
            }
        }
        else
        {
            /* Nobody was interested in this clean up the reassembled packet.*/
            Pktlib_freePacket (ptrClientMCB->cfg.pktlibInstHandle, ptrReassembledPacket);
        }
//fzm<--
    }

    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function processes ingress packets from the PASS-assisted reassembly
 *      input queue. It performs the following tasks:
 *      @li Forward the non-fragmented IP packet with its flow id and count = 1
 *      to destination queue. This avoids reordering the non-fragmented packets.
 *      @li For IPSEC inner IP fragments, call SA LLD to perform the post-decryption
 *      operation including padding check and IPSEC header and authentication tag removal.
 *      @li Invoke the IP reassembly function
 *      @li Forward the reassembled IP packet with its flow id and fragments count
 *      to the destination queue.
 *
 *  @param[in]  ptrClientMCB
 *      Pointer to the client MCB which is handling the reassembly
 *  @param[in]  ptrFragment
 *      Fragmented packet which has been received and which needs to be reassembled
 *  @param[in]  dstQueue
 *      Destination Queue where the packet will be pushed once the reassembly is complete
 *  @param[out]  errCode
 *      Error code populated on error
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      0       - Success
 *  @retval
 *      <0      - Error
 */
static int32_t Netfp_reassembleFragment
(
    Netfp_ClientMCB*    ptrClientMCB,
    Ti_Pkt*             ptrFragment,
    Qmss_QueueHnd       dstQueue,
    int32_t*            errCode
)
{
    int32_t                     retVal;
    uint32_t                    fragmentDataLen;
    uint8_t*                    ptrFragmentData;
    Netfp_IPHeader*             ptrIPHeader;
    Netfp_IPv6Header*           ptrIPv6Header;
    Netfp_IPFragmentInfo        fragmentInfo;
    Ti_Pkt*                     ptrReassembledPacket;
    Netfp_IPReassemblyContext*  ptrReassemblyContext;
    uint32_t                    index;
    uint8_t                     ihl;
    uint16_t                    ipChecksum;
    Netfp_VLANHeader*           ptrVLANHeader;

    /* Get the PS information from the fragment. */
    if (unlikely(Cppi_getPSData (Cppi_DescType_HOST, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc *)ptrFragment,
                        (uint8_t **)&fragmentInfo.pasaInfo, &fragmentInfo.pasaInfoLen) != CPPI_SOK))
    {
        *errCode = NETFP_EINTERNAL;
        return -1;
    }

    /* Get the fragmented data buffer: */
    Pktlib_getDataBuffer(ptrFragment, &ptrFragmentData, &fragmentDataLen);

    /* Invalidate the data buffer: We only need to invalidate the Layer2 and IP Header here and not the
     * entire packet length; since the operations are being done only on these headers. Since the cache
     * operations are done on a CACHE LINE which is 128 bytes; it is safe to invalidate just 1 cache
     * line */
    ptrClientMCB->cfg.beginMemAccess (ptrFragmentData, CACHE_L2_LINESIZE);

    /*********************************************************************************
     * NETFP Client: PRE Reassembly Hook
     *********************************************************************************/
    if (Netfp_preReassemblyHook (ptrClientMCB, ptrFragment, dstQueue, PASAHO_LINFO_READ_TFINDEX(fragmentInfo.pasaInfo)) < 0)
        return 0;

    /* Packet was received. The packet could be fragmented or could be a non fragmented packet which
     * is belongs to the stream being reassembled. */
    fragmentInfo.tfIndex  = PASAHO_LINFO_READ_TFINDEX(fragmentInfo.pasaInfo);
    fragmentInfo.dstQueue = dstQueue;
    fragmentInfo.ipOffset = PASAHO_LINFO_READ_START_OFFSET(fragmentInfo.pasaInfo);
    fragmentInfo.inPort   = PASAHO_LINFO_READ_INPORT(fragmentInfo.pasaInfo);

    /* Do we have a VLAN Header? */
    if (fragmentInfo.ipOffset == (ETHHDR_SIZE + VLANHDR_SIZE))
    {
        /* YES: Get the VLAN Header: */
        ptrVLANHeader = (Netfp_VLANHeader*)(ptrFragmentData + ETHHDR_SIZE);

        /* Get the VLAN identifier */
        fragmentInfo.vlanId = Netfp_ntohs(ptrVLANHeader->tci);
        fragmentInfo.vlanId = fragmentInfo.vlanId & 0xFFF;
    }
    else
    {
        /* NO: Initialize the VLAN identifier. */
        fragmentInfo.vlanId = 0;
    }

    /* Get the IP Header: */
    ptrIPHeader = (Netfp_IPHeader*)(ptrFragmentData + fragmentInfo.ipOffset);

    /* Determine the IP version and perform the reassembly operation accordingly. */
    if (Netfp_isIPv4Packet (ptrIPHeader))
    {
        /* IPv4 Packet: Validate the IPv4 header of the received packet */
        ihl = (ptrIPHeader->VerLen & 0xF);
        if (unlikely(ihl < 5))
        {
            /* Error: Invalid IPv4 Header Length */
            ptrClientMCB->ptrReassemblyMCB->stats.numIPv4HeaderError++;
            Netfp_freeFragment (ptrClientMCB, fragmentInfo.tfIndex, dstQueue, ptrFragment);
            return 0;
        }

        /* IPv4 Header checksum validation: */
        ipChecksum = ptrIPHeader->Checksum;
        Netfp_IPChecksum (ptrIPHeader);
        if (unlikely(ipChecksum != ptrIPHeader->Checksum))
        {
            /* Error: Bad IPv4 Checksum */
            ptrClientMCB->ptrReassemblyMCB->stats.numIPv4HeaderError++;
            Netfp_freeFragment (ptrClientMCB, fragmentInfo.tfIndex, dstQueue, ptrFragment);
            return 0;
        }

        /* Checksum calculations have caused the cache line to be dirty. We perform checksum calculations for
         * all fragments. We need to writeback the cache contents here else the buffer can be randomly overwritten
         * when the cache lines are evicted */
        ptrClientMCB->cfg.endMemAccess (ptrFragmentData, CACHE_L2_LINESIZE);

        /* Get the fragmention offset: */
        fragmentInfo.fragOffset  = Netfp_isIPv4PacketFragmented(ptrIPHeader);

        /* Populate the fragmentation information only if the packet is fragmented. */
        if (fragmentInfo.fragOffset != 0)
        {
            /* Packet was fragmented; populate the fragmentation information. */
            fragmentInfo.ptrIPHeader = (uint8_t*)ptrIPHeader;
            fragmentInfo.ipVer       = Netfp_IPVersion_IPV4;

            /* Extract the fields from the IPv4 header */
            fragmentInfo.protocol                = ptrIPHeader->Protocol;
            fragmentInfo.id                      = Netfp_ntohs (ptrIPHeader->Id);
            fragmentInfo.srcAddr.ver             = Netfp_IPVersion_IPV4;
            fragmentInfo.srcAddr.addr.ipv4.u.a32 = (ptrIPHeader->IPSrc[0] << 24) | (ptrIPHeader->IPSrc[1] << 16) |
                                                   (ptrIPHeader->IPSrc[2] << 8)  | (ptrIPHeader->IPSrc[3]);
            fragmentInfo.dstAddr.ver             = Netfp_IPVersion_IPV4;
            fragmentInfo.dstAddr.addr.ipv4.u.a32 = (ptrIPHeader->IPDst[0] << 24) | (ptrIPHeader->IPDst[1] << 16) |
                                                   (ptrIPHeader->IPDst[2] << 8)  | (ptrIPHeader->IPDst[3]);

            /* Increment the number of fragments received. */
            ptrClientMCB->ptrReassemblyMCB->stats.numIPv4Fragments++;
        }
    }
    else
    {
        /* IPv6 Packet: */
        ptrIPv6Header = (Netfp_IPv6Header*)(ptrFragmentData + fragmentInfo.ipOffset);

        /* Parse the Extension Headers in the IPv6 Packet: */
        fragmentInfo.numIPv6ExtHeader = Netfp_parseIPv6ExtensionHeader(ptrIPv6Header, &fragmentInfo.ipv6ExtensionHeader[0],
                                                                       &fragmentInfo.ptrIPv6FragHeader, &fragmentInfo.sizeIPv6ExtensionHeader);
        if (unlikely(fragmentInfo.numIPv6ExtHeader < 0))
        {
            /* Error: There seems to be an error parsing the IPv6 Extension headers */
            Netfp_freeFragment (ptrClientMCB, fragmentInfo.tfIndex, dstQueue, ptrFragment);

            /* Increment the statistics */
            ptrClientMCB->ptrReassemblyMCB->stats.numIPv6HeaderError++;
            return 0;
        }

        /* Is the packet a fragment? */
        if (fragmentInfo.ptrIPv6FragHeader != NULL)
        {
            /* YES: Populate the fragmentation information only if the packet is fragmented. */
            fragmentInfo.fragOffset  = Netfp_ntohs(fragmentInfo.ptrIPv6FragHeader->FragOffset);
            fragmentInfo.ipVer       = Netfp_IPVersion_IPV6;
            fragmentInfo.ptrIPHeader = (uint8_t*)ptrIPv6Header;
            fragmentInfo.protocol    = fragmentInfo.ptrIPv6FragHeader->NextHeader;
            fragmentInfo.id          = (fragmentInfo.ptrIPv6FragHeader->FragId[0] << 24) | (fragmentInfo.ptrIPv6FragHeader->FragId[1] << 16) |
                                       (fragmentInfo.ptrIPv6FragHeader->FragId[2] << 8)  | (fragmentInfo.ptrIPv6FragHeader->FragId[3]);

            /* Populate the source & destination IP address from the IPv6 header */
            fragmentInfo.srcAddr.ver = Netfp_IPVersion_IPV6;
            for (index = 0; index < 16; index++)
                fragmentInfo.srcAddr.addr.ipv6.u.a8[index] = ptrIPv6Header->SrcAddr.u.a8[index];
            fragmentInfo.dstAddr.ver = Netfp_IPVersion_IPV6;
            for (index = 0; index < 16; index++)
                fragmentInfo.dstAddr.addr.ipv6.u.a8[index] = ptrIPv6Header->DstAddr.u.a8[index];

            /* Increment the number of fragments received. */
            ptrClientMCB->ptrReassemblyMCB->stats.numIPv6Fragments++;
        }
        else
        {
            /* No IPv6 Fragment Extension header detected. This is not a fragment */
            fragmentInfo.fragOffset = 0;
        }
    }

    /* Check if a fragmented packet has been received? */
    if (fragmentInfo.fragOffset == 0)
    {
        /* Packet is non fragmented; forward the packet to the NETCP immediately. There is no need
         * to writeback the contents into the cache since there was no modification to the packet
         * or to the data buffer. */
        Qmss_queuePushDescSize (dstQueue, ptrFragment, 128);
        return 0;
    }

    /* Get the reassembly context which matches the received fragment */
    ptrReassemblyContext = Netfp_findReassemblyContext (ptrClientMCB->ptrReassemblyMCB, &fragmentInfo);
    if (ptrReassemblyContext == NULL)
    {
        /* Allocate a new reassembly context: */
        ptrReassemblyContext = Netfp_allocateReassemblyContext (ptrClientMCB->ptrReassemblyMCB);
        if (ptrReassemblyContext == NULL)
        {
            ptrReassemblyContext = Netfp_freeAndGetOldestReassemblyContext (ptrClientMCB);
            if (unlikely(ptrReassemblyContext == NULL))
            {
                /* Error: There is NO reassembly context available to handle the received fragment. We cannot process
                 * the fragment so we push it back to the NETCP such that the fragment count is decremented. */
                Netfp_freeFragment (ptrClientMCB, fragmentInfo.tfIndex, dstQueue, ptrFragment);

                /* Increment the statistics */
                ptrClientMCB->ptrReassemblyMCB->stats.noReassemblyContext++;
                return 0;
            }
            ptrClientMCB->ptrReassemblyMCB->stats.numFreedOldContext++;
        }

        /* This is a new reassembly context which has been created; determine if the traffic flow is being
         * handled by the NETCP or not and increment the statistics. */
        if (fragmentInfo.tfIndex == PA_INV_TF_INDEX)
            ptrClientMCB->ptrReassemblyMCB->stats.nonAccleratedTrafficFlow++;

        /* Populate the new allocated reassembly block: */
        ptrReassemblyContext->protocol         = fragmentInfo.protocol;
        ptrReassemblyContext->id               = fragmentInfo.id;
        ptrReassemblyContext->dstQueue         = fragmentInfo.dstQueue;
        ptrReassemblyContext->tfId             = fragmentInfo.tfIndex;
        ptrReassemblyContext->ipOffset         = fragmentInfo.ipOffset;
        ptrReassemblyContext->inPort           = fragmentInfo.inPort;
        ptrReassemblyContext->vlanId           = fragmentInfo.vlanId;
        ptrReassemblyContext->ptrPkt           = NULL;
        ptrReassemblyContext->ptrLastFrag      = NULL;
        ptrReassemblyContext->fragCnt          = 0;
        ptrReassemblyContext->origIPLength     = 0;
        ptrReassemblyContext->detectedIPLen    = 0;
        ptrReassemblyContext->overlapFrag      = 0;
        ptrReassemblyContext->corruptedPkt     = 0;
        ptrReassemblyContext->totalPacketLen   = 0;
        ptrReassemblyContext->timeout          = ptrClientMCB->ptrReassemblyMCB->cfg.reassemblyTimeout;
        memcpy (&ptrReassemblyContext->srcAddr, &fragmentInfo.srcAddr, sizeof (Netfp_IPAddr));
        memcpy (&ptrReassemblyContext->dstAddr, &fragmentInfo.dstAddr, sizeof (Netfp_IPAddr));
    }
    else
    {
        /* Reassembly context found: Ensure that the traffic flow id in the context matches the
         * traffic flow id reported in the received fragment */
        if (unlikely(ptrReassemblyContext->tfId != fragmentInfo.tfIndex))
        {
            Netfp_freeFragment(ptrClientMCB, fragmentInfo.tfIndex, dstQueue, ptrFragment); //fzm
            ptrClientMCB->ptrReassemblyMCB->stats.trafficFlowMismatch++;

            *errCode = NETFP_EINTERNAL;
            return -1;
        }
    }

    /* Sanity Check: All the fragments for the reassembly context should be received on the same switch port. */
    if (unlikely(ptrReassemblyContext->inPort != fragmentInfo.inPort))
    {
        /* Drop the reassembly context:  */
        ptrClientMCB->ptrReassemblyMCB->stats.numInPortError++;
        Netfp_freeFragment (ptrClientMCB, ptrReassemblyContext->tfId, ptrReassemblyContext->dstQueue, ptrFragment);
        return 0;
    }

    /* Sanity Check: All the fragments should have the same L2 Header Size */
    if (unlikely(ptrReassemblyContext->ipOffset != fragmentInfo.ipOffset))
    {
        /* Drop the reassembly context: We have received some fragments with a VLAN and some fragments without a
         * VLAN. This is not valid and we can simply drop the fragment. */
        ptrClientMCB->ptrReassemblyMCB->stats.numL2HeaderError++;
        Netfp_freeFragment (ptrClientMCB, ptrReassemblyContext->tfId, ptrReassemblyContext->dstQueue, ptrFragment);
        return 0;
    }

    /* Sanity Check: All the fragments should have the same VLAN Identifier. */
    if (unlikely(ptrReassemblyContext->vlanId != fragmentInfo.vlanId))
    {
        /* Drop the reassembly context: Mismatch of the VLAN identifier. */
        ptrClientMCB->ptrReassemblyMCB->stats.numL2HeaderError++;
        Netfp_freeFragment (ptrClientMCB, ptrReassemblyContext->tfId, ptrReassemblyContext->dstQueue, ptrFragment);
        return 0;
    }

    /* Is the reassembly context poisoned because of an corrupted fragment? */
    if (unlikely(ptrReassemblyContext->corruptedPkt == 1))
    {
        /* Drop the all new fragments which belong to this poisoned context: */
        Netfp_freeFragment (ptrClientMCB, ptrReassemblyContext->tfId, ptrReassemblyContext->dstQueue, ptrFragment);
        return 0;
    }

    /* Is the reassembly context poisoned because of an overlapping fragment? */
    if (unlikely(ptrReassemblyContext->overlapFrag == 1))
    {
        /* Drop all new fragments which belong to this poisoned context: */
        Netfp_freeFragment (ptrClientMCB, ptrReassemblyContext->tfId, ptrReassemblyContext->dstQueue, ptrFragment);
        return 0;
    }

    /* Increment the fragment count in the reassembly context. */
    ptrReassemblyContext->fragCnt++;

    /* Increment the stats: We have received an active fragment. */
    ptrClientMCB->ptrReassemblyMCB->stats.numActiveFragments++;

    /* Perform the appropriate IP reassembly operation: */
    if (fragmentInfo.ipVer == Netfp_IPVersion_IPV4)
        retVal = Netfp_ipv4Reassembly (ptrClientMCB, ptrReassemblyContext, ptrFragment, &fragmentInfo, &ptrReassembledPacket, errCode);
    else
        retVal = Netfp_ipv6Reassembly (ptrClientMCB, ptrReassemblyContext, ptrFragment, &fragmentInfo, &ptrReassembledPacket, errCode);

    /* Was the reassembly operation successful? */
    if (unlikely(retVal < 0))
        return -1;

    /* Was the packet fully reassembled? */
    if (ptrReassembledPacket == NULL)
        return 0;

    /* Packet has been reassembled. Complete the reassembly operations. */
    return Netfp_completeReassembly (ptrClientMCB, ptrReassemblyContext, ptrReassembledPacket, &fragmentInfo, errCode);
}

/**
 *  @b Description
 *  @n
 *      The function needs to be invoked by the application to ensure that
 *      the fragmented packets are received and reassembled. The function
 *      should only be invoked by the NETFP Client which has registered itself
 *      to handle the reassembly.
 *
 *  @param[in]  clientHandle
 *      NETFP Client Handle for which the reassembly is required.
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  @sa
 *      Netfp_registerReassemblyServices
 *
 *  \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_reassembly
(
    Netfp_ClientHandle  clientHandle,
    int32_t*            errCode
)
{
    Netfp_ReassemblyMCB*    ptrReassemblyMCB;
    Ti_Pkt*                 ptrFragment;
    Netfp_ClientMCB*        ptrClientMCB;
    int32_t                 retVal;
    // <fzm>
    uint32_t                processed = 0;
    // </fzm>

    /* Sanity Check: Ensure that the client is the registered to handle the NETFP Reassembly services */
    ptrClientMCB = (Netfp_ClientMCB*)clientHandle;
    if (unlikely((ptrClientMCB == NULL) || (ptrClientMCB->ptrReassemblyMCB == NULL)))
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Get the reassembly MCB */
    ptrReassemblyMCB = ptrClientMCB->ptrReassemblyMCB;

//<fzm>
    Qmss_QueueHnd queueHandle = Msgcom_getInternalMsgQueueInfo(ptrReassemblyMCB->cfg.outerIpReassemChannel);
//</fzm>

    /* Process all the messages which might be present on the outer IP reassembly channel. */
    while (/*<fzm>*/(ptrReassemblyMCB->cfg.maxPacketBurst == 0) || (processed++ < ptrReassemblyMCB->cfg.maxPacketBurst)/*</fzm>*/)
    {
        //<fzm>
        uint32_t limit = !ptrReassemblyMCB->cfg.maxPacketBurst ? PKTLIB_PREFETCHED_DESCRIPTORS_COUNT : (ptrReassemblyMCB->cfg.maxPacketBurst - processed);

        /* Receive messages from the outer IP reassembly channel. */
        ptrFragment = Pktlib_popPrefetch(queueHandle, &ptrReassemblyMCB->prefetchOuterData, limit);

        /* if (Msgcom_getMessage (ptrReassemblyMCB->cfg.outerIpReassemChannel, (MsgCom_Buffer**)&ptrFragment) < 0)
        {
            *errCode = NETFP_EINVAL;
            return -1;
        }*/

        //</fzm>

        /* Have all the fragments been processed. */
        if (ptrFragment == NULL)
            break;

        /* Increment the number of received outer IP fragments. */
        ptrReassemblyMCB->stats.numOuterIPPktReceived++;

        /* Reassemble the fragment. */
        retVal = Netfp_reassembleFragment(ptrClientMCB, ptrFragment, ptrClientMCB->netcpTxQueue[NSS_PA_QUEUE_OUTER_IP_INDEX], errCode);
        if (unlikely(retVal < 0))
            return -1;
    }

    // <fzm>
    processed = 0;

    queueHandle = Msgcom_getInternalMsgQueueInfo(ptrReassemblyMCB->cfg.innerIpReassemChannel);
    // </fzm>

    /* Process all the messages which might be present on the inner IP reassembly channel. */
    while (/*<fzm>*/(ptrReassemblyMCB->cfg.maxPacketBurst == 0) || (processed++ < ptrReassemblyMCB->cfg.maxPacketBurst)/*</fzm>*/)
    {
        //<fzm>
        uint32_t limit = !ptrReassemblyMCB->cfg.maxPacketBurst ? PKTLIB_PREFETCHED_DESCRIPTORS_COUNT : (ptrReassemblyMCB->cfg.maxPacketBurst - processed);

        /* Receive messages from the outer IP reassembly channel. */
        ptrFragment = Pktlib_popPrefetch(queueHandle, &ptrReassemblyMCB->prefetchInnerData, limit);

        /* Receive messages from the outer IP reassembly channel. */
        /*if (Msgcom_getMessage (ptrReassemblyMCB->cfg.innerIpReassemChannel, (MsgCom_Buffer**)&ptrFragment) < 0)
        {
            *errCode = NETFP_EINVAL;
            return -1;
        }*/

        //</fzm>

        /* Have all the fragments been processed. */
        if (ptrFragment == NULL)
            break;

        /* Increment the number of received inner IP fragments. */
        ptrReassemblyMCB->stats.numInnerIPPktReceived++;

        /* Reassemble the fragment. */
        retVal = Netfp_reassembleFragment(ptrClientMCB, ptrFragment, ptrClientMCB->netcpTxQueue[NSS_PA_QUEUE_INNER_IP_INDEX], errCode);
        if (unlikely(retVal < 0))
            return -1;
    }

    /* Reassembly management module. */
    ptrReassemblyMCB->cfg.reassemblyMgmt (ptrClientMCB, ptrReassemblyMCB->cfg.ptrReassemblyMgmtCfg);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function needs to be invoked by the application periodically every
 *      "timeout" ticks. The function is used to provide age out *stale* reassembly
 *      contexts.
 *
 *      For performance reasons there is no protection of the reassembly context lists
 *      between this function and @sa Netfp_reassembly; so it is highly *recommended*
 *      that these functions get invoked from the same thread context.
 *
 *  @param[in]  clientHandle
 *      NETFP Client Handle for which the reassembly is required.
 *  @param[in]  timeout
 *      Timeout in ticks after which the function is being invoked.
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
int32_t Netfp_reassemblyTimerTick
(
    Netfp_ClientHandle  clientHandle,
    int32_t             timeout,
    int32_t*            errCode
)
{
    Netfp_ReassemblyMCB*        ptrReassemblyMCB;
    Netfp_ClientMCB*            ptrClientMCB;
    Netfp_IPReassemblyContext*  ptrReassemblyContext;
    Ti_Pkt*                     ptrPkt;

    /* Sanity Check: */
    ptrClientMCB = (Netfp_ClientMCB*)clientHandle;
    if ((ptrClientMCB == NULL) || (ptrClientMCB->ptrReassemblyMCB == NULL))
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Get the reassembly MCB */
    ptrReassemblyMCB = ptrClientMCB->ptrReassemblyMCB;

    /* Age out the reassembly entries: */
    ptrReassemblyContext = (Netfp_IPReassemblyContext*)Netfp_listGetHead ((Netfp_ListNode**)&ptrReassemblyMCB->ptrUsedContextList);
    while (ptrReassemblyContext != NULL)
    {
        /* Handle the timeout. */
        ptrReassemblyContext->timeout = ptrReassemblyContext->timeout - timeout;
        if (ptrReassemblyContext->timeout <= 0)
        {
            Netfp_IPReassemblyContext* ptrNextReassemblyContext = NULL;

            /* Reassembly context has timed out */
            ptrReassemblyMCB->stats.numReassemblyTimeout++;

            /* Get the next element in the list.  */
            ptrNextReassemblyContext = (Netfp_IPReassemblyContext*)Netfp_listGetNext ((Netfp_ListNode*)ptrReassemblyContext);

            ptrPkt = ptrReassemblyContext->ptrPkt;

            /* Free the reassembled context and also cleanup the traffic flow in the NETCP subsystem. */
            Netfp_freeReassemblyContext (ptrClientMCB, ptrReassemblyMCB, ptrReassemblyContext, 1);

            /* Cleanup the *partial* reassembled packet. */
            Pktlib_freePacket (ptrClientMCB->cfg.pktlibInstHandle, ptrPkt);

            /* Traverse the list to the next element on the list */
            ptrReassemblyContext = ptrNextReassemblyContext;
        }
        else
        {
        	/* No time out: Traverse the list and get the next element in the list.  */
        	ptrReassemblyContext = (Netfp_IPReassemblyContext*)Netfp_listGetNext ((Netfp_ListNode*)ptrReassemblyContext);
        }
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The service function which is invoked by the NETFP Client to configure the reassembly
 *      in the NETCP subsystem. The PA inner and outer IP reassembly configurations have already
 *      been populated. This function is invoked during the reassembly initializations invoked
 *      for a specific NETFP client.
 *
 *  @sa
 *      Netfp_initReassembly
 *
 *  @param[in]  clientHandle
 *      NETFP Client Handle
 *  @param[in]  ptrInnerIPReassemblyConfig
 *      Pointer to the inner IP reassembly configuration
 *  @param[in]  ptrOuterIPReassemblyConfig
 *      Pointer to the outer IP reassembly configuration
 *  @param[out] errCode
 *      Error code populated on error.
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_configureReassembly
(
    Netfp_ClientHandle  clientHandle,
    paIpReassmConfig_t* ptrInnerIPReassemblyConfig,
    paIpReassmConfig_t* ptrOuterIPReassemblyConfig,
    int32_t*            errCode
)
{
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    uint32_t                result;
    int32_t                 jobId;
    Netfp_ClientMCB*        ptrNetfpClient;
    paIpReassmConfig_t*     ptrRemoteReassemblyConfig;

    /* Get the NETFP Client MCB: Services are available only if the client is active. */
    ptrNetfpClient = (Netfp_ClientMCB*)clientHandle;
    if (ptrNetfpClient->status != Netfp_ClientStatus_ACTIVE)
    {
        *errCode = NETFP_ENOTREADY;
        return -1;
    }

    /* Get the actual job which we will execute */
    jobHandle = Josh_findJobByAddress(ptrNetfpClient->joshHandle, (Josh_JobProtype)_Netfp_configureReassembly);
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
     *  int32_t _Netfp_configureReassembly
     *  (
     *  Netfp_ServerMCB*        ptrNetfpServer,
     *  paIpReassmConfig_t*     ptrInnerIPReassemblyConfig,
     *  paIpReassmConfig_t*     ptrOuterIPReassemblyConfig,
     *  int32_t*                errCode
     *  )
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Netfp_ServerHandle);

    /*  - Argument 2: */
    args[1].type   = Josh_ArgumentType_PASS_BY_REF;
    args[1].length = sizeof(paIpReassmConfig_t);

    /*  - Argument 3: */
    args[2].type   = Josh_ArgumentType_PASS_BY_REF;
    args[2].length = sizeof(paIpReassmConfig_t);

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

    /* Populate the NETFP Server handle */
    *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(ptrNetfpClient->cfg.serverHandle);

    /* Get the pointer to the inner IP reassembly configuration. */
    ptrRemoteReassemblyConfig = (paIpReassmConfig_t*)args[1].argBuffer;
    memcpy ((void*)ptrRemoteReassemblyConfig, (void *)ptrInnerIPReassemblyConfig, sizeof (paIpReassmConfig_t));

    /* Get the pointer to the outer IP reassembly configuration. */
    ptrRemoteReassemblyConfig = (paIpReassmConfig_t*)args[2].argBuffer;
    memcpy ((void*)ptrRemoteReassemblyConfig, (void *)ptrOuterIPReassemblyConfig, sizeof (paIpReassmConfig_t));

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
 *      The function is invoked by the NETFP client to register itself to handle
 *      the reassembly of all fragmented packets. There can only be one designated
 *      reassembly client in the system.
 *
 *  @param[in]  clientHandle
 *      Handle to the NETFP Client which is responsible for handling reassembly
 *  @param[in]  ptrReassemblyConfig
 *      Pointer to the reassembly configuration
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
int32_t Netfp_registerReassemblyService
(
    Netfp_ClientHandle      clientHandle,
    Netfp_ReassemblyConfig* ptrReassemblyConfig,
    int32_t*                errCode
)
{
    Netfp_IPReassemblyContext*  ptrIPReassemblyContext;
    Netfp_ReassemblyMCB*        ptrReassemblyMCB;
    Netfp_FlowCfg               flowCfg;
    paIpReassmConfig_t          paInnerIPReassemblyConfig;
    paIpReassmConfig_t          paOuterIPReassemblyConfig;
    Netfp_ClientMCB*            ptrNetfpClient;
    Pktlib_HeapCfg              heapCfg;
    uint32_t                    index;

    /* Sanity Check: Ensure that the arguments are valid */
    if ((clientHandle == NULL) || (ptrReassemblyConfig == NULL))
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Sanity Check: The configuration should provide for some number of free descriptors which can
     * used to receive the fragments. */
    if (ptrReassemblyConfig->numFragmentedPkt == 0)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Sanity Check: Reassembly Inner & Outer channels need to have been specified. These are the
     * channels where the packets are pushed once the reassembly is complete. */
    if ((ptrReassemblyConfig->outerIpReassemChannel == NULL) || (ptrReassemblyConfig->innerIpReassemChannel == NULL))
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Sanity Check: Ensure that the reassembly allocated & cleanup functions are provided. */
    if ((ptrReassemblyConfig->reassemblyHeapAlloc == NULL) ||
        (ptrReassemblyConfig->reassemblyHeapFree == NULL))
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Sanity Check: Validate the application supplied reassembly management support */
    if (ptrReassemblyConfig->reassemblyMgmt == NULL)
    {
        /* This indicates that we need to select the default NETFP reassembly management module
         * Validate the rest of the arguments: */
        if ((ptrReassemblyConfig->ptrReassemblyMgmtCfg == NULL) ||
            (ptrReassemblyConfig->sizeReassemblyCfg    != sizeof(Netfp_DefaultReassemblyMgmtCfg)))
        {
            *errCode = NETFP_EINVAL;
            return -1;
        }
    }
    else
    {
        /* Application provided reassembly management module. There is no need to perform any
         * validations; simply fall through. */
    }

    /* Get the NETFP Client MCB: Services are available only if the client is active. */
    ptrNetfpClient = (Netfp_ClientMCB*)clientHandle;
    if (ptrNetfpClient->status != Netfp_ClientStatus_ACTIVE)
    {
        *errCode = NETFP_ENOTREADY;
        return -1;
    }

    /* Allocate memory for the Reassembly MCB */
    ptrReassemblyMCB = ptrNetfpClient->cfg.malloc (sizeof(Netfp_ReassemblyMCB), 0);
    if (ptrReassemblyMCB == NULL)
    {
        *errCode = NETFP_ENOMEM;
        return -1;
    }

    /* Initialize the allocated memory block */
    memset ((void *)ptrReassemblyMCB , 0, sizeof(Netfp_ReassemblyMCB));

    /* Remember the pointer in the NETFP Client. */
    ptrNetfpClient->ptrReassemblyMCB = ptrReassemblyMCB;

    /* Copy over the reassembly configuration: */
    memcpy ((void *)&ptrReassemblyMCB->cfg, (void *)ptrReassemblyConfig, sizeof(Netfp_ReassemblyConfig));

    /* Setup the reassembly management modules configuration: */
    if (ptrReassemblyConfig->sizeReassemblyCfg != 0)
    {
        /* Allocate memory for the Reassembly management module */
        ptrReassemblyMCB->cfg.ptrReassemblyMgmtCfg = ptrNetfpClient->cfg.malloc (ptrReassemblyConfig->sizeReassemblyCfg, 0);
        if (ptrReassemblyMCB->cfg.ptrReassemblyMgmtCfg == NULL)
        {
            *errCode = NETFP_ENOMEM;
            return -1;
        }

        /* Initialize the allocated memory */
        memset ((void *)ptrReassemblyMCB->cfg.ptrReassemblyMgmtCfg, 0, ptrReassemblyConfig->sizeReassemblyCfg);

        /* Copy over the configuration */
        memcpy ((void *)ptrReassemblyMCB->cfg.ptrReassemblyMgmtCfg, (void*)ptrReassemblyConfig->ptrReassemblyMgmtCfg,
                ptrReassemblyConfig->sizeReassemblyCfg);
    }
    else
    {
        ptrReassemblyMCB->cfg.ptrReassemblyMgmtCfg = NULL;
    }

    /* Copy over the remaining reassembly management configuration parameters  */
    ptrReassemblyMCB->cfg.sizeReassemblyCfg = ptrReassemblyConfig->sizeReassemblyCfg;

    /* Plugin the default reassembly management API if required. */
    if (ptrReassemblyConfig->reassemblyMgmt == NULL)
        ptrReassemblyMCB->cfg.reassemblyMgmt = Netfp_defaultReassemblyTrafficMgmt;

    /* Create a "client local" reassembly heap from the specified memory region. This heap is now
     * created within the NETFP module so that we can writeback & invalidate the packets for
     * performance. This heap cannot be reused by an application which can break this optimization. */
    memset ((void *)&heapCfg, 0, sizeof(Pktlib_HeapCfg));

    /* Populate the heap configuration */
    strncpy(heapCfg.name, "__NETFP-Client-Reassembly-Heap", PKTLIB_MAX_CHAR);
    heapCfg.memRegion                       = ptrReassemblyConfig->reassemblyMemRegion;
    heapCfg.pktlibInstHandle                = ptrNetfpClient->cfg.pktlibInstHandle;
    heapCfg.sharedHeap                      = 0;
    heapCfg.useStarvationQueue              = 1;
    heapCfg.dataBufferSize                  = 9800 + 128; //fzm
    heapCfg.numPkts                         = ptrReassemblyConfig->numFragmentedPkt;
    heapCfg.numZeroBufferPackets            = 0;
    heapCfg.dataBufferPktThreshold          = 0;
    heapCfg.zeroBufferPktThreshold          = 0;
    heapCfg.arg                             = ptrReassemblyConfig->reassemblyHeapArg;
    heapCfg.heapInterfaceTable.dataMalloc   = ptrReassemblyConfig->reassemblyHeapAlloc;
    heapCfg.heapInterfaceTable.dataFree     = ptrReassemblyConfig->reassemblyHeapFree;
    if(ptrReassemblyConfig->pushToHead)
        heapCfg.pushToHead = 1;
    heapCfg.linearAlloc = 1;

    /* Create the NETFP Client reassembly heap */
    ptrReassemblyMCB->reassemblyHeapHandle = Pktlib_createHeap(&heapCfg, errCode);
    if (ptrReassemblyMCB->reassemblyHeapHandle == NULL)
        return -1;

    /* Initialize the flow configuration. */
    memset((void *)&flowCfg, 0, sizeof(Netfp_FlowCfg));

    /* Create a reassembly flow. This flow will be used when IP fragments are received by the NetCP and
     * are forwarded to the NETFP client for reassembly */
    flowCfg.numHeaps      = 1;
    flowCfg.heapHandle[0] = ptrReassemblyMCB->reassemblyHeapHandle;
    flowCfg.sopOffset     = 0;
    strncpy (flowCfg.name, "__NETFP-Client-Reassembly-Flow", NETFP_MAX_CHAR);
    ptrReassemblyMCB->reassemblyFlowId = Netfp_createFlow (clientHandle, &flowCfg, errCode);
    if (ptrReassemblyMCB->reassemblyFlowId < 0)
        return -1;

    /* Allocate memory for the reassembly contexts */
    for (index = 0; index < ptrReassemblyMCB->cfg.numReassemblyContexts; index++)
    {
        /* Allocate memory for the IP reassembly context */
        ptrIPReassemblyContext = ptrNetfpClient->cfg.malloc (sizeof(Netfp_IPReassemblyContext), 0);
        if (ptrIPReassemblyContext == NULL)
        {
            *errCode = NETFP_ENOMEM;
            return -1;
        }

        /* Initialize the allocated memory block */
        memset ((void *)ptrIPReassemblyContext, 0, sizeof(Netfp_IPReassemblyContext));

        /* Remember the reassembly MCB to which the context belongs. */
        ptrIPReassemblyContext->ptrReassemblyMCB = ptrReassemblyMCB;

        /* Add the context to the free list. */
        Netfp_listAdd ((Netfp_ListNode**)&ptrReassemblyMCB->ptrFreeContextList, (Netfp_ListNode*)ptrIPReassemblyContext);
    }

    /* Setup the inner & outer ip reassembly configurations. */
    paInnerIPReassemblyConfig.numTrafficFlow = pa_MAX_IP_REASM_TRAFFIC_FLOWS;
    paInnerIPReassemblyConfig.destFlowId     = ptrReassemblyMCB->reassemblyFlowId ;
    paInnerIPReassemblyConfig.destQueue      = PA_BOUNCE_QUEUE_DDR(Msgcom_getInternalMsgQueueInfo(ptrReassemblyConfig->innerIpReassemChannel));
    paOuterIPReassemblyConfig.numTrafficFlow = pa_MAX_IP_REASM_TRAFFIC_FLOWS;
    paOuterIPReassemblyConfig.destFlowId     = ptrReassemblyMCB->reassemblyFlowId ;
    paOuterIPReassemblyConfig.destQueue      = PA_BOUNCE_QUEUE_DDR(Msgcom_getInternalMsgQueueInfo(ptrReassemblyConfig->outerIpReassemChannel));

    /* Configure the NETFP Reassembly using the service module. */
    return Netfp_configureReassembly (clientHandle, &paInnerIPReassemblyConfig, &paOuterIPReassemblyConfig, errCode);
}

/**
 *  @b Description
 *  @n
 *      The function is invoked by the NETFP client to deregister itself from the
 *      reassembly handling. The function will shutdown the reassembly block in the
 *      NETC. However the function can fail (NETFP_ENOTREADY) because it is possible
 *      that fragments are already pending in the Outer/Inner MSGCOM channels.
 *      Applications would need to free up all these fragments and invoke the API again.
 *
 *  @param[in]  clientHandle
 *      Handle to the NETFP Client which is responsible for handling reassembly
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
int32_t Netfp_deregisterReassemblyService
(
    Netfp_ClientHandle      clientHandle,
    int32_t*                errCode
)
{
    Netfp_IPReassemblyContext*  ptrIPReassemblyContext;
    Netfp_ReassemblyMCB*        ptrReassemblyMCB;
    paIpReassmConfig_t          paInnerIPReassemblyConfig;
    paIpReassmConfig_t          paOuterIPReassemblyConfig;
    Netfp_ClientMCB*            ptrNetfpClient;
    Ti_Pkt*                     ptrFragment;

    /* Sanity Check: Ensure that the arguments are valid */
    if (clientHandle == NULL)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Get the NETFP Client: */
    ptrNetfpClient = (Netfp_ClientMCB*)clientHandle;

    /* Get the Reassembly MCB */
    ptrReassemblyMCB = ptrNetfpClient->ptrReassemblyMCB;

    /* Was the client registered for reassembly? */
    if (ptrReassemblyMCB == NULL)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* In order to disable the reassembly block. We need to set the number of traffic flows in the reassembly
     * configuration to 0 */
    memset ((void *)&paInnerIPReassemblyConfig, 0, sizeof(paIpReassmConfig_t));
    memset ((void *)&paOuterIPReassemblyConfig, 0, sizeof(paIpReassmConfig_t));

    /* Configure the reassembly block: */
    if (Netfp_configureReassembly (clientHandle, &paInnerIPReassemblyConfig, &paOuterIPReassemblyConfig, errCode) < 0)
        return -1;

    /* Cleanup any outstanding fragments which have been received on the Outer IP channel: */
    while (1)
    {
        if (Msgcom_getMessage (ptrReassemblyMCB->cfg.outerIpReassemChannel, (MsgCom_Buffer**)&ptrFragment) < 0)
        {
            *errCode = NETFP_EINVAL;
            return -1;
        }
        if (ptrFragment == NULL)
            break;

        /* Cleanup the fragment: */
        Pktlib_freePacket(ptrNetfpClient->cfg.pktlibInstHandle, ptrFragment);
    }

    /* Cleanup any outstanding fragments which have been received on the Inner IP channel: */
    while (1)
    {
        if (Msgcom_getMessage (ptrReassemblyMCB->cfg.innerIpReassemChannel, (MsgCom_Buffer**)&ptrFragment) < 0)
        {
            *errCode = NETFP_EINVAL;
            return -1;
        }
        if (ptrFragment == NULL)
            break;

        /* Cleanup the fragment: */
        Pktlib_freePacket(ptrNetfpClient->cfg.pktlibInstHandle, ptrFragment);
    }

    /* Reassembly block has been shutdown: Shutdown and flush all the used contexts. */
    ptrIPReassemblyContext = (Netfp_IPReassemblyContext*)Netfp_listGetHead((Netfp_ListNode**)&ptrReassemblyMCB->ptrUsedContextList);
    while (ptrIPReassemblyContext != NULL)
    {
        ptrFragment = ptrIPReassemblyContext->ptrPkt;

        /* Free the reassembly context: This will also move it to the free list. */
        Netfp_freeReassemblyContext (ptrNetfpClient, ptrReassemblyMCB, ptrIPReassemblyContext, 1);

        /* Cleanup the fragment from the context */
        Pktlib_freePacket(ptrNetfpClient->cfg.pktlibInstHandle, ptrFragment);

        /* Get the next IP reassembly context */
        ptrIPReassemblyContext = (Netfp_IPReassemblyContext*)Netfp_listGetHead((Netfp_ListNode**)&ptrReassemblyMCB->ptrUsedContextList);
    }

    /* Cycle through the free list and cleanup memory for all the contexts: */
    ptrIPReassemblyContext = (Netfp_IPReassemblyContext*)Netfp_listRemove((Netfp_ListNode**)&ptrReassemblyMCB->ptrFreeContextList);
    while (ptrIPReassemblyContext != NULL)
    {
        /* Add the reassembly context back to the free list. */
        ptrNetfpClient->cfg.free (ptrIPReassemblyContext, sizeof(Netfp_IPReassemblyContext));

        /* Get the next IP reassembly context */
        ptrIPReassemblyContext = (Netfp_IPReassemblyContext*)Netfp_listRemove((Netfp_ListNode**)&ptrReassemblyMCB->ptrFreeContextList);
    }

    /* Delete the reassembly heap: */
    if (Pktlib_deleteHeap (ptrNetfpClient->cfg.pktlibInstHandle, ptrReassemblyMCB->reassemblyHeapHandle, errCode) < 0)
    {
        /* Error: Packet Reassembly heap deletion failed. This could happen if there are fragmented packets which have
         * been received and are already present in the MSGCOM channels. These packets need to be cleaned up by the
         * application and only then can the heap be deleted. */
        if (*errCode == PKTLIB_EDATABUFFER_MISMATCH)
        {
            /* Error: Not ready to be deleted */
            *errCode = NETFP_ENOTREADY;
            return -1;
        }
        return -1;
    }

    /* Delete the reassembly flow: */
    if (Netfp_deleteFlow (clientHandle, "__NETFP-Client-Reassembly-Flow", errCode) < 0)
        return -1;

    /* Cleanup the memory for the reassembly management block: */
    if (ptrReassemblyMCB->cfg.sizeReassemblyCfg != 0)
        ptrNetfpClient->cfg.free (ptrReassemblyMCB->cfg.ptrReassemblyMgmtCfg, ptrReassemblyMCB->cfg.sizeReassemblyCfg);

    /* Cleanup memory for the reassembly MCB */
    ptrNetfpClient->cfg.free (ptrReassemblyMCB, sizeof(Netfp_ReassemblyMCB));

    /* Client is no longer registered for reassembly. */
    ptrNetfpClient->ptrReassemblyMCB = NULL;
    return 0;
}
//fzm-->
int32_t Netfp_cleanupAndReregisterPAReassembly
(
    Netfp_ClientHandle clientHandle,
    int32_t* errCode
)
{
    Netfp_IPReassemblyContext* ptrIPReassemblyContext;
    Netfp_ReassemblyMCB* ptrReassemblyMCB;
    paIpReassmConfig_t paInnerIPReassemblyConfig;
    paIpReassmConfig_t paOuterIPReassemblyConfig;
    Netfp_ClientMCB* ptrNetfpClient;
    Ti_Pkt* ptrFragment;
    /* Sanity Check: Ensure that the arguments are valid */
    if (clientHandle == NULL)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Get the NETFP Client: */
    ptrNetfpClient = (Netfp_ClientMCB*)clientHandle;

    /* Get the Reassembly MCB */
    ptrReassemblyMCB = ptrNetfpClient->ptrReassemblyMCB;

    /* Was the client registered for reassembly? */
    if (ptrReassemblyMCB == NULL)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* In order to disable the reassembly block. We need to set the number of traffic flows in the reassembly
    configuration to 0 */
    memset ((void *)&paInnerIPReassemblyConfig, 0, sizeof(paIpReassmConfig_t));
    memset ((void *)&paOuterIPReassemblyConfig, 0, sizeof(paIpReassmConfig_t));

    /* Configure the reassembly block: */
    if (Netfp_configureReassembly (clientHandle, &paInnerIPReassemblyConfig, &paOuterIPReassemblyConfig, errCode) < 0)
        return -1;

    /* Cleanup any outstanding fragments which have been received on the Outer IP channel: */
    while (1)
    {
        if (Msgcom_getMessage (ptrReassemblyMCB->cfg.outerIpReassemChannel, (MsgCom_Buffer**)&ptrFragment) < 0)
        {
            *errCode = NETFP_EINVAL;
            return -1;
        }

        if (ptrFragment == NULL)
            break;

        /* Cleanup the fragment: */
        Pktlib_freePacket(ptrNetfpClient->cfg.pktlibInstHandle, ptrFragment);
    }

    /* Cleanup any outstanding fragments which have been received on the Inner IP channel: */
    while (1)
    {
        if (Msgcom_getMessage (ptrReassemblyMCB->cfg.innerIpReassemChannel, (MsgCom_Buffer**)&ptrFragment) < 0)
        {
            *errCode = NETFP_EINVAL;
            return -1;
        }

        if (ptrFragment == NULL)
            break;
        /* Cleanup the fragment: */
        Pktlib_freePacket(ptrNetfpClient->cfg.pktlibInstHandle, ptrFragment);
    }

    /* Reassembly block has been shutdown: Shutdown and flush all the used contexts. */
    ptrIPReassemblyContext = (Netfp_IPReassemblyContext*)Netfp_listGetHead((Netfp_ListNode**)&ptrReassemblyMCB->ptrUsedContextList);
    while (ptrIPReassemblyContext != NULL)
    {
        ptrFragment = ptrIPReassemblyContext->ptrPkt;

        /* Free the reassembly context: This will also move it to the free list. */
        Netfp_freeReassemblyContext (ptrNetfpClient, ptrReassemblyMCB, ptrIPReassemblyContext, 0); /* Get the next IP reassembly context */
        ptrIPReassemblyContext = (Netfp_IPReassemblyContext*)Netfp_listGetHead((Netfp_ListNode**)&ptrReassemblyMCB->ptrUsedContextList);

        /* Cleanup the fragment from the context */
        Pktlib_freePacket(ptrNetfpClient->cfg.pktlibInstHandle, ptrFragment);
    }

    paInnerIPReassemblyConfig.numTrafficFlow = pa_MAX_IP_REASM_TRAFFIC_FLOWS;
    paInnerIPReassemblyConfig.destFlowId = ptrReassemblyMCB->reassemblyFlowId ;
    paInnerIPReassemblyConfig.destQueue = Msgcom_getInternalMsgQueueInfo(ptrReassemblyMCB->cfg.innerIpReassemChannel);
    paOuterIPReassemblyConfig.numTrafficFlow = pa_MAX_IP_REASM_TRAFFIC_FLOWS;
    paOuterIPReassemblyConfig.destFlowId = ptrReassemblyMCB->reassemblyFlowId ;
    paOuterIPReassemblyConfig.destQueue = Msgcom_getInternalMsgQueueInfo(ptrReassemblyMCB->cfg.outerIpReassemChannel);
    /* Configure the NETFP Reassembly using the service module. */
    return Netfp_configureReassembly (clientHandle, &paInnerIPReassemblyConfig, &paOuterIPReassemblyConfig, errCode);
}
//fzm<--

/**
 *  @b Description
 *  @n
 *      The function is invoked by the NETFP client to get the reassembly statistics.
 *      Reassembly statistics are valid only if the NETFP client has registered to
 *      provide reassembly services.
 *
 *  @param[in]  clientHandle
 *      Handle to the NETFP Client which is responsible for handling reassembly
 *  @param[out]  ptrStats
 *      Pointer to the reassembly statistics populated by the function
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
int32_t Netfp_getReassemblyStats
(
    Netfp_ClientHandle      clientHandle,
    Netfp_ReassemblyStats*  ptrStats,
    int32_t*                errCode
)
{
    Netfp_ClientMCB*   ptrClientMCB;
    Pktlib_HeapStats   heapStats;

    /* Sanity Check: Validate the arguments */
    ptrClientMCB = (Netfp_ClientMCB*)clientHandle;
    if ((ptrClientMCB == NULL) || (ptrStats == NULL))
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Sanity Check: Reassembly statistics are available only for a NETFP client which was
     * registered for reassembly services */
    if (ptrClientMCB->ptrReassemblyMCB == NULL)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Copy the reassembly statistics */
    memcpy ((void*)ptrStats, (void*)&ptrClientMCB->ptrReassemblyMCB->stats, sizeof(Netfp_ReassemblyStats));

    /* Get the reassembly heap statistics */
    Pktlib_getHeapStats (ptrClientMCB->ptrReassemblyMCB->reassemblyHeapHandle, &heapStats);

    /* Setup the starvation counter in the statistics block: */
    ptrClientMCB->ptrReassemblyMCB->stats.reassemblyHeapStarvationCounter = heapStats.dataBufferStarvationCounter;
    return 0;
}

