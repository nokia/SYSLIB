/**
 *   @file  netfp_internal.h
 *
 *   @brief
 *      Internal header file used by the NETFP module. Please do not
 *      directly include this file.
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

#ifndef __NETFP_INTERNAL_H__
#define __NETFP_INTERNAL_H__

/* Standard Include Files */
#include <linux/limits.h>
#include <pthread.h>
#include <semaphore.h>
#include <stdarg.h>

/* SYSLIB Common header file. */
#include <ti/runtime/common/syslib.h>

/* MCSDK Include Files */
#include <ti/drv/pa/nss_if.h>
#include <ti/drv/sa/salld.h>
#include <ti/drv/pa/pa.h>
#include <ti/drv/pa/pasahost.h>

/* SYSLIB Include Files */
#include <ti/runtime/netfp/include/listlib.h>
#include <ti/runtime/josh/josh.h>
#include <ti/runtime/name/name_db.h>
#include <ti/runtime/name/name_proxyClient.h>

#ifndef  likely
#ifdef __GNUC__
    #define likely(x)       __builtin_expect(!!(x), 1)
    #define unlikely(x)     __builtin_expect(!!(x), 0)
#else
    #define likely(x)       (x)
    #define unlikely(x)     (x)
#endif
#endif

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup NETFP_SYMBOL
 @{ */

/**
 * @brief Feature which allows the NETFP ZOMBIE modules to be cured and recoverable
 */
#define TI_NETFP_ZOMBIE_CURE

/**
 * @brief No special consideration given during LUT1-1 configuration
 */
#define NETFP_IPCFG_FLAGS_NONE              0

/**
 * @brief This flag indicates that only SPI value has to be added to the LUT1-1 for
 * ESP header match. Applicable in case of NAT traversal.
 */
#define NETFP_IPCFG_FLAGS_NATT              1

/**
 * @brief   This is used to define the maximum number of meta information blocks
 * which are allocated to each NETFP client. This will control the maximum number
 * of concurrents events which can be sent to a NETFP client without a response
 * received back.
 */
#define NETFP_MAX_EVENT_META_INFO           8

/**
 * @brief Internal Invalid MTU which is assigned to the Fast Path & Outbound Secure
 * channels during initialization.
 */
#define NETFP_INVALID_MTU                   0xFFFFFFFF


/**
 * @brief The default VLAN priority tag value to be used if priority marking packets
 * with VLAN if zero.
 */
#define NETFP_VLAN_PRIORITYTAG_DEFAULT      1

/**
 * @brief Defines if VLAN priority taggin is enabled.
 */

#define NETFP_VLAN_PRIORITYTAG_ENABLED      (1<<3)

/**
 * @brief VLAN priority tag mask (values 0-7).
 */

#define NETFP_VLAN_PRIORITYTAG_MASK         0x7

/**
 * @brief CPPI Tx channel used by SA Air Ciphering
 */
#if (defined(DEVICE_K2H) || defined (DEVICE_K2K))
#define NETFP_CPPI_CIPHER_TX_CHANNEL        7
#elif defined (DEVICE_K2L)
#define NETFP_CPPI_CIPHER_TX_CHANNEL        19
#else
#error "Error: Unsupported Device"
#endif

/**
 * @brief   PA Command Buffer Size which is the maximum size of the command sets
 * which are configured while sending out the packet.
 */
#define NETFP_PA_CMD_BUFFER_SIZE    sizeof(pasahoComChkCrc_t) + 3*sizeof(pasahoNextRoute_t) + \
                                    sizeof(pasahoIpFrag_t) + sizeof(pasahoShortInfo_t)

/**
@}
*/

/** @addtogroup NETFP_INTERNAL_ENUM
 @{ */

/**
 * @brief
 *  NETFP Module Status
 *
 * @details
 *  Generic enumeration which describes the internal status of the
 *  NETFP building blocks.
 */
typedef enum Netfp_Status
{
    /**
     * @brief   NETFP building block is active and operational
     */
    Netfp_Status_ACTIVE   = 0x1,

    /**
     * @brief   NETFP building block is a zombie and the building block
     * is not operational.
     */
    Netfp_Status_ZOMBIE   = 0x2,

    Netfp_Status_STOP     = 0x3 //fzm
}Netfp_Status;

/**
 * @brief
 *  NETFP Client Status
 *
 * @details
 *  NETFP Clients can be in one of the following states
 */
typedef enum Netfp_ClientStatus
{
    /**
     * @brief   The NETFP client is free and is not being used
     */
    Netfp_ClientStatus_FREE        = 0x0,

    /**
     * @brief   The NETFP client has been initialized but has still not been
     * registered with the server.
     */
    Netfp_ClientStatus_INITIALIZED = 0x1,

    /**
     * @brief   The NETFP client has been initialized and registered with the
     * server. NETFP services are not usable till the registeration is complete.
     */
    Netfp_ClientStatus_ACTIVE       = 0x2,

    /**
     * @brief   The NETFP client is INACTIVE but has not yet been deregistered
     * from the NETFP server.
     */
    Netfp_ClientStatus_INACTIVE     = 0x3
}Netfp_ClientStatus;

/**
 * @brief
 *  The enumeration describes the destination type
 *
 * @details
 *  Once the NETCP susbystem matches a packet it can be sent to multiple
 *  destinations as described below.
 */
typedef enum Netfp_DestType
{
    /**
     * @brief  This is the last rule the NETCP should not continue any further
     * lookups.
     */
    Netfp_DestType_LAST_RULE     = 0x1,

    /**
     * @brief  This is linked to another rule and the NETCP classification should
     * continue until there is another match.
     */
    Netfp_DestType_LINKED_RULE  = 0x2
}Netfp_DestType;

/**
 * @brief
 *  Event Enumeration
 *
 * @details
 *  The NETFP module generates events which are passed between the NETFP server
 *  and clients because there was a networking configuration change. The changes
 *  are propogated through the system and in certain cases it might affect the
 *  layer4 endpoint (socket/3gpp channel) and might render these endpoints unusable.
 *  For example: The interface has been deleted.  In other cases the events are used
 *  to propogate configuration changes which dont have any impact on the layer4
 *  endpoint. For example: The MTU of the interface is changed.
 */
typedef enum Netfp_EventId
{
    /**
     * @brief   Fast Path Deletion Event: This event is triggerred when a NETFP
     * client deletes a fast path through the NETFP exported API. Fast paths are shared
     * across multiple NETFP clients and so if the fast path gets deleted from a NETFP
     * clients the event is propogated to all the other NETFP clients which in turn
     * would ensure that none of the NETFP sockets are using this fast path. The NETFP
     * module will clean out all the the infected entities.
     */
    Netfp_EventId_DELETE_FP         = 0x1,

    /**
     * @brief   Interface Update event: NETFP Socket/3GPP channels are connected to use
     * an interface. The event is triggered to indicate that the properties of the interface
     * have changed but it does NOT impact the operations of the NETFP Sockets/3GPP channels.
     * Example: The interface has been bought up/down through an administrative command.
     */
    Netfp_EventId_UPDATE_INTERFACE,

    /**
     * @brief   Fast Path Update event: The fast path has been changed but the change does not cause
     * disruption of the layer4 services. Example: The next hop MAC address has changed.
     */
    Netfp_EventId_UPDATE_FP,

    Netfp_EventId_ADD_SW_OFFLOAD_SA, //fzm

    Netfp_EventId_DEL_SW_OFFLOAD_SA //fzm
}Netfp_EventId;

/**
 * @brief
 *  Interface event enumeration
 *
 * @details
 *  The NETFP Interface module generates events because of multiple reasons. The
 *  enumeration describes these reasons.
 */
typedef enum Netfp_EventIfReason
{
    /**
     * @brief   Interface status has been modified
     */
    Netfp_EventIfReason_STATUS = 0x1,

    /**
     * @brief   Inner to Outer DSCP map has been modified
     */
    Netfp_EventIfReason_INNER_OUTER_DSCP,

    /**
     * @brief   VLAN Priority map has been modified
     */
    Netfp_EventIfReason_VLAN_PBIT,

    /**
     * @brief   L3 QOS configuration has been modified
     */
    Netfp_EventIfReason_L3_QOS
}Netfp_EventIfReason;

/**
 * @brief
 *  Security Channel Types
 *
 * @details
 *  The NETFP module allows the creation of security channels which are of
 *  the following types.
 */
typedef enum Netfp_SecurityChannelType
{
    /**
     * @brief   F8 [Ciphering] channel
     */
    Netfp_SecurityChannelType_AIR_F8        = 0x1,

    /**
     * @brief   F9 [Integrity Protection] channel
     */
    Netfp_SecurityChannelType_AIR_F9        = 0x2,

    /**
     * @brief   IPSEC channel
     */
    Netfp_SecurityChannelType_IPSEC         = 0x3
}Netfp_SecurityChannelType;

/**
 * @brief
 *  NETFP Channel state
 *
 * @details
 *  Indicates the LTE Channel state.
 */
typedef enum Netfp_ChannelState
{
    /**
     * @brief   LTE Channel normal operational in progress.
     */
    Netfp_ChannelState_NORMAL          = 0x0,

    /**
     * @brief   LTE Channel is suspended.
     */
    Netfp_ChannelState_SUSPEND         = 0x1,

    /**
     * @brief   LTE Channel is reconfigured.
     */
    Netfp_ChannelState_RECONFIG        = 0x2,

    /**
     * @brief   LTE Channel is resumed.
     */
    Netfp_ChannelState_RESUME          = 0x3,

    /**
     * @brief   LTE Channel HandOver is initiated. This is valid for source or Target eNB.
     */
    Netfp_ChannelState_HO_INITIATED    = 0x4,

    /**
     * @brief   LTE Channel HandOver in progress. This is valid for Target eNB.
     */
    Netfp_ChannelState_HO_INPROGRESS   = 0x5,

    /**
     * @brief   LTE Channel HandOver complete. This is valid for Target eNB.
     */
    Netfp_ChannelState_HO_COMPLETE     = 0x6
}Netfp_ChannelState;

/**
 * @brief
 *  PA TX CMD Sequence
 *
 * @details
 *  This enum defines the PA TX CMD sequence for sends Egress packets.
 */
typedef enum Netfp_PATxCmdSequence
{
    /**
     * @brief   UDP CHECKSUM Offload command, including
     * the routing command to PA or SA.
     */
    Netfp_PATxCmdSequence_UDPChksum = 0,

    /**
     * @brief   SA Payload command
     */
    Netfp_PATxCmdSequence_SAPayload,

    /**
     * @brief   Outer IP Fragmentation command
     */
    Netfp_PATxCmdSequence_IPFrag,

    /**
     * @brief   L3Qos Routing command
     */
    Netfp_PATxCmdSequence_RouteL3Qos,

    /**
     * @brief   EMAC Routing command
     */
    Netfp_PATxCmdSequence_RouteEMAC,

    /**
     * @brief   Frame Protocol CRC Offload command.
     */
    Netfp_PATxCmdSequence_FrameProtoCrc,

    /**
     * @brief   MAX number of command allowed,
     * It is (last_command + 1) to account for the routing command
     * for UDP checksum.
     */
    Netfp_PATxCmdSequence_MAX
}Netfp_PATxCmdSequence;

/**
@}
*/

/** @addtogroup NETFP_INTERNAL_DATA_STRUCTURE
 @{ */

/**
 * @brief
 *  This is an opaque handle to the layer3 (IP) entry configured in the NETCP subsystem.
 */
typedef void*           Netfp_L3Handle;

/**
 * @brief
 *  This is an opaque handle to the server security channel.
 */
typedef void*           Netfp_SrvSecChannelHandle;

/**
 * @brief
 *  This is an opaque handle to the L4 Binding Information.
 */
typedef void*           Netfp_L4BindingHandle;

/**
 * @brief
 *  Internal Flow Information.
 *
 * @details
 *  This is an internal data structure which keeps track of all the named flows
 *  which have been created in the system.
 */
typedef struct Netfp_FlowInfo
{
    /**
     * @brief   Links to other flows
     */
    Netfp_ListNode  links;

    /**
     * @brief   Name of the flow.
     */
    char            name[NETFP_MAX_CHAR + 1];

    /**
     * @brief   CPPI Flow Identifier which has been created
     */
    uint32_t        cppiFlowId;

    /**
     * @brief   CPPI Flow handle
     */
    Cppi_FlowHnd    flowHnd;

    /**
     * @brief   Translated receive flow configuration.
     */
    Cppi_RxFlowCfg  rxFlowCfg;
}Netfp_FlowInfo;

/**
 * @brief
 *  Interface IP block
 *
 * @details
 *  Each networking interface in the NETFP module can have multiple IPv6
 *  addresses and a single IPv4 address. The structure is used to encapsulate
 *  all this information.
 */
typedef struct Netfp_InterfaceIPInfo
{
    /**
     * @brief   IP address & Subnet mask
     */
    Netfp_InterfaceIP       interfaceIP;

    /**
     * @brief   Flag to indicate the info is valid.
     */
    uint8_t                 bIsValid;
}Netfp_InterfaceIPInfo;

/**
 * @brief
 *  NETFP Physical inteface
 *
 * @details
 *  The NETFP Physical interface are created and registered with the NETFP
 *  server during initialization. These interfaces are registered with the
 *  NETFP master
 */
typedef struct Netfp_PhyInterface
{
    /**
     * @brief   Links to other physical interfaces
     */
    Netfp_ListNode  links;

    /**
     * @brief   Physical interface name
     */
    char            ifName[NETFP_MAX_CHAR + 1];

    /**
     * @brief   Switch port number associated with the physical interface
     */
    uint32_t        switchPortNum;

    /**
     * @brief   Each physical interface has a marking map which maps the inner
     * DSCP to the outer DSCP.
     */
    uint8_t         innerToOuterDSCPMap[64];
}Netfp_PhyInterface;

/**
 * @brief
 *  NETFP Interface
 *
 * @details
 *  The NETFP server keeps track of a list of networking interfaces which
 *  are used to send & receive packets.
 */
typedef struct Netfp_Interface
{
    /**
     * @brief   Links to other interfaces
     */
    Netfp_ListNode          links;

    /**
     * @brief   Network Interface Configuration.
     */
    Netfp_InterfaceCfg      cfg;

    /**
     * @brief   Status flag
     */
    uint32_t                status;

    /**
     * @brief   Index of the interface
     */
    uint32_t                index;

    /**
     * @brief   Size of the Layer2 Header associated with the interface.
     */
    uint32_t                headerSize;

    /**
     * @brief   Each interface in the subsystem can have have multiple
     * IPv4 address. Linux uses interface aliases to acheive the same
     */
    Netfp_InterfaceIPInfo   ipv4[NETFP_MAX_IP_ADDRESS];

    /**
     * @brief   Each interface in the subsystem can have multiple IPv6
     * addresses at the same time.
     */
    Netfp_InterfaceIPInfo   ipv6[NETFP_MAX_IP_ADDRESS];

    /**
     * @brief
     *  It is possible to instantiate an L3 shaper on each interface. Packets marked with
     *  the DSCP are shaped by sending them out to the specific QOS queue.
     */
    Netfp_L3QoSCfg          l3QosCfg;

    /**
     * @brief   Each interface in the subsystem is linked to a physical interface.
     */
    Netfp_PhyInterface*     ptrPhyInterface;

    /**
     * @brief   Pointer to the NETFP Server
     */
    struct Netfp_ServerMCB* ptrNetfpServer;
}Netfp_Interface;

/**
 * @brief
 *  Security Policy Information
 *
 * @details
 *  This is an internal data structure which keeps track of the security policies
 */
typedef struct Netfp_SPInfo
{
    /**
     * @brief   Links to other security policies
     */
    Netfp_ListNode      links;

    /**
     * @brief   Security Policy status:
     */
    Netfp_Status        status;

    /**
     * @brief   Configuration associated with the security policy.
     */
    Netfp_SPCfg         spCfg;

    /**
     * @brief   IPSEC ESP header length.
     */
    uint32_t            ipsecHdrLen;

    /**
     * @brief   IPSEC ESP trailer length without padding.
     */
    uint32_t            ipsecTrlLen;
}Netfp_SPInfo;

/**
 * @brief
 *  NETCP Destination Information
 *
 * @details
 *  The NETCP subsystem will match the properties of a received packet depending
 *  upon the configured rules. Once a match is found the packet is placed into
 *  a specific destination. The structures specifies this information.
 */
typedef struct Netfp_DstInfo
{
    /**
     * @brief   The packet received by the NETCP subsystem can either be forward to
     * the HOST or to SA Subsystem for further handling.
     */
    Netfp_DestType      dstType;

    /**
     * @brief  This is the handle to the Queue in which the packet will be placed if the
     * NETCP susbsystem is configured to pass the packets to the host.
     */
    Qmss_QueueHnd       channel;

    /**
     * @brief  NETCP Flow ID used to receive Inbound packet
     */
    uint32_t            flowId;
}Netfp_DstInfo;

/**
 * @brief
 *  NETCP Layer3 Information
 *
 * @details
 *  The NETCP subsystem will match the properties of the received packet
 *  in the LUT1-x subsystem
 */
typedef struct Netfp_IPCfg
{
    /**
     * @brief   Source IP Address
     */
    Netfp_IPAddr    srcIP;

    /**
     * @brief   Destination IP Address
     */
    Netfp_IPAddr    dstIP;

    /**
     * @brief   Type of Service.
     */
    uint8_t         tos;

    /**
     * @brief   IP Protocol (IPv4) / Next Header (IPv6)
     */
    uint8_t         protocol;

    /**
     * @brief   SPI value. Valid only for IPSEC tunnels
     */
    uint32_t        spi;

    /**
     * @brief   SPI mode. Used for Fast Path creation.
     */
    Netfp_SPIDMode  spidMode;
}Netfp_IPCfg;

//<fzm>
typedef enum Netfp_LUTType
{
    Netfp_LUTType_DEFAULT   = 0,
    Netfp_LUTType_SW_CHANNEL
}Netfp_LUTType;
//</fzm>

/**
 * @brief
 *  Internal LUT information .
 *
 * @details
 *  This is an internal data structure which keeps track of LUT1 information.
 */
typedef struct Netfp_LUTInfo
{
    /**
     * @brief   Links to other cascading entry
     */
    Netfp_ListNode      links;

    /**
     * @brief   Fast Path Lut information
     */
    paLUT1Info_t        lutInfo;

    /**
     * @brief LUT resource name used to interact with RM to allocate/free LUT1 index
     */
    char                rmName[NETFP_MAX_CHAR];

    /**
     *  NETCP Layer3 Information
     */
    Netfp_IPCfg         ptrIPCfg;

    /**
     *  NETCP IP LUT previous link
     */
    paLnkHandle_t       prevLinkHandle;

    /**
     *  NETCP IP LUT next link
     */
    paLnkHandle_t       nextLinkHandle;

    /**
     * @brief   NETFP Layer3 IP Handle associated with the outer IP.
     */
    Netfp_L3Handle      ipHandle;

    /**
     * @brief   Reference Counter
     */
    uint32_t            refCount;

    /**
     * @brief   LUT entry match count
     */
    uint16_t            matchPktCntIndex;

    Netfp_LUTType       lutType; //fzm
}Netfp_LUTInfo;

/**
 * @brief
 *  Inbound Fast Path Information
 *
 * @details
 *  The data structure represents the inbound fast path runtime information.
 */
typedef struct Netfp_InboundFastPath
{
    /**
     * @brief   Links to other fast paths
     */
    Netfp_ListNode                  links;

    /**
     * @brief   Configuration associated with the fast path.
     */
    Netfp_InboundFPCfg              cfg;

    /**
     * @brief   Direction always set to INBOUND.
     */
    Netfp_Direction                 direction;

    /**
     * @brief   Fast Path status:
     */
    Netfp_Status                    status;

    /**
     * @brief   Flag which indicates that the Fast Path is a parent and has programmed
     * the LUT entry. If the flag is set to 0 it indicates that the fast path is a child
     * and reuses the LUT entry which was programmed by the parent. This is valid only
     * for INGRESS Fast paths.
     */
    uint32_t                        isParent;

    /**
     * @brief   Flag which indicates that the fast path is secure (IPSEC)
     */
    uint32_t                        isSecure;

    /**
     * @brief   NETFP security policy handle associated with this fast path.
     */
    Netfp_SPInfo*                   ptrSPInfo;

    /**
     * @brief   NETFP IPSEC channel associated with the fast path. This could be NULL
     * for non-secure fast paths.
     */
    struct Netfp_IPSecChannel*      ptrIPSecChannel;

    /**
     * @brief   Parent Ingress Fast Path Information. Parents for ingress fast paths
     * will program the LUT entry while children inherit this.
     */
    struct Netfp_InboundFastPath*   ptrParentIngressFPInfo;

   /**
    * @brief   Fast Path LUT information
    */
    Netfp_LUTInfo*                  ptrFpLutInfo;
}Netfp_InboundFastPath;

/**
 * @brief
 *  Outbound Fast Path Information
 *
 * @details
 *  The data structure represents the outbound fast path runtime information.
 */
typedef struct Netfp_OutboundFastPath
{
    /**
     * @brief   Links to other fast paths
     */
    Netfp_ListNode                  links;

    /**
     * @brief   Configuration associated with the fast path.
     */
    Netfp_OutboundFPCfg             cfg;

    /**
     * @brief   Direction always set to OUTBOUND.
     */
    Netfp_Direction                 direction;

    /**
     * @brief   Fast Path status: Fast paths are marked as a ZOMBIE and cannot be
     * used if the next hop MAC address has not been resolved
     */
    Netfp_Status                    status;

    /**
     * @brief   PMTU associated with the fast path
     */
    uint32_t                        fastPathPMTU;

    /**
     * @brief   Age of the PMTU:
     */
    int32_t                         agePMTU;

    /**
     * @brief   Flag which indicates that the fast path is secure (IPSEC)
     */
    uint32_t                        isSecure;

    /**
     * @brief   NETFP security policy handle associated with this fast path.
     */
    Netfp_SPInfo*                   ptrSPInfo;

    /**
     * @brief   Interface to be used to send out the packet.
     */
    Netfp_Interface*                ptrNetfpInterface;

    /**
     * @brief   Request identifier to track if the next hop MAC address has been resolved
     * by the NETFP proxy.
     */
    uint32_t                        requestId;

    /**
     * @brief   Next HOP MAC address to be used. This is populated as a part of
     * the route resolution process.
     */
    uint8_t                         nextHopMACAddress[6];
}Netfp_OutboundFastPath;

/**
 * @brief
 *  Route Recomputation Node
 *
 * @details
 *  The data structure is used to carry the route recomputation information. This
 *  node carries transient information which is used only when a routing table has
 *  been flushed.
 */
typedef struct Netfp_RecomputeRoute
{
    /**
     * @brief   Links to other route recalculation nodes
     */
    Netfp_ListNode          links;

    /**
     * @brief  Destination IP address for which the route is to be recomputed
     */
    Netfp_IPAddr            dstIPAddress;

    /**
     * @brief  Source IP address for which the route is to be recomputed
     */
    Netfp_IPAddr            srcIPAddress;

    /**
     * @brief   Pointer to the route resolution identifier
     */
    uint32_t*               ptrResolvedId;
}Netfp_RecomputeRoute;

/**
 * @brief
 *  NETFP Socket Bind information
 *
 * @details
 *  This is the socket binding information which is populated by the NETFP
 *  server; once the socket has been bound on the NETFP client.
 */
typedef struct Netfp_SockBindInfo
{
    /**
     * @brief   Flag which indicates that the LUT entry was programmed and the handle is stored
     * in the l4Handle
     */
    uint8_t                 lutEntryValid;

    /**
     * @brief   PA LLD Layer4 Handle associated with the socket.
     */
    paHandleL4_t            l4Handle;

    /**
     * @brief   Ingress Fast Path handle
     */
    Netfp_InboundFPHandle   ingressFastPathHandle;

    /**
     * @brief   Security policy identifier which is being used
     */
    uint32_t                spId;

    /**
     * @brief   Secure Status: Flag which indicates if packets received
     * on the socket are over a secure fast path or not
     */
    uint16_t                isSecure;

    /**
     * @brief   Inner source IP address to be used while sending out packets. The outer
     * IP source address is specified as a part of the connect information since it
     * is derived from the egress SA which is not known during "bind".
     */
    Netfp_IPAddr            innerIPSrc;

    /**
     * @brief   L4 Binding Handle
     */
    Netfp_L4BindingHandle   l4BindingHandle;

    /**
     * @brief   3GPP layer properties of the socket.
     */
    Netfp_Sock3GPPCfg       l3GPPcfg;
}Netfp_SockBindInfo;

/**
 * @brief
 *  NETFP Socket L2 connect information
 *
 * @sa Netfp_populateSocketL2ConnectInfo
 *
 * @details
 *  This is the NETFP Socket L2 connect information which specifies the L2 information
 *  to be used to send out the packet. The L2 connect information is derived from the
 *  interface to be used. With routing changes it is possible that the socket L2 connect
 *  information is changed.
 */
typedef struct Netfp_SockL2ConnectInfo
{
    /**
     * @brief   Interface Handle which is being used by the socket
     */
    Netfp_IfHandle          ifHandle;

    /**
     * @brief   Layer2 Header Size
     */
    uint16_t                l2HeaderSize;

    /**
     * @brief   Switch port information associated with the interface on which the
     * packet is to be transmitted.
     */
    uint32_t                switchPortNum;

    /**
     * @brief   MTU associated with the interface
     */
    uint32_t                mtu;

    /**
     * @brief   Interface status associated. 1 implies that the interface is UP; 0 implies that the
     * interface is DOWN. Packets will be dropped if the interface is marked as down.
     */
    uint32_t                ifStatus;

    /**
     * @brief   VLAN identifier if the interface is VLAN enabled.
     */
    uint32_t                vlanId;

    /**
     * @brief   L3 QOS configuration to be used. This is derived from the egress fast
     * path. L3 QOS shapers are optional and might not be configured.
     */
    Netfp_L3QoSCfg          l3QosCfg;

    /**
     * @brief   IPSec channel flow ID used to pass packets between PA and SA.
     */
    uint32_t                ipsecChanFlowID;

    /**
     * @brief   This field is used only if the socket is connected over an IPSEC tunnel
     * and this configuration specifies information which was used to create the IPSEC
     * tunnel. This information is used during the packet transmission to setup the ESP
     * header and trailer.
     */
    Sa_GenCtrlInfo_t        saGeneralChannelCtrlInfo;

    /**
     * @brief   This field is used only if the socket is connected over an IPSEC tunnel
     * and this contains information which is interpreted by the NETCP subsystem to
     * encrypt the packet. This was initialized and stored during channel creation.
     */
    Sa_SWInfo_t             ipSecTxInfo;

    /**
     * @brief   Sockets will send out data on a specific physical interface. This specifies
     * the mapping of the inner DSCP to the outer DSCP.
     */
    uint8_t                 innerToOuterDSCPMap[64];

    /**
     * @brief   This is mapping which maps the NETFP socket priority to the VLAN priority bits.
     */
    Netfp_VLANPriorityMap   vlanMap;

    /**
     * @brief   Source MAC address to be populated in the packet. This is the MAC address of the
     * interface on which the packet is to be transmitted.
     */
    uint8_t                 srcMacAddress[6];

    /**
     * @brief   Destination MAC address to be populated in the packet.
     */
    uint8_t                 dstMacAddress[6];
}Netfp_SockL2ConnectInfo;

/**
 * @brief
 *  NETFP Socket Connect information
 *
 * @details
 *  This is the socket connect information which is populated by the NETFP
 *  server; once the socket has been connected on the NETFP client. Information
 *  from this structure will be used whenever a packet is to be sent on the
 *  specific socket.
 */
typedef struct Netfp_SockConnectInfo
{
    /**
     * @brief   Egress Fast Path handle which is being used
     */
    Netfp_OutboundFPHandle  egressFastPathHandle;

    /**
     * @brief   Initial socket status calculated during the connect time with the
     * current system configuration. The socket status can change over time due
     * to system events and configuration changes.
     */
    Netfp_Status            initalSocketStatus;

    /**
     * @brief   Security policy identifier which is being used
     */
    uint32_t                spId;

    /**
     * @brief   Number of IP addresses to be added.
     */
    uint32_t                numIP;

    /**
     * @brief   Inner IP destination address to which the packet is being sent
     */
    Netfp_IPAddr            innerIPDst;

    /**
     * @brief   Outer IP destination address to which the packet is being sent
     * There are 2 copies of the connect information. One is actively being used while
     * the other one can be updated by new configuration. Once the configuration change
     * is completed the 'cfgIndex' is switched.
     */
    Netfp_IPAddr            outerIPDst[2]; //fzm

    /**
     * @brief   Outer IP source address which is derived from the egress
     * security association.
     * There are 2 copies of the connect information. One is actively being used while
     * the other one can be updated by new configuration. Once the configuration change
     * is completed the 'cfgIndex' is switched.
     */
    Netfp_IPAddr            outerIPSrc[2]; //fzm

    /**
     * @brief   Secure Status: Flag which indicates if the packets being sent over
     * the socket are over an IPSEC tunnel or not.
     */
    uint16_t                isSecure;

    /**
     * @brief   Specifies NAT-T configuration.
     */
    Netfp_nattEncapCfg      nattEncapCfg;

    /**
     * @brief   Specifies ESP header margin for secure packets.
     */
    uint32_t                pktHeaderMargin;

    /**
     * @brief   Specifies ESP trailer margin for secure packets.
     */
    uint32_t                pktTrailerMargin;

    /**
     * @brief   Specifies ESP padding margin for secure packets.
     */
    uint32_t                pktPaddingMargin;

    /**
     * @brief   Fast paths specify a DSCP mapping which maps the NETFP socket priority
     * to a specific DSCP. This mapping is used to mark the inner DSCP value. VLAN
     * priority bits (if applicable) are always marked using the physical interface
     * mappings
     */
    uint8_t                 fpDSCPMapping[NETFP_MAX_SOCK_PRIORITY];

    /**
     * @brief   Configuration index which indicates the active L2 configuration block.
     */
    uint32_t                cfgIndex;

    /**
     * @brief   L2 infomation required by the socket which is used to send out packets
     * There are 2 copies of the connect information. One is actively being used while
     * the other one can be updated by new configuration. Once the configuration change
     * is completed the 'cfgIndex' is switched.
     */
    Netfp_SockL2ConnectInfo l2Info[2];
}Netfp_SockConnectInfo;

/**
     * @brief   PA commands supporting optimized version of NetfpSend
     * To be used for IPV4 non-secured packets without fragmentation
*/
typedef struct PaCommandSet
{
    uint32_t                    cmdBuffer[NETFP_PA_CMD_BUFFER_SIZE];
    uint16_t                    cmdBufferSize;
}PaCommandSet;

/**
     * @brief   The buffer to be filled with network headers for optimized version of NetfpSend
     * To be used for IPV4 non-secured packets without fragmentation
*/
typedef struct NetfpNetHeaders
{
    uint8_t                            headerBuff[14 + 4 + 40 + 8 /*ETHHDR_SIZE + VLANHDR_SIZE + IPv6HDR_SIZE + UDPHDR_SIZE*/];
    uint32_t                           l2HeaderSize;
    uint32_t                           headerSize;
    uint32_t                           partialChkSum;
}NetfpNetHeaders;

/**
 * @brief
 *  NETFP Sockets
 *
 * @details
 *  The NETFP sockets are endpoints which can be used to send & receive packets
 */
typedef struct Netfp_Socket
{
    /**
     * @brief   Links to other sockets.
     */
    Netfp_ListNode                      links;

    /**
     * @brief   Family associated with the socket.
     */
    Netfp_SockFamily                    family;

    /**
     * @brief   Pointer to the NETFP client which has opened the socket.
     */
    struct Netfp_ClientMCB*             ptrNetfpClient;

    /**
     * @brief   Layer4 local properties associated with the socket
     */
    Netfp_SockAddr                      localSockAddr;

    /**
     * @brief   Layer4 peer properties associated with the socket
     */
    Netfp_SockAddr                      peerSockAddr;

    /**
     * @brief   Status flag indicating the status of the socket.
     */
    Netfp_Status                        status;

    /**
     * @brief   Socket extended statistics
     */
    Netfp_ExtendedSocketStats           extendedStats;

    /**
     * @brief   State of the socket: BOUND or CONNECTED or both
     */
    uint8_t                             sockState;

    /**
     * @brief   Dont Fragment Bit in the IPv4 Header.
     */
    uint8_t                             dontFrag;

    /**
     * @brief   Flag which indicates if Frame Protocol CRC offload is enabled or not.
     */
    uint8_t                             frameProtoCrcOffload;

    /**
     * @brief   Flag which indicates if UDP checksum offload is enabled or not.
     */
    uint8_t                             udpChksumOffload;

    /**
     * @brief  Priority of the socket: This is used to mark the DSCP in the IP header
     * for each packet which is sent out
     */
    uint8_t                             priority;

    /**
     * @brief  Priority of the socket w/o VLAN: This is used to set the priority bits in the VLAN header
     * for each packet which is sent out on VLAN id of zero (aka priority tagging). Priority is input as
     * 0 to 7 via setsockopt().
     */
    uint8_t                             priorityTag;

    /**
     * @brief  Bind information: This specifies the local networking properties associated
     * with the socket. These properties are used in order to receive data from the core
     * network
     */
    Netfp_SockBindInfo                  bindInfo;

    /**
     * @brief  Connect information: This specifies the remote networking properties of the peer
     * to which a socket will be sending data. These properties are used while sending data
     * to the core network.
     */
    Netfp_SockConnectInfo               connectInfo;

    /**
     * @brief   Application registered POST_ROUTING Hook
     */
    Netfp_HookFunction                  postRoutingHook;

    /**
     * @brief   Application registered POST_ROUTING argument
     */
    uint32_t                            postRoutingHookArg;

    /**
     * @brief  Sockets can be utilized by the LTE 3GPP layer. This is the pointer to the LTE 3GPP
     * security channel which is utilizing the socket. This field is NULL for all other use cases.
     */
    struct Netfp_3GPPSecurityChannel*   ptr3GPPSecurityChannel;

    /**
     * @brief Structures supporting optimized version of NetfpSend
     * To be used for IPV4 non-secured packets without fragmentation
     */
    NetfpNetHeaders                    netHeaders[2];
    PaCommandSet                       paCommands[2];
}Netfp_Socket;

/**
 * @brief
 *  IP Reassembly Context
 *
 * @details
 *  This structure holds the reassembly information for each packet under construction.
 */
typedef struct Netfp_IPReassemblyContext
{
    /**
     * @brief   Links to other reassembly contexts
     */
    Netfp_ListNode                  links;

    /**
     * @brief   Pointer to the reassembly MCB to which the context belongs
     */
    struct Netfp_ReassemblyMCB*     ptrReassemblyMCB;

    /**
     * @brief   IP Protocol (IPv4) / Next Header (IPv6).
     */
    uint8_t                         protocol;

    /**
     * @brief   Unique identifier for fragments of a given packet.
     */
    uint32_t                        id;

    /**
     * @brief   Source IP Address
     */
    Netfp_IPAddr                    srcAddr;

    /**
     * @brief   Destination IP Address
     */
    Netfp_IPAddr                    dstAddr;

    /**
     * @brief   NetCP queue to which reassembled packets are sent to.
     */
    Qmss_QueueHnd                   dstQueue;

    /**
     * @brief   Status flag which if set implies that the reassembly context has detected
     * overlapping fragments. This will cause all subsequent fragments matching the context
     * to be dropped
     */
    int32_t                         overlapFrag;

    /**
     * @brief   Status flag which if set implies that the reassembly context has detected
     * corrupted fragments. This will cause all subsequent fragments
     * matching the context to be dropped
     */
    int32_t                         corruptedPkt;

    /**
     * @brief   Traffic Flow Id passed by the NETCP. The traffic flow id will be set to
     * PA_INV_TF_INDEX if there is no more traffic flows available in the NETCP subsystem.
     */
    uint16_t                        tfId;

    /**
     * @brief   Number of fragments detected while reassembling the packet.
     */
    uint16_t                        fragCnt;

    /**
     * @brief   Expected IP Payload length of the full reassembled packet.
     */
    uint16_t                        origIPLength;

    /**
     * @brief   Current IP Payload length of the packet under construction.
     */
    uint16_t                        detectedIPLen;

    /**
     * @brief   Offset to the IP header in the fragment.
     */
    uint16_t                        ipOffset;

    /**
     * @brief   Switch port on which the fragments for the reassembly context were received.
     */
    uint8_t                         inPort;

    /**
     * @brief   VLAN Identifier if any which has been received on the reassembly context
     */
    uint16_t                        vlanId;

    /**
     * @brief   Total packet length of the packet including all headers and trailers
     * i.e L2, OuterIP, ESP, InnerIP, Payload and ESP Trailer
     */
    uint16_t                        totalPacketLen;

    /**
     * @brief   Reassembly context timeout in application specified unit ticks
     */
    int32_t                         timeout;

    /**
     * @brief   Pointer to the first descriptor in this context (packet under construction).
     */
    Ti_Pkt*                         ptrPkt;

    /**
     * @brief   Pointer to last fragment MF=0, offset!=0
     */
    Ti_Pkt*                         ptrLastFrag;
}Netfp_IPReassemblyContext;

/**
 * @brief Meta information used by optimized version of NetfpSend
 * To be used for IPV4 non-secured packets
 */
typedef struct Netfp_SockTxMetaInfo_FZM
{
    Ti_Pkt*                     ptrPayload;
    uint8_t*                    ptrHeaderBuffer;
    Ti_Pkt*                     ptrHeaderPkt;
    uint32_t                    headerSize;
    Netfp_SockL2ConnectInfo*    l2ConnectInfo;
    uint8_t                     outerDSCP;
    uint8_t                     innerDSCP;
    uint8_t                     l2CfgIndex;
    uint8_t                     hwUDPChksum;
    paTxChksum_t                udpHwChkSum;
    uint8_t                     hwFragmentation;
    uint8_t                     priority;
}Netfp_SockTxMetaInfo_FZM;

/**
 * @brief
 *  NETFP Socket Meta Information.
 *
 * @details
 *  The structure encapsulates the meta information which is required
 *  to send a packet over the socket. This information is populated as the packet
 *  traverses the various layers in the NETFP
 */
typedef struct Netfp_SockTxMetaInfo
{
    /**
     * @brief   Header Packet which carries all the standard networking headers
     */
    Ti_Pkt*                     ptrHeaderPkt;

    /**
     * @brief   Pointer to the L2 connect information which is to be used.
     */
    Netfp_SockL2ConnectInfo*    ptrL2ConnectInfo;

    /**
     * @brief   Size of the networking headers (Including L2, L3 and L4)
     */
    uint32_t                    headerSize;

    /**
     * @brief   Size of the trailers if any
     */
    uint32_t                    trailerSize;

    /**
     * @brief   Networking header buffer packet.
     */
    uint8_t*                    ptrHeaderBuffer;

    /**
     * @brief   Data Payload packet which carries the actual packet to be sent
     */
    Ti_Pkt*                     ptrPayload;

    /**
     * @brief   Priority of the packet which is being sent out. This is derived
     * from the NETFP socket priority.
     */
    uint8_t                     priority;

    /**
     * @brief   Priority of the packet which is being sent out with VLAN id zero.
     *  This is derived from the NETFP socket priority tag. Highest bit defines if tagging
     *  is enabled.
     */
    uint8_t                     priorityTag;

    /**
     * @brief   Flag which indicates if Frame Protocol CRC Offload is required.This is set
     * to 1 to allow the PA to compute the CRC. If the packet has already
     * been fragmented by the software the flag is set to 0.
     */
    uint8_t                     frameProtoCrc;

    /**
     * @brief   Frame Protocol Payload offset used for WCDMA Frame Protocol.
     */
    uint8_t                     frameProtoPayloadOffset;

    /**
     * @brief   GTPU Header Size if present. Applicable only if a GTPU packet
     * is being sent out.
     */
    uint8_t                     gtpuHeaderSize;

    /**
     * @brief   Size of the packet being sent out.
     */
    uint32_t                    packetSize;

    /**
     * @brief   Inner DSCP to be used to be configured in the IP packet
     */
    uint8_t                     innerDSCP;

    /**
     * @brief   Outer DSCP to be used to be configured in the IP packet
     */
    uint8_t                     outerDSCP;

    /**
     * @brief   UDP checksum info used for IPv6 hardware UDP checksum calculation
     */
    paTxChksum_t                udpHwChkSum;

    /**
     * @brief   Flag which indicates if PA Fragmentation is required.This is set
     * to 1 to allow the PA to handle the fragmentation. If the packet has already
     * been fragmented by the software the flag is set to 0.
     */
    uint8_t                     hwFragmentation;

    /**
     * @brief   Flag for hardware UDP checksum offload. This is set to 1 to allow
     * PA to handle UDP checksum in hardware.
     */
    uint8_t                     hwUDPChksum;

    /**
     * @brief  Maximum ESP trailer overhead.  Applicable only if a ESP packet
     * is being sent out.
     */
    uint8_t                     innerHeaderSize;  //fzm
}Netfp_SockTxMetaInfo;

/**
 * @brief
 *  Security channel software information
 *
 * @details
 *  The data structure defines the security software information which is populated after
 *  the channel is created. This information is then utililized by the client security
 *  channels to send & receive data without having to use the SA LLD API
 */
typedef struct Netfp_SecuritySwInfo
{
    /**
     * @brief   SA Channel receive Software information which is specific to each security
     * channel and is populated by the SA LLD depending upon the configuration parameters
     */
    Sa_SWInfo_t                 rxInfo;

    /**
     * @brief   SA Channel transmit Software information which is specific to each security
     * channel and is populated by the SA LLD depending upon the configuration parameters
     */
    Sa_SWInfo_t                 txInfo;
}Netfp_SecuritySwInfo;

/**
 * @brief
 *  Security Context
 *
 * @details
 *  This data structure is used to hold information relevant to the security context
 *  which is placed into the NETCP subsystem.
 */
typedef struct Netfp_SecurityContext
{
    /**
     * @brief   Links to other server security channels.
     */
    Netfp_ListNode              links;

    /**
     * @brief   Security Protocol as defined by the NETFP
     */
    Netfp_SaProtocol            netfpProtocol;

    /**
     * @brief   Security Context Identifier
     */
    uint16_t                    secContextId;

    /**
     * @brief   Security context pointer
     */
    uint8_t*                    ptrSecurityContext;

    /**
     * @brief   Size of the security context
     */
    uint32_t                    sizeSecurityContext;
}Netfp_SecurityContext;

/**
 * @brief
 *  Server Security Channel
 *
 * @details
 *  This data structure is a wrapper data structure provided to keep track of the SA LLD
 *  required information. This information is populated during SA channel creation and is
 *  required to bring down the SA channels. The data structure is used to manage both
 *  AIR and IPSEC channels.
 */
typedef struct Netfp_SrvSecurityChannel
{
    /**
     * @brief   Links to other server security channels.
     */
    Netfp_ListNode              links;

    /**
     * @brief   NETFP Server associated with the security channel. Each security channel is
     * created in the context of a particular NETFP Server.
     */
    struct Netfp_ServerMCB*     ptrNetfpServer;

    /**
     * @brief   Security Protocol as defined by the NETFP
     */
    Netfp_SaProtocol            netfpProtocol;

    /**
     * @brief   SA Channel handle returned by the SA LLD.
     */
    Sa_ChanHandle               saChannelHandle;

    /**
     * @brief   Base address of the memory allocated to create the SA channel
     */
    void*                       bases[sa_N_BUFS];

    /**
     * @brief   Sizes of the memory allocated to create the SA channel
     */
    int32_t                     sizes[sa_N_BUFS];

    /**
     * @brief   Alignment requirements to create the SA channel
     */
    int32_t                     aligns[sa_N_BUFS];

    /**
     * @brief   Each server security channel can have multiple security context; one
     * for the receive and the other for the transmit direction.
     */
    Netfp_SecurityContext       secContext[2];

    /**
     * @brief   This is the general channel control information which is populated while creating
     * the SA LLD channel.
     */
     Sa_GenCtrlInfo_t           saGeneralChannelCtrlInfo;

    /**
     * @brief   Software information which is populated after the channel has been
     * created and which is stored
     */
    Netfp_SecuritySwInfo        swInfo;

    /**
     * @brief   Timestamp to determine when this security channel should be moved from
     * slow cleanup list to the garbage collection
     */
    uint64_t slowCleanupTimestamp;
}Netfp_SrvSecurityChannel;

/**
 * @brief
 *  IPSEC Security Channel Virtual Link
 *
 * @details
  *  This data structure describes the IPSEC channel virtual link information.
 */
typedef struct Netfp_PAVirtLink
{
    /**
     * @brief   Virtual link handle returned by PA. This handle is used to link the inner
     * and outer LUT rules. This is also valid only for INBOUND IPSEC channels.
     */
    paLnkHandle_t               vlinkHandle;

    /**
     * @brief   Virtual link is shared acrose SAs created by rekeying. This count keeps
     * track of the number of SAs referenced the vlinkHandle.
     */
    uint32_t                    refCount;
}Netfp_PAVirtLink;

/**
 * @brief
 *  IPSEC NAT-T IP channel Info .
 *
 * @details
 *  The structure contains NetCP NAT-T IP channel Info.
 */
typedef struct Netfp_NattIPInfo
{
    /**
     * @brief   Links to other cascading entry
     */
    Netfp_ListNode      links;

    /**
     * @brief   IP Configuration for the NAT-T channel.
     */
    Netfp_IPCfg         ipCfg;

    /**
     * @brief   NAT-T IP channel Lut information
     */
    Netfp_LUTInfo*      nattLutInfo;

    /**
     * @brief   Reference Counter
     */
    uint32_t            refCount;
}Netfp_NattIPInfo;

//<fzm>

typedef struct Netfp_SwIPInfo
{
    Netfp_ListNode      links;
    Netfp_IPCfg         ipCfg;
    Netfp_LUTInfo*      lutInfo;
    uint32_t            refCount;
}Netfp_SwIPInfo;

typedef struct Netfp_NattSwIPInfo
{
    Netfp_ListNode      links;
    Netfp_IPCfg         ipCfg;
    Netfp_LUTInfo*      nattLutInfo;
    uint32_t            refCount;
}Netfp_NattSwIPInfo;

//</fzm>

/**
 * @brief
 *  IPSEC Security Channel
 *
 * @details
 *  This data structure describes the IPSEC channel. IPSEC channels are associated with
 *  the security association. IPSEC channels only exist in the context of the NETFP server
 *  to the NETFP clients these are opaque handles and have no meaning.
 */
typedef struct Netfp_IPSecChannel
{
    /**
     * @brief   Links to other IPSEC security channels.
     */
    Netfp_ListNode              links;

    /**
     * @brief   Security association configuration
     */
    Netfp_SACfg                 saCfg;

    /**
     * @brief   IPSEC channel status: IPSEC outbound channels are marked as a ZOMBIE
     * and cannot be used if the next hop MAC address has not been resolved
     */
    Netfp_Status                status;

    /**
     * @brief   PMTU associated with the outbound IPSEC channel
     */
    uint32_t                    ipsecPMTU;

    /**
     * @brief   Age of the PMTU:
     */
    int32_t                     agePMTU;

    /**
     * @brief   Pointer to the server security channel. Since IPSEC channels also exist
     * only in the context of the NETFP server. This is not treated as an *opaque* handle
     */
    Netfp_SrvSecurityChannel*   ptrSrvSecurityChannel;

    /**
     * @brief   Virtual link handle returned by PA. This handle is used to link the inner
     * and outer LUT rules. This is also valid only for INBOUND IPSEC channels.
     */
    Netfp_PAVirtLink*           ptrNetfpPAVlink;

    /**
     * @brief   IPSec channel Lut information
     */
    Netfp_LUTInfo*               ptrSaLutInfo;

    /**
     * @brief   NAT-T Lut information
     */
    Netfp_NattIPInfo*           ptrNattIPInfo;

    Netfp_SwIPInfo*             ptrSwIPInfo; //fzm

    Netfp_NattSwIPInfo*         ptrNattSwIPInfo; //fzm

    /**
     * @brief   Interface to be used to send out the secure packets.
     */
    Netfp_Interface*            ptrNetfpInterface;

    /**
     * @brief   Route resolution identifier to track if the next hop MAC address has been
     * resolved by the NETFP proxy.
     */
    uint32_t                    requestId;

    /**
     * @brief   Next HOP MAC address to be used. This is populated as a part of
     * the route resolution process.
     */
    uint8_t                     nextHopMACAddress[6];

    /**
     * @brief indicates that server tried to do recomputation of stopped SA
     * which means that its route info doesn't exist in proxy (routes were flushed),
     * so we don't have to notify proxy on later SA deletion (that would delete route
     * not associated with the SA)
     */
    uint32_t                    recomputationDoneOnStopped;
}Netfp_IPSecChannel;

/**
 * @brief
 * Configuration to add an IP entry in Lut table
 *
 * @details
 *  The data structure represents the configuration needed to program
 *an Inbound IP LUT entry.
 */
typedef struct Netfp_IPLutCfg
{
    /**
     *  NETCP Layer 3 Configuration
     */
    Netfp_IPCfg         ipCfg;

    /**
     *  Multicast mode configuration
     */
    Netfp_FastpathMode  fastpathMode;

    /**
     *  NETCP IP LUT previous link
     */
    paLnkHandle_t       prevLinkHandle;

    /**
     *  NETCP IP LUT previous link
     */
    paLnkHandle_t       nextLinkHandle;

    /**
     *  Associate IPSec channel with the IP Lut entry
     */
    Netfp_IPSecChannel* ptrIPSecChannel;

    /**
     *  NETCP Destination Info used to deliver the inbound packet
     */
    Netfp_DstInfo       dstInfo;

    /**
     *  Flags used to setup IP Lut entry
     */
    uint8_t             flags;

    /**
     *  Extra pCmd attach to the LUT entry
     */
    paCmdInfo_t         pCmd;
}Netfp_IPLutCfg;

/**
 * @brief
 *  3GPP Security channel
 *
 * @details
 *  The structure describes the 3GPP security channel which are created for LTE (Air ciphering)
 *  on the NETFP clients. Each of these channels is associated with a security channel on the
 *  server. This structure holds information which is relevant only to the client. 3GPP security
 *  channels define both SRB as well as DRB's.
 */
typedef struct Netfp_3GPPSecurityChannel
{
    /**
     * @brief   Links to other 3GPP security channels.
     */
    Netfp_ListNode              links;

    /**
     * @brief   NETFP client associated with the security channel. Client security
     * channels are valid only on a specific client.
     */
    struct Netfp_ClientMCB*     ptrNetfpClient;

    /**
     * @brief   Pointer to the NETFP user to which the radio bearer belongs
     */
    struct Netfp_User*          ptrNetfpUser;

    /**
     * @brief   Radio bearer identifier associated with the security channel.
     */
    uint8_t                     rbId;

    /**
     * @brief   Binding configuration which was specified while creating the LTE channel
     * These are valid only for DRB.
     */
    Netfp_LTEChannelBindCfg     bindCfg;

    /**
     * @brief   Connect configuration which was specified while creating the LTE channel
     * These are valid only for DRB.
     */
    Netfp_LTEChannelConnectCfg  connectCfg;

    /**
     * @brief   3GPP security channels associated with a data radio bearer are also associated
     * with a socket which allows communication with the core network. For SRB this field is
     * set to NULL.
     */
    Netfp_Socket*               ptrNetfpSocket;

    /**
     * @brief   This is the handle to the server security channel. This is stored
     * in the NETFP client channel block as an opaque handle and should *NOT* be decoded.
     */
    Netfp_SrvSecChannelHandle   srvSecurityChannelHandle;

    /**
     * @brief   Software information which is populated after the channel has been
     * created and which is stored from the server security channel into the client
     * channel. This information is then used in the data path.
     */
    Netfp_SecuritySwInfo        swInfo;

    /**
     * @brief   This queue is used to buffer the DRB packets during re-establishment.
     */
    Qmss_QueueHnd               reEstDataBufferQueue;

    /**
     * @brief   This queue is used to buffer the DRB packets during HandOver.
     */
    Qmss_QueueHnd               hoDataBufferQueue;

    /**
     * @brief   Value that indentifies the current stream of packets. This value is used to differentiate
     * the current stream of packets from the ones received prior to re-establishment.
     */
    uint32_t                    currentMark;

    /**
     * @brief   Channel state.
     */
    Netfp_ChannelState          state;
}Netfp_3GPPSecurityChannel;

/**
 * @brief
 *  User Block
 *
 * @details
 *  The structure specifies the user block which keeps track of a user which is
 *  connected and registered with the NETFP module.
 */
typedef struct Netfp_User
{
    /**
     * @brief   Links to other security block.
     */
    Netfp_ListNode                  links;

    /**
     * @brief   Status of the user: Active or Zombie
     */
    Netfp_Status                    status;

    /**
     * @brief   User configuration
     */
    Netfp_UserCfg                   cfg;

    /**
     * @brief   Pointer to the NETFP client which has created the user.
     */
    struct Netfp_ClientMCB*         ptrNetfpClient;

    /**
     * @brief   SRB Client security authentication channels: SRB1 and SRB2
     */
    Netfp_3GPPSecurityChannel       SRBAuthSecurityChannel[3];

    /**
     * @brief   SRB Client security authentication channels: SRB1 and SRB2
     */
    Netfp_3GPPSecurityChannel       SRBCipherSecurityChannel[3];

    /**
     * @brief   List of all DRB security Channels added to this user.
     */
    Netfp_3GPPSecurityChannel*      ptrDRBSecurityChannelList;
}Netfp_User;

/**
 * @brief
 *  Air side F8 configuration
 *
 * @details
 *  The structure is used to specify the configuration required to configure
 *  the security channel for F8 i.e. ciphering.
 */
typedef struct Netfp_F8Cfg
{
    /**
     * @brief   User identifier
     */
    uint16_t                    ueId;

    /**
     * @brief   Radio bearer identifier
     */
    uint16_t                    rbId;

    /**
     * @brief   Quality of service class identifier for this radio bearer
     */
    uint16_t                    qci;

    /**
     * @brief   Flag which if set to 1 indicates that the radio bearer is a data radio
     * bearer else this is a signaling radio bearer.
     */
    uint8_t                     isDataRadioBearer;

    /**
     * @brief   Ciphering mode to be configured
     */
    Netfp_3gppCipherMode        cipherMode;

    /**
     * @brief   Encryption key for the SRB
     */
    uint8_t                     hKeyRrcEnc[16];

    /**
     * @brief   Encryption key for the DRB
     */
    uint8_t                     hKeyUpEnc[16];

    /**
     * @brief   Flow identifier which is used to pass packets to the SA for encoding
     */
    uint8_t                     encodeFlowId;

    /**
     * @brief   This is the queue where the packets will be placed by the SA once they
     * have been encoded
     */
    Qmss_QueueHnd               encodeQueue;

    /**
     * @brief   Flow identifier which is used to pass packets to the SA for decoding
     */
    uint8_t                     decodeFlowId;

    /**
     * @brief   This is the queue where the packets will be placed by the SA once they
     * have been decoded
     */
    Qmss_QueueHnd               decodeQueue;

    /**
     * @brief   CountC value associated with the radio bearer
     */
    uint32_t                    countC;

    /**
     * @brief   Value that indentifies the current stream of packets. This value is used to differentiate
     * the current stream of packets from the ones received prior to re-establishment.
     */
    uint32_t                    currentMark;
}Netfp_F8Cfg;

/**
 * @brief
 *  Air side F9 configuration
 *
 * @details
 *  The structure is used to specify the configuration required to configure
 *  the security channel for F9 i.e. integrity protection.
 */
typedef struct Netfp_F9Cfg
{
    /**
     * @brief   User identifier
     */
    uint16_t                ueId;

    /**
     * @brief   Radio bearer identifier
     */
    uint16_t                rbId;

    /**
     * @brief   Quality of service class identifier for this radio bearer
     */
    uint16_t                qci;

    /**
     * @brief   Flag which if set to 1 indicates that the radio bearer is a data radio
     * bearer else this is a signaling radio bearer.
     */
    uint8_t                 isDataRadioBearer;

    /**
     * @brief   Authentication integrity protection key.
     */
    uint8_t                 hKeyRrcInt[16];

    /**
     * @brief   Authentication mode to be configured
     */
    Netfp_3gppAuthMode      authMode;

    /**
     * @brief   This is the downlink destination queue to which the packet is placed after the MAC-I has
     * been added to the packet
     */
    Qmss_QueueHnd           downlinkQueue;

    /**
     * @brief   Flow identifer which is used to pick the packets and pass to the SA so that the
     * MAC-I can be added to the packet.
     */
    uint8_t                 downlinkFlowId;

    /**
     * @brief   This is the uplink destination queue to which the packet is placed after the MAC-I has been
     * removed from the packet.
     */
    Qmss_QueueHnd           uplinkQueue;

    /**
     * @brief   Flow identifer which is used to pick the packets and pass to the SA so that the
     * MAC-I can be removed from the packet
     */
    uint8_t                 uplinkFlowId;

    /**
     * @brief   CountC value associated with the radio bearer
     */
    uint32_t                countC;
}Netfp_F9Cfg;

/**
 * @brief
 *  Security Channel Configuration
 *
 * @details
 *  The function is used to configure the security channel from the
 *  NETFP client.
 */
typedef struct Netfp_SecChannelCfg
{
    /**
     * @brief   Security channel type
     */
    Netfp_SecurityChannelType        type;

    union
    {
        Netfp_F8Cfg                 f8Cfg;
        Netfp_F9Cfg                 f9Cfg;
        Netfp_SACfg                 ipSecCfg;
    }u;
}Netfp_SecChannelCfg;

/**
 * @brief
 *  User information passed between Client and Server when creating a user.
 *
 * @details
 *  The structure contains the information passed between client and server such as
 *  Temporary queues used between ciphering and authentication passed by the client,
 *  Server Security Channel Handle passed back by the server,
 *  Software information populated when creating security channel passed back by the server.
 */
typedef struct Netfp_SRBInfo
{
    /**
     * @brief   Server security channel handle
     */
    Netfp_SrvSecChannelHandle    srvSecurityChannelHandle;

    /**
     * @brief   software information populated when creating security channel
     */
    Netfp_SecuritySwInfo        swInfo;

    /**
     * @brief   Temporary queues used between ciphering and authentication
     */
    Qmss_QueueHnd               destQueueHnd;
}Netfp_SRBInfo;

/**
 * @brief
 *  SRB information passed between Client and Server when creating a user.
 *
 * @details
 *  The structure contains the information passed between client and server such as
 *  temporary queues used between ciphering and authentication passed by the client,
 *  Server Security Channel Handle passed back by the server,
 *  Software information populated when creating security channel passed back by the server.
 */
typedef struct Netfp_UserSRBInfo
{
    /**
     * @brief   User information for SRB authentication
     */
    Netfp_SRBInfo               SRBAuthInfo[3];

    /**
     * @brief   User information for SRB ciphering
     */
    Netfp_SRBInfo               SRBCipherInfo[3];
}Netfp_UserSRBInfo;

/**
 * @brief
 *  NETFP Layer4 Node
 *
 * @details
 *  The node tracks the Layer4 (UDP & GTPU Id) ports which have been bound by the
 *  sockets running on the NETFP Clients.
 */
typedef struct Netfp_Layer4Node
{
    /**
     * @brief   Links to other layer4 nodes
     */
    Netfp_ListNode          links;

    /**
     * @brief   Binding socket address
     */
    Netfp_SockAddr          sockAddress;
}Netfp_Layer4Node;

/**
 * @brief
 *  NETFP Proxy Server Node
 *
 * @details
 *  NETFP Proxy and Server communicate with each other using JOSH asynchronous
 *  jobs. The node is used to track all such communication
 */
typedef struct Netfp_ServerToProxyNode
{
    /**
     * @brief   Links to other routes pending resolution
     */
    Netfp_ListNode          links;

    /**
     * @brief   Unique request identifier allocated to each request sent by the server to
     * the NETFP Proxy.
     */
    uint32_t                requestId;

    /**
     * @brief   Proxy Server interface block which holds all the information which was passed
     * to the NETFP Proxy.
     */
    Netfp_ProxyServerInfo   proxyServerInfo;
}Netfp_ServerToProxyNode;

//<fzm>

typedef struct Netfp_PendingSW_SAOffloadNode
{
    Netfp_ListNode          links;

    uint64_t                startOffloadTicks;
    Netfp_IPSecChannel*     ptrIPSecChannel;
    uint32_t                jobID;
    Netfp_EventId           eventType;
}Netfp_PendingSW_SAOffloadNode;

//</fzm>

/**
 * @brief
 *  Interface Event Meta Information
 *
 * @details
 *  The structure describes the interface event meta information which is populated
 *  and passed from the NETFP Server to each of the NETFP clients to indicate that
 *  there has been a modification to the interface subsystem. This structure is
 *  valid for the following events:
 *
 *  @sa Netfp_EventId_DELETE_INTERFACE
 *  @sa Netfp_EventId_UPDATE_INTERFACE
 */
typedef struct Netfp_EventIfMetaInfo
{
    /**
     * @brief   Interface handle which has been updated by the event.
     */
    Netfp_IfHandle          ifHandle;

    /**
     * @brief   Interface events can be generated because of multiple reasons
     */
    Netfp_EventIfReason     reason;

    /**
     * @brief   Each reason can have multiple meta information associated with it
     */
    union
    {
        /**
         * @brief   New MTU of the interface after the configuration change
         */
        uint32_t                mtu;

        /**
         * @brief   New status of the interface after the configuration change
         */
        uint32_t                status;

        /**
         * @brief   Each physical interface has a marking map which maps the inner
         * DSCP to the outer DSCP.
         */
        uint8_t                 innerToOuterDSCPMap[64];

        /**
         * @brief   Switch port information associated with the interface on which the
         * packet is to be transmitted.
         */
        uint32_t                switchPortNum;

        /**
         * @brief   Mapping which maps the socket priority to the VLAN Priority bits
         */
        Netfp_VLANPriorityMap   vlanMap;

        /**
         * @brief   New L3 QOS configuration which is to be used.
         */
        Netfp_L3QoSCfg          l3QosCfg;
    }u;
}Netfp_EventIfMetaInfo;

/**
 * @brief
 *  Fast Path Event Meta Information
 *
 * @details
 *  The structure describes the fast path event meta information which is populated
 *  and passed from the NETFP Server to each of the NETFP clients to indicate that
 *  there the fast path has been deleted. The event is a trigger to all the l4
 *  endpoints (sockets/3gpp) channels that the IP endpoints are no longer valid
 *  and hence the l4 endpoints will be not be operational anymore
 *
 *  @sa Netfp_EventId_DELETE_FP
 *  @sa Netfp_EventId_UPDATE_FAST_PATH
 */
typedef struct Netfp_EventFastPathMetaInfo
{
    /**
     * @brief   Fast Path handle for which the event has been generated
     * This could be an INBOUND/OUTBOUND Fast path; we are not differentiating
     * between the fast paths while generating the events.
     */
    void*                       fpHandle;

    /**
     * @brief   Status of the NETFP Fast Path
     */
    Netfp_Status                status;

    /**
     * @brief   Reason because of which the fast path event is generated
     */
    Netfp_Reason                reason;

    /**
     * @brief L2 connect information to be used to populate the sockets.
     */
    Netfp_SockL2ConnectInfo     l2ConnectInfo;

    /**
     * @brief The new source IP addresses in the event of a rekey/update.
     */
    Netfp_IPAddr                srcIP; //fzm

    /**
     * @brief The new destination IP addresses in the event of a rekey/update.
     */
    Netfp_IPAddr                dstIP; //fzm
}Netfp_EventFastPathMetaInfo;

/**
 * @brief
 *  Security Policy Event Meta Information
 *
 * @details
 *  The structure describes the Security Association meta information which is populated
 *  and passed from the NETFP Server to each of the NETFP clients to indicate that
 *  SA rekey has happened. The new spi is provoide in the Meta info.
 *
 *  @sa Netfp_EventId_SP_UPDATE
 */
typedef struct Netfp_EventSPMetaInfo
{
    /**
     * @brief   Status of the Security Policy
     */
    Netfp_Status            status;

    /**
     * @brief   Security policy Identifier
     */
    uint32_t                spId;

    /**
     * @brief   SPI value updated after SA rekey event
     */
    uint32_t                spi;

    /**
     * @brief   This is the general channel control information which is populated while creating
     * the SA LLD channel.
     */
    Sa_GenCtrlInfo_t        saGeneralChannelCtrlInfo;

    /**
     * @brief   Software information which is populated after the channel has been
     * created and which is stored
     */
    Netfp_SecuritySwInfo    swInfo;
}Netfp_EventSPMetaInfo;

//<fzm>

typedef struct Netfp_EventSAMetaInfo
{
    Netfp_IPAddr            dstIP;

    uint32_t                spi;

    uint32_t                swInfo[sa_MAX_SW_INFO_SIZE];

    uint32_t                jobID;
}Netfp_EventSAMetaInfo;

//</fzm>

/**
 * @brief
 *  Event Meta Information
 *
 * @details
 *  This is the event meta information which is used to exchange messages between the NETFP
 *  servers and clients and to indicate networking configuration changes.
 */
typedef struct Netfp_EventMetaInfo
{
    /**
     * @brief   Links to event meta information
     */
    Netfp_ListNode              links;

    /**
     * @brief   Event identifier
     */
    Netfp_EventId               eventId;

    /**
     * @brief   Event Meta information which is relevant on the basis of the event identifier
     */
    union
    {
        Netfp_EventIfMetaInfo           ifMeta;
        Netfp_EventFastPathMetaInfo     fpMeta;
        Netfp_EventSPMetaInfo           spMeta;
        Netfp_EventSAMetaInfo           saMeta;
    }u;

    /**
     * @brief   Job identifer which is valid only when the event has been dispatched from the
     * NETFP Server to the NETFP client. Events are sent using the asynchronous JOSH framework.
     */
    int32_t                     jobId;
}Netfp_EventMetaInfo;

/**
 * @brief
 *  NETFP Reassembly MCB
 *
 * @details
 *  The NETFP Reassembly MCB information which is required by the designed NETFP
 *  client which is responisble for handling the reassembly of all fragmentes.
 */
typedef struct Netfp_ReassemblyMCB
{
    /**
     * @brief  Reassembly configuration
     */
    Netfp_ReassemblyConfig      cfg;

    /**
     * @brief  This is the reassembly heap which is used by the NETCP subsystem to receive
     * fragmented packets. There is only 1 NETFP client in the system which is responsible
     * for the reassembly of all packets. For all other clients this heap handle is set to
     * NULL.
     */
    Pktlib_HeapHandle           reassemblyHeapHandle;

    /**
     * @brief  This is the reassembly flow identifier which is configured in the NETCP
     * subsystem which will allow the fragmented packets to be received by the NETCP and
     * passed to the NETFP client for reassembly operations. This is valid only if the client
     * is associated with a valid reassembly heap.
     */
    int32_t                     reassemblyFlowId;

    /**
     * @brief  Reassembly statistics
     */
    Netfp_ReassemblyStats       stats;

    /**
     * @brief  This is the pointer to the list of free reassembly contexts which can be used
     */
    Netfp_IPReassemblyContext*  ptrFreeContextList;

    /**
     * @brief  This is the pointer to the list of used reassembly contexts which have packets
     * being reassembled.
     */
    Netfp_IPReassemblyContext*  ptrUsedContextList;

//<fzm>
    Pktlib_Prefetch             prefetchOuterData;
    Pktlib_Prefetch             prefetchInnerData;
//</fzm>

}Netfp_ReassemblyMCB;

/**
 * @brief
 *  NETFP Multicast Info
 *
 * @details
 *  The NETFP Multicast Informational block contains all the information which is
 *  required to send/receive multicast data. The structure is used by both the NETFP
 *  Clients and servers.
 */
typedef struct Netfp_MulticastInfo
{
    /**
     * @brief  Status flag which indicates if the informational block has been initialized
     * or not.
     */
    uint8_t                 isInitialized;

    /**
     * @brief  Ethernet virtual link handle used for IPv4: This field is set by the NETFP
     * Server and is *not* used by the NETFP Clients.
     */
    Netfp_L2Handle          vlinkIPv4Handle;

    /**
     * @brief  Ethernet virtual link handle used for IPv6: This field is set by the NETFP
     * Server and is *not* used by the NETFP Clients.
     */
    Netfp_L2Handle          vlinkIPv6Handle;

    /**
     * @brief  Wildcarded IPv4 Fast Path for multicast packets: This field is only used on
     * the NETFP Server to track the programmed LUT entry
     */
    Netfp_InboundFastPath*  ptrIPv4MulticastFP;

    /**
     * @brief  Wildcarded IPv6 Fast Path for multicast packets: This field is only used on
     * the NETFP Server to track the programmed LUT entry
     */
    Netfp_InboundFastPath*  ptrIPv6MulticastFP;

    /**
     * @brief  Ethernet virtual link identifier used for IPv4: This field is set by the NETFP
     * Server and is used by the NETFP Clients to send multicast IPv4 data.
     */
    int8_t                  vlinkIPv4Id;

    /**
     * @brief  Ethernet virtual link identifier used for IPv6: This field is set by the NETFP
     * Server and is used by the NETFP Clients to send multicast IPv6 data.
     */
    int8_t                  vlinkIPv6Id;
}Netfp_MulticastInfo;

//<fzm>

typedef struct Netfp_ClientJoshJobListEntry
{
    Netfp_ListNode        links;
    void*                 packetPtr;
} Netfp_ClientJoshJobListEntry;

typedef struct Netfp_ClientJoshConfig
{
    /**
     * @brief Stores unix socket name of a client
     */
    char                          clientChannelName[108];

    /**
     * @brief Stores unix socket name of the server
     */
    char                          serverChannelName[108];

    /**
     * @brief Stores unix socket fd
     */
    int                           sockFd;
} Netfp_ClientJoshConfig;
//</fzm>

/**
 * @brief
 *  NETFP Client MCB
 *
 * @details
 *  The NETFP Client MCB holds all the relevant information pertinent to the
 *  NETFP Client.
 */
typedef struct Netfp_ClientMCB
{
    /**
     * @brief  Configuration which was used to initialize the NETFP client.
     */
    Netfp_ClientConfig          cfg;

    /**
     * @brief  Proxy configuration applicable only for the NETFP Proxy clients.
     */
    Netfp_ProxyCfg              proxyCfg;

    /**
     * @brief  Name Database handle. All resources which are created or used by the
     * NETFP client will be added/access in this database.
     */
    Name_DBHandle               databaseHandle;

    /**
     * @brief  Status of the NETFP Client.
     */
    Netfp_ClientStatus          status;

    /**
     * @brief   This is the CPPI handle to the NETCP CPDMA block
     */
    Cppi_Handle                 passCPDMAHandle;

    /**
     * @brief   These are the queues which are used to send data to the NETCP subsystem.
     * These are the well defines queues which are mapped to the PA CPDMA blocks. These
     * queues are shared between the NETFP server and all the clients.
     */
    Qmss_QueueHnd               netcpTxQueue[QMSS_MAX_PASS_QUEUE];

    /**
     * @brief   These are the queues which are used to store the packets after the F8
     * operation and before they are submitted for F9. This is a temporary queue and
     * should be optimized. There is 1 queue for each SRB
     */
    Qmss_QueueHnd               F8ToF9QueueHnd[3];

    /**
     * @brief   These are the queues which are used to store the packets after the F9
     * operation and before they are submitted for F8. This is a temporary queue and
     * should be optimized. There is 1 queue for each SRB
     */
    Qmss_QueueHnd               F9ToF8QueueHnd[3];

    /**
     * @brief  JOSH Node handle associated with the NETFP client.
     */
    Josh_NodeHandle             joshHandle;

    /**
     * @brief  SNOW3G Registered F8 functionality for SRB
     */
    Netfp_F8Function            f8;

    /**
     * @brief  SNOW3G Registered F9 functionality for SRB
     */
    Netfp_F9Function            f9;

    /**
     * @brief  The NETFP DSP Client uses this channel to send messages
     * to the DSP server.
     */
    MsgCom_ChHandle             clientChannel;

    /**
     * @brief  This is the NETFP Server channel associated with the NETFP client.
     */
    MsgCom_ChHandle             serverChannel;

    /**
     * @brief  Multicast Services Informational block.
     */
    Netfp_MulticastInfo         multicastInfo;

    /**
     * @brief  List of all the sockets which have been opened by the NETFP client.
     */
    Netfp_Socket*               ptrSocketList;

    /**
     * @brief  List of all the flows which have been opened by the NETFP client.
     */
    Netfp_FlowInfo*             ptrFlowList;

    /**
     * @brief  List of all the users which have been registered by the NETFP client.
     */
    Netfp_User*                 ptrUserList;

    /**
     * @brief  Pointer to the reassembly MCB. This is valid only for the client which has
     * been registered for reassembly. For all other clients this is set to NULL.
     */
    Netfp_ReassemblyMCB*        ptrReassemblyMCB;

    /**
     * @brief  Channel handle of PA CPDMA corresponding to SA air ciphering Tx channel.
     */
    Cppi_ChHnd                  cppiCipherTxChHnd;

    /**
     * @brief   Application registered reassembly hook(s)
     */
    Netfp_HookFunction          reassemblyHook[Netfp_Hook_POST_INNER_REASSEMBLY + 1];

    /**
     * @brief   Application registered reassembly argument(s)
     */
    uint32_t                    reassemblyHookArg[Netfp_Hook_POST_INNER_REASSEMBLY + 1];

    Netfp_ClientJoshConfig     joshConfig;

    Netfp_SaEventHandler       addSaEventHandler;

    Netfp_SaEventHandler       delSaEventHandler;
}Netfp_ClientMCB;

/**
 * @brief
 *  NETFP Client Block
 *
 * @details
 *  The NETFP Client blocks are used by the NETFP Server to hold all the client specific
 *  information. There could be multiple NETFP client blocks which are valid and associated
 *  with the NETFP server.
 */
typedef struct Netfp_ClientBlock
{
    /**
     * @brief  Name of the NETFP client
     */
    char                    name[NETFP_MAX_CHAR + 1];

    /**
     * @brief  Handle of the NETFP client. This is an opaque handle from the NETFP Server's
     * perspective.
     */
    Netfp_ClientHandle      clientHandle;

    /**
     * @brief  Status of the NETFP client on the server.
     */
    Netfp_ClientStatus      status;

    /**
     * @brief  Handle of the server MSGCOM channel. There is a unique MSGCOM channel created
     * on the server which maps to a single NETFP client.
     */
    MsgCom_ChHandle         serverChannel;

    /**
     * @brief  Handle of the client MSGCOM channel. There is a unique MSGCOM writer channel
     * found on the server which maps to a single NETFP client.
     */
    MsgCom_ChHandle         clientChannel;

    /**
     * @brief  JOSH Node handle which has the JOSH framework handle between the NETFP
     * server and the client. Active clients need to have a valid JOSH handle else the
     * client block is considered to be inactive and the rest of the fields in this
     * structure are ignored.
     */
    Josh_NodeHandle         joshHandle;

    /**
     * @brief  The NETFP server will allocate an event from the free list above and
     * will JOSHify the event to the NETFP client. This is done via the JOSH asynch
     * framework. Elements in the list are events which need to be sent to the NETFP
     * clients but have still not been sent.
     */
    Netfp_EventMetaInfo*    eventDispatchList;

    /**
     * @brief  The NETFP server will place an event in this list once it has been
     * sent to the NETFP client but the response has still not been received. Once
     * the response has been received the server will remove and place the event
     * back into the free list.
     */
    Netfp_EventMetaInfo*    eventDispatchedList;

    /**
     * @brief (fzm) Stores unix socket name of a client
     */
    char                    clientChannelName[108];

    /**
     * @brief (fzm) Stores unix socket name of the server (for particular client)
     */
    char                    serverChannelName[108];
}Netfp_ClientBlock;

/**
 * @brief
 *  NETFP Command Set Block
 *
 * @details
 *  The PA subsystem can be configured by various command sets. These command sets
 *  need to be uniquely identified in the system by keeping a unique index and making
 *  sure that there are no overlaps. The structure keeps track of the unique index
 *  and also tracks the various command sets.
 */
typedef struct Netfp_CmdSetBlock
{
    /**
     * @brief  This is a global counter which generates unique command set tracking
     * index.
     */
    uint32_t        cmdSetIndexCounter;

    /**
     * @brief  Remove Header Tracking Index: This command set has been configured to
     * ensure that the networking headers are removed by the NETCP subsytem.
     */
    int32_t         cmdSetHdrRemoveIndex;

    /**
     * @brief  Frame Protocol CRC Tracking Index: This command set has been configured to
     * ensure that the FP CRC verification is done on the incoming WCDMA packets by the NETCP subsytem.
     */
    int32_t         cmdSetFpCrcIndex;
}Netfp_CmdSetBlock;

/**
 * @brief
 *  Cascaded information maintained on NetFP server.
 *
 * @details
 *  This is an internal data structure which keeps track of all the
 *  cascaded entries in the system.
 */
typedef struct Netfp_EthRuleInfo
{
    /**
     * @brief   Links to other cascading entry
     */
    Netfp_ListNode        links;

    /**
     * @brief   Cascading Configuration.
     */
    Netfp_EthRuleCfg      cfg;

    /**
     * @brief   Ingress Physical Interface for the cascading entry.
     */
    Netfp_PhyInterface*   ptrIngressEthIntf;

    /**
     * @brief   Egress Physical Interface for the cascading entry.
     */
    Netfp_PhyInterface*   ptrEgressEthIntf;

    /**
     * @brief   LUT1-0 Handles for this this cascading entry
     */
    Netfp_L2Handle        ethHandle;

    /**
     * @brief   LUT1-0 index used to programmed the Ethernet Rule
     */
    uint32_t              lutIndex;

    /**
     * @brief   User Statistics index
     */
    uint16_t              userStatsIndex[NETFP_MAX_USR_STATS_PER_RULE];
}Netfp_EthRuleInfo;

//fzm-->
/**
 * @brief
 *  Data about Software LUT1_1 extension
 */
typedef struct Netfp_SwLutInfo
{
    /**
     * @brief  Status flag which indicates if the informational block has been initialized
     * or not.
     */
    uint8_t                isInitialized;

    /**
     * @brief  Init data.
     */
    Netfp_initSwLut        initSwLut;
}Netfp_SwLutInfo;
//<--fzm

/**
 * @brief
 *  NETFP Server MCB
 *
 * @details
 *  The NETFP Server MCB holds all the relevant information pertinent to the
 *  NETFP Server.
 */
typedef struct Netfp_ServerMCB
{
    /**
     * @brief  Configuration which was used to initialize the NETFP Server.
     */
    Netfp_ServerConfig          cfg;

    /**
     * @brief  Name Database handle. All resources which are created or used by the
     * NETFP server will be added/access in this database.
     */
    Name_DBHandle               databaseHandle;

    /**
     * @brief  Client blocks which hold all the client specific information. This
     * at any point in time will have a list of all the valid NETFP clients which
     * are registered with the NETFP Server.
     */
    Netfp_ClientBlock           clientBlock[NETFP_MAX_CLIENTS];

    /**
     * @brief  NETFP Proxy client block which is registered with the system.
     */
    Netfp_ClientBlock*          proxyClientBlock;

    /**
     * @brief   This is the CPPI handle to the NETCP CPDMA block
     */
    Cppi_Handle                 passCPDMAHandle;

    /**
     * @brief   This is the handle returned by the PA driver.
     */
    Pa_Handle                   paHandle;

    /**
     * @brief   This is the handle returned by the SA driver.
     */
    Sa_Handle                   saHandle;

    /**
     * @brief   This is the PA response queue which will hold all the responses
     * to commands which have been sent by the host.
     */
    Qmss_QueueHnd               paCfgRespQueue;

    /**
     * @brief   This is the PA Receive flow handle which is used to configure the
     * PA command handler.
     */
    Cppi_FlowHnd                paRxFlowHandle;

    /**
     * @brief   IPSEC Flow Handle: The flow handle is used to pass packets between the
     * PA and SA subsystem.
     *
     * INBOUND Direction: Packets received are passed from the PA after the LUT1-1 match
     * to the SA where they are decrypted and are then passed back to the PA for the LUT1-2
     * match.
     *
     * OUTBOUND Direction: Packets are pushed to the SA for encryption and after encryption
     * packets are passed back to the PA for command set processing (QOS etc)
     */
    Cppi_FlowHnd                ipsecFlowHandle;

    /**
     * @brief  Channel handle of PA CPDMA corresponding to SA air ciphering Tx channel.
     */
    Cppi_ChHnd                  cppiCipherTxChHnd;

    /**
     * @brief   These are the queues which are used to send data to the NETCP subsystem.
     */
    Qmss_QueueHnd               netcpTxQueue[QMSS_MAX_PASS_QUEUE];

    /**
     * @brief   These are the command sets which are programmed in the NETCP subsystem
     * The block keeps track of these sets.
     */
    Netfp_CmdSetBlock           cmdSet;

    /**
     * @brief  Multicast Services Informational block.
     */
    Netfp_MulticastInfo         multicastInfo;

    /**
     * @brief   Interface based routing: base flow identifier
     */
    uint32_t                    interfaceBaseFlowId;

    /**
     * @brief   Interface based routing: base queue number
     */
    uint32_t                    interfaceBaseQueue;

    /**
     * @brief   NAT-T UDP encapsulation Configuration
     */
    Netfp_NattCfg               nattCfg;

    /**
     * @brief   List of all the registered interfaces in the system.
     */
    Netfp_Interface*            ptrInterfaceList;

    /**
     * @brief   List of all the inbound fast paths in the system
     */
    Netfp_InboundFastPath*      ptrInboundFPList;

    /**
     * @brief   List of all the inbound fast paths in the system
     */
    Netfp_OutboundFastPath*     ptrOutboundFPList;

    /**
     * @brief   Inbound security policy list
     */
    Netfp_SPInfo*               ptrInboundSPList;

    /**
     * @brief   Outbound security policy list
     */
    Netfp_SPInfo*               ptrOutboundSPList;

    /**
     * @brief   Pointer to the pending list of all information which has been passed by the NETFP
     * server to the proxy. These are waiting for the PROXY to update the status
     */
    Netfp_ServerToProxyNode*    ptrPendingProxyServerList;

    Netfp_PendingSW_SAOffloadNode* ptrPendingSW_SAOffloadList; //fzm

    /**
     * @brief   Server security channel list: This is a list of the server security channels which are
     * active in the server.
     */
    Netfp_SrvSecurityChannel*   ptrServerSecurityChannelList;

    /**
     * @brief   Pointer to the security context identifier database
     */
    uint32_t*                   ptrSecContextIdDatabase;

    /**
     * @brief   List of all the Inbound IPSEC channels which are present in the server
     */
    Netfp_IPSecChannel*         ptrIPSecInboundChannels;

    /**
     * @brief   List of all the Outbound IPSEC channels which are present in the server
     */
    Netfp_IPSecChannel*         ptrIPSecOutboundChannels;

    /**
     * @brief   List of the all the physical interfaces present in the server.
     */
    Netfp_PhyInterface*         ptrPhyInterfaceList;

    /**
     * @brief   Status flag which indicates that a route recomputation is in progress.
     */
    uint32_t                    isRouteRecomputationInProgress;

    /**
     * @brief   List of the the routes which are pending the recomputation
     */
    Netfp_RecomputeRoute*       ptrRouteRecomputationList;

    /**
     * @brief   List of the all the added Ethernet Rules present in the server.
     */
    Netfp_EthRuleInfo*          ptrEtherRuleList;

    /**
     * @brief   List of the all the NAT-T IP channels configuration in LUT1-1.
     */
    Netfp_NattIPInfo*           ptrNattIPChanList;

    /**
     * @brief   List of the all the IP LUT channels configuration in LUT1-1.
     */
    Netfp_LUTInfo*              ptrOuterIPLutInfoList;

    /**
     * @brief   List of the all the SW IP LUT channels configuration in LUT1-1.
     */
    Netfp_SwIPInfo*            ptrSwIPChanList; //fzm

    /**
     * @brief   List of the all the NAT-T SW IP LUT channels configuration in LUT1-1.
     */
    Netfp_NattSwIPInfo*        ptrNattSwIPChanList; //fzm

    /**
     * @brief   Initialization data for SW IP LUT.
     */
    Netfp_SwLutInfo             swLutInfo; //fzm

    /**
     * @brief   Virtual link to be used for SW IP channels
     */
    Netfp_PAVirtLink*           ptrNetfpSW_LUT_PAVlink; //fzm

    /**
     * @brief   List of the all the IP LUT channels configuration in LUT1-2.
     */
    Netfp_LUTInfo*              ptrInnerIPLutInfoList;

    /**
     * @brief   List of the all the Layer4 nodes (LUT-2) entries
     */
    Netfp_Layer4Node*           ptrLayer4List;

    /**
     * @brief   Flag which indicates if Frame Protocol CRC Offload is enabled.
     */
    uint32_t                    frameProtoCrcOffload;

    /**
     * @brief   Pointer to the garbage context security list.
     */
    Netfp_SecurityContext*      ptrGarbageSecurityContextList;

    /**
     * @brief   User statistics configuration
     */
    Netfp_SysUserStatCfg        userStatCfg;

    /**
     * @brief   This is the pointer to the memory to get the user statistics.
     */
    paUsrStats_t*               ptrUserStats;

    /**
     * @brief   Pointer to the slow cleanup context security list.
     */
    Netfp_SrvSecurityChannel*      ptrSlowCleanupSecurityContextList;
}Netfp_ServerMCB;

/**
@}
*/

/**********************************************************************
 ************************* Inline Functions ***************************
 **********************************************************************/

/**
 *  @b Description
 *  @n
 *      Utility wrapper function which provides logs NETFP server messages
 *      via the application supplied logging function
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP server
 *  @param[in]  logLevel
 *      Logging Level
 *  @param[in]  fmt
 *      Format string
 *  @param[in]  ...
 *      Variable arguments
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable.
 */
static inline void Netfp_logMsg
(
    Netfp_ServerMCB*    ptrNetfpServer,
    Netfp_LogLevel      logLevel,
    const char*         fmt,
    ...
)
{
    va_list arg;

    /* Pass control to the application supplied calling function. */
    if (ptrNetfpServer->cfg.logFxn)
    {
        va_start (arg, fmt);
        ptrNetfpServer->cfg.logFxn (logLevel, fmt, arg);
        va_end (arg);
    }
    return;
}

// <fzm>
static inline void Netfp_dumpMsg
(
    Netfp_ServerMCB*    ptrNetfpServer,
    Netfp_LogLevel      logLevel,
    const char*         fmt,
    ...
    )
{
    va_list arg;

    /* Pass control to the application supplied calling function. */
    if (ptrNetfpServer->cfg.dumpFxn)
    {
        va_start(arg, fmt);
        ptrNetfpServer->cfg.dumpFxn(logLevel, fmt, arg);
        va_end(arg);
    }
    return;
}
// </fzm>

/**
 *  @b Description
 *  @n
 *      Utility Function which determines the IP address family
 *
 *  @param[in]   ipAddress
 *      IP address for which the family is required
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      1 - IPv4 address
 *  @retval
 *      0 - IPv6 address
 */
static inline uint8_t Netfp_isAddressIPv4(Netfp_IPAddr ipAddress)
{
    /* Check the version? */
    if (ipAddress.ver == Netfp_IPVersion_IPV4)
        return 1;
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Internal Utility Function which is used to match the IP address.
 *      The function will determine the family of the the IP address and will
 *      perform the necessary comparison.
 *
 *  @param[in]  ptrIPAddress1
 *      IP Address1
 *  @param[in]  ptrIPAddress2
 *      IP Address 2
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      1   -   IP Address is the same
 *  @retval
 *      0   -   IP Address is not the same
 */
static inline uint8_t Netfp_matchIP (Netfp_IPAddr* ptrIPAddress1, Netfp_IPAddr* ptrIPAddress2)
{
    /* Comparison can proceed only if they belong to the same version. */
    if (ptrIPAddress1->ver != ptrIPAddress2->ver)
        return 0;

    /* Is the IP address IPv4 or IPv6? */
    if (ptrIPAddress1->ver == Netfp_IPVersion_IPV4)
    {
        /* IPv4 Address: */
        if (ptrIPAddress1->addr.ipv4.u.a32 == ptrIPAddress2->addr.ipv4.u.a32)
            return 1;
        return 0;
    }

    /* IPv6 Address: */
    if ((ptrIPAddress1->addr.ipv6.u.a32[0] == ptrIPAddress2->addr.ipv6.u.a32[0]) &&
        (ptrIPAddress1->addr.ipv6.u.a32[1] == ptrIPAddress2->addr.ipv6.u.a32[1]) &&
        (ptrIPAddress1->addr.ipv6.u.a32[2] == ptrIPAddress2->addr.ipv6.u.a32[2]) &&
        (ptrIPAddress1->addr.ipv6.u.a32[3] == ptrIPAddress2->addr.ipv6.u.a32[3]))
        return 1;
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to check if src or dst IP address is a wild
 *  carding address(all zeros)
 *
 *  @param[in]  ipAddr
 *      IP address to check against the wild carding address
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      1 -   Wild carding address
 *  @retval
 *      0 -   Not a Wild carding address
 */
static inline int32_t Netfp_checkIPForWildCarding (Netfp_IPAddr ipAddr)
{
    Netfp_IPAddr   ipWildCardingAddr;

    /* Prepare for Wild carding Address */
    memset((void *)&ipWildCardingAddr, 0, sizeof(Netfp_IPAddr));
    ipWildCardingAddr.ver = ipAddr.ver;

    return ( Netfp_matchIP(&ipAddr, &ipWildCardingAddr) );
}

/**
 *  @b Description
 *  @n
 *      Utility function which is used to return the minimum of two values
 *
 *  @param[in]  a
 *      First value to be compared
 *  @param[in]  b
 *      Second value to be compared
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Minimum value
 */
static inline uint32_t Netfp_min (uint32_t a, uint32_t b)
{
    return (a > b) ? b : a;
}

/**********************************************************************
 ********************** Internal EXPORTED API *************************
 **********************************************************************/

/* Client/Server Services: */
extern int32_t _Netfp_stopClient (Netfp_ServerMCB* ptrNetfpServer, const char* clientName, int32_t* errCode);
extern int32_t _Netfp_getServerStatus (Netfp_ServerMCB* ptrNetfpServer, int32_t* errCode);

/* SA Services: */
extern int32_t Netfp_saInit (Netfp_ServerMCB* ptrNetfpServer, int32_t* errCode);
extern Netfp_SrvSecChannelHandle _Netfp_createSecurityChannel(Netfp_ServerMCB* ptrNetfpServer, Netfp_SecChannelCfg* ptrSecChannelCfg,
                                                              Netfp_SecuritySwInfo* ptrSwInfo, int32_t* errCode);
extern Netfp_SrvSecChannelHandle Netfp_createSecurityChannel (Netfp_ClientMCB* ptrNetfpClient, Netfp_SecChannelCfg* ptrSecChannelCfg,
                                                              Netfp_SecuritySwInfo* ptrSwInfo, int32_t* errCode);
extern int32_t _Netfp_createSrb (Netfp_ServerMCB* ptrNetfpServer, Netfp_UserCfg* ptrUserCfg, Netfp_UserSRBInfo* ptrUserSrbInfo, int32_t* errCode);
extern int32_t Netfp_createSrb (Netfp_ClientMCB* ptrNetfpClient, Netfp_UserCfg* ptrUserCfg, Netfp_UserSRBInfo* ptrUserSrbInfo, int32_t* errCode);
extern int32_t _Netfp_deleteSrb (Netfp_ServerMCB* ptrNetfpServer, Netfp_UserSRBInfo* ptrUserSrbInfo, int32_t* errCode);
extern int32_t Netfp_deleteSrb (Netfp_ClientMCB* ptrNetfpClient, Netfp_UserSRBInfo* ptrUserSrbInfo, int32_t* errCode);

extern int32_t Netfp_deleteSecurityChannel (Netfp_ClientMCB* ptrNetfpClient, Netfp_SrvSecChannelHandle srvSecurityChannel,
                                            int32_t* errCode);
extern int32_t _Netfp_deleteSecurityChannel(Netfp_ServerMCB* ptrNetfpServer, Netfp_SrvSecChannelHandle srvSecurityChannel,
                                            int32_t* errCode);
extern int32_t Netfp_deleteSecurityChannelSlow
(
    Netfp_ServerMCB*            ptrNetfpServer,
    Netfp_SrvSecChannelHandle   srvSecurityChannel,
    int32_t*                    errCode
);

extern int32_t Netfp_executeSlowSecContextCleanup(Netfp_ServerMCB* ptrNetfpServer);

extern int32_t Netfp_killSecurityChannel(Netfp_ServerMCB* ptrNetfpServer, int32_t* errCode);
extern int32_t Netfp_encryptPkt(Netfp_Socket* ptrNetfpSocket, Netfp_SockTxMetaInfo* ptrSockTxMetaInfo, Ti_Pkt* ptrPayload);
extern int32_t Netfp_killSecurityAssociation(Netfp_ServerMCB* ptrNetfpServer, int32_t* errCode);
extern int32_t Netfp_getSecurityChannelOpt (Netfp_ClientMCB* ptrNetfpClient, Netfp_SrvSecChannelHandle srvSecurityChannelHandle,
                                            Netfp_OptionTLV* ptrOptInfo, int32_t* errCode);
extern int32_t Netfp_getSAStatistics (Sa_ChanHandle saChannelHandle, Sa_Stats_t*  ptrSAStats);
extern int32_t _Netfp_getIPsecStats (Netfp_ServerMCB* ptrNetfpServer, Netfp_IPSecChannel* ptrIPSecChannel,
                                     Netfp_IpSecStats* ipsecStats, int32_t* errCode);
extern int32_t Netfp_getChanSwInfo(Sa_ChanHandle saChannelHandle, Sa_PktDir_t dir, Sa_SWInfo_t*    swInfo);
extern void Netfp_setupIPSECChannelPMTU(Netfp_ServerMCB* ptrNetfpServer, Netfp_IPSecChannel* ptrIPSecChannel, uint32_t newPMTU);
extern int32_t Netfp_configureCountC(Netfp_ServerMCB* ptrNetfpServer, Netfp_SrvSecurityChannel* ptrSrvSecurityChannel, uint32_t countC);
extern int32_t Netfp_getEncIvSize(Netfp_IPSecChannel* ptrIPSecChannel);
extern int32_t Netfp_getAuthDigestSize(Netfp_IPSecChannel* ptrIPSecChannel);

/* Interface Module: */
extern Netfp_Interface* _Netfp_findInterface (Netfp_ServerMCB* ptrNetfpServer, const char* name, Netfp_InterfaceCfg* ptrInterfaceCfg);
extern int32_t Netfp_transmitInterface(Netfp_Socket* ptrSocket,Netfp_SockTxMetaInfo* ptrSockTxMetaInfo, Ti_Pkt* ptrPayload);
extern int32_t Netfp_transmitInterface_FZM(Netfp_Socket* ptrNetfpSocket, Netfp_SockTxMetaInfo_FZM* ptrSockTxMetaInfo, Ti_Pkt* ptrPayload);
extern int32_t Netfp_rawTransmitInterface(Netfp_ClientMCB* ptrNetfpClient, Ti_Pkt* ptrPayload, int32_t switchPort, int32_t* errCode);
extern int32_t Netfp_killInterface(Netfp_ServerMCB* ptrNetfpServer, int32_t* errCode);

/* PA Module: */
extern Pa_Handle Netfp_paInit(Netfp_ServerMCB* ptrNetfpServer, uint32_t maxL2Handles, uint32_t maxL3Handles);
extern int32_t Netfp_paInitCmdHandler(Netfp_ServerMCB* ptrNetfpServer, Pktlib_HeapHandle heapHandle);
extern int32_t Netfp_paSetupCommandSets(Netfp_ServerMCB* ptrNetfpServer);
extern int32_t Netfp_registerServices (Josh_NodeHandle nodeHandle);
extern int32_t Netfp_paAddEth (Netfp_ServerMCB*  ptrNetfpServer, paEthInfo2_t* ethInfo, paRouteInfo2_t*  routeInfo,  paRouteInfo2_t*  failInfo,
                                   Netfp_L2Handle* ethHandle,  uint32_t lutIndex,  int32_t* errCode);
extern int32_t Netfp_paDelEth(Netfp_ServerMCB* ptrNetfpServer, Netfp_L2Handle ethHandle);
extern int32_t Netfp_addIP(Netfp_ServerMCB* ptrNetfpServer, Netfp_IPCfg* ptrIPCfg, paLnkHandle_t prevLinkHandle,
                           paLnkHandle_t nextLinkHandle, Netfp_IPSecChannel* ptrIPSecChannel,
                           Netfp_DstInfo* ptrDstInfo, uint8_t flags, Netfp_LUTInfo*  ptrNetfpLutInfo, Netfp_L3Handle* ipHandle);
extern int32_t Netfp_delIP(Netfp_ServerMCB* ptrNetfpServer, Netfp_L3Handle ipHandle);
extern int32_t Netfp_addGTPTunnel(Netfp_ServerMCB* ptrNetfpServer, uint32_t gtpTunnelEndpointId,
                                  Netfp_L3Handle l3Handle, uint32_t swInfo,
                                  Netfp_SrvSecurityChannel* ptrSrvSecurityChannel, Netfp_DstInfo* ptrDstInfo, uint8_t replace,
                                  uint8_t removeHeaders, paHandleL4_t* l4Handle);
extern int32_t Netfp_addPort(Netfp_ServerMCB* ptrNetfpServer, uint16_t portNumber, Netfp_L3Handle l3Handle, uint32_t flowId,
                             uint32_t swInfo, Netfp_DstInfo* ptrDstInfo, paCmdInfo_t* ptrCmdSetInfo, paHandleL4_t* l4Handle);
extern int32_t Netfp_delL4Handle(Netfp_ServerMCB* ptrNetfpServer, paHandleL4_t l4Handle);
extern int32_t Netfp_configureReassembly(Netfp_ClientHandle clientHandle, paIpReassmConfig_t* ptrInnerIPReassemblyConfig,
                                         paIpReassmConfig_t* ptrOuterIPReassemblyConfig, int32_t* errCode);
extern int32_t _Netfp_configureReassembly(Netfp_ServerMCB* ptrNetfpServer, paIpReassmConfig_t* ptrInnerIPReassemblyConfig,
                                          paIpReassmConfig_t* ptrOuterIPReassemblyConfig, int32_t* errCode);
extern int32_t _Netfp_configureGTPUControlMessage(Netfp_ServerMCB* ptrNetfpServer, Netfp_GTPUControlCfg* ptrGTPUControlCfg, int32_t* errCode);
extern int32_t Netfp_killInboundFastPath(Netfp_ServerMCB* ptrNetfpServer, int32_t* errCode);
extern int32_t Netfp_killOutboundFastPath(Netfp_ServerMCB* ptrNetfpServer, int32_t* errCode);
extern int32_t Netfp_killSecurityPolicy(Netfp_ServerMCB* ptrNetfpServer, int32_t* errCode);
extern int32_t Netfp_setupDefaultFailInfo(Netfp_ServerMCB* ptrNetfpServer, paRouteInfo2_t* ptrFailInfo);
extern int32_t Netfp_createMACVirtualLink(Netfp_ServerMCB* ptrNetfpServer, Netfp_L2Handle* vlinkHandle, int8_t* vlinkId, int32_t* errCode);
extern int32_t Netfp_deleteMACVirtualLink(Netfp_ServerMCB* ptrNetfpServer, Netfp_L2Handle vlinkHandle, int32_t* errCode);

/* Policy checks: */
extern int32_t Netfp_policyCheckPort(Netfp_Direction direction, Netfp_SPInfo* ptrSP, uint16_t port);
extern Netfp_SPInfo* Netfp_findSPById (Netfp_ServerMCB* ptrNetfpServer, uint32_t spId);
extern uint8_t Netfp_policyCheckIP (Netfp_Direction direction, Netfp_SPInfo* ptrSP, Netfp_IPAddr ip);

/* Socket Services: */
extern int32_t Netfp_secureBind(Netfp_SockHandle sockHandle, Netfp_SockAddr* ptrSockAddr,
                                Netfp_SrvSecChannelHandle srvSecurityChannelHandle, int32_t* errCode);
extern int32_t Netfp_socketHook (Netfp_Hook hook, Netfp_ClientMCB* ptrNetfpClient, Netfp_Socket* ptrNetfpSocket, Ti_Pkt* ptrPayload);

/* Route Services: */
extern int32_t Netfp_sendProxyRequest(Netfp_ServerMCB* ptrNetfpServer, Netfp_ProxyServerBulkMsg* ptrProxyServerBulkInfo);
extern void Netfp_cleanupRequest(Netfp_ServerMCB* ptrNetfpServer, Netfp_ProxyServerBulkMsg* ptrProxyServerBulkInfo);
extern int32_t Netfp_populateServerToProxyNode(Netfp_ServerMCB* ptrNetfpServer, Netfp_ProxyServerInfo* ptrProxyServerInfo,uint32_t* ptrRequestId);
extern int32_t Netfp_processRouteRecomputation(Netfp_ServerMCB* ptrNetfpServer, int32_t* errCode);

/* User Stats API: */
extern int32_t Netfp_createUserStats(Netfp_ServerMCB* ptrNetfpServer, uint32_t numStats, Netfp_UserStatsCfg* ptrUserStat,
                                     uint16_t* ptrStatsIdx, int32_t* errCode);
extern int32_t Netfp_getUserStats(Netfp_ServerMCB* ptrNetfpServer, paUsrStats_t* ptrUserStats, int32_t* errCode);
extern int32_t Netfp_deleteUserStats(Netfp_ServerMCB* ptrNetfpServer, uint32_t numUserStats, uint16_t* ptrUserStatsIndex, int32_t* errCode);

/* Event Services: */
extern int32_t Netfp_initClientEventMgmt(Netfp_ServerMCB* ptrNetfpServer, int32_t clientIndex, int32_t* errCode);
extern int32_t Netfp_deinitClientEventMgmt(Netfp_ServerMCB* ptrNetfpServer, int32_t clientIndex, int32_t* errCode);
extern int32_t Netfp_processEvents (Netfp_ServerMCB* ptrNetfpServer);
extern void Netfp_generateEvent(Netfp_ServerMCB* ptrNetfpServer, Netfp_EventMetaInfo* ptrEventInfo);
extern void Netfp_updateFP (Netfp_ServerMCB* ptrNetfpServer, Netfp_Reason reason, uint32_t spi, Netfp_Interface* ptrNetfpInterface);
extern void Netfp_updateSP (Netfp_ServerMCB* ptrNetfpServer, Netfp_Reason reason, Netfp_SAHandle origSAHandle, Netfp_SAHandle newSAHandle);
extern void Netfp_updateSA (Netfp_ServerMCB* ptrNetfpServer, Netfp_Reason reason, Netfp_Interface* ptrNetfpInterface);

/* Fast Path Services: */
extern int32_t Netfp_isValidInboundFastPath(Netfp_ServerMCB* ptrNetfpServer, Netfp_InboundFastPath* ptrInboundFP);
extern int32_t Netfp_isValidOutboundFastPath(Netfp_ServerMCB* ptrNetfpServer, Netfp_OutboundFastPath* ptrOutboundFP);
extern void Netfp_ageFastPathPMTU(Netfp_ServerMCB* ptrNetfpServer, uint32_t timeout);
extern void Netfp_setupFastPathPMTU(Netfp_ServerMCB* ptrNetfpServer, Netfp_OutboundFastPath* ptrOutboundFastPath, uint32_t newPMTU);
extern void Netfp_notifyOutboundFPChanges(Netfp_ServerMCB* ptrNetfpServer, Netfp_OutboundFastPath* ptrOutboundFastPath,
                                          uint8_t bIsSameInterface, Netfp_Interface* ptrResolvedInterface,
                                          uint8_t* oldNextHopMACAddress, uint8_t* resolvedNextHopMACAddress);
extern void Netfp_removeFPFromRecomputationList(Netfp_ServerMCB* ptrNetfpServer, const Netfp_OutboundFastPath *ptrOutboundFastPath);
extern void Netfp_removeSAFromRecomputationList(Netfp_ServerMCB* ptrNetfpServer, const Netfp_IPSecChannel *ptrIPSecChannel);

/* Socket Services: */
extern void Netfp_populateSocketL2ConnectInfo(Netfp_ServerMCB* ptrNetfpServer,Netfp_OutboundFastPath* ptrOutboundFP, Netfp_SockL2ConnectInfo*);
extern void Netfp_socketUpdateInterfaceHandler(Netfp_ClientMCB* ptrNetfpClient, Netfp_EventMetaInfo* ptrEventMetaInfo);
extern void Netfp_socketUpdateFastPathHandler(Netfp_ClientMCB* ptrNetfpClient, Netfp_EventMetaInfo* ptrEventMetaInfo);
extern void Netfp_socketDeleteFastPathHandler(Netfp_ClientMCB* ptrNetfpClient, Netfp_EventMetaInfo* ptrEventMetaInfo);
extern int32_t Netfp_suspendSocket(Netfp_SockHandle sockHandle, Netfp_SockAddr* ptrSockAddr, int32_t* errCode);
extern int32_t Netfp_resumeSocket(Netfp_SockHandle sockHandle, Netfp_SockAddr* ptrSockAddr, int32_t* errCode);
extern int32_t Netfp_resumeSecureSocket(Netfp_SockHandle sockHandle, Netfp_SockAddr* ptrSockAddr, Netfp_SrvSecChannelHandle secChHandle, int32_t* errCode);
extern void Netfp_incNonSecureIPv4Stats(Netfp_Socket* ptrNetfpSocket, Netfp_SockTxMetaInfo* ptrSockTxMetaInfo, Ti_Pkt* ptrPayload);
extern void Netfp_incNonSecureStats_FZM(Netfp_Socket* ptrNetfpSocket, Netfp_SockTxMetaInfo_FZM* ptrSockTxMetaInfo, uint32_t packetLen);
extern void Netfp_incSecureIPv4Stats(Netfp_Socket* ptrNetfpSocket, Netfp_SockTxMetaInfo* ptrSockTxMetaInfo, Ti_Pkt* ptrPayload);
extern void Netfp_incNonSecureIPv6Stats(Netfp_Socket* ptrNetfpSocket, Netfp_SockTxMetaInfo* ptrSockTxMetaInfo, Ti_Pkt* ptrPayload);
extern void Netfp_incSecureIPv6Stats(Netfp_Socket* ptrNetfpSocket, Netfp_SockTxMetaInfo* ptrSockTxMetaInfo, Ti_Pkt* ptrPayload);

/* Service Registeration: */
extern int32_t Netfp_registerEventMgmtServices(Josh_NodeHandle nodeHandle);
extern int32_t Netfp_registerReassemblyServices (Josh_NodeHandle nodeHandle);
extern int32_t Netfp_registerInterfaceServices (Josh_NodeHandle nodeHandle);
extern int32_t Netfp_registerFastPathServices (Josh_NodeHandle nodeHandle);
extern int32_t Netfp_registerRouteServices (Josh_NodeHandle nodeHandle);
extern int32_t Netfp_registerSocketServices (Josh_NodeHandle nodeHandle);
extern int32_t Netfp_registerPAServices(Josh_NodeHandle nodeHandle);
extern int32_t Netfp_registerSAServices (Josh_NodeHandle nodeHandle);
extern int32_t Netfp_registerIPSecServices (Josh_NodeHandle nodeHandle);
extern int32_t Netfp_registerProxyServerServices (Josh_NodeHandle nodeHandle);
extern int32_t Netfp_registerMulticastServices (Josh_NodeHandle nodeHandle);

/* Reassembly Management Module: */
extern void Netfp_defaultReassemblyTrafficMgmt (Netfp_ClientHandle clientHandle, void* ptrReassemblyMgmtCfg);

/* LUT Services */
extern Netfp_LUTInfo* Netfp_allocIPLutEntry(Netfp_ServerMCB* ptrNetfpServer, Netfp_IPLutCfg* ptrIPLutCfg,
                                     uint8_t isIPSECChannel, int32_t* errCode);
extern int32_t Netfp_freeIPLutEntry(Netfp_ServerMCB* ptrNetfpServer, Netfp_LUTInfo* ptrLutInfo);

/* Debug Functions: */
extern void Netfp_xdump(uint8_t*  cp, int  length, const char*  prefix );

/**********************************************************************
 ******************** Realm Internal EXPORTED API *********************
 **********************************************************************/

/* Realm Specific Exported functions: */
extern uint32_t Netfp_getUniqueId(void); //fzm
extern void Netfp_cycleDelay(int32_t const count);

#ifdef __cplusplus
}
#endif

#endif /* __NETFP_INTERNAL_H__ */
