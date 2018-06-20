//***************************************************************************************
//
// Copyright 2016 Nokia, All Rights Reserved
//
//***************************************************************************************

#ifndef __NETFP_MASTER_PHY_PROXY_INTERFACE_H__
#define __NETFP_MASTER_PHY_PROXY_INTERFACE_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Standard Include Files. */
#include <stdint.h>

/**
@defgroup NETFP_MASTER_SYMBOL  NETFP Master Defined Symbols
@ingroup NETFP_MASTER_LIB_API
*/
/**
@defgroup NETFP_MASTER_FUNCTION  NETFP Master  Exported Functions
@ingroup NETFP_MASTER_LIB_API
*/
/**
@defgroup NETFP_MASTER_DATA_STRUCTURE  NETFP Master  Data Structures
@ingroup NETFP_MASTER_LIB_API
*/
/**
@defgroup NETFP_MASTER_ENUM  NETFP Master  Enumerations
@ingroup NETFP_MASTER_LIB_API
*/
/**
@defgroup NETFP_MASTER_INTERNAL_ENUM  NETFP Master Internal Enumerations
@ingroup NETFP_MASTER_LIB_API
*/
/**
@defgroup NETFP_MASTER_INTERNAL_DATA_STRUCTURE  NETFP Master Internal Data Structure
@ingroup NETFP_MASTER_LIB_API
*/
/**
@defgroup NETFP_MASTER_INTERNAL_FUNCTION  NETFP Master Internal Functions
@ingroup NETFP_MASTER_LIB_API
*/
/**
@defgroup NETFP_MASTER_ERROR_CODE  NETFP Master Error code
@ingroup NETFP_MASTER_LIB_API
*/

/**
@addtogroup NETFP_MASTER_SYMBOL
@{
*/

/**
 * @brief   Maximum number of characters
 */
#define NETFP_MASTER_MAX_CHAR               32

/**
 * @brief   Unix socket name associated with the NETFP master.
 */
#define NETFP_MASTER_SOCKET_NAME            "NetfpMaster"

/**
 * @brief   Maximum number of physical interfaces supported by the NETFP Master
 */
#define NETFP_MASTER_MAX_INTERFACE          8

/**
 * @brief   Maximum number of user stats can be added for LUT rules
 */
#define NETFP_MASTER_MAX_USR_STATS_PER_RULE 2

/**
@}
*/

/**
 * @brief   Base for NETFP Master error codes
 */
#define NETFP_MASTER_SYSLIB_ERRNO_BASE      -14000

/** @addtogroup NETFP_MASTER_ERROR_CODE
 *
 * @brief
 *  Base error code for the NETFP master module is defined in the
 *  \include ti/runtime/common/syslib.h
 *
 @{ */

/**
 * @brief   Error Code: Invalid argument
 */
#define NETFP_MASTER_EINVAL                 NETFP_MASTER_SYSLIB_ERRNO_BASE-1

/**
 * @brief   Error Code: Out of memory
 */
#define NETFP_MASTER_ENOMEM                 NETFP_MASTER_SYSLIB_ERRNO_BASE-2

/**
 * @brief   Error Code: Entity is not found
 */
#define NETFP_MASTER_ENOTFOUND              NETFP_MASTER_SYSLIB_ERRNO_BASE-3

/**
 * @brief   Error Code: Internal error
 */
#define NETFP_MASTER_EINTERNAL              NETFP_MASTER_SYSLIB_ERRNO_BASE-4

/**
@}
*/

/**
@addtogroup NETFP_MASTER_ENUM
@{
*/

/**
 * @brief
 *  Message types
 *
 * @details
 *  Enumeration which describes the various messages which can be exchanged
 *  with the NETFP master
 */
typedef enum NetfpMaster_MessageType
{
    /**
     * @brief   Register for notifications request issued to the NETFP master
     * to indicate that the requesting entity is interested in receiving updates
     * from the NETFP master
     */
    NetfpMaster_MessageType_REGISTER_NOTIFICATION    = 0x1,

    /**
     * @brief   Deregister for notifications issued to the NETFP master to indicate
     * that the requesting entity is no longer interested in receiving updates from
     * the NETFP master
     */
    NetfpMaster_MessageType_DEREGISTER_NOTIFICATION,

    /**
     * @brief   Get Interface list request issued to the NETFP master to get a list of
     * all the interfaces registered with the NETFP master
     */
    NetfpMaster_MessageType_GET_INTERFACE_LIST,

    /**
     * @brief   Get Interface request issued to the NETFP master to get the
     * properties for a specific interface
     */
    NetfpMaster_MessageType_GET_INTERFACE_REQUEST,

    /**
     * @brief   Get NETCP Statistics issued to the NETFP Master to get the global NETCP
     * statistics
     */
    NetfpMaster_MessageType_GET_NETCP_STATS,

    /**
     * @brief   Message can be used to rename the interface. Interface renaming is *NOT*
     * possible after the NETFP Clients are actively using the NETFP system to send/receive
     * data.
     */
    NetfpMaster_MessageType_RENAME_INTERFACE,

    /**
     * @brief   Message can be used to modify and update the Inner to Outer DSCP map
     */
    NetfpMaster_MessageType_SET_INNER_OUTER_DSCP_MAP,

    /**
     * @brief  Message can be used to set the routing mode for a particular physical
     * port. Routing modes can be either "dscp" or "dp-bit"
     */
    NetfpMaster_MessageType_SET_ROUTING_MODE,

    /**
     * @brief  Message can be used to set the default host priority
     */
    NetfpMaster_MessageType_SET_DEFAULT_HOST_PRIORITY,

    /**
     * @brief  Message can be used to set the default forwarding priority for a specific
     * physical interface
     */
    NetfpMaster_MessageType_SET_DEFAULT_FWD_PRIORITY,

    /**
     * @brief  Message can be used to modify the DSCP mapping to L2 QOS channel
     */
    NetfpMaster_MessageType_SET_DSCP_MAPPING,

    /**
     * @brief  Message can be used to modify the Pbit mapping to L2 QOS channel
     */
    NetfpMaster_MessageType_SET_VLAN_MAPPING,

    /**
     * @brief   Get statistics request issued to the NETFP master to
     * get the reassembly statistics
     */
    NetfpMaster_MessageType_GET_REASSEMBLY_STATS_REQUEST,

    /**
     * @brief   Set Port mirror request which is issued to the NETFP master
     * to enable/disable the port mirroring feature.
     *
     * @sa NetfpMaster_SetPortMirrorRequest
     */
    NetfpMaster_MessageType_SET_PORT_MIRROR_REQUEST,

    /**
     * @brief   Set Port mirror request which is issued to the NETFP master
     * to enable/disable the port mirroring feature.
     *
     * @sa NetfpMaster_SetPortCaptureRequest
     */
    NetfpMaster_MessageType_SET_PORT_CAPTURE_REQUEST,

    /**
     * @brief   The update notice is sent from the NETFP master to the registered
     * entities to indicate that the specified interface marking map had been modified
     * The NETFP master will automatically generate this event after a SET Interface
     * MAP request has been sent out
     *
     * NOTE: No response needs to be generated for this message
     *
     * @sa NetfpMaster_UpdateIfNotice
     */
    NetfpMaster_MessageType_UPDATE_NOTICE,

    /**
     * @brief  Add Ethernet Rule which is issued to the NETFP master to
     * add ethernet entries in  LUT1-0 for the given configuration.
     */
    NetfpMaster_MessageType_ADD_ETHERNET_RULE,

    /**
     * @brief  Delete Ethernet Rule which is issued to the NETFP master to
     * delete ethernet entries in  LUT1-0 for the given ethernet rule handle.
     */
    NetfpMaster_MessageType_DEL_ETHERNET_RULE,

    /**
     * @brief  Request Ethernet Rules user stats
     */
    NetfpMaster_MessageType_GET_ETHERNET_RULE_STATS,

    /**
     * @brief  Display Ethernet Rules which are added in NETFP master
     */
    NetfpMaster_MessageType_DISPLAY_ETHERNET_RULE,

    /**
     * @brief  Get interface based routing info
     */
    NetfpMaster_MessageType_GET_INTERFACE_ROUTING_INFO,

    /**
     * @brief  Get NAT-T configuration
     */
    NetfpMaster_MessageType_GET_NATT_SETTINGS_REQUEST,

    /**
     * @brief  Get user stats configuration
     */
    NetfpMaster_MessageType_GET_USRSTATS_SETTINGS_REQUEST,

    /**
     * @brief  Set the PRIORITY Override mode for the specific interface. This is valid
     * only if the routing mode is set to DP-Bit.
     */
    NetfpMaster_MessageType_SET_PRIORITY_OVERRIDE,

    /**
     * @brief  Set the broadcast/multicast preclassification configuration.
     *
     * @sa  NetfpMaster_SetPreclassficiation
     */
    NetfpMaster_MessageType_SET_PRECLASSIFICATION,

    /**
     * @brief  Get Frame Protocol CRC Offload settings
     */
    NetfpMaster_MessageType_GET_FRAME_PROTO_CRC_SETTINGS_REQUEST,

    /**
     * @brief   Response packet generated for any of the above requests.
     */
    NetfpMaster_MessageType_RESPONSE
}NetfpMaster_MessageType;

/**
@}
*/

/**
@addtogroup NETFP_MASTER_DATA_STRUCTURE
@{
*/

/**
 * @brief
 *  Get interface request
 *
 * @details
 *  The structure is populated by the application and sent to the master
 *  to get the interface properties
 */
typedef struct NetfpMaster_GetIfRequest
{
    /**
     * @brief   Interface name for which the properties are requested
     */
    char    ifName[NETFP_MASTER_MAX_CHAR];
}NetfpMaster_GetIfRequest;

/**
 * @brief
 *  Get interface request
 *
 * @details
 *  The structure is populated by the master and sent back to the application
 *  in response to the get request
 */
typedef struct NetfpMaster_GetIfResponse
{
    /**
     * @brief   Switch port number associated with the physical interface
     */
    uint32_t    switchPort;

    /**
     * @brief   This specifies the mapping between the inner DSCP to outer DSCP
     */
    uint8_t     innerToOuterDSCPMap[64];
}NetfpMaster_GetIfResponse;

/**
 * @brief
 *  Rename Interface
 *
 * @details
 *  The structure is populated by the application to rename the interface
 *  to use a new name
 */
typedef struct NetfpMaster_RenameInterface
{
    /**
     * @brief   Old interface name
     */
    char            oldIfName[NETFP_MASTER_MAX_CHAR];

    /**
     * @brief   New interface name
     */
    char            newIfName[NETFP_MASTER_MAX_CHAR];
}NetfpMaster_RenameInterface;

/**
 * @brief
 *  Set interface request
 *
 * @details
 *  The structure is populated by the application and sent to the master
 *  to set the inner to outer DSCP mapping
 */
typedef struct NetfpMaster_SetInnerOuterDSCPMap
{
    /**
     * @brief   Interface name for which the properties are being configured
     */
    char            ifName[NETFP_MASTER_MAX_CHAR];

    /**
     * @brief   Inner DSCP for which the DSCP markings are to be modified
     */
    uint8_t         innerDSCP;

    /**
     * @brief   New outer DSCP value which is to be used.
     */
    uint8_t         outerDSCP;
}NetfpMaster_SetInnerOuterDSCPMap;

/**
 * @brief
 *  Set Preclassification status
 *
 * @details
 *  The structure is populated by the application and sent to the master
 *  to set the preclassification status
 */
typedef struct NetfpMaster_SetPreclassficiation
{
    /**
     * @brief   Interface name for which the preclassification is to be configured
     */
    char            ifName[NETFP_MASTER_MAX_CHAR];

    /**
     * @brief   Set to 1 to enable broadcast preclassification. Set to 0 for multicast
     * preclassification
     */
    uint32_t        isBroadcast;

    /**
     * @brief   Status of the preclassification: Set to 1 to enable and 0 to disable
     */
    uint32_t        enablePreclassfication;

    /**
     * @brief   Flow identifier associated with the preclassification
     */
    uint32_t        preclassificationFlowId;

    /**
     * @brief   Queue identifier associated with the preclassification
     */
    uint32_t        preclassificationQueueId;
}NetfpMaster_SetPreclassficiation;

/**
 * @brief
 *  Enumeration for the direction
 *
 * @details
 *  Enumeration the SA, SP, Fast path, ciphering direction.
 */
typedef enum NetfpMaster_Direction
{
    /**
     * @brief   Ingress for packets coming from EPC to eNodeB
     */
    NetfpMaster_Direction_INBOUND     = 1,

    /**
     * @brief    Egress for packets going from eNodeB to EPC
     */
    NetfpMaster_Direction_OUTBOUND    = 2
}NetfpMaster_Direction;

/**
 * @brief
 *  Set port mirror request
 *
 * @details
 *  The structure is populated by the application and sent to the master
 *  to enable/disable the port mirror request
 */
typedef struct NetfpMaster_SetPortMirrorRequest
{
    /**
     * @brief   Status which specifies if the port mirroring needs to be enabled/disabled
     */
    uint32_t            isEnable;

    /**
     * @brief   Direction in which the port mirror should work.
     */
    NetfpMaster_Direction     direction;

    /**
     * @brief   Source switch port. Packets from this switch port will be mirrored
     */
    uint8_t             srcPort;

    /**
     * @brief   Destination port. Packets from the source port will be mirrored onto this port
     */
    uint8_t             dstPort;
}NetfpMaster_SetPortMirrorRequest;

/**
 * @brief
 *  Set port capture request
 *
 * @details
 *  The structure is populated by the application and sent to the master
 *  to enable/disable the port capture request
 */
typedef struct NetfpMaster_SetPortCaptureRequest
{
    /**
     * @brief   Status which specifies if the port capture needs to be enabled/disabled
     */
    uint32_t            isEnable;

    /**
     * @brief   Direction in which the port capture should work. This always needs to be
     * specified
     */
    NetfpMaster_Direction     direction;

    /**
     * @brief   Switch Port number on which the packets need to be captured. This always
     * needs to be specified.
     */
    uint8_t             portToBeCaptured;

    /**
     * @brief  Free queue identifier from where the NETCP will pick up packets and will then
     * place these packets into the destination queue. This needs to be specified only if
     * port capturing is being enabled.
     */
    uint16_t            freeQueueId;

    /**
     * @brief   Destination queue identifier in which the captured packets will be placed.
     * This needs to be specified only if port capturing is being enabled.
     */
    uint16_t            dstQueue;

    /**
     * @brief   Software information to be added to the packet. This needs to be specified only if
     * port capturing is being enabled.
     */
    uint32_t            swInfo;
}NetfpMaster_SetPortCaptureRequest;

/**
 * @brief
 *  Set default host priority
 *
 * @details
 *  The structure is populated by the application and sent to the master to modify the default
 *  host priority for the entire system.
 */
typedef struct NetfpMaster_SetDefaultHostPriority
{
    /**
     * @brief   New default Host priority which is to be configured
     */
    uint8_t     defaultHostPriority;
}NetfpMaster_SetDefaultHostPriority;

/**
 * @brief
 *  Set default forwarding priority
 *
 * @details
 *  The structure is populated by the application and sent to the master to modify the default
 *  forwarding priority for each physical interface
 */
typedef struct NetfpMaster_SetDefaultFwdPriority
{
    /**
     * @brief   New default forwarding priority for all non-ip packets
     */
    uint8_t     defaultForwardPriority;

    /**
     * @brief   Name of the physical interface for which the routing mode is to be modified
     */
    char        ifName[NETFP_MASTER_MAX_CHAR];
}NetfpMaster_SetDefaultFwdPriority;

/**
 * @brief
 *  Set Routing mode
 *
 * @details
 *  The structure is populated by the application and sent to the master to modify the routing
 *  mode for an interface.
 */
typedef struct NetfpMaster_SetRoutingMode
{
    /**
     * @brief   Name of the physical interface for which the routing mode is to be modified
     */
    char        ifName[NETFP_MASTER_MAX_CHAR];

    /**
     * @brief   Routing mode can be either of the following "dscp" or "dp-bit". Any other
     * value will result in an error.
     */
    char        routingMode[NETFP_MASTER_MAX_CHAR];
}NetfpMaster_SetRoutingMode;

/**
 * @brief
 *  Set Priority override
 *
 * @details
 *  The structure is populated by the application and sent to the master
 *  to enable/disable the priority override.
 */
typedef struct NetfpMaster_SetPriorityOverride
{
    /**
     * @brief   Name of the physical interface for which the priority mode is configured.
     */
    char        ifName[NETFP_MASTER_MAX_CHAR];

    /**
     * @brief   Set to 1 to enable the priority override else set to 0.
     */
    uint32_t    priorityOverride;
}NetfpMaster_SetPriorityOverride;

/**
 * @brief
 *  DSCP Map
 *
 * @details
 *  The structure is populated by the applications and sent to the NETFP master to modify the
 *  DSCP to L2 QOS Mapping
 */
typedef struct NetfpMaster_SetDSCPMap
{
    /**
     * @brief   Name of the physical interface
     */
    char                ifName[NETFP_MASTER_MAX_CHAR];

    /**
     * @brief   DSCP for which the mapping is being changed
     */
    uint8_t             dscp;

    /**
     * @brief   New queue offset which is to be used.
     */
    uint8_t             queueOffset;
}NetfpMaster_SetDSCPMap;

/**
 * @brief
 *  VLAN Map
 *
 * @details
 *  The structure is populated by the applications and sent to the NETFP master to modify the
 *  VLAN Priority bits to L2 QOS Mapping
 */
typedef struct NetfpMaster_SetVLANMap
{
    /**
     * @brief   Name of the physical interface
     */
    char                ifName[NETFP_MASTER_MAX_CHAR];

    /**
     * @brief   Pbit for which the mapping is being changed
     */
    uint8_t             pbit;

    /**
     * @brief   New queue offset which is to be used.
     */
    uint8_t             queueOffset;
}NetfpMaster_SetVLANMap;

/**
 * @brief
 *  Update interface notice
 *
 * @details
 *  The structure is populated by the NETFP master and sent to all the entities which
 *  have registered for notifications.
 */
typedef struct NetfpMaster_UpdateIfNotice
{
    /**
     * @brief   Interface name for which the update interface notice is being propogated.
     */
    char    ifName[NETFP_MASTER_MAX_CHAR];

    /**
     * @brief   New Interface name to be used from this point on. The new & old interface
     * name will be different only if there is an interface rename command. In the case of
     * all other update messages; the 2 names will always be same.
     */
    char    newIfName[NETFP_MASTER_MAX_CHAR];
}NetfpMaster_UpdateIfNotice;

/**
 * @brief
 *  Destination for the Ethernet Rule programmed in LUT 1-0
 *
 * @details
 *  Enumeration describes the allowed destination for Ethernet Rules programmed
 *  in LUT 1-0.
 */
typedef enum NetfpMaster_EthRuleDst
{
    /**
     * @brief    Destination of the Ethernet Rule is EMAC
     */
    NetfpMaster_EthRuleDst_EMAC    = 0,

    /**
     * @brief    Destination of the Ethernet Rule is continue parsing
     */
    NetfpMaster_EthRuleDst_CONTINUE,

    /**
     * @brief    Destination of the Ethernet Rule is Linux interface
     */
    NetfpMaster_EthRuleDst_HOST,

    /**
     * @brief    Destination of the Ethernet Rule is discard
     */
    NetfpMaster_EthRuleDst_DISCARD
}NetfpMaster_EthRuleDst;

/**
 * @brief
 *  NETFP User Counter Length
 *
 * @details
 *  This is the User counter Length used for PA user statistics.
 */
typedef enum NetfpMaster_UserStatsLen
{
    /**
     * @brief   32bit counter
     */
    NetfpMaster_UserStatsLen_32b,

    /**
     * @brief   64bit counter
     */
    NetfpMaster_UserStatsLen_64b
}NetfpMaster_UserStatsLen;

/**
 *  @defgroup paUsrStatsTypes PA User-defined Ststaistics Counter Types
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name User-defined Ststaistics Counter Types
 *
 *  Definition of Counter types of the User-defined Statistics
 */
/** @ingroup paUsrStatsTypes */
/*@{*/
typedef enum {
  pa_USR_STATS_TYPE_PACKET = 0,   /**< Packet Counter */
  pa_USR_STATS_TYPE_BYTE,         /**< Byte Counter */
  pa_USR_STATS_TYPE_DISABLE       /**< Counter to be disabled */
} NetfpMaster_paUsrStatsTypes_e;
/*@}*/

/**
 * @brief
 *  Ethernet Rule User Stats Configuration
 *
 * @details
 *  This is the User stats configuration for an Ethernet Rule.
 */
typedef struct NetfpMaster_UserStatsCfg
{
    /**
     * @brief   User Stats counter type 32bit or 64bit
     */
     NetfpMaster_UserStatsLen  userStatsLen;

    /**
     * @brief   User Stats type packet or byte
     */
    NetfpMaster_paUsrStatsTypes_e    userStatsType;
}NetfpMaster_UserStatsCfg;

/**
 * @brief
 *  Ethernet rule add request
 *
 * @details
 *  The structure is populated by the application and sent to the master to program
 *  PA LUT 1-0 with Ethernet Rule configuration.
 */
typedef struct NetfpMaster_addEthRuleRequest
{
    /**
     * @brief   Region Name used to allocate LUT1-0 index. LUT1-0 regions
     * and indices are defined in dts file.
     */
    char                rmRegionName[NETFP_MASTER_MAX_CHAR];

    /**
     * @brief   type of the Ethernet interface .
     */
    uint16_t            ethType;

    /**
     * @brief   VLAN identifier if the Ethermet type is VLAN .
     */
    uint16_t            vlanId;

    /**
     * @brief   A flag when set to 1 indicates that vlan field will not be match
     * with the given vlan Id. When it is set to 0, vlanId field will be matched
     * for LUT10 rule.
     */
    uint32_t            anyVlanId;

    /**
     * @brief   Source MAC address to be matched in the cascading entry.
     */
    uint8_t             srcMacAddress[6];

    /**
     * @brief   Destination MAC address to be matched in the cascading entry .
     */
    uint8_t             dstMacAddress[6];

    /**
     * @brief   Ingress Physical Ethernet Interface name.
     */
    char                ingressIfName[NETFP_MASTER_MAX_CHAR];

    /**
     * @brief   Ethernet rule destination. It can be HOST or EMAC
     */

    NetfpMaster_EthRuleDst    dstType;

    /**
     * @brief   Egress Physical Ethernet Interface name.
     */
    char                egressIfName[NETFP_MASTER_MAX_CHAR];

    /**
     * @brief   Number of User stats associated with this rule. Support both bytes stats and
     * packets stats. Only support up to 2 stats per rule
     */
    uint32_t            numUserStats;

    /**
     * @brief   User Statistics configuration for the Ethernet Rule
     */
    NetfpMaster_UserStatsCfg  userStatsCfg[NETFP_MASTER_MAX_USR_STATS_PER_RULE];
}NetfpMaster_addEthRuleRequest;

/**
 * @brief
 *  This is an opaque handle to the NETFP Ethernet Rule entry.
 */
typedef void*   NetfpMaster_EthRuleHandle;

/**
 * @brief
 *  Ethernet rule delete request
 *
 * @details
 *  The structure is populated by the application and sent to the master to delete
 *  Ethernet Rule with the handle specified.
 */
typedef struct NetfpMaster_delEthRuleRequest
{
    /**
     * @brief   Handle of Ethernet rules to be deleted
     */
    NetfpMaster_EthRuleHandle      ethRuleHandle;
}NetfpMaster_delEthRuleRequest;

/**
 * @brief
 *  Ethernet rule statistics request
 *
 * @details
 *  The structure is populated by the application and sent to the master to get the
 *  statistics associated with an ethernet rule
 */
typedef struct NetfpMaster_getEthRuleStats
{
    /**
     * @brief   Handle to the ethernet rule for which the stats are requested
     */
    NetfpMaster_EthRuleHandle      ethRuleHandle;
}NetfpMaster_getEthRuleStats;

/**
 * @brief
 *  Request structure
 *
 * @details
 *  This is the request structure populated by the application and passed to the
 *  NETFP master.
 */
typedef struct NetfpMaster_Request
{
    /**
     * @brief   Request message type
     */
    NetfpMaster_MessageType     msgType;

    /**
     * @brief   Unique transaction identifier associated with the request. This can
     * be matched to the corresponding response.
     */
    uint32_t                    id;

    /**
     * @brief   Union which is populated by the application depending upon the request
     * message type
     */
    union
    {
        /**
         * @brief   Request populated by the application to get the properties associated
         * with the interface
         */
        NetfpMaster_GetIfRequest            getIfRequest;

        /**
         * @brief   Request populated by the application to rename the interface
         */
        NetfpMaster_RenameInterface         renameInterface;

        /**
         * @brief   Request populated by the application to change the Inner to Outer DSCP
         * mapping
         */
        NetfpMaster_SetInnerOuterDSCPMap    setInnerOuterDSCPMap;

        /**
         * @brief   Request populated by the application to setup the preclassification
         */
        NetfpMaster_SetPreclassficiation    setPreclassification;

        /**
         * @brief   Request populated by the application to change the QOS routing mode
         * for a particular physical interface.
         */
        NetfpMaster_SetRoutingMode          setRoutingMode;

        /**
         * @brief   Request populated by the application to change the QOS priority override
         * for a particular physical interface. This is valid only for DP-Bit mode
         */
        NetfpMaster_SetPriorityOverride     setPriorityOverride;

        /**
         * @brief   Request populated by the application to change the global default host
         * priority
         */
        NetfpMaster_SetDefaultHostPriority  setDefaultHostPriority;

        /**
         * @brief   Request populated by the application to change the default host forwarding
         * priority per physical interface
         */
        NetfpMaster_SetDefaultFwdPriority   setDefaultFwdPriority;

        /**
         * @brief   Request populated by the application to change the DSCP to L2 QOS Mapping
         */
        NetfpMaster_SetDSCPMap              setDSCPMap;

        /**
         * @brief   Request populated by the application to change the VLAN to L2 QOS Mapping
         */
        NetfpMaster_SetVLANMap              setVLANMap;

        /**
         * @brief   Request is populated by the application for the enable/disable the port
         * mirroring feature
         */
        NetfpMaster_SetPortMirrorRequest    setPortMirrorRequest;

        /**
         * @brief   Request is populated by the application for the enable/disable the port
         * capturing feature
         */
        NetfpMaster_SetPortCaptureRequest   setPortCaptureRequest;

        /**
         * @brief   Update interface notice request populated by the NETFP master for the UPDATE
         * Notice request
         */
        NetfpMaster_UpdateIfNotice          updateIfNotice;

        /**
         * @brief   Deleting Ethernet Rule request which is populated by application.
         */
        NetfpMaster_addEthRuleRequest       addEthRuleRequest;

        /**
         * @brief   Deleting Ethernet Rule request which is populated by application.
         */
        NetfpMaster_delEthRuleRequest       delEthRuleRequest;

        /**
         * @brief   Request is populated by the application to get the Ethernet Rule statistics
         */
        NetfpMaster_getEthRuleStats         getEthRuleStats;
    }u;
}NetfpMaster_Request;

/**
 * @brief
 *  Response for interface based routing info request
 *
 * @details
 *  The structure is populated by the master and sent back to netfp server
 *  in response to the get interface based routing information request.
 */
typedef struct NetfpMaster_GetIfRtInfoResponse
{
    /**
     * @brief   Base Queue number used for interface based routing in PA
     */
    uint32_t          baseQueue;

    /**
     * @brief   Base Flow identifier used for interface based routing in PA
     */
    uint32_t          baseFlowId;
}NetfpMaster_GetIfRtInfoResponse;

/**
 * @brief
 *  Get Frame Protocol CRC Offload Setting response
 *
 * @details
 *  The structure is populated by the master and sent back to netfp server
 *  in response to the get Frame Protocol CRC Offload Setting request.
 */
typedef struct NetfpMaster_GetFrameProtoCrcResponse
{
    /**
     * @brief   Indicates support for offloading of verfication and computation of Frame Protocol CRC
     */
    uint32_t        frameProtoCrcOffload;
}NetfpMaster_GetFrameProtoCrcResponse;

/**
 * @brief
 *  Ethernet rule response
 *
 * @details
 *  The structure is populated by the master and sent back to NETFP Server
 *  in response to the get ethernet rule statistics
 */
typedef struct NetfpMaster_GetEthRuleStatsResp
{
    /**
     * @brief   Indicates support for offloading of verfication and computation of Frame Protocol CRC
     */
    ;
}NetfpMaster_GetEthRuleStatsResp;

/**
 * @brief
 *  Reassembly statistics
 *
 * @details
 *  This is the NETFP client reassembly statistics
 */
typedef struct NetfpMaster_ReassemblyStats
{
    /**
     * @brief   Number of active reassembly contexts which exist in the
     * clients reassembly module at that instant of time.
     */
    uint32_t        activeReassemblyContexts;

    /**
     * @brief   Number of IP packets received on the outer IP reassembly channel
     * These packets could be fragments or non-fragments which have been passed to
     * the host for ordering.
     */
    uint32_t        numOuterIPPktReceived;

    /**
     * @brief   Number of IP packets received on the inner IP reassembly channel
     * These packets could be fragments or non-fragments which have been passed to
     * the host for ordering.
     */
    uint32_t        numInnerIPPktReceived;

    /**
     * @brief   Number of *active* fragments. This is the sum total of all
     * the fragments in all the active reassembly contexts.
     */
    uint32_t        numActiveFragments;

    /**
     * @brief   Number of fragmented packets which are passed from the NETCP
     * to the NETFP client but these packets are not associated with a valid
     * traffic flow and so there could be packet reordering in these cases.
     */
    uint32_t        nonAccleratedTrafficFlow;

    /**
     * @brief   Number of IPv4 fragments received
     */
    uint32_t        numIPv4Fragments;

    /**
     * @brief   Number of IPv4 fragments received
     */
    uint32_t        numIPv6Fragments;

    /**
     * @brief   Number of reassembled packets
     */
    uint32_t        numReassembledPackets;

    /**
     * @brief   Number of reassembly timeouts
     */
    uint32_t        numReassemblyTimeout;

    /**
     * @brief   Number of duplicated fragments
     */
    uint32_t        numDuplicatedFragment;

    /**
     * @brief   Number of overlapping fragments
     */
    uint32_t        numOverlappingFragment;

    /**
     * @brief   Number of IPv4 fragments dropped because of header errors.
     */
    uint32_t        numIPv4HeaderError;

    /**
     * @brief   Number of IPv6 fragments dropped because of header errors
     */
    uint32_t        numIPv6HeaderError;

    /**
     * @brief   Number of fragments received on the outer IP channel which rejected using the application
     * supplied hook
     */
    uint32_t        numPreOuterFragmentsRejected;

    /**
     * @brief   Number of fragments received on the inner IP channel which rejected using the application
     * supplied hook
     */
    uint32_t        numPreInnerFragmentsRejected;

    /**
     * @brief   Number of packets rejected after reassembly using the application supplied hook.
     * The fragments were received on the outer IP channel
     */
    uint32_t        numPostOuterReassembledPktsRejected;

    /**
     * @brief   Number of packets rejected after reassembly using the application supplied hook.
     * The fragments were received on the inner IP channel
     */
    uint32_t        numPostInnerReassembledPktsRejected;

    /**
     * @brief   Number of large packets which exceed the NETFP_PASS_MAX_BUFFER_SIZE
     * These packets after reassembly cannot be passed to the NETCP subsystem for
     * further classification and so they are sent to the NETFP client "Large Packet
     * Channel" if one is configured.
     */
    uint32_t        numLargePackets;

    /**
     * @brief   Number of times the PKTLIB reassembly heap was starved
     */
    uint8_t         reassemblyHeapStarvationCounter;

    /**
     * @brief   Number of traffic flows which were not deleted from the NETCP subsystem.
     * This error counter indicates that the number of packets which were allocated in
     * the Reassembly configuration was less *OR* implies that there is a memory leak
     * in the *internal* reassembly heap. This would result in the NETCP running out
     * of traffic flows and resulting in out of order packets.
     */
    uint32_t        numFreeTrafficFlowFailure;

    /**
     * @brief   Number of fragments dropped because there the reassembly module ran out
     * of reassembly contexts
     */
    uint32_t        noReassemblyContext;

    /**
     * @brief   This is the number of times the default reassembly management module was
     * invoked because the number of active fragments exceeded the upper threshold.
     */
    uint32_t        numDefaultReassemblyMgmtInvocations;

    /**
     * @brief   This is the number of fragments dropped and discarded by the default reassembly
     * management module.
     */
    uint32_t        numDefaultReassemblyMgmtDroppedFragments;

     /**
     * @brief   This is the number of fragments that have
     *          inconsistent information between first and any
     *          succeeding fragment
     *
     */
    // fzm
    // According to https://e2eprivate.ti.com/nokia_siemens_networks/k2_-_fsm4_-_fzm_-_lrc_nokia/f/191/p/5346/28187.aspx
    // request to TI was crated to add below extra counter for new drop scenario
    uint32_t        numInconsistentInfoDroppedFragments;

}NetfpMaster_ReassemblyStats;

/**
 * @brief
 *  NATT configuration on Netfp Server
 *
 * @details
 *  Data structure defines NAT-T configurations from Netfp master
 */
typedef struct NetfpMaster_NattCfg
{
    /**
     * @brief   UDP port used for IPSec NAT-T UDP Encapsulation
     */
    uint32_t    udpPort;

    /**
     * @brief   NAT-T wildcarded LUT entry setting
     */
    uint32_t    wildCardedEntry;
}NetfpMaster_NattCfg;

/**
 * @brief
 *  NETCP System user stats configuration
 *
 * @details
 *  The structure defines the user statistics configuration which is used to
 *  configure the user statistics counters in the NETCP.
 */
typedef struct NetfpMaster_SysUserStatCfg
{
    /**
     * @brief   Total number of user stats. It is the sum of 32b and 64b user stats
     */
    uint32_t    numTotalUserStats;

    /**
     * @brief   Total number of 64b user stats.
     */
    uint32_t    num64bUserStats;
}NetfpMaster_SysUserStatCfg;

/**
 * @brief
 *  User Stats
 *
 * @details
 *  The structure defines the user statistics associated which are linked
 *  to Ethernet or IP rules.
 */
typedef struct NetfpMaster_UserStats
{
    /**
     * @brief   User Stats counter type 32bit or 64bit
     */
     uint64_t   userStats[NETFP_MASTER_MAX_USR_STATS_PER_RULE];
}NetfpMaster_UserStats;

/**
 * @ingroup palld_api_structures
 * @brief PA Classify1 Statistics Structure
 *
 * @details This structures define the PA Classify1-specific statistics provided
 *          with API function @ref Pa_formatStatsReply ().
 */
typedef struct NetfpMaster_paClassify1Stats_s {

  uint32_t nPackets;                /**< Number of packets entering Classify1 PDSPs */
  uint32_t nIpv4Packets;            /**< Number of IPv4 packets */
  uint32_t nIpv4PacketsInner;       /**< Number of Inner IPv4 packets */
  uint32_t nIpv6Packets;            /**< Number of IPv6 packets */
  uint32_t nIpv6PacketsInner;       /**< Number of Inner IPv6 packets */
  uint32_t nCustomPackets;          /**< Number of custom LUT1 packets */
  uint32_t nSrioPackets;            /**< Number of SRIO packets */
  uint32_t nLlcSnapFail;            /**< Number of packets with corrupt LLC Snap */
  uint32_t nTableMatch;             /**< Number of packets with table match found */
  uint32_t nNoTableMatch;           /**< Number of packets without table match found */
  uint32_t nIpFrag;                 /**< Number of Ingress fragmented IP packets */
  uint32_t nIpDepthOverflow;        /**< Number of packets with too many IP layers */
  uint32_t nVlanDepthOverflow;      /**< Number of packets with too many VLANs */
  uint32_t nGreDepthOverflow;       /**< Number of packets with too many GREs */
  uint32_t nMplsPackets;            /**< Number of MPLS packets */
  uint32_t nParseFail;              /**< Number of packets which can not be parsed */
  uint32_t nInvalidIPv6Opt;         /**< Number of IPv6 packets which contains invalid IPv6 options */
  uint32_t nTxIpFrag;               /**< Number of Egress fragmented IP packets */
  uint32_t nSilentDiscard;          /**< Number of packets dropped */
  uint32_t nInvalidControl;         /**< Number of packet received with invalid control information */
  uint32_t nInvalidState;           /**< Number of times the PA detected an illegal state and recovered */
  uint32_t nSystemFail;             /**< Number of times the PA detected an unrecoverable state and restarted */

} NetfpMaster_paClassify1Stats_t;

/**
 * @ingroup palld_api_structures
 * @brief PA Classify2 Statistics Structure
 *
 * @details This structures define the PA Classify2-specific statistics provided
 *          with API function @ref Pa_formatStatsReply ().
 */
typedef struct NetfpMaster_paClassify2Stats_s  {

  uint32_t nPackets;                /**< Number of packets entering Classify2 PDSP */
  uint32_t nUdp;                    /**< Number of UDP packets */
  uint32_t nTcp;                    /**< Number of TCP packets */
  uint32_t nCustom;                 /**< Number of custom LUT2 packets */
  uint32_t reserved3;               /**< Reserved for future use */
  uint32_t reserved4;               /**< Reserved for future use */
  uint32_t nSilentDiscard;          /**< Number of packets dropped */
  uint32_t nInvalidControl;         /**< Number of packet received with invalid control information */

} NetfpMaster_paClassify2Stats_t;

/**
 * @ingroup palld_api_structures
 * @brief PA Modifier Statistics Structure
 *
 * @details This structures define the PA Modifier-specific statistics provided
 *          with API function @ref Pa_formatStatsReply ().
 */
typedef struct NetfpMaster_paModifyStats_s   {
  uint32_t nCommandFail;            /**< Number of invalid commands */

} NetfpMaster_paModifyStats_t;

/**
 * @ingroup palld_api_structures
 * @brief PA Common Statistics Structure
 *
 * @details This structures define the PA Common statistics provided
 *          with API function @ref Pa_formatStatsReply ().
 */
typedef struct NetfpMaster_paCommonStats_s  {

  uint32_t reserved5;               /**< Reserved for future use */

} NetfpMaster_paCommonStats_t;

/**
 * @ingroup palld_api_structures
 * @brief PA System Statistics Structure
 *
 * @details This structures define the PA System statistics provided
 *          with API function @ref Pa_formatStatsReply ().
 */

typedef struct NetfpMaster_paSysStats_s  {

  NetfpMaster_paClassify1Stats_t classify1;     /**< Classify1-specific statistics */
  NetfpMaster_paClassify2Stats_t classify2;     /**< Classify2-specific statistics */
  NetfpMaster_paModifyStats_t    modify;        /**< Modifier-specific statistics */
  NetfpMaster_paCommonStats_t    common;        /**< Common statistics */

} NetfpMaster_paSysStats_t;

/**
 * @brief
 *  Response structure
 *
 * @details
 *  This is the response structure populated by the NETFP master and sent back to the
 *  application in response to the request issues.
 */
typedef struct NetfpMaster_Response
{
    /**
     * @brief   Response Message type
     */
    NetfpMaster_MessageType     msgType;

    /**
     * @brief   Response messages are generated for a specific request.
     */
    NetfpMaster_MessageType     reqType;

    /**
     * @brief   Unique transaction identifier in the request for which the response
     * is generated.
     */
    uint32_t                    id;

    /**
     * @brief   Response status error code: 0 indicates that the request was successfully
     * processed and the response is valid. Negative values reflect the error code
     */
    int32_t                     errCode;

    /**
     * @brief   Union which is populated by the application depending upon the request
     * message type
     */
    union
    {
        /**
         * @brief   Get interface response populated in the response to the GET Interface
         * request
         */
        NetfpMaster_GetIfResponse           getIfResponse;

        /**
         * @brief   NETCP Statistics populated in response to the GET NETCP Statistics request
         */
        NetfpMaster_paSysStats_t                        netcpStats;

        /**
         * @brief   Reassembly statistics populated in response to the GET statistics request
         */
        NetfpMaster_ReassemblyStats               reassemblyStats;

        /**
         * @brief   List of the interface names registered with the NETFP Master populated
         * in response to the GET Interface list request. This is a NULL terminated list
         */
        char                                ifName[NETFP_MASTER_MAX_INTERFACE][NETFP_MASTER_MAX_CHAR];

        /**
         * @brief  Add Ethernet Rule response - handle to the ethernet rule
         */
        NetfpMaster_EthRuleHandle                 ethRuleHandle;

        /**
         * @brief  Response to get Interface Based Routing info
         */
        NetfpMaster_GetIfRtInfoResponse     ifBasedRouteInfo;

        /**
         * @brief   IPSec NAT-T UDP Encapsulation Settings
         */
        NetfpMaster_NattCfg                       nattCfg;

        /**
         * @brief   User statistics configuration
         */
        NetfpMaster_SysUserStatCfg                sysUserStatCfg;

        /**
         * @brief   User statistics associated with the Ethernet rule.
         */
        NetfpMaster_UserStats                      stats;

        /**
         * @brief  Response to get Frame Protocol CRC Offload settings
         */
        NetfpMaster_GetFrameProtoCrcResponse frameProtoInfo;
    }u;
}NetfpMaster_Response;

/**
@}
*/

#ifdef __cplusplus
}
#endif

#endif /* __NETFP_MASTER_PHY_PROXY_INTERFACE_H__ */
