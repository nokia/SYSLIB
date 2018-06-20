/**
 *   @file  netfp_master.h
 *
 *   @brief
 *      NETFP Master exported header file
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2013-2104 Texas Instruments, Inc.
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
#ifndef __NETFP_MASTER__H__
#define __NETFP_MASTER__H__

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
#include <pthread.h>
#include <limits.h>

#include <ti/runtime/netfp/netfp.h>

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
#define NETFP_MASTER_MAX_CHAR           32

/**
 * @brief   Unix socket name associated with the NETFP master.
 */
#define NETFP_MASTER_SOCKET_NAME        "NetfpMaster"

/**
 * @brief   Maximum number of physical interfaces supported by the NETFP Master
 */
#define NETFP_MASTER_MAX_INTERFACE      8

/**
@}
*/

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
#define NETFP_MASTER_EINVAL                 SYSLIB_ERRNO_NETFP_MASTER_BASE-1

/**
 * @brief   Error Code: Out of memory
 */
#define NETFP_MASTER_ENOMEM                 SYSLIB_ERRNO_NETFP_MASTER_BASE-2

/**
 * @brief   Error Code: Entity is not found
 */
#define NETFP_MASTER_ENOTFOUND              SYSLIB_ERRNO_NETFP_MASTER_BASE-3

/**
 * @brief   Error Code: Internal error
 */
#define NETFP_MASTER_EINTERNAL              SYSLIB_ERRNO_NETFP_MASTER_BASE-4

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
    Netfp_Direction     direction;

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
    Netfp_Direction     direction;

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
    char                rmRegionName[NETFP_MAX_CHAR];

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
    char                ingressIfName[NETFP_MAX_CHAR];

    /**
     * @brief   Ethernet rule destination. It can be HOST or EMAC
     */

    Netfp_EthRuleDst    dstType;

    /**
     * @brief   Egress Physical Ethernet Interface name.
     */
    char                egressIfName[NETFP_MAX_CHAR];

    /**
     * @brief   Number of User stats associated with this rule. Support both bytes stats and
     * packets stats. Only support up to 2 stats per rule
     */
    uint32_t            numUserStats;

    /**
     * @brief   User Statistics configuration for the Ethernet Rule
     */
    Netfp_UserStatsCfg  userStatsCfg[NETFP_MAX_USR_STATS_PER_RULE];
}NetfpMaster_addEthRuleRequest;

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
    Netfp_EthRuleHandle      ethRuleHandle;
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
    Netfp_EthRuleHandle      ethRuleHandle;
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
        paSysStats_t                        netcpStats;

        /**
         * @brief   Reassembly statistics populated in response to the GET statistics request
         */
        Netfp_ReassemblyStats               reassemblyStats;

        /**
         * @brief   List of the interface names registered with the NETFP Master populated
         * in response to the GET Interface list request. This is a NULL terminated list
         */
        char                                ifName[NETFP_MASTER_MAX_INTERFACE][NETFP_MASTER_MAX_CHAR];

        /**
         * @brief  Add Ethernet Rule response - handle to the ethernet rule
         */
        Netfp_EthRuleHandle                 ethRuleHandle;

        /**
         * @brief  Response to get Interface Based Routing info
         */
        NetfpMaster_GetIfRtInfoResponse     ifBasedRouteInfo;

        /**
         * @brief   IPSec NAT-T UDP Encapsulation Settings
         */
        Netfp_NattCfg                       nattCfg;

        /**
         * @brief   User statistics configuration
         */
        Netfp_SysUserStatCfg                sysUserStatCfg;

        /**
         * @brief   User statistics associated with the Ethernet rule.
         */
        Netfp_UserStats                      stats;

        /**
         * @brief  Response to get Frame Protocol CRC Offload settings
         */
        NetfpMaster_GetFrameProtoCrcResponse frameProtoInfo;
    }u;
}NetfpMaster_Response;

/**
@}
*/

#endif /* __NETFP_MASTER__H__ */
