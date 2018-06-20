/**
 *   @file  netfp.h
 *
 *   @brief
 *      Header file for the NETFP library. The file exposes the
 *      data structures and exported NETFP API which are available
 *      for use by applications.
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

/** @defgroup NETFP_LIB_API Network Fast Path
 */
#ifndef __NETFP_H__
#define __NETFP_H__

#include <stdio.h>

/* SYSLIB Common header file. */
#include <ti/runtime/common/syslib.h>

/* MCSDK Include files */
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/cppi/cppi_desc.h>
#include <ti/drv/pa/pa.h>
#include <ti/drv/pa/pasahost.h>
#include <ti/drv/rm/rm.h>
#include <ti/drv/rm/rm_services.h>
#include <ti/drv/sa/salld.h>

/* SYSLIB Include Files. */
#include <ti/runtime/common/syslib.h>
#include <ti/runtime/msgcom/msgcom.h>
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/name/name.h>
#include <ti/runtime/name/name_db.h>
#include <ti/runtime/name/name_proxyClient.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
@defgroup NETFP_SYMBOL  NETFP Defined Symbols
@ingroup NETFP_LIB_API
*/
/**
@defgroup NETFP_FUNCTION  NETFP Exported Functions
@ingroup NETFP_LIB_API
*/
/**
@defgroup NETFP_DATA_STRUCTURE  NETFP Data Structures
@ingroup NETFP_LIB_API
*/
/**
@defgroup NETFP_OSAL_API  NETFP OS Abstraction Layer
@ingroup NETFP_LIB_API
*/
/**
@defgroup NETFP_ENUM  NETFP Enumerations
@ingroup NETFP_LIB_API
*/
/**
@defgroup NETFP_INTERNAL_ENUM  NETFP Internal Enumerations
@ingroup NETFP_LIB_API
*/
/**
@defgroup NETFP_INTERNAL_DATA_STRUCTURE  NETFP Internal Data Structure
@ingroup NETFP_LIB_API
*/
/**
@defgroup NETFP_INTERNAL_FUNCTION  NETFP Internal Functions
@ingroup NETFP_LIB_API
*/
/**
@defgroup NETFP_DEVICE_FUNCTION  NETFP Device Functions
@ingroup NETFP_LIB_API
*/
/**
@defgroup NETFP_ERROR_CODE  NETFP Error code
@ingroup NETFP_LIB_API
*/
/**
@defgroup NETFP_INTERNAL_DSP_FUNCTIONS  NETFP Internal DSP Realm functions
@ingroup NETFP_LIB_API
*/

/** @addtogroup NETFP_ERROR_CODE
 *
 * @brief
 *  Base error code for the NETFP module is defined in the
 *  \include ti/runtime/common/syslib.h
 *
 @{ */

/**
 * @brief   Error Code: Out of memory
 */
#define NETFP_ENOMEM                        SYSLIB_ERRNO_NETFP_BASE-1

/**
 * @brief   Error Code: Invalid Arguments.
 */
#define NETFP_EINVAL                        SYSLIB_ERRNO_NETFP_BASE-2

/**
 * @brief   Error Code: NETFP services are not available.
 */
#define NETFP_ENOTREADY                     SYSLIB_ERRNO_NETFP_BASE-3

/**
 * @brief   Error Code: Internal Error
 */
#define NETFP_EINTERNAL                     SYSLIB_ERRNO_NETFP_BASE-4

/**
 * @brief   Error Code: No Route exists for the socket to send data
 */
#define NETFP_ENOROUTE                      SYSLIB_ERRNO_NETFP_BASE-5

/**
 * @brief   Error Code: Socket is NOT connected
 */
#define NETFP_ENOTCONNECTED                 SYSLIB_ERRNO_NETFP_BASE-6

/**
 * @brief   Error Code: Fragmentation Failed
 */
#define NETFP_EFRAGFAIL                     SYSLIB_ERRNO_NETFP_BASE-7

/**
 * @brief   Error Code: Not Implemented
 */
#define NETFP_ENOTIMPL                      SYSLIB_ERRNO_NETFP_BASE-8

/**
 * @brief   Error Code: Limit exceeded.
 */
#define NETFP_ENOSPACE                      SYSLIB_ERRNO_NETFP_BASE-9

/**
 * @brief   Error Code: Permission Denied Error
 */
#define NETFP_ENOPERM                       SYSLIB_ERRNO_NETFP_BASE-10

/**
 * @brief   Error Code: The resource is in use.
 */
#define NETFP_EINUSE                        SYSLIB_ERRNO_NETFP_BASE-11

/**
 * @brief   Error Code: 3GPP Channels & Sockets is no longer valid
 * because the underlying configuration was modified.
 */
#define NETFP_ECFG                          SYSLIB_ERRNO_NETFP_BASE-12

/**
 * @brief   Error Code: The packet header has an invalid protocol
 * or length.
 */
#define NETFP_EINVHDR                       SYSLIB_ERRNO_NETFP_BASE-13

/**
 * @brief   Error Code: The SA LLD send/receive data API failed.
 */
#define NETFP_ESA                           SYSLIB_ERRNO_NETFP_BASE-14

/**
 * @brief   Error Code: The underlying JOSH framework returned an error
 */
#define NETFP_EJOSH                         SYSLIB_ERRNO_NETFP_BASE-15

/**
 * @brief   Error Code: No matching entry found
 */
#define NETFP_ENOTFOUND                     SYSLIB_ERRNO_NETFP_BASE-16

/**
 * @brief   Error Code: Timeout while configuring the PA subsystem
 */
#define NETFP_ETIMEOUT                      SYSLIB_ERRNO_NETFP_BASE-17

/**
@}
*/

/** @addtogroup NETFP_SYMBOL
 @{ */

#define NETFP_RETVAL_SUCCESS                   0
#define NETFP_RETVAL_ERROR                    -1

/**
 * @brief   NETFP Library Version. Versions numbers are encoded in the following
 * format:
 *  0xAABBCCDD -> Arch (AA); API Changes (BB); Major (CC); Minor (DD)
 */
#define NETFP_VERSION_ID                   (0x02000000)

/**
 * @brief   Maximum number of characters
 */
#define NETFP_MAX_CHAR                      32

/**
 * @brief   This is the maximum number of L2 handles which can be supported by
 * the NETFP module.
 */
#define NETFP_MAX_L2_HANDLES                64 //fzm

/**
 * @brief   This is the maximum number of L3 handles which can be supported by
 * the NETFP module.
 */
#define NETFP_MAX_L3_HANDLES                64

/**
 * @brief   Maximum number of clients which can be supported concurrently by the
 * NETFP Server.
 */
#define NETFP_MAX_CLIENTS                   4

/**
 * @brief   Maximum number of IP addresses which can be configured concurrently
 * on an interface
 */
#define NETFP_MAX_IP_ADDRESS                8

/**
 * @brief   Maximum number of NETFP socket priorities supported by the NETFP module
 */
#define NETFP_MAX_SOCK_PRIORITY             64

/**
 * @brief   Maximum number of heap handles which can be specified and which can be
 * used by the NETCP to receive data while creating the flows.
 */
#define NETFP_MAX_HEAP_HANDLE               4

/**
 * @brief   Maximum IPSEC key length
 * rfc4106: AES-GCM-ESP with a 256 bit key is 36 octets
 */
#define NETFP_MAX_IPSEC_KEY_LEN             36

/**
 * @brief   Maximum number of user stats can be added for LUT rules
 */
#define NETFP_MAX_USR_STATS_PER_RULE        2

/**
 * @brief   This is the maximum number of data radio bearer identifers. Each data radio
 * bearer identifer is 5 bits and so at most there can be a maximum of 32 identifiers.
 */
#define NETFP_MAX_DRB                       32

/**
 * @brief   Maximum buffer sizes in the PA sub-system. Packet greater than this size are
 * fragmented in the NETFP Library; similarly packets after reassembly greater than this
 * size can no longer be classified by the NETCP subsystem and so these packets are passed
 * to the large packet channel if one is configured.
 */
#define NETFP_PASS_MAX_BUFFER_SIZE          9800

/**
 * @brief   Invalid security policy identifier which indicates that there is no
 * security policy to be used.
 */
#define NETFP_INVALID_SPID                  0xFFFFFFFF

/**
 * @brief   Invalid switch port. This is a return value which is returned when the socket is
 * NOT active and the Netfp_getSockOpt is used with the option Netfp_Option_SWITCH_PORT
 */
#define NETFP_INVALID_SWITCH_PORT           0xFFFFFFFF

/**
 * @brief   This is the maximun age of the PMTU in seconds (10 minutes) after which the PMTU
 * values are reset.
 */
#define NETFP_MAX_PMTU_AGE                  (10*60)

/**
 * @brief   This is the Maximum Transmit command supported on Egress path
 */
#define NETFP_TX_CMD_MAX                    8

/**
 * @brief   This is the Maximum Tx command stats index
 */
#define NETFP_TX_CMD_STATS_MAX              1 << NETFP_TX_CMD_MAX

/**********************************************************************************
 * Standard Endianess Macros
 **********************************************************************************/

#ifdef _BIG_ENDIAN

/**
 * @brief   Macro which converts 16 bit data from host to network order.
 */
#define  Netfp_htons(a) (a)

/**
 * @brief   Macro which converts 32 bit data from host to network order.
 */
#define  Netfp_htonl(a) (a)

/**
 * @brief   Macro which converts 32 bit data from network to host order.
 */
#define  Netfp_ntohl(a) (a)

/**
 * @brief   Macro which converts 16 bit data from network to host order.
 */
#define  Netfp_ntohs(a) (a)

#else

/**
 * @brief   Macro which converts 16 bit data from host to network order.
 */
#define  Netfp_htons(a)    ( (((a)>>8)&0xff) + (((a)<<8)&0xff00) )

/**
 * @brief   Macro which converts 32 bit data from host to network order.
 */
#define  Netfp_htonl(a)    ( (((a)>>24)&0xff) + (((a)>>8)&0xff00) + \
                             (((unsigned int)(a)<<8)&0xff0000) + (((unsigned int)(a)<<24)&0xff000000) )

/**
 * @brief   Macro which converts 32 bit data from network to host order.
 */
#define  Netfp_ntohl(a)   Netfp_htonl(a)

/**
 * @brief   Macro which converts 16 bit data from network to host order.
 */
#define  Netfp_ntohs(a)   Netfp_htons(a)

#endif /* _BIG_ENDIAN */

//fzm-->
// we should always have the following relations:
// PPT < client_UINTC < proxy_client < master_UINTC < server_UINTC < proxy_UINTC
#define DEFAULT_PACKET_PROCESSING_THREAD_SCHED_PRIORITY (49)
#define NETFP_CLIENT_UINTC_THREAD_SCHED_PRIORITY        (51)
#define NETFP_MASTER_UINTC_SCHED_PRIORITY               (53)
#define NETFP_PROXY_UINTC_SCHED_PRIORITY                (55)

#define NETFP_PROXY_CLIENT_NICENESS                     (-10)
#define NETFP_PROXY_CORE_THREAD_NICENESS                (-5)
#define DISPATCHER_CLIENT_NICENESS                      (-10)
#define NETFP_SERVER_NICENESS                           (-9)
//fzm<--

/**
@}
*/

/**
@addtogroup NETFP_ENUM
@{
*/

/**
 * @brief
 *  NETFP execution realm
 *
 * @details
 *  NETFP (server or clients) can execute in either of the following
 *  execution realms.
 */
typedef enum Netfp_ExecutionRealm
{
    /**
     * @brief   Executing on the ARM
     */
    Netfp_ExecutionRealm_ARM = 0x1,

    /**
     * @brief   Executing on the DSP
     */
    Netfp_ExecutionRealm_DSP = 0x2
}Netfp_ExecutionRealm;

/**
 * @brief
 *  Enumeration for IP Version
 *
 * @details
 *  Enumeration which describes the IP version being used.
 */
typedef enum Netfp_IPVersion
{
    /**
     * @brief   Invalid IP Version:
     */
    Netfp_IPVersion_INVALID = 0x0,

    /**
     * @brief   IPv4
     */
    Netfp_IPVersion_IPV4    = 0x1,

    /**
     * @brief   IPv6
     */
    Netfp_IPVersion_IPV6    = 0x2
}Netfp_IPVersion;

/**
 * @brief
 *  Enumeration for the direction
 *
 * @details
 *  Enumeration the SA, SP, Fast path, ciphering direction.
 */
typedef enum Netfp_Direction
{
    /**
     * @brief   Ingress for packets coming from EPC to eNodeB
     */
    Netfp_Direction_INBOUND     = 1,

    /**
     * @brief    Egress for packets going from eNodeB to EPC
     */
    Netfp_Direction_OUTBOUND    = 2
}Netfp_Direction;

/**
 * @brief
 *  IPSEC Protocols
 *
 * @details
 *  IPSEC protocols which are currently supported by NETFP
 */
typedef enum Netfp_IPSecProto
{
    /**
     * @brief  Encapsulating Security Payload
     */
    Netfp_IPSecProto_IPSEC_ESP = 1
}Netfp_IPSecProto;

/**
 * @brief
 *  IPSEC Transport mode
 *
 * @details
 *  IPSEC transport modes which are currently supported by NETFP
 */
typedef enum Netfp_IPSecMode
{
    /**
     * @brief  IPSEC Tunnel Mode
     */
    Netfp_IPSecMode_IPSEC_TUNNEL = 1
}Netfp_IPSecMode;

/**
 * @brief
 *  IPSEC Authentication modes
 *
 * @details
 *  IPSEC Authentication modes currently supported
 */
typedef enum Netfp_IpsecAuthMode
{
    /**
     * @brief  No Authentication
     */
    Netfp_IpsecAuthMode_NULL        = 1,

    /**
     * @brief  HMAC with SHA1 mode
     */
    Netfp_IpsecAuthMode_HMAC_SHA1   = 2,

    /**
     * @brief  HMAC with MD5 mode
     */
    Netfp_IpsecAuthMode_HMAC_MD5    = 3,

    /**
     * @brief  AES with XCBC mode
     */
    Netfp_IpsecAuthMode_AES_XCBC    = 4,

    /**
     * @brief  HMAC with SHA2 mode
     */
    Netfp_IpsecAuthMode_HMAC_SHA2_256    = 5,

    /**
     * @brief  AES with GMAC mode
     */
    Netfp_IpsecAuthMode_AES_GMAC = 6

}Netfp_IpsecAuthMode;

/**
 * @brief
 *  IPSEC Ciphering modes
 *
 * @details
 *  IPSEC ciphering modes currently supported
 */
typedef enum Netfp_IpsecCipherMode
{
    /**
     * @brief  No encryption
     */
    Netfp_IpsecCipherMode_NULL = 0,

    /**
     * @brief  AES Counter mode
     */
    Netfp_IpsecCipherMode_AES_CTR,

    /**
     * @brief  AES CBC mode
     */
    Netfp_IpsecCipherMode_AES_CBC,

    /**
     * @brief  3DES CBC mode
     */
    Netfp_IpsecCipherMode_3DES_CBC,

    /**
     * @brief  DES CBC mode
     */
    Netfp_IpsecCipherMode_DES_CBC,

    /**
     * @brief  AES GCM mode
     */
    Netfp_IpsecCipherMode_AES_GCM

}Netfp_IpsecCipherMode;

/**
 * @brief
 *  NETFP Fragmentation Levels
 *
 * @details
 *  This is the NETFP IPSEC fragmentation levels.  This enumeration describes at what
 *  IP level the fragmentation should be done
 */
typedef enum Netfp_IPSecFragLevel
{
    /**
     * @brief   Fragment the packets on inner IP header. Packets fragmented before encryption
     * in the NETFP software.
     */
    Netfp_IPSecFragLevel_INNER_IP   =  0,

    /**
     * @brief   Fragment the packets on outer IP header. Packets fragmented after encryption in
     * NETCP hardware
     */
    Netfp_IPSecFragLevel_OUTER_IP   = 1
}Netfp_IPSecFragLevel;

/**
 * @brief
 *  NETFP 3GPP operations
 *
 * @details
 *  This enumerations describes the type of 3GPP operation which needs to be done of the packet.
 *  Packets can be ciphered or deciphered.
 */
typedef enum Netfp_3gppOperation
{
    /**
     * @brief   Ciphering operation
     */
    Netfp_3gppOperation_Cipher      = 0,

    /**
     * @brief   Deciphering operation
     */
    Netfp_3gppOperation_Decipher    = 1
}Netfp_3gppOperation;

/**
 * @brief
 *  Ciphering Modes
 *
 * @details
 *  Enumeration which describes the various supported ciphering modes
 */
typedef enum Netfp_3gppCipherMode
{
    /**
     * @brief   No encryption
     */
    Netfp_3gppCipherMode_EEA0 = 0,

    /**
     * @brief   SNOW3G
     */
    Netfp_3gppCipherMode_EEA1 = 1,

    /**
     * @brief   AES Counter mode(128 AES CNTR)
     */
    Netfp_3gppCipherMode_EEA2 = 2
}Netfp_3gppCipherMode;

/**
 * @brief
 *  Authentication Modes
 *
 * @details
 *  Enumeration which describes the various supported authentication modes
 */
typedef enum Netfp_3gppAuthMode
{
    /**
     * @brief   No authenticiation
     */
    Netfp_3gppAuthMode_EIA0 = 0,

    /**
     * @brief   SNOW3G
     */
    Netfp_3gppAuthMode_EIA1,

    /**
     * @brief   Cipher-based Message Authentication Code(AES CMAC) mode
     */
    Netfp_3gppAuthMode_EIA2
}Netfp_3gppAuthMode;

/**
 * @brief
 *  Netfp Socket Family
 *
 * @details
 *  This is the socket family associated with the sockets.
 */
typedef enum Netfp_SockFamily
{
    /**
     * @brief  IPv4
     */
    Netfp_SockFamily_AF_INET  = 0x1,

    /**
     * @brief  IPv6
     */
    Netfp_SockFamily_AF_INET6 = 0x2
}Netfp_SockFamily;

/**
 * @brief
 *  Netfp Options Enumeration
 *
 * @details
 *  The enumeration describes a list of all the supported options which
 *  are used to configure various sub modules in the NETFP subsystem.
 *
 *  @sa
 *  Netfp_OptionTLV
 */
typedef enum Netfp_Option
{
    /**
     * @brief   Option which can be used to get/set the MTU of the specified
     * interface. The option is valid only for the functions Netfp_getIfOpt
     * and Netfp_setIfOpt.
     *
     * @sa Netfp_getIfOpt
     * @sa Netfp_setIfOpt
     *
     * NOTE: The length in the TLV should be 4 bytes for this option.
     */
    Netfp_Option_MTU                        = 0x1,

    /**
     * @brief   Option which can be used to get/set the status of the specified
     * interface. The option is valid only for the functions Netfp_getIfOpt
     * and Netfp_setIfOpt.
     *
     * @sa Netfp_getIfOpt
     * @sa Netfp_setIfOpt
     *
     * NOTE: The length in the TLV should be 4 bytes for this option. Acceptable
     * values are
     *  1   -   Interface is up
     *  0   -   Interface is down
     */
    Netfp_Option_IFACE_STATUS,

    /**
     * @brief   Option which can be used to add an IP address to the interface
     * The option is valid only under the function Netfp_setIfOpt.
     *
     * @sa Netfp_setIfOpt
     * @sa Netfp_InterfaceIP
     *
     * NOTE: The length in the TLV should be set to the size of the Netfp_InterfaceIP
     * and the value should be populated with the IP address which is to be added
     */
    Netfp_Option_ADD_IP,

    /**
     * @brief   Option which can be used to delete an IP address to the interface
     * The option is valid only under the function Netfp_setIfOpt.
     *
     * @sa Netfp_setIfOpt
     * @sa Netfp_InterfaceIP
     *
     * NOTE: The length in the TLV should be set to the size of the Netfp_InterfaceIP
     * and the value should be populated with the IP address which is to be deleted
     */
    Netfp_Option_DEL_IP,

    /**
     * @brief   Option which can be used to get a list of all the IPv6 address
     * which are configured on the interface. The option is valid only under the
     * function Netfp_getIfOpt.
     *
     * To retreive the IPv4 address use the Netfp_findInterface API.
     *
     * @sa Netfp_getIfOpt
     * @sa Netfp_InterfaceIP
     * @sa Netfp_findInterface
     *
     * NOTE: Interfaces can hold NETFP_MAX_IPV6_ADDRESS so ensure that the memory
     * allocated is (sizeof(Netfp_InterfaceIP) * NETFP_MAX_IPV6_ADDRESS) bytes.
     */
    Netfp_Option_GET_IPv6,

    /**
     * @brief   Option to setup priority for a socket. This option is valid only
     * for the functions Netfp_getSockOpt and Netfp_setSockOpt. The priority is used
     * only for connected sockets and would map to the DSCP which is to be configured
     * in the IP header of any packet transmitted by the socket.
     *
     * @sa Netfp_getSockOpt
     * @sa Netfp_setSockOpt
     *
     * NOTE: The length in the TLV should be 1 byte for this option.
     */
    Netfp_Option_PRIORITY,

    /**
     * @brief   Option to setup priority for packets send out via selected socket.
     * This option is valid only for the functions Netfp_getSockOpt and Netfp_setSockOpt.
     * The priority is used only for connected sockets and would set VLAN header priority field
     * for any packet transmitted by the socket when VLAN identifier is zero. Tag can be
     * disabled by using value of -1. Valid values are as per 802.1Q priority, 0-7.
     *
     * @sa Netfp_getSockOpt
     * @sa Netfp_setSockOpt
     *
     * NOTE: The length in the TLV should be 1 byte for this option.
     */
    Netfp_Option_PRIORITYTAG,

    /**
     * @brief   Option which can be used to get the last used CountC value by a LTE channel.
     * The option is valid only for the functions Netfp_getChannelOpt API.
     *
     * @sa Netfp_getChannelOpt
     *
     * NOTE: The length in the TLV should be 4 bytes for this option.
     */
    Netfp_Option_COUNTC,

    /**
     * @brief   Option to get the state of the socket. This option is valid only
     * for the Netfp_getSockOpt functions. Sockets can be active or not depending
     * upon the configuration. State of 1 indicates that the socket is operating
     * properly while 0 indicates that the socket is not active anymore.
     *
     * @sa Netfp_getSockOpt
     *
     * NOTE: The length in the TLV should be 1 byte for this option.
     */
    Netfp_Option_STATE,

    /**
     * @brief   Option to enable/disable the DF bit in the IPv4 header. Set to 1
     * to set the DF bit and set to 0 to clear the DF bit.
     *
     * @sa Netfp_getSockOpt
     * @sa Netfp_setSockOpt
     *
     * NOTE: The length in the TLV should be 1 byte for this option.
     */
    Netfp_Option_DONT_FRAG,

    /**
     * @brief   Option to get the MTU associated with the socket. Initially
     * this is setup to the MTU of the interface to which the socket is connected
     * on but this can change because of PMTU updates.
     *
     * @sa Netfp_getSockOpt
     * @sa Netfp_Reason_PMTU_CHANGE
     *
     * NOTE: The length in the TLV should be 4 byte for this option.
     */
    Netfp_Option_SOCK_MTU,

    /**
     * @brief   Option to get/clear the priority statistics for a socket.
     *
     * To get the statistics; use the Netfp_getSockOpt API and ensure that the
     * length is set to sizeof(Netfp_SocketPriorityStats). The function will
     * populate the "value" with the statistics of the socket [Priority is
     * derived from the socket priority]
     *
     * To clear the statistics; use the Netfp_setSockOpt API and ensure that the
     * length is set to 1. The function will clear the statistics of the socket
     * [Priority is derived from the socket priority]
     *
     * @sa Netfp_setSockOpt
     * @sa Netfp_getSockOpt
     */
    Netfp_Option_PRIORITY_STATISTICS,

    /**
     * @brief   Option to get the total cumulative statistics for the socket across
     * all priorities. The statistics cannot be cleared.
     *
     * @sa Netfp_getSockOpt
     *
     * NOTE: The length in the TLV should be sizeof(Netfp_SocketStats) for this option.
     */
    Netfp_Option_TOTAL_STATISTICS,

    /**
     * @brief   Option to get the extended statistics for the socket. Extended statistics
     * provide more detailed statistical information. The statistics cannot be cleared.
     *
     * @sa Netfp_getSockOpt
     *
     * NOTE: The length in the TLV should be sizeof(Netfp_ExtendedSocketStats) for
     * this option.
     */
    Netfp_Option_EXTENDED_STATISTICS,

    /**
     * @brief   Option to get/set the VLAN Priority mapping for a specific interface
     * The interface handle should be a VLAN enabled interface. Using this option for
     * a NON-VLAN Interface will result in an error.
     *
     * @sa Netfp_setIfOpt
     * @sa Netfp_getIfOpt
     *
     * NOTE: The length in the TLV should be set to the size of the sizeof(Netfp_VLANPriorityMap)
     * and the value should be populated with the new configuration.
     */
    Netfp_Option_VLAN_EGRESS_PRIORITY,

    /**
     * @brief   Option to get the L3 QOS shaper configuration for a specific interface
     * The option is valid only under the function Netfp_getIfOpt/Netfp_setIfOpt.
     *
     * @sa Netfp_setIfOpt
     * @sa Netfp_getIfOpt
     *
     * NOTE: The length in the TLV should be set to the size of the sizeof(Netfp_L3QoSCfg)
     * and the value should be populated with the new configuration.
     */
    Netfp_Option_L3_QOS,

    /**
     * @brief   Option to get the switch port number being used by an active socket to
     * transmit packets. If the socket is not active then the function will populate the
     * switch port number as NETFP_INVALID_SWITCH_PORT.
     *
     * @sa Netfp_getSockOpt
     * @sa NETFP_INVALID_SWITCH_PORT
     *
     * NOTE: The length in the TLV should be 4 bytes for this option.
     */
    Netfp_Option_SWITCH_PORT,

    /**
     * @brief   Option to get the VLAN identifier associated with the interface. This is
     * valid only for active sockets and if the interface being used by the socket is VLAN
     * enabled. In all other cases the function will populate 0x0 as the VLAN identifier.
     *
     * @sa Netfp_getSockOpt
     *
     * NOTE: The length in the TLV should be 2 bytes for this option.
     */
    Netfp_Option_VLAN_ID,

    /**
     * @brief   Option to enable/disable Tx UDP checksum offload. By default UDP checksum
     * Offload is enabled.
     *
     * @sa Netfp_getSockOpt
     *
     * NOTE: The length in the TLV should be 1 bytes for this option.
     */
    Netfp_Option_TX_CHECKSUM_OFFLOAD,

    /**
     * @brief   Option to enable/disable offloading of Frame protocol CRC computation.
     * By default Frame protocol CRC Offload is disabled.
     *
     * @sa Netfp_setSockOpt
     *
     * NOTE: The length in the TLV should be 1 bytes for this option.
     */
    Netfp_Option_FRAME_PROTOCOL_CRC_OFFLOAD,

    /**
     * @brief   Option to configure socket's 3GPPP layer properties.
     *
     * @sa Netfp_setSockOpt
     *
     * NOTE: The length in the TLV should be set to the size of the sizeof(Netfp_Sock3GPPCfg)
     * and the value should be populated with the configuration parameters. This socket option
     * can be set only once.
     */
    Netfp_Option_SOCK_3GPP_CFG
}Netfp_Option;

/**
 * @brief
 *  NETFP Hook identifiers
 *
 * @details
 *  The NETFP allows applications to register hooks at the following points in the NETFP packet flow.
 */
typedef enum Netfp_Hook
{
    /**
     * @brief  The NETFP hook is invoked on the reception of outer IP fragments or these could
     * be packets received on the outer IP reassembly channel matching a flow which is under
     * reassembly.
     */
    Netfp_Hook_PRE_OUTER_REASSEMBLY,

    /**
     * @brief  The NETFP hook is invoked on the reception of inner IP fragments or these could
     * be packets received on the inner IP reassembly channel matching a flow which is under
     * reassembly.
     */
    Netfp_Hook_PRE_INNER_REASSEMBLY,

    /**
     * @brief  The NETFP hook is invoked once all the IP fragments have been successfully
     * reassembled and before these packets are passed back to the NETCP for further
     * classification. IP Reassembly is done for the entire system.
     */
    Netfp_Hook_POST_OUTER_REASSEMBLY,

    /**
     * @brief  The NETFP hook is invoked once all the IP fragments have been successfully
     * reassembled and before these packets are passed back to the NETCP for further
     * classification. IP Reassembly is done for the entire system.
     */
    Netfp_Hook_POST_INNER_REASSEMBLY,

    /**
     * @brief  The NETFP hook is the final hook once all the L2 headers have been added and just
     * before the packet is pushed to the NETCP for transmission (Non-Secure) or to the SA for
     * IPSEC encryption. This hook is registered per socket.
     */
    Netfp_Hook_POST_ROUTING
}Netfp_Hook;

/**
 * @brief
 *  NETFP Hook return codes
 *
 * @details
 *  These are the codes which are returned by the NETFP Hooks. Packet traversal through the NETFP
 *  flows is impacted by these return codes.
 */
typedef enum Netfp_HookReturn
{
    /**
     * @brief  The NETFP hook has accepted the packet and continue as normal.
     */
    Netfp_HookReturn_ACCEPT = 0x1,

    /**
     * @brief  The NETFP hook has dropped the packet. Cleanup and exit
     */
    Netfp_HookReturn_DROP,

    /**
     * @brief  The NETFP hook has stolen the packet.
     */
    Netfp_HookReturn_STOLEN
}Netfp_HookReturn;

/**
 * @brief
 *  NETFP Logging Levels
 *
 * @details
 *  Log Levels of messages which are generated by the NETFP server
 */
typedef enum Netfp_LogLevel
{
    /**
    * @brief   Debug message
    */
    Netfp_LogLevel_VERBOSE, //fzm
    /**
     * @brief   Debug message
     */
    Netfp_LogLevel_DEBUG,

    /**
     * @brief   Informational message
     */
    Netfp_LogLevel_INFO,

    /**
     * @brief   Error message
     */
    Netfp_LogLevel_ERROR
}Netfp_LogLevel;

/**
 * @brief
 *  SA Protocols
 *
 * @details
 *  Enumeration describes the security protocols which are supported
 *  in the NETFP.
 */
typedef enum Netfp_SaProtocol
{
    /**
     * @brief   IPSEC Protocol
     */
    Netfp_SaProtocol_IPSEC    =  1,

    /**
     * @brief   Air ciphering
     */
    Netfp_SaProtocol_3GPP      = 2
}Netfp_SaProtocol;

/**
 * @brief
 *  Netfp Reason Enumeration
 *
 * @details
 *  The enumeration describes a list of all the reasons which affect the socket
 *  operations. These are passed to the socket using the NOTIFY call back function.
 *
 *  @sa
 *  Netfp_NotifyFunction
 */
typedef enum Netfp_Reason
{
    /**
     * @brief  The event is generated if a NETFP client deletes the fast path using the NETFP
     * exported API. The NETFP socket was attached to this fast path and so the NETFP socket
     * is no longer operational. There is no recovery from this event. Applications need to
     * close the socket. Failure to do so will result in unpredictable behavior
     */
    Netfp_Reason_FAST_PATH_DELETE   = 0x1,

    /**
     * @brief  The event is generated if a NETFP client deletes the security association using
     * the NETFP exported API. The NETFP socket was using the security association and so the
     * NETFP socket is no longer operational. There is no recovery from this event. Applications
     * need to close the socket. Failure to do so will result in unpredictable behavior
     */
    Netfp_Reason_SA_DELETE,

    /**
     * @brief  The event is generated if a NETFP client deletes the security policy using the NETFP
     * exported API. The NETFP socket was using the security policy but now the NETFP socket is
     * out of policy and is no longer operational. There is no recovery from this event. Applications
     * need to close the socket. Failure to do so will result in unpredictable behavior
     */
    Netfp_Reason_SP_DELETE,

    /**
     * @brief  The event is generated if a NETFP client deletes the interface using the NETFP
     * exported API. The NETFP socket was using the interface to send and receive data and so the
     * NETFP socket is no longer operational.  There is no recovery from this event. Applications
     * need to close the socket. Failure to do so will result in unpredictable behavior
     */
    Netfp_Reason_INTERFACE_DELETE,

    /**
     * @brief   The event is generated if the next hop MAC address is reachable. The NETFP socket
     * is operational and can send out packets.
     */
    Netfp_Reason_NEIGH_REACHABLE,

    /**
     * @brief   The event is generated if the next hop MAC address is unreachable. The NETFP socket
     * is no longer operational and cannot send out packets anymore. Sockets can recover from this
     * condition once the neighbor becomes reachable
     */
    Netfp_Reason_NEIGH_UNREACHABLE,

    /**
     * @brief   Connected NETFP sockets are using an 'interface' to send out packets. The event is a
     * notification to indicate that the MTU of the interface has been changed. This is an informational
     * message.
     */
    Netfp_Reason_IF_MTU_CHANGE,

    /**
     * @brief   Connected NETFP sockets are using an 'interface' to send out packets. The event is a
     * notification to indicate that the PMTU of the route has changed.
     */
    Netfp_Reason_PMTU_CHANGE,

    /**
     * @brief   Connected NETFP sockets are using an 'interface' to send out packets. The event is a
     * notification to indicate that the interface has been bought down. This is an informational
     * message.
     */
    Netfp_Reason_IF_DOWN,

    /**
     * @brief   Connected NETFP sockets are using an 'interface' to send out packets. The event is a
     * notification to indicate that the interface has been bought up. This is an informational
     * message.
     */
    Netfp_Reason_IF_UP,

    /**
     * @brief   Connected NETFP sockets are using an interface to send out packets. Interfaces specify a
     * unique mapping which maps the Inner DSCP to the Outer DSCP. This is an informational message to the
     * application to indicate that there was an update to the mapping.
     */
    Netfp_Reason_DSCP_MAPPING,

    /**
     * @brief   Connected NETFP sockets are using an interface to send out packets. Interfaces specify a
     * unique mapping which maps the Socket Priority to VLAN-Pbits. This is an informational message to the
     * application to indicate that there was an update to the mapping.
     */
    Netfp_Reason_VLAN_EGRESS,

    /**
     * @brief   Connected NETFP sockets are using an interface to send out packets. Interfaces could be
     * attached to an L3 Shaper. This is an informational message to the application to indicate that the
     * L3 QOS Shaper configuration has been modified.
     */
    Netfp_Reason_L3_QOS,

    /**
     * @brief   The event is generated if the security policy becomes inactive. This could be because
     * the SA is no longer reachable (cable unplugged) *OR* after rekeying the SA (new config) is no
     * longer reachable.
     */
    Netfp_Reason_SP_INACTIVE,

    /**
     * @brief   The event is generated if the security policy becomes active. This could be because
     * the SA becomes reachable (cable plugged) *OR* after rekeying the SA (new config) is reachable.
     */
    Netfp_Reason_SP_ACTIVE,

    /**
     * @brief   The event is generated because the rekeying procedure has been implemented
     */
    Netfp_Reason_REKEY_SA,

    /**
     * @brief   Unknown reason: This is a catch all condition.
     */
    Netfp_Reason_UNKNOWN
}Netfp_Reason;

/**
 * @brief
 *  SPID Mode
 *
 * @details
 *  Enumeration describes spid mode used in IPsec tunnel, non-secure
 *  or wild carding Fast Path.
 */
typedef enum Netfp_SPIDMode
{
    /**
     * @brief   NO SPID specified, used for non-secure mode
     */
    Netfp_SPIDMode_INVALID    =  0,

    /**
     * @brief   Any SPID, used for fast path wild carding without specifying spid
     */
    Netfp_SPIDMode_ANY_SECURE,

    /**
     * @brief   Specific spid to be matched with SP configuration, used for
       general Ipsec modes
     */
    Netfp_SPIDMode_SPECIFIC
}Netfp_SPIDMode;

/**
 * @brief
 *  Destination for the Ethernet Rule programmed in LUT 1-0
 *
 * @details
 *  Enumeration describes the allowed destination for Ethernet Rules programmed
 *  in LUT 1-0.
 */
typedef enum Netfp_EthRuleDst
{
    /**
     * @brief    Destination of the Ethernet Rule is EMAC
     */
    Netfp_EthRuleDst_EMAC    = 0,

    /**
     * @brief    Destination of the Ethernet Rule is continue parsing
     */
    Netfp_EthRuleDst_CONTINUE,

    /**
     * @brief    Destination of the Ethernet Rule is Linux interface
     */
    Netfp_EthRuleDst_HOST,

    /**
     * @brief    Destination of the Ethernet Rule is discard
     */
    Netfp_EthRuleDst_DISCARD //fzm
}Netfp_EthRuleDst;

/**
 * @brief
 *  Netfp Proxy Server Operation Type
 *
 * @details
 *  Enumeration describes the type of operation performed by the Netfp Proxy Server.
 */
typedef enum Netfp_ProxyServerOp
{
    /**
     * @brief   Indicates a request from Server to Proxy.
     */
    Netfp_ProxyServerOp_REQUEST    =  0,

    /**
     * @brief   Indicates a response from Proxy to Server.
     */
    Netfp_ProxyServerOp_RESPONSE,

    /**
     * @brief   Indicates an async status update sent by Proxy to Server.
     * This is used to update the neighbor status.
     */
    Netfp_ProxyServerOp_UPDATE_NEIGH,

    /**
     * @brief   Indicates an async status update sent by Proxy to Server.
     * This is used to update the PMTU
     */
    Netfp_ProxyServerOp_UPDATE_MTU
}Netfp_ProxyServerOp;

/**
 * @brief
 *  Multicast operational Mode
 *
 * @details
 *  Enumeration which describes the operational mode for multicast services
 *  while creating the fast path. In order for an application to receive
 *  multicast data please be aware of the "Multicast Preclassification" feature
 *  which allows NETCP to bypass the LUT1-0 lookup for multicast packets.
 *  This is configured in the NETFP Master. Applications might need to either:-
 *  - Disable Preclassification
 *  - Override default preclassification to receive data to custom application
 *  - Use Ethernet rules
 */
typedef enum Netfp_FastpathMode
{
    /**
     * @brief   Normal fast path mode. The configuration here can handle
     * the following cases:
     *      - Unicast IP address
     *      - Multicast IP address
     * Each IP address will have an entry added to the NETCP Lookup
     * Table.
     */
    Netfp_FastpathMode_NORMAL   = 0x0,

    /**
     * @brief   Information only fast path mode. The configuration here can handle
     * the following cases:
     *      - Unicast IP address
     * In this mode, no entry will be added to the NETCP Lookup Table.
     */
    Netfp_FastpathMode_INFOONLY,

    /**
     * @brief   Multicast shared mode: The configuration here can handle only
     * the following cases:-
     *      - Multicast IP address
     * In this mode the NETFP library will create a single LUT1-1 entry which can
     * be used to receive multicast data.
     */
    Netfp_FastpathMode_MULTICASTSHARED,

    Netfp_FastpathMode_SW_EXTENSION_CHANNEL //fzm
}Netfp_FastpathMode;

/**
 * @brief
 *  Raw Destination type
 *
 * @details
 *  The enumeration describes the destination which where the raw packet payload
 *  is to be sent.
 */
typedef enum Netfp_RawDstType
{
    /**
     * @brief    Raw Packet is destined to be sent over a switch port
     */
    Netfp_RawDstType_SWITCH_PORT = 1,

    /**
     * @brief    Raw Packet is destined to be sent over the control path back to
     * the Linux Ethernet driver.
     */
    Netfp_RawDstType_CONTROL_PATH,

    /**
     * @brief    Raw packet is destined to be sent back to the NETCP for further classification
     * since this packet could be destined for a Local Fast Path and Socket.
     */
    Netfp_RawDstType_FAST_PATH
}Netfp_RawDstType;

/**
 * @brief
 *  The enumeration describes the Frame Protocol type
 */
typedef enum Netfp_FpType
{
    /**
     * @brief  This is the default type, used for non-FP frames.
     */
    Netfp_FpType_NONE               = 0x0,

    /**
     * @brief  This is the HS-DSCH Type 1 data frame protocol.
     */
    Netfp_FpType_HS_DSCH_TYPE1      = 0x1,

    /**
     * @brief  This is the HS-DSCH Type 2 data frame protocol.
     */
    Netfp_FpType_HS_DSCH_TYPE2      = 0x2,

    /**
     * @brief  This is the HS-DSCH Type 3 data frame protocol.
     */
    Netfp_FpType_HS_DSCH_TYPE3      = 0x3,

    /**
     * @brief  This is the DL-DCH data frame protocol.
     */
    Netfp_FpType_DL_DCH             = 0x4,

    /**
     * @brief  This is the DL-FACH data frame protocol.
     */
    Netfp_FpType_DL_FACH            = 0x5,

    /**
     * @brief  This is the DL-PCH data frame protocol.
     */
    Netfp_FpType_DL_PCH             = 0x6,

    /**
     * @brief  This is the E-DCH data frame protocol.
     */
    Netfp_FpType_EDCH               = 0x7
}Netfp_FpType;

/**
 * @brief
 *  The enumeration describes the 3GPP Protocols supported
 */
typedef enum Netfp_3GPPProto
{
    /**
     * @brief  Frame Protocol.
     */
    Netfp_3GPPProto_FRAME_PROTOCOL  = 0x1,

    /**
     * @brief  GTPU
     */
    Netfp_3GPPProto_GTPU
}Netfp_3GPPProto;

/**
@}
*/

/** @addtogroup NETFP_OSAL_API
 @{ */

/**
 *  @b Description
 *  @n
 *     The function is used by the NETFP module to allocate memory.
 *
 *  @param[in]  size
 *      Number of bytes which need to be allocated
 *  @param[in]  alignment
 *      Memory alignment requirments
 *
 *  @retval
 *      Success -   Pointer to the allocated memory
 *  @retval
 *      Error   -   NULL
 */
typedef void* (*Osal_NetfpMalloc) (uint32_t size, uint32_t alignment);

/**
 *  @b Description
 *  @n
 *     The function is used by the NETFP module to cleanup memory.
 *
 *  @param[in]  ptr
 *      Pointer to the memory to be cleaned up
 *  @param[in]  size
 *      Number of bytes which need to be allocated
 *
 *  @retval
 *      Not applicable.
 */
typedef void (*Osal_NetfpFree)(void* ptr, uint32_t size);

/**
 *  @b Description
 *  @n
 *     The function is used by the NETFP module to allocate memory for the
 *     security context.
 *
 *  @param[in]  protocol
 *      The NETFP Security protocol for which the security context is being
 *      allocated
 *  @param[in]  size
 *      Number of bytes which need to be allocated
 *  @param[in]  alignment
 *      Memory alignment requirments
 *
 *  @retval
 *      Success -   Pointer to the allocated memory
 *  @retval
 *      Error   -   NULL
 */
typedef void* (*Osal_NetfpMallocSecurityContext)(Netfp_SaProtocol protocol, uint32_t size, uint32_t alignment);

/**
 *  @b Description
 *  @n
 *     The function is used by the NETFP module to cleanup memory allocated
 *     for the security context
 *
 *  @param[in]  protocol
 *      The NETFP Security protocol for which the security context is being
 *      cleaned up
 *  @param[in]  ptr
 *      Pointer to the memory to be cleaned up
 *  @param[in]  size
 *      Number of bytes which need to be allocated
 *
 *  @retval
 *      Not applicable.
 */
typedef void (*Osal_NetfpFreeSecurityContext)(Netfp_SaProtocol protocol, void* ptr, uint32_t size);

/**
 *  @b Description
 *  @n
 *     The function is used by the NETFP module to protect its internal
 *     resources from concurrent access within a single core.
 *
 *  @retval
 *      Opaque handle to the criticial section object
 */
typedef void* (*Osal_NetfpEnterCS)(void);

/**
 *  @b Description
 *  @n
 *     The function is used by the NETFP module to protect its internal
 *     resources from concurrent access within a single core.
 *
 *  @param[in]  csHandle
 *      Opaque handle to the criticial section object
 *
 *  @retval
 *      Not applicable
 */
typedef void (*Osal_NetfpExitCS)(void* csHandle);

/**
 *  @b Description
 *  @n
 *     The function is used by the NETFP module to invalidate the contents
 *     of the cache.
 *
 *  @param[in]  ptr
 *      Pointer to the buffer to be invalidated
 *  @param[in]  size
 *      Size of the buffer to be invalidated
 *
 *  @retval
 *      Not applicable
 */
typedef void (*Osal_NetfpBeginMemoryAccess)(void* ptr, uint32_t size);

/**
 *  @b Description
 *  @n
 *     The function is used by the NETFP module to writeback the contents
 *     of the cache.
 *
 *  @param[in]  ptr
 *      Pointer to the buffer to be written back
 *  @param[in]  size
 *      Size of the buffer to be written back
 *
 *  @retval
 *      Not applicable
 */
typedef void (*Osal_NetfpEndMemoryAccess)(void* ptr, uint32_t size);

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to create a semaphore associated with a NETFP job.
 *      If the jobs are executing in SYNC mode then once a job is submitted the callee
 *      will block on this semaphore until the result is received.
 *
 *  @retval
 *      Opaque Semaphore handle
 */
typedef void* (*Osal_NetfpCreateSem)(void);

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to pend on the semaphore associated with the NETFP job
 *      This is done once a sync job is submitted.
 *
 *  @param[in]  semHandle
 *      Opaque Semaphore Handle
 *
 *  @retval
 *      Not Applicable.
 */
typedef void (*Osal_NetfpPendSem)(void* semHandle);

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to post the semaphore associated with the job
 *      once the result packet associated with the SYNC job has been received.
 *
 *  @param[in]  semHandle
 *      Opaque Semaphore Handle
 *
 *  @retval
 *      Not Applicable
 */
typedef void (*Osal_NetfpPostSem)(void* semHandle);

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to delete a semaphore associated with a JOB.
 *      Each JOB is associated with a unique semaphore.
 *
 *  @param[in]  semHandle
 *      Opaque Semaphore Handle
 *
 *  @retval
 *      Not applicable
 */
typedef void (*Osal_NetfpDeleteSem)(void* semHandle);

/**
@}
*/


/** @addtogroup NETFP_DATA_STRUCTURE
 @{ */

/**
 * @brief   NETFP Server Handle
 */
typedef void*   Netfp_ServerHandle;

/**
 * @brief   NETFP Client Handle
 */
typedef void*   Netfp_ClientHandle;

/**
 * @brief   NETFP Interface Handle
 */
typedef void*   Netfp_IfHandle;

/**
 * @brief   NETFP Inbound Fast Path Handle
 */
typedef void*   Netfp_InboundFPHandle;

/**
 * @brief   NETFP Outbound Fast Path Handle
 */
typedef void*   Netfp_OutboundFPHandle;

/**
 * @brief   NETFP Security Association Handle
 */
typedef void*   Netfp_SAHandle;

/**
 * @brief   NETFP Socket Handle
 */
typedef void*   Netfp_SockHandle;

/**
 * @brief   NETFP L2 Handle to the layer2 rules added
 */
typedef void*   Netfp_L2Handle;

/**
 * @brief   NETFP Handle to the user which has been registered with NETFP
 */
typedef void*   Netfp_UserHandle;

/**
 * @brief
 *  This is an opaque handle to the NETFP Ethernet Rule entry.
 */
typedef void*   Netfp_EthRuleHandle;

/**
 * @brief
 *  IPv4 Address
 *
 * @details
 *  IPv4 Address represented in network order.
 */
typedef struct Netfp_IPN
{
    union
    {
        uint8_t    a8[4];
        uint32_t   a32;
    }u;
}Netfp_IPN;

/**
 * @brief
 *  IPv6 Address
 *
 * @details
 *  IPv6 Address represented in network order.
 */
typedef struct Netfp_IP6N
{
    union
    {
        uint8_t    a8[16];
        uint16_t   a16[8];
        uint32_t   a32[4];
    }u;
}Netfp_IP6N;

/**
 * @brief
 *  IP Address Format used by the NETFP Library
 *
 * @details
 *  Generic data structure which encapsulates both IPv4
 *  and IPv6 addresses.
 */
typedef struct Netfp_IPAddr
{
    Netfp_IPVersion     ver;
    union
    {
        Netfp_IPN   ipv4;
        Netfp_IP6N  ipv6;
    }addr;
}Netfp_IPAddr;

/**
 * @brief
 *  Proxy Server Request Info
 *
 * @details
 *
 *  The NETFP Server can send a request to get the next hop MAC address for the specific
 *  specific DST/SRC IP address. The NETFP Proxy will start monitoring the next
 *  hop MAC address.
 *
 *  Once the NETFP Server is no longer interested in the DST/SRC IP address it can
 *  indicate to the NETFP Proxy to stop monitoring the routes.
 */
typedef struct Netfp_ProxyServerRequestInfo
{
    /**
     * @brief   Flag which indicates NETFP Proxy should start/stop monitoring process
     */
    uint32_t                startMonitor;
}Netfp_ProxyServerRequestInfo;

/**
 * @brief
 *  Proxy Server Response Info
 *
 * @details
 *
 *  The NETFP Proxy uses this structure to send a response to the start/stop monitoring request issued by NETFP Server.
 */
typedef struct Netfp_ProxyServerResponseInfo
{
    /**
     * @brief   Next Hop MAC address of the neighbor monitored by Proxy.
     * Next hop MAC address resolved by proxy.
     * In case of unresolved neighbor, stop monitoring request or error, it is set to zero.
     */
    uint8_t                 nextHopMacAddress[6];

    /**
     * @brief Handle to the interface associated with the route resolution.
     * Interface handle is set to the interface using which the route was resolved.
     * In case of unresolved neighbor, stop monitoring request or error it is set to NULL.
     */
    Netfp_IfHandle          ifHandle;
}Netfp_ProxyServerResponseInfo;

/**
 * @brief
 *  Proxy Server Neighbor Update Info
 *
 * @details
 *  The NETFP Proxy uses this structure to send a asynchronous neighbor update to the NETFP Server
 */
typedef struct Netfp_ProxyServerUpdateNeighInfo
{
    /**
     * @brief   This is the old next hop MAC address when the neighbor MAC address changes.
     */
    uint8_t                 oldMacAddress[6];

    /**
     * @brief   Next Hop MAC address of the neighbor monitored by Proxy.
     */
    uint8_t                 nextHopMacAddress[6];

    /**
     * @brief Handle to the interface associated with the route resolution.
     */
    Netfp_IfHandle          ifHandle;
}Netfp_ProxyServerUpdateNeighInfo;

/**
 * @brief
 *  Proxy Server MTU Update Info
 *
 * @details
 *  The NETFP Proxy uses this structure to send a asynchronous MTU update to the NETFP Server
 */
typedef struct Netfp_ProxyServerUpdateMTU
{
    /**
     * @brief   This is the new MTU associated with the path.
     */
    uint32_t            newMTU;
}Netfp_ProxyServerUpdateMTU;

/**
 * @brief
 *  Proxy Server Interface
 *
 * @details
 *  This structure is the interface between the NETFP Proxy and Server.
 *  The  opType defines the kind of operation being performed.
 */
typedef struct Netfp_ProxyServerInfo
{
    /**
     * @brief   Indicates the type of operation performed by NETFP Proxy.
     */
    Netfp_ProxyServerOp     opType;

    /**
     * @brief   Destination IP address.
     */
    Netfp_IPAddr            dstIP;

    /**
     * @brief   Source IP address.
     */
    Netfp_IPAddr            srcIP;

    /**
     * @brief   Unique request identifier allocated to each request sent by the server to
     * the NETFP Proxy.
     */
    uint32_t                requestId;

    union
    {
        /**
         * @brief   Netfp Proxy Server request. This is only valid when the
         * opType is set to Netfp_ProxyServerOp_REQUEST
         */
        Netfp_ProxyServerRequestInfo        req;

        /**
         * @brief   Netfp Proxy Server response. This is only valid when the
         * opType is set to Netfp_ProxyServerOp_RESPONSE
         */
        Netfp_ProxyServerResponseInfo       resp;

        /**
         * @brief   Netfp Proxy Server asynchronous neighbor update. This is only valid when the
         * opType is set to Netfp_ProxyServerOp_UPDATE_NEIGH
         */
        Netfp_ProxyServerUpdateNeighInfo    updateNeigh;

        /**
         * @brief   Netfp Proxy Server MTU update. This is only valid when the
         * opType is set to Netfp_ProxyServerOp_UPDATE_MTU
         */
        Netfp_ProxyServerUpdateMTU          updateMTU;
    }u;
} __attribute__((packed)) Netfp_ProxyServerInfo;

/**
 * @brief Max number of entries that contained in the bulk
 *        message.
 */
#define BULK_INFO_MAX_ENTRIES 50

/**
 * @brief
 *  Proxy Server Bulk Message
 *
 * @details
 *  This structure is the interface between the NETFP Proxy and
 *  Server as the packed version of many ProxyServerInfo
 *  subblocks.
 */
typedef struct Netfp_ProxyServerBulkMsg
{
    uint32_t numberOfEntries;
    Netfp_ProxyServerInfo proxyServerInfo[BULK_INFO_MAX_ENTRIES];
} __attribute__((packed)) Netfp_ProxyServerBulkMsg;

/**
 * @brief
 *  Proxy Server Interface
 *
 * @details
 *  This structure is the interface between the NETFP Proxy and
 *  Server. It represents ProxyServerInfo as single instance but
 *  in ProxyServerBulk presentation. It will support keeping
 *  common interface for exchanging BulkMsg-like data between
 *  server and proxy.
 */
typedef struct Netfp_ProxyServerOneMsg
{
    uint32_t numberOfEntries;
    Netfp_ProxyServerInfo proxyServerInfo;
} __attribute__((packed)) Netfp_ProxyServerOneMsg;

/**
 *  @b Description
 *  @n
 *      This is the function which is registered by the NETFP Proxy and is used to interface
 *      with the NETFP server. The interface function is invoked by the NETFP Server under
 *      the following scenarios:
 *      - In order to send packets out to a destination IP address NETFP requires the route
 *        to be resolved which maps the IP address to the next hop MAC address. Route
 *        resolution is a time consuming process and the work is handled by the NETFP
 *        Proxy application. The NETFP Proxy will now start monitoring the IP address and will
 *        keep the next hop MAC address alive.
 *      - Once the fast path/SA is complete the NETFP server informs the PROXY to stop monitoring
 *        the specific IP address.
 *
 *  @param[in]  clientHandle
 *      NETFP Proxy client handle
 *  @param[in]  ptrProxyServerInterface
 *      Pointer to the proxy server interface information block.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
typedef int32_t (*Netfp_ProxyServerIfFunction)(Netfp_ClientHandle clientHandle, Netfp_ProxyServerBulkMsg* proxyServerBulkInfo);
/**
 *  @b Description
 *  @n
 *      NETFP Notify function: This is the function which is registered with the socket
 *      layer and is invoked when there is a configuration modification which has
 *      been detected which has caused the socket to become non-operational. Configuration
 *      modifications could be events such as Interface, Fast Path deletion *OR* MTU changes
 *      etc.
 *
 *      NOTE: The function is invoked from within the critical section.
 *
 *  @param[in]  reason
 *      Reason code because of which the notify function was invoked
 *  @param[in]  sockHandle
 *      Socket Handle which is affected
 *
 *  @retval
 *      Not applicable
 */
typedef void (*Netfp_NotifyFunction)(Netfp_Reason reason, Netfp_SockHandle sockHandle);

/**
 *  @b Description
 *  @n
 *      NETFP Reassembly Management Function: This is the function which is registered
 *      to handle the management of the reassembly context. Under heavy load or denial
 *      of service attacks it is possible that all the reassembly contexts are exhausted
 *      So applications can implement appropriate error recovery mechanisms using this
 *      API.
 *
 *      NETFP provides a default implementation which implements the following:
 *      - If (Number of active frags > Upper Buffer Threshold)
 *          Drop oldest fragments until Lower Buffer Threshold
 *
 *       This is a low level function and application developers will need to use the
 *       internal NETFP data structures to implement a custom algorithm. Refer to the
 *       default implementation as a template to get started
 *
 *  @param[in]  clientHandle
 *      NETFP Client handle
 *  @param[in]  ptrReassemblyMgmtCfg
 *      Pointer to the reassembly management configuration
 *
 *  @retval
 *      Not applicable
 */
typedef void (*Netfp_ReassemblyMgmt)(Netfp_ClientHandle clientHandle, void* ptrReassemblyMgmtCfg);

/**
 *  @b Description
 *  @n
 *      NETFP Hook function: This is the hook function which is registered with the NETFP module
 *      and is invoked by the NETFP module to pass control to the registered hook. Applications
 *      can use this feature to take control of the packet and perform any customized actions.
 *
 *  @param[in]  hook
 *      Hook number
 *  @param[in]  sockHandle
 *      Socket handle for which the hook is invoked. This parameter can be NULL for certain hooks
 *  @param[in]  ptrPkt
 *      Pointer to the packet
 *  @param[in]  arg
 *      Customized application registered argument which is passed as is
 *
 *  @retval
 *      Hook return
 */
typedef Netfp_HookReturn (*Netfp_HookFunction)(Netfp_Hook hook, Netfp_SockHandle sockHandle, Ti_Pkt* ptrPkt, uint32_t arg);

//<fzm>
struct Netfp_EventMetaInfo;
typedef void (*Netfp_SaEventHandler)(struct Netfp_EventMetaInfo* eventMetaInfo);
//</fzm>

/**
 *  @b Description
 *  @n
 *      NETFP F8 function: This is the function which is registered with the NETFP client
 *      and which allows the users on the client to use the SNOW3G ciphering support. This
 *      is required only for the SRB. Data radio bearer do not use this functionality since
 *      this is done natively in the NETCP module.
 *
 *  @param[in]  key
 *      Ciphering key
 *  @param[in]  count
 *      Value of countC
 *  @param[in]  bearer
 *      Bearer identifier
 *  @param[in]  dir
 *      Direction (Downlink or Uplink)
 *  @param[in]  data
 *      Pointer to the data
 *  @param[in]  length
 *      Length of the data
 *
 *  @retval
 *      Not applicable
 */
typedef void (*Netfp_F8Function) (uint8_t* key, int count, int bearer, int dir, uint8_t* data, int length);

/**
 *  @b Description
 *  @n
 *      NETFP F9 function: This is the function which is registered with the NETFP client
 *      and which allows the users on the client to use the SNOW3G integrity protection support.
 *
 *  @param[in]  key
 *      Integrity Protection key
 *  @param[in]  count
 *      Value of countC
 *  @param[in]  fresh
 *      Frest status
 *  @param[in]  dir
 *      Direction (Downlink or Uplink)
 *  @param[in]  data
 *      Pointer to the data
 *  @param[in]  length
 *      Length of the data
 *  @param[out] macI
 *      Generated MACI
 *
 *  @retval
 *      Not applicable
 */
typedef void (*Netfp_F9Function) (uint8_t* key, int count, int fresh, int dir, uint8_t* data, unsigned long long length, uint32_t* macI);

/**
 *  @b Description
 *  @n
 *      NETFP Server Logging function which is passed to the server during
 *      initialization. The function is used to pass messages from the server to
 *      the application during execution.
 *
 *  @param[in]  logLevel
 *      Log level of the message
 *  @param[in]  fmt
 *      Formatted string
 *  @param[in]  arg
 *      Variable length of arguments.
 *
 *  @retval
 *      Not applicable
 */
typedef void (*Netfp_ServerLogFn)(Netfp_LogLevel logLevel, const char* fmt, va_list arg) __attribute__ ((format (printf, 2, 0)));

/**
 * @brief
 *  NETFP Server configuration
 *
 * @details
 *  The NETFP Server configuration describes the various configuration properties
 *  which are required to initialize and start the NETFP Server.
 */
typedef struct Netfp_ServerConfig
{
    /**
     * @brief  Name of the server which should be unique in the domain.
     */
    char                                serverName[NETFP_MAX_CHAR];

    /**
     * @brief   Named resource instance identifier. Server will use this named
     * resource to store named resource information. It is responsibility of the
     * application to ensure that this identifier is the same across both the
     * NETFP servers and clients use the samed named resource instance identifier.
     * Failure to do so will result in the NETFP functionality to not work correctly.
     */
    uint32_t                            nrInstanceId;

    /**
     * @brief   PKTLIB Instance handle. All packets sent and received on the NETFP client
     * configuration are operating within this specific PKTLIB instance.
     */
    Pktlib_InstHandle                   pktlibInstHandle;

    /**
     * @brief   This is the MSGCOM instance handle which is used to create and manage all
     * the MSGCOM channels used between the NETFP Server & NETFP client(s). Please ensure
     * that the NETFP servers and clients are on the same MSGCOM instance handle.
     */
    Msgcom_InstHandle                   msgcomInstHandle;

    /**
     * @brief   Server execution realm
     */
    Netfp_ExecutionRealm                realm;

    /**
     * @brief   Logging function. This is an optional parameter but if specified allows
     * application to get access to the log messages which are generated by the NETFP
     * server
     */
    Netfp_ServerLogFn                   logFxn;

    // <fzm>
    Netfp_ServerLogFn                   dumpFxn;
    // </fzm>

    /**
     * @brief   Server heap handle which is used to send and receive messages to the
     * NETFP clients. It is recommended that the heap handle be local to the server
     */
    Pktlib_HeapHandle                   serverHeapHandle;

    /**
     * @brief   This is the RM Service handle which is used to exchange messages between
     * the RMv2 server & client to ensure that the PA resources are also managed by the
     * RMv2. Setting this value to NULL bypasses the resource management and could lead
     * to resource conflicts between the DSP and ARM.
     */
    Rm_ServiceHandle*                   rmServiceHandle;

    /**
     * @brief   Handle to ResMgr system configuration block. This handle is used to
       alloc/free LUT resources.
     */
    Resmgr_SysCfgHandle                 sysRMHandle;

    /**
     * @brief  This is the heap handle which will be used to send commands
     * to the PA subsystem. The NETFP server is the only entity which can configure
     * the PA so the heap does not need to be a shared heap.
     */
    Pktlib_HeapHandle                   cmdHeapHandle;

    /**
     * @brief  The flag is used to specify if the NETFP server is responsible for
     * initializing and setting up the SA subsystem in the NETCP. The PA subsystem is
     * already initialized and setup the Linux kernel.
     */
    uint8_t                             initSecureSubsystem;

    /**
     * @brief  This is the heap handle which is used to create the flow which is used
     * to pass IPSEC packets internally in the NETCP subsystem. It is recommended that
     * this heap not be shared by other entities.
     */
    Pktlib_HeapHandle                   ipSecHeapHandle;

    /**
     * @brief  This is the virtual address of the PA subsytem. This field is applicable
     * only if the server execution realm is ARM. In the DSP realm this field should be
     * set to 0 since there is no concept of a virtual address. In the case of ARM however
     * please refer to the following Resource Manager API: ResMgr_getPASSVirtualAddress.
     */
    uint32_t                            passCfgVirtualAddress;

    /**
     * @brief  This is the maximum number of security channels which can be configured
     * in the system. This should always be a multiple of 32.
     */
    uint16_t                            maxSecurityChannels;

    /**
     * @brief  This is the base security context identifier. The NETFP server will use
     * security context identifiers from: base to (base + maxSecurityChannels)
     */
    uint32_t                            baseSecurityContextId;

    /**
     * @brief  The flag is used to specify if an user stats will be enabled for IP Lut entries.
     */
     uint32_t                           enableIPLutEntryCount;

    /**
     * @brief  OSAL Malloc
     */
    Osal_NetfpMalloc                    malloc;

    /**
     * @brief  OSAL Free
     */
    Osal_NetfpFree                      free;

    /**
     * @brief  OSAL Malloc Security Context
     */
    Osal_NetfpMallocSecurityContext     mallocSecurityContext;

    /**
     * @brief  OSAL Free Security Context
     */
    Osal_NetfpFreeSecurityContext       freeSecurityContext;

    /**
     * @brief  OSAL Begin memory access
     */
    Osal_NetfpBeginMemoryAccess         beginMemAccess;

    /**
     * @brief  OSAL End memory access
     */
    Osal_NetfpEndMemoryAccess           endMemAccess;

    /**
     * @brief  OSAL NETFP Enter single core critical section
     */
    Osal_NetfpEnterCS                   enterCS;

    /**
     * @brief  OSAL NETFP Exit single core critical section
     */
    Osal_NetfpExitCS                    exitCS;

    /**
     * @brief  OSAL Create Semaphore
     */
    Osal_NetfpCreateSem                  createSem;

    /**
     * @brief  OSAL Delete Semaphore
     */
    Osal_NetfpDeleteSem                  deleteSem;

    /**
     * @brief  OSAL Post Semaphore
     */
    Osal_NetfpPostSem                    postSem;

    /**
     * @brief  OSAL Pend Semaphore
     */
    Osal_NetfpPendSem                    pendSem;
}Netfp_ServerConfig;

/**
 * @brief
 *  NETFP default reassembly management configuration
 *
 * @details
 *  NETFP provides a default reassembly management configuration which ensures
 *  that if the number of active fragments exceeds the upper thresholds then the
 *  oldest fragments will be dropped until the lower threshold is met.
 */
typedef struct Netfp_DefaultReassemblyMgmtCfg
{
    /**
     * @brief  This is the upper threshold for the number of used buffers in the
     * reassembly system. When the total number of used buffers exceeds this value,
     * the oldest N fragments are dropped where
     *  N = bufferThresholdUpper-bufferThresholdLower.
     */
    uint32_t                bufferThresholdUpper;

    /**
     * @brief  This is the lower threshold for the number of used buffers in the
     * reassembly system.
     */
    uint32_t                bufferThresholdLower;
}Netfp_DefaultReassemblyMgmtCfg;

typedef int32_t (*LargePacketHandler)(void *); //fzm

/**
 * @brief
 *  NETFP Reassembly configuration
 *
 * @details
 *  The structures describes the NETFP reassembly configuration. There should be
 *  1 designated client in the system which is responsible for performing the
 *  reassembly operations for all received fragments.
 */
typedef struct Netfp_ReassemblyConfig
{
    /**
     * @brief  Memory region from where descriptors are carved out which are used
     * to receive fragmented packets from the NETCP subsystem.
     */
    Qmss_MemRegion          reassemblyMemRegion;

    /**
     * @brief  Number of packets which need to be carved out from the specified
     * memory region. This is the number of outstanding fragments which can be
     * received and await reassembly.
     */
    uint32_t                numFragmentedPkt;

    /**
     * @brief  This is the total number of reassembly contexts or packets under
     * construction at any given time.
     */
    uint32_t                numReassemblyContexts;

    /**
     * @brief  This is the timeout threshold in application preferred units
     * for any packet under construction. If the packet reassembly is not complete
     * within the specific time the reassembly context is cleaned up and the packets
     * are dropped.
     *
     * The unit of time here should be the same as the "timeout" provided in the
     * function @sa Netfp_reassemblyTimerTick.
     */
    uint32_t                reassemblyTimeout;

    /**
     * @brief  The MSGCOM *reader* channel handle where the outer IP fragments are
     * sent by the NETCP and picked up by the reassembly software. The channels
     * should be configured in non-blocking mode.
     */
    MsgCom_ChHandle         outerIpReassemChannel;

    /**
     * @brief  The MSGCOM *reader* channel handle where the inner IP fragments are
     * sent by the NETCP and picked up by the reassembly software.The channels
     * should beconfigured in non-blocking mode.
     */
    MsgCom_ChHandle         innerIpReassemChannel;

    /**
     * @brief  This is the handle to the MSGCOM *writer* channel where packets >
     * NETFP_PASS_MAX_BUFFER_SIZE are sent to immediately after reassembly before any
     * IP/UDP lookups. This is because of the maximum limit of the NetCP ingress
     * buffer size. If this channel is configured, then the reassembly heap should
     * be created with one or more bufferless descriptors.
     */
    MsgCom_ChHandle         largePacketChannel;

    /**
     * @brief   Reassembly Heap application specific argument. This is passed back
     * to the reassembly heap allocation & cleanup API when the internal reassembly
     * heap is created.
     */
    uint32_t                reassemblyHeapArg;

    /**
     * @brief   This is used to allocate the packets for the reassembly heap
     */
    Pktlib_MallocData       reassemblyHeapAlloc;

    /**
     * @brief   This is used to free the packets for the reassembly heap
     */
    Pktlib_FreeData         reassemblyHeapFree;

    /**
     * @brief   Reassembly management: Select NULL will use the default NETFP provided
     * reassembly management. In this case please ensure that the reassembly configuration
     * is populated as per the Netfp_DefaultReassemblyMgmtCfg structure. If this is NOT
     * set to NULL the application supplied callback function is invoked.
     */
    Netfp_ReassemblyMgmt    reassemblyMgmt;

    /**
     * @brief   Reassembly configuration: This is the reassembly management configuration.
     * This should be specified as per the Netfp_DefaultReassemblyMgmtCfg structure if
     * the NETFP default reassembly management is required. Else this can point to an
     * application supplied custom data structure.
     */
    void*                   ptrReassemblyMgmtCfg;

    /**
     * @brief   Reassembly configuration: This is the size of the reassembly configuration
     * data structure which has been passed above. For default NETFP reassembly management
     * this should be set as the sizeof(Netfp_DefaultReassemblyMgmtCfg) else this will
     * result in an error. For the application defined reassembly management this should be
     * set to the application specified configuration size.
     */
    uint32_t                sizeReassemblyCfg;

    /**
     * @brief   Reassembly configuration: Nokia added function pointer when set, will be
     * called if the reassembled packet is larger than the max size the pa could handle
     */
    LargePacketHandler      largePacketHandler;

    // <fzm>
    /**
    * @brief   Reassembly configuration: If set reassembly will try to push large packets
    * to appropriately bound netfp_socket
    */
    uint32_t                largePacketPushToSocket;

    /**
    * @brief   Reassembly configuration: This is a number of max number of packets that will be
    * processed in a single dispatch of the ReassemblyService. 0 means no limit.
    */
    uint32_t                maxPacketBurst;

    /**
    * @brief   Reassembly configuration: This option specifies whether the created heap
    * shall be created with return push policy set to HEAD (it is TAIL by default)
    */
    uint32_t                pushToHead;
    // </fzm>

}Netfp_ReassemblyConfig;

/**
 * @brief
 *  NETFP Client configuration
 *
 * @details
 *  The NETFP Client configuration describes the various configuration properties
 *  which are required to initialize and start the NETFP client.
 */
typedef struct Netfp_ClientConfig
{
    /**
     * @brief  Name of the server to which the client is connecting.
     */
    char                            serverName[NETFP_MAX_CHAR];

    /**
     * @brief   Named resource instance identifier. Clients will use this named
     * resource to store named resource information. It is responsibility of the
     * application to ensure that this identifier is the same across both the
     * NETFP servers and clients use the samed named resource instance identifier.
     * Failure to do so will result in the NETFP functionality to not work correctly.
     */
    uint32_t                        nrInstanceId;

    /**
     * @brief   PKTLIB Instance handle. All packets sent and received on the NETFP client
     * configuration are operating within this specific PKTLIB instance.
     */
    Pktlib_InstHandle               pktlibInstHandle;

    /**
     * @brief   This is the MSGCOM instance handle which is used to create and manage all
     * the MSGCOM channels used between the NETFP Server & NETFP client(s). Please ensure
     * that the NETFP servers and clients are on the same MSGCOM instance handle.
     */
    Msgcom_InstHandle               msgcomInstHandle;

    /**
     * @brief   NETFP clients communicate with the NETFP Server via MSGCOM channels.
     * Each MSGCOM channel used for this communication is a DIRECT Interrupt channel
     * whose configuration is specified here
     */
    Msgcom_DirectInterruptCfg       directInterruptCfg;

    /**
     * @brief  Handle of the server to which the client will connect to.
     */
    Netfp_ServerHandle              serverHandle;

    /**
     * @brief  Name of the client which should be unique in the domain.
     */
    char                            clientName[NETFP_MAX_CHAR];

    /**
     * @brief  Handle to the name client. This is required to be configured especially
     * if the NETFP servers and clients are present in different realms. This can be set
     * to NULL if the client and server are in the same realm.
     */
    Name_ClientHandle               nameClientHandle;

    /**
     * @brief   Client execution realm
     */
    Netfp_ExecutionRealm            realm;

    /**
     * @brief  PKTLIB Heap handle which is used to allocate networking headers. This heap
     * is used by any 3GPP channels or sockets while sending out packets. The heap should
     * not be shared across any other NETFP client.
     */
    Pktlib_HeapHandle               netHeaderHeapHandle;

    /**
     * @brief  PKTLIB Heap handle which is used to allocate packets which are used for software
     * fragmentation. The heap should be local to the NETFP client and should not be shared
     * with other NETFP clients. The heap should have zero buffer packets only.
     */
    Pktlib_HeapHandle               fragmentHeap;

    /**
     * @brief  Heap handle which is used to allocate messages which are sent from the DSP client
     * to the Server. It is recommended that the heap handle be local to the NETFP client.
     */
    Pktlib_HeapHandle               clientHeapHandle;

    /**
     * @brief  OSAL Malloc
     */
    Osal_NetfpMalloc                malloc;

    /**
     * @brief  OSAL Free
     */
    Osal_NetfpFree                  free;

    /**
     * @brief  OSAL Begin memory access
     */
    Osal_NetfpBeginMemoryAccess     beginMemAccess;

    /**
     * @brief  OSAL End memory access
     */
    Osal_NetfpEndMemoryAccess       endMemAccess;

    /**
     * @brief  OSAL Enter single core critical section
     */
    Osal_NetfpEnterCS               enterCS;

    /**
     * @brief  OSAL Exit single core critical section
     */
    Osal_NetfpExitCS                exitCS;

    /**
     * @brief  OSAL Create Semaphore
     */
    Osal_NetfpCreateSem             createSem;

    /**
     * @brief  OSAL Delete Semaphore
     */
    Osal_NetfpDeleteSem             deleteSem;

    /**
     * @brief  OSAL Post Semaphore
     */
    Osal_NetfpPostSem               postSem;

    /**
     * @brief  OSAL Pend Semaphore
     */
    Osal_NetfpPendSem               pendSem;
}Netfp_ClientConfig;

/**
 * @brief
 *  NETFP Ethernet Configuration
 *
 * @details
 *  The structure is populated by the application and is used to specify
 *  and configure the L2 handle in the NETCP subsystem.
 */
typedef struct Netfp_EthCfg
{
    /**
     * @brief   Source MAC Address.
     */
    uint8_t     srcMac[6];

    /**
     * @brief   Destination MAC Address.
     */
    uint8_t     dstMac[6];

    /**
     * @brief   VLAN Identifier.
     */
    uint16_t    vlan;

    /**
     * @brief   Ethernet type.
     */
    uint16_t    ethertype;
}Netfp_EthCfg;

/**
 * @brief
 *  NETFP Interface Types
 *
 * @details
 *  This is the type of the interface supported by the NETFP module.
 */
typedef enum Netfp_InterfaceType
{
    /**
     * @brief   Standard Ethernet Interface
     */
    Netfp_InterfaceType_ETH  = 0x1,

    /**
     * @brief   VLAN based Interface
     */
    Netfp_InterfaceType_VLAN = 0x2
}Netfp_InterfaceType;

/**
 * @brief
 *  Netfp Option IP Address
 *
 * @details
 *  This is the structure which holds the IP address & Subnet mask information
 *  when an IP address is to be added/deleted from an interface. This is used
 *  in conjunction with the Netfp_Option_ADD_IP & Netfp_Option_DEL_IP
 *
 * @sa
 *  Netfp_Option
 */
typedef struct Netfp_InterfaceIP
{
    /**
     * @brief   This is the IP address which is to be added/deleted from
     * an interface.
     */
    Netfp_IPAddr        ipAddress;

    /**
     * @brief   Subnet mask associated with the IP address.
     */
    Netfp_IPAddr        subnetMask;
}Netfp_InterfaceIP;

/**
 * @brief
 *  Netfp VLAN Priority Map
 *
 * @details
 *  The structure is used to specify the mapping of the NETFP socket
 *  to VLAN Priority bits.
 */
typedef struct Netfp_VLANPriorityMap
{
    /**
     * @brief   The mapping marks NETFP socket priorities to the VLAN pbits
     */
    uint8_t     socketToVLANPriority[NETFP_MAX_SOCK_PRIORITY];
}Netfp_VLANPriorityMap;

/**
 * @brief
 *  L3 QoS Shaping configuration
 *
 * @details
 *  The structure is used to map the NETFP socket priority to the corresponding
 *  L3 QOS queue for shaping.
 */
typedef struct Netfp_L3QoSCfg
{
    /**
     * @brief   L3 QoS configuration status. The configuration fields are NOT
     * valid if the flag is set to 0.
     */
    int32_t     isEnable;

    /**
     * @brief   Flow identifier to be used for the L3 shaping. The flow is used by
     * the L3 QOS block to receive packets into the QOS engine. These packets are
     * then shaped via the QOS firmware
     */
    int32_t     flowId;

    /**
     * @brief   Inner DSCP is mapped to a specific QOS queue for shaping.
     */
    uint16_t    qid[64];
}Netfp_L3QoSCfg;

/**
 * @brief
 *  NETFP Interface Configuration
 *
 * @details
 *  The NETFP Library operates over NETFP interfaces and this information is required
 *  for the creation of the layer2 headers.
 */
typedef struct Netfp_InterfaceCfg
{
    /**
     * @brief   Interface Type
     */
    Netfp_InterfaceType     type;

    /**
     * @brief   Name of the NETFP Interface
     */
    char                    name[NETFP_MAX_CHAR];

    /**
     * @brief   IP Address associated with the interface
     */
    Netfp_IPAddr            ipAddress;

    /**
     * @brief   Subnet Mask associated with the interface
     */
    Netfp_IPAddr            subnetMask;

    /**
     * @brief   MAC address associated with the interface
     */
    uint8_t                 macAddress[6];

    /**
     * @brief   MTU associated with the interface
     */
    uint32_t                mtu;

    /**
     * @brief   VLAN identifier if the interface is VLAN enabled.
     */
    uint32_t                vlanId;

    /**
     * @brief   VLAN priority mapping applicable only if the interface is VLAN enabled.
     */
    Netfp_VLANPriorityMap   vlanMap;

    /**
     * @brief   Indicates whether this interface is purely logical interface or
     * has a physical interface associated with it
     */
    uint32_t                isLogicalInterface;
}Netfp_InterfaceCfg;

/**
 * @brief
 *  NETFP Route Configuration
 *
 * @details
 *  This is the NETFP Routing Entry which describes the route to be used to reach a
 *  specific endpoint.
 */
typedef struct Netfp_RouteCfg
{
    /**
     * @brief   This is the destination IP address which is to used to determine the
     * route. This field is applicable only for destination based routing.
     * - Default route the IP address is all 0's with a valid IP version
     * NOTE: For source based routing; please reset the field to all 0's.
     */
    Netfp_IPAddr        dstIPAddress;

    /**
     * @brief   This is the source IP address which is to used to determine the
     * route. This field is applicable only for source based routing.
     * NOTE: For destination based routing; please reset the field to all 0's.
     */
    Netfp_IPAddr        srcIPAddress;

    /**
     * @brief   Name of the interface on which the destination is located.
     */
    char                ifName[NETFP_MAX_CHAR];

    /**
     * @brief   Next Hop MAC Address
     */
    uint8_t             macAddress[6];
}Netfp_RouteCfg;

/**
 * @brief
 *  Hook configuration
 *
 * @details
 *  The structure is populated by the application and is used to register/unregister
 *  hooks. Hooks allow applications to capture the packet as it flows through the NETFP
 *  module at well-defined entry points.
 */
typedef struct Netfp_HookCfg
{
    /**
     * @brief   Hook entry point
     */
    Netfp_Hook          hook;

    /**
     * @brief   Socket Handle for which the hook is being registered. This should be NULL
     * for the POST_REASSEMBLY Hook; since reassembly is done for the entire system. This
     * needs to be specified for the POST_ROUTING.
     */
    Netfp_SockHandle    sockHandle;

    /**
     * @brief   Application registered hook function
     */
    Netfp_HookFunction  hookFxn;

    /**
     * @brief   Application registered arguments which is passed to the hook function.
     */
    uint32_t            hookArg;
}Netfp_HookCfg;

/**
 * @brief
 *  The structure describes the security policy configuration
 *
 * @details
 *  The structure is populated by the application and is used to create
 *  a security policy.
 */
typedef struct Netfp_SPCfg
{
    /**
     * @brief   Direction in which the security policy is created
     */
    Netfp_Direction     direction;

    /**
     * @brief
     *  This is an opaque handle to the Security Association. If NULL
     *  then it implies that the security policy is non secure and is
     *  not associated with an IPSEC Security association.
     */
    Netfp_SAHandle      saHandle;

    /**
     * @brief   Security policy Identifier.
     */
    uint32_t            spId;

    /**
     * @brief   Range of allowed Ingress Inner IP addresses.
     */
    Netfp_IPAddr        srcIP;

    /**
     * @brief   Ingress Inner IP Mask. Specify allowed range of Ingress Inner IP address.
     */
    uint8_t             srcIPPrefixLen;

    /**
     * @brief   Range of allowed Egress Inner IP addresses.
     */
    Netfp_IPAddr        dstIP;

    /**
     * @brief   Egress Inner IP Mask. Specify allowed range of Egress Inner IP address.
     */
    uint8_t             dstIPPrefixLen;

    /**
     * @brief   IP Protocol (IPv4) / Next Header (IPv6)
     */
    uint8_t             protocol;

    /**
     * @brief   Allowed Source Port range start number.
     */
    uint16_t            srcPortStart;

    /**
     * @brief   Allowed Source Port range end number.
     */
    uint16_t            srcPortEnd;

    /**
     * @brief   Allowed Destination Port range start number.
     */
    uint16_t            dstPortStart;

    /**
     * @brief   Allowed Destination Port range end number.
     */
    uint16_t            dstPortEnd;

    /**
     * @brief   Allowed GTPU ID range start number.
     */
    uint32_t            gtpuIdStart;

    /**
     * @brief   Allowed GTPU ID range end number.
     */
    uint32_t            gtpuIdEnd;

    /**
     * @brief   Allowed DSCP values. Not supported.
     */
    uint64_t            dscpFilter;
}Netfp_SPCfg;

/**
 * @brief
 * Ipsec tunnel lifetime configuration
 *
 * @details
 *  The structure describes the IPSEC lifetime configuration parameters
 *  which describes the IPSEC rekeying configuration
 */
typedef struct Netfp_IPSecLifetime
{
    /**
     * @brief   Number of bytes that may be processed by the SA
     * before SOFT expiry
     */
    uint64_t    softByteLimit;

    /**
     * @brief   Number of bytes that may be processed by the SA
     * before HARD expiry
     */
    uint64_t    hardByteLimit;

    /**
     * @brief   Number of packets that may be processed by the SA
     * before SOFT expiry
     */
    uint64_t    softPacketLimit;

    /**
     * @brief   Number of packets that may be processed by the SA
     * before HARD expiry
     */
    uint64_t    hardPacketLimit;

    /**
     * @brief   Number of seconds after the creation of the SA until
     * its SOFT expiry (Not used)
     */
    uint64_t    softAddExpires;

    /**
     * @brief   Number of seconds after the creation of the SA until
     * its HARD expiry (Not used)
     */
    uint64_t    hardAddExpires;

    /**
     * @brief    Number of seconds after the first use of the SA until
     * its SOFT expiry (Not used)
     */
    uint64_t    softUseExpires;

    /**
     * @brief   Number of seconds after the first use of the SA until
     * its HARD expiry (Not used)
     */
    uint64_t    hardUseExpires;
}Netfp_IPSecLifetime;

/**
 * @brief
 *  NETFP Ingress IPSEC statistics definition
 *
 * @details
 *  This is the NETFP Ingress IPSEC statistics definition.
 */
typedef struct Netfp_InIpSecStats
{
    /**
     * @brief   Number of incoming packets which are discarded due to failed anti-replay check.
     */
    uint32_t    inIpsecDiscReplayFail;

    /**
     * @brief   Number of incoming packets which are discarded due to failed integrity check.
     */
    uint32_t    inIpsecDiscIntegrityFail;

    /**
     * @brief   The number of incoming data processed by the tunnel.
     */
    uint64_t    inIpsecBytes;

    /**
     * @brief   The number of incoming packets for the tunnel.
     */
    uint64_t    inIpsecPkts;
}Netfp_InIpSecStats;

/**
 * @brief
 *  NETFP IPSEC egress statistics definition
 *
 * @details
 *  This is the NETFP IPSEC egress statistics definition.
 */
typedef struct Netfp_OutIpSecStats
{
    /**
     * @brief   Number of packets which are discarded due sequence number overflow.
     */
    uint32_t    outIpsecDiscSeqOv;

    /**
     * @brief   The number of outgoing data processed by the tunnel.
     */
    uint64_t    outIpsecBytes;

    /**
     * @brief   The number of outgoing packets for the tunnel.
     */
    uint64_t    outIpsecPkts;
}Netfp_OutIpSecStats;

/**
 * @brief
 *  NETFP IPSEC statistics definition used by NetFP Proxy
 *
 * @details
 *  This is the NETFP IPSEC statistics definition.
 */
typedef struct Netfp_IpSecStats
{
    /* Handle of SA for which rekey stats are sent */
    Netfp_SAHandle      saHandle;

    /**
     * @brief   SPI for the IPSEC tunnel.
     */
    uint32_t            spi;

    union
    {
        /**
         * @brief   Ingress IPSEC tunnel stats.
         */
        Netfp_InIpSecStats  in;

        /**
         * @brief   Ingress IPSEC tunnel stats.
         */
        Netfp_OutIpSecStats out;
    }u;
}Netfp_IpSecStats;

/**
 * @brief
 *  IPSEC configuration
 *
 * @details
 *  The structure describes the configuration for IPSEC.
 */
typedef struct Netfp_IPSecCfg
{
    /**
     * @brief   IPsec protocol used.
     */
    Netfp_IPSecProto        protocol;

    /**
     * @brief   IPsec transport mode.
     */
    Netfp_IPSecMode         mode;

    /**
     * @brief Authentication mode
     */
    Netfp_IpsecAuthMode     authMode;

    /**
     * @brief Encryption mode
     */
    Netfp_IpsecCipherMode   encMode;

    /**
     * @brief   Flag to turn ON/OFF extended sequence Number
     */
    uint32_t                esnEnabled;

    /**
     * @brief   Size of the session authentication key in bytes. Key is not valid if size is zero.
     */
    uint16_t                keyAuthSize;

    /**
     * @brief   Size of the session encryption key in bytes. Key is not valid if size is zero.
     */
    uint16_t                keyEncSize;

    /**
     * @brief   Size of the session MAC key in bytes.
     */
    uint16_t                keyMacSize;

    /**
     * @brief   IPSec authentication key.
     */
    uint8_t                 keyAuth[NETFP_MAX_IPSEC_KEY_LEN];

    /**
     * @brief   IPSec encryption key.
     */
    uint8_t                 keyEnc[NETFP_MAX_IPSEC_KEY_LEN];

    /**
     * @brief   IPSec lifetime configuration parameters.
     */
    Netfp_IPSecLifetime     lifetime;

    /**
     * @brief   Initial value of the extended sequence Number (lower 32-bit).
     */
    uint32_t                esnLo;

    /**
     * @brief   Initial value of the extended sequence Number (upper 32-bit).
     */
    uint32_t                esnHi;
}Netfp_IPSecCfg;

/**
 * @brief
 *  Response for NAT-T Encapsulation Configuration
 *
 * @details
 *  The structure describes the encapsulation configuration for IPsec NAT-T.
 */
typedef struct Netfp_nattEncapCfg
{
    /**
     * @brief   Source UDP port number used for UDP encapsulation
     */
    uint32_t          srcPort;

    /**
     * @brief   Destination UDP port number used for UDP encapsulation
     */
    uint32_t          dstPort;
}Netfp_nattEncapCfg;

//<fzm>

typedef enum Netfp_OffloadMode
{
    Netfp_OffloadMode_HARDWARE = 0,
    Netfp_OffloadMode_SOFTWARE
} Netfp_OffloadMode;

//</fzm>

/**
 * @brief
 *  Security Association
 *
 * @details
 *  The structure describes the security association configuration which specifies
 *  the IPSEC configuration.
 */
typedef struct Netfp_SACfg
{
    /**
     * @brief   Direction in which the security association is created
     */
    Netfp_Direction         direction;

    /**
     * @brief   Security Parameters Index.
     */
    uint32_t                spi;

    /**
     * @brief   IPSec configuration
     */
    Netfp_IPSecCfg          ipsecCfg;

    /**
     * @brief   Outer IP Destination address:
     *
     *  For direction == Netfp_Direction_INBOUND
     *  This should be configured to match the Outer IP Header destination IP address when
     *  packets are received by the eNodeB.
     *
     *  For direction == Netfp_Direction_OUTBOUND
     *  This is the outer IP header destination IP address. Any packet sent by the eNodeB over
     *  the IPSEC tunnel will have this address in the IP header.
     */
    Netfp_IPAddr            dstIP;

    /**
     * @brief   Outer IP source address:
     *
     *  For direction == Netfp_Direction_INBOUND
     *  This should be configured to match the outer IP Header Source IP address when packets
     *  are received by the eNodeB.
     *
     *  For direction == Netfp_Direction_OUTBOUND
     *  This is the outer IP header source IP address. Any packet sent by the eNodeB over the
     *  IPSEC tunnel will have this address in the IP header.
     */
    Netfp_IPAddr            srcIP;

    /**
     * @brief  Fragmentation Level: This is valid only for OUTBOUND security associations.
     *  This is used to determine if NETFP should create inner or outer fragments.
     *  Default value is Netfp_IPSecFragLevel_INNER_IP
     */
    Netfp_IPSecFragLevel    fragLevel;

    /**
     * @brief   Manual route insertion: This is an advanced feature and has only
     * been provided for debug purposes. The interface handle here specifies the
     * NETFP interface over which the secure packets should be sent out. It is highly
     * recommended that the NETFP Proxy be used to determine the route to be taken.
     * This field should be set to NULL in order for the NETFP Proxy to be consulted.
     *
     * NOTE: This field could be deprecated in future releases
     */
    Netfp_IfHandle          ifHandle;

    /**
     * @brief   Manual route insertion: This is an advanced feature which has been
     * provided for debug purposes. If a valid next hop MAC address is specified then
     * the NETFP will use this to send out all packets. It is highly recommended that
     * the NETFP Proxy be used to determine the route to be taken. This field is only
     * looked into if a valid interface handle is specified.
     *
     * NOTE: This field could be deprecated in future releases
     */
    uint8_t                 nextHopMACAddress[6];

    /**
     * @brief   Specifies the size of the replay window.
     */
    uint32_t                replayWindowSize;

    /**
     * @brief   Specifies the NAT-T UDP encapsulation configuration.
     */
    Netfp_nattEncapCfg      nattEncapCfg;

    Netfp_OffloadMode       offloadMode; //fzm
}Netfp_SACfg;

/**
 * @brief
 *  Inbound Fast path configuration
 *
 * @details
 *  The INBOUND Fast path configuration is used to specify the layer3
 *  endpoints which are required to receive packets by the application.
 */
typedef struct Netfp_InboundFPCfg
{
    /**
     * @brief   Name of the fast path: This should be unique in the system
     */
    char                    name[NETFP_MAX_CHAR];

    /**
     * @brief   Security Policy identifier. If a valid security policy identifier
     * is specified; the fast path configuration will be verified to ensure if the
     * fast path creation is allowed or not. The value NETFP_INVALID_SPID implies
     * that there is no security policy associated.
     */
    uint32_t                spId;

    /**
     * @brief   SPID mode. If no secuirty policy is associated, set the mode to be
     Netfp_SPIDMode_INVALID. If wild carded fast path does not need to match any spid,
     set the mode to be Netfp_SPIDMode_ANY_SECURE. For cases secure policy identifier is
     specified, set the mode to Netfp_SPIDMode_SPECIFIC and set valid spId.
     */
    Netfp_SPIDMode          spidMode;

    /**
     * @brief   Multicast mode: This is applicable only if the fast path is to
     * receive multicast packets.
     */
    Netfp_FastpathMode      fastpathMode;

    /**
     * @brief   Destination IP Address: This should be configured to match the
     * inner IP header destination IP address when packets are received by the
     * eNodeB.
     */
    Netfp_IPAddr            dstIP;

    /**
     * @brief   Source IP Address: This should be configured to match the inner
     * IP source address when packets are received by the eNodeB.
     */
    Netfp_IPAddr            srcIP;
}Netfp_InboundFPCfg;

/**
 * @brief
 *  Outbound Fast path configuration
 *
 * @details
 *  The OUTBOUND Fast path configuration is used to specify the layer3
 *  endpoints which are required to send packets by the application.
 */
typedef struct Netfp_OutboundFPCfg
{
    /**
     * @brief   Name of the fast path: This should be unique in the system
     */
    char                name[NETFP_MAX_CHAR];

    /**
     * @brief   Security Policy identifier. If a valid security policy identifier
     * is specified; the fast path configuration will be verified to ensure if the
     * fast path creation is allowed or not. The value NETFP_INVALID_SPID implies
     * that there is no security policy associated.
     */
    uint32_t            spId;

    /**
     * @brief   Destination IP Address: This should be configured to match the
     * inner IP header destination IP address when packets are sent by the eNodeB.
     */
    Netfp_IPAddr        dstIP;

    /**
     * @brief   Source IP Address: This should be configured to match the inner
     * IP source address when packets are sent by the eNodeB. The field should always
     * be specified
     */
    Netfp_IPAddr        srcIP;

    /**
     * @brief   DSCP Mapping: This is used to map the NETFP socket priority to
     * the DSCP. The mapping is used to mark the DSCP in the Inner IP header. Outer
     * IP DSCP are marked using the marking map specified in the NETFP master.
     *
     *  NOTE: Application must specify only the DSCP 6-bit value and the NETFP
     *  library will construct the TOS byte using this value for the higher order 6
     *  bits of TOS and by setting the lower order 2 bits of TOS to zero.
     */
    uint8_t             dscpMapping[NETFP_MAX_SOCK_PRIORITY];

    /**
     * @brief   Manual route insertion: This is an advanced feature and has only
     * been provided for debug purposes. The interface handle here specifies the
     * NETFP interface over which the packets should be sent out. It is highly
     * recommended that the NETFP Proxy be used to determine the route to be taken.
     * This field should be set to NULL in order for the NETFP Proxy to be consulted.
     *
     * NOTE: This field could be deprecated in future releases
     */
    Netfp_IfHandle      ifHandle;

    /**
     * @brief   Manual route insertion: This is an advanced feature which has been
     * provided for debug purposes. If a valid next hop MAC address is specified then
     * the NETFP will use this to send out all packets. It is highly recommended that
     * the NETFP Proxy be used to determine the route to be taken. This field is only
     * looked into if a valid interface handle is specified.
     *
     * NOTE: This field could be deprecated in future releases
     */
    uint8_t             nextHopMACAddress[6];
}Netfp_OutboundFPCfg;

/**
 * @brief
 *  The structure describes the NetFP flow configuration.
 *
 * @details
 *  The structure is populated by the application and is used to specify
 *  the heaps from where the packets will be picked up the NETCP module and
 *  the MSGCOM channel where these packets will then be placed.
 */
typedef struct Netfp_FlowCfg
{
    /**
     * @brief   Name of the flow
     */
    char                name[NETFP_MAX_CHAR];

    /**
     * @brief   This is the number of heaps which can be used by the NETCP to
     * receive data. This cannot exceed the value of 'NETFP_MAX_HEAP_HANDLE'
     */
    uint8_t             numHeaps;

    /**
     * @brief   This is the number of heaps which can be used by the NETCP to
     * actually receive data. The NETCP module is capable of using different
     * heaps for receiving data depending on the size of the received packet.
     * This is based on the 'threshold' described below.
     */
    Pktlib_HeapHandle   heapHandle[NETFP_MAX_HEAP_HANDLE];

    /**
     * @brief   This is used to specify the max. threshold size of each heap
     * When a packet is received by the NETCP module the information here is
     * used to determine which 'heap' is to be used to get a free packet.
     */
    uint32_t            threshold[NETFP_MAX_HEAP_HANDLE];

    /**
     * @brief   This field specifies the number of bytes that are to be skipped
     * in the SOP buffer before beginning to write the payload.  This value must
     * be less than the minimum size of a buffer in the system
     */
    uint32_t            sopOffset;
}Netfp_FlowCfg;

/**
 * @brief
 *  Frame Protocol channel configuration
 *
 * @details
 *  The structure describes the Frame Protocol channel configuration.
 */
typedef struct Netfp_FrameProtoCfg
{
    /**
     * @brief   Frame protocol CRC verification is enabled.
     */
    uint8_t                 enableRxFpCrc;

    /**
     * @brief Type of Frame Protocol message.
     */
    Netfp_FpType            fpType;

    /**
     * @brief Number of multiplexed transport bearers for the DCH Frame Protocol.
     */
    uint8_t                 numDchBearers;
}Netfp_FrameProtoCfg;

/**
 * @brief
 *  Peer Socket address information
 *
 * @details
 *  This is the peer socket address information which is populated
 *  by the source IP address & source port of the remote peer
 */
typedef struct Netfp_PeerSockAddr
{
    /**
     * @brief   Peer IP addresses.
     */
    Netfp_IPAddr        sin_addr;

    /**
     * @brief   Peer Port number.
     */
    uint16_t            sin_port;
}Netfp_PeerSockAddr;

/**
 * @brief
 *  Netfp Socket Bind Address Information
 *
 * @details
 *  The structure specifies the information which is required to bind
 *  the socket to the specific L4 properties.
 */
typedef struct Netfp_SockAddrBind
{
    /**
     * @brief  Packets meant for the particular socket are marked with the
     * specific application information word specified here.
     */
    uint32_t                appInfo;

    /**
     * @brief  Flow identifier which is used by the PA CPDMA to place
     * the packet into the host.
     */
    uint32_t                flowId;

    /**
     * @brief  This is the inbound fast path handle over which packets are received.
     */
    Netfp_InboundFPHandle   inboundFPHandle;

    /**
     * @brief  Notify function which is invoked when the underlying configuration
     * for the socket has been modified and the socket becomes unoperational. The
     * NETFP server will send notification events to all the NETFP clients once
     * a configuration event occurs. The application is notified through this API
     * The socket can no longer be used. It is highly *recommended* that applications
     * pass this function; passing a NULL is allowed but it would imply that a
     * configuration change would go undetected.
     */
    Netfp_NotifyFunction    notifyFunction;

    /**
     * @brief  The actual QMSS Queue handle where the packet will be placed
     * once it has been received and matched by the NETCP subsystem
     */
    Qmss_QueueHnd           queueHandle;
}Netfp_SockAddrBind;

/**
 * @brief
 *  Netfp Socket Connect Address Information
 *
 * @details
 *  The structure specifies the information which is required to connect
 *  the socket to the specific L4 properties.
 */
typedef struct Netfp_SockAddrConnect
{
    /**
     * @brief  This is the outbound fast path handle over which packets
     * are transmitted and is valid only for connect operations
     */
    Netfp_OutboundFPHandle   outboundFPHandle;
}Netfp_SockAddrConnect;

/**
 * @brief
 *  Netfp Socket Address
 *
 * @details
 *  This is the AF_INET family Socket address data structure
 */
typedef struct Netfp_SockAddr
{
    /**
     * @brief  This is the socket address family: AF_INET or AF_INET6
     */
    Netfp_SockFamily    sin_family;

    /**
     * @brief  Configuration pertinent to the operation
     */
    union op
    {
        /**
         * @brief  Configuration specified during the bind operation only.
         */
        Netfp_SockAddrBind      bind;

        /**
         * @brief  Configuration specified during the connect operation only.
         */
        Netfp_SockAddrConnect   connect;
    }op;

    /**
     * @brief  Port Number: If port number is 2152 then the GTPU Identifier
     * is checked else the GTPU Identifier is not looked at.
     */
    uint16_t            sin_port;

    /**
     * @brief  GTPU Identifier: Only looked if the port number is 2152
     */
    uint32_t            sin_gtpuId;
}Netfp_SockAddr;

/**
 * @brief
 *  Netfp Socket UP Configuration
 *
 * @details
 *  The structure describes the 3GPP protocol configuration. This structure is used when
 *  setting the socket option Netfp_Option_SOCK_3GPP_CFG.
 */
typedef struct Netfp_Sock3GPPCfg
{
    /**
     * @brief  Type of 3GPP protocol.
     */
    Netfp_3GPPProto                     protocol;

    /**
     * @brief  Configuration pertinent to the operation
     */
    union cfg
    {
        /**
         * @brief  Frame Protocol configuration. Valid when protocol == Netfp_3GPPProto_FRAME_PROTOCOL
         */
        Netfp_FrameProtoCfg             frameProtoCfg;
    }cfg;
}Netfp_Sock3GPPCfg;

/**
 * @brief
 *  GTPU Control configuration
 *
 * @details
 *  The structure describes the GTPU control configuration which specifies
 *  the handling of the GTPU control messages
 */
typedef struct Netfp_GTPUControlCfg
{
    /**
     * @brief  This is the flow identifier which is used to receive the GTPU control messages
     * listed below.
     */
    int32_t             gtpuMsgFlowId;

    /**
     * @brief  This is the destination queue to which the GTPU ping request packet
     * are placed.
     */
    Qmss_QueueHnd       gtpuPingReqQueueHnd;

    /**
     * @brief  This is the destination queue to which the GTPU ping response packets
     * are placed
     */
    Qmss_QueueHnd       gtpuPingRespQueueHnd;

    /**
     * @brief  This is the destination queue to which the GTPU error indication
     * packets are sent.
     */
    Qmss_QueueHnd       gtpuErrorIndQueueHnd;

    /**
     * @brief  This is the destination queue to which the GTPU header notification packets
     * are sent.
     */
    Qmss_QueueHnd       gtpuHdrNotifyQueueHnd;

    /**
     * @brief  This is the destination queue to which the GTPU end marker packets are sent.
     */
    Qmss_QueueHnd       gtpuEndMarkerQueueHnd;

    /**
     * @brief  This is the destination queue to which the GTPU parsing error or unsupported messages
     * are sent.
     */
    Qmss_QueueHnd       gtpuErrorQueueHnd;

    /**
     * @brief  This is the destination queue to which the GTPU Id mismatch packets are sent.
     */
    Qmss_QueueHnd       gtpuIdMatchFailQueueHnd;
}Netfp_GTPUControlCfg;

/**
 * @brief
 *  LTE channel bind configuration
 *
 * @details
 *  The structure describes the 3GPP LTE channels binding information
 *  which allows an LTE DRB channel to receive data from the core network.
 */
typedef struct Netfp_LTEChannelBindCfg
{
    /**
     * @brief  Fast Path Handle associated with the socket. This is the ingress
     * handle over which packets are received on the channel.
     */
    Netfp_InboundFPHandle       fpHandle;

    /**
     * @brief  GTPU Identifier which is matched on received packet. Packets
     * matching this GTPU identifier are processed further as described by the
     * rest of the fields in this structure
     */
    uint32_t                    sin_gtpuId;

    /**
     * @brief  Flow identifier which is used by the NETCP module to receive
     * packets which match the GTPU identifier.
     */
    uint32_t                    flowId;

    /**
     * @brief  Notify function which is invoked when the underlying configuration
     * for the socket has been modified and the socket becomes unoperational. The
     * NETFP server will send notification events to all the NETFP clients once
     * a configuration event occurs. The application is notified through this API
     * The socket can no longer be used. It is highly *recommended* that applications
     * pass this function; passing a NULL is allowed but it would imply that a
     * configuration change would go undetected.
     */
    Netfp_NotifyFunction        notifyFunction;

    /**
     * @brief  Initial countC value to be used for the ciphering operations.
     */
    uint32_t                    countC;

    /**
     * @brief  Flag which defines the data path
     * Fast Path Mode:
     *  - Packet is received and passed to the PA for LUT classification
     *  - Packet matches the LUT2 (GTPU identifier match)
     *  - Packet is forwarded by the NETCP directly to the SA for encryption
     *  - Software receives an encrypted packet
     *
     * Non-Fast Path Mode:
     *  - Packet is received and passed to the PA for LUT classification
     *  - Packet matches the LUT2 (GTPU identifier match)
     *  - Packet is passed to the software.
     *  - Software uses the Netfp_encodeDrb API to encrypt the packet
     */
    uint8_t                     enableFastPath;

    /**
     * @brief  ROHC Queue: This is the queue where the packets will be
     * placed after the GTPU identifier match. This is applicable only for
     * non fast path modes.
     */
    Qmss_QueueHnd               chDrbRohc;

    /**
     * @brief  Encryption Queue: This is the queue where the packets will be
     * received after they have been encrypted by the SA.
     *
     * Fast Path Mode:
     *  - Packet is received and passed to the NETCP; resultant encoded packet
     *    is received here.
     *
     * Non-Fast Path Mode:
     *  - Packet is received and passed to the NETCP and after the GTPU identifier
     *    match it is passed to the software [Queue is the sockAddr.bind.queueHandle]
     *  - Software picks the packet from the sockAddr.bind.queueHandle and then uses
     *    the Netfp_encodeDrb API to encrypt the packet. Resultant encoded packet is
     *    received here.
     */
    Qmss_QueueHnd               chDrbEnc;

    /**
     * @brief
     * On target eNB, this field indicates HandOver is in progress for this
     * radio bearer.
     * On Source eNB, this field is not applicable.
     *
     * If Hand Over is in progress, i.e, isHOInProgress = 1, the data arriving at target eNB
     * will be buffered internally by NetFP post GTPU match until HandOver is complete and
     * the application is ready to process the incoming packets. This is done to
     * maintain the ordering of countC used to cipher the packets.
     */
    uint8_t                     isHOInProgress;
}Netfp_LTEChannelBindCfg;

/**
 * @brief
 *  LTE channel connect configuration
 *
 * @details
 *  The structure describes the 3GPP LTE channels connect information which allows an LTE
 *  DRB channel to send data to the core network.
 */
typedef struct Netfp_LTEChannelConnectCfg
{
    /**
     * @brief  Fast Path Handle associated with the socket. This is the egress
     * handle over which packets are transmitted for the channel.
     */
    Netfp_OutboundFPHandle      fpHandle;

    /**
     * @brief  GTPU Identifier which is added to the GTPU header while sending out packets
     * over the channel.
     */
    uint32_t                    sin_gtpuId;

    /**
     * @brief  Quality of service class indication field which is associated
     * with the specific radio bearer.
     */
    uint8_t                     qci;

    /**
     * @brief  This is the DSCP which is to be marked in the IP header while sending
     * out packets from this channel to the core network.
     */
    uint8_t                     dscp;

    /**
     * @brief  This is the flow identifier which is used to pass encrypted packets
     * received from UE to the NETCP for decryption.
     */
    uint32_t                    flowId;

    /**
     * @brief  This is the queue which is used to receive the decrypted packets.
     * Packets for data radio bearers are decrypted using the Netfp_decodeDrb API
     * and the NETCP will place the resultant packets into this queue.
     */
    Qmss_QueueHnd               chDrbDec;
}Netfp_LTEChannelConnectCfg;

/**
 * @brief
 *  Options TLV data structure
 *
 * @details
 *  Specifies the option type, length, value.
 */
typedef struct Netfp_OptionTLV
{
    /**
     * @brief   Option Name
     */
    Netfp_Option        type;

    /**
     * @brief   Option Length
     */
    int32_t             length;

    /**
     * @brief   Option Value
     */
    void*               value;
}Netfp_OptionTLV;

/**
 * @brief
 *  Reassembly statistics
 *
 * @details
 *  This is the NETFP client reassembly statistics
 */
typedef struct Netfp_ReassemblyStats
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
     * @brief   Number of corrupted packets
     */
    uint32_t        numCorruptedPackets;

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
     * @brief   Number of large packets which exceed the NETFP_PASS_MAX_BUFFER_SIZE
     * and were sent to netfp_socket directly
     */
    uint32_t        numLargePacketsFwdToSocket;

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
     * @brief   Number of reassembly contexts freed when there was no free context to allocate
     */
    uint32_t        numFreedOldContext;
    
    /**
     * @brief   Number of fragments dropped due to traffic flow mismatch
     */
    uint32_t        trafficFlowMismatch;

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
     * @brief   Number of fragments dropped because of Layer 2 header errors.
     */
    uint32_t        numL2HeaderError;

    /**
     * @brief   Number of fragments dropped because they weren't received on the same switch port.
     */
    uint32_t        numInPortError;

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

}Netfp_ReassemblyStats;

/**
 * @brief
 *  Socket Statistics
 *
 * @details
 *  Generic counters which keep track of the number of packets and bytes sent by a socket.
 */
typedef struct Netfp_SocketStats
{
    /**
     * @brief   Number of IP packets sent via the socket
     */
    uint64_t        outIPPackets;

    /**
     * @brief   Number of bytes on the IP Layer sent via the socket
     */
    uint64_t        outIPOctets;

    /**
     * @brief   Number of bytes on the Ethernet Layer sent via the socket
     */
    uint64_t        outEthOctets;
}Netfp_SocketStats;

/**
 * @brief
 *  Extended Socket statistics
 *
 * @details
 *  This is the NETFP socket statistics which tracks the statistics for all data
 *  which has been transmitted on the socket.
 */
typedef struct Netfp_ExtendedSocketStats
{
    /**
     * @brief   Number of UDP Packets transmitted
     */
    uint64_t                    numUDPTxPkts;

    /**
     * @brief   Number of GTPU Packets transmitted
     */
    uint64_t                    numGTPUTxPkts;

    /**
     * @brief   Number of packets transmitted in the 6in6 tunnel
     */
    uint64_t                    numTxPkts6in6Tunnel;

    /**
     * @brief   Number of packets transmitted in the 4in4 tunnel
     */
    uint64_t                    numTxPkts4in4Tunnel;

    /**
     * @brief   Number of packets transmitted in the 6in4 tunnel
     */
    uint64_t                    numTxPkts6in4Tunnel;

    /**
     * @brief   Number of packets transmitted in the 4in6 tunnel
     */
    uint64_t                    numTxPkts4in6Tunnel;

    /**
     * @brief   Number of IPv4 fragments created in the software
     */
    uint64_t                    numIPv4Frags;

    /**
     * @brief   Number of IPv6 fragments created in the software
     */
    uint64_t                    numIPv6Frags;

    /**
     * @brief   Number of packets which were rejected using the application supplied hook
     * These packets will not be sent to the NETCP
     */
    uint64_t                    numPktsRejected;

    /**
     * @brief   Number of packets which has UDP encapsulation headers.
     */
    uint64_t                    numPktsUdpEncap;

    /**
     * @brief   Number of IPv4 fragments which could not be created
     */
    uint32_t                    numIPv4FragsFail;

    /**
     * @brief   Number of IPv6 fragments which could not be created
     */
    uint32_t                    numIPv6FragsFail;

    /**
     * @brief   Number of packets with software computed Frame Protocol CRC.
     */
    uint32_t                    numFrameProtoCrc;

    /**
     * @brief   Total number of IP packets sent via the socket.
     */
    Netfp_SocketStats           totalStats;

    /**
     * @brief   Stats which are maintained for each socket priority.
     */
    Netfp_SocketStats           priorityStats[64];

    /**
     * @brief   Transmit layer stats for debugging purpose
     */
    uint32_t                    ifaceTxStats[NETFP_TX_CMD_STATS_MAX];

    /**
     * @brief   Cumulative count of packets that were dropped during re-establishment.
     */
    uint64_t                    reEstDroppedPackets;
}Netfp_ExtendedSocketStats;

/**
 * @brief
 *  User Configuration
 *
 * @details
 *  The structure describes a user configuration which is specified while creating
 *  and registering a user with the NETFP module. The configuration is used to
 *  specify the authentication and ciphering modes and the keys to be used. Once the
 *  user has been created multiple data radio bearers can be attached to the user.
 */
typedef struct Netfp_UserCfg
{
    /**
     * @brief   User identifier
     */
    uint16_t                ueId;

    /**
     * @brief   Authentication mode
     */
    Netfp_3gppAuthMode      authMode;

    /**
     * @brief   SRB Ciphering mode
     */
    Netfp_3gppCipherMode    srbCipherMode;

    /**
     * @brief   DRB Ciphering mode
     */
    Netfp_3gppCipherMode    drbCipherMode;

    /**
     * @brief   SRB authentication key
     */
    uint8_t                 hKeyRrcInt[16];

    /**
     * @brief   SRB encryption key
     */
    uint8_t                 hKeyRrcEnc[16];

    /**
     * @brief   DRB encryption key
     */
    uint8_t                 hKeyUpEnc[16];

    /**
     * @brief   Flow identifier to be used by the SA to pass SRB packets to the application.
     */
    uint32_t                srbFlowId;

    /**
     * @brief   Initial countC value
     */
    uint8_t                 initialCountC;

    /**
     * @brief   SRB1 Encoded Queue: This is the queue where the packets for SRB1 will be placed
     * after they have been encoded
     */
    Qmss_QueueHnd           chSrb1Enc;

    /**
     * @brief   SRB1 Decoded Queue: This is the queue where the packets for SRB1 will be placed
     * after they have been decoded
     */
    Qmss_QueueHnd           chSrb1Dec;

    /**
     * @brief   SRB2 Encoded Queue: This is the queue where the packets for SRB2 will be placed
     * after they have been encoded
     */
    Qmss_QueueHnd           chSrb2Enc;

    /**
     * @brief   SRB2 Decoded Queue: This is the queue where the packets for SRB2 will be placed
     * after they have been decoded
     */
    Qmss_QueueHnd           chSrb2Dec;
}Netfp_UserCfg;

/**
 * @brief
 *  Proxy Registeration configuration
 *
 * @details
 *  NETFP Proxy is a special NETFP client which provides certain important
 *  services. The structure describes the configuration information which needs
 *  to be specified to register the NETFP proxy with the server. This structure
 *  is valid only for NETFP clients and should *NOT* be used by other clients.
 */
typedef struct Netfp_ProxyCfg
{
    /**
     * @brief   NETFP Proxy-Server Interface function
     */
    Netfp_ProxyServerIfFunction      proxyServerInterfaceFunction;
}Netfp_ProxyCfg;

/**
 * @brief
 *  Preclassification configuration
 *
 * @details
 *  Physical interfaces can be configured to bypass the LUT lookup if the received packet
 *  is a broadcast/multicast packet. The configuration information here can be used to
 *  specify the flow and queue where these packets need to be placed.
 */
typedef struct Netfp_PreClassificationCfg
{
    /**
     * @brief   Set to 1 enable broadcast preclassification
     */
    uint8_t             enableBroadcast;

    /**
     * @brief   Set to 1 enable multicast preclassification
     */
    uint8_t             enableMulticast;

    /**
     * @brief   Switch port number belonging to the interface on which the preclassification
     * is being configured
     */
    uint8_t             switchPortNum;

    /**
     * @brief   Flow identifier which is used to receive the broadcast packets. This is applicable
     * only of the broadcast preclassification is enabled
     */
    uint32_t            broadcastFlowId;

    /**
     * @brief   Queue identifier which is used to receive the broadcast packets. This is applicable
     * only of the broadcast preclassification is enabled
     */
    uint32_t            broadcastQueueId;

    /**
     * @brief   Flow identifier which is used to receive the multicast packets. This is applicable
     * only of the multicast preclassification is enabled
     */
    uint32_t            multicastFlowId;

    /**
     * @brief   Queue identifier which is used to receive the multicast packets. This is applicable
     * only of the multicast preclassification is enabled
     */
    uint32_t            multicastQueueId;
}Netfp_PreClassificationCfg;

/**
 * @brief
 *  Port mirror configuration
 *
 * @details
 *  This is the port mirror configuration which can be used to stream packets received from the
 *  source (switch) port to the destination (switch) port. This can be done automatically by the
 *  NETCP subystem without any software intervention.
 */
typedef struct Netfp_PortMirrorCfg
{
    /**
     * @brief   Status which specifies if the port capturing needs to be enabled/disabled
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
}Netfp_PortMirrorCfg;

/**
 * @brief
 *  Port capture configuration
 *
 * @details
 *  This is the port capture configuration which can be used to capture packets received on a
 *  source (switch) port to a destination queue using the specific flow.
 */
typedef struct Netfp_PortCaptureCfg
{
    /**
     * @brief   Status which specifies if the port capturing needs to be enabled/disabled
     */
    uint32_t            isEnable;

    /**
     * @brief   Direction in which the port mirror should work.
     */
    Netfp_Direction     direction;

    /**
     * @brief   Source switch port from where the packets need to be captured
     */
    uint8_t             portToBeCaptured;

    /**
     * @brief   Flow identifier which is to be used to capture these packets.
     */
    uint8_t             flowId;

    /**
     * @brief   Queue identifier on which the packets sent/received will be captured and placed.
     */
    uint16_t            queueId;

    /**
     * @brief   Software information to be added to the packet.
     */
    uint32_t            swInfo;
}Netfp_PortCaptureCfg;

/**
 * @brief
 *  NETFP User Counter Length
 *
 * @details
 *  This is the User counter Length used for PA user statistics.
 */
typedef enum Netfp_UserStatsLen
{
    /**
     * @brief   32bit counter
     */
    Netfp_UserStatsLen_32b,

    /**
     * @brief   64bit counter
     */
    Netfp_UserStatsLen_64b
}Netfp_UserStatsLen;

/**
 * @brief
 *  Ethernet Rule User Stats Configuration
 *
 * @details
 *  This is the User stats configuration for an Ethernet Rule.
 */
typedef struct Netfp_UserStatsCfg
{
    /**
     * @brief   User Stats counter type 32bit or 64bit
     */
     Netfp_UserStatsLen  userStatsLen;

    /**
     * @brief   User Stats type packet or byte
     */
    paUsrStatsTypes_e    userStatsType;
}Netfp_UserStatsCfg;

/**
 * @brief
 *  User Stats
 *
 * @details
 *  The structure defines the user statistics associated which are linked
 *  to Ethernet or IP rules.
 */
typedef struct Netfp_UserStats
{
    /**
     * @brief   User Stats counter type 32bit or 64bit
     */
     uint64_t   userStats[NETFP_MAX_USR_STATS_PER_RULE];
}Netfp_UserStats;

/**
 * @brief
 *  Ethernet Rule configuration
 *
 * @details
 *  This is the Ethernet Rule configuration to be added in PA LUT 1-0.
 */
typedef struct Netfp_EthRuleCfg
{
    /**
     * @brief   Region Name used to allocate LUT1-0 index. LUT1-0 regions
     *          and indices are defined in dts file.
     */
    char       rmRegionName[NETFP_MAX_CHAR];

    /**
     * @brief   type of the Ethernet interface .
     */
    uint16_t   ethType;

    /**
     * @brief   VLAN identifier if the Ethermet type is VLAN .
     */
    uint16_t   vlanId;

    /**
     * @brief   A flag when set to 1 indicates that vlan field will not be match
     * with the given vlan Id. When it is set to 0, vlanId field will be matched
     * for LUT10 rule.
     */
    uint32_t   anyVlanId;

    /**
     * @brief  Flag to indicate if the Rule has Qos enabled.
     */
    uint32_t   enableQos;

    /**
     * @brief   Base queue which is associated with the interface and L2 shaper
     */
    uint16_t    baseQueue;

    /**
     * @brief   Base flow identifier which is the first flow identifier allocated to the interface
     */
    int32_t     baseFlow;

    /**
     * @brief   Base Queue number for qos based routing on the interface
     */
    int32_t     qosBaseQueue;

    /**
     * @brief   Base flow identifier for qos based routing which is the first flow
     * identifier allocated to the interface
     */
    int32_t     qosBaseFlowId;

    /**
     * @brief   Source MAC address to be matched in the cascading entry.
     */
    uint8_t    srcMacAddress[6];

    /**
     * @brief   Destination MAC address to be matched in the cascading entry .
     */
    uint8_t    dstMacAddress[6];

    /**
     * @brief   Ingress Physical Ethernet Interface name.
     */
    char       ingressIfName[NETFP_MAX_CHAR];

    /**
     * @brief   Rule configuration : Ingress switch port number
     */
    uint32_t   inPort;

    /**
     * @brief   Ethernet rule destination. It can be HOST or EMAC
     */
    Netfp_EthRuleDst dstType;

    /**
     * @brief   Egress Physical Ethernet Interface name.
     */
    char       egressIfName[NETFP_MAX_CHAR];

    /**
     * @brief   Rule configuration : Egress switch port number
     */
    uint32_t   outPort;

    /**
     * @brief   Number of User stats associated with this rule. Support both
     * bytes and packets stats
     */
    uint32_t    numUserStats;

    /**
     * @brief   User Statistics configuration for the Ethernet Rule
     */
    Netfp_UserStatsCfg    userStatsCfg[NETFP_MAX_USR_STATS_PER_RULE];
}Netfp_EthRuleCfg;

/**
 * @brief
 *  NETFP Security Context buffer definition
 *
 * @details
 *  Data structure defines the SA-specific software information required for all packets to
 * be delivered to SA.
 */
typedef struct Netfp_SwContext
{
    /**
     * @brief   Rx Flow id used to push packets between SA and PA
     */
    uint32_t       flowId;

    /**
     * @brief   Software security information defined by SA
     */
    Sa_SWInfo_t    swInfo;
}Netfp_SwContext;

/**
 * @brief
 *  NATT configuration on Netfp Server
 *
 * @details
 *  Data structure defines NAT-T configurations from Netfp master
 */
typedef struct Netfp_NattCfg
{
    /**
     * @brief   UDP port used for IPSec NAT-T UDP Encapsulation
     */
    uint32_t    udpPort;

    /**
     * @brief   NAT-T wildcarded LUT entry setting
     */
    uint32_t    wildCardedEntry;
}Netfp_NattCfg;

/**
 * @brief
 *  NETCP System user stats configuration
 *
 * @details
 *  The structure defines the user statistics configuration which is used to
 *  configure the user statistics counters in the NETCP.
 */
typedef struct Netfp_SysUserStatCfg
{
    /**
     * @brief   Total number of user stats. It is the sum of 32b and 64b user stats
     */
    uint32_t    numTotalUserStats;

    /**
     * @brief   Total number of 64b user stats.
     */
    uint32_t    num64bUserStats;
}Netfp_SysUserStatCfg;

/**
 * @brief
 *  Raw Destination Switch port
 *
 * @details
 *  The structure is used to carry information which is required to use the
 *  raw interface to send packets out on to a switch port.
 */
typedef struct Netfp_RawDstSwitchPort
{
    /**
     * @brief   Switch port number on which the packets need to be sent.
     */
    uint32_t    port;
}Netfp_RawDstSwitchPort;

/**
 * @brief
 *  Raw Destination Fast Path
 *
 * @details
 *  The structure is used to carry information which is required to use the
 *  raw interface to send packets to the NETFP Fast Path
 */
typedef struct Netfp_RawDstFastPath
{
    /**
     * @brief   IP version of the packet being sent out.
     */
    Netfp_IPVersion     version;

    /**
     * @brief   L3 Header Offset
     */
    uint16_t            l3Offset;
}Netfp_RawDstFastPath;

/**
 * @brief
 *  Raw Destination
 *
 * @details
 *  The structure describes the destination to which the raw packets are being sent
 */
typedef struct Netfp_RawDst
{
    /**
     * @brief   Raw Destination Type:
     */
    Netfp_RawDstType        type;

    union dst
    {
        /**
         * @brief   Switch Port information applicable only if the destination is switch port
         */
        Netfp_RawDstSwitchPort      switchPort;

        /**
         * @brief   Fast Path information applicable only if the destination is fast path
         */
        Netfp_RawDstFastPath        fastPath;
    }dst;
}Netfp_RawDst;

//fzm-->
/**
 * @brief
 *  Software LUT1_1 extension
 *
 * @details
 *  The structure describes the data to be passed to netfp server from the netfp client to initialize its internal structures
 */
typedef struct Netfp_initSwLut
{
    /**
     * @brief   Handler to keep reference on server side which client is to be notified about new events
     */
    Netfp_ClientHandle client;

    /**
     * @brief   Target queue for LUT1_1 configuration
     */
    Qmss_QueueHnd      queue;

    /**
     * @brief   Target flow for LUT1_1 configuration
     */
    uint32_t           flowId;
} Netfp_initSwLut;
//<--fzm

/**
@}
*/

/** @addtogroup NETFP_DEVICE_FUNCTION
 @{ */

/**
 *  @b Description
 *  @n
 *      Default Fail Information configuration. This specifies the default
 *      action to be taken when there is no match on a specified packet.
 *
 *  @param[out]  ptrFailInfo
 *      Pointer to the fail information populated by this API.
 *
 *  @retval
 *      Not Applicable
 */
extern void Netfp_setupFailInfo(paRouteInfo2_t* ptrFailInfo);

/**
@}
*/

/**********************************************************************
 **************************** EXPORTED API ****************************
 **********************************************************************/

/* NETFP Server API: */
extern Netfp_ServerHandle Netfp_initServer (Netfp_ServerConfig* ptrServerCfg, int32_t* errCode);
extern int32_t Netfp_registerClient (Netfp_ServerHandle serverHandle, const char* clientName,
                                     Msgcom_DirectInterruptCfg* ptrDirectInterruptCfg, int32_t* errCode);
extern int32_t Netfp_isClientActive(Netfp_ServerHandle serverHandle, const char* clientName, int32_t* errCode);
extern int32_t Netfp_deregisterClient(Netfp_ServerHandle netfpServerHandle, const char* clientName, int32_t* errCode);
extern int32_t Netfp_executeServer(Netfp_ServerHandle netfpServerHandle);
extern int32_t Netfp_executeServerTimeout (Netfp_ServerHandle netfpServerHandle, uint32_t timeoutInSec,int32_t* errCode);
extern int32_t Netfp_deleteServer (Netfp_ServerHandle netfpServerHandle, int32_t* errCode);
extern int32_t Netfp_displayClient(Netfp_ServerHandle serverHandle);
extern void Netfp_displayServerGenInfo(Netfp_ServerHandle netfpServerHandle);
extern int32_t Netfp_displayReassemblyContext(Netfp_ServerHandle serverHandle);
extern int32_t Netfp_getNETCPStats(Netfp_ServerHandle serverHandle, paSysStats_t* ptrNETCPStats, int32_t* errCode);
extern int32_t Netfp_initFrameProtoCRC(Netfp_ServerHandle serverHandle, int32_t* errCode);
extern int32_t Netfp_enableFrameProtoCrcServices(Netfp_ServerHandle serverHandle, int32_t* errCode);

/* Advanced NETFP Server API: */
extern int32_t Netfp_initQueueBounce(Netfp_ServerHandle serverHandle, paQueueBounceConfig_t paQueueBounceConfig, int32_t* errCode);
extern int32_t Netfp_initEQOS(Netfp_ServerHandle serverHandle, uint8_t egressDefaultPriority, int32_t* errCode);
extern int32_t Netfp_enableL2Shaper(Netfp_ServerHandle netfpServerHandle, paEQosModeConfig_t* ptrQoSCfg, int32_t* errCode);
extern int32_t Netfp_initPreClassification (Netfp_ServerHandle serverHandle, int32_t* errCode);
extern int32_t Netfp_setupPreclassification (Netfp_ServerHandle serverHandle, Netfp_PreClassificationCfg*, int32_t* errCode);
extern int32_t Netfp_initPortMirroringCapturing(Netfp_ServerHandle netfpServerHandle, int32_t* errCode);
extern int32_t Netfp_setupPortMirror(Netfp_ServerHandle netfpServerHandle,Netfp_PortMirrorCfg* ptrPortMirrorCfg, int32_t* errCode);
extern int32_t Netfp_setupPortCapture(Netfp_ServerHandle netfpServerHandle,Netfp_PortCaptureCfg* ptrPortCaptureCfg, int32_t* errCode);
extern int32_t Netfp_initInterfaceRoutingInfo(Netfp_ServerHandle serverHandle, uint32_t baseFlowId, uint32_t baseQueue, int32_t*  errCode);
extern int32_t Netfp_initNatt(Netfp_ServerHandle serverHandle, Netfp_NattCfg* nattCfg, int32_t* errCode);
extern int32_t Netfp_updateNattInfo(Netfp_ServerHandle  serverHandle, Netfp_NattCfg* nattCfg, int32_t* errCode);

/* NETFP Client API: */
extern Netfp_ServerHandle Netfp_startServer (Name_DBHandle databaseHandle, Name_ClientHandle clientHandle, const char* serverName, int32_t* errCode);
extern int32_t Netfp_isServerStopped (Name_DBHandle databaseHandle, Name_ClientHandle clientHandle, const char* serverName, int32_t* errCode);
extern Netfp_ClientHandle Netfp_initClient (Netfp_ClientConfig* ptrClientCfg, int32_t* errCode);
extern int32_t Netfp_startClient (Netfp_ClientHandle clientHandle, int32_t* errCode);
extern int32_t Netfp_stopClient (Netfp_ClientHandle clientHandle, int32_t* errCode);
extern int32_t Netfp_getServerStatus (Netfp_ClientHandle clientHandle, int32_t* errCode);
extern void Netfp_executeClient(Netfp_ClientHandle netfpClientHandle);
extern int32_t Netfp_deleteClient (Netfp_ClientHandle netfpClientHandle, int32_t* errCode);
extern int32_t Netfp_register3GPPSnow3GServices (Netfp_ClientHandle netfpClientHandle, Netfp_F8Function f8, Netfp_F9Function f9, int32_t* errCode);
extern int32_t Netfp_initMulticastServices (Netfp_ClientHandle netfpClientHandle, int32_t* errCode);

/* IPv6 Exported Utility API: */
extern void    Netfp_convertIP6ToStr (Netfp_IP6N address, char* strIPAddress);
extern int32_t Netfp_convertStrToIP6 (const char* stringIP, Netfp_IP6N* address);

/* Interface API: */
extern Netfp_IfHandle Netfp_createInterface (Netfp_ClientHandle clientHandle, Netfp_InterfaceCfg* ptrInterfaceCfg, int32_t* errCode);
extern int32_t Netfp_deleteInterface (Netfp_ClientHandle clientHandle, Netfp_IfHandle ifHandle, int32_t* errCode);
extern Netfp_IfHandle Netfp_findInterface (Netfp_ClientHandle clientHandle, const char* ifName, Netfp_InterfaceCfg* ptrIfCfg, int32_t* errCode);
extern int32_t Netfp_getIfOpt(Netfp_ClientHandle clientHandle, Netfp_IfHandle ifHandle, Netfp_OptionTLV* ptrOptInfo, int32_t* errCode);
extern int32_t Netfp_setIfOpt(Netfp_ClientHandle clientHandle, Netfp_IfHandle ifHandle, Netfp_OptionTLV* ptrOptInfo, int32_t* errCode);
extern int32_t Netfp_displayInterface(Netfp_ServerHandle serverHandle);
extern int32_t Netfp_setupPhyInterface(Netfp_ServerHandle serverHandle, const char* ifName, uint32_t switchPortNum,
                                       uint8_t* ptrInnerToOuterDSCPMap, int32_t* errCode);
extern int32_t Netfp_renamePhysicalInterface(Netfp_ServerHandle serverHandle, const char* oldIfName, const char* newIfName, int32_t* errCode);

/* Reassembly Service: */
extern int32_t Netfp_registerReassemblyService(Netfp_ClientHandle clientHandle, Netfp_ReassemblyConfig* ptrReassemblyConfig, int32_t* errCode);
extern int32_t Netfp_deregisterReassemblyService (Netfp_ClientHandle clientHandle, int32_t* errCode);
extern int32_t Netfp_cleanupAndReregisterPAReassembly (Netfp_ClientHandle clientHandle, int32_t* errCode); //fzm
extern int32_t Netfp_reassemblyTimerTick(Netfp_ClientHandle clientHandle, int32_t timeout, int32_t* errCode);
extern int32_t Netfp_reassembly(Netfp_ClientHandle clientHandle, int32_t* errCode);
extern int32_t Netfp_getReassemblyStats(Netfp_ClientHandle clientHandle, Netfp_ReassemblyStats* ptrStats, int32_t* errCode);

/* Netfp User Stats API: */
extern int32_t Netfp_initUserStats (Netfp_ServerHandle serverHandle, Netfp_SysUserStatCfg* ptrUserStatCfg, int32_t* errCode);

/* NETFP Proxy Services: */
extern int32_t Netfp_registerProxyService (Netfp_ClientHandle proxyNetfpClientHandle, Netfp_ProxyCfg* ptrProxyCfg, int32_t* errCode);
extern int32_t Netfp_sendProxyResponse(Netfp_ClientHandle clientHandle, Netfp_ProxyServerBulkMsg* ptrProxyServerBulkMsg, int32_t* errCode);
extern int32_t Netfp_asyncUpdate(Netfp_ClientHandle clientHandle, Netfp_ProxyServerInfo* ptrProxyServerInfo, int32_t* errCode);
extern void Netfp_displayProxyServerInterface(Netfp_ServerHandle serverHandle);

/* Flow API: */
extern int32_t Netfp_createFlow (Netfp_ClientHandle clientHandle, Netfp_FlowCfg* ptrFlowCfg, int32_t* errCode);
extern int32_t Netfp_findFlow(Netfp_ClientHandle clientHandle,const char* flowName, int32_t* errCode);
extern int32_t Netfp_deleteFlow(Netfp_ClientHandle clientHandle,const char* ptrFlowName, int32_t* errCode);

/* Fast Path API: */
extern Netfp_InboundFPHandle Netfp_createInboundFastPath(Netfp_ClientHandle clientHandle, Netfp_InboundFPCfg* ptrInboundFPCfg, int32_t* errCode);
extern Netfp_InboundFPHandle Netfp_findInboundFastPath(Netfp_ClientHandle clientHandle, const char* name, int32_t* errCode);
extern int32_t Netfp_isInboundFastPathActive(Netfp_ClientHandle clientHandle, Netfp_InboundFPHandle inboundFPHandle, int32_t* status, int32_t* errCode);
extern int32_t Netfp_deleteInboundFastPath (Netfp_ClientHandle clientHandle, Netfp_InboundFPHandle fpHandle, int32_t* errCode);
extern Netfp_OutboundFPHandle Netfp_createOutboundFastPath(Netfp_ClientHandle clientHandle, Netfp_OutboundFPCfg* ptrOutboundFPCfg, int32_t* errCode);
extern int32_t Netfp_isOutboundFastPathActive(Netfp_ClientHandle clientHandle, Netfp_OutboundFPHandle outboundFPHandle, int32_t* status, int32_t* errCode);
extern Netfp_OutboundFPHandle Netfp_findOutboundFastPath(Netfp_ClientHandle clientHandle, const char* name, int32_t* errCode);
extern int32_t Netfp_deleteOutboundFastPath (Netfp_ClientHandle clientHandle, Netfp_OutboundFPHandle fpHandle, int32_t* errCode);
extern int32_t Netfp_displayInboundFastPath(Netfp_ServerHandle serverHandle);
extern int32_t Netfp_displayOutboundFastPath(Netfp_ServerHandle serverHandle);
extern int32_t Netfp_recomputeRoutes(Netfp_ClientHandle clientHandle, int32_t* errCode);

/* Inbound LUT entry API: */
extern int32_t Netfp_displayLutInfoList(Netfp_ServerHandle serverHandle);

/* L4 Binding API: */
extern int32_t Netfp_displayL4Binding(Netfp_ServerHandle serverHandle);

/* Security Context API: */
extern int32_t Netfp_displaySecurityContext(Netfp_ServerHandle serverHandle);

/* Security Policy API: */
extern int32_t Netfp_addSP(Netfp_ClientHandle clientHandle, Netfp_SPCfg* ptrSPConfig, int32_t* errCode);
extern int32_t Netfp_delSP(Netfp_ClientHandle clientHandle, uint32_t spId, int32_t* errCode);
extern int32_t Netfp_isSPActive(Netfp_ClientHandle clientHandle, uint32_t spId, int32_t* status, int32_t* errCode);
extern int32_t Netfp_displaySP(Netfp_ServerHandle serverHandle, Netfp_Direction direction);

/* Security Association API: */
extern Netfp_SAHandle Netfp_addSA(Netfp_ClientHandle clientHandle, Netfp_SACfg* ptrSAConfig, int32_t* errCode);
extern int32_t Netfp_stopSA (Netfp_ClientHandle clientHandle, Netfp_SAHandle saHandle, int32_t* errCode);
extern int32_t Netfp_delSA (Netfp_ClientHandle clientHandle, Netfp_SAHandle saHandle, int32_t* errCode);
extern Netfp_SAHandle Netfp_rekeySA (Netfp_ClientHandle clientHandle, Netfp_SAHandle oldSAHandle, Netfp_SACfg* ptrSAConfig, int32_t*  errCode);
extern int32_t Netfp_displaySA(Netfp_ServerHandle serverHandle, Netfp_Direction direction);
extern int32_t Netfp_getIPsecStats (Netfp_ClientHandle clientHandle, Netfp_SAHandle saHandle, Netfp_IpSecStats* ipsecStats, int32_t* errCode);
//fzm-->
extern Netfp_SAHandle Netfp_changeRootSA(Netfp_ClientHandle clientHandle, Netfp_SAHandle oldSAHandle, Netfp_SAHandle newSAHandle, int32_t* errCode);
extern int32_t Netfp_initSwLutInfo(Netfp_initSwLut* initSwLut, int32_t* errCode);
extern int32_t Netfp_swSaOffloadDone(Netfp_ClientHandle clientHandle, uint32_t jobID, int32_t status, int32_t* errCode);
//fzm<--
extern int32_t Netfp_getIPSecSwInfo(Netfp_ClientHandle clientHandle, Netfp_SAHandle saHandle, Netfp_SwContext* swContext, int32_t* errCode);

/* Socket API: */
extern Netfp_SockHandle Netfp_socket (Netfp_ClientHandle clientHandle, Netfp_SockFamily family, int32_t* errCode);
extern int32_t Netfp_bind (Netfp_SockHandle sockHandle, Netfp_SockAddr* ptrSockAddr, int32_t* errCode);
extern int32_t Netfp_connect(Netfp_SockHandle sockHandle, Netfp_SockAddr* ptrSockAddr, int32_t* errCode);
extern int32_t Netfp_send(Netfp_SockHandle sockHandle, Ti_Pkt* ptrPayload, uint8_t frameProtoPayloadOffset, int32_t* errCode);
extern int32_t Netfp_send_FZM(Netfp_SockHandle sockHandle, Ti_Pkt* ptrPayload, int32_t* errCode);
extern int32_t Netfp_rawSend(Netfp_ClientHandle clientHandle, Ti_Pkt* ptrPayload, Netfp_RawDst* ptrDst, int32_t* errCode);
extern int32_t Netfp_closeSocket(Netfp_SockHandle sockHandle, int32_t* errCode);
extern int32_t Netfp_getSockOpt(Netfp_SockHandle sockHandle, Netfp_OptionTLV* ptrOptInfo, int32_t* errCode);
extern int32_t Netfp_setSockOpt(Netfp_SockHandle sockHandle, Netfp_OptionTLV* ptrOptInfo, int32_t* errCode);
extern const char*   Netfp_getReasonString(Netfp_Reason reason);
extern int32_t Netfp_getVlanInfo(Netfp_SockHandle socketHandle, uint32_t* vlanId, uint32_t* switchPortNum); //fzm
extern Qmss_QueueHnd Netfp_getSocketQueueByAddress (Netfp_ClientHandle clientHandle, Netfp_IPAddr* ptrNetfpAddr, uint16_t sin_port, uint16_t secure, uint32_t* appInfo); //fzm

/* Receive API: */
extern int32_t Netfp_getPayload(Ti_Pkt* ptrRxPkt, uint8_t gtpuPayload, uint8_t** ptrPayload, uint32_t* payloadLen, Netfp_PeerSockAddr* ptrPeerAddress);

/* LTE 3GPP Services: */
extern int32_t Netfp_configureGTPUControlMessage(Netfp_ClientHandle clientHandle,Netfp_GTPUControlCfg* ptrGTPUControlCfg, int32_t* errCode);
extern Netfp_UserHandle Netfp_createUser(Netfp_ClientHandle clientHandle, Netfp_UserCfg* ptrUserCfg, int32_t* errCode);
extern Netfp_SockHandle Netfp_createLTEChannel(Netfp_UserHandle ueHandle, uint8_t rbId, Netfp_SockFamily family,
                                               Netfp_LTEChannelBindCfg* ptrLTEChannelBindCfg, Netfp_LTEChannelConnectCfg* ptrLTEChannelConnectCfg,
                                               int32_t* errCode);
extern int32_t Netfp_deleteLTEChannel(Netfp_SockHandle socketHandle, int32_t* errCode);
extern int32_t Netfp_deleteUser(Netfp_UserHandle ueHandle, int32_t* errCode);
extern int32_t Netfp_encodeSrb(Netfp_UserHandle ueHandle, Ti_Pkt* ptrPkt, uint32_t countC, uint8_t bearerId);
extern int32_t Netfp_decodeSrb(Netfp_UserHandle ueHandle, Ti_Pkt* ptrPkt, uint32_t countC, uint8_t bearerId);
extern int32_t Netfp_encodeDrb(Netfp_UserHandle ueHandle, Ti_Pkt *ptrPkt, uint32_t countC, uint8_t bearerId);
extern int32_t Netfp_decodeDrb(Netfp_UserHandle ueHandle, Ti_Pkt *ptrPkt, uint32_t countC, uint8_t bearerId);
extern void Netfp_getCountC (Ti_Pkt* ptrPkt, uint32_t* countC);
extern int32_t Netfp_cipher(Netfp_UserHandle ueHandle, uint8_t rbId, Netfp_3gppOperation opType, uint8_t direction,
                            Ti_Pkt* ptrPkt, uint32_t countC, Qmss_QueueHnd f8QueueHnd);
extern int32_t Netfp_generateMacI(Netfp_UserHandle ueHandle, Ti_Pkt* ptrPkt, uint32_t countC, uint8_t rbId, uint8_t direction,
                                  Qmss_QueueHnd f9QueueHnd);
extern Netfp_3gppCipherMode Netfp_getSrbCipherMode(Netfp_UserHandle ueHandle);
extern Netfp_3gppAuthMode Netfp_getSrbAuthMode(Netfp_UserHandle ueHandle);
extern void Netfp_getPacketId(Ti_Pkt* ptrPkt, uint16_t* ueId, uint8_t* qci, uint8_t* rbId, uint8_t* contextId);
extern uint32_t Netfp_getPacketError(Ti_Pkt* ptrPkt);
extern int32_t Netfp_suspendLTEChannel(Netfp_UserHandle ueHandle, uint8_t rbId, uint32_t flowId, int32_t* errCode);
extern Netfp_SockHandle Netfp_reconfigureLTEChannel(Netfp_UserHandle newUeHandle, Netfp_UserHandle oldUeHandle, uint8_t rbId,
                                                uint32_t countC, int32_t* errCode);
extern int32_t Netfp_resumeLTEChannel(Netfp_UserHandle newUeHandle, Netfp_UserHandle oldUeHandle, uint8_t rbId, int32_t* errCode);
extern int32_t Netfp_getSuspendedPacket(Netfp_UserHandle ueHandle, uint8_t rbId, Ti_Pkt** ptrPkt, int32_t* errCode);
extern int32_t Netfp_getLTEChannelOpt(Netfp_UserHandle ueHandle, uint8_t rbId, Netfp_OptionTLV* ptrOptInfo, int32_t* errCode);
extern int32_t Netfp_initiateSourceHandOver(Netfp_UserHandle ueHandle, uint8_t rbId, uint32_t flowId, Qmss_QueueHnd sourceHOQueue, int32_t* errCode);
extern int32_t Netfp_completeTargetHandOver(Netfp_UserHandle ueHandle, uint8_t rbId, uint32_t countC, uint32_t* targetHandOverId, int32_t* errCode);
extern int32_t Netfp_getTargetHandOverPackets(uint32_t targetHandOverId, Ti_Pkt** ptrPkt, int32_t* errCode);

/* Ethernet Rule API: */
extern Netfp_EthRuleHandle Netfp_addEthRule (Netfp_ServerHandle serverHandle, Netfp_EthRuleCfg* ptrEthRuleCfg, int32_t* errCode);
extern int32_t Netfp_delEthRule (Netfp_ServerHandle serverHandle, Netfp_EthRuleHandle ethRuleHandle, int32_t* errCode);
extern int32_t Netfp_getEthRuleStats(Netfp_ServerHandle serverHandle, Netfp_EthRuleHandle ethRuleHandle, Netfp_UserStats* ptrUserStats, int32_t* errCode);
extern int32_t Netfp_displayEthRule(Netfp_ServerHandle serverHandle);

/* Hook API: */
int32_t Netfp_registerHook (Netfp_ClientHandle clientHandle, Netfp_HookCfg* ptrHookCfg, int32_t* errCode);
int32_t Netfp_unregisterHook (Netfp_ClientHandle clientHandle, Netfp_Hook hook, Netfp_SockHandle sockHandle, int32_t* errCode);

//<fzm>
int32_t Netfp_registerSaEventHooks (Netfp_ClientHandle clientHandle, Netfp_SaEventHandler addSaEventHandler, Netfp_SaEventHandler delSaEventHandler, int32_t* errCode);
//</fzm>

#ifdef __cplusplus
}
#endif

#endif /* __NETFP_H__ */

