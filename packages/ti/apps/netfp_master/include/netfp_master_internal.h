/**
 *   @file  master_internal.h
 *
 *   @brief
 *      NETFP Master internal header file
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
#ifndef __MASTER_INTERNAL_H__
#define __MASTER_INTERNAL_H__

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
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/un.h>

/* MCSDK Include files. */
#include <ti/csl/csl_cache.h>
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/drv/sa/salld.h>
#include <ti/runtime/hplib/hplib.h>

/* SYSLIB Include Files */
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/msgcom/msgcom.h>
#include <ti/runtime/netfp/netfp.h>
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/apps/netfp_master/include/listlib.h>
#include <ti/apps/netfp_master/netfp_master.h>

/**********************************************************************
 ********************* NETFP Master Definitions ***********************
 **********************************************************************/

/**
 * @brief   TODO: This is the maximum DSCP values which can be supported
 * This is a constant and should be defined in the netfp_net.h
 */
#define NETFP_MASTER_MAX_DSCP_ENTRY     64

/**
 * @brief   TODO: This is the maximum VLAN priorities which can be supported
 * This is a constant and should be defined in the netfp_net.h
 */
#define NETFP_MASTER_MAX_VLAN_ENTRY     8

/**
 * @brief   TODO: This is the maximum number of flows which can be supported by
 * the NETCP CPDMA.
 */
#define NETFP_MASTER_MAX_FLOW           64

/**
 * @brief   This is the maximum number of queues which can be programmed into the
 * eQoS block.
 */
#define NETFP_MASTER_MAX_QUEUE          256

/**
 * @brief   This is the maximum size of an ethernet packet.
 */
#define ETH_MAX_PKT_SIZE                1536

/**********************************************************************
 ******************** NETFP Master Enumerations ***********************
 **********************************************************************/

/**
 * @brief
 *  Enumeration for logging levels
 *
 * @details
 *  Log messages are logged at the following levels.
 */
typedef enum NetfpMaster_LogLevel
{
    /**
    * @brief  Verbose messages
    */
    NetfpMaster_LogLevel_VERBOSE, //fzm

    /**
     * @brief  Debug messages
     */
    NetfpMaster_LogLevel_DEBUG,

    /**
     * @brief  Informational messages
     */
    NetfpMaster_LogLevel_INFO,

    /**
     * @brief  Fatal Error Messages
     */
    NetfpMaster_LogLevel_ERROR
}NetfpMaster_LogLevel;

/**
 * @brief
 *  Routing mode
 *
 * @details
 *  Enumeration which describes the Enhanced QOS routing mode configured.
 */
typedef enum NetfpMaster_RoutingMode
{
    /**
     * @brief  DSCP Routing mode
     */
    NetfpMaster_RoutingMode_DSCP    = 0x1,

    /**
     * @brief  DPBIT Routing mode
     */
    NetfpMaster_RoutingMode_DPBIT   = 0x2
}NetfpMaster_RoutingMode;

/**********************************************************************
 ******************** NETFP Master Structures *************************
 **********************************************************************/

/**
 * @brief
 *  NETFP Master QoS Entry
 *
 * @details
 *  The structure describes the flow and queue to be used to send data with a specific
 *  DSCP value/VLAN priority to the eQOS PDSP for shaping.
 */
typedef struct NetfpMaster_QoSEntry
{
    /**
     * @brief   Flow offset to be used to pass packets to the eQoS PDSP.
     */
    uint8_t         flowOffset;

    /**
     * @brief   Queue offset to be used to pass packets to the eQoS PDSP.
     */
    uint8_t         queueOffset;
}NetfpMaster_QoSEntry;

/**
 * @brief
 *  NETFP Master flow block
 *
 * @details
 *  The structure describes the flow block which is used to configure the eQoS
 */
typedef struct NetfpMaster_FlowBlock
{
    /**
     * @brief   Flow offset to be used to configure the eQoS
     */
    uint8_t             flowOffset;

    /**
     * @brief   Number of descriptors associated with the flow
     */
    uint32_t            numDescriptors;

    /**
     * @brief   Flow identifier
     */
    int32_t             flowId;

    /**
     * @brief   Heap handle associated with the flow
     */
    Pktlib_HeapHandle   heapHandle;
}NetfpMaster_FlowBlock;

/**
 * @brief
 *  NETFP Master Port capture
 *
 * @details
 *  The structure describes the port capture block
 */
typedef struct NetfpMaster_PortCapture
{
    /**
     * @brief   Links to other NETFP master port capturing nodes
     */
    NetfpMaster_ListNode                links;

    /**
     * @brief   Port capture request which was used to create this node
     */
    NetfpMaster_SetPortCaptureRequest   portCaptureRequest;

    /**
     * @brief   Flow identifier used for capturing.
     */
    int32_t                             flowId;
}NetfpMaster_PortCapture;

/**
 * @brief
 *  NETFP Master Entity block
 *
 * @details
 *  The structure describes entities which have been registered with the
 *  NETFP master for notifications.
 */
typedef struct NetfpMaster_Entity
{
    /**
     * @brief   Links to other NETFP entities
     */
    NetfpMaster_ListNode        links;

    /**
     * @brief   UNIX socket address of the entity.
     */
    struct sockaddr_un          address;
}NetfpMaster_Entity;

/**
 * @brief
 *  NETFP Master Interface control block
 *
 * @details
 *  The NETFP master interface control block which keeps track of the
 *  QoS configuration block maintained for each interface
 */
typedef struct NetfpMaster_IfBlock
{
    /**
     * @brief   Links to other NETFP master interface blocks
     */
    NetfpMaster_ListNode        links;

    /**
     * @brief   Name of the interface
     */
    char                        name[NETFP_MASTER_MAX_CHAR];

    /**
     * @brief   Routing mode configured in the NETCP eQoS.
     */
    NetfpMaster_RoutingMode     routingMode;

    /**
     * @brief   Priority Override flag valid only if the routing mode is DP-Bit.
     */
    uint32_t                    priorityOverride;

    /**
     * @brief   Default Priority to be used if the packet was not marked
     */
    uint32_t                    defaultFwdPriority;

    /**
     * @brief   Base queue which is associated with the interface and L2 shaper
     */
    uint16_t                    baseQueue;

    /**
     * @brief   Switch port to be used to send the packet
     */
    uint32_t                    switchPort;

    /**
     * @brief   Maximum flow offset requested
     */
    uint32_t                    maxFlowOffset;

    /**
     * @brief   Base flow identifier which is the first flow identifier allocated to the interface
     */
    int32_t                     baseFlowId;

    /**
     * @brief   Maximum queue offset requested
     */
    uint32_t                    maxQueueOffset;

    /**
     * @brief   Broadcast preclassification enable status: Set to 1 to indicate that broadcast preclassification
     * was configured and that the broadcast packets will be routed using the specified flow identifier to the
     * queue.
     */
    uint32_t                    broadcastPreclassification;

    /**
     * @brief   Broadcast preclassification flow identifier which is used to specify the flow which is to be used
     * to receive the broadcast packets.
     */
    uint32_t                    broadcastPreclassificationFlowId;

    /**
     * @brief   Broadcast preclassification queue identifier which is used to specify the queue which is to be used
     * to place the received broadcast packets.
     */
    uint32_t                    broadcastPreclassificationQueueId;

    /**
     * @brief   Multicast preclassification enable status: Set to 1 to indicate that multicast preclassification
     * was configured and that the multicast packets will be routed using the specified flow identifier to the
     * queue.
     */
    uint32_t                    multicastPreclassification;

    /**
     * @brief   Multicast preclassification flow identifier which is used to specify the flow which is to be used
     * to receive the multicast packets.
     */
    uint32_t                    multicastPreclassificationFlowId;

    /**
     * @brief   Multicast preclassification queue identifier which is used to specify the queue which is to be used
     * to place the received multicast packets.
     */
    uint32_t                    multicastPreclassificationQueueId;

    /**
     * @brief  Queue number allocated for the eQOS usage. These are actual base queue number + the offset
     */
    uint16_t                    eqosQueueHandle[NETFP_MASTER_MAX_QUEUE];

    /**
     * @brief   DSCP entry which is used to configure the NETCP enhanced QoS
     */
    NetfpMaster_QoSEntry        dscpMap[NETFP_MASTER_MAX_DSCP_ENTRY];

    /**
     * @brief   VLAN priority entry which is used to configure the NETCP enhanced QoS
     */
    NetfpMaster_QoSEntry        vlanMap[NETFP_MASTER_MAX_VLAN_ENTRY];

    /**
     * @brief   This specifies the mapping between the inner DSCP to outer DSCP and is specified
     * for each interface
     */
    uint8_t                     innerToOuterDSCPMap[NETFP_MASTER_MAX_DSCP_ENTRY];

    /**
     * @brief   Flow blocks which keep track of the maximum number of flows associated
     * with the interface
     */
    NetfpMaster_FlowBlock       flowBlock[NETFP_MASTER_MAX_FLOW];
}NetfpMaster_IfBlock;

/**
 * @brief
 *  NETFPD Master Control Block
 *
 * @details
 *  The NETFPD Master control block which keeps track of all the
 *  persistent information used by the NETFPD application
 */
typedef struct NetfpMaster_MCB
{
    /**
     * @brief   Name of the configuration file
     */
    char                        cfgFile[PATH_MAX];

    /**
     * @brief Pipe handler to enable the handling of signals.
     */
    int32_t                     signalPipe[2];

    /**
     * @brief   State of the enhanced QOS
     */
    uint32_t                    enableEQOS;

    /**
     * @brief   <fzm> The flag indicates that eQoS will be disabled,
     * no matter what is set in the config file
     */
    uint32_t                    forceDisableEQOS; //fzm

    /**
     * @brief   This is the CPPI handle to the NETCP CPDMA block
     */
    Cppi_Handle                 passCPDMAHandle;

    /**
     * @brief   State of the reassembly handling
     */
    uint32_t                    reassemblyHandling;

    /**
     * @brief  Number of descriptors configured for the reassembly process.
     */
    uint32_t                    numReassemblyDescriptors;

    /**
     * @brief  Reassembly timeout which is used to cleanup stale fragments which
     * were not reassembled successfully.
     */
    uint32_t                    reassemblyTimeout;

    /**
     * @brief  Number of reassembly context which are used to track all the active
     * fragments which are in process of being reassembled.
     */
    uint32_t                    numReassemblyContext;

    /**
     * @brief  Default Host Priority all non-ip packets generated by the host.
     */
    uint32_t                    defaultHostPriority;

    /**
     * @brief   Physical interface base Queue
     */
    uint32_t                    interfaceBaseQueue;

    /**
     * @brief   Physical interface base Flow id
     */
    uint32_t                    interfaceBaseFlow;

    /**
     * @brief   NETFP User statistics configuration
     */
    Netfp_SysUserStatCfg        userStatCfg;

    /**
     * @brief   IPSec NAT-T UDP encapsulation settings
     */
    Netfp_NattCfg               nattCfg;

    /**
     * @brief  Memory region handle allocated for the NETFP master
     */
    Qmss_MemRegion              memoryRegionHandle;

    /**
     * @brief RM Client Name
     */
    char                        rmClientName[128];

    /**
     * @brief Named resource instance identifier.
     */
    uint32_t                    nrInstanceId;

    /**
     * @brief NETFP Master Logging level.
     */
    NetfpMaster_LogLevel        logLevel;

    /**
     * @brief   Database handle used by the master
     */
    Name_DBHandle               databaseHandle;

    /**
     * @brief   Handle to the internal NETFP Server
     */
    Netfp_ServerHandle          netfpServerHandle;

    /**
     * @brief   Handle to the internal NETFP client
     */
    Netfp_ClientHandle          netfpClientHandle;

    /**
     * @brief Global System configuration handle
     */
    Resmgr_SysCfgHandle         handleSysCfg;

    /**
     * @brief   PKTLIB Instance handle
     */
    Pktlib_InstHandle           pktlibInstanceHandle;

    /**
     * @brief   MSGCOM Instance populated and used by the NETFP server.
     */
    Msgcom_InstHandle           msgcomInstanceHandle;

    /**
     * @brief NETFP Master execution status
     */
    uint32_t                    executionStatus;

    /**
     * @brief PKTLIB Heap Handle used by the master
     */
    Pktlib_HeapHandle           masterHeapHandle;

    /**
     * @brief   Global flag which enables system wide preclassification: This flag is set to 1
     * if any interface in the configuration file has preclassification enabled.
     */
    int32_t                     enableGlobalPreclassification;

    /**
     * @brief   The outer IP channel which receives outer IPv4 fragments.
     */
    MsgCom_ChHandle             outerIPChannel;

    /**
     * @brief   The inner IP channel which receives inner IPv4 fragments.
     */
    MsgCom_ChHandle             innerIPChannel;

    /**
     * @brief   Outer IP event associated with the channel
     */
    uint32_t                    outerIPEvent;

    /**
     * @brief   Inner IP event associated with the channel
     */
    uint32_t                    innerIPEvent;

    /**
     * @brief   Management socket used to send/receive messages from other applications
     */
    int32_t                     masterMgmtSocket;

    /**
     * @brief   Global flag used to indicate support for offloading of
     * verfication and computation of Frame Protocol CRC
     */
    uint32_t                    frameProtoCrcOffload;

    /**
     * @brief Pointer to the interface list
     */
    NetfpMaster_IfBlock*        ptrInterfaceList;

    /**
     * @brief Pointer to the entity list
     */
    NetfpMaster_Entity*         ptrEntityList;

    /**
     * @brief Pointer to the port capture list.
     */
    NetfpMaster_PortCapture*    ptrPortCaptureList;

    /**
     * @brief Pointer to the port capture list.
     */
    paQueueBounceConfig_t    paQueueBounceConfig;
}NetfpMaster_MCB;

/************************************************************************************
 * Exported API:
 ************************************************************************************/

/* Logging API: */
extern void NetfpMaster_log (NetfpMaster_LogLevel level, const char* fmt, ...) __attribute__ ((format (printf, 2, 3)));
// <fzm>
extern void NetfpMaster_dump(NetfpMaster_LogLevel level, const char* fmt, ...) __attribute__ ((format (printf, 2, 3)));
// </fzm>
extern uint32_t NetfpMaster_roundNumberDescriptors (uint32_t numDesc);

/* Preclassification API: */
extern int32_t NetfpMaster_setupPreclassification (NetfpMaster_IfBlock* ptrNetfpIfBlock, int32_t* errCode);

/* Management: */
extern int32_t NetfpMaster_initMgmt(NetfpMaster_MCB* ptrNetfpMasterMCB);
extern int32_t NetfpMaster_processMgmt(NetfpMaster_MCB* ptrNetfpMasterMCB);
extern int32_t NetfpMaster_deinitMgmt(NetfpMaster_MCB* ptrNetfpMasterMCB);

/* eQoS exported API: */
extern int32_t NetfpMaster_initQoS(NetfpMaster_MCB* ptrNetfpMasterMCB);
extern int32_t NetfpMaster_setupL2Shaper(NetfpMaster_MCB* ptrNetfpMasterMCB, NetfpMaster_IfBlock* ptrNetfpIfBlock,int32_t* errCode);
extern int32_t NetfpMaster_deInitQoS(NetfpMaster_MCB* ptrNetfpMasterMCB);

#endif /* __MASTER_INTERNAL_H__ */

