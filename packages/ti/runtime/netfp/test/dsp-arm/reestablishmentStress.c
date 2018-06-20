/*
 *   @file  reestablishment.c
 *
 *   @brief
 *      Simulated Reestablishment Procedure
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

/* CSL Include Files. */
#include <ti/csl/csl_chip.h>
#include <ti/csl/csl_cacheAux.h>
#include <ti/csl/cslr_cp_ace.h>

/* MCSDK Include Files. */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>

/* SYSLIB Include Files.  */
#include <ti/runtime/msgcom/msgcom.h>
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/netfp/netfp.h>
#include <ti/drv/rm/rm.h>

/**********************************************************************
 ************************ Extern Declarations *************************
 **********************************************************************/

/* NETFP Client Handle: */
extern Netfp_ClientHandle   netfpClientHandle;
extern Pktlib_HeapHandle    netfp10KReceiveHeap;
extern Pktlib_InstHandle    appPktlibInstanceHandle;

extern void appWritebackBuffer(void* ptr, uint32_t size);
extern void appInvalidateBuffer(void* ptr, uint32_t size);

/**********************************************************************
 ************************ Global Variables ****************************
 **********************************************************************/

Qmss_QueueHnd       gtpuQueue;
Qmss_QueueHnd       encodeQueue;
Qmss_QueueHnd       decodeQueue;
Netfp_SockHandle    socketHandle[64][11];
uint32_t            gUEHandle[64];
uint32_t            maxUE  = 64;
uint32_t            maxDRB = 11;

/**********************************************************************
 ********************* Reestablishment Procedure **********************
 **********************************************************************/

/**
 *  @b Description
 *  @n
 *      Utility function which is used to create an interface.
 *
 *  @param[in]  ifname
 *      Interface Name.
 *
 *  @retval
 *      Success -   Interface Handle
 *  @retval
 *      Error   -   NULL
 */
static Netfp_IfHandle Test_createInterface(char* ifname)
{
    Netfp_InterfaceCfg  ifConfig;
    Netfp_IfHandle      ifHandle;
    int32_t             errCode;

    /* Initialize the interface configuration */
    memset ((void *)&ifConfig, 0, sizeof(Netfp_InterfaceCfg));

    /* Populate the configuration block. */
    ifConfig.type                          = Netfp_InterfaceType_ETH;
    ifConfig.mtu                           = 1500;
    ifConfig.ipAddress.ver                 = Netfp_IPVersion_IPV4;
    ifConfig.ipAddress.addr.ipv4.u.a8[0]   = 192;
    ifConfig.ipAddress.addr.ipv4.u.a8[1]   = 168;
    ifConfig.ipAddress.addr.ipv4.u.a8[2]   = 100;
    ifConfig.ipAddress.addr.ipv4.u.a8[3]   = 1;
    ifConfig.subnetMask.ver                = Netfp_IPVersion_IPV4;
    ifConfig.subnetMask.addr.ipv4.u.a8[0]  = 0xFF;
    ifConfig.subnetMask.addr.ipv4.u.a8[1]  = 0xFF;
    ifConfig.subnetMask.addr.ipv4.u.a8[2]  = 0xFF;
    ifConfig.subnetMask.addr.ipv4.u.a8[3]  = 0x0;
    strcpy (ifConfig.name, ifname);

    /* Populate a MAC Address. */
    ifConfig.macAddress[0] = 0x00;
    ifConfig.macAddress[1] = 0x01;
    ifConfig.macAddress[2] = 0x02;
    ifConfig.macAddress[3] = 0x03;
    ifConfig.macAddress[4] = 0x04;
    ifConfig.macAddress[5] = 0x05;

    /* Create & register the interface with the NETFP Library. */
    ifHandle = Netfp_createInterface (netfpClientHandle, &ifConfig, &errCode);
    if (ifHandle == NULL)
        System_printf ("Error: Unable to create the NETFP interface [Error code %d]\n", errCode);

    /* Return the interface handle. */
    return ifHandle;
}

/**
 *  @b Description
 *  @n
 *      Utility function which is used to create a fast path
 *
 *  @param[in]  fpName
 *      Name of the fast path which is to be created
 *  @param[in]  spId
 *      Security Policy Identifier
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  @retval
 *      Success -   Fast Path Handle
 *  @retval
 *      Error   -   NULL
 */
static Netfp_InboundFPHandle Test_createInboundFastPath
(
    char*       fpName,
    uint32_t    spId,
    int32_t*    errCode
)
{
    Netfp_InboundFPCfg  inboundFPCfg;

    /* Initialize the fast path configuration */
    memset ((void *)&inboundFPCfg, 0, sizeof (Netfp_InboundFPCfg));

    /* Initialize spidMode */
    if(spId == NETFP_INVALID_SPID)
        inboundFPCfg.spidMode = Netfp_SPIDMode_INVALID;
    else
        inboundFPCfg.spidMode = Netfp_SPIDMode_SPECIFIC;

    /* Populate the Fast Path configuration. */
    inboundFPCfg.spId                       = (uint32_t)spId;
    inboundFPCfg.dstIP.ver                  = Netfp_IPVersion_IPV4;
    inboundFPCfg.dstIP.addr.ipv4.u.a8[0]    = 192;
    inboundFPCfg.dstIP.addr.ipv4.u.a8[1]    = 168;
    inboundFPCfg.dstIP.addr.ipv4.u.a8[2]    = 100;
    inboundFPCfg.dstIP.addr.ipv4.u.a8[3]    = 1;
    inboundFPCfg.srcIP.ver                  = Netfp_IPVersion_IPV4;
    inboundFPCfg.srcIP.addr.ipv4.u.a8[0]    = 192;
    inboundFPCfg.srcIP.addr.ipv4.u.a8[1]    = 168;
    inboundFPCfg.srcIP.addr.ipv4.u.a8[2]    = 200;
    inboundFPCfg.srcIP.addr.ipv4.u.a8[3]    = 1;
    strcpy (inboundFPCfg.name, fpName);

    /* Create the inbound fast path */
    return Netfp_createInboundFastPath (netfpClientHandle, &inboundFPCfg, errCode);
}

/**
 *  @b Description
 *  @n
 *      Utility function which is used to create a fast path
 *
 *  @param[in]  fpName
 *      Name of the fast path which is to be created
 *  @param[in]  spId
 *      Security Policy Identifier
 *  @param[in]  ifName
 *      Manual routing interface name to be used. Can be NULL for NETFP Proxy
 *  @param[in]  nextHopMACAddress
 *      Manual routing next hop MAC address to be used.
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  @retval
 *      Success -   Fast Path Handle
 *  @retval
 *      Error   -   NULL
 */
static Netfp_OutboundFPHandle Test_createOutboundFastPath
(
    char*               fpName,
    uint32_t            spId,
    char*               ifName,
    uint8_t*            nextHopMACAddress,
    int32_t*            errCode
)
{
    Netfp_OutboundFPCfg outboundFPCfg;
    Netfp_IfHandle      ifHandle;

    /* Initialize the fast path configuration */
    memset ((void *)&outboundFPCfg, 0, sizeof (Netfp_OutboundFPCfg));

    /* Populate the Fast Path configuration. */
    outboundFPCfg.spId                       = (uint32_t)spId;
    outboundFPCfg.dstIP.ver                  = Netfp_IPVersion_IPV4;
    outboundFPCfg.dstIP.addr.ipv4.u.a8[0]    = 192;
    outboundFPCfg.dstIP.addr.ipv4.u.a8[1]    = 168;
    outboundFPCfg.dstIP.addr.ipv4.u.a8[2]    = 200;
    outboundFPCfg.dstIP.addr.ipv4.u.a8[3]    = 1;
    outboundFPCfg.srcIP.ver                  = Netfp_IPVersion_IPV4;
    outboundFPCfg.srcIP.addr.ipv4.u.a8[0]    = 192;
    outboundFPCfg.srcIP.addr.ipv4.u.a8[1]    = 168;
    outboundFPCfg.srcIP.addr.ipv4.u.a8[2]    = 100;
    outboundFPCfg.srcIP.addr.ipv4.u.a8[3]    = 1;
    strcpy (outboundFPCfg.name, fpName);

    /* Was an interface name specified? */
    if (ifName != NULL)
    {
        ifHandle = Netfp_findInterface (netfpClientHandle, ifName, NULL, errCode);
        if (ifHandle == NULL)
        {
            System_printf ("Error: Unable to find the interface '%s' [Error code %d]\n", ifName, *errCode);
            return NULL;
        }

        /* Populate the manual routing section: */
        outboundFPCfg.ifHandle = ifHandle;
        memcpy ((void *)&outboundFPCfg.nextHopMACAddress[0], (void *)nextHopMACAddress, 6);
    }

    /* Create the outbound fast path */
    return Netfp_createOutboundFastPath (netfpClientHandle, &outboundFPCfg, errCode);
}

/**
 *  @b Description
 *  @n
 *      Utility function which is used to create an LTE user
 *
 *  @param[in]  ueId
 *      UE identifier
 *  @param[in]  rxFlowId
 *      Receive Flow identifier to be configured.
 *  @param[in]  encodeQueue
 *      Encode Queue
 *  @param[in]  decodeQueue
 *      Decode Queue
 *
 *  @retval
 *      Success -   User handle
 *  @retval
 *      Error   -   NULL
 */
static Netfp_UserHandle Test_createUser
(
    uint8_t         ueId,
    int32_t         rxFlowId,
    Qmss_QueueHnd   encodeQueue,
    Qmss_QueueHnd   decodeQueue
)
{
    Netfp_UserCfg       userCfg;
    uint8_t             encryptionKey[16];
    uint8_t             integrityKey[16];
    Netfp_UserHandle    ueHandle;
    uint32_t            index;
    int32_t             errCode;

    /* Initialize the user security configuration. */
    memset ((void *)&userCfg, 0, sizeof(Netfp_UserCfg));

    /* Initialize the integrity and ciphering keys */
    for (index = 0; index < 16; index++)
    {
        encryptionKey[index] = (TSCH & 0xFF);
        integrityKey[index]  = (TSCL & 0xFF);
    }

    /* Initialize the user security configuration. */
    memset ((void *)&userCfg, 0, sizeof(Netfp_UserCfg));

    /* Populate the user security configuration. */
    userCfg.authMode         = Netfp_3gppAuthMode_EIA2;
    userCfg.srbCipherMode    = Netfp_3gppCipherMode_EEA2;
    userCfg.drbCipherMode    = Netfp_3gppCipherMode_EEA2;
    userCfg.ueId             = ueId;
    userCfg.srbFlowId        = rxFlowId;
    userCfg.initialCountC    = 0;
    userCfg.chSrb1Enc        = encodeQueue;
    userCfg.chSrb1Dec        = decodeQueue;
    userCfg.chSrb2Enc        = encodeQueue;
    userCfg.chSrb2Dec        = decodeQueue;
    memcpy ((void *)&userCfg.hKeyRrcInt[0],(void *)integrityKey,  sizeof(userCfg.hKeyRrcInt));
    memcpy ((void *)&userCfg.hKeyRrcEnc[0],(void *)encryptionKey, sizeof(userCfg.hKeyRrcEnc));
    memcpy ((void *)&userCfg.hKeyUpEnc[0], (void *)encryptionKey, sizeof(userCfg.hKeyUpEnc));

    /* Create the user */
    ueHandle = Netfp_createUser (netfpClientHandle, &userCfg, &errCode);
    if (ueHandle == NULL)
        System_printf ("Error: LTE Creation User %d failed [Error code %d]\n", ueId, errCode);

    /* Return the user handle. */
    return ueHandle;
}

/**
 *  @b Description
 *  @n
 *      Utility function which is used to create an LTE channel
 *
 *  @param[in]  ueHandle
 *      User handle on which the channel is created
 *  @param[in]  rbId
 *      Radio Bearer identifier
 *  @param[in]  fpInboundV4Handle
 *      Inbound handle
 *  @param[in]  fpOutboundV4Handle
 *      Outbound handle
 *  @param[in]  gtpuChannelHandle
 *      Handle to which non fast path packets will be placed after reception
 *  @param[in]  encodeChannel
 *      Packets where encoded packets are placed
 *  @param[in]  decodeChannel
 *      Packets where decoded packets are placed
 *  @param[in]  rxFlowId
 *      Flow identifier to be used
 *  @param[in]  gtpuIdentifier
 *      GTPU identifier
 *
 *  @retval
 *      Success -   Socket handle
 *  @retval
 *      Error   -   NULL
 */
static Netfp_SockHandle Test_createLTEChannel
(
    Netfp_UserHandle        ueHandle,
    uint8_t                 rbId,
    Netfp_InboundFPHandle   fpInboundV4Handle,
    Netfp_OutboundFPHandle  fpOutboundV4Handle,
    Qmss_QueueHnd           gtpuChannelHandle,
    Qmss_QueueHnd           encodeChannel,
    Qmss_QueueHnd           decodeChannel,
    int32_t                 rxFlowId,
    uint32_t                gtpuIdentifier
)
{
    Netfp_LTEChannelBindCfg     lteChannelBindCfg;
    Netfp_LTEChannelConnectCfg  lteChannelConnectCfg;
    Netfp_SockHandle            lteDRBChannel;
    int32_t                     errCode;

    /* Populate the channel bind configuration: */
    lteChannelBindCfg.flowId          = rxFlowId;
    lteChannelBindCfg.notifyFunction  = NULL;
    lteChannelBindCfg.chDrbRohc       = gtpuChannelHandle;
    lteChannelBindCfg.fpHandle        = fpInboundV4Handle;
    lteChannelBindCfg.sin_gtpuId      = gtpuIdentifier;
    lteChannelBindCfg.countC          = 0;
    lteChannelBindCfg.enableFastPath  = 1;
    lteChannelBindCfg.isHOInProgress  = 0;
    lteChannelBindCfg.chDrbEnc        = encodeChannel;

    /* Populate the channel connect configuration: */
    lteChannelConnectCfg.fpHandle       = fpOutboundV4Handle;
    lteChannelConnectCfg.sin_gtpuId     = gtpuIdentifier;
    lteChannelConnectCfg.qci            = 3;
    lteChannelConnectCfg.dscp           = 0x22;
    lteChannelConnectCfg.flowId         = rxFlowId;
    lteChannelConnectCfg.chDrbDec       = decodeChannel;

    /* Create the DRB LTE channel: */
    lteDRBChannel = Netfp_createLTEChannel (ueHandle, rbId, Netfp_SockFamily_AF_INET,
                                            &lteChannelBindCfg, &lteChannelConnectCfg, &errCode);
    if (lteDRBChannel == NULL)
        System_printf ("Error: Failed to create the LTE channel for %d [Error code %d]\n", rbId, errCode);

    return lteDRBChannel;
}

/**
 *  @b Description
 *  @n
 *      The function is used to montior the SA Context count
 *      and in the case of an error; the code will be halted
 *
 *  @retval
 *      Context Cache Count
 */
static void Test_monitorSA (void)
{
#ifdef DEVICE_K2H
    uint32_t            netcpContextCount;
    CSL_Cp_aceRegs*     ptrSARegs = (CSL_Cp_aceRegs*)CSL_NETCP_CFG_SA_CFG_REGS;
    uint32_t            numSecurityContextError;
    uint32_t            saCipheringQueueCount;

    /* Read the NETCP Context Cache Control Register */
    netcpContextCount = *((uint32_t*)0x020C0100);

    /* Validate: Ensure that the security context does not exceed the MAX permissible. */
    netcpContextCount = CSL_FEXTR (netcpContextCount, 30, 24);
    if (netcpContextCount > 64)
        asm (" swbp 0");

    /* Validate: Ensure that there are no context errors */
    numSecurityContextError = ptrSARegs->SRAM0[129] + ptrSARegs->SRAM0[2177];
    if (numSecurityContextError > 0)
        asm (" swbp 0");

    /* Validate: Is the SA Ciphering queue stuck? */
    saCipheringQueueCount = Qmss_getQueueEntryCount ((Qmss_QueueHnd)647);
    if (saCipheringQueueCount != 0)
        asm (" swbp 0");
#else
    /* This is currently NOT supported and we need to fix this patch. */
    #warn "Porting is needed for the Reestablishment Stress Test to K2L"

#if 0
    uint32_t            netcpContextCount;
    CSL_Cp_aceRegs*     ptrSARegs = (CSL_Cp_aceRegs*)CSL_NETCP_CFG_SA_CFG_REGS;
    uint32_t            numSecurityContextError;
    uint32_t            saCipheringQueueCount;

    /* This is currently NOT supported and we need to fix this patch. */
    #error "Porting the Reestablishment Stress Test to K2L"

    /* Read the NETCP Context Cache Control Register */
    netcpContextCount = *((uint32_t*)0x020C0100);

    /* Validate: Ensure that the security context does not exceed the MAX permissible. */
    netcpContextCount = CSL_FEXTR (netcpContextCount, 30, 24);
    if (netcpContextCount > 64)
        asm (" swbp 0");

    /* Validate: Ensure that there are no context errors */
    numSecurityContextError = ptrSARegs->SRAM0[129] + ptrSARegs->SRAM0[2177];
    if (numSecurityContextError > 0)
        asm (" swbp 0");

    /* Validate: Is the SA Ciphering queue stuck? */
    saCipheringQueueCount = Qmss_getQueueEntryCount ((Qmss_QueueHnd)647);
    if (saCipheringQueueCount != 0)
        asm (" swbp 0");
#endif
#endif
}

/**
 *  @b Description
 *  @n
 *      The function is used to encode & decode
 *
 *  @retval
 *      Success -  0
 *  @retval
 *      Error   -  <0
 */
static void Test_encodeDecodeThread(UArg arg0, UArg arg1)
{
    uint32_t                currentCountC;
    int32_t                 errCode;
    int32_t                 ueIndex;
    int32_t                 rbIndex;
    Ti_Pkt*                 ptrPkt;
    uint8_t*                ptrDataBuffer;
    uint32_t                dataBufferLen;
    int32_t                 displayMessage = 0;
    uint32_t                DRBIterationCounter = 0;
    uint32_t                SRBIterationCounter = 0;
    uint32_t                success;
    Netfp_UserHandle        myUEHandle;

    /* Loop around performing the operations */
    while (1)
    {
        /* Decode/Encode data over all the users: */
        for (ueIndex = 0; ueIndex < maxUE; ueIndex++)
        {
            /***********************************************************************************
             * SRB: Encode/Decode Data
             ***********************************************************************************/
            for (rbIndex = 1; rbIndex < 2; rbIndex++)
            {
                /* Initialize the countC */
                currentCountC = 0x1000 + ueIndex;

                /* Initialize the status of the operation: */
                success = 1;

                /* Get the UE Handle which is to be used: */
                myUEHandle = (Netfp_UserHandle)gUEHandle[ueIndex];

                /* Allocate a dummy packet for the decode operation: */
                ptrPkt = Pktlib_allocPacket (appPktlibInstanceHandle, netfp10KReceiveHeap, 128);
                if (ptrPkt != NULL)
                {
                    /* Get the data buffer: */
                    Pktlib_getDataBuffer (ptrPkt, &ptrDataBuffer, &dataBufferLen);

                    /* Initialize the dummy data: */
                    memset ((void *)ptrDataBuffer, 0x11, dataBufferLen);
                    appWritebackBuffer (ptrDataBuffer, dataBufferLen);

                    /* Decode the SRB: */
                    errCode = Netfp_decodeSrb (myUEHandle, ptrPkt, currentCountC, rbIndex);
                    if (errCode < 0)
                    {
                        System_printf ("Error: Unable to encode the packet for the dummy decode [%d:%d] operation [Error code %d]\n",
                                ueIndex, rbIndex, errCode);
                        success = 0;
                        Pktlib_freePacket (appPktlibInstanceHandle, ptrPkt);
                    }

                    /* Monitor the SA: */
                    Test_monitorSA();
                }
                else
                {
                    /* Error: Unable to allocate memory. This is a failure */
                    success = 0;
                }

                /* Flush all the packets in the decode queue */
                ptrPkt = (Ti_Pkt *) QMSS_DESC_PTR (Qmss_queuePop (decodeQueue));
                while (ptrPkt != NULL)
                {
                    /* Cleanup the packet: */
                    Pktlib_freePacket (appPktlibInstanceHandle, ptrPkt);
                    ptrPkt = (Ti_Pkt *) QMSS_DESC_PTR (Qmss_queuePop (decodeQueue));
                }

                /* Allocate a dummy packet for the encode operation: */
                ptrPkt = Pktlib_allocPacket (appPktlibInstanceHandle, netfp10KReceiveHeap, 128);
                if (ptrPkt != NULL)
                {
                    /* Get the data buffer: */
                    Pktlib_getDataBuffer (ptrPkt, &ptrDataBuffer, &dataBufferLen);

                    /* Initialize the dummy data: */
                    memset ((void *)ptrDataBuffer, 0x22, dataBufferLen);
                    appWritebackBuffer (ptrDataBuffer, dataBufferLen);

                    /* Encode the SRB: */
                    errCode = Netfp_encodeSrb (myUEHandle, ptrPkt, currentCountC, rbIndex);
                    if (errCode < 0)
                    {
                        System_printf ("Error: Unable to encode the packet for the dummy encode [%d:%d] operation [Error code %d]\n",
                                 ueIndex, rbIndex, errCode);
                        success = 0;
                        Pktlib_freePacket (appPktlibInstanceHandle, ptrPkt);
                    }

                    /* Monitor the SA: */
                    Test_monitorSA();
                }
                else
                {
                    /* Error: Unable to allocate memory. This is a failure */
                    success = 0;
                }

                /* Flush all the packets in the encode queue */
                ptrPkt = (Ti_Pkt *) QMSS_DESC_PTR (Qmss_queuePop (encodeQueue));
                while (ptrPkt != NULL)
                {
                    /* Cleanup the packet: */
                    Pktlib_freePacket (appPktlibInstanceHandle, ptrPkt);
                    ptrPkt = (Ti_Pkt *) QMSS_DESC_PTR (Qmss_queuePop (encodeQueue));
                }

                /* Debug Message: */
                if ((displayMessage == 1) && (success == 1))
                    System_printf ("Debug: %d:%d good\n", ueIndex, rbIndex);
                if ((displayMessage == 1) && (success == 0))
                    System_printf ("Debug: %d:%d bad\n", ueIndex, rbIndex);

                /* Was the operation successful? */
                if (success == 1)
                {
                    /* YES: We count only the cases which were successful. */
                    SRBIterationCounter++;
                    if ((SRBIterationCounter % 1000) == 0)
                        System_printf ("Debug: SRB iteration %d completed\n", SRBIterationCounter);
                }
            }

            /***********************************************************************************
             * DRB: Encode/Decode Data
             ***********************************************************************************/
            for (rbIndex = 3; rbIndex < maxDRB; rbIndex++)
            {
                /* Initialize the countC */
                currentCountC = 0x1000 + ueIndex;

                /* Initialize the status of the operation: */
                success = 1;

                /* Allocate a dummy packet for the decode operation: */
                ptrPkt = Pktlib_allocPacket (appPktlibInstanceHandle, netfp10KReceiveHeap, 128);
                if (ptrPkt != NULL)
                {
                    /* Get the data buffer: */
                    Pktlib_getDataBuffer (ptrPkt, &ptrDataBuffer, &dataBufferLen);

                    /* Initialize the dummy data: */
                    memset ((void *)ptrDataBuffer, 0x11, dataBufferLen);
                    appWritebackBuffer (ptrDataBuffer, dataBufferLen);

                    /* Decode the DRB: */
                    errCode = Netfp_decodeDrb ((Netfp_UserHandle)gUEHandle[ueIndex], ptrPkt, currentCountC, rbIndex);
                    if (errCode < 0)
                    {
                        if (errCode != NETFP_EINVAL)
                            System_printf ("Error: Unable to encode the packet for the dummy decode [%d:%d] operation [Error code %d]\n",
                                     ueIndex, rbIndex, errCode);
                        success = 0;
                        Pktlib_freePacket (appPktlibInstanceHandle, ptrPkt);
                    }

                    /* Monitor the SA: */
                    Test_monitorSA();
                }
                else
                {
                    /* Error: Unable to allocate memory. This is a failure */
                    success = 0;
                }

                /* Flush all the packets in the decode queue */
                ptrPkt = (Ti_Pkt *) QMSS_DESC_PTR (Qmss_queuePop (decodeQueue));
                while (ptrPkt != NULL)
                {
                    /* Cleanup the packet: */
                    Pktlib_freePacket (appPktlibInstanceHandle, ptrPkt);
                    ptrPkt = (Ti_Pkt *) QMSS_DESC_PTR (Qmss_queuePop (decodeQueue));
                }

                /* Allocate a dummy packet for the encode operation: */
                ptrPkt = Pktlib_allocPacket (appPktlibInstanceHandle, netfp10KReceiveHeap, 128);
                if (ptrPkt != NULL)
                {
                    /* Get the data buffer: */
                    Pktlib_getDataBuffer (ptrPkt, &ptrDataBuffer, &dataBufferLen);

                    /* Initialize the dummy data: */
                    memset ((void *)ptrDataBuffer, 0x22, dataBufferLen);
                    appWritebackBuffer (ptrDataBuffer, dataBufferLen);

                    /* Encode the DRB: */
                    errCode = Netfp_encodeDrb ((Netfp_UserHandle)gUEHandle[ueIndex], ptrPkt, currentCountC, rbIndex);
                    if (errCode < 0)
                    {
                        if (errCode != NETFP_EINVAL)
                            System_printf ("Error: Unable to encode the packet for the dummy encode [%d:%d] operation [Error code %d]\n",
                                     ueIndex, rbIndex, errCode);
                        success = 0;
                        Pktlib_freePacket (appPktlibInstanceHandle, ptrPkt);
                    }

                    /* Monitor the SA: */
                    Test_monitorSA();
                }
                else
                {
                    /* Error: Unable to allocate memory. This is a failure */
                    success = 0;
                }

                /* Flush all the packets in the encode queue */
                ptrPkt = (Ti_Pkt *) QMSS_DESC_PTR (Qmss_queuePop (encodeQueue));
                while (ptrPkt != NULL)
                {
                    /* Cleanup the packet: */
                    Pktlib_freePacket (appPktlibInstanceHandle, ptrPkt);
                    ptrPkt = (Ti_Pkt *) QMSS_DESC_PTR (Qmss_queuePop (encodeQueue));
                }

                /* Debug Message: */
                if ((displayMessage == 1) && (success == 1))
                    System_printf ("Debug: %d:%d good\n", ueIndex, rbIndex);
                if ((displayMessage == 1) && (success == 0))
                    System_printf ("Debug: %d:%d bad\n", ueIndex, rbIndex);

                /* Was the operation successful? */
                if (success == 1)
                {
                    /* YES: We count only the cases which were successful. */
                    DRBIterationCounter++;
                    if ((DRBIterationCounter % 1000) == 0)
                        System_printf ("Debug: DRB iteration %d completed\n", DRBIterationCounter);
                }
            }
            /* Monitor the SA: */
            Test_monitorSA();
        }
        /* No more log messages [Once we cycle through 1 iteration] */
        displayMessage = 0;
        Task_sleep(2);
    }
}

/**
 *  @b Description
 *  @n
 *      The function is used to test the NETFP Reestablishment API
 *
 *  @retval
 *      Success -  0
 *  @retval
 *      Error   -  <0
 */
static int32_t Test_netfpLTEReestablishment(void)
{
    Netfp_IfHandle          ifHandle;
    Netfp_OutboundFPHandle  fpOutboundHandle;
    Netfp_InboundFPHandle   fpInboundHandle;
    Netfp_FlowCfg           flowCfg;
    int32_t                 myFlowHandle;
    Netfp_UserHandle        reestablishedUeHandle;
    Netfp_UserHandle        tmpUeHandle;
    Netfp_SockHandle        reestablishedLteDRBChannel;
    uint32_t                currentCountC;
    int32_t                 errCode;
    int32_t                 ueIndex;
    int32_t                 rbIndex;
    Netfp_OptionTLV         optCfg;
    uint8_t                 isAllocated;
    uint8_t                 nextHopMACAddress[6] = { 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA };
    uint32_t                gtpuIdentifier = 0xabcd000;
    uint32_t                iterationCounter = 1;
    uint32_t                displayMessage = 1;
    Task_Params             taskParams;

    /**********************************************************************************
     * Test Initialization:
     *********************************************************************************/

    /* Create a dummy interface */
    ifHandle = Test_createInterface("eth1");
    if (ifHandle == NULL)
        return -1;

    /* Create the inbound fast path */
    fpInboundHandle = Test_createInboundFastPath ("Inbound", NETFP_INVALID_SPID, &errCode);
    if (fpInboundHandle == NULL)
    {
        System_printf ("Error: Unable to create the inbound fast path [Error code %d]\n", errCode);
        return -1;
    }

    /* Create the outbound fast path */
    fpOutboundHandle = Test_createOutboundFastPath ("Outbound", NETFP_INVALID_SPID, "eth1", &nextHopMACAddress[0], &errCode);
    if (fpOutboundHandle == NULL)
    {
        System_printf ("Error: Unable to create the outbound fast path [Error code %d]\n", errCode);
        return -1;
    }

    /* Initialize the flow configuration. */
    memset((void *)&flowCfg, 0, sizeof(Netfp_FlowCfg));

    /* Populate the flow configuration: Use the NETFP Data Receive Heap for this purpose. */
    flowCfg.numHeaps      = 1;
    flowCfg.heapHandle[0] = netfp10KReceiveHeap;
    flowCfg.sopOffset     = 0;
    strcpy (flowCfg.name, "Reestablishment-Flow");

    /* Create a test flow which will be used in the unit tests. */
    myFlowHandle = Netfp_createFlow (netfpClientHandle, &flowCfg, &errCode);
    if (myFlowHandle < 0)
    {
        System_printf ("Error: Fast Path Flow Creation Failed [Error code %d]\n", errCode);
        return -1;
    }

    /* Open the encode/decode queue */
    encodeQueue = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated);
    decodeQueue = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated);
    gtpuQueue   = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated);

    /* Cycle through and create all the users */
    for (ueIndex = 0; ueIndex < maxUE; ueIndex++)
    {
        /* Create a user */
        gUEHandle[ueIndex] = (uint32_t)Test_createUser (ueIndex, myFlowHandle, encodeQueue, decodeQueue);
        if (gUEHandle[ueIndex] == 0)
            return -1;

        /* Cycle through and create channels for each DRB */
        for (rbIndex = 3; rbIndex < maxDRB; rbIndex++)
        {
            socketHandle[ueIndex][rbIndex] = Test_createLTEChannel ((Netfp_UserHandle)gUEHandle[ueIndex], rbIndex,
                                                                    fpInboundHandle, fpOutboundHandle,
                                                                    gtpuQueue, encodeQueue, decodeQueue,
                                                                    myFlowHandle, gtpuIdentifier++);
            if (socketHandle[ueIndex][rbIndex] == NULL)
                return -1;
        }
    }

    /* Debug Message: */
    System_printf ("***********************************************************************************\n");
    System_printf ("Debug: Executing the LTE Restablishment Tests [MAX UE: %d MAX Drb: %d]\n", maxUE, maxDRB);
    System_printf ("Debug: Encode Queue %x Decode Queue %x GTPU Queue %x\n", encodeQueue, decodeQueue, gtpuQueue);
    System_printf ("Debug: SRB/DRB Thread Decode/Encode with Reestablishment\n");
    System_printf ("Debug: SRB Ciphering Mode: %s Authentication Mode: %s\n",
            (Netfp_getSrbCipherMode((Netfp_UserHandle)gUEHandle[0]) == Netfp_3gppCipherMode_EEA0) ? "None" : "AES",
            (Netfp_getSrbAuthMode((Netfp_UserHandle)gUEHandle[0])   == Netfp_3gppAuthMode_EIA0)   ? "None" : "AES");

    /* Create Task for encode/decode task */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 8*1024;
    taskParams.priority  = 12;
    Task_create(Test_encodeDecodeThread, &taskParams, NULL);

    /**********************************************************************************
     * Reestablishment Procedure:
     *********************************************************************************/
    while (1)
    {
        for (ueIndex = 0; ueIndex < maxUE; ueIndex++)
        {
            /**********************************************************************************
             * Suspend the DRB Channels for the specified user:
             *********************************************************************************/
            for (rbIndex = 3; rbIndex < maxDRB; rbIndex++)
            {
                if (Netfp_suspendLTEChannel ((Netfp_UserHandle)gUEHandle[ueIndex], rbIndex, myFlowHandle,  &errCode) < 0)
                {
                    System_printf ("Error: Suspend Channel Failed (Error %d)\n", errCode);
                    continue;
                }
                /* Monitor the SA: */
                Test_monitorSA ();
            }

            /**********************************************************************************
             * Recreate the user with a new configuration:
             *********************************************************************************/
            reestablishedUeHandle = Test_createUser (ueIndex, myFlowHandle, encodeQueue, decodeQueue);
            if (reestablishedUeHandle == NULL)
                return -1;

            /* Monitor the SA: */
            Test_monitorSA ();

            /**********************************************************************************
             * Reconfigure the LTE DRB channel:
             *********************************************************************************/
            for (rbIndex = 3; rbIndex < maxDRB; rbIndex++)
            {
                /* For fast path bearers SA maintains the CountC value. We are resuming countC in this test.
                 * Get the countc value from SA */
                currentCountC = 0xFFFF;

                /* Count C is being resumed; so we specify the last countC which we had used. */
                optCfg.type   = Netfp_Option_COUNTC;
                optCfg.length = 4;
                optCfg.value  = (void*)&currentCountC;
                if (Netfp_getLTEChannelOpt ((Netfp_UserHandle)gUEHandle[ueIndex], rbIndex, &optCfg, &errCode) < 0)
                {
                    /* Failed to get Countc value. */
                    System_printf ("Error: Failed to get countC [Error code %d]\n", errCode);
                    return -1;
                }

                /* Reconfigure the LTE channel: */
                reestablishedLteDRBChannel = Netfp_reconfigureLTEChannel(reestablishedUeHandle, (Netfp_UserHandle)gUEHandle[ueIndex],
                                                                         rbIndex, currentCountC, &errCode);
                if (reestablishedLteDRBChannel == NULL)
                {
                    System_printf ("Error: Reconfigure LTE channel Failed (Error %d)\n", errCode);
                    return -1;
                }

                /* Monitor the SA: */
                Test_monitorSA ();

                /**********************************************************************************
                 * Resume the LTE Channel:
                 *********************************************************************************/
                if (Netfp_resumeLTEChannel(reestablishedUeHandle, (Netfp_UserHandle)gUEHandle[ueIndex], rbIndex, &errCode) < 0)
                {
                    System_printf ("Error: Resume LTE channel Failed (Error %d)\n", errCode);
                    return -1;
                }

                /* Monitor the SA: */
                Test_monitorSA ();

                /* Delete the old LTE channel: */
                if (Netfp_deleteLTEChannel (socketHandle[ueIndex][rbIndex] , &errCode) < 0)
                {
                    System_printf ("Error: Unable to delete the channel [Error code %d]\n", errCode);
                    return -1;
                }

                /* Monitor the SA: */
                Test_monitorSA ();

                /* Remember the new socket handle: */
                socketHandle[ueIndex][rbIndex] = reestablishedLteDRBChannel;
            }

            /* Debug Message: */
            if (displayMessage == 1)
                System_printf ("Debug: Reestablished UE(%d) From 0x%x to 0x%x\n", ueIndex, gUEHandle[ueIndex], (uint32_t)reestablishedUeHandle);

            /* Rememember the new UE Handle: */
            tmpUeHandle        = (Netfp_UserHandle)gUEHandle[ueIndex];
            gUEHandle[ueIndex] = (uint32_t)reestablishedUeHandle;

            /* Delete the older user */
            if (Netfp_deleteUser (tmpUeHandle, &errCode) < 0)
            {
                System_printf ("Error: Unable to delete the user [Error code %d]\n", errCode);
                return -1;
            }

            /* Monitor the SA: */
            Test_monitorSA ();
        }

        /* Enable the log messages: */
        displayMessage = 0;
        System_printf ("Debug: Reestablishment %d iterations done for MAX Users %d\n", iterationCounter++, maxUE);
    }
}

/**
 *  @b Description
 *  @n
 *      Entry point to the test task which executes the unit tests.
 *
 *  @retval
 *      Not Applicable.
 */
void Test_reestablishmentThread(UArg arg0, UArg arg1)
{
    System_printf ("*******************************************************\n");
    System_printf ("**** Simulated Reestablishment Procedure Testing ******\n");
    System_printf ("*******************************************************\n");

    /* Invoke the procedure: */
    if (Test_netfpLTEReestablishment() < 0)
        System_printf ("Error: Reestablishment Procedure Failed\n");

    return ;
}

