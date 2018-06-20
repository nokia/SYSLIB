/**
 *   @file  ipsec.c
 *
 *   @brief
 *      NETFP Proxy IPSEC Implementation. The function interfaces with
 *      the IPSEC snooper.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2015 Texas Instruments, Inc.
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
#include <stdarg.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <pthread.h>
#include <getopt.h>
#include <sys/ioctl.h>
#include <dlfcn.h>
#include <dirent.h>

/* NETFP Proxy includes */
#include <ti/apps/netfp_proxy/include/netfp_proxy_pvt.h>
#include <ti/apps/netfp_proxy/netfp_proxy.h>
#include <ti/apps/netfp_proxy/include/ipsecmgr_snoop.h>

/**********************************************************************
 ************************ Local Structures ****************************
 **********************************************************************/

/**
 * @brief
 *  NETFP Proxy Security Associtation Info
 *
 * @details
 *  Holds info regarding security associations
 *  offloaded using Proxy.
 */
typedef struct NetfpProxy_SAInfo
{
    /**
     * @brief   Link to other SA Info objects
     */
    List_Node               list_n;

    /**
     * @brief   Security Parameters Index (SPI)
     */
    uint32_t                spi;

    /**
     * @brief   SA Handle
     */
    Netfp_SAHandle          saHandle;

    /**
     * @brief   Reference count
     * SAs can be shared by multiple policies. This tracks
     * the number of policies using the SA. The SA is deleted
     * when this count reaches zero.
     */
    uint32_t                refCount;
}NetfpProxy_SAInfo;

/**
 * @brief
 *  NETFP Proxy Security Policy Info
 *
 * @details
 *  Holds info regarding security policies
 *  offloaded in Netfp Proxy.
 */
typedef struct NetfpProxy_SPInfo
{
    /**
     * @brief   Link to other SP Info objects
     */
    List_Node               list_n;

    /**
     * @brief   Policy id
     */
    uint32_t                policyId;
}NetfpProxy_SPInfo;

/**
 * @brief
 *  NETFP Proxy IPsec based database based on reqid
 *
 * @details
 *  Holds info regarding security associations/policies
 *  refrenced by reqid.
 */
typedef struct NetfpProxy_reqIdInfo
{
    /**
     * @brief   Link to other reqid Info objects
     */
    List_Node               list_n;

    /**
     * @brief   Requiest Id
     */
    uint32_t                reqId;

    /**
     * @brief   Direction
     */
    Netfp_Direction         dir;

    /**
     * @brief   List of Security Associations 
     */
    NetfpProxy_SAInfo*      saList;

    /**
     * @brief   List of Security Policies
     */
    NetfpProxy_SPInfo*      spList;
}NetfpProxy_reqIdInfo;

/**
 * @brief
 *  IPSEC management MCB
 *
 * @details
 *  The structures is the IPSEC Management MCB.
 */
typedef struct NetfpProxy_IPSECMgmtMCB
{
    /**
     * @brief   Time when the SA statistics were retreived from the NETFP Server
     */
    time_t                      statTime;

    /**
     * @brief   List of policies offloaded using Proxy.
     */
    NetfpProxy_reqIdInfo*       reqIdList;
}NetfpProxy_IPSECMgmtMCB;

/**********************************************************************
 ************************ Global Variables ****************************
 **********************************************************************/

/* IPSEC MCB: */
NetfpProxy_IPSECMgmtMCB     gIPSECMgmtMCB;

/**********************************************************************
 ************************ IPSEC Functions *****************************
 **********************************************************************/

/**
 *  @b Description
 *  @n
 *      Given a NetFP Direction value, returns its corresponding
 *      name in string representation.
 *
 *  @param[in]  direction
 *      Direction value
 *
 *  @retval
 *      String associated with the direction
 */
static char* NetfpProxy_getNetfpDirection2Str (Netfp_Direction direction)
{
    switch (direction)
    {
        case Netfp_Direction_INBOUND:
            return "Ingress";
        case Netfp_Direction_OUTBOUND:
            return "Egress";
        default:
            return "Invalid";
    }
}

/**
 *  @b Description
 *  @n
 *      Function to implement task sleep functionality for IPSecMgr
 *
 *  @param[in]  time_in_msec
 *      Time in milliseconds to sleep
 *
 *  @retval
 *      Not Applicable.
 */
static void NetfpProxy_ipsecMgrtaskSleep (uint32_t time_in_msec)
{
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is called periodically and is used to update the SA statistics
 *      in the kernel. This is done by passing the information to the IPSEC Snooper.
 *
 *  @retval
 *      Not applicable
 */
void NetfpProxy_updateSAStats (void)
{
    NetfpProxy_SAInfo*      ptrSAInfo;
    NetfpProxy_reqIdInfo*   ptrReqIdInfo;
    ipsecmgr_lft_cur_t      curLft;
    Netfp_IpSecStats        netfpIpsecStats;
    int32_t                 errCode;
    time_t                  currTime;

    /* Check to see if it is time to poll stats from NETFP Server */
    currTime = time(NULL);
    if ((currTime - gIPSECMgmtMCB.statTime) < gNetfpProxyMcb.statsUpdateInterval )
        return;

    /* Record the time when the statistics are being retreived. */
    gIPSECMgmtMCB.statTime = currTime;

    /* Cycle through all SA Information blocks registered in the PROXY */
    ptrReqIdInfo = (NetfpProxy_reqIdInfo*)List_getHead ((List_Node**)&gIPSECMgmtMCB.reqIdList);
    while (ptrReqIdInfo != NULL)
    {
        ptrSAInfo = (NetfpProxy_SAInfo*)List_getHead ((List_Node**)&ptrReqIdInfo->saList);
        while (ptrSAInfo != NULL)
        {
            /* Get the NETFP SA Channel statistics: */
            if (Netfp_getIPsecStats(gNetfpProxyMcb.netfpClientHandle, ptrSAInfo->saHandle, &netfpIpsecStats, &errCode) < 0)
            {
                /* Error: Unable to get the NETFP SA channel statistics */
                NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to get the NETFP stats for NETFP SA 0x%x [Error code %d]\n",
                                   ptrSAInfo->saHandle, errCode);
            }
            else
            {
                /* Initialize the lifetime statistics which need to be passed to the IPSEC Snooper */
                memset((void *)&curLft, 0, sizeof(ipsecmgr_lft_cur_t) );

                /* Copy over the stats sent by NETFP */
                if (ptrReqIdInfo->dir == Netfp_Direction_INBOUND)
                {
                    curLft.bytes   = netfpIpsecStats.u.in.inIpsecBytes;
                    curLft.packets = netfpIpsecStats.u.in.inIpsecPkts;
                }
                else
                {
                    curLft.bytes   = netfpIpsecStats.u.out.outIpsecBytes;
                    curLft.packets = netfpIpsecStats.u.out.outIpsecPkts;
                }
                curLft.add_time =   0;
                curLft.use_time =   0;

                /* Update SA stats in Linux Kernel through the IPSEC snooper */
                if (ipsecmgr_snoop_sa_expiry ((ipsecmgr_fp_handle_t)ptrSAInfo, 0, &curLft) < 0)
                    NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: IPSEC Snooper SA Update failed for SPI: 0x%x\n", ptrSAInfo->spi);
            }

            /* Get the next SA Information block */
            ptrSAInfo = (NetfpProxy_SAInfo*)List_getNext ((List_Node*)ptrSAInfo);
        }

        /* Get the next ReqId Information block */
        ptrReqIdInfo = (NetfpProxy_reqIdInfo*)List_getNext ((List_Node*)ptrReqIdInfo);
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to display the internal for the IPSEC module and is used for
 *      debugging.
 *
 *  @retval
 *      Not applicable
 */
void NetfpProxy_ipsecDump (void)
{
    NetfpProxy_reqIdInfo*   ptrReqIdInfo;
    NetfpProxy_SPInfo*      ptrSPInfo;
    NetfpProxy_SAInfo*      ptrSAInfo;

    /* Display the Policy Database */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "*******************************************\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "IPSec Database\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "*******************************************\n");

    /* Cycle through the ReqId Information List: */
    ptrReqIdInfo = (NetfpProxy_reqIdInfo*)List_getHead ((List_Node**)&gIPSECMgmtMCB.reqIdList);
    while (ptrReqIdInfo != NULL)
    {
        /* Dump all relevant ReqId information */
        NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "*******************************************\n");
        NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: reqId:%d [%s] \n",
                           ptrReqIdInfo->reqId, NetfpProxy_getNetfpDirection2Str(ptrReqIdInfo->dir));

        /* Dump all relevant SP information */
        NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "*******************************************\n");
        NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "SP list \n");

        ptrSPInfo = (NetfpProxy_SPInfo*)List_getHead ((List_Node**)&ptrReqIdInfo->spList);
        while (ptrSPInfo != NULL)
        {
            /* Dump all relevant SP information */
            NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: policy id (%d)\n",
                               ptrSPInfo->policyId);

            /* Get the next SP Information: */
            ptrSPInfo = (NetfpProxy_SPInfo*)List_getNext ((List_Node*)ptrSPInfo);
        }

        /* Dump all relevant SA information */
        NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "*******************************************\n");
        NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "SA list \n");

        ptrSAInfo = (NetfpProxy_SAInfo*)List_getHead ((List_Node**)&ptrReqIdInfo->saList);
        while (ptrSAInfo != NULL)
        {
            /* Dump all relevant SA information */
            NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "SPI:0x%x   NETFP-SA: %p [Count %d]\n",
                               ptrSAInfo->spi, 
                               ptrSAInfo->saHandle, ptrSAInfo->refCount);

            /* Get the next SA Information: */
            ptrSAInfo = (NetfpProxy_SAInfo*)List_getNext ((List_Node*)ptrSAInfo);
        }
        NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "*******************************************\n");

        /* Get the next ReqId Information: */
        ptrReqIdInfo = (NetfpProxy_reqIdInfo*)List_getNext ((List_Node*)ptrReqIdInfo);
    }

    /* Dump IPSecMgr states */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "*******************************************\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: IPsecMgr states: \n");

    ipsecmgr_snoop_dump_state();

    return;
}

/**
 *  @b Description
 *  @n
 *      Adding a security association in the Proxy SA list based 
 *  on reqid and its direction.
 *
 *  @param[in]  ptrReqIdInfo
 *      pointer of a ReqIdInfo to add the Security Association
 *  @param[in]  spi
 *      SPI of the security association
 *  @param[in]  saHandle
 *      Netfp server saHandle for the secuirty association
 *  @param[in]  errCode
 *      Netfp Proxy error code
 *
 *  @retval
 *      Success -   (>0) Added SA entry
 *  @retval
 *      ERROR   -   NULL
 */
static NetfpProxy_SAInfo* NetfpProxy_addSAInfo (NetfpProxy_reqIdInfo*  ptrReqIdInfo, uint32_t spi, Netfp_SAHandle saHandle, int32_t* errCode)
{
    NetfpProxy_SAInfo* ptrSAInfo;

    /* Allocate memory for SAInfo */
    if ((ptrSAInfo = malloc (sizeof (NetfpProxy_SAInfo))) == NULL)
    {
        *errCode = NETFP_PROXY_RETVAL_E_NO_MEM;
        return NULL;
    }

    memset(ptrSAInfo, 0, sizeof(NetfpProxy_SAInfo));

    /* Update SAInfo */
    ptrSAInfo->spi         = spi;
    ptrSAInfo->saHandle    = saHandle;
    ptrSAInfo->refCount    = 1;

    /* Add this entry to Proxy's SA Mapping List */
    List_addNode ((List_Node**)&ptrReqIdInfo->saList, (List_Node*)ptrSAInfo);

    return ptrSAInfo;
}

/**
 *  @b Description
 *  @n
 *      Delete a security association in the SA list
 *  based on reqid and its direction.
 *
 *  @param[in]  ptrReqIdInfo
 *      pointer of a ReqIdInfo associated with the Security Association
 *  @param[in]  ptrSAInfo
 *      Security association to be deleted.
 *
 *  @retval
 *      Not applicable
 */
static void NetfpProxy_delSAInfo (NetfpProxy_reqIdInfo*  ptrReqIdInfo, NetfpProxy_SAInfo* ptrSAInfo)
{
    /* Remove the entry and free its memory */
    List_removeNode ((List_Node**)&ptrReqIdInfo->saList, (List_Node*)ptrSAInfo);
    free (ptrSAInfo);

    return ;
}

/**
 *  @b Description
 *  @n
 *      Delete a security association list from reqid Info
 *
 *  @param[in]  ptrReqIdInfo
 *      pointer of a ReqIdInfo to delete the Security Association List
 *
 *  @retval
 *      Not applicable
 */
static void NetfpProxy_delSAInfoList (NetfpProxy_reqIdInfo*  ptrReqIdInfo)
{
    NetfpProxy_SAInfo*      ptrSAInfo;

    /* Cycle through all the SAInfo blocks: */
    ptrSAInfo = (NetfpProxy_SAInfo*)List_getHead ((List_Node**)&ptrReqIdInfo->saList);

    while (ptrSAInfo != NULL)
    {
        NetfpProxy_delSAInfo(ptrReqIdInfo, ptrSAInfo);

        /* Get the next SA Information block */
        ptrSAInfo = (NetfpProxy_SAInfo*)List_getNext ((List_Node*)ptrSAInfo);
    }

    return ;
}

/**
 *  @b Description
 *  @n
 *      Given ReqIdInfo and SPI, looks through the SA list 
 *      for a match.
 *
 *  @param[in]  ptrReqIdInfo
 *      pointer of a ReqIdInfo to find a match for SPI
 *  @param[in]  spi
 *      SPI to match
 *
 *  @retval
 *      Success -   (>0) Matching SAInfo entry
 *  @retval
 *      ERROR   -   NULL
 */
static NetfpProxy_SAInfo* NetfpProxy_findSAInfo (NetfpProxy_reqIdInfo* ptrReqIdInfo, uint32_t spi)
{
    NetfpProxy_SAInfo*      ptrSAInfo;

    /* Cycle through all the SA Information blocks: */
    ptrSAInfo = (NetfpProxy_SAInfo*)List_getHead ((List_Node**)&ptrReqIdInfo->saList);

    while (ptrSAInfo != NULL)
    {
        /* Do we have a match? */
        if (ptrSAInfo->spi == spi)
            return ptrSAInfo;

        /* Get the next SA Information block */
        ptrSAInfo = (NetfpProxy_SAInfo*)List_getNext ((List_Node*)ptrSAInfo);
    }

    return NULL;
}

/**
 *  @b Description
 *  @n
 *      Given reqid and dir, find the matching ReqidInfo
 *
 *  @param[in]  reqid
 *      Reqid 
 *  @param[in]  dir
 *      Direction
 *
 *  @retval
 *      Success -   (>0) Matching ReqIdInfo entry
 *  @retval
 *      ERROR   -   NULL
 */
static NetfpProxy_reqIdInfo* NetfpProxy_findReqIdInfo (uint32_t reqid, Netfp_Direction dir)
{
    NetfpProxy_reqIdInfo*  ptrReqIdInfo;

    /* Cycle through all the ReqId Information blocks: */
    ptrReqIdInfo = (NetfpProxy_reqIdInfo*)List_getHead ((List_Node**)&gIPSECMgmtMCB.reqIdList);
    while (ptrReqIdInfo != NULL)
    {
        if ( (reqid == ptrReqIdInfo->reqId) && (dir == ptrReqIdInfo->dir) )
        {
            return ptrReqIdInfo;
        }

        /* Get the next ReqId Information block */
        ptrReqIdInfo = (NetfpProxy_reqIdInfo*)List_getNext ((List_Node*)ptrReqIdInfo);
    }
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      Add ReqId Info to the reqid list.
 *
 *  @param[in]  reqid
 *      Reqid to match
 *  @param[in]  dir
 *      dir to match
 *  @param[in]  errCode
 *      Pointer to save error code
 *
 *  @retval
 *      Success -   (>0) Matching Reqid Info entry
 *  @retval
 *      ERROR   -   NULL
 */
static NetfpProxy_reqIdInfo* NetfpProxy_addReqIdInfo (uint32_t reqId, Netfp_Direction dir, int32_t* errCode)
{
    NetfpProxy_reqIdInfo*  ptrReqIdInfo;

    if ((ptrReqIdInfo = malloc (sizeof (NetfpProxy_reqIdInfo))) == NULL)
    {
        *errCode = NETFP_PROXY_RETVAL_E_NO_MEM;
        return NULL;
    }

    /* Initialize structure */
    memset(ptrReqIdInfo, 0, sizeof (NetfpProxy_reqIdInfo));

    /* Update the reqId information */
    ptrReqIdInfo->reqId           = reqId;
    ptrReqIdInfo->dir             = dir;

    /* Add this entry to Proxy's SA Mapping List */
    List_addNode ((List_Node**)&gIPSECMgmtMCB.reqIdList, (List_Node*)ptrReqIdInfo);

    return ptrReqIdInfo;
}

/**
 *  @b Description
 *  @n
 *      Delete a Reqid Info entry from the list.
 *
 *  @param[in]  ptrReqIdInfo
 *      Pointer to the reqidInfo to be deleted
 *
 *  @retval
 *      Not applicable
 */
static void NetfpProxy_delReqIdInfo (NetfpProxy_reqIdInfo*  ptrReqIdInfo)
{
    /* Remove the entry and free its memory */
    List_removeNode ((List_Node**)&gIPSECMgmtMCB.reqIdList, (List_Node*)ptrReqIdInfo);
    free (ptrReqIdInfo);

    return ;
}

/**
 *  @b Description
 *  @n
 *      Add a security policy in the Proxy SP list of a given ReqIdInfo .
 *
 *  @param[in]  ptrReqIdInfo
 *      Pointer of a ReqIdInfo to add the Security Association
 *  @param[in]  policy_id
 *      Security policy id
 *  @param[in]  errCode
 *      Netfp Proxy error code
 *
 *  @retval
 *      Success -   (>0) Added SPInfo entry
 *  @retval
 *      ERROR   -   NULL
 */
static NetfpProxy_SPInfo* NetfpProxy_addSPInfo (NetfpProxy_reqIdInfo*  ptrReqIdInfo, uint32_t policy_id, int32_t* errCode)
{
    NetfpProxy_SPInfo* ptrSPInfo;

    if ( (ptrSPInfo = (NetfpProxy_SPInfo *) malloc (sizeof (NetfpProxy_SPInfo))) == NULL)
    {
        *errCode = NETFP_PROXY_RETVAL_E_NO_MEM;
        return NULL;
    }

    /* Initialize SPInfo */
    memset(ptrSPInfo, 0, sizeof(NetfpProxy_SPInfo));

    /* Save policy id */
    ptrSPInfo->policyId         = policy_id;

    /* Add this entry to Proxy's SPInfo List */
    List_addNode ((List_Node**)&ptrReqIdInfo->spList, (List_Node*)ptrSPInfo);

    return ptrSPInfo;
}

/**
 *  @b Description
 *  @n
 *      Delete security policy from Proxy database.
 *
 *  @param[in]  ptrReqIdInfo
 *      Pointer of a ReqIdInfo to delete the Security policy from
 *  @param[in]  ptrSPInfo
 *      Security policy to be deleted.
 *
 *  @retval
 *      Not applicable
 */
static void NetfpProxy_delSPInfo (NetfpProxy_reqIdInfo*  ptrReqIdInfo, NetfpProxy_SPInfo* ptrSPInfo)
{
    /* Remove the entry and free its memory */
    List_removeNode ((List_Node**)&ptrReqIdInfo->spList, (List_Node*)ptrSPInfo);
    free (ptrSPInfo);
}

/**
 *  @b Description
 *  @n
 *      Given ReqIdInfo and policy id, find security policy info in Proxy database.
 *
 *  @param[in]  ptrReqIdInfo
 *      Pointer to the ReqIdInfo to find the security policy
 *  @param[in]  policyId
 *      Policy id  to match
 *
 *  @retval
 *      Success -   (>0) Matching SPInfo entry
 *  @retval
 *      ERROR   -   NULL
 */
static NetfpProxy_SPInfo* NetfpProxy_findSPInfo (NetfpProxy_reqIdInfo* ptrReqIdInfo, uint32_t policyId)
{
    NetfpProxy_SPInfo* ptrSPInfo;

    ptrSPInfo = (NetfpProxy_SPInfo*)List_getHead ((List_Node**)&ptrReqIdInfo->spList);
    while (ptrSPInfo != NULL)
    {
        /* Do we have a match? */
        if (ptrSPInfo->policyId == policyId)
            return ptrSPInfo;

        /* Get the next SP Information block */
        ptrSPInfo = (NetfpProxy_SPInfo*)List_getNext ((List_Node*)ptrSPInfo);
    }
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      Given policy id, find security policy info in Proxy database.
 *
 *  @param[in]  policyId
 *      policyId to match
 *
 *  @retval
 *      Success -   (>0) Matching SPInfo entry
 *  @retval
 *      ERROR   -   NULL
 */
static NetfpProxy_SPInfo* NetfpProxy_findPolicyInProxy (uint32_t policyId)
{
    NetfpProxy_reqIdInfo* ptrReqIdInfo;
    NetfpProxy_SPInfo*    ptrSPInfo;

    /* Walk through all the ReqId Information blocks: */
    ptrReqIdInfo = (NetfpProxy_reqIdInfo*)List_getHead ((List_Node**)&gIPSECMgmtMCB.reqIdList);
    while (ptrReqIdInfo != NULL)
    {
        /* Walk through all SPInfo to find a match */
        ptrSPInfo = (NetfpProxy_SPInfo*)List_getHead ((List_Node**)&ptrReqIdInfo->spList);
        while (ptrSPInfo != NULL)
        {
            /* Do we have a match? */
            if (ptrSPInfo->policyId == policyId)
                return ptrSPInfo;
    
            /* Get the next SP Information block */
            ptrSPInfo = (NetfpProxy_SPInfo*)List_getNext ((List_Node*)ptrSPInfo);
        }

        /* Get the next ReqId Information block */
        ptrReqIdInfo = (NetfpProxy_reqIdInfo*)List_getNext ((List_Node*)ptrReqIdInfo);
    }

    return NULL;
}

/**
 *  @b Description
 *  @n
 *      The function is called from the IPSEC Snooper and is used to translate
 *      the IPSEC snooper SA configuration into the NETFP SA configuration. The
 *      function will also add the security association into the NETFP Server.
 *
 *  @param[in]  af
 *      Address family for IP.
 *  @param[in]  sa_id
 *      Pointer to the SA identifiers.
 *  @param[in]  sa_info
 *      Pointer to the IPSec parameters.
 *  @param[in]  dscp_map_cfg
 *      DSCP mapping config valid only in tunnel mode.
 *  @param[in]  if_name
 *      For DIR==OUTBOUND, this is the MAC interface from which src mac
 *      would be derived. For DIR==INBOUND, this is the MAC interface
 *      to which SA could be linked.
 *  @param[in]   encap
 *      UDP encapsulation info for NAT-T
 *  @param[out]  sa_handle
 *      Handle to the security association in NETFP.
 *
 *  \ingroup NETFP_PROXY_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      ERROR   -   <0
 */
static int32_t NetfpProxy_addSA
(
    ipsecmgr_af_t               af,
    ipsecmgr_sa_id_t*           sa_id,
    ipsecmgr_sa_info_t*         sa_info,
    ipsecmgr_sa_dscp_map_cfg_t* dscp_map_cfg,
    ipsecmgr_ifname_t*          if_name,
    ipsecmgr_sa_encap_tmpl_t*   encap,
    ipsecmgr_fp_handle_t*       sa_handle
)
{
    Netfp_SACfg                 saCfg;
    Netfp_SAHandle              saHandle;
    int32_t                     index, errCode;
    NetfpProxy_SAInfo*          ptrSAInfo;
    NetfpProxy_SAInfo*          ptrNewSAInfo;
    NetfpProxy_reqIdInfo*       ptrReqIdInfo;
    int32_t                     intErr;

    /* Initialize the return value */
    *sa_handle = (ipsecmgr_fp_handle_t)NULL;

    /* Sanity Check: Validate the arguments */
    if ((sa_info->dir != DIR_INBOUND) && (sa_info->dir != DIR_OUTBOUND))
    {
        /* Error: Invalid direction */
        NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Error: Invalid direction during adding SA [Direction %d]\n", sa_info->dir);
        return -1;
    }

    /* Sanity Check: Validate the arguments */
    if ((af != IPSECMGR_AF_IPV4) && (af != IPSECMGR_AF_IPV6))
    {
        /* Error: Invalid address family */
        NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Error: Invalid address family during adding SA [AF %d]\n", af);
        return -1;
    }

    /* Sanity Check: IPSec protocol in the only supported protocol */
    if (sa_id->proto != SA_PROTO_ESP)
    {
        /* Error: Invalid protocol */
        NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Error: Invalid protocol [Protocol %d]\n", sa_id->proto);
        return -1;
    }

    /* Sanity Check: Only Tunnel mode is supported */
    if (sa_info->mode != SA_MODE_TUNNEL)
    {
        /* Error: Invalid protocol */
        *sa_handle = (ipsecmgr_fp_handle_t)NULL;
        NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Error: Invalid mode [Mode %d]\n", sa_info->mode);
        return -1;
    }

    /* Validate the key lengths. */
    if (sa_info->auth_key_len > NETFP_MAX_IPSEC_KEY_LEN)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Authentication key size (%d) is invalid.\n", sa_info->auth_key_len);
        return -1;
    }
    if (sa_info->enc_key_len > NETFP_MAX_IPSEC_KEY_LEN)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Encryption key size (%d) is invalid.\n", sa_info->enc_key_len);
        return -1;
    }

    /* Initialize the SA Config structure. */
    memset ((void *)&saCfg, 0, sizeof (Netfp_SACfg));

    /* Get the SA direction. */
    if (sa_info->dir == DIR_INBOUND)
        saCfg.direction = Netfp_Direction_INBOUND;
    else
        saCfg.direction = Netfp_Direction_OUTBOUND;

    /* Get the SPI. */
    saCfg.spi = sa_id->spi;

    /* Has the SA already been offloaded to NETFP? 
     * If the SA (with matching reqid, dir, and spi) is already offloaded, 
     * this is a case for mulitple traffic selector to share the same SA. 
     * Re-use the SA by incrementing the refCount.
     */
    ptrReqIdInfo = NetfpProxy_findReqIdInfo(sa_info->reqid, saCfg.direction);
    if (ptrReqIdInfo != NULL)
    {
        ptrSAInfo = NetfpProxy_findSAInfo (ptrReqIdInfo, sa_id->spi);
        if (ptrSAInfo != NULL)
        {
            /* YES: SPI has already been added to the NETFP Server */
            ptrSAInfo->refCount++;

            NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Reusing SA(%p) SPI:0x%x already configured in NetFP. Refcnt=%d\n", 
                                                       ptrSAInfo, ptrSAInfo->spi, ptrSAInfo->refCount);

            *sa_handle = (ipsecmgr_fp_handle_t)(ptrSAInfo);
            return 0;
        }
    }

    /* Get the IP protocol version. */
    if (af == IPSECMGR_AF_IPV4)
    {
        /* IPv4 Family: */
        saCfg.dstIP.ver = Netfp_IPVersion_IPV4;
        saCfg.srcIP.ver = Netfp_IPVersion_IPV4;

        /* Populate the source and destination IP addresses. */
        for (index = 0; index < 4; index++)
        {
            saCfg.dstIP.addr.ipv4.u.a8[index] = sa_id->daddr.ipv4[index];
            saCfg.srcIP.addr.ipv4.u.a8[index] = sa_info->saddr.ipv4[index];
        }
    }
    else
    {
        /* IPv6 Family: */
        saCfg.dstIP.ver = Netfp_IPVersion_IPV6;
        saCfg.srcIP.ver = Netfp_IPVersion_IPV6;

        /* Populate the source and destination IP addresses. */
        for (index = 0; index < 16; index++)
        {
            saCfg.dstIP.addr.ipv6.u.a8[index] = sa_id->daddr.ipv6[index];
            saCfg.srcIP.addr.ipv6.u.a8[index] = sa_info->saddr.ipv6[index];
        }
    }

    /* Only IPSEC protocols are supported */
    saCfg.ipsecCfg.protocol = Netfp_IPSecProto_IPSEC_ESP;
    saCfg.ipsecCfg.mode = Netfp_IPSecMode_IPSEC_TUNNEL;
    /* Get the authentication mode algorithm. */
    if (sa_info->auth.algo == SA_AALG_NONE || sa_info->auth.algo == SA_AALG_NULL)
        saCfg.ipsecCfg.authMode = Netfp_IpsecAuthMode_NULL;
    else if (sa_info->auth.algo == SA_AALG_HMAC_SHA1)
        saCfg.ipsecCfg.authMode = Netfp_IpsecAuthMode_HMAC_SHA1;
    else if (sa_info->auth.algo == SA_AALG_HMAC_MD5)
        saCfg.ipsecCfg.authMode = Netfp_IpsecAuthMode_HMAC_MD5;
    else if (sa_info->auth.algo == SA_AALG_AES_XCBC)
        saCfg.ipsecCfg.authMode = Netfp_IpsecAuthMode_AES_XCBC;
    else if (sa_info->auth.algo == SA_AALG_HMAC_SHA2_256)
        saCfg.ipsecCfg.authMode = Netfp_IpsecAuthMode_HMAC_SHA2_256;
    else if (sa_info->auth.algo == SA_AALG_AES_GMAC)
        saCfg.ipsecCfg.authMode = Netfp_IpsecAuthMode_AES_GMAC;
    else
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unsupported authentication algorithm (%d)\n", sa_info->auth.algo);
        return -1;
    }

    /* Get the encryption mode algorithm. */
    if (sa_info->enc.algo == SA_EALG_NULL)
        saCfg.ipsecCfg.encMode = Netfp_IpsecCipherMode_NULL;
    else if (sa_info->enc.algo == SA_EALG_AES_CTR)
        saCfg.ipsecCfg.encMode = Netfp_IpsecCipherMode_AES_CTR;
    else if (sa_info->enc.algo == SA_EALG_AES_CBC)
        saCfg.ipsecCfg.encMode = Netfp_IpsecCipherMode_AES_CBC;
    else if (sa_info->enc.algo == SA_EALG_3DES_CBC)
        saCfg.ipsecCfg.encMode = Netfp_IpsecCipherMode_3DES_CBC;
    else if (sa_info->enc.algo == SA_EALG_DES_CBC)
        saCfg.ipsecCfg.encMode = Netfp_IpsecCipherMode_DES_CBC;
    else if (sa_info->enc.algo == SA_EALG_AES_GCM)
        saCfg.ipsecCfg.encMode = Netfp_IpsecCipherMode_AES_GCM;
    else
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "ERROR: Unsupported encryption algorithm (%d)\n", sa_info->enc.algo);
        return -1;
    }

    /* Populate ESN parameters. */
    saCfg.ipsecCfg.esnLo = sa_info->esnlo;
    if (sa_info->flags & IPSECMGR_SA_FLAGS_ESN)
    {
        saCfg.ipsecCfg.esnEnabled = 1;
        saCfg.ipsecCfg.esnHi = sa_info->esnhi;
    }
    else
    {
        saCfg.ipsecCfg.esnEnabled = 0;
        saCfg.ipsecCfg.esnHi = 0;
    }

    /* Store the key sizes */
    saCfg.ipsecCfg.keyAuthSize = sa_info->auth_key_len;
    saCfg.ipsecCfg.keyEncSize  = sa_info->enc_key_len;
    saCfg.ipsecCfg.keyMacSize  = sa_info->auth.icvlen;

    /* Copy over the authentication/encryption keys. */
    memcpy ((void *)saCfg.ipsecCfg.keyAuth, (void *)sa_info->auth_key, sizeof (uint8_t) * sa_info->auth_key_len);
    memcpy ((void *)saCfg.ipsecCfg.keyEnc, (void *)sa_info->enc_key, sizeof (uint8_t) * sa_info->enc_key_len);

    /* Get the lifetime configuration */
    saCfg.ipsecCfg.lifetime.softByteLimit   = sa_info->lft.soft_byte_limit;
    saCfg.ipsecCfg.lifetime.hardByteLimit   = sa_info->lft.hard_byte_limit;
    saCfg.ipsecCfg.lifetime.softPacketLimit = sa_info->lft.soft_packet_limit;
    saCfg.ipsecCfg.lifetime.hardPacketLimit = sa_info->lft.hard_packet_limit;
    saCfg.ipsecCfg.lifetime.softAddExpires  = sa_info->lft.soft_add_expires;
    saCfg.ipsecCfg.lifetime.hardAddExpires  = sa_info->lft.hard_add_expires;
    saCfg.ipsecCfg.lifetime.softUseExpires  = sa_info->lft.soft_use_expires;
    saCfg.ipsecCfg.lifetime.hardUseExpires  = sa_info->lft.hard_use_expires;

    /* Set the replay window size. It is currently hardcoded to 128. This needs to be extended
     * to read per childSA. */
    saCfg.replayWindowSize = 128;

    /* Is NAT traversal enabled? Else, port numbers should be set to 0. */
    if(encap != NULL)
    {
        /* Add NAT-T UDP encapsulation info */
        saCfg.nattEncapCfg.srcPort = encap->sport;
        saCfg.nattEncapCfg.dstPort = encap->dport;
    }

#if 0 //Devkit needs update
    /* Setup the fragmentation level for this SA. */
    if (sa_info->flags & IPSECMGR_SA_FLAGS_OUTER_FRAG)
    {
        /* Perform outer IP fragmentation on this tunnel */
        saCfg.fragLevel  = Netfp_IPSecFragLevel_OUTER_IP;
    }
    else
    {
        /* Perform inner IP fragmentation on this tunnel */
        saCfg.fragLevel  = Netfp_IPSecFragLevel_INNER_IP;
    }
#else
    /* Enable Inner IP fragmentation by default */
    saCfg.fragLevel  = Netfp_IPSecFragLevel_INNER_IP;
#endif

    /* Find the ReqId Mapping information. The following two scenarioes are handled here:
     * 1. Add new SA when reqId info does not exist
       2. Rekey SA when reqid info already exist
     */
    if (ptrReqIdInfo != NULL)
    {
        /* This a SA rekey request. Call the NETFP to add the rekeyed security association. */
        NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Rekeying security association\n");

        /* The first SAInfo on the list is used as the old SA */
        ptrSAInfo = (NetfpProxy_SAInfo*)List_getHead ((List_Node**)&ptrReqIdInfo->saList);
        if (ptrSAInfo == NULL)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Rekey SA with old SA Info = NULL\n");
            return -1;
        }

        /* Call Netfp server to Rekey SA */
        saHandle = Netfp_rekeySA (gNetfpProxyMcb.netfpClientHandle, ptrSAInfo->saHandle, &saCfg, &errCode);
        if (saHandle == NULL)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Rekey SA failed [Error code %d]\n", errCode);
            return -1;
        }
    }
    else
    {
        /* This is a new SA entry. Call Netfp_addSA to setup the SA in NetFP */
        NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Add new security association\n");
        saHandle = Netfp_addSA (gNetfpProxyMcb.netfpClientHandle, &saCfg, &errCode);
        if (saHandle == NULL)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Adding SA on Netfp Server failed [Error code %d]\n", errCode);
            return -1;
        }

        ptrReqIdInfo = NetfpProxy_addReqIdInfo(sa_info->reqid, saCfg.direction, &errCode);
        if (ptrReqIdInfo == NULL)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Adding ReqId Info failed [Error code %d]\n", errCode);

            /* Delete SA from Netfp Server */
            Netfp_delSA (gNetfpProxyMcb.netfpClientHandle, saHandle, &intErr);
            return -1;
        }
    }

    /* Create a new SAInfo for both RekeySA and newSA */
    ptrNewSAInfo = NetfpProxy_addSAInfo (ptrReqIdInfo, saCfg.spi, saHandle, &errCode);
    if (ptrNewSAInfo == NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Failed to create SAInfo for Req Id: %d [Error %d], memory is not available.\n", 
                          ptrReqIdInfo->reqId, errCode);

        if (Netfp_delSA (gNetfpProxyMcb.netfpClientHandle, saHandle, &errCode) < 0)
        {
            /* FATAL Error: Log the error */
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: NETFP Delete SA failed [Error code: %d]\n", errCode);
        }
        return -1;
    }

    /* Return the Proxy's SA info handle. */
    *sa_handle = (ipsecmgr_fp_handle_t)(ptrNewSAInfo);

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Succesfully added %s SA with SPI 0x%x in NETFP\n",
                       NetfpProxy_getNetfpDirection2Str(ptrReqIdInfo->dir), ptrNewSAInfo->spi);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to translate the security association
 *      parameters and call the function to delete a security
 *      association.
 *
 *  @param[in]  reqid
 *      Reqid associated with the SA.
 *  @param[in]  dir
 *      SA direction.
 *  @param[in]  saHandle
 *      Handle to the security association to be deleted.
 *
 *  \ingroup NETFP_PROXY_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t NetfpProxy_deleteSA (uint32_t reqid, ipsecmgr_dir_t dir, ipsecmgr_fp_handle_t sa_handle)
{
    NetfpProxy_SAInfo*        ptrSAInfo;
    NetfpProxy_reqIdInfo*     ptrReqIdInfo;
    int32_t                   errCode;
    int32_t                   retVal = 0;

    /* Proxy is in reset. No point processing these callbacks */
    if (gNetfpProxyMcb.bIsInReset)
        return 0;

    /* Validate input */
    if (sa_handle == (ipsecmgr_fp_handle_t)NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "ERROR: Delete SA failed with wrong SA handle\n",
                sa_handle);
        return -1;
    }

    /* Get the SA Info handle */
    ptrSAInfo = (NetfpProxy_SAInfo*)(sa_handle);

    /* Find ReqIdInfo in Proxy database */
    ptrReqIdInfo = NetfpProxy_findReqIdInfo(reqid, dir);
    if(ptrReqIdInfo == NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "ERROR: Delete SA(%p) called for reqId %d dir %s failed\n",
                sa_handle, ptrReqIdInfo->reqId, NetfpProxy_getNetfpDirection2Str(ptrReqIdInfo->dir));
        return -1;
    }
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "DEBUG: Delete SA(%p) called for reqId %d dir %s\n",
            sa_handle, ptrReqIdInfo->reqId, NetfpProxy_getNetfpDirection2Str(ptrReqIdInfo->dir));

    /* Sanity check SAInfo */
    if (NetfpProxy_findSAInfo(ptrReqIdInfo, ptrSAInfo->spi) != ptrSAInfo) 
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "DEBUG: Delete SA(%p) can not find SA for SPI 0x%x \n",
                sa_handle, ptrSAInfo->spi);
        return -1;
    }

    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "DEBUG: Found SA(%p) SPI 0x%x with refCount=%d\n",
            sa_handle, ptrSAInfo->spi, ptrSAInfo->refCount);

    /* Decrement the refCount */
    ptrSAInfo->refCount--;

    /* Check refcount, wait until refcount ==0 to remove the entry */
    if(ptrSAInfo->refCount > 0)
        return 0;

    /* RefCount reaches 0, entry needs to be deleted from Proxy database and Netfp Server*/
    while (1)
    {
        /* Delete SA from NETFP server: */
        retVal = Netfp_delSA (gNetfpProxyMcb.netfpClientHandle, ptrSAInfo->saHandle, &errCode);
        if (retVal < 0)
        {
            /* Error: Delete SA failed; determine the error code to determine if we need to try again */
            if (errCode == NETFP_EINUSE)
                continue;

            /* FATAL Error: Log the error */
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: NETFP Delete SA failed [Error code: %d]\n", errCode);

            /* Remove SAInfo from SA List*/
            NetfpProxy_delSAInfo(ptrReqIdInfo, ptrSAInfo);

            return -1;
        }
        else
        {
            /* Delete SA was successful: */
            break;
        }
    }

    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "DEBUG: delete SA(%p) SPI 0x%x with refCount=%d is successful, SAInfo entry will be deteted\n",
            ptrSAInfo, ptrSAInfo->spi, ptrSAInfo->refCount);

    /* Remove SAInfo from SA List*/
    NetfpProxy_delSAInfo(ptrReqIdInfo, ptrSAInfo);

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is called from the IPSEC Snooper and is used to translate
 *      the IPSEC snooper policy configuration into a NETFP policy. The function
 *      will also add the security policy into the NETFP Server.
 *
 *  @param[in]  af
 *      Address family for IP.
 *  @param[in]  sel
 *      IPSec SA selector used in IPSec Security policy.
 *  @param[in]  dir
 *      Direction in which the security policy is created.
 *  @param[in]  reqid
 *      Corresponding request ID for the SP.
 *  @param[in]  handleSAInfo
 *      Handle to the NETFP Proxy SA Information
 *  @param[in]  policy_id
 *      Security Policy ID for which policy needs to be added
 *  @param[out]  sp_handle
 *      Handle to the security policy added in NETFP.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t NetfpProxy_addSP
(
    ipsecmgr_af_t           af,
    ipsecmgr_selector_t*    sel,
    ipsecmgr_dir_t          dir,
    uint32_t                reqid,
    ipsecmgr_fp_handle_t    handleSAInfo,
    ipsecmgr_policy_id_t    policy_id,
    ipsecmgr_fp_handle_t*   sp_handle
)
{
    Netfp_SPCfg             spCfg;
    uint32_t                index;
    int32_t                 errCode;
    NetfpProxy_SPInfo*      ptrSPInfo;
    NetfpProxy_reqIdInfo*   ptrReqIdInfo;
    NetfpProxy_SAInfo*      ptrSAInfo;

    /* Sanity Check: Validate the arguments */
    if (sel == NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: NULL Selector passed\n", sel);
        return -1;
    }

    /* Sanity Check: Validate the arguments */
    if ((af != IPSECMGR_AF_IPV4) && (af != IPSECMGR_AF_IPV6))
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Invalid address family (%d)\n", af);
        return -1;
    }

    /* Sanity Check: Validate the arguments */
    if ((dir != DIR_OUTBOUND) && (dir != DIR_INBOUND))
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Invalid direction (%d)\n", dir);
        return -1;
    }

    /* Sanity Check: Validate the arguments */
    if ((sel->l5_selector != NULL) && (sel->l5_selector->proto != IPSECMGR_L5_PROTO_GTPU))
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: L5 protocol (%d) is invalid.\n", sel->l5_selector->proto);
        return -1;
    }

    /* Sanity Check: Validate the sp_handle */
    if (sp_handle == NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: NULL sp_handle passed\n", sel);
        return -1;
    }

    /* Initialize sp_handle with NULL pointer */
    *sp_handle = (ipsecmgr_fp_handle_t)NULL;

    /* Initialize the SP Config structure. */
    memset ((void *)&spCfg, 0, sizeof (Netfp_SPCfg));

    /* Populate the policy configuration */
    spCfg.spId = policy_id;

    /* Get the IP protocol version. */
    if (af == IPSECMGR_AF_IPV4)
    {
        /* IPv4 Family: */
        spCfg.dstIP.ver = Netfp_IPVersion_IPV4;
        spCfg.srcIP.ver = Netfp_IPVersion_IPV4;

        /* Populate the source and destination IP addresses. */
        for (index = 0; index < 4; index++)
        {
            spCfg.dstIP.addr.ipv4.u.a8[index] = sel->daddr.ipv4[index];
            spCfg.srcIP.addr.ipv4.u.a8[index] = sel->saddr.ipv4[index];
        }
    }
    else
    {
        /* IPv6 Family: */
        spCfg.dstIP.ver = Netfp_IPVersion_IPV6;
        spCfg.srcIP.ver = Netfp_IPVersion_IPV6;

        /* Populate the source and destination IP addresses. */
        for (index = 0; index < 16; index++)
        {
            spCfg.dstIP.addr.ipv6.u.a8[index] = sel->daddr.ipv6[index];
            spCfg.srcIP.addr.ipv6.u.a8[index] = sel->saddr.ipv6[index];
        }
    }

    /* Get the security policy direction. */
    if (dir == DIR_OUTBOUND)
        spCfg.direction = Netfp_Direction_OUTBOUND;
    else
        spCfg.direction = Netfp_Direction_INBOUND;

    /* Populate remaining SP parameters. */
    spCfg.srcIPPrefixLen = sel->prefixlen_s;
    spCfg.dstIPPrefixLen = sel->prefixlen_d;
    spCfg.protocol       = sel->proto;
    spCfg.dstPortStart   = sel->dport_start;
    spCfg.dstPortEnd     = sel->dport_end;
    spCfg.srcPortStart   = sel->sport_start;
    spCfg.srcPortEnd     = sel->sport_end;

    /* If layer 5 properties are available then populate them. */
    if (sel->l5_selector)
    {
        spCfg.gtpuIdStart = sel->l5_selector->value.gtpu.teid_start;
        spCfg.gtpuIdEnd   = sel->l5_selector->value.gtpu.teid_end;
    }
    else
    {
        /* Non-GTPU. Set the tunnel Ids to 0. */
        spCfg.gtpuIdStart = 0;
        spCfg.gtpuIdEnd   = 0;
    }

    /* Find/create reqidInfo in proxy database. 
       For secure policy, the reqIdInfo should exist - created through addSA()
       For non-secure policy(passthrough policy) , reqidInfo entry needs to be created in Proxy database
     */
    ptrReqIdInfo = NetfpProxy_findReqIdInfo(reqid, spCfg.direction);
    if (ptrReqIdInfo == NULL)
    {
        ptrReqIdInfo = NetfpProxy_addReqIdInfo(reqid, spCfg.direction , &errCode);
        if(ptrReqIdInfo == NULL)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Adding Non-Secure policy failed with adding reqIdInfo [Policy Id: 0x%x]\n", policy_id);
            return -1;
        }
    }

    /* Find SAInfo from reqIdinfo */
    ptrSAInfo = (NetfpProxy_SAInfo*)List_getHead ((List_Node**)&ptrReqIdInfo->saList);
    if (ptrSAInfo == NULL)
    {
        /* Non-secure policy */
        spCfg.saHandle = NULL;
    }
    else
    {
        /* secure policy */
        if(ptrSAInfo != (Netfp_SAHandle)handleSAInfo)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Adding Secure policy failed with improper sa_handle 0x%x vs. 0x%x\n",
                              handleSAInfo, ptrSAInfo->saHandle );
            return -1;
        }
        spCfg.saHandle = ptrSAInfo->saHandle;
    }

    /* Add the security policy in NETFP. */
    if (Netfp_addSP (gNetfpProxyMcb.netfpClientHandle, &spCfg, &errCode) < 0)
    {
        /* Error: NETFP Server was unable to add the security policy */
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Adding security policy %d in NETFP Server failed [Error code: %d]\n",
                           policy_id, errCode);

        /* TODO: delSA? */
        if (ptrSAInfo != NULL)
        {
            /* Delete SA */
        }

        /* TODO : Remove ReqId entry ?? */
        //NetfpProxy_delReqIdInfo (ptrReqIdInfo);
        return -1;
    }

    /* Add SPInfo in Proxy database */
    if ( (ptrSPInfo = NetfpProxy_addSPInfo(ptrReqIdInfo, policy_id, &errCode)) == NULL)
    {
        /* Error: NETFP Server was unable to add the security policy */
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Adding security policy %d in NETFP Proxy failed [Error code: %d]\n",
                           ptrSPInfo->policyId, errCode);

        /* TODO: delSA? */
        if (ptrSAInfo != NULL)
        {
            /* Delete SA */
        }

        /* Delete ReqIdInfo entry from Proxy database */
        NetfpProxy_delReqIdInfo (ptrReqIdInfo);

        return -1;
    }

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Succesfully setup %s policy %d NETFP SA Handle: 0x%x\n",
                       NetfpProxy_getNetfpDirection2Str (ptrReqIdInfo->dir), ptrSPInfo->policyId, spCfg.saHandle);

    /* Return the Policy info entry handle */
    *sp_handle = (ipsecmgr_fp_handle_t)ptrSPInfo;
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to translate the security policy
 *      parameters and call the function to delete a security
 *      policy.
 *
 *  @param[in]  spHandle
 *      Handle to the security policy to be deleted.
 *  @param[in]  policy_id
 *      Security Policy ID for which policy needs to be deleted
 *  @param[in]  reqid
 *      ReqId associated with the security policy.
 *  @param[in]  dir
 *      Direction in which the security policy was created.
 *
 *  \ingroup NETFP_PROXY_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      ERROR   -   <0
 */
static int32_t NetfpProxy_deleteSP
(
    ipsecmgr_fp_handle_t    sp_handle,
    ipsecmgr_policy_id_t    policy_id,
    uint32_t                reqid,
    ipsecmgr_dir_t          dir
)
{
    NetfpProxy_SPInfo*      ptrSPInfo;
    NetfpProxy_reqIdInfo*   ptrReqIdInfo;
    int32_t                 errCode;
    int32_t                 retVal = 0;

    /* Proxy is in reset. No point processing these callbacks */
    if (gNetfpProxyMcb.bIsInReset)
        return retVal;

    /* Validate input */
    if (sp_handle == (ipsecmgr_fp_handle_t)NULL)
        return -1;

    /* Get SPInfo from sp_handle */
    ptrSPInfo  =   (NetfpProxy_SPInfo*)(sp_handle);

    /* Find reqId Info */
    ptrReqIdInfo = NetfpProxy_findReqIdInfo(reqid, dir);
    if(ptrReqIdInfo == NULL)
        return -1;

    /* Sanity check on SPInfo handle */
    if (NetfpProxy_findSPInfo(ptrReqIdInfo, ptrSPInfo->policyId) != ptrSPInfo)
        return -1;

    retVal = Netfp_delSP (gNetfpProxyMcb.netfpClientHandle, ptrSPInfo->policyId, &errCode);
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "DEBUG: Cleaning up NetFP policy with SPID %d on Netfp Server\n", policy_id);

    /* Clean up policy info from Proxy database */
    NetfpProxy_delSPInfo (ptrReqIdInfo, ptrSPInfo);

    /* Remove ReqIdinfo entry if no SP in the splist */
    if((ptrSPInfo = (NetfpProxy_SPInfo*)List_getHead ((List_Node**)&ptrReqIdInfo->spList)) == NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "DEBUG: Found empty NetFP policy list, remove SAInfo list\n");

        /* Remove all the SAInfo ?*/
        NetfpProxy_delSAInfoList(ptrReqIdInfo);

        /* Remove ReqIdInfo from list */
        NetfpProxy_delReqIdInfo(ptrReqIdInfo);
    }

    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "DEBUG: NetfpProxy_deleteSP with policy_id %d succeeded\n", policy_id);

    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get security context for a
 *      given SA.
 *
 *  @param[in]  sa_handle
 *      Handle to the security association for which the
 *      security context must be retrieved
 *  @param[in]  hw_ctx
 *      Context info to be filled in.
 *
 *  \ingroup NETFP_PROXY_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      ERROR   -   <0
 */
static int32_t NetfpProxy_getSwInfo
(
    ipsecmgr_fp_handle_t    sa_handle,
    ipsecmgr_sa_hw_ctx_t*   hw_ctx
)
{
    int32_t                 retVal;
    Netfp_SwContext         secContext;
    NetfpProxy_SAInfo*      ptrSAInfo;
    int32_t                 errCode;

    /* Validate input */
    if (sa_handle == (ipsecmgr_fp_handle_t)NULL)
        return -1;

    /* Get Proxy SAInfo*/
    ptrSAInfo = (NetfpProxy_SAInfo *)sa_handle;

    /* Get the security context info from the NETFP library */
    retVal = Netfp_getIPSecSwInfo (gNetfpProxyMcb.netfpClientHandle, ptrSAInfo->saHandle, &secContext, &errCode);

    /* Process the result */
    if (retVal < 0)
    {
        /* Error getting the context, return error */
        return errCode;
    }
    else
    {
        /* Copy over the context info returned */
        hw_ctx->swinfo_sz   =   secContext.swInfo.size;
        memcpy ((void *)hw_ctx->swinfo, (void *)secContext.swInfo.swInfo, IPSECMGR_SA_SWINFO_MAX_SZ * sizeof (uint32_t));
        hw_ctx->flow_id     =   secContext.flowId;
    }

    /* return success */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      IPSecMgr Start Offload Response message handler: This function is called by the
 *      IPSecMgr library when it has a response ready corresponding to a start offload
 *      SP request issued by the user application.
 *
 *  @param[in]  rsp
 *      IPSecMgr's Start Offload SP response
 *  @param[in]  addr
 *      Destination address (user application) to send the response to
 *  @param[in]  addr_size
 *      Size of destination address passed
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t NetfpProxy_ipsecMgrStartOffloadRspHandler
(
    ipsecmgr_snoop_offload_sp_rsp_param_t*  rsp,
    void*                                   addr,
    uint32_t                                addr_size
)
{
    NetfpProxy_OffloadSpResp    cmdOffloadSpResp;
    NetfpProxy_SPInfo*          ptrSPInfo;
    NetfpProxy_SAInfo*          ptrSAInfo;

    /* Drop ACK packets. Only pass DONE packets back to application */
    if (rsp->type == RSP_TYPE_ACK)
        return 0;

    /* Get the pointer to the policy from the response */
    ptrSPInfo  = (NetfpProxy_SPInfo*)(rsp->sp_handle);
    if (ptrSPInfo == NULL)
        return -1;

    /* Get the pointer to the SA Information: */
    ptrSAInfo = (NetfpProxy_SAInfo*)rsp->sa_handle;

    /* We need to pass back the IPC command response back. Initialize the response */
    memset ((void *)&cmdOffloadSpResp, 0, sizeof (cmdOffloadSpResp));

    /* Was the IPSEC snooper successful? */
    if (rsp->result == RESULT_SUCCESS)
        cmdOffloadSpResp.retVal = NETFP_PROXY_RETVAL_SUCCESS;
    else
        cmdOffloadSpResp.retVal = NETFP_PROXY_RETVAL_E_OP_FAILED;

    /* Populate the remaining fields in the IPC response: */
    cmdOffloadSpResp.policyId       = ptrSPInfo->policyId;
    cmdOffloadSpResp.netfpErrCode   = rsp->err_code;

    /* Is this a secure or non-secure policy? */
    if (ptrSAInfo != NULL)
        cmdOffloadSpResp.netfpSaHandle  = (uint32_t)ptrSAInfo->saHandle;
    else
        cmdOffloadSpResp.netfpSaHandle  = 0;

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Sending IPC Start Offload response type:%d xid: %d retVal:%d\n",
                       rsp->type, rsp->trans_id, rsp->result);

    /* Send the response to the application */
    gNetfpProxyMcb.pluginCfg.report_cmd_response (NETFP_PROXY_CMDTYPE_START_OFFLOAD_SP_REQ, rsp->trans_id,
                                                  &cmdOffloadSpResp, addr, addr_size);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      IPSecMgr Stop Offload Response message handler: This function is called by the
 *      IPSecMgr library when it has a response ready corresponding to a stop offload
 *      SP requestissued by the user application.
 *
 *  @param[in]  rsp
 *      IPSEC Snooper Stop Offload SP response
 *  @param[in]  addr
 *      Destination address (user application) to send the response to
 *  @param[in]  addr_size
 *      Size of destination address passed
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      ERROR   -   <0
 */
static int32_t NetfpProxy_ipsecMgrStopOffloadRspHandler
(
    ipsecmgr_snoop_stop_offload_rsp_param_t*    rsp,
    void*                                       addr,
    uint32_t                                    addr_size
)
{
    NetfpProxy_OffloadSpResp    cmdOffloadSpResp;

    /* Proxy is in reset. No point processing these callbacks */
    if (gNetfpProxyMcb.bIsInReset)
        return 0;

    /* Drop ACK packets. Only pass DONE packets back to application */
    if (rsp->type == RSP_TYPE_ACK)
        return 0;

    /* We need to pass back the IPC command response back. Initialize the response */
    memset ((void *)&cmdOffloadSpResp, 0, sizeof (cmdOffloadSpResp));

    /* Was the IPSEC snooper successful? */
    if (rsp->result == RESULT_SUCCESS)
        cmdOffloadSpResp.retVal = NETFP_PROXY_RETVAL_SUCCESS;
    else
        cmdOffloadSpResp.retVal = NETFP_PROXY_RETVAL_E_OP_FAILED;

    /* Track the policy id which has been stopped */
    cmdOffloadSpResp.policyId = rsp->trans_id;

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Sending IPC Stop Offload response type:%d xid: %d retVal:%d\n",
                       rsp->type, rsp->trans_id, rsp->result);

    /* Send the response to the application */
    gNetfpProxyMcb.pluginCfg.report_cmd_response (NETFP_PROXY_CMDTYPE_STOP_OFFLOAD_SP_REQ, rsp->trans_id, &cmdOffloadSpResp,
                                                  addr, addr_size);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the registered command handler for the IPC message to start the
 *      offload of a security policy
 *
 *  @param[in]  cmdId
 *      Unique Id to identify this command request/response pair in the system.
 *  @param[in]  cmdCfg
 *      Command parameters
 *  @param[in]  appData
 *      Application specific data corresponding to this request. Will be passed back as-is in
 *      response from NetFP Proxy. Optional parameter.
 *  @param[in]  appDataLen
 *      Length of application specific data length passed to this API. Can be zero.
 *
 *  @retval
 *      Success -  0
 *  @retval
 *      Error   - <0
 */
int32_t NetfpProxy_ipsecStartOffload
(
    NetfpProxy_CmdId    cmdId,
    void*               cmdCfg,
    void*               appData,
    uint32_t            appDataLen
)
{
    ipsecmgr_snoop_offload_sp_req_param_t   offload_sp_req;
    NetfpProxy_OffloadSpReq*                ptrCmdOffloadSpReq;

    /* Get the pointer to the Offload Security Policy request: */
    ptrCmdOffloadSpReq = (NetfpProxy_OffloadSpReq*)cmdCfg;

    /* Initialize the IPSEC Manager offload request: */
    memset ((void *)&offload_sp_req, 0, sizeof (offload_sp_req));

    /* Populate the IPSEC Manager offload request: */
    offload_sp_req.trans_id  = cmdId;
    offload_sp_req.policy_id = ptrCmdOffloadSpReq->policyId;
    offload_sp_req.sa_flags  = 0;
    offload_sp_req.if_name   = NULL;

    /* Was a DSCP configuration specified in the request? */
    if ((ptrCmdOffloadSpReq->dscpCfg.policy == 0) || (ptrCmdOffloadSpReq->dscpCfg.policy > NETFP_PROXY_DSCP_USE_MAP_TABLE))
    {
        /* NO: Assume the default Policy; copy DSCP from inner IP to Outer IP */
        offload_sp_req.dscp_cfg = NULL;
    }
    else
    {
        /* YES: Allocate memory for the DSCP configuration */
        offload_sp_req.dscp_cfg = malloc (sizeof (ipsecmgr_sa_dscp_map_cfg_t));
        if (offload_sp_req.dscp_cfg == NULL)
            return NETFP_PROXY_RETVAL_E_NO_MEM;

        /* Copy over the DSCP configuration: */
        offload_sp_req.dscp_cfg->type = ptrCmdOffloadSpReq->dscpCfg.policy;

        /* Is this a fixed value DSCP mapping? */
        if (offload_sp_req.dscp_cfg->type == SA_DSCP_MAP_USE_FIXED_VAL)
        {
            /* YES: Set the fixed value correctly */
            offload_sp_req.dscp_cfg->value.fixed_val = ptrCmdOffloadSpReq->dscpCfg.value.fixedVal;
        }
        else
        {
            /* NO: Copy over the DSCP mapping table */
            memcpy ((void *)offload_sp_req.dscp_cfg->value.map_table, (void *)ptrCmdOffloadSpReq->dscpCfg.value.mapTable, sizeof (uint8_t)*64);
        }
    }

    /* Was a GTPU Identifier range specified? */
    if (ptrCmdOffloadSpReq->gtpuIdRange.min > 0)
    {
        /* YES: Valid GTPU range has been specified; allocate memory for it. */
        offload_sp_req.l5_selector = malloc (sizeof (ipsecmgr_l5_selector_t));
        if (offload_sp_req.l5_selector == NULL)
            return NETFP_PROXY_RETVAL_E_NO_MEM;

        /* Populate the GTPU Range: */
        offload_sp_req.l5_selector->proto                   = IPSECMGR_L5_PROTO_GTPU;
        offload_sp_req.l5_selector->value.gtpu.teid_start   = ptrCmdOffloadSpReq->gtpuIdRange.min;
        offload_sp_req.l5_selector->value.gtpu.teid_end     = ptrCmdOffloadSpReq->gtpuIdRange.max;
    }

    /* Is this a shared policy? */
    if (ptrCmdOffloadSpReq->bIsSharedPolicy == 1)
    {
        /* Policy/SA shared between Linux & NETFP. Indicate the same to IPSecMgr */
        offload_sp_req.sa_flags |= IPSECMGR_SA_FLAGS_SHARED;
    }

    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Sending req xid: %d pid:%d\n", offload_sp_req.trans_id, offload_sp_req.policy_id);

    /* Check if the policy has already been offloaded */
    if (NetfpProxy_findPolicyInProxy(offload_sp_req.policy_id) != NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Offload policy request for policy %d is duplicated \n", ptrCmdOffloadSpReq->policyId);
        return NETFP_PROXY_RETVAL_E_DUPLICATE;
    }

    /* Send the request to the IPSEC snooper */
    if (ipsecmgr_snoop_offload_sp_req (&offload_sp_req, appData, appDataLen))
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Offload policy request for 0x%x failed\n", ptrCmdOffloadSpReq->policyId);
        return NETFP_PROXY_RETVAL_E_OP_FAILED;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the registered command handler for the IPC message to stop the
 *      offload of a security policy
 *
 *  @param[in]  cmdId
 *      Unique Id to identify this command request/response pair in the system.
 *  @param[in]  cmdCfg
 *      Command parameters
 *  @param[in]  appData
 *      Application specific data corresponding to this request. Will be passed back as-is in
 *      response from NetFP Proxy. Optional parameter.
 *  @param[in]  appDataLen
 *      Length of application specific data length passed to this API. Can be zero.
 *
 *  @retval
 *      Success -  0
 *  @retval
 *      Error   - <0
 */
int32_t NetfpProxy_ipsecStopOffload(NetfpProxy_CmdId cmdId, void* cmdCfg, void* appData, uint32_t appDataLen)
{
    ipsecmgr_snoop_stop_offload_req_param_t stop_offload_sp_req;
    NetfpProxy_OffloadSpReq*                ptrCmdOffloadSpReq;

    /* Get the pointer to the offload security policy request: */
    ptrCmdOffloadSpReq = (NetfpProxy_OffloadSpReq*)cmdCfg;

    /* Check if the policy has been offloaded */
    if (NetfpProxy_findPolicyInProxy(ptrCmdOffloadSpReq->policyId) == NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Stop Offload policy request for policy %d failed \n", ptrCmdOffloadSpReq->policyId);
        return NETFP_PROXY_RETVAL_E_OP_FAILED;
    }

    /* Initialize the IPSEC snooper stop offload message */
    memset ((void *)&stop_offload_sp_req, 0, sizeof (stop_offload_sp_req));

    /* Populate the IPSEC Snooper stop offload message and send this to the snooper */
    stop_offload_sp_req.trans_id     = cmdId;
    stop_offload_sp_req.policy_id    = ptrCmdOffloadSpReq->policyId;
    stop_offload_sp_req.no_expire_sa = 1;
    if (ipsecmgr_snoop_stop_offload (&stop_offload_sp_req, appData, appDataLen))
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: IPSEC Snooper stop offload failed\n");
        return NETFP_PROXY_RETVAL_E_OP_FAILED;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to execute the IPSEC module
 *
 *  @retval
 *      Not applicable
 */
void NetfpProxy_ipsecExecute (void)
{
    ipsecmgr_snoop_run();
    NetfpProxy_updateSAStats ();
}

/**
 *  @b Description
 *  @n
 *      The function is used to initialize the IPSEC manager
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   >0
 */
int32_t NetfpProxy_ipsecInit (void)
{
    struct ipsecmgr_snoop_fp_cfg_cb     fp_cfg_cb;
    struct ipsecmgr_snoop_platform_cb   plat_cb;
    struct ipsecmgr_snoop_mgnt_cb       mgnt_cb;
    int32_t                             status;

    /* Initialize the IPSEC management MCB */
    memset ((void*)&gIPSECMgmtMCB, 0, sizeof(NetfpProxy_IPSECMgmtMCB));

    /* Initialize the Fast Path configuration: */
    memset ((void *)&fp_cfg_cb, 0, sizeof(struct ipsecmgr_snoop_fp_cfg_cb));

    /* Populate the configuration: */
    fp_cfg_cb.add_sa     = NetfpProxy_addSA;
    fp_cfg_cb.add_sp     = NetfpProxy_addSP;
    fp_cfg_cb.del_sa     = NetfpProxy_deleteSA;
    fp_cfg_cb.del_sp     = NetfpProxy_deleteSP;
    fp_cfg_cb.get_sa_ctx = NetfpProxy_getSwInfo;

    /* Initialize the Platform configuration: */
    memset ((void *)&plat_cb, 0, sizeof(struct ipsecmgr_snoop_platform_cb));

    /* Populate the configuration: */
    plat_cb.log_msg = (ipsecmgr_snoop_log_msg_t)NetfpProxy_logMsg;
    plat_cb.sleep   = NetfpProxy_ipsecMgrtaskSleep;

    /* Initialize the management configuration: */
    memset ((void *)&mgnt_cb, 0, sizeof(struct ipsecmgr_snoop_mgnt_cb));

    /* Populate the configuration: */
    mgnt_cb.offload_sp_rsp   = NetfpProxy_ipsecMgrStartOffloadRspHandler;
    mgnt_cb.stop_offload_rsp = NetfpProxy_ipsecMgrStopOffloadRspHandler;
    mgnt_cb.rekey_event      = NULL; // No explicit notifications needed on Rekey completion

    /* Initialize the IPSEC manager snooper */
    status = ipsecmgr_snoop_init (&fp_cfg_cb, &mgnt_cb, &plat_cb);
    if (status < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Initializing IPSEC Manager failed (%d)\n", status);
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to deinitialize the IPSEC manager
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   >0
 */
int32_t NetfpProxy_ipsecDeinit (void)
{
    NetfpProxy_reqIdInfo*                   ptrReqIdInfo;
    NetfpProxy_SPInfo*                      ptrSPInfo;
    ipsecmgr_snoop_stop_offload_req_param_t stop_offload_sp_req;
    uint32_t                                xid = 0xFF;

    /* Switch over all offloaded tunnels back to Linux */
    ptrReqIdInfo = (NetfpProxy_reqIdInfo*)List_getHead ((List_Node**)&gIPSECMgmtMCB.reqIdList);
    while (ptrReqIdInfo != NULL)
    {
        ptrSPInfo = (NetfpProxy_SPInfo*)List_getHead ((List_Node**)&ptrReqIdInfo->spList);
        while(ptrSPInfo != NULL)
        {
            /* Populate a IPSecMgr Stop offload SP request message */
            memset ((void *)&stop_offload_sp_req, 0, sizeof (stop_offload_sp_req));
            stop_offload_sp_req.trans_id    =   xid++;
            stop_offload_sp_req.policy_id   =   ptrSPInfo->policyId;

            /* Send stop offload message */
            ipsecmgr_snoop_stop_offload (&stop_offload_sp_req, NULL, 0);

            ptrSPInfo = (NetfpProxy_SPInfo*)List_getNext ((List_Node*)ptrSPInfo);
        }

        ptrReqIdInfo = (NetfpProxy_reqIdInfo*)List_getNext ((List_Node*)ptrReqIdInfo);
    }

    /* Stop IPSecMgr instance */
    ipsecmgr_snoop_shutdown ();
    return 0;
}

