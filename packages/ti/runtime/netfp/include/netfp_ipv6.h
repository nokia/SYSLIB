/**
 *   @file  netfp_ipv6.h
 *
 *   @brief
 *      Internal header file used by the NETFP module. Please do not
 *      directly include this file. The file defines the IPv6 interface
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
 
#ifndef __NETFP_IPV6_H__
#define __NETFP_IPV6_H__

#include <ti/runtime/netfp/netfp.h>

/**
 *  @b Description
 *  @n  
 *      Utility Function used to compare the 2 IPv6 Addresses.
 *
 *  @param[in]   ptrAddress1
 *      The first IPv6 Address which is to be compared.
 *  @param[in]   ptrAddress2
 *      The second IPv6 Address which is to be compared.
 *      
 *  @retval
 *      Match      -   1
 *  @retval
 *      No Match   -   0
 */
static inline uint8_t Netfp_matchIPv6Address (Netfp_IP6N* ptrAddress1, Netfp_IP6N* ptrAddress2)
{
    /* Compare the IPv6 Addresses. */
    if ((ptrAddress1->u.a32[0] == ptrAddress2->u.a32[0]) &&
        (ptrAddress1->u.a32[1] == ptrAddress2->u.a32[1]) &&
        (ptrAddress1->u.a32[2] == ptrAddress2->u.a32[2]) &&
        (ptrAddress1->u.a32[3] == ptrAddress2->u.a32[3]))
    {
        /* Perfect Match. */
        return 1;
    }
    else
    {
        /* No Match. */
        return 0;
    }
}

/** 
 *  @b Description
 *  @n  
 *      Utility Function which determines if the IPv6 Address specified is an 
 *      unspecified address or not?
 *
 *  @param[in]   ptrIPv6Address
 *      Pointer to the IPv6 address 
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION 
 *      
 *  @retval
 *      Unspecified         -   1
 *  @retval
 *      Not Unspecified     -   0
 */
static inline uint8_t Netfp_isIPv6Unspecified (Netfp_IP6N* ptrIPv6Address)
{
    Netfp_IP6N  ipv6_unspecified;

    /* Initialize the unspecified address */
    memset ((void *)&ipv6_unspecified, 0, sizeof(Netfp_IP6N));

    /* Return the status. */
    return Netfp_matchIPv6Address (ptrIPv6Address, &ipv6_unspecified);
}

/** 
 *  @b Description
 *  @n  
 *      Utility Function which determines if the IPv6 Address specified is a 
 *      link local address or not?
 *
 *  @param[in]   address
 *      IPv6 Address which needs to be verfied.
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION 
 *      
 *  @retval
 *      Link Local         -   1
 *  @retval
 *      Not Link Local     -   0
 */
static inline uint8_t Netfp_isIPv6LinkLocal (Netfp_IP6N address)
{
    if (address.u.a16[0] == Netfp_htons(0xFE80))
        return 1;
    return 0; 

}

/***********************************************************************
 ************************* Exported Functions **************************
 ***********************************************************************/

/* IPv6 API: */
extern void Netfp_getIPv6SubnetMask (uint16_t bits, Netfp_IP6N* subnetMask);

#endif /* __NETFP_IPV6_H__ */
