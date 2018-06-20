/**
 *   @file  netfp_ipv6.c
 *
 *   @brief
 *      Utility IPv6 Functions.
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
#include <arpa/inet.h>
#include <errno.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/* SYSLIB Include Files. */
#include <ti/apps/netfp_config/include/NetFP_System_printf.h>
#include <ti/runtime/netfp/netfp.h>
#include <ti/runtime/netfp/include/netfp_internal.h>

/**************************************************************************
 *********************** NetFP IPv6 Functions *****************************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      Utility Function which converts the IPv6 address to string format.
 *
 *  @param[in]   address
 *      The IPv6 Address to be converted
 *  @param[out]  strIPAddress
 *      The IPv6 Address in String Format.
 *
 *  \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Not Applicable.
 */
void Netfp_convertIP6ToStr (Netfp_IP6N address, char* strIPAddress)
{
    if(inet_ntop(AF_INET6, &address, strIPAddress, 40) == NULL)
        System_printf("ERROR: Failed to convert IPv6 address to string: %s", strerror(errno));
}

/**
 *  @b Description
 *  @n
 *      Utility Function that validates whether a given ASCII
 *      character is a valid hexadecimal digit.
 *
 *  @param[in]   ch
 *      The character that needs to be validated
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success     -   1
 *  @retval
 *      Error       -   0
 */
static int32_t Netfp_isValidHexDigit (int ch)
{
	/* Valid Hexadecimal char. return 1. RFC 2396. */
	if ((ch >= '0' && ch <= '9') ||
		(ch >= 'A' && ch <= 'F') ||
		(ch >= 'a' && ch <= 'f'))
		return 1;
	else
		return 0;
}

/**
 *  @b Description
 *  @n
 *      Utility function that converts a given ASCII character
 *      to its hexadecimal value.
 *
 *  @param[in]   ch
 *      The character that needs to be converted to hex.
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   Hex equivalent
 *  @retval
 *      Error   -   <0
 */
static int32_t Netfp_getHexValue (int ch)
{
    if (ch >= '0' && ch <= '9')
    {
        return (ch -'0');
    }
    else if (ch >= 'A' && ch <= 'F')
    {
        return (ch - 'A' + 10);
    }
    else if (ch >= 'a' && ch <= 'f')
    {
        return (ch - 'a' + 10);
    }
    else
    {
        /* Not a valid hexadecimal char, so return error */
        return -1;
    }
}

/**
 *  @b Description
 *  @n
 *      Utility Function which converts an IPv6 Address to string format.
 *
 *  @param[in]   StringIP
 *      The IPv6 Address in String Format
 *  @param[out]  address
 *      The IPv6 Address.
 *
 *  \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      0   -   Success
 *  @retval
 *      -1  -   Error
 */
int32_t Netfp_convertStrToIP6 (const char* StringIP, Netfp_IP6N* address)
{
    int32_t num_colon_sep = 0;
    int32_t num_dcolon_sep = 0;
    int32_t index = 0, exp_index = 0;

    /* Basic Validations: */
    if ((StringIP == NULL) || (address == NULL))
        return -1;

    /* Initialize the IPv6 Address */
    memset((void *)address, 0, sizeof(Netfp_IP6N));

    /* Cycle through and verify if the address had zero compression or not?
     * We run through the Entire string and check the number of ':'
     * and '::' separators. */
    while (StringIP[index] != 0)
    {
        /* Parse through the string, and when we encounter a ':' increment the number
         * of colons by 1 and if we encounter another ':' following a ':', i.e.,
         * a '::', increment both number of colons and number of double colons.
         * These numbers are used for validation purposes once we step out of
         * the loop. */
        if (StringIP[index] == ':')
        {
            num_colon_sep++;

            if (StringIP[index + 1] == ':')
                num_dcolon_sep++;
        }
        else if (!Netfp_isValidHexDigit(StringIP[index]))
        {
            /* ASCII char not a valid hexadecimal int. return
             * error. */
            return -1;
        }

        index = index + 1;
    }

    /* A Valid IPv6 Address cannot have more than 8 16-bit hexadecimal peices
     * separated by ":" separator or more than one "::". Also, if it doesnt have
     * any "::" separated pieces, then it must exactly have 7 ":". Otherwise,
     * its an invalid IPv6 address. */
    if (num_colon_sep > 7 || num_dcolon_sep > 1 || (!num_dcolon_sep && num_colon_sep != 7))
        return -1;

    /* Iterate through the string and convert the characters to their hexadecimal value
     * to insert into IPv6 address. */
    index = 0;
    while (StringIP[index] != 0)
    {
        if (StringIP[index] == ':')
        {
            if (StringIP[index + 1] == ':')
                exp_index += (8 - num_colon_sep);
            else
                exp_index ++;
        }
        else
        {
            address->u.a16[exp_index] = Netfp_htons((Netfp_ntohs(address->u.a16[exp_index]) << 4) + Netfp_getHexValue(StringIP[index]));
        }

		index ++;
    }

    /* Address has been converted. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Utility Function which gets the subnet mask given the number
 *      of bits.
 *      For example: Subnet Mask = 0xFFFF:: if the bits is 16
 *
 *  @param[in]    bits
 *      Number of bits in the subnet mask
 *  @param[out]   subnetMask
 *      Pointer to the computed subnet mask in network order.
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not Applicable.
 */
void Netfp_getIPv6SubnetMask(uint16_t bits, Netfp_IP6N* subnetMask)
{
    uint16_t    index;
    uint32_t    seed;
    uint32_t*   ptrSubnetMask;

    /* Basic Validations: Make sure bits is in the range? */
    if ((subnetMask == NULL) || (bits > 128))
        return;

    /* Initialize the Subnet Mask. */
    subnetMask->u.a32[0] = 0;
    subnetMask->u.a32[1] = 0;
    subnetMask->u.a32[2] = 0;
    subnetMask->u.a32[3] = 0;

    /* Start with the Highest Order address word. */
    ptrSubnetMask = &subnetMask->u.a32[0];

    /* Loop off */
    while (1)
    {
        /* Initialize the seed. */
        seed = 0x80000000;

        /* Determine the number of bits we need to set in the first 4 bytes. */
        if (bits < 32)
            index = bits;
        else
            index = 32;

        /* Set out all the bits */
        while (index-- != 0)
        {
            *ptrSubnetMask = *ptrSubnetMask | seed;
            seed  = seed >> 1;
        }
        *ptrSubnetMask = Netfp_htonl(*ptrSubnetMask);

        /* We have taken care of the 32 bits; do we need to proceed? */
        if (bits <= 32)
            return;

        /* YES. Take out the bits that we took care off and goto the next address. */
        bits = bits - 32;
        ptrSubnetMask++;
    }
}

