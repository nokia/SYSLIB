/**
 *   @file  netCfg.h
 *
 *   @brief
 *      Networking configurations
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2012 Texas Instruments, Inc.
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

#ifndef __NETCFG_H__
#define __NETCFG_H__

/* Test Configuration: The structure is populated by reading the test
 * configuration file and providing a network setup which is then used
 * by the tests. */
typedef struct Test_NetfpConfigInfo
{
    uint8_t     eNodeBMACAddress[6];
    uint8_t     eNodeBIPAddress[4];
    Netfp_IP6N  eNodeBIPAddress6[4];
    uint8_t     secGwIPAddress[4];
    Netfp_IP6N  secGwIPAddress6;
    uint8_t     secGwMACAddress[6];
    uint8_t     pdnGwMACAddress[6];
    uint8_t     pdnGw2MACAddress[6];
    uint8_t     pdnGwIPAddress[4];
    uint8_t     pdnGw2IPAddress[4];
    Netfp_IP6N  pdnGwIPAddress6;
    Netfp_IP6N  pdnGw2IPAddress6;
}Test_NetfpConfigInfo;

#endif /* __NETCFG_H__ */

