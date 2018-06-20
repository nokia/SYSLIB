/**
 *   @file  resmgr_cfg.c
 *
 *   @brief
 *      Resource Manager Configuration for entire system.
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
 *
 */
 
 /* Resource Manager Include Files. */
#include <ti/runtime/resmgr/resmgr.h>

/* All configuration requests are placed in the configuration memory section. */
#pragma DATA_SECTION(gResmgrCfg, ".cfgMemorySection");

/** 
 * @brief 
 *  Sample System Configuration requested.
 *
 * @details
 *  This is a sample platform configuration request for all 4 cores.
 */
Resmgr_ResourceCfg    gResmgrCfg[4] =
{
    /* CORE 0: Requested Configuration. */
	{
		0,    /* Status of the configuration request (Should always be set to 0)  */
		0,    /* Core Number of the REQUESTOR                                     */
		100,  /* Number of General Purpose Queues requested                       */
		3,    /* Number of CPINTC Output  requested                               */
		2,    /* Number of Accumulator Channels requested                         */
		0,    /* Number of Hardware Semaphores requested                          */
        1,    /* Number of QPEND Queues requested                                 */
		/* Requested Memory Region Configuration. */
		{
            /* Name,           Type,                      Linking RAM,                            Num,      Size */
			{ "Core0-DDR3-0", Resmgr_MemRegionType_DDR3,  Resmgr_MemRegionLinkingRAM_DONT_CARE,   16384,    128},
		}
	},
    /* CORE 1: Requested Configuration. */
    {
        0,    /* Status of the configuration request (Should always be set to 0)  */
        1,    /* Core Number of the REQUESTOR                                     */
        0,    /* Number of General Purpose Queues requested                       */
        0,    /* Number of CPINTC Output  requested                               */
        1,    /* Number of Accumulator Channels requested                         */
        0,    /* Number of Hardware Semaphores requested                          */
        0,    /* Number of QPEND Queues requested                                 */
        /* Requested Memory Region Configuration. */
    },
    /* CORE 2: Requested Configuration. */
	{
		0,    /* Status of the configuration request (Should always be set to 0)  */
		2,    /* Core Number of the REQUESTOR                                     */
		0,    /* Number of General Purpose Queues requested                       */
		0,    /* Number of CPINTC Output  requested                               */
		0,    /* Number of Accumulator Channels requested                         */
		0,    /* Number of Hardware Semaphores requested                          */
        0,    /* Number of QPEND Queues requested                                 */
		/* Requested Memory Region Configuration. */
	},
    /* CORE 2: Requested Configuration. */
	{
		0,    /* Status of the configuration request (Should always be set to 0)  */
		3,    /* Core Number of the REQUESTOR                                     */
		0,    /* Number of General Purpose Queues requested                       */
		0,    /* Number of CPINTC Output  requested                               */
		0,    /* Number of Accumulator Channels requested                         */
		0,    /* Number of Hardware Semaphores requested                          */
        0,    /* Number of QPEND Queues requested                                 */
		/* Requested Memory Region Configuration. */
	},
};

