/**
 *   @file  testautomation.c
 *
 *   @brief
 *      Unit Test Code for the message communicator module which executes
 *      on Core0
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
#include <stdint.h>

/* BIOS/XDC Include Files. */
#include <xdc/runtime/System.h>
#include <ti/sysbios/family/c66/Cache.h>


#ifdef ARM_DSP_DOWNLOAD
/* parameters for DSP in  .test_automation */
/* data section                            */
typedef struct DSP_CLI{
    uint32_t magic;     //4 bytes
    uint32_t numparams; //8 bytes
    uint32_t param[20]; //88 bytes
    uint32_t netfpdatSz;//92 bytes
    char netfpdat[1400];//1492 bytes
}DSP_CLI;

static unsigned int paramIndex=0;

/**
 * Data section for CLI test automation.
 * Selection for CLI simulation is written
 * by arm application to data section .testautomation
 * befOre DSP is started. The size of data section
 * must be multiple of cache line size.
 */

#define test_automationSZ 12*128 //1536 bytes
#pragma DATA_SECTION (test_automation, ".test_automation")
uint32_t test_automation[test_automationSZ/sizeof(uint32_t)];


static int validateTAsection(DSP_CLI* dsp_cli)
{
    if(paramIndex==0){
        /* First time user interaction is requested. */
        /* Invalidate test automation data section   */
        /* Validate the parametets                   */
        Cache_inv(test_automation, test_automationSZ, Cache_Type_ALLD, 1);
        
        if(dsp_cli->magic != 0xabbababe){
            System_printf("\nError: wrong magic in test automation data section\n");
            return -1;
        }
    }
    return 0;
}

static uint32_t getUserInputTA(void)
{
    DSP_CLI* dsp_cli=(DSP_CLI*)test_automation;
    if(paramIndex==0){
        /* First time user interaction is requested. */
        /* Invalidate test automation data section   */
        /* Validate the parametets                   */

        if(validateTAsection(dsp_cli)){
            return UINT32_MAX;
        }
        System_printf("\nTest automation %lu parameters\n",dsp_cli->numparams);
        if(dsp_cli->numparams >20){
            System_printf("\nError: wrong number of parameters\n");
            return UINT32_MAX;
        }
        
        System_printf("\nTest automation: selection %lu\n",dsp_cli->param[0]);
        paramIndex++;
        return dsp_cli->param[0];
    }else{
        /* Return the next parameter */
        if(paramIndex<dsp_cli->numparams){
            return (dsp_cli->param[paramIndex++]);
        }else{
            System_printf("Error: test automation out of parameters\n");
            return UINT32_MAX;
        }
    }
}


char* testautomation_netfpdatPtr(void)
{
    DSP_CLI* dsp_cli=(DSP_CLI*)test_automation;
    if(validateTAsection(dsp_cli)){
        return NULL;
    }
    return dsp_cli->netfpdat;
}
testautomation_netfpdatSz(void)
{
    DSP_CLI* dsp_cli=(DSP_CLI*)test_automation;
    if(validateTAsection(dsp_cli)){
        return NULL;
    }
    return dsp_cli->netfpdatSz;
}


#endif

uint32_t getUserInput(void)
{
    uint32_t testSelection;

#ifdef ARM_DSP_DOWNLOAD
    testSelection = getUserInputTA();
#else
    /* Wait for the user input. */
    scanf ("%d", &testSelection);
#endif    
    return testSelection;
}
