/**
 *   @file  master.h
 *
 *   @brief
 *      Header file used by the RAT master
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
#ifndef __MASTER_H__
#define __MASTER_H__

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/**********************************************************************
 ************************ Local Definitions ***************************
 **********************************************************************/

/* Base domain identifiers for the LTE Release9 */
#define LTE_REL9A_DOMAIN_ID         0x1000
#define LTE_REL9B_DOMAIN_ID         0x2000

/* Base domain identifiers for the LTE Release10 Deployment1 */
#define LTE_REL10D1_L2_DOMAIN_ID     0xF000
#define LTE_REL10D1_L1A_DOMAIN_ID    0xE000
#define LTE_REL10D1_L1B_DOMAIN_ID    0xD000

/* Base domain identifiers for the LTE Release10 Deployment2 */
#define LTE_REL10D2_L2A_DOMAIN_ID    0x0100
#define LTE_REL10D2_L2B_DOMAIN_ID    0x0200
#define LTE_REL10D2_L1A_DOMAIN_ID    0x0300
#define LTE_REL10D2_L1B_DOMAIN_ID    0x0400

#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_BLUE    "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN    "\x1b[36m"
#define ANSI_COLOR_RESET   "\x1b[0m"

static inline void LOG_ERROR (char* fmt, ...)
{
    va_list arg;

    va_start (arg, fmt);
    printf ("%s", ANSI_COLOR_RED);
    vprintf (fmt, arg);
    printf ("%s", ANSI_COLOR_RESET);
    va_end (arg);
}

static inline void LOG_DEBUG (char* fmt, ...)
{
    va_list arg;

    va_start (arg, fmt);
    printf ("%s", ANSI_COLOR_RESET);
    vprintf (fmt, arg);
    printf ("%s", ANSI_COLOR_RESET);
    va_end (arg);
}

static inline void LOG_INFO (char* fmt, ...)
{
    va_list arg;

    va_start (arg, fmt);
    printf ("%s", ANSI_COLOR_GREEN);
    vprintf (fmt, arg);
    printf ("%s", ANSI_COLOR_RESET);
    va_end (arg);
}

/********************************************************************************************
 * Extern Declarations:
 ********************************************************************************************/

/* Core Master Exported: */
extern void Rat_DoPolicyOffloads (Name_DBHandle nameDBHandle, pid_t netfpProxyPid);
extern void Rat_DoPolicyOffloads_FromFile (Name_DBHandle nameDBHandle, char* cellName, pid_t netfpProxyPid);
extern pid_t Rat_SpawnSyslibServer(char* syslibServerName, char* argv[]);

/* Release10 Deployment2 Exported: */
extern void Rat_Rel10D2_CLI (void);

/* Release10 Deployment1 Exported: */
extern void Rat_Rel10D1_CLI (void);

/* Release9 Exported: */
extern void Rat_Rel9CLI (void);


#endif /* __MASTER_H__ */
