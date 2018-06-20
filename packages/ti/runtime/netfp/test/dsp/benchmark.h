/**
 *   @file  benchmark.h
 *
 *   @brief
 *      The header file has macros which are used to collect the 
 *      profiling information for the specific functions.
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

#ifndef __BENCHMARK_H__
#define __BENCHMARK_H__

/**
 * @brief   Use this macro to benchmark a specific function. It will 
 * internally declared all the variables required to keep track of the
 * benchmarking information.
 */
#define BENCHMARK_INIT(FXN_NAME)                            \
        static uint64_t    start_##FXN_NAME;                \
        static uint64_t    end_##FXN_NAME;                  \
        static uint32_t    count_##FXN_NAME = 0;            \
        static uint32_t    min_##FXN_NAME   = 0xFFFFFFFF;   \
        static uint32_t    max_##FXN_NAME   = 0x0;          \
        static uint64_t    total_##FXN_NAME = 0x0;          \

/**
 * @brief   This should be called just before the function being profiled is called.
 */
#define BENCHMARK_START(FXN_NAME)               start_##FXN_NAME = _itoll (TSCH, TSCL)

/**
 * @brief   This should be called just after the function being profiled returns.
 */
#define BENCHMARK_END(FXN_NAME)                 end_##FXN_NAME = _itoll (TSCH, TSCL)

/**
 * @brief   Use this macro to update the various benchmarking counters
 */
#define BENCHMARK_UPDATE(FXN_NAME)                                                      \
    do {                                                                                \
        min_##FXN_NAME   = _minu4(min_##FXN_NAME, (end_##FXN_NAME - start_##FXN_NAME)); \
        max_##FXN_NAME   = _maxu4(max_##FXN_NAME, (end_##FXN_NAME - start_##FXN_NAME)); \
        count_##FXN_NAME = count_##FXN_NAME + 1;                                        \
        total_##FXN_NAME = total_##FXN_NAME + (end_##FXN_NAME - start_##FXN_NAME);      \
    } while (0)

/**
 * @brief   Use this macro to display the benchmarking information for the specific
 * function.
 */
#define BENCHMARK_DISPLAY(FXN_NAME)                                                 \
    do {                                                                            \
        System_printf ("%s(%d) -> Min %d Max %d Avg %d \n",                         \
                       #FXN_NAME, count_##FXN_NAME, min_##FXN_NAME,                 \
                       max_##FXN_NAME, total_##FXN_NAME/count_##FXN_NAME);          \
    } while (0)

#endif /* __BENCHMARK_H__ */
