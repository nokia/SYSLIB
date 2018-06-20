//******************************************************************************
//
// Copyright 2016 Nokia, All Rights Reserved
//
//******************************************************************************

#ifndef NETFP_SYSTEM_PRINTF_H
#define NETFP_SYSTEM_PRINTF_H

void Osal_syslibLoggingFunction(const char* format, ...) __attribute__((weak));

#define System_printf(...) do { \
                               if(Osal_syslibLoggingFunction) \
                                   Osal_syslibLoggingFunction(__VA_ARGS__); \
                               else \
                                   printf(__VA_ARGS__); \
                           } while(0)

#endif // NETFP_SYSTEM_PRINTF_H
