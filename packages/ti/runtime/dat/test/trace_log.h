/**
 *   @file  trace_log.h
 *
 *   @brief
 *      Header file containing the logging APIs.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2012 Texas Instruments, Inc.
 *  \par
 */


/* UIA Logging and DAT Include Files */
#include <ti/uia/events/UIAEvt.h>
#include <ti/uia/runtime/LogUC.h>
#include <ti/runtime/dat/dat.h>

/**********************************************************************
 ************************ Trace_log Extern Variables ******************
 **********************************************************************/

/* Handle to the trace object. */
extern   Dat_TraceObjectBody*    ptrTraceObjBody;

/* Default logger definition */
#ifdef __ARMv7
extern void* logger0;
#endif

/**********************************************************************
 ************************** Trace_log Definitions *********************
 **********************************************************************/

/**
 * @brief
 *  Defines for focused and non-focused component instances.
 */
#define TRACE_NON_FOCUSED           0
#define TRACE_FOCUSED               1

/**
 * @brief
 *  Default event code used in Log_write
 *  APIs of UIA Event Type
 */
#define TRACE_DEFAULT_EVENT_CODE    0

/**********************************************************************
 ************************** Trace_log Functions ***********************
 **********************************************************************/

/**
 * @brief
 *  Logging function with 2 arguments
 *
 * @details
 *  Function acts as a filter based on verbosity
 *  settings and writes to the producer buffer.
 */
static inline void Trace_log2
(
    Trace_componentType     tcId,
    Dat_TraceLevel          eventLevel,
    uint32_t                isFocused,
    xdc_runtime_Log_Event   uiaEventType,
    IArg                    arg0,
    IArg                    arg1
)
{
    /* Event Filtering based on the component and class verbosity settings is
     * done in DAT. Check if the specified level is loggable for this
     * component. */
    if (Dat_filter (ptrTraceObjBody, tcId, eventLevel, isFocused))
        Log_iwriteUC2 (logger0, uiaEventType, arg0, arg1);
}

/**
 * @brief
 *  Logging function with 3 arguments
 *
 * @details
 *  Function acts as a filter based on verbosity
 *  settings and writes to the producer buffer.
 */
static inline void Trace_log3
(
    Trace_componentType     tcId,
    Dat_TraceLevel          eventLevel,
    uint32_t                isFocused,
    xdc_runtime_Log_Event   uiaEventType,
    IArg                    arg0,
    IArg                    arg1,
    IArg                    arg2
)
{
    /* Event Filtering based on the component and class verbosity settings is
     * done in DAT. Check if the specified level is loggable for this
     * component. */
    if (Dat_filter (ptrTraceObjBody, tcId, eventLevel, isFocused))
        Log_iwriteUC3 (logger0, uiaEventType, arg0, arg1, arg2);
}

/**
 * @brief
 *  Logging function with 4 arguments
 *
 * @details
 *  Function acts as a filter based on verbosity
 *  settings and writes to the producer buffer.
 */
static inline void Trace_log4
(
    Trace_componentType     tcId,
    Dat_TraceLevel          eventLevel,
    uint32_t                isFocused,
    xdc_runtime_Log_Event   uiaEventType,
    IArg                    arg0,
    IArg                    arg1,
    IArg                    arg2,
    IArg                    arg3
)
{
    /* Event Filtering based on the component and class verbosity settings is
     * done in DAT. Check if the specified level is loggable for this
     * component. */
    if (Dat_filter (ptrTraceObjBody, tcId, eventLevel, isFocused))
        Log_iwriteUC4 (logger0, uiaEventType, arg0, arg1, arg2, arg3);
}


