/**
 *   @file  trace_phy.h
 *
 *   @brief
 *      Header file defining the PHY trace components. 
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2012 Texas Instruments, Inc.
 *  \par
 */


/**
 * @brief 
 *  Enumeration which lists the trace component IDs of PHY
 */
typedef enum Trace_componentType
{
    TRACE_COMPONENT_PHY_EQUALIZER = 0,
    TRACE_COMPONENT_PHY_PRACH,
    TRACE_COMPONENT_PHY_PUCCH,
    TRACE_COMPONENT_PHY_PDSCH,
    TRACE_COMPONENT_PHY_PUSCH,
    TRACE_COMPONENT_PHY_DAT,    /* Only for unit test purposes. */
    TRACE_COMPONENT_PHY_MAX
}Trace_componentType;

/**
 * @brief
 *  List of PHY component names
 */
char* phyComponentList[] =
{
    "TRACE_COMPONENT_PHY_EQUALIZER",
    "TRACE_COMPONENT_PHY_PRACH",
    "TRACE_COMPONENT_PHY_PUCCH",
    "TRACE_COMPONENT_PHY_PDSCH",
    "TRACE_COMPONENT_PHY_PUSCH",
    "TRACE_COMPONENT_PHY_DAT",    /* Only for unit test purposes. */
};

/* In this example, max TCids issued to PHY is 6. */
#define TRACE_NUM_COMPONENT_PHY     TRACE_COMPONENT_PHY_MAX

