/**
 *   @file  dat_verbosity_levels.h
 *
 *   @brief
 *      Common header file that lists all the debug/trace event levels.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2012 Texas Instruments, Inc.
 *  \par
 */

/**
 * @brief
 *  Bitmasks for class trace levels.
 *  Enabled BIOS load logging
 */
#define DAT_CLASS_LEVEL_BIOSLOAD   0x0001

/**
 * @brief
 *  Bitmasks for class trace levels.
 *  Enabled SYS/BIOS task logging
 */
#define DAT_CLASS_LEVEL_BIOSTASK   0x0002

/**
 * @brief
 *  Bitmasks for class trace levels.
 *  Enabled SYS/BIOS main module logging
 */
#define DAT_CLASS_LEVEL_BIOSMAIN   0x0004

/**
 * @brief
 *  Bitmasks for class trace levels.
 *  Enabled SYS/BIOS HWI logging
 */
#define DAT_CLASS_LEVEL_HWI        0x0008

/**
 * @brief
 *  Bitmasks for class trace levels.
 *  Enabled SYS/BIOS SWI logging
 */
#define DAT_CLASS_LEVEL_SWI        0x0010

/**
 * @brief
 *  Bitmasks for class trace levels.
 *  User-defined trace level. Can be uniquely defined for each class.
 */
#define DAT_CLASS_LEVEL_USERDEF0   0x0020

/**
 * @brief
 *  Bitmasks for class trace levels.
 *  User-defined trace level. Can be uniquely defined for each class.
 */
#define DAT_CLASS_LEVEL_USERDEF1   0x0040

/**
 * @brief
 *  Bitmasks for class trace levels.
 *  User-defined trace level. Can be uniquely defined for each class.
 */
#define DAT_CLASS_LEVEL_USERDEF2   0x0080

/**
 * @brief
 *  Bitmasks for class trace levels.
 * User-defined trace level. Can be uniquely defined for each class.
 */
#define DAT_CLASS_LEVEL_USERDEF3   0x0100

/**
 * @brief
 *  Bitmasks for class trace levels.
 *  User-defined trace level. Can be uniquely defined for each class.
 */
#define DAT_CLASS_LEVEL_USERDEF4   0x0200

/**
 * @brief
 *  Bitmasks for class trace levels.
 * User-defined trace level. Can be uniquely defined for each class. */
#define DAT_CLASS_LEVEL_USERDEF5   0x0400

/**
 * @brief
 *  Bitmasks for class trace levels.
 * User-defined trace level. Can be uniquely defined for each class. */
#define DAT_CLASS_LEVEL_USERDEF6   0x0800

/**
 * @brief
 *  Bitmasks for class trace levels.
 * User-defined trace level. Can be uniquely defined for each class. */
#define DAT_CLASS_LEVEL_USERDEF7   0x1000

/**
 * @brief
 *  Bitmasks for class trace levels.
 * User-defined trace level. Can be uniquely defined for each class. */
#define DAT_CLASS_LEVEL_USERDEF8   0x2000

/**
 * @brief
 *  Bitmasks for class trace levels.
 * User-defined trace level. Can be uniquely defined for each class. */
#define DAT_CLASS_LEVEL_USERDEF9   0x4000

/**
 * @brief
 *  Bitmasks for disabling all the traces.
 */
#define DAT_CLASS_LEVEL_DISABLEALL 0x8000

/**
 * @brief
 *  Bitmasks for component trace levels.
 *  Critical errors or exceptions
 */
#define DAT_COMP_LEVEL_EXCEPTION   0x0001

/**
 * @brief
 *  Bitmasks for component trace levels.
 *  Errors that can be recovered from */
#define DAT_COMP_LEVEL_ERROR       0x0002

/**
 * @brief
 *  Bitmasks for component trace levels.
 * Warning messages */
#define DAT_COMP_LEVEL_WARNING     0x0004

/**
 * @brief
 *  Bitmasks for component trace levels.
 * High-level debug messages; important information printed occasionally */
#define DAT_COMP_LEVEL_PE0         0x0008

/**
 * @brief
 *  Bitmasks for component trace levels.
 * Mid-level debug messages; printed less frequently (example: less than once per TTI) */
#define DAT_COMP_LEVEL_PE1         0x0010

/**
 * @brief
 *  Bitmasks for component trace levels.
 * Low-level debug messages; printed frequently (example: many per TTI) */
#define DAT_COMP_LEVEL_PE2         0x0020

/**
 * @brief
 *  Bitmasks for component trace levels.
 * Benchmarks for the component */
#define DAT_COMP_LEVEL_BENCHMARK0  0x0040

/**
 * @brief
 *  Bitmasks for component trace levels.
 * Benchmarks for the component */
#define DAT_COMP_LEVEL_BENCHMARK1  0x0080

/**
 * @brief
 *  Bitmasks for component trace levels.
 * User-defined trace level. Can be uniquely defined for each component. */
#define DAT_COMP_LEVEL_USERDEF0    0x0100

/**
 * @brief
 *  Bitmasks for component trace levels.
 * User-defined trace level. Can be uniquely defined for each component. */
#define DAT_COMP_LEVEL_USERDEF1    0x0200

/**
 * @brief
 *  Bitmasks for component trace levels.
 * User-defined trace level. Can be uniquely defined for each component. */
#define DAT_COMP_LEVEL_USERDEF2    0x0400

/**
 * @brief
 *  Bitmasks for component trace levels.
 * User-defined trace level. Can be uniquely defined for each component. */
#define DAT_COMP_LEVEL_USERDEF3    0x0800

/**
 * @brief
 *  Bitmasks for component trace levels.
 * User-defined trace level. Can be uniquely defined for each component. */
#define DAT_COMP_LEVEL_USERDEF4    0x1000

/**
 * @brief
 *  Bitmasks for component trace levels.
 * User-defined trace level. Can be uniquely defined for each component. */
#define DAT_COMP_LEVEL_USERDEF5    0x2000

/**
 * @brief
 *  Bitmasks for component trace levels.
 * User-defined trace level. Can be uniquely defined for each component. */
#define DAT_COMP_LEVEL_USERDEF6    0x4000

/**
 * @brief
 *  Bitmasks for component trace levels.
 * User-defined trace level. Can be uniquely defined for each component. */
#define DAT_COMP_LEVEL_USERDEF7    0x8000

