#ifndef SCH_TYPES_H
#define SCH_TYPES_H

#ifdef _MSC_VER
typedef __int64				int64_t;
typedef unsigned __int64	uint64_t;
typedef __int32				int32_t;
typedef unsigned __int32	uint32_t;
typedef __int16				int16_t;
typedef unsigned __int16	uint16_t;
typedef __int8				int8_t;
typedef unsigned __int8		uint8_t;
#else
#include <stdint.h>
#endif

//#include "CI_types.h"

#define MIN_INT64       (-(1<<63))      /* ie -2^63 */
#define MAX_INT64       (0x7FFFFFFF00000000) /* ie  2^63-1 */
#define MIN_INT32       (-2147483648)   /* ie -2^31 */
#define MAX_INT32       (2147483647)   /* ie -2^31 */
#define MIN_INT16       (-32768)        /* ie -2^15 */ 
#define MAX_INT16       (32767)         /* ie  2^15-1 */
#define MIN_INT8		(-128)			/* ie -2^7 */
#define MAX_INT8		(127)			/* ie 2^7-1 */
#define MAX_UINT32      (4294967295)    /* ie 2^32-1 */
#define MAX_UINT16      (65535)         /* ie 2^16-1 */
#define MAX_UINT8       (255)           /* ie 2^8-1 */

/* typedef Uint32 UInt32;
typedef Uint16 UInt16;
typedef Uint8  UInt8;
typedef Uint64 UInt64; */

#define min(a,b)        (((a)>(b))? (b):(a))
#define max(a,b)        (((a)<(b))? (b):(a))
#define abs(a)          (((a)<0)? -(a):(a))

#define TRUE					1
#define FALSE					0

/* #define NULL					0 */
#define NUMTTIS                 100 /* ms */
#define NUMPRBS                 100 /* PRBs */
#define NUMUES                  512
#define MAX_UEPWR               23 	/* dBm */
#define TTISTART				9

#define SORT_INDEX_TYPE uint32_t
#define SORT_KEY_TYPE   uint32_t
#define SORT_KEY_MIN 	0 			/* MIN_INT32 for int32_t key type */

//typedef CI_Bool     tbool;
//typedef Uint8       tuchar;
//typedef Int8        tchar;
//typedef Uint16      tuint;
//typedef Int16       tint;
//typedef Uint32      tulong;
//typedef Int32       tlong;
//typedef Uint64      tulonglong;
//typedef Int64       tlonglong;

//typedef Uint16      UFract;
//typedef Int16       Fract;
//typedef Uint32      ULFract;
//typedef Int32       LFract;

#endif /* SCH_TYPES_H */
