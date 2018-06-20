#ifndef SCH_MERGESORT_H
#define SCH_MERGESORT_H

#ifdef ORG_CODE
#include "sch_types.h"
#else
#include <ti/runtime/dpm/include/sch_types.h>
#endif

static int do_merge(unsigned int **inputs, int *sizes, unsigned int **indexes, int count, unsigned int *output, unsigned int *order);
void SortIndex_merge32(unsigned int * restrict array, int size, unsigned int * restrict index, unsigned int * restrict output, unsigned int * restrict order, unsigned int * restrict scratch);
void MergeIndex_twoWays32 (unsigned int **inputs, int *sizes, unsigned int **indexes, \
						   unsigned int * restrict out, unsigned int * restrict order, unsigned int * restrict scratch);
void MergeIndex_threeWays32(unsigned int **inputs, int *sizes, unsigned int **indexes, unsigned int * restrict out, unsigned int * restrict order, unsigned int * restrict scratch);						   
void MergeIndex_fourWays32(unsigned int **inputs, int *sizes, unsigned int **indexes, unsigned int * restrict out, unsigned int * restrict order, unsigned int * restrict scratch);


#endif /* SCH_MERGESORT_H */
