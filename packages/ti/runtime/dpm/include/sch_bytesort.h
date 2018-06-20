#ifndef SCH_BYTESORT_H
#define SCH_BYTESORT_H

#ifdef ORG_CODE
#include "sch_types.h"
#else
#include <ti/runtime/dpm/include/sch_types.h>
#endif

//int32_t sortindex_byte (int16_t N, int16_t K, int16_t start_byte, int16_t start_index, SORT_KEY_TYPE * restrict array, SORT_INDEX_TYPE * restrict index);
//void sortindex_bit32 (int16_t N, int16_t K, SORT_KEY_TYPE * restrict key, SORT_INDEX_TYPE * restrict index);
//static inline uint32_t threshold_search_first_byte(uint32_t * restrict intlvArr, uint32_t N, uint32_t K, uint32_t Kmargin, uint32_t i_start);
//static inline uint32_t threshold_search_second_byte(uint32_t * restrict intlvArr, uint32_t N, uint32_t K, uint32_t Kmargin, uint32_t i_start);
//static inline uint32_t threshold_search_third_byte(uint32_t * restrict intlvArr, uint32_t N, uint32_t K, uint32_t Kmargin, uint32_t i_start);
//static inline uint32_t threshold_search_last_byte(uint32_t * restrict intlvArr, uint32_t N, uint32_t K, uint32_t Kmargin, uint32_t i_start);
//static inline uint32_t threshold_search_first_byte(uint32_t * restrict intlvArr, uint32_t N, uint32_t K, uint32_t i_start);
//static inline uint32_t threshold_search_next_byte(uint32_t * restrict intlvArr, uint32_t N, uint32_t K, uint32_t i_start);
static inline uint32_t threshold_search(uint32_t * restrict intlvArr, uint32_t N, uint32_t K, uint32_t Kmargin);
static inline uint32_t threshold_search_zero_next(uint32_t * restrict intlvArr, uint32_t N, uint32_t K, uint32_t Kmargin, uint32_t i_start);
//static inline uint32_t threshold_search(uint32_t * restrict intlvArr, uint32_t N, uint32_t K);
//static inline uint32_t threshold_search_zero_next(uint32_t * restrict intlvArr, uint32_t N, uint32_t K, uint32_t thrsh_start, uint8_t ite_left, uint32_t i_start);

#ifndef ORG_CODE
extern uint32_t out_index[];
#endif

uint32_t selectindex_bit32 (int16_t N, int16_t K, SORT_KEY_TYPE * restrict key, SORT_INDEX_TYPE * restrict index);

#endif /* SCH_BYTESORT_H */

