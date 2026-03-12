#if !defined(HT15_UTIL)
#define HT15_UTIL

#include "definitions.h"

/* alignment represents the size of the buffer at 1 << alignment kindof like int2 int3 in fasm */
static inline u32 circle_buffer_index_at(u8 alignment, i32 index);

/* returns time in milis sense device turned on */
time32 get_time();

#if defined(HT15_UTIL_IMPLEMENTATION)
static inline u32 circle_buffer_index_at(u8 alignment, i32 index){
        return index & ((1<<alignment)-1);
}

time32 get_time(){
    /* TODO:*/
    return 0;
}


#endif /* HT15_UTIL_IMPLEMENTATION */

#endif /* HT15_UTIL */
