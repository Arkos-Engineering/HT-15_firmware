#if !defined(HT15_UTIL)
#define HT15_UTIL

#include "definitions.h"

/* alignment represents the size of the buffer at 1 << alignment kindof like int2 int3 in fasm */
static inline u32 circle_buffer_index_at(u8 alignment, i32 index);
static inline bool8 test_bit_at_index(u8 * bits, u32 index);
static inline void set_bit_at_index(u8 * bits, u32 index, bool8 bit);

/* returns time in milis sense device turned on */
time32 get_time();

#if defined(HT15_UTIL_IMPLEMENTATION)
static inline u32 circle_buffer_index_at(u8 alignment, i32 index){
        return index & ((1<<alignment)-1);
}
/*TODO: make these take in a bitset struct that has a size. */
static inline bool8 test_bit_at_index(u8 * bits, u32 index){
    u32 byte_index = index >> 3;
    u32 bit_offset = index & 7;
    return (bits[byte_index] >> bit_offset) & 1;
}

static inline void set_bit_at_index(u8 * bits, u32 index, bool8 bit) {
    u32 byte_index = index >> 3;
    u32 bit_offset = index & 7;
    bits[byte_index] = (bits[byte_index] & ~(1 << bit_offset)) | (bit << bit_offset);
}

time32 get_time(){
    /* TODO:*/
    return 0;
}


#endif /* HT15_UTIL_IMPLEMENTATION */

#endif /* HT15_UTIL */
