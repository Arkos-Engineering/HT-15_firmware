#if !defined(HT15_DEFINITIONS)
#define HT15_DEFINITIONS

#include <stdint.h>
#include <inttypes.h>

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;

typedef int8_t i8;
typedef int16_t i16;
typedef int32_t i32;
typedef u32 time32;

/* TODO @Zea as of December 20 2025: we should probably use fixed point values here instead of software floats
    At some point we should poison these values and remove all floats and doubles from the code. */
typedef float f32;
typedef double f64;

typedef _Bool bool8;

typedef char const * c_str;

#define U32_MAX UINT32_MAX

#define ifor(i, v) for(u32 i = 0; i < (u32)v; ++i)
#define array_size(a) (sizeof(a)/sizeof(*a))
#define alignof(x) __alignof__(x)

#define MHZ 1000000
#define KHZ 1000

#define PROCESSOR_CLOCK_MHZ 150

//I2C addresses
#define audioamp_i2c_address 0b0011000

//PIO assignments, PIO block and state machine
#define ENCODER_PIO pio0
#define ENCODER_SM 0
#define I2S_MIC_PIO pio1
#define I2S_MIC_SM 0
#define I2S_HEADSET_MIC_PIO pio1
#define I2S_HEADSET_MIC_SM 1
#define I2S_OUTPUT_PIO pio1
#define I2S_OUTPUT_SM 2

#define PINS \
    X(buttonmatrix_a,       0)\
    X(buttonmatrix_b,       1)\
    X(buttonmatrix_c,       2)\
    X(buttonmatrix_d,       3)\
    X(buttonmatrix_0,       4)\
    X(buttonmatrix_1,       5)\
    X(buttonmatrix_2,       6)\
    X(buttonmatrix_3,       7)\
    X(buttonmatrix_4,       8)\
    X(buttonmatrix_5,       9)\
    X(spi1_clk,             10)\
    X(spi1_sdi,             11)\
    X(spi1_sdo,             12)\
    X(audioamp_masterclock, 13)\
    X(audioamp_sdi,         14)\
    X(audioamp_scl,         15)\
    X(audioamp_wordselect,  16)\
    X(audioamp_sdo,         17)\
    X(sd_cs,                18)\
    X(sd_card_detect,       19)\
    X(flash_cs,             20)\
    X(display_cs,           21)\
    X(display_busy,         22)\
    X(display_reset,        23)\
    X(charger_status,       24)\
    X(audioamp_reset,       25)\
    X(mars,                 26)\
    X(led_status,           27)\
    X(mic_scl,              28)\
    X(mic_sdo,              29)\
    X(mic_wordselect,       30)\
    X(encoder_a,            31)\
    X(encoder_b,            32)\
    X(rf_gpio2,             33)\
    X(rf_gpio1,             34)\
    X(rf_gpio0,             35)\
    X(rf_sdo,               36)\
    X(rf_sel,               37)\
    X(rf_sclk,              38)\
    X(rf_sdi,               39)\
    X(rf_gpio3,             40)\
    X(headset_ptt,          41)\
    X(i2c1_sda,             42)\
    X(i2c1_scl,             43)\
    X(headset_conn,         44)\
    X(pot_volume,           45)\
    X(chgr_conn,            46)\
    X(v_bat,                47)

typedef enum {
    #define X(name, pin) pin_##name = pin,
    PINS
    #undef X
    pin_max_enum,
    pin_unknown = U32_MAX
} pin;

#define BLINK_CODES\
    X(okay, 0)\
    X(stdio_failed_to_initalize, 3)\
    X(next_blink_code_idk, 7)\

typedef enum{
    #define X(name, code) blink_code_##name = code,
    BLINK_CODES
    #undef X
    blink_codes_max_enum,
    blink_codes_unknown = U32_MAX
} blink_code;

#define KEYS\
    X(up)\
    X(down)\
    X(left)\
    X(right)\
    X(lock)\
    X(side1)\
    X(side2)\
    X(ptt)\
    X(back)\
    X(enter)\
    X(1)\
    X(2)\
    X(3)\
    X(4)\
    X(5)\
    X(6)\
    X(7)\
    X(8)\
    X(9)\
    X(0)\
    X(star)\
    X(hash)

typedef enum {
    #define X(key) key_##key,
    KEYS
    #undef X
    key_max_enum,
    key_unknown = UINT32_MAX
} key_tag;
typedef struct key{u8 value;}key;

#define KEY_STATES\
    X(released)\
    X(pressed)\
    X(repeat)

typedef enum{
    #define X(s) key_state_##s,
    KEY_STATES
    #undef X
    key_state_max_enum,
    key_state_unknown = U32_MAX
} key_state;

typedef enum {
    spi1_device_none = 0,
    spi1_device_display,
    spi1_device_sd,
    spi1_device_flash,
} spi1_device;

#endif /*HT15_DEFINITIONS*/
