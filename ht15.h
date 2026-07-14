/* main firmware library file*/
#if !defined(HT15_H)
#define HT15_H

#include "definitions.h"

#if defined(__cplusplus)
#extern "C" {
#endif

#if !defined(HT15_EXPORT)
#define HT15_EXPORT 
#endif

HT15_EXPORT bool8 ht15_initalize(void);
HT15_EXPORT bool8 ht15_run(void);

#if defined(__cplusplus)
}
#endif

/* NO_CHECKIN */ #define HT15_IMPLEMENTATION
#if defined(HT15_IMPLEMENTATION) | defined(MOCK_RADIO)

#if !defined(MOCK_RADIO)
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <pico.h>
#include <pico/stdlib.h>
#include "pico/multicore.h"
#include "pico/rand.h"
#include "pico/mutex.h"
#include "hardware/adc.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include <pico/bootrom.h>

#define HT15_UTIL_IMPLEMENTATION
#include "util.h"

#define HT15_AUDIO_TOOLKIT_IMPLEMENTATION
#include "audio_toolkit.h"

#define PICO_SSD1681_IMPLEMENTATION
#include"pico_ssd1681.h"

#define PICO_TLV320DAC3100_IMPLEMENTATION
#include "pico_tlv320dac3100.h"

#define RFMODULE_2M70CM_IMPLEMENTATION
#include "pico_rfmodule_2M70CM.h"

#include "quadrature_encoder.pio.h"

#define PICO_I2S_MIC_IMPLEMENTATION
// Looks like we are going to have to rewire the PCB. This PIO library needs DIN, BCLK, WORDSELECT in that order. on the mic I need to swap SCL and SDO. On the codec, I need to swap SDI, SDO
// #include "pico_i2s.h"
#include "ht15_i2s_mic_in.pio.h"

#define PICO_I2S_CODEC_IMPLEMENTATION
#include "ht15_i2s_codec_io.pio.h"

mutex_t rfmodule_mutex;


rfmodule_2m70cm_state_t rfmodule_state = {
    .config = {
        .spi_port = spi0,
        .spi_pin_mosi = pin_rf_sdi,
        .spi_pin_miso = pin_rf_sdo,
        .spi_pin_sck = pin_rf_sclk,
        .spi_pin_cs = pin_rf_sel,
        .spi_baudrate = 10 * MHZ,
        .spi_shared = false,

        .i2c_port = i2c1,
        .i2c_pin_sda = pin_i2c1_sda,
        .i2c_pin_scl = pin_i2c1_scl,
        .i2c_baudrate = 100 * KHZ,
        .i2c_shared = true,

        .pin_gpio0 = pin_rf_gpio0,
        .pin_gpio1 = pin_rf_gpio1,
        .pin_gpio2 = pin_rf_gpio2,
        .pin_gpio3 = pin_rf_gpio3,
    },
};


const u8 button_sense_pin[] = {pin_buttonmatrix_0, pin_buttonmatrix_1, pin_buttonmatrix_2, pin_buttonmatrix_3, pin_buttonmatrix_4, pin_buttonmatrix_5};
const u8 button_power_pin[] = {pin_buttonmatrix_a, pin_buttonmatrix_b, pin_buttonmatrix_c, pin_buttonmatrix_d};

bool8 led_status_value = 1;
key_state key_states[key_max_enum] = {key_state_released};
u32 last_key_changed_timestamp[key_max_enum] = {0};
u32 millis_until_repeating = 420;

#define BUTTONS_SIZE array_size(button_power_pin) * array_size(button_sense_pin)
#define BUTTON_DEBOUNCE_BUFFER_SIZE 1<<2
bool8 button_debounce_buffer[BUTTONS_SIZE * BUTTON_DEBOUNCE_BUFFER_SIZE] = {0}; 
u32 button_debounce_buffer_index = 0;

key_tag key_map[BUTTONS_SIZE] = {
    key_left, key_up,   key_right, key_side2,
    key_back, key_down, key_enter, key_side1,
    key_1,    key_2,    key_3,     key_ptt,
    key_4,    key_5,    key_6,     key_lock,
    key_7,    key_8,    key_9,     key_unknown,
    key_star, key_0,    key_hash,  key_unknown,
};

u32 encoder_last_value = 0;
u8 encoder_debounce_state = 0;

u8 last_volume = 0;
u8 current_volume = 0;

u8 selected_channel = 1;

tlv320dac3100_t audioamp;
static f32 calculate_volume(u8 volume);
static void audio_amp_set_volume(u8 volume);
static void audio_stop_system_clock();
static void audio_start_system_clock();
static void audio_amp_reset_hard();

static void i2c1_scan_bus(){
    printf("Scanning I2C1 bus...\n");
    for (i8 address = 1; address < 127; address++) {
        u8 txdata[] = {0x00};
        i16 ret = i2c_write_blocking(i2c1, address, txdata, 1, false);
        if (ret >= 0) {
            printf("Found device at address 0x%02X\n", address);
        }
    }
    printf("I2C1 scan complete.\n");
}

static f32 get_battery_voltage(){
    adc_select_input(pin_v_bat-40); /* V_BAT is ADC7, ADC input is 0 indexed */
    return (f32)adc_read()*.003791; /* conversion factor for voltage divider and ADC step size (127/27)*(3.3/4095) */
}

static u8 get_volume_pot(){
    adc_select_input(pin_pot_volume-40); /* POT_VOLUME is ADC5, ADC input is 0 indexed */
    return 99-((u8)((f32)adc_read()*0.02442));
    return 0;
}


static void print_button_debounce_buffer(){
    ifor(i, 1<<2){
        printf("button buffer %"PRIu32":\n", i);
        ifor(b, BUTTONS_SIZE){
            printf("button %"PRIu32" state %"PRIi8"\n", b, button_debounce_buffer[(i * BUTTONS_SIZE) + b]);
        }
        printf("\n");
    }
}

static void print_key_states(){
    static c_str key_names[key_max_enum] = {
        #define X(name) #name,
        KEYS
        #undef X
    };

    static c_str key_state_names[key_max_enum] = {
        #define X(name) #name,
        KEY_STATES
        #undef X
    };

    ifor(k, key_max_enum){
        printf("key %s : %s \n", key_names[k], key_state_names[key_states[k]]);
    } 
}

static void spi1_force_select(spi1_device device){
    while(spi_is_busy(spi1)){asm volatile("nop");} //wait for any ongoing SPI transactions to finish
    gpio_put(pin_display_cs, !(device == spi1_device_display));
    gpio_put(pin_flash_cs,   !(device == spi1_device_flash));
    gpio_put(pin_sd_cs,      !(device == spi1_device_sd));
}

static void rf_init(){
    if(rfmodule_2m70cm_init(&rfmodule_state)!=0){
        printf("Error initializing RF module!\n");
        return;
    } else{
        printf("RF module initialized successfully!\n");
    }
}

static void rf_transmit(u64 frequency_hz, bool8 amp_enable, f32 dbm, bool8 state){
    // rfmodule_2m70cm_write_register(&rfmodule_state, CC1200_REG_CFM_TX_DATA_IN, get_rand_32() & 0xFF); /* random data */
    // rfmodule_2m70cm_set_tx_data_raw(&rfmodule_state, get_rand_32()&0xFF);
    // printf("rf part/version: %02X / state %u\n",
    //     rfmodule_2m70cm_read_register(&rfmodule_state, CC1200_REG_PARTVERSION),
    //     rfmodule_2m70cm_read_register(&rfmodule_state, CC1200_REG_MARCSTATE));

    if(state==rfmodule_state.is_keyed){
        return;
    }

    mutex_enter_blocking(&rfmodule_mutex);

    if(!state){
        printf("RF test: turning off carrier\n");
        rfmodule_2m70cm_set_tx(&rfmodule_state, false);
        rfmodule_2m70cm_set_power_mode(&rfmodule_state, RFMODULE_2M70CM_POWER_MODE_RX_ONLY);
        
        mutex_exit(&rfmodule_mutex);
        return;
    }

    rfmodule_2m70cm_write_cmd(&rfmodule_state, CC1200_CMD_SIDLE);
    if(amp_enable){
        rfmodule_2m70cm_set_power_mode(&rfmodule_state, RFMODULE_2M70CM_POWER_MODE_ON);
        rfmodule_2m70cm_set_output_dbm(&rfmodule_state, dbm);
    } else{
        rfmodule_2m70cm_set_power_mode(&rfmodule_state, RFMODULE_2M70CM_POWER_MODE_OFF);
        cc1200_set_output_level(&rfmodule_state, dbm);

    }

    rfmodule_2m70cm_set_modulation(&rfmodule_state, RFMODULE_MODULATION_FM);
    printf("symbol rate set to: %fsps\n", rfmodule_2m70cm_set_symbol_rate_sps(&rfmodule_state, AUDIO_SAMPLE_RATE));
    printf("upsampler set to: %i\n", rfmodule_2m70cm_set_upsampler(&rfmodule_state, 64));
    rfmodule_2m70cm_set_bw(&rfmodule_state, (u32)(25*KHZ));
    rfmodule_2m70cm_set_frequency(&rfmodule_state, frequency_hz);
    // sleep_ms(1);
    rfmodule_2m70cm_set_tx_data_raw(&rfmodule_state, 0);
    rfmodule_2m70cm_set_tx(&rfmodule_state, true);

    mutex_exit(&rfmodule_mutex);

    return;
}

static void sd_init(){
/*     gpio_init(pin_sd_card_detect);
    gpio_set_dir(pin_sd_card_detect, GPIO_IN);

    pico_fatfs_set_config(&sd_cfg);

    if(gpio_get(pin_sd_card_detect) == 0){
        printf("SD card detected, initializing...\n");
        // sd_fatfs_return_code = f_mount(&sd_fatfs, "", 1);
        for (int i = 0; i < 5; i++) {
            sd_fatfs_return_code = f_mount(&sd_fatfs, "", 1);
            if (sd_fatfs_return_code == FR_OK) { break; }
            printf("Mount attempt %d failed with return code %d. Retrying...\n", i+1, sd_fatfs_return_code);
            pico_fatfs_reboot_spi();
        }
        if(sd_fatfs_return_code!= FR_OK){
            printf("Failed to mount SD card with return code %d\n", sd_fatfs_return_code);
            return -1;
        }
        // if(sd_fatfs_return_code == FR_OK){
        //     printf("SD card initialized successfully!\n");
        // } else {
        //     printf("SD card initialization failed with return code %d\n", sd_fatfs_return_code);
        //     return -1;
        // }
    } else {
        printf("No SD card detected.\n");
        return 0;
    }
    if (sd_fatfs_return_code == FR_OK) {
        printf("Creating test DIR");
        f_mkdir("HT15");
        // f_unmount("");
    }
    return 1; */
}

static void display_init(){
    ssd1681_config_t display_config;
    ssd1681_get_default_config_3wire(&display_config);
    display_config.spi_port = 1;
    display_config.spi_baudrate = 20 * MHZ;
    display_config.spi_mode = SSD1681_SPI_3WIRE;
    display_config.pin_mosi = pin_spi1_sdi;
    display_config.pin_sck = pin_spi1_clk;
    display_config.pin_cs = pin_display_cs;
    display_config.pin_rst = pin_display_reset;
    display_config.pin_busy = pin_display_busy;


    if(ssd1681_init(&display_config)!=0){
        printf("Error initializing display!\n");
        return;
    } else{
        printf("Display initialized successfully!\n");
    }

    // sleep_ms(100);
    // ssd1681_clear(SSD1681_COLOR_BLACK);
    // ssd1681_write_buffer(SSD1681_COLOR_BLACK);
    // ssd1681_update(SSD1681_UPDATE_CLEAN_FULL);

    ssd1681_clear(SSD1681_COLOR_BLACK);
    // ssd1681_clear(SSD1681_COLOR_RED);
    ssd1681_write_buffer(SSD1681_COLOR_BLACK);
    // ssd1681_write_buffer(SSD1681_COLOR_RED);
    ssd1681_update(SSD1681_UPDATE_CLEAN_FULL);
}

static void encoder_init(){
    gpio_init(pin_encoder_a);
    gpio_set_dir(pin_encoder_a, GPIO_IN);
    gpio_init(pin_encoder_b);
    gpio_set_dir(pin_encoder_b, GPIO_IN);

    //cant use PIO in the prototype. GPIO 31 and 32 are in different blocks and the quadrature encoder PIO program needs both pins to be in the same block.
    /* initialize quadrature encoder PIO program on pin_encoder_a and pin_encoder_b */
    // pio_add_program(ENCODER_PIO, &quadrature_encoder_program);
    // quadrature_encoder_program_init(ENCODER_PIO, ENCODER_SM, pin_encoder_a, 0);
}
static i8 encoder_get_difference(){
    encoder_debounce_state=((encoder_debounce_state<<1) | gpio_get(pin_encoder_a)) & 0b111;/* store the last three states of the encoder A pin */
    if(encoder_debounce_state==0b100){
        if(gpio_get(pin_encoder_b)){
            printf("Channel +\n");
            return 1;
        } else{
            printf("Channel -\n");
            return -1;
        }
    }
    return 0;
    
    // Below is for PIO. cant use it on prototype because of pin assignment issues mentioned in encoder_init()
    // u32 new_value = quadrature_encoder_get_count(ENCODER_PIO, ENCODER_SM);
    // i8 difference = (i32)(new_value - encoder_last_value);
    // printf("Encoder raw value: %"PRIu32"\n", new_value);
    // printf("Encoder difference: %"PRId8"\n", difference);
    // encoder_last_value = new_value;
    // return difference;
}

static void audio_amp_reset_hard(){
    gpio_init(pin_audioamp_reset);
    gpio_set_dir(pin_audioamp_reset, GPIO_OUT);
    gpio_put(pin_audioamp_reset, 0); /* start_reset */
    sleep_us(10);
    gpio_put(pin_audioamp_reset, 1); /* finish_reset */
}

static void audio_start_system_clock(){
    /* configure microcontroller to output required MCLK signal on pin specified by master_clock_pin, derived from system clock
    set clkout to the crystal oscillator frequency (12MHz) */
    clock_gpio_init(pin_audioamp_masterclock, CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_XOSC_CLKSRC, 1);
}

static void audio_stop_system_clock(){
    gpio_init(pin_audioamp_masterclock);
    gpio_set_dir(pin_audioamp_masterclock, GPIO_OUT);
    gpio_put(pin_audioamp_masterclock, 0);
}

/* set volume on audio amp (0-99) */
static void audio_amp_set_volume(u8 volume){
    f32 vol = calculate_volume(volume);
    tlv320_set_channel_volume(&audioamp, 0, vol);  /* Left channel */
    tlv320_set_channel_volume(&audioamp, 1, vol);   /* Right channel */
}

static void mic_init(u32 sample_rate){
    u32 sample_depth = 32;
    f32 clock_div = ((float)PROCESSOR_CLOCK_MHZ * MHZ)/((float)(sample_depth * sample_rate * 8 * 2));
    i8 offset = pio_add_program(I2S_MIC_PIO, &ht15_i2s_mic_in_program);
    if(offset >= 0){
        ht15_i2s_mic_in_program_init(I2S_MIC_PIO, I2S_MIC_SM, offset, pin_mic_scl, clock_div);
        printf("Mic I2S initialized successfully with clock divider of %f!\n", clock_div);
        return;
    } 
    printf("Mic failed to init. Could not fit program in PIO memory");
}

static void audio_codec_I2S_init(u32 sample_rate){
    u32 sample_depth = 32;
    f32 clock_div = ((float)PROCESSOR_CLOCK_MHZ * MHZ)/((float)(sample_depth * sample_rate * 10 * 2));
    i8 offset = pio_add_program(I2S_CODEC_PIO, &ht15_i2s_codec_io_program);
    if(offset >= 0){
        ht15_i2s_codec_io_init(I2S_CODEC_PIO, I2S_CODEC_SM, offset, pin_audioamp_sdi, clock_div);
        printf("Codec I2S initialized successfully with clock divider of %f!\n", clock_div);
        return;
    } 
    printf("Codec I2S failed to init. Could not fit program in PIO memory");
}

static void audio_codec_init(){
    // init I2S PIO block
    audio_codec_I2S_init(AUDIO_SAMPLE_RATE*AUDIO_CODEC_OVERSAMPLING_RATIO);


    audio_amp_reset_hard();

    // Initialize TLV320DAC3100 audio amplifier over I2C and init audio_config->audioamp
    tlv320_init(&audioamp, i2c1, audioamp_i2c_address);
    
    audio_start_system_clock();
    
    // Set DAC processing block (PRB_P25 supports beep generator)
    tlv320_set_dac_processing_block(&audioamp, 25);

    // Set PLL clock input to MCLK (12MHz from RP2350)
    tlv320_set_pll_clock_input(&audioamp, TLV320DAC3100_PLL_CLKIN_BCLK);
    sleep_ms(10);
    // Set codec clock input to PLL (12MHz from RP2350)
    tlv320_set_codec_clock_input(&audioamp, TLV320DAC3100_CODEC_CLKIN_PLL);

    if(AUDIO_SAMPLE_RATE != 8*KHZ){
        printf("Sample rates other than 8KHZ are currently not supported by the audio codec. Defaulting to 8KHZ. Some things may not work as expected");
    }
    // Set clock dividers for 8kHz sample rate
    tlv320_set_ndac(&audioamp, true, 12);
    tlv320_set_mdac(&audioamp, true, 8);
    tlv320_set_dosr(&audioamp, 128);
    //PLL_CLK = CLK_IN * (R*(J+(D/10000)))/P
    tlv320_set_pll_values(&audioamp, 1, 16, 12, 0); // Set PLL to 98.304MHz
    tlv320_power_pll(&audioamp, true);

    // tlv320_set_codec_clock_input(&audioamp, TLV320DAC3100_CODEC_CLKIN_BCLK);
    // if(AUDIO_SAMPLE_RATE != 8*KHZ){
    //      printf("Sample rates other than 8KHZ are currently not supported by the audio codec. Defaulting to 8KHZ. Some things may not work as expected");
    // }
    // tlv320_set_ndac(&audioamp, true, 1);
    // tlv320_set_mdac(&audioamp, true, 1);
    // tlv320_set_dosr(&audioamp, 64);

    // Configure codec interface - I2S, 32-bit, codec is slave
    // IMPORTANT: Must be set BEFORE configuring BCLK dividers per datasheet power-up sequence
    tlv320_set_codec_interface(&audioamp, TLV320DAC3100_FORMAT_I2S, TLV320DAC3100_DATA_LEN_32, false, false);

    // Configure BCLK generation:
    // tlv320_set_bclk_n(&audioamp, true, 1);  // Enable BCLK divider with N=1
    
    // Configure BCLK output behavior (Page 1, Reg 0x1D):
    // - invert_bclk=false: normal polarity
    // - active_when_powered_down=true: keep BCLK running 
    // - source=DAC_MOD_CLK: derive from DAC_MOD_CLK (2MHz) for cleaner division
    // tlv320_set_bclk_config(&audioamp, false, false, TLV320DAC3100_BCLK_SRC_DAC_MOD_CLK);

    // Configure timer/delay clock divider (Page 3, Reg 16)
    // For 12MHz MCLK, divider of 12 gives ~1MHz timer clock for beep generator
    // Per TLV320DAC3100 datasheet Section 7.4.3
    tlv320_config_delay_divider(&audioamp, true, 12);

    // Route DAC to speaker mixer (not headphone)
    tlv320_configure_analog_inputs(&audioamp, TLV320_DAC_ROUTE_MIXER, TLV320_DAC_ROUTE_MIXER, false, false, false, false);
    
    sleep_ms(50);

    // Power on DAC
    // Enable DAC data path - both channels on, normal routing
    tlv320_set_dac_data_path(&audioamp, true, true, TLV320_DAC_PATH_NORMAL, TLV320_DAC_PATH_NORMAL, TLV320_VOLUME_STEP_2SAMPLE);

    // Unmute DAC channels
    tlv320_set_dac_volume_control(&audioamp, false, false, TLV320_VOL_RIGHT_TO_LEFT);
    
    // Set DAC digital volume (0dB for left and right channels)
    tlv320_set_channel_volume(&audioamp, false, 0);  // Left channel 0dB
    tlv320_set_channel_volume(&audioamp, true, 0);   // Right channel 0dB

    // Enable speaker amplifier
    tlv320_enable_speaker(&audioamp, true);
    
    // Configure speaker driver - gain and unmute
    // TLV320_SPK_GAIN_6DB = +6dB gain stage
    tlv320_configure_spk_pga(&audioamp, TLV320_SPK_GAIN_6DB, true);
    
    // Set speaker analog volume - route enabled, 0dB gain
    tlv320_set_spk_volume(&audioamp, true, 0);

    tlv320_set_bclk_offset(&audioamp, 1);
    
    // Wait for output drivers to stabilize
    sleep_ms(50);

    audio_amp_set_volume(current_volume);

}

static void poll_input(){
    u32 columns = array_size(button_power_pin);
    u32 rows = array_size(button_sense_pin);
    ifor(c, columns){
        gpio_put(button_power_pin[c], 1);
        sleep_us(1);
        ifor(r, rows){
            bool8 pin = gpio_get(button_sense_pin[r]);
            button_debounce_buffer[(button_debounce_buffer_index * columns * rows) + r * columns + c] = pin;
        }
        gpio_put(button_power_pin[c], 0);
        sleep_us(1);
    }
    /* evaluate if a button is released or not */
    bool8 buttons[BUTTONS_SIZE] = {0};
    ifor(b, BUTTONS_SIZE) {
        ifor(d, BUTTON_DEBOUNCE_BUFFER_SIZE){
            u32 buffer_offset = circle_buffer_index_at(2, button_debounce_buffer_index - d);
            /* quick and durty but if the last 4 loops and the button is off we can assume its actually off and not ringing.*/
            buttons[b] = button_debounce_buffer[buffer_offset * BUTTONS_SIZE + b]; 
        }
    }
    button_debounce_buffer_index = circle_buffer_index_at(2, button_debounce_buffer_index+1);
    
    ifor(b, BUTTONS_SIZE){
        u32 key = key_map[b];
        if(key == key_unknown) continue;

        if(buttons[b]){
            if(key_states[key] == key_state_repeat) continue;
            else if(key_states[key] == key_state_pressed){
                /* time32 time_sense_last_change = time_us_32() - last_key_changed_timestamp[key];
                printf("button %" PRIu32" time %" PRIu32" \n", b, last_key_changed_timestamp[key]);
                if(time_sense_last_change > millis_until_repeating){
                    key_states[key] = key_state_repeat;
                } */
            }else {
                key_states[key] = key_state_pressed;
                last_key_changed_timestamp[key] = time_us_32();
            }
        }else{
            key_states[key] = key_state_released;
            last_key_changed_timestamp[key] = time_us_32();
        }
    }

    /* process encoder */
    selected_channel += encoder_get_difference();

    /* read volume pot */
    current_volume = get_volume_pot();
    if(abs(current_volume-last_volume)>2){
        printf("%"PRIu8":volume\n", current_volume);
        last_volume = current_volume;
        audio_amp_set_volume(current_volume);
    }
}

#define HTUI_EXTERNAL_EXPORT static
#define HTUI_IMPLEMENTATION
#include "ui_bitmaps.h"

HTUI_EXTERNAL_EXPORT bool8 htui_external_draw(htui_display_draw_command * command, void * p_user_state){
    /*TODO: make this clean just the area not the whole screen.*/
    if(command->mode == htui_draw_command_mode_deep_clean){
        ssd1681_write_buffer_and_update_if_ready(SSD1681_UPDATE_CLEAN_FULL);
    }else if(command->mode == htui_draw_command_mode_full_deep_clean){
        ssd1681_write_buffer_and_update_if_ready(SSD1681_UPDATE_CLEAN_FULL_AGGRESSIVE);
    }
    
    // printf("COmmand x %d y %d w %d h %d \n", command->x, command->y, command->width, command->height);
    ifor(px, command->width){
        ifor(py, command->height){
            ssd1681_write_point(SSD1681_COLOR_BLACK, command->x + px, command->y + py, !test_bit_at_index(command->buffer, px + py * command->width));
        }
    }


    if(command->mode == htui_draw_command_mode_finished){
        ssd1681_write_buffer_and_update_if_ready(SSD1681_UPDATE_FAST_PARTIAL);
        // ssd1681_write_buffer(SSD1681_COLOR_BLACK);
        // ssd1681_write_cmd(CMD_DISPLAY_UPDATE_CONTROL);
        // ssd1681_write_data(0x00);
        // ssd1681_write_data(0x80);
        // ssd1681_write_cmd(CMD_DISPLAY_UPDATE_CONTROL_2);
        // ssd1681_write_data(0xFE);
        // ssd1681_write_cmd(CMD_MASTER_ACTIVATION);
    }
    return true;
}

HTUI_EXTERNAL_EXPORT bool8 htui_external_list_fonts(fat_str ** out_fonts, u32 * out_fonts_size, void * user_state){
    static fat_str internal_font = {
        .size = array_size("internal_font"),
        .data = "internal_font",
    };

    *out_fonts = &internal_font;
    *out_fonts_size = 1;
    return 1;
}

HTUI_EXTERNAL_EXPORT bool8 htui_external_list_code_points(fat_str const font, u8 ** out_code_points, u32 * out_code_points_size, void * user_state){
    if(memcmp(font.data, "internal_font", font.size)) return 0;

    *out_code_points = glyph_code_points;
    *out_code_points_size = 26;
    return 1;
}

HTUI_EXTERNAL_EXPORT bool8 htui_external_get_glyph(fat_str const font, u32 code_point_index, htui_glyph const ** out_glyph, void * user_state){
    if(memcmp(font.data, "internal_font", font.size)) return 0;

    if(code_point_index > 'z' || code_point_index < 'a') return 0;

    *out_glyph = glyphs + (code_point_index-'a');

    return 1;
}

#endif /*!defined(MOCK_RADIO)*/

HT15_EXPORT bool8 ht15_initalize(void){

#if !defined(MOCK_RADIO)
    gpio_init(pin_led_status);
    gpio_set_dir(pin_led_status, GPIO_OUT);
    gpio_put(pin_led_status, 1);

    if(!stdio_init_all()){

        ifor(i, blink_code_stdio_failed_to_initalize){
            sleep_ms(333);
            gpio_put(pin_led_status, 0);
            sleep_ms(333);
            gpio_put(pin_led_status, 1);
            /* reset_usb_boot(0,0); */
        }
    }

    ifor(i, 6){
            gpio_init(button_sense_pin[i]);
            gpio_set_dir(button_sense_pin[i], GPIO_IN);
            gpio_pull_down(button_sense_pin[i]);
    }
    ifor(i, 4){
            gpio_init(button_power_pin[i]);
            gpio_set_dir(button_power_pin[i], GPIO_OUT);
            gpio_pull_down(button_power_pin[i]);
    }

    set_sys_clock_khz(PROCESSOR_CLOCK_MHZ * KHZ, false);
    clock_configure(clk_peri, 0, CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS, PROCESSOR_CLOCK_MHZ * MHZ, PROCESSOR_CLOCK_MHZ * MHZ); // set periphreal clock to same as system clock to not be limited at USB clock max (48MHz/2)

    sleep_ms(1000);

    printf("System clock: %d MHz\n", clock_get_hz(clk_sys) / MHZ);

    /* initalize i2c */
    i2c1_hw->enable = 0;
    sleep_us(10);
    gpio_set_function(pin_i2c1_sda, GPIO_FUNC_I2C);
    gpio_set_function(pin_i2c1_scl, GPIO_FUNC_I2C);
    i2c_init(i2c1, 100 * KHZ);

    // gpio_init(pin_display_cs);
    // gpio_set_dir(pin_display_cs, GPIO_OUT);
    // gpio_init(pin_sd_cs);
    // gpio_set_dir(pin_sd_cs, GPIO_OUT);
    // gpio_init(pin_flash_cs);
    // gpio_set_dir(pin_flash_cs, GPIO_OUT);
    // spi1_force_select(spi1_device_none); /* deselect all devices on spi1 bus */

    // spi_init(spi1, 8 * MHZ); //8MHz is 20 MHz measured for some reason
    // spi_set_format(spi1, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    // gpio_set_function(pin_spi1_sdi, GPIO_FUNC_SPI);
    // gpio_set_function(pin_spi1_sdo, GPIO_FUNC_SPI);
    // gpio_set_function(pin_spi1_clk, GPIO_FUNC_SPI);
    
    /*TODO: all display stuff goes on another thread*/
    printf("initalize display\n");
    display_init();
    printf("initalize encoder\n");
    encoder_init();
    printf("initalize rf\n");
    rf_init();
    printf("initalize audio codec\n");
    audio_codec_init();
    printf("initalize mic\n");
    mic_init(AUDIO_MIC_OVERSAMPLING_RATIO*AUDIO_SAMPLE_RATE);

    // printf("initalize sd\n");
    /* SD card needs to init before anything else because it starts in SD mode and needs to be switched to SPI mode before the display can be used, which shares the same SPI bus */
    // sd_init();


    adc_init();
    adc_gpio_init(pin_v_bat);
    adc_gpio_init(pin_pot_volume);
#else

#endif
    return 1;
}

/* map 0-99 to -61 to 0 dB */
static f32 calculate_volume(u8 volume){ return ((f32)volume * 0.619191f) - 61.0f; }

static void audio_beep(u16 frequency_hz, u16 duration_ms, i8 volume_db){
#if !defined(MOCK_RADIO)
    // Sample rate is 48kHz based on our clock divider configuration
    const u32 sample_rate = AUDIO_SAMPLE_RATE;
    
    // Frequency must be less than sample_rate/4 per datasheet
    if (frequency_hz >= sample_rate / 4) {
        frequency_hz = sample_rate / 4 - 1;
    }
    
    // Configure the beep tone (sin/cos coefficients and length)
    // This calculates the proper phase increment for the DDS
    if (!tlv320_configure_beep_tone(&audioamp, (f32)frequency_hz, duration_ms, sample_rate)) {
        printf("Failed to configure beep tone\n");
        return;
    }
    
    // Set beep volume (0dB max, -61dB min)
    // Using same volume for both channels
    tlv320_set_beep_volume(&audioamp, volume_db, volume_db);
    
    // Start the beep
    tlv320_enable_beep(&audioamp, true);
    
    // printf("Beep: %dHz, %dms, %ddB\n", frequency_hz, duration_ms, volume_db);
#endif
}


u64 realtime_loop_rate_hz = AUDIO_SAMPLE_RATE;
void ht15_run_realtime_core(void){
    u16 tone_hz = 1000;
    bool8 transmit_tone = false;

    // f32 mic_gain_multiplier = .0000001;
    u16 mic_highpass_cutoff_hz = 500;
    u16 mic_lowpass_cutoff_hz = 4000;

    f32 mic_gain_db = 40.0f; // 93 is a good gain for a quiet room talking into the mic, however lets go lower for headroom and let the autogain take care of it

    f32 mic_highpass_tracker = (f32)(ht15_i2s_mic_get_one_sample_raw_blocking() >> 8);
    f32 mic_lowpass_antialias_tracker = mic_highpass_tracker;
    f32 mic_autogain_tracker_slow = 1.0f;
    f32 mic_autogain_tracker_fast = 1.0f;
    i32 mic_oversample_buffer[AUDIO_MIC_OVERSAMPLING_RATIO];
    i32 tx_audio_sample = 0;

    u16 speaker_highpass_cutoff_hz = 10;
    f32 speaker_highpass_tracker = 0.0f;
    f32 speaker_lowpass_tracker = speaker_highpass_tracker;
    i32 speaker_oversample_buffer[AUDIO_CODEC_OVERSAMPLING_RATIO];
    i32 rx_audio_sample = 0;

    u64 realtime_cycle_count = 0;
    u64 loop_time_target_us = 1000000/realtime_loop_rate_hz;
    u16 slowest_loop_time_us = 0;
    float rolling_average_loop_time_us = 0.0f;
    u64 current_time_us = time_us_64();
    u64 loop_target_end_us = current_time_us + loop_time_target_us;

    while(true){
        
        // Oversample the mic and convert the 32 bit value to a 24 bit (hardware samples at 24 but packs into 32)
        for(i8 i=0; i<AUDIO_MIC_OVERSAMPLING_RATIO; i++){
            mic_oversample_buffer[i] = audio_toolkit_lowpass_filter_i32(&mic_lowpass_antialias_tracker, (i32)(ht15_i2s_mic_get_one_sample_raw_blocking() >> 8), mic_lowpass_cutoff_hz, realtime_loop_rate_hz*AUDIO_MIC_OVERSAMPLING_RATIO);
        }
        tx_audio_sample = audio_toolkit_oversample_i32(mic_oversample_buffer, AUDIO_MIC_OVERSAMPLING_RATIO); // decimate (well, average) to finish the oversampling

        tx_audio_sample = audio_toolkit_highpass_filter_i32(&mic_highpass_tracker, tx_audio_sample, mic_highpass_cutoff_hz, realtime_loop_rate_hz); // remove low end to block DC and to not interfere with the CTCSS tone
        tx_audio_sample = audio_toolkit_gain_i32(tx_audio_sample, mic_gain_db); //apply mic gain

        //tx
        if(rfmodule_state.is_keyed){
            // printf("RF Module Keyed");
            if(mutex_try_enter(&rfmodule_mutex, 0)){
                //do autogain on signal path so far; should be only mic data
                tx_audio_sample = audio_toolkit_autogain_i32(&mic_autogain_tracker_slow, tx_audio_sample, -27.0f, -25.0f, 25.0f, .1f, .2f, realtime_loop_rate_hz); // autogain
                tx_audio_sample = audio_toolkit_autogain_i32(&mic_autogain_tracker_fast, tx_audio_sample, -30.0f, -12.0f, 0.0f, .001f, .05f, realtime_loop_rate_hz); // limiter targeting -30dBFS

                tx_audio_sample = audio_toolkit_gain_i32(tx_audio_sample, 24.0f); //gain to -6dBFS (after limiter targets -30dBFS)

                //add tone to audio to transmit
                if(transmit_tone){
                    tx_audio_sample += audio_toolkit_generate_tone_i32(tone_hz, loop_target_end_us);
                }
                rfmodule_2m70cm_set_tx_data_raw(&rfmodule_state, tx_audio_sample/33554432); // transmit (and shrink sample_to_transmit from 32 to 7 bits)

                mutex_exit(&rfmodule_mutex);
            }
        } else{ //rx
            // printf("rx");
            for(i8 i=0; i<AUDIO_CODEC_OVERSAMPLING_RATIO; i++){
                rx_audio_sample = audio_toolkit_generate_tone_i32(1000, time_us_64());

                // rx_audio_sample = audio_toolkit_highpass_filter_i32(&speaker_highpass_tracker, rx_audio_sample, speaker_highpass_cutoff_hz, realtime_loop_rate_hz); // remove low end to block DC and hopefully remove any clicks
                ht15_i2s_codec_io_put_one_sample_raw_blocking(rx_audio_sample); //L
                ht15_i2s_codec_io_put_one_sample_raw_blocking(rx_audio_sample); //R

            }


        }


        realtime_cycle_count += 1;
        //control loop timing, track slowest loop time and rolling average for performance monitoring
        current_time_us = time_us_64();
        sleep_us(loop_target_end_us>current_time_us ? loop_target_end_us-current_time_us : 0);
        // if (loop_start_us > slowest_loop_time_us){
        //     slowest_loop_time_us = loop_start_us;
        // }
        // if(realtime_cycle_count%8000 == 0){
        //     printf("realtime avg loop time microseconds: %f\n", rolling_average_loop_time_us);
        // } 
        // rolling_average_loop_time_us = (rolling_average_loop_time_us  *0.999f) + ((float)loop_start_us * 0.001f);
        loop_target_end_us += loop_time_target_us;
    }

}

HT15_EXPORT bool8 ht15_run(void){
    u32 cycle = 0;
    bool8 should_clean_display = 1;

    poll_input(); // make sure curent_volume is set before launching the realtime core
    mutex_init(&rfmodule_mutex);
    multicore_launch_core1(ht15_run_realtime_core);

    u64 loop_time_target_us = 10000; //target loop time of 10ms
    u16 slowest_loop_time_us = 0;
    float rolling_average_loop_time_us = 0.0f;
    u64 loop_start_us = time_us_64();


    htui_state ui_state;
    u32 settings_button_id = 0;
    bool8 in_settings = false;
    printf("Initalize\n");
    htui_initalize(200, 200, &ui_state, NULL);

    while(1){
        poll_input();

        bool8 any_key_pressed = 0;
        bool8 any_key_held = 0;
        ifor(key, key_max_enum){
            any_key_held |= key_states[key] == key_state_repeat;
            any_key_pressed |= key_states[key] == key_state_pressed;
            if(key == key_enter && key_states[key] == key_state_pressed){
                should_clean_display = 1;
            }
        }

        /* if(any_key_held){ if((cycle & 0b1111) == 0b1111) led_status_value = !led_status_value; }
        else */ if(any_key_pressed && !key_states[key_ptt]){
            led_status_value = 0;
            audio_beep(4000, 50, calculate_volume(current_volume));
        } else if((cycle & 0b111111) == 0b111111) led_status_value = !led_status_value;

        if(!(cycle & 0b111111)){
            // printf("trying to display settings\n");

            //holdover from the old UI, New UI does not refresh the screen before trying to draw to it. Also added my voltage and volume back until we can integrate it to the new UI
            if(should_clean_display){
               ssd1681_wait_busy();
               should_clean_display = ssd1681_write_buffer_and_update_if_ready(SSD1681_UPDATE_FAST_FULL)? 0 : 1;
            }
            char voltage_string[6];
            sprintf(voltage_string, "%.2fV", get_battery_voltage());
            char channel_string[10];
            snprintf(channel_string, 10, "CH %d", selected_channel);


            ssd1681_draw_string(SSD1681_COLOR_BLACK, 130, 10, voltage_string, 5, 1, SSD1681_FONT_8);
            char volume_string[10];
            u16 written = snprintf(volume_string, 3, "%"PRIu8"<|", current_volume);
            ssd1681_draw_string(SSD1681_COLOR_BLACK, 180, 10, volume_string, written, 1, SSD1681_FONT_8);
            htui_area_info main_area_info = {
                .type = htui_area_type_vertical,
            };

            htui_begin_area(&ui_state, &main_area_info);
                if(htui_button(&ui_state, &settings_button_id, "settings") == htui_component_state_pressed){
                    in_settings = true;
                }
            htui_end(&ui_state);
            if(!htui_end_and_render(&ui_state)){
                printf("end and render failed.\n");
            }
        }

        rf_transmit(439*MHZ, true, 25, key_states[key_ptt] == key_state_pressed);
        gpio_put(pin_led_status, led_status_value);
        cycle += 1;


        //control loop timing, track slowest loop time and rolling average for performance monitoring
        loop_start_us = time_us_64()-loop_start_us;
        sleep_us(loop_time_target_us>loop_start_us ? loop_time_target_us-loop_start_us : 0);
        if (loop_start_us > slowest_loop_time_us){
            slowest_loop_time_us = loop_start_us;
        }
        rolling_average_loop_time_us = (rolling_average_loop_time_us  *0.999f) + ((float)loop_start_us * 0.001f);
        loop_start_us = time_us_64();
    }
    return 1;
}

#endif /* HT15_IMPLEMENTATION */
#endif /* HT15_H */
