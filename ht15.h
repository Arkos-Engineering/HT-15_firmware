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
#if defined(HT15_IMPLEMENTATION)

#include <stdlib.h>
#include <stdio.h>
#include <pico.h>
#include <pico/stdlib.h>
#include "pico/multicore.h"
#include "hardware/adc.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include <pico/bootrom.h>

#define PICO_SSD1681_IMPLEMENTATION
#include"pico_ssd1681.h"
#define PICO_TLV320DAC3100_IMPLEMENTATION
#include "pico_tlv320dac3100.h"

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

u32 encoder_debounce_state = 0;
i8 encoder_state = 0;

u8 last_volume = 0;
u8 current_volume = 0;

tlv320dac3100_t audioamp;
static f32 calculate_volume(u8 volume);
static void audio_amp_set_volume(u8 volume);
static void audio_stop_system_clock();
static void audio_start_system_clock();
static void audio_amp_reset_hard();

/* alignment represents the size of the buffer at 1 << alignment kindof like int2 int3 in fasm */
static inline u32 circle_buffer_index_at(u8 alignment, i32 index){
        return index & ((1<<alignment)-1);
}

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

/* TODO@Zea as of December 20 2025 add knobs */
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
    encoder_debounce_state=((encoder_debounce_state<<1) | gpio_get(pin_encoder_a)) & 0b111;/* store the last three states of the encoder A pin */
    if(encoder_debounce_state==0b100){
        if(gpio_get(pin_encoder_b)){
            encoder_state = 1;
            printf("Channel +\n");
        } else{
            encoder_state = -1;
            printf("Channel -\n");
        }
        encoder_debounce_state=0;
    }else{
        encoder_state = 0;
    }

    /* read volume pot */
    current_volume = get_volume_pot();
    if(abs(current_volume-last_volume)>2){
        printf("%"PRIu8":volume\n", current_volume);
        last_volume = current_volume;
        audio_amp_set_volume(current_volume);
    }
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
    display_config.spi_baudrate = 10 * MHZ;
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
    ssd1681_write_buffer(SSD1681_COLOR_BLACK);
    ssd1681_update(SSD1681_UPDATE_CLEAN_FULL);
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

/* map 0-99 to -61 to 0 dB */
static f32 calculate_volume(u8 volume){ return ((f32)volume * 0.619191f) - 61.0f; }

/* set volume on audio amp (0-99) */
static void audio_amp_set_volume(u8 volume){
    f32 vol = calculate_volume(volume);
    tlv320_set_channel_volume(&audioamp, 0, vol);  /* Left channel */
    tlv320_set_channel_volume(&audioamp, 1, vol);   /* Right channel */
}

static void audio_beep(uint16_t frequency_hz, uint16_t duration_ms, int8_t volume_db){
    // Sample rate is 48kHz based on our clock divider configuration
    const uint32_t sample_rate = 48000;
    
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
}

static void audio_init(){
    audio_amp_reset_hard();

    // Initialize TLV320DAC3100 audio amplifier over I2C and init audio_config->audioamp
    tlv320_init(&audioamp, i2c1, audioamp_i2c_address);
    
    audio_start_system_clock();
    
    // Set DAC processing block (PRB_P25 supports beep generator)
    tlv320_set_dac_processing_block(&audioamp, 25);

    // Set PLL clock input to MCLK (12MHz from RP2350)
    tlv320_set_pll_clock_input(&audioamp, TLV320DAC3100_PLL_CLKIN_MCLK);
    sleep_ms(10);
    // Set codec clock input to PLL (12MHz from RP2350)
    tlv320_set_codec_clock_input(&audioamp, TLV320DAC3100_CODEC_CLKIN_PLL);

    // Set clock dividers for 16kHz sample rate
    // MCLK=12MHz, NDAC=3, MDAC=2, DOSR=125
    // DAC_CLK = MCLK / NDAC = 12MHz / 3 = 4MHz
    // DAC_MOD_CLK = DAC_CLK / MDAC = 4MHz / 2 = 2MHz  
    // DAC_FS = DAC_MOD_CLK / DOSR = 2MHz / 125 = 16kHz
    tlv320_set_ndac(&audioamp, true, 2);
    tlv320_set_mdac(&audioamp, true, 8);
    tlv320_set_dosr(&audioamp, 128);
    tlv320_set_pll_values(&audioamp, 1, 1, 8, 1920); // Set PLL to multiply MCLK by 4 (12MHz * 4 = 48MHz PLL clock)

    tlv320_power_pll(&audioamp, true);

    // Configure codec interface - I2S, 16-bit, codec is master (bclk_out=true, wclk_out=true)
    // IMPORTANT: Must be set BEFORE configuring BCLK dividers per datasheet power-up sequence
    tlv320_set_codec_interface(&audioamp, TLV320DAC3100_FORMAT_I2S, TLV320DAC3100_DATA_LEN_16, true, true);

    // Configure BCLK generation:
    tlv320_set_bclk_n(&audioamp, true, 1);  // Enable BCLK divider with N=1
    
    // Configure BCLK output behavior (Page 1, Reg 0x1D):
    // - invert_bclk=false: normal polarity
    // - active_when_powered_down=true: keep BCLK running 
    // - source=DAC_MOD_CLK: derive from DAC_MOD_CLK (2MHz) for cleaner division
    tlv320_set_bclk_config(&audioamp, false, true, TLV320DAC3100_BCLK_SRC_DAC_MOD_CLK);

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
    tlv320_set_dac_volume_control(&audioamp, true, true, TLV320_VOL_RIGHT_TO_LEFT);
    
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
    
    // Wait for output drivers to stabilize
    sleep_ms(50);
}

HT15_EXPORT bool8 ht15_initalize(void){
    // set_sys_clock_khz(120000, true);
    // set_sys

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
    gpio_init(pin_encoder_a);
    gpio_set_dir(pin_encoder_a, GPIO_IN);
    gpio_init(pin_encoder_b);
    gpio_set_dir(pin_encoder_b, GPIO_IN);


    sleep_ms(1000);

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
    printf("initalize audio\n");
    audio_init();

    // printf("initalize sd\n");
    /* SD card needs to init before anything else because it starts in SD mode and needs to be switched to SPI mode before the display can be used, which shares the same SPI bus */
    // sd_init();


    adc_init();
    adc_gpio_init(pin_v_bat);
    adc_gpio_init(pin_pot_volume);

    return 1;
}

HT15_EXPORT bool8 ht15_run(void){
    u32 cycle = 0;
    bool8 should_clean_display = 1;
    u8 selected_channel = 1;
    while(1){
        poll_input();

        bool8 any_key_pressed = 0;
        bool8 any_key_held = 0;
        ifor(key, key_max_enum){
            any_key_held |= key_states[key] == key_state_repeat;
            any_key_pressed |= key_states[key] == key_state_pressed;
            if(key == key_ptt && key_states[key] == key_state_pressed){
                i2c1_scan_bus();
            }
            if(key == key_enter && key_states[key] == key_state_pressed){
                should_clean_display = 1;
            }
        } 

        /* if(any_key_held){ if((cycle & 0b1111) == 0b1111) led_status_value = !led_status_value; }
        else */ if(any_key_pressed){
            led_status_value = 0;
            audio_beep(4000, 20, calculate_volume(current_volume));
        } else if((cycle & 0b11111) == 0b11111) led_status_value = !led_status_value;

        encoder_state=((encoder_state<<1) | gpio_get(pin_encoder_a)) & 0b111;//store the last three states of the encoder A pin
        if(encoder_state==0b10){
            if(gpio_get(pin_encoder_b)){
                printf("Channel +\n");
                selected_channel++;
            } else{
                printf("Channel -\n");
                selected_channel--;
            }
            // encoder_state=0;
        }


        if(!(cycle & 31)){
        // if(cycle % 100 == 0){
            char voltage_string[6];
            sprintf(voltage_string, "%.2fV", get_battery_voltage());
            char channel_string[10];
            snprintf(channel_string, 10, "CH %d", selected_channel);
            printf("%s\n", channel_string);
            ssd1681_draw_string(SSD1681_COLOR_BLACK, 40, 50, "HT-15", 5, 1, SSD1681_FONT_24);
            ssd1681_draw_string(SSD1681_COLOR_BLACK, 40, 75, channel_string, 6, 1, SSD1681_FONT_12);
            ssd1681_draw_string(SSD1681_COLOR_BLACK, 10, 10, voltage_string, 5, 1, SSD1681_FONT_8);

            char volume_string[10];
            int writen = snprintf(volume_string, 3, "%"PRIu8"<|", current_volume);
            ssd1681_draw_string(SSD1681_COLOR_BLACK, 180, 10, volume_string, writen, 1, SSD1681_FONT_8);

            if(should_clean_display){
                should_clean_display = ssd1681_write_buffer_and_update_if_ready(SSD1681_UPDATE_FAST_FULL)? 0 : 1;
            } else {
                ssd1681_write_buffer_and_update_if_ready(SSD1681_UPDATE_FAST_PARTIAL);
            }
        }

        gpio_put(pin_led_status, led_status_value);
        cycle += 1;
        sleep_ms(24);
    }
    return 1;
}

#endif /* HT15_IMPLEMENTATION */
#endif /* HT15_H */
