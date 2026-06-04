/* this is to test logically test the features of the ht15 radio ui from a computer, it will use the main loop of the radio and implement functions it needs in order for the code to run.
 * this would be our source of trueth so that it will be easier to find hardware/software bugs and fuzz the logic of the radio without needing to have the full device.
 * this will also make development of features easier on the side of the ui because we can get a full development loop much smaller than constantly having to flash the radio
 * this will also be used to make sure the logic of the radio can run indefinently because a full cpu will run much faster we can sprint through many loops of the radio to make sure it is very stable
 * */

#include"definitions.h"
#include <math.h>
#include <stdbool.h>

typedef int spi_inst_t;
typedef int i2c_inst_t;
typedef int mutex_t;

void mutex_init(mutex_t *mtx){

}

void multicore_launch_core1(void (*entry)(void)){

}


u64 time_us_64(){

}

u32 time_us_32(){

}

void sleep_us(uint64_t us){

}

#define MOCK_RADIO
#include "pico_rfmodule_2M70CM.h"

rfmodule_error_code_t rfmodule_2m70cm_set_tx_data_raw(rfmodule_2m70cm_state_t *dev, u8 data){
    return RFMODULE_ERROR_SUCCESS;
}

rfmodule_2m70cm_state_t rfmodule_state = {};
mutex_t rfmodule_mutex;

bool mutex_try_enter(mutex_t *mtx, u32 *owner_out){

}

void mutex_exit(mutex_t *mtx){

}

static void poll_input(){

}

bool8 led_status_value = 1;
key_state key_states[key_max_enum] = {key_state_released};

#define HTUI_IMPLEMENTATION
#include "ui_bitmaps.h"

#include "ht15.h"

int main(int argv, char const ** argc){
    ht15_initalize();
    ht15_run();
}

