/* this is to test logically test the features of the ht15 radio ui from a computer, it will use the main loop of the radio and implement functions it needs in order for the code to run.
 * this would be our source of trueth so that it will be easier to find hardware/software bugs and fuzz the logic of the radio without needing to have the full device.
 * this will also make development of features easier on the side of the ui because we can get a full development loop much smaller than constantly having to flash the radio
 * this will also be used to make sure the logic of the radio can run indefinently because a full cpu will run much faster we can sprint through many loops of the radio to make sure it is very stable
 * */

#include"definitions.h"
#include <stdbool.h>

typedef int spi_inst_t;
typedef int i2c_inst_t;
typedef int mutex_t;

void mutex_init(UNUSED mutex_t *mtx){

}

void multicore_launch_core1(UNUSED void (*entry)(void)){

}

u64 time_us_64(void){

}

u32 time_us_32(void){

}

void sleep_us(UNUSED uint64_t us){

}

#define MOCK_RADIO
#include "pico_rfmodule_2M70CM.h"

rfmodule_error_code_t rfmodule_2m70cm_set_tx_data_raw(UNUSED rfmodule_2m70cm_state_t *dev, UNUSED u8 data){
    return RFMODULE_ERROR_SUCCESS;
}

rfmodule_2m70cm_state_t rfmodule_state = {0};
mutex_t rfmodule_mutex;

bool mutex_try_enter(UNUSED mutex_t *mtx, UNUSED u32 *owner_out){

}

void mutex_exit(UNUSED mutex_t *mtx){

}

static inline void gpio_put(unsigned int UNUSED gpio, bool UNUSED value){

}


static f32 get_battery_voltage(void){
    return 5.0f;
}

bool8 led_status_value = 1;
key_state key_states[key_max_enum] = {key_state_released};

u8 last_volume = 0;
u8 current_volume = 0;
u8 selected_channel = 1;

static void rf_transmit(u64 UNUSED frequency_hz, bool8 UNUSED amp_enable, f32 UNUSED dbm, bool8 state){

}

#define HTUI_IMPLEMENTATION
#include "ui_bitmaps.h"
#define HT15_UTIL_IMPLEMENTATION
#include "util.h"

#include <GLFW/glfw3.h>
#include <SDL3/SDL.h>
#include <SDL3/SDL_error.h>
#include <SDL3/SDL_events.h>
#include <SDL3/SDL_init.h>
#include <SDL3/SDL_oldnames.h>
#include <SDL3/SDL_rect.h>
#include <SDL3/SDL_render.h>
#include <SDL3/SDL_pixels.h>
#include <stdlib.h>
#include <string.h>

#define WIDTH 200
#define HEIGHT 200

u8 fake_display_pixels[WIDTH * HEIGHT] = {0};

/* foreground / background*/
typedef enum{
    dark,
    light,
} display_mode;

typedef struct {
    SDL_Event event;
    SDL_Renderer * renderer;
    SDL_Window * window;
    SDL_Texture * fake_display;
    display_mode mode;
    u8 * glyphs;
    i32 glyphs_x, glyphs_y;
} display_state;
display_state mock_display_state = {0};

/* glibc */
struct timespec
{
#ifdef __USE_TIME64_REDIRECTS
  __time64_t tv_sec;		/* Seconds.  */
#else
  __time_t tv_sec;		/* Seconds.  */
#endif
#if __WORDSIZE == 64 \
  || (defined __SYSCALL_WORDSIZE && __SYSCALL_WORDSIZE == 64) \
  || (__TIMESIZE == 32 && !defined __USE_TIME64_REDIRECTS)
  __syscall_slong_t tv_nsec;	/* Nanoseconds.  */
#else
# if __BYTE_ORDER == __BIG_ENDIAN
  int: 32;           /* Padding.  */
  long int tv_nsec;  /* Nanoseconds.  */
# else
  long int tv_nsec;  /* Nanoseconds.  */
  int: 32;           /* Padding.  */
# endif
#endif
};
extern int nanosleep (const struct timespec *__requested_time, struct timespec *__remaining);

void micronap(void){
    struct timespec spec = {.tv_nsec = 1500 * 100000};
    struct timespec remaining = {0};
    nanosleep(&spec, &remaining);
}

#define BY4(arr, i, val)do{\
    arr[(i)] = val;\
    arr[(i)+1] = val;\
    arr[(i)+2] = val;\
    arr[(i)+3] = val;\
}while(0)

SDL_Rect full_display = {.x = 0, .y = 0, .w = WIDTH, .h = HEIGHT};
SDL_Color COLOR_BLACK = {0};
SDL_Color COLOR_WHITE = {.r = 255,.g = 255,.b = 255,.a = 255};
SDL_Color COLOR_RED = {.r = 255,.g = 0,.b = 255,.a = 255};

/* todo make some of these pixels sticky to mimic paper display. */
void update_fake_display(htui_display_draw_command * command){ 
    SDL_Rect rect = {.x = command->x, .y = command->y, .w = command->width, .h = command->height };
    SDL_Color * pixels;
    int pitch = 0;
    SDL_LockTexture(mock_display_state.fake_display, &rect, (void **)&pixels, &pitch);

    ifor(y, command->height){
        SDL_Color * row = pixels + y * (pitch/sizeof(SDL_Color));
        ifor(x, command->width){
            bool8 white = test_bit_at_index(command->buffer, y * command->width + x);
            if(white) row[x] = COLOR_WHITE;
            else row[x] = COLOR_BLACK;
        }
    }
    SDL_UnlockTexture(mock_display_state.fake_display);
}

void show_fake_display(void){
    SDL_FRect src = {
        .x = 0,
        .y = 0,
        .w = WIDTH,
        .h = HEIGHT
    }, dest = {
        .x = 0,
        .y = 0,
        .w = 1000,
        .h = 1000,
    };

    SDL_RenderClear(mock_display_state.renderer);
    SDL_RenderTexture(mock_display_state.renderer, mock_display_state.fake_display, &src, &dest);
    SDL_RenderPresent(mock_display_state.renderer);
}

void clear_fake_display(bool8 bw){
    ifor(i, WIDTH * HEIGHT) set_bit_at_index(fake_display_pixels, i, bw);
    htui_display_draw_command command = {
        .x = 0,
        .y = 0,
        .width = WIDTH,
        .height = HEIGHT,
        .buffer = fake_display_pixels,
    };
    update_fake_display(&command);
    show_fake_display();
}

HTUI_EXTERNAL_EXPORT bool8 htui_external_draw(htui_display_draw_command * command, void * UNUSED p_user_data){
    if(!command) return false;

    if(command->mode == htui_draw_command_mode_full_deep_clean){
        clear_fake_display(1);
        micronap();

        clear_fake_display(0);
        micronap();

        clear_fake_display(1);
        micronap();

        clear_fake_display(0);
        micronap();

        clear_fake_display(1);
        micronap();
    }

    update_fake_display(command);

    if(command->mode == htui_draw_command_mode_finished){
        show_fake_display();
    }

    return 1;
}

c_str const font = "internal_font";
fat_str fonts[] = {
    {.size = sizeof(font)-1, .data = font}, 
};
u8 * code_points = "abcdefghijklmnopqrstuvwxyz";

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"


HTUI_EXTERNAL_EXPORT bool8 htui_external_list_fonts(fat_str ** out_fonts, u32 * out_fonts_size, UNUSED void * user_state){
    *out_fonts = fonts;
    *out_fonts_size = 1;
    return true;
}

HTUI_EXTERNAL_EXPORT bool8 htui_external_list_code_points(fat_str const font, u8 ** out_code_points, u32 * out_code_points_size, void * UNUSED user_state){
    if(memcmp(font.data, fonts[0].data, fonts[0].size) == 0){
        *out_code_points = code_points;
        *out_code_points_size = 26;
    }else abort();
    return true;
}

HTUI_EXTERNAL_EXPORT bool8 htui_external_get_glyph(fat_str const font, u32 code_point, htui_glyph const ** out_glyph, void * user_state){
    __attribute__((unused)) display_state * state = user_state;

    if(!font.data || font.size == 0) return false;

    if(memcmp(font.data, fonts[0].data, fonts[0].size) == 0){
        u32 code_point_index = (code_point -'a');

        if(code_point_index > 25) return false;

        *out_glyph = glyphs + code_point_index;
        
    }else return false;

    return true;
}

static void poll_input(void){
    SDL_PollEvent(&mock_display_state.event);
    if(mock_display_state.event.type == SDL_EVENT_QUIT){
        exit(EXIT_SUCCESS);
    }
}

#include "ht15.h"

int main(int UNUSED argv, char const ** UNUSED argc){
    if(!SDL_Init(SDL_INIT_VIDEO)) {
        puts(SDL_GetError());
        abort();
    }

    SDL_CreateWindowAndRenderer("mock_ht15", 1000, 1000, 0, &mock_display_state.window, &mock_display_state.renderer);
    SDL_SetRenderDrawColor(mock_display_state.renderer, 0, 0, 0, 0);
    SDL_RenderClear(mock_display_state.renderer);

    mock_display_state.fake_display = SDL_CreateTexture(mock_display_state.renderer, SDL_PIXELFORMAT_ABGR8888, SDL_TEXTUREACCESS_STREAMING, WIDTH, HEIGHT);
    if(!mock_display_state.fake_display) abort();
    clear_fake_display(1);
    ht15_initalize();
    ht15_run();
}

