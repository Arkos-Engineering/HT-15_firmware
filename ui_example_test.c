/* cc -std=c99 -o uitest ui_example_test.c -lSDL3 -g; ./uitest */

#include <SDL3/SDL_pixels.h>
#include <math.h>
#include "definitions.h"
#include <GLFW/glfw3.h>
#include <SDL3/SDL.h>
#include <SDL3/SDL_error.h>
#include <SDL3/SDL_events.h>
#include <SDL3/SDL_init.h>
#include <SDL3/SDL_oldnames.h>
#include <SDL3/SDL_rect.h>
#include <SDL3/SDL_render.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <wctype.h>

#define WIDTH 200
#define HEIGHT 200

#define HTUI_EXPORT_STATIC
#define HTUI_IMPLEMENTATION
#include "ui.h"

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
    htui_state state;
    display_mode mode;
} display_state;

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
void update_fake_display(display_state * state, htui_display_draw_command * command){ 
    SDL_Rect rect = {.x = command->x, .y = command->y, .w = command->width, .h = command->height };
    SDL_Color * pixels;
    int pitch = 0;
    SDL_LockTexture(state->fake_display, &rect, (void **)&pixels, &pitch);

    ifor(y, command->height){
        SDL_Color * row = pixels + y * (pitch/sizeof(SDL_Color));
        ifor(x, command->width){
            if(command->buffer[y * HEIGHT + x]) row[x] = COLOR_WHITE;
            else row[x] = COLOR_BLACK;
        }
    }
    SDL_UnlockTexture(state->fake_display);
}

void show_fake_display(display_state * state){
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

    SDL_RenderClear(state->renderer);
    SDL_RenderTexture(state->renderer, state->fake_display, &src, &dest);
    SDL_RenderPresent(state->renderer);
}

void clear_fake_display(display_state * state, bool8 bw){
    ifor(i, WIDTH * HEIGHT) fake_display_pixels[i] = bw;
    htui_display_draw_command command = {
        .x = 0,
        .y = 0,
        .width = WIDTH,
        .height = HEIGHT,
        .buffer = fake_display_pixels,
    };
    update_fake_display(state, &command);
    show_fake_display(state);
}

HTUI_DRAW_EXPORT bool8 htui_external_draw(htui_display_draw_command * command, void * p_user_data){
    if(!command) return false;
    if(!p_user_data) return false;
    display_state * state = (display_state *)p_user_data;

    if(command->mode == htui_full_deep_clean){
        SDL_FRect r = { 0, 0, WIDTH, HEIGHT};

        clear_fake_display(state, 1);
        micronap();

        clear_fake_display(state, 0);
        micronap();

        clear_fake_display(state, 1);
        micronap();

        clear_fake_display(state, 0);
        micronap();

        clear_fake_display(state, 1);
        micronap();
    }

    update_fake_display(state, command);

    if(command->mode == htui_finished){
        show_fake_display(state);
    }

    return 1;
}



int main(){
    display_state state;
    memset(&state, 0, sizeof(state));
    u32 counter = 0;

    if(!SDL_Init(SDL_INIT_VIDEO)) {
        puts(SDL_GetError());
        abort();
    }

    SDL_CreateWindowAndRenderer("ht15_ui_test", 1000, 1000, 0, &state.window, &state.renderer);
    SDL_SetRenderDrawColor(state.renderer, 0, 0, 0, 0);
    SDL_RenderClear(state.renderer);

    state.fake_display = SDL_CreateTexture(state.renderer, SDL_PIXELFORMAT_ABGR8888, SDL_TEXTUREACCESS_STREAMING, WIDTH, HEIGHT);
    if(!state.fake_display) abort();
    clear_fake_display(&state, 1);

    htui_initalize(&state.state);

    while(1){
        SDL_PollEvent(&state.event);
        if(state.event.type == SDL_EVENT_QUIT){
            exit(EXIT_SUCCESS);
        }

        HTUI_AREA(&state.state, htui_size_small, 0, 1, 1 ){
            HTUI_AREA(&state.state, ){

            }
        }
        /* overall screen layout*/
        htui_begin_area_vertical(&state.state, 0, 0, 1, 1);
            /*place for batter and volume and other hud things */
            htui_begin_area_horizontal(&state.state, htui_size_small, 0, 1, 0);
                /*should inherit font size from area*/
                c_str text = "Big Chungus";
                fat_str str = {.data = text, .size = strlen(text)-1};
                htui_text(&state.state, str, htui_text_display_marque);

            htui_end_area(&state.state);
        htui_end_area(&state.state);
        htui_end_and_render(&state.state, &state);

        counter += 1;
    }
}


