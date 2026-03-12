
#if !defined(HTUI)
#define HTUI

#include "definitions.h"


#if !defined(_HTUI_EXPORT) && defined(HTUI_EXPORT_STATIC)
#define _HTUI_EXPORT static
#define HTUI_EXTERNAL_EXPORT __attribute((unused)) static 
#else

#if !defined(HTUI_EXTERNAL_EXPORT)
#define HTUI_EXTERNAL_EXPORT extern
#endif

#if !defined (_HTUI_EXPORT)
#define _HTUI_EXPORT
#endif

#endif

typedef enum{
        /* don't update the screen yet */
        htui_pending,
        /* deep clean the area in the command before writing data to the screen */
        htui_deep_clean,
        /* completely wipe the screan before processing any commands. */
        htui_full_deep_clean,
        /* submit and screen updates */
        htui_finished,
} htui_display_draw_command_mode;

typedef struct{
        u8 x, y, width, height;
        /* row major */
        u8 * buffer;
        htui_display_draw_command_mode mode;
} htui_display_draw_command;


/*TODO: these external functions only work in a static linking context, if we are dynamic linking these probably need to be function pointers that are stored in the state. */

/* this is meant to be implementoud by the code using this module. this is called multiple times until the mode is set to finished. */
HTUI_EXTERNAL_EXPORT bool8 htui_external_draw(htui_display_draw_command * command, void * p_user_state);


/* basic font loading stuff, why because I want japanese on this radio. And i guess other languages are cool too. https://www.gnu.org/software/gettext/manual/html_node/index.html*/
typedef struct {
        u8 * data;
        u8 x, y, xoff, yoff;
} htui_glyph;

/* TODO these need to have a callback passed into them because they might unload the data
 * and the burden of holding onto the data should be on the ui library*/
HTUI_EXTERNAL_EXPORT bool8 htui_external_list_font_files(fat_str ** out_fonts, u32 * out_fonts_size, void * user_state);
HTUI_EXTERNAL_EXPORT bool8 htui_external_list_code_points(fat_str const font, u8 ** out_code_points, u32 * out_code_points_size, void * user_state);
/* TODO: add font size stuff.*/
HTUI_EXTERNAL_EXPORT bool8 htui_external_load_glyph(fat_str const font, u32 code_point_index, htui_glyph ** out_glyph, void * user_state);

typedef enum{
        htui_component_state_error,
        htui_component_state_hovered = 1,
        htui_component_state_pressed,
        htui_component_state_success
} htui_component_state;

typedef enum{
        htui_text_display_fade = 1 << 0,
        htui_text_display_wrap = 1 << 1,
        htui_text_display_marque = 1 << 2,
        htui_text_display_bold = 1 << 3,
        htui_text_display_underline = 1 << 4,
} htui_text_display_flags;

typedef enum{
        htui_screen_state_unknown = 0,
        htui_screen_state_dirty,
        htui_screen_state_clean,
} htui_screen_state;

typedef enum {
        htui_size_tiny,
        htui_size_small,
        htui_size_normal,
        htui_size_big,
        htui_size_giant
} htui_size;

#define HTUI_COMPONENT_TYPE\
        X(text)\
        X(button)\
        X(scroller)\
        X(throbber)\
        X(progress_bar)
typedef enum{
#define X(name) htui_component_type_##name,
HTUI_COMPONENT_TYPE
#undef X
} htui_component_type;

typedef struct {
        htui_component_type type;
} htui_component_info;

#define HTUI_AREA_TYPE\
        X(vertical)\
        X(horizontal)\
        X(floating)
typedef enum{
#define X(name) htui_area_type_##name,
HTUI_AREA_TYPE
#undef X
} htui_area_type;

typedef struct{
        htui_area_type type;
} htui_area_info;

typedef enum PACKED{
        htui_command_type_button,
        htui_command_type_text,
        htui_command_type_image,
        htui_command_type_begin_area_vertical,
        htui_command_type_begin_area_horizontal,
        htui_command_type_begin_area_floating,
        htui_command_type_end_area,
} htui_command_type;

/*TODO: this can become a generic that we figure out the structure based on the first enum bytes. */
typedef struct {
        htui_command_type type;
        c_str text;
        u32 text_size;
        u16 component_id;
        htui_size size;
        u8 handle_size;
        union{
                bool8 is_pressable;
                u8 distance_from_left;
                u8 distance_from_top;
                u8 speed;
                bool8 flex;
        };
        bool8 horizontal_lines;
        bool8 bold;
} htui_command;

typedef enum{
        htui_element_box,
        htui_element_character,
        htui_element_line,
        htui_element_dither,
        htui_element_dither_gradient,

        htui_element_box_n,
        htui_element_box_s,
        htui_element_box_e,
        htui_element_box_w,
        htui_element_box_ne,
        htui_element_box_nw,
        htui_element_box_se,
        htui_element_box_sw,
} htui_element_type;

typedef struct{
        htui_element_type type;
        u8 x, y, x2, y2;
} htui_element;

#define MAX_DRAW_COMMANDS 69
typedef struct{
        htui_screen_state screen_state;
        u16 width,height;
        void * internal_buffer;
        void * user_state;
        /*TODO as of Mar 11 2025: hold onto previous state for each component to tell if it needs to be re-rendered for its section of the screen. @Zea Lynn*/

        u16 component_id;

        u16 selected_component_id;
        bool is_pressed;

        htui_command  previous_commands[MAX_DRAW_COMMANDS];
        u32 previous_commands_size;

        htui_command commands[MAX_DRAW_COMMANDS];
        u32 commands_size;

} htui_state;

_HTUI_EXPORT void htui_initalize(htui_state * state);

/*TODO as of March 11 2025: figure out how to only tell the paper display controller what parts of the buffer to re_render. @Zea Lynn*/
_HTUI_EXPORT bool8 htui_end_and_render(htui_state * state, void * user_state);

/* renderes with a border and changes when selected, cannot have other pressables inside it*/
_HTUI_EXPORT bool8 htui_add_component(htui_state * state, u32 * component_id, htui_component_info * info);

/* returns true if pressed */
_HTUI_EXPORT htui_component_state htui_button(htui_state * state, u32 * component_id, c_str title);

_HTUI_EXPORT bool8 htui_begin_area(htui_state * state, htui_area_info * info);
/** ends operation for component or area*/
_HTUI_EXPORT bool8 htui_end(htui_state * state);

#define HTUI_IMPLEMENTATION
#if defined(HTUI_IMPLEMENTATION)

#include <stdbool.h>
#include <string.h>

/*TODO: have this sized based on some external macros also have it be a bitset*/
u8 buffer[200*200] = {0};

_HTUI_EXPORT void htui_initalize(htui_state * state);
_HTUI_EXPORT bool8 htui_begin(htui_state * state){
        state->commands_size = 0;
        return true;
}

_HTUI_EXPORT bool8 htui_end_and_render(htui_state * state, void * user_state){

        htui_display_draw_command command = { 50, 50, 100, 100, buffer, htui_full_deep_clean, };

        u16 x, y;

        fat_str * fonts;
        u32 fonts_size;
        htui_external_list_font_files(&fonts, &fonts_size, user_state);

        if(state->screen_state == htui_screen_state_unknown){
                if(!htui_external_draw(&command, user_state)){ return 0; }
                state->screen_state=htui_screen_state_dirty;
        }
        /* 
        render things in order based on their area
        check component matches previous component.
        look for anything blocking the view of the component.
        if an areas offset was updated because a header disapeared, it means everything in that area neads to rerender on the display.

        things like lines on boxes should not need to display.
        if listing out boxes we should be able to combine the top and bottom parts of each box and avoid rerendering
         */
        ifor(i, state->commands_size){
                /* TODO: render a border and show when button is hovered.*/
                if(state->commands[i].type == htui_command_type_button){
                        u8 x_offset = 0;
                        c_str text = state->commands[i].text;
                        ifor(t, state->commands[i].text_size){
                                /*TODO: check code point exists in current font map. htui_external_list_code_points*/
                                htui_glyph * temp_glyph;
                                htui_external_load_glyph(current_font, text[t], &temp_glyph, state->user_state);
                                x_offset = 
                        }
                }
        }

        
        /* TODO pointer to double buffer swap.*/
        state->previous_commands_size = state->commands_size;
        state->commands_size = 0;

        command.mode = htui_finished;
        if(!htui_external_draw(&command, user_state)){ return 0; }

        return 1;
}

static bool8 _htui_push_back_command(htui_state * state, htui_command * command){
        if(state->commands_size+1 > MAX_DRAW_COMMANDS) return false;
        state->commands[state->commands_size] = *command;
        return true;
}

#define _HTUI_MAX_COMPONENT_ID 42069

/* if the id passed in is outside of the currently allocated ids return false*/
static bool8 _htui_try_get_next_component_id(htui_state * state, u32 * component_id){
        if(state->component_id > _HTUI_MAX_COMPONENT_ID) return false;
        if(*component_id != U32_MAX){
                if(*component_id > state->component_id) return false;
                else return true;
        }
        *component_id = (state->component_id++);
        return true;
}

_HTUI_EXPORT bool8 htui_begin_area(htui_state * state, htui_area_info * info){
        htui_command command = {
                .type = htui_command_type_begin_area_vertical,
        };
        if(!_htui_push_back_command(state, &command)) return false;

        return true;
}

_HTUI_EXPORT htui_component_state htui_button(htui_state * state, u32 * component_id, c_str title){
        if(!_htui_try_get_next_component_id(state, component_id)) return false;

        htui_command button_command = {
                .type = htui_command_type_button,
                .text = title,
                .text_size = strlen(title),
                .component_id = *component_id,
                .size = htui_size_normal,
                .is_pressable = true,
        };

        if(!_htui_push_back_command(state, &button_command)) return htui_component_state_error;
        if(state->selected_component_id == *component_id){
                if(state->is_pressed) return htui_component_state_pressed;
                else return htui_component_state_hovered;
        }
        
        return htui_component_state_success;
}


#endif /* HTUI_IMPLEMENTATION */
#endif /* HTUI */
