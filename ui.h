
#include <stdbool.h>
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

HTUI_EXTERNAL_EXPORT bool8 htui_external_list_font_files(fat_str ** out_fonts, u32 * out_fonts_size, void * user_state);
HTUI_EXTERNAL_EXPORT bool8 htui_external_list_code_points(fat_str const font, u8 ** out_code_points, u32 * out_code_points_size, void * user_state);
HTUI_EXTERNAL_EXPORT bool8 htui_external_load_glyph(fat_str const font, u32 code_point_index, htui_glyph ** out_glyph, void * user_state);

typedef enum PACKED{
        htui_begin_pressable_type,
        htui_end_pressable_type,
        htui_text_type,
        htui_image_type,
        htui_begin_area_vertical_type,
        htui_begin_area_horizontal_type,
        htui_begin_area_floating_type,
        htui_end_area_type,
} htui_command_type;

typedef enum{
        htui_component_state_none,
        htui_component_state_selected = 1,
        htui_component_state_pressed,
} htui_component_state;

typedef enum{
        htui_area_vertical,
        htui_area_horizontal,
        htui_area_floating,
} htui_area_type;

typedef enum{
        htui_text_display_fade,
        htui_text_display_wrap,
        htui_text_display_marque,
        htui_text_display_bold,
        htui_text_display_underline,
} htui_text_display_type;

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

typedef struct {
        fat_str text;
        htui_command_type type;
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
} htui_element_type;

typedef struct{
        htui_element_type type;
        u8 x, y, x2, y2;
} htui_element;

#define MAX_DRAW_COMMANDS 200
typedef struct{
        htui_screen_state screen_state;
        u16 width,height;
        void * internal_buffer;
        void * user_state;
        /*TODO as of Mar 11 2025: hold onto previous state for each component to tell if it needs to be re-rendered for its section of the screen. @Zea Lynn*/

        htui_command * commands;
        u32 commands_size;

} htui_state;

_HTUI_EXPORT void htui_initalize(htui_state * state);

/*TODO as of March 11 2025: figure out how to only tell the paper display controller what parts of the buffer to re_render. @Zea Lynn*/
_HTUI_EXPORT bool8 htui_end_and_render(htui_state * state, void * user_state);

/* renderes with a border and changes when selected, cannot have other pressables inside it*/
_HTUI_EXPORT htui_component_state htui_begin_pressable(htui_state * state, u16 component_id);
_HTUI_EXPORT void htui_end_pressable(htui_state * state);
_HTUI_EXPORT void htui_text(htui_state * state, fat_str text, htui_text_display_type display);
/* convenience but is just a pressable with text in it. */
_HTUI_EXPORT void htui_button(htui_state * state, u16 component_id, fat_str text);

_HTUI_EXPORT htui_component_state htui_throbber(htui_state * state, u16 component_id, u8 speed);
_HTUI_EXPORT htui_component_state htui_scroller_vertical(htui_state * state, u16 component_id, u8 handle_size, u8 distance_from_top);
_HTUI_EXPORT htui_component_state htui_scroller_horizontal(htui_state * state, u16 component_id, u8 handle_size, u8 distance_from_left);
_HTUI_EXPORT htui_component_state htui_loader_horizontal(htui_state * state, u16 component_id, u8 completion);
/* returns false if there was a problem ending the button. */

_HTUI_EXPORT void htui_begin_area_vertical(htui_state * state, htui_size max_width, bool8 flex, bool8 horizontal_lines, bool8 bold);
_HTUI_EXPORT void htui_begin_area_horizontal(htui_state * state, htui_size max_height, bool8 flex, bool8 vertical_lines, bool8 bold);
_HTUI_EXPORT void htui_begin_area_floating(htui_state * state, u8 x, u8 y, u8 w, u8 h);
_HTUI_EXPORT void htui_end_area(htui_state * state);


#define HTUI_IMPLEMENTATION
#if defined(HTUI_IMPLEMENTATION)

/*TODO: have this sized based on some external macros also have it be a bitset*/
u8 buffer[200*200] = {0};

_HTUI_EXPORT void htui_initalize(htui_state * state);
_HTUI_EXPORT bool8 htui_begin(htui_state * state){
        state->commands_size = 0;
        return true;
}


_HTUI_EXPORT bool8 htui_end_and_render(htui_state * state, void * user_state){

        htui_display_draw_command command = { 50, 50, 100, 100, buffer, htui_full_deep_clean, };

        if(state->screen_state == htui_screen_state_unknown){
                if(!htui_external_draw(&command, user_state)){ return 0; }
                state->screen_state=htui_screen_state_dirty;
        }

        command.mode = htui_finished;
        if(!htui_external_draw(&command, user_state)){ return 0; }

        return 1;
}
_HTUI_EXPORT htui_component_state htui_begin_pressable(htui_state * state, u16 component_id){
}

_HTUI_EXPORT void htui_end_pressable(htui_state * state);
_HTUI_EXPORT void htui_text(htui_state * state, fat_str text, htui_text_display_type display);
_HTUI_EXPORT void htui_button(htui_state * state, u16 component_id, fat_str text);

_HTUI_EXPORT htui_component_state htui_throbber(htui_state * state, u16 component_id, u8 speed);
_HTUI_EXPORT htui_component_state htui_scroller_vertical(htui_state * state, u16 component_id, u8 handle_size, u8 distance_from_top);
_HTUI_EXPORT htui_component_state htui_scroller_horizontal(htui_state * state, u16 component_id, u8 handle_size, u8 distance_from_left);
_HTUI_EXPORT htui_component_state htui_loader_horizontal(htui_state * state, u16 component_id, u8 completion);
_HTUI_EXPORT void htui_begin_area(htui_state * state, htui_size max_width, bool8 flex, bool8 horizontal_lines, bool8 bold);
_HTUI_EXPORT void htui_begin_area_vertical(htui_state * state, htui_size max_width, bool8 flex, bool8 horizontal_lines, bool8 bold);
_HTUI_EXPORT void htui_begin_area_horizontal(htui_state * state, htui_size max_height, bool8 flex, bool8 vertical_lines, bool8 bold);
_HTUI_EXPORT void htui_begin_area_floating(htui_state * state, u8 x, u8 y, u8 w, u8 h);
_HTUI_EXPORT void htui_end_area(htui_state * state);

#define HTUI_AREA(state, ...) htui_begin_area(state, __VA_ARGS__); for(int i = 0; i < 1; ++i,htui_end_area(state))

#endif /* HTUI_IMPLEMENTATION */
#endif /* HTUI */
