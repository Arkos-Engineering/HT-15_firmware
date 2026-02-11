
#if !defined(OSR_UI)
#define OSR_UI

#include "definitions.h"

#if !defined(OSR_UI_EXPORT)
#define OSR_UI_EXPORT
#endif

typedef enum PACKED{
        ht15ui_text_type,
        ht15ui_image_type,
        ht15ui_button_type,
        ht15ui_end_button_type,
        ht15ui_begin_list_vertical_type,
        ht15ui_end_list_vertical_type,
        ht15ui_begin_list_horizontal_type,
        ht15ui_end_list_horizontal_type,
} ht15ui_command_type;

typedef struct {
        c_str text;
        enum PACKED options{
                ht15ui_editable = 1 <<0,
                //if 0 fades else wraps text.
                ht15ui_fade_or_wrap = 1 <<0,

        } options;
} ht15ui_text_command;
typedef struct {

} ht15ui_image_command;
typedef struct {

} ht15ui_button_command;
typedef struct {

} ht15ui_list_vertical_command;
typedef struct {

} ht15ui_end_list_vertical_command;
typedef struct {

} ht15ui_list_horizontal_command;
typedef struct {

} ht15ui_end_list_horizontal_command;

typedef struct{
        ht15ui_command_type type;
        u8 id;
} ht15ui_component_id;

typedef struct{
        //Inexes into component ids.
        u16 component_index;
        ht15ui_command_type type;
} ht15ui_component;

typedef enum{
        ht15ui_selected = 1,
        ht15ui_pressed,
} ht15ui_component_state;

#define MAX_DRAW_COMMANDS 200
typedef struct{
        u16 width,height;
        void * window_buffer;
        //TODO as of Mar 11 2025: hold onto previous state for each component to tell if it needs to be re-rendered for its section of the screen. @Zea Lynn

        //TODO as of Mar 11 2025: component_ids can grow larger than MAX_DRAW_COMMANDS due to it being the way to address unique components. @Zea Lynn
        ht15ui_component_id            component_ids[MAX_DRAW_COMMANDS];
        ht15ui_component               components[MAX_DRAW_COMMANDS];
        ht15ui_text_command            text_commands[MAX_DRAW_COMMANDS];
        ht15ui_image_command           image_commands[MAX_DRAW_COMMANDS];
        ht15ui_button_command          button_commands[MAX_DRAW_COMMANDS];
        ht15ui_list_vertical_command   list_vertical_commands[MAX_DRAW_COMMANDS];
        ht15ui_list_horizontal_command list_horizontal_commands[MAX_DRAW_COMMANDS];

        u16                           component_id_count;
        u8                            commands_count;
} ht15ui_state;

OSR_UI_EXPORT bool ht15ui_begin(ht15ui_state * state);
//TODO as of March 11 2025: figure out how to only tell the paper display controller what parts of the buffer to re_render. @Zea Lynn
OSR_UI_EXPORT bool ht15ui_end_and_render(ht15ui_state * state, void * output_buffer);

//mest be acompanied with end button
OSR_UI_EXPORT ht15ui_component_state ht15ui_button(ht15ui_state * state, u16 component_id, ht15ui_button_command command);
//returns false if there was a problem ending the button.
OSR_UI_EXPORT bool ht15ui_end_button(ht15ui_state * state);

#if !defined(OSR_UI_IMPLEMENTATION) && !defined(OSR_UI_OMIT_IMPLEMENTATION)
#define OSR_UI_IMPLEMENTATION

#endif /* OSR_UI_IMPLEMENTATION */
#endif /* OSR_UI */
