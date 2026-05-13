#include "definitions.h"
#define HT15_UTIL_IMPLEMENTATION
#include "util.h"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef enum{
    parsing_mode_args,
    parsing_mode_file,
    parsing_mode_glyph_count,
    parsing_mode_dimensions_width,
    parsing_mode_dimensions_height,
} parsing_mode_type;

static inline void print_bits(u8 bits){
    ifor(bi, 8){
        bool8 bit = test_bit_at_index(&bits, 7 - bi);
        if(bit) fprintf(stdout, "1");
        else fprintf(stdout, "0");
    }
    bits = 0;
}

int main(int argv, char ** argc){
    parsing_mode_type parsing_mode = parsing_mode_args;
    c_str file = NULL;
    u8 width = 0, height = 0, count = 0;
    bool8 has_unknown_argument = 0;
    bool8 ascii_mode = 0;

    for(int i = 1; i < argv; ++i){
        if(parsing_mode == parsing_mode_file){
            parsing_mode = parsing_mode_args;
            file = argc[i];
        } else if(parsing_mode == parsing_mode_dimensions_width){
            parsing_mode = parsing_mode_dimensions_height;
            width = atol(argc[i]);
        }else if(parsing_mode == parsing_mode_dimensions_height){
            parsing_mode = parsing_mode_args;
            height = atol(argc[i]);
        }else if (parsing_mode == parsing_mode_glyph_count) {
            parsing_mode = parsing_mode_args;
            count = atol(argc[i]);
        } else if(parsing_mode == parsing_mode_args){
            if(strcmp(argc[i], "-f") == 0) parsing_mode = parsing_mode_file;
            else if(strcmp(argc[i], "-d") == 0) parsing_mode = parsing_mode_dimensions_width;
            else if(strcmp(argc[i], "-c") == 0) parsing_mode = parsing_mode_glyph_count;
            else if(strcmp(argc[i], "-ascii") == 0) ascii_mode = 1;
            else{
                fprintf(stderr, "Unknown arugument %s", argc[i]);
                has_unknown_argument = 1;
            }
        }         
    }

    if(has_unknown_argument){
        exit(1);
    }

    if(file == NULL){
        fprintf(stderr, "no file specified.\n");
        exit(1);
    }

    int x = 0,y = 0,c = 0;
    stbi_uc * image = stbi_load(file, &x, &y, &c, 1);

    int glyphs_per_row = x / width;

    if(ascii_mode){
        ifor(i, count){
            int yoff = (i/glyphs_per_row) * height; 
            int xoff = (i%glyphs_per_row) * width;

            ifor(iy, height){
                ifor(ix, width){
                    u32 image_index = (iy + yoff) * x + xoff + ix;
                    if(image[image_index] > 0) fprintf(stdout, "#");
                    else fprintf(stdout, " ");
                }
                fprintf(stdout, "\n");
            }
            fprintf(stdout, "\n");
        }
        return 0;
    }

    ifor(i, count){
        fprintf(stdout, "u8 const width%d = %d;\nu8 const height%d = %d;\n", i, width, i, height);
        fprintf(stdout, "u8 const data_%d[] = {", i);
        u32 bit = 0;
        int yoff = (i/glyphs_per_row) * height; 
        int xoff = (i%glyphs_per_row) * width;
        u8 bits = 0;
        ifor(iy, height){
            ifor(ix, width){
                if(height > y) return 1;
                u32 image_index = (iy + yoff) * x + xoff + ix;
                if((bit&(8-1)) == 0){
                    if(bit > 0){
                        print_bits(bits);
                        bits = 0;
                        fprintf(stdout, ", ");
                    }
                    fprintf(stdout, "0b");
                }
                set_bit_at_index(&bits, bit & 7, image[image_index] > 0);
                bit += 1;
            }
        }
        print_bits(bits);
        fprintf(stdout, "};\n");
    }
}
