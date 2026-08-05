#include <stdlib.h>

#define HT15_EXPORT static
#define HT15_IMPLEMENTATION
#define USE_DEBUG_UI
#include "ht15.h"

int main(){
    if(!ht15_initalize()) exit(1);
    if(!ht15_run()) exit(1);
}

