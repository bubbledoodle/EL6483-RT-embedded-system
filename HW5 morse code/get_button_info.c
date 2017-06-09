#include "get_button_info.h"
#include <stdio.h>





if(cp == 250 && button_flag == 0){
    printf("Button pressed\n");
    button_flag = 1;
}

if(cr == 250 && button_flag == 1){
    printf("Button released\n");
    button_flag = 0;
}