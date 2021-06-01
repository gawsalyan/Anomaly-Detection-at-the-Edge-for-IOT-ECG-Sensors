#include "INPUT_LAYER.h"
#include "MATH_ANN.h"
#include <stdlib.h>
#include <stdio.h>

static struct INPUT_LAYER new(uint8_t In_Size, const char *Layer_Name){
		struct INPUT_LAYER input = {                       
                        };
                input.layer = LAYER.new(Layer_Name);
                input.layer.n_InputSize = In_Size;
                input.layer.n_OutputSize = In_Size;
    return  input;
}

const struct INPUT_LAYERClass INPUT_LAYER ={
    .new=&new
};