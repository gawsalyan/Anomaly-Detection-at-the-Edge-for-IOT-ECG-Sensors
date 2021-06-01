#include "LAYER.h"

static struct LAYER new(const char *Layer_Name){
  return (struct LAYER){
      .layer_Name = Layer_Name
  };
}

const struct LAYERClass LAYER ={
    .new=&new
};
		
//const struct LAYER_Class LAYER={...};