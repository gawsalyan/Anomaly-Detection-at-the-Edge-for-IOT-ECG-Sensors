#ifndef INPUT_LAYER_H
#define INPUT_LAYER_H

#include "LAYER.h"

struct INPUT_LAYER{
  struct LAYER layer;
  
  //void (*init)(void); 
  //void (*predict)(void); 

};
	
extern const struct INPUT_LAYERClass {
  struct INPUT_LAYER (*new)(uint8_t In_Size, const char *Layer_Name);  
} INPUT_LAYER;


#endif


//C++

//class INPUT_LAYER : public LAYER {
//    /*** Variables header...*/
//
//    /*** Functions header... */
//    //INPUT_LAYER(uint8_t N_INPUT_SIZE,string Name);    //Constructor
//    INPUT_LAYER(uint8_t N_INPUT_SIZE);    //Constructor
//}
