#ifndef CONCAT_LAYER_H
#define CONCAT_LAYER_H

#include "LAYER.h"

struct CONCAT_LAYER{
  struct LAYER layer;
  //fixed_point_ann_t[][]  dLdW;
  //fixed_point_ann_t[]    dLdb;
  void (*init)(struct CONCAT_LAYER *this, struct LAYER *in1, struct LAYER *in2); 
  void (*predict)(struct CONCAT_LAYER *this, fixed_point_ann_t out[], fixed_point_ann_t X1[], fixed_point_ann_t X2[]); 

};
	
extern const struct CONCAT_LAYERClass {
  struct CONCAT_LAYER (*new)(const char *Layer_Name);  
} CONCAT_LAYER;

#endif


//C++

//class CONCAT_LAYER : public LAYER {
//    /*** Variables header...*/
//
//    /*** Functions header... */
//    //CONCAT_LAYER(string Name);    //Constructor
//    CONCAT_LAYER();    //Constructor
//    
//    void init_CONCAT_LAYER(LAYER in1, LAYER in2);
//    void predict_CONCAT_LAYER(fixed_point_ann_t A[], fixed_point_ann_t X1[], fixed_point_ann_t X2[]);
//}
