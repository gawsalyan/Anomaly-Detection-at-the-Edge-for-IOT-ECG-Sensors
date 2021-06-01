#ifndef MULTICLASS_LAYER_H
#define MULTICLASS_LAYER_H

#include "LAYER.h"

struct MULTICLASS_LAYER{
  struct LAYER layer;
  uint8_t n_HidenNodes;
  fixed_point_ann_t *W;
  fixed_point_ann_t *b;
  //fixed_point_ann_t[][]  dLdW;
  //fixed_point_ann_t[]    dLdb;
  void (*init)(struct MULTICLASS_LAYER *this, struct LAYER *in, fixed_point_ann_t *Weights, fixed_point_ann_t *Bias); 
  void (*predict)(struct MULTICLASS_LAYER *this, fixed_point_ann_t out[], fixed_point_ann_t X[]); 

};
	
extern const struct MULTICLASS_LAYERClass {
  struct MULTICLASS_LAYER (*new)( uint8_t No_HidenNodes, const char *Layer_Name);  
} MULTICLASS_LAYER;

#endif



//C++

//class MULTICLASS_LAYER : public LAYER {
//    /*** Variables header...*/
//     uint8_t n_HidenNodes, n_r, n_c;
//     fixed_point_t[][]  W;
//     fixed_point_t[]    b;
//     fixed_point_t[][]  dLdW;
//     fixed_point_t[]    dLdb; 
//
//    /*** Functions header... */
//    //MULTICLASS_LAYER(uint8_t No_HidenNodes,string Name);    //Constructor
//    MULTICLASS_LAYER(uint8_t No_HidenNodes);    //Constructor
//    
//    void init_MULTICLASS_LAYER(LAYER in);
//    void predict_MULTICLASS_LAYER(fixed_point_t A[], fixed_point_t X[]);
//}