#ifndef FC_LAYER_H
#define FC_LAYER_H

#include "LAYER.h"

struct FC_LAYER{
  struct LAYER layer;
  uint8_t n_HidenNodes;
  fixed_point_ann_t *W;
  fixed_point_ann_t *b;
  //fixed_point_ann_t[][]  dLdW;
  //fixed_point_ann_t[]    dLdb;
  void (*init)(struct FC_LAYER *this, struct LAYER *in, fixed_point_ann_t *Weights, fixed_point_ann_t *Bias); 
  void (*predict)(struct FC_LAYER *this, fixed_point_ann_t out[], fixed_point_ann_t X[]); 

};
	
extern const struct FC_LAYERClass {
  struct FC_LAYER (*new)( uint8_t No_HidenNodes, const char *Layer_Name);  
} FC_LAYER;

#endif


//C++

//class FC_LAYER : public LAYER {
//    /*** Variables header...*/
//     uint8_t n_HidenNodes, n_r, n_c;
//     fixed_point_ann_t[][]  W;
//     fixed_point_ann_t[]    b;
//     fixed_point_ann_t[][]  dLdW;
//     fixed_point_ann_t[]    dLdb; 
//
//    /*** Functions header... */
//    //FC_LAYER(uint8_t No_HidenNodes,string Name);    //Constructor
//    FC_LAYER(uint8_t No_HidenNodes);    //Constructor
//    
//    void init_FC_LAYER(LAYER in);
//    void predict_FC_LAYER(fixed_point_ann_t A[], fixed_point_ann_t X[]);
//}
