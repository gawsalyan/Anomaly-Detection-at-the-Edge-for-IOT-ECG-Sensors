/***
Author: Siva Gawsalyan
***/

#ifndef LAYER_H
#define LAYER_H

#include "FixedPointANN.h"

//#define LOWER -2**FIXED_POINT_FRACTIONAL_ANN_BITS;
//#define UPPER  2**FIXED_POINT_FRACTIONAL_ANN_BITS;
 
struct LAYER {
    const char *layer_Name;
    int n_InputSize;
    int n_OutputSize;
   
};

extern const struct LAYERClass {
  struct LAYER (*new)(const char *Layer_Name);
} LAYER;

#endif






//class LAYER{
//
//  public:
//    //string layer_Name;
//    uint8_t n_InputSize;
//    uint8_t n_OutputSize;
//    fixed_point_ann_t LOWER = -2**FIXED_POINT_FRACTIONAL_ANN_BITS;
//    fixed_point_ann_t UPPER = 2**FIXED_POINT_FRACTIONAL_ANN_BITS;
//
//};