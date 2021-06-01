//#include "CONCAT_LAYER.h"
//#include "MATH_ANN.h"
//#include <stdlib.h>
//#include <stdio.h>
//
//CONCAT_LAYER::CONCAT_LAYER(){ //string Name){ // Constructor declaration
//          //layer_Name = Name;
//}
//    
//void CONCAT_LAYER::init_CONCAT_LAYER(LAYER in1, LAYER in2){
//      
//      n_InputSize = in1.n_OutputSize; // + in2.n_OutputSize;           
//      n_OutputSize = in1.n_OutputSize + in2.n_OutputSize;;
//}
//
//void CONCAT_LAYER::predict_CONCAT_LAYER(fixed_point_ann_t out[], fixed_point_ann_t* X1, fixed_point_ann_t* X2){
//      
//       for (int i = 0; i<n_InputSize; i++){
//            out[i] = *(X1+i);
//          }
//           
//       for (int j = 0; j<n_OutputSize; j++){
//            out[n_InputSize+j] = *(X2+j);
//          }
//
//}
