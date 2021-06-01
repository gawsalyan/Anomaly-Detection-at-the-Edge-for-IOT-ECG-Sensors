#include "CONCAT_LAYER.h"
#include "MATH_ANN.h"
#include <stdlib.h>
#include <stdio.h>


static void init_CONCAT_LAYER(struct CONCAT_LAYER *this, struct LAYER *in1, struct LAYER *in2){
    this->layer.n_InputSize = in1->n_OutputSize;
    this->layer.n_OutputSize = in1->n_OutputSize + in2->n_OutputSize;
} 


static void predict_CONCAT_LAYER(struct CONCAT_LAYER *this, fixed_point_ann_t out[], fixed_point_ann_t X1[], fixed_point_ann_t X2[]){
      //printf("INsz:%d, OUTsz:%d)",this->layer.n_InputSize, this->layer.n_OutputSize);
      for (int i = 0; i<this->layer.n_InputSize; i++){
            out[i] = *(X1+i);
            //printf("%d, ", out[i]);
          }
           
      for (int j = 0; j<this->layer.n_OutputSize - this->layer.n_InputSize; j++){
            out[this->layer.n_InputSize+j] = *(X2+j);
            //printf("%d, ", out[this->layer.n_InputSize+j]);
          }
      //printf("\n");
}


static struct CONCAT_LAYER new(const char *Layer_Name){
		struct CONCAT_LAYER concat = {                       
                        };
                concat.layer = LAYER.new(Layer_Name);
                concat.init = &init_CONCAT_LAYER;
                concat.predict = &predict_CONCAT_LAYER;
    return  concat;
}

const struct CONCAT_LAYERClass CONCAT_LAYER ={
    .new=&new
};

//
//CONCAT_LAYER::CONCAT_LAYER(string Name){ // Constructor declaration
//          layer_Name = Name;
//}
//    
//void CONCAT_LAYER::init_CONCAT_LAYER(LAYER in1, LAYER in2){
//      
//      n_InputSize = in1.n_OutputSize; // + in2.n_OutputSize;           
//      n_OutputSize = in1.n_OutputSize + in2.n_OutputSize;;
//}
//
//void CONCAT_LAYER::predict_CONCAT_LAYER(fixed_point_t out[], fixed_point_t* X1, fixed_point_t* X2){
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
