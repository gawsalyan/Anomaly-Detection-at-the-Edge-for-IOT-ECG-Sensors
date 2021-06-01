#include "MULTICLASS_LAYER.h"
#include "MATH_ANN.h"
#include <stdlib.h>
#include <stdio.h>

static void init_MULTICLASS_LAYER(struct MULTICLASS_LAYER *this, struct LAYER *in, fixed_point_ann_t *Weights, fixed_point_ann_t *Bias){
    this->W = Weights;
    this->b = Bias;
    this->layer.n_InputSize = in->n_OutputSize;
    this->layer.n_OutputSize = this->n_HidenNodes;
//      for (int i = 0; i<this->n_HidenNodes; i++){
//        for (int j = 0; i<this->layer.n_InputSize; j++){
//          W[i][j] = (rand() % (UPPER - LOWER + 1)) + LOWER;
//          dLdW[i][j] = 0;
//        }
//        b[i] = 0;
//        dLdb = 0;
//      }
} 

static void predict_MULTICLASS_LAYER(struct MULTICLASS_LAYER *this, fixed_point_ann_t out[], fixed_point_ann_t X[]){
       // Dynamically allocate memory using calloc() 
       fixed_point_ann_2t *A = (fixed_point_ann_2t*)calloc(this->n_HidenNodes, sizeof(fixed_point_ann_2t)); 

       for (int i = 0; i<this->n_HidenNodes; i++){
          A[i] = 0;
          for (int j = 0; j<this->layer.n_InputSize; j++){
            A[i] = A[i] + fixed_mul32_ANN(*((this->W+i*this->layer.n_InputSize)+j),*(X+j));
          }
           out[i] = ((A[i] >>SHIFT_ANN_BIT)  + *(this->b+i));
           //out[i] = fast_Sigmoid(out[i]);
       }
       fast_SoftMAX(out, out, this->n_HidenNodes); 

       free(A);
}

static struct MULTICLASS_LAYER new(uint8_t No_HidenNodes, const char *Layer_Name){
		struct MULTICLASS_LAYER multi = {                       
                        .n_HidenNodes = No_HidenNodes
		};
                multi.layer = LAYER.new(Layer_Name);
                multi.init = &init_MULTICLASS_LAYER;
                multi.predict = &predict_MULTICLASS_LAYER;
    return multi;
}

const struct MULTICLASS_LAYERClass MULTICLASS_LAYER ={
    .new=&new
};

//MULTICLASS_LAYER::MULTICLASS_LAYER(uint8_t No_HidenNodes,string Name){ // Constructor declaration
//          layer_Name = Name;
//          n_HidenNodes = No_HidenNodes;
//}
//    
//void MULTICLASS_LAYER::init_MULTICLASS_LAYER(LAYER in){
//      
//      n_InputSize = in.n_OutputSize;           
//      for (int i = 0; i<n_HidenNodes; i++){
//        for (int j = 0; i<n_InputSize; j++){
//          W[i][j] = (rand() % (UPPER - LOWER + 1)) + LOWER;
//          dLdW[i][j] = 0;
//        }
//        b[i] = 0;
//        dLdb = 0;
//      }
//      n_OutputSize = n_HidenNodes;
//}
//
//void MULTICLASS_LAYER::predict_MULTICLASS_LAYER(fixed_point_t out[], fixed_point_t* X){
//      
//       // Dynamically allocate memory using calloc() 
//       fixed_point_32t *A = (fixed_point_32t*)calloc(n_HidenNodes, sizeof(fixed_point_32t)); 
//
//       for (int i = 0; i<n_HidenNodes; i++){
//          A[i] = 0;
//          for (int j = 0; j<n_InputSize; j++){
//            A[i] = A[i] + fixed_mul32(W[i][j],*(X+j)));
//          }
//           out[i] = (A[i] + b[i])>>SHIFT_BIT);
//       }
//       out[i] = fast_Sigmoid(out, out, n_HidenNodes);
//
//       free(A);
//}
