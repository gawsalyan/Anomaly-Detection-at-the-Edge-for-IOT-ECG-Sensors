#include "FC_LAYER.h"
#include "MATH_ANN.h"
#include <stdlib.h>
#include <stdio.h>

static void init_FC_LAYER(struct FC_LAYER *this, struct LAYER *in, fixed_point_ann_t *Weights, fixed_point_ann_t *Bias){
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

static void predict_FC_LAYER(struct FC_LAYER *this, fixed_point_ann_t out[], fixed_point_ann_t X[]){
       // Dynamically allocate memory using calloc() 
       fixed_point_ann_2t *A = (fixed_point_ann_2t*)calloc(this->n_HidenNodes, sizeof(fixed_point_ann_2t)); 
       
       //printf("FC LAYER:\n");

       for (int i = 0; i<this->n_HidenNodes; i++){
          A[i] = 0;
          for (int j = 0; j<this->layer.n_InputSize; j++){
            //printf("W%d%d:%d, ",i,j,*((this->W+i)+j));
            //printf("X%d:%d, ",j,*(X+j));
            A[i] = A[i] + fixed_mul32_ANN(*((this->W+i*this->layer.n_InputSize)+j),*(X+j));
            //printf("A%d:%d, ",j,A[i]);
          }
           //printf("\nA:%d, ",A[i]);
           out[i] = ((A[i] >>SHIFT_ANN_BIT) + *(this->b+i));
           //printf("\nA%d:%d, ",i,A[i]);
           //printf("b:%d,",*(this->b+i));
           //printf("%d:",out[i]);
           out[i] = fast_Sigmoid(out[i]);
           //printf("%d, ",out[i]);
           //printf("\n"); 
       }
       //printf("\n"); 
       free(A);
}


static struct FC_LAYER new(uint8_t No_HidenNodes, const char *Layer_Name){
		struct FC_LAYER fc = {                       
                        .n_HidenNodes = No_HidenNodes
		};
                fc.layer = LAYER.new(Layer_Name);
                fc.init = &init_FC_LAYER;
                fc.predict = &predict_FC_LAYER;
    return fc;
}

const struct FC_LAYERClass FC_LAYER ={
    .new=&new
};


