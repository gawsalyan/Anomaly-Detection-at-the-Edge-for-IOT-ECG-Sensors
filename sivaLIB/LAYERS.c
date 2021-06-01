#include "LAYERS.h"
#include "MATH_ANN.h"
#include <stdlib.h>
#include <stdio.h>

//FC_LAYER

FC_LAYER::FC_LAYER(uint8_t No_HidenNodes,string Name){ // Constructor declaration
          layer_Name = Name;
          n_HidenNodes = No_HidenNodes;
}
    
void FC_LAYER::init_LAYER(LAYER in){
      
      n_InputSize = in.n_OutputSize;           
      for (int i = 0; i<n_HidenNodes; i++){
        for (int j = 0; i<n_InputSize; j++){
          W[i][j] = (rand() % (UPPER - LOWER + 1)) + LOWER;
          dLdW[i][j] = 0;
        }
        b[i] = 0;
        dLdb = 0;
      }
      n_OutputSize = n_HidenNodes;

}

//fixed_point_t (*activation)(FC_LAYER *fc_Layer);

fixed_point_t* FC_LAYER::predict_LAYER(fixed_point_t* X){
        
       fixed_point_32t[n_HidenNodes] A;
       for (int i = 0; i<n_HidenNodes; i++){
          A[i] = 0;
          for (int j = 0; i<n_InputSize; j++){
            A[i] = A[i] + fixed_mul32(W[i][j],*(X+j)));
          }
           A[i] = ((A[i] + b[i])>>SHIFT_BIT);
       }

       return A;
}

//SIGMOID_LAYER

SIGMOID_LAYER::SIGMOID_LAYER(string Name){
    layer_Name = Name;
}

void SIGMOID_LAYER::init_LAYER(LAYER in){     
      n_InputSize = in.n_OutputSize;           
      n_OutputSize = n_InputSize;
}

fixed_point_t* SIGMOID_LAYER::predict_LAYER(fixed_point_t* X){
        
       fixed_point_32t[n_InputSize] A;
       for (int i = 0; i<n_InputSize; i++){
          A[i] = fast_Sigmoid(*(X+i))
       }
       return A;
}




LSTM_LAYER::LSTM_LAYER(uint8_t No_HidenNodes,string Name){ // Constructor declaration
          layer_Name = Name;
          n_HidenNodes = No_HidenNodes;
}
    
void FC_LAYER::init_LAYER(LAYER in){
      
      n_InputSize = in.n_OutputSize;           
      for (int i = 0; i<n_HidenNodes; i++){
        for (int j = 0; i<n_InputSize; j++){
          W[i][j] = (rand() % (UPPER - LOWER + 1)) + LOWER;
          dLdW[i][j] = 0;
        }
        b[i] = 0;
        dLdb = 0;
      }
      n_OutputSize = n_HidenNodes;

}
class LSTM_LAYER : public LAYER {
    /*** Variables header...*/
     uint8_t n_HidenNodes, n_X, n_r, n_c;
     fixed_point_t[][]  Wf, Wi, Wc, Wo;
     fixed_point_t[]    bf, bi, bc, bo;
     fixed_point_t[][]  dLdWf, dLdWi, dLdWc, dLdWo;
     fixed_point_t[]    dLdbf, dLdbi, dLdbc, dLdbo; 

    /*** Functions header... */
    LSTM_LAYER(uint8_t No_HidenNodes, uint8_t n_X, string Name);    //Constructor
    
    void init_LAYER(LAYER in);
    fixed_point_t* predict_LAYER(fixed_point_t X[][]);
}






