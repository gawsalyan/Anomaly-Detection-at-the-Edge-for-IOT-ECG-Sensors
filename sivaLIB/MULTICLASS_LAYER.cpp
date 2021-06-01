//#include "MULTICLASS_LAYER.h"
//#include "MATH_ANN.h"
//#include <stdlib.h>
//#include <stdio.h>
//
//MULTICLASS_LAYER::MULTICLASS_LAYER(uint8_t No_HidenNodes){ //,string Name){ // Constructor declaration
//          //layer_Name = Name;
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
//void MULTICLASS_LAYER::predict_MULTICLASS_LAYER(fixed_point_ann_t out[], fixed_point_ann_t* X){
//      
//       // Dynamically allocate memory using calloc() 
//       fixed_point_ann_2t *A = (fixed_point_ann_2t*)calloc(n_HidenNodes, sizeof(fixed_point_ann_2t)); 
//
//       for (int i = 0; i<n_HidenNodes; i++){
//          A[i] = 0;
//          for (int j = 0; j<n_InputSize; j++){
//            A[i] = A[i] + fixed_mul32_ANN(W[i][j],*(X+j)));
//          }
//           out[i] = (A[i] + b[i])>>SHIFT_ANN_BIT);
//       }
//       out[i] = fast_Sigmoid(out, out, n_HidenNodes);
//
//       free(A);
//}
