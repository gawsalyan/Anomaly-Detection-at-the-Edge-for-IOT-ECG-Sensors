//#include "LSTM_LAYER.h"
//#include "MATH_ANN.h"
//#include <stdlib.h>
//#include <stdio.h>
//
////FC_LAYER
//
//LSTM_LAYER::LSTM_LAYER(uint8_t No_HidenNodes, uint8_t N_X, uint8_t N_Y){ //, string Name){ // Constructor declaration
//          //layer_Name = Name;
//          n_HidenNodes = No_HidenNodes;
//          n_a = n_HidenNodes;
//          n_x = N_X;
//          n_y = N_Y;
//          n_concat = n_a + n_x;
//  
//}
//    
//void LSTM_LAYER::init_LSTM_LAYER(LAYER in){
//      
//      n_InputSize = in.n_OutputSize; 
//      Ts = n_InputSize          
//      for (int i = 0; i<n_HidenNodes; i++){
//        for (int j = 0; i<(n_a + n_x); j++){
//          Wi[i][j] = (rand() % (UPPER - LOWER + 1)) + LOWER;
//          dLdWi[i][j] = 0;
//          Wf[i][j] = (rand() % (UPPER - LOWER + 1)) + LOWER;
//          dLdWf[i][j] = 0;
//          Wc[i][j] = (rand() % (UPPER - LOWER + 1)) + LOWER;
//          dLdWc[i][j] = 0;
//          Wo[i][j] = (rand() % (UPPER - LOWER + 1)) + LOWER;
//          dLdWo[i][j] = 0;
//        }
//        bi[i] = 0; dLdbi = 0;
//        bf[i] = 0; dLdbf = 0;
//        bc[i] = 0; dLdbc = 0;
//        bo[i] = 0; dLdbo = 0;
//      }
//      n_OutputSize = n_HidenNodes;
//}
//
//void LSTM_LAYER::predict_LSTM_LAYER(fixed_point_ann_t out[], fixed_point_ann_t X[][], fixed_point_ann_t A_pre[], fixed_point_ann_t C_pre[]){
//      
//       
//       for (int t = 0; t<Ts; t++){
//           fixed_point_ann_t *Ay = (fixed_point_ann_t*)calloc(n_y, sizeof(fixed_point_ann_t));
//           LSTM_CELL_FORWARD(Ay, X[t], A_pre, C_pre);
//           
//           memcpy(out+t*n_y, Ay, n_y * sizeof(fixed_point_ann_t));
//           free(Ay);
//       }
//       
//}
//
//void LSTM_LAYER::LSTM_CELL_FORWARD(fixed_point_ann_t A[], fixed_point_ann_t X[], fixed_point_ann_t A_pre[], fixed_point_ann_t C_pre[]){
//  
//  fixed_point_ann_t *CONCAT = (fixed_point_ann_t*)calloc(n_concat, sizeof(fixed_point_ann_t)); 
//  memcpy(CONCAT, A_pre, n_a * sizeof(fixed_point_ann_t));
//  memcpy(CONCAT + n_a, X, n_x * sizeof(fixed_point_ann_t)); 
//  
//  fixed_point_ann_2t *itin  = (fixed_point_ann_2t*)calloc(n_a,sizeof(fixed_point_ann_2t));
//  fixed_point_ann_t *it  = (fixed_point_ann_t*)calloc(n_a,sizeof(fixed_point_ann_t));
//  fixed_point_ann_2t *ftin  = (fixed_point_ann_2t*)calloc(n_a,sizeof(fixed_point_ann_2t));
//  fixed_point_ann_t *ft  = (fixed_point_ann_t*)calloc(n_a,sizeof(fixed_point_ann_t));
//  fixed_point_ann_2t *cctin  = (fixed_point_ann_2t*)calloc(n_a,sizeof(fixed_point_ann_2t));
//  fixed_point_ann_t *cct  = (fixed_point_ann_t*)calloc(n_a,sizeof(fixed_point_ann_t));
//  fixed_point_ann_2t *otin  = (fixed_point_ann_2t*)calloc(n_a,sizeof(fixed_point_ann_2t));
//  fixed_point_ann_t *ot  = (fixed_point_ann_t*)calloc(n_a,sizeof(fixed_point_ann_t));
//  fixed_point_ann_2t *yytin  = (fixed_point_ann_2t*)calloc(n_y,sizeof(fixed_point_ann_2t));
//  fixed_point_ann_t *yyt  = (fixed_point_ann_t*)calloc(n_y,sizeof(fixed_point_ann_t));
//
//
//  for (int i=0; i< n_a; i++){
//    for (int j=0; j< n_a + n_x; j++){        
//      *(ftin+i) = *(ftin+i) + fixed_mul32_ANN(Wf[i][j],*(CONCAT+j));
//      *(itin+i) = *(itin+i) + fixed_mul32_ANN(Wi[i][j],*(CONCAT+j));
//      *(cctin+i) = *(cctin+i) + fixed_mul32_ANN(Wc[i][j],*(CONCAT+j));
//      *(otin+i) = *(otin+i) + fixed_mul32_ANN(Wo[i][j],*(CONCAT+j));
//    }
//    *(ftin+i) = *(ftin+i) + bf[i];
//    *(ft + i) = fast_Sigmoid((*(ftin+i) >> SHIFT_ANN_BIT));
//    *(itin+i) = *(itin+i) + bi[i];
//    *(it + i) = fast_Sigmoid((*(itin+i) >> SHIFT_ANN_BIT));
//    *(cctin+i) = *(cctin+i) + bc[i];
//    *(cct + i) = fast_Tanh((*(cctin+i) >> SHIFT_ANN_BIT));
//    C_pre[i] = fixed_mul_ANN(*(ft+i),C_pre[i]) + fixed_mul_ANN(*(it+i),(cct+i));
//    *(otin+i) = *(otin+i) + bo[i];
//    *(ot + i) = fast_Sigmoid((*(otin+i) >> SHIFT_ANN_BIT));
//    A_pre[i] = fixed_mul_ANN(*(ot+i),fast_Tanh(C_pre[i]));
//  }
//
//  for(int i=0; i<n_y; i++){
//    for(int j=0; j<n_a; j++){
//      *(yytin+i) = *(yytin+i) + fixed_mul32_ANN(Wy[i][j],Apre[j])
//    }
//    *(yyt+i) = *(yytin+i)>>SHIFT_ANN_BIT + by[i]; 
//  }
//  fast_SoftMAX(A, yyt, n_y);
//
//  free(CONCAT);
//  free(itin); free(it); 
//  free(ftin); free(ft); 
//  free(cctin); free(cct); 
//  free(otin); free(ot); 
//  free(yytin); free(yyt); 
//}
