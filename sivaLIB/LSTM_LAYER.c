#include "LSTM_LAYER.h"
#include "MATH_ANN.h"
#include <stdlib.h>
#include <stdio.h>
#include "ReadCSV.h"


static fixed_point_ann_t Weights_Ix[160] = {3777,14584,647,-5579,-4307,443,1519,9996,-13111,-1203,1236,-2302,-1595,13508,-963,-5309,
-9237,4575,7296,-5270,-6948,-3937,1905,10676,-2426,-17056,5518,3706,4846,-7012,868,7480,
17094,15783,-3949,-8175,-4717,-1871,1776,4009,12173,-10503,11194,14023,4353,-13670,-2067,24024,
15191,-9898,-333,8241,15084,4396,-1633,-731,3487,-19210,5447,15213,9835,-1966,-575,-9224,
3616,8641,1049,-11363,4734,-853,-7387,-20285,532,9179,5440,-5177,-569,-16263,3839,-22367,
5641,-3577,2152,7773,2229,14729,-6906,12134,-12883,-4846,-14740,-346,-8137,-10241,-6178,-4773,
-7015,-5879,7710,7285,-5894,9978,8578,-754,-7076,-1919,-8282,-3298,-15773,3104,-12047,3804,
-4593,9900,-1965,236,-3678,-8647,3254,2183,3599,-2239,19505,15545,-16227,5164,-6241,3077,
-9535,-4397,-9157,9076,15960,478,-6613,-5539,6835,12070,10846,8816,16071,17128,1926,3601,
4227,-1696,16875,-15568,-8072,4102,1251,103,9298,-8327,11228,11546,12647,5844,9219,12941};

static fixed_point_ann_t Bias_Ix[10] = {-6301,10420,10219,-12068,-10079,-5684,3863,1769,12259,5773};

static fixed_point_ann_t Weights_Fx[160] = {22468,-9101,9811,-2262,-19065,6654,-11749,17422,8264,-6102,15679,632,3060,-1692,6922,-4721,
-2979,-13476,-3827,17197,7925,8891,-2136,-3581,7576,9407,17465,2782,-10309,-431,1776,21017,
-5167,6312,-11332,5240,16682,-7665,-18987,-24373,10063,12786,-2088,7035,-15514,-495,-5775,12621,
-6033,9143,10557,-17413,12537,-11608,6459,-1099,5237,-14155,-14050,-15467,3082,3094,3486,-25365,
20776,-5961,-7817,-10709,-2981,9273,1605,-915,8366,7030,-8304,-893,1071,5408,4641,6211,
7952,-8039,-9645,-1743,-1786,-451,10684,1933,1338,210,-9146,5407,10942,8720,606,9853,
-2492,-18580,-8522,3231,12859,16081,-14902,-24640,-3090,31565,15042,6665,7746,-1440,-2981,4844,
1215,-656,-2645,5011,-5905,-989,23777,16552,-15015,425,20876,10725,742,12509,1268,11289,
-3707,8333,-5204,4478,5297,-4632,2255,8117,-738,-1396,-12558,-13463,-8511,-501,-1178,-11984,
11498,-10966,1553,13999,9124,27226,13947,-8191,-23458,2345,-10097,5381,-538,-5901,1424,6705};

static fixed_point_ann_t Bias_Fx[10] = {11374,12277,-7347,-21544,6527,8434,12318,15198,-3158,12290};

static fixed_point_ann_t Weights_Cx[160] = {11341,-4445,-14222,19045,20466,4258,4625,-16611,-5132,-153,5580,3890,12241,13398,2480,-2254,
-10564,11929,3849,-2511,-981,-16461,-4152,-6681,19904,15657,-2743,-13249,-3860,1964,-2190,-12264,
-13158,18684,5796,7690,6140,-1286,-2921,-8778,19203,3855,3029,231,3770,2981,5342,-14720,
21154,-29356,1324,5883,9996,22965,-12977,19429,-8136,-15723,2615,5922,18504,21995,3465,9563,
4911,-15608,-7314,7922,8501,15804,8384,5934,-20117,-2051,-1381,-544,11540,7637,-852,19318,
662,-8631,-38388,32972,46065,16020,-13938,-26285,3952,36627,7660,9384,6112,-980,-3522,-2506,
-15226,-759,-26229,12750,15204,-6786,-6278,-22083,22924,3245,-2730,-1856,-825,-7635,4451,-1964,
17280,709,10418,-12924,-1438,-3912,-6381,6499,-7000,-5673,-758,109,730,12155,-2504,-17145,
10845,9128,19088,-26601,-31546,-17344,11908,2990,-13421,-23284,-15788,3867,-5201,-13741,8464,-453,
1843,21715,9240,6248,3044,-2209,2521,4532,22140,2927,5980,5419,-3005,-16021,1229,-2199};

static fixed_point_ann_t Bias_Cx[10] = {-3292,-24462,-12135,9188,18361,15714,13016,-5231,-4667,4837};

static fixed_point_ann_t Weights_Ox[160] = {-2011,-4895,4431,18729,1216,12016,-6977,-12455,1738,4956,-726,-7735,-13257,-5380,-8510,-4965,
-4528,15419,15440,-15747,-17630,-15625,18355,3796,32994,-13116,384,10458,1163,12948,2753,11317,
-12580,3415,-6793,9374,14409,-13001,-19767,-1779,1046,-9664,5311,7499,-12752,13252,9081,8891,
7947,-15197,-6346,-3396,24160,6298,4271,-7599,7732,-8787,1746,10808,3074,12652,-1494,-11189,
14777,5127,-4587,26376,3405,7121,17420,7740,-4388,-1512,-2796,-5061,2017,-8831,-7324,1045,
10624,-8695,-4211,-7884,-842,12283,1012,-16821,-12452,10267,14278,368,-14557,-10661,-15417,-477,
9353,-7655,10036,3345,-5946,4085,8802,477,-2969,-4465,-12298,-10005,-4977,7446,-2624,-13576,
-2364,6789,3179,926,1077,1079,-3317,-17927,8091,8225,-4638,-9,4595,4459,465,4224,
-12081,-13661,-8907,2622,14647,1758,-1747,-6090,12826,20397,-1442,-4764,15437,27156,4821,5805,
-8790,-3003,-20177,722,21681,15330,-734,-26065,-9903,26585,18746,-8584,-16765,-9367,-7682,-1465};

static fixed_point_ann_t Bias_Ox[10] = {-14404,8068,6563,-6337,20338,-4420,-1512,3617,11104,3200};

static fixed_point_ann_t Weights_Yx[20] = {9165,4922,4905,9598,-388,-5501,-6207,-1586,9901,-4587,
-10800,-5917,-5582,-11455,2635,9604,6682,4479,-10764,5322};

static fixed_point_ann_t Bias_Yx[2] = {2803,1808};




static void init_LSTM_LAYER(struct LSTM_LAYER *this, struct LAYER *in, 
          fixed_point_ann_t *Weights_I, fixed_point_ann_t *Bias_I,
          fixed_point_ann_t *Weights_F, fixed_point_ann_t *Bias_F,
          fixed_point_ann_t *Weights_C, fixed_point_ann_t *Bias_C,
          fixed_point_ann_t *Weights_O, fixed_point_ann_t *Bias_O,
          fixed_point_ann_t *Weights_Y, fixed_point_ann_t *Bias_Y){    
    
    this->Wi = Weights_Ix; this->bi = Bias_Ix;
    this->Wf = Weights_Fx; this->bf = Bias_Fx;
    this->Wc = Weights_Cx; this->bc = Bias_Cx;
    this->Wo = Weights_Ox; this->bo = Bias_Ox;
    this->Wy = Weights_Yx; this->by = Bias_Yx;
  

    //readCSV("C:/Users/gawsa/Desktop/weights/Wo.csv", (fixed_point_ann_t *)(this->Wo), 10,16); 
       
//      for (int i = 0; i<this->n_HidenNodes; i++){
//        for (int j = 0; i<(this->n_a + this->n_x); j++){
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

    this->Ts = in->n_OutputSize;

    this->layer.n_InputSize = in->n_OutputSize;
    this->layer.n_OutputSize = this->n_HidenNodes;

} 

void LSTM_CELL_FORWARD(struct LSTM_LAYER *this, fixed_point_ann_t A[], fixed_point_ann_t X[], fixed_point_ann_t A_pre[], fixed_point_ann_t C_pre[]){
  
  uint8_t n_a = this->n_a;
  uint8_t n_y = this->n_y;
  uint8_t n_concat = this->n_concat; 
  //printf("n_a:%d,n_y:%d,n_concat:%d\n",n_a,n_y,n_concat);
  
  fixed_point_ann_t *CONCAT = (fixed_point_ann_t*)calloc(n_concat, sizeof(fixed_point_ann_t)); 
  memcpy(CONCAT, A_pre, (n_a) * sizeof(fixed_point_ann_t));
  memcpy(CONCAT + (n_a), X, (this->n_x) * sizeof(fixed_point_ann_t));
  //for(int i=0; i<n_concat;i++){ 
  //    printf("%d, ",*(CONCAT+i));
  // }
  //printf("\n");
  
  
  fixed_point_ann_2t *itin  = (fixed_point_ann_2t*)calloc((n_a),sizeof(fixed_point_ann_2t));
  fixed_point_ann_t *it  = (fixed_point_ann_t*)calloc((n_a),sizeof(fixed_point_ann_t));
  fixed_point_ann_2t *ftin  = (fixed_point_ann_2t*)calloc((n_a),sizeof(fixed_point_ann_2t));
  fixed_point_ann_t *ft  = (fixed_point_ann_t*)calloc((n_a),sizeof(fixed_point_ann_t));
  fixed_point_ann_2t *cctin  = (fixed_point_ann_2t*)calloc((n_a),sizeof(fixed_point_ann_2t));
  fixed_point_ann_t *cct  = (fixed_point_ann_t*)calloc((n_a),sizeof(fixed_point_ann_t));
  fixed_point_ann_2t *otin  = (fixed_point_ann_2t*)calloc((n_a),sizeof(fixed_point_ann_2t));
  fixed_point_ann_t *ot  = (fixed_point_ann_t*)calloc((n_a),sizeof(fixed_point_ann_t));
  fixed_point_ann_2t *yytin  = (fixed_point_ann_2t*)calloc((n_y),sizeof(fixed_point_ann_2t));
  fixed_point_ann_t *yyt  = (fixed_point_ann_t*)calloc((n_y),sizeof(fixed_point_ann_t));


  for (int i=0; i< (n_a); i++){
    for (int j=0; j< (n_concat); j++){
      //printf("%d, ", *((this->Wf+i*n_concat)+j));      
      *(ftin+i) = *(ftin+i) + fixed_mul32_ANN(*((this->Wf+i*n_concat)+j),*(CONCAT+j));
      *(itin+i) = *(itin+i) + fixed_mul32_ANN(*((this->Wi+i*n_concat)+j),*(CONCAT+j));
      *(cctin+i) = *(cctin+i) + fixed_mul32_ANN(*((this->Wc+i*n_concat)+j),*(CONCAT+j));
      *(otin+i) = *(otin+i) + fixed_mul32_ANN(*((this->Wo+i*n_concat)+j),*(CONCAT+j));
    }
    *(ft + i) = (*(ftin+i)>> SHIFT_ANN_BIT) + this->bf[i];
     //printf("%d, ", *(ft + i));   
    *(ft + i) = fast_Sigmoid(*(ft + i));
     //printf("%d, ", *(ft + i)); 
    *(it + i) = (*(itin+i)>> SHIFT_ANN_BIT) + this->bi[i];
     //printf("%d, ", *(it + i)); 
    *(it + i) = fast_Sigmoid(*(it + i));
     //printf("%d, ", *(it + i)); 
    *(cct + i) = (*(cctin+i)>> SHIFT_ANN_BIT) + this->bc[i];
     //printf("%d, ", *(cct + i)); 
    *(cct + i) = fast_Tanh(*(cct + i));
     //printf("%d, ", *(cct + i)); 
     //printf("%d, ", C_pre[i]); 
    C_pre[i] = fixed_mul_ANN(*(ft+i),C_pre[i]) + fixed_mul_ANN(*(it+i),*(cct+i));
     //printf("%d, ", C_pre[i]); 
    *(ot + i) = (*(otin+i) >> SHIFT_ANN_BIT) + this->bo[i];
     //printf("%d, ", *(ot + i)); 
    *(ot + i) = fast_Sigmoid(*(ot + i));
     //printf("%d, ", *(ot + i)); 
    A_pre[i] = fixed_mul_ANN(*(ot+i),fast_Tanh(C_pre[i]));
     //printf("%d, ", A_pre[i]); 
    //printf("\n");
  }
  //printf("\n");

  for(int i=0; i<(n_y); i++){
    for(int j=0; j<(n_a); j++){
      *(yytin+i) = *(yytin+i) + fixed_mul32_ANN(*((this->Wy+i*n_a)+j), A_pre[j]);
      //printf("W %d, A %d, Y %d, ", *((this->Wy+i*n_a)+j), A_pre[j], (int)*(yytin+i));
      //printf("\n");
    }
    *(yyt+i) = (*(yytin+i)>>SHIFT_ANN_BIT) + this->by[i]; 
    //printf("%d, %d, %d, ", (int)this->by[i], (int)(*(yytin+i)>>SHIFT_ANN_BIT), (int)*(yyt+i));
    //printf("\n");
  }
  //printf("\n");

  fast_SoftMAX(A, yyt, n_y);

//  for(int i=0; i<n_y;i++){ 
//      printf("%d, ",*(A+i));
//   }
//  printf("\n");

  free(CONCAT);
  free(itin); free(it); 
  free(ftin); free(ft); 
  free(cctin); free(cct); 
  free(otin); free(ot); 
  free(yytin); free(yyt); 
}


static void predict_LSTM_LAYER(struct LSTM_LAYER *this, fixed_point_ann_t out[], fixed_point_ann_t X[][NO_LSTM_FEATURE], fixed_point_ann_t A_pre[], fixed_point_ann_t C_pre[]){
       
       for (int t = 0; t<this->Ts; t++){
           fixed_point_ann_t *Ay = (fixed_point_ann_t*)calloc(this->n_y, sizeof(fixed_point_ann_t));
           LSTM_CELL_FORWARD(this, Ay, X[t], A_pre, C_pre);
           
           memcpy(out+(t*(this->n_y)), Ay, (this->n_y) * sizeof(fixed_point_ann_t));
           free(Ay);
       }       
}


static struct LSTM_LAYER new(uint8_t No_HidenNodes, uint8_t N_Y, const char *Layer_Name){
//		struct LSTM_LAYER lstm = malloc(sizeof(LSTM_LAYER));
//                lstm.n_HidenNodes = No_HidenNodes;
//                lstm.n_a = No_HidenNodes,
//                lstm.n_x = NO_LSTM_FEATURE,
//                lstm.n_y = N_Y, 
//                lstm.n_concat = No_HidenNodes + NO_LSTM_FEATURE,
//                lstm.Wi = (fixed_point_ann_t *)malloc((No_HidenNodes*(No_HidenNodes + NO_LSTM_FEATURE) + 1)*sizeof(fixed_point_ann_t *)),
//                lstm.Wf = (fixed_point_ann_t *)malloc((No_HidenNodes*(No_HidenNodes + NO_LSTM_FEATURE) + 1)*sizeof(fixed_point_ann_t *)),
//                        .Wc = (fixed_point_ann_t *)malloc((No_HidenNodes*(No_HidenNodes + NO_LSTM_FEATURE) + 1)*sizeof(fixed_point_ann_t *)),
//                        .Wo = (fixed_point_ann_t *)malloc((No_HidenNodes*(No_HidenNodes + NO_LSTM_FEATURE) + 1)*sizeof(fixed_point_ann_t *)),
//                        .Wy = (fixed_point_ann_t *)malloc((N_Y*(No_HidenNodes) + 1)*sizeof(fixed_point_ann_t *))                          

                struct LSTM_LAYER lstm = {                       
                        .n_HidenNodes = No_HidenNodes,
                        .n_a = No_HidenNodes,
                        .n_x = NO_LSTM_FEATURE,
                        .n_y = N_Y, 
                        .n_concat = No_HidenNodes + NO_LSTM_FEATURE,                    
		};
                lstm.Wi = (fixed_point_ann_t *)malloc((No_HidenNodes*(No_HidenNodes + NO_LSTM_FEATURE) + 1)*sizeof(fixed_point_ann_t *));
                lstm.Wf = (fixed_point_ann_t *)malloc((No_HidenNodes*(No_HidenNodes + NO_LSTM_FEATURE) + 1)*sizeof(fixed_point_ann_t *));
                lstm.Wc = (fixed_point_ann_t *)malloc((No_HidenNodes*(No_HidenNodes + NO_LSTM_FEATURE) + 1)*sizeof(fixed_point_ann_t *));
                lstm.Wo = (fixed_point_ann_t *)malloc((No_HidenNodes*(No_HidenNodes + NO_LSTM_FEATURE) + 1)*sizeof(fixed_point_ann_t *));

                lstm.layer = LAYER.new(Layer_Name);
                lstm.init = &init_LSTM_LAYER;
                lstm.predict = &predict_LSTM_LAYER;
    return lstm;
}

const struct LSTM_LAYERClass LSTM_LAYER ={
    .new=&new
};
//LSTM_LAYER

//C++

//LSTM_LAYER::LSTM_LAYER(uint8_t No_HidenNodes, uint8_t N_X, uint8_t N_Y, string Name){ // Constructor declaration
//          layer_Name = Name;
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
//void LSTM_LAYER::predict_LSTM_LAYER(fixed_point_t out[], fixed_point_t X[][]){
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
//           out[i] = fast_Sigmoid(out[i]);
//       }
//
//       free(A);
//}
//
//void LSTM_LAYER::LSTM_CELL_FORWARD(fixed_point_t A[], fixed_point_t X[], fixed_point_t A_pre){
//  fixed_point_t *CONCAT = (fixed_point_t*)calloc(n_concat, sizeof(fixed_point_t)); 
//  memcpy(CONCAT, A_pre, n_a * sizeof(fixed_point_t));
//  memcpy(CONCAT + n_a, X, n_x * sizeof(fixed_point_t)); 
//  
//  fixed_point_32t *itin  = (fixed_point_32t*)calloc(n_a,sizeof(fixed_point_32t));
//  fixed_point_t *it  = (fixed_point_t*)calloc(n_a,sizeof(fixed_point_t));
//  fixed_point_32t *ftin  = (fixed_point_32t*)calloc(n_a,sizeof(fixed_point_32t));
//  fixed_point_t *ft  = (fixed_point_t*)calloc(n_a,sizeof(fixed_point_t));
//  fixed_point_32t *ctin  = (fixed_point_32t*)calloc(n_a,sizeof(fixed_point_32t));
//  fixed_point_t *ct  = (fixed_point_t*)calloc(n_a,sizeof(fixed_point_t));
//  fixed_point_32t *otin  = (fixed_point_32t*)calloc(n_a,sizeof(fixed_point_32t));
//  fixed_point_t *ot  = (fixed_point_t*)calloc(n_a,sizeof(fixed_point_t));
//
//  for (int i=0; i< n_a; i++){
//    for (int j=0; j< n_a + n_x; 
//  }
//
//  free(CONCAT);
//  free(itin); free(it); free(ftin); free(ft); free(ctin); free(ct); free(otin); free(ot); 
//}
