#include "ANN.h"
#include "MATH_ANN.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void init_ANN_SIVA(struct ANN_SIVA *this){

  //printf("LOADING ANN PARAMETERS ...\n");

  char* path = "C:/Users/gawsa/Desktop/weights/Wi";
  fixed_point_ann_t Wi[LSTM_N_A][LSTM_N_CONCAT] = {0}; 
  //readWeights(path, (fixed_point_ann_t *)Wi, LSTM_N_A,LSTM_N_CONCAT); 

  path = "C:/Users/gawsa/Desktop/weights/Bi";
  fixed_point_ann_t Bi[LSTM_N_A][1]= {0};  
  //readWeights(path, (fixed_point_ann_t *)Bi, LSTM_N_A,1); 
  
  path = "C:/Users/gawsa/Desktop/weights/Wf";
  fixed_point_ann_t Wf[LSTM_N_A][LSTM_N_CONCAT] = {0};   
  //readWeights(path, (fixed_point_ann_t *)Wf, LSTM_N_A,LSTM_N_CONCAT); 
  
  path = "C:/Users/gawsa/Desktop/weights/Bf";
  fixed_point_ann_t Bf[LSTM_N_A][1]= {0};   
  //readWeights(path, (fixed_point_ann_t *)Bf, LSTM_N_A,1); 

  path = "C:/Users/gawsa/Desktop/weights/Wc";
  fixed_point_ann_t Wc[LSTM_N_A][LSTM_N_CONCAT] = {0};   
  //readWeights(path, (fixed_point_ann_t *)Wc, LSTM_N_A,LSTM_N_CONCAT);
  
  path = "C:/Users/gawsa/Desktop/weights/Bc";
  fixed_point_ann_t Bc[LSTM_N_A][1]= {0};  
  //readWeights(path, (fixed_point_ann_t *)Bc, LSTM_N_A,1); 

  path = "C:/Users/gawsa/Desktop/weights/Wo";
  fixed_point_ann_t Wo[LSTM_N_A][LSTM_N_CONCAT] = {0};  
  //readWeights(path, (fixed_point_ann_t *)Wo, LSTM_N_A,LSTM_N_CONCAT);
  
  path = "C:/Users/gawsa/Desktop/weights/Bo";
  fixed_point_ann_t Bo[LSTM_N_A][1]= {0};   
  //readWeights(path, (fixed_point_ann_t *)Bo, LSTM_N_A,1); 

  path = "C:/Users/gawsa/Desktop/weights/Wy";
  fixed_point_ann_t Wy[LSTM_N_Y][LSTM_N_A] = {0};   
  //readWeights(path, (fixed_point_ann_t *)Wy, LSTM_N_Y,LSTM_N_A);

  path = "C:/Users/gawsa/Desktop/weights/By";
  fixed_point_ann_t By[LSTM_N_Y][1]= {0};  
  //readWeights(path, (fixed_point_ann_t *)By, LSTM_N_Y,1); 

  path = "C:/Users/gawsa/Desktop/weights/Wfc_1";
  fixed_point_ann_t Wfc_1[FC_IN_HNODES][FC_IN] = {477,26,-192,-455,278,
                                                  -721,-66,163,486,-318};   
  //readWeights(path, (fixed_point_ann_t *)Wfc_1, FC_IN_HNODES,FC_IN);

  path = "C:/Users/gawsa/Desktop/weights/Bfc_1";
  fixed_point_ann_t Bfc_1[FC_IN_HNODES]= {28,389}; 
  //readWeights(path, (fixed_point_ann_t *)Bfc_1, FC_IN_HNODES,1); 

  path = "C:/Users/gawsa/Desktop/weights/Wfc_2";
  fixed_point_ann_t Wfc_2[FC_MERGE_HNODES][LSTM_N_Y*LSTM_TS + FC_IN_HNODES] = {37,16,18,-4,0,-6,1,-2,0,-3,-780,896,
                                                                              102,-88,89,-89,-462,453,193,-193,2895,-2879,188,-324,
                                                                              79,-70,22,6,1332,-1327,65,-56,-20,45,-282,408,
                                                                              -28,86,-452,529,-296,375,73,-11,-68,134,99,-146,
                                                                              -65,44,-21,3,835,-830,1350,-1368,93,-95,-215,273}; 
  //readWeights(path, (fixed_point_ann_t *)Wfc_2, FC_MERGE_HNODES,LSTM_N_Y*LSTM_TS + FC_IN_HNODES);

  path = "C:/Users/gawsa/Desktop/weights/Bfc_2";
  fixed_point_ann_t Bfc_2[FC_MERGE_HNODES][1] = {-88,18,-74,43,-77}; 
  //readWeights(path, (fixed_point_ann_t *)Bfc_2, FC_MERGE_HNODES,1);

  path = "C:/Users/gawsa/Desktop/weights/Wfc_3";
  fixed_point_ann_t Wfc_3[N_CLASSES][FC_MERGE_HNODES] = {-327,3,5,0,-0,
                                                          373,-0,-1,-1,1};  
  //readWeights(path, (fixed_point_ann_t *)Wfc_3, N_CLASSES,FC_MERGE_HNODES);

  path = "C:/Users/gawsa/Desktop/weights/Bfc_3";
  fixed_point_ann_t Bfc_3[N_CLASSES][1] = {267,-78};  
  //readWeights(path, (fixed_point_ann_t *)Bfc_3, N_CLASSES,1); 

  this->lstm_layer_In.init(&(this->lstm_layer_In),&(this->in_PCA.layer),Wi,Bi,Wf,Bf,Wc,Bc,Wo,Bo,Wy,By);
  this->fc_layer_In.init(&(this->fc_layer_In),&(this->in_RR.layer), Wfc_1, Bfc_1);
  this->concat_layer.init(&(this->concat_layer),&(this->lstm_layer_In.layer),&(this->fc_layer_In.layer));
  this->fc_layer_Merge.init(&(this->fc_layer_Merge), &(this->concat_layer.layer), Wfc_2, Bfc_2);
  this->multiclass_layer.init(&(this->multiclass_layer), &(this->fc_layer_Merge.layer),Wfc_3,Bfc_3);  
  
  //NRF_LOG_INFO("DONE: LOADING ANN PARAMETERS.");
  //printf("DONE: LOADING ANN PARAMETERS\n");
} 

static void predict_ANN_SIVA(struct ANN_SIVA *this, fixed_point_ann_t out[], fixed_point_ann_t X1[][NO_LSTM_FEATURE], fixed_point_ann_t X2[]){
  
  //printf("%d \n%",X2[0]);

  fixed_point_ann_t *A_LSTM = (fixed_point_ann_t *)calloc((this->lstm_layer_In.n_y * this->lstm_layer_In.Ts),sizeof(fixed_point_ann_t));
  fixed_point_ann_t *A_RR = (fixed_point_ann_t *)calloc(this->fc_layer_In.n_HidenNodes,sizeof(fixed_point_ann_t));
  fixed_point_ann_t *A_concat = (fixed_point_ann_t *)calloc((this->lstm_layer_In.n_y * this->lstm_layer_In.Ts)+this->fc_layer_In.n_HidenNodes,sizeof(fixed_point_ann_t));  
  
  fixed_point_ann_t *A_pre = (fixed_point_ann_t *)calloc(this->lstm_layer_In.n_a,sizeof(fixed_point_ann_t));
  fixed_point_ann_t *C_pre = (fixed_point_ann_t *)calloc(this->lstm_layer_In.n_a,sizeof(fixed_point_ann_t));
 
  this->lstm_layer_In.predict(&(this->lstm_layer_In),A_LSTM,X1, A_pre,C_pre);
  this->fc_layer_In.predict(&(this->fc_layer_In),A_RR,X2); 
  
  this->concat_layer.predict(&(this->concat_layer),A_concat,A_LSTM,A_RR);
  free(A_LSTM); free(A_RR); free(A_pre); free(C_pre);

  fixed_point_ann_t *A_merge = (fixed_point_ann_t *)calloc(this->fc_layer_Merge.n_HidenNodes,sizeof(fixed_point_ann_t));
  this->fc_layer_Merge.predict(&(this->fc_layer_Merge),A_merge,A_concat);
  this->multiclass_layer.predict(&(this->multiclass_layer),out,A_merge);
  free(A_concat); free(A_merge);   
}


static struct ANN_SIVA new(void){
		struct ANN_SIVA ann_SIVA = {                                              
		};
                ann_SIVA.in_PCA = INPUT_LAYER.new(LSTM_TS,"IN1");
                ann_SIVA.lstm_layer_In = LSTM_LAYER.new(LSTM_N_A,LSTM_N_Y,"LSTM");
                ann_SIVA.in_RR = INPUT_LAYER.new(FC_IN,"IN2");
                ann_SIVA.fc_layer_In = FC_LAYER.new(FC_IN_HNODES,"FC1"); 
                ann_SIVA.fc_layer_Merge = FC_LAYER.new(FC_MERGE_HNODES,"FC2");
                ann_SIVA.concat_layer = CONCAT_LAYER.new("CONCAT");
                ann_SIVA.multiclass_layer = MULTICLASS_LAYER.new(N_CLASSES,"CLASS");
                ann_SIVA.init = &init_ANN_SIVA;
                ann_SIVA.predict = &predict_ANN_SIVA;
    return ann_SIVA;
}

const struct ANNClass ANN_SIVA ={
    .new=&new
};


void readWeights(char* path, fixed_point_ann_t *numberArray, int row, int column){
    FILE *fp;
    //printf(path);printf("\n");
    fp = fopen(path, "r");
    rewind(fp);
    //read file into array
    //int *numberArray[column];//(int **)calloc(row*column, sizeof(int **));  
    
    // Return if could not open file 
    if (fp == NULL){
        //printf("Error Reading File\n");
        exit (0);
        }
    
    int i_row = 0;
    int j_col = 0;
    int val = 0;
    int sign = 1;
    
    do
    { 
        
        // Taking input single character at a time 
        char c = fgetc(fp); 
        //printf("%c",c);

        // Checking for end of file 
        if (feof(fp)){ 
            //printf("came here\n");
            break ; 
            }
  
        //printf("ch:%c, i:%d, j:%d\n", c, i_row,j_col); 
        
        if(c == '\n') {
          //*(*(numberArray + i_row) + j_col) = val;
          *((numberArray + i_row*column) + j_col) = sign*val;
          i_row = i_row + 1;
          j_col = 0;
          val = 0;
          sign = 1;
        }
        if(c==','){
           //*(*(numberArray + i_row) + j_col) = val;
          *((numberArray + i_row*column) + j_col) = sign*val;
          j_col = j_col + 1;
          val = 0;
          sign = 1;
        }
        if(c=='-'){
          sign = -1;
        }
        if ((c >= 48) && (c <= 57)) {
          val = val*10 + c - 48;
        }

    }  while(1); 
    
//    printf("MAtrix is:\n");
//    for (int i = 0; i < row; i++){
//      for (int j = 0; j < column; j++){
//        printf("%d,",  *((numberArray + i*column) + j)); //*((numberArray + i*column) + j)
//      }
//      printf("\n");
//    }
    
    fclose(fp);
    return &numberArray;
}


//FILE* fp1;
////struct my_record records[100];
//fp1 = fopen("sample.csv", "r");
//    while (1) {
//        c = fgetc(fp1);
//        if (c == EOF)
//            break;
//        else
//            printf("%c", c);
//    }
//
//    for (i = 1; i <= 10; i++) {
//        fscanf(fp1, "%d,%[^,],%[^,],%[^,],%[^,],%d,%d",
//               &c1[i], &c2[i], &c3[i], &c4[i], &c5[i],
//               &c6[i], &c7[i], &c8[i], &c9[i], &c10[i]);
//    }



// for testing in main

////    int Ain1[5] = {20,-50, 23, 2, -1}; //{55,55,55,1,1};
////    int Aout1[2] ={0};
////    ann.fc_layer_In.predict(&ann.fc_layer_In, Aout1, Ain1);
////      for (int i = 0; i<2; i++){
////        printf("%d,",Aout[i]);
////      }
////      printf("\n");
//    
////    int Ain1[5] = {53,55, 52, 1, -1}; //{55,55,55,1,1};
////    int Ain2[5][6] = {{1, 36,  0,  0,  0, 27}
////                  ,  {1, 43,  0,  0,  0, 19}
////                  ,  {6, 44,  0,  0,  0, 15}
////                  ,  {1, 40,  0,  0,  0, 23}
////                  ,  {1, 29,  0,  0,  0, 34}
////                      }; //{55,55,55,1,1};
//
////    int Ain1[5] = {50, 51, 48, 2, -1};
////    int Ain2[5][6] =  {{3,    39,     0,     0,     0,    21}
////     ,{3,   40,     0,     0,     0,    21}
////     ,{1,    37,     0,     0,     0,    26}
////     ,{1,    30,     0,     0,     0,    33}
////     ,{1,    37,     0,     0,     0,    26}};
//
//    int Ain1[5] = {34,60,50, 9,-18};
//    int Ain2[5][6] = {{3,    47,     0,     0,     0,    14}
//      ,{1,    40,     0,     0,     0,    23}
//      ,{2,    50,     0,     0,     0,    12}
//      ,{1,    59,     0,     0,     0,     4}
//      ,{0,    52,     0,     0,     0,    12}};
//
//
//    int Aout2[10] ={0};
//    int Apre[10] ={0};
//    int Cpre[10] ={0};
////    ann.lstm_layer_In.predict(&ann.lstm_layer_In, Aout2, Ain2, Apre, Cpre);
////      for (int i = 0; i<10; i++){
////        printf("%d,",Aout[i]);
////      }
////      printf("\n");
//
//    int Aout[2] ={0};
//    ann.predict(&ann,Aout,Ain2,Ain1); 
//    for (int i = 0; i<2; i++){
//        printf("%d,",Aout[i]);
//      }
//
////    printf("FC1:\n");
////    for (int i = 0; i<2; i++){
////      for (int j = 0; j<5; j++){
////        printf("%d,",*((ann.fc_layer_In.W+i*5)+j));
////      }
////      printf("\n");
////    }
