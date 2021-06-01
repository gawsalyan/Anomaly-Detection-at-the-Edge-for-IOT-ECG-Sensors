#include <stdio.h>
#include <stdlib.h>
#include "FixedPointANN.h"
#include <string.h>

void readCSV(char* path, fixed_point_ann_t *num, int row, int column);


//    printf("FC1:\n");
//    for (int i = 0; i<2; i++){
//      for (int j = 0; j<5; j++){
//        printf("%d,",*((ann.fc_layer_In.W+i*5)+j));
//      }
//      printf("\n");
//    }
//    printf("FC2:\n");
//    for (int i = 0; i<5; i++){
//      for (int j = 0; j<12; j++){
//        printf("%d,",*((ann.fc_layer_Merge.W+i*12)+j));
//      }
//      printf("\n");
//    }
//    printf("FC3:\n");
//    for (int i = 0; i<2; i++){
//      for (int j = 0; j<5; j++){
//        printf("%d,",*((ann.multiclass_layer.W+i*5)+j));
//      }
//      printf("\n");
//    }
//    printf("LSTM:\n");
//    printf("Wi:\n");
//    for (int i = 0; i<10; i++){
//      for (int j = 0; j<16; j++){
//        printf("%d,",*((ann.lstm_layer_In.Wi+i*16)+j));
//      }
//      printf("\n");
//    }
//    printf("Bi:\n");
//    for (int i = 0; i<10; i++){
//        printf("%d,",*(ann.lstm_layer_In.bi+i));
//      printf("\n");
//    }
//    printf("Wf:\n");
//    for (int i = 0; i<10; i++){
//      for (int j = 0; j<16; j++){
//        printf("%d,",*((ann.lstm_layer_In.Wf+i*16)+j));
//      }
//      printf("\n");
//    }
//    printf("Bf:\n");
//    for (int i = 0; i<10; i++){
//        printf("%d,",*(ann.lstm_layer_In.bf+i));
//      printf("\n");
//    }
//    printf("Wc:\n");
//    for (int i = 0; i<10; i++){
//      for (int j = 0; j<16; j++){
//        printf("%d,",*((ann.lstm_layer_In.Wc+i*16)+j));
//      }
//      printf("\n");
//    }
//    printf("Bc:\n");
//    for (int i = 0; i<10; i++){
//        printf("%d,",*(ann.lstm_layer_In.bc+i));
//      printf("\n");
//    }
//    printf("Wo:\n");
//    for (int i = 0; i<10; i++){
//      for (int j = 0; j<16; j++){
//        printf("%d,",*((ann.lstm_layer_In.Wo+i*16)+j));
//      }
//      printf("\n");
//    }
//    printf("Bo:\n");
//    for (int i = 0; i<10; i++){
//        printf("%d,",*(ann.lstm_layer_In.bo+i));
//      printf("\n");
//    }
//    printf("Wy:\n");
//    for (int i = 0; i<2; i++){
//      for (int j = 0; j<10; j++){
//        printf("%d,",*((ann.lstm_layer_In.Wy+i*10)+j));
//      }
//      printf("\n");
//    }
//    printf("By:\n");
//    for (int i = 0; i<2; i++){
//        printf("%d,",*(ann.lstm_layer_In.by+i));
//      printf("\n");
//    }