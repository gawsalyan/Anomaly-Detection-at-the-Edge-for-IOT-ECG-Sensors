/***
Author: Siva Gawsalyan
***/

#ifndef ANN_H
#define ANN_H

#include "INPUT_LAYER.h"
#include "FC_LAYER.h"
#include "LSTM_LAYER.h"
#include "CONCAT_LAYER.h"
#include "MULTICLASS_LAYER.h"

#include <string.h>

#define LSTM_TS 5 
#define LSTM_N_A 10
#define LSTM_N_Y 2
#define LSTM_N_X 6
#define LSTM_N_CONCAT LSTM_N_A+LSTM_N_X
#define FC_IN 5 
#define FC_IN_HNODES 2
#define FC_MERGE_HNODES 5
#define N_CLASSES 2   

struct ANN_SIVA{

     /*** Variables header...*/
     struct INPUT_LAYER in_PCA, in_RR;
     struct FC_LAYER fc_layer_In, fc_layer_Merge;
     struct LSTM_LAYER lstm_layer_In;
     struct CONCAT_LAYER concat_layer;
     struct MULTICLASS_LAYER multiclass_layer;

    /*** Functions header... */
    void (*init)(struct ANN_SIVA *this);
    void (*predict)(struct ANN_SIVA *this, fixed_point_ann_t out[], fixed_point_ann_t X1[][NO_LSTM_FEATURE], fixed_point_ann_t X2[]); 
};
	
extern const struct ANNClass {
  struct ANN_SIVA (*new)( void );  
} ANN_SIVA;

void readWeights(char* path,fixed_point_ann_t *num, int row, int column);

#endif





//class ANN{
//
//     /*** Variables header...*/
//     //string ANN_Name;
//     INPUT_LAYER in_PCA, in_RR;
//     FC_LAYER fc_layer_In, fc_layer_Merge;
//     LSTM_LAYER lstm_layer_In;
//     CONCAT_LAYER concat_layer;
//     MULTICLASS_LAYER multiclass_layer;
//
//    /*** Functions header... */
//    //ANN(string Name);    //Constructor
//    ANN();    //Constructor
//    
//
//    void init_ANN();
//    void predict(fixed_point_ann_t A[], fixed_point_ann_t X1[][], fixed_point_ann_t X2[]);
//};
