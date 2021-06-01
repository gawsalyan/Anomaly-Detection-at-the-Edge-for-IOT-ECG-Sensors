#ifndef LSTM_LAYER_H
#define LSTM_LAYER_H

#include "LAYER.h"

#define NO_LSTM_FEATURE 6

struct LSTM_LAYER{
  struct LAYER layer;
    /*** Variables header...*/
     uint8_t n_HidenNodes, n_a, n_x, n_y, Ts, n_concat;
     fixed_point_ann_t  *Wi, *Wf, *Wc, *Wo, *Wy;
     fixed_point_ann_t  *bi, *bf, *bc, *bo, *by;
    //     fixed_point_t[][]  dLdWi, dLdWf, dLdWc, dLdWo, dLdWy;
    //     fixed_point_t[]    dLdbi, dLdbf, dLdbc, dLdbo, dLdby; 

    /*** Functions header... */
    void (*init)(struct LSTM_LAYER *this, struct LAYER *in, 
          fixed_point_ann_t *Weights_I, fixed_point_ann_t *Bias_I,
          fixed_point_ann_t *Weights_F, fixed_point_ann_t *Bias_F,
          fixed_point_ann_t *Weights_C, fixed_point_ann_t *Bias_C,
          fixed_point_ann_t *Weights_O, fixed_point_ann_t *Bias_O,
          fixed_point_ann_t *Weights_Y, fixed_point_ann_t *Bias_Y); 
    void (*predict)(struct LSTM_LAYER *this, fixed_point_ann_t out[], 
          fixed_point_ann_t X[][NO_LSTM_FEATURE], fixed_point_ann_t A_pre[], fixed_point_ann_t C_pre[]); 
};
	
extern const struct LSTM_LAYERClass {
  struct LSTM_LAYER (*new)(uint8_t No_HidenNodes, uint8_t N_Y, const char *Layer_Name);  
} LSTM_LAYER;


#endif


// C++

//class LSTM_LAYER : public LAYER {
//    /*** Variables header...*/
//     uint8_t n_HidenNodes, n_a, n_x, n_y, Ts, n_concat;
//     fixed_point_t[][]  Wi, Wf, Wc, Wo, Wy;
//     fixed_point_t[]    bi, bf, bc, bo, by;
//     fixed_point_t[][]  dLdWi, dLdWf, dLdWc, dLdWo, dLdWy;
//     fixed_point_t[]    dLdbi, dLdbf, dLdbc, dLdbo, dLdby; 
//
//    /*** Functions header... */
//    //LSTM_LAYER(uint8_t fixed_point_ann_t, uint8_t N_X, uint8_t N_Y, string Name);    //Constructor
//    LSTM_LAYER(uint8_t fixed_point_ann_t, uint8_t N_X, uint8_t N_Y);    //Constructor
//    
//    void init_LSTM_LAYER(LAYER in);
//    void predict_LSTM_LAYER(fixed_point_t A[], fixed_point_t X[][]);
//    void LSTM_CELL_FORWARD(fixed_point_t A[], fixed_point_t X[], fixed_point_t A_pre[], fixed_point_t C_pre[]);
//}