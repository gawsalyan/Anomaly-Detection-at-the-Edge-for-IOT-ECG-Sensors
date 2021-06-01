///***
//Author: Siva Gawsalyan
//***/
//
//#ifndef LAYERS_H
//#define LAYERS_H
//
//#include "FixedPointANN.h"
// 
//class LAYERS{
//
//  public:
//    string layer_Name;
//    uint8_t n_InputSize;
//    uint8_t n_OutputSize;
//};
//
//class FC_LAYER : public LAYERS {
//    /*** Variables header...*/
//     uint8_t n_HidenNodes, n_r, n_c;
//     fixed_point_t[][]  W;
//     fixed_point_t[]    b;
//     fixed_point_t[][]  dLdW;
//     fixed_point_t[]    dLdb; 
//
//    /*** Functions header... */
//    FC_LAYER(uint8_t No_HidenNodes,string Name);    //Constructor
//    
//    void init_LAYER(LAYER in);
//    fixed_point_t* predict_LAYER(fixed_point_t X[]);
//}
//
//class SIGMOID_LAYER : public LAYERS {
//    /*** Variables header...*/
//
//    /*** Functions header... */
//    SIGMOID_LAYER(string Name);    //Constructor
//    
//    void init_LAYER(LAYER in);
//    fixed_point_t* predict_LAYER(fixed_point_t X[]);
//}
//
//
//class LSTM_LAYER : public LAYERS {
//    /*** Variables header...*/
//     uint8_t n_HidenNodes, n_X, n_r, n_c;
//     fixed_point_t[][]  Wf, Wi, Wc, Wo;
//     fixed_point_t[]    bf, bi, bc, bo;
//     fixed_point_t[][]  dLdWf, dLdWi, dLdWc, dLdWo;
//     fixed_point_t[]    dLdbf, dLdbi, dLdbc, dLdbo; 
//
//    /*** Functions header... */
//    LSTM_LAYER(uint8_t No_HidenNodes, uint8_t n_X, string Name);    //Constructor
//    
//    void init_LAYER(LAYER in);
//    fixed_point_t* predict_LAYER(fixed_point_t X[][]);
//}
//
//
// 
//#endif