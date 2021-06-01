#ifndef _HRV_H_
#define _HRV_H_

#include "FixedPoint.h"

// HRV metrics

  //RRIndex Relative RR intervals of grade x.
  //RRIndex Relative RR intervals of grade x.
  //Example:
  //        float RR_INT[11] = {1.1028, 0.8750, 0.8917, 0.9333, 0.9556, 0.9000, 0.8695, 0.8667, 0.8639, 0.9139, 0.9139};
  //        float rrIndex = HRV_RRIndex(&RR_INT[9]);  //9 is the index for the interested RR interval
  //         printf("RRIndex: %.3f\n", rrIndex);                  //output: 0.000
  fixed_point_t HRV_RRIndex(fixed_point_t *RR_prepre);
  
  //weighted SDNN (modified version of SDNN)
  //Example:
  //        float RR_INT[11] = {1.1028, 0.8750, 0.8917, 0.9333, 0.9556, 0.9000, 0.8695, 0.8667, 0.8639, 0.9139, 0.9139};
  //        int w = 10; //weight
  //        int interest = 9; //location of the interested beat in the array
  //        int n = 11; //length of the vector
  //        float wSDNN = weightedSDNN(w,RR_INT,interest,n); 
  //         printf("wSDNN: %.3f\n", wSDNN);                      //output: 0.206
  fixed_point_t weightedSDNN(int w, fixed_point_t *RR_INT, int interest, int n);
  
  //SDNN
  //Example:
  //        float RR_INT[11] = {1.1028, 0.8750, 0.8917, 0.9333, 0.9556, 0.9000, 0.8695, 0.8667, 0.8639, 0.9139, 0.9139};
  //        int n = 11; //length of the vector
  //        float SDNN = HRV_SDNN(RR_INT,n);   
  //         printf("SDNN: %.3f\n", SDNN);                  //output: 0.065
  fixed_point_t HRV_SDNN(fixed_point_t *RR_INT, int n);
  
  //SD1
  fixed_point_t HRV_SD1(fixed_point_t *RR_preprepre);

  //SD2
  fixed_point_t HRV_SD2(fixed_point_t *RR_preprepre);




#endif