#include "HRV.h"
#include "FixedPoint.h"
#include <stdio.h>
#include <math.h>

//RRIndex Relative RR intervals of grade x. 
fixed_point_t HRV_RRIndex(fixed_point_t *RR_pre){
//(2 * (Rpre - Rprepre)/ (Rpre - Rprepre))
  //uint8_t grade = 1; 
  fixed_point_t RRIndex = 0;
  
  RRIndex = *RR_pre; //RR.pre.pre to be subtracted
  RR_pre++;                  //RR.pre.
  //printf("RR: %d %d\n", *RR_pre, RRIndex); 
  RRIndex = fixed_div((RRIndex - *RR_pre),(*RR_pre + RRIndex)); 
  return RRIndex << 1;
}


//weighted SDNN (modified version of SDNN)
fixed_point_t weightedSDNN(int w, fixed_point_t *RR_INT, int interest, int n){
  
  fixed_point_32t wSDNN = 0;
  int sum = Sum(RR_INT,n);
  fixed_point_t mean = sum/n;
    
  //printf("SUM: %d, Mean: %d \n", sum, mean); 

  for (int i=0; i<n; i++){   
    fixed_point_t temp = (RR_INT[i] - mean);
    //printf("%d, ", temp);
    fixed_point_32t temp32 = (fixed_mul32(temp,temp));
    //printf("%d, \n", temp32);   
    if (i == interest){
      wSDNN = wSDNN + (w*temp32);
    }else{
      wSDNN = wSDNN + (temp32);
    }
    //printf("RR_%d: %d %d\n",i+1, RR_INT[i], temp32); 
  }
  
  wSDNN = sqrt(w*wSDNN/n); 
  wSDNN = wSDNN << (FIXED_POINT_FRACTIONAL_32BITS / 2);
  return (wSDNN >> SHIFT_BIT);
  //printf("wSDNN: %.4f \n", wSDNN); 
}


//SDNN
fixed_point_t HRV_SDNN(fixed_point_t *RR_INT, int n){
  
  fixed_point_32t SDNN = 0;
  int sum = Sum(RR_INT,n);
  fixed_point_t mean = sum/n;
  
  //printf("SUM: %.2f, Mean: %.4f \n", sum, mean); 

  for (int i=0; i<n; i++){   
    fixed_point_t temp = (RR_INT[i] - mean);
    fixed_point_32t temp32 = fixed_mul32(temp,temp);
    SDNN = SDNN + temp32;
    //printf("RR_%d: %.2f %.4f\n",i+1, RR_INT[i], temp); 
  }
  
  SDNN = sqrt(SDNN/n); 
  SDNN = SDNN << (FIXED_POINT_FRACTIONAL_32BITS / 2);
  return (SDNN >> SHIFT_BIT);
  //printf("SDNN: %.4f \n", wSDNN); 
}


//SD1
fixed_point_t HRV_SD1(fixed_point_t *RR_INT){
  
  fixed_point_t R[2] = {-2896, 2896};
  fixed_point_t XR[3] = {(fixed_mul32(R[0],RR_INT[0]) + fixed_mul32(R[1],RR_INT[1]))>>SHIFT_BIT,
                             (fixed_mul32(R[0],RR_INT[1]) + fixed_mul32(R[1],RR_INT[2]))>>SHIFT_BIT,
                             (fixed_mul32(R[0],RR_INT[2]) + fixed_mul32(R[1],RR_INT[3]))>>SHIFT_BIT};
  fixed_point_32t SD1 = 0;
  int sum = Sum(XR,3);
  fixed_point_t mean = sum/3;
  
  //printf("SUM: %.2f, Mean: %.4f \n", sum, mean); 

  for (int i=0; i<3; i++){   
    fixed_point_t temp = (XR[i] - mean);
    fixed_point_32t temp32 = fixed_mul32(temp,temp);
    SD1 = SD1 + temp32;
    //printf("RR_%d: %.2f %.4f\n",i+1, RR_INT[i], temp); 
  }
  
  SD1 = sqrt(SD1/3); 
  SD1 = SD1 << (FIXED_POINT_FRACTIONAL_32BITS / 2);
  return (SD1 >> SHIFT_BIT);
  //printf("SD1: %.4f \n", wSDNN); 
}


//SD2
fixed_point_t HRV_SD2(fixed_point_t *RR_INT){
  
  fixed_point_t R[2] = {2896, 2896};
  fixed_point_t XR[3] = {(fixed_mul32(R[0],RR_INT[0]) + fixed_mul32(R[1],RR_INT[1]))>>SHIFT_BIT,
                             (fixed_mul32(R[0],RR_INT[1]) + fixed_mul32(R[1],RR_INT[2]))>>SHIFT_BIT,
                             (fixed_mul32(R[0],RR_INT[2]) + fixed_mul32(R[1],RR_INT[3]))>>SHIFT_BIT};
  fixed_point_32t SD2 = 0;
  int sum = Sum(XR,3);
  fixed_point_t mean = sum/3;
  
  //printf("SUM: %.2f, Mean: %.4f \n", sum, mean); 

  for (int i=0; i<3; i++){   
    fixed_point_t temp = (XR[i] - mean);
    fixed_point_32t temp32 = fixed_mul32(temp,temp);
    SD2 = SD2 + temp32;
    //printf("RR_%d: %.2f %.4f\n",i+1, RR_INT[i], temp); 
  }
  
  SD2 = sqrt(SD2/3); 
  SD2 = SD2 << (FIXED_POINT_FRACTIONAL_32BITS / 2);
  return (SD2 >> SHIFT_BIT);
  //printf("SD1: %.4f \n", wSDNN); 
}
