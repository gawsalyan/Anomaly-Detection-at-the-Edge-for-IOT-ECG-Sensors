#include "FixedPointANN.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

// Math Library

/// Converts double to fixed format
inline fixed_point_ann_t float_to_fixed_ANN(double input){
    return (fixed_point_ann_t)((input * (double)(1 << FIXED_POINT_FRACTIONAL_ANN_BITS)));
}


//mutilply 2 fixed point numbers
fixed_point_ann_t fixed_mul_ANN(fixed_point_ann_t x, fixed_point_ann_t y){
    int mul = ((int)x * (int)y);
    //printf("mul %d\n", mul);
    return (mul >> FIXED_POINT_FRACTIONAL_ANN_BITS);
}

//mutilply 2 fixed point numbers
fixed_point_ann_2t fixed_mul32_ANN(fixed_point_ann_t x, fixed_point_ann_t y){
    int64_t mul = ((int64_t)x) * ((int64_t)y);
    //printf("size int %d\n", sizeof(int));
    //printf("mul %d\n", mul);
    return (mul >> (FIXED_POINT_FRACTIONAL_ANN_BITS - SHIFT_ANN_BIT));
}

//divide 2 fixed point numbers
fixed_point_ann_t fixed_div_ANN(fixed_point_ann_t x, fixed_point_ann_t y){
    return ((int32_t)x * (1 << FIXED_POINT_FRACTIONAL_ANN_BITS)) / y;
}


// Normalize the input vector 
void normalize_ANN(fixed_point_ann_t maapstd[], fixed_point_ann_t *Sig, int n){
    
    fixed_point_ann_t maxVal; 
    mapstd(maapstd, Sig, n);
    maxVal = maxV(maapstd, n); 
    //printf("Max: %.3f",maxVal);
    for (int i = 0; i<n; i++){
     maapstd[i] = fixed_div(maapstd[i],maxVal); 
     //printf("%.3f, ",maapstd[i]);   
    }
    //printf("\n");   
}


// map standard deviation the input vector
void mapstd_ANN(fixed_point_ann_t maapstd[], fixed_point_ann_t *Sig, int n){
    
    fixed_point_ann_t meaan,SD;
    //static float maapstd[175];
     meaan = mean(Sig,n);
     //printf("Mean: %.3f ",meaan);
     SD = std_mean(Sig, n, meaan);
     //printf("SD: %.3f ",SD);
    
    for (int i = 0; i < n; i++){
      maapstd[i] = fixed_div((*(Sig+i) - meaan),SD); 
      //printf("%.3f, ",maapstd[i]);
      //printf("sig:%.5f mena:%.3f SD:%.3f map:%.4f, ",*(Sig+i), meaan, SD, maapstd[i]);
    } 
     //printf("\n");
     //printf("mapstd[62]: %.3f \n",*(maapstd+62));
}


//Standard deviation given mean
fixed_point_ann_t std_mean_ANN(fixed_point_ann_t *Sig, int n, fixed_point_ann_t meaan){
    
    int sum = 0;
    fixed_point_ann_2t SD = 0;

    for (int i = 0; i < n; i++){
        fixed_point_ann_t temp = (*(Sig+i) - meaan); 
        SD += (fixed_mul32(temp,temp));
    }

    //return round(SD / (n-1));
    SD = (sqrt((SD / (n-1))));
    SD = SD << (FIXED_POINT_FRACTIONAL_ANN_2BITS / 2);
    return (SD >> SHIFT_ANN_BIT);

}


//Standard Deviation
fixed_point_ann_t std_ANN(fixed_point_ann_t *Sig, int n){
    int sum = 0; 
    fixed_point_ann_t meaan;
    fixed_point_ann_2t SD = 0;   
    
    meaan = mean(Sig,n);    
    for (int i = 0; i < n; i++){
        fixed_point_ann_t temp = (*(Sig+i) - meaan); 
        SD += (fixed_mul32(temp,temp));
    }
    
    //return round(SD / (n-1));
    SD = (sqrt((SD / (n-1))));
    SD = SD << (FIXED_POINT_FRACTIONAL_ANN_2BITS / 2);
    return (SD >> SHIFT_ANN_BIT);
}


// Mean
fixed_point_ann_t mean_ANN(fixed_point_ann_t *Sig, int n){
//    float sum = 0.0;
//    for (int i = 0; i < n; i++) {
//        sum += *(Sig+i);
//    }
    return (Sum(Sig,n) / n);
}

//Sum of n number of floats
int Sum_ANN(fixed_point_ann_t *a, int n){
  int64_t sum = 0;
  for(int i = 0; i < n; i++) {
     sum = sum + *(a+i);
     //printf("%d, Sum: %d, ", *(a+i),sum);
  }
  
  return sum;
}


//max in the n element vector
fixed_point_ann_t maxV_ANN(fixed_point_ann_t *Sig, int n){
  fixed_point_ann_t maxVal = *Sig; 
  for (int i=1; i < n; i++){
    if (maxVal < abs(*(Sig+i))){
       maxVal = abs(*(Sig+i));
    }
  }
  //printf("MaxV = %.3f\n", maxVal);
  return maxVal;
}


//min in the n element vector
fixed_point_ann_t minV_ANN(fixed_point_ann_t *Sig, int n){
  fixed_point_ann_t minVal = *Sig; 
  for (int i=1; i<n; i++){
    if (minVal > abs(*(Sig+i))){
       minVal = abs(*(Sig+i));
    }
  }
  return minVal;
}

