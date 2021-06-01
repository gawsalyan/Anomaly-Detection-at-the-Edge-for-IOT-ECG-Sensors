#include "FixedPoint.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

fixed_point_32t qrssum(fixed_point_t *Sig, int R, int band){
    int qrsSUM;
    qrsSUM = Sum((Sig+R-band),(2*band + 1));
    //printf("Details: R-band:%d, n:2*band+1:%d\n",(R-band),(2*band+1));
    return qrsSUM << SHIFT_BIT;
}

fixed_point_32t qrsEnergy(fixed_point_t *Sig, int R, int band){
    int qrsENERGY;

    qrsENERGY = 0;
    for(int i = 0; i < 2*band + 1; i++) {
       qrsENERGY += (Square(*(Sig+R-band + i)) >> (FIXED_POINT_FRACTIONAL_BITS - SHIFT_BIT));
    }
    //printf("Details: R-band:%d, n:2*band+1:%d\n",(R-band),(2*band+1));
    return qrsENERGY;
}

fixed_point_t vvPeakSTD(fixed_point_t *Sig, int R, int band){
    fixed_point_t *Signal = (Sig+R-band);
    fixed_point_t maxPeak = *Signal; 
    fixed_point_t minPeak = *Signal; 
    for (int i=1; i<(2*band + 1); i++){
      if (maxPeak < *(Signal+i)){
         maxPeak = *(Signal+i);
      }
      if (minPeak > *(Signal+i)){
         minPeak = *(Signal+i);
      } 
    }
    //printf("Max: %d, Min: %d\n", maxPeak, minPeak);
    return(maxPeak - minPeak);
}


// Math Library

/// Converts fixed format -> double
inline double fixed_to_float(fixed_point_t input){
    return ((double)input / (double)(1 << FIXED_POINT_FRACTIONAL_BITS));
}

/// Converts double to fixed format
inline fixed_point_t float_to_fixed(double input){
    return (fixed_point_t)((input * (double)(1 << FIXED_POINT_FRACTIONAL_BITS)));
}


//mutilply 2 fixed point numbers
fixed_point_t fixed_mul(fixed_point_t x, fixed_point_t y){
    int mul = ((int)x * (int)y);
    //printf("mul %d\n", mul);
    return (mul >> FIXED_POINT_FRACTIONAL_BITS);
}

//mutilply 2 fixed point numbers
fixed_point_32t fixed_mul32(fixed_point_t x, fixed_point_t y){
    int64_t mul = ((int64_t)x << SHIFT_BIT) * ((int64_t)y << SHIFT_BIT);
    //printf("size int %d\n", sizeof(int));
    //printf("mul %d\n", mul);
    return (mul >> FIXED_POINT_FRACTIONAL_32BITS);
}

//divide 2 fixed point numbers
fixed_point_t fixed_div(fixed_point_t x, fixed_point_t y){
    return ((int32_t)x * (1 << FIXED_POINT_FRACTIONAL_BITS)) / y;
}


// Normalize the input vector 
void normalize(fixed_point_t maapstd[], fixed_point_t *Sig, int n){
    
    fixed_point_t maxVal; 
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
void mapstd(fixed_point_t maapstd[], fixed_point_t *Sig, int n){
    
    fixed_point_t meaan,SD;
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
fixed_point_t std_mean(fixed_point_t *Sig, int n, fixed_point_t meaan){
    
    int sum = 0;
    fixed_point_32t SD = 0;

    for (int i = 0; i < n; i++){
        fixed_point_t temp = (*(Sig+i) - meaan); 
        SD += (fixed_mul32(temp,temp));
    }

    //return round(SD / (n-1));
    SD = (sqrt((SD / (n-1))));
    SD = SD << (FIXED_POINT_FRACTIONAL_32BITS / 2);
    return (SD >> SHIFT_BIT);

}


//Standard Deviation
fixed_point_t std(fixed_point_t *Sig, int n){
    int sum = 0; 
    fixed_point_t meaan;
    fixed_point_32t SD = 0;   
    
    meaan = mean(Sig,n);    
    for (int i = 0; i < n; i++){
        fixed_point_t temp = (*(Sig+i) - meaan); 
        SD += (fixed_mul32(temp,temp));
    }
    
    //return round(SD / (n-1));
    SD = (sqrt((SD / (n-1))));
    SD = SD << (FIXED_POINT_FRACTIONAL_32BITS / 2);
    return (SD >> SHIFT_BIT);
}


// Mean
fixed_point_t mean(fixed_point_t *Sig, int n){
//    float sum = 0.0;
//    for (int i = 0; i < n; i++) {
//        sum += *(Sig+i);
//    }
    return (Sum(Sig,n) / n);
}

//Sum of n number of floats
int Sum(fixed_point_t *a, int n){
  int sum = 0;
  for(int i = 0; i < n; i++) {
     sum += *a;
     a++;
  }
  //printf("Sum: %.3f",sum);
  return sum;
}


//max in the n element vector
fixed_point_t maxV(fixed_point_t *Sig, int n){
  fixed_point_t maxVal = *Sig; 
  for (int i=1; i < n; i++){
    if (maxVal < abs(*(Sig+i))){
       maxVal = abs(*(Sig+i));
    }
  }
  //printf("MaxV = %.3f\n", maxVal);
  return maxVal;
}


//min in the n element vector
fixed_point_t minV(fixed_point_t *Sig, int n){
  fixed_point_t minVal = *Sig; 
  for (int i=1; i<n; i++){
    if (minVal > abs(*(Sig+i))){
       minVal = abs(*(Sig+i));
    }
  }
  return minVal;
}

