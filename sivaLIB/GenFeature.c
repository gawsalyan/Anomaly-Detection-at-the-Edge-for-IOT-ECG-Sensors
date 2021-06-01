#include "GenFeature.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

uint16_t qrssum(uint16_t *Sig, int R, int band){
    uint16_t qrsSUM;
    qrsSUM = Sum((Sig+R-band),(2*band + 1));
    printf("Details: R-band:%d, n:2*band+1:%d\n",(R-band),(2*band+1));
    return qrsSUM;
}

uint16_t vvPeakSTD(uint16_t *Sig, int R, int band){
    uint16_t *Signal = (Sig+R-band);
    uint16_t maxPeak = *Signal; 
    uint16_t minPeak = *Signal; 
    for (int i=1; i<(2*band + 1); i++){
      if (maxPeak < *(Signal+i)){
         maxPeak = *(Signal+i);
      }
      if (minPeak > *(Signal+i)){
         minPeak = *(Signal+i);
      } 
    }
    printf("Max: %.3f, Min: %.3f \n", maxPeak, minPeak);
    return(maxPeak - minPeak);
}

// Math .................................................................................................................................................

// Normalize the input vector 
void normalize(uint16_t maapstd[], uint16_t *Sig, int n){
    
    uint16_t maxVal; 
    mapstd(maapstd, Sig, n);
    maxVal = fabs(maxV(maapstd, n)); 
    //printf("Max: %.3f",maxVal);
    for (int i = 0; i<n; i++){
     maapstd[i] = maapstd[i]/maxVal; 
     //printf("%.3f, ",maapstd[i]);   
    }
    //printf("\n");   
}

// map standard deviation the input vector
void mapstd(uint16_t maapstd[], uint16_t *Sig, int n){
    
    uint16_t meaan,SD;
    //static float maapstd[175];
    meaan = mean(Sig,n);
    //printf("Mean: %.3f ",meaan);
    SD = std_mean(Sig, n, meaan);
    //printf("SD: %.3f ",SD);
    
    for (int i = 0; i < n; i++){
      maapstd[i] = (*(Sig+i) - meaan)/SD; 
      //printf("%.3f, ",maapstd[i]);
      //printf("sig:%.5f mena:%.3f SD:%.3f map:%.4f, ",*(Sig+i), meaan, SD, maapstd[i]);
    } 
    //printf("\n");
    //printf("mapstd[62]: %.3f \n",*(maapstd+62));
}

//Standard deviation given mean
uint16_t std_mean(uint16_t *Sig, int n, uint16_t mean){
    
    uint16_t sum = 0.0, SD = 0.0;

    for (int i = 0; i < n; i++){
        SD += pow(*(Sig+i) - mean, 2);
    }
    return sqrt(SD / (n-1));
}

//Standard Deviation
uint16_t std(uint16_t *Sig, int n){
    uint16_t sum = 0.0, meaan, SD = 0.0;   
    meaan = mean(Sig,n);    
    for (int i = 0; i < n; i++){
        SD += pow(*(Sig+i) - meaan, 2);
    }
    return sqrt(SD / (n-1));
}

// Mean
uint16_t mean(uint16_t *Sig, int n){
//    float sum = 0.0;
//    for (int i = 0; i < n; i++) {
//        sum += *(Sig+i);
//    }
    return (Sum(Sig,n) / n);
}

//Sum of n number of floats
uint16_t Sum(uint16_t *a, int n){
  uint16_t sum = 0.0;
  for(int i = 0; i < n; i++) {
     sum +=*a;
     a++;
  }
  //printf("Sum: %.3f",sum);
  return sum;
}

//max in the n element vector
uint16_t maxV(uint16_t *Sig, int n){
  uint16_t maxVal = *Sig; 
  for (int i=1; i < n; i++){
    if (maxVal < *(Sig+i)){
       maxVal = *(Sig+i);
    }
  }
  //printf("MaxV = %.3f\n", maxVal);
  return maxVal;
}

//min in the n element vector
uint16_t minV(uint16_t *Sig, int n){
  uint16_t minVal = *Sig; 
  for (int i=1; i<n; i++){
    if (minVal > *(Sig+i)){
       minVal = *(Sig+i);
    }
  }
  return minVal;
}




//float * mapstd175(float *Sig, int n){
//    
//    float meaan,SD;
//    static float maapstd[175];
//     meaan = mean(Sig,n);
//     printf("Mean: %.3f ",meaan);
//     SD = std_mean(Sig, n, meaan);
//     printf("SD: %.3f ",SD);
//    
//    for (int i = 0; i < n; i++){
//      maapstd[i] = (*(Sig+i) - meaan)/SD; 
//    }
//    
//      printf("mapstd: %.3f \n",*maapstd);
//    return maapstd;
//
//}




