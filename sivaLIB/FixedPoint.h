#ifndef _FIXEDPOINT_H_
#define _FIXEDPOINT_H_

#include <stdint.h>

/// Fixed-point Format: 1.5 (16-bit)
typedef int16_t fixed_point_t;
typedef int32_t fixed_point_32t;

#define FIXED_POINT_FRACTIONAL_BITS 12
#define FIXED_POINT_FRACTIONAL_32BITS 16
#define FIXED_POINT_FRACTIONAL_BITS_VALUEONE 2**FIXED_POINT_FRACTIONAL_BITS
#define SHIFT_BIT (FIXED_POINT_FRACTIONAL_32BITS - FIXED_POINT_FRACTIONAL_BITS)

// Feature Extraction 

fixed_point_32t qrssum(fixed_point_t *Sig, int R, int band);    
  //QRS Sum of the vector in th eproposed band
  // Sig is the vector of the beat (Ex: size-175 @ 250Hz)
  // R is the location of R peak in the vector (Ex: 62 @ 250Hz)
  // band is the proposed interval band where the sum has to be calculated around the R peak location
  // Example:
  //          float x[] = {1, 1.1, 0.98, 1.05, 1.01};
  //          R=2; //in the middle; 
  //          band=1;   //as the proposed window is [1,2,3]:{1.1,0.98,1.05}
  //          float qrsSum = qrssum(x, R, band);  
  //          printf("qrsSum : %.3f\n", qrsSum);          //output: 3.130

fixed_point_32t qrsEnergy(fixed_point_t *Sig, int R, int band);
//QRS Energy of the vector in th eproposed band
  // Sig is the vector of the beat (Ex: size-175 @ 250Hz)
  // R is the location of R peak in the vector (Ex: 62 @ 250Hz)
  // band is the proposed interval band where the sum has to be calculated around the R peak location
  // Example:
  //          float x[] = {1, 1.1, 0.98, 1.05, 1.01};
  //          R=2; //in the middle; 
  //          band=1;   //as the proposed window is [1,2,3]:{1.1,0.98,1.05}
  //          float qrsEnergy = qrsqrsEnergy(x, R, band);  
  //          printf("qrsEnergy : %.3f\n", qrsEnergy);          //output: 3.27
              
fixed_point_t vvPeakSTD(fixed_point_t *Sig, int R, int band);
  //VV peak to peak difference in the proposed band
  // Sig is the vector of the beat (Ex: size-175 @ 250Hz)
  // R is the location of R peak in the vector (Ex: 62 @ 250Hz)
  // band is the proposed interval band where the sum has to be calculated around the R peak location
  // Example:
  //          float x[] = {1, 1.1, 0.98, 1.05, 1.01};
  //          R=2; //in the middle; 
  //          band=1;   //as the proposed window is [1,2,3]:{1.1,0.98,1.05}
  //          float vvSTD = vvPeakSTD(x, R, band);
  //          printf("vvSTD : %.3f\n", vvSTD);            //output: 0.120






/// Math Functions

//Converts fixed format -> double
double fixed_to_float(fixed_point_t input);

//Converts double -> fixed format
fixed_point_t float_to_fixed(double input);

//mutilply 2 fixed point numbers
fixed_point_t fixed_mul(fixed_point_t x, fixed_point_t y);

//mutilply 2 fixed point numbers
fixed_point_32t fixed_mul32(fixed_point_t x, fixed_point_t y);

//divide 2 fixed point numbers
fixed_point_t fixed_div(fixed_point_t x, fixed_point_t y);

//Normalize the n element vector
void normalize(fixed_point_t maapstd[], fixed_point_t *Sig, int n);

//Map Standard Deviation of the n element vector
void mapstd(fixed_point_t maapstd[], fixed_point_t *Sig, int n);

//Standard deviation given the mean of n element vector
fixed_point_t std_mean(fixed_point_t *Sig, int n, fixed_point_t mean);

//Standard deviation of n element vector
fixed_point_t std(fixed_point_t *Sig, int n);

//Mean of n element vector
fixed_point_t mean(fixed_point_t *Sig, int n);

//Sum of n element vector
int Sum(fixed_point_t *a, int n);

//Square: Power of 2
#define Square(x) ((x)*(x))

//max in the n element vector
fixed_point_t maxV(fixed_point_t *Sig, int n);

//min in the n element vector
fixed_point_t minV(fixed_point_t *Sig, int n);

#endif