#ifndef _FIXEDPOINTANN_H_
#define _FIXEDPOINTANN_H_

#include <stdint.h>

/// Fixed-point Format: 1.5 (16-bit)
typedef int32_t fixed_point_ann_t;
typedef int64_t fixed_point_ann_2t;



#define FIXED_POINT_FRACTIONAL_ANN_BITS 6
#define FIXED_POINT_FRACTIONAL_ANN_2BITS 10
#define FIXED_POINT_FRACTIONAL_ANN_BITS_VALUEONE (1<<FIXED_POINT_FRACTIONAL_ANN_BITS)
//#define FIXED_POINT_FRACTIONAL_ANN_BITS_VALUEONE 2**FIXED_POINT_FRACTIONAL_ANN_BITS
#define SHIFT_ANN_BIT (FIXED_POINT_FRACTIONAL_ANN_2BITS - FIXED_POINT_FRACTIONAL_ANN_BITS)



/// Math Functions

//Converts double -> fixed format
fixed_point_ann_t float_to_fixed_ANN(double input);

//mutilply 2 fixed point numbers
fixed_point_ann_t fixed_mul_ANN(fixed_point_ann_t x, fixed_point_ann_t y);

//mutilply 2 fixed point numbers
fixed_point_ann_2t fixed_mul32_ANN(fixed_point_ann_t x, fixed_point_ann_t y);

//divide 2 fixed point numbers
fixed_point_ann_t annfixed_div_ANN(fixed_point_ann_t x, fixed_point_ann_t y);

//Normalize the n element vector
void normalize_ANN(fixed_point_ann_t maapstd[], fixed_point_ann_t *Sig, int n);

//Map Standard Deviation of the n element vector
void mapstd_ANN(fixed_point_ann_t maapstd[], fixed_point_ann_t *Sig, int n);

//Standard deviation given the mean of n element vector
fixed_point_ann_t std_mean_ANN(fixed_point_ann_t *Sig, int n, fixed_point_ann_t mean);

//Standard deviation of n element vector
fixed_point_ann_t std_ANN(fixed_point_ann_t *Sig, int n);

//Mean of n element vector
fixed_point_ann_t mean_ANN(fixed_point_ann_t *Sig, int n);

//Sum of n element vector
int Sum_ANN(fixed_point_ann_t *a, int n);

//Square: Power of 2
#define Square(x) ((x)*(x))

//max in the n element vector
fixed_point_ann_t maxV_ANN(fixed_point_ann_t *Sig, int n);

//min in the n element vector
fixed_point_ann_t minV_ANN(fixed_point_ann_t *Sig, int n);

#endif