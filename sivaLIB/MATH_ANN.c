#include "MATH_ANN.h"


fixed_point_ann_t fast_Sigmoid(fixed_point_ann_t x){
    fixed_point_ann_t f  = (fixed_div_ANN(x,(FIXED_POINT_FRACTIONAL_ANN_BITS_VALUEONE + abs(x)))>>1) + (FIXED_POINT_FRACTIONAL_ANN_BITS_VALUEONE>>1);
    return f;
}


fixed_point_ann_t fast_Tanh(fixed_point_ann_t x){      
    fixed_point_ann_t f;
    if (x > maxRng){
      f = maxPr;
    } 
    else if  (x > 0){
      f = maxPr - fixed_mul_ANN(CoeffA,(fixed_point_ann_t)(Square(x - maxRng)>>FIXED_POINT_FRACTIONAL_ANN_BITS));
    }
    else if (x < -maxRng){
      f = -maxPr;
    }
    else if (x < 0){
      f = fixed_mul_ANN(CoeffA,(fixed_point_ann_t)(Square(x + maxRng)>>FIXED_POINT_FRACTIONAL_ANN_BITS)) - maxPr;
    }else{
      f = 1;
    }
    return f;
}


void fast_SoftMAX(fixed_point_ann_t out[],fixed_point_ann_t *x, int n){


  int sumA = Sum_ANN(x,n) + n*FIXED_POINT_FRACTIONAL_ANN_BITS_VALUEONE;
  
  //printf("%d+%d,sum:%d\n", *x, *(x+1), sumA);

  for (int i = 0; i < n; i++){
      out[i] = fixed_div_ANN((FIXED_POINT_FRACTIONAL_ANN_BITS_VALUEONE + *(x+i)),(fixed_point_ann_t) sumA);
  }
}

