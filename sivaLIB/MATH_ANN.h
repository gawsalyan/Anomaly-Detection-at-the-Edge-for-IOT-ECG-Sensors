#include "FixedPointANN.h"
//#include <math.h>
 
#define maxPr 61 //round(0.96016 * factor);
#define maxRng 123 //round(1.92033 * factor);
#define CoeffA 17 //round(0.26037 * factor);

fixed_point_ann_t fast_Sigmoid(fixed_point_ann_t x);
fixed_point_ann_t fast_Tanh(fixed_point_ann_t x);
void fast_SoftMAX(fixed_point_ann_t out[], fixed_point_ann_t *x, int n);