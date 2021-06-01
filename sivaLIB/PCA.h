#ifndef _PCA_H_
#define _PCA_H_

#include "FixedPoint.h"

//Define the canonical Principla Component
//extern const float PC[175];

//The function only works with the 175 long vector of ECG
//Principal Component is already set default inside the fuction
// int n = 175; //do not change
fixed_point_t findPCAcoeff(fixed_point_t *Sig, int n);

void findPCAcoeffs(fixed_point_t A[], fixed_point_t *Sig, int n);

fixed_point_t PCAstd(fixed_point_t *PCA, int n);

#endif