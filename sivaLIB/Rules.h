#ifndef _RULES_H_
#define _RULES_H_

#include "FixedPoint.h"


int checkforArrhythmia_II(fixed_point_t wSDNN, fixed_point_t rrIndex,fixed_point_t *RRprepre, fixed_point_t RRmean, fixed_point_t SD1,
                            fixed_point_t SD2, fixed_point_t  SDNN, fixed_point_t PCA, fixed_point_t PCAstd, 
                            fixed_point_32t QRSsum, fixed_point_32t QRSenergy);

int checkforArrhythmia(fixed_point_t wSDNN, fixed_point_t rrIndex,fixed_point_t *RRprepre, fixed_point_32t QRSsum, fixed_point_t PCAcoef, 
                        fixed_point_t VVstd, fixed_point_t preVVstd, fixed_point_t SDNN, fixed_point_t PCAstd);

int checkforArrhythmia2(fixed_point_t wSDNN, fixed_point_t rrIndex,fixed_point_t * RRprepre);

int check_wSDNN_RRindex(fixed_point_t wSDNN, fixed_point_t rrIndex);

int checkforRRIndex(fixed_point_t rrIndex);

#define TSI_fp_0_6 2458 //0.6
#define TSI_fp_1_8 7373 //1.8
#define TSI_fp_1_15 4710 //1.15
#define TSI_fp_0_3 1229 //0.3
#define TSI_fp_0_8 3277 //0.8
#define TSI_fp_2_2 9011 //2.2
#define TSI_fp_3_0 12288 //3.0
#define TSI_fp_0_2 819 //0.2
int checkforTsipouras(fixed_point_t RRprepre, fixed_point_t RRpre, fixed_point_t RRpos); 

#define EXA_fp_1_464 5997 //1.464
#define EXA_fp_1_377 5640 //1.377
#define EXA_fp_0_358 1466 //0.358
#define EXA_fp_0_656 2687 //0.656
#define EXA_fp_1_1484 4704 // 1.1484
#define EXA_fp_1_461 5984 // 1.1484
int checkforExarchos(fixed_point_t RRprepre, fixed_point_t RRpre, fixed_point_t RRpos);

#endif