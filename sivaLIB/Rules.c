#include "Rules.h"
#include <math.h>
#include <stdlib.h>

struct{ 
    //Define result boolean with 1 bit size
    //Normal:0 , Abnormal:1
    unsigned int result : 1;
    unsigned int resultTsipouras : 1;
    unsigned int resultExarchos : 1;     
    
    //Define the reason for the class
    unsigned int reason : 8; 
    
} RESULT; 


//checkforArrhythmia_II
//INCART
//const fixed_point_t theta_1 = 3132; //(0.76456*2^12)  
//const fixed_point_t theta_2 = -27238; //(-6.65*2^12)
//const fixed_point_t theta_3 = -11756; //(-2.87*2^12)
//const fixed_point_t theta_4 = -1352; //(-0.33*2^12)
//const fixed_point_t theta_5 = 2990; //(0.73*2^12)
//const fixed_point_t theta_6 = 5325; //(1.3*2^12)
//const fixed_point_t theta_7 = -3482; //(-0.85*2^12)
//const int mu_PCA = -3007; //-506; //-2072660; //(-506.0205*2^12)
//const int mu_PCAstd = 990; //108; //440542; //(107.5543*2^12)
//const fixed_point_t mu_QRSsum = 4180; //(1.0206*2^12)
//const int mu_QRSenergy = 390; //1595996; //(389.6474*2^12)
//MIT
const fixed_point_t theta_1 = 1556; //(0.38*2^12)  
const fixed_point_t theta_2 = -27238; //(-6.65*2^12)
const fixed_point_t theta_3 = -11756; //(-2.87*2^12)
const fixed_point_t theta_4 = -1352; //(-0.33*2^12)
const fixed_point_t theta_5 = 2990; //(0.73*2^12)
const fixed_point_t theta_6 = 5325; //(1.3*2^12)
const fixed_point_t theta_7 = -3482; //(-0.85*2^12)
const fixed_point_t mu_PCA = -18126; //(-4.4254*2^12)
const fixed_point_t mu_PCAstd = 1420; //(0.3467*2^12)
const int mu_QRSsum = -14;//-57492; //(-14.0361*2^12)
const fixed_point_t mu_QRSenergy = 31741; //(7.7493*2^12)

int checkforArrhythmia_II(fixed_point_t wSDNN, fixed_point_t rrIndex,fixed_point_t *RRprepre, fixed_point_t RRmean, fixed_point_t SD1,
                            fixed_point_t SD2, fixed_point_t  SDNN, fixed_point_t PCA, fixed_point_t PCAstd, 
                            fixed_point_32t QRSsum, fixed_point_32t QRSenergy){
  RESULT.result = 0;  //apriori Normal
  if  ((*(RRprepre+1) - wSDNN - rrIndex) < theta_1){
      RESULT.result = 1;
    if (((RRmean + SD1 + SD2)- (*(RRprepre+2) + SDNN + ((fixed_point_t) ((PCA << (FIXED_POINT_FRACTIONAL_BITS))/mu_PCA)) 
        + abs((fixed_point_t)((QRSsum >> (SHIFT_BIT))/mu_QRSsum)) + ((fixed_point_t) ((QRSenergy << (FIXED_POINT_FRACTIONAL_BITS - SHIFT_BIT))/mu_QRSenergy)))) < theta_2){
        RESULT.result = 1; 
    }else{
        RESULT.result = 0; 
                if((*(RRprepre+1) - SDNN -wSDNN - rrIndex - (((fixed_point_t) ((PCA<< (FIXED_POINT_FRACTIONAL_BITS ))/mu_PCA)) 
                    + ((fixed_point_t) ((PCAstd<< (FIXED_POINT_FRACTIONAL_BITS))/mu_PCAstd)))) < theta_3){
                    RESULT.result = 1;                             
                }else{
                    if(((*(RRprepre+1) + *(RRprepre+2)) - wSDNN - rrIndex - ((fixed_point_t) ((PCA << (FIXED_POINT_FRACTIONAL_BITS))/mu_PCA))) < theta_4){
                        RESULT.result = 1;              
                    }else{
                        RESULT.result = 0;  
                    }
                }
    }
   }else{
      RESULT.result = 0;
      if(*(RRprepre+2) < theta_5){
          if(((fixed_point_t) ((QRSenergy << (FIXED_POINT_FRACTIONAL_BITS - SHIFT_BIT))/mu_QRSenergy) >> SHIFT_BIT) > theta_6){
               if(((fixed_point_t)((QRSsum >> (SHIFT_BIT))/mu_QRSsum)) < theta_7){
                   RESULT.result = 1;  
               }else{
                   RESULT.result = 0;  
               }     
          }else{
               RESULT.result = 0;  
          }     
      }else{
          RESULT.result = 0;  
      }
   }
   
   //printf("%d, ", RESULT.reason); 
   return (RESULT.result + 48);
}


//checkforArrhythmia
const fixed_point_t thr_wSDNN = 6144;//float thr_wSDNN = 0.4;
const fixed_point_t thr_rrIndex = -860; //float thr_rrIndex = 0.22;

int checkforArrhythmia(fixed_point_t wSDNN, fixed_point_t rrIndex,fixed_point_t *RRprepre, fixed_point_32t QRSsum, fixed_point_t PCAcoef, 
                        fixed_point_t VVstd, fixed_point_t preVVstd, fixed_point_t SDNN, fixed_point_t PCAstd){

  RESULT.result = 0;  //apriori Normal
  RESULT.reason = 0;

  if ((wSDNN < 16385) & (wSDNN > 0)){
      if (wSDNN < thr_wSDNN){
      //RESULT.result = 0;  
      RESULT.result = checkforRRIndex(rrIndex);
        if (RESULT.result){
           RESULT.reason = RESULT.reason + 1 << 1;
        }
    
        RESULT.resultTsipouras = checkforTsipouras(*RRprepre, *(RRprepre+1), *(RRprepre+2));
        if (RESULT.resultTsipouras){
           RESULT.result = 1;
           RESULT.reason = RESULT.reason + 1 << 2;
        } else if((QRSsum > 850000 & PCAcoef > 0)|(QRSsum < -850000 & PCAcoef < 0)){
           RESULT.result = 1; 
           RESULT.reason = RESULT.reason + 1 << 3;
        } else if (VVstd > 8192 & preVVstd < 8192){
           RESULT.result = 1; 
           RESULT.reason = RESULT.reason + 1 << 4;
        } else if (SDNN > 820){
           RESULT.result = 1; 
           RESULT.reason = RESULT.reason + 1 << 5;
        }
    

      }else{
        RESULT.result = 1;  //Abnormal
        RESULT.reason = 1;
      }

      RESULT.resultExarchos = checkforExarchos(*RRprepre, *(RRprepre+1), *(RRprepre+2));
      if (RESULT.resultExarchos){
         RESULT.result = 1; 
         RESULT.reason = RESULT.reason + 1 << 6;
      }
         
      if (PCAstd > 18000){
         RESULT.result = 1; 
         RESULT.reason = RESULT.reason + 1 << 7;
      }    

  }else{
    RESULT.result = 1;  //Abnormal
    RESULT.reason = 1;
  }

  //RESULT.result = (RESULT.result | RESULT.resultTsipouras | RESULT.resultExarchos);
  //printf("%d, ", RESULT.reason); 

  return (RESULT.result + 48);
}

int checkforArrhythmia2(fixed_point_t wSDNN, fixed_point_t rrIndex,fixed_point_t *RRprepre){

  RESULT.result = 0;  //apriori Normal
  RESULT.reason = 0;

  if (abs(wSDNN) < thr_wSDNN){
    //RESULT.result = 0;  
    RESULT.result = checkforRRIndex(rrIndex);
    if (RESULT.result){
       RESULT.reason = RESULT.reason + 1 << 1;
    }
    
    RESULT.resultTsipouras = checkforTsipouras(*RRprepre, *(RRprepre+1), *(RRprepre+2));
    if (RESULT.resultTsipouras){
       RESULT.reason = RESULT.reason + 1 << 2;
    }else{
      RESULT.resultExarchos = checkforExarchos(*RRprepre, *(RRprepre+1), *(RRprepre+2));
      if (RESULT.resultExarchos){
         RESULT.reason = RESULT.reason + 1 << 3;
      }   
    }
    

  }else{
    RESULT.result = 1;  //Abnormal
    RESULT.reason = 1;
  }

  //RESULT.result = (RESULT.result | RESULT.resultTsipouras | RESULT.resultExarchos);
  
  //printf("%d, ", RESULT.reason); 

  return RESULT.result;
}

int check_wSDNN_RRindex(fixed_point_t wSDNN, fixed_point_t rrIndex){

  RESULT.result = 0;  //apriori Normal

  if (abs(wSDNN) > thr_wSDNN){
    //RESULT.result = 0;  
    RESULT.result = checkforRRIndex(rrIndex);

  }else{
    RESULT.result = 0;  //ABNORMAL
  }
  return RESULT.result;
}


int checkforExarchos(fixed_point_t RRprepre, fixed_point_t RRpre, fixed_point_t RRpos){

  RESULT.resultExarchos = 0;  //apriori Normal
  
  if ((RRpre <= EXA_fp_1_464 && (RRprepre + RRpre + RRpos <= EXA_fp_1_377)) ||
        ((RRprepre + RRpre + RRpos <= EXA_fp_1_377) && RRpre > EXA_fp_0_358 && RRpre <= EXA_fp_0_656 && fixed_div(RRpos,RRprepre) > EXA_fp_1_1484) || 
        (RRpre > EXA_fp_1_461)){
      RESULT.resultExarchos = 1; 
  }
  
  return RESULT.resultExarchos;
}

int checkforTsipouras(fixed_point_t RRprepre, fixed_point_t RRpre, fixed_point_t RRpos){

  RESULT.resultTsipouras = 0;  //apriori Normal

  //rule 1: Ventricular flutter/fibrillation beat classification
  if ((RRpre < TSI_fp_0_6) && ((fixed_mul32(TSI_fp_1_8,RRpre) >> SHIFT_BIT) < RRprepre)){                                                                 // C1
      RESULT.resultTsipouras = 1;
      
      //C2 need to be checked for pulse train                                                                     // C2

  }

  //rule2: Premature ventricular contractions    
  if ((((fixed_mul32(TSI_fp_1_15,RRpre) >> SHIFT_BIT) < RRprepre) && ((fixed_mul32(TSI_fp_1_15,RRpre) >> SHIFT_BIT) < RRpos)) ||                                                        // C3
      (abs(RRprepre - RRpre) < TSI_fp_0_3 && ((RRprepre < TSI_fp_0_8) || (RRpre < TSI_fp_0_8)) && (RRpos > (fixed_mul32(TSI_fp_0_6,(RRprepre + RRpre)) >> SHIFT_BIT))) ||   // C4
      (abs(RRpre - RRpos) < TSI_fp_0_3 && ((RRpre < TSI_fp_0_8) || (RRpos < TSI_fp_0_8)) && (RRprepre > (fixed_mul32(TSI_fp_0_6,(RRpre + RRpos)) >> SHIFT_BIT)))){        // C5
      RESULT.resultTsipouras = 1;
  }

  //rule3: 2degree heart block beats
  if (((TSI_fp_2_2 < RRpre) && (RRpre < TSI_fp_3_0)) && ((abs(RRprepre-RRpre) < TSI_fp_0_2) || (abs(RRpre-RRpos) < TSI_fp_0_2))){             // C6
      RESULT.resultTsipouras = 1;
  } 
  
  return RESULT.resultTsipouras;
}


int checkforRRIndex(fixed_point_t rrIndex){

  RESULT.result = 0;  //apriori Normal

    if (rrIndex < thr_rrIndex){
      RESULT.result = 1;  //apriori Normal
    }
  return RESULT.result;
}