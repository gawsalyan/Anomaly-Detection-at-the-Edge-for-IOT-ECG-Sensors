#ifndef _GENFEATURE_H_
#define _GENFEATURE_H_

#define FULLSIZEINT uint16_t

typedef union FIXED3_13tag {
    FULLSIZEINT full;
    struct part3_13tag {
        FULLSIZEINT integer: 3;
        FULLSIZEINT fraction: FULLSIZEINT-3;
    } part;
} FIXED3_13;

#define FIXED3_13CONST(A,B) (FULLSIZEINT)((A<<13) + ((B + 0.00390625)*128))

  
uint16_t qrssum(uint16_t *Sig, int R, int band);    
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
              
uint16_t vvPeakSTD(uint16_t *Sig, int R, int band);
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

// Math ........................................................................................................................

  // Normalize the n element vector
  void normalize(uint16_t maapstd[], uint16_t *Sig, int n);

  //Map Standard Deviation of the n element vector
  void mapstd(uint16_t maapstd[], uint16_t *Sig, int n);

  // Standard deviation given the mean of n element vector
  uint16_t std_mean(uint16_t *Sig, int n, uint16_t mean);

  // Standard deviation of n element vector
  uint16_t std(uint16_t *Sig, int n);

  // Mean of n element vector
  uint16_t mean(uint16_t *Sig, int n);

  //Sum of n element vector
  uint16_t Sum(uint16_t *a, int n);

  //Square: Power of 2
  #define Square(x) ((x)*(x))

  //max in the n element vector
  uint16_t maxV(uint16_t *Sig, int n);

  //min in the n element vector
  uint16_t minV(uint16_t *Sig, int n);

#endif