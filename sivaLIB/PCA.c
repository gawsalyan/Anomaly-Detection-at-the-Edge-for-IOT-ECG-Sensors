#include "PCA.h"
#include "FixedPoint.h"
#include <stdio.h>
#include <math.h>
//#include <stdint.h>

//Define the canonical Principla Component
//const fixed_point_t PC[175] = {0xFEE1, 0xFEB8, 0xFEE1, 0xFEB8, 0xFEE1, 0xFEE1, 0xFEE1, 0xFEE1, 0xFEE1, 0xFEE1, 
//            0xFF0A, 0xFF0A, 0xFF33, 0xFF33, 0xFF33, 0xFF5C, 0xFF5C, 0xFF85, 0xFF85, 0xFFAE, 0xFFAE, 0xFFAE, 0xFFAE, 0xFFAE, 0xFFAE, 
//            0xFFD7, 0xFFD7, 0xFFD7, 0xFFAE, 0xFFAE, 0xFF85, 0xFF85, 0xFF5C, 0xFF5C, 0xFF33, 0xFF33, 0xFF0A, 0xFF0A, 0xFEE1, 0xFEE1, 
//            0xFEB8, 0xFEB8, 0xFE8F, 0xFE8F, 0xFE8F, 0xFE8F, 0xFE8F, 0xFE8F, 0xFE8F, 0xFEB8, 0xFE8F, 0xFE8F, 0xFE8F, 0xFE8F, 0xFEB8, 
//            0xFF33, 0x0029, 0x01EC, 0x04A4, 0x0800, 0x0BAE, 0x0E8F, 0x1000, 0x0F5C, 0x0C52, 0x07AE, 0x02E1, 0xFF5C, 0xFDC3, 0xFDEC, 
//            0xFEB8, 0xFF33, 0xFF33, 0xFF0A, 0xFEB8, 0xFE8F, 0xFE66, 0xFE66, 0xFE66, 0xFE66, 0xFE66, 0xFE66, 0xFE66, 0xFE66, 0xFE66, 
//            0xFE8F, 0xFE8F, 0xFE8F, 0xFE8F, 0xFE8F, 0xFE8F, 0xFE8F, 0xFE8F, 0xFEB8, 0xFEB8, 0xFEB8, 0xFEB8, 0xFEB8, 0xFEE1, 0xFEE1, 
//            0xFEE1, 0xFEE1, 0xFEE1, 0xFF0A, 0xFF0A, 0xFF0A, 0xFF33, 0xFF33, 0xFF5C, 0xFF5C, 0xFF85, 0xFF85, 0xFFAE, 0xFFD7, 0xFFD7, 
//            0x0000, 0x0029, 0x0029, 0x0052, 0x007B, 0x007B, 0x00A4, 0x00A4, 0x00CD, 0x00CD, 0x00F6, 0x00F6, 0x00F6, 0x011F, 0x011F, 
//            0x011F, 0x011F, 0x011F, 0x011F, 0x011F, 0x011F, 0x011F, 0x011F, 0x011F, 0x011F, 0x00F6, 0x00F6, 0x00F6, 0x00F6, 0x00CD, 
//            0x00CD, 0x00A4, 0x00A4, 0x007B, 0x0052, 0x0052, 0x0029, 0x0000, 0x0000, 0xFFD7, 0xFFAE, 0xFFAE, 0xFF85, 0xFF85, 0xFF5C, 
//            0xFF5C, 0xFF33, 0xFF33, 0xFF33, 0xFF33, 0xFF0A, 0xFF0A, 0xFF0A, 0xFF0A, 0xFF0A, 0xFF0A, 0xFF0A, 0xFF0A, 0xFF0A, 0xFF0A};

const fixed_point_t PC_N[175] = {-214,-209,-202,-194,-187,-177,-163,-148,-134,-119,-98,-77,-57,-36,-12,10,25,35,43,45,41,33,27,21,14,0,-14,-29,-50,-77,-102,-120,-136,-155,-174,-191,-208,-228,-249,-263,-270,-277,-281,-279,-273,-270,-264,-251,-231,-205,-167,-103,-19,76,223,505,959,1573,2313,3105,3792,4096,3740,2782,1540,308,-571,-935,-921,-759,-610,-527,-503,-504,-508,-506,-497,-486,-477,-470,-463,-452,-440,-431,-421,-409,-398,-391,-385,-372,-358,-345,-333,-317,-297,-280,-263,-244,-223,-205,-189,-173,-153,-134,-119,-103,-87,-71,-61,-52,-41,-30,-25,-20,-11,-4,-2,-1,2,8,12,12,15,21,25,27,31,38,44,48,53,60,66,71,74,80,85,84,84,86,86,80,69,60,53,42,31,20,11,0,-16,-31,-42,-55,-70,-83,-94,-106,-120,-131,-139,-146,-154,-162,-166,-169,-175,-181,-182,-180,-179,-180,-178,-174,-173};
const fixed_point_t PC_L[175] = {-71,-51,-33,-17,-1,19,47,81,111,132,140,141,139,145,164,192,222,243,246,232,205,171,140,119,105,94,86,81,74,63,48,28,6,-13,-26,-31,-30,-28,-31,-48,-92,-170,-277,-385,-454,-462,-433,-422,-466,-551,-611,-538,-221,362,1089,1751,2217,2553,2925,3411,3884,4096,3828,3077,2071,1132,438,-66,-498,-893,-1208,-1399,-1470,-1469,-1437,-1383,-1301,-1178,-1002,-792,-588,-430,-328,-275,-250,-229,-205,-183,-167,-155,-145,-136,-127,-119,-113,-111,-114,-117,-122,-126,-133,-141,-151,-164,-180,-198,-218,-239,-259,-278,-297,-316,-338,-361,-385,-407,-426,-442,-455,-466,-475,-484,-488,-487,-479,-468,-453,-433,-412,-391,-368,-343,-317,-289,-261,-232,-207,-183,-161,-138,-116,-92,-68,-43,-18,3,20,34,47,61,74,86,94,98,97,94,91,88,83,74,65,55,42,26,8,-9,-26,-39,-51,-62,-74,-89,-104,-116,-123};
const fixed_point_t PC_R[175] = {-212,-222,-235,-242,-240,-239,-242,-243,-236,-232,-230,-231,-226,-222,-223,-225,-223,-220,-222,-228,-227,-223,-227,-234,-237,-234,-234,-239,-239,-235,-233,-235,-237,-236,-232,-237,-242,-238,-232,-229,-227,-216,-204,-190,-172,-149,-129,-133,-186,-271,-360,-491,-665,-840,-1091,-1431,-1700,-1955,-2391,-3059,-3795,-4096,-3946,-3740,-3462,-3099,-2709,-2306,-1947,-1629,-1346,-1103,-849,-551,-259,-35,94,149,169,182,188,186,181,183,188,187,188,195,206,215,221,234,253,269,282,303,328,348,366,388,415,442,465,490,521,551,574,598,629,662,685,707,737,768,792,812,837,864,886,901,921,944,959,970,984,1001,1013,1019,1025,1033,1038,1035,1030,1028,1025,1014,999,990,982,965,943,926,911,890,864,842,825,804,779,756,737,716,691,668,651,633,609,588,573,558,537,516,499,485,465,446,432,419,400,378,363,350,332,311,296};
const fixed_point_t PC_S[175] = {557,524,485,449,425,397,366,331,308,288,259,228,213,198,171,140,128,127,108,80,73,72,61,46,33,35,28,11,2,1,-4,-15,-27,-27,-28,-37,-51,-53,-49,-56,-61,-55,-41,-30,-34,-42,-59,-104,-184,-257,-314,-401,-512,-635,-814,-1066,-1395,-1787,-2275,-2960,-3734,-4096,-3819,-3245,-2604,-1988,-1599,-1431,-1292,-1130,-934,-723,-539,-406,-270,-115,10,79,126,158,175,183,202,235,270,293,317,361,411,447,482,531,590,641,681,736,793,838,868,910,960,996,1018,1034,1058,1071,1073,1070,1069,1065,1049,1034,1024,1005,980,944,917,895,860,817,784,755,716,672,632,595,554,512,470,442,408,369,332,308,284,251,223,203,189,162,135,115,105,83,58,42,29,8,-22,-52,-76,-107,-143,-172,-183,-188,-199,-200,-184,-165,-153,-137,-111,-84,-83,-102,-120,-133,-154,-186,-216,-247,-300,-380,-461};
const fixed_point_t PC_V[175] = {-246,-239,-241,-233,-213,-199,-199,-195,-174,-157,-155,-150,-131,-112,-107,-104,-89,-73,-69,-70,-61,-45,-42,-52,-54,-45,-47,-59,-66,-62,-63,-80,-92,-93,-97,-115,-132,-134,-137,-154,-176,-187,-193,-210,-223,-217,-199,-198,-228,-282,-371,-522,-753,-1051,-1408,-1820,-2282,-2743,-3177,-3593,-3947,-4096,-3976,-3733,-3480,-3222,-2961,-2717,-2497,-2298,-2106,-1925,-1766,-1592,-1360,-1075,-775,-502,-272,-87,49,143,221,300,365,404,439,486,533,565,599,645,692,727,755,797,849,888,922,970,1029,1082,1128,1184,1252,1316,1367,1422,1488,1547,1588,1625,1670,1709,1729,1737,1751,1761,1749,1724,1706,1689,1655,1602,1553,1508,1448,1373,1302,1242,1177,1097,1022,960,896,819,744,686,630,559,488,435,390,333,278,244,222,192,159,138,131,121,104,95,98,93,78,68,71,72,57,44,46,48,38,27,27,28,18,3,3,8,1,-13,-18};
const fixed_point_t PC_F[175] = {-215,-217,-215,-214,-218,-218,-211,-213,-214,-213,-209,-206,-206,-204,-199,-192,-182,-170,-150,-130,-112,-93,-65,-36,-14,0,7,17,15,2,-13,-30,-56,-88,-122,-150,-174,-190,-188,-166,-130,-78,-12,77,182,292,404,518,643,767,892,1024,1181,1353,1517,1732,2069,2511,2994,3466,3883,4096,3775,2882,1850,996,428,72,-157,-278,-348,-417,-483,-535,-578,-615,-636,-644,-651,-662,-676,-684,-692,-706,-717,-725,-732,-744,-757,-768,-781,-796,-818,-832,-839,-849,-864,-878,-884,-892,-897,-898,-886,-877,-867,-853,-839,-822,-804,-781,-753,-727,-699,-672,-639,-597,-562,-536,-505,-472,-451,-437,-427,-420,-417,-423,-430,-434,-437,-444,-447,-448,-448,-451,-452,-446,-439,-438,-436,-424,-413,-404,-392,-380,-371,-366,-360,-349,-330,-315,-302,-280,-250,-215,-178,-132,-66,11,86,175,281,394,508,631,766,897,1021,1135,1248,1354,1446,1511,1563,1601,1618};

//Rules
const fixed_point_t PC[175] = {214, 209, 202, 194, 187, 177, 163, 148, 134, 119, 98, 77, 57, 36, 12, -10, -25, -35, -43, -45, -41, -33, -27, -21, -14, 0, 14, 29, 50, 77, 102, 120, 136, 155, 174, 191, 208, 228, 249, 263, 270, 277, 281, 279, 273, 270, 264, 251, 231, 205, 167, 103, 19, -76, -223, -505, -959, -1573, -2313, -3105, -3792, -4096, -3740, -2782, -1540, -308, 571, 935, 921, 759, 610, 527, 503, 504, 508, 506, 497, 486, 477, 470, 463, 452, 440, 431, 421, 409, 398, 391, 385, 372, 358, 345, 333, 317, 297, 280, 263, 244, 223, 205, 189, 173, 153, 134, 119, 103, 87, 71, 61, 52, 41, 30, 25, 20, 11, 4, 2, 1, -2, -8, -12, -12, -15, -21, -25, -27, -31, -38, -44, -48, -53, -60, -66, -71, -74, -80, -85, -84, -84, -86, -86, -80, -69, -60, -53, -42, -31, -20, -11, 0, 16, 31, 42, 55, 70, 83, 94, 106, 120, 131, 139, 146, 154, 162, 166, 169, 175, 181, 182, 180, 179, 180, 178, 174, 173};


fixed_point_t findPCAcoeff(fixed_point_t *Sig, int n){
    
    //lefT = round((89/Fs)*Fs_Req); righT = round((162/Fs)*Fs_Req); 
    //uint8_t lenSig = round(0.25*Fs) + round(0.45 *Fs); 
    //uint8_t lenSig = sizeof(Sig)/sizeof(float);

    //noSample = size(S);
    //[lenSB, nPCA] = size(SB); 

//    if (lenSig != 175){
//        printf("lenSig: %d, No signal length match between PC and Signal", lenSig);
//        return 0;
//    }else{
//      
//      printf("lenSig: %d, No signal length match between PC and Signal\n", lenSig);

      //Find the mean vector:
      fixed_point_t meanSig = mean(Sig,n); 
      //printf("mean of the Sig: %.3f\n", meanSig);


      //Represent each sample i.e., image as a linear combination of basis vectors.
      fixed_point_32t PCAcoeff = 0;
      for (int i=0; i<n; i++){
        PCAcoeff += (fixed_mul32((*(Sig+i) - meanSig), PC[i]));
      }  
      
      return ( PCAcoeff >> SHIFT_BIT);

//    }
}

void findPCAcoeffs(fixed_point_t A[], fixed_point_t *Sig, int n){
    
      //Find the mean vector:
      fixed_point_t meanSig = mean(Sig,n); 
      //printf("mean of the Sig: %.3f\n", meanSig);

      fixed_point_t diffMeanSig = 0;

      //Represent each sample i.e., image as a linear combination of basis vectors.
      fixed_point_32t PCAcoeff_N = 0;
      fixed_point_32t PCAcoeff_L = 0;
      fixed_point_32t PCAcoeff_R = 0;
      fixed_point_32t PCAcoeff_S = 0;
      fixed_point_32t PCAcoeff_V = 0;
      fixed_point_32t PCAcoeff_F = 0;
      for (int i=0; i<n; i++){
        diffMeanSig =  *(Sig+i) - meanSig;
        PCAcoeff_N += (fixed_mul32(diffMeanSig, PC_N[i]));
        PCAcoeff_L += (fixed_mul32(diffMeanSig, PC_L[i]));
        PCAcoeff_R += (fixed_mul32(diffMeanSig, PC_R[i]));
        PCAcoeff_S += (fixed_mul32(diffMeanSig, PC_S[i]));
        PCAcoeff_V += (fixed_mul32(diffMeanSig, PC_V[i]));
        PCAcoeff_F += (fixed_mul32(diffMeanSig, PC_F[i]));
      }  
      
      A[0] =  PCAcoeff_N >> SHIFT_BIT;
      A[1] =  PCAcoeff_L >> SHIFT_BIT;
      A[2] =  PCAcoeff_R >> SHIFT_BIT;
      A[3] =  PCAcoeff_S >> SHIFT_BIT;
      A[4] =  PCAcoeff_V >> SHIFT_BIT;
      A[5] =  PCAcoeff_F >> SHIFT_BIT;

}

fixed_point_t PCAstd(fixed_point_t *PCA, int n){
  
  fixed_point_32t SD = 0;
  int sum = Sum(PCA,n);
  fixed_point_t mean = sum/n;
  
  //printf("SUM: %d, Mean: %d \n", sum, mean); 

  for (int i=0; i<n; i++){   
    fixed_point_t temp = (PCA[i] - mean);
    fixed_point_32t temp32 = fixed_mul32(temp,temp);
    SD = SD + temp32;
    //printf("RR_%d: %.2f %.4f\n",i+1, PCA[i], temp); 
  }
  //printf("SD: %d \n", SD); 
  SD = sqrt(SD/n); 
  //printf("SD: %d \n", SD); 
  SD = SD << (FIXED_POINT_FRACTIONAL_32BITS / 2);
  //printf("SD: %d \n", SD); 
  return (SD >> SHIFT_BIT);
}






