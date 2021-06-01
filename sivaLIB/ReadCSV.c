#include "ReadCSV.h"

void readCSV(char* path, fixed_point_ann_t *numberArray, int row, int column){
    FILE *fp;
    fp = fopen(path, "r");
    rewind(fp);
    
    //read file into array
    //int *numberArray[column];//(int **)calloc(row*column, sizeof(int **));  
    
    // Return if could not open file 
    if (fp == NULL){
        printf("Error Reading File\n");
        exit (0);
        }
    
    int i_row = 0;
    int j_col = 0;
    int val = 0;
    int sign = 1;

    do
    { 
        
        // Taking input single character at a time 
        char c = fgetc(fp); 
  
        // Checking for end of file 
        if (feof(fp)) 
            break ; 
  
        //printf("ch:%c, i:%d, j:%d\n", c, i_row,j_col); 
        
        if(c == '\n') {
          //*(*(numberArray + i_row) + j_col) = val;
          *((numberArray + i_row*column) + j_col) = sign*val;
          i_row = i_row + 1;
          j_col = 0;
          val = 0;
          sign = 1;
        }
        if(c==','){
           //*(*(numberArray + i_row) + j_col) = val;
          *((numberArray + i_row*column) + j_col) = sign*val;
          j_col = j_col + 1;
          val = 0;
          sign = 1;
        }
        if(c=='-'){
          sign = -1;
        }
        if ((c >= 48) && (c <= 57)) {
          val = val*10 + c - 48;
        }

    }  while(1); 
    
//    printf("MAtrix is:\n");
//    for (int i = 0; i < row; i++){
//      for (int j = 0; j < column; j++){
//        printf("%d,",  *((numberArray + i*column) + j)); //*((numberArray + i*column) + j)
//      }
//      printf("\n");
//    }
    
    fclose(fp);
    return &numberArray;
}