//#include "ANN.h"
//#include "MATH_ANN.h"
//#include <stdlib.h>
//#include <stdio.h>
//
//#include <iostream>
//#include <fstream>
//#include <string>
//
//using namespace std;
//
//
////FC_LAYER
//
//ANN::ANN(){ //string Name){ // Constructor declaration
//  //ANN_Name = Name;
//  in_PCA = INPUT_LAYER(5);
//  lstm_layer_In = LSTM_LAYER(10,6,2);
//
//  in_RR = INPUT_LAYER(5);
//  fc_layer_In = FC_LAYER(2);
//  
//  concat_layer = CONCAT_LAYER();
//  fc_layer_Merge = FC_LAYER(10);
//  multiclass_layer = MULTICLASS_LAYER(2);
//
//}
//    
//void ANN::init_ANN(){
//      
//
//      
//}
//
//
//
//
//
//void read_weights() 
//{ 
//    ifstream myFile;
//    myFile.open("test.csv");
//
//    int _data[3][3];
//    int i = 0;
//    int j = 0;
//
//    while (myFile.good())
//    {
//        string line;
//        getline(myFile, line, ',');
//
//        _data[i][j] = stoi(line);
//        j++;
//        if (j > 3) {
//            i++;
//            j = 0;
//        }
//    }
//
//    for (int i = 0; i < 3; i++)
//    {
//        for (int j = 0; j < 3; j++)
//        {
//            cout << _data[i][j] << " ";
//        }
//        cout << endl;
//    }
//} 
