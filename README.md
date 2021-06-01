The program is developed using Nordic Semiconductor's software development kit for the nRF52 series SoC nRF5 SDK. To run the program, download the nRF5 SDK, extract it and copy the repository (extracted) in nRF5_SDK_15.3.0_59ac345/examples/ble_peripheral such that the path to *.emProject becomes nRF5_SDK_.../examples/ble_peripheral/Anomaly-Detection-at-the-Edge-for_IOT-ECG-Sensors/pca10040/s132/ses/*.emProject. Open the .emProject file in Segger Embedded Studio for ARM.

Nordic Semiconductor's nRF52832 Serial peripheral interface master with EasyDMA (SPIM) list feature is used to improve power efficency. The algorithm is implemented in its true spirit and many smart choices are made to make it efficient such as:

- avoiding literal multiplication and division as much as possible by using bit shifting
- avoiding literal division for divisors which are not powers of 2 by rounding them to nearest power of 2 number where possible and then using bit shifting
- avoiding literal division when the divisors were initially changing but are then fixed and become powers of 2 by using bit shifting
- avoiding 'for' loop for moving integration windows and similar summations by subtracting and adding single single elements at both ends of the window
- avoiding lot of calculations per iteration needed for circular arrays by separating the code for initial few iterations which involve calculations than the rest of the iterations

# Preprocessor
The module extract the data buffer and using the peak information identify beat window and feed it for the next steps.
- To extract features and produce statistical information
- To feed as an input feature vector to ANNet or Rule based algorithm

# Fixed Point LIBRARY  
This is based on the previous implementation of the fixed point machine learning toolbox in MATLAB.
Only the forward function is implemented for the ANN library.

## ANNet

Proposed a novel binary Electrocardiogram (ECG) classification neural network for continuous cardiac observation suitable for a wearable platform to diagnose cardiovascular diseases at their early stages with low computational complexity and power consumption. The presented solution utilizes a novel architecture consisting of Long Short Term Memory (LSTM) cells and Multi-Layer Perceptrons (MLP). The experimental evaluation of the network outperforms most previously reported methodologies in the literature both in terms of accuracy, complexity and power consumption. It achieves very high accuracy in routine clinical records while showing an overall accuracy of ~97% across the ECG records in the MIT-BIH arrhythmia database.

The algorithm was mapped to a fixed point implementation, retained and ported to an embedded platform. In Laboratory testing the overall system design was demonstrated to achieve significant power consumption savings when used to gate the wireless transmission of ECG signals to only broadcast those beats deemed to be anomalous. In addition our design retains the advantages of having stand-alone continuous cardiac classifications without the need for always-on wireless connectivity making our proposed system very suitable for wearable platforms.

## RULES

Electrocardiogram (ECG) analysis is efficient in diagnosing cardiovascular diseases which are the leading cause of global deaths. However, it is very difficult to analyze and monitor  continuously, due to its nature of complexity and inter person variation. On the other hand, heart attacks can cause strokes or fatal conditions in individuals. Such adverse scenarios can be mitigated by early anomaly detection using Point of Care personal ECG monitors. The goal of this study is to develop a power-efficient anomaly detection algorithm with crisp rules without enigmatic black-box models to identify anomalies at the edge. In this paper, we have introduced a novel rule-based algorithm to detect abnormal heart beats based on a set of statistical and morphological features. The rule base is derived from the linear combination of the highest contributing features identified with the gradient analysis of a trained neural network. The proposed set of rules are an explainable model which achieves very high VEB (~91%) and SVEB (~98%) test sensitivity as guided by Association for the Advancement of Medical Instrumentation standards and results in significant power savings.

# Others
## Pan-Tompkins-algorithm-with-BLE 

This is an efficient implementation of Pan-Tompkins algorithm by Adnan Ashrof on Nordic Semiconductor's nRF52832-QFAA interfaced with Texas Instruments' ADS1292RIPBS. Nordic Semiconductor's nRF52 Development Kit nRF52-DK is used along with ProtoCentral's ADS1292R ECG Respiration Breakout Kit. The ECG singals are generated through CONTEC's MS400 Multiparameter Simulator.
