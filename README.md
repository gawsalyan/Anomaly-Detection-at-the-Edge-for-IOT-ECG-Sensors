# Pan-Tompkins-algorithm-with-BLE

This is an efficient implementation of Pan-Tompkins algorithm on Nordic Semiconductor's nRF52832-QFAA interfaced with Texas Instruments' ADS1292RIPBS. Nordic Semiconductor's nRF52 Development Kit nRF52-DK is used along with ProtoCentral's ADS1292R ECG Respiration Breakout Kit. The ECG singals are generated through CONTEC's MS400 Multiparameter Simulator.

The program is developed using Nordic Semiconductor's software development kit for the nRF52 series and the nRF51 series SoC nRF5 SDK. To run the program, download the nRF5 SDK, extract it and copy the repository (extracted) in nRF5_SDK_15.3.0_59ac345/examples/ble_peripheral such that the path to Pan_Tompkins_algorithm_with_BLE_pca10040.emProject becomes nRF5_SDK_15.3.0_59ac345/examples/ble_peripheral/Pan-Tompkins-algorithm-master/pca10040/s132/ses/Pan_Tompkins_algorithm_with_BLE_pca10040.emProject. Open the .emProject file in Segger Embedded Studio for ARM.

Nordic Semiconductor's nRF52832 Serial peripheral interface master with EasyDMA (SPIM) list feature is used to improve power efficency. The algorithm is implemented in its true spirit and many smart choices are made to make it efficient such as:

- avoiding literal multiplication and division as much as possible by using bit shifting
- avoiding literal division for divisors which are not powers of 2 by rounding them to nearest power of 2 number where possible and then using bit shifting
- avoiding literal division when the divisors were initially changing but are then fixed and become powers of 2 by using bit shifting
- avoiding 'for' loop for moving integration windows and similar summations by subtracting and adding single single elements at both ends of the window
- avoiding lot of calculations per iteration needed for circular arrays by separating the code for initial few iterations which involve calculations than the rest of the iterations

The mod operators (%) in the program will be more efficiently implemented if the ARRAY_LIST_SIZE is power of 2. The compiler automatically replaces the division and multiplication involved in the modulo operation by simple logical operation.

The algorithm consumes an average of 19 uA current with ADS1292R ADC channel rate of 250 SPS and SPIM array list size of 1000. The current consumption is measured using Nordic Semiconductor's Power Profiler Kit nRF6707.
