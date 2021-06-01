/**
 * Copyright (c) 2014 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 * @defgroup uart_example_main main.c
 * @{
 * @ingroup uart_example
 * @brief UART Example Application main file.
 *
 * This file contains the source code for a sample application using UART.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>

#include "FixedPoint.h"
#include "HRV.h"
#include "Rules.h"
#include "LAYERS.h"

#include "app_uart.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf.h"
#include "bsp.h"
#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif


//#define ENABLE_LOOPBACK_TEST  /**< if defined, then this example will be a loopback test, which means that TX should be connected to RX to get data loopback. */

#define MAX_TEST_DATA_BYTES     (15U)                /**< max number of test bytes to be used for tx and rx. */
#define UART_TX_BUF_SIZE 2048                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 2048                        /**< UART RX buffer size. */

#define BYTE_LEN 8
#define DATA_BUFFER_SIZE 1000
#define R_POS_SIZE 11
#define PCA_SIZE 10
#define BEAT_LEN 175


//DEFINE THE DATA BUFFER
  int sampcount = 0;
  fixed_point_t Data_Buf[DATA_BUFFER_SIZE];
  int beatcount = 0;
  int Rpos[R_POS_SIZE];

//DEFINE OUTPUT STRUCTURE
struct HRVmetric{ 
    fixed_point_t wSDNN;              // weighted SDNN
    fixed_point_t SDNN;               // SDNN
    fixed_point_t rrIndex;            // RR Index (2 * (Rpre - Rprepre)/ (Rpre - Rprepre))
  }; 

struct{
    fixed_point_t RR[R_POS_SIZE];     // RR interval in duration
    fixed_point_t PCA[PCA_SIZE];      // PCA of previous 10 beats
    fixed_point_32t qrsSum;           // QRS sum around the band
    fixed_point_t prevvSTD;           // Max - Min V change of previous beat
    fixed_point_t vvSTD;              // Max - Min V change
    fixed_point_t PCAcoef;            // PCA coefficient with respect to Normal
    fixed_point_t PCAstd;             // PCA standard deviation
    struct HRVmetric HRV;             // HRV structure
    unsigned int Result : 1;          // Annotation
  } BEAT;



void uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}


#ifdef ENABLE_LOOPBACK_TEST
/* Use flow control in loopback test. */
#define UART_HWFC APP_UART_FLOW_CONTROL_ENABLED

/** @brief Function for setting the @ref ERROR_PIN high, and then enter an infinite loop.
 */
static void show_error(void)
{

    bsp_board_leds_on();
    while (true)
    {
        // Do nothing.
    }
}


/** @brief Function for testing UART loop back.
 *  @details Transmitts one character at a time to check if the data received from the loopback is same as the transmitted data.
 *  @note  @ref TX_PIN_NUMBER must be connected to @ref RX_PIN_NUMBER)
 */
static void uart_loopback_test()
{
    uint8_t * tx_data = (uint8_t *)("\r\nLOOPBACK_TEST\r\n");
    uint8_t   rx_data;

    // Start sending one byte and see if you get the same
    for (uint32_t i = 0; i < MAX_TEST_DATA_BYTES; i++)
    {
        uint32_t err_code;
        while (app_uart_put(tx_data[i]) != NRF_SUCCESS);

        nrf_delay_ms(10);
        err_code = app_uart_get(&rx_data);

        if ((rx_data != tx_data[i]) || (err_code != NRF_SUCCESS))
        {
            show_error();
        }
    }
    return;
}
#else
/* When UART is used for communication with the host do not use flow control.*/
#define UART_HWFC APP_UART_FLOW_CONTROL_DISABLED
#endif


/**
 * @brief Function for main application entry.
 */
int main(void)
{
    uint32_t err_code;
    

 //Sample Beat
    fixed_point_t Signal[BEAT_LEN];
    memset(BEAT.RR, 0x00, R_POS_SIZE);
    memset(BEAT.PCA, 0x00, PCA_SIZE);
    //BEAT.RR = {0,0,0,0,0,0,0,0,0,0,0};

//BEAT REGION
  //Sample Frequency
  const int Fs = 250;
  const int SamplingShift = 8; //2^8 is close to the Fs 250 Hz, hence shifting by 8 can be treated as in time
  //Sample length of single beat 
  const int lenSignal = round(0.7*Fs);
  //R peak location and left and right bands
  const int RLoc = round(0.25*Fs)-1, leftBand = RLoc, rightBand = (lenSignal-RLoc), band = round(0.056*Fs);

   

    bsp_board_init(BSP_INIT_LEDS);

    const app_uart_comm_params_t comm_params =
      {
          RX_PIN_NUMBER,
          TX_PIN_NUMBER,
          RTS_PIN_NUMBER,
          CTS_PIN_NUMBER,
          UART_HWFC,
          false,
#if defined (UART_PRESENT)
          NRF_UART_BAUDRATE_115200
#else
          NRF_UARTE_BAUDRATE_115200
#endif
      };

    APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_error_handle,
                         APP_IRQ_PRIORITY_LOWEST,
                         err_code);

    APP_ERROR_CHECK(err_code);

#ifndef ENABLE_LOOPBACK_TEST
    printf("\r\nUART example started.\r\n"); 
    
    //For Test ...
    //fixed_point_t RR[11] = {3152, 3168, 3152, 3264, 2624, 3968, 3376, 3248, 3168, 3072, 3360};
    //BEAT.HRV.rrIndex = HRV_RRIndex(&RR[8]);
    //printf("%d \n", BEAT.HRV.rrIndex);

    while (true)
    {
       
        
        FC_LAYER fc_layer1;
        


        uint8_t cr;
        while (app_uart_get(&cr) != NRF_SUCCESS);
        //while (app_uart_put(cr) != NRF_SUCCESS);
        
        if (cr == '\n'){
            
            while (app_uart_get(&cr) != NRF_SUCCESS);
            if (cr == 2){
                beatcount = 0; 
                sampcount = 0;
                memset(BEAT.RR, 0x00, R_POS_SIZE);
                memset(BEAT.PCA, 0x00, PCA_SIZE);
                memset(Data_Buf, 0x00, DATA_BUFFER_SIZE);
                //printf("New set of data identified\n");
            }else if (cr == 0){
                //do nothing
            }else if (cr == 1) {
                Rpos[beatcount % R_POS_SIZE] = sampcount;
                  for (int i = 0; i < R_POS_SIZE - 1; i++){  
                    BEAT.RR[i]  = BEAT.RR[i + 1];
                    //printf("%d, ", BEAT.RR[i]);
                    BEAT.PCA[i] = BEAT.PCA[i + 1];
                    //printf("%d, ", BEAT.PCA[i]);
                  } 
                  BEAT.RR[R_POS_SIZE-1] = (sampcount - Rpos[(beatcount-1) % R_POS_SIZE]) << (FIXED_POINT_FRACTIONAL_BITS - SamplingShift); 
                  //printf("%d\n", BEAT.RR[R_POS_SIZE-1]);                    


                if (beatcount > 0){
                    int tempbeatcount = beatcount - 1;
                    //printf("Beat No.%d on sample %d\n",tempbeatcount, sampcount);
                    int loc = (((Rpos[tempbeatcount % R_POS_SIZE] % DATA_BUFFER_SIZE) - leftBand) > 0) ? 
                                  ((Rpos[tempbeatcount % R_POS_SIZE] % DATA_BUFFER_SIZE) - leftBand) : 
                                  DATA_BUFFER_SIZE + ((Rpos[tempbeatcount % R_POS_SIZE] % DATA_BUFFER_SIZE) - leftBand);
                    for (int i=0; i < lenSignal; i++){
                      int temploc = ((loc + i) < 1000) ? (loc + i) : (loc + i - 1000);
                      Signal[i] = Data_Buf[temploc];
                      //printf("%d, ", Signal[i]);
                    }
                    //printf("\n");

                    //normalise the input signal 
                        fixed_point_t maapstd[BEAT_LEN];
                        normalize(maapstd, Signal, lenSignal);   
                        //printf("mapstd Signal 1: %d, 2: %d\n", *(maapstd+17), *(maapstd+20)); 
                        for (int i = 0; i<BEAT_LEN-1; i++){
                          //printf("%d, ", maapstd[i]);
                        }
                        //printf("%d\n", maapstd[BEAT_LEN-1]);

                    
                     //QRS Sum 0.05*Fs:0.05*Fs                     
                      //BEAT.qrsSum = qrssum(maapstd, RLoc, band);
                      //printf("qrsSum : %d\n", BEAT.qrsSum); 

                    //SDNN: RR standard deviation
                      //float rrSTD;
                      //rrSTD = HRV_SDNN(RR_INT,10);
                      //printf("rrTSD/SDNN : %.4f\n", rrSTD); 
  
                    //VVstd
                      //BEAT.prevvSTD = BEAT.vvSTD;
                      //BEAT.vvSTD = vvPeakSTD(maapstd, RLoc, band);
                      //printf("vvSTD : %d\n", BEAT.vvSTD); 

                    //PCA
                      //BEAT.PCAcoef = findPCAcoeff(maapstd, BEAT_LEN);
                      //printf("PCAcoeff: %d\n", BEAT.PCAcoef); 
                      //BEAT.PCA[PCA_SIZE-1] = BEAT.PCAcoef;      
                      //printf("%d\n", BEAT.PCA[PCA_SIZE-1]);          
                      //BEAT.PCAstd = PCAstd(BEAT.PCA, PCA_SIZE);                      

                    //if (beatcount > 10){           
                      //HRV Test
                        //BEAT.HRV.wSDNN = weightedSDNN(10,BEAT.RR,9,11);
                        //BEAT.HRV.SDNN = HRV_SDNN(BEAT.RR,11);   
                        //BEAT.HRV.rrIndex = HRV_RRIndex(&BEAT.RR[9]);
                        //BEAT.Result = checkforArrhythmia(BEAT.HRV.wSDNN,BEAT.HRV.rrIndex, &BEAT.RR[8], BEAT.qrsSum, BEAT.PCAcoef, BEAT.vvSTD, BEAT.prevvSTD, BEAT.HRV.SDNN, BEAT.PCAstd);
                        //BEAT.Result = check_wSDNN_RRindex(BEAT.HRV.wSDNN,BEAT.HRV.rrIndex);
                        //printf("%d, %d, %d, %d, %d, %d, %d, %d, %d,", beatcount, BEAT.HRV.wSDNN, BEAT.HRV.SDNN, BEAT.HRV.rrIndex, BEAT.qrsSum, BEAT.vvSTD, BEAT.PCAcoef, BEAT.PCAstd, BEAT.Result);                   
                    
                        //BEAT.Result = checkforTsipouras(BEAT.RR[8], BEAT.RR[9], BEAT.RR[10]);
                        //printf("%d, ",BEAT.Result);

                        //BEAT.Result = checkforExarchos(BEAT.RR[8], BEAT.RR[9], BEAT.RR[10]);
                        //printf("%d\n",BEAT.Result);
                    //}

                }else{
                  
                  for (int i = R_POS_SIZE - 2; i >= 0; i--){  
                    BEAT.RR[i]  = BEAT.RR[i + 1];
                    //printf("%d, ", BEAT.RR[i]);
                  } 
                
                }

                beatcount++;               
                
            }            
            
          //16bits data: 1st Byte | 2nd Byte   
            while (app_uart_get(&cr) != NRF_SUCCESS);
            //printf("The fi values: %d\n", cr);
            Data_Buf[sampcount % DATA_BUFFER_SIZE] = cr << BYTE_LEN;
            while (app_uart_get(&cr) != NRF_SUCCESS);
            //printf("The fi values: %d\n", cr);
            Data_Buf[sampcount % DATA_BUFFER_SIZE] = Data_Buf[sampcount % DATA_BUFFER_SIZE] | cr;
            
            sampcount++;
        }
        
        if (cr == 'q' || cr == 'Q')
        {
            //printf(" \r\nExit!: %d\r\n", sampcount);
            //while (true)
            //{
                // Do nothing.
            //}
        }
    }
#else

    // This part of the example is just for testing the loopback .
    while (true)
    {
        uart_loopback_test();
    }
#endif
}


/** @} */
