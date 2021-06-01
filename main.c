/**
 * Copyright (c) 2015 - 2019, Nordic Semiconductor ASA
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

/* Settings: Do not change below */
#define SIVA_BLE
//#define ADNAN_PT_ALGO
/*********/

/* Setting Region */
//#define SIVA_SERIAL
#define ADNAN_SPIM
#ifdef SIVA_BLE
#define SIVA_BLE_AD
#endif

//#define SIVA_ANN
#define SIVA_RULE
/*********/



#if defined(SIVA_ANN) || defined(SIVA_RULE)
#ifdef SIVA_BLE_AD
#ifndef BLE_ALERT_ECGSAMPLE
//#define BLE_ALERT_1BYTE
#endif
#ifndef BLE_ALERT_1BYTE
#define BLE_ALERT_ECGSAMPLE
#endif
#endif
#endif

#if defined(SIVA_SERIAL) || defined(ADNAN_SPIM)
#if defined(SIVA_BLE) && defined(SIVA_BLE_AD)
#ifndef SIVA_ANN
#ifndef SIVA_RULE
#define BLE_ALL_SAMPLES
#endif
#endif
#endif
#endif

//#include "app_util_platform.h"
#include "nrf_delay.h"
#include "boards.h"
//#include "app_error.h"
#include <string.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrfx_timer.h"
#include "nrfx_gpiote.h"


#include <stdint.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "app_timer.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp_btn_ble.h"
#include "nrf_pwr_mgmt.h"
#include "ble_lbs.h"

#ifdef ADNAN_SPIM
#include "nrfx_spim.h"
#include "nrfx_ppi.h"
#include "ads1292r.h"
#endif

#ifdef SIVA_ANN
#include "FixedPoint.h"
#include "FixedPointANN.h"
#include "HRV.h"
#include "Rules.h"
#include "ANN.h"
#endif

#ifdef SIVA_RULE
#include "FixedPoint.h"
#include "HRV.h"
#include "Rules.h"
#include "PCA.h"
#endif

#ifdef SIVA_SERIAL
#include "nrf_serial.h"
#include "app_util.h"
#include "nrf_drv_power.h"
#include "app_error.h"
#endif

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif


#ifdef SIVA_ANN
#define PCA_SIZE 5
#define NO_PC 6
#define BEAT_LEN 175

#define SHIFT_ANN_NORM_BIT (FIXED_POINT_FRACTIONAL_BITS - FIXED_POINT_FRACTIONAL_ANN_BITS)

fixed_point_ann_t Aout[2] = {0};

struct ANN_SIVA ann;
#endif

#ifdef SIVA_RULE
#define PCA_SIZE 10
#define NO_PC 1
#define BEAT_LEN 175
#endif

bool sendBeat = false;

#ifdef ADNAN_PKD
#define RR_SIZE 8
#define BUFF ARRAY_LIST_SIZE
#define TWO_SEC 500
//#define TWO_SEC 1000
#define WINDOW_WIDTH 38
//#define WINDOW_WIDTH 150
#define REF_PERIOD 50
//#define REF_PERIOD 200
#define T_PERIOD 90
//#define T_PERIOD 360
#define DELAY_F 21
#define DELAY_I 61
#endif

#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define DEVICE_NAME                     "A_SIVA_BLE"                               /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */

#define APP_ADV_DURATION                0 //18000                                       /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(400, UNIT_1_25_MS)        //400     /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(400, UNIT_1_25_MS)        //400     /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         //4000    /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

#ifdef ADNAN_SPIM
/*
#define MEMORY_SIZE_ROW 1000
#define MEMORY_SIZE_COL 6
static uint8_t memory[MEMORY_SIZE_ROW][MEMORY_SIZE_COL];
static uint16_t memory_index = 0;

static bool volatile memory_full = false;
*/

static volatile bool spi_xfer_done; 

static const nrfx_spim_t spim   = NRFX_SPIM_INSTANCE(2);
static const nrfx_timer_t timer = NRFX_TIMER_INSTANCE(1);

static nrf_ppi_channel_t ppi_channel;

static bool tx_rdy = true;
static bool resend;
static uint16_t ECG_index;

static uint16_t rr_avg1, rr_avg2 = 120;
static uint16_t global_index = 0;

//filter coefficients for pan algorithm
//int h_l[13] = {1, 2, 3, 4, 5, 6, 5, 4, 3, 2, 1, 0, 0};
//int h_h[33] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 31, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0};
//float h_d[5] = {-0.125, -0.25, 0, 0.25, 0.125};

void empty_in_pin_handler(nrfx_gpiote_pin_t     pin,
                          nrf_gpiote_polarity_t action)
{
}

#endif


//............Serial........................................//
#if defined(SIVA_SERIAL) || defined(ADNAN_SPIM)

#define BYTE_LEN 8
#define DATA_BUFFER_SIZE 1000
#define R_POS_SIZE 11

//BEAT REGION
//Sample Frequency
#define Fs 250
#define SamplingShift 8 //2^8 is close to the Fs 250 Hz, hence shifting by 8 can be treated as in time
//Sample length of single beat 
#define lenSignal round(0.7*Fs)
//R peak location and left and right bands
#define RLoc round(0.25*Fs)-1
#define leftBand RLoc
#define rightBand (lenSignal-RLoc)
#define band round(0.056*Fs)

//DEFINE OUTPUT STRUCTURE
#if defined(SIVA_ANN) || defined(SIVA_RULE)
struct HRVmetric{ 
    fixed_point_t wSDNN;              // weighted SDNN
    fixed_point_t SDNN;               // SDNN
    fixed_point_t rrIndex;            // RR Index (2 * (Rpre - Rprepre)/ (Rpre - Rprepre))
    fixed_point_t SD1;                // SD1
    fixed_point_t SD2;                // SD2
  }; 
#endif

struct BEAT{
    #if defined(SIVA_ANN) || defined(SIVA_RULE)
      fixed_point_t RR[R_POS_SIZE];           // RR interval in duration
      fixed_point_t RRmean;                   // mean of RR interval
      fixed_point_t PCA[PCA_SIZE][NO_PC];     // PCA of previous beats
      fixed_point_32t qrsSum;                 // QRS sum around the band
      fixed_point_32t qrsEnergy;              // QRS energy around the band
      fixed_point_t prevvSTD;                 // Max - Min V change of previous beat
      fixed_point_t vvSTD;                    // Max - Min V change
      //fixed_point_t PCAcoefs[NO_PC];        // PCA coefficient with respect to Normal
      fixed_point_t PCAstd;                   // PCA standard deviation
      struct HRVmetric HRV;                   // HRV structure
    #endif
    uint8_t Result;                // Annotation
  } BEAT;

//DEFINE THE DATA BUFFER
  static int sampcount = 0;
  static int16_t Data_Buf[DATA_BUFFER_SIZE];
  static int beatcount = 0;
  static int Rpos[R_POS_SIZE];

  static char tx_message[] = "\n\rHello...!\r\n";
  static char commma = ',';
  static char new_line = '\n';
  char cr[4];
#endif

#ifdef SIVA_SERIAL

static void sleep_handler(void)
{
    __WFE();
    __SEV();
    __WFE();
}

NRF_SERIAL_DRV_UART_CONFIG_DEF(m_uart0_drv_config,
                      RX_PIN_NUMBER, TX_PIN_NUMBER,
                      RTS_PIN_NUMBER, CTS_PIN_NUMBER,
                      NRF_UART_HWFC_ENABLED, NRF_UART_PARITY_EXCLUDED,   //NRF_UART_HWFC_ENABLED, NRF_UART_HWFC_DISABLED
                      NRF_UART_BAUDRATE_115200,
                      UART_DEFAULT_CONFIG_IRQ_PRIORITY);

#define SERIAL_FIFO_TX_SIZE 256
#define SERIAL_FIFO_RX_SIZE 256

NRF_SERIAL_QUEUES_DEF(serial_queues, SERIAL_FIFO_TX_SIZE, SERIAL_FIFO_RX_SIZE);


#define SERIAL_BUFF_TX_SIZE 32
#define SERIAL_BUFF_RX_SIZE 32

NRF_SERIAL_BUFFERS_DEF(serial_buffs, SERIAL_BUFF_TX_SIZE, SERIAL_BUFF_RX_SIZE);

NRF_SERIAL_CONFIG_DEF(serial_config, NRF_SERIAL_MODE_IRQ,
                      &serial_queues, &serial_buffs, NULL, sleep_handler);


NRF_SERIAL_UART_DEF(serial_uart, 0);
//.......end..Serial........................................//
#endif




#ifdef SIVA_BLE
BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */

static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
{
    {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
};
#endif

#ifdef SIVA_BLE
#if (defined(SIVA_ANN) || defined(SIVA_RULE)) 

#ifdef SIVA_ANN
void ble_siva_ann_result_send()
{
    uint16_t length;
    ret_code_t err_code;
    static uint8_t BLE_data[5] = {'~'};
//    static uint8_t index = 0;

//    NRF_LOG_INFO("Ready to send data over BLE NUS");

    if (!resend)
    {
        BLE_data[1] = (uint8_t)(Aout[0]>> 8);
        BLE_data[2] = (uint8_t)Aout[0];
        BLE_data[3] = (uint8_t)(Aout[1] >> 8);
        BLE_data[4] = (uint8_t)Aout[1];  
    }
    else
        resend = false;

    length = 5;

    err_code = ble_nus_data_send(&m_nus, BLE_data, &length, m_conn_handle);

    if (err_code == NRF_ERROR_RESOURCES)
    {
        tx_rdy = false;
        resend = true;
    }
    else if ((err_code != NRF_ERROR_INVALID_STATE) && (err_code != NRF_ERROR_NOT_FOUND))
        APP_ERROR_CHECK(err_code);
}
#endif

void ble_siva_ann_alert(char cr)
{
    uint16_t length = 1;
    ret_code_t err_code;
    static uint8_t BLE_data = '~';
//    static uint8_t index = 0;

//    NRF_LOG_INFO("Ready to send data over BLE NUS");

    if (!resend)
    {
        BLE_data = cr;
    }
    else
        resend = false;

    err_code = ble_nus_data_send(&m_nus, &BLE_data, &length, m_conn_handle);

    if (err_code == NRF_ERROR_RESOURCES)
    {
        tx_rdy = false;
        resend = true;
    }
    else if ((err_code != NRF_ERROR_INVALID_STATE) && (err_code != NRF_ERROR_NOT_FOUND))
        APP_ERROR_CHECK(err_code);
}
#endif


void ble_data_send()
{
    uint16_t length;
    ret_code_t err_code;
    static uint8_t BLE_data[5] = {'~'};
//    static uint8_t index = 0;

//    NRF_LOG_INFO("Ready to send data over BLE NUS");

    if (!resend)
    {
        BLE_data[1] = (uint8_t)(rr_avg1>> 8); //rr_avg1
        BLE_data[2] = (uint8_t)rr_avg1;
        BLE_data[3] = (uint8_t)(rr_avg2 >> 8);
        BLE_data[4] = (uint8_t)rr_avg2;   //rr_avg2
    }
    else
        resend = false;

    length = 5;

    err_code = ble_nus_data_send(&m_nus, BLE_data, &length, m_conn_handle);

    if (err_code == NRF_ERROR_RESOURCES)
    {
        tx_rdy = false;
        resend = true;
    }
    else if ((err_code != NRF_ERROR_INVALID_STATE) && (err_code != NRF_ERROR_NOT_FOUND))
        APP_ERROR_CHECK(err_code);
}


void ble_sample_signal_send(int16_t *cr, int li, uint16_t length){
    ret_code_t err_code;
    static uint8_t BLE_data[240] = {'~'};
//    static uint8_t index = 0;

//    NRF_LOG_INFO("Ready to send data over BLE NUS");

    if (!resend)
    {
        for(int ii=0;ii<length;){
          BLE_data[ii] = (uint8_t)((*(cr+li+(ii/2)))>>8);
          BLE_data[ii+1] = (uint8_t)(*(cr+li+(ii/2)));
          ii = ii+2;
        }
    }
    else
        resend = false;

    err_code = ble_nus_data_send(&m_nus, BLE_data, &length, m_conn_handle);

    if (err_code == NRF_ERROR_RESOURCES)
    {
        tx_rdy = false;
        resend = true;
    }
    else if ((err_code != NRF_ERROR_INVALID_STATE) && (err_code != NRF_ERROR_NOT_FOUND))
        APP_ERROR_CHECK(err_code);
}

void ble_sample_send(char cr[4])
{
    uint16_t length = 3;
    ret_code_t err_code;
    static uint8_t BLE_data[3] = {'~'};
//    static uint8_t index = 0;

//    NRF_LOG_INFO("Ready to send data over BLE NUS");

    if (!resend)
    {
        BLE_data[1] = (uint8_t)(cr[1]);
        BLE_data[2] = (uint8_t)cr[2];
    }
    else
        resend = false;

    err_code = ble_nus_data_send(&m_nus, BLE_data, &length, m_conn_handle);

    if (err_code == NRF_ERROR_RESOURCES)
    {
        tx_rdy = false;
        resend = true;
    }
    else if ((err_code != NRF_ERROR_INVALID_STATE) && (err_code != NRF_ERROR_NOT_FOUND))
        APP_ERROR_CHECK(err_code);
}


#endif


#ifdef ADNAN_PT_ALGO
void Pan_Tompkins_algorithm()
{
bool regular_prev;
uint16_t index, sub_index, qrs_f_index, qrs_index_prev, qrs_i_miss_index, rr_avg1, rr_avg2;
uint32_t peaki;
int32_t peakf, minf, slope, qrs_slope;
static bool phase1_done = false;
static bool phase2_done = false;
static bool asc = false;
static bool rr1_full = false;
static bool rr2_full = false;
static bool regular = true;
static uint16_t qrs_i_index, qrs_index, rr_low_limit, rr_high_limit, rr_missed_limit;
static uint16_t ref_index = 0;
static uint16_t t_index = 0;
static uint16_t t_miss_index = 0;
static uint16_t miss_index = 0;
static uint8_t rr1_index = 0;
static uint8_t rr2_index = 0;
static int32_t ECG_signal[BUFF] = {0};
static int32_t low_pass_filter[BUFF] = {0};
static int32_t high_pass_filter[BUFF] = {0};
static int32_t derivative[BUFF] = {0};
static uint32_t squaring_function[BUFF] = {0};
static uint32_t moving_window_integration[BUFF] = {0};
static uint32_t moving_window_sum = 0;
static uint32_t spki, npki, threshold_i1, threshold_i2;
static int32_t spkf, npkf, minf_curr, threshold_f1, threshold_f2;
static uint16_t rr1[RR_SIZE] = {0};
static uint16_t rr2[RR_SIZE] = {0};
static uint16_t rr_sum1 = 0;
static uint16_t rr_sum2 = 0;

if (ref_index <= BUFF)
ref_index = 1;
else
ref_index -= BUFF;

if (t_index == t_miss_index)
{
if (t_index <= BUFF)
t_index = 0;
else
t_index -= BUFF;
}
else
{
t_index = 0;
t_miss_index = 0;
}

if (miss_index <= BUFF)
miss_index = 0;
else
miss_index -= BUFF;

minf = minf_curr;
minf_curr = INT32_MAX;

  for (index = 0; index < BUFF; ++index)
  {
  ECG_signal[index] = (int32_t)(array_list[index].buffer[6] << 24 | array_list[index].buffer[7] << 16 | array_list[index].buffer[8] << 8) >> 8;

  if (index >= WINDOW_WIDTH)
  {
  low_pass_filter[index] = (low_pass_filter[index-1]<<1) - low_pass_filter[index-2] + ECG_signal[index] - (ECG_signal[index-6]<<1) + ECG_signal[index-12];
  high_pass_filter[index] = (low_pass_filter[index-16]<<5) - high_pass_filter[index-1] - low_pass_filter[index] + low_pass_filter[index-32];

  if (minf_curr > high_pass_filter[index])
  minf_curr = high_pass_filter[index];

  derivative[index] = (-high_pass_filter[index-4] - (high_pass_filter[index-3]<<1) + (high_pass_filter[index-1]<<1) + high_pass_filter[index]) >> 10;
  squaring_function[index] = derivative[index] * derivative[index];

  moving_window_sum -= squaring_function[index-WINDOW_WIDTH];
  moving_window_sum += squaring_function[index];
  moving_window_integration[index] = moving_window_sum >> 5;
  // moving_window_integration[index] = moving_window_sum >> 7;
  }

  else
  {
  low_pass_filter[index] = (low_pass_filter[(index-1+BUFF)%BUFF]<<1) - low_pass_filter[(index-2+BUFF)%BUFF] + ECG_signal[index] - (ECG_signal[(index-6+BUFF)%BUFF]<<1) + ECG_signal[(index-12+BUFF)%BUFF];
  high_pass_filter[index] = (low_pass_filter[(index-16+BUFF)%BUFF]<<5) - high_pass_filter[(index-1+BUFF)%BUFF] - low_pass_filter[index] + low_pass_filter[(index-32+BUFF)%BUFF];

  if (minf_curr > high_pass_filter[index])
  minf_curr = high_pass_filter[index];

  derivative[index] = (-high_pass_filter[(index-4+BUFF)%BUFF] - (high_pass_filter[(index-3+BUFF)%BUFF]<<1) + (high_pass_filter[(index-1+BUFF)%BUFF]<<1) + high_pass_filter[index]) >> 10;
  squaring_function[index] = derivative[index] * derivative[index];

  moving_window_sum -= squaring_function[(index-WINDOW_WIDTH+BUFF)%BUFF];
  moving_window_sum += squaring_function[index];
  moving_window_integration[index] = moving_window_sum >> 5;
  // moving_window_integration[index] = moving_window_sum >> 7;
  }

  // qrs[index] = 0;

  if (phase1_done && index >= ref_index)
  {
  if (moving_window_integration[index] > moving_window_integration[index-1] && asc == false)
  asc = true;

  else if (moving_window_integration[index] < moving_window_integration[index-1] && asc == true)
  {
  asc = false;

  peaki = moving_window_integration[index-1];

  if (peaki <= threshold_i1)
  {
  npki = (peaki >> 3) + npki - (npki >> 3);
  threshold_i1 = npki + ((spki - npki) >> 2);
  }
  else
  {
  spki = (peaki >> 3) + spki - (spki >> 3);
  threshold_i1 = npki + ((spki - npki) >> 2);

  peakf = INT32_MIN;

  for (sub_index = 0; sub_index < WINDOW_WIDTH; ++sub_index)
  if (peakf < high_pass_filter[(index-sub_index+BUFF)%BUFF])
  {
  qrs_f_index = (index - sub_index + BUFF) % BUFF;
  peakf = high_pass_filter[qrs_f_index];
  }

  if (peakf <= threshold_f1)
  {
  npkf = (peakf >> 3) + npkf - (npkf >> 3);
  threshold_f1 = npkf + ((spkf - npkf) >> 2);
  }
  else
  {
  if (index < t_index)
  {
  qrs_slope = 0;
  slope = 0;

  for (sub_index = 0; sub_index < WINDOW_WIDTH; ++sub_index)
  {
  if (qrs_slope < derivative[(qrs_i_index-sub_index+BUFF)%BUFF])
  qrs_slope = derivative[(qrs_i_index-sub_index+BUFF)%BUFF];

  if (slope < derivative[(index-sub_index+BUFF)%BUFF])
  slope = derivative[(index-sub_index+BUFF)%BUFF];
  }

  if (slope < (qrs_slope>>1))
  {
  npki = (peaki >> 3) + npki - (npki >> 3);
  threshold_i1 = npki + ((spki - npki) >> 2);

  npkf = (peakf >> 3) + npkf - (npkf >> 3);
  threshold_f1 = npkf + ((spkf - npkf) >> 2);

  // NRF_LOG_RAW_INFO("T Wave\n");

  continue;
  }
  }

  spkf = (peakf >> 3) + spkf - (spkf >> 3);
  threshold_f1 = npkf + ((spkf - npkf) >> 2);

  qrs_i_index = index - 1;

  ref_index = qrs_i_index + REF_PERIOD;
  t_index = qrs_i_index + T_PERIOD;
  t_miss_index = t_index;
  miss_index = qrs_i_index + rr_missed_limit;
  rr_intervals:
  qrs_index_prev = qrs_index;
  qrs_index = (qrs_f_index - DELAY_F + BUFF) % BUFF;

  // qrs[qrs_index] = 10000;

  if (phase2_done)
  {
  rr_sum1 -= rr1[rr1_index];
  rr1[rr1_index] = (qrs_index - qrs_index_prev + BUFF) % BUFF;
  rr_sum1 += rr1[rr1_index];

  if (rr2_full)
  {
  if (rr1[rr1_index] >= rr_low_limit && rr1[rr1_index] <= rr_high_limit)
  {
  rr_sum2 -= rr2[rr2_index];
  rr2[rr2_index] = rr1[rr1_index];
  rr_sum2 += rr2[rr2_index];

  rr_avg2 = rr_sum2 >> 3;
  rr2_index = (rr2_index + 1) % RR_SIZE;

  rr_low_limit = 0.92 * rr_avg2;
  rr_high_limit = 1.16 * rr_avg2;
  rr_missed_limit = 1.66 * rr_avg2;
  }

  rr_avg1 = rr_sum1 >> 3;
  rr1_index = (rr1_index + 1) % RR_SIZE;
  }

  else if (rr1_full)
  {
  if (rr1[rr1_index] >= rr_low_limit && rr1[rr1_index] <= rr_high_limit)
  {
  rr_sum2 -= rr2[rr2_index];
  rr2[rr2_index] = rr1[rr1_index];
  rr_sum2 += rr2[rr2_index];

  rr_avg2 = rr_sum2 / ++rr2_index;

  rr_low_limit = 0.92 * rr_avg2;
  rr_high_limit = 1.16 * rr_avg2;
  rr_missed_limit = 1.66 * rr_avg2;
  }

  rr_avg1 = rr_sum1 >> 3;
  rr1_index = (rr1_index + 1) % RR_SIZE;

  if (rr2_index == RR_SIZE)
  {
  rr2_index = 0;
  rr2_full = true;
  }
  }

  else
  {
  if (rr1[rr1_index] >= rr_low_limit && rr1[rr1_index] <= rr_high_limit)
  {
  rr_sum2 -= rr2[rr2_index];
  rr2[rr2_index] = rr1[rr1_index];
  rr_sum2 += rr2[rr2_index];

  rr_avg2 = rr_sum2 / ++rr2_index;

  rr_low_limit = 0.92 * rr_avg2;
  rr_high_limit = 1.16 * rr_avg2;
  rr_missed_limit = 1.66 * rr_avg2;
  }

  rr_avg1 = rr_sum1 / ++rr1_index;

  if (rr1_index == RR_SIZE)
  {
  rr1_index = 0;
  rr1_full = true;
  }

  if (rr2_index == RR_SIZE)
  {
  rr2_index = 0;
  rr2_full = true;
  }
  }

  regular_prev = regular;

  if (rr_avg1 == rr_avg2)
  {
  regular = true;

  if (regular != regular_prev)
  {
  threshold_i1 <<= 1;
  threshold_f1 = ((threshold_f1 - minf) << 1) + minf;
  }
  }
  else
  {
  regular = false;

  if (regular != regular_prev)
  {
  threshold_i1 >>= 1;
  threshold_f1 = ((threshold_f1 - minf) >> 1) + minf;
  }
  }
  }

  else
  {
  if (rr1_index)
  {
  rr1[0] = (qrs_index - qrs_index_prev + BUFF) % BUFF;
  rr2[0] = rr1[0];

  ++rr2_index;

  rr_sum1 = rr1[0];
  rr_sum2 = rr2[0];

  rr_avg1 = rr_sum1;
  rr_avg2 = rr_sum2;

  rr_low_limit = 0.92 * rr_avg2;
  rr_high_limit = 1.16 * rr_avg2;
  rr_missed_limit = 1.66 * rr_avg2;

  phase2_done = true;

  // NRF_LOG_RAW_INFO("rr_avg1 = %d, rr_avg2 = %d, rr_low_limit = %d, rr_high_limit = %d, rr_missed_limit = %d\n", rr_avg1, rr_avg2, rr_low_limit, rr_high_limit, rr_missed_limit);
  }
  else
  ++rr1_index;
  }

  // NRF_LOG_RAW_INFO("rr_avg1 = %d, rr_avg2 = %d, rr1 = %d, rr2 = %d\n", rr_avg1, rr_avg2, rr1[(rr1_index-1+RR_SIZE)%RR_SIZE], rr2[(rr2_index-1+RR_SIZE)%RR_SIZE]);
  /* NRF_LOG_RAW_INFO("threshold_i1 = %d, peaki = %d, qrs_i_index = %d\n", threshold_i1, peaki, qrs_i_index);
  NRF_LOG_RAW_INFO("threshold_f1 = %d, peakf = %d, qrs_f_index = %d\n", threshold_f1, peakf, qrs_f_index);
  NRF_LOG_RAW_INFO("qrs_index = %d\n", qrs_index);
  */ }
  }
  }

  if (phase2_done && index >= miss_index)
  {
  threshold_i2 = threshold_i1 >> 1;
  threshold_f2 = ((threshold_f1 - minf) >> 1) + minf;

  peaki = 0;

  for (sub_index = 0; sub_index < (rr_missed_limit-REF_PERIOD); ++sub_index)
  if (peaki < moving_window_integration[(index-sub_index+BUFF)%BUFF])
  {
  qrs_i_miss_index = (index - sub_index + BUFF) % BUFF;
  peaki = moving_window_integration[qrs_i_miss_index];
  }

  if (peaki > threshold_i2)
  {
  spki = (peaki >> 2) + spki - (spki >> 2);
  threshold_i1 = npki + ((spki - npki) >> 2);

  peakf = INT32_MIN;

  for (sub_index = 0; sub_index < WINDOW_WIDTH; ++sub_index)
  if (peakf < high_pass_filter[(qrs_i_miss_index-sub_index+BUFF)%BUFF])
  {
  qrs_f_index = (qrs_i_miss_index - sub_index + BUFF) % BUFF;
  peakf = high_pass_filter[qrs_f_index];
  }

  if (peakf > threshold_f2)
  {
  if (qrs_i_miss_index < ((qrs_i_miss_index>index) ? t_miss_index : t_index))
  {
  qrs_slope = 0;
  slope = 0;

  for (sub_index = 0; sub_index < WINDOW_WIDTH; ++sub_index)
  {
  if (qrs_slope < derivative[(qrs_i_index-sub_index+BUFF)%BUFF])
  qrs_slope = derivative[(qrs_i_index-sub_index+BUFF)%BUFF];

  if (slope < derivative[(qrs_i_miss_index-sub_index+BUFF)%BUFF])
  slope = derivative[(qrs_i_miss_index-sub_index+BUFF)%BUFF];
  }

  if (slope < (qrs_slope>>1))
  {
  npki = (peaki >> 3) + npki - (npki >> 3);
  threshold_i1 = npki + ((spki - npki) >> 2);

  npkf = (peakf >> 3) + npkf - (npkf >> 3);
  threshold_f1 = npkf + ((spkf - npkf) >> 2);

  // NRF_LOG_RAW_INFO("T Wave\n");

  continue;
  }
  }

  spkf = (peakf >> 2) + spkf - (spkf >> 2);
  threshold_f1 = npkf + ((spkf - npkf) >> 2);

  qrs_i_index = qrs_i_miss_index;

  ref_index = qrs_i_index + REF_PERIOD;
  t_index = qrs_i_index + T_PERIOD;
  t_miss_index = t_index;
  miss_index = qrs_i_index + rr_missed_limit;

  if (qrs_i_index > index)
  {

  if (ref_index <= BUFF)
  ref_index = 1;
  else
  ref_index -= BUFF;

  if (t_index <= BUFF)
  t_index = 0;
  else
  t_index -= BUFF;

  if (miss_index <= BUFF)
  miss_index = 0;
  else
  miss_index -= BUFF;
  }

  goto rr_intervals;
  }
  }
  }
  }

  else if (!phase1_done)
  {
  if (index == DELAY_I+5)
  {
  spki = moving_window_integration[index];
  npki = moving_window_integration[index] >> 9;

  // NRF_LOG_RAW_INFO("spki[DELAY_I] = %d, npki[DELAY_I] = %d\n", spki, npki);
  }
  else if (index > DELAY_I+5)
  {
  if (spki < moving_window_integration[index])
  spki = moving_window_integration[index];

  npki += moving_window_integration[index] >> 9;
  }

  if (index == DELAY_F+5)
  {
  minf_curr = high_pass_filter[index];

  spkf = high_pass_filter[index];
  npkf = high_pass_filter[index] >> 9;

  // NRF_LOG_RAW_INFO("spkf[DELAY_F] = %d, npkf[DELAY_F] = %d\n", spkf, npkf);
  }
  else if (index > DELAY_F+5)
  {
  if (spkf < high_pass_filter[index])
  spkf = high_pass_filter[index];

  npkf += high_pass_filter[index] >> 9;
  }

  if (index == TWO_SEC-1)
  {
  // NRF_LOG_RAW_INFO("spki = %d, npki = %d, spkf = %d, npkf = %d, minf = %d\n", spki, npki, spkf, npkf, minf);

  minf = minf_curr;

  spki >>= 1;
  npki >>= 1;

  //spkf = ((spkf - minf) >> 1) + minf;
  spkf = ((spkf - minf) >> 1) + ((spkf - minf) >> 2) + minf;
  //npkf = ((npkf - minf) >> 1) + minf;
  npkf = ((npkf - minf) >> 1) + ((npkf - minf) >> 2) + minf;

  threshold_i1 = npki + ((spki - npki) >> 2);
  threshold_f1 = npkf + ((spkf - npkf) >> 2);

  // NRF_LOG_RAW_INFO("spki = %d, npki = %d, threshold_i1 = %d, threshold_i2 = %d\n", spki, npki, threshold_i1, threshold_i2);
  // NRF_LOG_RAW_INFO("spkf = %d, npkf = %d, threshold_f1 = %d, threshold_f2 = %d\n", spkf, npkf, threshold_f1, threshold_f2);

  phase1_done = true;
  }
  }
  }
}



//void pan_algo(int32_t x) {
//
//  int i;
//  static int count = 0;
//  static int buf_lp[13] = {0};
//  static float buf_hp[33] = {0};
//  static float buf_de[5] = {0};
//  static float buf_ma[38] = {0}; //window size may change depending on sampling f
//  //static float buf_ma[150] = {0}; //window size may change depending on sampling f
//  static float buf_fm[5];
//  float ecg_temp;
//  int isMissed, isLocPeak, flag_peak, missed, loc;
//  static int flag_miss = 0;
//  static int miss_cnt = 0;
//  static float spk, npk, thre;
//  static float loc_peak = 0.0;
//  static int flag = 0;
//  static int flag_cnt = 0;
//  static float max_tr = 0.0;
//  static float mean_tr = 0.0;
//  static int mean_cnt = 0;
//  static int miss_valid = 1;
//  static int loc_miss;
//
//    uint16_t qrs_index_prev;
//    static bool phase2_done = false;
//    static bool rr1_full = false;
//    static bool rr2_full = false;
//    static uint16_t qrs_index, rr_low_limit, rr_high_limit, rr_missed_limit;
//    static uint8_t rr1_index = 0;
//    static uint8_t rr2_index = 0;
//    static uint16_t rr1[RR_SIZE] = {0};
//    static uint16_t rr2[RR_SIZE] = {0};
//    static uint16_t rr_sum1 = 0;
//    static uint16_t rr_sum2 = 0;
//
//  //low pass filter
//  for (i = 0; i < 12; i++)
//    buf_lp[i] = x * h_l[i] + buf_lp[i + 1];
//  buf_lp[12] = x * h_l[12];
//  ecg_temp = buf_lp[0] * 1.0 / 100.0;
//
//  //high pass filter
//  for (i = 0; i < 32; i++)
//    buf_hp[i] = ecg_temp * h_h[i] + buf_hp[i + 1];
//  buf_hp[32] = ecg_temp * h_h[32];
//  ecg_temp = buf_hp[0] / 20;
//
//  //five-point derivative
//  for (i = 0; i < 4; i++)
//    buf_de[i] = ecg_temp * h_d[i] + buf_de[i + 1];
//  buf_de[4] = ecg_temp * h_d[4];
//  ecg_temp = buf_de[0];
//
//  //squaring function
//  ecg_temp = ecg_temp * ecg_temp;
//
//  //moving average -- window size: round(0.15 * f) 250Hz(38)
//  for (i = 0; i < 37; i++)
//    buf_ma[i] = ecg_temp / 38 + buf_ma[i + 1];
//  buf_ma[37] = ecg_temp / 38;
//  ecg_temp = buf_ma[0];
///*
//  //moving average -- window size: round(0.15 * f) 250Hz(38)
//  for (i = 0; i < 149; i++)
//    buf_ma[i] = ecg_temp / 150 + buf_ma[i + 1];
//  buf_ma[149] = ecg_temp / 150;
//  ecg_temp = buf_ma[0];
//*/
//  //fiducial marks -- detect local peaks
//  if (count <= 4)
//    buf_fm[count] = ecg_temp;
//  else {
//    buf_fm[0] = buf_fm[1];
//    buf_fm[1] = buf_fm[2];
//    buf_fm[2] = buf_fm[3];
//    buf_fm[3] = buf_fm[4];
//    buf_fm[4] = ecg_temp;
//  }
//
//  isMissed = 0;
//  isLocPeak = 0;
//  flag_peak = 0;
//  if (flag_miss) {
//    miss_cnt = miss_cnt + 1;
//    if (miss_cnt >= 415) //1.66*f  250Hz(415)
//    //if (miss_cnt >= 1660) //1.66*f  250Hz(415)
//    {
//      isMissed = 1;
//      spk = 0.25 * loc_peak + 0.75 * spk;
//      miss_cnt = 0;
//    }
//  }
//
//  if (count >= 4) {
//    if (flag) {
//      flag_cnt = flag_cnt + 1;
//      if (flag_cnt >= 50) //0.2*f   250Hz(50)
//      //if (flag_cnt >= 200) //0.2*f   250Hz(50)
//      {
//        flag = 0;
//        flag_cnt = 0;
//      }
//    }
//    if ((buf_fm[1] >= buf_fm[0]) && (buf_fm[2] >= buf_fm[1]) && (buf_fm[2] >= buf_fm[3]) && (buf_fm[3] >= buf_fm[4])) {
//      loc_peak = buf_fm[2];
//      isLocPeak = 1;
//    }
//  }
//
//  //training stage
//  if (count < 500) //2*f  250Hz(500)
//  //if (count < 2000) //2*f  250Hz(500)
//  {
//    if (loc_peak > max_tr)
//      max_tr = loc_peak;
//    mean_tr = mean_tr + loc_peak;
//    mean_cnt = mean_cnt + 1;
//  } else if (count == 500) {
//  //} else if (count == 2000) {
//    thre = max_tr / 3.0;
//    spk = thre;
//    npk = mean_tr / mean_cnt / 2.0;
//  } else {
//    if (isLocPeak) {
//      if (loc_peak >= thre) {
//        if (flag == 0) {
//          flag_peak = 1;
//          spk = 0.125 * loc_peak + 0.875 * spk;
//          flag = 1;
//          miss_cnt = 0;
//          miss_valid = 1;
//          flag_miss = 1;
//        }
//      } else {
//        npk = 0.125 * loc_peak + 0.875 * npk;
//        if ((loc_peak >= thre / 2.0) && (miss_valid)) {
//          loc_miss = count;
//          miss_valid = 0;
//        }
//      }
//      thre = npk + 0.25 * (spk - npk);
//    }
//  }
//
//  if (flag_peak) {
////    printf("3000\n");
//                        qrs_index_prev = qrs_index;
//                        qrs_index = global_index;
//
////                        qrs[qrs_index] = 10000;
//
//                        if (phase2_done)
//                        {
//                            rr_sum1 -= rr1[rr1_index];
//                            rr1[rr1_index] = (qrs_index - qrs_index_prev + BUFF) % BUFF;
//                            rr_sum1 += rr1[rr1_index];
//
//                            if (rr2_full)
//                            {
//                                if (rr1[rr1_index] >= rr_low_limit && rr1[rr1_index] <= rr_high_limit)
//                                {
//                                    rr_sum2 -= rr2[rr2_index];
//                                    rr2[rr2_index] = rr1[rr1_index];
//                                    rr_sum2 += rr2[rr2_index];
//
//                                    rr_avg2 = rr_sum2 >> 3;
//                                    rr2_index = (rr2_index + 1) % RR_SIZE;
//
//                                    rr_low_limit = 0.92 * rr_avg2;
//                                    rr_high_limit = 1.16 * rr_avg2;
//                                    rr_missed_limit = 1.66 * rr_avg2;
//                                }
//
//                                rr_avg1 = rr_sum1 >> 3;
//                                rr1_index = (rr1_index + 1) % RR_SIZE;
//                            }
//
//                            else if (rr1_full)
//                            {
//                                if (rr1[rr1_index] >= rr_low_limit && rr1[rr1_index] <= rr_high_limit)
//                                {
//                                    rr_sum2 -= rr2[rr2_index];
//                                    rr2[rr2_index] = rr1[rr1_index];
//                                    rr_sum2 += rr2[rr2_index];
//
//                                    rr_avg2 = rr_sum2 / ++rr2_index;
//
//                                    rr_low_limit = 0.92 * rr_avg2;
//                                    rr_high_limit = 1.16 * rr_avg2;
//                                    rr_missed_limit = 1.66 * rr_avg2;
//                                }
//
//                                rr_avg1 = rr_sum1 >> 3;
//                                rr1_index = (rr1_index + 1) % RR_SIZE;
//
//                                if (rr2_index == RR_SIZE)
//                                {
//                                    rr2_index = 0;
//                                    rr2_full = true;
//                                }
//                            }
//
//                            else
//                            {
//                                if (rr1[rr1_index] >= rr_low_limit && rr1[rr1_index] <= rr_high_limit)
//                                {
//                                    rr_sum2 -= rr2[rr2_index];
//                                    rr2[rr2_index] = rr1[rr1_index];
//                                    rr_sum2 += rr2[rr2_index];
//
//                                    rr_avg2 = rr_sum2 / ++rr2_index;
//
//                                    rr_low_limit = 0.92 * rr_avg2;
//                                    rr_high_limit = 1.16 * rr_avg2;
//                                    rr_missed_limit = 1.66 * rr_avg2;
//                                }
//
//                                rr_avg1 = rr_sum1 / ++rr1_index;
//
//                                if (rr1_index == RR_SIZE)
//                                {
//                                    rr1_index = 0;
//                                    rr1_full = true;
//                                }
//
//                                if (rr2_index == RR_SIZE)
//                                {
//                                    rr2_index = 0;
//                                    rr2_full = true;
//                                }
//                            }
//                        }
//
//                        else
//                        {
//                            if (rr1_index)
//                            {
//                                rr1[0] = (qrs_index - qrs_index_prev + BUFF) % BUFF;
//                                rr2[0] = rr1[0];
//
//                                ++rr2_index;
//
//                                rr_sum1 = rr1[0];
//                                rr_sum2 = rr2[0];
//
//                                rr_avg1 = rr_sum1;
//                                rr_avg2 = rr_sum2;
//
//                                rr_low_limit = 0.92 * rr_avg2;
//                                rr_high_limit = 1.16 * rr_avg2;
//                                rr_missed_limit = 1.66 * rr_avg2;
//
//                                phase2_done = true;
//
////                                NRF_LOG_RAW_INFO("rr_avg1 = %d, rr_avg2 = %d, rr_low_limit = %d, rr_high_limit = %d, rr_missed_limit = %d\n", rr_avg1, rr_avg2, rr_low_limit, rr_high_limit, rr_missed_limit);
//                            }
//                            else
//                                ++rr1_index;
//                        }
//  }
///*  else
//    printf("0\n");
//*/
//  if (isMissed) {
//    missed = 1;
//    loc = loc_miss;
//  } else {
//    missed = 0;
//    loc = 0;
//  }
//
//  count = count + 1;
//}


#endif


#ifdef ADNAN_SPIM

#define TEST_STRING "Nordic"
static uint8_t       m_tx_buf[] = TEST_STRING;           /**< TX buffer. */
static uint8_t       m_rx_buf[sizeof(TEST_STRING) + 1];    /**< RX buffer. */
static const uint8_t m_length = sizeof(m_tx_buf); 

void timer_event_handler(nrf_timer_event_t event_type,
                         void            * p_context)
{
      
    nrf_spim_rx_buffer_set(spim.p_reg, array_list[0].buffer, BUFFER_SIZE);

    #ifndef UMFT4222H

    uint16_t index;
    int32_t channelData[ARRAY_LIST_SIZE] = {0};

      for (index = 0; index < ARRAY_LIST_SIZE; ++index)
      {     
            channelData[index] = (int32_t)(array_list[index].buffer[6] << 24 | array_list[index].buffer[7] << 16 | array_list[index].buffer[8] << 8) >> 8;
            channelData[index] += 30000;
            global_index = index;

            #ifdef ADNAN_PT_ALGO
              Pan_Tompkins_algorithm();       
              //pan_algo(channelData[index]);
            #endif       
      }

      //printf("Calling ble_data_send\r\n");
   
          #ifdef SIVA_BLE  
            if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
            {
                resend = false;
                //ECG_index = 0;
                ble_data_send();
            }
          #endif

      /*
      for (index = 0; index < ARRAY_LIST_SIZE && memory_index < MEMORY_SIZE_ROW; ++index, ++memory_index)
          memcpy(&memory[memory_index][0], &array_list[index].buffer[3], MEMORY_SIZE_COL);

      if (memory_index == MEMORY_SIZE_ROW)
      {
          for (memory_index = 0; index < ARRAY_LIST_SIZE && memory_index < MEMORY_SIZE_ROW; ++index, ++memory_index)
              memcpy(&memory[memory_index][0], &array_list[index].buffer[3], MEMORY_SIZE_COL);

          memory_full = true;
      }
      */

    #else

      #if defined(SIVA_ANN) || defined(SIVA_RULE)
        fixed_point_t Signal[BEAT_LEN];
      #endif
      
        for (uint16_t index = 0; index < ARRAY_LIST_SIZE; ++index)
        {     
              
              #ifdef ADNAN_PT_ALGO
                Pan_Tompkins_algorithm();       
                //pan_algo(channelData[index]);
              #endif

              if (array_list[index].buffer[6] == '\n'){
            
                  if (array_list[index].buffer[3] == '2'){
                      beatcount = 0; 
                      sampcount = 0;
                      memset(Data_Buf, 0x00, DATA_BUFFER_SIZE);
                      #if defined(SIVA_ANN) || defined(SIVA_RULE)
                        memset(BEAT.RR, 0x00, R_POS_SIZE);
                        memset(BEAT.PCA, 0x00, PCA_SIZE*NO_PC);
                      #endif
                      //printf("New set of data identified\n");
                  }else if (array_list[index].buffer[3] == '0'){
                      //do nothing
                  }else if (array_list[index].buffer[3] == '1') {
                      Rpos[beatcount % R_POS_SIZE] = sampcount;
                
                        #if defined(SIVA_ANN) || defined(SIVA_RULE)
                        for (int i = 0; i < R_POS_SIZE - 1; i++){  
                          BEAT.RR[i]  = BEAT.RR[i + 1];
                          //printf("%d, ", BEAT.RR[i]);                    
                        } 
                  
                  
                        for (int i = 0; i < PCA_SIZE - 1; i++){  
                          for (int j = 0; j < NO_PC; j++){
                              BEAT.PCA[i][j] = BEAT.PCA[i + 1][j];
                          }
                          //printf("%d, ", BEAT.PCA[i]);
                        }
                  
                        BEAT.RR[R_POS_SIZE-1] = (sampcount - Rpos[(beatcount-1) % R_POS_SIZE]) << (FIXED_POINT_FRACTIONAL_BITS - SamplingShift); 
                        //printf("%d\n", BEAT.RR[R_POS_SIZE-1]);                    
                        #endif

                
                      if (beatcount > 0){

                        #if defined(SIVA_ANN) || defined(SIVA_RULE)
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
                              //for (int i = 0; i<BEAT_LEN-1; i++){
                                //printf("%d, ", maapstd[i]);
                              //}
                              //printf("%d\n", maapstd[BEAT_LEN-1]);
                          #if defined(SIVA_RULE)

                          //RR mean
                             BEAT.RRmean = mean(BEAT.RR, 11); 

                          //QRS Sum 0.05*Fs:0.05*Fs                     
                            BEAT.qrsSum = qrssum(maapstd, RLoc, band);
                            //printf("qrsSum : %d\n", BEAT.qrsSum);
                          
                          //QRS Energy 0.05*Fs:0.05*Fs                     
                            BEAT.qrsEnergy = qrsEnergy(maapstd, RLoc, band);
                            //printf("qrsEnergy : %d\n", BEAT.qrsEnergy); 

                          //SDNN: RR standard deviation
                            //float rrSTD;
                            //rrSTD = HRV_SDNN(RR_INT,10);
                            //printf("rrTSD/SDNN : %.4f\n", rrSTD); 

                          //HRV Test
                           BEAT.HRV.SDNN = HRV_SDNN(BEAT.RR,11);  
                           BEAT.HRV.SD1 = HRV_SD1(&BEAT.RR[7]);  
                           BEAT.HRV.SD2 = HRV_SD2(&BEAT.RR[7]);  
                            

                          //VVstd
                            //BEAT.prevvSTD = BEAT.vvSTD;
                            //BEAT.vvSTD = vvPeakSTD(maapstd, RLoc, band);
                            //printf("vvSTD : %d\n", BEAT.vvSTD); 

                          //PCA
                            BEAT.PCA[PCA_SIZE-1][NO_PC -1] = findPCAcoeff(maapstd, BEAT_LEN);
                            //printf("PCAcoeff: %d\n", BEAT.PCAcoef); 
                            //BEAT.PCA[PCA_SIZE-1][0] = BEAT.PCAcoef;      
                            //printf("%d\n", BEAT.PCA[PCA_SIZE-1]);          
                            BEAT.PCAstd = PCAstd(BEAT.PCA, PCA_SIZE);  
                          #endif

                          #if defined(SIVA_ANN)
                          //PCA coefficients              
                            findPCAcoeffs(&BEAT.PCA[PCA_SIZE-1][0], maapstd, BEAT_LEN);
                          #endif

                          //if (beatcount > 10){  
                            //printf("%d, ",beatcount);
                            //(void)nrf_serial_write(&serial_uart, &beatcount, sizeof(int), NULL, 0);
                            //(void)nrf_serial_write(&serial_uart, &commma, 1, NULL, 0);
                             
                            //HRV Test
                              BEAT.HRV.wSDNN = weightedSDNN(10,BEAT.RR,9,11);
                              //BEAT.HRV.SDNN = HRV_SDNN(BEAT.RR,11);   
                              BEAT.HRV.rrIndex = HRV_RRIndex(&BEAT.RR[9]);
                            
                            #ifdef SIVA_RULE
                              //BEAT.Result = 49;
                              BEAT.Result = checkforArrhythmia_II(BEAT.HRV.wSDNN, BEAT.HRV.rrIndex,&BEAT.RR[8], BEAT.RRmean, BEAT.HRV.SD1, BEAT.HRV.SD2, BEAT.HRV.SDNN, &BEAT.PCA[PCA_SIZE-1][NO_PC-1], BEAT.PCAstd, BEAT.qrsSum, BEAT.qrsEnergy);
                              //BEAT.Result = checkforArrhythmia(BEAT.HRV.wSDNN,BEAT.HRV.rrIndex, &BEAT.RR[8], BEAT.qrsSum, BEAT.PCAcoef, BEAT.vvSTD, BEAT.prevvSTD, BEAT.HRV.SDNN, BEAT.PCAstd);
                              //BEAT.Result = check_wSDNN_RRindex(BEAT.HRV.wSDNN,BEAT.HRV.rrIndex);
                              //printf("%d, %d, %d, %d, %d, %d, %d, %d, %d,", beatcount, BEAT.HRV.wSDNN, BEAT.HRV.SDNN, BEAT.HRV.rrIndex, BEAT.qrsSum, BEAT.vvSTD, BEAT.PCAcoef, BEAT.PCAstd, BEAT.Result);                   
                    
                              //BEAT.Result = checkforTsipouras(BEAT.RR[8], BEAT.RR[9], BEAT.RR[10]);
                              //printf("%d, ",BEAT.Result);

                              //BEAT.Result = checkforExarchos(BEAT.RR[8], BEAT.RR[9], BEAT.RR[10]);
                              //printf("%d\n",BEAT.Result);
                            #endif
                        
                            //ANN EXECUTION
                            #if defined(SIVA_ANN)
                              fixed_point_ann_t Ain1[5] = {(BEAT.RR[9]>>SHIFT_ANN_NORM_BIT), (BEAT.RR[10]>>SHIFT_ANN_NORM_BIT), (mean(maapstd,BEAT_LEN)>>SHIFT_ANN_NORM_BIT), (BEAT.HRV.wSDNN>>SHIFT_ANN_NORM_BIT), (BEAT.HRV.rrIndex>>SHIFT_ANN_NORM_BIT)};
                              fixed_point_ann_t Ain2[5][6] = {0};
                              for (int i = 0; i < PCA_SIZE; i++){  
                                for (int j = 0; j < NO_PC; j++){
                                    Ain2[i][j] = BEAT.PCA[i][j]>>SHIFT_ANN_NORM_BIT;
                                  }
                                  //printf("%d, ", BEAT.PCA[i]);
                                }
                        
                              ann.predict(&ann,Aout,Ain2,Ain1); 
                        
                              if (Aout[0] > Aout[1]) BEAT.Result = 48;   //48 is 0: Normal Beat
                              else BEAT.Result = 49;                     //49 is 1: Abvnormal Beat
                            #endif

                              if (BEAT.Result == 49){
                                  sendBeat=true;
                                  #ifdef BLE_ALERT_1BYTE
                                    ble_siva_ann_alert(BEAT.Result);   
                                  #endif
                              }
                            #else
                                 BEAT.Result = 49;                      //49 is 1: Abvnormal Beat
                            #endif                     

                        }else{
                  
                          #if defined(SIVA_ANN) || defined(SIVA_RULE)
                          //Just copy the first RR interval to all as this is the first beat
                          for (int i = R_POS_SIZE - 2; i >= 0; i--){  
                            BEAT.RR[i]  = BEAT.RR[i + 1];
                            //printf("%d, ", BEAT.RR[i]);
                            }
                            
                          #endif
                  
                        }

                      beatcount++;               
                
                  } 
            
                  //Store read data to the ring buffer 
                  Data_Buf[sampcount % DATA_BUFFER_SIZE] = array_list[index].buffer[4] << BYTE_LEN;
                  Data_Buf[sampcount % DATA_BUFFER_SIZE] = Data_Buf[sampcount % DATA_BUFFER_SIZE] | array_list[index].buffer[5];

                  #ifdef BLE_ALL_SAMPLES
                  //ble_sample_send(cr);
                  if (sampcount%120==119){
                     ble_sample_signal_send(Data_Buf, sampcount % DATA_BUFFER_SIZE, 240);
                  }
                  #endif
            
                  #ifdef BLE_ALERT_ECGSAMPLE
                  if (BEAT.Result == 49 && sendBeat==true){
                      ble_sample_signal_send(Signal, 0,240);
                      ble_sample_signal_send(Signal, 240,110);
                      sendBeat = false;
                  }
                  #endif

                  sampcount++;
              }       
              
                     
        }
        
    #endif
  
}




void empty_spim_event_handler(nrfx_spim_evt_t const * p_event,
                              void *                  p_context)
{
    //spi_xfer_done = true;
    //NRF_LOG_INFO("Transfer completed.");
    //if (m_rx_buf[0] != 0)
    //{
        //NRF_LOG_INFO(" Received:");
        //NRF_LOG_HEXDUMP_INFO(m_rx_buf, strlen((const char *)m_rx_buf));
    //}
}
#endif

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for initializing the timer module.
 */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

#ifdef SIVA_BLE

/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_evt_t * p_evt)
{
    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        uint32_t err_code;

//        NRF_LOG_DEBUG("Received data from BLE NUS. Writing data on UART.");
//        NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);

        for (uint32_t i = 0; i < p_evt->params.rx_data.length; i++)
        {
            do
            {
                err_code = app_uart_put(p_evt->params.rx_data.p_data[i]);
                if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
                {
//                    NRF_LOG_ERROR("Failed receiving NUS message. Error 0x%x. ", err_code);
                    APP_ERROR_CHECK(err_code);
                }
            } while (err_code == NRF_ERROR_BUSY);
        }
        if (p_evt->params.rx_data.p_data[p_evt->params.rx_data.length - 1] == '\r')
        {
            while (app_uart_put('\n') == NRF_ERROR_BUSY);
        }
    }

    else if (p_evt->type == BLE_NUS_EVT_TX_RDY)
    {
        if (!tx_rdy)
        {
            tx_rdy = true;
            
//            uint16_t length = 6;
//            char BLE_data[] = "TX_RDY";
//            (void)ble_nus_data_send(&m_nus, BLE_data, &length, m_conn_handle);

        }
    }

    else if (p_evt->type == BLE_NUS_EVT_COMM_STARTED)
    {
        if (!tx_rdy)
        {
//            uint16_t length = 8;
//            char BLE_data[] = "COM_STRT";
//            (void)ble_nus_data_send(&m_nus, BLE_data, &length, m_conn_handle);
            
        }

        #ifdef SIVA_SERIAL
        ret_code_t ret = nrf_serial_write(&serial_uart,
                           tx_message,
                           strlen(tx_message),
                           NULL,
                           NRF_SERIAL_MAX_TIMEOUT);
        APP_ERROR_CHECK(ret);
        #endif
    }
}
/**@snippet [Handling the data received over BLE] */

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t           err_code;
    ble_nus_init_t     nus_init;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize NUS.
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}

/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;
        default:
            break;
    }
}

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
//            NRF_LOG_INFO("Connected");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
//            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
//            NRF_LOG_INFO("Disconnected");
            // LED indication will be changed when advertising starts.
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
//            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_2MBPS,
                .tx_phys = BLE_GAP_PHY_2MBPS,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
//        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
//    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
//                  p_gatt->att_mtu_desired_central,
//                  p_gatt->att_mtu_desired_periph);
}

/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;

        default:
            break;
    }
}

///**@brief   Function for handling app_uart events.
// *
// * @details This function will receive a single character from the app_uart module and append it to
// *          a string. The string will be be sent over BLE when the last character received was a
// *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
// */
///**@snippet [Handling the data received over UART] */
//void uart_event_handle(app_uart_evt_t * p_event)
//{
//    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
//    static uint8_t index = 0;
//    uint32_t       err_code;
//
//    switch (p_event->evt_type)
//    {
//        case APP_UART_DATA_READY:
//            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
//            index++;
//
//            if ((data_array[index - 1] == '\n') ||
//                (data_array[index - 1] == '\r') ||
//                (index >= m_ble_nus_max_data_len))
//            {
//                if (index > 1)
//                {
////                    NRF_LOG_DEBUG("Ready to send data over BLE NUS");
////                    NRF_LOG_HEXDUMP_DEBUG(data_array, index);
//
//                    do
//                    {
//                        uint16_t length = (uint16_t)index;
//                        err_code = ble_nus_data_send(&m_nus, data_array, &length, m_conn_handle);
//                        if ((err_code != NRF_ERROR_INVALID_STATE) &&
//                            (err_code != NRF_ERROR_RESOURCES) &&
//                            (err_code != NRF_ERROR_NOT_FOUND))
//                        {
//                            APP_ERROR_CHECK(err_code);
//                        }
//                    } while (err_code == NRF_ERROR_RESOURCES);
//                }
//
//                index = 0;
//            }
//            break;
//
//        case APP_UART_COMMUNICATION_ERROR:
//            APP_ERROR_HANDLER(p_event->data.error_communication);
//            break;
//
//        case APP_UART_FIFO_ERROR:
//            APP_ERROR_HANDLER(p_event->data.error_code);
//            break;
//
//        default:
//            break;
//    }
//}
///**@snippet [Handling the data received over UART] */
//
///**@brief  Function for initializing the UART module.
// */
///**@snippet [UART Initialization] */
//static void uart_init(void)
//{
//    uint32_t                     err_code;
//    app_uart_comm_params_t const comm_params =
//    {
//        .rx_pin_no    = RX_PIN_NUMBER,
//        .tx_pin_no    = TX_PIN_NUMBER,
//        .rts_pin_no   = RTS_PIN_NUMBER,
//        .cts_pin_no   = CTS_PIN_NUMBER,
//        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
//        .use_parity   = false,
//#if defined (UART_PRESENT)
//        .baud_rate    = NRF_UART_BAUDRATE_115200
//#else
//        .baud_rate    = NRF_UARTE_BAUDRATE_115200
//#endif
//    };
//
//    APP_UART_FIFO_INIT(&comm_params,
//                       UART_RX_BUF_SIZE,
//                       UART_TX_BUF_SIZE,
//                       uart_event_handle,
//                       APP_IRQ_PRIORITY_LOWEST,
//                       err_code);
//    APP_ERROR_CHECK(err_code);
//}
/**@snippet [UART Initialization] */

/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE; //BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
}
#endif

/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;
    
    //uint32_t err_code = bsp_init(BSP_INIT_NONE, bsp_event_handler);
    #ifdef SIVA_BLE
    uint32_t err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);
    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
    #else
    uint32_t err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, NULL);
    APP_ERROR_CHECK(err_code);
    #endif
    

    
}

/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{

    //NRF_LOG_INFO("Sleep..");
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}



int main(void)
{   
       
    //initialize the ann
    #if defined(SIVA_SERIAL) || defined(ADNAN_SPIM)

    #ifdef SIVA_ANN
      ann = ANN_SIVA.new();
      ann.init(&ann);

      //Sample Beat
      #ifdef SIVA_SERIAL
        fixed_point_t Signal[BEAT_LEN];
      #endif
      memset(BEAT.PCA, 0x00, PCA_SIZE*NO_PC);
      memset(BEAT.RR, 0x00, R_POS_SIZE);
    #endif   
    
    #endif

    bsp_board_init(BSP_INIT_LEDS);
    
    #if NRFX_CHECK(NRF_LOG_ENABLED)
      APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
      NRF_LOG_DEFAULT_BACKENDS_INIT();
    #endif

    #ifdef ADNAN_SPIM
      nrfx_spim_config_t spim_config = NRFX_SPIM_DEFAULT_CONFIG;
      spim_config.sck_pin   = SPI_SCK_PIN;
      spim_config.mosi_pin  = SPI_MOSI_PIN;
      spim_config.miso_pin  = SPI_MISO_PIN;
      spim_config.ss_pin    = SPI_SS_PIN;
      spim_config.orc       = 0x00;
      #ifndef UMFT4222H
      spim_config.frequency = NRF_SPIM_FREQ_250K;
      #else
      spim_config.frequency = NRF_SPIM_FREQ_250K;
      #endif
      spim_config.mode      = NRF_SPIM_MODE_1;
      APP_ERROR_CHECK(nrfx_spim_init(&spim, &spim_config, NULL, NULL));

      APP_ERROR_CHECK(ads1292r_init(&spim));

      nrfx_timer_config_t timer_cfg = NRFX_TIMER_DEFAULT_CONFIG;
      APP_ERROR_CHECK(nrfx_timer_init(&timer, &timer_cfg, timer_event_handler));
      nrfx_timer_extended_compare(&timer, NRF_TIMER_CC_CHANNEL0, ARRAY_LIST_SIZE, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
      nrfx_timer_enable(&timer);

      APP_ERROR_CHECK(nrfx_gpiote_init());
      nrfx_gpiote_in_config_t in_config = NRFX_GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
      APP_ERROR_CHECK(nrfx_gpiote_in_init(ADS1292R_DRDY_PIN, &in_config, empty_in_pin_handler));
      nrfx_gpiote_in_event_enable(ADS1292R_DRDY_PIN, false);

      APP_ERROR_CHECK(nrfx_ppi_channel_alloc(&ppi_channel));
      APP_ERROR_CHECK(nrfx_ppi_channel_assign(ppi_channel, nrfx_gpiote_in_event_addr_get(ADS1292R_DRDY_PIN), nrfx_spim_start_task_get(&spim)));
      APP_ERROR_CHECK(nrfx_ppi_channel_fork_assign(ppi_channel, nrfx_timer_task_address_get(&timer, NRF_TIMER_TASK_COUNT)));
      APP_ERROR_CHECK(nrfx_ppi_channel_enable(ppi_channel));
      
      #ifndef UMFT4222H
      nrfx_spim_uninit(&spim);
      #else
      nrfx_spim_uninit(&spim);
      #endif
    #endif

    bool erase_bonds;
    timers_init();
    power_management_init();
    
#ifdef SIVA_BLE
    //uart_init();  
    buttons_leds_init(&erase_bonds);  
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();

    sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
       
    #ifdef SIVA_BLE_AD
        advertising_start();
    #endif
#else
    NRF_POWER->DCDCEN = 1;
#endif

    #ifdef ADNAN_SPIM
      #ifndef UMFT4222H
        spim_config.frequency = NRF_SPIM_FREQ_8M;
        APP_ERROR_CHECK(nrfx_spim_init(&spim, &spim_config, empty_spim_event_handler, NULL));

        APP_ERROR_CHECK(ads1292r_command(&spim, ADS1292R_SPI_START));
        while (!*(volatile uint32_t *)nrfx_spim_end_event_get(&spim));

        APP_ERROR_CHECK(ads1292r_command(&spim, ADS1292R_SPI_RDATAC));
        while (!*(volatile uint32_t *)nrfx_spim_end_event_get(&spim));
      #else
        spim_config.frequency = NRF_SPIM_FREQ_8M;
        APP_ERROR_CHECK(nrfx_spim_init(&spim, &spim_config, empty_spim_event_handler, NULL));

        APP_ERROR_CHECK(ads1292r_command(&spim, ADS1292R_SPI_START));
        while (!*(volatile uint32_t *)nrfx_spim_end_event_get(&spim));

        APP_ERROR_CHECK(ads1292r_command(&spim, ADS1292R_SPI_RDATAC));
        while (!*(volatile uint32_t *)nrfx_spim_end_event_get(&spim));

        while (!*(volatile uint32_t *)nrfx_spim_end_event_get(&spim));
        while (!*(volatile uint32_t *)nrfx_spim_end_event_get(&spim));
        while (!*(volatile uint32_t *)nrfx_spim_end_event_get(&spim));
      #endif

      
      while (!*(volatile uint32_t *)nrfx_spim_end_event_get(&spim));
      APP_ERROR_CHECK(ads1292r_read_data(&spim));
    #endif

#ifdef SIVA_SERIAL
    nrf_gpio_cfg_output(TX_PIN_NUMBER);
    nrf_gpio_cfg_input(RX_PIN_NUMBER, GPIO_PIN_CNF_PULL_Disabled);

    ret_code_t ret = nrf_serial_init(&serial_uart, &m_uart0_drv_config, &serial_config);
    APP_ERROR_CHECK(ret);
   
    #ifndef SIVA_BLE_AD 
       nrf_delay_us(1000);
       ret = nrf_serial_write(&serial_uart,
                           tx_message,
                           strlen(tx_message),
                           NULL,
                           NRF_SERIAL_MAX_TIMEOUT);
        APP_ERROR_CHECK(ret);
    #endif
#endif

    #if NRFX_CHECK(NRF_LOG_ENABLED)
      // Start execution.
      //printf("\r\nUART started.\r\n");
      NRF_LOG_INFO("Debug logging for UART over RTT started.");
      NRF_LOG_FLUSH();
    #endif   
   
    for (;;)
    {
      
      #ifdef SIVA_SERIAL
        while (nrf_serial_read(&serial_uart, &cr, 4, NULL, 1000) != NRF_SUCCESS);
                 
        //(void)nrf_serial_write(&serial_uart, &cr, 4, NULL, 0);
        if (cr[3] == '\n'){
            
            if (cr[0] == '2'){
                beatcount = 0; 
                sampcount = 0;
                memset(Data_Buf, 0x00, DATA_BUFFER_SIZE);
                #if defined(SIVA_ANN) || defined(SIVA_RULE)
                  memset(BEAT.RR, 0x00, R_POS_SIZE);
                  memset(BEAT.PCA, 0x00, PCA_SIZE*NO_PC);
                #endif
                //printf("New set of data identified\n");
            }else if (cr[0] == '0'){
                //do nothing
            }else if (cr[0] == '1') {
                Rpos[beatcount % R_POS_SIZE] = sampcount;
                
                  #if defined(SIVA_ANN) || defined(SIVA_RULE)
                  for (int i = 0; i < R_POS_SIZE - 1; i++){  
                    BEAT.RR[i]  = BEAT.RR[i + 1];
                    //printf("%d, ", BEAT.RR[i]);                    
                  } 
                  
                  
                  for (int i = 0; i < PCA_SIZE - 1; i++){  
                    for (int j = 0; j < NO_PC; j++){
                        BEAT.PCA[i][j] = BEAT.PCA[i + 1][j];
                    }
                    //printf("%d, ", BEAT.PCA[i]);
                  }
                  
                  BEAT.RR[R_POS_SIZE-1] = (sampcount - Rpos[(beatcount-1) % R_POS_SIZE]) << (FIXED_POINT_FRACTIONAL_BITS - SamplingShift); 
                  //printf("%d\n", BEAT.RR[R_POS_SIZE-1]);                    
                  #endif

                
                if (beatcount > 0){

                 #if defined(SIVA_ANN) || defined(SIVA_RULE)
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
                              //for (int i = 0; i<BEAT_LEN-1; i++){
                                //printf("%d, ", maapstd[i]);
                              //}
                              //printf("%d\n", maapstd[BEAT_LEN-1]);
                          #if defined(SIVA_RULE)

                          //RR mean
                             BEAT.RRmean = mean(BEAT.RR, 11); 

                          //QRS Sum 0.05*Fs:0.05*Fs                     
                            BEAT.qrsSum = qrssum(maapstd, RLoc, band);
                            //printf("qrsSum : %d\n", BEAT.qrsSum);
                          
                          //QRS Energy 0.05*Fs:0.05*Fs                     
                            BEAT.qrsEnergy = qrsEnergy(maapstd, RLoc, band);
                            //printf("qrsEnergy : %d\n", BEAT.qrsEnergy); 

                          //SDNN: RR standard deviation
                            //float rrSTD;
                            //rrSTD = HRV_SDNN(RR_INT,10);
                            //printf("rrTSD/SDNN : %.4f\n", rrSTD); 

                          //HRV Test
                           BEAT.HRV.SDNN = HRV_SDNN(BEAT.RR,11);  
                           BEAT.HRV.SD1 = HRV_SD1(&BEAT.RR[7]);  
                           BEAT.HRV.SD2 = HRV_SD2(&BEAT.RR[7]);  
                            

                          //VVstd
                            //BEAT.prevvSTD = BEAT.vvSTD;
                            //BEAT.vvSTD = vvPeakSTD(maapstd, RLoc, band);
                            //printf("vvSTD : %d\n", BEAT.vvSTD); 

                          //PCA
                            BEAT.PCA[PCA_SIZE-1][0] = findPCAcoeff(maapstd, BEAT_LEN);
                            //printf("PCAcoeff: %d\n", BEAT.PCAcoef); 
                            //BEAT.PCA[PCA_SIZE-1][0] = BEAT.PCAcoef;      
                            //printf("%d\n", BEAT.PCA[PCA_SIZE-1]);          
                            //BEAT.PCAstd = PCAstd(BEAT.PCA, PCA_SIZE);  
                          #endif

                          #if defined(SIVA_ANN)
                          //PCA coefficients              
                            findPCAcoeffs(&BEAT.PCA[PCA_SIZE-1][0], maapstd, BEAT_LEN);
                          #endif

                          //if (beatcount > 10){  
                            //printf("%d, ",beatcount);
                            //(void)nrf_serial_write(&serial_uart, &beatcount, sizeof(int), NULL, 0);
                            //(void)nrf_serial_write(&serial_uart, &commma, 1, NULL, 0);
                             
                            //HRV Test
                              BEAT.HRV.wSDNN = weightedSDNN(10,BEAT.RR,9,11);
                              //BEAT.HRV.SDNN = HRV_SDNN(BEAT.RR,11);   
                              BEAT.HRV.rrIndex = HRV_RRIndex(&BEAT.RR[9]);
                            
                            #ifdef SIVA_RULE
                              //BEAT.Result = 49;
                              BEAT.Result = checkforArrhythmia_II(BEAT.HRV.wSDNN, BEAT.HRV.rrIndex,&BEAT.RR[8], BEAT.RRmean, BEAT.HRV.SD1, BEAT.HRV.SD2, BEAT.HRV.SDNN, &BEAT.PCA[PCA_SIZE-1][0], BEAT.PCAstd, BEAT.qrsSum, BEAT.qrsEnergy);
                              //BEAT.Result = checkforArrhythmia(BEAT.HRV.wSDNN,BEAT.HRV.rrIndex, &BEAT.RR[8], BEAT.qrsSum, BEAT.PCAcoef, BEAT.vvSTD, BEAT.prevvSTD, BEAT.HRV.SDNN, BEAT.PCAstd);
                              //BEAT.Result = check_wSDNN_RRindex(BEAT.HRV.wSDNN,BEAT.HRV.rrIndex);
                              //printf("%d, %d, %d, %d, %d, %d, %d, %d, %d,", beatcount, BEAT.HRV.wSDNN, BEAT.HRV.SDNN, BEAT.HRV.rrIndex, BEAT.qrsSum, BEAT.vvSTD, BEAT.PCAcoef, BEAT.PCAstd, BEAT.Result);                   
                    
                              //BEAT.Result = checkforTsipouras(BEAT.RR[8], BEAT.RR[9], BEAT.RR[10]);
                              //printf("%d, ",BEAT.Result);

                              //BEAT.Result = checkforExarchos(BEAT.RR[8], BEAT.RR[9], BEAT.RR[10]);
                              //printf("%d\n",BEAT.Result);
                            #endif
                        
                            //ANN EXECUTION
                            #if defined(SIVA_ANN)
                              fixed_point_ann_t Ain1[5] = {(BEAT.RR[9]>>SHIFT_ANN_NORM_BIT), (BEAT.RR[10]>>SHIFT_ANN_NORM_BIT), (mean(maapstd,BEAT_LEN)>>SHIFT_ANN_NORM_BIT), (BEAT.HRV.wSDNN>>SHIFT_ANN_NORM_BIT), (BEAT.HRV.rrIndex>>SHIFT_ANN_NORM_BIT)};
                              fixed_point_ann_t Ain2[5][6] = {0};
                              for (int i = 0; i < PCA_SIZE; i++){  
                                for (int j = 0; j < NO_PC; j++){
                                    Ain2[i][j] = BEAT.PCA[i][j]>>SHIFT_ANN_NORM_BIT;
                                  }
                                  //printf("%d, ", BEAT.PCA[i]);
                                }
                        
                              ann.predict(&ann,Aout,Ain2,Ain1); 
                        
                              if (Aout[0] > Aout[1]) BEAT.Result = 48;   //48 is 0: Normal Beat
                              else BEAT.Result = 49;                     //49 is 1: Abvnormal Beat
                            #endif

                              if (BEAT.Result == 49){
                                  sendBeat=true;
                                  #ifdef BLE_ALERT_1BYTE
                                    ble_siva_ann_alert(BEAT.Result);   
                                  #endif
                              }
                            #else
                                 BEAT.Result = 49;                      //49 is 1: Abvnormal Beat
                            #endif                     

                        }else{
                  
                          #if defined(SIVA_ANN) || defined(SIVA_RULE)
                          for (int i = R_POS_SIZE - 2; i >= 0; i--){  
                            BEAT.RR[i]  = BEAT.RR[i + 1];
                            
                            //printf("%d, ", BEAT.RR[i]);
                            }
                            
                          #endif
                  
                        }

                      beatcount++;                     
                
            } 
            
            //Store read data to the ring buffer 
            Data_Buf[sampcount % DATA_BUFFER_SIZE] = cr[1] << BYTE_LEN;
            Data_Buf[sampcount % DATA_BUFFER_SIZE] = Data_Buf[sampcount % DATA_BUFFER_SIZE] | cr[2];

            #ifdef BLE_ALL_SAMPLES
            //ble_sample_send(cr);
            if (sampcount%120==119){
               ble_sample_signal_send(Data_Buf, sampcount % DATA_BUFFER_SIZE, 240);
            }
            #endif
            
            #ifdef BLE_ALERT_ECGSAMPLE
            if (BEAT.Result == 49 && sendBeat==true){
                ble_sample_signal_send(Signal, 0,240);
                ble_sample_signal_send(Signal, 0,110);
                sendBeat = false;
            }
            #endif

            sampcount++;
        }       
        #endif

        #ifndef SIVA_BLE
          __WFI();
        #else
          idle_state_handle();
        #endif
        
    }

    while (1)
    {
        __WFI();
    }
}





// while (1)
//    {
//        __WFI();
//
////        NRF_LOG_FLUSH();
//
////        NRF_LOG_RAW_INFO("spki = %d, npki = %d, threshold_i1 = %d, threshold_i2 = %d\n", spki, npki, threshold_i1, threshold_i2);
////        NRF_LOG_RAW_INFO("spkf = %d, npkf = %d, threshold_f1 = %d, threshold_f2 = %d\n", spkf, npkf, threshold_f1, threshold_f2);
///*
//        for (index = 0; index < BUFF; ++index)
//        {
//            NRF_LOG_RAW_INFO("}~");
//            NRF_LOG_RAW_INFO("%c%c%c%c", ECG_signal[index] >> 24, ECG_signal[index] >> 16, ECG_signal[index] >> 8, ECG_signal[index]);
//            //NRF_LOG_RAW_INFO("%c%c%c%c", low_pass_filter[index] >> 24, low_pass_filter[index] >> 16, low_pass_filter[index] >> 8, low_pass_filter[index]);
//            //NRF_LOG_RAW_INFO("%c%c%c%c", high_pass_filter[index] >> 24, high_pass_filter[index] >> 16, high_pass_filter[index] >> 8, high_pass_filter[index]);
//            //NRF_LOG_RAW_INFO("%c%c%c%c", derivative[index] >> 24, derivative[index] >> 16, derivative[index] >> 8, derivative[index]);
//            //NRF_LOG_RAW_INFO("%c%c%c%c", squaring_function[index] >> 24, squaring_function[index] >> 16, squaring_function[index] >> 8, squaring_function[index]);
//            //NRF_LOG_RAW_INFO("%c%c%c%c", moving_window_integration[index] >> 24, moving_window_integration[index] >> 16, moving_window_integration[index] >> 8, moving_window_integration[index]);
//            NRF_LOG_RAW_INFO("%c%c%c%c", qrs[index] >> 24, qrs[index] >> 16, qrs[index] >> 8, qrs[index]);
//            NRF_LOG_RAW_INFO("\n");
//
//            NRF_LOG_FLUSH();
//        }
///*
//        if (memory_full)
//        {
//            memory_full = false;
//
//            for (index = 0; index < MEMORY_SIZE_ROW; ++index)
//            {
//                NRF_LOG_RAW_INFO("~%c%c%c%c", (int8_t)memory[index][0] >> 7, memory[index][0], memory[index][1], memory[index][2]);
//                NRF_LOG_RAW_INFO("%c%c%c%c\n", (int8_t)memory[index][3] >> 7, memory[index][3], memory[index][4], memory[index][5]);
//                NRF_LOG_FLUSH();
//            }
//        }
//*/
//    }