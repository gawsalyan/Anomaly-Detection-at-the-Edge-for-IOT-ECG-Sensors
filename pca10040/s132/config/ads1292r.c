#include "ads1292r.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include <string.h>
//#include "nrf_log.h"

static ret_code_t result             = 0;
static uint8_t        tx_buf[3]      = {0};
static uint8_t        rx_buf[3]      = {0};

spim_array_list_t array_list[ARRAY_LIST_SIZE];

static nrfx_spim_xfer_desc_t spim_xfer_desc =
{
    .p_tx_buffer = tx_buf,
    .tx_length   = 0,
    .p_rx_buffer = rx_buf,
    .rx_length   = 0,
};

ret_code_t ads1292r_init(nrfx_spim_t const * const p_instance)
{
    ret_code_t result = 0;
    

    nrf_gpio_cfg_output(ADS1292R_PWDN_RESET_PIN);
    #ifndef UMFT4222H
    nrf_gpio_pin_clear(ADS1292R_PWDN_RESET_PIN);
    nrf_delay_ms(50);
    #endif
    nrf_gpio_cfg_input(ADS1292R_DRDY_PIN, 0);

    //nrf_gpio_pin_set(ADS1292R_PWDN_RESET_PIN);
    //nrf_delay_ms(50);
    #ifndef UMFT4222H
    nrf_gpio_pin_clear(ADS1292R_PWDN_RESET_PIN);
    nrf_delay_ms(1);
    nrf_gpio_pin_set(ADS1292R_PWDN_RESET_PIN);
    nrf_delay_ms(1);  

    result = ads1292r_command(p_instance, ADS1292R_SPI_SDATAC);  //ADS1292R_SPI_SDATAC

    result = ads1292r_write_register(p_instance, ADS1292R_SPI_REG_CONFIG2, 0xA0);
    nrf_delay_ms(200);

    result = ads1292r_write_register(p_instance, ADS1292R_SPI_REG_CONFIG1, 0x01);
//    result = ads1292r_write_register(p_instance, ADS1292R_SPI_REG_CONFIG1, 0x03);
    result = ads1292r_write_register(p_instance, ADS1292R_SPI_REG_RLD_SENS, 0x2C);
    result = ads1292r_write_register(p_instance, ADS1292R_SPI_REG_RESP1, 0xC2);
    result = ads1292r_write_register(p_instance, ADS1292R_SPI_REG_RESP2, 0x83);

    result = ads1292r_command(p_instance, ADS1292R_SPI_OFFSETCAL);
    
    if (spim_xfer_desc.p_rx_buffer[2] != 0x73)
        return NRFX_ERROR_INVALID_STATE;
    
    #endif  
    
    #ifdef UMFT4222H
    result = ads1292r_command(p_instance, ADS1292R_SPI_OFFSETCAL);
    nrf_delay_ms(200);
    result = ads1292r_read_register(p_instance, ADS1292R_SPI_REG_ID);
    nrf_delay_ms(200);
    result = ads1292r_read_register(p_instance, ADS1292R_SPI_REG_ID);
    nrf_delay_ms(200);

    if (spim_xfer_desc.p_rx_buffer[2] != 0x08)
        return NRFX_ERROR_INVALID_STATE;
    #endif

    return result;
}

ret_code_t ads1292r_command(nrfx_spim_t const * const p_instance,   uint8_t command)
{
    tx_buf[0] = command;
    spim_xfer_desc.tx_length = 1;
    spim_xfer_desc.rx_length = 0;

    result = nrfx_spim_xfer(p_instance, &spim_xfer_desc, NRFX_SPIM_FLAG_NO_XFER_EVT_HANDLER);
/*
    NRF_LOG_INFO("ads1292r_command");

    NRF_LOG_INFO(" Transfered:");
    NRF_LOG_HEXDUMP_INFO(spim_xfer_desc.p_tx_buffer, 1);
*/
    return result;
}

ret_code_t ads1292r_write_register(nrfx_spim_t const * const p_instance,
                                   uint8_t                   address,
                                   uint8_t                   data)
{
    tx_buf[0] = ADS1292R_SPI_WREG | address;
    tx_buf[1] = 0;
    tx_buf[2] = data;
    spim_xfer_desc.tx_length = 3;
    spim_xfer_desc.rx_length = 0;

    result = nrfx_spim_xfer(p_instance, &spim_xfer_desc, NRFX_SPIM_FLAG_NO_XFER_EVT_HANDLER);
/*
    NRF_LOG_INFO("ads1292r_write_register");

    NRF_LOG_INFO(" Transfered:");
    NRF_LOG_HEXDUMP_INFO(spim_xfer_desc.p_tx_buffer, 3);
*/
    return result;
}

ret_code_t ads1292r_read_register(nrfx_spim_t const * const p_instance,
                                  uint8_t                   address)
{
          
    #ifndef UMFT4222H
    tx_buf[0] = ADS1292R_SPI_RREG | address;
    tx_buf[1] = 0;
    memset(spim_xfer_desc.p_rx_buffer, 0, 3);
    spim_xfer_desc.tx_length = 2;
    spim_xfer_desc.rx_length = 3;
    #else
    memset(spim_xfer_desc.p_rx_buffer, 0, 1);
    tx_buf[0] = ADS1292R_SPI_RREG; //SYNC_WORD 0x5A
    tx_buf[1] = ADS1292R_SPI_RREG; //CMD
    //tx_buf[2] = 0x02; //SiZE
    spim_xfer_desc.tx_length = 1;
    spim_xfer_desc.rx_length = 3;
    #endif
    result = nrfx_spim_xfer(p_instance, &spim_xfer_desc, NRFX_SPIM_FLAG_NO_XFER_EVT_HANDLER);
/*
    NRF_LOG_INFO("ads1292r_read_register");

    NRF_LOG_INFO(" Transfered:");
    NRF_LOG_HEXDUMP_INFO(spim_xfer_desc.p_tx_buffer, 2);

    NRF_LOG_INFO(" Received:");
    NRF_LOG_HEXDUMP_INFO(spim_xfer_desc.p_rx_buffer, 3);
*/
    return result;
}

ret_code_t ads1292r_read_data(nrfx_spim_t const * const p_instance)
{
    //while (!*(volatile uint32_t *)nrfx_spim_end_event_get(p_instance));
    #ifndef UMFT4222H
    spim_xfer_desc.tx_length = 0;
    spim_xfer_desc.p_rx_buffer = array_list[0].buffer;
    spim_xfer_desc.rx_length = BUFFER_SIZE;
    #else
    //tx_buf[0] = 0x11;
    spim_xfer_desc.tx_length = 0;
    spim_xfer_desc.p_rx_buffer = array_list[0].buffer;
    spim_xfer_desc.rx_length = BUFFER_SIZE;
    #endif
    

    result = nrfx_spim_xfer(p_instance, &spim_xfer_desc, NRFX_SPIM_FLAG_RX_POSTINC | NRFX_SPIM_FLAG_NO_XFER_EVT_HANDLER | NRFX_SPIM_FLAG_HOLD_XFER | NRFX_SPIM_FLAG_REPEATED_XFER);
/*
    NRF_LOG_RAW_INFO("~%c%c%c%c", (int8_t)spim_xfer_desc.p_rx_buffer[3] >> 7, spim_xfer_desc.p_rx_buffer[3], spim_xfer_desc.p_rx_buffer[4], spim_xfer_desc.p_rx_buffer[5]);
    NRF_LOG_RAW_INFO("%c%c%c%c\n", (int8_t)spim_xfer_desc.p_rx_buffer[6] >> 7, spim_xfer_desc.p_rx_buffer[6], spim_xfer_desc.p_rx_buffer[7], spim_xfer_desc.p_rx_buffer[8]);
*/
    return result;
}