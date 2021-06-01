#ifndef ADS1292R_H
#define ADS1292R_H

#include <nrfx_spim.h>

#define UMFT4222H

//==========================================================
// <h> ADS1292R_CONFIGURATION - ADS1292R configuration

//==========================================================
// <o> ADS1292R_PWDN_RESET_PIN  - Pin number
 
#define ADS1292R_PWDN_RESET_PIN 3

// <o> ADS1292R_DRDY_PIN  - Pin number
 
#define ADS1292R_DRDY_PIN 4

// </h> 
//==========================================================

// <h> ADS1292R_SPI_COMMANDS - ADS1292R Spi commands

//==========================================================
// <o> System Commands
 
#define ADS1292R_SPI_WAKEUP 0x02
#define ADS1292R_SPI_STANDBY 0x04
#define ADS1292R_SPI_RESET 0x06
#define ADS1292R_SPI_START 0x08
#define ADS1292R_SPI_STOP 0x0A
#define ADS1292R_SPI_OFFSETCAL 0x1A

// <o> Data Read Commands
 
#define ADS1292R_SPI_RDATAC 0x10
#define ADS1292R_SPI_SDATAC 0x11
#define ADS1292R_SPI_RDATA 0x12

// <o> Register Read Commands

#define ADS1292R_SPI_RREG 0x20
#define ADS1292R_SPI_WREG 0x40

// <o> Registers

#define ADS1292R_SPI_REG_ID 0x00
#define ADS1292R_SPI_REG_CONFIG1 0x01
#define ADS1292R_SPI_REG_CONFIG2 0x02
#define ADS1292R_SPI_REG_LOFF 0x03
#define ADS1292R_SPI_REG_CH1SET 0x04
#define ADS1292R_SPI_REG_CH2SET 0x05
#define ADS1292R_SPI_REG_RLD_SENS 0x06
#define ADS1292R_SPI_REG_LOFF_SENS 0x07
#define ADS1292R_SPI_REG_LOFF_STAT 0x08
#define ADS1292R_SPI_REG_RESP1 0x09
#define ADS1292R_SPI_REG_RESP2 0x0A
#define ADS1292R_SPI_REG_GPIO 0x0B

// </h> 
//==========================================================

#ifndef UMFT4222H
#define ARRAY_LIST_SIZE 1000
#else
#define ARRAY_LIST_SIZE 480
#endif
#define BUFFER_SIZE 9

typedef struct
{
    uint8_t buffer[BUFFER_SIZE];
} spim_array_list_t;

extern spim_array_list_t array_list[ARRAY_LIST_SIZE];

ret_code_t ads1292r_init(nrfx_spim_t const * const p_instance);

ret_code_t ads1292r_command(nrfx_spim_t const * const p_instance,
                            uint8_t                   command);

ret_code_t ads1292r_write_register(nrfx_spim_t const * const p_instance,
                                   uint8_t                   address,
                                   uint8_t                   data);

ret_code_t ads1292r_read_register(nrfx_spim_t const * const p_instance,
                                  uint8_t                   address);

ret_code_t ads1292r_read_data(nrfx_spim_t const * const p_instance);

#endif //ADS1292R_H