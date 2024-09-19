/*
 * haha_v3_xmega.h
 *
 * HaHa v3.0 Board
 *  Author: Reiner Dizon-Paradis
 */

#ifndef HAHA_H
#define HAHA_H

#include "avr_compiler.h"
// #include "spi_driver.h" // Uncomment if using SPI
#include <avr/io.h>

/* Pin Definitions for Chip Interconnection - Refer to the HaHa Manual */
#define HAHA_CM_PORT PORTA
#define HAHA_CLK_INTER_PORT PORTC
#define HAHA_CLK_INTER_PIN PIN0_bm
#define HAHA_CLK_INTER_DELAY 25

/* Function Prototypes */
void haha_interBegin(void);
void haha_clkInterPos(void);
void haha_clkInterNeg(void);
void haha_sendDataToFPGA(uint8_t data);

void haha_uart_init();
void haha_uart_print_char(char c);
void haha_uart_print_str(const char *str);
void haha_uart_print_u8_hex(uint8_t num);
void haha_uart_print_u16_hex(uint16_t num);
void haha_uart_print_u32_hex(uint32_t num);

// This function return the ADC temperature callibration value
// The ADC temperature sensor is designed to read 0 at 0K
// The value returned by this function is the ADC value at 85C (358K)
// Use linear interpolation to convert ADC value to temperature
uint16_t haha_get_adc_temp_callibration(void);

// Must be called before using the ADC to read temperature
void haha_config_adc_for_temperature(void);

// Read the ADC value (used for temperature)
uint16_t haha_read_adc(void);

/* SPI Declarations */
#ifdef SPI_DRIVER_H

/* Global variables */
SPI_Master_t spiMasterC;

/* Pin Definitions for SS for W25N Flash */
#define W25N_SS_PORT PORTC
#define W25N_SS_PIN PIN4_bm
#define W25N_ss_en() SPI_MasterSSLow(&W25N_SS_PORT, W25N_SS_PIN)
#define W25N_ss_di() SPI_MasterSSHigh(&W25N_SS_PORT, W25N_SS_PIN)

/* Pin Definitions for Flash HOLD_N pin/port */
#define W25N_HOLD_N_PORT PORTE
#define W25N_HOLD_N_PIN PIN1_bm

/* Pin Definitions for Flash WP_N pin/port */
#define W25N_WP_N_PORT PORTE
#define W25N_WP_N_PIN PIN2_bm

/* Pin Definitions for SS for MC3635 Accelerometer */
#define MC3635_SS_PORT PORTE
#define MC3635_SS_PIN PIN3_bm
#define MC3635_ss_en() SPI_MasterSSLow(&MC3635_SS_PORT, MC3635_SS_PIN)
#define MC3635_ss_di() SPI_MasterSSHigh(&MC3635_SS_PORT, MC3635_SS_PIN)

/* Function Prototypes */
void haha_v3_SPIBegin(void);

#endif

#endif /* HAHA_H */
