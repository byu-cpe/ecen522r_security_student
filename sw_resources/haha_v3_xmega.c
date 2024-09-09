/*
 * haha_v3_xmega.c
 *
 * HaHa v3.0 Board
 *  Author: Reiner Dizon-Paradis, Jeff Goeders
 */

#include "haha_v3_xmega.h"

#include <stddef.h>

uint8_t read_calibration_byte(uint8_t index);

/* Set up chip interconnection as outputs */
void haha_v3_interBegin(void) {
  HAHA_V3_CM_PORT.DIRSET = 0xFF; // set all PORTA as output
  HAHA_V3_CM_PORT.OUT = 0x00;    // set PORTA to all 0's
  HAHA_V3_CLK_INTER_PORT.DIRSET = HAHA_V3_CLK_INTER_PIN; // set PC0 as output
  haha_v3_clkInterNeg(); // set to falling edge of clock
}

/* Send a rising edge for the Chip interconnection clock */
void haha_v3_clkInterPos(void) {
  HAHA_V3_CLK_INTER_PORT.OUTSET = HAHA_V3_CLK_INTER_PIN;
  _delay_ms(HAHA_V3_CLK_INTER_DELAY);
}

/* Send a falling edge for the Chip interconnection clock */
void haha_v3_clkInterNeg(void) {
  HAHA_V3_CLK_INTER_PORT.OUTCLR = HAHA_V3_CLK_INTER_PIN;
  _delay_ms(HAHA_V3_CLK_INTER_DELAY);
}

/* Send a byte data to the FPGA */
void haha_v3_sendDataToFPGA(uint8_t data) {
  HAHA_V3_CM_PORT.OUT = data;
  haha_v3_clkInterPos(); // rising edge
  haha_v3_clkInterNeg(); // falling edge
}

uint8_t read_calibration_byte(uint8_t index) {
  uint8_t result;
  /* Load the NVM Command register to read the calibration row. */
  NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc;
  result = pgm_read_byte(index);
  /* Clean up NVM Command register. */
  NVM_CMD = NVM_CMD_NO_OPERATION_gc;
  return (result);
}

uint16_t haha_v3_get_adc_temp_callibration(void) {
  return (read_calibration_byte(offsetof(NVM_PROD_SIGNATURES_t, TEMPSENSE1))
          << 8) |
         read_calibration_byte(offsetof(NVM_PROD_SIGNATURES_t, TEMPSENSE0));
}

void haha_v3_config_adc_for_temperature(void) {
  // Configure the ADC to use the internal 1V reference
  ADCA.REFCTRL = ADC_REFSEL_INT1V_gc | ADC_TEMPREF_bm;

  // 12bit resolution
  ADCA.CTRLB = ADC_RESOLUTION_12BIT_gc;

  // ADC clock must be 62.5kHz, 32MHz / 512 = 62.5kHz
  ADCA.PRESCALER = ADC_PRESCALER_DIV512_gc;

  // 1:1 gain with internal positive reading
  ADCA.CH0.CTRL = ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_INTERNAL_gc;

  // Configure mux to read from temperature sensor
  ADCA.CH0.MUXCTRL = ADC_CH_MUXINT_TEMP_gc;

  // Configure the ADC settings (8-bit or 12-bit mode, etc.)
  ADCA.CTRLA |= ADC_ENABLE_bm;
//   sei();

  // Initiate a dummy reading
  ADCA.CH0.CTRL |= ADC_CH_START_bm;

  // Wait for result to be converted
  while (!(ADCA.INTFLAGS & ADC_CH0IF_bm))
    ;

  // Clear interrupt bit
  ADCA.INTFLAGS = ADC_CH0IF_bm;
}

uint16_t haha_v3_read_adc(void) {
  // Initiate reading
  ADCA.CH0.CTRL |= ADC_CH_START_bm;

  // Wait for the conversion to complete
  while (!(ADCA.INTFLAGS & ADC_CH0IF_bm))
    ;

  // Clear the interrupt flag
  ADCA.CH0.INTFLAGS = ADC_CH_CHIF_bm;

  // Read and return the result
  return ADCA.CH0.RES;
}

/* SPI Functions */
#ifdef SPI_DRIVER_H
void haha_v3_SPIBegin(void) {
  /* Initialize SPI master on port C */
  SPI_MasterInit(&spiMasterC, &SPIC, &PORTC, false, SPI_MODE_0_gc,
                 SPI_INTLVL_OFF_gc, false, SPI_PRESCALER_DIV4_gc);
}

/* Add your own function that initializes SS (HOLD_N, WP_N) ports as needed for
 * selected SPI device. It should called before SPI operations - Example below:
 */

/* Init SS pin as output with wired AND and pull-up. */
// MC3635_SS_PORT.DIRSET = MC3635_SS_PIN;
// MC3635_SS_PORT.PIN4CTRL = PORT_OPC_WIREDANDPULL_gc;

/* Set SS output to high. (No slave addressed). */
// MC3635_SS_PORT.OUTSET = MC3635_SS_PIN;

#endif
