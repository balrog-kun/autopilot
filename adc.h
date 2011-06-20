/*
 * ADC.
 */

void adc_init(void);
uint16_t adc_convert(uint8_t input);

void adc_convert_all(void (*finished)(void));
extern volatile uint16_t adc_values[5];
