/*
 * ADC.
 */

#include <avr/io.h>

static uint8_t admux = 0x00;
static uint8_t adcsra = 0x85; /* Timing */
void adc_init(void) {
	ADMUX = admux;
	ADCSRA = adcsra;
	DIDR0 = 0x3f;
}

uint16_t adc_convert(uint8_t input) {
	uint16_t ret;

	ADMUX = admux | input;
	ADCSRA = adcsra | (1 << ADSC);

	while((~ADCSRA) & (1 << ADIF));

	ADCSRA = adcsra | (1 << ADIF);

	ret = ADCL;
	return ret | (ADCH << 8);
}
