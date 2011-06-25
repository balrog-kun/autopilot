/*
 * ADC.
 *
 * Licensed under AGPLv3.
 */

#include <avr/io.h>
#include <avr/interrupt.h>

static uint8_t admux = 0x00;
static uint8_t adcsra = 0x87; /* Timing */
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

/*
 * Currenty the inputs connected are:
 * 0: Gyro 4x OUT X
 * 1: Gyro 4x OUT Y
 * 2: Gyro OUT REF
 * 3: Battery voltage
 * 8: Temperature
 *
 * The first three are converted two times each using the external 3.3V
 * reference voltage.  Battery may be better with the internal 5V voltage
 * (REFS1[1:0] = 1), arduino contains the required capacitor on-board.
 */
volatile uint16_t adc_values[5];
static volatile uint8_t adc_ch;
static uint8_t adc_ch_num[8] = { 0, 1, 2, 3, 8, 0, 1, 2 };
static void (*adc_finished)(void);

/* Should take about 0.92ms at F_CPU of 16MHz and prescaler of 128 */
void adc_convert_all(void (*finished)(void)) {
	ADMUX = admux | 0;
	ADCSRA = adcsra | (1 << ADIE) | (1 << ADSC);
	adc_ch = 0;
	adc_finished = finished;
}

ISR(ADC_vect) {
	if (adc_ch < 5) {
		adc_values[adc_ch] = ADC;
		if (adc_ch == 3) {
			ADMUX = admux | 0xc8;
			ADCSRA = adcsra | (1 << ADIE) | (1 << ADSC);
			adc_ch ++;
			return;
		}
	} else {
		adc_values[adc_ch - 5] += ADC;
		if (adc_ch == 7) {
			sei();
			return adc_finished();
		}
	}
	ADMUX = admux | adc_ch_num[++ adc_ch];
	ADCSRA = adcsra | (1 << ADIE) | (1 << ADSC);
}
