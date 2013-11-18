/*
 * LPC1343 ADC routines.
 *
 * Licensed under AGPLv3.
 */

#include "LPC13xx.h"		/* LPC13xx Peripheral Registers */

static void adc_init(void) {
	/* Disable the pull-ups, they seem to cause the Rx to detect a jumper */
	LPC_IOCON->PIO1_11 = 0x41;

	LPC_SYSCON->SYSAHBCLKCTRL |= 1 << 13;
	LPC_SYSCON->PDRUNCFG &= ~(1 << 4);

	my_delay(10);
	LPC_ADC->INTEN = 0;
	/* Start auto-repeated conversions on AD7 */
	LPC_ADC->CR = (1 << 7) | (0xff << 8) | (1 << 16);
}

static uint16_t adc_convert(uint8_t input) {
	return ((&LPC_ADC->DR0)[input] >> 6) & 0x3ff;
}
