/*
 * E-sky Hobby ET6I tx and rx.
 *
 * Licensed under AGPLv3.
 */

#include <avr/io.h>
#include <avr/interrupt.h>

#include "timer1.h"

volatile uint16_t rx_ch[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, };
volatile uint8_t rx_co_throttle = 0;
volatile uint8_t rx_co_right = 0;
volatile uint8_t rx_cy_front = 0;
volatile uint8_t rx_cy_right = 0;
volatile uint8_t rx_gyro_sw = 0;
volatile uint8_t rx_right_pot = 0;
volatile uint8_t rx_left_pot = 0;
volatile uint8_t rx_no_signal = 0;

#if 0

/* Rx sum signal connected to the PD2/INT0 pin.  */
void rx_init(void) {
	EICRA = 0x01;
	EIMSK = 0x01;
	DDRD &= ~0x04;
}

ISR(INT0_vect) {
	/* Timer1 overflows every >4ms, so we should be fine using TCNT1 raw */
	uint16_t now = TCNT1;

	if (PIND & 0x04) {
		if (now - rx_up > 40000)
			rx_chnum = 0;
		rx_up = now;
		return;
	}

	rx_ch[rx_chnum ++] = now - rx_up;
	if (rx_chnum >= sizeof(rx_ch) / sizeof(*rx_ch))
		rx_chnum = 0;
}
#else

/* Individual rx outputs connected to PB0, PB1, PB2, PB3 (and so on)
 * Note: only unmask interrupts for the pins actually connected to Rx.  */
void rx_init(void) {
	DDRB &= ~0x0f;
	PCMSK0 = 0x0f;
	PCICR |= 0x01;
}

static inline void rx_esky_update(void) {
	uint16_t up_raw = rx_ch[2] >= ((256 + 31) << 6) ?
		((256 + 31) << 6) - 1 : rx_ch[2];
#ifdef FIRST_4_CHANNELS_ONLY
	uint16_t back_raw = rx_ch[1] -
		(rx_ch[2] >> 2) - (rx_ch[2] >> 4);
	uint16_t left_raw = rx_ch[0] + (back_raw >> 1) -
		(rx_ch[2] >> 2) - (rx_ch[2] >> 4);

	static const uint8_t rx_esky_cy_f_bias[16] =
		{ 4, 4, 3, 3, 3, 2, 2, 1, 2, 3, 4, 6, 6, 6, 6, 5, };
	static const uint8_t rx_esky_cy_r_bias[16] =
		{ 5, 5, 6, 7, 7, 8, 8, 9, 9, 6, 3, 2, 1, 1, 2, 3, };

	rx_co_throttle = (up_raw >> 6) - 31;
	rx_co_right = (rx_ch[3] >> 6) - 10;
	rx_cy_front = ((~back_raw) >> 6) -
		rx_esky_cy_f_bias[rx_co_throttle >> 4];
	rx_cy_right = ((~left_raw) >> 6) + 61 +
		rx_esky_cy_r_bias[rx_co_throttle >> 4];
#else
	rx_co_throttle = (up_raw >> 6) - 31;
	rx_co_right = (rx_ch[3] >> 6) - 10;
	rx_cy_front = ((rx_ch[0] - rx_ch[1] * 2 - rx_ch[5]) >> 8) - 0x3a;
	rx_cy_right = (0x800 - rx_ch[5] - rx_ch[0]) >> 7;

	rx_gyro_sw = rx_ch[4] < 0x2300;
	rx_right_pot = (uint16_t) ((rx_ch[4] - 0x0300) & 0x1fff) >> 5;
	if (rx_ch[4] >= 0x4300)
		rx_right_pot = 0xff;
	/* TODO */
	rx_left_pot = (rx_ch[0] + rx_ch[1] - rx_ch[5] - (rx_ch[2] >> 1)) >> 8;
#endif

	rx_no_signal = 0;
}

ISR(PCINT0_vect) {
	static uint32_t rx_up = 0;
	static uint8_t rx_chnum = 0;
	uint32_t now = timer_read();

	/* Note: could even just use TCNT1 above at the timer frequency
	 * of 16MHz because the timer overflows at about 4ms only, but
	 * then we might miss the long pause which might be 10ms or longer.
	 * May be fixable though.
	 */

	/* The E-sky receiver starts pulsing the next channel almost
	 * immediately before the falling edge of the previous channel
	 * pulse, so the edges come about 0.01 ms apart only and
	 * sometimes the second interrupts gets missed.  Since they are
	 * so close we choose to unconditionally ignore the second one,
	 * because the first one stands a better chance of being reported
	 * accurately by the AVR core.  We ignore any edges < 0.5ms since
	 * the previous edge.
	 */

	if ((uint32_t) (now - rx_up) > F_CPU / 400)
		rx_chnum = 0;
	else if ((uint32_t) (now - rx_up) < F_CPU / 2000)
		return;
	else {
		/* XXX: should use F_CPU below */
		rx_ch[rx_chnum ++] = now - rx_up - F_CPU / 1050;

		/* Un-mix the channels.  The ET6I trasmitter is made
		 * specifically for helicopters and mixes the channels on
		 * the tx side to produce signals for: the motor, the 4
		 * servos and the gyro.  We need to perform some simple
		 * arithmetics to go back to the raw stick position values
		 * for any processing in the autopilot.  This is currently
		 * assuming the Idle Up setting ("aerobatic mode") is off.
		 */
#ifdef FIRST_4_CHANNELS_ONLY
		if (rx_chnum == 4)
#else
		if (rx_chnum == 6)
#endif
			rx_esky_update();
	}

	rx_up = now;
}
#endif
