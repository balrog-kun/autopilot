/*
 * E-sky Hobby ET6I tx and rx.
 *
 * Licensed under AGPLv3.
 */

#include "LPC13xx.h"		/* LPC13xx Peripheral Registers */

#include "timer1.h"

#define CHANNELS 5

volatile uint16_t rx_ch[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, };
volatile uint8_t rx_co_throttle = 0;
volatile uint8_t rx_co_right = 0x80;
volatile uint8_t rx_cy_front = 0x80;
volatile uint8_t rx_cy_right = 0x80;
volatile uint8_t rx_gyro_sw = 0;
volatile uint8_t rx_right_pot = 0;
volatile uint8_t rx_left_pot = 0;
volatile uint8_t rx_gear_sw = 0;
volatile uint8_t rx_id_sw = 0;
volatile uint8_t rx_no_signal = 0;

/* Individual rx outputs connected to PIO1_5 PIO1_8 (PIO1_9) PIO2_0
 * Note: only unmask interrupts for the pins actually connected to Rx.  */
void rx_init(void) {
	/* all reset defaults, other than: */
	LPC_GPIO1->IBE = (1 << 5) | (1 << 8);
	LPC_GPIO2->IBE = (1 << 0);
	LPC_GPIO1->IE = (1 << 5) | (1 << 8);
	LPC_GPIO2->IE = (1 << 0);
	/* we even leave the pull-ups enabled just in case */
	NVIC_EnableIRQ(EINT1_IRQn);
	NVIC_EnableIRQ(EINT2_IRQn);
}

#if 0
#define AIRPLANE_MODE

static inline void rx_esky_update(void) {
#ifdef AIRPLANE_MODE
	static unsigned int ch4_filter, ch4_cnt;
	rx_co_throttle = rx_ch[2] > 0x600 ? rx_ch[2] < 0x4600 ?
		(rx_ch[2] - 0x600) >> 6 : 0xff : 0;
	rx_co_right    = rx_ch[3] > 0x300 ? rx_ch[3] < 0x4300 ?
		(rx_ch[3] - 0x300) >> 6 : 0xff : 0;
	rx_cy_front    = (0x4400 - rx_ch[1]) >> 6;
	rx_cy_right    = (0x4500 - rx_ch[0]) >> 6;
	rx_left_pot    = (0x4400 - rx_ch[5]) >> 6;
	/* Try to counteract the noise close to 0x2280 */
	ch4_cnt ++;
	ch4_cnt &= 7;
	if (rx_ch[4] > 0x2280)
		ch4_filter |= 1 << ch4_cnt;
	else
		ch4_filter &= ~(1 << ch4_cnt);
	if (ch4_filter == 0x00 || ch4_filter == 0xff) {
		rx_right_pot   = ((rx_ch[4] - 0x0300) & 0x1fff) >> 5;
		if (rx_ch[4] <= 0x0300)
			rx_right_pot = 0x00;
		if (rx_ch[4] >= 0x4300)
			rx_right_pot = 0xff;
		rx_gyro_sw     = rx_ch[4] < 0x2280;
	}
#else
	uint16_t up_raw = rx_ch[2] >= ((256 + 31) << 6) ?
		((256 + 31) << 6) - 1 : rx_ch[2];
# if CHANNELS == 4
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
# else
	rx_co_throttle = (up_raw >> 6) - 31;
	rx_co_right = (rx_ch[3] >> 6) - 10;
	rx_cy_front = ((rx_ch[0] - rx_ch[1] * 2 - rx_ch[5]) >> 8) - 0x3a;
	rx_cy_right = (0x800 - rx_ch[5] - rx_ch[0]) >> 7;

	rx_gyro_sw = rx_ch[4] < 0x2180;
	rx_right_pot = (uint16_t) ((rx_ch[4] - 0x0200) & 0x1fff) >> 5;
	if (rx_ch[4] >= 0x4200)
		rx_right_pot = 0xff;
	else if (rx_ch[4] <= 0x0200)
		rx_right_pot = 0x00;
	/* TODO */
	rx_left_pot = (rx_ch[0] + rx_ch[1] - rx_ch[5] - (rx_ch[2] >> 1)) >> 8;
# endif
#endif

	rx_no_signal = 0;
}
#endif

#define RX_MIN 0.00105	/* 1ms + a bit */
#define RX_MAX 0.00195	/* 2ms - a bit */
#define CLK_MIN ((int) (F_CPU * RX_MIN))
#define CLK_MAX ((int) (F_CPU * RX_MAX))
#define CLK_PART(x) ((int) (F_CPU * (RX_MIN + (x) * (RX_MAX - RX_MIN))))
#define CLK_DIV ((1 << 24) / (CLK_MAX - CLK_MIN))
#define CLK_MAX2 (CLK_MIN + ((255 << 16) / CLK_DIV))

static void rx_tgy9x_update(void) {
	uint16_t ch5;

#define SCALE(out, in)	\
	if (in <= CLK_MIN)	\
		out = 0;	\
	else if (in >= CLK_MAX2)	\
		out = 255;	\
	else	\
		out = ((in - CLK_MIN) * CLK_DIV) >> 16;

	SCALE(rx_cy_front, rx_ch[0])
	SCALE(rx_co_throttle, rx_ch[1])
	SCALE(rx_cy_right, rx_ch[2])
	SCALE(rx_co_right, rx_ch[3])

	/* Tgy9x config:
	 * CH1 Ele  100
	 * CH2 Ail  100
	 * CH3 Thr  100
	 * CH4 Rud  100
	 * CH5 GEA   60   Offset 0
	 *  += 3POS  40   Offset -20
	 */
	if (rx_ch[4] > CLK_PART(0.445)) {
		rx_gear_sw = 1;
		ch5 = rx_ch[4] - CLK_PART(0.675) + CLK_MIN;
	} else {
		rx_gear_sw = 0;
		ch5 = rx_ch[4];
	}
	rx_id_sw = ch5 > CLK_PART(0.01) ? ch5 > CLK_PART(0.217) ?
		2 : 1 : 0;

	rx_no_signal = 0;
}

#define rx_update rx_tgy9x_update

static void rx_flip_handle(void) {
	static uint32_t rx_up = 0;
	static int rx_chnum = 9;
	uint32_t diff, now = timer_read();

	/* The receivers normally start pulsing the next channel almost
	 * immediately before the falling edge of the previous channel
	 * pulse, so the edges come about 0.01 ms apart only and
	 * sometimes the second interrupts gets missed.  Since they are
	 * so close we choose to unconditionally ignore the second one,
	 * because the first one stands a better chance of being reported
	 * accurately by the AVR core.  We ignore any edges < 0.5ms since
	 * the previous edge.
	 */

	diff = now - rx_up;
	if (diff > F_CPU / 200)
		rx_chnum = 0;
	else if (rx_chnum > 9 || diff < F_CPU / 2000)
		return;
	else {
		rx_ch[rx_chnum ++] = diff;

		if (rx_chnum == CHANNELS)
			rx_update();
	}

	rx_up = now;
}

void pioint1_isr(void) {
	LPC_GPIO1->IC = 0x3ff;
	rx_flip_handle();
}
void pioint2_isr(void) {
	LPC_GPIO2->IC = 0x3ff;
	rx_flip_handle();
}
