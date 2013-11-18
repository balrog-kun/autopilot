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

/* Rx PPM output connected to PIO1_9
 * Note: only unmask interrupts for the pins actually connected to Rx.  */
void rx_init(void) {
	/* Disable the pull-ups, they seem to cause the Rx to detect a jumper */
	LPC_IOCON->PIO1_9 = 0xc0;
	LPC_GPIO1->DIR &= ~((1 << 9));
	LPC_GPIO1->IS = ~((1 << 9));
	LPC_GPIO1->IBE = (1 << 9);
	LPC_GPIO1->IE = (1 << 9);
	NVIC_EnableIRQ(EINT1_IRQn);

//	LPC_IOCON->PIO1_11 = 0xc0;/////
//	LPC_GPIO1->DIR &= ~((1 << 11));////
}

#define RX_MIN 0.00073	/* 1ms + a bit */
#define RX_MAX 0.00177	/* 2ms - a bit */
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
	SCALE(rx_cy_right, rx_ch[1])
	SCALE(rx_co_throttle, rx_ch[2])
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
	static int rx_chnum = -2;
	uint32_t diff, now = timer_read();

	diff = now - rx_up;
	if ((LPC_GPIO1->DATA >> 9) & 1) {
		/* Rising edge */
		/* If not close to 250us, go to the invalid state */
		if (unlikely(diff < F_CPU / 4200 || diff > F_CPU / 3800))
			rx_chnum = -2;
		else if (rx_chnum >= -1)
			rx_chnum ++;
	} else {
		/* Falling edge */
		/* If notably less than 250us, go to the invalid state */
		if (unlikely(diff < F_CPU / 4200))
			rx_chnum = -2;
		/* If less than 2.8ms, save channel value */
		else if (diff < F_CPU / 350) {
			if (rx_chnum >= 0 && rx_chnum < 8)
				rx_ch[rx_chnum] = diff;
			/* Report new values ASAP for minimum latency */
			if (rx_chnum == CHANNELS - 1)
				rx_update();
		}
		/* If less than 7ms, go to the invalid state */
		else if (unlikely(diff < F_CPU / 150))
			rx_chnum = -2;
		/* Otherwise start a new cycle */
		else
			rx_chnum = -1;
	}

	rx_up = now;
}

void pioint1_isr(void) {
	LPC_GPIO1->IC = 0x3ff;
	rx_flip_handle();
}
