/*
 * A simple Arduino Duemilanove-based quadcopter autopilot.
 *
 * Licensed under AGPLv3.
 */

#include "LPC13xx.h"		/* LPC13xx Peripheral Registers */
#include "uart.h"
#include "timer1.h"

static volatile uint8_t cnt = 0;
static void handle_input(char ch) {
	cnt = 0;
	serial_write_str("capturing\r\ncurrent:");////
	serial_write_hex16(((LPC_GPIO1->DATA & 0x320) >> 5) | ((LPC_GPIO2->DATA << 1) & 2));////
}

void setup(void) {
	if (!(LPC_SYSCON->SYSAHBCLKCTRL & (1 << 16))) {
		LPC_SYSCON->SYSAHBCLKCTRL |= 1 << 16;
	}
	LPC_SYSCON->SYSAHBCLKCTRL |= 1 << 6;

	serial_init();
	timer_init();
	serial_set_handler(handle_input);
	__enable_irq();

	/* Wait for someone to attach to UART */
	my_delay(2000);

	LPC_GPIO1->DIR &= ~((1 << 5) | (1 << 8) | (1 << 9));
	LPC_GPIO2->DIR &= ~(1 << 0);
	LPC_IOCON->PIO1_5 = 0xd0;
	LPC_IOCON->PIO2_0 = 0xd0;
	LPC_IOCON->PIO1_8 = 0xd0;
	LPC_IOCON->PIO1_9 = 0xd0;
	LPC_GPIO1->IBE = (1 << 5) | (1 << 8) | (1 << 9);
	LPC_GPIO2->IBE = (1 << 0);
	LPC_GPIO1->IE = (1 << 5) | (1 << 8) | (1 << 9);
	LPC_GPIO2->IE = (1 << 0);
	/* we even leave the pull-ups enabled just in case */
	NVIC_EnableIRQ(EINT1_IRQn);
	NVIC_EnableIRQ(EINT2_IRQn);
}

static volatile uint32_t times[32];
static volatile uint16_t vals[32];
static volatile int printed = 1;
void loop(void) {
	int i;
	uint32_t prev = 0;
	serial_write1('u' + printed);
	my_delay(1000);
	serial_write1('0' + printed);
	if (printed)
		return;
	serial_write_str("\r\n");
	for (i = 0; i < 32; i ++) {
		serial_write1('t');
		serial_write_dec8(i);
		serial_write_hex32(times[i]);
		serial_write_hex32(times[i] - prev);
		prev = times[i];
		serial_write_hex16(vals[i]);
		serial_write_str("\r\n");
	}
	printed = 1;
}

int main(void) {
	setup();

	serial_write1('x');
	for (;;)
		loop();

	return 0;
}

void pisr(void) {
	uint32_t now = timer_read();

	if (cnt >= 32)
		return;
	times[cnt] = now;
	vals[cnt] = ((LPC_GPIO1->DATA & 0x320) >> 5) | ((LPC_GPIO2->DATA << 1) & 2);////
	cnt += 1;
	if (cnt == 32)
		printed = 0;
}
void pioint1_isr(void) {
	LPC_GPIO1->IC = 0x3ff;//LPC_GPIO1->IS & 0x320;
	pisr();
}

void pioint2_isr(void) {
	LPC_GPIO2->IC = 0x3ff;///LPC_GPIO2->IS & 1;
	pisr();
}
