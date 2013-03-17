/*
 * Generates signal that drives RC actuators (motors and servos).
 *
 * Licensed under AGPLv3.
 */

#include "LPC13xx.h"		/* LPC13xx Peripheral Registers */
#include "timer1.h"

volatile uint16_t actuators[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
static uint8_t devs = 4;

static void set_0_low(void) {
	LPC_GPIO0->DATA &= ~0b0010;
}
static void set_1_low(void) {
	LPC_GPIO0->DATA &= ~0b0001;
}
static void set_2_low(void) {
	LPC_GPIO0->DATA &= ~0b0100;
}
static void set_3_low(void) {
	LPC_GPIO0->DATA &= ~0b1000;
}
static void set_4_low(void) {
}
static void set_5_low(void) {
}
static void set_6_low(void) {
}
static void set_7_low(void) {
}

static void set_all_high(void) {
	LPC_GPIO0->DATA |= 0b1111;
}

static uint32_t base;
static const uint32_t mili = F_CPU / 1000;
static int stop;

static void actuators_update(void) {
	/*
	 * Use the standard timing..  the period is some 20ms so we
	 * wait 5ms, then set all of outputs high and then leave it high
	 * for between 1 and 2ms depending on the current actuator state.
	 */
	static const uint32_t milistar = mili * 6;/// - mili / 6;

	if (stop)
		return;

	set_timeout(base + mili * 5, set_all_high);
	if (devs > 0)
		set_timeout(base + milistar +
				((actuators[0] * mili) >> 16), set_0_low);
	if (devs > 1)
		set_timeout(base + milistar +
				((actuators[1] * mili) >> 16), set_1_low);
	if (devs > 2)
		set_timeout(base + milistar +
				((actuators[2] * mili) >> 16), set_2_low);
	if (devs > 3)
		set_timeout(base + milistar - mili / 2 +
				((actuators[3] * mili) >> 15), set_3_low);
	if (devs > 4)
		set_timeout(base + milistar +
				((actuators[4] * mili) >> 15), set_4_low);
	if (devs > 5)
		set_timeout(base + milistar +
				((actuators[5] * mili) >> 15), set_5_low);
	if (devs > 6)
		set_timeout(base + milistar +
				((actuators[6] * mili) >> 15), set_6_low);
	if (devs > 7)
		set_timeout(base + milistar +
				((actuators[7] * mili) >> 15), set_7_low);

	base += mili * 20;
	set_timeout(base, actuators_update);
}

void actuators_init(int devices) {
	devs = devices;

	LPC_IOCON->PIO0_1 = 0x0c0;
	LPC_IOCON->RESET_PIO0_0 = 0x001;
	LPC_IOCON->PIO0_2 = 0x0c0;
	LPC_IOCON->PIO0_3 = 0x0c0;

	/* Set the relevant GPIOs as outputs */
	LPC_GPIO0->DIR |= 0b1111;
}

void actuators_start(void) {
	/*
	 * Start sampling the standard PWM signal right away,
	 * the ESCs may complain if they don't detect the familiar signal
	 * right after power-up.
	 */
	base = timer_read();
	stop = 0;
	actuators_update();
}

void actuators_stop(void) {
	stop = 1;
}
