/*
 * Generates signal that drives RC actuators (motors and servos).
 */

#include <avr/io.h>
#include <pins_arduino.h>

#include "timer1.h"

static void pin_mode(uint8_t pin, uint8_t mode) {
	uint8_t bit = digitalPinToBitMask(pin + 3);
	volatile uint8_t *reg = portModeRegister(digitalPinToPort(pin + 3));

	if (mode)
		*reg |= bit;
	else
		*reg &= ~bit;
}

static void pin_set(uint8_t pin, uint8_t val) {
	uint8_t bit = digitalPinToBitMask(pin + 3);
	volatile uint8_t *reg = portOutputRegister(digitalPinToPort(pin + 3));

	if (val)
		*reg |= bit;
	else
		*reg &= ~bit;
}

volatile uint16_t actuators[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
static uint8_t devs = 0;

static void set_0_low(void) {
	pin_set(0, 0);
}
static void set_1_low(void) {
	pin_set(1, 0);
}
static void set_2_low(void) {
	pin_set(2, 0);
}
static void set_3_low(void) {
	pin_set(3, 0);
}
static void set_4_low(void) {
	pin_set(4, 0);
}
static void set_5_low(void) {
	pin_set(5, 0);
}
static void set_6_low(void) {
	pin_set(6, 0);
}
static void set_7_low(void) {
	pin_set(7, 0);
}

static void set_all_high(void) {
	uint8_t i;

	for (i = 0; i < devs; i ++)
		pin_set(i, 1);
}

static uint32_t base;
static const uint32_t mili = F_CPU / 1000;

static void actuators_update(void) {
	/*
	 * Use the standard timing..  the period is some 20ms so we
	 * wait 5ms, then set all of outputs high and then leave it high
	 * for between 1 and 2ms depending on the current actuator state.
	 */
	set_timeout(base + mili * 5, set_all_high);
	if (devs > 0)
		set_timeout(base + mili * 6 +
				((actuators[0] * mili) >> 16), set_0_low);
	if (devs > 1)
		set_timeout(base + mili * 6 +
				((actuators[1] * mili) >> 16), set_1_low);
	if (devs > 2)
		set_timeout(base + mili * 6 +
				((actuators[2] * mili) >> 16), set_2_low);
	if (devs > 3)
		set_timeout(base + mili * 6 +
				((actuators[3] * mili) >> 16), set_3_low);
	if (devs > 4)
		set_timeout(base + mili * 6 +
				((actuators[4] * mili) >> 16), set_4_low);
	if (devs > 5)
		set_timeout(base + mili * 6 +
				((actuators[5] * mili) >> 16), set_5_low);
	if (devs > 6)
		set_timeout(base + mili * 6 +
				((actuators[6] * mili) >> 16), set_6_low);
	if (devs > 7)
		set_timeout(base + mili * 6 +
				((actuators[7] * mili) >> 16), set_7_low);

	base += mili * 20;
	set_timeout(base, actuators_update);
}

void actuators_init(int devices) {
	uint8_t i;

	devs = devices;

	/* Set the relevant GPIOs low */
	for (i = 0; i < devices; i ++)
		pin_set(i, 0);

	/* Set the relevant GPIOs as outputs */
	for (i = 0; i < devices; i ++)
		pin_mode(i, 1);
}

void actuators_start(void) {
	/*
	 * Start sampling the standard PWM signal right away,
	 * the ESCs may complain if they don't detect the familiar signal
	 * right after power-up.
	 */
	base = timer_read();
	actuators_update();
}
