/*
 * A simple Arduino Duemilanove-based quadcopter autopilot.
 */

#include <avr/io.h>
#include <avr/interrupt.h>

#include "adc.h"
#include "timer1.h"
#include "uart.h"
#include "actuators.h"

static uint8_t motor[4] = { 0, 0, 0, 0 };

static void show_state(void) {
	serial_write_dec8(motor[0]);
	serial_write_dec8(motor[1]);
	serial_write_dec8(motor[2]);
	serial_write_dec8(motor[3]);
	serial_write_eol();
}

static void handle_input(char ch) {
	switch (ch) {
#define MOTOR_DOWN(n)				\
		if (motor[n] > 0)		\
			motor[n] --;		\
		actuator_set(n, motor[n] << 8);	\
		break;
#define MOTOR_UP(n)				\
		if (motor[n] < 255)		\
			motor[n] ++;		\
		actuator_set(n, motor[n] << 8);	\
		break;
	case 'a':
		MOTOR_DOWN(0);
	case 's':
		MOTOR_DOWN(1);
	case 'd':
		MOTOR_DOWN(2);
	case 'f':
		MOTOR_DOWN(3);
	case 'q':
		MOTOR_UP(0);
	case 'w':
		MOTOR_UP(1);
	case 'e':
		MOTOR_UP(2);
	case 'r':
		MOTOR_UP(3);
	default:
		return;
	}

	sei();
	show_state();
}

void setup(void) {
	uint8_t s = SREG;
	uint8_t m = MCUCR;

	serial_init();
	adc_init();
	timer_init();
	sei();
	actuators_init(4);
	serial_set_handler(handle_input);

	/* Wait for someone to attach to UART */
	my_delay(4000);

	serial_write_str("SREG:");
	serial_write_hex16(s);
	serial_write_str(", MCUCR:");
	serial_write_hex16(m);
	serial_write_eol();

	/* Wait for the ESCs to detect voltages etc. */
	my_delay(4000);
	actuators_start();

	show_state();
}

void loop(void) {
	my_delay(100);
	/*serial_write1('.');*/
}

int main(void) {
	setup();

	for (;;)
		loop();

	return 0;
}
