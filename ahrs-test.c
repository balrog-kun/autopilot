#include <avr/io.h>
#include <avr/interrupt.h>

#include "timer1.h"
#include "uart.h"
#include "twi.h"
#include "wmp.h"
#include "ahrs.h"

static void setup(void) {
	/* Initialise everything we need */
	serial_init();
	timer_init();
	twi_init();
	sei();

	/* Wait for someone to attach to UART */
	my_delay(1000);

	serial_write_str("Ok\r\n");

	wmp_on();
	/* Let the gyro stabilise */
	my_delay(200);

	ahrs_init();
}

static const char to_hex[16] = {
	'0', '1', '2', '3', '4', '5', '6', '7',
	'8', '9', 'a', 'b', 'c', 'd', 'e', 'f',
};

static void loop(void) {
	uint8_t *qq;
	int i;

	my_delay(50);
#if 1
	for (i = 0; i < 4; i ++) {
		qq = (uint8_t *) (q + i);
		serial_write1(to_hex[(qq[0] >>  4) & 15]);
		serial_write1(to_hex[(qq[0] >>  0) & 15]);
		serial_write1(to_hex[(qq[1] >>  4) & 15]);
		serial_write1(to_hex[(qq[1] >>  0) & 15]);
		serial_write1(to_hex[(qq[2] >>  4) & 15]);
		serial_write1(to_hex[(qq[2] >>  0) & 15]);
		serial_write1(to_hex[(qq[3] >>  4) & 15]);
		serial_write1(to_hex[(qq[3] >>  0) & 15]);
		serial_write1(',');
	}
	for (i = 0; i < 3; i ++) {
		qq = (uint8_t *) (mag + i);
		serial_write1(to_hex[(qq[0] >>  4) & 15]);
		serial_write1(to_hex[(qq[0] >>  0) & 15]);
		serial_write1(to_hex[(qq[1] >>  4) & 15]);
		serial_write1(to_hex[(qq[1] >>  0) & 15]);
		serial_write1(to_hex[(qq[2] >>  4) & 15]);
		serial_write1(to_hex[(qq[2] >>  0) & 15]);
		serial_write1(to_hex[(qq[3] >>  4) & 15]);
		serial_write1(to_hex[(qq[3] >>  0) & 15]);
		serial_write1(',');
	}
	for (i = 0; i < 3; i ++) {
		qq = (uint8_t *) (acc + i);
		serial_write1(to_hex[(qq[0] >>  4) & 15]);
		serial_write1(to_hex[(qq[0] >>  0) & 15]);
		serial_write1(to_hex[(qq[1] >>  4) & 15]);
		serial_write1(to_hex[(qq[1] >>  0) & 15]);
		serial_write1(to_hex[(qq[2] >>  4) & 15]);
		serial_write1(to_hex[(qq[2] >>  0) & 15]);
		serial_write1(to_hex[(qq[3] >>  4) & 15]);
		serial_write1(to_hex[(qq[3] >>  0) & 15]);
		serial_write1(',');
	}
#else
	serial_write_fp32((int32_t) (mag[0] * 0x10000), 256);
	serial_write_fp32((int32_t) (mag[1] * 0x10000), 256);
	serial_write_fp32((int32_t) (mag[2] * 0x10000), 256);
	serial_write_fp32((int32_t) (sqrt(mag[0] * mag[0] +
					mag[1] * mag[1] +
					mag[2] * mag[2]) * 0x10000), 256);
	serial_write_str(" m");
#endif
	serial_write1('\r');
	serial_write1('\n');
}

int main(void) {
	setup();

	for (;;)
		loop();

	return 0;
}
