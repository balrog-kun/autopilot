/*
 * A simple Arduino Duemilanove-based quadcopter autopilot.
 */

#include <avr/io.h>
#include <avr/interrupt.h>

#include "adc.h"
#include "timer1.h"
#include "uart.h"
#include "actuators.h"
#include "rx.h"
#include "twi.h"
#include "cmps09.h"
#include "ahrs.h"
#include "isqrt.h"

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

void nop(void) {}
void die(void) {
	cli();
	serial_write_str("ERROR");
	while (1);
}

void setup(void) {
	uint8_t s = SREG;
	uint8_t m = MCUCR;
	uint8_t ver, cnt, regs[6];
	int16_t v;
	uint32_t len;

	/* Initialise everything we need */
	serial_init();
	adc_init();
	timer_init();
	actuators_init(4);
	serial_set_handler(handle_input);
	rx_init();
	twi_init();
	sei();

	adc_convert_all(nop);

	rx_no_signal = 10;

	/* Wait for someone to attach to UART */
	my_delay(4000);

	serial_write_str("SREG:");
	serial_write_hex16(s);
	serial_write_str(", MCUCR:");
	serial_write_hex16(m);
	serial_write_eol();

	/* Perform all the status sanity checks */

	serial_write_str("Battery voltage:");
	/* Reference volatage is 3.3V & resistors divide input voltage by ~5 */
	serial_write_fp32((uint32_t) adc_values[3] * 323 * (991 + 241),
			0x400L * 100 * 241);
	serial_write1('V');
	serial_write_eol();
	/* TODO: check that li-poly voltage is not below 3.2V per cell
	 * (unless Li-Po-Fe) */

	serial_write_str("CPU temperature:");
	/* Reference volatage is 1.1V now */
	serial_write_fp32((adc_values[4] - 269) * 1100, 0x400);
	serial_write_eol();

	ver = 0xff;
	cmps09_read_bytes(0, 1, &ver);
	serial_write_str("Magnetometer revision:");
	serial_write_hex16(ver);
	serial_write_eol();
	if (ver != 0x02)
		die();

	serial_write_str("Checking if gyro readings in range.. ");
	/* 1.23V expected -> 2 * 0x400 * 1.23V / 3.3V == 0x2fb */
	cnt = 0;
	while (adc_values[0] > 0x2a0 && adc_values[0] < 0x350 &&
			adc_values[1] > 0x2a0 && adc_values[1] < 0x350 &&
			adc_values[2] > 0x2a0 && adc_values[2] < 0x350 &&
			cnt ++ < 20) {
		adc_values[0] = 2 * adc_convert(0);
		adc_values[1] = 2 * adc_convert(1);
		adc_values[2] = 2 * adc_convert(2);
	}
	if (cnt < 21)
		die();
	serial_write_str("yep");
	serial_write_eol();

	serial_write_str("Checking magnetic field magnitude.. ");
	cmps09_read_bytes(10, 6, regs);
	v = (((uint16_t) regs[0] << 8) | regs[1]) - cmps09_mag_calib[0];
	len = (int32_t) v * v;
	v = (((uint16_t) regs[2] << 8) | regs[3]) - cmps09_mag_calib[1];
	len += (int32_t) v * v;
	v = (((uint16_t) regs[4] << 8) | regs[5]) - cmps09_mag_calib[2];
	len += (int32_t) v * v;
	len = isqrt32(len);
	serial_write_fp32(len, 1000);
	serial_write_str(" T");
	serial_write_eol();
	if (len > 600 || len < 300)
		die();

	serial_write_str("Checking accelerometer readings.. ");
	v = 0;
	for (cnt = 0; cnt < 16; cnt ++) {
		cmps09_read_bytes(16, 6, regs);
		v = ((int16_t) (((uint16_t) regs[0] << 8) | regs[1]) + 1) >> 1;
		len += (int32_t) v * v;
		v = ((int16_t) (((uint16_t) regs[2] << 8) | regs[3]) + 1) >> 1;
		len += (int32_t) v * v;
		v = ((int16_t) (((uint16_t) regs[4] << 8) | regs[5]) + 1) >> 1;
		len += (int32_t) v * v;
		my_delay(20);
	}
	len = (isqrt32(len) + 1) >> 1;
	/* TODO: the scale seems to change a lot with temperature? */
	serial_write_fp32(len, 0x4050);
	serial_write_str(" g");
	serial_write_eol();
	if (len > 0x4070 || len < 0x3fe0)
		die();

	serial_write_str("Receiver signal: ");
	serial_write_str(rx_no_signal ? "NOPE" : "yep");
	serial_write_eol();
	if (!rx_no_signal && rx_co_throttle > 5) {
		serial_write_str("Throttle stick is not in the bottom "
				"position\r\n");
		die();
	}

	serial_write_str("Calibrating sensors..\r\n");

	/* Start the software clever bits */
	ahrs_init();
	actuators_start();

	serial_write_str("AHRS loop and actuator signals are running\r\n");

	show_state();
}

void loop(void) {
	my_delay(200);
	serial_write_fp32(ahrs_pitch, ROLL_PITCH_180DEG / 180);
	serial_write_fp32(ahrs_roll, ROLL_PITCH_180DEG / 180);
	serial_write_fp32((int32_t) ahrs_yaw * 180, 32768);
	serial_write_eol();
}

int main(void) {
	setup();

	for (;;)
		loop();

	return 0;
}
