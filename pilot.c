/*
 * A simple Arduino Duemilanove-based quadcopter autopilot.
 *
 * Licensed under AGPLv3.
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
#include "trig.h"
#include "isqrt.h"

static uint8_t motor[4] = { 0, 0, 0, 0 };
static uint8_t debug = 0x00;
static uint8_t prev_sw = 0;
enum debug_e {
	DEBUG_ATTITUDE,
	DEBUG_VELOCITY,
	DEBUG_ACCELERATION,
	DEBUG_MAGNETIC,
	DEBUG_RX,
	DEBUG_MOTORS,
	DEBUG_BAT_N_TEMP,
};

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
	case '1' ... '8':
		debug ^= 1 << (ch - '1');
		break;
	default:
		return;
	}

	sei();
	show_state();
}

static void nop(void) {}
static void die(void) {
	cli();
	serial_write_str("ERROR");
	while (1);
}

static void setup(void) {
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
	serial_write1('C');
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
	if (len > 0x4170 || len < 0x3f00)
		die();

	serial_write_str("Receiver signal: ");
	serial_write_str(rx_no_signal ? "NOPE" : "yep");
	serial_write_eol();
	if (rx_no_signal || rx_co_throttle > 5 || rx_gyro_sw) {
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
	prev_sw = rx_gyro_sw;
}

/* The modes are a set of boolean switches that can be set or reset/cleared
 * using the "gyro switch" on the transmitter.  Every move of the switch
 * changes the mode pointed at by the "CH5" potentiometer which is on the
 * right side of the transmitter.
 */
enum modes_e {
	/* Arm/disarm the motors (dangerous!) */
	MODE_MOTORS_ARMED,
	/* Enable compass-based heading-hold, pitch/roll hold is always on */
	MODE_HEADINGHOLD_ENABLE,
	/* Adaptively change motor output differences, don't use absolute
	 * values.  HEADINGHOLD is forced on when this is on.  */
	MODE_ADAPTIVE_ENABLE,
	/* Try to keep enhancing the neutral attitude based on acceleration */
	MODE_AUTONEUTRAL_ENABLE,
	/* TODO: Cyclic stick controls the camera pan&tilt instead of the
	 * vehicle's attitude */
	MODE_PANTILT_ENABLE,
	/* TODO: Emergency land or return home, panic, scream! */
	MODE_EMERGENCY,
};
static uint8_t modes =
	(0 << MODE_MOTORS_ARMED) |
	(0 << MODE_HEADINGHOLD_ENABLE) |
	(0 << MODE_ADAPTIVE_ENABLE) |
	(0 << MODE_AUTONEUTRAL_ENABLE) |
	(0 << MODE_PANTILT_ENABLE) |
	(0 << MODE_EMERGENCY);
#define SET_ONLY 0

static void modes_update(void) {
	uint8_t num;

	if (likely(rx_gyro_sw == prev_sw))
		return;
	prev_sw = rx_gyro_sw;

	num = ((uint16_t) rx_right_pot + 36) / 49;
	modes &= ~(1 << num) | SET_ONLY;
	modes |= prev_sw << num;
}

static int constants_cnt = 0;

static void control_update(void) {
	int16_t cur_pitch, cur_roll, cur_yaw, raw_pitch, raw_roll;
	int16_t dest_pitch, dest_roll, dest_yaw, base_throttle;

	static uint8_t yaw_deadband_pos = 0x80;
	static uint8_t roll_deadband_pos = 0x80;
	static uint8_t pitch_deadband_pos = 0x80;

	static int16_t neutral_pitch = 0;
	static int16_t neutral_roll = 0;
	static int16_t neutral_yaw = 0;

	uint8_t co_right = rx_co_right, cy_right = rx_cy_right,
		cy_front = rx_cy_front, co_throttle = rx_co_throttle;

	/* Motors (top view):
	 * (A)_   .    _(B)
	 *    '#_ .  _#'
	 *      '#__#'
	 * - - - _##_ - - - - pitch axis
	 *     _#'. '#_
	 *   _#'  .   '#_
	 * (C)    .     (D)
	 *        |
	 *        '--- roll axis
	 */
	int32_t a, b, c, d;

	rx_no_signal = (rx_no_signal < 255) ? rx_no_signal + 1 : 255;

	/* Yaw stick deadband in heading-hold mode */
	if ((modes & (1 << MODE_HEADINGHOLD_ENABLE)) ||
			(modes & (1 << MODE_ADAPTIVE_ENABLE))) {
		co_right += 0x80 - yaw_deadband_pos;
		if (co_right >= 0x80 - 3 && co_right <= 0x80 + 3)
			co_right = 0x80;
		else if (co_right > 0x80)
			co_right -= 3;
		else
			co_right += 3;
	} else
		yaw_deadband_pos = co_right;

	/* Roll stick deadband in velocity-hold mode */
	if (modes & (1 << MODE_AUTONEUTRAL_ENABLE)) {
		cy_right += 0x80 - pitch_deadband_pos;
		if (cy_right >= 0x80 - 3 && cy_right <= 0x80 + 3)
			cy_right = 0x80;
		else if (cy_right > 0x80)
			cy_right -= 3;
		else
			cy_right += 3;
	} else
		roll_deadband_pos = cy_right;

	/* Pitch stick deadband in velocity-hold mode */
	if (modes & (1 << MODE_AUTONEUTRAL_ENABLE)) {
		cy_front += 0x80 - pitch_deadband_pos;
		if (cy_front >= 0x80 - 3 && cy_front <= 0x80 + 3)
			cy_front = 0x80;
		else if (cy_front > 0x80)
			cy_front -= 3;
		else
			cy_front += 3;
	} else
		pitch_deadband_pos = cy_front;

	cli();
	raw_pitch = (ahrs_pitch + 32768) >> 16;
	raw_roll = (ahrs_roll + 32768) >> 16;
	cur_pitch = raw_pitch + ((ahrs_pitch_rate + 2) >> 2);
	cur_roll = raw_roll + ((ahrs_roll_rate + 2) >> 2);
	cur_yaw = ahrs_yaw + (ahrs_yaw_rate << 5);
	sei();

	if (modes & (1 << MODE_AUTONEUTRAL_ENABLE)) {
		/* TODO */
		if (raw_pitch > neutral_pitch && accel_acceleration[0] < 0)
			neutral_pitch -= accel_acceleration[0] *
				(raw_pitch - neutral_pitch);
		if (raw_pitch < neutral_pitch && accel_acceleration[0] > 0)
			neutral_pitch += accel_acceleration[0] *
				(raw_pitch - neutral_pitch);

		if (raw_roll > neutral_roll && accel_acceleration[1] < 0)
			neutral_roll -= accel_acceleration[1] *
				(raw_roll - neutral_roll);
		if (raw_roll < neutral_roll && accel_acceleration[1] > 0)
			neutral_roll += accel_acceleration[1] *
				(raw_roll - neutral_roll);
	}

	if (modes & (1 << MODE_PANTILT_ENABLE)) {
		co_right = 0x80;
		cy_right = 0x80;
		cy_front = 0x80;
	}

	dest_pitch = neutral_pitch + ((int16_t) cy_front << 5) - (128 << 5);
	dest_roll = neutral_roll + ((int16_t) cy_right << 5) - (128 << 5);
	neutral_yaw += ((int16_t) co_right << 2) - (128 << 2);
	dest_yaw = neutral_yaw;

	/* TODO: divide by cos(angle from neutral), which we can probably
	 * approximate with (dest-cur_pitch / 90 + dest-cur_roll / 90) for now
	 */
	base_throttle = co_throttle << 7;

	dest_pitch = -(cur_pitch + dest_pitch) / 1;
	dest_roll = -(cur_roll + dest_roll) / 1;
	dest_yaw = -(cur_yaw - dest_yaw) / 1;

	dest_yaw <<= 2;

#if 0
	/* Some easing */
	if (dest_pitch < 0x400 && dest_pitch > -0x400)
		dest_pitch >>= 2;
	else if (dest_pitch > 0)
		dest_pitch -= 0x300;
	else
		dest_pitch += 0x300;
	if (dest_roll < 0x400 && dest_roll > -0x400)
		dest_roll >>= 2;
	else if (dest_roll > 0)
		dest_roll -= 0x300;
	else
		dest_roll += 0x300;
#endif

#define CLAMP(x, mi, ma)	\
	if (x < mi)		\
		x = mi;		\
	if (x > ma)		\
		x = ma;

	if (modes & (1 << MODE_HEADINGHOLD_ENABLE)) {
		CLAMP(dest_yaw, -0xc00, 0xc00);
	} else {
		dest_yaw = (128 << 5) - ((int16_t) co_right << 5);
		if (modes & (1 << MODE_ADAPTIVE_ENABLE)) {
			dest_yaw -= ahrs_yaw_rate << 4;
			CLAMP(dest_yaw, -0xc00, 0xc00);
		}
		neutral_yaw = cur_yaw;
	}

	if (modes & (1 << MODE_ADAPTIVE_ENABLE)) {
#if 0
		static int16_t prev_diff_pitch = 0;
		static int16_t prev_diff_roll = 0;
		int16_t diff[4];

		prev_diff_pitch += dest_pitch >> 4;
		prev_diff_roll += dest_roll >> 4;

		dest_pitch = prev_diff_pitch;
		dest_roll = prev_diff_roll;
#else
		/* Our motors are currently modelled as OUTPUT = a * INPUT,
		 * TODO: use the OUTPUT = b + a * INPUT or the
		 * OUTPUT = gain_lut[INPUT] model.
		 */
		static int16_t throttle_base[4] = { 0, -0x1000, 0, 0 };
		static int16_t throttle_diff[4] = { 0, -0x1000, 0, 0 };

#define Q_LEN	16
		static uint8_t q_idx = 0;
		static int32_t motor_input[4][Q_LEN];
		static int16_t pitch_rates[Q_LEN],
			       roll_rates[Q_LEN], yaw_rates[Q_LEN];

		int16_t diff[4], sum;
		uint8_t q_next;
		int32_t pitch_gain, roll_gain, yaw_gain;
		int32_t motor_gain[4][2];
#endif

		diff[0] = 0 + dest_pitch + dest_roll + dest_yaw;
		diff[1] = 0 - dest_pitch + dest_roll - dest_yaw;
		diff[2] = 0 + dest_pitch - dest_roll - dest_yaw;
		diff[3] = 0 - dest_pitch - dest_roll + dest_yaw;

		a = ((((int32_t) base_throttle * ((int32_t) throttle_base[0] +
							0x4000)) >> 1) +
				(int32_t) diff[0] *
				(throttle_diff[0] + 0x2000)) >> 13;
		b = ((((int32_t) base_throttle * ((int32_t) throttle_base[1] +
							0x4000)) >> 1) +
				(int32_t) diff[1] *
				(throttle_diff[1] + 0x2000)) >> 13;
		c = ((((int32_t) base_throttle * ((int32_t) throttle_base[2] +
							0x4000)) >> 1) +
				(int32_t) diff[2] *
				(throttle_diff[2] + 0x2000)) >> 13;
		d = ((((int32_t) base_throttle * ((int32_t) throttle_base[3] +
							0x4000)) >> 1) +
				(int32_t) diff[3] *
				(throttle_diff[3] + 0x2000)) >> 13;

		q_next = (q_idx + 1) & 15;
		/* XXX: would it be better to use ahrs_pitch difference
		 * instead of ahrs_pitch_rate? */
		pitch_rates[q_next] = ahrs_pitch_rate;
		roll_rates[q_next] = ahrs_roll_rate;
		yaw_rates[q_next] = ahrs_yaw_rate;
		motor_input[0][q_next] = motor_input[0][q_idx] + diff[0];
		motor_input[1][q_next] = motor_input[1][q_idx] + diff[1];
		motor_input[2][q_next] = motor_input[2][q_idx] + diff[2];
		motor_input[3][q_next] = motor_input[3][q_idx] + diff[3];
		q_idx = q_next;

		/* Calculate the control loop gain over about 150ms */
#define DIFF_START (Q_LEN - 12)
#define DIFF_END   (Q_LEN - 0)
#define DIFF_OFF   3
		pitch_gain = pitch_rates[(q_idx + DIFF_END) & 15] -
			pitch_rates[(q_idx + DIFF_START) & 15];
		roll_gain = roll_rates[(q_idx + DIFF_END) & 15] -
			roll_rates[(q_idx + DIFF_START) & 15];
		yaw_gain = yaw_rates[(q_idx + DIFF_END) & 15] -
			yaw_rates[(q_idx + DIFF_START) & 15];
		/* TODO: detect or calculate the right value based on
		 * the propeller parameters.  The problem with hardcoding
		 * any value is that it won't account for any tilt of
		 * the motor shaft due to construction characteristics or
		 * physical wear */
		yaw_gain <<= 4;

		motor_gain[0][0] =
			motor_input[0][(q_idx + DIFF_END - DIFF_OFF) & 15] -
			motor_input[0][(q_idx + DIFF_START - DIFF_OFF) & 15];
		motor_gain[1][0] =
			motor_input[1][(q_idx + DIFF_END - DIFF_OFF) & 15] -
			motor_input[1][(q_idx + DIFF_START - DIFF_OFF) & 15];
		motor_gain[2][0] =
			motor_input[2][(q_idx + DIFF_END - DIFF_OFF) & 15] -
			motor_input[2][(q_idx + DIFF_START - DIFF_OFF) & 15];
		motor_gain[3][0] =
			motor_input[3][(q_idx + DIFF_END - DIFF_OFF) & 15] -
			motor_input[3][(q_idx + DIFF_START - DIFF_OFF) & 15];
		motor_gain[0][1] = 0 + pitch_gain + roll_gain + yaw_gain;
		motor_gain[1][1] = 0 - pitch_gain + roll_gain - yaw_gain;
		motor_gain[2][1] = 0 + pitch_gain - roll_gain - yaw_gain;
		motor_gain[3][1] = 0 - pitch_gain - roll_gain + yaw_gain;
		motor_gain[0][1] -= (motor_gain[0][0] + 16) >> 5;
		motor_gain[1][1] -= (motor_gain[1][0] + 16) >> 5;
		motor_gain[2][1] -= (motor_gain[2][0] + 16) >> 5;
		motor_gain[3][1] -= (motor_gain[3][0] + 16) >> 5;

		/* Update the factors */
#define MGAIN 0x800

		sum = (throttle_diff[0] + throttle_diff[1] +
				throttle_diff[2] + throttle_diff[3] + 2) >> 2;
		if (motor_gain[0][0] > MGAIN || motor_gain[0][0] < -MGAIN) {
			if (motor_gain[0][0] > 0)
				motor_gain[0][1] = -motor_gain[0][1];
			throttle_diff[0] += ((motor_gain[0][1] + 512) >> 10) -
				sum;
			CLAMP(throttle_diff[0], -0x1800, 0x4000);
		}
		if (motor_gain[1][0] > MGAIN || motor_gain[1][0] < -MGAIN) {
			if (motor_gain[1][0] > 0)
				motor_gain[1][1] = -motor_gain[1][1];
			throttle_diff[1] += ((motor_gain[1][1] + 512) >> 10) -
				sum;
			CLAMP(throttle_diff[1], -0x1800, 0x4000);
		}
		if (motor_gain[2][0] > MGAIN || motor_gain[2][0] < -MGAIN) {
			if (motor_gain[2][0] > 0)
				motor_gain[2][1] = -motor_gain[2][1];
			throttle_diff[2] += ((motor_gain[2][1] + 512) >> 10) -
				sum;
			CLAMP(throttle_diff[2], -0x1800, 0x4000);
		}
		if (motor_gain[3][0] > MGAIN || motor_gain[3][0] < -MGAIN) {
			if (motor_gain[3][0] > 0)
				motor_gain[3][1] = -motor_gain[3][1];
			throttle_diff[3] += ((motor_gain[3][1] + 512) >> 10) -
				sum;
			CLAMP(throttle_diff[3], -0x1800, 0x4000);
		}

		sum = (throttle_base[0] + throttle_base[1] +
				throttle_base[2] + throttle_base[3] + 2) >> 2;
		throttle_base[0] += ((diff[0] + 32) >> 6) - sum;
		throttle_base[1] += ((diff[1] + 32) >> 6) - sum;
		throttle_base[2] += ((diff[2] + 32) >> 6) - sum;
		throttle_base[3] += ((diff[3] + 32) >> 6) - sum;
		CLAMP(throttle_base[0], -0x1800, 0x4000);
		CLAMP(throttle_base[1], -0x1800, 0x4000);
		CLAMP(throttle_base[2], -0x1800, 0x4000);
		CLAMP(throttle_base[3], -0x1800, 0x4000);
	} else {
		a = (int32_t) base_throttle + dest_pitch + dest_roll + dest_yaw;
		b = (int32_t) base_throttle - dest_pitch + dest_roll - dest_yaw;
		c = (int32_t) base_throttle + dest_pitch - dest_roll - dest_yaw;
		d = (int32_t) base_throttle - dest_pitch - dest_roll + dest_yaw;
		/* HACK */
		b = ((b * 5) >> 3) - 6;
	}
	CLAMP(a, 0, 40000);
	CLAMP(b, 0, 25000);/* HACK */
	CLAMP(c, 0, 40000);
	CLAMP(d, 0, 40000);
	if (unlikely(!(modes & (1 << MODE_MOTORS_ARMED))))
		a = b = c = d = 0;
	actuator_set(0, (uint16_t) a);
	actuator_set(1, (uint16_t) b);
	actuator_set(2, (uint16_t) c);
	actuator_set(3, (uint16_t) d);

	if (constants_cnt ++ >= 25) /* About half a sec */
		constants_cnt = 0;
}

static void send_debug_info(void) {
	if (debug & (1 << DEBUG_ATTITUDE)) {
		serial_write_str("ATT");
		serial_write_fp32(ahrs_pitch, ROLL_PITCH_180DEG / 180);
		serial_write_fp32(ahrs_roll, ROLL_PITCH_180DEG / 180);
		serial_write_fp32((int32_t) ahrs_yaw * 180, 32768);
		serial_write_eol();
	}
	if (debug & (1 << DEBUG_VELOCITY)) {
		serial_write_str("V");
		serial_write_hex32(accel_velocity[0]);
		serial_write_hex32(accel_velocity[1]);
		serial_write_hex32(accel_velocity[2]);
		serial_write_eol();
	}
	if (debug & (1 << DEBUG_ACCELERATION)) {
		serial_write_str("ACC");
		serial_write_hex16(accel_acceleration[0]);
		serial_write_hex16(accel_acceleration[1]);
		serial_write_hex16(accel_acceleration[2]);
		serial_write_eol();
	}
	if (debug & (1 << DEBUG_MAGNETIC)) {
		serial_write_str("MAG");
		serial_write_eol();
	}
	if (debug & (1 << DEBUG_RX)) {
		serial_write_str("RX");
		serial_write_dec8(rx_no_signal - 1);
		serial_write_hex16(rx_co_throttle);
		serial_write_hex16(rx_co_right);
		serial_write_hex16(rx_cy_front);
		serial_write_hex16(rx_cy_right);
		serial_write_dec8(rx_gyro_sw);
		serial_write_eol();
	}
	if (debug & (1 << DEBUG_MOTORS)) {
		serial_write_str("MOT");
		serial_write_hex16(actuators[0]);
		serial_write_hex16(actuators[1]);
		serial_write_hex16(actuators[2]);
		serial_write_hex16(actuators[3]);
		serial_write_eol();
	}
	if (debug & (1 << DEBUG_BAT_N_TEMP)) {
		serial_write_str("BAT");
		serial_write_fp32((uint32_t) adc_values[3] * 323 * (991 + 241),
				0x400L * 100 * 241);
		serial_write1('V');
		serial_write_fp32((adc_values[4] - 269) * 1100, 0x400);
		serial_write1('C');
		serial_write_eol();
	}
}

static void loop(void) {
	my_delay(20); /* 50Hz update rate */

	modes_update();
	control_update();

	if (debug && (constants_cnt == 0 || constants_cnt == 12))
		send_debug_info();
}

int main(void) {
	setup();

	for (;;)
		loop();

	return 0;
}
