/*
 * A simple LPC1343-based multicopter flight controller.
 *
 * Licensed under AGPLv3.
 */

/* ritewing2
  x - back
  y - right
  z - up
*/

/* ritewing3
  x - right
  y - front
  z - up
*/

/* tri0
  x - left?
  y - back?
  z - up?
 */

/* tri1
  x - left
  y - back
  z - up
  pitch_rate - right
  roll_rate - back
  yaw_rate - left
 */

#define LEFT_MOTOR	0
#define RIGHT_MOTOR	1
#define RUDDER_MOTOR	2
#define RUDDER_SERVO	0

#define REVERSE_MASK	\
	((0 << (13 - RUDDER_MOTOR)) |	\
	 (0 << (13 - RIGHT_MOTOR)) |	\
	 (1 << (13 - LEFT_MOTOR)))

#include <stdint.h>
#include "LPC13xx.h"		/* LPC13xx Peripheral Registers */

#include "uart.h"
#include "timer1.h"
#include "rx.h"
#include "actuators-hwpwm.h"
#include "actuators-serial.h"
#include "twi.h"
#include "l3g4200d.h"
#include "adxl345.h"
#include "hmc5883l.h"
#include "bmp085.h"
#include "ahrs.h"
//#include "gps.h"
//#include "adc.h"

int abs(int);
void *memcpy(void *, void *, int);

static void die(void) {
	cli();
	serial_write_str("ERROR\r\n");
	while (1);
}

static uint8_t prev_sw = 0;
static uint8_t ahrs_report = 0;
static uint8_t gps_report = 0;
static uint8_t rx_report = 0;

static void handle_input(char ch) {
	if (ch == '1')
		ahrs_report ^= 1;
	if (ch == '2')
		gps_report ^= 1;
	if (ch == '3')
		rx_report ^= 1;
	if (ch == '4') {
		/* TODO: receive six bytes describing rx and set no_signal=0 */
	}
}

static uint8_t yaw_deadband_pos = 0x80;
static uint8_t roll_deadband_pos = 0x80;
static uint8_t pitch_deadband_pos = 0x80;

enum {
	CFG_MAGIC,
	CFG_NEUTRAL_X,
	CFG_NEUTRAL_Y,
	CFG_ROLLPITCH_P,
	CFG_ROLLPITCH_D,
	CFG_YAW_P,
	CFG_YAW_D,
	CFG_ADAPTIVE_0,
	CFG_ADAPTIVE_1,
	CFG_ADAPTIVE_2,
	CFG_ADAPTIVE_3,
	__CFG_END,
};
static uint32_t config[16] = {
	/* Increment on ver change */
	[CFG_MAGIC] = 0xabcd0001, /* Increment on ver change */

	/* Try to find the optimal x and y for static hover, they are not
	 * zero because the sensor board is not mounted perfectly level and
	 * because the motors shafts are not perfectly vertical.
	 *
	 * The values are the components of the sensor board's "up" vector
	 * pointing perependicularly to the sensor board's plane in the
	 * reference frame who's x and y axis are true level and z is
	 * true "up".
	 * Components are in 1 / 4096 units.
	 */
	[CFG_NEUTRAL_X] = -90, /* 90 to the right */
	[CFG_NEUTRAL_Y] = -180, /* 180 to the front */

#define neutral_x ((int32_t) config[CFG_NEUTRAL_X])
#define neutral_y ((int32_t) config[CFG_NEUTRAL_Y])

	[CFG_ROLLPITCH_P] = 3 * 32,
	[CFG_ROLLPITCH_D] = 5 * 32 / 5,

	[CFG_YAW_P] = 10 * 32,
	[CFG_YAW_D] = 0,
};

#define FLASH_END 0x8000

static void config_save(void) {
	uint8_t buf[256];
	int i;

	uint32_t ret[2];
	uint32_t prepare_cmd[3] = {
		50,
		(FLASH_END - 1) / 4096,
		(FLASH_END - 1) / 4096
	};
	uint32_t write_cmd[5] = {
		51,
		FLASH_END - 256,
		(uint32_t) &buf,
		256,
		F_CPU / 1000,
	};

	for (i = 0; i < (int) (sizeof(buf) - sizeof(config)); i ++)
		buf[i] = 0;
	memcpy(buf + sizeof(buf) - sizeof(config), config, sizeof(config));

#define CMD_SUCCESS 0
	/* Call the In-Application Programmer command handler */
	/* Note our linker script makes sure the last 32 RAM bytes are
	 * free for the IAP to use */
	cli();
	((void (*)(uint32_t *, uint32_t *)) 0x1fff1ff1)(prepare_cmd, ret);
	sei();

	if (ret[0] != CMD_SUCCESS) {
		serial_write_str("Flash prep err ");
		serial_write_hex32(ret[0]);
		serial_write_str("\r\n");
		return;
	}

	cli();
	((void (*)(uint32_t *, uint32_t *)) 0x1fff1ff1)(write_cmd, ret);
	sei();

	if (ret[0] != CMD_SUCCESS) {
		serial_write_str("Flash write err ");
		serial_write_hex32(ret[0]);
		serial_write_str("\r\n");
		return;
	}
}

static void config_update(int co_right, int cy_right, int cy_front) {
	static uint32_t last_ts = 1;
	uint32_t now = timer_read();
	int op;

	if (co_right > 0x80 + 40)
		op = 1;
	else if (co_right < 0x80 - 40)
		op = -1;
	else if (cy_right > 0x80 + 40)
		op = 2;
	else if (cy_right < 0x80 - 40)
		op = -2;
	else if (cy_front > 0x80 + 40)
		op = 3;
	else if (cy_front < 0x80 - 40)
		op = -3;
	else {
		if (last_ts == 0)
			last_ts = now;
		return;
	}

	/* Only one operation is performed per one sitck movement out of
	 * the neutral position, i.e. holding a stick in any position does
	 * not cause repeated operations.  We do this by checking that
	 * the stick has been in the neutral position for at least 100ms
	 * since the last time it was not.  */
	if (last_ts == 0 || now - last_ts < F_CPU / 10) {
		last_ts = 0;
		return;
	}
	last_ts = 0;

	if (rx_id_sw == 2) {
		if (op > 1)
			op += 2;
		else if (op < -1)
			op -= 2;
	}

	switch (op) {
	case 1:
		/* Collective stick right: TODO */
		return;
	case -1:
		/* Collective stick left: Save config to flash */
		config_save();
		break;
	case 2:
		/* Cyclic stick right: Increase D by 3 */
		config[CFG_ROLLPITCH_D] += 3;
		break;
	case -2:
		/* Cyclic stick left: Decrease D by 3 */
		config[CFG_ROLLPITCH_D] -= 3;
		break;
	case 3:
		/* Cyclic stick front: Increase P by 10 */
		config[CFG_ROLLPITCH_P] += 10;
		break;
	case -3:
		/* Cyclic stick back: Decrease P by 10 */
		config[CFG_ROLLPITCH_P] -= 10;
		break;
	/* Id switch at 2 */
	case 4:
		/* Cyclic stick right: Move neutral X right */
		config[CFG_NEUTRAL_X] -= 10;
		break;
	case -4:
		/* Cyclic stick left: Move neutral X left */
		config[CFG_NEUTRAL_X] += 10;
		break;
	case 5:
		/* Cyclic stick front: Move neutral Y front */
		config[CFG_NEUTRAL_Y] -= 10;
		break;
	case -5:
		/* Cyclic stick back: Move neutral Y back */
		config[CFG_NEUTRAL_Y] += 10;
		break;
	}

	/* Confirm the operation by wiggling the servo */
	actuator_hwpwm_set(RUDDER_SERVO, (uint16_t) 0x9000);
	my_delay(200);
	actuator_hwpwm_set(RUDDER_SERVO, (uint16_t) 0x8000);
}

static void setup(void) {
	int i;

	/* Enable IOCON clock if not enabled */
	if (!(LPC_SYSCON->SYSAHBCLKCTRL & (1 << 16))) {
		LPC_SYSCON->SYSAHBCLKCTRL |= 1 << 16;
	}

	/* Initialise everything we need */
	serial_init();
	timer_init();
	actuators_hwpwm_init(1);
	actuators_serial_init(3);
	twi_init();
	serial_set_handler(handle_input);
	rx_init();
	//gps_init();
	//adc_init();
	sei();

	rx_no_signal = 10;

	/* Wait for someone to attach to UART */
	my_delay(100);

	/* Don't fail right away, wait with any further initialisation
	 * until the Tx is on.  This also lets the user defer the AHRS
	 * initialisation until the platform is stable and there are no
	 * shakes e.g. from the power plug being adjusted.  */
	while (rx_no_signal > 5);

	if (rx_no_signal) {
		serial_write_str("RX ");
		die();
	}
	if (rx_co_throttle > 10) {
		serial_write_str("Throttle ");
		die();
	}
	if (rx_gear_sw) {
		serial_write_str("Gear switch ");
		die();
	}

	/* Start the software clever bits */
	l3g4200d_init();
	hmc5883l_init();
	adxl345_init();
	bmp085_init();
	my_delay(200);
	ahrs_init();

	actuator_serial_set(RUDDER_MOTOR, 0);
	actuator_serial_set(LEFT_MOTOR, 0);
	actuator_serial_set(RIGHT_MOTOR, 0);
	actuators_serial_start();
	actuator_hwpwm_set(RUDDER_SERVO, 0x8000);
	actuators_hwpwm_start();

	serial_write_str("Ok\r\n");

	if (((uint32_t *) (FLASH_END - sizeof(config)))[CFG_MAGIC] ==
			config[CFG_MAGIC]) {
		/* There's a valid config block at the end of the flash,
		 * try to load it into RAM variables.  */
		memcpy(config, (uint32_t *) (FLASH_END - sizeof(config)),
				sizeof(config));
		for (i = CFG_MAGIC + 1; i < __CFG_END; i ++)
			serial_write_hex32(config[i]);
		serial_write_str("\r\nOld cfg loaded\r\n");
	}

	yaw_deadband_pos = rx_co_right;
	roll_deadband_pos = rx_cy_right;
	pitch_deadband_pos = rx_cy_front;
	prev_sw = rx_gear_sw;
}

/* The modes are a set of boolean switches that can be set or reset/cleared
 * using the "gear switch" on the transmitter.  Every move of the switch
 * changes the mode pointed at by the potentiometer which is on the
 * right side of the transmitter.
 */
enum modes_e {
	/* Arm/disarm the motors (dangerous!) */
	MODE_MOTORS_ARMED,
	/* Hold the geographic heading/bearing */
	MODE_HEADINGHOLD_ENABLE,
	/* Perhaps start gliding maintaining constant speed and orientation,
	 * and then, sense the altitude using a proximity sensor and try
	 * to brake and land */
	MODE_SOMETHING,
	/* Or RTL? rename MODE_RTL? */
	MODE_EMERGENCY,
	/* Adaptively change motor output differences and servo outputs,
	 * don't use absolute values.  Only works when HOLD enabled.  */
	MODE_ADAPTIVE_ENABLE,
	/* Cut the PPM signal off for a while so that the engine makes the
	 * "welcome" noise once the signal is restarted.  This is in case
	 * we emergency land far away and need some audio hints to find
	 * the vehicle.  It's not very loud though.  */
	MODE_PPM_STOP,
	/* Attempt to minimise rotation rates automatically */
	MODE_HOLD_ENABLE,
	/* Attempt to also maintain level pitch */
	MODE_PITCHLEVEL_ENABLE,
	/* Attempt to maintain level roll */
	MODE_ROLLLEVEL_ENABLE,
};
static uint32_t modes =
	(0 << MODE_MOTORS_ARMED) |
	(0 << MODE_HEADINGHOLD_ENABLE) |
	(0 << MODE_SOMETHING) |
	(0 << MODE_EMERGENCY) |
	(1 << MODE_ADAPTIVE_ENABLE) |
	(0 << MODE_PPM_STOP) |
	(1 << MODE_HOLD_ENABLE) |
	(1 << MODE_PITCHLEVEL_ENABLE) |
	(1 << MODE_ROLLLEVEL_ENABLE);
#define SET_ONLY 0

static void modes_update(void) {
	int num;
	static int debounce_cnt = 0;

	if (likely(rx_gear_sw == prev_sw)) {
		debounce_cnt = 0;
		return;
	}
	if (debounce_cnt ++ < 8)
		return;
	prev_sw = rx_gear_sw;

	num = ((int) rx_right_pot + 36) / 49;
	if (num == MODE_MOTORS_ARMED && prev_sw && rx_co_throttle > 30)
		return;
	modes &= ~(1 << num) | SET_ONLY;
	modes |= prev_sw << num;
}

static uint16_t output[4];
static void control_update(void) {
	int32_t cur_pitch, cur_roll, cur_yaw;
	int32_t dest_pitch, dest_roll, dest_yaw;
	int32_t ldiff, rdiff, bdiff, ydiff;

	int32_t cur_x, cur_y, cur_z;

	/* cli(); although reading single-byte should be ok */
	int co_right = rx_co_right, cy_right = rx_cy_right,
		cy_front = rx_cy_front, co_throttle = rx_co_throttle;
	/* sti(); */

	int32_t throttle_left, throttle_right, throttle_rear, rudder;

	co_throttle -= 10;
	if (co_throttle < 0)
		co_throttle = 0;

	if (co_throttle > 0x30)
		co_throttle += (0x30 - 0x10) * 2;
	else if (co_throttle > 0x10)
		co_throttle += (co_throttle - 0x10) * 2;

	/* Yaw stick deadband in heading-hold mode */
	if (modes & (1 << MODE_HOLD_ENABLE)) {
		co_right += 0x80 - yaw_deadband_pos;
		if (co_right >= 0x80 - 3 && co_right <= 0x80 + 3)
			co_right = 0x80;
		else if (co_right > 0x80)
			co_right -= 3;
		else
			co_right += 3;
	} else
		yaw_deadband_pos = co_right;

	if (modes & (1 << MODE_HOLD_ENABLE)) {
		cy_right += 0x80 - roll_deadband_pos;
		if (cy_right >= 0x80 - 3 && cy_right <= 0x80 + 3)
			cy_right = 0x80;
		else if (cy_right > 0x80)
			cy_right -= 3;
		else
			cy_right += 3;
	} else
		roll_deadband_pos = cy_right;

	if (modes & (1 << MODE_HOLD_ENABLE)) {
		cy_front += 0x80 - pitch_deadband_pos;
		if (cy_front >= 0x80 - 3 && cy_front <= 0x80 + 3)
			cy_front = 0x80;
		else if (cy_front > 0x80)
			cy_front -= 3;
		else
			cy_front += 3;
	} else
		pitch_deadband_pos = cy_front;

	/* TODO: around the time of take-off measure the throttle
	 * needed to hover and set something just slightly lower on emergency */
#define hover_thr 0x80

#if 1
	if (co_throttle > hover_thr && !(modes & (1 << MODE_MOTORS_ARMED))) {
		config_update(co_right, cy_right, cy_front);
		return;
	}
#endif

	if (modes & (1 << MODE_EMERGENCY)) {
		/* TODO: lots to do here, many possibilities to consider... */
		co_throttle = hover_thr;
		co_right = 0x80;
		cy_front = 0x80;
		cy_right = 0x80;

		/* Pull the cyclic stick back to exit RTL / Loiter */
		if (rx_no_signal < 5)/// && cy_front < 0x80 - 30)
			modes &= ~(1 << MODE_EMERGENCY);
	} else if (unlikely(rx_no_signal > 15)) {
		co_right = 0x80;
		cy_front = 0x80;
		cy_right = 0x80;
		if (rx_no_signal > 250)
			if ((modes & (1 << MODE_MOTORS_ARMED)) &&
					!(modes & (1 << MODE_PPM_STOP)) &&
					co_throttle >= hover_thr * 3 / 4)
				modes |= 1 << MODE_EMERGENCY;
	}

#define CLAMP(x, mi, ma)	\
	if (x < mi)		\
		x = mi;		\
	if (x > ma)		\
		x = ma;
#define CLAMPS(x, y, mi, ma)	\
	if (x < mi)		\
		y = mi;		\
	if (x > ma)		\
		y = ma;

	/* Apply a poor man's Expo */
#define EXPO(x, op, n)	\
	if (x op 128 + (n))	\
		x += x - (128 + (n));
	EXPO(cy_front, >, 40);
	EXPO(cy_front, <, -40);
	EXPO(cy_right, >, 40);
	EXPO(cy_right, <, -40);
	//EXPO(cy_front, >, 120);
	//EXPO(cy_front, <, -120);
	//EXPO(cy_right, >, 120);
	//EXPO(cy_right, <, -120);

	cur_x = (q1q3 - q0q2) * 4096.0f;
	cur_y = (q2q3 + q0q1) * 4096.0f;
	cur_z = (1.0f - q1q1 - q2q2) * 4096.0f;

	/* TODO: in all the formulas below, use the squares of the
	 * rotation rates?  */
	/* TODO; clamp the cur_x, cur_y, cur_z before using in
	 * dest_yaw/pitch/roll */
	dest_pitch = ((int) cy_front - 128) << 4;
	dest_roll = ((int) cy_right - 128) << 4;
	dest_yaw = ((int) co_right - 128) << 9;

	if (modes & ((1 << MODE_HOLD_ENABLE) | (1 << MODE_EMERGENCY))) {
		/*TODO:if (cur_x > 0) {
			if (cur_x >= 7000 || abs(cur_z) >= abs(cur_y)) {
				if (cur_z > 0)
					cur_z = 4096;
				else
					cur_z = -4096;
				cur_y = -cur_y;
			} else {
				if (cur_y > 0)
					cur_y = 4096;
				else
					cur_y = -4096;
				cur_z = -cur_z;
			}
		}*/

		cur_pitch = 0;
		cur_yaw = 0;
		cur_roll = 0;

		if (cur_z < 0) {
			if (cur_y > 0)
				cur_y -= cur_z;
			else
				cur_y += cur_z;
			if (cur_x > 0)
				cur_x -= cur_z;
			else
				cur_x += cur_z;
		}
		if (modes & ((1 << MODE_PITCHLEVEL_ENABLE) |
				(1 << MODE_EMERGENCY)))
			cur_pitch -= ((cur_y - neutral_y) *
					(int16_t) (uint16_t)
					config[CFG_ROLLPITCH_P]) / 32;
		if (modes & ((1 << MODE_ROLLLEVEL_ENABLE) |
				(1 << MODE_EMERGENCY)))
			cur_roll -= ((cur_x - neutral_x) *
					(int16_t) (uint16_t)
					config[CFG_ROLLPITCH_P]) / 32;

		/* Constrain the proportional factors */
		CLAMP(cur_pitch, -2000, 2000);
		CLAMP(cur_roll, -2000, 2000);

		/* TODO: we need to build a quaternion out of ahrs_pitch_rate,
		 * ahrs_roll_rate, ahrs_yaw_rate, and rotate [ 0, 0, 1 ] by the
		 * product of the two quaternions instead.
		 */
		/* The rotation rates here are taken with a factor of about
		 * 1/4th so in effect we take the roll, pitch, yaw we might
		 * have in about 250ms from now if the rotation rate remained
		 * unchanged.
		 *
		 * The calc for reference:
		 *
		 * cur_x unit can be said, with the sin(x) = x simplification,
		 * to be 180 / pi / 4096 deg ~= 0.014 deg.
		 *
		 * cur_pitch unit is currenlty 4x that, so 0.056 deg.
		 *
		 * ahrs_roll_rate unit = 1 / 64 deg/s ~= 0.016 deg/s
		 */
		{
			static int32_t prev_roll_rate = 0;
			static int32_t prev_yaw_rate = 0;
			static int32_t prev_pitch_rate = 0;
			int32_t roll_d, yaw_d, pitch_d;

			/* Get the derivative */
			roll_d = ahrs_roll_rate;
			yaw_d = ahrs_yaw_rate;
			pitch_d = ahrs_pitch_rate;

			/*
			 * Also add the second derivative.  With this I'm
			 * not sure if this can be called a PID loop anymore,
			 * but the second derivative is desirable because
			 * not only the whole airship body has a momentum,
			 * but also each propeller has a momentum of its
			 * own which, within some range, helps the rigid
			 * body maintain its angular acceleration.
			 */
			/* TODO: need to divide by time period */
			roll_d += (ahrs_roll_rate - prev_roll_rate) / 4;
			yaw_d += (ahrs_yaw_rate - prev_yaw_rate) / 4;
			pitch_d += (ahrs_pitch_rate - prev_pitch_rate) / 4;

			prev_roll_rate = ahrs_roll_rate;
			prev_yaw_rate = ahrs_yaw_rate;
			prev_pitch_rate = ahrs_pitch_rate;

			cur_pitch -= (roll_d * (int16_t) (uint16_t)
					config[CFG_ROLLPITCH_D]) / 32;
			cur_yaw -= (yaw_d * (int16_t) (uint16_t)
					config[CFG_YAW_P]) / 32;
			cur_roll += (pitch_d * (int16_t) (uint16_t)
					config[CFG_ROLLPITCH_D]) / 32;
		}

		dest_pitch = dest_pitch - cur_pitch;
		dest_roll = dest_roll - cur_roll;
		dest_yaw = dest_yaw - cur_yaw;
	}

	/* 7 / 8 is about 3 / 2 / sqrt(3) which is 1 / arm length for pitch. */
	ldiff = -dest_pitch * 7 / 8 + dest_roll; // + dest_yaw * 0.001
	rdiff = -dest_pitch * 7 / 8 - dest_roll; // - dest_yaw * 0.001
	bdiff = dest_pitch * 7 / 8 + (abs(dest_yaw) >> 6);
	ydiff = dest_yaw;
	{ /* HACK filter */
		//// convert this to YAW_D?
		static int32_t ydiffr = 0;
		ydiffr += ydiff - ydiffr / 16;////
		ydiff = ydiffr / 16;
	}

	if (modes & (1 << MODE_ADAPTIVE_ENABLE)) { /* Adapt coefficients */
		/* TODO: Maintan also a slow (longer term) average, then use
		 * quick - slow = a and slow = b in
		 * y = a * x + b
		 * Note that this needs to be _simple_ */
#define quick_avg(i) ((int32_t) config[CFG_ADAPTIVE_0 + i])
#define quick_avg_raw(i) config[CFG_ADAPTIVE_0 + i]

		if (co_throttle > hover_thr) { /* Adapt coefficients */
			int32_t sum;

			quick_avg_raw(0) += ldiff / 16;
			quick_avg_raw(1) += rdiff / 16;
			quick_avg_raw(2) += bdiff / 16;
			quick_avg_raw(3) += ydiff / 16;

			/* Balance the three motor throttles around zero */
			sum = (quick_avg(0) + quick_avg(1) + quick_avg(2)) / 4;
			quick_avg_raw(0) -= sum;
			quick_avg_raw(1) -= sum;
			quick_avg_raw(2) -= sum;
#define CO 2
#define HALFCO (1 << (CO - 1))
			CLAMPS(quick_avg(0), quick_avg_raw(0),
					-0x3000l << CO, 0x3000l << CO);
			CLAMPS(quick_avg(1), quick_avg_raw(1),
					-0x3000l << CO, 0x3000l << CO);
			CLAMPS(quick_avg(2), quick_avg_raw(2),
					-0x3000l << CO, 0x3000l << CO);
			CLAMPS(quick_avg(3), quick_avg_raw(3),
					-0x5000l << CO, 0x5000l << CO);
		}

		ldiff += (quick_avg(0) + HALFCO) >> CO;
		rdiff += (quick_avg(1) + HALFCO) >> CO;
		bdiff += (quick_avg(2) + HALFCO) >> CO;
		ydiff += (quick_avg(3) + HALFCO) >> CO;
	}

	CLAMP(ldiff, -0x6fff, 0x6fff);
	CLAMP(rdiff, -0x6fff, 0x6fff);
	CLAMP(bdiff, -0x6fff, 0x6fff);
	if (co_throttle < 50) {
		throttle_left = ((uint32_t) co_throttle *
				(uint16_t) (0x8000 + ldiff)) >> 8;
		throttle_right = ((uint32_t) co_throttle *
				(uint16_t) (0x8000 + rdiff)) >> 8;
		throttle_rear = ((uint32_t) co_throttle *
				(uint16_t) (0x8000 + bdiff)) >> 8;
	} else {
		throttle_left = ((uint32_t) co_throttle * 0x8000 +
				200 * ldiff) >> 8;
		throttle_right = ((uint32_t) co_throttle * 0x8000 +
				200 * rdiff) >> 8;
		throttle_rear = ((uint32_t) co_throttle * 0x8000 +
				200 * bdiff) >> 8;
	}
	rudder = 0x6c00 + ydiff;

	CLAMP(throttle_left, 0, 0xa000);
	CLAMP(throttle_right, 0, 0xa000);
	CLAMP(throttle_rear, 0, 0xa000);
	CLAMP(rudder, 0, 0xffff);

	if (modes & (1 << MODE_MOTORS_ARMED)) {
		actuator_serial_set(LEFT_MOTOR, (uint16_t) throttle_left);
		actuator_serial_set(RIGHT_MOTOR, (uint16_t) throttle_right);
		actuator_serial_set(RUDDER_MOTOR, (uint16_t) throttle_rear);
	}

	actuator_hwpwm_set(RUDDER_SERVO, (uint16_t) rudder);

	output[0] = (uint16_t) throttle_left;
	output[1] = (uint16_t) throttle_right;
	output[2] = (uint16_t) throttle_rear;
	output[3] = (uint16_t) rudder;
}

#define TEXT_DEBUG
#ifndef TEXT_DEBUG
static void serial_write2(float v) {
	uint16_t b = (v + 1.0f) * 32768.0f;
	serial_write1((b >> 8) & 0xff);
	serial_write1((b >> 0) & 0xff);
}
static void serial_write_bin16(uint16_t v) {
	serial_write1(v >> 8);
	serial_write1(v >> 0);
}
#if 0
static void serial_write_bin32(uint32_t v) {
	serial_write_bin16(v >> 16);
	serial_write_bin16(v >> 0);
}
#endif
#endif

static int32_t pressure = 0;

int fps = 0;////
static void status_update(void) {
	if (ahrs_report) {
#ifdef TEXT_DEBUG
		serial_write_fp32((101300 - pressure) * 83, 1000);////
		serial_write_hex16(fps);////
		serial_write_hex32(timer_read());////
		int16_t cur_x = (q1q3 - q0q2) * 4096.0f;
		int16_t cur_y = (q2q3 + q0q1) * 4096.0f;
		int16_t cur_z = (1.0f - q1q1 - q2q2) * 4096.0f;
		serial_write_fp32(cur_x, 16);
		serial_write_fp32(cur_y, 16);
		serial_write_fp32(cur_z, 16);
		serial_write_fp32(ahrs_pitch_rate, 16);
		serial_write_fp32(ahrs_roll_rate, 16);
		serial_write_fp32(ahrs_yaw_rate, 16);
		serial_write_str("\r\n");
#else
		if (rx_no_signal > 3)
			serial_write1(0xe2); /* ID */
		serial_write1(0xe0); /* ID */
		serial_write2(q[0]);
		serial_write2(q[1]);
		serial_write2(q[2]);
		serial_write2(q[3]);
		serial_write2(mag[0]);
		serial_write2(mag[1]);
		serial_write2(mag[2]);
		/* TODO: accumulate acc[]s between reports? */
		serial_write1((uint8_t) (int8_t) (acc[0] * 64.0f));
		serial_write1((uint8_t) (int8_t) (acc[1] * 64.0f));
		serial_write1((uint8_t) (int8_t) (acc[2] * 64.0f));
		serial_write_bin16(timer_read_hi());
		//serial_write1(actuators[0] >> 7);
#endif
	}
#if 0
	if (gps_report) {
		gps_update();
		if (gps_avail) {
			serial_write1(0xe1); /* ID */
			serial_write_bin32(gps_time);
			serial_write_bin32(gps_lat);
			serial_write_bin32(gps_lon);
			serial_write_bin16(gps_alt);
			serial_write_bin16(gps_hdop);
			serial_write1(modes);
			serial_write1(adc_convert(8) - 205);
		}
	}
#endif

	if (rx_report) {
#ifdef TEXT_DEBUG
		/*
		serial_write_hex16(rx_ch[0]);
		serial_write_hex16(rx_ch[1]);
		serial_write_hex16(rx_ch[2]);
		serial_write_hex16(rx_ch[3]);
		serial_write_hex16(rx_ch[4]);
		serial_write_hex16(rx_ch[5]);
		serial_write_hex16(rx_ch[6]);
		serial_write_hex16(rx_ch[7]);
		*/
		serial_write_dec8(rx_gear_sw);
		serial_write_dec8(rx_id_sw);
		serial_write_dec8(rx_co_throttle);
		serial_write_dec8(rx_co_right);
		serial_write_dec8(rx_cy_front);
		serial_write_dec8(rx_cy_right);
		serial_write_hex16(output[0]);
		serial_write_hex16(output[1]);
		serial_write_hex16(output[2]);
		serial_write_hex16(output[3]);
		serial_write_str("\r\n");
#else
#endif
	}
}

static uint32_t ts = 0;
static void loop(void) {
	static int counter = 0;
	static unsigned int stopped = 0;
	int32_t p;
	fps ++;////

	/* No set udpate rate, go as fast as possible */
#if 0
	my_delay(10); /*  10ms - 100Hz update rate */
#endif

	if (timer_read() > ts) {
		counter ++;
		ts += F_CPU / 50; /* About every 20ms (50Hz) */
		rx_no_signal = (rx_no_signal < 255) ? rx_no_signal + 1 : 255;
	}

	modes_update();
	ahrs_update();
	control_update();

	if (counter > 10) { /* About every 0.2s */
		counter = 0;
		status_update();
	}

	p = bmp085_read();
	if (p) {
		if (pressure)
			pressure += (p - pressure + 4) / 8;
		else
			pressure = p;
	}

	if (((modes >> MODE_PPM_STOP) & 1) != stopped) {
		stopped = (modes >> MODE_PPM_STOP) & 1;
		if (stopped) {
			actuators_serial_stop();
			actuators_hwpwm_stop();
		} else {
			actuators_serial_start();
			actuators_hwpwm_start();
		}
	}
}

int main(void) {
	setup();

	ts = timer_read();
	for (;;)
		loop();

	return 0;
}
