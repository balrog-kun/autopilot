/*
 * A simple fixed-point AHRS based on intertial and magentic sensors.
 *
 * Licensed under AGPLv3.
 *
 * This is probably the "heart" of the autopilot.
 *
 * This currently uses an "8-DoF" set of sensors: a 2-axis gyro and a 3-axis
 * accelerometer and magnetometer.  It should be easy to adapt to any other
 * set of sensors however.  The gyro output is integrated to obtain th
 * current attitude, but due to integration, it is susceptible to drift.  It
 * is reliable in detecting sudden turns however and is also a reliable
 * source of current rotation rate.  The magnetic field vector and the gravity
 * vector (approximated from the filtered accelerometer readings over a couple
 * of seconds) are assumed to not drift and are used as a reference to remove
 * the gyroscope's drift.  Their expected values are compared against the
 * "current" values and the attitude obtained from integrating rotation rate
 * is corrected based on this.  It should be easy to add more reference
 * vectors or work without one of them, or use a 3-axis gyro instead of
 * the 2-axis one.
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include "adc.h"
#include "twi.h"
#include "cmps09.h"
#include "timer1.h"
#include "ahrs.h"
#include "trig.h"

volatile int32_t ahrs_pitch, ahrs_roll;
volatile int16_t ahrs_yaw, ahrs_pitch_rate, ahrs_roll_rate, ahrs_yaw_rate;

static volatile int32_t rel_pitch, rel_roll;

/* Doesn't seem to make a whole lot of difference */
#define USE_REFERENCE_V
volatile static uint32_t now;
static uint32_t prev_ts;
static int32_t x_ref, y_ref;

static void gyro_ahrs_update(void) {
	uint32_t diff;
	uint16_t x, y;

	cli();
	x = adc_values[0];
	y = adc_values[1];

	diff = now - prev_ts;
	prev_ts = now;

#ifdef USE_REFERENCE_V
	x -= adc_values[2];
	y -= adc_values[2];
#endif

	/*
	 * Assuming the 10-bit ADC's reference volatage 3.3V and
	 * the gyro outputs to change 3.3 mV/deg/s, 1 LSB of
	 * x/y/ref corresponds to 0.9765625 deg/s, so nearly 1
	 * deg/s.  Assuming F_CPU of 16M reduced by TIME_RES bits,
	 * at 32 bits to represent 360 degrees of rotation we can
	 * afford almost exactly TIME_RES fractional bits in
	 * x_ref/y_ref, e.g:
	 * ((2 ** 32) / (0.9765625 * 360 * (16000000 >> 5)) == 24.43)
	 *
	 * We assume here that the time since last conversion is never
	 * longer than some 30ms, if it is too long this may overflow.
	 */
#define TIME_RES 2 /* 1+1 because adc_values are doubled */
#define DIFF_RES 4
#define REF_RES (DIFF_RES + 6)

	ahrs_roll_rate = ((x_ref + (1 << (REF_RES - 7))) >> (REF_RES - 6)) -
		((int16_t) x << 6);
	ahrs_pitch_rate = ((y_ref + (1 << (REF_RES - 7))) >> (REF_RES - 6)) -
		((int16_t) y << 6);
	sei();

	diff = (diff/* + (1 << (DIFF_RES - 1))*/) >> DIFF_RES;
	rel_roll += (int32_t) ((((int32_t) (int16_t) x << REF_RES) - x_ref) *
			diff + (1 << (REF_RES - DIFF_RES + TIME_RES - 1))) >>
		(REF_RES - DIFF_RES + TIME_RES);
	rel_pitch += (int32_t) ((((int32_t) (int16_t) y << REF_RES) - y_ref) *
			diff + (1 << (REF_RES - DIFF_RES + TIME_RES - 1))) >>
		(REF_RES - DIFF_RES + TIME_RES);
}

static void gyro_update(void) {
	now = timer_read();
	adc_convert_all(gyro_ahrs_update);

	/* The gyro's bandwidth is 140Hz so schedule the next measurement
	 * 1/140 sec from this measurement.  Don't try to compensate if the
	 * timer was late this time, if the timer triggers sooner than 1/140
	 * sec next time, the measurements won't be independent.  */
	set_timeout(now + F_CPU / 140, gyro_update);
}

static void gyro_cal_update(void) {
	x_ref += adc_values[0];
	y_ref += adc_values[1];
#ifdef USE_REFERENCE_V
	x_ref -= adc_values[2];
	y_ref -= adc_values[2];
#endif
}

static uint32_t v_ts;
static int16_t statica[3]; /* Initial Acc readings average */
static int16_t staticm[3]; /* Initial Mag readings average */
static int32_t avga[3];
static int32_t avgm[3];

static void vectors_cal(void) {
	uint8_t regs[12];

	cmps09_read_bytes(10, 12, regs);
	avgm[0] += (int16_t) (((uint16_t) regs[0] << 8) | regs[1]);
	avgm[1] += (int16_t) (((uint16_t) regs[2] << 8) | regs[3]);
	avgm[2] += (int16_t) (((uint16_t) regs[4] << 8) | regs[5]);
	avga[0] += (int16_t) (((uint16_t) regs[6] << 8) | regs[7]);
	avga[1] += (int16_t) (((uint16_t) regs[8] << 8) | regs[9]);
	avga[2] += (int16_t) (((uint16_t) regs[10] << 8) | regs[11]);
}

static void vectors_update(void) {
	int32_t pitch, roll, lensq;
	int16_t yaw;
	int16_t a[3], m[3]; /* Current Acc & Mag readings */
	int16_t rotated[3]; /* Rotated (predicted) vector */
	int16_t crossed[3]; /* Cross product of current & predicted vectors */
	uint8_t regs[12];
	uint16_t factor;

	/* The magnetometers's measurement frequency is 50Hz and the
	 * accelerometer's rate is limite by buffer referesh rate of 55Hz, so
	 * schedule the next measurement 1/50 sec from the time the previous
	 * measurement was *supposed* to happen.  Not sure if that's really
	 * better, need to think about it.  */
	v_ts += F_CPU / 50;
	set_timeout(v_ts, vectors_update);

	/* Retrieve current values of everything */

	cmps09_read_bytes(10, 12, regs); /* TODO: be async */
	m[1] = ((uint16_t) regs[0] << 8) | regs[1];
	m[0] = ((uint16_t) regs[2] << 8) | regs[3];
	m[2] = ((uint16_t) regs[4] << 8) | regs[5];
	a[0] = ((uint16_t) regs[6] << 8) | regs[7];
	a[1] = ((uint16_t) regs[8] << 8) | regs[9];
	a[2] = ((uint16_t) regs[10] << 8) | regs[11];
	m[1] -= cmps09_mag_calib[0];
	m[0] -= cmps09_mag_calib[1];
	m[2] -= cmps09_mag_calib[2];

	cmps09_xy_adjust(m);
	cmps09_xy_adjust(a);

	cli();
	pitch = -rel_pitch, rel_pitch = 0;
	roll = -rel_roll, rel_roll = 0;
	yaw = ahrs_yaw;
	sei();

	/* TODO: decoupling like in FlightCtrl */

	/* Integrate */

	pitch += ahrs_pitch;
	roll += ahrs_roll;
	if (unlikely(pitch > ROLL_PITCH_180DEG))
		pitch -= 2 * (uint32_t) ROLL_PITCH_180DEG;
	else if (unlikely(pitch < -ROLL_PITCH_180DEG))
		pitch += 2 * (uint32_t) ROLL_PITCH_180DEG;
	if (unlikely(roll > ROLL_PITCH_180DEG))
		roll -= 2 * (uint32_t) ROLL_PITCH_180DEG;
	else if (unlikely(roll < -ROLL_PITCH_180DEG))
		roll += 2 * (uint32_t) ROLL_PITCH_180DEG;

	/* Fuse information from the different sensors */

	/* TODO: use the dot product to check if the angle between
	 * current and predicted vector is > 90deg */

	/* We rotate the reference vectors to our local coordinate system
	 * and compare against our current vectors rather than rotating the
	 * measured vectors to produce the predicted/expected value and
	 * comparing against reference values.
	 *
	 * Note that while the magnetometer readings are more reliable in
	 * detecting quick changes, the readings have lower resolution than
	 * the accelerometer readings so there may be some noise caused by
	 * that.
	 *
	 * The CMPS09 magnetometer resolution seems to be about 0.001 tesla
	 * per 1 LSB, so |m| of 400 is about 0.4 tesla.
	 *
	 * The accelerometer resolution seems to be about 0.0006 m/s^2 per
	 * 1 LSB so 0x4000 is about 1g.
	 *
	 * Note that we give a higher priority to the magnetometer on the
	 * vertical axis and lower on the horizontal axes, and we do the
	 * opposite for the accelerometer.  This is because the magnetic
	 * field vector tends to be closer to horizontal (parallel to earth
	 * surface, outside of the polar areas anyway and away from big
	 * magnets), while gravity/acceleartion vector is on average vertical.
	 * If someone decides to mount the board differently to the
	 * vehicle's "normal" orientation, or the vehicle has no "normal"
	 * orientation, then these should be adjusted.  The differences in
	 * priority are on order of 2-4, so even without adjustments things
	 * should still work.
	 * Both sensors are given little priority on X/Y axes because the
	 * gyro is our primary data source.
	 */
	/*
	 * Normalise / sanitise / reality-check the magnetic vector,
	 * may be completely off..
	 * TODO: divide cross product by square length?
	 * TODO: should we compare field magnitude against constant values
	 * or the calibration vector (staticm)?  If we calibrated in an
	 * area of disturbed field, then all our later calculations will be
	 * wrong.
	 */
	lensq = hypot3(m);
	if (lensq < 300l * 300 || lensq > 500l * 500) {
		factor = 0;
		/* TODO: light a red LED? */
	} else if (lensq < 350l * 350)
		factor = lensq - 350l * 350;
	else if (lensq > 470l * 470)
		factor = 500l * 500 - lensq;
	else
		factor = 32000;

	rotate_rev(rotated, staticm, yaw, -pitch, -roll);
	cross(crossed, rotated, m, factor >> 3);

	rotate_rev(rotated, statica, yaw, -pitch, -roll);

	/* Assuming |m| and |staticm| of about 0.4T,
	 * crossed_n is about sin(angular distance) << 12
	 */
#define MAG_ROLLPITCH_PRIORITY 7
#define ACCEL_ROLLPITCH_PRIORITY 6
	yaw += (crossed[2] + 2) >> 2;
	pitch -= (int32_t) crossed[1] << MAG_ROLLPITCH_PRIORITY;
	roll += (int32_t) crossed[0] << MAG_ROLLPITCH_PRIORITY;

	cross(crossed, rotated, a, 1);
	yaw += (crossed[2] + 4) >> 3;
	pitch -= (int32_t) crossed[1] << ACCEL_ROLLPITCH_PRIORITY;
	roll += (int32_t) crossed[0] << ACCEL_ROLLPITCH_PRIORITY;
#ifdef CAL
	if (!(ahrs_pitch & 7)) {
		serial_write_hex16(m[0]);
		serial_write_hex16(m[1]);
		serial_write_hex16(m[2]);
		serial_write_dec32((int32_t) isqrt32(lensq));
		serial_write_dec32((int32_t) factor);
		serial_write_eol();
	}
#endif

	/* Write resulting current attitude */

	cli();
	ahrs_pitch = pitch;
	ahrs_roll = roll;
	ahrs_yaw_rate = yaw - ahrs_yaw;
	ahrs_yaw = yaw;
	sei();
}

void ahrs_init(void) {
	int i;

	/* Calibrate the sensors while waiting for the ESCs to detect
	 * voltages etc. */
	x_ref = y_ref = 0;
	avga[0] = avga[1] = avga[2] = 0;
	avgm[0] = avgm[1] = avgm[2] = 0;
	for (i = 0; i < (1 << REF_RES); i ++) {
		adc_convert_all(gyro_cal_update);
		my_delay(4);
		if (!(i & 7)) /* Every 32ms or so */
			vectors_cal();
	}
	statica[0] = (avga[0] + ((1 << (REF_RES - 4)) - 1)) >> (REF_RES - 3);
	statica[1] = (avga[1] + ((1 << (REF_RES - 4)) - 1)) >> (REF_RES - 3);
	statica[2] = (avga[2] + ((1 << (REF_RES - 4)) - 1)) >> (REF_RES - 3);
	staticm[1] = (avgm[0] + ((1 << (REF_RES - 4)) - 1)) >> (REF_RES - 3);
	staticm[0] = (avgm[1] + ((1 << (REF_RES - 4)) - 1)) >> (REF_RES - 3);
	staticm[2] = (avgm[2] + ((1 << (REF_RES - 4)) - 1)) >> (REF_RES - 3);
	staticm[1] -= cmps09_mag_calib[0];
	staticm[0] -= cmps09_mag_calib[1];
	staticm[2] -= cmps09_mag_calib[2];

	cmps09_xy_adjust(staticm);
	cmps09_xy_adjust(statica);
#ifdef CAL
	serial_write_hex16(statica[0]);
	serial_write_hex16(statica[1]);
	serial_write_hex16(statica[2]);
	serial_write_eol();
#endif

	/* TODO: use a constant predefiend statica such as:
	 * statica[0] = statica[1] = 0
	 */

	ahrs_pitch = ahrs_roll = ahrs_yaw = 0;
	rel_pitch = rel_roll = 0;

	/* Start the correcting vector updates first */
	v_ts = timer_read();
	vectors_update();

	/* Start the gyro output integration loop */
	prev_ts = timer_read();
	gyro_update();
}
