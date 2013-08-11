/*
 * An AHRS based on the Extended Kalman Filter.
 * Some calculations are based on the FreeIMU library by Fabio Varesano whose
 * development was supported by Universita' degli Studi di Torino within
 * the Piemonte Project.
 *
 * Licensed under AGPLv3.
 */

#include <math.h>
#include <stdint.h>

#include "twi.h"
#include "timer1.h"
#include "uart.h"
#include "cmps09.h"
#include "wmp.h"
#include "ahrs.h"
#include "l3g4200d.h"
#include "adxl345.h"
#include "hmc5883l.h"
#include "config.h"

VOL int16_t ahrs_pitch, ahrs_roll, ahrs_yaw;
VOL int16_t ahrs_pitch_rate, ahrs_roll_rate, ahrs_yaw_rate;

/* Acceleration is reported in the initial coordinate system rotated
 * by the angles above so really it is the local coordinate system
 * of the vehicle.  It should not include the gravitational
 * acceleration.
 */
//VOL int16_t accel_acceleration[3];

VOL float q[4];
VOL float mag[3], acc[3], avgalen, timediff;
VOL float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3;
VOL uint8_t mag_valid = 0, acc_valid = 0;

/* TODO: investigae if we should do what MatrixPilot does: boost Kp by:
 * 1x        if gyro rate < 50deg/sec
 * rate / 50 if gyre rate >= 50 but < 500deg/sec
 * 10x       if >= 500deg/sec
 *
 * Also set Ki 0 if gyro rate >= 20deg/sec?
 *
 * Also, assume rate never changes more than 0.1 between consecutive
 * measurements??
 */
static const float two_ki = 1.0 * 0.0001; /* 1 x integral gain */
static const float two_kp = 1.0 * 0.4; /* 1 x proportional gain */
static float integral_fb[3];
static uint32_t last_update;

static float staticg[3]; /* Zero-rate gyro reading.. believed to be... */
static float staticm[2]; /* Initial Mag readings average */

int abs(int);

/* Quake inverse square root */
static float inv_sqrt(float number) {
	uint32_t i = 0x5f375a86 - ((*(uint32_t *) &number) >> 1);
	return (*(float *) &i) *
		(1.5f - (number * 0.5f * (*(float *) &i) * (*(float *) &i)));
}

#define sqrt sqrtf
static uint16_t ccsqrt(uint32_t x) {
	/* This is mainly written for gcc on arm thumb2, on other architectures
	 * using uint16_t may generate better code.  When initialising the
	 * loop we could just set res = 0, i = 1 << 15 and we would get
	 * shorter code that is potentially slower.  */
	uint32_t ret;
	uint32_t i;

	asm volatile ("clz  %0, %1" : "=r" (i) : "r" (x));
	ret = 1 << ((31 - i) >> 1);
	for (i = ret >> 1; i; i >>= 1) {
		uint32_t t = ret | i;
		if (x >= t * t)
			ret |= i;
	}

	return ret;
}

static uint32_t hypot3(int16_t v[3]) {
	return ccsqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

static float hypot3f(float v[3]) {
	return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

static void mag_static_reset(void) {
	float g[3], gxm[3], invlen;

	/* The other way to do this would be take x = local_to_global(m);
	 *   staticm[0] = hypot(x[0], x[1]);
	 *   staticm[1] = x[2];
	 */

	/* Estimated gravity vector */
	g[0] = q0q2 - q1q3;
	g[1] = -q0q1 - q2q3;
	g[2] = q1q1 + q2q2 - 1.0f;

	gxm[0] = mag[1] * g[2] - mag[2] * g[1];
	gxm[1] = mag[2] * g[0] - mag[0] * g[2];
	gxm[2] = mag[0] * g[1] - mag[1] * g[0];
	staticm[0] = gxm[0] * gxm[0] + gxm[1] * gxm[1] + gxm[2] * gxm[2];
	staticm[1] = mag[0] * g[0] + mag[1] * g[1] + mag[2] * g[2];
	invlen = inv_sqrt(staticm[0] + staticm[1] * staticm[1]);
	/* TODO: use ccsqrt */
	staticm[0] = sqrtf(staticm[0]) * invlen;
	staticm[1] *= -invlen;
}

static float mag_calib_update(void) {
	static int32_t pitch_cnt;
	static int32_t yaw_cnt;
	static int32_t sample_cnt;
	static float prev_m[3];
	/*static float p0p0, p0p1, p0p2, p0p3, p1p1, p1p2, p1p3, p2p2, p2p3;*/
	static uint32_t min_len = 10000;
	static uint32_t max_len = 0;
	float t[3], m[3], diff_len, prev_len;
	int16_t imag[3];

	uint32_t len;
	static int invalid_cnt = 30;
	static int calibrating = 1;

	m[0] = mag[0];
	m[1] = mag[1];
	m[2] = mag[2];
	mag[0] -= mag_calib_x;
	mag[1] -= mag_calib_y;
	mag[2] -= mag_calib_z;
	imag[0] = mag[0] + 0.5f;
	imag[1] = mag[1] + 0.5f;
	imag[2] = mag[2] + 0.5f;
	len = hypot3(imag);

	if (len > min_len && len < max_len) {
		mag_valid = !calibrating;
		if (invalid_cnt > 0)
			invalid_cnt --;
	} else {
		mag_valid = 0;
		if (invalid_cnt < 30)
			invalid_cnt ++;
	}

	if (invalid_cnt > 10 && !calibrating) {
		calibrating = 1;
		sample_cnt = 0;
	}

	if (!calibrating)
		return len;

	/* Magnetometer is considered not calibrated, either because we've
	 * just booted up and there haven't been enough rotation to make
	 * sure that the values can be trusted, or because something in
	 * the vehicle's body changed the layout of soft-iron/hard-
	 * iron/whatever materials and previous calibration values became
	 * invalid */

	if (!sample_cnt) {
		prev_m[0] = m[0], prev_m[1] = m[1], prev_m[2] = m[2];
#if 0
		p0p0 = q0q0, p0p1 = q0q1, p0p2 = q0q2, p0p3 = q0q3,
			p1p1 = q1q1, p1p2 = q1q2, p1p3 = q1q3,
			p2p2 = q2q2, p2p3 = q2q3;
#endif
		min_len = 10000;
		max_len = 0;
		pitch_cnt = 0;
		yaw_cnt = 0;
		sample_cnt ++;
		return len;
	}

	if (len < min_len)
		min_len = len;
	if (len > max_len)
		max_len = len;
	if (min_len * 20 < max_len * 19) {
		/* Start the whole procedure again */
		sample_cnt = 1;
		pitch_cnt = 0;
		yaw_cnt = 0;
		min_len = 10000;
		max_len = 0;
	}

	pitch_cnt += ahrs_pitch_rate;
	yaw_cnt += ahrs_yaw_rate;

	if (fabsf(m[0] - prev_m[0]) + fabsf(m[1] - prev_m[1]) +
			fabsf(m[2] - prev_m[2]) < 60.0f)
		return len;

	/* Iteratively improve the offset vector moving it closer to the true
	 * value in the plane containing the latest two measurements.  This is
	 * just the solution of the obvious equation obtained by taking:
	 *
	 *    mag_field_measurement = offset + local_to_global(actual_mag_field)
	 *
	 * for two measurements.  The change in local_to_global obviously
	 * relies on the gyroscope and partially the accelerometer measurements.
	 * It does not take the magnetometer into account because mag_valid is
	 * false while we're calibrating.  So the magnetometer is being
	 * corrected using gyroscope readings when !mag_valid, and gyroscope
	 * readings are corrected using the magnetometer readings when
	 * mag_valid.  Not good.  But it should work for now.
	 *
	 * It's also possible to calibrate the magnetometer offset without
	 * relying on any other data, by sphere fitting.  This would be much
	 * more reliable, and actually can be extended to ellipsoid fitting to
	 * take into account not only soft/hard-iron effects but also other
	 * calibration parameters.  But it's awfully complicated and the
	 * ellipsoid fitting problem, as far as I know, is actually very very
	 * slow to converge.
	 *
	 * So we use this simple equation and take advantage of the gyros.
	 * The exact maths used here comes from William Permerlani's
	 * NullingMagnetometerOffset.pdf.
	 *
	 * global_to_local1 * local_to_global0 is:
	 * |t[0] t[1] t[2]|
	 * |t[3] t[4] t[5]|
	 * |t[6] t[7] t[8]|
	 *
	 * b += t*(b1 + b2 - b21 * b2 - b21t * b1);
	 */
	/* TODO: do all this after a new quaternion is calculated in
	 * ahrs_update() */
#if 0
	t[0] = (q0q0 + q1q1 - 1) * (p0p0 + p1p1 - 1) +
		(q1q2 - q0q3) * (p1p2 - p0p3) +
		(q1q3 + q0q2) * (p1p3 + p0p2);
	t[1] = (q0q0 + q1q1 - 1) * (p1p2 + p0p3) +
		(q1q2 - q0q3) * (p0p0 + p2p2 - 1) +
		(q1q3 + q0q2) * (p2p3 - p0p1);
	t[2] = (q0q0 + q1q1 - 1) * (p1p3 - p0p2) +
		(q1q2 - q0q3) * (p2p3 + p0p1) +
		(q1q3 + q0q2) * (1 - p1p1 - p1p2);
	t[3] = (q1q2 + q0q3) * (p0p0 + p1p1 - 1) +
		(q0q0 + q2q2 - 1) * (p1p2 - p0p3) +
		(q2q3 - q0q1) * (p1p3 + p0p2);
	t[4] = (q1q2 + q0q3) * (p1p2 + p0p3) +
		(q0q0 + q2q2 - 1) * (p0p0 + p2p2 - 1) +
		(q2q3 - q0q1) * (p2p3 - p0p1);
	t[5] = (q1q2 + q0q3) * (p1p3 - p0p2) +
		(q0q0 + q2q2 - 1) * (p2p3 + p0p1) +
		(q2q3 - q0q1) * (1 - p1p1 - p1p2);
	t[6] = (q1q3 - q0q2) * (p0p0 + p1p1 - 1) +
		(q2q3 + q0q1) * (p1p2 - p0p3) +
		(1 - q1q1 - q2q2) * (p1p3 + p0p2);
	t[7] = (q1q3 - q0q2) * (p1p2 + p0p3) +
		(q2q3 + q0q1) * (p0p0 + p2p2 - 1) +
		(1 - q1q1 - q2q2) * (p2p3 - p0p1);
	t[8] = (q1q3 - q0q2) * (p1p3 - p0p2) +
		(q2q3 + q0q1) * (p2p3 + p0p1) +
		(1 - q1q1 - q2q2) * (1 - p1p1 - p1p2);

	mag_calib_x += 0.3 * (m[0] + prev_m[0] -
		(t[0] * prev_m[0] + t[1] * prev_m[1] + t[2] * prev_m[2]) -
		(t[0] *      m[0] + t[3] *      m[1] + t[6] *      m[2]));
	mag_calib_y += 0.3 * (m[1] + prev_m[1] -
		(t[3] * prev_m[0] + t[4] * prev_m[1] + t[5] * prev_m[2]) -
		(t[1] *      m[0] + t[4] *      m[1] + t[7] *      m[2]));
	mag_calib_z += 0.3 * (m[2] + prev_m[2] -
		(t[6] * prev_m[0] + t[7] * prev_m[1] + t[8] * prev_m[2]) -
		(t[2] *      m[0] + t[5] *      m[1] + t[8] *      m[2]));
#else
	/* Use William Permerlani's second method ("revised") of
	 * magnetometer nulling.  This one converges more slowly it seems,
	 * but is much easier on the CPU, and much much much easier to
	 * implement.  It also doesn't use any externel input like
	 * the gyroscope measures or the current attitude estimate.
	 *
	 * The previous code above doesn't really work for some reason
	 * and for now I'm not trying to fix the remaining bugs because
	 * the one below is mathematically more correct and so much easier,
	 * only slower.
	 */
	t[0] = m[0] - prev_m[0];
	t[1] = m[1] - prev_m[1];
	t[2] = m[2] - prev_m[2];
	/* TODO: use ccsqrt */
	diff_len = sqrtf(t[0] * t[0] + t[1] * t[1] + t[2] * t[2]);
	prev_m[0] -= mag_calib_x;
	prev_m[1] -= mag_calib_y;
	prev_m[2] -= mag_calib_z;
	prev_len = sqrtf(prev_m[0] * prev_m[0] + prev_m[1] * prev_m[1] +
			prev_m[2] * prev_m[2]);

	mag_calib_x += 1.0f * t[0] / diff_len * ((float) len - prev_len);
	mag_calib_y += 1.0f * t[1] / diff_len * ((float) len - prev_len);
	mag_calib_z += 1.0f * t[2] / diff_len * ((float) len - prev_len);
#endif

	if (abs(pitch_cnt) > 64 * 30 && abs(yaw_cnt) > 64 * 30 &&
			invalid_cnt == 0 && sample_cnt > 15) {
		max_len = max_len * 1.10f;
		min_len = min_len * 0.90f;
		calibrating = 0;
		mag_valid = 1;
		mag[0] = m[0] - mag_calib_x;
		mag[1] = m[1] - mag_calib_y;
		mag[2] = m[2] - mag_calib_z;
		/* reset nominal magnetic vector again, staticm[] */
		mag_static_reset();
		return len;
	}

	/* Save new values for the next update */
	prev_m[0] = m[0], prev_m[1] = m[1], prev_m[2] = m[2];
#if 0
	p0p0 = q0q0, p0p1 = q0q1, p0p2 = q0q2, p0p3 = q0q3,
		p1p1 = q1q1, p1p2 = q1q2, p1p3 = q1q3,
		p2p2 = q2q2, p2p3 = q2q3;
#endif
	sample_cnt ++;
	return len;
}

static void vectors_update(void) {
	float recip_norm;
	float e[3] = { 0.0f, 0.0f, 0.0f };
	float qa, qb, qc;
	float g[3], grav[3];
	int16_t graw[3];///, gscale[3];
	int16_t araw[3], mraw[3];
	float len;
	uint32_t now;
	static int cnt = 0;

	/*
	 * Note the comments below are hardware-specific, out-of-date and
	 * often out of touch with the code they refer to.
	 */

	/* The gyro's bandwidth is 140Hz so schedule the next measurement
	 * 1/140 sec from this measurement.  Don't try to compensate if the
	 * timer was late this time because if the timer triggers sooner
	 * than 1/140 sec next time, the measurements won't be independent.  */
	/* ... */
	/* The magnetometers's measurement frequency is 50Hz and the
	 * accelerometer's rate is limited by buffer referesh rate of 55Hz, so
	 * schedule the next measurement 1/50 sec from the time the previous
	 * measurement was *supposed* to happen.  Not sure if that's really
	 * better, need to think about it.  */
#ifdef INTR
	set_timeout(timer_read() + F_CPU / 30, vectors_update);
#endif

	/* Retrieve current values of everything */
#if 0
	wmp_read(graw, gscale); /* TODO: average over 2+ readings? */
	cmps09_read(araw, mraw);
	mraw[0] = -(mraw[0] - cmps09_mag_calib[0]);
	mraw[1] = (mraw[1] - cmps09_mag_calib[1]);
	mraw[2] = -(mraw[2] - cmps09_mag_calib[2]);
#endif

#if 0
	g[0] = (((float) graw[0] - 0x2000) * (gscale[0] / 1000.0) -
			staticg[0]) * (M_PI / 0.00227 * 1.00 / 0x2100 / 180);
	g[1] = -(((float) graw[1] - 0x2000) * (gscale[1] / 1000.0) -
			staticg[1]) * (M_PI / 0.00227 * 1.00 / 0x2100 / 180);
	g[2] = (((float) graw[2] - 0x2000) * (gscale[2] / 1000.0) -
			staticg[2]) * (M_PI / 0.00227 * 1.00 / 0x2100 / 180);
#endif
#if 0
	cmps09_xy_adjust(m);
	cmps09_xy_adjust(a);
#endif

#if 0
	g[0] = (float) (int32_t) (((uint32_t) graw[0] << 9) - avgg[0]) *
		M_PI / 0.00227 * 1.35 / 0x2500 / 512 / 180;
	g[1] = -(float) (int32_t) (((uint32_t) graw[1] << 9) - avgg[1]) *
		M_PI / 0.00227 * 1.35 / 0x2500 / 512 / 180;
	g[2] = (float) (int32_t) (((uint32_t) graw[2] << 9) - avgg[2]) *
		M_PI / 0.00227 * 1.35 / 0x2500 / 512 / 180;
	g[0] *= gscale[0] / 1000.0;
	g[1] *= gscale[1] / 1000.0;
	g[2] *= gscale[2] / 1000.0;
#endif

	adxl345_read(araw);

	araw[0] = -araw[0];
	araw[1] = -araw[1];
	araw[2] = -araw[2];
	/* Hardcoded manually-collected axis scale values for now */
	/* TODO: some form of auto-calibration? */
#define ACCEL_X_SCALE (1.0f / 0.995f)
#define ACCEL_Y_SCALE (1.0f / 1.055f)
#define ACCEL_Z_SCALE (1.0f / 0.915f)
	acc[0] = (float) araw[0] * ACCEL_X_SCALE;
	acc[1] = (float) araw[1] * ACCEL_Y_SCALE;
	acc[2] = (float) araw[2] * ACCEL_Z_SCALE;

	hmc5883l_read(mraw);

	mag[0] = (float) mraw[0];
	mag[1] = (float) mraw[1];
	mag[2] = (float) mraw[2];
	len = mag_calib_update();

	l3g4200d_read(graw);
	/* REVISIT: 0.00875 or 0.0076294? */
	g[0] = (graw[0] - staticg[0]) * (float) (0.00875 * M_PI / 180.0);
	g[1] = (graw[1] - staticg[1]) * (float) (0.00875 * M_PI / 180.0);
	g[2] = (graw[2] - staticg[2]) * (float) (0.00875 * M_PI / 180.0);

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
	 */

	/* Use magnetometer measurement only when valid (avoids NaN
	 * in magnetometer normalisation): between 0.25 - 0.65 gauss */
	/* if (len > 256 && len < 666) */

	if (mag_valid) {
		float bx, bz;
		float w[3], m[3];

		/* Normalise magnetometer measurement */
		m[0] = mag[0] / len;
		m[1] = mag[1] / len;
		m[2] = mag[2] / len;

#if 0
		/* Reference direction of the geomagnetic field */
		hx = 2.0f * (m[0] * (q0q0 + q1q1 - 0.5) + m[1] *
				(q1q2 - q0q3) + m[2] * (q1q3 + q0q2));
		hy = 2.0f * (m[0] * (q1q2 + q0q3) + m[1] *
				(q0q0 + q2q2 - 0.5) + m[2] * (q2q3 - q0q1));
		bx = sqrt(hx * hx + hy * hy);
		bz = 2.0f * (m[0] * (q1q3 - q0q2) + m[1] *
				(q2q3 + q0q1) + m[2] * (0.5f - q1q1 - q2q2));
#endif
		bx = staticm[0];
		bz = staticm[1];

		/* Estimated direction of magnetic field */
		w[0] = bx * (q0q0 + q1q1 - 1.0f) + bz * (q1q3 - q0q2);
		w[1] = bx * (q1q2 - q0q3) + bz * (q2q3 + q0q1);
		w[2] = bx * (q1q3 + q0q2) + bz * (1.0f - q1q1 - q2q2);

		/* Error is sum of cross product between estimated
		 * direction and measured direction of field vectors */
		e[0] = m[1] * w[2] - m[2] * w[1];
		e[1] = m[2] * w[0] - m[0] * w[2];
		e[2] = m[0] * w[1] - m[1] * w[0];
	}

	len = hypot3f(acc);

	/* Compute feedback only if accelerometer measurement valid
	 * (avoids NaN in accelerometer normalisation), say: 0.75 - 1.25g
	 */
	acc_valid = (len > 0.60f / 0.0039f && len < 2.0f / 0.0039f);
	if (acc_valid) {
		float v[3], a[3];
		/* pay less attention to the accelerometer */
		///len <<= 0;

		/* Normalise accelerometer measurement */
		/* TODO: sure? */
		a[0] = acc[0] / len;
		a[1] = acc[1] / len;
		a[2] = acc[2] / len;

		/* Estimated gravity vector */
		v[0] = q0q2 - q1q3;
		v[1] = -q0q1 - q2q3;
		v[2] = q1q1 + q2q2 - 1.0f;

		/* Error is the cross product between estimated
		 * direction and measured direction of field vectors */
		e[0] += a[1] * v[2] - a[2] * v[1];
		e[1] += a[2] * v[0] - a[0] * v[2];
		e[2] += a[0] * v[1] - a[1] * v[0];
	}

	/* TODO: multiply g by (1 - Ki - Kp)? */
	now = timer_read();
	timediff = (float) (now - last_update) * (float) (1.0f / F_CPU);
	last_update = now;

	/* Compute and apply integral feedback if enabled */
	if (two_ki > 0.0f) {
		/* integral error scaled by Ki */
		integral_fb[0] += two_ki * e[0] * timediff;
		integral_fb[1] += two_ki * e[1] * timediff;
		integral_fb[2] += two_ki * e[2] * timediff;
		/* Apply integral feedback */
		g[0] += integral_fb[0];
		g[1] += integral_fb[1];
		g[2] += integral_fb[2];
	}

	/* Apply proportional feedback */
	g[0] += two_kp * e[0];
	g[1] += two_kp * e[1];
	g[2] += two_kp * e[2];

	/* Integrate rate of change of quaternion */
	qa = q[0];
	qb = q[1];
	qc = q[2];
#ifdef INTR
	cli();
#endif
	q[0] += (-qb * g[0] - qc * g[1] - q[3] * g[2]) * timediff * 0.5f;
	q[1] += (qa * g[0] + qc * g[2] - q[3] * g[1]) * timediff * 0.5f;
	q[2] += (qa * g[1] - qb * g[2] + q[3] * g[0]) * timediff * 0.5f;
	q[3] += (qa * g[2] + qb * g[1] - qc * g[0]) * timediff * 0.5f;

	if ((cnt ++ & 15) == 0) {
		/* Normalise the quaternion */
		recip_norm = inv_sqrt(q[0] * q[0] + q[1] * q[1] +
				q[2] * q[2] + q[3] * q[3]);
		q[0] *= recip_norm;
		q[1] *= recip_norm;
		q[2] *= recip_norm;
		q[3] *= recip_norm;
	}

	/* Auxiliary variables to avoid repeated arithmetics */
	q0q0 = 2.0f * q[0] * q[0];
	q0q1 = 2.0f * q[0] * q[1];
	q0q2 = 2.0f * q[0] * q[2];
	q0q3 = 2.0f * q[0] * q[3];
	q1q1 = 2.0f * q[1] * q[1];
	q1q2 = 2.0f * q[1] * q[2];
	q1q3 = 2.0f * q[1] * q[3];
	q2q2 = 2.0f * q[2] * q[2];
	q2q3 = 2.0f * q[2] * q[3];

#if 0
	/* Save the Euler angles defined with the Aerospace sequence.
	 * See Sebastian O.H. Madwick report:
	 * "An efficient orientation filter for inertial and intertial/magnetic
	 * sensor arrays" Chapter 2 Quaternion representation
	 */

	ahrs_psi = (uint16_t) (atan2(2 * q1q2 - 2 * q0q3,
			2 * q0q0 + 2 * q1q1 - 1) * 32768 / M_PI);
	ahrs_theta = (uint16_t) (-asin(2 * q1q3 + 2 * q0q2) *
			32768 / M_PI);
	ahrs_phi = (uint16_t) (atan2(2 * q2q3 - 2 * q0q1,
			1 - 2 * q1q1 - 2 * q2q2) * 32768 / M_PI);
#endif
	grav[0] = q1q3 - q0q2;
	grav[1] = q0q1 + q2q3;
	grav[2] = 1 - q1q1 - q2q2;

	ahrs_yaw = (uint16_t) (int) (atan2f(q1q2 - q0q3,
			q0q0 + q1q1 - 1) * (32768 / (float) M_PI));
	ahrs_pitch = (uint16_t) (int) (atanf(grav[0] / sqrtf(grav[1] * grav[1] +
				grav[2] * grav[2])) * (32768 / (float) M_PI));
	ahrs_roll = (uint16_t) (int) (atanf(grav[1] / sqrtf(grav[0] * grav[0] +
				grav[2] * grav[2])) * (32768 / (float) M_PI));
#if 0
	/* FIXME: overflows, naming */
	ahrs_yaw_rate = (int16_t) (g[0] * (256l * 180 / M_PI));
	ahrs_pitch_rate = (int16_t) (g[1] * (256l * 180 / M_PI));
	ahrs_roll_rate = (int16_t) (g[2] * (256l * 180 / M_PI));
#endif
	/* TODO: clamp? */
	//ahrs_roll_rate = ahrs_roll_rate * 0.75 + g[0] * (8l * 180 / M_PI);
	ahrs_roll_rate = g[0] * (float) (64l * 180 / M_PI);
	ahrs_pitch_rate = g[1] * (float) (64l * 180 / M_PI);
	ahrs_yaw_rate = g[2] * (float) (64l * 180 / M_PI);
#ifdef INTR
	sei();
#endif
}

/* Try to set a quaternion that aligns "a" with gravity ([0, 0, -1]),
 * and "m" with [staticm[0], 0, staticm[1]].
 * Note that staticm is not set at this point... */
static void quaternion_reset(float *a, float *m) {
	float halfangle, axislen, q0[3], q1[4], mref[2];

	/* TODO: we could avoid using any trig functions here, but I'm not
	 * sure whether it would make the code faster / simpler at all */
	axislen = sqrtf(a[0] * a[0] + a[1] * a[1]);
	halfangle = 0.5f * atan2f(axislen, -a[2]);
	q0[0] = cosf(halfangle);
	q0[1] = -a[1] * sinf(halfangle) / axislen;
	q0[2] = a[0] * sinf(halfangle) / axislen;
	/* Could be rewritten as: (?)
	q0[0] = sqrtf(1 - a[2] / full_len / 2)
	q0[1] = -a[1] * sqrtf(1 + a[2] / full_len / 2) / axixlen;
	q0[2] = a[0] * sqrtf(1 + a[2] / full_len / 2) / axixlen;
	with signs corrected as needed
	*/
	q0q0 = q0[0] * q0[0];
	q0q1 = q0[0] * q0[1];
	q0q2 = q0[0] * q0[2];
	q1q1 = q0[1] * q0[1];
	q1q2 = q0[1] * q0[2];
	q2q2 = q0[2] * q0[2];

	mref[0] = m[0] * (q0q0 + q1q1 - 0.5f) + m[1] * q1q2 + m[2] * q0q2;
	mref[1] = m[0] * q1q2 + m[1] * (q0q0 + q2q2 - 0.5f) - m[2] * q0q1;

	/* Might not work at the magnetic poles..., oh well. */
	halfangle = 0.5f * atan2f(-mref[1], mref[0]);
	q1[0] = cosf(halfangle);
	q1[1] = -sinf(halfangle) * 2.0f * q0q2;
	q1[2] = sinf(halfangle) * 2.0f * q0q1;
	q1[3] = sinf(halfangle) * 2.0f * (0.5f - q1q1 - q2q2);

	q[0] = q0[0] * q1[0] - q0[1] * q1[1] - q0[2] * q1[2];
	q[1] = q0[0] * q1[1] + q0[1] * q1[0] + q0[2] * q1[3];
	q[2] = q0[0] * q1[2] - q0[1] * q1[3] + q0[2] * q1[0];
	q[3] = q0[0] * q1[3] + q0[1] * q1[2] - q0[2] * q1[1];

	q0q0 = 2.0f * q[0] * q[0];
	q0q1 = 2.0f * q[0] * q[1];
	q0q2 = 2.0f * q[0] * q[2];
	q0q3 = 2.0f * q[0] * q[3];
	q1q1 = 2.0f * q[1] * q[1];
	q1q2 = 2.0f * q[1] * q[2];
	q1q3 = 2.0f * q[1] * q[3];
	q2q2 = 2.0f * q[2] * q[2];
	q2q3 = 2.0f * q[2] * q[3];
}

void ahrs_init(void) {
	int i;
	///uint16_t c[3], gscale[3];
	int16_t c[3];///
	int16_t a[3], m[3];
	int32_t avgg[3];
	float avgm[3], avga[3];
	///float invlen, axm[3];

	/* Calibrate the sensors while waiting for the ESCs to detect
	 * voltages etc. */
	avga[0] = avga[1] = avga[2] = 0;
	avgm[0] = avgm[1] = avgm[2] = 0;
	avgg[0] = avgg[1] = avgg[2] = 0;

	for (i = 0; i < 2048; i ++) {
#if 0
		wmp_read(c, gscale);
		avgg[0] += (float) c[0] - 0x2000;
		avgg[1] += (float) c[1] - 0x2000;
		avgg[2] += (float) c[2] - 0x2000;
		if (gscale[0] > 2000 || gscale[1] > 2000 || gscale[2] > 2000) {
			serial_write_str("Gyro fast-mode during calibration");
			while (1);
		}
#endif
		l3g4200d_read(c);
		avgg[0] += c[0];
		avgg[1] += c[1];
		avgg[2] += c[2];

		my_delay(4);
		if (!(i & 7)) { /* Every 30ms or so */
			adxl345_read(a);
			hmc5883l_read(m);
			avgm[0] += m[0];
			avgm[1] += m[1];
			avgm[2] += m[2];
			avga[0] -= (float) a[0] * ACCEL_X_SCALE;
			avga[1] -= (float) a[1] * ACCEL_Y_SCALE;
			avga[2] -= (float) a[2] * ACCEL_Z_SCALE;
		}
	}
	staticg[0] = avgg[0] * (1 / 2048.0f);
	staticg[1] = avgg[1] * (1 / 2048.0f);
	staticg[2] = avgg[2] * (1 / 2048.0f);

	/* "Level" and "northward" are currently defined as in the plane
	 * of the sensor board and wherever the magnetic field vector
	 * is pointing related to the average gravity vector.  So
	 * the boot-up attitude doesn't really matter, but the sensor
	 * board mounting does.
	 *
	 * The other options would be: define "neutral" (level, northward)
	 * orientation in relation to the initial/boot-up averaged
	 * magnetic and gravity vectors, or in relation to the sensor
	 * board but rotated by some reconfigured absolute angles.
	 *
	 * The code below leaves the reference "gravity" vector intact,
	 * but sets the reference magnetic vector in relation to the
	 * gravity vector, although ignoring its oriignal direction and
	 * only setting the angle from vertical.
	 */

	mag[0] = avgm[0] * (1.0f / 256.0f);
	mag[1] = avgm[1] * (1.0f / 256.0f);
	mag[2] = avgm[2] * (1.0f / 256.0f);
	mag_calib_update(); /* Subtract calibrated offset if available */

	acc[0] = avga[0] * (1.0f / 256.0f);
	acc[1] = avga[1] * (1.0f / 256.0f);
	acc[2] = avga[2] * (1.0f / 256.0f);
	avgalen = hypot3f(acc);
#if 0
	axm[0] = avgm[1] * avga[2] - avgm[2] * avga[1];
	axm[1] = avgm[2] * avga[0] - avgm[0] * avga[2];
	axm[2] = avgm[0] * avga[1] - avgm[1] * avga[0];
	staticm[0] = axm[0] * axm[0] + axm[1] * axm[1] + axm[2] * axm[2];
	staticm[1] = avgm[0] * avga[0] + avgm[1] * avga[1] + avgm[2] * avga[2];
	invlen = inv_sqrt(staticm[0] + staticm[1] * staticm[1]);
	/* TODO: use ccsqrt */
	staticm[0] = sqrtf(staticm[0]) * invlen;
	staticm[1] *= -invlen;
#endif

	/* TODO: check that the angle between the two vectors is
	 * in a sensible range (limits may need to be set per user
	 * lattitude? could also make use of the GPS?) */

	/* set initial pos */
	quaternion_reset(acc, mag);

	integral_fb[0] = integral_fb[1] = integral_fb[2] = 0.0f;

	/* Start updating the correcting vectors first */
	last_update = timer_read();
#ifdef INTR
	vectors_update();
#endif
}

#ifndef INTR
void ahrs_update(void) {
	vectors_update();
}
#endif
