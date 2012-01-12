/*
 * An AHRS based on the Extended Kalman Filter.
 * Some calculations are based on the FreeIMU library by Fabio Varesano whose
 * development was supported by Universita' degli Studi di Torino within
 * the Piemonte Project.
 *
 * Licensed under AGPLv3.
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <math.h>

#include "twi.h"
#include "timer1.h"
#include "uart.h"
#include "cmps09.h"
#include "wmp.h"
#include "isqrt.h"
#include "ahrs.h"
#include "trig.h"

volatile int16_t ahrs_pitch, ahrs_roll, ahrs_yaw;
volatile int16_t ahrs_pitch_rate, ahrs_roll_rate, ahrs_yaw_rate;

/* Acceleration is reported in the initial coordinate system rotated
 * by the angles above so really it is the local coordinate system
 * of the vehicle.  It should not include the gravitational
 * acceleration.
 */
volatile int16_t accel_acceleration[3];

volatile float q[4];
volatile float mag[3], acc[3];
static float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3;
static const float two_ki = 1.0 * 0.000; /* 1 x integral gain */
static const float two_kp = 1.0 * 0.2; /* 1 x proportional gain */
static float integral_fb[3];
static uint32_t last_update;

static float staticg[3]; /* Zero-rate gyro reading.. believed to be... */
static float staticm[2]; /* Initial Mag readings average */

/* Quake inverse square root */
static float inv_sqrt(float number) {
	uint32_t i = 0x5f375a86 - ((*(uint32_t *) &number) >> 1);
	return (*(float *) &i) *
		(1.5f - (number * 0.5f * (*(float *) &i) * (*(float *) &i)));
}

/* Quaternion implementation of the 'DCM filter' [Mayhony et al].
 * Incorporates the magnetic distortion compensation algorithms from
 * Sebastian Madgwick filter which eliminates the need for a reference
 * direction of flux (bx bz) to be predefined and limits the effect of
 * magnetic distortions to yaw axis only.
 *
 * See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
 */
static void vectors_update(void) {
	float recip_norm;
	float e[3] = { 0.0f, 0.0f, 0.0f };
	float qa, qb, qc;
	float timediff, grav[3], g[3];
	uint16_t graw[3], gscale[3];
	int16_t araw[3], mraw[3];
	uint16_t len;
	uint32_t now;
	static uint8_t cnt = 0;

	/* The gyro's bandwidth is 140Hz so schedule the next measurement
	 * 1/140 sec from this measurement.  Don't try to compensate if the
	 * timer was late this time because if the timer triggers sooner
	 * than 1/140 sec next time, the measurements won't be independent.  */
	/* ... */
	/* The magnetometers's measurement frequency is 50Hz and the
	 * accelerometer's rate is limite by buffer referesh rate of 55Hz, so
	 * schedule the next measurement 1/50 sec from the time the previous
	 * measurement was *supposed* to happen.  Not sure if that's really
	 * better, need to think about it.  */
	set_timeout(timer_read() + F_CPU / 30, vectors_update);

	/* Retrieve current values of everything */
	wmp_read(graw, gscale); /* TODO: average over 2+ readings? */
	cmps09_read(araw, mraw);
	mraw[0] -= cmps09_mag_calib[0];
	mraw[1] -= cmps09_mag_calib[1];
	mraw[2] -= cmps09_mag_calib[2];

#if 1
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

	mag[0] = -(float) mraw[0] / 512;
	mag[1] = (float) mraw[1] / 512;
	mag[2] = -(float) mraw[2] / 512;
	acc[0] = (float) araw[0] / 0x4000;
	acc[1] = -(float) araw[1] / 0x4000;
	acc[2] = (float) araw[2] / 0x4000;

	len = isqrt32(hypot3(mraw));

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
	if (len > 256 && len < 666) {
		float bx, bz;
		float w[3], m[3];

		/* Normalise magnetometer measurement */
		m[0] = -(float) mraw[0] / len;
		m[1] = (float) mraw[1] / len;
		m[2] = -(float) mraw[2] / len;

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
		w[0] = bx * (q0q0 + q1q1 - 1) + bz * (q1q3 - q0q2);
		w[1] = bx * (q1q2 - q0q3) + bz * (q2q3 + q0q1);
		w[2] = bx * (q1q3 + q0q2) + bz * (1 - q1q1 - q2q2);

		/* Error is sum of cross product between estimated
		 * direction and measured direction of field vectors */
		e[0] = m[1] * w[2] - m[2] * w[1];
		e[1] = m[2] * w[0] - m[0] * w[2];
		e[2] = m[0] * w[1] - m[1] * w[0];
	}

	len = isqrt32(hypot3(araw));

	/* Compute feedback only if accelerometer measurement valid
	 * (avoids NaN in accelerometer normalisation), say: 0.5 - 1.5g
	 */
	if (len > 0x2000 && len < 0x6000) {
		float v[3], a[3];
		/* pay less attention to the accelerometer */
		uint32_t alen = (uint32_t) len << 1;

		/* Normalise accelerometer measurement */
		/// sure?
		a[0] = (float) araw[0] / alen;
		a[1] = -(float) araw[1] / alen;
		a[2] = (float) araw[2] / alen;

		/* Estimated gravity vector */
		v[0] = q0q2 - q1q3;
		v[1] = -q0q1 - q2q3;
		v[2] = q1q1 + q2q2 - 1;

		/* Error is sum of cross product between estimated
		 * direction and measured direction of field vectors */
		e[0] += a[1] * v[2] - a[2] * v[1];
		e[1] += a[2] * v[0] - a[0] * v[2];
		e[2] += a[0] * v[1] - a[1] * v[0];
	}

	/* TODO: multiply g by (1 - Ki - Kp)? */
	now = timer_read();
	timediff = (float) (now - last_update) / F_CPU;
	last_update = now;

	/* Compute and apply integral feedback if enabled */
	if (two_ki > 0.0f) {
		/* integral error scaled by Ki */
		integral_fb[0] += two_ki * e[0] * timediff;
		integral_fb[1] += two_ki * e[1] * timediff;
		integral_fb[2] += two_ki * e[2] * timediff;
		/* apply integral feedback */
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
	cli();
	q[0] += (-qb * g[0] - qc * g[1] - q[3] * g[2]) * timediff * 0.5;
	q[1] += (qa * g[0] + qc * g[2] - q[3] * g[1]) * timediff * 0.5;
	q[2] += (qa * g[1] - qb * g[2] + q[3] * g[0]) * timediff * 0.5;
	q[3] += (qa * g[2] + qb * g[1] - qc * g[0]) * timediff * 0.5;

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
	q0q0 = 2 * q[0] * q[0];
	q0q1 = 2 * q[0] * q[1];
	q0q2 = 2 * q[0] * q[2];
	q0q3 = 2 * q[0] * q[3];
	q1q1 = 2 * q[1] * q[1];
	q1q2 = 2 * q[1] * q[2];
	q1q3 = 2 * q[1] * q[3];
	q2q2 = 2 * q[2] * q[2];
	q2q3 = 2 * q[2] * q[3];

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
#if 0
	grav[0] = q1q3 - q0q2;
	grav[1] = q0q1 + q2q3;
	grav[2] = 1 - q1q1 - q2q2;

	ahrs_yaw = (uint16_t) (atan2(q1q2 - q0q3,
			q0q0 + q1q1 - 1) * (32768 / M_PI));
	ahrs_pitch = (uint16_t) (atan(grav[0] / sqrt(grav[1] * grav[1] +
				grav[2] * grav[2])) * (32768 / M_PI));
	ahrs_roll = (uint16_t) (atan(grav[1] / sqrt(grav[0] *grav[0] +
				grav[2] * grav[2])) * (32768 / M_PI));
	/* FIXME: overflows, naming */
	ahrs_yaw_rate = (int16_t) (g[0] * (256l * 180 / M_PI));
	ahrs_pitch_rate = (int16_t) (g[1] * (256l * 180 / M_PI));
	ahrs_roll_rate = (int16_t) (g[2] * (256l * 180 / M_PI));
#endif
	sei();
}

/* Try to set a quaternion that aligns "a" with gravity ([0, 0, -1]),
 * and "m" with [staticm[0], 0, staticm[1]] */
static void quaternion_reset(float *a, float *m) {
	float halfangle, axislen, q0[3], q1[4], mref[2];

	/* TODO: we could avoid using any trig functions here, but I'm not
	 * sure whether it would make the code faster / simpler at all */
	axislen = sqrt(a[0] * a[0] + a[1] * a[1]);
	halfangle = 0.5 * atan2(axislen, -a[2]);
	q0[0] = cos(halfangle);
	q0[1] = -a[1] * sin(halfangle) / axislen;
	q0[2] = a[0] * sin(halfangle) / axislen;
	q0q0 = q0[0] * q0[0];
	q0q1 = q0[0] * q0[1];
	q0q2 = q0[0] * q0[2];
	q1q1 = q0[1] * q0[1];
	q1q2 = q0[1] * q0[2];
	q2q2 = q0[2] * q0[2];

	mref[0] = m[0] * (q0q0 + q1q1 - 0.5) + m[1] * q1q2 + m[2] * q0q2;
	mref[1] = m[0] * q1q2 + m[1] * (q0q0 + q2q2 - 0.5) - m[2] * q0q1;

	/* Might not work at the magnetic poles..., oh well. */
	halfangle = 0.5 * atan2(-mref[1], mref[0]);
	q1[0] = cos(halfangle);
	q1[1] = -sin(halfangle) * 2 * q0q2;
	q1[2] = sin(halfangle) * 2 * q0q1;
	q1[3] = sin(halfangle) * 2 * (0.5 - q1q1 - q2q2);

	q[0] = q0[0] * q1[0] - q0[1] * q1[1] - q0[2] * q1[2];
	q[1] = q0[0] * q1[1] + q0[1] * q1[0] + q0[2] * q1[3];
	q[2] = q0[0] * q1[2] - q0[1] * q1[3] + q0[2] * q1[0];
	q[3] = q0[0] * q1[3] + q0[1] * q1[2] - q0[2] * q1[1];

	q0q0 = 2 * q[0] * q[0];
	q0q1 = 2 * q[0] * q[1];
	q0q2 = 2 * q[0] * q[2];
	q0q3 = 2 * q[0] * q[3];
	q1q1 = 2 * q[1] * q[1];
	q1q2 = 2 * q[1] * q[2];
	q1q3 = 2 * q[1] * q[3];
	q2q2 = 2 * q[2] * q[2];
	q2q3 = 2 * q[2] * q[3];
}

void ahrs_init(void) {
	int i;
	uint16_t c[3], gscale[3];
	int16_t a[3], m[3];
	float avga[3], avgm[3], avgg[3];
	float invlen, axm[3];

	/* Calibrate the sensors while waiting for the ESCs to detect
	 * voltages etc. */
	avga[0] = avga[1] = avga[2] = 0;
	avgm[0] = avgm[1] = avgm[2] = 0;
	avgg[0] = avgg[1] = avgg[2] = 0;

	for (i = 0; i < 1024; i ++) {
		wmp_read(c, gscale);
		avgg[0] += (float) c[0] - 0x2000;
		avgg[1] += (float) c[1] - 0x2000;
		avgg[2] += (float) c[2] - 0x2000;
		if (gscale[0] > 2000 || gscale[1] > 2000 || gscale[2] > 2000) {
			serial_write_str("Gyro fast-mode during calibration");
			while (1);
		}

		my_delay(4);
		if (!(i & 7)) { /* Every 30ms or so */
			cmps09_read(a, m);
			avgm[0] += m[0] - cmps09_mag_calib[0];
			avgm[1] += m[1] - cmps09_mag_calib[1];
			avgm[2] += m[2] - cmps09_mag_calib[2];
			avga[0] += a[0];
			avga[1] += a[1];
			avga[2] += a[2];
		}
	}
	staticg[0] = avgg[0] / 1024;
	staticg[1] = avgg[1] / 1024;
	staticg[2] = avgg[2] / 1024;

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
	avgm[0] *= -0.0001;
	avgm[1] *= 0.0001;
	avgm[2] *= -0.0001;
	avga[0] *= 0.0001;
	avga[1] *= -0.0001;
	avga[2] *= 0.0001;
	axm[0] = avgm[1] * avga[2] - avgm[2] * avga[1];
	axm[1] = avgm[2] * avga[0] - avgm[0] * avga[2];
	axm[2] = avgm[0] * avga[1] - avgm[1] * avga[0];
	staticm[0] = sqrt(axm[0] * axm[0] + axm[1] * axm[1] + axm[2] * axm[2]);
	staticm[1] = avgm[0] * avga[0] + avgm[1] * avga[1] + avgm[2] * avga[2];
	invlen = inv_sqrt(staticm[0] * staticm[0] + staticm[1] * staticm[1]);
	staticm[0] *= invlen;
	staticm[1] *= -invlen;

	/* TODO: check that the angle between the two vectors is
	 * in a sensible range (limits may need to be set per user
	 * lattitude? could also make use of the GPS?) */

	/* set initial pos */
	quaternion_reset(avga, avgm);

	integral_fb[0] = integral_fb[1] = integral_fb[2] = 0.0f;

	/* Start updating the correcting vectors first */
	last_update = timer_read();
	vectors_update();
}
