/*
 * A simple fixed-point AHRS based on intertial and magentic sensors.
 *
 * Licensed under AGPLv3.
 */

void ahrs_init(void);
#ifndef INTR
void ahrs_update(void);////
#endif

#ifdef INTR
# define VOL volatile
#else
# define VOL
#endif

#define ROLL_PITCH_180DEG ((int32_t) (0.9765625 * F_CPU * 90))////
extern VOL int16_t ahrs_pitch, ahrs_roll;
extern VOL int16_t ahrs_yaw,
	ahrs_pitch_rate, ahrs_roll_rate, ahrs_yaw_rate;

/* Other non-ahrs data while we're there */
extern VOL int16_t accel_acceleration[3];
extern VOL float mag[3];
extern VOL float acc[3];

/* The quaternion */
extern VOL float q[4];
extern VOL float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3;

static inline void local_to_global(const float *l, float *g) {
	g[0] = l[0] * (q0q0 + q1q1 - 1) +
		l[1] * (q1q2 - q0q3) + l[2] * (q1q3 + q0q2);
	g[1] = l[0] * (q1q2 + q0q3) +
		l[1] * (q0q0 + q2q2 - 1) + l[2] * (q2q3 - q0q1);
	g[2] = l[0] * (q1q3 - q0q2) +
		l[1] * (q2q3 + q0q1) + l[2] * (1 - q1q1 - q2q2);
}

static inline void global_to_local(const float *g, float *l) {
	l[0] = g[0] * (q0q0 + q1q1 - 1) +
		g[1] * (q1q2 + q0q3) + g[2] * (q1q3 - q0q2);
	l[1] = g[0] * (q1q2 - q0q3) +
		g[1] * (q0q0 + q2q2 - 1) + g[2] * (q2q3 + q0q1);
	l[2] = g[0] * (q1q3 + q0q2) +
		g[1] * (q2q3 - q0q1) + g[2] * (1 - q1q1 - q2q2);
}
