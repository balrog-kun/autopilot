/*
 * A simple fixed-point AHRS based on intertial and magentic sensors.
 *
 * Licensed under AGPLv3.
 */

void ahrs_init(void);

#define ROLL_PITCH_180DEG ((int32_t) (0.9765625 * F_CPU * 90))
/* TODO: reduce to 16 bits for consumers */
extern volatile int32_t ahrs_pitch, ahrs_roll;
extern volatile int16_t ahrs_yaw,
       ahrs_pitch_rate, ahrs_roll_rate, ahrs_yaw_rate;

/* Other non-ahrs data while we're there */
extern volatile int16_t accel_acceleration[3];
extern volatile int32_t accel_velocity[3];
