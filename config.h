/*
 * A simple LPC1343-based multicopter flight controller.
 *
 * Licensed under AGPLv3.
 */

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
	CFG_MAG_CALIB_X,
	CFG_MAG_CALIB_Y,
	CFG_MAG_CALIB_Z,
	__CFG_END,
};

extern uint32_t config[16];

#define neutral_x ((int32_t) config[CFG_NEUTRAL_X])
#define neutral_y ((int32_t) config[CFG_NEUTRAL_Y])

#define mag_calib_x (*(float *) &config[CFG_MAG_CALIB_X])
#define mag_calib_y (*(float *) &config[CFG_MAG_CALIB_Y])
#define mag_calib_z (*(float *) &config[CFG_MAG_CALIB_Z])
