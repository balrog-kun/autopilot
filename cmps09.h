/*
 * CMPS09 magnetometer + accelerometer board driver.
 *
 * Licensed under AGPLv3.
 */

#define CMPS09_ADDR 0x60

static inline void cmps09_read_bytes(uint8_t from,
		uint8_t count, uint8_t *out) {
#ifdef I2C_RETRY
restart:
#endif
	/* Send start register number */
	if (unlikely(!i2c_send_byte(CMPS09_ADDR, from))) {
		serial_write1('C');
		serial_write1('S');
#ifdef I2C_RETRY
		goto restart;
#endif
	}

	if (!i2c_request_bytes(CMPS09_ADDR, count, out)) {
		serial_write1('C');
		serial_write1('R');
#ifdef I2C_RETRY
		goto restart;
#endif
	}
}

/* Returned values units (25C):
 *   for acceleration at nominal temperature: 1/16384 g
 *   for magnetic field magnitude at nominal temp: 1/1024 gauss (922-1126)
 * sensitivity change with temperature: +/- 1100 ppm/C :-(
 * bandwidth: ~18Hz
 */
static inline void cmps09_read(int16_t *a, int16_t *m) {
	uint8_t regs[12];

	cmps09_read_bytes(10, 12, regs);

	m[0] = ((uint16_t) regs[2] << 8) | regs[3];
	m[1] = ((uint16_t) regs[0] << 8) | regs[1];
	m[2] = ((uint16_t) regs[4] << 8) | regs[5];
	a[0] = ((uint16_t) regs[6] << 8) | regs[7];
	a[1] = ((uint16_t) regs[8] << 8) | regs[9];
	a[2] = ((uint16_t) regs[10] << 8) | regs[11];
}

/* This has been determined by averaging the magnetometer readings over a
 * period when the CMPS09 PCB has been rotated at a constant velocity in
 * such a way that it should (optimally) visit all the possible orientations
 * each for the same amount of time.  An easier alternative is to rotate the
 * PCB at any rate and only to visit the extreme orientations, and then
 * take the average of the minimum and maximum for each axis.  An even
 * easier alternative would be to take a known-working, calibrated
 * magnetometer and simply take the difference between its readings and
 * those of our IMU sensors.
 *
 * This data is specific to the physical hardware configuration of the
 * aircraft, it unfortunately depends on the distribution of ferrous
 * material around the device.  If there is little ferrous material then
 * the values will be low and even without any calibration ({ 0, 0, 0 }),
 * everything should be fine.
 *
 * The calibration data should be fairly independent of the place of
 * calibration, I think.  But it'll need to be udpated every time the
 * shape or the internal construction of the aircraft changes.
 */
#ifndef CAL
static const int16_t cmps09_mag_calib[3] = { -120, -35, -30 };
#else
static const int16_t cmps09_mag_calib[3] = { 0, 0, 0 };
#endif

/*
 * These two angles describe the CMPS09's axis orientation compared to
 * the vehicle's orientation.  We use this as the reference level to
 * avoid having to always start on a level surface.  If these angles
 * are slightly off because the CMPS09 was moved a little, the vehicle's
 * idea of level will be wrong.
 */
static const uint16_t cmps09_x_axis_cos = 65235;
static const uint16_t cmps09_x_axis_sin = 6273;
static const uint16_t cmps09_y_axis_cos = 65447;
static const uint16_t cmps09_y_axis_sin = 3415;

static inline void cmps09_xy_adjust(int16_t v[3]) {
	int16_t z;

	z = ((int32_t) v[2] * cmps09_y_axis_cos +
			(int32_t) v[0] * cmps09_y_axis_sin) >> 16;
	v[0] = ((int32_t) v[0] * cmps09_y_axis_cos -
			(int32_t) v[2] * cmps09_y_axis_sin) >> 16;

	v[2] = ((int32_t) z * cmps09_x_axis_cos +
			(int32_t) v[1] * cmps09_x_axis_sin) >> 16;
	v[1] = ((int32_t) v[1] * cmps09_x_axis_cos -
			(int32_t) z * cmps09_x_axis_sin) >> 16;
}
