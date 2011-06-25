/*
 * CMPS09 magnetometer + accelerometer board driver.
 */

#define CMPS09_ADDR 0x60

static inline void cmps09_read_bytes(uint8_t from,
		uint8_t count, uint8_t *out) {
	i2c_send_byte(CMPS09_ADDR, from); /* Send start register number */

	i2c_request_bytes(CMPS09_ADDR, count, out);
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
static const int16_t cmps09_mag_calib[3] = { -135, -20, -90 };
#else
static const int16_t cmps09_mag_calib[3] = { 0, 0, 0 };
#endif
