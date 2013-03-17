/*
 * Wii MotionPlus i2c driver (IDG650 gyro + some other gyro)
 *
 * Licensed under AGPLv3.
 */

static inline void wmp_read(uint16_t *g, uint16_t *scale) {
	uint8_t data[6];

#ifdef I2C_RETRY
restart:
#endif
	if (unlikely(!i2c_send_byte(0x52, 0x00))) {
		serial_write1('R');
		serial_write1('S');
#ifdef I2C_RETRY
		goto restart;
#endif
	}

	if (unlikely(!i2c_request_bytes(0x52, sizeof(data), data))) {
		serial_write1('R');
		serial_write1('R');
#ifdef I2C_RETRY
		goto restart;
#endif
	}
	if (unlikely((data[5] & 3) != 2)) {
		serial_write1('R');
		serial_write1('E');
#ifdef I2C_RETRY
		goto restart;
#endif
	}

	g[0] = ((uint16_t) (data[4] & 0xfc) << 6) | data[1];
	g[1] = ((uint16_t) (data[5] & 0xfc) << 6) | data[2];
	g[2] = ((uint16_t) (data[3] & 0xfc) << 6) | data[0];
	scale[0] = scale[1] = scale[2] = 1000;
	if (!(data[4] & 2))
		/*g[0] = (g[0] << 2) + (g[0] >> 1); Approx :-( */
		/* g[0] = ((uint32_t) g[0] * 4655) >> 10; Also approx :/ */
		scale[0] = 4545;
	if (!(data[3] & 1))
		/* g[1] = (g[1] << 2) + (g[1] >> 1); Approx :-( */
		scale[1] = 4545;
	if (!(data[3] & 2))
		/* g[2] = (g[2] << 2) + (g[2] >> 1); Approx :-( */
		scale[2] = 4545;
}

static inline void wmp_on(void) {
	uint8_t status = 0, n;
	uint16_t g[3];

	/* Initialise */
	if (unlikely(!i2c_send_bytes(0x53, 2, (uint8_t[]) { 0xf0, 0x55 })))
		serial_write1('z');
	while (status != 0xe) {
		if (unlikely(!i2c_send_byte(0x53, 0xf7))) {
			serial_write1('x');
			break;
		}
		if (unlikely(!i2c_request_bytes(0x53, 1, &n)))
			serial_write1('y');
		if (status != n) {
			status = n;
			serial_write_hex16((uint16_t) status);
		}
		my_delay(1);
	}

	/* Activate */
#ifdef I2C_RETRY
restart:
#endif
	if (unlikely(!i2c_send_bytes(0x53, 2, (uint8_t[]) { 0xfe, 0x04 }))) {
		serial_write1('I');
		serial_write1('S');
#ifdef I2C_RETRY
		goto restart;
#endif
	}

	/* Throw away the first couple of readings */
	my_delay(30); wmp_read(g, g); wmp_read(g, g);
	my_delay(30); wmp_read(g, g); wmp_read(g, g);
}

/* Turn off and enable passthrough? */
static inline void wmp_off(void) {
#ifdef I2C_RETRY
restart:
#endif
	if (unlikely(!i2c_send_bytes(0x52, 2, (uint8_t[]) { 0xf0, 0x55 }))) {
		serial_write1('U');
		serial_write1('S');
#ifdef I2C_RETRY
		goto restart;
#endif
	}
}
