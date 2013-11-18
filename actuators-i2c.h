/*
 * Generates the signal that drives RC actuators (motors and servos).
 *
 * Licensed under AGPLv3.
 */

volatile uint16_t actuators_i2c[4];

#define ESC_ADDR(n)	(0x29 + n)

static inline void i2c_act_raw_write(int target, uint16_t val) {
	/* Double GCC'ism */
	if (unlikely(!i2c_send_bytes(ESC_ADDR(target), 3,
					(uint8_t[]) { 0x00, val >> 8, val })))
		serial_write1('B');
}

static inline int i2c_act_raw_read(int target, uint8_t *buf) {
	if (unlikely(!i2c_send_byte(ESC_ADDR(target), 0x02))) {
		serial_write1('B');
		serial_write1('S');
		return 0;
	}

	if (unlikely(!i2c_request_bytes(ESC_ADDR(target), 7, buf))) {
		serial_write1('B');
		serial_write1('R');
		return 0;
	}

	return 1;
}

static void die(void);
void actuators_i2c_init(int devices) {
	uint8_t vals[4], buf[7];

	/* Wait for the ESCs to start main loop */
	my_delay(100);

	buf[6] = 0x00;
	i2c_act_raw_read(0, buf);
	vals[0] = buf[6];
	buf[6] = 0x00;
	i2c_act_raw_read(1, buf);
	vals[1] = buf[6];
	buf[6] = 0x00;
	i2c_act_raw_read(2, buf);
	vals[2] = buf[6];
	buf[6] = 0x00;
	i2c_act_raw_read(3, buf);
	vals[3] = buf[6];

	serial_write_hex16(vals[0]);
	serial_write_hex16(vals[1]);
	serial_write_hex16(vals[2]);
	serial_write_hex16(vals[3]);
	if (vals[0] == 0xab)
		serial_write_str("ESC 0 is alive\r\n");
	else
		serial_write_str("Can't contact ESC 0!!!\r\n");
	if (vals[1] == 0xab)
		serial_write_str("ESC 1 is alive\r\n");
	else
		serial_write_str("Can't contact ESC 1!!!\r\n");
	if (vals[2] == 0xab)
		serial_write_str("ESC 2 is alive\r\n");
	else
		serial_write_str("Can't contact ESC 2!!!\r\n");
	if (vals[3] == 0xab)
		serial_write_str("ESC 3 is alive\r\n");
	else
		serial_write_str("Can't contact ESC 3!!!\r\n");
	//// die() if not all alive or not all 0x00 (no battery operation).
	//// perhaps do some sort of ADC voltage check or to see if we really
	//// have no battery, or read VBUS somehow to see if we're powered
	//// from USB.
#if 1
	if (vals[0] != 0xab || vals[1] != 0xab ||
			vals[2] != 0xab || vals[3] != 0xab)
		die();
#endif
}

/* TODO: currently we don't continually send signals if the main loop
 * doesn't call us periodically and we could potentially cause a timeout
 * in the ESCs.  Normally the main loop will call us quite often though.  */
void actuators_i2c_start(void) {
}
void actuators_i2c_stop(void) {
}

static inline void actuator_i2c_set(uint8_t target, uint16_t value) {
	uint32_t now = timer_read();
	static uint32_t actuators_i2c_ts[4];

	if ((actuators_i2c[target] >> 2) == (value >> 2) &&
			now - actuators_i2c_ts[target] < F_CPU)
		return;

	actuators_i2c_ts[target] = now;
	i2c_act_raw_write(target, (value >> 1) |
			((REVERSE_MASK << target) & 0x8000));
	actuators_i2c[target] = value;
}

static inline int actuator_i2c_get_vals(uint8_t target,
		uint16_t *rpm, uint16_t *vbat, uint16_t *temp) {
	uint8_t buf[7];

	if (!i2c_act_raw_read(target, buf))
		return 0;

	*rpm = (buf[0] << 8) | buf[1];
	*vbat = (buf[2] << 8) | buf[3];
	*temp = (buf[4] << 8) | buf[5];

	return 1;
}
