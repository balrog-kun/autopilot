#define ADXL345_ADDR 0x53 /* Same as Wii Motion+ :-( */

static inline void adxl345_read(int16_t *a) {
	uint8_t regs[6];

	/* Send start register number + auto-increment bit */
	if (unlikely(!i2c_send_byte(ADXL345_ADDR, 0x32))) {
		serial_write1('A');
		serial_write1('S');
	}

	if (unlikely(!i2c_request_bytes(ADXL345_ADDR, sizeof(regs), regs))) {
		serial_write1('A');
		serial_write1('R');
	}

	a[0] = ((uint16_t) regs[1] << 8) | regs[0];
	a[1] = ((uint16_t) regs[3] << 8) | regs[2];
	a[2] = ((uint16_t) regs[5] << 8) | regs[4];
}

static inline void adxl345_init(void) {
	/* Write BW_RATE: Enable normal operation, set 200Hz output data rate */
	/* Write POWER_CTL: Enable measurement mode */
	if (unlikely(!i2c_send_bytes(ADXL345_ADDR, 3,
					(uint8_t[]) { 0x2c, 0x0b, 0x0b })))
		serial_write1('A');

	/* Write DATA_FORMAT: LSB justified, full +/-16g range */
	if (unlikely(!i2c_send_bytes(ADXL345_ADDR, 2,
					(uint8_t[]) { 0x31, 0x0b })))
		serial_write1('A');

	/* All other registers seem to have just the right defaults */
}
