#define HMC5883L_ADDR 0x1e

static inline void hmc5883l_read(int16_t *a) {
	uint8_t regs[6];

	/* Seems there's no need to send the register address to this chip,
	 * clever */
	if (unlikely(!i2c_send_byte(HMC5883L_ADDR, 0x03))) {
		serial_write1('M');
		serial_write1('S');
	}

	if (unlikely(!i2c_request_bytes(HMC5883L_ADDR, sizeof(regs), regs))) {
		serial_write1('M');
		serial_write1('R');
	}

	a[0] = ((uint16_t) regs[0] << 8) | regs[1];
	a[1] = ((uint16_t) regs[4] << 8) | regs[5];
	a[2] = ((uint16_t) regs[2] << 8) | regs[3];
	/* -4096 when any value overflows, would be better the sign was
	 * maintained, but we can live with it */
}

static inline void hmc5883l_init(void) {
	/* Write CRA, CRB and MR: continuous mode at max frequency, averaging */
	if (unlikely(!i2c_send_bytes(HMC5883L_ADDR, 4,
					(uint8_t[]) { 0x00, 0x78, 0x20, 0 })))
		serial_write1('M');
}
