#define L3G4200D_ADDR 0x69

static inline void l3g4200d_read(int16_t *g) {
	uint8_t regs[6];

	/* TODO: we might also want to read the status register at addr 27
	 * in the same block read to know whether what we're seeing is new
	 * data.  */

	/* Send start register number + auto-increment bit */
	if (unlikely(!i2c_send_byte(L3G4200D_ADDR, 0xa8))) {
		serial_write1('Z');
		serial_write1('S');
	}

	if (unlikely(!i2c_request_bytes(L3G4200D_ADDR, sizeof(regs), regs))) {
		serial_write1('Z');
		serial_write1('R');
	}

	g[0] = ((uint16_t) regs[1] << 8) | regs[0];
	g[1] = ((uint16_t) regs[3] << 8) | regs[2];
	g[2] = ((uint16_t) regs[5] << 8) | regs[4];
}

static inline void l3g4200d_init(void) {
	/* Write CTRL_REG1: Enable all and set 100Hz data rate */
	if (unlikely(!i2c_send_bytes(L3G4200D_ADDR, 2,
					(uint8_t[]) { 0x20, 0x0f })))
		serial_write1('Z');

	/* Enable the 16bit output registers buffering in CTRL_REG4 to
	 * avoid races */
	if (unlikely(!i2c_send_bytes(L3G4200D_ADDR, 2,
					(uint8_t[]) { 0x23, 0x80 })))
		serial_write1('Z');

	/* All other registers seem to have just the right defaults */
}
