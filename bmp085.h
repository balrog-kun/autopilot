#define BMP085_ADDR 0x77

static int16_t bmp085_config_ac123[3];
static uint16_t bmp085_config_ac456[3];
static int16_t bmp085_config_b12[3];
static int16_t bmp085_config_mbcd[3];
static int read_cnt;
static uint32_t read_end_ts;
static int32_t b3, b4;

static inline uint32_t bmp085_read(void) {
	uint8_t regs[3];
	uint32_t ret = 0;

	if ((int32_t) (timer_read() - read_end_ts) < 0)
		return ret;

	if (read_cnt & 127) {
		uint32_t p;

		/* Read the measurement result */
		if (unlikely(!i2c_send_byte(BMP085_ADDR, 0xf6))) {
			serial_write1('U');
			serial_write1('A');
		}
		if (unlikely(!i2c_request_bytes(BMP085_ADDR, 3, regs))) {
			serial_write1('U');
			serial_write1('R');
		}

		p = (regs[0] << 11) | (regs[1] << 3) | (regs[2] >> 5);
		p = (p - b3) * (50000 >> 3);
		if (p < 0x8000000)
			p = p * 2 / b4;
		else
			p = p / b4 * 2;
		ret = p + (((((p >> 8) * (p >> 8) * 3038 - 7357 * p) >> 16) +
					3791) >> 4);
	//	ret = p + ((((((p >> 8) * (p >> 8) * 3038) >> 16) - ((7357 * p) >> 16)) +
	//				3791) >> 4);
	} else {
		int32_t x, b6;

		/* Read the measurement result */
		if (unlikely(!i2c_send_byte(BMP085_ADDR, 0xf6))) {
			serial_write1('U');
			serial_write1('A');
		}
		if (unlikely(!i2c_request_bytes(BMP085_ADDR, 2, regs))) {
			serial_write1('U');
			serial_write1('R');
		}

		x = (regs[0] << 8) | regs[1];
		x -= bmp085_config_ac456[2];
		x *= bmp085_config_ac456[1];
		b6 = x >> 15;
		x = bmp085_config_mbcd[1] << 11;
		b6 += x / (b6 + bmp085_config_mbcd[2]) - 4000;

		/* X1 usually 0, optimise it out? */
		x = (bmp085_config_b12[1] * ((b6 * b6) >> 12)) >> 11;
		x += (bmp085_config_ac123[1] * b6) >> 11;
		b3 = (((bmp085_config_ac123[0] * 4 + x) << 3) + 2) / 4;
		x = (bmp085_config_ac123[2] * b6) >> 13;
		x += (bmp085_config_b12[0] * ((b6 * b6) >> 12)) >> 16;
		b4 = (bmp085_config_ac456[0] * (((x + 2) >> 2) + 32768)) >> 15;
	}

	read_cnt ++;
	if (read_cnt & 127) {
		/* Start a pressure measurement using 8 samples */
		if (unlikely(!i2c_send_bytes(BMP085_ADDR, 2,
						(uint8_t[]) { 0xf4, 0xf4 }))) {
			serial_write1('U');
			serial_write1('W');
		}

		read_end_ts = timer_read() + F_CPU / 36; /* 25.5ms + 1.5ms */
	} else {
		/* Start a temperature measurement */
		if (unlikely(!i2c_send_bytes(BMP085_ADDR, 2,
						(uint8_t[]) { 0xf4, 0x2e }))) {
			serial_write1('U');
			serial_write1('W');
		}

		read_end_ts = timer_read() + F_CPU / 165; /* 4.5ms + 1.5ms */
	}

	return ret;
}

static inline void bmp085_init(void) {
	uint8_t regs[22];

	/* Read the config registers */
	if (unlikely(!i2c_send_byte(BMP085_ADDR, 0xaa)))
		serial_write1('U');
	if (unlikely(!i2c_request_bytes(BMP085_ADDR, sizeof(regs), regs)))
		serial_write1('U');

	bmp085_config_ac123[0] = (regs[0] << 8) | regs[1];
	bmp085_config_ac123[1] = (regs[2] << 8) | regs[3];
	bmp085_config_ac123[2] = (regs[4] << 8) | regs[5];
	bmp085_config_ac456[0] = (regs[6] << 8) | regs[7];
	bmp085_config_ac456[1] = (regs[8] << 8) | regs[9];
	bmp085_config_ac456[2] = (regs[10] << 8) | regs[11];
	bmp085_config_b12[0] = (regs[12] << 8) | regs[13];
	bmp085_config_b12[1] = (regs[14] << 8) | regs[15];
	bmp085_config_mbcd[0] = (regs[16] << 8) | regs[17];
	bmp085_config_mbcd[1] = (regs[18] << 8) | regs[19];
	bmp085_config_mbcd[2] = (regs[20] << 8) | regs[21];

	read_cnt = 0;

	if (unlikely(!i2c_send_bytes(BMP085_ADDR, 2,
					(uint8_t[]) { 0xf4, 0x2e }))) {
		serial_write1('U');
		serial_write1('W');
	}
	read_end_ts = timer_read() + F_CPU / 165;
}
