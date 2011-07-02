/*
 * Fixed point sine & cosine and related stuff.
 *
 * Licensed under AGPLv3.
 */

int16_t sin_16_bhaskara(int16_t angle);
int16_t sin_16(int16_t x);
int16_t sin_16_l(int16_t x);
int16_t sin_32_l(int32_t x);

static inline int16_t cos_16_l(int16_t x) {
	return sin_16_l(x + 0x4000);
}

static inline int16_t cos_32_l(int32_t x) {
	x += ROLL_PITCH_180DEG >> 1;
	if (x > ROLL_PITCH_180DEG)
		x -= ROLL_PITCH_180DEG;
	return sin_32_l(x);
}

static inline void rotate(int16_t ret[3], int16_t v[3],
		int16_t y, int32_t p, int32_t r) {
	int16_t c, s, x;

	/* Roll */
	c = cos_32_l(r), s = sin_32_l(r);
	ret[1] = ((int32_t) v[1] * c - (int32_t) v[2] * s + 0x3fff) >> 15;
	ret[2] = ((int32_t) v[2] * c + (int32_t) v[1] * s + 0x3fff) >> 15;
	/* Pitch */
	c = cos_32_l(p), s = sin_32_l(p);
	x = ((int32_t) v[0] * c - (int32_t) ret[2] * s + 0x3fff) >> 15;
	ret[2] = ((int32_t) ret[2] * c + (int32_t) v[0] * s + 0x3fff) >> 15;
	/* Yaw */
	c = cos_16_l(y), s = sin_16_l(y);
	ret[1] = ((int32_t) ret[1] * c - (int32_t) x * s + 0x3fff) >> 15;
	ret[0] = ((int32_t) x * c + (int32_t) ret[1] * s + 0x3fff) >> 15;
}

static inline void rotate_rev(int16_t ret[3], int16_t v[3],
		int16_t y, int32_t p, int32_t r) {
	int16_t c, s, z;

	/* Yaw */
	c = cos_16_l(-y), s = sin_16_l(-y);
	ret[0] = ((int32_t) v[0] * c + (int32_t) v[1] * s + 0x3fff) >> 15;
	ret[1] = ((int32_t) v[1] * c - (int32_t) v[0] * s + 0x3fff) >> 15;
	/* Pitch */
	c = cos_32_l(-p), s = sin_32_l(-p);
	z = ((int32_t) v[2] * c + (int32_t) ret[0] * s + 0x3fff) >> 15;
	ret[0] = ((int32_t) ret[0] * c - (int32_t) v[2] * s + 0x3fff) >> 15;
	/* Roll */
	c = cos_32_l(-r), s = sin_32_l(-r);
	ret[2] = ((int32_t) z * c + (int32_t) ret[1] * s + 0x3fff) >> 15;
	ret[1] = ((int32_t) ret[1] * c - (int32_t) z * s + 0x3fff) >> 15;
}

static inline void cross(int16_t *ret, int16_t va[3], int16_t vb[3],
		uint16_t m) {
	ret[0] = (((int32_t) va[1] * vb[2] - (int32_t) va[2] * vb[1]) * m +
			(1 << 15)) >> 16;
	ret[1] = (((int32_t) va[2] * vb[0] - (int32_t) va[0] * vb[2]) * m +
			(1 << 15)) >> 16;
	ret[2] = (((int32_t) va[0] * vb[1] - (int32_t) va[1] * vb[0]) * m +
			(1 << 15)) >> 16;
}

static inline uint32_t hypot3(int16_t v[3]) {
	return (int32_t) v[0] * v[0] + (int32_t) v[1] * v[1] +
		(int32_t) v[2] * v[2];
}
