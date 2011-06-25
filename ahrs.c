/*
 * A simple fixed-point AHRS based on intertial and magentic sensors.
 *
 * This is probably the "heart" of the autopilot.
 *
 * This currently uses an "8-DoF" set of sensors: a 2-axis gyro and a 3-axis
 * accelerometer and magnetometer.  It should be easy to adapt to any other
 * set of sensors however.  The gyro output is integrated to obtain th
 * current attitude, but due to integration, it is susceptible to drift.  It
 * is reliable in detecting sudden turns however and is also a reliable
 * source of current rotation rate.  The magnetic field vector and the gravity
 * vector (approximated from the filtered accelerometer readings over a couple
 * of seconds) are assumed to not drift and are used as a reference to remove
 * the gyroscope's drift.  Their expected values are compared against the
 * "current" values and the attitude obtained from integrating rotation rate
 * is corrected based on this.  It should be easy to add more reference
 * vectors or work without one of them, or use a 3-axis gyro instead of
 * the 2-axis one.
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include "adc.h"
#include "twi.h"
#include "timer1.h"
#include "ahrs.h"

volatile int32_t ahrs_pitch, ahrs_roll;
volatile int16_t ahrs_yaw, ahrs_pitch_rate, ahrs_roll_rate, ahrs_yaw_rate;

static volatile int32_t rel_pitch, rel_roll;

/* Doesn't seem to make a whole lot of difference */
#define USE_REFERENCE_V
volatile static uint32_t now;
static uint32_t prev_ts;
static int32_t x_ref, y_ref;

static void gyro_ahrs_update(void) {
	uint32_t diff;
	uint16_t x = adc_values[0];
	uint16_t y = adc_values[1];

	diff = now - prev_ts;
	prev_ts = now;

#ifdef USE_REFERENCE_V
	x -= adc_values[2];
	y -= adc_values[2];
#endif

	/*
	 * Assuming the 10-bit ADC's reference volatage 3.3V and
	 * the gyro outputs to change 3.3 mV/deg/s, 1 LSB of
	 * x/y/ref corresponds to 0.9765625 deg/s, so nearly 1
	 * deg/s.  Assuming F_CPU of 16M reduced by TIME_RES bits,
	 * at 32 bits to represent 360 degrees of rotation we can
	 * afford almost exactly TIME_RES fractional bits in
	 * x_ref/y_ref, e.g:
	 * ((2 ** 32) / (0.9765625 * 360 * (16000000 >> 5)) == 24.43)
	 *
	 * We assume here that the time since last conversion is never
	 * longer than some 30ms, if it is too long this may overflow.
	 */
#define TIME_RES 2 /* 1+1 because adc_values are doubled */
#define DIFF_RES 3
#define REF_RES (DIFF_RES + 7)
	diff = (diff/* + (1 << (DIFF_RES - 1))*/) >> DIFF_RES;
	rel_roll += (int32_t) ((((int32_t) (int16_t) x << REF_RES) - x_ref) *
			diff + (1 << (REF_RES - DIFF_RES + TIME_RES - 1))) >>
		(REF_RES - DIFF_RES + TIME_RES);
	rel_pitch += (int32_t) ((((int32_t) (int16_t) y << REF_RES) - y_ref) *
			diff + (1 << (REF_RES - DIFF_RES + TIME_RES - 1))) >>
		(REF_RES - DIFF_RES + TIME_RES);
}

static void gyro_update(void) {
	now = timer_read();
	adc_convert_all(gyro_ahrs_update);

	/* The gyro's bandwidth is 140Hz so schedule the next measurement
	 * 1/140 sec from this measurement.  Don't try compensate if the
	 * timer was late this time, if the timer triggers sooner than 1/140
	 * sec next time, the measurements won't be independent.  */
	set_timeout(now + F_CPU / 140, gyro_update);
}

static void gyro_cal_update(void) {
	x_ref += adc_values[0];
	y_ref += adc_values[1];
#ifdef USE_REFERENCE_V
	x_ref -= adc_values[2];
	y_ref -= adc_values[2];
#endif
}

#define CMPS09_ADDR 0x60

static void cmps09_read_bytes(uint8_t from, uint8_t count, uint8_t *out) {
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

/* 15-bit sine LUT indexed by angle.  */
const uint16_t sin_lut[130] PROGMEM = {
	0, 402, 804, 1206, 1608, 2009, 2411, 2811, 3212, 3612, 4011, 4410,
	4808, 5205, 5602, 5998, 6393, 6787, 7180, 7571, 7962, 8351, 8740,
	9127, 9512, 9896, 10279, 10660, 11039, 11417, 11793, 12167, 12540,
	12910, 13279, 13646, 14010, 14373, 14733, 15091, 15447, 15800, 16151,
	16500, 16846, 17190, 17531, 17869, 18205, 18538, 18868, 19195, 19520,
	19841, 20160, 20475, 20788, 21097, 21403, 21706, 22006, 22302, 22595,
	22884, 23170, 23453, 23732, 24008, 24279, 24548, 24812, 25073, 25330,
	25583, 25833, 26078, 26320, 26557, 26791, 27020, 27246, 27467, 27684,
	27897, 28106, 28311, 28511, 28707, 28899, 29086, 29269, 29448, 29622,
	29792, 29957, 30118, 30274, 30425, 30572, 30715, 30853, 30986, 31114,
	31238, 31357, 31471, 31581, 31686, 31786, 31881, 31972, 32058, 32138,
	32214, 32286, 32352, 32413, 32470, 32522, 32568, 32610, 32647, 32679,
	32706, 32729, 32746, 32758, 32766, 32767, 32767,
};

/* 7th century formula that gives only about 0.0015 maximum error */
static int16_t sin_16_bhaskara(int16_t angle) {
	uint32_t w;
	if (!(angle & 0x3fff))
		return angle << 1;
	if (angle >= 0) {
		w = (uint32_t) angle * (0x8000 - angle);
		return (w << 4) / ((5L << 15) - (w >> 13));
	}
	angle = (uint16_t) angle & 0x7fff;
	w = (uint32_t) angle * (0x8000 - angle);
	return -(int16_t) ((w << 4) / ((5L << 15) - (w >> 13)));
}

/* About 0.012 maximum error, fast (about 40 AVR cycles) */
static int16_t sin_16(int16_t x) {
	if (x >= 0) {
		if (x > 0x4000)
			x = 0x8000 - x;
		return pgm_read_word(sin_lut + (x >> 7));
	}
	x = (uint16_t) x & 0x7fff;
	if (x > 0x4000)
		x = 0x8000 - x;
	return -pgm_read_word(sin_lut + (x >> 7));
}

/* About 0.0001 maximum error, relatively fast (about 80 AVR cycles) */
static int16_t sin_16_l(int16_t x) {
	int16_t b;
	if (x >= 0) {
		if (x > 0x4000)
			x = 0x8000 - x;
		b = pgm_read_word(sin_lut + (x >> 7));
		return b +
			(((uint16_t) (pgm_read_word(sin_lut + (x >> 7) + 1) -
				      b) * (x & 127) + 13) >> 7);
	}
	x = (uint16_t) x & 0x7fff;
	if (x > 0x4000)
		x = 0x8000 - x;
	b = pgm_read_word(sin_lut + (x >> 7));
	return -b -
		(((uint16_t) (pgm_read_word(sin_lut + (x >> 7) + 1) -
			      b) * (x & 127) + 13) >> 7);
}

static inline int16_t cos_16_l(int16_t x) {
	return sin_16_l(x + 0x4000);
}

/* 15-bit sine LUT indexed by angle.  */
const uint16_t sin_lut32[169] PROGMEM = {
	0, 307, 614, 921, 1228, 1535, 1841, 2148, 2454, 2760, 3066, 3371,
	3677, 3982, 4286, 4590, 4894, 5198, 5501, 5803, 6105, 6406, 6707,
	7007, 7307, 7606, 7904, 8202, 8499, 8795, 9090, 9385, 9679, 9972,
	10264, 10555, 10845, 11134, 11423, 11710, 11996, 12281, 12565, 12848,
	13130, 13411, 13691, 13969, 14246, 14522, 14797, 15070, 15342, 15612,
	15882, 16150, 16416, 16681, 16945, 17207, 17467, 17726, 17984, 18240,
	18494, 18746, 18997, 19247, 19494, 19740, 19985, 20227, 20468, 20707,
	20944, 21179, 21412, 21644, 21873, 22101, 22327, 22550, 22772, 22992,
	23210, 23425, 23639, 23851, 24060, 24268, 24473, 24676, 24877, 25076,
	25272, 25467, 25659, 25848, 26036, 26221, 26404, 26585, 26763, 26939,
	27113, 27284, 27453, 27619, 27783, 27945, 28104, 28261, 28415, 28567,
	28716, 28862, 29006, 29148, 29287, 29423, 29557, 29689, 29817, 29943,
	30067, 30187, 30305, 30421, 30534, 30644, 30751, 30856, 30958, 31057,
	31154, 31248, 31339, 31427, 31512, 31595, 31675, 31753, 31827, 31899,
	31967, 32034, 32097, 32157, 32215, 32270, 32321, 32371, 32417, 32460,
	32501, 32539, 32573, 32605, 32634, 32661, 32684, 32705, 32722, 32737,
	32749, 32758, 32764, 32767, 32767,
};

static int16_t sin_32_l(int32_t x) {
	int16_t b;
	if (x >= 0) {
		if (x > (ROLL_PITCH_180DEG >> 1))
			x = ROLL_PITCH_180DEG - x;
		b = pgm_read_word(sin_lut32 + (x >> 22));
		return b +
			(((pgm_read_word(sin_lut32 + (x >> 22) + 1) - b) *
			  (x & 0x3fffff) + 0x1fffff) >> 22);
	}
	x += ROLL_PITCH_180DEG;
	if (x > (ROLL_PITCH_180DEG >> 1))
		x = ROLL_PITCH_180DEG - x;
	b = pgm_read_word(sin_lut32 + (x >> 22));
	return -b -
		(((pgm_read_word(sin_lut32 + (x >> 22) + 1) - b) *
		  (x & 0x3fffff) + 0x1fffff) >> 22);
}

static inline int16_t cos_32_l(int32_t x) {
	x += ROLL_PITCH_180DEG >> 1;
	if (x > ROLL_PITCH_180DEG)
		x -= ROLL_PITCH_180DEG;
	return sin_32_l(x);
}

static inline void rotate(int16_t *ret, int16_t v[3],
		int16_t y, int32_t p, int32_t r) {
	int16_t c, s;
	int32_t x;

	/* Roll */
	c = cos_32_l(r), s = sin_32_l(r);
	ret[1] = ((int32_t) v[1] * c + (int32_t) v[2] * s + 0x3fff) >> 15;
	ret[2] = ((int32_t) v[2] * c - (int32_t) v[1] * s + 0x3fff) >> 15;
	/* Pitch */
	c = cos_32_l(p), s = sin_32_l(p);
	x = ((int32_t) v[0] * c - (int32_t) ret[2] * s + 0x3fff) >> 15;
	ret[2] = ((int32_t) ret[2] * c + (int32_t) v[0] * s + 0x3fff) >> 15;
	/* Yaw */
	c = cos_16_l(y), s = sin_16_l(y);
	ret[0] = ((int32_t) x * c + (int32_t) ret[1] * s + 0x3fff) >> 15;
	ret[1] = ((int32_t) ret[1] * c - (int32_t) x * s + 0x3fff) >> 15;
}

static inline void rotate_rev(int16_t *ret, int16_t v[3],
		int16_t y, int32_t p, int32_t r) {
	int16_t c, s;
	int32_t z;

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
	ret[1] = ((int32_t) ret[1] * c - z * s + 0x3fff) >> 15;
	ret[2] = (z * c + (int32_t) ret[1] * s + 0x3fff) >> 15;
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

static uint32_t v_ts;
static int16_t statica[3]; /* Initial Acc readings average */
static int16_t staticm[3]; /* Initial Mag readings average */
static int32_t avga[3];
static int32_t avgm[3];

static void vectors_cal(void) {
	uint8_t regs[12];

	cmps09_read_bytes(10, 12, regs);
	avgm[0] += (int16_t) (((uint16_t) regs[0] << 8) | regs[1]);
	avgm[1] += (int16_t) (((uint16_t) regs[2] << 8) | regs[3]);
	avgm[2] += (int16_t) (((uint16_t) regs[4] << 8) | regs[5]);
	avga[0] += (int16_t) (((uint16_t) regs[6] << 8) | regs[7]);
	avga[1] += (int16_t) (((uint16_t) regs[8] << 8) | regs[9]);
	avga[2] += (int16_t) (((uint16_t) regs[10] << 8) | regs[11]);
}

static void vectors_update(void) {
	int32_t pitch, roll, lensq;
	int16_t a[3], m[3]; /* Current Acc & Mag readings */
	int16_t rotated[3]; /* Rotated (predicted) vector */
	int16_t crossed[3]; /* Cross product of current & predicted vectors */
	uint8_t regs[12];
	uint16_t factor;

	/* The magnetometers's measurement frequency is 50Hz and the
	 * accelerometer's rate is limite by buffer referesh rate of 55Hz, so
	 * schedule the next measurement 1/50 sec from the time the previous
	 * measurement was *supposed* to happen.  Not sure if that's really
	 * better, need to think about it.  */
	v_ts += F_CPU / 50;
	set_timeout(v_ts, vectors_update);

	/* Retrieve current values of everything */

	cmps09_read_bytes(10, 12, regs); /* TODO: be async */
	m[0] = ((uint16_t) regs[0] << 8) | regs[1];
	m[1] = ((uint16_t) regs[2] << 8) | regs[3];
	m[2] = ((uint16_t) regs[4] << 8) | regs[5];
	a[0] = ((uint16_t) regs[6] << 8) | regs[7];
	a[1] = ((uint16_t) regs[8] << 8) | regs[9];
	a[2] = ((uint16_t) regs[10] << 8) | regs[11];
	m[0] -= cmps09_mag_calib[0];
	m[1] -= cmps09_mag_calib[1];
	m[2] -= cmps09_mag_calib[2];

	cli();
	pitch = -rel_pitch, rel_pitch = 0;
	roll = -rel_roll, rel_roll = 0;
	sei();

	/* TODO: decoupling like in FlightCtrl */
	ahrs_pitch_rate = pitch; /* TODO */
	ahrs_roll_rate = roll; /* TODO */

	/* Integrate */

	pitch += ahrs_pitch;
	roll += ahrs_roll;
//	pitch = ahrs_pitch;///
//	roll = ahrs_roll;///
	if (unlikely(pitch > ROLL_PITCH_180DEG))
		pitch -= 2 * (uint32_t) ROLL_PITCH_180DEG;
	else if (unlikely(pitch < -ROLL_PITCH_180DEG))
		pitch += 2 * (uint32_t) ROLL_PITCH_180DEG;
	if (unlikely(roll > ROLL_PITCH_180DEG))
		roll -= 2 * (uint32_t) ROLL_PITCH_180DEG;
	else if (unlikely(roll < -ROLL_PITCH_180DEG))
		roll += 2 * (uint32_t) ROLL_PITCH_180DEG;

	/* Fuse information from the different sensors */

	/* TODO: use the dot product to check if the angle between
	 * current and predicted vector is > 90deg */

	/* We rotate the current vectors to our local system and compare
	 * against our reference vectors rather than rotating the reference
	 * vectors to produce the predicted/expected value and comparing
	 * against measured values.
	 *
	 * Note that while the magnetometer readings are more reliable in
	 * detecting quick changes, the readings have lower resolution than
	 * the accelerometer readings so there may be some noise cause by
	 * that.
	 *
	 * The CMPS09 magnetometer resolution seems to be about 0.001 tesla
	 * per 1 LSB, so |m| of 400 is about 0.4 tesla.
	 *
	 * The accelerometer resolution seems to be about 0.0006 m/s^2 per
	 * 1 LSB so 0x4000 is about 1g.
	 *
	 * Note that we give a higher priority to the magnetometer on the
	 * vertical axis and lower on the horizontal axes, and we do the
	 * opposite for the accelerometer.  This is because the magnetic
	 * field vector tends to be closer to horizontal (parallel to earth
	 * surface, outside of the polar areas anyway and away from big
	 * magnets), while gravity/acceleartion vector is on average vertical.
	 * If someone decides to mount the board differently to the
	 * vehicle's "normal" orientation, or the vehicle has no "normal"
	 * orientation, then these should be adjusted.  The differences in
	 * priority are on order of 2-4, so even without adjustments things
	 * should still work.
	 * Both sensors are given little priority on X/Y axes because the
	 * gyro is our primary data source.
	 */
	/*
	 * Normalise / sanitise / reality-check the magnetic vector,
	 * may be completely off..
	 * TODO: divide cross product by square length?
	 * TODO: should we compare field magnitude against constant values
	 * or the calibration vector (staticm)?  If we calibrated in an
	 * area of disturbed field, then all our later calculations will be
	 * wrong.
	 */
	lensq = hypot3(m);
	if (lensq < 300l * 300 || lensq > 500l * 500) {
		factor = 0;
		/* TODO: light a red LED? */
	} else if (lensq < 350l * 350)
		factor = lensq - 350l * 350;
	else if (lensq > 470l * 470)
		factor = 500l * 500 - lensq;
	else
		factor = 32000;

	rotate_rev(rotated, m, ahrs_yaw, pitch, roll);
	cross(crossed, staticm, rotated, factor >> 3);
	/* Assuming |m| and |staticm| of about 0.4T,
	 * crossed_n is about sin(angular distance) << 12
	 */
	ahrs_yaw -= (crossed[2] + 2) >> 2;
	pitch -= (int32_t) crossed[1] << 6;
	roll += (int32_t) crossed[0] << 6;

	rotate_rev(rotated, a, ahrs_yaw, pitch, roll);
	cross(crossed, statica, rotated, 1);
	ahrs_yaw -= (crossed[2] + 4) >> 3;
	pitch -= (int32_t) crossed[1] << 8;
	roll += (int32_t) crossed[0] << 8;
#ifdef CAL
	if (!(ahrs_pitch & 7)) {
		serial_write_hex16(m[0]);
		serial_write_hex16(m[1]);
		serial_write_hex16(m[2]);
		serial_write_dec32((int32_t) isqrt32(lensq));
		serial_write_dec32((int32_t) factor);
		serial_write_eol();
	}
#endif

	/* Write resulting current attitude */

	cli();
	ahrs_pitch = pitch;
	ahrs_roll = roll;
	sei();
}

void ahrs_init(void) {
	int i;

	/* Calibrate the sensors while waiting for the ESCs to detect
	 * voltages etc. */
	x_ref = y_ref = 0;
	avga[0] = avga[1] = avga[2] = 0;
	avgm[0] = avgm[1] = avgm[2] = 0;
	for (i = 0; i < (1 << REF_RES); i ++) {
		adc_convert_all(gyro_cal_update);
		my_delay(4);
		if (!(i & 7)) /* Every 32ms or so */
			vectors_cal();
	}
	statica[0] = (avga[0] + ((1 << (REF_RES - 3)) - 1)) >> (REF_RES - 3);
	statica[1] = (avga[1] + ((1 << (REF_RES - 3)) - 1)) >> (REF_RES - 3);
	statica[2] = (avga[2] + ((1 << (REF_RES - 3)) - 1)) >> (REF_RES - 3);
	staticm[0] = (avgm[0] + ((1 << (REF_RES - 3)) - 1)) >> (REF_RES - 3);
	staticm[1] = (avgm[1] + ((1 << (REF_RES - 3)) - 1)) >> (REF_RES - 3);
	staticm[2] = (avgm[2] + ((1 << (REF_RES - 3)) - 1)) >> (REF_RES - 3);
	serial_write_hex16(statica[0]);/////
	serial_write_hex16(statica[1]);
	serial_write_hex16(statica[2]);
	serial_write_hex16(staticm[0]);
	serial_write_hex16(staticm[1]);
	serial_write_hex16(staticm[2]);
	serial_write_eol();////

	ahrs_pitch = ahrs_roll = ahrs_yaw = 0;
	rel_pitch = rel_roll = 0;

	/* Start the correcting vector updates first */
	v_ts = timer_read();
	vectors_update();

	/* Start the gyro output integration loop */
	prev_ts = timer_read();
	gyro_update();
}
