/* quad like tri1
  x - left
  y - back
  z - up
  pitch_rate - right
  roll_rate - back
  yaw_rate - left
 */

#define LF_MOTOR	0
#define RF_MOTOR	1
#define LR_MOTOR	2
#define RR_MOTOR	3

static const int REVERSE_MASK =
	((0 << (15 - LF_MOTOR)) | /* TODO */
	 (0 << (15 - RF_MOTOR)) |
	 (0 << (15 - LR_MOTOR)) |
	 (1 << (15 - RR_MOTOR)));

static void actuators_setup(void) {
	actuators_i2c_init(4);
}

static void actuators_start(void) {
	actuator_i2c_set(LF_MOTOR, 0);
	actuator_i2c_set(RF_MOTOR, 0);
	actuator_i2c_set(LR_MOTOR, 0);
	actuator_i2c_set(RR_MOTOR, 0);
	actuators_i2c_start();
}

static void model_control_update(int32_t dest_pitch, int32_t dest_roll,
		int32_t dest_yaw, int32_t co_throttle) {
	int32_t diff[4];
	int32_t throttle[4];
	int prev_armed = 0;

	dest_yaw /= 4;
	diff[LF_MOTOR] = -dest_pitch + dest_roll + dest_yaw;
	diff[RF_MOTOR] = -dest_pitch - dest_roll - dest_yaw;
	diff[LR_MOTOR] = dest_pitch + dest_roll - dest_yaw;
	diff[RR_MOTOR] = dest_pitch - dest_roll + dest_yaw;

	if (modes & (1 << MODE_ADAPTIVE_ENABLE)) { /* Adapt coefficients */
		/* TODO: Maintan also a slow (longer term) average, then use
		 * quick - slow = a and slow = b in
		 * y = a * x + b
		 * Note that this needs to be _simple_ */
#define quick_avg(i) ((int32_t) config[CFG_ADAPTIVE_0 + i])
#define quick_avg_raw(i) config[CFG_ADAPTIVE_0 + i]

		if (co_throttle > hover_thr) { /* Adapt coefficients */
			int32_t sum;

			quick_avg_raw(0) += diff[0] / 16;
			quick_avg_raw(1) += diff[1] / 16;
			quick_avg_raw(2) += diff[2] / 16;
			quick_avg_raw(3) += diff[3] / 16;

			/* Balance the three motor throttles around zero */
			sum = (quick_avg(0) + quick_avg(1) +
					quick_avg(2) + quick_avg(3)) / 4;
			quick_avg_raw(0) -= sum;
			quick_avg_raw(1) -= sum;
			quick_avg_raw(2) -= sum;
			quick_avg_raw(3) -= sum;
#define CO 2
#define HALFCO (1 << (CO - 1))
			CLAMPS(quick_avg(0), quick_avg_raw(0),
					-0x5000l << CO, 0x5000l << CO);
			CLAMPS(quick_avg(1), quick_avg_raw(1),
					-0x5000l << CO, 0x5000l << CO);
			CLAMPS(quick_avg(2), quick_avg_raw(2),
					-0x5000l << CO, 0x5000l << CO);
			CLAMPS(quick_avg(3), quick_avg_raw(3),
					-0x5000l << CO, 0x5000l << CO);
		}

		diff[0] += (quick_avg(0) + HALFCO) >> CO;
		diff[1] += (quick_avg(1) + HALFCO) >> CO;
		diff[2] += (quick_avg(2) + HALFCO) >> CO;
		diff[3] += (quick_avg(3) + HALFCO) >> CO;
	}

	CLAMP(diff[0], -0x6fff, 0x6fff);
	CLAMP(diff[1], -0x6fff, 0x6fff);
	CLAMP(diff[2], -0x6fff, 0x6fff);
	CLAMP(diff[3], -0x6fff, 0x6fff);
	/* TODO: allow negative values in some cases (reverse) */
	if (co_throttle < 50) {
		throttle[0] = (co_throttle * (0xa000 + 4 * diff[0])) >> 8;
		throttle[1] = (co_throttle * (0xa000 + 4 * diff[1])) >> 8;
		throttle[2] = (co_throttle * (0xa000 + 4 * diff[2])) >> 8;
		throttle[3] = (co_throttle * (0xa000 + 4 * diff[3])) >> 8;
	} else {
		throttle[0] = (co_throttle * 0xa000 + 200 * diff[0]) >> 8;
		throttle[1] = (co_throttle * 0xa000 + 200 * diff[1]) >> 8;
		throttle[2] = (co_throttle * 0xa000 + 200 * diff[2]) >> 8;
		throttle[3] = (co_throttle * 0xa000 + 200 * diff[3]) >> 8;
	}

	CLAMP(throttle[0], 0, 0xe000);
	CLAMP(throttle[1], 0, 0xe000);
	CLAMP(throttle[2], 0, 0xe000);
	CLAMP(throttle[3], 0, 0xe000);

	if (modes & (1 << MODE_MOTORS_ARMED)) {
		actuator_i2c_set(LF_MOTOR, (uint16_t) throttle[LF_MOTOR]);
		actuator_i2c_set(RF_MOTOR, (uint16_t) throttle[RF_MOTOR]);
		actuator_i2c_set(LR_MOTOR, (uint16_t) throttle[LR_MOTOR]);
		actuator_i2c_set(RR_MOTOR, (uint16_t) throttle[RR_MOTOR]);
		prev_armed = 1;
#if 1
	} else if (prev_armed) {
#else
	} else {
#endif
		actuator_i2c_set(LF_MOTOR, 0x0000);
		actuator_i2c_set(RF_MOTOR, 0x0000);
		actuator_i2c_set(LR_MOTOR, 0x0000);
		actuator_i2c_set(RR_MOTOR, 0x0000);
		prev_armed = 0;
	}

	output[0] = (uint16_t) throttle[0];
	output[1] = (uint16_t) throttle[1];
	output[2] = (uint16_t) throttle[2];
	output[3] = (uint16_t) throttle[3];
}

#define POLES 14

static void osd_motor_update(uint16_t *vbat) {
	static uint8_t cnt = 0;
	static uint32_t rpm_ts[4] = { 0, 0, 0, 0 };
	uint16_t vbatbuf, commcnt, temp;
	uint32_t timediff, rpm, tempv, tempr;

	cnt &= 3;
	timediff = timer_read() - rpm_ts[cnt];
	if (!actuator_i2c_get_vals(cnt, &commcnt, &vbatbuf, &temp))
		goto done;

	if (!cnt) {
		/* 5V reference voltage, 3.3k resistor to GND and
		 * 18k to VBAT, 16-bit max resolution */
		*vbat = (vbatbuf * 500 * 100) /
			((0x10000 * 100 * 33) / (33 + 180) / 10);
	}

	rpm = (uint32_t) commcnt * (F_CPU * 60 * 2 / POLES / 1000) /
		((timediff / 1000) ?: 1);
	rpm_ts[cnt] += timediff;

	/* Voltage (mV) from the NTC resistor */
	tempv = ((uint32_t) temp * 500 * 100) /
		((0x10000 * 100) / 10);
	/* NTC's Resistance (Ohm) */
	tempr = tempv * 3300 / ((5000 - tempv) ?: 1);
	/* R_inf = R_0 * e ^ (-B / T_0) */
#define NTC_B 4050
#define R_INF 0.0126f /* 10000 Ohm * e ^ (-4050 K / (25.0 C + 273.15 K)) */
	temp = (int16_t) (NTC_B * 100.0f / logf((tempr / R_INF) ?: 2)) - 27315;

	serial_write1(0x84); /* ESCDATA */
	serial_write1(cnt);
	serial_write_bin16(rpm);
	serial_write_bin16(temp);

done:
	cnt ++;
}
