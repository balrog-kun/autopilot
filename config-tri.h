/* ritewing2
  x - back
  y - right
  z - up
*/

/* ritewing3
  x - right
  y - front
  z - up
*/

/* tri0
  x - left?
  y - back?
  z - up?
 */

/* tri1
  x - left
  y - back
  z - up
  pitch_rate - right
  roll_rate - back
  yaw_rate - left
 */

#define LEFT_MOTOR	0
#define RIGHT_MOTOR	1
#define RUDDER_MOTOR	2
#define RUDDER_SERVO	0

static const int REVERSE_MASK =
	((0 << (15 - RUDDER_MOTOR)) |
	 (1 << (15 - RIGHT_MOTOR)) |
	 (1 << (15 - LEFT_MOTOR)));

static void actuators_setup(void) {
	actuators_hwpwm_init(1);
	actuators_serial_init(3);
}

static void actuators_start(void) {
	actuator_serial_set(RUDDER_MOTOR, 0);
	actuator_serial_set(LEFT_MOTOR, 0);
	actuator_serial_set(RIGHT_MOTOR, 0);
	actuators_serial_start();
	actuator_hwpwm_set(RUDDER_SERVO, 0x8000);
	actuators_hwpwm_start();
}

static void model_control_update(int32_t dest_pitch, int32_t dest_roll,
		int32_t dest_yaw, int32_t co_throttle) {
	int32_t ldiff, rdiff, bdiff, ydiff;
	int32_t throttle_left, throttle_right, throttle_rear, rudder;
	int prev_armed = 0;

	/* 7 / 8 is about 3 / 2 / sqrt(3) which is 1 / arm length for pitch. */
	ldiff = -dest_pitch * 7 / 8 + dest_roll; // + dest_yaw * 0.001
	rdiff = -dest_pitch * 7 / 8 - dest_roll; // - dest_yaw * 0.001
	bdiff = dest_pitch * 7 / 8; // + (abs(dest_yaw) >> 6);
	ydiff = dest_yaw;
	{ /* HACK filter */
		//// convert this to YAW_D?
		static int32_t ydiffr = 0;
		ydiffr += ydiff - ydiffr / 16;////
		ydiff = ydiffr / 16;
	}
#define RUDDER_NEUTRAL 0x6c00 /* Wild guess */

	/* TODO: move to non-model-specific part */
	if (modes & (1 << MODE_ADAPTIVE_ENABLE)) { /* Adapt coefficients */
		/* TODO: Maintan also a slow (longer term) average, then use
		 * quick - slow = a and slow = b in
		 * y = a * x + b
		 * Note that this needs to be _simple_ */
#define quick_avg(i) ((int32_t) config[CFG_ADAPTIVE_0 + i])
#define quick_avg_raw(i) config[CFG_ADAPTIVE_0 + i]

		if (co_throttle > hover_thr) { /* Adapt coefficients */
			int32_t sum;

			quick_avg_raw(0) += ldiff / 16;
			quick_avg_raw(1) += rdiff / 16;
			quick_avg_raw(2) += bdiff / 16;
			quick_avg_raw(3) += ydiff / 16;

			/* Balance the three motor throttles around zero */
			sum = (quick_avg(0) + quick_avg(1) + quick_avg(2)) / 4;
			quick_avg_raw(0) -= sum;
			quick_avg_raw(1) -= sum;
			quick_avg_raw(2) -= sum;
#define CO 2
#define HALFCO (1 << (CO - 1))
			CLAMPS(quick_avg(0), quick_avg_raw(0),
					-0x3000l << CO, 0x3000l << CO);
			CLAMPS(quick_avg(1), quick_avg_raw(1),
					-0x3000l << CO, 0x3000l << CO);
			CLAMPS(quick_avg(2), quick_avg_raw(2),
					-0x3000l << CO, 0x3000l << CO);
			CLAMPS(quick_avg(3), quick_avg_raw(3),
					-0x5000l << CO, 0x5000l << CO);
		}

		ldiff += (quick_avg(0) + HALFCO) >> CO;
		rdiff += (quick_avg(1) + HALFCO) >> CO;
		bdiff += (quick_avg(2) + HALFCO) >> CO;
		ydiff += (quick_avg(3) + HALFCO) >> CO;
	}
//	/* Calculate expected rudder val to add to bdiff */
//	rudder = RUDDER_NEUTRAL - ydiff;
//	CLAMP(rudder, 0, 0xffff);
//	bdiff += abs(rudder - RUDDER_NEUTRAL) >> 9;

	CLAMP(ldiff, -0x6fff, 0x6fff);
	CLAMP(rdiff, -0x6fff, 0x6fff);
	CLAMP(bdiff, -0x6fff, 0x6fff);
	if (co_throttle < 50) {
		throttle_left = ((uint32_t) co_throttle *
				(uint16_t) (0x8000 + ldiff)) >> 8;
		throttle_right = ((uint32_t) co_throttle *
				(uint16_t) (0x8000 + rdiff)) >> 8;
		throttle_rear = ((uint32_t) co_throttle *
				(uint16_t) (0x8000 + bdiff)) >> 8;
	} else {
		throttle_left = ((uint32_t) co_throttle * 0x8000 +
				200 * ldiff) >> 8;
		throttle_right = ((uint32_t) co_throttle * 0x8000 +
				200 * rdiff) >> 8;
		throttle_rear = ((uint32_t) co_throttle * 0x8000 +
				200 * bdiff) >> 8;
	}
	rudder = RUDDER_NEUTRAL + ydiff;

	CLAMP(throttle_left, 0, 0xb000);
	CLAMP(throttle_right, 0, 0xb000);
	CLAMP(throttle_rear, 0, 0xb000);
	CLAMP(rudder, 0, 0xffff);

	if (modes & (1 << MODE_MOTORS_ARMED)) {
		actuator_serial_set(LEFT_MOTOR, (uint16_t) throttle_left);
		actuator_serial_set(RIGHT_MOTOR, (uint16_t) throttle_right);
		actuator_serial_set(RUDDER_MOTOR, (uint16_t) throttle_rear);
		prev_armed = 1;
	} else if (prev_armed) {
		actuator_serial_set(LEFT_MOTOR, 0x0000);
		actuator_serial_set(RIGHT_MOTOR, 0x0000);
		actuator_serial_set(RUDDER_MOTOR, 0x0000);
		prev_armed = 0;
	}

	actuator_hwpwm_set(RUDDER_SERVO, (uint16_t) rudder);

	output[0] = (uint16_t) throttle_left;
	output[1] = (uint16_t) throttle_right;
	output[2] = (uint16_t) throttle_rear;
	output[3] = (uint16_t) rudder;
}

#define POLES 14

static void osd_motor_update(uint16_t *vbat) {
	static uint8_t cnt = 0;

	switch (cnt & 3) {
	case 0 ... 2:
		{
			static uint32_t rpm_ts[3] = { 0, 0, 0 };
			uint32_t timediff, rpm;

			timediff = timer_read() - rpm_ts[cnt & 3];
			rpm = (uint32_t) actuator_serial_get_rpm(cnt & 3) *
				(F_CPU * 60 * 2 / POLES / 1000) /
				(timediff / 1000);
			rpm_ts[cnt & 3] += timediff;

			serial_write1(0x84); /* ESCDATA */
			serial_write1(cnt & 3);
			serial_write_bin16(rpm);
		}
		break;
	case 3:
		/* 2.56V reference voltage, 47k resistor to GND and
		 * 470k to VBAT, 16-bit max resolution */
		*vbat = (actuator_serial_get_vbat(1) * 263 * 100) /
			((0x10000 * 100 * 47) / (47 + 470) / 10);
		break;
	}
	cnt ++;
}
