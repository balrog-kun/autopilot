/*
 * Generates the signal that drives RC actuators (motors and servos).
 *
 * Licensed under AGPLv3.
 */

volatile uint16_t actuators_serial[3];
volatile uint32_t actuators_serial_ts[3];

/* The ESC's serial operation timeout is something below 1600 cycles and
 * it runs at 16MHz.  */
#define ACT_TIMEOUT ((unsigned int) (1700.0 / 16000000 * F_CPU))

#define DATA_PIN_MASK	0b1000
#define ALL_PINS_MASK	0b1111
#define CLK_PINS_MASK	0b0111
#define CLK_PIN_MASK(n)	(1 << n)

/* Since we have pull-ups and pull-downs, we should be able to tell if
 * any device is holding the bus occupied, possibly erroneously as we
 * ourselves take care to make our slaves (the ESCs) release the bus
 * when they're done.  We can't know which ESC is culpable.  */
static int bus_busy(void) {
	//// 1. set the data pin to input
	//// 2. enable the pull-up
	//// 3. if data pin reads low, return 1
	//// 4. disable pull-up, enable pull-down
	//// 5. if data pin reads high, return 1
	//
	//// Unimplemented because code size is more precious at the moment
	//// than handling hypothetical error situations.  Also there are
	//// some reports that the pull-up/-down resistors let enough
	//// current through that even enabling them might cause a slight
	//// short on the bus anyway.
	return 0;
}

static inline void act_pulse(int target) {
	LPC_GPIO0->DATA |= CLK_PIN_MASK(target);
	asm volatile (
			"nop\n\t"
			"nop\n\t"
			"nop\n\t"
			"nop\n\t"
			"nop\n\t"
			"nop\n\t"
			"nop\n\t"
			"nop\n\t"
			"nop\n\t"
			"nop\n\t"
			"nop\n\t"
			"nop\n\t"
			"nop\n\t"
			::);
	LPC_GPIO0->DATA &= ~CLK_PINS_MASK;
}

static inline void act_pulse_wait(int pin) {
	uint32_t until;

	act_pulse(pin);
	until = timer_read() + (uint32_t) (F_CPU * 0.0000015);
	while ((int32_t) (timer_read() - until) < 0);
}

static inline void act_pulse_wait_short(int pin) {
	uint32_t until;

	act_pulse(pin);
	until = timer_read() + (uint32_t) (F_CPU * 0.000001);
	while ((int32_t) (timer_read() - until) < 0);
}

static inline void act_pulse_wait_long(int pin) {
	uint32_t until;

	act_pulse(pin);
	until = timer_read() + (uint32_t) (F_CPU * 0.000003);
	while ((int32_t) (timer_read() - until) < 0);
}

static inline void act_pulse_wait_asm(int pin) {
	act_pulse(pin);
	asm volatile (
			"nop\n\t"
			"nop\n\t"
			"nop\n\t"
			"nop\n\t"
			"nop\n\t"
			"nop\n\t"
			"nop\n\t"
			"nop\n\t"
			"nop\n\t"
			"nop\n\t"
			"nop\n\t"
			"nop\n\t"
			"nop\n\t"
			"nop\n\t"
			"nop\n\t"
			"nop\n\t"
			"nop\n\t"
			"nop\n\t"
			"nop\n\t"
			"nop\n\t"
			"nop\n\t"
			"nop\n\t"
			"nop\n\t"
			"nop\n\t"
			"nop\n\t"
			"nop\n\t"
			"nop\n\t"
			"nop\n\t"
			"nop\n\t"
			"nop\n\t"
			"nop\n\t"
			"nop\n\t"
			"nop\n\t"
			"nop\n\t"
			"nop\n\t"
			"nop\n\t"
			"nop\n\t"
			::);
}

static void try_reset(void) {
	int i;

	for (i = 0; i < 3; i ++) {
		/* If the slave is holding the bus busy because it had (or
		 * it thinks it had) a read operation without us knowing
		 * somehow, then sending it just a single bit with whatever
		 * value should cause it to release the bus.  We have to wait
		 * for that read-op to time out first, though.  */
		while (timer_read() - actuators_serial_ts[i] < ACT_TIMEOUT);

		act_pulse(i);

		actuators_serial_ts[i] = timer_read();
	}
}

/* uint32_t param only because gcc generates better code */
static inline void act_raw_write(int target, uint32_t val) {
	uint32_t flags;

	/* Set data pin high to signal a write op, clear interrupts */
	LPC_GPIO0->DATA |= DATA_PIN_MASK;
	flags = irq_save();
	act_pulse_wait_long(target);

#define SEND_BIT(b)	\
	if (val & (1 << 13))	\
		LPC_GPIO0->DATA |= DATA_PIN_MASK;	\
	else	\
		LPC_GPIO0->DATA &= ~ALL_PINS_MASK;	\
	act_pulse_wait_asm(target);	\
	val <<= 1;

	SEND_BIT(13)
	SEND_BIT(12)
	SEND_BIT(11)
	SEND_BIT(10)
	SEND_BIT(9)
	SEND_BIT(8)
	SEND_BIT(7)
	SEND_BIT(6)
	SEND_BIT(5)
	SEND_BIT(4)
	SEND_BIT(3)
	SEND_BIT(2)
	SEND_BIT(1)

	/* Bit 0 */
	if (val & (1 << 13))
		LPC_GPIO0->DATA |= DATA_PIN_MASK;
	else
		LPC_GPIO0->DATA &= ~ALL_PINS_MASK;
	act_pulse(target);

	actuators_serial_ts[target] = timer_read();
	irq_restore(flags);
}

/* FIXME: this shouldn't be used in this form in motor-run-time, rather use
 * a 14-bit op when we're already running.  */
static inline uint16_t act_raw_read(int target, int addr) {
	uint32_t flags;
	uint16_t ret = 0;

	/* Set data pin low to signal a read op */
	LPC_GPIO0->DATA &= ~ALL_PINS_MASK;
	flags = irq_save();
	act_pulse_wait(target);

	/* Send direction bit & receive data bit 15 */
	if (addr)
		LPC_GPIO0->DATA |= DATA_PIN_MASK;
	else
		LPC_GPIO0->DATA &= ~ALL_PINS_MASK;
	act_pulse_wait(target);
	LPC_GPIO0->DIR &= ~DATA_PIN_MASK; /* Set DATA to input */
	if (LPC_GPIO0->DATA & DATA_PIN_MASK)
		ret |= 1 << 15;

#define RECEIVE_BIT(b)	\
	act_pulse_wait_short(target);	\
	if (LPC_GPIO0->DATA & DATA_PIN_MASK)	\
		ret |= 1 << b;
	RECEIVE_BIT(14)
	RECEIVE_BIT(13)
	RECEIVE_BIT(12)
	RECEIVE_BIT(11)
	RECEIVE_BIT(10)
	RECEIVE_BIT(9)
	RECEIVE_BIT(8)
	RECEIVE_BIT(7)
	RECEIVE_BIT(6)
	RECEIVE_BIT(5)
	RECEIVE_BIT(4)
	RECEIVE_BIT(3)
	RECEIVE_BIT(2)
//	RECEIVE_BIT(1)
//	RECEIVE_BIT(0)

	actuators_serial_ts[target] = timer_read();
	irq_restore(flags);

//	/* Start a dummy op to tell the ESC to stop driving DATA? */
//	act_pulse(target);
	while (timer_read() - actuators_serial_ts[target] < ACT_TIMEOUT);
	LPC_GPIO0->DIR |= ALL_PINS_MASK;
	LPC_GPIO0->DATA &= ~ALL_PINS_MASK;

	return ret;
}

static void die(void);
void actuators_serial_init(int devices) {
	uint16_t vals[3];

	if (devices > 3)
		die();

	LPC_IOCON->RESET_PIO0_0 = 0x0c1; /* GPIO, no pull-up/-down */
	LPC_IOCON->PIO0_1 = 0x0c0; /* GPIO, no pull-up/-down */
	LPC_IOCON->PIO0_2 = 0x0c0; /* GPIO, no pull-up/-down */
	LPC_IOCON->PIO0_3 = 0x0c0; /* GPIO, no pull-up/-down */

	/* Set the DATA GPIOs as inputs, CLK outputs */
//	LPC_GPIO0->DIR &= ~DATA_PIN_MASK;
//	LPC_GPIO0->DIR |= CLK_PINS_MASK;
//	LPC_GPIO0->DATA &= ~ALL_PINS_MASK;
	/* Set DATA and CLK lines as outputs, oh well */
	LPC_GPIO0->DIR |= ALL_PINS_MASK;
	LPC_GPIO0->DATA &= ~ALL_PINS_MASK;

	/* Wait for the ESCs to start main loop */
	my_delay(1000);

	LPC_IOCON->PIO0_3 = 0x0c8; /* GPIO with a pull-down */

	vals[0] = act_raw_read(0, 0);
	vals[1] = act_raw_read(1, 0);
	vals[2] = act_raw_read(2, 0);

	serial_write_hex16(vals[0]);
	serial_write_hex16(vals[1]);
	serial_write_hex16(vals[2]);
	/* TODO: support hot-starting too */
	if (vals[0] == 0x1234)
		serial_write_str("ESC 1 is alive\r\n");
	else
		serial_write_str("Can't contact ESC 1!!!\r\n");
	if (vals[1] == 0x1234)
		serial_write_str("ESC 2 is alive\r\n");
	else
		serial_write_str("Can't contact ESC 2!!!\r\n");
	if (vals[2] == 0x1234)
		serial_write_str("ESC 3 is alive\r\n");
	else
		serial_write_str("Can't contact ESC 3!!!\r\n");
	//// die() if not all alive or not all 0x00 (no battery operation).
	//// perhaps do some sort of ADC voltage check or to see if we really
	//// have no battery, or read VBUS somehow to see if we're powered
	//// from USB.
#if 1
	if (vals[0] != 0x1234 || vals[1] != 0x1234 || vals[2] != 0x1234)
		die();
#endif

	/* Set the relevant GPIOs as outputs */
	LPC_GPIO0->DIR |= ALL_PINS_MASK;
	LPC_GPIO0->DATA &= ~ALL_PINS_MASK;
}

/* TODO: currently we don't continually send signals if the main loop
 * doesn't call us periodically and we could potentially cause a timeout
 * in the ESCs.  Normally the main loop will call us quite often though.  */
void actuators_serial_start(void) {
}
void actuators_serial_stop(void) {
}

static inline void actuator_serial_set(uint8_t target, uint16_t value) {
	uint32_t now = timer_read();

	if ((actuators_serial[target] >> 3) == (value >> 3) &&
			now - actuators_serial_ts[target] < F_CPU)
		return;

	/* Busy wait if the last operation happened just a moment ago */
	while (now - actuators_serial_ts[target] < ACT_TIMEOUT)
		now = timer_read();

	while (bus_busy()) {
		try_reset();
		while (timer_read() - actuators_serial_ts[target] < ACT_TIMEOUT)
			;
	}

	act_raw_write(target, (value >> 3) |
			(((REVERSE_MASK >> 2) << target) & 0x2000));
	actuators_serial[target] = value;
}

static inline uint16_t actuator_serial_get_rpm(uint8_t target) {
	uint32_t now = timer_read();

	/* Busy wait if the last operation happened just a moment ago */
	while (now - actuators_serial_ts[target] < ACT_TIMEOUT)
		now = timer_read();

	while (bus_busy()) {
		try_reset();
		while (timer_read() - actuators_serial_ts[target] < ACT_TIMEOUT)
			;
	}

	return act_raw_read(target, 0) - 0x1234;
}

static inline uint16_t actuator_serial_get_vbat(uint8_t target) {
	uint32_t now = timer_read();

	/* Busy wait if the last operation happened just a moment ago */
	while (now - actuators_serial_ts[target] < ACT_TIMEOUT)
		now = timer_read();

	while (bus_busy()) {
		try_reset();
		while (timer_read() - actuators_serial_ts[target] < ACT_TIMEOUT)
			;
	}

	return act_raw_read(target, 1);
}
