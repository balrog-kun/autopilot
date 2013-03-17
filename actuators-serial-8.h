/*
 * Generates the signal that drives RC actuators (motors and servos).
 *
 * Licensed under AGPLv3.
 */

volatile uint16_t actuators_serial[3];
volatile uint32_t actuators_serial_ts[3];

/* The ESC's serial operation timeout is something below 1000 cycles and
 * it runs at 16MHz.  */
#define ACT_TIMEOUT ((int) (1000.0 / 16000000 * F_CPU))

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
	////uint32_t until;

	act_pulse(pin);
	/* For some reason this is too long for reads, ok for writes,
	 * while using a constant number of NOPs works just ok.  */
#if 0
	until = timer_read() + (uint32_t) (F_CPU * 0.000004);
	while ((int32_t) (timer_read() - until) < 0);
#else
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
			"nop\n\t"
			"nop\n\t"
			"nop\n\t"
			::);
#endif
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

static inline void act_raw_write(int target, uint8_t val) {
	uint32_t flags;

	/* Set data pin high to signal a write op, clear interrupts */
	LPC_GPIO0->DATA |= DATA_PIN_MASK;
	flags = irq_save();
	act_pulse_wait(target);

	/* Bit 7 */
	if (val >> 7)
		LPC_GPIO0->DATA |= DATA_PIN_MASK;
	else
		LPC_GPIO0->DATA &= ~ALL_PINS_MASK;
	act_pulse_wait(target);
	val <<= 1;

	/* Bit 6 */
	if (val >> 7)
		LPC_GPIO0->DATA |= DATA_PIN_MASK;
	else
		LPC_GPIO0->DATA &= ~ALL_PINS_MASK;
	act_pulse_wait(target);
	val <<= 1;

	/* Bit 5 */
	if (val >> 7)
		LPC_GPIO0->DATA |= DATA_PIN_MASK;
	else
		LPC_GPIO0->DATA &= ~ALL_PINS_MASK;
	act_pulse_wait(target);
	val <<= 1;

	/* Bit 4 */
	if (val >> 7)
		LPC_GPIO0->DATA |= DATA_PIN_MASK;
	else
		LPC_GPIO0->DATA &= ~ALL_PINS_MASK;
	act_pulse_wait(target);
	val <<= 1;

	/* Bit 3 */
	if (val >> 7)
		LPC_GPIO0->DATA |= DATA_PIN_MASK;
	else
		LPC_GPIO0->DATA &= ~ALL_PINS_MASK;
	act_pulse_wait(target);
	val <<= 1;

	/* Bit 2 */
	if (val >> 7)
		LPC_GPIO0->DATA |= DATA_PIN_MASK;
	else
		LPC_GPIO0->DATA &= ~ALL_PINS_MASK;
	act_pulse_wait(target);
	val <<= 1;

	/* Bit 1 */
	if (val >> 7)
		LPC_GPIO0->DATA |= DATA_PIN_MASK;
	else
		LPC_GPIO0->DATA &= ~ALL_PINS_MASK;
	act_pulse_wait(target);
	val <<= 1;

	/* Bit 0 */
	if (val >> 7)
		LPC_GPIO0->DATA |= DATA_PIN_MASK;
	else
		LPC_GPIO0->DATA &= ~ALL_PINS_MASK;
	act_pulse(target);

	actuators_serial_ts[target] = timer_read();
	irq_restore(flags);
}

static inline uint8_t act_raw_read(int target, int addr) {
	uint32_t flags;
	uint8_t ret = 0;

	/* Set the relevant GPIOs as outputs */
	LPC_GPIO0->DIR |= ALL_PINS_MASK;

	/* Set data pin low to signal a read op */
	LPC_GPIO0->DATA &= ~ALL_PINS_MASK;
	flags = irq_save();
	act_pulse_wait(target);

	/* Send direction bit & receive data bit 7 */
	if (addr)
		LPC_GPIO0->DATA |= DATA_PIN_MASK;
	else
		LPC_GPIO0->DATA &= ~ALL_PINS_MASK;
	act_pulse_wait(target);
	LPC_GPIO0->DIR &= ~DATA_PIN_MASK; /* Set DATA to input */
	if (LPC_GPIO0->DATA & DATA_PIN_MASK)
		ret |= 1 << 7;

	/* Bit 6 */
	act_pulse_wait(target);
	if (LPC_GPIO0->DATA & DATA_PIN_MASK)
		ret |= 1 << 6;

	/* Bit 5 */
	act_pulse_wait(target);
	if (LPC_GPIO0->DATA & DATA_PIN_MASK)
		ret |= 1 << 5;

	/* Bit 4 */
	act_pulse_wait(target);
	if (LPC_GPIO0->DATA & DATA_PIN_MASK)
		ret |= 1 << 4;

	/* Bit 3 */
	act_pulse_wait(target);
	if (LPC_GPIO0->DATA & DATA_PIN_MASK)
		ret |= 1 << 3;

	/* Bit 2 */
	act_pulse_wait(target);
	if (LPC_GPIO0->DATA & DATA_PIN_MASK)
		ret |= 1 << 2;

	/* Bit 1 */
	act_pulse_wait(target);
	if (LPC_GPIO0->DATA & DATA_PIN_MASK)
		ret |= 1 << 1;

	/* Bit 0 */
	act_pulse_wait(target);
	if (LPC_GPIO0->DATA & DATA_PIN_MASK)
		ret |= 1 << 0;

	irq_restore(flags);

	/* Start a dummy op to tell the ESC to stop driving DATA? */
	my_delay(1);
	act_pulse(target);
	actuators_serial_ts[target] = timer_read();

	return ret;
}

static void die(void);
void actuators_serial_init(int devices) {
	uint8_t vals[3];

	if (devices > 3)
		die();

	LPC_IOCON->RESET_PIO0_0 = 0x0c1; /* GPIO, no pull-up/-down */
	LPC_IOCON->PIO0_1 = 0x0c0; /* GPIO, no pull-up/-down */
	LPC_IOCON->PIO0_2 = 0x0c0; /* GPIO, no pull-up/-down */
	LPC_IOCON->PIO0_3 = 0x0c0; /* GPIO, no pull-up/-down */

	/* Set the relevant GPIOs as inputs */
	LPC_GPIO0->DIR &= ~ALL_PINS_MASK;

	my_delay(200);

	LPC_IOCON->PIO0_3 = 0x0c8; /* GPIO with a pull-down */

	vals[0] = act_raw_read(0, 0);
	vals[1] = act_raw_read(1, 0);
	vals[2] = act_raw_read(2, 0);
	my_delay(200);
	vals[0] = act_raw_read(0, 0);
	vals[1] = act_raw_read(1, 0);
	vals[2] = act_raw_read(2, 0);

	/* TODO: support hot-starting too */
	if (vals[0] == 0x05)
		serial_write_str("ESC 1 is alive\r\n");
	else
		serial_write_str("Can't contact ESC 1!!!\r\n");
	if (vals[1] == 0x05)
		serial_write_str("ESC 2 is alive\r\n");
	else
		serial_write_str("Can't contact ESC 2!!!\r\n");
	if (vals[2] == 0x05)
		serial_write_str("ESC 3 is alive\r\n");
	else
		serial_write_str("Can't contact ESC 3!!!\r\n");
	//// die() if not all alive or not all 0x00 (no battery operation)

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

	if ((actuators_serial[target] >> 8) == (value >> 8) &&
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

	act_raw_write(target, value >> 8);
	actuators_serial[target] = value;
}
