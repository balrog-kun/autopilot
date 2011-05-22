/*
 * 16-bit timer 1 used for timekeeping as well as arbitrary timeouts.
 * Complex stuff.
 */

#include <avr/io.h>
#include <avr/interrupt.h>

#ifndef NULL
# define NULL 0
#endif

void timer_init(void) {
	TCCR1A = 0x00;
	TCCR1B = 0x01;
	TIMSK1 = 0x01;
}

static volatile uint16_t timer_cycles = 0;
static uint32_t last_sec = 0;
static uint32_t seconds = 0;

static void update_timeouts(void);

/* Always called with interrupts disabled */
static void timer_overflow(void) {
	uint32_t new_val;

	timer_cycles ++;

	new_val = (uint32_t) timer_cycles << 16;
	if (new_val >= last_sec + F_CPU || new_val < last_sec) {
		last_sec += F_CPU;
		seconds ++;
	}
}

/* Timekeeping */
ISR(TIMER1_OVF_vect) {
	timer_overflow();

	sei();

	update_timeouts();
}

/* Read current time in cpu cycles (way more complex than it should be..) */
uint32_t timer_read(void) {
	/*
	 * The simple, but not 100% race safe version, not even in interrupt
	 * context, is:
	 * return TCNT1 | ((uint32_t) timer_cycles << 16);
	 */
	uint16_t lo, hi, sreg;

	/*
	 * First make sure no overflow is pending.  This can only happen
	 * with interrupts disabled already..
	 */
	while (TIFR1 & 1) {
		TIFR1 |= 1;

		timer_overflow();
		update_timeouts();
	}

	sreg = SREG;
	cli();

	lo = TCNT1, hi = timer_cycles;
	/*
	 * If an overflow is now pending (must have happened after the cli)
	 * and lo is low, meaning that only a few cycles had happened since
	 * the last overflow, then we assume that hi contains the value from
	 * before the overflow, while lo contains the value from after it
	 * which is the only "interesting" scenario.  Compensate for that.
	 */
	if ((TIFR1 & 1) && lo < 0x8000)
		hi ++;

	SREG = sreg;
	return ((uint32_t) hi << 16) | lo;
}

/* Burn some cycles */
void my_delay(uint16_t msecs) {
	uint32_t end = timer_read() + (uint32_t) msecs * (F_CPU / 1000);
	while (timer_read() < end);
}

/*
 * This is a super simple timeout support.  There is a limited number of
 * timeouts that can be set at any given point.  Timeouts that are far
 * enough from now and far enough from other timeouts, should be rather
 * accurate, but there are no guarantees.  The callback may happen a
 * little later than expected, but not sooner than expected.  This is
 * guaranteed, but the "little later" can be rather long in extreme
 * cases.  It will be reasonably short if callback routines are reasonable
 * short.  Timeouts being set need to be within 2^32 / F_CPU / 2 seconds
 * from now, which is about 120 seconds at 16MHz.
 *
 * Callbacks currently run with interrupts enabled in order let the timer
 * overflow do its job, but if the callbacks don't take too long, they
 * could be given the comfort of interrupts disabled which would simplify
 * the code here a little.
 */
#define MAX_TIMEOUTS	16

static struct timeout_s {
	uint32_t when;
	void (*callback)(void);
	uint8_t next;
} timeouts[MAX_TIMEOUTS];
static uint8_t next = 0xff;
static uint8_t k = 0;

void set_timeout(uint32_t when, void (*callback)(void)) {
	uint8_t i, *j, sreg;

	sreg = SREG;
	cli();

	for (i = next, j = &next; i != 0xff; j = &timeouts[i].next, i = *j)
		if (!((uint32_t) (timeouts[i].when - when) >> 31))
			break;
	for (; timeouts[k].callback; k = (k + 1) & (MAX_TIMEOUTS - 1));
	timeouts[k].when = when;
	timeouts[k].callback = callback;
	timeouts[k].next = i;
	*j = k;

	SREG = sreg;

	if (j == &next)
		update_timeouts();
}

ISR(TIMER1_COMPA_vect) {
	/*
	 * This only happens when at least the first timeout has expired,
	 * but we can't just go on and call back here because we want to
	 * allow callbacks to schedule new timeouts...
	 */
	sei();

	update_timeouts();
}

static volatile uint8_t in_update = 0;

/*
 * Another assumption that we make, but which really depends on factors
 * like the compiler flags:
 *
 * setting the timer compare register to a new value will take less than
 * MIN_DELAY cycles.
 */
#define MIN_DELAY 40

static void update_timeouts(void) {
	uint8_t sreg = SREG;
	uint32_t now;

	cli();

	if (in_update)
		return;

	TIMSK1 = 0x01;

	/* Only run the callbacks if the caller has not disabled interrupts */
	if (!(sreg & 0x80))
		goto update_reg;

	in_update = 1;

	while (next != 0xff) {
		void (*cb)(void);

		now = timer_read();
		if (!((uint32_t) (timeouts[next].when - now) >> 31))
			break;

		cb = timeouts[next].callback;
		timeouts[next].callback = NULL;
		next = timeouts[next].next;

		sei();
		cb();
		cli();
	}

	in_update = 0;

update_reg:
	if (next != 0xff && timer_cycles == (timeouts[next].when >> 16)) {
		/*
		 * If the desired value is in the past or very close to now,
		 * we make it now + MIN_DELAY cycles to avoid any race
		 * conditions.  The hope is that this function will not take
		 * longer than MIN_DELAY cycles.
		 */
		uint16_t ocra = timeouts[next].when;
		if (TCNT1 - ocra > MIN_DELAY)
			OCR1A = ocra;
		else
			OCR1A = TCNT1 + MIN_DELAY;
		TIMSK1 = 0x03;
	}

	SREG = sreg;
}
