/*
 * 32-bit timer 1 used for timekeeping as well as arbitrary timeouts.
 *
 * Licensed under AGPLv3.
 */

#include "LPC13xx.h"		/* LPC13xx Peripheral Registers */
#include "timer1.h"

#ifndef NULL
# define NULL 0
#endif

void timer_init(void) {
	LPC_SYSCON->SYSAHBCLKCTRL |= 1 << 10;
	LPC_TMR32B1->MCR = 0x000;

	/* Start counting */
	LPC_TMR32B1->TCR = 0x1;

	NVIC_EnableIRQ(TIMER_32_1_IRQn);
}

uint16_t timer_read_hi(void) {
	return LPC_TMR32B1->TC >> 16;
}

/* Burn some cycles */
void my_delay(uint16_t msecs) {
	uint32_t end = timer_read() + (uint32_t) msecs * (F_CPU / 1000);
	while ((int32_t) (timer_read() - end) < 0);
}

/*
 * This is a super simple timeout support.  There is a limited number of
 * timeouts that can be set at any given point.  Timeouts that are far
 * enough from now and far enough from other timeouts, should be rather
 * accurate, but there are no guarantees.  The callback may happen a
 * little later than expected, but not sooner than expected.  This is
 * guaranteed, but the "little later" can be rather long in extreme
 * cases.  It will be reasonably short if callback routines are reasonably
 * short.  Timeouts being set need to be within 2^32 / F_CPU / 2 seconds
 * from now, which is about 120 seconds at 16MHz.
 *
 * Callbacks currently run with interrupts enabled in order let the timer
 * overflow do its job, but if the callbacks don't take too long, they
 * could be given the comfort of interrupts disabled which would simplify
 * the code here a little. (revisit, check if still true -- I think now
 * we let the callback re-enable interrupts if it so desires)
 */
#define MAX_TIMEOUTS	16

static struct timeout_s {
	uint32_t when;
	void (*callback)(void);
	int next;
} timeouts[MAX_TIMEOUTS];
static int next = 0xff;
static int k = 0;

static void update_timeouts(void);

void set_timeout(uint32_t when, void (*callback)(void)) {
	uint32_t flags = irq_save();
	int i, *j;

	for (i = next, j = &next; i != 0xff; j = &timeouts[i].next, i = *j)
		if ((uint32_t) (when - timeouts[i].when) >> 31)
			break;
	for (; timeouts[k].callback; k = (k + 1) & (MAX_TIMEOUTS - 1));
	timeouts[k].when = when;
	timeouts[k].callback = callback;
	timeouts[k].next = i;
	*j = k;

	if (j == &next)
		update_timeouts();

	irq_restore(flags);
}

static volatile int updating = 0;
static volatile int updated;

void timer32_1_isr(void) {
	uint32_t now;

	__disable_irq();
	/* Clear the interrupt */
	LPC_TMR32B1->IR = 0x01;

#if 0
	updating = 1;
#endif
	//updated = 1;

	do {
		void (*cb)(void) = timeouts[next].callback;
		timeouts[next].callback = NULL;
		next = timeouts[next].next;
		updated = 0;

		//__enable_irq();
		cb();
		__disable_irq();

		now = timer_read();
	} while (next != 0xff && (uint32_t) (timeouts[next].when - now) >> 31);

	//updating = 0;
	if (!updated)
		update_timeouts();

	__enable_irq();
}

/*
 * Another assumption that we make, but which really depends on factors
 * like the compiler flags:
 *
 * setting the timer compare register to a new value will take less than
 * MIN_DELAY cycles.
 */
#define MIN_DELAY 32

/* Interrupts disabled here */
static void update_timeouts(void) {
	uint32_t now, then;

	//if (unlikely(updating))
	//	return;

	LPC_TMR32B1->MCR = 0x000;

	updated = 1;
	if (next == 0xff)
		return;

	/*
	 * If the desired value is in the past or very close to now,
	 * we make it now + MIN_DELAY cycles to avoid any race
	 * conditions.  The hope is that this function will not take
	 * longer than MIN_DELAY cycles.
	 */
	now = LPC_TMR32B1->TC;
	then = timeouts[next].when;
	if (unlikely(then - now + 512 < 512 + MIN_DELAY))
		then = now + MIN_DELAY;
	LPC_TMR32B1->MR0 = then;
	LPC_TMR32B1->MCR = 0x001;
}
