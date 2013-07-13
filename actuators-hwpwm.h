/*
 * Generates the signal that drives RC actuators (motors and servos).
 *
 * Licensed under AGPLv3.
 */

/* One pin, uses PIO1_9 aka 16-bit timer 1 match 0 aka USB conn. */
volatile uint16_t actuators_hwpwm[2];

/* Try every ~12ms?  If servo gets hot use ~20ms */
#define ACT_CYCLE 0.012
#define ACT_PRESCALER ((int) (ACT_CYCLE * F_CPU / 65536 + 1.0))
#define ACT_SC_CYCLE ((int) (ACT_CYCLE * F_CPU / ACT_PRESCALER))

static void die(void);
static inline void actuators_hwpwm_init(int devices) {
	if (devices > 2)
		die();

	LPC_IOCON->PIO1_9 = 0x001;
	LPC_SYSCON->SYSAHBCLKCTRL |= 1 << 8;

	LPC_TMR16B1->TCR = 0x0;
	LPC_TMR16B1->CCR = 0x0;
	LPC_TMR16B1->CTCR = 0x0;

	LPC_TMR16B1->PR = ACT_PRESCALER - 1;
	LPC_TMR16B1->MCR = 0x400;
	LPC_TMR16B1->MR3 = ACT_SC_CYCLE;
	LPC_TMR16B1->PWMC = 0x3;
}

void actuators_hwpwm_start(void) {
	LPC_TMR16B1->TCR = 0x1;
}
void actuators_hwpwm_stop(void) {
	LPC_TMR16B1->TCR = 0x0;
}

#define ACT_1MS ((int) (0.001 * F_CPU / ACT_PRESCALER))

static inline void actuator_hwpwm_set(uint8_t target, uint16_t value) {
	actuators_hwpwm[target] = value;
	if (target == 0)
		LPC_TMR16B1->MR0 = ACT_SC_CYCLE - ACT_1MS - ACT_1MS / 8 -
			((value * ACT_1MS) >> 16);
	else if (target == 1)
		LPC_TMR16B1->MR1 = ACT_SC_CYCLE - ACT_1MS - ACT_1MS / 8 -
			((value * ACT_1MS) >> 16);
}
