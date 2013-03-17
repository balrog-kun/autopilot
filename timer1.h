/*
 * 32-bit timer used for timekeeping as well as arbitrary timeouts.
 *
 * Licensed under AGPLv3.
 */

#include "LPC13xx.h"		/* LPC13xx Peripheral Registers */

void timer_init(void);
void my_delay(uint16_t msecs);
void set_timeout(uint32_t when, void (*callback)(void));
uint16_t timer_read_hi(void);
static inline uint32_t timer_read(void) {
	return LPC_TMR32B1->TC;
}

#define likely(x)	__builtin_expect((x), 1)
#define unlikely(x)	__builtin_expect((x), 0)

#define cli()		__disable_irq()
#define sei()		__enable_irq()
