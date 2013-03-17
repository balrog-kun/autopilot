#include <stdio.h>
#include <stdint.h>
#include <math.h>

static uint16_t ccsqrt(uint32_t x) {
	/* This is mainly written for gcc on arm thumb2, on other architectures
	 * using uin16_t may generate better code.  When initialising the
	 * loop we could just set res = 0, i = 1 << 16 and we would get
	 * shorter code but potentially slower.  */
	uint32_t ret;
	uint32_t i;

	asm volatile ("clz  %0, %1" : "=r" (i) : "r" (x));
	ret = 1 << ((31 - i) >> 1);
	for (i = ret >> 1; i; i >>= 1) {
		uint32_t t = res | i;
		if (x >= t * t)
			res |= i;
	}

	return res;
}

int main(void) {
	uint32_t i;

	for (i = 0; i < 0xffff0000; i += 1)
		if (ccsqrt(i) != (uint16_t) sqrt(i))
			printf("%i %i\n", ccsqrt(i), (int) sqrt(i));
	return 0;
}
