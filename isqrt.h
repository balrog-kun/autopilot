#ifndef _ISQRT_H
#define _ISQRT_H

#include <inttypes.h>

// coded in assembler file
extern uint16_t isqrt32(uint32_t n);
extern uint8_t  isqrt16(uint16_t n);
extern uint16_t ihypot(int16_t x, int16_t y);

#endif // _ISQRT_H
