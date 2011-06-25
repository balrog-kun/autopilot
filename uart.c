/*
 * Serial.
 *
 * Licensed under AGPLv3.
 */

#include <avr/io.h>
#include <avr/interrupt.h>

#include "timer1.h"

#ifndef NULL
# define NULL 0
#endif

void serial_init(void) {
	uint16_t baud = 8; /* 115200 at 16 MHz, TODO: use F_CPU */

	UBRR0H = baud >> 8;
	UBRR0L = baud & 255;
	UCSR0B = 0x98; /*(1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);*/
	UCSR0C = 0x06; /*(3 << UCSZ00);*/
}

static const long int timeout = 10000;
void serial_write1(char ch) {
	int count = 0;
#if defined(__AVR_ATmega128__) || defined(__AVR_ATmega1280__)
	while (!(UCSR0A & _BV(UDRE0)) && ++ count < timeout);
	UDR0 = ch;
	count = 0;
	while (!(UCSR1A & _BV(UDRE1)) && ++ count < timeout);
	UDR1 = ch;
#elif defined(__AVR_ATmega168__)  || defined(__AVR_ATmega328P__)
	while (!(UCSR0A & _BV(UDRE0)) && ++ count < timeout);
	UDR0 = ch;
#else
	/* m8,16,32,169,8515,8535,163 */
	while (!(UCSRA & _BV(UDRE)) && ++ count < timeout);
	UDR = ch;
#endif
}

static const char to_hex[16] = {
	'0', '1', '2', '3', '4', '5', '6', '7',
	'8', '9', 'a', 'b', 'c', 'd', 'e', 'f',
};

void serial_write_hex16(uint16_t val) {
	serial_write1(' ');
	serial_write1('0');
	serial_write1('x');
	serial_write1(to_hex[(val >> 12) & 15]);
	serial_write1(to_hex[(val >>  8) & 15]);
	serial_write1(to_hex[(val >>  4) & 15]);
	serial_write1(to_hex[(val >>  0) & 15]);
}

void serial_write_hex32(uint32_t val) {
	serial_write1(' ');
	serial_write1('0');
	serial_write1('x');
	serial_write1(to_hex[(val >> 28) & 15]);
	serial_write1(to_hex[(val >> 24) & 15]);
	serial_write1(to_hex[(val >> 20) & 15]);
	serial_write1(to_hex[(val >> 16) & 15]);
	serial_write1(to_hex[(val >> 12) & 15]);
	serial_write1(to_hex[(val >>  8) & 15]);
	serial_write1(to_hex[(val >>  4) & 15]);
	serial_write1(to_hex[(val >>  0) & 15]);
}

void serial_write_eol(void) {
	serial_write1('\r');
	serial_write1('\n');
}

void serial_write_str(const char *str) {
	while (*str)
		serial_write1(*str ++);
}

void serial_write_dec8(uint8_t val) {
	serial_write1(' ');
	if (val > 99)
		serial_write1(val / 100 + '0');
	if (val > 9)
		serial_write1((val / 10) % 10 + '0');
	serial_write1(val % 10 + '0');
}

static void serial_write_dec32_nosp(uint32_t val) {
	if (val > 999999999)
		serial_write1(val / 1000000000 + '0');
	if (val > 99999999)
		serial_write1((val / 100000000) % 10 + '0');
	if (val > 9999999)
		serial_write1((val / 10000000) % 10 + '0');
	if (val > 999999)
		serial_write1((val / 1000000) % 10 + '0');
	if (val > 99999)
		serial_write1((val / 100000) % 10 + '0');
	if (val > 9999)
		serial_write1((val / 10000) % 10 + '0');
	if (val > 999)
		serial_write1((val / 1000) % 10 + '0');
	if (val > 99)
		serial_write1((val / 100) % 10 + '0');
	if (val > 9)
		serial_write1((val / 10) % 10 + '0');
	serial_write1(val % 10 + '0');
}

void serial_write_dec32(uint32_t val) {
	serial_write1(' ');
	serial_write_dec32_nosp(val);
}

void serial_write_fp32(int32_t val, uint32_t unit) {
	uint32_t u;

	serial_write1(' ');
	if (val < 0) {
		serial_write1('-');
		val = 0 - val;
	}
	/* Let's hope the /'s and %'s get optimised together */
	serial_write_dec32_nosp(val / unit);
	val = val % unit;

	if (unit > 1 && likely(val))
		serial_write1('.');
	for (u = unit; u > 1 && likely(val); u /= 10) {
		val *= 10;
		serial_write1(val / unit + '0');
		val = val % unit;
	}
}

static void (*ch_handler)(char ch) = NULL;

void serial_set_handler(void (*handler)(char ch)) {
	ch_handler = handler;
}

ISR(USART_RX_vect) {
	uint8_t status = UCSR0A;
	uint8_t ch = UDR0;

	if (!unlikely(status & 0x14) && ch_handler)
		ch_handler(ch);
}
