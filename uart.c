/*
 * Serial.
 *
 * Licensed under AGPLv3.
 */

#include "LPC13xx.h"		/* LPC13xx Peripheral Registers */
#include "uart.h"
#include "timer1.h"

#ifndef NULL
# define NULL 0
#endif

#define TX_INTERRUPT	1	/* 0 if TX uses polling, 1 interrupt driven. */

#define IER_RBR		0x01
#define IER_THRE	0x02
#define IER_RLS		0x04

#define IIR_PEND	0x01
#define IIR_RLS		0x03
#define IIR_RDA		0x02
#define IIR_CTI		0x06
#define IIR_THRE	0x01

#define LSR_RDR		0x01
#define LSR_OE		0x02
#define LSR_PE		0x04
#define LSR_FE		0x08
#define LSR_BI		0x10
#define LSR_THRE	0x20
#define LSR_TEMT	0x40
#define LSR_RXFE	0x80

static void uartinit(uint32_t baudrate, uint8_t rtscts) {
	/* IO configuration */
	if (rtscts) {
		LPC_IOCON->PIO0_7 = 0x01;	/* UART CTS */
		LPC_IOCON->PIO1_5 = 0x01;	/* UART RTS */
	}
	LPC_IOCON->PIO1_6 = 0x01;	/* UART RXD */
	LPC_IOCON->PIO1_7 = 0x01;	/* UART TXD */

	/* Enable UART clock */
	LPC_SYSCON->UARTCLKDIV = 1;	/* divided by 1 */
	LPC_SYSCON->SYSAHBCLKCTRL |= 1 << 12;

	LPC_UART->LCR = 0x83;		/* 8 bits, no Parity, 1 Stop bit */
	//reg = LPC_SYSCON->UARTCLKDIV;
	//fdiv = (((F_CPU / LPC_SYSCON->SYSAHBCLKDIV) / reg) / 16) / baudrate;

	//LPC_UART->DLM = fdiv >> 8;
	//LPC_UART->DLL = fdiv >> 0;

	/* Copied from 12.6.15.1.2 in the manual */
	/* TODO: implement the auto FDR algorithm */
	LPC_UART->DLM = 0x00;
	LPC_UART->DLL = 0x04;
	LPC_UART->FDR = 0x85;

	LPC_UART->LCR = 0x03;		/* DLAB = 0 */
	LPC_UART->FCR = 0x07;		/* Enable and reset TX and RX FIFO. */

	/* Read to clear the line status. */
	/* not needed reg = LPC_UART->LSR; */

	/* Ensure a clean start, no data in either TX or RX FIFO. */
	while ((LPC_UART->LSR & (LSR_THRE | LSR_TEMT)) !=
			(LSR_THRE | LSR_TEMT));
	while (LPC_UART->LSR & LSR_RDR)
		(void) LPC_UART->RBR;	/* Dump data from RX FIFO */

	/* Enable the UART Interrupts */
	NVIC_EnableIRQ(UART_IRQn);
	/* Unmask */
#if TX_INTERRUPT
	LPC_UART->IER = IER_RBR | IER_THRE;
#else
	LPC_UART->IER = IER_RBR;
#endif
}

void serial_init(void) {
	uartinit(115200, 0);
}

#if TX_INTERRUPT
#define TX_LEN 64
uint8_t tx_buf[TX_LEN], tx_start = 0, tx_end = 0;
static void serial_tx_run(void) {
	while ((LPC_UART->LSR & LSR_THRE) && tx_start != tx_end)
		LPC_UART->THR = tx_buf[tx_start ++ & (TX_LEN - 1)];
}
#endif

void serial_write1(char ch) {
#if TX_INTERRUPT
	cli(); /* In case LSR changes after the check (FIXME: save flags) */
	if (tx_start == tx_end && (LPC_UART->LSR & LSR_THRE))
		LPC_UART->THR = ch;
	else
		tx_buf[tx_end ++ & (TX_LEN - 1)] = ch;
	sei();
#else
	while (!(LPC_UART->LSR & LSR_THRE));
	LPC_UART->THR = ch;
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
	//serial_write1('\r');
	serial_write1('\n');
}

void serial_write_str(const char *str) {
	while (*str)
		serial_write1(*str ++);
}

#if 1
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
#endif

static void (*ch_handler)(char ch) = NULL;

void serial_set_handler(void (*handler)(char ch)) {
	ch_handler = handler;
}

void uart_isr(void) {
	uint8_t iir, lsr;
	uint8_t dummy = dummy;

	iir = LPC_UART->IIR;

	iir >>= 1;		/* skip pending bit in IIR */
	iir &= 0x07;		/* check bit 1~3, interrupt identification */
	if (iir == IIR_RLS) {	/* Receive Line Status */
		lsr = LPC_UART->LSR;
		/* Receive Line Status */
		if (lsr & (LSR_OE | LSR_PE | LSR_FE | LSR_RXFE | LSR_BI)) {
			/* There are errors or break interrupt */
			/* Reading the LSR will clear the interrupt */
			dummy = LPC_UART->RBR;	/* Dummy read on RX to clear 
						   interrupt, then bail out */
			return;
		}
		if (lsr & LSR_RDR) {	/* Receive Data Ready */
			/* If no error on RLS, normal ready, save into the
			 * data buffer. */
			/* Note: reading RBR will clear the interrupt */
			char ch = LPC_UART->RBR;
			if (ch_handler)
				ch_handler(ch);
		}
	} else if (iir == IIR_RDA) {	/* Receive Data Available */
		/* Receive Data Available */
		char ch = LPC_UART->RBR;
		if (ch_handler)
			ch_handler(ch);
#if TX_INTERRUPT
	} else if (iir == IIR_THRE) {	/* Transmit holding register empty */
		serial_tx_run();
#endif
	}
}
