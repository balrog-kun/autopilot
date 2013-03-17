#include "LPC13xx.h"		/* LPC13xx Peripheral Registers */

/*
 * Function twi_init
 * Desc     readys twi pins and sets twi bitrate
 * Input    none
 * Output   none
 */
void twi_init(void) {
	LPC_IOCON->PIO0_4 = 0x01;
	LPC_IOCON->PIO0_5 = 0x01;

	LPC_SYSCON->SYSAHBCLKCTRL |= 1 << 5;
	LPC_SYSCON->PRESETCTRL |= 2;

	/* 12MHz I2C_PCLK and 400kHz bit rate */
	LPC_I2C->SCLH = 60; // 10 for fast-mode i2c
	LPC_I2C->SCLL = 60; // 20 for fast-mode i2c

	LPC_I2C->CONSET = 1 << 6; /* Enable I2C */
}

int i2c_send_bytes(uint8_t address, int count, const uint8_t *buf) {
	LPC_I2C->CONSET = 1 << 5; /* Start txing in master mode */
	while (!(LPC_I2C->CONSET & (1 << 3))); /* Wait for interrupt */
	if ((LPC_I2C->STAT & 0xff) != 0x08)
		goto err;
	LPC_I2C->DAT = address << 1;
	LPC_I2C->CONCLR = (1 << 3) | (1 << 5); /* Clear the flags */
	while (!(LPC_I2C->CONSET & (1 << 3))); /* Wait for interrupt */
	if ((LPC_I2C->STAT & 0xff) != 0x18)
		goto err;
	while (count) {
		LPC_I2C->DAT = *buf ++;
		LPC_I2C->CONCLR = 1 << 3; /* Clear the interrupt */
		count --;
		while (!(LPC_I2C->CONSET & (1 << 3))); /* Wait for interrupt */
		if ((LPC_I2C->STAT & 0xff) != 0x28)
			goto err;
	}
	LPC_I2C->CONSET = 1 << 4; /* Send a STOP */
	LPC_I2C->CONCLR = 1 << 3; /* Clear the interrupt */
	return 1;
err:
	LPC_I2C->CONSET = 1 << 4; /* Send a STOP */
	LPC_I2C->CONCLR = 1 << 3; /* Clear the interrupt */
	return 0;
}

int i2c_send_byte(uint8_t address, uint8_t byte) {
	return i2c_send_bytes(address, 1, &byte);
}

int i2c_request_bytes(uint8_t address, int count, uint8_t *buf) {
	LPC_I2C->CONSET = 1 << 5; /* Start txing in master mode */
	while (!(LPC_I2C->CONSET & (1 << 3))); /* Wait for interrupt */
	if ((LPC_I2C->STAT & 0xff) != 0x08)
		goto err;
	LPC_I2C->DAT = (address << 1) | 1;
	LPC_I2C->CONCLR = (1 << 3) | (1 << 5); /* Clear the flags */
	while (!(LPC_I2C->CONSET & (1 << 3))); /* Wait for interrupt */
	if ((LPC_I2C->STAT & 0xff) != 0x40)
		goto err;
	LPC_I2C->CONSET = 1 << 2; /* ACK every byte */
	while (count) {
		if (count == 1)
			LPC_I2C->CONCLR = 1 << 2; /* but NAK the last byte */
		LPC_I2C->CONCLR = 1 << 3; /* Clear the interrupt */
		while (!(LPC_I2C->CONSET & (1 << 3))); /* Wait for interrupt */
		if ((LPC_I2C->STAT & 0xf0) != 0x50)
			goto err;
		count --;
		*buf ++ = LPC_I2C->DAT;
		if ((LPC_I2C->STAT & 0xff) != 0x50)
			goto err;
	}
err:
	LPC_I2C->CONSET = 1 << 4; /* Send a STOP */
	LPC_I2C->CONCLR = (1 << 2) | (1 << 3); /* Clean up */
	return count == 0;
}
