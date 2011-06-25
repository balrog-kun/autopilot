/*
 * TWI/I2C library for Wiring & Arduino
 */

#ifndef TWI_FREQ
# define TWI_FREQ 100000L
#endif

#ifndef TWI_BUFFER_LENGTH
# define TWI_BUFFER_LENGTH 12
#endif

#define TWI_READY	0
#define TWI_MRX		1
#define TWI_MTX		2
#define TWI_SRX		3
#define TWI_STX		4

void twi_init(void);
void i2c_send_byte(uint8_t address, uint8_t byte);
void i2c_request_bytes(uint8_t address, uint8_t count, uint8_t *out);
