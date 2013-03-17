/*
 * TWI/I2C library for Wiring & Arduino
 */

void twi_init(void);
int i2c_send_byte(uint8_t address, uint8_t byte);
int i2c_send_bytes(uint8_t address, int count, const uint8_t *buf);
int i2c_request_bytes(uint8_t address, int count, uint8_t *buf);
