/*
 * Serial.
 *
 * Licensed under AGPLv3.
 */

void serial_init(void);
void serial_write1(char ch);
void serial_write_hex16(uint16_t val);
void serial_write_hex32(uint32_t val);
void serial_write_dec8(uint8_t val);
void serial_write_dec32(uint32_t val);
void serial_write_fp32(int32_t val, uint32_t unit);
void serial_write_eol(void);
void serial_write_str(const char *str);
void serial_set_handler(void (*handler)(char ch));
