#ifndef STRING_H_
#define STRING_H_

void atoi32(volatile char *string, uint8_t len, uint32_t *integer);
void atoi16(volatile char *string, uint8_t len, uint16_t *integer);
void atoi8(volatile char *string, uint8_t len, uint8_t *integer);
void atoid32(char *string, uint8_t len, uint32_t *integer, uint32_t *decimal);
void atod32(char *string, uint8_t len, uint32_t *decimal);
void i32toa(uint32_t in, uint8_t len, volatile char *out);
void i16toa(uint16_t in, uint8_t len, volatile char *out);
uint8_t i16toav(uint16_t in, volatile char *out);
void i16tox(uint16_t x, char *out);

#endif
