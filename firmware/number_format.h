#ifndef NUMBER_FORMAT_H_
#define NUMBER_FORMAT_H_

#include <stdint.h>

void i32toa(uint32_t in, uint8_t len, volatile char *out);
void i16toa(uint16_t in, uint8_t len, volatile char *out);
uint8_t i16toav(uint16_t in, volatile char *out);
void i16tox(uint16_t x, char *out);

#endif /* NUMBER_FORMAT_H_ */
