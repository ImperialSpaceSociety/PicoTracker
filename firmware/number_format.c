/*
 * helper functions - string conversion
 *
 * Stefan Biereigel
 *
 */

#include <stdint.h>
#include "number_format.h"

/* i32toa
 * 32 bit number to fixed-length output char
 */
void i32toa(uint32_t in, uint8_t len, volatile char *out) {
	static const uint32_t powers_of_ten[10] = {
		1000000000UL, 100000000UL, 10000000UL, 1000000UL, 100000UL,
		10000UL, 1000UL, 100UL, 10UL, 1UL
	};
	uint8_t i;
	uint8_t offset = (uint8_t)(10U - len);

	for (i = 0; i < len; i++) {
		uint8_t digit = 0;
		uint32_t divisor = powers_of_ten[offset + i];
		while (in >= divisor) {
			in -= divisor;
			digit++;
		}
		out[i] = (char)(digit + '0');
	}
}

/* i16toa
 * 16 bit number to fixed-length output char
 */
void i16toa(uint16_t in, uint8_t len, volatile char *out) {
	i32toa((uint32_t)in, len, out);
}

/* i16toa
 * 16 bit number to variable-length output char
 *
 * returns:	length of string
 */
uint8_t i16toav(uint16_t in, volatile char *out) {
	char digits[5];
	uint8_t first = 0U;
	uint8_t i;

	i16toa(in, 5U, digits);
	while (first < 4U && digits[first] == '0') first++;
	for (i = first; i < 5U; i++) out[i - first] = digits[i];
	return (uint8_t)(5U - first);
}

/* i16tox
 * 16 bit number to hexadecimal char representation
 *
 * writes 4 chars to the output pointer
 */
void i16tox(uint16_t x, char *out) {
	static const char hex[] = "0123456789ABCDEF";
	uint8_t i;
	for (i = 0U; i < 4U; i++) {
		out[3U - i] = hex[x & 0x0FU];
		x >>= 4;
	}
}
