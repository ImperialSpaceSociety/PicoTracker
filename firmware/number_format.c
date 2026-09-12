/*
 * helper functions - string conversion
 *
 * Stefan Biereigel
 *
 */

#include <inttypes.h>
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
	uint8_t i;
	for (i = len; i > 0; i--) {
		*(out + i - 1) = (in % 10) + '0';
		in /= 10;
	}
}

/* i16toa
 * 16 bit number to variable-length output char
 *
 * returns:	length of string
 */
uint8_t i16toav(uint16_t in, volatile char *out) {
	uint16_t mult = 10000;
	uint8_t cnt = 0;
	uint8_t start = 0;
	uint8_t len = 0;
	if (in == 0) {
		*out = '0';
		return 1;
	}

	while(mult > 0) {
		if (in >= mult) {
			in = in - mult;
			cnt++;
			start = 1;
		} else {
			*out = cnt + '0';
			cnt = 0;
			mult /= 10;
			if (start) {
				out++;
				len++;
			}
		}
	}

	return len;
}

/* i16tox
 * 16 bit number to hexadecimal char representation
 *
 * writes 4 chars to the output pointer
 */
void i16tox(uint16_t x, char *out) {
	uint8_t i;
	uint8_t tmp;
	for (i = 0; i < 4; i++) {
		tmp = (uint8_t) ((x >> (4*i)) & 0x000f);
		if (tmp < 10) {
			*(out+3-i) = '0' + tmp;
		} else {
			*(out+3-i) = 'A' + tmp - 10;
		}
	}
}

