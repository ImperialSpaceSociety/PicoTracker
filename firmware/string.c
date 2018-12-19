/*
 * helper functions - string conversion
 *
 * Stefan Biereigel
 *
 */

#include <inttypes.h>

/* atoid32
 * converts a fixed-length input string to 32bit integers, 
 * containing respective integer and decimal parts of input string
 */
void atoid32(char *string, uint8_t len, uint32_t *integer, uint32_t *decimal) {
	uint8_t j;
	uint32_t mult = 1;
	uint8_t int_port = 0;

	*integer = 0;
	*decimal = 0;
	for (j = 0; j < len; j++) {
		if (*(string + len - j - 1) == '.') {
			mult = 1;
			int_port = 1;
		} else {
			if (int_port) {
				*integer += mult * (*(string + len - j - 1) - '0');
			} else {
				*decimal += mult * (*(string + len - j - 1) - '0');
			}
			mult *= 10;
		}
	}
}

/* atod32
 * converts a fixed-length input string to 32bit integer, representing its decimal part,
 */
void atod32(char *string, uint8_t len, uint32_t *decimal) {
	uint8_t j;
	uint32_t mult = 1;

	*decimal = 0;
	for (j = 0; j < len; j++) {
		if (*(string + len - j - 1) == '.') {
			return;
		} else {
			*decimal += mult * (*(string + len - j - 1) - '0');
			mult *= 10;
		}
	}
}

/* atoi32
 * converts a fixed-length input string to 32bit integer, stops at decimal points,
 * so only integer part is returned
 *
 * the number MUST include a decimal portion (XX.XX)
 */
void atoi32(volatile char *string, uint8_t len, uint32_t *integer) {
	uint8_t j;
	uint32_t mult = 1;
	uint8_t start = 0;

	*integer = 0;
	for (j = 0; j < len; j++) {
		if (*(string + len - j - 1) == '.') {
			start = 1;
		} else {
			if (start) {
				*integer += mult * (*(string + len - j - 1) - '0');
				mult *= 10;
			}
		}
	}
}

/* atoi16
 * converts a fixed-length input string to 16bit integer, stops at decimal points,
 * so only integer part is returned
 *
 * the number MUST include a decimal portion (XX.XX)
 */
void atoi16(volatile char *string, uint8_t len, uint16_t *integer) {
	uint8_t j;
	uint16_t mult = 1;
	uint8_t start = 0;

	*integer = 0;
	for (j = 0; j < len; j++) {
		if (*(string + len - j - 1) == '.') {
			start = 1;
		} else {
			if (start) {
				*integer += mult * (*(string + len - j - 1) - '0');
				mult *= 10;
			}
		}
	}
}

/* atoi8
 * converts a fixed-length input string to 8bit integer, stops at decimal points,
 * so only integer part is returned
 *
 * the input MUST NOT include a decimal portion
 */
void atoi8(volatile char *string, uint8_t len, uint8_t *integer) {
	uint8_t j;
	uint32_t mult = 1;

	*integer = 0;
	for (j = 0; j < len; j++) {
		*integer += mult * (*(string + len - j - 1) - '0');
		mult *= 10;
	}
}

/* i32toa
 * 32 bit number to fixed-length output char
 */
void i32toa(uint32_t in, uint8_t len, volatile char *out) {
	uint8_t i;
	uint32_t mult = 1;
	for (i = len; i > 0; i--) {
		*(out + i - 1) = ((in % (mult*10)) / mult) + '0';
		mult *= 10;
	}
}

/* i16toa
 * 16 bit number to fixed-length output char
 */
void i16toa(uint16_t in, uint8_t len, volatile char *out) {
	uint8_t i;
	uint16_t mult = 1;
	for (i = len; i > 0; i--) {
		*(out + i - 1) = ((in % (mult*10)) / mult) + '0';
		mult *= 10;
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

