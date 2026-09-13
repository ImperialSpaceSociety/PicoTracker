#ifndef FIX_H_
#define FIX_H_

#include <stdint.h>

/* convert decimal degrees to degrees in uBlox output format (scaled by 10^7) */
#define COORD_UBX(x) ((int32_t) (x * 10000000.0f))

struct gps_fix {
	uint8_t type;		/* NAV-PVT fix type */
	uint8_t flags;		/* NAV-PVT fix status flags */
	uint8_t num_svs;	/* number of satellites used for solution, range 0 .. 19 */
	uint16_t year;		/* year, range 0 to 65535 */
	uint8_t month;		/* month, range 1 to 12 */
	uint8_t day;		/* day, range 1 to 31 */
	uint8_t hour;		/* hour, range 0 to 23 */
	uint8_t min;		/* minute, range 0 to 59 */
	uint8_t sec;		/* second, range 0 to 59 */
	int32_t lat;		/* latitude in deg * 10^7, range -90 .. +90 * 10^7 */
	int32_t lon;		/* longitude in deg * 10^7, range -180 .. +180 * 10^7 */
	uint16_t alt;		/* altitude in m, range 0m, up to ~40000m, clamped */
	uint16_t voltage_radio;	/* voltage in mV, range 0 .. 3300mV */
	uint16_t op_status;	/* packed GPS diagnostic status */
	int16_t temp_radio;     /* tracker interval temperature in degrees C, range -100 .. 100 */
};

#endif
