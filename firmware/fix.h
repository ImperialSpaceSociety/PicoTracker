#ifndef FIX_H_
#define FIX_H_

#include <inttypes.h>

/* convert decimal degrees to degrees in uBlox output format (scaled by 10^7) */ 
#define COORD_UBX(x) ((int32_t) (x * 10000000.0f))

struct gps_fix {
	uint8_t type;		/* type of fix (validity) */
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
	uint16_t voltage_bat;	/* battery voltage in mV, range 0 .. 3300mV */
	uint16_t voltage_sol;	/* solar voltage in mV, range 0 .. 3300mV */
	int16_t temperature_int;/* tracker interval temperature in °C, range -100 .. 100 */
};

#endif