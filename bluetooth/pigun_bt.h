#include <stdint.h>

#ifndef PIGUN_BT
#define PIGUN_BT

typedef struct pigun_report_t pigun_report_t;
struct pigun_report_t {

	short x;
	short y;
	uint8_t buttons;
};

extern pigun_report_t global_pigun_report;
void* pigun_core();

#endif
