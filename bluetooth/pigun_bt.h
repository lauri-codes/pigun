#include <stdint.h>

#ifndef PIGUN_BT
#define PIGUN_BT


// data container for the HID joystick report
typedef struct pigun_report_t pigun_report_t;
struct pigun_report_t {

	short x;
	short y;
	uint8_t buttons;
};

extern pigun_report_t global_pigun_report;

#ifdef __cplusplus
extern "C" {
#endif
	void* pigun_cycle(void*);
#ifdef __cplusplus
}
#endif


#endif
