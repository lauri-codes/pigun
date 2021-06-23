
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>

#include "pigun_bt.h"
#include <math.h>


pigun_report_t global_pigun_report;
float phase = 0;
unsigned char bt = 0;

void* pigun_core() {


	printf("pigun core setup...\n");
	global_pigun_report.x = 0;
	global_pigun_report.y = 0;
	global_pigun_report.buttons = 0;

	sleep(2);

	printf("pigun core starting acquisition...\n");
	while (42) {

		// processing should go here
		usleep(20);

		// come up with some coordinates
		//global_pigun_report.x = (short)((2*((float)rand() / RAND_MAX)-1)*32767);
		//global_pigun_report.y = (short)((2 * ((float)rand() / RAND_MAX) - 1) * 32767);
		
		global_pigun_report.x = (short)(cosf(phase) * 32767);
		global_pigun_report.y = (short)(sinf(phase) * 32767);

		phase += 0.0002;
		if (phase >= 2 * M_PI) {
			phase -= 2 * M_PI;
			global_pigun_report.buttons = (uint8_t)bt;
			bt++;
			printf("cycle!\n");
		}
	}
}
