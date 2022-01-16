#include <stdio.h>
#include "pigun.h"
#include "pigun_bt.h"


// detect peaks from the camera buffer data
// peak data goes into the global var  Peak* pigun_peaks
// the peaks should be in the order required by the calculator function
// returns the number of detected peaks
int pigun_detect(unsigned char* data) {

	// clear the peak data
	memset(pigun_peaks, 0, sizeof(Peak) * 4);

	// detect the peaks...



	return 0;
}


// This function computes the aiming coordinates from the peaks
// Coordinates have to go in global_pigun_report(.x and .y)
// and they are 16-bit signed integers (-32768, +32767)
void pigun_calculate_aim() {

	// calculate the aim position with normalised coordinates (float 0-1)
	float x = 0.5f, y = 0.5f;

	// ...

	// including the calibration...
	x += pigun_aimOffset_x;
	y += pigun_aimOffset_y;


	global_pigun_report.x = (short)((2 * x - 1) * 32767);
	global_pigun_report.y = (short)((2 * y - 1) * 32767);
}



// Setup the buffer to show in the preview window.
// source is the original frame buffer from the camera
// output is the buffer to send to preview/input port
void pigun_preview(MMAL_BUFFER_HEADER_T *output, MMAL_BUFFER_HEADER_T *source) {

	// copy the Y channel as is
	memcpy(output->data, source->data, PIGUN_NPX); // copy only Y

	// set UV to 0
	memset(&output->data[PIGUN_NPX], 128, PIGUN_NPX / 2); // reset U/V channels to single color

}
