
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <memory.h>
#include <sysexits.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define VERSION_STRING "v1.3.15"

#include <bcm2835.h>

#include "bcm_host.h"
#include "interface/vcos/vcos.h"

#include "interface/mmal/mmal.h"
#include "interface/mmal/mmal_logging.h"
#include "interface/mmal/mmal_buffer.h"
#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_util_params.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_connection.h"

#include <semaphore.h>

#include <X11/Xlib.h>
#include <X11/Xutil.h>


#ifndef PIGUN
#define PIGUN


// Standard port setting for the camera component
#define MMAL_CAMERA_PREVIEW_PORT 0
#define MMAL_CAMERA_VIDEO_PORT 1
#define MMAL_CAMERA_CAPTURE_PORT 2

// Camera acquisiton settings: This captures the full FoV with the V2 camera
// module
#define PIGUN_CAM_X 1640
#define PIGUN_CAM_Y 1232
#define PIGUN_FPS 40

// Camera output settings: these are ~1/4th of the camera acquisition. The
// vertical resolution needs to be a multiple of 16, and the horizontal
// resolution needs to be a multiple of 32!
#define PIGUN_RES_X 416 // 410 // 320
#define PIGUN_RES_Y 320 // 308 // 180

// total number of pixels in the buffer - has to be the product of the previous 2
#define PIGUN_NPX 133120 // 126280 // 57600 // 230400

#define PI 3.14159265
extern MMAL_PORT_T *port_prv_in1;


// maximum distance in pixels for peak matching
#define MAXPEAKDIST 5




// Button definitions - GPIO pins
#define PIN_TRG RPI_V2_GPIO_P1_11	// trigger goes on PIN #11 == GPIO 17 (should be this https://pinout.xyz/pinout/pin11_gpio17)
#define PIN_RLD RPI_V2_GPIO_P1_13	// reload  goes on PIN #13 == GPIO 27 (this is the little clip button like in real beretta M9)
#define PIN_AX3 RPI_V2_GPIO_P1_29	// AUX1 BT goes on PIN #29 == GPIO 5  (this should be connected  under the handle)
#define PIN_AX4 RPI_V2_GPIO_P1_32	// AUX2 BT goes on PIN #32 == GPIO 12  
#define PIN_AX5 RPI_V2_GPIO_P1_36	// AUX3 BT goes on PIN #36 == GPIO 16
#define PIN_AX6 RPI_V2_GPIO_P1_38	// AUX4 BT goes on PIN #38 == GPIO 20
#define PIN_AX7 RPI_V2_GPIO_P1_40	// AUX5 BT goes on PIN #40 == GPIO 21
#define PIN_CAL RPI_V2_GPIO_P1_15	// calibr  goes on PIN #15 == GPIO 22

// GPIO for LEDs
#define PIN_OUT_ERR RPI_V2_GPIO_P1_16 // maybe use a red led?
#define PIN_OUT_CAL RPI_V2_GPIO_P1_18
#define PIN_OUT_SOL RPI_V2_GPIO_P1_22




// Describes a peak in the camera image
typedef struct Peak Peak;
struct Peak {
   float row;
   float col;
   float maxI;
   float total;
   float tRow, tCol;
   // 0->struct is unused, 1->peak found
   unsigned short found;
};




typedef struct PigunAimPoint PigunAimPoint;
struct PigunAimPoint {
	float x, y;
};

// these are only used in pigun.c, not in the custom detection module
//extern int pigun_state;
//extern int pigun_button_pin[8];

extern Peak* pigun_peaks;

/// <summary>
/// This is the aiming point in normalised coords (0-1 on both x,y), before calibration is applied
/// </summary>
extern PigunAimPoint pigun_aim_norm;
extern PigunAimPoint pigun_cal_topleft;
extern PigunAimPoint pigun_cal_lowright;


// these function define how detection and aiming works
#ifdef __cplusplus
extern "C" {
#endif

int pigun_detect(unsigned char* data);
void pigun_calculate_aim();
void pigun_preview(MMAL_BUFFER_HEADER_T* output, MMAL_BUFFER_HEADER_T* source); // only used if PIGUN_PREVIEW is defined

#ifdef __cplusplus
}
#endif


#ifdef PIGUN_MOUSE
extern Display* displayMain;
extern Screen* screen;
extern int screenWidth;
extern int screenHeight;

	#ifdef __cplusplus
	extern "C" {
	#endif
		void mouseMove(float x, float y);
	#ifdef __cplusplus
	}
	#endif
#endif



/*
void pigun_camera_setup(MMAL_COMPONENT_T *cam, unsigned int resx, unsigned int resy);

void pigun_port_setformat(MMAL_PORT_T *port, unsigned int resx, unsigned int resy);
void pigun_port_setformat_buffered(MMAL_PORT_T *port, unsigned int resx, unsigned int resy, unsigned int nbuf);


void pigun_video_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer);

void pigun_compute_4corners(Peak* ABCD, float aspectRatio, float* CMatrix);
*/

// HELPER FUNCTIONS
#ifdef __cplusplus
extern "C" {
#endif
	int pigun_camera_gains(MMAL_COMPONENT_T *camera, int analog_gain, int digital_gain);
	int pigun_camera_awb(MMAL_COMPONENT_T *camera, int on);
	int pigun_camera_awb_gains(MMAL_COMPONENT_T *camera, float r_gain, float b_gain);
	int pigun_camera_blur(MMAL_COMPONENT_T *camera, int on);
	int pigun_camera_exposuremode(MMAL_COMPONENT_T *camera, int on);
#ifdef __cplusplus
}
#endif




#endif
