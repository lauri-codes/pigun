
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <memory.h>
#include <sysexits.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define VERSION_STRING "v1.3.15"

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




#ifndef PIGUN
#define PIGUN


// Standard port setting for the camera component
#define MMAL_CAMERA_PREVIEW_PORT 0
#define MMAL_CAMERA_VIDEO_PORT 1
#define MMAL_CAMERA_CAPTURE_PORT 2

// camera acquisiton settings
#define PIGUN_CAM_X 1280
#define PIGUN_CAM_Y 720
#define PIGUN_FPS 90
// camera output settings
#define PIGUN_RES_X 160
#define PIGUN_RES_Y 90
// total number of pixels in the buffer - has to be the product of the previous 2
#define PIGUN_NPX  14400 //57600 // 230400

extern MMAL_PORT_T *port_prv_in1;


// maximum distance in pixels for peak matching
#define MAXPEAKDIST 5

typedef struct Peak Peak;
struct Peak {

   // 0->struct is unused, 1->peak found
   unsigned short found;

   float row, col;
   float total;
   float tRow, tCol;

};

class Vector3D {
    public:
    float x;
    float y;
    float z;
    Vector3D (float x, float y, float z) {
        this->x = x;
        this->y = y;
        this->z = z;
    };
    Vector3D crossProduct(Vector3D other) {
        float newX = y*other.z - z*other.y;
        float newY = -x*other.z + z*other.x;
        float newZ = x*other.y - y*other.x;
        return Vector3(newX, newY, newZ);
    }
};



void pigun_camera_setup(MMAL_COMPONENT_T *cam, unsigned int resx, unsigned int resy);

void pigun_port_setformat(MMAL_PORT_T *port, unsigned int resx, unsigned int resy);
void pigun_port_setformat_buffered(MMAL_PORT_T *port, unsigned int resx, unsigned int resy, unsigned int nbuf);


void pigun_video_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer);



// HELPER FUNCTIONS
int pigun_camera_awb(MMAL_COMPONENT_T *camera, int on);
int pigun_camera_awb_gains(MMAL_COMPONENT_T *camera, float r_gain, float b_gain);
int pigun_camera_blur(MMAL_COMPONENT_T *camera, int on);



/*
/// Capture/Pause switch method
/// Simply capture for time specified
#define WAIT_METHOD_NONE           0
/// Cycle between capture and pause for times specified
#define WAIT_METHOD_TIMED          1
/// Switch between capture and pause on keypress
#define WAIT_METHOD_KEYPRESS       2
/// Switch between capture and pause on signal
#define WAIT_METHOD_SIGNAL         3
/// Run/record forever
#define WAIT_METHOD_FOREVER        4





// Video format information
// 0 implies variable
#define VIDEO_FRAME_RATE_NUM 30
#define VIDEO_FRAME_RATE_DEN 1

// Frames rates of 0 implies variable, but denominator needs to be 1 to prevent div by 0
#define PREVIEW_FRAME_RATE_NUM 0
#define PREVIEW_FRAME_RATE_DEN 1

/// Video render needs at least 2 buffers.
#define VIDEO_OUTPUT_BUFFERS_NUM 3





/// Interval at which we check for an failure abort during capture
#define ABORT_INTERVAL 100 // ms

typedef struct RASPIVIDYUV_STATE_S RASPIVIDYUV_STATE;

/*
/** Struct used to pass information in camera video port userdata to callback
 * /
typedef struct
{
	char *data;					/// YUV data container
	FILE *file_handle;                   /// File handle to write buffer data to.
	RASPIVIDYUV_STATE *pstate;           /// pointer to our state in case required in callback
	int abort;                           /// Set to 1 in callback if an error occurs to attempt to abort the capture
	FILE *pts_file_handle;               /// File timestamps
	int frame;
	int64_t starttime;
	int64_t lasttime;
} PORT_USERDATA;





/** Structure containing all state information for the current run
 * /
struct RASPIVIDYUV_STATE_S
{
   int timeout;                        /// Time taken before frame is grabbed and app then shuts down. Units are milliseconds
   int width;                          /// Requested width of image
   int height;                         /// requested height of image
   int framerate;                      /// Requested frame rate (fps)
   char *filename;                     /// filename of output file
   int verbose;                        /// !0 if want detailed run information
   int demoMode;                       /// Run app in demo mode
   int demoInterval;                   /// Interval between camera settings changes
   int waitMethod;                     /// Method for switching between pause and capture

   int onTime;                         /// In timed cycle mode, the amount of time the capture is on per cycle
   int offTime;                        /// In timed cycle mode, the amount of time the capture is off per cycle

   int onlyLuma;                       /// Only output the luma / Y plane of the YUV data
   int useRGB;                         /// Output RGB data rather than YUV

   RASPIPREVIEW_PARAMETERS preview_parameters;   /// Preview setup parameters
   RASPICAM_CAMERA_PARAMETERS camera_parameters; /// Camera setup parameters

   MMAL_COMPONENT_T *camera_component;    /// Pointer to the camera component
   MMAL_CONNECTION_T *preview_connection; /// Pointer to the connection from camera to preview

   MMAL_POOL_T *camera_pool;            /// Pointer to the pool of buffers used by camera video port

   PORT_USERDATA callback_data;         /// Used to move data to the camera callback

   int bCapturing;                      /// State of capture/pause

   int cameraNum;                       /// Camera number
   int sensor_mode;                     /// Sensor mode. 0=auto. Check docs/forum for modes selected by other values.

   int frame;
   char *pts_filename;
   int save_pts;
   int64_t starttime;
   int64_t lasttime;

   bool netListen;
};



MMAL_STATUS_T connect_ports(MMAL_PORT_T *output_port, MMAL_PORT_T *input_port, MMAL_CONNECTION_T **connection);

void pigun_signal_handler(int signal_number);
void pigun_default_status(RASPIVIDYUV_STATE *state);

MMAL_STATUS_T pigun_camcomp_make(RASPIVIDYUV_STATE *state);
void pigun_camcomp_destroy(RASPIVIDYUV_STATE *state);

int wait_for_next_change(RASPIVIDYUV_STATE *state);
int pause_and_test_abort(RASPIVIDYUV_STATE *state, int pause);
*/


#endif
