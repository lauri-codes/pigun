/*
 * File:   buffer_demo.c
 * Author: Tasanakorn
 *
 * Created on May 22, 2013, 1:52 PM
 */

 /* GENERAL IDEA OF THIS CODE
  *
  * the camera acquires... as fast as possible i guess!
  * camera.video output is configured with half resolution of the camera acquisition, at 90 fps
  * camera.video output is NOT connected to anything, but buffered into a pool
  * camera.video -> buffer (video_buffer_callback)
  * camera
  * video_buffer_callback copies the Y channel into another buffer
  * we use the Y channel to detect the IR LEDs
  *
  * camera video output uses MMAL_ENCODING_I420
  *
  *
  * */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/time.h>

#include <bcm2835.h>

#include "bcm_host.h"
#include "interface/vcos/vcos.h"

#include "interface/mmal/mmal.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_connection.h"

#include "pigun.h"
#include "pigun_bt.h"
#include <math.h>
#include <stdint.h>

#include <X11/Xlib.h>
#include <X11/Xutil.h>

// the HID report
pigun_report_t global_pigun_report;

// List of gpio code that will be used as the 8 buttons
int pigun_button_pin[8] = { PIN_TRG,PIN_RLD,PIN_AX3,PIN_AX4 ,PIN_AX5 , PIN_AX6 , PIN_AX7 ,PIN_CAL };

// counters for each button
uint8_t pigun_button_holder[8] = { 0,0,0,0,0,0,0,0 };
// a bit is set to 1 if the button was just pressed
uint8_t pigun_button_newpress = 0;


// detected peaks - in order
Peak* pigun_peaks;

// normalised aiming point - before calibration applies
PigunAimPoint pigun_aim_norm;

PigunAimPoint pigun_cal_topleft;
PigunAimPoint pigun_cal_lowright;


/// <summary>
/// State of the gun:
/// 0: idle/normal
/// 1: calibration - expect top-left
/// 2: calibration - expect bottom-right
/// </summary>
int pigun_state;

// Calibration offsets to apply along x/y in reduced coordinates (0,1)
float pigun_aimOffset_x, pigun_aimOffset_y;


#ifdef PIGUN_MOUSE
Display* displayMain;
Screen* screen;
int screenWidth;
int screenHeight;

void mouseMove(float x, float y) {

    Window root = DefaultRootWindow(displayMain);
    int screenX = round(x * screenWidth);
    int screenY = round(y * screenHeight);
    XWarpPointer(displayMain, None, root, 0, 0, 0, 0, screenX, screenY);
    XFlush(displayMain);
}
#endif

// GLOBAL MMAL STUFF
MMAL_PORT_T* pigun_video_port;
MMAL_POOL_T* pigun_video_port_pool;
#ifdef PIGUN_PREVIEW
MMAL_POOL_T* preview_input_port_pool = NULL;
MMAL_PORT_T* preview_input_port = NULL;
#endif


static inline void button_pressed(int buttonID){

    // it will have to be another 10 frames before the button can be pressed again
    pigun_button_holder[buttonID] = 10;

    // set the button in the HID report
    global_pigun_report.buttons |= (uint8_t)(1 << buttonID);

    // mark a good press event internally?
    pigun_button_newpress |= (uint8_t)(1 << buttonID);
}


static void preview_buffer_callback(MMAL_PORT_T* port, MMAL_BUFFER_HEADER_T* buffer) { mmal_buffer_header_release(buffer); }


/* Camera Callback Function
*
* This is called when the camera has a frame ready.
* buffer->data has the pixel values in the chosen encoding (I420).
*
*/
static void video_buffer_callback(MMAL_PORT_T* port, MMAL_BUFFER_HEADER_T* buffer) {

#ifdef PIGUN_TIMING

    // Measure elapsed time and print instantaneous FPS
    static int loop = 0;                // Global variable for loop number
    static struct timespec t1 = { 0, 0 }; // Global variable for previous loop time
    struct timespec t2;                 // Current loop time
    float dt;                           // Elapsed time between frames

    clock_gettime(CLOCK_MONOTONIC, &t2);
    if (t1.tv_sec != 0 && t1.tv_nsec != 0) {
        dt = t2.tv_sec + (t2.tv_nsec / 1000000000.0) - (t1.tv_sec + (t1.tv_nsec / 1000000000.0));
    }
    loop++;

    if (loop > 0 && loop % 50 == 0) {
    	printf("loop = %d, Framerate = %f fps, buffer->length = %d \n", loop, 1.0 / dt, buffer->length);
    }
    t1 = t2;
#endif

    MMAL_POOL_T* pool = (MMAL_POOL_T*)port->userdata;

    // call the peak detector function
    pigun_detect(buffer->data);
    // the peaks are supposed to be ordered by the detector function

    // TODO: maybe add a mutex/semaphore so that the main bluetooth thread
    // will wait until this is done with the x/y aim before reading the HID report

    // computes the aiming position from the peaks
    pigun_calculate_aim();



    // check the buttons ***************************************************

    // TODO: maybe add a mutex/semaphore so that the main bluetooth thread
    // will wait until this is done with the buttons before reading the HID report
    
    // make a copy of the current button state
    uint8_t buttonsDown = global_pigun_report.buttons;
    // mark all buttons as not being in "new press" state
    pigun_button_newpress = 0;

    for (int i = 0; i < 8; i++) {

        if (bcm2835_gpio_eds(pigun_button_pin[i])) {
            // this happens when falling edge is detected

            // record press
            if (pigun_button_holder[i] == 0) button_pressed(i);

            // clear GPIO event flag
            // this is done every time the event was detected, regardless of whether the
            // event really was a valid press, otherwise the BCM2835 wont detect it again!
            bcm2835_gpio_set_eds(pigun_button_pin[i]);

            // ignore the rest of the code since it is dealing with button releases
            continue;
        }

        // code here => the button was not pressed just now
        // either was was not pressed at all, or it was already pressed before this frame

        // if it was already pressed, check that it is still the case
        if ((buttonsDown >> i) & 1) {

            // if the hold timer is expired...
            if (pigun_button_holder[i] == 0) {
                
                // ... and the GPIO level is high
                if (bcm2835_gpio_lev(pigun_button_pin[i]) == 0) {

                    // then release the button in HID report
                    global_pigun_report.buttons &= ~(uint8_t)(1 << i);
                }
            }
            else {
                // code here => the hold timer needs to tick down
                pigun_button_holder[i]--;
                // even if the button was released in the meantime, it will not be released
                // in the HID report until the timer runs to 0.
            }
        }
        // if it wasnt pressed at all, then do nothing
    }

    // *********************************************************************
    // *** deal with some specific buttons *** *****************************

    if ((pigun_button_newpress >> 7) & 1) { // if CAL button was just pressed

        pigun_state = 1; // next trigger pull marks top-left calibration point
    }
    else if (pigun_button_newpress & 1) { // if TRIGGER was just pressed

        if (pigun_state == 1) {
            // set the top-left calibration point
            pigun_cal_topleft = pigun_aim_norm;
            pigun_state = 2;
        }
        else if (pigun_state == 2) {
            // set the low-right calibration point
            pigun_cal_lowright = pigun_aim_norm;
            pigun_state = 0;
        }
        else if (pigun_state == 0) {
            // TODO: fire the solenoid on some other pin?

        }
    }
    // *********************************************************************
    // *********************************************************************



    // setup a frame buffer to show in the preview window ******************
#ifdef PIGUN_PREVIEW
    // fetches a free buffer from the pool of the preview.input port
    MMAL_BUFFER_HEADER_T* preview_new_buffer = mmal_queue_get(preview_input_port_pool->queue);

    if (preview_new_buffer) {

        preview_new_buffer->length = buffer->length;
        pigun_preview(preview_new_buffer, buffer);


        // I guess this is where the magic happens...
        // the newbuffer is sent to the preview.input port

        if (mmal_port_send_buffer(preview_input_port, preview_new_buffer) != MMAL_SUCCESS) {
            printf("PIGUN ERROR: Unable to send buffer to preview input port\n");
        }
    }
    else {
        printf("PIGUN ERROR: mmal_queue_get (%d)\n", preview_new_buffer);
    }
#endif
    // *********************************************************************

    // we are done with this buffer, we can release it!
    mmal_buffer_header_release(buffer);

    // and send one back to the port (if still open)
    // I really dont get why this has to happen?!
    // but if we take it out, the whole thing stops working!
    // perhaps the port needs empty buffers to work with...
    if (port->is_enabled) {

        MMAL_STATUS_T status = MMAL_SUCCESS;
        MMAL_BUFFER_HEADER_T* new_buffer;
        new_buffer = mmal_queue_get(pool->queue);

        if (new_buffer)
            status = mmal_port_send_buffer(port, new_buffer);

        if (!new_buffer || status != MMAL_SUCCESS)
            printf("PIGUN ERROR: Unable to return a buffer to the video port\n");
    }

}



// Initialises the camera with libMMAL
int pigun_mmal_init(void) {

    printf("PIGUN: initializing camera...\n");

    MMAL_COMPONENT_T* camera = NULL;
    MMAL_COMPONENT_T* preview = NULL;

    MMAL_ES_FORMAT_T* format = NULL;

    MMAL_PORT_T* camera_preview_port = NULL;
    MMAL_PORT_T* camera_video_port = NULL;
    MMAL_PORT_T* camera_still_port = NULL;

    MMAL_POOL_T* camera_video_port_pool;

    MMAL_CONNECTION_T* camera_preview_connection = NULL;

    MMAL_STATUS_T status;

    // I guess this starts the driver?
    bcm_host_init();
    printf("BCM Host initialized.\n");

    status = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA, &camera);
    if (status != MMAL_SUCCESS) {
        printf("PIGUN ERROR: create camera returned %x\n", status);
        return -1;
    }

    // connect ports
    camera_preview_port = camera->output[MMAL_CAMERA_PREVIEW_PORT];
    camera_video_port = camera->output[MMAL_CAMERA_VIDEO_PORT];
    camera_still_port = camera->output[MMAL_CAMERA_CAPTURE_PORT];

    // configure the camera component **********************************
    {
        MMAL_PARAMETER_CAMERA_CONFIG_T cam_config = {
            { MMAL_PARAMETER_CAMERA_CONFIG, sizeof(cam_config)},
            .max_stills_w = PIGUN_CAM_X,
            .max_stills_h = PIGUN_CAM_Y,
            .stills_yuv422 = 0,
            .one_shot_stills = 1,
            .max_preview_video_w = PIGUN_CAM_X,
            .max_preview_video_h = PIGUN_CAM_Y,
            .num_preview_video_frames = 3,
            .stills_capture_circular_buffer_height = 0,
            .fast_preview_resume = 0,
            .use_stc_timestamp = MMAL_PARAM_TIMESTAMP_MODE_RESET_STC
        };
        mmal_port_parameter_set(camera->control, &cam_config.hdr);
    }
    // *****************************************************************
    // Setup camera video port format **********************************
    format = camera_video_port->format;

    format->encoding = MMAL_ENCODING_I420;
    format->encoding_variant = MMAL_ENCODING_I420;

    // video port outputs reduced resolution
    format->es->video.width = PIGUN_RES_X;
    format->es->video.height = PIGUN_RES_Y;
    format->es->video.crop.x = 0;
    format->es->video.crop.y = 0;
    format->es->video.crop.width = PIGUN_RES_X;
    format->es->video.crop.height = PIGUN_RES_Y;
    format->es->video.frame_rate.num = PIGUN_FPS;
    format->es->video.frame_rate.den = 1;

    camera_video_port->buffer_size = camera_video_port->buffer_size_recommended;
    camera_video_port->buffer_num = 4;

    // apply the format
    status = mmal_port_format_commit(camera_video_port);
    if (status != MMAL_SUCCESS) {
        printf("PIGUN ERROR: unable to commit camera video port format (%u)\n", status);
        return -1;
    }

    printf("PIGUN: camera video buffer_size = %d\n", camera_video_port->buffer_size);
    printf("PIGUN: camera video buffer_num = %d\n", camera_video_port->buffer_num);
    // *****************************************************************
    // crate buffer pool from camera.video output port *****************
    camera_video_port_pool = (MMAL_POOL_T*)mmal_port_pool_create(
        camera_video_port,
        camera_video_port->buffer_num,
        camera_video_port->buffer_size
    );
    camera_video_port->userdata = (struct MMAL_PORT_USERDATA_T*)camera_video_port_pool;

    // the port is enabled with the given callback function
    // the callback is called when a complete frame is ready at the camera.video output port
    status = mmal_port_enable(camera_video_port, video_buffer_callback);
    if (status != MMAL_SUCCESS) {
        printf("PIGUN ERROR: unable to enable camera video port (%u)\n", status);
        return -1;
    }
    printf("PIGUN: frame buffer created.\n");
    // *****************************************************************

    // not sure if this is needed - seems to work without as well
    status = mmal_component_enable(camera);

    // *** SETUP THE PREVIEW SYSTEM ************************************
#ifdef PIGUN_PREVIEW
    // create a renderer component to show the video on screen
    status = mmal_component_create(MMAL_COMPONENT_DEFAULT_VIDEO_RENDERER, &preview);
    if (status != MMAL_SUCCESS) {
        printf("PIGUN ERROR: unable to create preview (%u)\n", status);
        return -1;
    }

    // setup the preview input port
    preview_input_port = preview->input[0];
    MMAL_RECT_T previewWindow;
    previewWindow.x = 0;
    previewWindow.y = 0;
    previewWindow.width = PIGUN_RES_X;
    previewWindow.height = PIGUN_RES_Y;
    {
        MMAL_DISPLAYREGION_T param;
        param.hdr.id = MMAL_PARAMETER_DISPLAYREGION;
        param.hdr.size = sizeof(MMAL_DISPLAYREGION_T);
        param.set = MMAL_DISPLAY_SET_LAYER;
        param.layer = 0;
        param.set |= (MMAL_DISPLAY_SET_DEST_RECT | MMAL_DISPLAY_SET_FULLSCREEN);
        param.fullscreen = 0;
        param.dest_rect = previewWindow;
        status = mmal_port_parameter_set(preview_input_port, &param.hdr);
        if (status != MMAL_SUCCESS && status != MMAL_ENOSYS) {
            printf("PIGUN ERROR: unable to set preview port parameters (%u)\n", status);
            return -1;
        }
    }

    // setup the format of preview.input port -> same as camera.video output!
    mmal_format_copy(preview_input_port->format, camera_video_port->format);

    format = preview_input_port->format;

    format->encoding = MMAL_ENCODING_I420;
    format->encoding_variant = MMAL_ENCODING_I420;

    format->es->video.width = PIGUN_RES_X;
    format->es->video.height = PIGUN_RES_Y;
    format->es->video.crop.x = 0;
    format->es->video.crop.y = 0;
    format->es->video.crop.width = PIGUN_RES_X;
    format->es->video.crop.height = PIGUN_RES_Y;
    format->es->video.frame_rate.num = PIGUN_FPS;
    format->es->video.frame_rate.den = 1;

    preview_input_port->buffer_size = camera_video_port->buffer_size_recommended;
    preview_input_port->buffer_num = 8; // with a larger number of buffer

    printf("PIGUN: preview buffer_size = %d\n", preview_input_port->buffer_size);
    printf("PIGUN: preview buffer_num = %d\n", preview_input_port->buffer_num);

    status = mmal_port_format_commit(preview_input_port);
    if (status != MMAL_SUCCESS) {
        printf("PIGUN ERROR: unable to commit preview input port format (%u)\n", status);
        return -1;
    }

    // create a buffer pool for the preview.input port
    preview_input_port_pool = (MMAL_POOL_T*)mmal_port_pool_create(preview_input_port, preview_input_port->buffer_num, preview_input_port->buffer_size);

    preview_input_port->userdata = (struct MMAL_PORT_USERDATA_T*)preview_input_port_pool;
    // the input port is enabled, using the dull callback (it just releases the buffer)
    status = mmal_port_enable(preview_input_port, preview_buffer_callback);
    if (status != MMAL_SUCCESS) {
        printf("Error: unable to enable preview input port (%u)\n", status);
        return -1;
    }

    // camera.video is not connected directly to preview.input!!!!
    // the callback on camera.video is physically copying the output buffer into the buffer of the input port

    /*
    status = mmal_connection_create(&camera_preview_connection, camera_preview_port, preview_input_port, MMAL_CONNECTION_FLAG_TUNNELLING | MMAL_CONNECTION_FLAG_ALLOCATION_ON_INPUT);
    if (status != MMAL_SUCCESS) {
        printf("PIGUN ERROR: unable to create preview connection (%u)\n", status);
        return -1;
    }
    status = mmal_connection_enable(camera_preview_connection);
    if (status != MMAL_SUCCESS) {
        printf("PIGUN ERROR: unable to enable preview connection (%u)\n", status);
        return -1;
    }
    */

#endif
        // *****************************************************************

    printf("PIGUN: setting up parameters\n");
        // Disable exposure mode
    pigun_camera_exposuremode(camera, 0);

    // Set gains
    //if (argc == 3) {
        //int analog_gain = atoi(argv[1]);
        //int digital_gain = atoi(argv[2]);
        //pigun_camera_gains(camera, analog_gain, digital_gain);
    //}
    //
    // Setup automatic white balance
    pigun_camera_awb(camera, 0);
    pigun_camera_awb_gains(camera, 1, 1);

    // Setup blur
    pigun_camera_blur(camera, 1);
    printf("PIGUN: parameters set\n");

    // send the buffers to the camera.video output port so it can start filling them frame data

    // Send all the buffers to the encoder output port
    int num = mmal_queue_length(camera_video_port_pool->queue);
    int q;
    for (q = 0; q < num; q++) {
        MMAL_BUFFER_HEADER_T* buffer = mmal_queue_get(camera_video_port_pool->queue);

        if (!buffer)
            printf("PIGUN ERROR: Unable to get a required buffer %d from pool queue\n", q);

        if (mmal_port_send_buffer(camera_video_port, buffer) != MMAL_SUCCESS)
            printf("PIGUN ERROR: Unable to send a buffer to encoder output port (%d)\n", q);

        // we are not really dealing with errors... they are not supposed to happen anyway!
    }

    status = mmal_port_parameter_set_boolean(camera_video_port, MMAL_PARAMETER_CAPTURE, 1);
    if (status != MMAL_SUCCESS) {
        printf("PIGUN ERROR: %s: Failed to start capture\n", __func__); // what is this func?
        return -1;
    }

    // save necessary stuff to global vars
    pigun_video_port = camera_video_port;
    pigun_video_port_pool = camera_video_port_pool;

    printf("PIGUN: camera init done\n");
    return 0;
}



//void * test_main(int argc, char** argv) {
void* pigun_cycle(void* nullargs) {

    //int argc = 0;
    //char** argv;

#ifdef PIGUN_MOUSE
    // Open mouse connection
    displayMain = XOpenDisplay(NULL);
    if (displayMain == NULL)
    {
        fprintf(stderr, "PIGUN ERROR: Could not open main display\n");
        exit(EXIT_FAILURE);
    }
    else {
        screen = DefaultScreenOfDisplay(displayMain);
        screenWidth = screen->width;
        screenHeight = screen->height;
    }
#endif

    // Initialize the camera system
    int error = pigun_mmal_init();
    if (error != 0) {
        return NULL;
    }

    // normal state
    pigun_state = 0;
    

    // allocate peaks
    pigun_peaks = (Peak*)calloc(4, sizeof(Peak));

    // GPIO system
    if (!bcm2835_init()) {
        printf("PIGUN ERROR: failed to init BCM2835!\n");
        return NULL;
    }



    for (int i = 0; i < 8; ++i) {
        bcm2835_gpio_fsel(pigun_button_pin[i], BCM2835_GPIO_FSEL_INPT);   // set pin as INPUT
        bcm2835_gpio_set_pud(pigun_button_pin[i], BCM2835_GPIO_PUD_UP);   // give it a pullup resistor
        bcm2835_gpio_fen(pigun_button_pin[i]);  // detect falling edge - button is pressed
    }


    // repeat forever and ever!
    // there could be a graceful shutdown?
    while (1) {

        // in here we have to check the state of GPIOs
        // initially pigun_buttons and pigun_button_release are both 0
        // when a change is detected on a GPIO, both buttons and releases are set to 1
        // the buttons flag is reset to 0 when the frame callback handles the event
        // the release flag is reset to 0 when the button can be pressed again (to avoid jitter): maybe after some time or frames? need to test
        // buttons flag will not be set to 1 if the relase is already 1

        //bcm2835_gpio_eds(PIN) // test for event detection flag
        //bcm2835_gpio_lev(PIN) // returns the level of the pin

    }

#ifdef PIGUN_MOUSE
    XCloseDisplay(displayMain);
#endif

    free(pigun_peaks);
    return NULL;
}


