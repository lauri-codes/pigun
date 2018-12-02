#include <stdio.h>

#include "pigun.h"

#include "bcm_host.h"
#include "interface/mmal/mmal.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_connection.h"
#include "interface/vcos/vcos.h"



/* HOW THE CODE SHOULD WORK:
 * 
 * 
 * create camera component
 * get output from video port
 * do processing
 * 
 * lol not so easy tho... fucking libMMAL without documentation!
 * 
 * 
 * 
 * */


// global buffer pools
MMAL_POOL_T *pool_cam_vid;
MMAL_POOL_T *pool_prv_in1;
MMAL_PORT_T *port_prv_in1;


void preview_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {
	printf("callbacker len=%d \n", buffer->length);
	
	
	mmal_buffer_header_release(buffer);
}


int main(int argc, char **argv) {


	printf("hello world!\n");


	MMAL_COMPONENT_T *cam = NULL;
	MMAL_COMPONENT_T *prv = NULL;
	
	MMAL_ES_FORMAT_T *format;
	MMAL_STATUS_T status;
	
	MMAL_PORT_T *port_cam_prv = NULL;
	MMAL_PORT_T *port_cam_vid = NULL;
	MMAL_PORT_T *port_cam_img = NULL;
	
	
	MMAL_CONNECTION_T *con_cam_prv = NULL;
	
	bcm_host_init();
	
	
	status = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA, &cam);
	if(status != MMAL_SUCCESS) {
		printf("no camera!\n");
		return -1;
	}
	
	// get refs to the camera ports
	port_cam_prv = cam->output[MMAL_CAMERA_PREVIEW_PORT];
	port_cam_vid = cam->output[MMAL_CAMERA_VIDEO_PORT];
	port_cam_img = cam->output[MMAL_CAMERA_CAPTURE_PORT];
	
	
	// setup the camera with this resolution
	// using this resolution the full sensor area is used (in the X direction) --> fps capped at 40
	// 1640x922
	// 1640x1232
	pigun_camera_setup(cam, 1280, 720); // this seems ok and it can go to 90fps
	
	// set the format of cam.preview
	pigun_port_setformat(port_cam_prv, 640, 360); // the preview will get a resized version of the camera output
	
	// set the format of cam.video
	//pigun_port_setformat_buffered(port_cam_vid, 640, 360, 2);
	pigun_port_setformat(port_cam_vid, 640, 360);
	
	// the combination of camera mode and port output size should be tuned so that the lightgun can actually see the two reference LEDs
	// using small camera mode resolutions will force partial usage of the photosensor area, and the LEDs may not be visible.
	// with 640x480 camera mode, at 1m distance from the screen, the computer screen (~36") was not entirely visible!
	
	
	/*
	// create a buffer pool
	pool_cam_vid = (MMAL_POOL_T*)mmal_port_pool_create(port_cam_vid, port_cam_vid->buffer_num, port_cam_vid->buffer_size);
	port_cam_vid->userdata = (struct MMAL_PORT_USERDATA_T*) pool_cam_vid;
	
	status = mmal_port_enable(port_cam_vid, pigun_video_buffer_callback);
	if(status != MMAL_SUCCESS) {
		printf("error in port enable...\n");
		return -1;
	}*/
	/*
	status = mmal_port_enable(port_cam_vid, pigun_video_buffer_callback);
	*/
	status = mmal_component_enable(cam);
	
	
	// create a preview component
	status = mmal_component_create(MMAL_COMPONENT_DEFAULT_VIDEO_RENDERER, &prv);
	
	port_prv_in1 = prv->input[0];

	// configure the preview component
    {
        MMAL_DISPLAYREGION_T param;
        param.hdr.id = MMAL_PARAMETER_DISPLAYREGION;
        param.hdr.size = sizeof (MMAL_DISPLAYREGION_T);
        param.set = MMAL_DISPLAY_SET_LAYER;
        param.layer = 0;
        param.set |= MMAL_DISPLAY_SET_FULLSCREEN;
        param.fullscreen = 1;
        status = mmal_port_parameter_set(port_prv_in1, &param.hdr);
        if (status != MMAL_SUCCESS && status != MMAL_ENOSYS) {
            printf("Error: unable to set preview port parameters (%u)\n", status);
            return -1;
        }
    }
	
	
	// set input format
	mmal_format_copy(port_prv_in1->format, port_cam_vid->format);
	
	port_prv_in1->buffer_size = port_prv_in1->buffer_size_recommended;
	port_prv_in1->buffer_num = 4;
	
	mmal_port_format_commit(port_prv_in1);
	
	
	pool_prv_in1 = (MMAL_POOL_T*)mmal_port_pool_create(port_prv_in1, port_prv_in1->buffer_num, port_prv_in1->buffer_size);
	port_prv_in1->userdata = (struct MMAL_PORT_USERDATA_T*) pool_prv_in1;
	
	status = mmal_port_enable(port_prv_in1, preview_buffer_callback);
	
	
	// do the connection
	status = mmal_connection_create(&con_cam_prv, port_cam_prv, port_prv_in1, MMAL_CONNECTION_FLAG_TUNNELLING | MMAL_CONNECTION_FLAG_ALLOCATION_ON_INPUT);
    if (status != MMAL_SUCCESS) {
        printf("Error: unable to create connection (%u)\n", status);
        return -1;
    }

	// enable it?
    status = mmal_connection_enable(con_cam_prv);
    if (status != MMAL_SUCCESS) {
        printf("Error: unable to enable connection (%u)\n", status);
        return -1;
    }
    
    /*
    int num = mmal_queue_length(pool_cam_vid->queue);
    int q;
    for(q=0; q<num; q++) {
		MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(pool_cam_vid->queue);
		if(!buffer)
			printf("error in buffer %i \n", q);
	}
    */
    
    printf("looping...\n");
    //mmal_port_parameter_set_boolean(port_cam_vid, MMAL_PARAMETER_CAPTURE, 1);
    while (1);


	return 0;
}
