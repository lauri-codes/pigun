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
 * 
 * camera video output uses MMAL_ENCODING_I420
 * 
 * 
 * */

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>

#include "bcm_host.h"
#include "interface/vcos/vcos.h"

#include "interface/mmal/mmal.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_connection.h"

#include "pigun.h"
#include <opencv2/opencv.hpp>

using namespace cv;

Mat frame, image;
Ptr<SimpleBlobDetector> detector;
std::vector<KeyPoint> keypoints;



MMAL_POOL_T *camera_video_port_pool;
MMAL_POOL_T *preview_input_port_pool;
MMAL_PORT_T *preview_input_port = NULL;

static void preview_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {
    //printf("INFO:preview_buffer_callback buffer->length = %d\n", buffer->length);
    
    mmal_buffer_header_release(buffer);
}

static void video_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {
	
	
    static int loop = 0;
    static struct timespec t1;
    struct timespec t2;
    //printf("INFO:video_buffer_callback\n");
    if (loop == 0) {
        clock_gettime(CLOCK_MONOTONIC, &t1);
    }
    clock_gettime(CLOCK_MONOTONIC, &t2);

    int d = t2.tv_sec - t1.tv_sec;
	loop++;

    MMAL_BUFFER_HEADER_T *new_buffer;
    MMAL_BUFFER_HEADER_T *preview_new_buffer;
    MMAL_POOL_T *pool = (MMAL_POOL_T *) port->userdata;

    
    
    frame = Mat(PIGUN_RES_Y, PIGUN_RES_X, CV_8UC1, buffer->data);
    //detector->detect(frame, keypoints);
    
    int i;
    unsigned int tester = 0;
    // test a 3 loop over pixels pass
    for(int i=0; i<PIGUN_NPX; i++) {
		if(buffer->data[i] > 100)
			tester += 1;
	}
    /*for(int i=0; i<PIGUN_NPX; i++) {
		if(buffer->data[i] > 100)
			tester += 1;
	}*/
    //memset(buffer->data, tester, PIGUN_NPX);
    
    // fetches a free buffer from the pool of the preview.input port
    preview_new_buffer = mmal_queue_get(preview_input_port_pool->queue);

    if (preview_new_buffer) {
        
        
         
        memcpy(preview_new_buffer->data, buffer->data, PIGUN_NPX); // copy only Y 
        //memcpy(preview_new_buffer->data, frame.data, PIGUN_NPX); // copy only Y 
        memset(&preview_new_buffer->data[PIGUN_NPX], 0, PIGUN_NPX/4);
        memset(&preview_new_buffer->data[PIGUN_NPX+PIGUN_NPX/4], 0b10101010, PIGUN_NPX/4);
		preview_new_buffer->length = buffer->length;
        
        
        
        //Mat image;
        //drawKeypoints(frame, keypoints, image, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        //memcpy(preview_new_buffer->data, image.data, PIGUN_NPX); // copy only Y 
        
        // i guess this is where the magic happens...
        // the newbuffer is sent to the preview.input port
        if (mmal_port_send_buffer(preview_input_port, preview_new_buffer) != MMAL_SUCCESS) {
            printf("ERROR: Unable to send buffer \n");
        }
    } else {
        printf("ERROR: mmal_queue_get (%d)\n", preview_new_buffer);
    }

    if (loop % 10 == 0) {
        //fprintf(stderr, "loop = %d \n", loop);
        printf("loop = %d, Framerate = %d fps, buffer->length = %d tester=%d\n", loop, loop / (d + 1), buffer->length, tester);
    }

	// we are done with this buffer, we can release it!
    mmal_buffer_header_release(buffer);

    // and send one back to the port (if still open)
    // I really dont get why this has to happen?!
    // but if we take it out, the whole thing stops working!
    // perhaps the port needs empty buffers to work with...
    if (port->is_enabled) {
        MMAL_STATUS_T status;

        new_buffer = mmal_queue_get(pool->queue);

        if (new_buffer)
            status = mmal_port_send_buffer(port, new_buffer);

        if (!new_buffer || status != MMAL_SUCCESS)
            printf("Unable to return a buffer to the video port\n");
    }
    
}

int main(int argc, char** argv) {
	
    MMAL_COMPONENT_T *camera = 0;
    MMAL_COMPONENT_T *preview = 0;
    MMAL_ES_FORMAT_T *format;
    MMAL_STATUS_T status;
    MMAL_PORT_T *camera_preview_port = NULL, *camera_video_port = NULL, *camera_still_port = NULL;

	detector = SimpleBlobDetector::create();

    MMAL_CONNECTION_T *camera_preview_connection = 0;

    printf("Running...\n");

	// i guess this starts the driver?
    bcm_host_init();

    status = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA, &camera);
    if (status != MMAL_SUCCESS) {
        printf("Error: create camera %x\n", status);
        return -1;
    }

    camera_preview_port = camera->output[MMAL_CAMERA_PREVIEW_PORT];
    camera_video_port = camera->output[MMAL_CAMERA_VIDEO_PORT];
    camera_still_port = camera->output[MMAL_CAMERA_CAPTURE_PORT];

    {
        MMAL_PARAMETER_CAMERA_CONFIG_T cam_config = {
            { MMAL_PARAMETER_CAMERA_CONFIG, sizeof (cam_config)},
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

	// TODO: try to add a camera blur filter
	// ...

	/* commented by me!
    // Setup camera preview port format 
    format = camera_preview_port->format;

    format->encoding = MMAL_ENCODING_OPAQUE;
    format->encoding_variant = MMAL_ENCODING_I420;

    format->es->video.width = 1280;
    format->es->video.height = 720;
    format->es->video.crop.x = 0;
    format->es->video.crop.y = 0;
    format->es->video.crop.width = 1280;
    format->es->video.crop.height = 720;
	format->es->video.frame_rate.num = 90;
	format->es->video.frame_rate.den = 1;

    status = mmal_port_format_commit(camera_preview_port);

    if (status != MMAL_SUCCESS) {
        printf("Error: camera viewfinder format couldn't be set\n");
        return -1;
    }
	*/
	
    // Setup camera video port format **********************************
    format = camera_video_port->format;

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


    camera_video_port->buffer_size = camera_video_port->buffer_size_recommended;
    camera_video_port->buffer_num = 4;

    status = mmal_port_format_commit(camera_video_port);
    
    printf(" camera video buffer_size = %d\n", camera_video_port->buffer_size);
    printf(" camera video buffer_num = %d\n", camera_video_port->buffer_num);
    if (status != MMAL_SUCCESS) {
        printf("Error: unable to commit camera video port format (%u)\n", status);
        return -1;
    }
	// *****************************************************************

    // crate pool form camera.video output port
    camera_video_port_pool = (MMAL_POOL_T *)mmal_port_pool_create(camera_video_port, camera_video_port->buffer_num, camera_video_port->buffer_size);
    camera_video_port->userdata = (struct MMAL_PORT_USERDATA_T *) camera_video_port_pool;
	
	// the port is enabled with the given callback function
	// the callback is called when a complete frame is ready at the camera.video output port
    status = mmal_port_enable(camera_video_port, video_buffer_callback);
    if (status != MMAL_SUCCESS) {
        printf("Error: unable to enable camera video port (%u)\n", status);
        return -1;
    }

	// not sure if this is needed - seems to work without as well
    status = mmal_component_enable(camera);


	// create a renderer component to show the video on screen
    status = mmal_component_create(MMAL_COMPONENT_DEFAULT_VIDEO_RENDERER, &preview);
    if (status != MMAL_SUCCESS) {
        printf("Error: unable to create preview (%u)\n", status);
        return -1;
    }

	// setup the preview input port
    preview_input_port = preview->input[0];
    {
        MMAL_DISPLAYREGION_T param;
        param.hdr.id = MMAL_PARAMETER_DISPLAYREGION;
        param.hdr.size = sizeof (MMAL_DISPLAYREGION_T);
        param.set = MMAL_DISPLAY_SET_LAYER;
        param.layer = 0;
        param.set |= MMAL_DISPLAY_SET_FULLSCREEN;
        param.fullscreen = 1;
        status = mmal_port_parameter_set(preview_input_port, &param.hdr);
        if (status != MMAL_SUCCESS && status != MMAL_ENOSYS) {
            printf("Error: unable to set preview port parameters (%u)\n", status);
            return -1;
        }
    }
    mmal_format_copy(preview_input_port->format, camera_video_port->format);
	
	// setup the format of preview.input port -> same as camera.video output!
    format = preview_input_port->format;

    format->encoding = MMAL_ENCODING_I420;
    format->encoding_variant = MMAL_ENCODING_I420;

    format->es->video.width = PIGUN_RES_X;
    format->es->video.height = PIGUN_RES_Y;
    format->es->video.crop.x = 0;
    format->es->video.crop.y = 0;
    format->es->video.crop.width = PIGUN_RES_X;
    format->es->video.crop.height = PIGUN_RES_Y;
    format->es->video.frame_rate.num = 90;
    format->es->video.frame_rate.den = 1;

    preview_input_port->buffer_size = camera_video_port->buffer_size_recommended;
    preview_input_port->buffer_num = 8; // with a larger number of buffer

    printf(" preview buffer_size = %d\n", preview_input_port->buffer_size);
    printf(" preview buffer_num = %d\n", preview_input_port->buffer_num);
    
    status = mmal_port_format_commit(preview_input_port);

	// create a buffer pool for the preview.input port
    preview_input_port_pool = (MMAL_POOL_T *)mmal_port_pool_create(preview_input_port, preview_input_port->buffer_num, preview_input_port->buffer_size);

    preview_input_port->userdata = (struct MMAL_PORT_USERDATA_T *) preview_input_port_pool;
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
        printf("Error: unable to create connection (%u)\n", status);
        return -1;
    }
    status = mmal_connection_enable(camera_preview_connection);
    if (status != MMAL_SUCCESS) {
        printf("Error: unable to enable connection (%u)\n", status);
        return -1;
    }
     */
     
     // this sends the buffers to the camera.video output port so it can start filling them frame data
    if (1) {
        // Send all the buffers to the encoder output port
        int num = mmal_queue_length(camera_video_port_pool->queue);
        int q;

        for (q = 0; q < num; q++) {
            MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(camera_video_port_pool->queue);

            if (!buffer)
                printf("Unable to get a required buffer %d from pool queue\n", q);

            if (mmal_port_send_buffer(camera_video_port, buffer) != MMAL_SUCCESS)
                printf("Unable to send a buffer to encoder output port (%d)\n", q);
        }


    }

    if (mmal_port_parameter_set_boolean(camera_video_port, MMAL_PARAMETER_CAPTURE, 1) != MMAL_SUCCESS) {
        printf("%s: Failed to start capture\n", __func__);
    }

    while (1);

    return 0;
}