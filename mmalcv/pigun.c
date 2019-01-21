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
#include <math.h>

/*
#include <opencv2/opencv.hpp>
using namespace cv;

Mat frame, image;
Ptr<SimpleBlobDetector> detector;
std::vector<KeyPoint> keypoints;
*/

Peak lastPeaks[2];
Peak peaks[2];

/* ONE POSSIBLE WAY TO DO FAST DETECTION

the idea is to sweep a row and find a pattern of points like /\ or /- along the line
when found, record the peak max value, and column position
the system might also to find a second peak further along the row,
and it found, it will also be recorded in another data structure

at this point the row scan ended and a new row is being processed
if one peak was found in the previous line, we do not really know if it is the left ot right one!
but its column position was recorded
as more rows are processed, a peak at similar column may be found
if this one


// data structures for 2 blobs, one will naturally be left of the other!
Blob peaks[2] = 0;



// look at all rows of pixels
for each row:

    peakIdx = 0

    for each col:


        px <- img[row,col]

        if px < threshold: continue

        if px > prev:
            incFound = 1

        if px <= prev && incFound == 1:
            // we have found a complete peak
            



            // was this peak higher than the one previously found at this location?
            if
            // record the max value
            peakMax = prev

            


 */


static int pigun_detect(unsigned char *data) {

	// *** clear the peak data *** ***************************************
	
	memset(peaks, 0, sizeof(Peak) * 2);
	peaks[0].row = 0;
	peaks[1].row = 0;
	peaks[0].col = 0;
	peaks[1].col = PIGUN_RES_X-1;
	
	// *******************************************************************

	unsigned char *img = data;
	float px;
	short status = 0;
	float thr = 0.5f;
	float linesumRow, linesumCol, total, thisCol;
	float dists[2];
	unsigned short pID;
	
	
	for(int i=0; i<PIGUN_RES_Y; i++) { // loop over the rows
		
		// reset status when new pixel row starts
		status = 0;

		for(int j=0; j<PIGUN_RES_X; j++, img++) { // loop over the pixel columns

			// wait for some space after a peack is ended
			if(status < 0) {
				status++;
				continue;
			}

			px = (float)(*img)/255.0;
			
			// start recording the histogram
			if(status == 0 && px > thr) {
				status = 1;
				linesumCol = 0; linesumRow = 0;
			}
			
			if(status == 1) {
				
				if(px <= thr || j == PIGUN_RES_X-1) {
					// stop recording...
					status = -5;
					total = linesumRow;
					thisCol = linesumCol / total;
					// ... TODO
					// which peak is the closest in y to the one found now?
					dists[0] = fabsf(peaks[0].col - thisCol);
					dists[1] = fabsf(peaks[1].col - thisCol);
					pID = (dists[0]<dists[1])? 0 : 1; // get the closest peak
					peaks[pID].total += total;
					peaks[pID].tRow  += linesumRow * i;
					peaks[pID].tCol  += linesumCol;
					peaks[pID].col    = thisCol;
					peaks[pID].found  = 1;
					
				} else {
					// record the histogram
					linesumRow += px;
					linesumCol += px * j;
				}
				
			}
			
		} // end of loop over columns
	} // end of loop over rows

	peaks[0].row = peaks[0].tRow / peaks[0].total;
	peaks[0].col = peaks[0].tCol / peaks[0].total;

	peaks[1].row = peaks[1].tRow / peaks[1].total;
	peaks[1].col = peaks[1].tCol / peaks[1].total;


	return peaks[0].found + 2*peaks[1].found;
}

MMAL_POOL_T *camera_video_port_pool;
MMAL_POOL_T *preview_input_port_pool;
MMAL_PORT_T *preview_input_port = NULL;

static void preview_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {
    //printf("INFO:preview_buffer_callback buffer->length = %d\n", buffer->length);
    
    mmal_buffer_header_release(buffer);
}

static void video_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {
	
	// timing stuff
	static int loop = 0;
	static struct timespec t1;
	struct timespec t2;
	
	if (loop == 0) {
		clock_gettime(CLOCK_MONOTONIC, &t1);
	}
	
	
	MMAL_BUFFER_HEADER_T *new_buffer;
	MMAL_BUFFER_HEADER_T *preview_new_buffer;
	MMAL_POOL_T *pool = (MMAL_POOL_T *) port->userdata;
	
	
	// this should find the peaks
	int pfounds = pigun_detect(buffer->data);
	
	//memset(buffer->data, tester, PIGUN_NPX);

	// fetches a free buffer from the pool of the preview.input port
	preview_new_buffer = mmal_queue_get(preview_input_port_pool->queue);

	if (preview_new_buffer) {

		memcpy(preview_new_buffer->data, buffer->data, PIGUN_NPX); // copy only Y
        preview_new_buffer->data[PIGUN_RES_X*(int)(peaks[0].row)+(int)(peaks[0].col)] = 0;
        preview_new_buffer->data[PIGUN_RES_X*(int)(peaks[1].row)+(int)(peaks[1].col)] = 0;

		memset(&preview_new_buffer->data[PIGUN_NPX], 0, PIGUN_NPX/2); // reset U/V channels
		//memset(&preview_new_buffer->data[PIGUN_NPX+PIGUN_NPX/4], 0b10101010, PIGUN_NPX/4);
		
        preview_new_buffer->length = buffer->length;
		
		
		// i guess this is where the magic happens...
		// the newbuffer is sent to the preview.input port
		if (mmal_port_send_buffer(preview_input_port, preview_new_buffer) != MMAL_SUCCESS) {
		printf("ERROR: Unable to send buffer \n");
		}
	} else {
		printf("ERROR: mmal_queue_get (%d)\n", preview_new_buffer);
	}

    clock_gettime(CLOCK_MONOTONIC, &t2);
    int d = t2.tv_sec - t1.tv_sec;
    loop++;

	if (loop % 10 == 0) {
		//fprintf(stderr, "loop = %d \n", loop);
		printf("loop = %d, Framerate = %d fps, buffer->length = %d \n", 
            loop, loop / (d), buffer->length);
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

	//detector = SimpleBlobDetector::create();

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
