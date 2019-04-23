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
#include <Eigen/Dense>
#include "bcm_host.h"
#include "interface/vcos/vcos.h"

#include "interface/mmal/mmal.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_connection.h"

#include "pigun.h"
#include <math.h>

#include <iostream>
#include <fstream>
#include <utility>
#include <queue>
#include <vector>
#include <chrono>
#include <future>
#include <string>
#include <algorithm>
#include <stdint.h>

using namespace std;
using namespace Eigen;

string GetLineFromCin() {
    std::string line;
    std::getline(std::cin, line);
    return line;
}

Peak lastPeaks[2];
Peak peaks[3];
vector<bool> CHECKED(PIGUN_RES_X*PIGUN_RES_Y, false);       // Boolean array for storing which pixel locations have been checked in the blob detection
auto fut = std::async(std::launch::async, GetLineFromCin);  // Asyncronous task for listening to key input
pair<float,float> bottomLeft;                               // Stores the relative position of the screen left edge
pair<float,float> topRight;                                 // Stores the relative position of the screen left edge

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

/**
 * Performs a breadth-first search starting from the given starting index and
 * working on the given data array. Returns a list of indices found to belong
 * to the blob surrounding the starting point.
 */
vector<pair<int, int> > bfs(int idx, unsigned char *data, const float &threshold) {
    vector<pair<int, int> > indices;
    queue<int> toSearch;

    // First add the starting index to queue and mark as checked
    toSearch.push(idx);
    indices.push_back(make_pair(idx, data[idx]));
    CHECKED[idx] = true;

    // Do search until stack is emptied
    while(!toSearch.empty()) {
        int current = toSearch.front();

        // Check top, bottom, left, right. Add to stack if above treshold
        vector<int> toCheck;
        int top = current - PIGUN_RES_X;
        int bottom = current + PIGUN_RES_X;
        int left = current - 1;
        int right = current + 1;

        // Only check neighbours that are inside the image and not checked yet
        if (top >= 0 && !CHECKED[top]) {
            toCheck.push_back(top);
        }
        if (bottom < PIGUN_RES_X*PIGUN_RES_Y && !CHECKED[bottom]) {
            toCheck.push_back(bottom);
        }
        if (left >= 0 && !((left) % PIGUN_RES_X == 0) && !CHECKED[left]) {
            toCheck.push_back(left);
        }
        if (right < PIGUN_RES_X*PIGUN_RES_Y && !((right) % PIGUN_RES_X == 0) && !CHECKED[right]) {
            toCheck.push_back(right);
        }

        // Add each valid neighbour to stack if value over threshold
        for ( auto &i : toCheck ) {
            int iVal = data[i];
            if (iVal >= threshold) {
                toSearch.push(i);
                CHECKED[i] = true;
                indices.push_back(make_pair(i, iVal));
            }
        }

        // Remove top one from stack
        toSearch.pop();
    }

    return indices;
}

static int pigun_detect2(unsigned char *data) {


    // These parameters have to be tuned to optimize the search
    const unsigned int nBlobs = 2;        // How many blobs to search
    const unsigned int dx = 4;            // How many pixels are skipped in x direction
    const unsigned int dy = 4;            // How many pixels are skipped in y direction
    const unsigned int minBlobSize = 5;   // Have many pizels does a blob have to have to be considered valid
    const float threshold = 80;           // The minimum threshold for pixel intensity in a blob

    const unsigned int nx = ceil(float(PIGUN_RES_X)/float(dx));
    const unsigned int ny = ceil(float(PIGUN_RES_Y)/float(dy));

    // Reset the boolean array for marking pixels as checked.
    std::fill(CHECKED.begin(), CHECKED.end(), false);

    // Here the order actually matters: we loop in this order to get better cache
    // hit rate
    vector<vector<pair<int, int> > > blobs;
    for (int j=0; j < ny; ++j) {
        for (int i=0; i < nx; ++i) {
            int idx = j*dy*PIGUN_RES_X + i*dx;
            int value = data[idx];
            if (value >= threshold && !CHECKED[idx]) {
                vector<pair<int, int> > indices = bfs(idx, data, threshold);
                int blobSize = indices.size();
                if (blobSize >= minBlobSize) {
                    blobs.push_back(indices);
                }
            }
            if (blobs.size() == nBlobs) {
                break;
            }
        }
        if (blobs.size() == nBlobs) {
            break;
        }
    }

    // After blobs have been found, calculate their mean coordinate
    int iBlob = 0;
    for ( auto &blob : blobs ) {
        float sumX = 0;
        float sumY = 0;
        float sumVal = 0;
        float maxI = 0;
        for ( auto &wCoord : blob ) {
            int idx = wCoord.first;
            int val = wCoord.second;

            // Save maximum intensity
            if (val > maxI) {
                maxI = val;
            }

            // Transform flattened index to 2D coordinate
            int x = idx % PIGUN_RES_X;
            int y = idx / PIGUN_RES_X;

            // Add the weighted coordinate
            sumX += x*val;
            sumY += y*val;
            sumVal += val;
        }
        // Calculate intensity weighted mean coordinates of blobs
        float meanX = float(sumX)/sumVal;
        float meanY = float(sumY)/sumVal;
        //cout << "Blob in location: " << meanX << ", " << meanY << " -- " << sumVal << endl;

        // Store in global peaks variable
        peaks[iBlob].row = meanY;
        peaks[iBlob].col = meanX;
        peaks[iBlob].maxI = maxI;
        ++iBlob;
    }
}

static void preview_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {
    //printf("INFO:preview_buffer_callback buffer->length = %d\n", buffer->length);

    mmal_buffer_header_release(buffer);
}

static void video_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {


	// Measure elapsed time and print instantaneous FPS
	static int loop = 0;                // Global variable for loop number
	static struct timespec t1 = {0, 0}; // Global variable for previous loop time
	struct timespec t2;                 // Current loop time
	float dt;                           // Elapsed time between frames

    clock_gettime(CLOCK_MONOTONIC, &t2);
    if (t1.tv_sec != 0 && t1.tv_nsec != 0) {
        dt = t2.tv_sec + (t2.tv_nsec / 1000000000.0) - (t1.tv_sec + (t1.tv_nsec / 1000000000.0));
    }
    loop++;

	if (loop > 0 && loop % 50 == 0) {
		printf("loop = %d, Framerate = %f fps, buffer->length = %d \n",
            loop, 1.0 / dt, buffer->length);
	}
	t1 = t2;

	MMAL_BUFFER_HEADER_T *new_buffer;
	MMAL_BUFFER_HEADER_T *preview_new_buffer;
	MMAL_POOL_T *pool = (MMAL_POOL_T *) port->userdata;

	// This should find the peaks
	//int pfounds = pigun_detect(buffer->data);  // Filippo
    pigun_detect2(buffer->data);   		         // Lauri

    // Calculate the distance to the lights
    float fovX = 2*1280.0/3280.0*62.2; // binning*acquired resolution/full resolution*full fov
    float fovY = 2*720.0/2464.0*48.8;
    float anglesPerPixelX = fovX/320;
    float anglesPerPixelY = fovY/180;
    Vector3f a(peaks[0].col, -peaks[0].row, 0);
    Vector3f b(peaks[1].col, -peaks[1].row, 0);
    Vector3f dist = a-b;
    dist.x() *= anglesPerPixelX;
    dist.y() *= anglesPerPixelY;
    float abAngle = dist.norm();
    float aI = peaks[0].maxI;
    float bI = peaks[1].maxI;
    float abRatio = aI/bI;
    //cout << "a-b angle: " << abAngle << ", a-b intensity ratio: " << abRatio << ", a intensity: "  << aI << endl;
    //cout << "b intensity: " << bI << ", a intensity: " << aI << endl;

    // This is a funny one. At the beginning of the program we have launched an
    // asynchonous task that listens to keyboard input. Then for each loop we
    // check the status of that task and if finished we print the result an
    // spawn a new task.
    //
    if (fut.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        auto line = fut.get();

        // Set a new line. Subtle race condition between the previous line
        // and this. Some lines could be missed. To aleviate, you need an
        // io-only thread. I'll give an example of that as well.
        fut = std::async(std::launch::async, GetLineFromCin);

        // If l or r was entered, calibrate
        if (line == "a" || line == "s") {

            // Calculate screen x distance between LEDs
            float ds = sqrt(pow(peaks[0].col - peaks[1].col, 2) + pow(peaks[0].row - peaks[1].row, 2));

            // Calculate image center (gun pointer) location from center of peaks
            float ledXCenter = (peaks[0].col + peaks[1].col)/2;
            float ledYCenter = (peaks[0].row + peaks[1].row)/2;
            float gunDx = PIGUN_RES_X/2 - ledXCenter;
            float gunDy = PIGUN_RES_Y/2 - ledYCenter;

            // Save relative distance with respect to peak center and separation
            if (line == "a") {
                bottomLeft = make_pair(gunDx/ds, gunDy/ds);
                cout << "Bottom left calibrated at: " << bottomLeft.first << ", " << bottomLeft.second << endl;
            } else if (line == "s") {
                topRight = make_pair(gunDx/ds, gunDy/ds);
                cout << "Top right calibrated at: " << topRight.first << ", " << topRight.second << endl;
            }
        }

    }

	//memset(buffer->data, tester, PIGUN_NPX);

	// fetches a free buffer from the pool of the preview.input port
	preview_new_buffer = mmal_queue_get(preview_input_port_pool->queue);

	if (preview_new_buffer) {

		memcpy(preview_new_buffer->data, buffer->data, PIGUN_NPX); // copy only Y

		// Show crosshair
        preview_new_buffer->data[PIGUN_RES_X*(int)(PIGUN_RES_Y/2.0)+(int)(PIGUN_RES_X/2.0)] = 255;

		// Show the peaks with black dots
        preview_new_buffer->data[PIGUN_RES_X*(int)(peaks[0].row)+(int)(peaks[0].col)] = 0;
        preview_new_buffer->data[PIGUN_RES_X*(int)(peaks[1].row)+(int)(peaks[1].col)] = 0;
        //preview_new_buffer->data[PIGUN_RES_X*(int)(peaks[2].row)+(int)(peaks[2].col)] = 0;

        // Show the screen limits with lines
        float ds = sqrt(pow(peaks[0].col - peaks[1].col, 2) + pow(peaks[0].row - peaks[1].row, 2));
        float ledXCenter = (peaks[0].col + peaks[1].col)/2;
        float ledYCenter = (peaks[0].row + peaks[1].row)/2;
        //cout << ledXCenter << endl;
        //cout << ledYCenter << endl;
        float left = ledXCenter + bottomLeft.first*ds;
        float right = ledXCenter + topRight.first*ds;
        float top = ledYCenter + topRight.second*ds;
        float bottom = ledYCenter + bottomLeft.second*ds;
        //int leftPeakIdx, rightPeakIdx, bottomPeakIdx, topPeakIdx;
        //float dy;
        //if (peaks[0].col <= peaks[1].col) {
            //leftPeakIdx = 0;
            //rightPeakIdx = 1;
        //} else {
            //leftPeakIdx = 1;
            //rightPeakIdx = 0;
        //}
        //if (peaks[0].row <= peaks[1].row) {
            //topPeakIdx = 0;
            //bottomPeakIdx = 1;
        //} else {
            //topPeakIdx = 1;
            //bottomPeakIdx = 0;
        //}
        //float leftPeak = peaks[leftPeakIdx].col;
        //float rightPeak = peaks[rightPeakIdx].col;
        //float topPeak = peaks[topPeakIdx].row;
        //float bottomPeak = peaks[bottomPeakIdx].row;
        //float dx = rightPeak - leftPeak;
        //if (leftPeakIdx == topPeakIdx) {
            //dy = bottomPeak - topPeak;
        //} else {
            //dy = topPeak - bottomPeak;
        //}
        //float roll = 180/PI*atan2(dy, dx);
        //cout << roll << endl;
	/*
        // Decide which peak is which. The decision is based on the correct
        // handedness of the peaks and the minimum rotation with respect to
        // up-direction.
        float minAngle = 180;
        array<int, 3> abc;
        vector<int> indices{0,1,2};
        do {
            int i = indices[0];
            int j = indices[1];
            int k = indices[2];
            Vector3f a(peaks[i].col, -peaks[i].row, 0);
            Vector3f b(peaks[j].col, -peaks[j].row, 0);
            Vector3f c(peaks[k].col, -peaks[k].row, 0);

            // Calculate the cross-product to see if the handedness is correct
            Vector3f ba = a-b;
            Vector3f bc = c-b;
            Vector3f cross = ba.cross(bc);
            float z = cross.z();

            // Calculate angle of ba vector to up direction
            if (z <= 0) {
                float angle = 90 - abs(atan(ba.y()/ba.x())*180.0/PI);
                if (angle < minAngle) {
                    abc = {i, j, k};
                }
            }
        } while (next_permutation(indices.begin(), indices.end()));

        Peak a = peaks[abc[0]];
        Peak b = peaks[abc[1]];
        Peak c = peaks[abc[2]];
        preview_new_buffer->data[PIGUN_RES_X*(int)(a.row+1)+(int)(a.col)] = 0;
        preview_new_buffer->data[PIGUN_RES_X*(int)(b.row+2)+(int)(b.col)] = 0;
        preview_new_buffer->data[PIGUN_RES_X*(int)(c.row+3)+(int)(c.col)] = 0;
	*/
        preview_new_buffer->data[PIGUN_RES_X*(int)(bottom)+(int)(left)] = 255;
        preview_new_buffer->data[PIGUN_RES_X*(int)(top)+(int)(right)] = 255;
        preview_new_buffer->data[PIGUN_RES_X*(int)(ledYCenter)+(int)(ledXCenter)] = 255;
        preview_new_buffer->data[PIGUN_RES_X*(int)(ledYCenter)+(int)(ledXCenter)] = 255;
        //preview_new_buffer->data[PIGUN_RES_X*(int)(ledYCenter+bottomLeft.second)+(int)(ledXCenter+bottomLeft.first)] = 255;
        //preview_new_buffer->data[PIGUN_RES_X*(int)(ledYCenter+topRight.second)+(int)(ledXCenter+topRight.first)] = 255;

		memset(&preview_new_buffer->data[PIGUN_NPX], 0b10101010, PIGUN_NPX/2); // reset U/V channels
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

    // disable exposure mode
    pigun_camera_exposuremode(camera, 0);

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
        //param.set |= MMAL_DISPLAY_SET_FULLSCREEN;
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

    pigun_camera_awb(camera, 0);
    pigun_camera_awb_gains(camera, 1, 1);
    pigun_camera_blur(camera, 1);

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
