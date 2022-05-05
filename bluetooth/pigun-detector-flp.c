#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

#include "bcm_host.h"
#include "interface/vcos/vcos.h"

#include "interface/mmal/mmal.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_connection.h"

#include "pigun.h"
#include "pigun_bt.h"
#include <math.h>


#include <X11/Xlib.h>
#include <X11/Xutil.h>


//vector<bool> CHECKED(PIGUN_RES_X* PIGUN_RES_Y, false);			// Boolean array for storing which pixel locations have been checked in the blob detection
unsigned char* checked;
unsigned int* pxbuffer; // this is used by blob_detect to store the px indexes in the queue - the total allocation is PIGUN_RES_X* PIGUN_RES_Y

/**
 * Performs a breadth-first search starting from the given starting index and
 * working on the given data array. Returns a list of indices found to belong
 * to the blob surrounding the starting point.
 * 
 * WARNING: if the blob is too big, it will be cutoff and its position will not be correct!
 * 
 * return 0 if the blob was too small
 * return 1 if the blob was ok
 */
int blob_detect(int idx, unsigned char* data, const unsigned int blobID, const float threshold, const unsigned int minBlobSize, const unsigned int maxBlobSize) {
    
    unsigned int blobSize = 0;
    unsigned int x, y;
    unsigned long sumVal = 0;
    unsigned long sumX = 0, sumY = 0;
    unsigned int maxI = 0;

    // put the first px in the queue
    unsigned int qSize = 1; // length of the queue of idx to check
    pxbuffer[0] = idx;
    checked[idx] = 1;

    // Do search until stack is emptied or maximum size is reached
    while (qSize > 0 && blobSize < maxBlobSize) {
        
        // check the last element on the list
        qSize--;
        int current = pxbuffer[qSize];
        
        // do the blob position computation
        
        // Transform flattened index to 2D coordinate
        x = current % PIGUN_RES_X;
        y = current / PIGUN_RES_X;
        sumVal += data[current];
        sumX += (unsigned long)(data[current] * x);
        sumY += (unsigned long)(data[current] * y);
        if (data[current] > maxI) maxI = data[current];
        
        blobSize++;

        // check neighbours
        int other;
        other = current - PIGUN_RES_X;
        if (other >= 0 && !checked[other] && data[other] >= threshold && y > 0) {
            pxbuffer[qSize] = other;
            qSize++;
            checked[other] = 1;
        }

        other = current + PIGUN_RES_X;
        if (other >= 0 && !checked[other] && data[other] >= threshold && y < PIGUN_RES_Y-1) {
            pxbuffer[qSize] = other;
            qSize++;
            checked[other] = 1;
        }

        other = current - 1;
        if (other >= 0 && !checked[other] && data[other] >= threshold && x > 0) {
            pxbuffer[qSize] = other;
            qSize++;
            checked[other] = 1;
        }

        other = current + 1;
        if (other >= 0 && !checked[other] && data[other] >= threshold && x < PIGUN_RES_X-1) {
            pxbuffer[qSize] = other;
            qSize++;
            checked[other] = 1;
        }
    }
    // loop ends when there are no more px to check, or the blob is as big as it can be
    if (blobSize < minBlobSize) return 0;

    // code here => peak was good, save it
    
    //printf("peak found[%i]: %li %li -- %li -- %i --> ", blobID, sumX, sumY, sumVal, blobSize);

    pigun_peaks[blobID].col = ((float)sumX) / sumVal;
    pigun_peaks[blobID].row = ((float)sumY) / sumVal;
    pigun_peaks[blobID].maxI = (float)maxI;
    pigun_peaks[blobID].total = (pigun_peaks[blobID].row * PIGUN_RES_X + pigun_peaks[blobID].col);
    
    //printf("%f %f\n", pigun_peaks[blobID].col, pigun_peaks[blobID].row);

    return 1;
}

int peak_compare(const void* a, const void* b) {

    Peak* A = (Peak*)a;
    Peak* B = (Peak*)b;
	// DESCENDING ORDER NOW!
    if (B->total > A->total) return -1;
    else return 1;
}

/**
 * Clamps the given value between a minimum and maximum.
 */
int clamp(int d, int min, int max) {
  const int t = d < min ? min : d;
  return t > max ? max : t;
}

/**
 * Whenever only two peaks are present, sorts them correctly and artificially
 * adds the missing ones using an approximation.
 *
 * By creating a vector that points from the left peak to the left peak (=a), and
 * taking the cross product of this vector with the out-of-screen vector (=c), we
 * can create a new artificial axis (=b).
 */
void emulateFourPeaks() {
    
    /*
    * we have peaks 2,3 at this point:
    * 0---1
    * |   |
    * 2---3 <-- we have these two
    *
    * need to create 0,1 from them!
    */

    float ax = pigun_peaks[3].col - pigun_peaks[2].col;
    float ay = pigun_peaks[3].row - pigun_peaks[2].row;

    pigun_peaks[0].col = clamp(pigun_peaks[2].col + ay, 0, PIGUN_RES_X);// if (pigun_peaks[0].col < 0) pigun_peaks[0].col = 0;
    pigun_peaks[0].row = clamp(pigun_peaks[2].row - ax, 0, PIGUN_RES_Y);// if (pigun_peaks[0].row < 0) pigun_peaks[0].row = 0;

    pigun_peaks[1].col = clamp(pigun_peaks[3].col + ay, 0, PIGUN_RES_X);// if (pigun_peaks[1].col < 0) pigun_peaks[1].col = 0;
    pigun_peaks[1].row = clamp(pigun_peaks[3].row - ax, 0, PIGUN_RES_Y);// if (pigun_peaks[1].row < 0) pigun_peaks[1].row = 0;
}


/**
    * Detects peaks in the camera output and reports them under the global
    * "peaks"-variables.
    */
int pigun_detect(unsigned char* data) {

    /*FILE* fout = fopen("text.bin", "rb");
    fread(data, sizeof(unsigned char), PIGUN_NPX, fout);
    fclose(fout);*/

    //printf("detecting...\n");
    // These parameters have to be tuned to optimize the search
    // How many blobs to search
#ifdef PIGUN_FOUR_LEDS
    const unsigned int nBlobs = 4;
#else
    const unsigned int nBlobs = 2;
#endif
    const unsigned int dx = 4;            // How many pixels are skipped in x direction
    const unsigned int dy = 4;            // How many pixels are skipped in y direction
    const unsigned int minBlobSize = 20;  // Have many pixels does a blob have to have to be considered valid
    const unsigned int maxBlobSize =1000; // Maximum numer of pixels for a blob
    const float threshold = 130;          // The minimum threshold for pixel intensity in a blob

    const unsigned int nx = floor((float)(PIGUN_RES_X) / (float)(dx));
    const unsigned int ny = floor((float)(PIGUN_RES_Y) / (float)(dy));

    // Reset the boolean array for marking pixels as checked.
    if (checked == NULL) {
        checked = calloc(PIGUN_RES_X * PIGUN_RES_Y, sizeof(unsigned char));
        pxbuffer = malloc(sizeof(unsigned int) * PIGUN_RES_X * PIGUN_RES_Y);
    }
    memset(checked, 0, PIGUN_RES_X * PIGUN_RES_Y * sizeof(unsigned char));


    // Here the order actually matters: we loop in this order to get better cache
    // hit rate
    unsigned int blobID = 0;
    for (unsigned int j = 0; j < ny; ++j) {
        for (unsigned int i = 0; i < nx; ++i) {

            // pixel index in the buffer
            int idx = j * dy * PIGUN_RES_X + i * dx;
            int value = data[idx];

            // check if px is bright enough and not seen by the bfs before
            if (value >= threshold && !checked[idx]) {
                
                // we found a bright pixel! search nearby
                value = blob_detect(idx, data, blobID, threshold, minBlobSize, maxBlobSize);
                // peak was saved if good, move on to the next
                if (value == 1) {
                    blobID++;
                    // stop trying if we found the ones we deserve
                    if (blobID == nBlobs) break;
                }
            }
        }
        if (blobID == nBlobs) break;
    }

    // at this point we should have all the blobs we wanted
    // or maybe we are short
    if (blobID != nBlobs) {
        // if we are short, tell the callback we got an error
        return 1;
    }

    
    // Order peaks. The ordering is based on the distance of the peaks to
    // the screen corners:
    // Peak closest to top-left corner = A
    // Peak closest to bottom-left corner = B
    // Peak closest to top-right corner = C
    // Peak closest to bottom-right corner = D

#ifdef PIGUN_FOUR_LEDS
    
    /* 
        4 LED MODE:
	
        assuming we sort the peaks using the peak.total indexer (ascending),
        the camera sees:
	
        2---3      3---2
        |   |  OR  |   |
        0---1      1---0
	
        OR any similar pattern... in the end all we know is that:
        a. the first 2 peaks are the bottom LED bar
        b. the last 2 are the top LED bar
        but after sorting the ordering of top and bottom spots depends on
        camera rotation.
        we have to manually adjust them so that we get:

        0---1
        |   |
        2---3

        the aimer will use these in the correct order to compute the inverse projection!
    */

	// this will sort the peaks in descending peak.total order
	qsort(pigun_peaks, 4, sizeof(Peak), peak_compare);
	
	Peak tmp;

	// now make sure 0 is the top-left of the first two peaks
	if(pigun_peaks[0].col > pigun_peaks[1].col){
		tmp = pigun_peaks[0];
		pigun_peaks[0] = pigun_peaks[1];
		pigun_peaks[1] = tmp;
	}

	// and that 2 is the bottom-left of the last two peaks
	if(pigun_peaks[2].col > pigun_peaks[3].col){
		tmp = pigun_peaks[2];
		pigun_peaks[2] = pigun_peaks[3];
		pigun_peaks[3] = tmp;
	}

#else
    /*
    * 2 LED MODE:
    * 
    * assuming the LED bar was at the bottom of the screen,
    * we should now have peak 0 and 1 detected, in whatever order.
    * 
    * we have to manually adjust them so that we get:
    * 
    * 0---1
    * |   |
    * 2---3
    * 
    * the aimer will use these in the correct order
    * to compute the inverse projection!
    * 
    */

    // reorder 0,1 and place them in slot 2,3
    if (pigun_peaks[0].col > pigun_peaks[1].col) { // wrong ordering
        pigun_peaks[2] = pigun_peaks[1];
        pigun_peaks[3] = pigun_peaks[0];
    }
    else { // correct ordering
        pigun_peaks[2] = pigun_peaks[0];
        pigun_peaks[3] = pigun_peaks[1];
    }

    // create the 2 extra peaks
    emulateFourPeaks();
#endif

    //printf("detector done [%i]\n",blobID);
    return 0;
}


/**
    * Setup the buffer to show in the preview window. source is the original
    * frame buffer from the camera output is the buffer to send to
    * preview/input port
    */
void pigun_preview(MMAL_BUFFER_HEADER_T* output, MMAL_BUFFER_HEADER_T* source) {

    // The following line will copy the Y channel as seen by the camera
    memcpy(output->data, source->data, PIGUN_NPX); // copy only Y
	for(int i=0;i<PIGUN_NPX;i++){
        if (output->data[i] < 130)
            output->data[i] = 0;
        else
            output->data[i] /= 2;
	}
    // The following line will clean the Y channel so that only the peaks
    // etc. will be shown.
    //memset(&output->data[0], 0, PIGUN_NPX);
    
    // save frame to file
    //FILE* fout = fopen("text.bin", "wb");
    //fwrite(output->data, sizeof(unsigned char), PIGUN_NPX, fout);
    //fclose(fout);


    // Show crosshair
    output->data[PIGUN_RES_X * (int)(PIGUN_RES_Y / 2.0) + (int)(PIGUN_RES_X / 2.0)] = 255;

    // Show the peaks
//    output->data[PIGUN_RES_X * (int)(pigun_peaks[0].row) + (int)(pigun_peaks[0].col)] = 255;
    output->data[PIGUN_RES_X * (int)(pigun_peaks[1].row) + (int)(pigun_peaks[1].col)] = 255;
//    output->data[PIGUN_RES_X * (int)(pigun_peaks[2].row) + (int)(pigun_peaks[2].col)] = 255;
//    output->data[PIGUN_RES_X * (int)(pigun_peaks[3].row) + (int)(pigun_peaks[3].col)] = 255;

    // Set U/V channels to single color
    memset(&output->data[PIGUN_NPX], 128, PIGUN_NPX / 2);
}
