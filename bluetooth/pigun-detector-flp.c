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
    double sumVal = 0;
    double sumX = 0, sumY = 0;
    int maxI = 0;

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
        x = idx % PIGUN_RES_X;
        y = idx / PIGUN_RES_X;
        sumVal += data[current];
        sumX += data[current] * x;
        sumY += data[current] * y;
        if (data[current] > maxI) maxI = data[current];
        
        blobSize++;

        // check neighbours
        int other;
        other = current - PIGUN_RES_X;
        if (other >= 0 && !checked[other] && data[other] >= threshold) {
            pxbuffer[qSize] = other;
            qSize++;
            checked[other] = 1;
        }

        other = current + PIGUN_RES_X;
        if (other >= 0 && !checked[other] && data[other] >= threshold) {
            pxbuffer[qSize] = other;
            qSize++;
            checked[other] = 1;
        }

        other = current - 1;
        if (other >= 0 && !checked[other] && data[other] >= threshold) {
            pxbuffer[qSize] = other;
            qSize++;
            checked[other] = 1;
        }

        other = current + 1;
        if (other >= 0 && !checked[other] && data[other] >= threshold) {
            pxbuffer[qSize] = other;
            qSize++;
            checked[other] = 1;
        }
    }
    // loop ends when there are no more px to check, or the blob is as big as it can be
    if (blobSize < minBlobSize) return 0;

    // code here => peak was good, save it
    sumX /= sumVal;
    sumY /= sumVal;

    pigun_peaks[blobID].col = (float)sumX;
    pigun_peaks[blobID].row = (float)sumY;
    pigun_peaks[blobID].maxI = (float)maxI;
    pigun_peaks[blobID].total = (float)(sumX * PIGUN_RES_X + sumY);
    return 1;
}

int peak_compare(const void* a, const void* b) {

    Peak* A = (Peak*)a;
    Peak* B = (Peak*)b;

    if (B->total < A->total) return -1;
    else return 1;
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
    
    // Move the peaks: B->D, A->B.
    //pigun_peaks[3] = pigun_peaks[1];
    //pigun_peaks[1] = pigun_peaks[0];
    // in my code the two peaks are already B,D in their right slots (1,3) and in the right order

    // Add the missing two peaks.
    /*
    bl = 1x,1y,0
    br = 3x, 3y, 0
    a = br-bl
    c = 0,0,1
    b = a x c = ay cz - az cy, az cx - ax cz,  ax cy - ay cx = ay, -ax, 0
    */

    float ax = pigun_peaks[3].col - pigun_peaks[1].col;
    float ay = pigun_peaks[3].row - pigun_peaks[1].row;

    pigun_peaks[0].col = pigun_peaks[1].col + ay; if (pigun_peaks[0].col < 0) pigun_peaks[0].col = 0;
    pigun_peaks[0].row = pigun_peaks[1].row - ax; if (pigun_peaks[0].row < 0) pigun_peaks[0].row = 0;

    pigun_peaks[2].col = pigun_peaks[3].col + ay; if (pigun_peaks[2].col < 0) pigun_peaks[2].col = 0;
    pigun_peaks[2].row = pigun_peaks[3].row - ax; if (pigun_peaks[2].row < 0) pigun_peaks[2].row = 0;
}


/**
    * Detects peaks in the camera output and reports them under the global
    * "peaks"-variables.
    */
int pigun_detect(unsigned char* data) {

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
    const unsigned int minBlobSize = 5;   // Have many pixels does a blob have to have to be considered valid
    const unsigned int maxBlobSize = 500; // Maximum numer of pixels for a blob
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
    // 4 led mode:
    // we can convert peak x/y to an index (stored in peak.total)
    // and then sort the peaks ACBD (0,2,1,3)

    // this will sort the peaks my ascending peak.total order
    qsort(pigun_peaks, 4, sizeof(Peak), peak_compare);
    
    // now the are ordered 0,1,2,3
    Peak tmp = pigun_peaks[1];
    pigun_peaks[1] = pigun_peaks[2];
    pigun_peaks[2] = tmp;

#else
    // two blob case: flip them if 0 is right of 1
    // the two detected peaks are actually B and D - assuming the led bar is below!
    // but they are stored in 0,1 instead of 1,3
    if (pigun_peaks[0].col > pigun_peaks[1].col) {
        // in this case 0,1 -> 3,1
        pigun_peaks[3] = pigun_peaks[0];
    }
    else { // correct ordering
        pigun_peaks[3] = pigun_peaks[1];
        pigun_peaks[1] = pigun_peaks[0];
    }


    // Two peak mode: emulate A and C
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
    //memcpy(output->data, source->data, PIGUN_NPX); // copy only Y
    // The following line will clean the Y channel so that only the peaks
    // etc. will be shown.
    memset(&output->data[0], 0, PIGUN_NPX);

    // Show crosshair
    output->data[PIGUN_RES_X * (int)(PIGUN_RES_Y / 2.0) + (int)(PIGUN_RES_X / 2.0)] = 255;

    // Show the peaks
    output->data[PIGUN_RES_X * (int)(pigun_peaks[0].row) + (int)(pigun_peaks[0].col)] = 255;
    output->data[PIGUN_RES_X * (int)(pigun_peaks[1].row) + (int)(pigun_peaks[1].col)] = 255;
    output->data[PIGUN_RES_X*(int)(pigun_peaks[2].row)+(int)(pigun_peaks[2].col)] = 255;
    output->data[PIGUN_RES_X*(int)(pigun_peaks[3].row)+(int)(pigun_peaks[3].col)] = 255;

    // Set U/V channels to single color
    memset(&output->data[PIGUN_NPX], 128, PIGUN_NPX / 2);
}
