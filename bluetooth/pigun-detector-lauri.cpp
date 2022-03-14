#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <eigen3/Eigen/Dense>

#include "bcm_host.h"
#include "interface/vcos/vcos.h"

#include "interface/mmal/mmal.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_connection.h"

#include "pigun.h"
#include "pigun_bt.h"
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

#include <X11/Xlib.h>
#include <X11/Xutil.h>

using namespace std;
using namespace Eigen;

vector<bool> CHECKED(PIGUN_RES_X* PIGUN_RES_Y, false);			// Boolean array for storing which pixel locations have been checked in the blob detection


/**
 * Performs a breadth-first search starting from the given starting index and
 * working on the given data array. Returns a list of indices found to belong
 * to the blob surrounding the starting point.
 */
vector<pair<int, int> > bfs(int idx, unsigned char* data, const float& threshold, const unsigned int& maxBlobSize) {
    vector<pair<int, int> > indices;
    queue<int> toSearch;

    // First add the starting index to queue and mark as checked
    toSearch.push(idx);
    indices.push_back(make_pair(idx, data[idx]));
    CHECKED[idx] = true;

    // Do search until stack is emptied or maximum size is reached
    while (!toSearch.empty() && indices.size() < maxBlobSize) {
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
        if (bottom < PIGUN_RES_X * PIGUN_RES_Y && !CHECKED[bottom]) {
            toCheck.push_back(bottom);
        }
        if (left >= 0 && !((left) % PIGUN_RES_X == 0) && !CHECKED[left]) {
            toCheck.push_back(left);
        }
        if (right < PIGUN_RES_X * PIGUN_RES_Y && !((right) % PIGUN_RES_X == 0) && !CHECKED[right]) {
            toCheck.push_back(right);
        }

        // Add each valid neighbour to stack if value over threshold
        for (auto& i : toCheck) {
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

extern "C" {

    /**
     * Detects peaks in the camera output and reports them under the global
     * "peaks"-variables.
     */
    int pigun_detect(unsigned char* data) {
        // These parameters have to be tuned to optimize the search
        const unsigned int nBlobs = 2;        // How many blobs to search
        const unsigned int dx = 4;            // How many pixels are skipped in x direction
        const unsigned int dy = 4;            // How many pixels are skipped in y direction
        const unsigned int minBlobSize = 5;   // Have many pixels does a blob have to have to be considered valid
        const unsigned int maxBlobSize = 500; // Maximum numer of pixels for a blob
        const float threshold = 130;          // The minimum threshold for pixel intensity in a blob

        const unsigned int nx = ceil(float(PIGUN_RES_X) / float(dx));
        const unsigned int ny = ceil(float(PIGUN_RES_Y) / float(dy));

        // Reset the boolean array for marking pixels as checked.
        std::fill(CHECKED.begin(), CHECKED.end(), false);

        // Here the order actually matters: we loop in this order to get better cache
        // hit rate
        vector<vector<pair<int, int> > > blobs;
        for (int j = 0; j < ny; ++j) {
            for (int i = 0; i < nx; ++i) {
                int idx = j * dy * PIGUN_RES_X + i * dx;
                int value = data[idx];
                if (value >= threshold && !CHECKED[idx]) {
                    vector<pair<int, int> > indices = bfs(idx, data, threshold, maxBlobSize);
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
        for (auto& blob : blobs) {
            float sumX = 0;
            float sumY = 0;
            float sumVal = 0;
            float maxI = 0;
            for (auto& wCoord : blob) {
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
                sumX += x * val;
                sumY += y * val;
                sumVal += val;
            }
            // Calculate intensity weighted mean coordinates of blobs
            float meanX = float(sumX) / sumVal;
            float meanY = float(sumY) / sumVal;
            //cout << "Blob in location: " << meanX << ", " << meanY << " -- " << sumVal << endl;

            // Store in global pigun_peaks variable
            pigun_peaks[iBlob].row = meanY;
            pigun_peaks[iBlob].col = meanX;
            pigun_peaks[iBlob].maxI = maxI;
            ++iBlob;
        }

        // Order the peaks: a=top left, b=bottom left, c=top right, d=bottom_right
        Peak aa, bb, cc, dd;
        if (pigun_peaks[0].col < pigun_peaks[1].col) {
            bb = pigun_peaks[0];
            dd = pigun_peaks[1];
        }
        else {
            bb = pigun_peaks[1];
            dd = pigun_peaks[0];
        }
        aa.row = std::max(bb.row - 30, 0.0f);
        aa.col = bb.col;
        cc.row = std::max(dd.row - 30, 0.0f);
        cc.col = dd.col;

        // Calculate transformation matrix from camera to rectangle
        pigun_peaks[0] = aa;
        pigun_peaks[1] = bb;
        pigun_peaks[2] = cc;
        pigun_peaks[3] = dd;
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
}