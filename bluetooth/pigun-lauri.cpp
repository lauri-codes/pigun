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


string GetLineFromCin() {
    std::string line;
    std::getline(std::cin, line);
    return line;
}

vector<bool> CHECKED(PIGUN_RES_X* PIGUN_RES_Y, false);			// Boolean array for storing which pixel locations have been checked in the blob detection
auto fut = std::async(std::launch::async, GetLineFromCin);		// Asyncronous task for listening to key input
Vector3f bottomLeft(0, 0, 0);                                   // Stores the relative position of the screen left edge
Vector3f topRight(0, 0, 0);                                     // Stores the relative position of the screen left edge
Vector2f pigun_aim;                                             // stores the aiming position (normalised coordinates?)


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

/**
 * Transforms the crosshair position in the camera space into the custom
 * coordinate system defined by the four corners of the screen.
 */
Vector2f getAim()
{
    // Origin in transformed system
    Vector2f origin(pigun_peaks[1].col, pigun_peaks[1].row);
    // Crosshair in original system
    Vector2f x(float(PIGUN_RES_X) / 2.0f, float(PIGUN_RES_Y) / 2.0f);
    // Basis vectors in transformed system
    Vector2f a(pigun_peaks[3].col - pigun_peaks[1].col, pigun_peaks[3].row - pigun_peaks[1].row);
    Vector2f b(pigun_peaks[0].col - pigun_peaks[1].col, pigun_peaks[0].row - pigun_peaks[1].row);
    // Inverted basis matrix for transformed system. Uses the direct formula
    // for the inverse of a 2x2 matrix.
    Matrix2f BInverse;
    BInverse << b.y(), -b.x(),
        -a.y(), a.x();
    BInverse *= 1.0f/(a.x() * b.y() - b.x() * a.y());
    // Returns the crosshair position in transformed system.
    Vector2f xPrime = BInverse * (x - origin);

    // Clip to be between 0 and 1
    xPrime = xPrime.cwiseMax(0).cwiseMin(1);

    return xPrime;
}

// Calculate the coordinate system origin
Vector3f getOrigin()
{
    // Decide which light is which
    float x0 = pigun_peaks[0].col;
    float x1 = pigun_peaks[1].col;
    int aIndex, bIndex;
    if (x0 <= x1) {
        aIndex = 0;
        bIndex = 1;
    }
    else {
        aIndex = 1;
        bIndex = 0;
    }

    Vector3f a(pigun_peaks[aIndex].col, PIGUN_RES_Y - pigun_peaks[aIndex].row, 0);
    Vector3f b(pigun_peaks[bIndex].col, PIGUN_RES_Y - pigun_peaks[bIndex].row, 0);
    Vector3f origin = 0.5 * (b + a);

    return origin;
}

Vector3f centerToAxes(pair<Vector3f, Vector3f> axes, Vector3f origin)
{
    // Calculate the frame axis and origin
    Vector3f center(PIGUN_RES_X / 2, PIGUN_RES_Y / 2, 0.0);

    // Save the calibration location with respect to the origin at the
    // center of the leds and the axes determined by the leds
    Vector3f disp = center - origin;
    float inorm = axes.first.norm();
    float jnorm = axes.second.norm();
    float dx = disp.dot(axes.first) / (inorm * inorm);
    float dy = disp.dot(axes.second) / (jnorm * jnorm);
    return Vector3f(dx, dy, 0);
}



extern "C" {

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


    void pigun_calculate_aim() {
        Vector2f aim = getAim();
        float x = aim.x();                                                 
        float y = aim.y();
#ifdef PIGUN_MOUSE
        mouseMove(x, y);
#endif

        // Send the coordinate to global variable used by bluetooth              
        global_pigun_report.x = (short)((2 * x - 1) * 32767);                    
        global_pigun_report.y = (short)((2 * y - 1) * 32767);
    }

    // Setup the buffer to show in the preview window.
    // source is the original frame buffer from the camera
    // output is the buffer to send to preview/input port
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
