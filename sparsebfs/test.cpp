#include <iostream>
#include <fstream>
#include <utility>
#include <queue>
#include <vector>
#include <math.h>
#include <stdint.h>

using namespace std;

/**
 * Performs a breadth-first search starting from the given starting index and
 * working on the given data array. Returns a list of indices found to belong
 * to the blob surrounding the starting point.
 */
vector<pair<int, int> > bfs(int idx, int* data, vector<bool> &checked, const unsigned int &width, const unsigned int &height, const float &threshold) {
    vector<pair<int, int> > indices;
    queue<int> toSearch;

    // First add the starting index to queue and mark as checked
    toSearch.push(idx);
    indices.push_back(make_pair(idx, data[idx]));
    checked[idx] = true;

    // Do search until stack is emptied
    while(!toSearch.empty()) {
        int current = toSearch.front();

        // Check top, bottom, left, right. Add to stack if above treshold
        vector<int> toCheck;
        int top = current - width;
        int bottom = current + width;
        int left = current - 1;
        int right = current + 1;

        // Only check neighbours that are inside the image and not checked yet
        if (top >= 0 && !checked[top]) {
            toCheck.push_back(top);
        }
        if (bottom < width*height && !checked[bottom]) {
            toCheck.push_back(bottom);
        }
        if (left >= 0 && !((left) % width == 0) && !checked[left]) {
            toCheck.push_back(left);
        }
        if (right < width*height && !((right) % width == 0) && !checked[right]) {
            toCheck.push_back(right);
        }

        // Add each valid neighbour to stack if value over threshold
        for ( auto &i : toCheck ) {
            int iVal = data[i];
            if (iVal >= threshold) {
                toSearch.push(i);
                checked[i] = true;
                indices.push_back(make_pair(i, iVal));
            }
        }

        // Remove top one from stack
        toSearch.pop();
    }

    return indices;
}

/**
 * Used to read a binary file containing unsigned 8-bit integers into a
 * vector.
 */
vector<int> readfile(int width, int height) {
    uint8_t data[width*height];
    char buff[width*height];
    FILE *latfile;

    sprintf(buff, "%s", "../testimgs/flp/ybinary_1280x720.bin");
    latfile=fopen(buff, "r");
    fread(&(data[0]), sizeof(uint8_t), width*height, latfile);
    fclose(latfile);

    vector<int> v(data, data + sizeof data / sizeof data[0]);
    return v;
}

int main ()
{
    const unsigned int w = 1280;
    const unsigned int h = 720;
    const unsigned int nBlobs = 2;

    // These parameters have to be tuned to optimize the search
    const unsigned int dx = 4;            // How many pixels are skipped in x direction
    const unsigned int dy = 4;            // How many pixels are skipped in y direction
    const unsigned int minBlobSize = 10;  // Have many pizels does a blob have to have to be considered valid
    const float threshold = 200;          // The minimum threshold for pixel intensity in a blob

    const unsigned int nx = ceil(float(w)/float(dx));
    const unsigned int ny = ceil(float(h)/float(dy));

    // Read binary image data from file
    vector<int> data = readfile(w, h);

    // Create a boolean array for marking pixels as checked.
    vector<bool> checked(w*h, false);

    // Here the order actually matters: we loop in this order to get good cache
    // hit rate
    vector<vector<pair<int, int> > > blobs;
    for (int j=0; j < ny; ++j) {
        for (int i=0; i < nx; ++i) {
            int idx = j*dy*w + i*dx;
            float value = data[idx];
            if (value >= threshold && !checked[idx]) {
                vector<pair<int, int> > indices = bfs(idx, &data[0], checked, w, h, threshold);
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
    for ( auto &blob : blobs ) {
        float sumX = 0;
        float sumY = 0;
        float sumVal = 0;
        for ( auto &wCoord : blob ) {
            int idx = wCoord.first;
            int val = wCoord.second;

            // Transform flattened index to 2D coordinate
            int x = idx % w;
            int y = idx / w;

            // Add the weighted coordinate
            sumX += x*val;
            sumY += y*val;
            sumVal += val;
        }
        // Calculate intensity weighted mean coordinates of blobs
        float meanX = float(sumX)/sumVal;
        float meanY = float(sumY)/sumVal;
        cout << "Blob in location: " << meanX << ", " << meanY << endl;
    }
}

