#include <iostream>
#include <opencv2/opencv.hpp>
#include <sys/time.h>
#include <ctime>

typedef unsigned long long timestamp_t;

using namespace cv;

int main(int argc, char** argv) {
    VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cout << "Error opening camera" << std::endl;
        return 0;
    }
    std::cout << "Camera opened" << std::endl;


    cap.set(CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(CAP_PROP_FRAME_HEIGHT, 720);
    cap.set(CAP_PROP_FPS, 90);

	struct timeval tv, tv2;
	gettimeofday(&tv, NULL);
	
    //int frameCount = 0;
    for(int frameCount=0; frameCount<300; frameCount++) {
    //while(1) {
        
        // Capture frame-by-frame
        Mat frame;
        cap >> frame;

        // If the frame is empty, break immediately
        if (frame.empty()) {
            std::cout << "failed!" << std::endl;
            break;
        }

        // Display the resulting frame
        //imshow( "Frame", frame );

        // Press  ESC on keyboard to exit
        /*char c=(char)waitKey(1);  // Waits for a key press for 1 millisecond. 0 means wait infinitely...
        if (c==27) {
            break;
        }*/

        //std::cout << "Frame read " << frameCount << std::endl;
        //++frameCount;
    }
    
	
	gettimeofday(&tv2, NULL);
	double timer = (double)((tv2.tv_usec - tv.tv_usec) / 1000000.0);
	timer += (double)(tv2.tv_sec - tv.tv_sec);
	
	std::cout << "fps: " << 300.0/timer <<std::endl;
    
    // When everything done, release the video capture object
    cap.release();
    return 1;
}
