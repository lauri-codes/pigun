#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;

int main(int argc, char** argv) {
    VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cout << "Error opening camera" << std::endl;
        return 0;
    }
    std::cout << "Camera opened" << std::endl;


    cap.set(CAP_PROP_FRAME_WIDTH, 640);
    cap.set(CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(CAP_PROP_FPS, 90);

    //int frameCount = 0;
    while(1) {
        
        // Capture frame-by-frame
        Mat frame;
        cap >> frame;

        // If the frame is empty, break immediately
        if (frame.empty()) {
            break;
        }

        // Display the resulting frame
        imshow( "Frame", frame );

        // Press  ESC on keyboard to exit
        char c=(char)waitKey(1);  // Waits for a key press for 1 millisecond. 0 means wait infinitely...
        if (c==27) {
            break;
        }

        //std::cout << "Frame read " << frameCount << std::endl;
        //++frameCount;
    }
    // When everything done, release the video capture object
    cap.release();
    return 1;
}