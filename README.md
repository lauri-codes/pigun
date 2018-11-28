# pigun

Lightgun project ftw

## OpenCV Setup
 - Install OpenCV 3.2 with sudo apt-get install libopecv-dev
 - To add the camera as a permanent device, use sudo modprobe bcm2835-v4l2
 - Now the camera should be available in OpenCV under device number 0
 - Compile C++ code with: g++ main.cpp -o main `pkg-config --cflags --libs opencv`