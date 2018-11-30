# pigun

Lightgun project ftw

## OpenCV Setup
 - Install OpenCV 3.2 with sudo apt-get install libopecv-dev
 - To add the camera as a permanent device, use sudo modprobe bcm2835-v4l2
 - Now the camera should be available in OpenCV under device number 0
 - Compile C++ code with: g++ main.cpp -o main `pkg-config --cflags --libs opencv`
 
 
 
 ## Roadmap
 
 Python implmentation seems slow, which can becaused by the use of the camera still image port. This does some heavy denoising and it is naturally slow.
 - try to capture from video port (use_video_port=True)
 
 If not fast enough, we might need a custom C implementation using libMMAL, possibly having connected components done on the GPU and even directly attached to the camera output?!
 
