# pigun

Lightgun project ftw

## OpenCV Setup
 - Install OpenCV 3.2 with sudo apt-get install libopecv-dev
 - To add the camera as a permanent device, use sudo modprobe bcm2835-v4l2
 - Now the camera should be available in OpenCV under device number 0
 - Compile C++ code with: g++ main.cpp -o main `pkg-config --cflags --libs opencv`
 
timing result (removed imshow!): ~59 FPS (avg over 300 frames) without any frame processing @ 640x480

timing result (removed imshow!): ~20 FPS (avg over 300 frames) without any frame processing @ 1280x720


## MMAL test

timing capture at 1280x720@90fps --> 320x180 --> opencv CCL --> 5fps

timing capture at 1280x720@90fps --> 320x180 --> run 1 loop  on all pixels with threshold conditioncheck --> 65fps

timing capture at 1280x720@90fps --> 320x180 --> run 2 loops on all pixels with threshold conditioncheck --> 50fps






 ## Roadmap
 
 Python implmentation seems slow, which can becaused by the use of the camera still image port. This does some heavy denoising and it is naturally slow.
 - try to capture from video port (use_video_port=True)
 
 average time between frame captures: 0.052s (avg over 100 caps)
 array reshape: 0.057s 
 binarization: 0.055s (?!)
 connected components: 0.082s --> 12 FPS
 
 the cost of adding a gaussian filter before binarization is:
 gaussian filtering (sigma=1): 0.11s
 gaussian filtering (sigma=10): 0.33s

 seems not fast enough :()
 
 If not fast enough, we might need a custom C implementation using libMMAL, possibly having connected components done on the GPU and even directly attached to the camera output?!
 
