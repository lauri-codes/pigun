# PIGUN Bluetooth code

This folder contains some of the code from btstack, adapted for pigun.

The program runs in two threads:

- main function with bluetooth process (main_pigun.c)
- pigun camera acquisition cycle in a separate pthread (pigun.c)

The pigun camera acquisition cycle does:
- inits the camera with libmmal
- runs an infinite loop to get frames and compute aiming coordinates
- execute the video callback when camera frames are ready
  - compute peaks from frame by calling `pigun_detect(buffer->data)`
    - store peak info in global var `Peak *pigun_peaks`
  - compute aiming coordinates from peaks in `pigun_calculate_aim()`
    - aiming coordinates and button states are saved to a global var `pigun_report_t global_pigun_report`
   
The bluetooth thread will read `pigun_report_t global_pigun_report` and send HID reports when paired.


# Compiling

The GPIO system will be made with libBCM2835 so need to compile and install that first.

PIGUN main code is compiled with the following commands:

```
make bluetooth
make pigun
make pigun-detector-XXX
make pigun-aimer-XXX
make link
```

The detection module of choice (pigun-detector-XXX) has to be defined in the makefile, and it is supposed to compile to `pigun_detector.o`.
Similarly, the aimer module of choice (pigun-aimer-XXX) has to be defined in the makefile, and it is supposed to compile to `pigun_aimer.o`.

The dummy module includes both a dummy detector and aimer which do nothing at all, and compiles to `pigun_kernel.o` while removing any other detector and aimer object file.

## Compiler Flags

- PIGUN_PREVIEW: define to compile all the mmal preview parts that will show the camera preview (for debugging)
- PIGUN_MOUSE: define to compile the mouse-move parts of the code (requires a screen)


## Detection Module

A detection module must define these functions:

- `int pigun_detect(unsigned char* data)`
- `void pigun_preview(MMAL_BUFFER_HEADER_T* output, MMAL_BUFFER_HEADER_T* source)`

`pigun_preview` is required when compiling with PIGUN_PREVIEW defined.


## Aimer Module

The aimer module must define:

- `void pigun_calculate_aim()`

which has to compute the aiming position (x,y) and write it to the HID report.