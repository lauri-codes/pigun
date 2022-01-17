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
make pigun-XXX # like pigun-dummy
make link
```

The detection module of choice (pigun-XXX) has to be defined in the makefile, and it is supposed to compile to `pigun_kernel.o`.
The dummy module does nothing at all.

If defined, compiler flag PIGUN_PREVIEW will compile all the mmal preview parts that will show the camera preview (for debugging). 
The main cycle will call `pigun_preview(preview_new_buffer, buffer)` which must be defined in the custom detector module.
