#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <eigen3/Eigen/Dense>

#include "bcm_host.h"
#include "interface/vcos/vcos.h"

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


/**
 * Transforms a position in the camera space to the correponsding position in
 * screen space.
 */
Vector2f toScreen(Vector2f cameraPos)
{
    // Origin in transformed system
    Vector2f origin(pigun_peaks[1].col, pigun_peaks[1].row);
    // Crosshair in original system
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
    Vector2f screenPos = BInverse * (cameraPos - origin);

    // Clip to be between 0 and 1
    screenPos = screenPos.cwiseMax(0).cwiseMin(1);

    return screenPos;
}

extern "C" {

    /**
     * Used to calculate the mouse/joystick position in screen coordinates and
     * send it to bluetooth.
     */
    void pigun_calculate_aim() {
        Vector2f crosshair(float(PIGUN_RES_X) / 2.0f, float(PIGUN_RES_Y) / 2.0f);
        Vector2f aim = toScreen(crosshair);
        float x = aim.x();                                                 
        float y = aim.y();
#ifdef PIGUN_MOUSE
        mouseMove(x, y);
#endif

        // Send the coordinate to global variable used by bluetooth              
        global_pigun_report.x = (short)((2 * x - 1) * 32767);                    
        global_pigun_report.y = (short)((2 * y - 1) * 32767);
    }

}
