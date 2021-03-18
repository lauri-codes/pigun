#include <math.h>
#include <stdlib.h>

using namespace std;

int main(int argc, char** argv) {

    const int width = 320;
    const int height = 180;
    const float fovX = 62.2;
    const float fovY = 48.8;
    const float ledDist = 25; // In centimeters

    int x1 = 0;
    int y1 = 0;

    float ref1X = -10
    float ref1Y = -10

    float int1 = 10;
    float int2 = 20;
    float intRatio = int1/int2;                 // Ratio between LED intensities: I1/I2
    float distRatio = sqrt(1/intRatio);         // Ratio between LED distances: d1/d2
    float angleX = abs(ref1X-ref2X)/width*fovX; // Angle between LEDs

    // Figure out screen left side through calibration
    centerX = width/2;
    screenLeftAngle = abs(centerX-ref1X)/width*fovX;
    screenLeftAbs = ;

    return 0;
}
