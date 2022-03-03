#include "pigun.h"
#include "pigun_bt.h"


/*
	4 corners are:
	A------C
	|      |
	|      |
	B------D

	the resulting matrix C is stored in CMatrix, linearised
	the x,y, screen point then corresponds to
	P = cmat . {x, y, 1} / (cmat . {x, y, 1})[z]
	in recrangle coordinates
	P = (0,0) => A
	P = (1,1) => D
	it puzzles me that there is NO dependence on the rectangle aspect ratio?!?!!!?!!1111!

*/


void pigun_calculate_aim() {

	float aim_x, aim_y;
	float x1 = pigun_peaks[0].col;
	float x2 = pigun_peaks[1].col;
	float x3 = pigun_peaks[2].col;
	float x4 = pigun_peaks[3].col;
	float y1 = pigun_peaks[0].row;
	float y2 = pigun_peaks[1].row;
	float y3 = pigun_peaks[2].row;
	float y4 = pigun_peaks[3].row;

	// build the transformation matrix using the 4 points and apply it to the center of camera image
	// check out the mathematica notebook for some more insight
	float dy = (y1 - y2);
	float dxy = x1 * y2 - x2 * y1;
	float d1 = 1.0 / (x4 * dy + dxy - x1 * y4 + x2 * y4));
	aim_x = (dy * (PIGUN_RES_X / 2) + (x2 - x1) * (PIGUN_RES_Y / 2) + dxy * cmat[2]) * d1;

	dy = (y1 - y3);
	dxy = x1 * y3 - x3 * y1;
	d1 = 1.0 / (x4 * dy + dxy - x1 * y4 + x3 * y4);
	aim_y = (dy * (PIGUN_RES_X / 2) + (x3 - x1) * (PIGUN_RES_Y / 2) + dxy) * d1;

	aim_x += pigun_aimOffset_x;
	aim_y += pigun_aimOffset_y;

	// clamp between 0 and 1
	aim_x = (aim_x < 0) ? 0 : aim_x;
	aim_x = (aim_x > 1) ? 1 : aim_x;
	aim_y = (aim_y < 0) ? 0 : aim_y;
	aim_y = (aim_y > 1) ? 1 : aim_y;

	global_pigun_report.x = (short)((2 * x - 1) * 32767);
	global_pigun_report.y = (short)((2 * y - 1) * 32767);
}



/*
// ORIGINAL CODE
void pigun_compute_4corners(Peak* ABCD, float aspectRatio, float* CMatrix) {

	float x1 = ABCD[0].col;
	float x2 = ABCD[1].col;
	float x3 = ABCD[2].col;
	float x4 = ABCD[3].col;
	float y1 = ABCD[0].row;
	float y2 = ABCD[1].row;
	float y3 = ABCD[2].row;
	float y4 = ABCD[3].row;

	// build the transformation matrix using the 4 points
	// check out the mathematica notebook for some more insight
	float d1 = 1.0 / (x4 * (y1 - y2) + x1 * (y2 - y4) + x2 * (y4 - y1));
	CMatrix[0] = (y1 - y2) * d1;
	CMatrix[1] = (x2 - x1) * d1;
	CMatrix[2] = (x1 * y2 - x2 * y1) * d1;

	float d2 = 1.0 / (-x3 * y1 + x4 * y1 + x1 * y3 - x4 * y3 - x1 * y4 + x3 * y4);
	CMatrix[3] = (y1 - y3) * d2;
	CMatrix[4] = (x3 - x1) * d2;
	CMatrix[5] = (x1 * y3 - x3 * y1) * d2;

	float d3 = 1.0 / (x3 * y2 - x4 * y2 - x2 * y3 + x4 * y3 + x2 * y4 - x3 * y4);
	CMatrix[6] = CMatrix[0] + CMatrix[3] + (y2 - y3) * d3;
	CMatrix[7] = CMatrix[1] + CMatrix[4] + (x3 - x2) * d3;
	CMatrix[8] = CMatrix[2] + CMatrix[5] + (x2 * y3 - x3 * y2) * d3;

	// all that is left to do is cmat.{x,y,1} = aiming point
	// x,y is the aiming point in camera space, which always is in the middle (XRES/2, YRES/2)



}
*/


