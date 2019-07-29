#include "pigun.h"


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
void pigun_compute_4corners(Peak* ABCD, float aspectRatio, float* CMatrix) {

	float x1 = ABCD[0].col;
	float x2 = ABCD[1].col;
	float x3 = ABCD[2].col;
	float x4 = ABCD[3].col;
	float y1 = ABCD[0].row;
	float y2 = ABCD[1].row;
	float y3 = ABCD[2].row;
	float y4 = ABCD[3].row;

	float d1 = 1.0/(x4*(y1 - y2) + x1*(y2 - y4) + x2*(y4-y1));
	CMatrix[0] = (y1 - y2) * d1;
	CMatrix[1] = (x2 - x1) * d1;
	CMatrix[2] = (x1*y2 - x2*y1) * d1;

	float d2 = 1.0 / (-x3*y1 + x4*y1 + x1*y3 - x4*y3 - x1*y4 + x3*y4);
	CMatrix[3] = (y1 - y3) * d2;
	CMatrix[4] = (x3 - x1) * d2;
	CMatrix[5] = (x1*y3 - x3*y1) * d2;

	float d3 = 1.0 / (x3*y2 - x4*y2 - x2*y3 + x4*y3 + x2*y4 - x3*y4);
	CMatrix[6] = CMatrix[0] + CMatrix[3] + (y2 - y3) * d3;
	CMatrix[7] = CMatrix[1] + CMatrix[4] + (x3 - x2) * d3;
	CMatrix[8] = CMatrix[2] + CMatrix[5] + (x2*y3 - x3*y2) * d3;

}



