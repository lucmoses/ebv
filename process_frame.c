/* Copying and distribution of this file, with or without modification,
 * are permitted in any medium without royalty. This file is offered as-is,
 * without any warranty.
 */

/*! @file process_frame.c
 * @brief Contains the actual algorithm and calculations.
 */

/* Definitions specific to this application. Also includes the Oscar main header file. */
#include "template.h"
#include <string.h>
#include <stdlib.h>

#define IMG_SIZE NUM_COLORS*(OSC_CAM_MAX_IMAGE_WIDTH/2)*(OSC_CAM_MAX_IMAGE_HEIGHT/2)

const int nc = OSC_CAM_MAX_IMAGE_WIDTH / 2;
const int nr = OSC_CAM_MAX_IMAGE_HEIGHT / 2;
const int SizeBox = 5;
const int Oshift = 6;
void CalcDeriv();
void AvgDeriv(int);
void LocalMaximum();

int TextColor;
int avgDxy[3][IMG_SIZE];
int Mc[IMG_SIZE];
int McLocalMax[IMG_SIZE];
int absmax;

void ResetProcess() {
	//called when "reset" button is pressed
	if (TextColor == CYAN)
		TextColor = MAGENTA;
	else
		TextColor = CYAN;
}

void ProcessFrame() {
	uint32 t1, t2;
	//initialize counters
	if (data.ipc.state.nStepCounter == 1) {
		//use for initialization; only done in first step
		memset(data.u8TempImage[THRESHOLD], 0, IMG_SIZE);
		TextColor = CYAN;
	} else {
		//example for time measurement

		//example for copying sensor image to background image
		memcpy(data.u8TempImage[BACKGROUND], data.u8TempImage[SENSORIMG],
		IMG_SIZE);
		//example for time measurement

		//Edge dedector
		CalcDeriv();
		for (int i = 0; i < 3; i++) {
			AvgDeriv(i);
		}
		absmax = 0;
		//for (int i = 0; i < IMG_SIZE; i++) {
		for (int r = 7 * nc; r < nr * nc - 7 * nc; r += nc) {/* we skip the first and last line */
			for (int c = 7; c < nc - 7; c++) {
				int i = r + c;
				Mc[i] = ((avgDxy[0][i] >> Oshift) * (avgDxy[1][i] >> Oshift)
						- (avgDxy[2][i] >> Oshift) * (avgDxy[2][i] >> Oshift))
						- ((5
								* ((avgDxy[0][i] >> Oshift)
										+ (avgDxy[1][i] >> Oshift))
								* ((avgDxy[0][i] >> Oshift)
										+ (avgDxy[1][i] >> Oshift))) >> 7);
				absmax = MAX(Mc[i], absmax);
			}
		}

		absmax = (absmax/100*data.ipc.state.nThreshold);


		t1 = OscSupCycGet();
		LocalMaximum();
		/*for (int i = 0; i < IMG_SIZE; i++) {
		 data.u8TempImage[THRESHOLD][i] = (uint8) MIN(255,
		 MAX(0, (McLocalMax[i])));
		 }*/
		t2 = OscSupCycGet();

		//example for log output to console
		OscLog(INFO, "required = %d us\n", OscSupCycToMicroSecs(t2 - t1));

		//example for drawing output
		//draw Boxes
		//DrawBoundingBox(x1-SizeBox, y1+SizeBox, x1+SizeBox, y1-SizeBox, false, GREEN);
	}
}

void CalcDeriv() {
	int c, r;
	for (r = nc; r < nr * nc - nc; r += nc) {/* we skip the first and last line */
		for (c = 1; c < nc - 1; c++) {
			/* do pointer arithmetics with respect to center pixel location */
			unsigned char* p = &data.u8TempImage[SENSORIMG][r + c];
			/* implement Sobel filter */
			int dx = -(int) *(p - nc - 1) + (int) *(p - nc + 1)
					- 2 * (int) *(p - 1) + 2 * (int) *(p + 1)
					- (int) *(p + nc - 1) + (int) *(p + nc + 1);
			int dy = -(int) *(p - nc - 1) - 2 * (int) *(p - nc)
					- (int) *(p - nc + 1) + (int) *(p + nc - 1)
					+ 2 * (int) *(p + nc) + (int) *(p + nc + 1);

			//not yet averaged!!
			avgDxy[0][r + c] = dx * dx;
			avgDxy[1][r + c] = dy * dy;
			avgDxy[2][r + c] = dx * dy;

		}
	}
}

void AvgDeriv(int Index) {
	//do average in x-direction
	int c, r;
	int helpBuf[IMG_SIZE];
	int Border = 0;
	for (r = nc; r < nr * nc - nc; r += nc) {/* we skip first and last lines (empty) */
		for (c = Border + 1; c < nc - (Border + 1); c++) {/* +1 because we have one empty border column */
			/* do pointer arithmetics with respect to center pixel location */
			int* p = &avgDxy[Index][r + c];
			int sx = (*(p - 6) + *(p + 6)) + ((*(p - 5) + *(p + 5)) << 2)
					+ ((*(p - 4) + *(p + 4)) << 3)
					+ ((*(p - 3) + *(p + 3)) << 5)
					+ ((*(p - 2) + *(p + 2)) << 6)
					+ ((*(p - 1) + *(p + 1)) << 6) + (*p << 7);
			//now averaged
			helpBuf[r + c] = (sx >> 8);
		}
	}
	//do average in y-direction
	for (r = nc; r < nr * nc - nc; r += nc) {/* we skip first and last lines (empty) */
		for (c = Border + 1; c < nc - (Border + 1); c++) {/* +1 because we have one empty border column */
			/* do pointer arithmetics with respect to center pixel location */
			int* p = &helpBuf[r + c];
			int sy = (*(p - 6 * nc) + *(p + 6 * nc))
					+ ((*(p - 5 * nc) + *(p + 5 * nc)) << 2)
					+ ((*(p - 4 * nc) + *(p + 4 * nc)) << 3)
					+ ((*(p - 3 * nc) + *(p + 3 * nc)) << 5)
					+ ((*(p - 2 * nc) + *(p + 2 * nc)) << 6)
					+ ((*(p - nc) + *(p + nc)) << 6) + (*p << 7);
			//now averaged
			avgDxy[Index][r + c] = (sy >> 8);
			//	data.u8TempImage[THRESHOLD][r + c] = (uint8) MIN(255,
			//		MAX(0, (avgDxy[2][c+r])>>8));
		}
	}
}

void LocalMaximum() {
	//memset(McLocalMax, 0, sizeof(McLocalMax));
	int c, r;

	for (r = 7 * nc; r < nr * nc - 7 * nc; r += nc) {/* we skip the first and last line */
		for (c = 7; c < nc - 7; c++) {
			/* do pointer arithmetics with respect to center pixel location */
			int* p = &Mc[c + r];
			/* implement Sobel filter */
			int localMax = 0;
			for (int i = -6; i < 7; i++) {
				for (int j = -6; j < 7; j++) {
					if (localMax <= *(p + nc * i + j)) {
						//	McLocalMax[c+r+iHelp*nc+jHelp] = 0;
						localMax = *(p + i * nc + j);
					}
					//	else if(localMax > *(p+nc*i+j)){
					//	McLocalMax[c+r+i*nc+j] = 0;

					//}
				}

			}
			//McLocalMax[c+r+iHelp*nc+jHelp] = *(p+nc*iHelp+jHelp);
			if (localMax == *p && *p > absmax) {
				DrawBoundingBox(c - SizeBox, r / nc + SizeBox, c + SizeBox,
						r / nc - SizeBox, false, GREEN);

				//OscLog(INFO, "c = %d ; r = %d\n",c,r);
			}
		}
	}
}

void LocalMaximumV2() {
	//do average in x-direction
	int c, r;
	int helpBuf[IMG_SIZE];
	memset(helpBuf, 0, sizeof(helpBuf));
	int Border = 0;
	for (r = nc; r < nr * nc - nc; r += nc) {/* we skip first and last lines (empty) */
		for (c = Border + 1; c < nc - (Border + 1); c++) {/* +1 because we have one empty border column */
			/* do pointer arithmetics with respect to center pixel location */
			int* p = &Mc[c + r];
			int localMax = *p;
			for (int j = -5; j < 7; j++) {
								if (localMax <= *(p + nc + j)) {
									//	McLocalMax[c+r+iHelp*nc+jHelp] = 0;
									localMax = *(p +  nc + 6 * j);
			//now averaged
			helpBuf[r + c] = ( 8);
		}
	}
	//do average in y-direction
	for (r = nc; r < nr * nc - nc; r += nc) {/* we skip first and last lines (empty) */
		for (c = Border + 1; c < nc - (Border + 1); c++) {/* +1 because we have one empty border column */
			/* do pointer arithmetics with respect to center pixel location */
			int* p = &helpBuf[r + c];


								}
								//	else if(localMax > *(p+nc*i+j)){
								//	McLocalMax[c+r+i*nc+j] = 0;

								//}
							}
			//now averaged

			//	data.u8TempImage[THRESHOLD][r + c] = (uint8) MIN(255,
			//		MAX(0, (avgDxy[2][c+r])>>8));
		}
	}
}
