/*
 * Yuv_LED_filter.h
 *
 *  Created on: Oct 1, 2013
 *      Author: al
 */

#ifndef YUV_LED_FILTER_H_
#define YUV_LED_FILTER_H_



#include <opencv/cv.h>
#include <iostream>
#include <fstream>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include <cvd/vector_image_ref.h>
#include <cvd/image_interpolate.h>

/* Namespaces */
using namespace cv;
using namespace CVD;

/* Filter boundries and what have you */
#define RECT_OFFSET 1.5
#define MIN_AREA 1.00
#define MAX_TOL  100000.00
#define MAX_AREA 15000

// 3.13 (fun_RGB2LED) 	- Led Filter
void YUV_LED_filter(IplImage *imginput,IplImage *rotated );

#endif /* YUV_LED_FILTER_H_ */
