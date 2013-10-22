/*
 * LED_Filter.h
 *
 *  Created on: Aug 22, 2013
 *      Author: al
 */




#ifndef LED_FILTER_H_
#define LED_FILTER_H_

#include <opencv/cv.h>
#include <iostream>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include <cvd/vector_image_ref.h>
#include <cvd/image_interpolate.h>

/* Namespaces */
using namespace cv;
using namespace CVD;

/* Filter boundries and what have you */
#define RECT_OFFSET 2.5
#define MIN_AREA 1.00
#define MAX_TOL  100000.00
#define MAX_AREA 15000

// 3.13 (fun_RGB2LED) 	- Led Filter
IplImage* LED_filter(IplImage *imginput, ImageRef size, int filter);
#endif /* LED_FILTER_H_ */
