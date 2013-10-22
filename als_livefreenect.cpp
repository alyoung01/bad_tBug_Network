/*
 * als_livefreenect.cpp
 *
 *  Created on: Aug 21, 2013
 *      Author: Al
 */

#include "libfreenect.h"
#include "libfreenect_sync.h"
#include "libfreenect_cv.h"

//  3.1 (fun_kdep) - freenect_sync_get_depth_cv
IplImage *freenect_sync_get_depth_cv(int index)
{
	static IplImage *image = 0;
	static char *data = 0;
	if (!image) image = cvCreateImageHeader(cvSize(640,480), 16, 1);
	unsigned int timestamp;
	if (freenect_sync_get_depth((void**)&data, &timestamp, index, FREENECT_DEPTH_11BIT))
		return NULL;
	cvSetData(image, data, 640*2);
	return image;
}

//  3.2 (fun_krgb) - freenect_sync_get_rgb_cv
IplImage *freenect_sync_get_rgb_cv(int index)
{
	static IplImage *image = 0;
	static char *data = 0;
	if (!image) image = cvCreateImageHeader(cvSize(640,480), IPL_DEPTH_8U, 3);
	unsigned int timestamp;
	if (freenect_sync_get_video((void**)&data, &timestamp, index, FREENECT_VIDEO_RGB))
		return NULL;
	cvSetData(image, data, 640*3);
	return image;
}

//  3.3 (fun_glcolview) - GlViewColor
IplImage *GlViewColor(IplImage *depth)
{
	static IplImage *image = 0;
	if (!image) image = cvCreateImage(cvSize(640,480), IPL_DEPTH_8U, 3);
	unsigned char *depth_mid = (unsigned char*)(image->imageData);
	int i;
	for (i = 0; i < 640*480; i++) {
		int lb = ((short *)depth->imageData)[i] % 256;
		int ub = ((short *)depth->imageData)[i] / 256;
		switch (ub) {
		case 0:
			depth_mid[3*i+2] = 255;
			depth_mid[3*i+1] = 255-lb;
			depth_mid[3*i+0] = 255-lb;
			break;
		case 1:
			depth_mid[3*i+2] = 255;
			depth_mid[3*i+1] = lb;
			depth_mid[3*i+0] = 0;
			break;
		case 2:
			depth_mid[3*i+2] = 255-lb;
			depth_mid[3*i+1] = 255;
			depth_mid[3*i+0] = 0;
			break;
		case 3:
			depth_mid[3*i+2] = 0;
			depth_mid[3*i+1] = 255;
			depth_mid[3*i+0] = lb;
			break;
		case 4:
			depth_mid[3*i+2] = 0;
			depth_mid[3*i+1] = 255-lb;
			depth_mid[3*i+0] = 255;
			break;
		case 5:
			depth_mid[3*i+2] = 0;
			depth_mid[3*i+1] = 0;
			depth_mid[3*i+0] = 255-lb;
			break;
		default:
			depth_mid[3*i+2] = 0;
			depth_mid[3*i+1] = 0;
			depth_mid[3*i+0] = 0;
			break;
		}
	}
	return image;
}


// 3.4 (fun_kled_r) - freenect_sync_set_led_red_cv
void freenect_sync_set_led_red_cv(int index)
{
	int led_red=0;
	led_red = freenect_sync_set_led(LED_RED, index);
	//return 0;
}

// 3.5 (fun_kled_g) - freenect_sync_set_led_green_
void freenect_sync_set_led_green_cv(int index)
{
	int led_green=0;
	led_green = freenect_sync_set_led(LED_GREEN, index);
	//return 0;
}

// 3.6 (fun_kled_bg)- freenect_sync_set_led_blink_green_cv
void freenect_sync_set_led_blink_green_cv(int index)
{
	int led_green=0;
	led_green = freenect_sync_set_led(LED_BLINK_GREEN, index);
	//return 0;
}

// 3.7 (fun_kled_y) - freenect_sync_set_led_yellow_cv
void freenect_sync_set_led_yellow_cv(int index)
{
	int led_green=0;
	led_green = freenect_sync_set_led(LED_YELLOW, index);
	//return 0;
}

// 3.8 (fun_kled_off)- freenect_sync_set_led_off_cv
void freenect_sync_set_led_off_cv(int index)
{
	int led_green=0;
	led_green = freenect_sync_set_led(LED_OFF, index);
	//return 0;
}

// 3.9 (fun_kstop) - freenect_sync_stop_cv
void freenect_sync_stop_cv (void)
{
	freenect_sync_stop();
}
