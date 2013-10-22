/*
 * libfreenect_cv.h
 *
 *  Created on: Dec 20, 2012
 *      Author: max
 */

#ifndef FREENECT_CV_H
#define FREENECT_CV_H

#ifdef __cplusplus
extern "C++" {
#endif

#include <opencv/cv.h>
//****************************************************************************************************************************************
// Section 1 (lib) - The_Libraries
//****************************************************************************************************************************************
// 1.1 (lib_in) - The_Includes

// Open Kinect
#include "libfreenect.h"
#include "libfreenect_sync.h"
// OpenCV
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
// C time library
#include <ctime>
// LibCVD Stuff
#include <cvd/image_io.h>
#include <cvd/glwindow.h>               //Very cheap and cheerful X window with OpenGL capabilities
#include <cvd/gl_helpers.h>                 //OpenGL wrappers for various CVD types
#include <cvd/vector_image_ref.h>
#include <cvd/image_interpolate.h>
#include <cvd/utility.h>
// C++ Header that defines the standard input/output stream objects:
#include <iostream>
// C++ Header providing file stream classes
#include <fstream>
// C Standard Input and Output Library
#include <stdio.h>
// This stuff is for writing to UART on BB-xM
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <time.h>
#include <iostream>
#include <stdbool.h>
// TooN Library, stuff for easy matrix operations and what not.
#include <TooN/wls.h>
#include <TooN/SVD.h>
#include <TooN/TooN.h>
#include <TooN/se3.h>
// Max's local code for motion estimation
#include "MotionEst/uvq.h"
#include "MotionEst/uvq_icp.h"
// Al's local code for running the eyeBug
#include "Control_eyeBug/serial_config.h"
//#include "Control_eyeBug/serial_config.c"
#include "Control_eyeBug/eBugAPI.h"
// C Standard that defines several general purpose functions, including dynamic memory management, random number generation, communication
// with the environment, integer arithmetics, searching, sorting, and converting.
#include <cstdlib>
// C Header <cmath> declares a set of functions to compute common mathematical operations and transformations
#include <cmath>
// C header <algorithm> defines a collection of functions especially designed to be used on ranges of elements.
#include <algorithm>
// queues are a type of container adaptor, specifically designed to operate in a FIFO context (first-in first-out),
// where elements are inserted into one end of the container and extracted from the other.
#include <queue>
// Sets are containers that store unique elements following a specific order.
#include <set>
// A tuple is an object capable to hold a collection of elements. Each element can be of a different type.
#include <tuple>
// Vector library... duh
#include <vector>
// Class to represent individual threads of execution.
#include <thread>
// Mutex class - A mutex is a lockable object that is designed to signal when critical sections of code need
// exclusive access, preventing other threads with the same protection from executing concurrently and access the same memory locations.
#include <mutex>
#include "als_livefreenect.h"
#include "maxs_func.h"
#include "Yuv_LED_filter.h"
#include "LED_FILTER_2.h"
#include "pitcher.h"
#include "eyeBug_movement.h"
#include "sender.h"
#include "sequences.h"
#include "swarm.h"


// 1.2 (lib_ns) - The_Name_Spaces
using namespace cv;			// OpenCV
using namespace CVD;		// LibCVD
using namespace TooN;		// TooN
using namespace std;		// Standard librizzels

// 1.3 (lib_def) - The_Defines
#define WIDTH  480				// Image Width
#define HEIGHT 640				// Image Height
#define FOCAL_WIDTH (1.36*WIDTH)	// Focal Length
#define MOTION_EST FALSE
#define STEPS_P_ANGLE 9.333333333
#define TARGET_ID 11
#define TARGET_RADIUS 0.5


#define EYEBUG0 FALSE
//#define EYEBUG0 TRUE
/* Map definitions */
#define MAP_LENGTH 1.5
#define MAP_WIDTH  1

// 1.4 (lib_glo) - Global_values
static uint8_t H[WIDTH*HEIGHT]; //raw hue image
static uint8_t C[WIDTH*HEIGHT]; //raw chroma image

// 1.5 (lib_ell) - Ellipse_tracking
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


IplImage *freenect_sync_get_depth_cv(int index);
IplImage *freenect_sync_get_rgb_cv(int index);
void freenect_sync_set_led_red_cv(int index);
void freenect_sync_set_led_green_cv(int index);
void freenect_sync_set_led_blink_green_cv(int index);
void freenect_sync_set_led_yellow_cv(int index);
void freenect_sync_set_led_off_cv(int index);
void freenect_sync_stop_cv (void);
float pose_est(float current_cover, Camera::Linear cam, int loop_intterations);



#ifdef __cplusplus
}
#endif

#endif
