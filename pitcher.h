/*
 * pitcher.h
 *
 *  Created on: Sep 2, 2013
 *      Author: al
 */

#ifndef PITCHER_H_
#define PITCHER_H_

#include <iostream>
#include <unistd.h>
#include <stdio.h>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <stdlib.h>
#include <cstring>      // Needed for memset
#include <sys/socket.h> // Needed for the socket functions
#include <netdb.h>      // Needed for the socket functions
#include <algorithm>
#include <iterator>
// TooN Library, stuff for easy matrix operations and what not.
#include <TooN/wls.h>
#include <TooN/SVD.h>
#include <TooN/TooN.h>
#include <TooN/se3.h>
#include <cvd/camera.h>
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
// Open Kinect
#include "libfreenect.h"
#include "libfreenect_cv.h"
#include "libfreenect_sync.h"
#include "als_livefreenect.h"

//*- Need this for eyeBug control - *//
#include "eyeBug_movement.h"
#include "sender.h"

using namespace TooN;		// TooN
using namespace std;

/* Camera definitions */
#define X_AXIS_OFFSET 240

/* Rotation definitions */
//#define STEPS_P_ANGLE 9.333333333
//#define STEPS_P_ANGLE 8.4963
//#define STEPS_P_ANGLE 8.75
#define STEPS_P_ANGLE 8.85
#define STEPS_P_M 4545.4545

/* Pi for the deliciousness */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Camera y-axis offset */
#define Y_CAMERA_OFFSET 0.15
#define EYEBUG_RADIUS	0.115

/* Network sign in time */
#define SIGNIN_TIME 20

/* Functions */
bool fexists(const char *filename);
void ay_delay(int delay_time);
two_d_coordinates boundry_condition_adjuster(two_d_coordinates input, map_boundries map_parameters);

/* Structs */
struct global_sheet{
	int sizeofswarm;
	std::vector<int> netsignedin;
};

/* Classes */
class on_eyeBug{
	float x, y, angle;
	int ID;
	int sizeofswarm;
	bool net_signin,map_flag, got_ID, local_pose, rps;
	map_boundries global_map_parameters;
	vector<quadrant> quady;
	quadrant my_own_quadrant;

	CVD::ImageRef kinect_size;
	CVD::ImageRef rotated_size;
	int frame_count;
	std::vector<int> netsignedinIDs;
public:
	IplImage *current_rgb;
	IplImage *current_dep;
	Camera::Linear cam;

	// The "whats my..." commands allow us to view private values for the eyeBug without modifying them//
	int  whats_my_frame_count();
	int  whats_my_ID();
	two_d_coordinates whats_my_xy();
	int  whats_my_swarm_size();
	float whats_my_angle();
	quadrant whats_my_quadrant();
	map_boundries whats_my_boundries();
	CVD::ImageRef whats_my_image_size();
	bool is_rps_found();

	// The main control functions here //
	void starter();
	void network_sign_in(bool eyeBug0);
	void set_LEDs();
	void set_global_boundries(float lenny, float widdy);
	void non_zero_rps_found(seen_robotics sighted0);
	void eyeBug0_rps_found();
	void update_rps();
	void target_seen(seen_robotics target);
	void end_tbug();

	//  The complimentary functions //

	//	self_localisation(this_pointer);
	//	move_to_starting_point(this_pointer);
	//	quadrant_search_mode(this_pointer);

	// These are the commands used to move the eyeBug about the RPS
	void rotate_eyeBug(float new_angle);
	void move_eyeBug_forward_w_pose(float distance);
	void move_eyeBug_forward(float distance);
	void image_grabber();
	IplImage* rgb_image_grabber();

};


#endif /* PITCHER_H_ */
