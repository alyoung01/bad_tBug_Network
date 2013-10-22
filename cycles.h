/*
 * cycles.h
 *
 *  Created on: Sep 20, 2013
 *      Author: al
 */

#ifndef CYCLES_H_
#define CYCLES_H_

#include <opencv/cv.h>
#include <cvd/image_ref.h>
#include <cstdlib>
#include <cmath>
#include <algorithm>
#include <queue>
#include <set>
#include <tuple>
#include <thread>
#include <mutex>
#include "swarm.h"
#include "sequences.h"
#include "robots.h"

/* Camera Parameters */
#define WIDTH  480				// Image Width
#define HEIGHT 640				// Image Height
#define FOCAL_WIDTH (1.3206*WIDTH)

/* Global variables used in cycles */

static uint8_t H[WIDTH*HEIGHT]; //raw hue image
static uint8_t C[WIDTH*HEIGHT]; //raw chroma image

using namespace std;
using namespace cv;
/* Data Strcts */
//using cv::Point2f;
//struct led: public Point2f
//{
//	int colour;
//	led(Point2f pos=Point2f(),int colour=-1): Point2f(pos),colour(colour) {}
//	bool operator<(const led &other) const
//	{
//		return std::tie(x,y,colour)<std::tie(other.x,other.y,other.colour);
//	}
//}; //represents a coloured circle in the image

/* Functions */
void rgb2HC(void *data);
float ellipse_to_circle(float phi,float rz);
float circle_to_ellipse(float theta,float rz);

using std::vector;
vector<vector<led> > knn_graph_partition(const vector<led> &points);

float min_rounding(std::vector<std::pair<float,int> > &angles);
std::vector<robot> cycles(IplImage *picture, CVD::ImageRef size);

#endif /* CYCLES_H_ */
