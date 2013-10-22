/*
 * swarm.h
 *
 *  Created on: Sep 17, 2013
 *      Author: al
 */

#ifndef SWARM_H_
#define SWARM_H_
#include <iostream>
#include <unistd.h>
#include <stdio.h>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <stdlib.h>
#include <opencv/cv.h>
// 2.2 (Obj_swa)- Swarm
struct swarm
{
public:
	int id;
	bool wifi_confirm=false;
	bool visual_confirm=false;
	float x,y,angle;
};


struct two_d_coordinates
{
	float x,y;
};

struct quadrant
{
	float angle1, angle2;
	int id;
	int number;
};

struct map_boundries
{
	float rad, min_x, min_y, max_x, max_y;
};

struct seen_robotics
//***********************************************************************************************************************
// This is essentially a redefine of the robot struct, but given that it was being funny buggers with the headers
// this has to happen (AY)
//***********************************************************************************************************************
{
	int id;
	float x,y,angle;
	cv::RotatedRect ellipse;
};

class al_point
{
public:
	std::vector<seen_robotics> zero, sixty,one20, one80, two40, three100;
	void allocate_seen_robot(float angle, seen_robotics robot_to_add);
};


#endif /* SWARM_H_ */
