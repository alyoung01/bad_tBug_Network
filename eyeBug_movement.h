/*
 * eyeBug_movement.h
 *
 *  Created on: Sep 16, 2013
 *      Author: al
 */

#include "sender.h"
#include "Control_eyeBug/eBugAPI.h"
#include <time.h>
#include <unistd.h>
#include <fstream>
#include <cmath>
#include "sequences.h"
#include "swarm.h"

#define laptop false

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef STEPS_P_ANGLE
#define STEPS_P_ANGLE 9.333333333
#endif
#ifndef EYEBUG_MOVEMENT_H_
#define EYEBUG_MOVEMENT_H_


void new_coo(float x, float y);
float motor_walk(float dis_z);
void motor_rotate(float angle, int quad);
void move_robot(float new_x, float new_y);
void new_coo(float x, float y);
void activate_leds(int ID);

#endif /* EYEBUG_MOVEMENT_H_ */

