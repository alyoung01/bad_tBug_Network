/*
 * swarm.cpp
 *
 *  Created on: Sep 23, 2013
 *      Author: al
 */

#include "swarm.h"
using namespace std;

void al_point::allocate_seen_robot(float angle, seen_robotics robot_to_add)
{
	//*****************************************************************************************************
	// Open the results file
	//*****************************************************************************************************
	char results_dir[255];
	const char* home_dir = getenv("HOME");	// get home directory
	ofstream results;
	sprintf(results_dir,"%s/tBug_network/results/results.txt",home_dir); // update the rest of the gibberish
	results.open(results_dir, ios::app);
	//*****************************************************************************************************

	results << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
	results << "al_point::allocate_seen_robot(float angle, seen_robotics robot_to_add) \t\t\t\t\t Start time= " << (float)clock()/CLOCKS_PER_SEC << endl;

	angle -= ((int)(angle/360))*360;


	if((angle>330)&&(angle<30))
	{
		zero.push_back(robot_to_add);
	}
	else if((angle>30)&&(angle<90))
	{
		sixty.push_back(robot_to_add);
	}
	else if((angle>90)&&(angle<150))
	{
		one20.push_back(robot_to_add);
	}
	else if((angle>150)&&(angle<210))
	{
		one80.push_back(robot_to_add);
	}
	else if((angle>210)&&(angle<270))
	{
		two40.push_back(robot_to_add);
	}
	else if((angle>270)&&(angle<330))
	{
		three100.push_back(robot_to_add);
	}

	results << endl;
	results << "al_point::allocate_seen_robot(float angle, seen_robotics robot_to_add)= done \t\t\t\t\t End time= " << (float)clock()/CLOCKS_PER_SEC << endl;
	results << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
	results.flush();
	results.close();

}


