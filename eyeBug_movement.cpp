/*
 * eyeBug_movement.cpp
 *
 *  Created on: Sep 16, 2013
 *      Author: al
 */
#include "eyeBug_movement.h"

using namespace std;


// 3.22 (fun_mtr_walk) - Motor Walk
float motor_walk(float dis_z)
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


	results << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
	results << "\t\t\t\t\t Start time= " << (float)clock()/CLOCKS_PER_SEC << endl;
	results << "motor_walk_test called:" << endl;

	bool mo_est=true;
	bool flag=false;
	int steps = floor((4545.4545*dis_z));
	//	int steps = floor((4675*dis_z));
	vector<char> command_packet;			// To send to UART
	float window = (clock()/CLOCKS_PER_SEC);// Timer

	do{
		if(
				(laptop==false)
				&&
				(steps>1000)
				&&
				(clock()/CLOCKS_PER_SEC>=(float)window+5)
		)
		{

			results << "Sending Motor forward to UART" << endl;
			results << "Steps = " << steps << endl;

			command_packet= StepperMotorLeftRightStep(255, 1000, true, 2, true, 255, 1000, true, 2, true, false, 0);
			sender(command_packet);
			steps-=1000;

			window = (clock()/CLOCKS_PER_SEC);
		}
		else if (
				(laptop==false)
				&&
				(steps<1000)
				&&
				(clock()/CLOCKS_PER_SEC>=(float)window+5)
				&&
				(flag==false))
		{
			results << "Sending Final Motor forward to UART" << endl;
			results << "Steps = " << steps << endl;
			command_packet= StepperMotorLeftRightStep(255, steps, true, 2, true, 255, steps, true, 2, true, false, 0);
			sender(command_packet);
			flag=true;
			window = (clock()/CLOCKS_PER_SEC);
		}
		else if
		(
				(clock()/CLOCKS_PER_SEC>=(float)window+5)
				&&
				(steps<1000)
				&&
				(flag==true)
		)
		{mo_est=false;}
	}while(mo_est==true);

	results << "motor_walk_test called: done" << endl;
	results << "\t\t\t\t\t End time= " << (float)clock()/CLOCKS_PER_SEC << endl;
	results << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;

	//*****************************************************************************************************
	// Close the results file
	//*****************************************************************************************************
	results.flush();
	results.close();

	return (float) dis_z;
}

// 3.23 (fun_mtr_rotate) - Motor Rotate
void motor_rotate(float angle, int quad)
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

	results << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
	results << "\t\t\t\t\t Start time= " << (float)clock()/CLOCKS_PER_SEC << endl;
	results << "motor_rotate called:" << endl;
	results << "\t rotating " << angle << " degrees in quadrant " << quad << endl;

	int steps=0;
	vector<char> command_packet;			// To send to UART

	switch(quad)
	{
	case 1:
		steps= floor(angle*STEPS_P_ANGLE);
		if(steps==0)
		{
			results << " Don't need to update angle" << endl;
		}
		else
		{
			command_packet=StepperMotorLeftRightStep(255, steps, false, 3, true, 255, steps, true, 3, true, false, 0);
			sender(command_packet);
		}
		break;
	case 2:
		steps= floor(angle*STEPS_P_ANGLE);
		if(angle==180)
		{steps=1620;}
		command_packet=StepperMotorLeftRightStep(255, steps, false, 3, true, 255, steps, true, 3, true, false, 0);
		sender(command_packet);
		break;
	case 3:
		steps= floor(angle*STEPS_P_ANGLE);
		if(angle==90)
		{steps=810;}
		if(angle==180)
		{steps=1620;}
		command_packet=StepperMotorLeftRightStep(255, steps, true, 3, true, 255, steps, false, 3, true, false, 0);
		sender(command_packet);
		break;
	case 4:
		steps= floor(angle*STEPS_P_ANGLE);
		if(angle==90)
		{steps=810;}
		if(angle==180)
		{steps=1620;}
		if(steps==0)
		{
			results << " Don't need to update angle" << endl;
		}
		else
		{
			command_packet=StepperMotorLeftRightStep(255, steps, true, 3, true, 255, steps, false, 3, true, false, 0);
			sender(command_packet);
		}
		break;
	}

	sleep(6);

	results << "Steps = " << steps << endl;
	results << "motor_rotate called: done" << endl;
	results << "\t\t\t\t\t End time= " << (float)clock()/CLOCKS_PER_SEC << endl;
	results << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;

	//*****************************************************************************************************
	// Close the results file
	//*****************************************************************************************************
	results.flush();
	results.close();

}

// 3.24 (fun_move_robot) - Move Robot
void move_robot(two_d_coordinates new_coo, two_d_coordinates old_coo)
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

	results << "Moving from (" << old_coo.x << "," << old_coo.y << ") to (" << new_coo.x << "," << new_coo.y << ")" << endl;

	float delm;
	int quad=0;

	float dis=pow((pow((new_coo.y-old_coo.y),2)+pow((new_coo.x-old_coo.x),2)),0.5); // dis= sqrt((diff in y)^2+(diff in x)^2)
	// Total distance being moved
	results << "Moving " << dis << " m " << endl;

	// Determine quadrant
	if(
			(new_coo.y-old_coo.y>=0)
			&&
			(new_coo.x-old_coo.x>=0)
	)
	{
		quad=1;
	}
	else if(
			(new_coo.y-old_coo.y>=0)
			&&
			(new_coo.x-old_coo.x<=0)
	)
	{
		quad=2;
	}
	else if(
			(new_coo.y-old_coo.y<=0)
			&&
			(new_coo.x-old_coo.x<=0)
	)
	{
		quad=3;
	}
	else if(
			(new_coo.y-old_coo.y<=0)
			&&
			(new_coo.x-old_coo.x>=0)
	)
	{
		quad=4;
	}

	float angle=acos((new_coo.x-old_coo.x)/dis)*180/M_PI;
	results << "our quadrant is " << quad << endl;
	results << "Angle = " << angle << endl;
	motor_rotate(angle, quad);

	delm= motor_walk(dis);

	switch(quad)
	{
	case 1: quad=4;break;
	case 2: quad=3;break;
	case 3: quad=2;break;
	case 4: quad=1;break;
	}
	motor_rotate(angle,quad);	// Return to 0;

	//*****************************************************************************************************
	// Close the results file
	//*****************************************************************************************************
	results.flush();
	results.close();
}

// 3.25 (fun_rA_to_new_co) - Move Robot at right angles to new coordinates
void rA_to_new_coo(two_d_coordinates* new_coo, swarm* the_swarm, int eyeBugID)
{
	two_d_coordinates temp_old;
	two_d_coordinates temp_new;

	if(new_coo->x!=the_swarm[eyeBugID].x)
	{
		temp_old.x=the_swarm[eyeBugID].x;
		temp_old.y=the_swarm[eyeBugID].y;
		temp_new.x=new_coo->x;					//Only the x coordinate is the new one
		temp_new.y=the_swarm[eyeBugID].y;

		move_robot(temp_new, temp_old);
	}
	if(new_coo->y!=the_swarm[eyeBugID].y)
	{
		temp_old.x=new_coo->x;
		temp_old.y=the_swarm[eyeBugID].y;		// Now only the Y coordinate is the old one
		temp_new.x=new_coo->x;
		temp_new.y=new_coo->y;

		move_robot(temp_new, temp_old);
	}
}

// 3.21 (fun_activeLED) - Activate LED
void activate_leds(int ID)
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

	results << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
	results << "activate_leds() \t\t\t\t\t Start time= " << (float)clock()/CLOCKS_PER_SEC << endl;
	results << "ID: " << ID << endl;

	std::vector<uint16_t> mask(3);
	vector<char> command_packet;
	uint64_t n=0;

	for(int i=0;i<16;i++) //if(i%2)
	{
		n|=uint64_t(1)<<(i*3+seqs[ID][i]);
	} //set alternating red, green, blue LEDs

	mask.operator[](0)= n>>32;
	mask.operator[](1)= n<<32>>48;
	mask.operator[](2)= n<<48>>48;

	results << "\t Mask 1 = " << mask.operator[](2) << endl;
	results << "\t Mask 2 = " << mask.operator[](1) << endl;
	results << "\t Mask 3 = " << mask.operator[](0) << endl;

	command_packet = TLC5947_SetMultiple(mask.operator[](0), mask.operator[](1), mask.operator[](2), 500, false, 0x00); // Create Command packet
	sender(command_packet);

	results << "active_leds called: done" << endl;
	results << "\t\t\t\t\t End time= " << (float)clock()/CLOCKS_PER_SEC << endl;
	results << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;

	//*****************************************************************************************************
	// Close the results file
	//*****************************************************************************************************
	results.flush();
	results.close();

}
