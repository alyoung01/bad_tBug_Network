/*
 * pitcher.cpp
 *
 *  Created on: Sep 2, 2013
 *      Author: al
 */

# include "pitcher.h"

using namespace std;

bool fexists(const char *filename)
{
	ifstream ifile(filename);
	return ifile;
}

two_d_coordinates boundry_condition_adjuster(two_d_coordinates input, map_boundries map_parameters)
{
	if(input.x>map_parameters.max_x)
	{input.x = map_parameters.max_x;}
	if(input.x<map_parameters.min_x)
	{input.x = map_parameters.min_x;}
	if(input.y>map_parameters.max_y)
	{input.y = map_parameters.max_y;}
	if(input.y<map_parameters.min_y)
	{input.y = map_parameters.min_y;}
	return input;
}

void ay_delay(int delay_time)
{
	//*****************************************************************************************************
	// This program was written to try and act as a workaround for the defective global clock... It did
	// not work (AY)
	//*****************************************************************************************************
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
	results << "ay_delay(int delay_time) called\t\t\t\t\t Start time= " << (float)clock()/CLOCKS_PER_SEC << endl;
	results << "Delay time: " << delay_time << endl;

	// Sample the clock at the start of the delay
	float clocker = (float)clock()/CLOCKS_PER_SEC;

	while ((float)clock()/CLOCKS_PER_SEC< (clocker+delay_time))
	{
		//Wait loop
	}

	results << endl;
	results << "ay_delay(int delay_time): done \t\t\t\t\t End time= " << (float)clock()/CLOCKS_PER_SEC << endl;
	results << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
	results.flush();
	results.close();
}

int	 on_eyeBug::whats_my_ID()
{
	return ID;
}

float on_eyeBug::whats_my_angle()
{
	return angle;
}

two_d_coordinates on_eyeBug::whats_my_xy()
{
	two_d_coordinates current={x,y};
	return current;
}

int  on_eyeBug::whats_my_frame_count()
{
	return frame_count;
}

int	 on_eyeBug::whats_my_swarm_size()
{
	return sizeofswarm;
}

quadrant on_eyeBug::whats_my_quadrant()
{
	return my_own_quadrant;
}

map_boundries on_eyeBug::whats_my_boundries()
{
	return global_map_parameters;
}

bool on_eyeBug::is_rps_found()
{
	return rps;
}

CVD::ImageRef on_eyeBug::whats_my_image_size()
{
	return kinect_size;
}

void on_eyeBug::starter()
//*****************************************************************************************************
// on_eyeBug::starter()
// This is the setup junk, it 0s all our integers and sets all the flags needed to run the program
// correctly (AY)
//*****************************************************************************************************
{
	//*****************************************************************************************************
	// Open the results file
	//*****************************************************************************************************
	char results_dir[255];
	const char* home_dir = getenv("HOME");	// get home directory
	ofstream results;
	sprintf(results_dir,"%s/tBug_network/results/results.txt",home_dir); // update the rest of the gibberish
	results.open(results_dir);
	//*****************************************************************************************************
	results << "Start code..." << endl;
	results << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
	results << "The Tracker-Bug Network" << endl;
	results << "By: Alastair D Young" << endl;
	results << "ID: ADYOU1 19874650" << endl;
	results << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
	results << "\t\t\t\t\t Start time= " << (float)clock()/CLOCKS_PER_SEC << endl;
	results << "Starting Initalisation:" << endl;

	//%%%%%%%%%%%%%%%%% initial setting %%%%%%%%%%%%%%%%%%%%

	net_signin=false;
	got_ID=false;
	local_pose=false;
	map_flag=false;
	rps=false;

	cam.get_parameters()=makeVector(600,600,320,240);

	frame_count = 0;
	angle = 0;
	kinect_size.y=WIDTH;
	kinect_size.x=HEIGHT;
	rotated_size.y=HEIGHT;
	rotated_size.x=WIDTH;

	results.flush();
	results.close();
}

void on_eyeBug::set_global_boundries(float lenny, float widdy)
//***********************************************************************************************************************************
// set_global_boundries(float radius, float widdy)
// This function should set up our global values, so how far we will search from eyeBug 0. our relative point is treated as point 0, so
// everything is calculated plus or minus compared to that. Given that this is called AFTER network signin and self localisation, we should know our swarm number
// and be able to determine our quadrants from this too.
//***********************************************************************************************************************************
{
	//*****************************************************************************************************
	// Open the results file
	//*****************************************************************************************************
	char results_dir[255];
	char global_rps_dir[255];
	const char* home_dir = getenv("HOME");	// get home directory
	ofstream results;
	sprintf(results_dir,"%s/tBug_network/results/results.txt",home_dir); // update the rest of the gibberish
	results.open(results_dir, ios::app);
	//*****************************************************************************************************
	results << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
	results << "set_global_boundries(float lenny, float widdy) called\t\t\t\t\t Start time= " << (float)clock()/CLOCKS_PER_SEC << endl;

	float radius = pow((pow((widdy/2),2)+pow((lenny/2),2)),0.5);

	global_map_parameters.rad=radius;
	global_map_parameters.max_x=(widdy/2);
	global_map_parameters.max_y=(lenny/2);
	global_map_parameters.min_x=-(widdy/2);
	global_map_parameters.min_y=-(lenny/2);

	// ONLY HERE FOR DEBUGGGING //
	//	sizeofswarm = 3;
	//	ID = 1;
	//	x=-1.2;
	//	y=0.54;

	int number_of_quadrants = sizeofswarm;			// Done like this so it's easier to see whats going on.

	// Get everyones starting points (The RPS) //
	vector<two_d_coordinates> RPS;

	// Gets the RPS so we can assign quadrants //
	results << "Obtaining starting points for all tBugs in swarm from:" << endl;
	sprintf(global_rps_dir,"%s/tBug_network/global/rps.txt",home_dir); // update the rest of the gibberish
	results << global_rps_dir << endl;

	if (fexists(global_rps_dir))
	{
		results << "global sheet found" << endl;
		sleep(2);

		ifstream inputfile;
		inputfile.open(global_rps_dir, ios::in);

		results << "Reading global sheet" << endl;

		string line;
		vector<int> eyeBugs;
		int count = 1;
		float temp_x, temp_y;
		while (getline(inputfile, line))
		{
			std::istringstream iss(line);
			float temp;

			while (iss >> temp)
			{
				if(count==1)
				{
					temp_x = temp;
					count++;
				}
				else
				{
					temp_y = temp;
					RPS.push_back({temp_x, temp_y});
					count=1;
				}
			}
		}

		inputfile.close();

		results << "RPS FOUND" << endl;

		// This is not needed, only here for debuggin purposes //
		for(int counter = 0; counter < sizeofswarm; counter ++)
		{
			cout << "start of loop number " << counter << endl;
			cout << "eyeBug " << counter << " is at (x,y): (" << RPS[counter].x << "," << RPS[counter].y << ")" << endl;
			results << "eyeBug " << counter << " is at (x,y): (" << RPS[counter].x << "," << RPS[counter].y << ")" << endl;
		}
		// End of debugging readback
	}
	else
	{
		results << "error: no RPS available" << endl;
		exit(1);
	}

	// Quadrant assigner //
	cout << "num of quads" << number_of_quadrants << endl;

	quady.resize(number_of_quadrants);

	float sizeofslice = 360/number_of_quadrants;
	cout << "quadrant angle " << sizeofslice << endl;

	vector<bool> ID_assignments(sizeofswarm);
	for(int count = 0; count < number_of_quadrants; count++)
	{ID_assignments[count]=false;}

	for(int count = 0; count < number_of_quadrants; count++)
	{
		results << "CALCULATING QUADRANT " << count << endl;
		vector<float> absolutely_distance(sizeofswarm);
		float current_distance=2*radius;	// Set distance as large
		float tx,ty;
		int ID_selected;
		quadrant holder;
		two_d_coordinates quad_starting_point;

		holder.angle1=count*sizeofslice;
		holder.angle2=holder.angle1+sizeofslice;
		tx = (radius * cos(holder.angle1 * M_PI /180));
		ty = (radius * sin(holder.angle1 * M_PI /180));

		quad_starting_point = {tx,ty};
		results << "starting point before adjustment (x,y): (" << quad_starting_point.x << "," << quad_starting_point.y << ")" << endl;
		quad_starting_point = boundry_condition_adjuster(quad_starting_point, global_map_parameters);
		results << "starting point after adjustment (x,y): (" << quad_starting_point.x << "," << quad_starting_point.y << ")"  << endl;


		// Calculate the absolute distance from starting point to each eyeBug
		for(int count2 = 0; count2 < sizeofswarm; count2++)
		{
			float temp_x = abs(quad_starting_point.x -RPS[count2].x);
			float temp_y = abs(quad_starting_point.y -RPS[count2].y);
			absolutely_distance[count2] = pow((pow(temp_x,2)+pow(temp_y,2)),0.5);

			results << "absolute distance for " << count2 << "is " << absolutely_distance[count2] << endl;
		}
		// Establish which is the shortest of those available
		for(int count2 = 0; count2 < sizeofswarm; count2++)
		{
			if(
					(absolutely_distance[count2]<current_distance)
					&&
					(ID_assignments[count2]==false)
			)
			{
				current_distance = absolutely_distance[count2];
				ID_selected = count2;
			}
		}

		holder.id = ID_selected;
		holder.number = count;
		ID_assignments[ID_selected] = true;
		quady[count]=holder;

		results << "Quadrant " << count << " is assigned eyeBug " << quady[count].id << endl;

		// Quadrant assigned
		if(quady[count].id==ID)
		{
			my_own_quadrant=quady[count];
		}
	}

	results << "My quadrant details" << endl;
	results << "\t\t angle1 = " << my_own_quadrant.angle1 << endl;
	results << "\t\t angle2 = " << my_own_quadrant.angle2 << endl;
	results << "Global rad: " << 	global_map_parameters.rad << endl;
	results << "Global maxx: " << 	global_map_parameters.max_x << endl;
	results << "Global maxy: " << 	global_map_parameters.max_y << endl;
	results << "Global minx: " << 	global_map_parameters.min_x << endl;
	results << "Global miny: " << 	global_map_parameters.min_y << endl;
	results << endl;
	results << "set_global_boundries(float lenny, float widdy) = done \t\t\t\t\t End time= " << (float)clock()/CLOCKS_PER_SEC << endl;
	results << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
	results.flush();
	results.close();
	// This is in here for dubugging only
	//	sizeofswarm=3;
	//	ID = 4;
	// Take it out when you're done
	map_flag=true;
}

void on_eyeBug::network_sign_in(bool eyeBug0)
//*****************************************************************************************************
// on_eyeBug::network_sign_in(bool eyeBug0)
// This function is used to establish where the eyeBug sits within the tBug network. If the boolean input
// is true then the network will run the setup assuming it is the host. If not it will try and join the
// signin sheet of another eyeBug0. If there is no eyeBug0 within the network then the program will abort
// (AY)
//*****************************************************************************************************
{
	//*****************************************************************************************************
	// Open the results file
	//*****************************************************************************************************
	char results_dir[255];
	char signin_sheet[255];
	char global_sheet[255];
	char global_map_parameters[255];
	const char* home_dir = getenv("HOME");	// get home directory
	ofstream results;
	sprintf(results_dir,"%s/tBug_network/results/results.txt",home_dir); // update the rest of the gibberish
	sprintf(signin_sheet,"%s/tBug_network/global/signin_sheet.txt",home_dir); // update the rest of the gibberish
	sprintf(global_sheet,"%s/tBug_network/global/global_sheet.txt",home_dir); // Here is where we store the final object containing the ID
	sprintf(global_map_parameters,"%s/tBug_network/global/global_sheet.txt",home_dir);
	results.open(results_dir, ios::app);
	//*****************************************************************************************************
	ofstream signinsheet_create;
	results << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
	results << "network_sign_in() called\t\t\t\t\t Start time= " << (float)clock()/CLOCKS_PER_SEC << endl;
	results << "Sign in sheet dir = " << signin_sheet << endl;
	results << "Global list of eyeBugs in tBug network is stored at dir = " << global_sheet << endl;

	if(eyeBug0==true)
	{
		//***********************************************************************************************************************************//
		// This command deletes the old sign in sheet, (there shouldn't be one anyway but safety first), and creates a new one
		// It then delays X seconds, (well use sleep cause its hassle free), then reads who signed in, this should give us the
		// number of eyeBugs in swarm and their IDs (AY)
		//***********************************************************************************************************************************//
		system("exec rm -r ~/tBug_network/global/*");			// This should delete all old stuff
		signinsheet_create.open(signin_sheet);					// creates a new sign in sheet

		ID = 0;													// Set starting points and angles
		x=0;
		y=0;
		angle=0;
		got_ID=true;
		local_pose=true;

		results << "signed in with " << ID << " ID" << endl;
		signinsheet_create << ID << endl;						// Sign the sheet
		signinsheet_create.flush();
		signinsheet_create.close();

		results << "Going to sleep" << endl;
		ay_delay(SIGNIN_TIME);											// This gives time for the other eyeBugs to sign in

		//*******************************************************//
		// Read who signed in (Two hands for safety)
		// When eyeBug0 wakes up it checks who has signed in, this will guide how the system progresses
		//*******************************************************//

		if (fexists(signin_sheet)) // check that our file still exists (For safety)
		{
			ifstream signinsheet(signin_sheet);
			string line;
			vector<int> eyeBugs;	// This will be all the eyeBugs who signed in

			while (getline(signinsheet, line))	// Read out the signin sheet line by line
			{
				std::istringstream iss(line);
				int ID_loop;

				while (iss >> ID_loop)
				{
					eyeBugs.push_back(ID_loop);
				}
			}

			remove(signin_sheet);					// have to delete the old one if there is one
			netsignedinIDs = eyeBugs;				// Set the pitchers list to the final list
			sizeofswarm = eyeBugs.size();			// So we know how many eyeBugs will be involved
			net_signin = true;						// Trip this flag

			// create file
			ofstream outputfile;					// This will be the global list of tBugs
			outputfile.open(global_sheet, ios::out);

			for(int count=0; count < sizeofswarm; count++)
			{
				outputfile << netsignedinIDs[count] << endl;
			}

			// Dump the object into that sumbitch
			results << "Creating global tBug list" << endl;
			outputfile.flush();
			outputfile.close();
		}
		else
		{
			results << "ERROR!!! Someone deleted the sign in sheet" << endl;
			exit(1);
		}
		// Create the global_map parameters
	}
	else
	{
		//***********************************************************************************************************************************//
		// This command checks the sign in sheet for 30 seconds. If it exists, (which means that eyeBug0 is waiting for the troops to fall in), then the function
		// checks the sheet and takes the highest ID that hasn't already been allocated, which it returns. (AY)
		//***********************************************************************************************************************************//
		int eyeBugID;
		vector<int> eyeBugs;
		float window = (clock()/CLOCKS_PER_SEC); // Sample window

		results << "Reading the signin sheet from dir = " << signin_sheet << endl;

		// 30 second loop to check if the sign in sheet is there
		while
			(
					((float)(clock()/CLOCKS_PER_SEC)<(window+30))
					&&
					(got_ID==false)
			)
		{
			if (fexists(signin_sheet))
			{
				// If the file exists

				// Read off the existing names
				ifstream signinsheet(signin_sheet);
				string line;
				int eyeBugs;

				while (getline(signinsheet, line))
				{
					std::istringstream iss(line);
					int ID;

					while (iss >> ID)
					{
						eyeBugs=(ID);
					}
				}

				signinsheet.close();
				// This stuff should pop the last number in the ID sequence

				// Add one for the current eyeBugs ID
				eyeBugID=eyeBugs+1;
				ID = eyeBugID;
				x=0;
				y=0;
				angle=0;
				got_ID=true;

				// Sign the signin sheet
				ofstream signoutsheet;
				signoutsheet.open(signin_sheet, ios::app); 	// Open the file but don't overwrite
				signoutsheet << eyeBugID << endl;
				signoutsheet.flush();
				signoutsheet.close();
				results << "Got ID " << ID << endl;
			}
		}

		// This is just a check to ensure that we have an ID, this flag should be set to false otherwise
		if(got_ID==false)
		{
			results << "ERROR!!! missed the window, bowing out" << endl;
			exit(1);
		}

		results << "Reading global sheet from dir = " << global_sheet << endl;

		while(net_signin==false)					// Should only do this once
		{
			if (fexists(global_sheet))
			{
				results << "Reading global sheet" << endl;
				sleep(1);							// Make sure it's not still being written when it is read back

				ifstream globalsheet(global_sheet);	// Read off the sign
				string line;
				vector<int> eyeBugs;

				while (getline(globalsheet, line))
				{
					std::istringstream iss(line);
					int ID_loop;

					while (iss >> ID_loop)
					{
						eyeBugs.push_back(ID_loop);
					}
				}

				netsignedinIDs = eyeBugs;				// Set the pitchers list to the final list
				sizeofswarm = eyeBugs.size();			// Get global eyeBug number
				net_signin = true;
			}
		}

		results << "Network Signin complete" << endl;
		results << "global sheet obtained" << endl;
	}
	//*****************************************************************************************************
	// Close the results file
	//*****************************************************************************************************
	results << "Our swarm ID's are " << endl;
	for(int xer; xer< sizeofswarm; xer++)
	{
		results << netsignedinIDs[xer] << endl;
	}
	results << endl;
	results << "network_sign_in()= done \t\t\t\t\t End time= " << (float)clock()/CLOCKS_PER_SEC << endl;
	results << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
	results.flush();
	results.close();
}

void on_eyeBug::set_LEDs()
{
	//*****************************************************************************************************
	// on_eyeBug::set_LEDs() (TG)
	// This function will set the LED's of the eyeBug to match the sequence allocated through the networksignin
	// A unique 64 bit number has to be calculated here, then split into three 16 bit int so that it can match
	// the necessary configurations for Nick's eBug XMega stuff (AY)
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
	results << "set_LEDs() \t\t\t\t\t Start time= " << (float)clock()/CLOCKS_PER_SEC << endl;
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

	command_packet = TLC5947_SetMultiple(mask.operator[](0), mask.operator[](1), mask.operator[](2), 250, false, 0x00); // Create Command packet
	sender(command_packet);

	results << "turning off Kinect LED" << endl;
	freenect_sync_set_led_off_cv(0);				// Deactive the kinect's camera

	results <<  endl;
	results << "active_leds called: done \t\t\t\t\t End time= " << (float)clock()/CLOCKS_PER_SEC << endl;
	results << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
	//*****************************************************************************************************
	// Close the results file
	//*****************************************************************************************************
	results.flush();
	results.close();
}

void on_eyeBug::image_grabber()
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
	results << "image_grabber() \t\t\t\t\t Start time= " << (float)clock()/CLOCKS_PER_SEC << endl;

	current_rgb = 0;	// Clear out all the old data
	current_rgb = freenect_sync_get_rgb_cv(0);
	current_dep = freenect_sync_get_depth_cv(0);
	current_dep = GlViewColor(current_dep);

	if (!current_rgb)
	{
		results << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
		results << "Kinect not Kinected...(lol)" << endl;
		results << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
		printf("Error: Kinect not connected?\n");
	}
	else
	{
		cvCvtColor(current_rgb, current_rgb, CV_RGB2BGR);				// Converts grabbed image to BGR
		frame_count++;
		results << "Frame number:" << frame_count << endl;
	}

	results << endl;
	results << "image_grabber()= done \t\t\t\t\t End time= " << (float)clock()/CLOCKS_PER_SEC << endl;
	results << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
	results.flush();
	results.close();
}

IplImage* on_eyeBug::rgb_image_grabber()
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
	results << "image_grabber() \t\t\t\t\t Start time= " << (float)clock()/CLOCKS_PER_SEC << endl;

	IplImage* rgb_image;
	rgb_image = freenect_sync_get_rgb_cv(0);
	//	current_dep = freenect_sync_get_depth_cv(0);
	//	current_dep = GlViewColor(current_dep);

	if (!current_rgb)
	{
		results << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
		results << "Kinect not Kinected...(lol)" << endl;
		results << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
		printf("Error: Kinect not connected?\n");
		//		return -1;
	}
	else
	{
		cvCvtColor(rgb_image, rgb_image, CV_RGB2BGR);				// Converts grabbed image to BGR
		frame_count++;
		results << "Frame number:" << frame_count << endl;
	}


	results << endl;
	results << "image_grabber()= done \t\t\t\t\t End time= " << (float)clock()/CLOCKS_PER_SEC << endl;
	results << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
	results.flush();
	results.close();

	return rgb_image;
}

void on_eyeBug::non_zero_rps_found(seen_robotics sighted_rbt)
//*******************************************************************************************************************************************
// non_zero_rps_found(seen_robotics sighted_rbt)
// This function for all eyeBugs except for eyeBug 0, (whos a special child and get's their own function). This function will take
// the pixel coordinates from cycles, the angle and the ellipse data and find a scaling model for pixels to metres. It will then build a right
// angle triangle, allow for the camera offset, (theres a blind spot between the camera and our point of reference), and will extrapolate distance
// based off that. Cycles also returns an angle, which is the anticlockwise angle between led 1 in the sequence and the x plane of the image.
// from this we can figure out where our robot sits in the RPS. After all this is calculated the function will post the local coordinates and
// then wait for eyeBug0 to recieve all the local (x,y)s and post a global sheet, which should be a vector of 2-D coordinates the size
// of the swarm. (AY)
//*******************************************************************************************************************************************
{
	//*****************************************************************************************************
	// Open the results file
	//*****************************************************************************************************
	char results_dir[255];
	char local_rps_dir[255];
	char global_rps_dir[255];
	char eyeBug0_angle_dir[255];
	const char* home_dir = getenv("HOME");	// get home directory

	ofstream results;
	sprintf(results_dir,"%s/tBug_network/results/results.txt",home_dir); // update the rest of the gibberish
	results.open(results_dir, ios::app);
	//*****************************************************************************************************
	results << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
	results << "non_zero_rps_found(seen_robotics sighted_rbt) \t\t\t\t\t Start time= " << (float)clock()/CLOCKS_PER_SEC << endl;


	results << "eyeBug " << ID << " sees eyeBug " << sighted_rbt.id << " at pixel(x,y) = (" << sighted_rbt.x << "," << sighted_rbt.y << ")" << endl;

	// This section converts from pixel values to metres //
	float x_p2m = (float)EYEBUG_RADIUS / sighted_rbt.ellipse.size.height;	// Gets x-axis metres per pixel
	float y_p2m = (float)EYEBUG_RADIUS / sighted_rbt.ellipse.size.width;	// Gets y-axis metres per pixel
	results << "X pixels to metres: " << x_p2m << endl;
	results << "Y pixels to metres: " << y_p2m << endl;

	// This is where the module finds the scale
	float x_dis, y_dis, total_dis;

	// Converts the x pixel to a distance relative to the centre and finds the distance according to our scale
	results << "Number of pixels between ellipse centre and seen x zero point:" ;


	if(sighted_rbt.x>X_AXIS_OFFSET)
	{
		results << (sighted_rbt.x-X_AXIS_OFFSET) << endl;
		x_dis = x_p2m*(sighted_rbt.x-X_AXIS_OFFSET);
	}
	else
	{
		results << (X_AXIS_OFFSET-sighted_rbt.x) << endl;
		x_dis = -x_p2m*(X_AXIS_OFFSET-sighted_rbt.x);
	}

	// Finds the Y distance from the base of the image (plus an additional offset)
	y_dis = (y_p2m*(640 - sighted_rbt.y)) + Y_CAMERA_OFFSET;

	//	Uses pythag to get the distance from the camera.
	// 	z^2=x^2+y^2
	total_dis = pow((pow(x_dis,2)+pow(y_dis,2)), 0.5);

	results << "x distance:" << x_dis << endl;
	results << "y distance:" << y_dis << endl;
	results << "total distance:" << total_dis << endl;

	cout << "x distance:" << x_dis << endl;
	cout << "y distance:" << y_dis << endl;
	cout << "total distance:" << total_dis << endl;

	// Read the angle of eyeBug 0
	sprintf(eyeBug0_angle_dir,"%s/tBug_network/global/ID_0_angle.data",home_dir);
	ifstream angle_offset;
	float theta_1_off;
	angle_offset.open(eyeBug0_angle_dir, ios::in | ios::binary);
	angle_offset.read((char*)&theta_1_off, sizeof(theta_1_off));
	angle_offset.close();

	results << "eyeBug0 has rotated " << theta_1_off << " when this was seen" << endl;
	results << "angle between LED 1 and percieved X plane: " << sighted_rbt.angle << endl;

	// Rotation Matrix implementation
	float theta_3 = (sighted_rbt.angle - theta_1_off);
	cout << "theta_3: " << theta_3 << endl;

	//anticlockwise rotation
	//		x = ((x_dis * cos((theta_3 * M_PI / 180))) - (y_dis * sin((theta_3 * M_PI / 180))));
	//		y = ((x_dis * sin((theta_3 * M_PI / 180))) + (y_dis * cos((theta_3 * M_PI / 180))));

	//clockwise rotation
	x = -((x_dis * cos((theta_3 * M_PI / 180))) + (y_dis * sin((theta_3 * M_PI / 180))));
	y = -(-(x_dis * sin((theta_3 * M_PI / 180))) + (y_dis * cos((theta_3 * M_PI / 180))));

	total_dis = pow((pow(x,2)+pow(y,2)), 0.5);

	results << "I exist at x: " << x << endl;
	results << "I exist at Y: " << y << endl;
	results << "Robot ID: " << ID << endl;
	results << "Robot x: " << x << endl;
	results << "Robot y: " << y << endl;
	results << "Robot angle: " << angle << endl;

	two_d_coordinates local_coordinates = {x,y};
	cout << "(x,y) = (" << local_coordinates.x << "," << local_coordinates.y << ")" << endl;
	cout << "theta: " << (theta_3) << endl;
	cout << "z: " << total_dis << endl;

	// Post the coordinates where everyone can see
	ofstream local_rps;
	sprintf(local_rps_dir,"%s/tBug_network/global/ID_%i_rps.data",home_dir, ID); // update the rest of the gibberish
	results << "Posting local rps in: " << local_rps_dir << endl;

	local_rps.open(local_rps_dir, ios::out | ios::binary);

	// Dump the object into that sumbitch
	local_rps.write ((char*)&local_coordinates, sizeof(local_coordinates));
	local_rps.flush();
	local_rps.close();
	local_pose=true;


	sprintf(global_rps_dir,"%s/tBug_network/global/rps.txt",home_dir); // update the rest of the gibberish
	results << "RPS is " << rps << endl;
	results << "Waiting for eyeBug0 to post in " << endl;
	results << global_rps_dir << endl;

	// Wait here for the RPS now that we know where we are
	while(rps==false)
	{
		if (fexists(global_rps_dir))
		{
			sleep(2);
			ifstream inputfile;
			inputfile.open(global_rps_dir, ios::in);

			vector<two_d_coordinates> RPS;

			results << "Reading global sheet" << endl;

			string line;
			vector<int> eyeBugs;
			int count = 1;
			float temp_x, temp_y;
			while (getline(inputfile, line))
			{
				std::istringstream iss(line);
				float temp;

				while (iss >> temp)
				{
					if(count==1)
					{
						temp_x = temp;
						count++;
					}
					else
					{
						temp_y = temp;
						RPS.push_back({temp_x, temp_y});
						count=1;
					}
				}
			}


			inputfile.close();
			//			RPS.reserve(sizeofswarm);
			results << "RPS FOUND" << endl;

			for(int counter = 0; counter < sizeofswarm; counter ++)
			{
				cout << "start of loop number " << counter << endl;
				cout << "eyeBug " << counter << " is at (x,y): (" << RPS[counter].x << "," << RPS[counter].y << ")" << endl;
				results << "eyeBug " << counter << " is at (x,y): (" << RPS[counter].x << "," << RPS[counter].y << ")" << endl;
			}
			rps=true;
		}
	}
	results << endl;
	results << "rps_found(seen_robotics sighted0)= done \t\t\t\t\t End time= " << (float)clock()/CLOCKS_PER_SEC << endl;
	results << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
	results.flush();
	results.close();
}

void on_eyeBug::eyeBug0_rps_found()
//********************************************************************************************************************
// on_eyeBug::eyeBug0_rps_found()
// This is the contructor for the RPS. It will check the global drive for 2d coordinates for all the other eyeBugs in the
// tBug network. if they are present it will post a global list. If not it will break out of this function after a set amount of
// itterations to allow the eyeBug to rotate so that the other tBug members can get a different angle
//********************************************************************************************************************
{
	//*****************************************************************************************************
	// Open the results file
	//*****************************************************************************************************
	char results_dir[255];
	char local_rps_dir[255];
	char global_rps_dir[255];
	char eyeBug0_angle_dir[255];

	const char* home_dir = getenv("HOME");	// get home directory
	ofstream results;
	sprintf(results_dir,"%s/tBug_network/results/results.txt",home_dir); // update the rest of the gibberish
	results.open(results_dir, ios::app);
	//*****************************************************************************************************
	results << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
	results << "eyeBug0_rps_found() \t\t\t\t\t Start time= " << (float)clock()/CLOCKS_PER_SEC << endl;
	rps = false;

	if(ID!=0)
	{
		results << "This should not have been called, debug code" << endl;
		exit(1);
	}
	else
	{
		sprintf(global_rps_dir,"%s/tBug_network/global/rps.txt",home_dir); // update the rest of the gibberish

		// Post eyeBug0's coordinates //
		sprintf(local_rps_dir,"%s/tBug_network/global/ID_%i_rps.data",home_dir, ID); // update the rest of the gibberish
		sprintf(eyeBug0_angle_dir,"%s/tBug_network/global/ID_0_angle.data",home_dir); // update the rest of the gibberish
		results << "Posting ID(x,y): " << ID << "(" << x << "," << y << ")" << endl;
		results << "In directory: " << local_rps_dir << endl;

		ofstream local_rps;
		local_rps.open(local_rps_dir, ios::out | ios::binary);

		two_d_coordinates local_coordinates = {0,0};
		// Dump the object into that sumbitch
		local_rps.write ((char*)&local_coordinates, sizeof(local_coordinates));
		local_rps.flush();
		local_rps.close();

		results << "Posting eyeBug0 angle: " << angle << endl;
		results << "In directory: " << eyeBug0_angle_dir << endl;

		local_rps.open(eyeBug0_angle_dir, ios::out | ios::binary);

		// Dump the object into that sumbitch
		local_rps.write ((char*)&angle, sizeof(angle));
		local_rps.flush();
		local_rps.close();
		// Now check and make sure that the others are in the global directory
		results << "Clock is showing " << clock() << endl;
		results << "So in seconds land that is: " << clock()/CLOCKS_PER_SEC << endl;

		float window = (clock()/CLOCKS_PER_SEC); // Sample window
		results << "Our window value is: " << window << endl;
		// Need to break out of this condition
		int count=0;
		while(
				(rps==false)
				&&
				(count<10)
		)
		{
			int counter=0;
			vector<two_d_coordinates> Global_RPS;
			for(int eyeD=0;eyeD<sizeofswarm;eyeD++)
			{
				sprintf(local_rps_dir,"%s/tBug_network/global/ID_%i_rps.data",home_dir, eyeD); // update the rest of the gibberish
				if(fexists(local_rps_dir))
				{
					sleep(1);
					counter++;
					ifstream inputfile;
					inputfile.open(local_rps_dir, ios::in | ios::binary);

					two_d_coordinates returned;

					//read our input file
					inputfile.read ((char *)&returned, sizeof(returned));
					inputfile.close();
					cout << returned.x << " thats x" << endl;
					cout << returned.y << " thats y" << endl;
					cout << "eyeBug " << eyeD << " is at (" << returned.x << "," << returned.y << ")" << endl;
					Global_RPS.push_back(returned);
				}
			}


			if(counter==sizeofswarm)
			{
				results << "Global_RPS size:" << Global_RPS.size() << endl;

				if(counter==sizeofswarm)
				{
					cout << "there should be even "<< counter << sizeofswarm << endl;
					rps=true;// True=1, False=0
				}


				ofstream global_rps;
				global_rps.open(global_rps_dir, ios::out);

				for(int count=0; count < sizeofswarm; count++)
				{
					global_rps << Global_RPS[count].x << endl;
					global_rps << Global_RPS[count].y << endl;
				}

				global_rps.flush();
				global_rps.close();
				results << "posted RPS to dir: " << global_rps_dir << endl;
			}
			count++;
			cout << "count is up to " << count << endl;
		}
	}

	results << endl;
	results << "eyeBug0_rps_found(): done \t\t\t\t\t End time= " << (float)clock()/CLOCKS_PER_SEC << endl;
	results << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
	results.flush();
	results.close();
}

void on_eyeBug::update_rps()
//*****************************************************************************************************
// update_rps()
// This will read out all the local coordinates which are in the system at any given point and then
// replace the old global coordinates with the current ones
//*****************************************************************************************************
{
	//*****************************************************************************************************
	// Open the results file
	//*****************************************************************************************************
	char results_dir[255];
	char local_rps_dir[255];
	char global_rps_dir[255];
	const char* home_dir = getenv("HOME");	// get home directory
	ofstream results;
	sprintf(results_dir,"%s/tBug_network/results/results.txt",home_dir); // update the rest of the gibberish
	sprintf(global_rps_dir,"%s/tBug_network/global/rps.txt",home_dir); // update the rest of the gibberish
	results.open(results_dir, ios::app);
	//*****************************************************************************************************
	results << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
	results << "update_rps() \t\t\t\t\t Start time= " << (float)clock()/CLOCKS_PER_SEC << endl;

	vector<two_d_coordinates> Global_RPS;
	for(int eyeD=0;eyeD<sizeofswarm;eyeD++)
	{
		sprintf(local_rps_dir,"%s/tBug_network/global/ID_%i_rps.data",home_dir, eyeD); // update the rest of the gibberish
		if(fexists(local_rps_dir))
		{
			sleep(1);
			ifstream inputfile;
			inputfile.open(local_rps_dir, ios::in | ios::binary);

			two_d_coordinates returned;

			//read our input file
			inputfile.read ((char *)&returned, sizeof(returned));
			inputfile.close();
			cout << returned.x << " thats x" << endl;
			cout << returned.y << " thats y" << endl;
			cout << "eyeBug " << eyeD << " is at (" << returned.x << "," << returned.y << ")" << endl;
			Global_RPS.push_back(returned);
		}
	}

	results << "Updating RPS" << endl;

	ofstream global_rps;
	global_rps.open(global_rps_dir, ios::out);

	for(int count=0; count < sizeofswarm; count++)
	{
		global_rps << Global_RPS[count].x << endl;
		global_rps << Global_RPS[count].y << endl;
	}

	global_rps.flush();
	global_rps.close();
	results << "posted RPS to dir: " << global_rps_dir << endl;
	results << endl;
	results << "eyeBug0_rps_found(): done \t\t\t\t\t End time= " << (float)clock()/CLOCKS_PER_SEC << endl;
	results << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
	results.flush();
	results.close();
}

void on_eyeBug::target_seen(seen_robotics target)
//***************************************************************************************************************************************
// target_seen(seen_robotics target)
// This function does the some magic that the localisation does, converting pixel data into 2d cartesian coordinates, except now instead
// of converting it into a local coordinate, it posts the new value of the seen target in the global directory, where everyone can get
// access to it (AY)
//***************************************************************************************************************************************
{
	//*****************************************************************************************************
	// Open the results file
	//*****************************************************************************************************
	char results_dir[255];
	char target_dir[255];

	const char* home_dir = getenv("HOME");	// get home directory
	ofstream results;
	sprintf(results_dir,"%s/tBug_network/results/results.txt",home_dir); // update the rest of the gibberish
	sprintf(target_dir,"%s/tBug_network/global/target_found.data",home_dir); // target directory
	results.open(results_dir, ios::app);
	//*****************************************************************************************************
	results << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
	results << "target_seen(seen_robotics target) \t\t\t\t\t Start time= " << (float)clock()/CLOCKS_PER_SEC << endl;

	results << "eyeBug " << ID << " sees eyeBug " << target.id << " at pixel(x,y) = (" << target.x << "," << target.y << ")" << endl;

	// This section converts from pixel values to metres //
	float x_p2m = (float)EYEBUG_RADIUS / target.ellipse.size.height;	// Gets x-axis metres per pixel
	float y_p2m = (float)EYEBUG_RADIUS / target.ellipse.size.width;	// Gets y-axis metres per pixel
	results << "X pixels to metres: " << x_p2m << endl;
	results << "Y pixels to metres: " << y_p2m << endl;

	// This is where the module finds the scale
	float x_dis, y_dis, total_dis;

	// Converts the x pixel to a distance relative to the centre and finds the distance according to our scale
	results << "Number of pixels between ellipse centre and seen x zero point:" ;


	if(target.x>X_AXIS_OFFSET)
	{
		results << (target.x-X_AXIS_OFFSET) << endl;
		x_dis = x_p2m*(target.x-X_AXIS_OFFSET);
	}
	else
	{
		results << (X_AXIS_OFFSET-target.x) << endl;
		x_dis = -x_p2m*(X_AXIS_OFFSET-target.x);
	}

	// Finds the Y distance from the base of the image (plus an additional offset)
	y_dis = (y_p2m*(640 - target.y)) + Y_CAMERA_OFFSET;

	//	Uses pythag to get the distance from the camera.
	// 	z^2=x^2+y^2
	total_dis = pow((pow(x_dis,2)+pow(y_dis,2)), 0.5);

	results << "x distance:" << x_dis << endl;
	results << "y distance:" << y_dis << endl;
	results << "total distance:" << total_dis << endl;

	// Rotation Matrix implementation
	float theta_rot = (angle);
	cout << "theta_rot" << theta_rot << endl;

	//clockwise rotation
	x = (x_dis * cos((theta_rot * M_PI / 180))) + (y_dis * sin((theta_rot * M_PI / 180)));
	y = -(x_dis * sin((theta_rot * M_PI / 180))) + (y_dis * cos((theta_rot * M_PI / 180)));

	total_dis = pow((pow(x,2)+pow(y,2)), 0.5);

	results << "I exist at x: " << x << endl;
	results << "I exist at Y: " << y << endl;
	results << "Robot ID: " << ID << endl;
	results << "Robot x: " << x << endl;
	results << "Robot y: " << y << endl;
	results << "Robot angle: " << angle << endl;

	two_d_coordinates target_coordinates = {x,y};
	cout << "(x,y) = (" << target_coordinates.x << "," << target_coordinates.y << ")" << endl;
	cout << "theta: " << (theta_rot) << endl;
	cout << "z: " << total_dis << endl;

	// Post the TARGET coordinates where everyone can see
	ofstream target_rps;

	results << "Posting target rps in: " << target_dir << endl;

	target_rps.open(target_dir, ios::out | ios::binary);

	// Dump the object into that sumbitch
	target_rps.write ((char*)&target_coordinates, sizeof(target_coordinates));
	target_rps.flush();
	target_rps.close();
	local_pose=true;

	results << endl;
	results << "eyeBug0_rps_found(): done \t\t\t\t\t End time= " << (float)clock()/CLOCKS_PER_SEC << endl;
	results << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
	results.flush();
	results.close();

}

void on_eyeBug::rotate_eyeBug(float new_angle)
//***************************************************************************************************************************************
// target_seen(seen_robotics target)
// This function does the some magic that the localisation does, converting pixel data into 2d cartesian coordinates, except now instead
// of converting it into a local coordinate, it posts the new value of the seen target in the global directory, where everyone can get
// access to it (AY)
//***************************************************************************************************************************************
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
	results << "rotate_eyeBug(float new_angle) \t\t\t\t\t Start time= " << (float)clock()/CLOCKS_PER_SEC << endl;
	results << "rotating from angle " << angle << " to " << new_angle << endl;


	int16_t steps=0;
	int16_t freq = 255;
	int8_t stepmode = 3;
	int8_t options = 0;

	vector<char> command_packet;			// To send to UART

	bool aclkwse;
	float rotation_angle;

	if(new_angle>360)						// To find where it sits within the unit circle (AY)
	{
		results << "whoopsy, the input angle is greater than our unit circle range..." << endl;
		new_angle-= ((int)(new_angle/360))*360;
		results << "new_angle = " << new_angle << endl << "Much better" << endl;
	}

	if(angle>new_angle)
	{
		rotation_angle = angle-new_angle;
		aclkwse = false;
	}
	else
	{
		rotation_angle = new_angle-angle;
		aclkwse = true;
	}

	results << "Clockwise Rotation = " << aclkwse << endl;
	results << "Rotation Angle = " << rotation_angle << endl;




	if(rotation_angle!=0)
	{
		steps= floor(rotation_angle*STEPS_P_ANGLE);			// Get number of steps to nearest int
		if(steps>20)
		{
			if(aclkwse==1){command_packet = StepperMotorLeftRightStep(freq, steps, false, stepmode, false, freq, steps, true, stepmode, false, false, options);}
			else{command_packet = StepperMotorLeftRightStep(freq, steps, true, stepmode, false, freq, steps, false, stepmode, false, false, options);}

			results << "Number of steps = " << steps << endl;

			sender(command_packet);

			results << "ay_delay("<< ((steps/230)+1) <<") thrown in to ensure that UART Command is not overwritten" << endl;
			ay_delay(((steps/230)+1));
		}
		else
		{
			// We're not rotating cause the eyeBug goes apeshit on such a small rotation
			new_angle = angle;
		}
	}
	else
	{
		results << "Angle does not require updating dummy!" << endl;
	}
	angle = new_angle;
	results << "New angle = " << angle << endl;

	// Post angle here


	results << endl;
	results << "rotate_eyeBug(float new_angle)= done \t\t\t\t\t End time= " << (float)clock()/CLOCKS_PER_SEC << endl;
	results << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
	results.flush();
	results.close();
}

void on_eyeBug::move_eyeBug_forward_w_pose(float distance)
//************************************************************************************************************************
// on_eyeBug::move_eyeBug(float distance)
// This program should move the eyeBug forwards by the corresponding distance. It will then take the angle, calculate a new
// position and post them globally.
//************************************************************************************************************************
{
	//*****************************************************************************************************
	// Open the results file
	//*****************************************************************************************************
	char results_dir[255];
	char local_rps_dir[255];
	const char* home_dir = getenv("HOME");	// get home directory
	ofstream results;
	sprintf(results_dir,"%s/tBug_network/results/results.txt",home_dir); // update the rest of the gibberish
	results.open(results_dir, ios::app);
	//*****************************************************************************************************
	results << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
	results << "move_eyeBug(float distance) \t\t\t\t\t Start time= " << (float)clock()/CLOCKS_PER_SEC << endl;

	rps=true;

	if(rps==false)
	{
		results << "VOID!!!" << endl;
		results << "trying to move without an RPS" << endl;
		exit(1);
	}
	else
	{
		bool loop_flag=true;
		bool final_steps_flag=false;
		uint32_t steps = floor((STEPS_P_M*distance));
		vector<char> command_packet;			// To send to UART
		float window = (clock()/CLOCKS_PER_SEC);// Timer


		do{
			cout << "number of steps remaining = " << steps << endl;
			if(
					(steps>1000)
			)
			{
				results << "Sending Motor forward to UART" << endl;
				results << "Steps = " << steps << endl;

				command_packet= StepperMotorLeftRightStep(255, 500, true, 1, false, 255, 500, true, 1, false, false, 0);
				sender(command_packet);
				steps-=1000;
				distance = pose_est( 1.1, cam, 10);
				cout << distance << endl;
				x += distance * cos( angle * M_PI / 180);
				y += distance * sin( angle * M_PI / 180);
				cout << "(" << x << "," << y << ")" << endl;
			}
			else if(
					(steps>500)

			)
			{
				results << "Sending Motor forward to UART" << endl;
				results << "Steps = " << steps << endl;

				command_packet= StepperMotorLeftRightStep(255, 250, true, 1, false, 255, 250, true, 1, false, false, 0);
				sender(command_packet);
				steps-=500;
				distance = pose_est( 1.1, cam, 10);
				x += distance * cos( angle * M_PI / 180);
				y += distance * sin( angle * M_PI / 180);
				cout << "(" << x << "," << y << ")" << endl;
			}
			else if (
					(steps<500)
					&&
					(final_steps_flag==false)
			)
			{
				results << "Sending Final Motor forward to UART" << endl;
				results << "Steps = " << steps << endl;

				command_packet= StepperMotorLeftRightStep(255, steps, true, 2, false, 255, steps, true, 2, false, false, 0);
				sender(command_packet);
				steps=0;
				final_steps_flag=true;
			}
			else if
			((steps<1000)
					&&
					(final_steps_flag==true))
			{
				loop_flag=false;
				distance = pose_est( 1.1, cam, 10);
				x += distance * cos( angle * M_PI / 180);
				y += distance * sin( angle * M_PI / 180);
				cout << "(" << x << "," << y << ")" << endl;
			}
		}while(loop_flag==true);
		sleep(3);
	}

	// Update x and y
	results << "New (x,y): (" << x << "," << y << ")" << endl;

	// Post the coordinates where everyone can see
	ofstream local_rps;
	sprintf(local_rps_dir,"%s/tBug_network/global/ID_%i_rps.data",home_dir, ID); // update the rest of the gibberish
	results << "Posting local coordinates rps in: " << local_rps_dir << endl;
	two_d_coordinates local_coordinates = {x,y};

	local_rps.open(local_rps_dir, ios::out | ios::binary);

	// Dump the object into that sumbitch
	local_rps.write ((char*)&local_coordinates, sizeof(local_coordinates));
	local_rps.flush();
	local_rps.close();

	//*****************************************************************************************************
	// Close the results file
	//*****************************************************************************************************
	results << endl;
	results << "move_eyeBug(float distance): done\t\t\t\t\t End time= " << (float)clock()/CLOCKS_PER_SEC << endl;
	results << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
	results.flush();
	results.close();
}

void on_eyeBug::move_eyeBug_forward(float distance)
//************************************************************************************************************************
// on_eyeBug::move_eyeBug(float distance)
// This program should move the eyeBug forwards by the corresponding distance. It will then take the angle, calculate a new
// position and post them globally.
//************************************************************************************************************************
{
	//*****************************************************************************************************
	// Open the results file
	//*****************************************************************************************************
	char results_dir[255];
	char local_rps_dir[255];
	const char* home_dir = getenv("HOME");	// get home directory
	ofstream results;
	sprintf(results_dir,"%s/tBug_network/results/results.txt",home_dir); // update the rest of the gibberish
	results.open(results_dir, ios::app);
	//*****************************************************************************************************
	results << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
	results << "move_eyeBug(float distance) \t\t\t\t\t Start time= " << (float)clock()/CLOCKS_PER_SEC << endl;

	rps=true;

	if(rps==false)
	{
		results << "VOID!!!" << endl;
		results << "trying to move without an RPS" << endl;
		exit(1);
	}
	else
	{
		bool loop_flag=true;
		bool final_steps_flag=false;
		uint32_t steps = floor((STEPS_P_M*distance));
		vector<char> command_packet;			// To send to UART
		float window = (clock()/CLOCKS_PER_SEC);// Timer


		do{
			cout << "number of steps remaining = " << steps << endl;
			if(
					(steps>1000)
					//					&&
					//					(clock()/CLOCKS_PER_SEC>=(float)window+2.5)
			)
			{
				results << "Sending Motor forward to UART" << endl;
				results << "Steps = " << steps << endl;

				command_packet= StepperMotorLeftRightStep(255, 500, true, 1, false, 255, 500, true, 1, false, false, 0);
				sender(command_packet);
				steps-=1000;
				sleep(3);
				//				window = (clock()/CLOCKS_PER_SEC);
			}
			else if(
					(steps>500)
					//						&&
					//						(clock()/CLOCKS_PER_SEC>=(float)window+1)
			)
			{
				results << "Sending Motor forward to UART" << endl;
				results << "Steps = " << steps << endl;

				command_packet= StepperMotorLeftRightStep(255, 250, true, 1, false, 255, 250, true, 1, false, false, 0);
				sender(command_packet);
				steps-=500;
				sleep(3);
			}
			else if (
					(steps<500)
					&&
					(final_steps_flag==false)
			)
			{
				results << "Sending Final Motor forward to UART" << endl;
				results << "Steps = " << steps << endl;

				command_packet= StepperMotorLeftRightStep(255, steps, true, 2, false, 255, steps, true, 2, false, false, 0);
				sender(command_packet);
				steps=0;
				final_steps_flag=true;
				//				window = (clock()/CLOCKS_PER_SEC);
			}
			else if
			(
					(steps<1000)
					&&
					(final_steps_flag==true)
			)
			{
				loop_flag=false;
				sleep(3);
			}
		}while(loop_flag==true);
		sleep(3);
	}

	// Update x and y
	x += distance * cos( angle * M_PI / 180);
	y += distance * sin( angle * M_PI / 180);
	results << "New (x,y): (" << x << "," << y << ")" << endl;
	cout << "New (x,y): (" << x << "," << y << ")" << endl;
	// Post the coordinates where everyone can see
	ofstream local_rps;
	sprintf(local_rps_dir,"%s/tBug_network/global/ID_%i_rps.data",home_dir, ID); // update the rest of the gibberish
	results << "Posting local coordinates rps in: " << local_rps_dir << endl;
	two_d_coordinates local_coordinates = {x,y};

	local_rps.open(local_rps_dir, ios::out | ios::binary);

	// Dump the object into that sumbitch
	local_rps.write ((char*)&local_coordinates, sizeof(local_coordinates));
	local_rps.flush();
	local_rps.close();

	//*****************************************************************************************************
	// Close the results file
	//*****************************************************************************************************
	results << endl;
	results << "move_eyeBug(float distance): done\t\t\t\t\t End time= " << (float)clock()/CLOCKS_PER_SEC << endl;
	results << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
	results.flush();
	results.close();
}


void on_eyeBug::end_tbug()
{
	//*****************************************************************************************************
	// Open the results file
	//*****************************************************************************************************
	char results_dir[255];
	char global_rps_dir[255];
	const char* home_dir = getenv("HOME");	// get home directory
	ofstream results;
	sprintf(results_dir,"%s/tBug_network/results/results.txt",home_dir); // update the rest of the gibberish
	sprintf(global_rps_dir,"%s/tBug_network/global/rps.txt",home_dir);
	results.open(results_dir, ios::app);
	//*****************************************************************************************************
	results << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
	results << "end_tbug() \t\t\t\t\t Start time= " << (float)clock()/CLOCKS_PER_SEC << endl;

	results << "tBug Stats" << endl;
	results << "ID: " << ID << endl;
	results << "(x,y): " << "(" << x << "," << y <<")" << endl;
	results << "angle: " << angle << endl;
	results << "sizeofswarm: " << sizeofswarm << endl;

	results << "Quadrant: " << ID << endl;
	results << "net_signin:" << net_signin << endl;
	results << "map_flag: " << map_flag << endl;
	results << "got_ID: " << got_ID << endl;
	results << "local_pose: " << local_pose << endl;
	results << "rps: " << rps << endl;
	results << "global_map_parameters:" << endl;
	results << "\tglobal_map_parameters.len: " << global_map_parameters.rad << endl;
	results << "\tglobal_map_parameters (max_x,max_y): (" << global_map_parameters.max_x << "," << global_map_parameters.max_y << ")" << endl;
	results << "\tglobal_map_parameters (min_x,min_y): (" << global_map_parameters.min_x << "," << global_map_parameters.min_y << ")" << endl;
	results << "Frame Counter: " << frame_count << endl;
	results << endl;
	results << "tbug_network()= done \t\t\t\t\t End time= " << (float)clock()/CLOCKS_PER_SEC << endl;
	results << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
	results.flush();
	results.close();

	vector<char> command_packet;			// To send to UART
	command_packet = TLC5947_SetAllOff(false, 0x00);
	sender(command_packet);

	remove(global_rps_dir);
	exit(1);
}

