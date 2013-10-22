/*
 * libfreenect_cv.cpp
 *
 *  Created on: Dec 20, 2012
 *      Author: Alastair Young, Max Wang
 *      	  : ADYOU1 19874650
 *      Name  : Al's build from the basic
 */

#include "libfreenect_cv.h"

// ****************************************************************************************************************************************
// For eyeBug Commands
//****************************************************************************************************************************************
//1. Stop Motors
//StepperMotorStopBoth(bool disable1, bool disable2, bool xBee, int8_t options)
//StepperMotorStopBoth(true, true, false, 0);
//2. Drive Forwards
//StepperMotorLeftRightStep(int16_t freqHz1, int16_t nSteps1, bool directionForward1, int8_t stepMode1, bool rgbON1, int16_t freqHz2, int16_t nSteps2, bool directionForward2, int8_t stepMode2, bool rgbON2, bool xBee, int8_t options)
//StepperMotorLeftRightStep(255, 10000, true, 3, true, 255, 10000, true, 3, true, false, 0);
//3. Rotate antiClockwise
//StepperMotorLeftRightStep(int16_t freqHz1, int16_t nSteps1, bool directionForward1, int8_t stepMode1, bool rgbON1, int16_t freqHz2, int16_t nSteps2, bool directionForward2, int8_t stepMode2, bool rgbON2, bool xBee, int8_t options)
//StepperMotorLeftRightStep(255, 10000, false, 3, true, 255, 10000, true, 3, true, false, 0);
//4. Rotate Clockwise
//StepperMotorLeftRightStep(int16_t freqHz1, int16_t nSteps1, bool directionForward1, int8_t stepMode1, bool rgbON1, int16_t freqHz2, int16_t nSteps2, bool directionForward2, int8_t stepMode2, bool rgbON2, bool xBee, int8_t options)
//StepperMotorLeftRightStep(255, 10000, true, 3, true, 255, 10000, false, 3, true, false, 0);
//****************************************************************************************************************************************
//****************************************************************************************************************************************
// Section 2 (Obj) - Object_Definitions
//****************************************************************************************************************************************

// 2.1 (Obj_rob) - Robot
//
#ifndef _THE_ROBOTS
#define _THE_ROBOTS
struct robot
{
	int id;
	float x,y,angle;
	RotatedRect ellipse;
	Mat output;
};

#endif //_THE_ROBOTS

// 2.2 (Obj_swa)- Swarm

// 2.3 (Obj_led) - LED(T)
using cv::Point2f;
struct led: public Point2f
{
	int colour;
	led(Point2f pos=Point2f(),int colour=-1): Point2f(pos),colour(colour) {}
	bool operator<(const led &other) const
	{
		return std::tie(x,y,colour)<std::tie(other.x,other.y,other.colour);
	}
}; //represents a coloured circle in the image
//


//****************************************************************************************************************************************
// Section 3 (Fun) - The FUNctions
//****************************************************************************************************************************************
// 3.01 (fun_poseest)			- pose estimation						- main 					- (MW)
// 3.02 (fun_RGB2HC) 			- Convert RGB to Hue and Chroma Images  - main					- (TG)
// 3.03 (fun_ellipse2circle) 	- Ellipse to circle 					- main					- (TG)
// 3.04 (fun_circle2ellipse) 	- Circle to Ellipse 					- main					- (TG)
// 3.05 (fun_knngra) 			- knn graph partition 					- main					- (TG)
// 3.06 (fun_minround) 			- mimimum rounding 						- main					- (TG)
// 3.07 (fun_cycles) 			- cycles 								- main					- (TG)
// 3.08 (fun_cycelescons)		- convert cycles struct					- main					- (AY)
// 3.09 (fun_selflocal) 		- self localisation						- main					- (AY)
// 3.10 (fun_move_eyebug)		- move eyebug to position				- main					- (AY)
// 3.11 (fun_mtsp)		 		- move to starting point				- main					- (AY)
// 3.12 (fun_quad_search) 		- Search eyeBugs quadrant				- main					- (AY)
// 3.13 (fun_move_to_target)	- Move the eyeBug to op target			- main					- (AY)

/*************************	Declaring some functions**************************/
float pose_est(float current_cover, Camera::Linear cam, int loop_intterations);
void rgb2HC(void *data);
float ellipse_to_circle(float phi,float rz);
float circle_to_ellipse(float theta,float rz);
vector<vector<led> > knn_graph_partition(const vector<led> &points);
float min_rounding(vector<std::pair<float,int> > &angles);
vector<robot> cycles(IplImage *picture, ImageRef size);
seen_robotics cycles_constructor(robot cyc_output);
void self_localisation(on_eyeBug* current_eye);
void move_eyebug(on_eyeBug* current_eye, two_d_coordinates new_xy);
void move_to_starting_point(on_eyeBug* current_eye);
void quadrant_search_mode(on_eyeBug* current_eye);
void move_to_target(on_eyeBug* current_eye);
/*****************************************************************************/

// 3.01 (fun_poseest)			- pose estimation						- main 					- (MW)

float pose_est(float current_cover, Camera::Linear cam, int loop_intterations)
{
	cout << "pose est" << endl;
	Image<TooN::Vector<2> > UV = get_UV({480,640},cam);
	int loco = 0;
	IplImage *depth;					// This is our depth picture
	double k1 = 1.1863, k2 = 2842.5, k3 = 0.1236;
	double gamma[2048];
	int local_frame_count=0;
	SE3<> final_pose;

	Image<unsigned short> depthImg({480,640});
	Image<unsigned short> depthImg_prev({480,640});

	BasicImage<unsigned short> D_curr = depthImg;
	BasicImage<unsigned short> D_prev = depthImg_prev;



	for (int ii = 0; ii<2048; ii++)
	{
		gamma[ii] = k3 * tan(ii/k2 + k1);
	}

	do{
		depth = freenect_sync_get_depth_cv(0);
		cv::Mat imgMat(depth);

		if (!depth)
		{
			printf("Error: Kinect not connected?\n");
			exit(0);
		}


		cv_to_CVD(depth, D_curr);
		//	std::cout << "D1[50][50]:"<< D_curr[50][50]<<std::endl;
		Image<double> Q_curr = convert_bi_to_q(D_curr,gamma);
		Image<double> Q_prev = convert_bi_to_q(D_prev,gamma);

		// %%%%%%%%%%%%%%%% perform ICP %%%%%%%%%%%%%%%%%%%%
		if (local_frame_count>5)
		{
			SE3<> init, result;
			for(int it=0; it<25; it++)
			{
				result = compute_pose(Q_curr, Q_prev, init, UV, cam, 500);
				init = init*result;
			}

			final_pose = init * final_pose;

			std::cout << "pose estimation:" << std::endl;
			std::cout << final_pose <<std::endl;
			cout << final_pose.get_rotation().get_matrix()(0,1) << endl;
			cout << "x current=" << final_pose.get_translation().my_data[0] << endl;
			cout << "y current=" << final_pose.get_translation().my_data[1] << endl;
			cout << "z current=" << final_pose.get_translation().my_data[2] << endl;
		}
		CVD::copy(D_curr, D_prev, D_curr.size(),ImageRef(0,0),ImageRef(0,0));
		loco++;
		local_frame_count++;
	}while(loco<loop_intterations);

	current_cover+= final_pose.get_translation().my_data[2];

	return final_pose.get_translation().my_data[2];
}

// 3.02 (fun_RGB2HC) 			- Convert RGB to Hue and Chroma Images  - main					- (TG)
void rgb2HC(void *data) //convert from rgb colourspace to hue and chroma
{
	using namespace std;
	struct  __attribute__ ((packed)) rgb
	{
		uint8_t b,g,r;
	};


	auto p=(rgb *)data;
	int M,m;

	for(unsigned i=0;i<WIDTH*HEIGHT;i++)
	{
		auto r=p[i].r;
		auto g=p[i].g;
		auto b=p[i].b;
		M=max(r,max(g,b));
		m=min(r,min(g,b));
		bool clip=(M-m>128); //zero chroma and hue when chroma too small
		C[i]=clip*(M-m);
		H[i]=clip*(M==b?3:(M==r?1:2)); //here, ``hue'' is rounded to RED, GREEN, or BLUE
	}
}

// 3.03 (fun_ellipse2circle) 	- Ellipse to circle 					- main					- (TG)
float ellipse_to_circle(float phi,float rz) //projects ellipse angle onto 3D circle
{
	float K=rz*cos(phi);
	return 2*atan2(sqrt(1-K*K)-cos(phi),K+sin(phi));
}

// 3.04 (fun_circle2ellipse) 	- Circle to Ellipse 					- main					- (TG)
float circle_to_ellipse(float theta,float rz) //projects circle angle onto ellipse in image
{
	return atan2(rz+sin(theta),cos(theta));
}

// 3.05 (fun_knngra) 			- knn graph partition 					- main					- (TG)
using std::vector;
vector<vector<led> > knn_graph_partition(const vector<led> &points) //constructs 2-nearest neighbour graph and returns connected components
{
	vector<vector<int> > knngraph(points.size());
	vector<int> numbers;
	for(int i=0;i<points.size();i++) numbers.push_back(i);

	for(int i:numbers) //determine all edges in 2-nearest neighbour graph
	{
		vector<int> neighbours(3);
		vector<float> dists(points.size());
		for(int j:numbers)
		{
			auto p=points[j]-points[i];
			dists[j]=p.x*p.x+p.y*p.y*20; //scale y coordinate by ~4.5 in distance calculation
		}
		partial_sort_copy(numbers.begin(),numbers.end(),neighbours.begin(),neighbours.end(),
				[&dists](int a,int b){return dists[a]<dists[b];});

		knngraph[i].insert(knngraph[i].end(),neighbours.begin(),neighbours.end());
		knngraph[neighbours[1]].push_back(i);
		knngraph[neighbours[2]].push_back(i);
	}

	vector<vector<led> > components;
	std::set<int> done;
	for(int i=0;i<knngraph.size();i++) //partition graph into connected components
	{
		if(done.count(i)) continue;
		vector<led> component;
		std::queue<int> q;
		q.push(i);
		while(not q.empty()) //breadth-first spanning tree search
		{
			auto v=q.front();
			q.pop();

			if(done.insert(v).second) component.push_back(points[v]);
			for(auto j:knngraph[v])
				if(j!=v and not done.count(j)) q.push(j);
		}
		components.push_back(component);
	}
	return components;
}


// 3.06 (fun_minround) 			- mimimum rounding 						- main					- (TG)
float min_rounding(vector<std::pair<float,int> > &angles) //minimise rounding when selecting 16 evenly placed points on circle
{
	static const float pi=4*atanf(1);
	float angle=0,min=INFINITY;
	for(auto &j:angles)
	{
		float r=0;
		for(auto &k:angles)
		{
			float x=(k.first-j.first)*8/pi+16;
			x-=round(x);
			r+=x*x;
		}
		if(r<min)
		{
			min=r;
			angle=j.first;
		}
	}
	return angle;
}

// 3.07 (fun_cycles) 			- cycles 								- main					- (TG)
//***********************************************************************************************************************
// vector<robot> cycles(IplImage *picture, ImageRef size)
// This was written by Ph.D student and all around smart guy Tony Grubman, this is essentially treated as a black box which
// takes the YUV filtered image and fits an ellipse over it, (to ensure that our light sources fit into the physical expectations
// of the eyeBug. The sequence is then checked and fitted against a preexisting one. If it is the angle between the percieved
// x axis and the 1st LED in the sequence is returned.
//***********************************************************************************************************************
vector<robot> cycles(IplImage *picture, ImageRef size){
	//*****************************************************************************************************
	// Open the results file
	//*****************************************************************************************************
	char results_dir[255];
	char local_img_dir[400];
	const char* home_dir = getenv("HOME");	// get home directory
	ofstream results;
	sprintf(results_dir,"%s/tBug_network/results/results.txt",home_dir); // update the rest of the gibberish
	results.open(results_dir, ios::app);
	//*****************************************************************************************************

	results << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
	results << "vector<robot> cycles(IplImage *picture, ImageRef size) called\t\t\t\t\t Start time= " << (float)clock()/CLOCKS_PER_SEC << endl;

	using namespace std;
	using namespace cv;

	Mat hue(HEIGHT,WIDTH,CV_8UC1,H); //hue matrix
	Mat chroma(HEIGHT,WIDTH,CV_8UC1,C); //chroma matrix

	char out[WIDTH*HEIGHT*3]; //output image
	Mat o(HEIGHT,WIDTH,CV_8UC3,out);
	o=Scalar(0);

	rgb2HC(picture->imageData);

	static const Scalar rgb[]={Scalar(0,0,255),Scalar(0,255,0),Scalar(255,128,0),Scalar(255,255,255)};

	set<led> leds[3];	// Sets LEDs

	for(int colour=1;colour<=3;colour++)
	{
		static unsigned char f[WIDTH*HEIGHT];		// Define new char with characters amount equal to amount of pixels
		for(int i=0;i<WIDTH*HEIGHT;i++) f[i]=(H[i]==colour)?C[i]:0; //select only current colour

		Mat filter(HEIGHT,WIDTH,CV_8UC1,f);

		GaussianBlur(filter,filter,Size(3,3),0); 	//inplace 3x3 gaussian blur

		vector <vector<Point> > cont;				// Create a vector using "Point" called cont
		findContours(filter,cont,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE); //find contours in image

		for(auto &i:cont)
		{
			RotatedRect r=minAreaRect(i); //find a rectangle around contour
			Size2f s=r.size;
			float area=s.width*s.height;
			if(s.width>2*s.height or s.height>2*s.width or area<=4) continue; //ensure contours are large enough and not too stretched
			leds[colour-1].insert(led(r.center,colour-1));
		}
	}

	for(auto &i:leds[2]) for(auto j=leds[1].begin();j!=leds[1].end();j++) //remove green circles that are too close to a blue circle
		if(norm(i-*j)<5)
		{
			leds[1].erase(j);
			break;
		}

	vector<led> points;

	for(int colour=2;colour>=0;colour--) for(auto &i:leds[colour]) //construct vector of points
		points.push_back(i);

	for(auto &p:points) circle(o,p,4,rgb[p.colour],-1,CV_AA);

	auto components=knn_graph_partition(points);

	vector<robot> robots;
	for(auto &i:components) if(i.size()>=6) //fit an ellipse to every component with at least 6 points
	{
		vector<Point2f> v;
		for(auto &j:i) v.push_back(j);
		RotatedRect r=minAreaRect(v);
		if(r.size.height>4 and r.size.width>4)
		{
			RotatedRect e=fitEllipse(v);

			static const float pi=4*atanf(1);
			float s=sin(e.angle*pi/180);
			float c=cos(e.angle*pi/180);
			float w=e.size.width/2;
			float h=e.size.height/2;

			vector<led> good;
			for(auto &j:i)
			{
				auto p=j-e.center;
				float x=(c*p.x+s*p.y)/w;
				float y=(-s*p.x+c*p.y)/h;
				float r2=x*x+y*y;
				if(r2>0.85 and r2<1.2) good.push_back(j); //select only points that fit the ellipse well
			}

			if(good.size()<6) continue;

			v.clear();
			for(auto &j:good) v.push_back(j);

			e=fitEllipse(v); //refit ellipse to only good points

			s=sin(e.angle*pi/180);
			c=cos(e.angle*pi/180);
			w=e.size.width/2;
			h=e.size.height/2;

			float rz=max(w,h)/FOCAL_WIDTH;

			vector<pair<float,int> > angles;
			for(auto &j:good) //draw black circles in registered points
			{
				auto p=j-e.center;
				float x=(c*p.x+s*p.y)/w;
				float y=(-s*p.x+c*p.y)/h;

				float theta=ellipse_to_circle(-atan2(y,x)+pi/2,rz);
				angles.push_back(make_pair(theta,j.colour));
				//                    circle(o,j,2,Scalar(0,0,0),-1,CV_AA);
			}

			float base_angle=min_rounding(angles);

			vector<int> rounded(16,3);
			for(auto &j:angles) //perform rounding and record colour
			{
				int &z=rounded[int(round((j.first-base_angle)*8/pi+16))%16];
				z=(z==3 or z==j.second?j.second:3);
			}

			if(count(rounded.begin(),rounded.end(),3)>10) continue; //ensure there are still at least 6 points after rounding

			bool matched=false;
			pair<int,int> id;
			for(int j=0;j<sizeof(seqs)/sizeof(seqs[0]);j++) for(int k=0;k<16;k++) //try to match observed sequence to one from the table
			{
				int l;
				for(l=0;l<16;l++) if(rounded[l]!=3 and rounded[l]!=seqs[j][(k+l)%16]) break;
				if(l==16)
				{
					matched^=true;
					if(not matched) break; //ensure there is only one match
					id=make_pair(j,k);
				}
			}
			if(not matched)
			{
				for(int j=0;j<16;j++) //draw predicted LED positions as rays with appropriate colours
				{
					float phi=circle_to_ellipse(j*pi/8+base_angle,rz)-pi/2;
					float x=cos(phi)*w;
					float y=-sin(phi)*h;
					line(o,e.center,e.center+Point2f(x*c-y*s,x*s+y*c),rgb[rounded[j]],1,CV_AA);
				}
				continue;
			}
			for(int j=0;j<16;j++) //draw predicted LED positions as rays with appropriate colours
			{
				float phi=circle_to_ellipse(j*pi/8+base_angle,rz)-pi/2;
				float x=cos(phi)*w;
				float y=-sin(phi)*h;
				line(o,e.center,e.center+Point2f(x*c-y*s,x*s+y*c),rgb[rounded[j]],1,CV_AA);
			}
			int led0=16-id.second;
			robots.push_back({id.first,e.center.x,e.center.y,led0*pi/8+base_angle, e, o});
		}
		else //if ellipse is too ``squashed'', use rectangle bounding the points instead (good approximation)
		{
			//                ellipse(o,r,Scalar(255,128,0),1,CV_AA);
			//TO DO extract sequence of colours robustly (with detection of missing leds)
			auto e=r;
			auto good=i;

			static const float pi=4*atanf(1);
			float s=sin(e.angle*pi/180);
			float c=cos(e.angle*pi/180);
			float w=e.size.width/2;
			float h=e.size.height/2;

			float rz=max(w,h)/FOCAL_WIDTH;

			vector<pair<float,int> > angles;
			for(auto &j:good) //draw black circles in registered points
			{
				auto p=j-e.center;
				float x=(c*p.x+s*p.y)/w;
				float y=(-s*p.x+c*p.y)/h;

				float theta=ellipse_to_circle(-atan2(y,x)-pi/2,rz);
				angles.push_back(make_pair(theta,j.colour));
				//    circle(o,j,2,Scalar(0,0,0),-1,CV_AA);
			}

			float base_angle=min_rounding(angles);

			vector<int> rounded(16,3);
			for(auto &j:angles) //perform rounding and record colour
			{
				int &z=rounded[int(round((j.first-base_angle)*8/pi+16))%16];
				z=(z==3 or z==j.second?j.second:3);
			}

			if(count(rounded.begin(),rounded.end(),3)>10) continue; //ensure there are still at least 6 points after rounding

			bool matched=false;
			pair<int,int> id;
			for(int j=0;j<sizeof(seqs)/sizeof(seqs[0]);j++) for(int k=0;k<16;k++) //try to match observed sequence to one from the table
			{
				int l;
				for(l=0;l<16;l++) if(rounded[l]!=3 and rounded[l]!=seqs[j][(k+l)%16]) break;
				if(l==16)
				{
					matched^=true;
					if(not matched) break; //ensure there is only one match
					id=make_pair(j,k);
				}
			}
			if(not matched)
			{
				for(int j=0;j<16;j++) //draw predicted LED positions as rays with appropriate colours
				{
					float phi=circle_to_ellipse(j*pi/8+base_angle,rz)+pi/2;
					float x=cos(phi)*w;
					float y=-sin(phi)*h;
					line(o,e.center+Point2f(0,-50),e.center+Point2f(x*c-y*s,x*s+y*c),rgb[rounded[j]],1,CV_AA);
				}
				continue;
			}
			int led0=16-id.second;

			robots.push_back({id.first,e.center.x,e.center.y,led0*pi/8+base_angle, e, o});
		}
	}

	results << endl;
	results << "cycles()= done \t\t\t\t\t End time= " << (float)clock()/CLOCKS_PER_SEC << endl;
	results << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
	results.flush();
	results.close();
	return robots;
};




template <class Cam>
Image<TooN::Vector<2> > get_UV(ImageRef size, Cam& camera)
{
	Image<TooN::Vector<2> > result(size);
	ImageRef scan;
	do{
		result[scan]=camera.unproject(vec(scan));
	}while(scan.next(size));
	return result;
}


// 3.08 (fun_cycelescons)		- convert cycles struct					- main					- (AY)
seen_robotics cycles_constructor(robot cyc_output)
{
	seen_robotics map_input;
	map_input.x=cyc_output.x;
	map_input.y=cyc_output.y;
	map_input.id=cyc_output.id;
	map_input.angle= (cyc_output.angle*180/M_PI);	// Converts from radians to degrees
	map_input.angle-= ((int)map_input.angle/360) * 360;
	map_input.ellipse=cyc_output.ellipse;
	return map_input;
}

// 3.09 (fun_selflocal) 		- self localisation						- main					- (AY)

void self_localisation(on_eyeBug* current_eye)
//**************************************************************************************************************************************
// void self_localisation(on_eyeBug* current_eye)
// this is our first complimentary function, it has seperate ways of operating, one for eyeBug0 and the other for all other eyeBugs.
//
// eyeBug0 will continuously check for the relative coordinates of the other swarm eyeBugs until they are found. It will rotate
// at regular intervals as testing revealed that changing the eyeBugs image raises the chances of a successful identification.
//
// non0 eyeBugs will need to orientated facing towards eyeBug0, (they can rotate if time is not an issue), although the slow frame rate
// means that this will take a very long time to do so. They continuously grab images and process them for eyeBugs, once they are found they
// post their coordinates then wait for the rest of the swarm to finish localising (AY)
//**************************************************************************************************************************************
{
	//*****************************************************************************************************
	// Open the results file
	//*****************************************************************************************************
	char results_dir[255];
	char local_img_dir[400];
	const char* home_dir = getenv("HOME");	// get home directory
	ofstream results;
	sprintf(results_dir,"%s/tBug_network/results/results.txt",home_dir); // update the rest of the gibberish
	results.open(results_dir, ios::app);
	//*****************************************************************************************************

	results << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
	results << "self_localisation(on_eyeBug* current_eye) called\t\t\t\t\t Start time= " << (float)clock()/CLOCKS_PER_SEC << endl;

	//**************************************************************************************************************************************
	// For eyeBugs with ID's != 0
	//**************************************************************************************************************************************
	if(EYEBUG0==false)
	{
		results << "\t\t eyeBug " << current_eye->whats_my_ID() << " so looking for IDs <=" << current_eye->whats_my_swarm_size() << endl;
		cout << "localisation marker" << endl;

		bool loop_flag = false;
		while(loop_flag==false)
		{
			cout << "Frame count:" << current_eye->whats_my_frame_count() << endl;
			results << "Grabbing new frame" << endl;

			seen_robotics transfer_struct;
			std::vector <robot> cycles_results;
			current_eye->image_grabber();					// Grab next image

			// This will be the output of our YUV image filter
			IplImage* rotated = cvCreateImage({480,640},IPL_DEPTH_8U,3);

			results << "Processing frame via YUV_LED_filter" << endl;

			YUV_LED_filter(current_eye->current_rgb,rotated);	// Use YUV filter (4)
			cycles_results = cycles(rotated, current_eye->whats_my_image_size());

			if(cycles_results.size()!=0)	// If any eyeBugs were seen
			{
				cout << "Cycles: Hit \t\t\t\t\t\t";
				cout << "Frame count:" << current_eye->whats_my_frame_count() << endl;

				// Check through all the results returned by cycles
				for(int temp=0; temp<cycles_results.size(); temp++)
				{
					if(cycles_results[temp].id==0)	// If our central point was seen
					{
						//* Record results of what was seen *//
						results << "Found local coordinates" << endl;
						results << "x = " << cycles_results[temp].x << endl;
						results << "y = " << cycles_results[temp].y << endl;
						results << "angle = " << (cycles_results[temp].angle*180/M_PI) << endl;

						//* Feed the pixel data into the found RPS mode on eyeBug *//
						transfer_struct = cycles_constructor(cycles_results[temp]);
						current_eye->non_zero_rps_found(transfer_struct);
						loop_flag=true; // Hit flag so it stops looking for eyeBug0
					}
					//* Record results of what was seen *//
					results << "x = " << cycles_results[temp].x << endl;
					results << "y = " << cycles_results[temp].y << endl;
					results << "ID = " << cycles_results[temp].id << endl;
					results << "angle = " << cycles_results[temp].angle << endl;

					//* Save Image data into array *//
					ofstream imager;
					sprintf(local_img_dir,
							"%s/tBug_network/results/input_image_ID_%d_frame_num_%d.data",
							home_dir,
							cycles_results[temp].id,
							current_eye->whats_my_frame_count()); // update the rest of the gibberish

					u_int16_t input_array[current_eye->current_rgb->imageSize];

					for(int x = 0; x < current_eye->current_rgb->imageSize; x++)
					{
						input_array[x] = (u_int16_t)current_eye->current_rgb->imageData[x];
					}
					imager.open(local_img_dir, ios::out | ios::binary);
					imager.write((char *)&input_array,sizeof(input_array));
					imager.flush();
					imager.close();
				}
			}
			cvReleaseImage(&rotated);
		}
	}
	else
	{	// This condition is for eyeBug0
		results << "ID must be zero... What's you're ID?" << current_eye->whats_my_ID() << endl;
		results << "about to enter the RPS loop" << endl;

		//This is for the leader. He will not move and scan for those eyeBugs within the swarm
		while(current_eye->is_rps_found()!=true)
		{
			results << "start loop" << endl;
			current_eye->eyeBug0_rps_found(); // This should check for 5 seconds, posting it's current angle at the start of the cycle
			current_eye->rotate_eyeBug((current_eye->whats_my_angle()+20)); // Then rotate
		}
	}
	results << endl;
	results << "self_localisation(on_eyeBug* current_eye)= done \t\t\t\t\t End time= " << (float)clock()/CLOCKS_PER_SEC << endl;
	results << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
	results.flush();
	results.close();
}



// 3.10 (fun_move_eyebug)		- move eyebug to position				- main					- (AY)
void move_eyebug(on_eyeBug* current_eye, two_d_coordinates new_xy)
{
	//*****************************************************************************************************
	// Open the results file
	//*****************************************************************************************************
	char results_dir[255];
	char local_img_dir[255];
	const char* home_dir = getenv("HOME");	// get home directory
	ofstream results;
	sprintf(results_dir,"%s/tBug_network/results/results.txt",home_dir); // update the rest of the gibberish
	results.open(results_dir, ios::app);
	//*****************************************************************************************************
	results << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
	results << "move_eyebug(on_eyeBug* current_eye, two_d_coordinates new_xy) \t\t\t\t\t Start time= " << (float)clock()/CLOCKS_PER_SEC << endl;

	two_d_coordinates current_point = current_eye->whats_my_xy();

	float delx,dely,delh, new_angle;

	delx = new_xy.x - current_point.x;
	cout << "delx:" << delx << endl;
	results << "delx:" << delx << endl;

	dely = new_xy.y - current_point.y;
	cout << "dely:" << dely << endl;
	results << "dely:" << dely << endl;

	delh = pow((pow(delx,2)+pow(dely,2)),0.5);
	cout << "delh:" << delh << endl;
	results << "delh:" << delh << endl;

	// Don't use tan cause it can get confused
	//	new_angle = acos(delx/delh) * 180 / M_PI;

	new_angle = atan(dely/delx) * 180 / M_PI;
	cout << "tan angle:" << new_angle << endl;
	results << "tan angle:" << new_angle << endl;

	if((delx<0)&&(dely<0))
	{new_angle+= 180;}
	if((delx>0)&&(dely<0))
	{new_angle = 360 - new_angle;}

	cout << "new_angle:" << new_angle << endl;
	results << "new_angle:" << new_angle << endl;

	if(delh<0.1)
	{
		results << "No need to move such a small distance" << endl;
	}
	else
	{
		current_eye->rotate_eyeBug(new_angle);
		current_eye->move_eyeBug_forward(delh);
	}

	results << endl;
	results << "move_eyebug(on_eyeBug* current_eye, two_d_coordinates new_xy)= done \t\t\t\t\t End time= " << (float)clock()/CLOCKS_PER_SEC << endl;
	results << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
	results.flush();
	results.close();
}


// 3.11 (fun_mtsp)		 		- move to starting point				- main					- (AY)
void move_to_starting_point(on_eyeBug* current_eye)
//*****************************************************************************************************
// move_to_starting_point()
// Takes the boundry conditions and moves eyeBug to starting point (AY)
//*****************************************************************************************************
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
	results << "move_to_starting_point(on_eyeBug* current_eye) \t\t\t\t\t Start time= " << (float)clock()/CLOCKS_PER_SEC << endl;

	map_boundries bounds=current_eye->whats_my_boundries();
	quadrant quad1=current_eye->whats_my_quadrant();
	float tx,ty;
	two_d_coordinates starting_point, current_point;

	tx = (bounds.rad * cos(quad1.angle1 * M_PI /180));
	ty = (bounds.rad * sin(quad1.angle1 * M_PI /180));

	results << "tx:" << tx << endl;
	results << "ty:" << ty << endl;

	starting_point= {tx,ty};
	current_point= current_eye->whats_my_xy();

	results << "current_point (" << current_point.x << "," << current_point.y << ")" << endl;
	results << "starting point before adjustment (x,y): (" << starting_point.x << "," << starting_point.y << ")" << endl;

	starting_point = boundry_condition_adjuster(starting_point, bounds);
	results << "starting point after adjustment (x,y): (" << starting_point.x << "," << starting_point.y << ")"  << endl;

	move_eyebug(current_eye, starting_point);

	results << endl;
	results << "move_to_starting_point(on_eyeBug* current_eye) = done \t\t\t\t\t End time= " << (float)clock()/CLOCKS_PER_SEC << endl;
	results << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
	results.flush();
	results.close();
}


// 3.12 (fun_quad_search) 		- Search eyeBugs quadrant				- main					- (AY)
void quadrant_search_mode(on_eyeBug* current_eye)
{
	//*****************************************************************************************************
	// Open the results file
	//*****************************************************************************************************
	char results_dir[255];
	char target_dir[255];
	char local_img_dir[255];
	const char* home_dir = getenv("HOME");	// get home directory
	ofstream results;
	sprintf(results_dir,"%s/tBug_network/results/results.txt",home_dir); // update the rest of the gibberish
	sprintf(target_dir,"%s/tBug_network/global/target_found.data",home_dir); // target directory
	results.open(results_dir, ios::app);
	//*****************************************************************************************************
	results << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
	results << "quadrant_search_mode(on_eyeBug* current_eye) \t\t\t\t\t Start time= " << (float)clock()/CLOCKS_PER_SEC << endl;

	// Set search parameters (make global stuff (?))
	quadrant quadr = current_eye->whats_my_quadrant();
	int number_of_angle_points = 3;
	int number_of_radius_points = 4;
	int number_of_rotation_points = 4;
	int number_of_frames_checked = 5;
	bool ascending, search_mode;
	int anglec, radiusc, rotationc;
	float angle_slice, radius_slice, rotation_slice;
	two_d_coordinates current_xy, new_xy;

	results << "Quadrant Search parameters:" << endl;
	results << "\t\tQuadrant ID: " << quadr.id << endl;
	results << "\t\tnumber_of_angle_points: " << number_of_angle_points << endl;
	results << "\t\tnumber_of_radius_points: " << number_of_radius_points << endl;
	results << "\t\tnumber_of_rotation_points: " << number_of_rotation_points << endl;
	results << "\t\tnumber_of_frames_checked: " << number_of_frames_checked << endl;

	angle_slice = abs(quadr.angle2 - quadr.angle1) / number_of_angle_points;
	radius_slice = current_eye->whats_my_boundries().rad / number_of_radius_points;
	rotation_slice = 360 / number_of_rotation_points;
	current_xy = current_eye->whats_my_xy();

	results << "\t\tangle_slice: " << angle_slice << endl;
	results << "\t\tradius_slice: " << radius_slice << endl;
	results << "\t\trotation_slice: " << rotation_slice << endl << endl;

	if(EYEBUG0){current_eye->update_rps();}

	// Search mode
	search_mode = false;
	while(search_mode == false)
	{
		for(radiusc = 0; ((radiusc < number_of_radius_points) && (search_mode != true)); radiusc++)
		{
			results << "radiusc: " << radiusc << endl;
			for(anglec=0; ((anglec < number_of_angle_points) && (search_mode != true)); anglec++)
			{
				// Get new xy
				results << "ascending: " << ascending << endl;

				if(ascending==true)
				{
					new_xy.x = (current_eye->whats_my_boundries().rad - radiusc*radius_slice) * cos((quadr.angle1 + anglec*angle_slice) * M_PI / 180);
					new_xy.y = (current_eye->whats_my_boundries().rad - radiusc*radius_slice) * sin((quadr.angle1 + anglec*angle_slice) * M_PI / 180);
				}
				else
				{
					new_xy.x = (current_eye->whats_my_boundries().rad - radiusc*radius_slice) * cos((quadr.angle2 - anglec*angle_slice) * M_PI / 180);
					new_xy.y = (current_eye->whats_my_boundries().rad - radiusc*radius_slice) * sin((quadr.angle2 - anglec*angle_slice) * M_PI / 180);
				}

				// Check for boundry conditions
				results << "before adjustment (x,y): (" << new_xy.x << "," << new_xy.y << ")" << endl;
				new_xy = boundry_condition_adjuster(new_xy, current_eye->whats_my_boundries());
				results << "New (x,y): (" << new_xy.x << "," << new_xy.y << ")" << endl;

				// Move eyeBug to new point
				move_eyebug(current_eye, new_xy);

				for(rotationc = 0; ((rotationc < number_of_rotation_points) && (search_mode != true)); rotationc++)
				{
					//				cout << "Starting rotation to " << current_eye->whats_my_angle()+(rotationc * rotation_slice) << endl;
					current_eye->rotate_eyeBug(current_eye->whats_my_angle()+(rotationc * rotation_slice));
					if(EYEBUG0){current_eye->update_rps();}

					for(int framec = 0; ((framec < number_of_frames_checked) && (search_mode != true)); framec++)
					{
						if(fexists(target_dir))
						{
							search_mode = true;
						}
						else
						{
							results << "Global frame count: " << current_eye->whats_my_frame_count() << endl;
							results << "Local frame count: " << framec << endl;
							results << "Grabbing new frame" << endl;

							IplImage* rotated = cvCreateImage({480,640},8,3);
							seen_robotics transfer_struct;
							std::vector <robot> cycles_results;

							results << "Processing frame via YUV_LED_filter" << endl;

							YUV_LED_filter(current_eye->rgb_image_grabber(),rotated);	// Use YUV filter (4)
							cycles_results = cycles(rotated, current_eye->whats_my_image_size());

							if(cycles_results.size()!=0)
							{
								results << "Cycles detects something" << endl;
								results << "Global frame count:" << current_eye->whats_my_frame_count() << endl;

								for(int temp=0; temp<cycles_results.size(); temp++)
								{
									if(cycles_results[temp].id==TARGET_ID)
									{
										results << "Found target coordinates" << endl;
										results << "x = " << cycles_results[temp].x << endl;
										results << "y = " << cycles_results[temp].y << endl;
										results << "angle = " << (cycles_results[temp].angle*180/M_PI) << endl;

										transfer_struct = cycles_constructor(cycles_results[temp]);
										current_eye->target_seen(transfer_struct);
										search_mode=true;
									}
									else
									{
										results << "x = " << cycles_results[temp].x << endl;
										results << "y = " << cycles_results[temp].y << endl;
										results << "ID = " << cycles_results[temp].id << endl;
										results << "angle = " << cycles_results[temp].angle << endl;
									}
								}
							}
							cvReleaseImage(&rotated);
						}
					}
				}
			}
			if(ascending==true){ascending=false;}
			else{ascending=true;}
		}
	}
}

// 3.13 (fun_move_to_target)	- Move the eyeBug to op target			- main					- (AY)
void move_to_target(on_eyeBug* current_eye)
{
	//*****************************************************************************************************
	// Open the results file
	//*****************************************************************************************************
	char results_dir[255];
	char global_rps_dir[255];
	char target_dir[255];
	char local_img_dir[255];
	const char* home_dir = getenv("HOME");	// get home directory
	ofstream results;
	sprintf(results_dir,"%s/tBug_network/results/results.txt",home_dir); // update the rest of the gibberish
	sprintf(target_dir,"%s/tBug_network/global/target_found.data",home_dir); // target directory
	sprintf(global_rps_dir,"%s/tBug_network/global/rps.txt",home_dir); // update the rest of the gibberish
	results.open(results_dir, ios::app);
	//*****************************************************************************************************
	results << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
	results << "move_to_target(on_eyeBug* current_eye) \t\t\t\t\t Start time= " << (float)clock()/CLOCKS_PER_SEC << endl;


	sleep(2);
	// Get the RPS //
	vector<two_d_coordinates> RPS;

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

	//Create the boundry points
	two_d_coordinates target_location, my_point;
	vector<two_d_coordinates> coverage_points;

	if(fexists(target_dir))
	{
		ifstream inputfile;
		inputfile.open(target_dir, ios::in | ios::binary);
		//read our input file
		inputfile.read ((char *)&target_location, sizeof(target_location));
		inputfile.close();
		results << "Target location (x,y): (" << target_location.x << "," << target_location.y << ")" << endl;
	}

	// Get points
	for(int count = 0; count < current_eye->whats_my_swarm_size(); count++)
	{
		float angle_slice = 360 / current_eye->whats_my_swarm_size();
		coverage_points[count].x = EYEBUG_RADIUS * cos((angle_slice * count) * M_PI / 180);
		coverage_points[count].y = EYEBUG_RADIUS * sin((angle_slice * count) * M_PI / 180);
	}

	// Get my point
	my_point = coverage_points[current_eye->whats_my_quadrant().number];

	// Move to locations
	float abby_t, abby_p;
	abby_t = abs(pow((pow(target_location.x - current_eye->whats_my_xy().x, 2) + pow(target_location.y - current_eye->whats_my_xy().y, 2)), 0.5));
	abby_p = abs(pow((pow(my_point.x - current_eye->whats_my_xy().x, 2) + pow(my_point.y - current_eye->whats_my_xy().y, 2)), 0.5));

	if(abby_p < abby_t)
	{
		move_eyebug(current_eye, my_point);
	}
	else
	{
		move_to_starting_point(current_eye);
		move_eyebug(current_eye, my_point);
	}

	int photocount = 0;

	// Take three photos, get cycles, rgb and YUV filtered
	while(photocount < 3)
	{
		ofstream imager;
		sprintf(local_img_dir,
				"%s/tBug_network/results/target_%d_frame_num_%d.data",
				home_dir,
				current_eye->whats_my_frame_count()); // update the rest of the gibberish

		u_int16_t input_array[current_eye->current_rgb->imageSize];

		for(int x = 0; x < current_eye->current_rgb->imageSize; x++)
		{
			input_array[x] = (u_int16_t)current_eye->current_rgb->imageData[x];
		}
		imager.open(local_img_dir, ios::out | ios::binary);
		imager.write((char *)&input_array,sizeof(input_array));
		imager.flush();
		imager.close();

		photocount++;
	}
}

//****************************************************************************************************************************************
// Section 4 - Main Loop
//****************************************************************************************************************************************
int main(int argc, char **argv)
{
	//*****************************************************	 The tBug Network **********************************************************//
	on_eyeBug this_eyeBug;
	on_eyeBug* this_pointer = &this_eyeBug;

	this_eyeBug.starter();
	this_eyeBug.network_sign_in(EYEBUG0);
	this_eyeBug.set_LEDs();
	self_localisation(this_pointer);
	this_eyeBug.set_global_boundries(MAP_LENGTH, MAP_WIDTH);
	move_to_starting_point(this_pointer);
	quadrant_search_mode(this_pointer);
	move_to_target(this_pointer);
	this_eyeBug.end_tbug();
	return 0;
	//**********************************************************************************************************************************
}
