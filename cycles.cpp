///*
// * cycles.cpp
// *
// *  Created on: Sep 20, 2013
// *      Author: al
// */
//
//#ifndef CYCLES_CPP_
//#define CYCLES_CPP_
//
//#include "cycles.h"
//
//// 3.14 (fun_RGB2HC) 	- Convert RGB to Hue and Chroma Images
//void rgb2HC(void *data) //convert from rgb colourspace to hue and chroma
//{
//	using namespace std;
//	struct  __attribute__ ((packed)) rgb
//			{
//		uint8_t b,g,r;
//			};
//
//
//	auto p=(rgb *)data;
//	int M,m;
//
//	for(unsigned i=0;i<WIDTH*HEIGHT;i++)
//	{
//		auto r=p[i].r;
//		auto g=p[i].g;
//		auto b=p[i].b;
//		M=max(r,max(g,b));
//		m=min(r,min(g,b));
//		bool clip=(M-m>128); //zero chroma and hue when chroma too small
//		C[i]=clip*(M-m);
//		H[i]=clip*(M==b?3:(M==r?1:2)); //here, ``hue'' is rounded to RED, GREEN, or BLUE
//	}
//}
//
//// 3.15 (fun_ellipse2circle) - Ellipse to circle (T)
//float ellipse_to_circle(float phi,float rz) //projects ellipse angle onto 3D circle
//{
//	float K=rz*cos(phi);
//	return 2*atan2(sqrt(1-K*K)-cos(phi),K+sin(phi));
//}
//
//// 3.16 (fun_circle2ellipse) - Cricle to Ellipse (T)
//float circle_to_ellipse(float theta,float rz) //projects circle angle onto ellipse in image
//{
//	return atan2(rz+sin(theta),cos(theta));
//}
//
//// 3.17 (fun_knngra) - knn graph partition (T)
//using std::vector;
//vector<vector<led> > knn_graph_partition(const vector<led> &points) //constructs 2-nearest neighbour graph and returns connected components
//																																																																																																																																																																																																																																																																																																																																																																																																														{
//	vector<vector<int> > knngraph(points.size());
//	vector<int> numbers;
//	for(int i=0;i<points.size();i++) numbers.push_back(i);
//
//	for(int i:numbers) //determine all edges in 2-nearest neighbour graph
//	{
//		vector<int> neighbours(3);
//		vector<float> dists(points.size());
//		for(int j:numbers)
//		{
//			auto p=points[j]-points[i];
//			dists[j]=p.x*p.x+p.y*p.y*20; //scale y coordinate by ~4.5 in distance calculation
//		}
//		partial_sort_copy(numbers.begin(),numbers.end(),neighbours.begin(),neighbours.end(),
//				[&dists](int a,int b){return dists[a]<dists[b];});
//
//		knngraph[i].insert(knngraph[i].end(),neighbours.begin(),neighbours.end());
//		knngraph[neighbours[1]].push_back(i);
//		knngraph[neighbours[2]].push_back(i);
//	}
//
//	vector<vector<led> > components;
//	std::set<int> done;
//	for(int i=0;i<knngraph.size();i++) //partition graph into connected components
//	{
//		if(done.count(i)) continue;
//		vector<led> component;
//		std::queue<int> q;
//		q.push(i);
//		while(not q.empty()) //breadth-first spanning tree search
//		{
//			auto v=q.front();
//			q.pop();
//
//			if(done.insert(v).second) component.push_back(points[v]);
//			for(auto j:knngraph[v])
//				if(j!=v and not done.count(j)) q.push(j);
//		}
//		components.push_back(component);
//	}
//	return components;
//																																																																																																																																																																																																																																																																																																																																																																																																														}
//
//// 3.18 (fun_minround) - mimimum rounding (T)
//float min_rounding(vector<std::pair<float,int> > &angles) //minimise rounding when selecting 16 evenly placed points on circle
//{
//	static const float pi=4*atanf(1);
//	float angle=0,min=INFINITY;
//	for(auto &j:angles)
//	{
//		float r=0;
//		for(auto &k:angles)
//		{
//			float x=(k.first-j.first)*8/pi+16;
//			x-=round(x);
//			r+=x*x;
//		}
//		if(r<min)
//		{
//			min=r;
//			angle=j.first;
//		}
//	}
//	return angle;
//}
//
//
//// 3.19 (fun_cycles) - cycles (T)
////vector<robot> cycles(IplImage *picture, CVD::ImageRef size){
////	using namespace std;
////	using namespace cv;
////	Mat hue(HEIGHT,WIDTH,CV_8UC1,H); //hue matrix
////	Mat chroma(HEIGHT,WIDTH,CV_8UC1,C); //chroma matrix
////
////	//    char out[WIDTH*HEIGHT*3]; //output image
////	//    Mat o(HEIGHT,WIDTH,CV_8UC3,out);
////
////	rgb2HC(picture->imageData);
////
////	static const Scalar rgb[]={Scalar(255,0,0),Scalar(0,255,0),Scalar(0,128,255),Scalar(255,255,255)};
////
////	set<led> leds[3];	// Sets LEDs
////
////	for(int colour=1;colour<=3;colour++)
////	{
////		static unsigned char f[WIDTH*HEIGHT];		// Define new char with characters amount equal to amount of pixels
////		for(int i=0;i<WIDTH*HEIGHT;i++) f[i]=(H[i]==colour)?C[i]:0; //select only current colour
////
////		Mat filter(HEIGHT,WIDTH,CV_8UC1,f);
////
////
////		GaussianBlur(filter,filter,Size(3,3),0); 	//inplace 3x3 gaussian blur
////
////		vector <vector<Point> > cont;				// Create a vector using "Point" called cont
////		findContours(filter,cont,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE); //find contours in image
////
////
////		for(auto &i:cont)
////		{
////			RotatedRect r=minAreaRect(i); //find a rectangle around contour
////			Size2f s=r.size;
////			float area=s.width*s.height;
////			if(s.width>2*s.height or s.height>2*s.width or area<=4) continue; //ensure contours are large enough and not too stretched
////			leds[colour-1].insert(led(r.center,colour-1));
////		}
////	}
////
////	for(auto &i:leds[2]) for(auto j=leds[1].begin();j!=leds[1].end();j++) //remove green circles that are too close to a blue circle
////		if(norm(i-*j)<5)
////		{
////			leds[1].erase(j);
////			break;
////		}
////
////	vector<led> points;
////	for(int colour=2;colour>=0;colour--) for(auto &i:leds[colour]) //construct vector of points
////		points.push_back(i);
////
////	auto components=knn_graph_partition(points);
////	vector<robot> robots;
////	for(auto &i:components) if(i.size()>=6) //fit an ellipse to every component with at least 6 points
////	{
////		vector<Point2f> v;
////		for(auto &j:i) v.push_back(j);
////		RotatedRect r=minAreaRect(v);
////		if(r.size.height>4 and r.size.width>4)
////		{
////			RotatedRect e=fitEllipse(v);
////
////			static const float pi=4*atanf(1);
////			float s=sin(e.angle*pi/180);
////			float c=cos(e.angle*pi/180);
////			float w=e.size.width/2;
////			float h=e.size.height/2;
////
////			vector<led> good;
////			for(auto &j:i)
////			{
////				auto p=j-e.center;
////				float x=(c*p.x+s*p.y)/w;
////				float y=(-s*p.x+c*p.y)/h;
////				float r2=x*x+y*y;
////				if(r2>0.85 and r2<1.2) good.push_back(j); //select only points that fit the ellipse well
////			}
////
////			if(good.size()<6) continue;
////
////			v.clear();
////			for(auto &j:good) v.push_back(j);
////
////			e=fitEllipse(v); //refit ellipse to only good points
////
////			s=sin(e.angle*pi/180);
////			c=cos(e.angle*pi/180);
////			w=e.size.width/2;
////			h=e.size.height/2;
////
////			float rz=max(w,h)/FOCAL_WIDTH;
////
////			vector<pair<float,int> > angles;
////			for(auto &j:good) //draw black circles in registered points
////			{
////				auto p=j-e.center;
////				float x=(c*p.x+s*p.y)/w;
////				float y=(-s*p.x+c*p.y)/h;
////
////				float theta=ellipse_to_circle(-atan2(y,x)-pi/2,rz);
////				angles.push_back(make_pair(theta,j.colour));
////				//                    circle(o,j,2,Scalar(0,0,0),-1,CV_AA);
////			}
////
////			float base_angle=min_rounding(angles);
////
////			vector<int> rounded(16,3);
////			for(auto &j:angles) //perform rounding and record colour
////			{
////				int &z=rounded[int(round((j.first-base_angle)*8/pi+16))%16];
////				z=(z==3 or z==j.second?j.second:3);
////			}
////
////			if(count(rounded.begin(),rounded.end(),3)>10) continue; //ensure there are still at least 6 points after rounding
////
////			bool matched=false;
////			pair<int,int> id;
////			for(int j=0;j<sizeof(seqs)/sizeof(seqs[0]);j++) for(int k=0;k<16;k++) //try to match observed sequence to one from the table
////			{
////				int l;
////				for(l=0;l<16;l++) if(rounded[l]!=3 and rounded[l]!=seqs[j][(k+l)%16]) break;
////				if(l==16)
////				{
////					matched^=true;
////					if(not matched) break; //ensure there is only one match
////					id=make_pair(j,k);
////				}
////			}
////			if(not matched)
////			{
////				//                    for(int j=0;j<16;j++) //draw predicted LED positions as rays with appropriate colours
////				//                    {
////				//                        float phi=circle_to_ellipse(j*pi/8+base_angle,rz)+pi/2;
////				//                        float x=cos(phi)*w;
////				//                        float y=-sin(phi)*h;
////				//                        line(o,e.center,e.center+Point2f(x*c-y*s,x*s+y*c),rgb[rounded[j]],1,CV_AA);
////				//                    }
////				continue;
////			}
////			int led0=16-id.second;
////			robots.push_back({id.first,e.center.x,e.center.y,led0*pi/8+base_angle});
////		}
////		else //if ellipse is too ``squashed'', use rectangle bounding the points instead (good approximation)
////		{
////			//                ellipse(o,r,Scalar(255,128,0),1,CV_AA);
////			//TO DO extract sequence of colours robustly (with detection of missing leds)
////			auto e=r;
////			auto good=i;
////
////			static const float pi=4*atanf(1);
////			float s=sin(e.angle*pi/180);
////			float c=cos(e.angle*pi/180);
////			float w=e.size.width/2;
////			float h=e.size.height/2;
////
////			float rz=max(w,h)/FOCAL_WIDTH;
////
////			vector<pair<float,int> > angles;
////			for(auto &j:good) //draw black circles in registered points
////			{
////				auto p=j-e.center;
////				float x=(c*p.x+s*p.y)/w;
////				float y=(-s*p.x+c*p.y)/h;
////
////				float theta=ellipse_to_circle(-atan2(y,x)-pi/2,rz);
////				angles.push_back(make_pair(theta,j.colour));
////				//                    circle(o,j,2,Scalar(0,0,0),-1,CV_AA);
////			}
////
////			float base_angle=min_rounding(angles);
////
////			vector<int> rounded(16,3);
////			for(auto &j:angles) //perform rounding and record colour
////			{
////				int &z=rounded[int(round((j.first-base_angle)*8/pi+16))%16];
////				z=(z==3 or z==j.second?j.second:3);
////			}
////
////			if(count(rounded.begin(),rounded.end(),3)>10) continue; //ensure there are still at least 6 points after rounding
////
////			bool matched=false;
////			pair<int,int> id;
////			for(int j=0;j<sizeof(seqs)/sizeof(seqs[0]);j++) for(int k=0;k<16;k++) //try to match observed sequence to one from the table
////			{
////				int l;
////				for(l=0;l<16;l++) if(rounded[l]!=3 and rounded[l]!=seqs[j][(k+l)%16]) break;
////				if(l==16)
////				{
////					matched^=true;
////					if(not matched) break; //ensure there is only one match
////					id=make_pair(j,k);
////				}
////			}
////			if(not matched)
////			{
////				//                    for(int j=0;j<16;j++) //draw predicted LED positions as rays with appropriate colours
////				//                    {
////				//                        float phi=circle_to_ellipse(j*pi/8+base_angle,rz)+pi/2;
////				//                        float x=cos(phi)*w;
////				//                        float y=-sin(phi)*h;
////				//                        line(o,e.center+Point2f(0,-50),e.center+Point2f(x*c-y*s,x*s+y*c),rgb[rounded[j]],1,CV_AA);
////				//                    }
////				continue;
////			}
////			int led0=16-id.second;
////
////			robots.push_back({id.first,e.center.x,e.center.y,led0*pi/8+base_angle});
////		}
////	}
////	return robots;
////};
//#endif /* CYCLES_CPP_ */
