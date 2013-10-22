/*
 * Yuv_LED_filter.cpp
 *
 *  Created on: Oct 1, 2013
 *      Author: al
 */

#ifndef YUV_LED_FILTER_CPP_
#define YUV_LED_FILTER_CPP_

#include "Yuv_LED_filter.h"

using namespace std;
// 3.13 (fun_RGB2LED) 	- Led Filter
void YUV_LED_filter(IplImage *imginput,IplImage *rotated)
//**************************************************************************************************************************************
// void YUV_LED_filter(IplImage *imginput,IplImage *rotated)
// This is our LED identification filter, it acts as a object detection system, as well as classifying the light sources in their particular
// colours. An original image is inputed, which is filtered into YUV colourspace and split into three seperate boolian images, one for each
// of our LED colours. (Red, Green and Blue). A 3x3 gaussian blur is applied to each to increase the area these boolians cover, then
// each image is again thresholded to create a new boolian image
//
// The light source boolian image is then checked for contours, and these contours are used to define the areas which will be checked for colour
// classifications. The area is checked against our LED colour images and a histogram is constructed, then the most featured colour is selected
// and the light source is classified as that particular colour  and recorded in a sequence. A new image is then constructed featuring
// the classified light sources. This is then rotated and returned
//**************************************************************************************************************************************
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
	results << "YUV_LED_filter(IplImage *imginput,IplImage *rotated ) \t\t\t\t\t Start time= " << (float)clock()/CLOCKS_PER_SEC << endl;
	results << "Starting YUV Filter" << endl;

	CvScalar color;			// This is to allocate what color we want our drawn ellipse to be
	int redc,greenc,bluec;	// Counters for our histogram
	int ambthresh=100;		// Out threshold to remove non-light sources

	IplImage* imgYUV = cvCreateImage(cvGetSize(imginput), IPL_DEPTH_8U, 3);
	IplImage* imgboolian = cvCreateImage(cvGetSize(imginput), IPL_DEPTH_8U, 1);
	IplImage* processed  = cvCreateImage( cvGetSize(imginput), IPL_DEPTH_8U, 3);

	cvCvtColor(imginput, imgYUV, CV_BGR2YCrCb); //Change the color format from BGR to HSV

	Mat Y(480,640,CV_8UC1),Cr(480,640,CV_8UC1),Cb(480,640,CV_8UC1);	// Split the YUV image into a 3 Mat boolian image vector
	vector<Mat> YCRCB(3);
	YCRCB[0] = Y;
	YCRCB[1] = Cr;
	YCRCB[2] = Cb;
	split(imgYUV, YCRCB);

	vector<Mat> rgb(3);

	// Find only the colours within the desired range
	Mat G=(Cr<floor(0.3*255))&(Cb<floor(0.5*255));
	Mat R=(Cr>floor(0.65*255))&(Cb<floor(0.6*255))&(Y>floor(0.2*255));
	Mat B=(Cr<floor(0.9*255))&(Cb>floor(0.9*255))&(Y<floor(0.6*255));

	GaussianBlur(R, R, {3,3}, 1, 1);
	GaussianBlur(G, G, {3,3}, 1, 1);
	GaussianBlur(B, B, {3,3}, 1, 1);

	R=(R!=0);
	G=(G!=0);
	B=(B!=0);

//	imwrite("R.png",R);
//	imwrite("G.png",G);
//	imwrite("B.png",B);

	cvScale(imginput,imginput,(0.4));	// Adjust the contrast to remove the low end
	// Create a light source boolian image
	cvZero(imgboolian);
	for(int y =0 ; y< imginput->height; y++)
	{
		for(int x = 0; x< (imginput->width/2); x++)
		{
			if(
					((int)imginput->imageData[3*(y*imginput->width+x)+0]>=ambthresh)
					&&
					((int)imginput->imageData[3*(y*imginput->width+x)+1]>=ambthresh)
					&&
					((int)imginput->imageData[3*(y*imginput->width+x)+2]>=ambthresh)
			)
			{
				imgboolian->imageData[(y*imgboolian->width+x)+0]=255;
			}
			else
			{
				imgboolian->imageData[(y*imgboolian->width+x)+0]=0;
			}
		}
	}

	cvSmooth(imgboolian, imgboolian, CV_GAUSSIAN,3,3); //smooth the binary image using Gaussian kernel

	for(int y =0 ; y< imginput->height; y++)
	{
		for(int x = 0; x< (imginput->width); x++)
		{
			if((int)imgboolian->imageData[y*imgboolian->width+x]!=0)
			{
				imgboolian->imageData[y*imgboolian->width+x]=255;
			}
		}
	}

	// RGB data for histogram
	unsigned char *Rdata = (unsigned char*)(R.data);
	unsigned char *Gdata = (unsigned char*)(G.data);
	unsigned char *Bdata = (unsigned char*)(B.data);

	if(imgboolian!= 0)
	{
		int col; // Which colour we get from our histogram
		int count = 0;	// Number of leds we'll find in our image
		vector<int> sequence;	//

		// Use openCV stuff to find contours
		CvMemStorage* storage = cvCreateMemStorage(0);
		CvSeq* contour = 0;
		cvFindContours( imgboolian, storage, &contour, sizeof(CvContour), CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));

		// Zero off our processed image
		cvZero( processed );
		// Check for each contour
		for( ; contour != 0; contour = contour->h_next )
		{

			// find the floating point absolute area needed
			double actual_area = fabs(cvContourArea(contour, CV_WHOLE_SEQ, 0));
			// Check and make the area it covered is correct
			if (actual_area < MIN_AREA)
				continue;

			// Fir a rectangle over the thing
			CvRect rect = ((CvContour *)contour)->rect;
			int A = rect.width / 2;
			int Ber = rect.height / 2;
			double estimated_area = M_PI * A * Ber;
			double error = fabs(actual_area - estimated_area);
			// Check the error
			if (error > MAX_TOL)
				continue;


			// Construct histogram for the area we we're looking at.
			redc=0;
			greenc=0;
			bluec=0;
			for(int y = (rect.y -floor(rect.height*RECT_OFFSET)) ; y<(rect.y+floor(RECT_OFFSET*rect.height)); y++)
			{
				for(int x = (rect.x-floor(rect.width*RECT_OFFSET)); x< (rect.x+floor(RECT_OFFSET*rect.width)); x++)
				{

					if((y>=0)&&(y<cvGetSize(imginput).height)&&(x>=0)&&(x<cvGetSize(imginput).width))
					{
						if(Rdata[R.step*y + x]!=0)
						{redc++;}
						if(Gdata[G.step*y + x]!=0)
						{greenc++;}
						if(Bdata[B.step*y + x]!=0)
						{bluec++;}
					}
				}
			}

			// Allocate colours
			if(
					(
							(redc!=0)
							||
							(greenc!=0)
							||
							(bluec!=0)
					)
			)	// If loop makes sure that our light source has another color around it
			{
				count++; 	//Increment the light source counter
				if(
						(greenc>redc)
						&&
						(greenc>bluec)
				)
				{
					results << "greencounter:" << greenc << std::endl;
					results << "redcounter:" << redc << std::endl;
					results << "bluecounter:" << bluec << std::endl;
					color = CV_RGB(0,255,0);
					col = 1;
				}
				else if(
						(redc>bluec)
						&&
						(redc>greenc)
				)
				{
					results << "greencounter:" << greenc << std::endl;
					results << "redcounter:" << redc << std::endl;
					results << "bluecounter:" << bluec << std::endl;
					color = CV_RGB( 255, 0, 0);
					col = 0;
				}
				else if(
						(bluec>redc)
						&&
						(bluec>greenc)
				)
				{
					color = CV_RGB(0,0,255);
					results << "greencounter:" << greenc << std::endl;
					results << "redcounter:" << redc << std::endl;
					results << "bluecounter:" << bluec << std::endl;
					col = 2;
				}
				// Draw contours
				results << "Drawing contour number" << count << endl;
				cvDrawContours( processed, contour, color, color, -1, CV_FILLED, 8, cvPoint(0,0));

				sequence.push_back(col);	// Update sequence found
			}
		}
		results << "Light sources in image: " << count << endl;
		results << "sequence: ";
		for(int a =0; a<count; a++)
		{
			results << sequence[a] << " ";
		}
	}

	cvTranspose(processed, rotated);
	cvFlip(rotated, rotated, 0);
	cvReleaseImage(&imgYUV);
	cvReleaseImage(&imgboolian);
	cvReleaseImage(&processed);

	//*****************************************************************************************************
	// Close the results file
	//*****************************************************************************************************
	results << endl;
	results << "move_eyeBug(float distance): done\t\t\t\t\t End time= " << (float)clock()/CLOCKS_PER_SEC << endl;
	results << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
	results.flush();
	results.close();
}
#endif /* YUV_LED_FILTER_CPP_ */
