/*
 * LED_FILTER_2.cpp
 *
 *  Created on: Sep 29, 2013
 *      Author: al
 */

#include "LED_FILTER_2.h"

IplImage* LED_filter_2(IplImage *imginput, ImageRef size, int filter)
{
	CvScalar color;										// This is to allocate what color we want our drawn ellipse to be
	//	int redc,greenc,bluec,counter, redoff, greenoff;	// Counters, offsets and the like
	int ambthresh=60;
	//	cvShowImage("inputter",imginput);

	//	IplImage* imgHSV = cvCreateImage(cvGetSize(imginput), IPL_DEPTH_8U, 3);
	IplImage* imgboolian = cvCreateImage(cvGetSize(imginput), 8, 1);
	//	IplImage* imgGreenThresh= cvCreateImage(cvGetSize(imginput),IPL_DEPTH_8U, 1);
	//	IplImage* imgBlueThresh= cvCreateImage(cvGetSize(imginput),IPL_DEPTH_8U, 1);
	//	IplImage* imgRedThresh= cvCreateImage(cvGetSize(imginput),IPL_DEPTH_8U, 1);
	IplImage* placeholder  = cvCreateImage( cvGetSize(imginput), 8, 3 );
	IplImage* processed  = cvCreateImage( cvGetSize(imginput), 8, 3 );
	IplImage* modified  = cvCreateImage( cvGetSize(imginput), 8, 3 );
	IplImage* rotated = cvCreateImage({480,640}, 8 ,3);

	//	cvCvtColor(imginput, imgHSV, CV_BGR2HSV); //Change the color format from BGR to HSV
	//	switch(filter)
	//	{
	//	case 0://0=under the table
	//		cvInRangeS(imgHSV, cvScalar(38,floor(0.3 * 255),floor(0.3 * 255)), cvScalar(75,(1 * 256),(1 * 256)), imgGreenThresh);
	//		cvInRangeS(imgHSV, cvScalar(75,floor(0.5 * 255),floor(0.5 * 255)), cvScalar(130,(1 * 256),( 1 * 256 )), imgBlueThresh);
	//		cvInRangeS(imgHSV, cvScalar(160,floor(0.3 * 255),floor(0.5 * 255)), cvScalar(179,(1 * 256),( 1 * 256)), imgRedThresh);
	//		redoff=1;
	//		greenoff=4;
	//		break;
	//	case 1://1=Al's room
	//		cvInRangeS(imgHSV, cvScalar(38, floor(0 * 255),floor(0 * 255)), cvScalar(90 ,(1 * 256),(1 *256)), imgGreenThresh);
	//		cvInRangeS(imgHSV, cvScalar(75,floor(0.6 * 255),floor(0.3 * 255)), cvScalar(130,(1 * 256),( 1 * 256 )), imgBlueThresh);
	//		cvInRangeS(imgHSV, cvScalar(150,floor(0.1 * 255),floor(0.1 * 255)), cvScalar(179,(1 * 256),( 1 * 256)), imgRedThresh);
	//		redoff=25;
	//		greenoff=25;
	//		break;
	//	case 2:// lab w/ artificial light
	//		cvInRangeS(imgHSV, cvScalar(160,floor(0.9 * 255),floor(0.1 * 255)), cvScalar(179,(1 * 255),( 0.2 * 255)), imgRedThresh);
	//		cvInRangeS(imgHSV, cvScalar(38,floor(0.3 * 255),floor(0.2 * 255)), cvScalar(75,(1 * 255),( 0.4 * 255)), imgGreenThresh);
	//		cvInRangeS(imgHSV, cvScalar(75,floor(0.2 * 255),floor(0.1 * 255)), cvScalar(130,(0.9 * 255),( 0.4 * 255 )), imgBlueThresh);
	//		redoff=1;
	//		greenoff=1;
	//		break;
	//	case 3:	// In the dark
	//		cvInRangeS(imgHSV, cvScalar(floor((000/360)*180),floor(0.50 * 255),floor(0.70 * 255)), cvScalar(floor((030/360)*180),floor(1.00 * 255),floor( 1.00 * 255)), imgRedThresh);
	//		cvInRangeS(imgHSV, cvScalar(floor((070/360)*180),floor(0.50 * 255),floor(0.46 * 255)), cvScalar(floor((130/360)*180),floor(1.00 * 255),floor( 1.00 * 255)), imgGreenThresh);
	//		cvInRangeS(imgHSV, cvScalar(floor((200/360)*180),floor(0.60 * 255),floor(0.65 * 255)), cvScalar(floor((270/360)*180),floor(1.00 * 255),floor( 1.00 * 255)), imgBlueThresh);
	//		redoff=1; //FINISH THIS BEFORE YOU DO OTHER STUFF
	//		greenoff=1;
	//		break;
	//	default:
	//		std::cout << "no filtration" << std::endl;
	//		break;
	//	}
	//
	//	cvSmooth(imgGreenThresh, imgGreenThresh, CV_GAUSSIAN,9,9); //smooth the binary image using Gaussian kernel
	//	cvSmooth(imgBlueThresh, imgBlueThresh, CV_GAUSSIAN,9,9); //smooth the binary image using Gaussian kernel
	//	cvSmooth(imgRedThresh, imgRedThresh, CV_GAUSSIAN,9,9); //smooth the binary image using Gaussian kernel
	//
	//	cvThreshold( imgGreenThresh, imgGreenThresh, 100, 255, CV_THRESH_BINARY  );
	//	cvThreshold( imgRedThresh, imgRedThresh, 100, 255, CV_THRESH_BINARY  );
	//	cvThreshold( imgBlueThresh, imgBlueThresh, 100, 255, CV_THRESH_BINARY  );
	placeholder = imginput;
	cvScale(imginput,imginput,(0.4));

	//	cvThreshold( imginput, imginput, 100, 255, CV_THRESH_BINARY);
	//Try this if everything else fails /\

	cvZero( imgboolian );
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

	//	cvShowImage("red", imgRedThresh);
	//	cvShowImage("green", imgGreenThresh);
	//	cvShowImage("blue", imgBlueThresh);
	//	cvShowImage("gray",imgboolian);

	if(imgboolian!= 0)
	{
		CvMemStorage* storage = cvCreateMemStorage(0);
		CvSeq* contour = 0;
		cvFindContours( imgboolian, storage, &contour, sizeof(CvContour), CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));
		cvZero( processed );

		for( ; contour != 0; contour = contour->h_next )
		{
			double actual_area = fabs(cvContourArea(contour, CV_WHOLE_SEQ, 0));
			if (actual_area < MIN_AREA)
				continue;
			// Assuming the axes of the ellipse are vertical/perpendicular.
			//
			CvRect rect = ((CvContour *)contour)->rect;
			int A = rect.width / 2;
			int B = rect.height / 2;
			double estimated_area = M_PI * A * B;
			double error = fabs(actual_area - estimated_area);
			if (error > MAX_TOL)
				continue;
			// Allocate colours
			color = CV_RGB(255,255,255);
			// Draw contours
			cvDrawContours( processed, contour, color, color, -1, CV_FILLED, 8, cvPoint(0,0));
//			cvShowImage("outputter", processed);
//			waitKey(0);
		}
	}

	//	cout << "contours written" << endl;
	// Once we're done with the images we need to release the memory so we can reallocate it
	for(int y =0 ; y< imginput->height; y++)
	{
		for(int x = 0; x< (imginput->width); x++)
		{
			if(
					((int)processed->imageData[3*(y*processed->width+x)+0]!=0)
					||
					((int)processed->imageData[3*(y*processed->width+x)+1]!=0)
					||
					((int)processed->imageData[3*(y*processed->width+x)+2]!=0)
			)
			{
				modified->imageData[3*(y*modified->width+x)+0]= placeholder->imageData[3*(y*placeholder->width+x)+0];
				modified->imageData[3*(y*modified->width+x)+1]= placeholder->imageData[3*(y*placeholder->width+x)+1];
				modified->imageData[3*(y*modified->width+x)+2]= placeholder->imageData[3*(y*placeholder->width+x)+2];
			}
		}
	}
	//	cout << "base image filtered" << endl;


//	cvTranspose(modified, rotated);
//	cvFlip(rotated, rotated, 0);
//	cvShowImage("led out", rotated);
//	cvReleaseImage(&modified);
	cvReleaseImage(&imgboolian);
	//	cvReleaseImage(&imginput);
	cvReleaseImage(&processed);
	//	cout << "ready to output" << endl;
	//	cvShowImage("Rotated", rotated);
	//	cvWaitKey(0);
	//	return processed;
	return modified;
}






