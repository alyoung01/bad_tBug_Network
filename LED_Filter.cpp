/*
 * LED_Filter.cpp
 *
 *  Created on: Aug 22, 2013
 *      Author: al
 */


#include "LED_Filter.h"

// 3.13 (fun_RGB2LED) 	- Led Filter
IplImage* LED_filter(IplImage *imginput, ImageRef size, int filter)
{
	CvScalar color;									// This is to allocate what color we want our drawn ellipse to be
	int redc,greenc,bluec,counter, redoff, greenoff;	// Counters, offsets and the like
	int ambthresh=100;
	//	cvShowImage("inputter",imginput);

	IplImage* imgHSV = cvCreateImage(cvGetSize(imginput), IPL_DEPTH_8U, 3);
	IplImage* imgYUV = cvCreateImage(cvGetSize(imginput), IPL_DEPTH_8U, 3);
	IplImage* imgboolian = cvCreateImage(cvGetSize(imginput), 8, 1);
	IplImage* imgGreenThresh= cvCreateImage(cvGetSize(imginput),IPL_DEPTH_8U, 1);
	IplImage* imgBlueThresh= cvCreateImage(cvGetSize(imginput),IPL_DEPTH_8U, 1);
	IplImage* imgRedThresh= cvCreateImage(cvGetSize(imginput),IPL_DEPTH_8U, 1);
	IplImage* processed  = cvCreateImage( cvGetSize(imgboolian), 8, 3 );
	IplImage* rotated = cvCreateImage({480,640}, 8 ,3);


	cvCvtColor(imginput, imgHSV, CV_BGR2HSV); //Change the color format from BGR to HSV
	cvCvtColor(imginput, imgYUV, CV_BGR2YCrCb); //Change the color format from BGR to HSV
	//	Mat Y(640,480,CV_8UC1),Cr(640,480,CV_8UC1),Cb(640,480,CV_8UC1);
	Mat Y(480,640,CV_8UC1),Cr(480,640,CV_8UC1),Cb(480,640,CV_8UC1);
	vector<Mat> YCRCB(3);
	YCRCB[0] = Y;
	YCRCB[1] = Cr;
	YCRCB[2] = Cb;
	split(imgYUV, YCRCB);

	vector<Mat> rgb(3);
	Mat R=(Cr>0xb3)&(Cb<0x81);
	Mat B=(Cr<0x80)&(Cb>0xb0);
	Mat G=(Cr<0x83)&(Cb<0x6f);



	switch(filter)
	{
	case 0://0=under the table
		cvInRangeS(imgHSV, cvScalar(38,floor(0.3 * 255),floor(0.3 * 255)), cvScalar(75,(1 * 256),(1 * 256)), imgGreenThresh);
		cvInRangeS(imgHSV, cvScalar(75,floor(0.5 * 255),floor(0.5 * 255)), cvScalar(130,(1 * 256),( 1 * 256 )), imgBlueThresh);
		cvInRangeS(imgHSV, cvScalar(160,floor(0.3 * 255),floor(0.5 * 255)), cvScalar(179,(1 * 256),( 1 * 256)), imgRedThresh);
		redoff=1;
		greenoff=4;
		break;
	case 1://1=Al's room // In the dark
		cvInRangeS(imgHSV, cvScalar(38, floor(0 * 255),floor(0 * 255)), cvScalar(90 ,(1 * 256),(1 *256)), imgGreenThresh);
		cvInRangeS(imgHSV, cvScalar(75,floor(0.6 * 255),floor(0.3 * 255)), cvScalar(130,(1 * 256),( 1 * 256 )), imgBlueThresh);
		cvInRangeS(imgHSV, cvScalar(150,floor(0.1 * 255),floor(0.1 * 255)), cvScalar(179,(1 * 256),( 1 * 256)), imgRedThresh);
		redoff=25;
		greenoff=25;
		break;
	case 2:// lab w/ artificial light
		cvInRangeS(imgHSV, cvScalar(160,floor(0.9 * 255),floor(0.1 * 255)), cvScalar(179,(1 * 255),( 0.2 * 255)), imgRedThresh);
		cvInRangeS(imgHSV, cvScalar(38,floor(0.3 * 255),floor(0.2 * 255)), cvScalar(75,(1 * 255),( 0.4 * 255)), imgGreenThresh);
		cvInRangeS(imgHSV, cvScalar(75,floor(0.2 * 255),floor(0.1 * 255)), cvScalar(130,(0.9 * 255),( 0.4 * 255 )), imgBlueThresh);
		redoff=1;
		greenoff=1;
		break;
	case 3:	// In the dark
		cvInRangeS(imgHSV, cvScalar(floor((000/360)*180),floor(0.50 * 255),floor(0.70 * 255)), cvScalar(floor((030/360)*180),floor(1.00 * 255),floor( 1.00 * 255)), imgRedThresh);
		cvInRangeS(imgHSV, cvScalar(floor((070/360)*180),floor(0.50 * 255),floor(0.46 * 255)), cvScalar(floor((130/360)*180),floor(1.00 * 255),floor( 1.00 * 255)), imgGreenThresh);
		cvInRangeS(imgHSV, cvScalar(floor((200/360)*180),floor(0.60 * 255),floor(0.65 * 255)), cvScalar(floor((270/360)*180),floor(1.00 * 255),floor( 1.00 * 255)), imgBlueThresh);
		redoff=1; //FINISH THIS BEFORE YOU DO OTHER STUFF
		greenoff=1;
		break;
	case 4: // YUV

		break;
	default:
		std::cout << "no filtration" << std::endl;
		break;
	}


	if(filter!=4)
	{
		cvSmooth(imgGreenThresh, imgGreenThresh, CV_GAUSSIAN,9,9); //smooth the binary image using Gaussian kernel
		cvSmooth(imgBlueThresh, imgBlueThresh, CV_GAUSSIAN,9,9); //smooth the binary image using Gaussian kernel
		cvSmooth(imgRedThresh, imgRedThresh, CV_GAUSSIAN,9,9); //smooth the binary image using Gaussian kernel

		cvThreshold( imgGreenThresh, imgGreenThresh, 100, 255, CV_THRESH_BINARY  );
		cvThreshold( imgRedThresh, imgRedThresh, 100, 255, CV_THRESH_BINARY  );
		cvThreshold( imgBlueThresh, imgBlueThresh, 100, 255, CV_THRESH_BINARY  );
	}

	cvScale(imginput,imginput,(0.4));
	//	cvThreshold( imginput, imginput, 100, 255, CV_THRESH_BINARY);
	//Try this if everything else fails /
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

//	imshow("R",R);
//	imshow("G",G);
//	imshow("B",B);
	unsigned char *Rdata = (unsigned char*)(R.data);
	unsigned char *Gdata = (unsigned char*)(G.data);
	unsigned char *Bdata = (unsigned char*)(B.data);

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
			int Ber = rect.height / 2;
			double estimated_area = M_PI * A * Ber;
			double error = fabs(actual_area - estimated_area);
			if (error > MAX_TOL)
				continue;

			redc=0;
			greenc=0;
			bluec=0;
			counter=0;
//			std::cout << "Start the loop" << std::endl;
			for(int y = (rect.y -floor(rect.height*RECT_OFFSET)) ; y<(rect.y+floor(RECT_OFFSET*rect.height)); y++)
			{
				for(int x = (rect.x-floor(rect.width*RECT_OFFSET)); x< (rect.x+floor(RECT_OFFSET*rect.width)); x++)
				{
					// To allow for the thing to handle boundary conditions, this loop sets count to zero straight away
					//					if((x<=2)||(y<=2)||(y>=imginput->height)||(x>=imginput->width))
					//					{
					//						x=(rect.x+floor(2*rect.width));
					//						y=(rect.y+floor(2*rect.height));
					////						std::cout << "x = " << x ;
					////						std::cout << " y = " << y << std::endl;
					//					}

					if(Rdata[R.step*y + x]!=0)
					{redc++;}
					if(Gdata[G.step*y + x]!=0)
					{greenc++;}
					if(Bdata[B.step*y + x]!=0)
					{bluec++;}
					//					std::cout << "x = " << x ;
					//					std::cout << " y = " << y << std::endl;
					//					if((int)imgRedThresh->imageData[(y*imgRedThresh->width+x)+0]!=0)
					//					{redc++;}
					//					if((int)imgGreenThresh->imageData[(y*imgGreenThresh->width+x)+0]!=0)
					//					{greenc++;}
					//					if((int)imgBlueThresh->imageData[(y*imgBlueThresh->width+x)+0]!=0)
					//					{bluec++;}

					//					counter++;
					//					std::cout << "stuck in the loop" << std::endl;
				}
			}
//			std::cout << "end loop" << std::endl;;
			// Allocate colours
			if(
					(
							(redc!=0)
							||
							(greenc!=0)
							||
							(bluec!=0)
					)
					//					&&
					//					(4*rect.width*rect.height<MAX_AREA)
			)	// If loop makes sure that our light source has another color around it
			{
				//				if(
				//						((2*greenc)>redc)
				//						&&
				//						((greenoff*greenc)>bluec)
				//				)
				if(
						(greenc>redc)
						&&
						(greenc>bluec)
				)
				{
					color = CV_RGB(0,255,0);
				}
				else if(
						(redc>bluec)
						&&
						(redc>greenc)
//						((redoff*redc)>bluec)
//						&&
//						(redc>greenc)
				)
				{
					color = CV_RGB( 255, 0, 0);
				}
				else
				{
					color = CV_RGB(0,0,255);
				}
				//				std::cout << "ellipse deetsinputed" << std::endl;//	cvShowImage("red", imgRedThresh);
				//	cvShowImage("green", imgGreenThresh);
				//	cvShowImage("blue", imgBlueThresh);
				//				std::cout << "greencounter:" << greenc << std::endl;
				//				std::cout << "redcounter:" << redc << std::endl;
				//				std::cout << "bluecounter:" << bluec << std::endl;
				// Draw contours
				cvDrawContours( processed, contour, color, color, -1, CV_FILLED, 8, cvPoint(0,0));
				//				cvShowImage("outputter", processed);
				//				waitKey(0);
			}
		}
		// Once we're done with the images we need to release the memory so we can reallocate it
		cvReleaseMemStorage(&storage);
		cvReleaseImage(&imgHSV);
		cvReleaseImage(&imgboolian);
		cvReleaseImage(&imgGreenThresh);
		cvReleaseImage(&imgBlueThresh);
		cvReleaseImage(&imgRedThresh);
	}
	//	std::cout << "print in loop" << std::endl;

	cvTranspose(processed, rotated);
	cvFlip(rotated, rotated, 0);
//	imwrite("RedYUV.jpg",R);
//	imwrite("GreenYUV.jpg",G);
//	imwrite("BlueYUV.jpg",B);
//	cvSaveImage("input.jpg", imginput);
//	cvSaveImage("output.jpg", rotated);
	cvReleaseImage(&processed);

	//	cvShowImage("Rotated", rotated);
	//	cvWaitKey(0);
	//	return processed;
	return rotated;
}



