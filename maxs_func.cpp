/*
 * maxs_func.cpp
 *
 *  Created on: Aug 21, 2013
 *      Author: Al
 */
// OpenCV
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
// LibCVD Stuff
#include <cvd/image_io.h>
#include <cvd/glwindow.h>               //Very cheap and cheerful X window with OpenGL capabilities
#include <cvd/gl_helpers.h>                 //OpenGL wrappers for various CVD types
#include <cvd/vector_image_ref.h>
#include <cvd/image_interpolate.h>
#include <cvd/utility.h>

using namespace CVD;
// 3.10 (fun_cv2CVD) - cv_to_CVD
//%%%%%%%%%%%%%%%%% convert opencv to libcvd %%%%%%%%%%%%%%%%%%%%
void cv_to_CVD(IplImage *depth, BasicImage<unsigned short>& Q)
{
	int i = 0;
	for (int ii = 0; ii < 480; ii++){
		for (int jj = 0; jj < 640; jj++){
			Q[ii][jj] = ((short *)depth->imageData)[i];
			i++;
		}
	}
}

// 3.12 (fun_mm2invdep) - convert mm to inverse depth
//%%%%%%%%%%%%%%%%% convert mm to inverse depth %%%%%%%%%%%%%%%%%
Image<double> convert_mm_to_q(const Image<unsigned short int>& D){
	ImageRef size = D.size();
	Image<double> Q(size);
	ImageRef scan;
	do{
		Q[scan] = 1000.0/D[scan];
	} while(scan.next(size));
	return Q;
}


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

