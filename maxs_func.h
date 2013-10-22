/*
 * maxs_func.h
 *
 *  Created on: Aug 21, 2013
 *      Author: al
 */

using namespace CVD;
using namespace TooN;

#ifndef MAXS_FUNC_H_
#define MAXS_FUNC_H_

// 3.10 (fun_cv2CVD) - cv_to_CVD
void cv_to_CVD(IplImage *depth, BasicImage<unsigned short>& Q);
// 3.12 (fun_mm2invdep) - convert mm to inverse depth
Image<double> convert_mm_to_q(const Image<unsigned short int>& D);
template <class Cam>
Image<TooN::Vector<2> > get_UV(ImageRef size, Cam& camera);

#endif /* MAXS_FUNC_H_ */
