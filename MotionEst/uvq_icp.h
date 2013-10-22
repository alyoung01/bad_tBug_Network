/*
 * uvq_icp.h
 *
 *  Created on: 5 Oct 2012
 *      Author: max
 */

#ifndef UVQ_ICP_H
#define UVQ_ICP_H

#include "cvd/image.h"
#include "TooN/TooN.h"
#include "TooN/se3.h"
#include "cvd/camera.h"
#include "TooN/SVD.h"
#include "TooN/Cholesky.h"
#include "TooN/lapack.h"
#include <vector>
#include <stdlib.h>
#include "uvq.h"

using namespace std;
using namespace TooN;

UVQ get_uvq(const CVD::Image<double>& Q, const CVD::ImageRef& p, double u_image[][640], double v_image[][640]);

UVQ get_closest_point(const CVD::Image<double>& Q, const UVQ& point, const CVD::ImageRef& p, int scan_size,double u_image[][640], double v_image[][640]);

TooN::SE3<> compute_pose(const CVD::Image<double>& Q1, const CVD::Image<double>& Q2, const TooN::SE3<>& guess, const CVD::Image<TooN::Vector<2> >& UV, Camera::Linear& cam, const int num_samples);

TooN::SE3<> compute_pose_pt2pl(const CVD::Image<double>& Q1, const CVD::Image<double>& Q2, const TooN::SE3<>& guess, const CVD::Image<TooN::Vector<2> >& UV, Camera::Linear& cam, const int num_samples);

TooN::SE3<> compute_pose_bi_pt2pl(const CVD::Image<double>& Q1, const CVD::Image<double>& Q2, const TooN::SE3<>& guess, const CVD::Image<TooN::Vector<2> >& UV, Camera::Linear& cam, const int num_samples);


CVD::Image<double> convert_bi_to_q(const CVD::BasicImage<unsigned short int>& D, double gamma[]);

TooN::SE3<> compute_pose_pt2pl(const CVD::Image<double>& Q1, const CVD::Image<double>& Q2, TooN::SE3<>& guess, vector<Matrix<> > preprocessor_mat, double u_image[][640], double v_image[][640], const int num_samples);

TooN::SE3<> compute_pose_pt2pl_beam(const CVD::Image<double>& Q1, const CVD::Image<double>& Q2, TooN::SE3<>& guess, vector<Matrix<> > preprocessor_mat, double u_image[][640], double v_image[][640], const int num_samples);

TooN::SE3<> compute_pose_pt2pl_Bi_beam(const CVD::Image<double>& Q1, const CVD::Image<double>& Q2, TooN::SE3<>& guess, vector<Matrix<> > preprocessor_mat, double u_image[][640], double v_image[][640], const int num_samples);

TooN::SE3<> compute_pose_pt2pl_Bibeam(const CVD::Image<double>& Q1, const CVD::Image<double>& Q2, TooN::SE3<>& guess, vector<Matrix<> > preprocessor_mat, double u_image[][640], double v_image[][640], const int num_samples);

#endif


