/*
 * uvq_icp.cpp
 *
 *  Created on: 5 Oct 2012
 *      Author: max
 */


#include "uvq_icp.h"

#include <cvd/vector_image_ref.h>
#include <TooN/wls.h>
#include "uvq.h"

#include <vector>
#include <stdlib.h>
#include <TooN/helpers.h>
#include <TooN/TooN.h>
#include <TooN/SVD.h>
#include <TooN/Cholesky.h>
#include <TooN/lapack.h>



using namespace CVD;
using namespace TooN;
using namespace std;

Matrix<2> inverse(const Matrix<2>& M){
  Matrix<2> result;
  double det = M(0,0)*M(1,1)-M(0,1)*M(1,0);
  result(0,0) = M(1,1)/det;
  result(1,1) = M(0,0)/det;
  result(0,1) = -M(0,1)/det;
  result(1,0) = -M(1,0)/det;
  return result;
}


inline UVQ get_uvq(const CVD::Image<double>& Q, const CVD::ImageRef& p, const CVD::Image<TooN::Vector<2> >& UV ){
  UVQ uvq;
  //	TooN::Vector<2> r = vec(p);
  uvq.u() = UV[p][0];
  uvq.v() = UV[p][1];
  uvq.q() = Q[p];
  uvq.X() = p.x;
  uvq.Y() = p.y;
  return uvq;
}

const int num_search_offsets = 9;
ImageRef search_offsets[num_search_offsets]={
  ImageRef(0,0),
  ImageRef(2,1),
  ImageRef(1,2),
  ImageRef(-1,2),
  ImageRef(-2,1),
  ImageRef(-2,-1),
  ImageRef(-1,-2),
  ImageRef(1,-2),
  ImageRef(2,-1)
};




//bool get_closest_point(const CVD::Image<double>& Q, const UVQ& point, const Camera::Linear& cam, const CVD::Image<TooN::Vector<2> >& UV, UVQ& result, ImageRef& matchpos){
//  ImageRef p = ir_rounded(cam.project(point.uvq.slice<0,2>()));
//  if(!Q.in_image_with_border(p,1) || Q[p]<=0){
//    return false;
//  }
//  double best_dist = 1000000;
//  for(int i=0; i<num_search_offsets; i++){
//    ImageRef pos = p+search_offsets[i];
//    if(!Q.in_image_with_border(pos,1)) continue;
//    UVQ uvq2=get_uvq(Q,pos,UV);
//    UVQ diff = uvq2-point;
//    double d2 = diff.magnitude_sq();
//    if(d2<best_dist){
//      best_dist=d2;
//      result = uvq2;
//      matchpos = pos;
//    }
//  }
//  return true;
//}

bool get_closest_point(const CVD::Image<double>& Q, const UVQ& point, const Camera::Linear& cam, const CVD::Image<TooN::Vector<2> >& UV, UVQ& result, ImageRef& matchpos){
	ImageRef p = ir_rounded(cam.project(point.uvq.slice<0,2>()));
	int scan_size = 3;
	if(!Q.in_image_with_border(p,scan_size) || Q[p]<=0){
		return false;
	}
//	result.q() = 0;
	ImageRef minp(p.x-scan_size, p.y-scan_size);
	ImageRef maxp(p.x+scan_size+1, p.y+scan_size+1);
	ImageRef scan=minp;
	double best_dist = 1000000;
	do{
		if(!Q.in_image_with_border(scan,scan_size)) continue;
		UVQ uvq2=get_uvq(Q,scan,UV);
		UVQ diff = uvq2-point;
		double d2 = diff.magnitude_sq();
		if(d2<best_dist){
			best_dist=d2;
			result = uvq2;
			matchpos = scan;
		}
	}while(scan.next(minp,maxp));

	if (best_dist>0.10){
		return false;
	}else{
	return true;
	}
}





UVQ get_normal(const CVD::Image<double>& Q,  const CVD::Image<TooN::Vector<2> >& UV, ImageRef p, bool check_badpt){
  Matrix<2> duvdxy;
  duvdxy[0]=0.5*(UV[p+ImageRef(1,0)]-UV[p+ImageRef(-1,0)]);
  duvdxy[1]=0.5*(UV[p+ImageRef(0,1)]-UV[p+ImageRef(0,-1)]);

  Matrix<2> dxyduv = inverse(duvdxy);
  double a,b,c,d;
  if (Q[p+ImageRef(1,0)]<=0){
	  a = Q[p];
	  check_badpt = true;
  }else{
	  a = Q[p+ImageRef(1,0)];
  }
  if (Q[p+ImageRef(-1,0)]<=0){
	  b = Q[p];
	  check_badpt = true;
  }else{
	  b = Q[p+ImageRef(-1,0)];
  }
  if (Q[p+ImageRef(0,1)]<=0){
	  c = Q[p];
	  check_badpt = true;
  }else{
	  c = Q[p+ImageRef(0,1)];
  }
  if (Q[p+ImageRef(0,-1)]<=0){
	  d = Q[p];
	  check_badpt = true;
  }else{
	  d = Q[p+ImageRef(0,-1)];
  }

//  Vector<2> grad = dxyduv * makeVector(-0.5*(Q[p+ImageRef(1,0)]-Q[p+ImageRef(-1,0)]),
//                                       -0.5*(Q[p+ImageRef(0,1)]-Q[p+ImageRef(0,-1)]));
  Vector<2> grad = dxyduv * makeVector(-0.5*(a-b),
                                       -0.5*(c-d));

  UVQ result;
  result.uvq= unproject(grad)/(sqrt(1+grad*grad));
  return result;
}

TooN::SE3<> compute_pose(const CVD::Image<double>& Q1, const CVD::Image<double>& Q2, const TooN::SE3<>& guess, const CVD::Image<TooN::Vector<2> >& UV, Camera::Linear& cam, const int num_samples){

  ImageRef size = Q1.size();

  WLS<6> wls;
  for(int sample = 0; sample < num_samples; sample++){
    // get a random sample
    ImageRef p1;
    do{
      //p1.x = rand()%size.x;
      //p1.y = rand()%size.y;
    	p1.x = 40 + rand()%600;
    	p1.y = 40 + rand()%440;
    }while(Q1[p1]<=0);

    UVQ uvq1=get_uvq(Q1,p1,UV);
    //cout << "uvq1 " << uvq1 << endl;

    UVQ uvq1p = guess * uvq1; // transform according to current guess
    //cout << "uvq1p " << uvq1p << endl;

    UVQ uvq2;
    ImageRef p2;
    bool success=get_closest_point(Q2, uvq1p, cam, UV, uvq2, p2);
    if(!success) continue;

    // get the error between correspondence and transformed im 1 point
    UVQ diff = uvq2-uvq1p;
    
    Matrix<3,6> J = get_J(uvq1p);
    double weight=1.0;
    wls.add_mJ(diff.u(),J[0],weight);
    wls.add_mJ(diff.v(),J[1],weight);
    wls.add_mJ(diff.q(),J[2],weight);
  }
  wls.compute();

 // SE3<> result = SE3<>::exp(wls.get_mu())*guess;
  SE3<> result = SE3<>::exp(wls.get_mu());
  return result;
}

TooN::SE3<> compute_pose_pt2pl(const CVD::Image<double>& Q1, const CVD::Image<double>& Q2, const TooN::SE3<>& guess, const CVD::Image<TooN::Vector<2> >& UV, Camera::Linear& cam, const int num_samples){

  ImageRef size = Q1.size();
  vector<Vector<6> > Jacob;
  vector<double> dist;
  double sum_dist_beyond = 0;
  int count_occlusion = 0;
  int count_beyond = 0;
  vector<double> diff_buff;
  bool checkpt;

  WLS<6> wls;
  for(int sample = 0; sample < num_samples; sample++){
    // get a random sample
    ImageRef p1;
    do{
      p1.x = rand()%size.x;
      p1.y = rand()%size.y;
    }while(Q1[p1]<=0);


    UVQ uvq1=get_uvq(Q1,p1,UV);

    UVQ uvq1p = guess * uvq1; // transform according to current guess

    UVQ uvq2;
    ImageRef p2;
    bool success = get_closest_point(Q2, uvq1p, cam, UV, uvq2, p2);
    if(!success) continue;
    UVQ normal = get_normal(Q2,UV,p2, checkpt);
    if(checkpt) continue;

    // get the error between correspondence and transformed im 1 point
    UVQ diff = uvq2-uvq1p;

    double d = normal.uvq * diff.uvq;

    Matrix<3,6> J = get_J(uvq1p);

    diff_buff.push_back(d);
    Jacob.push_back(normal.uvq*J);

    double z_sub;
    z_sub = 1.0/uvq1p.q() - 1.0/uvq2.q();
    dist.push_back(z_sub);

    if (z_sub>0){
    	sum_dist_beyond += z_sub;
    	count_beyond++;
        	}else{
        		count_occlusion++;
        	}


 //   double weight=1.0;
 //   wls.add_mJ(d, normal.uvq*J, weight);
  }

  double mean_beyond = sum_dist_beyond/count_beyond;
  int num_point = count_beyond+count_occlusion;

  for(int ii = 0; ii < num_point; ii++){
	  double weight;
  	  	  if(dist[ii]>0){
  	  		  weight = mean_beyond/(mean_beyond+dist[ii]);
  	  	  }else{
  	  		  weight = 1.0;
  	  	  }
  	  	  wls.add_mJ(diff_buff[ii],Jacob[ii],weight);
    	}

  wls.compute();

  SE3<> result = SE3<>::exp(wls.get_mu())*guess;
  return result;
}

TooN::SE3<> compute_pose_bi_pt2pl(const CVD::Image<double>& Q1, const CVD::Image<double>& Q2, const TooN::SE3<>& guess, const CVD::Image<TooN::Vector<2> >& UV, Camera::Linear& cam, const int num_samples){

  ImageRef size = Q1.size();

  WLS<6> wls;
  vector<double> diff_buff;
  vector<Vector<6> > Jacob;
  vector<double> dist;
  double sum_dist_beyond = 0;
  int count_occlusion = 0;
  int count_beyond = 0;
  bool checkpt;

  for(int sample = 0; sample < num_samples; sample++){
    // get a random sample
    ImageRef p1;
    do{
      p1.x = rand()%size.x;
      p1.y = rand()%size.y;
    }while(Q1[p1]<=0);


    UVQ uvq1=get_uvq(Q1,p1,UV);

    UVQ uvq1p = guess * uvq1; // transform according to current guess

    UVQ uvq2;
    ImageRef p2;
    bool success = get_closest_point(Q2, uvq1p, cam, UV, uvq2, p2);
    if(!success) continue;
    UVQ normal = get_normal(Q2,UV,p2,checkpt);

    // get the error between correspondence and transformed im 1 point
    UVQ diff = uvq2-uvq1p;

    double d = normal.uvq * diff.uvq;

    Matrix<3,6> J = get_J(uvq1p);

    diff_buff.push_back(d);
    Jacob.push_back(normal.uvq*J);
 //   double weight=1.0;
 //   wls.add_mJ(d, normal.uvq*J, weight);
    double z_sub;
    z_sub = 1.0/uvq1p.q() - 1.0/uvq2.q();
    dist.push_back(z_sub);

    if (z_sub>0){
    	sum_dist_beyond += z_sub;
    	count_beyond++;
    }else{
    	count_occlusion++;
    }


    // now go from Q2 to Q1!
    do{
      p2.x = 1+rand()%(size.x-2);
      p2.y = 1+rand()%(size.y-2);
    }while(Q2[p2]<=0);

    uvq2=get_uvq(Q2,p2,UV);

    UVQ uvq2p = guess.inverse() * uvq2; // transform according to current guess

    success = get_closest_point(Q1, uvq2p, cam, UV, uvq1, p1);
    if(!success) continue;
    // get the normal from image 2 as before!!
    normal = get_normal(Q2,UV,p2,checkpt);

    // transform the closest point we found in image1 back into image2's frame using our guess
    uvq1p = guess * uvq1;

    // get the error between correspondence and transformed im 1 point
    diff = uvq2-uvq1p;

    d = normal.uvq * diff.uvq;

    J = get_J(uvq1p);

    diff_buff.push_back(d);
    Jacob.push_back(normal.uvq*J);
     //   double weight=1.0;
     //   wls.add_mJ(d, normal.uvq*J, weight);
//    double z_sub;
    z_sub = 1.0/uvq1p.q() - 1.0/uvq2.q();
    dist.push_back(z_sub);

    if (z_sub>0){
    	sum_dist_beyond += z_sub;
    	count_beyond++;
        	}else{
        		count_occlusion++;
        	}

//    weight=1.0;
//    wls.add_mJ(d, normal.uvq*J, weight);
  	  }

  double mean_beyond = sum_dist_beyond/count_beyond;
  int num_point = count_beyond+count_occlusion;

  for(int ii = 0; ii < num_point; ii++){
	  double weight;
	  if(dist[ii]>0){
		  weight = mean_beyond/(mean_beyond+dist[ii]);
	  }else{
		  weight = 1.0;
	  }
	  wls.add_mJ(diff_buff[ii],Jacob[ii],weight);
  }


  wls.compute();

  SE3<> result = SE3<>::exp(wls.get_mu())*guess;
  return result;
}


Image<double> convert_bi_to_q(const BasicImage<unsigned short int>& D, double gamma[]){
  ImageRef size = D.size();
  Image<double> Q(size);
  ImageRef scan;
  do{
    Q[scan] = 1/gamma[D[scan]];
  } while(scan.next(size));
  return Q;
}
#if 0

TooN::SE3<> compute_pose_pt2pl(const CVD::Image<double>& Q1, const CVD::Image<double>& Q2, TooN::SE3<>& guess, vector<Matrix<> > preprocessor_mat, double u_image[][640], double v_image[][640], const int num_samples){

  const int scan_size=3; // how much of image 2 to search for the closest point

  ImageRef size = Q1.size();
  int max_X = size.x;
  int max_Y = size.y;


  SE3<> result=guess;

  for(int it=0; it<5; it++){
    WLS<6> wls;
    for(int sample = 0; sample < num_samples; sample++){
      // get a random sample
      ImageRef p1;
      Vector<6> J;
      do{
        p1.x = rand()%max_X;
        p1.y = rand()%max_Y;
      }while(Q1[p1]<0 || Q1[p1]==0);


      UVQ uvq1=get_uvq(Q1,p1,u_image,v_image);

      //	std::cout << "first:"<<uvq1.uvq<<std::endl;

      Matrix<4,1> temp_buffer;

      temp_buffer(0,0) = uvq1.u();
      temp_buffer(1,0) = uvq1.v();
      temp_buffer(2,0) = 1;
      temp_buffer(3,0) = uvq1.q();

      temp_buffer = result * temp_buffer;

      double temp = temp_buffer(2,0);
      temp_buffer = temp_buffer*(1.0/temp);

      //			UVQ uvq1p = guess * uvq1; // transform according to current guess
      UVQ uvq1p;
      uvq1p.u() = temp_buffer(0,0);
      uvq1p.v() = temp_buffer(1,0);
      uvq1p.q() = temp_buffer(3,0);

      //         std::cout << "second:"<<uvq1p.uvq<<std::endl;

      // find pos in im2
      TooN::Vector<2> v;
      //			ImageRef p2 = ir_rounded(cam.project(uvq1p.uvq.slice<0,2>()));

      v[0] = uvq1p.u()*600+320;
      v[1] = uvq1p.v()*600+240;

      ImageRef p2 = ir_rounded(v);



      if(Q2.in_image(p2) && Q2[p2]>0 && Q2[p2]<15){

        // find correspondence in im 2
        UVQ uvq2 = get_closest_point(Q2, uvq1p, p2, scan_size, u_image,v_image);

        Vector<4> uvq2_temp,uvq1p_temp;
        uvq2_temp[0]=uvq2.u();
        uvq2_temp[1]=uvq2.v();
        uvq2_temp[2]=1;
        uvq2_temp[3]=uvq2.q();
        uvq1p_temp[0]=uvq1p.u();
        uvq1p_temp[1]=uvq1p.v();
        uvq1p_temp[2]=1;
        uvq1p_temp[3]=uvq1p.q();

        // get the error between correspondence and transformed im 1 point
        if (uvq1p_temp[3] != uvq2_temp[3]){
          if (uvq1p_temp[3] < uvq2_temp[3]){
            uvq2_temp *= (uvq1p_temp[3]/uvq2_temp[3]);
          }
          else{
            uvq1p_temp *= (uvq2_temp[3]/uvq1p_temp[3]);
          }
        }

        Vector<4> diffbuffer;
        diffbuffer = uvq1p_temp - uvq2_temp;
        diffbuffer[3] = uvq1p_temp[3];
        //	cout << "diff:"<<diffbuffer<<endl;
        Matrix<4,1> diff;
        diff[0][0]=diffbuffer[0];
        diff[1][0]=diffbuffer[1];
        diff[2][0]=diffbuffer[2];
        diff[3][0]=diffbuffer[3];

        ImageRef p_cor;
        p_cor.x = uvq2.X();
        p_cor.y = uvq2.Y();

        Matrix<9,1> Q;
        int count_Q = 0;
        for (int ii = 0; ii < 3; ii++){
          for (int jj = 0; jj <3; jj++){
            if (Q2[p_cor.y+ii-1][p_cor.x+jj-1]>0){
              Q[count_Q][0] = Q2[p_cor.y+ii-1][p_cor.x+jj-1];
            }
            else{
              Q[count_Q][0] = Q2[p_cor.y][p_cor.x];
            }
            count_Q++;
          }
        }

	//		cout << "Q:" << endl;
	//		cout << Q << endl;


        //			Matrix<9,3> L;
        //			int ii,jj;
        //			ii = p_cor.y;
        //			jj = p_cor.x;
        //			L(0, 0) = u_image[ii-1][jj-1];
        //		    L(0, 1) = v_image[ii-1][jj-1];
        //		    L(0, 2) = 1.0;
        //		    L(1, 0) = u_image[ii-1][jj];
        //		    L(1, 1) = v_image[ii-1][jj];
        //		    L(1, 2) = 1.0;
        //		    L(2, 0) = u_image[ii-1][jj+1];
        //		    L(2, 1) = v_image[ii-1][jj+1];
        //		    L(2, 2) = 1.0;
        //		    L(3, 0) = u_image[ii][jj-1];
        //		    L(3, 1) = v_image[ii][jj-1];
        //		    L(3, 2) = 1.0;
        //		    L(4, 0) = u_image[ii][jj];
        //		    L(4, 1) = v_image[ii][jj];
        //	        L(4, 2) = 1.0;
        //		    L(5, 0) = u_image[ii][jj+1];
        //		    L(5, 1) = v_image[ii][jj+1];
        //		    L(5, 2) = 1.0;
        //		    L(6, 0) = u_image[ii+1][jj-1];
        //		    L(6, 1) = v_image[ii+1][jj-1];
        //		    L(6, 2) = 1.0;
        //		    L(7, 0) = u_image[ii+1][jj];
        //			L(7, 1) = v_image[ii+1][jj];
        //			L(7, 2) = 1.0;
        //			L(8, 0) = u_image[ii+1][jj+1];
        //			L(8, 1) = v_image[ii+1][jj+1];
        //			L(8, 2) = 1.0;


        Matrix<3,9> L_mat;
        L_mat = preprocessor_mat.at(p_cor.x + p_cor.y*640);

        //			Matrix<3,3> tempMat;
        //			Matrix<3,9> precompute;
        //			SVD<3> svd;

        //			tempMat = L.T()*L;
        //			svd.compute(tempMat);
        //			precompute = svd.backsub(L.T());
        //			Matrix<3,1> normal_buffer = precompute*Q;

        Matrix<3,1> normal_buffer = L_mat*Q;





        Vector<3> normal3;
        normal3[0]=normal_buffer[0][0];
        normal3[1]=normal_buffer[1][0];
        normal3[2]=normal_buffer[2][0];
        TooN::normalize(normal3);

        //        cout << "normal:" << normal3<< endl;

        Matrix<1,4> normal;
        normal[0][0]=normal3[0];
        normal[0][1]=normal3[1];
        normal[0][2]=normal3[2];
        normal[0][3]=0.0;

        //errors = -(normal*diff)(0,0);
        double errors;
        //errors = -normal[0][0]*diff[0][0] - normal[0][1]*diff[1][0] - normal[0][2]*diff[2][0] - normal[0][3]*diff[3][0];
        errors = -(normal*diff)(0,0);

        J[0] = uvq1p.q()*normal[0][0];
        J[1] = uvq1p.q()*normal[0][1];
        J[2] = uvq1p.q()*normal[0][2];
        //    J.slice(0,3) = normal3*uvq1p.q();

        J[3] = uvq1p.v()*normal[0][2] - normal[0][1];
        J[4] = normal[0][0] - uvq1p.u()*normal[0][2];
        J[5] = uvq1p.u()*normal[0][1] - uvq1p.v()*normal[0][0];


        wls.add_mJ(errors,J,1.0);

      }
    }
    wls.compute();
    result = SE3<>::exp(wls.get_mu())*result;
  }

  return result;
}


TooN::SE3<> compute_pose_pt2pl_beam(const CVD::Image<double>& Q1, const CVD::Image<double>& Q2, TooN::SE3<>& guess, vector<Matrix<> > preprocessor_mat, double u_image[][640], double v_image[][640], const int num_samples){

  const int scan_size=3; // how much of image 2 to search for the closest point

  ImageRef size = Q1.size();
  int max_X = size.x;
  int max_Y = size.y;


  SE3<> result=guess;

  for(int it=0; it<5; it++){
    WLS<6> wls;
    vector<UVQ > samples;
    vector<UVQ > corr;
    vector<double> error_buff;
    vector<Vector<6> > Jacob;
    vector<double> dist;
    //		double sum_dist_occlusion = 0;
    double sum_dist_beyond = 0;
    int count_occlusion = 0;
    int count_beyond =0;
    for(int sample = 0; sample < num_samples; sample++){
      // get a random sample
      ImageRef p1;
      Vector<6> J;
      do{
        p1.x = rand()%max_X;
        p1.y = rand()%max_Y;
      }while(Q1[p1]<0 || Q1[p1]==0);


      UVQ uvq1=get_uvq(Q1,p1,u_image,v_image);

      Matrix<4,1> temp_buffer;

      temp_buffer(0,0) = uvq1.u();
      temp_buffer(1,0) = uvq1.v();
      temp_buffer(2,0) = 1;
      temp_buffer(3,0) = uvq1.q();

      temp_buffer = result * temp_buffer;

      double temp = temp_buffer(2,0);
      temp_buffer = temp_buffer*(1.0/temp);

      //			UVQ uvq1p = guess * uvq1; // transform according to current guess
      UVQ uvq1p;
      uvq1p.u() = temp_buffer(0,0);
      uvq1p.v() = temp_buffer(1,0);
      uvq1p.q() = temp_buffer(3,0);

      // find pos in im2
      TooN::Vector<2> v;

      v[0] = uvq1p.u()*600+320;
      v[1] = uvq1p.v()*600+240;

      ImageRef p2 = ir_rounded(v);

      uvq1p.X() = p2.x;
      uvq1p.Y() = p2.y;

      if(Q2.in_image(p2) && Q2[p2]>0 ){

        // find correspondence in im 2
        UVQ uvq2 = get_closest_point(Q2, uvq1p, p2, scan_size, u_image,v_image);

        samples.push_back(uvq1p);
        corr.push_back(uvq2);

        Vector<4> uvq2_temp,uvq1p_temp;
        uvq2_temp[0]=uvq2.u();
        uvq2_temp[1]=uvq2.v();
        uvq2_temp[2]=1;
        uvq2_temp[3]=uvq2.q();
        uvq1p_temp[0]=uvq1p.u();
        uvq1p_temp[1]=uvq1p.v();
        uvq1p_temp[2]=1;
        uvq1p_temp[3]=uvq1p.q();

        // get the error between correspondence and transformed im 1 point
        if (uvq1p_temp[3] != uvq2_temp[3]){
          if (uvq1p_temp[3] < uvq2_temp[3]){
            uvq2_temp *= (uvq1p_temp[3]/uvq2_temp[3]);
          }
          else{
            uvq1p_temp *= (uvq2_temp[3]/uvq1p_temp[3]);
          }
        }

        Vector<4> diffbuffer;
        diffbuffer = uvq1p_temp - uvq2_temp;
        diffbuffer[3] = uvq1p_temp[3];

        Matrix<4,1> diff;
        diff[0][0]=diffbuffer[0];
        diff[1][0]=diffbuffer[1];
        diff[2][0]=diffbuffer[2];
        diff[3][0]=diffbuffer[3];

        ImageRef p_cor;
        p_cor.x = uvq2.X();
        p_cor.y = uvq2.Y();

        Matrix<9,1> Q;
        int count_Q = 0;
        for (int ii = 0; ii < 3; ii++){
          for (int jj = 0; jj <3; jj++){
            if (Q2[p_cor.y+ii-1][p_cor.x+jj-1]>0){
              Q[count_Q][0] = Q2[p_cor.y+ii-1][p_cor.x+jj-1];
            }
            else{
              Q[count_Q][0] = Q2[p_cor.y][p_cor.x];
            }
            count_Q++;
          }
        }

        Matrix<3,9> L_mat;
        L_mat = preprocessor_mat.at(p_cor.x + p_cor.y*640);

        Matrix<3,1> normal_buffer = L_mat*Q;

        Vector<3> normal3;
        normal3[0]=normal_buffer[0][0];
        normal3[1]=normal_buffer[1][0];
        normal3[2]=normal_buffer[2][0];
        TooN::normalize(normal3);

        Matrix<1,4> normal;
        normal[0][0]=normal3[0];
        normal[0][1]=normal3[1];
        normal[0][2]=normal3[2];
        normal[0][3]=0.0;

        double errors;
        errors = -(normal*diff)(0,0);
        error_buff.push_back(errors);

        J[0] = uvq1p.q()*normal[0][0];
        J[1] = uvq1p.q()*normal[0][1];
        J[2] = uvq1p.q()*normal[0][2];

        J[3] = uvq1p.v()*normal[0][2] - normal[0][1];
        J[4] = normal[0][0] - uvq1p.u()*normal[0][2];
        J[5] = uvq1p.u()*normal[0][1] - uvq1p.v()*normal[0][0];

        Jacob.push_back(J);

        double distance_sub;
        distance_sub = 1.0/uvq1p.q() - 1.0/uvq2.q();
        dist.push_back(distance_sub);

        if(distance_sub>0){
          sum_dist_beyond += distance_sub;
          count_beyond++;
        }else{
          //				sum_dist_occlusion += distance_sub;
          count_occlusion++;
        }
      }
    }
    double mean_dist_beyond = sum_dist_beyond/count_beyond;
    //		double mean_dist_occlusion = sum_dist_occlusion/count_occlusion;
    int num_sample = count_beyond + count_occlusion;

    for (int ii = 0; ii < num_sample; ii++){
      double weight;
      if(dist.at(ii)>0){
        weight = mean_dist_beyond/(mean_dist_beyond+dist.at(ii));
      }else{
        weight = 1.0;
      }
      wls.add_mJ(error_buff.at(ii),Jacob.at(ii),weight);
    }
    wls.compute();
    result = SE3<>::exp(wls.get_mu())*result;
  }

  return result;
}

TooN::SE3<> compute_pose_pt2pl_Bi_beam(const CVD::Image<double>& Q1, const CVD::Image<double>& Q2, TooN::SE3<>& guess, vector<Matrix<> > preprocessor_mat, double u_image[][640], double v_image[][640], const int num_samples){
  //	int iter = 6;
  //	SE3<> result;
  //	for (int ii = 0; ii<iter; ii++){
  //		if (ii%2==0){
  //			result = result.inverse();
  //			result = compute_pose_pt2pl_beam(Q1, Q2, result, preprocessor_mat, u_image, v_image, 200);
  //		}else{
  //			result = result.inverse();
  //			result = compute_pose_pt2pl_beam(Q2, Q1, result, preprocessor_mat, u_image, v_image, 200);
  //		}
  //	}
  //	return result;
  const int scan_size=3; // how much of image 2 to search for the closest point

  ImageRef size = Q1.size();
  int max_X = size.x;
  int max_Y = size.y;
  int num_iter = 5;

  SE3<> result=guess;
  for(int it=0; it<num_iter; it++){
    if(it%2==0){
      result = result.inverse();

      WLS<6> wls;
      vector<UVQ > samples;
      vector<UVQ > corr;
      vector<double> error_buff;
      vector<Vector<6> > Jacob;
      vector<double> dist;
      //		double sum_dist_occlusion = 0;
      double sum_dist_beyond = 0;
      int count_occlusion = 0;
      int count_beyond =0;
      for(int sample = 0; sample < num_samples; sample++){
        // get a random sample
        ImageRef p1;
        Vector<6> J;
        do{
          p1.x = rand()%max_X;
          p1.y = rand()%max_Y;
        }while(Q1[p1]<0 || Q1[p1]==0);


        UVQ uvq1=get_uvq(Q1,p1,u_image,v_image);

        Matrix<4,1> temp_buffer;

        temp_buffer(0,0) = uvq1.u();
        temp_buffer(1,0) = uvq1.v();
        temp_buffer(2,0) = 1;
        temp_buffer(3,0) = uvq1.q();

        temp_buffer = result * temp_buffer;

        double temp = temp_buffer(2,0);
        temp_buffer = temp_buffer*(1.0/temp);

	//			UVQ uvq1p = guess * uvq1; // transform according to current guess
        UVQ uvq1p;
        uvq1p.u() = temp_buffer(0,0);
        uvq1p.v() = temp_buffer(1,0);
        uvq1p.q() = temp_buffer(3,0);

        // find pos in im2
        TooN::Vector<2> v;

        v[0] = uvq1p.u()*600+320;
        v[1] = uvq1p.v()*600+240;

        ImageRef p2 = ir_rounded(v);

        uvq1p.X() = p2.x;
        uvq1p.Y() = p2.y;

        if(Q2.in_image(p2) && Q2[p2]>0 ){

          // find correspondence in im 2
          UVQ uvq2 = get_closest_point(Q2, uvq1p, p2, scan_size, u_image,v_image);

          samples.push_back(uvq1p);
          corr.push_back(uvq2);

          Vector<4> uvq2_temp,uvq1p_temp;
          uvq2_temp[0]=uvq2.u();
          uvq2_temp[1]=uvq2.v();
          uvq2_temp[2]=1;
          uvq2_temp[3]=uvq2.q();
          uvq1p_temp[0]=uvq1p.u();
          uvq1p_temp[1]=uvq1p.v();
          uvq1p_temp[2]=1;
          uvq1p_temp[3]=uvq1p.q();

          // get the error between correspondence and transformed im 1 point
          if (uvq1p_temp[3] != uvq2_temp[3]){
            if (uvq1p_temp[3] < uvq2_temp[3]){
              uvq2_temp *= (uvq1p_temp[3]/uvq2_temp[3]);
            }
            else{
              uvq1p_temp *= (uvq2_temp[3]/uvq1p_temp[3]);
            }
          }

          Vector<4> diffbuffer;
          diffbuffer = uvq1p_temp - uvq2_temp;
          diffbuffer[3] = uvq1p_temp[3];

          Matrix<4,1> diff;
          diff[0][0]=diffbuffer[0];
          diff[1][0]=diffbuffer[1];
          diff[2][0]=diffbuffer[2];
          diff[3][0]=diffbuffer[3];

          ImageRef p_cor;
          p_cor.x = uvq2.X();
          p_cor.y = uvq2.Y();

          Matrix<9,1> Q;
          int count_Q = 0;
          for (int ii = 0; ii < 3; ii++){
            for (int jj = 0; jj <3; jj++){
              if (Q2[p_cor.y+ii-1][p_cor.x+jj-1]>0){
                Q[count_Q][0] = Q2[p_cor.y+ii-1][p_cor.x+jj-1];
              }
              else{
                Q[count_Q][0] = Q2[p_cor.y][p_cor.x];
              }
              count_Q++;
            }
          }

          Matrix<3,9> L_mat;
          L_mat = preprocessor_mat.at(p_cor.x + p_cor.y*640);

          Matrix<3,1> normal_buffer = L_mat*Q;

          Vector<3> normal3;
          normal3[0]=normal_buffer[0][0];
          normal3[1]=normal_buffer[1][0];
          normal3[2]=normal_buffer[2][0];
          TooN::normalize(normal3);

          Matrix<1,4> normal;
          normal[0][0]=normal3[0];
          normal[0][1]=normal3[1];
          normal[0][2]=normal3[2];
          normal[0][3]=0.0;

          double errors;
          errors = -(normal*diff)(0,0);
          error_buff.push_back(errors);

          J[0] = uvq1p.q()*normal[0][0];
          J[1] = uvq1p.q()*normal[0][1];
          J[2] = uvq1p.q()*normal[0][2];

          J[3] = uvq1p.v()*normal[0][2] - normal[0][1];
          J[4] = normal[0][0] - uvq1p.u()*normal[0][2];
          J[5] = uvq1p.u()*normal[0][1] - uvq1p.v()*normal[0][0];

          Jacob.push_back(J);

          double distance_sub;
          distance_sub = 1.0/uvq1p.q() - 1.0/uvq2.q();
          dist.push_back(distance_sub);

          if(distance_sub>0){
            sum_dist_beyond += distance_sub;
            count_beyond++;
          }else{
            //				sum_dist_occlusion += distance_sub;
            count_occlusion++;
          }
        }
      }
      double mean_dist_beyond = sum_dist_beyond/count_beyond;
      //		double mean_dist_occlusion = sum_dist_occlusion/count_occlusion;
      int num_sample = count_beyond + count_occlusion;

      for (int ii = 0; ii < num_sample; ii++){
        double weight;
        if(dist.at(ii)>0){
          weight = mean_dist_beyond/(mean_dist_beyond+dist.at(ii));
        }else{
          weight = 1.0;
        }
        wls.add_mJ(error_buff.at(ii),Jacob.at(ii),weight);
      }
      wls.compute();
      result = SE3<>::exp(wls.get_mu())*result;

    }else{
      result = result.inverse();
      WLS<6> wls;
      vector<UVQ > samples;
      vector<UVQ > corr;
      vector<double> error_buff;
      vector<Vector<6> > Jacob;
      vector<double> dist;
      //		double sum_dist_occlusion = 0;
      double sum_dist_beyond = 0;
      int count_occlusion = 0;
      int count_beyond =0;
      for(int sample = 0; sample < num_samples; sample++){
        // get a random sample
        ImageRef p1;
        Vector<6> J;
        do{
          p1.x = rand()%max_X;
          p1.y = rand()%max_Y;
        }while(Q2[p1]<0 || Q2[p1]==0);


        UVQ uvq1=get_uvq(Q2,p1,u_image,v_image);

        Matrix<4,1> temp_buffer;

        temp_buffer(0,0) = uvq1.u();
        temp_buffer(1,0) = uvq1.v();
        temp_buffer(2,0) = 1;
        temp_buffer(3,0) = uvq1.q();

        temp_buffer = result * temp_buffer;

        double temp = temp_buffer(2,0);
        temp_buffer = temp_buffer*(1.0/temp);

	//			UVQ uvq1p = guess * uvq1; // transform according to current guess
        UVQ uvq1p;
        uvq1p.u() = temp_buffer(0,0);
        uvq1p.v() = temp_buffer(1,0);
        uvq1p.q() = temp_buffer(3,0);

        // find pos in im2
        TooN::Vector<2> v;

        v[0] = uvq1p.u()*600+320;
        v[1] = uvq1p.v()*600+240;

        ImageRef p2 = ir_rounded(v);

        uvq1p.X() = p2.x;
        uvq1p.Y() = p2.y;

        if(Q1.in_image(p2) && Q1[p2]>0 ){

          // find correspondence in im 2
          UVQ uvq2 = get_closest_point(Q1, uvq1p, p2, scan_size, u_image,v_image);

          samples.push_back(uvq1p);
          corr.push_back(uvq2);

          Vector<4> uvq2_temp,uvq1p_temp;
          uvq2_temp[0]=uvq2.u();
          uvq2_temp[1]=uvq2.v();
          uvq2_temp[2]=1;
          uvq2_temp[3]=uvq2.q();
          uvq1p_temp[0]=uvq1p.u();
          uvq1p_temp[1]=uvq1p.v();
          uvq1p_temp[2]=1;
          uvq1p_temp[3]=uvq1p.q();

          // get the error between correspondence and transformed im 1 point
          if (uvq1p_temp[3] != uvq2_temp[3]){
            if (uvq1p_temp[3] < uvq2_temp[3]){
              uvq2_temp *= (uvq1p_temp[3]/uvq2_temp[3]);
            }
            else{
              uvq1p_temp *= (uvq2_temp[3]/uvq1p_temp[3]);
            }
          }

          Vector<4> diffbuffer;
          diffbuffer = uvq1p_temp - uvq2_temp;
          diffbuffer[3] = uvq1p_temp[3];

          Matrix<4,1> diff;
          diff[0][0]=diffbuffer[0];
          diff[1][0]=diffbuffer[1];
          diff[2][0]=diffbuffer[2];
          diff[3][0]=diffbuffer[3];

          ImageRef p_cor;
          p_cor.x = uvq2.X();
          p_cor.y = uvq2.Y();

          Matrix<9,1> Q;
          int count_Q = 0;
          for (int ii = 0; ii < 3; ii++){
            for (int jj = 0; jj <3; jj++){
              if (Q1[p_cor.y+ii-1][p_cor.x+jj-1]>0){
                Q[count_Q][0] = Q1[p_cor.y+ii-1][p_cor.x+jj-1];
              }
              else{
                Q[count_Q][0] = Q1[p_cor.y][p_cor.x];
              }
              count_Q++;
            }
          }

          Matrix<3,9> L_mat;
          L_mat = preprocessor_mat.at(p_cor.x + p_cor.y*640);

          Matrix<3,1> normal_buffer = L_mat*Q;

          Vector<3> normal3;
          normal3[0]=normal_buffer[0][0];
          normal3[1]=normal_buffer[1][0];
          normal3[2]=normal_buffer[2][0];
          TooN::normalize(normal3);

          Matrix<1,4> normal;
          normal[0][0]=normal3[0];
          normal[0][1]=normal3[1];
          normal[0][2]=normal3[2];
          normal[0][3]=0.0;

          double errors;
          errors = -(normal*diff)(0,0);
          error_buff.push_back(errors);

          J[0] = uvq1p.q()*normal[0][0];
          J[1] = uvq1p.q()*normal[0][1];
          J[2] = uvq1p.q()*normal[0][2];

          J[3] = uvq1p.v()*normal[0][2] - normal[0][1];
          J[4] = normal[0][0] - uvq1p.u()*normal[0][2];
          J[5] = uvq1p.u()*normal[0][1] - uvq1p.v()*normal[0][0];

          Jacob.push_back(J);

          double distance_sub;
          distance_sub = 1.0/uvq1p.q() - 1.0/uvq2.q();
          dist.push_back(distance_sub);

          if(distance_sub>0){
            sum_dist_beyond += distance_sub;
            count_beyond++;
          }else{
            //				sum_dist_occlusion += distance_sub;
            count_occlusion++;
          }
        }
      }
      double mean_dist_beyond = sum_dist_beyond/count_beyond;
      //		double mean_dist_occlusion = sum_dist_occlusion/count_occlusion;
      int num_sample = count_beyond + count_occlusion;

      for (int ii = 0; ii < num_sample; ii++){
        double weight;
        if(dist.at(ii)>0){
          weight = mean_dist_beyond/(mean_dist_beyond+dist.at(ii));
        }else{
          weight = 1.0;
        }
        wls.add_mJ(error_buff.at(ii),Jacob.at(ii),weight);
      }
      wls.compute();
      result = SE3<>::exp(wls.get_mu())*result;
    }
  }
  if (num_iter%2==0){
    result = result.inverse();
  }

  return result;
}

TooN::SE3<> compute_pose_pt2pl_Bibeam(const CVD::Image<double>& Q1, const CVD::Image<double>& Q2, TooN::SE3<>& guess, vector<Matrix<> > preprocessor_mat, double u_image[][640], double v_image[][640], const int num_samples){

  const int scan_size=3; // how much of image 2 to search for the closest point

  ImageRef size = Q1.size();
  int max_X = size.x;
  int max_Y = size.y;


  SE3<> result=guess;

  for(int it=0; it<5; it++){
    WLS<6> wls;
    vector<UVQ > samples;
    vector<UVQ > corr;
    vector<double> error_buff;
    vector<Vector<6> > Jacob;
    vector<double> dist;
    //		double sum_dist_occlusion = 0;
    double sum_dist_beyond = 0;
    int count_occlusion = 0;
    int count_beyond =0;
    int first_half = num_samples/2;
    for(int sample = 0; sample < first_half; sample++){
      // get a random sample
      ImageRef p1;
      Vector<6> J;
      do{
        p1.x = rand()%max_X;
        p1.y = rand()%max_Y;
      }while(Q1[p1]<0 || Q1[p1]==0);


      UVQ uvq1=get_uvq(Q1,p1,u_image,v_image);

      Matrix<4,1> temp_buffer;

      temp_buffer(0,0) = uvq1.u();
      temp_buffer(1,0) = uvq1.v();
      temp_buffer(2,0) = 1;
      temp_buffer(3,0) = uvq1.q();

      temp_buffer = result * temp_buffer;

      double temp = temp_buffer(2,0);
      temp_buffer = temp_buffer*(1.0/temp);

      //			UVQ uvq1p = guess * uvq1; // transform according to current guess
      UVQ uvq1p;
      uvq1p.u() = temp_buffer(0,0);
      uvq1p.v() = temp_buffer(1,0);
      uvq1p.q() = temp_buffer(3,0);

      // find pos in im2
      TooN::Vector<2> v;

      v[0] = uvq1p.u()*600+320;
      v[1] = uvq1p.v()*600+240;

      ImageRef p2 = ir_rounded(v);

      uvq1p.X() = p2.x;
      uvq1p.Y() = p2.y;

      if(Q2.in_image(p2) && Q2[p2]>0 ){

        // find correspondence in im 2
        UVQ uvq2 = get_closest_point(Q2, uvq1p, p2, scan_size, u_image,v_image);

        samples.push_back(uvq1p);
        corr.push_back(uvq2);

        Vector<4> uvq2_temp,uvq1p_temp;
        uvq2_temp[0]=uvq2.u();
        uvq2_temp[1]=uvq2.v();
        uvq2_temp[2]=1;
        uvq2_temp[3]=uvq2.q();
        uvq1p_temp[0]=uvq1p.u();
        uvq1p_temp[1]=uvq1p.v();
        uvq1p_temp[2]=1;
        uvq1p_temp[3]=uvq1p.q();

        // get the error between correspondence and transformed im 1 point
        if (uvq1p_temp[3] != uvq2_temp[3]){
          if (uvq1p_temp[3] < uvq2_temp[3]){
            uvq2_temp *= (uvq1p_temp[3]/uvq2_temp[3]);
          }
          else{
            uvq1p_temp *= (uvq2_temp[3]/uvq1p_temp[3]);
          }
        }

        Vector<4> diffbuffer;
        diffbuffer = uvq1p_temp - uvq2_temp;
        diffbuffer[3] = uvq1p_temp[3];

        Matrix<4,1> diff;
        diff[0][0]=diffbuffer[0];
        diff[1][0]=diffbuffer[1];
        diff[2][0]=diffbuffer[2];
        diff[3][0]=diffbuffer[3];

        ImageRef p_cor;
        p_cor.x = uvq2.X();
        p_cor.y = uvq2.Y();

        Matrix<9,1> Q;
        int count_Q = 0;
        for (int ii = 0; ii < 3; ii++){
          for (int jj = 0; jj <3; jj++){
            if (Q2[p_cor.y+ii-1][p_cor.x+jj-1]>0){
              Q[count_Q][0] = Q2[p_cor.y+ii-1][p_cor.x+jj-1];
            }
            else{
              Q[count_Q][0] = Q2[p_cor.y][p_cor.x];
            }
            count_Q++;
          }
        }

        Matrix<3,9> L_mat;
        L_mat = preprocessor_mat.at(p_cor.x + p_cor.y*640);

        Matrix<3,1> normal_buffer = L_mat*Q;

        Vector<3> normal3;
        normal3[0]=normal_buffer[0][0];
        normal3[1]=normal_buffer[1][0];
        normal3[2]=normal_buffer[2][0];
        TooN::normalize(normal3);

        Matrix<1,4> normal;
        normal[0][0]=normal3[0];
        normal[0][1]=normal3[1];
        normal[0][2]=normal3[2];
        normal[0][3]=0.0;

        double errors;
        errors = -(normal*diff)(0,0);
        error_buff.push_back(errors);

        J[0] = uvq1p.q()*normal[0][0];
        J[1] = uvq1p.q()*normal[0][1];
        J[2] = uvq1p.q()*normal[0][2];

        J[3] = uvq1p.v()*normal[0][2] - normal[0][1];
        J[4] = normal[0][0] - uvq1p.u()*normal[0][2];
        J[5] = uvq1p.u()*normal[0][1] - uvq1p.v()*normal[0][0];

        Jacob.push_back(J);

        double distance_sub;
        distance_sub = 1.0/uvq1p.q() - 1.0/uvq2.q();
        dist.push_back(distance_sub);

        if(distance_sub>0){
          sum_dist_beyond += distance_sub;
          count_beyond++;
        }else{
          //				sum_dist_occlusion += distance_sub;
          count_occlusion++;
        }
      }
    }

    for(int sample = first_half; sample < num_samples; sample++){
      // get a random sample
      ImageRef p1;
      Vector<6> J;
      do{
        p1.x = rand()%max_X;
        p1.y = rand()%max_Y;
      }while(Q2[p1]<0 || Q2[p1]==0);


      UVQ uvq1=get_uvq(Q2,p1,u_image,v_image);

      Matrix<4,1> temp_buffer;

      temp_buffer(0,0) = uvq1.u();
      temp_buffer(1,0) = uvq1.v();
      temp_buffer(2,0) = 1;
      temp_buffer(3,0) = uvq1.q();

      temp_buffer = result.inverse() * temp_buffer;

      double temp = temp_buffer(2,0);
      temp_buffer = temp_buffer*(1.0/temp);

      //			UVQ uvq1p = guess * uvq1; // transform according to current guess
      UVQ uvq1p;
      uvq1p.u() = temp_buffer(0,0);
      uvq1p.v() = temp_buffer(1,0);
      uvq1p.q() = temp_buffer(3,0);

      // find pos in im2
      TooN::Vector<2> v;

      v[0] = uvq1p.u()*600+320;
      v[1] = uvq1p.v()*600+240;

      ImageRef p2 = ir_rounded(v);

      uvq1p.X() = p2.x;
      uvq1p.Y() = p2.y;

      if(Q1.in_image(p2) && Q1[p2]>0 ){

        // find correspondence in im 1
        UVQ uvq2 = get_closest_point(Q1, uvq1p, p2, scan_size, u_image,v_image);

        samples.push_back(uvq2);
        corr.push_back(uvq1p);

        Vector<4> uvq2_temp,uvq1p_temp;
        uvq2_temp[0]=uvq2.u();
        uvq2_temp[1]=uvq2.v();
        uvq2_temp[2]=1;
        uvq2_temp[3]=uvq2.q();
        uvq1p_temp[0]=uvq1p.u();
        uvq1p_temp[1]=uvq1p.v();
        uvq1p_temp[2]=1;
        uvq1p_temp[3]=uvq1p.q();

        // get the error between correspondence and transformed im 1 point
        if (uvq1p_temp[3] != uvq2_temp[3]){
          if (uvq1p_temp[3] < uvq2_temp[3]){
            uvq2_temp *= (uvq1p_temp[3]/uvq2_temp[3]);
          }
          else{
            uvq1p_temp *= (uvq2_temp[3]/uvq1p_temp[3]);
          }
        }

        Vector<4> diffbuffer;
        diffbuffer = uvq2_temp - uvq1p_temp;
        diffbuffer[3] = uvq2_temp[3];

        Matrix<4,1> diff;
        diff[0][0]=diffbuffer[0];
        diff[1][0]=diffbuffer[1];
        diff[2][0]=diffbuffer[2];
        diff[3][0]=diffbuffer[3];

        ImageRef p_cor;
        p_cor.x = uvq2.X();
        p_cor.y = uvq2.Y();

        Matrix<9,1> Q;
        int count_Q = 0;
        for (int ii = 0; ii < 3; ii++){
          for (int jj = 0; jj <3; jj++){
            if (Q2[p2.y+ii-1][p2.x+jj-1]>0){
              Q[count_Q][0] = Q2[p2.y+ii-1][p2.x+jj-1];
            }
            else{
              Q[count_Q][0] = Q2[p2.y][p2.x];
            }
            count_Q++;
          }
        }

        Matrix<3,9> L_mat;
        L_mat = preprocessor_mat.at(p2.x + p2.y*640);

        Matrix<3,1> normal_buffer = L_mat*Q;

        Vector<3> normal3;
        normal3[0]=normal_buffer[0][0];
        normal3[1]=normal_buffer[1][0];
        normal3[2]=normal_buffer[2][0];
        TooN::normalize(normal3);

        Matrix<1,4> normal;
        normal[0][0]=normal3[0];
        normal[0][1]=normal3[1];
        normal[0][2]=normal3[2];
        normal[0][3]=0.0;

        double errors;
        errors = -(normal*diff)(0,0);
        error_buff.push_back(errors);

        J[0] = uvq2.q()*normal[0][0];
        J[1] = uvq2.q()*normal[0][1];
        J[2] = uvq2.q()*normal[0][2];

        J[3] = uvq2.v()*normal[0][2] - normal[0][1];
        J[4] = normal[0][0] - uvq2.u()*normal[0][2];
        J[5] = uvq2.u()*normal[0][1] - uvq2.v()*normal[0][0];

        Jacob.push_back(J);

        double distance_sub;
        distance_sub = 1.0/uvq2.q() - 1.0/uvq1p.q();
        dist.push_back(distance_sub);

        if(distance_sub>0){
          sum_dist_beyond += distance_sub;
          count_beyond++;
        }else{
          //				sum_dist_occlusion += distance_sub;
          count_occlusion++;
        }
      }
    }



    double mean_dist_beyond = sum_dist_beyond/count_beyond;
    //		double mean_dist_occlusion = sum_dist_occlusion/count_occlusion;
    int num_sample = count_beyond + count_occlusion;

    for (int ii = 0; ii < num_sample; ii++){
      double weight;
      if(dist.at(ii)>0){
        weight = mean_dist_beyond/(mean_dist_beyond+dist.at(ii));
      }else{
        weight = 1.0;
      }
      wls.add_mJ(error_buff.at(ii),Jacob.at(ii),weight);
    }
    wls.compute();
    result = SE3<>::exp(wls.get_mu())*result;
  }

  return result;
}

#endif


