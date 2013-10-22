/*
 * uvq.h
 *
 *  Created on: 5 Oct 2012
 *      Author: max
 */

#ifndef UVQ_H
#define UVQ_H

#include <TooN/TooN.h>
#include <TooN/se3.h>
#include <iostream>

class UVQ {
public:
	TooN::Vector<3> uvq;
	CVD::ImageRef XY;
	double& u() {return uvq[0];}
	double& v() {return uvq[1];}
	double& q() {return uvq[2];}
	int& X() {return XY.x;}
	int& Y() {return XY.y;}
	const double& u() const {return uvq[0];}
	const double& v() const {return uvq[1];}
	const double& q() const {return uvq[2];}
	const int& X() const {return XY.x;}
	const int& Y() const {return XY.y;}

	UVQ operator-(const UVQ& rhs){
		UVQ result;
		result.uvq = uvq-rhs.uvq;
		return result;
	}

	// this works for inverse metres from Kinect
	// because errors in u, v and q are all about 0.002
	double magnitude_sq(){return uvq*uvq;}

};

inline UVQ operator*(const TooN::SE3<>& E, const UVQ& rhs){
	using namespace TooN;
	UVQ result;
	Vector<2> uv = rhs.uvq.slice<0,2>();
	Vector<3> uv1 = unproject(uv);
	result.uvq = E.get_rotation()*uv1 + rhs.q()*E.get_translation();
	double qfac = 1.0 / result.uvq[2];
	result.uvq[0]*=qfac;
	result.uvq[1]*=qfac;
	result.uvq[2]=rhs.q()*qfac;
	return result;
}


// this is the Jacobian of derivatives of u v and q
// when left multiplied by SE3::exp(Vector<6>)
inline TooN::Matrix<3,6> get_J(const UVQ& uvq){
	using namespace TooN;
	const double u = uvq.u();
	const double v = uvq.v();
	const double q = uvq.q();
	Matrix <3,6> J;
	J[0] = makeVector(q, 0, -u*q, -u*v,   1+u*u, -v);
	J[1] = makeVector(0, q, -v*q, -1-v*v, u*v,    u);
	J[2] = makeVector(0, 0, -q*q, -v*q,   u*q,    0);
	return J;
}

inline std::ostream& operator << (std::ostream& os, const UVQ& uvq){
  return os << uvq.uvq;
}


#endif
