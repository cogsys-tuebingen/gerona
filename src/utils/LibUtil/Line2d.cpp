/**
 (c) Lehrstuhl RA Universitaet Tuebingen

 @author: Marks
 @date 2011

 @file Line2d.cpp
 */

///////////////////////////////////////////////////////////////////////////////
// I N C L U D E S
///////////////////////////////////////////////////////////////////////////////

// C/C++
#include <iostream>
#include <cmath>
#include <Eigen/LU>
// Project
#include "Line2d.h"
#include "MathHelper.h"

using namespace Eigen;
using namespace std;

///////////////////////////////////////////////////////////////////////////////
// I M P L E M E N T A T I O N
///////////////////////////////////////////////////////////////////////////////

Line2d::Line2d() {
    mDirection.setZero();
    mOrigin.setZero();
}

Line2d::Line2d( const Eigen::Vector2d &p1, const Eigen::Vector2d &p2 ) {
    if ( !Trough( p1, p2 )) {
        mDirection.setZero();
        mOrigin.setZero();
    } // TODO Error handling
}

Line2d::Line2d( const Vector2d &origin, const double angle ) {
	FromAngle( origin, angle );
}

double Line2d::GetSignedDistance(const Eigen::Vector2d &p) const
{
  /* see http://mathworld.wolfram.com/Point-LineDistance2-Dimensional.html
    with line=(p1,p2) point p0
    distance to line is
    d= |det(p2-p1 p1-p0)|/ |p2-p1]
    here [p2-p1| = |mDirection| = 1
    */

  Matrix2d M;
  M.col(0)=mDirection;
  M.col(1)=mOrigin-p;
  return -1.0*M.determinant();
}

double Line2d::GetDistance( const Eigen::Vector2d &p ) const {
	return fabs( (-mDirection[1]*p[0] + mDirection[0]*p[1]
	                 + mDirection[1]*mOrigin[0] - mDirection[0]*mOrigin[1] ));
}

double Line2d::GetRelativeLocation( const Eigen::Vector2d &p ) const {
    return mDirection(0) * (p(1) - mOrigin(1) - mDirection(1))
                 - (p(0) - mOrigin(0) - mDirection(0)) * mDirection(1);
}

bool Line2d::Trough( const Eigen::Vector2d &p1, const Eigen::Vector2d &p2 ) {
    if ((p2 - p1).norm() > 0 ) {
        mOrigin = p1;
        mDirection = (p2 - p1).normalized();
        return true;
    } else
        return false;
}

void Line2d::FromPolynomial( double a, double b ) {
    mOrigin(0) = 0;
    mOrigin(1) = b;

    mDirection(0) = 1;
    mDirection(1) = a;
    mDirection.normalize();
}

void Line2d::FromAngle( const Vector2d &origin, const double angle ) {
	mOrigin = origin;
	mDirection << cos( angle ), sin( angle );
}

double Line2d::GetAngle() const {
    return atan2(mDirection.y(),mDirection.x());
}

double Line2d::GetAngle( const Line2d &line ) const {
    double angle = acos( mDirection.dot( line.mDirection ));
    if ( angle >= M_PI )
        angle -= 2.0*M_PI;
    else if ( angle < -M_PI )
        angle += 2.0*M_PI;
    return angle;
}

bool Line2d::SetDirection( const Eigen::Vector2d &dir ) {
    if ( dir.norm() < 1E-9 ) {
        std::cerr << "Line2d: ERROR. Got invalid direction vector" << std::endl;
        return false;
    }

    mDirection = dir;
    mDirection.normalize();
    return true;
}

bool Line2d::SetDirection( const double x, const double y ) {
	if ( x == 0 && y == 0 )
		return false;

	mDirection << x, y;
	mDirection.normalize();
	return true;
}

void Line2d::InvertDirection() {
	mDirection = -mDirection;
}

double Line2d::GetProjection( const Vector2d &vec ) const {
	return vec.dot( mDirection ); // Direction has length 1
}

void Line2d::ParallelShift( const double &dist ) {
	mOrigin[0] += dist*mDirection[1];
	mOrigin[1] += -dist*mDirection[0];
}

Vector2d Line2d::GetOrthoVector() const {
	return Vector2d( mDirection[1], mDirection[0] );
}
