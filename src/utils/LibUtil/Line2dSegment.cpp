/**
 (c) Lehrstuhl RA Universitaet Tuebingen

 @author: Marks
 @date 2011

 @file Line2dSegment.cpp
 */

///////////////////////////////////////////////////////////////////////////////
// I N C L U D E S
///////////////////////////////////////////////////////////////////////////////

// Project
#include "MathHelper.h"
#include "Line2dSegment.h"

///////////////////////////////////////////////////////////////////////////////
// I M P L E M E N T A T I O N
///////////////////////////////////////////////////////////////////////////////

using namespace Eigen;

Line2dSegment::Line2dSegment() {
	mLength = 0;
}

bool Line2dSegment::SetStartEnd( const Vector2d &start, const Vector2d &end ) {
	mDirection = end - start;
	if ( mDirection.norm() == 0 )
		return false;
	mLength = mDirection.norm();
	mDirection.normalize();
	mOrigin = start;
	return true;
}

double Line2dSegment::GetMidDistance( const Line2d &line ) {
	Vector2d midP( 0.5*(mOrigin + GetEnd()));
	double dist = line.GetDistance( midP );
	if ( line.GetRelativeLocation( midP ) < 0 )
		dist = -dist;
	return dist;
}

double Line2dSegment::GetMinDistance( const Vector2d &p ) const {
	double startAngle = fabs( MathHelper::Angle( mDirection, mOrigin - p ));
	double endAngle = fabs( MathHelper::Angle( -mDirection, p - GetEnd()));

	if ( startAngle <= 0.5*M_PI && endAngle <= 0.5*M_PI )
		return GetDistance( p );

	if ( startAngle > endAngle )
		return (mOrigin - p).norm();

	return (GetEnd() - p).norm();
}

void Line2dSegment::GetProjectedDistance(
		const Line2d &line,
		double &start,
		double &end ) const {

	start = line.GetProjection( mOrigin );
	end = line.GetProjection( GetEnd());
}

void Line2dSegment::InvertDirection() {
	mOrigin = GetEnd();
	mDirection = -mDirection;
}
