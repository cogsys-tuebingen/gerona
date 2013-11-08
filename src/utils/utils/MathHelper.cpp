#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include "MathHelper.h"



// C/C++
#include <cmath>

///////////////////////////////////////////////////////////////////////////////
// I M P L E M E  N T A T I O N
///////////////////////////////////////////////////////////////////////////////

using namespace Eigen;

Vector3d MathHelper::OdometryDelta(
    const Eigen::Vector3d &old,
    const Eigen::Vector3d &latest ) {
  Vector3d odomDelta;
  double c = cos( old[2] );
  double s = sin( old[2] );
  odomDelta[0] = c*(latest[0] - old[0]) + s*(latest[1] - old[1]);
  odomDelta[1] = -s*(latest[0] - old[0]) + c*(latest[1] - old[1]);
  odomDelta[2] = AngleDelta( old[2], latest[2] );
  return odomDelta;
}

double MathHelper::OdomDistance( const Eigen::Vector3d &p1, const Eigen::Vector3d &p2 ) {
  return sqrt( pow( p1[0] - p2[0], 2 ) + pow( p1[1] - p2[1], 2 ));
}

double MathHelper::NormalizeAngle(const double angle) {
  double ret = angle;
  while (ret > M_PI)
    ret -= 2.0 * M_PI;
  while (ret < -M_PI)
    ret += 2.0 * M_PI;
  return ret;
}

double MathHelper::AngleDelta(const double a1, const double a2) {
  double ret = a2 - a1;
  while (ret > M_PI)
    ret -= 2.0 * M_PI;
  while (ret < -M_PI)
    ret += 2.0 * M_PI;
  return ret;
}

double MathHelper::Angle( const Vector2d &v1, const Vector2d &v2 ) {
  return acos( v1.dot( v2 ) / ( v1.norm() * v2.norm()));
}

double MathHelper::Angle( const Vector2d &v ) {
  if ( v[0] > 0 )
    return atan( v[1] / v[0]);
  if ( v[0] < 0 && v[1] >= 0 )
    return atan( v[1] / v[0]) + M_PI;
  if ( v[0] < 0 && v[1] < 0 )
    return atan( v[1] / v[0]) - M_PI;
  if ( v[0] < 1E-6 && v[1] > 0 )
    return 0.5*M_PI;
  if ( v[0] < 1E-6 && v[1] < 0 )
    return -0.5*M_PI;

  return 0.0;
}

double MathHelper::AngleClamp( double angleRad ) {
    while (angleRad < -M_PI)
        angleRad += 2*M_PI;

    while (angleRad >= M_PI)
        angleRad -= 2*M_PI;

    return angleRad;
}



void Quaternion2Ypr(const FVector& q, FVector& ypr)
{
    float sqx, sqy, sqz, sqw;

    sqx = q[0]*q[0];
    sqy = q[1]*q[1];
    sqz = q[2]*q[2];
    sqw = q[3]*q[3];
    if (ypr.size()<3) ypr.resize(3);
    ypr[0] = atan2(2.0 * (q[0]*q[1] + q[2]*q[3]),(sqx - sqy - sqz + sqw));
//    ypr[0] = atan2(2.0 * (q[0]*q[1] + q[2]*q[3]),1-2*(sqy+sqz));
    ypr[1] = asin(-2.0 * (q[0]*q[2] - q[1]*q[3]));

    ypr[2] = atan2(2.0 * (q[1]*q[2] + q[0]*q[3]),1-2*(sqx+sqy));

}




// 1D vec /////////////////////////////////////////////////////////////////////////////////////////

double min1D(const vecD& vec)
{
	if (vec.size() == 0) return 0.0;

	double min = std::numeric_limits<double>::infinity();

	for (unsigned i=0; i!=vec.size(); ++i) {
		if (vec[i] < min) {
			min = vec[i];
		}
	}

	return min;
}

double max1D(const vecD& vec)
{
	if (vec.size() == 0) return 0.0;

	double max = -std::numeric_limits<double>::infinity();

	for (unsigned i=0; i!=vec.size(); ++i) {
		if (vec[i] > max) {
			max = vec[i];
		}
	}

	return max;
}

double range1D(const vecD& vec)
{
	if (vec.size() == 0) return 0.0;

	return max1D(vec) - min1D(vec);
}

double mean1D(const vecD& vec)
{
	int size = vec.size();
	if (size == 0) {
		return 0.0;
	}

	double sum = 0.0;
	for (int i=0; i!=size; ++i) {
		sum += vec[i];
	}

	return sum / double(size);
}

double median1D(const vecD& vec)
{
	int size = vec.size();
	if (size == 0) {
		return 0.0;
	}
	
	vecD vecSort = vec;
	std::sort(vecSort.begin(), vecSort.end());
	if (size % 2 == 0) { // size even?
		return 0.5 * ((vecSort[size / 2 - 1]) + (vecSort[size / 2]));
	}
	else { // or odd?
		return vecSort[(size - 1) / 2];
	}
}

double variance1D(const vecD& vec)
{
	int size = vec.size();
	if (size < 2) {
		return 0.0;
	}

	double mean = mean1D(vec);
	
	double sum = 0.0;
	for (int i=0; i!=size; ++i) {
		sum += (vec[i] - mean) * (vec[i] - mean);
	}

	return sum / double(size - 1);
}

double standardDeviation1D(const vecD& vec)
{
	return std::sqrt(variance1D(vec));
}

double histogram1D(const vecD& vec, double begin, double end)
{
	int size = vec.size();
	if (size == 0) {
		return 0.0;
	}

	int n = 0;
	for (int i=0; i!=size; ++i) {
		if (vec[i] >= begin && vec[i] < end) {
			++n;
		}
	}	

	return double(n) / double(size);
}

// 2D vec /////////////////////////////////////////////////////////////////////////////////////////

void centroid2D(const vecD& vec, double& centerX, double& centerY)
{
	unsigned numOfPoints = vec.size() / 2;
	if (numOfPoints == 0 || vec.size() != numOfPoints * 2) {
		centerX = centerY = 0.0;
		return;
	}

	double sumX = 0.0;
	double sumY = 0.0;

	for (unsigned i=0; i<vec.size(); i+=2) {
		sumX += vec[i];
		sumY += vec[i+1];
	}

	centerX = sumX / double(numOfPoints);
	centerY = sumY / double(numOfPoints);
}

double maxDistanceFromCentroid2D(const vecD& vec)
{
	unsigned numOfPoints = vec.size() / 2;
	if (numOfPoints == 0 || vec.size() != numOfPoints * 2) {
		return 0.0;
	}

	double centerX, centerY;
	centroid2D(vec, centerX, centerY);

	double maxDist = 0.0;
	double dist = 0.0;

	for (unsigned i=0; i<vec.size(); i+=2) {
		dist = distance2D(vec[i], vec[i+1], centerX, centerY);
		if (dist > maxDist) {
			maxDist = dist;
		}
	}

	return maxDist;
}

double meanDistanceFromCentroid2D(const vecD& vec)
{
	unsigned numOfPoints = vec.size() / 2;
	if (numOfPoints == 0 || vec.size() != numOfPoints * 2) {
		return 0.0;
	}

	double centerX, centerY;
	centroid2D(vec, centerX, centerY);
	
	double sumDist = 0.0;
	double dist = 0.0;

	for (unsigned i=0; i<vec.size(); i+=2) {
		dist = distance2D(vec[i], vec[i+1], centerX, centerY);
		sumDist += dist;
	}

	return sumDist / double(numOfPoints);
}

double meanRelativeDistanceFromCentroid2D(const vecD& vec)
{
	double maxDist = maxDistanceFromCentroid2D(vec);
	if (maxDist == 0.0) return 0.0;

	return meanDistanceFromCentroid2D(vec) / maxDist;
}

double medianDistanceFromCentroid2D(const vecD& vec)
{
	unsigned numOfPoints = vec.size() / 2;
	if (numOfPoints == 0 || vec.size() != numOfPoints * 2) {
		return 0.0;
	}

	double centerX, centerY;
	centroid2D(vec, centerX, centerY);

	vecD distances;
	for (unsigned i=0; i<vec.size(); i+=2) {
		distances.push_back(distance2D(vec[i], vec[i+1], centerX, centerY));
	}

	return median1D(distances);
}

vecD getRelativeDistances2D(const vecD& vec)
{
	unsigned numOfPoints = vec.size() / 2;
	if (numOfPoints == 0 || vec.size() != numOfPoints * 2) {
		return vecD();
	}

	double maxDist = maxDistanceFromCentroid2D(vec);
	if (maxDist == 0.0) return vecD(numOfPoints, 0.0);

	double centerX, centerY;
	centroid2D(vec, centerX, centerY);
	
	vecD relativeDistances(numOfPoints);

	for (unsigned i=0; i<vec.size(); i+=2) {
		relativeDistances[i/2] = distance2D(vec[i], vec[i+1], centerX, centerY) / maxDist;
	}

	return relativeDistances;
}

double medianRelativeDistanceFromCentroid2D(const vecD& vec)
{
	return median1D(getRelativeDistances2D(vec));
}

double varianceDistanceFromCentroid2D(const vecD& vec)
{
	unsigned numOfPoints = vec.size() / 2;
	if (numOfPoints < 2 || vec.size() != numOfPoints * 2) {
		return 0.0;
	}

	double centerX, centerY;
	centroid2D(vec, centerX, centerY);

	double sum = 0.0;
	for (unsigned i=0; i<vec.size(); i+=2) {
		double dist = distance2D(vec[i], vec[i+1], centerX, centerY);
		sum += dist * dist;
	}

	return sum / double(numOfPoints - 1);
}

double varianceRelativeDistanceFromCentroid2D(const vecD& vec)
{
	unsigned numOfPoints = vec.size() / 2;
	if (numOfPoints < 2 || vec.size() != numOfPoints * 2) {
		return 0.0;
	}

	double maxDist = maxDistanceFromCentroid2D(vec);
	if (maxDist == 0.0) return 0.0;

	double centerX, centerY;
	centroid2D(vec, centerX, centerY);
	
	double sum = 0.0;

	for (unsigned i=0; i<vec.size(); i+=2) {
		double dist = distance2D(vec[i], vec[i+1], centerX, centerY);
		sum += dist * dist;
	}

	return sum / (maxDist * maxDist) / double(numOfPoints - 1);
}

double standardDeviationDistanceFromCentroid2D(const vecD& vec)
{
	return std::sqrt(varianceDistanceFromCentroid2D(vec));
}

double standardDeviationRelativeDistanceFromCentroid2D(const vecD& vec)
{
	return std::sqrt(varianceRelativeDistanceFromCentroid2D(vec));
}

double histogram2D(const vecD& vec, double begin, double end)
{
	unsigned numOfPoints = vec.size() / 2;
	if (numOfPoints == 0 || vec.size() != numOfPoints * 2) {
		return 0.0;
	}

	vecD relativeDistances = getRelativeDistances2D(vec);
	if (relativeDistances.size() == 0) return 0.0;
	int n = 0;

	for (unsigned i=0; i!=relativeDistances.size(); ++i) {
		if (relativeDistances[i] >= begin && relativeDistances[i] < end) {
			++n;
		}
	}

	return double(n) / double(relativeDistances.size());
}

// 2D binary matrix ///////////////////////////////////////////////////////////////////////////////

void centroid2D(const vecB& binaryMatrix, int width, int height, double& centerX, double& centerY)
{
	unsigned sizeOfSquare = width * height;
	if (width == 0 || height == 0 || binaryMatrix.size() != sizeOfSquare) {
		centerX = centerY = 0.0;
		return;
	}

	double sumX = 0.0;
	double sumY = 0.0;
	int numOfPoints = 0;
	int n = 0;

	for (int y=0; y!=height; ++y) {
		for (int x=0; x!=width; ++x) {
			if (!binaryMatrix[n++]) {
				sumX += x;
				sumY += y;
				numOfPoints++;
			}
		}
	}

	if (numOfPoints == 0) {
		centerX = centerY = 0;	
	}
	else {
		centerX = sumX / double(numOfPoints);
		centerY = sumY / double(numOfPoints);
	}
}

double maxDistanceFromCentroid2D(const vecB& binaryMatrix, int width, int height)
{
	unsigned sizeOfSquare = width * height;
	if (width == 0 || height == 0 || binaryMatrix.size() != sizeOfSquare) {
		return 0.0;
	}

	double centerX, centerY;
	centroid2D(binaryMatrix, width, height, centerX, centerY);

	double maxDist = 0.0;
	double dist = 0.0;
	int n = 0;

	for (int y=0; y!=height; ++y) {
		for (int x=0; x!=width; ++x) {
			if (!binaryMatrix[n++]) {
				dist = distance2D(x, y, centerX, centerY);
				if (dist > maxDist) {
					maxDist = dist;
				}
			}
		}
	}

	return maxDist;
}

double meanDistanceFromCentroid2D(const vecB& binaryMatrix, int width, int height)
{
	unsigned sizeOfSquare = width * height;
	if (width == 0 || height == 0 || binaryMatrix.size() != sizeOfSquare) {
		return 0.0;
	}

	double centerX, centerY;
	centroid2D(binaryMatrix, width, height, centerX, centerY);

	double sumDist = 0.0;
	int numOfPoints = 0;
	int n = 0;

	for (int y=0; y!=height; ++y) {
		for (int x=0; x!=width; ++x) {
			if (!binaryMatrix[n++]) {
				sumDist += distance2D(x, y, centerX, centerY);
				numOfPoints++;
			}
		}
	}

	if (numOfPoints == 0) return 0.0;
	return sumDist / double(numOfPoints);
}

double meanRelativeDistanceFromCentroid2D(const vecB& binaryMatrix, int width, int height)
{
	unsigned sizeOfSquare = width * height;
	if (width == 0 || height == 0 || binaryMatrix.size() != sizeOfSquare) {
		return 0.0;
	}

	double maxDist = maxDistanceFromCentroid2D(binaryMatrix, width, height);
	if (maxDist == 0.0) return 0.0;

	double centerX, centerY;
	centroid2D(binaryMatrix, width, height, centerX, centerY);
	
	double sumDist = 0.0;
	int numOfPoints = 0;
	int n = 0;

	for (int y=0; y!=height; ++y) {
		for (int x=0; x!=width; ++x) {
			if (!binaryMatrix[n++]) {
				sumDist += distance2D(x, y, centerX, centerY);
				numOfPoints++;
			}
		}
	}

	if (numOfPoints == 0) return 0.0;
	return sumDist / maxDist / double(numOfPoints);
}

double medianDistanceFromCentroid2D(const vecB& binaryMatrix, int width, int height)
{
	unsigned sizeOfSquare = width * height;
	if (width == 0 || height == 0 || binaryMatrix.size() != sizeOfSquare) {
		return 0.0;
	}

	double centerX, centerY;
	centroid2D(binaryMatrix, width, height, centerX, centerY);

	vecD distances;
	int n = 0;

	for (int y=0; y!=height; ++y) {
		for (int x=0; x!=width; ++x) {
			if (!binaryMatrix[n++]) {
				distances.push_back(distance2D(x, y, centerX, centerY));
			}
		}
	}

	return median1D(distances);
}

vecD getRelativeDistances2D(const vecB& binaryMatrix, int width, int height)
{
	vecD relativeDistances;
	unsigned sizeOfSquare = width * height;
	if (width == 0 || height == 0 || binaryMatrix.size() != sizeOfSquare) {
		return relativeDistances;
	}

	double centerX, centerY;
	centroid2D(binaryMatrix, width, height, centerX, centerY);
	
	double maxDist = maxDistanceFromCentroid2D(binaryMatrix, width, height);
	int n = 0;

	for (int y=0; y!=height; ++y) {
		for (int x=0; x!=width; ++x) {
			if (!binaryMatrix[n++]) {
				if (maxDist != 0.0) {
					relativeDistances.push_back(distance2D(x, y, centerX, centerY) / maxDist);
				}
				else {
					relativeDistances.push_back(0.0);
				}
			}
		}
	}

	return relativeDistances;
}

double medianRelativeDistanceFromCentroid2D(const vecB& binaryMatrix, int width, int height)
{
	return median1D(getRelativeDistances2D(binaryMatrix, width, height));
}

double varianceDistanceFromCentroid2D(const vecB& binaryMatrix, int width, int height)
{
	unsigned sizeOfSquare = width * height;
	if (width == 0 || height == 0 || binaryMatrix.size() != sizeOfSquare) {
		return 0.0;
	}

	double centerX, centerY;
	centroid2D(binaryMatrix, width, height, centerX, centerY);

	double sum = 0.0;
	int numOfPoints = 0;
	int n = 0;

	for (int y=0; y!=height; ++y) {
		for (int x=0; x!=width; ++x) {
			if (!binaryMatrix[n++]) {
				double dist = distance2D(x, y, centerX, centerY);
				sum += dist * dist;
				numOfPoints++;
			}
		}
	}

	if (numOfPoints < 2) return 0.0;
	return sum / double(numOfPoints - 1);
}

double varianceRelativeDistanceFromCentroid2D(const vecB& binaryMatrix, int width, int height)
{
	unsigned sizeOfSquare = width * height;
	if (width == 0 || height == 0 || binaryMatrix.size() != sizeOfSquare) {
		return 0.0;
	}

	double maxDist = maxDistanceFromCentroid2D(binaryMatrix, width, height);
	if (maxDist == 0.0) return 0.0;

	double centerX, centerY;
	centroid2D(binaryMatrix, width, height, centerX, centerY);
	
	double sum = 0.0;
	int numOfPoints = 0;
	int n = 0;

	for (int y=0; y!=height; ++y) {
		for (int x=0; x!=width; ++x) {
			if (!binaryMatrix[n++]) {
				double dist = distance2D(x, y, centerX, centerY);
				sum += dist * dist;
				numOfPoints++;
			}
		}
	}

	if (numOfPoints < 2) return 0.0;
	return sum / (maxDist*maxDist) / double(numOfPoints - 1);
}

double standardDeviationDistanceFromCentroid2D(const vecB& binaryMatrix, int width, int height)
{
	return std::sqrt(varianceDistanceFromCentroid2D(binaryMatrix, width, height));
}

double standardDeviationRelativeDistanceFromCentroid2D(const vecB& binaryMatrix, int width, int height)
{
	return std::sqrt(varianceRelativeDistanceFromCentroid2D(binaryMatrix, width, height));
}

double histogram2D(const vecB& binaryMatrix, int width, int height, double begin, double end)
{
	unsigned sizeOfSquare = width * height;
	if (width == 0 || height == 0 || binaryMatrix.size() != sizeOfSquare) {
		return 0.0;
	}

	vecD relativeDistances = getRelativeDistances2D(binaryMatrix, width, height);
	if (relativeDistances.size() == 0) return 0.0;
	int n = 0;

	for (unsigned i=0; i!=relativeDistances.size(); ++i) {
		if (relativeDistances[i] >= begin && relativeDistances[i] < end) {
			++n;
		}
	}

	return double(n) / double(relativeDistances.size());
}

// 2D misc ////////////////////////////////////////////////////////////////////////////////////////

double angleSmallerDeg2D(double x1, double y1, double x2, double y2, double x3, double y3)
{
	if (x1 == x3 && y1 == y3) return 0.0;

	double a1 = x1 - x2;
	double a2 = y1 - y2;
	double b1 = x3 - x2;
	double b2 = y3 - y2;

	double length1 = length2D(a1, a2);
	double length2 = length2D(b1, b2);
	if (length1 == 0.0 || length2 == 0.0) return 0.0;

	double prod = dotProduct2D(a1, a2, b1, b2) / (length1 * length2);
	if (prod < -1.0) prod = -1.0; else if (prod > 1.0) prod = 1.0;

	return angleDeg(std::acos(prod));
}

double angleSmallerDeg2D(point2D p1, point2D p2, point2D p3)
{
	return angleSmallerDeg2D(p1.first, p1.second, p2.first, p2.second, p3.first, p3.second);
}

double angleLeftDeg2D(double x1, double y1, double x2, double y2, double x3, double y3)
{
	double angle = angleSmallerDeg2D(x1, y1, x2, y2, x3, y3);

	if (angle != 0.0 && angle != 180.0) {
		double loc = location2D(x1, y1, x2, y2, x3, y3);
		if (loc > 0.0) angle = 360.0 - angle;
	}

	return angle;
}

double angleLeftDeg2D(point2D p1, point2D p2, point2D p3)
{
	return angleLeftDeg2D(p1.first, p1.second, p2.first, p2.second, p3.first, p3.second);
}

double angleRightDeg2D(double x1, double y1, double x2, double y2, double x3, double y3)
{
	double angle = angleSmallerDeg2D(x1, y1, x2, y2, x3, y3);

	if (angle != 0.0 && angle != 180.0) {
		double loc = location2D(x1, y1, x2, y2, x3, y3);
		if (loc < 0.0) angle = 360.0 - angle;
	}

	return angle;
}

double angleRightDeg2D(point2D p1, point2D p2, point2D p3)
{
	return angleRightDeg2D(p1.first, p1.second, p2.first, p2.second, p3.first, p3.second);
}

double areaTriangle2D(double x1, double y1, double x2, double y2, double x3, double y3)
{
	double a = distance2D(x1, y1, x2, y2);	
	double b = distance2D(x1, y1, x3, y3);
	double c = distance2D(x2, y2, x3, y3);

	double s = 0.5 * (a + b + c);
	double p = s * (s-a) * (s-b) * (s-c);
	if (p < 0.0) return 0.0; // just in case
	return std::sqrt(p);
}

double areaTriangle2D(point2D p1, point2D p2, point2D p3)
{
	return areaTriangle2D(p1.first, p1.second, p2.first, p2.second, p3.first, p3.second);
}

double distance2D(double x1, double y1, double x2, double y2)
{
	double distX = x2 - x1;
	double distY = y2 - y1;

	return std::sqrt(distX*distX + distY*distY);
}

double dotProduct2D(double x1, double y1, double x2, double y2)
{
	return x1*x2 + y1*y2;
}

double dotProduct2D(point2D p1, point2D p2)
{
	return dotProduct2D(p1.first, p1.second, p2.first, p2.second);
}

double length2D(double x, double y)
{
	return std::sqrt(x*x + y*y);
}

double length2D(point2D p)
{
	return length2D(p.first, p.second);
}

double location2D(double x1, double y1, double x2, double y2, double x3, double y3)
{
	return (x1 - x2) * (y3 - y2) - (x3 - x2) * (y1 - y2);
}

double location2D(point2D p1, point2D p2, point2D p3)
{
	return location2D(p1.first, p1.second, p2.first, p2.second, p3.first, p3.second);
}

void normalizeVector2D(double& x, double& y)
{
	double length = length2D(x, y);
	x /= length;
	y /= length;
}

// 3D vec /////////////////////////////////////////////////////////////////////////////////////////

void centroid3D(const vecD& vec, double& centerX, double& centerY, double& centerZ)
{
	unsigned numOfPoints = vec.size() / 3;
	if (numOfPoints == 0 || vec.size() != numOfPoints * 3) {
		centerX = centerY = centerZ = 0.0;
		return;
	}

	double sumX = 0.0;
	double sumY = 0.0;
	double sumZ = 0.0;

	for (unsigned i=0; i<vec.size(); i+=3) {
		sumX += vec[i];
		sumY += vec[i+1];
		sumZ += vec[i+2];
	}

	centerX = sumX / double(numOfPoints);
	centerY = sumY / double(numOfPoints);
	centerZ = sumZ / double(numOfPoints);
}

double maxDistanceFromCentroid3D(const vecD& vec)
{
	unsigned numOfPoints = vec.size() / 3;
	if (numOfPoints == 0 || vec.size() != numOfPoints * 3) {
		return 0.0;
	}

	double centerX, centerY, centerZ;
	centroid3D(vec, centerX, centerY, centerZ);

	double maxDist = 0.0;
	double dist = 0.0;

	for (unsigned i=0; i<vec.size(); i+=3) {
		dist = distance3D(vec[i], vec[i+1], vec[i+2], centerX, centerY, centerZ);
		if (dist > maxDist) {
			maxDist = dist;
		}
	}

	return maxDist;
}

double meanDistanceFromCentroid3D(const vecD& vec)
{
	unsigned numOfPoints = vec.size() / 3;
	if (numOfPoints == 0 || vec.size() != numOfPoints * 3) {
		return 0.0;
	}

	double centerX, centerY, centerZ;
	centroid3D(vec, centerX, centerY, centerZ);
	
	double sumDist = 0.0;
	double dist = 0.0;

	for (unsigned i=0; i<vec.size(); i+=3) {
		dist = distance3D(vec[i], vec[i+1], vec[i+2], centerX, centerY, centerZ);
		sumDist += dist;
	}

	return sumDist / double(numOfPoints);
}

double meanRelativeDistanceFromCentroid3D(const vecD& vec)
{
	unsigned numOfPoints = vec.size() / 3;
	if (numOfPoints == 0 || vec.size() != numOfPoints * 3) {
		return 0.0;
	}

	double centerX, centerY, centerZ;
	centroid3D(vec, centerX, centerY, centerZ);
	
	double maxDist = maxDistanceFromCentroid3D(vec);
	if (maxDist == 0.0) return 0.0;

	double sumDist = 0.0;

	for (unsigned i=0; i<vec.size(); i+=3) {
		sumDist += distance3D(vec[i], vec[i+1], vec[i+2], centerX, centerY, centerZ);
	}

	return sumDist / maxDist / double(numOfPoints);
}

double medianDistanceFromCentroid3D(const vecD& vec)
{
	unsigned numOfPoints = vec.size() / 3;
	if (numOfPoints == 0 || vec.size() != numOfPoints * 3) {
		return 0.0;
	}

	double centerX, centerY, centerZ;
	centroid3D(vec, centerX, centerY, centerZ);

	vecD distances;
	for (unsigned i=0; i<vec.size(); i+=3) {
		distances.push_back(distance3D(vec[i], vec[i+1], vec[i+2], centerX, centerY, centerZ));
	}

	return median1D(distances);
}

vecD getRelativeDistances3D(const vecD& vec)
{
	vecD relativeDistances;
	unsigned numOfPoints = vec.size() / 3;
	if (numOfPoints == 0 || vec.size() != numOfPoints * 3) {
		return relativeDistances;
	}

	double centerX, centerY, centerZ;
	centroid3D(vec, centerX, centerY, centerZ);
	
	double maxDist = maxDistanceFromCentroid3D(vec);

	for (unsigned i=0; i<vec.size(); i+=3) {
		if (maxDist != 0.0) {
			relativeDistances.push_back(distance3D(vec[i], vec[i+1], vec[i+2], centerX, centerY, centerZ) / maxDist);
		}
		else {
			relativeDistances.push_back(0.0);
		}
	}

	return relativeDistances;
}

double medianRelativeDistanceFromCentroid3D(const vecD& vec)
{
	return median1D(getRelativeDistances3D(vec));
}

double varianceDistanceFromCentroid3D(const vecD& vec)
{
	unsigned numOfPoints = vec.size() / 3;
	if (numOfPoints < 2 || vec.size() != numOfPoints * 3) {
		return 0.0;
	}

	double centerX, centerY, centerZ;
	centroid3D(vec, centerX, centerY, centerZ);

	double sum = 0.0;
	for (unsigned i=0; i<vec.size(); i+=3) {
		double dist = distance3D(vec[i], vec[i+1], vec[i+2], centerX, centerY, centerZ);
		sum += dist * dist;
	}

	return sum / double(numOfPoints - 1);
}

double varianceRelativeDistanceFromCentroid3D(const vecD& vec)
{
	unsigned numOfPoints = vec.size() / 3;
	if (numOfPoints < 2 || vec.size() != numOfPoints * 3) {
		return 0.0;
	}

	double centerX, centerY, centerZ;
	centroid3D(vec, centerX, centerY, centerZ);
	
	double maxDist = maxDistanceFromCentroid3D(vec);
	if (maxDist == 0.0) return 0.0;
	double sum = 0.0;

	for (unsigned i=0; i<vec.size(); i+=3) {
		double dist = distance3D(vec[i], vec[i+1], vec[i+2], centerX, centerY, centerZ);
		sum += dist * dist;
	}

	return sum / (maxDist * maxDist) / double(numOfPoints - 1);
}

double standardDeviationDistanceFromCentroid3D(const vecD& vec)
{
	return std::sqrt(varianceDistanceFromCentroid3D(vec));
}

double standardDeviationRelativeDistanceFromCentroid3D(const vecD& vec)
{
	return std::sqrt(varianceRelativeDistanceFromCentroid3D(vec));
}

double histogram3D(const vecD& vec, double begin, double end)
{
	unsigned numOfPoints = vec.size() / 3;
	if (numOfPoints == 0 || vec.size() != numOfPoints * 3) {
		return 0.0;
	}

	vecD relativeDistances = getRelativeDistances3D(vec);
	if (relativeDistances.size() == 0) return 0.0;
	int n = 0;

	for (unsigned i=0; i!=relativeDistances.size(); ++i) {
		if (relativeDistances[i] >= begin && relativeDistances[i] < end) {
			++n;
		}
	}

	return double(n) / double(relativeDistances.size());
}

// 3D misc ////////////////////////////////////////////////////////////////////////////////////////

void crossProduct3D(double x1, double y1, double z1, double x2, double y2, double z2, double& r1, double& r2, double& r3)
{
	r1 = y1*z2 - z1*y2;
	r2 = z1*x2 - x1*z2;
	r3 = x1*y2 - y1*x2;
}

double distance3D(double x1, double y1, double z1, double x2, double y2, double z2)
{
	double distX = x2 - x1;
	double distY = y2 - y1;
	double distZ = z2 - z1;

	return std::sqrt(distX*distX + distY*distY + distZ*distZ);
} 

double dotProduct3D(double x1, double y1, double z1, double x2, double y2, double z2)
{
	return x1*x2 + y1*y2 + z1*z2;
}

double extendX3D(const vecD& vec)
{
	if (vec.size() < 2) return 0.0;

	double minX = std::numeric_limits<double>::infinity();
	double maxX = -minX;

	for (unsigned i=0; i<vec.size(); i+=3) {
		if (vec[i] < minX) minX = vec[i];
 		if (vec[i] > maxX) maxX = vec[i];
	}
	
	return maxX - minX;
}

double extendY3D(const vecD& vec)
{
	if (vec.size() < 2) return 0.0;

	double minY = std::numeric_limits<double>::infinity();
	double maxY = -minY;

	for (unsigned i=0; i<vec.size(); i+=3) {
		if (vec[i+1] < minY) minY = vec[i+1];
 		if (vec[i+1] > maxY) maxY = vec[i+1];
	}
	
	return maxY - minY; 
}

double extendZ3D(const vecD& vec)
{
	if (vec.size() < 2) return 0.0;

	double minZ = std::numeric_limits<double>::infinity();
	double maxZ = -minZ;

	for (unsigned i=0; i<vec.size(); i+=3) {
		if (vec[i+2] < minZ) minZ = vec[i+2];
 		if (vec[i+2] > maxZ) maxZ = vec[i+2];
	}
	
	return maxZ - minZ;
}

void normalizeVector3D(double& x, double& y, double& z)
{
	double length = length3D(x, y, z);
	x /= length;
	y /= length;
	z /= length;
}

void transformVector3D(double t11, double t12, double t13, double t21, double t22, double t23, double t31, double t32, double t33, double& v1, double& v2, double& v3)
{
	double r1 = t11*v1 + t12*v2 + t13*v3;
	double r2 = t21*v1 + t22*v2 + t23*v3;
	double r3 = t31*v1 + t32*v2 + t33*v3;
	v1 = r1;
	v2 = r2;
	v3 = r3;
}

double length3D(double x, double y, double z)
{
	return std::sqrt(x*x + y*y + z*z);
}

// matrix operations //////////////////////////////////////////////////////////////////////////////

double determinant3D(double t11, double t12, double t13, double t21, double t22, double t23, double t31, double t32, double t33)
{
	return t11*t22*t33 + t21*t32*t13 + t31*t12*t23 -t13*t22*t31 -t23*t32*t11 -t33*t12*t21;
}

void invertMatrix3D(double& t11, double& t12, double& t13, double& t21, double& t22, double& t23, double& t31, double& t32, double& t33)
{
	double det = determinant3D(t11, t12, t13, t21, t22, t23, t31, t32, t33);
	if (det == 0.0) {
		t11 = t12 = t13 = t21 = t22 = t23 = t31 = t32 = t33 = 0.0;
		return;
	}
	double c = 1.0/det;
	
	double t11_ = c * (t22*t33 - t23*t32);
	double t12_ = c * (t13*t32 - t12*t33);
	double t13_ = c * (t12*t23 - t13*t22);
	double t21_ = c * (t23*t31 - t21*t33);
	double t22_ = c * (t11*t33 - t13*t31);
	double t23_ = c * (t13*t21 - t11*t23);
	double t31_ = c * (t21*t32 - t22*t31);
	double t32_ = c * (t12*t31 - t11*t32);
	double t33_ = c * (t11*t22 - t12*t21);

	t11 = t11_; t12 = t12_; t13 = t13_;
	t21 = t21_; t22 = t22_; t23 = t23_;
	t31 = t31_; t32 = t32_; t33 = t33_;
}

// miscellaneous functions ////////////////////////////////////////////////////////////////////////

double angleDeg(double angleRad)
{
	return (180.0 / M_PI) * angleRad;
}

double angleClampMpiPi(double angleRad)
{
    while (angleRad < -M_PI)
        angleRad += 2*M_PI;

    while (angleRad >= M_PI)
        angleRad -= 2*M_PI;

    return angleRad;
}

double areaPolygon(const vec2D& polygon)
{
	double sum = 0.0;

	for (unsigned i=0; i!=polygon.size(); ++i) {
		sum += polygon[i].first * polygon[(i+1) % polygon.size()].second - polygon[(i+1) % polygon.size()].first * polygon[i].second;
	}

	return 0.5 * sum;
}

double areaPolygonSigned(const vec2D& polygon)
{
	if (polygon.size() == 0) return 0.0;

	// The polygon has to be closed.
	if (polygon[polygon.size()-1] != polygon[0]) {
		vec2D polygonClosed = polygon;
		polygonClosed.push_back(polygon[0]);

		return areaPolygon(polygonClosed);
	}

	return areaPolygon(polygon);
}

double areaPolygonAbs(const vec2D& polygon)
{
	return std::fabs(areaPolygonSigned(polygon));
}

vec2D projection(const vecF& vec, unsigned dim)
{
	if (vec.size() < 3 || dim > 2) return vec2D();
	vec2D ret(vec.size()/3);

	for (unsigned i=0; i<ret.size(); ++i) {
		ret[i].first = (dim == 0 ? vec[i*3+1] : vec[i*3]);
		ret[i].second = (dim == 2 ? vec[i*3+1] : vec[i*3+2]);
	}

	return ret;
}

vec2D projection(const vecD& vec, unsigned dim)
{
	if (vec.size() < 3 || dim > 2) return vec2D();
	vec2D ret(vec.size()/3);

	for (unsigned i=0; i<ret.size(); ++i) {
		ret[i].first = (dim == 0 ? vec[i*3+1] : vec[i*3]);
		ret[i].second = (dim == 2 ? vec[i*3+1] : vec[i*3+2]);
	}

	return ret;
}

vecD projection2(const vecD& vec, unsigned dim)
{
	if (vec.size() < 3 || dim > 2) return vecD();
	vecD ret(vec.size()/3*2);

	int n = 0;
	for (unsigned i=0; i<vec.size(); i+=3) {
		ret[n++] = (dim == 0 ? vec[i+1] : vec[i]);
		ret[n++] = (dim == 2 ? vec[i+1] : vec[i+2]);
	}

	return ret;
}

void projectionPointPlane(double p1, double p2, double p3, double a, double b, double c, double d, double& r1, double& r2, double& r3)
{
	double t0 = (-d - dotProduct3D(a, b, c, p1, p2, p3)) / dotProduct3D(a, b, c, a, b, c);

	r1 = p1 + t0 * a;
	r2 = p2 + t0 * b;
	r3 = p3 + t0 * c;
}

bool pineda(point2D t1, point2D t2, point2D t3, point2D p)
{
  double x = p.first;
  double y = p.second;

  // edge (t1, t2)
  double x1 = t1.first;
  double y1 = t1.second;
  double x2 = t2.first;
  double y2 = t2.second;
  double dx = x2 - x1;
  double dy = y2 - y1;
  double d = dy*x - dx*y - dy*x1 + dx*y1;
  if (d > 0.0) return false;

  // edge (t2, t3)
  x1 = t2.first;
  y1 = t2.second;
  x2 = t3.first;
  y2 = t3.second;
  dx = x2 - x1;
  dy = y2 - y1;
  d = dy*x - dx*y - dy*x1 + dx*y1;
  if (d > 0.0) return false;

  // edge (t3, t1)
  x1 = t3.first;
  y1 = t3.second;
  x2 = t1.first;
  y2 = t1.second;
  dx = x2 - x1;
  dy = y2 - y1;
  d = dy*x - dx*y - dy*x1 + dx*y1;
  if (d > 0.0) return false;

  return true;
}

bool segmentIntersection(point2D p1, point2D p2, point2D p3, point2D p4, point2D& p, double& s, double& t)
{
	p.first = 0.0;
	p.second = 0.0;
	s = 0.0;
	t = 0.0;

	if (!segmentIntersectionMaybe(p1, p2, p3, p4)) return false;

	double x1 = p1.first;
	double y1 = p1.second;
	double x2 = p2.first;
	double y2 = p2.second;
	double x3 = p3.first;
	double y3 = p3.second;
	double x4 = p4.first;
	double y4 = p4.second;
	
	double t2 = x1*y4 + x4*y2 + x2*y3 + x3*y1 - x4*y1 - x1*y3 - x3*y2 - x2*y4;
	double t1, s1, s2;

	const double eps = std::numeric_limits<double>::epsilon();
	if (-eps <= t2 && t2 <= eps) t2 = 0.0;
	if (t2 != 0.0) {
		t1 = x1*y2 + x2*y3 + x3*y1 - x2*y1 - x3*y2 - x1*y3;
		t = t1 / t2;

		s2 = x2 - x1;

		if (-eps <= s2 && s2 <= eps) s2 = 0.0;
		if (s2 == 0.0) {
			s1 = y3 + t*y4 - t*y3 - y1;
			s2 = y2 - y1;
			if (-eps <= s2 && s2 <= eps) s2 = 0.0;
			if (s2 == 0.0) return false;
		}
		else {
			s1 = x3 + t*x4 - t*x3 - x1;
		}
		s = s1 / s2;
		p.first = x1 + s * (x2 - x1);
		p.second = y1 + s * (y2 - y1);
	}

	else {
		s2 = x3*y2 + x2*y4 + x4*y1 + x1*y3 - x2*y3 - x3*y1 - x1*y4 - x4*y2;
		if (-eps <= s2 && s2 <= eps) s2 = 0.0;
		if (s2 == 0.0) return false;

		s1 = x3*y4 + x4*y1 + x1*y3 - x4*y3 - x1*y4 - x3*y1;
		s = s1 / s2;

		t2 = x4 - x3;

		if (-eps <= t2 && t2 <= eps) t2 = 0.0;
		if (t2 == 0.0) {
			t1 = y1 + s*y2 - s*y1 -y3;
			t2 = y4 - y3;
			if (-eps <= t2 && t2 <= eps) t2 = 0.0;
			if (t2 == 0.0) return false;
		}
		else {
			t1 = x1 + s*x2 - s*x1 -x3;
		}

		t = t1 / t2;
		p.first = x1 + t * (x2 - x1);
		p.second = y1 + t * (y2 - y1);
	}

	if (-eps <= s && s<= eps) s = 0.0;
	if (1.0-eps <= s && s <= 1.0+eps) s = 1.0;
	if (-eps <= t && t <= eps) t = 0.0;
	if (1.0-eps <= t && t <= 1.0+eps) t = 1.0;
	if (!(0.0 <= s && s <= 1.0 && 0.0 <= t && t <= 1.0)) return false;
	return true;
}

bool segmentIntersection(point2D p1, point2D p2, point2D p3, point2D p4)
{
	point2D pTmp;
	double sTmp, tTmp;
	return segmentIntersection(p1, p2, p3, p4, pTmp, sTmp, tTmp);
}

bool realSegmentIntersection(point2D p1, point2D p2, point2D p3, point2D p4, point2D& p, double& s, double& t)
{
	return (segmentIntersection(p1, p2, p3, p4, p, s, t) 
	        && !((s == 0.0 || s == 1.0) && (t == 0.0 || t == 1.0)));
}

bool realSegmentIntersection(point2D p1, point2D p2, point2D p3, point2D p4)
{
	point2D pTmp;
	double sTmp, tTmp;

	return realSegmentIntersection(p1, p2, p3, p4, pTmp, sTmp, tTmp);
}

bool realSegmentIntersection(double p1x, double p1y, double p2x, double p2y, double p3x, double p3y, double p4x, double p4y)
{
	return realSegmentIntersection(std::pair<double, double>(p1x, p1y), std::pair<double, double>(p2x, p2y), std::pair<double, double>(p3x, p3y), std::pair<double, double>(p4x, p4y));
}

bool segmentIntersectionMaybe(point2D p1, point2D p2, point2D p3, point2D p4)
{
	double x1, x2, x3, x4, y1, y2, y3, y4;
	if (p1.first <= p2.first) {x1 = p1.first; x2 = p2.first;}
	else {x1 = p2.first; x2 = p1.first;}
	if (p3.first <= p4.first) {x3 = p3.first; x4 = p4.first;}
	else {x3 = p4.first; x4 = p3.first;}
	if (p1.second <= p2.second) {y1 = p1.second; y2 = p2.second;}
	else {y1 = p2.second; y2 = p1.second;}
	if (p3.second <= p4.second) {y3 = p3.second; y4 = p4.second;}
	else {y3 = p4.second; y4 = p3.second;}

	return !( (x2 < x3 || x1 > x4) && (y2 < y3 || y1 > y4) );
}


bool checkCircleSegmentIntersection (const Vector2d& p1, const Vector2d& p2, const Vector2d& m, double r )
{
    double dx = p2.x() - p1.x();
    double dy = p2.y() - p1.y();

    // parameter t of point on line nearest to circle center M
    double t = -((p1.x() - m.x())*dx + (p1.y() - m.y())*dy) / ((dx*dx) + (dy*dy));

    // we consider only the line segment with 0<=t<=1
    if(t < 0.0) {
        t = 0.0;
    } else if(t > 1.0) {
        t = 1.0;
    }

    dx = (p1.x() + t*(p2.x() - p1.x())) - m.x();
    dy = (p1.y() + t*(p2.y() - p1.y())) - m.y();
    double d = (dx*dx) + (dy*dy);

    if(d< r*r) {
        return true;
    } else {
        return false;
    }
}


int circleSegmentIntersection(const Vector2d& p1, const Vector2d& p2, const Vector2d& m, double r, Vector2d& s1, Vector2d& s2 )
{
    Vector2d d=p2-p1;

    double disc = (2*d.x()*d.x()*m.y()*p1.y() - d.x()*d.x()*m.y()*m.y() - d.x()*d.x()*p1.y()*p1.y() + d.x()*d.x()*r*r +
                   2*d.x()*d.y()*m.x()*m.y() - 2*d.x()*d.y()*m.x()*p1.y() - 2*d.x()*d.y()*m.y()*p1.x() +
                   2*d.x()*d.y()*p1.x()*p1.y() - d.y()*d.y()*m.x()*m.x() + 2*d.y()*d.y()*m.x()*p1.x() - d.y()*d.y()*p1.x()*p1.x()
                   + d.y()*d.y()*r*r);
    int n=0;
    double t[2];
    if (disc>TOL_GEOM) {
        double t1 = (d.x()*m.x() + d.y()*m.y() - d.x()*p1.x() - d.y()*p1.y() - sqrt(disc))/(d.x()*d.x()+d.y()*d.y());
        double t2 = (d.x()*m.x() + d.y()*m.y() - d.x()*p1.x() - d.y()*p1.y() + sqrt(disc))/(d.x()*d.x()+d.y()*d.y());
        if ((t1>=0.0)&&(t1<=1.0)) {
            t[n]=t1;
            ++n;
        }
        if ((t2>=0.0)&&(t2<=1.0)) {
            t[n]=t2;
            ++n;
        }
    } else if (fabs(disc)<TOL_GEOM) {
        t[0] = (d.x()*m.x() + d.y()*m.y() - d.x()*p1.x() - d.y()*p1.y())/(d.x()*d.x()+d.y()*d.y());
        if ((t[0]>=0.0)&&(t[0]<=1.0)) {
            ++n;
        }
    }
    if (n==2) {
        s1=p1+t[0]*d;
        s2=p1+t[1]*d;
        return 2;
    } else if (n==1) {
        s1=p1+t[0]*d;
        return 1;
    } else {
        return 0;
    }
}


///////////////////////////////////////////////////////////////////////////////////////////////////

void printVector(const vecB& vec)
{
	std::clog << "Vector[" << vec.size() << "](";
	for (unsigned i=0; i!=vec.size(); ++i) {
		std::clog << (i==0 ? "" : ", ") << vec[i];
	}
	std::clog << ")\n";
}

void printVector(const vecF& vec)
{
	std::clog << "Vector[" << vec.size() << "](";
	for (unsigned i=0; i!=vec.size(); ++i) {
		std::clog << (i==0 ? "" : ", ") << vec[i];
	}
	std::clog << ")\n";
}

void printVector(const vecD& vec)
{
	std::clog << "Vector[" << vec.size() << "](";
	for (unsigned i=0; i!=vec.size(); ++i) {
		std::clog << (i==0 ? "" : ", ") << vec[i];
	}
	std::clog << ")\n";
}

void printVector(const vec2D& vec)
{
	std::clog << "Vector[" << vec.size() << "](";
	for (unsigned i=0; i!=vec.size(); ++i) {
		std::clog << (i==0 ? "" : ", ") << "(" << vec[i].first << "," << vec[i].second << ")";
	}
	std::clog << ")\n";
}

void printBinaryMatrix(const vecB& binaryMatrix, int width, int height)
{
	for (int y=0; y!=height; ++y) {
		for (int x=0; x!=width; ++x) {
			std::clog << binaryMatrix[y*width + x];
		}
		std::clog << std::endl;
	}
	std::clog << std::endl;
}

void printPoint(const point2D& p)
{
	std::clog << "(" << p.first << "," << p.second << ")";
}

int ind(int x, int y, int width)
{
	return y * width + x;
}

vecB getOuterPoints(const vecB& binaryMatrix, int w, int h)
{
	vecB ret = binaryMatrix;

	for (int y=1; y!=h-1; ++y) {
		for (int x=1; x!=w-1; ++x) {

			if (!binaryMatrix[ind(x, y, w)]) {

				if ((!binaryMatrix[ind(x,   y-1, w)]	// 101
				  && !binaryMatrix[ind(x,   y+1, w)]	// 000
				  && !binaryMatrix[ind(x-1, y,   w)]	// 101
				  && !binaryMatrix[ind(x+1, y,   w)])

				 || (!binaryMatrix[ind(x-1, y-1, w)]	// 010
				  && !binaryMatrix[ind(x+1, y+1, w)]	// 101
				  && !binaryMatrix[ind(x+1, y-1, w)]	// 010
				  && !binaryMatrix[ind(x-1, y+1, w)])) {

						ret[ind(x, y, w)] = true;
				}
			}
		}
	}

	return ret;
}

vec2D getScaledVector(const vec2D& vec, double scaleX, double scaleY)
{
	vec2D ret(vec.size());
	
	for (unsigned i=0; i!=ret.size(); ++i) {
		ret[i].first = vec[i].first * scaleX;
		ret[i].second = vec[i].second * scaleY;
	}

	return ret;
}

vec2D getScaledVector(const vec2D& vec, double scale)
{
	return getScaledVector(vec, scale, scale);
}

vecF getSubset(const vecF& vec, const vecB& ind, bool invert)
{
	if (vec.size()/3 != ind.size()) return vec;

	vecF ret;
	for (unsigned i=0; i!=ind.size(); ++i) {
		if ((!invert && ind[i]) || (invert && !ind[i])) {
			for (unsigned j=0; j!=3; ++j) {
				ret.push_back(vec[i*3+j]);
			}
		} 
	}

	return ret;
}

double drand()
{
    return rand() / (RAND_MAX + 1.);
}


double sampleNormalDistribution(double b)
{
    // see Probabilistic Robotics p. 124
    double s = 0.0;

    for (int i = 0; i < 12; i++) {
        s += 2.0*drand() - 1.0;
    }

    return b*s*0.5;
}

void randPerm(int n,std::vector<int>& p)
{
	if (p.size() != n)
		p.resize(n);

	for (int i = 0; i < n; i++) {
		int j = rand() % (i + 1);
		p[i] = p[j];
		p[j] = i;
	}
}


void randSampleNoReplacement(int nmax, int nsamples, std::vector<int>& p)
{
	if (p.size() != nsamples)
		p.resize(nsamples);

	for (int i = 0; i < nsamples; i++) {
		bool unique = false;
		while (!unique)
		{
			p[i] = rand() % nmax;
			unique = true;
			for (int j = 0; j < i; j++)
				if (p[j] == p[i])
					unique = false;
		}
	}
}
