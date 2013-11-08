/** \file */

#ifndef MATH_HELPER_H
#define MATH_HELPER_H

#include <utility>
#include <limits>
#include <vector>
#include <Eigen/Core>
#include "MathHelper.h"
using namespace Eigen;


#include "Global.h"
typedef std::pair<double, double> point2D;
typedef std::vector<point2D> vec2D;
typedef std::vector<bool> vecB;
typedef std::vector<float> vecF;
typedef std::vector<double> vecD;

// geometry floating point tolerance
// fabs(x)<TOL_GEOM is considered as zero
const double TOL_GEOM = 1e-12;


// convert quaternion to YawPitchRoll
void Quaternion2Ypr(const FVector& quat, FVector& ypr);


#include <Eigen/Core>

///////////////////////////////////////////////////////////////////////////////
// D E C L A R A T I O N S
///////////////////////////////////////////////////////////////////////////////

class MathHelper {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static Eigen::Vector3d OdometryDelta(
      const Eigen::Vector3d &old,
      const Eigen::Vector3d &latest );

  static double NormalizeAngle(const double angle);

  static double AngleDelta(const double a1, const double a2);

  static double Angle( const Eigen::Vector2d &v1, const Eigen::Vector2d &v2 );

  static double Angle( const Eigen::Vector2d &v );

  static double OdomDistance( const Eigen::Vector3d &p1, const Eigen::Vector3d &p2 );

  /// Clamps the angle in radians into the interval [-pi, pi)
  static double AngleClamp(double angleRad);

  /// clamp the value within the interval [low, high]
  template <typename T>
  static T clamp(T value, T low, T high)
  {
      return (value < low) ? low : ((value > high) ? high : value);
  }

  /**
    find peaks in data vector
    http://billauer.co.il/peakdet.html
    */
  template <typename T>
  static bool peakDetect(const vector<T>& data, T delta, vector<unsigned int>& mins, vector<unsigned int>& maxs)
  {
    T mn=std::numeric_limits<T>::max();
    T mx=std::numeric_limits<T>::min();

    int mnpos = -1, mxpos = -1;

    int lookformax = 1;

    for (unsigned int i=0;i<data.size();++i) {
      const T& val=data[i];
      if (val > mx) {
        mx = val;
        mxpos = i;
      }
      if (val < mn) {
        mn = val;
        mnpos = i;
      }
      if (lookformax) {
        if (val < mx-delta) {
          maxs.push_back(mxpos);
          mn = val; mnpos= i;
          lookformax = 0;
        }
      }  else {
        if (val > mn+delta) {
          mins.push_back(mnpos);
          mx = val; mxpos = i;
          lookformax = 1;
        }
      }
    }
    return true;
  }

};



template <class T> T sqr(const T& x) { return x*x; }


// 1D vec /////////////////////////////////////////////////////////////////////////////////////////

/// Returns the minimum of the elements in vec.
double min1D(const vecD& vec);

/// Returns the maximum of the elements in vec.
double max1D(const vecD& vec);

/// Returns the range of the elements in vec.
double range1D(const vecD& vec);

/// Returns the mean of the elements in vec.
double mean1D(const vecD& vec);

/// Returns the median of the elements in vec.
double median1D(const vecD& vec);

/// Returns the variance of the elements in vec.
double variance1D(const vecD& vec);

/// Returns the standard deviation of the elements in vec.
double standardDeviation1D(const vecD& vec);

/// Returns the relative frequency of the elements of vec falling in the interval [begin, end).
double histogram1D(const vecD& vec, double begin, double end);

// 2D vec /////////////////////////////////////////////////////////////////////////////////////////

/// Calculates the centroid (centerX, centerY) of the elements in vec.
void centroid2D(const vecD& vec, double& centerX, double& centerY);

/// Returns the maximal distance of the elements of vec from their centroid.
double maxDistanceFromCentroid2D(const vecD& vec);

/// Returns the mean distance of the elements of vec from their centroid.
double meanDistanceFromCentroid2D(const vecD& vec);

/// Returns the mean relative distance of the elements of vec from their centroid.
double meanRelativeDistanceFromCentroid2D(const vecD& vec);

/// Returns the median distance of the elements of vec from their centroid.
double medianDistanceFromCentroid2D(const vecD& vec);

/// Returns the median relative distance of the elements of vec from their centroid.
double medianRelativeDistanceFromCentroid2D(const vecD& vec);

/// Returns the variance of the distances of the elements of vec from their centroid.
double varianceDistanceFromCentroid2D(const vecD& vec);

/// Returns the variance of the relative distances of the elements of vec from their centroid.
double varianceRelativeDistanceFromCentroid2D(const vecD& vec);

/// Returns the standard deviation of the distances of the elements of vec fromt their centroid.
double standardDeviationDistanceFromCentroid2D(const vecD& vec);

/// Returns the standard deviation of the relative distances of the elements of vec from their centroid.
double standardDeviationRelativeDistanceFromCentroid2D(const vecD& vec);

/// Returns the relative frequency of the relative distances (to the centroid) of the elements of vec 
/// falling in the interval [begin, end).
double histogram2D(const vecD& vec, double begin, double end);

// 2D binary matrix ///////////////////////////////////////////////////////////////////////////////

/// Calculates the centroid (centerX, centerY) of the points in the width * height square,
/// denoted by the binaryMatrix.
void centroid2D(const vecB& binaryMatrix, int width, int height, double& centerX, double& centerY);

/// Returns the max distance of the points in the width * height square,
/// denoted by the binaryMatrix.
double maxDistanceFromCentroid2D(const vecB& binaryMatrix, int width, int height);

/// Returns the mean distance of the points in the width * height square,
/// denoted by the binaryMatrix.
double meanDistanceFromCentroid2D(const vecB& binaryMatrix, int width, int height);

/// Returns the meam relative distance of the points in the width * height square,
/// denoted by the binaryMatrix.
double meanRelativeDistanceFromCentroid2D(const vecB& binaryMatrix, int width, int height);

/// Returns the median distance of the points in the width * height square,
/// denoted by the binaryMatrix.
double medianDistanceFromCentroid2D(const vecB& binaryMatrix, int width, int height);

/// Returns the median relative distance of the points in the width * height square,
/// denoted by the binaryMatrix.
double medianRelativeDistanceFromCentroid2D(const vecB& binaryMatrix, int width, int height);

/// Returns the variance of the distances of the points in the width * height square,
/// denoted by the binaryMatrix.
double varianceDistanceFromCentroid2D(const vecB& binaryMatrix, int width, int height);

/// Returns the variance of the relative distances of the points in the width * height square,
/// denoted by the binaryMatrix.
double varianceRelativeDistanceFromCentroid2D(const vecB& binaryMatrix, int width, int height);

/// Returns the standard deviation of the distances of the points in the width * height square,
/// denoted by the binaryMatrix.
double standardDeviationDistanceFromCentroid2D(const vecB& binaryMatrix, int width, int height);

/// Returns the standard deviation of the relative distances of the points in the width * height square,
/// denoted by the binaryMatrix.
double standardDeviationRelativeDistanceFromCentroid2D(const vecB& binaryMatrix, int width, int height);

/// Returns the relative frequency of the relative distances (to the centroid) of the points in the width * height square, 
/// denoted by the binaryMatrix, falling in the interval [begin, end).
double histogram2D(const vecB& binaryMatrix, int width, int height, double begin, double end);

// 2D misc ////////////////////////////////////////////////////////////////////////////////////////

/// Calculates the smaller angle between (x1, y1), (x2, y2) and (x3, y3) in degrees.
/** @attention The angle always lies between 0° and 180°. **/
double angleSmallerDeg2D(double x1, double y1, double x2, double y2, double x3, double y3);

/// Calculates the smaller angle between p1, p2, p3 in degrees.
/** @attention The angle always lies between 0° and 180°. **/
double angleSmallerDeg2D(point2D p1, point2D p2, point2D p3);

/// Calculates the left angle between (x1, y1), (x2, y2) and (x3, y3) in degrees.
/** @attention The angle lies in [0°,  360°). **/
double angleLeftDeg2D(double x1, double y1, double x2, double y2, double x3, double y3);

/// Calculates the left angle between p1, p2 and p3 in degrees.
/** @attention The angle lies in [0°,  360°). **/
double angleLeftDeg2D(point2D p1, point2D p2, point2D p3);

/// Calculates the right angle between (x1, y1), (x2, y2) and (x3, y3) in degrees.
/** @attention The angle lies in [0°,  360°). **/
double angleRightDeg2D(double x1, double y1, double x2, double y2, double x3, double y3);

/// Calculates the right angle between p1, p2 and p3 in degrees.
/** @attention The angle lies in [0°,  360°). **/
double angleRightDeg2D(point2D p1, point2D p2, point2D p3);

/// Calculates the area of the triangle ((x1, y1), (x2, y2), (x3, y3)).
double areaTriangle2D(double x1, double y1, double x2, double y2, double x3, double y3);

/// Calculates the area of the triangle (p1, p2, p3).
double areaTriangle2D(point2D p1, point2D p2, point2D p3);

/// Calculates the distance between (x1, y1) and (x2, y2).
double distance2D(double x1, double y1, double x2, double y2);

/// Returns the dot product of (x1, y1) and (x2, y2).
double dotProduct2D(double x1, double y1, double x2, double y2);

/// Returns the dot product of p1 and p2.
double dotProduct2D(point2D p1, point2D p2);

/// Returns the length of the vector (x, y).
double length2D(double x, double y);

/// Returns the length of the vector p.
double length2D(point2D p);

/// Determines wheter point (x3, y3) lies left, right or on the line through (x1, y1) and (x2, y2)
/// and returns a negative value, a positive value or zero respectively.
double location2D(double x1, double y1, double x2, double y2, double x3, double y3);
/// Determines wheter point p3 lies left, right or on the line through p1 and p2
/// and returns a negative value, a positive value or zero respectively.
double location2D(point2D p1, point2D p2, point2D p3);

/// Normalizes the vector (x, y).
void normalizeVector2D(double& x, double& y);

// 3D vec /////////////////////////////////////////////////////////////////////////////////////////

/// Calculates the centroid (centerX, centerY, centerZ) of the elements in vec.
void centroid3D(const vecD& vec, double& centerX, double& centerY, double& centerZ);

/// Returns the maximal distance of the elements of vec from their centroid.
double maxDistanceFromCentroid3D(const vecD& vec);

/// Returns the mean distance of the elements of vec from their centroid.
double meanDistanceFromCentroid3D(const vecD& vec);

/// Returns the mean relative distance of the elements of vec from their centroid.
double meanRelativeDistanceFromCentroid3D(const vecD& vec);

/// Returns the median distance of the elements of vec from their centroid.
double medianDistanceFromCentroid3D(const vecD& vec);

/// Returns the median relative distance of the elements of vec from their centroid.
double medianRelativeDistanceFromCentroid3D(const vecD& vec);

/// Returns the variance of the distances of the elements of vec from their centroid.
double varianceDistanceFromCentroid3D(const vecD& vec);

/// Returns the variance of the relative distances of the elements of vec from their centroid.
double varianceRelativeDistanceFromCentroid3D(const vecD& vec);

/// Returns the standard deviation of the distances of the elements of vec fromt their centroid.
double standardDeviationDistanceFromCentroid3D(const vecD& vec);

/// Returns the standard deviation of the relative distances of the elements of vec from their centroid.
double standardDeviationRelativeDistanceFromCentroid3D(const vecD& vec);

/// Returns the relative frequency of the relative distances (to the centroid) of the elements of vec 
/// falling in the interval [begin, end).
double histogram3D(const vecD& vec, double begin, double end);

// 3D misc ////////////////////////////////////////////////////////////////////////////////////////

/// Calculates the cross product (r1, r2, r3) of (x1, y1, z1) and (x2, y2, z2).
void crossProduct3D(double x1, double y1, double z1, double x2, double y2, double z2, double& r1, double& r2, double& r3);

/// Returns the distance between (x1, y1, z1) and (x2, y2, z2).
double distance3D(double x1, double y1, double z1, double x2, double y2, double z2);

/// Returns the dot product of (x1, y1, z1) and (x2, y2, z2). 
double dotProduct3D(double x1, double y1, double z1, double x2, double y2, double z2);

/// Returns the extend of the elements (x, y, z) of vec in the x-dimension.
double extendX3D(const vecD& vec);

/// Returns the extend of the elements (x, y, z) of vec in the y-dimension.
double extendY3D(const vecD& vec);

/// Returns the extend of the elements (x, y, z) of vec in the z-dimension.
double extendZ3D(const vecD& vec);

/// Normalizes the vector (x, y, z).
void normalizeVector3D(double& x, double& y, double& z);

/// Transforms the vector (v1, v2, v3) := (t)_ij * (v)_i.
void transformVector3D(double t11, double t12, double t13, double t21, double t22, double t23, double t31, double t32, double t33, double& v1, double& v2, double& v3);

/// Returns the length of the vector (x, y, z).
double length3D(double x, double y, double z);

// matrix operations //////////////////////////////////////////////////////////////////////////////

/// Returns the determinant of the matrix (t)_ij.
double determinant3D(double t11, double t12, double t13, double t21, double t22, double t23, double t31, double t32, double t33);

/// Inverts the matrix (t)_ij.
void invertMatrix3D(double& t11, double& t12, double& t13, double& t21, double& t22, double& t23, double& t31, double& t32, double& t33);

// miscellaneous functions ////////////////////////////////////////////////////////////////////////

/// Calculates the angle in degrees from an angle in radians.
double angleDeg(double angleRad);

/// Clamps the angle in radians into the interval [-pi, pi)
double angleClampMpiPi(double angleRad);

/// Calculates the signed area of the polygon.
/** @attention The polygon has to be simple, i.e. non-self-intersecting, and should be closed.
  *   If the points of the triangle are arranged counter-clockwise
  *   (when the positive x- and y-axes point to the right and up respectively),
  *   the area is positive, otherwise negative.
  **/
double areaPolygonSigned(const vec2D& polygon);

/// Calculates the absolute area of the polygon.
/** @attention The polygon has to be simple, i.e. non-self-intersecting, and should be closed. **/
double areaPolygonAbs(const vec2D& polygon);

/// [(a1,b1), (a2,b2), ..., (an,bn)] = projection([x1,y1,z1, x2,y2,z2, ..., xn,yn,zn])
/// d = 0: ai = yi, bi = zi
/// d = 1: ai = xi, bi = zi
/// d = 2: ai = xi, bi = yi
vec2D projection(const vecF& vec, unsigned dim);
vec2D projection(const vecD& vec, unsigned dim);

/// (a1,b1, a2,b2, ..., an,bn] = projection([x1,y1,z1, x2,y2,z2, ..., xn,yn,zn])
/// d = 0: ai = yi, bi = zi
/// d = 1: ai = xi, bi = zi
/// d = 2: ai = xi, bi = yi
vecD projection2(const vecD& vec, unsigned dim);

/// Calculates the projection (r1, r2, r3) of the point (p1, p2, p3) onto the plane E: a*x + b*y + c*z + d = 0.
void projectionPointPlane(double p1, double p2, double p3, double a, double b, double c, double d, double& r1, double& r2, double& r3);

/// Determines wheter point p lies in the triangle (t1, t2, t3) or not.
/** @attention The points of the triangle must be specified in a counter-clockwise direction,
  * when the positive x- and y-axes point to the right and up respectively. 
	**/
bool pineda(point2D t1, point2D t2, point2D t3, point2D p);

/// Determines wheter the segments (p1, p2) and (p3, p4) intersect each other or not, 
/// and calculates the intersection point p.
bool segmentIntersection(point2D p1, point2D p2, point2D p3, point2D p4, point2D& p, double& s, double& t);

/// Determines wheter the segments (p1, p2) and (p3, p4) intersect each other or not.
bool segmentIntersection(point2D p1, point2D p2, point2D p3, point2D p4);

/// Like segmentIntersection, but excludes the case, that the segments have a point in common.
bool realSegmentIntersection(point2D p1, point2D p2, point2D p3, point2D p4, point2D& p, double& s, double& t);

/// Like segmentIntersection, but excludes the case, that the segments have a point in common.
bool realSegmentIntersection(point2D p1, point2D p2, point2D p3, point2D p4);
bool realSegmentIntersection(double p1x, double p1y, double p2x, double p2y, double p3x, double p3y, double p4x, double p4y);

bool segmentIntersectionMaybe(point2D p1, point2D p2, point2D p3, point2D p4);

/**
   checks whether circle and line segment intersect
   @author bohlmann
   @date 2011/02/23
  */
bool checkCircleSegmentIntersection (const Vector2d& p1, const Vector2d& p2, const Vector2d& m, double r );

/**
  calculates the intersection points of the line segment P1P2 with the circle around M with radius r.
  The first returned point S1 is always the one nearer to P1
  @param p1 start point of line segment
  @param p2 end point of line segment
  @param m  center point of circle
  @param r radius of circle
  @param[out] s1 first intersection point (if it exists)
  @param[out] s2 second intersection point (if it exists)
  @return number of intersection points

  @author bohlmann
  @date 2011/02/23
  */
int circleSegmentIntersection(const Vector2d& p1, const Vector2d& p2, const Vector2d& m, double r, Vector2d& s1, Vector2d& s2 );

///////////////////////////////////////////////////////////////////////////////////////////////////

/// Prints the vector vec for debugging.
void printVector(const vecB& vec);
void printVector(const vecF& vec);
void printVector(const vecD& vec);
void printVector(const vec2D& vec);

/// Prints the width * height binary matrix binaryMatrix for debugging. 
void printBinaryMatrix(const vecB& binaryMatrix, int width, int height);

/// Prints the point p for debugging.
void printPoint(const point2D& p);

/// Returns the outer points in the width * height square,
/// denoted by the binaryMatrix.
vecB getOuterPoints(const vecB& binaryMatrix, int w, int h);

/// Returns the scaled vector vec. 
/// Scales with scaleX in the x-dimension and with scaleY in the y-dimension.
vec2D getScaledVector(const vec2D& vec, double scaleX, double scaleY);

/// Returns the scaled vector vec. 
/// Scales with scale both in the x- and in the y-dimension.
vec2D getScaledVector(const vec2D& vec, double scale);

/// Returns a subset of vec, indicated by ind.
vecF getSubset(const vecF& vec, const vecB& ind, bool invert=false);

// random sampling and permutations ///////////////////////////////////////////////////////////////

/// Generates random double values in the interval [0,1)
double drand();

/// Samples from a zero-centered normal distribution with standard deviation b
double sampleNormalDistribution(double b);

/// Fills the vector p with a random permutation of the numbers [0...n-1]
void randPerm(int n,std::vector<int>& p);

/// Randomly samples without replacement nsamples integral numbers of the interval [0...n-1]
void randSampleNoReplacement(int nmax, int nsamples, std::vector<int>& p);

#endif
