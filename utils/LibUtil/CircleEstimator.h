/**************************************************************************
    @project RA Outdoor Robot System
    @author Karsten Bohlmann
    @date 24.3.2010 early 21st century
    (c) Universitaet Tuebingen 2010

**************************************************************************/

#ifndef CIRCLEESTIMATOR_H
#define CIRCLEESTIMATOR_H
#include <list>
#include "StatsEstimator.h"
#include "Eigen/Core"
using namespace Eigen;


typedef std::list<Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > Vector3dList;

/**
  caluclates center and radius of a circle best fitting to
   a noisy set of points on the circles perimeter
   uses least-squares-fitting
   @ref umbach 2000 - a few methods for fitting circles to data
  */
class CircleEstimator
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CircleEstimator (double threshold=-1.0);
    ~CircleEstimator ();

    /**
      adds a point on the circles perimeter
      */
    void AddPoint (double x, double y,double phi);

    void AddPoint(const Vector3d& p);

    void SetThreshold (double thresh) {mThreshold = thresh*thresh;}
    double GetRadius() {return mRadius;}

    void GetPointList(Vector3dList& points, double errThreshold);
    /**
      approximation of radius by Kasa
      */
    double GetRadiusK() {return mRadiusK;}

    Vector3dList* GetPointList()  { return &mPoints; }
    Vector2d GetCenter();
    bool IsCircle();
    double GetArcLength();
    void Reset();
    /**
      calculates from previously added perimeter points
      radius and circle center
      */
    void Update();

    /**
      returns mean absolute distance between points and estimated circle
      */
    void GetCircleQuality (double& meanErr, double& stdErr);

private:
    void EliminateOutliers ();
    double mThreshold;
    Vector3dList mPoints;
    double  mRadius;
    double mRadiusK;
    double  mArcLength;
    Vector2d  mCenter;
    bool    mIsCircle;
    int     mUpdateInterval;
    StatsEstimator<double> mCircleQuality;
};




#endif // CIRCLEESTIMATOR_H
