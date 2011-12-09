#ifndef LASERENVIRONMENT_H
#define LASERENVIRONMENT_H
#include <Eigen/Core>

#include "Global.h"
using namespace Eigen;


class LaserEnvironment
{
public:    
    LaserEnvironment(int numberOfSegments=21);

    void ProcessLaserScan (const FVector& laserRanges, float minAngle, float maxAngle);


    void FindObstacles ();
    void SetRange(double range) {mRange=range;}
    void FindLongestCorridor (double width, double& bearing, double& length,
                               double minRad, double maxRad);

    double GetCorridorLength (double width, double course);
    double GetMinDist() {return mMinDist0;}

    /**
      check if there is an object within the rectangle (parallelogram) with
       angle course, width and length
      */
    bool  CheckCollision(double course, double width, double length);

    bool CheckCollision(const FVector& ranges, float min_angle, float max_angle, float course, float width,
                        float length,float thresh);


private:
    int mNumberOfSegments;
    int     mLaser0RangesNum;
    double  mViewRad; // view angle of laserscanner
    double  mLaserStartRad; // angle of first ray of laser
    double  mDeltaRad; // angle between two laser rays
    double  mStartRad; // angle of first laser beam to consider

    double  mEndRad;
    int     mStartIdx; // index of first laser beam to consider
    int     mEndIdx;
    int     mSegmentDelta; // number of indices in one segment
    double  mSegmentDeltaRad; // angle of one segment
    Vector2dVec mSegmentObstacles;
    double  mLaserMinDist; // minimum laser dist we accept as valid
    std::vector<double> mLaser0DataRanges;
    std::vector<double> mSegmentRanges;


    FVector   mSins,mCoss; // precalculated sinus and cosinus values for laserscan
    double mRho0;
    double mMinDist0;
    double mRange;
};

#endif // LASERENVIRONMENT_H
