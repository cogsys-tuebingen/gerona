#include <boost/progress.hpp>

#include <stdint.h>
#include <limits>
#ifndef EIGEN2_SUPPORT
#define EIGEN2_SUPPORT
#endif

#include "Eigen/Core"
using namespace Eigen;

#include "MathHelper.h"
#include "LaserEnvironment.h"

LaserEnvironment::LaserEnvironment(int numberOfSegments)
    :mNumberOfSegments(numberOfSegments)
{
    mStartRad = -90.0*M_PI/180.0;
    mEndRad = 90.0*M_PI/180.0;
    mLaserMinDist= 0.03;
    mRho0 = 0.0;
    mRange = 30;
    mSegmentRanges.resize(mNumberOfSegments);
    mSegmentObstacles.resize(mNumberOfSegments);
    mLaser0DataRanges.resize(1);
    mLaser0RangesNum = 1;
    mSins.resize(1);
    mCoss.resize(1);
    mSins[0]=0;
    mCoss[0]=1;
}



void LaserEnvironment::ProcessLaserScan (const FVector& laserRanges, float minAngle, float maxAngle)
{
    if (mLaser0RangesNum != laserRanges.size())
    {
        int n = laserRanges.size();
        mLaser0RangesNum = n;
        mViewRad = maxAngle-minAngle;
        mLaserStartRad = minAngle;
        if (mViewRad<=0.0) {
            return;
        }
        mDeltaRad = mViewRad/n;
        cout << "deltarad="<<mDeltaRad*180.0/M_PI << endl;
        mStartIdx = (mStartRad-minAngle)/mDeltaRad;
        if (mStartIdx<0) {
            mStartIdx=0;
            mStartRad=minAngle;
        }
        mEndIdx = (mEndRad-minAngle)/mDeltaRad;
        mSegmentDelta = (mEndIdx-mStartIdx)/mNumberOfSegments;
        int diff= (mEndIdx-mStartIdx)-mSegmentDelta*mNumberOfSegments;
        mStartIdx=mStartIdx+diff/2;
        mEndIdx = mStartIdx+mSegmentDelta*mNumberOfSegments;

        mSegmentDeltaRad = mSegmentDelta*mDeltaRad;
        cout << "segmentnum="<<mNumberOfSegments << " segmentdeltarad="<<mSegmentDeltaRad*180.0/M_PI << endl;
        mLaser0DataRanges.resize(mEndIdx-mStartIdx+1);

        mSegmentRanges.resize(mNumberOfSegments);
        mSegmentObstacles.resize(mNumberOfSegments);
    }
    // return if we dont have any laser readings
    if (mLaser0RangesNum==0) return;

    int idx=0;
    int s=0;
    double minDist = mRange;
    double minIdx = mStartIdx;
    mMinDist0 = mRange;
    for (int i=mStartIdx;i<mEndIdx;++i) {
        double dist = laserRanges[i];
        mLaser0DataRanges[idx++]= dist;
        if (dist > mLaserMinDist && dist<minDist ) {
            minDist = dist;
            minIdx = i;
        }
        if (dist > mLaserMinDist && dist<mMinDist0 ) {
            mMinDist0 = dist;
        }
        if (idx==(s+1)*mSegmentDelta) {
            if (s>=mNumberOfSegments) {
                cout << "idx="<<idx << "segnum*segdelta="<<mNumberOfSegments*mSegmentDelta<<endl;
                break;
            }
            mSegmentRanges[s]=minDist;
            double rad = mLaserStartRad+minIdx*mDeltaRad;
           // cout << "segment angle="<<(s*mSegmentDeltaRad+mStartRad)*180.0/M_PI<< " mindist="<<minDist << endl;
            mSegmentObstacles[s].y()=minDist*sin(rad);
            mSegmentObstacles[s].x()=minDist*cos(rad);
            minDist=mRange;
            ++s;
        }
    }
}


bool LaserEnvironment::CheckCollision(const FVector& ranges, float min_angle, float max_angle, float beta,
                                      float width, float length, float threshold)
{

  /*

 syms px py ax ay bx by cx cy b c a
>> [solb, solc]=solve(ax+c*(cx-ax)+b*(bx-ax)-px,ay+c*(cy-ay)+b*(by-ay)-py,b,c)

solb =

-(ax*cy - ay*cx - ax*py + ay*px + cx*py - cy*px)/(ax*by - ay*bx - ax*cy + ay*cx + bx*cy - by*cx)


solc =

(ax*by - ay*bx - ax*py + ay*px + bx*py - by*px)/(ax*by - ay*bx - ax*cy + ay*cx + bx*cy - by*cx)

http://stackoverflow.com/questions/1217585/parallelogram-contains-point
*/

  if (ranges.size()!=mSins.size()) {
    float astep=(max_angle-min_angle)/ranges.size();
    float angle=min_angle;
    mSins.resize(ranges.size());
    mCoss.resize(ranges.size());
    for (unsigned i=0;i<ranges.size();++i) {
      mSins[i]=std::sin(angle);
      mCoss[i]=std::cos(angle);
      angle+=astep;
    }
  }

  // corner points of the parallelogram
  float ax,ay,bx,by,cx,cy;



  float sbeta=std::sin(beta);
  float cbeta=std::cos(beta);
  ay=width/2.0f+length*sbeta;
  ax=0;
  by=-width/2.0f;
  bx=0.0f;
  cy=ay+threshold*sbeta;
  cx=ax+threshold*cbeta;
  // calc invariant parts of test equation
  float nom=ax*by - ay*bx - ax*cy + ay*cx + bx*cy - by*cx;
  float offb=-ax*cy + ay*cx;
  float offc=ax*by - ay*bx;

  unsigned coll_points=0;
  for (unsigned i=0;i<ranges.size();++i) {
     const float& r=ranges[i];

    if (r<threshold && r>0.01) {
      float& si=mSins[i];
      //if (si!=mSins[i]) cout << "arrrg";
      float& co=mCoss[i];
      float pb=ax*si-ay*co-cx*si+cy*co;
      float pc=-ax*si+ay*co+bx*si-by*co;

      if ((r*pb>=-offb) &&
          (r*pc>=-offc) &&
          (r*pb<=(nom-offb)) &&
          (r*pc<=(nom-offc))) {
          cout << "i="<<i << " point x="<<r*co << " y="<<r*si << " in parallelogram"<< endl;
          coll_points++;
          if (coll_points>1) {
            return true;
          }
      }
    }
  }
  return false;


}



bool LaserEnvironment::CheckCollision(double course, double width, double length)
{
    for (int s=0;s<mNumberOfSegments;++s) {
        double x= mSegmentObstacles[s].x();
        double y= mSegmentObstacles[s].y();


        if (x<0) continue;
        double d = mSegmentObstacles[s].norm();
        if (d<0.02) continue;
        double yoff = d*sin(course);

        if (y>yoff-width/2.0 && y<yoff+width/2.0 && d<length) {
            cout << "obstacle x="<<x << " y="<<y << endl;
            return true;
        }
    }
    return false;
}


double LaserEnvironment::GetCorridorLength (double width, double course)
{
    Vector2d p1=Vector2d::Zero();
    Vector2d p2,s1,s2;

    double radius = width/2.0;
    p2 << mRange*cos(course),mRange*sin(course);
    double minDist = mRange;
    for (Vector2dVec::iterator oIt=mSegmentObstacles.begin();oIt!=mSegmentObstacles.end();++oIt) {
        Vector2d& obstacle=*oIt;

        // grow obstacles by checking a circle around obstacle
        int n=circleSegmentIntersection(p1,p2,obstacle,radius,s1,s2);
        if (n>0) {
            double dist=s1.norm();
            if (dist<minDist) minDist = dist;
        }
    }
    return minDist;
}


void LaserEnvironment::FindLongestCorridor(double width, double& bestCourse, double& bestDist,
                                           double minRad, double maxRad)
{
    bestCourse = 0.0;
    bestDist = 0.0;
    Vector2d p1=Vector2d::Zero();

    Vector2d p2,s1,s2;
    bestDist = 0;
    bestCourse = minRad;
    double radius = width/2.0;

    double course = minRad;
    double stepRad = 5.0*M_PI/180.0;
    while (course<maxRad) {
        p2 << mRange*cos(course),mRange*sin(course);
        double minDist = mRange;
        for (Vector2dVec::iterator oIt=mSegmentObstacles.begin();oIt!=mSegmentObstacles.end();++oIt) {
            Vector2d& obstacle=*oIt;

            // grow obstacles by checking a circle around obstacle
            int n=circleSegmentIntersection(p1,p2,obstacle,radius,s1,s2);
            if (n>0) {
                double dist=s1.norm();
                if (dist<minDist) minDist = dist;
            }
        }
        if (minDist>bestDist) {
            bestDist = minDist;
            bestCourse = course;
        }
        course+=stepRad;
    }
}
