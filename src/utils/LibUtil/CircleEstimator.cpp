/**************************************************************************
    @project RA Outdoor Robot System
    @author Karsten Bohlmann
    @date 24.3.2010 early 21st century
    (c) Universitaet Tuebingen 2010

**************************************************************************/
#include <math.h>
#include <iostream>
#include "CircleEstimator.h"

CircleEstimator::CircleEstimator (double threshold)
{
    Reset();
    SetThreshold(threshold);
}


CircleEstimator::~CircleEstimator()
{
    // nothing to do
}


void CircleEstimator::Reset()
{
    mRadius = mRadiusK=-1;
    mCenter.setZero();
    mArcLength = 0;
    mPoints.clear();
    mUpdateInterval = 3;
    mIsCircle=false;
}


void CircleEstimator::AddPoint(double x, double y, double phi)
{
    Vector3d p(x,y,phi);
    mPoints.push_back(p);
}


void CircleEstimator::AddPoint(const Vector3d& p)
{
    mPoints.push_back(p);
}


void CircleEstimator::GetPointList(Vector3dList& points, double relErrThresh)
{
    points.clear();
    if (mRadiusK<=0) {
        return;
    }
    if (relErrThresh<=0) {
        points.insert(points.end(),mPoints.begin(),mPoints.end());
    } else {
        double r=mRadiusK;
        // select all good points
        for (Vector3dList::iterator pIt=mPoints.begin();pIt!=mPoints.end();++pIt) {
            double d= (pIt->head<2>()-mCenter).norm();
            double errRel = fabs((r-d)/r);
            if (errRel<relErrThresh) {
                points.push_back(*pIt);
            }
        }
    }
}


void CircleEstimator::EliminateOutliers()
{
    Vector3dList::iterator pIt=mPoints.begin();
    Vector3dList::iterator prevIt = pIt;
    std::list<Vector3dList *> pointGroups;
    Vector3dList *group = new Vector3dList();
    group->push_back(*prevIt);
    pointGroups.push_back(group);
    ++pIt;
    while (pIt!=mPoints.end()) {
        if ((pIt->head<2>()-prevIt->head<2>()).squaredNorm()<mThreshold) {
            // add to current group of points
            group->push_back(*pIt);
        } else {
            group = new Vector3dList();
            group->push_back(*pIt);
            pointGroups.push_back(group);
        }
        prevIt = pIt;
        ++pIt;
    }
    if (pointGroups.size()==1) {
        // nothing to do
        return;
    }
    // check first and last point
    if ((*mPoints.begin()-*prevIt).squaredNorm()<mThreshold) {
        // join the two groups
        (*pointGroups.begin())->insert((*pointGroups.begin())->end(),group->begin(),group->end());
        pointGroups.remove(pointGroups.back());
        delete group;
    }

    unsigned maxSize=0;
    for (std::list<Vector3dList *>::iterator gIt=pointGroups.begin();gIt!=pointGroups.end();++gIt) {
        std::cout << "group with size "<<(*gIt)->size() << std::endl;
        if ((*gIt)->size()>maxSize) {
            maxSize = (*gIt)->size();
            group = *gIt;
        }
    }
    mPoints.clear();
    std::cout << "using group with size "<<group->size() << std::endl;
    mPoints.insert(mPoints.end(),group->begin(),group->end());
}


void CircleEstimator::Update ()
{
    int n=mPoints.size();
    if (n<3) {
        return;
    }
    if (mThreshold>0) {
        EliminateOutliers();
    }

    double A,B,C,D,E;
    double Sxx=0,Sx=0,Sy=0,Sxy=0,Syy=0,Sxxx=0,Sxyy=0,Syxx=0,Syyy=0;

    for (Vector3dList::iterator pIt=mPoints.begin();pIt!=mPoints.end();++pIt) {

        double x=(*pIt)(0);
        double y=(*pIt)(1);
        Sxx+=x*x;
        Sx+=x;
        Sxy+=x*y;
        Sy+=y;
        Syy+=y*y;
        Sxyy+=x*y*y;
        Sxxx+=x*x*x;
        Syxx+=y*x*x;
        Syyy+=y*y*y;
    }
    A=n*Sxx-Sx*Sx;
    B=n*Sxy-Sx*Sy;
    C=n*Syy-Sy*Sy;
    D=0.5*(n*Sxyy-Sx*Syy+n*Sxxx-Sx*Sxx);
    E=0.5*(n*Syxx-Sy*Sxx+n*Syyy-Sy*Syy);
    double denom=A*C-B*B;
    if (fabs(denom)<1e-6) {
        mIsCircle=false;
    } else {
        mIsCircle=true;
        mCenter.x()=(D*C-B*E)/denom;
        mCenter.y()=(A*E-B*D)/denom;
        double rm=0;
        double rk=0;
        for (Vector3dList::iterator pIt=mPoints.begin();pIt!=mPoints.end();++pIt) {
            double x=(*pIt)(0);
            double y=(*pIt)(1);
            rm+=sqrt((x-mCenter.x())*(x-mCenter.x())+(y-mCenter.y())*(y-mCenter.y()));
            rk+=(x-mCenter.x())*(x-mCenter.x())+(y-mCenter.y())*(y-mCenter.y());
        }
        mRadiusK = sqrt(rk/double(n));
        mRadius=rm/n;
        mCircleQuality.reset();
        for (Vector3dList::iterator pIt=mPoints.begin();pIt!=mPoints.end();++pIt) {
          double d=fabs((mCenter-pIt->head<2>()).norm()-mRadiusK);
          mCircleQuality.update(d);
        }

        // calculate mean distance of points and std deviation


    }
}


void CircleEstimator::GetCircleQuality (double& meanErr, double& stdErr)
{
  meanErr=mCircleQuality.getMean();
  stdErr=mCircleQuality.getStd();
}


Vector2d CircleEstimator::GetCenter()
{
   return mCenter;
}


double CircleEstimator::GetArcLength ()
{
    // calculate arc length as angle between circle center, first point, last point and estimated radius
    Vector2d a,b,am,bm;
    a=mPoints.front().head<2>();
    b=mPoints.back().head<2>();
    am = a-mCenter;
    bm= b-mCenter;
    double cosphi=am.dot(bm)/(am.norm()*bm.norm());
    return acos(cosphi)*mRadius;

}

#ifdef TESTALGORITHM

int main()
{
    CircleEstimator est;
    est.SetThreshold(0.3);

    double step=M_PI/100;
    for (int i=0;i<200;++i) {
        double x=cos(i*step);
        double y=sin(i*step);
        if (i>15 && i<20) {
            x+=1.0;
            y+=1.0;
        }
        est.AddPoint(x,y,0);
    }
    est.Update();
    cout << "circle radius="<< est.GetRadius()<< endl;
}

#endif

