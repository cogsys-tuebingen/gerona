/**************************************************************************
    @project RA Outdoor Robot System
    @author Karsten Bohlmann
    @date 9/10/2010 early 21st century
    (c) Universitaet Tuebingen 2010

**************************************************************************/
#include <math.h>
#include "Global.h"
#include "BoundaryFinder.h"

LaserPointGroup::LaserPointGroup ()
    :mPoints(10),mNum(0)
{
    mMedDist = -1;
}


void LaserPointGroup::AddPoint(double dist, double rad)
{
    if (mNum>=mPoints.size()) {
        mPoints.resize(mNum*2);
    }
    mPoints[mNum].r = dist;
    mPoints[mNum].phi = rad;
    ++mNum;
}


void LaserPointGroup::CalcMedDist()
{
    mMedDist = 0;
    for (int i=0;i<mNum;++i) {
        mMedDist+=mPoints[i].r;
    }
    mMedDist = mMedDist/mNum;
}



BoundaryFinder::BoundaryFinder()
{
    mThresholdFactor = 4;
}


BoundaryFinder::~BoundaryFinder()
{
    // nothing to do yet
}




void BoundaryFinder::SetScan(const Laser2dScan& laserScan)
{
    double maxDist;
    GroupScanPoints(laserScan,mGroups,maxDist);
    PointGroupList outerGroups;
    FilterScanGroups (mGroups,outerGroups);
}


void BoundaryFinder::GroupScanPoints(const Laser2dScan &laserScan,
                                     PointGroupList &scanGroups, double &maxDist)
{
    double sinRes = sin(laserScan.resolution_/2.0);

    // find the most distant point first
    int maxIdx = -1;
    for (int i = 0; i<laserScan.ranges_.size();++i) {
        if (laserScan.ranges_[i]>maxDist) {
            maxDist = laserScan.ranges_[i];
            maxIdx = i;
        }
    }
    // group the points
    double phi = laserScan.min_rad_;
    double prevDist = 0.0;
    // counter how many points we skipped because of infinity/short dists
    int skipCount = 0;
    LaserPointGroup *group = new LaserPointGroup();
    for (int i=0;i<laserScan.ranges_.size();++i) {
        double dist = laserScan.ranges_[i];
        if (dist<=0.1) {
            // infinity or invalid value
            ++skipCount;
            continue;
        }
        double thresholdDist;
        if (skipCount>0) {
            thresholdDist= skipCount*dist*sinRes*2;
        } else {
            thresholdDist = skipCount*dist*sinRes*mThresholdFactor;
        }
        if (fabs(dist-prevDist)>thresholdDist) {
            if (group) {
                scanGroups.push_back(group);
            }
            group = new LaserPointGroup();
        }
        group->AddPoint(dist,phi);
    }

}


void BoundaryFinder::FilterScanGroups (PointGroupList& allGroups, PointGroupList& outerGroups)
{
    for (PointGroupList::const_iterator it=allGroups.begin();it!=allGroups.end();++it) {
        (*it)->CalcMedDist();
    }

}
