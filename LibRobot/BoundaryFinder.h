/**************************************************************************
    @project RA Outdoor Robot System
    @author Karsten Bohlmann
    @date 9/10/2010 early 21st century
    (c) Universitaet Tuebingen 2010

**************************************************************************/

#ifndef BOUNDARYFINDER_H
#define BOUNDARYFINDER_H
#include <list>
#include "Global.h"
#include "Laser2dScan.h"
struct Polar
{
    double r;
    double phi;
};
typedef vector<Polar> PolarVec;

class LaserPointGroup
{
public:
    LaserPointGroup ();
    void AddPoint (double dist, double rad);
    void CalcMedDist ();
private:
    PolarVec    mPoints;
    int         mNum;
    double      mMedDist;
};

typedef  std::list<LaserPointGroup*> PointGroupList;

class BoundaryFinder
{
public:
    BoundaryFinder();
    ~BoundaryFinder ();
    void SetScan (const Laser2dScan& laserScan);


private:
    void GroupScanPoints (const Laser2dScan& laserScan,
                          PointGroupList& scanGroups, double& maxDist);

    void FilterScanGroups (PointGroupList& allGroups, PointGroupList& outerGroups);
    PointGroupList mGroups;
    double mThresholdFactor;
};

#endif // BOUNDARYFINDER_H
