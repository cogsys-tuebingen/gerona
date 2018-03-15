#ifndef CLOSEDSET_H
#define CLOSEDSET_H


#include "plannerutils.h"



/**
 * @brief Helper class, searches for close points on the given tree level
 */
class ClosedSetLevel{
public:
    std::vector<cv::Point3f> entries_;

    inline float GetDistance(const cv::Point3f &p1, const cv::Point3f &p2)
    {
        return (p1.x-p2.x)*(p1.x-p2.x)+ (p1.y-p2.y)*(p1.y-p2.y);
    }

    inline bool Test(const cv::Point3f &pose)
    {
        for (int i = 0; i < entries_.size();++i)
        {
            if (GetDistance(pose,entries_[i]) < distanceSqrdThresh &&  std::abs(pose.z-entries_[i].z) < rotDiffThresh ) return true;
        }
        entries_.push_back(pose);
        return false;
    }

    void Reset()
    {
        entries_.clear();
    }

    float distanceSqrdThresh, rotDiffThresh;

};


/**
 * @brief Closed Set implementation used for A* like search algorithms
 */
class ClosedSet{

public:


    void Reset()
    {
        for (int tl = 0; tl < levels_.size();++tl)
        {

            levels_[tl].Reset();
        }

        numHits_ = 0;
    }

    void Setup(int levels, float maxDist, float maxRot)
    {
        if (levels_.size() != levels)
        {
            levels_.clear();
            for (int tl = 0; tl < levels;++tl)
            {
                ClosedSetLevel entry;

                entry.distanceSqrdThresh = maxDist*maxDist;
                entry.rotDiffThresh = maxRot;

                levels_.push_back(entry);
            }
        }
        else Reset();

    }

    bool Test(const int level, const cv::Point3f &pose)
    {
        bool res = levels_[level].Test(pose);
        if (res) numHits_++;
        return res;
    }

    std::vector<ClosedSetLevel> levels_;
    int numHits_;

};


#endif // CHASSISMODEL_H
