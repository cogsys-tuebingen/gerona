#ifndef PI_DWADT_H
#define PI_DWADT_H


#include "plannertraj_dt.h"



/**
 * @brief DWA implementation
 */
template < typename TS>
class PI_DWADT : public PlannerTrajDT< TS>
{
public:
    typedef std::shared_ptr<PI_DWADT<TS> > Ptr;
    static PI_DWADT<TS>::Ptr Create(){ return std::make_shared< PI_DWADT<TS>  >() ; }

    typedef PlannerTrajDT<TS> TB;
    using TB::config_;
    using TB::CreateTrajectory;
    using TB::bestScore_;
    using TB::GetStartNode;
    using TB::scorer_;
    using TB::expander_;
    using TB::GetNextNode;
    using TB::curImgVelocity_;
    using TB::curImgRobotPose_;
    using TB::tempCmds_;
    using TB::FinishedPlanning;



    PI_DWADT()
    {

    }

    int GetNumberNodes()
    {
        return std::max(config_.expanderConfig_.firstLevelSplits,config_.expanderConfig_.numSplits)+1;
    }

    cv::Point2f Plan()
    {        

        config_.plannerConfig_.maxLevel = 1;
        TrajNodeDT *startNode = GetStartNode();


        const int numSplits = expander_->Expand(0,startNode->endCmd_,tempCmds_);

        for (int i = 0; i < numSplits;++i)
        {
            CreateTrajectory(startNode,tempCmds_[i]);

        }

        FinishedPlanning();


        return cv::Point2f(0,0);

    }


};

#endif // PI_ASTAR_H
