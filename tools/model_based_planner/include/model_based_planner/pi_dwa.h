#ifndef PI_DWA_H
#define PI_DWA_H


#include "plannertraj.h"



/**
 * @brief DWA implementation
 */
template <typename TE, typename TS>
class PI_DWA : public PlannerTraj<TE, TS>
{
public:
    typedef std::shared_ptr<PI_DWA<TE,TS> > Ptr;
    static PI_DWA<TE,TS>::Ptr Create(){ return std::make_shared< PI_DWA<TE,TS>  >() ; }

    typedef PlannerTraj<TE, TS> TB;
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



    PI_DWA()
    {

    }

    int GetNumberNodes()
    {
        return std::max(config_.expanderConfig_.firstLevelSplits,config_.expanderConfig_.numSplits)+1;
    }

    cv::Point2f Plan()
    {        

        config_.plannerConfig_.maxLevel = 1;
        TrajNode *startNode = GetStartNode();


        const int numSplits = expander_.Expand(0,startNode->endCmd_,tempCmds_);

        for (int i = 0; i < numSplits;++i)
        {
            CreateTrajectory(startNode,tempCmds_[i]);

        }

        FinishedPlanning();


        return cv::Point2f(0,0);

    }


};

#endif // PI_ASTAR_H
