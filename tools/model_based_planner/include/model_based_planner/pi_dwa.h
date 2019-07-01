#ifndef PI_DWA_H
#define PI_DWA_H


#include "plannertraj.h"



/**
 * @brief DWA implementation
 */
template < typename TS>
class PI_DWA : public PlannerTraj< TS>
{
public:
    typedef std::shared_ptr<PI_DWA<TS> > Ptr;
    static PI_DWA<TS>::Ptr Create(){ return std::make_shared< PI_DWA<TS>  >() ; }

    /**
     * @brief using for accessing the nondependent base members
     */
    typedef PlannerTraj<TS> TB;
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

    /**
     * @brief Return max number of search iterations
     */
    int GetNumberNodes()
    {
        return std::max(config_.expanderConfig_.firstLevelSplits,config_.expanderConfig_.numSplits)+1;
    }

    /**
     * @brief Perform the planning. It creates a set of new command velocities and uses the one with the highest score.
     */
    cv::Point2f Plan()
    {        

        config_.plannerConfig_.maxLevel = 1;
        TrajNode *startNode = GetStartNode();


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
