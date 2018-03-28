#ifndef PI_ASTAR_H
#define PI_ASTAR_H


#include "plannertraj.h"
#ifdef USE_CLOSED_SET
#include "closedset.h"
#endif


/**
 * @brief Implementation of the A*-like planner. The closed set can be disabled of the expander parameters do not produce nodes that are close enough. This saves some computation time
 */
template <typename TS>
class PI_AStar : public PlannerTraj<TS>
{
public:
    typedef std::shared_ptr<PI_AStar<TS> > Ptr;
    static PI_AStar<TS>::Ptr Create(){ return std::make_shared< PI_AStar<TS>  >() ; }

    typedef PlannerTraj< TS> TB;
    using TB::openSet_;
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
    using TB::NextNodeAvailable;
    using TB::ClearPrioQueue;



    PI_AStar()
    {

    }

    int GetNumberNodes()
    {
        return config_.plannerConfig_.maxSearchIterations;
    }

    void IterateStar(TrajNode* start)
    {

        openSet_.push(start);



        for (int tl = 0; tl < config_.plannerConfig_.maxSearchIterations;++tl)
        {
            if (openSet_.empty()) break;
            TrajNode* curNode = openSet_.top();
            openSet_.pop();

            //const int numSplits = expander_.GetNumberChildren(curNode->level_);

            const int numSplits = expander_->Expand(curNode->level_, curNode->endCmd_,tempCmds_);

            for (int i = 0; i < numSplits;++i)
            {
                if (!NextNodeAvailable()) return;
                TrajNode* newNode = CreateTrajectory(curNode,tempCmds_[i]);
                scorer_.FinalNodeScore(*newNode);

#ifdef USE_CLOSED_SET
                if (newNode->validState_ >= 0 && newNode->level_ < config_.plannerConfig_.maxLevel && !closedSet_.Test(newNode->level_,newNode->end_->pose)) openSet_.push(newNode);
#else
                if (newNode->validState_ >= 0 && newNode->level_ < config_.plannerConfig_.maxLevel) openSet_.push(newNode);

#endif

            }


        }

    }


    cv::Point2f Plan()
    {

        ClearPrioQueue();

        TrajNode *startNode = GetStartNode();

#ifdef USE_CLOSED_SET
        closedSet_.Setup(config_.plannerConfig_.maxLevel,1.0,0.0174533);
#endif
        IterateStar(startNode);

        FinishedPlanning();


        return cv::Point2f(0,0);

    }

#ifdef USE_CLOSED_SET
    ClosedSet closedSet_;
#endif

};

#endif // PI_ASTAR_H
