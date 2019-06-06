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


    /**
     * @brief using for accessing the nondependent base members
     */
    typedef PlannerTraj< TS> TB;
    using TB::openSet_;
    using TB::config_;
    using TB::CreateTrajectory;
    using TB::bestScore_;
    using TB::bestNode_;
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
    using TB::GetBLResultTrajectory;



    PI_AStar()
    {

    }

    /**
     * @brief Return max number of search iterations
     */
    int GetNumberNodes()
    {
        return config_.plannerConfig_.maxSearchIterations;
    }

    /**
     * @brief Main A* iteration loop
     */
    void IterateStar(TrajNode* start)
    {

        openSet_.push(start);



        for (int tl = 0; tl < config_.plannerConfig_.maxSearchIterations;++tl)
        {
            if (openSet_.empty()) break;
            TrajNode* curNode = openSet_.top();
            openSet_.pop();

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

        /// TODO: test if no sub optimal paths are chosen
        /*
        while (!openSet_.empty())
        {
            TrajNode* curNode = openSet_.top();
            openSet_.pop();

            //leaves_.push_back(&out);
            if (curNode->fScore_ > bestScore_)
            {
                bestNode_ = curNode;
                bestScore_ = curNode->fScore_;

            }
        }
        */

    }

    /**
     * @brief If plan failed replan is called when replanning is enabled
     */
    void Replan()
    {
        Trajectory *result = GetBLResultTrajectory();
        bool doReplan = false;
        if (result == nullptr)
        {
            doReplan = true;
        }
        else if (result->end_ == nullptr)
        {
            doReplan = true;
        }
        else if (result->poseResults_.size() < config_.plannerConfig_.minNumberNodes)
        {
            bool reachedGoal = result->end_->validState == PERS_GOALREACHED;

            if (reachedGoal)
            {
                doReplan = false;
            } else
            {
                doReplan = true;
            }

        }

        if (!doReplan) return;

        /*
         * Create new configs with smaller stepSizes
         */

        PlannerExpanderConfig curConfig = config_.expanderConfig_;
        PlannerExpanderConfig newConfig = config_.expanderConfig_;

        newConfig.deltaTheta = (curConfig.firstLevelSplits > 0 ? curConfig.firstLevelDeltaTheta: curConfig.deltaTheta )/(float)config_.plannerConfig_.replanFactor;
        newConfig.numSplits =  (curConfig.firstLevelSplits > 0 ? curConfig.firstLevelSplits: curConfig.numSplits )*config_.plannerConfig_.replanFactor;
        newConfig.firstLevelDeltaLinear = -1;
        newConfig.firstLevelDeltaTheta = -1;
        newConfig.firstLevelLinearSplits = -1;
        newConfig.firstLevelSplits = -1;
        expander_->SetConfig(newConfig,config_.procConfig_.pixelSize);

        if ( ((float) newConfig.numSplits * newConfig.deltaTheta)/2.0f > config_.expanderConfig_.maxAngVel)
        {
            int oneSide =  (int)std::ceil(config_.expanderConfig_.maxAngVel / newConfig.deltaTheta);
            newConfig.numSplits = oneSide*2+1;
        }

        ClearPrioQueue();

        TrajNode *startNode = GetStartNode();

#ifdef USE_CLOSED_SET
        closedSet_.Setup(config_.plannerConfig_.maxLevel,1.0,0.0174533);
#endif
        IterateStar(startNode);

        FinishedPlanning();

        expander_->SetConfig(curConfig,config_.procConfig_.pixelSize);

    }

    /**
     * @brief Main planning function
     */
    cv::Point2f Plan()
    {

        ClearPrioQueue();

        TrajNode *startNode = GetStartNode();

#ifdef USE_CLOSED_SET
        closedSet_.Setup(config_.plannerConfig_.maxLevel,1.0,0.0174533);
#endif
        IterateStar(startNode);

        FinishedPlanning();


        /// Testing replan with finer resolution if planning fails
        if (config_.plannerConfig_.replanFactor > 0 && config_.plannerConfig_.minNumberNodes > 0)
        {
              Replan();

        }


        return cv::Point2f(0,0);

    }

#ifdef USE_CLOSED_SET
    ClosedSet closedSet_;
#endif

};

#endif // PI_ASTAR_H
