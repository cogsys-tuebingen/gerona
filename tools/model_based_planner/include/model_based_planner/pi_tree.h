#ifndef PI_TREE_H
#define PI_TREE_H


#include "plannertraj.h"

//#define USE_CLOSED_SET 1
#ifdef USE_CLOSED_SET
#include "closedset.h"
#endif



/**
 * @brief Depth first search implementation, The closed set can be disabled of the expander parameters do not produce nodes that are close enough. This saves some computation time.
 */
template <typename TS>
class PI_Tree : public PlannerTraj< TS>
{
public:
    typedef std::shared_ptr<PI_Tree<TS> > Ptr;
    static PI_Tree<TS>::Ptr Create(){ return std::make_shared< PI_Tree<TS>  >() ; }

    typedef PlannerTraj< TS> TB;
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


    PI_Tree()
    {

    }


    /**
     * @brief Return max number of search iterations
     */
    int GetNumberNodes()
    {
        int nodeCount = 1;
        int lastLevel = 1;
        int curLevel = 1;

        for (int tl = 0; tl <= config_.plannerConfig_.maxLevel;tl++)
        {
            if (tl == 0 && config_.expanderConfig_.firstLevelSplits > 0)
            {

                curLevel = config_.expanderConfig_.firstLevelSplits;
                lastLevel = curLevel*lastLevel;
                nodeCount = nodeCount + lastLevel;
            }
            else
            {
                curLevel = config_.expanderConfig_.numSplits;
                lastLevel = curLevel*lastLevel;
                nodeCount = nodeCount + lastLevel;
            }


        }

        return nodeCount;

    }




    /**
     * @brief Recursive function for tree iteration
     */
    void IterateTree(TrajNode* start)
    {

        if (start->level_ >= config_.plannerConfig_.maxLevel) return;
        if (start->validState_ < 0) return;
#ifdef USE_CLOSED_SET
        if (closedSet_.Test(start->level_,start->end_->pose)) return;
#endif

        const int numSplits = expander_->Expand(start->level_,start->endCmd_,tempCmds_);

        std::vector<TrajNode*> newNodes(numSplits);


        for (int i = 0; i < numSplits;++i)
        {
            TrajNode* newNode = CreateTrajectory(start, tempCmds_[i]);

            newNodes[i] = newNode;



        }
        for (int i = 0; i < numSplits;++i)
        {
             IterateTree(newNodes[i]);


        }



    }

    /**
     * @brief Perform planning
     */
    cv::Point2f Plan()
    {

        TrajNode *startNode = GetStartNode();
#ifdef USE_CLOSED_SET
        closedSet_.Setup(config_.plannerConfig_.maxLevel,1.0,0.0174533);
#endif

        IterateTree(startNode);

        FinishedPlanning();

        return cv::Point2f(0,0);

    }


#ifdef USE_CLOSED_SET
    ClosedSet closedSet_;
#endif

};

#endif // PI_ASTAR_H
