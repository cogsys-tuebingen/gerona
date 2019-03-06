#ifndef PLANNERNODESDT_H
#define PLANNERNODESDT_H


#include "plannerbase_dt.h"
#include "planner_nodeexpander.h"

#include <set>
#include <queue>

/**
 * @brief Planner base class with templates for expander and scorer
 */
template <typename TS>
class PlannerTrajDT : public PlannerBaseDT
{
public:

    //typedef std::shared_ptr<PlannerTraj> Ptr;

    void CreateNodeExpander(std::string expanderName)
    {
        if (expanderName == NodeExpander_AVNI::NE_NAME)
        {
            expander_ = NodeExpander_AVNI::Create();
            return;
        }
        if (expanderName == NodeExpander_AVT::NE_NAME)
        {
            expander_ = NodeExpander_AVT::Create();
            return;
        }
        if (expanderName == NodeExpander_LAVT::NE_NAME)
        {
            expander_ = NodeExpander_LAVT::Create();
            return;
        }
        if (expanderName == NodeExpander_AVLR::NE_NAME)
        {
            expander_ = NodeExpander_AVLR::Create();
            return;
        }

        expander_ = NodeExpander_AVNI::Create();

    }

    virtual void Initialize(DTPlannerConfig &config)
    {
        PlannerBaseDT::Initialize(config);
        CreateNodeExpander(config_.nodeExpanderType_);

        expander_->SetConfig(config_.expanderConfig_,config_.procConfig_.pixelSize);
        scorer_.SetConfig(config_.scorerConfig_, config_.plannerConfig_.subSampleTimeStep);
    }

    void SetPlannerParameters(PlannerConfig &config)
    {
        config_.plannerConfig_ = config;
    }

    void SetPlannerScorerParameters(PlannerScorerConfigDT &config)
    {
        config_.scorerConfig_ = config;
        scorer_.SetConfig(config_.scorerConfig_);
    }

    void SetPlannerExpanderParameters(PlannerExpanderConfig &config)
    {
        config_.expanderConfig_ = config;
        expander_->SetConfig(config_.expanderConfig_,config_.procConfig_.pixelSize);
    }

    virtual void SetGoalMap(const cv::Point3f goal)
    {
        goal_ = PoseToImgPose(goal);
        scorer_.SetGoal(goal_);
    }

    virtual void SetPathMap(const std::vector<cv::Point3f> &path)
    {

        path_.clear();
        path_.reserve(path.size());

        for (unsigned int tl = 0; tl < path.size();++tl)
        {
            path_.push_back(PoseToImgPose(path[tl]));
        }

        scorer_.SetPath(path_);

    }

    void FinishedPlanning()
    {
        if (bestNode_ == nullptr) return;

        TrajNodeDT* bestNodeParent = bestNode_->GetFirstNode();

        TrajNodeDT* bestNodeWithChilds = scorer_.CheckAllNodes(allNodes_, curNodeIdx_);
        if (bestNodeWithChilds != nullptr)
        {
            bestNode_ = bestNodeWithChilds;
            bestNodeParent = bestNodeWithChilds->GetFirstNode();

        }

        if (bestNodeParent == nullptr) return;

        scorer_.SetLastCmdVel(bestNodeParent->startCmd_);

    }



    inline TrajNodeDT* GetStartNode()
    {
        poseEstimator_.CreateMap(curImgRobotPose_,scorer_.config_);

        curNodeIdx_ = 0;
        bestScore_ = -99999999;

        //poseEstimator_.ResetPoseCounter();

        TrajNodeDT *startNode = GetNextNode();

        startNode->fScore_ = 0;

        startNode->validState_ = TNDT_VS_VALID;
        startNode->level_ = 0;
        startNode->parent_ = nullptr;
        startNode->startCmd_ = curImgVelocity_;
        startNode->endCmd_ = curImgVelocity_;
        startNode->time_ = 0;
        //startNode->poseResults_.resize(1);
        startNode->poseResults_[0].pose = curImgRobotPose_;
        startNode->start_ = &startNode->poseResults_[0];
        startNode->SetEnd(0);
        //SetWheelAnglesStart(startNode);
        //startNode->Reset();
        poseEstimator_.Evaluate(startNode->poseResults_[0]);

        scorer_.SetRobotPose(curImgRobotPose_,config_.procConfig_.pixelSizeInv* config_.expanderConfig_.maxLinVel*config_.plannerConfig_.lookAheadTime);
        scorer_.SetLastCmdVel(curImgVelocity_);
        return startNode;

    }


    inline bool NextNodeAvailable() const
    {
        return (unsigned int)curNodeIdx_ < allNodes_.size();
    }

    inline TrajNodeDT* GetNextNode()
    {
        if ((unsigned int)curNodeIdx_ >= allNodes_.size()) return nullptr;
        TrajNodeDT* res = &allNodes_[curNodeIdx_++];
        res->Reset();
        scorer_.ResetScores(res->scores);
        return res;
    }


    TrajNodeDT* CreateTrajectory(TrajNodeDT* prev, const cv::Point2f &cmd)
    {

        float curStep = config_.plannerConfig_.subSampleTimeStep;

        cv::Point3f curP = prev->end_->pose;

        TrajNodeDT &out = *GetNextNode();


        out.SetParent(prev);

        out.time_ = config_.plannerConfig_.trajectoryTimeStep;
        out.startCmd_ = cmd;
        out.endCmd_ = cmd;
        out.validState_ = TNDT_VS_VALID;


        PoseEvalResultsDT *prevPER = prev->end_;

        const float r = cmd.x/cmd.y;


        int tl = 0;
        for (tl = 0; tl < config_.plannerConfig_.numSubSamples;++tl)
        {
            PoseEvalResultsDT &results = out.poseResults_[tl];


            DriveModelDA::UpdatePose(curP,cmd*curStep, r,results.pose);
            curStep+=config_.plannerConfig_.subSampleTimeStep;
            results.cmd = cmd;
            poseEstimator_.Evaluate(results);
            //poseEstimator_->Evaluate(out.poseResults_[tl]);

            //poseEstimator_.CheckState(results);

            /// Testing: inlcude final node
            //scorer_.ScorePose(*prevPER,out.poseResults_[tl],out.scores);
            //scorer_.CheckGoal(results);
            if (!scorer_.CheckPose(results))
            {
                tl++;
                out.validState_ = TNDT_VS_NOTVALIDUNTILEND;
                break;
            }

            scorer_.ScorePose(out.poseResults_[tl],out.scores);


            prevPER = &out.poseResults_[tl];

        }

        out.SetEnd(tl > 0?tl-1:0);
        scorer_.ScoreNode(out);
        //scorer_.FinalNodeScore(out);

        if (out.validState_ == TNDT_VS_NOTVALIDUNTILEND || out.level_ >= config_.plannerConfig_.maxLevel)
        {

            scorer_.FinalNodeScore(out);

            //leaves_.push_back(&out);
            if (out.fScore_ > bestScore_)
            {
                bestNode_ = &out;
                bestScore_ = out.fScore_;

            }
        }
        return &out;
    }


protected:
    TS scorer_;
    INodeExpander::Ptr expander_;

};

#endif // PLANNERNODES_H
