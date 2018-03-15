#ifndef PLANNERNODES_H
#define PLANNERNODES_H


#include "plannerbase.h"
#include "planner_nodeexpander.h"

#include <set>
#include <queue>

/**
 * @brief Planner base class with templates for expander and scorer
 */
template <typename TE, typename TS>
class PlannerTraj : public PlannerBase
{
public:

    //typedef std::shared_ptr<PlannerTraj> Ptr;

    virtual void Initialize(ModelBasedPlannerConfig &config)
    {
        PlannerBase::Initialize(config);
        expander_.SetConfig(config_.expanderConfig_,config.procConfig_.pixelSize);
        scorer_.SetConfig(config_.scorerConfig_, config_.procConfig_.validThreshold,config_.procConfig_.notVisibleThreshold, config_.plannerConfig_.subSampleTimeStep);
    }

    void SetPlannerParameters(PlannerConfig &config)
    {
        config_.plannerConfig_ = config;
    }

    void SetPlannerScorerParameters(PlannerScorerConfig &config)
    {
        config_.scorerConfig_ = config;
        scorer_.SetConfig(config_.scorerConfig_);
    }

    void SetPlannerExpanderParameters(PlannerExpanderConfig &config)
    {
        config_.expanderConfig_ = config;
        expander_.SetConfig(config_.expanderConfig_,config_.procConfig_.pixelSize);
    }

    virtual void SetGoalMap(const cv::Point3f goal)
    {
        goal_ = PoseToImgPose(goal);
        scorer_.SetGoal(goal_);
    }

    void FinishedPlanning()
    {
        TrajNode* bestNodeParent = bestNode_->GetFirstNode();

        scorer_.SetLastCmdVel(bestNodeParent->startCmd_);

    }



    inline TrajNode* GetStartNode()
    {
        curNodeIdx_ = 0;
        bestScore_ = -99999999;

        //poseEstimator_.ResetPoseCounter();

        TrajNode *startNode = GetNextNode();

        startNode->fScore_ = 0;

        startNode->validState_ = TN_VS_VALID;
        startNode->level_ = 0;
        startNode->parent_ = nullptr;
        startNode->startCmd_ = curImgVelocity_;
        startNode->endCmd_ = curImgVelocity_;
        startNode->time_ = 0;
        //startNode->poseResults_.resize(1);
        startNode->poseResults_[0].pose = curImgRobotPose_;
        startNode->start_ = &startNode->poseResults_[0];
        startNode->SetEnd(0);
        //startNode->Reset();

        scorer_.SetRobotPose(curImgRobotPose_,curImgVelocity_.x*config_.plannerConfig_.lookAheadTime);
        scorer_.SetLastCmdVel(curImgVelocity_);
        return startNode;

    }


    inline bool NextNodeAvailable() const
    {
        return curNodeIdx_<allNodes_.size();
    }

    inline TrajNode* GetNextNode()
    {
        if ((unsigned int)curNodeIdx_ >= allNodes_.size()) return nullptr;
        TrajNode* res = &allNodes_[curNodeIdx_++];
        res->Reset();
        scorer_.ResetScores(res->scores);
        return res;
    }


    TrajNode* CreateTrajectory(TrajNode* prev, const cv::Point2f &cmd)
    {

        float curStep = config_.plannerConfig_.subSampleTimeStep;

        cv::Point3f curP = prev->end_->pose;

        TrajNode &out = *GetNextNode();


        out.SetParent(prev);

        out.time_ = config_.plannerConfig_.trajectoryTimeStep;
        out.startCmd_ = cmd;
        out.endCmd_ = cmd;
        out.validState_ = TN_VS_VALID;


        PoseEvalResults *prevPER = prev->end_;

        cv::Vec4f wheelAnglesRobot = poseEstimator_.robotModel_.GetWheelAnglesRobot(cmd);

        int tl = 0;
        for (tl = 0; tl < config_.plannerConfig_.numSubSamples;++tl)
        {
            PoseEvalResults &results = out.poseResults_[tl];
            results.SetWheelAnglesRobot(wheelAnglesRobot);

            DriveModelDA::UpdatePose(curP,cmd*curStep,results.pose);
            curStep+=config_.plannerConfig_.subSampleTimeStep;
            results.cmd = cmd;
            poseEstimator_.Evaluate(results);
            //poseEstimator_->Evaluate(out.poseResults_[tl]);
            CalculateAngleDiff(*prevPER,results);
            //poseEstimator_.CheckState(results);

            /// Testing: inlcude final node
            //scorer_.ScorePose(*prevPER,out.poseResults_[tl],out.scores);
            //scorer_.CheckGoal(results);
            if (!scorer_.CheckPose(results))
            {
                tl++;
                out.validState_ = TN_VS_NOTVALIDUNTILEND;
                break;
            }

            scorer_.ScorePose(out.poseResults_[tl],out.scores);


            prevPER = &out.poseResults_[tl];

        }

        out.SetEnd(tl > 0?tl-1:0);
        scorer_.ScoreNode(out);
        //scorer_.FinalNodeScore(out);

        if (out.validState_ == TN_VS_NOTVALIDUNTILEND || out.level_ >= config_.plannerConfig_.maxLevel)
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
    TE expander_;

};

#endif // PLANNERNODES_H
