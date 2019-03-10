#ifndef PLANNER_SCORERDT
#define PLANNER_SCORERDT


#include "plannerutils_dt.h"
#include <memory>
#include <config_dtplanner.h>
#include "utils_math_approx.h"

#include "poseevalresults_dt.h"



/**
 * @brief Base scorer for different scorer implementations
 */
struct NodeScorer_BaseDT
{
    void SetConfig(const PlannerScorerConfigDT &config, float poseTimeStep)
    {
        config_ = config;
        poseTimeStep_ = poseTimeStep;
    }

    void SetConfig(const PlannerScorerConfigDT &config)
    {
        config_ = config;

    }

    void SetLastCmdVel(const cv::Point2f &lastCmdVel)
    {
        lastCmdVel_ = lastCmdVel;

    }

    float poseTimeStep_;

    PlannerScorerConfigDT config_;
    cv::Point3f goal_;
    std::vector<cv::Point3f> path_;
    std::vector<cv::Point2f> path2_;

    float goalDistanceCutoff_;
    cv::Point3f curRobotPose_;
    cv::Point2f lastCmdVel_;

};



/**
 * @brief Scorer that includes a goal position
 */
struct NodeScorer_GoalDT : public NodeScorer_BaseDT
{
    static constexpr const char* const NS_NAME = "goal_scorer";

    /*
     * Score[0] = sumGravAngle
     * Score[1] = maxGravAngle
     * Score[2] = sumAngleDiff
     * Score[3] = maxAngleDiff
     * Score[4] = sumTipAngle
     * Score[5] = maxTipAngle
     * Score[6] = poseCount
     * Score[7] = angularVelChange
     * Score[8] = meanWheelSupport
     * Score[9] = minWheelSupport
     *
     * scores[10] = 0; Not visible counter
        scores[11] = 0; GoalDistance
        scores[12] = 0; Orientation to goal
        scores[13] = 0;
        scores[14] = 0;
        scores[15] = 0;
     */

    void SetRobotPose(cv::Point3f robotPose, float goalDistanceCutoff)
    {
        curRobotPose_ = robotPose;
        goalDistanceCutoff_ = goalDistanceCutoff;
    }

    void SetGoal(const cv::Point3f goal)
    {
        goal_ = goal;

    }

    void SetPath(const std::vector<cv::Point3f> &path)
    {
        if (path.empty())
        {
            path_.clear();
            path2_.clear();
            return;

        }
        goal_ = path[path.size()-1];

        path_.clear();
        path2_.clear();



    }


    inline void CheckGoal(PoseEvalResultsDT &results) const
    {
        const cv::Point2f goalDiff(goal_.x- results.pose.x,goal_.y- results.pose.y);
        if ((goalDiff.dot(goalDiff)) < config_.targetGoalDistanceImageSqr) results.validState = PERSDT_GOALREACHED;

    }

    inline void ResetScores(std::array<float, NUMBERSCORES> &scores) const
    {

        scores[0] = 0;
        scores[1] = 0;
        scores[2] = 9999;
        scores[3] = 9999;
        scores[4] = 0;
        scores[5] = 0;


        scores[6] = 0;
        scores[7] = 0;
        scores[8] = 0;
        scores[9] = 0;

        scores[10] = 0;
        scores[11] = 0;
        scores[12] = 0;
        scores[13] = 0;
        scores[14] = 0;
        scores[15] = 0;

    }



    inline void ScorePose(const PoseEvalResultsDT& results, std::array<float, NUMBERSCORES> &scores) const
    {

        const float meanD = (config_.dontCareDistanceImg - results.meanDist)*config_.dontCareDistanceImgInv;
        const float meanDSqr = meanD*meanD;
        const float minD = (config_.dontCareDistanceImg - results.minDist)*config_.dontCareDistanceImgInv;
        const float minDSqr = minD*minD;

        scores[0] += meanDSqr;
        scores[1] += minDSqr;

        scores[2] = std::min(scores[2],meanDSqr);
        scores[3] = std::min(scores[3],minDSqr);

        scores[6] += 1.0f;



    }

    inline bool CheckPose(PoseEvalResultsDT &results)
    {

        if (results.validState == PERSDT_COLLISION)
        {

            return false;
        }


        if (results.validState == PERSDT_OUTOFIMAGE)
        {
            return false;
        }

        if (results.minDist < config_.distanceThresholdImg)
        {
            results.validState = PERSDT_COLLISION;
            return false;
        }

        const cv::Point2f goalDiff(goal_.x- results.pose.x,goal_.y- results.pose.y);

        if ((goalDiff.dot(goalDiff)) < config_.targetGoalDistanceImageSqr)
        {
            results.validState = PERSDT_GOALREACHED;
            return false;
        }

        return true;

    }


    inline void ScoreNode(TrajNodeDT &current)  const
    {
        float velDiff = 0;
        if (current.parent_ != nullptr) velDiff = std::abs(current.endCmd_.y-current.parent_->endCmd_.y);
        current.scores[7] += velDiff;

        /*
        const cv::Point3f end = current.end_->pose;

        const cv::Point3f diff = goal_ - end;
        const cv::Point2f diffPos(diff.x,diff.y);
        const float dist = sqrt(diffPos.dot(diffPos));
        const float angleToGoal = atan2(diffPos.y,diffPos.x);
        const float angleDiff = fabs(end.z -angleToGoal);

        current.scores[11] = dist;
        current.scores[12] = angleDiff;
        */

        //return 0;
    }

    inline float GetAngleDifference(const float &a, const float &b) const
    {
        float res = a-b;
        if (res > CV_PIF) res = res - CV_2PIF;
        if (res < -CV_PIF) res = res + CV_2PIF;
        return std::abs(res);
    }

    inline void CalcGoalDistance(TrajNodeDT &current) const
    {
        const cv::Point2f rPos(curRobotPose_.x,curRobotPose_.y);
        const cv::Point2f gPos(goal_.x,goal_.y);

        const cv::Point2f robot2GoalVec = gPos- rPos;
        const float robot2GoalDistance = sqrt(robot2GoalVec.dot(robot2GoalVec));

        const cv::Point3f end = current.end_->pose;

        const cv::Point3f diff = goal_ - end;
        const cv::Point2f diffPos(diff.x,diff.y);
        const float dist = sqrt(diffPos.dot(diffPos));

        float distDiff = robot2GoalDistance - dist;
        if (distDiff > goalDistanceCutoff_) distDiff = goalDistanceCutoff_;

        const float distVal = distDiff/goalDistanceCutoff_;

        const float angleToGoal = atan2(diffPos.y,diffPos.x);
        //const float angleDiff = std::abs(end.z -angleToGoal);
        const float angleDiff = std::abs(GetAngleDifference(end.z ,angleToGoal));

        current.scores[11] = distVal;
        current.scores[12] = angleDiff;

    }

    inline void CalcGoalDistanceTest(TrajNodeDT &current) const
    {

        const cv::Point3f end = current.end_->pose;

        const cv::Point3f diff = goal_ - end;
        const cv::Point2f diffPos(diff.x,diff.y);
        float dist = sqrt(diffPos.dot(diffPos));

        if (dist > goalDistanceCutoff_) dist = goalDistanceCutoff_;

        const float distVal = dist/goalDistanceCutoff_;

        const float angleToGoal = atan2(diffPos.y,diffPos.x);
        //const float angleDiff = std::abs(end.z -angleToGoal);
        const float angleDiff = std::abs(GetAngleDifference(end.z ,angleToGoal));

        current.scores[11] = distVal;
        current.scores[12] = angleDiff;

    }

    inline float GetLastCmdVelDiff(TrajNodeDT &current)  const
    {
        const TrajNodeDT* tnPtr = current.GetFirstNode();

        const cv::Point2f curCmdVel = tnPtr->startCmd_;

        //return std::abs(curCmdVel.x - lastCmdVel_.x) + std::abs(curCmdVel.y - lastCmdVel_.y);
        return std::abs(curCmdVel.y - lastCmdVel_.y);


    }

    inline void FinalNodeScore(TrajNodeDT &current)  const
    {
        float endFactor = 1.0;

        switch (current.end_->validState)
        {
        case PERSDT_OUTOFIMAGE: endFactor = config_.end_outOfImage; break;
        case PERSDT_COLLISION: endFactor = config_.end_collision; break;
        case PERSDT_VALID: endFactor = config_.end_valid; break;
        case PERSDT_GOALREACHED: endFactor = config_.end_goalReached; break;

        default: endFactor = config_.end_valid;

        }

        CalcGoalDistance(current);

        float normalize =  current.scores[6];
        if (normalize == 0) normalize = 1.0;
        else normalize = 1.0f/normalize;

        float lowPoseCountPenalty = 0;
        if (current.scores[6]*poseTimeStep_ < config_.minPoseTime && current.end_->validState != PERSDT_GOALREACHED)
        {
            lowPoseCountPenalty = config_.end_poseCountLowPenalty;
        }
        float lastCmdVelDiff = GetLastCmdVelDiff(current);

        float minDistScore = current.scores[3];

        if (minDistScore > config_.dontCareDistanceImg) minDistScore = config_.dontCareDistanceImg;

        float levelNorm = 1.0f;
        if (current.level_ != 0) levelNorm = 1.0f/(float)current.level_;


        current.fScore_ =
                current.scores[0]*normalize*config_.f_meanMeanDist +
                current.scores[1]*normalize*config_.f_meanMinDist +
                current.scores[2]*config_.f_minMeanDist +
                current.scores[3]*config_.f_minMinDist +
                current.scores[6]*config_.f_poseC +
                (current.scores[7]*levelNorm)*config_.f_aVelD +
                current.scores[11]*config_.f_goalDistance+
                current.scores[12]*config_.f_goalOrientation+
                (current.scores[13]*levelNorm)*config_.f_pathDistance+
                lastCmdVelDiff * config_.f_lastCmdVelDiff +
                lowPoseCountPenalty+
                endFactor;


        current.finalScores[0] = current.scores[0]*normalize*config_.f_meanMeanDist;
        current.finalScores[1] = current.scores[1]*normalize*config_.f_meanMinDist;
        current.finalScores[2] = current.scores[2]*config_.f_minMeanDist;
        current.finalScores[3] = current.scores[3]*config_.f_minMinDist;
        current.finalScores[6] = current.scores[6]*config_.f_poseC;
        current.finalScores[7] = current.scores[7]*config_.f_aVelD;
        current.finalScores[9] = lastCmdVelDiff * config_.f_lastCmdVelDiff;
        current.finalScores[11] = current.scores[11]*config_.f_goalDistance;
        current.finalScores[12] = current.scores[12]*config_.f_goalOrientation;
        current.finalScores[13] = (current.scores[13]*levelNorm)*config_.f_pathDistance;
        current.finalScores[14] = endFactor;
        current.finalScores[15] = lowPoseCountPenalty;
        //current.finalScores[15] = lastCmdVelDiff * config_.f_lastCmdVelDiff ;


        TrajNodeDT* parPtr = current.GetFirstNode();
        //parPtr->bestChildScore_ = std::max(parPtr->bestChildScore_,current.fScore_);
        if (current.fScore_ > parPtr->bestChildScore_)
        {
            parPtr->bestChildScore_ = current.fScore_;
            parPtr->bestChild_ = &current;
            parPtr->fScore_ = current.fScore_;
        }
        if (current.validState_ > PERSDT_COLLISION)
        {
            parPtr->validChildCount_++;


        }

    }


    TrajNodeDT* CheckAllNodes(std::vector<TrajNodeDT> &allNodes, int nodeCounter)
    {

        float bestScore = -99999999;
        TrajNodeDT* resPtr = nullptr;

        if (config_.f_childCount <= 0) return resPtr;

        for (int i = 0; i < nodeCounter;++i)
        {
            TrajNodeDT* current = &allNodes[i];
            if (current->level_ != 1) continue;

            float tscore;
            if (current->bestChild_->end_->validState != PERSDT_GOALREACHED)
            {
                tscore = current->bestChildScore_ + (float)current->validChildCount_*config_.f_childCount;

                current->scores[10] = (float)current->validChildCount_;
                current->finalScores[10] = (float)current->validChildCount_*config_.f_childCount;
                current->fScore_ = tscore;
            }
            else
            {
                tscore = current->bestChildScore_;
                current->scores[10] = (float)current->validChildCount_;
                current->finalScores[10] = -1.0;

            }

            if (tscore > bestScore)
            {
                bestScore = tscore;
                resPtr = current->bestChild_;
            }


        }

        return resPtr;

    }


};



struct NodeScorer_PathDT : public NodeScorer_GoalDT
{

    static constexpr const char* const NS_NAME = "path_scorer";


    void SetPath(const std::vector<cv::Point3f> &path)
    {
        if (path.empty())
        {
            path_.clear();
            path2_.clear();
            return;

        }
        path_ = path;
        goal_ = path[path.size()-1];

        path2_.clear();
        for (unsigned int tl = 0; tl < path.size();++tl )
        {
            path2_.push_back(cv::Point2f(path[tl].x,path[tl].y));
        }


    }


    inline float SqDistancePtSegment(const cv::Point2f &a, const cv::Point2f &b, const cv::Point2f &p ) const
    {
        const cv::Point2f n = b - a;
        const cv::Point2f pa = a - p;

        const float c = n.dot( pa );

        // Closest point is a
        if ( c > 0.0f )
            return pa.dot(  pa );

        const cv::Point2f bp = p - b;

        // Closest point is b
        if ( n.dot( bp ) > 0.0f )
            return bp.dot( bp );

        // Closest point is between a and b
        const cv::Point2f e = pa - n * (c / n.dot( n ));

        return e.dot( e );
    }

    inline float GetMinPathDistance(const cv::Point3f p3)const
    {
        if (path2_.empty()) return 0;

        cv::Point2f p(p3.x,p3.y);

        float curDis = 99999999999.0f;

        for (unsigned int tl = 1; tl < path2_.size();++tl)
        {
            float tdis = SqDistancePtSegment(path2_[tl-1],path2_[tl],p);
            if (tdis < curDis) curDis = tdis;
        }
        return sqrt(curDis);

    }

    inline void ScoreNode(TrajNodeDT &current)  const
    {
        float velDiff = 0;
        if (current.parent_ != nullptr) velDiff = std::abs(current.endCmd_.y);
        current.scores[7] += velDiff;

        current.scores[13] += GetMinPathDistance( current.end_->pose);

        /*
        const cv::Point3f end = current.end_->pose;

        const cv::Point3f diff = goal_ - end;
        const cv::Point2f diffPos(diff.x,diff.y);
        const float dist = sqrt(diffPos.dot(diffPos));
        const float angleToGoal = atan2(diffPos.y,diffPos.x);
        const float angleDiff = fabs(end.z -angleToGoal);

        current.scores[11] = dist;
        current.scores[12] = angleDiff;
        */

        //return 0;
    }

};

struct NodeScorer_PathNGDT : public NodeScorer_PathDT
{

    static constexpr const char* const NS_NAME = "ngpath_scorer";


    inline bool CheckPose(PoseEvalResultsDT &results)
    {


        if (results.validState == PERSDT_COLLISION)
        {

            return false;
        }


        if (results.validState == PERSDT_OUTOFIMAGE)
        {
            return false;
        }

        return true;

    }

};


#endif // PLANNER_SCORER

