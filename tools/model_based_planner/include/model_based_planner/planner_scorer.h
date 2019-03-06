#ifndef PLANNER_SCORER
#define PLANNER_SCORER


#include "plannerutils.h"
#include <memory>
#include <config_planner.h>
#include "utils_math_approx.h"



/**
 * @brief Base scorer for different scorer implementations
 */
struct NodeScorer_Base
{
    void SetConfig(const PlannerScorerConfig &config, float validTresh, float visibleThresh, float poseTimeStep)
    {
        config_ = config;
        validThreshold_ = validTresh;
        notVisibleThreshold_ = visibleThresh;
        poseTimeStep_ = poseTimeStep;
    }

    void SetConfig(const PlannerScorerConfig &config)
    {
        config_ = config;

    }

    void SetLastCmdVel(const cv::Point2f &lastCmdVel)
    {
        lastCmdVel_ = lastCmdVel;

    }

    float validThreshold_;
    float notVisibleThreshold_;
    float poseTimeStep_;

    PlannerScorerConfig config_;
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
struct NodeScorer_Goal_T : public NodeScorer_Base
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


    inline void CheckGoal(PoseEvalResults &results) const
    {
        const cv::Point2f goalDiff(goal_.x- results.pose.x,goal_.y- results.pose.y);
        if ((goalDiff.dot(goalDiff)) < config_.targetGoalDistanceImageSqr) results.validState = PERS_GOALREACHED;

    }

    inline void ResetScores(std::array<float, NUMBERSCORES> &scores) const
    {

        scores[0] = 0;
        scores[1] = 0;
        scores[2] = 0;
        scores[3] = 0;
        scores[4] = 0;
        scores[5] = 0;


        scores[6] = 0;
        scores[7] = 0;
        scores[8] = 0;
        scores[9] = 1;

        scores[10] = 0;
        scores[11] = 0;
        scores[12] = 0;
        scores[13] = 0;
        scores[14] = 0;
        scores[15] = 0;

    }



    inline void ScorePose(const PoseEvalResults& results, std::array<float, NUMBERSCORES> &scores) const
    {

        //const float gravAngle = acos(results.gravAngle);
        //const float tipAngle = acos(results.tipAngle);
        const float gravAngle = (results.gravAngle);
        const float tipAngle = (results.tipAngle);
        scores[0] += gravAngle;
        scores[1] = std::max(scores[1],gravAngle);

        scores[2] += results.deltaAngle;
        scores[3] = std::max(scores[3], results.deltaAngle);
        scores[4] += tipAngle;
        scores[5] = std::max(scores[5], tipAngle);

        scores[6] += 1.0f;

        const float minWheelSupport = results.GetMinWheelSupport();
        scores[8] += minWheelSupport;
        scores[9] = std::min(scores[9], minWheelSupport);

        scores[10] += results.validState == PERS_NOTVISIBLE ? 1.0f : 0.0f;

    }

    inline bool CheckPose(PoseEvalResults &results)
    {

        if (results.TestWheelZValues(validThreshold_))
        {
            //results.validState = PERS_NOWHEELSUPPORT;
            //if (results.poseCounter > config_.noWheelSupportNearThreshold) results.validState = PERS_LOWWHEELSUPPORT_FAR;

            results.validState = ((float)results.poseCounter*poseTimeStep_ > config_.noWheelSupportNearThreshold || (std::abs(results.pose.z - curRobotPose_.z) > config_.noWheelSupportRotateThreshold) ) ? PERS_LOWWHEELSUPPORT_FAR : PERS_NOWHEELSUPPORT;

            return false;
        }

        if (results.TestWheelZValues(notVisibleThreshold_))
        {
            results.validState = PERS_NOTVISIBLE;
            if (config_.allowNotVisible)
            {
                results.a1 = 0;
                results.a2 = 0;

                results.n1.x = 0;
                results.n1.y = 0;
                results.n1.z = 1;
                results.n2.x = 0;
                results.n2.y = 0;
                results.n2.z = 1;
                return true;
            }
            return false;

        }


        if (results.validState == PERS_OUTOFIMAGE)
        {
            return false;
        }


        if (results.gravAngle > config_.gravAngleThreshold)
        {
            results.validState = PERS_EXCEEDGRAVANGLE;
            return false;
        }
        if (results.tipAngle > config_.tipAngleThreshold)
        {
            results.validState = PERS_EXCEEDTIPANGLE;
            return false;
        }
        if (results.deltaAngle > config_.deltaAngleThreshold)
        {
            results.validState = PERS_EXCEEDDELTAANGLE;
            return false;
        }

        if (results.validState == PERS_CHASSISCOLLISION)
        {
            return false;
        }

        if (results.GetMinWheelSupport() < config_.minWheelSupportThreshold)
        {
            results.validState = (results.poseCounter*poseTimeStep_ > config_.noWheelSupportNearThreshold)?  PERS_LOWWHEELSUPPORT_FAR : PERS_LOWWHEELSUPPORT;

            return false;
        }


        const cv::Point2f goalDiff(goal_.x- results.pose.x,goal_.y- results.pose.y);

        if ((goalDiff.dot(goalDiff)) < config_.targetGoalDistanceImageSqr)
        {
            results.validState = PERS_GOALREACHED;
            return false;
        }

        return true;

    }


    inline void ScoreNode(TrajNode &current)  const
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

    inline void CalcGoalDistance(TrajNode &current) const
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

    inline float GetLastCmdVelDiff(TrajNode &current)  const
    {
        const TrajNode* tnPtr = current.GetFirstNode();

        const cv::Point2f curCmdVel = tnPtr->startCmd_;

        //return std::abs(curCmdVel.x - lastCmdVel_.x) + std::abs(curCmdVel.y - lastCmdVel_.y);
        return std::abs(curCmdVel.y - lastCmdVel_.y);


    }

    inline void FinalNodeScore(TrajNode &current)  const
    {
        float endFactor = 1.0;

        switch (current.end_->validState)
        {
        case PERS_OUTOFIMAGE: endFactor = config_.end_outOfImage; break;
        case PERS_NOTVISIBLE: endFactor = config_.end_notVisible; break;
        case PERS_NOWHEELSUPPORT: endFactor = config_.end_noWheelSupport; break;
        case PERS_LOWWHEELSUPPORT: endFactor = config_.end_noWheelSupport; break;
        case PERS_LOWWHEELSUPPORT_FAR: endFactor = config_.end_noWheelSupportFar; break;
        case PERS_EXCEEDGRAVANGLE: endFactor = config_.end_exceedAngle; break;
        case PERS_EXCEEDTIPANGLE: endFactor = config_.end_exceedAngle; break;
        case PERS_EXCEEDDELTAANGLE: endFactor = config_.end_exceedAngle; break;
        case PERS_CHASSISCOLLISION: endFactor = config_.end_chassisCollision; break;
        case PERS_VALID: endFactor = config_.end_valid; break;
        case PERS_GOALREACHED: endFactor = config_.end_goalReached; break;

        default: endFactor = config_.end_valid;

        }

        CalcGoalDistance(current);

        float normalize =  current.scores[6];
        if (normalize == 0) normalize = 1.0;
        else normalize = 1.0f/normalize;

        float lowPoseCountPenalty = 0;
        if (current.scores[6]*poseTimeStep_ < config_.minPoseTime && current.end_->validState != PERS_GOALREACHED)
        {
            lowPoseCountPenalty = config_.end_poseCountLowPenalty;
        }
        float lastCmdVelDiff = GetLastCmdVelDiff(current);

        float levelNorm = 1.0f;
        if (current.level_ != 0) levelNorm = 1.0f/(float)current.level_;

        current.fScore_ =
                (current.scores[0]*normalize)*config_.f_meanGA +
                current.scores[1]*config_.f_maxGA +
                (current.scores[2]*normalize)*config_.f_meanAD +
                current.scores[3]*config_.f_maxAD +
                (current.scores[4]*normalize)*config_.f_meanTA +
                current.scores[5]*config_.f_maxTA +
                current.scores[6]*config_.f_poseC +
                (current.scores[7]*levelNorm)*config_.f_aVelD +
                (current.scores[8]*normalize)*config_.f_meanWS +
                current.scores[9]*config_.f_minWS +
                current.scores[10]*config_.f_numNotVisible +
                current.scores[11]*config_.f_goalDistance+
                current.scores[12]*config_.f_goalOrientation+
                (current.scores[13]*levelNorm)*config_.f_pathDistance+
                lastCmdVelDiff * config_.f_lastCmdVelDiff +
                lowPoseCountPenalty+
                endFactor;


        current.finalScores[0] = (current.scores[0]*normalize)*config_.f_meanGA;
        current.finalScores[1] = current.scores[1]*config_.f_maxGA;
        current.finalScores[2] = (current.scores[2]*normalize)*config_.f_meanAD;
        current.finalScores[3] = current.scores[3]*config_.f_maxAD;
        current.finalScores[4] = (current.scores[4]*normalize)*config_.f_meanTA;
        current.finalScores[5] = current.scores[5]*config_.f_maxTA ;
        current.finalScores[6] = current.scores[6]*config_.f_poseC;
        current.finalScores[7] = current.scores[7]*config_.f_aVelD;
        current.finalScores[8] = current.scores[8]*normalize*config_.f_meanWS;
        current.finalScores[9] = current.scores[9]*config_.f_minWS;
        current.finalScores[10] = current.scores[10];
        current.finalScores[11] = current.scores[11]*config_.f_goalDistance;
        current.finalScores[12] = current.scores[12]*config_.f_goalOrientation;
        current.finalScores[13] = (current.scores[13]*levelNorm)*config_.f_pathDistance;
        current.finalScores[14] = endFactor;
        current.finalScores[15] = lowPoseCountPenalty;
        //current.finalScores[15] = lastCmdVelDiff * config_.f_lastCmdVelDiff ;

        TrajNode* parPtr = current.GetFirstNode();
        //parPtr->bestChildScore_ = std::max(parPtr->bestChildScore_,current.fScore_);
        if (current.fScore_ > parPtr->bestChildScore_)
        {
            parPtr->bestChildScore_ = current.fScore_;
            parPtr->bestChild_ = &current;
        }
        if (current.validState_ >= PERS_LOWWHEELSUPPORT_FAR) parPtr->validChildCount_++;

    }


    inline TrajNode* CheckAllNodes(std::vector<TrajNode> &allNodes, int nodeCounter)
    {

        float bestScore = -99999999;
        TrajNode* resPtr = nullptr;

        if (config_.f_childCount <= 0) return resPtr;

        for (int i = 0; i < nodeCounter;++i)
        {
            TrajNode* current = &allNodes[i];
            if (current->level_ != 1) continue;

            float tscore = current->bestChildScore_ + (float)current->validChildCount_*config_.f_childCount;

            if (tscore > bestScore)
            {
                bestScore = tscore;
                resPtr = current->bestChild_;
            }


        }

        return resPtr;

    }



};



struct NodeScorer_Path_T : public NodeScorer_Goal_T
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
        cv::Point2f n = b - a;
        cv::Point2f pa = a - p;

        float c = n.dot( pa );

        // Closest point is a
        if ( c > 0.0f )
            return pa.dot(  pa );

        cv::Point2f bp = p - b;

        // Closest point is b
        if ( n.dot( bp ) > 0.0f )
            return bp.dot( bp );

        // Closest point is between a and b
        cv::Point2f e = pa - n * (c / n.dot( n ));

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

    inline void ScoreNode(TrajNode &current)  const
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

struct NodeScorer_PathNG_T : public NodeScorer_Path_T
{

    static constexpr const char* const NS_NAME = "ngpath_scorer";


    inline bool CheckPose(PoseEvalResults &results)
    {

        if (results.TestWheelZValues(validThreshold_))
        {
            //results.validState = PERS_NOWHEELSUPPORT;
            //if (results.poseCounter > config_.noWheelSupportNearThreshold) results.validState = PERS_LOWWHEELSUPPORT_FAR;

            results.validState = ((float)results.poseCounter*poseTimeStep_ > config_.noWheelSupportNearThreshold || (std::abs(results.pose.z - curRobotPose_.z) > config_.noWheelSupportRotateThreshold) ) ? PERS_LOWWHEELSUPPORT_FAR : PERS_NOWHEELSUPPORT;

            return false;
        }

        if (results.TestWheelZValues(notVisibleThreshold_))
        {
            results.validState = PERS_NOTVISIBLE;
            if (config_.allowNotVisible)
            {
                results.a1 = 0;
                results.a2 = 0;

                results.n1.x = 0;
                results.n1.y = 0;
                results.n1.z = 1;
                results.n2.x = 0;
                results.n2.y = 0;
                results.n2.z = 1;
                return true;
            }
            return false;

        }


        if (results.validState == PERS_OUTOFIMAGE)
        {
            return false;
        }


        if (results.gravAngle > config_.gravAngleThreshold)
        {
            results.validState = PERS_EXCEEDGRAVANGLE;
            return false;
        }
        if (results.tipAngle > config_.tipAngleThreshold)
        {
            results.validState = PERS_EXCEEDTIPANGLE;
            return false;
        }
        if (results.deltaAngle > config_.deltaAngleThreshold)
        {
            results.validState = PERS_EXCEEDDELTAANGLE;
            return false;
        }

        if (results.validState == PERS_CHASSISCOLLISION)
        {
            return false;
        }

        if (results.GetMinWheelSupport() < config_.minWheelSupportThreshold)
        {
            results.validState = (results.poseCounter*poseTimeStep_ > config_.noWheelSupportNearThreshold)?  PERS_LOWWHEELSUPPORT_FAR : PERS_LOWWHEELSUPPORT;

            return false;
        }



        return true;

    }

};

/**
 * @brief Scorer without goal position, currently not supported
 */

/*

struct NodeScorer_Simple_T : public NodeScorer_Base
{

//     * Score[0] = sumGravAngle
//     * Score[1] = maxGravAngle
//     * Score[2] = sumAngleDiff
//     * Score[3] = maxAngleDiff
//     * Score[4] = sumTipAngle
//     * Score[5] = maxTipAngle
//     * Score[6] = poseCount
//     * Score[7] = angularVelChange
//     * Score[8] = meanWheelSupport
//     * Score[9] = minWheelSupport
//     *
//     * scores[10] = 0; Not visible counter
//        scores[11] = 0;
//        scores[12] = 0;
//        scores[13] = 0;
//        scores[14] = 0;
//        scores[15] = 0;




    void SetGoal(const cv::Point3f &goal)
    {

    }

    inline void CheckGoal(PoseEvalResults &results)  const
    {

    }

    inline bool CheckPose(PoseEvalResults &results)
    {

        if (results.TestWheelZValues(validThreshold_))
        {
            results.validState = PERS_NOWHEELSUPPORT;

            if (results.poseCounter > config_.noWheelSupportNearThreshold) results.validState = PERS_LOWWHEELSUPPORT_FAR;

            return false;
        }

        if (results.TestWheelZValues(notVisibleThreshold_))
        {
            results.validState = PERS_NOTVISIBLE;
            if (config_.allowNotVisible)
            {
                results.a1 = 0;
                results.a2 = 0;

                results.n1.x = 0;
                results.n1.y = 0;
                results.n1.z = 1;
                results.n2.x = 0;
                results.n2.y = 0;
                results.n2.z = 1;
                return true;
            }
            return false;

        }


        if (results.validState == PERS_OUTOFIMAGE)
        {
            return false;
        }

        if (results.GetMinWheelSupport() < config_.minWheelSupportThreshold)
        {
            results.validState = PERS_LOWWHEELSUPPORT;
            if (results.poseCounter > config_.noWheelSupportNearThreshold) results.validState = PERS_LOWWHEELSUPPORT_FAR;

            return false;
        }

        if (results.gravAngle > config_.gravAngleThreshold)
        {
            results.validState = PERS_EXCEEDGRAVANGLE;
            return false;
        }
        if (results.tipAngle > config_.tipAngleThreshold)
        {
            results.validState = PERS_EXCEEDTIPANGLE;
            return false;
        }
        if (results.deltaAngle > config_.deltaAngleThreshold)
        {
            results.validState = PERS_EXCEEDDELTAANGLE;
            return false;
        }

        if (results.validState == PERS_CHASSISCOLLISION)
        {
            return false;
        }

        return true;

    }

    inline void ResetScores(std::array<float, NUMBERSCORES> &scores)  const
    {
#ifdef USE_REAL_ANGLE_SCORE
        scores[0] = 0;
        scores[1] = 0;
        scores[2] = 0;
        scores[3] = 0;
        scores[4] = 0;
        scores[5] = 0;
#else
        scores[0] = 0;
        scores[1] = 1;
        scores[2] = 0;
        scores[3] = 1;
        scores[4] = 0;
        scores[5] = 1;


#endif


        scores[6] = 0;
        scores[7] = 0;
        scores[8] = 0;
        scores[9] = 1;

        scores[10] = 0;
        scores[11] = 0;
        scores[12] = 0;
        scores[13] = 0;
        scores[14] = 0;
        scores[15] = 0;

    }







    inline void ScorePose(const PoseEvalResults& results, std::array<float, NUMBERSCORES> &scores) const
    {

        //const float gravAngle = acos(results.gravAngle);
        //const float tipAngle = acos(results.tipAngle);
        const float gravAngle = (results.gravAngle);
        const float tipAngle = (results.tipAngle);
        scores[0] += gravAngle;
        scores[1] = std::max(scores[1],gravAngle);

        scores[2] += results.deltaAngle;
        scores[3] = std::max(scores[3], results.deltaAngle);
        scores[4] += tipAngle;
        scores[5] = std::max(scores[5], tipAngle);

        scores[6] += 1.0f;

        const float minWheelSupport = results.GetMinWheelSupport();
        scores[8] += minWheelSupport;
        scores[9] = std::min(scores[9], minWheelSupport);

        scores[10] += results.validState == PERS_NOTVISIBLE ? 1.0f : 0.0f;

    }

    inline void ScoreNode(TrajNode &current)  const
    {
        float velDiff = 0;
        if (current.parent_ != nullptr) velDiff = fabs(current.endCmd_.y);
        current.scores[7] += velDiff;


        //return 0;
    }

    inline void FinalNodeScore(TrajNode &current)  const
    {
        float endFactor = 1.0;

        switch (current.end_->validState)
        {
            case PERS_OUTOFIMAGE: endFactor = config_.end_outOfImage; break;
            case PERS_NOTVISIBLE: endFactor = config_.end_notVisible; break;
            case PERS_NOWHEELSUPPORT: endFactor = config_.end_noWheelSupport; break;
            case PERS_LOWWHEELSUPPORT: endFactor = config_.end_noWheelSupport; break;
            case PERS_LOWWHEELSUPPORT_FAR: endFactor = config_.end_noWheelSupportFar; break;
        case PERS_EXCEEDGRAVANGLE: endFactor = config_.end_exceedAngle; break;
        case PERS_EXCEEDTIPANGLE: endFactor = config_.end_exceedAngle; break;
        case PERS_CHASSISCOLLISION: endFactor = config_.end_chassisCollision; break;
        case PERS_VALID: endFactor = config_.end_valid; break;

        default: endFactor = config_.end_valid;

        }


        float normalize =  current.scores[6];
        if (normalize == 0) normalize = 1.0;
        else normalize = 1.0f/normalize;


        current.fScore_ =
                (current.scores[0]*normalize)*config_.f_meanGA +
                current.scores[1]*config_.f_maxGA +
                (current.scores[2]*normalize)*config_.f_meanAD +
                current.scores[3]*config_.f_maxAD +
                (current.scores[4]*normalize)*config_.f_meanTA +
                current.scores[5]*config_.f_maxTA +
                current.scores[6]*config_.f_poseC +
                current.scores[7]*config_.f_aVelD +
                (current.scores[8]*normalize)*config_.f_meanWS +
                current.scores[9]*config_.f_minWS +
                current.scores[10]*config_.f_numNotVisible +
                endFactor;


        current.finalScores[0] = (current.scores[0]*normalize)*config_.f_meanGA;
        current.finalScores[1] = current.scores[1]*config_.f_maxGA;
                current.finalScores[2] = (current.scores[2]*normalize)*config_.f_meanAD;
                current.finalScores[3] = current.scores[3]*config_.f_maxAD;
                current.finalScores[4] = (current.scores[4]*normalize)*config_.f_meanTA;
                current.finalScores[5] = current.scores[5]*config_.f_maxTA ;
                current.finalScores[6] = current.scores[6]*config_.f_poseC;
                current.finalScores[7] = current.scores[7]*config_.f_aVelD;
                current.finalScores[8] = current.scores[8]*normalize*config_.f_meanWS;
                current.finalScores[9] = current.scores[9]*config_.f_minWS;
                current.finalScores[10] = current.scores[10];
                current.finalScores[11] = endFactor;
                current.finalScores[12] = current.fScore_;


    }




};

*/



#endif // PLANNER_SCORER

