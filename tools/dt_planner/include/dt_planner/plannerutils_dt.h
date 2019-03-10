#ifndef PLANNERUTILSDT_H
#define PLANNERUTILSDT_H


#include "poseevalresults_dt.h"
#include <array>

#define NUMBERSCORES 16
#define COMMANDEPSILON 0.00001f


/**
 * @brief Basic Trajectory struct
 */
struct TrajectoryDT
{
    TrajectoryDT()
    {

        poseResults_.clear();
        Reset();

    }
    TrajectoryDT(int numSteps):poseResults_(numSteps)
    {
        Reset();

    }


    void SetEnd(int idx)
    {
        end_ = &poseResults_[idx];
        numValid_ = idx+1;
    }

    void Reset()
    {
        time_ = 0;
        //cmd_ = cv::Point2f(0,0);
        start_ = nullptr;
        end_ = nullptr;

        for (unsigned int tl = 0; tl < poseResults_.size();++tl) poseResults_[tl].Reset();

    }

    void Reset(int numPoses)
    {
        time_ = 0;
        //cmd_ = cv::Point2f(0,0);
        start_ = nullptr;
        end_ = nullptr;

        poseResults_.resize(numPoses);

        for (unsigned int tl = 0; tl < poseResults_.size();++tl) poseResults_[tl].Reset();

    }


    float time_;
    int numValid_;

    //cv::Point2f cmd_;
    //cv::Point3f start_, end_;

    PoseEvalResultsDT* start_;
    PoseEvalResultsDT* end_;

    std::vector<PoseEvalResultsDT> poseResults_;

private:
    //Trajectory(const Trajectory& that) = delete;



};



/**
 * @brief Node class for path planning, adding scores and robot commands
 */
struct TrajNodeDT : public TrajectoryDT
{

    TrajNodeDT(int numSteps): TrajectoryDT(numSteps),
        parent_(nullptr), level_(0)
    {
        Reset();
    }

    TrajNodeDT(): TrajectoryDT(),
        parent_(nullptr), level_(0)
    {
        Reset();
    }

    void SetParent(TrajNodeDT* parent)
    {
        parent_ = parent;
        startCmd_ = parent_->endCmd_;
        start_ = parent->end_;
        level_ = parent_->level_+1;
        for (unsigned int tl = 0; tl < scores.size();++tl) scores[tl] = parent->scores[tl];

    }

    TrajNodeDT* GetFirstNode()
    {
        TrajNodeDT* res = this;

        while (res->parent_ != nullptr)
        {
            if (res->parent_->level_ != 0)
            {
            res = res->parent_;
            }
            else break;

        }

        return res;

    }

    void Reset()
    {
        parent_ = nullptr;
        start_ = nullptr;
        end_ = nullptr;
        level_ = 0;

        time_ = 0;
        startCmd_ = cv::Point2f(0,0);
        endCmd_ = cv::Point2f(0,0);

        scores.fill(0.0f);
        fScore_ = 0;
        validState_ = TNDT_VS_NOTASSIGNED;
        numValid_ = 0;

        bestChildScore_ = -99999999;
        validChildCount_ = 0;
        bestChild_ = nullptr;

        memset(&poseResults_[0],0,sizeof(PoseEvalResultsDT)*poseResults_.size());

        for (unsigned int tl = 0; tl < poseResults_.size();++tl) poseResults_[tl].validState = PERSDT_NOTASSIGNED;

    }


    /*
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
       */
    //                                             0           1           2           3     4  5      6       7      8        9               10           11          12            13         14                15
    //static const char * const scoreNames[] = { "meanMean", "meanMin", "minMean", "minMin", "","", "poseC", "aVelD", "", "lastCmdVelDiff", "vChildCnt", "goalDist", "goalOrien", "pathDist", "endFactor", "lowPoseCountPenalty" };

    static std::string GetScoreName(int idx)
    {

        switch (idx)
        {
        case 0: return "meanMean";
        case 1: return "meanMin";
        case 2: return "minMean";
        case 3: return "minMin";
        case 6: return "poseC";
        case 7: return "aVelD";
        case 9: return "lastCmdVelDiff";
        case 10: return "vChildCnt";
        case 11:return "goalDist";
        case 12:return "goalOrien";
        case 13:return "pathDist";
        case 14:return "endFactor";
        case 15:return "lowPoseCountPenalty";
        default: return "unknown";

        }
    }



    TrajNodeDT* parent_;
    int level_;

    cv::Point2f startCmd_;
    cv::Point2f endCmd_;


    std::array<float, NUMBERSCORES> scores;
    std::array<float, NUMBERSCORES> finalScores;

    //!values used by the Star type algorithms
    float fScore_;//,gScore_;
    int validState_;

    float bestChildScore_;
    int validChildCount_;
    TrajNodeDT* bestChild_;


};



struct CompareTNodeDT : public std::binary_function<TrajNodeDT*, TrajNodeDT*, bool> {
    bool operator()(const TrajNodeDT* lhs, const TrajNodeDT* rhs) const {
        return lhs->fScore_ < rhs->fScore_;
    }
};



#endif // PLANNERUTILS_H
