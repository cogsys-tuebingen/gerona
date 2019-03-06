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
