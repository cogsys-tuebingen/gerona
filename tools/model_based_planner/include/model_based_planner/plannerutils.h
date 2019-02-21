#ifndef PLANNERUTILS_H
#define PLANNERUTILS_H


#include "poseevalresults.h"
#include <array>

#define NUMBERSCORES 16
#define COMMANDEPSILON 0.00001f


/**
 * @brief Update model for differential drive and ackermann
 */
struct DriveModelDA
{
    static void UpdatePose(const cv::Point3f &in, const cv::Point2f &cmd, cv::Point3f &res){

        if (std::abs(cmd.y) < COMMANDEPSILON)
        {
            res.x = in.x+std::cos(in.z)*cmd.x;
            res.y = in.y+std::sin(in.z)*cmd.x;
            res.z = in.z;


        }
        else
        {

            const float r = cmd.x/cmd.y;

            res.x = in.x + r*(std::sin(in.z + cmd.y) - std::sin(in.z) );
            res.y = in.y + r*(-std::cos(in.z + cmd.y) + std::cos(in.z) );
            res.z = in.z + cmd.y;
        }

    }

    static void UpdatePose(const cv::Point3f &in, const cv::Point2f &cmd, const float &r, cv::Point3f &res){

        if (std::abs(cmd.y) < COMMANDEPSILON)
        {
            res.x = in.x+std::cos(in.z)*cmd.x;
            res.y = in.y+std::sin(in.z)*cmd.x;
            res.z = in.z;


        }
        else
        {


            res.x = in.x + r*(std::sin(in.z + cmd.y) - std::sin(in.z) );
            res.y = in.y + r*(-std::cos(in.z + cmd.y) + std::cos(in.z) );
            res.z = in.z + cmd.y;
        }

    }


};





/**
 * @brief Basic Trajectory struct
 */
struct Trajectory
{
    Trajectory()
    {

        poseResults_.clear();
        Reset();

    }
    Trajectory(int numSteps):poseResults_(numSteps)
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

    PoseEvalResults* start_;
    PoseEvalResults* end_;

    std::vector<PoseEvalResults> poseResults_;

private:
    //Trajectory(const Trajectory& that) = delete;



};



/**
 * @brief Node class for path planning, adding scores and robot commands
 */
struct TrajNode : public Trajectory
{

    TrajNode(int numSteps): Trajectory(numSteps),
        parent_(nullptr), level_(0)
    {
        Reset();
    }

    TrajNode(): Trajectory(),
        parent_(nullptr), level_(0)
    {
        Reset();
    }

    void SetParent(TrajNode* parent)
    {
        parent_ = parent;
        startCmd_ = parent_->endCmd_;
        start_ = parent->end_;
        level_ = parent_->level_+1;
        for (unsigned int tl = 0; tl < scores.size();++tl) scores[tl] = parent->scores[tl];

    }

    TrajNode* GetFirstNode()
    {
        TrajNode* res = this;

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
        validState_ = TN_VS_NOTASSIGNED;
        numValid_ = 0;

        for (unsigned int tl = 0; tl < poseResults_.size();++tl) poseResults_[tl].Reset();

    }


    TrajNode* parent_;
    int level_;

    cv::Point2f startCmd_;
    cv::Point2f endCmd_;


    std::array<float, NUMBERSCORES> scores;
    std::array<float, NUMBERSCORES> finalScores;

    //!values used by the Star type algorithms
    float fScore_;//,gScore_;
    int validState_;



};



struct CompareTNode : public std::binary_function<TrajNode*, TrajNode*, bool> {
    bool operator()(const TrajNode* lhs, const TrajNode* rhs) const {
        return lhs->fScore_ < rhs->fScore_;
    }
};



#endif // PLANNERUTILS_H
