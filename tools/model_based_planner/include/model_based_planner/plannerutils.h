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
 * @brief class for creating a clamped set of velocities the
 */
class SpeedRamp
{

public:

    SpeedRamp()
    {
        lastTime_ = 0;
        currentSpeed_ = 0;
        acceleration_ = 0.1;
        deacceleration_ = 0.1;
        minVel_ = 0.1;
        maxVel_ = 0.5;
    }

    double RequestSpeed(const double &req, const double &time)
    {
        auto deltaT = time - lastTime_;
        lastTime_ = time;

        auto diffV = req - currentSpeed_;

        if (diffV > 0.0)
        {
            auto maxVInc = acceleration_*deltaT;
            auto velChange = std::min(maxVInc,diffV);
            currentSpeed_ += velChange;
            currentSpeed_ = std::min(currentSpeed_,maxVel_);

        }
        else if (diffV < 0.0)
        {
            auto maxVDec = deacceleration_*deltaT;
            auto velChange = std::max(maxVDec,diffV);
            currentSpeed_ += velChange;
            currentSpeed_ = std::max(currentSpeed_,minVel_);
        }

        return currentSpeed_;
    }

    void SetCurrentSpeed(const double &speed){currentSpeed_ = speed;}

    void SetCurrentTime(const double &time){lastTime_ = time;}


    void Reset(const double &time) {currentSpeed_ = 0;lastTime_ = time;}


    double currentSpeed_;
    double lastTime_;

    double acceleration_;
    double deacceleration_;

    double minVel_,maxVel_;
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

        bestChildScore_ = -99999999.0f;
        validChildCount_ = 0;
        bestChild_ = nullptr;


        for (unsigned int tl = 0; tl < poseResults_.size();++tl) poseResults_[tl].Reset();

    }

    /*
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
        //current.finalScores[10] = current.scores[10];
        current.finalScores[11] = current.scores[11]*config_.f_goalDistance;
        current.finalScores[12] = current.scores[12]*config_.f_goalOrientation;
        current.finalScores[13] = (current.scores[13]*levelNorm)*config_.f_pathDistance;
        current.finalScores[14] = endFactor;
        current.finalScores[15] = lowPoseCountPenalty;

       */
    //                                                        0        1          2        3        4        5         6       7        8        9          10           11          12          13         14                15
    //static const constexpr char * const scoreNames[] = { "meanGA", "maxGA", "_meanAD", "maxAD", "meanTA","maxTA", "poseC", "aVelD", "meanWS", "minWS", "vChildCnt", "goalDist", "goalOrien", "pathDist", "endFactor", "lowPoseCountPenalty" };

    static std::string GetScoreName(int idx)
    {

        switch (idx)
        {
        case 0: return "meanGA";
        case 1: return "maxGA";
        case 2: return "meanAD";
        case 3: return "maxAD";
        case 4: return "meanTA";
        case 5: return "maxTA";
        case 6: return "poseC";
        case 7: return "aVelD";
        case 8: return "meanWS";
        case 9: return "minWS";
        case 10: return "vChildCnt";
        case 11:return "goalDist";
        case 12:return "goalOrien";
        case 13:return "pathDist";
        case 14:return "endFactor";
        case 15:return "lowPoseCountPenalty";
        default: return "unknown";

        }
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

    float bestChildScore_;
    int validChildCount_;
    TrajNode* bestChild_;


};



struct CompareTNode : public std::binary_function<TrajNode*, TrajNode*, bool> {
    bool operator()(const TrajNode* lhs, const TrajNode* rhs) const {
        return lhs->fScore_ < rhs->fScore_;
    }
};



#endif // PLANNERUTILS_H
