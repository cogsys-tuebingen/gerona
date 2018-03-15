#ifndef PLANNERBASE_H
#define PLANNERBASE_H





#include "poseestimator.h"
#include "plannerutils.h"
#include "planner_scorer.h"

#include <imodelbasedplanner.h>
#include <set>
#include <queue>
#include "utils_math_approx.h"


/**
 * @brief Base class for planners
 */
class PlannerBase : public IModelBasedPlanner
{
public:

    typedef std::shared_ptr<PlannerBase> Ptr;

    PlannerBase();


    virtual cv::Point2f Plan() = 0;


    virtual void SetPlannerParameters(PlannerConfig &config) = 0;
    virtual void SetPlannerScorerParameters(PlannerScorerConfig &config) = 0;
    virtual void SetPlannerExpanderParameters(PlannerExpanderConfig &config) = 0;

    virtual int GetNumberNodes() = 0;

    int GetNumberSplits()
    {
        const int angSplits = (config_.expanderConfig_.firstLevelSplits> config_.expanderConfig_.numSplits? config_.expanderConfig_.firstLevelSplits : config_.expanderConfig_.numSplits);
        const int linSplits = config_.expanderConfig_.firstLevelLinearSplits > 0 ? config_.expanderConfig_.firstLevelLinearSplits : 1;
        return angSplits*linSplits;
    }



    virtual void Initialize(ModelBasedPlannerConfig &config)
    {
        config_ = config;
        SetupNodes();

        //procConfig_ = config.procConfig_;
        //procConfig_.Setup();
        poseEstimator_.Setup(config);

        //plannerConfig_ = config.plannerConfig_;
        //plannerConfig_.Setup();

    }


    void UpdateDEM(const cv::Mat &dem)
    {
        poseEstimator_.SetDem(dem);
    }
    const cv::Mat GetDem()
    {
        return poseEstimator_.GetDEM();
    }



    cv::Point2f VelToImgVel(const cv::Point2f &curVel){
        cv::Point2f res = curVel;
        res.x *= config_.procConfig_.pixelSizeInv;
        return res;
    }


    cv::Point3f PoseToImgPose(const cv::Point3f &pose)
    {
        return cv::Point3f((pose.x-config_.procConfig_.imagePosBLMinX)*config_.procConfig_.pixelSizeInv,(pose.y-config_.procConfig_.imagePosBLMinY)*config_.procConfig_.pixelSizeInv,pose.z );
    }

    cv::Point3f ImgPoseToPose(const cv::Point3f &pose)
    {
        return cv::Point3f(pose.x*config_.procConfig_.pixelSize+config_.procConfig_.imagePosBLMinX,pose.y*config_.procConfig_.pixelSize+config_.procConfig_.imagePosBLMinY,pose.z );
    }

    void SetDEMPos(const cv::Point2f &minPos)
    {
        config_.procConfig_.imagePosBLMinX = minPos.x;
        config_.procConfig_.imagePosBLMinY = minPos.y;

    }

    cv::Point2f GetDEMPos()
    {
        return cv::Point2f(config_.procConfig_.imagePosBLMinX,config_.procConfig_.imagePosBLMinY);
    }




    void SetVelocity(const cv::Point2f &curVel)
    {
        curVelocity_ = curVel;
        curImgVelocity_ = VelToImgVel(curVelocity_);

    }

    cv::Point2f GetVelocity()
    {
        return curVelocity_;


    }

    void SetRobotPose(const cv::Point3f &curPose)
    {
        curRobotPose_ = curPose;
        curImgRobotPose_ = PoseToImgPose(curRobotPose_);
    }

    cv::Point3f GetRobotPoseImage()
    {
        return curImgRobotPose_;
    }



    /*
    virtual void SetGoalMap(const cv::Point3f goal)
    {
        goal_ = PoseToImgPose(goal);
    }
    */

    cv::Point3f GetGoal()
    {
        return goal_;
    }

    const ModelBasedPlannerConfig* GetConfig()
    {
        return &config_;
    }

    inline void CalculateAngleDiff(const PoseEvalResults &prevPER, PoseEvalResults &PER) const
    {
        if (prevPER.validState == PERS_NOTASSIGNED || prevPER.validState == PERS_NOTVISIBLE) PER.deltaAngle = 0;
        else
        {
            const cv::Point3f na1 = (PER.a1 > PER.a2)?PER.n1:PER.n2;
            const cv::Point3f na2 = (PER.a1 > PER.a2)?PER.n2:PER.n1;

            const cv::Point3f nb1 = (prevPER.a1 > prevPER.a2)?prevPER.n1:prevPER.n2;
            const cv::Point3f nb2 = (prevPER.a1 > prevPER.a2)?prevPER.n2:prevPER.n1;


            const float nd1 = na1.dot(nb1);
            const float nd2 = na2.dot(nb2);
            PER.deltaAngle = Utils_Math_Approx::facos(std::min(nd1,nd2));
            PER.poseCounter = prevPER.poseCounter+1;
        }
    }

    void SetupNodes()
    {

        allNodes_.clear();
        //allNodes_.reserve(maxNumNodes);

        for (int tl = 0; tl < GetNumberNodes();++tl)
        {
            //TrajNode tnode(config_.plannerConfig_.numSubSamples);
            //allNodes_.push_back(TrajNode(numSubSteps));
            //allNodes_.push_back(std::move(tnode));
            allNodes_.emplace_back(config_.plannerConfig_.numSubSamples);
        }

        //leaves_.reserve(maxNumNodes);

        curNodeIdx_ = 0;
        //openSet_.clear();
        openSet_ = prio_queue();

        tempCmds_.resize(GetNumberSplits());
    }


    inline void ClearPrioQueue()
    {
        openSet_ = prio_queue();
    }


    /*
    inline bool CheckState(const PoseEvalResults &results) const
    {
        return !(results.validState == PERS_OUTOFIMAGE
                 || results.validState == PERS_NOWHEELSUPPORT
                 || results.validState == PERS_GOALREACHED
                 || results.validState == PERS_EXCEEDGRAVANGLE
                 || results.validState == PERS_EXCEEDTIPANGLE);
        //if (results.validState == PERS_OUTOFIMAGE || results.validState == PERS_NOWHEELSUPPORT || results.gravAngle < config_.gravAngleThreshold) return false;

    }
    */


    //virtual cv::Point2f Plan() = 0;



    PoseEstimator* GetPoseEstimator()
    {
        return &poseEstimator_;
    }

    cv::Mat DrawDebugImage(float scalingFactor, bool drawRobot);



    void GetAllTrajectoryNodes(std::vector<TrajNode*> &trajectories)
    {
        for (int tl = 0; tl < curNodeIdx_;++tl) trajectories.push_back(&allNodes_[tl]);

    }


    Trajectory* GetResultTrajectory();

    Trajectory* GetBLResultTrajectory();
    TrajNode* GetBestNode()
    {

        return bestNode_;


    }

    int GetPoseCount()
    {
        int res = 0;
        for (int tl = 0; tl < curNodeIdx_;++tl) res += allNodes_[tl].numValid_;
        return res;

    }

    //virtual void SetPlannerParameters(PlannerConfig &config){}
    //virtual void SetPlannerScorerParameters(PlannerScorerConfig &config){}
    //virtual void SetPlannerExpanderParameters(PlannerExpanderConfig &config){}



protected:
    ModelBasedPlannerConfig config_;

    //ProcConfig procConfig_;
    PoseEstimator poseEstimator_;
    //PlannerConfig plannerConfig_;

    cv::Point2f curVelocity_;
    cv::Point3f curRobotPose_;

    cv::Point2f curImgVelocity_;
    cv::Point3f curImgRobotPose_;

    cv::Point3f goal_;


    typedef std::priority_queue<TrajNode*, std::vector<TrajNode*>,CompareTNode> prio_queue;
    prio_queue openSet_;

    //PlannerScorerConfig scorerConig_;
    float bestScore_;
    TrajNode* bestNode_;
    Trajectory resultTraj_;
    Trajectory resultBLTraj_;


    std::vector<TrajNode> allNodes_;
    int curNodeIdx_;

    //std::vector<TrajNode*> leaves_;
    //int curLeaveIdx_;

    std::vector<cv::Point2f> tempCmds_;




};








#endif // PLANNERBASE_H
