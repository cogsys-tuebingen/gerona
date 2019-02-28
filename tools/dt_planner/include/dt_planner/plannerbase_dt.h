#ifndef PLANNERBASEDT_H
#define PLANNERBASEDT_H





#include "poseestimator_dt.h"
#include "plannerutils_dt.h"
#include "planner_scorer_dt.h"

#include <idtplanner.h>
#include <set>
#include <queue>
#include "utils_math_approx.h"


/**
 * @brief Base class for planners
 */
class PlannerBaseDT : public IDTPlanner
{
public:

    typedef std::shared_ptr<PlannerBaseDT> Ptr;

    PlannerBaseDT();


    virtual cv::Point2f Plan() = 0;


    virtual void SetPlannerParameters(PlannerConfig &config) = 0;
    virtual void SetPlannerScorerParameters(PlannerScorerConfigDT &config) = 0;
    virtual void SetPlannerExpanderParameters(PlannerExpanderConfig &config) = 0;

    virtual int GetNumberNodes() = 0;

    int GetNumberSplits()
    {
        const int angSplits = (config_.expanderConfig_.firstLevelSplits> config_.expanderConfig_.numSplits? config_.expanderConfig_.firstLevelSplits : config_.expanderConfig_.numSplits);
        const int linSplits = config_.expanderConfig_.firstLevelLinearSplits > 0 ? config_.expanderConfig_.firstLevelLinearSplits : 1;
        return angSplits*linSplits;
    }




    virtual void Initialize(DTPlannerConfig &config)
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
        poseEstimator_.GetRobotModel()->SetDEMSize(dem.size());
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

    const DTPlannerConfig* GetConfig()
    {
        return &config_;
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


    //virtual cv::Point2f Plan() = 0;



    PoseEstimatorDT* GetPoseEstimator()
    {
        return &poseEstimator_;
    }

    cv::Mat DrawDebugImage(float scalingFactor, bool drawRobot);
    cv::Mat DrawDebugImage(PoseEvalResultsDT results, float scalingFactor, bool drawRobot);



    void GetAllTrajectoryNodes(std::vector<TrajNodeDT*> &trajectories)
    {
        for (int tl = 0; tl < curNodeIdx_;++tl) trajectories.push_back(&allNodes_[tl]);

    }


    TrajectoryDT* GetResultTrajectory();

    TrajectoryDT* GetBLResultTrajectory();
    TrajNodeDT* GetBestNode()
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
    DTPlannerConfig config_;

    //ProcConfig procConfig_;
    PoseEstimatorDT poseEstimator_;
    //PlannerConfig plannerConfig_;

    cv::Point2f curVelocity_;
    cv::Point3f curRobotPose_;

    cv::Point2f curImgVelocity_;
    cv::Point3f curImgRobotPose_;

    std::vector<cv::Point3f> path_;
    cv::Point3f goal_;


    typedef std::priority_queue<TrajNodeDT*, std::vector<TrajNodeDT*>,CompareTNodeDT> prio_queue;
    prio_queue openSet_;

    //PlannerScorerConfig scorerConig_;
    float bestScore_;
    TrajNodeDT* bestNode_;
    TrajectoryDT resultTraj_;
    TrajectoryDT resultBLTraj_;


    std::vector<TrajNodeDT> allNodes_;
    int curNodeIdx_;

    //std::vector<TrajNode*> leaves_;
    //int curLeaveIdx_;

    std::vector<cv::Point2f> tempCmds_;




};








#endif // PLANNERBASE_H
