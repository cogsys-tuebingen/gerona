#ifndef POSEESTIMATORDT_H
#define POSEESTIMATORDT_H



#include "robotmodel_dt.h"

#include "poseevalresults_dt.h"




/**
 * @brief Main pose estimator class, contains the robot model and the current DEM. Performs the actual pose estimation for a given pose with the robot model and the current dem
 */
class PoseEstimatorDT
{
public:
    PoseEstimatorDT();

    void Setup(DTPlannerConfig &config);

    //void SetDem(CVAlignedMat::ptr dem);
    void SetDem(cv::Mat dem);
    cv::Mat GetDEM(){
        return orgDem_;
    }
    cv::Mat GetDMap(){
        return dmap_;
    }

    void CreateMap(const cv::Point3f &rPos, const PlannerScorerConfigDT& scoreConf );


    void Evaluate(PoseEvalResultsDT &results) const;
    cv::Mat DrawDebugImage(PoseEvalResultsDT &results);

    RobotModelDT* GetRobotModel(){return &robotModel_; }

    cv::Point3f PoseToImgPose(const cv::Point3f &pose, const cv::Point2f &minPos)
    {
        ProcConfigDT &procConfig_ = robotModel_.GetProcConfig();

        procConfig_.imagePosBLMinX = minPos.x;
        procConfig_.imagePosBLMinY = minPos.y;

        return cv::Point3f((pose.x-procConfig_.imagePosBLMinX)*procConfig_.pixelSizeInv,(pose.y-procConfig_.imagePosBLMinY)*procConfig_.pixelSizeInv,pose.z );
    }

    RobotModelDT robotModel_;

private:
    CVAlignedMat::ptr dmapPtr_;
    cv::Mat dmap_;
    cv::Mat orgDem_;

};

#endif // POSEESTIMATOR_H
