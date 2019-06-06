#ifndef POSEESTIMATOR_H
#define POSEESTIMATOR_H



#include "robotmodel.h"





/**
 * @brief Main pose estimator class, contains the robot model and the current DEM. Performs the actual pose estimation for a given pose with the robot model and the current dem
 */
class PoseEstimator
{
public:
    PoseEstimator();

    void Setup(ModelBasedPlannerConfig &config);

    void SetDem(CVAlignedMat::ptr dem);
    void SetDem(cv::Mat dem);
    cv::Mat GetDEM(){
        return dem_;
    }

    void Evaluate(PoseEvalResults &results) const;
    cv::Mat DrawDebugImage(PoseEvalResults &results);

    RobotModel* GetRobotModel(){return &robotModel_; }

    cv::Point3f PoseToImgPose(const cv::Point3f &pose, const cv::Point2f &minPos)
    {
        ProcConfig &procConfig_ = robotModel_.GetProcConfig();

        imagePosBLMinX = minPos.x;
        imagePosBLMinY = minPos.y;

        return cv::Point3f((pose.x-imagePosBLMinX)*procConfig_.pixelSizeInv,(pose.y-imagePosBLMinY)*procConfig_.pixelSizeInv,pose.z );
    }

    RobotModel robotModel_;
    float imagePosBLMinX,imagePosBLMinY;
private:
    CVAlignedMat::ptr demPtr_;
    cv::Mat dem_;



};

#endif // POSEESTIMATOR_H
