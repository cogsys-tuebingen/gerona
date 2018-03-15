#ifndef POSEWRITER_H
#define POSEWRITER_H


#include "plannerutils.h"
#include "config_modelbasedplanner.h"


/**
 * @brief utility class for writing debug data
 */
class PoseWriter
{
public:
    //PoseWriter(std::string parentFolder);
    PoseWriter();

    void Init(std::string parentFolder);

    void WritePoses(const Trajectory *traj, cv::Point2f mapOrigin);
    void WritePoses(const Trajectory *traj, const cv::Mat &dem, cv::Point2f mapOrigin);
    void WritePoses(const Trajectory *traj, cv::Point2f mapOrigin, cv::Point3f robotWorldPose);

    void WriteConfig(const ModelBasedPlannerConfig &config, const std::string &modelFile);

    void WriteTimings(float ms, int numPoses);

    void SetMaxNumPoses(int numMaxPoses)
    {
        numMaxPoses_ = numMaxPoses;
    }

    //~PoseWriter();
private:
    int poseCounter_;
    std::string outputFolder_;
    long startUS_;

    int numMaxPoses_;

};


#endif // POSEWRITER_H
