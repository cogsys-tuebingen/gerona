#ifndef UTILSDRAW_H
#define UTILSDRAW_H


#include <imodelbasedplanner.h>
#include <scaleddrawproc.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <robotmodel.h>


/**
 * @brief Helper class for drawing debug images
 */
class DrawProc
{

public:
    DrawProc()
    {
        drawScaleFactor_ = 4.0;
        drawZMin_ = - 128;
        drawZMax_ =  128;

    }


    cv::Mat D16SImageToRGB(cv::Mat image, int imin, int imax);

    void DrawMapStates(const cv::Mat &dem, cv::Mat &drawMat, const ProcConfig &config);

    cv::Mat Draw(ScaledDrawProc &sdp,const cv::Mat dem, const ProcConfig &config,std::vector<TrajNode*> &trajectories, const Trajectory* bestTraj_, cv::Point3f goal,cv::Point3f robotPos);


    void DrawTrajectories(ScaledDrawProc &sdp, const std::vector<TrajNode*> &trajectories, const Trajectory* bestTraj_);
    void SetupDrawProc(ScaledDrawProc &drawProc,cv::Mat &img, float scaleFactor);

    //static void DrawRobot(ScaledDrawProc &drawProc,IModelBasedPlanner &planner);
    void DrawGoal(ScaledDrawProc &drawProc,cv::Point3f goal,cv::Point3f robotPos);
    void DrawPath(ScaledDrawProc &drawProc,std::vector<cv::Point3f> path);

    void DrawRobotScaled(ScaledDrawProc &drawProc, RobotModel &model, PoseEvalResults &results);
    void DrawRobotWheelScaled(ScaledDrawProc &proc, WheelModel &model, cv::Point2f pos, int angleIdx,cv::Point2i contact);

    void DrawWheelLinesScaled(ScaledDrawProc &proc, cv::Point2f &origin, WheelModel &model, int angle);
    cv::Mat GenerateMask(const cv::Mat &img);

    void DrawChassis(ScaledDrawProc &proc, ProcConfig &procConfig, ChassisModel &model, PoseEvalResults &results, cv::Point2f pos, int angleIdx);

    cv::Scalar GetEndStateColor(int validState);

//private:


    ModelBasedPlannerConfig conf;

    float drawScaleFactor_;
    float drawZMin_,drawZMax_;

};




#endif //UTILSDRAW_H
