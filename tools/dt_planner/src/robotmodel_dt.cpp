#include "robotmodel_dt.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "utils_math_approx.h"


RobotModelDT::RobotModelDT()
{
    demRowsF_ = -1;
    demColsF_ = -1;
}


int RobotModelDT::EvaluatePose(const cv::Mat &dem, PoseEvalResultsDT &results) const
{
    const cv::Point2f pos(results.pose.x,results.pose.y);
    const float angle = NormalizeAngle(results.pose.z);
    const int angleIdx = GetAngleIdxFast(angle);
    const RobotDescriptorDT &desc = GetDescriptor(angleIdx);
    const cv::Point2f robotCenter = pos-desc.baseLinkPosImage_;
    //const cv::Point2f robotCenterImg = robotCenter* procConfig_.pixelSizeInv;

    const float *demPtr = dem.ptr<float>();
    cv::Point2f testPoint;
    float distanceToPoint;
    float distanceToCircle;

    results.validState = PERSDT_VALID;

    results.minDist = dem.rows*dem.cols;

    for (int i = 0; i < testPoints_.size();++i)
    {
        testPoint = robotCenter + desc.testPositionsImage_ [i];
        if (testPoint.x < 0.0f || testPoint.y < 0.0f || testPoint.x >= demColsF_ || testPoint.y >= demRowsF_)
        {
            results.validState = PERSDT_OUTOFIMAGE;
            break;

        }
        else
        {
            distanceToPoint = demPtr[(int)testPoint.y * dem.cols + (int)testPoint.x ];
            distanceToCircle = distanceToPoint - testPoints_[i].radiusImg;
            results.distances[i] = distanceToCircle;
            results.minDist= std::min(results.minDist,distanceToCircle);
            results.meanDist += distanceToCircle;
            results.validState = distanceToCircle<0 ? PERSDT_COLLISION : results.validState;
        }
    }

    results.meanDist *= invNumTestPoints_;


    return 0;

}


void RobotModelDT::SetupRobot(DTPlannerConfig &config)
{

    config.procConfig_.Setup();
    SetupRobot(config.procConfig_,config.robotConfig_);

}



void RobotModelDT::SetupRobot(const ProcConfigDT &procConfig, const RobotConfigDT &robotConfig)
{


    procConfig_ = procConfig;
    config_ = robotConfig;
    testPoints_ = robotConfig.testPoints;


    invNumTestPoints_ = 1.0f/(float)testPoints_.size();

    float curAngle;

    for (int i = 0; i < config_.numAngleStep;i++)
    {
        curAngle = config_.angleStep*(float)i;
        cv::Mat rotMat = cv::getRotationMatrix2D(cv::Point2f(0,0),curAngle*(180.0/CV_PI),1.0);

        RobotDescriptorDT desc;

        desc.rotMat_ = rotMat;

        desc.sina_ = sin(curAngle);
        desc.cosa_ = cos(curAngle);

        cv::Mat twp(3,1,CV_64F);
        twp.at<double>(0,0) = config_.baseLinkPosCoord.x;
        twp.at<double>(1,0) = config_.baseLinkPosCoord.y;
        twp.at<double>(2,0) = 1.0;
        cv::Mat res2 = rotMat*(twp);


        desc.baseLinkPosImage_ = cv::Point2f(res2.at<double>(0,0)*procConfig.pixelSizeInv, -res2.at<double>(1,0)*procConfig.pixelSizeInv);




        for (unsigned int i = 0; i < testPoints_.size();i++)
        {
            //cv::Vec3f twp(wheels_[i].wheelPosCoord_.x,wheels_[i].wheelPosCoord_.y,1);
            //cv::Mat twp(3,1,CV_64F);
            twp.at<double>(0,0) = testPoints_[i].pointPosRobot.x;
            twp.at<double>(1,0) = testPoints_[i].pointPosRobot.y;
            twp.at<double>(2,0) = 1.0;

            cv::Mat res = rotMat*(twp);
            //desc.wheelPositionsCoord_.push_back(cv::Point2f(res.at<double>(0,0), res.at<double>(1,0)));
            desc.testPositionsImage_.push_back(cv::Point2f(res.at<double>(0,0)*procConfig.pixelSizeInv, -res.at<double>(1,0)*procConfig.pixelSizeInv ));

        }


        //desc.gravCenterImage_ = intersection(desc.wheelPositionsImage_[0],desc.wheelPositionsImage_[2],desc.wheelPositionsImage_[1],desc.wheelPositionsImage_[3]);

        /*
        if (testChassis_)
        {
            twp.at<double>(0,0) = chassisModel_.GetWheelPos().x;
            twp.at<double>(1,0) = chassisModel_.GetWheelPos().y;
            twp.at<double>(2,0) = 1.0;

            cv::Mat res = rotMat*(twp);
           desc.chassisPosCoord_ = (cv::Point2f(res.at<double>(0,0), res.at<double>(1,0)));
           desc.chassisPosImage_ = (cv::Point2f(res.at<double>(0,0)*GConfig::pC.pixelSizeInv_, -res.at<double>(1,0)*GConfig::pC.pixelSizeInv_ ));

        }
        */


        descriptors_.push_back(desc);


    }

}
