#ifndef ROBOTMODELDT_H
#define ROBOTMODELDT_H

#include <memory>
#include "cv_aligned_mat.h"
#include "config_dtplanner.h"
#include "robotdescriptor_dt.h"
#include "poseevalresults_dt.h"
#include "utils_math_approx.h"


/**
 * @brief The robot model implementation
 */
class RobotModelDT
{
public:
    typedef std::shared_ptr<RobotModelDT> Ptr;
    static RobotModelDT::Ptr Create(){ return std::make_shared< RobotModelDT >() ; }


    RobotModelDT();

    /**
     * @brief Initiliazes the robot model including wheels and chassis
     */
    void SetupRobot(DTPlannerConfig &config);

    void SetupRobot(const ProcConfigDT &procConfig, const RobotConfigDT &robotConfig);

    /**
     * @brief Evaluate for a given DEM and pose
     */
    int EvaluatePose(const cv::Mat &dem, PoseEvalResultsDT &results) const;

    /**
     * @brief Calculates the z-position of the base link
     */
    float GetBaseLinkZ(const PoseEvalResultsDT &results);





    /**
     * @brief helper function for testing intersection of two lines
     */
    inline bool intersection(cv::Point2f o1, cv::Point2f p1, cv::Point2f o2, cv::Point2f p2, cv::Point2f &r) const
    {
        cv::Point2f x = o2 - o1;
        cv::Point2f d1 = p1 - o1;
        cv::Point2f d2 = p2 - o2;

        float cross = d1.x*d2.y - d1.y*d2.x;
        if (std::abs(cross) < /*EPS*/1e-8)
            return false;

        double t1 = (x.x * d2.y - x.y * d2.x)/cross;
        r = o1 + d1 * t1;
        return true;
    }

    /**
     * @brief helper function for getting intersection of two lines
     */
    inline const cv::Point2f intersection(cv::Point2f o1, cv::Point2f p1, cv::Point2f o2, cv::Point2f p2) const
    {
        cv::Point2f x = o2 - o1;
        cv::Point2f d1 = p1 - o1;
        cv::Point2f d2 = p2 - o2;

        float cross = d1.x*d2.y - d1.y*d2.x;

        double t1 = (x.x * d2.y - x.y * d2.x)/cross;
        return o1 + d1 * t1;
    }


    const RobotDescriptorDT& GetDescriptor(const int &i) const {return descriptors_[i];}
    //WheelModel& GetWheel(const int &i) {return wheels_[i];}
    //ChassisModel& GetChassis() {return chassisModel_;}
    ProcConfigDT& GetProcConfig(){return procConfig_;}
    RobotConfigDT& GetRobotConfig(){return config_;}

    int GetNumTestPoints() {return testPoints_.size();}
    TestPointConfig& GetTestPoint(const int &i) {return testPoints_[i];}


    /**
     * @brief clamp angle to 0..2Pi
     */
    inline float NormalizeAngle(const float angle) const
    {        

        float res = angle;
        while (res >= CV_2PIF)
        {
            res -= CV_2PIF;
            //res = res%procConfig_.numAngleStep;
        }
        while (res < 0)
        {
            res += CV_2PIF;
        }
        return res;
    }


    /**
     * @brief Get descriptor index for a given angle
     */
    inline int GetAngleIdx(const float &angle) const
    {
        int res = (int)round(angle*config_.angleStepInv);
        //if (res >= procConfig_.numAngleStep || res < 0)
        while (res >= config_.numAngleStep)
        {
            res -= config_.numAngleStep;
            //res = res%procConfig_.numAngleStep;
        }
        while (res < 0)
        {
            res += config_.numAngleStep;
        }
        //if (res < 0) res = res%procConfig_.numAngleStep;
        return res;
    }


    /**
     * @brief Get descriptor index for a given angle without testing bounds
     */
    inline int GetAngleIdxFast(const float &angle) const
    {
        int res = (int)round(angle*config_.angleStepInv);

        if (res >= config_.numAngleStep)
        {
            res -= config_.numAngleStep;

        }
        else if (res < 0)
        {
            res += config_.numAngleStep;
        }

        return res;
    }

    void SetDEMSize(cv::Size size){demRowsF_ = size.height; demColsF_ = size.width;}


private:
    float angleStep_;

    cv::Point2f baseLinkPosImage_;

    std::vector<RobotDescriptorDT> descriptors_;

    //int numTestPoints_;

    ProcConfigDT procConfig_;
    RobotConfigDT config_;
    std::vector<TestPointConfig> testPoints_;

    float invNumTestPoints_;
    float demRowsF_,demColsF_;
    float dontCareDist_;




};

#endif // ROBOTMODEL_H
