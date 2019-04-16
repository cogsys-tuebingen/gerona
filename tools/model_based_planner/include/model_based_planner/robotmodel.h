#ifndef ROBOTMODEL_H
#define ROBOTMODEL_H

#include "wheelmodel.h"
#include "chassismodel.h"
#include "config_modelbasedplanner.h"
#include "robotdescriptor.h"
#include "poseevalresults.h"
#include "utils_math_approx.h"
#include "plannerutils.h"


/**
 * @brief The robot model implementation
 */
class RobotModel
{
public:
    typedef std::shared_ptr<RobotModel> Ptr;
    static RobotModel::Ptr Create(){ return std::make_shared< RobotModel >() ; }


    RobotModel();

    /**
     * @brief Initiliazes the robot model including wheels and chassis
     */
    void SetupRobot(ModelBasedPlannerConfig &config);

    void SetupRobot(const ProcConfig &procConfig, const RobotConfig &robotConfig,  const std::vector<WheelConfig> &wheelConfigs, const ChassisConfig &chassisConfig);

    /**
     * @brief Evaluate for a given DEM and pose
     */
    int EvaluatePose(const cv::Mat &dem, PoseEvalResults &results) const;

    /**
     * @brief Calculates the z-position of the base link
     */
    float GetBaseLinkZ(const PoseEvalResults &results);


    /**
     * @brief Calculate the wheel orientation in the world coordinate system for a cmd and robot angle
     */
    inline float GetWheelAngle(const WheelModel &wheel,const cv::Point2f &cmd, const float &robotAngle) const
    {
        //if () return robotAngle;
        if (!wheel.IsTurnable() || std::abs(cmd.y) < COMMANDEPSILON) return robotAngle;
        const float h = robotLengthHalf_*2.0f;
        const float r = (cmd.x*procConfig_.pixelSize)/cmd.y;
        const float modifier = r > 0? 1.0f:-1.0f;
        const float res = robotAngle+modifier*Utils_Math_Approx::fatan2(h,std::abs(r+wheel.config_.wheelPosRobot.y));
        return res;
    }

    /**
     * @brief Calculate the wheel orientation relative to the robot angle for a given command
     */
    inline float GetWheelAngleRobot(const WheelModel &wheel,const cv::Point2f &cmd) const
    {
        //if () return robotAngle;
        if (!wheel.IsTurnable() || std::abs(cmd.y) < COMMANDEPSILON) return 0;
        const float h = robotLengthHalf_*2.0f;
        const float r = (cmd.x*procConfig_.pixelSize)/cmd.y;
        const float modifier = r > 0? 1.0f:-1.0f;
        const float res = modifier*Utils_Math_Approx::fatan2(h,std::abs(r+wheel.config_.wheelPosRobot.y));
        return res;
    }

    /**
     * @brief Calculate the wheel orientation in the world coordinate system for a cmd and robot angle
     */
    inline float GetWheelAngleRobot(const int wheelIdx,const cv::Point2f &cmd) const
    {
        //if () return robotAngle;
        if (!wheels_[wheelIdx].IsTurnable() || std::abs(cmd.y) < COMMANDEPSILON) return 0;
        const float h = robotLengthHalf_*2.0f;
        const float r = (cmd.x*procConfig_.pixelSize)/cmd.y;
        const float modifier = r > 0? 1.0f:-1.0f;
        const float res = modifier*Utils_Math_Approx::fatan2(h,std::abs(r+wheels_[wheelIdx].config_.wheelPosRobot.y));
        return res;
    }

    /**
     * @brief Returns the orientation of all for wheels relative to the robot
     */
    inline cv::Vec4f GetWheelAnglesRobot(const cv::Point2f &cmd) const
    {
        cv::Vec4f results;
        results[0] = GetWheelAngleRobot(0,cmd);
        results[1] = GetWheelAngleRobot(1,cmd);
        results[2] = GetWheelAngleRobot(2,cmd);
        results[3] = GetWheelAngleRobot(3,cmd);
        return results;

    }

    /**
     * @brief Sets the wheel angle in world coordinates
     */
    inline void SetWheelAngle(const float robotAngle, const int &robotAngleIdx, const WheelModel &wheel, WheelEvalResults &res) const
    {
        if (!wheel.IsTurnable())
        {
            res.globalWheelAngle = robotAngle;
            res.wheelAngleIdx = robotAngleIdx;
        }
        else
        {
            res.globalWheelAngle = robotAngle+res.robotWheelAngle;
            res.wheelAngleIdx = GetAngleIdxFast(res.globalWheelAngle);
        }

    }


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


    const RobotDescriptor& GetDescriptor(const int &i) const {return descriptors_[i];}
    WheelModel& GetWheel(const int &i) {return wheels_[i];}
    ChassisModel& GetChassis() {return chassisModel_;}
    ProcConfig& GetProcConfig(){return procConfig_;}

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
        int res = (int)round(angle*procConfig_.angleStepInv);
        //if (res >= procConfig_.numAngleStep || res < 0)
        while (res >= procConfig_.numAngleStep)
        {
            res -= procConfig_.numAngleStep;
            //res = res%procConfig_.numAngleStep;
        }
        while (res < 0)
        {
            res += procConfig_.numAngleStep;
        }
        //if (res < 0) res = res%procConfig_.numAngleStep;
        return res;
    }


    /**
     * @brief Get descriptor index for a given angle without testing bounds
     */
    inline int GetAngleIdxFast(const float &angle) const
    {
        int res = (int)round(angle*procConfig_.angleStepInv);

        if (res >= procConfig_.numAngleStep)
        {
            res -= procConfig_.numAngleStep;

        }
        else if (res < 0)
        {
            res += procConfig_.numAngleStep;
        }

        return res;
    }



private:
    float angleStep_;

    cv::Point2f baseLinkPosImage_;


    std::vector<WheelModel> wheels_;
    std::vector<RobotDescriptor> descriptors_;
    ChassisModel chassisModel_;




    float frontWidthHalf_, backWidthHalf_;
    float robotLengthHalf_;
    float lw1_4,lw2_4,l_2, lw1_4Sqr, lw2_4Sqr;
    float w1_2, w2_2,w1pw2,w1mw2,mw1mw2;
    float fwZFactor,rwZFactor;


    ProcConfig procConfig_;
    RobotConfig config_;





};

#endif // ROBOTMODEL_H
