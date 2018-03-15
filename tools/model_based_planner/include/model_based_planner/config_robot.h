#ifndef ROBOTCONFIG_H
#define ROBOTCONFIG_H
#include <opencv2/core/core.hpp>

/**
 * @brief Vehicle parameters
 */
struct RobotConfig
{
    RobotConfig()
    {
        baseLinkPosCoord.x = 0;
        baseLinkPosCoord.y = 0;

        chassisTestTipAngleThreshold = 0.99;
    }

    /**
     * @brief In case the base link is not the center of the vehicle, the base link position in vehicle coordinates should be set here
     */
    cv::Point2f baseLinkPosCoord;
    /**
     * @brief only test two different vehice orientations if the cos(angle) is below this threshold
     */
    float chassisTestTipAngleThreshold;


};

/**
 * @brief Configuration for one wheel
 */
struct WheelConfig
{
    WheelConfig()
    {
        radius = 0;
        latRadius = 0;
        width = 0;
        jointPosWheel.x = 0;
        jointPosWheel.y = 0;
        wheelPosRobot.x = 0;
        wheelPosRobot.y = 0;

        rotTestSteps = 0;
        rotTestStepSize = 0;
        isTurnableWheel = 0;

    }


    float radius,latRadius,width;
    cv::Point2f jointPosWheel;
    cv::Point2f wheelPosRobot;

    int rotTestSteps;
    int rotTestStepSize;
    bool isTurnableWheel;




};


/**
 * @brief Chassis configuration
 */
struct ChassisConfig
{
    ChassisConfig()
    {
        chassisPosRobot.x = 0;
        chassisPosRobot.y = 0;
        chassisImageCenter.x = 0;
        chassisImageCenter.y = 0;
        chassisModelYSize = 1.0;
        chassisImageValueOffset = 0;
        chassisImageValueScale = 1.0;
        testChassis = false;
        chassisfileName = "";
    }


    std::string chassisfileName;

    cv::Point2f chassisPosRobot;
    cv::Point2f chassisImageCenter;

    float chassisModelYSize;
    float chassisImageValueScale;
    float chassisImageValueOffset;

    bool testChassis;

};


#endif // ROBOTCONFIG_H
