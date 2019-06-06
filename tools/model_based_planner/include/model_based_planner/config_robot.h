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


    /**
     * @brief wheel geometry parameters
     */
    float radius,latRadius,width;
    /**
     * @brief pivot point position in wheel coordinates
     */
    cv::Point2f jointPosWheel;
    /**
     * @brief wheel position in robot coordinates
     */
    cv::Point2f wheelPosRobot;

    /**
     * @brief number of test steps for wheel test
     */
    int rotTestSteps;
    /**
     * @brief rotation test step size
     */
    int rotTestStepSize;
    /**
     * @brief can this wheel turn?
     */
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


    /**
     * @brief file name of the chassis height image
     */
    std::string chassisfileName;

    /**
     * @brief position of the chassis center in robot coordinates
     */
    cv::Point2f chassisPosRobot;
    /**
     * @brief center of the chassis image in image coordinates. Defaults to image center. can be used to describe assymetric chassis
     */
    cv::Point2f chassisImageCenter;

    /**
     * @brief height of the chassis model in meters. Used for scaling the chassis image
     */
    float chassisModelYSize;
    /**
     * @brief factor for scaling the pixel values (height) of the chassis image
     */
    float chassisImageValueScale;
    /**
     * @brief this offset is added to the scaled image values
     */
    float chassisImageValueOffset;

    /**
     * @brief test for chassis collision?
     */
    bool testChassis;

};


#endif // ROBOTCONFIG_H
