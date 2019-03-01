#ifndef DTPLANNERCONFIG_H
#define DTPLANNERCONFIG_H

#include <string>
#include <opencv2/core/core.hpp>
//#include "config_planner.h"

//#include "model_based_planner/config_planner.h"

#include "model_based_planner/config_planner.h"


/**
 * @brief Scoring parameters
 */
struct PlannerScorerConfigDT
{
    PlannerScorerConfigDT()
    {

        distanceThreshold = 0.0;
        dontCareDistance = 0.4;

        targetGoalDistance = 0.1;
        minPoseTime = 0.8;



        f_meanMeanDist = 0;
        f_meanMinDist = 0.0;
        f_minMeanDist = 0;
        f_minMinDist = 0.01;
        f_poseC = 0.0;
        f_aVelD = 0.0;

        f_goalDistance = 4.0;
        f_goalOrientation = -1.0;
        f_pathDistance = -1.0;
        f_lastCmdVelDiff = -0.0;

        end_outOfImage = 0;
        end_valid = 0;
        end_goalReached = 10000;
        end_poseCountLowPenalty = -100;
        end_collision = -10000;
        targetGoalDistance = 0.2;

        noWheelSupportNearThreshold = 1.4f;

        maxUpStep = 0.05;
        maxDownStep = 0.05;


    }

    float distanceThreshold; // in pixel
    float dontCareDistance;
    float maxUpStep;
    float maxDownStep;

    float minPoseTime; // in sec

    float f_meanMinDist; // factor
    float f_meanMeanDist;// factor
    float f_minMinDist;// factor
    float f_minMeanDist;// factor
    float f_poseC;// factor
    float f_aVelD;// factor
    float f_goalDistance; // factor
    float f_goalOrientation; // factor
    float f_lastCmdVelDiff; // factor
    float f_pathDistance; // factor

    float end_outOfImage; // offset
    float end_valid;// offset
    float end_goalReached;// offset
    float end_poseCountLowPenalty;// offset
    float end_collision;// offset

    float targetGoalDistance; // in m
    float noWheelSupportNearThreshold; // in m

    //Calculated
    float targetGoalDistanceImage; // in pixels
    float targetGoalDistanceImageSqr; // in pixels^2

    float distanceThresholdImg; // in pixel
    float dontCareDistanceImg;
    float noWheelSupportNearThresholdImg;

    void Setup(float pixelSize)
    {
        targetGoalDistanceImage = targetGoalDistance/pixelSize;
        targetGoalDistanceImageSqr = targetGoalDistanceImage*targetGoalDistanceImage;
        distanceThresholdImg = distanceThreshold/pixelSize;
        dontCareDistanceImg = dontCareDistance/pixelSize;
        noWheelSupportNearThresholdImg = noWheelSupportNearThreshold/pixelSize;

    }


};


/**
 * @brief Configuration for one testpoint
 */
struct TestPointConfig
{
    TestPointConfig()
    {
        radius = 0;
        radiusImg = 0;
        pointPosRobot.x = 0;
        pointPosRobot.y = 0;

    }


    float radius;
    float radiusImg;
    cv::Point2f pointPosRobot;

};



/**
 * @brief Vehicle parameters
 */
struct RobotConfigDT
{
    RobotConfigDT()
    {
        numAngleStep = 360;

        baseLinkPosCoord.x = 0;
        baseLinkPosCoord.y = 0;

    }

    void Setup(float pixelSize)
    {
        angleStep = (CV_PI*2.0)/(double)numAngleStep;
        angleStepInv = 1.0/angleStep;

        testPoints.clear();
        for (int tl = 0; tl < testPointMat.rows;++tl)
        {
            TestPointConfig conf;

            conf.pointPosRobot.x = testPointMat.at<float>(tl,0);
            conf.pointPosRobot.y = testPointMat.at<float>(tl,1);
            conf.radius = testPointMat.at<float>(tl,2);
            conf.radiusImg = conf.radius/pixelSize;

            testPoints.push_back(conf);

        }

    }

    /**
     * @brief In case the base link is not the center of the vehicle, the base link position in vehicle coordinates should be set here
     */
    cv::Point2f baseLinkPosCoord;

    int numAngleStep;

    cv::Mat testPointMat;


//Calculated
    float angleStep;
    float angleStepInv;

    std::vector<TestPointConfig> testPoints;


};





/**
 * @brief Config describing the parameters of the DEM required for creating the vehicle model
 *
 */
struct ProcConfigDT
{
public:
    ProcConfigDT()
    {
        pixelSize  = 0.01;

        imagePosBLMinX = 0;
        imagePosBLMinY = 0;

        mapBaseHeight = 10000;

        notVisibleLevel = 1000;

        heightScale = 1000;

        Setup();


    }

    void Setup(){
        pixelSizeInv = 1.0/pixelSize;

    }



    float pixelSize;

    float mapBaseHeight;

    float notVisibleLevel;

    float heightScale;




//calculated

    float imagePosBLMinX,imagePosBLMinY;

    float pixelSizeInv;



};


/**
 * @brief Wheel config for reading from yaml, the config is converted to four configs for each wheel
 */
/*
struct WheelsConfigDT
{
    WheelsConfigDT()
    {
        wheelPosRobotFrontX = 1.0;
        wheelPosRobotRearX = -1.0;
        wheelPosRobotRearY = 1.0;
        wheelPosRobotFrontY = 1.0;
        wheelRadiusRear = 0.1;
        wheelRadiusFront = 0.1;

    }

    float wheelPosRobotFrontX;
    float wheelPosRobotRearX;
    float wheelPosRobotRearY;
    float wheelPosRobotFrontY;

    float wheelRadiusRear;
    float wheelRadiusFront;


};
*/


/**
 * @brief Complete config for the model based planner
 */
struct DTPlannerConfig
{

    static void ReadRobotConf(cv::FileStorage &fs, RobotConfigDT &rc)
    {
        cv::FileNode n = fs["Robot"];                                // Read mappings from a sequence
        n["baseLinkPosCoord"] >> rc.baseLinkPosCoord;
        n["numAngleStep"] >> rc.numAngleStep;
        n["testpoints"] >> rc.testPointMat;
        //rc.chassisTestTipAngleThreshold = (float)(n["chassisTestTipAngleThreshold"]);

    }
    /*
    static void ReadWheelConf(cv::FileStorage &fs, WheelsConfigDT &wc)
    {
        cv::FileNode n = fs["Wheels"];                                // Read mappings from a sequence
        wc.wheelPosRobotFrontX = (float)(n["wheelPosRobotFrontX"]);
        wc.wheelPosRobotRearX = (float)(n["wheelPosRobotRearX"]);
        wc.wheelPosRobotFrontY = (float)(n["wheelPosRobotFrontY"]);
        wc.wheelPosRobotRearY = (float)(n["wheelPosRobotRearY"]);

        wc.wheelRadiusFront = (float)(n["wheelRadiusFront"]);

        wc.wheelRadiusRear = (float)(n["wheelRadiusRear"]);


    }
    */


    static void ReadProcConf(cv::FileStorage &fs, ProcConfigDT &pc)
    {
        cv::FileNode n = fs["Proc"];                                // Read mappings from a sequence

        pc.pixelSize = (float)(n["pixelSize"]);
        pc.mapBaseHeight = (float)(n["mapBaseHeight"]);
        pc.notVisibleLevel = (float)(n["notVisibleLevel"]);
        pc.heightScale = (float)(n["heightScale"]);

        pc.Setup();


    }

    DTPlannerConfig()
    {
        plannerType_ = "AStar";
        nodeExpanderType_ = "angular_vel";
        scorerType_ = "goal_scorer";

    }

    void Setup()
    {
        procConfig_.Setup();
        plannerConfig_.Setup();
        robotConfig_.Setup(procConfig_.pixelSize);
        scorerConfig_.Setup(procConfig_.pixelSize);

        plannerConfig_.lookAheadTime = 5.0;
        plannerConfig_.maxLevel = 4;
        plannerConfig_.maxSearchIterations = 10000;
        plannerConfig_.minNumberNodes = 10;
        plannerConfig_.numSubSamples = 20;
        plannerConfig_.replanFactor = -1;



        expanderConfig_.deltaTheta = 0.1;
        expanderConfig_.numSplits = 9;
        expanderConfig_.firstLevelSplits = -1;
        expanderConfig_.firstLevelDeltaTheta = -1;
        expanderConfig_.firstLevelLinearSplits = 3;
        expanderConfig_.firstLevelDeltaLinear = 0.1;
        expanderConfig_.minLinVel = 0.1;
        expanderConfig_.maxLinVel = 0.5;
        expanderConfig_.maxAngVel = 3.14159265359;

    }



    bool ReadRobotDescription(std::string filename)
    {
        if (filename.length() < 2) return false;
        cv::FileStorage fs;
        fs.open(filename, cv::FileStorage::READ);
        if (!fs.isOpened()) return false;

        std::string configFolder = getFolderName(filename);

        ReadRobotConf(fs,robotConfig_);
        //ReadChassisConf(fs,chassisConfig_,configFolder);
        //ReadWheelConf(fs,wheelsConfig_);

        fs.release();

        Setup();
        return true;
    }

    bool ReadMapDescription(std::string filename)
    {
        if (filename.length() < 2) return false;
        cv::FileStorage fs;
        fs.open(filename, cv::FileStorage::READ);
        if (!fs.isOpened()) return false;

        ReadProcConf(fs,procConfig_);

        fs.release();

        Setup();
        return true;

    }

    /*
    std::vector<TestPointConfig> GetPointConfigs() const
    {

        std::vector<TestPointConfig> pointConfs;
        TestPointConfig tconf;// = wm1.GetConfig();

        tconf.radius = wheelsConfig_.wheelRadiusRear;
        //wconf.width = wheelsConfig_.wheelWidthRear;


        tconf.pointPosRobot.x = wheelsConfig_.wheelPosRobotRearX;
        tconf.pointPosRobot.y = wheelsConfig_.wheelPosRobotRearY;
        tconf.radiusImg = tconf.radius*procConfig_.pixelSizeInv;

        pointConfs.push_back(tconf);


        tconf.pointPosRobot.x = wheelsConfig_.wheelPosRobotRearX;
        tconf.pointPosRobot.y = -wheelsConfig_.wheelPosRobotRearY;
        tconf.radiusImg = tconf.radius*procConfig_.pixelSizeInv;

        pointConfs.push_back(tconf);

        tconf.radius = wheelsConfig_.wheelRadiusFront;
        //wconf.width = wheelsConfig_.wheelWidthFront;

        tconf.pointPosRobot.x = wheelsConfig_.wheelPosRobotFrontX;
        tconf.pointPosRobot.y = -wheelsConfig_.wheelPosRobotFrontY;
        tconf.radiusImg = tconf.radius*procConfig_.pixelSizeInv;

        pointConfs.push_back(tconf);

        tconf.pointPosRobot.x = wheelsConfig_.wheelPosRobotFrontX;
        tconf.pointPosRobot.y = wheelsConfig_.wheelPosRobotFrontY;
        tconf.radiusImg = tconf.radius*procConfig_.pixelSizeInv;

        pointConfs.push_back(tconf);

        return pointConfs;
    }
    */

    ProcConfigDT procConfig_;
    RobotConfigDT robotConfig_;
    PlannerConfig plannerConfig_;
    PlannerScorerConfigDT scorerConfig_;
    PlannerExpanderConfig expanderConfig_;
    //WheelsConfigDT wheelsConfig_;

    std::string plannerType_;
    std::string nodeExpanderType_;
    std::string scorerType_;


    std::string getFolderName(const std::string &s);


};


#endif // MODELBASEDPLANNERCONFIG_H
