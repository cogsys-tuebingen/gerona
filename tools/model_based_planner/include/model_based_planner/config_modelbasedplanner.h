#ifndef MODELBASEDPLANNERCONFIG_H
#define MODELBASEDPLANNERCONFIG_H

#include <string>

#include <config_proc.h>
#include <config_robot.h>
#include <config_planner.h>



/**
 * @brief Wheel config for reading from yaml, the config is converted to four configs, one for each wheel
 */
struct WheelsConfig
{
    WheelsConfig()
    {
        wheelPosRobotFrontX = 1.0;
        wheelPosRobotRearX = -1.0;
        wheelPosRobotRearY = 1.0;
        wheelPosRobotFrontY = 1.0;
        wheelJointPosRear = cv::Point2f(0,0);
        wheelLatRadiusRear = 0.1;
        wheelRadiusRear = 0.1;
        wheelWidthRear = 0.1;
        wheelJointPosFront = cv::Point2f(0,0);
        wheelLatRadiusFront = 0.1;
        wheelRadiusFront = 0.1;
        wheelWidthFront = 0.1;
        frontWheelsTurnable = 0;
        wheelRotTestSteps = 0;
        wheelRotTestStepSize = 0;

    }

    // All positions are in the vehicle frame

    float wheelPosRobotFrontX;      // x-position of the front wheels
    float wheelPosRobotRearX;       // x-position of the rear wheels
    float wheelPosRobotRearY;       // y-position of the rear wheels
    float wheelPosRobotFrontY;      // y-position of the front wheels
    cv::Point2f wheelJointPosRear;  // pivot point for the rear wheels
    float wheelLatRadiusRear;       // lateral radius of the rear wheels
    float wheelRadiusRear;          // radius of the rear wheels
    float wheelWidthRear;           // width of the rear wheels
    cv::Point2f wheelJointPosFront; // pivot point for the front wheels
    float wheelLatRadiusFront;      // lateral radius of the front wheels
    float wheelRadiusFront;         // radius of the front wheels
    float wheelWidthFront;          // width of the front wheels
    bool frontWheelsTurnable;       // are the front wheels turnable
    bool rearWheelsTurnable;        // are the rear wheels turnable
    int wheelRotTestSteps;          // number of test steps
    int wheelRotTestStepSize;       // angle step size of tests


};


/**
 * @brief Complete config for the model based planner
 */
struct ModelBasedPlannerConfig
{

    /**
     * @brief Functions for writing the config to files
     */
    /* Currently not used
    static void WriteRobotConf(cv::FileStorage &fs, RobotConfig &rc)
    {
        fs << "Robot";                              // text - mapping
        fs << "{" << "baseLinkPosCoord" << rc.baseLinkPosCoord << "}";
    }
    static void WriteWheelConf(cv::FileStorage &fs, WheelsConfig &wc)
    {
        fs << "Wheels";                              // text - mapping
        fs << "{";
        fs << "wheelPosRobotX" << wc.wheelPosRobotX;
        fs << "wheelPosRobotFrontY" << wc.wheelPosRobotFrontY;
        fs << "wheelPosRobotRearY" << wc.wheelPosRobotRearY;

        fs << "wheelJointPosFront" << wc.wheelJointPosFront;
        fs << "wheelRadiusFront" << wc.wheelRadiusFront;
        fs << "wheelWidthFront" << wc.wheelWidthFront;
        fs << "wheelLatRadiusFront" << wc.wheelLatRadiusFront;

        fs << "wheelJointPosRear" << wc.wheelJointPosRear;
        fs << "wheelRadiusRear" << wc.wheelRadiusRear;
        fs << "wheelWidthRear" << wc.wheelWidthRear;
        fs << "wheelLatRadiusRear" << wc.wheelLatRadiusRear;

        fs << "}";
    }
    static void WriteChassisConf(cv::FileStorage &fs, ChassisConfig &cc)
    {
        fs << "Chassis";                              // text - mapping
        fs << "{";
        fs << "chassisfileName" << cc.chassisfileName;
        fs << "chassisImageCenter" << cc.chassisImageCenter;
        fs << "chassisImageValueOffset" << cc.chassisImageValueOffset;
        fs << "chassisImageValueScale" << cc.chassisImageValueScale;
        fs << "chassisModelYSize" << cc.chassisModelYSize;
        fs << "chassisPosRobot" << cc.chassisPosRobot;
        fs << "testChassis" << cc.testChassis;
        fs << "}";
    }

    static void WriteProcConf(std::string filename, ProcConfig &pc)
    {
        cv::FileStorage fs;
        fs.open(filename, cv::FileStorage::WRITE);

        fs << "Chassis";                              // text - mapping
        fs << "{";
        fs << "numAngleStep" << pc.numAngleStep;
        fs << "heightScale" << pc.heightScale;
        fs << "mapBaseHeight" << pc.mapBaseHeight;
        fs << "wheelGroundLevel" << pc.wheelGroundLevel;
        fs << "maxHeight" << pc.maxHeight;
        fs << "pixelSize" << pc.pixelSize;
        fs << "imagePosBLMinX" << pc.imagePosBLMinX;
        fs << "imagePosBLMinY" << pc.imagePosBLMinY;
        fs << "validThresholdFactor" << pc.validThresholdFactor;
        fs << "convertImage" << pc.convertImage;
        fs << "}";

        fs.release();

    }
    */

    /**
     * @brief Read a robot config, see RobotConfig for parameter description
     */
    static void ReadRobotConf(cv::FileStorage &fs, RobotConfig &rc)
    {
        cv::FileNode n = fs["Robot"];
        n["baseLinkPosCoord"] >> rc.baseLinkPosCoord; // position of the base link frame in the vehicle frame
        rc.chassisTestTipAngleThreshold = (float)(n["chassisTestTipAngleThreshold"]); // if tip angle is above this threshold the chassis is tested for both configurations

    }
    /**
     * @brief Read the wheel config. See WheelsConfig for parameters description
     */
    static void ReadWheelConf(cv::FileStorage &fs, WheelsConfig &wc)
    {
        // See WheelsConfig for description
        cv::FileNode n = fs["Wheels"];
        wc.wheelPosRobotFrontX = (float)(n["wheelPosRobotFrontX"]);
        wc.wheelPosRobotRearX = (float)(n["wheelPosRobotRearX"]);
        wc.wheelPosRobotFrontY = (float)(n["wheelPosRobotFrontY"]);
        wc.wheelPosRobotRearY = (float)(n["wheelPosRobotRearY"]);
        n["wheelJointPosFront"] >> wc.wheelJointPosFront;

        wc.wheelRadiusFront = (float)(n["wheelRadiusFront"]);
        wc.wheelWidthFront = (float)(n["wheelWidthFront"]);
        wc.wheelLatRadiusFront = (float)(n["wheelLatRadiusFront"]);
        n["wheelJointPosRear"] >> wc.wheelJointPosRear;

        wc.wheelRadiusRear = (float)(n["wheelRadiusRear"]);
        wc.wheelWidthRear = (float)(n["wheelWidthRear"]);
        wc.wheelLatRadiusRear = (float)(n["wheelLatRadiusRear"]);

        wc.frontWheelsTurnable = (int)(n["frontWheelsTurnable"]);
        wc.wheelRotTestSteps = (int)(n["wheelRotTestSteps"]);
        wc.wheelRotTestStepSize = (int)(n["wheelRotTestStepSize"]);


    }
    /**
     * @brief Read the chassis config. See ChassisConfig for parameter description
     */
    static void ReadChassisConf(cv::FileStorage &fs, ChassisConfig &cc, std::string configFolder)
    {
        cv::FileNode n = fs["Chassis"];
        cc.chassisfileName = configFolder+"/"+(std::string)(n["chassisfileName"]);

        n["chassisImageCenter"] >> cc.chassisImageCenter;
        n["chassisPosRobot"] >> cc.chassisPosRobot;


        cc.chassisModelYSize = (float)(n["chassisModelYSize"]);
        cc.chassisImageValueScale = (float)(n["chassisImageValueScale"]);
        cc.chassisImageValueOffset = (float)(n["chassisImageValueOffset"]);

        cc.testChassis = (int)(n["testChassis"]);

    }

    /**
     * @brief Read the processing config. See ProcConfig for parameter description
     */
    static void ReadProcConf(cv::FileStorage &fs, ProcConfig &pc)
    {
        cv::FileNode n = fs["Proc"];

        pc.numAngleStep = (int)(n["numAngleStep"]);
        pc.heightScale = (float)(n["heightScale"]);
        pc.mapBaseHeight = (int)(n["mapBaseHeight"]);
        pc.wheelGroundLevel = (int)(n["wheelGroundLevel"]);
        pc.maxHeight = (int)(n["maxHeight"]);
        pc.pixelSize = (float)(n["pixelSize"]);
        pc.validThresholdFactor = (float)(n["validThresholdFactor"]);

        pc.Setup();


    }

    /**
     * @brief Constructor
     */
    ModelBasedPlannerConfig()
    {
        plannerType_ = "AStar";
        nodeExpanderType_ = "angular_vel";
        scorerType_ = "path_scorer";

    }

    /**
     * @brief calculate all values that can be pre-calculated.
     */
    void Setup()
    {
        procConfig_.Setup();
        plannerConfig_.Setup();
        scorerConfig_.Setup(procConfig_.pixelSize);

    }



    /**
     * @brief read the complete robot description
     */
    bool ReadRobotDescription(std::string filename)
    {
        if (filename.length() < 2) return false;
        cv::FileStorage fs;
        fs.open(filename, cv::FileStorage::READ);
        if (!fs.isOpened()) return false;

        std::string configFolder = getFolderName(filename);

        ReadRobotConf(fs,robotConfig_);
        ReadChassisConf(fs,chassisConfig_,configFolder);
        ReadWheelConf(fs,wheelsConfig_);

        fs.release();
        return true;
    }

    /**
     * @brief read processing parameters
     */
    bool ReadMapDescription(std::string filename)
    {
        if (filename.length() < 2) return false;
        cv::FileStorage fs;
        fs.open(filename, cv::FileStorage::READ);
        if (!fs.isOpened()) return false;

        ReadProcConf(fs,procConfig_);

        fs.release();
        return true;

    }

    /**
     * @brief convert the WheelsConfig into separate WheelConfigs, one for each wheel
     */
    std::vector<WheelConfig> GetWheelConfigs() const
    {

        std::vector<WheelConfig> wheelConfs;
        WheelConfig wconf;// = wm1.GetConfig();


        wconf.isTurnableWheel = false;
        wconf.rotTestSteps = 0;
        wconf.rotTestStepSize = 0;

        wconf.jointPosWheel.x = wheelsConfig_.wheelJointPosRear.x;
        wconf.jointPosWheel.y = wheelsConfig_.wheelJointPosRear.y;
        wconf.latRadius = wheelsConfig_.wheelLatRadiusRear;
        wconf.radius = wheelsConfig_.wheelRadiusRear;
        wconf.width = wheelsConfig_.wheelWidthRear;


        wconf.wheelPosRobot.x = wheelsConfig_.wheelPosRobotRearX;
        wconf.wheelPosRobot.y = wheelsConfig_.wheelPosRobotRearY;

        wheelConfs.push_back(wconf);


        wconf.wheelPosRobot.x = wheelsConfig_.wheelPosRobotRearX;
        wconf.wheelPosRobot.y = -wheelsConfig_.wheelPosRobotRearY;

        wheelConfs.push_back(wconf);



        wconf.isTurnableWheel = wheelsConfig_.frontWheelsTurnable;
        wconf.rotTestSteps = wheelsConfig_.wheelRotTestSteps;
        wconf.rotTestStepSize = wheelsConfig_.wheelRotTestStepSize;

        wconf.jointPosWheel.x = wheelsConfig_.wheelJointPosFront.x;
        wconf.jointPosWheel.y = wheelsConfig_.wheelJointPosFront.y;
        wconf.latRadius = wheelsConfig_.wheelLatRadiusFront;
        wconf.radius = wheelsConfig_.wheelRadiusFront;
        wconf.width = wheelsConfig_.wheelWidthFront;

        wconf.wheelPosRobot.x = wheelsConfig_.wheelPosRobotFrontX;
        wconf.wheelPosRobot.y = -wheelsConfig_.wheelPosRobotFrontY;

        wheelConfs.push_back(wconf);

        wconf.wheelPosRobot.x = wheelsConfig_.wheelPosRobotFrontX;
        wconf.wheelPosRobot.y = wheelsConfig_.wheelPosRobotFrontY;

        wheelConfs.push_back(wconf);

        return wheelConfs;
    }

    /**
     * @brief The configurations
     */
    ProcConfig procConfig_;
    RobotConfig robotConfig_;
    ChassisConfig chassisConfig_;
    PlannerConfig plannerConfig_;
    PlannerScorerConfig scorerConfig_;
    PlannerExpanderConfig expanderConfig_;
    WheelsConfig wheelsConfig_;

    /**
     * @brief The types to use for planner, expander and scorer.
     */
    std::string plannerType_;
    std::string nodeExpanderType_;
    std::string scorerType_;


    /**
     * @brief Get folder name of config files to read imges from there
     */
    std::string getFolderName(const std::string &s);


};


#endif // MODELBASEDPLANNERCONFIG_H
