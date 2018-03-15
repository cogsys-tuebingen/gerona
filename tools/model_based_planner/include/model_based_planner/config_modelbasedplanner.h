#ifndef MODELBASEDPLANNERCONFIG_H
#define MODELBASEDPLANNERCONFIG_H

#include <string>

#include <config_proc.h>
#include <config_robot.h>
#include <config_planner.h>



/**
 * @brief Wheel config for reading from yaml, the config is converted to four configs for each wheel
 */
struct WheelsConfig
{
    WheelsConfig()
    {
        wheelPosRobotX = 1.0;
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

    float wheelPosRobotX;
    float wheelPosRobotRearY;
    float wheelPosRobotFrontY;
    cv::Point2f wheelJointPosRear;
    float wheelLatRadiusRear;
    float wheelRadiusRear;
    float wheelWidthRear;
    cv::Point2f wheelJointPosFront;
    float wheelLatRadiusFront;
    float wheelRadiusFront;
    float wheelWidthFront;
    bool frontWheelsTurnable;
    int wheelRotTestSteps;
    int wheelRotTestStepSize;


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
    */

    /*
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

    static void ReadRobotConf(cv::FileStorage &fs, RobotConfig &rc)
    {
        cv::FileNode n = fs["Robot"];                                // Read mappings from a sequence
        n["baseLinkPosCoord"] >> rc.baseLinkPosCoord;
        rc.chassisTestTipAngleThreshold = (float)(n["chassisTestTipAngleThreshold"]);

    }
    static void ReadWheelConf(cv::FileStorage &fs, WheelsConfig &wc)
    {
        cv::FileNode n = fs["Wheels"];                                // Read mappings from a sequence
        wc.wheelPosRobotX = (float)(n["wheelPosRobotX"]);
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
    static void ReadChassisConf(cv::FileStorage &fs, ChassisConfig &cc, std::string configFolder)
    {
        cv::FileNode n = fs["Chassis"];                                // Read mappings from a sequence
        cc.chassisfileName = configFolder+"/"+(std::string)(n["chassisfileName"]);

        n["chassisImageCenter"] >> cc.chassisImageCenter;
        n["chassisPosRobot"] >> cc.chassisPosRobot;


        cc.chassisModelYSize = (float)(n["chassisModelYSize"]);
        cc.chassisImageValueScale = (float)(n["chassisImageValueScale"]);
        cc.chassisImageValueOffset = (float)(n["chassisImageValueOffset"]);

        cc.testChassis = (int)(n["testChassis"]);

    }

    static void ReadProcConf(cv::FileStorage &fs, ProcConfig &pc)
    {
        cv::FileNode n = fs["Proc"];                                // Read mappings from a sequence

        pc.numAngleStep = (int)(n["numAngleStep"]);
        pc.heightScale = (float)(n["heightScale"]);
        pc.mapBaseHeight = (int)(n["mapBaseHeight"]);
        pc.wheelGroundLevel = (int)(n["wheelGroundLevel"]);
        pc.maxHeight = (int)(n["maxHeight"]);
        pc.pixelSize = (float)(n["pixelSize"]);
        pc.imagePosBLMinX = (float)(n["imagePosBLMinX"]);
        pc.imagePosBLMinY = (float)(n["imagePosBLMinY"]);
        pc.validThresholdFactor = (float)(n["validThresholdFactor"]);
        pc.convertImage = (int)(n["convertImage"]);

        pc.Setup();


    }

    ModelBasedPlannerConfig()
    {
        plannerType_ = "AStar_AngularVel_WSPL";
    }

    void Setup()
    {
        procConfig_.Setup();
        plannerConfig_.Setup();
        scorerConfig_.Setup(procConfig_.pixelSize);

    }



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

    std::vector<WheelConfig> GetWheelConfigs() const
    {

        std::vector<WheelConfig> wheelConfs;
        WheelConfig wconf;// = wm1.GetConfig();

        float wheelXpos = wheelsConfig_.wheelPosRobotX;
        float wheelYPosR = wheelsConfig_.wheelPosRobotRearY;
        float wheelYPosF = wheelsConfig_.wheelPosRobotFrontY;

        wconf.isTurnableWheel = false;
        wconf.rotTestSteps = 0;
        wconf.rotTestStepSize = 0;

        wconf.jointPosWheel.x = wheelsConfig_.wheelJointPosRear.x;
        wconf.jointPosWheel.y = wheelsConfig_.wheelJointPosRear.y;
        wconf.latRadius = wheelsConfig_.wheelLatRadiusRear;
        wconf.radius = wheelsConfig_.wheelRadiusRear;
        wconf.width = wheelsConfig_.wheelWidthRear;


        wconf.wheelPosRobot.x = -wheelXpos;
        wconf.wheelPosRobot.y = wheelYPosR;

        wheelConfs.push_back(wconf);


        wconf.wheelPosRobot.x = -wheelXpos;
        wconf.wheelPosRobot.y = -wheelYPosR;

        wheelConfs.push_back(wconf);



        wconf.isTurnableWheel = wheelsConfig_.frontWheelsTurnable;
        wconf.rotTestSteps = wheelsConfig_.wheelRotTestSteps;
        wconf.rotTestStepSize = wheelsConfig_.wheelRotTestStepSize;

        wconf.jointPosWheel.x = wheelsConfig_.wheelJointPosFront.x;
        wconf.jointPosWheel.y = wheelsConfig_.wheelJointPosFront.y;
        wconf.latRadius = wheelsConfig_.wheelLatRadiusFront;
        wconf.radius = wheelsConfig_.wheelRadiusFront;
        wconf.width = wheelsConfig_.wheelWidthFront;

        wconf.wheelPosRobot.x = wheelXpos;
        wconf.wheelPosRobot.y = -wheelYPosF;

        wheelConfs.push_back(wconf);

        wconf.wheelPosRobot.x = wheelXpos;
        wconf.wheelPosRobot.y = wheelYPosF;

        wheelConfs.push_back(wconf);

        return wheelConfs;
    }

    ProcConfig procConfig_;
    RobotConfig robotConfig_;
    ChassisConfig chassisConfig_;
    PlannerConfig plannerConfig_;
    PlannerScorerConfig scorerConfig_;
    PlannerExpanderConfig expanderConfig_;
    WheelsConfig wheelsConfig_;

    std::string plannerType_;

    std::string getFolderName(const std::string &s);


};


#endif // MODELBASEDPLANNERCONFIG_H
