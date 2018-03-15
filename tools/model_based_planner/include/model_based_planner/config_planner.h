
#ifndef PLANNER_CONFIG
#define PLANNER_CONFIG

/**
 * @brief General parameters for path planners
 */
struct PlannerConfig
{
    PlannerConfig()
    {
        maxSearchIterations = 10000;
        maxLevel = 6;
        numSubSamples = 10;
        lookAheadTime = 2.0;
        trajectoryTimeStep = 0.3;
        subSampleTimeStep = 0.03;
    }

    void Setup()
    {
        trajectoryTimeStep = lookAheadTime/(float)maxLevel;
        subSampleTimeStep = trajectoryTimeStep/(float)numSubSamples;
    }

    int maxSearchIterations;
    int maxLevel;
    int numSubSamples;
    float lookAheadTime;


    // calculated
    float trajectoryTimeStep;
    float subSampleTimeStep;


};


/**
 * @brief Scoring parameters
 */
struct PlannerScorerConfig
{
    PlannerScorerConfig()
    {
        //gravAngleThreshold = 0.9;
        gravAngleThreshold = 0.2;
        deltaAngleThreshold = 0.1;

        tipAngleThreshold = 0.1;
        minWheelSupportThreshold = 0.5;
        targetGoalDistance = 0.1;
        minPoseTime = 0.8;
        noWheelSupportNearThreshold = 1.0;
        noWheelSupportRotateThreshold = 0.7;



        f_meanGA = 0;
        f_maxGA = -10.0;
        f_meanAD = 0;
        f_maxAD = -10.0;
        f_meanTA = 0;
        f_maxTA = -10.0;
        f_poseC = 0.0;
        f_aVelD = -10.0;
        f_meanWS = 0.0;
        f_minWS = 10.0;
        f_numNotVisible = -1.0;

        f_goalDistance = 1.0;
        f_goalOrientation = -2.0;
        f_lastCmdVelDiff = -2.0;

        end_outOfImage = 0;
        end_noWheelSupport = -10;
        end_noWheelSupportFar = -1;
        end_notVisible = 0;
        end_valid = 0;
        end_exceedAngle = -100;
        end_goalReached = 10000;
        end_poseCountLowPenalty = -100;
        end_chassisCollision = -9997;
        allowNotVisible = true;


    }

    float gravAngleThreshold; // in rad
    float deltaAngleThreshold; // in rad
    float tipAngleThreshold; // in rad
    float minWheelSupportThreshold; // in percent
    bool allowNotVisible;
    float noWheelSupportNearThreshold; // in sec
    float noWheelSupportRotateThreshold; // in rad
    float minPoseTime; // in sec

    float f_meanGA; // factor
    float f_maxGA;// factor
    float f_meanAD;// factor
    float f_maxAD;// factor
    float f_meanTA;// factor
    float f_maxTA;// factor
    float f_poseC;// factor
    float f_aVelD;// factor
    float f_meanWS;// factor
    float f_minWS;// factor
    float f_numNotVisible;// factor
    float f_goalDistance; // factor
    float f_goalOrientation; // factor
    float f_lastCmdVelDiff;

    float end_outOfImage; // offset
    float end_noWheelSupport; // offset
    float end_noWheelSupportFar;// offset
    float end_notVisible;// offset
    float end_valid;// offset
    float end_exceedAngle;// offset
    float end_goalReached;// offset
    float end_poseCountLowPenalty;// offset
    float end_chassisCollision;// offset

    float targetGoalDistance; // in m
    float targetGoalDistanceImage; // in pixels
    float targetGoalDistanceImageSqr; // in pixels^2

    void Setup(float pixelSize)
    {
        targetGoalDistanceImage = targetGoalDistance/pixelSize;
        targetGoalDistanceImageSqr = targetGoalDistanceImage*targetGoalDistanceImage;
    }


};


/**
 * @brief General parameters for path planners
 */
struct PlannerExpanderConfig
{
    PlannerExpanderConfig()
    {
        deltaTheta = 1.0;
        numSplits = 3;
        firstLevelSplits = -1;
        firstLevelDeltaTheta = 0;
        firstLevelLinearSplits = 3;
        firstLevelDeltaLinear = 0.1;

        minLinVel = 0.1;
        maxLinVel = 1.0;

    }

    float deltaTheta;
    int numSplits;
    int firstLevelSplits;
    int firstLevelLinearSplits;

    float firstLevelDeltaTheta;
    float firstLevelDeltaLinear;

    float minLinVel,maxLinVel;

};



#endif //PLANNER_SCORER_CONFIG
