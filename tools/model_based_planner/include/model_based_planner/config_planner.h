
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
        replanFactor = -1;
        minNumberNodes = -1;
    }

    /**
     * @brief Pre-calculate fixed values
     */
    void Setup()
    {
        trajectoryTimeStep = lookAheadTime/(float)maxLevel;
        subSampleTimeStep = trajectoryTimeStep/(float)numSubSamples;
    }

    /**
     * @brief Search terminates after maxSearchIterations
     */
    int maxSearchIterations;
    /**
     * @brief Tree nodes are not expanded beyond this level
     */
    int maxLevel;
    /**
     * @brief The numbers of pose checks for one sub trajectory
     */
    int numSubSamples;
    /**
     * @brief time horizon for planning
     */
    float lookAheadTime;
    /**
     * @brief if planning failed and this number is > 1 a new planning is initiated, alle search parameters are scaled by this factor. E.g. number of splits increases, angular veloctiy delta decreases
     */
    int replanFactor;
    /**
     * @brief minimum number of valid poses
     */
    int minNumberNodes;


    // calculated
    /**
     * @brief duration of for one sub trajectory
     */
    float trajectoryTimeStep;
    /**
     * @brief delta time between two pose tests
     */
    float subSampleTimeStep;


};


/**
 * @brief Scoring parameters, The trajectory with the HIGHEST score is selected!!
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

        f_goalDistance = 1.0;
        f_goalOrientation = -2.0;
        f_pathDistance = -1.0;
        f_lastCmdVelDiff = -2.0;

        f_childCount = 0;

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

        targetGoalDistance = 0.2;

    }

    float gravAngleThreshold; // in rad, limit for angle between pose and gravity vector
    float deltaAngleThreshold; // in rad, limit for angle between pose_t and pose_t-1
    float tipAngleThreshold; // in rad, limit for angle between two configurations
    float minWheelSupportThreshold; // in percent, the minimum percentage of wheel pixels close enough to the ground. A wheel is considered stable if this percentage is reached.
    bool allowNotVisible; // If true the planner is allowed to plan over unseen areas. Defaults to false for safety reasons.
    float noWheelSupportNearThreshold; // in sec, since the sometimes the environment is not visible from the current pose, if the unseen area is beyond this distance the planner can still plan there
    float noWheelSupportRotateThreshold; // in rad, same as noWheelSupportNearThreshold but for rotations
    float minPoseTime; // in sec, if a trajectory is shorter than this time it is considered not traversable

    float f_meanGA; // weighting factor for mean gravity angle.
    float f_maxGA;// weighting factor for max gravity angle
    float f_meanAD;// weighting factor for mean delta angle
    float f_maxAD;// weighting factor for mx delta angle
    float f_meanTA;// weighting factor for mean tip angle
    float f_maxTA;// weighting factor for max tip angle
    float f_poseC;// weighting factor for the number of valid poses along a trajectory
    float f_aVelD;// weighting factor angular velocity changes. Encourages driving straight
    float f_meanWS;// weighting factor for mean wheel support. Higher wheel support is better
    float f_minWS;// weighting factor for min wheel support. Higher wheel support is better
    //float f_numNotVisible;// factor
    float f_goalDistance; // weighting factor for distance to goal. This should be minimized
    float f_goalOrientation; // weighting factor for angle to goal. This should be minimized
    float f_lastCmdVelDiff; // weighting factor for not switching velocities too fast.
    float f_pathDistance; // weighting factor for distance to path. This should be minimized

    float f_childCount; // weighting factor for child count. Nodes with higher child counts are usually safer.


    /**
     * @brief End state weighting factors
     */
    float end_outOfImage; // offset
    float end_noWheelSupport; // offset
    float end_noWheelSupportFar;// offset
    float end_notVisible;// offset
    float end_valid;// offset
    float end_exceedAngle;// offset
    float end_goalReached;// offset
    float end_poseCountLowPenalty;// offset
    float end_chassisCollision;// offset

    /**
     * @brief Stop planning if within this range to goal
     */
    float targetGoalDistance; // in m

    //Calculated
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
        maxLinVel = 0.5;

        maxAngVel = 3.14159265359;

    }

    /**
     * @brief step size for angular velocity change
     */
    float deltaTheta;
    /**
     * @brief number of angular velocity splits
     */
    int numSplits;
    /**
     * @brief number of angular velocity splits on the first level of the tree
     */
    int firstLevelSplits;
    /**
     * @brief number of linear velocity splits on the first level of the tree
     */
    int firstLevelLinearSplits;

    /**
     * @brief step size for angular velocity change on the first level of the tree
     */
    float firstLevelDeltaTheta;
    /**
     * @brief step size for linear velocity change on the first level of the tree
     */
    float firstLevelDeltaLinear;

    /**
     * @brief minimum and maximum linear velocity
     */
    float minLinVel,maxLinVel;
    /**
     * @brief maximum angular velocity
     */
    float maxAngVel;

};



#endif //PLANNER_SCORER_CONFIG
