#ifndef POSEEVALRESULTS_H
#define POSEEVALRESULTS_H
#include <opencv2/core/core.hpp>




//enum PER_STATES { PERS_VALID= 0, PERS_SAFE_OUTOFIMAGE = 1, PERS_OUTOFIMAGE = -1, PERS_NOWHEELSUPPORT = -2, PERS_NOTASSIGNED = -3 };

/**
 * @brief The Pose eval result states enum
 */
enum PER_STATES
{
    PERS_VALID= 0,
    PERS_SAFE_OUTOFIMAGE    = 1,
    PERS_NOTVISIBLE         = 2,
    PERS_GOALREACHED        = 3,
    PERS_OUTOFIMAGE         = -1,
    PERS_NOWHEELSUPPORT     = -2,
    PERS_LOWWHEELSUPPORT    = -3,
    PERS_EXCEEDGRAVANGLE    = -4,
    PERS_EXCEEDTIPANGLE     = -5,
    PERS_EXCEEDDELTAANGLE   = -6,
    PERS_CHASSISCOLLISION   = -7,
    PERS_LOWWHEELSUPPORT_FAR    = -8,
    PERS_NOTASSIGNED        = -10
};

/**
 * @brief The wheel eval result states enum
 */
enum WER_STATES
{
    WERS_VALID= 0,
    WERS_NOTVISIBLE = 1,
    WERS_OUTOFIMAGE = -1,
    WERS_WHEELSUPPORTBELOWTHRESHOLD = -2,
    WERS_NOWHEELSUPPORT = -3

};



/**
 * @brief The Trajectory states enum
 */
enum TNODE_STATE {TN_VS_NOTASSIGNED= -2, TN_VS_NOTVALIDUNTILEND = -1, TN_VS_VALID = 0 };


/**
 * @brief The WheelEvalResults struct
 */
struct WheelEvalResults
{
    /**
     * @brief WheelEvalResults
     */
    WheelEvalResults()
    {
        Reset();
    }
    /**
     * @brief Reset all stats
     */
    void Reset()
    {
        zPos = 0.0f;
        contactPoint.x = 0;
        contactPoint.y = 0;
        wheelSupport = 1.0f;
        robotWheelAngle = 0.0f;
        globalWheelAngle = 0.0f;
        wheelAngleIdx = 0;
    }


    /**
     * @brief zValue in image coordinates
     */
    int zValue;

    /**
     * @brief Resulting z position of wheel
     */
    float zPos;

    /**
     * @brief Wheel contact point with ground in image coordinates
     */
    cv::Point2i contactPoint;

    /**
     * @brief wheelSupport ratio
     */
    float wheelSupport;


    /**
     * @brief global wheel angle index
     */
    int wheelAngleIdx;
    /**
     * @brief resulting global wheel angle
     */
    float globalWheelAngle;

    /**
     * @brief resulting wheel angle in robot coordinates
     */
    float robotWheelAngle;


    //cv::Point2i imagePos;
    //cv::Point2f wheelPos;




};

/**
 * @brief The PoseEvalResults struct
 */
struct PoseEvalResults
{
public:

    //PoseEvalResults(const PoseEvalResults&) = delete;
    //PoseEvalResults& operator=(const PoseEvalResults&) = delete;
    //PoseEvalResults(PoseEvalResults&&) noexcept {}
    //PoseEvalResults& operator=(PoseEvalResults&&) noexcept {}


    /**
     * @brief PoseEvalResults
     */
    PoseEvalResults()
    {
        Reset();

    }

    /**
     * @brief Reset all stats
     */
    void Reset()
    {
        wheelEvalResults_[0].Reset();
        wheelEvalResults_[1].Reset();
        wheelEvalResults_[2].Reset();
        wheelEvalResults_[3].Reset();


        pose.x = 0;
        pose.y = 0;
        pose.z = 0;
        n1.x = 0;
        n1.y = 0;
        n1.z = 1;
        n2.x = 0;
        n2.y = 0;
        n2.z = 1;

        a1 = 0;
        a2 = 0;

        gravAngle = 0;
        tipAngle = 0;
        deltaAngle = 0;

        caMinA = 30000;
        caMinB = 30000;

        caContactX1 = 0;
        caContactX2 = 0;
        caContactY1 = 0;
        caContactY2 = 0;

        validState = PERS_NOTASSIGNED;

        poseCounter = 0;
        stableWheelPairIdx = 0;

        dx1 = 0;
        dy1 = 0;
        dx2 = 0;
        dy2 = 0;
        start1 = 0;
        start2 = 0;

    }

    inline bool TestWheelZValues(const float testVal) const
    {
        return (wheelEvalResults_[0].zPos < testVal || wheelEvalResults_[1].zPos < testVal || wheelEvalResults_[2].zPos < testVal || wheelEvalResults_[3].zPos < testVal);

    }

    inline float GetMinWheelSupport() const
    {
        return std::min(std::min(wheelEvalResults_[0].wheelSupport,wheelEvalResults_[1].wheelSupport),std::min(wheelEvalResults_[2].wheelSupport,wheelEvalResults_[3].wheelSupport));

    }
    inline float GetMeanWheelSupport() const
    {
        return (wheelEvalResults_[0].wheelSupport+wheelEvalResults_[1].wheelSupport+wheelEvalResults_[2].wheelSupport+wheelEvalResults_[3].wheelSupport)/4.0f;

    }

    static std::string GetValidStateString(int state)
    {
        /*
enum PER_STATES
{
    PERS_VALID= 0,
    PERS_SAFE_OUTOFIMAGE    = 1,
    PERS_NOTVISIBLE         = 2,
    PERS_GOALREACHED        = 3,
    PERS_OUTOFIMAGE         = -1,
    PERS_NOWHEELSUPPORT     = -2,
    PERS_LOWWHEELSUPPORT    = -3,
    PERS_EXCEEDGRAVANGLE    = -4,
    PERS_EXCEEDTIPANGLE     = -5,
    PERS_EXCEEDDELTAANGLE   = -6,
    PERS_CHASSISCOLLISION   = -7,
    PERS_LOWWHEELSUPPORT_FAR    = -8,
    PERS_NOTASSIGNED        = -10
};
          */
        switch (state)
        {
            case PERS_VALID: return "PERS_VALID";
            case PERS_SAFE_OUTOFIMAGE: return "PERS_SAFE_OUTOFIMAGE";
            case PERS_NOTVISIBLE: return "PERS_NOTVISIBLE";
            case PERS_GOALREACHED: return "PERS_GOALREACHED";
            case PERS_OUTOFIMAGE: return "PERS_OUTOFIMAGE";
            case PERS_NOWHEELSUPPORT: return "PERS_NOWHEELSUPPORT";
            case PERS_LOWWHEELSUPPORT: return "PERS_LOWWHEELSUPPORT";
            case PERS_EXCEEDGRAVANGLE: return "PERS_EXCEEDGRAVANGLE";
            case PERS_EXCEEDTIPANGLE: return "PERS_EXCEEDTIPANGLE";
            case PERS_EXCEEDDELTAANGLE: return "PERS_EXCEEDDELTAANGLE";
        case PERS_CHASSISCOLLISION: return "PERS_CHASSISCOLLISION";
        case PERS_LOWWHEELSUPPORT_FAR: return "PERS_LOWWHEELSUPPORT_FAR";
        case PERS_NOTASSIGNED: return "PERS_NOTASSIGNED";
        default: return "unknown";

        }
    }

    inline void SetWheelAnglesRobot(const cv::Vec4f angles)
    {
        wheelEvalResults_[0].robotWheelAngle = angles[0];
        wheelEvalResults_[1].robotWheelAngle = angles[1];
        wheelEvalResults_[2].robotWheelAngle = angles[2];
        wheelEvalResults_[3].robotWheelAngle = angles[3];

    }

    inline void SetWheelAnglesGlobal(const float angle)
    {


        wheelEvalResults_[0].globalWheelAngle = angle;
        wheelEvalResults_[1].globalWheelAngle = angle;
        wheelEvalResults_[2].globalWheelAngle = angle;
        wheelEvalResults_[3].globalWheelAngle = angle;


    }


    /**
     * @brief wheelEvalResults
     */
    WheelEvalResults wheelEvalResults_[4];

    /**
     * @brief image pose of result
     */
    cv::Point3f pose;
    /**
     * @brief command velocity
     */
    cv::Point2f cmd;

    /**
     * @brief normal #1 of robot pose
     */
    cv::Point3f n1;
    /**
     * @brief normal #2 of robot pose
     */
    cv::Point3f n2;
    //cv::Point3f nSel;
    /**
     * @brief angle to gravity
     */
    float a1,a2;

    /**
     * @brief larger angle to gravity
     */
    float gravAngle;
    /**
     * @brief tipAngle
     */
    float tipAngle;
    /**
     * @brief angle difference to previous pose
     */
    float deltaAngle;

    /**
     * @brief smalles chassis distance to ground
     */
    float caMinA,caMinB;
    /**
     * @brief chassis contact points
     */
    int caContactX1,caContactX2,caContactY1,caContactY2;

    /**
     * @brief z values of base_link for both configurations
     */
    float z1,z2;

    /**
     * @brief roll and pitch angles for both configurations
     */
    float r1,p1,r2,p2;



    int stableWheelPairIdx;

    int poseCounter;

    /**
     * @brief validState of pose
     */
    int validState;

    /// Debug
    ///
    float dx1,dy1,dx2,dy2;
    float start1,start2;

};

#endif // POSEEVALRESULTS_H
