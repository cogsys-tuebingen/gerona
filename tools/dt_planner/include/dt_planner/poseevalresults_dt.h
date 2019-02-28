#ifndef POSEEVALRESULTSDT_H
#define POSEEVALRESULTSDT_H
#include <opencv2/core/core.hpp>


/**
 * @brief The Pose eval result states enum
 */
enum PER_STATES_DT
{
    PERSDT_VALID= 0,
    PERSDT_GOALREACHED        = 1,
    PERSDT_OUTOFIMAGE         = -1,
    PERSDT_COLLISION          = -2,
    PERSDT_NOTASSIGNED        = -5
};


/**
 * @brief The Trajectory states enum
 */
enum TNODE_STATE_DT {TNDT_VS_NOTASSIGNED= -2, TNDT_VS_NOTVALIDUNTILEND = -1, TNDT_VS_VALID = 0 };


#define NumPoseTestPoints 8

/**
 * @brief The PoseEvalResults struct
 */
struct PoseEvalResultsDT
{
public:

    //PoseEvalResults(const PoseEvalResults&) = delete;
    //PoseEvalResults& operator=(const PoseEvalResults&) = delete;
    //PoseEvalResults(PoseEvalResults&&) noexcept {}
    //PoseEvalResults& operator=(PoseEvalResults&&) noexcept {}


    /**
     * @brief PoseEvalResults
     */
    PoseEvalResultsDT()
    {
        Reset();

    }

    /**
     * @brief Reset all stats
     */
    void Reset()
    {
        pose.x = 0;
        pose.y = 0;
        pose.z = 0;

        cmd.x = 0;
        cmd.y = 0;

        meanDist = 0;
        minDist = 0;


        validState = PERSDT_NOTASSIGNED;

        for (int i = 0; i < NumPoseTestPoints;++i) distances[i] = 0;


    }

    static std::string GetValidStateString(int state)
    {

        switch (state)
        {
            case PERSDT_VALID: return "PERS_VALID";
            case PERSDT_COLLISION: return "PERS_COLLISION";
            case PERSDT_GOALREACHED: return "PERS_GOALREACHED";
            case PERSDT_OUTOFIMAGE: return "PERS_OUTOFIMAGE";
        case PERSDT_NOTASSIGNED: return "PERS_NOTASSIGNED";
        default: return "unknown";

        }
    }

    cv::Point3f pose;
    cv::Point2f cmd;

    float distances[NumPoseTestPoints];
    //int numUsedTestPoints;
    float minDist;
    float meanDist;

    /**
     * @brief validState of pose
     */
    int validState;

};

#endif // POSEEVALRESULTS_H
