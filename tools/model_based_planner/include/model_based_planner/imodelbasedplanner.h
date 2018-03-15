#ifndef ABSTRACT_MODELBASEDPLANNER
#define ABSTRACT_MODELBASEDPLANNER


#include <memory>
#include <plannerutils.h>
#include <config_modelbasedplanner.h>

enum PLANNER_TYPES { PT_AStar_AngularVel_WSPL=0, PT_TreeDWA_AngularVel_WSPL, PT_DWA_AngularVel_WSPL };

const std::string PLANNER_TYPES_STR[3] =
{
    "AStar_AngularVel_WSPL",
    "TreeDWA_AngularVel_WSPL",
    "DWA_AngularVel_WSPL"
};

/**
 * @brief Main interface class for the model based planner.
 */
class IModelBasedPlanner
{
public:

    typedef std::shared_ptr<IModelBasedPlanner> Ptr;
    static IModelBasedPlanner::Ptr Create(ModelBasedPlannerConfig &config);

    virtual ~IModelBasedPlanner() { }


    /**
     * @brief Set the current DEM for path planning
     */
    virtual void UpdateDEM(const cv::Mat &dem) = 0;

    /**
     * @brief Get the current DEM
     */
    virtual const cv::Mat GetDem() = 0;

    /**
     * @brief Set the current robot pose in world coordinates
     */
    virtual void SetRobotPose(const cv::Point3f &curPose) = 0;

    /**
     * @brief Sets the current robot velocity in the world coordinate system
     */
    virtual void SetVelocity(const cv::Point2f &curVel) = 0;

    /**
     * @brief Set the position of the current DEM in world coordinates
     */
    virtual void SetDEMPos(const cv::Point2f &minPos) = 0;

    /**
     * @brief Get the position of the current DEM in world coordinates
     */
    virtual cv::Point2f GetDEMPos() = 0;

    /**
     * @brief Perform planning
     */
    virtual cv::Point2f Plan() = 0;

    /**
     * @brief Get all trajectory nodees in map coordinates
     */
    virtual void GetAllTrajectoryNodes(std::vector<TrajNode*> &trajectoryNodes) = 0;

    /**
     * @brief Get the result trajectory in map coordinates
     */
    virtual Trajectory* GetResultTrajectory() = 0;

    /**
     * @brief Get the result trajectory in world coordinates
     */
    virtual Trajectory* GetBLResultTrajectory() = 0;

    /**
     * @brief Get a pointer to the leaf node with the highest score
     */
    virtual TrajNode* GetBestNode() = 0;


    /**
     * @brief Set the goal position in world coordinates
     */
    virtual void SetGoalMap(const cv::Point3f goal) = 0;


    /**
     * @brief Update the current planner parameters, re-initializes the planner if the maximum number of nodes changed
     */
    virtual void SetPlannerParameters(PlannerConfig &config) = 0;

    /**
     * @brief Update scoring parameters
     */
    virtual void SetPlannerScorerParameters(PlannerScorerConfig &config) = 0;

    /**
     * @brief Update expander parameters
     */
    virtual void SetPlannerExpanderParameters(PlannerExpanderConfig &config) = 0;


    /**
     * @brief Get a debug image of the latest planning
     */
    virtual cv::Mat DrawDebugImage(float scalingFactor, bool drawRobot) = 0;


    /**
     * @brief Get the number of tested poses during last planning
     */
    virtual int GetPoseCount() = 0;


};



#endif //ABSTRACT_MODELBASEDPLANNER
