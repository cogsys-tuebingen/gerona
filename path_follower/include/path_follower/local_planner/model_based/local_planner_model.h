#ifndef LOCAL_PLANNER_MODEL_H
#define LOCAL_PLANNER_MODEL_H

/// PROJECT
#include <path_follower/local_planner/abstract_local_planner.h>
#include <path_follower/parameters/model_parameters.h>

/// SYSTEM
#include <ros/time.h>

/// LIBRARIES
#include <model_based_planner/imodelbasedplanner.h>



#define MODEL_PLANNER_DEBUG 1

class LocalPlannerModel : public AbstractLocalPlanner
{
public:
    LocalPlannerModel();


    virtual Path::Ptr updateLocalPath() override;

    virtual void setGlobalPath(Path::Ptr path) override;


    virtual void setVelocity(geometry_msgs::Twist::_linear_type vector) override;
    virtual void setVelocity(geometry_msgs::Twist velocity) override;

    virtual void setVelocity(double velocity) override;

    virtual void reset() override;

    Path::Ptr updateLocalPath_BaseLink();
    Path::Ptr updateLocalPath_LocalMap();

    Path::Ptr CreateDummyWps();

    bool TestPath(SubPath &inPath);




private:

#ifdef MODEL_PLANNER_DEBUG
    ros::Publisher dbgImgPub_;

#endif

    struct MControllerParameters : public ModelParameters
    {
        MControllerParameters():
            ModelParameters()
        {

        }
    } m_opt_;


    Waypoint target_;
    Waypoint path_end_;

    virtual void setParams(const LocalPlannerParameters& opt) override;

    bool transform2base(ros::Time& now);

    bool transformWPS(std::string source, std::string target, SubPath &waypoints, ros::Time& now);

    bool GetTransform(ros::Time time,std::string targetFrame, std::string sourceFrame, tf::StampedTransform &trans);

    void printTimeUsage();

    void printNodeUsage(std::size_t& nnodes) const;
    void printVelocity() const;
    void printLevelReached() const;
    bool algo(SubPath& local_wps);

    void PublishDebugImage();


protected:

    IModelBasedPlanner::Ptr model_based_planner_;


    ModelBasedPlannerConfig config_;

    bool initialized_;

    bool use_velocity_;
    bool close_to_goal_;
    float lowerVelocity_;

    std::vector<cv::Point3f> currentPath_;


    ros::Time last_update_;

    int numFrames_;
    double totalPlanningTime_;
};

#endif // LOCAL_PLANNER_MODEL_H
