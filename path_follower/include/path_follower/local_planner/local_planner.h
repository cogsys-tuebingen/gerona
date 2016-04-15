#ifndef LOCAL_PLANNER_H
#define LOCAL_PLANNER_H

/// PROJECT
#include <path_follower/utils/path.h>

class PathFollower;

class LocalPlanner
{
public:
    virtual ~LocalPlanner();

    virtual void setGlobalPath(Path::Ptr path);

    virtual Path::Ptr updateLocalPath() = 0;

    virtual bool isNull() const;

protected:
    LocalPlanner(PathFollower& controller,
                 tf::Transformer &transformer);

protected:
    PathFollower& follower_;
    tf::Transformer &transformer_;

    Path::Ptr global_path_;

    tf::StampedTransform initial_map_to_odom_;
};

#endif // LOCAL_PLANNER_H
