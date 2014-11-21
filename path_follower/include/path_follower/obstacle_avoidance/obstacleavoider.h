#ifndef OBSTACLEAVOIDER_H
#define OBSTACLEAVOIDER_H

#include <Eigen/Core>
#include <pcl_ros/point_cloud.h>
#include <tf/tf.h>

#include <path_follower/pathfollowerparameters.h>
#include <path_follower/utils/path.h>

class ObstacleAvoider
{
public:
    typedef pcl::PointCloud<pcl::PointXY> ObstacleCloud;

    //! Additional information about the robots state, that might be required by the obstacle avodier
    struct State
    {
        //! Current path
        const Path::ConstPtr path;

        const PathFollowerParameters &parameters;
    };

    virtual void avoid(tf::Vector3* const cmd, const ObstacleCloud &obstacles, const State &state) = 0;
};

#endif // OBSTACLEAVOIDER_H
