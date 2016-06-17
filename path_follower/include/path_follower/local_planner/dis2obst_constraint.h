#ifndef DIS2OBST_CONSTRAINT_H
#define DIS2OBST_CONSTRAINT_H

#include <path_follower/local_planner/constraint.h>
#include <path_follower/utils/obstaclecloud.hpp>
#include <tf/tf.h>

class Dis2Obst_Constraint : public Constraint
{
public:
    typedef std::shared_ptr<Dis2Obst_Constraint> Ptr;

public:
    Dis2Obst_Constraint(ObstacleCloud::ConstPtr &msg, tf::Transformer &transformer);
    virtual ~Dis2Obst_Constraint();

    virtual bool isSatisfied(const LNode& point) override;
private:
    ObstacleCloud::ConstPtr obstacles;
    tf::Transformer &transformer_;
    ros::Time now_;
};

#endif // DIS2OBST_CONSTRAINT_H
