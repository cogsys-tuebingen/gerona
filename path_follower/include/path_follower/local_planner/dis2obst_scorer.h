#ifndef DIS2OBST_SCORER_H
#define DIS2OBST_SCORER_H

#include <path_follower/local_planner/scorer.h>
#include <path_follower/utils/obstaclecloud.hpp>
#include <tf/tf.h>

class Dis2Obst_Scorer : public Scorer
{
public:
    typedef std::shared_ptr<Dis2Obst_Scorer> Ptr;

public:
    Dis2Obst_Scorer(ObstacleCloud::ConstPtr &msg, tf::Transformer &transformer);
    virtual ~Dis2Obst_Scorer();

    virtual double score(const LNode& point) override;
private:
    ObstacleCloud::ConstPtr obstacles;
    tf::Transformer &transformer_;
    ros::Time now_;
};

#endif // DIS2PATH_SCORER_H
