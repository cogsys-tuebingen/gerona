#ifndef DIS2OBST_SCORER_H
#define DIS2OBST_SCORER_H

#include <path_follower/local_planner/scorer.h>
#include <path_follower/utils/obstaclecloud.hpp>

class Dis2Obst_Scorer : public Scorer
{
public:
    typedef std::shared_ptr<Dis2Obst_Scorer> Ptr;

public:
    Dis2Obst_Scorer(const ObstacleCloud::ConstPtr &msg);
    virtual ~Dis2Obst_Scorer();

    virtual double score(const tf::Point& point) override;

    //void setTransformer(const tf::Transformer &transformer);
private:
ObstacleCloud::ConstPtr obstacles;
//tf::Transformer &transformer_;
};

#endif // DIS2PATH_SCORER_H
