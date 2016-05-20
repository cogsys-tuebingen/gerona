/// HEADER
#include <path_follower/local_planner/dis2obst_scorer.h>

Dis2Obst_Scorer::Dis2Obst_Scorer(const ObstacleCloud::ConstPtr &msg):
    obstacles(msg)
{
    ROS_INFO_STREAM("Frame: " << obstacles->header.frame_id);
}

Dis2Obst_Scorer::~Dis2Obst_Scorer()
{

}
/*void Dis2Obst_Scorer::setTransformer(const tf::Transformer &transformer){
    transformer_ = transformer;
}*/

double Dis2Obst_Scorer::score(const tf::Point& point){
    return 0.0;
}
