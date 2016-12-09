/// HEADER
#include <path_follower/local_planner/level_scorer.h>

int Level_Scorer::max_level = 10;

Level_Scorer::Level_Scorer():
    Scorer()
{

}

Level_Scorer::~Level_Scorer()
{

}

void Level_Scorer::setLevel(const int& m_level){
    max_level = m_level;
}

double Level_Scorer::score(const LNode& point){
    sw.resume();
    double ls = (double)(max_level  - point.level_);
    sw.stop();
    return ls;
}
