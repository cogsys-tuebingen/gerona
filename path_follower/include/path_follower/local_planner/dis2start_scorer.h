#ifndef DIS2START_SCORER_H
#define DIS2START_SCORER_H

#include <path_follower/local_planner/scorer.h>
#include <path_follower/utils/path.h>

class Dis2Start_Scorer : public Scorer
{
public:
    typedef std::shared_ptr<Dis2Start_Scorer> Ptr;

public:
    Dis2Start_Scorer();
    virtual ~Dis2Start_Scorer();

    virtual double score(const tf::Point& point) override;
    void setDistances(const SubPath& path);
private:
    std::vector<double> distances;
    SubPath currentPath;
};

#endif // DIS2START_SCORER_H
