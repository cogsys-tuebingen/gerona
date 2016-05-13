#ifndef DIS2PATH_SCORER_H
#define DIS2PATH_SCORER_H

#include <path_follower/local_planner/scorer.h>
#include <path_follower/utils/path.h>

class Dis2Path_Scorer : public Scorer
{
public:
    typedef std::shared_ptr<Dis2Path_Scorer> Ptr;

public:
    Dis2Path_Scorer();
    virtual ~Dis2Path_Scorer();

    virtual double score(const tf::Point& point) override;

    void setSubPath(const SubPath& path);
private:
    SubPath currentPath;//For now stores the points of the current subpath
};

#endif // DIS2PATH_SCORER_H
