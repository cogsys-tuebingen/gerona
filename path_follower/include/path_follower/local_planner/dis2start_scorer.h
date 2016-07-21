#ifndef DIS2START_SCORER_H
#define DIS2START_SCORER_H

#include <path_follower/local_planner/scorer.h>

class Dis2Start_Scorer : public Scorer
{
public:
    typedef std::shared_ptr<Dis2Start_Scorer> Ptr;

public:
    Dis2Start_Scorer();
    virtual ~Dis2Start_Scorer();

    virtual double score(const LNode& point) override;
    void setPath(const SubPath& Path, const std::vector<double>& dists,
                 const std::size_t index1, const std::size_t index2);
private:
    SubPath currentPath;
    std::vector<double> distances;
    std::size_t index1_;
    std::size_t index2_;
};

#endif // DIS2START_SCORER_H
