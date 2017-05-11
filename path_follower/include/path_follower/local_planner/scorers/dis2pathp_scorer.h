#ifndef DIS2PATHP_SCORER_H
#define DIS2PATHP_SCORER_H

#include <path_follower/local_planner/scorer.h>

class Dis2PathP_Scorer : public Scorer
{
public:
    typedef std::shared_ptr<Dis2PathP_Scorer> Ptr;

public:
    Dis2PathP_Scorer();
    virtual ~Dis2PathP_Scorer();
    static void setMaxD(double& dis);

    virtual double score(const LNode& point) override;

private:
    static double MAX_DIS;
};

#endif // DIS2PATHP_SCORER_H
