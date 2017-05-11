#ifndef DIS2PATHD_SCORER_H
#define DIS2PATHD_SCORER_H

#include <path_follower/local_planner/scorer.h>

class Dis2PathD_Scorer : public Scorer
{
public:
    typedef std::shared_ptr<Dis2PathD_Scorer> Ptr;

public:
    Dis2PathD_Scorer();
    virtual ~Dis2PathD_Scorer();
    static void setMaxD(double& dis);

    virtual double score(const LNode& point) override;

private:
    static double MAX_DIS;
};

#endif // DIS2PATHD_SCORER_H
