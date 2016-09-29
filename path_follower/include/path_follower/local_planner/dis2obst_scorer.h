#ifndef DIS2OBST_SCORER_H
#define DIS2OBST_SCORER_H

#include <path_follower/local_planner/scorer.h>

class Dis2Obst_Scorer : public Scorer
{
public:
    typedef std::shared_ptr<Dis2Obst_Scorer> Ptr;

public:
    Dis2Obst_Scorer();
    virtual ~Dis2Obst_Scorer();
    static void setLimit(double dis2o);
    static void setVDis(double dis);
    static void setFactor(double factor);
    double computeFrontier(double angle);

    virtual double score(const LNode& point) override;

private:
    static double DIS2O_, factor_, full_d;
};

#endif // DIS2PATH_SCORER_H
