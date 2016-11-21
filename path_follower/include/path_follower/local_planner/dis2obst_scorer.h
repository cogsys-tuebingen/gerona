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
    static void setFactor(double factor);

    virtual double score(const LNode& point) override;

private:
    static double factor_;
};

#endif // DIS2PATH_SCORER_H
