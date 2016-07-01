#ifndef DIS2PATHI_SCORER_H
#define DIS2PATHI_SCORER_H

#include <path_follower/local_planner/scorer.h>

class Dis2PathI_Scorer : public Scorer
{
public:
    typedef std::shared_ptr<Dis2PathI_Scorer> Ptr;

public:
    Dis2PathI_Scorer();
    virtual ~Dis2PathI_Scorer();

    virtual double score(const LNode& point) override;
};

#endif // DIS2PATHI_SCORER_H
