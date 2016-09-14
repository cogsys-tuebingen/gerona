#ifndef LEVEL_SCORER_H
#define LEVEL_SCORER_H

#include <path_follower/local_planner/scorer.h>

class Level_Scorer : public Scorer
{
public:
    typedef std::shared_ptr<Level_Scorer> Ptr;

public:
    Level_Scorer();
    virtual ~Level_Scorer();

    virtual double score(const LNode& point) override;
};

#endif // LEVEL_SCORER_H
