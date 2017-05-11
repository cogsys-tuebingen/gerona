#ifndef CURVATURE_SCORER_H
#define CURVATURE_SCORER_H

#include <path_follower/local_planner/scorer.h>

class Curvature_Scorer : public Scorer
{
public:
    typedef std::shared_ptr<Curvature_Scorer> Ptr;

public:
    Curvature_Scorer();
    virtual ~Curvature_Scorer();
    static void setMaxC(double& radius);

    virtual double score(const LNode& point) override;
private:
    static double MAX_CURV;
};

#endif // CURVATURE_SCORER_H
