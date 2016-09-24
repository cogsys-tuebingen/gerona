#ifndef CURVATURED_SCORER_H
#define CURVATURED_SCORER_H

#include <path_follower/local_planner/scorer.h>

class CurvatureD_Scorer : public Scorer
{
public:
    typedef std::shared_ptr<CurvatureD_Scorer> Ptr;

public:
    CurvatureD_Scorer();
    virtual ~CurvatureD_Scorer();
    static void setMaxC(double& radius);

    virtual double score(const LNode& point) override;
private:
    static double MAX_CURV;
};

#endif // CURVATURED_SCORER_H
