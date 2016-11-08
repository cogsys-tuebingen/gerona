#ifndef SCORER_H
#define SCORER_H

#include <memory>
#include <path_follower/utils/path.h>
#include <cslibs_utils/MathHelper.h>
#include <cslibs_utils/Stopwatch.h>

class Scorer
{
public:
    typedef std::shared_ptr<Scorer> Ptr;

public:
    Scorer();
    virtual ~Scorer();

    double getWeight() const;
    void setWeight(double weight);

    double calculateScore(const LNode& point);

    long nsUsed();

protected:
    virtual double score(const LNode& point) = 0;

protected:
    Stopwatch sw;

    double weight_;
};

#endif // SCORER_H
