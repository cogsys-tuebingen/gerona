#ifndef SCORER_H
#define SCORER_H

#include <memory>
#include <cmath>
#include <path_follower/utils/path.h>
#include <utils_general/MathHelper.h>
#include <utils_general/Stopwatch.h>

class Scorer
{
public:
    typedef std::shared_ptr<Scorer> Ptr;

public:
    Scorer();
    virtual ~Scorer();

    virtual double score(const LNode& point) = 0;

    long nsUsed();
protected:
    Stopwatch sw;
};

#endif // SCORER_H
