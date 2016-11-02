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

    virtual double score(const LNode& point) = 0;

    long nsUsed();
protected:
    Stopwatch sw;
};

#endif // SCORER_H
