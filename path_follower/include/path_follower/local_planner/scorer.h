#ifndef SCORER_H
#define SCORER_H

#include <memory>
#include <tf/transform_datatypes.h>

class Scorer
{
public:
    typedef std::shared_ptr<Scorer> Ptr;

public:
    Scorer();
    virtual ~Scorer();

    virtual double score(const tf::Point& point) = 0;
};

#endif // SCORER_H
