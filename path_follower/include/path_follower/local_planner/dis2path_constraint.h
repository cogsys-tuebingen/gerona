#ifndef DIS2PATH_CONSTRAINT_H
#define DIS2PATH_CONSTRAINT_H

#include <path_follower/local_planner/constraint.h>
#include <path_follower/utils/path.h>

class Dis2Path_Constraint : public Constraint
{
public:
    typedef std::shared_ptr<Dis2Path_Constraint> Ptr;

public:
    Dis2Path_Constraint();
    virtual ~Dis2Path_Constraint();

    virtual bool isSatisfied(const tf::Point& point) override;

    void setSubPath(const SubPath& path,
                    const std::size_t index1, const std::size_t index2);
private:
    SubPath currentPath;//For now stores the points of the current subpath
    std::size_t index1_;
    std::size_t index2_;
};

#endif // DIS2PATH_CONSTRAINT_H
