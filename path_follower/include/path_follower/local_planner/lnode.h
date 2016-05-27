#ifndef LNODE_H
#define LNODE_H

#include <path_follower/utils/path.h>

class LNode
{
public:
    LNode(const Waypoint& data, int parent, int level);
    virtual ~LNode();

    Waypoint getData();
    int getParent();
    int getLevel();
    
    void setData(const Waypoint& data);
    void setParent(int parent);
    void setLevel(int level);
    
private:
    Waypoint data_;
    int parent_;
    int level_;
};

#endif // LNODE_H
