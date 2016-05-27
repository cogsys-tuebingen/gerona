///HEADER
#include <path_follower/local_planner/lnode.h>

LNode::LNode(const Waypoint& data, int parent, int level):
data_(data.x, data.y, data.orientation),parent_(parent),level_(level){

}

LNode::~LNode()
{

}

Waypoint LNode::getData(){
    return data_;
}
int LNode::getParent(){
    return parent_;
}
int LNode::getLevel(){
    return level_;
}
 
void LNode::setData(const Waypoint& data){
    data_.x = data.x;
    data_.y = data.y;
    data_.orientation = data.orientation;
}
void LNode::setParent(int parent){
    parent_ = parent;
}
void LNode::setLevel(int level){
    level_ = level;
}
