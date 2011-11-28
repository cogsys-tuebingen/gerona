#include <actionlib/server/simple_action_server.h>
#include "CarLikeDriveNode.h"

CarLikeDriveNode::CarLikeDriveNode()
{
  actionlib::SimpleActionServer as;
  as.acceptNewGoal();
  as.


}
