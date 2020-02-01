#!/usr/bin/env python
# license removed for brevity
import rospy
import actionlib

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from path_msgs.msg import PathSequence, DirectionalPath
from path_msgs.msg import FollowPathAction, FollowPathActionGoal

last_path = None

def path_callback(data):
    """Callback for path planner"""
    global last_path
    # remember global path
    last_path = data

def high_level_dummy():
    global last_path
    rospy.init_node('python_high_level_dummy', anonymous=True)

    # subscribe to global path planner
    rospy.Subscriber("plan", Path, path_callback)

    # publish path sequence for RViz
    pub = rospy.Publisher('path', PathSequence, queue_size=10)

    # client for the path following API
    client = actionlib.SimpleActionClient('follow_path', FollowPathAction)
    client.wait_for_server()

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        if last_path:
            # if we got a global plan, forward this to the follower via the action lib client
            action = FollowPathActionGoal()
            action.goal.path.header = last_path.header

            # convert nav_msgs/Path to path_msgs/DirectionalPath
            segment = DirectionalPath()
            segment.header = last_path.header
            segment.poses = last_path.poses
            segment.forward = True
            action.goal.path.paths.append(segment)

            # set follower options
            action.goal.follower_options.robot_controller.data = "kinematic_hbz"
            action.goal.follower_options.velocity = 0.75

            # visualize the path in RViz
            pub.publish(action.goal.path)
            # remember that this path is handled
            last_path = None

            # send the actionlib request
            client.send_goal(action.goal)
            # wait for the action to be finished
            client.wait_for_result(rospy.Duration.from_sec(5.0))

        rate.sleep()

if __name__ == '__main__':
    try:
        high_level_dummy()
    except rospy.ROSInterruptException:
        pass