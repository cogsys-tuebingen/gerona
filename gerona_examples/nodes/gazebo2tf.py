#!/usr/bin/env python
"""
Created on Thu Aug 30 13:52:35 2018

@author: jordan

Simple node that converts the model state from gazebo for specified model to a tf transform.
"""

import rospy
import tf
from gazebo_msgs.msg import ModelStates
import geometry_msgs.msg


def ModelStateCallback(data):

    idx = -1
    c = 0
    for name in data.name:
        if (name == modelName):
            idx = c
            
        c += 1
        
    if (idx < 0): return
    
    
    pose = data.pose[idx]
    twist = data.twist[idx]

    
    br = tf.TransformBroadcaster()

    t = geometry_msgs.msg.TransformStamped()    
    t.transform.translation = pose.position
    t.transform.rotation = pose.orientation
    t.child_frame_id = tfRobotFrame
    t.header.frame_id = tfWorldFrame
    t.header.stamp = rospy.Time.now()
    
    br.sendTransformMessage(t)
    

rospy.init_node('gazebo2tf', anonymous=True)
modelName = rospy.get_param("~modelName","summit_xl")
tfRobotFrame = rospy.get_param("~tfRobotFrame","base_footprint")
tfWorldFrame = rospy.get_param("~tfWorldFrame","odom")
 

 

def listener():
    rospy.Subscriber("/gazebo/model_states", ModelStates, ModelStateCallback)
    
    rospy.spin()

if __name__ == '__main__':
    listener()
