#!/usr/bin/env python
import roslib
roslib.load_manifest('path_follower')
import rospy
import tf
from nav_msgs.msg import Path


import matplotlib.pyplot as plt

xpath = []
ypath = []

def path_callback(data):
    global xpath
    global ypath
    
    xpath = []
    ypath = []
    
    for i in range(len(data.poses)):
        xpath.append(data.poses[i].pose.position.x)
        ypath.append(data.poses[i].pose.position.y)
        
    print xpath, ypath

if __name__ == '__main__':
    rospy.init_node('path_plotter')

    listener = tf.TransformListener()
    
    rospy.Subscriber("/path", Path, path_callback)

    fig = plt.figure()
        
    x = []
    y = []
    
    plt.ion()     
    plt.show()   
        
    ax = fig.add_subplot(111)
        
    rate = rospy.Rate(60.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
        x.append(trans[0]);
        y.append(trans[1]);
        
        ax.clear()
 #       ax.set_xlim(-10,10)
 #       ax.set_ylim(-10,10)
        ax.plot(x, y)
        ax.plot(xpath, ypath)
        plt.draw()

        rate.sleep()
