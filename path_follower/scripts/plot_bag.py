#!/usr/bin/env python
import roslib
roslib.load_manifest('path_follower')
roslib.load_manifest('tf')
import rospy
import tf
import rosbag
import math
import roslib
import sys

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


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print "Usage: ", sys.argv[0], " <path to bag file>"
        sys.exit()
    
    in_file = sys.argv[1]    
    
    rospy.init_node('path_bag_plotter')
    tfpublisher= rospy.Publisher("tf",tf.msg.tfMessage, queue_size=100)     
    
    bag = rosbag.Bag(in_file, 'r')

    x = []
    y = []    

    listener = tf.TransformListener()
     
    fig_preview = plt.figure("Preview")     
    plt.ion()     
    plt.show()  
    
    ax_preview = fig_preview.add_subplot(111)
     
    path = []
    for topic, msg, t in bag.read_messages(topics=['/path']):
        path_callback(msg)
            
    iters = 0
    renders = 1000
    for topic, msg, t in bag.read_messages(topics=['/tf']):
        tfpublisher.publish(msg)
        
        try:
            (trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
        x.append(trans[0]);
        y.append(trans[1]);

        iters += 1

        if iters > renders:                
            ax_preview.clear()
            ax_preview.plot(x, y)
            ax_preview.plot(xpath, ypath)
            plt.draw()
            iters = 0
            
            
    v = []
    vt = []
    start = False
    start_time = 0
    for topic, msg, t in bag.read_messages(topics=['/odom']):        
        if not start:
            start_time = msg.header.stamp.to_nsec()
            start = True
            
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        v.append(math.hypot(vx, vy))
        vt.append((msg.header.stamp.to_nsec() - start_time) / 1e9)
        
    bag.close()    


    plt.close()
    plt.ioff()     
    
    
    
    # FUN ZONE BEGIN
    fig_path = plt.figure(in_file + " Path")        
    ax = fig_path.add_subplot(111)                
    ax.clear()
    ax.plot(x, y)
    ax.plot(xpath, ypath)
    
    fig_velocity = plt.figure(in_file + " Velocity")        
    ax = fig_velocity.add_subplot(111)                
    ax.clear()
    ax.plot(vt, v)
     
    plt.show()    