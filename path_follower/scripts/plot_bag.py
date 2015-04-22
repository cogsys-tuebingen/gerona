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
import numpy

import matplotlib.pyplot as plt


def linedist(x1,y1, x2,y2, x3,y3): # x3,y3 is the point
    px = x2-x1
    py = y2-y1

    something = px*px + py*py

    u =  ((x3 - x1) * px + (y3 - y1) * py) / float(something)

    if u > 1:
        u = 1
    elif u < 0:
        u = 0

    x = x1 + u * px
    y = y1 + u * py

    dx = x - x3
    dy = y - y3

    # Note: If the actual distance does not matter,
    # if you only want to compare what this function
    # returns to other results of this function, you
    # can just return the squared distance instead
    # (i.e. remove the sqrt) to gain a little performance

    dist = math.sqrt(dx*dx + dy*dy)

    return dist
real_start_time = 12.6
real_end_time = 40.3
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
    times = []
    diffs = []
 
    listener = tf.TransformListener()
     
    fig_preview = plt.figure("Preview")     
    plt.ion()     
    plt.show()  
    
    ax_preview = fig_preview.add_subplot(111)
     
    for topic, msg, t in bag.read_messages(topics=['/path']):
        path_callback(msg)
            
    iters = 0
    renders = 1000
    start = False
    start_time = 0

    has_map_2_odom = False
    has_odom_2_base = False    
    
    t_map2odom = (0.,0.,0.)    
    r_map2odom = (0.,0.,0.,0.)    
    t_odom2base = (0.,0.,0.)    
    r_odom2base = (0.,0.,0.,0.)    
    
#    odom_2_base = tf.msg.TransformStamped()
    diff_sum = 0    
    diff_counter = 0
    for topic, msg, t in bag.read_messages(topics=['/tf']):
        trafo = msg.transforms[0]
        if trafo.child_frame_id == 'base_link':
            has_odom_2_base = True
            tr = trafo.transform.translation
            rot = trafo.transform.rotation
            t_odom2base = (tr.x, tr.y, tr.z)
            r_odom2base = (rot.x, rot.y, rot.z, rot.w)
        if trafo.child_frame_id == 'odom':
            has_map_2_odom = True
            tr = trafo.transform.translation
            rot = trafo.transform.rotation
            t_map2odom = (tr.x, tr.y, tr.z)
            r_map2odom = (rot.x, rot.y, rot.z, rot.w)
            
        if not has_map_2_odom or not has_odom_2_base:
            continue
        
        trans1_mat = tf.transformations.translation_matrix(t_map2odom)
        rot1_mat   = tf.transformations.quaternion_matrix(r_map2odom)
        mat1 = numpy.dot(trans1_mat, rot1_mat)

        trans2_mat = tf.transformations.translation_matrix(t_odom2base)
        rot2_mat    = tf.transformations.quaternion_matrix(r_odom2base)
        mat2 = numpy.dot(trans2_mat, rot2_mat)
        
        mat = numpy.dot(mat1, mat2)

        px = mat[0,3]
        py = mat[1,3]        
        
        x.append(px)
        y.append(py)

        if not start:
            start = True
            start_time = t.to_nsec()
            
        
        if (t.to_nsec() - start_time) / 1e9 > real_start_time:
            times.append((t.to_nsec() - start_time) / 1e9)
            closest_dist = 999999
            closest_idx = -1
            for i in range(len(xpath)):
                dist = math.hypot(xpath[i] - px, ypath[i] - py)
                if dist < closest_dist:
                    closest_dist = dist
                    closest_idx = i
            
            if closest_idx + 1 >= len(xpath):
               closest_idx = len(xpath) - 2
           
            d1 = linedist(xpath[closest_idx], ypath[closest_idx], xpath[closest_idx + 1], ypath[closest_idx + 1], px, py)
            d2 = linedist(xpath[closest_idx], ypath[closest_idx], xpath[closest_idx - 1], ypath[closest_idx - 1], px, py)
            diffs.append(min(d1, d2))
            if (t.to_nsec() - start_time) / 1e9 < real_end_time:
               diff_sum += min(d1, d2)
               diff_counter += 1
            iters += 1
    
            if iters > renders:                
                ax_preview.clear()
                ax_preview.plot(x, y)
                ax_preview.plot(xpath, ypath)
                plt.draw()
                iters = 0            
    print "Average error: ", diff_sum/diff_counter    
    print "Maximum error: ", max(diffs)
    print "Maximum time: ", max(times)
            
    v = []
    vt = []
    start = False
    start_time = 0
    v_sum = 0
    v_counter = 0
    for topic, msg, t in bag.read_messages(topics=['/odom']):        
        if not start:
            start_time = msg.header.stamp.to_nsec()
            start = True
        if ((msg.header.stamp.to_nsec() - start_time) / 1e9 > real_start_time):   
            vx = msg.twist.twist.linear.x
            vy = msg.twist.twist.linear.y
            v.append(math.hypot(vx, vy))
            if ((msg.header.stamp.to_nsec() - start_time) / 1e9 < real_end_time): 
                v_sum += math.hypot(vx, vy)
                v_counter += 1
            vt.append((msg.header.stamp.to_nsec() - start_time) / 1e9)
            
    print "Average speed: ", v_sum/v_counter
    print "Maximum speed: ", max(v)
        
    has_look_at = False
    look_at_x = 0
    look_at_y = 0   
    for topic, msg, t in bag.read_messages(topics=['/look_at']):                 
        has_look_at = True
        look_at_x = msg.point.x
        look_at_y = msg.point.y
#####
    v_c = []
    vt_c = []
    start = False
    start_time = 0
    time = 0
    for topic, msg, t in bag.read_messages(topics=['/cmd_vel', '/odom']): 
        if topic == '/odom':           
            time = msg.header.stamp.to_nsec()
            if not start:
                start_time = time
                start = True

        if topic == '/cmd_vel' and start:               
            vx_c = msg.linear.x
            vy_c = msg.linear.y
            v_c.append(math.hypot(vx_c, vy_c))
            vt_c.append((time - start_time) / 1e9)
        
    bag.close()    
#####

    plt.close()
    plt.ioff()       

    
    
    # FUN ZONE BEGIN
    fig_path = plt.figure(in_file + " Path")       
    fig_path.suptitle('Reference path vs driven path')
    ax = fig_path.add_subplot(111)                
    ax.clear()
    ax.plot(x, y)
    if has_look_at:
        ax.plot(look_at_x, look_at_y,'ro')
    #circle2=plt.Circle((3.8,0.3),.3,color='r',fill=False)
    #ax.add_patch(circle2)
    ax.plot(xpath, ypath)
    ax.set_xlabel('x_reference vs x_driven [m]')
    ax.set_ylabel('y_reference vs y_driven [m]')
    
    #
    v_filt = list(v)
    Ts = 0.2
    for i in range(len(v) - 1):
        v_filt[i+1] = Ts*v[i] + (1 - Ts)*v_filt[i]
    #
    fig_velocity = plt.figure(in_file + " Velocity")        
    fig_velocity.suptitle('Linear velocity over time')    
    ax = fig_velocity.add_subplot(111)                
    ax.clear()
    ax.plot(vt, v_filt)
    ax.set_xlabel('time [s]')
    ax.set_ylabel('velocity [m/s]')
    #ax.set_xlim(real_start_time,max(max(vt),max(times)))
    ax.set_xlim(real_start_time,real_end_time)
    ax.set_ylim(0,0.7)
    
    fig_cmd_velocity = plt.figure(in_file + " Command Velocity")        
    fig_cmd_velocity.suptitle('Command velocity over time')    
    ax = fig_cmd_velocity.add_subplot(111)                
    ax.clear()
    ax.plot(vt_c, v_c)      
    ax.set_xlabel('time [s]')
    ax.set_ylabel('cmd_vel [m/s]')

    fig_error = plt.figure(in_file + " Path Error")        
    fig_error.suptitle('Path following error')
    ax = fig_error.add_subplot(111)                
    ax.clear()
    ax.plot(times, diffs)
    ax.plot(times, diffs)
    ax.set_xlabel('time [s]')
    ax.set_ylabel('error [m]')        
    #ax.set_xlim(real_start_time,max(max(vt),max(times)))
    ax.set_xlim(real_start_time,real_end_time)     
    ax.set_ylim(0,0.3)
     
    plt.show()    