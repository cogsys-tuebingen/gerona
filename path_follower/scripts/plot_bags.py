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
    
def signedlinedist(x1,y1, x2,y2, x3,y3):
    px = x2-x1
    py = y2-y1
       
    something = px*px + py*py

    u =  ((x3 - x1) * px + (y3 - y1) * py) / float(something)

    inside = True
    if u > 1:
        u = 1
        inside = False
    elif u < 0:
        u = 0
        inside = False

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

    return dist, x, y, inside
    
def distance_between(p1, p2):
    p1x = p1[0]
    p1y = p1[1]
    p2x = p2[0]
    p2y = p2[1]
    
    return numpy.hypot(p2x - p1x, p2y - p1y)

def normalize(angle):
    return numpy.arctan2(numpy.sin(angle),numpy.cos(angle))

def project_error(t, segment, pt):
    
    sx = segment[0]
    sy = segment[1]
    ex = segment[2]
    ey = segment[3]
            
    px = pt[0]
    py = pt[1]

    (adistance, lx, ly, inside) = signedlinedist(sx,sy,ex,ey,px,py)
      
    p = (lx,ly)
    
    stop = numpy.arctan2(py-sy, px-sx)    
    stoe = numpy.arctan2(ey-sy, ex-sx)    
    
    distance = distance_between(p, pt) * numpy.sign(normalize(stop - stoe))
    pseudotime = t + distance_between(p, (sx, sy)) / distance_between((sx, sy), (ex, ey))
    
    return distance, pseudotime, inside

    

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
    if len(sys.argv) < 2:
        print "Usage: ", sys.argv[0], " <path to bag files>"
        sys.exit()
    
    in_files = []
    for i in range(len(sys.argv)-1):    
        in_files.append(sys.argv[i+1])
 
    a_x = []
    a_y = []    
    atimes = []
    adiffs = []
    av = []
    avt = []
    av_c = []
    avt_c = []
    real_start_time = 0
    real_end_time = 0
         
    has_look_at = False
    alook_at = []         
         
    for i in range(len(in_files)):        
        in_file = in_files[i]    
            
        bag = rosbag.Bag(in_file, 'r')
 
        x = []
        y = []    
        times = []
        diffs = []
        time2pseudo = []
        max_pseudo_time = 0
        
        fig_preview = plt.figure("Preview")     
        plt.ion()     
        plt.show()  
        
        ax_preview = fig_preview.add_subplot(111)

        if i == 0:        
            for topic, msg, t in bag.read_messages(topics=['/path']):
                path_callback(msg)
                
        
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
        
        av_c.append(v_c)
        avt_c.append(vt_c)        
        
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
                
            
            #if (t.to_nsec() - start_time) / 1e9 > real_start_time:
            if (start):
                #times.append((t.to_nsec() - start_time) / 1e9)
            
                current_time = (t.to_nsec() - start_time) / 1e9
            
                closest_dist = 999999
                closest_idx = -1
                for i in range(len(xpath)):
                    dist = math.hypot(xpath[i] - px, ypath[i] - py)
                    if dist < closest_dist:
                        closest_dist = dist
                        closest_idx = i
                
                if closest_idx + 1 >= len(xpath):
                   closest_idx = len(xpath) - 2
                
                path_segment1 = (xpath[closest_idx], ypath[closest_idx], xpath[closest_idx + 1], ypath[closest_idx + 1])
                path_segment2 = (xpath[closest_idx - 1], ypath[closest_idx - 1], xpath[closest_idx], ypath[closest_idx])
                
                (d1, pseudotime, i1) = project_error(closest_idx, path_segment1, (px, py))
                (d2, pseudotime, i2) = project_error(closest_idx-1, path_segment2, (px, py))
                
                if abs(d1) < abs(d2) and i1:
                    diffs.append(d1)
                elif abs(d2) < abs(d1) and i2:
                    diffs.append(d2)
                else:
                    continue;
                    
                times.append(pseudotime)
                
                time2pseudo.append((current_time, pseudotime))
                
                max_pseudo_time = max(max_pseudo_time, pseudotime)
                
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
                    

        a_x.append(x)
        a_y.append(y)             
        
        adiffs.append(diffs)
        atimes.append(times)
                    
        if (diff_counter > 0):
            print "Average error: ", diff_sum/diff_counter    
        print "Maximum error: ", max(diffs)
        print "Maximum time: ", max(times)
        
        
        
        
        v = []
        vt = []
        start = False
        start_time = 0
        v_sum = 0
        v_counter = 0
        hasDriven = False
        hasEnded = False
        flag = False
        i_begin = 0
        i_end = 0
        counter = 0
        for topic, msg, t in bag.read_messages(topics=['/odom']):        
            if not start:
                start_time = msg.header.stamp.to_nsec()
                start = True
           
            counter += 1
            
            vx = msg.twist.twist.linear.x
            vy = msg.twist.twist.linear.y
            
            current_time = (msg.header.stamp.to_nsec() - start_time) / 1e9
            
            if((math.hypot(vx, vy) > 0) and (hasDriven == False)):
                real_start_time = current_time
                i_begin = counter
                hasDriven = True
    
            if((math.hypot(vx, vy) < 0.2) and (math.hypot(vx, vy) > 0.001) and (hasDriven == True)):
                real_end_time = current_time
                i_end = counter
    #            hasEnded = True
                

            closest = 1e9
            closest_time = 0
            closest_pseudo_time = 0
            for (time, pseudo) in time2pseudo: 
                    dist = abs(time - current_time)
                    if dist < closest:
                        closest = dist
                        closest_time= time
                        closest_pseudo_time = pseudo
          
            vv = math.hypot(vx, vy)
            
            if closest_pseudo_time > 1 and closest_pseudo_time < max_pseudo_time:
                v.append(vv)
                #vt.append(current_time)
                vt.append(closest_pseudo_time)
       
        av.append(v)
        avt.append(vt)
        
        
        
        print "Real start time: ", real_start_time
        print "Real end time: ", real_end_time
              
        for topic, msg, t in bag.read_messages(topics=['/look_at']):                 
            has_look_at = True
            look_at_x = msg.point.x
            look_at_y = msg.point.y
            alook_at.append((look_at_x, look_at_y))

            
        bag.close()    
    
        plt.close()
        plt.ioff()       

    
    n = len(a_x)
    
    # FUN ZONE BEGIN
    fig_path = plt.figure(in_file + " Path")       
    #fig_path.suptitle('Reference path vs driven path')
    ax = fig_path.add_subplot(111)                
    ax.clear()
    for i in range(n):
        ax.plot(a_x[i], a_y[i])
    if has_look_at:
        for i in range(len(alook_at)):
            ax.plot(alook_at[i][0], alook_at[i][1],'ro')
    #circle2=plt.Circle((3.8,0.3),.3,color='r',fill=False)
    #ax.add_patch(circle2)
    ax.plot(xpath, ypath, '--', dashes = (10,20), linewidth = 3.0)
    ax.set_xlabel('x_reference vs x_driven [m]')
    ax.set_ylabel('y_reference vs y_driven [m]')
    
    fig_path.tight_layout()
    fig_path.savefig('fig1.pdf')
    
    #

    a_v_filt = []    
    for i in range(n):
        v = av[i]
        
        v_filt = list(v)
        Ts = 0.2
        for i in range(len(v) - 1):
            v_filt[i+1] = Ts*v[i] + (1 - Ts)*v_filt[i]
    #
            if (i >= i_begin and i <= i_end): 
                v_sum += v_filt[i]
                v_counter += 1
                v_sum += v_filt[len(v)-1]
                v_counter += 1

        a_v_filt.append(v_filt)   
   
        if v_counter > 0:        
            print "Average speed: ", v_sum/v_counter
        print "Maximum speed: ", max(v_filt)
         
    fig_velocity = plt.figure(in_file + " Velocity")        
    #fig_velocity.suptitle('Linear velocity over time')    
    ax = fig_velocity.add_subplot(111)                
    ax.clear()
    
    for i in range(n):
        ax.plot(avt[i], a_v_filt[i])
    ax.set_xlabel('time [s]')
    ax.set_ylabel('velocity [m/s]')
    
    ax.set_xlim(0,120)
#    ax.set_ylim(0,0.9)
    fig_velocity.tight_layout()
    fig_velocity.savefig('fig2.pdf')
#    ax.set_xlim(real_start_time,max(max(vt),max(times)))
    #ax.set_xlim(real_start_time,real_end_time)
    
    
    fig_cmd_velocity = plt.figure(in_file + " Command Velocity")        
    #fig_cmd_velocity.suptitle('Command velocity over time')    
    ax = fig_cmd_velocity.add_subplot(111)                
    ax.clear()
    for i in range(n):
        ax.plot(avt_c[i], av_c[i])      
    ax.set_xlabel('time [s]')
    ax.set_ylabel('cmd_vel [m/s]')

    fig_error = plt.figure(in_file + " Path Error")        
    #fig_error.suptitle('Path following error')
    ax = fig_error.add_subplot(111)                
    ax.clear()
    
    for i in range(n):
        ax.plot(atimes[i], adiffs[i])
    ax.set_xlabel('time [s]')
    ax.set_ylabel('error [m]')
    
    ax.set_xlim(0,120)
#    ax.set_ylim(-0.5,0.2)
    fig_error.tight_layout()
    fig_error.savefig('fig3.pdf')        
#    ax.set_xlim(real_start_time,max(max(vt),max(times)))
#ax.set_xlim(real_start_time,real_end_time)     
    
     
    plt.show()    
#    
#    fig.tight_layout()
#fig.savefig('test.png')