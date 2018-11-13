#!/usr/bin/env python
# coding=utf-8


#import roslib;roslib.load_manifest('beginner_tutorials') 
#from  beginner_tutorials.msg import Num
import sys, os, time
from myzmq.zmq_comm import *
from myzmq.zmq_cfg import *

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from PIL import Image as plimg
import numpy as np
import navi
from geometry_msgs.msg import Twist,Point32,Quaternion
import tf
from nav_msgs.msg import Odometry
import message_filters
import time
import webcam_demo

DISPLAY_COLOR_SEG = 0
rospy.init_node('seg_trans', anonymous=True)
#segcli = zmq_comm_cli_c(name=name_semanticseg, ip=ip_semanticseg, port=port_semanticseg)
res_pub = rospy.Publisher('/seg_processing', Point32, queue_size=1)
bridge = CvBridge()

def seg_process(image_sub,Odometry_sub):
    rospy.loginfo('start')
    img = bridge.imgmsg_to_cv2(image_sub, "bgr8")
    img2 = plimg.fromarray(cv2.cvtColor(img,cv2.COLOR_BGR2RGB))
    data_jpeg=img_rgb_to_jpeg(img2) 
    #data_jpeg.show()
    print(data_jpeg.shape)  
    rospy.loginfo('1')
    start = time.time()
    #res = segcli.execute(data_jpeg)
    res = webcam_demo.segmentation(data_jpeg)
    end = time.time()
    print('%30s'%'Got a segmentation result in ',str((end-start)*1000),'ms')
    rospy.loginfo('2')
    #print(res['seg'])   
    #np.save("seg_ind.npy",res['seg'])
    ##After doing segmentation we get seg_ind and seg_rgb
    ##some param for processing
    seg_ind = res['seg']
    transformation_matrix = np.array([[139.268,0.0,240.5],[0,139.268,180.5],[0.0,0.0,1.0]])
    img_open = navi.processing_segimg(seg_ind)
    grid = navi.grid_transformation(transformation_matrix)
    indices = navi.get_support_node(grid,img_open)
    #print(len(indices))
    reference_line = navi.find_road_boundary(img)
    target_node_c = navi.find_target_node(indices,reference_line,'I') 
    reference_position = [Odometry_sub.pose.pose.position.x,Odometry_sub.pose.pose.position.y,Odometry_sub.pose.pose.position.z]
    reference_orientation = [Odometry_sub.pose.pose.orientation.x,Odometry_sub.pose.pose.orientation.y,Odometry_sub.pose.pose.orientation.z,Odometry_sub.pose.pose.orientation.w]
    reference_position = np.array(reference_position)
    reference_orientation = np.array(reference_orientation)
    target_node_w = navi.calculate_target_w(target_node_c,reference_position,reference_orientation)
    ##get a rough target_node
    if(DISPLAY_COLOR_SEG):
        colorimg = res['rgb']
        cv2.imshow('segcolor', colorimg)
        cv2.waitKey(1)
    #temp = bridge.cv2_to_imgmsg(res['seg'], encoding="32FC1")   
    print(target_node_w)
    target = Point32()
    target.x = target_node_w[0]
    target.y = target_node_w[1]
    target.z = target_node_w[2]
    rospy.loginfo("%f %f %f" %(target.x,target.y,target.z))
    res_pub.publish(target)
    #Canny = getCannyCoutour(img)

    rospy.loginfo('end')
 
 
if __name__ == '__main__':
    image_sub = message_filters.Subscriber('/camera/image_raw', Image)
    Odometry_sub = message_filters.Subscriber('/sim_p3at/odom', Odometry)
    rospy.loginfo('[seg_trans]init done')
    ts = message_filters.TimeSynchronizer([image_sub, Odometry_sub], 10)
    ts.registerCallback(seg_process)
    rospy.spin() 

