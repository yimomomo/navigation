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
from std_msgs.msg import String

DISPLAY_COLOR_SEG = 1
rospy.init_node('seg_trans', anonymous=True)
segcli = zmq_comm_cli_c(name=name_semanticseg, ip=ip_semanticseg, port=port_semanticseg)
res_pub = rospy.Publisher('/seg_processing', Point32, queue_size=1)
bridge = CvBridge()

ID_C = np.array([-1,-1,-1,-1,-1, -1,-1,0,1,-1, -1,2,3,4,-1, -1,-1,5,-1,6, 7,8,9,10,11, 12,13,14,15,-1, -1,16,17,18,-1], dtype=np.int16)
colors = np.array([[128,64,128], [244,35,232], [70,70,70], [102,102,156], [190,153,153], [153,153,153], [250,170,30], [220,220,0], [107,142,35], [152,251,152], [255,0,0], [220,20,60], [70,130,180], [0,0,142], [0,0,70], [0,60,100], [0,80,100], [0,0,230], [119,11,32], [0,0,0]], dtype=np.uint8)

order_msg = "q"
img_resize = None


def order_callback(order_sub):
    global order_msg
    order_msg = order_sub.data
    print(type(order_msg))



def odometry_callback(Odometry_sub):
    #transformation_matrix = np.array([[139.268,0.0,240.5],[0,139.268,180.5],[0.0,0.0,1.0]])
    global img_resize
    global order_msg
    img2 = plimg.fromarray(cv2.cvtColor(img_resize,cv2.COLOR_BGR2RGB))
    data_jpeg=img_rgb_to_jpeg(img2) 
    #print(data_jpeg.shape)  
    rospy.loginfo('1')
    start = time.time()
    res = segcli.execute(data_jpeg)
    end = time.time()
    print('%30s'%'Got a segmentation result in ',str((end-start)*1000),'ms')
    rospy.loginfo('2')
    seg_ind = res.transpose(1,0)
    #print(type(seg_ind), seg_ind.shape)
    rospy.loginfo('end')   
    scale_x = float(480)/686
    scale_y = float(360)/376
    #scale_x = 1
    #scale_y = 1
    transformation_matrix = np.array([[349.459*scale_x,0.0,342.793*scale_x],[0,349.459*scale_y,188.472*scale_y],[0.0,0.0,1.0]])
    img_open = navi.processing_segimg(seg_ind)
    img_resize_copy = np.zeros(img_resize.shape)
    img_resize_copy = img_resize.copy()
    img_resize_copy2 = np.zeros(img_resize.shape)
    img_resize_copy2 = img_resize.copy()


    grid = navi.grid_transformation(img_resize_copy,transformation_matrix)
    img_resize_copy = np.zeros(img_resize.shape)
    img_resize_copy = img_resize.copy()
    indices = navi.get_support_node(img_resize,grid,img_open)
    reference_line = navi.find_road_boundary(img_resize_copy2)
    print(order_msg)
    target_node_c = navi.find_target_node(img_resize,indices,reference_line,order_msg) 
    reference_position = [Odometry_sub.pose.pose.position.x,Odometry_sub.pose.pose.position.y,Odometry_sub.pose.pose.position.z]
    #reference_position = [Odometry_sub.pose.position.x,Odometry_sub.pose.position.y,Odometry_sub.pose.position.z]
    reference_orientation = [Odometry_sub.pose.pose.orientation.x,Odometry_sub.pose.pose.orientation.y,Odometry_sub.pose.pose.orientation.z,Odometry_sub.pose.pose.orientation.w]
    reference_position = np.array(reference_position)
    reference_orientation = np.array(reference_orientation)
    target_node_w = navi.calculate_target_w(target_node_c,reference_position,reference_orientation)
    ##get a rough target_node
    '''
    '''
    if(DISPLAY_COLOR_SEG):
        segimg = ID_C[res]
        colorimg = colors[segimg]
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




def seg_process(image_sub):
    #rospy.loginfo('start')
    global img_resize
    img = bridge.imgmsg_to_cv2(image_sub, "bgr8")
    #print("original image size: %d %d"%(img.shape[0],img.shape[1]))
    img_resize = cv2.resize(img,(480,360))
    
 
def callback(message):
    print('test')
 
if __name__ == '__main__':
    #image_sub = message_filters.Subscriber('/rgb/image_rect_color', Image)
    image_sub = rospy.Subscriber('/rgb/image_rect_color', Image,seg_process)
    Odometry_sub = rospy.Subscriber('/RosAria/pose', Odometry, odometry_callback)
    order_sub = rospy.Subscriber('/order_publish',String,order_callback)
    #Odometry_sub = message_filters.Subscriber('/RosAria/pose', Odometry)
    #ts = message_filters.TimeSynchronizer([image_sub, Odometry_sub], 10)
    #ts.registerCallback(seg_process)
    rospy.loginfo('[seg_trans]init done')
    rospy.spin() 

