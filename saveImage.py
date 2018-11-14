#!/usr/bin/env python
# coding=utf-8

import sys, os, time
from myzmq.zmq_comm import *
from myzmq.zmq_cfg import *

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from PIL import Image as plimg
import numpy as np
import time

#DISPLAY_COLOR_SEG = 1
rospy.init_node('save', anonymous=True)
#segcli = zmq_comm_cli_c(name=name_semanticseg, ip=ip_semanticseg, port=port_semanticseg)
#res_pub = rospy.Publisher('/seg_img', Image, queue_size=1)
bridge = CvBridge()
i = 0
def inputimg_cb(data): 
    global i 
    
    #rospy.loginfo('start')
    img = bridge.imgmsg_to_cv2(data, "bgr8")
    cv2.imshow("src",img)
    #img = plimg.fromarray(cv2.cvtColor(img,cv2.COLOR_BGR2RGB))
    #data_jpeg=img_rgb_to_jpeg(img)
    #end = time.time()
    num = i%50
    if( num == 0):
        cv2.imwrite('/home/zx/catkin_ws/src/navigation/image/%05d'%(i/50) +'.jpg',img)
    
    i = i+1


if __name__ == '__main__':
    rospy.Subscriber('/rgb/image_rect_color', Image, inputimg_cb)
    #start = time.time()
    #getCannyCoutour()
    #rospy.loginfo('[seg_trans]init done')
    rospy.spin()
