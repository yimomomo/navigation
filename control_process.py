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
import navi
from geometry_msgs.msg import Twist,Point32,Quaternion
import tf
from nav_msgs.msg import Odometry
import message_filters
import time

import sys 
import signal 

from std_msgs.msg import String
rospy.init_node('control_process',anonymous = True)
#vel_pub = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size = 1)
vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
t_n = np.zeros(3)
c_p = np.zeros(3)
c_o = np.zeros(4)
control_flag = 0##according to the input order to control whether use auto control
order_msg = 'q'
def callback(point):
    global order_msg
    global c_p
    global c_o
    target_node = np.array([point.x,point.y,point.z]).T
    t_n = target_node
    if(order_msg == 'i'):
        print("auto control switch on!!")
        control_process(t_n,c_p,c_o)
    else:
        print("auto control switch off!!")
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0 
        twist.angular.z = 0 
        vel_pub.publish(twist)
    #rospy.loginfo(t_n.shape)
    #return target_node_w



def listener_target():
    point = Point32()
    rospy.Subscriber('/seg_processing', Point32, callback)
    

def callback2(odometry):
    global c_p
    global c_o
    current_position = [odometry.pose.pose.position.x,odometry.pose.pose.position.y,odometry.pose.pose.position.z]
    current_orientation = [odometry.pose.pose.orientation.x,odometry.pose.pose.orientation.y,odometry.pose.pose.orientation.z,odometry.pose.pose.orientation.w]
    current_position = np.array(current_position).T
    current_orientation = np.array(current_orientation).T
    c_p = current_position
    c_o = current_orientation
    #print(t_n.shape)
    #if(Flag):
    
        #rospy.loginfo("successfully process the control output")
        #Flag = 0
    #print("update current position!!!")
    #print(current_position.shape)
    #print(current_orientation.shape)
def listener_Odometry():
    odometry = Odometry()
    rospy.Subscriber('/RosAria/pose', Odometry, callback2)

def order_callback(order_sub):
    global order_msg
    order_msg = order_sub.data
    if(order_msg == 'i'):
        control_flag = 1
    else:
        control_flag = 0
    print(type(order_msg))

def control_process(target_node_w,current_position,current_orientation):
    target_node_w = np.array(target_node_w).T
    Rotation_Matrix = navi.calculate_Rotation_Matrix(current_orientation)
    if(target_node_w.shape != 0):  
        v,w = navi.update_output_control(target_node_w,Rotation_Matrix,current_position,current_orientation)
    #print(v)
    #print(w)
    twist = Twist()
    twist.linear.x = v[0]
    twist.linear.y = v[1]
    twist.linear.z = v[2]
    twist.angular.x = 0
    twist.angular.y = 0 
    twist.angular.z = w 
    vel_pub.publish(twist)
    #rospy.sleep(1)
    print("publish new velocity!!!")
    

if __name__ == '__main__':
    #rospy.Subscriber('/camera/image_raw', Image, inputimg_cb)
    #rospy.Subscriber('/sim_p3at/odom', Odometry, return_odom)
    #rospy.loginfo('[control_process]init done')
    #image_sub = message_filters.Subscriber('/seg_processing', Point)
    #Odometry_sub = message_filters.Subscriber('/sim_p3at/odom', Odometry)
    #t_n = []
    #c_p = []
    #c_o = []
    listener_target()
    listener_Odometry()
    order_sub = rospy.Subscriber('/order_publish',String,order_callback)
    rospy.loginfo('[control_process]init done')
    #ts = message_filters.TimeSynchronizer([image_sub, Odometry_sub], 10)
    #image_sub.registerCallback(return_)
    #Odometry_sub.registerCallback(control_process)
    rospy.spin()
    
        
    
    
