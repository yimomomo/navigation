####author:yimomomo
####function: low level image processing, Canny contour detection, hough line dection
####try to find if these feature could better sparate road area.
####



import cv2 
from cv_bridge import CvBridge
import sys,os,time
from PIL import Image as plimg
import numpy as np 
import rospy
import sensor_msgs
from sensor_msgs.msg import Image
#from myzmq.zmq_comm import *
#from myzmq.zmq_cfg import *
import operator

rospy.init_node('contour_extraction',anonymous = True)
contour_pub = rospy.Publisher('/extract',Image,queue_size = 1)
bridge = CvBridge()
def Canny_Contour_dection(data):
	rospy.loginfo("start read image from topic")
	img = bridge.imgmsg_to_cv2(data,"bgr8")
	#img = plimg.fromarray(cv2.cvtColor(img,cv2.COLOR_BGR2RGB))
	#data_jpeg = img_rgb_to_jpeg(img)

        original_point = np.array([240,180])
    	img_gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
	img_blur = cv2.GaussianBlur(img_gray,(3,3),0)
        
	contour = cv2.Canny(img_blur,20,60) 
        lines = cv2.HoughLines(contour,1,np.pi/180,100,100,5)
        lines1 = lines[:,0,:]
        lines_vertial = []
        lines_left = []
        lines_right = []
        for rho,theta in (lines1[:]):
            print(theta*180/np.pi)
            if((theta*180/np.pi < 80) or (theta*180/np.pi > 105)):
                #lines_dataset.append([rho,theta])
                a = np.cos(theta)
                b = np.sin(theta)
	        x0 = a*rho
	        y0 = b*rho
                
	        x1 = int(x0 + 1000*(-b))
	        y1 = int(y0 + 1000*(a))
	        x2 = int(x0 - 1000*(-b))
	        y2 = int(y0 - 1000*(a))
                if(abs((y2-y1)*1.0/((x2-x1)+0.000001))>3):
                    lines_vertial.append([int((x2+x1)/2),0])
                    cv2.line(img,(x1,y1),(x2,y2),(0,0,255),1)
                else:
                    k=(y2-y1)*1.0/(x2-x1)
                    b = y1*1.0 - x1*k*1.0
                    if(k>0):
                        lines_left.append([k,b])
                        cv2.line(img,(x1,y1),(x2,y2),(255,0,0),1) 
                    else:
                        lines_right.append([k,b])
                        cv2.line(img,(x1,y1),(x2,y2),(0,255,0),1)
            #if()
	lines_left = np.array(lines_left)
        lines_right = np.array(lines_right)
        lines_vertical = np.array(lines_vertical)
        print(lines_left)
        print(lines_right)
        if((lines_left.shape[0]!=0) and (lines_right.shape[0]!=0)):      
            line_left = lines_left[0]
            line_right = lines_right[0]
            point_x = -(line_left[1]-line_right[1])/(line_left[0]-line_right[0])
            point_y = line_left[0]*point_x + line_left[1]
            cv2.circle(img,(int(point_x),int(point_y)),2,(0,255,255),2)
        elif(lines_vertical.shape[0]!=0):
            point_x = lines_vertical[0][0]
            point_y = lines_vertical[0][1]]
            print([point_x,point_y])
        else:
            point_x = 239
            point_y = 0
        #for line1 in line_support:
        #    for line2 in line_support:
        #        if not(operator.eq(line1,line2)):
                   # point = cross_point(line1,line2)
                    #points.append(point)
        #print(points)
        #cv2.imshow("line_detector",img)
	cv2.imshow('line_detector',img)
	cv2.waitKey(1)
	#contour_pub.publish(contour)
	rospy.loginfo('end')
        return [point_x,point_y,239,359]




def cross_point(line1,line2):
    x1=line1[0]
    y1=line1[1]
    x2=line1[2]
    y2=line1[3]
    
    x3=line2[0]
    y3=line2[1]
    x4=line2[2]
    y4=line2[3]
    k1=(y2-y1)*1.0/(x2-x1)
    b1=y1*1.0-x1*k1*1.0

    if (x2-x1)==0:
        k1=None
        b1=0
    else:
        k2=(y4-y3)*1.0/(x4-x3)
        b2=y3*1.0-x3*k2*1.0


    if (x4-x3)==0:
        k2=None
        b2=0
    else:
        k2=(y4-y3)*1.0/(x4-x3)
        b2=y3*1.0-x3*k2*1.0
    if k2==None:
        x=x3
    else:
        x=(b2-b1)*1.0/(k1-k2)
    y=k1*x*1.0+b1*1.0
    return [x,y]

def IPM_transformation(data):
    rospy.loginfo("start read image from topic")
    img = bridge.imgmsg_to_cv2(data,"bgr8")

if __name__ == '__main__':
	rospy.Subscriber('/camera/image_raw',Image,Canny_Contour_dection)
	rospy.spin()
