##########################################
#author:yimomomo
#creation time:01.11.2018
#version:1.0
##########################################
import numpy as np
import matplotlib.pyplot as plt
import cv2
import math
#from getsemantic import 
from sklearn import linear_model
import sys, os, time
from myzmq.zmq_comm import *
from myzmq.zmq_cfg import *

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from PIL import Image as plimg
import numpy as np
from sklearn.cluster import KMeans
from scipy.spatial.distance import cdist
#import Eigen

#############################################################################################
##transform the grid on the ground to the image plane##
#############################################################################################
def grid_transformation(img,transformation_matrix):
    #w, h = img.shape[0:2]
    X = np.arange(-100,100,0.5)##param
    Z = np.arange(0.00001,5,1)##param
    Y = [0.6]##param   
    grid_transform= []
    grid_original = []
    grid = []
    print(transformation_matrix)
    for i in X:
        for j in Y:
            for k in Z:
                coordinate_c = np.array([i,j,k]).T
                coordinate_i = np.dot(transformation_matrix,coordinate_c)/k
                coordinate_i = coordinate_i.astype(np.int32)
                #coordinate_i[0] = int(coordinate_i[0])
                #coordinate_i[1] = int(coordinate_i[1])
                #coordinate_i[2] = int(coordinate_i[2])
                if(coordinate_i[0]<480 and coordinate_i[0]>=0):
                    if(coordinate_i[1]<360 and coordinate_i[1]>=0):
                        grid_transform.append(coordinate_i)
                        grid_original.append(coordinate_c)
                        grid.append([coordinate_c,coordinate_i])
                        #print(coordinate_i)
    for i in range(len(grid)):
        #print(grid[i][1])
        cv2.circle (img,(grid[i][1][0],grid[i][1][1]),3,(255,255,255),3)
    #img = cv2.resize(img,(480,360))
    cv2.imshow("grid_after",img)
    cv2.waitKey(1) 
    #print(grid)
    print("grid transformation done!!!")   
    return grid
###############################################################################################
##processing the segmentation image to get a more stable result. including binary/closing/......
###############################################################################################
def processing_segimg(seg_ind):
    w = seg_ind.shape[0]
    h = seg_ind.shape[1]
    support_img = np.zeros((w,h),dtype = np.uint8)
    #print(w)
    #print(h)
    
    for i in range(w):
        for j in range(h):
            if(seg_ind[i][j] == 7.0):
                support_img[i][j] = 255
            else:
                support_img[i][j] = 0
    support_img = support_img.transpose(1,0)
    size = 5#param
    kernel = np.ones((size,size),dtype = np.uint8)
    ##closing operation
    #img_erosion = cv2.erode(img, kernel, iterations=3) 
    #img_dilation = cv2.dilate(img,kernel,iterations=3) 
    img_close = cv2.erode(cv2.dilate(support_img, kernel), kernel)
    img_close = cv2.erode(cv2.dilate(img_close, kernel), kernel)
    img_close = cv2.erode(cv2.dilate(img_close, kernel), kernel)
    img_close = cv2.erode(cv2.dilate(img_close, kernel), kernel)
    img_close = cv2.erode(cv2.dilate(img_close, kernel), kernel)
    img_close = cv2.erode(cv2.dilate(img_close, kernel), kernel)
    img_close = cv2.erode(cv2.dilate(img_close, kernel), kernel)
    ##opening operation
    #img_erosion = cv2.erode(img, kernel, iterations=3) 
    #img_dilation = cv2.dilate(img,kernel,iterations=3) 
    img_open = cv2.dilate(cv2.erode(img_close, kernel), kernel)
    img_open = cv2.dilate(cv2.erode(img_open, kernel), kernel)
    img_open = cv2.dilate(cv2.erode(img_open, kernel), kernel)
    img_open = cv2.dilate(cv2.erode(img_open, kernel), kernel)
    img_open = cv2.dilate(cv2.erode(img_open, kernel), kernel)
    img_open = cv2.dilate(cv2.erode(img_open, kernel), kernel)
    img_open = cv2.dilate(cv2.erode(img_open, kernel), kernel)
    #support_img = np.arraay(support_img)



    img_open = cv2.dilate(cv2.erode(img_open, kernel), kernel)
    img_open = cv2.dilate(cv2.erode(img_open, kernel), kernel)
    img_open = cv2.dilate(cv2.erode(img_open, kernel), kernel)
    img_close = cv2.erode(cv2.dilate(img_open, kernel), kernel)
    img_close = cv2.erode(cv2.dilate(img_close, kernel), kernel)
    img_close = cv2.erode(cv2.dilate(img_close, kernel), kernel)
    #cv2.imshow("binarize_seg_img",support_img)
    #cv2.waitKey(100)
    #cv2.imshow("closing_seg_img",img_close)
    #cv2.waitKey(100)
    #cv2.imshow("opening_seg_img",img_open)
    #cv2.waitKey(100)
    cv2.imshow("img_seg_process",img_close)
    cv2.waitKey(1)
    print("process segmentation image done!!!")
    return img_close
###############################################################################################
##using hough line detection to detect the road boundary and specify the center line.
##on the one hand ,the boundary could let us keep distance from the boundary for safety reason
##on the other hand, the detected or calculated center line could narrow down the choices range
##of target node, try to make the vehicle run along the line as much as it can.
##Input:input_image
##Input:seg_image
##output:boundary_left
##output:boundary_right
##output:center_line
##need to be modified##
###############################################################################################
#def find_road_boundary(seg_ind):
#    #seg_ind = np.load(path_to_seg_ind)
#    boundary_left_pixelpair = []
#    boundary_right_pixelpair = []
#    print(seg_ind.shape)
#    for y in range(seg_ind.shape[1]):
#        print(y)
#        min_ind = []
#        min_ind = np.where(seg_ind[:,y] == 4.0)
#        X_ind = min_ind[0]
#        if(X_ind.shape[0] == 0):
#            continue
#        else:
            ##here we need to modify a little bit. not to direct find the most left point, but need to get rid of single points or small segment of road.
#            boundary_left_x = min(X_ind)
#            boundary_right_x = max(X_ind)
#
#            boundary_left_pixelpair.append([boundary_left_x,y])
#            boundary_right_pixelpair.append([boundary_right_x,y])
#            cv2.circle (img_seg,(int(boundary_left_x),int(y)),3,(255,0,0),3)
#            cv2.circle (img_seg,(int(boundary_right_x),int(y)),3,(0,255,0),3)
#    cv2.imshow("road_boundary_pixel",img_seg)  
#    cv2.waitKey(5000)
    #return boundary_left,boundary_right,center_line

###############################################################################################
'''
def find_road_boundary(img):
        original_point = np.array([239,179])
    	img_gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
	img_blur = cv2.GaussianBlur(img_gray,(3,3),0)
        img_copy = np.zeros(img.shape)
        img_copy = img.copy()
	contour = cv2.Canny(img_blur,30,80) 
 
        lines = cv2.HoughLines(contour,1,np.pi/180,100,100,5)
        #cv2.imshow("contour",contour)
        #cv2.waitKey(1000)
        if(lines is None):
            point_x = 239
            point_y = 0
            point1_x = 239
            point1_y = 359
        else:
            lines1 = lines[:,0,:]
            lines_vertical = []
            lines_left = []
            lines_right = []
            for rho,theta in (lines1[:]):
            #print(theta*180/np.pi)
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
                        lines_vertical.append([x1,y1,x2,y2])
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
            cv2.imshow("line_detection",img)
            cv2.waitKey(1)
	    lines_left = np.array(lines_left)
            lines_right = np.array(lines_right)
            lines_vertical = np.array(lines_vertical)
        #print(lines_left)
        #print(lines_right)
            if((lines_left.shape[0]!=0) and (lines_right.shape[0]!=0)):      
                line_left = lines_left[0]
                line_right = lines_right[0]
                #cv2.line(img,(line_left[0],line_left[1]),(line_left[2],line_left[0]),(255,255,255),1)
                point_x = -(line_left[1]-line_right[1])/(line_left[0]-line_right[0])
                point_y = line_left[0]*point_x + line_left[1]
                if(lines_vertical.shape[0]!=0):
                    point1_x = lines_vertical[0][0]
                    point1_y = lines_vertical[0][1]
                else:
                    point1_x = 239
                    point1_y = 359
                cv2.circle(img,(int(point_x),int(point_y)),2,(0,255,255),2)
            elif(lines_vertical.shape[0]!=0):
                point_x = lines_vertical[0][0]
                point_y = lines_vertical[0][1]
                point1_x = lines_vertical[0][2]
                point1_y = lines_vertical[0][3]
            #print([point_x,point_y])
            else:
                point_x = 239
                point_y = 0
                point1_x = 239
                point1_y = 359
        #for line1 in line_support:
        #    for line2 in line_support:
        #        if not(operator.eq(line1,line2)):
                   # point = cross_point(line1,line2)
                    #points.append(point)
        #print(points)
        #cv2.imshow("line_detector",img)
	#cv2.imshow('line_detector',img)
	#cv2.waitKey(50)
	#contour_pub.publish(contour) 
        
        reference_line = [point_x,point_y,point1_x,point1_y]
        cv2.line(img_copy,(int(point_x),int(point_y)),(point1_x,point1_y),(255,255,255),1)
        cv2.imshow('reference_line',img_copy)
	cv2.waitKey(1)
        return reference_line
'''
##############################################################################################
##############################################################################################
##############################################################################################
'''
use segmentation result to define the boundary line 

'''

def img_process(img_seg):
    #w = seg_ind.shape[0]
    #h = seg_ind.shape[1]
    img_seg_copy = np.zeros(img_seg.shape,np.uint8) 
    img_seg_copy = img_seg.copy()
    for i in range(img_seg.shape[0]):
        for j in range(img_seg.shape[1]):
            '''
            if(seg_ind[i,j] == 4.0):
                img_seg_copy[j,i,:] = 255
            else:
                img_seg_copy[j,i,:]= 0
            '''
            if((img_seg[i,j,:] == np.array([128,64,128])).all()):
                img_seg_copy[i,j,:] = np.array([255,255,255])
            else:
                img_seg_copy[i,j,:]= np.array([0,0,0])

    #cv2.imshow("road_area",img_seg_copy)
    #cv2.waitKey(1000)
    size = 7
    
    kernel = np.ones((size,size),dtype = np.uint8)
    img_close = cv2.erode(cv2.dilate(img_seg_copy, kernel), kernel)
    img_close = cv2.erode(cv2.dilate(img_close, kernel), kernel)
    img_close = cv2.erode(cv2.dilate(img_close, kernel), kernel)
    img_close = cv2.erode(cv2.dilate(img_close, kernel), kernel)
    img_close = cv2.erode(cv2.dilate(img_close, kernel), kernel)
    img_close = cv2.erode(cv2.dilate(img_close, kernel), kernel)
    img_close = cv2.erode(cv2.dilate(img_close, kernel), kernel)
    img_open = cv2.dilate(cv2.erode(img_close, kernel), kernel)
    img_open = cv2.dilate(cv2.erode(img_open, kernel), kernel)
    img_open = cv2.dilate(cv2.erode(img_open, kernel), kernel)
    img_open = cv2.dilate(cv2.erode(img_open, kernel), kernel)
    img_open = cv2.dilate(cv2.erode(img_open, kernel), kernel)
    img_open = cv2.dilate(cv2.erode(img_open, kernel), kernel)
    img_open = cv2.dilate(cv2.erode(img_open, kernel), kernel)
    cv2.imshow("img_seg_process",img_open)
    cv2.waitKey(1)
    return img_open

def find_road_boundary(img_open):
    
    img_gray = cv2.cvtColor(img_open,cv2.COLOR_BGR2GRAY)
    img_blur = cv2.GaussianBlur(img_gray,(3,3),0)        
    contour = cv2.Canny(img_blur,30,80)
 
    #cv2.imshow("opening_seg_img",img_open)
    #cv2.waitKey(1000)
    #cv2.imshow("contour",contour)
    #cv2.waitKey(1)
    #img_binary = cv2.threshold(img_open,127,255,cv2.THRESH_BINARY)
    
    #contours,hierarchy = cv2.findContours(img_binary )

    lines = cv2.HoughLines(contour,1,np.pi/180,60)
    lines_center = []
    lines_left = []
    lines_right = []
    points = []
    if(lines is None):
        print("No boundary detected in the current frame!!!")
        '''
        here we need to add the situation if we could not detect the line. 
        consider the line detected from the last frame and correct it to be the boundary in the 
        current frame. 
        if the no last frame, just turn around to find if there's road boundary.
        '''
        point_y = 210
        row = img_gray[:,point_y]
        index = sorted(np.where(row == 255)[0])
        #print(index)
        row_shape = len(index)
        point_x = (index[0] +  index[row_shape-1])/2     
        point1_y = 310
        row1 = img_gray[:,point1_y]
        index1 = sorted(np.where(row1 == 255)[0])
        #print(index1)
        row1_shape = len(index1)
        point1_x = (index1[0] +  index1[row1_shape-1])/2
        cv2.line(img_open,(point_x,point_y),(point1_x,point1_y),(0,0,255),1) 
        cv2.line(img_open,(point_x,point_y),(point1_x,point1_y),(0,0,255),1) 
        #if(len(boundary_in_last_frame) == 0):#the first frame
        #    reference_line = []
        #else:
        #    boundary_left = boundary_in_last_frame[0]
        #    boundary_right = boundary_in_last_frame[1]
            #boundary_left = np.array(boundary_left)
    else:
        lines1 = lines[:,0,:]
        for rho,theta in (lines1[:]):
            angle = theta*180/np.pi 
            if((angle < 180) or (angle > 0)):
                a = np.cos(theta)
                b = np.sin(theta)
	        x0 = a*rho
	        y0 = b*rho
	        x1 = int(x0 + 1000*(-b))
	        y1 = int(y0 + 1000*(a))
	        x2 = int(x0 - 1000*(-b))
	        y2 = int(y0 - 1000*(a))
                #cv2.line(img_open,(x1,y1),(x2,y2),(0,255,255),1)
                k = (y2 - y1)/(x2 - x1+0.000001) 
                b = y1*1.0 - x1*k*1.0
                if(k<-0.1):
                    lines_left.append([x1,y1,x2,y2,k,b])
                    #cv2.line(img_open,(x1,y1),(x2,y2),(255,0,0),1)
                elif(k>0.1):
                    lines_right.append([x1,y1,x2,y2,k,b])
                    #cv2.line(img_open,(x1,y1),(x2,y2),(0,255,0),1)
                points.append([k,b])
   
        '''
        delete the line that are mis-detected
        '''
        lines_left = sorted(lines_left,key=lambda x:x[4])
        lines_right = sorted(lines_right,key=lambda x:x[4])
        line_left = []
        line_right = []
        #lines_left = np.array(lines_left)
        #for line in lines_left:
        if(len(lines_left) != 0):    
            line_left = lines_left[len(lines_left)/2]
            cv2.line(img_open,(line_left[0],line_left[1]),(line_left[2],line_left[3]),(255,255,0),1)
        else:
            line_left = []
        if(len(lines_right)!=0):
            line_right =lines_right[len(lines_right)/2]
            cv2.line(img_open,(line_right[0],line_right[1]),(line_right[2],line_right[3]),(0,255,255),1)
        else:
            line_right = []

        #line_left = np.array(line_left)
        #line_right = np.array(line_right)
        print len(line_left)
        print len(line_right)
        if((len(line_left)!=0) and (len(line_right)!=0)):  
            print("both sides detected!")    
            #line_left = lines_left[0]
            #line_right = lines_right[0]
            point_x = int(-(line_left[5]-line_right[5])/(line_left[4]-line_right[4]))
            point_y = int(line_left[4]*point_x + line_left[5])
            point1_y = int(point_y + 50)#parameter
            point1_x = int(((point1_y-line_left[5])/line_left[4]+(point1_y-line_right[5])/line_right[4])/2)
            cv2.circle(img_open,(int(point_x),int(point_y)),3,(0,0,255),3)
            cv2.line(img_open,(point_x,point_y),(point1_x,point1_y),(0,0,255),1)  
        elif(len(line_left)!=0 and len(line_right)==0): 
            print("left line detected!") 
            point_x = line_left[0] + 100 ##parameter
            point_y = int(line_left[1] - 100*line_left[4]) ##parameter
            point1_x = line_left[2] + 100 ##parameter
            point1_y = int(line_left[3] - 100*line_left[4]) ##parameter
            cv2.line(img_open,(point_x,point_y),(point1_x,point1_y),(0,0,255),1) 
        elif(len(lines_left)==0 and len(line_right)!=0):
            print("right line detected!") 
            point_x = line_right[0] - 100 ##parameter
            point_y = int(line_right[1] + 100*line_right[4]) ##parameter
            point1_x = line_right[2] - 100 ##parameter
            point1_y = int(line_right[3] + 100*line_right[4]) ##parameter
            #cv2.line(img_open,(point_x,point_y),(point1_x,point1_y),(0,0,255),1) 
        else:
            print("No line detected!") 
            point_y = 280
            row = img_gray[:,point_y]
            index = sorted(np.where(row == row.max())[0])
            print(index)
            point_x = (index[0] +  index[len(index)-1])/2     
            point1_y = 330
            row1 = img_gray[:,point1_y]
            index1 = sorted(np.where(row1 ==row1.max())[0])
            print(index1)
            row1_shape = len(index1)
            point1_x = (index1[0] +  index1[row1_shape-1])/2
            cv2.line(img_open,(point_x,point_y),(point1_x,point1_y),(0,0,255),1) 

    #cv2.line(img_open,(point_x,point_y),(point1_x,point1_y)(0,255,255),1)  
    cv2.imshow("lines",img_open)
    cv2.waitKey(1)
    reference_line = [point_x,point_y,point1_x,point1_y]
    return reference_line



###############################################################################################
###############################################################################################
##get which node is in the support area and which not##    
def get_support_node(img,grid,img_open):
    #support_grid = np.zeros((seg_ind.shape[0],seg_ind.shape[1]))
    indices = []
    #grid_indices = []
    #support_grid = np.zeros((img_open.shape[0],img_open.shape[1]))
    for i in range (len(grid)):
##here we can use rgb image, not the index, then we can do erosion and dialiation first and get a more stable segmentation result.
        if(img_open[grid[i][1][1],grid[i][1][0]] == 255):##here only choose class road as support area
            #support_grid[grid[i][0],int(grid[i][1])] = 1
            cv2.circle(img,(grid[i][1][0],grid[i][1][1]),2,(255,255,0),2)
            indices.append(grid[i])
    indices = np.array(indices)
    
    #print(indices)
    cv2.imshow("support node",img)
    cv2.waitKey(1)
    print("get support node index done!!")
    return indices

##################################################################################################            
##select the target node according to the input order I:forward;find the farest support node right in front of the current direction. U:left;find the farest support node on the left. O:right;find the farest support node on the right.
##################################################################################################
def find_target_node(img_resize,indices,reference_line,input_order):
     #indices = where(arr==255)
    #print(indices)
    ##here we need to add the exception condition if there's no support grid point
    
    num = indices.shape[0]
    target_node_c = []
    if(num == 0):
        target_node_c = np.array([0,0.6,0])
    else:
        X = indices[:,1,0]
        Y = indices[:,1,1]
        min_X = np.min(X)
        min_Y = np.min(Y)
        max_X = np.max(X)
        max_Y = np.max(Y)
       
        if(input_order == "i" or input_order == "I"):
            '''
            min_ind = []
            min_ind = np.where(Y == min_Y)
            print(min_ind)
            X_ind = 240
            index_max = 0
            for i in min_ind[0]:
                if abs(X[i]-240) < X_ind:
                    X_ind = X[i]
                    index_max = i
            X_target = X_ind
            print(X_target) 
            target_node_ind = [X_target,min_Y,1]
            target_node_c = indices[index_max][0]
            print(target_node_c)
            #cv2.circle (img,(int(target_node_ind[0]),int(target_node_ind[1])),3,(255,255,255),3)
            #cv2.imshow("target_node_forward",img)
            #cv2.waitKey(5000)
            #print(target_node_ind)
            '''
            array_line = np.array([reference_line[2]-reference_line[0], reference_line[3]-reference_line[1]])
            h_min = 300
            for i in range(num):
                 point = indices[i,1]
                 array_point = np.array([point[0] - reference_line[0],point[1] - reference_line[1]])
                 h = np.linalg.norm(np.cross(array_point, array_line)/np.linalg.norm(array_line))
                 if h<h_min:
                     h_min = h
                     index_min = i 
            target_node_ind = indices[index_min][1]
            target_node_c = indices[index_min][0]
            cv2.circle (img_resize,(int(target_node_ind[0]),int(target_node_ind[1])),2,(255,255,255),2)
            cv2.imshow("target_node_forward",img_resize)
            cv2.waitKey(1)
        else:
            target_node_c = np.array([0,0.6,0])
        '''
        elif(input_order == 'u' or input_order == 'U'):
            min_ind = []
            min_ind = np.where(X == min_X)
            Y_ind = [Y[i] for i in min_ind[0]]
            Y_target = min(Y_ind)
            #print(Y_target)
            target_node_ind = [min_X,Y_target,1]
            #cv2.circle (img,(int(target_node_ind[0]),int(target_node_ind[1])),3,(255,255,255),3)
            #cv2.imshow("target_node_left",img)
            #cv2.waitKey(5000)
        elif(input_order == 'o' or input_order == 'O'):
            max_ind = []
            max_ind = np.where(X == max_X)
            Y_ind = [Y[i] for i in max_ind[0]]
            Y_target = min(Y_ind)
            #print(Y_target)
            target_node_ind = [max_X,Y_target,1]
            #cv2.circle (img,(int(target_node_ind[0]),int(target_node_ind[1])),2,(255,255,255),2)
            #cv2.imshow("target_node_right",img)
            #cv2.waitKey(50)
        #target_node_c = target_node
        '''
    return target_node_c
'''
def find_target_node(indices,input_order):
     #indices = where(arr==255)
    #print(indices)
    ##here we need to add the exception condition if there's no support grid point
    num = indices.shape[0]
    if(num == 0):
        target_node_c = np.array([0,1.2,0])
    else:
        X = indices[:,1,0]
        Y = indices[:,1,1]
        min_X = np.min(X)
        min_Y = np.min(Y)
        max_X = np.max(X)
        max_Y = np.max(Y)
        if(input_order == 'i' or input_order == 'I'):
            min_ind = []
            min_ind = np.where(Y == min_Y)
            print(min_ind)
            X_ind = 240
            index_max = 0
            for i in min_ind[0]:
                if abs(X[i]-240) < X_ind:
                    X_ind = X[i]
                    index_max = i
            X_target = X_ind
            print(X_target) 
            target_node_ind = [X_target,min_Y,1]
            target_node_c = indices[index_max][0]
            print(target_node_c)
            #cv2.circle (img,(int(target_node_ind[0]),int(target_node_ind[1])),3,(255,255,255),3)
            #cv2.imshow("target_node_forward",img)
            #cv2.waitKey(5000)
            #print(target_node_ind)
        elif(input_order == 'u' or input_order == 'U'):
            min_ind = []
            min_ind = np.where(X == min_X)
            Y_ind = [Y[i] for i in min_ind[0]]
            Y_target = min(Y_ind)
            print(Y_target)
            target_node_ind = [min_X,Y_target,1]
            cv2.circle (img,(int(target_node_ind[0]),int(target_node_ind[1])),3,(255,255,255),3)
            cv2.imshow("target_node_left",img)
            cv2.waitKey(5000)
        elif(input_order == 'o' or input_order == 'O'):
            max_ind = []
            max_ind = np.where(X == max_X)
            Y_ind = [Y[i] for i in max_ind[0]]
            Y_target = min(Y_ind)
            print(Y_target)
            target_node_ind = [max_X,Y_target,1]
            cv2.circle (img,(int(target_node_ind[0]),int(target_node_ind[1])),2,(255,255,255),2)
            cv2.imshow("target_node_right",img)
            cv2.waitKey(100)
    print("found target node!!!")
    return target_node_c
'''
#########################################################################################
##transform the quarteridon to the rotation matrix using the orientation info got from the 
##odometry topic


#########################################################################################
def calculate_Rotation_Matrix(node_orientation):
    Rotation_Matrix = np.zeros((3,3))
    cosine = np.cos(node_orientation[3])
    sine = np.sin(node_orientation[3])
    x = node_orientation[0]
    y = node_orientation[1]
    z = node_orientation[2]
    w = node_orientation[3]
    #Rotation_Matrix[0,0] = cosine + x**2*(1-cosine)
    #Rotation_Matrix[0,1] = x*y*(1-cosine)
    #Rotation_Matrix[0,2] = y*sine
    #Rotation_Matrix[1,0] = x*y*(1-cosine)
    #Rotation_Matrix[1,1] = cosine + y**2*(1-cosine)
    #Rotation_Matrix[1,2] = -x*sine
    #Rotation_Matrix[2,0] = -y*sine
    #Rotation_Matrix[2,1] = x*sine
    #Rotation_Matrix[2,2] = cosine


    Rotation_Matrix[0,0] = 2*(x**2 + w**2) -1
    Rotation_Matrix[0,1] = 2*(x*y + z*w)
    Rotation_Matrix[0,2] = 2*(x*z - y*w)
    Rotation_Matrix[1,0] = 2*(x*y - z*w)
    Rotation_Matrix[1,1] = 2*(y**2 + w**2) -1
    Rotation_Matrix[1,2] = 2*(y*z + x*w)
    Rotation_Matrix[2,0] = 2*(x*z + y*w)
    Rotation_Matrix[2,1] = 2*(y*z - x*w)
    Rotation_Matrix[2,2] = 2*(z**2 + w**2) -1
    #print(np.dot(Rotation_Matrix,Rotation_Matrix.T))
    #print(np.linalg.inv(Rotation_Matrix))
    #print(Rotation_Matrix.T)
    return Rotation_Matrix


#########################################################################################
##here current position is given by the ordometry of ros itself. when the next segmentation
##frame is not coming, the current node will be the same one as we select before, and we keep
##get the current position and adjust the reference velocity[v,w]
##using PID
##v = pd
##w = p/theta
##Input:reference_position the position of vehicle while we get a segmentation frame, in the world according to the initial position.
##Input:reference_orientation the position of vehicle while we get a segmentation frame, in the world according to the initial position.
##Input:target_node: temporal target position while next frame comes. pixel position in the image
#########################################################################################
def calculate_target_w(target_node_c,reference_position,reference_orientation):
    ##transform the target node to the world coordinates
    #target_node_c = 
    #print(target_node_c)   
    #ori = reference_orientation/np.sqrt(np.sum(reference_orientation**2))
    Rotation_Matrix = calculate_Rotation_Matrix(reference_orientation) 
    #Tcw = np.concatenate((Rotation_Matrix *))
    R0 = np.array([[0,0,1],[-1,0,0],[0,-1,0]])
    #R0 = np.array([[-1,0,0],[0,0,-1],[0,-1,0]])
    target_node = np.dot(R0,target_node_c) + np.array([0,0,1.2])
    #print(target_node)
    target_node = np.dot(Rotation_Matrix.T,target_node) + reference_position
    #print(target_node)
    #target_node_w = np.dot(R0.T,(target_node_c-np.array([0,0,1.2])))
    #print(target_node_w)
    #target_node_w = np.dot(Rotation_Matrix.T,(target_node_w-reference_position))
    #print(target_node_w)
    print("calculate target node position in world done!!!")
   # ori  = reference_orientation /  np.sqrt(np.sum(reference_orientation**2))
    #print(ori)
    #v = ori * target_node_c
    #print(v)
    return target_node
##############################################################################################
##when the time interval of processing two frame is too lang, we check if there is a processed frame in the queue. If yes, we continue calculate the new reference output_control using next frame with output_control. If not, then we detect the posture of vehicle in the time interval several times and update the reference control variable.
##############################################################################################
def update_output_control(target,Rotation_Matrix,current_position_w,current_orientation_w):
    #################################
    ##read the odometry information##

    #current_position_w =
    #################################
    #print(target)
   # Rotation_Matrix = calculate_Rotation_Matrix(reference_orientation)
    sub = (target-current_position_w)
    inverse = (np.linalg.inv(Rotation_Matrix.T))
    target_node_ccurrent = np.dot(Rotation_Matrix,sub)
    #print(target_node_ccurrent)
    #print(target_node_ccurrent)
    #print(target_node_ccurrent)
    distance_update = np.sqrt(np.sum((target_node_ccurrent[0:2])**2))
    #print(distance_update)
    theta_update = np.arctan(float(target_node_ccurrent[1])/(target_node_ccurrent[0]+0.000001))    
    p1 = 0.03##para
    p2 = 0.04
    v_reference_update = p1*distance_update
    w_reference_update = p2*theta_update
    v_x = v_reference_update * np.cos(theta_update)
    
    v_y = v_reference_update * np.sin(theta_update)
    v_z = 0
    v = [v_x,v_y,v_z]
    return v,w_reference_update




##############################################################################################
##here is only the temporal test input form ,will soon be modified to adapt to ros and combined 
## to the vehicle
##############################################################################################        
#img = cv2.imread("/home/yimomomo/catkin_ws/src/p3at_sim/tools/image-simu/00000.png")
#print(img.shape)
#img_seg = cv2.imread("/home/yimomomo/catkin_ws/src/p3at_sim/tools/image-simu/00000_seg.png")
#img = cv2.resize(img,(480,360))
#path_to_seg_ind = "/home/yimomomo/catkin_ws/src/p3at_sim/tools/seg_ind.npy"
#seg_ind = np.load(path_to_seg_ind)
#transformation_matrix = np.array([[139.268,0.0,240.5],[0,139.268,180.5],[0.0,0.0,1.0]])##param
#img_open = processing_segimg(seg_ind)
#grid = grid_transformation(transformation_matrix)

#indices = get_support_node(grid,img_open)
#print(np.sum(grid_support==255))
#cv2.imshow("support node ",grid_support.transpose(1,0))
#cv2.waitKey(20000)
#print(grid_support)
#reference_line = find_road_boundary(img)
#print(reference_line)
#target_node_c = find_target_node(indices,reference_line,'I')

#reference_position = np.array([16.3593792871,-4.73447164966,0]).T
#reference_position = np.array([16.3593792871,-4.73447164966,1.2]).T
#reference_orientation = np.array([-0.000053,-0.000049,-0.728410303685,0.68514117105])

#target_node_w = calculate_target_w(target_node_c,reference_position,reference_orientation)
#targe
#v,w = update_output_control(target_node_w,reference_position,reference_orientation)
#print(v)
#print(w)
'''
for i in range (119):
    print("seg_result_realimg/%05d"%i+".jpg")
    img = cv2.imread("image/162621/%05d"%i+".jpg")
    print(img.shape)
    img_seg = cv2.imread("seg_result_realimg/%05d"%i+".jpg")
    img = cv2.resize(img,(480,360))
    scale_x = float(480)/672
    scale_y = float(360)/376
    #img = cv2.imread("162621/00009.jpg")
    #print(img.shape)
    #seg_ind,seg_rgb = segmentation(img)
    #print(seg_rgb.max())
    #img_seg = (seg_rgb*255).astype('uint8')
    #cv2.imwrite("seg_result_realimg/%05d"%i + ".jpg",img_seg)
    path_to_seg_ind = "seg_result_realimg/index/%05d"%i + ".npy"
    seg_ind = np.load(path_to_seg_ind)
    transformation_matrix = np.array([[349.459*scale_x,0.0,342.793*scale_x],[0,349.459*scale_y,188.472*scale_y],[0.0,0.0,1.0]])##param
    img_open = processing_segimg(seg_ind)
    grid = grid_transformation(img,transformation_matrix)

    indices = get_support_node(grid,img_open)
    reference_line = find_road_boundary(img)
    target_node_c = find_target_node(indices,reference_line,'I')
'''
