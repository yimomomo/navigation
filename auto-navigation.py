##########################################
#author:yimomomo
#creation time:01.11.2018
#version:1.0
##########################################
import numpy as np
import matplotlib.pyplot as plt
import cv2
#from getsemantic import 
from sklearn import linear_model
import sys, os, time
from myzmq.zmq_comm import *
from myzmq.zmq_cfg import *

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from PIL import Image as plimg
import numpy as np
from sklearn.cluster import KMeans
from scipy.spatial.distance import cdist
#import Eigen

#############################################################################################
##transform the grid on the ground to the image plane##
#############################################################################################
def grid_transformation(img,transformation_matrix):
    w, h = img.shape[0:2]
    X = np.arange(-100,100,1)##param
    Z = np.arange(0.00001,6,1)##param
    Y = [1.2]##param
    
    grid = []
    print(transformation_matrix)
    for i in X:
        for j in Y:
            for k in Z:
                
                coordinate_w = np.array([[i,j,k]]).T
                coordinate_i = np.dot(transformation_matrix,coordinate_w)/k
                #coordinate_i = int(coordinate_i)
                coordinate_i[0] = int(coordinate_i[0])
                coordinate_i[1] = int(coordinate_i[1])
                #print(coordinate_w)
                #print(coordinate_i)
                if(coordinate_i[0]<480 and coordinate_i[0]>=0):
                    if(coordinate_i[1]<360 and coordinate_i[1]>=0):
                        grid.append(coordinate_i)
                        #print(coordinate_i)
                cv2.circle (img,(coordinate_i[0],coordinate_i[1]),3,(0,0,255),3)
    #img = cv2.resize(img,(480,360))
    cv2.imshow("grid_after",img)
    cv2.waitKey(5000)    
    return grid
###############################################################################################
##processing the segmentation image to get a more stable result. including binary/closing/......
###############################################################################################
def processing_segimg(seg_ind):
    w = seg_ind.shape[0]
    h = seg_ind.shape[1]
    support_img = np.zeros((w,h))
    print(w)
    print(h)
    
    for i in range(w):
        for j in range(h):
            if(seg_ind[i][j] == 4.0):
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
    ##opening operation
    #img_erosion = cv2.erode(img, kernel, iterations=3) 
    #img_dilation = cv2.dilate(img,kernel,iterations=3) 
    img_open = cv2.dilate(cv2.erode(img_close, kernel), kernel)
    #support_img = np.arraay(support_img)
    cv2.imshow("binarize_seg_img",support_img)
    cv2.waitKey(5000)
    cv2.imshow("closing_seg_img",img_close)
    cv2.waitKey(5000)
    cv2.imshow("opening_seg_img",img_open)
    cv2.waitKey(5000)
    return img_open
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
def find_road_boundary(img):
    	img_gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
	img_blur = cv2.GaussianBlur(img_gray,(3,3),0)
        
	contour = cv2.Canny(img_blur,20,60) 
        lines = cv2.HoughLines(contour,1,np.pi/180,100,100,5)
        lines_direction = []
        lines_intercept = []
        lines_dataset2 = []
        lines1 = lines[:,0,:]
        #for line in lines1:
        #    [x1,y1,x2,y2] = line
            #print(x1)
            #print(x2)
            #print(y1)
            #print(y2)
            
        #    if((x1 in range(0,480)) and (x2 in range(0,480)) and (y1 in range(0,360)) and (y2 in range(0,360))):
        #            dx,dy = x2-x1,y2-y1
        #            angle = np.arctan2(dy,dx) * (180/np.pi)               
        #            if (abs(angle)>10 and abs(angle) < 75):
        #                print("%d   %d   %d   %d"% (x1,y1,x2,y2))
        #                direction = dy/(dx+0.000001)
        #                intercept = y1 - direction*x1
       #                 if(abs(direction)>=40):
      #                      direction = 50
      #                      intercept = 100000
        #                print(direction)
        #                print(intercept)
     #                   lines_direction.append(direction)
     #                   lines_intercept.append(intercept)
     #                   cv2.line(img,(x1,y1),(x2,y2),(0,255,0),2)
     #                   lines_dataset2.append(line)
       # plt.figure()
       # plt.scatter(lines_direction,lines_intercept)
       # plt.show()

     #   lines_dataset = np.array([[lines_direction[i],lines_intercept[i]] for i in range(len(lines_direction))])
     #   print(lines_dataset)
        #K = range(1,4)
        #for k in K:
     #   k = 2
     #   kmeans = KMeans(n_clusters=k)
     #   kmeans.fit(lines_dataset)
      #  print(kmeans.cluster_centers_)
     #   boundaries = []
     #   for i in range (kmeans.cluster_centers_.shape[0]):
     #       boundary = []
    #        for j in range(360):
    #            x = int((j - kmeans.cluster_centers_[i][1])/kmeans.cluster_centers_[i][0])
                #cv2.circle (img,(x,j),2,(0,0,255),2)
        #meandistortion = sum 
        lines_dataset = []
        for rho,theta in (lines1[:]):
            print(theta*180/np.pi)
            if((theta*180/np.pi < 80) or (theta*180/np.pi > 105)):
                lines_dataset.append([rho,theta])
                a = np.cos(theta)
                b = np.sin(theta)
	        x0 = a*rho
	        y0 = b*rho
	        x1 = int(x0 + 1000*(-b))
	        y1 = int(y0 + 1000*(a))
	        x2 = int(x0 - 1000*(-b))
	        y2 = int(y0 - 1000*(a))
            #if()
	        cv2.line(img,(x1,y1),(x2,y2),(0,0,255),1)

        #cv2.imshow("line_detector",img)
	#cv2.imshow('Canny',contour)
	#cv2.waitKey(50000)
        lines_dataset = np.array(lines_dataset)
        k = 3
        kmeans = KMeans(n_clusters=k)
        kmeans.fit(lines_dataset)
        print(kmeans.cluster_centers_)
        boundaries = []
        for i in range (kmeans.cluster_centers_.shape[0]):
            boundary = []
            for j in range(360):
                x = int((j - kmeans.cluster_centers_[i][1])/kmeans.cluster_centers_[i][0])
                cv2.circle (img,(x,j),2,(0,255,0),2)
        cv2.imshow("line_detector",img)
	#cv2.imshow('Canny',contour)
	cv2.waitKey(50000)


	#contour_pub.publish(contour)
    #return boundary_left,boundary_right,center_line

###############################################################################################
###############################################################################################
##get which node is in the support area and which not##    
def get_support_node(grid,img_open):
    #support_grid = np.zeros((seg_ind.shape[0],seg_ind.shape[1]))
    indices = []
    for i in range (len(grid)):
##here we can use rgb image, not the index, then we can do erosion and dialiation first and get a more stable segmentation result.
        if(img_open[int(grid[i][1]),int(grid[i][0])] == 255):##here only choose class road as support area
            #support_grid[int(grid[i][0]),int(grid[i][1])] = 255
            cv2.circle(img,( grid[i][0],grid[i][1]),2,(255,255,0),2)
            indices.append(grid[i])
    indices = np.array(indices)
    print(indices.shape)
    cv2.imshow("support node",img)
    cv2.waitKey(5000)
    return indices

##################################################################################################            
##select the target node according to the input order I:forward;find the farest support node right in front of the current direction. U:left;find the farest support node on the left. O:right;find the farest support node on the right.
##################################################################################################
def find_target_node(indices,input_order):
     #indices = where(arr==255)
    #print(indices)
    ##here we need to add the exception condition if there's no support grid point
    X = indices[:,0]
    Y = indices[:,1]
    min_X = np.min(X)
    min_Y = np.min(Y)
    max_X = np.max(X)
    max_Y = np.max(Y)
    print(min_X)
    print(max_X)
    if(input_order == 'i' or input_order == 'I'):
        min_ind = []
        min_ind = np.where(Y == min_Y)
        X_ind = [X[i] for i in min_ind[0]] 
        X_target = max(X_ind)
        print(X_target)       
        #print(minimum_indices)
        target_node_ind = [X_target,min_Y,1]
        cv2.circle (img,(int(target_node_ind[0]),int(target_node_ind[1])),3,(255,255,255),3)
        cv2.imshow("target_node_forward",img)
        cv2.waitKey(5000)
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
        cv2.circle (img,(int(target_node_ind[0]),int(target_node_ind[1])),3,(255,255,255),3)
        cv2.imshow("target_node_right",img)
        cv2.waitKey(5000)
    return target_node_ind
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
    Rotation_Matrix[0,0] = cosine + x**2*(1-cosine)
    Rotation_Matrix[0,1] = x*y*(1-cosine)
    Rotation_Matrix[0,2] = y*sine
    Rotation_Matrix[1,0] = x*y*(1-cosine)
    Rotation_Matrix[1,1] = cosine + y**2*(1-cosine)
    Rotation_Matrix[1,2] = -x*sine
    Rotation_Matrix[2,0] = -y*sine
    Rotation_Matrix[2,1] = x*sine
    Rotation_Matrix[2,2] = cosine
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
def calculate_target_w(target_node,transformation_matrix,refernce_position,reference_orientation):
    ##transform the target node to the world coordinates
    target_node_c = np.dot(np.linalg.inv(transformation_matrix),target_node)    
    Rotation_Matrix = calculate_Rotation_Matrix(reference_orientation)
    target_node_w = np.dot(Rotation_Matrix.T,target_node_c) + reference_position
    #distance = np.sqrt(np.sum((target_node_c)**2))
    #theta = np.arctan(float(target_node_c[1])/(target_node_c[0]+0.000001))    
    #p = 0.1##para
    #v_reference = p*distance
    #w_reference = p*theta

##here publish the new velocity to the node
    print(target_node_w)
    return target_node_w
##############################################################################################
##when the time interval of processing two frame is too lang, we check if there is a processed frame in the queue. If yes, we continue calculate the new reference output_control using next frame with output_control. If not, then we detect the posture of vehicle in the time interval several times and update the reference control variable.
##############################################################################################
def update_output_control(target_node_w,current_position_w,current_orientation_w):
    #################################
    ##read the odometry information##

    #current_position_w =
    #################################
    Rotation_Matrix = calculate_Rotation_Matix(current_orientation_w)
    target_node_ccurrent = np.dot((np.linalg.inv(Rotation_Matrix)).T,(target_node_w-current_orientation_w))
    distance_update = np.sqrt(np.sum((target_node_ccurrent)**2))
    theta_update = np.arctan(float(target_node_ccurrent[1])/(target_node_ccurrent[0]+0.000001))    
    p = 0.1##para
    v_reference_update = p*distance_update
    w_reference_update = p*theta_update
    return v_reference_update,w_reference_update




##############################################################################################
##here is only the temporal test input form ,will soon be modified to adapt to ros and combined 
## to the vehicle
##############################################################################################        
img = cv2.imread("/home/yimomomo/catkin_ws/src/p3at_sim/tools/image-simu/00000.png")
img_seg = cv2.imread("/home/yimomomo/catkin_ws/src/p3at_sim/tools/image-simu/00000_seg.png")
#img = cv2.resize(img,(480,360))
path_to_seg_ind = "/home/yimomomo/catkin_ws/src/p3at_sim/tools/seg_ind.npy"
seg_ind = np.load(path_to_seg_ind)
#transformation_matrix = np.array([[139.268,0.0,240.5],[0,139.268,180.5],[0.0,0.0,1.0]])##param
#img_open = processing_segimg(seg_ind)
#grid = grid_transformation(img,transformation_matrix)

#indices = get_support_node(grid,img_open)
#print(np.sum(grid_support==255))
#cv2.imshow("support node ",grid_support.transpose(1,0))
#cv2.waitKey(20000)
#print(grid_support)

#target_node_ind = find_target_node(indices,'I')
#find_road_boundary(img)
reference_position = np.array([16.3933979379,-5.05906581049,0.0]).T
reference_orientation = np.array([-0.000048,-0.000053,-0.676016753933,0.73688624847])

target_node_w = calculate_target_w(target_node_ind,transformation_matrix,reference_position,reference_orientation)
#print(v)
#print(w)




