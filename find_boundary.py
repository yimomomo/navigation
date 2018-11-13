import numpy as np
import matplotlib.pyplot as plt
import cv2
import math
from sklearn.cluster import KMeans
from sklearn import metrics

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

    cv2.imshow("road_area",img_seg_copy)
    cv2.waitKey(1000)
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
    cv2.waitKey(1000)
    return img_open

def find_boundary(img_open):
    
    img_gray = cv2.cvtColor(img_open,cv2.COLOR_BGR2GRAY)
    img_blur = cv2.GaussianBlur(img_gray,(3,3),0)        
    contour = cv2.Canny(img_blur,30,80)
 
    cv2.imshow("opening_seg_img",img_open)
    cv2.waitKey(1000)
    cv2.imshow("contour",contour)
    cv2.waitKey(1)
    img_binary = cv2.threshold(img_open,127,255,cv2.THRESH_BINARY)
    
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
        if(len(boundary_in_last_frame) == 0):#the first frame
            reference_line = []
        else:
            boundary_left = boundary_in_last_frame[0]
            boundary_right = boundary_in_last_frame[1]
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
                if(k<0):
                    lines_left.append([x1,y1,x2,y2,k,b])
                    #cv2.line(img_open,(x1,y1),(x2,y2),(255,0,0),1)
                else:
                    lines_right.append([x1,y1,x2,y2,k,b])
                    #cv2.line(img_open,(x1,y1),(x2,y2),(0,255,0),1)
                points.append([k,b])
   
        '''
        delete the line that are mis-detected
        '''
        lines_left = sorted(lines_left)
        lines_right = sorted(lines_right)
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
            line_left = lines_left[0]
            line_right = lines_right[0]
            point_x = int(-(line_left[5]-line_right[5])/(line_left[4]-line_right[4]))
            point_y = int(line_left[4]*point_x + line_left[4])
            point1_x = 239
            point1_y = 359
            cv2.circle(img_open,(int(point_x),int(point_y)),3,(0,0,255),3)
            cv2.line(img_open,(point_x,point_y),(point1_x,point1_y),(0,0,255),1)  
        elif(len(line_left)!=0 and len(line_right)==0): 
            point_x = line_left[0] + 100 ##parameter
            point_y = int(line_left[1] - 100*line_left[4]) ##parameter
            point1_x = line_left[2] + 100 ##parameter
            point2_y = int(line_left[3] - 100*line_left[4]) ##parameter
            cv2.line(img_open,(point_x,point_y),(point1_x,point1_y),(0,0,255),1) 
        elif(len(lines_left)==0 and len(line_right)!=0):
            point_x = line_right[0] - 100 ##parameter
            point_y = int(line_right[1] + 100*line_right[4]) ##parameter
            point1_x = line_right[2] - 100 ##parameter
            point2_y = int(line_right[3] + 100*line_right[4]) ##parameter
            cv2.line(img_open,(point_x,point_y),(point1_x,point1_y),(0,0,255),1) 
        else:
            point_x = 239
            point_y = 0
            point1_x = 239
            point1_y = 359
            cv2.line(img_open,(point_x,point_y),(point1_x,point1_y),(0,0,255),1) 

    #cv2.line(img_open,(point_x,point_y),(point1_x,point1_y)(0,255,255),1)  
    cv2.imshow("lines",img_open)
    cv2.waitKey(5000)


def main():
    boundary_in_last_frame = []
    for i in range(1):
        seg_ind = np.load("index/%05d.npy"%i)
        #img = cv2.imread("162621/%05d.jpg"%i)
        #img = cv2.resize(img,(480,360))
        #img_seg = cv2.imread("%05d.jpg"%i)
        img_seg = cv2.imread("test.png")
        img_seg = cv2.resize(img_seg,(480,360))
        cv2.imshow("seg_img",img_seg)
        cv2.waitKey(5000)
        img_open = img_process(img_seg)
        find_boundary(img_open)

    print("process segmentation image done!!!")

if __name__ == '__main__':
    main()


#if __name__ == '__main__':
#    image_sub = rospy.Subscriber('/seg_processing', Image,find_boundary)
    #Odometry_sub = message_filters.Subscriber('/sim_p3at/odom', Odometry)
    #rospy.loginfo('[seg_trans]init done')
    #ts = message_filters.TimeSynchronizer([image_sub, Odometry_sub], 10)
    #ts.registerCallback(seg_process)
#    rospy.spin() 

