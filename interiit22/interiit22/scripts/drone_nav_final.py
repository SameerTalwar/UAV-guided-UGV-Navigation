#!/usr/bin/env python

#import libraries

from  __future__ import print_function
import sys
import rospy
import cv2
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import time
import math
from mavros_msgs.msg import PositionTarget
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import Image, PointCloud2, PointField
from statistics import mean, median, mode, stdev


class drone:


    def __init__(self):
        rospy.init_node("CV")

        self.flag = 0
        self.bridge = CvBridge()

        #Subscribers
        self.depth_image_sub = rospy.Subscriber("/depth_camera/depth/image_raw",Image,self.callback_depth) #Depth image
        self.rgb_image_sub = rospy.Subscriber("/depth_camera/rgb/image_raw",Image,self.callback_rgb) #RGB image
        self.pc_image_sub = rospy.Subscriber("/depth_camera/depth/points",PointCloud2,self.callback_pc) #PointCloud2 data
        #Publishers
        self.yaw_ = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=1) #Drone odometry in local (ENU)frame

        #arrays for storing images obtained from ROS

        self.imageshow_depth = np.zeros((640,480), dtype = np.uint8) #depth image
        self.imageshow_mask = np.zeros((640,480), dtype = np.uint8) #masked image
        self.imageshow_rgb=np.zeros((640,480,3), dtype = np.uint8) #rgb image
        self.imageshow_mask_rgb=np.zeros((640,480,3), dtype = np.uint8)
       

        ##Masking parameters
        self.band=50
        self.slope=0
        
        ##Navigation parameters

        #Contour center of mass coordinates
        self.cX=0
        self.cY=0

        #Image center coordinates
        self.xc=0
        self.yc=0

        #Yaw control
        self.prev_yaw = 0
        self.curr_yaw = 0
        self.yaw_calc= 0.0
        #Horizontal velocity control
        self.diff = 0
        self.prev_diff = 0
        #Height control
        self.road_pixel = 0
        self.initialiser= 0
        self.height=0
        self.init_height=0
    
        #Time interval for PID control.
        self.dt = 0.01


    def callback_depth(self,data):

        cv_image = self.bridge.imgmsg_to_cv2(data, "32FC1") #Image data from ROS
        cv_image_array = np.array(cv_image, dtype = np.dtype('f8')) #Converting to CV image
        #Normalize and scale, convert to uint8.
        cv_image_norm = cv2.normalize(cv_image_array, cv_image_array, 0, 1, cv2.NORM_MINMAX)
        img= (cv_image_norm*255).astype(np.uint8)
        #depth image
        self.imageshow_depth =img

        (rows,cols)=img.shape

        #central coordinates
        self.xc=int(0.5*cols)
        self.yc=int(0.5*rows)
        
        ##Masking##

        #Average pixel intensity of the road
        self.road_pixel=img[self.yc][self.xc]

        #Calculating a "band" of intensities (centered around self.road_pixel)
        self.slope = img[self.yc-30][self.xc]-self.road_pixel
        self.band=2.5*abs(self.slope)*8
        #In case of a very large band, reduce it to some fixed value
        if(self.band>45):
            self.band=45
        else:
            self.band=max(self.band,5)
        
        #Create a mask of the image to highlight the road shape
        mask=cv2.inRange(img,self.road_pixel-self.band,self.road_pixel+self.band)
        
        #Consider a section (100 pixels long) of the upper half of the mask
        #This section will be used to estimate the drone's required trajectory
        imgU= mask[self.yc-100:self.yc,0:2*self.xc]
        self.imageshow_UH=imgU

        ###Contour detection in the section of the upper half###

        #The center of mass of the contour can be thought of as a waypoint that the drone needs to go to.

        #The angle that the line joining the drone's position, and the center of mass of the road contour,
        #makes with the vertical (trajectory of the drone in the local frame),
        #is the required yaw angle relative to the current orientation (i.e the yaw must change by this amount)



        #Accoutning for the case when no contours are detected
        im2, contours, hierarchy = cv2.findContours(imgU,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        if (len(contours) == 0):
            self.flag = 1
        else:
            self.flag = 0
        
        #If contour is detected
        if (self.flag == 0):
             # calculate x,y coordinate of center
            c=contours[0]
            M = cv2.moments(c)
            if(M["m00"]!=0):
                self.cX = int(M["m10"] / M["m00"])
                self.cY = int(M["m01"] / M["m00"]) 
                self.cY=140+self.cY
                self.yaw_calc= 1.57-math.atan2((self.yc-self.cY),(self.cX-self.xc))  
            else:
                self.yaw_calc= 0

        #If no contours detected, go straight
        else:

            self.yaw_calc = 0

        self.curr_yaw = self.yaw_calc

        #Masked image
        self.imageshow_mask = mask

        #Convert masked image into a 3 channnel image for drawing colored arrows

        mask_rgb=cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR)
        mask_rgb = cv2.arrowedLine(mask_rgb, (self.xc,self.yc),(self.cX,self.cY),(0,255,0), 2)
        mask_rgb = cv2.arrowedLine(mask_rgb, (self.xc,self.yc),(self.xc,self.yc-80),(0,0,255), 2)
        mask_rgb = cv2.circle(mask_rgb,(self.xc,self.yc),5,(0,0,0),-1)
        mask_rgb = cv2.circle(mask_rgb,(self.xc,self.yc-80),5,(0,0,0),-1)
        mask_rgb = cv2.circle(mask_rgb,(self.cX,self.cY),5,(0,0,0),-1)
        mask_rgb = cv2.arrowedLine(mask_rgb, (self.xc,self.yc),(self.xc,self.yc-80),(0,0,255), 3)
        mask_rgb = cv2.line(mask_rgb, (self.xc,self.yc),(self.cX,self.yc),(255,0,0), 2)

        #masked image with arrows
        self.imageshow_mask_rgb = mask_rgb


    ##Callbacks##
       
    def callback_rgb(self,data): #callback for RGB image
        
        img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        img = cv2.arrowedLine(img, (self.xc,self.yc),(self.cX,self.cY),(0,255,0), 2)
        img = cv2.arrowedLine(img, (self.xc,self.yc),(self.xc,self.yc-80),(0,0,255), 2)
        
        
        self.imageshow_rgb=img


    def callback_pc(self,data): #callback for pointCloud2 data

        #depth estimation using the pointCloud2 data
        cloud_array=pc2.read_points(data, field_names = ("z"), skip_nans=True)
        cloud_points=[]
        for p in cloud_array:
            cloud_points.append(p[0])
        avg = sum(cloud_points)/len(cloud_points)
        med= median(cloud_points)
       #height of the road surface from the camera
        self.height = (avg+med)/2

        #store the initial (Takeoff) height
        #Drone will try to maintian this takeoff height
        if (self.initialiser < 5):
            self.init_height = self.height
        self.initialiser += 1
            

    ##Drone Navigation and Control##

    def drone_navigation(self):

        #Publishing Vx(forward velocity) for navigation.
        #Vy(horizontal velocity), for controlling the deviation from the center
        #Yaw, for aligning it with the (estimated)road trajectory, and
        #Z, for proper height control

        yaw_obj=PositionTarget()
        yaw_obj.coordinate_frame=8 #local(ENU frame)
        yaw_obj.type_mask=3008 #Required type-mask
        
        #PD for yaw control
        #We need to account for cases with very large calculated yaws(because of errenous contours)
        
        if(abs(self.yaw_calc)<0.05 or abs(self.yaw_calc)>1.57):
            yaw_obj.yaw=0
            yaw_obj.velocity.x= 1
        elif(0.8<abs(self.yaw_calc)<1.57):
            yaw_obj.velocity.x= 0.7
            yaw_obj.yaw=0.03*self.yaw_calc + 0.4*(self.curr_yaw-self.prev_yaw)/self.dt
            yaw_obj.yaw= - yaw_obj.yaw
        else:
            yaw_obj.velocity.x= 1 
            yaw_obj.yaw=0.035*self.yaw_calc + 0.2*(self.curr_yaw-self.prev_yaw)/self.dt
            yaw_obj.yaw= - yaw_obj.yaw
        
        #P control for horizontal velocity (To control the deviation from the center)
        #error signal is difference between the x coordinates of the drone center and the contour center of mass.
        self.diff = self.xc-self.cX
        yaw_obj.velocity.y = 0.008*self.diff + 0.000*(self.diff-self.prev_diff)/self.dt

        #Height control

        #height error, to be published  to position.z for correction
        yaw_obj.position.z = 0.5*(self.init_height - self.height)
        if (abs(yaw_obj.position.z) > 1):
            yaw_obj.position.z = 0.5*(yaw_obj.position.z)/abs(yaw_obj.position.z)
        
        #publish all the attributes
        self.yaw_.publish(yaw_obj)

        #Current values = previous values for  the next iteration 
        self.prev_yaw = self.curr_yaw
        self.prev_diff = self.diff
    
    #Displaying the image windows
    def Image_shower(self):
        cv2.imshow("mask", self.imageshow_mask)
        #cv2.imshow("rgb", self.imageshow_rgb)
        cv2.imshow("depth",self.imageshow_depth )
        cv2.imshow("arrows", self.imageshow_mask_rgb)

        cv2.waitKey(1)


#Main functiom
if __name__ == '__main__':
    drone = drone()
    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        try:
            drone.drone_navigation()
            drone.Image_shower()
            r.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            pass

        