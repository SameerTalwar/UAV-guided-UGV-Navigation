#!/usr/bin/env python

#Import libraries

from locale import CHAR_MAX
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist,TwistStamped
from prius_msgs.msg import Control
import math
import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import time
import math
from mavros_msgs.msg import PositionTarget

#Global variables, describing car parameters

NO_COMMAND=0
NEUTRAL=1
FORWARD=2
REVERSE=3

class Controler:
    def __init__(self):
        self.cdata = Control()
        self.bridge = CvBridge()

        self.img3=np.array([])
        #Car attributes
        self.vel = 0
        self.brake = 0
        self.gear = 0
        self.steer = 0
        #control parameters
        self.distance=0
        self.orien = 0 #orientation
        self.flag=0

        ##Publishers##
        self.pub = rospy.Publisher('/prius', Control, queue_size=10)
        ##Subscribers##
        self.image_sub = rospy.Subscriber("/depth_camera/rgb/image_raw",Image,self.callback)


       
    #callback for rgb image
    def callback(self,data):
        img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.img3=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    def check(self):
       
        

        #binary thresholding
        ret,thres = cv2.threshold(self.img3,80,255,cv2.THRESH_BINARY)

        #Morphological operations
        kernal    = np.ones((6,6),np.uint8)
        threshh   = cv2.dilate(thres,kernal,iterations=2)
        kernal1   = np.ones((3,3),np.uint8)
        threshd   = cv2.erode(threshh,kernal1,iterations=2)
        threshh   = cv2.dilate(threshd,kernal,iterations=2)
        threshd   = cv2.erode(threshh,kernal1,iterations=4)
        threshh   = cv2.dilate(threshd,kernal,iterations=2)
        thresh1   = cv2.erode(threshh,kernal1,iterations=4)

        #Detect contours
        contours, hierarchy = cv2.findContours(thresh1,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        #convert to 3 channel image
        img2=cv2.cvtColor(self.img3,cv2.COLOR_GRAY2BGR)

        #points to store Center of mass Coordinates of the contours (windshields)
        Px = [0,0]
        Py = [0,0]

        #sort contours by area
        sorted_contours= sorted(contours, key=cv2.contourArea, reverse= True)

        if(len(contours)>2):

            ##front windshield contour has second largest area
            ##back windshield contour has third largest area
           
            second_largest_contour=sorted_contours[1]
            third_largest_contour=sorted_contours[2]
            
            #Display the detected contours
            cv2.drawContours(img2, second_largest_contour, -1, (0,255,0), 4)
            cv2.drawContours(img2, third_largest_contour, -1, (0,255,0), 4)

    
            #calculating Center of masses (COM) of the contours
            

            #calculation of COM of front windshield(second largest area)
            M0=cv2.moments(second_largest_contour)
            Px[0]=int(M0["m10"] / M0["m00"])
            Py[0]=int(M0["m01"] / M0["m00"])


            #calculation of COM of back windshield(third largest area)
            M1=cv2.moments(third_largest_contour)
            Px[1]=int(M1["m10"] / M1["m00"])
            Py[1]=int(M1["m01"] / M1["m00"])

        elif(len(contours)==2): #one of the windshields isn't detected.
            M0=cv2.moments(sorted_contours[1])
            ax=int(M0["m10"] / M0["m00"])
            ay=int(M0["m01"] / M0["m00"])
            if ay<240:
                Px[0],Py[0] = ax, 0
                Px[1],Py[1] = ax,ay
            else:
                Px[0],Py[0] = ax, 480
                Px[1],Py[1] = ax,ay

        #Displaying the two center of masses
        cv2.circle(img2, (Px[0], Py[0]), 3, (255, 255, 255), -1)
        cv2.circle(img2, (Px[1], Py[1]), 3, (255, 255, 255), -1)

        # cv2.arrowedLine(img2,(Px[1],Py[1]), (Px[0],Py[0]),(255,0,0),2)


        # #orientation defining arrow joining com of windshield
        # cv2.arrowedLine(img2,(Px[0],Py[0]), (Px[1],Py[1]),(255,0,0),2)
        
        # # centre of car
        Cx = int((Px[0]+Px[1])/2)
        Cy = int((Py[0]+Py[1])/2)
        cv2.circle(img2, (Cx, Cy), 3, (0, 255, 0), -1)

        #Drone position(Center of the image)
        (rows,cols)=self.img3.shape
        xc=int(0.5*cols)
        yc=int(0.5*rows)+55
        cv2.circle(img2, (xc,yc), 3, (0, 0, 255), -1)
        

        ###Car control##

        #error signal is the difference between the y coordinates of the drone and the car.
        self.distance=Cy-yc

        # ppi is index of front windshield and ppj is of back windshield
        ppi=1
        ppj=0
            
        if Py[0]<Py[1]:
            ppj=1
            ppi=0
        dy=Py[ppj]-Py[ppi]
        dx=Px[ppi]-Px[ppj]

        #Estimating orientation of the car using the line joining the COMs of the two windshields
        ang= math.pi/2-math.atan2(dy,dx)
        ang2=math.pi/2-math.atan2(Py[ppj],320-Px[ppj])
        self.orien = ang -ang2 #Orientation
        print("angle is ",self.orien)
        #Display the arrows
        cv2.arrowedLine(img2, (Px[ppj],Py[ppj]),(Px[ppi],Py[ppi]),(255,0,0),2)
        cv2.arrowedLine(img2, (Px[ppj],Py[ppj]),(320,0),(0,0,255),2)

        cv2.imshow("Image", img2)
        cv2.waitKey(20)
        # cv2.destroyAllWindows


   ##Functions for publishing the car parameters###
    def setInputData(self,v,br,st,gear):
        self.vel      = v
        self.brake    = br
        self.steer    = st
        self.gear     = gear
        self.publishData()

    def setData(self):
        self.cdata.throttle = self.vel
        self.cdata.brake    = self.brake
        self.cdata.steer    = self.steer
        self.cdata.shift_gears = self.gear

    def publishData(self):
        self.setData()
        self.pub.publish(self.cdata)
        self.displayData()

    def displayData(self):
        brak = 'FORWARD'
        if self.brake==REVERSE:
            brak = 'REVERSE'
        if self.brake==NEUTRAL:
            brak = 'NEUTRAL'
        print('Velocity =',self.vel)
        print('Gear     =',brak)
        print('Braking  =',self.brake)
        print('Steering =',self.steer)
        print()

##Car Conrol##

#We have two types of error signals: the vertical distance between the drone and the car centers,
#and the alignment of the car with the vertical.

#We can assign speed, and steering as the control signals to drive the respective errors to 0


##Gain constants for PID controller for alignment(for properly aliging the car)##
kp1 = 10
ki1 = 0.0
kd1 = 2
dt  = 0.02
I1  = 0
d1  = 0
def alignment_pid(erro):
    error = erro
    global kp1, ki1, kd1,I1,d1
    I1   = I1+error*dt
    temp = kp1*error + ki1*I1 + kd1*(error-d1)/dt
    d1   = error
    print('error angle ',error)
    return 3*temp/(kp1*math.pi/2) #To normalize the value.
##Gain constants for PID controller for speed (for minimizing the difference car and drone centers)##
kp2 = 0.3
ki2 = 0
kd2 = 0.4
I2  = 0
d2  = 0
def speed_pid(error):
    global kp2,ki2,kd2,I2,d2 #self.orien   = math.atan2(-Py[1]+Py[0],Px[0]-Px[1]) -math.pi/2
    I2   = I2+error*dt
    temp = kp2*error + ki2*I2 + kd2*(error-d2)/dt
    d2   = error
    print("throttle or brake is ",temp)

    #Normalize
    if (temp > 0):
        if (temp>kp2*190):
            return 1
        return temp/(kp2*190)
    elif (temp < 0):
        if (temp>kp2*290):
            return -1
        return 1.5*temp/(kp2*290)
    else:
        return 0

if __name__ == '__main__':
    try:
        rospy.init_node('car_Controller', anonymous=True)
        carcontrol = Controler()
        velocity = 0                      # Range 0 to 1, 1 is max throttle
        brakes   = 0                      # Range 0 to 1, 1 is max brake
        steering = 0                      # Range -1 to +1, +1 is maximum left turn
        gears    = FORWARD                # FORWARD / NEUTRAL / REVERSE

        rate = rospy.Rate(50)
        time.sleep(1)
        while not rospy.is_shutdown():
            carcontrol.check()
            print("error is ",carcontrol.distance)
            tem = speed_pid(carcontrol.distance)
            tem2= alignment_pid(carcontrol.orien)
            kk  = min(abs(tem2),1)
            if tem2!=0:
                tem2=tem2*kk/abs(tem2)
            # if abs(tem2)>0.2:
            #     tem = 0.5
            steering = tem2
            if tem<0:
                brakes  =  min(1,-2*tem)
                velocity=  0
            else:
                brakes  =  0
                velocity=  tem
            carcontrol.setInputData(velocity,brakes,steering,gears)
            rate.sleep()
        

    except rospy.ROSInterruptException:
        carcontrol.setInputData(0,1,0,gears)
        pass
