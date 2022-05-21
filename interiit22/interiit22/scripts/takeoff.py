#!/usr/bin/env python
import rospy

import math
##from hector_uav_msgs.msg import PoseActionGoal

from time import sleep
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Point,Twist
from geometry_msgs.msg import PoseStamped
from math import atan2, cos, sin
from nav_msgs.msg import *
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL


from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from mavros_msgs.srv import *
from geometry_msgs.msg import TwistStamped

import numpy as np

#global variable
latitude =0.0
longitude=0.0

takeoff = False

def setStabilizeMode():
   rospy.wait_for_service('/mavros/set_mode')
   try:
       flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
       isModeChanged = flightModeService(custom_mode='STABILIZE') #return true or false
   except rospy.ServiceException, e:
       print "service set_mode call failed: %s. GUIDED Mode could not be set. Check that GPS is enabled"%e

def setGuidedMode():
   rospy.wait_for_service('/mavros/set_mode')
   try:
       flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
       isModeChanged = flightModeService(custom_mode='GUIDED') #return true or false
   except rospy.ServiceException, e:
       print "service set_mode call failed: %s. GUIDED Mode could not be set. Check that GPS is enabled"%e

def setArm():
   print("arming starts")
   rospy.wait_for_service('/mavros/cmd/arming')
   try:
       armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
       armService(True)
   except rospy.ServiceException, e:
       print "Service arm call failed: %s"%e

def setDisarm():
   rospy.wait_for_service('/mavros/cmd/arming')
   try:
       armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
       armService(False)
   except rospy.ServiceException, e:
       print "Service arm call failed: %s"%e

def setTakeoffMode():
   rospy.wait_for_service('/mavros/cmd/takeoff')
   try:
       takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
       response=takeoffService(altitude = 20, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
       takeoff=response.success
       print("takeoff value is ",takeoff)
       if takeoff:
           rospy.signal_shutdown("takeoff completed")
           execfile("mpn.py")
           execfile("final_trishit.py")
   except rospy.ServiceException, e:
       print "Service takeoff call failed: %s"%e

def setLandMode():
   rospy.wait_for_service('/mavros/cmd/land')
   try:
       landService = rospy.ServiceProxy('/mavros/cmd/land', mavros_msgs.srv.CommandTOL)
       isLanding = landService(altitude = 0, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
   except rospy.ServiceException, e:
       print "service land call failed: %s. The vehicle cannot land "%e


if __name__ == '__main__':
    rospy.init_node('gapter_pilot_node', anonymous=True)
    try:          
        while not takeoff:
            setGuidedMode()
            setArm()
            setTakeoffMode()
            rospy.sleep(1)
        print("SUCCESS!!")
    except rospy.ROSInterruptException:
        pass
