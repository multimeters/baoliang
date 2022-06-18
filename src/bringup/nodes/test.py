#!/usr/bin/python2
# -*- coding: utf-8 -*-
#from _typeshed import NoneType
import threading
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import cv2
import rospy
from nav_msgs.msg import Odometry
import  os
import time
import tf

class vstate:

    def __init__(self):
        self.left_top=[106.6350944321,29.7692284913]
        self.left_bo=[106.6350944321,29.7673778981]
        self.right_bo=[106.6375064488,29.7673778981]
        self.right_top=[106.6375064488,29.7692284913]
        self.mymap=None
        self.rowlen=None
        self.collen=None
        self.lat_pix=None
        self.lon_pix=None
        self.lon=None
        self.lat=None

    def callback(self,msg):
        (r, p, y) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        print(r,p,y)
        
    
#draw([106.6375064488,106.6350944321],[29.7692284913,29.7692284913])
if __name__ == '__main__':
    v=vstate()
    rospy.init_node('listener', anonymous=True)
 
    rospy.Subscriber("/odom", Odometry, v.callback)
 
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
