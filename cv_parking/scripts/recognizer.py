#!/usr/bin/env python

""" This script recognizes parking spots using opencv in ROS. """

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from copy import deepcopy

class ParkingSpotRecognizer(object):
    """ This robot will recognize a parking spot. """


    def __init__(self):
        """ Initialize the parking spot recognizer """
        rospy.init_node('parking_spot_recognizer')
        self.cv_image = None                        # the latest image from the camera
        self.bridge = CvBridge()                    # used to convert ROS messages to OpenCV
        cv2.namedWindow('video_window')
        cv2.namedWindow('threshold_image')
        self.hsv_lb = np.array([0, 70, 60]) # hsv lower bound 
        rospy.Subscriber("/camera/image_raw", Image, self.process_image)
        cv2.createTrackbar('H lb', 'threshold_image', 0, 255, self.set_h_lb)
        cv2.createTrackbar('S lb', 'threshold_image', 0, 255, self.set_s_lb)
        cv2.createTrackbar('V lb', 'threshold_image', 0, 255, self.set_v_lb)
        self.hsv_ub = np.array([30, 255, 140]) # hsv upper bound
        cv2.createTrackbar('H ub', 'threshold_image', 0, 255, self.set_h_ub)
        cv2.createTrackbar('S ub', 'threshold_image', 0, 255, self.set_s_ub)
        cv2.createTrackbar('V ub', 'threshold_image', 0, 255, self.set_v_ub)        

    def set_h_lb(self, val):
        """ set hue lower bound """
        self.hsv_lb[0] = val
        
    def set_s_lb(self, val):
        """ set saturation lower bound """
        self.hsv_lb[1] = val
        
    def set_v_lb(self, val):
        """ set value lower bound """
        self.hsv_lb[2] = val

    def set_h_ub(self, val):
        """ set hue upper bound """
        self.hsv_ub[0] = val

    def set_s_ub(self, val):
        """ set saturation upper bound """
        self.hsv_ub[1] = val
        
    def set_v_ub(self, val):
        """ set value upper bound """
        self.hsv_ub[2] = val
        
                
    def process_image(self, msg):
        """ Process image messages from ROS and stash them in an attribute
            called cv_image for subsequent processing """
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.hsv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
        #self.binary_image = cv2.inRange(self.hsv_image, (0,70,60), (30,255,140))
        self.binary_image = cv2.inRange(self.hsv_image, self.hsv_lb, self.hsv_ub)
        self.hough_lines()
        
               
    def hough_lines(self):
       """ This function uses the Hough Line Transform function to identify and visualize lines in our binary image."""
       
       lines = cv2.HoughLinesP(self.binary_image, rho=5, theta=np.deg2rad(10), threshold=100, minLineLength=5, maxLineGap=2)

       if lines != None:
           for x1,y1,x2,y2 in lines[0]:
               cv2.line(self.cv_image,(x1,y1),(x2,y2),(0,0,255),2)
   
    def find_endpoints(self):
        """This function uses the pinhole camera model to determine the endpoints of a parking spot in 3d space."""
        pass 
        
    
    def run(self):
        """ The main run loop"""
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if not self.cv_image is None:
                # creates a window and displays the image for X milliseconds
                cv2.imshow('video_window', self.cv_image)
                cv2.imshow('hsv_window', self.hsv_image)
                cv2.imshow('binary', self.binary_image)
                cv2.waitKey(5)
            r.sleep()

if __name__ == '__main__':
    node = ParkingSpotRecognizer()
    node.run()
