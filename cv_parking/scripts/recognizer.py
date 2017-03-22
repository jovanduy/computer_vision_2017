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
        rospy.Subscriber("/camera/image_raw", Image, self.process_image)
                
    def process_image(self, msg):
        """ Process image messages from ROS and stash them in an attribute
            called cv_image for subsequent processing """
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.hsv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
        self.binary_image = cv2.inRange(self.hsv_image, (0,70,60), (30,255,140))
        self.hough_lines()
        
    def hough_lines(self):
       """ This function uses the Hough Line Transform function to identify and visualize lines in our binary image."""
       
       lines = cv2.HoughLinesP(self.binary_image,1,np.pi/180,100,0,0)

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
